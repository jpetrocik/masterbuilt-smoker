# Brisket "Percent Done" / Tenderness Calculation Service

## Overview

Brisket tenderness is achieved through rendering collagen into gelatin. This rendering process is non-linear and accelerates exponentially at higher internal temperatures. This service calculates cumulative collagen rendering as a percentage (0-100%+) based on historical probe1 temperature telemetry from the smoker.

## Context

- Brisket cooks can run 12-18+ hours
- The smoker records probe telemetry every ~5 seconds, yielding massive datasets
- A direct calculation on raw 5-second data is expensive; aggregation to 1-minute intervals is more practical
- The calculation is slow enough that running it on every telemetry update (5-second intervals) is wasteful
- The metric is inherently slow-moving and doesn't require real-time precision

## Rendering Rate Model

The "Rendering Rate" (percentage of collagen rendered per hour) is determined by internal probe temperature, mapped via a lookup table. These rates represent the collagen-rendering speed at each temperature band.

| Temperature Range | Rate (% per hour) |
|---|---|
| < 140°F | 0% |
| 140–149°F | 1% |
| 150–159°F | 2% |
| 160–169°F | 3% |
| 170–179°F | 5% |
| 180–189°F | 9% |
| 190–194°F | 18% |
| 195–199°F | 25% |
| 200–204°F | 35% |
| 205–209°F | 55% |
| ≥ 210°F | 75% |

## Design Decisions

### 1. Where to Store the Calculated Value
- **Choice**: Include `percentRendered?: number` as an optional field in `SmokerTelemetryData`
- **Why**: The field is transient, computed, and specific to the WebSocket consumer. It doesn't belong in the persistent database record.
- **Implementation**: `insertTelemetry()` uses explicit column bindings, so adding the field to the interface has no DB side effects.

### 2. Calculation Frequency
- **Choice**: Run every 5 minutes via `setInterval`
- **Why**: Avoids expensive DB aggregation on every 5-second telemetry pulse. 5-minute staleness is acceptable for a slow-moving cook metric.
- **Alternative considered**: Lazy calculation with per-smoker 5-minute debounce (would add complexity; interval-based is simpler).

### 3. Active Smoker Tracking
- **Choice**: Track smokers via a `Set<string>` that gets populated on every telemetry event and cleared after each calculation tick
- **Why**: The scheduler doesn't need to query the database to find "active" smokers; they're implicitly the ones sending telemetry.
- **Scalability**: If there are many offline smokers, this avoids unnecessary calculations.

### 4. Cook Start Time
- **Choice**: Use `MIN(timestamp)` from `cook_history` for the smoker
- **Why**: Simple, authoritative, and doesn't require external tracking or configuration.
- **Limitation**: If history retention (`maxHistoryHours`) is shorter than the cook duration, the true cook start is lost. The calculation will be undercount. (Future enhancement: allow explicit cook start time tracking.)

### 5. Injection Point
- **Choice**: Attach `percentRendered` in `mqttService.ts:handleStatusMessage` before emitting to `telemetryEmitter`
- **Why**: Centralizes the enrichment logic at the source, and the WebSocket server receives already-augmented data.
- **Consideration**: The raw telemetry stored in `cook_history` never has `percentRendered` — it's a computed, transient field.

## Implementation

### Type Changes
**`server/src/types/index.ts`**
- Add `percentRendered?: number` to `SmokerTelemetryData` interface

### Database Functions
**`server/src/database/database.ts`**

Two new functions:

```typescript
export function getCookStartTime(smokerId: string): number | null
```
Returns the minimum timestamp from `cook_history` for the given `smokerId`, or `null` if no records exist.

```typescript
export function getMinuteAveragedProbeData(
  smokerId: string,
  startTime: number,
  endTime: number
): Array<{ minuteBucket: number; avgProbe1Temperature: number }>
```
Aggregates telemetry into 1-minute buckets and returns average probe1Temperature per bucket. Uses `timestamp / 60000` for bucketing (integer division in SQLite). Filters for `probe1Temperature IS NOT NULL`.

### Brisket Service
**`server/src/brisket/brisketService.ts`** (NEW file)

Exports:
- `startBrisketScheduler()` — initializes the 5-minute interval. Called once at server startup.
- `markSmokerActive(smokerId: string)` — records that a smoker has sent telemetry. Called on every MQTT status message.
- `getCachedPercentRendered(smokerId: string)` — returns the cached value (may be `undefined` if not yet computed). Called before emitting telemetry.

Internals:
- In-memory `Map<string, number>` for the cache (keyed by smokerId)
- In-memory `Set<string>` for active smokers
- `getRenderingRate(tempF: number): number` — switch/if-chain lookup
- `computeAndCache(smokerId: string)` — executes the full calculation pipeline

### MQTT Service Integration
**`server/src/mqtt/mqttService.ts`**

Two small changes:

1. In `connectMqtt()`, call `startBrisketScheduler()` alongside the existing `purgeOldHistory` interval.
2. In `handleStatusMessage()`, before emitting:
   - Call `markSmokerActive(smokerId)`
   - Retrieve `getCachedPercentRendered(smokerId)`
   - If defined, attach it to the outbound telemetry object before emitting

---

## Data Flow

```
Smoker sends MQTT status (every ~5 seconds)
  ↓
MQTT topic parsed → smokerId extracted
  ↓
handleStatusMessage(smokerId, payload)
  ├─ markSmokerActive(smokerId)
  ├─ insertTelemetry(timestampedData)                    [raw data, no percentRendered]
  ├─ percentRendered = getCachedPercentRendered(smokerId) [may be undefined]
  └─ telemetryEmitter.emit('telemetry', { ...data, percentRendered? })
       ↓
       WebSocket server broadcasts { type: 'live', data: enrichedTelemetry }
       ↓
       Frontend receives percentRendered (if defined)

Every 5 minutes (setInterval)
  ↓
brisketScheduler tick
  ├─ for each smokerId in activeSmokers:
  │   ├─ startTime = getCookStartTime(smokerId)
  │   ├─ buckets = getMinuteAveragedProbeData(smokerId, startTime, now)
  │   ├─ total = Σ(getRenderingRate(bucket.temp) / 60)
  │   └─ cache.set(smokerId, round(total, 2 decimals))
  └─ activeSmokers.clear()
```

---

## Edge Cases & Mitigations

### 1. First 5 Minutes of Uptime
- **Case**: Server starts but scheduler hasn't fired yet; telemetry arrives with `percentRendered: undefined`
- **Mitigation**: Frontend should handle missing field gracefully (display "—" or omit from UI)
- **Is this acceptable?** Yes for this project. If strict: could pre-compute for all smokers on startup.

### 2. Probe Unplugged / No Temperature Data
- **Case**: All `probe1Temperature` values are NULL for a smoker
- **Database query**: `getMinuteAveragedProbeData` returns empty array
- **Result**: `percentRendered` caches as `0.00`
- **Is this acceptable?** Yes. A missing probe means no rendering data available; zero is sensible.

### 3. Cook Longer Than History Retention Window
- **Case**: Cook runs 18 hours, but `maxHistoryHours` is 12
- **Impact**: `getCookStartTime` returns the oldest retained record, not the true cook start
- **Result**: Rendering percentage is underestimated
- **Mitigation**: Document the limitation. Future enhancement: track explicit cook start time in a separate table or in-memory map.

### 4. Multiple Concurrent Calculations for Same Smoker
- **Case**: If manual refresh or ad-hoc endpoint also triggers calculation
- **Impact**: Race condition on cache write (minor; last-write wins)
- **Mitigation**: Current design is simple and acceptable for single-server deployment. For HA, consider a lock or atomic update.

### 5. Stale Cache Across Server Restarts
- **Case**: Server restarts; cache is in-memory and lost
- **Impact**: First telemetry after restart has `percentRendered: undefined` until next 5-min tick
- **Mitigation**: On startup, could populate cache by running calculation for all smokers in DB. Not done currently to keep startup fast.

### 6. Smoker Inactive for > 5 Minutes
- **Case**: Telemetry stops arriving; `activeSmokers` set becomes empty
- **Impact**: Calculation no longer runs; cached value freezes at last update
- **Is this acceptable?** Yes. A frozen value for an inactive cook is correct. When telemetry resumes, the next tick will update it.

---

## Performance Considerations

- **Database query**: Compound index on `(smokerId, timestamp)` ensures the aggregation query is efficient even over 18 hours of data (≈65k 5-sec records = 1080 1-min buckets per cook).
- **Calculation cost**: One aggregation query + 1080 loop iterations (max) every 5 minutes. Negligible on a single-smoker or low-smoker workload.
- **Memory**: Cache is `Map<string, number>` (~64 bytes per smoker per entry). For 100 smokers, ~6.4 KB. Negligible.

---

## Testing / Verification

### Unit / Integration Test Ideas
1. Mock a sequence of temperature readings in different rate bands, verify the aggregation and sum.
2. Test `getRenderingRate()` edge cases (139.9°F, 140.0°F, 209.9°F, 210.0°F).
3. Test empty probe data returns 0%.

### Manual Verification
1. Start server + simulator; let it run 5+ minutes.
2. Open WebSocket connection; observe incoming `live` messages.
3. After first scheduler tick, `percentRendered` should appear in the data.
4. Manually run the aggregation SQL on the SQLite DB; compute a few buckets by hand.
5. Verify the cached value matches the manual calculation.

### Acceptance Criteria
- [ ] `percentRendered` appears in WebSocket `live` messages after first 5-min tick
- [ ] Value is a float rounded to 2 decimal places
- [ ] Value increases monotonically (or stays flat if probe temp doesn't rise)
- [ ] Value matches manual calculation from database

---

## Future Enhancements

1. **Explicit Cook Start Tracking**: Store cook start time in a separate table or in `smokers` table. Allow frontend to reset the clock mid-cook.
2. **Different Probes**: Generalize from `probe1` to any probe (most cooks use probe1, but some may use probe2 for brisket).
3. **Pre-Computed History**: Store daily snapshots of `percentRendered` for completed cooks (analytics / archive).
4. **Confidence Intervals**: If gaps exist in telemetry, note that the calculation is based on partial data.
5. **Temperature-Specific Alerts**: "Rendering accelerating — collagen breakdown in progress" at 190°F+.
6. **Target Rendering Level**: Allow user to set a target (e.g., 85%) and alert when reached, rather than just displaying a number.
