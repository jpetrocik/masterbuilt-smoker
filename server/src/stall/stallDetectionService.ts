// server/src/stall/stallDetectionService.ts
import { telemetryEmitter } from '../utils/eventEmitter';
import { SmokerTelemetryData, CookHistoryRow } from '../types';
import { getHistoricalData } from '../database/database';
import { getStallNotificationLock } from '../state/smokerState';

// Stall detection configuration constants
const STALL_SMOKER_TARGET_MIN_F = 170;
const STALL_PROBE_TEMP_MIN_F = 140;
const STALL_HISTORY_WINDOW_MS = 15 * 60 * 1000;
const STALL_MIN_HISTORY_AGE_MS = 15 * 60 * 1000;
const STALL_MIN_HISTORY_POINTS = 13;
const STALL_SMA_WINDOW_MS = 2 * 60 * 1000;  // 2-minute window for each SMA snapshot
const STALL_DELTA_THRESHOLD_F = 2.0;

export function initStallDetectionService(): void {
  telemetryEmitter.on('telemetry', (data: SmokerTelemetryData) => {
    for (let i = 1; i <= 4; i++) {
      const probeTemp = data[`probe${i}Temperature` as keyof SmokerTelemetryData] as number | undefined;
      if (probeTemp === undefined) continue;

      const lock = getStallNotificationLock(data.smokerId, i);
      if (!lock.notified && detectMeatStall(data.smokerId, i, probeTemp, data.smokerTarget)) {
        lock.notified = true;
        telemetryEmitter.emit('stallDetected', data.smokerId, i, probeTemp);
      }
    }
  });
}

/**
 * Detects if a meat stall is occurring for a specific probe.
 *
 * Algorithm Logic:
 * 1. Gating Conditions:
 *    - Smoker target temperature must be >= 170°F.
 *    - Probe temperature must be >= 140°F.
 * 2. Smoothing (Noise Reduction):
 *    - Calculates SMA of probe temperature over a 2-minute recent window (now to now-2 min).
 *    - Calculates SMA of probe temperature over a 2-minute historical window (now-13 to now-15 min).
 * 3. Trigger Evaluation:
 *    - Stall is detected if temperature change is <= 2.0°F over the 11-minute span between windows.
 *    - This indicates the probe has stopped warming during active cooking.
 */
function detectMeatStall(smokerId: string, probeIndex: number, probeTemp: number | undefined, smokerTarget: number): boolean {

  // 1. Gating Conditions (Pre-checks)
  if (
    probeTemp === undefined ||
    probeTemp < STALL_PROBE_TEMP_MIN_F ||
    smokerTarget < STALL_SMOKER_TARGET_MIN_F
  ) {
    return false;
  }

  const now = Date.now();
  const stallHistoryStartTime = now - STALL_HISTORY_WINDOW_MS;

  // Fetch historical data from SQLite
  const historicalData = getHistoricalData(smokerId, stallHistoryStartTime);

  // Split historical data into 2 data sets for SMA calculation:
  const recentWindowStart = now - STALL_SMA_WINDOW_MS;
  const historicalWindowStart = now - STALL_MIN_HISTORY_AGE_MS;
  const historicalWindowEnd = historicalWindowStart + STALL_SMA_WINDOW_MS;

  const recentWindowData = historicalData.filter(row => row.timestamp >= recentWindowStart && row.timestamp <= now);
  const historicalWindowData = historicalData.filter(row => row.timestamp >= historicalWindowStart && row.timestamp <= historicalWindowEnd);

  // Calculate SMA for both data sets
  const probeKey = `probe${probeIndex}Temperature` as keyof CookHistoryRow;

  const calculateSMA = (data: CookHistoryRow[]): number | null => {
    const filteredData = data
      .map(row => row[probeKey] as number | null)
      .filter((temp): temp is number => temp !== null && temp !== undefined);

    // Ensure both data sets have enough points (>= 13) to calculate SMA
    if (filteredData.length < STALL_MIN_HISTORY_POINTS) {
      return null;
    }

    return filteredData.reduce((sum, temp) => sum + temp, 0) / filteredData.length;
  };

  const recentSMATemps = calculateSMA(recentWindowData);
  const historicalSMATemps = calculateSMA(historicalWindowData);

  // If either window has insufficient valid readings, cannot calculate reliable SMA
  if (recentSMATemps === null || historicalSMATemps === null) {
    return false;
  }

  // Compare the two values to determine if a stall is occurring
  const delta = recentSMATemps - historicalSMATemps;

  // If the temperature increase is <= 2.0°F over that 10-minute span, the meat has stalled.
  return delta <= STALL_DELTA_THRESHOLD_F;
}
