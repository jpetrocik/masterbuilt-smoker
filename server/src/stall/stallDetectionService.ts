// server/src/stall/stallDetectionService.ts
import { telemetryEmitter } from '../utils/eventEmitter';
import { SmokerTelemetryData, CookHistoryRow } from '../types';
import { getHistoricalData } from '../database/database';
import { getStallNotificationLock } from '../state/smokerState';

// Stall detection configuration constants
const STALL_SMOKER_TARGET_MIN_F = 190;
const STALL_PROBE_TEMP_MIN_F = 150;
const STALL_PROBE_TEMP_MAX_F = 175;
const STALL_HISTORY_WINDOW_MS = 30 * 60 * 1000;
const STALL_MIN_HISTORY_AGE_MS = 25 * 60 * 1000;
const STALL_MIN_HISTORY_POINTS = 13;
const STALL_SMA_WINDOW_MS = 5 * 60 * 1000;
const STALL_COMPARISON_OFFSET_MS = 20 * 60 * 1000;
const STALL_DELTA_THRESHOLD_F = 1.0;

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
 *    - Smoker target temperature must be > 190°F.
 *    - Probe temperature must be between 150°F and 175°F.
 * 2. Smoothing (Noise Reduction):
 *    - Calculates a Simple Moving Average (SMA) of the probe temperature over the most recent 5-minute window.
 *    - Calculates a Historical SMA for a 5-minute window exactly 20 minutes ago (T-25 mins to T-20 mins).
 * 3. Trigger Evaluation:
 *    - Stall is detected if the temperature increase (Current SMA - Historical SMA) is <= 1.0°F over that 20-minute span.
 */
function detectMeatStall(smokerId: string, probeIndex: number, probeTemp: number | undefined, smokerTarget: number): boolean {

  // 1. Gating Conditions (Pre-checks)
  if (
    probeTemp === undefined ||
    probeTemp < STALL_PROBE_TEMP_MIN_F ||
    probeTemp > STALL_PROBE_TEMP_MAX_F ||
    smokerTarget <= STALL_SMOKER_TARGET_MIN_F
  ) {
    return false;
  }

  const now = Date.now();
  const thirtyMinutesAgo = now - STALL_HISTORY_WINDOW_MS;

  // Fetch historical data from SQLite
  const historicalData = getHistoricalData(smokerId, thirtyMinutesAgo);

  const twentyFiveMinutesAgo = now - STALL_MIN_HISTORY_AGE_MS;

  // We need at least 13 historical records for reliable stall detection,
  // and the oldest record must be at least 25 minutes old.
  if (historicalData.length < STALL_MIN_HISTORY_POINTS || historicalData[0].timestamp > twentyFiveMinutesAgo) {
    return false; // Insufficient history for reliable stall detection.
  }

  const totalDuration = historicalData[historicalData.length - 1].timestamp - historicalData[0].timestamp;
  if (totalDuration < twentyFiveMinutesAgo) {
    return false; // Not large enough window to calculate stall
  }

  const totalIntervals = historicalData.length - 1; // historicalData.length is guaranteed to be >= 13 here

  // Calculate the telemetry interval and ensure a reasonable minimum interval (e.g., 1 second) to prevent issues with extremely small or zero intervals
  let actualTelemetryIntervalMillis = totalDuration / totalIntervals;
  if (actualTelemetryIntervalMillis < 1000) {
    actualTelemetryIntervalMillis = 1000;
  }

  const expectedPointsPerMinute = (60 * 1000) / actualTelemetryIntervalMillis;
  const minRequiredPointsFor5MinSMA = Math.ceil(expectedPointsPerMinute * 5 * 0.5); // Require at least 50% of expected points for a 5-min window

  // 2. Smoothing (Noise Reduction) - Helper to calculate SMA for a given window
  const calculateSMA = (startTime: number, endTime: number): number | null => {
    const windowData = historicalData.filter(row => row.timestamp >= startTime && row.timestamp <= endTime);

    // We need a minimum number of data points to consider the SMA valid.
    // This minimum is now dynamically calculated based on the observed telemetry interval.
    if (windowData.length < minRequiredPointsFor5MinSMA) {
      return null;
    }

    let sum = 0;
    let validCount = 0;
    const probeKey = `probe${probeIndex}Temperature` as keyof CookHistoryRow;
    for (const row of windowData) {
      const temp = row[probeKey] as number | null;
      if (temp !== null && temp !== undefined) {
        sum += temp;
        validCount++;
      }
    }

    if (validCount === 0)
      return null;
    return sum / validCount;
  };

  // Calculate Current SMA (most recent 5-minute window)
  const currentSMAStartTime = now - STALL_SMA_WINDOW_MS;
  const currentSMA = calculateSMA(currentSMAStartTime, now);

  // Calculate Historical SMA (5-minute window from 25 mins ago to 20 mins ago)
  const historicalSMAStartTime = now - STALL_MIN_HISTORY_AGE_MS;
  const historicalSMAEndTime = now - STALL_COMPARISON_OFFSET_MS;
  const historicalSMA = calculateSMA(historicalSMAStartTime, historicalSMAEndTime);

  // If either SMA calculation failed due to insufficient data, we can't evaluate the stall.
  if (currentSMA === null || historicalSMA === null) {
    return false;
  }

  // 3. The Trigger Evaluation
  const delta = currentSMA - historicalSMA;

  // If the temperature increase is <= 1.0°F over that 20-minute span, the meat has stalled.
  return delta <= STALL_DELTA_THRESHOLD_F;
}
