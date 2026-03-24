// server/src/notifications/notificationService.ts
import { telemetryEmitter } from '../utils/eventEmitter';
import { SmokerTelemetryPayload, SmokerTelemetryData, CookHistoryRow } from '../types';
import { getTokensForSmoker, getHistoricalData } from '../database/database';
import { messaging } from '../firebase/firebaseAdmin';
import { getAndUpdateCookTimerNotificationLock, getAndUpdateSmokerTemperatureNotificationLock, getAndUpdateProbeTemperatureNotificationLock, getStallNotificationLock } from '../state/smokerState';
import config from '../config';

export function initNotificationService(): void {
  telemetryEmitter.on('telemetry', (smokerTelemetryData: SmokerTelemetryData) => {
    evaluateAndSend(smokerTelemetryData);
  });
}

function evaluateAndSend(smokerTelemetryData: SmokerTelemetryData): void {
  const { smokerId, smokerTemperature, smokerTarget, cookTimer } = smokerTelemetryData;

  // --- Check Smoker Temperature ---
  if (smokerTarget > 0) {
    const lock = getAndUpdateSmokerTemperatureNotificationLock(smokerId, smokerTarget);
    if (!lock.notified && smokerTemperature >= smokerTarget) {
      const tokens = getTokensForSmoker(smokerId);
      if (tokens.length > 0) {
        sendNotification(tokens, 'Smoker Ready!', `Smoker has reached ${smokerTarget}°F.`);
        lock.notified = true; // Mark as notified to prevent duplicates until target changes
      }
    }
  }

  //Checking probe temperatures and meat stall detection for each probe
  for (let i = 1; i <= 4; i++) {
    const probeTarget = smokerTelemetryData[`probe${i}Target` as keyof SmokerTelemetryData] as number | undefined;
    const probeTemp = smokerTelemetryData[`probe${i}Temperature` as keyof SmokerTelemetryData] as number | undefined;

    // --- Check Probe Temperatures ---
    if (probeTarget !== undefined && probeTarget > 0 && probeTemp !== undefined) {
      const lock = getAndUpdateProbeTemperatureNotificationLock(smokerId, i, probeTarget);
      if (!lock.notified && probeTemp >= probeTarget) {
        const tokens = getTokensForSmoker(smokerId);
        if (tokens.length > 0) {
          sendNotification(tokens, 'Meat Ready!', `Probe ${i} has reached ${probeTarget}°F.`);
          lock.notified = true; // Mark as notified to prevent duplicates until target changes
        }
      }
    }

    // --- Check for Meat Stall ---
    if (probeTemp !== undefined) {
      const lock = getStallNotificationLock(smokerId, i);
      if (!lock.notified && detectMeatStall(smokerId, i, probeTemp, smokerTarget)) {
        const tokens = getTokensForSmoker(smokerId);
        if (tokens.length > 0) {
          sendNotification(tokens, 'Meat Stall Detected!', `Probe ${i} has entered the stall at ${probeTemp}°F.`);
          lock.notified = true; // Mark as notified to prevent duplicates until stall condition clears
        }
      }
    }
  }

  // --- Check Cook Timer ---
  if (cookTimer !== undefined) {
    const lock = getAndUpdateCookTimerNotificationLock(smokerId, cookTimer);
    // Check if the timer just hit zero (the "zero-crossing" event)
    if (!lock.notified && cookTimer === 0) {
      const tokens = getTokensForSmoker(smokerId);
      if (tokens.length > 0) {
        sendNotification(tokens, 'Cook Finished!', 'Your cook timer has reached zero.');
        lock.notified = true; // Mark as notified to prevent duplicates until timer is reset
      }
    }
  }
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
    probeTemp < 150 ||
    probeTemp > 175 ||
    smokerTarget <= 190
  ) {
    return false;
  }

  const now = Date.now();
  const thirtyMinutesAgo = now - 30 * 60 * 1000;

  // Fetch historical data from SQLite
  const historicalData = getHistoricalData(smokerId, thirtyMinutesAgo);

  // Assuming telemetry arrives roughly every 5 seconds, 25 minutes of data should be around 300 points.
  // The Historical SMA needs data from 25 to 20 minutes ago.
  // If we don't have enough data history to even reach 25 minutes back, we can't reliably calculate the Historical SMA.
  const twentyFiveMinutesAgo = now - 25 * 60 * 1000;
  if (historicalData.length === 0 || historicalData[0].timestamp > twentyFiveMinutesAgo) {
    return false; // Insufficient history
  }

  // 2. Smoothing (Noise Reduction) - Helper to calculate SMA for a given window
  const calculateSMA = (startTime: number, endTime: number): number | null => {
    const windowData = historicalData.filter(row => row.timestamp >= startTime && row.timestamp <= endTime);

    // We need a minimum number of data points to consider the SMA valid.
    if (windowData.length < 5) {
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
  const currentSMAStartTime = now - 5 * 60 * 1000;
  const currentSMA = calculateSMA(currentSMAStartTime, now);

  // Calculate Historical SMA (5-minute window from 25 mins ago to 20 mins ago)
  const historicalSMAStartTime = now - 25 * 60 * 1000;
  const historicalSMAEndTime = now - 20 * 60 * 1000;
  const historicalSMA = calculateSMA(historicalSMAStartTime, historicalSMAEndTime);

  // If either SMA calculation failed due to insufficient data, we can't evaluate the stall.
  if (currentSMA === null || historicalSMA === null) {
    return false;
  }

  // 3. The Trigger Evaluation
  const delta = currentSMA - historicalSMA;

  // If the temperature increase is <= 1.0°F over that 20-minute span, the meat has stalled.
  return delta <= 1.0;
}

async function sendNotification(tokens: string[], title: string, body: string): Promise<void> {
  if (tokens.length === 0) {
    return;
  }

  const message = {
    tokens: tokens,
    data: {
      title: title,
      body: body,
      type: 'temperature_alert' // custom flag for the frontend
    }
  };

  try {
    const response = await messaging.sendEachForMulticast(message);
    console.log(`[FCM] Successfully sent message to ${response.successCount} tokens.`);
    if (response.failureCount > 0) {
      response.responses.forEach(resp => {
        if (!resp.success) {
          console.error(`[FCM] Failed to send notification: ${resp.error}`);
          // TODO: Add logic to remove invalid/unregistered tokens from the database
        }
      });
    }
  } catch (error) {
    console.error('[FCM] Error sending notification:', error);
  }
}
