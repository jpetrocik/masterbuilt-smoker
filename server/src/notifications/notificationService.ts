// server/src/notifications/notificationService.ts
import { telemetryEmitter } from '../utils/eventEmitter';
import { SmokerTelemetryPayload, SmokerTelemetryData } from '../types';
import { getTokensForSmoker } from '../database/database';
import { messaging } from '../firebase/firebaseAdmin';
import { checkAndReset, recordNotification } from '../state/smokerState';

export function initNotificationService(): void {
  telemetryEmitter.on('telemetry', (smokerTelemetryData: SmokerTelemetryData) => {
    evaluateAndSend(smokerTelemetryData);
  });
}

function evaluateAndSend(smokerTelemetryData: SmokerTelemetryData): void {
  const { smokerId, smokerTemperature, smokerTarget } = smokerTelemetryData;

  // --- Check Smoker Temperature ---
  if (smokerTarget > 0) {
    const lock = checkAndReset(smokerId, 'smoker', smokerTarget);
    if (!lock.notified && smokerTemperature >= smokerTarget) {
      const tokens = getTokensForSmoker(smokerId);
      if (tokens.length > 0) {
        sendNotification(tokens, 'Smoker Ready!', `Smoker has reached ${smokerTarget}°F.`);
        recordNotification(smokerId, 'smoker');
      }
    }
  }

  // --- Check Probe Temperatures ---
  for (let i = 1; i <= 4; i++) {
    const probeTemp = smokerTelemetryData[`probe${i}Temperature` as keyof SmokerTelemetryPayload] as number | undefined;
    const probeTarget = smokerTelemetryData[`probe${i}Target` as keyof SmokerTelemetryPayload] as number | undefined;

    if (probeTarget !== undefined && probeTarget > 0 && probeTemp !== undefined) {
      const lock = checkAndReset(smokerId, 'probe', probeTarget, i);
      if (!lock.notified && probeTemp >= probeTarget) {
        const tokens = getTokensForSmoker(smokerId);
        if (tokens.length > 0) {
          sendNotification(tokens, 'Meat Ready!', `Probe ${i} has reached ${probeTarget}°F.`);
          recordNotification(smokerId, 'probe', i);
        }
      }
    }
  }
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
