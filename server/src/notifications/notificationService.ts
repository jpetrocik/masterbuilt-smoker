// server/src/notifications/notificationService.ts
import { telemetryEmitter } from '../utils/eventEmitter';
import { SmokerTelemetryData } from '../types';
import { getTokensForSmoker } from '../database/database';
import { messaging } from '../firebase/firebaseAdmin';
import { getAndUpdateCookTimerNotificationLock, getAndUpdateSmokerTemperatureNotificationLock, getAndUpdateProbeTemperatureNotificationLock, getAndUpdatePresenceLock } from '../state/smokerState';

export function initNotificationService(): void {
  telemetryEmitter.on('telemetry', (smokerTelemetryData: SmokerTelemetryData) => {
    evaluateAndSend(smokerTelemetryData);
  });

  // Add listener for presence changes
  telemetryEmitter.on('presenceChange', (smokerId: string, status: 'online' | 'offline') => {
    handlePresenceChangeNotification(smokerId, status);
  });

  // Add listener for stall detection events
  telemetryEmitter.on('stallDetected', (smokerId: string, probeIndex: number, probeTemp: number) => {
    const tokens = getTokensForSmoker(smokerId);
    if (tokens.length > 0) {
      sendNotification(tokens, 'Meat Stall Detected!', `Probe ${probeIndex} has entered the stall at ${probeTemp}°F.`);
    }
  });

  // Add listener for generic notification events from any service
  telemetryEmitter.on('sendNotification', (smokerId: string, title: string, message: string) => {
    const tokens = getTokensForSmoker(smokerId);
    if (tokens.length > 0) {
      sendNotification(tokens, title, message);
    }
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

  //Checking probe temperatures for each probe
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

async function handlePresenceChangeNotification(smokerId: string, status: 'online' | 'offline'): Promise<void> {
  const lock = getAndUpdatePresenceLock(smokerId, status);

  if (status === 'offline' && !lock.notified) {
    const tokens = getTokensForSmoker(smokerId);
    if (tokens.length > 0) {
      console.log(`Starting 5-minute offline timer for smoker ${smokerId}`);
      lock.timerId = setTimeout(() => {
        console.log(`Smoker ${smokerId} is still offline after 5 minutes. Sending notification.`);
        sendNotification(tokens, 'Smoker Offline!', `Your smoker has gone offline.`);
        lock.timerId = undefined;
      }, 5 * 60 * 1000);
      lock.notified = true; // Prevents creating multiple timers
    }
  } else if (status === 'online') {
    if (lock.timerId) {
      console.log(`Smoker ${smokerId} came back online. Cancelling offline notification.`);
      clearTimeout(lock.timerId);
      lock.timerId = undefined;
    }
  }
}
