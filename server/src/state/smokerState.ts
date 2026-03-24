// server/src/state/smokerState.ts
interface NotificationLock {
  lastSeenValue: number;
  notified: boolean;
}

interface SmokerNotificationState {
  smoker: NotificationLock;
  cookTimer: NotificationLock;
  probes: { [key: number]: NotificationLock };
  stallNotified: { [key: number]: NotificationLock };
}

// In-memory state store
const smokerStates = new Map<string, SmokerNotificationState>();

// Helper to get a clean, default lock
function getDefaultLock(): NotificationLock {
  return { lastSeenValue: -1, notified: true };
}


// Helper to get or create a state for a smoker
function getSmokerState(smokerId: string): SmokerNotificationState {
  if (!smokerStates.has(smokerId)) {
    smokerStates.set(smokerId, {
      smoker: getDefaultLock(),
      cookTimer: getDefaultLock(),
      probes: {},
      stallNotified: {},
    });
  }
  return smokerStates.get(smokerId)!;
}

/**
 * Checks if the smoker's target temperature has changed.
 * If it has, it resets the notification lock for that item before returning.
 * @returns The current notification lock state for the smoker.
 */
export function getAndUpdateSmokerTemperatureNotificationLock(smokerId: string, currentSmokerTarget: number): NotificationLock {
  const state = getSmokerState(smokerId);
  const notificationLock: NotificationLock = state.smoker;

  if (currentSmokerTarget !== notificationLock.lastSeenValue) {
    notificationLock.lastSeenValue = currentSmokerTarget;
    notificationLock.notified = false;
  }
  
  return notificationLock;
}

/**
 * Checks if the probe's target temperature has changed.
 * If it has, it resets the notification lock for that item before returning.
 * @returns The current notification lock state for the probe.
 */
export function getAndUpdateProbeTemperatureNotificationLock(smokerId: string, probeIndex: number, currentProbeTarget: number): NotificationLock {
  const state = getSmokerState(smokerId);
  const notificationLock: NotificationLock = state.probes[probeIndex] || getDefaultLock();

  if (currentProbeTarget !== notificationLock.lastSeenValue) {
    // Target has changed, reset the lock
    notificationLock.lastSeenValue = currentProbeTarget;
    notificationLock.notified = false;
  }
  
  // Ensure it's stored back in the state if it was newly created
  state.probes[probeIndex] = notificationLock; 

  return notificationLock;
}

/**
 * Check if the cook timer has been reset after reaching zero.
 * Gets the previous cook timer value and updates it with the current one.
 * @returns The previous cook timer value for the given smoker.
 */
export function getAndUpdateCookTimerNotificationLock(smokerId: string, currentCookTimer: number): NotificationLock {
  const state = getSmokerState(smokerId);
  const notificationLock = state.cookTimer;
  if (notificationLock.lastSeenValue <= 0 && currentCookTimer > 0) {
    // Target has changed, reset the lock
    notificationLock.notified = false;
  }

    notificationLock.lastSeenValue = currentCookTimer;

  return notificationLock;;
}

/**
 * Checks if a stall notification has been sent for a specific probe.
 */
export function getStallNotificationLock(smokerId: string, probeIndex: number): NotificationLock {
  const state = getSmokerState(smokerId);
  const notificationLock: NotificationLock = state.stallNotified[probeIndex] || getDefaultLock();

  // Ensure it's stored back in the state if it was newly created
  state.stallNotified[probeIndex] = notificationLock; 

  return notificationLock;
}
