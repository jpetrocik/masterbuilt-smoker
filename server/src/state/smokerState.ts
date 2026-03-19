// server/src/state/smokerState.ts
interface NotificationLock {
  lastSeenValue: number;
  notified: boolean;
}

interface SmokerNotificationState {
  smoker: NotificationLock;
  lastSeenCookTimer: number;
  probes: { [key: number]: NotificationLock };
}

// In-memory state store
const smokerStates = new Map<string, SmokerNotificationState>();

// Helper to get a clean, default lock
function getDefaultLock(): NotificationLock {
  return { lastSeenValue: -1, notified: false };
}


// Helper to get or create a state for a smoker
function getSmokerState(smokerId: string): SmokerNotificationState {
  if (!smokerStates.has(smokerId)) {
    smokerStates.set(smokerId, {
      smoker: getDefaultLock(),
      lastSeenCookTimer: 0,
      probes: {},
    });
  }
  return smokerStates.get(smokerId)!;
}

/**
 * Checks if the target has changed for a given item (smoker or probe).
 * If it has, it resets the notification lock for that item.
 * @returns The current notification lock state for the item.
 */
export function checkAndReset(smokerId: string, type: 'smoker' | 'probe', currentValue: number, probeIndex?: number): NotificationLock {
  const state = getSmokerState(smokerId);
  const item: NotificationLock = (type === 'smoker')
    ? state.smoker
    : state.probes[probeIndex!] || getDefaultLock();

  if (currentValue !== item.lastSeenValue) {
    // Target has changed, reset the lock
    item.lastSeenValue = currentValue;
    item.notified = false;
  }
  
  if (type === 'probe' && probeIndex) {
    state.probes[probeIndex] = item;
  }

  return item;
}

/**
 * Marks a specific item as having been notified.
 */
export function recordNotification(smokerId: string, type: 'smoker' | 'probe', probeIndex?: number): void {
  const state = getSmokerState(smokerId);
  const item = (type === 'smoker')
    ? state.smoker
    : state.probes[probeIndex!];

  if (item) {
    item.notified = true;
  }
}

/**
 * Gets the previous cook timer value and updates it with the current one.
 * @returns The previous cook timer value for the given smoker.
 */
export function getAndUpdateCookTimer(smokerId: string, currentCookTimer: number): number {
  const state = getSmokerState(smokerId);
  const previousCookTimer = state.lastSeenCookTimer;
  state.lastSeenCookTimer = currentCookTimer;
  return previousCookTimer;
}
