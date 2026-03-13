// server/src/state/smokerState.ts
interface NotificationLock {
  lastSeenTarget: number;
  notified: boolean;
}

interface SmokerNotificationState {
  smoker: NotificationLock;
  probes: { [key: number]: NotificationLock };
}

// In-memory state store
const smokerStates = new Map<string, SmokerNotificationState>();

// Helper to get a clean, default lock
function getDefaultLock(): NotificationLock {
  return { lastSeenTarget: -1, notified: false };
}

// Helper to get or create a state for a smoker
function getSmokerState(smokerId: string): SmokerNotificationState {
  if (!smokerStates.has(smokerId)) {
    smokerStates.set(smokerId, {
      smoker: getDefaultLock(),
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
export function checkAndReset(smokerId: string, type: 'smoker' | 'probe', currentTarget: number, probeIndex?: number): NotificationLock {
  const state = getSmokerState(smokerId);
  const item: NotificationLock = (type === 'smoker')
    ? state.smoker
    : state.probes[probeIndex!] || getDefaultLock();

  if (currentTarget !== item.lastSeenTarget) {
    // Target has changed, reset the lock
    item.lastSeenTarget = currentTarget;
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
