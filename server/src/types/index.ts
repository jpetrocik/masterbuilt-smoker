export interface SmokerTelemetryPayload {
  temperature: number;
  targetTemperature: number;
  cookTimer: number;
  probe1?: number;
  targetProbe1?: number;
  probe2?: number;
  targetProbe2?: number;
  probe3?: number;
  targetProbe3?: number;
  probe4?: number;
  targetProbe4?: number;
}

export interface SmokerState {
  id: string;
  status: 'online' | 'offline';
  lastSeen: number;
}

export interface CookHistoryRow {
  id: number;
  smokerId: string;
  timestamp: number;
  temperature: number;
  targetTemperature: number;
  cookTimer: number;
  probe1?: number;
  targetProbe1?: number;
  probe2?: number;
  targetProbe2?: number;
  probe3?: number;
  targetProbe3?: number;
  probe4?: number;
  targetProbe4?: number;
}