export interface SmokerState {
  id: string;
  status: 'online' | 'offline';
  lastSeen: number;
}

export interface SmokerTelemetryPayload {
  smokerTemperature: number;
  smokerTarget: number;
  cookTimer: number;
  cookTime?: number;
  dutyCycle?: number;
  probe1Temperature?: number;
  probe1Target?: number;
  probe1Alarm?: boolean;
  probe2Temperature?: number;
  probe2Target?: number;
  probe2Alarm?: boolean;
  probe3Temperature?: number;
  probe3Target?: number;
  probe3Alarm?: boolean;
  probe4Temperature?: number;
  probe4Target?: number;
  probe4Alarm?: boolean;
}

export interface SmokerTelemetryData {
  smokerId: string;
  timestamp: number;
  smokerTemperature: number;
  smokerTarget: number;
  cookTimer: number;
  cookTime?: number;
  dutyCycle?: number;
  probe1Temperature?: number;
  probe1Target?: number;
  probe1Alarm?: boolean;
  probe2Temperature?: number;
  probe2Target?: number;
  probe2Alarm?: boolean;
  probe3Temperature?: number;
  probe3Target?: number;
  probe3Alarm?: boolean;
  probe4Temperature?: number;
  probe4Target?: number;
  probe4Alarm?: boolean;
}

export interface CookHistoryRow extends SmokerTelemetryData {
  id: number;
}


