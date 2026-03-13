// API Configuration
const API_BASE = import.meta.env.VITE_API_BASE || 'http://localhost:3000';

export interface Smoker {
  id: string;
  status: 'online' | 'offline';
  lastSeen: number;
}

export interface TelemetryData {
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
  timestamp?: number;
}

export interface WebSocketMessage {
  type: 'historical' | 'live';
  data: TelemetryData | TelemetryData[];
}

// Get list of online smokers
export async function getSmokers(status: 'online' | 'offline' | 'all' = 'online'): Promise<Smoker[]> {
  const response = await fetch(`${API_BASE}/api/smokers?status=${status}`);
  if (!response.ok) {
    throw new Error(`Failed to fetch smokers: ${response.statusText}`);
  }
  return response.json();
}

// Register FCM token for a smoker
export async function registerFcmToken(smokerId: string, fcmToken: string): Promise<void> {
  const response = await fetch(`${API_BASE}/api/fcm/register`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify({ smokerId, token: fcmToken }),
  });
  if (!response.ok) {
    throw new Error(`Failed to register FCM token: ${response.statusText}`);
  }
}

// Send command via WebSocket
export function sendCommand(ws: WebSocket | null, commandKey: string, commandValue?: string | number): void {
  if (!ws || ws.readyState !== WebSocket.OPEN) {
    console.warn('WebSocket is not connected');
    return;
  }

  const message = {
    action: 'command',
    commandKey,
    commandValue,
  };

  ws.send(JSON.stringify(message));
}
