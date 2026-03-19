import { useEffect, useRef, useState } from 'react';
import type { TelemetryData, WebSocketMessage } from './api';
import { useSmokerStore } from '../store/useSmokerStore';

const socketConnection: { ws: WebSocket | null; lastTelemetryTimestamp: number | null } = {
  ws: null,
  lastTelemetryTimestamp: null,
};

export const sendCommand = (command: object): void => {
  if (socketConnection.ws && socketConnection.ws.readyState === WebSocket.OPEN) {
    const message = JSON.stringify(command);
    console.log(`[WebSocket] Sending command:`, message);
    socketConnection.ws.send(message);
  } else {
    console.error('[WebSocket] Connection not open. Cannot send command.');
  }
};

const MIN_RECONNECT_DELAY = 2000;
const MAX_RECONNECT_DELAY = 30000;
const RECONNECT_MULTIPLIER = 2;

export function useTelemetryWebSocket(
  smokerId: string | null,
  sinceSeconds: number = 10800 // 3 hours
): void {
  const [reconnectAttempt, setReconnectAttempt] = useState(0);
  const reconnectDelay = useRef(MIN_RECONNECT_DELAY);
  const reconnectTimeoutId = useRef<number | null>(null);
  const setIsConnecting = useSmokerStore((state) => state.setIsConnecting);
  const setIsOnline = useSmokerStore((state) => state.setIsOnline);


  useEffect(() => {
    if (!smokerId) {
      setIsConnecting(false);
      return;
    };

    setIsConnecting(true);

    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const apiHost = import.meta.env.VITE_API_BASE || 'http://localhost:3000';
    const wsHost = apiHost.replace(/^https?:\/\//, '');
    const wsUrl = `${protocol}//${wsHost}/ws/smoker/${smokerId}/data?since=${sinceSeconds}`;
    
    const socket = new WebSocket(wsUrl);

    socket.onopen = () => {
      console.log(`[WebSocket] Connected to ${wsUrl}`);
      socketConnection.ws = socket;
      setIsConnecting(false);
      reconnectDelay.current = MIN_RECONNECT_DELAY; // Reset delay on successful connection

    };

    socket.onmessage = (event) => {
      try {
        const message: WebSocketMessage = JSON.parse(event.data);
        if (message.type === 'historical' && Array.isArray(message.data)) {
          useSmokerStore.getState().initHistoricalData(message.data);
        } else if (message.type === 'live' && !Array.isArray(message.data)) {
          useSmokerStore.getState().updateFromTelemetry(message.data as TelemetryData);
        } else if (message.type === 'presence' && !Array.isArray(message.data)) {
          const presenceData = message.data as { smokerId: string; status: 'online' | 'offline' };
          if (presenceData.smokerId === smokerId) {
            setIsOnline(presenceData.status === 'online');
          }
        }
      } catch (err) {
        console.error('Failed to parse WebSocket message:', err);
      }
    };

    socket.onerror = (error) => {
      console.error('WebSocket error:', error);
      socket.close();
    };

    socket.onclose = () => {
      console.log('WebSocket disconnected');
      if (socketConnection.ws === socket) {
        socketConnection.ws = null;
        setIsConnecting(true);
      }

      // Schedule reconnection
      reconnectTimeoutId.current = window.setTimeout(() => {
        setReconnectAttempt(prev => prev + 1);
      }, reconnectDelay.current);

      // Increase delay for next attempt
      reconnectDelay.current = Math.min(
        reconnectDelay.current * RECONNECT_MULTIPLIER,
        MAX_RECONNECT_DELAY
      );
    };

    // Cleanup function
    return () => {
      console.log('[WebSocket] Cleaning up connection.');
      // Prevent the onclose handler from firing during this controlled shutdown
      socket.onclose = null; 
      socket.close();
      if (socketConnection.ws === socket) {
         socketConnection.ws = null;
      }

      // Clear any pending reconnection timeout
      if (reconnectTimeoutId.current) {
        clearTimeout(reconnectTimeoutId.current);
        reconnectTimeoutId.current = null;
      }
    };
  }, [smokerId, sinceSeconds, reconnectAttempt, setIsConnecting, setIsOnline]);
}
