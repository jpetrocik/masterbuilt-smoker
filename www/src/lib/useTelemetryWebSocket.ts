import { useEffect, useRef } from 'react';
import type { WebSocketMessage } from './api';
import { useSmokerStore } from '../store/useSmokerStore';

// 1. Create a simple object to hold the active WebSocket instance
const socketConnection: { ws: WebSocket | null } = {
  ws: null,
};

// 2. Export a function to send commands
export const sendCommand = (command: object): void => {
  if (socketConnection.ws && socketConnection.ws.readyState === WebSocket.OPEN) {
    const message = JSON.stringify(command);
    console.log(`[WebSocket] Sending command:`, message);
    socketConnection.ws.send(message);
  } else {
    console.error('[WebSocket] Connection not open. Cannot send command.');
  }
};

export function useTelemetryWebSocket(
  smokerId: string | null,
  sinceSeconds: number = 10800 // 3 hours
): void {
  const ws = useRef<WebSocket | null>(null);
  const reconnectTimeoutRef = useRef<ReturnType<typeof setTimeout> | null>(null);

  useEffect(() => {
    if (!smokerId) return;

    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const apiHost = import.meta.env.VITE_API_BASE || 'http://localhost:3000';
    const wsHost = apiHost.replace(/^https?:\/\//, '');
    const wsUrl = `${protocol}//${wsHost}/ws/smoker/${smokerId}/data?since=${sinceSeconds}`;
    
    const socket = new WebSocket(wsUrl);

    socket.onopen = () => {
      console.log(`[WebSocket] Connected to ${wsUrl}`);
      console.log(`[WebSocket] Requesting historical data since: ${new Date(Date.now() - sinceSeconds * 1000).toISOString()}`);
      // 3. Assign the connected socket to our global object
      socketConnection.ws = socket;
    };

    socket.onmessage = (event) => {
      try {
        const message: WebSocketMessage = JSON.parse(event.data);
        
        if (message.type === 'historical' && Array.isArray(message.data)) {
          console.log(`[WebSocket] Received historical data: ${message.data.length} records`);
          useSmokerStore.getState().initHistoricalData(message.data);
        } else if (message.type === 'live' && !Array.isArray(message.data)) {
          console.log(`[WebSocket] Received live data`);
          useSmokerStore.getState().updateFromTelemetry(message.data);
        }
      } catch (err) {
        console.error('Failed to parse WebSocket message:', err);
      }
    };

    socket.onerror = (error) => {
      console.error('WebSocket error:', error);
    };

    socket.onclose = () => {
      console.log('WebSocket disconnected');
      // 4. Clear the global object on disconnect
      socketConnection.ws = null;
      reconnectTimeoutRef.current = setTimeout(() => {
        console.log('Attempting to reconnect WebSocket...');
      }, 5000);
    };

    ws.current = socket;

    return () => {
      socket.close();
      if (reconnectTimeoutRef.current) {
        clearTimeout(reconnectTimeoutRef.current);
      }
    };
  }, [smokerId, sinceSeconds]);
}
