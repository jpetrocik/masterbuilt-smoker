import { useEffect, useRef } from 'react';
import type { WebSocketMessage } from './api';
import { useSmokerStore } from '../store/useSmokerStore';

export function useTelemetryWebSocket(
  smokerId: string | null,
  sinceSeconds: number = 10800 // 3 hours
): void {
  const ws = useRef<WebSocket | null>(null);
  const reconnectTimeoutRef = useRef<ReturnType<typeof setTimeout> | null>(null);

  useEffect(() => {
    if (!smokerId) return;

    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    // Use the API base URL for WebSocket connection
    const apiHost = import.meta.env.VITE_API_BASE || 'http://localhost:3000';
    // Extract host from API base URL
    const wsHost = apiHost.replace(/^https?:\/\//, '');
    const wsUrl = `${protocol}//${wsHost}/ws/smoker/${smokerId}/data?since=${sinceSeconds}`;
    
    const socket = new WebSocket(wsUrl);

    socket.onopen = () => {
      console.log(`[WebSocket] Connected to ${wsUrl}`);
      console.log(`[WebSocket] Requesting historical data since: ${new Date(Date.now() - sinceSeconds * 1000).toISOString()}`);
    };

    socket.onmessage = (event) => {
      try {
        const message: WebSocketMessage = JSON.parse(event.data);
        
        // Handle message directly in the store
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
      // Attempt to reconnect after 5 seconds
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
