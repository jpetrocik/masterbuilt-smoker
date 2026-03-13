import { useState, useEffect, useRef } from 'react';
import type { WebSocketMessage } from './api';
import { useSmokerStore } from '../store/useSmokerStore';

export function useTelemetryWebSocket(
  smokerId: string | null,
  sinceSeconds: number = 10800 // 3 hours
): WebSocket | null {
  const [ws, setWs] = useState<WebSocket | null>(null);
  const reconnectTimeoutRef = useRef<ReturnType<typeof setTimeout> | null>(null);

  useEffect(() => {
    if (!smokerId) return;

    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const wsUrl = `${protocol}//${window.location.host}/ws/smoker/${smokerId}/data?since=${sinceSeconds}`;
    
    const socket = new WebSocket(wsUrl);

    socket.onopen = () => {
      console.log('WebSocket connected');
    };

    socket.onmessage = (event) => {
      try {
        const message: WebSocketMessage = JSON.parse(event.data);
        
        // Handle message directly in the store
        if (message.type === 'historical') {
          const dataArray = Array.isArray(message.data) ? message.data : [message.data];
          console.log(`[WebSocket] Received historical data: ${dataArray.length} records`);
          // Add timestamps to historical data if not present
          const dataWithTimestamps = dataArray.map((item: any) => ({
            ...item,
            timestamp: item.timestamp || Date.now(),
          }));
          useSmokerStore.getState().addHistoricalData(dataWithTimestamps);
        } else if (message.type === 'live') {
          const data = message.data as any;
          console.log(`[WebSocket] Received live data`);
          useSmokerStore.getState().updateFromTelemetry(data);
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

    setWs(socket);

    return () => {
      socket.close();
      if (reconnectTimeoutRef.current) {
        clearTimeout(reconnectTimeoutRef.current);
      }
    };
  }, [smokerId, sinceSeconds]);

  return ws;
}
