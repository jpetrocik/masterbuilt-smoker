import { WebSocketServer, WebSocket } from 'ws';
import http from 'http';
import url from 'url';
import { getHistoricalData } from '../database/database';
import { telemetryEmitter } from '../utils/eventEmitter';
import { SmokerTelemetryPayload } from '../types';

export function createWebSocketServer(server: http.Server): void {
  const wss = new WebSocketServer({ noServer: true });

  server.on('upgrade', (request, socket, head) => {
    if (!request.url) {
      socket.destroy();
      return;
    }
    
    const pathname = url.parse(request.url).pathname;
    
    // Check if the path matches /smoker/:id/data
    const match = pathname?.match(/^\/smoker\/([^\/]+)\/data$/);
    
    if (match) {
      const smokerId = match[1];
      
      wss.handleUpgrade(request, socket, head, (ws) => {
        wss.emit('connection', ws, request, smokerId);
      });
    } else {
      socket.destroy();
    }
  });

  wss.on('connection', (ws: WebSocket, request: http.IncomingMessage, smokerId: string) => {
    console.log(`WebSocket connected for smoker: ${smokerId}`);
    
    // Parse 'since' query parameter
    if (!request.url) return;
    const query = url.parse(request.url, true).query;
    const since = query.since ? parseInt(query.since as string, 10) : Date.now() - (24 * 60 * 60 * 1000); // Default to last 24 hours
    
    // Fetch and send historical data
    const historicalData = getHistoricalData(smokerId, since);
    if (historicalData.length > 0) {
      ws.send(JSON.stringify({
        type: 'historical',
        data: historicalData
      }));
    }
    
    // Listener for new telemetry data
    const onTelemetry = (payload: { smokerId: string; data: SmokerTelemetryPayload }) => {
      if (payload.smokerId === smokerId) {
        ws.send(JSON.stringify({
          type: 'live',
          data: payload.data
        }));
      }
    };
    
    telemetryEmitter.on('telemetry', onTelemetry);
    
    // Cleanup on close
    ws.on('close', () => {
      console.log(`WebSocket disconnected for smoker: ${smokerId}`);
      telemetryEmitter.off('telemetry', onTelemetry);
    });
    
    ws.on('error', (err) => {
      console.error(`WebSocket error for smoker ${smokerId}:`, err);
      telemetryEmitter.off('telemetry', onTelemetry);
    });
  });
}