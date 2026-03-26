import { WebSocketServer, WebSocket } from 'ws';
import http from 'http';
import url from 'url';
import { getHistoricalData } from '../database/database';
import { telemetryEmitter } from '../utils/eventEmitter';
import { SmokerTelemetryData } from '../types';
import { publishCommand } from '../mqtt/mqttService';

export function createWebSocketServer(server: http.Server): void {
  const wss = new WebSocketServer({ noServer: true });

  server.on('upgrade', (request, socket, head) => {
    if (!request.url) {
      socket.destroy();
      return;
    }
    
    const pathname = url.parse(request.url).pathname;
    
    // Check if the path matches /ws/smoker/:id/data
    const match = pathname?.match(/^\/ws\/smoker\/([^\/]+)\/data$/);
    
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
    console.log(`[WebSocket] Connection established for smoker: ${smokerId}`);
    
    // Parse 'since' query parameter (in seconds)
    if (!request.url) return;
    const query = url.parse(request.url, true).query;
    
    // Only fetch and send historical data if 'since' parameter is provided
    if (query.since) {
      const sinceSeconds = parseInt(query.since as string, 10);
      const since = Date.now() - (sinceSeconds * 1000);
      
      const historicalData = getHistoricalData(smokerId, since);

      let nextTimestamp = 0;
      const reducedHistoricalData = historicalData.filter(row => {
        if (row.timestamp >= nextTimestamp) {
          nextTimestamp = Math.floor(row.timestamp / 60000) * 60000 + 60000; // Next minute mark
          return true;
        }
        return false;
      });

      console.log(`[WebSocket] Fetching historical data for ${smokerId} since ${new Date(since).toISOString()}: ${historicalData.length} records`);
      if (reducedHistoricalData.length > 0) {
        ws.send(JSON.stringify({
          type: 'historical',
          data: reducedHistoricalData
        }));
      }
    }
    
    // Listener for new telemetry data
    const onTelemetry = (smokerTelemetryData: SmokerTelemetryData) => {
      if (smokerTelemetryData.smokerId === smokerId) {

        ws.send(JSON.stringify({
          type: 'live',
          data: smokerTelemetryData
        }));
      }
    };
    
    telemetryEmitter.on('telemetry', onTelemetry);

    // Listener for presence changes
    const onPresenceChange = (changedSmokerId: string, status: 'online' | 'offline') => {
      if (changedSmokerId === smokerId) {
        ws.send(JSON.stringify({
          type: 'presence',
          data: { smokerId: changedSmokerId, status }
        }));
      }
    };
    telemetryEmitter.on('presenceChange', onPresenceChange);

    // Handle incoming messages (commands)
    ws.on('message', (data) => {
      try {
        const messageStr = data.toString();
        const message = JSON.parse(messageStr);

        // Validate action
        if (message.action !== 'command') {
          console.warn(`Invalid action: ${message.action}. Expected 'command'.`);
          return;
        }

        // Validate commandKey whitelist
        const validCommandKeys = [
          'setTemp', 'setKp', 'setKi', 'setKd', 'setCookTime',
          'setProbe1Label', 'setProbe2Label', 'setProbe3Label', 'setProbe4Label',
          'setProbe1Target', 'setProbe2Target', 'setProbe3Target', 'setProbe4Target'
        ];

        if (!validCommandKeys.includes(message.commandKey)) {
          console.warn(`Invalid commandKey: ${message.commandKey}`);
          return;
        }

        // Translate to plain-text format
        let commandString: string;
        if (message.commandValue !== undefined && message.commandValue !== null) {
          commandString = `${message.commandKey}=${message.commandValue}`;
        } else {
          commandString = `${message.commandKey}`;
        }

        // Publish to MQTT
        publishCommand(smokerId, commandString);

      } catch (err) {
        console.warn(`Failed to process WebSocket message: ${(err as Error).message}`);
      }
    });
    
    // Cleanup on close
    ws.on('close', () => {
      console.log(`WebSocket disconnected for smoker: ${smokerId}`);
      telemetryEmitter.off('telemetry', onTelemetry);
      telemetryEmitter.off('presenceChange', onPresenceChange);
    });
    
    ws.on('error', (err) => {
      console.error(`WebSocket error for smoker ${smokerId}:`, err);
      telemetryEmitter.off('telemetry', onTelemetry);
      telemetryEmitter.off('presenceChange', onPresenceChange);
    });
  });
}