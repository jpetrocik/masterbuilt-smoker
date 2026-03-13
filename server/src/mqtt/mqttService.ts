import mqtt from 'mqtt';
import config, { getBrokerUrl } from '../config';
import { updateSmokerStatus, insertTelemetry, purgeOldHistory } from '../database/database';
import { telemetryEmitter } from '../utils/eventEmitter';
import { SmokerTelemetryPayload } from '../types';

let client: mqtt.MqttClient;

export function connectMqtt(): void {
  const brokerUrl = getBrokerUrl();
  console.log(`Connecting to MQTT broker at ${brokerUrl}`);
  
  client = mqtt.connect(brokerUrl, {
    clientId: config.mqtt.clientIdPrefix + Math.random().toString(16).substr(2, 8),
    clean: true,
    reconnectPeriod: 5000,
  });

  client.on('connect', () => {
    console.log('Connected to MQTT broker');
    
    // Subscribe to presence and status topics
    client.subscribe('smoker/+/presence', (err) => {
      if (err) console.error('Failed to subscribe to presence topics:', err);
    });
    
    client.subscribe('smoker/+/status', (err) => {
      if (err) console.error('Failed to subscribe to status topics:', err);
    });
  });

  client.on('message', (topic, message) => {
    const messageStr = message.toString();
    
    // Parse topic: smoker/<smoker_id>/presence or smoker/<smoker_id>/status
    const topicParts = topic.split('/');
    if (topicParts.length < 3) return;
    
    const smokerId = topicParts[1];
    const messageType = topicParts[2];
    
    console.log(`[MQTT] Message received: topic=${topic}, type=${messageType}`);
    
    if (messageType === 'presence') {
      handlePresenceMessage(smokerId, messageStr);
    } else if (messageType === 'status') {
      handleStatusMessage(smokerId, messageStr);
    }
  });

  client.on('error', (err) => {
    console.error('MQTT connection error:', err);
  });

  client.on('offline', () => {
    console.log('MQTT client is offline');
  });

  // Periodic cleanup
  setInterval(() => {
    purgeOldHistory(config.database.maxHistoryHours, config.database.offlinePurgeHours);
  }, 60 * 60 * 1000); // Run every hour
}

function handlePresenceMessage(smokerId: string, payload: string): void {
  const status = payload === 'online' ? 'online' : 'offline';
  console.log(`Presence update for ${smokerId}: ${status}`);
  updateSmokerStatus(smokerId, status);
}

function handleStatusMessage(smokerId: string, payload: string): void {
  try {
    // console.log(`Parsing status message for ${smokerId}:`, payload);
    const data: SmokerTelemetryPayload = JSON.parse(payload);
    // console.log(`Parsed data:`, data);
    insertTelemetry(smokerId, data);
    
    // Broadcast to WebSocket clients
    telemetryEmitter.emit('telemetry', { smokerId, data });
  } catch (err) {
    console.error(`Failed to parse status message for ${smokerId}:`, err);
  }
}

export function disconnectMqtt(): void {
  if (client) {
    client.end();
  }
}

export function publishCommand(smokerId: string, command: string): void {
  const topic = `smoker/${smokerId}/command`;
  
  if (!client || !client.connected) {
    console.error(`Cannot publish command: MQTT client not connected. Topic: ${topic}, Command: ${command}`);
    return;
  }
  
  client.publish(topic, command, { qos: 1, retain: false }, (err) => {
    if (err) {
      console.error(`Failed to publish command to ${topic}:`, err);
    } else {
      console.log(`Published command to ${topic}: ${command}`);
    }
  });
}