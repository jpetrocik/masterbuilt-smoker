import fs from 'fs';
import path from 'path';

interface Config {
  mqtt: {
    host: string;
    port: number;
    protocol: string;
    username?: string;
    password?: string;
    clientIdPrefix: string;
  };
  server: {
    port: number;
  };
  database: {
    path: string;
    maxHistoryHours: number;
    offlinePurgeHours: number;
  };
}

let config: Config;

try {
  const configPath = path.join(__dirname, '..', 'config.json');
  const configFile = fs.readFileSync(configPath, 'utf-8');
  config = JSON.parse(configFile);
} catch (error) {
  console.error('Error loading config.json:', error);
  // Default config
  config = {
    mqtt: {
      host: 'localhost',
      port: 1883,
      protocol: 'mqtt',
      clientIdPrefix: 'smoker-backend-'
    },
    server: {
      port: 3000
    },
    database: {
      path: './data/smoker_history.db',
      maxHistoryHours: 6,
      offlinePurgeHours: 1
    }
  };
}

// Construct broker URL
export function getBrokerUrl(): string {
  const { protocol, host, port, username, password } = config.mqtt;
  let auth = '';
  if (username && password) {
    auth = `${username}:${password}@`;
  }
  return `${protocol}://${auth}${host}:${port}`;
}

export default config;