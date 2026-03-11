import fs from 'fs';
import path from 'path';

interface Config {
  mqtt: {
    brokerUrl: string;
  };
  database: {
    path: string;
  };
  history: {
    retentionHours: number;
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
    mqtt: { brokerUrl: 'mqtt://localhost:1883' },
    database: { path: './smokers.db' },
    history: { retentionHours: 24 }
  };
}

export default config;