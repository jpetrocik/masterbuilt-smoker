const mqtt = require('mqtt');
const fs = require('fs');
const path = require('path');

// Load configuration
const configPath = path.join(__dirname, 'config.json');
const config = JSON.parse(fs.readFileSync(configPath, 'utf8'));

// Default values
const smokerId = config.smokerId || 'smoker123';
const mqttBroker = config.mqttBroker || 'mqtt://localhost:1883';
const publishInterval = config.publishInterval || 5000;
const activeProbes = config.activeProbes || 4;

// MQTT Topics
const presenceTopic = `smoker/${smokerId}/presence`;
const statusTopic = `smoker/${smokerId}/status`;
const commandTopic = `smoker/${smokerId}/command`;

// Simulator State
let state = {
  temperature: 75.0,
  targetTemperature: 0.0,
  cookTimer: 0, // countdown in seconds
  cookTime: 0, // elapsed time in seconds (simulated)
  dutyCycle: 0.0,
  probes: [
    { current: 75.0, target: 0.0, alarm: false },
    { current: 75.0, target: 0.0, alarm: false },
    { current: 75.0, target: 0.0, alarm: false },
    { current: 75.0, target: 0.0, alarm: false }
  ]
};

// Initialize probes based on activeProbes config
function initializeProbes() {
  for (let i = 0; i < 4; i++) {
    if (i < activeProbes) {
      // Probe is plugged in - start at 75°F
      state.probes[i].current = 75.0;
    } else {
      // Probe is not plugged in - set to 0.0 (simulates missing probe)
      state.probes[i].current = 0.0;
    }
    state.probes[i].target = 0.0;
    state.probes[i].alarm = false;
  }
}

// Initialize probes based on activeProbes config
initializeProbes();

// Connect to MQTT broker
const client = mqtt.connect(mqttBroker, {
  clientId: `smoker_sim_${smokerId}`,
  will: {
    topic: presenceTopic,
    payload: 'offline',
    qos: 1,
    retain: true
  },
  reconnectPeriod: 5000
});

client.on('connect', () => {
  console.log('Connected to MQTT broker');
  
  // Publish birth message
  client.publish(presenceTopic, 'online', { qos: 1, retain: true }, (err) => {
    if (err) {
      console.error('Failed to publish birth message:', err);
    } else {
      console.log(`Published birth message to ${presenceTopic}: online`);
    }
  });
  
  // Subscribe to command topic
  client.subscribe(commandTopic, { qos: 1 }, (err) => {
    if (err) {
      console.error('Failed to subscribe to command topic:', err);
    } else {
      console.log(`Subscribed to ${commandTopic}`);
    }
  });
  
  // Start telemetry publishing
  setInterval(publishTelemetry, publishInterval);
});

client.on('message', (topic, message) => {
  if (topic === commandTopic) {
    const command = message.toString();
    console.log(`Received command: ${command}`);
    handleCommand(command);
  }
});

client.on('error', (err) => {
  console.error('MQTT error:', err);
});

client.on('disconnect', () => {
  console.log('Disconnected from MQTT broker');
});

// Handle incoming commands
function handleCommand(command) {
  try {
    // Parse setTemp command (sets smoker target temperature)
    if (command.startsWith('setTemp=')) {
      const newTargetTemp = parseInt(command.split('=')[1], 10);
      const MIN_TEMP = 37.0;
      const oldTargetTemp = state.targetTemperature;
      
      // Update target temperature
      state.targetTemperature = newTargetTemp;
      
      // Handle cook time based on firmware logic:
      // When setting temp to 0, stop cook timer and cancel scheduled shutdown
      if (newTargetTemp === 0) {
        state.cookTime = 0;
        state.cookTimer = 0;
        state.dutyCycle = 0;
        console.log(`Smoker turned OFF (target temp set to 0)`);
      }
      // When setting temp from 0 to non-zero, start cook time
      else if (oldTargetTemp <= MIN_TEMP) {
        // Start cook time tracking (simulated as elapsed time starts from 0)
        state.cookTime = 0;
        console.log(`Smoker turned ON (target temp: ${newTargetTemp}°)`);
      }
      // When setting temp from non-zero to non-zero, do nothing (continue cooking)
      
      console.log(`Set smoker target temperature: ${newTargetTemp}°`);
    }
    
    // Parse setCookTime command
    else if (command.startsWith('setCookTime=')) {
      const minutes = parseInt(command.split('=')[1], 10);
      state.cookTimer = minutes * 60; // Convert to seconds
      console.log(`Set cook timer: ${minutes} minutes (${state.cookTimer} seconds)`);
    }
    
    // Parse setProbeXTarget command
    const probeTargetMatch = command.match(/setProbe(\d)Target=(\d+)/);
    if (probeTargetMatch) {
      const probeIndex = parseInt(probeTargetMatch[1], 10) - 1;
      const targetTemp = parseInt(probeTargetMatch[2], 10);
      if (probeIndex >= 0 && probeIndex < 4) {
        state.probes[probeIndex].target = targetTemp;
        console.log(`Set probe ${probeIndex + 1} target: ${targetTemp}°`);
      }
    }
    
    // Parse setProbeXLabel command (for compatibility, though not used in state)
    const probeLabelMatch = command.match(/setProbe(\d)Label=(.+)/);
    if (probeLabelMatch) {
      const probeIndex = parseInt(probeLabelMatch[1], 10) - 1;
      const label = probeLabelMatch[2];
      console.log(`Set probe ${probeIndex + 1} label: ${label}`);
    }
  } catch (err) {
    console.error('Error parsing command:', err);
  }
}

// Generate telemetry using random walk
function generateTelemetry() {
  const MIN_TEMP = 37.0; // Same as firmware
  const smokerIsOn = state.targetTemperature > MIN_TEMP;
  
  // Only update temperatures and cook time when smoker is ON
  if (smokerIsOn) {
    // Random walk for smoker temperature
    const change = (Math.random() * 3.0) - 1.0; // -1.0 to +2.0
    state.temperature = Math.min(225, Math.max(75, state.temperature + change));
    
    // Random walk for probe temperatures (only for active probes)
    for (let i = 0; i < 4; i++) {
      if (i < activeProbes) {
        // Only simulate random walk for plugged-in probes
        const change = (Math.random() * 3.0) - 1.0;
        state.probes[i].current = Math.min(225, Math.max(75, state.probes[i].current + change));
        
        // Check alarm conditions
        state.probes[i].alarm = state.probes[i].target > 0 && state.probes[i].current > state.probes[i].target;
      } else {
        // Unplugged probes remain at 0.0
        state.probes[i].current = 0.0;
        state.probes[i].alarm = false;
      }
    }
    
    // Simulate cook time elapsed (every 5 seconds = 5 seconds elapsed)
    if (state.cookTimer > 0) {
      state.cookTimer = Math.max(0, state.cookTimer - (publishInterval / 1000));
      
      // Auto-off: when cookTimer hits zero, turn off smoker
      if (state.cookTimer === 0) {
        state.targetTemperature = 0;
        state.dutyCycle = 0;
      }
    }
    
    // Only track elapsed cook time when smoker is ON
    state.cookTime += (publishInterval / 1000);
    
    // Simulate duty cycle based on target temperature
    // Simple simulation: duty cycle increases as we get further from target
    const tempDiff = state.targetTemperature - state.temperature;
    state.dutyCycle = Math.max(0, Math.min(1, 0.5 + (tempDiff * 0.05)));
  } else {
    // Smoker is OFF - temperatures remain stable, no cooking happening
    // Cook time is reset when smoker turns off
    state.cookTime = 0;
    state.dutyCycle = 0;
    
    // Commenting out this code since it's behavor is incorrect. 
    // With target temp of 0, if probe target temp is set low enough,
    // the probes should alarm
    // // Probes also don't change when smoker is off
    // for (let i = 0; i < 4; i++) {
    //   state.probes[i].alarm = false;
    // }
  }
  
  return buildPayload();
}

// Build JSON payload matching firmware structure
function buildPayload() {
  const payload = {
    temperature: Math.round(state.temperature * 10) / 10,
    targetTemperature: Math.round(state.targetTemperature * 10) / 10,
    cookTimer: Math.round(state.cookTimer),
    cookTime: Math.round(state.cookTime),
    dutyCycle: Math.round(state.dutyCycle * 100) / 100
  };
  
  // Include probe data for all 4 probes (matching firmware behavior)
  // probeX is only included if temperature > 0.0
  // targetProbeX and alarmProbeX are always included
  for (let i = 0; i < 4; i++) {
    const probeNum = i + 1;
    
    // Only include probeX if current temperature > 0.0 (simulates missing probe)
    if (state.probes[i].current > 0.0) {
      payload[`probe${probeNum}`] = Math.round(state.probes[i].current * 10) / 10;
    }
    
    // Always include targetProbeX and alarmProbeX
    payload[`targetProbe${probeNum}`] = Math.round(state.probes[i].target * 10) / 10;
    payload[`alarmProbe${probeNum}`] = state.probes[i].alarm;
  }
  
  return payload;
}

// Publish telemetry to MQTT
function publishTelemetry() {
  const payload = generateTelemetry();
  const payloadStr = JSON.stringify(payload);
  
  client.publish(statusTopic, payloadStr, { qos: 1, retain: false }, (err) => {
    if (err) {
      console.error('Failed to publish telemetry:', err);
    } else {
      console.log(`Published telemetry to ${statusTopic}:`, payloadStr);
    }
  });
}

/** 
// Handle process termination
process.on('SIGINT', () => {
  console.log('\nShutting down simulator...');
  
  // Publish offline message before disconnecting
  client.publish(presenceTopic, 'offline', { qos: 1, retain: true }, () => {
    client.end(() => {
      console.log('Simulator stopped');
      process.exit(0);
    });
  });
});

process.on('SIGTERM', () => {
  console.log('\nShutting down simulator...');
  
  // Publish offline message before disconnecting
  client.publish(presenceTopic, 'offline', { qos: 1, retain: true }, () => {
    client.end(() => {
      console.log('Simulator stopped');
      process.exit(0);
    });
  });
});
*/

console.log('Smoker Simulator Starting...');
console.log(`Smoker ID: ${smokerId}`);
console.log(`MQTT Broker: ${mqttBroker}`);
console.log(`Publish Interval: ${publishInterval}ms`);
console.log(`Active Probes: ${activeProbes}`);
console.log(`Presence Topic: ${presenceTopic}`);
console.log(`Status Topic: ${statusTopic}`);
console.log(`Command Topic: ${commandTopic}`);
console.log('Press Ctrl+C to stop');
