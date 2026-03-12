# Smoker Simulator

A Node.js-based MQTT simulator for testing the smoker backend and frontend.

## Features

- Simulates smoker temperature and probe temperatures using a realistic random walk algorithm
- Implements MQTT presence with Birth and Last Will and Testament (LWT) messages
- Publishes telemetry data to `smoker/<smokerId>/status` every 5 seconds
- Subscribes to and handles commands on `smoker/<smokerId>/command`
- Configurable number of active probes (0-4)

## Installation

1. Install dependencies:
```bash
npm install
```

## Configuration

Edit `config.json` to customize the simulator:

```json
{
  "smokerId": "smoker123",
  "mqttBroker": "mqtt://localhost:1883",
  "publishInterval": 5000,
  "activeProbes": 4
}
```

- **smokerId**: Unique identifier for the smoker
- **mqttBroker**: MQTT broker URL
- **publishInterval**: Telemetry publish interval in milliseconds
- **activeProbes**: Number of probes (0-4) that are "plugged in"

## Running the Simulator

```bash
npm start
```

The simulator will:
1. Connect to the MQTT broker
2. Publish an "online" birth message to `smoker/<smokerId>/presence`
3. Subscribe to `smoker/<smokerId>/command`
4. Publish telemetry data every 5 seconds to `smoker/<smokerId>/status`

## Telemetry Payload Structure

The simulator publishes JSON payloads matching the firmware structure:

```json
{
  "temperature": 224.5,
  "targetTemperature": 225.0,
  "cookTimer": 240,
  "cookTime": 300,
  "dutyCycle": 0.75,
  "probe1": 145.2,
  "targetProbe1": 165,
  "alarmProbe1": false,
  "probe2": 151.3,
  "targetProbe2": 160,
  "alarmProbe2": false
}
```

## Supported Commands

The simulator listens for and handles the following commands on `smoker/<smokerId>/command`:

- `setCookTime=<minutes>` - Set cook timer countdown
- `setProbeXTarget=<temp>` - Set probe X target temperature (X=1-4)
- `setProbeXLabel=<label>` - Set probe X label (logged but not stored)

## Random Walk Algorithm

Temperatures don't jump randomly but follow a realistic trend:
- Start at 75°F
- Each interval, add a random value between -1.0 and +2.0
- Cap maximum temperature at 225°F
- This creates a natural heating trend for frontend chart testing

## MQTT Topics

| Topic | Direction | Description |
|-------|-----------|-------------|
| `smoker/<smokerId>/presence` | Publish | LWT/Birth messages ("online"/"offline") |
| `smoker/<smokerId>/status` | Publish | Telemetry data (JSON) |
| `smoker/<smokerId>/command` | Subscribe | Commands from backend/frontend |

## Testing with Backend

1. Start your backend server
2. Start the simulator: `npm start`
3. The backend should detect the smoker as "online" via the presence topic
4. Telemetry data will be published every 5 seconds

## Testing with Frontend

1. Start the backend server
2. Start the simulator
3. Start the frontend (from the `www` directory)
4. The frontend should display real-time smoker and probe data
