# Product Requirements Document & Technical Specification
**Project:** Multitenant Smoker Telemetry API

## 1. Product Vision & Objective
Create a headless Node.js middleware service that acts as the source of truth for physical meat smoker hardware. The objective is to ingest high-frequency MQTT telemetry from multitenant smoker devices, persist that data locally via SQLite, and expose that data to client applications (web or mobile) through a clean WebSocket and REST interface. The client application relies on this backend to handle all hardware discovery, data retention, and offline-state management.

## 2. Client Application "User" Stories
*(Note to Developer/AI: In these stories, the "User" is the front-end client application).*
* **Discovery:** As a client app, I need to query a REST endpoint to get a list of all currently registered smokers and their online/offline status, so I can render a dashboard menu for the human user.
* **State Syncing:** As a client app, when I open a WebSocket connection for a specific `smokerId`, I need to optionally request the last *X* seconds of historical data, so I can immediately draw a complete temperature chart without waiting for new data to trickle in.
* **Real-time Streaming:** As a client app, once connected via WebSocket, I need to receive pushed JSON payloads every time the hardware publishes a new temperature reading, so I can update the live UI gauges instantly.
* **Presence Alerting:** As a client app, I need the backend to push a specific WebSocket event if the hardware drops offline (triggered by the MQTT LWT), so I can immediately alert the human user that the cook is unmonitored.

## 3. Out of Scope (V1.0)
* **No UI/Frontend Generation:** Do not generate any HTML, CSS, React, or frontend client code. This is strictly a headless backend service.
* **No User Authentication:** The API and WebSockets do not require JWTs, API keys, or login systems. Assume a trusted local network environment.
* **No Hardware Control (Yet):** This version is read-only for telemetry. The client application cannot send target temperatures back to the hardware. 
* **No Cloud Syncing:** All data remains strictly on the local SQLite database.

---

## 4. Technical Implementation Specification

### 4.1 Overview
Create a backend server to manage telemetry data from multiple meat smokers. The service will ingest real-time telemetry data formatted as JSON via MQTT topics, maintain a configurable history window of that data in a local database, and publish both historical and real-time data to connected web clients via WebSockets.

### 4.2 Core Tech Stack
* **Language:** TypeScript (Strict mode enabled)
* **Runtime:** Node.js (Version 22.5+ required for native SQLite support)
* **Framework:** Express.js (for basic HTTP routing/setup)
* **Real-time Communication:** `ws` (Standard WebSockets) to allow strict path-based connections for frontend clients.
* **Hardware Ingestion:** `mqtt` (MQTT.js) to subscribe to presence and telemetry topics.
* **Database:** `node:sqlite` (Use Node's native SQLite implementation) to store historical telemetry data.
* **Configuration:** The app must read from a local `config.json` file on startup.
* **Documentation:** Swagger UI for REST API documentation (development only).

### 4.2.1 Swagger Documentation
* **Purpose:** Provide interactive API documentation for REST endpoints during development.
* **Access:** Available at `/api-docs` when `NODE_ENV=development`.
* **Generation:** Uses `swagger-jsdoc` to generate OpenAPI spec from JSDoc comments in route handlers.
* **Security:** Strictly disabled in production (checked via `process.env.NODE_ENV`).
* **Dependencies:** `swagger-ui-express`, `swagger-jsdoc`.

### 4.3 Multitenant Architecture & Lifecycle
This is a multitenant system where each physical smoker is a separate tenant identified by a unique `smokerId`. The backend identifies and registers tenants dynamically based on MQTT presence topics.
* **Registration:** When a new topic is published with a new `smokerId`, the backend registers the smoker.
* **TTL / Purge:** If a smoker's status remains "offline" for a configurable number of hours (defined in `config.json`), the backend must completely purge that smoker's historical data from the database and deregister the tenant.

**MQTT Contract for Discovery:**
* **Presence Topic:** `smoker/<smoker_id>/presence`
* **LWT (Death / Disconnect):** Configured by hardware on connection. Payload: `"offline"`, Retain: `true`, QoS: `1`.
* **Birth Message:** Published immediately by hardware upon successful connection. Payload: `"online"`, Retain: `true`, QoS: `1`.

### 4.4 Telemetry Data Ingestion
Data is published by the hardware every 30 seconds to a status topic.
* **Telemetry Topic:** `smoker/<smoker_id>/status`
* **JSON Payload Example:**
  ```json
  {
    "temperature": 224,
    "targetTemperature": 225,
    "cookTimer": 5014,
    "probe1": 98,
    "targetProbe1": 134,
    "probe2": 96,
    "targetProbe2": 134,
    "probe3": 98,
    "targetProbe3": 134,
    "probe4": 101,
    "targetProbe4": 134
  }```

### 4.5 Data Persistence (node:sqlite) 
The backend must maintain a rolling window of telemetry history in SQLite. The maximum retention time is determined by a "configurable hours of history" setting in config.json.
* Dynamic Probes: The fields probe1 through probe4 and their respective target fields are optional. They only appear in the MQTT payload when a physical probe is plugged into the hardware.
* Null Handling: The database schema must allow NULL values for these probe columns. If a probe is added mid-cook, start recording it. If removed, log it as NULL.
* Extensibility: The ingestion logic must ignore any unexpected JSON elements without throwing errors, allowing future firmware updates to add fields safely.

### 4.6 WebSocket Publishing & Retrieval
Clients connect to the backend via WebSockets to retrieve historical batches and stream real-time updates.
* Connection Path: ws://<host>/ws/smoker/<smoker_id>/data
* Historical Batching (Query Parameter): An optional since query parameter (representing seconds into the past) can be appended: /ws/smoker/12345/data?since=360.
** If since is provided, the backend must immediately query SQLite for all records for that smoker_id created between now and since seconds ago, and send this batch to the client over the socket.
** The since value cannot exceed the configured maximum history hours.
** If no since parameter is provided, no historical batch is sent.
* Continuous Streaming: Once the connection is established (and any requested historical batch is sent), the backend must push all new incoming MQTT telemetry payloads for that specific smoker_id directly to the connected WebSocket client until the client disconnects.

### 5. Success Metrics & Acceptance Criteria
* The system successfully registers a new smokerId automatically upon receiving an MQTT Birth message.

* The system gracefully catches the MQTT LWT message and updates the database state to "offline".

* Connecting to ws://<host>/smoker/<smoker_id>/data?since=3600 successfully pulls the last hour of data from SQLite and transmits it over the socket before streaming live updates.

* The backend runs cleanly via npm start with no TypeScript errors, reading all variables from config.json.

### 6. Configuration File (config.json) 
The application must read its configuration from a config.json file at the root of the project. The AI should generate a default file using the exact structure below. All database retention, server ports, and MQTT connection logic must utilize these values dynamically.
```json
{
  "mqtt": {
    "host": "192.168.1.100",
    "port": 1883,
    "protocol": "mqtt",
    "username": "your_mqtt_user",
    "password": "your_mqtt_password",
    "clientIdPrefix": "smoker-backend-"
  },
  "server": {
    "port": 3000
  },
  "database": {
    "path": "./data/smoker_history.db",
    "maxHistoryHours": 6,
    "offlinePurgeHours": 1
  }
```
