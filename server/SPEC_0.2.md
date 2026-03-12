## 1. Iteration 2: Hardware Control & Command Forwarding

### 1.1 Objective
Enable two-way communication by allowing connected WebSocket clients to send configuration and control commands down to the physical smoker hardware. The Node.js backend will act as a validating bridge, translating JSON WebSocket messages into plain-text MQTT payloads.

### 1.2 MQTT Command Contract
The backend will publish commands to a dedicated topic for each tenant.
* **Command Topic:** `smoker/<smoker_id>/command`
* **Payload Format:** Plain text string in `key=value` format (or just `key` for flag commands).
* **QoS:** `1` (At least once delivery ensures commands are not lost).
* **Retain:** `false` (Commands are instantaneous actions, not persistent states).

### 1.3 WebSocket Client-to-Server Protocol
Clients with an active WebSocket connection to `/smoker/<smoker_id>/data` can push messages to the server. The server must listen for incoming `message` events on the socket.
* **Expected Client Payload:** JSON object containing the command key and an optional value.
  ```json
  {
    "action": "command",
    "commandKey": "setTemp",
    "commandValue": 225
  }```

### 1.4 Protocol
* setTemp=[temp]
Set the target temperature (°F or °C, depending on configuration smoker configuration), e.g. setTemp=225

* setKp=[number]
Set the PID Kp value.
Example: setKp=300

* setKi=[number]
Set the PID Ki value.
Example: setKi=0.08

* setKd=[number]
Set the PID Kd value.
Example: setKd=0.5

* setCookTime=[minutes]
Set the cook timer duration in minutes.
Example: setCookTime=120

* setProbe[X]Label=[string] Set label for probe, where X is a number between 1-4, e.g. setProbe1Label=Brisket.

* setProbe[X]Target=[temp]
 Set target for probe, where X is a number between 1-4, e.g. setProbe1Target=120.

