// Import required libraries
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Wire.h>

#define DRONE_I2C_ID 8

#define THROTTLE_UP "throttleUp"
#define THROTTLE_DOWN "throttleDown"
#define YAW_LEFT "yawLeft"
#define YAW_RIGHT "yawRight"
#define PITCH_UP "pitchUp"
#define ROLL_LEFT "rollLeft"
#define ROLL_RIGHT "rollRight"
#define PITCH_DOWN "pitchDown"

#define NOTIFY_CLIENTS_TYPE_LOG "log:"
#define NOTIFY_CLIENTS_TYPE_TEL "tel:"

unsigned long millisGetStatus;

const char *ssid = "ARDUINO_DRON_MARK_0";
//const char* password = "pass";

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

const char index_html[] PROGMEM = R"rawliteral(
<html lang="es">
  <head>
    <title>Prototipo Dron Mark 0</title>
    <meta name="viewport" content="width=device-width, initial-scale=1.0" />
    <meta charset="UTF-8" />
    <style>
      body {
        font-family: "sans-serif";
        height: 100%;
        width: 100%;
        margin: 0;
        overflow: none;
      }
      button {
        padding: 0;
      }
      h1 {
        font-size: 18px;
        line-height: 1;
        padding: 5px;
        margin: 0;
      }
      .flex {
        display: flex;
      }
      .flex-row {
        display: flex;
        flex-direction: row;
      }
      .flex-col {
        display: flex;
        flex-direction: column;
      }
      .flex-center {
        align-items: center;
      }
      .flex-justify-end {
        justify-content: end;
      }
      .w-full {
        width: 100%;
      }
      .h-full {
        height: 100%;
      }
      .w-height {
        height: 100%;
      }
      .rounded-full {
        border-radius: 100%;
      }
      .flex-1 {
        flex: 1;
      }
      .gap-50 {
        gap: 50px;
      }
      .text-right {
        text-align: right;
      }
      .aspect-box {
        aspect-ratio: 1 / 1;
      }
      @media screen and (min-width: 320px) and (max-width: 767px) and (orientation: portrait) {
        html {
          transform: rotate(-90deg);
          transform-origin: left top;
          width: 100vh;
          height: 100vw;
          overflow-x: hidden;
          position: absolute;
          top: 100%;
          left: 0;
        }
      }
      .led {
        width: 10px;
        height: 10px;
        border-radius: 100%;
        display: inline-block;
        margin-left: 2px;
        margin-right: 2px;
      }
      .led-on {
        background-color: lime;
      }
      .led-err {
        background-color: red;
      }
      .led-off {
        background-color: gray;
      }
      .indicator {
        display: flex;
        flex-direction: row;
        align-items: center;
        margin-right: 5px;
        border-right: 0.5px solid #ccc;
      }
      .log-item {
        padding: 2px;
        font-family: monospace;
        font-size: 11px;
        border-bottom: 0.5px solid #ccc;
      }
      .bar {
        width: 100%;
        position: relative;
        border: 0.5px solid #ccc;
        border-top: transparent;
        display: flex;
        justify-content: space-between;
        height: 20px;
        align-items: center;
      }
      .bar-indicator {
        position: absolute;
        height: 100%;
        width: 0%;
        overflow: hidden;
        background-color: lime;
        z-index: -1;
      }
      .bar-label {
        font-family: monospace;
        font-size: 11px;
        float: left;
      }
      .bar-value {
        font-family: monospace;
        font-size: 11px;
        text-align: right;
        float: right;
      }
    </style>
  </head>
  <body class="flex flex-row">
    <div class="flex flex-col w-height flex-1">
      <div class="flex flex-col">
        <div class="flex flex-row">
          <h1>Prototipo Dron Mark-0</h1>
          <div class="flex flex-row flex-1 flex-justify-end">
            <div class="indicator">
              <small>WS </small>
              <div id="ledWebSocketStatus" class="led led-off"></div>
            </div>
            <div class="indicator">
              <small>Vehículo </small>
              <div id="ledShipStatus" class="led led-off"></div>
            </div>
            <div class="indicator">
              <small>Batería </small>
              <div id="ledBatteryStatus" class="led led-off"></div>
            </div>
          </div>
        </div>
      </div>
      <p>
        <small
          >Proyecto para el curso de Electrónica de la carrera de Automatización
          y Robótica.<br />
          Rodrigo González. 2023.
        </small>
      </p>
      <div class="flex-row gap-50 flex-1">
        <div class="flex-col flex-1">
          <div class="flex-row flex-1">
            <div class="flex-1"></div>
            <button id="btnThrottleUp" class="flex-1 rounded-full">T</button>
            <div class="flex-1"></div>
          </div>
          <div class="flex-row flex-1">
            <button id="btnYawLeft" class="flex-1 rounded-full">Y</button>
            <div class="flex-1"></div>
            <button id="btnYawRight" class="flex-1 rounded-full">Y</button>
          </div>
          <div class="flex-row flex-1">
            <div class="flex-1"></div>
            <button id="btnThrottleDown" class="flex-1 rounded-full">T</button>
            <div class="flex-1"></div>
          </div>
        </div>
        <div class="flex flex-center flex-1">
          <div class="flex-col flex-1">
            <div class="aspect-box flex-1">
              <div class="flex-col h-full">
                <div class="flex-row flex-1">
                  <button id="btnThrottleUp" class="flex-1 rounded-full">
                    <small id="mB">B</small>
                  </button>
                  <button id="btnThrottleUp" class="flex-1 rounded-full">
                    <small id="mA">A</small>
                  </button>
                </div>
                <div class="flex-row flex-1">
                  <button id="btnThrottleUp" class="flex-1 rounded-full">
                    <small id="mC">C</small>
                  </button>
                  <button id="btnThrottleUp" class="flex-1 rounded-full">
                    <small id="mD">D</small>
                  </button>
                </div>
              </div>
            </div>
            <div class="flex-col" style="margin-top: 15px">
              <div class="bar">
                <div class="bar-indicator" id="accPerc"></div>
                <div class="bar-label">Aceleración</div>
                <div class="bar-value">
                  <span id="accVal"></span><small>μs</small>
                </div>
              </div>
              <div class="bar">
                <div class="bar-indicator" id="pitchPerc"></div>
                <div class="bar-label">Pitch</div>
                <div class="bar-value"><span id="pitchVal"></span>°</div>
              </div>
              <div class="bar">
                <div class="bar-indicator" id="rollPerc"></div>
                <div class="bar-label">Roll</div>
                <div class="bar-value"><span id="rolVal"></span>°</div>
              </div>
              <div class="bar">
                <div class="bar-indicator" id="yawPerc"></div>
                <div class="bar-label">Yaw</div>
                <div class="bar-value"><span id="yawVal"></span>°</div>
              </div>
            </div>
          </div>
        </div>
        <div class="flex-col flex-1">
          <div class="flex-row flex-1">
            <div class="flex-1"></div>
            <button id="btnPitchUp" class="flex-1 rounded-full">P</button>
            <div class="flex-1"></div>
          </div>
          <div class="flex-row flex-1">
            <button id="btnRollLeft" class="flex-1 rounded-full">R</button>
            <div class="flex-1"></div>
            <button id="btnRollRight" class="flex-1 rounded-full">R</button>
          </div>
          <div class="flex-row flex-1">
            <div class="flex-1"></div>
            <button id="btnPitchDown" class="flex-1 rounded-full">P</button>
            <div class="flex-1"></div>
          </div>
        </div>
      </div>
      <div class="flex flex-row">
        <div>
          <small>Mrk-prev: 1000</small>
        </div>
        <input
          type="range"
          class="flex-1"
          id="pulseRange"
          min="1000"
          max="2000"
        />
        <div>
          <small>2000</small>
        </div>
      </div>
    </div>
    <div class="flex flex-col" style="width: 100px">
      <strong><small>LOG</small></strong>
      <div
        class="flex flex-col w-full"
        style="overflow-y: scroll"
        id="logContainer"
      ></div>
    </div>
    <script>
      const $$ = (id) => document.getElementById(id);
      const ledWebSocketStatus = $$("ledWebSocketStatus");
      const ledShipStatus = $$("ledShipStatus");
      const ledBatteryStatus = $$("ledBatteryStatus");
      const btnThrottleUp = $$("btnThrottleUp");
      const btnYawLeft = $$("btnYawLeft");
      const btnYawRight = $$("btnYawRight");
      const btnThrottleDown = $$("btnThrottleDown");
      const btnPitchUp = $$("btnPitchUp");
      const btnRollLeft = $$("btnRollLeft");
      const btnRollRight = $$("btnRollRight");
      const btnPitchDown = $$("btnPitchDown");
      const pulseRange = $$("pulseRange");
      const logContainer = $$("logContainer");
      const mA = $$("mA");
      const mB = $$("mB");
      const mC = $$("mC");
      const mD = $$("mD");
      const accPerc = $$("accPerc");
      const accVal = $$("accVal");
      const pitchVal = $$("pitchVal");
      const rollVal = $$("rollVal");
      const yawVal = $$("yawVal");
      const gateway = `ws://${window.location.hostname}/ws`;
      let websocket;
      let hasWSError = false;
      window.addEventListener("load", onLoad);
      function initWebSocket() {
        console.log("Trying to open a WebSocket connection...");
        websocket = new WebSocket(gateway);
        websocket.onopen = onOpen;
        websocket.onclose = onClose;
        websocket.onerror = onError;
        websocket.onmessage = onMessage;
      }
      function onOpen(event) {
        ledWebSocketStatus.className = "led led-on";
        hasWSError = false;
        console.log("Connection opened");
      }
      function onClose(event) {
        console.log("Connection closed");
        if (!hasWSError) {
          ledWebSocketStatus.className = "led led-off";
        }
        setTimeout(initWebSocket, 2000);
      }
      function onError(event) {
        hasWSError = true;
        ledWebSocketStatus.className = "led led-err";
        console.log("Error", event);
      }
      function onMessage(event) {
        var state;
        const element = document.createElement("div");
        const [type, val] = event.data.split(":");
        if (type === "log") {
          element.innerText = val;
          element.className = "log-item";
          logContainer.appendChild(element);
        } else if (type === "tel") {
          const [sensorPitch, sensorRoll, sensorYaw, acc, _mA, _mB, _mC, _mD] =
            val.split("|");
          mA.innerText = _mA;
          mB.innerText = _mB;
          mC.innerText = _mC;
          mD.innerText = _mD;
          accVal.innerText = acc;
          accPerc.innerText = ((acc - 1000) * 100) / (2000 - 1000);
          pitchVal.innerText = sensorPitch;
          rollVal.innerText = sensorRoll;
          yawVal.innerText = sensorYaw;
        }
      }
      const onClickSend = (el, msg) =>
        el.addEventListener("click", () => websocket.send(msg));
      function onLoad(event) {
        initWebSocket();

        onClickSend(btnThrottleUp, "throttleUp");
        onClickSend(btnThrottleDown, "throttleDown");
        onClickSend(btnYawLeft, "yawLeft");
        onClickSend(btnYawRight, "yawRight");
        onClickSend(btnPitchUp, "pitchUp");
        onClickSend(btnRollLeft, "rollLeft");
        onClickSend(btnRollRight, "rollRight");
        onClickSend(btnPitchDown, "pitchDown");
        pulseRange.addEventListener("change", (event) => {
          console.log("pulse", pulseRange.value);
          websocket.send(pulseRange.value);
        });
      }
    </script>
  </body>
</html>
)rawliteral";

void notifyClients(String type, String data) {
  ws.textAll(type + data);
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo *)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    data[len] = 0;
    String message((char *)data);
    if (message == THROTTLE_UP
        || message == THROTTLE_DOWN
        || message == YAW_LEFT
        || message == YAW_RIGHT
        || message == PITCH_UP
        || message == ROLL_LEFT
        || message == ROLL_RIGHT
        || message == PITCH_DOWN) {
      notifyClients(NOTIFY_CLIENTS_TYPE_LOG, message);
      DRONE_execute(message);
      Serial.print("COMMAND:");
      Serial.println(message);
    } else {

      if (isNumber(message)) {
        Serial.print("Recibo pulsos:");
        Serial.println(message);
        DRONE_execute(message);
        notifyClients(NOTIFY_CLIENTS_TYPE_LOG, message);
        Serial.print("NUMBER:");
        Serial.println(message);
      }
    }
  }
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
             void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("WebSocket client #%u disconnected\n", client->id());
      break;
    case WS_EVT_DATA:
      handleWebSocketMessage(arg, data, len);
      break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}

void initWebSocket() {
  ws.onEvent(onEvent);
  server.addHandler(&ws);
}

void setup() {
  // Serial port for debugging purposes
  Serial.begin(9600);
  delay(10);

  // Connect to Wi-Fi
  WiFi.softAP(ssid);
  /*while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi: " + String(WiFi.status()));
  }*/

  // Print ESP Local IP Address
  delay(3000);
  Serial.print("IP: ");
  Serial.println(WiFi.softAPIP());


  initWebSocket();

  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", index_html);
  });

  // Start server
  server.begin();

  // Bus i2c con SDA=D1 y SCL=D2 de NodeMCU
  Wire.begin(D1, D2);

  millisGetStatus = millis();
}

void loop() {
  unsigned long now = millis();
  ws.cleanupClients();
  /* // Deshabilitado de momento
  if(now - millisGetStatus >= 1000) {
    DRONE_getStatus();
    millisGetStatus = now;
  }
  */
}

void DRONE_execute(String orden) {
  Wire.beginTransmission(DRONE_I2C_ID);
  Wire.write(orden.c_str(), orden.length());
  Wire.endTransmission();
}

void DRONE_getStatus() {
  Wire.requestFrom(DRONE_I2C_ID, 35);
  String receivedString;
  while (Wire.available()) {
    char c = Wire.read();
    receivedString += c;
  }
  Serial.println(receivedString);
  Serial.println("============");
  notifyClients(NOTIFY_CLIENTS_TYPE_TEL, receivedString);
}

bool isNumber(String str) {
  for (size_t i = 0; i < str.length(); i++) {
    if (!isDigit(str.charAt(i))) {
      return false;
    }
  }
  return true;
}