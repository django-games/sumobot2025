/*  
  Rui Santos & Sara Santos - Random Nerd Tutorials
  https://RandomNerdTutorials.com/esp32-wi-fi-car-robot-arduino/
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*/

#include <WiFi.h>
#include <WebServer.h>

// Replace with your network credentials
const char* ssid     = "REPLACE_WITH_YOUR_SSID";
const char* password = "REPLACE_WITH_YOUR_PASSWORD";

// Create an instance of the WebServer on port 80
WebServer server(80);

// Digital output pins for Arduino communication
int outputPin1 = 27;  // First bit
int outputPin2 = 26;  // Second bit

// Command definitions using 2 bits
// 00 = Stop
// 11 = Forward  
// 01 = Left
// 10 = Right

String valueString = String(0);

void handleRoot() {
  const char html[] PROGMEM = R"rawliteral(
  <!DOCTYPE HTML><html>
  <head>
    <meta name="viewport" content="width=device-width, initial-scale=1">    <link rel="icon" href="data:,">
    <style>
      html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center; background-color: #808080; color: white; }
      .button { -webkit-user-select: none; -moz-user-select: none; -ms-user-select: none; user-select: none; background-color: #2196F3; border: none; color: white; padding: 12px 28px; text-decoration: none; font-size: 26px; margin: 1px; cursor: pointer; }
      .button2 {background-color: #f44336;}
    </style>
    <script>
      function moveForward() { fetch('/forward'); }
      function moveLeft() { fetch('/left'); }
      function stopRobot() { fetch('/stop'); }
      function moveRight() { fetch('/right'); }
      function moveReverse() { fetch('/reverse'); }

      function updateMotorSpeed(pos) {
        document.getElementById('motorSpeed').innerHTML = pos;
        fetch(`/speed?value=${pos}`);
      }
    </script>
  </head>
  <body>
    <h1>ESP32 Motor Control</h1>
    <p><button class="button" onclick="moveForward()">FORWARD</button></p>
    <div style="clear: both;">
      <p>
        <button class="button" onclick="moveLeft()">LEFT</button>
        <button class="button button2" onclick="stopRobot()">STOP</button>
        <button class="button" onclick="moveRight()">RIGHT</button>
      </p>
    </div>
    <p><button class="button" onclick="moveReverse()">REVERSE</button></p>
    <p>Motor Speed: <span id="motorSpeed">0</span></p>
    <input type="range" min="0" max="100" step="25" id="motorSlider" oninput="updateMotorSpeed(this.value)" value="0"/>
  </body>
  </html>)rawliteral";
  server.send(200, "text/html", html);
}

void handleForward() {
  Serial.println("Forward");
  // Send 11 (Forward)
  digitalWrite(outputPin1, HIGH);
  digitalWrite(outputPin2, HIGH);
  server.send(200);
}

void handleLeft() {
  Serial.println("Left");
  // Send 01 (Left)
  digitalWrite(outputPin1, LOW);
  digitalWrite(outputPin2, HIGH);
  server.send(200);
}

void handleStop() {
  Serial.println("Stop");
  // Send 00 (Stop)
  digitalWrite(outputPin1, LOW);
  digitalWrite(outputPin2, LOW);
  server.send(200);
}

void handleRight() {
  Serial.println("Right");
  // Send 10 (Right)
  digitalWrite(outputPin1, HIGH);
  digitalWrite(outputPin2, LOW);
  server.send(200);
}

void handleReverse() {
  Serial.println("Reverse");
  // Send 00 (Stop) - treating reverse as stop for this implementation
  // You can modify this if you want a specific code for reverse
  digitalWrite(outputPin1, LOW);
  digitalWrite(outputPin2, LOW);
  server.send(200);
}

void handleSpeed() {
  if (server.hasArg("value")) {
    valueString = server.arg("value");
    int value = valueString.toInt();
    Serial.println("Speed setting received: " + String(value));
    // Note: Speed control removed since we're only sending digital commands
    // The Arduino will handle motor speed control
  }
  server.send(200);
}

void setup() {
  Serial.begin(115200);

  // Set the output pins for Arduino communication
  pinMode(outputPin1, OUTPUT);
  pinMode(outputPin2, OUTPUT);
  
  // Initialize pins to Stop state (00)
  digitalWrite(outputPin1, LOW);
  digitalWrite(outputPin2, LOW);
  
  // Connect to Wi-Fi
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  // Define routes
  server.on("/", handleRoot);
  server.on("/forward", handleForward);
  server.on("/left", handleLeft);
  server.on("/stop", handleStop);
  server.on("/right", handleRight);
  server.on("/reverse", handleReverse);
  server.on("/speed", handleSpeed);

  // Start the server
  server.begin();
}

void loop() {
  server.handleClient();
}