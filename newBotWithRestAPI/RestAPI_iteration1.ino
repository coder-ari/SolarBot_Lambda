#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>
#include <ArduinoJson.h>

const char* ssid = "Aritra's A53";
const char* password = "fzne5437";

WiFiServer server(80);
WiFiClient client;
String data = "";

// Pins
#define pwm_drive 18
#define dir_drive 17
#define pwm_brush 16
#define dir_brush 13
#define lin_act_1_pwm 33
#define lin_act_1_dir 32
#define voltage_pin 14
#define trig_1 15
#define echo_1 2
#define trig_2 23
#define echo_2 19
#define current_sensor_pin 34  // Current sensor (HW-872A) analog pin

// Variables
int rpm = 0;
int pos = 0;
int new_pos = 0;
int dist = 0;
int linear_actr_pwm = 255;
float reference_volt = 11.0;
float volt = 0.0;
float volt_scale_factor = 1.0;
long lastUltrasonicCheck = 0;
int measurement_1 = 0;
int measurement_2 = 0;
long lastActionTime = 0;
bool actuatorMoving = false;
long lastCurrentCheck = 0;
const float sensitivity = 0.066;  // HW-872A sensitivity in V/A
const float vRef = 2.5;           // Reference voltage (assuming 5V → 2.5V midpoint)

void setup() {
  Serial.begin(115200);

  // Pin Modes
  pinMode(dir_drive, OUTPUT);
  pinMode(pwm_drive, OUTPUT);
  pinMode(dir_brush, OUTPUT);
  pinMode(pwm_brush, OUTPUT);
  pinMode(lin_act_1_pwm, OUTPUT);
  pinMode(lin_act_1_dir, OUTPUT);
  pinMode(voltage_pin, INPUT);
  pinMode(trig_1, OUTPUT);
  pinMode(echo_1, INPUT);
  pinMode(trig_2, OUTPUT);
  pinMode(echo_2, INPUT);
  pinMode(current_sensor_pin, INPUT);

  // WiFi Setup
  Serial.println("Connecting to WIFI");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("\nWiFi connected");
  Serial.print("ESP32 Local IP: ");
  Serial.println(WiFi.localIP());
  server.begin();
}

void loop() {
  handleWiFiClient();
  // checkVoltage();
  checkUltrasonic();
  handleActuator();
  // checkCurrent();
}

void handleWiFiClient() {
  client = server.available();
  if (!client) return;

  data = checkClient();  
  Serial.print("Processed request: ");
  Serial.println(data);  // Print the request received

  if (data.indexOf("/data") != -1) {
    sendSensorData(client);
  }

  // Command processing
  String command = data.substring(0, 1);
  int value = data.substring(1).toInt();

  if (command == "f") forward(value);
  else if (command == "r") reverse(value);
  else if (command == "b") brake();
  else if (command == "c") run_brush(value);
  else if (command == "e") run_brush_rev(value);
  else if (command == "s") stop_brush();
  else if (command == "l" || command == "m" || command == "n" || command == "o") move_actuator(value);
  else if (command == "p") lin_stop();

  client.stop();
}


void checkVoltage() {
  volt = 13.3;
  linear_actr_pwm = calc_pwm(volt);
}

void checkUltrasonic() {
  if (millis() - lastUltrasonicCheck >= 100) {
    measurement_1 = ultrasonic_read(trig_1, echo_1);
    measurement_2 = ultrasonic_read(trig_2, echo_2);
    lastUltrasonicCheck = millis();

    Serial.print("Distance 1: ");
    Serial.print(measurement_1);
    Serial.print(" cm | Distance 2: ");
    Serial.print(measurement_2);
    Serial.println(" cm");

    if (measurement_1 < 30 || measurement_2 < 30) { 
      Serial.println("Obstacle detected! Stopping motors...");
      stop_brush();
      brake();
    }
  }
}

void move_actuator(int target_pos) {
  if (actuatorMoving) return;

  new_pos = target_pos;
  dist = abs(pos - new_pos);

  if (dist > 0) {
    actuatorMoving = true;
    lastActionTime = millis();
    if (pos < new_pos) lin_inc();
    else lin_dec();
  }
}

void handleActuator() {
  if (actuatorMoving && millis() - lastActionTime >= calc_time(dist)) {
    lin_stop();
    actuatorMoving = false;
    pos = new_pos;
  }
}

void checkCurrent() {
  if (millis() - lastCurrentCheck >= 500) { // Read current every 500ms
    int adc_value = analogRead(current_sensor_pin);
    float voltage = (adc_value / 4095.0) * 3.3; // ESP32 ADC 12-bit, 3.3V reference
    //float current = (voltage - vRef) / sensitivity;

    Serial.print("voltage ref: ");
    Serial.print(voltage);
    Serial.println(" V");

    lastCurrentCheck = millis();
  }
}

// Motor Functions
void forward(int pwm) {
  digitalWrite(dir_drive, HIGH);
  analogWrite(pwm_drive, pwm);
}

void reverse(int pwm) {
  digitalWrite(dir_drive, LOW);
  analogWrite(pwm_drive, pwm);
}

void brake() {
  digitalWrite(dir_drive, LOW);
  analogWrite(pwm_drive, 0);
}

void run_brush(int pwm) {
  digitalWrite(dir_brush, HIGH);
  analogWrite(pwm_brush, pwm);
}

void run_brush_rev(int pwm) {
  digitalWrite(dir_brush, LOW);
  analogWrite(pwm_brush, pwm);
}

void stop_brush() {
  analogWrite(pwm_brush, 0);
}

// Actuator Functions
void lin_inc() {
  digitalWrite(lin_act_1_dir, LOW);
  analogWrite(lin_act_1_pwm, linear_actr_pwm);
}

void lin_dec() {
  digitalWrite(lin_act_1_dir, HIGH);
  analogWrite(lin_act_1_pwm, linear_actr_pwm);
}

void lin_stop() {
  analogWrite(lin_act_1_pwm, 0);
}

int calc_time(int dist) {
  return (int)(1000 * dist / 6.893);
}

int calc_pwm(float volt) {
  return (int)((11.0 / volt) * 255);
}

// Ultrasonic Sensor Function
int ultrasonic_read(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long duration = pulseIn(echo, HIGH, 30000);
  return (duration / 2) * 0.0343;
}

void sendSensorData(WiFiClient client) {
  Serial.println("Preparing JSON response...");

  StaticJsonDocument<200> jsonDoc;
  jsonDoc["ultrasonic_1"] = measurement_1;
  jsonDoc["ultrasonic_2"] = measurement_2;
  // jsonDoc["Temperature"]
  // jsonDoc["MPU_X"]
  // jsonDoc["MPU_Y"]
  // jsonDoc["MPU_Z"]
  // jsonDoc["Speed_in_RPM"]
  // jsonDoc["Distance_in_m"]

  String response;
  serializeJson(jsonDoc, response);

  Serial.println("Sending JSON: " + response);

  // Send proper HTTP response
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: application/json");
  client.println("Access-Control-Allow-Origin: *");  // CORS support
  client.println("Connection: close");
  client.println();
  client.println(response);

  Serial.println("JSON Data Sent!");
}

String checkClient() {
  while (!client.available()) {
    delay(1);
  }
  String request = client.readStringUntil('\r');  // Read HTTP request
  Serial.println("Received request: " + request);

  if (request.startsWith("GET ")) {
    int firstSpace = request.indexOf(' ');
    int secondSpace = request.indexOf(' ', firstSpace + 1);
    String extractedPath = request.substring(firstSpace + 1, secondSpace);
    Serial.println("Extracted Path: " + extractedPath);
    return extractedPath;
  }

  return "";
}
