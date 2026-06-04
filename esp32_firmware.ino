#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>

// =====================================================================
// REGISTER ALLOCATIONS & DRIVE PIN PERIPHERALS
// =====================================================================
#define M_LEFT_Fwd         14
#define M_LEFT_Rev         27
#define M_RIGHT_Fwd        26
#define M_RIGHT_Rev        25

#define FORCE_SENSOR_PIN   34
#define GRIPPER_SERVO_PIN  23

const int pwmFreq = 5000;
const int pwmResolution = 8;
const int chLeftFwd = 0, chLeftRev = 1, chRightFwd = 2, chRightRev = 3;

// Exponential Moving Average filter parameters
float smoothedForce = 0.0;
const float EMA_GAIN = 0.25; 

const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";
const char* mqtt_server = "broker.hivemq.com";

const char* topic_move = "efe_robot_2026/pi/cmd/move";
const char* topic_arm  = "efe_robot_2026/pi/cmd/arm";

WiFiClient espClient;
PubSubClient client(espClient);
Servo gripperServo;

// =====================================================================
// MOTOR TRANSMISSION MANIPULATION MATRIX
// =====================================================================
void applyHBridgeOutputs(int leftPWM, int rightPWM) {
  if (leftPWM >= 0) {
    ledcWrite(chLeftFwd, leftPWM); ledcWrite(chLeftRev, 0);
  } else {
    ledcWrite(chLeftFwd, 0); ledcWrite(chLeftRev, abs(leftPWM));
  }
  
  if (rightPWM >= 0) {
    ledcWrite(chRightFwd, rightPWM); ledcWrite(chRightRev, 0);
  } else {
    ledcWrite(chLeftFwd, 0); ledcWrite(chLeftRev, abs(leftPWM));
  }
}

void parseProportionalMovement(int cruiseSpeed, int steeringOffset) {
  int targetBase = map(cruiseSpeed, 0, 100, 0, 255);
  int targetSteer = map(steeringOffset, -100, 100, -130, 130);

  int leftTrack = targetBase + targetSteer;
  int rightTrack = targetBase - targetSteer;

  applyHBridgeOutputs(max(-255, min(255, leftTrack)), max(-255, min(255, rightTrack)));
}

// =====================================================================
// RECONFIGURED PHYSICAL ACTUATOR BEHAVIOR STATE ROUTINES
// =====================================================================
void executeDynamicGrip(int targetAdcLimit) {
  int currentAngle = 45;
  gripperServo.write(currentAngle);
  delay(400);

  smoothedForce = analogRead(FORCE_SENSOR_PIN); // Seed initial filter trace baseline

  while (currentAngle <= 135) {
    int rawIn = analogRead(FORCE_SENSOR_PIN);
    
    // EMA Smoothing filter equation implementation
    smoothedForce = (EMA_GAIN * rawIn) + ((1.0 - EMA_GAIN) * smoothedForce);

    if (smoothedForce > targetAdcLimit) {
      Serial.println("[CLAMP MATCH]: Structural object integrity safe boundary matched.");
      break;
    }

    currentAngle += 2;
    gripperServo.write(currentAngle);
    delay(45);
  }
}

void handleActionRoutine(const char* action, int forceLimit) {
  applyHBridgeOutputs(0, 0); // Execute hard stop safety constraint
  
  if (strcmp(action, "grab") == 0) {
    Serial.println("[EXECUTION RAMP]: Executing safe adaptive pickup sequence...");
    delay(1000);
    executeDynamicGrip(forceLimit);
  } 
  else if (strcmp(action, "sort") == 0) {
    Serial.println("[EXECUTION RAMP]: Running industrial package routing sortation pass...");
    delay(1000);
    executeDynamicGrip(forceLimit);
    
    // Perform differential split sorting execution movement
    Serial.println("[SORT ROUTINE]: Repositioning container payload to left depot...");
    applyHBridgeOutputs(-150, 150); // Spin left
    delay(1200);
    applyHBridgeOutputs(0, 0);
    gripperServo.write(45); // Open claw to release package
    delay(500);
    applyHBridgeOutputs(150, -150); // Return back to vector origin tracking trajectory
    delay(1200);
    applyHBridgeOutputs(0, 0);
  }
}

// =====================================================================
// NETWORK PIPELINE TOPIC ROUTING DELEGATES
// =====================================================================
void callback(char* topic, byte* payload, unsigned int length) {
  StaticJsonDocument<256> doc;
  if (deserializeJson(doc, payload, length)) return;

  if (strcmp(topic, topic_move) == 0) {
    if (doc["hold"]) {
      applyHBridgeOutputs(0, 0);
    } else {
      parseProportionalMovement(doc["speed"], doc["steer"]);
    }
  } 
  else if (strcmp(topic, topic_arm) == 0) {
    const char* action = doc["action"];
    int forceLimit = doc["force_limit"];
    handleActionRoutine(action, forceLimit);
  }
}

void reconnect() {
  while (!client.connected()) {
    if (client.connect("ESP32_Proportional_Chassis")) {
      client.subscribe(topic_move);
      client.subscribe(topic_arm);
    } else { delay(5000); }
  }
}

void setup() {
  Serial.begin(115200);

  ledcSetup(chLeftFwd, pwmFreq, pwmResolution);  ledcAttachPin(M_LEFT_Fwd, chLeftFwd);
  ledcSetup(chLeftRev, pwmFreq, pwmResolution);  ledcAttachPin(M_LEFT_Rev, chLeftRev);
  ledcSetup(chRightFwd, pwmFreq, pwmResolution); ledcAttachPin(M_RIGHT_Fwd, chRightFwd);
  ledcSetup(chRightRev, pwmFreq, pwmResolution); ledcAttachPin(M_RIGHT_Rev, chRightRev);

  gripperServo.attach(GRIPPER_SERVO_PIN);
  applyHBridgeOutputs(0, 0);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { delay(500); }

  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}

void loop() {
  if (!client.connected()) { reconnect(); }
  client.loop();
}
