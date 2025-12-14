#include <WiFi.h>
#include <WebServer.h>

// L298N Motor Driver Pins - MOVEMENT
#define IN1 15
#define IN2 2
#define IN3 14
#define IN4 27
#define ENA 25  // Left motor PWM Speed Control
#define ENB 26  // Right motor PWM Speed Control

// L298N Motor Driver Pins - COLLECTING MECHANISM (No PWM needed)
#define IN5 18
#define IN6 19
#define IN7 32
#define IN8 33
// Note: Collecting motors run at full speed, no PWM enable pins needed

// HC-SR04 Ultrasonic Sensor Pins
#define TRIG_LEFT 21
#define ECHO_LEFT 22
#define TRIG_RIGHT 5
#define ECHO_RIGHT 4

// Buzzer Pin
#define BUZZER_PIN 23

// --- PWM Configuration ---
#define PWM_FREQ 5000       // 5kHz is a good frequency for L298N motors
#define PWM_RESOLUTION 8    // 8-bit resolution = values from 0-255

// --- Obstacle Detection Configuration ---
#define OBSTACLE_DISTANCE_CM 30  // Detection threshold in cm
#define WAIT_TIME_MS 10000       // 10 seconds wait time
#define TURN_TIME_MS 4000        // 4 seconds turn time (approximately 90 degrees)
#define FORWARD_TIME_MS 4000     // 4 seconds forward movement after turn
#define TURN_SPEED 250           // Speed for turning

WebServer server(80);

// WiFi Configuration
const char* ssid = "motoedge50fusion_2315";
const char* password = "12345678";

// Static IP Configuration
IPAddress staticIP(172, 31, 43, 153);
IPAddress gateway(172, 31, 43, 221);
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(8, 8, 8, 8);

// --- State Variables ---
unsigned long lastCommandTime = 0;
const unsigned long TIMEOUT_MS = 1000;
bool motorsActive = false;
unsigned long reachDuration = 500;
unsigned long finalReachDuration = 3000;  // 3 seconds for the final collecting push
bool isCollecting = false;

// --- Obstacle Avoidance State ---
bool obstacleDetected = false;
unsigned long obstacleDetectedTime = 0;
bool isAvoidingObstacle = false;

// --- PWM Motor Control Functions ---
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  digitalWrite(IN1, leftSpeed > 0 ? HIGH : LOW);
  digitalWrite(IN2, leftSpeed < 0 ? HIGH : LOW);
  ledcWrite(ENA, abs(leftSpeed));

  digitalWrite(IN3, rightSpeed > 0 ? HIGH : LOW);
  digitalWrite(IN4, rightSpeed < 0 ? HIGH : LOW);
  ledcWrite(ENB, abs(rightSpeed));

  if (leftSpeed != 0 || rightSpeed != 0) {
    motorsActive = true;
    lastCommandTime = millis();
  } else {
    motorsActive = false;
  }
}

void stopMotors() {
  setMotorSpeeds(0, 0);
  if (motorsActive) {
    Serial.println("MOTORS STOPPED");
    motorsActive = false;
  }
}

void simpleMoveForward() { setMotorSpeeds(255, 255); }
void simpleMoveBackward() { setMotorSpeeds(-255, -255); }
void simpleTurnLeft() { setMotorSpeeds(-TURN_SPEED, TURN_SPEED); }
void simpleTurnRight() { setMotorSpeeds(TURN_SPEED, -TURN_SPEED); }

// --- Buzzer Functions ---
void beepOnOff() {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(200);
  digitalWrite(BUZZER_PIN, LOW);
  delay(200);
  digitalWrite(BUZZER_PIN, HIGH);
  delay(200);
  digitalWrite(BUZZER_PIN, LOW);
}

// --- Ultrasonic Sensor Functions ---
float getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duration = pulseIn(echoPin, HIGH, 30000); // 30ms timeout
  if (duration == 0) return 999; // No echo received
  
  float distance = duration * 0.034 / 2; // Calculate distance in cm
  return distance;
}

// --- Collector & Reach Functions ---
void startCollecting() {
  // Start both collecting motors forward at full speed
  digitalWrite(IN5, HIGH);
  digitalWrite(IN6, LOW);
  digitalWrite(IN7, HIGH);
  digitalWrite(IN8, LOW);
  isCollecting = true;
  Serial.println("COLLECTING STARTED");
}

void stopCollecting() {
  // Stop both collecting motors
  digitalWrite(IN5, LOW);
  digitalWrite(IN6, LOW);
  digitalWrite(IN7, LOW);
  digitalWrite(IN8, LOW);
  isCollecting = false;
  Serial.println("COLLECTING STOPPED");
}

void executeReach() {
  Serial.printf("REACH: Moving forward for %lu ms\n", reachDuration);
  setMotorSpeeds(255, 255);
  delay(reachDuration);
  stopMotors();
  Serial.println("REACH: Complete");
}

void executeFinalReach() {
  Serial.printf("FINAL REACH: Moving forward at full speed for %lu ms\n", finalReachDuration);
  setMotorSpeeds(255, 255);
  delay(finalReachDuration);
  stopMotors();
  Serial.println("FINAL REACH: Complete - Shuttlecock collected");
}

// --- Obstacle Detection and Avoidance ---
void checkObstacles() {
  if (isAvoidingObstacle) return; // Skip checking if already avoiding
  
  float leftDistance = getDistance(TRIG_LEFT, ECHO_LEFT);
  float rightDistance = getDistance(TRIG_RIGHT, ECHO_RIGHT);
  
  Serial.printf("Distances - Left: %.1f cm, Right: %.1f cm\n", leftDistance, rightDistance);
  
  bool leftObstacle = leftDistance < OBSTACLE_DISTANCE_CM;
  bool rightObstacle = rightDistance < OBSTACLE_DISTANCE_CM;
  
  if (leftObstacle || rightObstacle) {
    if (!obstacleDetected) {
      // First detection
      Serial.println("OBSTACLE DETECTED!");
      stopMotors();
      beepOnOff(); // On-off beep
      obstacleDetected = true;
      obstacleDetectedTime = millis();
    } else {
      // Check if 10 seconds have passed
      if (millis() - obstacleDetectedTime >= WAIT_TIME_MS) {
        Serial.println("Obstacle still present after 10s - Starting avoidance");
        isAvoidingObstacle = true;
        
        // Decide turn direction based on which sensor detects closer object
        if (leftObstacle && !rightObstacle) {
          // Left sensor detects, turn right
          Serial.println("Turning RIGHT for 4 seconds (left obstacle)");
          simpleTurnRight();
          delay(TURN_TIME_MS);
          stopMotors();
        } else if (rightObstacle && !leftObstacle) {
          // Right sensor detects, turn left
          Serial.println("Turning LEFT for 4 seconds (right obstacle)");
          simpleTurnLeft();
          delay(TURN_TIME_MS);
          stopMotors();
        } else {
          // Both detect, choose based on which is closer
          if (leftDistance < rightDistance) {
            Serial.println("Turning RIGHT for 4 seconds (left closer)");
            simpleTurnRight();
            delay(TURN_TIME_MS);
            stopMotors();
          } else {
            Serial.println("Turning LEFT for 4 seconds (right closer)");
            simpleTurnLeft();
            delay(TURN_TIME_MS);
            stopMotors();
          }
        }
        
        // Move forward for 4 seconds
        Serial.println("Moving forward for 4 seconds");
        simpleMoveForward();
        delay(FORWARD_TIME_MS);
        stopMotors();
        
        // Reset states
        obstacleDetected = false;
        isAvoidingObstacle = false;
        Serial.println("Avoidance complete - Resuming normal operation");
      }
    }
  } else {
    // No obstacle detected, reset flag
    if (obstacleDetected && !isAvoidingObstacle) {
      Serial.println("Obstacle cleared");
      obstacleDetected = false;
    }
  }
}

// --- Web Server Setup & Handlers ---
void connectToWiFi() {
  Serial.println("Connecting to WiFi...");
  WiFi.config(staticIP, gateway, subnet, primaryDNS);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  Serial.print("Motor Board IP Address: ");
  Serial.println(WiFi.localIP());
}

void handleMove() {
  String direction = server.arg("dir");
  Serial.printf("Simple Move Command: %s\n", direction.c_str());
  if (direction == "forward") simpleMoveForward();
  else if (direction == "reverse") simpleMoveBackward();
  else if (direction == "left") simpleTurnLeft();
  else if (direction == "right") simpleTurnRight();
  else { server.send(400, "text/plain", "Invalid dir"); return; }
  lastCommandTime = millis();
  server.send(200, "text/plain", "OK");
}

void handleTurn() {
  if (!server.hasArg("dir") || !server.hasArg("speed")) {
    server.send(400, "text/plain", "Bad Request: Missing dir or speed");
    return;
  }
  String direction = server.arg("dir");
  int speed = server.arg("speed").toInt();
  speed = constrain(speed, 0, 255);

  Serial.printf("Turn Command: %s at speed %d\n", direction.c_str(), speed);

  if (direction == "left") {
    setMotorSpeeds(-speed, speed);
  } else if (direction == "right") {
    setMotorSpeeds(speed, -speed);
  } else {
    server.send(400, "text/plain", "Invalid turn direction");
    return;
  }
  lastCommandTime = millis();
  server.send(200, "text/plain", "OK");
}

void handleStop() {
  stopMotors();
  server.send(200, "text/plain", "STOP_OK");
}

void handleReach() { executeReach(); server.send(200, "text/plain", "REACH_OK"); }
void handleFinalReach() { executeFinalReach(); server.send(200, "text/plain", "FINAL_REACH_OK"); }
void handleCollectingOn() { startCollecting(); server.send(200, "text/plain", "COLLECT_ON_OK"); }
void handleCollectingOff() { stopCollecting(); server.send(200, "text/plain", "COLLECT_OFF_OK"); }

void setup() {
  Serial.begin(115200);

  // Motor pins
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(IN5, OUTPUT); pinMode(IN6, OUTPUT);
  pinMode(IN7, OUTPUT); pinMode(IN8, OUTPUT);
  
  // Ultrasonic sensor pins
  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);
  
  // Buzzer pin
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  // PWM Setup (only for movement motors)
  ledcAttach(ENA, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(ENB, PWM_FREQ, PWM_RESOLUTION);
  
  stopMotors();
  stopCollecting();

  connectToWiFi();

  // Server endpoints
  server.on("/move", HTTP_GET, handleMove);
  server.on("/turn", HTTP_GET, handleTurn);
  server.on("/stop", HTTP_GET, handleStop);
  server.on("/reach", HTTP_GET, handleReach);
  server.on("/final_reach", HTTP_GET, handleFinalReach);
  server.on("/on_collecting", HTTP_GET, handleCollectingOn);
  server.on("/off_collecting", HTTP_GET, handleCollectingOff);

  server.begin();
  Serial.println("HTTP server started.");
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi Disconnected. Attempting to reconnect...");
    connectToWiFi();
  }
  server.handleClient();

  // Check for obstacles continuously
  checkObstacles();

  // Safety timeout: If no commands are received, stop the motors
  if (motorsActive && !isAvoidingObstacle && (millis() - lastCommandTime > TIMEOUT_MS)) {
    Serial.println("Auto-stop triggered due to command timeout.");
    stopMotors();
  }
  
  delay(50); // Small delay to prevent loop from running too fast
}