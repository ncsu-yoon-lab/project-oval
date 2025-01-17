#include "PPMEncoder.h"

// Define the PPM pin
#define OUTPUT_PIN 10
#define CHANNEL_COUNT 1  // Single channel for VESC
#define BAUD_RATE 115200

// Define throttle limits (in microseconds)
#define MIN_THROTTLE 1000  // 1ms pulse
#define CENTER_THROTTLE 1500  // 1.5ms pulse
#define MAX_THROTTLE 2000  // 2ms pulse

// Test pattern parameters
#define STEP_SIZE 10       // How much to change the pulse width each step
#define STEP_DELAY 20      // Time between steps (ms)

int targetThrottle = CENTER_THROTTLE;
unsigned long lastUpdateTime = 0;
const int updateInterval = 100;  // Send updates every 100ms

void setup() {
  // Initialize PPM output
  ppmEncoder.begin(OUTPUT_PIN);
  
  // Initialize serial communication
  Serial.begin(BAUD_RATE);
  
  // Start at neutral position
  ppmEncoder.setChannel(0, CENTER_THROTTLE);
}

void handleSerialCommands() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    // Parse command
    if (command.startsWith("MODE")) {
      // MODE,0 for auto, MODE,1 for manual
      int mode = command.substring(5).toInt();
      currentMode = mode;
      Serial.print("MODE_CHANGED,");
      Serial.println(mode);
    }
  }
}

void sendUpdate(int currentThrottle) {
  // Send current throttle value to Pi
  if (millis() - lastUpdateTime >= updateInterval) {
    Serial.print("STATUS,");
    Serial.print(currentMode);
    Serial.print(",");
    Serial.println(currentThrottle);
    lastUpdateTime = millis();
  }
}

void manualMode() {
  static int currentThrottle = CENTER_THROTTLE;
  
  // Smoothly approach target throttle
  if (currentThrottle < targetThrottle) {
    currentThrottle += min(STEP_SIZE, targetThrottle - currentThrottle);
  } else if (currentThrottle > targetThrottle) {
    currentThrottle -= min(STEP_SIZE, currentThrottle - targetThrottle);
  }
  
  ppmEncoder.setChannel(0, currentThrottle);
  sendUpdate(currentThrottle);
  delay(STEP_DELAY);
}

void loop() {
  // Check for commands from Pi
  handleSerialCommands();
  
  manualMode();
  
}
