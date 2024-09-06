#include <Arduino.h>
// Define test values
int RadioControlData_control_mode = 2;
int RadioControlData_throttle_val = 1000;
int transmissionFullReversePos = 255;
float linear_x = -0.5;
float angular_z = 0.0;
bool safety_flag_LoRaRx = true;
bool safety_flag_cmd_vel = true;

// Define constants
const int transmissionNeutralPos = 266;
const int MIN_FORWARD_SERVO = 295;
const int transmissionFullForwardPos = 330;

// Define variables
int transmissionServoValue = 0;
int tranmissionLogicflag = 0;
float speed_setpoint = 0;
float actualSpeed = 0.3; // Example value

// Mock function for computeSpeedPID
float computeSpeedPID(float current_speed, float setpoint) {
  // Simple mock implementation
  Serial.println("In PID");
  return (setpoint - current_speed) / 2.0;
}

void setup() {
  Serial.begin(115200);
  delay(1000); // Wait for serial to initialize

  Serial.println("Starting test...");
  Serial.println("Initial values:");
  Serial.println("RadioControlData.control_mode = " + String(RadioControlData_control_mode));
  Serial.println("linear_x = " + String(linear_x));
  Serial.println("angular_z = " + String(angular_z));
  Serial.println("safety_flag_LoRaRx = " + String(safety_flag_LoRaRx));
  Serial.println("safety_flag_cmd_vel = " + String(safety_flag_cmd_vel));
  Serial.println();

  if (RadioControlData_control_mode == 1 && safety_flag_LoRaRx) {
      transmissionServoValue = map(RadioControlData_throttle_val, 0, 4095, transmissionFullReversePos, transmissionFullForwardPos);
      tranmissionLogicflag = 1;
  } else if (RadioControlData_control_mode == 2 && safety_flag_LoRaRx && safety_flag_cmd_vel) {
    if (linear_x > 0) { // Forward movement
      speed_setpoint = linear_x;
      float pid_output = computeSpeedPID(actualSpeed, speed_setpoint);
      transmissionServoValue = MIN_FORWARD_SERVO + map(pid_output * 100, 0, 100, 0, transmissionFullForwardPos - MIN_FORWARD_SERVO);
      tranmissionLogicflag = 2;
    } else if (linear_x == 0 && angular_z == 0) { // End of mission - stop
      transmissionServoValue = transmissionNeutralPos;    
      tranmissionLogicflag = 3;
    } else if (linear_x < 0) { // Reverse movement in auto mode - error state until programmed
      transmissionServoValue = transmissionNeutralPos;    
      tranmissionLogicflag = 4;
    } else { // Other case in auto mode
      transmissionServoValue = transmissionNeutralPos;   
      tranmissionLogicflag = 5;
    } // Close the else-if block      
  } else { // Saftey conditions not met or not in mode 1 or 2
    transmissionServoValue = transmissionNeutralPos;   
    tranmissionLogicflag = 6;
  }

  Serial.println("\nFinal values:");
  Serial.println("transmissionServoValue = " + String(transmissionServoValue));
  Serial.println("tranmissionLogicflag = " + String(tranmissionLogicflag));
  Serial.println("speed_setpoint = " + String(speed_setpoint));
}

void loop() {
  // Empty loop
}
