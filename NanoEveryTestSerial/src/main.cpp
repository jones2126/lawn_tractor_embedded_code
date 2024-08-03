#include <Arduino.h>

#define BUFFER_SIZE 1000

volatile int buffer[BUFFER_SIZE];
volatile int bufferIndex = 0;
volatile int totalRecords = 0;
volatile int processedRecords = 0;
unsigned long lastProcessTime = 0;
const unsigned long processInterval = 1000 / 10; // Interval in milliseconds (10 Hz)

// Function to read the serial port and get the current position
int getCurrentPosition() {
    if (Serial.available() > 0) {
        String positionStr = Serial.readStringUntil('\n');
        positionStr.trim();
        
        if (positionStr.length() > 0 && positionStr.indexOf(',') == -1) {
            int currentPosition = positionStr.toInt();
            totalRecords++;
            return currentPosition;
        }
    }
    
    return -1; // No valid data available
}

void setup() {
  Serial.begin(460800); // Initialize serial communication
}

void loop() {
  unsigned long currentTime = millis();

  // Continuously check for serial data and add to buffer
  int value = getCurrentPosition();
  if (value != -1) {
    buffer[bufferIndex] = value;
    bufferIndex = (bufferIndex + 1) % BUFFER_SIZE; // Circular buffer
  }

  // Process data at 10 Hz
  if (currentTime - lastProcessTime >= processInterval) {
    lastProcessTime = currentTime;

    // Example processing: print the oldest value in the buffer
    int processIndex = (bufferIndex + 1) % BUFFER_SIZE; // Get the next value to process
    Serial.print("Processing value: ");
    Serial.println(buffer[processIndex]);
    processedRecords++;

    // Print buffer size and record counts
    int bufferSize = (bufferIndex - processIndex + BUFFER_SIZE) % BUFFER_SIZE;
    Serial.print("processIndex: ");
    Serial.println(processIndex);
    Serial.print("Total records sent to buffer: ");
    Serial.println(totalRecords);
    Serial.print("Total records processed: ");
    Serial.println(processedRecords);
  }
}
