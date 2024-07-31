#include <Arduino.h>
const int TARGET_POSITION = 6563;
const unsigned long REPORT_INTERVAL = 20000; // 20 seconds in milliseconds
const unsigned long RESET_INTERVAL = 300000; // 5 minutes in milliseconds

unsigned long lastReportTime = 0;
unsigned long lastResetTime = 0;
unsigned long totalRecords = 0;
unsigned long targetRecords = 0;

String lastData = "";  // To store the last received data string

void filterIncomingData(int currentPosition) {
    if (currentPosition == TARGET_POSITION) {
        targetRecords++;
        Serial.println("TARGET: " + lastData);
        delay(1); // Short delay after sending a message
    }
}

int getCurrentPosition() {
    if (Serial.available() > 0) {
        lastData = Serial.readStringUntil('\n');
        
        // Parse the CSV data
        int commaIndex = lastData.indexOf(',');
        if (commaIndex == -1) {
            return -1; // Invalid data
        }
        
        String positionStr = lastData.substring(commaIndex + 1);
        commaIndex = positionStr.indexOf(',');
        if (commaIndex == -1) {
            return -1; // Invalid data
        }
        
        int currentPosition = positionStr.substring(0, commaIndex).toInt();
        
        totalRecords++;  
        return currentPosition;
    }
    
    return -1; // No data available
}

void setup() {
    Serial.begin(460800);
    while (!Serial) {
        ; // wait for serial port to connect. Needed for native USB
    }
    Serial.println("Arduino ready");
}

void loop() {
    unsigned long currentTime = millis();
    int position = getCurrentPosition();
    filterIncomingData(position);
    
    // Report every REPORT_INTERVAL milliseconds
    if (currentTime - lastReportTime >= REPORT_INTERVAL) {
        Serial.print("Total records processed: ");
        Serial.print(totalRecords);
        Serial.print(", Target records: ");
        Serial.println(targetRecords);
        delay(1); // Short delay after sending a message
        
        lastReportTime = currentTime;
    }
    
    // Reset counters every RESET_INTERVAL milliseconds (5 minutes)
    if (currentTime - lastResetTime >= RESET_INTERVAL) {
        Serial.println("Resetting counters");
        totalRecords = 0;
        targetRecords = 0;
        lastResetTime = currentTime;
        delay(1); // Short delay after sending a message
    }
    
    // Clear the input buffer if it's getting full
    if (Serial.available() > 64) {
        while (Serial.available() > 0) {
            Serial.read();
        }
        Serial.println("Buffer cleared");
        delay(1); // Short delay after sending a message
    }
}