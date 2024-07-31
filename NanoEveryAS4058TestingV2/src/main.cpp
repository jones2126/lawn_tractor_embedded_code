#include <Arduino.h>

const int QUEUE_SIZE = 4;
const unsigned long REPORT_INTERVAL = 20000; // 20 seconds in milliseconds
const unsigned long RESET_INTERVAL = 300000; // 5 minutes in milliseconds

const int ROLLOVER_THRESHOLD = 15000;
const int ROLLOVER_DROP = 8000;  // Minimum drop to consider it a rollover
const int ENCODER_MAX = 16384;  // 2^14, maximum value + 1 for a 14-bit encoder

unsigned long lastReportTime = 0;
unsigned long lastResetTime = 0;
unsigned long totalRecords = 0;
unsigned long targetRecords = 0;

int positionQueue[QUEUE_SIZE];
int queueIndex = 0;
bool queueFull = false;

String lastData = "";  // To store the last received data string

void addToPositionQueue(int position) {
    positionQueue[queueIndex] = position;
    queueIndex = (queueIndex + 1) % QUEUE_SIZE;
    if (!queueFull && queueIndex == 0) {
        queueFull = true;
    }
}

int calculateAverageDelta(int pos0, int pos1, int pos2) {
    int delta1 = (pos1 - pos0 + ENCODER_MAX) % ENCODER_MAX;
    int delta2 = (pos2 - pos1 + ENCODER_MAX) % ENCODER_MAX;
    return (delta1 + delta2) / 2;
}

int calculateExpectedPos3(int pos2, int avgDelta) {
    return (pos2 + avgDelta) % ENCODER_MAX;
}

void filterIncomingData(int currentPosition) {
    addToPositionQueue(currentPosition);
    
    if (queueFull) {
        int pos0 = positionQueue[(queueIndex - 4 + QUEUE_SIZE) % QUEUE_SIZE];
        int pos1 = positionQueue[(queueIndex - 3 + QUEUE_SIZE) % QUEUE_SIZE];
        int pos2 = positionQueue[(queueIndex - 2 + QUEUE_SIZE) % QUEUE_SIZE];
        int pos3 = positionQueue[(queueIndex - 1 + QUEUE_SIZE) % QUEUE_SIZE];

        bool condition1 = pos1 > pos0;
        bool condition2 = pos2 > pos1;
        bool condition3 = pos3 < pos2; 
        bool condition4 = pos1 > 13000;
        bool condition5 = pos2 > 15000;
        bool condition6 = pos3 > 2000; 

        if (condition1 && condition2 && condition3 && condition4 && condition5 && condition6) {
            targetRecords++;
            
            int avgDelta = calculateAverageDelta(pos0, pos1, pos2);
            int expectedPos3 = calculateExpectedPos3(pos2, avgDelta);
            
            // Replace pos3 in the queue with the expected value
            positionQueue[(queueIndex - 1 + QUEUE_SIZE) % QUEUE_SIZE] = expectedPos3;
            
            Serial.print("Anomaly detected. Last 4 positions: ");
            Serial.print(" 0:"); Serial.print(pos0); Serial.print(" ");
            Serial.print(" 1:"); Serial.print(pos1); Serial.print(" ");
            Serial.print(" 2:"); Serial.print(pos2); Serial.print(" ");
            Serial.print(" 3 (Original):"); Serial.print(pos3); Serial.print(" ");
            Serial.print("Avg Delta: "); Serial.print(avgDelta);
            Serial.print(" 3 (Corrected):"); Serial.println(expectedPos3);
            delay(1); // Short delay after sending a message
        }
    }
}

int getCurrentPosition() {
    if (Serial.available() > 0) {
        String positionStr = Serial.readStringUntil('\n');
        positionStr.trim();  // Remove any leading/trailing whitespace
        
        if (positionStr.length() > 0 && positionStr.indexOf(',') == -1) {
            float currentPosition = positionStr.toFloat();
            int roundedPosition = round(currentPosition);  // Round to nearest integer
            
            totalRecords++;
            lastData = positionStr;  // Store the last received data if needed
            return roundedPosition;
        } else {
            return -1;  // Invalid data
        }
    }
    
    return -1;  // No data available
}

void setup() {
    Serial.begin(460800);
    while (!Serial) {
        ; // wait for serial port to connect. Needed for native USB
    }
    Serial.println("Arduino ready");
    
    // Initialize position queue
    for (int i = 0; i < QUEUE_SIZE; i++) {
        positionQueue[i] = 0;
    }
}

void loop() {
    unsigned long currentTime = millis();
    int position = getCurrentPosition();
    if (position != -1) {
        filterIncomingData(position);
    }
    
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