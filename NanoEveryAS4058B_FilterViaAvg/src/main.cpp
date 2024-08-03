#include <Arduino.h>

const int QUEUE_SIZE = 10;
const unsigned long REPORT_INTERVAL = 120000; // 2 minutes in milliseconds
const unsigned long RESET_INTERVAL = 300000; // 5 minutes in milliseconds

const float MAX_SPEED = 2.0; // Maximum speed in m/s (adjust as needed)
const float G_FORCE_THRESHOLD = 2.0; // Maximum G-force (positive or negative)
const unsigned long OUTPUT_INTERVAL = 100; // Time between readings in milliseconds (10 Hz)

const int ENCODER_MAX = 16384; // 2^14, maximum value + 1 for a 14-bit encoder
const int TICKS_PER_REVOLUTION = 16384; // Assuming this is correct for your encoder
const float WHEEL_CIRCUMFERENCE = 1.59593; // in meters

// Define a struct to hold anomaly information
struct Anomaly {
    unsigned long index;
    float gForce;
};

const int MAX_ANOMALIES = 200; // Maximum number of anomalies to store
Anomaly anomalies[MAX_ANOMALIES];
int anomalyCount = 0;

unsigned long lastReportTime = 0;
unsigned long lastResetTime = 0;
unsigned long totalRecords = 0;
unsigned long filteredRecords = 0;

int positionQueue[QUEUE_SIZE];
float speedQueue[QUEUE_SIZE];
int queueIndex = 0;
bool queueFull = false;

float lastSpeed = 0;

float calculateSpeed(int pos1, int pos2) {
    int delta = pos2 - pos1;
    if (delta > ENCODER_MAX / 2) delta -= ENCODER_MAX;
    if (delta < -ENCODER_MAX / 2) delta += ENCODER_MAX;
    
    // Calculate speed in meters per second
    float rotations_per_second = (float)delta / TICKS_PER_REVOLUTION * (1000.0 / OUTPUT_INTERVAL);
    return rotations_per_second * WHEEL_CIRCUMFERENCE;
}

float filterIncomingData(int currentPosition) {
    static unsigned long recordIndex = 0;
    recordIndex++;

    if (!queueFull) {
        positionQueue[queueIndex] = currentPosition;
        if (queueIndex > 0) {
            float speed = calculateSpeed(positionQueue[queueIndex-1], currentPosition);
            speedQueue[queueIndex-1] = speed;
        }
        queueIndex++;
        if (queueIndex == QUEUE_SIZE) {
            queueFull = true;
        }
        return currentPosition;
    }
    
    int pos1 = positionQueue[(queueIndex-1+QUEUE_SIZE) % QUEUE_SIZE];
    int pos2 = currentPosition;
    float currentSpeed = calculateSpeed(pos1, pos2);
    float gForce = (currentSpeed - lastSpeed) / (OUTPUT_INTERVAL / 1000.0) / 9.81; // Convert to G-force
    
    bool isAnomaly = false;
    
    // if (abs(currentSpeed) > MAX_SPEED) {
    //     isAnomaly = true;
    // }
    
    if (abs(gForce) > G_FORCE_THRESHOLD) {
        isAnomaly = true;
        // Serial.print("G-force anomaly detected. Record index: ");
        // Serial.print(recordIndex);
        // Serial.print(", G-force: ");
        // Serial.println(gForce, 2);  // Print G-force with 2 decimal places

        // Store the anomaly if there's space
        if (anomalyCount < MAX_ANOMALIES) {
            anomalies[anomalyCount].index = recordIndex;
            anomalies[anomalyCount].gForce = gForce;
            anomalyCount++;
        }
    }
    
    if (isAnomaly) {
        filteredRecords++;

        // Use the average of recent valid speeds to estimate the new position
        float avgSpeed = 0;
        for (int i = 0; i < QUEUE_SIZE-1; i++) {
            avgSpeed += speedQueue[i];
        }
        avgSpeed /= (QUEUE_SIZE-1);
        
        float rotations = avgSpeed * (OUTPUT_INTERVAL / 1000.0) / WHEEL_CIRCUMFERENCE;
        int estimatedDeltaTicks = round(rotations * TICKS_PER_REVOLUTION);
        int estimatedPosition = (positionQueue[(queueIndex-1+QUEUE_SIZE) % QUEUE_SIZE] + estimatedDeltaTicks) % ENCODER_MAX;
        
        positionQueue[queueIndex] = estimatedPosition;
        speedQueue[(queueIndex-1+QUEUE_SIZE) % QUEUE_SIZE] = avgSpeed;
        
        currentPosition = estimatedPosition;
        currentSpeed = avgSpeed;
    } else {
        positionQueue[queueIndex] = currentPosition;
        speedQueue[(queueIndex-1+QUEUE_SIZE) % QUEUE_SIZE] = currentSpeed;
    }
    
    queueIndex = (queueIndex + 1) % QUEUE_SIZE;
    lastSpeed = currentSpeed;
    
    return currentPosition;
}

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
    Serial.begin(460800);
    while (!Serial) {
        ; // wait for serial port to connect. Needed for native USB
    }
    Serial.println("Arduino ready");
    
    // Initialize queues
    for (int i = 0; i < QUEUE_SIZE; i++) {
        positionQueue[i] = 0;
        speedQueue[i] = 0;
    }
}

void loop() {
    unsigned long currentTime = millis();
    int position = getCurrentPosition();
    if (position != -1) {
        int filteredPosition = round(filterIncomingData(position));
        // Serial.print("Original: ");
        // Serial.print(position);
        // Serial.print(", Filtered: ");
        // Serial.println(filteredPosition);
    }
    
    // Report every REPORT_INTERVAL milliseconds
    if (currentTime - lastReportTime >= REPORT_INTERVAL) {
        Serial.print("Total records processed: ");
        Serial.print(totalRecords);
        Serial.print(", Filtered records: ");
        Serial.println(filteredRecords);
        
        // Print out stored anomalies
        Serial.println("Anomalies detected:");
        for (int i = 0; i < anomalyCount; i++) {
            Serial.print("Index: ");
            Serial.print(anomalies[i].index);
            Serial.print(", G-force: ");
            Serial.println(anomalies[i].gForce, 2);
        }
        Serial.println("End of anomalies");
        
        // Reset anomaly count
        anomalyCount = 0;
        
        delay(1);
        lastReportTime = currentTime;
    }
    
    // Reset counters every RESET_INTERVAL milliseconds (5 minutes)
    if (currentTime - lastResetTime >= RESET_INTERVAL) {
        Serial.println("Resetting counters");
        totalRecords = 0;
        filteredRecords = 0;
        lastResetTime = currentTime;
        delay(1);
    }
    
    // Clear the input buffer if it's getting full
    if (Serial.available() > 64) {
        while (Serial.available() > 0) {
            Serial.read();
        }
        Serial.println("Buffer cleared");
        delay(1);
    }
}