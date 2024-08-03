#include <Arduino.h>

const unsigned long REPORT_INTERVAL = 20000; // 20 seconds in milliseconds
const float G_FORCE_THRESHOLD = 3.0; // G-force threshold for anomaly detection
const float TIME_STEP = 0.1; // 10 Hz sensor (0.1 seconds between readings)

unsigned long lastReportTime = 0;
unsigned long totalRecords = 0;
unsigned long anomalyCount = 0;

float lastSpeed = 0;
String inputBuffer = "";

// Function prototype
void processLine(String line);

void setup() {
    Serial.begin(460800);
    while (!Serial) {
        ; // wait for serial port to connect. Needed for native USB
    }
    Serial.println("Arduino ready");
}

void loop() {
    unsigned long currentTime = millis();

    // Read incoming data
    while (Serial.available() > 0) {
        char inChar = Serial.read();
        if (inChar == '\n') {
            processLine(inputBuffer);
            inputBuffer = "";
        } else {
            inputBuffer += inChar;
        }
    }

    // Report every REPORT_INTERVAL milliseconds
    if (currentTime - lastReportTime >= REPORT_INTERVAL) {
        Serial.print("Total records processed: ");
        Serial.print(totalRecords);
        Serial.print(", Anomalies detected: ");
        Serial.println(anomalyCount);
        lastReportTime = currentTime;
    }
}

void processLine(String line) {
    line.trim();
    if (line.length() > 0) {
        int commaIndex = line.indexOf(',');
        if (commaIndex != -1) {
            // Parse timestamp and speed from the data
            // unsigned long timestamp = line.substring(0, commaIndex).toInt();
            float speed = line.substring(commaIndex + 1).toFloat();

            // Calculate G-force
            if (totalRecords > 0) {
                float acceleration = (speed - lastSpeed) / TIME_STEP;
                float gForce = acceleration / 9.81;

                // Check for anomaly
                if (abs(gForce) > G_FORCE_THRESHOLD) {
                    anomalyCount++;
                    Serial.print("Anomaly detected - Record: ");
                    Serial.print(totalRecords);
                    Serial.print(", Speed: ");
                    Serial.print(speed, 2);
                    Serial.print(", G-force: ");
                    Serial.println(gForce, 2);
                }
            }

            lastSpeed = speed;
            totalRecords++;
        }
    }
}