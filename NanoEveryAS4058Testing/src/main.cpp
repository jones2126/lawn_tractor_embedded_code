#include <Arduino.h>

const int QUEUE_SIZE = 4;
const unsigned long REPORT_INTERVAL = 20000; // 20 seconds in milliseconds
const unsigned long RESET_INTERVAL = 300000; // 5 minutes in milliseconds

const int ROLLOVER_THRESHOLD = 15000;
const int ROLLOVER_DROP = 8000;  // Minimum drop to consider it a rollover

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

// void filterIncomingData(int currentPosition) {
//     addToPositionQueue(currentPosition);
    
//     if (queueFull) {
//         uint8_t conditions = 0;
//         conditions |= (positionQueue[0] < positionQueue[1]) << 0;
//         conditions |= (positionQueue[1] < positionQueue[2]) << 1;
//         conditions |= ((positionQueue[2] > ROLLOVER_THRESHOLD) && 
//                        (positionQueue[2] - positionQueue[3] > ROLLOVER_DROP)) << 2;
//         conditions |= (positionQueue[0] > 12000) << 3;
//         conditions |= (positionQueue[1] > 13000) << 4;
//         conditions |= (positionQueue[2] > 14000) << 5;
//         conditions |= (positionQueue[3] < positionQueue[2]) << 6;

//         if (conditions == 0b01111111) {
//             targetRecords++;
//             Serial.println("TARGET: " + lastData);
//             Serial.print("Last 4 positions: ");
//             for (int i = 0; i < QUEUE_SIZE; i++) {
//                 int index = (queueIndex - QUEUE_SIZE + i + QUEUE_SIZE) % QUEUE_SIZE;
//                 Serial.print(positionQueue[index]);
//                 Serial.print(" ");
//             }
//             Serial.println();
//             delay(1); // Short delay after sending a message
//         }
//     }
// }

void filterIncomingData(int currentPosition) {
    addToPositionQueue(currentPosition);
    
    if (queueFull) {
        //This ensures that even as the queue wraps around, we always access the  
        // last 4 elements in the correct order, from oldest to newest.
        int pos0 = positionQueue[(queueIndex - 4 + QUEUE_SIZE) % QUEUE_SIZE];
        int pos1 = positionQueue[(queueIndex - 3 + QUEUE_SIZE) % QUEUE_SIZE];
        int pos2 = positionQueue[(queueIndex - 2 + QUEUE_SIZE) % QUEUE_SIZE];
        int pos3 = positionQueue[(queueIndex - 1 + QUEUE_SIZE) % QUEUE_SIZE];

        //If these readings were the incoming data: 14077, 14834, 15624, 6563, 810
        // When the value 6563 is read we know it is out of place.  The criteria below 
        // was crafted by reviewing actual data.  19 of 4482 records, about 0.05%, had 
        // an issue
        bool condition1 = pos1 > pos0;
        bool condition2 = pos2 > pos1;
        bool condition3 = pos3 < pos2; 
        bool condition4 = pos1 > 13000;
        bool condition5 = pos2 > 15000;
        bool condition6 = pos3 > 2000; 

        if (condition1 && condition2 && condition3 && condition4 && condition5 && condition6) {
            targetRecords++;
            Serial.print("Last 4 positions: ");
            Serial.print(" 0:"); Serial.print(pos0); Serial.print(" ");
            Serial.print(" 1:"); Serial.print(pos1); Serial.print(" ");
            Serial.print(" 2:"); Serial.print(pos2); Serial.print(" ");
            Serial.print(" 3:"); Serial.print(pos3); Serial.print(" ");
            Serial.println();
            delay(1); // Short delay after sending a message
        }
    }
}

// void filterIncomingData(int currentPosition) {
//     addToPositionQueue(currentPosition);
    
//     if (queueFull) {
//         bool condition1 = positionQueue[2] > positionQueue[1];
//         bool condition2 = positionQueue[1] > positionQueue[0];
//         bool condition3 = positionQueue[3] < positionQueue[2];
//         bool condition4 = positionQueue[1] > 13000;
//         bool condition5 = positionQueue[2] > 15000;
//         bool condition6 = positionQueue[3] > 2000;


//         if (condition1 && condition2 && condition3 && condition4 && condition5 && condition6) {
//         //if (condition0 && condition1 && condition3 && condition4 && condition5 && condition6) {          
//             targetRecords++;
//             //Serial.println("TARGET: " + lastData);
//             Serial.print("Last 4 positions: ");
//             // for (int i = 0; i < QUEUE_SIZE; i++) {
//             //     int index = (queueIndex - QUEUE_SIZE + i + QUEUE_SIZE) % QUEUE_SIZE;
//             //     Serial.print(positionQueue[index]);
//             //     Serial.print(" ");
//             // }
//             Serial.print(" 0:"); Serial.print(positionQueue[0]); Serial.print(" ");
//             Serial.print(" 1:"); Serial.print(positionQueue[1]); Serial.print(" ");
//             Serial.print(" 2:"); Serial.print(positionQueue[2]); Serial.print(" ");
//             Serial.print(" 3:"); Serial.print(positionQueue[3]); Serial.print(" ");
//             Serial.println();
//             delay(1); // Short delay after sending a message
//         }
//     }
// }

int getCurrentPosition() {
    if (Serial.available() > 0) {
        String positionStr = Serial.readStringUntil('\n');
        positionStr.trim();  // Remove any leading/trailing whitespace
        
        // Check if the string is not empty and contains only digits and possibly a decimal point
        if (positionStr.length() > 0 && positionStr.indexOf(',') == -1) {
            float currentPosition = positionStr.toFloat();
            int roundedPosition = round(currentPosition);  // Round to nearest integer
            
            totalRecords++;
            lastData = positionStr;  // Store the last received data if needed
            //Serial.println(roundedPosition);
            return roundedPosition;
        } else {
            return -1;  // Invalid data
        }
    }
    
    return -1;  // No data available
}
// int getCurrentPosition() {
//     if (Serial.available() > 0) {
//         lastData = Serial.readStringUntil('\n');
        
//         // Parse the CSV data
//         int commaIndex = lastData.indexOf(',');
//         if (commaIndex == -1) {
//             return -1; // Invalid data
//         }
        
//         String positionStr = lastData.substring(commaIndex + 1);
//         commaIndex = positionStr.indexOf(',');
//         if (commaIndex == -1) {
//             return -1; // Invalid data
//         }
        
//         int currentPosition = positionStr.substring(0, commaIndex).toInt();
        
//         totalRecords++;  
//         return currentPosition;
//     }
    
//     return -1; // No data available
// }

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