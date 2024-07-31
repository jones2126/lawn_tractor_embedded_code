#include <Arduino.h>

const int TARGET_POSITION = 6563;
const int QUEUE_SIZE = 4; // Changed to 4
const int DATA_SIZE = 30; // Number of test data elements
const float TARGET_RATE = 2.0; // Target rate in Hz

unsigned long totalRecords = 0;
unsigned long targetRecords = 0;

// Define these constants at the top of your file, outside any function
const int ROLLOVER_THRESHOLD = 15000;
const int POST_ROLLOVER_MIN = 0;
const int POST_ROLLOVER_MAX = 2000;
const int ROLLOVER_DROP = 8000;  // Minimum drop to consider it a rollover

// Test data array
const int testData[DATA_SIZE] = {
    13434, 14087, 14749, 15422, 16127, 445, 1150, 1875, 2592, 3334,
    4053, 4720, 5402, 6075, 6802, 7528, 8262, 8994, 9715, 10439,
    11207, 11957, 12622, 13337, 14077, 14834, 15624, 6563, 810, 1605
};
int dataIndex = 0;

// Circular buffer for storing the last four positions
int positionQueue[QUEUE_SIZE];
int queueIndex = 0;
bool queueFull = false;

void addToPositionQueue(int position) {
    positionQueue[queueIndex] = position;
    queueIndex = (queueIndex + 1) % QUEUE_SIZE;
    if (!queueFull && queueIndex == 0) {
        queueFull = true;
    }
}

void printPositionQueue(int currentPosition) {
    Serial.print("Current position: ");
    Serial.print(currentPosition);
    Serial.print(" | Last 4 positions: "); // Changed to 4
    if (!queueFull) {
        for (int i = 0; i < queueIndex; i++) {
            Serial.print("(");
            Serial.print(i);
            Serial.print(":");
            Serial.print(positionQueue[i]);
            Serial.print(") ");
        }
    } else {
        for (int i = 0; i < QUEUE_SIZE; i++) {
            int index = (queueIndex - QUEUE_SIZE + i + QUEUE_SIZE) % QUEUE_SIZE;
            Serial.print("(");
            Serial.print(i);
            Serial.print(":");
            Serial.print(positionQueue[index]);
            Serial.print(") ");
        }
    }
    Serial.println();
}

// void filterIncomingData(int currentPosition) {
//     addToPositionQueue(currentPosition);
//     printPositionQueue(currentPosition);
    
//     if (queueFull) {
//         Serial.println("Checking conditions:");
//         bool condition1 = positionQueue[0] < positionQueue[1];
//         bool condition2 = positionQueue[1] < positionQueue[2];
//         bool condition3 = positionQueue[3] < positionQueue[2];
//         bool condition4 = positionQueue[0] > 12000;
//         bool condition5 = positionQueue[1] > 13000;
//         bool condition6 = positionQueue[2] > 14000;
//         bool condition7 = positionQueue[3] > 2000;

//         Serial.print("positionQueue[0] < positionQueue[1]: ");
//         Serial.println(condition1);
//         Serial.print("positionQueue[1] < positionQueue[2]: ");
//         Serial.println(condition2);
//         Serial.print("positionQueue[3] < positionQueue[2]: ");
//         Serial.println(condition3);
//         Serial.print("positionQueue[0] > 12000: ");
//         Serial.println(condition4);
//         Serial.print("positionQueue[1] > 13000: ");
//         Serial.println(condition5);
//         Serial.print("positionQueue[2] > 14000: ");
//         Serial.println(condition6);
//         Serial.print("positionQueue[3] > 2000: ");
//         Serial.println(condition7);

//         if (condition1 && condition2 && condition3 && 
//             condition4 && condition5 && condition6 && condition7) {
//             targetRecords++;
//             Serial.println("Target condition met!");
//             printPositionQueue(currentPosition);
//         } else {
//             Serial.println("Target condition not met.");
//         }
//     }
// }

void filterIncomingData(int currentPosition) {
    addToPositionQueue(currentPosition);
    printPositionQueue(currentPosition);
    
    if (queueFull) {
        uint8_t conditions = 0;
        conditions |= (positionQueue[0] < positionQueue[1]) << 0;
        conditions |= (positionQueue[1] < positionQueue[2]) << 1;
        conditions |= ((positionQueue[2] > ROLLOVER_THRESHOLD) && 
                       (positionQueue[2] - positionQueue[3] > ROLLOVER_DROP)) << 2;
        conditions |= (positionQueue[0] > 12000) << 3;
        conditions |= (positionQueue[1] > 13000) << 4;
        conditions |= (positionQueue[2] > 14000) << 5;
        conditions |= (positionQueue[3] < positionQueue[2]) << 6;  // Changed this condition

        Serial.println("Checking conditions:");
        for (int i = 0; i < 7; i++) {
            Serial.print("Condition ");
            Serial.print(i + 1);
            Serial.print(": ");
            Serial.println((conditions & (1 << i)) ? "1" : "0");
        }

        if (conditions == 0b01111111) {
            targetRecords++;
            Serial.println("Target condition met!");
            printPositionQueue(currentPosition);
        } else {
            Serial.println("Target condition not met.");
        }
    }
}

int getCurrentPosition() {
    if (dataIndex < DATA_SIZE) {
        int currentPosition = testData[dataIndex];
        dataIndex++;
        totalRecords++;
        return currentPosition;
    }
    return -1; // Indicates end of data
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
    static unsigned long lastProcessTime = 0;
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - lastProcessTime;
    
    // Process data at the target rate
    if (elapsedTime >= (1000 / TARGET_RATE)) {
        int position = getCurrentPosition();
        if (position == -1) {
            // End of data reached
            Serial.println("All data processed");
            Serial.print("Total records processed: ");
            Serial.println(totalRecords);
            Serial.print("Target records found: ");
            Serial.println(targetRecords);
            printPositionQueue(positionQueue[queueIndex - 1]); 
            while(1); // Stop execution
        } else {
            filterIncomingData(position);
        }
        lastProcessTime = currentTime;
    }
}