#include <Arduino.h>
unsigned long lastReportTime = 0;
unsigned long reportInterval = 10000;
int loopCounter = 0;
int messagesReceived = 0;
char warn_msg[100];
int RTS_received = 0;  // Counter for RTS messages
int CTS_received = 0;  // Counter for CTS messages
String data;
void clearInputBuffer() {
  while (Serial.available() > 0) {
    Serial.read();
  }
}

void send_with_handshake(String message) {
  Serial.println("in handshake"); 
  const int max_retries = 3; 
  int retries = 0;
  while (retries < max_retries) {
    Serial.println("RTS");
    //clearInputBuffer();
    unsigned long start_time = millis();
    unsigned long cts_timeout = 2000;
    bool cts_received = false;
    while (!cts_received && millis() - start_time < cts_timeout) {
      if (Serial.available() > 0) {
        data = Serial.readStringUntil('\n');
        data.trim();
        //Serial.println("data trimmed (" + data + ")");
        /*Serial.print("hex (");
        // Print each character in the string in hexadecimal format
        for (unsigned int i = 0; i < data.length(); i++)
        {
            char c = data.charAt(i);
            Serial.print(c, HEX);
            Serial.print(':');
        }
        Serial.println(")"); 
       */
        if (data == "CTS") {
          CTS_received++;  // Increase CTS counter when CTS message is received
          Serial.println(message);
          cts_received = true;
        }
      } else {
        Serial.println("RTS");
      }
      delay(100);
    }
    if (cts_received) {
      //Serial.println("cts received if statement"); 
      //clearInputBuffer();
      return;  
    } else {
      Serial.println("retrying"); 
      retries++;
    }
  }
  Serial.println("timeout error");
}

void report_out() {
  loopCounter++; 
  unsigned long currentTime = millis();
  if (currentTime - lastReportTime >= reportInterval) {
    snprintf(warn_msg, sizeof(warn_msg), "Message count: %d, Loop count: :%d, RTS received: :%d, CTS received: :%d", messagesReceived, loopCounter, RTS_received, CTS_received);
    send_with_handshake(warn_msg);
    messagesReceived = 0; 
    loopCounter = 0;
    lastReportTime = currentTime;
    RTS_received = 0; // Reset the RTS and CTS counters
    CTS_received = 0;
  }
}

void tx_rx(){
  if (Serial.available()) {
    data = Serial.readStringUntil('\n');
    //Serial.println(data);
    data.trim();
    if (data == "RTS") {
      RTS_received++; // Increase RTS counter when RTS message is received
      Serial.println("CTS");
      //clearInputBuffer();
      return;
    }
    else if (data == "pong") {
      messagesReceived++;
      send_with_handshake("ping");
    }
  }
}

void setup(){
  Serial.begin(115200);
  delay(8000);
  Serial.println("in setup");
  send_with_handshake("ping");
}

void loop(){
  tx_rx();
  report_out();
  delay (500);
}
