/* Credit: https://github.com/jgromes/RadioLib/wiki/Default-configuration#sx127xrfm9x---lora-modem  */
#include <Arduino.h>
#include <ros.h>
ros::NodeHandle  nh;
#include <std_msgs/String.h>
#include <RadioLib.h>
SX1276 radio = new Module(18, 26, 14, 33);
int transmissionState = RADIOLIB_ERR_NONE;
bool transmitFlag = false;
volatile bool operationDone = false;
int loopCounter = 0;
char msg_buf[120];
char warn_msg[120];
int receivedCounter;
unsigned long lastReportTime = 0;
int messagesReceived = 0;
const unsigned long reportInterval = 10000; // 10 seconds in milliseconds
unsigned long lastMessageTime = 0;
const unsigned long messageTimeout = 20000;  // 20 seconds in milliseconds

void setFlag(void){
  operationDone = true;
}

void tx_rx_LoRa(){
  if (operationDone){            // check if the previous operation finished
    operationDone = false;       // reset flag
    if (transmitFlag){           // the previous operation was transmission, listen for response
      if (transmissionState != RADIOLIB_ERR_NONE){
        snprintf(warn_msg, sizeof(warn_msg), "In tx_rx_LoRa - failed, transmissionState code: %d ", transmissionState);
        nh.loginfo(warn_msg); 
        //Serial.println("In tx_rx_LoRa - failed, transmissionState code");
      }
      radio.startReceive();  // listen for response
      transmitFlag = false;
    } else {
      String str;
      int state = radio.readData(str);
      if (state == RADIOLIB_ERR_NONE){
        messagesReceived++;
        lastMessageTime = millis();
      }
      sscanf(str.c_str(), "message from initiator #: %d ", &receivedCounter);  // pull off the number from the received message
      receivedCounter++;

      //snprintf(msg_buf, sizeof(msg_buf), "message from receiver #: %d ", receivedCounter);
      //transmissionState = radio.startTransmit(msg_buf);  

      strcpy(msg_buf, "message from receiver #: ");  // Convert the message string into a byte array
      strcat(msg_buf, std::to_string(receivedCounter).c_str());
      transmissionState = radio.startTransmit(reinterpret_cast<uint8_t*>(msg_buf), strlen(msg_buf));
        snprintf(msg_buf, sizeof(msg_buf), "packet sent with receiver #: %d ", receivedCounter);
      transmitFlag = true;
    }
  }
}

void report_out(){
  loopCounter++; 
  unsigned long currentTime = millis();
  if (currentTime - lastReportTime >= reportInterval) {
    snprintf(warn_msg, sizeof(warn_msg), "Message count: %d, Loop count: :%d ", messagesReceived, loopCounter);
    nh.logwarn(warn_msg);
    //Serial.print("Message count:");  Serial.println(messagesReceived);
    messagesReceived = 0; // Reset the messagesReceived counter
    loopCounter = 0;
    lastReportTime = currentTime;
  }
}

void LoRa_setup(){
  int state = radio.begin();
  if (state != RADIOLIB_ERR_NONE){
    while (1){
      snprintf(warn_msg, sizeof(warn_msg), "radio.begin()failed in LoRa_setup() - fatal, return code: %d", state); 
      nh.logwarn(warn_msg); 
      //Serial.println("radio.begin()failed in LoRa_setup()");
      delay(1000);     
      }  // loop forever
  }
  radio.setSyncWord(0x12);
  radio.setDio0Action(setFlag, RISING);
  state = radio.startReceive();
  if (state != RADIOLIB_ERR_NONE) {
    while (1){
      snprintf(warn_msg, sizeof(warn_msg), "radio.startReceive() failed in LoRa_setup() - fatal, return code: %d", state); 
      nh.logwarn(warn_msg); 
      //Serial.println("radio.startReceive() failed in LoRa_setup()");
      delay(1000);      
      }  // loop forever
  }
}

void check_message_cnt(){
  unsigned long currentTime = millis();
  if (currentTime - lastMessageTime >= messageTimeout) {
    //snprintf(warn_msg, sizeof(warn_msg), "receiver not getting messages - press reset on transmitter"); 
    //Serial.println(warn_msg);
    Serial.println("receiver not getting messages - press reset on transmitter");
    //nh.loginfo("receiver not getting messages - press reset on transmitter"); 
    lastMessageTime = millis();
  }  
}

void setup(){
  //Serial.begin(115200);
  nh.getHardware()->setBaud(57600);
  nh.initNode(); // Initialize the node
  delay(8000);
  //Serial.println("ttgo_LoRa_test_receivedr - starting LoRa Setup");
  nh.loginfo("ttgo_LoRa_test_receivedr - starting LoRa Setup"); 
  LoRa_setup(); 
}

void loop(){
  tx_rx_LoRa();
  report_out();
  check_message_cnt();
  //nh.spinOnce(); // Handle ROS communication
}