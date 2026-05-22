#include <WiFi.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <string.h>
#include <stdlib.h>
#include <EEPROM.h>

// CommandFlags
#define MoveJoystick    0x09
#define Move            0x10
#define Calib           0x11
#define EmerStop        0x12
#define Get_angle       0x13
#define Standby         0x14
#define Get_all         0x15
#define Stop            0x16
#define Move_Offset     0x17
#define MoveHome        0x26
#define ClearError      0x27
#define Open_Gripper    0x30
#define Closed_Gripper  0x31
// #define Rob_ena         0x29

//  JogMode 
#define Linear          0x18
#define J123            0x19
#define J456            0x20
#define Reori           0x21
#define Angle           0x22
#define Posi            0x23

// StatusFlags
#define Calibrated      0x01
#define Busy            0x02
#define Error           0x03
#define Complete        0x28
#define Home            0x25      // At home postion
#define OOR             0x04      // Out of Range
#define OOS             0x05      // Out of Speed
#define LIM             0x06      // Lost I2C Memory ie Lost calibration and 
                                  // last position 
#define LIE             0x07      // Lost I2C Encoder
#define LIP             0x08      // Lost all I2C Peripheral
#define TCP_error       0x0A
#define TCP_norm        0x0B


// Access Point credentials
const uint8_t address_slave_arr[7] = {0x00,0x06,0x07,0x08,0x09,0x10,0x11};
const char* ssid     = "ESP32-Robot-Mode";
const char* password = "doanxembro";
const uint8_t I2CSize = 30;
#define json_size       512
#define SDA_PIN          21
#define SCL_PIN          22
#define Air_Valve1       26
#define Air_Valve1_2     33

unsigned int commandTCP_OnReceive; 
unsigned int modeTCP_OnReceive; 
unsigned int enableTCP_OnReceive;
uint8_t status_all_joint;
volatile bool isNoti = false;
volatile bool newJsonFlag = false;
volatile float joint_tarAccel[7];
volatile float joint_tarSpeed[7];
volatile float joint_curSpeed[7] = {0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00};
volatile float joint_tarAngle[7];
volatile float joint_tarAngle_Prev[7];
volatile float joint_curAngle[7] = {0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00};
volatile uint8_t status_joint[7] = {0,0,0,0,0,0,0};
volatile uint8_t Ishome[7] = {0,0,0,0,0,0,0};
volatile uint8_t isGripperOpened = false;
volatile bool isCalib = false;
volatile bool rxReady = false;
volatile int numByte;
volatile bool start_joint[7];
volatile byte i2C_rx_buffer[I2CSize];
byte i2C_tx_buffer[I2CSize];
char serial_rx_buffer[50];
volatile bool fb_flag;
volatile bool isTarAngle_null = false;
volatile bool isTarSpeed_null = false;
volatile bool isTarAccel_null = false;

DynamicJsonDocument docSend(json_size);
DynamicJsonDocument docReceive(json_size);

WiFiServer server(80);
WiFiClient client;
TaskHandle_t TCP_OnReceive_handle = NULL;
TaskHandle_t TCP_OnRequest_handle = NULL; 
TaskHandle_t jointControl_TaskHandle = NULL;
TaskHandle_t jointFeedback_TaskHandle = NULL;
SemaphoreHandle_t i2cMutex; 
//------------------------------------------Begin I2C function-------------------------------------------------
void RxBuff_reset(){
  for(uint8_t j = 0; j < 20; j++){
    i2C_rx_buffer[j] = 0;
  }
}

void TxBuff_reset(){
  for(uint8_t j = 0; j < 20; j++){
    i2C_tx_buffer[j] = 0;
  }
}

//Encode angle to put in i2c TX package
void i2c_trans_angle(float target){
  char targetStr[8];
  dtostrf(target, 7, 2, targetStr);
  for(uint8_t i = 1; i < 9; i++){
    i2C_tx_buffer[i] = targetStr[i-1];
  }  
  // Serial.print("targetStr = ");
  // Serial.print(targetStr);
  // Serial.println(" ");
}

//Decode angle from i2c RX package
float i2c_receive_angle(){
  float target;
  char targetStr[8];
  for(uint8_t i =1; i < 9; i++){
    //      0->7                1->8
    targetStr[i-1] = i2C_rx_buffer[i];
  }
  target = atof(targetStr);
  return target;
}

//Encode speed to put in i2c TX package
void i2c_trans_speed(float target){
  char targetStr[8];
  dtostrf(target, 7, 2, targetStr);
  for(uint8_t i = 9; i < 17; i++){
    i2C_tx_buffer[i] = targetStr[i-9];
  }  
  // Serial.print("targetStr = ");
  // Serial.print(targetStr);
  // Serial.println(" ");
}

void i2c_trans_accel(float target){
  char targetStr[4];
  dtostrf(target, 3, 2, targetStr);
  for(uint8_t i = 17; i < 21; i++){
    i2C_tx_buffer[i] = targetStr[i-17];
  }  
  // Serial.print("targetStr = ");
  // Serial.print(targetStr);
  // Serial.println(" ");
}

bool checksCompl_all_joint(){
  uint8_t sum = 0;
  uint8_t sum_theory = 6;
  bool statusFunc;
  for (uint8_t i = 1; i < 7; i++){
    switch (status_joint[i]){
      case OOR: 
        // Serial.printf("Out of range joint %d",i);
        sum_theory -= 1;
        break;

      case OOS: 
        // Serial.printf("Out of speed joint %d: ",i);
        sum_theory -= 1;
        break;  

      case Busy: 
        // Serial.printf("Not Complete joint %d: ",i);
        sum += 0;
        break;

      case Complete:
        // Serial.printf("Complete joint %d: ",i);
        sum += 1;
        break;
    }
  }
  // Serial.printf("Complete Sum = %d \n",sum);
  if (sum == sum_theory) statusFunc = 1;
  else statusFunc = 0;

  return statusFunc;
}

bool checksCalib_all_joint(){
  uint8_t sum = 0;
  uint8_t sum_theory = 6;
  bool statusFunc;
  for (uint8_t i = 1; i < 7; i++){
    // Serial.printf("Checking Joint %d \n ",i);
    switch (status_joint[i]){
      case Calibrated:{
        if (i!=6) Serial.printf("Joint %d calibrated \n ",i);
        else Serial.printf("Joint %d calibrated \n\n ",i);
        sum += 1;
        break;
      }
      case LIM:{
        // if (i!=6) Serial.printf("Joint %d not calibrated \n ",i);
        // else Serial.printf("Joint %d not calibrated \n\n ",i);
        sum += 0;
      }
      case LIE:{
        break;
      }
      case LIP:{
        break;
      }
    }
  }
  // Serial.println(" ");
  if (sum == sum_theory) statusFunc = 1;
  else statusFunc = 0;
  return statusFunc;
}


// uint8_t get_robot_overall_status() {
//   bool anyBusy = false;
//   bool allComplete = true;

//   for (uint8_t i = 1; i <= 6; i++) {
      
//       uint8_t st = status_joint[i];

//       // Serious Error
//       if (st == LIP) return LIP;
//       if (st == LIE) return LIE;
//       if (st == LIM) return LIM;

//       // Less serious error
//       if (st == OOR) return OOR;    //Out of reach
//       if (st == OOS) return OOS;    //Out of speed

//       // if joint busy
//       if (st == Busy) {
//           anyBusy = true;
//           allComplete = false;
//       }

//       if (st == Complete) {

//       }
//   }

//   if (anyBusy) return Busy;

//   if (allComplete) return Complete;

//   return Ready;
// }
/*
void sendI2CPacket(uint8_t jointnumber, float target, float vel, int command){
  if (command != Standby) {
    Serial.printf("sendI2CPackage Command: 0x%0x to joint %d: %.2f deg, %.2f speed \n",command,jointnumber,target, vel);
    i2C_tx_buffer[0] = command;
    i2c_trans_angle(target);
    i2c_trans_speed(vel);
    Wire.beginTransmission(address_slave_arr[jointnumber]); 
    Wire.write(i2C_tx_buffer, sizeof(i2C_tx_buffer));
    Wire.endTransmission();  
    TxBuff_reset();
  }
  else {
    // Serial.printf("sendI2CPackage Standbyto joint %d \n", jointnumber);
    i2C_tx_buffer[0] = command;
    // i2c_trans_angle(target);
    Wire.beginTransmission(address_slave_arr[jointnumber]); 
    Wire.write(i2C_tx_buffer, sizeof(i2C_tx_buffer));
    Wire.endTransmission();  
    TxBuff_reset();
  }
}*/

void sendI2CPacket(uint8_t jointnumber, float target, float vel, float accel, int command){
  if (command != Standby && command != Calib ) {
    TxBuff_reset();
    Serial.printf("sendI2CPackage Command: 0x%0x to joint %d: %.2f deg, %.2f speed, %.2f accel  \n",command,jointnumber,target, vel,accel);
    i2C_tx_buffer[0] = command;
    i2c_trans_angle(target);
    i2c_trans_speed(vel);
    i2c_trans_accel(accel);
    Wire.beginTransmission(address_slave_arr[jointnumber]); 
    Wire.write(i2C_tx_buffer, sizeof(i2C_tx_buffer));
    Wire.endTransmission();  
    TxBuff_reset();
  }
  else {
    TxBuff_reset();
    Serial.printf("sendI2CPackage 0x%0x to joint %d \n", command, jointnumber);
    i2C_tx_buffer[0] = command;
    // i2c_trans_angle(target);
    Wire.beginTransmission(address_slave_arr[jointnumber]); 
    Wire.write(i2C_tx_buffer, sizeof(i2C_tx_buffer));
    Wire.endTransmission();  
    TxBuff_reset();
  }
}

void getSlave_data(uint8_t jointnumber){
  uint8_t rxLen = 0;
  // Serial.printf("getSlave_data (%d) \n", jointnumber);
  numByte = Wire.requestFrom(address_slave_arr[jointnumber],I2CSize,true);
  while(Wire.available() && rxLen < numByte){
    i2C_rx_buffer[rxLen++] = Wire.read();
  }
  rxReady = true;
}

bool I2C_parsing(uint8_t jointnumber){
  bool out = false;
  if (rxReady == false) return out;
  
  if (numByte != I2CSize) {
    // Serial.printf("I2C J%d frame error: len= %d; expected= %d\n", jointnumber, numByte, I2CSize);
    RxBuff_reset();
    rxReady = false;
    return out;
  }
  else{
    out = true;
    joint_curAngle[jointnumber] =  i2c_receive_angle();
    status_joint[jointnumber] = i2C_rx_buffer[17];
    Ishome[jointnumber] = i2C_rx_buffer[18];
    RxBuff_reset();
    rxReady = false;
    return out;
  }
}

void scanI2C() {
  Serial.println("I2C scanning");
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("Found I2C device at 0x%02X\n", addr);
    }
  }
}
//------------------------------------------End I2C function-------------------------------------------------


/*bool readingSerial(){
  bool status;
  if ((bool)Serial.available()){
    byte m = Serial.readBytesUntil('\n', serial_rx_buffer, sizeof(serial_rx_buffer));
    serial_rx_buffer[m] = '\0';  //null-byte
    float j1, j2, j3, j4, j5, j6;

    if (sscanf(serial_rx_buffer, "%f, %f, %f, %f, %f, %f", &j1, &j2, &j3, &j4, &j5, &j6) == 6){
      float arr[7] ={0, j1, j2, j3, j4, j5, j6};
      for (uint8_t i = 1; i < 7; i++){
        joint_tarAngle[i] = arr[i];
      }
      Serial.print("float j1 = ");
      Serial.println(joint_tarAngle[1]);
      Serial.print("float j2 = ");
      Serial.println(joint_tarAngle[2]);
      Serial.print("float j3 = ");
      Serial.println(joint_tarAngle[3]);
      Serial.print("float j4 = ");
      Serial.println(joint_tarAngle[4]);
      Serial.print("float j5 = ");
      Serial.println(joint_tarAngle[5]);
      Serial.print("float j6 = ");
      Serial.println(joint_tarAngle[6]);
      status = 1;
    }
    else {
      Serial.println("error in input!");
      status = 0;
    }
  }  
  return status;
}*/

void jointControl(void* parameter){
  while(1){
    switch(commandTCP_OnReceive){
      case 0:{
        break;
      }

      case MoveJoystick:{
        switch(modeTCP_OnReceive){
          case Linear:
          case Reori:
          case Angle:
          case Posi:{
            if(enableTCP_OnReceive == 1){
              if (memcmp((const void*)joint_tarAngle, (const void*)joint_tarAngle_Prev, 6 * sizeof(float)) == 0) {
              }
              else{
                // Serial.println("Moving 6 joints");
                for (uint8_t i = 1; i < 7; i++){
                  sendI2CPacket(i,joint_tarAngle[i],joint_tarSpeed[i],joint_tarAccel[i],MoveJoystick);
                }
                memcpy((void*)joint_tarAngle_Prev, (const void*)joint_tarAngle, 6 * sizeof(float));
              }
            }

            break;
          }

          case J123:{
            if(enableTCP_OnReceive == 1){

            }

            break;
          }
          case J456:{
            if(enableTCP_OnReceive == 1){
              
            }
            // Serial.println("Moving 3 joints");
            break;
          }
        }
      }

      case Move:{
        if(enableTCP_OnReceive == 1){
          if (newJsonFlag==true){
            switch(modeTCP_OnReceive){
              case Angle:{
                // Serial.println("Moving by angle");
                for (uint8_t i = 1; i < 7; i++){
                  sendI2CPacket(i,joint_tarAngle[i],joint_tarSpeed[i],joint_tarAccel[i],Angle);
                }
                Serial.println(" ");
                break;
              }

              case Posi:{
                // Serial.println("Moving by angle");
                for (uint8_t i = 1; i < 7; i++){
                  sendI2CPacket(i,joint_tarAngle[i],joint_tarSpeed[i],joint_tarAccel[i],Posi);
                }
                Serial.println(" ");
                break;
              }
              
              case Linear:{
                // Serial.println("Moving by angle");
                for (uint8_t i = 1; i < 7; i++){
                  // if (i == 1 || i ==2 || i ==5 || i ==6){
                    sendI2CPacket(i,joint_tarAngle[i],joint_tarSpeed[i],joint_tarAccel[i],Linear);
                  // }
                }
                Serial.println(" ");
                break;
              }
              default:{
                break;
              }
            }
            newJsonFlag = false;
          }
        }


        break;
      }

      case Calib:{
        if(enableTCP_OnReceive == 1){
          if (newJsonFlag==true){
            for (uint8_t i = 1; i < 7; i++){
              sendI2CPacket(i,0,0,joint_tarAccel[i],Calib);
            }
            Serial.println(" ");
            newJsonFlag = false;
          }
        }
        break;
      }

      case EmerStop:{
        Serial.println("case EmerStop");
        for (uint8_t i = 1; i < 7; i++){
        }
        // Serial.println(" ");
        break;
      }

      case Move_Offset:{
        if(enableTCP_OnReceive == 1){
          if (newJsonFlag==true){
            switch(modeTCP_OnReceive){
              case J123:{
                for (uint8_t i = 1; i < 4; i++){
                  sendI2CPacket(i,joint_tarAngle[i],joint_tarSpeed[i],joint_tarAccel[i],Move_Offset);
                }
                Serial.println(" ");
                break;
              }
              case J456:{
                // if (memcmp((const void*)joint_tarAngle, (const void*)joint_tarAngle_Prev, 6 * sizeof(float)) == 0) {
                // }
                // else{
                  for (uint8_t i = 4; i < 7; i++){
                    sendI2CPacket(i,joint_tarAngle[i],joint_tarSpeed[i],joint_tarAccel[i],Move_Offset);
                  }
                  Serial.println(" ");
                //   memcpy((void*)joint_tarAngle_Prev, (const void*)joint_tarAngle, 6 * sizeof(float));
                // }
                break;
              }
            }
            newJsonFlag = false;
          }

        }
        break;
      }

      case MoveHome:{
        if (newJsonFlag==true){
          for (uint8_t i = 1; i < 7; i++){
              sendI2CPacket(i, 0, 0,joint_tarAccel[i], MoveHome);
          }
          Serial.println(" ");
        }
        newJsonFlag = false;
        break;
      }

      case Standby:{
        if (newJsonFlag==true){
          for (uint8_t i = 1; i < 7; i++){
              sendI2CPacket(i, 0, 0,joint_tarAccel[i], Standby);
          }
          Serial.println(" ");
        }
        newJsonFlag = false;
        break;
      }

      case ClearError:{
        break;
      }
      
      case Open_Gripper:{
        if (newJsonFlag==true){
          if (isGripperOpened == 1){
            Serial.println("Already opened Gripper");
          }
          else{
            Serial.println("Open Gripper");
            digitalWrite(Air_Valve1,1);
            digitalWrite(Air_Valve1_2,1);
            isGripperOpened = 1;
          }

        }
        newJsonFlag = false;
        break;
      }

      case Closed_Gripper:{
        if (newJsonFlag==true){
          if (isGripperOpened == 1){
            Serial.println("Close Gripper");
            digitalWrite(Air_Valve1,0);
            digitalWrite(Air_Valve1_2,0);
            isGripperOpened = 0;
          }
          else{
            Serial.println("Already closed Gripper");
          }
        }
        newJsonFlag = false;
        break;
      }

      default:{

        break;
      }
    }
    vTaskDelay(30 / portTICK_PERIOD_MS);    
  }
  vTaskDelete(NULL); 
}

void jointFeedback(void* parameter){
  // uint8_t SumJointdata = 0
  // bool status_all_joint = 0;
  while(1){
    for (uint8_t i = 1; i < 7; i++){
      // xSemaphoreTake(i2cMutex, portMAX_DELAY);
      getSlave_data(i);
      if(I2C_parsing(i)){
        // Serial.printf("joint[%d]: Angle = %.2f; stat = %d; Ishome = %d \n", i, joint_curAngle[i],status_joint[i],Ishome[i]);
      }
      // xSemaphoreGive(i2cMutex);
    }
    // Serial.println(" ");

    if (isCalib == false){
      if (checksCalib_all_joint()) {
        status_all_joint = Calibrated;
        Serial.println("Calibrated 6 joints");
        isCalib = true;
      }
    }
    else{
      if (checksCompl_all_joint()) status_all_joint = Complete;
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);    
  }
  vTaskDelete(NULL); 
}

void TCP_OnReceive(void* parameter){
  bool status_all_joint = 0;

  while(1){
    //if disconnected or not connected 
    if (!client || !(client.connected())) {
        if (isNoti == false) Serial.println("client not connect");
        client = server.available();
        isNoti = true;
        vTaskDelay(10/ portTICK_PERIOD_MS);
        continue;
    }

    //if connected and kotlin send sth
    if (client.available()){
      DynamicJsonDocument doc(json_size);
      DeserializationError error;
      String json = client.readStringUntil('\n');

      //if kotlin send empty Json
      if (json.length() == 0) {
        Serial.println("Received empty JSON");
        commandTCP_OnReceive = 0;
        vTaskDelay(5/ portTICK_PERIOD_MS);
        continue;
      }
      else{
        error = deserializeJson(doc, json);
        if (error){
          Serial.print("Failed to parse JSON: ");
          // Serial.print(json);
          // Serial.println(" ");
          commandTCP_OnReceive = 0;
          continue;
        }
        else{
          Serial.print("Received JSON: ");
          Serial.print(json);
          Serial.println("");
        }
      }
      newJsonFlag = true;
      commandTCP_OnReceive = doc["command"];
      modeTCP_OnReceive = doc["mode"];
      enableTCP_OnReceive = doc["enable"];
      JsonArray jointsAngle = doc["angle"];
      JsonArray jointsSpeed = doc["speed"];
      JsonArray jointsAccel = doc["accel"];
      client.flush();

      // Serial.println(" ");
      // Serial.print("commandTCP_OnReceive: ");
      // Serial.print(commandTCP_OnReceive);
      // Serial.println(" ");
      // Serial.print("modeTCP_OnReceive: ");
      // Serial.print(modeTCP_OnReceive);
      // Serial.println(" ");
      // Serial.print("enableTCP_OnReceive: ");
      // Serial.print(enableTCP_OnReceive);
      // Serial.println(" ");

      if ((jointsAngle.isNull())) {
        isTarAngle_null = true;
      } 
      else{
        isTarAngle_null = false;
        for (uint8_t i = 0; i < jointsAngle.size(); i++) {
          joint_tarAngle[i+1] = jointsAngle[i];
          // Serial.printf("Target_angle joint %d: %.2f \n", i+1, joint_tarAngle[i+1]);
        }
      }

      if((jointsSpeed.isNull())){
       isTarSpeed_null = true;
       joint_tarSpeed[1] = 0.0;
       joint_tarSpeed[2] = 0.0;
       joint_tarSpeed[3] = 0.0;
       joint_tarSpeed[4] = 0.0;
       joint_tarSpeed[5] = 0.0;
       joint_tarSpeed[6] = 0.0;
      }
      else{
        // Serial.println(" ");
        isTarSpeed_null = false;
        for (uint8_t i = 0; i < jointsSpeed.size(); i++) {
          joint_tarSpeed[i+1] = jointsSpeed[i];
          // Serial.printf("Target_speed joint %d: %.2f \n", i+1, joint_tarSpeed[i+1]);
        }
      }

      if((jointsAccel.isNull())){
       isTarAccel_null = true;
       joint_tarAccel[1] = 0.0;
       joint_tarAccel[2] = 0.0;
       joint_tarAccel[3] = 0.0;
       joint_tarAccel[4] = 0.0;
       joint_tarAccel[5] = 0.0;
       joint_tarAccel[6] = 0.0;
      }
      else{
        // Serial.println(" ");
        isTarAccel_null = false;
        for (uint8_t i = 0; i < jointsAccel.size(); i++) {
          joint_tarAccel[i+1] = jointsAccel[i];
          // Serial.printf("Target_accel joint %d: %.2f \n", i+1, joint_tarAccel[i+1]);
        }
      }
    }
    // Serial.printf("TCP_OnReceive Stack Free: %u bytes\n", uxTaskGetStackHighWaterMark(NULL));
    vTaskDelay(200 / portTICK_PERIOD_MS);    
  }
  vTaskDelete(NULL); 
}

void TCP_OnRequest(void* parameter){
  uint8_t stat_func;
  uint8_t stat_all_joint = 0;
  while(1){
    DynamicJsonDocument doc(json_size);    
  
    if (client.connected()){
      // Serial.println("Begin of jointFeedback"); 
      doc["command"]  = commandTCP_OnReceive;
      doc["mode"]     = modeTCP_OnReceive;
      doc["enable"]   = enableTCP_OnReceive;
      // stat_all_joint = get_robot_overall_status();
      // stat_all_joint = Busy;
      stat_all_joint = status_all_joint;
      doc["rob_status"] = stat_all_joint;
      // Serial.printf("rob_status = 0x%0x \n", stat_all_joint);
      JsonObject curObj = doc.createNestedObject("current");
      JsonArray angleArr = curObj.createNestedArray("angle");
      JsonArray veloArr  = curObj.createNestedArray("velo");
      JsonArray statArr  = curObj.createNestedArray("stat");
      JsonArray homeArr  = curObj.createNestedArray("home");

      for (int i = 0; i < 6; i++) {
          angleArr.add(joint_curAngle[i+1]);
          veloArr.add(joint_curSpeed[i+1]);
          statArr.add(status_joint[i+1]);
          homeArr.add(Ishome[i+1]);
      }

      String jsonString;
      serializeJson(doc, jsonString);

      // Serial.print("jointFeedback String: ");
      // Serial.print(jsonString);
      // Serial.println(" ");

      client.println(jsonString);
      // client.flush();
      // Serial.println("End of jointFeedback"); 
    }
    else{

    }
    // Serial.printf("TCP_OnRequest Stack Free: %u bytes\n", uxTaskGetStackHighWaterMark(NULL));
    vTaskDelay(50 / portTICK_PERIOD_MS);    
  }
  vTaskDelete(NULL); 
} 

void setup() {
  Wire.setClock(1000000);
  Wire.begin();
  Serial.begin(115200);
  delay(3000);

  Serial.printf("Starting FreeRTOS: Memory Usage\nInitial Free Heap: %u bytes\n", xPortGetFreeHeapSize());  
  Serial.print("Setting AP (Access Point)…");
  WiFi.softAP(ssid, password);
  delay(3000);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
  server.begin();
  client.setTimeout(1000);
  // i2cMutex = xSemaphoreCreateMutex();
  delay(8000);
  // scanI2C(); 
  status_all_joint = LIM;
  pinMode(Air_Valve1,OUTPUT);
  pinMode(Air_Valve1_2,OUTPUT);
  digitalWrite(Air_Valve1,1);
  digitalWrite(Air_Valve1_2,1);
  // digitalWrite(Air_Valve,1);

  xTaskCreatePinnedToCore(
    TCP_OnReceive,                  // Function
    "TCP_OnReceive_Server_Task",    // Task name
    20000,                          // Stack size
    NULL,                           // Task input
    23,                             // Priority
    &TCP_OnReceive_handle,          // Task handle
    0                               // Run on core 0
  );

  xTaskCreatePinnedToCore(
    TCP_OnRequest,                  // Function
    "TCP_OnRequest_Server_Task",    // Task name
    20000,                          // Stack size
    NULL,                           // Task input
    22,                             // Priority
    &TCP_OnRequest_handle,          // Task handle
    0                               // Run on core 0
  );
  
  xTaskCreatePinnedToCore(
    jointControl,               // Function to run as a task
    "jointControl",             // Name of the task
    10000,                       // Stack size in bytes
    NULL,                       // Optional input (use a pointer if needed)
    23,                         // Priority (higher = more important)
    &jointControl_TaskHandle,   // Task handle (optional)
    1                           // Core 1
  );

  xTaskCreatePinnedToCore(
    jointFeedback,                // Function to run as a task
    "jointFeedback",              // Name of the task
    10000,                         // Stack size in bytes
    NULL,                         // Optional input (use a pointer if needed)
    22,                           // Priority (higher = more important)
    &jointFeedback_TaskHandle,    // Task handle (optional)
    1                             // Core 1
  );

  // for (uint8_t i = 1; i < 7; i++){
  //   getAngle(i);
  //   Serial.printf("Angle of Joint %d: %.2f \n",i,joint_curAngle[i]);
  // }
  // scanI2C();
  Serial.println("Ready for TCP/IP input");
}

void loop() {
  // static uint32_t lastCheck = 0;
  // if (millis() - lastCheck > 5000) {
  //   Serial.printf("Free Heap: %u bytes\n", xPortGetFreeHeapSize());
  //   lastCheck = millis();
  // }

}
