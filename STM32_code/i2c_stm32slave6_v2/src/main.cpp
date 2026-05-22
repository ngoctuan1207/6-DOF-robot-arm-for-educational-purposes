//STM Slave 6
#include<Arduino.h>
#include <AccelStepper.h>
#include<Wire.h>
#include<string.h>
#include<stdlib.h>
#include <AT24C256.h>
#include <math.h>
// #include "SpeedyStepper_v1.h"
#include "AS5600.h"

#define I2C1_clock      1000000L  // Clock speed to communicate with master
#define I2C2_clock      400000L     // Clock speed to communicate with AS5600(slave)
#define SDA2            PB11
#define SCL2            PB10
#define SDA1            PB7
#define SCL1            PB6
#define stepPin1        PA8
#define dirPin1         PA9       // DIR HIGH is CW, LOW is CCW
#define enPin1          PA10
#define IPS1            PB0
#define homeSpeed       2000       // home moving speed
#define homeAccel       1000
#define movingSpeed     12500
#define movingAccel     6400
#define angle_SR        20000       // Angle sampling rate
#define motor_SR        500       // Angle sampling rate

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
// #define Rob_ena         0x29
  
//  JogMode 
#define Linear          0x18
#define J623            0x19
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
#define maxVel          10000

const float J6_H = 181.0;
const float J6_L =-181.0;
const float J6_EH = 100.1/100;
const float J6_EL = 99.9/100;

const float offset_Angle = 1.0;
float offset_Zero;
const uint8_t I2CSize = 30;

const float pulley_ratio = 1;
const float gearBox_ratio = 0.05;
const float microStep = 0.25;      //0.0625
const float step_round = 200;
const float ratio_total = ((step_round/microStep)/pulley_ratio)/gearBox_ratio;

TwoWire Wire2(SDA2, SCL2);
AT24C256 eeprom(0x50, &Wire2);
AS5600L as5600_1(0x36,&Wire2);   //  use default Wire
HardwareTimer timer1(TIM1);
HardwareTimer timer2(TIM2);
HardwareTimer timer3(TIM3);
HardwareTimer timer4(TIM4);
AccelStepper AC_stepper(AccelStepper::DRIVER,stepPin1,dirPin1);

const uint8_t address_slave_arr[7] = {0x00,0x06,0x07,0x08,0x09,0x10,0x11};
const uint8_t address_slave = address_slave_arr[6];
byte i2C_tx_buffer[I2CSize];
volatile byte i2C_rx_buffer[I2CSize];

volatile bool rxReady = false;
volatile uint8_t cmd_flag;
volatile uint8_t stat_flag;
volatile uint8_t Ishome = 0;
volatile float targetAccel;
volatile float targetVel;
volatile float targetDeg_Int;
volatile float pending_linearTarget;
volatile bool hasPending_linearTarget = false;

uint8_t quadrant;
volatile int numByte;
static float oldPos = 367;
volatile float CurrAngle_int, CurrRawAngle_int;
volatile bool calib_flag,status;
static bool move_Issued = false;
volatile bool isCalib = false;
static float offset;
volatile bool IPS; //Use for interrupt
volatile bool motorTick = false;

//---------------------Start of AS5600 function-----------------------
void setup_AS5600();
void checkMagnet_n_Connection();
void getStatus();
void Timer_Encoder(bool On);
//-----------------------End of AS5600 function-----------------------

//---------------------Start of AT24C256 function-----------------------
float eeprom_get_angle();
bool eeprom_save(float target, float offset, bool stat_calib);
bool eeprom_save_angle(float target);
bool eeprom_get();
void eeprom_save_quadrants(float target);
//-----------------------End of AT24C256 function-----------------------

//---------------------Start of I2C1 function-----------------------
void Rx0Handler(int nBytes);
void Tx0Handler();
void i2c_trans_angle(float target);
void RxBuff_reset();
void TxBuff_reset();
float i2c_receive_angle();
void I2C_PeriCheck(bool verbose);
bool isTarget_inLimit(float target);
bool isTargetOld(float target);
bool I2C_parsing();
//-----------------------End of I2C1 function-----------------------

//-----------------------Start of function----------------------------
void CheckIPS();
long Degree_to_Pusle(float degree);
float Measure_raw_angle_int();
void Measure_angle_int();
void Motion_Control();
float Pusle_to_Degree(long pos);
//-------------------------End of function----------------------------

void setup() {
  Serial.begin(115200);
  delay(5000);
  Wire.begin(address_slave);
  Wire.onReceive(Rx0Handler);
  Wire.onRequest(Tx0Handler);
  Wire.setClock(I2C1_clock);

  Wire2.begin();
  Wire2.setClock(I2C2_clock); 
  Wire2.setTimeout(5);
  I2C_PeriCheck(true);
  setup_AS5600();
  delay(5000);

  pinMode(stepPin1, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  pinMode(enPin1, OUTPUT);
  pinMode(IPS1, INPUT);

  IPS = digitalRead(IPS1);
  attachInterrupt(digitalPinToInterrupt(IPS1), CheckIPS, CHANGE); //ENABLE ALWAYS WATCH FOR HOME
  AC_stepper.setMaxSpeed(4000);
  AC_stepper.setAcceleration(2000);
  // Timer_Encoder(true);
  Serial.printf("J6 ON, Home stat = %d \n",IPS);
  stat_flag = LIM;
}


void loop() {
  Motion_Control();
  bool stat = AC_stepper.run();
  if (stat == 0) {
    if (move_Issued == true) {
      stat_flag = Complete;
      move_Issued = false;
      Timer_Encoder(false);
    }
  }

  // if(move_Issued == true){
  //   bool stat = AC_stepper.run();
  //   if (stat == 0) {
  //     stat_flag = Complete;
  //     move_Issued = false;
  //   }
  // }

  // if (AC_stepper.run() == 0 && isCalib == true){
  //   stat_flag = Complete;
  // }
  // if (hasPending_linearTarget) {
  //   if (abs(AC_stepper.distanceToGo()) < 20) {
  //     AC_stepper.moveTo(pending_linearTarget);
  //     hasPending_linearTarget = false;
  //   }
  // }
}


//---------------------Start of I2C function-----------------------
//Decode angle from i2c RX package
float i2c_receive_angle(){
  float target;
  char targetStr[8];
  for(uint8_t i =1; i < 9; i++){
    targetStr[i-1] = i2C_rx_buffer[i];
  }
  target = atof(targetStr);
  return target;
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

//Decode velocity from i2c RX package
float i2c_receive_vel(){
  float target;
  char targetStr[7];
  for(uint8_t i =9; i < 17; i++){
    targetStr[i-9] = i2C_rx_buffer[i];
  }
  target = atof(targetStr);
  return target;
}

float i2c_receive_accel(){
  float target;
  char targetStr[4];
  for(uint8_t i =17; i < 21; i++){
    targetStr[i-17] = i2C_rx_buffer[i];
  }
  target = atof(targetStr);
  return target;
}

//Encode velocity to put in i2c TX package
void i2c_trans_vel(float target){
  char targetStr[7];
  dtostrf(target, 7, 2, targetStr);
  for(uint8_t i = 9; i < 17; i++){
    i2C_tx_buffer[i] = targetStr[i-9];
  }  
}

//Reset buffer TX and RX
void RxBuff_reset(){
  for(uint8_t j = 0; j < I2CSize; j++){
    i2C_rx_buffer[j] = 0;
  }
}

void TxBuff_reset(){
  for(uint8_t j = 0; j < I2CSize; j++){
    i2C_tx_buffer[j] = 0;
  }
}

//I2C interrupt
void Tx0Handler(){
  // Serial.println("Tx0Handler() called");
  TxBuff_reset();
  i2C_tx_buffer[0] = cmd_flag;
  i2c_trans_angle(CurrAngle_int);
  i2C_tx_buffer[17] = stat_flag;
  // Serial.printf("i2C_tx_buffer[17] = %d \n", stat_flag);
  i2C_tx_buffer[18] = IPS;
  // Serial.printf("i2C_tx_buffer[18] = %d \n", IPS);
  Wire.write(i2C_tx_buffer, sizeof(i2C_tx_buffer));
  TxBuff_reset();
}

void Rx0Handler(int nBytes){
  numByte = nBytes;
  // Serial.println("Package available");
  uint8_t rxLen = 0;
  while (Wire.available() && rxLen < nBytes) {
    i2C_rx_buffer[rxLen++] = Wire.read();
  }
  rxReady = true;
}

bool I2C_parsing(){
  bool out = false;
  if (rxReady == false) return out;
  
  if (numByte != I2CSize) {
    // Serial.printf("I2C frame error: len= %d; expected= %d\n", numByte, I2CSize);
    RxBuff_reset();
    rxReady = false;
    return out;
  }
  else{
    out = true;
    cmd_flag = i2C_rx_buffer[0];
    if (cmd_flag != Standby){
      targetDeg_Int = i2c_receive_angle();
      float Vel = i2c_receive_vel();
      if (Vel!= 0) {
        targetVel = Vel/100 * maxVel;
        AC_stepper.setMaxSpeed(targetVel);
      }
      else {
        targetVel = maxVel/2;
        AC_stepper.setMaxSpeed(targetVel);
      }

      float Acc = i2c_receive_accel();
      if (Acc!=0){
        targetAccel = targetVel/Acc;
        AC_stepper.setAcceleration(targetAccel);
      }
      else  {
        targetAccel = 1200;
        AC_stepper.setAcceleration(targetAccel);
      }

      // Serial.printf("I2C_parsing J6: targetDeg_Int = %.2f, cmd_flag = %d, \n J6: targetVel = %.2f, targetAccel = %.2f \n\n", targetDeg_Int, cmd_flag, targetVel,targetAccel);
    } 
    RxBuff_reset();
    rxReady = false;
    return out;
  }
}

void I2C_PeriCheck(bool verbose){
  byte i,error=0;  
  byte address[2]={54,80}; //0x36, 0x50
  int nDevices =0;
  int nDevicesLost =0;
  for(i = 0; i < 1; i++) {
    Wire2.beginTransmission(address[i]);
    error = Wire2.endTransmission();
    if (error == 0) {
      switch (address[i]){
        case 54: {
          if(verbose) Serial.println("I2C device found at address 0x36");
          nDevices++;
        }
        
        case 80: {
          if(verbose) Serial.println("I2C device found at address 0x50");
          nDevices++;
        } 
      }

    }
    else if (error == 4) {
      switch (address[i]){
        case 54: {
          if(verbose) Serial.println("Unknown error at address 0x36");
          stat_flag = LIE;
          nDevicesLost++;
        }
        case 80: {
          if(verbose) Serial.println("Unknown error at address 0x50");
          stat_flag = LIM;
          nDevicesLost++;
        } 
      }
      if (nDevicesLost ==2){
        if(verbose) Serial.println("Lost all I2C peri");
        stat_flag = LIP;
      }
    }
  }
  if(verbose) Serial.printf(" %d device(s) found \n \n",nDevices);
  stat_flag = Complete;
}

bool isTarget_inLimit(float target){
  bool out = false;
  if((target >= J6_L)&&(target <= J6_H)){
    out = true;
  }
  return out;
}

bool isTargetOld(float target){
  bool out = false;
  if((target == oldPos)){
    out = true;
  }
  else oldPos = target;
  return out;
}
//-----------------------End of I2C function-----------------------


//---------------------Start of AS5600 function-----------------------
void setup_AS5600(){
  checkMagnet_n_Connection(); 
  as5600_1.setOffset(offset);
  as5600_1.setDirection(1);   //Degree decrease when turn CCW
  as5600_1.setMaxAngle(4095);
  as5600_1.setPowerMode(AS5600_POWERMODE_NOMINAL);
  as5600_1.setHysteresis(3);
  as5600_1.setSlowFilter(0);
  as5600_1.setFastFilter(4);
}

void checkMagnet_n_Connection(){
  bool c = 0; //Connection
  uint8_t s = 0; //Status

  while(c == 0){
    c = as5600_1.isConnected(); //Connection
    s = as5600_1.readStatus();
    // uint8_t mask = (1<<5) | (1<<4) | (1<<3);
    // Serial.println(s,BIN);
    // Serial.println(s&mask,BIN);
    // if ((c == 1) && ((s&mask) == (0b00100000))){
    //   Serial.println("Magnet: detected, Status: good, Connection: Established");
    //   break;
    // }
    if (c==1) break;
  }
}

void getStatus(){
  Serial.print("STATUS:\t ");
  Serial.println(as5600_1.readStatus(), BIN);
  Serial.print("CONFIG:\t ");
  Serial.println(as5600_1.getConfigure(), BIN);
  Serial.print("  GAIN:\t ");
  Serial.println(as5600_1.readAGC(), BIN);
  Serial.print("MAGNET:\t ");
  Serial.println(as5600_1.readMagnitude(), BIN);
  Serial.print("DETECT:\t ");
  Serial.println(as5600_1.detectMagnet(), BIN);
  Serial.print("M HIGH:\t ");
  Serial.println(as5600_1.magnetTooStrong(), BIN);
  Serial.print("M  LOW:\t ");
  Serial.println(as5600_1.magnetTooWeak(), BIN);
  Serial.println();
  delay(1000);
}

void Timer_Encoder(bool On){
  if (On == true){
    timer2.pause();
    timer2.setOverflow(angle_SR,MICROSEC_FORMAT);
    timer2.attachInterrupt(Measure_angle_int); 
    timer2.resume();
  }
  else timer2.detachInterrupt();
}
//-----------------------End of AS5600 function-----------------------

//---------------------Start of AT24C256 function-----------------------
float eeprom_get_angle(){
  float target_func;
  char message_func[7];
  eeprom.read(0, (uint8_t*) message_func, sizeof(message_func));
  target_func = atof(message_func);
  Serial.printf("Curr angle in EEPROM = %.2f\n \n",target_func);
  return target_func;
}

bool eeprom_save_angle(float target){
  char message_func[7];
  char targetStr_func[7];
  float target_func;

  dtostrf(target, 6, 2, targetStr_func);
  eeprom.write(0, (uint8_t*)targetStr_func, sizeof(targetStr_func));
  delay(5);

  eeprom.read(0, (uint8_t*) message_func, sizeof(message_func));
  delay(5);
  target_func = atof(message_func);
  if (target_func!= target){
    Serial.println("Error saving to EEPROM");
    return 0;
  }
  else {
    Serial.println("Saving to EEPROM successfully");
    return 1;
  }
}

bool eeprom_save(float target, float offset, bool stat_calib){
  Serial.println("Before save");
  char message_func[7];
  char targetStr_func[7];
  float target_func;
  float offset_func;
  bool stat_func;

//Saving
  dtostrf(target, 6, 2, targetStr_func);
  eeprom.write(0, (uint8_t*)targetStr_func, sizeof(targetStr_func));
  delay(5);

  dtostrf(offset, 6, 2, targetStr_func);
  eeprom.write(8, (uint8_t*)targetStr_func, sizeof(targetStr_func));
  delay(5);

  dtostrf(stat_calib, 6, 2, targetStr_func);
  eeprom.write(17, (uint8_t*)targetStr_func, sizeof(targetStr_func));
  delay(5);

//Checking
  eeprom.read(0, (uint8_t*) message_func, sizeof(message_func));
  delay(5);
  target_func = atof(message_func);
  if (target_func!= target) Serial.println("Error saving Target to EEPROM");

  eeprom.read(8, (uint8_t*) message_func, sizeof(message_func));
  delay(5);
  offset_func = atof(message_func);
  if (offset_func!= offset) Serial.println("Error saving Offset to EEPROM");

  eeprom.read(17, (uint8_t*) message_func, sizeof(message_func));
  delay(5);
  stat_func = (bool)atoi(message_func);
  if (stat_func!= stat_calib) Serial.println("Error saving stat_calib to EEPROM");

  Serial.println("After save");
  return 1;
}

bool eeprom_get(){
  char message_func[7];
  char message_func_small[2];
  bool out = false;
  eeprom.read(0, (uint8_t*) message_func, sizeof(message_func));
  delay(5);
  oldPos = atof(message_func);
  Serial.printf("oldPos = %.2f\n",oldPos);

  eeprom.read(8, (uint8_t*) message_func, sizeof(message_func));
  delay(5);
  offset_Zero = atof(message_func);
  Serial.printf("offset_Zero = %.2f\n",offset_Zero);

  eeprom.read(17, (uint8_t*) message_func_small, sizeof(message_func_small));
  delay(5);
  calib_flag = (bool) message_func_small;
  Serial.printf("calib_flag = %d\n",calib_flag);
  if (calib_flag == 1){
    out = true;
  }
  eeprom.read(20, &quadrant, sizeof(quadrant));
  delay(5);
  Serial.printf("quadrant = %d\n \n",quadrant);

  return out;
}

void eeprom_save_quadrants(float target){
  char targetStr_func[2];
  char message_func[2];
  uint8_t quadrant_func = 0;
  uint8_t quadTest = 0;
  if (target >= 0){
    if (target >= 0 && target < 90) {
        quadrant_func = 1;
    } 
    else if (target >= 90 && target < 180) {
        quadrant_func = 2;
    } 
    else if (target >= 180 && target < 270) {
        quadrant_func = 3;
    } 
  }

  if (target < 0){
    if (target <= 0 && target > -90) {
        quadrant_func = 4;
    } 
    else if (target <= -90 && target > -180) {
        quadrant_func = 3;
    } 
    else if (target <= -180 && target > -270) {
        quadrant_func = 2;
    } 
  }
  Serial.printf("quadrant_func = %d\n",quadrant_func);

  eeprom.write(20, &quadrant_func, sizeof(quadrant_func));
  delay(5);
  eeprom.read(20, &quadTest, sizeof(quadTest));
  delay(5);
  if (quadTest!= quadrant_func) Serial.println("Error saving quadrant_func to EEPROM");
}

//-----------------------End of AT24C256 function-----------------------

//-----------------------Start of function----------------------------
void CheckIPS(){     
  IPS = digitalRead(IPS1);
}

void Measure_angle_int(){
  CurrAngle_int = (as5600_1.getCumulativePosition() * AS5600_RAW_TO_DEGREES)*gearBox_ratio*pulley_ratio;
  oldPos = CurrAngle_int;
}

long Degree_to_Pusle(float degree){
  return degree*ratio_total/360;
}

float Pusle_to_Degree(long pos){
  return pos*360/ratio_total;
}

void Motion_Control(){
  if(I2C_parsing()== true){
    switch(cmd_flag){
      case Angle:{
        if (isTarget_inLimit(-targetDeg_Int)){
          stat_flag = Busy;
          AC_stepper.moveTo(Degree_to_Pusle(-targetDeg_Int));
          move_Issued = true;
          Timer_Encoder(true);
        }
        cmd_flag = 0;
        break;
      }

      case Linear:{
        if (isTarget_inLimit(-targetDeg_Int)){
          stat_flag = Busy;
          pending_linearTarget = Degree_to_Pusle(-targetDeg_Int);   
          AC_stepper.moveTo(pending_linearTarget);
          move_Issued = true;
          Timer_Encoder(true);
        }
        cmd_flag = 0;
        break;
      }

      case MoveJoystick: {
        if(isTarget_inLimit(targetDeg_Int) && isTargetOld(targetDeg_Int)){
          stat_flag = Busy;
          long deltaSteps = Degree_to_Pusle(targetDeg_Int);
          AC_stepper.moveTo(deltaSteps);
          Timer_Encoder(true);
        }
        cmd_flag = 0;
        break;
      }

      case Calib:{
        Timer_Encoder(false);
        offset = as5600_1.rawAngle() * AS5600_RAW_TO_DEGREES;
        as5600_1.setOffset(offset);  
        as5600_1.resetCumulativePosition();  
        AC_stepper.setCurrentPosition(0L);
        Measure_angle_int();
        cmd_flag = 0;
        stat_flag = Calibrated;
        isCalib = true;
        Timer_Encoder(true);
        break;
      }

      case Standby:{
        Serial.println("Standby J6");
        cmd_flag = 0;
        break;
      }
      
      case Move_Offset:{
        if (move_Issued == false && targetDeg_Int != 0.0){
          stat_flag = Busy;
          AC_stepper.setAcceleration(500); 
          AC_stepper.setSpeed(4000);
          long deltaSteps = Degree_to_Pusle(-targetDeg_Int);
          AC_stepper.move(deltaSteps);
          move_Issued = true;
          Timer_Encoder(true);
        }
        cmd_flag = 0;
        break;
      }
  
      case MoveHome:{
        if (isCalib == true){
          stat_flag = Busy;
          AC_stepper.moveTo(Degree_to_Pusle(0));
          move_Issued = true;
          Timer_Encoder(true);
        }
        cmd_flag = 0;
        break;
      }
      default:{
        break;
      }
    }
  } 
  else{
    // Serial.println("No new data");
  }
}

//-------------------------End of function----------------------------

