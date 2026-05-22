//STM Slave 1
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
#define maxVel          10000

const float J1_H = 181.0;
const float J1_L =-181.0;
const float J1_EH = 100.1/100;
const float J1_EL = 99.9/100;

const float offset_Angle = 1.0;
float offset_Zero;
const uint8_t I2CSize = 30;

const float pulley_ratio = 0.25;
const float gearBox_ratio = 0.07;
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
const uint8_t address_slave = address_slave_arr[1];
byte i2C_tx_buffer[I2CSize];
volatile byte i2C_rx_buffer[I2CSize];
volatile bool Measure_flag = 0;
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
void Measure_angle_int();
void moveHome_lib_int();
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
  Wire2.setTimeout(5); // 5 ms
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
  as5600_1.resetCumulativePosition();  
  Timer_Encoder(true);
  Serial.printf("J1 ON, Home stat = %d \n",IPS);
  stat_flag = LIM;
}


void loop() {
  Motion_Control();
  bool stat = AC_stepper.run();
  if (stat == 0) {
    if (move_Issued == true) {
      stat_flag = Complete;
      move_Issued = false;
    }
  }
  else{
    if (Measure_flag == 1) {
      Measure_angle_int();
    }
  }
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
        // AC_stepper.setMaxSpeed(targetVel);
      }
      else {
        targetVel = maxVel/2;
        // AC_stepper.setMaxSpeed(targetVel);
      }

      float Acc = i2c_receive_accel();
      if (Acc!=0){
        targetAccel = targetVel/Acc;
        // AC_stepper.setAcceleration(targetAccel);
      }
      else  {
        targetAccel = 1200;
        // AC_stepper.setAcceleration(targetAccel);
      }

      // Serial.printf("I2C_parsing J1: targetDeg_Int = %.2f, cmd_flag = %d, \n J1: targetVel = %.2f, targetAccel = %.2f \n\n", targetDeg_Int, cmd_flag, targetVel,targetAccel);
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
  if((target >= J1_L)&&(target <= J1_H)){
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
void Measure_angle_flag(){
  Measure_flag = 1;
}

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
    timer2.attachInterrupt(Measure_angle_flag); 
    timer2.resume();
  }
  else timer2.detachInterrupt();
}
//-----------------------End of AS5600 function-----------------------

//-----------------------Start of function----------------------------
void CheckIPS(){     
  IPS = digitalRead(IPS1);
}

void Measure_angle_int(){
  CurrAngle_int = (as5600_1.getCumulativePosition() * AS5600_RAW_TO_DEGREES)*gearBox_ratio*pulley_ratio;
  oldPos = CurrAngle_int;
  Measure_flag = 0;
}

long Degree_to_Pusle(float degree){
  return degree*ratio_total/360;
}

float Pusle_to_Degree(long pos){
  return pos*360/ratio_total;
}

void moveHome_lib_int(){
  float CCW, CW, home, offset, currAngle;
  bool IPS_func = digitalRead(IPS1);
  if(IPS_func == 1){
    // Timer_Encoder(true);
    as5600_1.resetCumulativePosition();  

    AC_stepper.setCurrentPosition(0L);
    long deltaSteps = Degree_to_Pusle(10.0f);
    AC_stepper.move(deltaSteps);
    while(IPS_func == 1){
      AC_stepper.run();
      IPS_func = digitalRead(IPS1);
      if (IPS_func == 0){
        CW = CurrAngle_int;
        AC_stepper.stop();
        while(AC_stepper.distanceToGo() != 0){
          AC_stepper.run();
          if (AC_stepper.distanceToGo() == 0) break;
        }
        break;
      }
    }
    AC_stepper.runToNewPosition(-1*deltaSteps);

    IPS_func = digitalRead(IPS1);
    while(IPS_func == 0){
      AC_stepper.run();
      IPS_func = digitalRead(IPS1);
      if (IPS_func == 1){
        CCW = CurrAngle_int;
        Serial.printf("CCW: %.2f \n", CCW);
        AC_stepper.stop();
        while(AC_stepper.distanceToGo() != 0){
          AC_stepper.run();
          if (AC_stepper.distanceToGo() == 0) break;
        }
        break;
      }
    }

    // bool IPS_hold = IPS_func;
    // bool IPS_prev = 0;
    // deltaSteps = Degree_to_Pusle(-20.0f);
    // AC_stepper.move(deltaSteps);
    // Serial.printf("IPS_hold bf Turn CCW = %d \n",IPS_hold);
    // Serial.println("Turn CCW");

    // while(IPS_hold == 0){
    //   AC_stepper.run();
    //   IPS_func = digitalRead(IPS1);
    //   if (IPS_func != IPS_hold) IPS_prev = IPS_func;
    //   else{

    //   }
    //   if (IPS_func == 0 && IPS_prev ==1) {
    //     Serial.println("Stop CCW");
    //     CCW = CurrAngle_int;
    //     Serial.printf("CCW: %.2f \n", CCW);
    //     AC_stepper.stop();
    //     while(AC_stepper.distanceToGo() != 0){
    //       AC_stepper.run();
    //       if (AC_stepper.distanceToGo() == 0) break;
    //     }
    //     break;
    //   }
    // }

    home = CW + (abs(CW-CCW)/2);
    Serial.printf("home: %.2f \n", home);
    currAngle = CurrAngle_int;

    float delta = currAngle - home;
    Serial.printf("currAngle: %.2f, delta = %.2f \n", currAngle, delta);
    AC_stepper.move(Degree_to_Pusle((delta)));
    while (AC_stepper.distanceToGo() != 0){
      AC_stepper.run();
      if (AC_stepper.distanceToGo() == 0) break;
    }
    offset = as5600_1.rawAngle() * AS5600_RAW_TO_DEGREES;
    as5600_1.setOffset(offset);  
    Serial.printf("offset: %.2f \n", offset);
    Serial.printf("Magnet angle : %.2f \n", (as5600_1.readAngle() * AS5600_RAW_TO_DEGREES));
    as5600_1.resetCumulativePosition();  
    AC_stepper.setCurrentPosition(0L);
    Measure_angle_int();
    oldPos = CurrAngle_int;    
    Serial.printf("Angle After Calib: %.2f, CurrentPosition = %ld \n",oldPos,AC_stepper.currentPosition());
    // Timer_Encoder(false);
  }
}

void Motion_Control(){
  if(I2C_parsing()== true){
    switch(cmd_flag){
      case Angle:{
        if (isTarget_inLimit(-targetDeg_Int)){
        // if (move_Issued == false && (isTarget_inLimit(-targetDeg_Int) && isTargetOld(-targetDeg_Int))){
          // AC_stepper.setAcceleration(1200);
          // Serial.printf("J1_Angle_tar = %.2f \n",-targetDeg_Int);
          stat_flag = Busy;
          AC_stepper.setMaxSpeed(targetVel);        
          AC_stepper.setAcceleration(targetAccel);    
          AC_stepper.moveTo(Degree_to_Pusle(-targetDeg_Int));
          move_Issued = true;
          // Timer_Encoder(true);
          // cmd_flag = 0;
        }
        cmd_flag = 0;
        break;
      }

      case Linear:{
        if (isTarget_inLimit(-targetDeg_Int)){
          // Serial.printf("J1_linear_tar = %.2f \n",-targetDeg_Int);
          stat_flag = Busy;
          pending_linearTarget = Degree_to_Pusle(-targetDeg_Int);
          move_Issued = true;
          // hasPending_linearTarget = true;
          AC_stepper.setMaxSpeed(targetVel);        
          AC_stepper.setAcceleration(targetAccel);    
          AC_stepper.moveTo(pending_linearTarget);
          // Timer_Encoder(true);
        }
        cmd_flag = 0;
        break;
      }

      // case MoveJoystick: {
      //   if(isTarget_inLimit(targetDeg_Int) && isTargetOld(targetDeg_Int)){
      //     stat_flag = Busy;
      //     long deltaSteps = Degree_to_Pusle(targetDeg_Int);
      //     AC_stepper.moveTo(deltaSteps);
      //     // Serial.printf("Move to abs pos: %.2f \n",targetDeg_Int);
      //   }
      //   cmd_flag = 0;
      //   break;
      // }

      case Calib:{
        Timer_Encoder(false);
        // Serial.println("case Calib");
        offset = as5600_1.rawAngle() * AS5600_RAW_TO_DEGREES;
        as5600_1.setOffset(offset);  
        // Serial.printf("offset: %.2f \n", offset);
        // Serial.printf("Magnet angle : %.2f \n", (as5600_1.readAngle() * AS5600_RAW_TO_DEGREES));
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
        // Serial.println("Standby J1");
        cmd_flag = 0;
        break;
      }
      
      case Move_Offset:{
        if (move_Issued == false && targetDeg_Int!= 0.0){
          // Serial.printf("J1_offset_tar = %.2f \n",targetDeg_Int);
          stat_flag = Busy;
          AC_stepper.setAcceleration(500); 
          AC_stepper.setSpeed(4000);
          long deltaSteps = Degree_to_Pusle(-targetDeg_Int);
          AC_stepper.move(deltaSteps);
          // Timer_Encoder(true);
          move_Issued = true;
        }
        cmd_flag = 0;
        break;
      }
  
      case MoveHome:{
        if (isCalib == true){
          // Serial.println("Move home J1");
          stat_flag = Busy;
          // AC_stepper.runToNewPosition(0);
          AC_stepper.setMaxSpeed(targetVel);        
          AC_stepper.setAcceleration(targetAccel);    
          AC_stepper.moveTo(Degree_to_Pusle(0));
          move_Issued = true;
          // Timer_Encoder(true);
          // oldPos = CurrAngle_int;
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

