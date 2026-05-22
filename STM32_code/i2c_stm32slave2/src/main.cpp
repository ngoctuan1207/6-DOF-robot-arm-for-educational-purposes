//STM Slave 2
#include<Arduino.h>
#include<Wire.h>
#include<string.h>
#include<stdlib.h>
#include <AT24C256.h>
#include <math.h>
#include <../SERVO/Communication_Interface.h>

#define I2C1_clock                    1000000L
#define I2C2_clock                    400000L
#define SDA2                          PB11
#define SCL2                          PB10
#define stepPin1                      PA8
#define dirPin1                       PA9 //DIR HIGH is CW, LOW is CCW
#define enPin1                        PA10
#define IPS1                          PB0
#define SERIAL_RX_BUFFER_SIZE         1024
#define SERIAL_TX_BUFFER_SIZE         1024
#define homeSpeed                     4000
#define movingSpeed                   80000
#define angle_SR                      10000       // Angle sampling rate
#define MotorFlag_SR                  20000       // Angle sampling rate
#define ACCEL_TIME                    800       // Thời gian tăng tốc ms
#define DECEL_TIME                    800       // Thời gian gảim tốc ms
#define maxVel                        80000
#define Process_delay                 1500

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

const float J2_H = 111.0;
const float J2_L =-111.0;
const float J2_EH = 100.01/100;
const float J2_EL = 99.99/100;

TwoWire       Wire2(SDA2, SCL2);
AT24C256      eeprom(0x50, &Wire2);
HardwareTimer timer1(TIM1);
HardwareTimer timer2(TIM2);
HardwareTimer timer3(TIM3);
HardwareTimer timer4(TIM4);

//	DEFINE STATE
//===========================================================================
#define ID_2 2
#define ON_SERVO 1
#define OFF_SERVO 0
#define DELAY_TICK_ENABLE_SERVO 100
#define DELAY_TICK_DISABLE_SERVO 100
#define ACCELERATION_TIME_PARAMETER 3
#define DECELERATION_TIME_PARAMETER 4
#define NULL_POS -1
#define RETURN_OK 0
#define CW 1
#define CCW 0
#define FFLAG_MOTIONING       0x08000000UL
//===========================================================================

const uint8_t I2CSize = 30;
const uint8_t address_slave_arr[7] = {0x00,0x06,0x07,0x08,0x09,0x10,0x11};
const uint8_t address_slave = address_slave_arr[2];
byte i2C_tx_buffer[I2CSize];
volatile byte i2C_rx_buffer[I2CSize];
volatile bool Measure_flag = 0;
int eeAddress = 0;
volatile float targetDeg_Int;
volatile long targetVel;
volatile long targetAccel;
static float oldAcc = 0;
volatile uint8_t cmd_flag;
volatile uint8_t stat_flag;
uint8_t quadrant;
volatile int numByte;

bool calib_flag,status;
volatile float CurrAngle_int = 0;
volatile long pending_linearTarget;
static float CurrAngle;
static float offset_Zero;
static float oldPos;
volatile bool IPS; //Use for interrupt
// bool isRunning = 0;
// For Ezi motors
long POS_SERVO_2 = 0;
long VEL_SERVO_2 = 0; 
long lastest_position;
volatile bool movingFlag = 0;
volatile bool rxReady = false;
static bool move_Issued = false;
volatile bool isCalib = false;
const unsigned int Pulse_Per_Revolution_PPR = 10000;      //Constant number
const float pulley_ratio = 0.33;                          //20/60 pulley
const float Gearbox_2_Ratio = 0.0227;                     //Gearbox 1/44
const float ratio_total = pulley_ratio*Gearbox_2_Ratio;

//---------------------Start of I2C function-----------------------
void i2c_trans_angle(float target);
float i2c_receive_angle();
void Rx0Handler(int nBytes);
void Tx0Handler();
void TxBuff_reset();
void RxBuff_reset();
//-----------------------End of I2C function-----------------------

//-----------------------Start of function----------------------------
void CheckIPS();
void ENABLE_SERVO();
void DISABLE_SERVO();
float POS_TO_DEG_2_CONV(long pulse);
long DEG_TO_POS_2_CONV(float degree_target);
void CLEAR_POSITION();
void SET_POS_INIT(float theta2_init);
void SET_ACCELERATION_TIME(unsigned int accel_time);
void SET_DECELERATION_TIME(unsigned int decel_time);
void SAVE_PARAMETER();
unsigned int SPEED_SERVO_2(float degree_target, float degree_target_old, float time_target);
float GET_SPEED_2();
float GET_THETA_2();
bool GET_MOVING_FLAG_SERVO_2();
void MOVE_ORIGIN_SERVO_2();
void Joint_2_Calibration();
void Measure_angle_int();
void Timer_Encoder(bool On);
void Measure_angle_flag();
//-------------------------End of function----------------------------

//-----------------------Start of debug function----------------------
void CheckIPS_debug();
void Measure_angle_int_debug();
//-----------------------End of debug function------------------------

//---------------------Start of I2C function-----------------------
void Measure_angle_flag(){
  Measure_flag = 1;
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

//Decode angle from i2c RX package
float i2c_receive_angle(){
  float target;
  char targetStr[8];
  for(uint8_t i =1; i < 9; i++){
    //       0->7               1->8
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
          // 1->8              0->7
    i2C_tx_buffer[i] = targetStr[i-1];
  }  
  // Serial.print("TargetSt = ");
  // Serial.print(targetStr);
  // Serial.printf("\n vs target = %.2f \n", target);
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

bool isAccOld(float target){
  bool out = false;
  if (oldAcc == 0) {
    oldAcc = target;
    return out;
  }
  else{
    if((target == oldAcc)){
      out = true;
    }
    else oldAcc = target;
    return out;
  }
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
        targetVel = (i2c_receive_vel()/100) * maxVel;
        // if (move_Issued == true){
        //   SERVO_VelocityOverride(ID_2,targetVel);
        // }
      }
      else targetVel = maxVel/2;

      float Acc = i2c_receive_accel();
      // if (Acc!=0){
      if (Acc!=0 && !isAccOld(Acc)){
        targetAccel = Acc*1000;
        // targetAccel = Acc*1000*7;
        SET_ACCELERATION_TIME(targetAccel);
        // delayMicroseconds(Process_delay);
        SET_DECELERATION_TIME(targetAccel);
        // delayMicroseconds(Process_delay);
      }
      // else {
      //   targetAccel = ACCEL_TIME;
      //   SET_ACCELERATION_TIME(targetAccel);
      //   // delayMicroseconds(Process_delay);
      //   SET_DECELERATION_TIME(targetAccel);
      //   // delayMicroseconds(Process_delay);
      // }
      // Serial.printf("I2C_parsing J2: targetDeg_Int = %.2f, cmd_flag = %d,\n J2 targetVel = %ld, targetAccel/Decel = %ld \n\n", targetDeg_Int, cmd_flag, targetVel,targetAccel);
    } 
    // else targetDeg_Int = 370.0;
    // else if (cmd_flag == Standby) Serial.printf("I2C_parsing: Standby, cmd_flag = 0x%0x  \n", cmd_flag);
    // else if (cmd_flag == Move_Offset) move_Issued = false;
    RxBuff_reset();
    rxReady = false;
    return out;
  }
}

void I2C_PeriCheck(bool verbose){
  byte i,error=0;  
  byte address = 80; //0x36, 0x50
  int nDevices =0;
  int nDevicesLost =0;
  Wire2.beginTransmission(address);
  error = Wire2.endTransmission();
  if (error == 0) {
    if(verbose) Serial.println("I2C device found at address 0x80");
    nDevices++;
  }

  else if (error == 4) {
    if(verbose) Serial.println("Unknown error at address 0x80");
    stat_flag = LIM;
    nDevicesLost++;
  } 
  if (nDevicesLost == 0) stat_flag = Complete;
}

bool isTarget_inLimit(float target){
  bool out = false;
  if((target >= J2_L)&&(target <= J2_H)){
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

void Motion_Control(){
  // Serial.println("Begin Motion_Control");
  if(I2C_parsing()== true){
    // Serial.println("Begin Motion_Control: I2C_parsing()== true");
    switch(cmd_flag){
      case Angle:{
        if (isTarget_inLimit(targetDeg_Int)){
          stat_flag = Busy;
          SERVO_MoveSingleAxisAbsPos(ID_2,DEG_TO_POS_2_CONV(targetDeg_Int),targetVel);
          delayMicroseconds(Process_delay);
          move_Issued = true;
        }
        cmd_flag = 0;
        break;
      }

      case Move: {
        if(isTarget_inLimit(targetDeg_Int) && isTargetOld(targetDeg_Int)){
          stat_flag = Busy;
          SERVO_MoveSingleAxisAbsPos(ID_2,DEG_TO_POS_2_CONV(targetDeg_Int),80000);
          delayMicroseconds(Process_delay);
          move_Issued = true;
        }
        cmd_flag = 0;
        break;
      }

      case Calib:{
        Timer_Encoder(false);
        CLEAR_POSITION();
        delayMicroseconds(Process_delay);
        Measure_angle_int();
        delayMicroseconds(Process_delay);
        stat_flag = Calibrated;
        isCalib = true;
        Timer_Encoder(true);
        cmd_flag = 0;
        break;
      }

      case Standby:{
        // Serial.println("Standby J2");
        cmd_flag = 0;
        break;
      }
      
      case Move_Offset:{
        if (move_Issued == false && targetDeg_Int!= 0.0 && targetDeg_Int != 370){
          // SET_ACCELERATION_TIME(3000);
          // delayMicroseconds(Process_delay);
          // SET_DECELERATION_TIME(1000);
          // delayMicroseconds(Process_delay);
          stat_flag = Busy;
          SERVO_MoveSingleAxisIncPos(ID_2,DEG_TO_POS_2_CONV(targetDeg_Int),4000);
          move_Issued = true;
        }
        cmd_flag = 0;
        break;
      }

      case MoveHome:{
        if (isCalib == true){
          Serial.println("J2 Move home");
          if (move_Issued == false){
            stat_flag = Busy;
            SERVO_MoveSingleAxisAbsPos(ID_2,DEG_TO_POS_2_CONV(0),30000);
            delayMicroseconds(Process_delay);
            move_Issued = true;
          }
          else {
            SERVO_PositionAbsOverride(ID_2,DEG_TO_POS_2_CONV(0));
            SERVO_VelocityOverride(ID_2,30000);
          }
        }
        cmd_flag = 0;
        break;
      }
  
      case Linear:{
        if (isTarget_inLimit(targetDeg_Int) && move_Issued == false){
          // Serial.printf("J2_linearStart: targetDeg_Int = %.2f, J2_targetVel = %ld \n\n", targetDeg_Int, targetVel);
          stat_flag = Busy;
          pending_linearTarget = DEG_TO_POS_2_CONV(targetDeg_Int);
          SERVO_MoveSingleAxisAbsPos(ID_2,pending_linearTarget,targetVel);
          // delayMicroseconds(Process_delay);
          // delayMicroseconds(500);
          move_Issued = true;
        }
        else if (isTarget_inLimit(targetDeg_Int) && move_Issued == true){
          // stat_flag = Busy;
          // Serial.printf("J2_linear: targetDeg_Int = %.2f, J2_targetVel = %ld \n\n", targetDeg_Int, targetVel);
          pending_linearTarget = DEG_TO_POS_2_CONV(targetDeg_Int);
          // Serial.println("SERVO_PositionAbsOverride");
          SERVO_PositionAbsOverride(ID_2,pending_linearTarget);
          // delayMicroseconds(Process_delay);
          // Serial.println("SERVO_VelocityOverride");
          SERVO_VelocityOverride(ID_2,targetVel);
          // delayMicroseconds(Process_delay);
          // move_Issued = true;
        }

        cmd_flag = 0;
        break;
      }

      default:{
        break;
      }
    }
  } 
  else return;// Serial.println("End Motion_Control");
}

//-----------------------End of I2C function-----------------------

//-----------------------Start of function----------------------------
void CheckIPS(){     
  IPS = digitalRead(IPS1);
}

void ENABLE_SERVO()
{
  Serial.println("ENABLE SERVO ID 2 - JOINT 2 ENABLE");
  SERVO_ServoEnable(ID_2, ON_SERVO);
}

void DISABLE_SERVO()
{
  Serial.println("DISABLE SERVO ID 2 - JOINT 2 DISABLE");
  SERVO_ServoEnable(ID_2, OFF_SERVO);
}

float POS_TO_DEG_2_CONV(long pulse)
{
  float degree = (pulse * 360)  / (Pulse_Per_Revolution_PPR / ratio_total);
  return degree;
}

long DEG_TO_POS_2_CONV(float degree_target)
{
  long pulse = (degree_target * Pulse_Per_Revolution_PPR / ratio_total) / 360;
  return pulse;
}

void CLEAR_POSITION()
{
  SERVO_ClearPosition(ID_2);
}

void SET_POS_INIT(float theta2_init)
{
  SERVO_SetActualPos(ID_2, DEG_TO_POS_2_CONV(theta2_init));
}

void SET_ACCELERATION_TIME(unsigned int accel_time)
{
  SERVO_SetParameter(ID_2, ACCELERATION_TIME_PARAMETER, accel_time);
}

void SET_DECELERATION_TIME(unsigned int decel_time)
{
  SERVO_SetParameter(ID_2, DECELERATION_TIME_PARAMETER, decel_time);
}

void SAVE_PARAMETER()
{
  SERVO_SaveAllParameter(ID_2);
}

unsigned int SPEED_SERVO_2(float degree_target, float degree_target_old, float time_target)
{
    Serial.printf("Function 1 Received: degree_target = %0.2f, degree_target_old = %0.2f, time_target = %2d\n",
                  degree_target, degree_target_old, time_target);
    long pulse_Difference = abs(DEG_TO_POS_2_CONV(degree_target) - DEG_TO_POS_2_CONV(degree_target_old));
    Serial.printf("Function 1 Received: pulse_Difference = %8ld, time = %2u\n", pulse_Difference, time_target);
    unsigned int speed = pulse_Difference / time_target;
    Serial.printf("Speed_1 = %ld\n\n", speed);
    return speed;
}

float GET_SPEED_2()
{
    SERVO_GetActualVel(ID_2, &VEL_SERVO_2);                                                             // pps
    float angular_Velocity = (float)(VEL_SERVO_2 * 360) / (Pulse_Per_Revolution_PPR / ratio_total); // dps = (pps*360)/PPR_Total
    return angular_Velocity;
}

float GET_THETA_2()
{
  long Curr_Pos = 0;
  SERVO_GetActualPos(ID_2, &Curr_Pos);
  float theta_Servo = (float)(Curr_Pos * 360) / (Pulse_Per_Revolution_PPR / ratio_total);
  return theta_Servo;
}

void MOVE_ORIGIN_SERVO_2()
{
  do
  {
    SERVO_MoveVelocity(ID_2, homeSpeed * 2, CW);
  } while (digitalRead(IPS1) == true);
  SERVO_MoveStop(ID_2);
  long CW_Pos = 0;
  SERVO_GetActualPos(ID_2, &CW_Pos);
  // Serial.printf("CW_Pos: %ld\n",CW_Pos);
  delayMicroseconds(Process_delay);

  do
  {
    SERVO_MoveVelocity(ID_2, homeSpeed * 2, CCW);
  } while (digitalRead(IPS1) == false);

  do
  {
    SERVO_MoveVelocity(ID_2, homeSpeed * 2, CCW);
  } while (digitalRead(IPS1) == true);
  SERVO_MoveStop(ID_2);
  long CCW_Pos = 0;
  SERVO_GetActualPos(ID_2, &CCW_Pos);
  // Serial.printf("CCW_Pos: %ld\n",CCW_Pos);
  // Serial.printf("Diff Pos: %ld\n", abs(CW_Pos - CCW_Pos));
  delayMicroseconds(Process_delay);
  
  SERVO_MoveSingleAxisIncPos(ID_2, (long)(round((abs(CW_Pos - CCW_Pos))/2)), (unsigned long)homeSpeed * 2);
  // Serial.println("Move to origin");
  SERVO_ClearPosition(ID_2);
}

bool GET_MOVING_FLAG_SERVO_2(){
  bool stat = 0;
  unsigned long AxisStatus;
  SERVO_GetAxisStatus(ID_2, &AxisStatus);
  // delayMicroseconds(2500);
  if((AxisStatus & FFLAG_MOTIONING) != 0) stat = 1;
  // Serial.printf("Status: %d \n",stat);
  return stat;
}

void Joint_2_Calibration(){
  if (digitalRead(IPS1) == true)
  {
    // Serial.println("IF IPS1 == TRUE");
      MOVE_ORIGIN_SERVO_2();
  }
  else
  {
      long Act_Pos = 0;
      float Theta_Start_Move = 0;
      float Theta_Moving = 0;
      SERVO_GetActualPos(ID_2, &Act_Pos);
      // Serial.printf("Act Pos: %ld\n", Act_Pos);
      Theta_Start_Move = GET_THETA_2();
      if (Act_Pos <= 0)
      {
        // Serial.println("Act_Pos <= 0");
          do
          {
              SERVO_MoveVelocity(ID_2, homeSpeed * 2, CCW);
              if (Theta_Moving <= 30)
              {
                // Serial.println("Theta_Moving <= 30,CW");
                Theta_Moving = abs(Theta_Start_Move - GET_THETA_2());
              }

              else
              {
                // Serial.println("Theta_Moving > 30,CCW");
                SERVO_MoveStop(ID_2);
                do
                {
                  SERVO_MoveVelocity(ID_2, homeSpeed * 2, CW);
                } while (digitalRead(IPS1) == false);
              }
          } while (digitalRead(IPS1) == false);
          SERVO_MoveStop(ID_2);
          MOVE_ORIGIN_SERVO_2();
          Theta_Moving = 0;
      }
      else
      {
        // Serial.println("Act_Pos > 0");
          do
          {
              SERVO_MoveVelocity(ID_2, homeSpeed * 2, CW);
              if (Theta_Moving <= 30)
              {
                // Serial.println("Theta_Moving <= 30,CCW");
                Theta_Moving = abs(Theta_Start_Move - GET_THETA_2());
              }

              else
              {
                // Serial.println("Theta_Moving > 30,CW");
                SERVO_MoveStop(ID_2);

                do
                {
                  SERVO_MoveVelocity(ID_2, homeSpeed * 2, CCW);
                } while (digitalRead(IPS1) == false);
              }
          } while (digitalRead(IPS1) == false);
          SERVO_MoveStop(ID_2);
          MOVE_ORIGIN_SERVO_2();
          Theta_Moving = 0;
      }
  }
}

void Measure_angle_int(){
  CurrAngle_int = GET_THETA_2();
  Measure_flag = 0;
}

//-------------------------End of function----------------------------

//-----------------------Start of debug function----------------------
void CheckIPS_debug(){
  IPS = digitalRead(IPS1);
  Serial.println(IPS);
}

void Measure_angle_int_debug(){
  CurrAngle_int = GET_THETA_2();
  Serial.printf("CurrAngle_int = %.2f, IPS = %d \n",CurrAngle_int,IPS);
}

void Joint_2_Calibration_debug(){
  if (digitalRead(IPS1) == true)
  {
    Serial.println("IF IPS1 == TRUE");
      MOVE_ORIGIN_SERVO_2();
  }
  else
  {
      long Act_Pos = 0;
      float Theta_Start_Move = 0;
      float Theta_Moving = 0;
      SERVO_GetActualPos(ID_2, &Act_Pos);
      Serial.printf("Act Pos: %ld\n", Act_Pos);
      Theta_Start_Move = GET_THETA_2();
      if (Act_Pos <= 0)
      {
        Serial.println("Act_Pos <= 0");
          do
          {
              SERVO_MoveVelocity(ID_2, homeSpeed * 2, CCW);
              if (Theta_Moving <= 30)
              {
                Serial.println("Theta_Moving <= 30,CW");
                Theta_Moving = abs(Theta_Start_Move - GET_THETA_2());
              }

              else
              {
                Serial.println("Theta_Moving > 30,CCW");
                SERVO_MoveStop(ID_2);
                do
                {
                  SERVO_MoveVelocity(ID_2, homeSpeed * 2, CW);
                } while (digitalRead(IPS1) == false);
              }
          } while (digitalRead(IPS1) == false);
          SERVO_MoveStop(ID_2);
          MOVE_ORIGIN_SERVO_2();
          Theta_Moving = 0;
      }
      else
      {
        Serial.println("Act_Pos > 0");
          do
          {
              SERVO_MoveVelocity(ID_2, homeSpeed * 2, CW);
              if (Theta_Moving <= 30)
              {
                Serial.println("Theta_Moving <= 30,CCW");
                Theta_Moving = abs(Theta_Start_Move - GET_THETA_2());
              }

              else
              {
                Serial.println("Theta_Moving > 30,CW");
                SERVO_MoveStop(ID_2);

                do
                {
                  SERVO_MoveVelocity(ID_2, homeSpeed * 2, CCW);
                } while (digitalRead(IPS1) == false);
              }
          } while (digitalRead(IPS1) == false);
          SERVO_MoveStop(ID_2);
          MOVE_ORIGIN_SERVO_2();
          Theta_Moving = 0;
      }
  }
}

void MOVE_ORIGIN_SERVO_2_debug()
{
  do
  {
    SERVO_MoveVelocity(ID_2, homeSpeed * 2, CW);
  } while (digitalRead(IPS1) == true);
  SERVO_MoveStop(ID_2);
  long CW_Pos = 0;
  SERVO_GetActualPos(ID_2, &CW_Pos);
  Serial.printf("CW_Pos: %ld\n",CW_Pos);
  delayMicroseconds(Process_delay);

  do
  {
    SERVO_MoveVelocity(ID_2, homeSpeed * 2, CCW);
  } while (digitalRead(IPS1) == false);

  do
  {
    SERVO_MoveVelocity(ID_2, homeSpeed * 2, CCW);
  } while (digitalRead(IPS1) == true);
  SERVO_MoveStop(ID_2);
  long CCW_Pos = 0;
  SERVO_GetActualPos(ID_2, &CCW_Pos);
  Serial.printf("CCW_Pos: %ld\n",CCW_Pos);
  Serial.printf("Diff Pos: %ld\n", abs(CW_Pos - CCW_Pos));
  delayMicroseconds(Process_delay);
  
  SERVO_MoveSingleAxisIncPos(ID_2, (long)(round((abs(CW_Pos - CCW_Pos))/2)), (unsigned long)homeSpeed * 2);
  Serial.println("Move to origin");
  SERVO_ClearPosition(ID_2);
}

//-----------------------End of debug function------------------------

void setup() {
  Serial2.begin(115200,SERIAL_8N1);
  Serial.begin(115200);
  delay(5000);

  Wire.begin(address_slave);
  Wire.onReceive(Rx0Handler);
  Wire.onRequest(Tx0Handler);
  Wire.setClock(I2C1_clock);
  Wire2.begin();
  Wire2.setClock(I2C2_clock); 
  delay(5000);

  ENABLE_SERVO();
  delayMicroseconds(Process_delay);
  SET_ACCELERATION_TIME(ACCEL_TIME);
  delayMicroseconds(Process_delay);
  SET_DECELERATION_TIME(DECEL_TIME);
  delayMicroseconds(Process_delay);
  SAVE_PARAMETER();
  delayMicroseconds(Process_delay);
  CLEAR_POSITION();

  IPS = digitalRead(IPS1);
  attachInterrupt(digitalPinToInterrupt(IPS1), CheckIPS, CHANGE); //ENABLE ALWAYS WATCH FOR HOME
  stat_flag = LIM;
  Timer_Encoder(true);
  Serial.printf("J2 on, Home = %d \n",IPS);
  Serial.printf("Speed = %d; Accel =%d; stat_flag = 0x%02x\n\n", movingSpeed, ACCEL_TIME, stat_flag);
}


void loop() {
  Motion_Control();

  if (move_Issued == true){ 
    if (Measure_flag == 1) {
      Measure_angle_int();
      delayMicroseconds(100);
    }
    
    movingFlag = GET_MOVING_FLAG_SERVO_2();
    delayMicroseconds(100);

    if (movingFlag == 0){
      Measure_angle_int();  
      delayMicroseconds(100);
      stat_flag = Complete;
      move_Issued = false;
    } 
  }
}





