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

const float J3_H = 71.0;
const float J3_L =-111.0;
const float J3_EH = 100.01/100;
const float J3_EL = 99.99/100;

TwoWire       Wire2(SDA2, SCL2);
AT24C256      eeprom(0x50, &Wire2);
HardwareTimer timer1(TIM1);
HardwareTimer timer2(TIM2);
HardwareTimer timer3(TIM3);
HardwareTimer timer4(TIM4);

//	DEFINE STATE
//===========================================================================

#define ID_3 3
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
const uint8_t address_slave = address_slave_arr[3];
byte i2C_tx_buffer[I2CSize];
volatile byte i2C_rx_buffer[I2CSize];
volatile bool Measure_flag = 0;
int eeAddress = 0;

volatile float targetDeg_Int;
volatile long targetVel;
volatile float targetAccel;
static float oldAcc = 0;
volatile uint8_t cmd_flag;
volatile uint8_t stat_flag;
uint8_t quadrant;
volatile int numByte;

bool calib_flag,status;
float volatile CurrAngle_int;
volatile long pending_linearTarget;
static float CurrAngle;
static float offset_Zero;
static float oldPos;
volatile bool IPS; //Use for interrupt

// For Ezi motors
long POS_SERVO_3 = 0;
long VEL_SERVO_3 = 0; 
long lastest_position;
volatile bool movingFlag = 0;
volatile bool rxReady = false;
static bool move_Issued = false;
volatile bool isCalib = false;
unsigned int Pulse_Per_Revolution_PPR = 10000;            //Constant number
const float pulley_ratio = 1;                             //No pulley 
const float Gearbox_3_Ratio = 0.0211;                     //Gearbox 1/47.2
const float ratio_total = pulley_ratio*Gearbox_3_Ratio;

//---------------------Start of I2C function-----------------------
void i2c_trans_angle(float target);
float i2c_receive_angle();
void Rx0Handler(int nBytes);
void Tx0Handler();
void TxBuff_reset();
void RxBuff_reset();
//-----------------------End of I2C function-----------------------

//---------------------Start of AT24C256 function-----------------------
float eeprom_get_angle();
bool eeprom_save(float target, float offset, bool stat_calib);
bool eeprom_save_angle(float target);
void eeprom_get();
void eeprom_save_quadrants(float target);
//-----------------------End of AT24C256 function-----------------------

//-----------------------Start of function----------------------------
void CheckIPS();
void ENABLE_SERVO();
void DISABLE_SERVO();
float POS_TO_DEG_3_CONV(long pulse);
long DEG_TO_POS_3_CONV(float degree_target);
void CLEAR_POSITION();
void SET_POS_INIT(float theta3_init);
void SET_ACCELERATION_TIME(unsigned int accel_time);
void SET_DECELERATION_TIME(unsigned int decel_time);
void SAVE_PARAMETER();
unsigned int SPEED_SERVO_3(float degree_target, float degree_target_old, float time_target);
float GET_SPEED_3();
float GET_THETA_3();
bool GET_MOVING_FLAG_SERVO_3();
void MOVE_ORIGIN_SERVO_3();
void Joint_3_Calibration();
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
  char targetStr[7];
  for(uint8_t i =1; i < 9; i++){
    targetStr[i-1] = i2C_rx_buffer[i];
  }
  target = atof(targetStr);
  return target;
}

//Encode angle to put in i2c TX package
void i2c_trans_angle(float target){
  char targetStr[7];
  dtostrf(target, 7, 2, targetStr);
  for(uint8_t i = 1; i < 9; i++){
    i2C_tx_buffer[i] = targetStr[i-1];
  }  
}

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
        targetVel = i2c_receive_vel()/100 * maxVel;
      }
      else targetVel = maxVel/2;

      float Acc = i2c_receive_accel();
      if (Acc!=0 && !isAccOld(Acc)){
        // targetAccel = Acc*1000*7;
        targetAccel = Acc*1000;
        SET_ACCELERATION_TIME(targetAccel);
        // delayMicroseconds(Process_delay);
        SET_DECELERATION_TIME(targetAccel);
        // delayMicroseconds(Process_delay);
      }
      // else  {
      //   targetAccel = ACCEL_TIME;
      //   SET_ACCELERATION_TIME(targetAccel);
      //   // delayMicroseconds(Process_delay);
      //   SET_DECELERATION_TIME(targetAccel);
      //   // delayMicroseconds(Process_delay);
      // }
      // Serial.printf("I2C_parsing J3: targetDeg_Int = %.2f, cmd_flag = 0x%0x,\n J3 targetVel = %.2f, targetAccel = %.2f, out = %d \n\n", targetDeg_Int, cmd_flag, targetVel,targetAccel,out);
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
  if((target >= J3_L)&&(target <= J3_H)){
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
          SERVO_MoveSingleAxisAbsPos(ID_3,DEG_TO_POS_3_CONV(targetDeg_Int),targetVel);
          delayMicroseconds(Process_delay);
          move_Issued = true;
        }
        cmd_flag = 0;
        break;
      }

      case Move: {
        if(isTarget_inLimit(targetDeg_Int) && isTargetOld(targetDeg_Int)){
          stat_flag = Busy;
          SERVO_MoveSingleAxisAbsPos(ID_3,DEG_TO_POS_3_CONV(targetDeg_Int),80000);
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
        cmd_flag = 0;
        stat_flag = Calibrated;
        Timer_Encoder(true);
        isCalib = true;
        break;
      }

      case Standby:{
        Serial.println("Standby J3");
        break;
      }
      
      case Move_Offset:{
        if (move_Issued == false && targetDeg_Int!= 0.0 && targetDeg_Int != 370){
          // SET_ACCELERATION_TIME(3000);
          // SET_DECELERATION_TIME(1000);
          stat_flag = Busy;
          SERVO_MoveSingleAxisIncPos(ID_3,DEG_TO_POS_3_CONV(targetDeg_Int),4000);
          move_Issued = true;
        }
        cmd_flag = 0;
        break;
      }

      case MoveHome:{
        if (isCalib == true){
          Serial.println("J3 Move home");
          if (move_Issued == false){
            stat_flag = Busy;
            SERVO_MoveSingleAxisAbsPos(ID_3,DEG_TO_POS_3_CONV(0),30000);
            delayMicroseconds(Process_delay);
            move_Issued = true;
          }
          else{
            SERVO_PositionAbsOverride(ID_3,DEG_TO_POS_3_CONV(0));
            SERVO_VelocityOverride(ID_3,30000);
          }
        }
        cmd_flag = 0;
        break;
      }

      case Linear:{
        if (isTarget_inLimit(targetDeg_Int) && move_Issued == false){
          // Serial.printf("J3_linearStart: targetDeg_Int = %.2f, J3_targetVel = %ld \n\n", targetDeg_Int, targetVel);
          stat_flag = Busy;
          pending_linearTarget = DEG_TO_POS_3_CONV(targetDeg_Int);
          SERVO_MoveSingleAxisAbsPos(ID_3,pending_linearTarget,targetVel);
          // delayMicroseconds(Process_delay);
          move_Issued = true;
        }
        else if (isTarget_inLimit(targetDeg_Int) && move_Issued == true){
          // stat_flag = Busy;
          // Serial.printf("J3_linear: targetDeg_Int = %.2f, J3_targetVel = %ld \n\n", targetDeg_Int, targetVel);
          pending_linearTarget = DEG_TO_POS_3_CONV(targetDeg_Int);
          SERVO_PositionAbsOverride(ID_3,pending_linearTarget);
          SERVO_VelocityOverride(ID_3,targetVel);
          // delayMicroseconds(Process_delay);

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
  // Serial.println("End Motion_Control");
}

//-----------------------End of I2C function-----------------------

//-----------------------Start of function----------------------------
void CheckIPS(){     
  IPS = digitalRead(IPS1);
}

void ENABLE_SERVO()
{
  Serial.println("ENABLE SERVO ID 3 - JOINT 3 ENABLE");
  SERVO_ServoEnable(ID_3, ON_SERVO);
}

void DISABLE_SERVO()
{
  Serial.println("DISABLE SERVO ID 3 - JOINT 3 DISABLE");
  SERVO_ServoEnable(ID_3, OFF_SERVO);
}

float POS_TO_DEG_3_CONV(long pulse)
{
  float degree = (pulse * 360)  / (Pulse_Per_Revolution_PPR / ratio_total);
  return degree;
}

long DEG_TO_POS_3_CONV(float degree_target)
{
  long pulse = (degree_target * Pulse_Per_Revolution_PPR / ratio_total) / 360;
  return pulse;
}

void CLEAR_POSITION()
{
  SERVO_ClearPosition(ID_3);
}

void SET_POS_INIT(float theta3_init)
{
  SERVO_SetActualPos(ID_3, DEG_TO_POS_3_CONV(theta3_init));
}

void SET_ACCELERATION_TIME(unsigned int accel_time)
{
  SERVO_SetParameter(ID_3, ACCELERATION_TIME_PARAMETER, accel_time);
}

void SET_DECELERATION_TIME(unsigned int decel_time)
{
  SERVO_SetParameter(ID_3, DECELERATION_TIME_PARAMETER, decel_time);
}

void SAVE_PARAMETER()
{
  SERVO_SaveAllParameter(ID_3);
}

unsigned int SPEED_SERVO_3(float degree_target, float degree_target_old, float time_target)
{
    Serial.printf("Function 1 Received: degree_target = %0.2f, degree_target_old = %0.2f, time_target = %2d\n",
                  degree_target, degree_target_old, time_target);
    long pulse_Difference = abs(DEG_TO_POS_3_CONV(degree_target) - DEG_TO_POS_3_CONV(degree_target_old));
    Serial.printf("Function 1 Received: pulse_Difference = %8ld, time = %2u\n", pulse_Difference, time_target);
    unsigned int speed = pulse_Difference / time_target;
    Serial.printf("Speed_1 = %ld\n\n", speed);
    return speed;
}

float GET_SPEED_3()
{
    SERVO_GetActualVel(ID_3, &VEL_SERVO_3);                                                             // pps
    float angular_Velocity = (float)(VEL_SERVO_3 * 360) / (Pulse_Per_Revolution_PPR / ratio_total); // dps = (pps*360)/PPR_Total
    return angular_Velocity;
}

float GET_THETA_3()
{
  long Curr_Pos = 0;
  SERVO_GetActualPos(ID_3, &Curr_Pos);
  float theta_Servo = (float)(Curr_Pos * 360) / (Pulse_Per_Revolution_PPR / ratio_total);
  return theta_Servo;
}

void MOVE_ORIGIN_SERVO_3()
{
  do
  {
    SERVO_MoveVelocity(ID_3, homeSpeed * 2, CW);
  } while (digitalRead(IPS1) == true);
  SERVO_MoveStop(ID_3);
  long CW_Pos = 0;
  SERVO_GetActualPos(ID_3, &CW_Pos);
  // Serial.printf("CW_Pos: %ld\n",CW_Pos);
  delayMicroseconds(Process_delay);

  do
  {
    SERVO_MoveVelocity(ID_3, homeSpeed * 2, CCW);
  } while (digitalRead(IPS1) == false);

  do
  {
    SERVO_MoveVelocity(ID_3, homeSpeed * 2, CCW);
  } while (digitalRead(IPS1) == true);
  SERVO_MoveStop(ID_3);
  long CCW_Pos = 0;
  SERVO_GetActualPos(ID_3, &CCW_Pos);
  // Serial.printf("CCW_Pos: %ld\n",CCW_Pos);
  // Serial.printf("Diff Pos: %ld\n", abs(CW_Pos - CCW_Pos));
  delayMicroseconds(Process_delay);
  
  SERVO_MoveSingleAxisIncPos(ID_3, (long)(round((abs(CW_Pos - CCW_Pos))/2)), (unsigned long)homeSpeed * 2);
  // Serial.println("Move to origin");
  SERVO_ClearPosition(ID_3);
}

bool GET_MOVING_FLAG_SERVO_3(){
  bool stat = 0;
  unsigned long AxisStatus;
  // byte MOVING_BIT;
  SERVO_GetAxisStatus(ID_3, &AxisStatus);
  // delayMicroseconds(500);
  if((AxisStatus & FFLAG_MOTIONING) != 0) stat = 1;
  // Serial.printf("Status: %d \n",stat);
  return stat;
}

void Joint_3_Calibration(){
  if (digitalRead(IPS1) == true)
  {
    // Serial.println("IF IPS1 == TRUE");
      MOVE_ORIGIN_SERVO_3();
  }
  else
  {
      long Act_Pos = 0;
      float Theta_Start_Move = 0;
      float Theta_Moving = 0;
      SERVO_GetActualPos(ID_3, &Act_Pos);
      // Serial.printf("Act Pos: %ld\n", Act_Pos);
      Theta_Start_Move = GET_THETA_3();
      if (Act_Pos <= 0)
      {
        // Serial.println("Act_Pos <= 0");
          do
          {
              SERVO_MoveVelocity(ID_3, homeSpeed * 2, CCW);
              if (Theta_Moving <= 30)
              {
                // Serial.println("Theta_Moving <= 30,CW");
                Theta_Moving = abs(Theta_Start_Move - GET_THETA_3());
              }

              else
              {
                // Serial.println("Theta_Moving > 30,CCW");
                SERVO_MoveStop(ID_3);
                do
                {
                  SERVO_MoveVelocity(ID_3, homeSpeed * 2, CW);
                } while (digitalRead(IPS1) == false);
              }
          } while (digitalRead(IPS1) == false);
          SERVO_MoveStop(ID_3);
          MOVE_ORIGIN_SERVO_3();
          Theta_Moving = 0;
      }
      else
      {
        // Serial.println("Act_Pos > 0");
          do
          {
              SERVO_MoveVelocity(ID_3, homeSpeed * 2, CW);
              if (Theta_Moving <= 30)
              {
                // Serial.println("Theta_Moving <= 30,CCW");
                Theta_Moving = abs(Theta_Start_Move - GET_THETA_3());
              }

              else
              {
                // Serial.println("Theta_Moving > 30,CW");
                SERVO_MoveStop(ID_3);

                do
                {
                  SERVO_MoveVelocity(ID_3, homeSpeed * 2, CCW);
                } while (digitalRead(IPS1) == false);
              }
          } while (digitalRead(IPS1) == false);
          SERVO_MoveStop(ID_3);
          MOVE_ORIGIN_SERVO_3();
          Theta_Moving = 0;
      }
  }
}

void Measure_angle_int(){
  CurrAngle_int = GET_THETA_3();
  Measure_flag = 0;
}

//-------------------------End of function----------------------------

//-----------------------Start of debug function----------------------
void CheckIPS_debug(){
  IPS = digitalRead(IPS1);
  Serial.println(IPS);
}

void Measure_angle_int_debug(){
  CurrAngle_int = GET_THETA_3();
  Serial.printf("CurrAngle_int = %.2f, IPS = %d \n",CurrAngle_int,IPS);
}

void Joint_3_Calibration_debug(){
  if (digitalRead(IPS1) == true)
  {
    Serial.println("IF IPS1 == TRUE");
      MOVE_ORIGIN_SERVO_3();
  }
  else
  {
      long Act_Pos = 0;
      float Theta_Start_Move = 0;
      float Theta_Moving = 0;
      SERVO_GetActualPos(ID_3, &Act_Pos);
      Serial.printf("Act Pos: %ld\n", Act_Pos);
      Theta_Start_Move = GET_THETA_3();
      if (Act_Pos <= 0)
      {
        Serial.println("Act_Pos <= 0");
          do
          {
              SERVO_MoveVelocity(ID_3, homeSpeed * 2, CCW);
              if (Theta_Moving <= 30)
              {
                Serial.println("Theta_Moving <= 30,CW");
                Theta_Moving = abs(Theta_Start_Move - GET_THETA_3());
              }

              else
              {
                Serial.println("Theta_Moving > 30,CCW");
                SERVO_MoveStop(ID_3);
                do
                {
                  SERVO_MoveVelocity(ID_3, homeSpeed * 2, CW);
                } while (digitalRead(IPS1) == false);
              }
          } while (digitalRead(IPS1) == false);
          SERVO_MoveStop(ID_3);
          MOVE_ORIGIN_SERVO_3();
          Theta_Moving = 0;
      }
      else
      {
        Serial.println("Act_Pos > 0");
          do
          {
              SERVO_MoveVelocity(ID_3, homeSpeed * 2, CW);
              if (Theta_Moving <= 30)
              {
                Serial.println("Theta_Moving <= 30,CCW");
                Theta_Moving = abs(Theta_Start_Move - GET_THETA_3());
              }

              else
              {
                Serial.println("Theta_Moving > 30,CW");
                SERVO_MoveStop(ID_3);

                do
                {
                  SERVO_MoveVelocity(ID_3, homeSpeed * 2, CCW);
                } while (digitalRead(IPS1) == false);
              }
          } while (digitalRead(IPS1) == false);
          SERVO_MoveStop(ID_3);
          MOVE_ORIGIN_SERVO_3();
          Theta_Moving = 0;
      }
  }
}

void MOVE_ORIGIN_SERVO_3_debug()
{
  do
  {
    SERVO_MoveVelocity(ID_3, homeSpeed * 2, CW);
  } while (digitalRead(IPS1) == true);
  SERVO_MoveStop(ID_3);
  long CW_Pos = 0;
  SERVO_GetActualPos(ID_3, &CW_Pos);
  Serial.printf("CW_Pos: %ld\n",CW_Pos);
  delayMicroseconds(Process_delay);

  do
  {
    SERVO_MoveVelocity(ID_3, homeSpeed * 2, CCW);
  } while (digitalRead(IPS1) == false);

  do
  {
    SERVO_MoveVelocity(ID_3, homeSpeed * 2, CCW);
  } while (digitalRead(IPS1) == true);
  SERVO_MoveStop(ID_3);
  long CCW_Pos = 0;
  SERVO_GetActualPos(ID_3, &CCW_Pos);
  Serial.printf("CCW_Pos: %ld\n",CCW_Pos);
  Serial.printf("Diff Pos: %ld\n", abs(CW_Pos - CCW_Pos));
  delayMicroseconds(Process_delay);
  
  SERVO_MoveSingleAxisIncPos(ID_3, (long)(round((abs(CW_Pos - CCW_Pos))/2)), (unsigned long)homeSpeed * 2);
  Serial.println("Move to origin");
  SERVO_ClearPosition(ID_3);
}

//-----------------------End of debug function------------------------

//---------------------Start of AT24C256 function-----------------------
/*
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

void eeprom_get(){
  char message_func[7];
  char message_func_small[2];
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

  eeprom.read(20, &quadrant, sizeof(quadrant));
  delay(5);
  Serial.printf("quadrant = %d\n \n",quadrant);
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
*/
//-----------------------End of AT24C256 function-----------------------


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
  Serial.printf("J3 on, Home = %d \n",IPS);
  Serial.printf("Speed = %d; Accel =%d; stat_flag = 0x%02X\n\n", movingSpeed, ACCEL_TIME, stat_flag);
}


void loop() {
  Motion_Control();
  // if (move_Issued == true){
  //   movingFlag = GET_MOVING_FLAG_SERVO_3();
  //   // delayMicroseconds(Process_delay);
  //   if (movingFlag == 0){
  //     Measure_angle_int();
  //     delayMicroseconds(500);
  //     stat_flag = Complete;
  //     move_Issued = false;
  //   } 
  //   else {
  //     if (Measure_flag == 1) {
  //       Measure_angle_int();
  //       delayMicroseconds(500);
  //     }
  //   }
  // }
  if (move_Issued == true){ 
    if (Measure_flag == 1) {
      Measure_angle_int();
      delayMicroseconds(100);
    }
    
    movingFlag = GET_MOVING_FLAG_SERVO_3();
    delayMicroseconds(100);
    if (movingFlag == 0){
      Measure_angle_int();  
      delayMicroseconds(100);
      stat_flag = Complete;
      move_Issued = false;
    } 
  }

}





