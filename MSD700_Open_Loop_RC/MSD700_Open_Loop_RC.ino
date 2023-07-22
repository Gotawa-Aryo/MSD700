#include <RC_Receiver.h>

// RC is receiver with 4 PWM channel
#define RC_CH_COUNT 4
#define RC_CH1      48
#define RC_CH2      49
#define RC_CH3      47
#define RC_CH4      43 // used as failsafe

// MT is motor, used to customize motor parameter
#define MT_MAX_PWM  100

// ML is motor on the left
#define ML_EN   7
#define ML_RPWM 9
#define ML_LPWM 10
#define MTL_ACT 13

// MR is motor on the right
#define MR_EN   8
#define MR_RPWM 6
#define MR_LPWM 5
#define MTR_ACT 4

// Enable or disable DEBUG with serial monitor
#define DEBUG 1

// ARMED and DISARMED
#define ARMED    0x00
#define DISARMED 0x01

RC_Receiver receiver(RC_CH1, RC_CH2, RC_CH3, RC_CH4);
uint16_t pwm_in[RC_CH_COUNT + 1]; // this array starts from 1
long int pwm_offset[] = {0, 9, 18, 9, 9}; // this array starts from 1
int16_t  pwm_r = 0, pwm_l = 0, failsafe = DISARMED;

void setup() {
  Serial.begin(115200);
  
  pinMode(MTL_ACT, OUTPUT);
  pinMode(ML_EN, OUTPUT);
  pinMode(ML_RPWM, OUTPUT);
  pinMode(ML_LPWM, OUTPUT);
  
  pinMode(MTR_ACT, OUTPUT);
  pinMode(MR_EN, OUTPUT);
  pinMode(MR_RPWM, OUTPUT);
  pinMode(MR_LPWM, OUTPUT);
  
  digitalWrite(MTL_ACT, HIGH);
  digitalWrite(MTR_ACT, HIGH);

  // Offset calibration - already calibrated with initial value, uncomment to recalibrate
  // for(byte j = 0; j < 100; j++){
  //   for(byte i = 1; i <= 2; i++){
  //     pwm_offset[i] += receiver.getRaw(i);
  //   }
  //   delay(20);
  // }
  // for(byte i = 1; i <= 2; i++){
  //   pwm_offset[i] = 1500 - (pwm_offset[i]/100);
  // }
}

void loop() {
  // Get command from RC
  update_rc();

  // Update failsafe from RC input
  update_failsafe();

  // Update command from RC input
  update_cmd();

  // Write to motor
  write_motor();

  // Print to serial monitor if debug mode
  if(DEBUG){
    debug_all();
  }
    
}

void update_rc(){
  for(byte i = 1; i <= 4; i++){
    pwm_in[i] = receiver.getRaw(i) + pwm_offset[i];
  }
}

void update_failsafe(){
  // It is chosen because if RC is disconnected, the value is 0
  if(pwm_in[4] > 1500){
    failsafe = ARMED;
  }else{
    failsafe = DISARMED;
  }
}

void update_cmd(){
  if(failsafe == DISARMED){ // Disarmed condition
    pwm_r = 0;
    pwm_l = 0;
  }else{ // Armed condition
    int16_t cmd_front_back = map(pwm_in[1], 1000, 2000, -MT_MAX_PWM, MT_MAX_PWM);
    int16_t cmd_right_left = map(pwm_in[2], 1000, 2000, -MT_MAX_PWM, MT_MAX_PWM);
    pwm_r = cmd_right_left - cmd_front_back;
    pwm_l = cmd_right_left + cmd_front_back;
  }
}

void write_motor(){
  // Rotate right motor
  if(pwm_r == 0){
    digitalWrite(MR_EN, LOW);
  }else if(pwm_r > 0){
    digitalWrite(MR_EN, HIGH);
    analogWrite(MR_RPWM, 0);
    analogWrite(MR_LPWM, pwm_r);
  }else{
    digitalWrite(MR_EN, HIGH);
    analogWrite(MR_LPWM, 0);
    analogWrite(MR_RPWM, -pwm_r);
  }
  
  // Rotate left motor
  if(pwm_l == 0){
    digitalWrite(ML_EN, LOW);
  }else if(pwm_l > 0){
    digitalWrite(ML_EN, HIGH);
    analogWrite(ML_RPWM, 0);
    analogWrite(ML_LPWM, pwm_l);
  }else{
    digitalWrite(ML_EN, HIGH);
    analogWrite(ML_LPWM, 0);
    analogWrite(ML_RPWM, -pwm_l);
  }
}

void debug_all(){
  for(byte i = 1; i <= 4; i++){
    Serial.print("CH ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(pwm_in[i]);
    Serial.print("\t");
  }
  if(failsafe == ARMED){
    Serial.print("Status: ARMED\t");
  }else{
    Serial.print("Status: DISARMED\t");
  }
  Serial.print("RMOTOR: ");
  Serial.print(pwm_r);
  Serial.print("\tLMOTOR: ");
  Serial.print(pwm_l);
  Serial.print("\tOFFSET 1: ");
  Serial.print(pwm_offset[1]);
  Serial.print("\tOFFSET 2: ");
  Serial.print(pwm_offset[2]);
  Serial.print("\tOFFSET 3: ");
  Serial.print(pwm_offset[3]);
  Serial.print("\tOFFSET 4: ");
  Serial.print(pwm_offset[4]);
  Serial.println();
}