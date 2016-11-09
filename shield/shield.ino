#include <Wire.h>
#include <EVShield.h>
#include <EVs_NXTCam.h>

#define MODE_TRANSMIT 0
#define MODE_RECIEVE 1

#define DIRECTION_LEFT 0
#define DIRECTION_RIGHT 1
#define DIRECTION_UNKNOWN 2

#define ARM_POS_LEFT 2001
#define ARM_POS_RIGHT 2002

#define MOTOR_1_SPEED 20
#define MOTOR_1_TURN_DELAY 900
#define MOTOR_2_SPEED 40
#define MOTOR_2_GRAB_DELAY 1000

EVShield evShield(0x34, 0x36);

void setup() {
  evShield.init(SH_HardwareI2C);
  evShield.bank_a.motorReset();
  evShield.bank_b.motorReset();
  Serial.begin(9600);
}

void loop() {

}

/**
   COMMS
*/
void setComMode(int mode) {
  switch (mode) {
    case MODE_TRANSMIT:
      pinMode(8, OUTPUT);
      pinMode(9, OUTPUT);
      break;
    case MODE_RECIEVE:
      pinMode(8, INPUT);
      pinMode(9, INPUT);
      break;
  }
}

int getDirection() {
  if (digitalRead(8) == 0 && digitalRead(9) == 1) {
    return DIRECTION_LEFT;
  } else if (digitalRead(8) == 1 && digitalRead(9) == 0) {
    return DIRECTION_RIGHT;
  }
  return DIRECTION_UNKNOWN;
}

void sendConfirmation() {
  digitalWrite(8, HIGH);
  digitalWrite(9, HIGH);
  delay(10);
  digitalWrite(8, LOW);
  digitalWrite(9, LOW);
}

/**
   MOTOR 1 CONTROLS
*/
void armLeft() {
  evShield.bank_a.motorRunUnlimited(SH_Motor_1,
                                    SH_Direction_Forward,
                                    MOTOR_1_SPEED);
  delay(MOTOR_1_TURN_DELAY);
  evShield.bank_a.motorStop(SH_Motor_1, SH_Next_Action_BrakeHold);
}

void armRight() {
  evShield.bank_a.motorRunUnlimited(SH_Motor_1,
                                    SH_Direction_Reverse,
                                    MOTOR_1_SPEED);
  delay(MOTOR_1_TURN_DELAY);
  evShield.bank_a.motorStop(SH_Motor_1, SH_Next_Action_BrakeHold);
}

/**
   MOTOR 2 Controls
*/
void grabCan() {
  evShield.bank_b.motorRunUnlimited(SH_Motor_1,
                                    SH_Direction_Forward,
                                    MOTOR_2_SPEED);
  delay(MOTOR_2_GRAB_DELAY);
  evShield.bank_b.motorStop(SH_Motor_1, SH_Next_Action_BrakeHold);
}

void releaseCan() {
  evShield.bank_b.motorRunUnlimited(SH_Motor_1,
                                    SH_Direction_Reverse,
                                    MOTOR_2_SPEED);
  delay(MOTOR_2_GRAB_DELAY);
  evShield.bank_b.motorStop(SH_Motor_1, SH_Next_Action_BrakeHold);
}


