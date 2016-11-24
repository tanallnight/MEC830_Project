#include <Wire.h>
#include <EVShield.h>
#include <EVs_NXTCam.h>

#define MODE_TRANSMIT 0
#define MODE_RECIEVE 1

#define COMMAND_IDLE 0
#define COMMAND_LEFT 1
#define COMMAND_RIGHT 2
#define COMMAND_DROP 3

#define ARM_POS_LEFT 2001
#define ARM_POS_RIGHT 2002

#define MOTOR_1_SPEED 20
#define MOTOR_1_TURN_DELAY 900
#define MOTOR_2_SPEED 10
#define MOTOR_2_GRAB_DELAY 1350

#define COLOR_RED 5
#define COLOR_BLUE 6

EVShield evShield(0x34, 0x36);
EVs_NXTCam nxtCam;

int currentCommand;
int currentColor;
int currentArmPos = ARM_POS_RIGHT;
int nblobs;
uint8_t color[8];
uint8_t left[8];
uint8_t top[8];
uint8_t bottom[8];
uint8_t right[8];

void setup() {
  Serial.begin(9600);
  evShield.init(SH_HardwareI2C);
  evShield.bank_a.motorReset();
  evShield.bank_b.motorReset();
  nxtCam.init(&evShield, SH_BAS1);
  nxtCam.disableTracking();
  nxtCam.selectObjectMode();
  nxtCam.sortSize();
  delay(1000);
  armRight();
  delay(500);
  setComMode(MODE_RECIEVE);
  //blinkLed(5, 0, 255, 0);
}

void loop() {
  //  if (evShield.getButtonState(BTN_LEFT) == true) {
  //    grabCan();
  //    delay(100);
  //  } else if (evShield.getButtonState(BTN_RIGHT) == true) {
  //    releaseCan();
  //    delay(100);
  //  }
  currentCommand = getCommand();
  if (currentCommand != COMMAND_IDLE) {
    switch (currentCommand) {
      case COMMAND_LEFT:
        leftSubroutine();
        break;
      case COMMAND_RIGHT:
        rightSubroutine();
        break;
      case COMMAND_DROP:
        releaseCan();
        delay(500);
        break;
    }
  }
}

void leftSubroutine() {
  evShield.bank_b.ledSetRGB(255, 255, 255);
  armLeft();
  delay(500);
  grabCan();
  delay(500);
  currentColor = getColor();
  evShield.bank_b.ledSetRGB(0, 0, 0);
  delay(50);
  if (currentColor == COLOR_RED) {
    evShield.bank_a.ledSetRGB(255, 0, 0);
    if (currentArmPos == ARM_POS_RIGHT) {
      armLeft();
    }
  } else {
    evShield.bank_a.ledSetRGB(0, 0, 255);
    if (currentArmPos == ARM_POS_LEFT) {
      armRight();
    }
  }
  delay(2000);
}

void rightSubroutine() {
  evShield.bank_a.ledSetRGB(255, 255, 255);
  armRight();
  delay(500);
  grabCan();
  delay(500);
  currentColor = getColor();
  evShield.bank_a.ledSetRGB(0, 0, 0);
  delay(50);
  if (currentColor == COLOR_RED) {
    evShield.bank_a.ledSetRGB(255, 0, 0);
    if (currentArmPos == ARM_POS_RIGHT) {
      armLeft();
    }
  } else {
    evShield.bank_a.ledSetRGB(0, 0, 255);
    if (currentArmPos == ARM_POS_LEFT) {
      armRight();
    }
  }
  delay(2000);
}

void blinkLed(int t, int R, int G, int B) {
  bool ledState = false;
  for (int i = 0; i < t * 2; i++) {
    if (ledState) {
      evShield.bank_a.ledSetRGB(0, 0, 0);
      evShield.bank_b.ledSetRGB(0, 0, 0);
      ledState = false;
    } else {
      evShield.bank_a.ledSetRGB(R, G, B);
      evShield.bank_b.ledSetRGB(R, G, B);
      ledState = true;
    }
    delay(500);
  }
  evShield.bank_a.ledSetRGB(0, 0, 0);
  evShield.bank_b.ledSetRGB(0, 0, 0);
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

int getCommand() {
  if (digitalRead(8) == 0 && digitalRead(9) == 0) {
    return COMMAND_IDLE;
  } else if (digitalRead(8) == 0 && digitalRead(9) == 1) {
    return COMMAND_LEFT;
  } else if (digitalRead(8) == 1 && digitalRead(9) == 0) {
    return COMMAND_RIGHT;
  } else if (digitalRead(8) == 1 && digitalRead(9) == 1) {
    return COMMAND_DROP;
  }
  return COMMAND_IDLE;
}

void sendConfirmation() {
  setComMode(MODE_TRANSMIT);
  digitalWrite(8, HIGH);
  digitalWrite(9, HIGH);
  delay(10);
  digitalWrite(8, LOW);
  digitalWrite(9, LOW);
  setComMode(MODE_RECIEVE);
}

/**
   Camera
*/
int getColor() {
  if (currentArmPos != ARM_POS_LEFT) {
    armLeft();
  }
  delay(500);
  nxtCam.enableTracking();
  delay(1000);
  while (1) {
    nxtCam.issueCommand('J');
    delay(500);
    nxtCam.getBlobs(&nblobs, color, left, top, right, bottom);
    delay(500);
    nxtCam.issueCommand('K');

    uint8_t i;
    for (int i = 0; i < nblobs; i++) {
      if (color[i] == 2)  {
        nxtCam.disableTracking();
        return COLOR_RED;
      }
      if (color[i] == 3)  {
        nxtCam.enableTracking();
        return COLOR_BLUE;
      }
    }
  }
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
  delay(500);
  currentArmPos = ARM_POS_LEFT;
}

void armRight() {
  evShield.bank_a.motorRunUnlimited(SH_Motor_1,
                                    SH_Direction_Reverse,
                                    MOTOR_1_SPEED);
  delay(MOTOR_1_TURN_DELAY);
  evShield.bank_a.motorStop(SH_Motor_1, SH_Next_Action_BrakeHold);
  delay(500);
  currentArmPos = ARM_POS_RIGHT;
}

/**
   MOTOR 2 Controls
*/
void grabCan() {
  evShield.bank_b.motorRunUnlimited(SH_Motor_1,
                                    SH_Direction_Forward,
                                    MOTOR_2_SPEED);
  delay(MOTOR_2_GRAB_DELAY);
  evShield.bank_b.motorRunUnlimited(SH_Motor_1,
                                    SH_Direction_Forward,
                                    5);
}

void releaseCan() {
  evShield.bank_b.motorRunUnlimited(SH_Motor_1,
                                    SH_Direction_Reverse,
                                    MOTOR_2_SPEED);
  delay(MOTOR_2_GRAB_DELAY - 100);
  evShield.bank_b.motorStop(SH_Motor_1, SH_Next_Action_BrakeHold);
}


