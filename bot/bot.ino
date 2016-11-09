#include <mechbotShield.h>
#include <avr/io.h>
#include <util/delay.h>

#define MODE_TRANSMIT 0
#define MODE_RECIEVE 1

#define DIRECTION_LEFT 0
#define DIRECTION_RIGHT 1
#define DIRECTION_UNKNOWN 2

#define LINE_Kp 150
#define LINE_Ki 0.01
#define LINE_Kd 20

#define LINE_THRESHOLD_LEFT 770
#define LINE_THRESHOLD_LEFT_C 730
#define LINE_THRESHOLD_RIGHT_C 730
#define LINE_THRESHOLD_RIGHT 770
#define LINE_MOTOR_SPEED 600

#define CAN_THRESHOLD_LEFT 140
#define CAN_THRESHOLD_RIGHT 140

int lineSensors[4];
int distanceSensors[2];
int leftMotorSpeed = 0;
int rightMotorSpeed = 0;

int lineError = 0;
double lineP = 0;
double lineI = 0;
double lineD = 0;
double linePreviousError = 0;
double linePIDVal = 0;

int main(void) {

  initADC();
  initMotor();
  initSoftSerial();
  setLCDBackLight(255);
  clrLCD();

  while (1) {
    followLine();
  }

}

/**
   LINE FOLLOW
*/
void followLine() {
  readLineSensors();
  calculateLinePIDVal();
  PIDMotorControl(LINE_MOTOR_SPEED, linePIDVal);
}

void readLineSensors() {
  for (int i = 0; i < 4; i++) {
    lineSensors[i] = analog(i);
  }

  if ((lineSensors[0] >= LINE_THRESHOLD_LEFT) && (lineSensors[1] < LINE_THRESHOLD_LEFT_C) && (lineSensors[2] < LINE_THRESHOLD_RIGHT_C) && (lineSensors[3] < LINE_THRESHOLD_RIGHT))
    lineError = -3;
  else if ((lineSensors[0] >= LINE_THRESHOLD_LEFT) && (lineSensors[1] >= LINE_THRESHOLD_LEFT_C) && (lineSensors[2] < LINE_THRESHOLD_RIGHT_C) && (lineSensors[3] < LINE_THRESHOLD_RIGHT))
    lineError = -2;
  else if ((lineSensors[0] < LINE_THRESHOLD_LEFT) && (lineSensors[1] >= LINE_THRESHOLD_LEFT_C) && (lineSensors[2] < LINE_THRESHOLD_RIGHT_C) && (lineSensors[3] < LINE_THRESHOLD_RIGHT))
    lineError = -1;
  else if ((lineSensors[0] < LINE_THRESHOLD_LEFT) && (lineSensors[1] >= LINE_THRESHOLD_LEFT_C) && (lineSensors[2] >= LINE_THRESHOLD_RIGHT_C) && (lineSensors[3] < LINE_THRESHOLD_RIGHT))
    lineError = 0;
  else if ((lineSensors[0] < LINE_THRESHOLD_LEFT) && (lineSensors[1] < LINE_THRESHOLD_LEFT_C) && (lineSensors[2] >= LINE_THRESHOLD_RIGHT_C) && (lineSensors[3] < LINE_THRESHOLD_RIGHT))
    lineError = 1;
  else if ((lineSensors[0] < LINE_THRESHOLD_LEFT) && (lineSensors[1] < LINE_THRESHOLD_LEFT_C) && (lineSensors[2] >= LINE_THRESHOLD_RIGHT_C) && (lineSensors[3] >= LINE_THRESHOLD_RIGHT))
    lineError = 2;
  else if ((lineSensors[0] < LINE_THRESHOLD_LEFT) && (lineSensors[1] < LINE_THRESHOLD_LEFT_C) && (lineSensors[2] < LINE_THRESHOLD_RIGHT_C) && (lineSensors[3] >= LINE_THRESHOLD_RIGHT))
    lineError = 3;
  else if ((lineSensors[0] < LINE_THRESHOLD_LEFT) && (lineSensors[1] < LINE_THRESHOLD_LEFT_C) && (lineSensors[2] < LINE_THRESHOLD_RIGHT_C) && (lineSensors[3] < LINE_THRESHOLD_RIGHT)) {
    if (lineError < 0) {
      lineError = -5;
    } else {
      lineError = 5;
    }
  } else if ((lineSensors[0] >= LINE_THRESHOLD_LEFT) && (lineSensors[1] >= LINE_THRESHOLD_LEFT_C) && (lineSensors[2] >= LINE_THRESHOLD_RIGHT_C) && (lineSensors[3] >= LINE_THRESHOLD_RIGHT)) {
    intersection();
  }

}

void calculateLinePIDVal() {
  lineP = lineError;
  lineI = lineI + lineError;
  lineD = lineError - linePreviousError;

  linePIDVal = (LINE_Kp * lineP) + (LINE_Ki * lineI) + (LINE_Kd * lineD);

  linePreviousError = lineError;
}

void PIDMotorControl(int motorSpeed, double pidVal) {
  leftMotorSpeed = motorSpeed + pidVal;
  rightMotorSpeed = motorSpeed - pidVal;

  constrain(leftMotorSpeed, 0, 1000);
  constrain(rightMotorSpeed, 0 , 1000);

  motor(leftMotorSpeed, rightMotorSpeed);
}

/**
   CAN SENSING
*/
void printDistanceVals() {
  distanceSensors[0] = analog(4);
  distanceSensors[1] = analog(6);
  clrLCD();
  moveLCDCursor(0);
  lcdPrintDec(distanceSensors[0]);
  moveLCDCursor(8);
  lcdPrintDec(distanceSensors[1]);
}

int getCanDirection() {
  distanceSensors[0] = analog(4);
  distanceSensors[1] = analog(6);
  if (distanceSensors[0] > CAN_THRESHOLD_LEFT) {
    return DIRECTION_LEFT;
  } else if (distanceSensors[1] > CAN_THRESHOLD_RIGHT) {
    return (DIRECTION_RIGHT);
  } else {
    return DIRECTION_UNKNOWN;
  }
}

void intersection() {
  motor(0, 0);
  _delay_ms(500);
  motor(100, 100);
  _delay_ms(200);
  motor(0, 0);
  int canDirection = getCanDirection();
  clrLCD();
  lcdPrintDec(canDirection);
  _delay_ms(1000);
  motor(400, 400);
  _delay_ms(1000);
}

/**
   COMMS
*/
void setComMode(int mode) {
  switch (mode) {
    case MODE_TRANSMIT:
      DDRC |= (1 << PC4) | (1 << PC5);
      break;
    case MODE_RECIEVE:
      DDRC &= ~((1 << PC4) | (1 << PC5));
      break;
  }
}

void idle() {
  PORTC &= ~((1 << PC4) | (1 << PC5));
}

void sendDirection(int dir) {
  if (dir == DIRECTION_LEFT) {
    PORTC &= ~(1 << PC4);
    PORTC |= (1 << PC5);
  } else {
    PORTC &= ~(1 << PC5);
    PORTC |= (1 << PC4);
  }
}

bool getConfirmation() {
  if ((PINC & (1 << PC4)) && (PINC & (1 << PC5))) {
    return true;
  }
  return false;
}

