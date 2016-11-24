#include <mechbotShield.h>
#include <avr/io.h>
#include <util/delay.h>

#define MODE_TRANSMIT 0
#define MODE_RECIEVE 1

#define COMMAND_IDLE 0
#define COMMAND_LEFT 1
#define COMMAND_RIGHT 2
#define COMMAND_DROP 3

#define LINE_Kp 150
#define LINE_Ki 0.01
#define LINE_Kd 20

#define LINE_THRESHOLD_LEFT 770
#define LINE_THRESHOLD_LEFT_C 730
#define LINE_THRESHOLD_RIGHT_C 730
#define LINE_THRESHOLD_RIGHT 770
#define LINE_MOTOR_SPEED 500

#define CAN_THRESHOLD_LEFT 140
#define CAN_THRESHOLD_RIGHT 140

#define GARBAGE_THRESHOLD 450

int lineSensors[4];
int distanceSensors[3];
int leftMotorSpeed = 0;
int rightMotorSpeed = 0;
int frontSensor = 0;

int intersections = 0;

int lineError = 0;
double lineP = 0;
double lineI = 0;
double lineD = 0;
double linePreviousError = 0;
double linePIDVal = 0;

boolean ledsOn = false;

int main(void) {

  initADC();
  initMotor();
  initLEDs();
  DDRD &= ~((1 << PD3) | (1 << PD4) | (1 << PD5));
  PORTD |= 0b00111000;
  PORTB |= 0b00000000;
  initSoftSerial();
  setLCDBackLight(255);
  clrLCD();
  idle();
  while (1) {
    frontSensor = analog(5);
    if (frontSensor >= GARBAGE_THRESHOLD) {
      clrLCD();
      motor(0, 0);
      lcdPrint("GARBAGE");
      beep(500, 1000000);
      flashLEDS();
      _delay_ms(500);
    } else {
      followLine();
    }
  }
}

/**
   OPERATIONS
*/
void intersection() {
  switch (intersections) {
    case 0:
      grabIntersection();
      break;
    case 1:
      dropIntersection();
      break;
    case 2:
      grabIntersection();
      break;
    case 3:
      dropIntersection();
      break;
  }
}

void grabIntersection() {
  motor(0, 0);
  _delay_ms(300);
  int canDirection = getCanDirection();
  clrLCD();
  if (canDirection == COMMAND_LEFT) {
    lcdPrint("LEFT");
    sendCommand(COMMAND_LEFT);
  } else if (canDirection == COMMAND_RIGHT) {
    lcdPrint("RIGHT");
    sendCommand(COMMAND_RIGHT);
  }
  _delay_ms(10000);
  motor(500, 500);
  _delay_ms(800);
  intersections++;
}

void dropIntersection() {
  motor(0, 0);
  _delay_ms(300);
  sendCommand(COMMAND_DROP);
  _delay_ms(3000);
  motor(500, 500);
  _delay_ms(800);
  intersections++;
}

void initLEDs() {
  DDRB |= (1 << PB0) | (1 << PB1) | (1 << PB2) | (1 << PB3) | (1 << PB4) | (1 << PB5) ;
  DDRD |= (1 << PD6) | (1 << PD7) ;
}

void flashLEDS() {
  if (ledsOn) {
    PORTB &= ~((1 << PB0) | (1 << PB1) | (1 << PB2) | (1 << PB3) | (1 << PB4) | (1 << PB5));
    PORTD &= ~((1 << PD6) | (1 << PD7));
    ledsOn = false;
  } else {
    ledsOn = true;
    PORTB |= (1 << PB0) | (1 << PB1) | (1 << PB2) | (1 << PB3) | (1 << PB4) | (1 << PB5);
    PORTD |= (1 << PD6) | (1 << PD7) ;
  }
  _delay_ms(500);
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
   CAN/GARBAGE SENSING
*/
void printDistanceVals() {
  distanceSensors[0] = analog(4);
  distanceSensors[1] = analog(5);
  distanceSensors[2] = analog(6);
  clrLCD();
  moveLCDCursor(0);
  lcdPrintDec(distanceSensors[0]);
  moveLCDCursor(8);
  lcdPrintDec(distanceSensors[1]);
  moveLCDCursor(16);
  lcdPrintDec(distanceSensors[2]);
}

int getCanDirection() {
  distanceSensors[0] = analog(4);
  distanceSensors[1] = analog(6);
  if (distanceSensors[0] > CAN_THRESHOLD_LEFT) {
    return COMMAND_LEFT;
  } else if (distanceSensors[1] > CAN_THRESHOLD_RIGHT) {
    return (COMMAND_RIGHT);
  } else {
    return COMMAND_IDLE;
  }
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

void sendCommand(int command) {
  setComMode(MODE_TRANSMIT);
  switch (command) {
    case COMMAND_IDLE:
      PORTC &= ~((1 << PC4) | (1 << PC5));
      break;
    case COMMAND_LEFT:
      PORTC &= ~(1 << PC4);
      PORTC |= (1 << PC5);
      break;
    case COMMAND_RIGHT:
      PORTC &= ~(1 << PC5);
      PORTC |= (1 << PC4);
      break;
    case COMMAND_DROP:
      PORTC |= ((1 << PC4) | (1 << PC5));
      break;
  }
  _delay_ms(50);
  idle();
}

