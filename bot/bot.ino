#include <mechbotShield.h>
#include <avr/io.h>
#include <util/delay.h>

#define LINE_Kp 150
#define LINE_Ki 0.01
#define LINE_Kd 20

#define LINE_THRESHOLD_LEFT 790
#define LINE_THRESHOLD_LEFT_C 770
#define LINE_THRESHOLD_RIGHT_C 730
#define LINE_THRESHOLD_RIGHT 770
#define LINE_MOTOR_SPEED 600

int lineSensors[4];
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

  while (1) {
    followLine();
  }

}

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
    //Intersection
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

