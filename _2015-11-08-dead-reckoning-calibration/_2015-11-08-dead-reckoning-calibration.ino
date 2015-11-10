/* Required import for using 3pi motor library */
#include <OrangutanMotors.h>
/* Required import for for using 3pi LCD screen */
#include <OrangutanLCD.h>
/* Required import for for using 3pi user buttons */
#include <OrangutanPushbuttons.h>

/* Object for controlling 3pi motors.  Docs say only use one instance */
OrangutanMotors motors;
/* Object for controlling 3pi LCD screen.  Docs say only use one instance */
OrangutanLCD lcd;
/* Object for controlling 3pi user buttons.  Docs say only use one instance */
OrangutanPushbuttons buttons;

/* Constants */
const int calibrationTestTimeMilis = 250;
const int testStepSize = 16;

/* Gets run once when robot turns on or is reset */
void setup() {
  /* No initialization code required */
}

/* PWM duty cycle for each motor, -255 to 255 with 0 being active breaking */
int motorAConfiguredPwm=0;
int motorBConfiguredPwm=0;

int incrementMotorPwmAndRollover(int pwm, int increment) {
  pwm += increment;
  if (pwm > 256) {
    pwm = -256;
  } else if (pwm < -256) {
    pwm = 256;
  }
  return pwm;
}

void loop() {
 /* MAIN LOOP, once per calibration test */

 /* Display current settings */
 lcd.clear();
 /* First line */
 lcd.gotoXY(0,0);
 lcd.print("A:");
 lcd.print(motorAConfiguredPwm);
 /* Second line */
 lcd.gotoXY(0,1);
  lcd.print("B:");
 lcd.print(motorBConfiguredPwm);

 /* Get user input */
 unsigned char button = buttons.waitForPress(BUTTON_A | BUTTON_B | BUTTON_C);
 /* We must "debounce" button presses or else they will be registered multiple times */
 buttons.waitForRelease(button);
 delay(10);
 /* Process user input */
 if (button == BUTTON_A) {
  /* Loop again for additional user input */
  motorAConfiguredPwm = incrementMotorPwmAndRollover(motorAConfiguredPwm,testStepSize);
  return;
 } else if (button == BUTTON_C) {
  /* Loop again for additional user input */
  motorBConfiguredPwm = incrementMotorPwmAndRollover(motorBConfiguredPwm,testStepSize);
  return;
 } else {
  /* User has initiated test, continue to next section */
 }
 
 /* Wait for a bit so user's hand can move away */
 delay(500);

 /* Set motor's to speed based on user's settings */
 motors.setSpeeds(motorAConfiguredPwm,motorBConfiguredPwm);

 /* Wait for hard-coded amount of time */
 delay(calibrationTestTimeMilis);

 /* Set motor's off */
 motors.setSpeeds(0,0);

 /* END MAIN LOOP, Test is over */
}
