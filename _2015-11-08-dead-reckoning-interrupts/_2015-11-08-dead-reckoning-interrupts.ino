#include <OrangutanLEDs.h>
#include <OrangutanMotors.h>
#include <OrangutanPushbuttons.h>
#include <OrangutanLCD.h>
#include <math.h>
#include <Pololu3pi.h>
#include <PololuQTRSensors.h>

OrangutanMotors motors;
OrangutanLEDs leds;
OrangutanPushbuttons buttons;
OrangutanLCD lcd;
Pololu3pi robot;
unsigned int sensors[5];

const int counterPreset = 65536-20000;  // 1000hz

void setup() {
  /* initialize timer1 for our motor timing code*/
  noInterrupts(); // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;

  TCNT1 = counterPreset;  // always preload counter
  TCCR1B |= (1 << CS10);  // no prescalar
  TIMSK1 |= (1 << TOIE1); // enable timer overflow interrupt
  interrupts(); // enable all interrupts

  robot.init(2000);

  buttons.waitForPress(BUTTON_A | BUTTON_B | BUTTON_C);
  delay(1000);
  calibrateSensors();
}

void calibrateSensors() {
  
  // Auto-calibration: turn right and left while calibrating the
  // sensors.
  for (int counter=0; counter<80; counter++)
  {
    if (counter < 20 || counter >= 60)
      deadReckoningSetMotors(40, -40);
    else
      deadReckoningSetMotors(-40, 40);

    // This function records a set of sensor readings and keeps
    // track of the minimum and maximum values encountered.  The
    // IR_EMITTERS_ON argument means that the IR LEDs will be
    // turned on during the reading, which is usually what you
    // want.
    robot.calibrateLineSensors(IR_EMITTERS_ON);

    // Since our counter runs to 80, the total delay will be
    // 80*20 = 1600 ms.
    delay(10);
  }
  deadReckoningSetMotors(0, 0);
}

bool ledOn = false;
int count = 0;
int deadReckoningLastMotorSpeedA=0;
int deadReckoningLastMotorSpeedB=0;
int deadReckoningNextMotorSpeedA=0;
int deadReckoningNextMotorSpeedB=0;
/* TODO: long might overflow after 842s of full speed */
long deadReckoningDistanceA=0;
long deadReckoningDistanceB=0;

ISR(TIMER1_OVF_vect)          // timer compare interrupt service routine
{
  /* Must pre-load time to keep interrupt period constant*/
  TCNT1 = counterPreset;

  /* Add a millisecond worth of distance */
  deadReckoningDistanceA += deadReckoningLastMotorSpeedA;
  deadReckoningDistanceB += deadReckoningLastMotorSpeedB;

  /* Set speed for next millisecond */
  motors.setSpeeds(
    round(deadReckoningNextMotorSpeedA*calibratedMotorPwm(0.5, 0.5, deadReckoningNextMotorSpeedA)),
    round(deadReckoningNextMotorSpeedB*calibratedMotorPwm(1.0, 0.97, deadReckoningNextMotorSpeedB)));
  deadReckoningLastMotorSpeedA = deadReckoningNextMotorSpeedA;
  deadReckoningLastMotorSpeedB = deadReckoningNextMotorSpeedB;
  
  /* Blink LED, just for fun */
  if (++count==1000) {
    count=0;
   if (ledOn) {
    leds.red(LOW);
    ledOn = false;
   } else {
    leds.red(HIGH);
    ledOn = true;
   }
  }
}

void deadReckoningSetMotors(int speedA, int speedB) {
  deadReckoningNextMotorSpeedA = speedA;
  deadReckoningNextMotorSpeedB = speedB;
}

/* TODO locking of shared variables? */
long deadReckoningGetMotorADistance() {
  return deadReckoningDistanceA;
}
long deadReckoningGetMotorBDistance() {
  return deadReckoningDistanceB;
}
void deadReckoningResetMotorDistances() {
  deadReckoningDistanceA=0;
  deadReckoningDistanceB=0;
}

double calibratedMotorPwm(double A, double B, double speed) {
  return (B-A)/255*speed + A;
}

const int turnSpeed = 61;
const int turnTimeMilis = 251;

void turnRight() {
  deadReckoningSetMotors(turnSpeed,-1 * turnSpeed);
  delay(turnTimeMilis);
  deadReckoningSetMotors(0,0);
}

void turnLeft() {
  deadReckoningSetMotors(-1 * turnSpeed,turnSpeed);
  delay(turnTimeMilis);
  deadReckoningSetMotors(0,0);
}

void goForward() {
  deadReckoningSetMotors(turnSpeed,turnSpeed);
  delay(turnTimeMilis * 2);
  deadReckoningSetMotors(0,0);
}

void printDistance() {
 /* Display current settings */
 lcd.clear();
 /* First line */
 lcd.gotoXY(0,0);
 //lcd.print("A:");
 lcd.print(deadReckoningGetMotorADistance());
 /* Second line */
 lcd.gotoXY(0,1);
 //lcd.print("B:");
 lcd.print(deadReckoningGetMotorBDistance());  
}

void printSensors(char* message) {
  lcd.clear();
  lcd.gotoXY(0,0); lcd.print(message); lcd.print(sensors[1]); 
  lcd.gotoXY(0,1); lcd.print(sensors[2]); lcd.print(sensors[3]);
}

double g=64.0;
double vi=64.0;
int verticalSpeedAfterBeingThrownUp(double milliseconds) {
  /* displacement = ax+bx^2 */
  /* speed = a + 2bx */
  return round(vi-milliseconds/1000.0*g);
}

int direction = 0;
double maxWheelDifference = 50.0;
void followLine(double speed) {
  double linePosition = robot.readLine(sensors, IR_EMITTERS_ON);
  double turn = maxWheelDifference * (linePosition - 2000.0)/2000.0;
  deadReckoningSetMotors(round(speed+turn),round(speed-turn));
}

void calibrateMotorPwm(int speed) {
  /* Drive in a straight line for the first segment */
  while( ! (sensors[1]>500 && sensors[2]>500 && sensors[3]>500)) {
    followLine(speed); // todo various speeds
    //printSensors("A ");
    delay(10);
  }
  delay(10);
  deadReckoningSetMotors(0,0);
  delay(1000);

  /* Drive through cross */
  //printSensors("B ");
  deadReckoningSetMotors(speed,speed);
  while(sensors[1]>500 && sensors[2]>500 && sensors[3]>500) {
    robot.readLine(sensors, IR_EMITTERS_ON);
    //printSensors("A ");
    delay(10);
  }
  delay(10);
  deadReckoningSetMotors(0,0);
  delay(1000);

  /* For second segment, drive blindly */
  unsigned int positionA = robot.readLine(sensors, IR_EMITTERS_ON);
  deadReckoningSetMotors(speed,speed);
  while( ! (sensors[1]>500 && sensors[2]>500)
  || !(sensors[2]>500 && sensors[3]>500)) {
    robot.readLine(sensors, IR_EMITTERS_ON);
    //printSensors("C ");
    delay(10);
  }
  deadReckoningSetMotors(0,0);

  /* See where the line has moved */
  unsigned int positionB = robot.readLine(sensors, IR_EMITTERS_ON);
  lcd.clear();
  lcd.gotoXY(0,0); lcd.print(positionA);
  lcd.gotoXY(0,1); lcd.print(positionB);
}

int totalMilliseconds = 0;
int delayMilliseconds = 10;
void loop() {
  /* Square dance 
  goForward();
  printDistance();
  delay(1000);
  turnRight();
  printDistance();
  delay(1000);
  */

  /* Bouncing ball */
  int speed = verticalSpeedAfterBeingThrownUp(totalMilliseconds);
  if (speed < -1*vi) {
    deadReckoningSetMotors(0,0);
    delay(1);
    totalMilliseconds=0;
    vi = 0.9*vi;
    return;
  }
  deadReckoningSetMotors(speed,speed);
  totalMilliseconds += delayMilliseconds;
  delay(delayMilliseconds);
  

  /* Calibration 
  calibrateMotorPwm(50);
  while(true);
  */
}
