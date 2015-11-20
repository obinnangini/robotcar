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
    round(deadReckoningNextMotorSpeedA*calibratedMotorPwm(0.97, 1.0, deadReckoningNextMotorSpeedA)),
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

int speedFunction(){
  return 100;
}

int totalMilliseconds = 0;
int delayMilliseconds = 10;
void loop() {
  /* Determine at what speed we should be travelling */
  int speed = speedFunction();

  // Get the position of the line.  Note that we *must* provide
  // the "sensors" argument to read_line() here, even though we
  // are not interested in the individual sensor readings.
  unsigned int position = robot.readLine(sensors, IR_EMITTERS_ON);
  if (position < 1000)
  {
    // We are far to the right of the line: turn left.

    // Set the right motor to 100 and the left motor to zero,
    // to do a sharp turn to the left.  Note that the maximum
    // value of either motor speed is 255, so we are driving
    // it at just about 40% of the max.
    deadReckoningSetMotors(0, speed);

    // Just for fun, indicate the direction we are turning on
    // the LEDs.
    OrangutanLEDs::left(HIGH);
    OrangutanLEDs::right(LOW);
  }
  else if (position < 3000)
  {
    // We are somewhat close to being centered on the line:
    // drive straight.
    int speed = speedFunction();
    deadReckoningSetMotors(speed,speed);
    OrangutanLEDs::left(HIGH);
    OrangutanLEDs::right(HIGH);
  }
  else
  {
    // We are far to the left of the line: turn right.
    deadReckoningSetMotors(speed, 0);
    OrangutanLEDs::left(LOW);
    OrangutanLEDs::right(HIGH);
  }
}
