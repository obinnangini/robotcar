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
long deadReckoningTime=0;

ISR(TIMER1_OVF_vect)          // timer compare interrupt service routine
{
  /* Must pre-load time to keep interrupt period constant*/
  TCNT1 = counterPreset;

  /* Increment time tracking */
  deadReckoningTime++;
  
  /* Add a millisecond worth of distance */
  deadReckoningDistanceA += deadReckoningLastMotorSpeedA;
  deadReckoningDistanceB += deadReckoningLastMotorSpeedB;

  /* Set speed for next millisecond */
  motors.setSpeeds(deadReckoningNextMotorSpeedA,deadReckoningNextMotorSpeedB);
  deadReckoningLastMotorSpeedA = deadReckoningNextMotorSpeedA;
  deadReckoningLastMotorSpeedB = deadReckoningNextMotorSpeedB;
}

void deadReckoningSetMotors(int speedA, int speedB) {
  deadReckoningNextMotorSpeedA = round(speedA*calibratedMotorPwm(0.97, 1.0, speedA));
  deadReckoningNextMotorSpeedB = round(speedB*calibratedMotorPwm(1.0, 0.97, speedB));
}

/* TODO locking of shared variables? */
long deadReckoningGetMotorADistance() {
  return deadReckoningDistanceA;
}
long deadReckoningGetMotorBDistance() {
  return deadReckoningDistanceB;
}
void deadReckoningReset() {
  deadReckoningDistanceA=0;
  deadReckoningDistanceB=0;
  deadReckoningTime=0;
}

double calibratedMotorPwm(double A, double B, double speed) {
  return (B-A)/255*speed + A;
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

double maxWheelDifference = 50.0;
void followLine(double speed) {
  double linePosition = robot.readLine(sensors, IR_EMITTERS_ON);
  double turn = maxWheelDifference * (linePosition - 2000.0)/2000.0;
  deadReckoningSetMotors(round(speed+turn),round(speed-turn));
}

int totalMilliseconds = 0;
int delayMilliseconds = 10;
unsigned int last_proportional = 0;
long integral = 0;
double jumpspeed = 0;
double direction = -1.0;
double maxHeight = 11.0;
double x = 0.0;
double y = 9.0;
double theta = 3.1415927;

int speedFunction(){

    double sl = deadReckoningGetMotorADistance()/12800.0;
    double sr = deadReckoningGetMotorBDistance()/12800.0;
    double b = 1.0; // wheel base
    double time = deadReckoningTime;
    deadReckoningReset();
  if (sl==sr) {
    y += sl*sin(theta);
  } else {
    double s = (sr+sl)/2.0;
    theta += (sr - sl)/b;
    x += s*cos(theta);
    y += s*sin(theta);

 /*
    double r = (a+b)/2.0/(a-b);

    double relativeTheta = 360.0 * (a-b) / 2.0 / 3.1415927;
    theta += relativeTheta;

    double relativeY = r * sin(relativeTheta);
    y += relativeY; //*sin(theta+90);
    */
  }
  if (y>maxHeight) {
    y=maxHeight;
  }
  if (y<0.0) {
    y=0.0;
  }
  double speed = 128.0*(maxHeight - y)/maxHeight;
  if (speed < 16.0) speed = 16.0;
  return round(speed);
}

void turn(double r, int velocity, boolean right) {
  double bottomR = r-0.5;
  double topR = r+0.5;
  
  double ratioBottom = bottomR/topR;
  if (right) {
    deadReckoningSetMotors(velocity, ratioBottom*velocity);
  } else {
    deadReckoningSetMotors(ratioBottom*velocity, velocity);
  }
}

int jeffsReadLine() {
  
}

double getRadiusbyTime(double time) {
  double timeSeconds = time/200.0;
  /* Linear 
  return -0.007*time + 1000;
  */
  /* Parabolic */
  return 1.0*timeSeconds*timeSeconds - 5.0*timeSeconds + 9.0;
}

boolean atCross() {
  return sensors[0]>200 && sensors[4]>200;
}

void loop() {
  delay(1);
  /* Determine at what speed we should be travelling */
  int speed = speedFunction();
  
   unsigned int position = robot.readLine(sensors, IR_EMITTERS_ON);
  lcd.clear();
lcd.print(position);
//lcd.print(",");
//lcd.print(sensors[4]);
//lcd.gotoXY(0,1);
//lcd.print(round(100.0*theta));

   if (atCross()) {
    deadReckoningSetMotors(0,0);
    delay(500);
    deadReckoningSetMotors(-62,62);
    delay(480);
    deadReckoningSetMotors(0,0);
    delay(5000);
    direction = -1.0 * direction;
    deadReckoningTime = 0;
    deadReckoningReset();
    if (direction>0)
      theta=0.0;
    else
      theta=3.1415927;
    deadReckoningReset();
    y=9.0;
    return;
   } else if(position == 4000 || position == 0) {
    double r = getRadiusbyTime((direction>0)?16:16);
    if (direction>0) r = 10;
    else r=11;
    speed = jumpspeed*(r/9.0);
    turn(r, speed, direction > 0.0);
   } else {
    deadReckoningTime = 0;
    jumpspeed = speed;
    int proportional = (int)position - 2000;
    int derivative = proportional - last_proportional;
    integral += proportional;
    last_proportional = proportional;
    int power_difference = proportional/20 + integral/10000 + derivative*3/2;
//    const int maximum = 100;
    if (power_difference > speed)
      power_difference = speed;
    if (power_difference < -speed)
      power_difference = -speed;
  
    if (power_difference < 0)
      deadReckoningSetMotors(speed + power_difference, speed);
    else
      deadReckoningSetMotors(speed, speed - power_difference);
   }
  return;
//deadReckoningSetMotors(speed,speed);
//return;
  // Get the position of the line.  Note that we *must* provide
  // the "sensors" argument to read_line() here, even though we
  // are not interested in the individual sensor readings.

  
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
