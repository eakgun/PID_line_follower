#include <AFMotor.h>    //Adafruit Motor Shield Library. First you must download and install AFMotor library
#include <QTRSensors.h> //Pololu QTR Sensor Library. First you must download and install QTRSensors library
  
AF_DCMotor motor1(1, MOTOR12_1KHZ ); //create motor #1 using M1 output on Motor Drive Shield, set to 1kHz PWM frequency
AF_DCMotor motor2(2, MOTOR12_1KHZ ); //create motor #2 using M2 output on Motor Drive Shield, set to 1kHz PWM frequency
  
#define KP 1.2 //experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
#define KD 6.1//experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd)
 
#define M1_min 100  //minimum speed of the Motor1
#define M2_min 100  //minimum speed of the Motor2
#define M1_max 150   //max. speed of the Motor1
#define M2_max 150  //max. speed of the Motor2
#define NUM_SENSORS 6         //number of sensors used
#define TIMEOUT 2500          //waits for 2500 us for sensor outputs to go low
#define DEBUG 1


QTRSensorsRC qtrrc((unsigned char[]) {A0,A1,A2,A3,A4,A5} ,NUM_SENSORS, TIMEOUT);
  
unsigned int sensorValues[NUM_SENSORS];


void setup()
{
pinMode(2,INPUT);
Serial.begin(9600);
delay(1500);
int i;
for (i = 0; i < 250; i++)
{
qtrrc.calibrate(QTR_EMITTERS_ON);
delay(20);
}
set_motors(0,0);
}
int error = 0;
int lastError = 0;
int last_proportional = 0;
int integral = 0;
unsigned int sensors[6];
unsigned int position = qtrrc.readLine(sensors);

void loop(){
  
  int mz80 = digitalRead(2); 
  position = qtrrc.readLine(sensors);
  
  PID_control();
  serialData();
  lost();
}
// -------------------------------------------------------------------FUNCTIONS-------------------------------------------------------------------------------
void PID_control(){
  sensors[6];
  position = qtrrc.readLine(sensors);
  error = 2500 - position; 

int motorSpeed = KP * error + KD * (error - lastError);
lastError = error;
  
int leftMotorSpeed = M1_min + motorSpeed;
int rightMotorSpeed = M2_min - motorSpeed;

set_motors(leftMotorSpeed, rightMotorSpeed);
  
  }
 
void set_motors(int motor1speed, int motor2speed) 
{
if (motor1speed > M1_max ) motor1speed = M1_max;
if (motor2speed > M2_max ) motor2speed = M2_max;
if (motor1speed < 0) motor1speed = 0; 
if (motor2speed < 0) motor2speed = 0;
motor1.setSpeed(motor1speed); 
motor2.setSpeed(motor2speed);
motor1.run(FORWARD); 
motor2.run(FORWARD);
  }
// --------------------------------------------------------------------------------------------------------------------------------------------------  
void serialData(){
    for (unsigned char i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(sensors[i]);
 
    Serial.print('\t');
  }
  Serial.print(error);
  Serial.print('\t');
  Serial.println(position); // comment this line out if you are using raw values

  }
// --------------------------------------------------------------------------------------------------------------------------------------------------  
void mz80 (){
    if (mz80 == 0){
    
   motor1.run(RELEASE);
   motor2.run(RELEASE);
   delay(400);
   if(mz80==0){
    motor1.run(FORWARD);
    motor2.run(FORWARD);
    motor1.setSpeed(0);
    motor2.setSpeed(90);
    delay(800);
    motor1.setSpeed(0);
    motor2.setSpeed(0);
    delay(250);
    position = qtrrc.readLine(sensors);
    while( sensors[0] > 750 && sensors[1] > 750 && sensors[2] > 750 && sensors[3] > 750 && sensors[4] > 750 && sensors[5] > 750 ){
       motor1.setSpeed(70);
       motor2.setSpeed(70);
       position = qtrrc.readLine(sensors);
      }
    
     
    
    }
    return;
  }
    }
// --------------------------------------------------------------------------------------------------------------------------------------------------  
void lost(){
    if( sensors[0] > 750 && sensors[1] > 750 && sensors[2] > 750 && sensors[3] > 750 && sensors[4] > 750 && sensors[5] > 750 ){
    while (sensors[0] > 750 && sensors[1] > 750 && sensors[2] > 750 && sensors[3] > 750 && sensors[4] > 750 && sensors[5] > 750 ){
        motor1.run(BACKWARD);
        motor2.run(BACKWARD);
        motor1.setSpeed(80);
        motor2.setSpeed(80);
        delay(100);
        position = qtrrc.readLine(sensors);
      
        }
         motor1.run(FORWARD);
         motor2.run(FORWARD);
     
    return;
  }
    }
 void kurtar(){
    if( (sensors[0] < 300 && sensors[1] < 300 && sensors[2] < 300  && sensors[5] > 750)|| (sensors[0] > 750 && sensors[3] < 300 && sensors[4] < 300  && sensors[5] < 300) ){
      motor1.setSpeed(0);
      motor2.setSpeed(0);
    while (sensors[0] < 300 && sensors[1] < 300 && sensors[2] < 300  && sensors[5] > 750){
        motor1.run(BACKWARD);
        motor1.setSpeed(80); 
        motor2.setSpeed(0);
        position = qtrrc.readLine(sensors);
        }
        
    while (sensors[0] > 750 && sensors[3] < 300 && sensors[4] < 300  && sensors[5] < 300){
        motor2.run(BACKWARD);
        motor2.setSpeed(80); 
        motor1.setSpeed(0);
        position = qtrrc.readLine(sensors);
        }
    motor1.run(FORWARD); 
    motor2.run(FORWARD);
    return;
  }
    }
   
void manual_calibration() {
  
int i;
for (i = 0; i < 150; i++)
{
qtrrc.calibrate(QTR_EMITTERS_ON);
delay(20);
}
  
if (DEBUG) {
Serial.begin(9600);
for (int i = 0; i < NUM_SENSORS; i++)
{
Serial.print(qtrrc.calibratedMinimumOn[i]);
Serial.print(' ');
}
Serial.println();
  
for (int i = 0; i < NUM_SENSORS; i++)
{
Serial.print(qtrrc.calibratedMaximumOn[i]);
Serial.print(' ');
}
Serial.println();
Serial.println();
}
}
