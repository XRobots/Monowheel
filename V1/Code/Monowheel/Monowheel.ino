#include <PID_v1.h>
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

#include <Servo.h>
Servo myservo;

MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;

#define Gyr_Gain 0.00763358 

#define LED_PIN 13
bool blinkState = false;

float AccelX;
float AccelY;
float AccelZ;

float GyroX;
float GyroY;
float GyroZ;

float mixX;
float mixY;
float mixYa;

float servoOut;
float servoOffset = 0;

float dt=0.01;

unsigned long start1;
int pwm1;
int pwm1a;
unsigned long start2;
int pwm2;
int pwm2a;

double Pk1 = 12;  
double Ik1 = 0;
double Dk1 = 0;

double Setpoint1, Input1, Output1, Output1a;    // PID variables
PID PID1(&Input1, &Output1, &Setpoint1, Pk1, Ik1 , Dk1, DIRECT);    // PID Setup

double Pk2 = 10;  
double Ik2 = 0;
double Dk2 = 0;

double Setpoint2, Input2, Output2, Output2a;    // PID variables
PID PID2(&Input2, &Output2, &Setpoint2, Pk2, Ik2 , Dk2, DIRECT);    // PID Setup

unsigned long currentMillis;
long previousMillis = 0;    // set up timers
long interval = 10;        // time constant for timer

void setup() {

    Wire.begin();
    accelgyro.initialize();

    pinMode(2, INPUT);    // input from r/c rec
    pinMode(3, INPUT);    // input from r/c rec
    pinMode(6,OUTPUT);
    pinMode(5,OUTPUT);
    attachInterrupt(0, timeit1, CHANGE);     // pin 2
    attachInterrupt(1, timeit2, CHANGE);     // pin 3

    PID1.SetMode(AUTOMATIC);              
    PID1.SetOutputLimits(-255, 255);
    PID1.SetSampleTime(10);

    PID2.SetMode(AUTOMATIC);              
    PID2.SetOutputLimits(-300,300);
    PID2.SetSampleTime(10);

    myservo.attach(7);
    myservo.writeMicroseconds(1500+servoOffset); 

    Serial.begin(115200);
    
}

void loop() {
  
    currentMillis = millis();
    if (currentMillis - previousMillis >= 10) {  // start timed event
        previousMillis = currentMillis;

        accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);    
        AccelX = ax;
        AccelY = ay;
        AccelZ = az;
        GyroX = Gyr_Gain * (gx);
        GyroY = Gyr_Gain * (gy)*-1;
        GyroZ = Gyr_Gain * (gz);
      
        AccelY = (atan2(AccelY, AccelZ) * 180 / PI);
        AccelX = (atan2(AccelX, AccelZ) * 180 / PI);
  
        float K = 0.8;
        float A = K / (K + dt);
      
        mixX = A *(mixX+GyroX*dt) + (1-A)*AccelY;    
        mixY = A *(mixY+GyroY*dt) + (1-A)*AccelX;

        mixYa = mixY + 3.3;

        //Serial.println(mixYa, 2);        // roll
        //Serial.print(",");
        //Serial.println(mixX, 2);      // pitch  

                

        pwm1a = map(pwm1,1000,2000,-45,45);
        Setpoint1 = pwm1a;
        Input1 = mixX;
        PID1.Compute();

        pwm2a = map(pwm2,1000,2000,20,-20);
        Setpoint2 = pwm2a;
        Input2 = mixY;
        PID2.Compute();

        Serial.println(Output2);

        myservo.writeMicroseconds(1550 + Output2);

            
        if (Output1 > 0) {                  // drive forwards
            Output1a = abs(Output1);
            analogWrite(6,Output1a);
            analogWrite(5,0);
        }
    
        else if (Output1 < 0) {             // drive backwards
            Output1a = abs(Output1);
            analogWrite(5,Output1a);
            analogWrite(6,0);
        }
    
        else {
            analogWrite(5,0);
            analogWrite(6,0);
        }


        if (pwm1a < 4 && pwm1a > -4) {        // make sure motors are off if the stick is in the middle
            analogWrite(5,0);
            analogWrite(6,0);
        }
    
    
        }   // end of timed loop 
    
}


// timer functions

void timeit1() {
    if (digitalRead(2) == HIGH) {
      start1 = micros();
    }
    else {
      pwm1 = micros() - start1;
    }
  }


void timeit2() {
    if (digitalRead(3) == HIGH) {
      start2 = micros();
    }
    else {
      pwm2 = micros() - start2;
    }
  }
