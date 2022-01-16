#include <Dynamixel2Arduino.h>
#include <PID_v1.h>
#include <Servo.h>

// Radio
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(7, 8); // CE, CSN
const byte addresses[][6] = {"00001", "00002"}; 

//**************remote control****************
struct RECEIVE_DATA_STRUCTURE {
  //put your variable definitions here for the data you want to send
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  int16_t menuDown;
  int16_t Select;
  int16_t menuUp;
  int16_t toggleBottom;
  int16_t toggleTop;
  int16_t toggle1;
  int16_t toggle2;
  int16_t mode;
  int16_t RLR;
  int16_t RFB;
  int16_t RT;
  int16_t LLR;
  int16_t LFB;
  int16_t LT;
};

RECEIVE_DATA_STRUCTURE mydata_remote;

float RLR = 0;
float RFB = 0;
float RT = 340;
float LLR = 0;
float LFB = 0;
float LT = 0;

unsigned long currentMillis;
unsigned long previousMillis;

Servo myservo;  // create servo object to control a servo

// Gyro PID

double Pk1 = 2;  
double Ik1 = 10;
double Dk1 = 0.1;

double SetpointAccum;
double SetpointAccum2;
double Output1a;

double Setpoint1, Input1, Output1;    // PID variables
PID PID1(&Input1, &Output1, &Setpoint1, Pk1, Ik1 , Dk1, DIRECT);    // PID Setup

// wheel PID

double Pk2 = 4;  
double Ik2 = 5;
double Dk2 = 0.1;

double Setpoint2, Input2, Output2, Output2a;    // PID variables
PID PID2(&Input2, &Output2, &Setpoint2, Pk2, Ik2 , Dk2, DIRECT);    // PID Setup

float pot;
float IMUroll;
float IMUpitch;

float var1;
float var2;
int check;

int sw1;
int sw2;
float pot1;   // ESC speed
float pot2;   // setpoint trim

#define DXL_SERIAL   Serial3
#define DEBUG_SERIAL Serial
const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN 

const uint8_t DXL_ID = 1;
const float DXL_PROTOCOL_VERSION = 2.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

//This namespace is required to use Control table item names
using namespace ControlTableItem;

void setup() {
  // put your setup code here, to run once:

  radio.begin();
  radio.openWritingPipe(addresses[0]); // 00002
  radio.openReadingPipe(1, addresses[1]); // 00001
  radio.setPALevel(RF24_PA_MIN);

  radio.startListening();

  pinMode(50, INPUT_PULLUP);    // ODrive init

  PID1.SetMode(AUTOMATIC);              
  PID1.SetOutputLimits(-90, 90);
  PID1.SetSampleTime(10);

  PID2.SetMode(AUTOMATIC);              
  PID2.SetOutputLimits(-127, 127);
  PID2.SetSampleTime(10);

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(34, INPUT_PULLUP);
  pinMode(36, INPUT_PULLUP);

  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);

  
  myservo.attach(3);  // attaches the servo on pin 3 to the servo object
  
  // Use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(115200);
  //while(!DEBUG_SERIAL);

  Serial2.begin(115200); // read IMU data

  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(57600);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  dxl.ping(DXL_ID);

  // Turn off torque when configuring items in EEPROM area
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_POSITION);
  dxl.torqueOn(DXL_ID);

  // Limit the maximum velocity in Position Control Mode. Use 0 for Max speed
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID, 500);
}

void loop() {

    currentMillis = millis();

    if (currentMillis - previousMillis >= 10) {  // start timed loop
          previousMillis = currentMillis;

          // check for radio data
          if (radio.available()) {
            radio.read(&mydata_remote, sizeof(RECEIVE_DATA_STRUCTURE));
          }

          RFB = thresholdStick(mydata_remote.RFB);          
          RLR = thresholdStick(mydata_remote.RLR);
          RT = thresholdStick(mydata_remote.RT);
          LFB = thresholdStick(mydata_remote.LFB);
          LLR = thresholdStick(mydata_remote.LLR);
          LT = thresholdStick(mydata_remote.LT); 

          RFB = RFB / 20;
          LLR = LLR / 50;

          //Serial.print(RFB);
          //Serial.print(" , ");

          pot2 = analogRead(A1);        // scale set point trim
          pot2 = (pot2 - 512)/100; 

          pot1 = analogRead(A0);
          pot1 = map(pot1,0,1023,1500,2200);
          myservo.write(pot1);               // drive gyro ESCs

          sw1 = digitalRead(34);        
          sw2 = digitalRead(36);

          if (Serial2.available() > 1){
                    check = Serial2.parseInt();
                    if (check == 500) {                   // look for check character to check it's the start of the data
                        var1 = Serial2.parseInt();
                        var2 = Serial2.parseInt();
    
                        if (Serial2.read() == '\n') {     // end of IMU data 
                            IMUroll = var1 / 100;         // divide by 100 to get our decimal places back
                            IMUpitch = var2 / 100;
                        }
                    }
          }
          
          //Serial.print(IMUroll);
          //Serial.print(" , ");
          //Serial.print(IMUpitch);
          //Serial.print(" , ");
          //Serial.print(sw1);
          //Serial.print(" , ");
          //Serial.print(sw2);
          //Serial.print(" , ");
          //Serial.print(pot1);
          //Serial.print(" , ");
          //Serial.print(pot2);
          //Serial.print(",");
          //Serial.print(SetpointAccum2);
          //Serial.println();       
      
          
          SetpointAccum2 = SetpointAccum2 + Output1/1500;  
          SetpointAccum2 = constrain(SetpointAccum2,-1,1);

          // gyro PID
      
          Setpoint1 = (SetpointAccum2) + pot2 + LLR;    // fine tune cetre point
          Input1 = IMUroll;       // use IMU over serial as input
          PID1.Compute();     // compute PID output    
      
          Output1 = constrain(Output1, -35, 35);    // make sure we don't turn gyros further than 25'
          Output1a = Output1+180;                    // use the centre part of the Dynamixel rotation 

          if (sw2 == 0) {   // enable Gyro servo
              dxl.setGoalPosition(DXL_ID, Output1a, UNIT_DEGREE);    // drive Dynamixel to move gyros            
          }                

          // wheel PID

          Setpoint2 = RFB;         // read controller to drive
          Input2 = IMUpitch;
          PID2.Compute();

          if (sw1 == 0) {   // enable main drive wheel
              // drive DC motor here

              if (Output2 > 0) {
                Output2a = abs(Output2);
                analogWrite(5,Output2a);
                analogWrite(6,0);                
              }
              else if (Output2 < 0) {
                Output2a = abs(Output2);
                analogWrite(6,Output2a);
                analogWrite(5,0);                
              }
              else {
                Output2a = abs(Output2);
                analogWrite(6,0);
                analogWrite(5,0);                
              }
          }

          else  {     // disable drive wheel
            analogWrite(6,0);
            analogWrite(5,0);             
          }


    }   // end of 10ms loop

  

}
