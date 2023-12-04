#define S2 31
#define S3 33      
#define sensorOut1 35 

#define S22 37
#define S33 39
#define sensorOut2 41

#define LeftFrontpinTrig 22
#define LeftFrontpinEcho 24
#define LeftBehindpinTrig 26
#define LeftBehindpinEcho 28
#define LeftFrontpinTrig 30
#define LeftFrontpinEcho 32
#define LeftBehindpinTrig 34
#define LeftBehindpinEcho 36

#include <Wire.h>

const int MPU_ADDR = 0x68;  // I2C통신을 위한 MPU6050의 주소
//int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;   // 가속도(Acceleration)와 자이로(Gyro)
int16_t AcX, AcY, AcZ;
double angleAcY;
const double RADIAN_TO_DEGREE = 180 / 3.14159;

// length of kickboard (It needs to be updated)
static double kickboard_length_cm = 3.5;

//----------------------------------------------------For Driving Mode--------------------------------------------------//
// If users change the button to park, driving mode must be false.
bool driving_mode = true;
// If white color is repetitively detected, on_the_crosswalk flag is set.
// It names "on_the_crosswalk", but it will be also setted when user is changing lane repetitivley.
bool on_the_crosswalk = true;
// If ultrasound is detected objects repetitively, many_objects_around flag is set.
bool many_objects_around = true;
//----------------------------------------------------------------------------------------------------------------------//


//---------------------------------------------------For Parking Mode---------------------------------------------------//
bool untilted_parking = false;
bool parallel_with_beside_kickboard = false;
bool on_the_parking_line = false;

// If parking is completed, parking completion flag is set.
bool parking_complete = false;
//----------------------------------------------------------------------------------------------------------------------//

int R_Min = 5; 
int R_Max = 25; 
int G_Min = 4; 
int G_Max = 42;
int B_Min = 4; 
int B_Max = 45; 

int Red1 = 0, Red2 = 0;
int Green1 = 0, Green2 = 0;
int Blue1 = 0, Blue2 = 0;

int redValue1, redValue2;
int greenValue1, greenValue2;
int blueValue1, blueValue2;
int Frequency;

bool park_flag_Red, park_flag_Green, park_flag_Blue; //Total three colors

void setup() {
  pinMode(S2, OUTPUT);    
  pinMode(S3, OUTPUT);      
  pinMode(sensorOut1, INPUT); 
  pinMode(S22, OUTPUT);    
  pinMode(S33, OUTPUT);      
  pinMode(sensorOut2, INPUT); 
  Serial.begin(115200);      
  delay(1000);            
}

void loop() {
  if(!driving_mode){
    detect_parking_line();
  }
}

int getRed(int x) {
  if(x==1){
    digitalWrite(S2,LOW);
    digitalWrite(S3,LOW);
    Frequency = pulseIn(sensorOut1, LOW);
  }
  else if(x==2){
    digitalWrite(S22,LOW);
    digitalWrite(S33,LOW);
    Frequency = pulseIn(sensorOut2, LOW);
  }

  return Frequency;
}

int getGreen(int x) {
  if(x==1){
    digitalWrite(S2,HIGH);
    digitalWrite(S3,HIGH);
    Frequency = pulseIn(sensorOut1, LOW);
  }
  else if(x==2){
    digitalWrite(S22,HIGH);
    digitalWrite(S33,HIGH);
    Frequency = pulseIn(sensorOut2, LOW);
  }

  return Frequency;
}

int getBlue(int x) {
  if(x==1){
    digitalWrite(S2,LOW);
    digitalWrite(S3,LOW);
    Frequency = pulseIn(sensorOut1, LOW);
  }
  else if(x==2){
    digitalWrite(S22,LOW);
    digitalWrite(S33,LOW);
    Frequency = pulseIn(sensorOut2, LOW);
  }
  return Frequency;
}

void park() {
  // When detect red line
  if(169<redValue1 && redValue1<209 && 135<greenValue1 && greenValue1<175 && 155<blueValue1 && blueValue1<195){ //I set the wide range of values. This must be changed.
    if(169<redValue2 && redValue2<209 && 135<greenValue2 && greenValue2<175 && 155<blueValue2 && blueValue2<195){
      park_flag_Red = true;
    }
  }

  // When detect green line
  else if(169<redValue1 && redValue1<209 && 135<greenValue1 && greenValue1<175 && 155<blueValue1 && blueValue1<195){ //I set the wide range of values. This must be changed.
    if(169<redValue2 && redValue2<209 && 135<greenValue2 && greenValue2<175 && 155<blueValue2 && blueValue2<195){
      park_flag_Green = true;
    }
  }

  // When detect blue line
  else if(169<redValue1 && redValue1<209 && 135<greenValue1 && greenValue1<175 && 155<blueValue1 && blueValue1<195){ //I set the wide range of values. This must be changed.
    if(169<redValue2 && redValue2<209 && 135<greenValue2 && greenValue2<175 && 155<blueValue2 && blueValue2<195){
      park_flag_Blue = true;
    }
  }

  // When detected color is not R,G and B.
  else{
      park_flag_Red = false;
      park_flag_Green = false;
      park_flag_Blue = false;
  }
}

void detect_parking_line(){
  Red1 = getRed(1);
  Red2 = getRed(2);
  redValue1 = map(Red1, R_Min,R_Max,255,0); // all of the values need to be calibrated. This is temporay value. 
  //We should add calibrating code or We should set the min & max value by hand.
  redValue2 = map(Red2, R_Min,R_Max,255,0);
 
  Green1 = getGreen(1);
  Green2 = getGreen(2);
  greenValue1 = map(Green1, G_Min,G_Max,255,0);
  greenValue2 = map(Green2, G_Min,G_Max,255,0);
 
  Blue1 = getBlue(1);
  Blue2 = getBlue(2);
  blueValue1 = map(Blue1, B_Min,B_Max,255,0);
  blueValue2 = map(Blue2, B_Min,B_Max,255,0); 
 
  park();
  
  if(!park_flag_Red && !park_flag_Green && !park_flag_Blue){
    on_the_parking_line = false;
  }
  else if(park_flag_Red){
    on_the_parking_line = false;
  }
  else if(park_flag_Green){
    on_the_parking_line = true;
  }
  else if(park_flag_Blue){
    on_the_parking_line = true;
  }
}

