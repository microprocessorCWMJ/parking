#define S2 31
#define S3 33      
#define sensorOut1 35 
#define S22 37
#define S32 39
#define sensorOut2 41
#define S23 43
#define S33 45      
#define sensorOut3 47 
#define S24 49
#define S34 51
#define sensorOut4 53

#define LeftFrontpinTrig 22
#define LeftFrontpinEcho 24
#define LeftBehindpinTrig 26
#define LeftBehindpinEcho 28
#define RightFrontpinTrig 30
#define RightFrontpinEcho 32
#define RightBehindpinTrig 34
#define RightBehindpinEcho 36

#include <Wire.h>

const int MPU_ADDR = 0x68;  // I2C통신을 위한 MPU6050의 주소
const double RADIAN_TO_DEGREE = 180 / 3.14159;

// length of kickboard (It needs to be updated)
const double kickboard_length_cm = 3.5;

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

class parking{
  private:
    int16_t AcX, AcY, AcZ;
    double angleAcY;
    
    int R_Min = 5; 
    int R_Max = 25; 
    int G_Min = 4; 
    int G_Max = 42;
    int B_Min = 4; 
    int B_Max = 45; 

    int Red1 = 0, Red2 = 0, Red3 = 0, Red4 = 0;
    int Green1 = 0, Green2 = 0, Green3 = 0, Green4 = 4;
    int Blue1 = 0, Blue2 = 0, Blue3 = 0, Blue4 = 0;

    int redValue1, redValue2, redValue3, redValue4;
    int greenValue1, greenValue2, greenValue3, greenValue4;
    int blueValue1, blueValue2, blueValue3, blueValue4;
    int Frequency;

    bool park_flag_Red, park_flag_Green, park_flag_Blue; //Total three colors
  
  public:
    int getRed(int x) {
      if(x==1){
        digitalWrite(S2,LOW);
        digitalWrite(S3,LOW);
        Frequency = pulseIn(sensorOut1, LOW);
      }
      else if(x==2){
        digitalWrite(S22,LOW);
        digitalWrite(S32,LOW);
        Frequency = pulseIn(sensorOut2, LOW);
      }
      else if(x==3){
        digitalWrite(S23,LOW);
        digitalWrite(S33,LOW);
        Frequency = pulseIn(sensorOut3, LOW);
      }
      else if(x==4){
        digitalWrite(S24,LOW);
        digitalWrite(S34,LOW);
        Frequency = pulseIn(sensorOut4, LOW);
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
        digitalWrite(S32,HIGH);
        Frequency = pulseIn(sensorOut2, LOW);
      }
      else if(x==3){
        digitalWrite(S23,HIGH);
        digitalWrite(S33,HIGH);
        Frequency = pulseIn(sensorOut3, LOW);
      }
      else if(x==4){
        digitalWrite(S24,HIGH);
        digitalWrite(S34,HIGH);
        Frequency = pulseIn(sensorOut4, LOW);
      }
      return Frequency;
    }

    int getBlue(int x) {
      if(x==1){
      digitalWrite(S2,LOW);
      digitalWrite(S3,HIGH);
      Frequency = pulseIn(sensorOut1, LOW);
      }
      else if(x==2){
      digitalWrite(S22,LOW);
      digitalWrite(S32,HIGH);
      Frequency = pulseIn(sensorOut2, LOW);
      }
      else if(x==3){
      digitalWrite(S23,LOW);
      digitalWrite(S33,HIGH);
      Frequency = pulseIn(sensorOut3, LOW);
      }
      else if(x==4){
        digitalWrite(S24,LOW);
        digitalWrite(S34,HIGH);
        Frequency = pulseIn(sensorOut4, LOW);
      }
      return Frequency;
    }

    void park() {
      // When detect red line
      // I set the wide range of values. This must be changed.
      if(169<redValue1 && redValue1<209 && 135<greenValue1 && greenValue1<175 && 155<blueValue1 && blueValue1<195){
        if(169<redValue2 && redValue2<209 && 135<greenValue2 && greenValue2<175 && 155<blueValue2 && blueValue2<195){
          if(169<redValue3 && redValue3<209 && 135<greenValue3 && greenValue3<175 && 155<blueValue3 && blueValue3<195){
            if(169<redValue4 && redValue4<209 && 135<greenValue4 && greenValue4<175 && 155<blueValue4 && blueValue4<195){
              park_flag_Red = true;
            }
          }
        }
      }

      // When detect green line
      // I set the wide range of values. This must be changed.
      else if(169<redValue1 && redValue1<209 && 135<greenValue1 && greenValue1<175 && 155<blueValue1 && blueValue1<195){ 
        if(169<redValue2 && redValue2<209 && 135<greenValue2 && greenValue2<175 && 155<blueValue2 && blueValue2<195){
          if(169<redValue3 && redValue3<209 && 135<greenValue3 && greenValue3<175 && 155<blueValue3 && blueValue3<195){
            if(169<redValue4 && redValue4<209 && 135<greenValue4 && greenValue4<175 && 155<blueValue4 && blueValue4<195){
              park_flag_Green = true;
            }
          }
        }
      }

      // When detect blue line
      // I set the wide range of values. This must be changed.
      else if(169<redValue1 && redValue1<209 && 135<greenValue1 && greenValue1<175 && 155<blueValue1 && blueValue1<195){ 
        if(169<redValue2 && redValue2<209 && 135<greenValue2 && greenValue2<175 && 155<blueValue2 && blueValue2<195){
          if(169<redValue3 && redValue3<209 && 135<greenValue3 && greenValue3<175 && 155<blueValue3 && blueValue3<195){
            if(169<redValue4 && redValue4<209 && 135<greenValue4 && greenValue4<175 && 155<blueValue4 && blueValue4<195){
              park_flag_Blue = true;
            }
          }
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
      Red3 = getRed(3);
      Red4 = getRed(4);
      // all of the values need to be calibrated. This is temporay value.
      // We should add calibrating code or We should set the min & max value by hand. 
      redValue1 = map(Red1, R_Min,R_Max,255,0); 
      redValue2 = map(Red2, R_Min,R_Max,255,0);
      redValue3 = map(Red3, R_Min,R_Max,255,0);
      redValue4 = map(Red4, R_Min,R_Max,255,0);

      Green1 = getGreen(1);
      Green2 = getGreen(2);
      Green3 = getGreen(3);
      Green4 = getGreen(4);
      greenValue1 = map(Green1, G_Min,G_Max,255,0);
      greenValue2 = map(Green2, G_Min,G_Max,255,0);
      greenValue3 = map(Green3, G_Min,G_Max,255,0);
      greenValue4 = map(Green4, G_Min,G_Max,255,0);

      Blue1 = getBlue(1);
      Blue2 = getBlue(2);
      Blue3 = getBlue(3);
      Blue4 = getBlue(4);
      blueValue1 = map(Blue1, B_Min,B_Max,255,0);
      blueValue2 = map(Blue2, B_Min,B_Max,255,0); 
      blueValue3 = map(Blue3, B_Min,B_Max,255,0);
      blueValue4 = map(Blue4, B_Min,B_Max,255,0);
    
      park();
      
      if(!park_flag_Red && !park_flag_Green && !park_flag_Blue){
        on_the_parking_line = false;
      }
      else {
        on_the_parking_line = true;
      }
    }

    double measureDistanceCm(uint8_t pinTrig, uint8_t pinEcho){
      digitalWrite(pinTrig, LOW);
      delayMicroseconds(5);
      digitalWrite(pinTrig, HIGH);
      delayMicroseconds(10);
      digitalWrite(pinTrig, LOW);
      
      double duration = pulseIn(pinEcho, HIGH);
      double cm = 0.0343 * (duration/2);
      return cm;
    }

    void measureAngle_for_parking(){
      if(on_the_parking_line){

        double distance_LeftFront = measureDistanceCm(LeftFrontpinTrig, LeftFrontpinEcho);
        double distance_LeftBehind = measureDistanceCm(LeftBehindpinTrig, LeftBehindpinEcho);
        double distance_RightFront = measureDistanceCm(RightFrontpinTrig, RightFrontpinEcho);
        double distance_RightBehind = measureDistanceCm(RightBehindpinTrig, RightBehindpinEcho);

        //They will be used when there are kickboards on your both side.
        bool Left_parallel, Right_parallel;

        // To prevent distance errors.
        if(distance_LeftFront < 1000 && distance_LeftBehind < 1000 && distance_RightFront < 1000 && distance_RightBehind < 1000){

          // Assume that no kickboards beside you
          if (distance_LeftFront >= 300 && distance_LeftBehind >= 300 && distance_RightFront >= 300 && distance_RightBehind >= 300){
            parallel_with_beside_kickboard = true;
          }

          // Assume that no kickboard on your right side
          else if(distance_RightFront >= 300 && distance_RightBehind >= 300){
            if (distance_LeftFront >= distance_LeftBehind){
              double distance_Leftdiff = distance_LeftFront - distance_LeftBehind;
              double cos_angle_Left = acos(distance_Leftdiff / kickboard_length_cm);
              cos_angle_Left = cos_angle_Left * RADIAN_TO_DEGREE;

              // Crieria of wrong parking angle: 75 degree (It needs to be updated)
              if (cos_angle_Left <= 75) {
                parallel_with_beside_kickboard = false;
              }
              else {
                parallel_with_beside_kickboard = true;
              }
            }

            else{
              double distance_Leftdiff = distance_LeftBehind - distance_LeftFront;
              double cos_angle_Left = acos(distance_Leftdiff / kickboard_length_cm);
              cos_angle_Left = cos_angle_Left * RADIAN_TO_DEGREE;
              if (cos_angle_Left <= 75){
                parallel_with_beside_kickboard = false;
              }
              else {
                parallel_with_beside_kickboard = true;
              }
            }
          }

          // Assume that no kickboard on your left side
          else if(distance_LeftFront >= 300 && distance_LeftBehind >= 300){
            if (distance_RightFront >= distance_RightBehind){
              double distance_Rightdiff = distance_RightFront - distance_RightBehind;
              double cos_angle_Right = acos(distance_Rightdiff / kickboard_length_cm);
              cos_angle_Right = cos_angle_Right * RADIAN_TO_DEGREE;
              if (cos_angle_Right <= 75) {
                parallel_with_beside_kickboard = false;
              }
              else {
                parallel_with_beside_kickboard = true;
              }
            }

            else{
              double distance_Rightdiff = distance_RightBehind - distance_RightFront;
              double cos_angle_Right = acos(distance_Rightdiff / kickboard_length_cm);
              cos_angle_Right = cos_angle_Right * RADIAN_TO_DEGREE;
              if (cos_angle_Right <= 75){
                parallel_with_beside_kickboard = false;
              }
              else {
                parallel_with_beside_kickboard = true;
              }
            }
          }

          // Assume that there are kickboards on your both side.
          else{
            if (distance_LeftFront >= distance_LeftBehind){
              double distance_Leftdiff = distance_LeftFront - distance_LeftBehind;
              double cos_angle_Left = acos(distance_Leftdiff / kickboard_length_cm);
              cos_angle_Left = cos_angle_Left * RADIAN_TO_DEGREE;
              if (cos_angle_Left <= 75) {
                Left_parallel = false;
              }
              else {
                Left_parallel = true;
              }
            }

            else{
              double distance_Leftdiff = distance_LeftBehind - distance_LeftFront;
              double cos_angle_Left = acos(distance_Leftdiff / kickboard_length_cm);

              cos_angle_Left = cos_angle_Left * RADIAN_TO_DEGREE;
              if (cos_angle_Left <= 75){
                Left_parallel = false;
              }
              else {
                Left_parallel = true;
              }
            }

            if (distance_RightFront >= distance_RightBehind){
              double distance_Rightdiff = distance_RightFront - distance_RightBehind;
              double cos_angle_Right = acos(distance_Rightdiff / kickboard_length_cm);
              cos_angle_Right = cos_angle_Right * RADIAN_TO_DEGREE;
              if (cos_angle_Right <= 75) {
                Right_parallel = false;
              }
              else {
                Right_parallel = true;
              }
            }

            else{
              double distance_Rightdiff = distance_RightBehind - distance_RightFront;
              double cos_angle_Right = acos(distance_Rightdiff / kickboard_length_cm);
              cos_angle_Right = cos_angle_Right * RADIAN_TO_DEGREE;
              if (cos_angle_Right <= 75){
                Right_parallel = false;
              }
              else {
                Right_parallel = true;
              }
            }

            if(Right_parallel && Left_parallel){
              parallel_with_beside_kickboard = true;
            }
            else {
              parallel_with_beside_kickboard = false;
            }
          }
        }
      }
    }

    // 기울기 센서와 아두이노 간 I2C 통신을 시작하게 해준다.
    void initSensor() {
      Wire.begin();
      Wire.beginTransmission(MPU_ADDR);  // I2C 통신용 어드레스(주소)
      Wire.write(0x6B);                  // MPU6050과 통신을 시작하기 위해서는 0x6B번지에
      Wire.write(0);
      Wire.endTransmission(true);
    }

    void getData() {
      Wire.beginTransmission(MPU_ADDR);
      Wire.write(0x3B);  // AcX 레지스터 위치(주소)를 지칭합니다
      Wire.endTransmission(false);
      Wire.requestFrom(MPU_ADDR, 14, true);  // AcX 주소 이후의 14byte의 데이터를 요청
      AcX = Wire.read() << 8 | Wire.read();  //두 개의 나뉘어진 바이트를 하나로 이어 붙여서 각 변수에 저장
      AcY = Wire.read() << 8 | Wire.read();
      AcZ = Wire.read() << 8 | Wire.read();
    }

    double getAngleY() {
      getData();
      // 삼각함수를 이용한 피치(Pitch)의 각도 구하기
      angleAcY = atan(-AcX / sqrt(pow(AcY, 2) + pow(AcZ, 2)));
      angleAcY *= RADIAN_TO_DEGREE;
    }

    void detect_tilted_parking(){
      if(on_the_parking_line && parallel_with_beside_kickboard){
        initSensor();
        getAngleY();
        // 35.26 degree is an error value (experimentally)
        if(angleAcY != 35.26){
        }
        if (AcX != -1 && AcY != -1 && AcZ != -1 && angleAcY != 35.26) {
          if (angleAcY >= 40 || angleAcY <= -40) {
            untilted_parking = false;
          }
          else{
            untilted_parking = true;
          }
        }
      }
    }
};

// I2C 통신을 통해 가속도와 자이로 값을 불러오기 때문에 SCL, SDA pin만을 사용한다.
void setup() {
  Serial.begin(115200);

  pinMode(LeftFrontpinTrig, OUTPUT);
  pinMode(LeftFrontpinEcho, INPUT);
  pinMode(LeftBehindpinTrig, OUTPUT);
  pinMode(LeftBehindpinEcho, INPUT);

  pinMode(RightFrontpinTrig, OUTPUT);
  pinMode(RightFrontpinEcho, INPUT);
  pinMode(RightBehindpinTrig, OUTPUT);
  pinMode(RightBehindpinEcho, INPUT);

  pinMode(S2, OUTPUT);    
  pinMode(S3, OUTPUT);      
  pinMode(sensorOut1, INPUT); 

  pinMode(S22, OUTPUT);    
  pinMode(S32, OUTPUT);      
  pinMode(sensorOut2, INPUT);

  pinMode(S23, OUTPUT);    
  pinMode(S33, OUTPUT);      
  pinMode(sensorOut3, INPUT); 

  pinMode(S24, OUTPUT);    
  pinMode(S34, OUTPUT);      
  pinMode(sensorOut4, INPUT);  

  delay(100);
}

void loop() {
  if(!driving_mode){
    parking parking;
    parking.detect_parking_line();
    parking.measureAngle_for_parking();
    parking.detect_tilted_parking();

    if(untilted_parking && parallel_with_beside_kickboard && on_the_parking_line){
      // if(주차 버튼을 눌러준 경우) { Serial.println("주차 완료") } 가 되도록 수정할 예정
      Serial.println("주차가 완료되었습니다.");
      parking_complete = true;
      delay(30000);
    }

    else{
      //if(주차 버튼을 눌러준 경우) 아래 코드가 실행되도록 수정할 예정
      if(!on_the_parking_line){
        Serial.println("주차 선을 확인해주세요.");
      }
      else if(!parallel_with_beside_kickboard){
        Serial.println("옆 킥보드와 평행하게 주차해주세요. (주변에 장애물이 있다면 치워주세요)");
      }
      else if(!untilted_parking){
        Serial.println("킥보드를 지면과 평행하게 주차해주세요.");
      }
      parking_complete = false;
    }
  }
}
