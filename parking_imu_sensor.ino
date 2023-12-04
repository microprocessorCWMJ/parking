/* 아래 코드관련 실습에 대한 설명과 회로도 및 자료는 https://rasino.tistory.com/ 에 있습니다    */
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

// I2C 통신을 통해 가속도와 자이로 값을 불러오기 때문에 SCL, SDA pin만을 사용한다.
void setup() {
  Serial.begin(9600);
  delay(200);
}

void loop() {
  if(!driving_mode){
    detect_tilted_parking();
    

    if(untilted_parking && parallel_with_beside_kickboard && on_the_parking_line){
      // if(주차 버튼을 눌러준 경우) { Serial.println("주차 완료") } 가 되도록 수정할 예정
      Serial.println("주차가 완료되었습니다.");
      parking_complete = true;
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


