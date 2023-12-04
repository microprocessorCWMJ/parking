//We wrote the code first for the sensors to be attached to the left side of the kickboard
//It needs to be written about right side
#define LeftFrontpinTrig 22
#define LeftFrontpinEcho 24
#define LeftBehindpinTrig 26
#define LeftBehindpinEcho 28
#define RightFrontpinTrig 30
#define RightFrontpinEcho 32
#define RightBehindpinTrig 34
#define RightBehindpinEcho 36

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
      if (distance_LeftFront >= 300 && distance_LeftBehind >= 300 && distance_RightFront >= 300 && distance_RightBehind > 300){
        parallel_with_beside_kickboard = true;
      }

      // Assume that no kickboard on your right side
      else if(distance_RightFront >= 300 && distance_RightBehind > 300){
        if (distance_LeftFront >= distance_LeftBehind){
          double distance_Leftdiff = distance_LeftFront - distance_LeftBehind;
          double cos_angle_Left = acos(distance_Leftdiff / kickboard_length_cm);
          cos_angle_Left = cos_angle_Left * RADIAN_TO_DEGREE;
          // Serial.print("distance_diff: ");
          // Serial.print(distance_diff);
          // Serial.print(" / cos_angle: ");
          // Serial.println(cos_angle);

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

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(LeftFrontpinTrig, OUTPUT);
  pinMode(LeftFrontpinEcho, INPUT);
  pinMode(LeftBehindpinTrig, OUTPUT);
  pinMode(LeftBehindpinEcho, INPUT);

  pinMode(RightFrontpinTrig, OUTPUT);
  pinMode(RightFrontpinEcho, INPUT);
  pinMode(RightBehindpinTrig, OUTPUT);
  pinMode(RightBehindpinEcho, INPUT);
}

void loop() {
  if(!driving_mode){
    measureAngle_for_parking();
  }
}