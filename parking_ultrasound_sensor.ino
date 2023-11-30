//We wrote the code first for the sensors to be attached to the left side of the kickboard
//It needs to be written about right side
#define LeftFrontpinTrig 42
#define LeftFrontpinEcho 48
#define LeftBehindpinTrig 26
#define LeftBehindpinEcho 35

double measureDistanceCmFront(){
  digitalWrite(LeftFrontpinTrig, LOW);
  delayMicroseconds(5);
  digitalWrite(LeftFrontpinTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(LeftFrontpinTrig, LOW);
  
  double duration_front = pulseIn(LeftFrontpinEcho, HIGH);
  double cm_front = 0.0343 * (duration_front/2);
  return cm_front;
}

double measureDistanceCmBehind(){
  digitalWrite(LeftBehindpinTrig, LOW);
  delayMicroseconds(5);
  digitalWrite(LeftBehindpinTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(LeftBehindpinTrig, LOW);
  
  double duration_behind = pulseIn(LeftBehindpinEcho, HIGH);
  double cm_behind = 0.0343 * (duration_behind/2);
  return cm_behind;
}


// length of kickboard (It needs to be updated)
double kickboard_length_cm = 3.5;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(LeftFrontpinTrig, OUTPUT);
  pinMode(LeftFrontpinEcho, INPUT);
  pinMode(LeftBehindpinTrig, OUTPUT);
  pinMode(LeftBehindpinEcho, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  double distance_front = measureDistanceCmFront();
  double distance_behind = measureDistanceCmBehind();

  // To prevent distance errors.
  if (distance_front >= 1000 or distance_behind >= 1000){
    Serial.println("distance Error");
  }

  else{
    if (distance_front >= distance_behind){
      double distance_diff = distance_front - distance_behind;
      double cos_angle = acos(distance_diff / kickboard_length_cm);
      cos_angle = cos_angle * 180 / 3.14;
      Serial.print("distance_diff: ");
      Serial.print(distance_diff);
      Serial.print(" / cos_angle: ");
      Serial.println(cos_angle);
      if (cos_angle <= 75){
        Serial.println("기울인 채로 주차하지 마시오.");
      }
    }

    else{
      double distance_diff = distance_behind - distance_front;
      double cos_angle = acos(distance_diff / kickboard_length_cm);
      cos_angle = cos_angle * 180 / 3.14;
      Serial.print("distance_diff: ");
      Serial.print(distance_diff);
      Serial.print(" / cos_angle: ");
      Serial.println(cos_angle);

      // Crieria of wrong parking angle: 75 degree (It needs to be updated)
      if (cos_angle <= 75){
        Serial.println("기울인 채로 주차하지 마시오.");
      }
    }
  }
  delay(100);
}