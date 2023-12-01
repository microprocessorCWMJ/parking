#define S2 30        
#define S3 28      
#define sensorOut1 26 

#define S22 27
#define S33 29
#define sensorOut2 31


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

uint8_t park_flag1 = 0, park_flag2 = 0, park_flag3 = 0, x = 0; //Total three colors

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
  Red1 = getRed(1);
  Red2 = getRed(2);
  redValue1 = map(Red1, R_Min,R_Max,255,0); // all of the values need to be calibrated. This is temporay value. 
  //We should add calibrating code or We should set the min & max value by hand.
  redValue2 = map(Red2, R_Min,R_Max,255,0);
  delay(200);
 
  Green1 = getGreen(1);
  Green2 = getGreen(2);
  greenValue1 = map(Green1, G_Min,G_Max,255,0);
  greenValue2 = map(Green2, G_Min,G_Max,255,0);
  delay(200);
 
  Blue1 = getBlue(1);
  Blue2 = getBlue(2);
  blueValue1 = map(Blue1, B_Min,B_Max,255,0);
  blueValue2 = map(Blue2, B_Min,B_Max,255,0); 
  delay(200);

  Serial.print("Red1 = ");
  Serial.print(redValue1);  
  Serial.print("    ");
  Serial.print("Green1 = ");
  Serial.print(greenValue1); 
  Serial.print("    ");
  Serial.print("Blue1 = ");
  Serial.println(blueValue1);
  delay(200);   

  Serial.print("Red2 = ");
  Serial.print(redValue2);  
  Serial.print("    ");
  Serial.print("Green2 = ");
  Serial.print(greenValue2); 
  Serial.print("    ");
  Serial.print("Blue2 = ");
  Serial.println(blueValue2);
  delay(200);   

  
  park();
  
  if(park_flag1==1){
    Serial.println("color1 detected");
    Serial.println("GOOD");
    delay(500);

    exit(0);
  }
  else if(park_flag2==1){
    Serial.println("color2 detected");
    Serial.println("GOOD");
    delay(500);

    exit(0);
  }
  else if(park_flag3==1){
    Serial.println("color3 detected");
    Serial.println("GOOD");
    delay(500);

    exit(0);
  }
  else{
    Serial.println("Please match the line");
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

int park() {

  Serial.println("Stand still for 5 seconds...");

  Serial.print("Red1 = ");
  Serial.print(redValue1);  
  Serial.print("    ");
  Serial.print("Green1 = ");
  Serial.print(greenValue1); 
  Serial.print("    ");
  Serial.print("Blue1 = ");
  Serial.println(blueValue1);
  delay(200);   

  Serial.print("Red2 = ");
  Serial.print(redValue2);  
  Serial.print("    ");
  Serial.print("Green2 = ");
  Serial.print(greenValue2); 
  Serial.print("    ");
  Serial.print("Blue2 = ");
  Serial.println(blueValue2);
  delay(200);   

  delay(5000);

  if(169<redValue1 && redValue1<209 && 135<greenValue1 && greenValue1<175 && 155<blueValue1 && blueValue1<195){ //I set the wide range of values. This must be changed.
    if(169<redValue2 && redValue2<209 && 135<greenValue2 && greenValue2<175 && 155<blueValue2 && blueValue2<195){
      park_flag1 = 1;
      return park_flag1;
    }

  }
  else if(169<redValue1 && redValue1<209 && 135<greenValue1 && greenValue1<175 && 155<blueValue1 && blueValue1<195){ //I set the wide range of values. This must be changed.
    if(169<redValue2 && redValue2<209 && 135<greenValue2 && greenValue2<175 && 155<blueValue2 && blueValue2<195){
      park_flag2 = 1;
      return park_flag2;
    }
  }

  else if(169<redValue1 && redValue1<209 && 135<greenValue1 && greenValue1<175 && 155<blueValue1 && blueValue1<195){ //I set the wide range of values. This must be changed.
    if(169<redValue2 && redValue2<209 && 135<greenValue2 && greenValue2<175 && 155<blueValue2 && blueValue2<195){
      park_flag3 = 1;
      return park_flag3;
    }
  }

  else{
      return 0;
  }
}

