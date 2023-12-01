#define S2 30        
#define S3 28      
#define sensorOut 26 


int R_Min = 5; 
int R_Max = 25; 
int G_Min = 4; 
int G_Max = 42;
int B_Min = 4; 
int B_Max = 45; 

/*Define int variables*/
int Red = 0;
int Green = 0;
int Blue = 0;

int redValue;
int greenValue;
int blueValue;
int Frequency;

uint8_t park_flag = 0;

void setup() {
  pinMode(S2, OUTPUT);    
  pinMode(S3, OUTPUT);      
  pinMode(sensorOut, INPUT); 
  Serial.begin(115200);      
  delay(1000);            
}

void loop() {
  Red = getRed();
  redValue = map(Red, R_Min,R_Max,255,0); // all of the values need to be calibrated. This is temporay value. 
  //We should add calibrating code or We should set the min&max value by hand.
  delay(200);
 
  Green = getGreen();
  greenValue = map(Green, G_Min,G_Max,255,0);
  delay(200);
 
  Blue = getBlue();
  blueValue = map(Blue, B_Min,B_Max,255,0); 
  delay(200);

  Serial.print("Red = ");
  Serial.print(redValue);  
  Serial.print("    ");
  Serial.print("Green = ");
  Serial.print(greenValue); 
  Serial.print("    ");
  Serial.print("Blue = ");
  Serial.println(blueValue);
  delay(200);   


  
  park();
  
  if(park_flag==1){
    Serial.println("GOOD");
    delay(500);

    exit(0);
  }
  else{
    Serial.println("Please match the line");
  }
}



int getRed() {
  digitalWrite(S2,LOW);
  digitalWrite(S3,LOW);
  Frequency = pulseIn(sensorOut, LOW); /*Get the Red Color Frequency*/
  return Frequency;
}

int getGreen() {
  digitalWrite(S2,HIGH);
  digitalWrite(S3,HIGH);
  Frequency = pulseIn(sensorOut, LOW); /*Get the Green Color Frequency*/
  return Frequency;
}

int getBlue() {
  digitalWrite(S2,LOW);
  digitalWrite(S3,HIGH);
  Frequency = pulseIn(sensorOut, LOW); /*Get the Blue Color Frequency*/
  return Frequency;
}

int park() {

  Serial.println("Stand still for 5 seconds...");
  
  delay(5000);

  if(169<redValue<209 && 135<greenValue<175 && 155<blueValue<195){ //I set the wide range of values. This must be changed.

    park_flag = 1;

    return park_flag;
  }
  
}

