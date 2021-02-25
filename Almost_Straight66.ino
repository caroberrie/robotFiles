

int dirR = 4;
int pwmR = 5;
int pwmL = 6;
int dirL = 7;
int reset;
int candleCount;
int fanLed = 3;
int micLed = 5;
 bool go = false;
 uint8_t oldtimer, oldadc, oldadmux;

 #include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>

//Define Pins
#define EncoderPinAL 18   // Encoder Pin A pin 2 and pin 3 are inturrpt pins
#define EncoderPinAR 19   // Encoder Pin B

volatile int countsR;
volatile int countsL;
int discount=0;
int ldiscount=0;
int rdiscount=0;
int lsCan=0;

double DspeedL = 200;
double DspeedR = 180;
double speedL = DspeedL;
double speedR = DspeedR;
  
void setup() {
  // put youR setup code heRe, to Run once:
  Serial.begin(9600);
  pinMode(dirR, OUTPUT);
  pinMode(pwmR, OUTPUT);
  pinMode(pwmL, OUTPUT);
  pinMode(dirL, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(2, INPUT);
  pinMode(8, INPUT);
  pinMode(fanLed, OUTPUT);
  pinMode(micLed, OUTPUT);
  pinMode(13, INPUT);
  digitalWrite(3, LOW);
  digitalWrite(1, LOW);

  pinMode(EncoderPinAR, INPUT); //initialize Encoder Pins
  digitalWrite(EncoderPinAR, LOW); //initialize Pin States
  pinMode(EncoderPinAL, INPUT); //initialize Encoder Pins
  digitalWrite(EncoderPinAL, LOW); //initialize Pin States
//  attachInterrupt(4, readEncoderR, CHANGE); //attach interrupt to PIN 2 
//attachInterrupt(5, readEncoderL, CHANGE); //attach interrupt to PIN 2 
  countsL = 0;
  countsR = 0;

  oldtimer = TIMSK0;
  oldadc = ADCSRA;
  oldadmux = ADMUX;
  TIMSK0 = 0;
  ADCSRA = 0xe4; 
  ADMUX = 0x40;
  DIDR0 = 0x01;
}

void loop() {
  // put youR main code heRe, to Run Repeatedly:
 int x[1024];
  // put your main code here, to run repeatedly:
  TIMSK0 = 0;
  ADCSRA = 0xe4;
  ADMUX = 0x40;
  cli();
  for (int i = 0 ; i < 1024 ; i++) {
      
    while(!(ADCSRA & 0x10));
    ADCSRA = 0xf5;
    byte m = ADCL;
    byte j = ADCH;
    int k = (j << 8) | m;
    //k -= 0x0200;
    //k <<= 6;
    x[i] = k;
  }

  sei();
    int peak[256];
    memset(peak,0,256*sizeof(int));
    int peakcount=0;
  for(int i=1; i < 1024 - 1; i++){
   if(x[i] != 0){
    //Serial.println(x[i]);
    if(x[i-1] <x[i] and x[i]> x[i+1] and x[i]>350) {
      peak[peakcount]=i;
      peakcount++;
    }
   }
  }

  int start[128];
  memset(start,0,128*sizeof(int));
  int startcount = 0;
  for(int i=1; i < 256; i++){  
    if((peak[i] - peak[i-1]) == 10){
      startcount++;
    }
  
   
  }
  TIMSK0 = oldtimer;
  ADCSRA = oldadc;
  ADMUX = oldadmux;

  digitalWrite(13, HIGH); 
  digitalWrite(3, LOW);
  digitalWrite(8, HIGH);
  digitalWrite(2, HIGH);
  Serial.println(startcount);
  if(startcount >40)
  {
    go = true;
    setMotors(175,175);
  }
  if(digitalRead(13) == LOW){
    go=true;
    setMotors(175,175);
  }
  if(go==true)digitalWrite(micLed, HIGH);
  while(go==true){
    // motors();
   //checkCandle();
   Serial.println(frontWallDistance());
  // Serial.print("   ");
   //Serial.println(rCandle());
   delay(250);
  }
  delay(1000);
  
}

void motors(){
 // isCornered();
  if(frontWallDistance()<30){
    setMotors(-180,-180);
    setMotors(-180,-120);
    delay(400);
    setMotors(120,180);
    delay(600);
    setMotors(175,175);
  }
  else
  {
    setMotors(speedL+error(),speedR-error());
  }
  checkCandle();
  reseter();
  
}


void reseter(){
  reset++;
  if(reset>31){
    reset = 0;
  }
}
      
double rightWallDistance() {
  int rvolt = analogRead(4);
  double rdist = pow(rvolt, -1.041)*3009.2;
  //Serial.println(rdist);
  return rdist;
}

double leftWallDistance() {
  int rvolt = analogRead(2);
  double ldist = pow(rvolt, -1.041)*3009.2;
  return ldist;
}

double error(){
  
  if(rightWallDistance()>20){
    return(160);
  }
  else{
    return constrain((rightWallDistance() - 12),-15,15)*5;
  }
}

double frontWallDistance() {
  int volt = analogRead(5);
  return 0.0008*pow(volt, 2) - 0.4756*volt + 94.552;
}


void turnRight(){
  setMotors(speedL+50, speedR-50);
}

void normalSpeed(){
  setMotors(speedL,speedR);  
}

void leftCandle(){
  if(reset<1){
    int lsCan = 0;
  }
  if(analogRead(8)>100){
    lsCan++;
  }
  if(lsCan>15){
    lsCan=0;
    stopMotors();
    delay(100);
    setMotors(-130,130);
    delay(250);
     setMotors(180,180);
    delay(750);
  }
}

void checkCandle(){
  leftCandle();
  int lCan = lCandle();
  int rCan = rCandle();
 // Serial.print(lCan);
 // Serial.print("   ");
 // Serial.println(rCan);
  if( lCan>=80 or rCan>=80){
    candleCount++;
  }
  if(candleCount>10){
    findCandle();
    candleCount = 0;
  }
  else if(reset>30){
    candleCount = 0;
  }
}

void findCandle(){
  int lCan = lCandle();
  int rCan = rCandle();
  int diff = lCan - rCan;
  stopMotors();
  delay(1000);
  while(lCan >=80 or rCan>=80){
    lCan = lCandle();
    rCan = rCandle();
    Serial.print(lCan);
    Serial.print("   ");
    Serial.println(rCan);
    diff = lCan - rCan;
    if(abs(diff)>10){
      if(diff>0){
        setMotors(-150,150);
      }
      else if(diff<-10){
        setMotors(150,-150);
      }
    }
    else {
      setMotors(150,150);
      delay(250);
      stopMotors();
      extinguish();
      lCan=0;
      rCan=0;
    }
  }
}

void extinguish(){
  fan(true);
  delay(2000);
  setMotors(-180,180);
  delay(300);
  setMotors(-130,130);
  delay(300);
  setMotors(-100,100);
  delay(5000);
  stopMotors();
  delay(1000);
  setMotors(180,-180);
  delay(300);
  setMotors(130,-130);
  delay(300);
  setMotors(100,-100);
  delay(2000); 
  stopMotors();
  fan(false);
  delay(5000);
  
    
}

int lCandle(){
  return analogRead(3);
}

int rCandle(){
  return analogRead(1);
}

void stopMotors(){
  setMotors(0,0);
}

bool rightBump(){
  if(digitalRead(8)==LOW)return(true);
  else return(false);
}

bool leftBump(){
  if(digitalRead(2)==LOW) return(true);
  else return(false);
}

void fan(bool x){
  if(x==true){
    digitalWrite(12, HIGH);
    digitalWrite(fanLed , HIGH);
  }
  else if (x==false){
    digitalWrite(12, LOW);
    digitalWrite(fanLed, LOW);
  }
}

void isCornered(){
  if(rightWallDistance()<8 and leftWallDistance()<8 and frontWallDistance()<30){
    setMotors(-120,-120);
    delay(250);
  }
}

void setMotors(int dLeftSpeed, int dRightSpeed) {
 int leftSpeed = dLeftSpeed;
 int rightSpeed = dRightSpeed;
 
 if(leftBump()==true){
  leftSpeed = dLeftSpeed + 50;
  rightSpeed = -dRightSpeed*.75;
 }
 else if(rightBump() == true){
  rightSpeed = dRightSpeed + 50;
  leftSpeed = -dLeftSpeed*.75;
 } 
  analogWrite(pwmR, constrain(abs(rightSpeed),-255,255));
  analogWrite(pwmL, constrain(abs(leftSpeed),-255,255));
  digitalWrite(dirR, LOW);
  digitalWrite(dirL, HIGH);
  if (leftSpeed < 0)
    digitalWrite(dirL, LOW);
  else
    digitalWrite(dirL, HIGH);
  if (rightSpeed < 0)
    digitalWrite(dirR, HIGH);
  else
    digitalWrite(dirR, LOW);
}

