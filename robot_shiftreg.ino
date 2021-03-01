
#include <ros.h>

#define enB 10
#define enA 11
#define RenA 3
#define RenB 9
#define FRB A5
#define FRA A4
#define BRB A3
#define BRA A2
#define FLA A1
#define FLB A0
#define BLB 4
#define BLA 2

int latchpin = 5;  // Latch pin of 74HC595 is connected to Digital pin 5
int clockpin = 6; // Clock pin of 74HC595 is connected to Digital pin 6
int datapin = 7;  // Data pin of 74HC595 is connected to Digital pin 4

int FRALastState = 0;
int FRBLastState = 0;
int BRALastState = 0;
int BRBLastState = 0;
int FLALastState = 0;
int FLBLastState = 0;
int BLALastState = 0;
int BLBLastState = 0;
int FRAState = 0;
int FRBState = 0;
int BRAState = 0;
int BRBState = 0;
int FLAState = 0;
int FLBState = 0;
int BLAState = 0;
int BLBState = 0;
int counter=0;

byte data = 0;  

void setup(){
  Serial.begin(9600);
  pinMode(FRB, INPUT);
  pinMode(FRA, INPUT);
  pinMode(BRB, INPUT);
  pinMode(BRA, INPUT);
  pinMode(FLB, INPUT);
  pinMode(FLA, INPUT);
  pinMode(BLB, INPUT);
  pinMode(BLA, INPUT);
  pinMode(enA, OUTPUT);
  pinMode(RenA, OUTPUT);
  pinMode(RenB, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(latchpin, OUTPUT);
  pinMode(datapin, OUTPUT);  
  pinMode(clockpin, OUTPUT); 
   
  int power=100;
  analogWrite(enA, power);
  analogWrite(enB, power);
  analogWrite(RenA, power);
  analogWrite(RenB, power);
  FRALastState = digitalRead(FRA);
  FRBLastState = digitalRead(FRB);
  BRALastState = digitalRead(BRA);
  BRBLastState = digitalRead(BRB);
  FLALastState = digitalRead(FLA);
  FLBLastState = digitalRead(FLB);
  BLALastState = digitalRead(BLA);
  BLBLastState = digitalRead(BLB);
  Serial.println(FRALastState); 
  Serial.print("Set up complete");
}


      


void shiftWrite(int desiredPin, boolean desiredState){
//    "desiredPin" is the shift register output pin
//    you want to affect (0-7)

  bitWrite(data,desiredPin,desiredState);
  shiftOut(datapin, clockpin, MSBFIRST, data);
  digitalWrite(latchpin, LOW);
  digitalWrite(latchpin, HIGH);
}

void Encoder(char A,char B){
  FRAState = digitalRead(A);
  FRBState = digitalRead(B);
  
  if (FRAState != FRALastState){
    if (FRBState !=FRAState){
      counter ++;
      }
    else{
      counter --;
    }
    String pos=String(A);
    Serial.print(pos);
    Serial.print("position: ");
    Serial.println(counter);
    FRALastState = FRAState;
      
}}
void loop(){
  //Encoder(FLA,FLB);
//  Encoder(BLA,BLB);
  //Encoder(FRA,FRB);
  //Encoder(BRA,BRB);
  
  delay(500);
  
  stop_all();
  delay(500);
  fr_back();
  }
void test(){
  //test each wheel forward and backward
  fl_forward();
  delay(500);
  fl_back();
  delay(500);
  stop_all();
  bl_forward();
  delay(500);
  bl_back();
  delay(500);
  stop_all();
  fr_forward();
  delay(500);
  fr_back();
  delay(500);
  stop_all();
  br_forward();
  delay(500);
  br_back();
  delay(600);
  stop_all();
  
}
void demo(){
  delay(1000);
  backward(5000);
  stop_all();
  delay(1000);
  forward(500);
  stop_all();
  delay(1000);
  turn_left(1000);
  stop_all();
  turn_right(1000);
  stop_all();
  slide_right(2000);
  stop_all();
  slide_left(2000);
  stop_all();
  diag_right_f(1000);
  stop_all();
  diag_right_b(1000);
  stop_all();
  diag_left_f(1000);
  stop_all();
  diag_left_b(1000);
  stop_all();
  delay(10000);
}
void backward(int wait){      
      fl_back();
      fr_back();
      bl_back();
      br_back();
      delay(wait);

} 
void stop_all(){
      fl_stop();
      fr_stop();
      bl_stop();
      br_stop();
}
void forward(int wait){
      fl_forward();
      fr_forward();
      bl_forward();
      br_forward();
      delay(wait);
}
void turn_left(int wait){
  fl_forward();
  bl_forward();
  fr_back();
  br_back();
  delay(wait);
    
}
void turn_right(int wait){
  fr_forward();
  br_forward();
  fl_back();
  bl_back();
  delay(wait);
}
void slide_right(int wait){
  fl_forward();
  br_forward();
  fr_back();
  bl_back();
  delay(wait);
}
void slide_left(int wait){
  fr_forward();
  bl_forward();
  fl_back();
  br_back();
  delay(wait);
}

void diag_right_f(int wait){
  fl_forward();
  br_forward();
  delay(wait);
}
void diag_right_b(int wait){
  fl_back();
  br_back();
  delay(wait);
}

void diag_left_f(int wait){
  fr_forward();
  bl_forward();
  delay(wait);
}

void diag_left_b(int wait){
  fr_back();
  bl_back();
  delay(wait);
}

void fl_forward(){
      shiftWrite(3, HIGH);
      shiftWrite(4, LOW);
}

void fl_stop(){
      shiftWrite(3, LOW);
      shiftWrite(4, LOW);
}
void fl_back(){
      shiftWrite(3, LOW);
      shiftWrite(4, HIGH);
}


void br_back(){
     //works
      shiftWrite(1, HIGH);
      shiftWrite(2, LOW);
}
void br_forward(){ 
      shiftWrite(1, LOW);
      shiftWrite(2, HIGH);
}

void br_stop(){
      shiftWrite(2, LOW);
      shiftWrite(1, LOW);
      
}

void fr_back(){
      shiftWrite(5, LOW);
      shiftWrite(6, HIGH);
}
void fr_forward(){
      shiftWrite(5, HIGH);
      shiftWrite(6, LOW);
}
void fr_stop(){
      shiftWrite(5, LOW);
      shiftWrite(6, LOW);
}
void bl_back(){
      shiftWrite(7, LOW);
      shiftWrite(0, HIGH);
}

void bl_forward(){
      shiftWrite(7, HIGH);
      shiftWrite(0, LOW);
}
void bl_stop(){
      shiftWrite(7, LOW);
      shiftWrite(0, LOW);
}
