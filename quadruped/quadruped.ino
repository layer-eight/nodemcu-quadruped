#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

#define SERVOMIN 150
#define SERVOMAX 500
#define SERVO_FREQ 50

//Global Variables
bool firstInit = true;

enum legEnum {
  legA, legB, legC, legD
};

typedef struct hipKneeStruct{
  int hip;
  int knee;
};

hipKneeStruct hipKneeArray[4];





void setup() {
  //Initialize Serial communication
  Serial.begin(115200);
  Serial.println();
  Serial.println("Start quadruped");

  //Initialize PWM Servo Driver
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);

  //Initial hip and knee values
  hipKneeArray[legA].hip = 250;
  hipKneeArray[legA].knee = 150;
  hipKneeArray[legB].hip = 390;
  hipKneeArray[legB].knee = 150;
  hipKneeArray[legC].hip = 250;
  hipKneeArray[legC].knee = 500;
  hipKneeArray[legD].hip = 390;
  hipKneeArray[legD].knee = 500;
  
  for(int i = 0; i<4;i++){
    moveLeg(i,hipKneeArray[i].hip,hipKneeArray[i].knee);
  }
}

void loop() {

}

void moveForward(){
  //Target position for Servo A and Servo C about 90 degree angle, one step and about 45 degrees
  int target_90_AC = 330;
  int target_step_AC = 170;
  int target_45_AC = 250;
  
  //Target position for Servo B and Servo D about 90 degree angle, one step and about 45 degrees
  int target_90_BD = 310;
  int target_step_BD = 470;
  int target_45_BD = 390;

  //Step B to target_step_BD
  stepPinValues_Up(legB, target_step_BD);

  //Move A target_45_AC and C target_90_AC
  //D++ and B++ as long as A and C move to target position
  moveAll(target_45_AC, target_90_AC, false);

  //Step D to target_90_BD
  stepPinValues_Down(legD, target_90_BD);

  //Step C to target_step_AC
  stepPinValues_Down(legC, target_step_AC);

  //Move D target_45_BD and B target_90_BD
  //D++ and B++ as long as A and C move to target position
  moveAll(target_45_BD, target_90_BD, true);

  //Step A to target_90_AC
  stepPinValues_Up(legA, target_90_AC);

}

//TODO 
void moveBackward(){
  
}
//TODO
void turnRight(){
  
}
//TODO
void turnLeft(){
  
}



//TODO Check the stuff that happens with the knees. find idea so the method can be used for servo B and D too
// A and B have 150 as beginning position C and D beginn at 500
//Method that implements single step for servo 0 and 2
//Method only works in one direction currently
void stepPinValues_Up(int legNum, int target){
  bool moving = true;
  
  int* hip = &hipKneeArray[legNum].hip;
  int* knee = &hipKneeArray[legNum].knee;
  int hipStart = *hip;

  while(moving){
    if(*hip < target){
      *hip++;
    }
    
    if(*hip <= hipStart + ((target - hipStart)/2)){
      *knee +=2;
    }else{
      *knee -=2;
    }
    
    moveLeg(legNum,*hip,*knee);

    if(*hip == target){
      moving = false;
    }
  }
}



//TODO Check the stuff that happens with the knees. find idea so the method can be used for servo A and C too
// A and B have 150 as beginning position C and D beginn at 500
//Step movement for servo which has a lower target position than the current pin value
void stepPinValues_Down(int servoNum, int target){
  bool moving = true;
  
  int* hip = &hipKneeArray[legNum].hip;
  int* knee = &hipKneeArray[legNum].knee;
  int hipStart = *hip;

  while(moving){
    if(*hip > target){
      *hip--;
    }
    
    if(*hip >= target + ((hipStart - target)/2)){
      *knee -=2;
    }else{
      *knee +=2;
    }
    
    moveLeg(servoNum,*hip,*knee);

    if(*hip == target){
      moving = false;
    }
  }
}
// 
void moveAll(int firstTarget, int secondTarget, bool isBD){
  bool moving = true;

  int* hip1 = &hipA;
  int* hip2 = &hipC;
  int* hip3 = &hipB;
  int* hip4 = &hipD;
  if (isBD){
    hip1 = &hipB;
    hip2 = &hipD;
    hip3 = &hipA;
    hip4 = &hipC;
  }

  int decreaseHip1 = *hip1;
  int increaseHip1 = *hip2;
  int decreaseHip2 = *hip1;
  int increaseHip2 = *hip2;
  while(moving){

    
    
    if((decreaseHip1 == firstTarget)&&(increaseHip1 == secondTarget)){
      moving = false;
    }
  }
}

//Method that sets pin value for both knee and hip joint
void moveLeg(int servo, int hipTarget, int kneeTarget){
  pwm.setPin(servo+4, kneeTarget, false);
  pwm.setPin(servo, hipTarget, false);
}
