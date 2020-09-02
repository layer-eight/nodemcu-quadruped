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

//Target positions for Servo A and Servo C about 90 degree angle, one step and about 45 degrees
int target_90_AC = 330;
int target_step_AC = 170;
int target_45_AC = 250;

//Target positions for Servo B and Servo D about 90 degree angle, one step and about 45 degrees
int target_90_BD = 310;
int target_step_BD = 470;
int target_45_BD = 390;

//void moveLeg(int servo, int hipTarget, int kneeTarget);
//void moveForward();
//void stepPinValues_Up(int legNum, int target);
//void stepPinValues_Down(int legNum, int target);
//void moveHip(int firstTarget, int secondTarget, bool isAC);


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
  delay(100);
}

void loop() {
  moveForward();
}

void moveForward(){
  
  initWalkingStance();

  //Step D to target_step_BD
  stepPinValues_Up(legD, target_step_BD);

  //Move C target_45_AC and A target_90_AC
  //B++ and D++ as long as C and A move to target position
  moveHip(target_90_AC, target_45_AC, true);

  //Step B to target_90_BD
  stepPinValues_Down(legB, target_90_BD);

  //Step A to target_step_AC
  stepPinValues_Down(legA, target_step_AC);

  //Move B target_45_BD and D target_90_BD
  //A++ and C++ as long as B and D move to target position
  moveHip(target_45_BD, target_90_BD, false);

  //Step C to target_90_AC
  stepPinValues_Up(legC, target_90_AC);
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

void initWalkingStance(){
  //Initial hip values
  hipKneeArray[legA].hip = 250;
  hipKneeArray[legB].hip = 390;
  hipKneeArray[legC].hip = 330;
  hipKneeArray[legD].hip = 310;
  
  for(int i = 0; i<4;i++){
    pwm.setPin(i,hipKneeArray[i].hip,false);
  }
}

//TODO Check the stuff that happens with the knees. find idea so the method can be used for servo B and D too
// A and B have 150 as beginning position C and D beginn at 500
//Method that implements single step for servo 0 and 2
//Method only works in one direction currently
void stepPinValues_Up(int legNum, int target){
  bool moving = true;
  
  int hip = hipKneeArray[legNum].hip;
  int knee = hipKneeArray[legNum].knee;
  int hipStart = hip;
  
  while(moving){
    if(hip < target){
      hip++;
    }
    
    if(hip <= hipStart + ((target - hipStart)/2)){
      knee -=2;
    }else{
      knee +=2;
    }
    //Serial.printf("hip: %d, knee: %d\n", hip, knee);
    moveLeg(legNum,hip,knee);
    
    if(hip == target){
      moving = false;
      hipKneeArray[legNum].hip = hip;
      hipKneeArray[legNum].knee = knee;
    }
  }
}

//TODO Check the stuff that happens with the knees. find idea so the method can be used for servo A and C too
// A and B have 150 as beginning position C and D beginn at 500
//Step movement for servo which has a lower target position than the current pin value
void stepPinValues_Down(int legNum, int target){
  bool moving = true;
  
  int hip = hipKneeArray[legNum].hip;
  int knee = hipKneeArray[legNum].knee;
  int hipStart = hip;

  while(moving){
    if(hip > target){
      hip--;
    }
    
    if(hip >= target + ((hipStart - target)/2)){
      knee +=2;
    }else{
      knee -=2;
    }
    
    moveLeg(legNum,hip,knee);

    if(hip == target){
      moving = false;
      hipKneeArray[legNum].hip = hip;
      hipKneeArray[legNum].knee = knee;
    }
  }
}

//fistTarget Value is smaller than current pin value
//secondTarget is higher than
void moveHip(int firstTarget, int secondTarget, bool isAC){
  bool moving = true;

  int hip1 = hipKneeArray[legA].hip;
  int hip2 = hipKneeArray[legC].hip;
  int hip3 = hipKneeArray[legB].hip;
  int hip4 = hipKneeArray[legD].hip; 
  if(!isAC){
    hip1 = hipKneeArray[legB].hip;
    hip2 = hipKneeArray[legD].hip;
    hip3 = hipKneeArray[legA].hip;
    hip4 = hipKneeArray[legC].hip;
  }

  while(moving){
    if(hip1 < firstTarget){
      hip1++;
      if(isAC){
        pwm.setPin(legA,hip1,false);
      }else{
        pwm.setPin(legB,hip1,false);
      }
    }
    if (hip2 > secondTarget){
      hip2--;
      if(isAC){
        pwm.setPin(legC, hip2, false);
      }else{
        pwm.setPin(legD, hip2, false);
      }
    }
    
    hip3++;
    hip4--;
    
    if(isAC){
      pwm.setPin(legB, hip3, false);
      pwm.setPin(legD, hip4, false);
    }else{
      pwm.setPin(legA, hip3, false);
      pwm.setPin(legC, hip4, false);
    }

    //Serial.printf("hip1: %d, hip2: %d, hip3: %d, hip4: %d\n", hip1, hip2, hip3, hip4);
    
    if((hip1 == firstTarget)&&( hip2 == secondTarget)){
      moving = false;

      if(isAC){
        hipKneeArray[legA].hip = hip1;
        hipKneeArray[legC].hip = hip2;
        hipKneeArray[legB].hip = hip3;
        hipKneeArray[legD].hip = hip4;
      }else{
        hipKneeArray[legB].hip = hip1;
        hipKneeArray[legD].hip = hip2;
        hipKneeArray[legA].hip = hip3;
        hipKneeArray[legC].hip = hip4;
      }
    }
  }
}

//Method that sets pin value for both knee and hip joint
void moveLeg(int servo, int hipTarget, int kneeTarget){
  pwm.setPin(servo+4, kneeTarget, false);
  pwm.setPin(servo, hipTarget, false);
}
