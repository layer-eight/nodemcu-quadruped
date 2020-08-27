#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

#define SERVOMIN 150
#define SERVOMAX 500
#define SERVO_FREQ 50

//Global Variables
bool firstInit = true;

//the current pin value for each servo motor
int hipA, kneeA, hipB, kneeB, hipC, kneeC, hipD, kneeD;


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

}

void loop() {
  pwm.setPin(4,150,false);
  pwm.setPin(5,150,false);
  pwm.setPin(6,500,false);//C
  pwm.setPin(7,500,false);//D
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
  singleStep_AB(1, target_step_BD);

  //Move A target_45_AC and C target_90_AC
  //D++ and B++ as long as A and C move to target position
  moveAll(target_45_AC, target_90_BD);

  //Step D to target_90_BD
  singleStep_CD(3, target_90_BD);

  //Step C to target_step_AC
  singleStep_CD(2, target_step_AC);

  //Move D target_45_BD and B target_90_BD
  //D++ and B++ as long as A and C move to target position
  moveAll(target_45_BD, target_90_BD);

  //Step A to target_90_AC
  singleStep_AB(0, target_90_AC);

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



//TODO Implement correct method 
//Method that implements single step for servo 0 and 2
//Method only works in one direction currently
void singleStep_AB(int servoNum, int target){
  bool moving = true;
  
  int* h = &hipA;
  int* k = &kneeA;
  if (servoNum == 1){
    h = &hipB;
    k = &kneeB;
  }

  int hip = *h;
  int knee = *k;

  while(moving){
    if(hip < target){
      hip++;
    }
    
    if(hip <= hip + ((target - hip)/2)){
      knee +=2;
    }else{
      knee -=2;
    }
    
    moveLeg(servoNum,hip,knee);

    if(hip == target){
      moving = false;
    }
  }
}

void moveLeg(int servo, int hipTarget, int kneeTarget){
  pwm.setPin(servo+4, kneeTarget, false);
  pwm.setPin(servo, hipTarget, false);
}

//TODO Implement correct method 
//Method that implements single step for servo 1 and 3
void singleStep_CD(int servoNum, int target){
  bool moving = true;
  
  int* h = &hipC;
  int* k = &kneeC;
  if (servoNum == 3){
    h = &hipD;
    k = &kneeD;
  }

  int hip = *h;
  int knee = *k;

  while(moving){
    if(hip > target){
      hip--;
    }
    
    if(hip >= target + ((hip - target)/2)){
      knee -=2;
    }else{
      knee +=2;
    }
    
    moveLeg(servoNum,hip,knee);

    if(hip == target){
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
