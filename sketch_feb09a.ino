
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <MotorCHR.h>

#define EN 53
#define DIRR 54
#define PWMR 5
#define DIRL 52
#define PWML 3
#define C2L 4
#define C1L 14
#define C1R 7
#define C2R 2
//1920L
//1920R

float velocityLin = 0.0;
float velocityRad = 0.0;


void getVel(const geometry_msgs::Twist &cmd){
  velocityLin = cmd.linear.x;
  velocityRad = cmd.angular.z;
}

ros::NodeHandle node;
ros::Subscriber<geometry_msgs::Twist> subscriber("cmd_vel", &getVel);

int ticksR = 1920;
int ticksL = 1920;
float r = 0.04;
float L = 0.24;
long timer = 0;
MotorCHR motorR(C1R, C2R, DIRR, PWMR, ticksR, false);
MotorCHR motorL(C1L, C2L, DIRL, PWML, ticksL, true);
void getTicksR() {
  motorR.counter();
}
void getTicksL() {
  motorL.counter();
}

void setup() {
  pinMode(EN, OUTPUT);
  digitalWrite(EN, HIGH);
  pinMode(C1R, INPUT);
  pinMode(C2R, INPUT_PULLDOWN);
  pinMode(DIRR, OUTPUT);
  pinMode(PWMR, OUTPUT);
  attachInterrupt(0, getTicksR, CHANGE);
  pinMode(C1L, INPUT);
  pinMode(C2L, INPUT_PULLDOWN);
  pinMode(DIRL, OUTPUT);
  pinMode(PWML, OUTPUT);
  pinMode(13, OUTPUT);
  attachInterrupt(2, getTicksL, CHANGE);
  //Serial.setTimeout(3);
  node.initNode();
  node.subscribe(subscriber);
  motorR.Pk = 20;
  motorR.Ik = 0.03;
  motorL.Pk = 20;
  motorL.Ik = 0.03;
}

void loop() {
  /*if(Serial.available()>0){
    String data = Serial.readString();
    if(data[0] == 'p'){motorR.Pk = data.substring(1).toFloat();}
    else if(data[0] == 'i'){motorR.Ik = data.substring(1).toFloat();}
    else if(data[0] == 'd'){motorR.Dk = data.substring(1).toFloat();}
    else{velocityLin = data.toFloat();}
  }*/
 
  /*if(millis() - timer > 50){
    Serial.print(motorR.getVelocity());
    Serial.print(',');
    Serial.println(motorR.goal_velocity);
    //Serial.println(motorR.PWM_signal);
    //Serial.print(",");
    //Serial.println(motorR.getVelocity());
   // Serial.print(",");
    //Serial.println(motorR.PWM_signal);
    timer = millis();
  }*/
  motorR.setVelocity((velocityLin / r) + (velocityRad * L) / (2 * r));
  motorL.setVelocity((velocityLin / r) - (velocityRad * L) / (2 * r));
  node.spinOnce();
}
