//Hexpod gait reinforcement learning
//dasc @ HVL

//Debugging Serial printing
#define DEBUG //Uncomment for debugging mode

#ifdef DEBUG
#define DEBUG_PRINT(x)    Serial.print (x)
#define DEBUG_PRINTLN(x)  Serial.println (x)
#define DEBUG_DELAY(x)    delay(x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#define DEBUG_DELAY(x) 
#endif


//Libraries
#include <Servo.h>
#include <Wire.h>
#include <VL53L0X.h>
//#include "QuickMedianLib.h"

//Servos
Servo frontServo;  // create servo object to control the three servos
Servo midServo;
Servo backServo;

//TOF-Sensor
VL53L0X sensor; // create VL53L0X object to sense distance to wall with ToF-Sensor

//Variable Definitions
long distance = 0;
long olddistance = 0;
unsigned long mil = 0;

//Rewards for binary reward function
int negreward = -1; //tune rewards
int posreward = 1;
int neutralreward = 0;

//Leg angles
// left and right, back and forward seen from the robots perspective
int frontang = 100; //center angle front legs servo; higher value is more back on right side
int backang = 95; //center angle back legs servo; higher value is more back on right side
int midang = 70; //center angle mid legs servo
int frontleftback = -7; //-11
int frontrightback = 7; //11
int midleftdown = -10; //-20 with battery  weight
int midrightdown = 10; //15 with battery weight
int backleftback = -7; //-11
int backrightback = 7; //11

//Variables for Q-Learning
int A[8][3] = {{frontleftback, midleftdown, backleftback}, //Actions. All actions possible in all states
  {frontrightback, midleftdown, backleftback},
  {frontleftback, midrightdown, backleftback},
  {frontleftback, midleftdown, backrightback},
  {frontrightback, midrightdown, backleftback},
  {frontrightback, midleftdown, backrightback},
  {frontleftback, midrightdown, backrightback},
  {frontrightback, midrightdown, backrightback}
}; 
int i, j = 0;
int state, oldstate;
int action;
int max_q_index = 0;
bool noUpdate = false; //Flag for not updating the q-Value of the current iteration due to unrealistic sensor reading
float delta_q = 0;
float max_q = 0;
float reward = 0;
//Learn Parameter
float e_greed = 50;//93; // factor for  e-greedy action selection in range from 0-99; balance between exploration & exploitation
float e_greed_final = 93; //98
float e_greed_step = 0.075; //value with which the e_greed factor is increased in each iteration. This way exploring at the beginning and exploitation at the end can be achieved.
float alpha = 1.0; //0.85 // Learn rate: Low-pass filter for changes of the entries of the q-matrix in presence of q-value update
float alpha_final = 0.1;
float alpha_step = 0.0005;
float gamma = 0.7; // 0.7 //discount factor of Q-value of next state
bool crawldir = 0; //0 = forward; 1 = backward;
float Q[8][8] = {{0, 0, 0, 0, 0, 0, 0, 0}, //Q-Values[states][actions]
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0}
};

//  float Q[8][8] = {{0, 0, 0, 0, 0, 100, 0, 0}, // Biased q values for right gait
//                   {0, 0, 0, 0, 0, 0, 0, 0},
//                   {100, 0, 0, 0, 0, 0, 0, 0},
//                   {0, 0, 0, 0, 0, 0, 0, 0},
//                   {0, 0, 0, 0, 0, 0, 0, 0},
//                   {0, 0, 0, 0, 0, 0, 0, 100},
//                   {0, 0, 0, 0, 0, 0, 0, 0},
//                   {0, 0, 100, 0, 0, 0, 0, 0}};


//**********SETUP***********
//**************************
void setup() {

  //Hardware setup
  //Servos
  frontServo.attach(9);  // attaches the servo on pin 9 to the servo object
  midServo.attach(10);  // attaches the servo on pin 9 to the servo object
  backServo.attach(11);  // attaches the servo on pin 9 to the servo object

  //ToF-Sensor
  Wire.begin(); //establish I2C connection to the sensor
  sensor.init(); //initialize sensro
  sensor.setTimeout(500); //500
  sensor.setMeasurementTimingBudget(200000); // increase timing budget to 200 ms
  olddistance = sensor.readRangeSingleMillimeters(); //sense initial distance

  //Serial Monitor
  Serial.begin(115200); // initialize serial interface

  //Q-Learning Setup
  randomSeed(olddistance); //Use the intitial distance measurement to select a random randomSeed
  state = random(0, 8); //random initial state
  frontServo.write(frontang + A[state][0]); // Move servos into state
  midServo.write(midang + A[state][1]);
  backServo.write(backang + A[state][2]);
  delay(2000);
}


//**********MAIN LOOP*******
//**************************
void loop() {
  DEBUG_PRINT("New Iteration, Nr.: ");
  DEBUG_PRINTLN(j);
  DEBUG_PRINT("Current state: ");
  DEBUG_PRINTLN(state);

  //Pick an action by e-greedy policy
  action = e_greedy(e_greed, e_greed_final, e_greed_step, Q, state);

  //Execute the chosen action:
  frontServo.write(frontang + A[action][0]);
  midServo.write(midang + A[action][1]);  // Move servos into state
  backServo.write(backang + A[action][2]);
  delay(150); //15
  //DEBUG_DELAY(1000);

  // Acquire reward signal
  //TOF Sensor
  //mil = millis();
  distance = sensor.readRangeSingleMillimeters();
  //mil = millis() - mil;
  //DEBUG_PRINTLN(mil);
  if (sensor.timeoutOccurred()) { Serial.println(" TIMEOUT"); }
  //delay(300);
  
    DEBUG_PRINTLN(olddistance);
    DEBUG_PRINTLN(distance);
    DEBUG_PRINTLN(olddistance - distance);
  if (abs(olddistance - distance) < 40)
  {
    reward = olddistance - distance;
    //reward = reward / 70;
    //reward = reward * abs(reward);
    //reward = reward * reward * reward;
    //reward = pow(0.35*reward-0.4, 3);
    reward = pow(((0.6 * reward) - 1.1), 3) + 0.2 * reward; //Best so far; 650 @ 16
    //reward = 0.2 * pow(((0.89 * reward) - 1.8), 3) + 0.05 * reward; //okay but not necessarily better than above
    //reward = pow(((3*reward)-1.1), 3) + reward;
    //reward = pow(3.2, reward-4.5);
    //////reward = 1 - (distance/1000)^0.4;
    //reward = -85+ pow(1.04, (0.95 *((olddistance-distance)+110)));// 1.04^(0.95 *((olddistance-distance)+110));
    //  if ((olddistance - distance) > 20) //(29 microseconds per centimeter)/2 => cm=(ms/29)/2
    //  {
    //    reward = posreward;
    //  }
    //  else {
    //    reward = negreward;
    //  }
    noUpdate = false;
  }
  else {
    noUpdate = true;
    DEBUG_PRINTLN("NO UPDATE");
  }
  reward = reward / 10;
  if (crawldir){reward = reward * (-1);}
  DEBUG_PRINT("Reward received: ");
  DEBUG_PRINTLN(reward);

  olddistance = distance;  
  oldstate = state; //Observe resulting next state -> deterministic model -> pick from table according to chosen action
  state = action;
  //compute Q update
  if (noUpdate == false)
  {
    max_q = Q[state][0];
    for (i = 1; i < 8; i++) { //compute possible max q value of new state
      if (Q[state][i] > max_q) {
        max_q = Q[state][i];
      }
    }
      if (alpha >= alpha_final + alpha_step) //Increase e_greedy value ofer time to exploit more after some learning
      {
      alpha = alpha - alpha_step;
      }
      else {alpha = alpha_final;}
    DEBUG_PRINT("Alpha: ");
    DEBUG_PRINTLN(alpha);
    delta_q = reward + (gamma * max_q) - Q[oldstate][action];
    DEBUG_PRINT("max-Q: ");
    DEBUG_PRINTLN(max_q);
    DEBUG_PRINT("max-Q * gamma: ");
    DEBUG_PRINTLN(gamma * max_q);
    DEBUG_PRINT("Delta-Q: ");
    DEBUG_PRINTLN(delta_q);
    DEBUG_PRINT("Old Q: ");
    DEBUG_PRINTLN(Q[oldstate][action]);
    Q[oldstate][action] = Q[oldstate][action] + alpha * delta_q;
    DEBUG_PRINT("New Q: ");
    DEBUG_PRINTLN(Q[oldstate][action]);
  }
  else {
    //Serial.println("NO Q-VALUE UPDATE");
    delay(2500);
    olddistance = sensor.readRangeSingleMillimeters();
  }

  //Print Q-matrix
  for (i = 0; i < 8; i++)
  {
    DEBUG_PRINT(Q[i][0]);
    DEBUG_PRINT(", ");
    DEBUG_PRINT(Q[i][1]);
    DEBUG_PRINT(", ");
    DEBUG_PRINT(Q[i][2]);
    DEBUG_PRINT(", ");
    DEBUG_PRINT(Q[i][3]);
    DEBUG_PRINT(", ");
    DEBUG_PRINT(Q[i][4]);
    DEBUG_PRINT(", ");
    DEBUG_PRINT(Q[i][5]);
    DEBUG_PRINT(", ");
    DEBUG_PRINT(Q[i][6]);
    DEBUG_PRINT(", ");
    DEBUG_PRINTLN(Q[i][7]);
  }
  DEBUG_PRINTLN();
  //DEBUG_DELAY(8000);

  j++; //Iteration counter to monitor learning progress
}


/*************************************
  /**********FUNCTION DECLARATIONS******
  /************************************/

//long usdist(int pingPin, int start_signal)
//{
//  long temppingduration, pingduration = 0;
//  int i;
//
//  for (i=1; i<2; i++)
//    {
//      pinMode(pingPin,OUTPUT);
//      pinMode(start_signal,OUTPUT);
//      digitalWrite(start_signal,HIGH);
//      delayMicroseconds(20);
//      digitalWrite(start_signal,LOW);
//      digitalWrite(pingPin,LOW);
//      delayMicroseconds(2);
//      digitalWrite(pingPin,HIGH);
//      delayMicroseconds(5);
//      digitalWrite(pingPin,LOW);
//      pinMode(pingPin,INPUT);
//      temppingduration =pulseIn(pingPin,HIGH);
//      //Serial.println(temppingduration);
//      pingduration = pingduration + temppingduration;
//    }
//  pingduration = pingduration / (i-1);
//  return pingduration;
//}


//float lp_rate_filter(float x, float x_max, float x_min, float x_max_abs_delta_per_dt,
//                    float dt, float RC, float y, float x_delta){
//
//    float filtered = y;
//    float alpha = dt/(RC + dt);
//    float x_dt = x_delta/dt;
//    float x_abs_dt = x_dt;
//    if (x_abs_dt < 0){x_abs_dt = -1*x_abs_dt;}
//    if (x < x_max && x > x_min && x_abs_dt < x_max_abs_delta_per_dt){
//        filtered = alpha*x + (1-alpha)*y;
//    }
//
//    return filtered;
//}
