/*
This code is for controlling a wheelchair using a joystick and smartphone.
Obstacle detection and heart rate monitoring are additional features in this wheelchair.
*/

#include <NewPing.h> //for ultrasonic sensor
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"

//heart rate sensor parameters
MAX30105 particleSensor;

const int deadzone = 90;
const int statX = 510;
const int statY = 515;

const int frontdis_us=60; //accepted distance between the front us sensor and the obstacle
const int backdis_us=60; //accepted distance between the back us sensor and the obstacle
const int sidedis_us=40; //accepted distance between the side us sensors and the obstacle

//ultrasonic sensors params
#define SONAR_NUM   5 // Number of sensors.
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.
#define PING_INTERVAL 35 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).


//pins for the X and Y axes of the joystick (analog)
int joyX = 7;
int joyY = 6;
int sw2=5;

//motor drivers
int leftpwm_leftmotor=6; // Arduino PWM output pin 4; connect to IBT-2 LPWM left motor
int rightpwm_leftmotor=7; // Arduino PWM output pin 13; connect to IBT-2 RPWM left motor
int leftpwm_rightmotor=11; // Arduino PWM output pin 9; connect to IBT-2 LPWM right motor
int rightpwm_rightmotor=12;  // Arduino PWM output pin 10; connect to IBT-2 RPWM right motor

//buttons
int changemode_button=22; //button for changing the mode of operation of the wheelchair (input); connect to pin 12
int buzzer_button=9; //button for activating the buzzer; connect to pin 12
int inc_spd_sig=23; //button to increase the speed of the wheelchair
int dec_spd_sig=24; //button to decrease the speed of the wheechair
int stop_us=5; //button to activate and deactivate the ultrasonic sensor

int buzzer_output = 10; //the buzzer should be connected to pin 11
int us_state=1;

//LEDs
int led_joy=3; //LED for showing that joystick mode is activated; use pin 8
int led_voice=8; //LED for showing that voice command mode is activated; use pin 7
int led_phone=4; //LED to show that phone command mode is activated; use pin 1
//speed level leds
int sp1_sig=25;
int sp2_sig=26;
int sp3_sig=27;

//these are the pins for the ultrasonic sensors that will be used (front, back, left and right)
int SensorTrig_front1=38;
int SensorEcho_front1=39;
int SensorTrig_front2=40; //The front facing ultrasonic sensor trig pin should be connected to pin 2 of the Arduino
int SensorEcho_front2=41; //The front facing ultrasonic sensor echo pin should be connected to pin 3 of the Arduino
int SensorTrig_back=44; //The back facing ultrasonic sensor trig pin should be connected to pin 4 of the Arduino
int SensorEcho_back=45; //The back facing ultrasonic sensor echo pin should be connected to pin 14 of the Arduino
int SensorTrig_left=46; //The left facing ultrasonic sensor trig pin should be connected to pin 15 of the Arduino
int SensorEcho_left=47; //The left facing ultrasonic sensor echo pin should be connected to pin 16 of the Arduino
int SensorTrig_right=42; //The right facing ultrasonic sensor trig pin should be connected to pin 17 of the Arduino
int SensorEcho_right=43; //The right facin g ultrasonic sensor echo pin should be connected to pin 18 of the Arduino

//Ultrasonic sensors Setups
NewPing ultrasonic_sensors[SONAR_NUM]={
  NewPing(SensorTrig_front1,SensorEcho_front1, MAX_DISTANCE), //ultrasonic sensor for the front
  NewPing(SensorTrig_front2,SensorEcho_front2, MAX_DISTANCE), //ultrasonic sensor for the front
  NewPing(SensorTrig_back,SensorEcho_back, MAX_DISTANCE), //ultrasonic sensor for the back
  NewPing(SensorTrig_left, SensorEcho_left, MAX_DISTANCE), //ultrasonic sensor for the left
  NewPing(SensorTrig_right,SensorEcho_right, MAX_DISTANCE) //ultrasonic sensor for the right
};

const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred
float beatsPerMinute;
int beatAvg;

//speed levels
int speed_level[3]={40,70,100};
int current_speed=1; //used for iterating over the speed level array

int static_variable=500;
int lastRed = 0;

int mode=2; //this shows which mode of control will be used for the wheelchair; the default state is voice control
int lastmodebutton_state=HIGH;
int lastinc_button_state=HIGH;
int lastdec_button_state=HIGH;
int lastus_button_state=HIGH;

char Rxbyte='0'; //received byte from Python

unsigned long pingTimer[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor

/*
0 element: front
1 element: back
2 element: left
3 element: right
*/
int freezone[5]={1,1,1,1,1};         // Keeps the value that allows movement in all directions. 1=free to move, 0=don't move

unsigned int cm[SONAR_NUM];         // Where the ping distances are stored.
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.


void setup() {
  TCCR4B = TCCR4B & B11111000 | B00000010;  // for PWM frequency of 3921.16 Hz (6/7)
  TCCR1B = TCCR1B & B11111000 | B00000010;  // for PWM frequency of 3921.16 Hz (11/12)
  
  pinMode(changemode_button, INPUT_PULLUP);
  pinMode(stop_us, INPUT_PULLUP);
  
  pinMode(buzzer_button, INPUT_PULLUP);
  pinMode(buzzer_output, OUTPUT);

  pinMode(inc_spd_sig, INPUT_PULLUP);
  pinMode(dec_spd_sig, INPUT_PULLUP);
  
  pinMode(leftpwm_leftmotor, OUTPUT);
  pinMode(rightpwm_leftmotor, OUTPUT);

  pinMode(leftpwm_rightmotor, OUTPUT);
  pinMode(rightpwm_rightmotor, OUTPUT);

  pinMode(led_joy, OUTPUT);
  pinMode(led_voice, OUTPUT);
  pinMode(led_phone, OUTPUT);
  pinMode(sp1_sig, OUTPUT);
  pinMode(sp2_sig, OUTPUT);
  pinMode(sp3_sig, OUTPUT);

  digitalWrite(leftpwm_leftmotor, LOW);
  digitalWrite(rightpwm_leftmotor, LOW);
  digitalWrite(leftpwm_rightmotor, LOW);
  digitalWrite(rightpwm_rightmotor, LOW);  
  
  Serial.begin(9600);

  //bluetooth serial connection
  Serial2.begin(9600);

  //heart rate
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }
  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED
  
  pingTimer[0] = millis() + 75;           // First ping starts at 75ms, gives time for the Arduino to chill before starting.
  for (uint8_t i = 1; i < SONAR_NUM; i++) // Set the starting time for each sensor.
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
}

/*mode 0: voice control
mode 1: joystick control
mode 2: smartphone control
*/

void loop() {
  long int time=millis();
   for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through all the sensors.
    if (millis() >= pingTimer[i]) {         // Is it this sensor's time to ping?
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;  // Set next time this sensor will be pinged.
      if (i == 0 && currentSensor == SONAR_NUM - 1) oneSensorCycle(); // Sensor ping cycle complete, do something with the results.
      ultrasonic_sensors[currentSensor].timer_stop();          // Make sure previous timer is canceled before starting a new ping (insurance).
      currentSensor = i;                          // Sensor being accessed.
      cm[currentSensor] = 0;                      // Make distance zero in case there's no ping echo for this sensor.
      ultrasonic_sensors[currentSensor].ping_timer(echoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
    }
  }
  button_actions();
  //heart_rate();
  switch(mode){
  //voice control mode
    case 0:    
    {
      // Serial.println("Mode 0");
      pyVoice();      
      break;
    }  
  //joystick mode
  case 1:
  {
    //Serial.println("Mode 2");
    joystickControl();
    break;
  }
  //smartphone control    
    case 2: {
      //Serial.println("Mode 3");
      smartphone_control();
      break;      
    }
  }
} 

//function to control the left and right motor
void motor_control(int LeftPWM, int RightPWM)
{
  if (LeftPWM>0 && RightPWM>0){
    if (freezone[0]==1 && freezone[1]==1)
    {
      //move forwards
      int newspeed=map(LeftPWM, 0, 255, 0, speed_level[current_speed]);  
      //newspeed=30;    
        Serial.println("Forwards ");
      Serial.println(newspeed);
      analogWrite(leftpwm_leftmotor,0);
      //-5
      //+2
      analogWrite(leftpwm_rightmotor,0);
      analogWrite(rightpwm_leftmotor, newspeed-6);
      analogWrite(rightpwm_rightmotor, newspeed+1);                  
    }
    else
    {
      //stop the motor
      analogWrite(leftpwm_leftmotor, 0);
      analogWrite(leftpwm_rightmotor, 0);
      analogWrite(rightpwm_leftmotor, 0);
      analogWrite(rightpwm_rightmotor, 0);
      Serial.println("Stop");
    }
    
  }    
  else if (LeftPWM>0 && RightPWM==0){
    if (freezone[4]==1)
    {
      //move right
      int newspeed=map(LeftPWM, 0, 255, 0, speed_level[current_speed]);
      Serial.println("Going Right ");
      Serial.println(newspeed);
      analogWrite(leftpwm_leftmotor, 0);
      analogWrite(leftpwm_rightmotor, 0);
      analogWrite(rightpwm_leftmotor, newspeed);
      analogWrite(rightpwm_rightmotor, 10);
    }
    else
    {
      //stop the motor
      Serial.println("Stop");
      analogWrite(leftpwm_leftmotor, 0);
      analogWrite(leftpwm_rightmotor, 0);
      analogWrite(rightpwm_leftmotor, 0);
      analogWrite(rightpwm_rightmotor, 0);
    }
  }
  else if(LeftPWM==0 && RightPWM>0){
    Serial.println("Going Left");
    if (freezone[3]==1)
    {
      //move left
      int newspeed=map(RightPWM, 0, 255, 0, speed_level[current_speed]);
       Serial.print("Going Left ");
      Serial.println(newspeed);
      analogWrite(leftpwm_leftmotor, 0);
      analogWrite(leftpwm_rightmotor, 0);
      analogWrite(rightpwm_leftmotor, 10);
      analogWrite(rightpwm_rightmotor, newspeed);
    }
    else
    {
      //stop the motor
      Serial.println("Stop");
      analogWrite(leftpwm_leftmotor, 0);
      analogWrite(leftpwm_rightmotor, 0);
      analogWrite(rightpwm_leftmotor, 0);
      analogWrite(rightpwm_rightmotor, 0);
    }
  }
  else if (LeftPWM<0 && RightPWM<0){
    //Serial.println("Reverse");
    if (freezone[2]==1)
    {
      //going reverse
      int newspeed=-1*(LeftPWM);
      //newspeed=map(newspeed, 0, 255, 0, SPEED);
      newspeed=40;
      Serial.print("Reverse ");
      Serial.println(newspeed);
      analogWrite(leftpwm_leftmotor, newspeed+7);
      //-4
      analogWrite(leftpwm_rightmotor, newspeed);
      analogWrite(rightpwm_leftmotor, 0);
      analogWrite(rightpwm_rightmotor, 0);      
    }
    else
    {
      //stop the motor
      Serial.println("Stop");
      analogWrite(leftpwm_leftmotor, 0);
      analogWrite(leftpwm_rightmotor, 0);
      analogWrite(rightpwm_leftmotor, 0);
      analogWrite(rightpwm_rightmotor, 0);
    }
  }
  else{
    Serial.println("Stop"); 
    analogWrite(leftpwm_leftmotor, 0);
    analogWrite(leftpwm_rightmotor, 0);
    analogWrite(rightpwm_leftmotor, 0);
    analogWrite(rightpwm_rightmotor, 0);
  }
}

void mode_lights() 
{
  //this function controls the lights for showing the current mode of control of the wheelchair
  if (mode==0){
    digitalWrite(led_voice, HIGH);
    digitalWrite(led_joy, LOW);
    digitalWrite(led_phone, LOW);    
  }
  else if  (mode==1){
    digitalWrite(led_voice, LOW);
    digitalWrite(led_joy, HIGH);
    digitalWrite(led_phone, LOW);
  }

  else{
    digitalWrite(led_voice, LOW);
    digitalWrite(led_joy, LOW);
    digitalWrite(led_phone, HIGH);    
  }

}

void speed_level_lights(){
  if (current_speed==0){ 
    digitalWrite(sp1_sig, HIGH);
    digitalWrite(sp2_sig, LOW);
    digitalWrite(sp3_sig, LOW);    
  }
  else if  (current_speed==1){
    digitalWrite(sp1_sig, LOW);
    digitalWrite(sp2_sig, HIGH);
    digitalWrite(sp3_sig, LOW);
  }

  else{
    digitalWrite(sp1_sig, LOW);
    digitalWrite(sp2_sig, LOW);
    digitalWrite(sp3_sig, HIGH);    
  }

}

//this function handles all the button requests 
void button_actions(){
  //mode change
  int modebutton_reading=digitalRead(changemode_button);
  if (lastmodebutton_state!=modebutton_reading && modebutton_reading==LOW){
    ++mode;
    if (mode==2)
    {
      Serial2.end();
      Serial2.begin(9600);     
    }
    if (mode==3){
      mode=0;
    }
  }
  lastmodebutton_state=modebutton_reading;

  //inc speed level button
  int inc_btn_reading=digitalRead(inc_spd_sig);
  if (inc_btn_reading!=lastinc_button_state && inc_btn_reading==LOW){
    ++current_speed;
    if (current_speed==3){
      current_speed=2;
    }
  }
  lastinc_button_state=inc_btn_reading;

  //dec speed level button
  int dec_btn_reading=digitalRead(dec_spd_sig);
  if (dec_btn_reading!=lastdec_button_state && dec_btn_reading==LOW){
    --current_speed;
    if (current_speed==-1){
      current_speed=0;
    }
  }
  lastdec_button_state=dec_btn_reading;  

  //activate or deactivate the ultrasonic sensors
  int act_ultrasonic=digitalRead(stop_us);
  if (act_ultrasonic!=lastus_button_state && act_ultrasonic==LOW){
    Serial.println("Yes, I am reading it");
    us_state=!us_state;
    Serial.println(us_state);
    if (us_state==0){
      for (uint8_t i = 0; i < SONAR_NUM; i++) pingTimer[i] = -1;    //stop ping for ultrasonic sensors
      for (int i=0; i<SONAR_NUM; i++) freezone[i]=1;
      Serial.println("It is free to move");
    }
    
    else {
    //starts pinging of ultrasonic sensors
      pingTimer[0] = millis();
      for (uint8_t i = 1; i < SONAR_NUM; i++) pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
    }
  }
  lastus_button_state=act_ultrasonic;   


  mode_lights();
  speed_level_lights();
  //activates the buzzer when the button is pressed
  int buzzer_state=!(digitalRead(buzzer_button));
  digitalWrite(buzzer_output, buzzer_state);  
}

 void heart_rate(){
  long irValue = particleSensor.getIR();
  if (checkForBeat(irValue) == true) // checkForBeat is a function of heartRate.h Library
  {
    //We sensed a beat!
    long delta = millis() - lastBeat;
    lastBeat = millis();
    beatsPerMinute = 60 / (delta / 1000.0);
    if (beatsPerMinute < 255 && beatsPerMinute > 20)
    {
      rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
      rateSpot %= RATE_SIZE; //Wrap variable
      //Take average of readings
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }
  if (irValue>= 50000){
    Serial2.println("BPM="+String(beatAvg));
  }
  else{
    Serial2.print("No finger?");
    Serial2.println();
  }
}

//function to receive input from the joystick
void joystickControl(){
  int Xval=analogRead(joyX);
  int Yval=analogRead(joyY);
  int motorspeed=0;
  
 
  //stop the motor (deadzone considered)
  if (Xval>(statX-deadzone) && Xval<(statX+deadzone) && Yval>(statY-deadzone) && Yval<(statY+deadzone)){
    motor_control(motorspeed,motorspeed);  //stop left and right motor  
  }

  else{
    //forward and reverse condition
    if (Yval>(statY-deadzone) && Yval<(statY+deadzone)){
      
      //forward condition
      if (Xval<=(statX-deadzone)){
        motorspeed=map(Xval, (statX-deadzone), 0, 0, 255);
        //Serial.println("Forwards");         
        motorspeed=(motorspeed);
        motor_control(motorspeed, motorspeed);
      }
      
      //backwards condition
      if (Xval>=(statX+deadzone))
      {
        motorspeed=map(Xval, (statX+deadzone), 1023, 0, 255);
        //Serial.println("Backwards");
        motorspeed=-(motorspeed);        
        motor_control(motorspeed, motorspeed);    
      }
    }
    
    //left and right condition
    else if (Xval>(statX-deadzone) && Xval<(statX+deadzone)){
      //left condition
      if (Yval<=(statY-deadzone)){
        motorspeed=map(Yval, (statY-deadzone),0, 0, 255);
        //Serial.println("Left"); 
        motor_control(0, motorspeed);
      }

      //right condition
      if (Yval>=(statY+deadzone)){
        motorspeed=map(Yval, (statY+deadzone), 1023, 0, 255);
        //Serial.println("Right");         
        motor_control(motorspeed, 0);        
      }      
    }

    else{
      Serial.println("Stop"); 
      motor_control(motorspeed,motorspeed);  //stop left and right motor  
    }
    
  }
}

void pyVoice(){
  if (Serial.available()){
    Rxbyte=Serial.readString().toInt();
    switch(Rxbyte){
      //left
      case 1:
      {
        motor_control(0,255);
        break;        
      }
      //right
      case 2:
      {
        motor_control(255,0);
        break;
      }
      //forward
      case 3:
      {
        motor_control(255, 255);
        break;
      }
      //backwards
      case 4:
      {
        motor_control(-255, -255);
        break;
      }
      //stop
      case 5:
      {
        motor_control(0,0);
        break;
      }

      default:
        motor_control(0,0);
        break;  
    }  
  }
}

void smartphone_control(){
  if (Serial2.available()){
    Rxbyte=Serial2.read();
    Serial.println(Rxbyte);
  }
  switch(Rxbyte) {
    //left
    case '1':
    {
      motor_control(0,255);
      break;        
    }
    //right
    case '2':
    {
      motor_control(255,0);
      break;
    }
    //forward
    case '3':
    {
      motor_control(255, 255);
      break;
    }
    //backwards
    case '4':
    {
      motor_control(-255, -255);
      break;
    }
    //stop
    case '5':
    {
      motor_control(0,0);
      break;
    }

    default:
      motor_control(0,0);
      break;  
  }
}

void echoCheck() {
  if (ultrasonic_sensors[currentSensor].check_timer())
    cm[currentSensor] = ultrasonic_sensors[currentSensor].ping_result / US_ROUNDTRIP_CM;
}

void oneSensorCycle() {
  // The following code would be replaced with your code that does something with the ping result.
  //front front us
   for (uint8_t i = 0; i < SONAR_NUM; i++) {
     Serial.print(i);
    Serial.print("=");
    Serial.print(cm[i]);
    Serial.print("cm ");
  }
  Serial.println();

  for (int i=0; i<=SONAR_NUM; i++)
  {
    if (i==0 || i==1)
    {
    if (cm[i]>frontdis_us || cm[i]==0)
    {
      freezone[i]=1;
    }
    else
    freezone[i]=0;
    }
    else if (i==2)
    {
      if(cm[i]>backdis_us || cm[i]==0)
      {
        freezone[i]=1;
      }
      else
      freezone[i]=0;
    }
    else
    {
      if (cm[i]>sidedis_us || cm[i]==0)
      {
        freezone[i]=1;
      }
      else
      freezone[i]=0;
    }
  }
  }
