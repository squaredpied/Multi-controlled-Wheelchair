#include <SoftwareSerial.h>
//#include "VoiceRecognitionV3.h"
#include "VoiceRecognitionV4.h" //voice recognition version 4
#include <NewPing.h> //for ultrasonic sensor

//pins for the X and Y axes of the joystick
#define joyX A0
#define joyY A1
#define deadzone 50
#define statX 518
#define statY 515

//speed of the wheelchair (PWM that will be used)
#define SPEED 80

//ultrasonic sensors params
#define SONAR_NUM      4 // Number of sensors.
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.
#define PING_INTERVAL 33 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).


int static_variable=500;


int changemode_button=22; //button for changing the mode of operation of the wheelchair (input); connect to pin 12
int buzzer_button=13; //button for activating the buzzer; connect to pin 13
int buzzer_output = 11; //the buzzer should be connected to pin 11


int LPWM_Output_L = 5; // Arduino PWM output pin 5; connect to IBT-2 pin 1 (RPWM)
int RPWM_Output_L = 6; // Arduino PWM output pin 6; connect to IBT-2 pin 2 (LPWM)

int LPWM_Output_R = 10; // Arduino PWM output pin 5; connect to IBT-2 pin 1 (RPWM)
int RPWM_Output_R = 9; // Arduino PWM output pin 6; connect to IBT-2 pin 2 (LPWM)

int led_joy=8; //LED for showing that joystick mode is activated; use pin 8
int led_voice=7; //LED for showing that voice command mode is activated; use pin 7
int led_phone=1; //LED to show that phone command mode is activated; use pin 1

int mode=0; //this shows which mode of control will be used for the wheelchair; the default state is voice control
int lastmodebutton_state=HIGH;
int Rxbyte=0; //received byte from Python

//these are the pins for the ultrasonic sensors that will be used (front, back, left and right)
int SensorTrig_front=2; //The front facing ultrasonic sensor trig pin should be connected to pin 2 of the Arduino
int SensorEcho_front=3; //The front facing ultrasonic sensor echo pin should be connected to pin 3 of the Arduino
int SensorTrig_back=4; //The back facing ultrasonic sensor trig pin should be connected to pin 4 of the Arduino
int SensorEcho_back=14; //The back facing ultrasonic sensor echo pin should be connected to pin 14 of the Arduino
int SensorTrig_left=15; //The left facing ultrasonic sensor trig pin should be connected to pin 15 of the Arduino
int SensorEcho_left=16; //The left facing ultrasonic sensor echo pin should be connected to pin 16 of the Arduino
int SensorTrig_right=17; //The right facing ultrasonic sensor trig pin should be connected to pin 17 of the Arduino
int SensorEcho_right=18; //The right facin g ultrasonic sensor echo pin should be connected to pin 18 of the Arduino

//Ultrasonic sensors Setups
NewPing ultrasonic_sensors[SONAR_NUM]={
  NewPing(SensorTrig_front,SensorEcho_front), //ultrasonic sensor for the front
  NewPing(SensorTrig_back,SensorEcho_back), //ultrasonic sensor for the back
  NewPing(SensorTrig_left, SensorEcho_left), //ultrasonic sensor for the left
  NewPing(SensorTrig_right,SensorEcho_right) //ultrasonic sensor for the right
}

unsigned long pingTimer[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor.
unsigned int freezone[SONAR_NUM]=[1,1,1,1];         // Keeps the value that allows movement in all directions. 1=free to move, 0=don't move
/*0 element: front
1 element: back
2 element: left
3 element: right
*/
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.


//voice recognition config params

//VR myVR(50,51);    // 2:RX 3:TX, you can choose your favourite pins.
VR myVR(&Serial1); 

uint8_t records[7]; // save record
uint8_t buf[64];

#define forward (0)
#define backwards (1)
#define Stop (2)
#define left (3)
#define right (4)

void printSignature(uint8_t *buf, int len)
{
  int i;
  for(i=0; i<len; i++){
    if(buf[i]>0x19 && buf[i]<0x7F){
      Serial.write(buf[i]);
    }
    else{
      Serial.print("[");
      Serial.print(buf[i], HEX);
      Serial.print("]");
    }
  }
}

/**
  @brief   Print signature, if the character is invisible, 
           print hexible value instead.
  @param   buf  -->  VR module return value when voice is recognized.
             buf[0]  -->  Group mode(FF: None Group, 0x8n: User, 0x0n:System
             buf[1]  -->  number of record which is recognized. 
             buf[2]  -->  Recognizer index(position) value of the recognized record.
             buf[3]  -->  Signature length
             buf[4]~buf[n] --> Signature
*/
void printVR(uint8_t *buf)
{
  Serial.println("VR Index\tGroup\tRecordNum\tSignature");

  Serial.print(buf[2], DEC);
  Serial.print("\t\t");

  if(buf[0] == 0xFF){
    Serial.print("NONE");
  }
  else if(buf[0]&0x80){
    Serial.print("UG ");
    Serial.print(buf[0]&(~0x80), DEC);
  }
  else{
    Serial.print("SG ");
    Serial.print(buf[0], DEC);
  }
  Serial.print("\t");

  Serial.print(buf[1], DEC);
  Serial.print("\t\t");
  if(buf[3]>0){
    printSignature(buf+4, buf[3]);
  }
  else{
    Serial.print("NONE");
  }
  Serial.println("\r\n");
}


void setup() {
  
  pinMode(changemode_button, INPUT_PULLUP);
  
  pinMode(buzzer_button, INPUT_PULLUP);
  pinMode(buzzer_output, OUTPUT);
  
  pinMode(LPWM_Output_L, OUTPUT);
  pinMode(RPWM_Output_L, OUTPUT);

  pinMode(LPWM_Output_R, OUTPUT);
  pinMode(RPWM_Output_R, OUTPUT);

  pinMode(led_joy, OUTPUT);
  pinMode(led_voice, OUTPUT);
  pinMode(led_phone, OUTPUT);

  //ultrasonic sensors setup
/*  
  pinMode(SensorTrig_front, OUTPUT);
  pinMode(SensorTrig_back, OUTPUT);
  pinMode(SensorTrig_left, OUTPUT);
  pinMode(SensorTrig_back, OUTPUT);
  
  pinMode(SensorEcho_front, INPUT);
  pinMode(SensorEcho_back, INPUT);
  pinMode(SensorEcho_left, INPUT);
  pinMode(SensorEcho_back, INPUT);   
*/
   Serial.begin(9600);

//voice recognition module config
  Serial1.begin(9600);
  
  //bluetooth serial connection
  Serial2.begin(9600);
  
  pingTimer[0] = millis() + 75;           // First ping starts at 75ms, gives time for the Arduino to chill before starting.
  for (uint8_t i = 1; i < SONAR_NUM; i++) // Set the starting time for each sensor.
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;

  //voice recognition elechouse
  Serial.println("Elechouse Voice Recognition V3 Module\r\nControl LED sample");
  
    
  if(myVR.clear() == 0){
    Serial.println("Recognizer cleared.");
  }else{
    Serial.println("Not find VoiceRecognitionModule.");
    Serial.println("Please check connection and restart Arduino.");
    while(1);
  }
  
  if(myVR.load((uint8_t)forward) >= 0){
    Serial.println("forward loaded");
  }
  
  if(myVR.load((uint8_t)backwards) >= 0){
    Serial.println("backwards loaded");
  }

  if(myVR.load((uint8_t)Stop) >= 0){
    Serial.println("Stop loaded");
  }

  if(myVR.load((uint8_t)left) >= 0){
    Serial.println("left loaded");
  }

  if(myVR.load((uint8_t)right) >= 0){
    Serial.println("right loaded");
  }
}

//function to control the left and right motor
void motor_control(int LeftPWM, int RightPWM){
  if (LeftPWM>0 && RightPWM>0){
    Serial.println("Forwards");
    if (freezone[0]==1)
    {
      //move forwards

    }
    else
    {
      //stop the motor

    }
    
  }    
  else if (LeftPWM>0 && RightPWM==0){
    Serial.println("Going Right");
    if (freezone[3]==1)
    {
      //move right

    }
    else
    {
      //stop the motor

    }
  }
  else if(LeftPWM==0 && RightPWM>0){
    Serial.println("Going Left");
    if (freezone[2]==1)
    {
      //move left

    }
    else
    {
      //stop the motor

    }
  }
  else if (LeftPWM==0 && RightPWM==0){
    Serial.println("Stop");
  }
  else{
    Serial.println("Reverse");
    if (freezone[1]==1)
    {
      //going reverse

    }
    else
    {
      //stop the motor

    }
  }    
}

void mode_lights() 
{
  //this function controls the lights for showing the current mode of control of the wheelchair
  Rxbyte=5;
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

//this function handles all the button requests 
void button_actions(){
  int modebutton_reading=digitalRead(changemode_button);
  if (lastmodebutton_state!=modebutton_reading && modebutton_reading==LOW){
    ++mode;
    if (mode==3){
      mode=0;
    }
    for (uint8_t i = 0; i < SONAR_NUM; i++) pingTimer[i] = -1;    //stop ping for ultrasonic sensors
    delay(50);
    //starts pinging of ultrasonic sensors
    pingTimer[0] = millis();
    for (uint8_t i = 1; i < SONAR_NUM; i++) pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
  }
  lastmodebutton_state=modebutton_reading;

  mode_lights();
  
  //activates the buzzer when the button is pressed
  int buzzer_state=!(digitalRead(buzzer_output));
  digitalWrite(buzzer_output, buzzer_state);  
}

//function to receive input from the joystick
void joystickControl(){
  int Xval=analogRead(joyX);
  int Yval=analogRead(joyY);
  int motorspeed=0;
/*
Serial.print("Variable_1:");

  Serial.print(Xval);

  Serial.print(",");

  Serial.print("Variable_2:");

  Serial.print(Yval);

  Serial.print(",");

  Serial.print("Variable_3:");

  Serial.println(static_variable);  


  Serial.print("Xval is ");
  Serial.print(Xval);
  Serial.print("\t Yval is ");
  Serial.print(Yval);
  Serial.println(" ");
  */
 
  //stop the motor (deadzone considered)
  if (Xval>(statX-deadzone) && Xval<(statX+deadzone) && Yval>(statY-deadzone) && Yval<(statY+deadzone)){
    motor_control(motorspeed,motorspeed);  //stop left and right motor  
  }

  else{
    //forward and reverse condition
    if (Yval>(statY-deadzone) && Yval<(statY+deadzone)){
      //forward condition
      if (Xval>=(statX+deadzone))
      {
        motorspeed=map(Xval, (statX+deadzone), 1023, 0, 255);
        motor_control(motorspeed, motorspeed);    
      }

      //backwards condition
      if (Xval<=(statX-deadzone)){
        motorspeed=map(Xval, (statX-deadzone), 0, 0, 255);
        motorspeed=-(motorspeed);
        motor_control(motorspeed, motorspeed);
      }
    }
    
    //left and right condition
    else if (Xval>(statX-deadzone) && Xval<(statX+deadzone)){
      //left condition
      if (Yval<=(statY-deadzone)){
        motorspeed=map(Yval, (statY-deadzone),0, 0, 255);
        motor_control(0, motorspeed);
      }

      //right condition
      if (Yval>=(statY+deadzone)){
        motorspeed=map(Yval, (statY+deadzone), 1023, 0, 255);
        motor_control(motorspeed, 0);        
      }      
    }

    else{
      motor_control(motorspeed,motorspeed);  //stop left and right motor  
    }
    
  }
}

void voiceControl(){
  int ret;
  ret = myVR.recognize(buf, 50);
  if(ret>0){
    switch(buf[1]){
      case forward:{
        //send command to move the wheelchair forward
        motor_control(255, 255);
        break;        
      }
      case backwards:{
        //send command to move the wheelchair backwards
        motor_control(-255, -255);
        break;
      }
      case Stop:{
        //send command to stop the wheelchair
        motor_control(0,0);
        break;        
      }
      case left:{
        //send command to turn the wheelchair to the left
        motor_control(0,255);
        break;
      }      
      case right:{
        //send command to turn the wheelchair to the right
        motor_control(255,0);
        break;        
      }
      default:{
        //stop the wheelchair if the command was not recognized
        Serial.println("Record function undefined");
        motor_control(0,0);
        break;        
      }
    }
    /** voice recognized */
    //printVR(buf);
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

/*mode 0: voice control
mode 1: joystick control
mode 2: smartphone control
*/

void loop() {
  for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through all the sensors.
    if (millis() >= pingTimer[i]) {         // Is it this sensor's time to ping?
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;  // Set next time this sensor will be pinged.
      sonar[currentSensor].timer_stop();          // Make sure previous timer is canceled before starting a new ping (insurance).
      currentSensor = i;                          // Sensor being accessed.
      cm[currentSensor] = 0;                      // Make distance zero in case there's no ping echo for this sensor.
      sonar[currentSensor].ping_timer(echoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
    }
  }

  button_actions();
  //joystick control
  switch(mode){
    //voice control mode
    case 0:    
    {
      //Serial.println("Mode 1");
      voiceControl();      
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
    case 2:{
      //Serial.println("Mode 3");
      smartphone_control();
      break;      
    }
  }  
}

void echoCheck() {
  if (sonar[currentSensor].check_timer())
    pingResult(currentSensor, sonar[currentSensor].ping_result / US_ROUNDTRIP_CM);
}

void pingResult(uint8_t sensor, int cm) {
  // The following code would be replaced with your code that does something with the ping result.
  if (cm<40){
    freezone[sensor]=0;
  }
  else
  freezone[sensor]=1;
  /*
  Serial.print(sensor);
  Serial.print(" ");
  Serial.print(cm);
  Serial.println("cm");
  */
}
