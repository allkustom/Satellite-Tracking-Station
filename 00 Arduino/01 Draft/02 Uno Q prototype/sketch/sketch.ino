// Active the below line in Uno Q env
#include <Arduino_RouterBridge.h>

bool executed = false;

#include <AccelStepper.h>
#include <math.h>
#include "manager.h"

int assignedStationID;

const int limitSwitchPin = 12;
int limitSwitchState;
int lastLimitSwitchState = LOW;

unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;


const int stepperCount = 2;
const int dirPin[stepperCount] = {2,3};
const int stepPin[stepperCount] = {4,5};

// Motor Enable/Disable purpose
// Can add motor disable mode with this
const int enPin[stepperCount] = {6,7};
const int stepControlPin[3] = {8,9,10};


const long fullStepRev = 200;

// const long gearboxRatio = 20;
// const int motorDirection = -1;
// const long microStepSetting = 4;
// const float speedAmp = 0.25f;

long gearboxRatio = 1;
int motorDirection = 1;
long microStepSetting = 16;
float speedAmp = 2.0f;

const long microStepping = microStepSetting * gearboxRatio;
const long stepsPerRevolution = fullStepRev * microStepping;

// Higher number = faster
const float speeds[2] = {100.0f * microStepping * speedAmp, 200.0f * microStepping * speedAmp};

// Acceleration
const float accelSpeed = 200 * microStepping * speedAmp;

AccelStepper* st[stepperCount];

String msg;
bool moving = false;
int cmdMsgCount = 0;

// Angle calculation with Trig
const float RAD2DEG = 180.0 / M_PI;

struct Coord {
  float x,y,z;
};


float maxRadius = 100.0f;
Coord CurrentCoord = {0.0, maxRadius, 0.0 };
// Coord CurrentCoord = {maxRadius, 0.0, 0.0 };
Coord TargetCoord = {0.0, 0.0, 0.0};


//---------------------------------------------------------
//---------------------------------------------------------
// Utilites
long revToSteps(float rev){
  // Monitor.println((long)round(rev * (float)stepsPerRevolution));
  return (long)round(rev * (float)stepsPerRevolution);
  

  //// [!] Arduino Uno Q doesn't support lround????????????
  //// lround() blocks the 'Bridge.provide' method.
  // return lround(rev * (float)stepsPerRevolution);
}

float clampAzim(float a){
  while(a > 180.0f) a -= 360.0f;
  while(a <= -180.0f) a += 360.0f;
  return a;
}
float clampElev(float a){
  if(a > 90.0f) a = 90.0f;
  else if(a < -45.0f) a= -45.0f;
  return a;
}




//---------------------------------------------------------
//---------------------------------------------------------
// Motor Settings

// Enable/Disable motor
void enableMotor(bool activate){
  for(int i = 0; i < stepperCount; i++){
    digitalWrite(enPin[i], activate ? LOW : HIGH);
  }
}

void setProfileMotor(float maxSpeed, float accelSpeeds){
  for(int i = 0; i < stepperCount; i++){
    st[i]->setMaxSpeed(maxSpeed);
    st[i]->setAcceleration(accelSpeeds);
  }
}

void setProfileMotorSep(int index, float maxSpeed, float accelSpeeds){
  st[index]->setMaxSpeed(maxSpeed);
  st[index]->setAcceleration(accelSpeeds);
  Serial.println("Motor: " + (String)index + " - Speed: " + (String)maxSpeed + "/Accel: " + (String)accelSpeeds);
}

//---------------------------------------------------------
//---------------------------------------------------------

// Controling both motor individually
void stepperRot(float rev_L, float rev_R, float speed){
  // Set direction of rotation
  // [true] => clockwise
  // [false] =? counter-clockwise
  int dirL = rev_L < 0 ? -1 : 1;
  int dirR = rev_R < 0 ? -1 : 1;

  dirL = dirL * motorDirection;
  dirR = dirR * motorDirection;



  // // Turn into long and apply absolute
  long moveSteps_L = labs(revToSteps(rev_L));
  long moveSteps_R = labs(revToSteps(rev_R));

  long maxSteps = max(moveSteps_L, moveSteps_R);
  Monitor.println(maxSteps);
  if(maxSteps == 0) return;

  float ratioL = (float)moveSteps_L / (float)maxSteps;
  float ratioR = (float)moveSteps_R / (float)maxSteps;

  float speedL = speed * ratioL;
  float speedR = speed * ratioR;
  float accelL = accelSpeed * ratioL;
  float accelR = accelSpeed * ratioR;
  setProfileMotorSep(0, speedL, accelL);
  setProfileMotorSep(1, speedR, accelR);




  long targetL = st[0] -> currentPosition() + dirL * moveSteps_L;
  long targetR = st[1] -> currentPosition() + dirR * moveSteps_R;
  
  st[0]->moveTo(targetL);
  st[1]->moveTo(targetR);

  
  bool anyRunning;

  do{
    anyRunning = false;
    for(int i = 0; i < stepperCount; i++){
      st[i]->run();
      if(st[i]->distanceToGo() != 0){
        anyRunning = true;
      }
    }
  } while(anyRunning);
}

void cmdCoord(String cmdString){
  cmdString.toLowerCase();
  if(cmdString.startsWith("a")){
    float cmdStringCut = cmdString.substring(1).toFloat();
    stepperRot(-(1.0f / 360.0f * cmdStringCut), -(1.0f / 360.0f * cmdStringCut), speeds[0]);
    return;
  }else if (cmdString.startsWith("d")){
    float cmdStringCut = cmdString.substring(1).toFloat();
    stepperRot((1.0f / 360.0f * cmdStringCut), (1.0f / 360.0f * cmdStringCut), speeds[0]);
    return;
  }else if (cmdString.startsWith("w")){
    float cmdStringCut = cmdString.substring(1).toFloat();
    stepperRot(-(1.0f / 360.0f * cmdStringCut), (1.0f / 360.0f * cmdStringCut), speeds[0]);
    return;
  }else if (cmdString.startsWith("s")){
    float cmdStringCut = cmdString.substring(1).toFloat();
    stepperRot((1.0f / 360.0f * cmdStringCut), -(1.0f / 360.0f * cmdStringCut), speeds[0]);
    return;
  }
  
  if (cmdString.startsWith("action")){
    TargetCoord = {45,45,45};
    angleCal();
    TargetCoord = {-45,45,10};
    angleCal();
    TargetCoord = {20,70,85};
    angleCal();
    TargetCoord = {-75,-50,10};
    angleCal();
    TargetCoord = {45,-45,85};
    angleCal();
    TargetCoord = {0,0,0};
    angleCal();
    return;
  }

  if(cmdString == "calib0"){
    calibSeq();
    return;
  }

  if(cmdString == "calib1"){
    TargetCoord = {0,0,0};
    angleCal();
    calibSeq();
    return;
  }


  int cmd = cmdString.toInt();
  if(cmd == 9999 ){
    cmdMsgCount = 0;
    Serial.println("Reset coord cmd");
    return;    
  }
  
  if(cmdMsgCount == 0 ){
    TargetCoord.x = cmd;
    Serial.print("Target X: ");
    Serial.println(cmd);
  }else if(cmdMsgCount == 1 ){
    TargetCoord.y = cmd;
    Serial.print("Target Y: ");
    Serial.println(cmd);
  }else{
    TargetCoord.z = cmd;
    Serial.print("Target Z: ");
    Serial.println(cmd);

    cmdMsgCount = 0;
    moving = true;

    Serial.print("Target Coordinate: ");
    Serial.print(TargetCoord.x);
    Serial.print(" / ");
    Serial.print(TargetCoord.y);
    Serial.print(" / ");
    Serial.println(TargetCoord.z);
    angleCal();
    moving = false;
    return;
  }
  cmdMsgCount++;
}



void angleCal(){
  float azim_t = atan2(TargetCoord.x, TargetCoord.y) * RAD2DEG;
  // *** Arduino Uno Q not support 'hypot'
  // *** Manually calculate with basic features
  // float hori_t = hypot(TargetCoord.x, TargetCoord.y);
  float hori_t = sqrt(pow(TargetCoord.x,2) + pow(TargetCoord.y,2));
  float elev_t = atan2(TargetCoord.z, hori_t) * RAD2DEG;
  // Serial.print("Elev_t is ");
  // Serial.println(elev_t);
  if(elev_t < -45.0f){
    TargetCoord.z = -hori_t;
  }else if (elev_t > 90.0f){
    TargetCoord.z = hori_t;
  }
  elev_t = clampElev(elev_t);
  // Serial.print("Revised Elev_t is ");
  // Serial.println(elev_t);
  // Serial.println("New Target Cooord Z is " + (String)TargetCoord.z);

  float azim_c = atan2(CurrentCoord.x, CurrentCoord.y) * RAD2DEG;
  // float hori_c = hypot(CurrentCoord.x, CurrentCoord.y);
  float hori_c = sqrt(pow(CurrentCoord.x,2) + pow(CurrentCoord.y,2));
  float elev_c = atan2(CurrentCoord.z, hori_c) * RAD2DEG;

  float angle_1 = clampAzim(azim_t - azim_c);

  // float angle_2 = clampElev(elev_t - elev_c);
  // if((elev_t - elev_c) < -45.0f){
  //   TargetCoord.z = -fabsf(TargetCoord.y);
  // }
  // elev_t = clampElev(elev_t);
  // elev_c = clampElev(elev_c);

  float angle_2 = elev_t - elev_c;



  // Use result as a direction
  bool dir_azim = angle_1 < 0 ? true : false;
  bool dir_elev = angle_2 < 0 ? false : true;

  // Serial.println("Angle_1: " + (String)angle_1 + " / dir_azim: " + dir_azim);
  // Serial.println("Angle_2: " + (String)angle_2 + " / dir_elev: " + dir_elev);

  angle_1 = fabsf(angle_1);
  angle_2 = fabsf(angle_2);
  float rev_1 = angle_1 / 360.0f;
  float rev_2 = angle_2 / 360.0f;

  // Serial.print("Target Azim: "); Serial.println(azim_t);
  // Serial.print("Current Azim: "); Serial.println(azim_c);
  // Serial.print("Delta Angle_1: "); Serial.println(angle_1);


  rotToTarget(dir_azim, rev_1, dir_elev, rev_2);
  updateCurrentCoord(TargetCoord);
}

void rotToTarget(bool dir_azim, float rev_1, bool dir_elev, float rev_2){

  float rev_1_L = rev_1;
  float rev_1_R = rev_1;
  float rev_2_L = rev_2;
  float rev_2_R = rev_2;

  if(dir_azim){
    rev_1_L = rev_1_L * -1;
    rev_1_R = rev_1_R * -1;
  }
  if(dir_elev){
    rev_2_L = rev_2_L * -1;
  }else{
    rev_2_R = rev_2_R * -1;
  }

  float rev_L = rev_1_L + rev_2_L;
  float rev_R = rev_1_R + rev_2_R;
  
  Serial.println("rev_1: " + (String)rev_1 + " / " + "rev_2: " + (String)rev_2);
  Serial.println("rev_L: " + (String)rev_L + " / " + "rev_R: " + (String)rev_R);

  stepperRot(rev_L, rev_R, speeds[0]);

}


void updateCurrentCoord(Coord v){
  CurrentCoord.x = v.x;
  CurrentCoord.y = v.y;
  CurrentCoord.z = v.z;
}

bool limitCalib(){
  
  int reading = digitalRead(limitSwitchPin);
  // Serial.println(reading);
  if(reading != lastLimitSwitchState){
    lastDebounceTime = millis();
  }

  if((millis() - lastDebounceTime) > debounceDelay){
    if(reading != limitSwitchState){
      limitSwitchState = reading;
      if(limitSwitchState == LOW){
        Serial.println("Hit Switch");
        return true;
      }
    }
  }
  lastLimitSwitchState = reading;
  return false;
}

void calibSeq(){
  bool onCalib = true;
  while (onCalib){
    stepperRot(-0.01f,0.01f, speeds[0]);
    delay(25);
    if(limitCalib()){
      onCalib = false;
      Serial.println("Top(90 degree) Hit");
    }
  }
  onCalib = true;
  stepperRot(0.3f, -0.3f, speeds[0]);
  while (onCalib){
    stepperRot(0.01f,-0.01f, speeds[0]);
    delay(25);
    if(limitCalib()){
      onCalib = false;
      Serial.println("Bot(-45 degree) Hit");
    }
  }
  stepperRot(-0.125f, 0.125f, speeds[0]);
  Serial.println("Angle Calibration Done");

  Bridge.call("callReady", true);
}

void pythonCoord(float x, float y, float z){
  Monitor.println("Requested Coord is : " + (String)x + " / " + (String)y + " / " + (String)z);
  TargetCoord.x = x;
  TargetCoord.y = y;
  TargetCoord.z = z;
  // executeAngleCal = activate;

  angleCal();
}

//---------------------------------------------------------
//---------------------------------------------------------

void setup() {
  Bridge.begin();

  Monitor.begin();
  Serial.begin(115200);
  delay(500);

  // [!] Provide functions to Python, under Uno Q env
  Bridge.provide_safe("pythonInput", pythonCoord);

  
  assignedStationID = int(stationID);
  Monitor.println("Station ID is " + (String)assignedStationID);
  // Serial.println("Station ID is " + (String)assignedStationID);
  pinMode(limitSwitchPin, INPUT_PULLUP);
  Monitor.println("Limit Switch Ready");
  // Serial.println("Limit Switch Ready");

  for(int i=0; i<stepperCount; i++){
    pinMode(dirPin[i], OUTPUT);
    pinMode(stepPin[i], OUTPUT);
    if (enPin[i] != 255) {
      pinMode(enPin[i], OUTPUT);
      digitalWrite(enPin[i], LOW);
    }
  }
  

  for(int i =0; i < 3; i ++){
    pinMode(stepControlPin[i], OUTPUT);
    digitalWrite(stepControlPin[i], HIGH);
    // digitalWrite(stepControlPin[i], LOW);
  }



  if(microStepSetting == 4){
    digitalWrite(stepControlPin[2], LOW);    
  }

  for(int i=0; i<stepperCount; i++){
    st[i] = new AccelStepper(AccelStepper::DRIVER, stepPin[i], dirPin[i]);
    // Safe Pulse width for A4988
    st[i]->setMinPulseWidth(3);
    st[i]->setMaxSpeed(speeds[0]);
    st[i]->setAcceleration(accelSpeed);
  }  

  delay(1000);
  Monitor.println("Angle Calibration Start");
  // Serial.println("Angle Calibration Start");
  
  stepperRot(-0.15f, 0.15f, speeds[0]);
  calibSeq();

  delay(500);
  Serial.println("READY");

  


}



void loop() {
  // if(!executed){
  //   delay(5000);
  //   Bridge.notify("callReady", true);
  //   executed = true;
  // }
  

  // while (Serial.available() > 0) {
  //   char c = (char)Serial.read();

  //   if (c == '\n' || c == '\r') {
  //     if (msg.length() > 0) {

  //       // int cmd = msg.toInt();
  //       // cmdCoord(cmd);

  //       cmdCoord(msg);
  //       msg = "";
  //       // Serial.print("CMD = ");
  //       // Serial.println(cmd);

  //       // actionSeq(cmd);
  //       // moving = true;
  //     }
  //   } else {
  //     if(!moving){
  //       msg += c;
  //     }
  //   }
  // }

    TargetCoord = {45,45,45};
    angleCal();
    delay(500);
    TargetCoord = {-45,45,10};
    angleCal();
    delay(500);
    TargetCoord = {20,70,-85};
    angleCal();
    delay(500);
    TargetCoord = {-75,-50,10};
    angleCal();
    delay(500);
    TargetCoord = {45,-45,85};
    angleCal();
    delay(500);
    TargetCoord = {0,0,0};
    angleCal();
    delay(500);

  // Monitor.println("Arduino Working");
  // Bridge.call("callReady", true);
  // delay(500);
  //   TargetCoord = {0,0,0};
  //   angleCal();
  //   delay(500);
  

  // Bridge.call("Call");
  // Monitor.println("Arduino Working");
  // delay(500);
  

}

