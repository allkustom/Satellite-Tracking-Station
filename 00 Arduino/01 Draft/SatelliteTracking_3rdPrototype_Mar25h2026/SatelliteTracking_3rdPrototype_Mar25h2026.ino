#include <AccelStepper.h>
#include <math.h>

// TODO, Apr 9th
// Match A4988's STEP/DIR according to the setting
// Add GY-273 at SDA/SCL and D11(DRDY)

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


const int fullStepRev = 200;

// Apply gearbox ratio
const int gearboxRatio = 1;
// Apply gearbox rotation shift
const int motorDirection = 1;

// Microstepping setting
// Fullstep -> 1
// Halfstep -> 2
// ...
// 1/16 step -> 16
const int microStepping = 16;
const int stepsPerRevolution = fullStepRev * microStepping * gearboxRatio;
const float speedAmp = 2.0f;

// Higher number = faster
const int speeds[2] = {100 * microStepping * speedAmp, 200 * microStepping * speedAmp};

// Acceleration
const int accelSpeed = 100 * microStepping * speedAmp;

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
  return lround(rev * (float)stepsPerRevolution);
}

float wrapDeg(float a){
  while(a > 180.0f) a -= 360.0f;
  while(a <= -180.0f) a += 360.0f;
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
void stepperRot(float rev_L, float rev_R, int speed){
  // Set direction of rotation
  // [true] => clockwise
  // [false] =? counter-clockwise
  int dirL = rev_L < 0 ? -1 : 1;
  int dirR = rev_R < 0 ? -1 : 1;

  dirL = dirL * motorDirection;
  dirR = dirR * motorDirection;

  // Turn into long and apply absolute
  double moveSteps_L = labs(revToSteps(rev_L));
  double moveSteps_R = labs(revToSteps(rev_R));


  // Map speed, accel
  // When the goal revolution is different,
  // slower the less revoluting moter
  float adjustSpeed;
  float adjustAccel;
  if(moveSteps_L > moveSteps_R){
    adjustSpeed = moveSteps_R/moveSteps_L * speed ;
    adjustAccel = moveSteps_R/moveSteps_L * accelSpeed ;
    setProfileMotorSep(0,speed, accelSpeed);
    setProfileMotorSep(1,adjustSpeed, adjustAccel);
  }else if(moveSteps_L < moveSteps_R){
    adjustSpeed = moveSteps_L/moveSteps_R * speed ;
    adjustAccel = moveSteps_L/moveSteps_R * accelSpeed ;
    setProfileMotorSep(0, adjustSpeed, adjustAccel);
    setProfileMotorSep(1,speed, accelSpeed);
  }else{
    setProfileMotor(speed, accelSpeed);
  }

  // Set target and direction
  long target;
  // target = st[0]->currentPosition() + moveSteps_L;
  // st[0]->moveTo(target);
  // target = st[1]->currentPosition() + moveSteps_R;
  // st[1]->moveTo(target);
  target = st[0]->currentPosition() + (long)dirL * moveSteps_L;
  st[0]->moveTo(target);
  target = st[1]->currentPosition() + (long)dirR * moveSteps_R;
  st[1]->moveTo(target);


  bool anyRunning;
  do{
    anyRunning = false;
    for(int i =0; i< stepperCount; i++){
      if(st[i]->distanceToGo() != 0){
        st[i]->run();
        anyRunning = true;
      }
    }
  }
  while (anyRunning);
}

void cmdCoord(String cmdString){
  cmdString.toLowerCase();
  if(cmdString.startsWith("z")){
    float cmdStringCut = cmdString.substring(1).toFloat();
    stepperRot((1.0f / 360.0f * cmdStringCut), (1.0f / 360.0f * cmdStringCut), speeds[0]);
    return;
  }else if (cmdString.startsWith("x")){
    float cmdStringCut = cmdString.substring(1).toFloat();
    stepperRot(-(1.0f / 360.0f * cmdStringCut), -(1.0f / 360.0f * cmdStringCut), speeds[0]);
    return;
  }
  else if (cmdString.startsWith("action")){
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
  // TargetCoord = {coordX,coordY,coordZ};

  float azim_t = atan2(TargetCoord.x, TargetCoord.y) * RAD2DEG;
  float hori_t = hypot(TargetCoord.x, TargetCoord.y);
  float elev_t = atan2(TargetCoord.z, hori_t) * RAD2DEG;

  float azim_c = atan2(CurrentCoord.x, CurrentCoord.y) * RAD2DEG;
  float hori_c = hypot(CurrentCoord.x, CurrentCoord.y);
  float elev_c = atan2(CurrentCoord.z, hori_c) * RAD2DEG;

  float angle_1 = wrapDeg(azim_t - azim_c);
  float angle_2 = elev_t - elev_c;


  // Use result as a direction
  bool dir_azim = angle_1 < 0 ? true : false;
  bool dir_elev = angle_2 < 0 ? false : true;

  // Serial.println("Angle_1: " + (String)angle_1 + " / dir_azim: " + dir_azim);
  // Serial.println("Angle_2: " + (String)angle_2 + " / dir_elev: " + dir_elev);

  angle_1 = fabsf(angle_1);
  angle_2 = fabsf(angle_2);
  float rev_1 = angle_1 / 360;
  float rev_2 = angle_2 / 360;

  Serial.print("Target Azim: "); Serial.println(azim_t);
  Serial.print("Current Azim: "); Serial.println(azim_c);
  Serial.print("Delta Angle_1: "); Serial.println(angle_1);


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
  stepperRot(-0.20f, 0.20f, speeds[0]);
  bool onCalib = true;
  while (onCalib){
    stepperRot(-0.005f,0.005f, speeds[0]);
    delay(25);
    if(limitCalib()){
      onCalib = false;
      Serial.println("Top(90 degree) Hit");
    }
  }
  onCalib = true;
  stepperRot(0.35f, -0.35f, speeds[0]);
  while (onCalib){
    stepperRot(0.005f,-0.005f, speeds[0]);
    delay(25);
    if(limitCalib()){
      onCalib = false;
      Serial.println("Bot(-45 degree) Hit");
    }
  }
  stepperRot(-0.125f, 0.125f, speeds[0]);
  Serial.println("Angle Calibration Done");


  
}

//---------------------------------------------------------
//---------------------------------------------------------

void setup() {
  Serial.begin(9600);
  pinMode(limitSwitchPin, INPUT_PULLUP);
  Serial.println("Limit Switch Ready");

  for(int i=0; i<stepperCount; i++){
    pinMode(dirPin[i], OUTPUT);
    pinMode(stepPin[i], OUTPUT);
    if (enPin[i] != 255) {
      pinMode(enPin[i], OUTPUT);
      digitalWrite(enPin[i], LOW);
    }
  }
  
  for(int i =0; i < 4; i ++){
    pinMode(stepControlPin[i], OUTPUT);
    digitalWrite(stepControlPin[i], HIGH);
  }

  for(int i=0; i<stepperCount; i++){
    st[i] = new AccelStepper(AccelStepper::DRIVER, stepPin[i], dirPin[i]);
    // Safe Pulse width for A4988
    st[i]->setMinPulseWidth(3);
    st[i]->setMaxSpeed(speeds[0]);
    st[i]->setAcceleration(accelSpeed);
  }  

  delay(1000);
  Serial.println("Angle Calibration Start");
  calibSeq();

  delay(1000);
  Serial.println("CMD Ready");

}



void loop() {

  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\n' || c == '\r') {
      if (msg.length() > 0) {

        // int cmd = msg.toInt();
        // cmdCoord(cmd);

        cmdCoord(msg);
        msg = "";
        // Serial.print("CMD = ");
        // Serial.println(cmd);

        // actionSeq(cmd);
        // moving = true;
      }
    } else {
      if(!moving){
        msg += c;
      }
    }
  }

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
  

}

