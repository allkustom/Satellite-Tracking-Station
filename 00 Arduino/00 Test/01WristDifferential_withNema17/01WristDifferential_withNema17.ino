#include <AccelStepper.h>
#include <math.h>

const int stepperCount = 2;
const int dirPin[stepperCount] = {2,4};
const int stepPin[stepperCount] = {3,5};

// Motor Enable/Disable purpose
// Can add motor disable mode with this
const int enPin[stepperCount] = {6,7};
const int stepControlPin = 8;

const int fullStepRev = 200;
// Microstepping setting
// Fullstep -> 1
// Halfstep -> 2
// ...
// 1/16 step -> 16
const int microStepping = 16;
const int stepsPerRevolution = fullStepRev * microStepping;
const float speedAmp = 2.0f;

// Higher number = faster
const int speeds[2] = {20 * microStepping * speedAmp, 200 * microStepping * speedAmp};

// Acceleration
const int accelSpeed = 20 * microStepping * speedAmp;

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
Coord TargetCoord = {0.0, 0.0, 0.0 };

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

// void actionSeq(int cases){
//   switch(cases){
//     // Coordinate tracking
//     case 6:
//       angleCal(10.0, 10.0, 10.0);      
//       break;
//     case 7:
//       angleCal(-15.0, -5.0, 5.0);
//       break;
//     case 9:
//       angleCal(0,maxRadius, 0);
//       break;
  
//     default:
//       Serial.println("CMD out fo range");
//       moving = false;
//       break;
//   }
// }

void cmdCoord(int cmd){
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

  float azim_t = atan2(TargetCoord.y, TargetCoord.x) * RAD2DEG;
  float hori_t = hypot(TargetCoord.x, TargetCoord.y);
  float elev_t = atan2(TargetCoord.z, hori_t) * RAD2DEG;

  float azim_c = atan2(CurrentCoord.y, CurrentCoord.x) * RAD2DEG;
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
//---------------------------------------------------------
//---------------------------------------------------------

void setup() {
  Serial.begin(9600);

  for(int i=0; i<stepperCount; i++){
    pinMode(dirPin[i], OUTPUT);
    pinMode(stepPin[i], OUTPUT);
    if (enPin[i] != 255) {
      pinMode(enPin[i], OUTPUT);
      digitalWrite(enPin[i], LOW);
    }
  }
  
  pinMode(stepControlPin, OUTPUT);
  digitalWrite(stepControlPin, HIGH);

  for(int i=0; i<stepperCount; i++){
    st[i] = new AccelStepper(AccelStepper::DRIVER, stepPin[i], dirPin[i]);
    // Safe Pulse width for A4988
    st[i]->setMinPulseWidth(3);
    st[i]->setMaxSpeed(speeds[0]);
    st[i]->setAcceleration(accelSpeed);
  }  

  delay(1000);
  Serial.println("CMD Ready");



}

void loop() {

  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\n' || c == '\r') {
      if (msg.length() > 0) {
        int cmd = msg.toInt();
        msg = "";

        cmdCoord(cmd);
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


}

