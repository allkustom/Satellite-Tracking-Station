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
const float speedAmp = 5.0f;

// Higher number = faster
const int speeds[2] = {20 * microStepping * speedAmp, 200 * microStepping * speedAmp};

// Acceleration
const int accelSpeed = 20 * microStepping * speedAmp;

AccelStepper* st[stepperCount];

String msg;
bool moving = false;

// Angle calculation with Trig
const float RAD2DEG = 180.0 / M_PI;
float angle_1;
float angle_2;
float rev_1;
float rev_2;
bool dir_yaw = true;
bool dir_pitch = true;

struct Coord {
  float x,y,z;
};


float maxRadius = 100.0f;
Coord CurrentCoord = {0.0, maxRadius, 0.0 };
Coord TargetCoord = {0.0, 0.0, 0.0 };

long revToSteps(float rev){
  return lround(rev * (float)stepsPerRevolution);
}

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

int dirSet(bool dir, bool sep, int index){
  int base = dir ? -1 : +1;
  if(!sep) return base;

  if(index == 0) return base;
  if(index == 1) return -base;
  return base;
}

void stepperRot(bool dir, bool sep, float rev_L, float rev_R, int speed){
  long moveSteps_L = labs(revToSteps(rev_L));
  long moveSteps_R = labs(revToSteps(rev_R));
  if(moveSteps_L < 0 || moveSteps_R < 0) return;

  // enableMotor(true);
  // setProfileMotor(speed, accelSpeed);

  // Num. 0 => Left
  // Num. 1 => Right

  if(rev_L > rev_R){
    float adjustSpeed = rev_L/rev_R*speed ;
    float adjustAccel = rev_L/rev_R*accelSpeed ;
    setProfileMotorSep(0,adjustSpeed, adjustAccel);
    setProfileMotorSep(1,speed, accelSpeed);
  }else if(rev_L < rev_R){
    int adjustSpeed = rev_R/rev_L * speed ;
    float adjustAccel = rev_R/rev_L * accelSpeed ;
    setProfileMotorSep(1, adjustSpeed, adjustAccel);
    setProfileMotorSep(0,speed, accelSpeed);
  }else{
    setProfileMotor(speed, accelSpeed);
  }

  for(int i =0; i< stepperCount; i++){
    int dSet = dirSet(dir, sep, i);
    if(i == 0){
      long target = st[i]->currentPosition() + (long)dSet * moveSteps_L;
      st[i]->moveTo(target);
    }else{
      long target = st[i]->currentPosition() + (long)dSet * moveSteps_R;
      st[i]->moveTo(target);
    }
  }

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

void actionSeq(int cases){
  switch(cases){
    case 0:
      stepperRot(true, false, 0.5f,0.5f, speeds[0]);
      break;
    case 1:
      stepperRot(false, false, 0.5f,0.5f, speeds[0]);
      break;
    case 2:
      stepperRot(true, true, 0.25f,0.25f, speeds[0]);
      break;
    case 3:
      stepperRot(false, true, 0.25f,0.25f, speeds[0]);
      break;
    case 4:
      stepperRot(true, false, 0.5f,0.25f, speeds[0]);
      break;
    case 5:
      stepperRot(false, false, 0.5f,0.25f, speeds[0]);
      break;

    // Coordinate tracking
    case 6:
      // angleCal(120.0, 50.0, 100.0);
      angleCal(10.0, 10.0, 10.0);
      
      break;

    case 7:
      // angleCal(-120.0, 50.0, 20.0);
      angleCal(-15.0, -5.0, 5.0);
      

      break;
      

    // Action Seq
    case 9:
      angleCal(0,maxRadius, 0);

            // stepperRot(true, true, 0.25f, 0.25f, speeds[0]);
      // stepperRot(false, true, 0.5f, 0.5f, speeds[1]);
      // stepperRot(true, true, 0.25f, 0.25f, speeds[1]);
      // stepperRot(true, false, 0.5f, 0.25f, speeds[0]);
      // stepperRot(false, false, 0.5f, 0.25f, speeds[0]);
      // stepperRot(true, false, 0.5f, 0.5f, speeds[0]);
      // stepperRot(false, false, 0.5f, 0.5f, speeds[1]);
      break;
  
    default:
      Serial.println("CMD out fo range");
      moving = false;
      break;

  }
}

static inline float wrapDeg(float a){
  while(a > 180.0f) a -= 360.0f;
  while(a <= -180.0f) a += 360.0f;
  return a;
}

void angleCal(float coordX, float coordY, float coordZ){
  TargetCoord = {coordX,coordY,coordZ};

  // float normX = fabsf(coordX);
  // float normY = fabsf(coordY);
  // float normZ = fabsf(coordZ);
  // Serial.println("Norm_1 :" + (String)normX + " / "+ (String)normY + " / "+ (String)normZ + " / ");

  // float target_cal_1 = normY / hypot(normX,normY);
  // float target_cal_2 = normZ / (sqrt(pow(normX,2) + pow(normY,2) + pow(normZ,2)));

  // normX = fabsf(CurrentCoord.x);
  // normY = fabsf(CurrentCoord.y);
  // normZ = fabsf(CurrentCoord.z);

  // float current_cal_1 = normY / (sqrt(pow(normX,2) + pow(normY, 2)));
  // float current_cal_2 = normZ / (sqrt(pow(normX,2) + pow(normY,2) + pow(normZ,2)));
  // if(normX == 0.0 && normY == 0.0 && normZ == 0.0){
  //   angle_1 = (acos(target_cal_1) * RAD2DEG);
  //   angle_2 = (acos(target_cal_2) * RAD2DEG);
  // }else{
  //   angle_1 = (acos(target_cal_1) * RAD2DEG) + (acos(current_cal_1) * RAD2DEG);
  //   angle_2 = (acos(target_cal_2) * RAD2DEG) - (acos(current_cal_2) * RAD2DEG);
  // }

  //-----------------------------------------------
  //-----------------------------------------------
  // Feb 24th
  // Made a mistake
  // It was better to use tan instead of cos
  // shift the logic into 'atan2' to check the coordinate's quadrant

  float yaw_t = atan2(coordY, coordX) * RAD2DEG;
  float hori_t = hypot(coordX, coordY);
  float pitch_t = atan2(coordZ, hori_t) * RAD2DEG;

  float yaw_c = atan2(CurrentCoord.y, CurrentCoord.x) * RAD2DEG;
  float hori_c = hypot(CurrentCoord.x, CurrentCoord.y);
  float pitch_c = atan2(CurrentCoord.z, hori_c) * RAD2DEG;

  angle_1 = wrapDeg(yaw_t - yaw_c);
  angle_2 = pitch_t - pitch_c;


  // Use result as a direction
  dir_yaw = angle_1 < 0 ? true : false;
  dir_pitch = angle_2 < 0 ? false : true;

  // Serial.println("Angle_1: " + (String)angle_1 + " / dir_yaw: " + dir_yaw);
  // Serial.println("Angle_2: " + (String)angle_2 + " / dir_pitch: " + dir_pitch);




  angle_1 = fabsf(angle_1);
  angle_2 = fabsf(angle_2);

  updateCurrentRev();
  updateCurrentCoord(coordX, coordY, coordZ);
  rotateToTarget();
}

void rotateToTarget(){

  // For debugging purpose, separated the 2 angle movement
  // Calculate right result based on the direction and turn it into a point to point smooth action
  stepperRot(dir_yaw, false, rev_1, rev_1, speeds[0]);
  stepperRot(dir_pitch, true, rev_2, rev_2, speeds[0]);

  // if(rev_1 < rev_2){
  //   stepperRot(dir_pitch,true, rev_1+rev_2, rev_2 - rev_1, speeds[0]);
  // }else{
  //   stepperRot(dir_yaw,false, rev_1+rev_2, rev_1 - rev_2, speeds[0]);
  // }
}

void updateCurrentRev(){
  rev_1 = angle_1 / 360;
  rev_2 = angle_2 / 360;
  Serial.println("rev_1 : " + (String)rev_1);
  Serial.println("rev_2 : " + (String)rev_2);
    
}
void updateCurrentCoord(float coordX, float coordY, float coordZ){
  CurrentCoord.x = coordX;
  CurrentCoord.y = coordY;
  CurrentCoord.z = coordZ;
}


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

  // This for changing the step mode
  // pinMode(ms1, OUTPUT);
  // pinMode(ms2, OUTPUT);
  // pinMode(ms3, OUTPUT);
  // digitalWrite(ms1,HIGH);
  // digitalWrite(ms2,HIGH);

  delay(200);

    Serial.println("CMD Ready");



}

void loop() {

  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\n' || c == '\r') {
      if (msg.length() > 0) {
        int cmd = msg.toInt();
        msg = "";

        Serial.print("CMD = ");
        Serial.println(cmd);

        actionSeq(cmd);
        moving = true;
      }
    } else {
      msg += c;
    }
  }


}

