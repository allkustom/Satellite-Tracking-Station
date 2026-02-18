#include <math.h>

float cordX = 7.0f;
float cordY = 12.0f;
float cordZ = 5.0f;
float angle_1;
float angle_2;



const float RAD2DEG = 180.0 / M_PI; 

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  float cal_1 = cordY / (sqrt(pow(cordX,2) + pow(cordY, 2)));
  float cal_2 = cordZ / (sqrt(pow(cordX,2) + pow(cordY,2)+pow(cordZ,2)));
  angle_1 = acos(cal_1) * RAD2DEG;
  angle_2 = acos(cal_2) * RAD2DEG;

  Serial.println("Coordination : " + (String)cordX + " / " + (String)cordY + " / " + (String)cordZ);
  Serial.println("Base Cal : " + (String)cal_1 + " / " + (String)cal_2);
  Serial.println("First angle : " + (String)angle_1);
  Serial.println("Second angle : " + (String)angle_2);


  

  // Serial.println(pow(2,2));
  // Serial.println(sqrt(8));
  // Serial.println(acos(0.5)*RAD2DEG);

}

void loop() {
  // put your main code here, to run repeatedly:

}
