
#include "Pixy2.h"
#include <Servo.h>

#define XMOY0 (pixy.frameWidth/2) //39
#define YMOY0 0

#define MOT_MAX        1100

Pixy2 pixy;
Servo escRight, escLeft, leServo;   // Création de l'objet permettant le contrôle de l'ESC

uint8_t x1, x2, y1, y2;
uint8_t x3, x4, y3, y4;
uint8_t x5, x6, y5, y6;
uint8_t xMoy0, yMoy0, xMoy1, yMoy1, xMoy2, yMoy2;
int correction_deerPast;
uint8_t index1, index2;


void setup()
{

  escRight.attach(5, 1000, 2000); // On attache l'ESC au port numérique 9 (port PWM obligatoire)
  escLeft.attach(6, 1000, 2000);
  leServo.attach(8);

  delay(15);

  Serial.begin(9600);

  leServo.write(90);
  
//  delay(12000);
//  escRight.writeMicroseconds(1000);
//  escLeft.writeMicroseconds(1000);
//  delay(15000);
  
  // we need to initialize the pixy object
  pixy.init();
  // Change to line tracking program
  pixy.changeProg("line");
  pixy.setLamp(0, 0);

  pixy.line.getAllFeatures();

  xMoy0 = XMOY0;
  yMoy0 = YMOY0;
  index1 = 0;
  index2 = 1;

}





void RecupCoord()
{
  uint8_t x, y;
  
  if (pixy.line.numVectors == 2) {
    x3 = pixy.line.vectors[0].m_x1;
    y3 = pixy.line.vectors[0].m_y1; //coordonnées fleche vecteur 1
    x4 = pixy.line.vectors[1].m_x1;
    y4 = pixy.line.vectors[1].m_y1; //coordonnées fleche vecteur 2
    x5 = pixy.line.vectors[0].m_x0;
    y5 = pixy.line.vectors[0].m_y0; //coordonnées base vecteur 1
    x6 = pixy.line.vectors[1].m_x0;
    y6 = pixy.line.vectors[1].m_y0; //coordonnées base vecteur 2

    index1 = pixy.line.vectors[0].m_index;
    index2 = pixy.line.vectors[1].m_index;

    x1 = (x3+x5)/2;
    y1 = (y3+y5)/2;
    x2 = (x4+x6)/2;
    y2 = (y4+y6)/2;
  }
  else if (pixy.line.numVectors == 1) {
    x = pixy.line.vectors[0].m_x1;
    y = pixy.line.vectors[0].m_y1;
    if (x5-10 < x < x5+10){
      x1 = x; 
      x2 = x2;
      index1 = pixy.line.vectors[0].m_index;
    }
    if (x6-10 < x < x6+10){
      x2 = x; 
      x1 = x1;
      index2 = pixy.line.vectors[0].m_index;
    }
    else {
      x1 = x1;
      x2 = x2;
    }
  }
  else if (pixy.line.numVectors > 2) {
    for (int i=0; i<pixy.line.numVectors; i++)
    { 
      if (index1 == pixy.line.vectors[i].m_index){
        x3 = pixy.line.vectors[i].m_x1;
        y3 = pixy.line.vectors[i].m_y1;
        x5 = pixy.line.vectors[i].m_x0;
        y5 = pixy.line.vectors[i].m_y0;
      }
      else if (index2 == pixy.line.vectors[i].m_index){
        x4 = pixy.line.vectors[i].m_x1;
        y4 = pixy.line.vectors[i].m_y1;
        x6 = pixy.line.vectors[i].m_x0;
        y6 = pixy.line.vectors[i].m_y0;
      }
    }
    x1 = (x3+x5)/2;
    y1 = (y3+y5)/2;
    x2 = (x4+x6)/2;
    y2 = (y4+y6)/2;
  }
 }

void MoyennageCoord()
{
  uint16_t j = 0;
  uint16_t  xM1 = 0, yM1 = 0, xM2 = 0, yM2 = 0;

  for (int i = 0; i < 3; i++) {
    RecupCoord();
    xM1 += (uint16_t)x1;
    yM1 += (uint16_t)y1;
    xM2 += (uint16_t)x2;
    yM2 += (uint16_t)y2;
    j += 1;
//    Serial.print("xM1 ");
//    Serial.println(xM1);
//    Serial.print("xM2 ");
//    Serial.println(xM2);
//    Serial.print("yM1 ");
//    Serial.println(yM1);
//    Serial.print("yM2 ");
//    Serial.println(yM2);
  }
  xMoy1 = (uint8_t)(xM1 / j);
  yMoy1 = (uint8_t)(yM1 / j);
  xMoy2 = (uint8_t)(xM2 / j);
  yMoy2 = (uint8_t)(yM2 / j);
  Serial.print("xMoy1 ");
  Serial.println(xMoy1);
  Serial.print("xMoy2 ");
  Serial.println(xMoy2);
  Serial.print("yMoy1 ");
  Serial.println(yMoy1);
  Serial.print("yMoy2 ");
  Serial.println(yMoy2);
}


void RetourCoordonnee_x0()
{
  MoyennageCoord();


  xMoy0 = (xMoy1 + xMoy2) / 2;
  yMoy0 = (yMoy1 + yMoy2) / 2;
  Serial.print("xMoy0 ");
  Serial.println(xMoy0);
  Serial.print("yMoy0 ");
  Serial.println(yMoy0);

}


void escVitesseDual(int vitesse)
{
  escRight.writeMicroseconds(vitesse);
  escLeft.writeMicroseconds(vitesse);
}

void deerServo(int deer)
{
  leServo.write(deer);
}

void start_nxp()
{
  int8_t res;
  int error;
  int correction_deer;

  res = pixy.line.getAllFeatures();

  // If error or nothing detected, stop motors
  if (res <= 0)
  {
    //escVitesseDual(1000);
    Serial.print("stop ");
    Serial.println(res);
    return;
  }

  // We found the vector...
  if (res & LINE_VECTOR)
  {

    RetourCoordonnee_x0();


    error = xMoy0 - XMOY0;
    correction_deer = 90 + error;

    Serial.print("error ");
    Serial.println(error);
    Serial.print("correction_deer ");
    Serial.println(correction_deer);


    if (20<=correction_deer<=160){
      deerServo(correction_deer);
      correction_deerPast = correction_deer;
    }
    else {
      deerServo(correction_deerPast);
    }
    
    
    //escVitesseDual(1050);

  }
}


void loop()
{

  start_nxp();
  //  RetourCoordonnee_x0();
    Serial.print("....");
    Serial.print("....");
    Serial.print("....");
    Serial.print("....");
}
