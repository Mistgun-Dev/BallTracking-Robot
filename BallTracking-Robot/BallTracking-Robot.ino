#include <ZumoMotors.h>
#include <SPI.h>  
#include <Pixy.h>

ZumoMotors motors;
Pixy pixy;

typedef unsigned short UINT16;
typedef unsigned char  UINT8;
typedef signed char    INT8;
typedef signed short   INT16;
typedef unsigned int   UINT32;
typedef signed int     INT32;
typedef unsigned char  BYTE;
   
INT32  MOTOR_s32MotorSpeed = 210;             // Vitesse du moteur
INT32  MOTOR_s32MotorSpeedRotate = 0;         // Vitesse de rotation du moteur pour aller a gauche ou a droite
UINT16 PIXY_u16ViewWidth= 319;                // Taille en longueur de la résolution Pixy
UINT16 PIXY_u16ViewHeight = 199;              // Taille en hauteur de la résolution Pixy
UINT16 PIXY_au16PixyMiddleInterval[2] = { (PIXY_u16ViewWidth/2) + 30 ,
                                          (PIXY_u16ViewWidth/2) - 30 };
UINT8 PIXY_u8LastDetectObject = -1;
#define RIGHT       0
#define LEFT        1
#define FRONT       2   
#define calDistance 20 //8 //in inches 24inches or 2 foot
int calWidth = 111; //Calibrated width reading
int calHeight = 107; //Calibrated height reading
int focalLengthWidth;  //calculated focal length for width
int focalLengthHeight; //calculated focal length for height
float widthOfObject = 9; //inches (3.75 inches) real size of your object
float heightOfObject = 9; //inches (2.5 inches) real size of your object
float distanceWidth;   //calculated distance based on the width of the object
float distanceHeight;  //calculated distance based on the height of the object 
float avg;
int pixelsWidth;   //read by the camera
int pixelsHeight; //read by the camera
float inches,feet;

UINT32 u32GetDistanceObj(UINT16 u16objID)
{
  pixelsWidth = pixy.blocks[u16objID].width;
  pixelsHeight = pixy.blocks[u16objID].height;
  distanceWidth = (widthOfObject * focalLengthWidth) / pixelsWidth;
  distanceHeight = (heightOfObject * focalLengthHeight) / pixelsHeight;
  avg = (distanceWidth + distanceHeight)/2;
  avg = round(avg);
  feet = avg/12;
  
  return avg;
}

UINT32 u32GetSpeedRotation(UINT16 u16objectX)
{
   INT32 s32Res = (PIXY_u16ViewWidth / 2) - u16objectX;
   s32Res = abs(s32Res) * 2.5;
   static int i;
   /*
   if(i++%50 == 0)
   {
    Serial.print("res =  ");
    Serial.print(s32Res);
    Serial.print(" abs : ");
    Serial.print(s32Res);
    Serial.print("\n");
   }
*/
   if(s32Res > PIXY_u16ViewWidth * 2)
    return 0;
     
   return s32Res;
}

void setup() 
{
  focalLengthWidth = (calWidth * calDistance) / widthOfObject;
  focalLengthHeight = (calHeight * calDistance) / heightOfObject;

  
  Serial.begin(9600);
  Serial.print("Starting...\n");
  pixy.init();
  MOTOR_vdStop(-1);
}

void MOTOR_vdStop(INT32 u32Delay)
{
   motors.setSpeeds(0,0);

   if(u32Delay < 0)
      return;
      
   delay(u32Delay);
}

void MOTOR_vdMoveForward(INT32 speedLeft, INT32 speedRight)
{
   motors.setSpeeds(speedLeft , speedRight);
}

void MOTOR_vdRotateRight(INT32 speed)
{
   motors.setSpeeds(speed, -speed);
}

void MOTOR_vdRotateLeft(INT32 speed)
{
   motors.setSpeeds(-speed , speed);
}

void MOTOR_vdMoveBack(INT32 speed)
{
   motors.setSpeeds(-speed , -speed);
}

void vdTrackObject(UINT16 u16objID, UINT16 u16objectX, UINT16 u16objectY, UINT16 u16ObjectWidth, UINT16 u16ObjectHeight)
{
  UINT32 u32DistanceObj = u32GetDistanceObj(u16objID);
  MOTOR_s32MotorSpeedRotate = (INT32)u32GetSpeedRotation(u16objectX);
  UINT8  u8DistanceMin = 18;

  
  //Si l'objet se trouve dans le coin gauche du champ de vision de la caméra, on avance en tournant à gauche
  if(u16objectX <= PIXY_au16PixyMiddleInterval[0] && u16objectX <= PIXY_au16PixyMiddleInterval[1])
  {
    if(u32DistanceObj > u8DistanceMin)
      MOTOR_vdMoveForward(MOTOR_s32MotorSpeed, MOTOR_s32MotorSpeedRotate);
    else
      MOTOR_vdRotateLeft(MOTOR_s32MotorSpeedRotate);

    PIXY_u8LastDetectObject = LEFT;
  }
  //Si l'objet se trouve dans le coin droit du champ de vision de la caméra, on avance en tournant à droite
  else if(u16objectX >= PIXY_au16PixyMiddleInterval[0] && u16objectX >= PIXY_au16PixyMiddleInterval[1])
  {
    if(u32DistanceObj > u8DistanceMin)
      MOTOR_vdMoveForward(MOTOR_s32MotorSpeedRotate , MOTOR_s32MotorSpeed);
    else
      MOTOR_vdRotateRight(MOTOR_s32MotorSpeedRotate);

    PIXY_u8LastDetectObject = RIGHT;
  }
  else
  {
    if(u32DistanceObj > u8DistanceMin)
    {
      MOTOR_vdMoveForward(MOTOR_s32MotorSpeed , MOTOR_s32MotorSpeed);
      PIXY_u8LastDetectObject = FRONT;
    }
    else
      MOTOR_vdStop(-1);      
  }
}

void loop() 
{
  static int i = 0;
  int j;
  uint16_t blocks;
  char buf[32]; 
  
  //Récuperer le nombre de blocks détécté
  blocks = pixy.getBlocks();

  //Si il y a des blocks, on les traites
  if(blocks)
  {
      for (j=0; j<blocks; j++)
      {
        // Si la signature == Balle
        if(pixy.blocks[j].signature == 1)
        {
          vdTrackObject(j, pixy.blocks[j].x , pixy.blocks[j].y, pixy.blocks[j].width, pixy.blocks[j].height);
        }
      }
  }
  else
  {
    MOTOR_s32MotorSpeedRotate = 170;
    
    if(PIXY_u8LastDetectObject == LEFT)
      MOTOR_vdRotateLeft(MOTOR_s32MotorSpeedRotate);
      
    else if(PIXY_u8LastDetectObject == RIGHT)
      MOTOR_vdRotateRight(MOTOR_s32MotorSpeedRotate);
      
    else if(PIXY_u8LastDetectObject == FRONT)
      MOTOR_vdMoveForward(MOTOR_s32MotorSpeed , MOTOR_s32MotorSpeed);
      
    else
      MOTOR_vdStop(-1);      
  }

  delay(50);
}
