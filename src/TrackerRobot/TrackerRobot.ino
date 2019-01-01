#include <ZumoMotors.h>
#include <SPI.h>  
#include <Pixy.h>
#include <Pushbutton.h> 

/* Redéfinition des intitulés des types de données */
typedef unsigned short UINT16;
typedef unsigned char  UINT8;
typedef signed char    INT8;
typedef signed short   INT16;
typedef unsigned int   UINT32;
typedef signed int     INT32;
typedef unsigned char  BYTE;

#define PALET_BLEU  1
#define PALET_ROUGE 2
#define PALET_VERT  3
#define RIGHT       4                           // Coté droit
#define LEFT        5                           // Coté gauche
#define FRONT       6                           // Milieu (devant)

ZumoMotors motors;                              // Instanciation de l'objet moteur gérant les déplacement du moteur
Pushbutton button(ZUMO_BUTTON);                 // Boutton situé sur la carte ZumoShield 
Pixy pixy;                                      // Objet gérant la caméra Pixy
   
INT32  MOTOR_s32MotorSpeed = 90;               // Vitesse du moteur
INT32  MOTOR_s32MotorSpeedRotate = 0;           // Vitesse de rotation du moteur pour aller a gauche ou a droite
UINT16 PIXY_u16CameraWidth= 319;                // Taille en longueur de la résolution Pixy
UINT16 PIXY_u16CameraHeight = 199;              // Taille en hauteur de la résolution Pixy
float  f32SpeedRotationRatio = 1.5;             // Coeff Multiplicateur pour calculer la vitesse de rotation en fonction de la position de la allse dans le champ de vision
UINT16 u16MinDistanceObject = 20;               // Le robot s'arret lorsque cette distance le sépare de l'objet cible (cm)
float  f32Distance = -1;                        // Distance récupéré par l'ultrason
UINT8  PIXY_u8LastDetectObject = -1;            // Derniere position connue de l'objet avant qu'il ne disparaisse du champ de vision   
bool   bObstacleIsFounded = false;              // Booléen servant a savoir si un obstacle est en face de nous ou pas
bool   bSearchDistanceMin = true;               // Booléen servant a activer ou non la recherche du palet le plus proche
UINT8  u8foundedSignature[100];                 // Tableau qui contient les signatures trouvés lors de la recherche du palet le plus proche
int nbSignatureFounded = 0;                     // Nombre de signature trouvée lors de la recherche du palet le plus proche
bool  bObjectIsInFront = false;
UINT16 PIXY_au16PixyMiddleInterval[2] =         // Interval dans lequel l'objet cible est considéré comme étant au milieu de champ de vision
            { (PIXY_u16CameraWidth/2) + 30 ,    
            (PIXY_u16CameraWidth/2) - 30 };

const byte ULTRASON_TRIGGER_PIN = A1;           // Broche TRIGGER de l'ultrason
const byte ULTRASON_ECHO_PIN = A4;               // Broche ECHO de l'ultrason
const unsigned long MEASURE_TIMEOUT = 25000UL;  // Timeout de l'ulstrason, 25ms = ~8m à 340m/s
const float ULTRASON_SOUND_SPEED = 340.0/1000;  // Vitesse du son dans l'air pour l'ultrason

void setup() 
{
  Serial.begin(9600);
  Serial.print("Starting...\n");
  
  pixy.init();
  ULTRASON_vdInit();
  MOTOR_vdStop(-1);

  memset(u8foundedSignature, 0, 100);
}

void ULTRASON_vdInit()
{
    pinMode(ULTRASON_TRIGGER_PIN, OUTPUT);
    digitalWrite(ULTRASON_TRIGGER_PIN, LOW);
    pinMode(ULTRASON_ECHO_PIN, INPUT);
}


float ULTRASON_f32GetDistance()
{
    digitalWrite(ULTRASON_TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(ULTRASON_TRIGGER_PIN, LOW);

    long measure = pulseIn(ULTRASON_ECHO_PIN, HIGH, MEASURE_TIMEOUT);
    
    float distance_mm = measure / 2.0 * ULTRASON_SOUND_SPEED;

    return distance_mm / 10.0;
}

INT32 s32GetSpeedRotation(UINT16 u16objectX)
{
   INT32 s32Res = (PIXY_u16CameraWidth / 2) - u16objectX;
   s32Res = abs(s32Res) * f32SpeedRotationRatio;
   
   if(s32Res > PIXY_u16CameraWidth * 2)
    return 0;
     
   return s32Res;
}

void MOTOR_vdStop(INT32 u32Delay)
{
   motors.setSpeeds(0,0);

   if(u32Delay < 0)
      return;
      
   delay(u32Delay);
}

//void MOTOR_vdMoveForward(INT32 speedLeft, INT32 speedRight) { motors.setSpeeds(speedLeft , speedRight);  }
void MOTOR_vdMoveForward(INT32 speed)   { motors.setSpeeds(  speed ,   speed);  }
void MOTOR_vdRotateRight(INT32 speed)   { motors.setSpeeds(  speed  , -speed);  }
void MOTOR_vdRotateLeft(INT32 speed)    { motors.setSpeeds( -speed  ,  speed ); }
void MOTOR_vdMoveBack(INT32 speed)      { motors.setSpeeds( -speed  , -speed);  }


void vdTrackObject(UINT16 u16objectX, UINT16 u16objectY)
{
  //Si l'objet se trouve dans le coin gauche du champ de vision de la caméra, on effectue une rotation à gauche
  if(u16objectX <= PIXY_au16PixyMiddleInterval[0] && u16objectX <= PIXY_au16PixyMiddleInterval[1])
  {
    MOTOR_vdRotateLeft(MOTOR_s32MotorSpeedRotate);
  }
  
  //Si l'objet se trouve dans le coin droit du champ de vision de la caméra, on effectue une rotation à droite
  else if(u16objectX >= PIXY_au16PixyMiddleInterval[0] && u16objectX >= PIXY_au16PixyMiddleInterval[1])
  {
    MOTOR_vdRotateRight(MOTOR_s32MotorSpeedRotate);
   
  }

  //Si aucun obstacle n'est façe à nous, et que l'objet se trouve au milieu du champ de vision de la caméra, on récupere sa distance
  else
  {
    // Si l'objet est loin devant nous on peut avancer
    if(f32Distance > u16MinDistanceObject && f32Distance != 0)
    {       
      MOTOR_vdMoveForward(MOTOR_s32MotorSpeed * 2 );
    }
    
    // Si l'objet est trop près de nous on doit reculer
    else if(f32Distance < u16MinDistanceObject && f32Distance > u16MinDistanceObject - 5 && f32Distance != 0)
      MOTOR_vdMoveBack(MOTOR_s32MotorSpeed); 
      
    // Sinon, si l'objet est bien devant nous dans l'intervalle donné, on arrête le moteur
    else
      MOTOR_vdStop(-1);      
  }
}

bool bSignatureWasFounded(int signature, int nbSig)
{
  int i;
  
  if(nbSig == 0)
  return false;
  
  for(i=0 ; i<nbSig ; i++)
    if(u8foundedSignature[i] == signature)
      return true;

   return false;
}

unsigned char u8ChooseNearObject(UINT8 *u8TabSignature, UINT8 u8Size)
{
  int i;
  int min = u8TabSignature[0];
  
  for(i=1 ; i<u8Size ; i++)
  {
    if(min < u8TabSignature[i])
      min = i;
  }

  return min;
}

void loop() 
{
  uint16_t blocks;
  char buf[32]; 
  int signature;
  static int time_now = 0;
  int u8NearObject = 0;

  
  //Récuperer le nombre de blocks détécté
  blocks = pixy.getBlocks();

  if(bSearchDistanceMin == true && bObjectIsInFront == false)
    MOTOR_vdRotateRight(MOTOR_s32MotorSpeed);

  //Si il y a des blocks, on les traites
  if(blocks == true)
  {
    for (int j=0; j<blocks; j++)
    {
      if(bSearchDistanceMin == true && bObjectIsInFront == false)
      {  
        signature = pixy.blocks[j].signature;

          // Si l'on est déja tombé sur cette signature, et que l'on retombe dessus, alors c'est qu'on a fait un tour complet.
          /*
          if(signature == u8foundedSignature[0])
          {
            u8NearObject = u8ChooseNearObject(u8foundedSignature, nbSignatureFounded);
            Serial.print("Le palet le plus proche est : ");
            if(u8NearObject == PALET_BLEU) Serial.println("le BLEU");
            else if(u8NearObject == PALET_ROUGE) Serial.println("le ROUGE");
            else if(u8NearObject == PALET_VERT) Serial.println("le VERT");
            bSearchDistanceMin = false;

            break;
          }
          */
        if((signature == PALET_BLEU || signature == PALET_VERT || signature == PALET_ROUGE) && bSignatureWasFounded(signature,nbSignatureFounded) == false)
        {
     
          // Sinon, on continue la récupération des distances de chaques palets
          
            int u16objectX = pixy.blocks[j].x;
            int u16objectWidth = pixy.blocks[j].width;
            if(u16objectX <= PIXY_au16PixyMiddleInterval[0] && u16objectX >= PIXY_au16PixyMiddleInterval[1])
            {
              bObjectIsInFront = true;
              MOTOR_vdStop(-1);
              time_now = millis();
            }
            break;
        }
      }
      
      // Rehcerche du palet le plus proche terminée, on avance jusqu'a lui
      else if(bSearchDistanceMin == false && bObjectIsInFront == false)
      {
        if(signature == u8NearObject)
        {
          int u16objectX = pixy.blocks[j].x;
          int u16objectY = pixy.blocks[j].y;
          vdTrackObject(u16objectX, u16objectY);
        }
        else
          MOTOR_vdRotateRight(MOTOR_s32MotorSpeed);
      }
    }
  }    
  
  //Ce bloc est non bloquant, il sert a recuperer la distance dès que l'on est devant un palet
  if(bObjectIsInFront == true)
  {
    if(millis() > (time_now) + 1000)
    {
    
      if(millis() > (time_now) + 2000)
      {
        bObjectIsInFront = false;
      }
      
      f32Distance = ULTRASON_f32GetDistance();
     
      if(signature == PALET_BLEU) Serial.print("PALET BLEU détécté, distance = ");
      else if(signature == PALET_ROUGE) Serial.print("PALET ROUGE détécté, distance = ");
      else if(signature == PALET_VERT) Serial.print("PALET VERT détécté, distance = ");
      Serial.print(f32Distance);
      Serial.println(" cm");
      
      u8foundedSignature[nbSignatureFounded++] = signature;
    }
  }

  /*
  f32Distance = ULTRASON_f32GetDistance();
              Serial.print("Objet Trouvé, distance = ");
              Serial.println(f32Distance);
  */
  delay(30);
}
