/*
 * Software to controll the motors of the UMI-RTX robot using multiple H-bidge IC's.
 * The software runs on an Arduino mega.
 * new version V0.6
 * 18/1/2018
 */
 
 /*
  * Changes in this ref:
  * - added basic movements.
  * - added motor stop at maximum.
  * - found a possible way to fix the yaw not properly following the elbow.
  * - added the option to set the yaw to follow the elbow.
  * - added serial communication to the vision host computer
  * 
  * Things that need to be added:
  * - zed encoder maximum needs to be changed.
  * - need to test everything.
  * - Think about restrictions of the robot. when starting from back a bit, then moving left then forwards. this causes problems. maybe the old software can calculate this.
  * - Exeption list with forbidden position. Motor might hit it's self if not implemented.
  * - 
  *///
//git testje
//____________________________________________DEFINES________________________________________________
//Direction defines
#define RIGHT 2
#define LEFT  1
#define STOP  0

#define DEBUG
//#define TEST_POSITIONS
#define DEMO


#define SHOULDER //works
#define ZED// not tested
//#define WRIST_RH//works
//#define WRIST_LH//works
#define ELBOW// works
#define YAW //works
//#define WRIST
//toggles checking for power on.
//#define VOLTAGE

//way to test of the yaw following the elbow works.
#define FOLLOW_YAW

//this tests is the yaw follows the elbow properly.
#define TEST_YAW

//stops the motor when hitting the maximum encoder count.
//#define MAXIMUM_LIMIT

//toggeling limits the movement the robot can make whem moving forward or backwards. 
//#define RESTRICK_MOVEMENT

//way to test if the serial connection works.
#define SERIAL_TEST
#define biem
//pin to measure if power is present.
#define MEAS_VOLTAGE 14

//toggle to use basic or advanced movements. L-R basic is only yaw, advanced is whole robot.
#define BASIC_MOVEMENT

///////////////////
//maximum rage or motors: NUMBERS NEED TO BE CHECKED AND IMPLEMENTED!
#define SHOULDER_MAX_POS      2630 //correct   //5260
#define ELBOW_MAX_POS         2418//correct   //4836 2418
#define ZED_MAX_POS           1196   //2392
#define WRIST_LH_MAX_POS      1220//2142 //1071
#define WRIST_RH_MAX_POS      1220//8442 // 44221
#define WRIST_MIN_POS        -1220
#define YAW_MAX_POS           1100//correct //2142
#define GRIPPER_MAX           1200
////////////////////////

//Motor pins need to change to real layout
#define SHOULDER_PLUS       22
#define SHOULDER_MIN        23
#define SHOULDER_ENABLE     4

#define ELBOW_PLUS          24
#define ELBOW_MIN           25
#define ELBOW_ENABLE        5

#define ZED_PLUS            26
#define ZED_MIN             27
#define ZED_ENABLE          6

#define WRIST_RH_PLUS       28
#define WRIST_RH_MIN        29
#define WRIST_RH_ENABLE     7

#define WRIST_LH_PLUS       30
#define WRIST_LH_MIN        31
#define WRIST_LH_ENABLE     8

#define YAW_PLUS            32
#define YAW_MIN             33
#define YAW_ENABLE          9

/////////////////////////////////////////////////////

//Encoder interrupt pins
#define SHOULDER_ENCODER          2
#define ELBOW_ENCODER             3
#define ZED_ENCODER               18
#define WRIST_RH_ENCODER          19
#define WRIST_LH_ENCODER          20
#define YAW_ENCODER               21
///////////////////////////////////////////////////////

//_____________________________________DEG_to_RAD_to_DEG______________________________________________

float L1 = 252.5;
float L2 = 252.5;
float L3 = 253;
int L_arm = 758;
float rad;
float deg;

//functie voor graden naar radialen
float degtorad(float deg){
  float rad= deg*(PI/180); //0.017453292519943295769236907684886;
  return rad;
} 

//functie voor radialen naar graden
float radtodeg(float rad){
  float deg= rad*(180/PI);//57.295779513082320876798154814105;
  return deg;
}

//____________________________________________VARIABLES________________________________________________

//Encoder counters
static volatile int shoulderEncoder = 0;
static volatile int elbowEncoder = 0;
static volatile int zedEncoder = 0;
static volatile int wristRHEncoder = 0;
static volatile int wristLHEncoder = 0;
static volatile int yawEncoder = 0;
////////////////////////

// Target positions per axis
int shoulderTarget; //= 0;
int elbowTarget;// = 0;
int zedTarget;// = 0;
int wristRHTarget;// = 0;
int wristLHTarget;// = 0;
int yawTarget;// = 0;
///////////////////////////

//Move direction per motor
int elbowDirection = 0;
int shoulderDirection = 0;
int zedDirection = 0;
int wristRHDirection = 0;
int wristLHDirection = 0;
int yawDirection = 0;
////////////////////


//////////////

//init variables aka the initVar's
int initShoulder = 0;
int initZed = 0;
int initWristRH = 0;
int initWristLH = 0;
int initElbow = 0;
int initYaw = 0;
///////////////////////////////////////

//compensation position for the yaw
static int yawCounter = 0;
static volatile int yawEncoderCounter = 0;
boolean yawFollowElbow = false;  
boolean yawMiddle = false;
boolean movingForward = false;
boolean basicMovement = false;

//position queues for all motors. used in demo mode
int shoulderPositionQueue[5] = {SHOULDER_MAX_POS/2, SHOULDER_MAX_POS/2, 0, 1300, SHOULDER_MAX_POS/2};
int elbowPositionQueue[5] = {ELBOW_MAX_POS/2, ELBOW_MAX_POS/2, ELBOW_MAX_POS/2, ELBOW_MAX_POS/2, ELBOW_MAX_POS/2};
int zedPositionQueue[5] = {ZED_MAX_POS/2, 400,ZED_MAX_POS/2, 400, ZED_MAX_POS/2};
int yawPositionQueue[5] = {YAW_MAX_POS/2, YAW_MAX_POS, YAW_MAX_POS/2, 0, YAW_MAX_POS/2};

//____________________________________________calculations_______________________________________________


//**************************************************************************
// mega functie die bijna alle hoeken en encoder count uitrekend.
// De zed wordt hier niet uitgerekend en wordt ook nog geen rekening gehouden met huidige posities
//**************************************************************************
void *berekening(float *phi,int *x,int *z, int *y,int *encoderS, int *encoderE,int *encoderY,int *encoderW1, int *encoderW2, int *encoderZ)
{
  float L12;
  float xcor;
  float zcor;
  float xpols;
  float zpols;
  float c2;
  float Theta1; //dit is de hoek van de schouder bereik 0-180graden
  float Theta2; //dit is de hoek van de elleboog bereik 0-331graden
  float Theta10;
  float Theta11;
  float Theta3; //dit is de hoek van de pols bereik 70-290graden
  float z1;
  float x1;
  xcor = L3*cos(degtorad(*phi));//berekend x verplaatsing vanaf cordinaat
  zcor = L3*sin(degtorad(*phi));//berekend z verplaatsing vanaf cordinaat
  xpols = *x - xcor;                         //X-positie wrist motor
  zpols = *z - zcor;                         //Z-positie wrist motor
  c2 = (sq(abs(xpols)) + sq(abs(zpols))); 
  L12 = sqrt(c2);       //lengte tussen oorsprong en pols cordinaten                   
  
//***********************************************************************************************************
// hier worden de hoeken van de motoren berekend links van het midden dus met de elleboog naar links
//***********************************************************************************************************
 if (xpols<0){
    int lastTheta1= Theta1; //onthoud vorige theta
    int lastTheta2= Theta2; //onthoud vorige theta
    int lastTheta3= Theta3; //onthoud vorige theta
    Theta10= radtodeg(asin(abs(xpols/L12))); //hoek tussen x-as en loodlijn tussen schouder en pols
    Theta11= radtodeg(acos((L12/2)/L1));// hoek tussen loodlijn en schouder segment
    Theta1= Theta10+Theta11; //Totaal Theta 1
    Theta2= 2*radtodeg(asin((L12/2)/L1));      //hoek van elleboog
    Theta3 = 360-(Theta1 + Theta2) + *phi; //Bij het uit tekenen van de armen tot deze formule gekomen
    
      // volgende if statement zorgt dat het berijk van het gewricht niet word overschreden maar zet m tegen de maximale waarde
    if (Theta1<0 || Theta1>180){
      Serial.print("THETA1 out of range");
      Serial.println(Theta1);
      *encoderS = round((Theta1/0.03422)/2);
      if (Theta1<0)Theta1=lastTheta1;return; //zet oude waarde terug en springt uit de functie
      if (Theta1>180)Theta1=lastTheta1;return; //zet oude waarde terug en springt uit de functie
    } 
      // volgende if statement zorgt dat het berijk van het gewricht niet word overschreden maar zet m tegen de maximale waarde
    if (Theta2<0 || Theta2>331){
      Serial.print("THETA2 out of range");
      Serial.println(Theta2);
      if (Theta2<0)Theta2=lastTheta2;return; //zet oude waarde terug en springt uit de functie
      if (Theta2>331)Theta2=lastTheta2;return; //zet oude waarde terug en springt uit de functie
    }
    // volgende if statement zorgt dat het berijk van het gewricht niet word overschreden maar zet m tegen de maximale waarde
    if (Theta3<70 || Theta3>290){
      Serial.print("THETA3 out of range");
      Serial.println(Theta3);
      if (Theta3<70)Theta3=lastTheta3;return; //zet oude waarde terug en springt uit de functie
      if (Theta3>290)Theta3=lastTheta3;return; //zet oude waarde terug en springt uit de functie
    }   
  }
//***********************************************************************************************************
// hier worden de hoeken van de motoren berekend voor de rechterkant van de arm, met de elleboog naar rechts.
//***********************************************************************************************************
  else if (xpols>=0){ 
    int lastTheta1= Theta1; //onthoud vorige theta
    int lastTheta2= Theta2; //onthoud vorige theta
    int lastTheta3= Theta3; //onthoud vorige theta
    Theta10= radtodeg(asin(zpols/L12));// hoek tussen x-as en loodlijn
    Theta11= radtodeg(acos((L12/2)/L1)); // hoek tussen schouder seg en loodlijn
    Theta1= abs(Theta10) -abs(Theta11); // hoek tussen x-as en schouder seg
    Theta2 = 360-(180-(2*Theta11)); //Bij het uit tekenen van de armen tot deze formule gekomen
    Theta3 = 180-(2*Theta11 +Theta1)+*phi;//Bij het uit tekenen van de armen tot deze formule gekomen
    // volgende if statement zorgt dat het berijk van het gewricht niet word overschreden maar zet m tegen de maximale waarde
    if (Theta1<0 || Theta1>180){
      Serial.print("THETA1 out of range");
      Serial.println(Theta1);
      *encoderS = round((Theta1/0.03422)/2);
      if (Theta1<0)Theta1=lastTheta1;return; //zet oude waarde terug en springt uit de functie
      if (Theta1>180)Theta1=lastTheta1;return; //zet oude waarde terug en springt uit de functie
    }
      // volgende if statement zorgt dat het berijk van het gewricht niet word overschreden maar zet m tegen de maximale waarde
    if (Theta2<0 || Theta2>331){
      Serial.print("THETA2 out of range");
      Serial.println(Theta2);
      if (Theta2<0)Theta2=lastTheta2;return; //zet oude waarde terug en springt uit de functie
      if (Theta2>331)Theta2=lastTheta2;return; //zet oude waarde terug en springt uit de functie
    }
    if (Theta3<70 || Theta3>290){
      Serial.print("THETA3 out of range");
      Serial.println(Theta3);
      if (Theta3<70)Theta3=lastTheta3;return; //zet oude waarde terug en springt uit de functie
      if (Theta3>290)Theta3=lastTheta3;return; //zet oude waarde terug en springt uit de functie
    }   
  }
  
  float yc = *y;// berekening van Zed eerst even data omzetten, c staat voor conversion
  *encoderZ = round((yc/0.2667)/2);//encoder van Zed uitrekenen
  //*encoderW1= coil_rot(*rot);//functie die de encoders van pols rotatie uitrekenen.
  *encoderW2= -1*(*encoderW1);

  if(L12<=505){  
    *encoderS = round((Theta1/0.03422)/2);
    *encoderE = round((Theta2/0.06844)/2);
    *encoderY = round((Theta3/0.10267)/2);
    #ifdef DEBUG
    Serial.print("L12:");Serial.println(L12);
    Serial.print("zpols:");Serial.println(zpols);
    Serial.print("xpols:");Serial.println(xpols);
    Serial.print("theta10:");Serial.println(Theta10);
    Serial.print("theta11:");Serial.println(Theta11);
    Serial.print("theta1:");Serial.println(Theta1);
    Serial.print("theta2:");Serial.println(Theta2);
    Serial.print("theta3:");Serial.println(Theta3);
    #endif
 
    Serial.print("Shouder:");
    Serial.println(*encoderS);
    Serial.print("Elleboog");
    Serial.println(*encoderE);
    Serial.print("Pols: ");
    Serial.println(*encoderY);
    Serial.print("ZED: ");
    Serial.println(*encoderZ);
  }
  else Serial.println("coordinaten buiten bereik");
  return;
}

//________________________OVERIGE_FUNCTIES______________________________________

int coil_rot(int rot){
  int encoder;
  encoder= ((2*rot)/0.07415)/2;
  return encoder;
}

int retrieve_x(){
  Serial.print("voer X coordinaat in tussen de -600 en 600:");
  while(Serial.available()==0);
  int x= Serial.parseInt();
  Serial.println(x);
  while (x<-600 || x>600){
    Serial.println("buiten bereik pannekoek, LEZEN!");
    Serial.print("voer X coordinaat in tussen de -600 en 600:");
    while(Serial.available()==0);
    x= Serial.parseInt();
    Serial.println(x);
  }
  return x;  
}

int retrieve_z(){
  Serial.print("voer Z coordinaat in tussen de 0 en 700:");
  while(Serial.available()==0);
  int z= Serial.parseInt();
  Serial.println(z);
  while (z<0 || z>700){
    Serial.println("buiten bereik pannekoek, LEZEN!");
    Serial.print("voer Z coordinaat in tussen de 0 en 700:");
    while(Serial.available()==0);
    z= Serial.parseInt();
    Serial.println(z);
  }  
  return z;
}

int retrieve_y(){
  Serial.print("voer y coordinaat in tussen de 0 en 915:");
  while(Serial.available()==0);
  int y= Serial.parseInt();
  Serial.println(y);
  while (y<0 || y>915){
    Serial.println("buiten bereik pannekoek, LEZEN!");
    Serial.print("voer y coordinaat in tussen de 0 en 915:");
    while(Serial.available()==0);
    y= Serial.parseInt();
    Serial.println(y);
  }  
  return y;
}

float retrieve_phi(){
  Serial.print("voer phi in tussen 0 en 180 graden:");
  while(Serial.available()==0);
  float phi= Serial.parseInt();
  Serial.println(phi);
  while (phi<0 || phi>180){
    Serial.println("buiten bereik pannekoek, LEZEN!");
    Serial.print("voer phi in tussen 0 en 180 graden:");
    while(Serial.available()==0);
    phi= Serial.parseInt();
    Serial.println(phi);
  }  
  return phi;
}

int retrieve_rot(){
  Serial.print("voer rot graden in tussen de 181 en -132:");
  while(Serial.available()==0);
  int rot= Serial.parseInt();
  Serial.println(rot);
  while (rot<-132 || rot>181){
    Serial.println("buiten bereik pannekoek, LEZEN!");
    Serial.print("voer rot graden in tussen de -132 en 181:");
    while(Serial.available()==0);
    rot= Serial.parseInt();
    Serial.println(rot);
  }  
  return rot;
}

//____________________________________________MOTOR FUNCTIONS________________________________________________


//Motor controll functions

//the wrist has to be initialised in a diffrent way. the two motors, LH and RH, need to work together to find the right position.

//inits the provided motor by moving it till the encoders stop counting.
//demo's a few positions
void demoMode()
{
  for(int i = 0; i< 5; i++)
    {
        moveToPos(&zedTarget, zedPositionQueue[i], &zedEncoder, ZED_PLUS, ZED_MIN, ZED_ENABLE, 300);
        moveToPos(&shoulderTarget, shoulderPositionQueue[i], &shoulderEncoder, SHOULDER_PLUS, SHOULDER_MIN, SHOULDER_ENABLE, 130);//200 //130 works fine
        moveToPos(&elbowTarget, elbowPositionQueue[i], &elbowEncoder, ELBOW_PLUS, ELBOW_MIN ,ELBOW_ENABLE, 133);
        moveToPos(&yawTarget,yawPositionQueue[i], &yawEncoder, YAW_PLUS, YAW_MIN, YAW_ENABLE, 65);// 60
        Serial.print("Position: ");
        Serial.println(i);

        delay(7000);
        Serial.print("ZED encoder count = ");
        Serial.println(getEncoderCount(ZED_PLUS));
        Serial.print("YAW encoder count = ");
        Serial.println(getEncoderCount(YAW_PLUS));
        Serial.print("Shoulder Encoder Count = ");
        Serial.println(getEncoderCount(SHOULDER_PLUS));
        Serial.print("Elbow encoder count = ");
        Serial.println(getEncoderCount(ELBOW_PLUS));
     }
}
//sets the motor so it moves to a given position
void moveToPos(int *target, int targetPosition, volatile int *encoder, int plus, int minus, int enable, int moveSpeed)
{
     basicMovement = false;
//  if(plus == WRIST_RH_PLUS)
//    *target = targetPosition * -1;
//  else
    #ifdef YAW_TEST
    if(plus == YAW_MOTOR_PLUS)
      *target = targetPostion + yawCounter;
    #else
    *target = targetPosition;
    #endif
  if(*target > *encoder)
    {
       Serial.println("RIGHT");
       setDirection(plus, RIGHT);
       analogWrite(enable, moveSpeed);
       digitalWrite(plus,  HIGH);
       digitalWrite(minus, LOW);
    }
    else if(*target < *encoder)
    {
      Serial.println("LEFT");
      setDirection(plus, LEFT);
      analogWrite(enable, moveSpeed);
      digitalWrite(plus,  LOW);
      digitalWrite(minus, HIGH);
    }
//    else
//      Serial.println("Position Error");
}
//maybe functions to set direction. This is ugly AF.

//moves motor for a given time
void moveMotor(int moveTime,int moveSpeed, int movementDirection, int plus, int minus, int enable)
{
  setDirection(plus, movementDirection);
  analogWrite(enable, moveSpeed);
  if(movementDirection == 2)
    {
      digitalWrite(plus, HIGH);
      digitalWrite(minus, LOW);
    }
    else if(movementDirection == 1)
    {
      digitalWrite(plus, LOW);
      digitalWrite(minus, HIGH);
      
    }
  delay(moveTime);
  digitalWrite(plus, LOW);
  digitalWrite(minus, LOW);
  analogWrite(enable, 0);
 }
//sets direction the motor is moving to appropriate value
void setDirection(int motorPlus,int moveDirection)
{
   switch(motorPlus)
  {
   case SHOULDER_PLUS:
    shoulderDirection = moveDirection;  
    break;
   case ELBOW_PLUS:
    elbowDirection = moveDirection;  
    break;
   case WRIST_RH_PLUS:
    wristRHDirection = moveDirection;  
    break;
   case WRIST_LH_PLUS:
    wristLHDirection = moveDirection;  
    break;
   case ZED_PLUS:
    zedDirection = moveDirection;  
    break;
   case YAW_PLUS:
    yawDirection = moveDirection;  
    break;
   default:
    Serial.println("error on direction");
    break;
   } 
}




//____________________________________________ENCODERS_______________________________________________




//function to get the encoder count of a given motor. function only needs to be used if the value can not be called upon through a pointer.
int getEncoderCount(int motorPlus)
{
    switch(motorPlus)
        {
         case SHOULDER_PLUS:
          return shoulderEncoder;  
          break;
         case ELBOW_PLUS:
          return elbowEncoder;  
          break;
         case WRIST_RH_PLUS:
          return wristRHEncoder;  
          break;
         case WRIST_LH_PLUS:
          return wristLHEncoder;  
          break;
         case ZED_PLUS:
          return zedEncoder;  
          break;
         case YAW_PLUS:
          return yawEncoder;  
          break;
         default:
          Serial.println("encoder count problem");
          return -1;
          break;
         } 
}
//sets the selected encoder to a value.
void setEncoderCount(int motorPlus, int value)
{
    switch(motorPlus)
        {
         case SHOULDER_PLUS:
            shoulderEncoder = value;  
          break;
         case ELBOW_PLUS:
          elbowEncoder = value;  
          break;
         case WRIST_RH_PLUS:
          wristRHEncoder = value;  
          break;
         case WRIST_LH_PLUS:
          wristLHEncoder = value;  
          break;
         case ZED_PLUS:
          zedEncoder = value;  
          break;
         case YAW_PLUS:
          yawEncoder = value;  
          break;
         default:
          Serial.println("encoder set problem");
          break;
         } 
}



//____________________________________________INIT_______________________________________________





//sets the init value of the motor to true or false. This is done for the interrupt functions of the motor.
void setInit(int motorPlus, int stat)
{
    switch(motorPlus)
      {
       case SHOULDER_PLUS:
        shoulderEncoder = stat;  
        break;
       case ELBOW_PLUS:
        elbowEncoder = stat;  
        break;
       case WRIST_RH_PLUS:
        wristRHEncoder = stat;  
        break;
       case WRIST_LH_PLUS:
        wristLHEncoder = stat;  
        break;
       case ZED_PLUS:
        zedEncoder = stat;  
        break;
       case YAW_PLUS:
        yawEncoder = stat;  
        break;
       default:
        Serial.println("setup Error");
        break;
       } 
}
void testWrist()
{
  analogWrite(WRIST_LH_ENABLE, 255);
  digitalWrite(WRIST_LH_PLUS, LOW);
  digitalWrite(WRIST_LH_MIN, HIGH);
  analogWrite(WRIST_RH_ENABLE, 255);
  digitalWrite(WRIST_RH_PLUS, HIGH);
  digitalWrite(WRIST_RH_MIN, LOW);
  Serial.println("hond");
  delay(1500);
  exit(0);
//  analogWrite(WRIST_LH_ENABLE, 0);
//  digitalWrite(WRIST_LH_PLUS, LOW);
//  digitalWrite(WRIST_LH_MIN, LOW);
//  analogWrite(WRIST_RH_ENABLE, 0);
//  digitalWrite(WRIST_RH_PLUS, LOW);
//  digitalWrite(WRIST_RH_MIN, LOW);
  
  
//____________________________________________SETUP________________________________________________


}
void setup() {
  //setting pinmodes for motors
  for(int i = 4; i < 10; i++)
    pinMode(i, OUTPUT);
  //setting pinmodes for enables
  for(int i = 22; i < 34; i++)
     pinMode(i, OUTPUT);
  //setting pinmodes for encoder, pull-up for transistors in encoders
  pinMode(SHOULDER_ENCODER, INPUT_PULLUP);
  pinMode(ELBOW_ENCODER,    INPUT_PULLUP);
  pinMode(YAW_ENCODER,      INPUT_PULLUP);
  pinMode(ZED_ENCODER,      INPUT_PULLUP);
  pinMode(WRIST_RH_ENCODER, INPUT_PULLUP);
  pinMode(WRIST_LH_ENCODER, INPUT_PULLUP);
  //setting voltage measure pinmode
  pinMode(MEAS_VOLTAGE, INPUT);
  Serial.begin(9600);//might need to talk faster with host.
  
  //Attaching interrupts for encoders
  attachInterrupt(digitalPinToInterrupt(SHOULDER_ENCODER), readShoulderEncoder, RISING);//CHANGE
  attachInterrupt(digitalPinToInterrupt(ELBOW_ENCODER),    readElbowEncoder,    RISING);//CHANGE
  attachInterrupt(digitalPinToInterrupt(WRIST_RH_ENCODER), readWristRHEncoder,  RISING);//CHANGE
  attachInterrupt(digitalPinToInterrupt(WRIST_LH_ENCODER), readWristLHEncoder,  RISING);//CHANGE
  #ifdef ZED
  attachInterrupt(digitalPinToInterrupt(ZED_ENCODER),      readZedEncoder,      RISING);//CHANGE
  #endif
  attachInterrupt(digitalPinToInterrupt(YAW_ENCODER),      readYawEncoder,      RISING);//CHANGE

  #ifdef VOLTAGE
  while(digitalRead(MEAS_VOLTAGE)!= HIGH)
  {
    Serial.println("please turn on the power");
    delay(3000);
  }
  #endif


  //initAllMotors(); //init


}



//____________________________________________LOOP________________________________________________



void loop() {
// put your main code here, to run repeatedly:
 testWrist();
  #ifdef DEMO
moveToPos(&zedTarget, ZED_MAX_POS/2, &zedEncoder, ZED_PLUS, ZED_MIN, ZED_ENABLE, 250);
moveToPos(&shoulderTarget, 300, &shoulderEncoder, SHOULDER_PLUS, SHOULDER_MIN, SHOULDER_ENABLE, 130);//200 //130 works fine
delay(100);
moveToPos(&elbowTarget, ELBOW_MAX_POS/2, &elbowEncoder, ELBOW_PLUS, ELBOW_MIN ,ELBOW_ENABLE, 133);
delay(100);
moveToPos(&yawTarget, (YAW_MAX_POS), &yawEncoder, YAW_PLUS, YAW_MIN, YAW_ENABLE, 65);// 60
delay(7000);
moveToPos(&shoulderTarget, SHOULDER_MAX_POS, &shoulderEncoder, SHOULDER_PLUS, SHOULDER_MIN, SHOULDER_ENABLE, 130);//200 //130 works fine
delay(100);
moveToPos(&elbowTarget, ELBOW_MAX_POS/2, &elbowEncoder, ELBOW_PLUS, ELBOW_MIN ,ELBOW_ENABLE, 133);
delay(100);
moveToPos(&yawTarget, (YAW_MAX_POS/2), &yawEncoder, YAW_PLUS, YAW_MIN, YAW_ENABLE, 65);// 60
delay(7000);
moveToPos(&shoulderTarget, SHOULDER_MAX_POS/2, &shoulderEncoder, SHOULDER_PLUS, SHOULDER_MIN, SHOULDER_ENABLE, 130);//200 //130 works fine
delay(100);
moveToPos(&elbowTarget, ELBOW_MAX_POS/2, &elbowEncoder, ELBOW_PLUS, ELBOW_MIN ,ELBOW_ENABLE, 133);
delay(100);
moveToPos(&yawTarget, (YAW_MAX_POS/2), &yawEncoder, YAW_PLUS, YAW_MIN, YAW_ENABLE, 65);// 60
delay(7000);
demoMode();
exit(0);
moveMotor(1000,255, LEFT, SHOULDER_PLUS, SHOULDER_MIN, SHOULDER_ENABLE);
Serial.println(shoulderEncoder);
Serial.println(shoulderTarget);
Serial.println(shoulderEncoder);
moveToPos(&zedTarget, ZED_MAX_POS/2, &zedEncoder, ZED_PLUS, ZED_MIN, ZED_ENABLE, 250);
moveToPos(&wristLHTarget, 0, &wristLHEncoder, WRIST_LH_PLUS, WRIST_LH_MIN, WRIST_LH_ENABLE, 250);
moveToPos(&wristRHTarget, 0, &wristRHEncoder, WRIST_RH_PLUS, WRIST_RH_MIN, WRIST_RH_ENABLE, 250);
delay(400);
stopAllMotors();
delay(5000);
  #endif


//____________________________________________________XYZ Implementatie______________________________________________
  

 

  #ifdef biem 
  char keyboardInput = Serial.read();
  if (keyboardInput == 'X') 
  {
    static int room_S,room_E,room_Y,encoderS, encoderE,encoderY,encoderW1,encoderW2,encoderZ; //S=shoulder, E=elbow, W=wrist(rot=rotation),Z=zed(y-as)
    static float rot =0;
    int x   = retrieve_x();
    int z   = retrieve_z();
    int y   = retrieve_y();
    static float phi =10;
    berekening (&phi, &x, &z, &y, &encoderS, &encoderE, &encoderY, &encoderW1, &encoderW2, &encoderZ);
  }

  #endif



  #ifdef SERIAL_TEST
  readData();
  #endif


//   #ifdef TEST_POSITIONS
// while(Serial.available()==0);
// int y= Serial.parseInt();
// Serial.println(y);
//      moveToPos(&shoulderTarget, y, &shoulderEncoder, SHOULDER_PLUS, SHOULDER_MIN, SHOULDER_ENABLE, 200); //200
// while(Serial.available()==0);
// int x= Serial.parseInt();
// Serial.println(x);
//      moveToPos(&elbowTarget, x, &elbowEncoder, ELBOW_PLUS, ELBOW_MIN, ELBOW_ENABLE, 200); 
//      while(Serial.available()==0);
// int z= Serial.parseInt();
// Serial.println(z);
//      moveToPos(&yawTarget, z, &yawEncoder, YAW_PLUS, YAW_MIN, YAW_ENABLE, 200); 
//   #endif 


}



