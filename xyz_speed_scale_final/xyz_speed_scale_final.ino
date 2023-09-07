/**********************************************************************************************************************************************************************/
#include <Servo.h>
Servo servo[6];
int speed=250;
/**********************************************************************************************************************************************************************/
//MINIMUM and MAXIMUM PWM pulse sizes
#define MAXI 2200
#define MINI 800
/**********************************************************************************************************************************************************************/
//Defining Positions of servos mounted in opposite direction
#define Opp1 1
#define Opp2 3
#define Opp3 5
/**********************************************************************************************************************************************************************/
//constants for calculations of positions of connection points
#define pii  3.14159
#define degTOrad 180/pii
#define thirtydeg pii/6
/**********************************************************************************************************************************************************************/
//base positions of servos, in this positions servo arms are perfectly horizontal, in microseconds (us)
//static int base[6]={1475,1470,1490,1480,1460,1490};
static int base[6]={1500,1500,1500,1500,1500,1500};
/**********************************************************************************************************************************************************************/
// Intial position for platform - x,y,z,rot(x),rot(y),rot(z)
static float initial[6]={0.0,0.0,0.0, radians(0), radians(0), radians(0)};
/**********************************************************************************************************************************************************************/
float prev[6],curr[6];
String val1,val2,val3,val4,val5,val6;
String last[6]={"A","B","C","D","E","F"};
/**********************************************************************************************************************************************************************/
//static float inpX[] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}; 
//static float inpZ[] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};      
//static float inpY[] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};      
//static float inprotX[] = {radians(0),radians(0),radians(0),radians(0),radians(0),radians(0),radians(0),radians(0),radians(0),radians(0),radians(0),radians(0),radians(0),radians(0),radians(0),radians(0)};
//static float inprotY[] = {radians(0),radians(0),radians(0),radians(0),radians(0),radians(0),radians(0),radians(0),radians(0),radians(0),radians(0),radians(0),radians(0),radians(0),radians(0),radians(0)};  
//static float inprotZ[] = {radians(0),radians(0),radians(0),radians(0),radians(0),radians(0),radians(0),radians(0),radians(0),radians(0),radians(0),radians(0),radians(0),radians(0),radians(0),radians(0)};
//int s= sizeof(inpX) / sizeof(inpX[0]);  
/**********************************************************************************************************************************************************************/
//Actual degree of rotation of servo arms, used to reduce complexity of calculating new degree of rotation, (start at 0 - horizontal)
static float theta_a [6]={0.0,0.0,0.0,0.0,0.0,0.0};
/**********************************************************************************************************************************************************************/
//Array of previous servo positions in us
static int servo_pos_prev[6];
/**********************************************************************************************************************************************************************/
//Array of current servo positions in us
static int servo_pos[6];
/**********************************************************************************************************************************************************************/
//rotation of servo arms in respect to axis x
const float beta[] = {pii/2,-pii/2,-pii/6, 5*pii/6,-5*pii/6,pii/6},
//const float beta[] = {pii/2,-pii/2,-5*pii/6, pii/6,-pii/6,5*pii/6},
//maximum servo positions, 0 is horizontal position
servo_min=radians(-80),servo_max=radians(80);
/**********************************************************************************************************************************************************************/
//servo_mult - multiplier used for conversion radians->servo pulse in us
//L1-effective length of servo arm
//L2 - length of base and platform connecting arm
//z_gap - height of platform above base, 0 is height of servo arms
//const float servo_mult=400/(pii/4),L1 = 0.79,L2 = 4.66, z_gap = 4.05;
const float servo_mult=400/(pii/4),L1 = 2.5,L2 = 17.5, z_gap = 16.25;
/**********************************************************************************************************************************************************************/
//AD distance from center of platform to attachment points (arm attachment point)
//XD distance from center of base to center of servo rotation points (servo axis)
//theta_x-angle between two servo axis points, theta_a - between platform attachment points
//const_theta-helper variable
//const float AD = 2.42, XD =2.99, theta_x = radians(37.5), theta_at = radians(8), const_theta=(pii/3-theta_x)/2;
const float AD = 8.593, XD =9.1562, theta_x = radians(41.581), theta_at = radians(10.0145), const_theta=(pii/3-theta_x)/2;
/**********************************************************************************************************************************************************************/
// Inverse Kinematics
//p[][]=x y values for servo rotation points
//re[]{}=x y z values of platform attachment points positions
//equations used for p and re will affect postion of X axis, they can be changed to achieve specific X axis position
const float p[2][6]={
          {
            -XD*cos(thirtydeg-const_theta),-XD*cos(thirtydeg-const_theta),
            XD*sin(const_theta),XD*cos(thirtydeg+const_theta),
            XD*cos(thirtydeg+const_theta),XD*sin(const_theta)
         },
         {
            -XD*sin(thirtydeg-const_theta),XD*sin(thirtydeg-const_theta),
             XD*cos(const_theta),XD*sin(thirtydeg+const_theta),
            -XD*sin(thirtydeg+const_theta),-XD*cos(const_theta)
         }
      },
      re[3][6] = {
          {
              -AD*sin(thirtydeg+theta_at/2),-AD*sin(thirtydeg+theta_at/2),
              -AD*sin(thirtydeg-theta_at/2),AD*cos(theta_at/2),
               AD*cos(theta_at/2),-AD*sin(thirtydeg-theta_at/2),
          },{
              -AD*cos(thirtydeg+theta_at/2),AD*cos(thirtydeg+theta_at/2),
               AD*cos(thirtydeg-theta_at/2),AD*sin(theta_at/2),
              -AD*sin(theta_at/2),-AD*cos(thirtydeg-theta_at/2),
          },{
              0,0,0,0,0,0
          }
};
/**********************************************************************************************************************************************************************/
//arrays used for servo rotation calculation
//H[]-center position of platform can be moved with respect to base, this is
//translation vector representing this move
static float Rot[3][3], posA[3][6], Trans[3], HomeTr[3] = {0,0,z_gap};
/**********************************************************************************************************************************************************************/
/**********************************************************************************************************************************************************************/
void setup(){
//attachment of servos to PWM digital piins of arduino
   servo[0].attach(3, MINI, MAXI);
   servo[1].attach(4, MINI, MAXI);
   servo[2].attach(5, MINI, MAXI);
   servo[3].attach(6, MINI, MAXI);
   servo[4].attach(7, MINI, MAXI);
   servo[5].attach(8, MINI, MAXI);
//begin of serial communication
   Serial.begin(115200);
//putting into base position
   setPosin(initial);
   for(int i=0;i<6;i++) {prev[i]=0; curr[i]=0;}
   delay(1000);
}
/**********************************************************************************************************************************************************************/
/**********************************************************************************************************************************************************************/
//function calculating needed servo rotation value
float findAlpha(int i){
   static int n;
   static float th=0;
   static float q[3], dl[3], dl2;
   double min=servo_min;
   double max=servo_max;
   n=0;
   th=theta_a[i];
   while(n<20){
    //calculation of position of base attachment point (point on servo arm where is leg connected)
      q[0] = L1*cos(th)*cos(beta[i]) + p[0][i];
      q[1] = L1*cos(th)*sin(beta[i]) + p[1][i];
      q[2] = L1*sin(th);

    //calculation of distance between according platform attachment point and base attachment point
      dl[0] = posA[0][i] - q[0];
      dl[1] = posA[1][i] - q[1];
      dl[2] = posA[2][i] - q[2];
      dl2 = sqrt(dl[0]*dl[0] + dl[1]*dl[1] + dl[2]*dl[2]);

    // if this distance is the same as leg length, value of theta_a is corrent, we return it
      if(abs(L2-dl2)<0.01){
         return th;
      }

    // if not, we split the searched space in half, then try next value
      if(dl2<L2){
         max=th;
      }else{
         min=th;
      }
      n+=1;
      if(max==servo_min || min==servo_max){
         return th;
      }
      th = min+(max-min)/2;
   }
   return th;
   
}
/**********************************************************************************************************************************************************************/
//function calculating rotation matrix
void findRotMatrix(float pe[])
{
   float phi=pe[3];
   float theta=pe[4];
   float psi=pe[5];
   
   Rot[0][0] = cos(psi)*cos(theta);
   Rot[0][1] = sin(psi)*cos(theta);
   Rot[0][2] = -sin(theta);
   
   Rot[1][0] = -sin(psi)*cos(phi)+cos(psi)*sin(theta)*sin(phi);
   Rot[1][1] = cos(psi)*cos(phi)+sin(psi)*sin(theta)*sin(phi);
   Rot[1][2] = -cos(psi)*sin(phi)+sin(psi)*sin(theta)*cos(phi);
   
   Rot[2][0] = sin(psi)*sin(phi)+cos(psi)*cos(phi)*sin(theta);
   Rot[2][1] = cos(theta)*sin(phi);
   Rot[2][2] = cos(theta)*cos(phi);
}
/**********************************************************************************************************************************************************************/
//calculates wanted position of platform attachment points using calculated rotation matrix and translation vector
void findPosA(float pe[])
{
   for(int i=0;i<6;i++){
      posA[0][i] = Trans[0]+Rot[0][0]*(re[0][i])+Rot[0][1]*(re[1][i])+Rot[0][2]*(re[2][i]);
      posA[1][i] = Trans[1]+Rot[1][0]*(re[0][i])+Rot[1][1]*(re[1][i])+Rot[1][2]*(re[2][i]);
      posA[2][i] = Trans[2]+Rot[2][0]*(re[0][i])+Rot[2][1]*(re[1][i])+Rot[2][2]*(re[2][i]);
   }
}
/**********************************************************************************************************************************************************************/
//function calculating translation vector = desired move vector + home translation vector
void findTrans(float pe[])
{
   Trans[0] = pe[0]+HomeTr[0];
   Trans[1] = pe[1]+HomeTr[1];
   Trans[2] = pe[2]+HomeTr[2];
}
/**********************************************************************************************************************************************************************/
// function for setting the platform in the initial base positon
unsigned char setPosin(float pe[]){
    unsigned char errorCt;
    errorCt=0;
    for(int i = 0; i < 6; i++)
    {
        findTrans(pe);
        findRotMatrix(pe);
        findPosA(pe);
        theta_a[i]=findAlpha(i);
        if(i==Opp1||i==Opp2||i==Opp3){
            servo_pos[i] = constrain(base[i] - (theta_a[i])*servo_mult, MINI,MAXI);
        }
        else{
            servo_pos[i] = constrain(base[i] + (theta_a[i])*servo_mult, MINI,MAXI);
        }
    }

  // Move all motors
    for(int i = 0; i < 6; i++)
    {
       if(theta_a[i]==servo_min||theta_a[i]==servo_max||servo_pos[i]==MINI||servo_pos[i]==MAXI){
            errorCt++;
        }
        servo[i].writeMicroseconds(servo_pos[i]);
    }

  //  current servo postion becomes previous servo position for the next movement, so currently servo_pos[] is stored in servo_pos_prev[]
   for(int i=0;i<6;i++) servo_pos_prev[i] = servo_pos[i];

    return errorCt;
}
/**********************************************************************************************************************************************************************/
// function for setting the platform according to the required positon given as input
unsigned char setPos(float pe[]){
    unsigned char errorCt;
    errorCt=0;
    for(int i = 0; i < 6; i++)
    {
        findTrans(pe);
        findRotMatrix(pe);
        findPosA(pe);
        theta_a[i]=findAlpha(i);
        // theta_a[i]=findAlpha(&i);
        if(i==Opp1||i==Opp2||i==Opp3){
            servo_pos[i] = constrain(base[i] - (theta_a[i])*servo_mult, MINI,MAXI);
        }
        else{
            servo_pos[i] = constrain(base[i] + (theta_a[i])*servo_mult, MINI,MAXI);
        }
    }

    for(int i = 0; i < 6; i++)
    {
        if(theta_a[i]==servo_min||theta_a[i]==servo_max||servo_pos[i]==MINI||servo_pos[i]==MAXI){
            errorCt++;
        }
    }
     
    //  degrees store the change in servo position required from the previous servo position in us
    int degrees[6];
    // mx is maximum absolute change required
    int mx=0;
    for(int i=0;i<6;i++){
         degrees[i] = (servo_pos[i]-servo_pos_prev[i]);
         mx = max(mx,abs(degrees[i]));
    }    
    // Move all motors according to required degrees[i] value
    for (int i = 1; i <=abs(mx); i++) {
      if(servo_pos_prev[0]+i<servo_pos[0]) servo[0].write(servo_pos_prev[0] + i);
      else if(servo_pos_prev[0]-i>servo_pos[0]) servo[0].write(servo_pos_prev[0] - i);
      
      if(servo_pos_prev[1]+i<servo_pos[1]) servo[1].write(servo_pos_prev[1] + i);
      else if(servo_pos_prev[1]-i>servo_pos[1]) servo[1].write(servo_pos_prev[1] - i);
      
      if(servo_pos_prev[2]+i<servo_pos[2]) servo[2].write(servo_pos_prev[2] + i);
      else if(servo_pos_prev[2]-i>servo_pos[2]) servo[2].write(servo_pos_prev[2] - i);
      
      if(servo_pos_prev[3]+i<servo_pos[3]) servo[3].write(servo_pos_prev[3] + i);
      else if(servo_pos_prev[3]-i>servo_pos[3]) servo[3].write(servo_pos_prev[3] - i);
      
      if(servo_pos_prev[4]+i<servo_pos[4]) servo[4].write(servo_pos_prev[4] + i);
      else if(servo_pos_prev[4]-i>servo_pos[4]) servo[4].write(servo_pos_prev[4] - i);

      if(servo_pos_prev[5]+i<servo_pos[5]) servo[5].write(servo_pos_prev[5] + i);
      else if(servo_pos_prev[5]-i>servo_pos[5]) servo[5].write(servo_pos_prev[5] - i);
      for(int j=0;j<6;j++){
         
      Serial.println(String(prev[j]+(((curr[j]-prev[j])*i)/mx))+last[j]  );
          
      }
      
      delay(1000 / speed);
    }
    
     Serial.println("done");
     for(int j=0;j<6;j++) prev[j]=curr[j];
     delay(100);
 
   //  current servo postion becomes previous servo position for the next movement, so currently servo_pos[] is stored in servo_pos_prev[]
   for(int i=0;i<6;i++) servo_pos_prev[i] = servo_pos[i];

    return errorCt;
}
/**********************************************************************************************************************************************************************/
/**********************************************************************************************************************************************************************/
//main control loop, obtain requested action from serial connection, then execute it
void loop()
{
     while(Serial.available() > 0) {  

   
      val1 = Serial.readStringUntil('@');   //Serial.println("x = "+val1);  
      val2 = Serial.readStringUntil('@');   //Serial.println("y = "+val2); 
      val3 = Serial.readStringUntil('@');   //Serial.println("z = "+val3); 
      val4 = Serial.readStringUntil('@');   //Serial.println("xangle = "+val4); 
      val5 = Serial.readStringUntil('@');   //Serial.println("yangle = "+val5); 
      val6 = Serial.readStringUntil('@');   //Serial.println("zangle = "+val6); 
   
       curr[0] = val1.toFloat();
       curr[1] = val2.toFloat();
       curr[2] = val3.toFloat();  
       curr[3] = radians(val4.toFloat());  
       curr[4] = radians(val5.toFloat());  
       curr[5] = radians(val6.toFloat());  
         
        float arr[6];
        arr[0] = curr[1];
        arr[1] = curr[0];
        arr[2] = -curr[2];  
        arr[3] = curr[4];
        arr[4] = curr[3] ; 
        arr[5] = curr[5];
        setPos(arr);
        delay(100);
     }
}
/**********************************************************************************************************************************************************************/
/**********************************************************************************************************************************************************************/
/**********************************************************************************************************************************************************************/
/**********************************************************************************************************************************************************************/
