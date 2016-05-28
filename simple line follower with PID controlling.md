# Line-Follower-Robot
/* Black line on White Surface
 *  Five IR sensor
 *  
 *  From Back side of the robot left motor is motor 'A', right motor is 'B' and also
 *  sensor 0 to sensor 4 are counteed from left to right
 *  
 *  
 */
 
int pGain=170;  //pGain, dGain, iGain varies for various robot. so need to be tuned them
int iGain=0;
int dGain=50;
int iInteg=0;
int eInteg=0;
int ePrev=0;

int speedA;
int speedB;
int avgSpeed=120;

float turn=0;

int INA=10;  
int INB=11;
int INC=12;
int IND=13;

int EN1=8;
int EN2=9;

float pid=0;
float error=0;

void setup() {
  pinMode(INA, OUTPUT );
  pinMode(INB, OUTPUT );
  pinMode(INC, OUTPUT );
  pinMode(IND, OUTPUT );
  pinMode(EN1, OUTPUT);
  pinMode(EN2, OUTPUT);

  Serial.begin( 9600 );
}

float PID(float cur_value,float req_value)
{         
  error = req_value - cur_value;
  pid = (pGain * error) + (dGain * (error - ePrev))+(iGain * eInteg);

  eInteg += error;                  // integral is simply a summation over time
  ePrev = error;                    // save previous for derivative

  return pid;
}

void a_for()        ///Motor A forward direction
{
  digitalWrite(INA,LOW);
  digitalWrite(INB,HIGH);
}

void a_bac()        ///Motor A backward direction
{
  digitalWrite(INA,HIGH);
  digitalWrite(INB,LOW);
}

void b_for()        ///Motor B forward direction
{
  digitalWrite(INC,LOW);
  digitalWrite(IND,HIGH);
}

void b_bac()        ///Motor B backward direction
{
  digitalWrite(INC,HIGH);
  digitalWrite(IND,LOW);
}

void a_stop()
{
  digitalWrite(INA,LOW);
  digitalWrite(INB,LOW);
}
void b_stop()
{
  digitalWrite(INC,LOW);
  digitalWrite(IND,LOW);
}

void loop()
{
  int s[5];
  int value[5]={450,450,450,450,450}; //threshold values for each sensor
  for(int i=0;i<5;i++)
  {
    if(analogRead(i)>value[i])
    {
       s[i]=1;
    }
    else
    {
      s[i]=0;
    }
  }
   
/*  for(int i=0;i<7;i++)    //For test
  {
     Serial.print(s[i]);  //
      Serial.print('\t'); //     
  }
  Serial.println();
  delay(500);
*/

  float avgSensor = s[0]*1 + s[1]*2 + s[2]*3 + s[3]*4 + s[4]*5 ;
  
  avgSensor = (float) avgSensor / (s[0] + s[1] + s[2] + s[3] + s[4]);
  
  if(avgSensor!=0)
  {
    turn=PID(avgSensor,3);
  }

    /*Serial.println(turn);
    delay(500);*/
//////////*******PID Calculations*********///////////
      
  if(turn>=500)          //Limit the pid value
    turn=500;
  else if(turn<=-500)
    turn=-500;

  if(turn>0)            // when left arrayes touch line
  {
    if(turn>250)        //Vary hard turn
    {
      a_bac();
      speedA=(turn-250)/5;
    }
    else                //Normar turn
    {
      a_for();
      speedA=(250-turn)/5;
    }
    speedB=avgSpeed;
  }
  
  else if(turn==0)           //When On the line
  {
    a_for();
    b_for();
    speedA=avgSpeed;
    speedB=avgSpeed;
  }

  else if(turn<0)            //When Right sensor touchs line
  {
    if(turn<-250)        //Very hard turn
    {
      b_bac();
      speedB=(-turn-250)/5;
    }
    else                //Normal turn
    {
      b_for();
      speedB=(250+turn)/5;
    }
    speedA=avgSpeed;
  }
  analogWrite(EN1,speedA);
  analogWrite(EN2,speedB);
}
