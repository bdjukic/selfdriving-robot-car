#include <ros.h>
#include <std_msgs/UInt8.h>

ros::NodeHandle nh;

/*define logic control output pin*/
int in1=6;
int in2=7;
int in3=8;
int in4=9;

/*define channel enable output pins*/
int ENA=5;
int ENB=11;

void moveRobot(const std_msgs::UInt8& msg)
{ 
  switch(msg.data)
  {
   case 0:
   {
     digitalWrite(ENA,HIGH);
     digitalWrite(ENB,HIGH);
     digitalWrite(in1,HIGH);
     digitalWrite(in2,LOW);
     digitalWrite(in3,LOW);
     digitalWrite(in4,HIGH);
     
     nh.logdebug("Moving forward");
     break;
   }
   case 1:
   {
     digitalWrite(ENA,HIGH);
     digitalWrite(ENB,HIGH);
     digitalWrite(in1,LOW);
     digitalWrite(in2,HIGH);
     digitalWrite(in3,HIGH);
     digitalWrite(in4,LOW);
  
     nh.logdebug("Moving back");
     break;
   } 
   case 2:
   {
     digitalWrite(ENA,HIGH);
     digitalWrite(ENB,HIGH);
     digitalWrite(in1,HIGH);
     digitalWrite(in2,LOW);
     digitalWrite(in3,HIGH);
     digitalWrite(in4,LOW); 
  
     nh.logdebug("Turning left");
     break;
   } 
   case 3:
   {
     digitalWrite(ENA,HIGH);
     digitalWrite(ENB,HIGH);
     digitalWrite(in1,LOW);
     digitalWrite(in2,HIGH);
     digitalWrite(in3,LOW);
     digitalWrite(in4,HIGH);
  
     nh.logdebug("Turning right");
     break;
   }
   case 4:
   {
     digitalWrite(ENA,LOW);
     digitalWrite(ENB,LOW);
     digitalWrite(in1,LOW);
     digitalWrite(in2,LOW);
     digitalWrite(in3,LOW);
     digitalWrite(in4,LOW);
  
     nh.logdebug("Breaking");
     break;
   }
  } 
}

ros::Subscriber<std_msgs::UInt8> robot_car_subscriber("robot_car_arduino", &moveRobot);

void setup() 
{
  Serial.begin(9600); //Open the serial port and set the baud rate to 9600

  /*Set the defined pins to the output*/
  pinMode(in1,OUTPUT);
  pinMode(in2,OUTPUT);
  pinMode(in3,OUTPUT);
  pinMode(in4,OUTPUT);
  pinMode(ENA,OUTPUT);
  pinMode(ENB,OUTPUT);
  
  nh.initNode();
  
  nh.subscribe(robot_car_subscriber);
}

void loop() 
{
  nh.spinOnce();
  
  delay(1);
}
