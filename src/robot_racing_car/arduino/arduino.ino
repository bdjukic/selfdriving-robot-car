#include <ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int16.h>

ros::NodeHandle nh;

/*define logic control output pin*/
int Echo = A4;  
int Trig = A5; 
int in1=6;
int in2=7;
int in3=8;
int in4=9;

/*define channel enable output pins*/
int ENA=5;
int ENB=11;
int ABS = 100;

void set_speed(const std_msgs::Int16& msg)
{
  int new_speed = msg.data;
  
  if (new_speed <= 0)
  {
    ABS = 0;
  }
  else if (new_speed >= 255)
  {
    ABS = 255;
  } 
  else
  {
    ABS = msg.data;
  }
}

void move_robot(const std_msgs::UInt8& msg)
{ 
  switch(msg.data)
  {
   case 0:
   {
     analogWrite(ENA,ABS);
     analogWrite(ENB,ABS);
     digitalWrite(in1,LOW);
     digitalWrite(in2,HIGH);
     digitalWrite(in3,HIGH);
     digitalWrite(in4,LOW);
     
     nh.logdebug("Moving forward");
     break;
   }
   case 1:
   { 
     analogWrite(ENA,ABS);
     analogWrite(ENB,ABS);
     digitalWrite(in1,HIGH);
     digitalWrite(in2,LOW);
     digitalWrite(in3,LOW);
     digitalWrite(in4,HIGH);
     
     nh.logdebug("Moving back");
     break;
   } 
   case 2:
   {
     analogWrite(ENA,ABS);
     analogWrite(ENB,ABS);
     digitalWrite(in1,HIGH);
     digitalWrite(in2,LOW);
     digitalWrite(in3,HIGH);
     digitalWrite(in4,LOW); 
  
     nh.logdebug("Turning left");
     break;
   } 
   case 3:
   {
     analogWrite(ENA,ABS);
     analogWrite(ENB,ABS);
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

int get_distance()   
{
  digitalWrite(Trig, LOW);   
  delayMicroseconds(2);
  digitalWrite(Trig, HIGH);  
  delayMicroseconds(20);
  digitalWrite(Trig, LOW);   
  
  float distance = pulseIn(Echo, HIGH);  
  distance= distance/58; 
  
  return (int)distance;
}  

ros::Subscriber<std_msgs::UInt8> steering_subscriber("robot_car_steering", &move_robot);
ros::Subscriber<std_msgs::Int16> speed_subscriber("robot_car_speed", &set_speed);

std_msgs::Int16 ultrasonic_sensor_message;
ros::Publisher ultrasonic_sensor_publisher("robot_car_ultrasonic_sensor", &ultrasonic_sensor_message);

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
  
  nh.subscribe(steering_subscriber);
  nh.subscribe(speed_subscriber);
}

void loop() 
{
//  ultrasonic_sensor_message.data = get_distance();
//  ultrasonic_sensor_publisher.publish(&ultrasonic_sensor_message);
  
  nh.spinOnce();
  
  delay(1);
}
