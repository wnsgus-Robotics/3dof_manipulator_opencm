#include <DynamixelWorkbench.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include "w_ik.h"

#define pi 3.14159

#define MOTER 1   //mx = 1 ax-12 = 2  
#define LINK1 130
#define LINK2 125


int p_j1=0,p_j2=0,p_j3=0,p_j4=0,p_j5=0;

uint16_t rate =1020;
int model =0;
DynamixelWorkbench dx;

ros::NodeHandle nh;
void order_cb( const std_msgs::Float32MultiArray& manipulator_msg)
{
    int type=manipulator_msg.data[0];
    if(type==1)
    {
      pose_control(manipulator_msg.data[1],manipulator_msg.data[2],0);
      base_control(manipulator_msg.data[3],0);
      gripper_control(manipulator_msg.data[4],0);
    }
    else if(type==0)
    {
      joint_control(manipulator_msg.data[1],manipulator_msg.data[2],manipulator_msg.data[3],manipulator_msg.data[4]);
      gripper_control(manipulator_msg.data[5],0);
    }

}
ros::Subscriber<std_msgs::Float32MultiArray> ik_sub("manipulator_msg", &order_cb);

void setup() 
{
  nh.initNode();
  nh.subscribe(ik_sub);
  init();
  dx.goalPosition(11,convert_dgree(MOTER,0));
  dx.goalPosition(12,convert_dgree(MOTER,0));
  dx.goalPosition(13,convert_dgree(MOTER,0));
  dx.goalPosition(14,convert_dgree(MOTER,0));
  gripper_control(1,0);
}
void loop() 
{
 nh.spinOnce();
}


int convert_dgree(char ch,double a)
{
  if(MOTER==1)
   return a*512/45+2048; 
  else if(MOTER==2)
   return a*(-307)/90+512;
}
void joint_control(double j1,double j2,double j3,double j4)
{
  dx.jointMode(11,abs(p_j1-j1)*0.55+1,0);
  dx.goalPosition(11, convert_dgree(MOTER,j1));
  dx.jointMode(12,abs(p_j2-j2)*0.55+1,0);
  dx.goalPosition(12, convert_dgree(MOTER,j2));
  dx.jointMode(13,abs(p_j3-j3)*0.55+1,0);
  dx.goalPosition(13, convert_dgree(MOTER,j3));
   dx.jointMode(14,abs(p_j4-j4)*0.55+1,0);
  dx.goalPosition(14, convert_dgree(MOTER,j4));
  p_j1=j1;
  p_j2=j2;
  p_j3=j3;
  p_j4=j4;


}
double convert_rad(double a)
{
  return a*180/pi;
}

void end_ef(float j1,float j2)
{
 float j3=max_dgree(j1)+max_dgree(j2);

 dx.jointMode(14,abs(p_j4-convert_rad(-j3))*0.55+1,0);
 dx.goalPosition(14, convert_dgree(MOTER,convert_rad(-j3)));
 p_j4=convert_rad(-j3);
}

void pose_control(double x,double y,int i)
{
   int lengths[] = {LINK1,LINK2};
   wnsgus_ik is(3, lengths);
   if(model==i ||i==0)
   {
   if(is.solve(x,y,lengths))
     {
       end_ef(is.getAngle(0),is.getAngle(1));
       dx.jointMode(13,abs(p_j3-(90+convert_rad(max_dgree(is.getAngle(1)))))*0.55+1,0);
       dx.goalPosition(13, convert_dgree(MOTER,90+convert_rad(max_dgree(is.getAngle(1)))));
       dx.jointMode(12,abs(p_j2-(convert_rad(max_dgree(is.getAngle(0)))-90))*0.55+1,0);
       dx.goalPosition(12, convert_dgree(MOTER,convert_rad(max_dgree(is.getAngle(0)))-90));
       p_j3=90+convert_rad(max_dgree(is.getAngle(1)));
       p_j2=convert_rad(max_dgree(is.getAngle(0)))-90;
       }
       //Serial.print(convert_rad(a.getAngle(0))-90);
       //Serial.print(",");
       //Serial.print(90+convert_rad(a.getAngle(1)));
   }

}
float max_dgree(double a)
{
  if(a>90)
    a=90;
  else if(a<-90)
    a=-90;
  return a;
}
void base_control(float a,int i)
{
     dx.jointMode(11,abs(p_j1-a)*0.55+1,0);
     dx.goalPosition(11, convert_dgree(MOTER,a));
     p_j1=a;
}

void init()
{
  dx.init("3",1000000);
  for(int i=11;i<16;i++)
    dx.ping(i, &rate);
  dx.jointMode(11,10,0);
  dx.jointMode(12,10,0);
  dx.jointMode(13,10,0);
  dx.jointMode(14,10,0);
  dx.jointMode(15,50,0);
}



void gripper_control(int a,int i)
{
  if(model==i ||i==0)
  {
    if(a==1)
      dx.goalPosition(15, convert_dgree(MOTER,80));
    else if(a==0)
      dx.goalPosition(15, convert_dgree(MOTER,-60));
  }
}
