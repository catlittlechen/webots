#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/gps.h>
#include <webots/receiver.h>
#include <stdio.h>
#include <stdlib.h>
#define SPEED 20
#define TIME_STEP 32
#define findline 30
#define readytime 20
WbDeviceTag ir0,ir1,ir2,ir3;

long map[8][8];
double left_speed, right_speed;
double line[findline][2];
int linecount=0;
int one=0;
int twenty=100;

int state=0;
int time=10;
int turnstate=1;
#define Nothing 0
#define IsWall_front_left 1
#define IsWall_front_right 2
#define IsWall 3
#define Corner 4
#define Ready 5
void go(){
left_speed=SPEED;
right_speed=SPEED;
}
void turn_left(){
left_speed=-SPEED;
right_speed=SPEED;
turnstate=0;
}
void turn_right(){
left_speed=SPEED;
right_speed=-SPEED;
turnstate=1;
}
void back(){
left_speed=-SPEED;
right_speed=-SPEED;
}
bool saveload(double x,double z)
{
  line[linecount][0]=x;
  line[linecount][1]=z;
  linecount=(linecount+1)%findline;
  
  int xl = (x+0.4)/0.1;
  int zl = (z+0.4)/0.1;
  ++map[xl][zl];
  
  if (line[linecount][0]-x==0&&line[linecount][1]-z==0)
    return true;
  else
    return false;
}
bool min_value(long a,long b)
{
  if (a<b)
  return true;
  else
  return false;
}
bool choose(int *left)
{
  int pre = (linecount - 1 +findline)%findline;
  *left=-1;
  int dir;
  
  double x_low = line[pre][0]-line[linecount][0];
  double z_low = line[pre][1]-line[linecount][1];
  if (x_low<=0)
    if (z_low<=0)
      dir=1;
    else
      dir=4;
  else
    if (z_low<=0)
      dir=2;
    else
      dir=3;
  
  
  int xl = (line[pre][0]+0.4)/0.1;
  int zl = (line[pre][1]+0.4)/0.1;
  long minvalue = map[xl][zl];
  switch (dir)
  {
  case 1:
  if (xl>0&&min_value(minvalue,map[xl-1][zl]))
  {
    minvalue=map[xl-1][zl];
    *left =0;
  }
  if (zl>0&&min_value(minvalue,map[xl][zl-1]))
  {
    minvalue=map[xl][zl-1];
    *left =1;
  }
  break;
  case 4:
  if (xl>0&&min_value(minvalue,map[xl-1][zl]))
  {
    minvalue=map[xl-1][zl];
    *left=1;
  }
  if (zl<7&&min_value(minvalue,map[xl][zl+1]))
  {
    minvalue=map[xl][zl+1];
    *left=0;
  }
  break;
  case 2:
  if (xl>0&&min_value(minvalue,map[xl-1][zl]))
  {
    minvalue=map[xl-1][zl];
    *left=0;
  }
  if (zl<7&&min_value(minvalue,map[xl][zl+1]))
  {
    minvalue=map[xl][zl+1];
    *left=1;
  }
  break;
  case 3:
  if (xl<7&&min_value(minvalue,map[xl+1][zl]))
  {
    minvalue=map[xl+1][zl];
    *left=0;
  }
    if (zl<7&&min_value(minvalue,map[xl][zl+1]))
  {
    minvalue=map[xl][zl+1];
    *left=1;
  }
  }
  if (*left == -1)
    return false;
  else
    return true;
}

void init()
{
int i,j;
for (i=0;i<8;++i)
  for (j=0;j<8;++j)
    map[i][j]=0;
for (i=0;i<findline;++i)
  for (j=0;j<2;++j)
    line[i][j]=0.0;
}

int main()
{
  int choose_left=-1;
  init();
  wb_robot_init();
  ir0 = wb_robot_get_device("ir0");
  ir1 = wb_robot_get_device("ir1");
  ir2 = wb_robot_get_device("ir2");
  ir3 = wb_robot_get_device("ir3");
  wb_distance_sensor_enable(ir0, TIME_STEP);
  wb_distance_sensor_enable(ir1, TIME_STEP);
  wb_distance_sensor_enable(ir2, TIME_STEP);
  wb_distance_sensor_enable(ir3, TIME_STEP);
  while(wb_robot_step(TIME_STEP)!=-1) 
  {
    double ir0_value = wb_distance_sensor_get_value(ir0);
    double ir1_value = wb_distance_sensor_get_value(ir1);
    double ir2_value = wb_distance_sensor_get_value(ir2);
    double ir3_value = wb_distance_sensor_get_value(ir3);
    WbDeviceTag gps = wb_robot_get_device("gps");
    wb_gps_enable(gps, TIME_STEP);
    WbDeviceTag receiver = wb_robot_get_device("receiver");
    wb_receiver_enable(receiver, TIME_STEP);
    const double *gps_values = wb_gps_get_values(gps);
    
    bool d = saveload(gps_values[0],gps_values[2]);
    if (twenty-->0)
      d=false;
    //printf("%f %f %f %f ",ir0_value,ir1_value,ir2_value,ir3_value);
    //int i=10;
    //while (i--)
    //{
    //  printf("%d",state);
    //}
    switch (state)
    {
      case Nothing:
      {
            go();
            
            if ((ir0_value>1000&&ir1_value>1000))
            {
              if(ir3_value>1000)
              {
                if (ir2_value<1000)
                  state = Ready;
                else
                  state = Ready;
              }
              else
                state = Ready;
            }
            else if(ir0_value>1000)
                state = Ready;
            else if(ir1_value>1000)
                state = Ready;
            else if (d)
            {
                state = Ready;
            }
            time=readytime;
            break;
      }
      case Ready:
      {
          if (time-- > 0)
          {
            go();
            break;
          }
          if ((ir0_value>1000&&ir1_value>1000))
            {
              if(ir3_value>1000)
              {
                if (ir2_value<1000)
                  state = IsWall_front_right;
                else
                  state = Corner;
              }
              else if(ir2_value>1000)
                state = IsWall_front_left;
              //else state=IsWall;
            }
            else if(ir0_value>1000)
                state = IsWall_front_left;
            else if(ir1_value>1000)
                state = IsWall_front_right;
            //else 
            if (d)
            {
                state = IsWall;
                time=readytime;
            }
            break;
      }
      case IsWall_front_right:
      {
            turn_left();
          if (ir0_value<1000&&ir1_value<1000&&ir3_value>1000)
              state = Nothing;
          break;
      }
      case IsWall_front_left:
      {
          turn_right();
          if (ir0_value<1000&&ir1_value<1000&&ir2_value>1000)
              state = Nothing;
          break;
      }
      case Corner:
      {
          turn_right();
          if (ir0_value<1000&&ir1_value<1000&&ir3_value>1000&&ir3_value>1000)
              state = Nothing;
          break;
      }
      case IsWall:
      {
        
        if (choose(&choose_left))
          if (choose_left == 1)
            turn_left();
          else
            turn_right();
        else
          if (turnstate)
            turn_right();
          else
            turn_left();
        time--;
        if (time==0)
          state = Nothing;
      }
    }
    printf(" %f %f\n",gps_values[0],gps_values[2]);
    if (gps_values[0]<-0.25 && gps_values[2]<-0.25 && one--!=0)
    {
      printf("success!");
      break;
    }
    wb_differential_wheels_set_speed(left_speed, right_speed); 
  }
  return 0;
}
