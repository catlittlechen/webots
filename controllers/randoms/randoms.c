#include <webots/supervisor.h>
#include <webots/robot.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <math.h>
#define TIME_STEP 32
#define INIT_SIZE 2
#define MAX_SIZE  10
static double positions[MAX_SIZE];
static int init_pos_x[INIT_SIZE];    
static int init_pos_y[INIT_SIZE];
static char name[7] = "WALL_01";
WbNodeRef current_brick;
WbFieldRef current_translation;
WbFieldRef current_rotation;
const char *robot="ck";
void init()
 { 
  int i;
  int n=INIT_SIZE;
  srand(time(0));
  double translation[3];
  double rotation[4] = {0,1,0,0};
  
  
  for (i=0;i<MAX_SIZE;++i)
  {
    positions[i]=i*0.1-0.45;
  }
  for (i=0;i<n;++i)
  {
    init_pos_x[i] = (i+1)*3;
    init_pos_y[i] = rand()%(INIT_SIZE)+3;
  }
  for (i=1;i<=INIT_SIZE;++i)
  {
    name[6] = '0'+i;
    current_brick = wb_supervisor_node_get_from_def(name);
    current_translation = wb_supervisor_node_get_field(current_brick,"translation");
    current_rotation = wb_supervisor_node_get_field(current_brick,"rotation");
    translation[0]=positions[init_pos_x[i-1]];
    translation[1]=0.05;
    translation[2]=positions[init_pos_y[i-1]]+0.05;
    wb_supervisor_field_set_sf_vec3f(current_translation,translation);
    wb_supervisor_field_set_sf_rotation(current_rotation,rotation);
  }
  return;
}

void create_world()
{
init();
}
int main(int argc, char **argv)
{
   wb_robot_init();
    
   create_world();
   
   do {
   
      //WbNodeRef robots = wb_supervisor_node_get_from_def(robot);
     // WbFieldRef robot_translation=wb_supervisor_node_get_field(robots,"translation");
     // WbFieldRef robot_rotation=wb_supervisor_node_get_field(robots,"rotation");
      //const double *t= wb_supervisor_field_get_sf_vec3f(robot_translation);
      //const double *r= wb_supervisor_field_get_sf_rotation(robot_rotation);
      //printf("%f,%f\n",t[0],t[2]);
    } while (wb_robot_step(TIME_STEP) != -1);

  wb_robot_cleanup();
  return 0;
}
