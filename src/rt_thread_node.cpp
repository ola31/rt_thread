#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "geometry_msgs/Twist.h"


#include "can_test/can.h"
#include "can_test/motor_driver.h"


#include<sys/mman.h>
#include <signal.h>
#include <unistd.h>
#include <time.h>
#include <iostream>
#include <sys/time.h>
#include <stdio.h>
#include <pthread.h>
#include <inttypes.h>


/************xenomai**************/

#include <alchemy/task.h>
#include <alchemy/timer.h>
#include <alchemy/mutex.h>
#include <alchemy/sem.h>
#include <boilerplate/trace.h>
#include <xenomai/init.h>

#define NSEC_PER_SEC 1000000000

unsigned int cycle_ns = 10*1000000; //Control Cycle 10[ms]

//loop flag(thread)
bool CAN_run = true;

//global variables
float vel_arr[2];
static int operating_mode=2;           //start mode = cmd_vel mode
int r_rpm=0;
int l_rpm=0;


//Xenomai time variables
RT_TASK RT_task1;
RTIME now1, previous1; //Tread 1 cycle time
double thread_time1 = 0.0; //Thread 1 cycle time
double max_time = 0.0;


/***********************************/
/************function***************/
void CAN_task(void* arg);

void delay(clock_t n);

/********** ROS callback functions********/
void modeCallback(const std_msgs::Int8::ConstPtr& msg);
void cmd_velCallback(const geometry_msgs::Twist::ConstPtr& msg);
void rpmCallback(const can_test::rpm::ConstPtr& msg);



int main(int argc, char **argv)
{
  ros::init(argc, argv, "rt_thread_node");
  ros::NodeHandle nh;

  ros::Subscriber cmd_vel_sub = nh.subscribe("/cmd_vel", 1000, cmd_velCallback);
  ros::Subscriber rpm_sub = nh.subscribe("/rpm", 1000, rpmCallback);
  ros::Subscriber mode_sub = nh.subscribe("/mode", 1000, modeCallback);

  rt_task_create(&RT_task1,"CAN_tesk",0,90,0);
  ROS_INFO("thread_created..\n");
  rt_task_start(&RT_task1,&CAN_task,NULL);
  ROS_INFO("thread_started..\n");

  //Thread Setting End

  CAN_initialize();
  Reset_ENC();

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    ros::spinOnce();

    loop_rate.sleep();
  }

  CAN_run = false;
  return 0;
}


void CAN_task(void* arg){

  unsigned int count=0;
  rt_task_set_periodic(NULL, TM_NOW, cycle_ns*10);  //10ms

  static struct Encoder_data enc_data;
  Encoder_REQ();

  while (CAN_run){

    rt_task_wait_period(NULL);

    enc_data = read_Encoder();
    ROS_INFO("R_posi : %d   L_posi : %d",enc_data.R_posi,enc_data.L_posi);

    if(operating_mode == 2 || operating_mode== 3){
      contol_vel(vel_arr);
      //ROS_INFO("Linear_x : %f angular_z : %f",linear_x,angular_z);
    }
    else if(operating_mode == 1){
      send_RPM(r_rpm,l_rpm);
    }

    Encoder_REQ();
  }


}



//fuction for test
void delay(clock_t n){
  clock_t start = clock();
  while(clock()-start <n);

}




/********** ROS Callback fuctions ***********/

void modeCallback(const std_msgs::Int8::ConstPtr& msg){

  if(operating_mode != msg->data){
    Torque_OFF();
    //ROS_INFO("modecallback1");
  }
  operating_mode = msg->data;

  //ROS_INFO("modecallback2");

  if(operating_mode == 1){}
    //ROS_INFO("Operating Mode : RPM control mode");
  else if(operating_mode == 2){}
    //ROS_INFO("Operating Mode : CMD_VEL control mode");
  else if(operating_mode == 0){
    ROS_INFO("TQ_OFF!!");
    Torque_OFF();
  }
 else if(operating_mode == 3){
    //ROS_INFO("Dymanic cmd_vel mode");
 }
  else
    ROS_WARN("Invalid control mode number");
}


void cmd_velCallback(const geometry_msgs::Twist::ConstPtr& msg){

  float linear_x = msg->linear.x;
  float angular_z = msg->angular.z;
  vel_arr[2] = {linear_x,angular_z}; //전역변수 배열에 저장

}

void rpmCallback(const can_test::rpm::ConstPtr& msg){

  if(operating_mode == 1){
    //ROS_INFO("rpmcallback1");
    r_rpm = (msg->r_rpm);
    l_rpm = (msg->l_rpm)*-1;
  }
  //ROS_INFO("rpmcallback2");

}
