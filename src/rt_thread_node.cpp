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

unsigned int cycle_ns = 1000000; //Control Cycle 1[ms]

//loop flag(thread)
bool CAN_run = true;

//Xenomai time variables
RT_TASK RT_task1;
RTIME now1, previous1; //Tread 1 cycle time
double thread_time1 = 0.0; //Thread 1 cycle time
double max_time = 0.0;


/***********************************/
/************function***************/
void CAN_task(void* arg);

void delay(clock_t n);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rt_thread_node");
  ros::NodeHandle nh;

  ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);
  ros::Subscriber cmd_vel_sub = nh.subscribe("/cmd_vel", 1000, cmd_velCallback);

  rt_task_create(&RT_task1,"CAN_tesk",0,90,0);
  ROS_INFO("thread_created..\n");
  rt_task_start(&RT_task1,&CAN_task,NULL);
  ROS_INFO("thread_started..\n");

  //Thread Setting End

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    std_msgs::String msg;
    msg.data = "hello world";

    chatter_pub.publish(msg);

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
    Encoder_REQ();
  }


}

//fuction for test
void delay(clock_t n){
  clock_t start = clock();
  while(clock()-start <n);

}
