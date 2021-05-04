#include "ros/ros.h"
#include "std_msgs/String.h"



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
bool kudos_run = true;

//Xenomai time variables
RT_TASK RT_task1;
RTIME now1, previous1; //Tread 1 cycle time
double thread_time1 = 0.0; //Thread 1 cycle time
double max_time = 0.0;


/***********************************/
/************function***************/
void kudos_task(void* arg);

void delay(clock_t n);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rt_thread_node");
  ros::NodeHandle nh;

  ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);

  rt_task_create(&RT_task1,"KUDOS_tesk",0,90,0);
  printf("thread_created..\n");
  rt_task_start(&RT_task1,&kudos_task,NULL);
  printf("thread_started..\n");

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
  kudos_run = false;
  return 0;
}


void kudos_task(void* arg){

  unsigned int count=0;
  rt_task_set_periodic(NULL, TM_NOW, cycle_ns*1);

  while (kudos_run){

    rt_task_wait_period(NULL);
    printf("%d\n",count);
    count++;
    delay(1000);
  }


}

//fuction for test
void delay(clock_t n){
  clock_t start = clock();
  while(clock()-start <n);

}
