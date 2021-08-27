#include "rt_thread/rt_thread.h"



#define NSEC_PER_SEC 1000000000

RT_THREAD rt; //declare object

unsigned int cycle_ns = 1000000; //Control Cycle 1[ms]

//loop flag(thread)
bool kudos_run = true;

//Xenomai time variables
RT_TASK RT_task1;
RTIME now1, previous1; //Tread 1 cycle time
double thread_time1 = 0.0; //Thread 1 cycle time
double max_time = 0.0;

float vel_arr[2] = {0,0}; //linear_x, angular_z <global_variable>
int r_rpm,l_rpm = 0.0;

/***********************************/
/************function***************/
void kudos_task(void* arg);

void delay(clock_t n);


static int operating_mode=2;           //start mode = cmd_vel mode


void modeCallback(const std_msgs::Int8::ConstPtr& msg){

  if(operating_mode != msg->data){
    rt.Torque_OFF();
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
    rt.Torque_OFF();
  }
 else if(operating_mode == 3){
    //ROS_INFO("Dymanic cmd_vel mode");
 }
else if(operating_mode == 4){
    //ROS_INFO("Dymanic cmd_vel mode");
 }
  else
    ROS_WARN("Invalid control mode number");
}

void cmd_velCallback(const geometry_msgs::Twist::ConstPtr& msg){

  float linear_x = msg->linear.x;
  float angular_z = msg->angular.z;
  vel_arr[0] = linear_x;
  vel_arr[1] = angular_z;

  if(operating_mode == 2 || operating_mode==3 || operating_mode==4){
     //rt.contol_vel(vel_arr);
    //ROS_INFO("Linear_x : %f angular_z : %f",linear_x,angular_z);
  }
}

void rpmCallback(const md_imu::rpm::ConstPtr& msg){

  if(operating_mode == 1){
    //ROS_INFO("rpmcallback1");
    r_rpm = (msg->r_rpm)*-1;
    l_rpm = msg->l_rpm;
    //rt.send_RPM(r_rpm,l_rpm);
  }
  //ROS_INFO("rpmcallback2");

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rt_thread_node");
  ros::NodeHandle nh;

  ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);
  ros::Publisher present_rpm_pub = nh.advertise<md_imu::rpm>("/present_rpm", 1000);
  ros::Publisher led_status_pub = nh.advertise<std_msgs::UInt16>("/md_driver_status", 1000); // to use gui status led
  ros::Subscriber mode_sub = nh.subscribe("/mode", 1000, modeCallback);
  ros::Subscriber cmd_vel_sub = nh.subscribe("/cmd_vel", 1000, cmd_velCallback);
  ros::Subscriber rpm_sub = nh.subscribe("/rpm", 1000, rpmCallback);


  ros::Publisher angle_pub = nh.advertise<std_msgs::Float32>("/angle/x", 1000);
  ros::Publisher gyro_z_pub = nh.advertise<std_msgs::Float32>("/gyro/z", 1000);
  ros::Publisher angle_y_pub = nh.advertise<std_msgs::Float32>("/angle/y", 1000);
  ros::Publisher angle_z_pub = nh.advertise<std_msgs::Float32>("/angle/z", 1000);

  rt_task_create(&RT_task1,"KUDOS_tesk",0,90,0);
  printf("thread_created..\n");
  rt_task_start(&RT_task1,&kudos_task,NULL);
  printf("thread_started..\n");

  //Thread Setting End

  ros::Rate loop_rate(100);  //100hz = 10ms

  rt.initialize_md_imu_driver();
  rt.Reset_ENC();

  //rt.set_sync_tx_cycle(5);  //10ms = 100hz
  //rt.save_params();
  //rt.software_reset();

  rt.set_sync_req(false);

  struct Encoder_data enc_data;

  //md.Encoder_REQ();
  rt.imu_req();
  loop_rate.sleep();

  while (ros::ok())
  {
    std_msgs::String msg;
    msg.data = "hello world";

    rt::rpm msg;
    std_msgs::UInt16 status_msg;

    msg.r_rpm = r_rpm_g;
    msg.l_rpm = l_rpm_g;

    std_msgs::Float32 gyro_z_msg;
    std_msgs::Float32 angle_y_msg;
    std_msgs::Float32 angle_z_msg;

    gyro_z_msg.data = rt.gyro_z;
    angle_y_msg.data = rt.angle_y;
    angle_z_msg.data = rt.angle_z;

    status_msg.data = 1;

    chatter_pub.publish(msg);

    //present_rpm_pub.publish(msg);
    led_status_pub.publish(status_msg);
    gyro_z_pub.publish(gyro_z_msg);
    angle_y_pub.publish(angle_y_msg);
    angle_z_pub.publish(angle_z_msg);

    ros::spinOnce();
    loop_rate.sleep();
  }
  kudos_run = false;
  return 0;
}


void kudos_task(void* arg){

  unsigned int count=0;
  rt_task_set_periodic(NULL, TM_NOW, cycle_ns*10);  //10ms

  while (kudos_run){

    rt_task_wait_period(NULL);

    rt.imu_read();

    if(operating_mode == 1){
      rt.send_RPM(r_rpm,l_rpm);
    }
    if(operating_mode == 2 || operating_mode==3 || operating_mode==4){
       rt.contol_vel(vel_arr);
      //ROS_INFO("Linear_x : %f angular_z : %f",linear_x,angular_z);
    }

    printf("%d\n",count);
    count++;
    rt.imu_req();
  }


}

//fuction for test
void delay(clock_t n){
  clock_t start = clock();
  while(clock()-start <n);

}
