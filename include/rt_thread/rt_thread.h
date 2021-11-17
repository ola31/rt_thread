#ifndef RT_THREAD_H
#define RT_THREAD_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include "rt_thread/rpm.h"

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


#include "can_test/can.h"

#define GEAR_RATIO 30

#define LINEAR 0
#define ANGULAR 1

#define PI 3.141593

#define wheel_separation 0.470
#define wheel_radius 0.150


#define DEG_2_RAD(value) (3.141592/180)*value
#define G 9.80665


#define PID_VER 1            //CAN통신 주고받기 가능 여부 테스트 위한 것
#define PID_REQ_PID_DATA 4   //

#define PID_PNT_VEL_CMD 207
#define PID_MONITOR 196      //모터 1의 모니터데이터(위치정보:D4,5,6,7)
#define PID_MONITOR2 201     //모터 2의 모니터데이터

struct Encoder_data{
  int R_posi;
  int L_posi;
};

static int r_rpm_g,l_rpm_g;



class RT_THREAD : public CAN
{
  public:

    float acc_x = 0.0;
    float acc_y = 0.0;
    float acc_z = 0.0;

    float gyro_x = 0.0;
    float gyro_y = 0.0;
    float gyro_z = 0.0;

    float angle_x= 0.0;
    float angle_y = 0.0;
    float angle_z = 0.0;

    bool is_posi_mode_ = false;

    void initialize_md_imu_driver(void);
    void md_write(BYTE data_array[]);
    void imu_write(BYTE data_array[]);

    void send_RPM(short R_RPM, short L_RPM);
    struct Encoder_data read_Encoder(void);
    void Data_REQ(BYTE R_PID);
    void Encoder_REQ(void);
    void Torque_OFF(void);
    void contol_vel(float *cmd_vel);
    void Reset_ENC(void);

    void cmd(int cmd_num);
    void save_params(void);
    void software_reset(void);
    void set_sync_tx_cycle(int cycle_num);
    void set_sync_req(bool onoff);
    void read_sync_data(void);

    void posi_control(float mot1_rev, float mot2_rev);
    void set_posi_maxvel(int max_rpm);
    void angle_turn(float deg);

    void imu_req(void);
    void imu_read(void);

    void angleY_req(void);
    void angleZ_req(void);
    void gyroZ_req(void);
    void imuReadOnce(void);




    int Byte2Int32(BYTE d4, BYTE d5, BYTE d6, BYTE d7);
    short Byte2Int16(BYTE d1, BYTE d2);
    float Byte2float32(BYTE d4, BYTE d5, BYTE d6, BYTE d7);

};










#endif // RT_THREAD_H
