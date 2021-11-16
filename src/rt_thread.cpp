#include "rt_thread/rt_thread.h"


void RT_THREAD::initialize_md_imu_driver(void){
  CAN_initialize(_250k,"CAN0");  //CAN bit_rate : 250kbps, ID : 0xB7AC01, is_ext_mode : true
}

void RT_THREAD::md_write(BYTE data_array[]){
  struct can_frame md_frame;
  md_frame.can_id = 0xB7AC01;        //32bit
  md_frame.can_id |= CAN_EFF_FLAG;    //extended CAN mode FLAG
  md_frame.can_dlc=8;
  memcpy(md_frame.data,data_array,8); //copy (data_array)->(frame)
  if(send_port(&md_frame) == -1){
    ROS_WARN("md_write_fuction error");
  }

}

void RT_THREAD::imu_write(BYTE data_array[]){
  struct can_frame imu_frame;
  imu_frame.can_id = 0x02;        //32bit
  imu_frame.can_id |= CAN_EFF_FLAG;    //extended CAN mode FLAG
  imu_frame.can_dlc=8;
  memcpy(imu_frame.data,data_array,8); //copy (data_array)->(frame)
  if(send_port(&imu_frame) == -1){
    ROS_WARN("imu_write_fuction error");
  }

}

void RT_THREAD::send_RPM(short R_RPM, short L_RPM){

  BYTE RPM_vel_arr[8]={PID_PNT_VEL_CMD,1,0,0,1,0,0,0};

  //오른쪽 모터 : 1번모터(D2,D3) ,왼쪽모터 2번 모터(D5,D6)
  //int8_t D1 =1;         //2ch 제어기는 D1,D4가 0이 아니면 두 채널 모두 구동(??)

  BYTE D2=R_RPM & 0xff;        //Low data
  BYTE D3=R_RPM>>8 & 0xff;     //high data

  //BYTE D4=1;
  BYTE D5=L_RPM & 0xff;        //Low data
  BYTE D6=L_RPM>>8 & 0xff;     //high data
  //BYTE D7;

  RPM_vel_arr[2]=D2;
  RPM_vel_arr[3]=D3;
  RPM_vel_arr[5]=D5;
  RPM_vel_arr[6]=D6;

  md_write(RPM_vel_arr);

  r_rpm_g = R_RPM;
  l_rpm_g = L_RPM;
  //ROS_INFO("CAN_write");
}

/*
void read_Encoder(int *left_value, int *right_value){

  int r_enc,l_enc;
  BYTE *read_arr=nullptr;

  //CAN_REQ(PID_MONITOR,read_arr);   //오른쪽모터 = 1번모터일경우
  //r_enc = Byte2Int32(read_arr[4],read_arr[5],read_arr[6],read_arr[7]);

  //CAN_REQ(PID_MONITOR2,read_arr);  //왼쪽모터 = 1번모터일경우
  //l_enc = Byte2Int32(read_arr[4],read_arr[5],read_arr[6],read_arr[7]);

  //*left_value = l_enc;
  //*right_value = r_enc;

}

*/

void RT_THREAD::Data_REQ(BYTE R_PID){

//****************REQ***********************************//
  BYTE REQ_Arr[8] = {PID_REQ_PID_DATA, R_PID,0,0,0,0,0,0};
  md_write(REQ_Arr);
//****************REQ***********************************//

}


void RT_THREAD::Encoder_REQ(void){
    Data_REQ(PID_MONITOR);
    Data_REQ(PID_MONITOR2);
}
/*
struct Encoder_data read_Encoder(void){

  struct Encoder_data enc_data;


 // enc_data.R_posi=read_R_Encoder();
//enc_data.L_posi=read_L_Encoder();
  return enc_data;
}*/

struct Encoder_data RT_THREAD::read_Encoder(void){

  struct CAN_data can_data;
  struct Encoder_data enc_data;
  can_data = CAN_read();

  if(can_data.data[0] == PID_MONITOR){

     enc_data.L_posi = Byte2Int32(can_data.data[4],can_data.data[5],can_data.data[6],can_data.data[7]);
  }
  else if(can_data.data[0] == PID_MONITOR2){

     enc_data.R_posi = Byte2Int32(can_data.data[4],can_data.data[5],can_data.data[6],can_data.data[7]);
  }
  else{
    ROS_WARN("Read_Encoder_failed 1");
    //return 0;
  }

  can_data = CAN_read();

  if(can_data.data[0] == PID_MONITOR){
     enc_data.L_posi = Byte2Int32(can_data.data[4],can_data.data[5],can_data.data[6],can_data.data[7]);
  }
  else if(can_data.data[0] == PID_MONITOR2){
     enc_data.R_posi = Byte2Int32(can_data.data[4],can_data.data[5],can_data.data[6],can_data.data[7]);
  }
  else{
    ROS_WARN("Read_Encoder_failed 2");
    //return 0;
  }

  return enc_data;

}

void RT_THREAD::Torque_OFF(void){
  BYTE TQ_OFF[8]={5,0,0,0,0,0,0,0};  //자연정지 PID_TQ_OFF(5번) ,private??
  md_write(TQ_OFF);
  ROS_INFO("Motor_Torque_OFF");
}

void RT_THREAD::Reset_ENC(void){
  BYTE reset[8] = {13,0,0,0,0,0,0,0};
  md_write(reset);
}



/*****************************************************************************
 * value(cmd_vel)값을 받아서 RPM값으로 변환하고 send_RPM 함수를 실행
 * 실제 사용하는 모터구동함수
 ****************************************************************************/

void RT_THREAD::contol_vel(float *cmd_vel){
  float lin_vel = cmd_vel[LINEAR];
  float ang_vel = cmd_vel[ANGULAR];

  short R_wheel_RPM=0,L_wheel_RPM=0;  //signed 16 bit

  R_wheel_RPM = -1*(short)(GEAR_RATIO*30.0*((2*lin_vel) + (wheel_separation*ang_vel))/(2*wheel_radius*PI));
  L_wheel_RPM = (short)(GEAR_RATIO*30.0*((2*lin_vel) - (wheel_separation*ang_vel))/(2*wheel_radius*PI));

  send_RPM(R_wheel_RPM,L_wheel_RPM);

}


//position control
//
void RT_THREAD::posi_control(float mot1_rev, float mot2_rev){

  //1rec : 1,962,480;
  int mot1_tick = (int)(-1*mot1_rev*1962480);
  int mot2_tick = (int)(mot2_rev*1962480);

  BYTE inc_posi_arr1[8]={244,0,0,0,0,0,0,0}; //mot1
  BYTE inc_posi_arr2[8]={251,0,0,0,0,0,0,0}; //mot2


  //오른쪽 모터 : 1번모터(D2,D3) ,왼쪽모터 2번 모터(D5,D6)
  //int8_t D1 =1;         //2ch 제어기는 D1,D4가 0이 아니면 두 채널 모두 구동(??)

  BYTE D1=mot1_tick & 0xff;        //Low data
  BYTE D2=mot1_tick>>8 & 0xff;     //high data
  BYTE D3=mot1_tick>>16 & 0xff;
  BYTE D4=mot1_tick>>24 & 0xff;

  BYTE D11=mot2_tick & 0xff;        //Low data
  BYTE D22=mot2_tick>>8 & 0xff;     //high data
  BYTE D33=mot2_tick>>16 & 0xff;
  BYTE D44=mot2_tick>>24 & 0xff;



  inc_posi_arr1[1]=D1;
  inc_posi_arr1[2]=D2;
  inc_posi_arr1[3]=D3;
  inc_posi_arr1[4]=D4;

  inc_posi_arr2[1]=D11;
  inc_posi_arr2[2]=D22;
  inc_posi_arr2[3]=D33;
  inc_posi_arr2[4]=D44;

  md_write(inc_posi_arr1);
  md_write(inc_posi_arr2);

}

void RT_THREAD::set_posi_maxvel(int max_rpm){

  BYTE max_rpm_arr[8]={176,0,0,0,0,0,0,0}; //mot1


  //오른쪽 모터 : 1번모터(D2,D3) ,왼쪽모터 2번 모터(D5,D6)
  //int8_t D1 =1;         //2ch 제어기는 D1,D4가 0이 아니면 두 채널 모두 구동(??)

  BYTE D1=max_rpm & 0xff;        //Low data
  BYTE D2=max_rpm>>8 & 0xff;     //high data

  max_rpm_arr[1]=D1;
  max_rpm_arr[2]=D2;


  md_write(max_rpm_arr);

}










/********************************************************/
/*********************      IMU      ********************/
/********************************************************/

void RT_THREAD::cmd(int cmd_num){

  /*
   *
   * 1 : - 센서의 구성 파라미터를 플래시 메모리에 저장 ('cmd=1' 또는 'fw')
   * 2 : - 센서의 구성 파라미터를 공장 출하시 설정값으로 초기화 ('cmd=2' 또는 'fd')
   * 3 : - 가속도와 각속도 센서의 바이어스와 스케일 보정 실시 ('cmd=3' 또는 'cal')
   * 4 : - 자기 센서의 바이어스와 스케일 보정 실시 ('cmd=4' 또는 'cam')
   * 5 : - Euler 각도를 0으로 리셋 (‘cmd=5’ 또는 ‘zro’)
   * 9 : - 캘리브레이션 데이터만 리셋 (‘cmd=9’ 또는 ‘rcd’)
   * 99 : - 센서를 소프트웨어 리셋 함 ('cmd=99' 또는 'rst')
   *
   */

  BYTE cmd_arr[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  cmd_arr[0] = 0x18;

  cmd_arr[1] = (7) & 0xff;    //low data
  cmd_arr[2] = (7)>>8 & 0xff; //high data

  cmd_arr[4] = (cmd_num) & 0xff;
  cmd_arr[5] = (cmd_num)>>8 & 0xff;
  cmd_arr[6] = (cmd_num)>>16 & 0xff;
  cmd_arr[7] = (cmd_num)>>24 & 0xff;

  imu_write(cmd_arr);
}

void RT_THREAD::save_params(void){
  cmd(1);
  /*
   * 센서의 구성 파라미터 값을 변경하면, 변경된 값들은 RAM에 기록되어 센서가 켜져 있는 동안만 유지됩니다.
   * 만일 센서가 재시작 되면 이 값들은 소실되고 플래시 메모리에 저장된 값을 RAM으로 다시 읽어 들이게 됩니다.
   * 그렇기 때문에 구성 파라미터 값을 변경하고 계속 유지되기를 바란다면,
   * 'cmd=1' 명령을 사용하여 변경한 구성 파라미터 값을 플래시 메모리에 저장해야 합니다.
   */
}

void RT_THREAD::software_reset(void){
  cmd(99);  //센서를 소프트웨어적으로 리셋 시킵니다. 센서의 전원을 껐다 켜는 것과 같습니다.
}

void RT_THREAD::set_sync_tx_cycle(int cycle_num){
  BYTE cycle_arr[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  cycle_arr[0] = 0x18;

  cycle_arr[1] = (24) & 0xff;    //low data
  cycle_arr[2] = (24)>>8 & 0xff; //high data

  cycle_arr[4] = (cycle_num) & 0xff;
  cycle_arr[5] = (cycle_num)>>8 & 0xff;
  cycle_arr[6] = (cycle_num)>>16 & 0xff;
  cycle_arr[7] = (cycle_num)>>24 & 0xff;

  imu_write(cycle_arr);
}

void RT_THREAD::set_sync_req(bool onoff){
  //CAN 동기화 데이터 전송 설정
  BYTE cmd_arr[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  cmd_arr[0] = 0x18;
  cmd_arr[1] = 22 & 0xff;    //low data
  cmd_arr[2] = 22>>8 & 0xff; //high data
  if(onoff == true){
    cmd_arr[4] = 0x06; //0x07;   //6 : gyro, angle //// 7 : accel, gyro, angle
  }
  else{
    cmd_arr[4] = 0x00;
  }
  imu_write(cmd_arr);
}

void RT_THREAD::read_sync_data(void){
  struct CAN_data imu_can_data;
  int i=0;
  for(i=0;i<2;i++){
    imu_can_data = CAN_read();
    if(imu_can_data.data[1]==51){
      this->acc_x = G * Byte2Int16(imu_can_data.data[2],imu_can_data.data[3])/1000.0;  //m/s^2
      this->acc_y = G * Byte2Int16(imu_can_data.data[4],imu_can_data.data[5])/1000.0;
      this->acc_z = G * Byte2Int16(imu_can_data.data[6],imu_can_data.data[7])/1000.0;
    }
    else if(imu_can_data.data[1]==52){
      this->gyro_x = DEG_2_RAD(Byte2Int16(imu_can_data.data[2],imu_can_data.data[3])/10.0);  //rad/s
      this->gyro_y = DEG_2_RAD(Byte2Int16(imu_can_data.data[4],imu_can_data.data[5])/10.0);
      this->gyro_z = DEG_2_RAD(Byte2Int16(imu_can_data.data[6],imu_can_data.data[7])/10.0);
    }
    else if(imu_can_data.data[1]==53){
      this->angle_x = Byte2Int16(imu_can_data.data[2],imu_can_data.data[3])/100.0;  //deg
      this->angle_y = Byte2Int16(imu_can_data.data[4],imu_can_data.data[5])/100.0;
      this->angle_z = Byte2Int16(imu_can_data.data[6],imu_can_data.data[7])/100.0;
    }
    else{
      ROS_WARN("Something is wrong(read_sync_data) :( ");
    }
  }


}


void RT_THREAD::imu_req(void){
  BYTE imu_req_arr[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  int i=0,j=0;

  //imu_req_arr[0] = 0x3c;//0x18;//0x3c;
  //imu_req_arr[1] = 53 & 0xff;    //low data
  //imu_req_arr[2] = 53>>8 & 0xff; //high data

  //imu_req_arr[3]=0x01;
  //imu_req_arr[4]=0x00;

  //CAN_write(imu_req_arr);
  imu_req_arr[0] = 0x3c;


  for(i=2;i>=1;i--){

    //i=0 : acc
    //i=1 : gyro
    //i=2 : angle

    imu_req_arr[1] = (51+i) & 0xff;    //low data
    imu_req_arr[2] = (51+i)>>8 & 0xff; //high data

    for(j=2;j<=3;j++){

      //j=1 : x
      //j=2 : y
      //j=3 : z
     if(i==1 && j == 2) continue;  //don't need gyro_y
     imu_req_arr[3] = j;
     imu_write(imu_req_arr);
    }
  }
  //imu_write(imu_req_arr);  //마지막걸 한번 더

}
void RT_THREAD::imu_read(void){
  struct CAN_data imu_can_data;

  //imu_can_data = CAN_read();

  //this->angle_x = Byte2float32(imu_can_data.data[4],imu_can_data.data[5],imu_can_data.data[6],imu_can_data.data[7]);

  int i=0;

  for(i=0;i<3;i++){
    imu_can_data = CAN_read();
    if(imu_can_data.data[1]==51){

      if(imu_can_data.data[3]==1) this->acc_x = Byte2float32(imu_can_data.data[4],imu_can_data.data[5],imu_can_data.data[6],imu_can_data.data[7]);
      else if(imu_can_data.data[3]==2) this->acc_y = Byte2float32(imu_can_data.data[4],imu_can_data.data[5],imu_can_data.data[6],imu_can_data.data[7]);
      else if(imu_can_data.data[3]==3) this->acc_z = Byte2float32(imu_can_data.data[4],imu_can_data.data[5],imu_can_data.data[6],imu_can_data.data[7]);
      else ROS_WARN("Something is wrong (acc_read)");

    }

    else if(imu_can_data.data[1]==52){

      if(imu_can_data.data[3]==1) this->gyro_x = DEG_2_RAD(Byte2float32(imu_can_data.data[4],imu_can_data.data[5],imu_can_data.data[6],imu_can_data.data[7]));
      else if(imu_can_data.data[3]==2) this->gyro_y = DEG_2_RAD(Byte2float32(imu_can_data.data[4],imu_can_data.data[5],imu_can_data.data[6],imu_can_data.data[7]));
      else if(imu_can_data.data[3]==3) this->gyro_z = DEG_2_RAD(Byte2float32(imu_can_data.data[4],imu_can_data.data[5],imu_can_data.data[6],imu_can_data.data[7]));
      else ROS_WARN("Something is wrong (gyro_read)");
    }

    else if(imu_can_data.data[1]==53){
      //단위 deg
      if(imu_can_data.data[3]==1) this->angle_x = Byte2float32(imu_can_data.data[4],imu_can_data.data[5],imu_can_data.data[6],imu_can_data.data[7]);
      else if(imu_can_data.data[3]==2) this->angle_y = Byte2float32(imu_can_data.data[4],imu_can_data.data[5],imu_can_data.data[6],imu_can_data.data[7]);
      else if(imu_can_data.data[3]==3) this->angle_z = Byte2float32(imu_can_data.data[4],imu_can_data.data[5],imu_can_data.data[6],imu_can_data.data[7]);
      else ROS_WARN("Something is wrong (angle_read)");

    }
    else{
      ROS_WARN("Something is wrong??");
    }

  }
  //ROS_INFO("angle x : %f",angle_x);
}

void RT_THREAD::angleY_req(void){
  BYTE imu_req_arr[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  imu_req_arr[0] = 0x3c;
  imu_req_arr[1] = (53) & 0xff;    //low data
  imu_req_arr[2] = (53)>>8 & 0xff; //high data
  imu_req_arr[3] = 2;

  imu_write(imu_req_arr);

}
void RT_THREAD::angleZ_req(void){
  BYTE imu_req_arr[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  imu_req_arr[0] = 0x3c;
  imu_req_arr[1] = (53) & 0xff;    //low data
  imu_req_arr[2] = (53)>>8 & 0xff; //high data
  imu_req_arr[3] = 3;

  imu_write(imu_req_arr);
}
void RT_THREAD::gyroZ_req(void){
  BYTE imu_req_arr[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  imu_req_arr[0] = 0x3c;
  imu_req_arr[1] = (52) & 0xff;    //low data
  imu_req_arr[2] = (52)>>8 & 0xff; //high data
  imu_req_arr[3] = 3;

  imu_write(imu_req_arr);
}

void RT_THREAD::imuReadOnce(void){
  struct CAN_data imu_can_data;
  imu_can_data = CAN_read();
  if(imu_can_data.data[1]==51){

    if(imu_can_data.data[3]==1) this->acc_x = Byte2float32(imu_can_data.data[4],imu_can_data.data[5],imu_can_data.data[6],imu_can_data.data[7]);
    else if(imu_can_data.data[3]==2) this->acc_y = Byte2float32(imu_can_data.data[4],imu_can_data.data[5],imu_can_data.data[6],imu_can_data.data[7]);
    else if(imu_can_data.data[3]==3) this->acc_z = Byte2float32(imu_can_data.data[4],imu_can_data.data[5],imu_can_data.data[6],imu_can_data.data[7]);
    else ROS_WARN("Something is wrong (acc_read)");

  }

  else if(imu_can_data.data[1]==52){

    if(imu_can_data.data[3]==1) this->gyro_x = DEG_2_RAD(Byte2float32(imu_can_data.data[4],imu_can_data.data[5],imu_can_data.data[6],imu_can_data.data[7]));
    else if(imu_can_data.data[3]==2) this->gyro_y = DEG_2_RAD(Byte2float32(imu_can_data.data[4],imu_can_data.data[5],imu_can_data.data[6],imu_can_data.data[7]));
    else if(imu_can_data.data[3]==3) this->gyro_z = DEG_2_RAD(Byte2float32(imu_can_data.data[4],imu_can_data.data[5],imu_can_data.data[6],imu_can_data.data[7]));
    else ROS_WARN("Something is wrong (gyro_read)");
  }

  else if(imu_can_data.data[1]==53){
    //단위 deg
    if(imu_can_data.data[3]==1) this->angle_x = Byte2float32(imu_can_data.data[4],imu_can_data.data[5],imu_can_data.data[6],imu_can_data.data[7]);
    else if(imu_can_data.data[3]==2) this->angle_y = Byte2float32(imu_can_data.data[4],imu_can_data.data[5],imu_can_data.data[6],imu_can_data.data[7]);
    else if(imu_can_data.data[3]==3) this->angle_z = Byte2float32(imu_can_data.data[4],imu_can_data.data[5],imu_can_data.data[6],imu_can_data.data[7]);
    else ROS_WARN("Something is wrong (angle_read)");

  }
  else{
    ROS_WARN("Something is wrong??");
  }

}

/***********************************************************
 * 1바이트 D4,D5,D6,D7 데이터를 받아 원 데이터인 4바이트 정수를 만듬
 ***********************************************************/
int RT_THREAD::Byte2Int32(BYTE d4, BYTE d5, BYTE d6, BYTE d7)
{
  return ((int)d4 | (int)d5<<8 | (int)d6<<16 | (int)d7<<24);
}


short RT_THREAD::Byte2Int16(BYTE d1, BYTE d2)
{
  return (short)((short)d1 | (short)d2<<8);
}

float RT_THREAD::Byte2float32(BYTE d4, BYTE d5, BYTE d6, BYTE d7)
{
  float result = 0.0;
  BYTE data[4] = {d4,d5,d6,d7};
  memcpy(&result, &data, sizeof(float));
  return result;
  //return (float)(d4 | d5<<8 | d6<<16 | d7<<24);
}
