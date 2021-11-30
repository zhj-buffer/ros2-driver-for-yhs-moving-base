#include <fcntl.h>
#include <dirent.h>
#include <linux/input.h>
#include <sys/stat.h>
#include <pthread.h>
#include <time.h>
#include <unistd.h>

#include <mutex>

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "std_msgs/msg/string.hpp"

#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/twist.h"
#include "tf2_ros/transform_broadcaster.h"
#include "nav_msgs/msg/odometry.hpp"
#include "ros2_drive_package_msg/msg/ctrl_cmd.hpp"
#include "ros2_drive_package_msg/msg/io_cmd.hpp"
#include "ros2_drive_package_msg/msg/ctrl_fb.hpp"
#include "ros2_drive_package_msg/msg/l_wheel_fb.hpp"
#include "ros2_drive_package_msg/msg/r_wheel_fb.hpp"
#include "ros2_drive_package_msg/msg/io_fb.hpp"
#include "ros2_drive_package_msg/msg/free_ctrl_cmd.hpp"
#include "ros2_drive_package_msg/msg/bms_fb.hpp"
#include "ros2_drive_package_msg/msg/bms_flag_fb.hpp"

#include "ros2_drive_package_can_ctrol/controlcan.h"

using namespace std::chrono_literals;

class ros2_drive_package_can_ctrol: public rclcpp::Node
{
		public:
				ros2_drive_package_can_ctrol()
						: Node("ros2_drive_package_can_ctrol")
				{


						RCLCPP_INFO(this->get_logger(), "Node inited:'");
						init_can();
						ctrl_cmd_sub_ = this->create_subscription<ros2_drive_package_msg::msg::CtrlCmd>("ctrl_cmd", 5, std::bind(&ros2_drive_package_can_ctrol::ctrl_cmdCallBack, this, std::placeholders::_1));
						io_cmd_sub_ = this->create_subscription<ros2_drive_package_msg::msg::IoCmd>("io_cmd", 5, std::bind(&ros2_drive_package_can_ctrol::io_cmdCallBack, this, std::placeholders::_1));
						free_ctrl_cmd_sub_ = this->create_subscription<ros2_drive_package_msg::msg::FreeCtrlCmd>("rear_free_ctrl_cmd", 5, std::bind(&ros2_drive_package_can_ctrol::free_ctrl_cmdCallBack, this, std::placeholders::_1));

						ctrl_fb_pub_ = this->create_publisher<ros2_drive_package_msg::msg::CtrlFb>("ctrl_fb",5);
						io_fb_pub_ = this->create_publisher<ros2_drive_package_msg::msg::IoFb>("io_fb",5);
						r_wheel_fb_pub_ = this->create_publisher<ros2_drive_package_msg::msg::RWheelFb>("r_wheel_fb",5);
						l_wheel_fb_pub_ = this->create_publisher<ros2_drive_package_msg::msg::LWheelFb>("l_wheel_fb",5);
						bms_fb_pub_ = this->create_publisher<ros2_drive_package_msg::msg::BmsFb>("bms_fb",5);
						bms_flag_fb_pub_ = this->create_publisher<ros2_drive_package_msg::msg::BmsFlagFb>("bms_flag_fb",5);

						timer_ = this->create_wall_timer(100ms, std::bind(&ros2_drive_package_can_ctrol::timer_callback, this));
				}

				void io_cmdCallBack(const ros2_drive_package_msg::msg::IoCmd::SharedPtr msg);
				void ctrl_cmdCallBack(const ros2_drive_package_msg::msg::CtrlCmd::SharedPtr msg);
				void free_ctrl_cmdCallBack(const ros2_drive_package_msg::msg::FreeCtrlCmd::SharedPtr msg);
				void recvData();
				void sendData();
				void init_can();

		private:
				rclcpp::Publisher<ros2_drive_package_msg::msg::CtrlFb>::SharedPtr ctrl_fb_pub_;
				rclcpp::Publisher<ros2_drive_package_msg::msg::LWheelFb>::SharedPtr l_wheel_fb_pub_;
				rclcpp::Publisher<ros2_drive_package_msg::msg::RWheelFb>::SharedPtr r_wheel_fb_pub_;
				rclcpp::Publisher<ros2_drive_package_msg::msg::IoFb>::SharedPtr io_fb_pub_;
				rclcpp::Publisher<ros2_drive_package_msg::msg::BmsFb>::SharedPtr bms_fb_pub_;
				rclcpp::Publisher<ros2_drive_package_msg::msg::BmsFlagFb>::SharedPtr bms_flag_fb_pub_;

				rclcpp::Subscription<ros2_drive_package_msg::msg::CtrlCmd>::SharedPtr ctrl_cmd_sub_;
				rclcpp::Subscription<ros2_drive_package_msg::msg::IoCmd>::SharedPtr io_cmd_sub_;
				rclcpp::Subscription<ros2_drive_package_msg::msg::FreeCtrlCmd>::SharedPtr free_ctrl_cmd_sub_;

				rclcpp::TimerBase::SharedPtr timer_;
				void timer_callback()
				{
						recvData();
				}

				std::mutex cmd_mutex_;

				unsigned char sendData_u_io_[8] = {0};
				unsigned char sendData_u_vel_[8] = {0};

				VCI_CAN_OBJ send[1];

};
void ros2_drive_package_can_ctrol::io_cmdCallBack(const ros2_drive_package_msg::msg::IoCmd::SharedPtr msg)
{
		static unsigned char count_1 = 0;

		this->cmd_mutex_.lock();

		memset(sendData_u_io_,0,8);

		sendData_u_io_[0] = 0xff;
		if(msg->io_cmd_lamp_ctrl)
				sendData_u_io_[0] &= 0xff;
		else sendData_u_io_[0] &= 0xfe;
		if(msg->io_cmd_unlock)
				sendData_u_io_[0] &= 0xff;
		else sendData_u_io_[0] &= 0xfd;

		sendData_u_io_[1] = 0xff;
		if(msg->io_cmd_lower_beam_headlamp)
				sendData_u_io_[1] &= 0xff;
		else sendData_u_io_[1] &= 0xfe;
		if(msg->io_cmd_upper_beam_headlamp)
				sendData_u_io_[1] &= 0xff;
		else sendData_u_io_[1] &= 0xfd;

		if(msg->io_cmd_turn_lamp == 0)
				sendData_u_io_[1] &= 0xf3;
		if(msg->io_cmd_turn_lamp == 1)
				sendData_u_io_[1] &= 0xf7;
		if(msg->io_cmd_turn_lamp == 2)
				sendData_u_io_[1] &= 0xfb;

		if(msg->io_cmd_braking_lamp)
				sendData_u_io_[1] &= 0xff;
		else sendData_u_io_[1] &= 0xef;
		if(msg->io_cmd_clearance_lamp)
				sendData_u_io_[1] &= 0xff;
		else sendData_u_io_[1] &= 0xdf;
		if(msg->io_cmd_fog_lamp)
				sendData_u_io_[1] &= 0xff;
		else sendData_u_io_[1] &= 0xbf;

		sendData_u_io_[2] = msg->io_cmd_speaker;

		sendData_u_io_[3] = 0;
		sendData_u_io_[4] = 0;
		sendData_u_io_[5] = 0;

		count_1 ++;
		if(count_1 == 16)	count_1 = 0;

		sendData_u_io_[6] =  count_1 << 4;

		sendData_u_io_[7] = sendData_u_io_[0] ^ sendData_u_io_[1] ^ sendData_u_io_[2] ^ sendData_u_io_[3] ^ sendData_u_io_[4] ^ sendData_u_io_[5] ^ sendData_u_io_[6];

		send[0].ID=0x18C4D7D0;

		memcpy(send[0].Data,sendData_u_io_,8);

		if(VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1)
		{
		}	

		RCLCPP_INFO(this->get_logger(), "CAN Datasend %s ID: %x\n", send[0].Data, send[0].ID );
		this->cmd_mutex_.unlock();
}

//
void ros2_drive_package_can_ctrol::ctrl_cmdCallBack(const ros2_drive_package_msg::msg::CtrlCmd::SharedPtr msg)
{
		short linear = msg->ctrl_cmd_linear * 1000;
		short angular = msg->ctrl_cmd_angular * 100;
		static unsigned char count = 0;

		this->cmd_mutex_.lock();

		memset(sendData_u_vel_,0,8);

		sendData_u_vel_[0] = sendData_u_vel_[0] | (0x0f & msg->ctrl_cmd_gear);

		sendData_u_vel_[0] = sendData_u_vel_[0] | (0xf0 & ((linear & 0x0f) << 4));

		sendData_u_vel_[1] = (linear >> 4) & 0xff;

		sendData_u_vel_[2] = sendData_u_vel_[2] | (0x0f & (linear >> 12));


		sendData_u_vel_[2] = sendData_u_vel_[2] | (0xf0 & ((angular & 0x0f) << 4));

		sendData_u_vel_[3] = (angular >> 4) & 0xff;

		sendData_u_vel_[4] = sendData_u_vel_[4] | (0x0f & (angular >> 12));


		count ++;

		if(count == 16)	count = 0;

		sendData_u_vel_[6] =  count << 4;


		sendData_u_vel_[7] = sendData_u_vel_[0] ^ sendData_u_vel_[1] ^ sendData_u_vel_[2] ^ sendData_u_vel_[3] ^ sendData_u_vel_[4] ^ sendData_u_vel_[5] ^ sendData_u_vel_[6];

		send[0].ID = 0x18C4D1D0;

		memcpy(send[0].Data,sendData_u_vel_,8);

		if(VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1)
		{
		}

		RCLCPP_INFO(this->get_logger(), "CAN Datasend %s ID: %x\n", send[0].Data, send[0].ID );
		this->cmd_mutex_.unlock();
}

//
void ros2_drive_package_can_ctrol::free_ctrl_cmdCallBack(const ros2_drive_package_msg::msg::FreeCtrlCmd::SharedPtr msg)
{
		short linearl = msg->free_ctrl_cmd_velocity_l * 1000;
		short linearr = msg->free_ctrl_cmd_velocity_r * 1000;
		static unsigned char count_3 = 0;

		unsigned char sendData_u_tem_[8] = {0};

		this->cmd_mutex_.lock();

		sendData_u_tem_[0] = sendData_u_tem_[0] | (0x0f & msg->ctrl_cmd_gear);

		sendData_u_tem_[0] = sendData_u_tem_[0] | (0xf0 & ((linearl & 0x0f) << 4));

		sendData_u_tem_[1] = (linearl >> 4) & 0xff;

		sendData_u_tem_[2] = sendData_u_tem_[2] | (0x0f & (linearl >> 12));


		sendData_u_tem_[2] = sendData_u_tem_[2] | (0xf0 & ((linearr & 0x0f) << 4));

		sendData_u_tem_[3] = (linearr >> 4) & 0xff;

		sendData_u_tem_[4] = sendData_u_tem_[4] | (0x0f & (linearr >> 12));

		count_3 ++;

		if(count_3 == 16)	count_3 = 0;

		sendData_u_tem_[6] =  count_3 << 4;


		sendData_u_tem_[7] = sendData_u_tem_[0] ^ sendData_u_tem_[1] ^ sendData_u_tem_[2] ^ sendData_u_tem_[3] ^ sendData_u_tem_[4] ^ sendData_u_tem_[5] ^ sendData_u_tem_[6];

		send[0].ID = 0x18C4D2D0;

		memcpy(send[0].Data,sendData_u_tem_,8);

		if(VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1)
		{
		}	
		
		RCLCPP_INFO(this->get_logger(), "CAN Datasend %s ID: %x\n", send[0].Data, send[0].ID );
		this->cmd_mutex_.unlock();
}


//数据接收解析线程
void ros2_drive_package_can_ctrol::recvData()
{

		int reclen=0;
		int j;
		int ind=0;
		VCI_CAN_OBJ rec[3000];//接收缓存，设为3000为佳。


		if((reclen=VCI_Receive(VCI_USBCAN2,0,ind,rec,3000,100))>0)//调用接收函数，如果有数据，进行数据处理显示。
		{
				for(j=0;j<reclen;j++)
				{
						RCLCPP_INFO(this->get_logger(), "CAN recvData ID: %x \n",rec[j].ID);
						switch (rec[j].ID)
						{
								case 0x18C4D1EF:
										{
												ros2_drive_package_msg::msg::CtrlFb msg;
												msg.ctrl_fb_target_gear = 0x0f & rec[j].Data[0];

												msg.ctrl_fb_linear = (float)((short)((rec[j].Data[2] & 0x0f) << 12 | rec[j].Data[1] << 4 | (rec[j].Data[0] & 0xf0) >> 4)) / 1000;

												msg.ctrl_fb_angular = (float)((short)((rec[j].Data[4] & 0x0f) << 12 | rec[j].Data[3] << 4 | (rec[j].Data[2] & 0xf0) >> 4)) / 100;

												unsigned char crc = rec[j].Data[0] ^ rec[j].Data[1] ^ rec[j].Data[2] ^ rec[j].Data[3] ^ rec[j].Data[4] ^ rec[j].Data[5] ^ rec[j].Data[6];

												if(crc == rec[j].Data[7])
												{

														ctrl_fb_pub_->publish(msg);
												}

												break;
										}

								case 0x18C4D7EF:
										{
												ros2_drive_package_msg::msg::LWheelFb msg;
												msg.l_wheel_fb_velocity = (float)((short)(rec[j].Data[1] << 8 | rec[j].Data[0])) / 1000;

												msg.l_wheel_fb_pulse = (int)(rec[j].Data[5] << 24 | rec[j].Data[4] << 16 | rec[j].Data[3] << 8 | rec[j].Data[2]);

												unsigned char crc = rec[j].Data[0] ^ rec[j].Data[1] ^ rec[j].Data[2] ^ rec[j].Data[3] ^ rec[j].Data[4] ^ rec[j].Data[5] ^ rec[j].Data[6];

												if(crc == rec[j].Data[7])
												{

														l_wheel_fb_pub_->publish(msg);
												}

												break;
										}

								case 0x18C4D8EF:
										{
												ros2_drive_package_msg::msg::RWheelFb msg;
												msg.r_wheel_fb_velocity = (float)((short)(rec[j].Data[1] << 8 | rec[j].Data[0])) / 1000;

												msg.r_wheel_fb_pulse = (int)(rec[j].Data[5] << 24 | rec[j].Data[4] << 16 | rec[j].Data[3] << 8 | rec[j].Data[2]);

												unsigned char crc = rec[j].Data[0] ^ rec[j].Data[1] ^ rec[j].Data[2] ^ rec[j].Data[3] ^ rec[j].Data[4] ^ rec[j].Data[5] ^ rec[j].Data[6];

												if(crc == rec[j].Data[7])
												{

														r_wheel_fb_pub_->publish(msg);
												}

												break;
										}

								case 0x18C4DAEF:
										{
												ros2_drive_package_msg::msg::IoFb msg;
												if(0x01 & rec[j].Data[0]) msg.io_fb_lamp_ctrl = true;	else msg.io_fb_lamp_ctrl = false;

												if(0x02 & rec[j].Data[1]) msg.io_fb_unlock = true;	else msg.io_fb_unlock = false;

												if(0x01 & rec[j].Data[1]) msg.io_fb_lower_beam_headlamp = true;	else msg.io_fb_lower_beam_headlamp = false;

												if(0x02 & rec[j].Data[1]) msg.io_fb_upper_beam_headlamp = true;	else msg.io_fb_upper_beam_headlamp = false;

												msg.io_fb_turn_lamp = (0xc0 & rec[j].Data[1]) >> 2;

												if(0x10 & rec[j].Data[1]) msg.io_fb_braking_lamp = true;	else msg.io_fb_braking_lamp = false;

												if(0x20 & rec[j].Data[1]) msg.io_fb_clearance_lamp = true;	else msg.io_fb_clearance_lamp = false;

												if(0x40 & rec[j].Data[1]) msg.io_fb_fog_lamp = true;	else msg.io_fb_fog_lamp = false;

												if(0x01 & rec[j].Data[2]) msg.io_fb_speaker = true;	else msg.io_fb_speaker = false;

												if(0x01 & rec[j].Data[3]) msg.io_fb_fl_impact_sensor = true;	else msg.io_fb_fl_impact_sensor = false;

												if(0x02 & rec[j].Data[3]) msg.io_fb_fm_impact_sensor = true;	else msg.io_fb_fm_impact_sensor = false;

												if(0x04 & rec[j].Data[3]) msg.io_fb_fr_impact_sensor = true;	else msg.io_fb_fr_impact_sensor = false;

												if(0x08 & rec[j].Data[3]) msg.io_fb_rl_impact_sensor = true;	else msg.io_fb_rl_impact_sensor = false;

												if(0x10 & rec[j].Data[3]) msg.io_fb_rm_impact_sensor = true;	else msg.io_fb_rm_impact_sensor = false;

												if(0x20 & rec[j].Data[3]) msg.io_fb_rr_impact_sensor = true;	else msg.io_fb_rr_impact_sensor = false;

												if(0x01 & rec[j].Data[4]) msg.io_fb_fl_drop_sensor = true;	else msg.io_fb_fl_drop_sensor = false;

												if(0x02 & rec[j].Data[4]) msg.io_fb_fm_drop_sensor = true;	else msg.io_fb_fm_drop_sensor = false;

												if(0x04 & rec[j].Data[4]) msg.io_fb_fr_drop_sensor = true;	else msg.io_fb_fr_drop_sensor = false;

												if(0x08 & rec[j].Data[4]) msg.io_fb_rl_drop_sensor = true;	else msg.io_fb_rl_drop_sensor = false;

												if(0x10 & rec[j].Data[4]) msg.io_fb_rm_drop_sensor = true;	else msg.io_fb_rm_drop_sensor = false;

												if(0x20 & rec[j].Data[4]) msg.io_fb_rr_drop_sensor = true;	else msg.io_fb_rr_drop_sensor = false;

												if(0x01 & rec[j].Data[5]) msg.io_fb_estop = true;	else msg.io_fb_estop = false;

												if(0x02 & rec[j].Data[5]) msg.io_fb_joypad_ctrl = true;	else msg.io_fb_joypad_ctrl = false;

												if(0x04 & rec[j].Data[5]) msg.io_fb_charge_state = true;	else msg.io_fb_charge_state = false;

												unsigned char crc = rec[j].Data[0] ^ rec[j].Data[1] ^ rec[j].Data[2] ^ rec[j].Data[3] ^ rec[j].Data[4] ^ rec[j].Data[5] ^ rec[j].Data[6];

												if(crc == rec[j].Data[7])
												{

														io_fb_pub_->publish(msg);
												}

												break;
										}


										//bms反馈
								case 0x18C4E1EF:
										{
												ros2_drive_package_msg::msg::BmsFb msg;
												msg.bms_fb_voltage = (float)((unsigned short)(rec[j].Data[1] << 8 | rec[j].Data[0])) / 100;

												msg.bms_fb_current = (float)((short)(rec[j].Data[3] << 8 | rec[j].Data[2])) / 100;

												msg.bms_fb_remaining_capacity = (float)((unsigned short)(rec[j].Data[5] << 8 | rec[j].Data[4])) / 100;

												unsigned char crc = rec[j].Data[0] ^ rec[j].Data[1] ^ rec[j].Data[2] ^ rec[j].Data[3] ^ rec[j].Data[4] ^ rec[j].Data[5] ^ rec[j].Data[6];

												if(crc == rec[j].Data[7])
												{

														bms_fb_pub_->publish(msg);
												}

												break;
										}

										//bms_flag反馈
								case 0x18C4E2EF:
										{
												ros2_drive_package_msg::msg::BmsFlagFb msg;
												msg.bms_flag_fb_soc = rec[j].Data[0];

												if(0x01 & rec[j].Data[1]) msg.bms_flag_fb_single_ov = true;	else msg.bms_flag_fb_single_ov = false;

												if(0x02 & rec[j].Data[1]) msg.bms_flag_fb_single_uv = true;	else msg.bms_flag_fb_single_uv = false;

												if(0x04 & rec[j].Data[1]) msg.bms_flag_fb_ov = true;	else msg.bms_flag_fb_ov = false;

												if(0x08 & rec[j].Data[1]) msg.bms_flag_fb_uv = true;	else msg.bms_flag_fb_uv = false;

												if(0x10 & rec[j].Data[1]) msg.bms_flag_fb_charge_ot = true;	else msg.bms_flag_fb_charge_ot = false;

												if(0x20 & rec[j].Data[1]) msg.bms_flag_fb_charge_ut = true;	else msg.bms_flag_fb_charge_ut = false;

												if(0x40 & rec[j].Data[1]) msg.bms_flag_fb_discharge_ot = true;	else msg.bms_flag_fb_discharge_ot = false;

												if(0x80 & rec[j].Data[1]) msg.bms_flag_fb_discharge_ut = true;	else msg.bms_flag_fb_discharge_ut = false;

												if(0x01 & rec[j].Data[2]) msg.bms_flag_fb_charge_oc = true;	else msg.bms_flag_fb_charge_oc = false;

												if(0x02 & rec[j].Data[2]) msg.bms_flag_fb_discharge_oc = true;	else msg.bms_flag_fb_discharge_oc = false;

												if(0x04 & rec[j].Data[2]) msg.bms_flag_fb_short = true;	else msg.bms_flag_fb_short = false;

												if(0x08 & rec[j].Data[2]) msg.bms_flag_fb_ic_error = true;	else msg.bms_flag_fb_ic_error = false;

												if(0x10 & rec[j].Data[2]) msg.bms_flag_fb_lock_mos = true;	else msg.bms_flag_fb_lock_mos = false;

												if(0x20 & rec[j].Data[2]) msg.bms_flag_fb_charge_flag = true;	else msg.bms_flag_fb_charge_flag = false;

												msg.bms_flag_fb_hight_temperature = (float)((short)(rec[j].Data[4] << 4 | rec[j].Data[3] >> 4)) / 10;

												msg.bms_flag_fb_low_temperature = (float)((short)((rec[j].Data[6] & 0x0f) << 8 | rec[j].Data[5])) / 10;

												unsigned char crc = rec[j].Data[0] ^ rec[j].Data[1] ^ rec[j].Data[2] ^ rec[j].Data[3] ^ rec[j].Data[4] ^ rec[j].Data[5] ^ rec[j].Data[6];

												if(crc == rec[j].Data[7])
												{

														bms_flag_fb_pub_->publish(msg);
												}
												break;
										}
								default:
										break;
						}
				}
		}
}

//数据发送线程
void ros2_drive_package_can_ctrol::sendData()
{
		//复位CAN1通道。
		VCI_ResetCAN(VCI_USBCAN2, 0, 0);
		usleep(100000);

		//关闭设备。
		VCI_CloseDevice(VCI_USBCAN2,0);	
		RCLCPP_INFO(this->get_logger(), "send Data'");
}

void ros2_drive_package_can_ctrol::init_can()
{
		RCLCPP_INFO(this->get_logger(), "Open Can device\n'");
		//打开设备
		if(VCI_OpenDevice(VCI_USBCAN2,0,0)==1)
		{
				RCLCPP_INFO(this->get_logger(), ">>open can deivce success!");
		}
		else
		{
				RCLCPP_INFO(this->get_logger(), ">>open can deivce error!");
				return;
		}


		//初始化参数，严格参数二次开发函数库说明书。
		VCI_INIT_CONFIG config;
		config.AccCode=0;
		config.AccMask=0xFFFFFFFF;
		//接收所有帧
		config.Filter=1;
		/*波特率500 Kbps  0x00  0x1C*/
		config.Timing0=0x00;
		config.Timing1=0x1C;
		//正常模式	
		config.Mode=0;	

		send[0].SendType=1;
		send[0].RemoteFlag=0;
		send[0].ExternFlag=1;
		send[0].DataLen=8;

		bool error = false;
		if(VCI_InitCAN(VCI_USBCAN2,0,0,&config)!=1)
		{
				RCLCPP_INFO(this->get_logger(), ">>Init CAN1 error!\n");
				error = true;
		}
		if(VCI_StartCAN(VCI_USBCAN2,0,0)!=1)
		{
				RCLCPP_INFO(this->get_logger(), ">>Start CAN1 error!\n");
				error = true;
		}

		if (error == false)
			RCLCPP_INFO(this->get_logger(), "CAN setup successfully'\n");
}


int main(int argc, char ** argv)
{

	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ros2_drive_package_can_ctrol>());
	rclcpp::shutdown();

	return 0;
}
