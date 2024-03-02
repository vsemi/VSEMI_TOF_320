#include "ctrl.h"

#include "ctrl.h"

#include <stdio.h>
#include <stdio.h>
#include <iomanip>
#include <ios>
#include <iostream>
#include <sstream>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "socketcan.h"
#include "serial.hpp"

extern bool ros_initialized;

extern bool distance_ctrl;

extern bool truck_behind;
extern bool balanced_fill;

extern bool falling_found;
extern bool dest_found;
extern bool x_in_boundary;
extern bool y_in_boundary;

extern int ctrl_signal_life_count;
extern int ctrl_signal_life_assigned;

extern int ctrl_current_ctrl_type;

extern float sprout_angle_Z;
extern float sprout_angle_available;

extern int sprout_y_moves;
extern float truck_center_distance_estimated;

extern pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_fall_point;
extern pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_dest_point;

extern pcl::PointXYZRGBL display_ctr_p1;
extern pcl::PointXYZRGBL display_ctr_p2;
extern bool display_ctrl_available;

extern scpp::SocketCan socket_can;
extern SerialConnection serial;

extern int comm_ctrl_type;
extern bool comm_ctrl_enabled;

extern void _pause_camera(bool p);

extern std::string error_msg;

int control_x = 0x00;
int control_y = 0x00;

std::chrono::steady_clock::time_point signal_last_sent_time;

void update_ctrl_signal()
{
	//std::cout << "===> update_ctrl_signal ... " << std::endl;
	//std::cout << "         ===> falling_found: " << falling_found << std::endl;
	//std::cout << "         ===> dest_found: " << dest_found << std::endl;
	//std::cout << "         ===> x_in_boundary: " << x_in_boundary << std::endl;
	//std::cout << "         ===> ctrl_signal_life_count: " << ctrl_signal_life_count << std::endl;

	int ctr_x = 0x00;
	int ctr_y = 0x00;
	int ctrl_type = -1;

	if (dest_found)
	{
		std::chrono::steady_clock::time_point cur_time = std::chrono::steady_clock::now();
		double interval = ((double) std::chrono::duration_cast<std::chrono::microseconds>(cur_time - signal_last_sent_time).count()) / 1000000.0;
		
		pcl::PointXYZRGBL p1 = cloud_fall_point->points[0];
		pcl::PointXYZRGBL p2 = cloud_dest_point->points[0];

		float dx = std::fabs(p2.x - p1.x);
		float dy = std::fabs(p2.y - p1.y);
		//std::cout << "   falling point to target dx: " << dx << std::endl;
		//std::cout << "   falling point to target dy: " << dy << std::endl;	
		//std::cout << "   distance_ctrl: " << distance_ctrl << " falling_found: " << falling_found << std::endl;	
		//std::cout << "   x_in_boundary: " << x_in_boundary << std::endl;
		//std::cout << "   y_in_boundary: " << y_in_boundary << std::endl;

		if (distance_ctrl && dy >= CTRL_SIGNAL_MIN_DISTANCE_Y && dx < CTRL_SIGNAL_MIN_DISTANCE_X) 
		{
			ctr_y = 0x02;
			if (p2.y > p1.y)
			{
				ctr_y = 0x01;
			}
		}
		
		if (ctr_y != 0x00) 
		{
			ctrl_type = 1;
		}

		if (
			(dx > CTRL_SIGNAL_MIN_DISTANCE_X) || 
			((! x_in_boundary) && dx > 0.25 * CTRL_SIGNAL_MIN_DISTANCE_X && dy < 1.5 * CTRL_SIGNAL_MIN_DISTANCE_Y) ||
			((! x_in_boundary) && dx <= 0.25 * CTRL_SIGNAL_MIN_DISTANCE_X && dy < CTRL_SIGNAL_MIN_DISTANCE_Y)
		)
		{
			ctr_x = 0x80;
			if (p2.x > p1.x)
			{
				ctr_x = 0x40;
			}
		} 
		
		if (ctr_x != 0x00)
		{
			ctrl_type = 0;
			ctr_y = 0x00; // moving x takes priority
		}
		//std::cout << "ctr_x: " << ctr_x << std::endl;
		//std::cout << "ctr_y: " << ctr_y << std::endl;
		//std::cout << "ctrl_type: " << ctrl_type << std::endl;
		//std::cout << "control_x: " << control_x << std::endl;
		//std::cout << "control_y: " << control_y << std::endl;
		//std::cout << "ctrl_current_ctrl_type: " << ctrl_current_ctrl_type << std::endl;

		if (((ctr_x != 0x00 || ctr_y != 0x00)) && (ctr_x == control_x && ctr_y == control_y && ctrl_type == ctrl_current_ctrl_type)) // same direction, only reset life time, no update
		{
			//std::cout << "No need to update signal as the same, ctrl_signal_life_count remains: " << ctrl_signal_life_count << std::endl;
			int ev_life_count = 0;
			if (ctrl_type == 0) 
			{
				if (truck_behind)
				{
					ev_life_count = CTRL_SIGNAL_START_X + 0.5 * CTRL_SIGNAL_REPEAT_PER_METER_X * dx; // when truck behind, need less time
				} else 
				{
					ev_life_count = CTRL_SIGNAL_START_X + CTRL_SIGNAL_REPEAT_PER_METER_X * dx;
				}
			}
			else if (ctrl_type == 1 && (! y_in_boundary)) 
			{
				ev_life_count = CTRL_SIGNAL_START_Y + CTRL_SIGNAL_REPEAT_PER_METER_Y;// * dy; // just sending once 
			}
			
			if (ev_life_count > 0)
			{
				ctrl_signal_life_assigned = ctrl_signal_life_count = ev_life_count;
				//if (ctrl_current_ctrl_type == 1)
				//{
					//std::cout << "      ======> update signal life time for distance adjustment: " << std::endl;
					//std::cout << "         ===> x_in_boundary: " << x_in_boundary << std::endl;
					//std::cout << "         ===> falling point to target dx: " << dx << std::endl;
					//std::cout << "         ===> falling point to target dy: " << dy << std::endl;	
					//std::cout << "         ===> distance_ctrl: " << distance_ctrl << " falling_found: " << falling_found << std::endl;	
					//std::cout << "      ===> ctrl_current_ctrl_type reset to: " << ctrl_current_ctrl_type << std::endl;
					//std::cout << "         ===> ctrl_signal_life_count reset to: " << ctrl_signal_life_assigned << std::endl;
				//}

				//display needs update
				display_ctr_p1 = p1;
				display_ctr_p2 = p2;

				display_ctrl_available = true;
			}
		} else if ((ctr_x != 0x00 || ctr_y != 0x00) && ctrl_type != ctrl_current_ctrl_type)
		{
			// check time interval
			if (interval > CTRL_SIGNAL_CHANGE_DIRECTION_MIN_PERIOD) // to allow changing direction after 0.5 second, even when not in boundary, because need time to break
			{
				int ev_life_count = 0;
				if (ctrl_type == 0) 
				{
					ev_life_count = CTRL_SIGNAL_START_X + CTRL_SIGNAL_REPEAT_PER_METER_X * dx;
				}
				else if (ctrl_type == 1 && (! y_in_boundary)) 
				{
					ev_life_count = CTRL_SIGNAL_START_Y + CTRL_SIGNAL_REPEAT_PER_METER_Y; // * dy; // just sending once 
				}
				
				if (ev_life_count > 0)
				{					
					control_x = ctr_x;
					control_y = ctr_y;
					ctrl_current_ctrl_type = ctrl_type;
					if (ctrl_current_ctrl_type == 0 && (! truck_behind)) // for distance control, wait for the second signal to confirm its move
					{
						ctrl_signal_life_assigned = ctrl_signal_life_count = ev_life_count;

						display_ctr_p1 = p1;
						display_ctr_p2 = p2;
						display_ctrl_available = true;
					} else if (ctrl_current_ctrl_type == 1 && (! falling_found)) // means triggered by truck distance other than falling location, otherwise if by falling, to be double confirmed
					{
						ctrl_signal_life_assigned = ctrl_signal_life_count = ev_life_count;

						display_ctr_p1 = p1;
						display_ctr_p2 = p2;
						display_ctrl_available = true;
					}
					//if (ctrl_current_ctrl_type == 1)
					//{
						//std::cout << "   New signal for distance adjustment created ======================================>" << std::endl;
						//std::cout << "      ===> x_in_boundary: " << x_in_boundary << std::endl;
						//std::cout << "      ===> falling point to target dx: " << dx << std::endl;
						//std::cout << "      ===> falling point to target dy: " << dy << std::endl;	
						//std::cout << "      ===> distance_ctrl: " << distance_ctrl << " falling_found: " << falling_found << std::endl;	
						//std::cout << "      ===> ctrl_current_ctrl_type reset to: " << ctrl_current_ctrl_type << std::endl;
						//std::cout << "      ===> ctrl_signal_life_count reset to: " << ctrl_signal_life_assigned << std::endl;
					//}
				}
			} else // should stop
			{
				// if already stopped, no need to do anything, otherwise ...
				if (ctrl_current_ctrl_type >= 0 && ctrl_signal_life_count > 0) // ctrl_signal_life_count equals 0 to be fine, as it is stop signal, only need to reset when it greater than 0
				{
					ctrl_signal_life_count = 0;
					ctrl_signal_life_assigned = 0;
					ctrl_current_ctrl_type = -1;
					control_x = 0x00;
					control_y = 0x00;
				}
			}
		} else if (ctrl_type < 0 && ctrl_current_ctrl_type >= 0 && ctrl_signal_life_count > 0)
		{
			//std::cout << "         ===> needs to stop as no need to move" << std::endl;
			int ctrl_signal_age = ctrl_signal_life_assigned - ctrl_signal_life_count;
			//std::cout << "         ===>ctrl_signal_age: " << ctrl_signal_age << std::endl;
			if (ctrl_signal_life_count > 0 && ctrl_signal_age >= CTRL_SIGNAL_START_X)
			{
				ctrl_signal_life_count = 0;
				ctrl_signal_life_assigned = 0;
				ctrl_current_ctrl_type = -1;
				control_x = 0x00;
				control_y = 0x00;
				//std::cout << "            ===> reset ctrl_signal_life_count: " << ctrl_signal_life_count << std::endl;
			} else if (ctrl_signal_life_count > (CTRL_SIGNAL_START_X - ctrl_signal_age)) 
			{
				int s_l = ctrl_signal_life_count - (CTRL_SIGNAL_START_X - ctrl_signal_age);
				ctrl_signal_life_count = CTRL_SIGNAL_START_X - ctrl_signal_age;
				ctrl_signal_life_assigned -= s_l;
				//std::cout << "            ===> reduced ctrl_signal_life_count: " << ctrl_signal_life_count << std::endl;
				if (ctrl_signal_life_count < 0)
				{
					ctrl_signal_life_count = 0;
					ctrl_signal_life_assigned = 0;
					ctrl_current_ctrl_type = -1;
					control_x = 0x00;
					control_y = 0x00;
					//std::cout << "            ===> reduced but non left: " << ctrl_signal_life_count << std::endl;
				}
			} else 
			{
				//std::cout << "         ===> nothing to do ...  " << std::endl;
			}
			//std::cout << "      ===> ctrl_signal_life_count: " << ctrl_signal_life_count << std::endl;
		} else 
		{
			if ((ctrl_current_ctrl_type == 1 || (truck_behind && ctrl_current_ctrl_type >= 0)) && ctrl_signal_life_count == -1)
			{
				ctrl_signal_life_count = 0;
				ctrl_signal_life_assigned = 0;
				ctrl_current_ctrl_type = -1;
				control_x = 0x00;
				control_y = 0x00;
				//std::cout << "         ===>unable to confirm the distance movement, cancel the created but not started signal. " << std::endl;
			}
		}
	}
	//std::cout << "2. ctrl_signal_life_count: " << ctrl_signal_life_count << std::endl;
}

void ctrl_rcv()
{
	std::cout << "Ctrl signal listener starting ... " << std::endl;
	while (ros_initialized && comm_ctrl_enabled)
	{
		try {			
			uint8_t data [6]; 
			int ctrl_mode = 0;
			// to read data from comm
			//std::cout << "   ===> comm_ctrl_type: " << comm_ctrl_type << std::endl;
			
			if (comm_ctrl_type == 1)
			{					
				int len = serial.readData(6);
				//std::cout << "   ===> len: " << len << std::endl;
				
				if (len > 0)
				{
					// data ready
					if (serial.data_array[3] == 0x01)
					{
						ctrl_mode = 1;
					}
					else if (serial.data_array[3] == 0x03)
					{
						ctrl_mode = 3;
					}
					else if (serial.data_array[3] == 0x04)
					{
						ctrl_mode = 4;
					}

					data[0] = serial.data_array[5];
					data[1] = serial.data_array[6];
					data[2] = serial.data_array[7];
					data[3] = serial.data_array[8];
					data[4] = serial.data_array[9];
					data[5] = serial.data_array[10];
				}
			}
			
			else if (comm_ctrl_type == 2)
			{
				scpp::CanFrame fr;
				if (socket_can.read(fr) == scpp::STATUS_OK)
				{
					std::cout << "=========> receiving CAN ID: " << fr.id << std::endl;
					// data ready
					if (fr.id == CAN_ID_SYSTEM)
					{
						ctrl_mode = 1;
					}
					else if (fr.id == CAN_ID_MANUAL_INTERFERED)
					{
						ctrl_mode = 3;
					}
					else if (fr.id == CAN_ID_ANGLES)
					{
						ctrl_mode = 4;
					}

					data[0] = fr.data[0];
					data[1] = fr.data[1];
					data[2] = fr.data[2];
					data[3] = fr.data[3];
					data[4] = fr.data[4];
					data[5] = fr.data[5];
				}
			}

			if (ctrl_mode == 0) continue;

			std::cout << "Ctrl signal source: " << ctrl_mode << std::endl;
			std::cerr << "Ctrl data received: " << std::endl;

			for (int i = 0; i < 6; i ++)
			{
				std::stringstream ss;
				ss << std::hex << std::setw(2) << static_cast<int>(data[i]);
				std::cerr << ss.str() << " ";
			}
			std::cerr << std::endl;
			
			if (ctrl_mode == 1)
			{
				if (data[0] == 0x02)
				{
					// auto
					_pause_camera(false);
					//std::cout << "   ===> ctrl auto: " << data[0] << std::endl;
				} else if (data[0] == 0x01)
				{
					_pause_camera(true);
					//std::cout << "   ===> ctrl manual: " << data[0] << std::endl;
				}

				// truck loc
				if (data[1] == 0x01)
				{
					truck_behind  = false;
					//std::cout << "   ===> truck side: " << data[1] << std::endl;
				} else if (data[1] == 0x02)
				{
					truck_behind  = true;
					//std::cout << "   ===> truck behind: " << data[1] << std::endl;
				}

				// balanced fill
				if (data[2] == 0x01)
				{
					balanced_fill = false;
					//std::cout << "   ===> balanced fill ON: " << data[2] << std::endl;
				} else if (data[2] == 0x02)
				{
					balanced_fill = true;
					//std::cout << "   ===> balanced fill OFF: " << data[2] << std::endl;
				}

				// distance ctrl
				if (data[3] == 0x01)
				{
					distance_ctrl = true;
					//std::cout << "   ===> distance ctrl ON: " << data[3] << std::endl;
				} else if (data[3] == 0x02)
				{
					distance_ctrl = false;
					//std::cout << "   ===> distance ctrl OFF: " << data[3] << std::endl;
				}
			} else if (ctrl_mode == 4)
			{
				//
				//if (data[0] == 0x02)
				//{
					//uint8_t uint8_array[2];
					//uint8_array[0] = data[0];
					//uint8_array[1] = data[1];

					//uint16_t u16 = (uint8_array[0] << 8) |  uint8_array[1];

					//int16_t i = (int16_t) u16; //0xFFF8 is -8;
					//sprout_angle_Z = (int) i;

					//if (! sprout_angle_available) sprout_angle_available = true;

					//std::cout << "   ===> receiving sprout_angle_Z: " << sprout_angle_Z << std::endl;
				//}
			} else if (ctrl_mode == 3)
			{
				// manual interfere stopped
				if (data[0] == 0 && data[1] == 0 && data[2] == 0 && data[3] == 0 && data[4] == 0 && data[5] == 0)
				{
					truck_center_distance_estimated = 3.0; // should reset to the actual distance of truck?
					sprout_y_moves = 0;
					//std::cout << "   ===> Sprout distance counter has been reset." << std::endl;
				}
			}
			
		} catch(...)
		{
			std::cerr << "   ===> Ctrl Comm error." << std::endl;
		}

		//usleep(1);
	}
}

void ctrl_send()
{
	std::cout << "Ctrl signal sender starting ... " << std::endl;

	signal_last_sent_time = std::chrono::steady_clock::now();

	while (ros_initialized && comm_ctrl_enabled)
	{
		try {
			std::chrono::steady_clock::time_point cur_time = std::chrono::steady_clock::now();
			double interval = ((double) std::chrono::duration_cast<std::chrono::microseconds>(cur_time - signal_last_sent_time).count()) / 1000000.0;
			// also needs to check the age of the signal, if too old, stop
			if (interval < CTRL_SIGNAL_INTERVAL)
			{
				usleep(10000);
				continue;
			}

			uint8_t data[6];
			data[0] = 0x00;
			data[1] = 0x00;

			if (ctrl_signal_life_count > 0)
			{
				if (ctrl_current_ctrl_type == 0) data[0] = control_x;
				else if (ctrl_current_ctrl_type == 1) 
				{
					if (abs(sprout_y_moves) >= 5)
					{
						// stop
						//std::cout << "   ===> sprout_y_moves: " << sprout_y_moves << ", y move stopped: " << std::endl;
						ctrl_signal_life_count = 0;
						control_x = 0x00;
						control_y = 0x00;
					} else 
					{
						if (control_y == 0x01)
						{
							// stretch
							sprout_y_moves ++;
							truck_center_distance_estimated += 0.25;
						} else if (control_y == 0x02)
						{
							// contract
							sprout_y_moves --;
							truck_center_distance_estimated -= 0.25;
						}
						//std::cout << "   ===> sprout_y_moves: " << sprout_y_moves << ", y move ... " << std::endl;
						//std::cout << "   ===> truck_center_distance_estimated: " << truck_center_distance_estimated << std::endl;
						data[1] = control_y;
					}
				}
			} 

			// sending stop signal only once	
			//std::cout << "   ===> ctrl_signal_life_count: " << ctrl_signal_life_count << std::endl;
			if (ctrl_signal_life_count >= 0)
			{
				data[2] = 0;
				data[3] = 0;
				data[4] = 0;
				data[5] = 0;

				std::ostringstream ostr;
				for (int i = 0; i < 6; i ++)
				{
					ostr << std::hex << static_cast<int>(data[i]);
					ostr << " ";
				}
				
				if (comm_ctrl_type == 1)
				{
					uint8_t serial_frame [12]; 
					serial_frame[0]  = 0xFF;
					serial_frame[1]  = 0x00;
					serial_frame[2]  = 0x00;
					serial_frame[3]  = 0x02;

					serial_frame[4]  = 0x06;

					serial_frame[5]   = data[0];
					serial_frame[6]   = data[1];
					serial_frame[7]   = data[2];
					serial_frame[8]   = data[3];
					serial_frame[9]   = data[4];
					serial_frame[10]  = data[5];

					serial_frame[11] = 0xFF;

					//std::cerr << "Serial frame: " << std::endl;
					//for (int i = 0; i < 12; i ++)
					//{
						//std::stringstream ss;
						//ss << std::hex << std::setw(2) << static_cast<int>(serial_frame[i]);
						//std::cerr << ss.str() << " ";
					//}
					//std::cerr << std::endl;

					serial.sendData(&serial_frame[0], 12);
				}
				else if (comm_ctrl_type == 2)
				{
					scpp::CanFrame cf_to_write;
					cf_to_write.id = CAN_ID_CTRL; //0x98eeeeef;
					cf_to_write.len = 6;

					cf_to_write.data[0]  = data[0];
					cf_to_write.data[1]  = data[1];
					cf_to_write.data[2]  = data[2];
					cf_to_write.data[3]  = data[3];
					cf_to_write.data[4]  = data[4];
					cf_to_write.data[5]  = data[5];

					auto write_sc_status = socket_can.write(cf_to_write);
					if (write_sc_status == scpp::STATUS_OK)
					{
						//std::cout << "CAN: " << ostr.str() << ", life remains: " << ctrl_signal_life_count << std::endl;
						error_msg = "";
					}
					else
					{
						std::cout << "\nError: " << std::endl;
						printf("      => something went wrong on CAN socket write, error code : %d \n", int32_t(write_sc_status));
						error_msg = "CAN Signal Failed";
					}

				}
				if (ctrl_signal_life_count == 0)
				{
					ctrl_current_ctrl_type = -1;
					control_x = 0x00;
					control_y = 0x00;
					display_ctrl_available = false;
					//std::cout << "   ======> stop signal sent." << std::endl;
				}
				signal_last_sent_time = std::chrono::steady_clock::now();
			}

			ctrl_signal_life_count --;
			if (ctrl_signal_life_count < 0) 
			{
				ctrl_signal_life_count = -1; // sending stop signal only once
				ctrl_signal_life_assigned = -1;
			}
		} catch (...) {
			//
		}
	}
}
