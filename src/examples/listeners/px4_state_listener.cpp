/****************************************************************************
 *
 * Copyright 2017 Proyectos y Sistemas de Mantenimiento SL (eProsima).
 *           2018 PX4 Pro Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief PX4 State uORB topic listener example
 * @file px4_state_listener.cpp
 * @addtogroup examples
 * @author Inha Baek <devbbaegi@gmail.com>
 */

#include <iostream>
#include <memory>
#include <unistd.h>
#include <time.h>

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_status.hpp> // for Mode/Arming status
#include <px4_msgs/msg/vehicle_local_position.hpp> // for local position
#include <px4_msgs/msg/battery_status.hpp> // for Battery status

#include <CppLinuxSerial/SerialPort.hpp>
#include <thread>
#include <chrono>

using namespace mn::CppLinuxSerial;
using namespace std;
using std::placeholders::_1;

std::string sendmsg = "";

bool tEnd = false;
std::string readData = "";
bool read_parse = false;


float position_x = 0.00;
float position_y = 0.00;
float position_z = 0.00;
float yaw = 0.00;
std::string px4_mode = "NONE";
int bat_remain = 0;

#define PI 3.14159265

float rad2deg(float radian)
{
    return radian*180/PI;
}
float deg2rad(float degree)
{
    return degree*PI/180;
}

/**
 * @brief PX4 State uORB topic data callback
 */
class PX4StateListener : public rclcpp::Node
{
public:
    PX4StateListener() : Node("px4_state_listener")
    {
        position_subscription_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
                    "fmu/vehicle_local_position/out", 10, std::bind(&PX4StateListener::position_callback, this, _1));

        status_subscription_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
                    "fmu/vehicle_status/out", 10, std::bind(&PX4StateListener::status_callback, this, _1));

        battery_subscription_ = this->create_subscription<px4_msgs::msg::BatteryStatus>(
                    "fmu/battery_status/out", 10, std::bind(&PX4StateListener::battery_callback, this, _1));


    }

private:
    void position_callback(const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg)
    {
        /*
        std::cout << "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n";
        */
        //std::cout << "RECEIVED PX4 Position DATA"   << std::endl;
        /*
        std::cout << "============================="   << std::endl;
        std::cout << "ts: "          << msg->timestamp    << std::endl;

        std::cout << "Local Position x: " << msg->x  << std::endl;

        std::cout << "Local Position y: " << msg->y  << std::endl;
        std::cout << "Local Postiion z: " << msg->z  << std::endl;
        std::cout << "Yaw: " << msg->heading  << std::endl;
        std::cout << "Velocity x: " << msg->vx << std::endl;
        std::cout << "Velocity y: " << msg->vy << std::endl;
        std::cout << "Velocity z: " << msg->vz << std::endl;
        */
        position_x = msg->x;
        position_y = msg->y;
        position_z = msg->z;
        yaw = rad2deg(msg->heading);
        //sendmsg = "x : " + std::to_string(msg->x);// + ", y : " + std::to_string(msg->y) + ", z : " + std::to_string(msg->z)+"\n";
    }


    void status_callback(const px4_msgs::msg::VehicleStatus::UniquePtr msg)
    {
        //std::cout << "RECEIVED PX4 STATE DATA"   << std::endl;
        /*
        std::cout << "============================="   << std::endl;
        std::cout << "ts: "          << msg->timestamp    << std::endl;
        std::cout << "Mode: " << msg->nav_state  << std::endl;
        std::cout << "Arming: " << msg->arming_state  << std::endl;
        */

        /*
        # Navigation state, i.e. "what should vehicle do".
        uint8 NAVIGATION_STATE_MANUAL = 0		# Manual mode
        uint8 NAVIGATION_STATE_ALTCTL = 1		# Altitude control mode
        uint8 NAVIGATION_STATE_POSCTL = 2		# Position control mode
        uint8 NAVIGATION_STATE_AUTO_MISSION = 3		# Auto mission mode
        uint8 NAVIGATION_STATE_AUTO_LOITER = 4		# Auto loiter mode
        uint8 NAVIGATION_STATE_AUTO_RTL = 5		# Auto return to launch mode
        uint8 NAVIGATION_STATE_AUTO_LANDENGFAIL = 8 	# Auto land on engine failure
        uint8 NAVIGATION_STATE_AUTO_LANDGPSFAIL = 9	# Auto land on gps failure (e.g. open loop loiter down)
        uint8 NAVIGATION_STATE_ACRO = 10		# Acro mode
        uint8 NAVIGATION_STATE_UNUSED = 11		# Free slot
        uint8 NAVIGATION_STATE_DESCEND = 12		# Descend mode (no position control)
        uint8 NAVIGATION_STATE_TERMINATION = 13		# Termination mode
        uint8 NAVIGATION_STATE_OFFBOARD = 14
        uint8 NAVIGATION_STATE_STAB = 15		# Stabilized mode
        uint8 NAVIGATION_STATE_UNUSED2 = 16		# Free slot
        uint8 NAVIGATION_STATE_AUTO_TAKEOFF = 17	# Takeoff
        uint8 NAVIGATION_STATE_AUTO_LAND = 18		# Land
        uint8 NAVIGATION_STATE_AUTO_FOLLOW_TARGET = 19	# Auto Follow
        uint8 NAVIGATION_STATE_AUTO_PRECLAND = 20	# Precision land with landing target
        uint8 NAVIGATION_STATE_ORBIT = 21       # Orbit in a circle
        uint8 NAVIGATION_STATE_MAX = 22
        */

        if(msg->nav_state == 0){px4_mode = "MANUAL";}
        else if(msg->nav_state == 1){px4_mode = "ATLCTL";}
        else if(msg->nav_state == 2){px4_mode = "POSCTL";}
        else if(msg->nav_state == 3){px4_mode = "AUTO_MISSION";}
        else if(msg->nav_state == 4){px4_mode = "AUTO_LOITER";}
        else if(msg->nav_state == 5){px4_mode = "AUTO_RTL";}
        else if(msg->nav_state == 6){px4_mode = "AUTOLANDENGFAIL";}
        else if(msg->nav_state == 7){px4_mode = "AUTOLANDGPSFAIL";}
        else if(msg->nav_state == 8){px4_mode = "ACRO";}
        else if(msg->nav_state == 9){px4_mode = "UNUSED";}
        else if(msg->nav_state == 14){px4_mode = "OFFBOARD";}
        else if(msg->nav_state == 18){px4_mode = "AUTO_LAND";}
        //sendmsg = "Mode : " + std::to_string(msg->nav_state);
    }

    void battery_callback(const px4_msgs::msg::BatteryStatus::UniquePtr msg)
    {
        //std::cout << "RECEIVED PX4 Battery DATA"   << std::endl;
        /*
        std::cout << "============================="   << std::endl;
        std::cout << "ts: "          << msg->timestamp    << std::endl;
        std::cout << "Battery Remaining: " << msg->remaining  << std::endl;
        */
        bat_remain = (int)(100*msg->remaining);
        //sendmsg = "Battery remaining : " + std::to_string(msg->remaining);
    }


    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr position_subscription_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_subscription_;
    rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr battery_subscription_;
};

void callback_out(){
    std::cout << position_x << std::endl;
}


void Serial_TX(){

    while(!tEnd){
        // Create serial port object and open serial port
        SerialPort serialPort("/dev/ttyUSB0", BaudRate::B_57600);
        // Use SerialPort serialPort("/dev/ttyACM0", 13000); instead if you want to provide a custom baud rate
        serialPort.SetTimeout(0); // Block when reading until any data is received

        serialPort.Open();

        sendmsg = "pNED " + std::to_string(position_x) +
                  " " + std::to_string(position_y) +
                  " " + std::to_string(position_z) +
                "\nYaw " + std::to_string(yaw) +
                " / Bat "  + std::to_string(bat_remain) +
                px4_mode + "\n";

        serialPort.Write(sendmsg);


        //delay
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        // Close the serial portendl;
        //serialPort.Close();
    }
}

void Serial_RX(){

    while(!tEnd){
        // Create serial port object and open serial port
        SerialPort serialPort("/dev/ttyUSB0", BaudRate::B_57600);
        // Use SerialPort serialPort("/dev/ttyACM0", 13000); instead if you want to provide a custom baud rate
        serialPort.SetTimeout(100); // Block when reading until any data is received (buffer size = 100)

        serialPort.Open();

        std::string readData;
        serialPort.Read(readData);

        if(!readData.empty()){
            std::cout << readData << std::endl;
        }

        // Close the serial portendl;
        //serialPort.Close();
    }
}



int main(int argc, char *argv[])
{

    //Set ROS2 node
    std::cout << "Starting px4_state_listener node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    //rclcpp::WallRate loop_rate(1);

    //Serial TX/RX Thread
    std::thread t1(Serial_TX);
    std::thread t2(Serial_RX);

    rclcpp::spin(std::make_shared<PX4StateListener>());

    //t1.join();
    //t2.join();

    rclcpp::shutdown();
    tEnd = true;
    return 0;
}
