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
 * @author Inha Baek <ihb@realtimevisual.com>
 */

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <CppLinuxSerial/SerialPort.hpp>
#include <thread>
#include <chrono>

using namespace mn::CppLinuxSerial;
using namespace std;

/**
 * @brief PX4 State uORB topic data callback
 */
class PX4StateListener : public rclcpp::Node
{
public:
        explicit PX4StateListener() : Node("px4_state_listener") {
                subscription_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
                        "fmu/vehicle_local_position/out",
#ifdef ROS_DEFAULT_API
            10,
#endif
                        [this](const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg) {
			std::cout << "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n";
                        std::cout << "RECEIVED PX4 STATE DATA"   << std::endl;
			std::cout << "============================="   << std::endl;
			std::cout << "ts: "          << msg->timestamp    << std::endl;
                        std::cout << "Local Position x: " << msg->x  << std::endl;
                        std::cout << "Local Position y: " << msg->y  << std::endl;
                        std::cout << "Local Postiion z: " << msg->z  << std::endl;
                        std::cout << "Yaw: " << msg->heading  << std::endl;
                        std::cout << "Velocity x: " << msg->vx << std::endl;
                        std::cout << "Velocity y: " << msg->vy << std::endl;
                        std::cout << "Velocity z: " << msg->vz << std::endl;

		});
	}

private:
        rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr subscription_;
};

void Serial_TX(){
    while(true){
        // Create serial port object and open serial port
        SerialPort serialPort("/dev/ttyUSB0", BaudRate::B_57600);
        // Use SerialPort serialPort("/dev/ttyACM0", 13000); instead if you want to provide a custom baud rate
        serialPort.SetTimeout(0); // Block when reading until any data is received
        serialPort.Open();

        // Write some ASCII datae
        serialPort.Write("Hello");

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        // Close the serial portendl;
        //serialPort.Close();
    }
}

void Serial_RX(){
    while(true){
        // Create serial port object and open serial port
        SerialPort serialPort("/dev/ttyUSB0", BaudRate::B_57600);
        // Use SerialPort serialPort("/dev/ttyACM0", 13000); instead if you want to provide a custom baud rate
        serialPort.SetTimeout(-1); // Block when reading until any data is received
        serialPort.Open();

        // Read some data back (will block until at least 1 byte is received due to the SetTimeout(-1) call above)
        std::string readData;
        serialPort.Read(readData);

        if (!readData.empty()){
            std::cout << readData << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // Close the serial portendl;
        //serialPort.Close();
    }
}


int main(int argc, char *argv[])
{


    std::cout << "Starting px4_state_listener node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PX4StateListener>());
    std::thread t1(Serial_TX);
    std::thread t2(Serial_RX);

    t1.join();
    t2.join();


    rclcpp::shutdown();
    return 0;
}
