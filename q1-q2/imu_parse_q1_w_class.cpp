#include "imu.h"
#include <iostream>

int main() {
    try {
        //initialize the imu 
        IMU imu1("/dev/pts/4", 921600);

        while (true) {
            //read and print data from IMU1
            IMU::Data data1 = imu1.read();
            std::cout << "IMU1 - Packet: " << data1.packet_count
                      << ", X: " << data1.x_rate_rdps
                      << ", Y: " << data1.y_rate_rdps
                      << ", Z: " << data1.z_rate_rdps << std::endl;

        }
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
