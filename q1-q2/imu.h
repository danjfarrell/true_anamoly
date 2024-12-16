#ifndef IMU_H
#define IMU_H

#include <string>
#include <vector>
#include <cstdint>
#include <stdexcept>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <arpa/inet.h>

class IMU {
public:
    struct Data {
        uint32_t packet_count;
        float x_rate_rdps;
        float y_rate_rdps;
        float z_rate_rdps;
    };

    IMU(const std::string& device, int baud_rate);
    ~IMU();

    Data read();

private:
    std::string uart_device;
    int fd;          //uart file descriptor
    int baud_rate;   

    bool parseIMUData(const std::vector<uint8_t>& buffer, Data& imu_data);
    void setupUART();
    //convert netwokr byte order to host for floats
	float ntohf(uint32_t net_float);
};

#endif // IMU_H

