#ifndef IMU_H
#define IMU_H

#include <string>
#include <vector>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <cstdint>
#include <stdexcept>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <arpa/inet.h>
#include "threadsafequeue.h"

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
    int fd; //uart file descriptor
    int baud_rate;

    bool parseIMUData(const std::vector<uint8_t>& buffer, Data& imu_data);
    void setupUART();
    //convert netwokr byte order to host for floats
	float ntohf(uint32_t net_float);
};


void imuReader(IMU& imu, ThreadSafeQueue<IMU::Data>& queue, std::atomic<bool>& running);
void imuBroadcaster(ThreadSafeQueue<IMU::Data>& queue, int udp_port, std::atomic<bool>& running);


#endif // IMU_SINGLE_H
