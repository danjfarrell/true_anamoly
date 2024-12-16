#include "imu.h"
#include <iostream>
#include <cstring>

IMU::IMU(const std::string& device, int baud_rate) 
    : uart_device(device), fd(-1), baud_rate(baud_rate) {
    setupUART();
}

IMU::~IMU() {
	
	//clear the buffers
    if (tcflush(fd, TCIOFLUSH) != 0) {
        std::cerr << "Failed to clear UART buffers: " << strerror(errno) << std::endl;
    } else {
        std::cout << "UART buffers cleared successfully." << std::endl;
    }
	
    if (fd >= 0) {
        close(fd);
    }
}

float IMU::ntohf(uint32_t net_float) {
    //swap floats using ntohl for little-endian systems
    uint32_t host_float = ntohl(net_float);
    return *reinterpret_cast<float*>(&host_float);
}


void IMU::setupUART() {
    //open the UART
    fd = open(uart_device.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        throw std::runtime_error("Failed to open UART device: " + uart_device);
    }

    //config parameters
    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        close(fd);
        throw std::runtime_error("Failed to get UART attributes.");
    }

    //set the baud rate
    speed_t speed;
    switch (baud_rate) {
        case 921600: speed = B921600; break;
		case 460800: speed = B460800; break;
		case 230400: speed = B230400; break;
		case 115200: speed = B115200; break;
		case 57600:  speed = B57600; break;
		case 38400:  speed = B38400; break;
		case 19200:  speed = B19200; break;
        case 9600:   speed = B9600; break;
		case 4800:   speed = B4800; break;
		case 2400:   speed = B2400; break;
		case 1200:   speed = B1200; break;
        default: throw std::invalid_argument("Unsupported baud rate.");
    }
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    //configure UART:
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; //8-bit chars
    tty.c_cflag &= ~PARENB;                    //parity
    tty.c_cflag &= ~CSTOPB;                    //stop bit
    tty.c_cflag &= ~CRTSCTS;                   //hardware flow control
    tty.c_cflag |= (CLOCAL | CREAD);           //enable receiver & set local mode

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); //input
    tty.c_oflag &= ~OPOST;                         //output
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);        // Disable sfc

	//throw erros if UART failed 
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        close(fd);
        throw std::runtime_error("Failed to set UART attributes.");
    }
		//clear buffers to be sure to have an empty port on start
    tcflush(fd, TCIOFLUSH);
}

bool IMU::parseIMUData(const std::vector<uint8_t>& buffer, Data& imu_data) {
    const uint8_t header[] = {0x7F, 0xF0, 0x1C, 0xAF};

    // Validate header
    if (buffer.size() < 20 || !std::equal(std::begin(header), std::end(header), buffer.begin())) {
        return false;
    }

    //extract packet count and convert network byte order to host byte order
    uint32_t packet_count_net;
    std::memcpy(&packet_count_net, &buffer[4], sizeof(packet_count_net));
    imu_data.packet_count = ntohl(packet_count_net);

uint32_t x_rate_net, y_rate_net, z_rate_net;
// extract rate data and convert byte order for floats
std::memcpy(&x_rate_net, &buffer[8], sizeof(x_rate_net));
std::memcpy(&y_rate_net, &buffer[12], sizeof(y_rate_net));
std::memcpy(&z_rate_net, &buffer[16], sizeof(z_rate_net));

imu_data.x_rate_rdps = ntohf(x_rate_net);
imu_data.y_rate_rdps = ntohf(y_rate_net);
imu_data.z_rate_rdps = ntohf(z_rate_net);

    return true;
}

IMU::Data IMU::read() {
    std::vector<uint8_t> buffer(20);

    // read the uart data
    ssize_t bytes_read = ::read(fd, buffer.data(), buffer.size());
    if (bytes_read != static_cast<ssize_t>(buffer.size())) {
        throw std::runtime_error("Failed to read a complete IMU packet.");
    }

    Data imu_data;
	//parse the data
    if (!parseIMUData(buffer, imu_data)) {
        throw std::runtime_error("Failed to parse IMU data.");
    }

    return imu_data;
}
