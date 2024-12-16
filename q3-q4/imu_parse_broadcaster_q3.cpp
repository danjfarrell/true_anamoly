#include "imu.h"
#include <thread>
#include <iostream>
#include "threadsafequeue.h"

int main() {

	try {
        IMU imu("/dev/pts/4", 921600); // change depending on the port is setup
		
		//create the queue
        ThreadSafeQueue<IMU::Data> queue;
        std::atomic<bool> running(true);

		//create the threads
        std::thread readerThread(imuReader, std::ref(imu), std::ref(queue), std::ref(running));
        std::thread broadcasterThread(imuBroadcaster, std::ref(queue), 5000, std::ref(running));

		//listen for Enter to be pressed
        std::cout << "Press Enter to stop the program..." << std::endl;
        std::cin.get();

        running = false;
		
		//notify all the threads that running is false
		queue.notifyAll();

        if (readerThread.joinable()){
            readerThread.join();
        }

        if (broadcasterThread.joinable()){
            broadcasterThread.join();
        }

        std::cout << "Program terminated cleanly." << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
