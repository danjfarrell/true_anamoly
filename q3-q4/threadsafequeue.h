#ifndef THREADSAFEQUEUE_H
#define THREADSAFEQUEUE_H

#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>

template <typename T>
class ThreadSafeQueue {
public:
    bool push(const T& item, size_t max_size);
    bool pop(T& item, std::atomic<bool>& running);
    size_t size() const;

    //notify all waiting threads of closue
    void notifyAll();

private:
    std::queue<T> queue;
    mutable std::mutex mtx;
    std::condition_variable cv;
};


template <typename T>
bool ThreadSafeQueue<T>::push(const T& item, size_t max_size) {

        std::unique_lock<std::mutex> lock(mtx);
        if(queue.size() >= max_size){
			return false;
		}

		queue.push(item);

		cv.notify_one();
	return true;
}


template <typename T>
bool ThreadSafeQueue<T>::pop(T& item, std::atomic<bool>& running) {
    std::unique_lock<std::mutex> lock(mtx);

    // wait until the queue is not empty or the program is terminating
    cv.wait(lock, [&]() { return !queue.empty() || !running; });

    if (!running && queue.empty()) {
        cv.notify_all(); // notify all threads
        return false;    
    }

    item = queue.front();
    queue.pop();
    return true;
}

template <typename T>
size_t ThreadSafeQueue<T>::size() const {
    std::lock_guard<std::mutex> lock(mtx);
    return queue.size();
}

//used to be able to notify from anywhere
template <typename T>
void ThreadSafeQueue<T>::notifyAll() {
    std::lock_guard<std::mutex> lock(mtx);
    cv.notify_all(); // notify all waiting threads
}

#endif // THREADSAFEQUEUE_H
