# Compiler and flags
CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -pthread

# Target executable name
TARGET = imu_broadcaster

# Source files
SRCS = imu.cpp imu_parse_broadcaster_q3.cpp

# Object files (derived from SRCS)
OBJS = $(SRCS:.cpp=.o)

# Header files
HEADERS = imu.h threadsafequeue.h

# Default target
all: $(TARGET)

# Rule to build the target
$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^

# Rule to compile .cpp files to .o files
%.o: %.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean up build files
clean:
	rm -f $(OBJS) $(TARGET)

# Phony targets
.PHONY: all clean
