import serial
import struct
import time
import random

def generate_imu_packet(packet_count):

    header = b'\x7F\xF0\x1C\xAF'
    #make random data for each rate
    x_rate = random.uniform(-500.0, 500.0)  
    y_rate = random.uniform(-500.0, 500.0) 
    z_rate = random.uniform(-500.0, 500.0)  

    # pack the data into a byte array using network byte order
    packet = header
    packet += struct.pack('>I', packet_count)  
    packet += struct.pack('>f', x_rate)       
    packet += struct.pack('>f', y_rate)       
    packet += struct.pack('>f', z_rate)      

    return packet , x_rate, y_rate, z_rate

def main():
    uart_port = '/dev/pts/5'  # change depending on the UART port set
    baud_rate = 921600

    try:
        #open the UART port
        with serial.Serial(uart_port, baud_rate, timeout=1) as ser:
            packet_count = 0
            while True:
                #generate a packet 
                imu_packet, x, y, z = generate_imu_packet(packet_count)
                #send a new IMU packet
                ser.write(imu_packet)
                print(f"Sent packet {packet_count},X:{x:.3f},Y:{y:.3f},Z:{z:.3f} ")

                packet_count += 1
                time.sleep(0.08) 

    except serial.SerialException as e:
        print(f"Error opening or using serial port: {e}")

if __name__ == "__main__":
    main()
