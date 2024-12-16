import serial
import struct
import time
import random
import socket
import threading
import signal
import sys

# flags for control threads
terminate = False
uart_port = None
baud_rate = None
sock = None

def signal_handler(sig, frame):
    #set the terminate flag and stop threads cleaning 

    global terminate, uart_port,baud_rate,sock
    terminate = True
    print("\nTerminating program...")

    #rest the uart so that it can be cleared
    ser = serial.Serial(uart_port, baud_rate, timeout=1)
    if ser:
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        ser.close()
        print("UART buffers cleared and UART closed.")

    
    #check the socket to see if it is active and empty it
    if sock:
        sock.setblocking(False)  # Set the socket to non-blocking mode
        try:
            while True:
                data, addr = sock.recvfrom(65535)  # Read a packet
                print(f"Discarded packet from {addr}: {data}")
        except BlockingIOError:
            # No more data to read
            pass
        finally:
            sock.setblocking(True)  # Restore to blocking mode



        sock.close()
        print("UDP buffer cleared and socket closed.")

    print("Shutdown complete.")
    sys.exit(0)


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

    return packet, (packet_count, x_rate, y_rate, z_rate)

def send_imu_data(uart_port, baud_rate, original_data_store):

    global terminate

    
    try:
        with serial.Serial(uart_port, baud_rate, timeout=1) as ser:
            packet_count = 0
            while not terminate:
                imu_packet, (packet_count, x_rate, y_rate, z_rate) = generate_imu_packet(packet_count)
                original_data_store[packet_count] = (x_rate, y_rate, z_rate)
                ser.write(imu_packet)
                packet_count += 1
                time.sleep(0.08)  # sleep for 80ms

    except serial.SerialException as e:
        print(f"Error opening or using serial port: {e}")

def receive_udp_data(port, original_data_store):

    global terminate, sock
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("127.0.0.1", port))
        print(f"Listening for UDP data on port {port}...")

        while not terminate:
            data, _ = sock.recvfrom(1024)  
            print(f"Received UDP: {data.decode()}")
            udp_packet = data.decode()

            # extract packet count and rates from the UDP message
            try:
                parts = udp_packet.split(", ")
                packet_count = int(parts[0].split(": ")[1])
                x_rate = float(parts[1].split(": ")[1])
                y_rate = float(parts[2].split(": ")[1])
                z_rate = float(parts[3].split(": ")[1])

                # compare the data received from the udp to the data stored
                # stored in the original data dictionary
                if packet_count in original_data_store:
                    original = original_data_store[packet_count]
                    print(f"Verification for Packet {packet_count}:")
                    print(f"  Original: X={original[0]:.2f}, Y={original[1]:.2f}, Z={original[2]:.2f}")
                    print(f"  Received: X={x_rate:.2f}, Y={y_rate:.2f}, Z={z_rate:.2f}")
                else:
                    print(f"Packet {packet_count} not found in original data store.")
            except (IndexError, ValueError) as e:
                print(f"Error parsing UDP data: {e}")
                

    except Exception as e:
        print(f"Error in UDP listener: {e}")

def main():
    
    global uart_port,baud_rate,port
    
    uart_port = '/dev/pts/3'  # change depending on the UART port set
    baud_rate = 921600
    udp_port = 5000

    signal.signal(signal.SIGINT, signal_handler)
    time.sleep(2)
    original_data_store = {}  # store sent IMU data keyed by packet count

    # start the IMU data sender in a separate thread
    sender_thread = threading.Thread(target=send_imu_data, args=(uart_port, baud_rate, original_data_store))
    sender_thread.daemon = True
    sender_thread.start()

    # start the UDP listener and verifier in a separate thread
    udp_thread = threading.Thread(target=receive_udp_data, args=(udp_port, original_data_store))
    udp_thread.daemon = True
    udp_thread.start()

    # file original_data_store 
    try:
        with serial.Serial(uart_port, baud_rate, timeout=1) as ser:
            packet_count = 0
            while True:
                #imu_packet, (packet_count, x_rate, y_rate, z_rate) = generate_imu_packet(packet_count)
                #original_data_store[packet_count] = (x_rate, y_rate, z_rate)
                #ser.write(imu_packet)
                #packet_count += 1
                time.sleep(0.08)
    except KeyboardInterrupt:

        signal_handler(None, None)
        

if __name__ == "__main__":
    main()
