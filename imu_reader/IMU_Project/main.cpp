#include <iostream>
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitions
#include <unistd.h>     // UNIX standard function definitions
#include <cstring>
#include <cstdlib>      // For atoi
#include <sys/socket.h> // UDP Socket
#include <arpa/inet.h>  // UDP definitions
#include <netinet/in.h>

// Include the Manufacturer Library
#include "EasyProfile/EasyObjectDictionary.h"
#include "EasyProfile/EasyProfile.h"

// --- CONSTANTS ---
const int BAUD_RATE = B115200;

// Initialize Library Objects
EasyObjectDictionary eOD;
EasyProfile eP(&eOD);

// Function to setup Linux Serial Port
int open_serial_port(const char* port) {
    int fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        perror("open_port: Unable to open serial port");
        return -1;
    }

    struct termios options;
    tcgetattr(fd, &options);

    // Set Baud Rate
    cfsetispeed(&options, BAUD_RATE);
    cfsetospeed(&options, BAUD_RATE);

    // Raw mode (no echo, no processing)
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB; // No Parity
    options.c_cflag &= ~CSTOPB; // 1 Stop Bit
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;     // 8 Data Bits
    
    // Disable hardware flow control
    options.c_cflag &= ~CRTSCTS;
    
    // Raw input/output
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;

    tcsetattr(fd, TCSANOW, &options);
    return fd;
}

int main(int argc, char *argv[]) {
    // --- 0. PARSE ARGUMENTS ---
    // Default Configuration
    const char* serial_port = "/dev/ttyACM0";
    const char* udp_ip = "127.0.0.1";
    int udp_port = 5000;

    // Override if arguments provided
    if (argc > 1) {
        serial_port = argv[1];
    }
    if (argc > 2) {
        udp_ip = argv[2];
    }
    if (argc > 3) {
        udp_port = atoi(argv[3]);
    }

    std::cout << "========================================" << std::endl;
    std::cout << " IMU UDP BRIDGE CONFIGURATION" << std::endl;
    std::cout << " Serial Port : " << serial_port << std::endl;
    std::cout << " Target IP   : " << udp_ip << std::endl;
    std::cout << " Target Port : " << udp_port << std::endl;
    std::cout << " Usage: ./program [PORT] [IP] [UDP_PORT]" << std::endl;
    std::cout << "========================================" << std::endl;

    // --- 1. SETUP UDP ---
    int sockfd;
    struct sockaddr_in servaddr;

    // Create socket file descriptor
    if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
        perror("socket creation failed");
        return 1;
    }

    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(udp_port);
    servaddr.sin_addr.s_addr = inet_addr(udp_ip);

    // --- 2. SETUP SERIAL ---
    std::cout << "[INFO] Opening Serial Port..." << std::endl;
    int serial_fd = open_serial_port(serial_port);
    if (serial_fd < 0) {
        std::cerr << "[ERROR] Failed to open " << serial_port << std::endl;
        return 1;
    }

    std::cout << "[INFO] Reading IMU Data & Broadcasting..." << std::endl;

    // Buffer for incoming serial data
    char rxBuffer[256];
    
    // Buffer for outgoing UDP data (4 floats = 16 bytes)
    float udpPacket[4]; 

    while (true) {
        // Read Raw Bytes from USB
        int bytes_read = read(serial_fd, rxBuffer, sizeof(rxBuffer));
        
        if (bytes_read > 0) {
            // Feed data into the library parser
            Ep_Header header;
            
            if (eP.On_RecvPkg(rxBuffer, bytes_read, &header) == EP_SUCC_) {
                
                bool data_ready = false;

                // Decode Packet
                switch (header.cmd) {
                    // CASE A: Combined Data
                    case EP_CMD_COMBO_: {
                        Ep_Combo data;
                        if (eOD.Read_Ep_Combo(&data) == EP_SUCC_) {
                            udpPacket[0] = data.q1 * 1.0e-7f; // W
                            udpPacket[1] = data.q2 * 1.0e-7f; // X
                            udpPacket[2] = data.q3 * 1.0e-7f; // Y
                            udpPacket[3] = data.q4 * 1.0e-7f; // Z
                            data_ready = true;
                        }
                        break;
                    }

                    // CASE B: Specific Quaternion Packet
                    case EP_CMD_Q_S1_E_: {
                        Ep_Q_s1_e data;
                        if (eOD.Read_Ep_Q_s1_e(&data) == EP_SUCC_) {
                            udpPacket[0] = data.q[0]; // W
                            udpPacket[1] = data.q[1]; // X
                            udpPacket[2] = data.q[2]; // Y
                            udpPacket[3] = data.q[3]; // Z
                            data_ready = true;
                        }
                        break;
                    }
                }

                // --- 3. SEND UDP ---
                if (data_ready) {
                    sendto(sockfd, (const char *)udpPacket, sizeof(udpPacket),
                           MSG_CONFIRM, (const struct sockaddr *) &servaddr,
                           sizeof(servaddr));
                }
            }
        }
        
        usleep(500); // 0.5ms delay to prevent CPU hogging
    }

    close(serial_fd);
    close(sockfd);
    return 0;
}