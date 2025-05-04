/**
 *@file IcsHardSerialClass.cpp
 *@brief ICS3.5/3.6 arduino library use HardwareSerial
 *@author Vyankatesh Ashtekar
 *@date 2024/12/20
 *@version 2.0.0
 *@copyright © Vyankatesh Ashtekar
 **/

#include <iostream>
#include "IcsHardSerialClass.h"

/**
 *@brief constructor
 *@param[in] device UART device name
 *@param[in] enpin Pin number of transmit/receive switching pin
 *@param[in] baudrate Servo communication speed
 *@param[in] timeout Reception timeout (ms)
 **/
IcsHardSerialClass::IcsHardSerialClass(const char *device, unsigned char enpin, unsigned int baudrate, int timeout)
{
    int fd; ///< File descriptor

    // Wiring Pi setup. Use BCM numbering of pins
    wiringPiSetupGpio();

    if (enpin < 0)
        std::cerr << "invalid pin number, swithcing to enpin = " << enpin_default << std::endl;

    // Check for possible interference with serial pins
    bool flag_pin_intereference = false;
    for (int i = 0; i < 10; i++)
    {
        if (enpin == serialPinsList[i])
        {
            std::cerr << "The defined enable pin is already in use, swithcing to enpin = " << enpin_default << std::endl;
            flag_pin_intereference = true;
            break;
        }
    }

    // Copy the enable pin number to the default variable
    if (!flag_pin_intereference)
    {
        enpin_default = enpin;
    }

    // Enable pin set to listening mode by default
    pinMode(enpin_default, OUTPUT);
    digitalWrite(enpin_default, LOW);

    // Open the serial port
    if (device)
    {
        // Attempt to open the UART device. Using blocking mode because packet size is short?
        fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd < 0)
        {
            // Failure
            std::cerr << "Error: Unable to open serial device: " << device << std::endl;
        }
        else
        {
            // Success
            fd_default = fd;
            std::cout << "Serial port opened successfully" << std::endl;
            pinMode(enpin_default, OUTPUT);   // Configure enable pin
            digitalWrite(enpin_default, LOW); // Set default state
        }
    }
    else
    {
        std::cerr << "Error: Null device name provided!" << std::endl;
    }

    // Flush the I/O buffers
    if (ioctl(fd_default, TCFLSH, TCIOFLUSH) < 0)
    {
        std::cerr << "Failed to flush the serial port I/O buffers" << std::endl;
    }

    // Get the serial port attributes
    if (ioctl(fd_default, TCGETS2, &opt) < 0)
    {
        std::cerr << "Failed to get " << device << " attributes" << std::endl;
    }
    else
    {
        // Success. backup the attrbutes and restore while closing the port
        ioctl(fd_default, TCGETS2, &opt_backup);
    }

    // Set custom baud rate
    opt.c_cflag &= ~CBAUD;   // Clear standard baud rate bits
    opt.c_cflag |= BOTHER;   // Use custom baud rate
    opt.c_ispeed = baudrate; // Set input baud rate
    opt.c_ospeed = baudrate; // Set output baud rate
    baudrate_default = baudrate;
    std::cout << "Set the baudrate at: " << baudrate_default << std::endl;

    // Set 8-bit data frame and even parity
    opt.c_cflag &= ~CSIZE;  // Clear all the size bits
    opt.c_cflag |= CS8;     // 8 bits per byte
    opt.c_cflag |= PARENB;  // Enable parity
    opt.c_cflag &= ~PARODD; // Even parity
    opt.c_cflag &= ~CSTOPB; // 1 stop bit
	
    opt.c_cflag &= ~CRTSCTS;                // Disable hardware flow control
	opt.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)
    opt.c_iflag &= ~(IXON | IXOFF | IXANY); // Disable software flow control

    // Disable echo, signal interpretation, and special handling of received bytes
    opt.c_lflag &= ~ICANON;
    opt.c_lflag &= ~ECHO;                                                        // Disable echo
    opt.c_lflag &= ~ECHOE;                                                       // Disable erasure
    opt.c_lflag &= ~ECHONL;                                                      // Disable new-line echo
    opt.c_lflag &= ~ISIG;                                                        // Disable interpretation of INTR, QUIT and SUSP
    opt.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable special handling of received bytes

    // Prevent special interpretation of output bytes and conversion of newline
    opt.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes
    opt.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

    timeout_default = timeout;               // Used during reading manually
    // opt.c_cc[VTIME] = timeout_default / 100; // Convert timeout from milliseconds to tenths of a second
    // opt.c_cc[VMIN] = 1;

    opt.c_cc[VTIME] = 0;
	opt.c_cc[VMIN] = 0;    

    // Apply the new settings
    if (ioctl(fd_default, TCSETS2, &opt) < 0)
    {
        std::cerr << "Failed to set " << device << " attributes" << std::endl;
    }
    else
    {
        std::cout << "Serial the port attributes successfully" << std::endl;
    }
}

/**
 *@brief destructor
 *@post Releases UART file descriptor and cleans up resources
 **/
IcsHardSerialClass::~IcsHardSerialClass()
{
    if (fd_default >= 0)
    {
        ioctl(fd_default, TCSETS2, &opt_backup); // Reset the serial port settings
        close(fd_default);                       // Close UART file descriptor
    }
}

// function rewritten for raspberrry pi
bool IcsHardSerialClass::synchronize(unsigned char *txBuf, unsigned char txLen, unsigned char *rxBuf, unsigned char rxLen)
{
    // Enable transmission
    digitalWrite(enpin_default, HIGH);

    // Write the tx buffer, write: blocking call
    if (write(fd_default, txBuf, txLen) != txLen)
    {
        std::cerr << "Failed to write all " << txLen << "bytes to the serial port" << std::endl;
        return false;
    }
    // Keep enable HIGH till the transmission is complete
    if (baudrate_default == 115200)
		delayMicroseconds(180);
	else
		delayMicroseconds(20); // For 1250000

    // Disable transmission, start listening
    digitalWrite(enpin_default, LOW);

	// Return delay time
    if (baudrate_default == 115200)
		delayMicroseconds(100);
	else
		delayMicroseconds(50); // For 1250000

    // Start reading the data  3 reply bytes. We are expecting a total of rxLen bytes.
    int fion = 0;
    unsigned int readStartTime = micros();
    unsigned int currTime = readStartTime;
    
    // Read number of bytes received (should have 3 bytes back by now)
    ioctl(fd_default, FIONREAD, &fion);
    // If not a single byte has been received, wait a little more and check again
    while (fion < 1)
	{	
		// Wait a little more
		delayMicroseconds(50);
		
		// received something? get out
		ioctl(fd_default, FIONREAD, &fion);
		if (fion)
			break;
		
		// Nothing yet? Check your watch till TO
		currTime = micros();
		if ((currTime - readStartTime) > timeout_default)
        {
            std::cout << "Timeout" << std::endl;
            break;
        }
	}
    
    // Read byte by byte until there is no more to read
    unsigned char reChar;
    int bytesRead = 0;
    bool flag_incorrect_bytes_read = false;
    while (fion > 0)
    {
        read(fd_default, &reChar, sizeof(reChar));
        rxBuf[bytesRead] = reChar;
        bytesRead++;

        if (bytesRead > rxLen) // Prevent buffer overflow
        {
            std::cerr << "bytesRead more than expected rxLen." << std::endl;
            flag_incorrect_bytes_read = true;
        }

        // Check available bytes again and update fion
        ioctl(fd_default, FIONREAD, &fion);
    }

    if (bytesRead != rxLen)
    {
        std::cerr << "Failed to read expected number of bytes. Expected: " << rxLen << " Actually read: " << bytesRead << std::endl;
        flag_incorrect_bytes_read = true;
    }

    // Flush the read and write buffers
    ioctl(fd_default, TCFLSH, TCIOFLUSH);


    return true && (!flag_incorrect_bytes_read);
}
