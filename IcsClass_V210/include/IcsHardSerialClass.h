
/**
*  @file IcsHardSerialClass.h
* @brief ICS3.5/3.6 Raspberry Pi library
* @author Vyankatesh Ashtekar
* @date 2024/12/20
* @version 2.0.0
* @copyright Vyankatesh Ashtekar 2024

**/

#ifndef _ics_HardSerial_Servo_h_
#define _ics_HardSerial_Servo_h_

#include "IcsBaseClass.h"
#include <asm/termbits.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <wiringPi.h>

// IcsHardSerialClass class ///////////////////////////////////////////////////
/**
 * @class IcsHardSerialClass
 * @brief A class that allows access to Kondo Kagaku's KRS servos from Raspberry Pi's UART
 * @brief Derived from IcsBaseClass
 **/
class IcsHardSerialClass : public IcsBaseClass
{
  // Type definitions within the class
public:
  // Constructor, Destructor
public:
  // Constructor
  IcsHardSerialClass(const char *device, unsigned char enpin, unsigned int baudrate, int timeout);

  // Descructor
  ~IcsHardSerialClass();

  // Variables
public:
protected:
  int fd_default = -1;                                         ///< File descriptor
  int enpin_default = 18;                                      ///< Variable to store the pin number of the enable pin (for switching between send and receive)
  unsigned int baudrate_default = 115200;                      ///< Variable to store the communication speed of ICS
  unsigned int timeout_default = 100;                          ///< Variable to store the communication timeout (ms)
  struct termios2 opt;                                         ///< Serial port settings
  struct termios2 opt_backup;                                  ///< Backup of current serial port settings
  int serialPinsList[10] = {14, 15, 0, 1, 4, 5, 8, 9, 12, 13}; // UART0-4 Tx Rx pins BCM numbering
  int enPinsList[5] = {18, 7, 6, 25, 19};                      // Enable pins based on Venky's PCB. BCM numbering.

  // Functions

  // Data Transmission and Reception
public:
  virtual bool synchronize(unsigned char *txBuf, unsigned char txLen, unsigned char *rxBuf, unsigned char rxLen);

  // Servo Related // All together
public:
};

#endif
