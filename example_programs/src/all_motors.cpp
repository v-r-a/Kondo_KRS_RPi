// g++ all_motors.cpp IcsHardSerialClass.cpp IcsBaseClass.cpp -o trial_krs -lwiringPi -Wall
// sudo chmod 666 /dev/ttyAMA0
// sudo chmod 666 /dev/ttyAMA1
// sudo chmod 666 /dev/ttyAMA2
// sudo chmod 666 /dev/ttyAMA3

#include <cstdio>
#include <iostream>
#include <wiringPi.h>
#include <IcsHardSerialClass.h>

// Number of motors on each port
int nm0 = 6;
int nm1 = 6;
int nm2 = 4;
int nm3 = 4;

// Motor IDs
int ID0[6] = {1, 2, 3, 4, 5, 6};    // LL
int ID1[6] = {7, 8, 9, 10, 11, 12}; // RL
int ID2[4] = {14, 15, 16, 17};      // LH
int ID3[4] = {13, 18, 19, 20};      // RH

// GPIO pin for Tx signal     14:AMA0, 00:AMA1, 04:AMA2, 08:AMA3, 12:AMA4
// GPIO pin for Rx signal     15:AMA0, 01:AMA1, 05:AMA2, 09:AMA4, 13:AMA4
// GPIO pin for enable signal 18:AMA0, 07:AMA1, 06:AMA2, 25:AMA3, 19:AMA4
// Enable pins BCM numbering
int En0 = 18;
int En1 = 07;
int En2 = 06;
int En3 = 25;
int En4 = 19;
// Serial ports
const char *device0 = "/dev/ttyAMA1";
const char *device1 = "/dev/ttyAMA2";
const char *device2 = "/dev/ttyAMA3";
const char *device3 = "/dev/ttyAMA4";

int main()
{
  // uses BCM numbering of the GPIOs and directly accesses the GPIO registers.
  if (wiringPiSetupGpio() == -1)
  {
    printf("Error initialising wiringPi GPIO\n");
    return 1;
  }
  printf("Bidirectional voltage shifter OE to HIGH\n");
  pinMode(26, OUTPUT);
  digitalWrite(26, HIGH);
  delay(100);

  // Baud rate
  unsigned int baudRate = 1250000;
  // Timeout in milliseconds
  int timeout = 10;

  // Create instances of IcsHardSerialClass
  IcsHardSerialClass krs0(device0, En1, baudRate, timeout);
  IcsHardSerialClass krs1(device1, En2, baudRate, timeout);
  IcsHardSerialClass krs2(device2, En3, baudRate, timeout);
  IcsHardSerialClass krs3(device3, En4, baudRate, timeout);

  int pos = 7500;

  int reply = 0;
  // Main loop to control the servo
  while (true)
  {
    // Set position to motors on Port 0
    for (int i = 0; i < nm0; i++)
    {
      reply = krs0.setPos(ID0[i], pos);
      printf("ID: %d, reply: %d\n", ID0[i], reply);
      delay(10);
    }

    // Set position to motors on Port 1
    for (int i = 0; i < nm1; i++)
    {
      reply = krs1.setPos(ID1[i], pos);
      printf("ID: %d, reply: %d\n", ID1[i], reply);
      delay(10);
    }
    // Set position to motors on Port 2
    for (int i = 0; i < nm2; i++)
    {
      reply = krs2.setPos(ID2[i], pos);
      printf("ID: %d, reply: %d\n", ID2[i], reply);
      delay(10);
    }

    // Set position to motors on Port 3
    for (int i = 0; i < nm3; i++)
    {
      reply = krs3.setPos(ID3[i], pos);
      printf("ID: %d, reply: %d\n", ID3[i], reply);
      delay(10);
    }

    printf("One cycle complete\n");
  }

  return 0;
}
