/**
*@file IcsBaseClass.h
*@brief ICS3.5/3.6 bace library header file
*@author Kondo Kagaku Co.,Ltd.
*@date 2017/12/27
*@version 2.1.0
*@copyright Kondo Kagaku Co., Ltd. 2020

*@mainpage Overview of IcsClass
*This library is for operating Kondo Scientific robot servos (KRS series). <br>
*Compatible with ICS3.5 and ICS3.6. <br>
*Currently designed to be used with Arduino products. <br>
*For usage and details, please see our website below. <br>
*<A HREF="http://kondo-robot.com/">http://kondo-robot.com/</A><br>
*If you have any problems, please contact us by referring to our website. <br>

*@par change history
*@par 2020/02/20 Ver 2.1.0
*-Change getStr in keywords.txt to getStrc
*-Change synchronize from protected to public //Because I wanted to use it externally
*-Implementation of ID command
*@par 2017/12/27 ver 2.0.0
*-Separate the communication part of ICSClass. Create IcsBaseClass.
*-"IcsHardSerialClass" which uses Serial
*<br> Derive "IcsSoftSerialClass" from IcsBaseClass using SoftSerial
*-Since SoftSerial could not use parity and the timing did not match, I created "KoCustomSoftSerial"
*@par 2016/12/27　ver 1.0.0
*-Initial distribution
**/

#ifndef __ics_Base_Servo_h__
#define __ics_Base_Servo_h__

// IcsBaseClass class ////////////////////////////////////////////////////
/**
 *@class IcsBaseClass
 *@brief Kondo Scientific's ICS 3.5/3.6 Base class for operating servo motors via microcontroller
 *@brief Create a base class to use Raspberry Pi UART
 **/
class IcsBaseClass
{
  // Fixed value (published)
public:
  // Servo ID range //////////////////////////////////////
  static constexpr int MAX_ID = 31; ///< Maximum value of servo ID
  static constexpr int MIN_ID = 0;  ///< Minimum value of servo ID

  // Servo maximum and minimum limit values

  static constexpr int MAX_POS = 11500; ///< Servo position maximum value

  static constexpr int MIN_POS = 3500; ///< Servo position minimum value
  static constexpr int ICS_FALSE = -1; ///< Value when ICS communication etc. fails

  // Fixed value (undisclosed)
protected:
  static constexpr float ANGLE_F_FALSE = 9999.9; ///< When calculating the angle, if it is not within the range, set it to 999.9 (if it is on the negative side, add a minus)
  static constexpr int ANGLE_I_FALSE = 0x7FFF;   ///< When calculating the angle, if it is not within the range, set it to 0x7FFF (if it is on the negative side, add a minus)

  // Each parameter setting range
  static constexpr int MAX_127 = 127; ///< Maximum value of parameter

  static constexpr int MAX_63 = 63; ///< Maximum value of parameter (current value)

  static constexpr int MIN_1 = 1;         ///< Minimum value of parameter
                                          // static const float MAX_DEG = 135.0;
  static constexpr float MAX_DEG = 180.0; ///< Maximum value of angle

  // static const float MIN_DEG = -135.0;
  static constexpr float MIN_DEG = -180.0; ///< Maximum value of angle

  // static const int MAX_100DEG = 13500;
  static constexpr int MAX_100DEG = 18000; ///< Maximum value of angle (x100)

  // static const int MIN_100DEG = -13500;
  static constexpr int MIN_100DEG = -18000; ///< Minimum value of angle (x100)

  // type definition in class
public:
  // constructor, destructor
  virtual ~IcsBaseClass() {}

public:
  // constructor

  // variable
public:
protected:
  // function

  // data transmission/reception
public:
  // Data transmission/reception //////////////////////////////////////////////////////////////////////////////////////////////
  /**
   *@brief Sending and receiving ICS communication
   *@param[in,out] *txBuf
   *@param[in] txLen
   *@param[out] *rxBuf Receive storage buffer
   *@param[in] rxLen Number of received data
   *@retval true Communication successful
   *@retval false Communication failure
   *@attention Please note that the number of sent data and received data varies depending on the command.
   *@attention This function must be written externally.
   **/
  virtual bool synchronize(unsigned char *txBuf, unsigned char txLen, unsigned char *rxBuf, unsigned char rxLen) = 0;

  // servo related
public:
  // Servo positioning settings
  int setPos(unsigned char id, unsigned int pos); // Target value setting
  int setFree(unsigned char id);                  // Servo weakness + reading current value

  // Write various parameters
  int setStrc(unsigned char id, unsigned int strc);  // Stretch writing 1 to 127 1 (weak) <=> 127 (strong)
  int setSpd(unsigned char id, unsigned int spd);    // Speed ​​writing 1 to 127 1(slow) <=> 127(fast)
  int setCur(unsigned char id, unsigned int curlim); // Write current limit value 1 to 63 1 (low) <=> 63 (high)
  int setTmp(unsigned char id, unsigned int tmplim); // Temperature upper limit write 1 to 127 127 (low temperature) <=> 1 (high temperature)

  // Read various parameters
  int getStrc(unsigned char id); // Stretch reading 1 to 127 1 (weak) <=> 127 (strong)
  int getSpd(unsigned char id);  // Speed ​​reading 1 to 127 1(slow)<=> 127(fast)
  int getCur(unsigned char id);  // Read current value 63←0 | 64→127
  int getTmp(unsigned char id);  // Read current temperature 127 (low temperature) <=> 0 (high temperature)
  int getPos(unsigned char id);  // Read current position *Valid for ICS3.6 or later

  int getID();
  int setID(unsigned char id);

protected:
  // Servo ID limit
  unsigned char idMax(unsigned char id);

  ////Servo movable range parameter range limit setting
  bool maxMin(int maxPos, int minPos, int val);

  // Angle related
public:
  // Angle conversion Convert from POS to angle
  static int degPos(float deg);
  // Angle conversion Convert from angle to POS
  static float posDeg(int pos);

  // Angle conversion x100 Convert from POS to angle
  static int degPos100(int deg);
  // Angle conversion x100 Convert from angle to POS
  static int posDeg100(int pos);
};
#endif