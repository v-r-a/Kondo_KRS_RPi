/**
 *	@file IcsBaseClass.cpp
 *	@brief ICS3.5/3.6 base library
 *	@author Kondo Kagaku Co.,Ltd.
 *	@date	2020/02/20
 *	@version 2.1.0
 *	@copyright &copy; Kondo Kagaku Co.,Ltd. 2017
 **/
#include <cstdio>
#include <wiringPi.h>
#include "IcsBaseClass.h"

// Servo ID range /////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Check if the servo ID is within range
 * @param[in] id
 * @return the ID sent
 * @retval 0xFF out of range
 **/
unsigned char IcsBaseClass::idMax(unsigned char id)
{
  if ((char)id < MIN_ID)
  {
    id = 0xFF;
  }
  if (id > MAX_ID)
  {
    id = 0xFF;
  }
  return id;
}

// Servo movable range parameter range limit setting ///////////////////////////////////////////////////////////////
/**
 * @brief Keep within the maximum and minimum value range of position data
 * @param[in] maxPos Maximum value
 * @param[in] minPos Minimum value
 * @param[in,out] val current value
 * @return Position data with limit
 **/
bool IcsBaseClass::maxMin(int maxPos, int minPos, int val)
{
  if (val > maxPos)
  {
    return false;
  }
  if (val < minPos)
  {
    return false;
  }
  return true;
}

// Angle conversion　From angle to POS/////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Convert angle data (float type) to position data
 * @param[in] deg Angle (deg) (float type)
 * @return position data
 * @retval -1 out of range
 **/
int IcsBaseClass::degPos(float deg)
{
  if (deg > MAX_DEG)
  {
    return -1;
  }
  if (deg < MIN_DEG)
  {
    return -1;
  }
  int pos = deg * 29.633;
  pos = pos + 7500;
  return pos;
}

// Angle conversion POS to angle /////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Convert position data to angle data (float type)
 * @param[in] pos position data
 * @return Angle (deg) (float type)
 * @retval #ANGLE_F_FALSE Out of range in positive direction
 * @retval -#ANGLE_F_FALSE (0x8000) Out of range in negative direction
 **/
float IcsBaseClass::posDeg(int pos)
{
  pos = pos - 7500;
  float deg = pos / 29.633;

  if (deg > MAX_DEG)
  {
    return ANGLE_F_FALSE;
  }
  if (deg < MIN_DEG)
  {
    return -ANGLE_F_FALSE;
  }

  return deg;
}

// Angle conversion x100 Angle to POS///////////////////////////////////////////////////////////////////////////////
/**
 * @brief Convert angle data x100 (int type) to position data
 * @param[in] deg Angle (deg x100) (int type)
 * @return Converted position data
 * @retval -1 out of range
 **/
int IcsBaseClass::degPos100(int deg)
{
  if (deg > MAX_100DEG)
  {
    return -1;
  }
  if (deg < MIN_100DEG)
  {
    return -1;
  }
  long a = ((long)deg * 2963) / 10000;
  int pos = a + 7500;
  return pos;
}

// Angle conversion x100 POS to angle///////////////////////////////////////////////////////////////////////////////
/**
 * @brief Convert position data to angle data (int type)
 * @param[in] pos position data
 * @return Angle (deg x100) (int type)
 * @retval #ANGLE_I_FALSE Out of range in positive direction
 * @retval -#ANGLE_I_FALSE Out of range in negative direction
 **/
int IcsBaseClass::posDeg100(int pos)
{
  long a = pos - 7500;
  int deg = (a * 1000) / 296;

  if (deg > MAX_100DEG)
  {
    return ANGLE_I_FALSE;
  }
  if (deg < MIN_100DEG)
  {
    return -ANGLE_I_FALSE;
  }
  return deg;
}

// Servo angle set //////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Change the angle of the servo motor
 * @param[in] id Servo motor ID number
 * @param[in] pos position data
 * @return position data
 * @retval -1 out of range, communication failure
 **/
int IcsBaseClass::setPos(unsigned char id, unsigned int pos)
{
  unsigned char txCmd[3];
  unsigned char rxCmd[3];
  unsigned int rePos;
  bool flg;

  if ((id != idMax(id)) || (!maxMin(MAX_POS, MIN_POS, pos))) // When out of range
  {
    return ICS_FALSE;
  }

  txCmd[0] = 0x80 + id;             // CMD
  txCmd[1] = ((pos >> 7) & 0x007F); // POS_H
  txCmd[2] = (pos & 0x007F);        // POS_L

  // sending and receiving
  flg = synchronize(txCmd, sizeof txCmd, rxCmd, sizeof rxCmd);
  if (flg == false)
  {
    return ICS_FALSE;
  }

  rePos = ((rxCmd[1] << 7) & 0x3F80) + (rxCmd[2] & 0x007F);

  return rePos;
}

// Servo free mode ///////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Sets the servo motor to a free (weak) state.
 * @param[in] id Servo motor ID number
 * @return position data
 * @retval -1 out of range, communication failure
 **/
int IcsBaseClass::setFree(unsigned char id)
{
  unsigned char txCmd[3];
  unsigned char rxCmd[3];
  unsigned int rePos;
  bool flg;

  if (id != idMax(id)) // When out of range
  {
    return ICS_FALSE;
  }

  txCmd[0] = 0x80 + id; // CMD
  txCmd[1] = 0;
  txCmd[2] = 0;

  // sending and receiving
  flg = synchronize(txCmd, sizeof txCmd, rxCmd, sizeof rxCmd);
  if (flg == false)
  {
    return ICS_FALSE;
  }

  rePos = ((rxCmd[1] << 7) & 0x3F80) + (rxCmd[2] & 0x007F);

  return rePos;
}

// Stretch value write 1 to 127 /////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Write the servo motor stretch (holding force) value
 * @param[in] id Servo motor ID number
 * @param[in] strc Stretch data (1 to 127)
 * @return Written stretch data
 * @retval -1 Communication failure
 * @note Stretch writing 1-127 1 (weak) 127 (strong)
 **/
int IcsBaseClass::setStrc(unsigned char id, unsigned int strc)
{
  unsigned char txCmd[3];
  unsigned char rxCmd[3];
  unsigned int reData;
  bool flg;

  if ((id != idMax(id)) || (!maxMin(MAX_127, MIN_1, strc))) // When out of range
  {
    return ICS_FALSE;
  }

  txCmd[0] = 0xC0 + id; // CMD
  txCmd[1] = 0x01;      // SC stretch
  txCmd[2] = strc;      // stretch

  // sending and receiving
  flg = synchronize(txCmd, sizeof txCmd, rxCmd, sizeof rxCmd);
  if (flg == false)
  {
    return ICS_FALSE;
  }
  reData = rxCmd[2];

  return reData;
}

// Speed ​​value writing 1 to 127 ///////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Change the speed (output) of the servo motor
 * @param[in] id Servo motor ID number
 * @param[in] spd Speed ​​data (1 to 127)
 * @return Written speed data
 * @retval -1 Communication failure
 * @note Speed ​​writing 1-127 1(slow) 127(fast)
 **/
int IcsBaseClass::setSpd(unsigned char id, unsigned int spd)
{
  unsigned char txCmd[3];
  unsigned char rxCmd[3];
  unsigned int reData;
  bool flg;

  if ((id != idMax(id)) || (!maxMin(MAX_127, MIN_1, spd))) // When out of range
  {
    return ICS_FALSE;
  }

  txCmd[0] = 0xC0 + id; // CMD
  txCmd[1] = 0x02;      // SC speed
  txCmd[2] = spd;       // speed

  // sending and receiving
  flg = synchronize(txCmd, sizeof txCmd, rxCmd, sizeof rxCmd);
  if (flg == false)
  {
    return ICS_FALSE;
  }

  reData = rxCmd[2];

  return reData;
}

// Write current limit value 1 to 63 ////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Change the current limit value of the servo motor
 * @param[in] id Servo motor ID number
 * @param[in] curlim Current value data
 * @return Servo motor current limit value
 * @retval -1 Communication failure
 * @note Write current limit value 1 to 63 1 (low) 63 (high)
 **/
int IcsBaseClass::setCur(unsigned char id, unsigned int curlim)
{
  unsigned char txCmd[3];
  unsigned char rxCmd[3];
  unsigned int reData;
  bool flg;

  if ((id != idMax(id)) || (!maxMin(MAX_63, MIN_1, curlim))) // When out of range
  {
    return ICS_FALSE;
  }

  txCmd[0] = 0xC0 + id; // CMD
  txCmd[1] = 0x03;      // SC current value
  txCmd[2] = curlim;    // Current limit value

  // sending and receiving
  flg = synchronize(txCmd, sizeof txCmd, rxCmd, sizeof rxCmd);
  if (flg == false)
  {
    return ICS_FALSE;
  }

  reData = rxCmd[2];

  return reData;
}

// Temperature limit value writing 1 to 127 //////////////////////////////////////////////////////////////////////////////
/**
 * @brief Change the temperature limit value of the servo motor
 * @param[in] id Servo motor ID number
 * @param[in] tmplim Temperature limit value data
 * @return Servo motor temperature limit value
 * @retval -1 Communication failure
 * @note Temperature upper limit writing 1 to 127 127 (low temperature) 1 (high temperature)
 **/
int IcsBaseClass::setTmp(unsigned char id, unsigned int tmplim)
{
  unsigned char txCmd[3];
  unsigned char rxCmd[3];
  unsigned int reData;
  bool flg;
  //  if (id != idMax(id)) //範囲外の時
  //  {
  //    return ICS_FALSE;
  //  }
  //  tmplim = maxMin(MAX_127, MIN_1, tmplim);   //入力値範囲

  if ((id != idMax(id)) || (!maxMin(MAX_127, MIN_1, tmplim))) // When out of range
  {
    return ICS_FALSE;
  }

  txCmd[0] = 0xC0 + id; // CMD
  txCmd[1] = 0x04;      // SC temperature value
  txCmd[2] = tmplim;    // Temperature limit value

  // sending and receiving
  flg = synchronize(txCmd, sizeof txCmd, rxCmd, sizeof rxCmd);
  if (flg == false)
  {
    return ICS_FALSE;
  }

  reData = rxCmd[2];

  return reData;
}

// Stretch value readout 1 to 127 /////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Reads the servo motor stretch (holding force) value
 * @param[in] id Servo motor ID number
 * @return Loaded stretch data
 * @retval -1 Communication failure
 * @note Stretch reading 1 to 127 1 (weak) 127 (strong)
 **/
int IcsBaseClass::getStrc(unsigned char id)
{
  unsigned char txCmd[2];
  unsigned char rxCmd[3];
  unsigned int reData;
  bool flg;
  // id = idMax(id);          //ID範囲
  if (id != idMax(id)) // When out of range
  {
    return ICS_FALSE;
  }
  txCmd[0] = 0xA0 + id; // CMD
  txCmd[1] = 0x01;      // SC stretch

  // sending and receiving
  flg = synchronize(txCmd, sizeof txCmd, rxCmd, sizeof rxCmd);
  if (flg == false)
  {
    return ICS_FALSE;
  }

  reData = rxCmd[2];

  return reData;
}

// Speed ​​value readout 1 to 127 ///////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Read the speed (output) of the servo motor
 * @param[in] id Servo motor ID number
 * @return Read speed data
 * @retval -1 Communication failure
 * @note Speed ​​reading 1-127 1(slow) 127(fast)
 **/
int IcsBaseClass::getSpd(unsigned char id)
{
  unsigned char txCmd[2];
  unsigned char rxCmd[3];
  unsigned int reData;
  bool flg;
  if (id != idMax(id)) // When out of range
  {
    return ICS_FALSE;
  }
  txCmd[0] = 0xA0 + id; // CMD
  txCmd[1] = 0x02;      // SC speed

  // sending and receiving
  flg = synchronize(txCmd, sizeof txCmd, rxCmd, sizeof rxCmd);
  if (flg == false)
  {
    return ICS_FALSE;
  }

  reData = rxCmd[2];

  return reData;
}

// Current value readout 0 to 63 in forward rotation, 64 to 127 in reverse rotation ///////////////////////////////////////////////////////////
/**
 * @brief Reads the current value of the servo motor
 * @param[in] id Servo motor ID number
 * @return Current value of servo motor
 * @retval -1 Communication failure
 * @note Reading current value: 0 to 63 during forward rotation, 64 to 127 during reverse rotation
 **/
int IcsBaseClass::getCur(unsigned char id)
{
  unsigned char txCmd[2];
  unsigned char rxCmd[3];
  unsigned int reData;
  bool flg;
  if (id != idMax(id)) // When out of range
  {
    return ICS_FALSE;
  }
  txCmd[0] = 0xA0 + id; // CMD
  txCmd[1] = 0x03;      // SC current value

  // sending and receiving
  flg = synchronize(txCmd, sizeof txCmd, rxCmd, sizeof rxCmd);
  if (flg == false)
  {
    return ICS_FALSE;
  }

  reData = rxCmd[2];

  return reData;
}

// Temperature value readout 1 to 127 ////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Read the current temperature value of the servo motor
 * @param[in] id Servo motor ID number
 * @return Current temperature value of servo motor
 * @retval -1 Communication failure
 * @note Read current temperature 0 (high temperature) ~ 127 (low temperature)
 **/
int IcsBaseClass::getTmp(unsigned char id)
{
  unsigned char txCmd[2];
  unsigned char rxCmd[3];
  unsigned int reData;
  bool flg;
  if (id != idMax(id)) // When out of range
  {
    return ICS_FALSE;
  }
  txCmd[0] = 0xA0 + id; // CMD
  txCmd[1] = 0x04;      // SC temperature value

  // sending and receiving
  flg = synchronize(txCmd, sizeof txCmd, rxCmd, sizeof rxCmd);
  if (flg == false)
  {
    return ICS_FALSE;
  }

  reData = rxCmd[2];

  return reData;
}

// Read current value *Valid for ICS3.6 or later ///////////////////////////////////////////////////////////////////////////
/**
 * @brief Read the current position data of the servo motor
 * @param[in] id Servo motor ID number
 * @return Current position data of specified ID
 * @retval -1 Communication failure
 * @attention Valid from ICS3.6. No reply will be returned if sent to an ICS3.5 servo motor.
 **/
int IcsBaseClass::getPos(unsigned char id)
{
  unsigned char txCmd[2];
  unsigned char rxCmd[4];
  unsigned int reData;
  bool flg;
  if (id != idMax(id)) // When out of range
  {
    return ICS_FALSE;
  }
  txCmd[0] = 0xA0 + id; // CMD
  txCmd[1] = 0x05;      // Angle readout

  // sending and receiving
  flg = synchronize(txCmd, sizeof txCmd, rxCmd, sizeof rxCmd);
  if (flg == false)
  {
    return ICS_FALSE;
  }

  reData = ((rxCmd[2] << 7) & 0x3F80) + (rxCmd[3] & 0x007F);

  return reData;
}

// Loading ID ////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Get the ID of the ICS device
 * @return Returned ID data
 * @retval -1 out of range, communication failure
 * @date 2020/02/20 Added from Ver2.1.0
 * Please connect only one-to-one with @attention ID command. If you connect many, unintended data will be returned.
 **/
int IcsBaseClass::getID()
{
  unsigned char txCmd[4];
  unsigned char rxCmd[1];
  int id;
  bool flg;

  txCmd[0] = 0xFF; // CMD
  txCmd[1] = 0;    // ID loading
  txCmd[2] = 0;    // ID loading
  txCmd[3] = 0;    // ID loading

  // sending and receiving

  flg = synchronize(txCmd, sizeof txCmd, rxCmd, sizeof rxCmd);
  if (flg == false)
  {
    return ICS_FALSE;
  }

  delay(520); // It takes at least 500ms for a command to respond

  id = 0x1F & rxCmd[0]; // If you mask the data, it becomes an id.

  return id;
}

// Write ID ////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Write the ID of the ICS device
 * @param[in] id Servo motor ID number
 * @return Returned ID data
 * @retval -1 out of range, communication failure
 * @date 2020/02/20 Added from Ver2.1.0
 * Please connect only one-to-one with @attention ID command. If multiple IDs are connected, all IDs will be rewritten.
 **/
int IcsBaseClass::setID(unsigned char id)
{
  unsigned char txCmd[4];
  unsigned char rxCmd[1];
  int reID;
  bool flg;

  txCmd[0] = 0xE0 + id; // CMD
  txCmd[1] = 1;         // Write ID
  txCmd[2] = 1;         // Write ID
  txCmd[3] = 1;         // Write ID

  // sending and receiving

  flg = synchronize(txCmd, sizeof txCmd, rxCmd, sizeof rxCmd);
  if (flg == false)
  {
    return ICS_FALSE;
  }

  delay(520); // It takes at least 500ms for a command to respond

  reID = 0x1F & rxCmd[0]; // If you mask the data, it becomes an id.

  return id;
}
