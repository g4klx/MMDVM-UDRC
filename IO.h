/*
 *   Copyright (C) 2015,2016,2017,2018 by Jonathan Naylor G4KLX
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#if !defined(IO_H)
#define  IO_H

#include "AudioCallback.h"
#include "Globals.h"
#include "SampleRB.h"
#include "Biquad.h"
#include "FIR.h"

class CIO : public IAudioCallback {
public:
  CIO();

  void start();

  void process();

  void write(MMDVM_STATE mode, float* samples, uint16_t length);

  uint16_t getSpace() const;

  void setDecode(bool dcd);
  void setADCDetection(bool detect);
  void setMode();

  void interrupt();

  void setParameters(bool rxInvert, bool txInvert, bool pttInvert, float rxLevel, float cwIdTXLevel, float dstarTXLevel, float dmrTXLevel, float ysfTXLevel, float p25TXLevel, float nxdnTXLevel, float pocsagTXLevel, float txDCOffset, float rxDCOffset);

  void getOverflow(bool& adcOverflow, bool& dacOverflow);

  bool hasTXOverflow();
  bool hasRXOverflow();

  bool hasLockout() const;

  void resetWatchdog();
  uint32_t getWatchdog();

  virtual void readCallback(const float* input, unsigned int nSamples);
  virtual void writeCallback(float* output, int& nSamples);

private:
  bool                 m_started;
  CSampleRB            m_rxBuffer;
  CSampleRB            m_txBuffer;

  CBiquad              m_dcFilter;

  CFIR                 m_rrcFilter;
  CFIR                 m_gaussianFilter;
  CFIR                 m_boxcarFilter;
  CFIR                 m_nxdnFilter;
  CFIR                 m_nxdnISincFilter;

  bool                 m_pttInvert;
  float                m_rxLevel;
  float                m_cwIdTXLevel;
  float                m_dstarTXLevel;
  float                m_dmrTXLevel;
  float                m_ysfTXLevel;
  float                m_p25TXLevel;
  float                m_nxdnTXLevel;
  float                m_pocsagTXLevel;

  float                m_rxDCOffset;
  float                m_txDCOffset;

  uint32_t             m_ledCount;
  bool                 m_ledValue;

  bool                 m_detect;

  uint16_t             m_adcOverflow;
  uint16_t             m_dacOverflow;

  volatile uint32_t    m_watchdog;

  bool                 m_lockout;

  // Hardware specific routines
  void initInt();
  void startInt();

  bool getCOSInt();

  void setLEDInt(bool on);
  void setPTTInt(bool on);
  void setCOSInt(bool on);
};

#endif

