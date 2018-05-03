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

#if !defined(P25RX_H)
#define  P25RX_H

#include "P25Defines.h"

enum P25RX_STATE {
  P25RXS_NONE,
  P25RXS_HDR,
  P25RXS_LDU
};

class CP25RX {
public:
  CP25RX();

  void samples(const float* samples, uint8_t length);

  void reset();

private:
  P25RX_STATE m_state;
  uint32_t    m_bitBuffer[P25_RADIO_SYMBOL_LENGTH];
  float       m_buffer[P25_LDU_FRAME_LENGTH_SAMPLES];
  uint16_t    m_bitPtr;
  uint16_t    m_dataPtr;
  uint16_t    m_hdrStartPtr;
  uint16_t    m_lduStartPtr;
  uint16_t    m_lduEndPtr;
  uint16_t    m_minSyncPtr;
  uint16_t    m_maxSyncPtr;
  uint16_t    m_hdrSyncPtr;
  uint16_t    m_lduSyncPtr;
  float       m_maxCorr;
  uint16_t    m_lostCount;
  uint8_t     m_countdown;
  float       m_centre[16U];
  float       m_centreVal;
  float       m_threshold[16U];
  float       m_thresholdVal;
  uint8_t     m_averagePtr;

  void processNone(float sample);
  void processHdr(float sample);
  void processLdu(float sample);
  bool correlateSync();
  void calculateLevels(uint16_t start, uint16_t count);
  void samplesToBits(uint16_t start, uint16_t count, uint8_t* buffer, uint16_t offset, float centre, float threshold);
  void writeLdu(uint8_t* ldu);
};

#endif
