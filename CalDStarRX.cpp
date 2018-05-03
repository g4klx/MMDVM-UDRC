/*
 *   Copyright (C) 2009-2016,2018 by Jonathan Naylor G4KLX
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

#include "Globals.h"
#include "CalDStarRX.h"
#include "Utils.h"

const unsigned int BUFFER_LENGTH = 200U;

const uint32_t PLLMAX = 0x10000U;
const uint32_t PLLINC = PLLMAX / DSTAR_RADIO_SYMBOL_LENGTH;
const uint32_t INC    = PLLINC / 32U;

// D-Star bit order version of 0x55 0x2D 0x16
const uint32_t DATA_SYNC_DATA1 = 0x00AAB468U;
const uint32_t DATA_SYNC_DATA2 = 0x00554B97U;
const uint32_t DATA_SYNC_MASK  = 0x00FFFFFFU;
const uint8_t  DATA_SYNC_ERRS  = 2U;

CCalDStarRX::CCalDStarRX() :
m_pll(0U),
m_prev(false),
m_patternBuffer(0x00U),
m_rxBuffer(),
m_ptr(0U)
{
}

void CCalDStarRX::samples(const float* samples, uint8_t length)
{
  for (uint16_t i = 0U; i < length; i++) {
    bool bit = samples[i] < 0.0F;

    if (bit != m_prev) {
      if (m_pll < (PLLMAX / 2U))
        m_pll += INC;
      else
        m_pll -= INC;
    }

    m_prev = bit;

    m_pll += PLLINC;

    if (m_pll >= PLLMAX) {
      m_pll -= PLLMAX;
      process(samples[i]);
    }
  }
}

void CCalDStarRX::process(float value)
{
  m_patternBuffer <<= 1;
  if (value < 0.0F)
    m_patternBuffer |= 0x01U;

  m_rxBuffer[m_ptr++] = value;
  if (m_ptr >= 24U)
    m_ptr = 0U;

  if (countBits32((m_patternBuffer & DATA_SYNC_MASK) ^ DATA_SYNC_DATA1) <= DATA_SYNC_ERRS) {
    float max = -1.0F;
    float min =  1.0F;

    for (uint8_t i = 0U; i < 24U; i++) {
      float value = m_rxBuffer[i];

      if (value > max)
        max = value;
      if (value < min)
        min = value;
    }

    int16_t intMin = int16_t(min * 2048.0F + 0.5F);
    int16_t intMax = int16_t(max * 2048.0F + 0.5F);

    if ((intMax - intMin) > 5) {
      uint8_t buffer[5U];
      buffer[0U] = 0x00U;
      buffer[1U] = (intMax >> 8) & 0xFFU;
      buffer[2U] = (intMax >> 0) & 0xFFU;
      buffer[3U] = (intMin >> 8) & 0xFFU;
      buffer[4U] = (intMin >> 0) & 0xFFU;

      serial.writeCalData(buffer, 5U);
    }
  }

  if (countBits32((m_patternBuffer & DATA_SYNC_MASK) ^ DATA_SYNC_DATA2) <= DATA_SYNC_ERRS) {
    float max = -1.0F;
    float min =  1.0F;

    for (uint8_t i = 0U; i < 24U; i++) {
      float value = m_rxBuffer[i];

      if (value > max)
        max = value;
      if (value < min)
        min = value;
    }

    int16_t intMin = int16_t(min * 2048.0F + 0.5F);
    int16_t intMax = int16_t(max * 2048.0F + 0.5F);

    if ((intMax - intMin) > 5) {
      uint8_t buffer[5U];
      buffer[0U] = 0x80U;
      buffer[1U] = (intMax >> 8) & 0xFFU;
      buffer[2U] = (intMax >> 0) & 0xFFU;
      buffer[3U] = (intMin >> 8) & 0xFFU;
      buffer[4U] = (intMin >> 0) & 0xFFU;

      serial.writeCalData(buffer, 5U);
    }
  }
}

