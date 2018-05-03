/*
 *   Copyright (C) 2009-2018 by Jonathan Naylor G4KLX
 *   Copyright (C) 2017 by Andy Uribe CA6JAU
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
#include "YSFTX.h"

#include "YSFDefines.h"

// Generated using rcosdesign(0.2, 8, 5, 'sqrt') in MATLAB
const float RRC_0_2_FILTER[] = {0.0000000F,  0.0000000F,  0.0000000F,  0.0000000F, 0.0259399F,  0.0066833F, -0.0219727F, -0.0472412F,
			       -0.0547791F, -0.0357666F,  0.0072327F,  0.0588074F, 0.0952148F,  0.0937805F,  0.0441589F, -0.0436707F,
			       -0.1386719F, -0.1965942F, -0.1750183F, -0.0498352F, 0.1724548F,  0.4523315F,  0.7266235F,  0.9267273F,
			        0.9999695F,  0.9267273F,  0.7266235F,  0.4523315F, 0.1724548F, -0.0498352F, -0.1750183F, -0.1965942F,
			       -0.1386719F, -0.0436707F,  0.0441589F,  0.0937805F, 0.0952148F,  0.0588074F,  0.0072327F, -0.0357666F,
			       -0.0547791F, -0.0472412F, -0.0219727F,  0.0066833F, 0.0259399F}; // numTaps = 45, L = 5
const uint16_t RRC_0_2_FILTER_PHASE_LEN = 9U; // phaseLength = numTaps/L

const float YSF_LEVELA_HI =  0.757;
const float YSF_LEVELB_HI =  0.252;
const float YSF_LEVELC_HI = -0.252;
const float YSF_LEVELD_HI = -0.757;

const float YSF_LEVELA_LO =  0.379;
const float YSF_LEVELB_LO =  0.126;
const float YSF_LEVELC_LO = -0.126;
const float YSF_LEVELD_LO = -0.379;

const uint8_t YSF_START_SYNC = 0x77U;
const uint8_t YSF_END_SYNC   = 0xFFU;

CYSFTX::CYSFTX() :
m_buffer(1500U),
m_modFilter(YSF_RADIO_SYMBOL_LENGTH, RRC_0_2_FILTER_PHASE_LEN, RRC_0_2_FILTER, 4U),
m_poBuffer(),
m_poLen(0U),
m_poPtr(0U),
m_txDelay(240U),      // 200ms
m_loDev(false)
{
}

void CYSFTX::process()
{
  if (m_buffer.getData() == 0U && m_poLen == 0U)
    return;

  if (m_poLen == 0U) {
    if (!m_tx) {
      for (uint16_t i = 0U; i < m_txDelay; i++)
        m_poBuffer[m_poLen++] = YSF_START_SYNC;
    } else {
      for (uint8_t i = 0U; i < YSF_FRAME_LENGTH_BYTES; i++) {
        uint8_t c = m_buffer.get();
        m_poBuffer[m_poLen++] = c;
      }
    }

    m_poPtr = 0U;
  }

  if (m_poLen > 0U) {
    uint16_t space = io.getSpace();
    
    while (space > (4U * YSF_RADIO_SYMBOL_LENGTH)) {
      uint8_t c = m_poBuffer[m_poPtr++];
      writeByte(c);

      space -= 4U * YSF_RADIO_SYMBOL_LENGTH;
      
      if (m_poPtr >= m_poLen) {
        m_poPtr = 0U;
        m_poLen = 0U;
        return;
      }
    }
  }
}

uint8_t CYSFTX::writeData(const uint8_t* data, uint8_t length)
{
  if (length != (YSF_FRAME_LENGTH_BYTES + 1U))
    return 4U;

  uint16_t space = m_buffer.getSpace();
  if (space < YSF_FRAME_LENGTH_BYTES)
    return 5U;

  for (uint8_t i = 0U; i < YSF_FRAME_LENGTH_BYTES; i++)
    m_buffer.put(data[i + 1U]);

  return 0U;
}

void CYSFTX::writeByte(uint8_t c)
{
  float inBuffer[4U];
  float outBuffer[YSF_RADIO_SYMBOL_LENGTH * 4U];

  const uint8_t MASK = 0xC0U;

  for (uint8_t i = 0U; i < 4U; i++, c <<= 2) {
    switch (c & MASK) {
      case 0xC0U:
        inBuffer[i] = m_loDev ? YSF_LEVELA_LO : YSF_LEVELA_HI;
        break;
      case 0x80U:
        inBuffer[i] = m_loDev ? YSF_LEVELB_LO : YSF_LEVELB_HI;
        break;
      case 0x00U:
        inBuffer[i] = m_loDev ? YSF_LEVELC_LO : YSF_LEVELC_HI;
        break;
      default:
        inBuffer[i] = m_loDev ? YSF_LEVELD_LO : YSF_LEVELD_HI;
        break;
    }
  }

  m_modFilter.process(inBuffer, outBuffer, 4U);

  io.write(STATE_YSF, outBuffer, YSF_RADIO_SYMBOL_LENGTH * 4U);
}

void CYSFTX::setTXDelay(uint8_t delay)
{
  m_txDelay = 600U + uint16_t(delay) * 12U;        // 500ms + tx delay

  if (m_txDelay > 1200U)
    m_txDelay = 1200U;
}

uint8_t CYSFTX::getSpace() const
{
  return m_buffer.getSpace() / YSF_FRAME_LENGTH_BYTES;
}

void CYSFTX::setLoDev(bool on)
{
  m_loDev = on;
}

