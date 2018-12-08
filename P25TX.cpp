/*
 *   Copyright (C) 2016,2017,2018 by Jonathan Naylor G4KLX
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
#include "P25TX.h"

#include "P25Defines.h"

// Generated using rcosdesign(0.2, 8, 10, 'normal') in MATLAB
static float RC_0_2_FILTER[] = {-0.0135502F, -0.0273751F, -0.0400098F, -0.0499283F, -0.0556963F, -0.0561541F, -0.0506302F, -0.0390027F,
								-0.0217292F,  0.0000000F,  0.0244148F,  0.0492264F,  0.0718406F,  0.0896023F,  0.1000092F,  0.1010163F,
								 0.0913724F,  0.0706503F,  0.0395520F,  0.0000000F, -0.0451064F, -0.0918912F, -0.1357463F, -0.1717277F,
								-0.1948912F, -0.2008118F, -0.1858577F, -0.1476791F, -0.0854518F,  0.0000000F,  0.1060213F,  0.2283395F,
								 0.3611866F,  0.4977874F,  0.6306955F,  0.7523118F,  0.8554949F,  0.9340800F,  0.9832758F,  1.0000000F,
								 0.9832758F,  0.9340800F,  0.8554949F,  0.7523118F,  0.6306955F,  0.4977874F,  0.3611866F,  0.2283395F,
								 0.1060213F,  0.0000000F, -0.0854518F, -0.1476791F, -0.1858577F, -0.2008118F, -0.1948912F, -0.1717277F,
								-0.1357463F, -0.0918912F, -0.0451064F,  0.0000000F,  0.0395520F,  0.0706503F,  0.0913724F,  0.1010163F,
								 0.1000092F,  0.0896023F,  0.0718406F,  0.0492264F,  0.0244148F,  0.0000000F, -0.0217292F, -0.0390027F,
								-0.0506302F, -0.0561541F, -0.0556963F, -0.0499283F, -0.0400098F, -0.0273751F, -0.0135502F,  0.0000000F}; // numTaps = 80, L = 10
const uint16_t RC_0_2_FILTER_PHASE_LEN = 8U; // phaseLength = numTaps/L

// Generated in MATLAB using the following commands, and then normalised for unity gain
// shape2 = 'Inverse-sinc Lowpass';
// d2 = fdesign.interpolator(1, shape2);
// h2 = design(d2, 'SystemObject', true);
static float LOWPASS_FILTER[] = {0.0394910F, -0.0686972F, 0.1315958F, -0.2564165F, 0.6408582F, 0.6408582F, -0.2564165F, 0.1315958F,
								-0.0686972F, 0.0394910F};
const uint16_t LOWPASS_FILTER_LEN = 10U;

const float P25_LEVELA =  0.504;
const float P25_LEVELB =  0.168;
const float P25_LEVELC = -0.168;
const float P25_LEVELD = -0.504;

const uint8_t P25_START_SYNC = 0x77U;

CP25TX::CP25TX() :
m_buffer(4000U),
m_modFilter(P25_RADIO_SYMBOL_LENGTH, RC_0_2_FILTER_PHASE_LEN, RC_0_2_FILTER, 4U),
m_lpFilter(LOWPASS_FILTER_LEN, LOWPASS_FILTER, P25_RADIO_SYMBOL_LENGTH * 4U),
m_poBuffer(),
m_poLen(0U),
m_poPtr(0U),
m_txDelay(240U)       // 200ms
{
}

void CP25TX::process()
{
  if (m_buffer.getData() == 0U && m_poLen == 0U)
    return;

  if (m_poLen == 0U) {
    if (!m_tx) {
      for (uint16_t i = 0U; i < m_txDelay; i++)
        m_poBuffer[m_poLen++] = P25_START_SYNC;
    } else {
      uint8_t length = m_buffer.get();
      for (uint8_t i = 0U; i < length; i++) {
        uint8_t c = m_buffer.get();
        m_poBuffer[m_poLen++] = c;
      }
    }

    m_poPtr = 0U;
  }

  if (m_poLen > 0U) {
    uint16_t space = io.getSpace();
    
    while (space > (4U * P25_RADIO_SYMBOL_LENGTH)) {
      uint8_t c = m_poBuffer[m_poPtr++];
      writeByte(c);

      space -= 4U * P25_RADIO_SYMBOL_LENGTH;
      
      if (m_poPtr >= m_poLen) {
        m_poPtr = 0U;
        m_poLen = 0U;
        return;
      }
    }
  }
}

uint8_t CP25TX::writeData(const uint8_t* data, uint8_t length)
{
  if (length < (P25_TERM_FRAME_LENGTH_BYTES + 1U))
    return 4U;

  uint16_t space = m_buffer.getSpace();
  if (space < length)
    return 5U;

  m_buffer.put(length - 1U);
  for (uint8_t i = 0U; i < (length - 1U); i++)
    m_buffer.put(data[i + 1U]);

  return 0U;
}

void CP25TX::writeByte(uint8_t c)
{
  float inBuffer[4U];
  float intBuffer[P25_RADIO_SYMBOL_LENGTH * 4U];
  float outBuffer[P25_RADIO_SYMBOL_LENGTH * 4U];

  const uint8_t MASK = 0xC0U;

  for (uint8_t i = 0U; i < 4U; i++, c <<= 2) {
    switch (c & MASK) {
      case 0xC0U:
        inBuffer[i] = P25_LEVELA;
        break;
      case 0x80U:
        inBuffer[i] = P25_LEVELB;
        break;
      case 0x00U:
        inBuffer[i] = P25_LEVELC;
        break;
      default:
        inBuffer[i] = P25_LEVELD;
        break;
    }
  }

  m_modFilter.process(inBuffer, intBuffer, 4U);

  m_lpFilter.process(intBuffer, outBuffer, P25_RADIO_SYMBOL_LENGTH * 4U);

  io.write(STATE_P25, outBuffer, P25_RADIO_SYMBOL_LENGTH * 4U);
}

void CP25TX::setTXDelay(uint8_t delay)
{
  m_txDelay = 600U + uint16_t(delay) * 12U;        // 500ms + tx delay

  if (m_txDelay > 1200U)
    m_txDelay = 1200U;
}

uint8_t CP25TX::getSpace() const
{
  return m_buffer.getSpace() / P25_LDU_FRAME_LENGTH_BYTES;
}
