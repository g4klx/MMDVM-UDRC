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
#include "NXDNTX.h"

#include "NXDNDefines.h"

// Generated using rcosdesign(0.2, 8, 10, 'sqrt') in MATLAB
const float RRC_0_2_FILTER[] = {0.0000000F,  0.0000000F,  0.0000000F,  0.0000000F,  0.0000000F,  0.0000000F,  0.0000000F,  0.0000000F,
				0.0000000F,  0.0259399F,  0.0180664F,  0.0066833F, -0.0071411F, -0.0219727F, -0.0359802F, -0.0472412F,
			       -0.0539856F, -0.0547791F, -0.0487366F, -0.0357666F, -0.0166016F,  0.0072327F,  0.0333252F,  0.0588074F,
			        0.0804749F,  0.0952148F,  0.1002808F,  0.0937805F,  0.0748901F,  0.0441589F,  0.0035400F, -0.0436707F,
			       -0.0928650F, -0.1386719F, -0.1751404F, -0.1965942F, -0.1978455F, -0.1750183F, -0.1257629F, -0.0498352F,
			        0.0509338F,  0.1724548F,  0.3087769F,  0.4523315F,  0.5946045F,  0.7266235F,  0.8398438F,  0.9267273F,
			        0.9813232F,  0.9999695F,  0.9813232F,  0.9267273F,  0.8398438F,  0.7266235F,  0.5946045F,  0.4523315F,
			        0.3087769F,  0.1724548F,  0.0509338F, -0.0498352F, -0.1257629F, -0.1750183F, -0.1978455F, -0.1965942F,
			       -0.1751404F, -0.1386719F, -0.0928650F, -0.0436707F,  0.0035400F,  0.0441589F,  0.0748901F,  0.0937805F,
			        0.1002808F,  0.0952148F,  0.0804749F,  0.0588074F,  0.0333252F,  0.0072327F, -0.0166016F, -0.0357666F,
			       -0.0487366F, -0.0547791F, -0.0539856F, -0.0472412F, -0.0359802F, -0.0219727F, -0.0071411F,  0.0066833F,
			        0.0180664F, 0.0259399F}; // numTaps = 90, L = 10
const uint16_t RRC_0_2_FILTER_PHASE_LEN = 9U; // phaseLength = numTaps/L

const float NXDN_SINC_FILTER[] = {0.0174561F, -0.0306091F, -0.0077209F,  0.0077515F,  0.0225830F, 0.0393677F, 0.0580444F, 0.0771179F,
				  0.0942993F,  0.1073303F,  0.1143494F,  0.1143494F,  0.1073303F, 0.0942993F, 0.0771179F, 0.0580444F,
				  0.0393677F,  0.0225830F,  0.0077515F, -0.0077209F, -0.0306091F, 0.0174561F};
const uint16_t NXDN_SINC_FILTER_LEN = 22U;

const float NXDN_LEVELA =  0.294;
const float NXDN_LEVELB =  0.098;
const float NXDN_LEVELC = -0.098;
const float NXDN_LEVELD = -0.294;

const uint8_t NXDN_PREAMBLE[] = {0x57U, 0x75U, 0xFDU};
const uint8_t NXDN_SYNC = 0x5FU;

CNXDNTX::CNXDNTX() :
m_buffer(4000U),
m_modFilter(NXDN_RADIO_SYMBOL_LENGTH, RRC_0_2_FILTER_PHASE_LEN, RRC_0_2_FILTER, 4U),
m_sincFilter(NXDN_SINC_FILTER_LEN, NXDN_SINC_FILTER, NXDN_RADIO_SYMBOL_LENGTH * 4U),
m_poBuffer(),
m_poLen(0U),
m_poPtr(0U),
m_txDelay(240U)      // 200ms
{
}

void CNXDNTX::process()
{
  if (m_buffer.getData() == 0U && m_poLen == 0U)
    return;

  if (m_poLen == 0U) {
    if (!m_tx) {
      for (uint16_t i = 0U; i < m_txDelay; i++)
        m_poBuffer[m_poLen++] = NXDN_SYNC;
      m_poBuffer[m_poLen++] = NXDN_PREAMBLE[0U];
      m_poBuffer[m_poLen++] = NXDN_PREAMBLE[1U];
      m_poBuffer[m_poLen++] = NXDN_PREAMBLE[2U];
    } else {
      for (uint8_t i = 0U; i < NXDN_FRAME_LENGTH_BYTES; i++) {
        uint8_t c = m_buffer.get();
        m_poBuffer[m_poLen++] = c;
      }
    }

    m_poPtr = 0U;
  }

  if (m_poLen > 0U) {
    uint16_t space = io.getSpace();
    
    while (space > (4U * NXDN_RADIO_SYMBOL_LENGTH)) {
      uint8_t c = m_poBuffer[m_poPtr++];
      writeByte(c);

      space -= 4U * NXDN_RADIO_SYMBOL_LENGTH;
      
      if (m_poPtr >= m_poLen) {
        m_poPtr = 0U;
        m_poLen = 0U;
        return;
      }
    }
  }
}

uint8_t CNXDNTX::writeData(const uint8_t* data, uint8_t length)
{
  if (length != (NXDN_FRAME_LENGTH_BYTES + 1U))
    return 4U;

  uint16_t space = m_buffer.getSpace();
  if (space < NXDN_FRAME_LENGTH_BYTES)
    return 5U;

  for (uint8_t i = 0U; i < NXDN_FRAME_LENGTH_BYTES; i++)
    m_buffer.put(data[i + 1U]);

  return 0U;
}

void CNXDNTX::writeByte(uint8_t c)
{
  float inBuffer[4U];
  float intBuffer[NXDN_RADIO_SYMBOL_LENGTH * 4U];
  float outBuffer[NXDN_RADIO_SYMBOL_LENGTH * 4U];

  const uint8_t MASK = 0xC0U;

  for (uint8_t i = 0U; i < 4U; i++, c <<= 2) {
    switch (c & MASK) {
      case 0xC0U:
        inBuffer[i] = NXDN_LEVELA;
        break;
      case 0x80U:
        inBuffer[i] = NXDN_LEVELB;
        break;
      case 0x00U:
        inBuffer[i] = NXDN_LEVELC;
        break;
      default:
        inBuffer[i] = NXDN_LEVELD;
        break;
    }
  }

  m_modFilter.process(inBuffer, intBuffer, 4U);

  m_sincFilter.process(intBuffer, outBuffer, NXDN_RADIO_SYMBOL_LENGTH * 4U);

  io.write(STATE_NXDN, outBuffer, NXDN_RADIO_SYMBOL_LENGTH * 4U);
}

void CNXDNTX::setTXDelay(uint8_t delay)
{
  m_txDelay = 300U + uint16_t(delay) * 6U;        // 500ms + tx delay

  if (m_txDelay > 1200U)
    m_txDelay = 1200U;
}

uint8_t CNXDNTX::getSpace() const
{
  return m_buffer.getSpace() / NXDN_FRAME_LENGTH_BYTES;
}

