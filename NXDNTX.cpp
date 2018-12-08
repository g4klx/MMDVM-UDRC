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

// NXDN RRC filter + SINC filter
static float RRC_0_2_FILTER[] = {0.0015564F,  0.0021668F,  0.0028077F,  0.0034181F,  0.0039369F,  0.0043031F,  0.0043947F,  0.0042116F,
								 0.0036622F,  0.0026551F,  0.0011902F, -0.0007935F, -0.0032655F, -0.0062258F, -0.0095828F, -0.0133366F,
								-0.0173040F, -0.0214240F, -0.0254830F, -0.0284433F, -0.0325327F, -0.0358898F, -0.0383618F, -0.0397961F,
								-0.0401013F, -0.0391858F, -0.0370190F, -0.0335398F, -0.0288095F, -0.0228584F, -0.0158391F, -0.0078433F,
								 0.0009156F,  0.0102237F,  0.0198676F,  0.0295419F,  0.0389721F,  0.0478835F,  0.0559099F,  0.0628071F,
								 0.0682699F,  0.0720237F,  0.0738548F,  0.0736106F,  0.0710776F,  0.0662252F,  0.0590533F,  0.0495315F,
								 0.0378124F,  0.0241096F,  0.0086367F, -0.0082705F, -0.0261849F, -0.0447401F, -0.0633564F, -0.0815149F,
								-0.0986663F, -0.1142003F, -0.1275063F, -0.1379742F, -0.1450850F, -0.1482284F, -0.1469466F, -0.1408124F,
								-0.1294595F, -0.1126133F, -0.0900906F, -0.0618000F, -0.0278329F,  0.0117496F,  0.0566118F,  0.1064486F,
								 0.1607105F,  0.2188787F,  0.2802515F,  0.3440657F,  0.4094974F,  0.4756310F,  0.5415815F,  0.6063722F,
								 0.6690573F,  0.7286599F,  0.7843257F,  0.8351085F,  0.8802759F,  0.9190955F,  0.9509262F,  0.9752495F,
								 0.9916990F,  1.0000000F,  1.0000000F,  0.9916990F,  0.9752495F,  0.9509262F,  0.9190955F,  0.8802759F,
								 0.8351085F,  0.7843257F,  0.7286599F,  0.6690573F,  0.6063722F,  0.5415815F,  0.4756310F,  0.4094974F,
								 0.3440657F,  0.2802515F,  0.2188787F,  0.1607105F,  0.1064486F,  0.0566118F,  0.0117496F, -0.0278329F,
								-0.0618000F, -0.0900906F, -0.1126133F, -0.1294595F, -0.1408124F, -0.1469466F, -0.1482284F, -0.1450850F,
								-0.1379742F, -0.1275063F, -0.1142003F, -0.0986663F, -0.0815149F, -0.0633564F, -0.0447401F, -0.0261849F,
								-0.0082705F,  0.0086367F,  0.0241096F,  0.0378124F,  0.0495315F,  0.0590533F,  0.0662252F,  0.0710776F,
								 0.0736106F,  0.0738548F,  0.0720237F,  0.0682699F,  0.0628071F,  0.0559099F,  0.0478835F,  0.0389721F,
								 0.0295419F,  0.0198676F,  0.0102237F,  0.0009156F, -0.0078433F, -0.0158391F, -0.0228584F, -0.0288095F,
								-0.0335398F, -0.0370190F, -0.0391858F, -0.0401013F, -0.0397961F, -0.0383618F, -0.0358898F, -0.0325327F,
								-0.0284433F, -0.0254830F, -0.0214240F, -0.0173040F, -0.0133366F, -0.0095828F, -0.0062258F, -0.0032655F,
								-0.0007935F,  0.0011902F,  0.0026551F,  0.0036622F,  0.0042116F,  0.0043947F,  0.0043031F,  0.0039369F,
								 0.0034181F,  0.0028077F,  0.0021668F,  0.0015564F}; // numTaps = 180, L = 20
const uint16_t RRC_0_2_FILTER_PHASE_LEN = 9U; // phaseLength = numTaps/L

const float NXDN_LEVELA =  0.294;
const float NXDN_LEVELB =  0.098;
const float NXDN_LEVELC = -0.098;
const float NXDN_LEVELD = -0.294;

const uint8_t NXDN_PREAMBLE[] = {0x57U, 0x75U, 0xFDU};
const uint8_t NXDN_SYNC = 0x5FU;

CNXDNTX::CNXDNTX() :
m_buffer(4000U),
m_modFilter(NXDN_RADIO_SYMBOL_LENGTH, RRC_0_2_FILTER_PHASE_LEN, RRC_0_2_FILTER, 4U),
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

  m_modFilter.process(inBuffer, outBuffer, 4U);

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

