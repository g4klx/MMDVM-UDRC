/*
 *   Copyright (C) 2015,2016,2017,2018 by Jonathan Naylor G4KLX
 *   Copyright (C) 2015 by Jim Mclaughlin KI6ZUM
 *   Copyright (C) 2016 by Colin Durbridge G4EML
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
#include "IO.h"

// Generated using [b, a] = butter(1, 0.001) in MATLAB
const float DC_FILTER[] = {0.001568334F, 0.001568334F, 0.000000000F, 0.996863306F, 0.000000000F}; // {b0, b1, b2, -a1, -a2}
const uint32_t DC_FILTER_STAGES = 1U; // One Biquad stage

// Generated using rcosdesign(0.2, 8, 5, 'sqrt') in MATLAB
const float RRC_0_2_FILTER[] = {0.0122375F,  0.0031738F, -0.0103760F, -0.0223083F, -0.0258484F, -0.0168762F,  0.0034180F,  0.0277405F,
				0.0449219F,  0.0442505F,  0.0208435F, -0.0205994F, -0.0654297F, -0.0927734F, -0.0825806F, -0.0234985F,
				0.0813904F,  0.2134705F,  0.3429260F,  0.4373474F,  0.4719238F,  0.4373474F,  0.3429260F,  0.2134705F,
				0.0813904F, -0.0234985F, -0.0825806F, -0.0927734F, -0.0654297F, -0.0205994F,  0.0208435F,  0.0442505F,
				0.0449219F,  0.0277405F,  0.0034180F, -0.0168762F, -0.0258484F, -0.0223083F, -0.0103760F,  0.0031738F,
				0.0122375F,  0.0000000F};
const uint16_t RRC_0_2_FILTER_LEN = 42U;

// Generated using rcosdesign(0.2, 8, 10, 'sqrt') in MATLAB
const float NXDN_0_2_FILTER[] = {0.0086670F,  0.0060425F,  0.0022278F, -0.0023804F, -0.0073242F, -0.0119934F, -0.0157776F, -0.0180054F,
				-0.0182800F, -0.0162659F, -0.0119324F, -0.0055237F,  0.0024109F,  0.0111084F,  0.0196228F,  0.0268555F,
				 0.0317688F,  0.0334778F,  0.0313110F,  0.0249939F,  0.0147400F,  0.0011902F, -0.0145569F, -0.0310059F,
				-0.0462646F, -0.0584412F, -0.0656128F, -0.0660400F, -0.0584106F, -0.0419617F, -0.0166321F,  0.0169983F,
				 0.0575562F,  0.1030273F,  0.1509399F,  0.1984253F,  0.2424927F,  0.2802734F,  0.3092651F,  0.3274841F,
				 0.3337097F,  0.3274841F,  0.3092651F,  0.2802734F,  0.2424927F,  0.1984253F,  0.1509399F,  0.1030273F,
				 0.0575562F,  0.0169983F, -0.0166321F, -0.0419617F, -0.0584106F, -0.0660400F, -0.0656128F, -0.0584412F,
				-0.0462646F, -0.0310059F, -0.0145569F,  0.0011902F,  0.0147400F,  0.0249939F,  0.0313110F,  0.0334778F,
				 0.0317688F,  0.0268555F,  0.0196228F,  0.0111084F,  0.0024109F, -0.0055237F, -0.0119324F, -0.0162659F,
				-0.0182800F, -0.0180054F, -0.0157776F, -0.0119934F, -0.0073242F, -0.0023804F,  0.0022278F,  0.0060425F,
				 0.0086670F,  0.0000000F};
const uint16_t NXDN_0_2_FILTER_LEN = 82U;

const float NXDN_ISINC_FILTER[] = { 0.0241089F, -0.0331116F, -0.0327454F, -0.0168762F,  0.0227966F,  0.0714417F,  0.0963135F,  0.0656738F,
				   -0.0272522F, -0.1499939F, -0.2390747F, -0.2299805F, -0.0946655F,  0.1355286F,  0.3770142F,  0.5308228F,
				    0.5308228F,  0.3770142F,  0.1355286F, -0.0946655F, -0.2299805F, -0.2390747F, -0.1499939F, -0.0272522F,
				    0.0656738F,  0.0963135F,  0.0714417F,  0.0227966F, -0.0168762F, -0.0327454F, -0.0331116F,  0.0241089F};
const uint16_t NXDN_ISINC_FILTER_LEN = 32U;

// Generated using gaussfir(0.5, 4, 5) in MATLAB
const float GAUSSIAN_0_5_FILTER[] = {0.0002441F, 0.0031738F, 0.0231934F, 0.0963745F, 0.2264709F, 0.3010864F, 0.2264709F, 0.0963745F,
				     0.0231934F, 0.0031738F, 0.0002441F, 0.0000000F};
const uint16_t GAUSSIAN_0_5_FILTER_LEN = 12U;

// One symbol boxcar filter
const float BOXCAR_FILTER[] = {0.3662109F, 0.3662109F, 0.3662109F, 0.3662109F, 0.3662109F, 0.0000000F};
const uint16_t BOXCAR_FILTER_LEN = 6U;

const float DC_OFFSET = 0.0F;

CIO::CIO() :
m_started(false),
m_rxBuffer(RX_RINGBUFFER_SIZE),
m_txBuffer(TX_RINGBUFFER_SIZE),
m_dcFilter(DC_FILTER_STAGES, DC_FILTER),
m_rrcFilter(RRC_0_2_FILTER_LEN, RRC_0_2_FILTER, RX_BLOCK_SIZE),
m_gaussianFilter(GAUSSIAN_0_5_FILTER_LEN, GAUSSIAN_0_5_FILTER, RX_BLOCK_SIZE),
m_boxcarFilter(BOXCAR_FILTER_LEN, BOXCAR_FILTER, RX_BLOCK_SIZE),
m_nxdnFilter(NXDN_0_2_FILTER_LEN, NXDN_0_2_FILTER, RX_BLOCK_SIZE),
m_nxdnISincFilter(NXDN_ISINC_FILTER_LEN, NXDN_ISINC_FILTER, RX_BLOCK_SIZE),
m_pttInvert(false),
m_rxLevel(0.5F),
m_cwIdTXLevel(0.5F),
m_dstarTXLevel(0.5F),
m_dmrTXLevel(0.5F),
m_ysfTXLevel(0.5F),
m_p25TXLevel(0.5F),
m_nxdnTXLevel(0.5F),
m_pocsagTXLevel(0.5F),
m_rxDCOffset(DC_OFFSET),
m_txDCOffset(DC_OFFSET),
m_ledCount(0U),
m_ledValue(true),
m_detect(false),
m_adcOverflow(0U),
m_dacOverflow(0U),
m_watchdog(0U),
m_lockout(false)
{
  initInt();
}

void CIO::start()
{
  if (m_started)
    return;

  startInt();

  m_started = true;

  setMode();
}

void CIO::process()
{
  m_ledCount++;
  if (m_started) {
    // Two seconds timeout
    if (m_watchdog >= 48000U) {
      if (m_modemState == STATE_DSTAR || m_modemState == STATE_DMR || m_modemState == STATE_YSF || m_modemState == STATE_P25 || m_modemState == STATE_NXDN) {
        m_modemState = STATE_IDLE;
        setMode();
      }

      m_watchdog = 0U;
    }

    if (m_ledCount >= 24000U) {
      m_ledCount = 0U;
      m_ledValue = !m_ledValue;
      setLEDInt(m_ledValue);
    }
  } else {
    if (m_ledCount >= 240000U) {
      m_ledCount = 0U;
      m_ledValue = !m_ledValue;
      setLEDInt(m_ledValue);
    }
    return;
  }

  m_lockout = getCOSInt();

  // Switch off the transmitter if needed
  if (m_txBuffer.getData() == 0U && m_tx) {
    m_tx = false;
    setPTTInt(m_pttInvert ? true : false);
  }

  if (m_rxBuffer.getData() >= RX_BLOCK_SIZE) {
    float samples[RX_BLOCK_SIZE];

    for (uint16_t i = 0U; i < RX_BLOCK_SIZE; i++) {
      float sample;
      m_rxBuffer.get(sample);

      // Detect ADC overflow
      if (m_detect && (sample == -1.0F || sample == 1.0F))
        m_adcOverflow++;

      samples[i] = (sample - m_rxDCOffset) * m_rxLevel;
    }

    if (m_lockout)
      return;

    float dcValues[RX_BLOCK_SIZE];
    m_dcFilter.process(samples, dcValues, RX_BLOCK_SIZE);

    float offset = 0.0F;
    for (uint8_t i = 0U; i < RX_BLOCK_SIZE; i++)
      offset += dcValues[i];
    offset /= float(RX_BLOCK_SIZE);

    float dcSamples[RX_BLOCK_SIZE];
    for (uint8_t i = 0U; i < RX_BLOCK_SIZE; i++)
      dcSamples[i] = samples[i] - offset;

    if (m_modemState == STATE_IDLE) {
      if (m_dstarEnable) {
        float GMSKVals[RX_BLOCK_SIZE];
        m_gaussianFilter.process(dcSamples, GMSKVals, RX_BLOCK_SIZE);
        dstarRX.samples(GMSKVals, RX_BLOCK_SIZE);
      }

      if (m_p25Enable) {
        float P25Vals[RX_BLOCK_SIZE];
        m_boxcarFilter.process(dcSamples, P25Vals, RX_BLOCK_SIZE);
        p25RX.samples(P25Vals, RX_BLOCK_SIZE);
      }

      if (m_nxdnEnable) {
        float NXDNValsTmp[RX_BLOCK_SIZE];
        m_nxdnFilter.process(dcSamples, NXDNValsTmp, RX_BLOCK_SIZE);
        float NXDNVals[RX_BLOCK_SIZE];
        m_nxdnISincFilter.process(NXDNValsTmp, NXDNVals, RX_BLOCK_SIZE);

        nxdnRX.samples(NXDNVals, RX_BLOCK_SIZE);
      }

      if (m_dmrEnable || m_ysfEnable) {
        float RRCVals[RX_BLOCK_SIZE];
        m_rrcFilter.process(samples, RRCVals, RX_BLOCK_SIZE);

        if (m_ysfEnable)
          ysfRX.samples(RRCVals, RX_BLOCK_SIZE);

        if (m_dmrEnable)
          dmrDMORX.samples(RRCVals, RX_BLOCK_SIZE);
      }
    } else if (m_modemState == STATE_DSTAR) {
      if (m_dstarEnable) {
        float GMSKVals[RX_BLOCK_SIZE];
        m_gaussianFilter.process(dcSamples, GMSKVals, RX_BLOCK_SIZE);
        dstarRX.samples(GMSKVals, RX_BLOCK_SIZE);
      }
    } else if (m_modemState == STATE_DMR) {
      if (m_dmrEnable) {
        float DMRVals[RX_BLOCK_SIZE];
        m_rrcFilter.process(samples, DMRVals, RX_BLOCK_SIZE);

        dmrDMORX.samples(DMRVals, RX_BLOCK_SIZE);
      }
    } else if (m_modemState == STATE_YSF) {
      if (m_ysfEnable) {
        float YSFVals[RX_BLOCK_SIZE];
        m_rrcFilter.process(dcSamples, YSFVals, RX_BLOCK_SIZE);
        ysfRX.samples(YSFVals, RX_BLOCK_SIZE);
      }
    } else if (m_modemState == STATE_P25) {
      if (m_p25Enable) {
        float P25Vals[RX_BLOCK_SIZE];
        m_boxcarFilter.process(dcSamples, P25Vals, RX_BLOCK_SIZE);
        p25RX.samples(P25Vals, RX_BLOCK_SIZE);
      }
    } else if (m_modemState == STATE_NXDN) {
      if (m_nxdnEnable) {
        float NXDNValsTmp[RX_BLOCK_SIZE];
        m_nxdnFilter.process(dcSamples, NXDNValsTmp, RX_BLOCK_SIZE);
        float NXDNVals[RX_BLOCK_SIZE];
        m_nxdnISincFilter.process(NXDNValsTmp, NXDNVals, RX_BLOCK_SIZE);

        nxdnRX.samples(NXDNVals, RX_BLOCK_SIZE);
      }
    } else if (m_modemState == STATE_DSTARCAL) {
      float GMSKVals[RX_BLOCK_SIZE];
      m_gaussianFilter.process(samples, GMSKVals, RX_BLOCK_SIZE);

      calDStarRX.samples(GMSKVals, RX_BLOCK_SIZE);
    }
  }
}

void CIO::write(MMDVM_STATE mode, float* samples, uint16_t length)
{
  if (!m_started)
    return;

  if (m_lockout)
    return;

  // Switch the transmitter on if needed
  if (!m_tx) {
    m_tx = true;
    setPTTInt(m_pttInvert ? false : true);
  }

  float txLevel = 0.0F;
  switch (mode) {
    case STATE_DSTAR:
      txLevel = m_dstarTXLevel;
      break;
    case STATE_DMR:
      txLevel = m_dmrTXLevel;
      break;
    case STATE_YSF:
      txLevel = m_ysfTXLevel;
      break;
    case STATE_P25:
      txLevel = m_p25TXLevel;
      break;
    case STATE_NXDN:
      txLevel = m_nxdnTXLevel;
      break;
    case STATE_POCSAG:
      txLevel = m_pocsagTXLevel;
      break;
    default:
      txLevel = m_cwIdTXLevel;
      break;
  }

  for (uint16_t i = 0U; i < length; i++) {
    float res = (samples[i] * txLevel) + m_txDCOffset;

    // Detect DAC overflow
    if (res >= 1.0F || res <= -1.0F)
      m_dacOverflow++;

    m_txBuffer.put(res);
  }
}

uint16_t CIO::getSpace() const
{
  return m_txBuffer.getSpace();
}

void CIO::setDecode(bool dcd)
{
  if (dcd != m_dcd)
    setCOSInt(dcd ? true : false);

  m_dcd = dcd;
}

void CIO::setADCDetection(bool detect)
{
  m_detect = detect;
}

void CIO::setMode()
{
}

void CIO::setParameters(bool rxInvert, bool txInvert, bool pttInvert, float rxLevel, float cwIdTXLevel, float dstarTXLevel, float dmrTXLevel, float ysfTXLevel, float p25TXLevel, float nxdnTXLevel, float pocsagTXLevel, float txDCOffset, float rxDCOffset)
{
  m_pttInvert = pttInvert;

  m_rxLevel       = rxLevel;
  m_cwIdTXLevel   = cwIdTXLevel;
  m_dstarTXLevel  = dstarTXLevel;
  m_dmrTXLevel    = dmrTXLevel;
  m_ysfTXLevel    = ysfTXLevel;
  m_p25TXLevel    = p25TXLevel;
  m_nxdnTXLevel   = nxdnTXLevel;
  m_pocsagTXLevel = pocsagTXLevel;

  m_rxDCOffset   = DC_OFFSET + rxDCOffset;
  m_txDCOffset   = DC_OFFSET + txDCOffset;
  
  if (rxInvert)
    m_rxLevel = -m_rxLevel;
  
  if (txInvert) {
    m_dstarTXLevel  = -m_dstarTXLevel;
    m_dmrTXLevel    = -m_dmrTXLevel;
    m_ysfTXLevel    = -m_ysfTXLevel;
    m_p25TXLevel    = -m_p25TXLevel;
    m_nxdnTXLevel   = -m_nxdnTXLevel;
    m_pocsagTXLevel = -m_pocsagTXLevel;
  }
}

void CIO::getOverflow(bool& adcOverflow, bool& dacOverflow)
{
  adcOverflow = m_adcOverflow > 0U;
  dacOverflow = m_dacOverflow > 0U;

  m_adcOverflow = 0U;
  m_dacOverflow = 0U;
}

bool CIO::hasTXOverflow()
{
  return m_txBuffer.hasOverflowed();
}

bool CIO::hasRXOverflow()
{
  return m_rxBuffer.hasOverflowed();
}

void CIO::resetWatchdog()
{
  m_watchdog = 0U;
}

uint32_t CIO::getWatchdog()
{
  return m_watchdog;
}

bool CIO::hasLockout() const
{
  return m_lockout;
}

void CIO::readCallback(const float* input, unsigned int nSamples)
{
  for (unsigned int i = 0U; i < nSamples; i++)
    m_rxBuffer.put(input[i]);
}

void CIO::writeCallback(float* output, unsigned int& nSamples)
{
  for (unsigned int i = 0U; i < nSamples; i++)
    m_txBuffer.get(output[i]);
}

