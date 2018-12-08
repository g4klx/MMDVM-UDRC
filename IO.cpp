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

// Generated using [b, a] = butter(1, 0.0005) in MATLAB
static float DC_FILTER[] = {0.000784782F, 0.000000000F, 0.000784782F, 0.000000000F, 0.998430436F, 0.000000000F}; // {b0, 0, b1, b2, -a1, -a2}
const uint32_t DC_FILTER_STAGES = 1U; // One Biquad stage

// Generated using rcosdesign(0.2, 8, 10, 'sqrt') in MATLAB
static float RRC_0_2_FILTER[] = {0.0086673F,  0.0060427F,  0.0022279F, -0.0023804F, -0.0073244F, -0.0119938F, -0.0157781F, -0.0180059F,
								-0.0182806F, -0.0162664F, -0.0119327F, -0.0055239F,  0.0024110F,  0.0111087F,  0.0196234F,  0.0268563F,
								 0.0317698F,  0.0334788F,  0.0313120F,  0.0249947F,  0.0147404F,  0.0011902F, -0.0145573F, -0.0310068F,
								-0.0462661F, -0.0584429F, -0.0656148F, -0.0660421F, -0.0584124F, -0.0419630F, -0.0166326F,  0.0169988F,
								 0.0575579F,  0.1030305F,  0.1509445F,  0.1984313F,  0.2425001F,  0.2802820F,  0.3092746F,  0.3274941F,
								 0.3337199F,  0.3274941F,  0.3092746F,  0.2802820F,  0.2425001F,  0.1984313F,  0.1509445F,  0.1030305F,
								 0.0575579F,  0.0169988F, -0.0166326F, -0.0419630F, -0.0584124F, -0.0660421F, -0.0656148F, -0.0584429F,
								-0.0462661F, -0.0310068F, -0.0145573F,  0.0011902F,  0.0147404F,  0.0249947F,  0.0313120F,  0.0334788F,
								 0.0317698F,  0.0268563F,  0.0196234F,  0.0111087F,  0.0024110F, -0.0055239F, -0.0119327F, -0.0162664F,
								-0.0182806F, -0.0180059F, -0.0157781F, -0.0119938F, -0.0073244F, -0.0023804F,  0.0022279F,  0.0060427F,
								 0.0086673F,  0.0000000F};
const uint16_t RRC_0_2_FILTER_LEN = 82U;

// Generated using rcosdesign(0.2, 8, 20, 'sqrt') in MATLAB
static float NXDN_0_2_FILTER[] = {0.0061342F,  0.0053102F,  0.0042726F,  0.0030213F,  0.0015870F,  0.0000000F, -0.0016785F, -0.0034181F,
								 -0.0051881F, -0.0068972F, -0.0084841F, -0.0099185F, -0.0111393F, -0.0121158F, -0.0127262F, -0.0130314F,
								 -0.0129398F, -0.0124210F, -0.0115055F, -0.0101627F, -0.0084536F, -0.0063478F, -0.0039064F, -0.0012207F,
								  0.0017090F,  0.0047609F,  0.0078738F,  0.0109256F,  0.0138859F,  0.0166021F,  0.0189825F,  0.0209662F,
								  0.0224616F,  0.0233772F,  0.0236518F,  0.0232551F,  0.0221259F,  0.0202643F,  0.0176702F,  0.0143742F,
								  0.0104373F,  0.0058901F,  0.0008240F, -0.0046083F, -0.0103153F, -0.0161138F, -0.0219123F, -0.0274972F,
								 -0.0327158F, -0.0373852F, -0.0413221F, -0.0443739F, -0.0463881F, -0.0472121F, -0.0466933F, -0.0447401F,
								 -0.0412915F, -0.0362865F, -0.0296640F, -0.0214850F, -0.0117496F, -0.0005493F,  0.0120243F,  0.0258187F,
								  0.0406812F,  0.0564592F,  0.0728782F,  0.0897244F,  0.1067537F,  0.1236915F,  0.1403241F,  0.1563158F,
								  0.1714835F,  0.1855220F,  0.1981872F,  0.2093265F,  0.2186956F,  0.2261422F,  0.2315744F,  0.2348704F,
								  0.2359691F,  0.2348704F,  0.2315744F,  0.2261422F,  0.2186956F,  0.2093265F,  0.1981872F,  0.1855220F,
								  0.1714835F,  0.1563158F,  0.1403241F,  0.1236915F,  0.1067537F,  0.0897244F,  0.0728782F,  0.0564592F,
								  0.0406812F,  0.0258187F,  0.0120243F, -0.0005493F, -0.0117496F, -0.0214850F, -0.0296640F, -0.0362865F,
								 -0.0412915F, -0.0447401F, -0.0466933F, -0.0472121F, -0.0463881F, -0.0443739F, -0.0413221F, -0.0373852F,
								 -0.0327158F, -0.0274972F, -0.0219123F, -0.0161138F, -0.0103153F, -0.0046083F,  0.0008240F,  0.0058901F,
								  0.0104373F,  0.0143742F,  0.0176702F,  0.0202643F,  0.0221259F,  0.0232551F,  0.0236518F,  0.0233772F,
								  0.0224616F,  0.0209662F,  0.0189825F,  0.0166021F,  0.0138859F,  0.0109256F,  0.0078738F,  0.0047609F,
								  0.0017090F, -0.0012207F, -0.0039064F, -0.0063478F, -0.0084536F, -0.0101627F, -0.0115055F, -0.0124210F,
								 -0.0129398F, -0.0130314F, -0.0127262F, -0.0121158F, -0.0111393F, -0.0099185F, -0.0084841F, -0.0068972F,
								 -0.0051881F, -0.0034181F, -0.0016785F,  0.0000000F,  0.0015870F,  0.0030213F,  0.0042726F,  0.0053102F,
								  0.0061342F,  0.0000000F};
const uint16_t NXDN_0_2_FILTER_LEN = 162U;

static float NXDN_ISINC_FILTER[] = {0.2324290F, -0.0406812F, -0.0566424F, -0.0796838F, -0.1037324F, -0.1222572F, -0.1290933F, -0.1195715F,
								   -0.0911893F, -0.0444960F,  0.0171209F,  0.0874966F,  0.1586962F,  0.2220527F,  0.2695395F,  0.2949614F,
									0.2949614F,  0.2695395F,  0.2220527F,  0.1586962F,  0.0874966F,  0.0171209F, -0.0444960F, -0.0911893F,
								   -0.1195715F, -0.1290933F, -0.1222572F, -0.1037324F, -0.0796838F, -0.0566424F, -0.0406812F,  0.2324290F};
const uint16_t NXDN_ISINC_FILTER_LEN = 32U;

#if !defined (DSTARBOXCAR)
// Generated using gaussfir(0.5, 4, 10) in MATLAB
static float GAUSSIAN_0_5_FILTER[] = {0.0000305F, 0.0001221F, 0.0004578F, 0.0015870F, 0.0046083F, 0.0115970F, 0.0253914F, 0.0481887F,
									  0.0793176F, 0.1132237F, 0.1402020F, 0.1505478F, 0.1402020F, 0.1132237F, 0.0793176F, 0.0481887F,
									  0.0253914F, 0.0115970F, 0.0046083F, 0.0015870F, 0.0004578F, 0.0001221F, 0.0000305F, 0.0000000F};
const uint16_t GAUSSIAN_0_5_FILTER_LEN = 24U;
#endif
// One symbol boxcar filter
static float BOXCAR_FILTER[] = {0.1831111F, 0.1831111F, 0.1831111F, 0.1831111F, 0.1831111F, 0.1831111F, 0.1831111F, 0.1831111F, 0.1831111F,
								0.1831111F, 0.0000000F, 0.0000000F};
const uint16_t BOXCAR_FILTER_LEN = 12U;

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
    if (m_watchdog >= 96000U) {
      if (m_modemState == STATE_DSTAR || m_modemState == STATE_DMR || m_modemState == STATE_YSF || m_modemState == STATE_P25 || m_modemState == STATE_NXDN || m_modemState == STATE_POCSAG) {
        m_modemState = STATE_IDLE;
        setMode();
      }

      m_watchdog = 0U;
    }

    if (m_ledCount >= 48000U) {
      m_ledCount = 0U;
      m_ledValue = !m_ledValue;
      setLEDInt(m_ledValue);
    }
  } else {
    if (m_ledCount >= 480000U) {
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

void CIO::writeCallback(float* output, int& nSamples)
{
  for (int i = 0U; i < nSamples; i++)
    m_txBuffer.get(output[i]);
}

