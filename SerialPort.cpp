/*
 *   Copyright (C) 2013,2015,2016,2017,2018 by Jonathan Naylor G4KLX
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

#include "SerialPort.h"

const uint8_t MMDVM_FRAME_START  = 0xE0U;

const uint8_t MMDVM_GET_VERSION  = 0x00U;
const uint8_t MMDVM_GET_STATUS   = 0x01U;
const uint8_t MMDVM_SET_CONFIG   = 0x02U;
const uint8_t MMDVM_SET_MODE     = 0x03U;
const uint8_t MMDVM_SET_FREQ     = 0x04U;

const uint8_t MMDVM_CAL_DATA     = 0x08U;
const uint8_t MMDVM_RSSI_DATA    = 0x09U;

const uint8_t MMDVM_SEND_CWID    = 0x0AU;

const uint8_t MMDVM_DSTAR_HEADER = 0x10U;
const uint8_t MMDVM_DSTAR_DATA   = 0x11U;
const uint8_t MMDVM_DSTAR_LOST   = 0x12U;
const uint8_t MMDVM_DSTAR_EOT    = 0x13U;

const uint8_t MMDVM_DMR_DATA1    = 0x18U;
const uint8_t MMDVM_DMR_LOST1    = 0x19U;
const uint8_t MMDVM_DMR_DATA2    = 0x1AU;
const uint8_t MMDVM_DMR_LOST2    = 0x1BU;
const uint8_t MMDVM_DMR_SHORTLC  = 0x1CU;
const uint8_t MMDVM_DMR_START    = 0x1DU;
const uint8_t MMDVM_DMR_ABORT    = 0x1EU;

const uint8_t MMDVM_YSF_DATA     = 0x20U;
const uint8_t MMDVM_YSF_LOST     = 0x21U;

const uint8_t MMDVM_P25_HDR      = 0x30U;
const uint8_t MMDVM_P25_LDU      = 0x31U;
const uint8_t MMDVM_P25_LOST     = 0x32U;

const uint8_t MMDVM_NXDN_DATA    = 0x40U;
const uint8_t MMDVM_NXDN_LOST    = 0x41U;

const uint8_t MMDVM_POCSAG_DATA  = 0x50U;

const uint8_t MMDVM_ACK          = 0x70U;
const uint8_t MMDVM_NAK          = 0x7FU;

const uint8_t MMDVM_SERIAL       = 0x80U;

const uint8_t MMDVM_TRANSPARENT  = 0x90U;

const uint8_t MMDVM_DEBUG1       = 0xF1U;
const uint8_t MMDVM_DEBUG2       = 0xF2U;
const uint8_t MMDVM_DEBUG3       = 0xF3U;
const uint8_t MMDVM_DEBUG4       = 0xF4U;
const uint8_t MMDVM_DEBUG5       = 0xF5U;

#define DESCRIPTION              "MMDVM 20180613 (D-Star/DMR/System Fusion/P25/NXDN/POCSAG)"

#define concat(a, b, c) a " (Build: " b " " c ")"
const char HARDWARE[] = concat(DESCRIPTION, __TIME__, __DATE__);

const uint8_t PROTOCOL_VERSION   = 1U;


CSerialPort::CSerialPort() :
m_serialPort(),
m_buffer(),
m_ptr(0U),
m_len(0U),
m_debug(false),
m_repeat()
{
}

void CSerialPort::sendACK()
{
  uint8_t reply[4U];

  reply[0U] = MMDVM_FRAME_START;
  reply[1U] = 4U;
  reply[2U] = MMDVM_ACK;
  reply[3U] = m_buffer[2U];

  m_serialPort.write(reply, 4);
}

void CSerialPort::sendNAK(uint8_t err)
{
  uint8_t reply[5U];

  reply[0U] = MMDVM_FRAME_START;
  reply[1U] = 5U;
  reply[2U] = MMDVM_NAK;
  reply[3U] = m_buffer[2U];
  reply[4U] = err;

  m_serialPort.write(reply, 5);
}

void CSerialPort::getStatus()
{
  io.resetWatchdog();

  uint8_t reply[15U];

  // Send all sorts of interesting internal values
  reply[0U]  = MMDVM_FRAME_START;
  reply[1U]  = 11U;
  reply[2U]  = MMDVM_GET_STATUS;

  reply[3U]  = 0x00U;
  if (m_dstarEnable)
    reply[3U] |= 0x01U;
  if (m_dmrEnable)
    reply[3U] |= 0x02U;
  if (m_ysfEnable)
    reply[3U] |= 0x04U;
  if (m_p25Enable)
    reply[3U] |= 0x08U;
  if (m_nxdnEnable)
    reply[3U] |= 0x10U;

  reply[4U]  = uint8_t(m_modemState);

  reply[5U]  = m_tx  ? 0x01U : 0x00U;

  bool adcOverflow;
  bool dacOverflow;
  io.getOverflow(adcOverflow, dacOverflow);

  if (adcOverflow)
    reply[5U] |= 0x02U;

  if (io.hasRXOverflow())
    reply[5U] |= 0x04U;

  if (io.hasTXOverflow())
    reply[5U] |= 0x08U;

  if (io.hasLockout())
    reply[5U] |= 0x10U;

  if (dacOverflow)
    reply[5U] |= 0x20U;
    
  reply[5U] |= m_dcd ? 0x40U : 0x00U;

  if (m_dstarEnable)
    reply[6U] = dstarTX.getSpace();
  else
    reply[6U] = 0U;

  if (m_dmrEnable) {
    reply[7U] = 10U;
    reply[8U] = dmrDMOTX.getSpace();
  } else {
    reply[7U] = 0U;
    reply[8U] = 0U;
  }

  if (m_ysfEnable)
    reply[9U] = ysfTX.getSpace();
  else
    reply[9U] = 0U;

  if (m_p25Enable)
    reply[10U] = p25TX.getSpace();
  else
    reply[10U] = 0U;

  if (m_nxdnEnable)
    reply[11U] = nxdnTX.getSpace();
  else
    reply[11U] = 0U;

  m_serialPort.write(reply, 12);
}

void CSerialPort::getVersion()
{
  uint8_t reply[100U];

  reply[0U] = MMDVM_FRAME_START;
  reply[1U] = 0U;
  reply[2U] = MMDVM_GET_VERSION;

  reply[3U] = PROTOCOL_VERSION;

  uint8_t count = 4U;
  for (uint8_t i = 0U; HARDWARE[i] != 0x00U; i++, count++)
    reply[count] = HARDWARE[i];

  reply[1U] = count;

  m_serialPort.write(reply, count);
}

uint8_t CSerialPort::setConfig(const uint8_t* data, uint8_t length)
{
  if (length < 18U)
    return 4U;

  bool rxInvert  = (data[0U] & 0x01U) == 0x01U;
  bool txInvert  = (data[0U] & 0x02U) == 0x02U;
  bool pttInvert = (data[0U] & 0x04U) == 0x04U;
  bool ysfLoDev  = (data[0U] & 0x08U) == 0x08U;
  bool simplex   = (data[0U] & 0x80U) == 0x80U;

  m_debug = (data[0U] & 0x10U) == 0x10U;

  bool dstarEnable  = (data[1U] & 0x01U) == 0x01U;
  bool dmrEnable    = (data[1U] & 0x02U) == 0x02U;
  bool ysfEnable    = (data[1U] & 0x04U) == 0x04U;
  bool p25Enable    = (data[1U] & 0x08U) == 0x08U;
  bool nxdnEnable   = (data[1U] & 0x10U) == 0x10U;
  bool pocsagEnable = (data[1U] & 0x20U) == 0x20U;

  uint8_t txDelay = data[2U];
  if (txDelay > 50U)
    return 4U;

  MMDVM_STATE modemState = MMDVM_STATE(data[3U]);

  if (modemState != STATE_IDLE && modemState != STATE_DSTAR && modemState != STATE_DMR && modemState != STATE_YSF && modemState != STATE_P25 && modemState != STATE_NXDN && modemState != STATE_POCSAG && modemState != STATE_DSTARCAL && modemState != STATE_DMRCAL && modemState != STATE_RSSICAL && modemState != STATE_LFCAL && modemState != STATE_P25CAL1K && modemState != STATE_DMRDMO1K && modemState != STATE_NXDNCAL1K)
    return 4U;
  if (modemState == STATE_DSTAR && !dstarEnable)
    return 4U;
  if (modemState == STATE_DMR && !dmrEnable)
    return 4U;
  if (modemState == STATE_YSF && !ysfEnable)
    return 4U;
  if (modemState == STATE_P25 && !p25Enable)
    return 4U;
  if (modemState == STATE_NXDN && !nxdnEnable)
    return 4U;
  if (modemState == STATE_POCSAG && !pocsagEnable)
    return 4U;

  float rxLevel = float(data[4U]) / 255.0F;

  uint8_t colorCode = data[6U];
  if (colorCode > 15U)
    return 4U;

  float cwIdTXLevel  = float(data[5U]) / 255.0F;
  float dstarTXLevel = float(data[9U]) / 255.0F;
  float dmrTXLevel   = float(data[10U]) / 255.0F;
  float ysfTXLevel   = float(data[11U]) / 255.0F;
  float p25TXLevel   = float(data[12U]) / 255.0F;

  float txDCOffset = float((data[13U]) - 128) / 128.0F;
  float rxDCOffset = float((data[14U]) - 128) / 128.0F;

  float nxdnTXLevel = float(data[15U]) / 255.0F;

  uint8_t ysfTXHang = data[16U];

  float pocsagTXLevel = float(data[17U]) / 255.0F;

  m_modemState  = modemState;

  m_dstarEnable  = dstarEnable;
  m_dmrEnable    = dmrEnable;
  m_ysfEnable    = ysfEnable;
  m_p25Enable    = p25Enable;
  m_nxdnEnable   = nxdnEnable;
  m_pocsagEnable = pocsagEnable;
  m_duplex       = !simplex;

  dstarTX.setTXDelay(txDelay);
  ysfTX.setTXDelay(txDelay);
  p25TX.setTXDelay(txDelay);
  dmrDMOTX.setTXDelay(txDelay);
  nxdnTX.setTXDelay(txDelay);
  pocsagTX.setTXDelay(txDelay);

  dmrDMORX.setColorCode(colorCode);

  ysfTX.setParams(ysfLoDev, ysfTXHang);

  io.setParameters(rxInvert, txInvert, pttInvert, rxLevel, cwIdTXLevel, dstarTXLevel, dmrTXLevel, ysfTXLevel, p25TXLevel, nxdnTXLevel, pocsagTXLevel, txDCOffset, rxDCOffset);

  io.start();

  return 0U;
}

uint8_t CSerialPort::setMode(const uint8_t* data, uint8_t length)
{
  if (length < 1U)
    return 4U;

  MMDVM_STATE modemState = MMDVM_STATE(data[0U]);

  if (modemState == m_modemState)
    return 0U;

  if (modemState != STATE_IDLE && modemState != STATE_DSTAR && modemState != STATE_DMR && modemState != STATE_YSF && modemState != STATE_P25 && modemState != STATE_NXDN && modemState != STATE_POCSAG && modemState != STATE_DSTARCAL && modemState != STATE_DMRCAL && modemState != STATE_RSSICAL && modemState != STATE_LFCAL && modemState != STATE_P25CAL1K && modemState != STATE_DMRDMO1K && modemState != STATE_NXDNCAL1K)
    return 4U;
  if (modemState == STATE_DSTAR && !m_dstarEnable)
    return 4U;
  if (modemState == STATE_DMR && !m_dmrEnable)
    return 4U;
  if (modemState == STATE_YSF && !m_ysfEnable)
    return 4U;
  if (modemState == STATE_P25 && !m_p25Enable)
    return 4U;
  if (modemState == STATE_NXDN && !m_nxdnEnable)
    return 4U;
  if (modemState == STATE_POCSAG && !m_pocsagEnable)
    return 4U;

  setMode(modemState);

  return 0U;
}

void CSerialPort::setMode(MMDVM_STATE modemState)
{
  switch (modemState) {
    case STATE_DMR:
      DEBUG1("Mode set to DMR");
      dstarRX.reset();
      ysfRX.reset();
      p25RX.reset();
      nxdnRX.reset();
      cwIdTX.reset();
      break;
    case STATE_DSTAR:
      DEBUG1("Mode set to D-Star");
      dmrDMORX.reset();
      ysfRX.reset();
      p25RX.reset();
      nxdnRX.reset();
      cwIdTX.reset();
      break;
    case STATE_YSF:
      DEBUG1("Mode set to System Fusion");
      dmrDMORX.reset();
      dstarRX.reset();
      p25RX.reset();
      nxdnRX.reset();
      cwIdTX.reset();
      break;
    case STATE_P25:
      DEBUG1("Mode set to P25");
      dmrDMORX.reset();
      dstarRX.reset();
      ysfRX.reset();
      nxdnRX.reset();
      cwIdTX.reset();
      break;
    case STATE_NXDN:
      DEBUG1("Mode set to NXDN");
      dmrDMORX.reset();
      dstarRX.reset();
      ysfRX.reset();
      p25RX.reset();
      cwIdTX.reset();
      break;
    case STATE_POCSAG:
      DEBUG1("Mode set to POCSAG");
      dmrDMORX.reset();
      dstarRX.reset();
      ysfRX.reset();
      p25RX.reset();
      nxdnRX.reset();
      cwIdTX.reset();
      break;
    case STATE_DSTARCAL:
      DEBUG1("Mode set to D-Star Calibrate");
      dmrDMORX.reset();
      dstarRX.reset();
      ysfRX.reset();
      p25RX.reset();
      nxdnRX.reset();
      cwIdTX.reset();
      break;
    case STATE_DMRCAL:
      DEBUG1("Mode set to DMR Calibrate");
      dmrDMORX.reset();
      dstarRX.reset();
      ysfRX.reset();
      p25RX.reset();
      nxdnRX.reset();
      cwIdTX.reset();
      break;
    case STATE_RSSICAL:
      DEBUG1("Mode set to RSSI Calibrate");
      dmrDMORX.reset();
      dstarRX.reset();
      ysfRX.reset();
      p25RX.reset();
      nxdnRX.reset();
      cwIdTX.reset();
      break;
    case STATE_LFCAL:
      DEBUG1("Mode set to 80 Hz Calibrate");
      dmrDMORX.reset();
      dstarRX.reset();
      ysfRX.reset();
      p25RX.reset();
      nxdnRX.reset();
      cwIdTX.reset();
      break;
    case STATE_P25CAL1K:
      DEBUG1("Mode set to P25 1011 Hz Calibrate");
      dmrDMORX.reset();
      dstarRX.reset();
      ysfRX.reset();
      p25RX.reset();
      nxdnRX.reset();
      cwIdTX.reset();
      break;
    case STATE_DMRDMO1K:
      DEBUG1("Mode set to DMR MS 1031 Hz Calibrate");
      dmrDMORX.reset();
      dstarRX.reset();
      ysfRX.reset();
      p25RX.reset();
      nxdnRX.reset();
      cwIdTX.reset();
      break;
    case STATE_NXDNCAL1K:
      DEBUG1("Mode set to NXDN 1031 Hz Calibrate");
      dmrDMORX.reset();
      dstarRX.reset();
      ysfRX.reset();
      p25RX.reset();
      nxdnRX.reset();
      cwIdTX.reset();
      break;
    default:
      DEBUG1("Mode set to Idle");
      // STATE_IDLE
      break;
  }

  m_modemState = modemState;

  io.setMode();
}

bool CSerialPort::open(const std::string& port, const std::string& path)
{
  return m_serialPort.open(port, SERIAL_115200, true, path);
}

void CSerialPort::process()
{
  uint8_t buffer[100U];
  int len = m_serialPort.read(buffer, 100U);
  if (len <= 0)
    return;

  for (int i = 0; i < len; i++) {
    uint8_t c = buffer[i];

    if (m_ptr == 0U) {
      if (c == MMDVM_FRAME_START) {
        // Handle the frame start correctly
        m_buffer[0U] = c;
        m_ptr = 1U;
        m_len = 0U;
      } else {
        m_ptr = 0U;
        m_len = 0U;
      }
    } else if (m_ptr == 1U) {
      // Handle the frame length
      m_len = m_buffer[m_ptr] = c;
      m_ptr = 2U;
    } else {
      // Any other bytes are added to the buffer
      m_buffer[m_ptr] = c;
      m_ptr++;

      // The full packet has been received, process it
      if (m_ptr == m_len) {
        uint8_t err = 2U;

        switch (m_buffer[2U]) {
          case MMDVM_GET_STATUS:
            getStatus();
            break;

          case MMDVM_GET_VERSION:
            getVersion();
            break;

          case MMDVM_SET_CONFIG:
            err = setConfig(m_buffer + 3U, m_len - 3U);
            if (err == 0U)
              sendACK();
            else
              sendNAK(err);
            break;

          case MMDVM_SET_MODE:
            err = setMode(m_buffer + 3U, m_len - 3U);
            if (err == 0U)
              sendACK();
            else
              sendNAK(err);
            break;

          case MMDVM_SET_FREQ:
            sendACK();
            break;

          case MMDVM_CAL_DATA:
            if (m_modemState == STATE_DSTARCAL)
              err = calDStarTX.write(m_buffer + 3U, m_len - 3U);
            if (m_modemState == STATE_DMRCAL || m_modemState == STATE_LFCAL || m_modemState == STATE_DMRDMO1K)
              err = calDMR.write(m_buffer + 3U, m_len - 3U);
            if (m_modemState == STATE_P25CAL1K)
              err = calP25.write(m_buffer + 3U, m_len - 3U);
            if (m_modemState == STATE_NXDNCAL1K)
              err = calNXDN.write(m_buffer + 3U, m_len - 3U);
            if (err == 0U) {
              sendACK();
            } else {
              DEBUG2("Received invalid calibration data", err);
              sendNAK(err);
            }
            break;

          case MMDVM_SEND_CWID:
            err = 5U;
            if (m_modemState == STATE_IDLE)
              err = cwIdTX.write(m_buffer + 3U, m_len - 3U);
            if (err != 0U) {
              DEBUG2("Invalid CW Id data", err);
              sendNAK(err);
            }
            break;

          case MMDVM_DSTAR_HEADER:
            if (m_dstarEnable) {
              if (m_modemState == STATE_IDLE || m_modemState == STATE_DSTAR)
                err = dstarTX.writeHeader(m_buffer + 3U, m_len - 3U);
            }
            if (err == 0U) {
              if (m_modemState == STATE_IDLE)
                setMode(STATE_DSTAR);
            } else {
              DEBUG2("Received invalid D-Star header", err);
              sendNAK(err);
            }
            break;

          case MMDVM_DSTAR_DATA:
            if (m_dstarEnable) {
              if (m_modemState == STATE_IDLE || m_modemState == STATE_DSTAR)
                err = dstarTX.writeData(m_buffer + 3U, m_len - 3U);
            }
            if (err == 0U) {
              if (m_modemState == STATE_IDLE)
                setMode(STATE_DSTAR);
            } else {
              DEBUG2("Received invalid D-Star data", err);
              sendNAK(err);
            }
            break;

          case MMDVM_DSTAR_EOT:
            if (m_dstarEnable) {
              if (m_modemState == STATE_IDLE || m_modemState == STATE_DSTAR)
                err = dstarTX.writeEOT();
            }
            if (err == 0U) {
              if (m_modemState == STATE_IDLE)
                setMode(STATE_DSTAR);
            } else {
              DEBUG2("Received invalid D-Star EOT", err);
              sendNAK(err);
            }
            break;

          case MMDVM_DMR_DATA2:
            if (m_dmrEnable) {
              if (m_modemState == STATE_IDLE || m_modemState == STATE_DMR)
                err = dmrDMOTX.writeData(m_buffer + 3U, m_len - 3U);
            }
            if (err == 0U) {
              if (m_modemState == STATE_IDLE)
                setMode(STATE_DMR);
            } else {
              DEBUG2("Received invalid DMR data", err);
              sendNAK(err);
            }
            break;

          case MMDVM_YSF_DATA:
            if (m_ysfEnable) {
              if (m_modemState == STATE_IDLE || m_modemState == STATE_YSF)
                err = ysfTX.writeData(m_buffer + 3U, m_len - 3U);
            }
            if (err == 0U) {
              if (m_modemState == STATE_IDLE)
                setMode(STATE_YSF);
            } else {
              DEBUG2("Received invalid System Fusion data", err);
              sendNAK(err);
            }
            break;

          case MMDVM_P25_HDR:
            if (m_p25Enable) {
              if (m_modemState == STATE_IDLE || m_modemState == STATE_P25)
                err = p25TX.writeData(m_buffer + 3U, m_len - 3U);
            }
            if (err == 0U) {
              if (m_modemState == STATE_IDLE)
                setMode(STATE_P25);
            } else {
              DEBUG2("Received invalid P25 header", err);
              sendNAK(err);
            }
            break;

          case MMDVM_P25_LDU:
            if (m_p25Enable) {
              if (m_modemState == STATE_IDLE || m_modemState == STATE_P25)
                err = p25TX.writeData(m_buffer + 3U, m_len - 3U);
            }
            if (err == 0U) {
              if (m_modemState == STATE_IDLE)
                setMode(STATE_P25);
            } else {
              DEBUG2("Received invalid P25 LDU", err);
              sendNAK(err);
            }
            break;

          case MMDVM_NXDN_DATA:
            if (m_nxdnEnable) {
              if (m_modemState == STATE_IDLE || m_modemState == STATE_NXDN)
                err = nxdnTX.writeData(m_buffer + 3U, m_len - 3U);
            }
            if (err == 0U) {
              if (m_modemState == STATE_IDLE)
                setMode(STATE_NXDN);
            } else {
              DEBUG2("Received invalid NXDN data", err);
              sendNAK(err);
            }
            break;

          case MMDVM_POCSAG_DATA:
            if (m_pocsagEnable) {
              if (m_modemState == STATE_IDLE || m_modemState == STATE_POCSAG)
                err = pocsagTX.writeData(m_buffer + 3U, m_len - 3U);
            }
            if (err == 0U) {
              if (m_modemState == STATE_IDLE)
                setMode(STATE_POCSAG);
            } else {
              DEBUG2("Received invalid POCSAG data", err);
              sendNAK(err);
            }
            break;

          case MMDVM_TRANSPARENT:
            // Do nothing on the MMDVM.
            break;

          default:
            // Handle this, send a NAK back
            sendNAK(1U);
            break;
        }

        m_ptr = 0U;
        m_len = 0U;
      }
    }
  }

  if (io.getWatchdog() >= 48000U) {
    m_ptr = 0U;
    m_len = 0U;
  }
}

void CSerialPort::writeDStarHeader(const uint8_t* header, uint8_t length)
{
  if (m_modemState != STATE_DSTAR && m_modemState != STATE_IDLE)
    return;

  if (!m_dstarEnable)
    return;

  uint8_t reply[50U];
  reply[0U] = MMDVM_FRAME_START;
  reply[1U] = 0U;
  reply[2U] = MMDVM_DSTAR_HEADER;

  uint8_t count = 3U;
  for (uint8_t i = 0U; i < length; i++, count++)
    reply[count] = header[i];

  reply[1U] = count;

  m_serialPort.write(reply, count);
}

void CSerialPort::writeDStarData(const uint8_t* data, uint8_t length)
{
  if (m_modemState != STATE_DSTAR && m_modemState != STATE_IDLE)
    return;

  if (!m_dstarEnable)
    return;

  uint8_t reply[20U];

  reply[0U] = MMDVM_FRAME_START;
  reply[1U] = 0U;
  reply[2U] = MMDVM_DSTAR_DATA;

  uint8_t count = 3U;
  for (uint8_t i = 0U; i < length; i++, count++)
    reply[count] = data[i];

  reply[1U] = count;

  m_serialPort.write(reply, count);
}

void CSerialPort::writeDStarLost()
{
  if (m_modemState != STATE_DSTAR && m_modemState != STATE_IDLE)
    return;

  if (!m_dstarEnable)
    return;

  uint8_t reply[3U];

  reply[0U] = MMDVM_FRAME_START;
  reply[1U] = 3U;
  reply[2U] = MMDVM_DSTAR_LOST;

  m_serialPort.write(reply, 3);
}

void CSerialPort::writeDStarEOT()
{
  if (m_modemState != STATE_DSTAR && m_modemState != STATE_IDLE)
    return;

  if (!m_dstarEnable)
    return;

  uint8_t reply[3U];

  reply[0U] = MMDVM_FRAME_START;
  reply[1U] = 3U;
  reply[2U] = MMDVM_DSTAR_EOT;

  m_serialPort.write(reply, 3);
}

void CSerialPort::writeDMRData(bool slot, const uint8_t* data, uint8_t length)
{
  if (m_modemState != STATE_DMR && m_modemState != STATE_IDLE)
    return;

  if (!m_dmrEnable)
    return;

  uint8_t reply[40U];

  reply[0U] = MMDVM_FRAME_START;
  reply[1U] = 0U;
  reply[2U] = slot ? MMDVM_DMR_DATA2 : MMDVM_DMR_DATA1;

  uint8_t count = 3U;
  for (uint8_t i = 0U; i < length; i++, count++)
    reply[count] = data[i];

  reply[1U] = count;

  m_serialPort.write(reply, count);
}

void CSerialPort::writeDMRLost(bool slot)
{
  if (m_modemState != STATE_DMR && m_modemState != STATE_IDLE)
    return;

  if (!m_dmrEnable)
    return;

  uint8_t reply[3U];

  reply[0U] = MMDVM_FRAME_START;
  reply[1U] = 3U;
  reply[2U] = slot ? MMDVM_DMR_LOST2 : MMDVM_DMR_LOST1;

  m_serialPort.write(reply, 3);
}

void CSerialPort::writeYSFData(const uint8_t* data, uint8_t length)
{
  if (m_modemState != STATE_YSF && m_modemState != STATE_IDLE)
    return;

  if (!m_ysfEnable)
    return;

  uint8_t reply[130U];

  reply[0U] = MMDVM_FRAME_START;
  reply[1U] = 0U;
  reply[2U] = MMDVM_YSF_DATA;

  uint8_t count = 3U;
  for (uint8_t i = 0U; i < length; i++, count++)
    reply[count] = data[i];

  reply[1U] = count;

  m_serialPort.write(reply, count);
}

void CSerialPort::writeYSFLost()
{
  if (m_modemState != STATE_YSF && m_modemState != STATE_IDLE)
    return;

  if (!m_ysfEnable)
    return;

  uint8_t reply[3U];

  reply[0U] = MMDVM_FRAME_START;
  reply[1U] = 3U;
  reply[2U] = MMDVM_YSF_LOST;

  m_serialPort.write(reply, 3);
}

void CSerialPort::writeP25Hdr(const uint8_t* data, uint8_t length)
{
  if (m_modemState != STATE_P25 && m_modemState != STATE_IDLE)
    return;

  if (!m_p25Enable)
    return;

  uint8_t reply[120U];

  reply[0U] = MMDVM_FRAME_START;
  reply[1U] = 0U;
  reply[2U] = MMDVM_P25_HDR;

  uint8_t count = 3U;
  for (uint8_t i = 0U; i < length; i++, count++)
    reply[count] = data[i];

  reply[1U] = count;

  m_serialPort.write(reply, count);
}

void CSerialPort::writeP25Ldu(const uint8_t* data, uint8_t length)
{
  if (m_modemState != STATE_P25 && m_modemState != STATE_IDLE)
    return;

  if (!m_p25Enable)
    return;

  uint8_t reply[250U];

  reply[0U] = MMDVM_FRAME_START;
  reply[1U] = 0U;
  reply[2U] = MMDVM_P25_LDU;

  uint8_t count = 3U;
  for (uint8_t i = 0U; i < length; i++, count++)
    reply[count] = data[i];

  reply[1U] = count;

  m_serialPort.write(reply, count);
}

void CSerialPort::writeP25Lost()
{
  if (m_modemState != STATE_P25 && m_modemState != STATE_IDLE)
    return;

  if (!m_p25Enable)
    return;

  uint8_t reply[3U];

  reply[0U] = MMDVM_FRAME_START;
  reply[1U] = 3U;
  reply[2U] = MMDVM_P25_LOST;

  m_serialPort.write(reply, 3);
}

void CSerialPort::writeNXDNData(const uint8_t* data, uint8_t length)
{
  if (m_modemState != STATE_NXDN && m_modemState != STATE_IDLE)
    return;

  if (!m_nxdnEnable)
    return;

  uint8_t reply[130U];

  reply[0U] = MMDVM_FRAME_START;
  reply[1U] = 0U;
  reply[2U] = MMDVM_NXDN_DATA;

  uint8_t count = 3U;
  for (uint8_t i = 0U; i < length; i++, count++)
    reply[count] = data[i];

  reply[1U] = count;

  m_serialPort.write(reply, count);
}

void CSerialPort::writeNXDNLost()
{
  if (m_modemState != STATE_NXDN && m_modemState != STATE_IDLE)
    return;

  if (!m_nxdnEnable)
    return;

  uint8_t reply[3U];

  reply[0U] = MMDVM_FRAME_START;
  reply[1U] = 3U;
  reply[2U] = MMDVM_NXDN_LOST;

  m_serialPort.write(reply, 3);
}

void CSerialPort::writeCalData(const uint8_t* data, uint8_t length)
{
  if (m_modemState != STATE_DSTARCAL)
    return;

  uint8_t reply[130U];

  reply[0U] = MMDVM_FRAME_START;
  reply[1U] = 0U;
  reply[2U] = MMDVM_CAL_DATA;

  uint8_t count = 3U;
  for (uint8_t i = 0U; i < length; i++, count++)
    reply[count] = data[i];

  reply[1U] = count;

  m_serialPort.write(reply, count);
}

void CSerialPort::writeRSSIData(const uint8_t* data, uint8_t length)
{
  if (m_modemState != STATE_RSSICAL)
    return;

  uint8_t reply[30U];

  reply[0U] = MMDVM_FRAME_START;
  reply[1U] = 0U;
  reply[2U] = MMDVM_RSSI_DATA;

  uint8_t count = 3U;
  for (uint8_t i = 0U; i < length; i++, count++)
    reply[count] = data[i];

  reply[1U] = count;

  m_serialPort.write(reply, count);
}

void CSerialPort::writeDebug(const char* text)
{
  if (!m_debug)
    return;

  uint8_t reply[130U];

  reply[0U] = MMDVM_FRAME_START;
  reply[1U] = 0U;
  reply[2U] = MMDVM_DEBUG1;

  uint8_t count = 3U;
  for (uint8_t i = 0U; text[i] != '\0'; i++, count++)
    reply[count] = text[i];

  reply[1U] = count;

  m_serialPort.write(reply, count);
}

void CSerialPort::writeDebug(const char* text, int16_t n1)
{
  if (!m_debug)
    return;

  uint8_t reply[130U];

  reply[0U] = MMDVM_FRAME_START;
  reply[1U] = 0U;
  reply[2U] = MMDVM_DEBUG2;

  uint8_t count = 3U;
  for (uint8_t i = 0U; text[i] != '\0'; i++, count++)
    reply[count] = text[i];

  reply[count++] = (n1 >> 8) & 0xFF;
  reply[count++] = (n1 >> 0) & 0xFF;

  reply[1U] = count;

  m_serialPort.write(reply, count);
}

void CSerialPort::writeDebug(const char* text, int16_t n1, int16_t n2)
{
  if (!m_debug)
    return;

  uint8_t reply[130U];

  reply[0U] = MMDVM_FRAME_START;
  reply[1U] = 0U;
  reply[2U] = MMDVM_DEBUG3;

  uint8_t count = 3U;
  for (uint8_t i = 0U; text[i] != '\0'; i++, count++)
    reply[count] = text[i];

  reply[count++] = (n1 >> 8) & 0xFF;
  reply[count++] = (n1 >> 0) & 0xFF;

  reply[count++] = (n2 >> 8) & 0xFF;
  reply[count++] = (n2 >> 0) & 0xFF;

  reply[1U] = count;

  m_serialPort.write(reply, count);
}

void CSerialPort::writeDebug(const char* text, int16_t n1, int16_t n2, int16_t n3)
{
  if (!m_debug)
    return;

  uint8_t reply[130U];

  reply[0U] = MMDVM_FRAME_START;
  reply[1U] = 0U;
  reply[2U] = MMDVM_DEBUG4;

  uint8_t count = 3U;
  for (uint8_t i = 0U; text[i] != '\0'; i++, count++)
    reply[count] = text[i];

  reply[count++] = (n1 >> 8) & 0xFF;
  reply[count++] = (n1 >> 0) & 0xFF;

  reply[count++] = (n2 >> 8) & 0xFF;
  reply[count++] = (n2 >> 0) & 0xFF;

  reply[count++] = (n3 >> 8) & 0xFF;
  reply[count++] = (n3 >> 0) & 0xFF;

  reply[1U] = count;

  m_serialPort.write(reply, count);
}

void CSerialPort::writeDebug(const char* text, int16_t n1, int16_t n2, int16_t n3, int16_t n4)
{
  if (!m_debug)
    return;

  uint8_t reply[130U];

  reply[0U] = MMDVM_FRAME_START;
  reply[1U] = 0U;
  reply[2U] = MMDVM_DEBUG5;

  uint8_t count = 3U;
  for (uint8_t i = 0U; text[i] != '\0'; i++, count++)
    reply[count] = text[i];

  reply[count++] = (n1 >> 8) & 0xFF;
  reply[count++] = (n1 >> 0) & 0xFF;

  reply[count++] = (n2 >> 8) & 0xFF;
  reply[count++] = (n2 >> 0) & 0xFF;

  reply[count++] = (n3 >> 8) & 0xFF;
  reply[count++] = (n3 >> 0) & 0xFF;

  reply[count++] = (n4 >> 8) & 0xFF;
  reply[count++] = (n4 >> 0) & 0xFF;

  reply[1U] = count;

  m_serialPort.write(reply, count);
}
