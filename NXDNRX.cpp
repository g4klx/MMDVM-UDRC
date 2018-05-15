/*
 *   Copyright (C) 2009-2018 by Jonathan Naylor G4KLX
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
#include "NXDNRX.h"
#include "Utils.h"

const float SCALING_FACTOR = 0.5722;

const uint8_t MAX_FSW_BIT_START_ERRS = 1U;
const uint8_t MAX_FSW_BIT_RUN_ERRS   = 3U;

const uint8_t MAX_FSW_SYMBOLS_ERRS = 2U;

const uint8_t BIT_MASK_TABLE[] = {0x80U, 0x40U, 0x20U, 0x10U, 0x08U, 0x04U, 0x02U, 0x01U};

#define WRITE_BIT1(p,i,b) p[(i)>>3] = (b) ? (p[(i)>>3] | BIT_MASK_TABLE[(i)&7]) : (p[(i)>>3] & ~BIT_MASK_TABLE[(i)&7])

const uint8_t NOAVEPTR = 99U;

const uint16_t NOENDPTR = 9999U;

const unsigned int MAX_FSW_FRAMES = 5U + 1U;

CNXDNRX::CNXDNRX() :
m_state(NXDNRXS_NONE),
m_bitBuffer(),
m_buffer(),
m_bitPtr(0U),
m_dataPtr(0U),
m_startPtr(NOENDPTR),
m_endPtr(NOENDPTR),
m_fswPtr(NOENDPTR),
m_minFSWPtr(NOENDPTR),
m_maxFSWPtr(NOENDPTR),
m_maxCorr(0.0F),
m_lostCount(0U),
m_countdown(0U),
m_centre(),
m_centreVal(0.0F),
m_threshold(),
m_thresholdVal(0.0F),
m_averagePtr(NOAVEPTR)
{
}

void CNXDNRX::reset()
{
  m_state        = NXDNRXS_NONE;
  m_dataPtr      = 0U;
  m_bitPtr       = 0U;
  m_maxCorr      = 0.0F;
  m_averagePtr   = NOAVEPTR;
  m_startPtr     = NOENDPTR;
  m_endPtr       = NOENDPTR;
  m_fswPtr       = NOENDPTR;
  m_minFSWPtr    = NOENDPTR;
  m_maxFSWPtr    = NOENDPTR;
  m_centreVal    = 0.0F;
  m_thresholdVal = 0.0F;
  m_lostCount    = 0U;
  m_countdown    = 0U;
}

void CNXDNRX::samples(const float* samples, uint8_t length)
{
  for (uint8_t i = 0U; i < length; i++) {
    float sample = samples[i];

    m_bitBuffer[m_bitPtr] <<= 1;
    if (sample < 0.0F)
      m_bitBuffer[m_bitPtr] |= 0x01U;

    m_buffer[m_dataPtr] = sample;

    switch (m_state) {
    case NXDNRXS_DATA:
      processData(sample);
      break;
    default:
      processNone(sample);
      break;
    }

    m_dataPtr++;
    if (m_dataPtr >= NXDN_FRAME_LENGTH_SAMPLES)
      m_dataPtr = 0U;

    m_bitPtr++;
    if (m_bitPtr >= NXDN_RADIO_SYMBOL_LENGTH)
      m_bitPtr = 0U;
  }
}

void CNXDNRX::processNone(float sample)
{
  bool ret = correlateFSW();
  if (ret) {
    // On the first sync, start the countdown to the state change
    if (m_countdown == 0U) {
      io.setDecode(true);
      io.setADCDetection(true);

      m_averagePtr = NOAVEPTR;

      m_countdown = 5U;
    }
  }

  if (m_countdown > 0U)
    m_countdown--;

  if (m_countdown == 1U) {
    m_minFSWPtr = m_fswPtr + NXDN_FRAME_LENGTH_SAMPLES - 1U;
    if (m_minFSWPtr >= NXDN_FRAME_LENGTH_SAMPLES)
      m_minFSWPtr -= NXDN_FRAME_LENGTH_SAMPLES;

    m_maxFSWPtr = m_fswPtr + 1U;
    if (m_maxFSWPtr >= NXDN_FRAME_LENGTH_SAMPLES)
      m_maxFSWPtr -= NXDN_FRAME_LENGTH_SAMPLES;

    m_state      = NXDNRXS_DATA;
    m_countdown  = 0U;
  }
}

void CNXDNRX::processData(float sample)
{
  if (m_minFSWPtr < m_maxFSWPtr) {
    if (m_dataPtr >= m_minFSWPtr && m_dataPtr <= m_maxFSWPtr)
      correlateFSW();
  } else {
    if (m_dataPtr >= m_minFSWPtr || m_dataPtr <= m_maxFSWPtr)
      correlateFSW();
  }

  if (m_dataPtr == m_endPtr) {
    // Only update the centre and threshold if they are from a good sync
    if (m_lostCount == MAX_FSW_FRAMES) {
      m_minFSWPtr = m_fswPtr + NXDN_FRAME_LENGTH_SAMPLES - 1U;
      if (m_minFSWPtr >= NXDN_FRAME_LENGTH_SAMPLES)
        m_minFSWPtr -= NXDN_FRAME_LENGTH_SAMPLES;

      m_maxFSWPtr = m_fswPtr + 1U;
      if (m_maxFSWPtr >= NXDN_FRAME_LENGTH_SAMPLES)
        m_maxFSWPtr -= NXDN_FRAME_LENGTH_SAMPLES;
    }

    calculateLevels(m_startPtr, NXDN_FRAME_LENGTH_SYMBOLS);

    DEBUG4("NXDNRX: sync found pos/centre/threshold", m_fswPtr, int16_t(m_centreVal * 2048.0F), int16_t(m_thresholdVal * 2048.0F));

    uint8_t frame[NXDN_FRAME_LENGTH_BYTES + 3U];
    samplesToBits(m_startPtr, NXDN_FRAME_LENGTH_SYMBOLS, frame, 8U, m_centreVal, m_thresholdVal);

    // We've not seen a data sync for too long, signal RXLOST and change to RX_NONE
    m_lostCount--;
    if (m_lostCount == 0U) {
      DEBUG1("NXDNRX: sync timed out, lost lock");

      io.setDecode(false);
      io.setADCDetection(false);

      serial.writeNXDNLost();

      m_state      = NXDNRXS_NONE;
      m_endPtr     = NOENDPTR;
      m_averagePtr = NOAVEPTR;
      m_countdown  = 0U;
      m_maxCorr    = 0.0F;
    } else {
      frame[0U] = m_lostCount == (MAX_FSW_FRAMES - 1U) ? 0x01U : 0x00U;
      writeData(frame);
      m_maxCorr = 0;
    }
  }
}

bool CNXDNRX::correlateFSW()
{
  if (countBits32((m_bitBuffer[m_bitPtr] & NXDN_FSW_SYMBOLS_MASK) ^ NXDN_FSW_SYMBOLS) <= MAX_FSW_SYMBOLS_ERRS) {
    uint16_t ptr = m_dataPtr + NXDN_FRAME_LENGTH_SAMPLES - NXDN_FSW_LENGTH_SAMPLES + NXDN_RADIO_SYMBOL_LENGTH;
    if (ptr >= NXDN_FRAME_LENGTH_SAMPLES)
      ptr -= NXDN_FRAME_LENGTH_SAMPLES;

    float corr = 0.0F;
    float min =  1.0F;
    float max = -1.0F;

    for (uint8_t i = 0U; i < NXDN_FSW_LENGTH_SYMBOLS; i++) {
      float val = m_buffer[ptr];

      if (val > max)
        max = val;
      if (val < min)
        min = val;

      switch (NXDN_FSW_SYMBOLS_VALUES[i]) {
      case +3:
        corr -= (val + val + val);
        break;
      case +1:
        corr -= val;
        break;
      case -1:
        corr += val;
        break;
      default:  // -3
        corr += (val + val + val);
        break;
      }

      ptr += NXDN_RADIO_SYMBOL_LENGTH;
      if (ptr >= NXDN_FRAME_LENGTH_SAMPLES)
        ptr -= NXDN_FRAME_LENGTH_SAMPLES;
    }

    if (corr > m_maxCorr) {
      if (m_averagePtr == NOAVEPTR) {
        m_centreVal    = (max + min) / 2.0F;
        m_thresholdVal = (max - m_centreVal) * SCALING_FACTOR;
      }

      uint16_t startPtr = m_dataPtr + NXDN_FRAME_LENGTH_SAMPLES - NXDN_FSW_LENGTH_SAMPLES + NXDN_RADIO_SYMBOL_LENGTH;
      if (startPtr >= NXDN_FRAME_LENGTH_SAMPLES)
        startPtr -= NXDN_FRAME_LENGTH_SAMPLES;

      uint8_t sync[NXDN_FSW_BYTES_LENGTH];
      samplesToBits(startPtr, NXDN_FSW_LENGTH_SYMBOLS, sync, 0U, m_centreVal, m_thresholdVal);

      uint8_t maxErrs;
      if (m_state == NXDNRXS_NONE)
        maxErrs = MAX_FSW_BIT_START_ERRS;
      else
        maxErrs = MAX_FSW_BIT_RUN_ERRS;

     uint8_t errs = 0U;
      for (uint8_t i = 0U; i < NXDN_FSW_BYTES_LENGTH; i++)
        errs += countBits8((sync[i] & NXDN_FSW_BYTES_MASK[i]) ^ NXDN_FSW_BYTES[i]);

      if (errs <= maxErrs) {
        m_maxCorr   = corr;
        m_lostCount = MAX_FSW_FRAMES;
        m_fswPtr    = m_dataPtr;

        m_startPtr = startPtr;

        m_endPtr = m_dataPtr + NXDN_FRAME_LENGTH_SAMPLES - NXDN_FSW_LENGTH_SAMPLES - 1U;
        if (m_endPtr >= NXDN_FRAME_LENGTH_SAMPLES)
          m_endPtr -= NXDN_FRAME_LENGTH_SAMPLES;

        return true;
      }
    }
  }

  return false;
}

void CNXDNRX::calculateLevels(uint16_t start, uint16_t count)
{
  float maxPos = -1.0F;
  float minPos =  1.0F;
  float maxNeg =  1.0F;
  float minNeg = -1.0F;

  for (uint16_t i = 0U; i < count; i++) {
    float sample = m_buffer[start];

    if (sample > 0.0F) {
      if (sample > maxPos)
        maxPos = sample;
      if (sample < minPos)
        minPos = sample;
    } else {
      if (sample < maxNeg)
        maxNeg = sample;
      if (sample > minNeg)
        minNeg = sample;
    }

    start += NXDN_RADIO_SYMBOL_LENGTH;
    if (start >= NXDN_FRAME_LENGTH_SAMPLES)
      start -= NXDN_FRAME_LENGTH_SAMPLES;
  }

  float posThresh = (maxPos + minPos) / 2.0F;
  float negThresh = (maxNeg + minNeg) / 2.0F;

  float centre = (posThresh + negThresh) / 2.0F;

  float threshold = posThresh - centre;

  DEBUG5("NXDNRX: pos/neg/centre/threshold", int16_t(posThresh * 2048.0F), int16_t(negThresh * 2048.0F), int16_t(centre * 2048.0F), int16_t(threshold * 2048.0F));

  if (m_averagePtr == NOAVEPTR) {
    for (uint8_t i = 0U; i < 16U; i++) {
      m_centre[i]    = centre;
      m_threshold[i] = threshold;
    }

    m_averagePtr = 0U;
  } else {
    m_centre[m_averagePtr]    = centre;
    m_threshold[m_averagePtr] = threshold;

    m_averagePtr++;
    if (m_averagePtr >= 16U)
      m_averagePtr = 0U;
  }

  m_centreVal    = 0.0F;
  m_thresholdVal = 0.0F;

  for (uint8_t i = 0U; i < 16U; i++) {
    m_centreVal    += m_centre[i];
    m_thresholdVal += m_threshold[i];
  }

  m_centreVal    /= 16.0F;
  m_thresholdVal /= 16.0F;
}

void CNXDNRX::samplesToBits(uint16_t start, uint16_t count, uint8_t* buffer, uint16_t offset, float centre, float threshold)
{
  for (uint16_t i = 0U; i < count; i++) {
    float sample = m_buffer[start] - centre;

    if (sample < -threshold) {
      WRITE_BIT1(buffer, offset, false);
      offset++;
      WRITE_BIT1(buffer, offset, true);
      offset++;
    } else if (sample < 0.0F) {
      WRITE_BIT1(buffer, offset, false);
      offset++;
      WRITE_BIT1(buffer, offset, false);
      offset++;
    } else if (sample < threshold) {
      WRITE_BIT1(buffer, offset, true);
      offset++;
      WRITE_BIT1(buffer, offset, false);
      offset++;
    } else {
      WRITE_BIT1(buffer, offset, true);
      offset++;
      WRITE_BIT1(buffer, offset, true);
      offset++;
    }

    start += NXDN_RADIO_SYMBOL_LENGTH;
    if (start >= NXDN_FRAME_LENGTH_SAMPLES)
      start -= NXDN_FRAME_LENGTH_SAMPLES;
  }
}

void CNXDNRX::writeData(uint8_t* data)
{
  serial.writeNXDNData(data, NXDN_FRAME_LENGTH_BYTES + 1U);
}

