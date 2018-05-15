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
#include "YSFRX.h"
#include "Utils.h"

const float SCALING_FACTOR = 0.5722F;

const uint8_t MAX_SYNC_BIT_START_ERRS = 2U;
const uint8_t MAX_SYNC_BIT_RUN_ERRS   = 4U;

const uint8_t MAX_SYNC_SYMBOLS_ERRS = 3U;

const uint8_t BIT_MASK_TABLE[] = {0x80U, 0x40U, 0x20U, 0x10U, 0x08U, 0x04U, 0x02U, 0x01U};

#define WRITE_BIT1(p,i,b) p[(i)>>3] = (b) ? (p[(i)>>3] | BIT_MASK_TABLE[(i)&7]) : (p[(i)>>3] & ~BIT_MASK_TABLE[(i)&7])

const uint8_t NOAVEPTR = 99U;

const uint16_t NOENDPTR = 9999U;

const unsigned int MAX_SYNC_FRAMES = 4U + 1U;

CYSFRX::CYSFRX() :
m_state(YSFRXS_NONE),
m_bitBuffer(),
m_buffer(),
m_bitPtr(0U),
m_dataPtr(0U),
m_startPtr(NOENDPTR),
m_endPtr(NOENDPTR),
m_syncPtr(NOENDPTR),
m_minSyncPtr(NOENDPTR),
m_maxSyncPtr(NOENDPTR),
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

void CYSFRX::reset()
{
  m_state        = YSFRXS_NONE;
  m_dataPtr      = 0U;
  m_bitPtr       = 0U;
  m_maxCorr      = 0.0F;
  m_averagePtr   = NOAVEPTR;
  m_startPtr     = NOENDPTR;
  m_endPtr       = NOENDPTR;
  m_syncPtr      = NOENDPTR;
  m_minSyncPtr   = NOENDPTR;
  m_maxSyncPtr   = NOENDPTR;
  m_centreVal    = 0.0F;
  m_thresholdVal = 0.0F;
  m_lostCount    = 0U;
  m_countdown    = 0U;
}

void CYSFRX::samples(const float* samples, uint8_t length)
{
  for (uint8_t i = 0U; i < length; i++) {
    float sample = samples[i];

    m_bitBuffer[m_bitPtr] <<= 1;
    if (sample < 0)
      m_bitBuffer[m_bitPtr] |= 0x01U;

    m_buffer[m_dataPtr] = sample;

    switch (m_state) {
    case YSFRXS_DATA:
      processData(sample);
      break;
    default:
      processNone(sample);
      break;
    }

    m_dataPtr++;
    if (m_dataPtr >= YSF_FRAME_LENGTH_SAMPLES)
      m_dataPtr = 0U;

    m_bitPtr++;
    if (m_bitPtr >= YSF_RADIO_SYMBOL_LENGTH)
      m_bitPtr = 0U;
  }
}

void CYSFRX::processNone(float sample)
{
  bool ret = correlateSync();
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
    m_minSyncPtr = m_syncPtr + YSF_FRAME_LENGTH_SAMPLES - 1U;
    if (m_minSyncPtr >= YSF_FRAME_LENGTH_SAMPLES)
      m_minSyncPtr -= YSF_FRAME_LENGTH_SAMPLES;

    m_maxSyncPtr = m_syncPtr + 1U;
    if (m_maxSyncPtr >= YSF_FRAME_LENGTH_SAMPLES)
      m_maxSyncPtr -= YSF_FRAME_LENGTH_SAMPLES;

    m_state      = YSFRXS_DATA;
    m_countdown  = 0U;
  }
}

void CYSFRX::processData(float sample)
{
  if (m_minSyncPtr < m_maxSyncPtr) {
    if (m_dataPtr >= m_minSyncPtr && m_dataPtr <= m_maxSyncPtr)
      correlateSync();
  } else {
    if (m_dataPtr >= m_minSyncPtr || m_dataPtr <= m_maxSyncPtr)
      correlateSync();
  }

  if (m_dataPtr == m_endPtr) {
    // Only update the centre and threshold if they are from a good sync
    if (m_lostCount == MAX_SYNC_FRAMES) {
      m_minSyncPtr = m_syncPtr + YSF_FRAME_LENGTH_SAMPLES - 1U;
      if (m_minSyncPtr >= YSF_FRAME_LENGTH_SAMPLES)
        m_minSyncPtr -= YSF_FRAME_LENGTH_SAMPLES;

      m_maxSyncPtr = m_syncPtr + 1U;
      if (m_maxSyncPtr >= YSF_FRAME_LENGTH_SAMPLES)
        m_maxSyncPtr -= YSF_FRAME_LENGTH_SAMPLES;
    }

    calculateLevels(m_startPtr, YSF_FRAME_LENGTH_SYMBOLS);

    DEBUG4("YSFRX: sync found pos/centre/threshold", m_syncPtr, int16_t(m_centreVal * 2048.0F), int16_t(m_thresholdVal * 2048.0F));

    uint8_t frame[YSF_FRAME_LENGTH_BYTES + 3U];
    samplesToBits(m_startPtr, YSF_FRAME_LENGTH_SYMBOLS, frame, 8U, m_centreVal, m_thresholdVal);

    // We've not seen a data sync for too long, signal RXLOST and change to RX_NONE
    m_lostCount--;
    if (m_lostCount == 0U) {
      DEBUG1("YSFRX: sync timed out, lost lock");

      io.setDecode(false);
      io.setADCDetection(false);

      serial.writeYSFLost();

      m_state      = YSFRXS_NONE;
      m_endPtr     = NOENDPTR;
      m_averagePtr = NOAVEPTR;
      m_countdown  = 0U;
      m_maxCorr    = 0.0F;
    } else {
      frame[0U] = m_lostCount == (MAX_SYNC_FRAMES - 1U) ? 0x01U : 0x00U;
      writeData(frame);
      m_maxCorr = 0.0F;
    }
  }
}

bool CYSFRX::correlateSync()
{
  if (countBits32((m_bitBuffer[m_bitPtr] & YSF_SYNC_SYMBOLS_MASK) ^ YSF_SYNC_SYMBOLS) <= MAX_SYNC_SYMBOLS_ERRS) {
    uint16_t ptr = m_dataPtr + YSF_FRAME_LENGTH_SAMPLES - YSF_SYNC_LENGTH_SAMPLES + YSF_RADIO_SYMBOL_LENGTH;
    if (ptr >= YSF_FRAME_LENGTH_SAMPLES)
      ptr -= YSF_FRAME_LENGTH_SAMPLES;

    float corr = 0.0F;
    float min =  1.0F;
    float max = -1.0F;

    for (uint8_t i = 0U; i < YSF_SYNC_LENGTH_SYMBOLS; i++) {
      float val = m_buffer[ptr];

      if (val > max)
        max = val;
      if (val < min)
        min = val;

      switch (YSF_SYNC_SYMBOLS_VALUES[i]) {
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

      ptr += YSF_RADIO_SYMBOL_LENGTH;
      if (ptr >= YSF_FRAME_LENGTH_SAMPLES)
        ptr -= YSF_FRAME_LENGTH_SAMPLES;
    }

    if (corr > m_maxCorr) {
      if (m_averagePtr == NOAVEPTR) {
        m_centreVal    = (max + min) / 2.0F;
        m_thresholdVal = (max - m_centreVal) * SCALING_FACTOR;
      }

      uint16_t startPtr = m_dataPtr + YSF_FRAME_LENGTH_SAMPLES - YSF_SYNC_LENGTH_SAMPLES + YSF_RADIO_SYMBOL_LENGTH;
      if (startPtr >= YSF_FRAME_LENGTH_SAMPLES)
        startPtr -= YSF_FRAME_LENGTH_SAMPLES;

      uint8_t sync[YSF_SYNC_BYTES_LENGTH];
      samplesToBits(startPtr, YSF_SYNC_LENGTH_SYMBOLS, sync, 0U, m_centreVal, m_thresholdVal);

      uint8_t maxErrs;
      if (m_state == YSFRXS_NONE)
        maxErrs = MAX_SYNC_BIT_START_ERRS;
      else
        maxErrs = MAX_SYNC_BIT_RUN_ERRS;

      uint8_t errs = 0U;
      for (uint8_t i = 0U; i < YSF_SYNC_BYTES_LENGTH; i++)
        errs += countBits8(sync[i] ^ YSF_SYNC_BYTES[i]);

      if (errs <= maxErrs) {
        m_maxCorr   = corr;
        m_lostCount = MAX_SYNC_FRAMES;
        m_syncPtr   = m_dataPtr;

        m_startPtr = startPtr;

        m_endPtr = m_dataPtr + YSF_FRAME_LENGTH_SAMPLES - YSF_SYNC_LENGTH_SAMPLES - 1U;
        if (m_endPtr >= YSF_FRAME_LENGTH_SAMPLES)
          m_endPtr -= YSF_FRAME_LENGTH_SAMPLES;

        return true;
      }
    }
  }

  return false;
}

void CYSFRX::calculateLevels(uint16_t start, uint16_t count)
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

    start += YSF_RADIO_SYMBOL_LENGTH;
    if (start >= YSF_FRAME_LENGTH_SAMPLES)
      start -= YSF_FRAME_LENGTH_SAMPLES;
  }

  float posThresh = (maxPos + minPos) / 2.0F;
  float negThresh = (maxNeg + minNeg) / 2.0F;

  float centre = (posThresh + negThresh) / 2.0F;

  float threshold = posThresh - centre;

  DEBUG5("YSFRX: pos/neg/centre/threshold", int16_t(posThresh * 2048.0F), int16_t(negThresh * 2048.0F), int16_t(centre * 2048.0F), int16_t(threshold * 2048.0F));

  if (m_averagePtr == NOAVEPTR) {
    for (uint8_t i = 0U; i < 16U; i++) {
      m_centre[i]    = centre;
      m_threshold[i] = threshold;
    }

    m_averagePtr = 0U;
  } else {
    m_centre[m_averagePtr] = centre;
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

void CYSFRX::samplesToBits(uint16_t start, uint16_t count, uint8_t* buffer, uint16_t offset, float centre, float threshold)
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

    start += YSF_RADIO_SYMBOL_LENGTH;
    if (start >= YSF_FRAME_LENGTH_SAMPLES)
      start -= YSF_FRAME_LENGTH_SAMPLES;
  }
}

void CYSFRX::writeData(uint8_t* data)
{
  serial.writeYSFData(data, YSF_FRAME_LENGTH_BYTES + 1U);
}

