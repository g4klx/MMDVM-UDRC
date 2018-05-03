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
#include "P25RX.h"
#include "Utils.h"

const float SCALING_FACTOR = 0.5722F;

const uint8_t MAX_SYNC_BIT_START_ERRS = 2U;
const uint8_t MAX_SYNC_BIT_RUN_ERRS   = 4U;

const uint8_t MAX_SYNC_SYMBOLS_ERRS = 2U;

const uint8_t BIT_MASK_TABLE[] = { 0x80U, 0x40U, 0x20U, 0x10U, 0x08U, 0x04U, 0x02U, 0x01U };

#define WRITE_BIT1(p,i,b) p[(i)>>3] = (b) ? (p[(i)>>3] | BIT_MASK_TABLE[(i)&7]) : (p[(i)>>3] & ~BIT_MASK_TABLE[(i)&7])

const uint8_t NOAVEPTR = 99U;

const uint16_t NOENDPTR = 9999U;

const unsigned int MAX_SYNC_FRAMES = 4U + 1U;

CP25RX::CP25RX() :
m_state(P25RXS_NONE),
m_bitBuffer(),
m_buffer(),
m_bitPtr(0U),
m_dataPtr(0U),
m_hdrStartPtr(NOENDPTR),
m_lduStartPtr(NOENDPTR),
m_lduEndPtr(NOENDPTR),
m_minSyncPtr(NOENDPTR),
m_maxSyncPtr(NOENDPTR),
m_hdrSyncPtr(NOENDPTR),
m_lduSyncPtr(NOENDPTR),
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

void CP25RX::reset()
{
  m_state         = P25RXS_NONE;
  m_dataPtr       = 0U;
  m_bitPtr        = 0U;
  m_maxCorr       = 0.0F;
  m_averagePtr    = NOAVEPTR;
  m_hdrStartPtr   = NOENDPTR;
  m_lduStartPtr   = NOENDPTR;
  m_lduEndPtr     = NOENDPTR;
  m_hdrSyncPtr    = NOENDPTR;
  m_lduSyncPtr    = NOENDPTR;
  m_minSyncPtr    = NOENDPTR;
  m_maxSyncPtr    = NOENDPTR;
  m_centreVal     = 0.0F;
  m_thresholdVal  = 0.0F;
  m_lostCount     = 0U;
  m_countdown     = 0U;
}

void CP25RX::samples(const float* samples, uint8_t length)
{
  for (uint8_t i = 0U; i < length; i++) {
    float sample = samples[i];

    m_bitBuffer[m_bitPtr] <<= 1;
    if (sample < 0.0F)
      m_bitBuffer[m_bitPtr] |= 0x01U;

    m_buffer[m_dataPtr] = sample;

    switch (m_state) {
    case P25RXS_HDR:
      processHdr(sample);
      break;
    case P25RXS_LDU:
      processLdu(sample);
      break;
    default:
      processNone(sample);
      break;
    }

    m_dataPtr++;
    if (m_dataPtr >= P25_LDU_FRAME_LENGTH_SAMPLES)
      m_dataPtr = 0U;

    m_bitPtr++;
    if (m_bitPtr >= P25_RADIO_SYMBOL_LENGTH)
      m_bitPtr = 0U;
  }
}

void CP25RX::processNone(float sample)
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
      // These are the sync positions for the following LDU after a HDR
      m_minSyncPtr = m_hdrSyncPtr + P25_HDR_FRAME_LENGTH_SAMPLES - 1U;
      if (m_minSyncPtr >= P25_LDU_FRAME_LENGTH_SAMPLES)
        m_minSyncPtr -= P25_LDU_FRAME_LENGTH_SAMPLES;

      m_maxSyncPtr = m_hdrSyncPtr + P25_HDR_FRAME_LENGTH_SAMPLES + 1U;
      if (m_maxSyncPtr >= P25_LDU_FRAME_LENGTH_SAMPLES)
        m_maxSyncPtr -= P25_LDU_FRAME_LENGTH_SAMPLES;

      m_state     = P25RXS_HDR;
      m_countdown = 0U;
  }
}

void CP25RX::processHdr(float sample)
{
  if (m_minSyncPtr < m_maxSyncPtr) {
    if (m_dataPtr >= m_minSyncPtr && m_dataPtr <= m_maxSyncPtr)
      correlateSync();
  } else {
    if (m_dataPtr >= m_minSyncPtr || m_dataPtr <= m_maxSyncPtr)
      correlateSync();
  }

  if (m_dataPtr == m_maxSyncPtr) {
    if (m_hdrSyncPtr != m_lduSyncPtr) {
      calculateLevels(m_hdrStartPtr, P25_HDR_FRAME_LENGTH_SYMBOLS);

      DEBUG4("P25RX: sync found in Hdr pos/centre/threshold", m_hdrSyncPtr, m_centreVal, m_thresholdVal);

      uint8_t frame[P25_HDR_FRAME_LENGTH_BYTES + 1U];
      samplesToBits(m_hdrStartPtr, P25_HDR_FRAME_LENGTH_SYMBOLS, frame, 8U, m_centreVal, m_thresholdVal);

      frame[0U] = 0x01U;
      serial.writeP25Hdr(frame, P25_HDR_FRAME_LENGTH_BYTES + 1U);
    }

    m_minSyncPtr = m_lduSyncPtr + P25_LDU_FRAME_LENGTH_SAMPLES - 1U;
    if (m_minSyncPtr >= P25_LDU_FRAME_LENGTH_SAMPLES)
      m_minSyncPtr -= P25_LDU_FRAME_LENGTH_SAMPLES;

    m_maxSyncPtr = m_lduSyncPtr + 1U;
    if (m_maxSyncPtr >= P25_LDU_FRAME_LENGTH_SAMPLES)
      m_maxSyncPtr -= P25_LDU_FRAME_LENGTH_SAMPLES;

    m_state   = P25RXS_LDU;
    m_maxCorr = 0.0F;
  }
}

void CP25RX::processLdu(float sample)
{
  if (m_minSyncPtr < m_maxSyncPtr) {
    if (m_dataPtr >= m_minSyncPtr && m_dataPtr <= m_maxSyncPtr)
      correlateSync();
  } else {
    if (m_dataPtr >= m_minSyncPtr || m_dataPtr <= m_maxSyncPtr)
      correlateSync();
  }

  if (m_dataPtr == m_lduEndPtr) {
    // Only update the centre and threshold if they are from a good sync
    if (m_lostCount == MAX_SYNC_FRAMES) {
      m_minSyncPtr = m_lduSyncPtr + P25_LDU_FRAME_LENGTH_SAMPLES - 1U;
      if (m_minSyncPtr >= P25_LDU_FRAME_LENGTH_SAMPLES)
        m_minSyncPtr -= P25_LDU_FRAME_LENGTH_SAMPLES;

      m_maxSyncPtr = m_lduSyncPtr + 1U;
      if (m_maxSyncPtr >= P25_LDU_FRAME_LENGTH_SAMPLES)
        m_maxSyncPtr -= P25_LDU_FRAME_LENGTH_SAMPLES;
    }

    calculateLevels(m_lduStartPtr, P25_LDU_FRAME_LENGTH_SYMBOLS);

    DEBUG4("P25RX: sync found in Ldu pos/centre/threshold", m_lduSyncPtr, m_centreVal, m_thresholdVal);

    uint8_t frame[P25_LDU_FRAME_LENGTH_BYTES + 3U];
    samplesToBits(m_lduStartPtr, P25_LDU_FRAME_LENGTH_SYMBOLS, frame, 8U, m_centreVal, m_thresholdVal);

    // We've not seen a data sync for too long, signal RXLOST and change to RX_NONE
    m_lostCount--;
    if (m_lostCount == 0U) {
      DEBUG1("P25RX: sync timed out, lost lock");

      io.setDecode(false);
      io.setADCDetection(false);

      serial.writeP25Lost();

      m_state      = P25RXS_NONE;
      m_lduEndPtr  = NOENDPTR;
      m_averagePtr = NOAVEPTR;
      m_countdown  = 0U;
      m_maxCorr    = 0.0F;
    } else {
      frame[0U] = m_lostCount == (MAX_SYNC_FRAMES - 1U) ? 0x01U : 0x00U;
      writeLdu(frame);
      m_maxCorr = 0.0F;
    }
  }
}

bool CP25RX::correlateSync()
{
  if (countBits32((m_bitBuffer[m_bitPtr] & P25_SYNC_SYMBOLS_MASK) ^ P25_SYNC_SYMBOLS) <= MAX_SYNC_SYMBOLS_ERRS) {
    uint16_t ptr = m_dataPtr + P25_LDU_FRAME_LENGTH_SAMPLES - P25_SYNC_LENGTH_SAMPLES + P25_RADIO_SYMBOL_LENGTH;
    if (ptr >= P25_LDU_FRAME_LENGTH_SAMPLES)
      ptr -= P25_LDU_FRAME_LENGTH_SAMPLES;

    float corr = 0.0F;
    float min =  1.0F;
    float max = -1.0F;

    for (uint8_t i = 0U; i < P25_SYNC_LENGTH_SYMBOLS; i++) {
      float val = m_buffer[ptr];

      if (val > max)
        max = val;
      if (val < min)
        min = val;

      switch (P25_SYNC_SYMBOLS_VALUES[i]) {
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

      ptr += P25_RADIO_SYMBOL_LENGTH;
      if (ptr >= P25_LDU_FRAME_LENGTH_SAMPLES)
        ptr -= P25_LDU_FRAME_LENGTH_SAMPLES;
    }

    if (corr > m_maxCorr) {
      if (m_averagePtr == NOAVEPTR) {
        m_centreVal = (max + min) / 2.0F;

        m_thresholdVal = (max - m_centreVal) * SCALING_FACTOR;
      }

      uint16_t startPtr = m_dataPtr + P25_LDU_FRAME_LENGTH_SAMPLES - P25_SYNC_LENGTH_SAMPLES + P25_RADIO_SYMBOL_LENGTH;
      if (startPtr >= P25_LDU_FRAME_LENGTH_SAMPLES)
        startPtr -= P25_LDU_FRAME_LENGTH_SAMPLES;

      uint8_t sync[P25_SYNC_BYTES_LENGTH];
      samplesToBits(startPtr, P25_SYNC_LENGTH_SYMBOLS, sync, 0U, m_centreVal, m_thresholdVal);

      uint8_t maxErrs;
      if (m_state == P25RXS_NONE)
        maxErrs = MAX_SYNC_BIT_START_ERRS;
      else
        maxErrs = MAX_SYNC_BIT_RUN_ERRS;

      uint8_t errs = 0U;
      for (uint8_t i = 0U; i < P25_SYNC_BYTES_LENGTH; i++)
        errs += countBits8(sync[i] ^ P25_SYNC_BYTES[i]);

      if (errs <= maxErrs) {
        m_maxCorr     = corr;
        m_lostCount   = MAX_SYNC_FRAMES;

        m_lduSyncPtr  = m_dataPtr;

        // These are the positions of the start and end of an LDU
        m_lduStartPtr = startPtr;

        m_lduEndPtr = m_dataPtr + P25_LDU_FRAME_LENGTH_SAMPLES - P25_SYNC_LENGTH_SAMPLES - 1U;
        if (m_lduEndPtr >= P25_LDU_FRAME_LENGTH_SAMPLES)
          m_lduEndPtr -= P25_LDU_FRAME_LENGTH_SAMPLES;

        if (m_state == P25RXS_NONE) {
          m_hdrSyncPtr = m_dataPtr;

          // This is the position of the start of a HDR
          m_hdrStartPtr = startPtr;

          // These are the range of positions for a sync for an LDU following a HDR
          m_minSyncPtr = m_dataPtr + P25_HDR_FRAME_LENGTH_SAMPLES - 1U;
          if (m_minSyncPtr >= P25_LDU_FRAME_LENGTH_SAMPLES)
            m_minSyncPtr -= P25_LDU_FRAME_LENGTH_SAMPLES;

          m_maxSyncPtr = m_dataPtr + P25_HDR_FRAME_LENGTH_SAMPLES + 1U;
          if (m_maxSyncPtr >= P25_LDU_FRAME_LENGTH_SAMPLES)
            m_maxSyncPtr -= P25_LDU_FRAME_LENGTH_SAMPLES;
        }

        return true;
      }
    }
  }

  return false;
}

void CP25RX::calculateLevels(uint16_t start, uint16_t count)
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

    start += P25_RADIO_SYMBOL_LENGTH;
    if (start >= P25_LDU_FRAME_LENGTH_SAMPLES)
      start -= P25_LDU_FRAME_LENGTH_SAMPLES;
  }

  float posThresh = (maxPos + minPos) / 2.0F;
  float negThresh = (maxNeg + minNeg) / 2.0F;

  float centre = (posThresh + negThresh) / 2.0F;

  float threshold = posThresh - centre;

  DEBUG5("P25RX: pos/neg/centre/threshold", posThresh, negThresh, centre, threshold);

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

void CP25RX::samplesToBits(uint16_t start, uint16_t count, uint8_t* buffer, uint16_t offset, float centre, float threshold)
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

    start += P25_RADIO_SYMBOL_LENGTH;
    if (start >= P25_LDU_FRAME_LENGTH_SAMPLES)
      start -= P25_LDU_FRAME_LENGTH_SAMPLES;
  }
}

void CP25RX::writeLdu(uint8_t* ldu)
{
  serial.writeP25Ldu(ldu, P25_LDU_FRAME_LENGTH_BYTES + 1U);
}

