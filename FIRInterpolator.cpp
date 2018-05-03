/*
 *   Copyright (C) 2018 by Jonathan Naylor G4KLX
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

#include "FIRInterpolator.h"

CFIRInterpolator::CFIRInterpolator(uint8_t L, uint16_t phaseLength, const float* pCoeffs, uint32_t blockSize) :
m_L(L),
m_phaseLength(phaseLength),
m_pCoeffs(pCoeffs)
{
  m_pState = new float[L * phaseLength + blockSize - 1U];
}

void CFIRInterpolator::process(const float* pSrc, float* pDst, uint32_t blockSize)
{
  float *pState = m_pState;                 /* State pointer */
  const float *pCoeffs = m_pCoeffs;               /* Coefficient pointer */
  float *pStateCurnt;                        /* Points to the current sample of the state */
  float *ptr1;                        /* Temporary pointers for state and coefficient buffers */
  const float *ptr2;                        /* Temporary pointers for state and coefficient buffers */


  float sum;                                 /* Accumulator */
  uint32_t i, blkCnt;                            /* Loop counters */
  uint16_t phaseLen = m_phaseLength, tapCnt;    /* Length of each polyphase filter component */


  /* S->pState buffer contains previous frame (phaseLen - 1) samples */
  /* pStateCurnt points to the location where the new input data should be written */
  pStateCurnt = m_pState + (phaseLen - 1U);

  /* Total number of intput samples */
  blkCnt = blockSize;

  /* Loop over the blockSize. */
  while (blkCnt > 0U)
  {
    /* Copy new input sample into the state buffer */
    *pStateCurnt++ = *pSrc++;

    /* Loop over the Interpolation factor. */
    i = m_L;

    while (i > 0U)
    {
      /* Set accumulator to zero */
      sum = 0.0f;

      /* Initialize state pointer */
      ptr1 = pState;

      /* Initialize coefficient pointer */
      ptr2 = pCoeffs + (i - 1U);

      /* Loop over the polyPhase length */
      tapCnt = phaseLen;

      while (tapCnt > 0U)
      {
        /* Perform the multiply-accumulate */
        sum += *ptr1++ * *ptr2;

        /* Increment the coefficient pointer by interpolation factor times. */
        ptr2 += m_L;

        /* Decrement the loop counter */
        tapCnt--;
      }

      /* The result is in the accumulator, store in the destination buffer. */
      *pDst++ = sum;

      /* Decrement the loop counter */
      i--;
    }

    /* Advance the state pointer by 1
     * to process the next group of interpolation factor number samples */
    pState = pState + 1;

    /* Decrement the loop counter */
    blkCnt--;
  }

  /* Processing is complete.
   ** Now copy the last phaseLen - 1 samples to the start of the state buffer.
   ** This prepares the state buffer for the next function call. */

  /* Points to the start of the state buffer */
  pStateCurnt = m_pState;

  tapCnt = phaseLen - 1U;

  while (tapCnt > 0U)
  {
    *pStateCurnt++ = *pState++;

    /* Decrement the loop counter */
    tapCnt--;
  }
}

