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

#include "FIR.h"

CFIR::CFIR(uint16_t numTaps, const float* pCoeffs, uint32_t blockSize) :
m_numTaps(numTaps),
m_pCoeffs(pCoeffs)
{
  m_pState = new float[numTaps + blockSize - 1U];
}

void CFIR::process(const float* pSrc, float* pDst, uint32_t blockSize)
{
   float *pState  = m_pState;                 /* State pointer */
   const float *pCoeffs = m_pCoeffs;               /* Coefficient pointer */
   float *pStateCurnt;                        /* Points to the current sample of the state */
   float *px;                            /* Temporary pointers for state and coefficient buffers */
   const float *pb;                            /* Temporary pointers for state and coefficient buffers */
   uint32_t numTaps = m_numTaps;                 /* Number of filter coefficients in the filter */
   uint32_t i, tapCnt, blkCnt;                    /* Loop counters */

   float acc;

   /* S->pState points to state array which contains previous frame (numTaps - 1) samples */
   /* pStateCurnt points to the location where the new input data should be written */
   pStateCurnt = &(m_pState[(numTaps - 1U)]);

   /* Initialize blkCnt with blockSize */
   blkCnt = blockSize;

   while (blkCnt > 0U)
   {
      /* Copy one sample at a time into state buffer */
      *pStateCurnt++ = *pSrc++;

      /* Set the accumulator to zero */
      acc = 0.0f;

      /* Initialize state pointer */
      px = pState;

      /* Initialize Coefficient pointer */
      pb = pCoeffs;

      i = numTaps;

      /* Perform the multiply-accumulates */
      do
      {
         /* acc =  b[numTaps-1] * x[n-numTaps-1] + b[numTaps-2] * x[n-numTaps-2] + b[numTaps-3] * x[n-numTaps-3] +...+ b[0] * x[0] */
         acc += *px++ * *pb++;
         i--;

      } while (i > 0U);

      /* The result is store in the destination buffer. */
      *pDst++ = acc;

      /* Advance state pointer by 1 for the next sample */
      pState = pState + 1;

      blkCnt--;
   }

   /* Processing is complete.
   ** Now copy the last numTaps - 1 samples to the starting of the state buffer.
   ** This prepares the state buffer for the next function call. */

   /* Points to the start of the state buffer */
   pStateCurnt = m_pState;

   /* Copy numTaps number of values */
   tapCnt = numTaps - 1U;

   /* Copy data */
   while (tapCnt > 0U)
   {
      *pStateCurnt++ = *pState++;

      /* Decrement the loop counter */
      tapCnt--;
   }
}

