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

#include "Biquad.h"

CBiquad::CBiquad(uint32_t numStages, const float* pCoeffs) :
m_numStages(numStages),
m_pCoeffs(pCoeffs)
{
  m_pState = new float[4U * numStages];
}

void CBiquad::process(const float* pSrc, float* pDst, uint32_t blockSize)
{
  const float *pIn = pSrc;                         /*  source pointer            */
  float *pOut = pDst;                        /*  destination pointer       */
  float *pState = m_pState;                 /*  pState pointer            */
  const float *pCoeffs = m_pCoeffs;               /*  coefficient pointer       */
  float acc;                                 /*  Simulates the accumulator */
  float b0, b1, b2, a1, a2;                  /*  Filter coefficients       */
  float Xn1, Xn2, Yn1, Yn2;                  /*  Filter pState variables   */
  float Xn;                                  /*  temporary input           */
  uint32_t sample, stage = m_numStages;         /*  loop counters             */

  do
  {
    /* Reading the coefficients */
    b0 = *pCoeffs++;
    b1 = *pCoeffs++;
    b2 = *pCoeffs++;
    a1 = *pCoeffs++;
    a2 = *pCoeffs++;

    /* Reading the pState values */
    Xn1 = pState[0];
    Xn2 = pState[1];
    Yn1 = pState[2];
    Yn2 = pState[3];

    /*      The variables acc holds the output value that is computed:
     *    acc =  b0 * x[n] + b1 * x[n-1] + b2 * x[n-2] + a1 * y[n-1]   + a2 * y[n-2]
     */

    sample = blockSize;

    while (sample > 0U)
    {
      /* Read the input */
      Xn = *pIn++;

      /* acc =  b0 * x[n] + b1 * x[n-1] + b2 * x[n-2] + a1 * y[n-1] + a2 * y[n-2] */
      acc = (b0 * Xn) + (b1 * Xn1) + (b2 * Xn2) + (a1 * Yn1) + (a2 * Yn2);

      /* Store the result in the accumulator in the destination buffer. */
      *pOut++ = acc;

      /* Every time after the output is computed state should be updated. */
      /* The states should be updated as:    */
      /* Xn2 = Xn1    */
      /* Xn1 = Xn     */
      /* Yn2 = Yn1    */
      /* Yn1 = acc   */
      Xn2 = Xn1;
      Xn1 = Xn;
      Yn2 = Yn1;
      Yn1 = acc;

      /* decrement the loop counter */
      sample--;
    }

    /*  Store the updated state variables back into the pState array */
    *pState++ = Xn1;
    *pState++ = Xn2;
    *pState++ = Yn1;
    *pState++ = Yn2;

    /*  The first stage goes from the input buffer to the output buffer. */
    /*  Subsequent numStages  occur in-place in the output buffer */
    pIn = pDst;

    /* Reset the output pointer */
    pOut = pDst;

    /* decrement the loop counter */
    stage--;

  } while (stage > 0U);
}

