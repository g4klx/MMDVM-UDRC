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

#if !defined(BIQUAD_H)
#define  BIQUAD_H

#include <cstdint>

class CBiquad {
public:
  CBiquad(uint32_t numStages, const float* pCoeffs);

  void process(const float* pSrc, float* pDst, uint32_t blockLength);

private:
  uint16_t     m_numStages;
  const float* m_pCoeffs;
  float*       m_pState;
};

#endif

