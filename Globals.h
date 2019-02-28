/*
 *   Copyright (C) 2015-2019 by Jonathan Naylor G4KLX
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

#if !defined(GLOBALS_H)
#define  GLOBALS_H

#include <cstdio>
#include <cstdint>
#include <cstring>

enum MMDVM_STATE {
  STATE_IDLE      = 0,
  STATE_DSTAR     = 1,
  STATE_DMR       = 2,
  STATE_YSF       = 3,
  STATE_P25       = 4,
  STATE_NXDN      = 5,
  STATE_POCSAG    = 6,

  // Dummy states start at 90
  STATE_NXDNCAL1K = 91,
  STATE_DMRDMO1K  = 92,
  STATE_P25CAL1K  = 93,
  STATE_LFCAL     = 95,
  STATE_RSSICAL   = 96,
  STATE_CWID      = 97,
  STATE_DMRCAL    = 98,
  STATE_DSTARCAL  = 99,
  STATE_INTCAL    = 100,
  STATE_CALPOCSAG = 101
};

#include "SerialPort.h"
#include "DMRDMORX.h"
#include "DMRDMOTX.h"
#include "DStarRX.h"
#include "DStarTX.h"
#include "YSFRX.h"
#include "YSFTX.h"
#include "P25RX.h"
#include "P25TX.h"
#include "NXDNRX.h"
#include "NXDNTX.h"
#include "POCSAGTX.h"
#include "CalDStarRX.h"
#include "CalDStarTX.h"
#include "CalDMR.h"
#include "CalP25.h"
#include "CalNXDN.h"
#include "CalPOCSAG.h"
#include "CWIdTX.h"
#include "Debug.h"
#include "IO.h"

const uint16_t RX_BLOCK_SIZE = 2U;

const uint16_t TX_RINGBUFFER_SIZE = 500U;
const uint16_t RX_RINGBUFFER_SIZE = 600U;

extern MMDVM_STATE m_modemState;

extern bool m_dstarEnable;
extern bool m_dmrEnable;
extern bool m_ysfEnable;
extern bool m_p25Enable;
extern bool m_nxdnEnable;
extern bool m_pocsagEnable;

extern bool m_duplex;

extern bool m_tx;
extern bool m_dcd;

extern CSerialPort serial;
extern CIO io;

extern CDStarRX dstarRX;
extern CDStarTX dstarTX;

extern CDMRDMORX dmrDMORX;
extern CDMRDMOTX dmrDMOTX;

extern CYSFRX ysfRX;
extern CYSFTX ysfTX;

extern CP25RX p25RX;
extern CP25TX p25TX;

extern CNXDNRX nxdnRX;
extern CNXDNTX nxdnTX;

extern CPOCSAGTX pocsagTX;

extern CCalDStarRX calDStarRX;
extern CCalDStarTX calDStarTX;
extern CCalDMR     calDMR;
extern CCalP25     calP25;
extern CCalNXDN    calNXDN;
extern CCalPOCSAG  calPOCSAG;

extern CCWIdTX cwIdTX;

#endif

