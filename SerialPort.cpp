/*
 *   Copyright (C) 2013,2015-2019 by Jonathan Naylor G4KLX
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

#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>

#include <cassert>

#include "Globals.h"

#include "SerialPort.h"

#define CHECK_BIT(var, bit) \
	(var & (1 << bit))
#define UINT8_TO_FLOAT(var) \
	(float(var) / 255.0f)

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
	m_ptr(0U),
	m_len(0U),
	m_debug(true),
	m_repeat(),
	m_ptyPath("/dev/ttyMMDVM0")
{
}

void CSerialPort::setPtyPath(const std::string& ptyPath)
{
	m_ptyPath = ptyPath;
}

void CSerialPort::sendACK(mmdvm_frame &frame)
{
	uint8_t reply[] = {
	    MMDVM_FRAME_START,
	    4,
	    MMDVM_ACK,
	    frame.operation
	};

	write(reply, 4);
}

void CSerialPort::sendNAK(mmdvm_frame &frame, uint8_t err)
{

    uint8_t reply[] = {
        MMDVM_FRAME_START,
        5,
        MMDVM_NAK,
        frame.operation,
        err
    };

	write(reply, 5);
}

void CSerialPort::getStatus()
{
	io.resetWatchdog();

	uint8_t reply[20U];

	// Send all sorts of interesting internal values
	reply[0U]  = MMDVM_FRAME_START;
	reply[1U]  = 13U;
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
	if (m_pocsagEnable)
		reply[3U] |= 0x20U;

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

	if (m_pocsagEnable)
		reply[12U] = pocsagTX.getSpace();
	else
		reply[12U] = 0U;

	write(reply, 13);
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

	write(reply, count);
}

uint8_t CSerialPort::setConfig(mmdvm_frame &frame)
{
	if(frame.length != sizeof(mmdvm_config_frame) + 3)
		return 4;

	//  XXX This is probably an unnecessary copy.
	//  XXX But it makes shortens the code after.
	mmdvm_config_frame config = frame.config;

	bool rxInvert  = CHECK_BIT(config.config_flags, 0);
	bool txInvert  = CHECK_BIT(config.config_flags, 1);
	bool pttInvert = CHECK_BIT(config.config_flags, 2);
	bool ysfLoDev  = CHECK_BIT(config.config_flags, 3);
	bool simplex   = CHECK_BIT(config.config_flags, 7);


	m_debug = CHECK_BIT(config.config_flags, 4);

	bool dstarEnable  = CHECK_BIT(config.protocol_enable_flags, 0);
	bool dmrEnable    = CHECK_BIT(config.protocol_enable_flags, 1);
	bool ysfEnable    = CHECK_BIT(config.protocol_enable_flags, 2);
	bool p25Enable    = CHECK_BIT(config.protocol_enable_flags, 3);
	bool nxdnEnable   = CHECK_BIT(config.protocol_enable_flags, 4);
	bool pocsagEnable = CHECK_BIT(config.protocol_enable_flags, 5);


	MMDVM_STATE modemState = MMDVM_STATE(config.modem_state);

	if (modemState != STATE_IDLE &&
	    modemState != STATE_DSTAR &&
	    modemState != STATE_DMR &&
	    modemState != STATE_YSF &&
	    modemState != STATE_P25 &&
	    modemState != STATE_NXDN &&
	    modemState != STATE_POCSAG &&
	    modemState != STATE_DSTARCAL &&
	    modemState != STATE_DMRCAL &&
	    modemState != STATE_RSSICAL &&
	    modemState != STATE_LFCAL &&
	    modemState != STATE_P25CAL1K &&
	    modemState != STATE_DMRDMO1K &&
	    modemState != STATE_NXDNCAL1K &&
		modemState != STATE_POCSAGCAL)
		return 4;
	if (modemState == STATE_DSTAR && !dstarEnable)
		return 4;
	if (modemState == STATE_DMR && !dmrEnable)
		return 4;
	if (modemState == STATE_YSF && !ysfEnable)
		return 4;
	if (modemState == STATE_P25 && !p25Enable)
		return 4;
	if (modemState == STATE_NXDN && !nxdnEnable)
		return 4;
	if (modemState == STATE_POCSAG && !pocsagEnable)
		return 4;


	m_modemState  = modemState;

	m_dstarEnable  = dstarEnable;
	m_dmrEnable    = dmrEnable;
	m_ysfEnable    = ysfEnable;
	m_p25Enable    = p25Enable;
	m_nxdnEnable   = nxdnEnable;
	m_pocsagEnable = pocsagEnable;
	m_duplex       = !simplex;

	float rxLevel = float(config.rx_level) / 255.0F;
	float cwIdTXLevel  = float(config.cw_id_level) / 255.0F;

	if (config.color_code > 15)
		return 4;

	dmrDMORX.setColorCode(config.color_code);

	// XXX Where are bytes 7 and 8?

	float dstarTXLevel  = UINT8_TO_FLOAT(config.dstar_tx_level);
	float dmrTXLevel    = UINT8_TO_FLOAT(config.dmr_tx_level);
	float ysfTXLevel    = UINT8_TO_FLOAT(config.ysf_tx_level);
	float p25TXLevel    = UINT8_TO_FLOAT(config.p25_tx_level);
	float nxdnTXLevel   = UINT8_TO_FLOAT(config.nxdn_tx_level);
	float pocsagTXLevel = UINT8_TO_FLOAT(config.pocsag_tx_level);

	float txDCOffset = float(config.tx_dc_offset - 128) / 128.0F;
	float rxDCOffset = float(config.rx_dc_offset - 128) / 128.0F;

	if (config.tx_delay > 50U)
		return 4;

	dstarTX.setTXDelay(config.tx_delay);
	ysfTX.setTXDelay(config.tx_delay);
	p25TX.setTXDelay(config.tx_delay);
	dmrDMOTX.setTXDelay(config.tx_delay);
	nxdnTX.setTXDelay(config.tx_delay);
	pocsagTX.setTXDelay(config.tx_delay);

	ysfTX.setParams(ysfLoDev, config.ysf_tx_hang);

	io.setParameters(rxInvert, txInvert, pttInvert, rxLevel, cwIdTXLevel, dstarTXLevel, dmrTXLevel, ysfTXLevel, p25TXLevel, nxdnTXLevel, pocsagTXLevel, txDCOffset, rxDCOffset);

	io.start();

	return 0U;
}

uint8_t CSerialPort::setMode(mmdvm_frame &frame)
{
	if (frame.length != sizeof(frame.mode) + 3)
		return 4;

	MMDVM_STATE modemState = MMDVM_STATE(frame.mode);

	if (modemState == m_modemState)
		return 0;

	if (modemState != STATE_IDLE &&
	    modemState != STATE_DSTAR &&
	    modemState != STATE_DMR &&
	    modemState != STATE_YSF &&
	    modemState != STATE_P25 &&
	    modemState != STATE_NXDN &&
	    modemState != STATE_POCSAG &&
	    modemState != STATE_DSTARCAL &&
	    modemState != STATE_DMRCAL &&
	    modemState != STATE_RSSICAL &&
	    modemState != STATE_LFCAL &&
	    modemState != STATE_P25CAL1K &&
	    modemState != STATE_DMRDMO1K &&
	    modemState != STATE_NXDNCAL1K &&
		modemState != STATE_POCSAGCAL)
		return 4;
	if (modemState == STATE_DSTAR && !m_dstarEnable)
		return 4;
	if (modemState == STATE_DMR && !m_dmrEnable)
		return 4;
	if (modemState == STATE_YSF && !m_ysfEnable)
		return 4;
	if (modemState == STATE_P25 && !m_p25Enable)
		return 4;
	if (modemState == STATE_NXDN && !m_nxdnEnable)
		return 4;
	if (modemState == STATE_POCSAG && !m_pocsagEnable)
		return 4;

	setMode(modemState);

	return 0;
}

void CSerialPort::setMode(MMDVM_STATE modemState)
{
	switch (modemState) {
	case STATE_DMR:
		DEBUG1("Mode set to DMR");
		break;
	case STATE_DSTAR:
		DEBUG1("Mode set to D-Star");
		break;
	case STATE_YSF:
		DEBUG1("Mode set to System Fusion");
		break;
	case STATE_P25:
		DEBUG1("Mode set to P25");
		break;
	case STATE_NXDN:
		DEBUG1("Mode set to NXDN");
		break;
	case STATE_POCSAG:
		DEBUG1("Mode set to POCSAG");
		break;
	case STATE_DSTARCAL:
		DEBUG1("Mode set to D-Star Calibrate");
		break;
	case STATE_DMRCAL:
		DEBUG1("Mode set to DMR Calibrate");
		break;
	case STATE_RSSICAL:
		DEBUG1("Mode set to RSSI Calibrate");
		break;
	case STATE_LFCAL:
		DEBUG1("Mode set to 80 Hz Calibrate");
		break;
	case STATE_P25CAL1K:
		DEBUG1("Mode set to P25 1011 Hz Calibrate");
		break;
	case STATE_DMRDMO1K:
		DEBUG1("Mode set to DMR MS 1031 Hz Calibrate");
		break;
	case STATE_NXDNCAL1K:
		DEBUG1("Mode set to NXDN 1031 Hz Calibrate");
		break;
	case STATE_POCSAGCAL:
		DEBUG1("Mode set to POCSAG Calibrate");
		break;
	default:		// STATE_IDLE
		DEBUG1("Mode set to Idle");
		break;
	}

	if (modemState != STATE_DSTAR)
		dstarRX.reset();

	if (modemState != STATE_DMR)
		dmrDMORX.reset();

	if (modemState != STATE_YSF)
		ysfRX.reset();

	if (modemState != STATE_P25)
		p25RX.reset();

	if (modemState != STATE_NXDN)
		nxdnRX.reset();

	cwIdTX.reset();

	m_modemState = modemState;

	io.setMode();
}

bool CSerialPort::open() {
	assert(!m_ptyPath.empty());

	m_fd = ::posix_openpt(O_RDWR | O_NOCTTY | O_NDELAY);
	if (m_fd < 0) {
		::fprintf(stderr, "Cannot open pty master: %s\n", strerror(errno));
		return false;
	}

	if (::grantpt(m_fd) == -1 || ::unlockpt(m_fd) == -1) {
		::fprintf(stderr, "Error Initializing slave pty: %s\n", strerror(errno));
		return false;
	}

	char* pts_name = ::ptsname(m_fd);

	if (::unlink(m_ptyPath.c_str()) == -1)
		::fprintf(stderr, "Link does not exist: %s <> %s\n", pts_name, m_ptyPath.c_str());

	if ((::symlink(pts_name, m_ptyPath.c_str())) == -1) {
		::fprintf(stderr,"Error creating symlink from %s to %s\n", pts_name, m_ptyPath.c_str());
		return false;
	} else {
		::fprintf(stderr, "Virtual pty: %s <> %s\n", pts_name, m_ptyPath.c_str());
	}

	return true;
}


// XXX Probably need to look at this function a bit
int CSerialPort::write(const unsigned char* buffer, unsigned int length)
{
	assert(buffer != NULL);
	assert(m_fd != -1);

	if (length == 0U)
		return 0;

	unsigned int ptr = 0U;
	while (ptr < length) {
		ssize_t n = ::write(m_fd, buffer + ptr, length - ptr);
		if (n < 0) {
			if (errno != EAGAIN) {
				::fprintf(stderr, "Error returned from write(), errno=%d\n", errno);
				return -1;
			}
		}

		if (n > 0)
			ptr += n;
	}

	return length;
}


void CSerialPort::process()
{
	mmdvm_frame frame = { 0 };
	uint8_t err = 2;

	int read_bytes = ::read(m_fd, (void *) &frame, 2);
	if(read_bytes < 0) {
		if(errno == EAGAIN) {
			return;
		} else if(errno == EIO) {
			::fprintf(stderr, "Slave disconnected, reopening master\n");
			::close(m_fd);
			open();
			return;
		} else {
			::fprintf(stderr, "Couldn't read from pty: %s\n", strerror(errno));
			return;
		}
	}

	if(read_bytes == 0)
		return;

	//  This means that the start byte is *probably* off by one.  Increment it
	//  and find the length.
	if(frame.start_byte != MMDVM_FRAME_START &&
	   frame.length == MMDVM_FRAME_START) {
		frame.start_byte = MMDVM_FRAME_START;
		int read_bytes = ::read(m_fd, (void *) &(frame.length), 1);
		if(read_bytes != 1) {
			if(errno != EAGAIN)
				::fprintf(stderr, "Couldn't read from pty: %s\n", strerror(errno));
			return;
		}
	}

	if(frame.start_byte != MMDVM_FRAME_START)
		return;

	int to_read = frame.length - 2;

	uint8_t *insertion_point = (uint8_t *) &(frame.operation);
	while(to_read > 0) {
		read_bytes = ::read(m_fd, (char *) insertion_point, to_read);
		if(read_bytes < 0) {
			if(errno == -EAGAIN)
				continue;
			else {
				::fprintf(stderr, "Couldn't read from pty: %s\n", strerror(errno));
				continue;
			}
		}
		to_read -= read_bytes;
		insertion_point += read_bytes;
	}


	switch(frame.operation) {
	case MMDVM_GET_STATUS:
		getStatus();
		break;
	case MMDVM_GET_VERSION:
		getVersion();
		break;
	case MMDVM_SET_CONFIG:
		err = setConfig(frame);
		if (err == 0U)
			sendACK(frame);
		else
			sendNAK(frame, err);
		break;
	case MMDVM_SET_MODE:
		err = setMode(frame);
		if (err == 0U)
			sendACK(frame);
		else
			sendNAK(frame, err);
		break;

	case MMDVM_SET_FREQ:
		sendACK(frame);
		break;

	case MMDVM_CAL_DATA:
		if (m_modemState == STATE_DSTARCAL)
			err = calDStarTX.write(frame.data, frame.length - 3U);
		if (m_modemState == STATE_DMRCAL ||
		    m_modemState == STATE_LFCAL ||
		    m_modemState == STATE_DMRDMO1K)
			err = calDMR.write(frame.data, frame.length - 3U);
		if (m_modemState == STATE_P25CAL1K)
			err = calP25.write(frame.data, frame.length - 3U);
		if (m_modemState == STATE_NXDNCAL1K)
			err = calNXDN.write(frame.data, frame.length - 3U);
		if (err == 0U) {
			sendACK(frame);
		} else {
			DEBUG2("Received invalid calibration data", err);
			sendNAK(frame, err);
		}
		break;

	case MMDVM_SEND_CWID:
		err = 5;
		if (m_modemState == STATE_IDLE)
			err = cwIdTX.write(frame.data, frame.length - 3U);
		if (err != 0) {
			DEBUG2("Invalid CW Id data", err);
			sendNAK(frame, err);
		}
		break;


	case MMDVM_DSTAR_HEADER:
		if (m_dstarEnable) {
			if (m_modemState == STATE_IDLE || m_modemState == STATE_DSTAR)
				err = dstarTX.writeHeader(frame.data, frame.length - 3);
		}
		if (err == 0U) {
			if (m_modemState == STATE_IDLE)
				setMode(STATE_DSTAR);
		} else {
			DEBUG2("Received invalid D-Star header", err);
			sendNAK(frame, err);
		}
		break;

	case MMDVM_DSTAR_DATA:
		if (m_dstarEnable) {
			if (m_modemState == STATE_IDLE || m_modemState == STATE_DSTAR)
				err = dstarTX.writeData(frame.data, frame.length - 3);
		}
		if (err == 0U) {
			if (m_modemState == STATE_IDLE)
				setMode(STATE_DSTAR);
		} else {
			DEBUG2("Received invalid D-Star data", err);
			sendNAK(frame, err);
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
			sendNAK(frame, err);
		}
		break;

	case MMDVM_DMR_DATA2:
		if (m_dmrEnable) {
			if (m_modemState == STATE_IDLE || m_modemState == STATE_DMR)
				err = dmrDMOTX.writeData(frame.data, frame.length - 3);
		}
		if (err == 0U) {
			if (m_modemState == STATE_IDLE)
				setMode(STATE_DMR);
		} else {
			DEBUG2("Received invalid DMR data", err);
			sendNAK(frame, err);
		}
		break;

	case MMDVM_YSF_DATA:
		if (m_ysfEnable) {
			if (m_modemState == STATE_IDLE || m_modemState == STATE_YSF)
				err = ysfTX.writeData(frame.data, frame.length - 3);
		}
		if (err == 0U) {
			if (m_modemState == STATE_IDLE)
				setMode(STATE_YSF);
		} else {
			DEBUG2("Received invalid System Fusion data", err);
			sendNAK(frame, err);
		}
		break;

	case MMDVM_P25_HDR:
		if (m_p25Enable) {
			if (m_modemState == STATE_IDLE || m_modemState == STATE_P25)
				err = p25TX.writeData(frame.data, frame.length - 3);
		}
		if (err == 0U) {
			if (m_modemState == STATE_IDLE)
				setMode(STATE_P25);
		} else {
			DEBUG2("Received invalid P25 header", err);
			sendNAK(frame, err);
		}
		break;

	case MMDVM_P25_LDU:
		if (m_p25Enable) {
			if (m_modemState == STATE_IDLE || m_modemState == STATE_P25)
				err = p25TX.writeData(frame.data, frame.length - 3);
		}
		if (err == 0U) {
			if (m_modemState == STATE_IDLE)
				setMode(STATE_P25);
		} else {
			DEBUG2("Received invalid P25 LDU", err);
			sendNAK(frame, err);
		}
		break;

	case MMDVM_NXDN_DATA:
		if (m_nxdnEnable) {
			if (m_modemState == STATE_IDLE || m_modemState == STATE_NXDN)
				err = nxdnTX.writeData(frame.data, frame.length - 3);
		}
		if (err == 0U) {
			if (m_modemState == STATE_IDLE)
				setMode(STATE_NXDN);
		} else {
			DEBUG2("Received invalid NXDN data", err);
			sendNAK(frame, err);
		}
		break;

	case MMDVM_POCSAG_DATA:
		if (m_pocsagEnable) {
			if (m_modemState == STATE_IDLE || m_modemState == STATE_POCSAG)
				err = pocsagTX.writeData(frame.data, frame.length - 3);
		}
		if (err == 0U) {
			if (m_modemState == STATE_IDLE)
				setMode(STATE_POCSAG);
		} else {
			DEBUG2("Received invalid POCSAG data", err);
			sendNAK(frame, err);
		}
		break;

	case MMDVM_TRANSPARENT:
		// Do nothing on the MMDVM.
		break;

	default:
		::fprintf(stderr, "Got operation %d\n", frame.operation);
		sendNAK(frame, 1);
		break;
	}

	// XXX Evaluate this.  Don't think this is needed.
	if (io.getWatchdog() >= 48000U) {
		m_ptr = 0U;
		m_len = 0U;
	}
}

inline void CSerialPort::writeSingleByteReply(const uint8_t reply) {
	uint8_t reply_frame[] = {
		MMDVM_FRAME_START,
		3,
		reply
	};

	write(reply_frame, 3);
}

void CSerialPort::writeDStarLost() {
	if (!m_dstarEnable ||
	    (m_modemState != STATE_DSTAR && m_modemState != STATE_IDLE))
		return;

	writeSingleByteReply(MMDVM_DSTAR_LOST);
}

void CSerialPort::writeDStarEOT() {
	if (!m_dstarEnable ||
	    (m_modemState != STATE_DSTAR && m_modemState != STATE_IDLE))
		return;

	writeSingleByteReply(MMDVM_DSTAR_EOT);
}

void CSerialPort::writeDataFrame(const uint8_t operation, const uint8_t *data, uint8_t length) {
	uint8_t *frame = (uint8_t *) malloc(length + 3);
	*frame = MMDVM_FRAME_START;
	*(frame + 1) = length + 3;
	*(frame + 2) = operation;

	memcpy (frame + 3, data, length);

	write(frame, length + 3);

	free(frame);
}


void CSerialPort::writeDStarHeader(const uint8_t* header, uint8_t length) {
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

	write(reply, count);
}

void CSerialPort::writeDStarData(const uint8_t* data, uint8_t length) {
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

	write(reply, count);
}

void CSerialPort::writeDMRData(bool slot, const uint8_t* data, uint8_t length) {
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

	write(reply, count);
}

void CSerialPort::writeDMRLost(bool slot) {
	if (!m_dmrEnable ||
	    (m_modemState != STATE_DMR && m_modemState != STATE_IDLE))
		return;

	writeSingleByteReply(slot ? MMDVM_DMR_LOST2 : MMDVM_DMR_LOST1);
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

	write(reply, count);
}

void CSerialPort::writeYSFLost() {
	if (!m_ysfEnable ||
	    (m_modemState != STATE_YSF && m_modemState != STATE_IDLE))
		return;

	writeSingleByteReply(MMDVM_YSF_LOST);
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

	write(reply, count);
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

	write(reply, count);
}

void CSerialPort::writeP25Lost() {
	if (!m_p25Enable ||
	    (m_modemState != STATE_P25 && m_modemState != STATE_IDLE))
		return;

	writeSingleByteReply(MMDVM_P25_LOST);
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

	write(reply, count);
}

void CSerialPort::writeNXDNLost() {
	if (!m_nxdnEnable ||
	    (m_modemState != STATE_NXDN && m_modemState != STATE_IDLE))
		return;

	writeSingleByteReply(MMDVM_NXDN_LOST);
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

	write(reply, count);
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

	write(reply, count);
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

	write(reply, count);
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

	write(reply, count);
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

	write(reply, count);
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

	write(reply, count);
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

	write(reply, count);
}

/* Need Close Function */
