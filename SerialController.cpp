/*
 *   Copyright (C) 2002-2004,2007-2011,2013,2014-2019 by Jonathan Naylor G4KLX
 *   Copyright (C) 1999-2001 by Thomas Sailor HB9JNX
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

#include "SerialController.h"

#include <cstring>
#include <cassert>

#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <cerrno>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>


CSerialController::CSerialController() :
m_fd(-1)
{
}

CSerialController::~CSerialController()
{
}

bool CSerialController::open(const std::string& device, SERIAL_SPEED speed, bool assertRTS, const std::string& path)
{
	assert(!device.empty());
	assert(!path.empty());
	assert(m_fd == -1);

	m_fd = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NDELAY, 0);
	if (m_fd < 0) {
		::fprintf(stderr, "Cannot open device - %s\n", device.c_str());
		return false;
	}

	if (::isatty(m_fd)) {
		if (::grantpt(m_fd) == -1) {
			::fprintf(stderr, "Error granting pseudotty to : %s\n", device.c_str());
			return false;
		}

		if (::unlockpt(m_fd) == -1) {
			::fprintf(stderr, "Error unlocking pseudotty for : %s\n", device.c_str());
			return false;
		}

		char* pts_name = ::ptsname(m_fd);

		if (::unlink(path.c_str()) == -1)
			::fprintf(stderr, "Link does not exist: %s <> %s\n", pts_name, path.c_str());

		if ((::symlink(pts_name, path.c_str())) == -1) {
			::fprintf(stderr,"Error creating symlink from %s to %s\n", pts_name, path.c_str());
			return false;
		} else {
			::fprintf(stderr, "Virtual pty: %s <> %s\n", pts_name, path.c_str());
		}

		termios termios;
		if (::tcgetattr(m_fd, &termios) < 0) {
			::fprintf(stderr, "Cannot get the attributes for %s\n", device.c_str());
			::close(m_fd);
			return false;
		}

		termios.c_lflag    &= ~(ECHO | ECHOE | ICANON | IEXTEN | ISIG);
		termios.c_iflag    &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON | IXOFF | IXANY);
		termios.c_cflag    &= ~(CSIZE | CSTOPB | PARENB | CRTSCTS);
		termios.c_cflag    |= CS8;
		termios.c_oflag    &= ~(OPOST);
		termios.c_cc[VMIN]  = 0;
		termios.c_cc[VTIME] = 10;

		switch (speed) {
			case SERIAL_1200:
				::cfsetospeed(&termios, B1200);
				::cfsetispeed(&termios, B1200);
				break;
			case SERIAL_2400:
				::cfsetospeed(&termios, B2400);
				::cfsetispeed(&termios, B2400);
				break;
			case SERIAL_4800:
				::cfsetospeed(&termios, B4800);
				::cfsetispeed(&termios, B4800);
				break;
			case SERIAL_9600:
				::cfsetospeed(&termios, B9600);
				::cfsetispeed(&termios, B9600);
				break;
			case SERIAL_19200:
				::cfsetospeed(&termios, B19200);
				::cfsetispeed(&termios, B19200);
				break;
			case SERIAL_38400:
				::cfsetospeed(&termios, B38400);
				::cfsetispeed(&termios, B38400);
				break;
			case SERIAL_115200:
				::cfsetospeed(&termios, B115200);
				::cfsetispeed(&termios, B115200);
				break;
			case SERIAL_230400:
				::cfsetospeed(&termios, B230400);
				::cfsetispeed(&termios, B230400);
				break;
			default:
				::fprintf(stderr, "Unsupported serial port speed - %d\n", int(speed));
				::close(m_fd);
				return false;
		}

		if (::tcsetattr(m_fd, TCSANOW, &termios) < 0) {
			::fprintf(stderr, "Cannot set the attributes for %s\n", device.c_str());
			::close(m_fd);
			return false;
		}

		if (assertRTS) {
			unsigned int y;
			if (::ioctl(m_fd, TIOCMGET, &y) < 0) {
				::fprintf(stderr, "Cannot get the control attributes for %s\n", device.c_str());
				::close(m_fd);
				return false;
			}

			y |= TIOCM_RTS;
                                                                                
			if (::ioctl(m_fd, TIOCMSET, &y) < 0) {
				::fprintf(stderr, "Cannot set the control attributes for %s\n", device.c_str());
				::close(m_fd);
				return false;
			}
		}
	}

	return true;
}

int CSerialController::read(unsigned char* buffer, unsigned int length)
{
	assert(buffer != NULL);
	assert(m_fd != -1);

	if (length == 0U)
		return 0;

	unsigned int offset = 0U;

	while (offset < length) {
		fd_set fds;
		FD_ZERO(&fds);
		FD_SET(m_fd, &fds);
		int n;
		if (offset == 0U) {
			struct timeval tv;
			tv.tv_sec  = 0;
			tv.tv_usec = 0;
			n = ::select(m_fd + 1, &fds, NULL, NULL, &tv);
			if (n == 0)
				return 0;
		} else {
			n = ::select(m_fd + 1, &fds, NULL, NULL, NULL);
		}

		if (n < 0) {
			::fprintf(stderr, "Error from select(), errno=%d\n", errno);
			return -1;
		}

		if (n > 0) {
			ssize_t len = ::read(m_fd, buffer + offset, length - offset);
			if (len < 0) {
				if (errno != EAGAIN) {
					::fprintf(stderr, "Error from read(), errno=%d\n", errno);
					return -1;
				}
			}

			if (len > 0)
				offset += len;
		}
	}

	return length;
}

int CSerialController::write(const unsigned char* buffer, unsigned int length)
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

void CSerialController::close()
{
	assert(m_fd != -1);

	::close(m_fd);
	m_fd = -1;
}

