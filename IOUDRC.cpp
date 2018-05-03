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

#include "Globals.h"
#include "IO.h"

#include <wiringPi.h>

const int BASE_PIN        = 6;
const int DUPLEX_PTT_PIN  = 12;
const int SIMPLEX_PTT_PIN = 23;
const int PKSQL_PIN       = 5;
const int SQL_PIN         = 25;

void CIO::initInt()
{
  ::wiringPiSetup();

  ::pinMode(PKSQL_PIN, INPUT);

  // Set pull ups on the input pins
  ::pullUpDnControl(PKSQL_PIN, PUD_UP);

  ::pinMode(DUPLEX_PTT_PIN,  OUTPUT);
  ::pinMode(SIMPLEX_PTT_PIN, OUTPUT);
  ::pinMode(BASE_PIN,        OUTPUT);
}

void CIO::startInt()
{
  if (m_pttInvert) {
    digitalWrite(DUPLEX_PTT_PIN,  LOW);
    digitalWrite(SIMPLEX_PTT_PIN, LOW);
  } else {
    digitalWrite(DUPLEX_PTT_PIN,  HIGH);
    digitalWrite(SIMPLEX_PTT_PIN, HIGH);
  }
}

bool CIO::getCOSInt()
{
  return digitalRead(PKSQL_PIN) == LOW;
}

void CIO::setLEDInt(bool on)
{
  digitalWrite(BASE_PIN, on ? LOW : HIGH);
}

void CIO::setPTTInt(bool on)
{
  if (m_pttInvert) {
    if (m_duplex)
      digitalWrite(DUPLEX_PTT_PIN, on ? HIGH : LOW);
    else
      digitalWrite(SIMPLEX_PTT_PIN, on ? HIGH : LOW);
  } else {
    if (m_duplex)
      digitalWrite(DUPLEX_PTT_PIN, on ? LOW : HIGH);
    else
      digitalWrite(SIMPLEX_PTT_PIN, on ? LOW : HIGH);
  }
}

void CIO::setCOSInt(bool on)
{
}

