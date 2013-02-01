/* openups-hid.h - subdriver to monitor openUPS USB/HID devices with NUT
 *
 *  Copyright (C)
 *  2003 - 2009	Arnaud Quette <ArnaudQuette@Eaton.com>
 *  2005 - 2006	Peter Selinger <selinger@users.sourceforge.net>
 *  2008 - 2009	Arjen de Korte <adkorte-guest@alioth.debian.org>
 *         2012	Nicu Pavel <npavel@mini-box.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#ifndef OPENUPS_HID_H
#define OPENUPS_HID_H

#include "usbhid-ups.h"

/* constants for converting HID read values to real values */
static const float vin_scale = 0.03545 * 100;
static const float vout_scale = 0.02571 * 100;
static const float vbat_scale = 0.00857 * 100;
static const float ccharge_scale = 0.8274 / 10;
static const float cdischarge_scale = 16.113 / 10;

/* Thermistor table used for temperature lookups 
 * taken from the windows monitoring application
 */
static unsigned int therm_tbl[] = 
{ 
	(unsigned int)0x31,
	(unsigned int)0x40,
	(unsigned int)0x53,
	(unsigned int)0x68,
	(unsigned int)0x82,
	(unsigned int)0xA0,
	(unsigned int)0xC3,
	(unsigned int)0xE9,
	(unsigned int)0x113,
	(unsigned int)0x13F,
	(unsigned int)0x16E,
	(unsigned int)0x19F,
	(unsigned int)0x1CF,
	(unsigned int)0x200,
	(unsigned int)0x22F,
	(unsigned int)0x25C,
	(unsigned int)0x286,
	(unsigned int)0x2AE,
	(unsigned int)0x2D3,
	(unsigned int)0x2F4,
	(unsigned int)0x312,
	(unsigned int)0x32D,
	(unsigned int)0x345,
	(unsigned int)0x35A,
	(unsigned int)0x36D,
	(unsigned int)0x37E,
	(unsigned int)0x38C,
	(unsigned int)0x399,
	(unsigned int)0x3A5,
	(unsigned int)0x3AF,
	(unsigned int)0x3B7,
	(unsigned int)0x3BF,
	(unsigned int)0x3C6,
	(unsigned int)0x3CC
};

static unsigned int therm_tbl_size = sizeof(therm_tbl)/sizeof(therm_tbl[0]);

extern subdriver_t openups_subdriver;

#endif /* OPENUPS_HID_H */
