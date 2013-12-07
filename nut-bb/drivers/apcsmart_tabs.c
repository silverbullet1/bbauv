/* apcsmart_tabs.c - common tables for APC smart protocol units
 *
 * Copyright (C) 1999  Russell Kroll <rkroll@exploits.org>
 *           (C) 2000  Nigel Metheringham <Nigel.Metheringham@Intechnology.co.uk>
 *           (C) 2011  Michal Soltys <soltys@ziu.info>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include "apcsmart_tabs.h"

/* APC_MULTI variables *must* be listed in order of preference */
apc_vartab_t apc_vartab[] = {

	{ "ups.temperature",		'C',	APC_POLL|APC_F_CELSIUS },
	{ "ups.load",			'P',	APC_POLL|APC_F_PERCENT },
	{ "ups.test.interval",		'E',	APC_F_HOURS },
	{ "ups.test.result",		'X',	APC_POLL },
	{ "ups.delay.start",		'r',	APC_F_SECONDS },
	{ "ups.delay.shutdown",		'p',	APC_F_SECONDS },
	{ "ups.id",			'c',	APC_STRING },
	{ "ups.contacts",		'i',	APC_POLL|APC_F_HEX },
	{ "ups.display.language",	'\014',	0 },
	{ "input.voltage",		'L',	APC_POLL|APC_F_VOLT },
	{ "input.frequency",		'F',	APC_POLL|APC_F_DEC },
	{ "input.sensitivity",		's',	0 },
	{ "input.quality",		'9',	APC_POLL|APC_F_HEX },
	{ "input.transfer.low",		'l',	APC_F_VOLT },
	{ "input.transfer.high",	'u',	APC_F_VOLT },
	{ "input.transfer.reason",	'G',	APC_POLL|APC_F_REASON },
	{ "input.voltage.maximum",	'M',	APC_POLL|APC_F_VOLT },
	{ "input.voltage.minimum",	'N',	APC_POLL|APC_F_VOLT },
	{ "output.current",		'/',	APC_POLL|APC_F_AMP },
	{ "output.voltage",		'O',	APC_POLL|APC_F_VOLT },
	{ "output.voltage.nominal",	'o',	APC_F_VOLT },
	{ "ambient.humidity",		'h',	APC_POLL|APC_F_PERCENT },
	{ "ambient.humidity.high",	'{',	APC_F_PERCENT },
	{ "ambient.humidity.low",	'}',	APC_F_PERCENT },
	{ "ambient.temperature",	't',	APC_POLL|APC_F_CELSIUS },
	{ "ambient.temperature.high",	'[',	APC_F_CELSIUS },
	{ "ambient.temperature.low",	']',	APC_F_CELSIUS },
	{ "battery.date",		'x',	APC_STRING },
	{ "battery.charge",		'f',	APC_POLL|APC_F_PERCENT },
	{ "battery.charge.restart",	'e',	APC_F_PERCENT },
	{ "battery.voltage",		'B',	APC_POLL|APC_F_VOLT },
	{ "battery.voltage.nominal",	'g',	0 },
	{ "battery.runtime",		'j',	APC_POLL|APC_F_MINUTES },
	{ "battery.runtime.low",	'q',	APC_F_MINUTES },
	{ "battery.packs",		'>',	APC_F_DEC },
	{ "battery.packs.bad",		'<',	APC_F_DEC },
	{ "battery.alarm.threshold",	'k',	0 },
	{ "ups.serial",			'n',	0 },
	{ "ups.mfr.date",		'm',	0 },
	{ "ups.model",			'\001',	0 },
	{ "ups.firmware.aux",		'v',	0 },
	{ "ups.firmware",		'b',	APC_MULTI },
	{ "ups.firmware",		'V',	APC_MULTI },

	{ 0, 0, 0 }
	/* todo:

	   I = alarm enable (hex field) - split into alarm.n.enable
	   J = alarm status (hex field) - split into alarm.n.status

	0x15 = output voltage selection (APC_F_VOLT)
	0x5C = load power (APC_POLL|APC_F_PERCENT)

	 */
};

/*
 * apc commands mapped to nut's instant commands extra values are either
 * exactly 2-char prefix, or longer than 2-char extended regex
 */
apc_cmdtab_t apc_cmdtab[] = {
	{ "test.panel.start",	0,	APC_CMD_FPTEST,		0 },
	{ "test.failure.start",	0,	APC_CMD_SIMPWF,		0 },
	{ "test.battery.start",	0,	APC_CMD_BTESTTOGGLE,	0 },
	{ "test.battery.stop",	0,	APC_CMD_BTESTTOGGLE,	0 },
	{ "shutdown.return",	"^at:[0-9]{1,3}$",
				APC_CMD_GRACEDOWN,	APC_NASTY },
	{ "shutdown.return",	"cs",	APC_CMD_SOFTDOWN,	APC_NASTY },
	{ "shutdown.return",	0,	APC_CMD_SOFTDOWN,	APC_NASTY },
	{ "shutdown.stayoff",	0,	APC_CMD_SHUTDOWN,	APC_NASTY|APC_REPEAT },
	{ "load.off",		0,	APC_CMD_OFF,		APC_NASTY|APC_REPEAT },
	{ "load.on",		0,	APC_CMD_ON,		APC_REPEAT },
	{ "bypass.start",	0,	APC_CMD_BYPTOGGLE,	0 },
	{ "bypass.stop",	0,	APC_CMD_BYPTOGGLE,	0 },
	{ "calibrate.start",	0,	APC_CMD_CALTOGGLE,	0 },
	{ "calibrate.stop",	0,	APC_CMD_CALTOGGLE,	0 },

	{ 0, 0, 0, 0 }
};

/* compatibility with hardware that doesn't do APC_CMDSET ('a') */
apc_compattab_t apc_compattab[] = {
	/* APC Matrix */
	{ "0XI",	"@789ABCDEFGKLMNOPQRSTUVWXYZcefgjklmnopqrsuwxz/<>\\^\014\026", 0 },
	{ "0XM",	"@789ABCDEFGKLMNOPQRSTUVWXYZcefgjklmnopqrsuwxz/<>\\^\014\026", 0 },
	{ "0ZI",	"@79ABCDEFGKLMNOPQRSUVWXYZcefgjklmnopqrsuxz/<>", 0 },
	{ "5UI",	"@79ABCDEFGKLMNOPQRSUVWXYZcefgjklmnopqrsuxz/<>", 0 },
	{ "5ZM",	"@79ABCDEFGKLMNOPQRSUVWXYZcefgjklmnopqrsuxz/<>", 0 },
	/* APC600 */
	{ "6QD",	"@79ABCDEFGKLMNOPQRSUVWXYZcefgjklmnopqrsuxz", 0 },
	{ "6QI",	"@79ABCDEFGKLMNOPQRSUVWXYZcefgjklmnopqrsuxz", 0 },
	{ "6TD",	"@79ABCDEFGKLMNOPQRSUVWXYZcefgjklmnopqrsuxz", 0 },
	{ "6TI",	"@79ABCDEFGKLMNOPQRSUVWXYZcefgjklmnopqrsuxz", 0 },
	/* SmartUPS 900 */
	{ "7QD",	"@79ABCDEFGKLMNOPQRSUVWXYZcefgjklmnopqrsuxz", 0 },
	{ "7QI",	"@79ABCDEFGKLMNOPQRSUVWXYZcefgjklmnopqrsuxz", 0 },
	{ "7TD",	"@79ABCDEFGKLMNOPQRSUVWXYZcefgjklmnopqrsuxz", 0 },
	{ "7TI",	"@79ABCDEFGKLMNOPQRSUVWXYZcefgjklmnopqrsuxz", 0 },
	/* SmartUPS 900I */
	{ "7II",	"@79ABCEFGKLMNOPQSUVWXYZcfg", 0 },
	/* SmartUPS 2000I */
	{ "9II",	"@79ABCEFGKLMNOPQSUVWXYZcfg", 0 },
	{ "9GI",	"@79ABCEFGKLMNOPQSUVWXYZcfg", 0 },
	/* SmartUPS 1250 */
	{ "8QD",	"@79ABCDEFGKLMNOPQRSUVWXYZcefgjklmnopqrsuxz", 0 },
	{ "8QI",	"@79ABCDEFGKLMNOPQRSUVWXYZcefgjklmnopqrsuxz", 0 },
	{ "8TD",	"@79ABCDEFGKLMNOPQRSUVWXYZcefgjklmnopqrsuxz", 0 },
	{ "8TI",	"@79ABCDEFGKLMNOPQRSUVWXYZcefgjklmnopqrsuxz", 0 },
	/* CS 350 */
	{ "5.4.D",	"@\1ABPQRSUYbdfgjmnx9",	0 },
	/* Smart-UPS 600 */
	{  "D9",	"@789ABCEFGKLMNOPQRSUVWXYZ", 0 },
	{  "D8",	"@789ABCEFGKLMNOPQRSUVWXYZ", 0 },
	{  "D7",	"@789ABCEFGKLMNOPQRSUVWXYZ", 0 },
	{  "D6",	"@789ABCEFGKLMNOPQRSUVWXYZ", 0 },
	{  "D5",	"@789ABCEFGKLMNOPQRSUVWXYZ", 0 },
	{  "D4",	"@789ABCEFGKLMNOPQRSUVWXYZ", 0 },

	{ 0, 0, 0 }
};

upsdrv_info_t apc_tab_info = {
	"APC command table",
	APC_TABLE_VERSION,
	0,
	0,
	{ 0 }
};

