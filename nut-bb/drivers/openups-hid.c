/* openups-hid.c - subdriver to monitor openUPS USB/HID devices with NUT
 *
 *  Copyright (C)
 *  2003 - 2012	Arnaud Quette <ArnaudQuette@Eaton.com>
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

#include "usbhid-ups.h"
#include "openups-hid.h"
#include "main.h"		/* for getval() */
#include "usb-common.h"

#define OPENUPS_HID_VERSION	"openUPS HID 0.1"
#define OPENUPS_VENDORID	0x04d8

static char openups_scratch_buf[20];

/* USB IDs device table */
static usb_device_id_t openups_usb_device_table[] = {
	/* openUPS  minimum required firmware 1.4 */
	{USB_DEVICE(OPENUPS_VENDORID, 0xd004), NULL},

	/* Terminating entry */
	{-1, -1, NULL}
};

static const char *openups_charging_fun(double value);
static const char *openups_discharging_fun(double value);
static const char *openups_online_fun(double value);
static const char *openups_nobattery_fun(double value);
static const char *openups_off_fun(double value);

static const char *openups_scale_vin_fun(double value);
static const char *openups_scale_vout_fun(double value);
static const char *openups_scale_vbat_fun(double value);
static const char *openups_scale_ccharge_fun(double value);
static const char *openups_scale_cdischarge_fun(double value);
static const char *openups_temperature_fun(double value);

static info_lkp_t openups_charging_info[] = {
	{0, NULL, openups_charging_fun}
};

static info_lkp_t openups_discharging_info[] = {
	{0, NULL, openups_discharging_fun}
};

static info_lkp_t openups_online_info[] = {
	{0, NULL, openups_online_fun}
};

static info_lkp_t openups_nobattery_info[] = {
	{0, NULL, openups_nobattery_fun}
};

static info_lkp_t openups_off_info[] = {
	{0, NULL, openups_off_fun}
};

static info_lkp_t openups_vin_info[] = {
	{0, NULL, openups_scale_vin_fun}
};

static info_lkp_t openups_vout_info[] = {
	{0, NULL, openups_scale_vout_fun}
};

static info_lkp_t openups_vbat_info[] = {
	{0, NULL, openups_scale_vbat_fun}
};

static info_lkp_t openups_ccharge_info[] = {
	{0, NULL, openups_scale_ccharge_fun}
};

static info_lkp_t openups_cdischarge_info[] = {
	{0, NULL, openups_scale_cdischarge_fun}
};

static info_lkp_t openups_temperature_info[] = {
	{0, NULL, openups_temperature_fun}
};

static const char *openups_charging_fun(double value)
{
	return value ? "chrg" : "!chrg";
}

static const char *openups_discharging_fun(double value)
{
	return value ? "dischrg" : "!dischrg";
}

static const char *openups_online_fun(double value)
{
	return value ? "online" : "!online";
}

static const char *openups_nobattery_fun(double value)
{
	return value ? "nobattery" : "!nobattery";
}

static const char *openups_off_fun(double value)
{
	return value ? "!off" : "off";
}

static const char *openups_scale_vin_fun(double value)
{
	snprintf(openups_scratch_buf, sizeof(openups_scratch_buf), "%.2f", value * vin_scale);
	return openups_scratch_buf;
}

static const char *openups_scale_vout_fun(double value)
{
	snprintf(openups_scratch_buf, sizeof(openups_scratch_buf), "%.2f", value * vout_scale);
	return openups_scratch_buf;
}

static const char *openups_scale_vbat_fun(double value)
{
	snprintf(openups_scratch_buf, sizeof(openups_scratch_buf), "%.2f", value * vbat_scale);
	return openups_scratch_buf;
}

static const char *openups_scale_ccharge_fun(double value)
{
	snprintf(openups_scratch_buf, sizeof(openups_scratch_buf), "%.3f", value * ccharge_scale);
	return openups_scratch_buf;
}

static const char *openups_scale_cdischarge_fun(double value)
{
	snprintf(openups_scratch_buf, sizeof(openups_scratch_buf), "%.3f", value * cdischarge_scale);
	return openups_scratch_buf;
}

static const char *openups_temperature_fun(double value)
{
	int i;
	int pos = -1;
	unsigned int thermistor = value * 100;

	if (thermistor <= therm_tbl[0]) {
		snprintf(openups_scratch_buf, sizeof(openups_scratch_buf), "%d", -40);
	} else {
		if (thermistor >= therm_tbl[therm_tbl_size - 1]) {
			snprintf(openups_scratch_buf, sizeof(openups_scratch_buf), "%d", 125);
		} else {
			for (i = therm_tbl_size - 1; i >= 0; i--) {
				if (thermistor >= therm_tbl[i]) {
					pos = i;
					break;
				}
			}

			if (thermistor == therm_tbl[pos]) {
				snprintf(openups_scratch_buf, sizeof(openups_scratch_buf), "%d", pos * 5 - 40);
			} else {
				int t1 = pos * 5 - 40;
				int t2 = (pos + 1) * 5 - 40;

				unsigned int d1 = therm_tbl[pos];
				unsigned int d2 = therm_tbl[pos + 1];

				float temp = (float) (thermistor - d1) * (t2 - t1) / (d2 - d1) + t1;
				snprintf(openups_scratch_buf, sizeof(openups_scratch_buf), "%.2f", temp);
			}
		}
	}

	return openups_scratch_buf;
}

/* --------------------------------------------------------------- */
/*      Vendor-specific usage table */
/* --------------------------------------------------------------- */

/* OPENUPS usage table */
static usage_lkp_t openups_usage_lkp[] = {
	{"Cell1", 0x00000001},	/* Battery cell 1 on J6 pin 1 */
	{"Cell2", 0x00000002},	/* Battery cell 2 on J6 pin 2 */
	{"Cell3", 0x00000003},	/* Battery cell 3 on J6 pin 3 */
	{"Cell4", 0x00000004},	/* Battery cell 4 on J6 pin 4 */
	{"Cell5", 0x00000005},	/* Battery cell 5 on J6 pin 5 */
	{"Cell6", 0x00000006},	/* Battery cell 6 on J4 pin 1 */
	/* Usage table for windows monitoring app only updates when 
	 * certain request codes are written to USB endpoint */
	/*{ "OpenUPSExtra", 0xff000001 }, */
	{NULL, 0}
};

static usage_tables_t openups_utab[] = {
	openups_usage_lkp,
	hid_usage_lkp,
	NULL,
};

/* --------------------------------------------------------------- */
/* HID2NUT lookup table                                            */
/* --------------------------------------------------------------- */

static hid_info_t openups_hid2nut[] = {
	{"ups.serial", 0, 0, "UPS.PowerSummary.iSerialNumber", NULL, "%s", 0, stringid_conversion},

	/* Battery */
	{"battery.type", 0, 0, "UPS.PowerSummary.iDeviceChemistry", NULL, "%s", HU_FLAG_STATIC, stringid_conversion},
	{"battery.mfr.date", 0, 0, "UPS.PowerSummary.iOEMInformation", NULL, "%s", 0, stringid_conversion},
	{"battery.voltage", 0, 0, "UPS.PowerSummary.Voltage", NULL, "%.2f", HU_FLAG_QUICK_POLL, NULL},
	/* { "battery.voltage.nominal", 0, 0, "UPS.PowerSummary.ConfigVoltage", NULL, NULL, HU_FLAG_QUICK_POLL, openups_vbat_info }, */
	{"battery.current", 0, 0, "UPS.PowerSummary.Current", NULL, "%.3f", HU_FLAG_QUICK_POLL, NULL},
	{"battery.capacity", 0, 0, "UPS.PowerSummary.DesignCapacity", NULL, "%.0f", HU_FLAG_STATIC, NULL},
	{"battery.charge", 0, 0, "UPS.PowerSummary.RemainingCapacity", NULL, "%.0f", HU_FLAG_QUICK_POLL, NULL},
	{"battery.charge.low", 0, 0, "UPS.PowerSummary.RemainingCapacityLimit", NULL, "%.0f", HU_FLAG_QUICK_POLL, NULL},
	{"battery.charge.warning", 0, 0, "UPS.PowerSummary.WarningCapacityLimit", NULL, "%.0f", 0, NULL},
	{"battery.runtime", 0, 0, "UPS.PowerSummary.RunTimeToEmpty", NULL, "%.0f", HU_FLAG_QUICK_POLL, NULL},
	{"battery.temperature", 0, 0, "UPS.PowerSummary.Temperature", NULL, NULL, HU_FLAG_QUICK_POLL, openups_temperature_info},
	{"battery.cell1.voltage", 0, 0, "UPS.PowerSummary.Battery.Cell1", NULL, NULL, HU_FLAG_QUICK_POLL, openups_vbat_info},
	{"battery.cell2.voltage", 0, 0, "UPS.PowerSummary.Battery.Cell2", NULL, NULL, HU_FLAG_QUICK_POLL, openups_vbat_info},
	{"battery.cell3.voltage", 0, 0, "UPS.PowerSummary.Battery.Cell3", NULL, NULL, HU_FLAG_QUICK_POLL, openups_vbat_info},
	{"battery.cell4.voltage", 0, 0, "UPS.PowerSummary.Battery.Cell4", NULL, NULL, HU_FLAG_QUICK_POLL, openups_vbat_info},
	{"battery.cell5.voltage", 0, 0, "UPS.PowerSummary.Battery.Cell5", NULL, NULL, HU_FLAG_QUICK_POLL, openups_vbat_info},
	{"battery.cell6.voltage", 0, 0, "UPS.PowerSummary.Battery.Cell6", NULL, NULL, HU_FLAG_QUICK_POLL, openups_vbat_info},

	/* Output */
	{"output.voltage", 0, 0, "UPS.PowerSummary.Output.Voltage", NULL, NULL, HU_FLAG_QUICK_POLL, openups_vout_info},
	{"output.current", 0, 0, "UPS.PowerSummary.Output.Current", NULL, NULL, HU_FLAG_QUICK_POLL, openups_cdischarge_info},

	/* Input */
	{"input.voltage", 0, 0, "UPS.PowerSummary.Input.Voltage", NULL, NULL, HU_FLAG_QUICK_POLL, openups_vin_info},
	{"input.current", 0, 0, "UPS.PowerSummary.Input.Current", NULL, NULL, HU_FLAG_QUICK_POLL, openups_ccharge_info},

	/* Status */
	{"BOOL", 0, 0, "UPS.PowerSummary.PresentStatus.Good", NULL, NULL, HU_FLAG_QUICK_POLL, openups_off_info},
	{"BOOL", 0, 0, "UPS.PowerSummary.PresentStatus.InternalFailure", NULL, NULL, HU_FLAG_QUICK_POLL, commfault_info},
	{"BOOL", 0, 0, "UPS.PowerSummary.PresentStatus.Overload", NULL, NULL, HU_FLAG_QUICK_POLL, overload_info},
	{"BOOL", 0, 0, "UPS.PowerSummary.PresentStatus.OverTemperature", NULL, NULL, HU_FLAG_QUICK_POLL, overheat_info},
	{"BOOL", 0, 0, "UPS.PowerSummary.PresentStatus.ShutdownImminent", NULL, NULL, HU_FLAG_QUICK_POLL, shutdownimm_info},
	{"BOOL", 0, 0, "UPS.PowerSummary.PresentStatus.BelowRemainingCapacityLimit", NULL, NULL, HU_FLAG_QUICK_POLL, lowbatt_info},
	{"BOOL", 0, 0, "UPS.PowerSummary.PresentStatus.RemainingTimeLimitExpired", NULL, NULL, HU_FLAG_QUICK_POLL, timelimitexpired_info},
	{"BOOL", 0, 0, "UPS.PowerSummary.PresentStatus.Charging", NULL, NULL, HU_FLAG_QUICK_POLL, openups_charging_info},
	{"BOOL", 0, 0, "UPS.PowerSummary.PresentStatus.Discharging", NULL, NULL, HU_FLAG_QUICK_POLL, openups_discharging_info},
	{"BOOL", 0, 0, "UPS.PowerSummary.PresentStatus.NeedReplacement", NULL, NULL, 0, replacebatt_info},
	{"BOOL", 0, 0, "UPS.PowerSummary.PresentStatus.ACPresent", NULL, NULL, HU_FLAG_QUICK_POLL, openups_online_info},
	{"BOOL", 0, 0, "UPS.PowerSummary.PresentStatus.BatteryPresent", NULL, NULL, HU_FLAG_QUICK_POLL, openups_nobattery_info},

	/* end of structure. */
	{NULL, 0, 0, NULL, NULL, NULL, 0, NULL}
};

static const char *openups_format_model(HIDDevice_t * hd)
{
	return hd->Product;
}

static const char *openups_format_mfr(HIDDevice_t * hd)
{
	return hd->Vendor ? hd->Vendor : "openUPS";
}

static const char *openups_format_serial(HIDDevice_t * hd)
{
	return hd->Serial;
}

/* this function allows the subdriver to "claim" a device: return 1 if
 * the device is supported by this subdriver, else 0. */
static int openups_claim(HIDDevice_t * hd)
{
	int status = is_usb_device_supported(openups_usb_device_table, hd);

	switch (status) {
	case POSSIBLY_SUPPORTED:
		/* by default, reject, unless the productid option is given */
		if (getval("productid")) {
			return 1;
		}
		possibly_supported("openUPS", hd);
		return 0;

	case SUPPORTED:
		return 1;

	case NOT_SUPPORTED:
	default:
		return 0;
	}
}

subdriver_t openups_subdriver = {
	OPENUPS_HID_VERSION,
	openups_claim,
	openups_utab,
	openups_hid2nut,
	openups_format_model,
	openups_format_mfr,
	openups_format_serial,
};