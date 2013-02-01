/* openups-hid.c - subdriver to monitor openUPS USB/HID devices with NUT
 *
 *  Copyright (C)
 *  2003 - 2012	Arnaud Quette <ArnaudQuette@Eaton.com>
 *  2005 - 2006	Peter Selinger <selinger@users.sourceforge.net>
 *  2008 - 2009	Arjen de Korte <adkorte-guest@alioth.debian.org>
 *
 *  Note: this subdriver was initially generated as a "stub" by the
 *  gen-usbhid-subdriver script. It must be customized.
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
#include "main.h"	/* for getval() */
#include "usb-common.h"

#define OPENUPS_HID_VERSION	"openUPS HID 0.1"
/* FIXME: experimental flag to be put in upsdrv_info */

/* openUPS */
#define OPENUPS_VENDORID	0x04d8

/* USB IDs device table */
static usb_device_id_t openups_usb_device_table[] = {
	/* openUPS */
	{ USB_DEVICE(OPENUPS_VENDORID, 0xd004), NULL },

	/* Terminating entry */
	{ -1, -1, NULL }
};


/* --------------------------------------------------------------- */
/*      Vendor-specific usage table */
/* --------------------------------------------------------------- */

/* OPENUPS usage table */
static usage_lkp_t openups_usage_lkp[] = {
	{ "OPENUPS1",	0x00000001 },
	{ "OPENUPS2",	0x00000002 },
	{ "OPENUPS3",	0x00000003 },
	{ "OPENUPS4",	0x00000004 },
	{ "OPENUPS5",	0x00000005 },
	{ "OPENUPS6",	0x00000006 },
	{ "OPENUPS7",	0xff000001 },
	{  NULL, 0 }
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

  { "unmapped.openups7.openups7", 0, 0, "OPENUPS7.OPENUPS7", NULL, "%.0f", 0, NULL },
  { "unmapped.ups.powersummary.battery.openups1", 0, 0, "UPS.PowerSummary.Battery.OPENUPS1", NULL, "%.0f", 0, NULL },
  { "unmapped.ups.powersummary.battery.openups2", 0, 0, "UPS.PowerSummary.Battery.OPENUPS2", NULL, "%.0f", 0, NULL },
  { "unmapped.ups.powersummary.battery.openups3", 0, 0, "UPS.PowerSummary.Battery.OPENUPS3", NULL, "%.0f", 0, NULL },
  { "unmapped.ups.powersummary.battery.openups4", 0, 0, "UPS.PowerSummary.Battery.OPENUPS4", NULL, "%.0f", 0, NULL },
  { "unmapped.ups.powersummary.battery.openups5", 0, 0, "UPS.PowerSummary.Battery.OPENUPS5", NULL, "%.0f", 0, NULL },
  { "unmapped.ups.powersummary.battery.openups6", 0, 0, "UPS.PowerSummary.Battery.OPENUPS6", NULL, "%.0f", 0, NULL },
  { "unmapped.ups.powersummary.capacitygranularity1", 0, 0, "UPS.PowerSummary.CapacityGranularity1", NULL, "%.0f", 0, NULL },
  { "unmapped.ups.powersummary.capacitygranularity2", 0, 0, "UPS.PowerSummary.CapacityGranularity2", NULL, "%.0f", 0, NULL },
  { "unmapped.ups.powersummary.capacitymode", 0, 0, "UPS.PowerSummary.CapacityMode", NULL, "%.0f", 0, NULL },
  { "unmapped.ups.powersummary.configcurrent", 0, 0, "UPS.PowerSummary.ConfigCurrent", NULL, "%.0f", 0, NULL },
  { "unmapped.ups.powersummary.configvoltage", 0, 0, "UPS.PowerSummary.ConfigVoltage", NULL, "%.0f", 0, NULL },
  { "unmapped.ups.powersummary.current", 0, 0, "UPS.PowerSummary.Current", NULL, "%.0f", 0, NULL },
  { "unmapped.ups.powersummary.designcapacity", 0, 0, "UPS.PowerSummary.DesignCapacity", NULL, "%.0f", 0, NULL },
  { "unmapped.ups.powersummary.fullchargecapacity", 0, 0, "UPS.PowerSummary.FullChargeCapacity", NULL, "%.0f", 0, NULL },
  { "unmapped.ups.powersummary.idevicechemistry", 0, 0, "UPS.PowerSummary.iDeviceChemistry", NULL, "%.0f", 0, NULL },
  { "unmapped.ups.powersummary.imanufacturer", 0, 0, "UPS.PowerSummary.iManufacturer", NULL, "%.0f", 0, NULL },
  { "unmapped.ups.powersummary.imanufacturername", 0, 0, "UPS.PowerSummary.iManufacturerName", NULL, "%.0f", 0, NULL },
  { "unmapped.ups.powersummary.iname", 0, 0, "UPS.PowerSummary.iName", NULL, "%.0f", 0, NULL },
  { "unmapped.ups.powersummary.input.current", 0, 0, "UPS.PowerSummary.Input.Current", NULL, "%.0f", 0, NULL },
  { "unmapped.ups.powersummary.input.voltage", 0, 0, "UPS.PowerSummary.Input.Voltage", NULL, "%.0f", 0, NULL },
  { "unmapped.ups.powersummary.ioeminformation", 0, 0, "UPS.PowerSummary.iOEMInformation", NULL, "%.0f", 0, NULL },
  { "unmapped.ups.powersummary.iproduct", 0, 0, "UPS.PowerSummary.iProduct", NULL, "%.0f", 0, NULL },
  { "unmapped.ups.powersummary.iserialnumber", 0, 0, "UPS.PowerSummary.iSerialNumber", NULL, "%.0f", 0, NULL },
  { "unmapped.ups.powersummary.output.current", 0, 0, "UPS.PowerSummary.Output.Current", NULL, "%.0f", 0, NULL },
  { "unmapped.ups.powersummary.output.voltage", 0, 0, "UPS.PowerSummary.Output.Voltage", NULL, "%.0f", 0, NULL },
  { "unmapped.ups.powersummary.powersummaryid", 0, 0, "UPS.PowerSummary.PowerSummaryID", NULL, "%.0f", 0, NULL },
  { "unmapped.ups.powersummary.presentstatus.acpresent", 0, 0, "UPS.PowerSummary.PresentStatus.ACPresent", NULL, "%.0f", 0, NULL },
  { "unmapped.ups.powersummary.presentstatus.batterypresent", 0, 0, "UPS.PowerSummary.PresentStatus.BatteryPresent", NULL, "%.0f", 0, NULL },
  { "unmapped.ups.powersummary.presentstatus.belowremainingcapacitylimit", 0, 0, "UPS.PowerSummary.PresentStatus.BelowRemainingCapacityLimit", NULL, "%.0f", 0, NULL },
  { "unmapped.ups.powersummary.presentstatus.charging", 0, 0, "UPS.PowerSummary.PresentStatus.Charging", NULL, "%.0f", 0, NULL },
  { "unmapped.ups.powersummary.presentstatus.conditioningflag", 0, 0, "UPS.PowerSummary.PresentStatus.ConditioningFlag", NULL, "%.0f", 0, NULL },
  { "unmapped.ups.powersummary.presentstatus.discharging", 0, 0, "UPS.PowerSummary.PresentStatus.Discharging", NULL, "%.0f", 0, NULL },
  { "unmapped.ups.powersummary.presentstatus.good", 0, 0, "UPS.PowerSummary.PresentStatus.Good", NULL, "%.0f", 0, NULL },
  { "unmapped.ups.powersummary.presentstatus.internalfailure", 0, 0, "UPS.PowerSummary.PresentStatus.InternalFailure", NULL, "%.0f", 0, NULL },
  { "unmapped.ups.powersummary.presentstatus.needreplacement", 0, 0, "UPS.PowerSummary.PresentStatus.NeedReplacement", NULL, "%.0f", 0, NULL },
  { "unmapped.ups.powersummary.presentstatus.overcharged", 0, 0, "UPS.PowerSummary.PresentStatus.OverCharged", NULL, "%.0f", 0, NULL },
  { "unmapped.ups.powersummary.presentstatus.overload", 0, 0, "UPS.PowerSummary.PresentStatus.Overload", NULL, "%.0f", 0, NULL },
  { "unmapped.ups.powersummary.presentstatus.overtemperature", 0, 0, "UPS.PowerSummary.PresentStatus.OverTemperature", NULL, "%.0f", 0, NULL },
  { "unmapped.ups.powersummary.presentstatus.present", 0, 0, "UPS.PowerSummary.PresentStatus.Present", NULL, "%.0f", 0, NULL },
  { "unmapped.ups.powersummary.presentstatus.remainingtimelimitexpired", 0, 0, "UPS.PowerSummary.PresentStatus.RemainingTimeLimitExpired", NULL, "%.0f", 0, NULL },
  { "unmapped.ups.powersummary.presentstatus.shutdownimminent", 0, 0, "UPS.PowerSummary.PresentStatus.ShutdownImminent", NULL, "%.0f", 0, NULL },
  { "unmapped.ups.powersummary.presentstatus.shutdownrequested", 0, 0, "UPS.PowerSummary.PresentStatus.ShutdownRequested", NULL, "%.0f", 0, NULL },
  { "unmapped.ups.powersummary.remainingcapacity", 0, 0, "UPS.PowerSummary.RemainingCapacity", NULL, "%.0f", 0, NULL },
  { "unmapped.ups.powersummary.remainingcapacitylimit", 0, 0, "UPS.PowerSummary.RemainingCapacityLimit", NULL, "%.0f", 0, NULL },
  { "unmapped.ups.powersummary.runtimetoempty", 0, 0, "UPS.PowerSummary.RunTimeToEmpty", NULL, "%.0f", 0, NULL },
  { "unmapped.ups.powersummary.temperature", 0, 0, "UPS.PowerSummary.Temperature", NULL, "%.0f", 0, NULL },
  { "unmapped.ups.powersummary.voltage", 0, 0, "UPS.PowerSummary.Voltage", NULL, "%.0f", 0, NULL },
  { "unmapped.ups.powersummary.warningcapacitylimit", 0, 0, "UPS.PowerSummary.WarningCapacityLimit", NULL, "%.0f", 0, NULL },

  /* end of structure. */
  { NULL, 0, 0, NULL, NULL, NULL, 0, NULL }
};

static const char *openups_format_model(HIDDevice_t *hd) {
	return hd->Product;
}

static const char *openups_format_mfr(HIDDevice_t *hd) {
	return hd->Vendor ? hd->Vendor : "openUPS";
}

static const char *openups_format_serial(HIDDevice_t *hd) {
	return hd->Serial;
}

/* this function allows the subdriver to "claim" a device: return 1 if
 * the device is supported by this subdriver, else 0. */
static int openups_claim(HIDDevice_t *hd)
{
	int status = is_usb_device_supported(openups_usb_device_table, hd->VendorID, hd->ProductID);

	switch (status)
	{
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
