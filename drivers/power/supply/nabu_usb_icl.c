// SPDX-License-Identifier: GPL-2.0-only
/*
 * Xiaomi Pad 5 (nabu): PM8150B USB input for the SMB5 charger.
 *
 * There is no SMB5 charger driver here, so the USBIN input current limit stays
 * at the 500 mA left by the bootloader and the tablet discharges while plugged
 * in. Program the limit from the USB PD contract (TCPM), the Type-C Rp
 * advertisement or the BC1.2 result,
 * and expose a USB power supply so userspace knows a charger is connected.
 *
 * Register layout: downstream smb5-reg.h / qpnp-smb5.c (ICL step 50 mA).
 */
#include <linux/device.h>
#include <linux/init.h>
#include <linux/minmax.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>

#define FG_DEV_NAME			"c440000.spmi:pmic@2:fuel-gauge@4000"

#define ICL_STATUS_REG			0x1107
#define APSD_RESULT_STATUS_REG		0x1308
#define  DCP_CHARGER_BIT		BIT(3)
#define  CDP_CHARGER_BIT		BIT(2)
#define  OCP_CHARGER_BIT		BIT(1)
#define USBIN_INT_RT_STS_REG		0x1310
#define  USBIN_PLUGIN_RT_STS_BIT	BIT(4)
#define USBIN_CURRENT_LIMIT_CFG_REG	0x1370
#define  ICL_STEP_MA			50
#define  ICL_MAX_MA			3000
#define TYPE_C_SNK_STATUS_REG		0x1506
#define  SNK_RP_1P5_BIT			BIT(2)
#define  SNK_RP_3P0_BIT			BIT(1)

#define FIND_RETRIES			60
#define TCPM_PSY_NAME			"tcpm-source-psy-c440000.spmi:pmic@2:typec@1500"

static struct {
	struct regmap *map;
	struct power_supply *psy;
	struct delayed_work find_work;
	struct delayed_work poll_work;
	unsigned int orig_cfg;
	unsigned int tries;
	bool online;
} nu;

static char *nabu_usb_supplied_to[] = { "qcom-battery" };

static const enum power_supply_property nabu_usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
};

static int nabu_usb_get_property(struct power_supply *psy,
				 enum power_supply_property psp,
				 union power_supply_propval *val)
{
	unsigned int icl;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = nu.online;
		return 0;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		if (regmap_read(nu.map, ICL_STATUS_REG, &icl))
			return -EIO;
		val->intval = icl * ICL_STEP_MA * 1000;
		return 0;
	default:
		return -EINVAL;
	}
}

static const struct power_supply_desc nabu_usb_desc = {
	.name		= "pm8150b-usb",
	.type		= POWER_SUPPLY_TYPE_USB,
	.properties	= nabu_usb_props,
	.num_properties	= ARRAY_SIZE(nabu_usb_props),
	.get_property	= nabu_usb_get_property,
};

/* Current granted by a USB PD contract negotiated by TCPM, in mA. */
static unsigned int nabu_usb_pd_ma(void)
{
	union power_supply_propval val;
	struct power_supply *psy;
	unsigned int ma = 0;

	psy = power_supply_get_by_name(TCPM_PSY_NAME);
	if (!psy)
		return 0;
	if (!power_supply_get_property(psy, POWER_SUPPLY_PROP_ONLINE, &val) &&
	    val.intval &&
	    !power_supply_get_property(psy, POWER_SUPPLY_PROP_CURRENT_MAX, &val) &&
	    val.intval > 0)
		ma = val.intval / 1000;
	power_supply_put(psy);

	return ma;
}

/* Limit to program, in register units; the bootloader value when unsure. */
static unsigned int nabu_usb_wanted_cfg(void)
{
	unsigned int snk, apsd, ma = nabu_usb_pd_ma();

	if (!ma && !regmap_read(nu.map, TYPE_C_SNK_STATUS_REG, &snk)) {
		if (snk & SNK_RP_3P0_BIT)
			ma = 3000;
		else if (snk & SNK_RP_1P5_BIT)
			ma = 1500;
	}
	if (!ma && !regmap_read(nu.map, APSD_RESULT_STATUS_REG, &apsd) &&
	    (apsd & (DCP_CHARGER_BIT | CDP_CHARGER_BIT | OCP_CHARGER_BIT)))
		ma = 1500;
	if (!ma)
		return nu.orig_cfg;

	return min(ma, ICL_MAX_MA) / ICL_STEP_MA;
}

static void nabu_usb_poll(struct work_struct *work)
{
	unsigned int rt, cur, want;
	bool online;

	online = !regmap_read(nu.map, USBIN_INT_RT_STS_REG, &rt) &&
		 (rt & USBIN_PLUGIN_RT_STS_BIT);
	want = online ? nabu_usb_wanted_cfg() : nu.orig_cfg;

	if (!regmap_read(nu.map, USBIN_CURRENT_LIMIT_CFG_REG, &cur) && cur != want) {
		regmap_write(nu.map, USBIN_CURRENT_LIMIT_CFG_REG, want);
		pr_info("nabu-usb: input current limit %u -> %u mA\n",
			cur * ICL_STEP_MA, want * ICL_STEP_MA);
	}

	if (online != nu.online) {
		nu.online = online;
		power_supply_changed(nu.psy);
	}

	schedule_delayed_work(&nu.poll_work, HZ);
}

/* The PMIC probes asynchronously; wait for the fuel gauge to find its regmap. */
static void nabu_usb_find(struct work_struct *work)
{
	struct power_supply_config cfg = {
		.supplied_to = nabu_usb_supplied_to,
		.num_supplicants = ARRAY_SIZE(nabu_usb_supplied_to),
	};
	struct device *fg, *pmic;

	fg = bus_find_device_by_name(&platform_bus_type, NULL, FG_DEV_NAME);
	if (!fg) {
		if (++nu.tries < FIND_RETRIES)
			schedule_delayed_work(&nu.find_work, HZ);
		else
			pr_warn("nabu-usb: %s not found\n", FG_DEV_NAME);
		return;
	}
	pmic = fg->parent;
	put_device(fg);

	nu.map = dev_get_regmap(pmic, NULL);
	if (!nu.map || regmap_read(nu.map, USBIN_CURRENT_LIMIT_CFG_REG, &nu.orig_cfg)) {
		pr_warn("nabu-usb: PM8150B regmap unavailable\n");
		return;
	}

	nu.psy = power_supply_register(pmic, &nabu_usb_desc, &cfg);
	if (IS_ERR(nu.psy)) {
		pr_warn("nabu-usb: power supply registration failed: %ld\n",
			PTR_ERR(nu.psy));
		return;
	}

	schedule_delayed_work(&nu.poll_work, 0);
}

static int __init nabu_usb_init(void)
{
	if (!of_machine_is_compatible("xiaomi,nabu"))
		return 0;

	INIT_DELAYED_WORK(&nu.find_work, nabu_usb_find);
	INIT_DELAYED_WORK(&nu.poll_work, nabu_usb_poll);
	schedule_delayed_work(&nu.find_work, 0);
	return 0;
}
late_initcall(nabu_usb_init);
