/*
 * Galaxy Camera 2 (EK-GC200): NFC field detection
 *
 * The GC200 carries a passive NXP NTAG213 tag. The AP cannot read or write
 * it; it only sees the tag's field-detect output on GPIO_NFC_IRQ, which
 * idles high and pulses low for 0.1-0.2 s about once a second while a
 * phone's NFC field is on the tag.
 *
 * This counts approaches: a pulse after at least SF2_NFC_QUIET_MS without
 * one is a new approach. The count is /sys/kernel/nfc_field/approaches;
 * it is sysfs_notify()ed on every change, so userspace can sleep in poll()
 * until a phone comes near (a phone held on the tag counts once).
 *
 * Samsung's GC200 kernel (drivers/nfc/sf2_nfc_tag.c) sent a uevent on every
 * pulse instead. The pin is not in an alive bank, so it cannot wake the
 * system from suspend; nothing here keeps it awake either.
 *
 * Copyright (C) 2026 The GalaxyCameraRevived Project
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>

#include <plat/gpio-cfg.h>
#include <mach/gpio-midas.h>

/* irq-gpioint.c */
extern int s5p_register_gpio_interrupt(int pin);

#define SF2_NFC_QUIET_MS	3000	/* no pulse for this long: the phone left */
#define SF2_NFC_DEBOUNCE_MS	20	/* a real pulse is still low after this */

static struct kobject *nfc_kobj;
static unsigned int approaches;
static unsigned long last_pulse;	/* jiffies */
static bool seen_pulse;

static ssize_t approaches_show(struct kobject *kobj,
			       struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", approaches);
}

static struct kobj_attribute approaches_attr = __ATTR_RO(approaches);

static irqreturn_t sf2_nfc_field_irq(int irq, void *data)
{
	unsigned long now;

	/* a glitch on the line is gone again by now */
	msleep(SF2_NFC_DEBOUNCE_MS);
	if (gpio_get_value(GPIO_NFC_IRQ))
		return IRQ_HANDLED;

	now = jiffies;
	if (!seen_pulse ||
	    time_after(now, last_pulse + msecs_to_jiffies(SF2_NFC_QUIET_MS))) {
		approaches++;
		pr_info("sf2-nfc-field: phone field on the tag (%u)\n",
			approaches);
		sysfs_notify(nfc_kobj, NULL, "approaches");
	}
	last_pulse = now;
	seen_pulse = true;

	return IRQ_HANDLED;
}

static int __init sf2_nfc_field_init(void)
{
	int irq, ret;

	ret = gpio_request(GPIO_NFC_IRQ, "nfc_field");
	if (ret) {
		pr_err("sf2-nfc-field: gpio_request failed (%d)\n", ret);
		return ret;
	}
	s3c_gpio_cfgpin(GPIO_NFC_IRQ, S3C_GPIO_INPUT);
	s3c_gpio_setpull(GPIO_NFC_IRQ, S3C_GPIO_PULL_NONE);
	gpio_direction_input(GPIO_NFC_IRQ);

	/* GPF0 is a GPIO interrupt group, registered on first use */
	irq = s5p_register_gpio_interrupt(GPIO_NFC_IRQ);
	if (irq < 0) {
		pr_err("sf2-nfc-field: no interrupt for the pin (%d)\n", irq);
		ret = irq;
		goto err_gpio;
	}

	nfc_kobj = kobject_create_and_add("nfc_field", kernel_kobj);
	if (!nfc_kobj) {
		ret = -ENOMEM;
		goto err_gpio;
	}
	ret = sysfs_create_file(nfc_kobj, &approaches_attr.attr);
	if (ret)
		goto err_kobj;

	ret = request_threaded_irq(gpio_to_irq(GPIO_NFC_IRQ), NULL,
				   sf2_nfc_field_irq,
				   IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				   "nfc_field", NULL);
	if (ret) {
		pr_err("sf2-nfc-field: request_irq failed (%d)\n", ret);
		goto err_file;
	}

	pr_info("sf2-nfc-field: watching the tag's field detect (irq %d)\n",
		gpio_to_irq(GPIO_NFC_IRQ));
	return 0;

err_file:
	sysfs_remove_file(nfc_kobj, &approaches_attr.attr);
err_kobj:
	kobject_put(nfc_kobj);
	nfc_kobj = NULL;
err_gpio:
	gpio_free(GPIO_NFC_IRQ);
	return ret;
}
late_initcall(sf2_nfc_field_init);
