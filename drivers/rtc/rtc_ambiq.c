/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2024 Ambiq Micro
 */


#include <zephyr/drivers/rtc.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/util.h>

#define DT_DRV_COMPAT ambiq_rtc

LOG_MODULE_REGISTER(ambiq_rtc, CONFIG_RTC_LOG_LEVEL);

#include <am_mcu_apollo.h>

#define AMBIQ_RTC_ALARM_TIME_MASK                                                         \
	(RTC_ALARM_TIME_MASK_SECOND | RTC_ALARM_TIME_MASK_MINUTE | RTC_ALARM_TIME_MASK_HOUR | \
	 RTC_ALARM_TIME_MASK_WEEKDAY | RTC_ALARM_TIME_MASK_MONTH | RTC_ALARM_TIME_MASK_YEAR)

/* struct tm start time:   1st, Jan, 1900 */
#define TM_YEAR_REF 1900
#define AMBIQ_RTC_YEAR_MAX 2199

struct ambiq_rtc_data {
	struct k_spinlock lock;
#ifdef CONFIG_RTC_ALARM
	struct rtc_time alarm_time;
	uint16_t alarm_set_mask;
	rtc_alarm_callback alarm_user_callback;
	void *alarm_user_data;
	bool alarm_pending;
#endif
};

static struct ambiq_rtc_data rtc_data;

static int iYear;

static void rtc_time_to_ambiq_time_set(const struct rtc_time *tm, am_hal_rtc_time_t *atm)
{
	atm->ui32CenturyEnable = true;
	atm->ui32Century = ((tm->tm_year <= 99) || (tm->tm_year >= 200));
	atm->ui32Year = tm->tm_year;
	if (tm->tm_year > 99) {
		atm->ui32Year = tm->tm_year % 100;
	}
	atm->ui32Weekday = tm->tm_wday;
	atm->ui32Month = tm->tm_mon + 1;
	atm->ui32DayOfMonth = tm->tm_mday;
	atm->ui32Hour = tm->tm_hour;
	atm->ui32Minute = tm->tm_min;
	atm->ui32Second = tm->tm_sec;

	/* Nanoseconds times 10mil is hundredths */
	atm->ui32Hundredths = tm->tm_nsec / 10000000;
	iYear = atm->ui32Year;
}

/* To set the timer registers */
static void ambiq_time_to_rtc_time_set(const am_hal_rtc_time_t *atm, struct rtc_time *tm)
{
	tm->tm_year = atm->ui32Year;
	if (atm->ui32Century == 0)
		tm->tm_year += 100;
	else
		tm->tm_year += 200;
	tm->tm_wday = atm->ui32Weekday;
	tm->tm_mon = atm->ui32Month - 1;
	tm->tm_mday = atm->ui32DayOfMonth;
	tm->tm_hour = atm->ui32Hour;
	tm->tm_min = atm->ui32Minute;
	tm->tm_sec = atm->ui32Second;

	/* Nanoseconds times 10mil is hundredths */
	tm->tm_nsec = atm->ui32Hundredths * 10000000;
}

/* To set the timer registers */
static int ambiq_rtc_set_time(const struct device *dev,
							const struct rtc_time *timeptr)
{
	int err = 0;
	am_hal_rtc_time_t ambiq_time = {0};

	struct ambiq_rtc_data *data = dev->data;

	if (timeptr->tm_year + TM_YEAR_REF > AMBIQ_RTC_YEAR_MAX) {
		return -EINVAL;
	}

	k_spinlock_key_t key = k_spin_lock(&data->lock);

	LOG_DBG("set time: year = %d, mon = %d, mday = %d, wday = %d, hour = %d, "
			"min = %d, sec = %d",
			timeptr->tm_year, timeptr->tm_mon, timeptr->tm_mday, timeptr->tm_wday,
			timeptr->tm_hour, timeptr->tm_min, timeptr->tm_sec);

	/* Convertn to Ambiq Time */
	rtc_time_to_ambiq_time_set(timeptr, &ambiq_time);

	err = am_hal_rtc_time_set(&ambiq_time);
	if (err) {
		LOG_WRN("Set Timer returned an error - %d!", err);
	}

	k_spin_unlock(&data->lock, key);

	return err;
}

/* To get from the timer registers */
static int ambiq_rtc_get_time(const struct device *dev, struct rtc_time *timeptr)
{
	int err = 0;
	am_hal_rtc_time_t ambiq_time = {0};
	struct ambiq_rtc_data *data = dev->data;

	k_spinlock_key_t key = k_spin_lock(&data->lock);

	err = am_hal_rtc_time_get(&ambiq_time);
	if (err != 0) {
		LOG_WRN("Get Timer returned an error - %d!", err);
		goto unlock;
	}

	if ((iYear == 99) && (ambiq_time.ui32Year == 0)) {
		if (ambiq_time.ui32CenturyEnable == true) {
			ambiq_time.ui32Century = 0;
			am_hal_rtc_time_t ambiq_time2 = ambiq_time;

			am_hal_rtc_time_set(&ambiq_time2);
			if (err != 0) {
				LOG_WRN("Get Timer returned an error - %d!", err);
				goto unlock;
			}
			am_hal_rtc_time_get(&ambiq_time);
		}
	}

	ambiq_time_to_rtc_time_set(&ambiq_time, timeptr);

	LOG_DBG("get time: year = %d, mon = %d, mday = %d, wday = %d, hour = %d, "
			"min = %d, sec = %d",
			timeptr->tm_year, timeptr->tm_mon, timeptr->tm_mday, timeptr->tm_wday,
			timeptr->tm_hour, timeptr->tm_min, timeptr->tm_sec);

unlock:
	k_spin_unlock(&data->lock, key);

	return err;
}

#ifdef CONFIG_RTC_ALARM
static int lcl_time_input_validate(const struct rtc_time *timeptr, uint16_t mask)
{
	if (timeptr != NULL) {
		if (mask & RTC_ALARM_TIME_MASK_YEAR) {
			if (timeptr->tm_year > 100) {
				return -EINVAL;
			}
		}

		if (mask & RTC_ALARM_TIME_MASK_MONTH) {
			if (timeptr->tm_mon > 12) {
				return -EINVAL;
			}
		}

		if (mask & RTC_ALARM_TIME_MASK_MONTHDAY) {
			if (timeptr->tm_mday > 31) {
				return -EINVAL;
			}
		}

		if (mask & RTC_ALARM_TIME_MASK_WEEKDAY) {
			if (timeptr->tm_wday > 6) {
				return -EINVAL;
			}
		}

		if (mask & RTC_ALARM_TIME_MASK_HOUR) {
			if (timeptr->tm_hour >= 24) {
				return -EINVAL;
			}
		}

		if (mask & RTC_ALARM_TIME_MASK_MINUTE) {
			if (timeptr->tm_min >= 60) {
				return -EINVAL;
			}
		}

		if (mask & RTC_ALARM_TIME_MASK_SECOND) {
			if (timeptr->tm_sec >= 60) {
				return -EINVAL;
			}
		}
	}

	return 0;
}

static int ambiq_rtc_alarm_get_supported_fields(const struct device *dev,
								uint16_t id, uint16_t *mask)
{
	ARG_UNUSED(dev);

	if (id != 0U) {
		LOG_ERR("Invalid ID %d", id);
		return -EINVAL;
	}

	*mask = AMBIQ_RTC_ALARM_TIME_MASK;

	return 0;
}

/* To get from the alarm registers */
static int ambiq_rtc_alarm_get_time(const struct device *dev, uint16_t id, uint16_t *mask,
		struct rtc_time *timeptr)
{

	int err = 0;
	am_hal_rtc_time_t ambiq_time = {0};
	struct ambiq_rtc_data *data = dev->data;

	if (id != 0U) {
		LOG_ERR("Invalid ID %d", id);
		return -EINVAL;
	}

	k_spinlock_key_t key = k_spin_lock(&data->lock);

	err = am_hal_rtc_alarm_get(&ambiq_time, NULL);
	if (err != 0) {
		LOG_DBG("Invalid Input Value");
		return -EINVAL;
	}

	ambiq_time_to_rtc_time_set(&ambiq_time, timeptr);

	*mask = data->alarm_set_mask;

	LOG_DBG("get alarm: wday = %d, mon = %d, mday = %d, hour = %d, min = %d, sec = %d, "
		"mask = 0x%04x", timeptr->tm_wday, timeptr->tm_mon, timeptr->tm_mday,
		timeptr->tm_hour, timeptr->tm_min, timeptr->tm_sec, *mask);

	k_spin_unlock(&data->lock, key);

	return err;
}

static int ambiq_rtc_alarm_set_time(const struct device *dev, uint16_t id, uint16_t mask,
		const struct rtc_time *timeptr)
{
	int err = 0;
	struct ambiq_rtc_data *data = dev->data;
	am_hal_rtc_time_t ambiq_time = {0};
	uint16_t mask_available;

	if (id != 0U) {
		LOG_ERR("Invalid ID %d", id);
		return -EINVAL;
	}

	err = lcl_time_input_validate(timeptr, mask);
	if (err != 0) {
		LOG_DBG("Invalid Input Value");
		return -EINVAL;
	}

	(void)ambiq_rtc_alarm_get_supported_fields(NULL, 0, &mask_available);

	if (mask & ~mask_available) {
		return -EINVAL;
	}

	data->alarm_set_mask = mask;

	k_spinlock_key_t key = k_spin_lock(&data->lock);

	/* Disable and clear the alarm */
	am_hal_rtc_interrupt_disable(AM_HAL_RTC_INT_ALM);
	am_hal_rtc_interrupt_clear(AM_HAL_RTC_INT_ALM);

	/* When mask is 0 */
	if (mask == 0) {
		LOG_DBG("The alarm is disabled");
		goto unlock;
	}

	LOG_DBG("set alarm: second = %d, min = %d, hour = %d, mday = %d, month = %d,"
			"wday = %d,  mask = 0x%04x",
			timeptr->tm_sec, timeptr->tm_min, timeptr->tm_hour, timeptr->tm_mday,
			timeptr->tm_mon, timeptr->tm_wday, mask);

	/* Convertn to Ambiq Time */
	rtc_time_to_ambiq_time_set(timeptr, &ambiq_time);

	/*  Set RTC ALARM, Ambiq must have interval != AM_HAL_RTC_ALM_RPT_DIS */
	if (0 != am_hal_rtc_alarm_set(&ambiq_time, AM_HAL_RTC_ALM_RPT_YR)) {
		LOG_DBG("Invalid Input Value");
		err = -EINVAL;
		goto unlock;
	}

	am_hal_rtc_interrupt_enable(AM_HAL_RTC_INT_ALM);

unlock:
	k_spin_unlock(&data->lock, key);

	return err;
}

static int ambiq_rtc_alarm_is_pending(const struct device *dev, uint16_t id)
{
	struct ambiq_rtc_data *data = dev->data;
	int ret = 0;

	if (id != 0) {
		return -EINVAL;
	}

	K_SPINLOCK(&data->lock) {
		ret = data->alarm_pending ? 1 : 0;
		data->alarm_pending = false;
	}

	return ret;
}

static void ambiq_rtc_isr(const struct device *dev)
{
	/* Clear the RTC alarm interrupt. 8*/
	am_hal_rtc_interrupt_clear(AM_HAL_RTC_INT_ALM);

#if defined(CONFIG_RTC_ALARM)
	struct ambiq_rtc_data *data = dev->data;

	if (data->alarm_user_callback) {
		data->alarm_user_callback(dev, 0, data->alarm_user_data);
		data->alarm_pending = false;
	} else {
		data->alarm_pending = true;
	}
#endif
}

static int ambiq_rtc_alarm_set_callback(const struct device *dev, uint16_t id,
		rtc_alarm_callback callback, void *user_data)
{

	struct ambiq_rtc_data *data = dev->data;

	if (id != 0U) {
		LOG_ERR("Invalid ID %d", id);
		return -EINVAL;
	}

	K_SPINLOCK(&data->lock) {
		data->alarm_user_callback = callback;
		data->alarm_user_data = user_data;
		if ((callback == NULL) && (user_data == NULL)) {
			am_hal_rtc_interrupt_disable(AM_HAL_RTC_INT_ALM);
		}
	}

	return 0;
}
#endif

static int ambiq_rtc_init(const struct device *dev)
{
#ifdef CONFIG_RTC_ALARM
	struct ambiq_rtc_data *data = dev->data;
#endif

	/* Enable the XTAL for RTC. */
	am_hal_clkgen_control(AM_HAL_RTC_OSC_XT, NULL);

	/* Enable the RTC. */
	am_hal_rtc_osc_enable();

#ifdef CONFIG_RTC_ALARM
	data->alarm_user_callback = NULL;
	data->alarm_pending = false;

	IRQ_CONNECT(DT_INST_IRQN(0), DT_INST_IRQ(0, priority), ambiq_rtc_isr, DEVICE_DT_INST_GET(0),
			0);
	irq_enable(DT_INST_IRQN(0));
#endif
	return 0;
}

static const struct rtc_driver_api ambiq_rtc_driver_api = {
	.set_time = ambiq_rtc_set_time,
	.get_time = ambiq_rtc_get_time,
#ifdef CONFIG_RTC_ALARM
	.alarm_get_supported_fields = ambiq_rtc_alarm_get_supported_fields,
	.alarm_set_time = ambiq_rtc_alarm_set_time,
	.alarm_get_time = ambiq_rtc_alarm_get_time,
	.alarm_is_pending = ambiq_rtc_alarm_is_pending,
	.alarm_set_callback = ambiq_rtc_alarm_set_callback,
#endif
};

DEVICE_DT_INST_DEFINE(0, &ambiq_rtc_init, NULL, &rtc_data, NULL, POST_KERNEL,
			  CONFIG_RTC_INIT_PRIORITY, &ambiq_rtc_driver_api);
