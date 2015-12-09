/*
 * linux/drivers/video/backlight/pwm_bl.c
 *
 * simple PWM based backlight control, board code has to setup
 * 1) pin configuration so PWM waveforms can output
 * 2) platform_data being correctly configured
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/err.h>
#include <linux/pwm.h>
#include <linux/pwm_backlight.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/mutex.h>


struct pwm_bl_data {
	struct pwm_device	*pwm;
	struct device		*dev;
	unsigned int		period;
	unsigned int		lth_brightness;
	unsigned int		bl_gpio;/*zhangwei add*/
	unsigned int		*levels;
	int			(*notify)(struct device *,
					  int brightness);
	void			(*notify_after)(struct device *,
					int brightness);
	int			(*check_fb)(struct device *, struct fb_info *);
	void			(*exit)(struct device *);
};

#define LEVEL_LIMIT 33

static DEFINE_MUTEX(fix_lock);

static int pwm_set_pulse(int brightness, unsigned int gpio_bl)
{
		int pulse_count;
		int bl_level;
		static int bl_level_last = 1;
		unsigned long flags;
		bl_level = LEVEL_LIMIT - brightness;
		mutex_lock(&fix_lock);
		if(bl_level == 33)
		{
			local_irq_save(flags);
			gpio_direction_output(gpio_bl, 0);
			mdelay(10);
			local_irq_restore(flags);
			bl_level_last = 0;
			goto set_bl_return;
		}
		
		if(bl_level == 0)
		{
			local_irq_save(flags);
			gpio_direction_output(gpio_bl, 1);
			udelay(5);
			local_irq_restore(flags);
			goto set_bl_return;
		}
			
		if(bl_level == bl_level_last)
			goto set_bl_return ;
			
		if(bl_level > bl_level_last)
			pulse_count = bl_level - bl_level_last;
		else
			pulse_count = 32 - bl_level_last + bl_level;
		
		bl_level_last = bl_level;	
		
		while(pulse_count --)
		{
			local_irq_save(flags);
			gpio_direction_output(gpio_bl, 0);
			udelay(1);
			gpio_direction_output(gpio_bl, 1);
			udelay(1);
			local_irq_restore(flags);
		}
		
set_bl_return:		
		mutex_unlock(&fix_lock);
//		gpio_free(gpio_bl);
		return 0;
}
static int pwm_backlight_update_status(struct backlight_device *bl)
{
	struct pwm_bl_data *pb = bl_get_data(bl);
	int brightness = bl->props.brightness;
///zhangwei del	int max = bl->props.max_brightness;

	if (bl->props.power != FB_BLANK_UNBLANK ||
	    bl->props.fb_blank != FB_BLANK_UNBLANK ||
	    bl->props.state & BL_CORE_FBBLANK)
		{
			brightness = 0;
			pr_debug("[david] -- %s -- 1 -- %d,%d,%d\n", __func__, bl->props.power, bl->props.fb_blank, bl->props.state);
		}
	if (pb->notify)
		{
			brightness = pb->notify(pb->dev, brightness);
			pr_debug("[david] -- %s -- 2\n", __func__);
		}
	if (brightness == 0) {
			pr_debug("[david] -- %s -- 3\n", __func__);
		//modify by david
		
		//pwm_config(pb->pwm, 0, pb->period);
		//pwm_disable(pb->pwm);

		/*gpio_direction_output(pb->bl_gpio, 0);		
		udelay(100);*/
		if(pwm_set_pulse(brightness, pb->bl_gpio))//add by d.z 20141221
		pr_info("%s, set pulse level fail\n", __func__);
		//end
	} else {
	pr_debug("[david] -- %s --  4, %d\n\n", __func__, brightness);
		//modify by david
		/*
		int duty_cycle;

		if (pb->levels) {
			duty_cycle = pb->levels[brightness];
			max = pb->levels[max];
		} else {
			duty_cycle = brightness;
		}

		duty_cycle = pb->lth_brightness +
		     (duty_cycle * (pb->period - pb->lth_brightness) / max);
		pwm_config(pb->pwm, duty_cycle, pb->period);
		pwm_enable(pb->pwm);
		*/
		if(pwm_set_pulse(brightness, pb->bl_gpio))//add by d.z 20141221
		pr_info("%s, set pulse level fail\n", __func__);
		
		//end
	}

	if (pb->notify_after)
		pb->notify_after(pb->dev, brightness);

	return 0;
}

static int pwm_backlight_get_brightness(struct backlight_device *bl)
{
	return bl->props.brightness;
}

static int pwm_backlight_check_fb(struct backlight_device *bl,
				  struct fb_info *info)
{
	struct pwm_bl_data *pb = bl_get_data(bl);

	return !pb->check_fb || pb->check_fb(pb->dev, info);
}

static const struct backlight_ops pwm_backlight_ops = {
	.update_status	= pwm_backlight_update_status,
	.get_brightness	= pwm_backlight_get_brightness,
	.check_fb	= pwm_backlight_check_fb,
};

#ifdef CONFIG_OF
static int pwm_backlight_parse_dt(struct device *dev,
				  struct platform_pwm_backlight_data *data)
{
	struct device_node *node = dev->of_node;
	struct property *prop;
	int length;
	u32 value;
	int ret;
	u32 bl_gpio_id;//zhangwei add

	if (!node)
		return -ENODEV;

	memset(data, 0, sizeof(*data));

	/* determine the number of brightness levels */
	prop = of_find_property(node, "brightness-levels", &length);
	if (!prop)
		return -EINVAL;

	data->max_brightness = length / sizeof(u32);

	/* read brightness levels from DT property */
	if (data->max_brightness > 0) {
		size_t size = sizeof(*data->levels) * data->max_brightness;

		data->levels = devm_kzalloc(dev, size, GFP_KERNEL);
		if (!data->levels)
			return -ENOMEM;

		ret = of_property_read_u32_array(node, "brightness-levels",
						 data->levels,
						 data->max_brightness);
		if (ret < 0)
			return ret;

		ret = of_property_read_u32(node, "default-brightness-level",
					   &value);
		if (ret < 0)
			return ret;
//zhangwei add
		bl_gpio_id = of_get_named_gpio(node, "qcom,bl-gpio", 0);

		if (bl_gpio_id < 0)
		{
			pr_err("[david] -- %s -- fail to get bl_gpio\n", __func__);
			return ret;
		}
		data->bl_gpio = bl_gpio_id;
///add end
		data->dft_brightness = value;
		data->max_brightness--;
	}

	/*
	 * TODO: Most users of this driver use a number of GPIOs to control
	 *       backlight power. Support for specifying these needs to be
	 *       added.
	 */

	return 0;
}

static struct of_device_id pwm_backlight_of_match[] = {
	{ .compatible = "pwm-backlight" },
	{ }
};

MODULE_DEVICE_TABLE(of, pwm_backlight_of_match);
#else
static int pwm_backlight_parse_dt(struct device *dev,
				  struct platform_pwm_backlight_data *data)
{
	return -ENODEV;
}
#endif

static int pwm_backlight_probe(struct platform_device *pdev)
{
	struct platform_pwm_backlight_data *data = pdev->dev.platform_data;
	struct platform_pwm_backlight_data defdata;
	struct backlight_properties props;
	struct backlight_device *bl;
	struct pwm_bl_data *pb;
	unsigned int max;
	int ret;

	pr_info("[david] -- %s\n", __func__);
	if (!data) {
		ret = pwm_backlight_parse_dt(&pdev->dev, &defdata);
		if (ret < 0) {
			dev_err(&pdev->dev, "failed to find platform data\n");
			return ret;
		}

		data = &defdata;
	}

	if (data->init) {
		ret = data->init(&pdev->dev);
		if (ret < 0)
			return ret;
	}

	pb = devm_kzalloc(&pdev->dev, sizeof(*pb), GFP_KERNEL);
	if (!pb) {
		dev_err(&pdev->dev, "no memory for state\n");
		ret = -ENOMEM;
		goto err_alloc;
	}

	if (data->levels) {
		max = data->levels[data->max_brightness];
		pb->levels = data->levels;
	} else
		max = data->max_brightness;

	pb->notify = data->notify;
	pb->notify_after = data->notify_after;
	pb->check_fb = data->check_fb;
	pb->exit = data->exit;
	pb->dev = &pdev->dev;

	//modify by david
	ret = gpio_request(data->bl_gpio, "bl_gpio_id");
	if(ret < 0)
	{
		printk("[david] -- %s -- fail to apply bl_gpio\n" , __func__);
		goto err_alloc;
	}
	pb->bl_gpio = data->bl_gpio;
	pr_info("[david] -- %s -- get bl_gpio\n", __func__);
	//del by d.z
	/*
	pb->pwm = devm_pwm_get(&pdev->dev, NULL);
	if (IS_ERR(pb->pwm)) {
		dev_err(&pdev->dev, "unable to request PWM, trying legacy API\n");

		pb->pwm = pwm_request(data->pwm_id, "pwm-backlight");
		if (IS_ERR(pb->pwm)) {
			dev_err(&pdev->dev, "unable to request legacy PWM\n");
			ret = PTR_ERR(pb->pwm);
			goto err_alloc;
		}
	}

	dev_dbg(&pdev->dev, "got pwm for backlight\n");
*/
	/*
	 * The DT case will set the pwm_period_ns field to 0 and store the
	 * period, parsed from the DT, in the PWM device. For the non-DT case,
	 * set the period from platform data.
	 */
	 /*
	if (data->pwm_period_ns > 0)
		pwm_set_period(pb->pwm, data->pwm_period_ns);

	pb->period = pwm_get_period(pb->pwm);
	pb->lth_brightness = data->lth_brightness * (pb->period / max);
*/
	memset(&props, 0, sizeof(struct backlight_properties));
	props.type = BACKLIGHT_RAW;
	props.max_brightness = data->max_brightness;
	bl = backlight_device_register(dev_name(&pdev->dev), &pdev->dev, pb,
				       &pwm_backlight_ops, &props);
	if (IS_ERR(bl)) {
		dev_err(&pdev->dev, "failed to register backlight\n");
		ret = PTR_ERR(bl);
		goto err_alloc;
	}

	if (data->dft_brightness > data->max_brightness) {
		dev_warn(&pdev->dev,
			 "invalid default brightness level: %u, using %u\n",
			 data->dft_brightness, data->max_brightness);
		data->dft_brightness = data->max_brightness;
	}

	bl->props.brightness = data->dft_brightness;
	backlight_update_status(bl);

	platform_set_drvdata(pdev, bl);
	return 0;

err_alloc:
	if (data->exit)
		data->exit(&pdev->dev);
	return ret;
}

static int pwm_backlight_remove(struct platform_device *pdev)
{
	struct backlight_device *bl = platform_get_drvdata(pdev);
	struct pwm_bl_data *pb = bl_get_data(bl);

	backlight_device_unregister(bl);
	pwm_config(pb->pwm, 0, pb->period);
	pwm_disable(pb->pwm);
	if (pb->exit)
		pb->exit(&pdev->dev);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int pwm_backlight_suspend(struct device *dev)
{
	struct backlight_device *bl = dev_get_drvdata(dev);
	struct pwm_bl_data *pb = bl_get_data(bl);
	pr_debug("[david] -- %s", __func__);

	if (pb->notify)
		pb->notify(pb->dev, 0);
	/*
	pwm_config(pb->pwm, 0, pb->period);
	pwm_disable(pb->pwm);
	*/
	if(pwm_set_pulse(0, pb->bl_gpio))//add by d.z 20141221
		pr_info("%s, set pulse level fail\n", __func__);
	if (pb->notify_after)
		pb->notify_after(pb->dev, 0);
	return 0;
}

static int pwm_backlight_resume(struct device *dev)
{
	struct backlight_device *bl = dev_get_drvdata(dev);
	struct pwm_bl_data *pb = bl_get_data(bl);
	pr_debug("[david] -- %s", __func__);
	if(pwm_set_pulse(LEVEL_LIMIT, pb->bl_gpio))//add by d.z 20141221
		pr_info("%s, set pulse level fail\n", __func__);
	backlight_update_status(bl);
	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(pwm_backlight_pm_ops, pwm_backlight_suspend,
			 pwm_backlight_resume);

static struct platform_driver pwm_backlight_driver = {
	.driver		= {
		.name		= "pwm-backlight",
		.owner		= THIS_MODULE,
		.pm		= &pwm_backlight_pm_ops,
		.of_match_table	= of_match_ptr(pwm_backlight_of_match),
	},
	.probe		= pwm_backlight_probe,
	.remove		= pwm_backlight_remove,
};

module_platform_driver(pwm_backlight_driver);

MODULE_DESCRIPTION("PWM based Backlight Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:pwm-backlight");

