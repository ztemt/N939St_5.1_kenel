#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <asm/errno.h> 
#include <linux/cdev.h>
#include <linux/of_gpio.h>


#define KTD_I2C_NAME			"ktd2026"

static int major;
struct i2c_client *ktd20xx_client;
static struct class *ktd20xx_cls;
struct class *breathled_class;
struct device *breathled_dev;
struct device *ktd22xx_dev;


static int bre_led_gpio;

int breath_leds = 0;//1--breath_led,255--turn on 0--turn off
int led_on_off = 255;
unsigned long value = 0;

void ktd22xx_lowbattery_breath_leds_red(unsigned char brightness){
	/*
	 * RED flash time period: 2.5s, rise/fall 1s, sleep 0.5s 
	 * reg5 = 0xaa, reg1 = 0x12
	 */
	
	i2c_smbus_write_byte_data(ktd20xx_client, 0x04, 0x00);// initialization LED off
        i2c_smbus_write_byte_data(ktd20xx_client, 0x00, 0x20);// mode set---IC work when both SCL and SDA goes high
	i2c_smbus_write_byte_data(ktd20xx_client, 0x06, brightness);//set current is 15mA
	i2c_smbus_write_byte_data(ktd20xx_client, 0x05, 0xaa);//rase time
	i2c_smbus_write_byte_data(ktd20xx_client, 0x01, 0x12);//dry flash period
	i2c_smbus_write_byte_data(ktd20xx_client, 0x02, 0x00);//reset internal counter
	i2c_smbus_write_byte_data(ktd20xx_client, 0x04, 0x02);//allocate led1 to timer1
	i2c_smbus_write_byte_data(ktd20xx_client, 0x02, 0x56);//led flashing(curerent ramp-up and down countinuously)
}

void ktd22xx_lowbattery_breath_leds_green(unsigned char brightness){
	/*
	 * Green flash time period: 2.5s, rise/fall 1s, sleep 0.5s 
	 * reg5 = 0xaa, reg1 = 0x12
	 */
	i2c_smbus_write_byte_data(ktd20xx_client, 0x04, 0x00);// initialization LED off
        i2c_smbus_write_byte_data(ktd20xx_client, 0x00, 0x20);// mode set---IC work when both SCL and SDA goes high
	i2c_smbus_write_byte_data(ktd20xx_client, 0x07, brightness);//set current is 15mA
	i2c_smbus_write_byte_data(ktd20xx_client, 0x05, 0x77);//rase time
	i2c_smbus_write_byte_data(ktd20xx_client, 0x01, 0x1E);//dry flash period
	i2c_smbus_write_byte_data(ktd20xx_client, 0x02, 0x00);//reset internal counter
	i2c_smbus_write_byte_data(ktd20xx_client, 0x04, 0x08);//allocate led1 to timer1
	i2c_smbus_write_byte_data(ktd20xx_client, 0x02, 0x50);//led flashing(curerent ramp-up and down countinuously)
}

void ktd22xx_lowbattery_breath_leds_blue(unsigned char brightness){
	/*
	 * Blue flash time period: 2.5s, rise/fall 1s, sleep 0.5s 
	 * reg5 = 0xaa, reg1 = 0x12
	 */
	i2c_smbus_write_byte_data(ktd20xx_client, 0x04, 0x00);// initialization LED off
        i2c_smbus_write_byte_data(ktd20xx_client, 0x00, 0x20);// mode set---IC work when both SCL and SDA goes high
	i2c_smbus_write_byte_data(ktd20xx_client, 0x08, brightness);//set current is 15mA
	i2c_smbus_write_byte_data(ktd20xx_client, 0x05, 0xaa);//rase time
	i2c_smbus_write_byte_data(ktd20xx_client, 0x01, 0x12);//dry flash period
	i2c_smbus_write_byte_data(ktd20xx_client, 0x02, 0x00);//reset internal counter
	i2c_smbus_write_byte_data(ktd20xx_client, 0x04, 0x20);//allocate led1 to timer1
	i2c_smbus_write_byte_data(ktd20xx_client, 0x02, 0x56);//led flashing(curerent ramp-up and down countinuously)
}

void ktd2xx_led_on_red(unsigned char brightness){
	
	printk(" michael_Leepp after =================ktd2xx_led_on_red!\n");
        i2c_smbus_write_byte_data(ktd20xx_client, 0x04, 0x00);// initialization LED off
	i2c_smbus_write_byte_data(ktd20xx_client, 0x06, brightness);//set current is 15mA
        i2c_smbus_write_byte_data(ktd20xx_client, 0x00, 0x00);//mode set---IC work when both SCL and SDA goes high
	i2c_smbus_write_byte_data(ktd20xx_client, 0x04, 0x01);//turn om led
}

void ktd2xx_led_on_green(unsigned char brightness){
        i2c_smbus_write_byte_data(ktd20xx_client, 0x04, 0x00);// initialization LED off
	i2c_smbus_write_byte_data(ktd20xx_client, 0x07, brightness);//set current is 15mA
        i2c_smbus_write_byte_data(ktd20xx_client, 0x00, 0x00);//mode set---IC work when both SCL and SDA goes high
	i2c_smbus_write_byte_data(ktd20xx_client, 0x04, 0x04);//turn om led
}

void ktd2xx_led_on_blue(unsigned char brightness){
        i2c_smbus_write_byte_data(ktd20xx_client, 0x04, 0x00);// initialization LED off
	i2c_smbus_write_byte_data(ktd20xx_client, 0x08, brightness);//set current is 15mA
        i2c_smbus_write_byte_data(ktd20xx_client, 0x00, 0x00);//mode set---IC work when both SCL and SDA goes high
	i2c_smbus_write_byte_data(ktd20xx_client, 0x04, 0x10);//turn om led
}

 void ktd2xx_led_off(void){
	//turn off LED
        i2c_smbus_write_byte_data(ktd20xx_client, 0x00, 0x08);//Device OFF-Either SCL goes low or SDA stops toggling
}

void ktd22xx_breath_leds_time(int blink){
	//set breath led flash time from blink
	int period, flashtime;
	period = (blink >> 8) & 0xff;
	flashtime = blink & 0xff;
	printk("ktd20xx led write blink = %x, period = %x, flashtime = %x\n", blink, period, flashtime);

	i2c_smbus_write_byte_data(ktd20xx_client, 0x04, 0x00);// initialization LED off
	i2c_smbus_write_byte_data(ktd20xx_client, 0x00, 0x20);// mode set---IC work when both SCL and SDA goes high
	i2c_smbus_write_byte_data(ktd20xx_client, 0x06, 0xbf);//set current is 24mA
	i2c_smbus_write_byte_data(ktd20xx_client, 0x05, period);//rase time
	i2c_smbus_write_byte_data(ktd20xx_client, 0x01, flashtime);//dry flash period
	i2c_smbus_write_byte_data(ktd20xx_client, 0x02, 0x00);//reset internal counter
	i2c_smbus_write_byte_data(ktd20xx_client, 0x04, 0x02);//allocate led1 to timer1
	i2c_smbus_write_byte_data(ktd20xx_client, 0x02, 0x56);//led flashing(curerent ramp-up and down countinuously)

}

//EXPORT_SYMBOL(ktd22xx_breath_leds);
//EXPORT_SYMBOL(ktd2xx_led_on);
EXPORT_SYMBOL(ktd2xx_led_off);
EXPORT_SYMBOL(breath_leds);
EXPORT_SYMBOL(led_on_off);


static ssize_t Breathled_switch_show ( struct device *dev,
                                      struct device_attribute *attr, char *buf )
{
   return sprintf ( buf, "%d\n", breath_leds);
}
static ssize_t Breathled_switch_store ( struct device *dev,
                                       struct device_attribute *attr, const char *buf, size_t count )
{
	unsigned char mode;
	unsigned char brightness;
	int ret;
	unsigned long value;


	//printk(KERN_ERR" michael_Leepp =================Breathled_switch_store mode1!\n");
	
	ret = strict_strtoul(buf, 0, &value);
	if(ret < 0)
	{
		printk(KERN_ERR "%s:strict_strtoul failed, ret=%d\n", __func__, ret);	
		return ret;	
	}	
	
	if( value )
	{
		mode = (value>>8)&0xff;
		brightness = value & 0xff;
	}
	else
	{
		mode = 0;
		brightness = 255;
	}
	
	//printk(KERN_ERR" michael_Leepp =================Breathled_switch_store mode2 %d %d!\n", mode, brightness);

	if( mode == 0x1){
	   breath_leds = 1;
	   led_on_off = 1;
	   ktd2xx_led_on_red(brightness);
	}
	else if( mode == 0x2){
	   breath_leds = 2;
	   led_on_off = 2;
	   ktd2xx_led_on_green(brightness);
	}
	else if( mode == 0x3){
	   breath_leds = 3;
	   led_on_off = 3;
	   ktd2xx_led_on_blue(brightness);
	}
	else if( mode == 0x4){
	   breath_leds = 4;
	   led_on_off = 4;
	   ktd22xx_lowbattery_breath_leds_red(brightness);
	}
	else if( mode == 0x5){
	   breath_leds = 5;
	   led_on_off = 5;
	   ktd22xx_lowbattery_breath_leds_green(brightness);
	}
	else if( mode == 0x6){
	   breath_leds = 6;
	   led_on_off = 6;
	   ktd22xx_lowbattery_breath_leds_blue(brightness);
	}
	else{
	   breath_leds = 255;
	   led_on_off = 7;
	   ktd2xx_led_off();
	}
	
	return count;

}

static ssize_t Breathled_switch_show2 ( struct device *dev,
                                      struct device_attribute *attr, char *buf )
{
   return sprintf ( buf, "value :%ld\n", value);
}
static ssize_t Breathled_switch_store2 ( struct device *dev,
                                       struct device_attribute *attr, const char *buf, size_t count )
{
	//unsigned long value = 0;
	int ret;
	
    if (NULL == buf)
        return -EINVAL;
	 
	ret = strict_strtoul(buf, 0, &value);
	if(ret < 0)
	{
		printk(KERN_ERR "%s:strict_strtoul failed, ret=%d\n", __func__, ret);	
		return ret;	
	}	
	//sprintf ( buf, "%d\n", sn3191_breath_led);
	ktd22xx_breath_leds_time(value);
	breath_leds = 1;
	
	return count;

}

static DEVICE_ATTR(led, 0666, Breathled_switch_show, Breathled_switch_store);
static DEVICE_ATTR(breath_led, 0666, Breathled_switch_show2, Breathled_switch_store2);

 #if 0
static void breathled_attr_create(void){
	breathled_class = class_create(THIS_MODULE, "breath_led");
	    if (IS_ERR(breathled_class))
	        pr_err("Failed to create class(gt_dclick_class)!\n");
	    breathled_dev = device_create(breathled_class,
	                                     NULL, 0, NULL, "breathled");
	    if (IS_ERR(breathled_dev))
	        pr_err("Failed to create device(breathled_dev)!\n");

	       // update
	    if (device_create_file(breathled_dev, &dev_attr_breath_led) < 0)
	        pr_err("Failed to create device file(%s)!\n", dev_attr_breath_led.attr.name);
}
#endif
static int ktd20xx_open(struct inode *inode, struct file *file)
{
	printk(" michael_Leepp =================ktd20xx_open!\n");
	return 0;
}
static ssize_t ktd20xx_read(struct file *file, char __user *buf, size_t size, loff_t * offset)
{
	return 0;
}

static ssize_t ktd20xx_write(struct file *file, const char __user *buf, size_t size, loff_t *offset)
{

	unsigned char rec_data[2];
	unsigned char mode;
	unsigned char brightness;
	
	memset(rec_data, 0, 2);  //清零
	
	
	//printk(" michael_Leepp =================ktd20xx_write!\n");

	
	if (copy_from_user(rec_data, buf, 2))  
	{  
		return -EFAULT;  
	}  
	
	if( rec_data[1] & 0x80 )
	{
		mode = rec_data[1] & 0x0f;
		brightness = rec_data[0];
	}
	else
	{
		mode = rec_data[0];
		brightness = 255;
	}
	
	printk(" michael_Leepp =================ktd20xx_write mode %d %d!\n", mode, brightness);

	
	if( mode == 0x1){
	   breath_leds = 1;
	   led_on_off = 1;
	   ktd2xx_led_on_red(brightness);
	}
	else if( mode == 0x2){
	   breath_leds = 2;
	   led_on_off = 2;
	   ktd2xx_led_on_green(brightness);
	}
	else if( mode == 0x3){
	   breath_leds = 3;
	   led_on_off = 3;
	   ktd2xx_led_on_blue(brightness);
	}
	else if( mode == 0x4){
	   breath_leds = 4;
	   led_on_off = 4;
	   ktd22xx_lowbattery_breath_leds_red(brightness);
	}
	else if( mode == 0x5){
	   breath_leds = 5;
	   led_on_off = 5;
	   ktd22xx_lowbattery_breath_leds_green(brightness);
	}
	else if( mode == 0x6){
	   breath_leds = 6;
	   led_on_off = 6;
	   ktd22xx_lowbattery_breath_leds_blue(brightness);
	}
	else{
	   breath_leds = 255;
	   led_on_off = 7;
	   ktd2xx_led_off();
	}
	
    return 1;
}

//static const struct file_operations
static const struct file_operations ktd20xx_fops = {
	.owner = THIS_MODULE,
	.open = ktd20xx_open,
	.read  = ktd20xx_read,
	.write = ktd20xx_write,
};


static int ktd_gpio_config(void)
{
	int ret; 
	
	
	if (gpio_is_valid(bre_led_gpio)) 
	{
			ret = gpio_request(bre_led_gpio,
					   "home_key_en");
			if (ret < 0) {
				pr_err("%s:rst_gpio=[%d] req failed:%d\n",
				       __func__, bre_led_gpio, ret);
				return -EBUSY;
			}
			ret = gpio_direction_output(bre_led_gpio, 1);
			if (ret < 0) {
				pr_err("%s: set dirn rst failed: %d\n",
				       __func__, ret);
				return -EBUSY;
			}
			
			printk(" michael_Leepp before set gpio1[%d]=================%d!\n", bre_led_gpio, gpio_get_value(bre_led_gpio));
			
			gpio_set_value(bre_led_gpio, 1);
			
			printk(" michael_Leepp after  set gpio1[%d]=================%d!\n", bre_led_gpio, gpio_get_value(bre_led_gpio));
	}
		
	return 0;
}


static int ktd_get_dt_data(struct device *dev)
{
	int rc = 0;
	struct device_node *of_node = NULL;
	
	
	of_node = dev->of_node;
	
	if(!of_node)
	{
		pr_err("%s: invalid of_node\n", __func__);
		goto error;
	}
	
	
	bre_led_gpio = of_get_named_gpio(of_node, "ktd,bre_led_en", 0);
	if(bre_led_gpio < 0)
	{
		
		pr_err("%s: Can't get bre_led_gpio \n", __func__);
		goto error;
	}
	
	return 0;
	
error:
	return rc;
}

static int ktd20xx_probe(struct i2c_client *client, const struct i2c_device_id *id){

	int ret;
	int err = 0;

	printk("[%s]: Enter!\n", __func__);
	ktd20xx_client = kzalloc(sizeof(struct i2c_client), GFP_KERNEL);
	if (!ktd20xx_client) {
		dev_err(&client->dev,
				"%s: memory allocation failed.", __func__);
		err = -ENOMEM;
		goto exit1;
	}

	ktd20xx_client = client;
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev,
				"%s: check_functionality failed.", __func__);
		err = -ENODEV;
		goto exit0;
	}
	/* create i2c struct, when receve and transmit some byte */
	//printk("ktd2026 address is %x \n",ktd20xx_client->addr);

	major = register_chrdev(0, KTD_I2C_NAME, &ktd20xx_fops);

	//ktd2xx_led_off(); //turn off led when first start ktd
	ret = i2c_smbus_write_byte_data(ktd20xx_client, 0x00, 0x07);// rset  ic  //add by lidan
	
	ret = i2c_smbus_write_byte_data(ktd20xx_client, 0x06, 0x00);//set current is 0.125mA
	ret = i2c_smbus_write_byte_data(ktd20xx_client, 0x04, 0x00);//turn off leds	
	if(ret < 0){
		printk("can't find ktd2026 led control ic!");
	}
	else{
		/* create device node sys/class/ktd20xx/ktd2206/led
		 * led: 0:breath led 1:open led  2:close led
		 */	
		ktd20xx_cls = class_create(THIS_MODULE, "ktd20xx");
		ktd22xx_dev = device_create(ktd20xx_cls, NULL, MKDEV(major, 0), NULL, "ktd2026"); /* /dev/ktd20xx */
		if (device_create_file(ktd22xx_dev, &dev_attr_led) < 0)
		    pr_err("Failed to create device file(%s)!\n", dev_attr_led.attr.name);
		if (device_create_file(ktd22xx_dev, &dev_attr_breath_led) < 0)
		    pr_err("Failed to create device file(%s)!\n", dev_attr_led.attr.name);
	}
	
	ktd_get_dt_data(&client->dev);

	ktd_gpio_config();
	
	
	//ktd2xx_led_on_green();
	//ktd22xx_lowbattery_breath_leds_green();
	
exit0:
exit1:

	return err;
}

static void ktd20xx_shutdown(struct i2c_client *client)
{
	i2c_smbus_write_byte_data(ktd20xx_client, 0x00, 0x07);//
}

static int ktd20xx_remove(struct i2c_client *client){

	printk("[%s]: Enter!\n", __func__);
	
	unregister_chrdev(0, KTD_I2C_NAME);
	class_destroy(ktd20xx_cls);
		
	return 0;
}
	

static const struct i2c_device_id ktd2xx_id[] = {
	{KTD_I2C_NAME, 0},
	{ }
};


static struct of_device_id ktd20xx_match_table[] = {
        { .compatible = "ktd,ktd2026",},
        { },
};

/* 1. 分配一个i2c_driver结构体 */
/* 2. 设置i2c_driver结构体 */
static struct i2c_driver ktd20xx_driver = {
	.driver = {
		.name	= KTD_I2C_NAME,
		.owner = THIS_MODULE,
		.of_match_table = ktd20xx_match_table,
	},
	.probe = ktd20xx_probe,
	.remove = ktd20xx_remove,
	.id_table = ktd2xx_id,
        .shutdown = ktd20xx_shutdown,
};

static int __init ktd20xx_init(void)
{
	int err;
	printk("%s\n",__func__);
	err = i2c_add_driver(&ktd20xx_driver);
	if (err) {
		printk(KERN_WARNING "ktd20xx driver failed "
		       "(errno = %d)\n", err);
	} else {
		printk( "Successfully added driver %s\n",
		          ktd20xx_driver.driver.name);
	}
	return err;
}

static void __exit ktd20xx_exit(void)
{
	printk("%s\n",__func__);
	i2c_del_driver(&ktd20xx_driver);
}

module_init(ktd20xx_init);
module_exit(ktd20xx_exit);

MODULE_LICENSE("GPL");

