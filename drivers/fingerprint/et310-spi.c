/*
 * Simple synchronous userspace interface to SPI devices
 *
 * Copyright (C) 2006 SWAPP
 *	Andrea Paterniani <a.paterniani@swapp-eng.it>
 * Copyright (C) 2007 David Brownell (simplification, cleanup)
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
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include "et310.h"
#include <linux/input.h>
#ifdef FP_SPI_DEBUG
#define DEBUG_PRINT(fmt, args...)	pr_err(fmt, ## args)
#else
/* Don't do anything in release builds */
#define DEBUG_PRINT(fmt, args...)
#endif

/* for TINY4412 platform's CON15 XEINT11 pin */
//#define egis_reset_gpio		112
/* for TINY4412 platform's CON15 XEINT12 pin */
#define TRIGGER_FPS_GPIO_PIN	115
#define FPC_BTP_SPI_CLOCK_SPEED			8000000
#define MASS_READ_TRANSMITION_SIZE 2304
#define EGIS_MASS_READ_SEGMENT_COUNT 10

static int egis_reset_gpio;
static int egis_int_gpio;
static int egis_power_3v3;
//static int egis_power_115;
//zhongzhu add  start =================
struct input_dev *egis_input_dev;  
static struct work_struct  egis_work;
static struct workqueue_struct * egis_workqueue = NULL;
static spinlock_t irq_lock;
unsigned int irq_gpio;
struct timer_list report_timer;
int  report_flag = 1;
int  fp_count = 0;
struct semaphore sem;
struct device *esfp0_dev;
//zhongzhu add  start =================

static DECLARE_BITMAP(minors, N_SPI_MINORS);

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

static unsigned bufsiz = 4096;
module_param(bufsiz, uint, S_IRUGO);
MODULE_PARM_DESC(bufsiz, "data bytes in biggest supported SPI message");

/*--------------------------- Data Transfer -----------------------------*/
static struct spi_transfer msg_xfer[EGIS_MASS_READ_SEGMENT_COUNT];
int fp_mass_read(struct fp_data *fp, u8 addr, u8 *buf, int read_len)
{
#if 0
	int status;
	struct spi_device *spi;
	struct spi_message m;
	u8 *write_addr;
	/* Set start address */
	u8 *read_data = kzalloc(read_len + 2, GFP_KERNEL);
	struct spi_transfer t_set_addr = {
		.tx_buf = NULL,
		.len = 2,
	};

	/* Write and read data in one data query section */
	struct spi_transfer t_read_data = {
		.tx_buf = NULL,
		.rx_buf = NULL,
		.len = read_len + 2,
	};

	if (read_data == NULL)
		return -ENOMEM;

	write_addr = kzalloc(2, GFP_KERNEL);
	write_addr[0] = FP_WRITE_ADDRESS;
	write_addr[1] = addr;

	DEBUG_PRINT("%s read_len = %d\n", __func__, read_len);

	read_data[0] = FP_READ_DATA;

	t_set_addr.tx_buf = write_addr;
	t_read_data.tx_buf = t_read_data.rx_buf = read_data;

	spi = fp->spi;

	spi_message_init(&m);
	spi_message_add_tail(&t_set_addr, &m);
	status = spi_sync(spi, &m);
	spi_message_init(&m);
	spi_message_add_tail(&t_read_data, &m);
//	spi->mode = 3;
	status = spi_sync(spi, &m);

	kfree(write_addr);

	if (status == 0)
		memcpy(buf, read_data + 2, read_len);
	else
		pr_err("%s read data error status = %d\n"
				, __func__, status);
	kfree(read_data);

	return status;
#endif
	int status = 0;
	struct spi_device *spi;
	struct spi_message m;
	u8 write_addr[] = {FP_WRITE_ADDRESS, addr};
	struct spi_transfer t_set_addr = {
		.tx_buf = write_addr,
		.len = 2,
	};
	u8 read_data[] = {FP_READ_DATA, 0x0, 0x0};
	struct spi_transfer t_read_data = {
		.tx_buf = read_data,
		.len = 3
	};
	int const msg_count = read_len / MASS_READ_TRANSMITION_SIZE;
	int const msg_remainder = read_len % MASS_READ_TRANSMITION_SIZE;
	int msg_index;

	spi = fp->spi;
	spi_message_init(&m);
	spi_message_add_tail(&t_set_addr, &m);
	status = spi_sync(spi, &m);
	if (status < 0) {
		pr_err("%s read data error status = %d", __func__, status);
		goto out;
	}

	/*Set read data*/

	spi_message_init(&m);
	spi_message_add_tail(&t_read_data, &m);

	/*Seperate msgs into transmition size*/
	//memset(msg_xfer, 0x00, sizeof(msg_xfer));
	memset(msg_xfer, 0x00, sizeof(struct spi_transfer)*EGIS_MASS_READ_SEGMENT_COUNT);
	for (msg_index = 0; msg_index < msg_count; msg_index++) {
		msg_xfer[msg_index].tx_buf = buf + msg_index * MASS_READ_TRANSMITION_SIZE;
		msg_xfer[msg_index].rx_buf = buf + msg_index * MASS_READ_TRANSMITION_SIZE;
		msg_xfer[msg_index].len = MASS_READ_TRANSMITION_SIZE;
		spi_message_add_tail(&msg_xfer[msg_index], &m);
	}
	if (msg_remainder) {
		msg_xfer[msg_index].tx_buf = buf + msg_index * MASS_READ_TRANSMITION_SIZE;
		msg_xfer[msg_index].rx_buf = buf + msg_index * MASS_READ_TRANSMITION_SIZE;
		msg_xfer[msg_index].len = msg_remainder;
		spi_message_add_tail(&msg_xfer[msg_index], &m);
	}

	status = spi_sync(spi, &m);
	if (status < 0) {
		pr_err("%s read data error status = %d", __func__, status);
		goto out;
	}

out:
	return status;
}

/*
 * Read io register
 */
int fp_io_read_register(struct fp_data *fp, u8 *addr, u8 *buf)
{
	int status = 0;
	struct spi_device *spi;
	struct spi_message m;
	int read_len = 1;

	u8 write_addr[] = {FP_WRITE_ADDRESS, 0x00};
	u8 read_value[] = {FP_READ_DATA, 0x00};
	u8 val, addrval;

	u8 result[] = {0xFF, 0xFF};
	struct spi_transfer t_set_addr = {
		.speed_hz = FPC_BTP_SPI_CLOCK_SPEED,
		.tx_buf = write_addr,
		.len = 2,
	};
	struct spi_transfer t = {
		.speed_hz = FPC_BTP_SPI_CLOCK_SPEED,
		.tx_buf = read_value,
		.rx_buf = result,
		.len = 2,
	};

	if (copy_from_user(&addrval, (const u8 __user *) (uintptr_t) addr
		, read_len)) {
		pr_err("%s buffer copy_from_user fail", __func__);
		status = -EFAULT;
		return status;
	}

	DEBUG_PRINT("%s read_len = %d\n", __func__, read_len);

	spi = fp->spi;

	/*Set address*/
	write_addr[1] = addrval;

	/*Start to read*/
	spi_message_init(&m);
	spi_message_add_tail(&t_set_addr, &m);
	status = spi_sync(spi, &m);
	if (status < 0) {
		pr_err("%s read data error status = %d\n"
				, __func__, status);
		return status;
	}
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
//	spi->mode = 3;
	status = spi_sync(spi, &m);
	if (status < 0) {
		pr_err("%s read data error status = %d\n"
				, __func__, status);
		return status;
	}

	val = result[1];

	DEBUG_PRINT("%s Read add_val = %x buf = 0x%x\n", __func__,
			addrval, val);

	if (copy_to_user((u8 __user *) (uintptr_t) buf, &val, read_len)) {
		pr_err("%s buffer copy_to_user fail status", __func__);
		status = -EFAULT;
		return status;
	}

	return status;
}

/*
 * Write data to register
 */
int fp_io_write_register(struct fp_data *fp, u8 *buf)
{
	int status = 0;
	struct spi_device *spi;
	int write_len = 2;
	struct spi_message m;

	u8 write_addr[] = {FP_WRITE_ADDRESS, 0x00};
	u8 write_value[] = {FP_WRITE_DATA, 0x00};
	u8 val[2];

	struct spi_transfer t1 = {
		.speed_hz = FPC_BTP_SPI_CLOCK_SPEED,
		.tx_buf = write_addr,
		.len = 2,
	};
	struct spi_transfer t2 = {
		.speed_hz = FPC_BTP_SPI_CLOCK_SPEED,
		.tx_buf = write_value,
		.len = 2,
	};

	if (copy_from_user(val, (const u8 __user *) (uintptr_t) buf
		, write_len)) {
		pr_err("%s buffer copy_from_user fail", __func__);
		status = -EFAULT;
		return status;
	}

	DEBUG_PRINT("%s write_len = %d\n", __func__, write_len);
	DEBUG_PRINT("%s address = %x data = %x\n", __func__, val[0], val[1]);

	spi = fp->spi;

	/*Set address*/
	write_addr[1] = val[0];
	/*Set value*/
	write_value[1] = val[1];

	spi_message_init(&m);
	spi_message_add_tail(&t1, &m);
	status = spi_sync(spi, &m);
	if (status < 0) {
		pr_err("%s write data error status = %d\n",
				__func__, status);
		return status;
	}
	spi_message_init(&m);
	spi_message_add_tail(&t2, &m);
//	spi->mode = 3;
	status = spi_sync(spi, &m);
	if (status < 0) {
		pr_err("%s write data error status = %d\n",
				__func__, status);
		return status;
	}

	return status;
}

int fp_read_register(struct fp_data *fp, u8 addr, u8 *buf)
{
	int status;
	struct spi_device *spi;
	struct spi_message m;

	/*Set address*/
	u8 write_addr[] = {FP_WRITE_ADDRESS, addr};
	u8 read_value[] = {FP_READ_DATA, 0x00};
	u8 result[] = {0xFF, 0xFF};

	struct spi_transfer t1 = {
		.speed_hz = FPC_BTP_SPI_CLOCK_SPEED,
		.tx_buf = write_addr,
		.len = 2,
	};
	struct spi_transfer t2 = {
		.speed_hz = FPC_BTP_SPI_CLOCK_SPEED,
		.tx_buf = read_value,
		.rx_buf = result,
		.len = 2,
	};

	spi = fp->spi;
	spi_message_init(&m);
	spi_message_add_tail(&t1, &m);
	status = spi_sync(spi, &m);

	if (status == 0) {
		*buf = result[1];
		DEBUG_PRINT("fp_read_register address = %x result = %x %x\n"
					, addr, result[0], result[1]);
	} else
		pr_err("%s read data error status = %d\n"
				, __func__, status);
	spi_message_init(&m);			
	spi_message_add_tail(&t2, &m);
//	spi->mode = 3;
	status = spi_sync(spi, &m);

	if (status == 0) {
		*buf = result[1];
		DEBUG_PRINT("fp_read_register address = %x result = %x %x\n"
					, addr, result[0], result[1]);
	} else
		pr_err("%s read data error status = %d\n"
				, __func__, status);

	return status;
}

int fp_io_get_one_image(
	struct fp_data *fp,
	u8 *buf,
	u8 *image_buf
	)
{
	uint8_t read_val,
			*tx_buf = (uint8_t *)buf,
			*work_buf = NULL,
			*val = kzalloc(6, GFP_KERNEL);
	int32_t status;
	uint32_t frame_not_ready_count = 0, read_count;

	DEBUG_PRINT("%s\n", __func__);

	if (val == NULL)
		return -ENOMEM;

	if (copy_from_user(val, (const u8 __user *) (uintptr_t) tx_buf, 6)) {
		pr_err("%s buffer copy_from_user fail\n", __func__);
		status = -EFAULT;
		goto end;
	}

	/* total pixel , width * hight */
	read_count = val[0] * val[1];

	while (1) {
		status = fp_read_register
				(fp, FSTATUS_FP_ADDR, &read_val);
		if (status < 0)
			goto end;

		if (read_val & FRAME_READY_MASK)
			break;

		if (frame_not_ready_count >= 250) {
			pr_err("frame_not_ready_count = %d\n",
					frame_not_ready_count);
			break;
		}
		frame_not_ready_count++;
	}

	work_buf = kzalloc(read_count, GFP_KERNEL);
	if (work_buf == NULL) {
		status = -ENOMEM;
		goto end;
	}
	status = fp_mass_read(fp, FDATA_FP_ADDR, work_buf, read_count);
	if (status < 0) {
		pr_err("%s call fp_mass_read error status = %d\n"
				, __func__, status);
		goto end;
	}

	if (copy_to_user((u8 __user *) (uintptr_t) image_buf,
		work_buf, read_count)) {
		pr_err("buffer copy_to_user fail status = %d\n", status);
		status = -EFAULT;
	}
end:
	kfree(val);
	kfree(work_buf);
	return status;
}

/*----------------------- EEPROM ------------------------*/

int fp_eeprom_wren(struct fp_data *fp)
{
	int status = 0;
	struct spi_device *spi;
	struct spi_message m;

	u8 write_data[] = {FP_EEPROM_WREN_OP};
	struct spi_transfer t = {
		.speed_hz = FPC_BTP_SPI_CLOCK_SPEED,
		.tx_buf = write_data,
		.len = 1,
	};

	DEBUG_PRINT("%s opcode = %x\n", __func__, write_data[0]);

	spi = fp->spi;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
//	spi->mode = 3;
	status = spi_sync(spi, &m);
	if (status < 0) {
		pr_err("%s spi_sync error status = %d\n",
				__func__, status);
		return status;
	}

	return status;
}

int fp_eeprom_wrdi(struct fp_data *fp)
{
	int status = 0;
	struct spi_device *spi;
	struct spi_message m;

	u8 write_data[] = {FP_EEPROM_WRDI_OP};
	struct spi_transfer t = {
		.speed_hz = FPC_BTP_SPI_CLOCK_SPEED,
		.tx_buf = write_data,
		.len = 1,
	};

	DEBUG_PRINT("%s opcode = %x\n", __func__, write_data[0]);

	spi = fp->spi;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
//	spi->mode = 3;
	status = spi_sync(spi, &m);
	if (status < 0) {
		pr_err("%s spi_sync error status = %d\n",
				__func__, status);
		return status;
	}

	return status;
}

int fp_eeprom_rdsr(struct fp_data *fp, u8 *buf)
{
	int status;
	struct spi_device *spi;
	struct spi_message m;
	u8 val,
	   read_value[] = {FP_EEPROM_RDSR_OP, 0x00},
	   result[] = {0xFF, 0xFF};

	struct spi_transfer t = {
		.speed_hz = FPC_BTP_SPI_CLOCK_SPEED,
		.tx_buf = read_value,
		.rx_buf = result,
		.len = 2,
	};

	spi = fp->spi;
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
//	spi->mode = 3;
	status = spi_sync(spi, &m);
	if (status < 0) {
		pr_err("%s spi_sync error status = %d\n",
				__func__, status);
		return status;
	}

	val = result[1];

	DEBUG_PRINT("%s address = %x buf = %x\n", __func__,
			FP_EEPROM_RDSR_OP, val);

	if (copy_to_user((u8 __user *) (uintptr_t) buf, &val, 1)) {
		pr_err("%s buffer copy_to_user fail status\n", __func__);
		status = -EFAULT;
		return status;
	}

	return status;
}

int fp_eeprom_wrsr(struct fp_data *fp, u8 *buf)
{
	int status;
	struct spi_device *spi;
	struct spi_message m;
	u8 val;

	u8 write_data[] = {FP_EEPROM_WRSR_OP, 0x00};

	struct spi_transfer t = {
		.speed_hz = FPC_BTP_SPI_CLOCK_SPEED,
		.tx_buf = write_data,
		.len = 2,
	};

	if (copy_from_user(&val, (const u8 __user *) (uintptr_t) buf
		, 1)) {
		pr_err("%s buffer copy_from_user fail\n", __func__);
		status = -EFAULT;
		return status;
	}

	DEBUG_PRINT("%s data = %x\n", __func__, val);

	spi = fp->spi;

	write_data[1] = val;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
//	spi->mode = 3;
	status = spi_sync(spi, &m);
	if (status < 0) {
		pr_err("%s spi_sync error status = %d\n",
				__func__, status);
		return status;
	}

	return status;
}

int fp_eeprom_read(struct fp_data *fp, u8 *addr, u8 *buf, int read_len)
{
	int status = 0;
	struct spi_device *spi;
	struct spi_message m;
	u8 addrval, *read_value = kzalloc(read_len + 2, GFP_KERNEL);

	struct spi_transfer t = {
		.speed_hz = FPC_BTP_SPI_CLOCK_SPEED,
		.tx_buf = NULL,
		.rx_buf = NULL,
		.len = read_len + 2,
	};

	if (read_value == NULL)
		return -ENOMEM;

	if (copy_from_user(&addrval, (const u8 __user *) (uintptr_t) addr
		, 1)) {
		pr_err("%s buffer copy_from_user fail\n", __func__);
		status = -EFAULT;
		goto exit;
	}

	DEBUG_PRINT("%s read_len = %d\n", __func__, read_len);
	DEBUG_PRINT("%s addrval = %x\n", __func__, addrval);

	spi = fp->spi;

	read_value[0] = FP_EEPROM_READ_OP;
	read_value[1] = addrval;

	t.tx_buf = t.rx_buf = read_value;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
//	spi->mode = 3;
	status = spi_sync(spi, &m);
	if (status < 0) {
		pr_err("%s spi_sync error status = %d\n"
				, __func__, status);
		goto exit;
	}

	if (copy_to_user((u8 __user *) (uintptr_t) buf,
				read_value + 2, read_len)) {
		pr_err("%s buffer copy_to_user fail status\n", __func__);
		status = -EFAULT;
		goto exit;
	}

exit:
	kfree(read_value);

	return status;
}

/*
 * buf - the data wrote to sensor with address info
 * write_len - the length of the data write to memory without address
 */
int fp_eeprom_write(struct fp_data *fp, u8 *buf, int write_len)
{
	int status = 0;
	struct spi_device *spi;
	struct spi_message m;

	u8 *write_value = kzalloc(write_len + 2, GFP_KERNEL);

	struct spi_transfer t = {
		.speed_hz = FPC_BTP_SPI_CLOCK_SPEED,
		.tx_buf = NULL,
		.len = write_len + 2,
	};

	if (write_value == NULL)
		return -ENOMEM;

	write_value[0] = FP_EEPROM_WRITE_OP;

	if (copy_from_user(write_value + 1, (const u8 __user *) (uintptr_t) buf
		, write_len + 1)) {
		pr_err("%s buffer copy_from_user fail\n", __func__);
		status = -EFAULT;
		goto exit;
	}

	DEBUG_PRINT("%s write_len = %d\n", __func__, write_len);
	DEBUG_PRINT("%s address = %x\n", __func__, write_value[1]);

	spi = fp->spi;

	t.tx_buf = write_value;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
//	spi->mode = 3;
	status = spi_sync(spi, &m);
	if (status < 0) {
		pr_err("%s read data error status = %d\n",
				__func__, status);
		goto exit;
	}

exit:
	kfree(write_value);

	return status;
}

/* ------------------------------ Interrupt -----------------------------*/

/*
 * Interrupt description
 */

#define FP_INT_DETECTION_PERIOD  10
#define FP_DETECTION_THRESHOLD	10

/*
 * FPS interrupt table
 */

struct interrupt_desc fps_ints[] = {
	{TRIGGER_FPS_GPIO_PIN , 0, "BUT0" , 0} /* TINY4412CON15 XEINT12 pin */
};
int fps_ints_size = ARRAY_SIZE(fps_ints);

static DECLARE_WAIT_QUEUE_HEAD(interrupt_waitq);

/*
 *	FUNCTION NAME.
 *		interrupt_timer_routine
 *
 *	FUNCTIONAL DESCRIPTION.
 *		basic interrupt timer inital routine
 *
 *	ENTRY PARAMETERS.
 *		gpio - gpio address
 *
 *	EXIT PARAMETERS.
 *		Function Return
 */

void interrupt_timer_routine(
	unsigned long _data
)
{
	struct interrupt_desc *bdata = (struct interrupt_desc *)_data;
	pr_info("FPS interrupt count = %d" , bdata->int_count);
	if (bdata->int_count >= bdata->detect_threshold) {
		bdata->finger_on = 1;
		pr_info("FPS triggered !!!!!!!\n");
	} else
		pr_info("FPS not triggered !!!!!!!\n");
	bdata->int_count = 0;
	wake_up_interruptible(&interrupt_waitq);
}

/*
 *	FUNCTION NAME.
 *		fingerprint_interrupt
 *
 *	FUNCTIONAL DESCRIPTION.
 *		finger print interrupt callback routine
 *
 *	ENTRY PARAMETERS.
 *		irq
 *		dev_id
 *
 *	EXIT PARAMETERS.
 *		Function Return
 */

static irqreturn_t fingerprint_interrupt(
	int irq,
	void *dev_id
)
{
	struct interrupt_desc *bdata = (struct interrupt_desc *)dev_id;
	if (!bdata->int_count)
		mod_timer(&bdata->timer,
			jiffies + msecs_to_jiffies(bdata->detect_period));
	bdata->int_count++;
	return IRQ_HANDLED;
}

/*
 *	FUNCTION NAME.
 *		Interrupt_Init
 *
 *	FUNCTIONAL DESCRIPTION.
 *		button initial routine
 *
 *	ENTRY PARAMETERS.
 *		gpio - gpio address
 *
 *	EXIT PARAMETERS.
 *		Function Return int
 */

int Interrupt_Init(int int_num, int detect_period, int detect_threshold)
{
	int i, irq, err = 0;

	pr_info("FP %s int_num = %d detect_period = %d detect_threshold = %d\n",
				__func__,
				int_num,
				detect_period,
				detect_threshold);

	for (i = 0; i < ARRAY_SIZE(fps_ints); i++) {
		fps_ints[i].int_count = 0;
		fps_ints[i].finger_on = 0;
		if (i == int_num) {
			fps_ints[i].detect_period = detect_period;
			fps_ints[i].detect_threshold = detect_threshold;
		} else {
			fps_ints[i].detect_period = FP_INT_DETECTION_PERIOD;
			fps_ints[i].detect_threshold = FP_DETECTION_THRESHOLD;
		}
		if (!fps_ints[i].gpio)
			continue;

		/*
		 * set the IRQ function and GPIO.
		 * then setting the interrupt trigger type
		 */
		irq = gpio_to_irq(fps_ints[i].gpio);
		err = request_irq(
				irq, fingerprint_interrupt,
				IRQ_TYPE_EDGE_RISING,
				fps_ints[i].name, (void *)&fps_ints[i]);
		if (err)
			break;

	}

	if (err) {
		i--;
		for (; i >= 0; i--) {
			if (!fps_ints[i].gpio)
				continue;
			irq = gpio_to_irq(fps_ints[i].gpio);
			disable_irq(irq);
			free_irq(irq, (void *)&fps_ints[i]);
			del_timer_sync(&fps_ints[i].timer);
		}
		return -EBUSY;
	}
	return 0;
}

/*
 *	FUNCTION NAME.
 *		Interrupt_Free
 *
 *	FUNCTIONAL DESCRIPTION.
 *		free all interrupt resource
 *
 *	ENTRY PARAMETERS.
 *		gpio - gpio address
 *
 *	EXIT PARAMETERS.
 *		Function Return int
 */

int Interrupt_Free(void)
{
	int irq, i;

	for (i = 0; i < ARRAY_SIZE(fps_ints); i++) {
		if (!fps_ints[i].gpio)
			continue;
		irq = gpio_to_irq(fps_ints[i].gpio);
		free_irq(irq, (void *)&fps_ints[i]);

		del_timer_sync(&fps_ints[i].timer);
	}

	return 0;
}

/*
 *	FUNCTION NAME.
 *		fps_interrupt_re d
 *
 *	FUNCTIONAL DESCRIPTION.
 *		FPS interrupt read status
 *
 *	ENTRY PARAMETERS.
 *		wait poll table structure
 *
 *	EXIT PARAMETERS.
 *		Function Return int
 */

unsigned int fps_interrupt_poll(
	struct file *file,
	struct poll_table_struct *wait
)
{
	unsigned int mask = 0;
	int i = 0;

	pr_info("%s\n", __func__);

	for (i = 0; i < fps_ints_size; i++)
		fps_ints[i].int_count = 0;
	poll_wait(file, &interrupt_waitq, wait);
	for (i = 0; i < fps_ints_size; i++) {
		if (fps_ints[i].finger_on) {
			mask |= POLLIN | POLLRDNORM;
			fps_ints[i].finger_on = 0;
		}
	}
	return mask;
}

void fps_interrupt_abort(void)
{
	int i = 0;
	for (i = 0; i < fps_ints_size; i++)
		fps_ints[i].finger_on = 0;
	wake_up_interruptible(&interrupt_waitq);
}

/*-------------------------------------------------------------------------*/

static void fp_reset(void)
{
	pr_info("%s\n", __func__);
	gpio_set_value(egis_reset_gpio, 0);
	msleep(30);
	gpio_set_value(egis_reset_gpio, 1);
	msleep(20);
}

static void fp_reset_set(int enable)
{
	pr_info("%s\n", __func__);
	pr_info("%s enable %d\n", __func__, enable);
	if (enable == 0) {
		gpio_set_value(egis_reset_gpio, 0);
		msleep(30);
	} else {
		gpio_set_value(egis_reset_gpio, 1);
		msleep(20);
	}
}

static ssize_t fp_read(struct file *filp,
						char __user *buf,
						size_t count,
						loff_t *f_pos)
{
	/*Implement by vendor if needed*/
	return 0;
}

static ssize_t fp_write(struct file *filp,
						const char __user *buf,
						size_t count,
						loff_t *f_pos)
{
	/*Implement by vendor if needed*/
	return 0;
}

static long fp_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	int retval = 0;
	struct fp_data *fp;
	struct spi_device *spi;
	u32 tmp;
	struct egis_ioc_transfer *ioc = NULL;

	/* Check type and command number */
	if (_IOC_TYPE(cmd) != EGIS_IOC_MAGIC) {
		pr_err("%s _IOC_TYPE(cmd) != EGIS_IOC_MAGIC", __func__);
		return -ENOTTY;
	}
	DEBUG_PRINT("==========%s,%d,cmd=%d\n", __func__,__LINE__,cmd);
	/*
	 * Check access direction once here; don't repeat below.
	 * IOC_DIR is from the user perspective, while access_ok is
	 * from the kernel perspective; so they look reversed.
	 */
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE,
						(void __user *)arg,
						_IOC_SIZE(cmd));
	if (err == 0 && _IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ,
						(void __user *)arg,
						_IOC_SIZE(cmd));
	if (err) {
		pr_err("%s err", __func__);
		return -EFAULT;
	}

	/*
	 * guard against device removal before, or while,
	 * we issue this ioctl.
	 */
	fp = filp->private_data;
	spin_lock_irq(&fp->spi_lock);
	spi = spi_dev_get(fp->spi);
	spin_unlock_irq(&fp->spi_lock);

	if (spi == NULL) {
		pr_err("%s spi == NULL", __func__);
		return -ESHUTDOWN;
	}

	mutex_lock(&fp->buf_lock);

	/* segmented and/or full-duplex I/O request */
	if (_IOC_NR(cmd) != _IOC_NR(EGIS_IOC_MESSAGE(0))
					|| _IOC_DIR(cmd) != _IOC_WRITE) {
		retval = -ENOTTY;
		DEBUG_PRINT("==========%s,%d\n", __func__,__LINE__);
		goto out;
	}

	tmp = _IOC_SIZE(cmd);
	if ((tmp == 0) || (tmp % sizeof(struct egis_ioc_transfer)) != 0) {
		retval = -EINVAL;
		DEBUG_PRINT("==========%s,%d,tmp=%d\n", __func__,__LINE__,tmp);
		goto out;
	}

	/* copy into scratch area */
	ioc = kmalloc(tmp, GFP_KERNEL);
	if (ioc == NULL) {
		retval = -ENOMEM;
		DEBUG_PRINT("==========%s,%d\n", __func__,__LINE__);
		goto out;
	}
	if (__copy_from_user(ioc, (void __user *)arg, tmp)) {
		retval = -EFAULT;
		DEBUG_PRINT("==========%s,%d\n", __func__,__LINE__);
		goto out;
	}

	/*
	 * Read register
	 * tx_buf include register address will be read
	 */
	if (ioc->opcode == FP_REGISTER_READ) {
		u8 *address = ioc->tx_buf;
		u8 *result = ioc->rx_buf;
		DEBUG_PRINT("fp FP_REGISTER_READ\n");
		DEBUG_PRINT("==========%s,%d\n", __func__,__LINE__);
		retval = fp_io_read_register(fp, address, result);
		if (retval < 0)	{
			pr_err("%s FP_REGISTER_READ error retval = %d\n"
			, __func__, retval);
			goto out;
		}
	}

	/*
	 * Write data to register
	 * tx_buf includes address and value will be wrote
	 */
	if (ioc->opcode == FP_REGISTER_WRITE) {
		u8 *buf = ioc->tx_buf;
		DEBUG_PRINT("fp FP_REGISTER_WRITE");

		retval = fp_io_write_register(fp, buf);
		if (retval < 0) {
			pr_err("%s FP_REGISTER_WRITE error retval = %d\n"
			, __func__, retval);
			goto out;
		}
	}

	/*
	 * Get one frame data from sensor
	 */
	if (ioc->opcode == FP_GET_ONE_IMG) {
		u8 *buf = ioc->tx_buf;
		u8 *image_buf = ioc->rx_buf;
		DEBUG_PRINT("fp FP_GET_ONE_IMG\n");

		retval = fp_io_get_one_image(fp, buf, image_buf);
		if (retval < 0) {
			pr_err("%s FP_GET_ONE_IMG error retval = %d\n"
			, __func__, retval);
			goto out;
		}
	}

	if (ioc->opcode == FP_SENSOR_RESET)
	{	
		DEBUG_PRINT("fp FP_SENSOR_RESET\n");
		fp_reset();
	}
	if (ioc->opcode == FP_RESET_SET) {
		pr_info("%s FP_SENSOR_RESET\n", __func__);
		pr_info("%s status = %d\n", __func__, ioc->len);
		fp_reset_set(ioc->len);
	}

	if (ioc->opcode == FP_SET_SPI_CLOCK) {
		__u32 current_speed = spi->max_speed_hz;
		pr_info("%s FP_SET_SPI_CLOCK\n", __func__);
		pr_info("%s speed_hz = %d\n", __func__, ioc->speed_hz);
		pr_info("%s current_speed = %d\n", __func__, current_speed);

		spi->max_speed_hz = ioc->speed_hz;
		retval = spi_setup(spi);
		if (retval < 0) {
			pr_err("%s spi_setup error %d\n", __func__, retval);
			spi->max_speed_hz = current_speed;
		}
		pr_info("%s spi speed_hz = %d\n", __func__, spi->max_speed_hz);
	}

	if (ioc->opcode == FP_EEPROM_WREN) {
		pr_info("%s FP_EEPROM_WREN\n", __func__);
		fp_reset_set(0);
		fp_eeprom_wren(fp);
		fp_reset_set(1);
	}

	if (ioc->opcode == FP_EEPROM_WRDI) {
		pr_info("%s FP_EEPROM_WRDI\n", __func__);
		fp_reset_set(0);
		fp_eeprom_wrdi(fp);
		fp_reset_set(1);
	}

	if (ioc->opcode == FP_EEPROM_RDSR) {
		u8 *result = ioc->rx_buf;
		pr_info("%s FP_EEPROM_RDSR\n", __func__);
		fp_reset_set(0);
		fp_eeprom_rdsr(fp, result);
		fp_reset_set(1);
	}

	if (ioc->opcode == FP_EEPROM_WRSR) {
		u8 *buf = ioc->tx_buf;
		pr_info("%s FP_EEPROM_WRSR\n", __func__);
		fp_reset_set(0);
		fp_eeprom_wrsr(fp, buf);
		fp_reset_set(1);
	}

	if (ioc->opcode == FP_EEPROM_READ) {
		u8 *buf = ioc->tx_buf;
		u8 *result = ioc->rx_buf;
		pr_info("%s FP_EEPROM_READ\n", __func__);
		fp_reset_set(0);
		fp_eeprom_read(fp, buf, result, ioc->len);
		fp_reset_set(1);
	}

	if (ioc->opcode == FP_EEPROM_WRITE) {
		u8 *buf = ioc->tx_buf;
		pr_info("%s FP_EEPROM_WRITE\n", __func__);
		fp_reset_set(0);
		fp_eeprom_write(fp, buf, ioc->len);
		fp_reset_set(1);
	}

	if (ioc->opcode == FP_POWER_ONOFF)
		pr_info("power control status = %d\n", ioc->len);

	/*
	 * Trigger inital routine
	 */
	if (ioc->opcode == INT_TRIGGER_INIT) {
		pr_info(">>> fp Trigger function init\n");
		retval = Interrupt_Init(
				(int)ioc->pad[0],
				(int)ioc->pad[1],
				(int)ioc->pad[2]);
		pr_info("trigger init = %d\n", retval);
	}

	/*
	 * trigger
	 */
	if (ioc->opcode == INT_TRIGGER_CLOSE) {
		pr_info("<<< fp Trigger function close\n");
		retval = Interrupt_Free();
		pr_info("trigger close = %d\n", retval);
	}

	/*
	 * read interrupt status
	 */
	if (ioc->opcode == INT_TRIGGER_ABORT)
		fps_interrupt_abort();

out:
	if (ioc != NULL)
		kfree(ioc);

	mutex_unlock(&fp->buf_lock);
	spi_dev_put(spi);
	if (retval < 0)
		pr_err("%s retval = %d", __func__, retval);
	return retval;
}

#ifdef CONFIG_COMPAT
static long fp_compat_ioctl(struct file *filp,
	unsigned int cmd,
	unsigned long arg)
{
	return fp_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#else
#define fp_compat_ioctl NULL
#endif /* CONFIG_COMPAT */

static int fp_open(struct inode *inode, struct file *filp)
{
	struct fp_data *fp;
	int			status = -ENXIO;

	DEBUG_PRINT("%s\n", __func__);
	mutex_lock(&device_list_lock);

	list_for_each_entry(fp, &device_list, device_entry)
	{
		if (fp->devt == inode->i_rdev) {
			status = 0;
			break;
		}
	}
	if (status == 0) {
		if (fp->buffer == NULL) {
			fp->buffer = kmalloc(bufsiz, GFP_KERNEL);
			if (fp->buffer == NULL) {
				dev_dbg(&fp->spi->dev, "open/ENOMEM\n");
				status = -ENOMEM;
			}
		}
		if (status == 0) {
			fp->users++;
			filp->private_data = fp;
			nonseekable_open(inode, filp);
		}
	} else
		pr_debug("fp: nothing for minor %d\n", iminor(inode));

	mutex_unlock(&device_list_lock);
	return status;
}

static int fp_release(struct inode *inode, struct file *filp)
{
	struct fp_data *fp;

	DEBUG_PRINT("%s\n", __func__);
	mutex_lock(&device_list_lock);
	fp = filp->private_data;
	filp->private_data = NULL;

	/* last close? */
	fp->users--;
	if (fp->users == 0) {
		int	dofree;

		kfree(fp->buffer);
		fp->buffer = NULL;

		/* ... after we unbound from the underlying device? */
		spin_lock_irq(&fp->spi_lock);
		dofree = (fp->spi == NULL);
		spin_unlock_irq(&fp->spi_lock);

		if (dofree)
			kfree(fp);
	}
	mutex_unlock(&device_list_lock);

	return 0;
}

static const struct file_operations fp_fops = {
	.owner = THIS_MODULE,
	.write = fp_write,
	.read = fp_read,
	.unlocked_ioctl = fp_ioctl,
	.compat_ioctl = fp_compat_ioctl,
	.open = fp_open,
	.release = fp_release,
	.llseek = no_llseek,
	.poll = fps_interrupt_poll
};

static int egis_parse_dt(struct device *dev)
{
    struct device_node *np = dev->of_node;
	unsigned int egis_power_3v3_flags;
	unsigned int egis_reset_gpio_flags;
	unsigned int egis_int_gpio_flags;
	
	DEBUG_PRINT("%s\n", __func__);
	egis_power_3v3 = of_get_named_gpio_flags(np, "egis,power-gpio",    0, &egis_power_3v3_flags);
	if(egis_power_3v3 < 0)
		DEBUG_PRINT("egis_power_3v3 request error,%s\n", __func__);
	else
	{
  		if(gpio_request(egis_power_3v3, "fppower"))
			DEBUG_PRINT("egis_power_3v3 request error,%s\n", __func__);			
		else	
			gpio_direction_output(egis_power_3v3, 1);
	}
	DEBUG_PRINT("==========%s,%d\n", __func__,__LINE__);
	
    egis_reset_gpio = of_get_named_gpio_flags(np, "egis,reset-gpio",    0, &egis_reset_gpio_flags);
	if (egis_reset_gpio < 0)
		DEBUG_PRINT("egis_reset_gpio request error,%s\n", __func__);
	else
	{
  		if(gpio_request(egis_reset_gpio, "fpreset"))
			DEBUG_PRINT("egis_reset_gpio request error,%s\n", __func__);
		else			
			gpio_direction_output(egis_reset_gpio, 1);
	}
	
	egis_int_gpio = of_get_named_gpio_flags(np, "egis,int-gpio",    0, &egis_int_gpio_flags);
	if (egis_int_gpio < 0)
		DEBUG_PRINT("egis_int_gpio request error,%s\n", __func__);
    else
	{
  		if(gpio_request(egis_int_gpio, "fpint"))
			DEBUG_PRINT("egis_int_gpio request error,%s\n", __func__);
		else	
			DEBUG_PRINT("egis_int_gpio request ok,%s\n", __func__);
	}
	
    return 0;
}

/*-------------------------------------------------------------------------*/
void egis_irq_disable(void)
{
	unsigned long irqflags;

	spin_lock_irqsave(&irq_lock, irqflags);
	disable_irq_wake(irq_gpio);
	spin_unlock_irqrestore(&irq_lock, irqflags);
}
void egis_irq_enable(void)
{
	unsigned long irqflags = 0;

	spin_lock_irqsave(&irq_lock, irqflags);
	enable_irq_wake(irq_gpio);
	spin_unlock_irqrestore(&irq_lock, irqflags);
}
void report_timer_deal(unsigned long data)
{	
//	printk("report_timer_deal set flag to 1 ======\n");
	report_flag = 1;
	fp_count = 0;
//	egis_irq_enable();
	
}
static void egis_work_func(struct work_struct *work)
{		
//		egis_irq_disable();
	down(&sem);
	fp_count++;
	if(report_flag)
	{
		if(fp_count > 6)
		{
		#if 0
			input_report_key(egis_input_dev, KEY_FP1, 1);
			input_sync(egis_input_dev);
			input_report_key(egis_input_dev, KEY_FP1, 0);
			input_sync(egis_input_dev);		
		#else
			kobject_uevent(&esfp0_dev->kobj, KOBJ_CHANGE);
//			printk("%s\n",kobject_get_path(&esfp0_dev->kobj,GFP_KERNEL));
		#endif
			report_flag = 0;
			mod_timer(&report_timer, jiffies + msecs_to_jiffies(10));
//			printk("report key ====== count = %d\n", fp_count);			
		}
	}
//	printk("egis_work_func======\n");
	up(&sem);
}

static irqreturn_t egis_irq_handler(int irq, void *dev_id)
{
//	DEBUG_PRINT("irq occur==========%s,%d\n", __func__,__LINE__);		
	queue_work(egis_workqueue, &egis_work);
	return IRQ_HANDLED;
}

static struct class *fp_class;

/*-------------------------------------------------------------------------*/
//static int __devinit fp_probe(struct spi_device *spi)
static int  fp_probe(struct spi_device *spi)
{
	struct fp_data *fp;
	int status;
	unsigned long minor;
	int i;
	struct spi_device *spi_initialize;
	int err;
	struct device *dev;	
	
	DEBUG_PRINT("%s\n", __func__);
	DEBUG_PRINT("==========%s,%d\n", __func__,__LINE__);
//zhongzhu add start ==========	
	egis_workqueue  = create_singlethread_workqueue("fp_wq");
	INIT_WORK(&egis_work, egis_work_func);
	spin_lock_init(&irq_lock);
//zhongzhu add end ==========		
	/* Allocate driver data */
	fp = kzalloc(sizeof(*fp), GFP_KERNEL);
	if (fp == NULL)
		return -ENOMEM;

	/* Initialize the driver data */
//	spi->mode = 3;
	fp->spi = spi;
	//spi_setup(spi);
	spin_lock_init(&fp->spi_lock);
	sema_init(&sem, 1);
	mutex_init(&fp->buf_lock);
	DEBUG_PRINT("==========%s,%d\n", __func__,__LINE__);
	INIT_LIST_HEAD(&fp->device_entry);

	/*
	 * If we can allocate a minor number, hook up this device.
	 * Reusing minors is fine so long as udev or mdev is working.
	 */
	mutex_lock(&device_list_lock);
	minor = find_first_zero_bit(minors, N_SPI_MINORS);
	if (minor < N_SPI_MINORS) {
		
		fp->devt = MKDEV(FP_MAJOR, minor);
		dev = device_create(fp_class, &spi->dev, fp->devt,
					fp, "esfp0");
		status = IS_ERR(dev) ? PTR_ERR(dev) : 0;
	} else{
		dev_dbg(&spi->dev, "no minor number available!\n");
		status = -ENODEV;
	}
	if (status == 0) {
		set_bit(minor, minors);
		list_add(&fp->device_entry, &device_list);
	}
	mutex_unlock(&device_list_lock);
	DEBUG_PRINT("==========%s,%d\n", __func__,__LINE__);
	
	egis_parse_dt(&spi->dev);	
	spin_lock_irq(&fp->spi_lock);
	spi_initialize= spi_dev_get(fp->spi);
	spin_unlock_irq(&fp->spi_lock);	

	spi_initialize->mode = SPI_MODE_3;
	spi_initialize->bits_per_word = 8;
	printk("%s spi port using  : %s\n", __func__, spi_initialize->modalias);

	spi_setup(spi_initialize);
	esfp0_dev = dev;
	fp_reset();

	if (status == 0)
		spi_set_drvdata(spi, fp);
	else
		kfree(fp);
//zhongzhu add irq start	

	err = irq_gpio = gpio_to_irq(egis_int_gpio);
	if (err < 0)
		DEBUG_PRINT("gpio_to_irq failed==========%s,%d\n", __func__,__LINE__);
	err = request_irq(irq_gpio, egis_irq_handler, IRQF_TRIGGER_RISING , "egis-irq", NULL);
	if (err)
		DEBUG_PRINT("request_irq failed==========%s,%d\n", __func__,__LINE__);	
	
	egis_irq_disable();
	
	
	egis_input_dev = input_allocate_device();
	if (!egis_input_dev) {		
		DEBUG_PRINT("==========failed to allocate input device %s,%d\n", __func__,__LINE__);
		return -ENOMEM;
	}	

	egis_input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY);
	input_set_capability(egis_input_dev, EV_KEY, KEY_FP1);
	input_set_capability(egis_input_dev, EV_KEY, KEY_FP1);
	egis_input_dev->name = "egis-fingerprint sensor";;
	 err = input_register_device(egis_input_dev);
   	 if (err)
    	{
       	 printk("========Register %s input device failed\n", egis_input_dev->name);
		 input_free_device(egis_input_dev);
        	return -ENODEV;
   	 }
	egis_irq_enable();
	setup_timer(&report_timer, report_timer_deal,0);
//zhongzhu add irq end 
	
	for (i = 0; i < fps_ints_size; i++) {
		setup_timer(&fps_ints[i].timer, interrupt_timer_routine,
					(unsigned long)&fps_ints[i]);
	}
	DEBUG_PRINT("==========%s,%d\n", __func__,__LINE__);
	
	return status;
}
//static int __devexit fp_remove(struct spi_device *spi)
static int  fp_remove(struct spi_device *spi)
{
	struct fp_data *fp = spi_get_drvdata(spi);
	DEBUG_PRINT("%s\n", __func__);

	/* make sure ops on existing fds can abort cleanly */
	spin_lock_irq(&fp->spi_lock);
	fp->spi = NULL;
	spi_set_drvdata(spi, NULL);
	spin_unlock_irq(&fp->spi_lock);
//zhongzhu add start =====
	destroy_workqueue(egis_workqueue);
	input_unregister_device(egis_input_dev);
//zhongzhu add end =====
	/* prevent new opens */
	mutex_lock(&device_list_lock);
	list_del(&fp->device_entry);
	device_destroy(fp_class, fp->devt);
	clear_bit(MINOR(fp->devt), minors);
	if (fp->users == 0)
		kfree(fp);
	input_unregister_device(egis_input_dev);  //zhongzhu add
	mutex_unlock(&device_list_lock);

	return 0;
}

static struct spi_driver fp_spi_driver = {
	.driver = {
		.name = "spidev",
		.owner = THIS_MODULE,
		.bus	= &spi_bus_type,
	},
	.probe = fp_probe,
	.remove = fp_remove,
};

/*-------------------------------------------------------------------------*/

static int __init fp_init(void)
{
	int status;
	DEBUG_PRINT("%s\n", __func__);

	/*
	 * Claim our 256 reserved device numbers.  Then register a class
	 * that will key udev/mdev to add/remove /dev nodes.  Last, register
	 * the driver which manages those device numbers.
	 */
	BUILD_BUG_ON(N_SPI_MINORS > 256);
	status = register_chrdev(FP_MAJOR, "spi", &fp_fops);
	if (status < 0)
		return status;
	fp_class = class_create(THIS_MODULE, "fp");
	if (IS_ERR(fp_class)) {
		unregister_chrdev(FP_MAJOR, fp_spi_driver.driver.name);
		return PTR_ERR(fp_class);
	}	
	status = spi_register_driver(&fp_spi_driver);	
	if (status < 0) {
		class_destroy(fp_class);
		unregister_chrdev(FP_MAJOR, fp_spi_driver.driver.name);
	}	
	return status;
}
module_init(fp_init);

static void __exit fp_exit(void)
{
	DEBUG_PRINT("%s\n", __func__);
	gpio_free(egis_reset_gpio);
	spi_unregister_driver(&fp_spi_driver);
	class_destroy(fp_class);
	unregister_chrdev(FP_MAJOR, fp_spi_driver.driver.name);
}
module_exit(fp_exit);

MODULE_AUTHOR("Wang YuWei, <robert.wang@egistec.com>");
MODULE_DESCRIPTION("SPI Interface for ET310");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:spidev");
