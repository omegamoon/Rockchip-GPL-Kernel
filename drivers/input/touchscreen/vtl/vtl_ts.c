/*
 * VTL CTP driver
 *
 * Copyright (C) 2013 VTL Corporation.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/gpio.h>


//#include <linux/ctp.h>//allwiner
//extern struct ctp_config_info config_info;//allwiner

#include "vtl_ts.h"
#include "ct365.h"


// ****************************************************************************
// Globel or static variables
// ****************************************************************************
struct ts_driver	g_driver;

static struct ts_info	g_ts = {

	.driver = &g_driver,
};
static struct ts_info	*pg_ts = &g_ts;

static struct i2c_device_id vtl_ts_id[] = {
	{ DRIVER_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c,vtl_ts_id);
/*
static struct i2c_board_info i2c_info[] = {
	{
		I2C_BOARD_INFO(DRIVER_NAME, 0x01),
		.platform_data = NULL,
	},
};
*/

// ****************************************************************************
// Function declaration
// ****************************************************************************
/*
void ct36x_ts_reg_read(struct i2c_client *client, unsigned short addr, char *buf, int len)
{
	struct i2c_msg msgs;

	msgs.addr = addr;
	msgs.flags = 0x01;  // 0x00: write 0x01:read 
	msgs.len = len;
	msgs.buf = buf;
	#if(PLATFORM == ROCKCHIP)
	msgs.scl_rate = TS_I2C_SPEED;
	#endif

	i2c_transfer(client->adapter, &msgs, 1);
}

void ct36x_ts_reg_write(struct i2c_client *client, unsigned short addr, char *buf, int len)
{
	struct i2c_msg msgs;

	msgs.addr = addr;
	msgs.flags = 0x00;  // 0x00: write 0x01:read 
	msgs.len = len;
	msgs.buf = buf;
	#if(PLATFORM == ROCKCHIP)
	msgs.scl_rate = TS_I2C_SPEED;
	#endif

	i2c_transfer(client->adapter, &msgs, 1);
}
*/
static int vtl_ts_config(struct ts_info *ts)
{
	struct device *dev;
	struct ts_config_info *config_info;	
	int err;

	DEBUG();

	dev = &ts->driver->client->dev;
	
	
	/* ts config */
	ts->config_info.touch_point_number = TOUCH_POINT_NUM;
	#if(PLATFORM == ALLWINER)
	{
		extern struct ctp_config_info config_info;
		ts->config_info.screen_max_x	   = config_info.screen_max_x;
        	ts->config_info.screen_max_y	   = config_info.screen_max_y;
		ts->config_info.irq_gpio_number    = config_info.irq_gpio_number;
        	ts->config_info.rst_gpio_number    = config_info.wakeup_gpio_number;
		
	}
	#else 
	if(dev->platform_data !=NULL){
		config_info = dev->platform_data;
		ts->config_info.screen_max_x	   = config_info->screen_max_x;
    ts->config_info.screen_max_y	   = config_info->screen_max_y;
    ts->config_info.revert_x_flag = config_info->revert_x_flag;
    ts->config_info.revert_y_flag = config_info->revert_y_flag;
    ts->config_info.exchange_x_y_flag = config_info->exchange_x_y_flag;
		ts->config_info.irq_gpio_number    = config_info->irq_gpio_number;
    ts->config_info.rst_gpio_number    = config_info->rst_gpio_number;
			
	}
	/*else
	{	
		ts->config_info.screen_max_x	   = SCREEN_MAX_X;
        	ts->config_info.screen_max_y	   = SCREEN_MAX_y;
		ts->config_info.irq_gpio_number    = TS_IRQ_GPIO_NUM;
        	ts->config_info.rst_gpio_number    = TS_RST_GPIO_NUM;		
	}*/
	#endif

	/* IRQ config*/
	ts->config_info.irq_number = gpio_to_irq(ts->config_info.irq_gpio_number);
	printk("irq = %d,rst = %d,int =%d__\n",ts->config_info.irq_number,ts->config_info.rst_gpio_number,ts->config_info.irq_gpio_number);
	// init reset pin
	err = gpio_request(ts->config_info.rst_gpio_number, "ts_rst");
	if ( err ) {

		return -EIO;
	}
	gpio_direction_output(ts->config_info.rst_gpio_number, 1);
	//gpio_set_value(ts->config_info.rst_gpio_number, 1);


	// init irq pin
	err = gpio_request(ts->config_info.irq_gpio_number, "ts_irq");
	if ( err ) {
		printk("___irq request faild___\n");
		gpio_free(ts->config_info.rst_gpio_number);
		return -EIO;
	}
	
	gpio_direction_input(ts->config_info.irq_gpio_number);

	/* IRQ config*/
	//ts->config_info.irq_number = gpio_to_irq(ts->config_info.irq_gpio_number);

	return 0;
}


void vtl_ts_free_gpio(void)
{
	struct ts_info *ts;
	ts =pg_ts;
	DEBUG();

	//free_irq(ts->config_info.irq_number,ts);	
	gpio_free(ts->config_info.irq_gpio_number);
	gpio_free(ts->config_info.irq_gpio_number);	
}

void vtl_ts_hw_reset(void)
{
	struct ts_info *ts;	
	ts =pg_ts;
	DEBUG();

	mdelay(500);
	gpio_set_value(ts->config_info.rst_gpio_number, 0);
	mdelay(50);
	gpio_set_value(ts->config_info.rst_gpio_number, 1);
	mdelay(500);
}



static irqreturn_t vtl_ts_irq(int irq, void *dev)
{
	struct ts_info *ts;	
	ts =pg_ts;
		
	DEBUG();

	//disable_irq_nosync(ts->config_info.irq_number);// Disable ts interrupt
	queue_work(ts->driver->workqueue, &ts->driver->event_work);

	return IRQ_HANDLED;
}

static union ts_xy_data* vtl_read_xy_data(struct ts_info *ts)
{
	struct i2c_msg msgs;
	int err;
		
	DEBUG();

	msgs.addr = ts->driver->client->addr;
	msgs.flags = 0x01;  // 0x00: write 0x01:read 
	msgs.len = sizeof(ts->xy_data.buf);
	msgs.buf = ts->xy_data.buf;
	#if(PLATFORM == ROCKCHIP)
	msgs.scl_rate = TS_I2C_SPEED;
	#endif
	err = i2c_transfer( ts->driver->client->adapter, &msgs, 1);
	if(err != 1){
		printk("___%s:i2c read err___\n",__func__);
		return NULL;
	}
	return &ts->xy_data;
}

static void vtl_report_xy_coord(struct input_dev *input_dev,union ts_xy_data *xy_data,unsigned char touch_point_number)
{
	struct ts_info *ts;	
	ts =pg_ts;
	
	int id;
	int sync;
	int x, y;
	unsigned int press;
	static unsigned int release;
		
	DEBUG();

	/* report points */
	sync = 0;  press = 0;
	for ( id = 0; id <touch_point_number; id++ ) {
		if ((xy_data->point[id].xhi != 0xFF) && (xy_data->point[id].yhi != 0xFF) &&
		     ( (xy_data->point[id].status == 1) || (xy_data->point[id].status == 2))) {


		x = (xy_data->point[id].xhi<<4)|(xy_data->point[id].xlo&0xF);
		y = (xy_data->point[id].yhi<<4)|(xy_data->point[id].ylo&0xF);

		#if 0
			{
				printk("ID:       %d\n", xy_data->point[id].id);
				printk("status:   %d\n", xy_data->point[id].status);
				printk("X:        %d\n", x);
				printk("Y:        %d\n", y);
			}
		#endif
				
		if (ts->config_info.exchange_x_y_flag) {
			int tmp;
			tmp = x;
			x = y;
			y = tmp;	
		}
		if (ts->config_info.revert_x_flag)
			x = ts->config_info.screen_max_x - x;
		
		if (ts->config_info.revert_y_flag)
			y = ts->config_info.screen_max_y - y;


		#if 1
			input_mt_slot(input_dev, xy_data->point[id].id - 1);
			input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, true);
			input_report_abs(input_dev, ABS_MT_POSITION_X, x);
			input_report_abs(input_dev, ABS_MT_POSITION_Y, y);
			input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, 30);
			input_report_abs(input_dev, ABS_MT_WIDTH_MAJOR, 128);
		#else
			input_report_abs(input_dev, ABS_MT_TRACKING_ID, xy_data->point[id].id - 1);
			input_report_abs(input_dev, ABS_MT_POSITION_X,  x);
			input_report_abs(input_dev, ABS_MT_POSITION_Y,  y);
			input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, 30);
			input_report_abs(input_dev, ABS_MT_WIDTH_MAJOR, 128);
			
			input_mt_sync(input_dev);
		#endif
	
			sync = 1;
			press |= 0x01 << (xy_data->point[id].id - 1);
		}
	}
	
	release &= (release ^ press);//release point flag
	for ( id = 0; id < touch_point_number; id++ ) {
		if ( release & (0x01<<id) ) {
		#if 1
			input_mt_slot(input_dev, id);
			input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, false);
		#else	
			input_report_abs(input_dev, ABS_MT_TRACKING_ID, id);
			input_report_abs(input_dev, ABS_MT_POSITION_X,  x);
			input_report_abs(input_dev, ABS_MT_POSITION_Y,  y);
			input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, 0);
			input_report_abs(input_dev, ABS_MT_WIDTH_MAJOR, 0);

			input_mt_sync(input_dev);
		#endif
			sync = 1;
		}
	}
	release = press;

	if ( sync ) {
		input_sync(input_dev);
	}

}

static void vtl_ts_workfunc(struct work_struct *work)
{
	
	union ts_xy_data *xy_data;
	struct input_dev *input_dev;
	unsigned char touch_point_number;

	DEBUG();

	input_dev = pg_ts->driver->input_dev;
	touch_point_number = pg_ts->config_info.touch_point_number;
	
	xy_data = vtl_read_xy_data(pg_ts);
	if(xy_data != NULL){
		vtl_report_xy_coord(input_dev,xy_data,touch_point_number);
	}
	else
	{
		printk("____xy_data error___\n");
	}
	
	// Enable ts interrupt
	//enable_irq(pg_ts->config_info.irq_number);
}



int vtl_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct ts_info *ts;	
	ts =pg_ts;

	DEBUG();

	disable_irq(ts->config_info.irq_number);
	cancel_work_sync(&ts->driver->event_work);
	//ts_sleep(client);
	ct36x_chip_go_sleep(client);

	return 0;
}

int vtl_ts_resume(struct i2c_client *client)
{
	struct ts_info *ts;	
	ts =pg_ts;

	DEBUG();

	/* Hardware reset */
	vtl_ts_hw_reset();
	enable_irq(ts->config_info.irq_number);

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void vtl_ts_early_suspend(struct early_suspend *handler)
{
	struct ts_info *ts;	
	ts =pg_ts;

	DEBUG();

	vtl_ts_suspend(ts->driver->client, PMSG_SUSPEND);
}

static void vtl_ts_early_resume(struct early_suspend *handler)
{
	struct ts_info *ts;	
	ts =pg_ts;

	DEBUG();

	vtl_ts_resume(ts->driver->client);
}
#endif

int __devexit vtl_ts_remove(struct i2c_client *client)
{
	struct ts_info *ts;	
	ts =pg_ts;

	DEBUG();

	/* Driver clean up */

	free_irq(ts->config_info.irq_number, ts);
	vtl_ts_free_gpio();

	#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ts->driver->early_suspend);
	#endif
	
	cancel_work_sync(&ts->driver->event_work);
	destroy_workqueue(ts->driver->workqueue);

	input_unregister_device(ts->driver->input_dev);
	input_free_device(ts->driver->input_dev);

	if ( ts->driver->proc_entry != NULL ){
		remove_proc_entry(DRIVER_NAME, NULL);
	}

	//i2c_set_clientdata(client, NULL);

	return 0;
}

static int init_input_dev(struct ts_info *ts)
{
	struct input_dev *input_dev;
	struct device *dev;	
	int err;

	DEBUG();

	
	dev = &ts->driver->client->dev;
	
	/* allocate input device */
	ts->driver->input_dev = input_allocate_device();
	if ( ts->driver->input_dev == NULL ) {
		dev_err(dev, "Unable to allocate input device for device %s.\n", DRIVER_NAME);
		return -1;
	}
		
	input_dev = ts->driver->input_dev;

	input_dev->name = "VTL for wld";
	input_dev->phys = "I2C";
    	input_dev->id.bustype = BUS_I2C;
    	input_dev->id.vendor  = 0xaaaa;
    	input_dev->id.product = 0x5555;
    	input_dev->id.version = 0x0001; 
	
	/* config input device */
	__set_bit(EV_SYN, input_dev->evbit);
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(EV_ABS, input_dev->evbit);

	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit);
	
	input_mt_init_slots(input_dev, TOUCH_POINT_NUM);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, ts->config_info.screen_max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, ts->config_info.screen_max_y, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);

	/* register input device */
	err = input_register_device(input_dev);
	if ( err ) {
		dev_err(dev, "Unable to register input device for device %s.\n", DRIVER_NAME);
		return -1;
	}
	
	return 0;
}

int ct36x_test_tp(struct i2c_client *client)
{
	struct i2c_msg msgs;
	char buf;
	msgs.addr = 0x7F;
	msgs.flags = 0x01;
	msgs.len = 1;
	msgs.buf = &buf;
	msgs.scl_rate = TS_I2C_SPEED;

	if (i2c_transfer(client->adapter, &msgs, 1) != 1) {
		return -1;
	}
	
	return 0;
}

int vtl_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = -1;
	struct ts_info *ts;
	struct device *dev;

	struct ts_config_info *pdata = client->dev.platform_data;

	DEBUG();

	if (pdata->tp_enter_init && pdata->tp_enter_init() < 0)
		return -ENODEV;
	
	ts = pg_ts;
	ts->driver->client = client;
	dev = &ts->driver->client->dev;

	
		
	if (ct36x_test_tp(client) < 0) {
			printk("vtl tp not found\n");
			goto ERR_TS_CONFIG;
	}
		
	/* Request platform resources (gpio/interrupt pins) */
	err = vtl_ts_config(ts);
	if(err){

		dev_err(dev, "VTL touch screen config Failed.\n");
		goto ERR_TS_CONFIG;
	}

	/* Check I2C Functionality */
	err = i2c_check_functionality(client->adapter, I2C_FUNC_I2C);
	if ( !err ) {
		dev_err(dev, "Check I2C Functionality Failed.\n");
		return ENODEV;
	}
	
	//ct36x_update(client);
	vtl_ts_hw_reset();

	/*init input dev*/
	err = init_input_dev(ts);
	if(err){

		dev_err(dev, "init input dev failed.\n");
		goto ERR_INIT_INPUT;
	}
	
	/* Create Proc Entry File */
	ts->driver->proc_entry = create_proc_entry(DRIVER_NAME, 0666/*S_IFREG | S_IRUGO | S_IWUSR*/, NULL);
	if ( ts->driver->proc_entry == NULL ) {
		dev_err(dev, "Failed creating proc dir entry file.\n");
		goto ERR_PROC_ENTRY;
	} else {
		ts->driver->proc_entry->proc_fops = NULL;//&vtl_ts_fops;
	}
	
	/* register early suspend */
#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->driver->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->driver->early_suspend.suspend = vtl_ts_early_suspend;
	ts->driver->early_suspend.resume = vtl_ts_early_resume;
	register_early_suspend(&ts->driver->early_suspend);
#endif
	
	/* Create work queue */
	INIT_WORK(&ts->driver->event_work, vtl_ts_workfunc);
	ts->driver->workqueue = create_singlethread_workqueue(DRIVER_NAME);
	
	/* Init irq */
	
	err = request_irq(ts->config_info.irq_number, vtl_ts_irq, IRQF_TRIGGER_FALLING, DRIVER_NAME, ts);
	if ( err ) {
		dev_err(dev, "Unable to request irq for device %s.\n", DRIVER_NAME);
		goto ERR_IRQ_REQ;
	}

	if (pdata->tp_exit_init)
		pdata->tp_exit_init(1);
		
	printk("___%s() end____ \n", __func__);
	
	return 0;

ERR_IRQ_REQ:
	cancel_work_sync(&ts->driver->event_work);
	destroy_workqueue(ts->driver->workqueue);
	#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ts->driver->early_suspend);
	#endif
	if ( ts->driver->proc_entry ){
		remove_proc_entry(DRIVER_NAME, NULL); 
	}

ERR_PROC_ENTRY:
	if(ts->driver->input_dev){
		input_free_device(ts->driver->input_dev);
		//input_unregister_device(ts->driver->input);
	}
ERR_INIT_INPUT:
	gpio_free(ts->config_info.rst_gpio_number);
	gpio_free(ts->config_info.irq_gpio_number);
ERR_TS_CONFIG:

	if (pdata->tp_exit_init)
		pdata->tp_exit_init(0);
	return err;
}

struct i2c_driver vtl_ts_driver  = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= DRIVER_NAME
	},
	.id_table	= vtl_ts_id,
	.probe      = vtl_ts_probe,
	.suspend	= vtl_ts_suspend,
	.resume	    = vtl_ts_resume,
	.remove 	= __devexit_p(vtl_ts_remove),
};

int __init vtl_ts_init(void)
{
	DEBUG();
	return i2c_add_driver(&vtl_ts_driver);
}

void __exit vtl_ts_exit(void)
{
	DEBUG();
	i2c_del_driver(&vtl_ts_driver);
}

module_init(vtl_ts_init);
module_exit(vtl_ts_exit);

MODULE_AUTHOR("VTL");
MODULE_DESCRIPTION("VTL TouchScreen driver");
MODULE_LICENSE("GPL");


