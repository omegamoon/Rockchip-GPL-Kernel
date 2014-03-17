
#include <linux/i2c.h>
#include <linux/delay.h>
#include "vtl_ts.h"


#define CT36X_TS_CHIP_DEBUG 0

#define CT36X_CHIP_FLASH_SECTOR_NUM	256
#define CT36X_CHIP_FLASH_SECTOR_SIZE	128
#define CT36X_CHIP_FLASH_SOURCE_SIZE	8

//static 
unsigned char binary_data[] = {
#include "dzh242hmj01_97_CT363_V01_F58A_130724.dat"
//#include "N601_StandardCode.dat"
//#include "CT363_FiveX_20x30_V851302121.dat"
//	#include "PB101JG722_79_CT363_V01_E692_130428.dat"
//#include "CT363M.dat"
};

/*
* Private functions
*/

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

static void ct36x_chip_solfware_rest(struct i2c_client *client)
{
	unsigned char buf[2];	
	
	if ( CT36X_TS_CHIP_DEBUG )
	printk(">>>>> %s() called <<<<< \n", __FUNCTION__);

	buf[0] = 0x00;
	buf[1] = 0x5a;
	ct36x_ts_reg_write(client, 0x7F, buf, 2);
	mdelay(10);
}
static void ct36x_chip_set_idle(struct i2c_client *client, unsigned char *buf)
{
	if ( CT36X_TS_CHIP_DEBUG )
	printk(">>>>> %s() called <<<<< \n", __FUNCTION__);

	buf[0] = 0x00;
	buf[1] = 0xA5;
	ct36x_ts_reg_write(client, 0x7F, buf, 2);
	mdelay(10);
}

static void ct36x_chip_rst_offset(struct i2c_client *client, unsigned char *buf)
{
	if ( CT36X_TS_CHIP_DEBUG )
	printk(">>>>> %s() called <<<<< \n", __FUNCTION__);

	buf[0] = 0x00;
	ct36x_ts_reg_write(client, 0x7F, buf, 1);
	mdelay(20);
}

static int ct36x_chip_get_busstatus(struct i2c_client *client, unsigned char *buf)
{
	if ( CT36X_TS_CHIP_DEBUG )
	printk(">>>>> %s() called <<<<< \n", __FUNCTION__);

	ct36x_ts_reg_read(client, 0x7F, buf, 1);
	mdelay(10);

	return buf[0];
}

static int ct36x_chip_erase_flash(struct i2c_client *client, unsigned char *buf)
{
	int ret = -1;

	if ( CT36X_TS_CHIP_DEBUG )
	printk(">>>>> %s() called <<<<< \n", __FUNCTION__);

	// Erase 32k flash
	buf[0] = 0x00;
	buf[1] = 0x33;
	buf[2] = 0x00;
	ct36x_ts_reg_write(client, 0x7F, buf, 3);
	mdelay(10);

	// Reset I2C offset address
	ct36x_chip_rst_offset(client, buf);

	// Read I2C bus status
	ret = ct36x_chip_get_busstatus(client, buf);
	if ( ret != 0xAA ) {
		return -1;
	}

	return 0;
}

/*
** Prepare code segment
*/
static int ct36x_chip_set_code(unsigned int flash_addr, unsigned char *buf)
{
	unsigned char cod_chksum;

	//if ( CT36X_TS_CHIP_DEBUG )
	//printk(">>>>> %s() called <<<<< \n", __FUNCTION__);

	// Flash address
	// data length
	buf[2] = (char)(flash_addr >> 8);
	buf[3] = (char)(flash_addr & 0xFF);
	buf[4] = 0x08;

	// Fill firmware source data
	//if ( (sec == 1 && cod == 4) || (sec == 1 && cod == 5) ) {
	//if ( flash_addr == (CT36X_CHIP_FLASH_SECTOR_SIZE + 32) || 
	//flash_addr == (CT36X_CHIP_FLASH_SECTOR_SIZE + 40) ) {
	if ( flash_addr == (160) || flash_addr == (168) ) {
		buf[6] = ~binary_data[flash_addr + 0];
		buf[7] = ~binary_data[flash_addr + 1];
		buf[8] = ~binary_data[flash_addr + 2];
		buf[9] = ~binary_data[flash_addr + 3];
		buf[10] = ~binary_data[flash_addr + 4];
		buf[11] = ~binary_data[flash_addr + 5];
		buf[12] = ~binary_data[flash_addr + 6];
		buf[13] = ~binary_data[flash_addr + 7];
	} else {
		buf[6] = binary_data[flash_addr + 0];
		buf[7] = binary_data[flash_addr + 1];
		buf[8] = binary_data[flash_addr + 2];
		buf[9] = binary_data[flash_addr + 3];
		buf[10] = binary_data[flash_addr + 4];
		buf[11] = binary_data[flash_addr + 5];
		buf[12] = binary_data[flash_addr + 6];
		buf[13] = binary_data[flash_addr + 7];
	}
			
	/* Calculate a checksum by Host controller. 
	** Checksum =  ~(FLASH_ADRH+FLASH_ADRL+LENGTH+
	** Binary_Data1+Binary_Data2+Binary_Data3+Binary_Data4+
	** Binary_Data5+Binary_Data6+Binary_Data7+Binary_Data8) + 1
	*/
	cod_chksum = ~(buf[2]+buf[3]+buf[4]+
				buf[6]+buf[7]+buf[8]+buf[9]+
				buf[10]+buf[11]+buf[12]+buf[13]) + 1;
	buf[5] = cod_chksum;

	return cod_chksum;
}

static int ct36x_chip_write_firmware(struct i2c_client *client, unsigned char *buf)
{
	int ret = -1;
	int sec, cod;
	unsigned char cod_chksum;
	unsigned int fin_chksum;
	unsigned int flash_addr;

	if ( CT36X_TS_CHIP_DEBUG )
	printk(">>>>> %s() called <<<<< \n", __FUNCTION__);

	// Code checksum, final checksum
	cod_chksum = 0x00; fin_chksum = 0x00;

	// Flash write command
	buf[0] = 0x00;
	buf[1] = 0x55;

	// 256 sectors, 128 bytes per sectors
	for ( sec = 0; sec < CT36X_CHIP_FLASH_SECTOR_NUM; sec++ ) {
		flash_addr = sec * CT36X_CHIP_FLASH_SECTOR_SIZE;
		// 16 segments, 8 bytes per segment
		for ( cod = 0; cod < (CT36X_CHIP_FLASH_SECTOR_SIZE/CT36X_CHIP_FLASH_SOURCE_SIZE); cod++ ) {
			// Fill binary data
			cod_chksum = ct36x_chip_set_code(flash_addr, buf);
			fin_chksum += cod_chksum;

			// Write firmware source data
			ct36x_ts_reg_write(client, 0x7F, buf, 14);

			// 
			mdelay(1);
			//printk("cod=%d\n",cod);
			// Increase flash address 8bytes for each write command
			flash_addr += CT36X_CHIP_FLASH_SOURCE_SIZE;
		}
		//
		//printk("sec=%d\n",sec);
		mdelay(20);
	}

	return 0;
}

static int ct36x_chip_read_infoblk(struct i2c_client *client, unsigned char *buf)
{
	if ( CT36X_TS_CHIP_DEBUG )
	printk(">>>>> %s() called <<<<< \n", __FUNCTION__);

	return 0;
}

static int ct36x_chip_erase_infoblk(struct i2c_client *client, unsigned char *buf)
{
	int ret = -1;

	if ( CT36X_TS_CHIP_DEBUG )
	printk(">>>>> %s() called <<<<< \n", __FUNCTION__);

	// info block erase command
	buf[0] = 0x00;
	buf[1] = 0x60;
	buf[2] = 0x00;
	ct36x_ts_reg_write(client, 0x7F, buf, 3);
	mdelay(10);

	// Reset I2C offset address
	ct36x_chip_rst_offset(client, buf);

	// Read I2C bus status
	ret = ct36x_chip_get_busstatus(client, buf);
	if ( ret != 0xAA ) {
		printk("trim data erase error!!! \n");
		return -1;
	}

	return 0;
}

static int ct36x_chip_write_infoblk(struct i2c_client *client, unsigned char *buf)
{
	//int ret = -1;
	int sec, cod;
	unsigned int flash_addr;

	if ( CT36X_TS_CHIP_DEBUG )
	printk(">>>>> %s() called <<<<< \n", __FUNCTION__);

	flash_addr = 0x00;

	// write info block 0
	buf[0] = 0x00;
	buf[1] = 0x61;

	for ( cod = 0; cod < 16; cod++ ) {
	// Flash address
	// data length
	buf[2] = (char)(flash_addr >> 8);
	buf[3] = (char)(flash_addr & 0xFF);
	buf[4] = 0x08;
	if ( flash_addr == 0x0000 )
	buf[6] = 0x17;
	else
	buf[6] = 0x00;
	
	buf[7] = 0x00;
	buf[8] = 0x00;
	buf[9] = 0x00;
	buf[10] = 0x00;
	buf[11] = 0x00;
	buf[12] = 0x00;
	buf[13] = 0x00;

	buf[5] = (~(buf[2]+buf[3]+buf[4]+buf[6]+buf[7]+buf[8]+buf[9]+buf[10]+buf[11]+buf[12]+buf[13]))+1;
		
	ct36x_ts_reg_write(client, 0x7F, buf, 14);
	mdelay(10);

	flash_addr += 8;
	}

	return 0;
}

int ct36x_chip_get_binchksum(unsigned char *buf)
{
	int sec, cod;
	unsigned char cod_chksum;
	unsigned int fin_chksum = 0;
	unsigned int flash_addr;

	if ( CT36X_TS_CHIP_DEBUG )
	printk(">>>>> %s() called <<<<< \n", __FUNCTION__);

	// 256 sectors, 128 bytes per sectors
	for ( sec = 0; sec < CT36X_CHIP_FLASH_SECTOR_NUM; sec++ ) {
		flash_addr = sec * CT36X_CHIP_FLASH_SECTOR_SIZE;
		// 16 segments, 8 bytes per segment
		for ( cod = 0; cod < (CT36X_CHIP_FLASH_SECTOR_SIZE/CT36X_CHIP_FLASH_SOURCE_SIZE); cod++ ) {
			// Fill binary data
			cod_chksum = ct36x_chip_set_code(flash_addr, buf);
			fin_chksum += cod_chksum;

			// Increase flash address 8bytes for each write command
			flash_addr += CT36X_CHIP_FLASH_SOURCE_SIZE;
		}
	}

	return (unsigned short)fin_chksum;
}

int ct36x_chip_get_fwchksum(struct i2c_client *client, unsigned char *buf)
{
	int fwchksum = 0x00;

	if ( CT36X_TS_CHIP_DEBUG )
	printk(">>>>> %s() called <<<<< \n", __FUNCTION__);

	buf[0] = 0xFF;
#ifndef CT363M
	buf[1] = 0x8F;
#else
	buf[1] = 0x6F;
#endif
	buf[2] = 0xFF;
	ct36x_ts_reg_write(client, client->addr, buf, 3);
	mdelay(20);

	buf[0] = 0x00;
#ifndef CT363M
	buf[1] = 0xE1;
#else
	buf[1] = 0xEa;
#endif
	//buf[1] = 0xE1;
	//buf[1] = 0xEa;
	ct36x_ts_reg_write(client, client->addr, buf, 2);
	mdelay(500);

	buf[0] = 0xFF;
#ifndef CT363M
	buf[1] = 0x8E;
#else
	buf[1] = 0x6E;
#endif
	//buf[1] = 0x8E;
	//buf[1] = 0x6E;
	buf[2] = 0x0E;
	ct36x_ts_reg_write(client, client->addr, buf, 3);
	mdelay(20);

	ct36x_chip_rst_offset(client, buf);//20130513

	ct36x_ts_reg_read(client, client->addr, buf, 3);
	mdelay(20);

	fwchksum = ((buf[0]<<8) | buf[1]);

	return fwchksum;
}

int ct36x_chip_get_ver(struct i2c_client *client, unsigned char *buf)
{
	if ( CT36X_TS_CHIP_DEBUG )
	printk(">>>>> %s() called <<<<< \n", __FUNCTION__);

	// Read version command
	buf[0] = 0xFF;
	buf[1] = 0x3F;
	buf[2] = 0xFF;

	ct36x_ts_reg_write(client, client->addr, buf, 3);
	mdelay(10);

	buf[0] = 0x00;
	ct36x_ts_reg_write(client, client->addr, buf, 1);
	mdelay(10);

	// do read version
	ct36x_ts_reg_read(client, client->addr, buf, 1);
	mdelay(10);

	return buf[0];
}


void ct36x_chip_go_sleep(struct i2c_client *client)
{
	
	unsigned char buf[3];
	if ( CT36X_TS_CHIP_DEBUG )
	printk(">>>>> %s() called <<<<< \n", __FUNCTION__);

	buf[0] = 0xFF;
#ifndef CT363M
	buf[1] = 0x8F;
#else
	buf[1] = 0x6F;
#endif
	//buf[1] = 0x8F;
	//buf[1] = 0x6F;
	buf[2] = 0xFF;
	ct36x_ts_reg_write(client, client->addr, buf, 3);
	mdelay(3);

	buf[0] = 0x00;
	buf[1] = 0xAF;
	ct36x_ts_reg_write(client, client->addr, buf, 2);
	mdelay(3);

	//mdelay(50);
}

int ct36x_chip_go_bootloader(struct i2c_client *client, unsigned char *buf)
{
	int ret = -1;

	if ( CT36X_TS_CHIP_DEBUG )
	printk(">>>>> %s() called <<<<< \n", __FUNCTION__);
	
	// Init bootloader
	ct36x_chip_set_idle(client, buf);

	// Reset I2C offset address
	ct36x_chip_rst_offset(client, buf);

	// Get I2C bus status
	ret = ct36x_chip_get_busstatus(client, buf);
	if ( ret != 0xAA ) {
		printk("I2C bus status: 0x%x.\n", ret);
		return -1;
	}

	// Erase flash
	ret = ct36x_chip_erase_flash(client, buf);
	if ( ret ) {
		printk("Erase flash failed.\n");
		return -1;
	}

	// Write source data
	ct36x_chip_write_firmware(client, buf);
	
	return 0;
}



unsigned char ct365_get_vendor(struct i2c_client *client)
{
	unsigned char buf[3] ={0};
	int ret;

	printk(">>>>> %s() called <<<<< \n", __FUNCTION__);

	//ct36x_platform_hw_reset(&ct36x_ts);
	ct36x_chip_set_idle(client, buf);

	ct36x_chip_rst_offset(client, buf);

	// Read I2C bus status
	ret = ct36x_chip_get_busstatus(client, buf);
	if ( ret != 0xAA ) {
		printk("___i2c status = %d___\n",ret);
		return 0;
	}

	// read chip id command
	buf[0] = 0xFF;
	buf[1] = 0xF0;
	buf[2] = 0x00;
	
	ct36x_ts_reg_write(client, 0x01, buf, 3);
	mdelay(10);

	
	buf[0] = 0x00;
	ct36x_ts_reg_write(client, 0x01, buf, 1);
	mdelay(10);

	// do read chip id
	ct36x_ts_reg_read(client, 0x01, buf, 1);
	
	//printk("___chip ID = %d___\n",buf[0]);
	//ct36x_platform_hw_reset(&ct36x_ts);
	ct36x_chip_solfware_rest(client);

	return buf[0];
}

extern void vtl_ts_hw_reset(void);

unsigned char ct36x_update(struct i2c_client *client)
{
	unsigned char buf[20];
	unsigned char updcnt;
	int binchksum;
	int fwchksum;
	unsigned char ret;

	
	// Get binary Checksum
	binchksum = ct36x_chip_get_binchksum(buf);
	if ( CT36X_TS_CHIP_DEBUG )
	printk("Bin checksum: 0x%x\n", binchksum);

	// Get firmware Checksum
	fwchksum = ct36x_chip_get_fwchksum(client,buf);
	if ( CT36X_TS_CHIP_DEBUG )
	printk("Fw checksum: 0x%x\n", fwchksum);
       	

	updcnt =2 ;
	while ( binchksum != fwchksum && updcnt--) {
		/* Update Firmware */
		ct36x_chip_go_bootloader(client,buf);

		/* Hardware reset */
		
		vtl_ts_hw_reset();

		// Get firmware Checksum
		fwchksum = ct36x_chip_get_fwchksum(client,buf);
		if ( CT36X_TS_CHIP_DEBUG )
		printk("Fw checksum: 0x%x\n", fwchksum);
	printk("______Fw update %s,binchksum: 0x%x, fwchksum: 0x%x_____\n", binchksum != fwchksum ? "Failed" : "Success", binchksum, fwchksum);
	}

	

	return (binchksum != fwchksum ?1:0);

}



void ct36x_chip_set_adapter_on(struct i2c_client *client)
{
	unsigned char buf[3] = {0};	
	//if ( CT36X_TS_CHIP_DEBUG )
	printk(">>>>> %s() called <<<<< \n", __FUNCTION__);

	buf[0] = 0xFF;
#ifndef CT363M
	buf[1] = 0x8F;
#else
	buf[1] = 0x6F;
#endif
	//buf[1] = 0x8F;
	//buf[1] = 0x6F;
	buf[2] = 0xFF;
	ct36x_ts_reg_write(client, client->addr, buf, 3);
	mdelay(3);

	buf[0] = 0x00;
	buf[1] = 0xE3;
	ct36x_ts_reg_write(client, client->addr, buf, 2);
	mdelay(3);
}

void ct36x_chip_set_adapter_off(struct i2c_client *client)
{
	
	unsigned char buf[3] = {0};
	//if ( CT36X_TS_CHIP_DEBUG )
	printk(">>>>> %s() called <<<<< \n", __FUNCTION__);

	buf[0] = 0xFF;
#ifndef CT363M
	buf[1] = 0x8F;
#else
	buf[1] = 0x6F;
#endif
	//buf[1] = 0x8F;
	//buf[1] = 0x6F;
	buf[2] = 0xFF;
	ct36x_ts_reg_write(client, client->addr, buf, 3);
	mdelay(3);

	buf[0] = 0x00;
	buf[1] = 0xE2;
	ct36x_ts_reg_write(client, client->addr, buf, 2);
	mdelay(3);
}

