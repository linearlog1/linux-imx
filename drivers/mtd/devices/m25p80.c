/*
 * MTD SPI driver for ST M25Pxx (and similar) serial flash chips
 *
 * Author: Mike Lavender, mike@steroidmicros.com
 *
 * Copyright (c) 2005, Intec Automation Inc.
 *
 * Some parts are based on lart.c by Abraham Van Der Merwe
 *
 * Cleaned up and generalized based on mtd_dataflash.c
 *
 * This code is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/err.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/device.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>

#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/mtd/spi-nor.h>

#define	MAX_CMD_SIZE		6

#define M25P80_SYSFS_BOARD_CONFIG

struct m25p {
	struct spi_device	*spi;
	struct spi_nor		spi_nor;
	struct mtd_info		mtd;
	u8			command[MAX_CMD_SIZE];
};

#ifdef M25P80_SYSFS_BOARD_CONFIG
#define M25P80_BOARD_CONFIG_SIZE       0x10000
#define M25P80_CONFIG_STRING_LENGTH    128
#define M25P80_CONFIG_PAGE_SIZE        0x100
#define M25P80_CONFIG_MAC_SIZE         6
#define M25P80_CONFIG_HOTKEY_SIZE      10


/* ================================================================ *
 * Causion:                                                         *
 * All the new config should be added at the end of the structure   *
 * Otherwise, the ambigous board config would be made               *
 * Config for any MAC address should use *_mac as its name          *
 * ================================================================ */
 
const char *m25p80_config_table[] = {
	"magic_number",
	"serial_number",
	"board_number",
	"board_id",
	"eth_mac",
	NULL
};

struct m25p80_board_config {
	int   cfg_num;
	int   cfg_size;
	char  *cfg_buf;
	char  itmbuf[M25P80_CONFIG_STRING_LENGTH];
};

struct m25p80_board_config boardcfg;
 
#endif


static int m25p80_read_reg(struct spi_nor *nor, u8 code, u8 *val, int len)
{
	struct m25p *flash = nor->priv;
	struct spi_device *spi = flash->spi;
	int ret;

	ret = spi_write_then_read(spi, &code, 1, val, len);
	if (ret < 0)
		dev_err(&spi->dev, "error %d reading %x\n", ret, code);

	return ret;
}

static void m25p_addr2cmd(struct spi_nor *nor, unsigned int addr, u8 *cmd)
{
	/* opcode is in cmd[0] */
	cmd[1] = addr >> (nor->addr_width * 8 -  8);
	cmd[2] = addr >> (nor->addr_width * 8 - 16);
	cmd[3] = addr >> (nor->addr_width * 8 - 24);
	cmd[4] = addr >> (nor->addr_width * 8 - 32);
}

static int m25p_cmdsz(struct spi_nor *nor)
{
	return 1 + nor->addr_width;
}

static int m25p80_write_reg(struct spi_nor *nor, u8 opcode, u8 *buf, int len)
{
	struct m25p *flash = nor->priv;
	struct spi_device *spi = flash->spi;

	flash->command[0] = opcode;
	if (buf)
		memcpy(&flash->command[1], buf, len);

	return spi_write(spi, flash->command, len + 1);
}

static ssize_t m25p80_write(struct spi_nor *nor, loff_t to, size_t len,
			    const u_char *buf)
{
	struct m25p *flash = nor->priv;
	struct spi_device *spi = flash->spi;
	struct spi_transfer t[2] = {};
	struct spi_message m;
	int cmd_sz = m25p_cmdsz(nor);
	ssize_t ret;

	spi_message_init(&m);

	if (nor->program_opcode == SPINOR_OP_AAI_WP && nor->sst_write_second)
		cmd_sz = 1;

	flash->command[0] = nor->program_opcode;
	m25p_addr2cmd(nor, to, flash->command);

	t[0].tx_buf = flash->command;
	t[0].len = cmd_sz;
	spi_message_add_tail(&t[0], &m);

	t[1].tx_buf = buf;
	t[1].len = len;
	spi_message_add_tail(&t[1], &m);

	ret = spi_sync(spi, &m);
	if (ret)
		return ret;

	ret = m.actual_length - cmd_sz;
	if (ret < 0)
		return -EIO;
	return ret;
}

static inline unsigned int m25p80_rx_nbits(struct spi_nor *nor)
{
	switch (nor->flash_read) {
	case SPI_NOR_DUAL:
		return 2;
	case SPI_NOR_QUAD:
		return 4;
	default:
		return 0;
	}
}

/*
 * Read an address range from the nor chip.  The address range
 * may be any size provided it is within the physical boundaries.
 */
static ssize_t m25p80_read(struct spi_nor *nor, loff_t from, size_t len,
			   u_char *buf)
{
	struct m25p *flash = nor->priv;
	struct spi_device *spi = flash->spi;
	struct spi_transfer t[2];
	struct spi_message m;
	unsigned int dummy = nor->read_dummy;
	ssize_t ret;

	/* convert the dummy cycles to the number of bytes */
	dummy /= 8;

	if (spi_flash_read_supported(spi)) {
		struct spi_flash_read_message msg;

		memset(&msg, 0, sizeof(msg));

		msg.buf = buf;
		msg.from = from;
		msg.len = len;
		msg.read_opcode = nor->read_opcode;
		msg.addr_width = nor->addr_width;
		msg.dummy_bytes = dummy;
		/* TODO: Support other combinations */
		msg.opcode_nbits = SPI_NBITS_SINGLE;
		msg.addr_nbits = SPI_NBITS_SINGLE;
		msg.data_nbits = m25p80_rx_nbits(nor);

		ret = spi_flash_read(spi, &msg);
		if (ret < 0)
			return ret;
		return msg.retlen;
	}

	spi_message_init(&m);
	memset(t, 0, (sizeof t));

	flash->command[0] = nor->read_opcode;
	m25p_addr2cmd(nor, from, flash->command);

	t[0].tx_buf = flash->command;
	t[0].len = m25p_cmdsz(nor) + dummy;
	spi_message_add_tail(&t[0], &m);

	t[1].rx_buf = buf;
	t[1].rx_nbits = m25p80_rx_nbits(nor);
	t[1].len = min(len, spi_max_transfer_size(spi));
	spi_message_add_tail(&t[1], &m);

	ret = spi_sync(spi, &m);
	if (ret)
		return ret;

	ret = m.actual_length - m25p_cmdsz(nor) - dummy;
	if (ret < 0)
		return -EIO;
	return ret;
}

#ifdef M25P80_SYSFS_BOARD_CONFIG
static int m25p80_find_config(const char *name)
{
	int index = 0;

	while (m25p80_config_table[index] != NULL)
	{
		if (!strcmp(name, m25p80_config_table[index]))
			return index;
		index++;
	}

	return -EINVAL;
}

static void m25p80_print_config_table(void)
{
	struct mtd_info *mtd = get_mtd_device_nm("config");
	
	int i, retlen = 0;
	int num = boardcfg.cfg_num;
	int size = boardcfg.cfg_size;
	char *buffer = boardcfg.cfg_buf;
	char *pch = NULL;
	char *bufptr = NULL;
	int cfglen = M25P80_CONFIG_STRING_LENGTH;
	char dispbuf[M25P80_CONFIG_STRING_LENGTH] = {0};
	
	mtd->_read(mtd, 0x0, size, &retlen, buffer);

	pr_info("====================================================\n");

	pr_info("board config : num = %d, size = %d\n", num, size);
	
	for (i = 0; i < num; i++) {
		memset(dispbuf, 0x0, cfglen);
		bufptr = buffer + (i * cfglen);
		
		pch = strstr(m25p80_config_table[i], "_mac");
		if(pch)
			sprintf(dispbuf, "%02x:%02x:%02x:%02x:%02x:%02x",
						bufptr[0], bufptr[1], bufptr[2], bufptr[3], bufptr[4], bufptr[5]);
		else
			strcpy(dispbuf, bufptr);

		pr_info("boardcfg[%d] = %-16s, value = %s\n",
						i, m25p80_config_table[i], dispbuf);
	}

	pr_info("====================================================\n");
}

static int m25p80_config_control(const char *name, char *srcbuf, bool write)
{
	int index = 0, retlen = 0, offset = 0;
	int cfglen = M25P80_CONFIG_STRING_LENGTH;
	int srclen = 0;

	struct erase_info erase_part;

	int size = boardcfg.cfg_size;
	char *dstbuf = boardcfg.cfg_buf;
	char *bufptr = NULL;
	
	struct mtd_info *mtd = get_mtd_device_nm("config");

	if(!srcbuf)
	{
		pr_err("[%s] invalid read/write buffer\n", __func__);
		return -EINVAL;
	}

	index = m25p80_find_config(name);
	if (index < 0)
	{
		pr_err("[%s] invalid board config : %s\n", __func__, name);
		return -EINVAL;
	}

	memset(dstbuf, 0x0, sizeof(dstbuf));

	offset = (index * cfglen);
	bufptr = dstbuf + offset;

	mtd->_read(mtd, 0x0, size, &retlen, dstbuf);
	
	if(write)
	{
		if(strstr(name, "_mac") != NULL)
			srclen = M25P80_CONFIG_MAC_SIZE;
		else
		{
			srclen = strlen(srcbuf);
			if(srcbuf[srclen - 1] == 0xA)
				srclen--;
		}

		memset(bufptr, 0x0, cfglen);
		
		// Use memcpy instead of strcpy
		// Prevent "00" MAC addr cause strlen shortage
		memcpy(bufptr, srcbuf, srclen);

		erase_part.addr = 0;
		erase_part.len = mtd->size;
		erase_part.callback = NULL;
		erase_part.mtd = mtd;
		mtd->_erase(mtd, &erase_part);
		mtd->_write(mtd, 0x0, size, &retlen, dstbuf);
		m25p80_print_config_table();
	}
	else
	{
		memcpy(srcbuf, bufptr, cfglen);
	}

	return 0;	
}

//=============================================================//
// boardcfg read/write function
//=============================================================//
int m25p80_get_config(const char* name, char *buf)
{
	struct mtd_info *mtd = get_mtd_device_nm("config");
	
	int retlen = 0, index = 0, cfglen = 0;
	int size = boardcfg.cfg_size;
	char *buffer = boardcfg.cfg_buf;
	char *bufptr = NULL;
	char dstbuf[M25P80_CONFIG_STRING_LENGTH] = {0};

	memset(dstbuf, 0x0, sizeof(dstbuf));
	
	mtd->_read(mtd, 0x0, size, &retlen, buffer);

	index = m25p80_find_config(name);

	bufptr = buffer + (M25P80_CONFIG_STRING_LENGTH * index);

	if(strstr(name, "_mac") != NULL)
	{
		cfglen = (M25P80_CONFIG_MAC_SIZE * 3) - 1; //MAC addr length

		sprintf(dstbuf, "%02x:%02x:%02x:%02x:%02x:%02x",
					bufptr[0], bufptr[1], bufptr[2], bufptr[3], bufptr[4], bufptr[5]);
	}
	else if(strstr(name, "hotkey") != NULL)
	{
		cfglen = M25P80_CONFIG_HOTKEY_SIZE;

		memcpy(dstbuf, bufptr, cfglen);
	}
	else
	{
		cfglen = strlen(bufptr);
		
		strcpy(dstbuf, bufptr);		
	}

	memcpy(buf, dstbuf, cfglen);

	return 0;	
}
EXPORT_SYMBOL(m25p80_get_config);

int m25p80_set_config(const char* name, char *buf)
{
	int ret;
	char *pch = NULL;
	char srcbuf[M25P80_CONFIG_STRING_LENGTH] = {0};
	
	memset(srcbuf, 0x0, sizeof(srcbuf));
	
	pch = strstr(name, "_mac");
	if(strstr(name, "_mac") != NULL)
	{
		sscanf(buf, "%02x:%02x:%02x:%02x:%02x:%02x",
			(unsigned int *)&srcbuf[0], (unsigned int *)&srcbuf[1], (unsigned int *)&srcbuf[2],
			(unsigned int *)&srcbuf[3], (unsigned int *)&srcbuf[4], (unsigned int *)&srcbuf[5]);
	}
	else if(strstr(name, "hotkey") != NULL)
		memcpy(srcbuf, buf, M25P80_CONFIG_HOTKEY_SIZE);
	else
		memcpy(srcbuf, buf, strlen(buf));

	ret = m25p80_config_control(name, srcbuf, 1);
	if (ret < 0)
		pr_err("[%s] invalid board config item\n", __func__);
	
	return 0;	
}
EXPORT_SYMBOL(m25p80_set_config);
//=============================================================//

static int m25p80_board_config_init(struct device *dev)
{
	struct mtd_info *mtd = get_mtd_device_nm("config");
	struct erase_info erase_part;
	const char magic_str[] = "advantech";

	int num = 0, size = 0, retlen = 0;
	int cfglen = M25P80_CONFIG_STRING_LENGTH;
	char *buffer = NULL;

	// Obtain config item number, minus one since the last string is NULL
	num = (sizeof(m25p80_config_table)/sizeof(char *)) - 1;
	if(num < 0)
	{
		dev_err(dev, "[%s] get boardcfg size error", __func__);
		return -EINVAL;
	}

	// Do the page aligment
	size = (((num * M25P80_CONFIG_STRING_LENGTH) + (M25P80_CONFIG_PAGE_SIZE - 1)) 
				/ M25P80_CONFIG_PAGE_SIZE) * M25P80_CONFIG_PAGE_SIZE; 

	// Allocate memory buffer
	buffer = (char *)kzalloc(size, GFP_KERNEL);
	if(!buffer)
	{
		dev_err(dev, "[%s] get boardcfg size error", __func__);
		return -ENOMEM;
	}

	// Read the first config item : magic_number
	mtd->_read(mtd, 0x0, cfglen, &retlen, buffer);

	// If magic number is mismatch, initialize the parition
	if(strncmp(buffer, magic_str, strlen(magic_str)))
	{
		dev_info(dev, "[UNKNOWN] init config partition\n");
		erase_part.addr = 0;
		erase_part.len = mtd->size;
		erase_part.callback = NULL;
		erase_part.mtd = mtd;
		mtd->_erase(mtd, &erase_part);

		memset(buffer, 0x0, cfglen); 
		strncpy(buffer, magic_str, strlen(magic_str));
		mtd->_write(mtd, 0x0, size, &retlen, buffer);
	}
	else
		dev_info(dev, "[MATCH] legal config partition\n");

	// Assign value to the global structure : boardcfg
	boardcfg.cfg_num = num;
	boardcfg.cfg_size = size;
	boardcfg.cfg_buf = buffer;
	
	// Print all the values of the config table
	m25p80_print_config_table();

	return 0;	
}

//Serial Number
static ssize_t m25p80_serial_number_show(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	int ret;

	memset(buf, 0x0, M25P80_CONFIG_STRING_LENGTH);
	
	ret = m25p80_config_control("serial_number", buf, 0);

	if (ret < 0)
		dev_err(dev, "[%s] invalid board config item\n", __func__);

	return strlen(buf);
}

static ssize_t m25p80_serial_number_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	
	ret = m25p80_config_control("serial_number", (char *)buf, 1);

	if (ret < 0)
		dev_err(dev, "[%s] invalid board config item\n", __func__);

	return count;
}

static DEVICE_ATTR(serial_number, 0644, m25p80_serial_number_show, m25p80_serial_number_store);

//Board Number
static ssize_t m25p80_board_number_show(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	int ret;

	memset(buf, 0x0, M25P80_CONFIG_STRING_LENGTH);
	
	ret = m25p80_config_control("board_number", buf, 0);

	if (ret < 0)
		dev_err(dev, "[%s] invalid board config item\n", __func__);

	return strlen(buf);
}

static ssize_t m25p80_board_number_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	
	ret = m25p80_config_control("board_number", (char *)buf, 1);

	if (ret < 0)
		dev_err(dev, "[%s] invalid board config item\n", __func__);

	return count;
}

static DEVICE_ATTR(board_number, 0644, m25p80_board_number_show, m25p80_board_number_store);

// Board ID
static ssize_t m25p80_board_id_show(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	int ret;

	memset(buf, 0x0, M25P80_CONFIG_STRING_LENGTH);
	
	ret = m25p80_config_control("board_id", buf, 0);

	if (ret < 0)
		dev_err(dev, "[%s] invalid board config item\n", __func__);

	return strlen(buf);
}

static ssize_t m25p80_board_id_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	
	ret = m25p80_config_control("board_id", (char *)buf, 1);

	if (ret < 0)
		dev_err(dev, "[%s] invalid board config item\n", __func__);

	return count;
}

static DEVICE_ATTR(board_id, 0644, m25p80_board_id_show, m25p80_board_id_store);

//Ethernet MAC address
static ssize_t m25p80_eth_mac_show(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	int ret, maclen;
	char macbuf[M25P80_CONFIG_STRING_LENGTH] = {0};
	

	memset(buf, 0x0, M25P80_CONFIG_STRING_LENGTH);
	
	ret = m25p80_config_control("eth_mac", macbuf, 0);

	if (ret < 0)
		dev_err(dev, "[%s] invalid board config item\n", __func__);

	sprintf(buf, "%02x:%02x:%02x:%02x:%02x:%02x",
			macbuf[0], macbuf[1], macbuf[2], macbuf[3], macbuf[4], macbuf[5]);

	// char * 2 + ':'
	maclen = (M25P80_CONFIG_MAC_SIZE * 3) - 1;

	return maclen;
}

static ssize_t m25p80_eth_mac_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	char macbuf[M25P80_CONFIG_STRING_LENGTH] = {0};

	sscanf(buf, "%02x:%02x:%02x:%02x:%02x:%02x",
			(unsigned int *)&macbuf[0], (unsigned int *)&macbuf[1], (unsigned int *)&macbuf[2],
			(unsigned int *)&macbuf[3], (unsigned int *)&macbuf[4], (unsigned int *)&macbuf[5]);

	ret = m25p80_config_control("eth_mac", macbuf, 1);

	if (ret < 0)
		dev_err(dev, "[%s] invalid board config item\n", __func__);

	return count;
}

static DEVICE_ATTR(eth_mac, 0644, m25p80_eth_mac_show, m25p80_eth_mac_store);

static struct attribute *m25p80_attributes[] = {
	&dev_attr_serial_number.attr,
	&dev_attr_board_number.attr,
	&dev_attr_board_id.attr,
	&dev_attr_eth_mac.attr,
	NULL
};

static const struct attribute_group m25p80_attr_group = {
	.attrs = m25p80_attributes,
};
#endif

/*
 * board specific setup should have ensured the SPI clock used here
 * matches what the READ command supports, at least until this driver
 * understands FAST_READ (for clocks over 25 MHz).
 */
static int m25p_probe(struct spi_device *spi)
{
	struct flash_platform_data	*data;
	struct m25p *flash;
	struct spi_nor *nor;
	enum read_mode mode = SPI_NOR_NORMAL;
	char *flash_name;
	int ret;

	data = dev_get_platdata(&spi->dev);

	flash = devm_kzalloc(&spi->dev, sizeof(*flash), GFP_KERNEL);
	if (!flash)
		return -ENOMEM;

	nor = &flash->spi_nor;

	/* install the hooks */
	nor->read = m25p80_read;
	nor->write = m25p80_write;
	nor->write_reg = m25p80_write_reg;
	nor->read_reg = m25p80_read_reg;

	nor->dev = &spi->dev;
	spi_nor_set_flash_node(nor, spi->dev.of_node);
	nor->priv = flash;

	spi_set_drvdata(spi, flash);
	flash->spi = spi;

	if (spi->mode & SPI_RX_QUAD)
		mode = SPI_NOR_QUAD;
	else if (spi->mode & SPI_RX_DUAL)
		mode = SPI_NOR_DUAL;

	if (data && data->name)
		nor->mtd.name = data->name;

	/* For some (historical?) reason many platforms provide two different
	 * names in flash_platform_data: "name" and "type". Quite often name is
	 * set to "m25p80" and then "type" provides a real chip name.
	 * If that's the case, respect "type" and ignore a "name".
	 */
	if (data && data->type)
		flash_name = data->type;
	else if (!strcmp(spi->modalias, "spi-nor"))
		flash_name = NULL; /* auto-detect */
	else
		flash_name = spi->modalias;

	ret = spi_nor_scan(nor, flash_name, mode);
	if (ret)
		return ret;

	ret = mtd_device_register(&nor->mtd, data ? data->parts : NULL,
				   data ? data->nr_parts : 0);
				   
	if (ret < 0) {
		dev_err(&spi->dev, "mtd_device_register failed : %d\n", ret);
		return ret;
	}
#ifdef M25P80_SYSFS_BOARD_CONFIG
	ret = m25p80_board_config_init(&spi->dev);
	
	if (ret) {
		dev_err(&spi->dev, "board config init error : %d\n", ret);
		return ret;
	}	

	ret = sysfs_create_group(&spi->dev.kobj, &m25p80_attr_group);
	
	if (ret) {
		dev_err(&spi->dev, "create sysfs group error\n");
		return ret;
	}
#endif

	return ret;
}


static int m25p_remove(struct spi_device *spi)
{
	struct m25p	*flash = spi_get_drvdata(spi);

	/* Clean up MTD stuff. */
	return mtd_device_unregister(&flash->spi_nor.mtd);
}

/*
 * Do NOT add to this array without reading the following:
 *
 * Historically, many flash devices are bound to this driver by their name. But
 * since most of these flash are compatible to some extent, and their
 * differences can often be differentiated by the JEDEC read-ID command, we
 * encourage new users to add support to the spi-nor library, and simply bind
 * against a generic string here (e.g., "jedec,spi-nor").
 *
 * Many flash names are kept here in this list (as well as in spi-nor.c) to
 * keep them available as module aliases for existing platforms.
 */
static const struct spi_device_id m25p_ids[] = {
	/*
	 * Allow non-DT platform devices to bind to the "spi-nor" modalias, and
	 * hack around the fact that the SPI core does not provide uevent
	 * matching for .of_match_table
	 */
	{"spi-nor"},

	/*
	 * Entries not used in DTs that should be safe to drop after replacing
	 * them with "spi-nor" in platform data.
	 */
	{"s25sl064a"},	{"w25x16"},	{"m25p10"},	{"m25px64"},

	/*
	 * Entries that were used in DTs without "jedec,spi-nor" fallback and
	 * should be kept for backward compatibility.
	 */
	{"at25df321a"},	{"at25df641"},	{"at26df081a"},
	{"mr25h256"},
	{"mx25l4005a"},	{"mx25l1606e"},	{"mx25l6405d"},	{"mx25l12805d"},
	{"mx25l25635e"},{"mx66l51235l"},
	{"n25q064"},	{"n25q128a11"},	{"n25q128a13"},	{"n25q512a"},
	{"s25fl256s1"},	{"s25fl512s"},	{"s25sl12801"},	{"s25fl008k"},
	{"s25fl064k"},
	{"sst25vf040b"},{"sst25vf016b"},{"sst25vf032b"},{"sst25wf040"},
	{"m25p40"},	{"m25p80"},	{"m25p16"},	{"m25p32"},
	{"m25p64"},	{"m25p128"},
	{"w25x80"},	{"w25x32"},	{"w25q32"},	{"w25q32dw"},
	{"w25q80bl"},	{"w25q128"},	{"w25q256"},

	/* Flashes that can't be detected using JEDEC */
	{"m25p05-nonjedec"},	{"m25p10-nonjedec"},	{"m25p20-nonjedec"},
	{"m25p40-nonjedec"},	{"m25p80-nonjedec"},	{"m25p16-nonjedec"},
	{"m25p32-nonjedec"},	{"m25p64-nonjedec"},	{"m25p128-nonjedec"},

	{ },
};
MODULE_DEVICE_TABLE(spi, m25p_ids);

static const struct of_device_id m25p_of_table[] = {
	/*
	 * Generic compatibility for SPI NOR that can be identified by the
	 * JEDEC READ ID opcode (0x9F). Use this, if possible.
	 */
	{ .compatible = "jedec,spi-nor" },
	{}
};
MODULE_DEVICE_TABLE(of, m25p_of_table);

static struct spi_driver m25p80_driver = {
	.driver = {
		.name	= "m25p80",
		.of_match_table = m25p_of_table,
	},
	.id_table	= m25p_ids,
	.probe	= m25p_probe,
	.remove	= m25p_remove,

	/* REVISIT: many of these chips have deep power-down modes, which
	 * should clearly be entered on suspend() to minimize power use.
	 * And also when they're otherwise idle...
	 */
};

module_spi_driver(m25p80_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Mike Lavender");
MODULE_DESCRIPTION("MTD SPI driver for ST M25Pxx flash chips");
