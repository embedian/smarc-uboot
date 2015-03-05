/*
 * smarcfimx6.h
 *
 * Embedian SMARC-FiMX6 boards information header
 *
 * Copyright (C) 2015, Embedian, Inc. - http://www.embedian.com/
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef _SMARCFIMX6_H_
#define _SMARCFIMX6_H_

/*
* SMARC-FiMX6 Config held in module eeprom device.
*
* Header Format
*
*  Name                 Size            Contents
*                       (Bytes)
*-------------------------------------------------------------
*  Header               4       0xAA, 0x55, 0x33, 0xEE
*
*  Board Name           8       Name for board in ASCII.
*                               example "FIMX6Q1G" = "SMARC-FiMX6
*                               Quad Core and 1GB DDR3 memory"
*
*  Version              4       Hardware version code for board in
*                               in ASCII. "00A0" = rev.0A
*  Serial Number        12      Serial number of the board. This is a 12
*                               character string which is: WWYYMSkknnnn, where
*                               WW = 2 digit week of the year of production
*                               YY = 2 digit year of production
*				kk = 2 digit module variants
*                               nnnn = incrementing board number
*  Configuration Option 32      Codes to show the configuration
*                               setup on this board.
*  MAC Address (LAN1)		MAC Address for FEC controller
*  MAC Address (LAN2, if any)	MAC Address for 2nd LAN (if any)
*  Available    32700           Available space for other non-volatile
*                               codes/data
*/

#define HDR_NO_OF_MAC_ADDR	3
#define HDR_ETH_ALEN		6
#define HDR_NAME_LEN		8

struct smarcfimx6_id {
	unsigned int  magic;
	char name[HDR_NAME_LEN];
	char version[4];
	char serial[12];
	char config[32];
	char mac_addr[HDR_NO_OF_MAC_ADDR][HDR_ETH_ALEN];
};

static inline int board_is_fimx6slo(struct smarcfimx6_id *header)
{
        return !strncmp(header->name, "SMCMXSLO", HDR_NAME_LEN);
}

static inline int board_is_fimx6u1g(struct smarcfimx6_id *header)
{
        return !strncmp(header->name, "SMCMXU1G", HDR_NAME_LEN);
}

static inline int board_is_fimx6d1g(struct smarcfimx6_id *header)
{
        return !strncmp(header->name, "SMCMXD1G", HDR_NAME_LEN);
}

static inline int board_is_fimx6d2g(struct smarcfimx6_id *header)
{
	return !strncmp(header->name, "SMCMXD2G", HDR_NAME_LEN);
}

static inline int board_is_fimx6q1g(struct smarcfimx6_id *header)
{
        return !strncmp(header->name, "SMCMXQ1G", HDR_NAME_LEN);
}

static inline int board_is_fimx6q2g(struct smarcfimx6_id *header)
{
        return !strncmp(header->name, "SMCMXQ2G", HDR_NAME_LEN);
}

/*
 * Read ethernet MAC address from EEPROM for SMARC-FiMX6DVEVM compatible boards.
 * Returns 1 if found, 0 otherwise.
 */
int smarcfimx6_read_mac_address(uint8_t *buf)
{
#ifdef CONFIG_SYS_I2C_EEPROM_ADDR
        /* Read MAC address. */
	i2c_set_bus_num(0);
        if (i2c_read(CONFIG_SYS_I2C_EEPROM_ADDR, 0x3C,
                CONFIG_SYS_I2C_EEPROM_ADDR_LEN, (uint8_t *) &buf[0], 6))
                goto i2cerr;

        /* Check that MAC address is valid. */
        if (!is_valid_ether_addr(buf))
                goto err;

        return 1; /* Found */

i2cerr:
        printf("Read from EEPROM @ 0x%02x failed\n",
                CONFIG_SYS_I2C_EEPROM_ADDR);
err:
#endif /* CONFIG_SYS_I2C_EEPROM_ADDR */

        return 0;
}

/*
 * If there is no MAC address in the environment, then it will be initialized
 * (silently) from the value in the EEPROM.
 */
void smarcfimx6_sync_env_enetaddr(uint8_t *rom_enetaddr)
{
        uint8_t env_enetaddr[6];
        int ret;

        ret = eth_getenv_enetaddr_by_index("eth", 0, env_enetaddr);
        if (!ret) {
                /*
                 * There is no MAC address in the environment, so we
                 * initialize it from the value in the EEPROM.
                 */
                debug("### Setting environment from EEPROM MAC address = "
                        "\"%pM\"\n",
                        env_enetaddr);
                ret = !eth_setenv_enetaddr("ethaddr", rom_enetaddr);
        }
        if (!ret)
                printf("Failed to set mac address from EEPROM: %d\n", ret);
}

#endif
