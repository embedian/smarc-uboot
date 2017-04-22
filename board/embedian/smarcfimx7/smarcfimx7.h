/*
 * smarcfimx7.h
 *
 * Embedian SMARC-FiMX7 boards information header
 *
 * Copyright (C) 2017, Embedian, Inc. - http://www.embedian.com/
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef _SMARCFIMX7_H_
#define _SMARCFIMX7_H_
/*
* SMARC-FiMX7 Config held in module eeprom device.
*
* Header Format
*
*  Name                 Size            Contents
*                       (Bytes)
*-------------------------------------------------------------
*  Header               4       0xAA, 0x55, 0x33, 0xEE
*
*  Board Name           8       Name for board in ASCII.
*                               example "SMCMX7D1" = "SMARC-FiMX7
*                               Dual Core and 1GB DDR3 memory". "SMCMX7S0" = 
*				SMARC-FiMX7 Computer on Module with solo Core 
*				and 512MB DDR3L Configuration
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

struct smarcfimx7_id {
	unsigned int  magic;
	char name[HDR_NAME_LEN];
	char version[4];
	char serial[12];
	char config[32];
	char mac_addr[HDR_NO_OF_MAC_ADDR][HDR_ETH_ALEN];
};

static inline int board_is_smcmx7s0(struct smarcfimx7_id *header)
{
        return !strncmp(header->name, "SMCMX7S0", HDR_NAME_LEN);
}

static inline int board_is_smcmx7d1(struct smarcfimx7_id *header)
{
        return !strncmp(header->name, "SMCMX7D1", HDR_NAME_LEN);
}

/*
 * Read ethernet MAC address from EEPROM for SMARC-FiMX7 compatible boards.
 * Returns 1 if found, 0 otherwise.
 */
int smarcfimx7_read_mac_address(uint8_t *buf)
{
#ifdef CONFIG_SYS_I2C_EEPROM_ADDR
        /* Read MAC address. */
        i2c_set_bus_num(1);
        if (i2c_read(CONFIG_SYS_I2C_EEPROM_ADDR, 0x3C,
                CONFIG_SYS_I2C_EEPROM_ADDR_LEN, (uint8_t *) &buf[0], 6))
                goto i2cerr;

        /* Check that MAC address is valid. */
        if (!is_valid_ethaddr(buf))
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
void smarcfimx7_sync_env_enetaddr(uint8_t *rom_enetaddr)
{
        i2c_set_bus_num(1);
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

int smarcfimx7_read_mac1_address(uint8_t *buf)
{
#ifdef CONFIG_SYS_I2C_EEPROM_ADDR
        /* Read MAC address. */
        i2c_set_bus_num(1);
        if (i2c_read(CONFIG_SYS_I2C_EEPROM_ADDR, 0x42,
                CONFIG_SYS_I2C_EEPROM_ADDR_LEN, (uint8_t *) &buf[0], 6))
                goto i2cerr;

        /* Check that MAC address is valid. */
        if (!is_valid_ethaddr(buf))
                goto err;

        return 1; /* Found */

i2cerr:
        printf("Read 2nd GBE MAC address from EEPROM @ 0x%02x failed\n",
                CONFIG_SYS_I2C_EEPROM_ADDR);
err:
#endif /* CONFIG_SYS_I2C_EEPROM_ADDR */

        return 0;
}

/*
 * If there is no MAC address in the environment, then it will be initialized
 * (silently) from the value in the EEPROM.
 */
void smarcfimx7_sync_env_enet1addr(uint8_t *rom_enet1addr)
{
        i2c_set_bus_num(1);
        uint8_t env_enet1addr[6];
        int ret;

        ret = eth_getenv_enetaddr_by_index("eth", 1, env_enet1addr);
        if (!ret) {
                /*
                 * There is no MAC address in the environment, so we
                 * initialize it from the value in the EEPROM.
                 */
                debug("### Setting environment from EEPROM MAC address = "
                        "\"%pM\"\n",
                        env_enet1addr);
                ret = !eth_setenv_enetaddr("eth1addr", rom_enet1addr);
        }
        if (!ret)
                printf("Failed to set 2nd GBE mac address from EEPROM: %d\n", ret);
}

#endif
