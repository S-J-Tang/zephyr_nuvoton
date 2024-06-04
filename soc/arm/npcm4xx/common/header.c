/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 by Nuvoton Electronics Corporation                      */
/* All rights reserved.							      */
/*----------------------------------------------------------------------------*/

#include <stddef.h>
#include <stdint.h>
#include <header.h>

extern unsigned long _vector_table;
#ifndef CONFIG_XIP
extern unsigned long __main_fw_seg_size__;
#endif
extern unsigned long __main_fw_seg_end__;

void Rom_hook2(void)
{
#define adr_FIU_FwInfo_status	0x100c59d4
#define bit_auth_pass		0x02
#define M8(addr) (*((volatile uint8_t *) (addr)))
#define M32(addr) (*((volatile unsigned long *) (addr)))
#define DEVALT5		0x400c3015
#define EMC_CTL		0x400c304d
#define GPIOB0_DEVALT	6
#define RMII_EN		0
#define GPIOB_IN	0x40097001
#define GPIOB_DIR	0x40097002
#define GPIOB_PULL	0x40097003
#define GPIOB_PUD	0x40097004
#define GPIOB0_BIT	0
#define BIT(x)		(1<<(x))

    M8(DEVALT5) &= ~BIT(GPIOB0_DEVALT);
    M8(EMC_CTL) &= ~BIT(RMII_EN);
    M8(GPIOB_DIR) &= ~BIT(GPIOB0_BIT);
    M8(GPIOB_PULL) |= BIT(GPIOB0_BIT);
    M8(GPIOB_PUD) &= ~BIT(GPIOB0_BIT);

    if(M32(adr_FIU_FwInfo_status) & bit_auth_pass)
    {
	if (0 == (M8(GPIOB_IN) & BIT(GPIOB0_BIT)))
        {
            M32(adr_FIU_FwInfo_status) &= ~bit_auth_pass;
        }
    }

    M8(GPIOB_PULL) &= ~BIT(GPIOB0_BIT);
}

__attribute__((section(".header"))) struct FIRMWARE_HEDAER_TYPE fw_header = {
	.hUserFWEntryPoint = (uint32_t)(&_vector_table),
	.hUserFWRamCodeFlashStart = CONFIG_FLASH_BASE_ADDRESS +
		NPCM4XX_FW_HEADER_SIZE,
#ifndef CONFIG_XIP
	.hUserFWRamCodeFlashEnd = CONFIG_FLASH_BASE_ADDRESS +
		NPCM4XX_FW_HEADER_SIZE +
		(uint32_t)(&__main_fw_seg_size__),
	.hUserFWRamCodeRamStart = (uint32_t)(&_vector_table),
#else
	.hUserFWRamCodeFlashEnd = CONFIG_FLASH_BASE_ADDRESS +
		NPCM4XX_FW_HEADER_SIZE,
	.hUserFWRamCodeRamStart = 0,
#endif

	.hRomHook1Ptr = 0, /* Hook 1 muse be flash code */
	.hRomHook2Ptr = Rom_hook2,
	.hRomHook3Ptr = 0,
	.hRomHook4Ptr = 0,

	/* seg 0 - fw image information */
	.hFwSeg[0].hOffset = 0x210, /* fixed, can't change */
	.hFwSeg[0].hSize = 0x500, /* fixed, can't change */

	/* seg 1 - RomHook (no use) */
	.hFwSeg[1].hOffset = NPCM4XX_FW_HEADER_SIZE,
	.hFwSeg[1].hSize = NPCM4XX_FW_HEADER_SIZE,

	/* seg 2 - fw body start and end offset (flash view) */
	.hFwSeg[2].hOffset = NPCM4XX_FW_HEADER_SIZE,
#ifndef CONFIG_XIP
	.hFwSeg[2].hSize = (uint32_t)(&__main_fw_seg_end__) - CONFIG_SRAM_BASE_ADDRESS,
#else
	.hFwSeg[2].hSize = (uint32_t)(&__main_fw_seg_end__) - CONFIG_FLASH_BASE_ADDRESS,
#endif

	/* seg 3 - reserved */
	.hFwSeg[3].hOffset = 0, /* Reserved for fan table and fill in BinGentool */
	.hFwSeg[3].hSize = 0, /* Reserved for fan table and fill in BinGentool */
};
