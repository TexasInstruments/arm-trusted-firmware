/*
 * Copyright (c) 2024, Texas Instruments Inc. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <lib/mmio.h>
#include "lpm_trace.h"
#include "ddr.h"

#define DENALI_CTL_167__SFR_OFFS	0x29c
#define DENALI_CTL_276__SFR_OFFS	0x450
#define DENALI_CTL_277__SFR_OFFS	0x454
#define DENALI_CTL_158__SFR_OFFS	0x278
#define DENALI_CTL_353__SFR_OFFS	0x584
#define DENALI_CTL_345__SFR_OFFS	0x564
#define DENALI_CTL_337__SFR_OFFS	0x544
#define LP_MODE_LONG_SELF_REFRESH    0x31 //0x51 AU
#define LP_MODE_LONG_SELF_REFRESH_PHY_CTRL     0x51 //0x51 AU
#define LP_MODE_LONG_SELF_REFRESH_EXIT     0x2 //0x51 AU
#define LPDDR4_DRAM_CLASS_REG_VALUE  0xB

#define CSL_DDR16SS0_REGS_SS_CFG_SSCFG_BASE                                                        (0xf300000UL)
#define CSL_DDR16SS0_CTLPHY_WRAP_CTL_CFG_CTLCFG_BASE                                               (0xf308000UL)

#define CHNG_DDR4_FSP_REQ      			(0x0U)
#define CHNG_DDR4_FSP_ACK  				(0x4U)
#define DDR4_FSP_CLKCHNG_REQ			(0x80U)
#define DDR4_FSP_CLKCHNG_ACK			(0x84U)
#define DDR32SS_PMCTRL					(0x1000U)
#define CSL_WKUP_CTRL_MMR_CFG4_DDR32SS_PMCTRL                            (0x00001000U)

#define NUM_DDR_CTL_REG     423 // 0-422
#define NUM_DDR_PI_REG      345 // 0-344
#define NUM_DDR_DATA_0_REG  126 // 0-125
#define NUM_DDR_DATA_1_REG  126 // 256-381
#define NUM_DDR_ADDR_0_REG   43 // 512-554
#define NUM_DDR_ADDR_1_REG   43 // 768-810
#define NUM_DDR_ADDR_2_REG   43 // 1024-1066
#define NUM_DDR_PHY_REG     126 // 1280-1405
#define NUM_ALL_DDR_REG (NUM_DDR_CTL_REG + NUM_DDR_PI_REG + NUM_DDR_DATA_0_REG + NUM_DDR_DATA_1_REG + NUM_DDR_ADDR_0_REG + NUM_DDR_ADDR_1_REG + NUM_DDR_ADDR_2_REG + NUM_DDR_PHY_REG)
#define DENALI_CTL_00_DATA  0x00000B00
#define DENALI_PI_00_DATA   0x00000B00

#define DDRSS_PHY_Core_REGISTER_BLOCK__OFFS	0x5400
/* Macros for register block Address_Slice_2 */
#define DDRSS_Address_Slice_2_REGISTER_BLOCK__OFFS	0x5000
/* Macros for register block Address_Slice_1 */
#define DDRSS_Address_Slice_1_REGISTER_BLOCK__OFFS	0x4c00
/* Macros for register block Address_Slice_0 */
#define DDRSS_Address_Slice_0_REGISTER_BLOCK__OFFS	0x4800
/* Macros for register block Data_Slice_1 */
#define DDRSS_Data_Slice_1_REGISTER_BLOCK__OFFS	0x4400
/* Macros for register block Data_Slice_0 */
#define DDRSS_Data_Slice_0_REGISTER_BLOCK__OFFS	0x4000
/* Macros for register block PI */
#define DDRSS_PI_REGISTER_BLOCK__OFFS	0x2000

#define DENALI_PI_83__SFR_OFFS	0x14c

#define DENALI_CTL_342__SFR_OFFS	0x558

#define DENALI_PI_0__SFR_OFFS	0x0
#define DENALI_CTL_0__SFR_OFFS	0x0
// -----------------------------------------------------------------------------
// Set these based on DDR memory [16GB] - Value of 0x11 sets 16GB
// -----------------------------------------------------------------------------
#define SDRAM_IDX  0xf
#define REGION_IDX 0xf

#define CSL_EMIF_SSCFG_V2A_CTL_REG                                             (0x00000020U)
#define CSL_EMIF_CTLCFG_DENALI_PHY_1306                                        (0x00005468U)
#define CSL_EMIF_CTLCFG_DENALI_CTL_21                                          (0x00000054U)
#define CSL_EMIF_CTLCFG_DENALI_CTL_20                                          (0x00000050U)
#define CSL_EMIF_CTLCFG_DENALI_CTL_106                                         (0x000001A8U)
#define CSL_EMIF_CTLCFG_DENALI_PI_4                                            (0x00002010U)
#define CSL_EMIF_CTLCFG_DENALI_PI_6                                            (0x00002018U)
#define CSL_EMIF_CTLCFG_DENALI_PI_23                                           (0x0000205CU)
#define CSL_EMIF_CTLCFG_DENALI_PI_33                                           (0x00002084U)
#define CSL_EMIF_CTLCFG_DENALI_PI_67                                           (0x0000210CU)
#define CSL_EMIF_CTLCFG_DENALI_PI_133                                          (0x00002214U)
#define CSL_EMIF_CTLCFG_DENALI_PI_134                                          (0x00002218U)
#define CSL_EMIF_CTLCFG_DENALI_PI_138                                          (0x00002228U)
#define CSL_EMIF_CTLCFG_DENALI_PI_181                                          (0x000022D4U)
#define CSL_EMIF_CTLCFG_DENALI_PI_182                                          (0x000022D8U)
#define CSL_EMIF_CTLCFG_DENALI_PI_188                                          (0x000022F0U)
#define CSL_EMIF_CTLCFG_DENALI_PI_189                                          (0x000022F4U)
#define CSL_EMIF_CTLCFG_DENALI_PI_190                                          (0x000022F8U)
#define CSL_EMIF_CTLCFG_DENALI_PI_191                                          (0x000022FCU)
#define CSL_EMIF_CTLCFG_DENALI_PI_192                                          (0x00002300U)
#define CSL_EMIF_CTLCFG_DENALI_PI_193                                          (0x00002304U)
#define CSL_EMIF_CTLCFG_DENALI_PI_199                                          (0x0000231CU)
#define CSL_EMIF_CTLCFG_DENALI_PI_223                                          (0x0000237CU)
#define CSL_EMIF_CTLCFG_DENALI_PI_226                                          (0x00002388U)
#define CSL_EMIF_CTLCFG_DENALI_PI_229                                          (0x00002394U)

// -----------------------------------------------------------------------------
// DDR EMIF Handle
// -----------------------------------------------------------------------------
typedef struct emif_handle_s{
  // Configuration address base
  uint64_t           ss_cfg_base_addr;
  uint64_t           ctl_cfg_base_addr;
}emif_handle_t;

__wkupsramdata emif_handle_t Emifhandle;
__wkupsramdata uint32_t ddrss_save_restore[NUM_ALL_DDR_REG];

// poll_for_init_completion - Sub-routine to poll for init completion
__wkupsramfunc void poll_for_init_completion(struct emif_handle_s * h) {
    #if defined(CTL_INIT_ONLY)
        while(((mmio_read_32(h->ctl_cfg_base_addr + DENALI_CTL_342__SFR_OFFS)) & 0x02000000)!= 0x02000000); // Poll for CTL Init completion
    #elif defined(PI_INIT_ONLY)
        while(((mmio_read_32(h->ctl_cfg_base_addr + DDRSS_PI_REGISTER_BLOCK__OFFS + DENALI_PI_83__SFR_OFFS)) & 0x1) != 0x1); // Poll for PI Init completion
    #else
	lpm_seq_trace(0x6);
        while(((mmio_read_32(h->ctl_cfg_base_addr + DDRSS_PI_REGISTER_BLOCK__OFFS + DENALI_PI_83__SFR_OFFS)) & 0x1) != 0x1); // Poll for PI Init completion
	lpm_seq_trace(0x7);
        while(((mmio_read_32(h->ctl_cfg_base_addr + DENALI_CTL_342__SFR_OFFS)) & 0x02000000)!= 0x02000000); // Poll for CTL Init completion
	lpm_seq_trace(0x8);

    #endif
}

/* Write to a specific field in an MMR. */
__wkupsramfunc void Write_MMR_Field(uint32_t mmr_address, uint32_t field_value, uint32_t width, uint32_t leftshift)
{
    uint32_t val;
    uint32_t mask;
    val = mmio_read_32(mmr_address);   //Grab the MMR value
    mask = (((1 << width) - 1) << leftshift); //Build a mask of 1s for the field.
    mask = (~(mask)); //Invert the mask so that the field will be zero'd out with the AND operation.
    val &= mask; //Zero out the field in the register.
    val |= (field_value << leftshift); //Assign the value to that specific field.
	mmio_write_32(mmr_address, val);
}

__wkupsramfunc void configure_sdram_region_idx(struct emif_handle_s * h, uint32_t sdram_idx, uint32_t region_idx) {
    uint32_t rd_val;
    rd_val = mmio_read_32(h->ss_cfg_base_addr + CSL_EMIF_SSCFG_V2A_CTL_REG);
    rd_val = (rd_val & 0xFFFFFC00);
    rd_val = rd_val | (sdram_idx << 5) | (region_idx);
    mmio_write_32((h->ss_cfg_base_addr + CSL_EMIF_SSCFG_V2A_CTL_REG), rd_val); // Programming the region_idx and sdram_idx fields for address mapping [Set 9:5 and 4:0 to 0x11 for 8GB]
}

__wkupsramfunc void sdram_region_idx_cfg(struct emif_handle_s * h) {
    configure_sdram_region_idx(h, SDRAM_IDX, REGION_IDX); // Programming the region_idx and sdram_idx fields for address mapping [Set 9:5 and 4:0 to 0x11 for 8GB]
}

__wkupsramfunc void put_ddr_in_sr(bool enable){
    
    if (enable){
        uint32_t lp_status = 0U;
        lpm_seq_trace(0x3);
        // Program Self Refresh mode
        Write_MMR_Field(DDRSS0_CTRL_BASE + DENALI_CTL_158__SFR_OFFS, LP_MODE_LONG_SELF_REFRESH_PHY_CTRL, 7U, 8U);  // force INT_MASK_LOWPOWER to NOT mask interrupts for low power complete and timeout
        // Poll for Self Refresh Mode change
        Write_MMR_Field(DDRSS0_CTRL_BASE + DENALI_CTL_353__SFR_OFFS, 0x0U, 16U, 16U);  // force INT_MASK_LOWPOWER to NOT mask interrupts for low power complete and timeout
        lp_status = (mmio_read_32(DDRSS0_CTRL_BASE + DENALI_CTL_337__SFR_OFFS) & 0x10000U);
        while(lp_status != 0x10000U){
            lp_status = (mmio_read_32(DDRSS0_CTRL_BASE + DENALI_CTL_337__SFR_OFFS) & 0x10000U);
        }
        Write_MMR_Field(DDRSS0_CTRL_BASE + DENALI_CTL_345__SFR_OFFS, 0x1U, 16U, 16U);  // clear low power complete
        lp_status = ((mmio_read_32(DDRSS0_CTRL_BASE + DENALI_CTL_167__SFR_OFFS) & 0x7F00U) >> 8U);
        while(lp_status != 0x4FU){
            lp_status = ((mmio_read_32(DDRSS0_CTRL_BASE + DENALI_CTL_167__SFR_OFFS) & 0x7F00U) >> 8U);
        }
        lpm_seq_trace(0x4);
    } else {
        uint32_t lp_status = 0U;
        lpm_seq_trace(0x5);
        // Program Self Refresh mode
        Write_MMR_Field(DDRSS0_CTRL_BASE + DENALI_CTL_158__SFR_OFFS, LP_MODE_LONG_SELF_REFRESH_EXIT, 7U, 8U);  // force INT_MASK_LOWPOWER to NOT mask interrupts for low power complete and timeout
        // Poll for Self Refresh Mode change
        Write_MMR_Field(DDRSS0_CTRL_BASE + DENALI_CTL_353__SFR_OFFS, 0x0U, 16U, 16U);  // force INT_MASK_LOWPOWER to NOT mask interrupts for low power complete and timeout
        lp_status = (mmio_read_32(DDRSS0_CTRL_BASE + DENALI_CTL_337__SFR_OFFS) & 0x10000U);
        while(lp_status != 0x10000U){
            lp_status = (mmio_read_32(DDRSS0_CTRL_BASE + DENALI_CTL_337__SFR_OFFS) & 0x10000U);
        }
        Write_MMR_Field(DDRSS0_CTRL_BASE + DENALI_CTL_345__SFR_OFFS, 0x1U, 16U, 16U);  // clear low power complete
        lp_status = ((mmio_read_32(DDRSS0_CTRL_BASE + DENALI_CTL_167__SFR_OFFS) & 0x7F00U) >> 8U);
        while(lp_status != 0x40U){
            lp_status = ((mmio_read_32(DDRSS0_CTRL_BASE + DENALI_CTL_167__SFR_OFFS) & 0x7F00U) >> 8U);
        }
        lpm_seq_trace(0x6);
    }
}


__wkupsramfunc int32_t put_ddr_in_rtc_lpm(void){

	uint32_t req, req_type;
	uint32_t lp_status = 0U;

	// disable auto entry / exit
	Write_MMR_Field(DDRSS0_CTRL_BASE + DENALI_CTL_167__SFR_OFFS, 0U, 4U, 16U);
	Write_MMR_Field(DDRSS0_CTRL_BASE + DENALI_CTL_167__SFR_OFFS, 0U, 4U, 24U);
	Write_MMR_Field(DDRSS0_CTRL_BASE + DENALI_CTL_276__SFR_OFFS, 1U, 1U, 24U); //mr_fsp_data_valid_f0
	// KL Note: If a DFS request is made for frequency set N, if the mr_fsp_data_valid_fN is not set to a �b1, the DFS operation will not occur
	Write_MMR_Field(DDRSS0_CTRL_BASE + DENALI_CTL_277__SFR_OFFS, 1U, 1U, 8U); //mr_fsp_data_valid_f2
	Write_MMR_Field((WKUP_CTRL_MMR_SEC_4_BASE + CHNG_DDR4_FSP_REQ), 0x0U, 2U, 0U); //CHNG_DDR4_FSP_REQ req_type bits 1:0
	Write_MMR_Field((WKUP_CTRL_MMR_SEC_4_BASE + CHNG_DDR4_FSP_REQ), 0x1U, 1U, 8U); //CHNG_DDR4_FSP_REQ req bit 8
	req = (mmio_read_32((WKUP_CTRL_MMR_SEC_4_BASE + DDR4_FSP_CLKCHNG_REQ)) & 0x80U); //DDR4_FSP_CLKCHNG_REQ req bit 7
	while(req == 0x0U){
		req = (mmio_read_32((WKUP_CTRL_MMR_SEC_4_BASE + DDR4_FSP_CLKCHNG_REQ)) & 0x80U); //DDR4_FSP_CLKCHNG_REQ req bit 7
	}
	req_type = (req & 0x03U); //DDR4_FSP_CLKCHNG_REQ req_type bits 1:0
	if(req_type == 0U){
		Write_MMR_Field((MAIN_PLL_MMR_BASE + (0U * 0x1000U) + ((2U*0x4U) + 0x80U)), 0x4FU, 7U, 0U); //HSDIV2 2000/80 = 25MHz
	}else{
		return -1;
	}
	mmio_write_32(((WKUP_CTRL_MMR_SEC_4_BASE + DDR4_FSP_CLKCHNG_ACK)), 0x1U); //set the ack bit
	while(((mmio_read_32(((WKUP_CTRL_MMR_SEC_4_BASE + DDR4_FSP_CLKCHNG_REQ)))) & 0x80U) == 0x80U);
	mmio_write_32(((WKUP_CTRL_MMR_SEC_4_BASE + DDR4_FSP_CLKCHNG_ACK)), 0x0U); //clear the ack bit
	req = (mmio_read_32((WKUP_CTRL_MMR_SEC_4_BASE + CHNG_DDR4_FSP_ACK)) & 0x80U); //CHNG_DDR4_FSP_ACK ack bit 7
	while(req == 0x0U){
		req = (mmio_read_32((WKUP_CTRL_MMR_SEC_4_BASE + CHNG_DDR4_FSP_ACK)) & 0x80U); //CHNG_DDR4_FSP_ACK ack bit 7
	}
    req_type = (mmio_read_32((WKUP_CTRL_MMR_SEC_4_BASE + CHNG_DDR4_FSP_ACK)) & 0x01U);
	if(req_type == 0U){
	}else{
		return -2;
	}
	req = mmio_read_32(WKUP_CTRL_MMR_SEC_4_BASE + CHNG_DDR4_FSP_REQ);
	req &= ~0x100U;
	mmio_write_32((WKUP_CTRL_MMR_SEC_4_BASE + CHNG_DDR4_FSP_REQ), req); // de-assert req
	// Put the device into SRPD long with memory clock gating using external hardware interface. (LP_EXT_CMD=�h19)
	// Wait for LP_STATE = �h49 (DDR4) / �h4E (LPDDR4)
	// Program Self Refresh mode
	Write_MMR_Field(DDRSS0_CTRL_BASE + DENALI_CTL_158__SFR_OFFS, LP_MODE_LONG_SELF_REFRESH, 7U, 8U);  // force INT_MASK_LOWPOWER to NOT mask interrupts for low power complete and timeout
	// Poll for Self Refresh Mode change
	Write_MMR_Field(DDRSS0_CTRL_BASE + DENALI_CTL_353__SFR_OFFS, 0x0U, 16U, 16U);  // force INT_MASK_LOWPOWER to NOT mask interrupts for low power complete and timeout
	lp_status = (mmio_read_32(DDRSS0_CTRL_BASE + DENALI_CTL_337__SFR_OFFS) & 0x10000U);
	while(lp_status != 0x10000U){
		lp_status = (mmio_read_32(DDRSS0_CTRL_BASE + DENALI_CTL_337__SFR_OFFS) & 0x10000U);
	}
	Write_MMR_Field(DDRSS0_CTRL_BASE + DENALI_CTL_345__SFR_OFFS, 0x1U, 16U, 16U);  // clear low power complete
	lp_status = ((mmio_read_32(DDRSS0_CTRL_BASE + DENALI_CTL_167__SFR_OFFS) & 0x7F00U) >> 8U);
	while(lp_status != 0x4EU){
		lp_status = ((mmio_read_32(DDRSS0_CTRL_BASE + DENALI_CTL_167__SFR_OFFS) & 0x7F00U) >> 8U);
	}

	//Enable DDR data retention by writing b0110 to WKUP_CTRL_MMR. DDR32SS_PMCTRL.data_retention
	Write_MMR_Field((WKUP_CTRL_MMR_SEC_4_BASE + DDR32SS_PMCTRL), 0x6U, 4U, 0U);
	Write_MMR_Field((WKUP_CTRL_MMR_SEC_4_BASE + DDR32SS_PMCTRL), 0x1U, 1U, 31U);
	lp_status = mmio_read_32((WKUP_CTRL_MMR_SEC_4_BASE + DDR32SS_PMCTRL));
	while(lp_status != ((1U<<31) | 0x6U)){
		lp_status = mmio_read_32((WKUP_CTRL_MMR_SEC_4_BASE + DDR32SS_PMCTRL));
	}
	Write_MMR_Field((WKUP_CTRL_MMR_SEC_4_BASE + DDR32SS_PMCTRL), 0x0U, 1U, 31U);

	return 0;
}

__wkupsramfunc void emif_instance_select(struct emif_handle_s * h){

    // Config Addresses
    h->ss_cfg_base_addr = (uint64_t)(CSL_DDR16SS0_REGS_SS_CFG_SSCFG_BASE);
    h->ctl_cfg_base_addr = (uint64_t)(CSL_DDR16SS0_CTLPHY_WRAP_CTL_CFG_CTLCFG_BASE);
}

__wkupsramfunc void start_PI_CTL_init(struct emif_handle_s * h) {

    uint32_t wr_init_val;

    wr_init_val = ((LPDDR4_DRAM_CLASS_REG_VALUE << 8)|0x1);
    // Set START bit in register for PI module
    mmio_write_32(h->ctl_cfg_base_addr + DDRSS_PI_REGISTER_BLOCK__OFFS + DENALI_PI_0__SFR_OFFS, wr_init_val);
    volatile int i = 0;
    for(i = 0; i < 1000; i++); // Small delay
    // Set START bit in register for controller
    mmio_write_32(h->ctl_cfg_base_addr + DENALI_CTL_0__SFR_OFFS, wr_init_val);
}

__wkupsramfunc void save_ddr_registers(struct emif_handle_s * h) {

    int i, j;

    // DDRSS Memory Base
    uint32_t DDR_CTL_REG_BASE = h->ctl_cfg_base_addr;
    uint32_t DDR_PI_REG_BASE = (h->ctl_cfg_base_addr) + DDRSS_PI_REGISTER_BLOCK__OFFS;
    uint32_t DDR_PHY_DATA_SLICE_0_REG_BASE = (h->ctl_cfg_base_addr) + DDRSS_Data_Slice_0_REGISTER_BLOCK__OFFS;
    uint32_t DDR_PHY_DATA_SLICE_1_REG_BASE = (h->ctl_cfg_base_addr) + DDRSS_Data_Slice_1_REGISTER_BLOCK__OFFS;
    uint32_t DDR_PHY_ADDR_SLICE_0_REG_BASE = (h->ctl_cfg_base_addr) + DDRSS_Address_Slice_0_REGISTER_BLOCK__OFFS;
    uint32_t DDR_PHY_ADDR_SLICE_1_REG_BASE = (h->ctl_cfg_base_addr) + DDRSS_Address_Slice_1_REGISTER_BLOCK__OFFS;
    uint32_t DDR_PHY_ADDR_SLICE_2_REG_BASE = (h->ctl_cfg_base_addr) + DDRSS_Address_Slice_2_REGISTER_BLOCK__OFFS;
    uint32_t DDR_PHY_CORE_REG_BASE = (h->ctl_cfg_base_addr) + DDRSS_PHY_Core_REGISTER_BLOCK__OFFS;

    j = 0;
    for(i = 0; i < NUM_DDR_CTL_REG; i++, j++) {
       ddrss_save_restore[j] = mmio_read_32(DDR_CTL_REG_BASE + i*4);
    }
    for(i = 0; i < NUM_DDR_PI_REG; i++, j++) {
       ddrss_save_restore[j] = mmio_read_32(DDR_PI_REG_BASE + i*4);
    }
    for(i = 0; i < NUM_DDR_DATA_0_REG; i++, j++) {
       ddrss_save_restore[j] = mmio_read_32(DDR_PHY_DATA_SLICE_0_REG_BASE + i*4);
    }
    for(i = 0; i < NUM_DDR_DATA_1_REG; i++, j++) {
       ddrss_save_restore[j] = mmio_read_32(DDR_PHY_DATA_SLICE_1_REG_BASE + i*4);
    }
    for(i = 0; i < NUM_DDR_ADDR_0_REG; i++, j++) {
       ddrss_save_restore[j] = mmio_read_32(DDR_PHY_ADDR_SLICE_0_REG_BASE + i*4);
    }
    for(i = 0; i < NUM_DDR_ADDR_1_REG; i++, j++) {
       ddrss_save_restore[j] = mmio_read_32(DDR_PHY_ADDR_SLICE_1_REG_BASE + i*4);
    }
    for(i = 0; i < NUM_DDR_ADDR_2_REG; i++, j++) {
       ddrss_save_restore[j] = mmio_read_32(DDR_PHY_ADDR_SLICE_2_REG_BASE + i*4);
    }
    for(i = 0; i < NUM_DDR_PHY_REG; i++, j++) {
       ddrss_save_restore[j] = mmio_read_32(DDR_PHY_CORE_REG_BASE + i*4);
    }
}


__wkupsramfunc void restore_ddr_registers(struct emif_handle_s * h) {

    int j;

    // DDRSS Memory Base
    uint32_t DDR_CTL_REG_BASE = h->ctl_cfg_base_addr;
    uint32_t DDR_PI_REG_BASE = (h->ctl_cfg_base_addr) + DDRSS_PI_REGISTER_BLOCK__OFFS;
    uint32_t DDR_PHY_DATA_SLICE_0_REG_BASE = (h->ctl_cfg_base_addr) + DDRSS_Data_Slice_0_REGISTER_BLOCK__OFFS;
    uint32_t DDR_PHY_DATA_SLICE_1_REG_BASE = (h->ctl_cfg_base_addr) + DDRSS_Data_Slice_1_REGISTER_BLOCK__OFFS;
    uint32_t DDR_PHY_ADDR_SLICE_0_REG_BASE = (h->ctl_cfg_base_addr) + DDRSS_Address_Slice_0_REGISTER_BLOCK__OFFS;
    uint32_t DDR_PHY_ADDR_SLICE_1_REG_BASE = (h->ctl_cfg_base_addr) + DDRSS_Address_Slice_1_REGISTER_BLOCK__OFFS;
    uint32_t DDR_PHY_ADDR_SLICE_2_REG_BASE = (h->ctl_cfg_base_addr) + DDRSS_Address_Slice_2_REGISTER_BLOCK__OFFS;
    uint32_t DDR_PHY_CORE_REG_BASE = (h->ctl_cfg_base_addr) + DDRSS_PHY_Core_REGISTER_BLOCK__OFFS;

    mmio_write_32(DDR_CTL_REG_BASE + DENALI_CTL_0__SFR_OFFS, DENALI_CTL_00_DATA);
    j = 1; // Skip the first CTL register write
    for(int i = 1; i < NUM_DDR_CTL_REG; i++, j++) {
            mmio_write_32(DDR_CTL_REG_BASE + i*4, ddrss_save_restore[j]);
    }
    mmio_write_32(DDR_PI_REG_BASE + DENALI_PI_0__SFR_OFFS, DENALI_PI_00_DATA);
    j++; // Skip the first PI register write
    for(int i = 1; i < NUM_DDR_PI_REG; i++, j++) {
            mmio_write_32(DDR_PI_REG_BASE + i*4, ddrss_save_restore[j]);
    }
    for(int i = 0; i < NUM_DDR_DATA_0_REG; i++, j++) {
        mmio_write_32(DDR_PHY_DATA_SLICE_0_REG_BASE + i*4, ddrss_save_restore[j]);
    }
    for(int i = 0; i < NUM_DDR_DATA_1_REG; i++, j++) {
        mmio_write_32(DDR_PHY_DATA_SLICE_1_REG_BASE + i*4, ddrss_save_restore[j]);
    }
    for(int i = 0; i < NUM_DDR_ADDR_0_REG; i++, j++) {
        mmio_write_32(DDR_PHY_ADDR_SLICE_0_REG_BASE + i*4, ddrss_save_restore[j]);
    }
    for(int i = 0; i < NUM_DDR_ADDR_1_REG; i++, j++) {
        mmio_write_32(DDR_PHY_ADDR_SLICE_1_REG_BASE + i*4, ddrss_save_restore[j]);
    }
    for(int i = 0; i < NUM_DDR_ADDR_2_REG; i++, j++) {
        mmio_write_32(DDR_PHY_ADDR_SLICE_2_REG_BASE + i*4, ddrss_save_restore[j]);
    }
    for(int i = 0; i < NUM_DDR_PHY_REG; i++, j++) {
        mmio_write_32(DDR_PHY_CORE_REG_BASE + i*4, ddrss_save_restore[j]);
    }
}

#define CANUART_WAKE_OFF_MODE				(0x1310U)

__wkupsramfunc void ddr_save_restore_exit_sequence(struct emif_handle_s * h) {

    uint32_t lp_status;

    // Restore the default values from the reg_config file
    sdram_region_idx_cfg(h); // Programming the region_idx and sdram_idx fields for address mapping

    // Write back the copied registers
    restore_ddr_registers(h);

    // PHY_SET_DFI_INPUT_3:RW_D:24:4:=0x00 PHY_SET_DFI_INPUT_2:RW_D:16:4:=0x00 PHY_SET_DFI_INPUT_1:RW_D:8:4:=0x00 PHY_SET_DFI_INPUT_0:RW_D:0:4:=0x00
    Write_MMR_Field(h->ctl_cfg_base_addr + CSL_EMIF_CTLCFG_DENALI_PHY_1306, 0x1, 1, 0);

    // PI_INIT_LVL_EN:RW:0:1:=0x00
    Write_MMR_Field(h->ctl_cfg_base_addr + CSL_EMIF_CTLCFG_DENALI_PI_4, 0x0, 1, 0);

    // PHY_INDEP_TRAIN_MODE:RW:24:1:=0x01
    Write_MMR_Field(h->ctl_cfg_base_addr + CSL_EMIF_CTLCFG_DENALI_CTL_20, 0x1, 1, 24);

    // PHY_INDEP_INIT_MODE:RW:8:1:=0x01
    Write_MMR_Field(h->ctl_cfg_base_addr + CSL_EMIF_CTLCFG_DENALI_CTL_21, 0x1, 1, 8);

    // PI_DRAM_INIT_EN:RW:8:1:=0x1 PI_DLL_RST:RW:0:1:=0x1
    Write_MMR_Field(h->ctl_cfg_base_addr + CSL_EMIF_CTLCFG_DENALI_PI_138, 0x1, 1, 0);

    // PWRUP_SREFRESH_EXIT:RW:0:0:=0x00
    Write_MMR_Field(h->ctl_cfg_base_addr + CSL_EMIF_CTLCFG_DENALI_CTL_106, 0x0, 1, 0);

    // PI_PWRUP_SREFRESH_EXIT:RW+:8:1:=0x01
    Write_MMR_Field(h->ctl_cfg_base_addr + CSL_EMIF_CTLCFG_DENALI_PI_134, 0x1, 1, 8);

    // PI_DRAM_INIT_EN=1
    Write_MMR_Field(h->ctl_cfg_base_addr + CSL_EMIF_CTLCFG_DENALI_PI_138, 0x1, 1, 8);

    // De-asserting data retention pin and wake Control bits
    Write_MMR_Field((WKUP_CTRL_MMR_SEC_4_BASE + CSL_WKUP_CTRL_MMR_CFG4_DDR32SS_PMCTRL), 0x0U, 1U, 31U);
    Write_MMR_Field((WKUP_CTRL_MMR_SEC_4_BASE + CSL_WKUP_CTRL_MMR_CFG4_DDR32SS_PMCTRL), 0x0U, 4U, 0U);
    Write_MMR_Field((WKUP_CTRL_MMR_SEC_4_BASE + CSL_WKUP_CTRL_MMR_CFG4_DDR32SS_PMCTRL), 0x1U, 1U, 31U);
    Write_MMR_Field((WKUP_CTRL_MMR_SEC_4_BASE + CSL_WKUP_CTRL_MMR_CFG4_DDR32SS_PMCTRL), 0x0U, 4U, 0U);
    lp_status = mmio_read_32((WKUP_CTRL_MMR_SEC_4_BASE + CSL_WKUP_CTRL_MMR_CFG4_DDR32SS_PMCTRL));
    while(lp_status != ((1U<<31U))){
        lp_status = mmio_read_32((WKUP_CTRL_MMR_SEC_4_BASE + CSL_WKUP_CTRL_MMR_CFG4_DDR32SS_PMCTRL));
    }
    Write_MMR_Field((WKUP_CTRL_MMR_SEC_4_BASE + CSL_WKUP_CTRL_MMR_CFG4_DDR32SS_PMCTRL), 0x0, 1, 31);

    // Start Initialization [PI_START=1 and START=1]
    start_PI_CTL_init(h);

    // Wait for INIT_DONE interrupt
    poll_for_init_completion(h);

    /* dfi_phymstr_cs_state_r = 0, dfi_phymstr_state_sel_r = 0, and PI_SELF_REFRESH_EN = 1 to enable self-refresh
     * during training since PI does not send refresh commands during CA leveling
     */
    Write_MMR_Field(h->ctl_cfg_base_addr + CSL_EMIF_CTLCFG_DENALI_PI_6, 0x1, 1, 0); //PI_DFI_PHYMSTR_CS_STATE_R
    Write_MMR_Field(h->ctl_cfg_base_addr + CSL_EMIF_CTLCFG_DENALI_PI_6, 0x1, 1, 8); //PI_DFI_PHYMSTR_STATE_SEL_R
    Write_MMR_Field(h->ctl_cfg_base_addr + CSL_EMIF_CTLCFG_DENALI_PI_133, 0x1, 1, 24); //PI_SELF_REFRESH_EN

    // LPDDR4 PI sequence
    Write_MMR_Field(h->ctl_cfg_base_addr + CSL_EMIF_CTLCFG_DENALI_PI_23, 0x1, 1, 24); //PI_WRLVL_REQ
    Write_MMR_Field(h->ctl_cfg_base_addr + CSL_EMIF_CTLCFG_DENALI_PI_33, 0x1, 1, 24); //PI_RDLVL_GATE_REQ
    Write_MMR_Field(h->ctl_cfg_base_addr + CSL_EMIF_CTLCFG_DENALI_PI_33, 0x1, 1, 16); //PI_RDLVL_REQ
    Write_MMR_Field(h->ctl_cfg_base_addr + CSL_EMIF_CTLCFG_DENALI_PI_67, 0x1, 1, 8); //PI_WDQLVL_REQ
    Write_MMR_Field(h->ctl_cfg_base_addr + CSL_EMIF_CTLCFG_DENALI_PI_181, 0x0, 2, 16); //PI_WRLVL_EN_F0
    Write_MMR_Field(h->ctl_cfg_base_addr + CSL_EMIF_CTLCFG_DENALI_PI_181, 0x1, 2, 24); //PI_WRLVL_EN_F1
    Write_MMR_Field(h->ctl_cfg_base_addr + CSL_EMIF_CTLCFG_DENALI_PI_182, 0x1, 2, 0); //PI_WRLVL_EN_F2
    Write_MMR_Field(h->ctl_cfg_base_addr + CSL_EMIF_CTLCFG_DENALI_PI_188, 0x0, 2, 24); //PI_RDLVL_GATE_EN_F0
    Write_MMR_Field(h->ctl_cfg_base_addr + CSL_EMIF_CTLCFG_DENALI_PI_189, 0x0, 2, 8); //PI_RDLVL_GATE_EN_F1
    Write_MMR_Field(h->ctl_cfg_base_addr + CSL_EMIF_CTLCFG_DENALI_PI_189, 0x1, 2, 24); //PI_RDLVL_GATE_EN_F2
    Write_MMR_Field(h->ctl_cfg_base_addr + CSL_EMIF_CTLCFG_DENALI_PI_188, 0x0, 2, 16); //PI_RDLVL_EN_F0
    Write_MMR_Field(h->ctl_cfg_base_addr + CSL_EMIF_CTLCFG_DENALI_PI_189, 0x0, 2, 0); //PI_RDLVL_EN_F1
    Write_MMR_Field(h->ctl_cfg_base_addr + CSL_EMIF_CTLCFG_DENALI_PI_189, 0x1, 2, 16); //PI_RDLVL_EN_F2
    Write_MMR_Field(h->ctl_cfg_base_addr + CSL_EMIF_CTLCFG_DENALI_PI_190, 0x0, 2, 24); //PI_RDLVL_PAT0_EN_F0
    Write_MMR_Field(h->ctl_cfg_base_addr + CSL_EMIF_CTLCFG_DENALI_PI_191, 0x0, 2, 24); //PI_RDLVL_PAT0_EN_F1
    Write_MMR_Field(h->ctl_cfg_base_addr + CSL_EMIF_CTLCFG_DENALI_PI_192, 0x0, 2, 24); //PI_RDLVL_PAT0_EN_F2
    Write_MMR_Field(h->ctl_cfg_base_addr + CSL_EMIF_CTLCFG_DENALI_PI_191, 0x0, 2, 16); //PI_RDLVL_MULTI_EN_F0
    Write_MMR_Field(h->ctl_cfg_base_addr + CSL_EMIF_CTLCFG_DENALI_PI_192, 0x0, 2, 16); //PI_RDLVL_MULTI_EN_F1
    Write_MMR_Field(h->ctl_cfg_base_addr + CSL_EMIF_CTLCFG_DENALI_PI_193, 0x0, 2, 16); //PI_RDLVL_MULTI_EN_F1
    Write_MMR_Field(h->ctl_cfg_base_addr + CSL_EMIF_CTLCFG_DENALI_PI_191, 0x0, 2, 8); //PI_RDLVL_MULTI_EN_F0
    Write_MMR_Field(h->ctl_cfg_base_addr + CSL_EMIF_CTLCFG_DENALI_PI_192, 0x0, 2, 8); //PI_RDLVL_MULTI_EN_F1
    Write_MMR_Field(h->ctl_cfg_base_addr + CSL_EMIF_CTLCFG_DENALI_PI_193, 0x0, 2, 8); //PI_RDLVL_MULTI_EN_F2
    Write_MMR_Field(h->ctl_cfg_base_addr + CSL_EMIF_CTLCFG_DENALI_PI_191, 0x0, 2, 0); //PI_RDLVL_RXCAL_EN_F0
    Write_MMR_Field(h->ctl_cfg_base_addr + CSL_EMIF_CTLCFG_DENALI_PI_192, 0x0, 2, 0); //PI_RDLVL_RXCAL_EN_F1
    Write_MMR_Field(h->ctl_cfg_base_addr + CSL_EMIF_CTLCFG_DENALI_PI_193, 0x0, 2, 0); //PI_RDLVL_RXCAL_EN_F2
    Write_MMR_Field(h->ctl_cfg_base_addr + CSL_EMIF_CTLCFG_DENALI_PI_199, 0x0, 2, 0); //PI_CALVL_EN_F0
    Write_MMR_Field(h->ctl_cfg_base_addr + CSL_EMIF_CTLCFG_DENALI_PI_199, 0x0, 2, 8); //PI_CALVL_EN_F1
    Write_MMR_Field(h->ctl_cfg_base_addr + CSL_EMIF_CTLCFG_DENALI_PI_199, 0x0, 2, 16); //PI_CALVL_EN_F2
    Write_MMR_Field(h->ctl_cfg_base_addr + CSL_EMIF_CTLCFG_DENALI_PI_223, 0x0, 2, 8); //PI_WDQLVL_EN_F0
    Write_MMR_Field(h->ctl_cfg_base_addr + CSL_EMIF_CTLCFG_DENALI_PI_226, 0x0, 2, 8); //PI_WDQLVL_EN_F1
    Write_MMR_Field(h->ctl_cfg_base_addr + CSL_EMIF_CTLCFG_DENALI_PI_229, 0x1, 2, 8); //PI_WDQLVL_EN_F2
}

__wkupsramfunc void enter_lpm_self_refresh(struct emif_handle_s * h) {
    uint32_t lp_status = 0;
    // Program Self Refresh mode
    mmio_write_32(h->ctl_cfg_base_addr + DENALI_CTL_158__SFR_OFFS, (LP_MODE_LONG_SELF_REFRESH << 8));

    // Poll for Self Refresh Mode change
    while(lp_status != 0x4E) {
        lp_status = ((mmio_read_32(h->ctl_cfg_base_addr + DENALI_CTL_167__SFR_OFFS) & 0x7F00) >> 8);
    }
}

__wkupsramfunc int32_t save_ddr_reg_configs(void){

	uint32_t lp_status;
    // Save DDR register context in WKUP SRAM, Put the DDR in self refresh
    emif_instance_select(&Emifhandle);
    save_ddr_registers(&Emifhandle);
    enter_lpm_self_refresh(&Emifhandle);
    // Enable DDR data retention by writing b0110 to WKUP_CTRL_MMR. DDR32SS_PMCTRL.data_retention
    Write_MMR_Field((WKUP_CTRL_MMR_SEC_4_BASE + CSL_WKUP_CTRL_MMR_CFG4_DDR32SS_PMCTRL), 0x6U, 4U, 0U);
    Write_MMR_Field((WKUP_CTRL_MMR_SEC_4_BASE + CSL_WKUP_CTRL_MMR_CFG4_DDR32SS_PMCTRL), 0x1U, 1U, 31U);
    lp_status = mmio_read_32((WKUP_CTRL_MMR_SEC_4_BASE + CSL_WKUP_CTRL_MMR_CFG4_DDR32SS_PMCTRL));
    while(lp_status != ((1U<<31U) | 0x6U)){
        lp_status = mmio_read_32((WKUP_CTRL_MMR_SEC_4_BASE + CSL_WKUP_CTRL_MMR_CFG4_DDR32SS_PMCTRL));
    }
    Write_MMR_Field((WKUP_CTRL_MMR_SEC_4_BASE + CSL_WKUP_CTRL_MMR_CFG4_DDR32SS_PMCTRL), 0x0, 1, 31);

	return 0;
}

__wkupsramfunc int32_t restore_ddr_reg_configs(void){

	// Restore DDR Controller Context & Take DDR out of self refresh, retaining of DDR & Remove DDR data retention
    emif_instance_select(&Emifhandle);
    ddr_save_restore_exit_sequence(&Emifhandle);

	return 0;
}
