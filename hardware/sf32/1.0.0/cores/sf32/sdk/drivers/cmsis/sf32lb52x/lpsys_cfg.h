#ifndef __LPSYS_CFG_H
#define __LPSYS_CFG_H

typedef struct
{
    __IO uint32_t SYSCR;
    __IO uint32_t RTC_TR;
    __IO uint32_t RTC_DR;
    __IO uint32_t ULPMCR;
    __IO uint32_t DBGR;
    __IO uint32_t MDBGR;
    __IO uint32_t BISTCR;
    __IO uint32_t BISTR;
    __IO uint32_t ROMCR0;
    __IO uint32_t ROMCR1;
    __IO uint32_t ROMCR2;
    __IO uint32_t ROMCR3;
    __IO uint32_t ROMCR4;
    __IO uint32_t ROMCR5;
    __IO uint32_t USART4_PINR;
    __IO uint32_t USART5_PINR;
    __IO uint32_t PTA_PINR;
} LPSYS_CFG_TypeDef;


/**************** Bit definition for LPSYS_CFG_SYSCR register *****************/
#define LPSYS_CFG_SYSCR_WDT2_REBOOT_Pos  (0U)
#define LPSYS_CFG_SYSCR_WDT2_REBOOT_Msk  (0x1UL << LPSYS_CFG_SYSCR_WDT2_REBOOT_Pos)
#define LPSYS_CFG_SYSCR_WDT2_REBOOT     LPSYS_CFG_SYSCR_WDT2_REBOOT_Msk
#define LPSYS_CFG_SYSCR_DBG_SWAP_Pos    (1U)
#define LPSYS_CFG_SYSCR_DBG_SWAP_Msk    (0x3UL << LPSYS_CFG_SYSCR_DBG_SWAP_Pos)
#define LPSYS_CFG_SYSCR_DBG_SWAP        LPSYS_CFG_SYSCR_DBG_SWAP_Msk
#define LPSYS_CFG_SYSCR_LDO_VSEL_Pos    (3U)
#define LPSYS_CFG_SYSCR_LDO_VSEL_Msk    (0x1UL << LPSYS_CFG_SYSCR_LDO_VSEL_Pos)
#define LPSYS_CFG_SYSCR_LDO_VSEL        LPSYS_CFG_SYSCR_LDO_VSEL_Msk

/**************** Bit definition for LPSYS_CFG_RTC_TR register ****************/
#define LPSYS_CFG_RTC_TR_SS_Pos         (0U)
#define LPSYS_CFG_RTC_TR_SS_Msk         (0x3FFUL << LPSYS_CFG_RTC_TR_SS_Pos)
#define LPSYS_CFG_RTC_TR_SS             LPSYS_CFG_RTC_TR_SS_Msk
#define LPSYS_CFG_RTC_TR_SU_Pos         (11U)
#define LPSYS_CFG_RTC_TR_SU_Msk         (0xFUL << LPSYS_CFG_RTC_TR_SU_Pos)
#define LPSYS_CFG_RTC_TR_SU             LPSYS_CFG_RTC_TR_SU_Msk
#define LPSYS_CFG_RTC_TR_ST_Pos         (15U)
#define LPSYS_CFG_RTC_TR_ST_Msk         (0x7UL << LPSYS_CFG_RTC_TR_ST_Pos)
#define LPSYS_CFG_RTC_TR_ST             LPSYS_CFG_RTC_TR_ST_Msk
#define LPSYS_CFG_RTC_TR_MNU_Pos        (18U)
#define LPSYS_CFG_RTC_TR_MNU_Msk        (0xFUL << LPSYS_CFG_RTC_TR_MNU_Pos)
#define LPSYS_CFG_RTC_TR_MNU            LPSYS_CFG_RTC_TR_MNU_Msk
#define LPSYS_CFG_RTC_TR_MNT_Pos        (22U)
#define LPSYS_CFG_RTC_TR_MNT_Msk        (0x7UL << LPSYS_CFG_RTC_TR_MNT_Pos)
#define LPSYS_CFG_RTC_TR_MNT            LPSYS_CFG_RTC_TR_MNT_Msk
#define LPSYS_CFG_RTC_TR_HU_Pos         (25U)
#define LPSYS_CFG_RTC_TR_HU_Msk         (0xFUL << LPSYS_CFG_RTC_TR_HU_Pos)
#define LPSYS_CFG_RTC_TR_HU             LPSYS_CFG_RTC_TR_HU_Msk
#define LPSYS_CFG_RTC_TR_HT_Pos         (29U)
#define LPSYS_CFG_RTC_TR_HT_Msk         (0x3UL << LPSYS_CFG_RTC_TR_HT_Pos)
#define LPSYS_CFG_RTC_TR_HT             LPSYS_CFG_RTC_TR_HT_Msk
#define LPSYS_CFG_RTC_TR_PM_Pos         (31U)
#define LPSYS_CFG_RTC_TR_PM_Msk         (0x1UL << LPSYS_CFG_RTC_TR_PM_Pos)
#define LPSYS_CFG_RTC_TR_PM             LPSYS_CFG_RTC_TR_PM_Msk

/**************** Bit definition for LPSYS_CFG_RTC_DR register ****************/
#define LPSYS_CFG_RTC_DR_DU_Pos         (0U)
#define LPSYS_CFG_RTC_DR_DU_Msk         (0xFUL << LPSYS_CFG_RTC_DR_DU_Pos)
#define LPSYS_CFG_RTC_DR_DU             LPSYS_CFG_RTC_DR_DU_Msk
#define LPSYS_CFG_RTC_DR_DT_Pos         (4U)
#define LPSYS_CFG_RTC_DR_DT_Msk         (0x3UL << LPSYS_CFG_RTC_DR_DT_Pos)
#define LPSYS_CFG_RTC_DR_DT             LPSYS_CFG_RTC_DR_DT_Msk
#define LPSYS_CFG_RTC_DR_MU_Pos         (8U)
#define LPSYS_CFG_RTC_DR_MU_Msk         (0xFUL << LPSYS_CFG_RTC_DR_MU_Pos)
#define LPSYS_CFG_RTC_DR_MU             LPSYS_CFG_RTC_DR_MU_Msk
#define LPSYS_CFG_RTC_DR_MT_Pos         (12U)
#define LPSYS_CFG_RTC_DR_MT_Msk         (0x1UL << LPSYS_CFG_RTC_DR_MT_Pos)
#define LPSYS_CFG_RTC_DR_MT             LPSYS_CFG_RTC_DR_MT_Msk
#define LPSYS_CFG_RTC_DR_WD_Pos         (13U)
#define LPSYS_CFG_RTC_DR_WD_Msk         (0x7UL << LPSYS_CFG_RTC_DR_WD_Pos)
#define LPSYS_CFG_RTC_DR_WD             LPSYS_CFG_RTC_DR_WD_Msk
#define LPSYS_CFG_RTC_DR_YU_Pos         (16U)
#define LPSYS_CFG_RTC_DR_YU_Msk         (0xFUL << LPSYS_CFG_RTC_DR_YU_Pos)
#define LPSYS_CFG_RTC_DR_YU             LPSYS_CFG_RTC_DR_YU_Msk
#define LPSYS_CFG_RTC_DR_YT_Pos         (20U)
#define LPSYS_CFG_RTC_DR_YT_Msk         (0xFUL << LPSYS_CFG_RTC_DR_YT_Pos)
#define LPSYS_CFG_RTC_DR_YT             LPSYS_CFG_RTC_DR_YT_Msk
#define LPSYS_CFG_RTC_DR_CB_Pos         (24U)
#define LPSYS_CFG_RTC_DR_CB_Msk         (0x1UL << LPSYS_CFG_RTC_DR_CB_Pos)
#define LPSYS_CFG_RTC_DR_CB             LPSYS_CFG_RTC_DR_CB_Msk
#define LPSYS_CFG_RTC_DR_ERR_Pos        (31U)
#define LPSYS_CFG_RTC_DR_ERR_Msk        (0x1UL << LPSYS_CFG_RTC_DR_ERR_Pos)
#define LPSYS_CFG_RTC_DR_ERR            LPSYS_CFG_RTC_DR_ERR_Msk

/**************** Bit definition for LPSYS_CFG_ULPMCR register ****************/
#define LPSYS_CFG_ULPMCR_RAM_RM_Pos     (0U)
#define LPSYS_CFG_ULPMCR_RAM_RM_Msk     (0x3UL << LPSYS_CFG_ULPMCR_RAM_RM_Pos)
#define LPSYS_CFG_ULPMCR_RAM_RM         LPSYS_CFG_ULPMCR_RAM_RM_Msk
#define LPSYS_CFG_ULPMCR_RAM_RME_Pos    (4U)
#define LPSYS_CFG_ULPMCR_RAM_RME_Msk    (0x1UL << LPSYS_CFG_ULPMCR_RAM_RME_Pos)
#define LPSYS_CFG_ULPMCR_RAM_RME        LPSYS_CFG_ULPMCR_RAM_RME_Msk
#define LPSYS_CFG_ULPMCR_RAM_RA_Pos     (5U)
#define LPSYS_CFG_ULPMCR_RAM_RA_Msk     (0x3UL << LPSYS_CFG_ULPMCR_RAM_RA_Pos)
#define LPSYS_CFG_ULPMCR_RAM_RA         LPSYS_CFG_ULPMCR_RAM_RA_Msk
#define LPSYS_CFG_ULPMCR_RAM_WA_Pos     (7U)
#define LPSYS_CFG_ULPMCR_RAM_WA_Msk     (0x7UL << LPSYS_CFG_ULPMCR_RAM_WA_Pos)
#define LPSYS_CFG_ULPMCR_RAM_WA         LPSYS_CFG_ULPMCR_RAM_WA_Msk
#define LPSYS_CFG_ULPMCR_RAM_WPULSE_Pos  (10U)
#define LPSYS_CFG_ULPMCR_RAM_WPULSE_Msk  (0x7UL << LPSYS_CFG_ULPMCR_RAM_WPULSE_Pos)
#define LPSYS_CFG_ULPMCR_RAM_WPULSE     LPSYS_CFG_ULPMCR_RAM_WPULSE_Msk
#define LPSYS_CFG_ULPMCR_RAM_RDY_Pos    (15U)
#define LPSYS_CFG_ULPMCR_RAM_RDY_Msk    (0x1UL << LPSYS_CFG_ULPMCR_RAM_RDY_Pos)
#define LPSYS_CFG_ULPMCR_RAM_RDY        LPSYS_CFG_ULPMCR_RAM_RDY_Msk
#define LPSYS_CFG_ULPMCR_ROM_RM_Pos     (16U)
#define LPSYS_CFG_ULPMCR_ROM_RM_Msk     (0x3UL << LPSYS_CFG_ULPMCR_ROM_RM_Pos)
#define LPSYS_CFG_ULPMCR_ROM_RM         LPSYS_CFG_ULPMCR_ROM_RM_Msk
#define LPSYS_CFG_ULPMCR_ROM_RME_Pos    (20U)
#define LPSYS_CFG_ULPMCR_ROM_RME_Msk    (0x1UL << LPSYS_CFG_ULPMCR_ROM_RME_Pos)
#define LPSYS_CFG_ULPMCR_ROM_RME        LPSYS_CFG_ULPMCR_ROM_RME_Msk

/***************** Bit definition for LPSYS_CFG_DBGR register *****************/
#define LPSYS_CFG_DBGR_LP2HP_NMI_Pos    (28U)
#define LPSYS_CFG_DBGR_LP2HP_NMI_Msk    (0x1UL << LPSYS_CFG_DBGR_LP2HP_NMI_Pos)
#define LPSYS_CFG_DBGR_LP2HP_NMI        LPSYS_CFG_DBGR_LP2HP_NMI_Msk
#define LPSYS_CFG_DBGR_HP2LP_NMIE_Pos   (29U)
#define LPSYS_CFG_DBGR_HP2LP_NMIE_Msk   (0x1UL << LPSYS_CFG_DBGR_HP2LP_NMIE_Pos)
#define LPSYS_CFG_DBGR_HP2LP_NMIE       LPSYS_CFG_DBGR_HP2LP_NMIE_Msk
#define LPSYS_CFG_DBGR_HP2LP_NMIF_Pos   (30U)
#define LPSYS_CFG_DBGR_HP2LP_NMIF_Msk   (0x1UL << LPSYS_CFG_DBGR_HP2LP_NMIF_Pos)
#define LPSYS_CFG_DBGR_HP2LP_NMIF       LPSYS_CFG_DBGR_HP2LP_NMIF_Msk
#define LPSYS_CFG_DBGR_READY_Pos        (31U)
#define LPSYS_CFG_DBGR_READY_Msk        (0x1UL << LPSYS_CFG_DBGR_READY_Pos)
#define LPSYS_CFG_DBGR_READY            LPSYS_CFG_DBGR_READY_Msk

/**************** Bit definition for LPSYS_CFG_MDBGR register *****************/
#define LPSYS_CFG_MDBGR_LS_RAM0_Pos     (0U)
#define LPSYS_CFG_MDBGR_LS_RAM0_Msk     (0x1UL << LPSYS_CFG_MDBGR_LS_RAM0_Pos)
#define LPSYS_CFG_MDBGR_LS_RAM0         LPSYS_CFG_MDBGR_LS_RAM0_Msk
#define LPSYS_CFG_MDBGR_LS_RAM1_Pos     (1U)
#define LPSYS_CFG_MDBGR_LS_RAM1_Msk     (0x1UL << LPSYS_CFG_MDBGR_LS_RAM1_Pos)
#define LPSYS_CFG_MDBGR_LS_RAM1         LPSYS_CFG_MDBGR_LS_RAM1_Msk
#define LPSYS_CFG_MDBGR_LS_ROM_Pos      (2U)
#define LPSYS_CFG_MDBGR_LS_ROM_Msk      (0x1UL << LPSYS_CFG_MDBGR_LS_ROM_Pos)
#define LPSYS_CFG_MDBGR_LS_ROM          LPSYS_CFG_MDBGR_LS_ROM_Msk
#define LPSYS_CFG_MDBGR_PD_ROM_Pos      (3U)
#define LPSYS_CFG_MDBGR_PD_ROM_Msk      (0x1UL << LPSYS_CFG_MDBGR_PD_ROM_Pos)
#define LPSYS_CFG_MDBGR_PD_ROM          LPSYS_CFG_MDBGR_PD_ROM_Msk

/**************** Bit definition for LPSYS_CFG_BISTCR register ****************/
#define LPSYS_CFG_BISTCR_BIST_MODE_Pos  (0U)
#define LPSYS_CFG_BISTCR_BIST_MODE_Msk  (0x1UL << LPSYS_CFG_BISTCR_BIST_MODE_Pos)
#define LPSYS_CFG_BISTCR_BIST_MODE      LPSYS_CFG_BISTCR_BIST_MODE_Msk
#define LPSYS_CFG_BISTCR_BIST_DONE_Pos  (1U)
#define LPSYS_CFG_BISTCR_BIST_DONE_Msk  (0x1UL << LPSYS_CFG_BISTCR_BIST_DONE_Pos)
#define LPSYS_CFG_BISTCR_BIST_DONE      LPSYS_CFG_BISTCR_BIST_DONE_Msk
#define LPSYS_CFG_BISTCR_BIST_FAIL_Pos  (2U)
#define LPSYS_CFG_BISTCR_BIST_FAIL_Msk  (0x1UL << LPSYS_CFG_BISTCR_BIST_FAIL_Pos)
#define LPSYS_CFG_BISTCR_BIST_FAIL      LPSYS_CFG_BISTCR_BIST_FAIL_Msk

/**************** Bit definition for LPSYS_CFG_BISTR register *****************/
#define LPSYS_CFG_BISTR_BIST_FAIL_ROM_Pos  (0U)
#define LPSYS_CFG_BISTR_BIST_FAIL_ROM_Msk  (0x3FUL << LPSYS_CFG_BISTR_BIST_FAIL_ROM_Pos)
#define LPSYS_CFG_BISTR_BIST_FAIL_ROM   LPSYS_CFG_BISTR_BIST_FAIL_ROM_Msk
#define LPSYS_CFG_BISTR_BIST_FAIL_RAM_Pos  (6U)
#define LPSYS_CFG_BISTR_BIST_FAIL_RAM_Msk  (0x3UL << LPSYS_CFG_BISTR_BIST_FAIL_RAM_Pos)
#define LPSYS_CFG_BISTR_BIST_FAIL_RAM   LPSYS_CFG_BISTR_BIST_FAIL_RAM_Msk
#define LPSYS_CFG_BISTR_BIST_FAIL_RFC_Pos  (8U)
#define LPSYS_CFG_BISTR_BIST_FAIL_RFC_Msk  (0x1UL << LPSYS_CFG_BISTR_BIST_FAIL_RFC_Pos)
#define LPSYS_CFG_BISTR_BIST_FAIL_RFC   LPSYS_CFG_BISTR_BIST_FAIL_RFC_Msk

/**************** Bit definition for LPSYS_CFG_ROMCR0 register ****************/
#define LPSYS_CFG_ROMCR0_CMP_Pos        (0U)
#define LPSYS_CFG_ROMCR0_CMP_Msk        (0xFFFFFFFFUL << LPSYS_CFG_ROMCR0_CMP_Pos)
#define LPSYS_CFG_ROMCR0_CMP            LPSYS_CFG_ROMCR0_CMP_Msk

/**************** Bit definition for LPSYS_CFG_ROMCR1 register ****************/
#define LPSYS_CFG_ROMCR1_CMP_Pos        (0U)
#define LPSYS_CFG_ROMCR1_CMP_Msk        (0xFFFFFFFFUL << LPSYS_CFG_ROMCR1_CMP_Pos)
#define LPSYS_CFG_ROMCR1_CMP            LPSYS_CFG_ROMCR1_CMP_Msk

/**************** Bit definition for LPSYS_CFG_ROMCR2 register ****************/
#define LPSYS_CFG_ROMCR2_CMP_Pos        (0U)
#define LPSYS_CFG_ROMCR2_CMP_Msk        (0xFFFFFFFFUL << LPSYS_CFG_ROMCR2_CMP_Pos)
#define LPSYS_CFG_ROMCR2_CMP            LPSYS_CFG_ROMCR2_CMP_Msk

/**************** Bit definition for LPSYS_CFG_ROMCR3 register ****************/
#define LPSYS_CFG_ROMCR3_CMP_Pos        (0U)
#define LPSYS_CFG_ROMCR3_CMP_Msk        (0xFFFFFFFFUL << LPSYS_CFG_ROMCR3_CMP_Pos)
#define LPSYS_CFG_ROMCR3_CMP            LPSYS_CFG_ROMCR3_CMP_Msk

/**************** Bit definition for LPSYS_CFG_ROMCR4 register ****************/
#define LPSYS_CFG_ROMCR4_CMP_Pos        (0U)
#define LPSYS_CFG_ROMCR4_CMP_Msk        (0xFFFFFFFFUL << LPSYS_CFG_ROMCR4_CMP_Pos)
#define LPSYS_CFG_ROMCR4_CMP            LPSYS_CFG_ROMCR4_CMP_Msk

/**************** Bit definition for LPSYS_CFG_ROMCR5 register ****************/
#define LPSYS_CFG_ROMCR5_CMP_Pos        (0U)
#define LPSYS_CFG_ROMCR5_CMP_Msk        (0xFFFFFFFFUL << LPSYS_CFG_ROMCR5_CMP_Pos)
#define LPSYS_CFG_ROMCR5_CMP            LPSYS_CFG_ROMCR5_CMP_Msk

/************* Bit definition for LPSYS_CFG_USART4_PINR register **************/
#define LPSYS_CFG_USART4_PINR_TXD_PIN_Pos  (0U)
#define LPSYS_CFG_USART4_PINR_TXD_PIN_Msk  (0x7UL << LPSYS_CFG_USART4_PINR_TXD_PIN_Pos)
#define LPSYS_CFG_USART4_PINR_TXD_PIN   LPSYS_CFG_USART4_PINR_TXD_PIN_Msk
#define LPSYS_CFG_USART4_PINR_RXD_PIN_Pos  (8U)
#define LPSYS_CFG_USART4_PINR_RXD_PIN_Msk  (0x7UL << LPSYS_CFG_USART4_PINR_RXD_PIN_Pos)
#define LPSYS_CFG_USART4_PINR_RXD_PIN   LPSYS_CFG_USART4_PINR_RXD_PIN_Msk
#define LPSYS_CFG_USART4_PINR_RTS_PIN_Pos  (16U)
#define LPSYS_CFG_USART4_PINR_RTS_PIN_Msk  (0x7UL << LPSYS_CFG_USART4_PINR_RTS_PIN_Pos)
#define LPSYS_CFG_USART4_PINR_RTS_PIN   LPSYS_CFG_USART4_PINR_RTS_PIN_Msk
#define LPSYS_CFG_USART4_PINR_CTS_PIN_Pos  (24U)
#define LPSYS_CFG_USART4_PINR_CTS_PIN_Msk  (0x7UL << LPSYS_CFG_USART4_PINR_CTS_PIN_Pos)
#define LPSYS_CFG_USART4_PINR_CTS_PIN   LPSYS_CFG_USART4_PINR_CTS_PIN_Msk

/************* Bit definition for LPSYS_CFG_USART5_PINR register **************/
#define LPSYS_CFG_USART5_PINR_TXD_PIN_Pos  (0U)
#define LPSYS_CFG_USART5_PINR_TXD_PIN_Msk  (0x7UL << LPSYS_CFG_USART5_PINR_TXD_PIN_Pos)
#define LPSYS_CFG_USART5_PINR_TXD_PIN   LPSYS_CFG_USART5_PINR_TXD_PIN_Msk
#define LPSYS_CFG_USART5_PINR_RXD_PIN_Pos  (8U)
#define LPSYS_CFG_USART5_PINR_RXD_PIN_Msk  (0x7UL << LPSYS_CFG_USART5_PINR_RXD_PIN_Pos)
#define LPSYS_CFG_USART5_PINR_RXD_PIN   LPSYS_CFG_USART5_PINR_RXD_PIN_Msk
#define LPSYS_CFG_USART5_PINR_RTS_PIN_Pos  (16U)
#define LPSYS_CFG_USART5_PINR_RTS_PIN_Msk  (0x7UL << LPSYS_CFG_USART5_PINR_RTS_PIN_Pos)
#define LPSYS_CFG_USART5_PINR_RTS_PIN   LPSYS_CFG_USART5_PINR_RTS_PIN_Msk
#define LPSYS_CFG_USART5_PINR_CTS_PIN_Pos  (24U)
#define LPSYS_CFG_USART5_PINR_CTS_PIN_Msk  (0x7UL << LPSYS_CFG_USART5_PINR_CTS_PIN_Pos)
#define LPSYS_CFG_USART5_PINR_CTS_PIN   LPSYS_CFG_USART5_PINR_CTS_PIN_Msk

/*************** Bit definition for LPSYS_CFG_PTA_PINR register ***************/
#define LPSYS_CFG_PTA_PINR_BT_ACTIVE_Pos  (0U)
#define LPSYS_CFG_PTA_PINR_BT_ACTIVE_Msk  (0x7UL << LPSYS_CFG_PTA_PINR_BT_ACTIVE_Pos)
#define LPSYS_CFG_PTA_PINR_BT_ACTIVE    LPSYS_CFG_PTA_PINR_BT_ACTIVE_Msk
#define LPSYS_CFG_PTA_PINR_BT_COLLISION_Pos  (8U)
#define LPSYS_CFG_PTA_PINR_BT_COLLISION_Msk  (0x7UL << LPSYS_CFG_PTA_PINR_BT_COLLISION_Pos)
#define LPSYS_CFG_PTA_PINR_BT_COLLISION  LPSYS_CFG_PTA_PINR_BT_COLLISION_Msk
#define LPSYS_CFG_PTA_PINR_BT_PRIORITY_Pos  (16U)
#define LPSYS_CFG_PTA_PINR_BT_PRIORITY_Msk  (0x7UL << LPSYS_CFG_PTA_PINR_BT_PRIORITY_Pos)
#define LPSYS_CFG_PTA_PINR_BT_PRIORITY  LPSYS_CFG_PTA_PINR_BT_PRIORITY_Msk
#define LPSYS_CFG_PTA_PINR_WLAN_ACTIVE_Pos  (24U)
#define LPSYS_CFG_PTA_PINR_WLAN_ACTIVE_Msk  (0x7UL << LPSYS_CFG_PTA_PINR_WLAN_ACTIVE_Pos)
#define LPSYS_CFG_PTA_PINR_WLAN_ACTIVE  LPSYS_CFG_PTA_PINR_WLAN_ACTIVE_Msk

#endif