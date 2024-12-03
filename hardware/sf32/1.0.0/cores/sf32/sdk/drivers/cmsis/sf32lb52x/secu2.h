#ifndef __SECU2_H
#define __SECU2_H

typedef struct
{
    __IO uint32_t SECU_CTRL;
    __IO uint32_t IRQ_STAT0;
    __IO uint32_t IRQ_STAT1;
    __IO uint32_t IRQ_CFG0;
    __IO uint32_t IRQ_CFG1;
    __IO uint32_t LPMST_ATTR_CFG;
    __IO uint32_t LPSLV_ATTR_CFG;
    __IO uint32_t RAM0_PRIV_CFG0;
    __IO uint32_t RAM0_PRIV_CFG1;
    __IO uint32_t RAM0_SEC_CFG0;
    __IO uint32_t RAM0_SEC_CFG1;
    __IO uint32_t RAM1_PRIV_CFG0;
    __IO uint32_t RAM1_PRIV_CFG1;
    __IO uint32_t RAM1_SEC_CFG0;
    __IO uint32_t RAM1_SEC_CFG1;
} SECU2_TypeDef;


/**************** Bit definition for SECU2_SECU_CTRL register *****************/
#define SECU2_SECU_CTRL_LPMST_LOCK_Pos  (0U)
#define SECU2_SECU_CTRL_LPMST_LOCK_Msk  (0x1UL << SECU2_SECU_CTRL_LPMST_LOCK_Pos)
#define SECU2_SECU_CTRL_LPMST_LOCK      SECU2_SECU_CTRL_LPMST_LOCK_Msk
#define SECU2_SECU_CTRL_LPSLV_LOCK_Pos  (1U)
#define SECU2_SECU_CTRL_LPSLV_LOCK_Msk  (0x1UL << SECU2_SECU_CTRL_LPSLV_LOCK_Pos)
#define SECU2_SECU_CTRL_LPSLV_LOCK      SECU2_SECU_CTRL_LPSLV_LOCK_Msk
#define SECU2_SECU_CTRL_RAM_LOCK_Pos    (2U)
#define SECU2_SECU_CTRL_RAM_LOCK_Msk    (0x1UL << SECU2_SECU_CTRL_RAM_LOCK_Pos)
#define SECU2_SECU_CTRL_RAM_LOCK        SECU2_SECU_CTRL_RAM_LOCK_Msk
#define SECU2_SECU_CTRL_LPMST_ATTR_UPDATE_Pos  (8U)
#define SECU2_SECU_CTRL_LPMST_ATTR_UPDATE_Msk  (0x1UL << SECU2_SECU_CTRL_LPMST_ATTR_UPDATE_Pos)
#define SECU2_SECU_CTRL_LPMST_ATTR_UPDATE  SECU2_SECU_CTRL_LPMST_ATTR_UPDATE_Msk

/**************** Bit definition for SECU2_IRQ_STAT0 register *****************/
#define SECU2_IRQ_STAT0_ILL_PTC2_PRIV_W_Pos  (0U)
#define SECU2_IRQ_STAT0_ILL_PTC2_PRIV_W_Msk  (0x1UL << SECU2_IRQ_STAT0_ILL_PTC2_PRIV_W_Pos)
#define SECU2_IRQ_STAT0_ILL_PTC2_PRIV_W  SECU2_IRQ_STAT0_ILL_PTC2_PRIV_W_Msk
#define SECU2_IRQ_STAT0_ILL_PTC2_PRIV_R_Pos  (1U)
#define SECU2_IRQ_STAT0_ILL_PTC2_PRIV_R_Msk  (0x1UL << SECU2_IRQ_STAT0_ILL_PTC2_PRIV_R_Pos)
#define SECU2_IRQ_STAT0_ILL_PTC2_PRIV_R  SECU2_IRQ_STAT0_ILL_PTC2_PRIV_R_Msk
#define SECU2_IRQ_STAT0_ILL_PTC2_SEC_W_Pos  (2U)
#define SECU2_IRQ_STAT0_ILL_PTC2_SEC_W_Msk  (0x1UL << SECU2_IRQ_STAT0_ILL_PTC2_SEC_W_Pos)
#define SECU2_IRQ_STAT0_ILL_PTC2_SEC_W  SECU2_IRQ_STAT0_ILL_PTC2_SEC_W_Msk
#define SECU2_IRQ_STAT0_ILL_PTC2_SEC_R_Pos  (3U)
#define SECU2_IRQ_STAT0_ILL_PTC2_SEC_R_Msk  (0x1UL << SECU2_IRQ_STAT0_ILL_PTC2_SEC_R_Pos)
#define SECU2_IRQ_STAT0_ILL_PTC2_SEC_R  SECU2_IRQ_STAT0_ILL_PTC2_SEC_R_Msk
#define SECU2_IRQ_STAT0_ILL_DMAC2_PRIV_W_Pos  (4U)
#define SECU2_IRQ_STAT0_ILL_DMAC2_PRIV_W_Msk  (0x1UL << SECU2_IRQ_STAT0_ILL_DMAC2_PRIV_W_Pos)
#define SECU2_IRQ_STAT0_ILL_DMAC2_PRIV_W  SECU2_IRQ_STAT0_ILL_DMAC2_PRIV_W_Msk
#define SECU2_IRQ_STAT0_ILL_DMAC2_PRIV_R_Pos  (5U)
#define SECU2_IRQ_STAT0_ILL_DMAC2_PRIV_R_Msk  (0x1UL << SECU2_IRQ_STAT0_ILL_DMAC2_PRIV_R_Pos)
#define SECU2_IRQ_STAT0_ILL_DMAC2_PRIV_R  SECU2_IRQ_STAT0_ILL_DMAC2_PRIV_R_Msk
#define SECU2_IRQ_STAT0_ILL_DMAC2_SEC_W_Pos  (6U)
#define SECU2_IRQ_STAT0_ILL_DMAC2_SEC_W_Msk  (0x1UL << SECU2_IRQ_STAT0_ILL_DMAC2_SEC_W_Pos)
#define SECU2_IRQ_STAT0_ILL_DMAC2_SEC_W  SECU2_IRQ_STAT0_ILL_DMAC2_SEC_W_Msk
#define SECU2_IRQ_STAT0_ILL_DMAC2_SEC_R_Pos  (7U)
#define SECU2_IRQ_STAT0_ILL_DMAC2_SEC_R_Msk  (0x1UL << SECU2_IRQ_STAT0_ILL_DMAC2_SEC_R_Pos)
#define SECU2_IRQ_STAT0_ILL_DMAC2_SEC_R  SECU2_IRQ_STAT0_ILL_DMAC2_SEC_R_Msk
#define SECU2_IRQ_STAT0_ILL_SECU2_PRIV_W_Pos  (8U)
#define SECU2_IRQ_STAT0_ILL_SECU2_PRIV_W_Msk  (0x1UL << SECU2_IRQ_STAT0_ILL_SECU2_PRIV_W_Pos)
#define SECU2_IRQ_STAT0_ILL_SECU2_PRIV_W  SECU2_IRQ_STAT0_ILL_SECU2_PRIV_W_Msk
#define SECU2_IRQ_STAT0_ILL_SECU2_PRIV_R_Pos  (9U)
#define SECU2_IRQ_STAT0_ILL_SECU2_PRIV_R_Msk  (0x1UL << SECU2_IRQ_STAT0_ILL_SECU2_PRIV_R_Pos)
#define SECU2_IRQ_STAT0_ILL_SECU2_PRIV_R  SECU2_IRQ_STAT0_ILL_SECU2_PRIV_R_Msk
#define SECU2_IRQ_STAT0_ILL_SECU2_SEC_W_Pos  (10U)
#define SECU2_IRQ_STAT0_ILL_SECU2_SEC_W_Msk  (0x1UL << SECU2_IRQ_STAT0_ILL_SECU2_SEC_W_Pos)
#define SECU2_IRQ_STAT0_ILL_SECU2_SEC_W  SECU2_IRQ_STAT0_ILL_SECU2_SEC_W_Msk
#define SECU2_IRQ_STAT0_ILL_SECU2_SEC_R_Pos  (11U)
#define SECU2_IRQ_STAT0_ILL_SECU2_SEC_R_Msk  (0x1UL << SECU2_IRQ_STAT0_ILL_SECU2_SEC_R_Pos)
#define SECU2_IRQ_STAT0_ILL_SECU2_SEC_R  SECU2_IRQ_STAT0_ILL_SECU2_SEC_R_Msk

/**************** Bit definition for SECU2_IRQ_STAT1 register *****************/
#define SECU2_IRQ_STAT1_ILL_RAM0_PRIV_W_Pos  (0U)
#define SECU2_IRQ_STAT1_ILL_RAM0_PRIV_W_Msk  (0x1UL << SECU2_IRQ_STAT1_ILL_RAM0_PRIV_W_Pos)
#define SECU2_IRQ_STAT1_ILL_RAM0_PRIV_W  SECU2_IRQ_STAT1_ILL_RAM0_PRIV_W_Msk
#define SECU2_IRQ_STAT1_ILL_RAM0_PRIV_R_Pos  (1U)
#define SECU2_IRQ_STAT1_ILL_RAM0_PRIV_R_Msk  (0x1UL << SECU2_IRQ_STAT1_ILL_RAM0_PRIV_R_Pos)
#define SECU2_IRQ_STAT1_ILL_RAM0_PRIV_R  SECU2_IRQ_STAT1_ILL_RAM0_PRIV_R_Msk
#define SECU2_IRQ_STAT1_ILL_RAM0_SEC_W_Pos  (2U)
#define SECU2_IRQ_STAT1_ILL_RAM0_SEC_W_Msk  (0x1UL << SECU2_IRQ_STAT1_ILL_RAM0_SEC_W_Pos)
#define SECU2_IRQ_STAT1_ILL_RAM0_SEC_W  SECU2_IRQ_STAT1_ILL_RAM0_SEC_W_Msk
#define SECU2_IRQ_STAT1_ILL_RAM0_SEC_R_Pos  (3U)
#define SECU2_IRQ_STAT1_ILL_RAM0_SEC_R_Msk  (0x1UL << SECU2_IRQ_STAT1_ILL_RAM0_SEC_R_Pos)
#define SECU2_IRQ_STAT1_ILL_RAM0_SEC_R  SECU2_IRQ_STAT1_ILL_RAM0_SEC_R_Msk
#define SECU2_IRQ_STAT1_ILL_RAM1_PRIV_W_Pos  (4U)
#define SECU2_IRQ_STAT1_ILL_RAM1_PRIV_W_Msk  (0x1UL << SECU2_IRQ_STAT1_ILL_RAM1_PRIV_W_Pos)
#define SECU2_IRQ_STAT1_ILL_RAM1_PRIV_W  SECU2_IRQ_STAT1_ILL_RAM1_PRIV_W_Msk
#define SECU2_IRQ_STAT1_ILL_RAM1_PRIV_R_Pos  (5U)
#define SECU2_IRQ_STAT1_ILL_RAM1_PRIV_R_Msk  (0x1UL << SECU2_IRQ_STAT1_ILL_RAM1_PRIV_R_Pos)
#define SECU2_IRQ_STAT1_ILL_RAM1_PRIV_R  SECU2_IRQ_STAT1_ILL_RAM1_PRIV_R_Msk
#define SECU2_IRQ_STAT1_ILL_RAM1_SEC_W_Pos  (6U)
#define SECU2_IRQ_STAT1_ILL_RAM1_SEC_W_Msk  (0x1UL << SECU2_IRQ_STAT1_ILL_RAM1_SEC_W_Pos)
#define SECU2_IRQ_STAT1_ILL_RAM1_SEC_W  SECU2_IRQ_STAT1_ILL_RAM1_SEC_W_Msk
#define SECU2_IRQ_STAT1_ILL_RAM1_SEC_R_Pos  (7U)
#define SECU2_IRQ_STAT1_ILL_RAM1_SEC_R_Msk  (0x1UL << SECU2_IRQ_STAT1_ILL_RAM1_SEC_R_Pos)
#define SECU2_IRQ_STAT1_ILL_RAM1_SEC_R  SECU2_IRQ_STAT1_ILL_RAM1_SEC_R_Msk

/***************** Bit definition for SECU2_IRQ_CFG0 register *****************/
#define SECU2_IRQ_CFG0_ILL_PTC2_PRIV_W_MASK_Pos  (0U)
#define SECU2_IRQ_CFG0_ILL_PTC2_PRIV_W_MASK_Msk  (0x1UL << SECU2_IRQ_CFG0_ILL_PTC2_PRIV_W_MASK_Pos)
#define SECU2_IRQ_CFG0_ILL_PTC2_PRIV_W_MASK  SECU2_IRQ_CFG0_ILL_PTC2_PRIV_W_MASK_Msk
#define SECU2_IRQ_CFG0_ILL_PTC2_PRIV_R_MASK_Pos  (1U)
#define SECU2_IRQ_CFG0_ILL_PTC2_PRIV_R_MASK_Msk  (0x1UL << SECU2_IRQ_CFG0_ILL_PTC2_PRIV_R_MASK_Pos)
#define SECU2_IRQ_CFG0_ILL_PTC2_PRIV_R_MASK  SECU2_IRQ_CFG0_ILL_PTC2_PRIV_R_MASK_Msk
#define SECU2_IRQ_CFG0_ILL_PTC2_SEC_W_MASK_Pos  (2U)
#define SECU2_IRQ_CFG0_ILL_PTC2_SEC_W_MASK_Msk  (0x1UL << SECU2_IRQ_CFG0_ILL_PTC2_SEC_W_MASK_Pos)
#define SECU2_IRQ_CFG0_ILL_PTC2_SEC_W_MASK  SECU2_IRQ_CFG0_ILL_PTC2_SEC_W_MASK_Msk
#define SECU2_IRQ_CFG0_ILL_PTC2_SEC_R_MASK_Pos  (3U)
#define SECU2_IRQ_CFG0_ILL_PTC2_SEC_R_MASK_Msk  (0x1UL << SECU2_IRQ_CFG0_ILL_PTC2_SEC_R_MASK_Pos)
#define SECU2_IRQ_CFG0_ILL_PTC2_SEC_R_MASK  SECU2_IRQ_CFG0_ILL_PTC2_SEC_R_MASK_Msk
#define SECU2_IRQ_CFG0_ILL_DMAC2_PRIV_W_MASK_Pos  (4U)
#define SECU2_IRQ_CFG0_ILL_DMAC2_PRIV_W_MASK_Msk  (0x1UL << SECU2_IRQ_CFG0_ILL_DMAC2_PRIV_W_MASK_Pos)
#define SECU2_IRQ_CFG0_ILL_DMAC2_PRIV_W_MASK  SECU2_IRQ_CFG0_ILL_DMAC2_PRIV_W_MASK_Msk
#define SECU2_IRQ_CFG0_ILL_DMAC2_PRIV_R_MASK_Pos  (5U)
#define SECU2_IRQ_CFG0_ILL_DMAC2_PRIV_R_MASK_Msk  (0x1UL << SECU2_IRQ_CFG0_ILL_DMAC2_PRIV_R_MASK_Pos)
#define SECU2_IRQ_CFG0_ILL_DMAC2_PRIV_R_MASK  SECU2_IRQ_CFG0_ILL_DMAC2_PRIV_R_MASK_Msk
#define SECU2_IRQ_CFG0_ILL_DMAC2_SEC_W_MASK_Pos  (6U)
#define SECU2_IRQ_CFG0_ILL_DMAC2_SEC_W_MASK_Msk  (0x1UL << SECU2_IRQ_CFG0_ILL_DMAC2_SEC_W_MASK_Pos)
#define SECU2_IRQ_CFG0_ILL_DMAC2_SEC_W_MASK  SECU2_IRQ_CFG0_ILL_DMAC2_SEC_W_MASK_Msk
#define SECU2_IRQ_CFG0_ILL_DMAC2_SEC_R_MASK_Pos  (7U)
#define SECU2_IRQ_CFG0_ILL_DMAC2_SEC_R_MASK_Msk  (0x1UL << SECU2_IRQ_CFG0_ILL_DMAC2_SEC_R_MASK_Pos)
#define SECU2_IRQ_CFG0_ILL_DMAC2_SEC_R_MASK  SECU2_IRQ_CFG0_ILL_DMAC2_SEC_R_MASK_Msk
#define SECU2_IRQ_CFG0_ILL_SECU2_PRIV_W_MASK_Pos  (8U)
#define SECU2_IRQ_CFG0_ILL_SECU2_PRIV_W_MASK_Msk  (0x1UL << SECU2_IRQ_CFG0_ILL_SECU2_PRIV_W_MASK_Pos)
#define SECU2_IRQ_CFG0_ILL_SECU2_PRIV_W_MASK  SECU2_IRQ_CFG0_ILL_SECU2_PRIV_W_MASK_Msk
#define SECU2_IRQ_CFG0_ILL_SECU2_PRIV_R_MASK_Pos  (9U)
#define SECU2_IRQ_CFG0_ILL_SECU2_PRIV_R_MASK_Msk  (0x1UL << SECU2_IRQ_CFG0_ILL_SECU2_PRIV_R_MASK_Pos)
#define SECU2_IRQ_CFG0_ILL_SECU2_PRIV_R_MASK  SECU2_IRQ_CFG0_ILL_SECU2_PRIV_R_MASK_Msk
#define SECU2_IRQ_CFG0_ILL_SECU2_SEC_W_MASK_Pos  (10U)
#define SECU2_IRQ_CFG0_ILL_SECU2_SEC_W_MASK_Msk  (0x1UL << SECU2_IRQ_CFG0_ILL_SECU2_SEC_W_MASK_Pos)
#define SECU2_IRQ_CFG0_ILL_SECU2_SEC_W_MASK  SECU2_IRQ_CFG0_ILL_SECU2_SEC_W_MASK_Msk
#define SECU2_IRQ_CFG0_ILL_SECU2_SEC_R_MASK_Pos  (11U)
#define SECU2_IRQ_CFG0_ILL_SECU2_SEC_R_MASK_Msk  (0x1UL << SECU2_IRQ_CFG0_ILL_SECU2_SEC_R_MASK_Pos)
#define SECU2_IRQ_CFG0_ILL_SECU2_SEC_R_MASK  SECU2_IRQ_CFG0_ILL_SECU2_SEC_R_MASK_Msk

/***************** Bit definition for SECU2_IRQ_CFG1 register *****************/
#define SECU2_IRQ_CFG1_ILL_RAM0_PRIV_W_MASK_Pos  (0U)
#define SECU2_IRQ_CFG1_ILL_RAM0_PRIV_W_MASK_Msk  (0x1UL << SECU2_IRQ_CFG1_ILL_RAM0_PRIV_W_MASK_Pos)
#define SECU2_IRQ_CFG1_ILL_RAM0_PRIV_W_MASK  SECU2_IRQ_CFG1_ILL_RAM0_PRIV_W_MASK_Msk
#define SECU2_IRQ_CFG1_ILL_RAM0_PRIV_R_MASK_Pos  (1U)
#define SECU2_IRQ_CFG1_ILL_RAM0_PRIV_R_MASK_Msk  (0x1UL << SECU2_IRQ_CFG1_ILL_RAM0_PRIV_R_MASK_Pos)
#define SECU2_IRQ_CFG1_ILL_RAM0_PRIV_R_MASK  SECU2_IRQ_CFG1_ILL_RAM0_PRIV_R_MASK_Msk
#define SECU2_IRQ_CFG1_ILL_RAM0_SEC_W_MASK_Pos  (2U)
#define SECU2_IRQ_CFG1_ILL_RAM0_SEC_W_MASK_Msk  (0x1UL << SECU2_IRQ_CFG1_ILL_RAM0_SEC_W_MASK_Pos)
#define SECU2_IRQ_CFG1_ILL_RAM0_SEC_W_MASK  SECU2_IRQ_CFG1_ILL_RAM0_SEC_W_MASK_Msk
#define SECU2_IRQ_CFG1_ILL_RAM0_SEC_R_MASK_Pos  (3U)
#define SECU2_IRQ_CFG1_ILL_RAM0_SEC_R_MASK_Msk  (0x1UL << SECU2_IRQ_CFG1_ILL_RAM0_SEC_R_MASK_Pos)
#define SECU2_IRQ_CFG1_ILL_RAM0_SEC_R_MASK  SECU2_IRQ_CFG1_ILL_RAM0_SEC_R_MASK_Msk
#define SECU2_IRQ_CFG1_ILL_RAM1_PRIV_W_MASK_Pos  (4U)
#define SECU2_IRQ_CFG1_ILL_RAM1_PRIV_W_MASK_Msk  (0x1UL << SECU2_IRQ_CFG1_ILL_RAM1_PRIV_W_MASK_Pos)
#define SECU2_IRQ_CFG1_ILL_RAM1_PRIV_W_MASK  SECU2_IRQ_CFG1_ILL_RAM1_PRIV_W_MASK_Msk
#define SECU2_IRQ_CFG1_ILL_RAM1_PRIV_R_MASK_Pos  (5U)
#define SECU2_IRQ_CFG1_ILL_RAM1_PRIV_R_MASK_Msk  (0x1UL << SECU2_IRQ_CFG1_ILL_RAM1_PRIV_R_MASK_Pos)
#define SECU2_IRQ_CFG1_ILL_RAM1_PRIV_R_MASK  SECU2_IRQ_CFG1_ILL_RAM1_PRIV_R_MASK_Msk
#define SECU2_IRQ_CFG1_ILL_RAM1_SEC_W_MASK_Pos  (6U)
#define SECU2_IRQ_CFG1_ILL_RAM1_SEC_W_MASK_Msk  (0x1UL << SECU2_IRQ_CFG1_ILL_RAM1_SEC_W_MASK_Pos)
#define SECU2_IRQ_CFG1_ILL_RAM1_SEC_W_MASK  SECU2_IRQ_CFG1_ILL_RAM1_SEC_W_MASK_Msk
#define SECU2_IRQ_CFG1_ILL_RAM1_SEC_R_MASK_Pos  (7U)
#define SECU2_IRQ_CFG1_ILL_RAM1_SEC_R_MASK_Msk  (0x1UL << SECU2_IRQ_CFG1_ILL_RAM1_SEC_R_MASK_Pos)
#define SECU2_IRQ_CFG1_ILL_RAM1_SEC_R_MASK  SECU2_IRQ_CFG1_ILL_RAM1_SEC_R_MASK_Msk

/************** Bit definition for SECU2_LPMST_ATTR_CFG register **************/
#define SECU2_LPMST_ATTR_CFG_LCPU_SEC_Pos  (0U)
#define SECU2_LPMST_ATTR_CFG_LCPU_SEC_Msk  (0x1UL << SECU2_LPMST_ATTR_CFG_LCPU_SEC_Pos)
#define SECU2_LPMST_ATTR_CFG_LCPU_SEC   SECU2_LPMST_ATTR_CFG_LCPU_SEC_Msk
#define SECU2_LPMST_ATTR_CFG_LCPU_SEC_USE_Pos  (2U)
#define SECU2_LPMST_ATTR_CFG_LCPU_SEC_USE_Msk  (0x1UL << SECU2_LPMST_ATTR_CFG_LCPU_SEC_USE_Pos)
#define SECU2_LPMST_ATTR_CFG_LCPU_SEC_USE  SECU2_LPMST_ATTR_CFG_LCPU_SEC_USE_Msk
#define SECU2_LPMST_ATTR_CFG_DMAC2_SEC_Pos  (4U)
#define SECU2_LPMST_ATTR_CFG_DMAC2_SEC_Msk  (0x1UL << SECU2_LPMST_ATTR_CFG_DMAC2_SEC_Pos)
#define SECU2_LPMST_ATTR_CFG_DMAC2_SEC  SECU2_LPMST_ATTR_CFG_DMAC2_SEC_Msk
#define SECU2_LPMST_ATTR_CFG_DMAC2_PRIV_Pos  (5U)
#define SECU2_LPMST_ATTR_CFG_DMAC2_PRIV_Msk  (0x1UL << SECU2_LPMST_ATTR_CFG_DMAC2_PRIV_Pos)
#define SECU2_LPMST_ATTR_CFG_DMAC2_PRIV  SECU2_LPMST_ATTR_CFG_DMAC2_PRIV_Msk
#define SECU2_LPMST_ATTR_CFG_DMAC2_SEC_USE_Pos  (6U)
#define SECU2_LPMST_ATTR_CFG_DMAC2_SEC_USE_Msk  (0x1UL << SECU2_LPMST_ATTR_CFG_DMAC2_SEC_USE_Pos)
#define SECU2_LPMST_ATTR_CFG_DMAC2_SEC_USE  SECU2_LPMST_ATTR_CFG_DMAC2_SEC_USE_Msk
#define SECU2_LPMST_ATTR_CFG_DMAC2_PRIV_USE_Pos  (7U)
#define SECU2_LPMST_ATTR_CFG_DMAC2_PRIV_USE_Msk  (0x1UL << SECU2_LPMST_ATTR_CFG_DMAC2_PRIV_USE_Pos)
#define SECU2_LPMST_ATTR_CFG_DMAC2_PRIV_USE  SECU2_LPMST_ATTR_CFG_DMAC2_PRIV_USE_Msk
#define SECU2_LPMST_ATTR_CFG_PTC2_SEC_Pos  (8U)
#define SECU2_LPMST_ATTR_CFG_PTC2_SEC_Msk  (0x1UL << SECU2_LPMST_ATTR_CFG_PTC2_SEC_Pos)
#define SECU2_LPMST_ATTR_CFG_PTC2_SEC   SECU2_LPMST_ATTR_CFG_PTC2_SEC_Msk
#define SECU2_LPMST_ATTR_CFG_PTC2_PRIV_Pos  (9U)
#define SECU2_LPMST_ATTR_CFG_PTC2_PRIV_Msk  (0x1UL << SECU2_LPMST_ATTR_CFG_PTC2_PRIV_Pos)
#define SECU2_LPMST_ATTR_CFG_PTC2_PRIV  SECU2_LPMST_ATTR_CFG_PTC2_PRIV_Msk
#define SECU2_LPMST_ATTR_CFG_PTC2_SEC_USE_Pos  (10U)
#define SECU2_LPMST_ATTR_CFG_PTC2_SEC_USE_Msk  (0x1UL << SECU2_LPMST_ATTR_CFG_PTC2_SEC_USE_Pos)
#define SECU2_LPMST_ATTR_CFG_PTC2_SEC_USE  SECU2_LPMST_ATTR_CFG_PTC2_SEC_USE_Msk
#define SECU2_LPMST_ATTR_CFG_PTC2_PRIV_USE_Pos  (11U)
#define SECU2_LPMST_ATTR_CFG_PTC2_PRIV_USE_Msk  (0x1UL << SECU2_LPMST_ATTR_CFG_PTC2_PRIV_USE_Pos)
#define SECU2_LPMST_ATTR_CFG_PTC2_PRIV_USE  SECU2_LPMST_ATTR_CFG_PTC2_PRIV_USE_Msk

/************** Bit definition for SECU2_LPSLV_ATTR_CFG register **************/
#define SECU2_LPSLV_ATTR_CFG_DMAC2_SEC_Pos  (0U)
#define SECU2_LPSLV_ATTR_CFG_DMAC2_SEC_Msk  (0x1UL << SECU2_LPSLV_ATTR_CFG_DMAC2_SEC_Pos)
#define SECU2_LPSLV_ATTR_CFG_DMAC2_SEC  SECU2_LPSLV_ATTR_CFG_DMAC2_SEC_Msk
#define SECU2_LPSLV_ATTR_CFG_DMAC2_PRIV_Pos  (1U)
#define SECU2_LPSLV_ATTR_CFG_DMAC2_PRIV_Msk  (0x1UL << SECU2_LPSLV_ATTR_CFG_DMAC2_PRIV_Pos)
#define SECU2_LPSLV_ATTR_CFG_DMAC2_PRIV  SECU2_LPSLV_ATTR_CFG_DMAC2_PRIV_Msk
#define SECU2_LPSLV_ATTR_CFG_PTC2_SEC_Pos  (2U)
#define SECU2_LPSLV_ATTR_CFG_PTC2_SEC_Msk  (0x1UL << SECU2_LPSLV_ATTR_CFG_PTC2_SEC_Pos)
#define SECU2_LPSLV_ATTR_CFG_PTC2_SEC   SECU2_LPSLV_ATTR_CFG_PTC2_SEC_Msk
#define SECU2_LPSLV_ATTR_CFG_PTC2_PRIV_Pos  (3U)
#define SECU2_LPSLV_ATTR_CFG_PTC2_PRIV_Msk  (0x1UL << SECU2_LPSLV_ATTR_CFG_PTC2_PRIV_Pos)
#define SECU2_LPSLV_ATTR_CFG_PTC2_PRIV  SECU2_LPSLV_ATTR_CFG_PTC2_PRIV_Msk

/************** Bit definition for SECU2_RAM0_PRIV_CFG0 register **************/
#define SECU2_RAM0_PRIV_CFG0_ST_ADDR_Pos  (10U)
#define SECU2_RAM0_PRIV_CFG0_ST_ADDR_Msk  (0x3FFFFFUL << SECU2_RAM0_PRIV_CFG0_ST_ADDR_Pos)
#define SECU2_RAM0_PRIV_CFG0_ST_ADDR    SECU2_RAM0_PRIV_CFG0_ST_ADDR_Msk

/************** Bit definition for SECU2_RAM0_PRIV_CFG1 register **************/
#define SECU2_RAM0_PRIV_CFG1_END_ADDR_Pos  (10U)
#define SECU2_RAM0_PRIV_CFG1_END_ADDR_Msk  (0x3FFFFFUL << SECU2_RAM0_PRIV_CFG1_END_ADDR_Pos)
#define SECU2_RAM0_PRIV_CFG1_END_ADDR   SECU2_RAM0_PRIV_CFG1_END_ADDR_Msk

/************** Bit definition for SECU2_RAM0_SEC_CFG0 register ***************/
#define SECU2_RAM0_SEC_CFG0_ST_ADDR_Pos  (10U)
#define SECU2_RAM0_SEC_CFG0_ST_ADDR_Msk  (0x3FFFFFUL << SECU2_RAM0_SEC_CFG0_ST_ADDR_Pos)
#define SECU2_RAM0_SEC_CFG0_ST_ADDR     SECU2_RAM0_SEC_CFG0_ST_ADDR_Msk

/************** Bit definition for SECU2_RAM0_SEC_CFG1 register ***************/
#define SECU2_RAM0_SEC_CFG1_END_ADDR_Pos  (10U)
#define SECU2_RAM0_SEC_CFG1_END_ADDR_Msk  (0x3FFFFFUL << SECU2_RAM0_SEC_CFG1_END_ADDR_Pos)
#define SECU2_RAM0_SEC_CFG1_END_ADDR    SECU2_RAM0_SEC_CFG1_END_ADDR_Msk

/************** Bit definition for SECU2_RAM1_PRIV_CFG0 register **************/
#define SECU2_RAM1_PRIV_CFG0_ST_ADDR_Pos  (10U)
#define SECU2_RAM1_PRIV_CFG0_ST_ADDR_Msk  (0x3FFFFFUL << SECU2_RAM1_PRIV_CFG0_ST_ADDR_Pos)
#define SECU2_RAM1_PRIV_CFG0_ST_ADDR    SECU2_RAM1_PRIV_CFG0_ST_ADDR_Msk

/************** Bit definition for SECU2_RAM1_PRIV_CFG1 register **************/
#define SECU2_RAM1_PRIV_CFG1_END_ADDR_Pos  (10U)
#define SECU2_RAM1_PRIV_CFG1_END_ADDR_Msk  (0x3FFFFFUL << SECU2_RAM1_PRIV_CFG1_END_ADDR_Pos)
#define SECU2_RAM1_PRIV_CFG1_END_ADDR   SECU2_RAM1_PRIV_CFG1_END_ADDR_Msk

/************** Bit definition for SECU2_RAM1_SEC_CFG0 register ***************/
#define SECU2_RAM1_SEC_CFG0_ST_ADDR_Pos  (10U)
#define SECU2_RAM1_SEC_CFG0_ST_ADDR_Msk  (0x3FFFFFUL << SECU2_RAM1_SEC_CFG0_ST_ADDR_Pos)
#define SECU2_RAM1_SEC_CFG0_ST_ADDR     SECU2_RAM1_SEC_CFG0_ST_ADDR_Msk

/************** Bit definition for SECU2_RAM1_SEC_CFG1 register ***************/
#define SECU2_RAM1_SEC_CFG1_END_ADDR_Pos  (10U)
#define SECU2_RAM1_SEC_CFG1_END_ADDR_Msk  (0x3FFFFFUL << SECU2_RAM1_SEC_CFG1_END_ADDR_Pos)
#define SECU2_RAM1_SEC_CFG1_END_ADDR    SECU2_RAM1_SEC_CFG1_END_ADDR_Msk

#endif