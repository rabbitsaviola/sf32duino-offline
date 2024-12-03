#ifndef __LPSYS_PINMUX_H
#define __LPSYS_PINMUX_H

typedef struct
{
__IO uint32_t PAD_PB00;
__IO uint32_t PAD_PB01;
__IO uint32_t PAD_PB02;
__IO uint32_t PAD_PB03;
} LPSYS_PINMUX_TypeDef;


/************* Bit definition for LPSYS_PINMUX_PAD_PB00 register **************/
#define LPSYS_PINMUX_PAD_PB00_FSEL_Pos  (0U)
#define LPSYS_PINMUX_PAD_PB00_FSEL_Msk  (0x7UL << LPSYS_PINMUX_PAD_PB00_FSEL_Pos)
#define LPSYS_PINMUX_PAD_PB00_FSEL      LPSYS_PINMUX_PAD_PB00_FSEL_Msk
#define LPSYS_PINMUX_PAD_PB00_PE_Pos    (4U)
#define LPSYS_PINMUX_PAD_PB00_PE_Msk    (0x1UL << LPSYS_PINMUX_PAD_PB00_PE_Pos)
#define LPSYS_PINMUX_PAD_PB00_PE        LPSYS_PINMUX_PAD_PB00_PE_Msk
#define LPSYS_PINMUX_PAD_PB00_PS_Pos    (5U)
#define LPSYS_PINMUX_PAD_PB00_PS_Msk    (0x1UL << LPSYS_PINMUX_PAD_PB00_PS_Pos)
#define LPSYS_PINMUX_PAD_PB00_PS        LPSYS_PINMUX_PAD_PB00_PS_Msk
#define LPSYS_PINMUX_PAD_PB00_IE_Pos    (6U)
#define LPSYS_PINMUX_PAD_PB00_IE_Msk    (0x1UL << LPSYS_PINMUX_PAD_PB00_IE_Pos)
#define LPSYS_PINMUX_PAD_PB00_IE        LPSYS_PINMUX_PAD_PB00_IE_Msk
#define LPSYS_PINMUX_PAD_PB00_IS_Pos    (7U)
#define LPSYS_PINMUX_PAD_PB00_IS_Msk    (0x1UL << LPSYS_PINMUX_PAD_PB00_IS_Pos)
#define LPSYS_PINMUX_PAD_PB00_IS        LPSYS_PINMUX_PAD_PB00_IS_Msk
#define LPSYS_PINMUX_PAD_PB00_SR_Pos    (8U)
#define LPSYS_PINMUX_PAD_PB00_SR_Msk    (0x1UL << LPSYS_PINMUX_PAD_PB00_SR_Pos)
#define LPSYS_PINMUX_PAD_PB00_SR        LPSYS_PINMUX_PAD_PB00_SR_Msk
#define LPSYS_PINMUX_PAD_PB00_DS0_Pos   (9U)
#define LPSYS_PINMUX_PAD_PB00_DS0_Msk   (0x1UL << LPSYS_PINMUX_PAD_PB00_DS0_Pos)
#define LPSYS_PINMUX_PAD_PB00_DS0       LPSYS_PINMUX_PAD_PB00_DS0_Msk
#define LPSYS_PINMUX_PAD_PB00_DS1_Pos   (10U)
#define LPSYS_PINMUX_PAD_PB00_DS1_Msk   (0x1UL << LPSYS_PINMUX_PAD_PB00_DS1_Pos)
#define LPSYS_PINMUX_PAD_PB00_DS1       LPSYS_PINMUX_PAD_PB00_DS1_Msk
#define LPSYS_PINMUX_PAD_PB00_POE_Pos   (11U)
#define LPSYS_PINMUX_PAD_PB00_POE_Msk   (0x1UL << LPSYS_PINMUX_PAD_PB00_POE_Pos)
#define LPSYS_PINMUX_PAD_PB00_POE       LPSYS_PINMUX_PAD_PB00_POE_Msk

/************* Bit definition for LPSYS_PINMUX_PAD_PB01 register **************/
#define LPSYS_PINMUX_PAD_PB01_FSEL_Pos  (0U)
#define LPSYS_PINMUX_PAD_PB01_FSEL_Msk  (0x7UL << LPSYS_PINMUX_PAD_PB01_FSEL_Pos)
#define LPSYS_PINMUX_PAD_PB01_FSEL      LPSYS_PINMUX_PAD_PB01_FSEL_Msk
#define LPSYS_PINMUX_PAD_PB01_PE_Pos    (4U)
#define LPSYS_PINMUX_PAD_PB01_PE_Msk    (0x1UL << LPSYS_PINMUX_PAD_PB01_PE_Pos)
#define LPSYS_PINMUX_PAD_PB01_PE        LPSYS_PINMUX_PAD_PB01_PE_Msk
#define LPSYS_PINMUX_PAD_PB01_PS_Pos    (5U)
#define LPSYS_PINMUX_PAD_PB01_PS_Msk    (0x1UL << LPSYS_PINMUX_PAD_PB01_PS_Pos)
#define LPSYS_PINMUX_PAD_PB01_PS        LPSYS_PINMUX_PAD_PB01_PS_Msk
#define LPSYS_PINMUX_PAD_PB01_IE_Pos    (6U)
#define LPSYS_PINMUX_PAD_PB01_IE_Msk    (0x1UL << LPSYS_PINMUX_PAD_PB01_IE_Pos)
#define LPSYS_PINMUX_PAD_PB01_IE        LPSYS_PINMUX_PAD_PB01_IE_Msk
#define LPSYS_PINMUX_PAD_PB01_IS_Pos    (7U)
#define LPSYS_PINMUX_PAD_PB01_IS_Msk    (0x1UL << LPSYS_PINMUX_PAD_PB01_IS_Pos)
#define LPSYS_PINMUX_PAD_PB01_IS        LPSYS_PINMUX_PAD_PB01_IS_Msk
#define LPSYS_PINMUX_PAD_PB01_SR_Pos    (8U)
#define LPSYS_PINMUX_PAD_PB01_SR_Msk    (0x1UL << LPSYS_PINMUX_PAD_PB01_SR_Pos)
#define LPSYS_PINMUX_PAD_PB01_SR        LPSYS_PINMUX_PAD_PB01_SR_Msk
#define LPSYS_PINMUX_PAD_PB01_DS0_Pos   (9U)
#define LPSYS_PINMUX_PAD_PB01_DS0_Msk   (0x1UL << LPSYS_PINMUX_PAD_PB01_DS0_Pos)
#define LPSYS_PINMUX_PAD_PB01_DS0       LPSYS_PINMUX_PAD_PB01_DS0_Msk
#define LPSYS_PINMUX_PAD_PB01_DS1_Pos   (10U)
#define LPSYS_PINMUX_PAD_PB01_DS1_Msk   (0x1UL << LPSYS_PINMUX_PAD_PB01_DS1_Pos)
#define LPSYS_PINMUX_PAD_PB01_DS1       LPSYS_PINMUX_PAD_PB01_DS1_Msk
#define LPSYS_PINMUX_PAD_PB01_POE_Pos   (11U)
#define LPSYS_PINMUX_PAD_PB01_POE_Msk   (0x1UL << LPSYS_PINMUX_PAD_PB01_POE_Pos)
#define LPSYS_PINMUX_PAD_PB01_POE       LPSYS_PINMUX_PAD_PB01_POE_Msk

/************* Bit definition for LPSYS_PINMUX_PAD_PB02 register **************/
#define LPSYS_PINMUX_PAD_PB02_FSEL_Pos  (0U)
#define LPSYS_PINMUX_PAD_PB02_FSEL_Msk  (0x7UL << LPSYS_PINMUX_PAD_PB02_FSEL_Pos)
#define LPSYS_PINMUX_PAD_PB02_FSEL      LPSYS_PINMUX_PAD_PB02_FSEL_Msk
#define LPSYS_PINMUX_PAD_PB02_PE_Pos    (4U)
#define LPSYS_PINMUX_PAD_PB02_PE_Msk    (0x1UL << LPSYS_PINMUX_PAD_PB02_PE_Pos)
#define LPSYS_PINMUX_PAD_PB02_PE        LPSYS_PINMUX_PAD_PB02_PE_Msk
#define LPSYS_PINMUX_PAD_PB02_PS_Pos    (5U)
#define LPSYS_PINMUX_PAD_PB02_PS_Msk    (0x1UL << LPSYS_PINMUX_PAD_PB02_PS_Pos)
#define LPSYS_PINMUX_PAD_PB02_PS        LPSYS_PINMUX_PAD_PB02_PS_Msk
#define LPSYS_PINMUX_PAD_PB02_IE_Pos    (6U)
#define LPSYS_PINMUX_PAD_PB02_IE_Msk    (0x1UL << LPSYS_PINMUX_PAD_PB02_IE_Pos)
#define LPSYS_PINMUX_PAD_PB02_IE        LPSYS_PINMUX_PAD_PB02_IE_Msk
#define LPSYS_PINMUX_PAD_PB02_IS_Pos    (7U)
#define LPSYS_PINMUX_PAD_PB02_IS_Msk    (0x1UL << LPSYS_PINMUX_PAD_PB02_IS_Pos)
#define LPSYS_PINMUX_PAD_PB02_IS        LPSYS_PINMUX_PAD_PB02_IS_Msk
#define LPSYS_PINMUX_PAD_PB02_SR_Pos    (8U)
#define LPSYS_PINMUX_PAD_PB02_SR_Msk    (0x1UL << LPSYS_PINMUX_PAD_PB02_SR_Pos)
#define LPSYS_PINMUX_PAD_PB02_SR        LPSYS_PINMUX_PAD_PB02_SR_Msk
#define LPSYS_PINMUX_PAD_PB02_DS0_Pos   (9U)
#define LPSYS_PINMUX_PAD_PB02_DS0_Msk   (0x1UL << LPSYS_PINMUX_PAD_PB02_DS0_Pos)
#define LPSYS_PINMUX_PAD_PB02_DS0       LPSYS_PINMUX_PAD_PB02_DS0_Msk
#define LPSYS_PINMUX_PAD_PB02_DS1_Pos   (10U)
#define LPSYS_PINMUX_PAD_PB02_DS1_Msk   (0x1UL << LPSYS_PINMUX_PAD_PB02_DS1_Pos)
#define LPSYS_PINMUX_PAD_PB02_DS1       LPSYS_PINMUX_PAD_PB02_DS1_Msk
#define LPSYS_PINMUX_PAD_PB02_POE_Pos   (11U)
#define LPSYS_PINMUX_PAD_PB02_POE_Msk   (0x1UL << LPSYS_PINMUX_PAD_PB02_POE_Pos)
#define LPSYS_PINMUX_PAD_PB02_POE       LPSYS_PINMUX_PAD_PB02_POE_Msk

/************* Bit definition for LPSYS_PINMUX_PAD_PB03 register **************/
#define LPSYS_PINMUX_PAD_PB03_FSEL_Pos  (0U)
#define LPSYS_PINMUX_PAD_PB03_FSEL_Msk  (0x7UL << LPSYS_PINMUX_PAD_PB03_FSEL_Pos)
#define LPSYS_PINMUX_PAD_PB03_FSEL      LPSYS_PINMUX_PAD_PB03_FSEL_Msk
#define LPSYS_PINMUX_PAD_PB03_PE_Pos    (4U)
#define LPSYS_PINMUX_PAD_PB03_PE_Msk    (0x1UL << LPSYS_PINMUX_PAD_PB03_PE_Pos)
#define LPSYS_PINMUX_PAD_PB03_PE        LPSYS_PINMUX_PAD_PB03_PE_Msk
#define LPSYS_PINMUX_PAD_PB03_PS_Pos    (5U)
#define LPSYS_PINMUX_PAD_PB03_PS_Msk    (0x1UL << LPSYS_PINMUX_PAD_PB03_PS_Pos)
#define LPSYS_PINMUX_PAD_PB03_PS        LPSYS_PINMUX_PAD_PB03_PS_Msk
#define LPSYS_PINMUX_PAD_PB03_IE_Pos    (6U)
#define LPSYS_PINMUX_PAD_PB03_IE_Msk    (0x1UL << LPSYS_PINMUX_PAD_PB03_IE_Pos)
#define LPSYS_PINMUX_PAD_PB03_IE        LPSYS_PINMUX_PAD_PB03_IE_Msk
#define LPSYS_PINMUX_PAD_PB03_IS_Pos    (7U)
#define LPSYS_PINMUX_PAD_PB03_IS_Msk    (0x1UL << LPSYS_PINMUX_PAD_PB03_IS_Pos)
#define LPSYS_PINMUX_PAD_PB03_IS        LPSYS_PINMUX_PAD_PB03_IS_Msk
#define LPSYS_PINMUX_PAD_PB03_SR_Pos    (8U)
#define LPSYS_PINMUX_PAD_PB03_SR_Msk    (0x1UL << LPSYS_PINMUX_PAD_PB03_SR_Pos)
#define LPSYS_PINMUX_PAD_PB03_SR        LPSYS_PINMUX_PAD_PB03_SR_Msk
#define LPSYS_PINMUX_PAD_PB03_DS0_Pos   (9U)
#define LPSYS_PINMUX_PAD_PB03_DS0_Msk   (0x1UL << LPSYS_PINMUX_PAD_PB03_DS0_Pos)
#define LPSYS_PINMUX_PAD_PB03_DS0       LPSYS_PINMUX_PAD_PB03_DS0_Msk
#define LPSYS_PINMUX_PAD_PB03_DS1_Pos   (10U)
#define LPSYS_PINMUX_PAD_PB03_DS1_Msk   (0x1UL << LPSYS_PINMUX_PAD_PB03_DS1_Pos)
#define LPSYS_PINMUX_PAD_PB03_DS1       LPSYS_PINMUX_PAD_PB03_DS1_Msk
#define LPSYS_PINMUX_PAD_PB03_POE_Pos   (11U)
#define LPSYS_PINMUX_PAD_PB03_POE_Msk   (0x1UL << LPSYS_PINMUX_PAD_PB03_POE_Pos)
#define LPSYS_PINMUX_PAD_PB03_POE       LPSYS_PINMUX_PAD_PB03_POE_Msk

#endif