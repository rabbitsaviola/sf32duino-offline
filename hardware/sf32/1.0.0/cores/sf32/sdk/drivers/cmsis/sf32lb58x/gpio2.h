#ifndef __GPIO2_H
#define __GPIO2_H

typedef struct
{
    __IO uint32_t DIR0;
    __IO uint32_t DOR0;
    __IO uint32_t DOSR0;
    __IO uint32_t DOCR0;
    __IO uint32_t DOER0;
    __IO uint32_t DOESR0;
    __IO uint32_t DOECR0;
    __IO uint32_t IER0;
    __IO uint32_t IESR0;
    __IO uint32_t IECR0;
    __IO uint32_t ITR0;
    __IO uint32_t ITSR0;
    __IO uint32_t ITCR0;
    __IO uint32_t IPHR0;
    __IO uint32_t IPHSR0;
    __IO uint32_t IPHCR0;
    __IO uint32_t IPLR0;
    __IO uint32_t IPLSR0;
    __IO uint32_t IPLCR0;
    __IO uint32_t ISR0;
    __IO uint32_t IER0_EXT;
    __IO uint32_t IESR0_EXT;
    __IO uint32_t IECR0_EXT;
    __IO uint32_t ISR0_EXT;
    __IO uint32_t OEMR0;
    __IO uint32_t OEMSR0;
    __IO uint32_t OEMCR0;
    __IO uint32_t RSVD1[5];
    __IO uint32_t DIR1;
    __IO uint32_t DOR1;
    __IO uint32_t DOSR1;
    __IO uint32_t DOCR1;
    __IO uint32_t DOER1;
    __IO uint32_t DOESR1;
    __IO uint32_t DOECR1;
    __IO uint32_t IER1;
    __IO uint32_t IESR1;
    __IO uint32_t IECR1;
    __IO uint32_t ITR1;
    __IO uint32_t ITSR1;
    __IO uint32_t ITCR1;
    __IO uint32_t IPHR1;
    __IO uint32_t IPHSR1;
    __IO uint32_t IPHCR1;
    __IO uint32_t IPLR1;
    __IO uint32_t IPLSR1;
    __IO uint32_t IPLCR1;
    __IO uint32_t ISR1;
    __IO uint32_t IER1_EXT;
    __IO uint32_t IESR1_EXT;
    __IO uint32_t IECR1_EXT;
    __IO uint32_t ISR1_EXT;
    __IO uint32_t OEMR1;
    __IO uint32_t OEMSR1;
    __IO uint32_t OEMCR1;
} GPIO2_TypeDef;


/******************* Bit definition for GPIO2_DIR0 register *******************/
#define GPIO2_DIR0_IN_Pos               (0U)
#define GPIO2_DIR0_IN_Msk               (0xFFFFFFFFUL << GPIO2_DIR0_IN_Pos)
#define GPIO2_DIR0_IN                   GPIO2_DIR0_IN_Msk

/******************* Bit definition for GPIO2_DOR0 register *******************/
#define GPIO2_DOR0_OUT_Pos              (0U)
#define GPIO2_DOR0_OUT_Msk              (0xFFFFFFFFUL << GPIO2_DOR0_OUT_Pos)
#define GPIO2_DOR0_OUT                  GPIO2_DOR0_OUT_Msk

/****************** Bit definition for GPIO2_DOSR0 register *******************/
#define GPIO2_DOSR0_DOS_Pos             (0U)
#define GPIO2_DOSR0_DOS_Msk             (0xFFFFFFFFUL << GPIO2_DOSR0_DOS_Pos)
#define GPIO2_DOSR0_DOS                 GPIO2_DOSR0_DOS_Msk

/****************** Bit definition for GPIO2_DOCR0 register *******************/
#define GPIO2_DOCR0_DOC_Pos             (0U)
#define GPIO2_DOCR0_DOC_Msk             (0xFFFFFFFFUL << GPIO2_DOCR0_DOC_Pos)
#define GPIO2_DOCR0_DOC                 GPIO2_DOCR0_DOC_Msk

/****************** Bit definition for GPIO2_DOER0 register *******************/
#define GPIO2_DOER0_DOE_Pos             (0U)
#define GPIO2_DOER0_DOE_Msk             (0xFFFFFFFFUL << GPIO2_DOER0_DOE_Pos)
#define GPIO2_DOER0_DOE                 GPIO2_DOER0_DOE_Msk

/****************** Bit definition for GPIO2_DOESR0 register ******************/
#define GPIO2_DOESR0_DOES_Pos           (0U)
#define GPIO2_DOESR0_DOES_Msk           (0xFFFFFFFFUL << GPIO2_DOESR0_DOES_Pos)
#define GPIO2_DOESR0_DOES               GPIO2_DOESR0_DOES_Msk

/****************** Bit definition for GPIO2_DOECR0 register ******************/
#define GPIO2_DOECR0_DOEC_Pos           (0U)
#define GPIO2_DOECR0_DOEC_Msk           (0xFFFFFFFFUL << GPIO2_DOECR0_DOEC_Pos)
#define GPIO2_DOECR0_DOEC               GPIO2_DOECR0_DOEC_Msk

/******************* Bit definition for GPIO2_IER0 register *******************/
#define GPIO2_IER0_IER_Pos              (0U)
#define GPIO2_IER0_IER_Msk              (0xFFFFFFFFUL << GPIO2_IER0_IER_Pos)
#define GPIO2_IER0_IER                  GPIO2_IER0_IER_Msk

/****************** Bit definition for GPIO2_IESR0 register *******************/
#define GPIO2_IESR0_IES_Pos             (0U)
#define GPIO2_IESR0_IES_Msk             (0xFFFFFFFFUL << GPIO2_IESR0_IES_Pos)
#define GPIO2_IESR0_IES                 GPIO2_IESR0_IES_Msk

/****************** Bit definition for GPIO2_IECR0 register *******************/
#define GPIO2_IECR0_IEC_Pos             (0U)
#define GPIO2_IECR0_IEC_Msk             (0xFFFFFFFFUL << GPIO2_IECR0_IEC_Pos)
#define GPIO2_IECR0_IEC                 GPIO2_IECR0_IEC_Msk

/******************* Bit definition for GPIO2_ITR0 register *******************/
#define GPIO2_ITR0_ITR_Pos              (0U)
#define GPIO2_ITR0_ITR_Msk              (0xFFFFFFFFUL << GPIO2_ITR0_ITR_Pos)
#define GPIO2_ITR0_ITR                  GPIO2_ITR0_ITR_Msk

/****************** Bit definition for GPIO2_ITSR0 register *******************/
#define GPIO2_ITSR0_ITS_Pos             (0U)
#define GPIO2_ITSR0_ITS_Msk             (0xFFFFFFFFUL << GPIO2_ITSR0_ITS_Pos)
#define GPIO2_ITSR0_ITS                 GPIO2_ITSR0_ITS_Msk

/****************** Bit definition for GPIO2_ITCR0 register *******************/
#define GPIO2_ITCR0_ITC_Pos             (0U)
#define GPIO2_ITCR0_ITC_Msk             (0xFFFFFFFFUL << GPIO2_ITCR0_ITC_Pos)
#define GPIO2_ITCR0_ITC                 GPIO2_ITCR0_ITC_Msk

/****************** Bit definition for GPIO2_IPHR0 register *******************/
#define GPIO2_IPHR0_IPH_Pos             (0U)
#define GPIO2_IPHR0_IPH_Msk             (0xFFFFFFFFUL << GPIO2_IPHR0_IPH_Pos)
#define GPIO2_IPHR0_IPH                 GPIO2_IPHR0_IPH_Msk

/****************** Bit definition for GPIO2_IPHSR0 register ******************/
#define GPIO2_IPHSR0_IPHS_Pos           (0U)
#define GPIO2_IPHSR0_IPHS_Msk           (0xFFFFFFFFUL << GPIO2_IPHSR0_IPHS_Pos)
#define GPIO2_IPHSR0_IPHS               GPIO2_IPHSR0_IPHS_Msk

/****************** Bit definition for GPIO2_IPHCR0 register ******************/
#define GPIO2_IPHCR0_IPHC_Pos           (0U)
#define GPIO2_IPHCR0_IPHC_Msk           (0xFFFFFFFFUL << GPIO2_IPHCR0_IPHC_Pos)
#define GPIO2_IPHCR0_IPHC               GPIO2_IPHCR0_IPHC_Msk

/****************** Bit definition for GPIO2_IPLR0 register *******************/
#define GPIO2_IPLR0_IPL_Pos             (0U)
#define GPIO2_IPLR0_IPL_Msk             (0xFFFFFFFFUL << GPIO2_IPLR0_IPL_Pos)
#define GPIO2_IPLR0_IPL                 GPIO2_IPLR0_IPL_Msk

/****************** Bit definition for GPIO2_IPLSR0 register ******************/
#define GPIO2_IPLSR0_IPLS_Pos           (0U)
#define GPIO2_IPLSR0_IPLS_Msk           (0xFFFFFFFFUL << GPIO2_IPLSR0_IPLS_Pos)
#define GPIO2_IPLSR0_IPLS               GPIO2_IPLSR0_IPLS_Msk

/****************** Bit definition for GPIO2_IPLCR0 register ******************/
#define GPIO2_IPLCR0_IPLC_Pos           (0U)
#define GPIO2_IPLCR0_IPLC_Msk           (0xFFFFFFFFUL << GPIO2_IPLCR0_IPLC_Pos)
#define GPIO2_IPLCR0_IPLC               GPIO2_IPLCR0_IPLC_Msk

/******************* Bit definition for GPIO2_ISR0 register *******************/
#define GPIO2_ISR0_IS_Pos               (0U)
#define GPIO2_ISR0_IS_Msk               (0xFFFFFFFFUL << GPIO2_ISR0_IS_Pos)
#define GPIO2_ISR0_IS                   GPIO2_ISR0_IS_Msk

/***************** Bit definition for GPIO2_IER0_EXT register *****************/
#define GPIO2_IER0_EXT_IER_Pos          (0U)
#define GPIO2_IER0_EXT_IER_Msk          (0xFFFFFFFFUL << GPIO2_IER0_EXT_IER_Pos)
#define GPIO2_IER0_EXT_IER              GPIO2_IER0_EXT_IER_Msk

/**************** Bit definition for GPIO2_IESR0_EXT register *****************/
#define GPIO2_IESR0_EXT_IES_Pos         (0U)
#define GPIO2_IESR0_EXT_IES_Msk         (0xFFFFFFFFUL << GPIO2_IESR0_EXT_IES_Pos)
#define GPIO2_IESR0_EXT_IES             GPIO2_IESR0_EXT_IES_Msk

/**************** Bit definition for GPIO2_IECR0_EXT register *****************/
#define GPIO2_IECR0_EXT_IEC_Pos         (0U)
#define GPIO2_IECR0_EXT_IEC_Msk         (0xFFFFFFFFUL << GPIO2_IECR0_EXT_IEC_Pos)
#define GPIO2_IECR0_EXT_IEC             GPIO2_IECR0_EXT_IEC_Msk

/***************** Bit definition for GPIO2_ISR0_EXT register *****************/
#define GPIO2_ISR0_EXT_IS_Pos           (0U)
#define GPIO2_ISR0_EXT_IS_Msk           (0xFFFFFFFFUL << GPIO2_ISR0_EXT_IS_Pos)
#define GPIO2_ISR0_EXT_IS               GPIO2_ISR0_EXT_IS_Msk

/****************** Bit definition for GPIO2_OEMR0 register *******************/
#define GPIO2_OEMR0_OEM_Pos             (0U)
#define GPIO2_OEMR0_OEM_Msk             (0xFFFFFFFFUL << GPIO2_OEMR0_OEM_Pos)
#define GPIO2_OEMR0_OEM                 GPIO2_OEMR0_OEM_Msk

/****************** Bit definition for GPIO2_OEMSR0 register ******************/
#define GPIO2_OEMSR0_OEMS_Pos           (0U)
#define GPIO2_OEMSR0_OEMS_Msk           (0xFFFFFFFFUL << GPIO2_OEMSR0_OEMS_Pos)
#define GPIO2_OEMSR0_OEMS               GPIO2_OEMSR0_OEMS_Msk

/****************** Bit definition for GPIO2_OEMCR0 register ******************/
#define GPIO2_OEMCR0_OEMC_Pos           (0U)
#define GPIO2_OEMCR0_OEMC_Msk           (0xFFFFFFFFUL << GPIO2_OEMCR0_OEMC_Pos)
#define GPIO2_OEMCR0_OEMC               GPIO2_OEMCR0_OEMC_Msk

/******************* Bit definition for GPIO2_DIR1 register *******************/
#define GPIO2_DIR1_IN_Pos               (0U)
#define GPIO2_DIR1_IN_Msk               (0xFFFFFFFFUL << GPIO2_DIR1_IN_Pos)
#define GPIO2_DIR1_IN                   GPIO2_DIR1_IN_Msk

/******************* Bit definition for GPIO2_DOR1 register *******************/
#define GPIO2_DOR1_OUT_Pos              (0U)
#define GPIO2_DOR1_OUT_Msk              (0xFFFFFFFFUL << GPIO2_DOR1_OUT_Pos)
#define GPIO2_DOR1_OUT                  GPIO2_DOR1_OUT_Msk

/****************** Bit definition for GPIO2_DOSR1 register *******************/
#define GPIO2_DOSR1_DOS_Pos             (0U)
#define GPIO2_DOSR1_DOS_Msk             (0xFFFFFFFFUL << GPIO2_DOSR1_DOS_Pos)
#define GPIO2_DOSR1_DOS                 GPIO2_DOSR1_DOS_Msk

/****************** Bit definition for GPIO2_DOCR1 register *******************/
#define GPIO2_DOCR1_DOC_Pos             (0U)
#define GPIO2_DOCR1_DOC_Msk             (0xFFFFFFFFUL << GPIO2_DOCR1_DOC_Pos)
#define GPIO2_DOCR1_DOC                 GPIO2_DOCR1_DOC_Msk

/****************** Bit definition for GPIO2_DOER1 register *******************/
#define GPIO2_DOER1_DOE_Pos             (0U)
#define GPIO2_DOER1_DOE_Msk             (0xFFFFFFFFUL << GPIO2_DOER1_DOE_Pos)
#define GPIO2_DOER1_DOE                 GPIO2_DOER1_DOE_Msk

/****************** Bit definition for GPIO2_DOESR1 register ******************/
#define GPIO2_DOESR1_DOES_Pos           (0U)
#define GPIO2_DOESR1_DOES_Msk           (0xFFFFFFFFUL << GPIO2_DOESR1_DOES_Pos)
#define GPIO2_DOESR1_DOES               GPIO2_DOESR1_DOES_Msk

/****************** Bit definition for GPIO2_DOECR1 register ******************/
#define GPIO2_DOECR1_DOEC_Pos           (0U)
#define GPIO2_DOECR1_DOEC_Msk           (0xFFFFFFFFUL << GPIO2_DOECR1_DOEC_Pos)
#define GPIO2_DOECR1_DOEC               GPIO2_DOECR1_DOEC_Msk

/******************* Bit definition for GPIO2_IER1 register *******************/
#define GPIO2_IER1_IER_Pos              (0U)
#define GPIO2_IER1_IER_Msk              (0xFFFFFFFFUL << GPIO2_IER1_IER_Pos)
#define GPIO2_IER1_IER                  GPIO2_IER1_IER_Msk

/****************** Bit definition for GPIO2_IESR1 register *******************/
#define GPIO2_IESR1_IES_Pos             (0U)
#define GPIO2_IESR1_IES_Msk             (0xFFFFFFFFUL << GPIO2_IESR1_IES_Pos)
#define GPIO2_IESR1_IES                 GPIO2_IESR1_IES_Msk

/****************** Bit definition for GPIO2_IECR1 register *******************/
#define GPIO2_IECR1_IEC_Pos             (0U)
#define GPIO2_IECR1_IEC_Msk             (0xFFFFFFFFUL << GPIO2_IECR1_IEC_Pos)
#define GPIO2_IECR1_IEC                 GPIO2_IECR1_IEC_Msk

/******************* Bit definition for GPIO2_ITR1 register *******************/
#define GPIO2_ITR1_ITR_Pos              (0U)
#define GPIO2_ITR1_ITR_Msk              (0xFFFFFFFFUL << GPIO2_ITR1_ITR_Pos)
#define GPIO2_ITR1_ITR                  GPIO2_ITR1_ITR_Msk

/****************** Bit definition for GPIO2_ITSR1 register *******************/
#define GPIO2_ITSR1_ITS_Pos             (0U)
#define GPIO2_ITSR1_ITS_Msk             (0xFFFFFFFFUL << GPIO2_ITSR1_ITS_Pos)
#define GPIO2_ITSR1_ITS                 GPIO2_ITSR1_ITS_Msk

/****************** Bit definition for GPIO2_ITCR1 register *******************/
#define GPIO2_ITCR1_ITC_Pos             (0U)
#define GPIO2_ITCR1_ITC_Msk             (0xFFFFFFFFUL << GPIO2_ITCR1_ITC_Pos)
#define GPIO2_ITCR1_ITC                 GPIO2_ITCR1_ITC_Msk

/****************** Bit definition for GPIO2_IPHR1 register *******************/
#define GPIO2_IPHR1_IPH_Pos             (0U)
#define GPIO2_IPHR1_IPH_Msk             (0xFFFFFFFFUL << GPIO2_IPHR1_IPH_Pos)
#define GPIO2_IPHR1_IPH                 GPIO2_IPHR1_IPH_Msk

/****************** Bit definition for GPIO2_IPHSR1 register ******************/
#define GPIO2_IPHSR1_IPHS_Pos           (0U)
#define GPIO2_IPHSR1_IPHS_Msk           (0xFFFFFFFFUL << GPIO2_IPHSR1_IPHS_Pos)
#define GPIO2_IPHSR1_IPHS               GPIO2_IPHSR1_IPHS_Msk

/****************** Bit definition for GPIO2_IPHCR1 register ******************/
#define GPIO2_IPHCR1_IPHC_Pos           (0U)
#define GPIO2_IPHCR1_IPHC_Msk           (0xFFFFFFFFUL << GPIO2_IPHCR1_IPHC_Pos)
#define GPIO2_IPHCR1_IPHC               GPIO2_IPHCR1_IPHC_Msk

/****************** Bit definition for GPIO2_IPLR1 register *******************/
#define GPIO2_IPLR1_IPL_Pos             (0U)
#define GPIO2_IPLR1_IPL_Msk             (0xFFFFFFFFUL << GPIO2_IPLR1_IPL_Pos)
#define GPIO2_IPLR1_IPL                 GPIO2_IPLR1_IPL_Msk

/****************** Bit definition for GPIO2_IPLSR1 register ******************/
#define GPIO2_IPLSR1_IPLS_Pos           (0U)
#define GPIO2_IPLSR1_IPLS_Msk           (0xFFFFFFFFUL << GPIO2_IPLSR1_IPLS_Pos)
#define GPIO2_IPLSR1_IPLS               GPIO2_IPLSR1_IPLS_Msk

/****************** Bit definition for GPIO2_IPLCR1 register ******************/
#define GPIO2_IPLCR1_IPLC_Pos           (0U)
#define GPIO2_IPLCR1_IPLC_Msk           (0xFFFFFFFFUL << GPIO2_IPLCR1_IPLC_Pos)
#define GPIO2_IPLCR1_IPLC               GPIO2_IPLCR1_IPLC_Msk

/******************* Bit definition for GPIO2_ISR1 register *******************/
#define GPIO2_ISR1_IS_Pos               (0U)
#define GPIO2_ISR1_IS_Msk               (0xFFFFFFFFUL << GPIO2_ISR1_IS_Pos)
#define GPIO2_ISR1_IS                   GPIO2_ISR1_IS_Msk

/***************** Bit definition for GPIO2_IER1_EXT register *****************/
#define GPIO2_IER1_EXT_IER_Pos          (0U)
#define GPIO2_IER1_EXT_IER_Msk          (0xFFFFFFFFUL << GPIO2_IER1_EXT_IER_Pos)
#define GPIO2_IER1_EXT_IER              GPIO2_IER1_EXT_IER_Msk

/**************** Bit definition for GPIO2_IESR1_EXT register *****************/
#define GPIO2_IESR1_EXT_IES_Pos         (0U)
#define GPIO2_IESR1_EXT_IES_Msk         (0xFFFFFFFFUL << GPIO2_IESR1_EXT_IES_Pos)
#define GPIO2_IESR1_EXT_IES             GPIO2_IESR1_EXT_IES_Msk

/**************** Bit definition for GPIO2_IECR1_EXT register *****************/
#define GPIO2_IECR1_EXT_IEC_Pos         (0U)
#define GPIO2_IECR1_EXT_IEC_Msk         (0xFFFFFFFFUL << GPIO2_IECR1_EXT_IEC_Pos)
#define GPIO2_IECR1_EXT_IEC             GPIO2_IECR1_EXT_IEC_Msk

/***************** Bit definition for GPIO2_ISR1_EXT register *****************/
#define GPIO2_ISR1_EXT_IS_Pos           (0U)
#define GPIO2_ISR1_EXT_IS_Msk           (0xFFFFFFFFUL << GPIO2_ISR1_EXT_IS_Pos)
#define GPIO2_ISR1_EXT_IS               GPIO2_ISR1_EXT_IS_Msk

/****************** Bit definition for GPIO2_OEMR1 register *******************/
#define GPIO2_OEMR1_OEM_Pos             (0U)
#define GPIO2_OEMR1_OEM_Msk             (0xFFFFFFFFUL << GPIO2_OEMR1_OEM_Pos)
#define GPIO2_OEMR1_OEM                 GPIO2_OEMR1_OEM_Msk

/****************** Bit definition for GPIO2_OEMSR1 register ******************/
#define GPIO2_OEMSR1_OEMS_Pos           (0U)
#define GPIO2_OEMSR1_OEMS_Msk           (0xFFFFFFFFUL << GPIO2_OEMSR1_OEMS_Pos)
#define GPIO2_OEMSR1_OEMS               GPIO2_OEMSR1_OEMS_Msk

/****************** Bit definition for GPIO2_OEMCR1 register ******************/
#define GPIO2_OEMCR1_OEMC_Pos           (0U)
#define GPIO2_OEMCR1_OEMC_Msk           (0xFFFFFFFFUL << GPIO2_OEMCR1_OEMC_Pos)
#define GPIO2_OEMCR1_OEMC               GPIO2_OEMCR1_OEMC_Msk

#endif