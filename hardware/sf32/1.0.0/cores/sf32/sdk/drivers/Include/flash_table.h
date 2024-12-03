
#ifndef _SIF_FLASH_CMD_TABLE_H__
#define _SIF_FLASH_CMD_TABLE_H__

#ifdef SF32LB55X
    #include "bf0_hal_qspi_ex.h"
#else
    #include "bf0_hal_mpi_ex.h"
#endif

typedef struct FLASH_FULL_CHIP_ID
{
    uint8_t manufacture_id;
    uint8_t memory_type;
    uint8_t memory_density;// union 16 bits as device ID for NAND
    uint8_t support_dtr;
    uint32_t mem_size;  // flash size with bytes
} FLASH_RDID_TYPE_T;


const SPI_FLASH_FACT_CFG_T *spi_flash_get_cmd_by_id(uint8_t fid, uint8_t did, uint8_t type);
int spi_flash_get_size_by_id(uint8_t fid, uint8_t did, uint8_t type);
int spi_flash_is_support_dtr(uint8_t fid, uint8_t did, uint8_t type);

const SPI_FLASH_FACT_CFG_T *spi_nand_get_cmd_by_id(uint8_t fid, uint8_t did, uint8_t type);
const SPI_FLASH_FACT_CFG_T *spi_nand_get_default_ctable(void);
int spi_nand_get_size_by_id(uint8_t fid, uint8_t did, uint8_t type);

#endif  // _SIF_FLASH_CMD_TABLE_H__
