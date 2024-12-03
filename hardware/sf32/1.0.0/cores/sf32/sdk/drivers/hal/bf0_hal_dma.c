/**
  ******************************************************************************
  * @file   bf0_hal_dma.c
  * @author Sifli software development team
  * @brief   DMA HAL module driver.
  *
  ******************************************************************************
*/
/**
 *
 * Copyright (c) 2019 - 2022,  Sifli Technology
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Sifli integrated circuit
 *    in a product or a software update for such product, must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Sifli nor the names of its contributors may be used to endorse
 *    or promote products derived from this software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Sifli integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SIFLI TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SIFLI TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "bf0_hal.h"

/** @addtogroup BF0_HAL_Driver
  * @{
  */

/** @defgroup DMA DMA
  * @brief DMA HAL module driver
  * @{
  */

#if defined(HAL_DMA_MODULE_ENABLED)||defined(_SIFLI_DOXYGEN_)

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/** @defgroup DMA_Private_Functions DMA Private Functions
  * @{
  */
static void DMA_SetConfig(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t Counts);
static uint32_t DMA_GetSrcBytes(DMA_HandleTypeDef *hdma, uint32_t Counts);


/**
  * @} DMA_Private_Functions
  */

/* Exported functions ---------------------------------------------------------*/

/** @defgroup DMA_Exported_Functions DMA Exported Functions
  * @{
  */

/** @defgroup DMA_Exported_Functions_Group1 Initialization and de-initialization functions
 *  @brief   Initialization and de-initialization functions
 *
@verbatim
 ===============================================================================
             ##### Initialization and de-initialization functions  #####
 ===============================================================================
    [..]
    This section provides functions allowing to initialize the DMA Channel source
    and destination addresses, incrementation and data sizes, transfer direction,
    circular/normal mode selection, memory-to-memory mode selection and Channel priority value.
    [..]
    The HAL_DMA_Init() function follows the DMA configuration procedures as described in
    reference manual.

@endverbatim
  * @{
  */

/**
  * @brief  Initialize the DMA according to the specified
  *         parameters in the DMA_InitTypeDef and initialize the associated handle.
  * @param  hdma Pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA Channel.
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_DMA_Init(DMA_HandleTypeDef *hdma)
{
    uint32_t tmp;

    /* Check the DMA handle allocation */
    if (hdma == NULL)
    {
        return HAL_ERROR;
    }

    /* Check the parameters */
    HAL_ASSERT(IS_DMA_ALL_INSTANCE(hdma->Instance));
    HAL_ASSERT(IS_DMA_DIRECTION(hdma->Init.Direction));
    HAL_ASSERT(IS_DMA_PERIPHERAL_INC_STATE(hdma->Init.PeriphInc));
    HAL_ASSERT(IS_DMA_MEMORY_INC_STATE(hdma->Init.MemInc));
    HAL_ASSERT(IS_DMA_PERIPHERAL_DATA_SIZE(hdma->Init.PeriphDataAlignment));
    HAL_ASSERT(IS_DMA_MEMORY_DATA_SIZE(hdma->Init.MemDataAlignment));
    HAL_ASSERT(IS_DMA_MODE(hdma->Init.Mode));
    HAL_ASSERT(IS_DMA_PRIORITY(hdma->Init.Priority));

    HAL_ASSERT(IS_DMA_ALL_REQUEST(hdma->Init.Request));

    /* Compute the channel index */
    if (((uint32_t)(hdma->Instance) >= (uint32_t)(DMA1_Channel1))
            && ((uint32_t)(hdma->Instance) <= (uint32_t)(DMA1_Channel8)))
    {
        /* DMA1 */
        hdma->ChannelIndex = (((uint32_t)hdma->Instance - (uint32_t)DMA1_Channel1) / ((uint32_t)DMA1_Channel2 - (uint32_t)DMA1_Channel1)) << 2;
        hdma->DmaBaseAddress = DMA1;
    }
    else if (((uint32_t)(hdma->Instance) >= (uint32_t)(DMA2_Channel1))
             && ((uint32_t)(hdma->Instance) <= (uint32_t)(DMA2_Channel8)))
    {
        /* DMA2 */
        hdma->ChannelIndex = (((uint32_t)hdma->Instance - (uint32_t)DMA2_Channel1) / ((uint32_t)DMA2_Channel2 - (uint32_t)DMA2_Channel1)) << 2;
        hdma->DmaBaseAddress = DMA2;
    }
#ifdef SF32LB58X
    else if (((uint32_t)(hdma->Instance) >= (uint32_t)(DMA3_Channel1))
             && ((uint32_t)(hdma->Instance) <= (uint32_t)(DMA3_Channel8)))
    {
        /* DMA3 */
        hdma->ChannelIndex = (((uint32_t)hdma->Instance - (uint32_t)DMA3_Channel1) / ((uint32_t)DMA3_Channel2 - (uint32_t)DMA3_Channel1)) << 2;
        hdma->DmaBaseAddress = DMA3;
    }
#endif
    else
    {
        //TODO: error

    }

    /* Change DMA peripheral state */
    hdma->State = HAL_DMA_STATE_BUSY;

    /* Set burst size */
    hdma->Instance->CBSR = hdma->Init.BurstSize;

    /* Get the CR register value */
    tmp = hdma->Instance->CCR;

    /* Clear PL, MSIZE, PSIZE, MINC, PINC, CIRC, DIR and MEM2MEM bits */
    tmp &= ((uint32_t)~(DMA_CCR_PL    | DMA_CCR_MSIZE  | DMA_CCR_PSIZE  |
                        DMA_CCR_MINC  | DMA_CCR_PINC   | DMA_CCR_CIRC   |
                        DMA_CCR_DIR   | DMA_CCR_MEM2MEM));

    /* Prepare the DMA Channel configuration */
    tmp |=  hdma->Init.Direction        |
            hdma->Init.PeriphInc           | hdma->Init.MemInc           |
            hdma->Init.PeriphDataAlignment | hdma->Init.MemDataAlignment |
            hdma->Init.Mode                | hdma->Init.Priority;

    /* Write to DMA Channel CR register */
    hdma->Instance->CCR = tmp;


    /* Set request selection */
    if (hdma->Init.Direction != DMA_MEMORY_TO_MEMORY)
    {

        uint8_t index;
        index = (hdma->ChannelIndex >> 2) & 7;

        if (index <= 3)
        {
            /* Reset request selection for DMA1 Channelx */
            hdma->DmaBaseAddress->CSELR1 &= ~(DMA_CSELR_C1S << (index * DMAC_CSELR1_C2S_Pos));

            /* Configure request selection for DMA1 Channelx */
            hdma->DmaBaseAddress->CSELR1 |= (uint32_t)(hdma->Init.Request << (index * DMAC_CSELR1_C2S_Pos));
        }
        else
        {
            index &= 3;
            /* Reset request selection for DMA1 Channelx */
            hdma->DmaBaseAddress->CSELR2 &= ~(DMA_CSELR_C1S << (index * DMAC_CSELR1_C2S_Pos));

            /* Configure request selection for DMA1 Channelx */
            hdma->DmaBaseAddress->CSELR2 |= (uint32_t)(hdma->Init.Request << (index * DMAC_CSELR1_C2S_Pos));
        }

    }


    /* Initialise the error code */
    hdma->ErrorCode = HAL_DMA_ERROR_NONE;

    /* Initialize the DMA state*/
    hdma->State  = HAL_DMA_STATE_READY;

    /* Allocate lock resource and initialize it */
    hdma->Lock = HAL_UNLOCKED;

    return HAL_OK;
}

/**
  * @brief  DeInitialize the DMA peripheral.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA Channel.
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_DMA_DeInit(DMA_HandleTypeDef *hdma)
{

    /* Check the DMA handle allocation */
    if (NULL == hdma)
    {
        return HAL_ERROR;
    }

    /* Check the parameters */
    HAL_ASSERT(IS_DMA_ALL_INSTANCE(hdma->Instance));

    /* Disable the selected DMA Channelx */
    __HAL_DMA_DISABLE(hdma);

    /* Compute the channel index */
    if (((uint32_t)(hdma->Instance) >= (uint32_t)(DMA1_Channel1))
            && ((uint32_t)(hdma->Instance) <= (uint32_t)(DMA1_Channel8)))
    {
        /* DMA1 */
        hdma->ChannelIndex = (((uint32_t)hdma->Instance - (uint32_t)DMA1_Channel1) / ((uint32_t)DMA1_Channel2 - (uint32_t)DMA1_Channel1)) << 2;
        hdma->DmaBaseAddress = DMA1;
    }
    else if (((uint32_t)(hdma->Instance) >= (uint32_t)(DMA2_Channel1))
             && ((uint32_t)(hdma->Instance) <= (uint32_t)(DMA2_Channel8)))
    {
        /* DMA2 */
        hdma->ChannelIndex = (((uint32_t)hdma->Instance - (uint32_t)DMA2_Channel1) / ((uint32_t)DMA2_Channel2 - (uint32_t)DMA2_Channel1)) << 2;
        hdma->DmaBaseAddress = DMA2;
    }
#ifdef SF32LB58X
    else if (((uint32_t)(hdma->Instance) >= (uint32_t)(DMA3_Channel1))
             && ((uint32_t)(hdma->Instance) <= (uint32_t)(DMA3_Channel8)))
    {
        /* DMA3 */
        hdma->ChannelIndex = (((uint32_t)hdma->Instance - (uint32_t)DMA3_Channel1) / ((uint32_t)DMA3_Channel2 - (uint32_t)DMA3_Channel1)) << 2;
        hdma->DmaBaseAddress = DMA3;
    }
#endif
    else
    {
        //TODO: error
    }

    /* Reset DMA Channel control register */
    hdma->Instance->CCR  = 0;

    /* Clear all flags */
    hdma->DmaBaseAddress->IFCR = (DMA_ISR_GIF1 << (hdma->ChannelIndex & 0x1cU));


    /* Reset DMA channel selection register */

    if ((hdma->ChannelIndex >> 2) <= 3)
    {
        hdma->DmaBaseAddress->CSELR1 &= ~(DMA_CSELR_C1S << ((hdma->ChannelIndex & 0x1cU) << 1));
    }
    else
    {
        uint8_t index;
        index = hdma->ChannelIndex >> 2;
        index = (index - 4) & 3;
        hdma->DmaBaseAddress->CSELR2 &= ~(DMA_CSELR_C1S << (index * DMAC_CSELR1_C2S_Pos));
    }


    /* Clean callbacks */
    hdma->XferCpltCallback = NULL;
    hdma->XferHalfCpltCallback = NULL;
    hdma->XferErrorCallback = NULL;
    hdma->XferAbortCallback = NULL;

    /* Initialise the error code */
    hdma->ErrorCode = HAL_DMA_ERROR_NONE;

    /* Initialize the DMA state */
    hdma->State = HAL_DMA_STATE_RESET;

    /* Release Lock */
    __HAL_UNLOCK(hdma);

    return HAL_OK;
}

/**
  * @} DMA_Exported_Functions_Group1
  */

/** @defgroup DMA_Exported_Functions_Group2 Input and Output operation functions
 *  @brief   Input and Output operation functions
 *
@verbatim
 ===============================================================================
                      #####  IO operation functions  #####
 ===============================================================================
    [..]  This section provides functions allowing to:
      (+) Configure the source, destination address and data length and Start DMA transfer
      (+) Configure the source, destination address and data length and
          Start DMA transfer with interrupt
      (+) Abort DMA transfer
      (+) Poll for transfer complete
      (+) Handle DMA interrupt request

@endverbatim
  * @{
  */

/**
  * @brief  Start the DMA Transfer.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA Channel.
  * @param  SrcAddress The source memory Buffer address
  * @param  DstAddress The destination memory Buffer address
  * @param  Counts The counts of data transfer action
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_DMA_Start(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t Counts)
{
    HAL_StatusTypeDef status = HAL_OK;

    /* Check the parameters */
    HAL_ASSERT(IS_DMA_BUFFER_SIZE(Counts));

    /* Process locked */
    __HAL_LOCK(hdma);

    if (HAL_DMA_STATE_READY == hdma->State)
    {
        /* Change DMA peripheral state */
        hdma->State = HAL_DMA_STATE_BUSY;
        hdma->ErrorCode = HAL_DMA_ERROR_NONE;

        /* Disable the peripheral */
        __HAL_DMA_DISABLE(hdma);

        /* Configure the source, destination address and the data length & clear flags*/
        DMA_SetConfig(hdma, SrcAddress, DstAddress, Counts);


        mpu_dcache_clean((void *)SrcAddress, DMA_GetSrcBytes(hdma, Counts));
        /* Enable the Peripheral */
        __HAL_DMA_ENABLE(hdma);
    }
    else
    {
        /* Process Unlocked */
        __HAL_UNLOCK(hdma);
        status = HAL_BUSY;
    }
    return status;
}

/**
  * @brief  Start the DMA Transfer with interrupt enabled.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA Channel.
  * @param  SrcAddress The source memory Buffer address
  * @param  DstAddress The destination memory Buffer address
  * @param  Counts The counts of data transfer action
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_DMA_Start_IT(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t Counts)
{
    HAL_StatusTypeDef status = HAL_OK;

    /* Check the parameters */
    HAL_ASSERT(IS_DMA_BUFFER_SIZE(Counts));

    /* Process locked */
    __HAL_LOCK(hdma);

    if (HAL_DMA_STATE_READY == hdma->State)
    {
        /* Change DMA peripheral state */
        hdma->State = HAL_DMA_STATE_BUSY;
        hdma->ErrorCode = HAL_DMA_ERROR_NONE;

        /* Disable the peripheral */
        __HAL_DMA_DISABLE(hdma);

        /* Configure the source, destination address and the data length & clear flags*/
        DMA_SetConfig(hdma, SrcAddress, DstAddress, Counts);

        /* Enable the transfer complete interrupt */
        /* Enable the transfer Error interrupt */
        if (NULL != hdma->XferHalfCpltCallback)
        {
            /* Enable the Half transfer complete interrupt as well */
            __HAL_DMA_ENABLE_IT(hdma, (DMA_IT_TC | DMA_IT_HT | DMA_IT_TE));
        }
        else
        {
            __HAL_DMA_DISABLE_IT(hdma, DMA_IT_HT);
            __HAL_DMA_ENABLE_IT(hdma, (DMA_IT_TC | DMA_IT_TE));
        }

        mpu_dcache_clean((void *)SrcAddress, DMA_GetSrcBytes(hdma, Counts));

        /* Enable the Peripheral */
        __HAL_DMA_ENABLE(hdma);
    }
    else
    {
        /* Process Unlocked */
        __HAL_UNLOCK(hdma);

        /* Remain BUSY */
        status = HAL_BUSY;
    }
    return status;
}

/**
  * @brief  Abort the DMA Transfer.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA Channel.
    * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_DMA_Abort(DMA_HandleTypeDef *hdma)
{
    HAL_StatusTypeDef status = HAL_OK;

    /* Check the DMA peripheral handle */
    if (NULL == hdma)
    {
        return HAL_ERROR;
    }

    /* Disable DMA IT */
    __HAL_DMA_DISABLE_IT(hdma, (DMA_IT_TC | DMA_IT_HT | DMA_IT_TE));


    /* Disable the channel */
    __HAL_DMA_DISABLE(hdma);

    /* Clear all flags */
    hdma->DmaBaseAddress->IFCR = (DMA_ISR_GIF1 << (hdma->ChannelIndex & 0x1cU));

    /* Change the DMA state */
    hdma->State = HAL_DMA_STATE_READY;

    /* Process Unlocked */
    __HAL_UNLOCK(hdma);

    return status;
}

/**
  * @brief  Aborts the DMA Transfer in Interrupt mode.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *                 the configuration information for the specified DMA Channel.
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_DMA_Abort_IT(DMA_HandleTypeDef *hdma)
{
    HAL_StatusTypeDef status = HAL_OK;

    if (HAL_DMA_STATE_BUSY != hdma->State)
    {
        /* no transfer ongoing */
        hdma->ErrorCode = HAL_DMA_ERROR_NO_XFER;

        status = HAL_ERROR;
    }
    else
    {
        /* Disable DMA IT */
        __HAL_DMA_DISABLE_IT(hdma, (DMA_IT_TC | DMA_IT_HT | DMA_IT_TE));

        /* Disable the channel */
        __HAL_DMA_DISABLE(hdma);


        /* Clear all flags */
        hdma->DmaBaseAddress->IFCR = (DMA_ISR_GIF1 << (hdma->ChannelIndex & 0x1cU));

        /* Change the DMA state */
        hdma->State = HAL_DMA_STATE_READY;

        /* Process Unlocked */
        __HAL_UNLOCK(hdma);

        /* Call User Abort callback */
        if (hdma->XferAbortCallback != NULL)
        {
            hdma->XferAbortCallback(hdma);
        }
    }
    return status;
}

/**
  * @brief  Polling for transfer complete.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *                  the configuration information for the specified DMA Channel.
  * @param  CompleteLevel Specifies the DMA level complete.
  * @param  Timeout       Timeout duration.
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_DMA_PollForTransfer(DMA_HandleTypeDef *hdma, HAL_DMA_LevelCompleteTypeDef CompleteLevel, uint32_t Timeout)
{
    uint32_t temp;
    uint32_t tickstart;

    if (HAL_DMA_STATE_BUSY != hdma->State)
    {
        /* no transfer ongoing */
        hdma->ErrorCode = HAL_DMA_ERROR_NO_XFER;
        __HAL_UNLOCK(hdma);
        return HAL_ERROR;
    }

    /* Polling mode not supported in circular mode */
    if (0U != (hdma->Instance->CCR & DMA_CCR_CIRC))
    {
        hdma->ErrorCode = HAL_DMA_ERROR_NOT_SUPPORTED;
        return HAL_ERROR;
    }

    /* Get the level transfer complete flag */
    if (HAL_DMA_FULL_TRANSFER == CompleteLevel)
    {
        /* Transfer Complete flag */
        temp = DMA_FLAG_TC1 << (hdma->ChannelIndex & 0x1cU);
    }
    else
    {
        /* Half Transfer Complete flag */
        temp = DMA_FLAG_HT1 << (hdma->ChannelIndex & 0x1cU);
    }

    /* Get tick */
    tickstart = HAL_GetTick();

    while (0U == (hdma->DmaBaseAddress->ISR & temp))
    {
        if ((0U != (hdma->DmaBaseAddress->ISR & (DMA_FLAG_TE1 << (hdma->ChannelIndex & 0x1cU)))))
        {
            /* When a DMA transfer error occurs */
            /* A hardware clear of its EN bits is performed */
            /* Clear all flags */
            hdma->DmaBaseAddress->IFCR = (DMA_ISR_GIF1 << (hdma->ChannelIndex & 0x1cU));

            /* Update error code */
            hdma->ErrorCode = HAL_DMA_ERROR_TE;

            /* Change the DMA state */
            hdma->State = HAL_DMA_STATE_READY;

            /* Process Unlocked */
            __HAL_UNLOCK(hdma);

            return HAL_ERROR;
        }
        /* Check for the Timeout */
        if (Timeout != HAL_MAX_DELAY)
        {
            if (((HAL_GetTick() - tickstart) > Timeout) || (Timeout == 0U))
            {
                /* Update error code */
                hdma->ErrorCode = HAL_DMA_ERROR_TIMEOUT;

                /* Change the DMA state */
                hdma->State = HAL_DMA_STATE_READY;

                /* Process Unlocked */
                __HAL_UNLOCK(hdma);

                return HAL_ERROR;
            }
        }
    }


    if (HAL_DMA_FULL_TRANSFER == CompleteLevel)
    {
        /* Clear the transfer complete flag */
        hdma->DmaBaseAddress->IFCR = (DMA_FLAG_TC1 << (hdma->ChannelIndex & 0x1cU));

        /* The selected Channelx EN bit is cleared (DMA is disabled and
        all transfers are complete) */
        hdma->State = HAL_DMA_STATE_READY;
    }
    else
    {
        /* Clear the half transfer complete flag */
        hdma->DmaBaseAddress->IFCR = (DMA_FLAG_HT1 << (hdma->ChannelIndex & 0x1cU));
    }

    /* Process unlocked */
    __HAL_UNLOCK(hdma);

    return HAL_OK;
}

/**
  * @brief  Handle DMA interrupt request.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA Channel.
  * @retval None
  */
__HAL_ROM_USED void HAL_DMA_IRQHandler(DMA_HandleTypeDef *hdma)
{
    uint32_t flag_it = hdma->DmaBaseAddress->ISR;
    uint32_t source_it = hdma->Instance->CCR;

    /* Half Transfer Complete Interrupt management ******************************/
    if ((0U != (flag_it & (DMA_FLAG_HT1 << (hdma->ChannelIndex & 0x1cU)))) && (0U != (source_it & DMA_IT_HT)))
    {
        /* Disable the half transfer interrupt if the DMA mode is not CIRCULAR */
        if ((hdma->Instance->CCR & DMA_CCR_CIRC) == 0U)
        {
            /* Disable the half transfer interrupt */
            __HAL_DMA_DISABLE_IT(hdma, DMA_IT_HT);
        }
        /* Clear the half transfer complete flag */
        hdma->DmaBaseAddress->IFCR = DMA_ISR_HTIF1 << (hdma->ChannelIndex & 0x1cU);

        /* DMA peripheral state is not updated in Half Transfer */
        /* but in Transfer Complete case */

        if (hdma->XferHalfCpltCallback != NULL)
        {
            /* Half transfer callback */
            hdma->XferHalfCpltCallback(hdma);
        }
    }

    /* Transfer Complete Interrupt management ***********************************/
    else if ((0U != (flag_it & (DMA_FLAG_TC1 << (hdma->ChannelIndex & 0x1cU)))) && (0U != (source_it & DMA_IT_TC)))
    {
        if ((hdma->Instance->CCR & DMA_CCR_CIRC) == 0U)
        {
            /* Disable the transfer complete and error interrupt */
            __HAL_DMA_DISABLE_IT(hdma, DMA_IT_TE | DMA_IT_TC);

            /* Change the DMA state */
            hdma->State = HAL_DMA_STATE_READY;
        }
        /* Clear the transfer complete flag */
        hdma->DmaBaseAddress->IFCR = (DMA_ISR_TCIF1 << (hdma->ChannelIndex & 0x1cU));

        /* Process Unlocked */
        __HAL_UNLOCK(hdma);

        if (hdma->XferCpltCallback != NULL)
        {
            /* Transfer complete callback */
            hdma->XferCpltCallback(hdma);
        }
    }

    /* Transfer Error Interrupt management **************************************/
    else if ((0U != (flag_it & (DMA_FLAG_TE1 << (hdma->ChannelIndex & 0x1cU)))) && (0U != (source_it & DMA_IT_TE)))
    {
        /* When a DMA transfer error occurs */
        /* A hardware clear of its EN bits is performed */
        /* Disable ALL DMA IT */
        __HAL_DMA_DISABLE_IT(hdma, (DMA_IT_TC | DMA_IT_HT | DMA_IT_TE));

        /* Clear all flags */
        hdma->DmaBaseAddress->IFCR = (DMA_ISR_GIF1 << (hdma->ChannelIndex & 0x1cU));

        /* Update error code */
        hdma->ErrorCode = HAL_DMA_ERROR_TE;

        /* Change the DMA state */
        hdma->State = HAL_DMA_STATE_READY;

        /* Process Unlocked */
        __HAL_UNLOCK(hdma);

        if (hdma->XferErrorCallback != NULL)
        {
            /* Transfer error callback */
            hdma->XferErrorCallback(hdma);
        }
    }
    else
    {
        /* Nothing To Do */
    }
    return;
}

/**
  * @brief  Register callbacks
  * @param  hdma                 pointer to a DMA_HandleTypeDef structure that contains
  *                               the configuration information for the specified DMA Channel.
  * @param  CallbackID           User Callback identifer
  *                               a HAL_DMA_CallbackIDTypeDef ENUM as parameter.
  * @param  pCallback            pointer to private callbacsk function which has pointer to
  *                               a DMA_HandleTypeDef structure as parameter.
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_DMA_RegisterCallback(DMA_HandleTypeDef *hdma, HAL_DMA_CallbackIDTypeDef CallbackID, void (* pCallback)(DMA_HandleTypeDef *_hdma))
{
    HAL_StatusTypeDef status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(hdma);

    if (HAL_DMA_STATE_READY == hdma->State)
    {
        switch (CallbackID)
        {
        case  HAL_DMA_XFER_CPLT_CB_ID:
            hdma->XferCpltCallback = pCallback;
            break;

        case  HAL_DMA_XFER_HALFCPLT_CB_ID:
            hdma->XferHalfCpltCallback = pCallback;
            break;

        case  HAL_DMA_XFER_ERROR_CB_ID:
            hdma->XferErrorCallback = pCallback;
            break;

        case  HAL_DMA_XFER_ABORT_CB_ID:
            hdma->XferAbortCallback = pCallback;
            break;

        default:
            status = HAL_ERROR;
            break;
        }
    }
    else
    {
        status = HAL_ERROR;
    }

    /* Release Lock */
    __HAL_UNLOCK(hdma);

    return status;
}

/**
  * @brief  UnRegister callbacks
  * @param  hdma                 pointer to a DMA_HandleTypeDef structure that contains
  *                               the configuration information for the specified DMA Channel.
  * @param  CallbackID           User Callback identifer
  *                               a HAL_DMA_CallbackIDTypeDef ENUM as parameter.
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_DMA_UnRegisterCallback(DMA_HandleTypeDef *hdma, HAL_DMA_CallbackIDTypeDef CallbackID)
{
    HAL_StatusTypeDef status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(hdma);

    if (HAL_DMA_STATE_READY == hdma->State)
    {
        switch (CallbackID)
        {
        case  HAL_DMA_XFER_CPLT_CB_ID:
            hdma->XferCpltCallback = NULL;
            break;

        case  HAL_DMA_XFER_HALFCPLT_CB_ID:
            hdma->XferHalfCpltCallback = NULL;
            break;

        case  HAL_DMA_XFER_ERROR_CB_ID:
            hdma->XferErrorCallback = NULL;
            break;

        case  HAL_DMA_XFER_ABORT_CB_ID:
            hdma->XferAbortCallback = NULL;
            break;

        case   HAL_DMA_XFER_ALL_CB_ID:
            hdma->XferCpltCallback = NULL;
            hdma->XferHalfCpltCallback = NULL;
            hdma->XferErrorCallback = NULL;
            hdma->XferAbortCallback = NULL;
            break;

        default:
            status = HAL_ERROR;
            break;
        }
    }
    else
    {
        status = HAL_ERROR;
    }

    /* Release Lock */
    __HAL_UNLOCK(hdma);

    return status;
}

/**
  * @} DMA_Exported_Functions_Group2
  */



/** @defgroup DMA_Exported_Functions_Group3 Peripheral State and Errors functions
 *  @brief    Peripheral State and Errors functions
 *
@verbatim
 ===============================================================================
            ##### Peripheral State and Errors functions #####
 ===============================================================================
    [..]
    This subsection provides functions allowing to
      (+) Check the DMA state
      (+) Get error code

@endverbatim
  * @{
  */

/**
  * @brief  Return the DMA hande state.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA Channel.
  * @retval HAL state
  */
__HAL_ROM_USED HAL_DMA_StateTypeDef HAL_DMA_GetState(DMA_HandleTypeDef *hdma)
{
    /* Return DMA handle state */
    return hdma->State;
}

/**
  * @brief  Return the DMA error code.
  * @param  hdma : pointer to a DMA_HandleTypeDef structure that contains
  *              the configuration information for the specified DMA Channel.
  * @retval DMA Error Code
  */
__HAL_ROM_USED uint32_t HAL_DMA_GetError(DMA_HandleTypeDef *hdma)
{
    return hdma->ErrorCode;
}

/**
  * @} DMA_Exported_Functions_Group3
  */

/**
  * @} DMA_Exported_Functions
  */

/** @addtogroup DMA_Private_Functions
  * @{
  */

/**
  * @brief  Remap memory address for DMA
            In A0, When HCPU used DMA2(Implemented in LCPU), DMA2 is using LCPU RAM space, Need to map HCPU RAM address to LCPU RAM address.
  * @param  hdma       pointer to a DMA_HandleTypeDef structure that contains
  *                     the configuration information for the specified DMA Channel.
  * @param  SrcAddress The source memory Buffer address
  * @param  DstAddress The destination memory Buffer address
  * @retval None
  */
void DMA_Remap(DMA_HandleTypeDef *hdma, uint32_t *SrcAddress, uint32_t *DstAddress)
{
#if defined(SF32LB58X)
    if (hdma->DmaBaseAddress == hwp_dmac3)
    {
        if (hdma->Init.Direction == DMA_MEMORY_TO_PERIPH || hdma->Init.Direction == DMA_MEMORY_TO_MEMORY)
            (*SrcAddress) = HCPU_ADDR_2_LCPU_ADDR(* SrcAddress);
        if (hdma->Init.Direction == DMA_PERIPH_TO_MEMORY || hdma->Init.Direction == DMA_MEMORY_TO_MEMORY)
            (*DstAddress) = HCPU_ADDR_2_LCPU_ADDR(* DstAddress);
    }
    /* All HPSYS MPI is accessed by SBUS address */
    if (HCPU_IS_MPI_CBUS_ADDR(*SrcAddress))
    {
        (*SrcAddress) = HCPU_MPI_SBUS_ADDR(*SrcAddress);
    }
#else
    if (hdma->DmaBaseAddress == hwp_dmac2)
    {
        if (hdma->Init.Direction == DMA_MEMORY_TO_PERIPH || hdma->Init.Direction == DMA_MEMORY_TO_MEMORY)
            (*SrcAddress) = HCPU_ADDR_2_LCPU_ADDR(* SrcAddress);
        if (hdma->Init.Direction == DMA_PERIPH_TO_MEMORY || hdma->Init.Direction == DMA_MEMORY_TO_MEMORY)
            (*DstAddress) = HCPU_ADDR_2_LCPU_ADDR(* DstAddress);
    }
#if defined(HCPU_IS_MPI_CBUS_ADDR)
    if (HCPU_IS_MPI_CBUS_ADDR(*SrcAddress))
    {
        (*SrcAddress) = HCPU_MPI_SBUS_ADDR(*SrcAddress);
    }
#endif /* HCPU_IS_MPI_CBUS_ADDR */
#endif /* SF32LB58X */
}


/**
  * @brief  Sets the DMA Transfer parameter.
  * @param  hdma       pointer to a DMA_HandleTypeDef structure that contains
  *                     the configuration information for the specified DMA Channel.
  * @param  SrcAddress The source memory Buffer address
  * @param  DstAddress The destination memory Buffer address
  * @param  Counts The counts of data transfer action
  * @retval HAL status
  */
static void DMA_SetConfig(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t Counts)
{
    /* Clear all flags */
    hdma->DmaBaseAddress->IFCR = (DMA_ISR_GIF1 << (hdma->ChannelIndex & 0x1cU));

    /* Configure DMA Channel data length */
    hdma->Instance->CNDTR = Counts;

    DMA_Remap(hdma, &SrcAddress, &DstAddress);

    /* Memory to Peripheral */
    if ((hdma->Init.Direction) == DMA_MEMORY_TO_PERIPH)
    {
        /* Configure DMA Channel destination address */
        hdma->Instance->CPAR = DstAddress;

        /* Configure DMA Channel source address */
        hdma->Instance->CM0AR = SrcAddress;
    }
    /* Peripheral to Memory */
    else
    {
        /* Configure DMA Channel source address */
        hdma->Instance->CPAR = SrcAddress;

        /* Configure DMA Channel destination address */
        hdma->Instance->CM0AR = DstAddress;
    }

#if defined(SF32LB55X)||defined(SF32LB58X)||defined(SF32LB56X)
    {
        uint8_t accross_1M = 0;
        switch (hdma->Init.Direction)
        {
        case DMA_MEMORY_TO_PERIPH:
            if (IS_DMA_ACCROSS_1M_BOUNDARY(SrcAddress, Counts))
                accross_1M = 1;
            break;
        case DMA_PERIPH_TO_MEMORY:
            if (IS_DMA_ACCROSS_1M_BOUNDARY(DstAddress, Counts))
                accross_1M = 1;
            break;
        case DMA_MEMORY_TO_MEMORY:
            if (IS_DMA_ACCROSS_1M_BOUNDARY(SrcAddress, Counts) || IS_DMA_ACCROSS_1M_BOUNDARY(DstAddress, Counts))
                accross_1M = 1;
            break;
        default:
            break;
        }
        HAL_ASSERT(accross_1M == 0);
    }
#endif

}

static uint32_t DMA_GetSrcBytes(DMA_HandleTypeDef *hdma, uint32_t Counts)
{
    uint32_t bytes;
    switch (hdma->Init.PeriphDataAlignment)
    {
    case DMA_PDATAALIGN_BYTE:
        bytes = 1;
        break;
    case DMA_PDATAALIGN_HALFWORD:
        bytes = 2;
        break;
    case DMA_PDATAALIGN_WORD:
        bytes = 4;
        break;
    default:
        bytes = 0;
        HAL_ASSERT(0);
        break;
    }

    if (DMA_PINC_ENABLE == hdma->Init.PeriphInc)
    {
        return bytes * Counts;
    }
    else
    {
        return bytes;
    }
}
/**
  * @brief  Sets the DMA request source
  * @param  dma_chl     DMA request channel
  * @param  request     request number
  */
__HAL_ROM_USED void HAL_DMA_Select_Source(DMA_Channel_TypeDef *dma_chl, uint8_t request)
{
    DMAC_TypeDef *t = NULL;
    uint32_t temp = (uint32_t)dma_chl;
    uint32_t channel;

    t = (DMAC_TypeDef *)(temp & 0xfffff000);
    temp = (uint32_t) & (t->CCR1);
    channel = ((uint32_t)dma_chl - temp) / sizeof(DMA_Channel_TypeDef);
    if (request <= DMA_CSELR_C1S)
    {
        if (channel < 4)
        {
            t->CSELR1 |= (request << (channel * DMAC_CSELR1_C2S_Pos));
        }
        else if (channel < MAX_DMA_CHANNEL)
        {
            t->CSELR1 |= request << ((channel - 4) * DMAC_CSELR1_C2S_Pos);
        }
        else
        {
            HAL_ASSERT(0);
        }
    }
}


/**
  * @} DMA_Private_Functions
  */

#endif /* HAL_DMA_MODULE_ENABLED */
/**
  * @} DMA
  */

/**
  * @} BF0_HAL_Driver
  */

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
