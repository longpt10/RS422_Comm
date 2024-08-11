/*
 * rs458_comm.c
 *
 *  Created on: May 30, 2024
 *      Author: Administrator
 */

//! \file   rs485_comm.c
//! \brief  These functions define for communication of device
//!

// **************************************************************************
// the includes
#include "rs422_comm_sci.h"

// **************************************************************************
// the global

// **************************************************************************
// the functions
RS422_COMM_SCI_Handle RS422_COMM_SCI_init(void *pMemory, const size_t numBytes)
{
    RS422_COMM_SCI_Handle handle;

    if(numBytes < sizeof(RS422_COMM_SCI_Obj))
    {
        return((RS422_COMM_SCI_Handle)NULL);
    }
    // assign the handle
    handle = (RS422_COMM_SCI_Handle)pMemory;

    RS422_COMM_SCI_Obj *obj = (RS422_COMM_SCI_Obj*)handle;

    obj->sciHandle  = RS422_SCI_BASE;
    obj->timerHandle = CPUTIMER0_BASE;
    obj->baudrate    = 115200;
    memset(obj->txBuff, 0, sizeof(obj->txBuff));
    memset(obj->rxBuff, 0, sizeof(obj->rxBuff));
    obj->txTimeOutTicker = 0U;
    obj->rxTimeOutTicker = 0U;
    obj->rxTimeOutTickerFifo12 = 0U;
    obj->successPacket = 0U;
    obj->dropPacket = 0U;
    obj->txState = RS422_MSG_INFO_PUMP_TO_AP_OR_HMI;
    obj->rxState = RS422_RX_WAITING;
    obj->isCommWithHMIFlag = false;
    obj->flashAnsFlag      = false;
    obj->updateRxParamFlag = false;
    return handle;
}

//------------------------------------------------------------------------------
void RS422_COMM_SCI_setupGpio(RS422_COMM_SCI_Handle handle)
{
#ifdef DEBUG
    ASSERT(handle);
#endif
    GPIO_setPinConfig(DEVICE_GPIO_CFG_SCIRXDA);
    GPIO_setPadConfig(DEVICE_GPIO_PIN_SCIRXDA, GPIO_PIN_TYPE_STD | GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(DEVICE_GPIO_PIN_SCIRXDA, GPIO_QUAL_ASYNC);

    GPIO_setPinConfig(DEVICE_GPIO_CFG_SCITXDA);
    GPIO_setPadConfig(DEVICE_GPIO_PIN_SCITXDA, GPIO_PIN_TYPE_STD | GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(DEVICE_GPIO_PIN_SCITXDA, GPIO_QUAL_ASYNC);

    GPIO_setPinConfig(DEVICE_GPIO_CFG_LED1);
    GPIO_setDirectionMode(DEVICE_GPIO_PIN_LED1, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(DEVICE_GPIO_PIN_LED1, GPIO_PIN_TYPE_STD);


    GPIO_setPinConfig(DEVICE_GPIO_CFG_LED2);
    GPIO_setDirectionMode(DEVICE_GPIO_PIN_LED2, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(DEVICE_GPIO_PIN_LED2, GPIO_PIN_TYPE_STD);
}

//------------------------------------------------------------------------------
void RS422_COMM_SCI_setupTimer(RS422_COMM_SCI_Handle handle, uint32_t periodCount)
{
#ifdef DEBUG
    ASSERT(handle);
#endif
    RS422_COMM_SCI_Obj *obj = (RS422_COMM_SCI_Obj*)handle;
    CPUTimer_setPreScaler(obj->timerHandle, 0U);  // divide by 1 (SYSCLKOUT)
    CPUTimer_setPeriod(obj->timerHandle, periodCount);
    CPUTimer_stopTimer(obj->timerHandle);                // Stop timer / reload / restart
    CPUTimer_setEmulationMode(obj->timerHandle, CPUTIMER_EMULATIONMODE_STOPAFTERNEXTDECREMENT);
    CPUTimer_reloadTimerCounter(obj->timerHandle);       // Reload counter with period value
    CPUTimer_resumeTimer(obj->timerHandle);
}

//------------------------------------------------------------------------------
void RS422_COMM_SCI_setup(RS422_COMM_SCI_Handle handle)
{
#ifdef DEBUG
    ASSERT(handle);
#endif
    RS422_COMM_SCI_Obj *obj = (RS422_COMM_SCI_Obj*)handle;

    // Initialize SCIC and its FIFO.
    SCI_performSoftwareReset(obj->sciHandle);

    // Configure SCIC for echoback.
    SCI_setConfig(obj->sciHandle, DEVICE_LSPCLK_FREQ, obj->baudrate,
                        ( SCI_CONFIG_WLEN_8 |
                          SCI_CONFIG_STOP_ONE |
                          SCI_CONFIG_PAR_NONE ) );

    SCI_enableModule(obj->sciHandle);
    SCI_resetChannels(obj->sciHandle);
    SCI_enableFIFO(obj->sciHandle);

    // RX and TX FIFO Interrupts Enabled
    SCI_enableInterrupt(obj->sciHandle, (SCI_INT_RXFF | SCI_INT_TXFF));
    SCI_disableInterrupt(obj->sciHandle, SCI_INT_RXERR);

    SCI_clearInterruptStatus(obj->sciHandle, SCI_INT_TXFF | SCI_INT_RXFF);

//     The transmit FIFO generates an interrupt when FIFO status
//     bits are less than or equal to 2 out of 16 words
//     The receive FIFO generates an interrupt when FIFO status
//     bits are greater than equal to 2 out of 16 words

    SCI_setFIFOInterruptLevel(obj->sciHandle, SCI_FIFO_TX13, SCI_FIFO_RX13);

    SCI_performSoftwareReset(obj->sciHandle);
    SCI_resetRxFIFO(obj->sciHandle);
    SCI_resetTxFIFO(obj->sciHandle);

}

//------------------------------------------------------------------------------
void RS422_COMM_SCI_serialize(RS422_COMM_SCI_Handle handle)
{
#ifdef DEBUG
    ASSERT(handle);
#endif

}

//------------------------------------------------------------------------------
void RS422_COMM_SCI_write(RS422_COMM_SCI_Handle handle)
{
#ifdef DEBUG
    ASSERT(handle);
#endif
    RS422_COMM_SCI_Obj *obj = (RS422_COMM_SCI_Obj*)handle;
    uint16_t i;
    obj->txTimeOutTicker++;
    if(SCI_getTxFIFOStatus(obj->sciHandle) == SCI_FIFO_TX0)
    {
        switch (obj->txState)
        {
            case RS422_MSG_INFO_PUMP_TO_AP_OR_HMI:
            {
                for(i = 0; i < 8; i++)
                {
                  SCI_writeCharNonBlocking(obj->sciHandle, obj->txBuff[i]);
                }
                if(obj->isCommWithHMIFlag)
                {
                    obj->txState = RS422_MSG_INFO_PUMP_TO_HMI;
                }
            }
                break;
            case RS422_MSG_INFO_PUMP_TO_HMI:
            {
                for(i = 0; i < 14; i++)
                {
                    SCI_writeCharNonBlocking(obj->sciHandle, obj->txBuff[i]);
                }
                if(obj->isCommWithHMIFlag)
                {
                    if(obj->flashAnsFlag)
                    {
                        obj->txState = RS422_MSG_INFO_FLASH_PUMP_TO_HMI;
                    }
                    else
                    {
                        obj->txState = RS422_MSG_INFO_PUMP_TO_AP_OR_HMI;
                    }
                }
            }
                break;
            case RS422_MSG_INFO_FLASH_PUMP_TO_HMI:
            {
                for(i = 0; i < 13; i++)
                {
                   SCI_writeCharNonBlocking(obj->sciHandle, obj->txBuff[i]);
                }
                obj->flashAnsFlag = false;
                obj->txState = RS422_MSG_INFO_PUMP_TO_AP_OR_HMI;
            }
                break;
            default:
                break;
        }

        SCI_clearInterruptStatus(obj->sciHandle, SCI_INT_TXFF);
        obj->txTimeOutTicker = 0U;
    }
    else if(obj->txTimeOutTicker > 20000U)
    {
        obj->txTimeOutTicker = 0;
        SCI_resetTxFIFO(obj->sciHandle);
    }
}

//------------------------------------------------------------------------------
void RS422_COMM_SCI_read(RS422_COMM_SCI_Handle handle)
{
#ifdef DEBUG
    ASSERT(handle);
#endif
    RS422_COMM_SCI_Obj *obj = (RS422_COMM_SCI_Obj*)handle;
    obj->rxTimeOutTicker++;
    uint16_t i;
    if(SCI_getRxFIFOStatus(obj->sciHandle) != SCI_FIFO_RX0)
    {
        obj->rxTimeOutTicker = 0U;

        if(SCI_getRxFIFOStatus(obj->sciHandle) == SCI_FIFO_RX13)
        {
            SCI_clearOverflowStatus(obj->sciHandle);
            SCI_clearInterruptStatus(obj->sciHandle, SCI_INT_RXFF);
        }
        else
        {
            obj->rxTimeOutTickerFifo12++;
        }
        if(obj->rxTimeOutTickerFifo12 > 10) // wait timeout 10ms
        {
            obj->rxTimeOutTickerFifo12 = 0U;
            if(SCI_getRxFIFOStatus(obj->sciHandle) == SCI_FIFO_RX5)
            {
                obj->crcRx = 0U;
                // For better performance should not be use with for loop
                for(i = 0; i < 5; i++)
                {
                    obj->rxBuff[i] = SCI_readCharBlockingFIFO(obj->sciHandle);
                    if(i < 4)
                    {
                        obj->crcRx += obj->rxBuff[i];
                    }
                }
                obj->crcRx &= 0x00FF;
                if(obj->crcRx == obj->rxBuff[4])
                {
                    obj->rxState = RS422_RX_SUCCESS;
                    obj->successPacket++;
                    obj->updateRxParamFlag = true;
                }
                else
                {
                    obj->rxState = RS422_RX_FAILED;
                    obj->dropPacket++;
                    obj->updateRxParamFlag = false;
                }
                SCI_clearOverflowStatus(obj->sciHandle);
                SCI_clearInterruptStatus(obj->sciHandle, SCI_INT_RXFF);
            }
        }
    }

//    if(obj->rxTimeOutTicker > 5000) // 3s
//    {
//        obj->rxTimeOutTicker = 0U;
//        // Reset the RX
//        SCI_resetRxFIFO(obj->sciHandle);
//        SCI_enableFIFO(obj->sciHandle);
//    }

}


