#include "driverlib.h"
#include "device.h"
#include "rs422_comm_sci.h"



#define SYSTEM_FREQUENCY            (DEVICE_SYSCLK_FREQ / 1000000U)     // 200 Mhz
#define MICROSEC                    (SYSTEM_FREQUENCY)
#define MILLISEC                    (1000U*MICROSEC)

//
// Globals
//
RS422_COMM_SCI_Handle rs422Handle;
RS422_COMM_SCI_Obj    rs422Obj;

typedef struct
{
    uint32_t tickerTask1;
    uint32_t tickerTask2;
    uint32_t tickerTask3;
    uint16_t timeBaseTask1;
    uint16_t timeBaseTask2;
    uint16_t timeBaseTask3;
}SYSTEM_TIME_t;

void task1(void *eventData);
void updateTxBuff(RS422_COMM_SCI_Handle handle);
void updateRxParam(RS422_COMM_SCI_Handle handle);
float32_t speedFbk = 0.0f;
float32_t speedRef = 0.0f;
float32_t iRms = 0.5;

SYSTEM_TIME_t sysTime;
//
// Main
//
void main(void)
{
    //
    // Initialize device clock and peripherals
    //
    Device_init();

    //
    // Setup GPIO by disabling pin locks and enabling pullups
    //
    Device_initGPIO();

    //
    // Initialize PIE and clear PIE registers. Disables CPU interrupts.
    //
    Interrupt_initModule();

    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    //
    Interrupt_initVectorTable();

    //
    // Board Initialization
    //
    //Board_init();
    rs422Handle = RS422_COMM_SCI_init(&rs422Obj, sizeof(rs422Obj));
    RS422_COMM_SCI_setupGpio(rs422Handle);
    RS422_COMM_SCI_setup(rs422Handle);
    RS422_COMM_SCI_setupTimer(rs422Handle, MILLISEC);
    //
    // Enables CPU interrupts
    //
    Interrupt_enableMaster();


    //
    // Send Characters forever starting with 0x00 and going through 0xFF.
    // After sending each, check the receive buffer for the correct value.
    //
    while(1)
    {
        // Super loop in here
        if(CPUTimer_getTimerOverflowStatus(rs422Handle->timerHandle))
        {
            CPUTimer_clearOverflowFlag(rs422Handle->timerHandle);
            task1(NULL);

            // 1ms
            RS422_COMM_SCI_read(rs422Handle);
            updateRxParam(rs422Handle);
        }

    }
}
void updateTxBuff(RS422_COMM_SCI_Handle handle)
{
#ifdef DEBUG
    ASSERT(handle);
#endif
    RS422_COMM_SCI_Obj *obj = (RS422_COMM_SCI_Obj*)handle;
    uint16_t temp;
    uint16_t i;
    switch (obj->txState)
    {
        case RS422_MSG_INFO_PUMP_TO_AP_OR_HMI:
        {
            obj->crcTx = 0U;
            for(i = 0; i < 16; i++)
            {
                obj->txBuff[i] = 0;
            }
            obj->txBuff[0] = 0x00FE & 0x00FF; // header 1
            obj->txBuff[1] = 0x00AA & 0x00FF; // header 2

            temp = (uint16_t) 1;
            obj->txBuff[2] = temp & 0x00FF;  // pump state

            speedRef = 9100.0f;
            temp = (uint16_t) (speedRef / 100.0f);
            obj->txBuff[3] = temp & 0x00FF; // pump speed in RPM
            speedFbk = 9100.0f;
            temp = (uint16_t) (speedFbk / 100.0f);
            obj->txBuff[4] = temp & 0x00FF; // pump speed fbk

            iRms = 0.5f;
            temp = (uint16_t) (iRms * 10.0f);
            obj->txBuff[5] = temp & 0x00FF; // pump irms

            obj->txBuff[6] = 0; // temperature and position

            for(i = 0; i < 7; i++)
            {
                obj->crcTx += obj->txBuff[i];
            }

            temp = obj->crcTx;
            obj->txBuff[7] = temp & 0x00FF;
        }
            break;
        case RS422_MSG_INFO_PUMP_TO_HMI:
        {
            obj->crcTx = 0U;
            for(i = 0; i < 16; i++)
            {
             obj->txBuff[i] = 0;
            }
            obj->txBuff[0] = 0x00F0 & 0x00FF; // Header 1
            obj->txBuff[1] = 0x00AA & 0x00FF; // header 2

            temp = (uint16_t) (12.5f * 10.0f);
            obj->txBuff[2] = temp & 0x00FF;  // va

            temp = (uint16_t) (12.5f * 10.0f);
            obj->txBuff[3] = temp & 0x00FF;  //vb

            temp = (uint16_t) (12.5f * 10.0f);
            obj->txBuff[4] = temp & 0x00FF; // vc

            temp = (uint16_t) (0.52f * 100.0f);
            obj->txBuff[5] = temp & 0x00FF; // i offset A

            temp = (uint16_t) (0.49f * 100.0f);
            obj->txBuff[6] = temp & 0x00FF; // i offset B

            temp = (uint16_t) (0.51f * 100.0f);
            obj->txBuff[7] = temp & 0x00FF; // i offset C

            temp = (uint16_t) (24.0f * 100.0f);
            obj->txBuff[8] = temp & 0x00FF; // vdc bus

            temp = (uint16_t) 3;
            obj->txBuff[9] = temp & 0x00FF; // hall state sensor

            obj->txBuff[10] = 0; // fault bit

            obj->txBuff[11] = 0; // reserve

            for(i = 0; i < 12; i++)
            {
                obj->crcTx += obj->txBuff[i];
            }
            temp = obj->crcTx;
            obj->txBuff[12] = temp & 0x00FF;

        }
            break;
        case RS422_MSG_INFO_FLASH_PUMP_TO_HMI:
        {
            obj->crcTx = 0U;
            for(i = 0; i < 16; i++)
            {
                obj->txBuff[i] = 0;
            }
        }
            break;
        default:
            break;
    }
}
void updateRxParam(RS422_COMM_SCI_Handle handle)
{
#ifdef DEBUG
    ASSERT(handle);
#endif
        RS422_COMM_SCI_Obj *obj = (RS422_COMM_SCI_Obj*)handle;

        if((obj->rxState == RS422_RX_SUCCESS) && (obj->updateRxParamFlag == true))
        {
            GPIO_togglePin(DEVICE_GPIO_PIN_LED2);
            if(((obj->rxBuff[0] & 0x00FF) == 0x00FE) && ((obj->rxBuff[1] & 0x00FF) == 0x00AA))
            {
                if(((obj->rxBuff[2] & 0x00FF) == 0x00A0) && ((obj->rxBuff[3] & 0x00FF) == 0x00A0))
                {
                    obj->isCommWithHMIFlag = true;
                }
                else
                {

                }
            }
                obj->rxState = RS422_RX_WAITING;
                obj->updateRxParamFlag = false;
          }
}

void task1(void *eventData)
{
#ifdef DEBUG
    ASSERT(rs422Handle);
#endif
        sysTime.timeBaseTask1++;
        // update the tx buff
        updateTxBuff(rs422Handle);
        //
        if(sysTime.timeBaseTask1 > 1000)  // 1s
        {
            GPIO_togglePin(DEVICE_GPIO_PIN_LED1);
            RS422_COMM_SCI_write(rs422Handle);
            sysTime.timeBaseTask1 = 0;
        }
}
