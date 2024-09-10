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

typedef struct
{
   float32_t  iBandwidth;
   float32_t  rs;
   float32_t  ls;
   float32_t  polepairs;
   float32_t  maxSpeed;
   float32_t  kTorque;
   float32_t  fcLpfSpeed;
   float32_t  kpSpeed;
   float32_t  kiSpeed;
   float32_t  kdSpeed;
   float32_t  Uminmax;
}FLASH_DATA_t;

void task1(void *eventData);
void updateTxBuff(RS422_COMM_SCI_Handle handle);
void updateRxParam(RS422_COMM_SCI_Handle handle);
float32_t speedFbk = 9100.0f;
float32_t speedRef = 9100.0f;
float32_t dutyRef = 0.0;
uint16_t  state = 1;
float32_t iRms = 0.5;
float32_t temperature = 60.0f;;

SYSTEM_TIME_t sysTime;
FLASH_DATA_t  flashData;
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
    DEVICE_DELAY_US(1000000);
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

    flashData.iBandwidth = 3000.0f;
    flashData.rs = 0.027f; // Ohm
    flashData.ls = 0.039f; // mH
    flashData.polepairs = 1.0f;
    flashData.maxSpeed = 12000.0f; // RMP
    flashData.kTorque = 8.3f;
    flashData.fcLpfSpeed = 200.0f;
    flashData.kpSpeed = 0.3f;
    flashData.kiSpeed = 0.1f;
    flashData.kdSpeed = 0.2f;
    flashData.Uminmax = 0.5f;

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

            temp = (uint16_t) state;
            obj->txBuff[2] = temp & 0x00FF;  // pump state


            temp = (uint16_t) (speedRef / 100.0f);
            obj->txBuff[3] = temp & 0x00FF; // pump speed in RPM

            temp = (uint16_t) (speedFbk / 100.0f);
            obj->txBuff[4] = temp & 0x00FF; // pump speed fbk

            temp = (uint16_t) (iRms * 10.0f);
            obj->txBuff[5] = temp & 0x00FF; // pump irms

            temp = (uint16_t) (temperature / 2.0f);
            temp &= 0x003F;
            temp |= (0 << 6);
            obj->txBuff[6] = temp & 0x00FF; // temperature and position

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
            obj->txBuff[12] = 0; // reserve 1

            for(i = 0; i < 13; i++)
            {
              obj->crcTx += obj->txBuff[i];
            }
            temp = obj->crcTx;
            obj->txBuff[13] = temp & 0x00FF;
        }
            break;
        case RS422_MSG_INFO_FLASH_PUMP_TO_HMI:
        {
            obj->crcTx = 0U;
            for(i = 0; i < 16; i++)
            {
             obj->txBuff[i] = 0;
            }
            obj->txBuff[0] = 0x00F2 & 0x00FF; // Header 1
            obj->txBuff[1] = 0x00AA & 0x00FF; // header 2

            temp = (uint16_t) (flashData.iBandwidth * 0.01f);
            obj->txBuff[2] = temp & 0x00FF;  // i bandwidth

            temp = (uint16_t) (flashData.rs * 1000.0f);
            obj->txBuff[3] = temp & 0x00FF;  // rs

            temp = (uint16_t) (flashData.ls * 1000.0f);
            obj->txBuff[4] = temp & 0x00FF; // ls

            temp = (uint16_t) (flashData.polepairs * 1.0f);
            obj->txBuff[5] = temp & 0x00FF; // pole pairs

            temp = (uint16_t) (flashData.maxSpeed * 0.001f);
            obj->txBuff[6] = temp & 0x00FF; // max speed

            temp = (uint16_t) (flashData.kTorque * 100.0f);
            obj->txBuff[7] = temp & 0x00FF; // k torque

            temp = (uint16_t) (flashData.fcLpfSpeed * 0.01f);
            obj->txBuff[8] = temp & 0x00FF; // fc filter

            temp = (uint16_t) (flashData.kpSpeed * 100.0f);
            obj->txBuff[9] = temp & 0x00FF; // Kp

            temp = (uint16_t) (flashData.kiSpeed * 100.0f);
            obj->txBuff[10] = temp & 0x00FF; // Ki

            temp = (uint16_t) (flashData.kdSpeed * 100.0f);
            obj->txBuff[11] = temp & 0x00FF; // Kd

            temp = (uint16_t) (flashData.Uminmax * 100.0f);
            obj->txBuff[12] = temp & 0x00FF; // Uminmax

            for(i = 0; i < 13; i++)
            {
                obj->crcTx += obj->txBuff[i];
            }
            temp = obj->crcTx;
            obj->txBuff[13] = temp & 0x00FF;

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
                          state  =    (obj->rxBuff[2] & 0x00FF);  // command
                          if( state == 2)
                          {
                              speedRef = (float32_t)(obj->rxBuff[3] & 0x00FF)*100.0f; // speed ref
                          }
                          else if(state == 3)
                          {
                              dutyRef = (float32_t)(obj->rxBuff[3] & 0x00FF)/100.0f;
                          }
                }
            }
            else if(((obj->rxBuff[0] & 0x00FF) == 0x00F1) && ((obj->rxBuff[1] & 0x00FF) == 0x00AA)) // write flash
            {
                flashData.iBandwidth = (float32_t)(obj->rxBuff[2] & 0x00FF)*100.0f;
                flashData.rs   = (float32_t)(obj->rxBuff[3] & 0x00FF)/1000.0f;
                flashData.ls   = (float32_t)(obj->rxBuff[4] & 0x00FF)/1000.0f;
                flashData.polepairs   = (float32_t)(obj->rxBuff[5] & 0x00FF);
                flashData.maxSpeed   = (float32_t)(obj->rxBuff[6] & 0x00FF)*1000.0f;
                flashData.kTorque   = (float32_t)(obj->rxBuff[7] & 0x00FF)/100.0f;
                flashData.fcLpfSpeed = (float32_t)(obj->rxBuff[8] & 0x00FF)*100.0f;
                flashData.kpSpeed = (float32_t)(obj->rxBuff[9] & 0x00FF)/100.0f;
                flashData.kiSpeed = (float32_t)(obj->rxBuff[10] & 0x00FF)/100.0f;
                flashData.kdSpeed = (float32_t)(obj->rxBuff[11] & 0x00FF)/100.0f;
                flashData.Uminmax = (float32_t)(obj->rxBuff[12] & 0x00FF)/100.0f;

            }
            else if(((obj->rxBuff[0] & 0x00FF) == 0x00F2) && ((obj->rxBuff[1] & 0x00FF) == 0x00AA)) // read flash
            {
                obj->flashAnsFlag = true;
            }
            else
            {

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
        if(sysTime.timeBaseTask1 > 50)  // 50ms
        {
            GPIO_togglePin(DEVICE_GPIO_PIN_LED1);
            RS422_COMM_SCI_write(rs422Handle);
            sysTime.timeBaseTask1 = 0;
        }
}
