/*
 * rs422_comm_sci.h
 *
 *  Created on: Jul 20, 2024
 *      Author: Administrator
 */

#ifndef COMMUNICATION_RS422_COMM_SCI_H_
#define COMMUNICATION_RS422_COMM_SCI_H_

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
//! \defgroup RS485 RS485_OBJ
//! @{
//
//*****************************************************************************

//the includes
//drivers
#include "device.h"
#include "driverlib.h"
#include <string.h>

// **************************************************************************
// the defines
#define   RS422_SCI_BASE          SCIA_BASE

// **************************************************************************
// the typedefs
typedef enum
{
    RS422_MSG_INFO_PUMP_TO_AP_OR_HMI,
    RS422_MSG_INFO_PUMP_TO_HMI,
    RS422_MSG_INFO_FLASH_PUMP_TO_HMI
}RS422_STATE_TX_e;

typedef enum
{
    RS422_RX_WAITING,
    RS422_RX_FAILED,
    RS422_RX_SUCCESS
}RS422_STATE_RX_e;

//! \brief Defines the rs422 object
//!
typedef struct _RS422_COMM_SCI_Obj_
{
    uint32_t            sciHandle;                   //!< the lin peripheral handle
    uint32_t            timerHandle;                 //!< the timer handle
    uint32_t            baudrate;                    //!< the baudrate
    uint16_t            txBuff[16];                  //!< the tx buffer
    uint16_t            rxBuff[16];                  //!< the rx buffer
    uint16_t            txTimeOutTicker;             //!< the tx time out ticker
    uint16_t            rxTimeOutTicker;             //!< the rx time out ticker
    uint16_t            rxTimeOutTickerFifo12;       //!< the rx time out ticker
    uint16_t            crcRx;                       //!< the checksum for Rx
    uint16_t            crcTx;                       //!< the checksum for Tx
    uint16_t            successPacket;               //!< the success packet
    uint16_t            dropPacket;                  //!< the fail packet
    RS422_STATE_TX_e    txState;                     //!< the txState
    RS422_STATE_RX_e    rxState;                     //!< the rxState
    bool                isCommWithHMIFlag;           //!< the flag indicate the pump is talk with HMI
    bool                flashAnsFlag;                //!< the flag flash answer flag
    bool                updateRxParamFlag;           //!< the flag rx param
}RS422_COMM_SCI_Obj;

//! \brief      Defines the serial handle
//!
typedef struct _RS422_COMM_SCI_Obj_ *RS422_COMM_SCI_Handle;

// *****************************************************************************
// the globals

// **************************************************************************
// the function prototypes
//! \brief     Initializes the serial object
//! \param[in] pMemory   A pointer to the memory for serial object
//! \param[in] numBytes  The number of bytes allocated for serial object, bytes
//! \return The serial object handle
extern RS422_COMM_SCI_Handle RS422_COMM_SCI_init(void *pMemory, const size_t numBytes);

//! \brief     config dma and uart peripheral for communication,
//! \param[in] handle           serial object handle.
//! \return    none
extern void RS422_COMM_SCI_setup(RS422_COMM_SCI_Handle handle);

//! \brief     config dma and uart peripheral for communication,
//! \param[in] handle           serial object handle.
//! \return    none
extern void RS422_COMM_SCI_setupGpio(RS422_COMM_SCI_Handle handle);

//! \brief     config dma and uart peripheral for communication,
//! \param[in] handle           serial object handle.
//! \return    none
extern void RS422_COMM_SCI_setupTimer(RS422_COMM_SCI_Handle handle, uint32_t periodCount);

//! \brief     encode the tx message
//! \param[in] handle           serial object handle.
//! \return    none
extern void RS422_COMM_SCI_serialize(RS422_COMM_SCI_Handle handle);


//! \brief     Comm write msg,
//! \param[in] handle           serial object handle.
//! \return    none
extern void RS422_COMM_SCI_write(RS422_COMM_SCI_Handle handle);


//! \brief     Comm read msg,
//! \param[in] handle           serial object handle.
//! \return    none
extern void RS422_COMM_SCI_read(RS422_COMM_SCI_Handle handle);







//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif



#endif /* COMMUNICATION_RS422_COMM_SCI_H_ */
