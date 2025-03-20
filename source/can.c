/***************************************************************************//**
  @file     can.c
  @brief    CAN wrapper
  @author   Nicolás Magliola
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include "can.h"

#include "fsl_mcan.h"
#include "fsl_iocon.h"

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

#define USE_IMPROVED_TIMING_CONFIG (1U)
#define MCAN_CLK_FREQ              CLOCK_GetMCanClkFreq()
#define CAN_DATASIZE 		CAN_MAX_BYTES_PER_MSG


// RX FIFO
#define RX_FIFO_NUM			0
#define RX_FIFO_CANT		32

// TX FIFO
#define TX_FIFO_CANT		32


// Memoria
#define MSG_RAM_BASE        0x04000000U
#define STD_FILTER_OFS 		0x0
#define RX_FIFO_OFS    		0x10U
#define TX_BUFFER_OFS  		(RX_FIFO_OFS + (8+CAN_DATASIZE)*RX_FIFO_CANT)
#define MSG_RAM_SIZE   		(TX_BUFFER_OFS + (8+CAN_DATASIZE)*TX_FIFO_CANT)


// Varios
#define FXDS2BYTES(val)		(((val)<5)? 8+4*(val) : 16*((val)-3))
#define STDID_OFFSET        (29-11)


/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

/*******************************************************************************
 * VARIABLES WITH GLOBAL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * FUNCTION PROTOTYPES FOR PRIVATE FUNCTIONS WITH FILE LEVEL SCOPE
 ******************************************************************************/

// simil a la del driver, pero corrigiendo bug
static status_t myMCAN_ReadRxFifo(CAN_Type *base, uint8_t fifoBlock, mcan_rx_buffer_frame_t *pRxFrame);
static uint32_t MCAN_GetRxFifo0ElementAddress(CAN_Type *base);
static uint32_t MCAN_GetRxFifo1ElementAddress(CAN_Type *base);


/*******************************************************************************
 * ROM CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * STATIC VARIABLES AND CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

#ifndef MSG_RAM_BASE
/* The MCAN IP stipulates that the Message RAM start address must align with 0x10000, but in some devices, the single
 * available RAM block size is less than this value, so we must try to place the Message RAM variables at the start of
 * the RAM block to avoid memory overflow during link application. By default, the data section is before of the bss
 * section, so adds the initial value for Message RAM variables to confirm it be placed in the data section, and it must
 * be initialized with 0 before used. */
#if defined(__CC_ARM) || defined(__ARMCC_VERSION)
/* The KEIL --sort=algorithm only impacts code sections, and the data sections is sorted by name, so by specifying the
 * memory as .data to ensure that it is put in the first place of RAM. */
__attribute__((aligned(1U << CAN_MRBA_BA_SHIFT), section(".data"))) uint8_t msgRam[MSG_RAM_SIZE] = {1U};
#else
SDK_ALIGN(uint8_t msgRam[MSG_RAM_SIZE], 1U << CAN_MRBA_BA_SHIFT) = {1U};
#endif
#else
#define msgRam MSG_RAM_BASE
#endif

static bool canInit = false;


/*******************************************************************************
 *******************************************************************************
                        GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

bool can_init(uint32_t baudrate)
{
	if (canInit)
		return false;

	// 1) Configuro pines CAN
	const uint32_t pin_canConfig = IOCON_PIO_FUNC(1) | IOCON_PIO_MODE(2) | IOCON_PIO_DIGIMODE(1); // Mux Alt1 ; pullup ; digital
	IOCON_PinMuxSet(IOCON, 1, 2, pin_canConfig);
	IOCON_PinMuxSet(IOCON, 1, 3, pin_canConfig);

	// 2) Set MCAN clock 96Mhz/4=24MHz
	CLOCK_SetClkDiv(kCLOCK_DivCanClk, 4U, true);
	CLOCK_AttachClk(kMCAN_DIV_to_MCAN);

	// 3) Calculo time quanta (Update the improved timing configuration)
	mcan_config_t mcanConfig;
	MCAN_GetDefaultConfig(&mcanConfig);
	mcanConfig.baudRateA = baudrate;
	// solo para testear!!!              mcanConfig.enableLoopBackExt = true;
    mcan_timing_config_t timing_config;
    memset(&timing_config, 0, sizeof(timing_config));
    if (!MCAN_CalculateImprovedTimingValues(mcanConfig.baudRateA, MCAN_CLK_FREQ, &timing_config))
    	return false;
    memcpy(&(mcanConfig.timingConfig), &timing_config, sizeof(mcan_timing_config_t));

    MCAN_Init(CAN0, &mcanConfig, MCAN_CLK_FREQ);

    /* Set Message RAM base address and clear to avoid BEU/BEC error. */
    MCAN_SetMsgRAMBase(CAN0, (uint32_t)msgRam);
    memset((void *)msgRam, 0, MSG_RAM_SIZE * sizeof(uint8_t));

    /* STD filter config. */
    mcan_frame_filter_config_t rxFilter;
    rxFilter.address  = STD_FILTER_OFS;
    rxFilter.idFormat = kMCAN_FrameIDStandard;
    rxFilter.listSize = 1U;
    rxFilter.nmFrame  = kMCAN_reject0;
    rxFilter.remFrame = kMCAN_rejectFrame;
    MCAN_SetFilterConfig(CAN0, &rxFilter);

    mcan_std_filter_element_config_t stdFilter;
    stdFilter.sfec = kMCAN_storeinFifo0;
    stdFilter.sft   = kMCAN_range; /* Filter mode */
    stdFilter.sfid1 = 0x001; // ID min
    stdFilter.sfid2 = 0x7fe; // ID max
    MCAN_SetSTDFilterElement(CAN0, &rxFilter, &stdFilter, 0);

    /* RX FIFO config. */
    mcan_rx_fifo_config_t rxFifo;
    rxFifo.address       = RX_FIFO_OFS;
    rxFifo.elementSize   = RX_FIFO_CANT;
    rxFifo.watermark     = 1;
    rxFifo.opmode        = kMCAN_FifoBlocking;
    rxFifo.datafieldSize = kMCAN_8ByteDatafield;
#if RX_FIFO_NUM == 0
    MCAN_SetRxFifo0Config(CAN0, &rxFifo);
#elif RX_FIFO_NUM == 1
    MCAN_SetRxFifo1Config(CAN0, &rxFifo);
#endif // RX_FIFO_NUM == xx

    /* TX buffer config. */
    mcan_tx_buffer_config_t txBufferConfig;
    txBufferConfig.address       = TX_BUFFER_OFS;
    txBufferConfig.dedicatedSize = 0;
    txBufferConfig.fqSize        = TX_FIFO_CANT;
    txBufferConfig.datafieldSize = kMCAN_8ByteDatafield;
    MCAN_SetTxBufferConfig(CAN0, &txBufferConfig);

    /* Finish software initialization and enter normal mode, synchronizes to
           CAN bus, ready for communication */
    MCAN_EnterNormalMode(CAN0);

    canInit = true;


    return true;
}


/******************* RX *******************/

bool can_isNewRxMsg(void)
{
	return ((CAN0->RXF0S & CAN_RXF0S_F0FL_MASK) != 0U);
}


bool can_readRxMsg(can_msg_t* msg)
{
	// leo un RX buffer de la RX FIFO 0
	mcan_rx_buffer_frame_t rxBuffer = {.size = CAN_DATASIZE, .data = msg->data};
	if (myMCAN_ReadRxFifo(CAN0, 0, &rxBuffer) != kStatus_Success)
		return false;

	// guardo info del Rx buffer en msg
	msg->id = rxBuffer.id >> STDID_OFFSET; // se guarda left-justified en 29bits o 11bits
	msg->len = rxBuffer.dlc;
	// no hace falta, ya se copia en MCAN_ReadRxFifo() porque inicialicé rxBuffer.data = msg->data     memcpy(msg->data, rxBuffer.data, msg->len);

	return true;
}

/******************* TX *******************/

bool can_isTxReady(void)
{
	return ((CAN0->TXFQS & CAN_TXFQS_TFQF_MASK) == 0); // TX FIFO no está FULL
}


bool can_sendTxMsg(const can_msg_t* msg)
{
	// 1) ver que la TX FIFO no esté llena
	if (CAN0->TXFQS & CAN_TXFQS_TFQF_MASK)
		return false; // TX FIFO está FULL

	// 2) obtener el TX FIFO put index
	uint8_t idx = (CAN0->TXFQS & CAN_TXFQS_TFQPI_MASK) >> CAN_TXFQS_TFQPI_SHIFT;

	// 3) escribir la info en dicho TX FIFO buffer
	mcan_tx_buffer_frame_t txBuffer = {.xtd=kMCAN_FrameIDStandard, .rtr=kMCAN_FrameTypeData, .fdf=false, .brs=false, .efc=false, .size=8};
	txBuffer.id = msg->id << STDID_OFFSET;
	txBuffer.dlc = msg->len;
	txBuffer.data = (uint8_t*) (msg->data);
	if (MCAN_WriteTxBuffer(CAN0, idx, &txBuffer) != kStatus_Success)
		return false; // falló la escritura

	// 4) Escribir un AddRequest de ese buffer
	CAN0->TXBAR = (1<<idx);

    return true;
}



/*******************************************************************************
 *******************************************************************************
                        LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/


static status_t myMCAN_ReadRxFifo(CAN_Type *base, uint8_t fifoBlock, mcan_rx_buffer_frame_t *pRxFrame)
{
    /* Assertion. */
    assert((0U == fifoBlock) || (1U == fifoBlock));
    assert(NULL != pRxFrame);

    mcan_rx_buffer_frame_t *elementAddress = NULL;

    if (0U == fifoBlock)
    {
        if ((base->RXF0S & CAN_RXF0S_F0FL_MASK) != 0U)
        {
            elementAddress = (mcan_rx_buffer_frame_t *)(MCAN_GetMsgRAMBase(base) + MCAN_GetRxFifo0ElementAddress(base));
        }
        else
        {
            return kStatus_Fail;
        }
    }
    else
    {
        if ((base->RXF1S & CAN_RXF1S_F1FL_MASK) != 0U)
        {
            elementAddress = (mcan_rx_buffer_frame_t *)(MCAN_GetMsgRAMBase(base) + MCAN_GetRxFifo1ElementAddress(base));
        }
        else
        {
            return kStatus_Fail;
        }
    }
    (void)memcpy(pRxFrame, elementAddress, 8U);
    // NM: BUG DE NXP!!!!! data apunta un buffer que libera con el acknowledge!!!      pRxFrame->data = (uint8_t *)((uint32_t)elementAddress + 8U);
    (void)memcpy(pRxFrame->data, (uint8_t*)elementAddress + 8, pRxFrame->size);
    /* Acknowledge the read. */
    if (0U == fifoBlock)
    {
        base->RXF0A = (base->RXF0S & CAN_RXF0S_F0GI_MASK) >> CAN_RXF0S_F0GI_SHIFT;
    }
    else
    {
        base->RXF1A = (base->RXF1S & CAN_RXF1S_F1GI_MASK) >> CAN_RXF1S_F1GI_SHIFT;
    }
    return kStatus_Success;
}

static uint32_t MCAN_GetRxFifo0ElementAddress(CAN_Type *base)
{
    uint32_t eSize;
    uint32_t eAddress;
    eSize = (base->RXESC & CAN_RXESC_F0DS_MASK) >> CAN_RXESC_F0DS_SHIFT;
    if (eSize < 5U)
    {
        eSize += 4U;
    }
    else
    {
        eSize = eSize * 4U - 10U;
    }
    eAddress = base->RXF0C & CAN_RXF0C_F0SA_MASK;
    eAddress += ((base->RXF0S & CAN_RXF0S_F0GI_MASK) >> CAN_RXF0S_F0GI_SHIFT) * eSize * 4U;
    return eAddress;
}

static uint32_t MCAN_GetRxFifo1ElementAddress(CAN_Type *base)
{
    uint32_t eSize;
    uint32_t eAddress;
    eSize = (base->RXESC & CAN_RXESC_F1DS_MASK) >> CAN_RXESC_F1DS_SHIFT;
    if (eSize < 5U)
    {
        eSize += 4U;
    }
    else
    {
        eSize = eSize * 4U - 10U;
    }
    eAddress = base->RXF1C & CAN_RXF1C_F1SA_MASK;
    eAddress += ((base->RXF1S & CAN_RXF1S_F1GI_MASK) >> CAN_RXF1S_F1GI_SHIFT) * eSize * 4U;
    return eAddress;
}

void send_can_message(uint16_t id, uint8_t *data, uint8_t length) {
    if (length > CAN_MAX_BYTES_PER_MSG) return;

    can_msg_t msg;
    msg.id = id;
    msg.len = length;
    for (int i = 0; i < length; i++) {
        msg.data[i] = data[i];
    }

    // Esperar hasta que el CAN esté listo para enviar
    while (!can_isTxReady());
    can_sendTxMsg(&msg);


}

bool receive_can_message(uint16_t expected_id, uint8_t *buffer) {
    can_msg_t msg;

    // Verificar si hay un mensaje nuevo
    if (can_isNewRxMsg()) {
        if (can_readRxMsg(&msg) && msg.id == expected_id) {
            for (int i = 0; i < msg.len; i++) {
                buffer[i] = msg.data[i];
            }
            return true;
        }
    }
    return false;
}



/*******************************************************************************
 ******************************************************************************/
