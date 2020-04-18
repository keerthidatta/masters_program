
#include "can.h"
#include "lib/printfx.h"
#include "can_tests.h"

 uint16_t CRC_Delimiter;


/*****************************************************************************************
*
*                                   Global Variables
*
*******************************************************************************************/
CANConfig_t CANConfig;
#if (SERVER_NODE == 1)
    uint8_t respFrame[11];
#endif

/*****************************************************************************************
*
*									Helper Functions
*
*******************************************************************************************/
void setBit(uint8_t* array, uint16_t bitPos) {

	uint8_t byte_position;
	byte_position = bitPos/8;
	bitPos = bitPos%8;
	*(array + byte_position) |= (1 << bitPos) ;
    //array[(bitPos>>3)] |= (1<<(bitPos&7));
}

void clearBit(uint8_t* array, uint16_t bitPos) {

	uint8_t byte_position;
    byte_position = bitPos/8;
    bitPos = bitPos%8;
    *(array + byte_position) &= ~(1 << bitPos) ;
}




uint8_t testBit(uint8_t* array, uint16_t bitPos) {
	uint16_t value;
	uint8_t byte_position;
    byte_position = bitPos/8;
    bitPos = bitPos%8;
    value = *(array + byte_position)&(1 << bitPos);
    value = value >> bitPos;
	return value;
}

uint16_t writeBuffer(uint8_t* dst, uint16_t dst_msbBit, uint32_t value, uint8_t bitLen)
{
	uint8_t Mask = 0x01;
	uint8_t value1;
	for(uint8_t i = 0; i<bitLen; i++)
	{
		value1 = value>>(bitLen-1-i);
		if(value1 & Mask)
		{
			setBit(dst, dst_msbBit+i);
		}
		else
		{
			clearBit(dst, dst_msbBit+i);
		}
	}
	return dst_msbBit + bitLen;
}


uint32_t readBuffer(uint8_t* src, uint16_t src_msbBit, uint8_t bitLen) {

	uint8_t bit;
	uint32_t value = 0;
	for(uint8_t i = 0; i<bitLen; i++)
	{
	    bit = testBit(src, src_msbBit - 1 + i);
	    value |= (bit<<i);

	    /*
		bit = testBit(src, src_msbBit + i);
		value |= (bit<<(bitLen - 1 - i));
		*/

	    /*
	    bit = testBit(src, src_msbBit + i);
	    value |= (bit<<i);
	     */
	}
	return value;
}

/*****************************************************************************************
*
*									CRC Calculation
*
*******************************************************************************************/
void CAN_shiftReg(uint32_t *status, uint32_t polynom, uint32_t msbMask, uint8_t nextBit)
{
    *status <<=1;
    *status |=nextBit;
    if(*status&msbMask)
    {

        *status ^= polynom;
    }
}

uint32_t CAN_createCRC(uint8_t* src, uint16_t bitLen, uint32_t polynom){
	uint16_t status = 0;
	uint8_t nextBit;
	uint8_t i;
	uint32_t msbMask=0;
	uint32_t pol=polynom,polDegree=0;

    while(pol>>=1)
    {
        polDegree++;
    }

	// Compute the Mask for the most significant bit of polynom
    msbMask|=((uint32_t)1<<polDegree);
    printfx("BITMAASK %d \n", msbMask);
	for (i = 0; i < bitLen; i++) {
		nextBit = testBit(src, i);
		CAN_shiftReg (&status, polynom, msbMask, nextBit);
	}
	return status;
}

/*****************************************************************************************
*
*									Common CAN Bus Functions
*
*******************************************************************************************/
void CAN_init() {
    Pin_OutputMode(CAN_PORT, CAN_TX_PIN);
    Pin_SelectIO(CAN_PORT, CAN_TX_PIN);
    Pin_InputMode(CAN_PORT, CAN_RX_PIN);
    Pin_SelectIO(CAN_PORT, CAN_RX_PIN);
    Pin_EnableInt(CAN_PORT,CAN_RX_PIN);
    Pin_FallingEdgeInt(CAN_PORT,CAN_RX_PIN);
    Pin_EnableResistor(CAN_PORT,CAN_RX_PIN);
    //Set Baud rate
    CAN_setBaudrate(12500); //Maximum Baud rate : 22000
    //Pin_High(CAN_PORT,CAN_RX_PIN);
    Pin_High(CAN_PORT, CAN_TX_PIN);
}

void CAN_setBaudrate(uint32_t baudrate) {
    CANConfig.baudrate = baudrate;
    CANConfig.bit_time = 1000000 / baudrate;
}

int8_t CAN_sendBit(uint8_t bit, Time_t* time) {
#if (TEST_EN == 1)
	return TEST_sendBit(bit);
#else
    Time_t waitTime= *time + CANConfig.bit_time/2;
    busyDelayUntil(time);
	if (bit){
	    Pin_High(CAN_PORT, CAN_TX_PIN);
	}
	else {
	    Pin_Low(CAN_PORT, CAN_TX_PIN);
	}
    busyDelayUntil(&waitTime);

	if(Read_Pin(CAN_PORT,CAN_TX_PIN) == bit)
	    return SUCCESS;
	else
	    return ERR_COLLISION;
#endif
}

uint8_t CAN_recvBit(Time_t *time) {
#if (TEST_EN == 1)
	return TEST_recvBit();
#else
    Time_t waitTime= *time + CANConfig.bit_time/2;
    busyDelayUntil(&waitTime);
    Pin_Toggle(3, 6);
    return Read_Pin(CAN_PORT,CAN_RX_PIN);
#endif
}

/*****************************************************************************************
*
*									Send Frame
*
*******************************************************************************************/
/*******************************
*
*									Send Frame
*
*******************************/
int16_t CAN_createFrame(int32_t ID, uint8_t IDE, uint8_t* payload, uint8_t DLC, uint8_t* dst, uint8_t dstLen) {

    uint32_t ID_A = ID >> 18;
    uint32_t ID_B = ID;
    uint16_t data_offset = 0;
	uint8_t FramePayload;
    uint32_t status = 0;



    if(DLC > 8){
    	DLC = 8;
    }


    for(uint8_t j = 0; j< (dstLen); j++){
        *(dst + j) = 0;
    }


    if(dst != 0){

	    writeBuffer(dst, 0, 0, 1);
	    if(IDE == 0)
	    {
	    	data_offset = 19;

	        writeBuffer(dst, 1, ID, 11);

	        writeBuffer(dst, 12, 0, 1);

	        writeBuffer(dst, 13, IDE, 1);

	        writeBuffer(dst, 14, 0, 1);

	        writeBuffer(dst, 15, DLC, 4);
	    }

		else if(IDE == 1)
		{
			data_offset = 39;

			writeBuffer(dst, 1, ID_A, 11);

			writeBuffer(dst, 12, 1, 1);

			writeBuffer(dst, 13, IDE, 1);

			writeBuffer(dst, 14, ID_B, 18);

			writeBuffer(dst, 32, 0, 1);

			writeBuffer(dst, 33, 0, 2);

	        writeBuffer(dst, 35, DLC, 4);
		}
	    for(int i = 0; i < DLC; i++){
	    	FramePayload = *(payload + i);
	        writeBuffer(dst, (data_offset + i*8), FramePayload, 8);
	    }
	    CRC_Delimiter = (DLC * 8 + data_offset + 15);

		status = CAN_createCRC(dst, (DLC * 8 + data_offset + 15), 0xC599);

		writeBuffer(dst, (DLC * 8 + data_offset), status, 15);

		writeBuffer(dst, CRC_Delimiter, 1, 1);

	}

	return SUCCESS;
}

int16_t CAN_sendFrame(uint8_t* msg, uint8_t doACK) {
    Time_t timeNow;
    uint8_t bit;
    uint8_t oldbit = 1;
    uint16_t bitPos;
    uint16_t bitNumber = 0;
    uint16_t bitsSent = 0;
    uint16_t Bitlength;
    uint8_t IDEBit;
    uint8_t value;
    uint8_t DLC;
    uint16_t data_offset;

    IDEBit = testBit(msg, 13);
    if(IDEBit)
    {
        //readBuffer(uint8_t* src, uint16_t src_msbBit, uint8_t bitLen)
        DLC = readBuffer(msg, 37, 4);
        data_offset = 39;
    }
    else
    {
        DLC = readBuffer(msg, 17, 4);
        data_offset = 19;
    }

    Bitlength = (DLC * 8) + data_offset + 15 + 1;
    getCurrentTime(&timeNow);

    for(bitPos = 0; bitPos < Bitlength; bitPos++)
    {
        bit = testBit(msg, bitPos);
        timeNow += CANConfig.bit_time;
        value = CAN_sendBit(bit, &timeNow);
        if(value < 0)
        {
            return value;
        }
        if(oldbit == bit)
        {
            bitNumber++;
        }
        else
        {
            bitNumber = 0;
        }
        bitsSent++;
        oldbit = bit;

        if(bitNumber == 4)
        {
            timeNow += CANConfig.bit_time;
            value = CAN_sendBit(bit^0x01, &timeNow);
            if(value < 0)
            {
                return value;
            }
            bitsSent++;
            bitNumber = 0;
            oldbit = bit^0x01;
        }

    }
    if(doACK)
    {
        timeNow += CANConfig.bit_time;
        value = CAN_sendBit(1, &timeNow);
        if(value != ERR_COLLISION)
        {
            printf("Error No ACK");
            return ERR_NOACK;
        }
        bitsSent++;
    }
    else
    {
        timeNow += CANConfig.bit_time;
        value = CAN_sendBit(0, &timeNow);
        if(value < 0){
            printf("Error 2");
            return value;
        }
        bitsSent++;
    }

    timeNow += CANConfig.bit_time;
    value = CAN_sendBit(1, &timeNow);
    if(value < 0)
    {
        printf("Error 3");
        return value;
    }
    bitsSent++;

    for(uint8_t k = 0; k<7; k++)
    {
        timeNow += CANConfig.bit_time;
        value = CAN_sendBit(1, &timeNow);
        if(value < 0)
        {
            printf("Error 4");
            return value;
        }
        bitsSent++;
    }
    return bitsSent;
}


int16_t CAN_trySendFrame(uint8_t* msg, uint8_t doACK, uint8_t retries) {

    uint16_t value=0;
    uint8_t i=0;

    uint8_t EOF = 0;
    Time_t timeNow;
    uint8_t BusIdle = 0;

    getCurrentTime(&timeNow);
    for(uint8_t i = 0; i < 7; i++){
        if(CAN_recvBit(&timeNow))
        {
            EOF++;
            if(EOF == 7){
                BusIdle = 1;
            }
        }
        else{
            BusIdle = 0;
            return ERR_BUSBUSY;
        }
    }
    if(BusIdle)
    {
        value= CAN_sendFrame(msg, doACK);
        //if there is error try to retransmit -> retries number of time
        if(value<0)
        {
            for(i=retries; i>0;i--)
            {
                //wait for specific amount of time
                busyDelay(CAN_RETRANSMISSION_TIME);
                value=CAN_sendFrame(msg, doACK);
                if(value==SUCCESS){
                    //transmission sucessful -> return retries +1 attempts
                     return (retries +1);
                }
            }
        }
    }
}

/*****************************************************************************************
*
*                                   Receive Frame
*
*******************************************************************************************/
int8_t  CAN_receiveFrame(uint8_t* dst, uint8_t dstLen, Time_t time) {

    Time_t timeNow = time;
    uint8_t bit;
    uint8_t oldbit = 1;
    uint8_t bitNumber = 0;
    uint8_t TotalBitsReceived = 0;
    uint32_t status = 0;
    uint8_t Bitlength = dstLen*8;
    uint8_t Extended = 0;
    uint8_t DLC = 0;

    for(uint8_t i=0; i<Bitlength; i++)
    {
        bit = CAN_recvBit(&timeNow);
        timeNow += CANConfig.bit_time;

        if(bit == oldbit)
        {
            bitNumber++;
        }
        else
        {
            bitNumber = 0;
        }

        if(bit){
            setBit(dst, TotalBitsReceived);
        }
        else{
            clearBit(dst, TotalBitsReceived);
        }

        TotalBitsReceived++;
        if(bitNumber == 4)
        {
            if(bit == CAN_recvBit(&timeNow)){
                printf("Bit number %d", TotalBitsReceived);
                return ERR_BITSTUFF;
            }
            timeNow += CANConfig.bit_time;
            bitNumber = 0;
            oldbit = bit^0x01;
        }
        else
        {
            oldbit = bit;
        }

        CAN_shiftReg(&status, CRC_CAN, 0x8000, bit);


        if(TotalBitsReceived == 13){
            Extended = bit;
            //printf("IDE %d\n", bit);
        }


        if(!Extended){
            if((TotalBitsReceived == 16))
                DLC |= bit << 3;
            else if((TotalBitsReceived == 17))
                DLC |= bit << 2;
            else if((TotalBitsReceived == 18))
                DLC |= bit << 1;
            else if((TotalBitsReceived == 19))
                DLC |= bit;
            else if((TotalBitsReceived == (19 + (DLC * 8) +  15)) && !Extended)
                break;
        }
        else
        {
            if((TotalBitsReceived == 36) && Extended)
                DLC |= bit << 3;
            else if((TotalBitsReceived == 37) && Extended)
                DLC |= bit << 2;
            else if((TotalBitsReceived == 38) && Extended)
                DLC |= bit << 1;
            else if((TotalBitsReceived == 39) && Extended)
                DLC |= bit;
            else if((TotalBitsReceived == (39 + (DLC * 8) +  15)) && Extended)
                break;
        }
    }

    bit = CAN_recvBit(&timeNow);
    timeNow += CANConfig.bit_time;

    if(bit)
       setBit(dst, TotalBitsReceived);
    else
       clearBit(dst, TotalBitsReceived);

    TotalBitsReceived++;

    if(status == 0)
    {
        timeNow += CANConfig.bit_time;
        CAN_sendBit(0, &timeNow);
    }
    else
    {
        return ERR_CRC;
    }
    //CAN_decodeFrame(&dst);
    return SUCCESS;
}

void CAN_decodeFrame(uint8_t* dst) {

    uint8_t IDEBit;
    uint8_t DLC;
    uint16_t data_offset;

    uint16_t ID_A;
    uint8_t StandardRTR;
    uint8_t ExtendedRTR;

    uint16_t ID_B;
    uint64_t Data;
    uint16_t CRC;
    uint8_t CRCDelimiter;

    uint8_t StandardReserved;

    uint8_t ExtendedSRR;
    uint8_t ExtendedReserved;

    IDEBit = testBit(dst, 13);
    if(IDEBit)
    {
        ExtendedSRR = testBit(dst, 12);
        ID_B = readBuffer(dst, 30, 18);
        ExtendedRTR = testBit(dst, 32);
        ExtendedReserved = readBuffer(dst, 33, 2);
        DLC = readBuffer(dst, 37, 4);
        data_offset = 39;
    }
    else
    {
        ID_A = readBuffer(dst, 10, 11);
        StandardRTR = testBit(dst, 12);
        StandardReserved = testBit(dst, 14);
        DLC = readBuffer(dst, 17, 4);
        data_offset = 19;
    }

    Data = readBuffer(dst, (DLC * 8 + data_offset-2), (DLC * 8));
    CRC = readBuffer(dst, (DLC * 8 + data_offset-2 + 15), 15);
    CRCDelimiter = testBit(dst, (DLC * 8 + 39 + 15 + 1));


    printf("Data %lu\n", Data);
    printf("CRC %d\n", CRC);
    printf("Data %d\n", CRCDelimiter);
    printf("ID_B %d\n", ID_B);
    printf("ID_A %d\n", ID_A);


#if (SERVER_NODE == 1)
    // Perform requested operation and send response

    uint16_t valid = 1, result=0;
    printfx ("\nOperation: ");
    switch (id){
        case OP_ADD:
            printfx (" + Addition");
            result = (int16_t)data[0] + (int16_t)data[1];
            break;
        case OP_SUB:
            printfx (" - Subtraction");
            result = (int16_t)data[0] - (int16_t)data[1];
            break;
        case OP_MUL:
            printfx (" * Multiplication");
            result = (int16_t)data[0] * (int16_t)data[1];
            break;
        default:
            valid = 0;
            printfx (" INVALID!");
    }
    printfx("\n");
    if (valid){
        printfx("Sending result ...\n");
        CAN_createFrame(0x00, 0, (uint8_t*)&result, 2, respFrame, sizeof(respFrame));
        busyDelay(CAN_RETRANSMISSION_TIME/2);
        CAN_trySendFrame(respFrame, CAN_DOACK, 0);
    }
#endif

}

