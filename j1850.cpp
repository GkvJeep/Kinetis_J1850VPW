/*
 * j1850.cpp
 *
 *  Created on: 26 но€б. 2017 г.
 *      Author: Piton
 */
#include "j1850.h"

uint32_t J1850::LastTime;
size_t  J1850::_tx_Len;
size_t  J1850::_rx_Len;
uint8_t  J1850::_bufrx[],J1850::_buftx[];
bool J1850::fTx_Pending;
volatile uint8_t *J1850::_p_tx;
volatile uint8_t *J1850::_p_rx;

//====================================================================
void J1850::Init(size_t pinIn,size_t pinOut)
{
	//Debug
	    pinMode(pinIn, INPUT_PULLUP);
	    _p_rx=portInputRegister(pinIn);


		_p_tx = portOutputRegister(pinOut);
	     pinMode(pinOut, OUTPUT_OPENDRAIN);
	    *_p_tx=0;

	    FTM1_SC = (FTM_SC_CLKS(0x00) | FTM_SC_PS(0x00)); /* Clear status and control register */
	    FTM1_MOD = 0xFFFF;        /* Set up modulo register */
	    FTM1_C0SC = 0x10;        // Continuous  Capture mode
	    FTM1_C1SC = 0x10;        // Continuous  Capture mode
	    FTM1_SC = (FTM_SC_CLKS(0x01) | FTM_SC_PS(0x05)); /* 48000/(2^5) => 1.5 Mhz */

	    NVIC_SET_PRIORITY(IRQ_FTM1, 64); // 0 = highest priority, 255 = lowest
	    attachInterruptVector(IRQ_FTM1, Timer_isr);
	    NVIC_ENABLE_IRQ(IRQ_FTM1);
	    attachInterrupt(pinIn, isr_Pin,CHANGE);


}
//=====================================================================
size_t J1850::Out_J1850(uint8_t *data,size_t len)
{
	if((len > BUFSIZE_J1850-1) || (len==0))
			                return 1;
	if(fTx_Pending)
        					return 2;

	memcpy(_buftx, data, len);
	_buftx[len] =  Crc8_J1850(_buftx, len);
	_tx_Len = len + 1;
	FTM1_C1V = (FTM1_CNT + IFS_TIME_R);
	//Enable TimeOut & Clear Flag
	FTM1_STATUS &= ~(FTM_STATUS_CH1F); //Clear flag
	FTM1_C1SC |= FTM_CSC_CHIE;
	fTx_Pending = true;
	return 0;

}
//======================================================================
void J1850::isr_Pin(void)
{
	static size_t _rx_Bit;
	static uint8_t _rx_Byte;

	int delta = FTM1_CNT - LastTime;
	LastTime=FTM1_CNT;

	 // Reload the overflow timer with EOD timeout
	FTM1_C0V = (LastTime + EOD_TIME_R);

 	bool pin = *_p_rx;

    if ((FTM1_C0SC & FTM_CSC_CHIE)==0) { //if Idle test SOF Time
      if (pin == 0){ //Start ?
    	int  longbit = (delta - SOF_TIME_R);
      	if(longbit<0) longbit=-longbit; // abs()

      	if (longbit < SOF_DEV_R){ //SOF Time ?
      		// found SOF, start header/data sampling
      		_rx_Bit = 0;
      		_rx_Len = 0;
      		//Enable Interrupt
    		FTM1_STATUS &= ~(FTM_STATUS_CH0F); //Clear flag
      		FTM1_C0SC |= FTM_CSC_CHIE;
        }
      }
    }//if (fIdle)
    else {
      int shortbit = (delta - SHORTBIT_TIME_R);
      if(shortbit<0) shortbit=-shortbit; // abs()
      int longbit  = (delta - LONGBIT_TIME_R);
      if(longbit<0) longbit=-longbit;
          _rx_Bit++;
    	  _rx_Byte<<=1;
      if (shortbit < SHORTBIT_DEV_R) {
             if (!pin) // short pulse & pulse was low => active "1"
      	         _rx_Byte++;
      }else
      if (longbit < LONGBIT_DEV_R) {
        	if (pin)// long pulse & pulse was high  => passive "1"
      	 	    _rx_Byte++;
      }else
      { // unknown bit, Error
    	 //Disable Interrupt
    	  FTM1_C0SC &=~(FTM_CSC_CHIE);
          return;
      }

      if (_rx_Bit >= 8){ //Check Byte
    	  _bufrx[_rx_Len++] = _rx_Byte;
    	  _rx_Bit = 0;

      	if (_rx_Len >= sizeof(_bufrx)) { //Check Size
     		// too many data bytes, error
      		//Disable Interrupt
      	     FTM1_C0SC &=~(FTM_CSC_CHIE);
      	}//Check Size
      }//Check Byte
   }
}
//======================================================================
 void J1850::Timer_isr(void)
{
	static size_t TxPinState;
	static size_t _mask_bit=0,_ind_tx;

	size_t _currT = FTM1_CNT;// - (Adj *F_TIC/10); // add  adjust delay enter isr

	if( (FTM1_C1SC & FTM_CSC_CHIE) && (FTM1_STATUS & FTM_STATUS_CH1F)){ //Tx Interrupt

		FTM1_STATUS = ~(FTM_STATUS_CH1F);//Clear flag

		if(!_mask_bit){

			 if(!(FTM1_C0SC & FTM_CSC_CHIE)){  //Bus Idle ?
 				 // Start SOF
				 *_p_tx = (TxPinState = ACTIVE);
				  _mask_bit=0x80;
				  _ind_tx = 0;
			 }

			 FTM1_C1V = (_currT + SOF_TIME_R);
			 return;
		 }

  // Test Collision (only if  TxPinState == PASSIVE)

		if(*_p_rx != TxPinState){
			_mask_bit=0; //repeat
			*_p_tx = (TxPinState = PASSIVE);
			FTM1_C1V = (_currT + IFS_TIME_R);
			return;
		}

	//Test End
		if(_ind_tx >= _tx_Len){
			*_p_tx = (TxPinState = PASSIVE); //passive
			//disable the compare interrupt, message sending has completed.
			 FTM1_C1SC &=~(FTM_CSC_CHIE);
			fTx_Pending  = false;
			_mask_bit=0;
			return;
		}


		*_p_tx =  (TxPinState ^= 1); //Togle

	size_t bitValue = _buftx[_ind_tx] & _mask_bit;
	size_t TimeBit;

		if(!TxPinState ) //Passive
				TimeBit = bitValue ? LONGBIT_TIME_R : SHORTBIT_TIME_R;
		else
				TimeBit = bitValue ? SHORTBIT_TIME_R : LONGBIT_TIME_R ;

		  FTM1_C1V = (_currT + TimeBit);

		  _mask_bit >>=1;

		  if(!_mask_bit){
			  _ind_tx++;
			  _mask_bit = 0x80;
		  }
	}

//===========================================
	if((FTM1_C0SC & FTM_CSC_CHIE) && (FTM1_STATUS & FTM_STATUS_CH0F))
		{
		LastTime = FTM1_CNT;
		FTM1_STATUS = ~(FTM_STATUS_CH0F);
		FTM1_C0SC &=~(FTM_CSC_CHIE);
	   //TODO EMPTY
		_rx_Len = 0;
	}



}
//======================================================================
uint8_t J1850::Crc8_J1850(uint8_t *buffer, size_t len)
{
	uint8_t crc_reg=0xff,poly;
	size_t i,j;
	uint8_t *byte_point;
	uint8_t bit_point;

	for (i = 0, byte_point = buffer; i < len; ++i, ++byte_point)
	{
		for (j = 0, bit_point = 0x80 ; j < 8; ++j, bit_point >>= 1)
		{
			if (bit_point & *byte_point)	// case for new bit = 1
			{
				poly = (crc_reg & 0x80 ? 1 : 0x1C);
				crc_reg= ( (crc_reg << 1) | 1) ^ poly;
			}
			else		// case for new bit = 0
			{
				poly = (crc_reg & 0x80 ? 0x1D : 0);
				crc_reg = (crc_reg << 1) ^ poly;
			}
		}
	}
	return ~crc_reg;	// Return CRC
}


