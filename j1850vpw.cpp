/*
 * j1850.cpp
 *
 *  Created on: 26 но€б. 2017 г.
 *      Author: Piton
 */

#include "j1850vpw.h"


#if (F_CPU == 48000000)
#define BUS_IDLE     ((FTM1_C0SC & FTM_CSC_CHIE)==0)
#define TX_PENDING   (FTM1_C1SC & FTM_CSC_CHIE)


J1850_VPW* J1850_VPW::_instance;
// static ISR method which fires our singleton's method.
void J1850_VPW::pin_isr(void) {
	J1850_VPW::_instance->PinRx();}
void J1850_VPW::ftm1_isr(void){
	J1850_VPW::_instance->Time();}
//=====================================================
void J1850_VPW::Init(size_t pinIn,size_t pinOut)
{
		pinMode(pinIn, INPUT/*_PULLUP*/);
	    _pin_rx=portInputRegister(pinIn);    //bitband

	    pinMode(pinOut, OUTPUT_OPENDRAIN);
		_p_tx = portOutputRegister(pinOut);  //bitband
	    *_p_tx=0;
	    error_rx=0;

	    FTM1_SC = (FTM_SC_CLKS(0x00) | FTM_SC_PS(0x00)); /* Clear status and control register */
	    FTM1_MOD = 0xFFFF;        	/* Set up modulo register */
	    FTM1_C0SC = 0x10;        	// Continuous  Capture mode
	    FTM1_C1SC = 0x10;        	// Continuous  Capture mode
	    FTM1_SC = (FTM_SC_CLKS(0x01) | FTM_SC_PS(DIV)); /* 48000/(2^5) => 1.5 Mhz */

	    NVIC_SET_PRIORITY(IRQ_FTM1, 32); //2 Level
	    attachInterruptVector(IRQ_FTM1, ftm1_isr);
	    NVIC_ENABLE_IRQ(IRQ_FTM1);

	    NVIC_SET_PRIORITY(IRQ_PORTC, 32); //2 Level
	    attachInterrupt(pinIn, pin_isr,CHANGE);

}
//=====================================================================
size_t J1850_VPW::Tx_J1850(uint8_t *data,size_t len)
{
	if((len > MAX_SIZE-1) || (len==0))
			                 return 1;
	if(TX_PENDING)
        					 return 2;

	memcpy(_buftx, data, len);
	_buftx[len] =  Crc8_J1850(_buftx, len);
	_tx_Len = len + 1;
	FTM1_C1V = (FTM1_CNT + IFS_TIME_R);

	//Enable TimeOut & Clear Flag
	FTM1_STATUS &= ~(FTM_STATUS_CH1F); //Clear flag
	FTM1_C1SC |= FTM_CSC_CHIE;

	return 0;

}
//======================================================================
void J1850_VPW::PinRx(void)
{
	static size_t _rx_Bit;
	static uint8_t _rx_Byte;

	uint16_t dT = FTM1_CNT - LastTime;
	LastTime=FTM1_CNT;

	 // Reload the overflow timer with EOD timeout
	FTM1_C0V = (LastTime + EOD_TIME_R);

 	bool pin = *_pin_rx;

    if (BUS_IDLE) { //if Idle wait  SOF
      if (pin == 0){ // End SOF ?

    	  int  sof_bit = (dT - SOF_TIME_R); //Deviation

    	  if(sof_bit<0) sof_bit=-sof_bit; // abs(Deviation)


      	if (sof_bit < SOF_DEV_R){ //SOF Time ?

      		// found SOF, start header/data sampling
      		_rx_Bit = 0;
      		_bufrx.Len = 0;

      		//Clear flag
    		FTM1_STATUS &= ~(FTM_STATUS_CH0F);
    		//Enable Interrupt
    		FTM1_C0SC |= FTM_CSC_CHIE;
        }
      }
    }//if (fIdle)
    else
    {
    	//deviation
    	int shortbit = (dT - SHORTBIT_TIME_R);
        int longbit  = (dT - LONGBIT_TIME_R);


      // abs() deviation
  	   if(shortbit<0)  shortbit = -shortbit;
       if(longbit<0)   longbit  = -longbit;

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
      {
    	  error_rx++; // unknown bit, Error
     	 //Disable Interrupt
    	  FTM1_C0SC &=~(FTM_CSC_CHIE);
          return;
      }

      if (_rx_Bit >= 8){ //Check Byte
    	  _bufrx.Data[_bufrx.Len++] = _rx_Byte;
    	  _rx_Bit = 0;

      	if (_bufrx.Len >= sizeof(_bufrx.Data)) { //Check Size
      		 error_rx++; // too many data bytes, error
      		// Disable Interrupt
      	     FTM1_C0SC &=~(FTM_CSC_CHIE);
      	}//Check Size
      }//Check Byte
   }
}
//======================================================================
 void J1850_VPW::Time(void)
{
	static size_t TxPinState;
	static size_t _mask_bit=0,_ind_tx;

	size_t _currT = FTM1_CNT;   // - (Adj *F_TIC/10); // TODO add  adjust delay enter isr

	if( TX_PENDING  && (FTM1_STATUS & FTM_STATUS_CH1F)){ //Tx Interrupt

		FTM1_STATUS = ~(FTM_STATUS_CH1F);//Clear flag interrupt

//Test Start Frame
	 if(!_mask_bit){
			if(BUS_IDLE){
				// Start SOF
				  *_p_tx = (TxPinState = ACTIVE);
				  _mask_bit=0x80;
				  _ind_tx = 0;
 				  FTM1_C1V = (_currT + SOF_TIME_R);
			 }
			else
				//Wait BUS IDLE
				 FTM1_C1V = (_currT + (SOF_TIME_R/4));
		 } else
// Test Collision
		if(*_pin_rx != TxPinState){
			_mask_bit=0; //repeat
			*_p_tx = (TxPinState = PASSIVE);
			FTM1_C1V = (_currT + IFS_TIME_R);
		} else
// Test completed  Transmit message
		if(_ind_tx >= _tx_Len){
			*_p_tx = (TxPinState = PASSIVE); //passive
			//disable the compare interrupt, message sending has completed.
			 FTM1_C1SC &=~(FTM_CSC_CHIE); // Tx_Pending  = false;
			 _mask_bit=0;
		} else
 //Transmit BIT
		{  	*_p_tx =  (TxPinState ^= 1); //Togle

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
	}

//====== Test EOD timeout ======
	if((FTM1_C0SC & FTM_CSC_CHIE) && (FTM1_STATUS & FTM_STATUS_CH0F))
		{
		LastTime = FTM1_CNT;
		FTM1_STATUS = ~(FTM_STATUS_CH0F); //Clear flag interrupt
		FTM1_C0SC &=~(FTM_CSC_CHIE); //disable interrupt
		if(!isFull() ){
			uint8_t crc = Crc8_J1850((uint8_t*)&_bufrx.Data, _bufrx.Len-1 );
			if(crc !=_bufrx.Data[_bufrx.Len-1])
											_bufrx.Len|=0x80;
			write(_bufrx);
		}
	}
}
 //======================================================================
  uint8_t J1850_VPW::Crc8_J1850(uint8_t *buffer, size_t len)
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

#endif
