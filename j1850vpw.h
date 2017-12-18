/*
* The MIT License (MIT)
*
* Copyright (c) 2017,Konstantin Grjznov
*
* Permission is hereby granted, free of charge, to any person obtaining a copy of this software
* and associated documentation files (the “Software”), to deal in the Software without restriction,
* including without limitation the rights to use, copy, modify, merge, publish, distribute,
* sublicense, and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all copies or
* substantial portions of the Software.
*
*THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
*INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
*IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
*DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
*ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
*IN THE SOFTWARE.
*
*     Created on: 11/26/2017 ã. v.1.0
*/

#ifndef J1850VPW_H_
#define J1850VPW_H_

#include "Arduino.h"



#if (F_CPU == 48000000)
#define ACTIVE 1
#define PASSIVE 0

#define DIV     5 	// 2^5=32    =>  48'000'000/32 = 1.5 MHz
#define F_TIC  15   // 1.5*10

// Timing for a long bit pulse uS
#define LONGBIT_TIME   128  //  Nominal
#define LONGBIT_TIME_R (LONGBIT_TIME * F_TIC/10)
#define LONGBIT_DEV     16 //Deviation
#define LONGBIT_DEV_R   (LONGBIT_DEV * F_TIC/10)


// Timing for short bit pulse uS
#define SHORTBIT_TIME   64  //Nominal
#define SHORTBIT_TIME_R (SHORTBIT_TIME * F_TIC/10)
#define SHORTBIT_DEV    15 //Deviation
#define SHORTBIT_DEV_R  (SHORTBIT_DEV * F_TIC/10)

// Timing for start of frame uS
#define SOF_TIME      200     //Nominal
#define SOF_TIME_R    (SOF_TIME * F_TIC/10 )
#define SOF_DEV        18   //Deviation
#define SOF_DEV_R     (SOF_DEV * F_TIC/10 )

// Timing for end of data frame uS
#define EOD_TIME       200   //Nominal
#define EOD_TIME_R     (EOD_TIME * F_TIC/10 )
#define EOD_DEV        18   //Deviation
#define EOD_DEV_R      (EOD_DEV * F_TIC/10) )
// Timing for end of frame uS
#define EOF_TIME      280   //Nominal
#define EOF_TIME_R	  (EOF_TIME * F_TIC/10 )
#define EOF_DEV        18  //Deviation
#define EOF_DEV_R     (EOF_DEV * F_TIC/10 )

// Timing for IN-FRAME RESPONSE uS
#define IFS_TIME      300   //Nominal
#define IFS_TIME_R		(IFS_TIME * F_TIC/10 )
#define IFS_DEV        20  //Deviation
#define IFS_DEV_R	   (IFS_DEV)
// Timing for BREAK uS
#define BREAK_TIME      300  //Nominal
#define BREAK_TIME_R	(BREAK_TIME * F_TIC/10 )
#define BREAK_DEV        20 //Deviation
#define BREAK_DEV_R     (BREAK_DEV * F_TIC/10) )


// Storage, max 11 data bytes + CRC
#define MAX_SIZE 12
#define MAX_FIFO 8

typedef struct{
	size_t  Len;
	uint8_t Data[MAX_SIZE];
}Str_J1850;


class J1850_VPW
{
#define F_SIZE 4
public:

	 J1850_VPW():
		   LastTime(0),_tx_Len(0),error_rx(0),_start(0),_end(0)
		{
		 	  pElems =  new Str_J1850[F_SIZE];
		 	 _instance = this;

	 	 };

	 void Init(size_t pin_In,size_t pin_Out);
	 size_t Tx_J1850(uint8_t *data,size_t len);
	 inline int isEmpty() { return (_end == _start);}
	  Str_J1850 Rx_J1850(void){
		 	 Str_J1850 _t = pElems[_start & (F_SIZE-1)];
		     _start = inc(_start);
		     return _t;
	 }

private:
	 static J1850_VPW* _instance;
	 uint16_t  LastTime;
	 volatile uint8_t *_p_tx,*_pin_rx;
	 size_t _tx_Len;
	 uint8_t _buftx[MAX_SIZE];
	 size_t error_rx;
	 Str_J1850 _bufrx;

	 static void ftm1_isr(void);
	 void  Time(void);
	 static void pin_isr(void);
	 void  PinRx(void);
	 uint8_t Crc8_J1850(uint8_t *buffer, size_t len);


	 //===========   fifo =======
	 int _start;
	 int _end;
	 Str_J1850 *pElems;
	 inline int isFull()  { return (_end == (_start ^ F_SIZE));}
	 inline void write(Str_J1850 &value) {
		 	pElems[_end & (F_SIZE-1)] = value;
		     if (isFull()) { /* full, overwrite moves start pointer */
		         _start = inc(_start);
		     	 }
		        _end = inc(_end);
	 	 }

	 inline int inc(int p) {return (p + 1)&(2 * F_SIZE-1);}


};

#endif
#endif /* J1850VPW_H_ */
