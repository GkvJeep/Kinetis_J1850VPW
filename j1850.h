/*
 * j1850.h
 *
 *  Created on: 26 но€б. 2017 г.
 *      Author: Piton
 */

#ifndef J1850_H_
#define J1850_H_

#include "Arduino.h"



#define ACTIVE 1
#define PASSIVE 0
#define F_TIC  15
// Timing for a long bit pulse
#define LONGBIT_TIME   128  //Nominal
#define LONGBIT_TIME_R (LONGBIT_TIME * F_TIC/10)
#define LONGBIT_DEV     16
#define LONGBIT_DEV_R   (LONGBIT_DEV * F_TIC/10)


// Timing for short bit pulse
#define SHORTBIT_TIME   64  //Nominal
#define SHORTBIT_TIME_R (SHORTBIT_TIME * F_TIC/10)
#define SHORTBIT_DEV    15
#define SHORTBIT_DEV_R  (SHORTBIT_DEV * F_TIC/10)

// Timing for start of frame
#define SOF_TIME      200     //Nominal
#define SOF_TIME_R    (SOF_TIME * F_TIC/10 )
#define SOF_DEV        18
#define SOF_DEV_R     (SOF_DEV * F_TIC/10 )

// Timing for end of data frame
#define EOD_TIME       200   //Nominal
#define EOD_TIME_R     (EOD_TIME * F_TIC/10 )
#define EOD_DEV        18
#define EOD_DEV_R      (EOD_DEV * F_TIC/10) )
// Timing for end of frame
#define EOF_TIME      280   //Nominal
#define EOF_TIME_R	  (EOF_TIME * F_TIC/10 )
#define EOF_DEV        18
#define EOF_DEV_R     (EOF_DEV * F_TIC/10 )

// Timing for IN-FRAME RESPONSE
#define IFS_TIME      300   //Nominal
#define IFS_TIME_R		(IFS_TIME * F_TIC/10 )
#define IFS_DEV        20
#define IFS_DEV_R	   (IFS_DEV)
// Timing for BREAK
#define BREAK_TIME      300  //Nominal
#define BREAK_TIME_R	(BREAK_TIME * F_TIC/10 )
#define BREAK_DEV        20
#define BREAK_DEV_R     (BREAK_DEV * F_TIC/10) )


// Storage, max 11 data bytes + CRC
#define BUFSIZE_J1850 12


class J1850{
public:

	J1850(){};

	size_t Out_J1850(uint8_t *data,size_t len);
	void Init(size_t pin_In,size_t pin_Out);
	 static void  isr_Pin(void);
	 static void  Timer_isr(void);

private:
    uint8_t Crc8_J1850(uint8_t *buffer, size_t len);

    static bool fTx_Pending;
    static uint32_t  LastTime;
	static volatile uint8_t *_p_tx,*_p_rx;
	static size_t _tx_Len;
	static size_t _rx_Len;
	static uint8_t _bufrx[BUFSIZE_J1850],_buftx[BUFSIZE_J1850];
};

#endif /* J1850_H_ */
