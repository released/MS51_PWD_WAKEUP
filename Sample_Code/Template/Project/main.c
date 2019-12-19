/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright(c) 2019 Nuvoton Technology Corp. All rights reserved.                                         */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/

/***********************************************************************************************************/
/* Website: http://www.nuvoton.com                                                                         */
/*  E-Mail : MicroC-8bit@nuvoton.com                                                                       */
/*  Date   : Jan/21/2019                                                                                   */
/***********************************************************************************************************/

/************************************************************************************************************/
/*  File Function: MS51 DEMO project                                                                        */
/************************************************************************************************************/

#include "MS51_8K.h"

typedef enum{
	flag_ADC_DATA = 0 ,
	
	flag_DEFAULT	
}Flag_Index;

typedef enum{
	WKPS_1_1 = 0 ,
	WKPS_1_4 = 2 ,
	WKPS_1_16 = 4 ,
	WKPS_1_64 = 6 ,	
	
	WKPS_1_256 = 8 ,
	WKPS_1_512 = 9 ,
	WKPS_1_1024 = 10 ,
	WKPS_1_2048 = 11 ,

	WKPS_DEFAULT	
}WKPS_Index;

typedef enum{
	target_CH0 = 0 ,
	target_CH1 ,
	target_CH2 ,
	target_CH3 ,	
	
	target_CH4 ,
	target_CH5 ,
	target_CH6 ,
	target_CH7 ,

	target_CH_DEFAULT	
}Channel_Index;

typedef enum{
	GPIO_Wakeup = 0 ,
	GPIO_ADC ,
	GPIO_TIMER ,
	
	GPIO_DEFAULT	
}GPIO_Index;

#define GPIO_PIN_WAKEUP							(P30)
#define GPIO_PIN_ADC								(P05)
#define GPIO_PIN_TIMER							(P17)	//(P04)

/*
		FUNCTION ENABLE
*/

#define ENABLE_REDUCE_CODESIZE
//#define ENABLE_ADC_IRQ
//#define ENABLE_TIMER_IRQ
#define ENABLE_UART
#define ENABLE_WAKEUP


/*
		INTERFACE DEFINE
*/

//#define ENABLE_16MHz
#define ENABLE_24MHz

#if defined (ENABLE_16MHz)
#define SYS_CLOCK 								(16000000ul)
#elif defined (ENABLE_24MHz)
#define SYS_CLOCK 								(24000000ul)
#endif

#define ADC_RESOLUTION							(4096ul)
#define ADC_REF_VOLTAGE							(5000ul)//(3300ul)	//(float)(3.3f)

#define ADC_SAMPLE_COUNT 						(4ul)			// 8
#define ADC_SAMPLE_POWER 						(2ul)			//(5)	 	// 3	,// 2 ^ ?
//#define ADC_SAMPLETIME_MS						(50ul)

#define TIMER_5MIN								(1000ul*60*5)
//#define TIMER_2MIN								(1000ul*60*2)
//#define TIMER_1MIN								(1000ul*60*1)
#define TIMER_12S								(1000ul*12)
#define TIMER_4S									(1000ul*4)
//#define TIMER_50MS								(50ul)
//#define TIMER_100MS								(100ul)

#define ADDR_BASE 								(0x1E00)		// use last 512 byte of 8K flash
#define ADDRESS_ALARM      					(ADDR_BASE+1)
//#define ADDRESS_CNT	      						(ADDR_BASE+2)

#define WRITE_KEY	     						(0x24)			//customize define
#define ERASE_KEY	     						(0x00)

#define ABS(X)  									((X) > 0 ? (X) : -(X))

uint8_t BitFlag = 0;
#define BitFlag_ON(flag)							(BitFlag|=flag)
#define BitFlag_OFF(flag)							(BitFlag&=~flag)
#define BitFlag_READ(flag)							((BitFlag&flag)?1:0)
#define ReadBit(bit)								(uint8_t)(1<<bit)

/*
		INTERFACE VARIABLE
*/


//UART 0
bit BIT_TMP;
bit BIT_UART;
bit uart0_receive_flag=0;
unsigned char uart0_receive_data;

//TIMER 0
uint8_t 	TH0_Tmp = 0;
uint8_t 	TL0_Tmp = 0;

//ADC
double  Bandgap_Voltage,AVdd,Bandgap_Value;      //please always use "double" mode for this
xdata unsigned char ADCdataVBGH, ADCdataVBGL;
//unsigned long int movingAverage_Target = 0;
unsigned long int movingAverageSum_Target = 0;
uint16_t adc_data = 0;

//Misc
volatile unsigned char xdata page_buffer[128];
uint8_t data_RWK = 0;

uint8_t is_flag_set(Flag_Index idx)
{
	return BitFlag_READ(ReadBit(idx));
}

void set_flag(Flag_Index idx , uint8_t en)
{
	if (en)
	{
		BitFlag_ON(ReadBit(idx));
	}
	else
	{
		BitFlag_OFF(ReadBit(idx));
	}
}

uint16_t get_ADCModifiedMovingAverage (void)
{
	static uint16_t cnt = 0;
	uint16_t movingAverage_Target = 0;

	#if defined (ENABLE_ADC_IRQ)
	if (is_flag_set(flag_ADC_DATA))
	{
		set_flag(flag_ADC_DATA , Disable);

		movingAverageSum_Target += adc_data;
		if (cnt++ >= (ADC_SAMPLE_COUNT-1))
		{
			cnt = 0;
			movingAverage_Target = (uint16_t)(movingAverageSum_Target >> ADC_SAMPLE_POWER) ;	//	/ADC_SAMPLE_COUNT;;
		}		
	}	
	#else

	for (cnt = 0 ; cnt < ADC_SAMPLE_COUNT ; cnt++)
	{
		while(ADCF == 0);
		movingAverageSum_Target += (((ADCRH<<4) + ADCRL)>>1)<<1;
	}
	movingAverage_Target = (uint16_t)(movingAverageSum_Target >> ADC_SAMPLE_POWER) ;
	
	#endif

	return movingAverage_Target;
}



uint16_t get_ADCConvertChannel(void)
{
	volatile uint16_t adc_value = 0;
//	volatile uint16_t duty_value = 0;
	
	adc_value = get_ADCModifiedMovingAverage();

//	adc_value = (adc_value <= ADC_CONVERT_TARGET) ? (ADC_CONVERT_TARGET) : (adc_value); 
//	adc_value = (adc_value >= ADC_RESOLUTION) ? (ADC_RESOLUTION) : (adc_value); 

	
//	printf("DUTY:%2d,ADC : %4d ,ADC within range:%4d (VREF : %e v)\r\n",duty_value , adc_value, target_value ,ADC_CALC_DATA_TO_VOLTAGE(adc_value,AVdd) );
//	printf("ADC: 0x%4X (%e v)\r\n",adc_value , ADC_CALC_DATA_TO_VOLTAGE(adc_value,AVdd));	
//	printf("ADC: 0x%4X (%e ,%e)\r\n",target_value , Bandgap_Voltage,AVdd);

	set_ADCCON0_ADCS; //after convert , trigger again

	return adc_value;
}

uint8_t read_APROM_BYTE(uint16_t code *u16_addr)
{
	  uint8_t rdata;
	  rdata = *u16_addr>>8;
	  return rdata;
}

void write_APROM_BYTE(uint16_t u16_addr,uint8_t u8_data)
{
	uint8_t looptmp=0;
	uint16_t u16_addrl_r;

	uint16_t tmp=0;
	//Check page start address
	u16_addrl_r=(u16_addr/128)*128;
	//Save APROM data to XRAM0
	for(looptmp=0;looptmp<0x80;looptmp++)
	{
		tmp = read_APROM_BYTE((unsigned int code *)(u16_addrl_r+looptmp));
		page_buffer[looptmp]=tmp;
	}
	// Modify customer data in XRAM
	page_buffer[u16_addr&0x7f] = u8_data;

	//Erase APROM DATAFLASH page
	IAPAL = u16_addrl_r&0xff;
	IAPAH = (u16_addrl_r>>8)&0xff;
	IAPFD = 0xFF;
	set_CHPCON_IAPEN; 
	set_IAPUEN_APUEN;
	IAPCN = 0x22;     
	set_IAPTRG_IAPGO; 

	//Save changed RAM data to APROM DATAFLASH

	set_CHPCON_IAPEN; 
	set_IAPUEN_APUEN;
	IAPCN = 0x21;
	for(looptmp=0;looptmp<0x80;looptmp++)
	{
		IAPAL = (u16_addrl_r&0xff)+looptmp;
		IAPAH = (u16_addrl_r>>8)&0xff;
		IAPFD = page_buffer[looptmp];
		set_IAPTRG_IAPGO;      
	}
	clr_IAPUEN_APUEN;
	clr_CHPCON_IAPEN;
}  


void set_gpio_Function(GPIO_Index idx , uint8_t on)
{
	switch(idx)
	{
		case GPIO_Wakeup:
			GPIO_PIN_WAKEUP = (on == Enable) ? (Enable) : (Disable) ;
			break;

		case GPIO_ADC:
			GPIO_PIN_ADC = (on == Enable) ? (Enable) : (Disable) ;
			break;

		case GPIO_TIMER:
			GPIO_PIN_TIMER = (on == Enable) ? (Enable) : (Disable) ;
			break;				
	}
}

void read_ADCAVdd(void)
{
    UINT8 BandgapHigh,BandgapLow,BandgapMark;
    double bgvalue;

	/*Read bandgap value */	
    set_CHPCON_IAPEN;
    IAPCN = READ_UID;
    IAPAL = 0x0d;
    IAPAH = 0x00;
    set_IAPTRG_IAPGO;
    BandgapLow = IAPFD;
    BandgapMark = BandgapLow&0xF0;
    BandgapLow = BandgapLow&0x0F;
    IAPAL = 0x0C;
    IAPAH = 0x00;
    set_IAPTRG_IAPGO;
    BandgapHigh = IAPFD;
    Bandgap_Value = (BandgapHigh<<4)+BandgapLow;
    Bandgap_Voltage= Bandgap_Value*3/4;
    clr_CHPCON_IAPEN;

	/* ADC Low speed initial*/  
    ENABLE_ADC_BANDGAP;
    ADCCON1|=0x30;            /* clock divider */
    ADCCON2|=0x0E;            /* AQT time */
    AUXR1|=SET_BIT4;          /* ADC clock low speed */
	
	/*start bandgap ADC */
    clr_ADCCON0_ADCF;
    set_ADCCON0_ADCS;                                
    while(ADCF == 0);
    ADCdataVBGH = ADCRH;
    ADCdataVBGL = ADCRL;
	
	/* to convert VDD value */
    bgvalue = (ADCRH<<4) + ADCRL;
    AVdd = (0x1000/bgvalue)*Bandgap_Voltage;

//    printf ("\r\n BG Voltage = %e\r\n", Bandgap_Voltage); 
//    printf ("\r\n VDD voltage = %e\r\n", AVdd); 	
}


void init_ADCMMA(void)
{
	movingAverageSum_Target = 0;
	set_flag(flag_ADC_DATA , Disable);
}

#if defined (ENABLE_ADC_IRQ)
void ADC_ISR(void) interrupt 11          // Vector @  0x5B
{
	//	adc_data = ((ADCRH<<4) + ADCRL);	
	adc_data = (((ADCRH<<4) + ADCRL)>>1)<<1;

	set_flag(flag_ADC_DATA , Enable);	
    clr_ADCCON0_ADCF; //clear ADC interrupt flag
}
#endif

void init_ADCChannel(Channel_Index CH)	//P0.1 , ADC_CH7
{
	read_ADCAVdd();

	switch(CH)
	{
		case target_CH7: 
		    ENABLE_ADC_CH7;
			break;	
	
		#if defined (ENABLE_REDUCE_CODESIZE)
		case target_CH0: 
		    ENABLE_ADC_CH0;
			break;

		case target_CH1: 
		    ENABLE_ADC_CH1;
			break;

//		case target_CH2: 
//		    ENABLE_ADC_CH2;
//			break;

		case target_CH3: 
		    ENABLE_ADC_CH3;
			break;

		case target_CH4: 
		    ENABLE_ADC_CH4;
			break;

		case target_CH5: 
		    ENABLE_ADC_CH5;
			break;

		case target_CH6: 
		    ENABLE_ADC_CH6;
			break;
		#endif		
	}

  /* ADC Low speed initial*/  
    ADCCON1|=0X30;            /* clock divider */
    ADCCON2|=0X0E;            /* AQT time */

	#if 0
    AUXR1|=SET_BIT4;          /* ADC clock low speed */
	#else
    AUXR1 &= ~SET_BIT4;			//high speed , 500k sps
	#endif

	clr_ADCCON0_ADCF;
	set_ADCCON0_ADCS;                  // ADC start trig signal

	#if defined (ENABLE_ADC_IRQ)	// Enable ADC interrupt (if use interrupt)
    set_IE_EADC;                        
    ENABLE_GLOBAL_INTERRUPT;
	#endif

	init_ADCMMA();

//	return ((ADCRH<<4) + ADCRL);

}

void Timer0_Delay(unsigned long u32SYSCLK, unsigned int u16CNT, unsigned int u16DLYUnit)
{
	unsigned char TL0TMP, TH0TMP;

	TIMER0_FSYS_DIV12;                                  //T0M=0, Timer0 Clock = Fsys/12
	ENABLE_TIMER0_MODE1;                                   //Timer0 is 16-bit mode
	TL0TMP = LOBYTE(65535-((u32SYSCLK/1000000)*u16DLYUnit/12));
	TH0TMP = HIBYTE(65535-((u32SYSCLK/1000000)*u16DLYUnit/12));
  
	while (u16CNT != 0)
	{
		TL0=TL0TMP;
		TH0=TH0TMP;
		set_TCON_TR0;                                    //Start Timer0
		while (!TF0);                       //Check Timer0 Time-Out Flag
		clr_TCON_TF0;
		clr_TCON_TR0;                       //Stop Timer0
		u16CNT --;
	}
//    clr_TCON_TR0;                                     //Stop Timer0
}

#if defined (ENABLE_TIMER_IRQ)
void Timer0_IRQHandler(void)
{
	// insert function , 1 ms IRQ

}

void Timer0_ISR(void) interrupt 1        // Vector @  0x0B
{
    TH0 = TH0_Tmp;
    TL0 = TL0_Tmp;
    clr_TCON_TF0;
	
	Timer0_IRQHandler();
}

void init_TIMER0(void)
{
	ENABLE_TIMER0_MODE1;
	
	TH0_Tmp = HIBYTE(TIMER_DIV12_VALUE_1ms_FOSC_240000);
	TL0_Tmp = LOBYTE(TIMER_DIV12_VALUE_1ms_FOSC_240000); 

    TH0 = TH0_Tmp;
    TL0 = TL0_Tmp;

    ENABLE_TIMER0_INTERRUPT;                       //enable Timer0 interrupt
    ENABLE_GLOBAL_INTERRUPT;                       //enable interrupts
  
    set_TCON_TR0;                                  //Timer0 run
}

#endif

#if defined (ENABLE_WAKEUP)
void WakeUp_Timer_ISR (void)   interrupt 17     //ISR for self wake-up timer
{
	set_gpio_Function(GPIO_Wakeup,Enable);	
	
	ENABLE_UART0_INTERRUPT;	
    clr_WKCON_WKTF;	//clr_WKTF;               	//clear interrupt flag
}

void set_WakeupTimer(uint8_t R)
{
    clr_WKCON_WKTR;
    RWK = 0xFF;                      		//  if prescale is 0x00, never set RWK = 0xff
    RWK = 256 - R;
    ENABLE_WKT_INTERRUPT;                 // enable WKT interrupt
    ENABLE_GLOBAL_INTERRUPT;

	clr_WKCON_WKTF;
    set_WKCON_WKTR;                    	// Wake-up timer run 	
}

void init_WakeupTimer(WKPS_Index idx_PSC , uint16_t msec)
{
	/*
		WKT pre-scalar 
		These bits determine the pre-scale of WKT clock. 
		000 = 1/1. 
		001 = 1/4. 
		010 = 1/16. 
		011 = 1/64. 
		100 = 1/256. 
		101 = 1/512. 
		110 = 1/1024. 
		111 = 1/2048. 
		
		timer base 10k, Pre-scale = 1/512
		10 000 / 512 = 19.53125 Hz = 0.0512 
		4000 msec = 0.0512 * (RWK) 	
		
	*/

	uint16_t LIRC = 10000;
	uint8_t psc = 0;
	uint16_t res = 0;

	switch(idx_PSC)
	{
		case WKPS_1_512:
			psc = SET_BIT2|SET_BIT0;	
			break;	
	
		#if defined (ENABLE_REDUCE_CODESIZE)
		case WKPS_1_1:
			psc = 0x00;
			break;
		case WKPS_1_4:
			psc = SET_BIT0;			
			break;			
		case WKPS_1_16:
			psc = SET_BIT1;
			break;			
		case WKPS_1_64:
			psc = SET_BIT1|SET_BIT0;
			break;			
		case WKPS_1_256:
			psc = SET_BIT2;			
			break;				
		case WKPS_1_1024:
			psc = SET_BIT2|SET_BIT1;				
			break;
		case WKPS_1_2048:
			psc = SET_BIT2|SET_BIT1|SET_BIT0;			
			break;
		#endif
			
	}

    WKCON = psc;	
	res = LIRC >> idx_PSC;
	res = (msec/1000)*res;
	data_RWK = res;		//	PSC = 512 , data_RWK = 77

}
#endif


#if defined (ENABLE_UART)
void send_UARTString(uint8_t* Data)
{
	#if 1
	uint16_t i = 0;

	while (Data[i] != '\0')
	{
		#if 1
		SBUF = Data[i++];
		#else
		UART_Send_Data(UART0,Data[i++]);	
		#endif
	}

	#endif

	#if 0
	uint16_t i = 0;
	
	for(i = 0;i< (strlen(Data)) ;i++ )
	{
		UART_Send_Data(UART0,Data[i]);
	}
	#endif

	#if 0
    while(*Data)  
    {  
        UART_Send_Data(UART0, (unsigned char) *Data++);  
    } 
	#endif
}

void send_UARTASCII(uint16_t Temp)
{
    uint8_t print_buf[16];
    uint16_t i = 15, j;

    *(print_buf + i) = '\0';
    j = (uint16_t)Temp >> 31;
    if(j)
        (uint16_t) Temp = ~(uint16_t)Temp + 1;
    do
    {
        i--;
        *(print_buf + i) = '0' + (uint16_t)Temp % 10;
        (uint16_t)Temp = (uint16_t)Temp / 10;
    }
    while((uint16_t)Temp != 0);
    if(j)
    {
        i--;
        *(print_buf + i) = '-';
    }
    send_UARTString(print_buf + i);
}

void Serial_ISR (void) interrupt 4 
{
    if (RI)
    {   
      uart0_receive_flag = 1;
      uart0_receive_data = SBUF;
      clr_SCON_RI;                                         // Clear RI (Receive Interrupt).
    }
    if  (TI)
    {
      if(!BIT_UART)
      {
          TI = 0;
      }
    }
}

void init_UART0(void)
{
	#if 1
	unsigned long u32Baudrate = 115200;
	P06_QUASI_MODE;    //Setting UART pin as Quasi mode for transmit
	SCON = 0x50;          //UART0 Mode1,REN=1,TI=1
	set_PCON_SMOD;        //UART0 Double Rate Enable
	T3CON &= 0xF8;        //T3PS2=0,T3PS1=0,T3PS0=0(Prescale=1)
	set_T3CON_BRCK;        //UART0 baud rate clock source = Timer3
	
	#if defined (ENABLE_16MHz)
	RH3    = HIBYTE(65536 - (1000000/u32Baudrate)-1);  
	RL3    = LOBYTE(65536 - (1000000/u32Baudrate)-1);  
	#elif defined (ENABLE_24MHz)
	RH3    = HIBYTE(65536 - (SYS_CLOCK/16/u32Baudrate));  
	RL3    = LOBYTE(65536 - (SYS_CLOCK/16/u32Baudrate));  
	#endif
	
	set_T3CON_TR3;         //Trigger Timer3
	set_IE_ES;

	ENABLE_GLOBAL_INTERRUPT;

	set_SCON_TI;
	BIT_UART=1;
	#else
    UART_Open(SYS_CLOCK,UART0_Timer3,115200);
    ENABLE_UART0_PRINTF;   
	#endif
}
#endif

#if defined (ENABLE_16MHz)
void MODIFY_HIRC_16(void)
{
    unsigned char data hircmap0,hircmap1;
    set_CHPCON_IAPEN;
    IAPAL = 0x30;
    IAPAH = 0x00;
    IAPCN = READ_UID;
    set_IAPTRG_IAPGO;
    hircmap0 = IAPFD;
    IAPAL = 0x31;
    IAPAH = 0x00;
    set_IAPTRG_IAPGO;
    hircmap1 = IAPFD;
    clr_CHPCON_IAPEN;
    TA=0XAA;
    TA=0X55;
    RCTRIM0 = hircmap0;
    TA=0XAA;
    TA=0X55;
    RCTRIM1 = hircmap1;
}

#elif defined (ENABLE_24MHz)
void MODIFY_HIRC_24(void)
{
    unsigned char data hircmap0,hircmap1;
/* Check if power on reset, modify HIRC */
    if (PCON&SET_BIT4)
    {
        set_CHPCON_IAPEN;
        IAPAL = 0x38;
        IAPAH = 0x00;
        IAPCN = READ_UID;
        set_IAPTRG_IAPGO;
        hircmap0 = IAPFD;
        IAPAL = 0x39;
        IAPAH = 0x00;
        set_IAPTRG_IAPGO;
        hircmap1 = IAPFD;
        clr_CHPCON_IAPEN;
        TA=0XAA;
        TA=0X55;
        RCTRIM0 = hircmap0;
        TA=0XAA;
        TA=0X55;
        RCTRIM1 = hircmap1;
        clr_CHPCON_IAPEN;
    }
}

#endif

void flow_ReportDataToUart(uint16_t arg)
{
	send_UARTASCII(arg);
	send_UARTString("\r\n");	
}

void flow_EntryLowPower(void)
{
	#if defined (ENABLE_WAKEUP)		
	set_WakeupTimer(data_RWK);

	#if 1
	ALL_GPIO_QUASI_MODE;
	DISABLE_UART0_INTERRUPT;	
	DISABLE_BOD;
	set_PCON_PD;
	
	#else
	ENABLE_GLOBAL_INTERRUPT;
	set_PCON_IDLE;
	#endif
	
	#endif	
}


void main (void) 
{
	#if defined (ENABLE_16MHz)
	MODIFY_HIRC_16();
	#elif defined (ENABLE_24MHz)
    MODIFY_HIRC_24();
	#endif

    ALL_GPIO_QUASI_MODE;
    ENABLE_GLOBAL_INTERRUPT;
	
	#if defined (ENABLE_WAKEUP)	
 	init_WakeupTimer(WKPS_1_512,TIMER_4S);
	#endif
	
	#if defined (ENABLE_UART)
    init_UART0();
	send_UARTString("INIT UART\r\n");

	// send ASCII
	flow_ReportDataToUart(57);

		// send string + ASCII
	send_UARTString("Show temperature : ");
	flow_ReportDataToUart(23);
	
	#endif
	
	init_ADCChannel(target_CH7);
	
    while(1)
    {		
		set_gpio_Function(GPIO_Wakeup,Disable);
				
		flow_EntryLowPower();

		#if defined (ENABLE_UART)
    	init_UART0();
		#endif

		set_gpio_Function(GPIO_TIMER,Disable);
		Timer0_Delay(SYS_CLOCK,100,1000);
		set_gpio_Function(GPIO_TIMER,Enable);

		set_gpio_Function(GPIO_ADC,Disable);		
		init_ADCChannel(target_CH7);
		flow_ReportDataToUart(get_ADCConvertChannel());
		set_gpio_Function(GPIO_ADC,Enable);
		
    }
}



