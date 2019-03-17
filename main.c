/*******************************************************************************
 * File Name          : main.c
 * Author             : zn007 and jose luu (modifs)
 * Date First Issued  : 09/30/2018
 * Description        : Main program body  8mhz cy
 ********************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_flash.h"
#include "stm32f10x_gpio.h"
#include "misc.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_spi.h"
#include "core_cm3.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define FIRMWARE_VERSION 119
/* Private macro -------------------------------------------------------------*/
#define adf4351_LD1 GPIOB->BSRR= GPIO_Pin_1 
#define adf4351_LD2 GPIOB->ODR ^= GPIO_Pin_1 //
/* Private variables ---------------------------------------------------------*/

#define LE1()      {GPIOB->CRH&=0XFFF0FFFF;GPIOB->CRH|=0x00030000;} 
#define LE2()    {GPIOA->CRL&=0XFFF0FFFF;GPIOA->CRL|=0x00030000;}

#define adf4351_LE        PBout(12)
#define adf4351_LEA        PAout(4)

#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 

#define GPIOA_ODR_Addr    (GPIOA_BASE+12) 
#define GPIOB_ODR_Addr    (GPIOB_BASE+12) 
#define GPIOA_IDR_Addr    (GPIOA_BASE+8) 
#define GPIOB_IDR_Addr    (GPIOB_BASE+8) 

#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)   
#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n) 

uint16_t adcx=0;
uint16_t adcx1=0;
unsigned char flag76=0;          
unsigned char flag73=0;         
unsigned char flag6D=0; 
unsigned char flag=0;
unsigned char flaga=1;
 
unsigned char len0=0;            
unsigned char Buff[26];        
ErrorStatus HSEStartUpStatus;
    
/* Private function prototypes -----------------------------------------------*/
void RCC_Configuration(void);
void GPIO_Configuration(void);
void NVIC_Configuration(void);
void USART_Configuration1(void);
void Adc_Init(void);
void SPI2_Conf(void);
void SPI1_Conf(void);
///////////////////////////////////////////////////////////////////////////
void delay_us(int uSec)
{
	volatile uint32_t cycles = (SystemCoreClock / 1000000L)*uSec;
	volatile uint32_t start = DWT->CYCCNT;
	do {
	} while (DWT->CYCCNT - start < cycles);
}
///////////////////////////////////////////////////////////////////////////
void delay_ms(int mSec)
{
	uint32_t uSec = mSec * 1000;
	volatile uint32_t cycles = (SystemCoreClock / 1000000L)*uSec;
	volatile uint32_t start = DWT->CYCCNT;
	do {
	} while (DWT->CYCCNT - start < cycles);
}

/////////////////////////////////////////////////////////////
void WriteToADF4351(unsigned long buf)
{
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(SPI2, ((unsigned char)(buf>>24)));
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(SPI2, ((unsigned char)(buf>>16)));
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(SPI2, ((unsigned char)(buf>>8)));
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(SPI2, ((unsigned char)buf));
	LE1();adf4351_LE=1;
	LE1();adf4351_LE=0;
}
/////////////////////////////////////////////////////////////
void WriteToADF4351A(unsigned long buf)                            ///SPI1
{
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(SPI1, ((unsigned char)(buf>>24)));
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(SPI1, ((unsigned char)(buf>>16)));
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(SPI1, ((unsigned char)(buf>>8)));
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(SPI1, ((unsigned char)buf));
	LE2();adf4351_LEA=1;
	LE2();adf4351_LEA=0;
}

//R1
//(DB14:DB3) modulus value
//(DB26:DB15) phase value
//(DB27) prescaler
//DB28) phase adjust
#define R1_ADF_OFF 0x08008011 // modulus=2 phase:1 prescaler:1
#define R1_ADF_ON  0x0800E1A9 // modulus=3125   phase:3 prescaler: 1(8/9)
//R2
//(DB5) powerdown
//(DB6) pdpolarity
//(DB12:DB9) chargepump
//(DB23:DB14) rcounter
#define R2_ADF_OFF 0x00004E62 // powerdown:1 pdpolarity:1 chargepump:7 rcounter:1
#define R2_ADF_ON  0x00004E42 // pdpolarity:1 positive, chargepump:7 2.5ma, rcounter:1
//R3
//(DB14:DB3) clockdivider
#define R3_ADF 0x000004B3 // clock divider:150
//R4
//(DB4:DB3) output level
//(DB5) output enable
//(DB19:DB12) bandselectclockdivider BSCD
//(DB22:DB20) RFdividerselect
//(DB23) feedback select
#define R4_ADF 0x008C803C // output level:3 5dbm, outputEn:1, BSCD: 200 , RFD:6 ,64 feedback:1
//R5
//(DB23:DB22) LDpinmode
#define R5_ADF 0x00580005 //LDpinmode:1 digital lock detect

//////////////////////////////////////////////////////////////////////
void adf4351_wr_serial(unsigned long wr0,unsigned long wr1,unsigned long wr2,unsigned long wr4)  
{
	WriteToADF4351(R5_ADF);		// Bit 22: digital lock detect
	WriteToADF4351(wr4);		
	WriteToADF4351(R3_ADF);	
	WriteToADF4351(wr2);		                    
	WriteToADF4351(wr1);	                      
	WriteToADF4351(wr0);		                
}
//////////////////////////////////////////////////////////////////////
void adf4351_wr_serialA(unsigned long wr0,unsigned long wr1,unsigned long wr2,unsigned long wr4)  
{
	WriteToADF4351A(R5_ADF);		
	WriteToADF4351A(wr4);		
	WriteToADF4351A(R3_ADF);	
	WriteToADF4351A(wr2);		                    
	WriteToADF4351A(wr1);	                      
	WriteToADF4351A(wr0);		                
}
///////////////////////////////////////////////////////////////////
void USART1_IRQHandler(void)          
{
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET){
		Buff[len0++]= USART_ReceiveData(USART1); 
		if(Buff[0]==0x8F){  // framing prefix
			switch (Buff[1]){
			case  'a':     //   idem 'x' (log) sweep with us delay between measurement and audiosens, 27 bytes command, returns n x 2 x 2 bytes
				break;
			case  'b':     //   idem 'w' (linear) sweep with us delay between measurement and audiosens, 27 bytes command, returns n x 2 x 2 bytes
				break;
			case  'c':     //   idem 'x' (log) sweep with us delay between measurement, 25 bytes command, returns n x 2 x 2 bytes
				break;
			case  'd':     //   idem 'w' (linear) sweep with us delay between measurement, 25 bytes command, returns n x 2 x 2 bytes
				break;
			case  'e':		//   pll adjust initially for AD9951
				break;
			case  'f':		//   vfo 10 bytes, no return
				break;  
			case  'm':		//   read measure value now, 1 byte command, returns 4 bytes
				flag6D++;
				Buff[0]=0;
				Buff[1]=0;
				len0=0;
				break;
			case  'n':		//   read measure value now (log?), 1 byte command, returns 4 bytes
				break;
				//  'o'  activate SWV relay, 2 byte command o0 or o1, no return
				//  'r'  attenuators setting, 3 bytes commannd, no return
			case  's':		//  status, 1 byt command 4 bytes return (version, atten, lo-hi of pin4 ADC measure)
				flag73++; 
				Buff[0] = 0;
				Buff[1] = 0;
				len0 = 0;
				break;
			case  'v':		//   get firmware version, 1 byte command, returns 1 byte
				flag76++; 
				Buff[0] = 0;
				Buff[1] = 0;
				len0 = 0;
				break;
			case  'w':		//  sweep AD8361 (linear RMS) 22 bytes command, returns  n x 2 x 2 bytes
				break;
			case 'x':		//  sweep AD8307 (log) 22 bytes command, returns  n x 2 x 2 bytes
				break;           
			case  'z':		//  counter, 9 bytes command, returns binary on 4 bytes
				break;              
			default:
				break;
			} 
		} else {
			Buff[0]=0;
			Buff[1]=0;
			len0=0;
		}          
	}
}
////////////////////////////////////////////////////////////////////////
void ADC1_2_IRQHandler(void)
{
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
	adcx1=(((ADC_GetConversionValue(ADC1))>>2)-80);
	ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
}

typedef enum {tracking,analyzer} e_whichADF;
void adf4351_wr(e_whichADF whichADF, unsigned int wr0, unsigned int wr1, unsigned int wr2, unsigned int wr4)
{
	switch (whichADF) {
	case tracking:
		adf4351_wr_serial( wr0, wr1, wr2, wr4); 
		break;
	case analyzer:
		adf4351_wr_serialA( wr0, wr1, wr2, wr4); 
		break;
	default:
		break;
	}
}
		   
int generate_RF(e_whichADF whichADF, unsigned long Freq)
{
	unsigned long int Freqint;      
	unsigned long int Freqfrac;
	int count;
	unsigned int R0 = 0;

	if((Freq<3200000u)||(Freq>440000000u)){ // invalid frequency, turn off ADF
		count=1;
		adf4351_wr( whichADF, R0, R1_ADF_OFF, R2_ADF_OFF, R4_ADF);
	} else {
		count=0;
	}
	if((3199999u<Freq)&&( Freq<6875000u)){    //<68.75MHz
		Freqint=(Freq<<6)/2500000;
		Freqfrac=(Freq<<6)/800%3125;  
		R0= (Freqint<<15)+ (Freqfrac<<3);
		//   RF divisor 64 (value 6), 125Hz step size
		adf4351_wr(whichADF, R0, R1_ADF_ON, R2_ADF_ON, R4_ADF | 6<<20);
	}   else if((6874999u<Freq)&&(Freq<13750000u)){   //>=68.75mHz &  <137.5mHz
		Freqint=(Freq<<5)/2500000;
		Freqfrac=(Freq<<5)/800%3125;
		R0= (Freqint<<15)+ (Freqfrac<<3);
		// RF divisor 32 (value 5), 250Hz step size
		adf4351_wr(whichADF, R0, R1_ADF_ON, R2_ADF_ON, R4_ADF | 5<<20);
	}  else if((13749999u<Freq)&&(Freq<27500000u)){  //>=137.5mHz &  <275mHz
		Freqint=(Freq<<4)/2500000;
		Freqfrac=(Freq<<4)/800%3125;
		R0= (Freqint<<15)+ (Freqfrac<<3);
		// RF divisor 16 (value 4), 500Hz step size
		adf4351_wr(whichADF, R0, R1_ADF_ON, R2_ADF_ON, R4_ADF | 4<<20);
	} else if((27499999u<Freq)&&(Freq<55000000u)){         //>=275mHz   &  <550mHz  
		Freqint=(Freq<<3)/2500000;
		Freqfrac=(Freq<<3)/800%3125;
		R0= (Freqint<<15)+ (Freqfrac<<3);
		// RF divisor 8 (value 3), 1kHz step size
		adf4351_wr(whichADF, R0, R1_ADF_ON, R2_ADF_ON, R4_ADF | 3<<20);
	} else if((54999999u<Freq)&&(Freq<110000000u)){        //>=550mHz   &  <1100mHz
		Freqint=(Freq<<2)/2500000;
		Freqfrac=(Freq<<2)/800%3125;
		R0= (Freqint<<15)+ (Freqfrac<<3);
		// RF divisor 4 (value 2), 2kHz step size
		adf4351_wr(whichADF, R0, R1_ADF_ON, R2_ADF_ON, R4_ADF | 2<<20);
	}  else if((109999999u<Freq)&&(Freq<220000000u)){       //>=1100mHz  &  <2200mHz 
		Freqint=(Freq<<1)/2500000; 
		Freqfrac=(Freq<<1)/800%3125;
		R0= (Freqint<<15)+ (Freqfrac<<3);
		// RF divisor 2 (value 1), 4kHz step size
		adf4351_wr(whichADF, R0, R1_ADF_ON, R2_ADF_ON, R4_ADF | 1<<20);
	} else if(219999999u<Freq){                            //>=2200mHz 
		Freqint=Freq/2500000; 
		Freqfrac=Freq/800%3125;
		R0= (Freqint<<15)+ (Freqfrac<<3);
		// RF divisor 1 (value 0), 8k0Hz step size
		adf4351_wr(whichADF, R0, R1_ADF_ON, R2_ADF_ON, R4_ADF | 0<<20);
	}
	adf4351_LD1;  
	Buff[0]=0;
	Buff[1]=0;  
	len0=0;
	return count;
}
////////////////////////////////////////////////////////////////////
int main(void)
{
	unsigned char count=1;
	unsigned long count1=0;
	unsigned long cc=0;	
	unsigned long int FreqA;         
	unsigned long int FreqintA;       
	unsigned long int FreqfracA;      
	unsigned long int Freqj; 
	unsigned long int Freqs;
	unsigned long int Jjmps;        
	unsigned long int R0; 
	unsigned long int R0A; 
	unsigned long int n; 
	unsigned int Delayus;   
  
#ifdef DEBUG
	//debug();
#endif
  
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	// Enable hi resolution counter 
	DWT->CTRL &= ~0x00000001;
	DWT->CTRL |= 0x00000001;


	RCC_Configuration();
	NVIC_Configuration();
	GPIO_Configuration();
	USART_Configuration1();
	Adc_Init(); 
	SPI2_Conf();
	SPI1_Conf();
	// turn off both ADFs
	adf4351_wr(tracking,  0, 0x08008011,0x00004E62,0x00EC803C); 
	adf4351_wr(analyzer, 0, 0x08008011,0x00004E62,0x00EC803C); 
	
	while(1){
		if(((~GPIOA->IDR)& GPIO_IDR_IDR12)&&(flaga)){// determine button press (pin A12 is low)
			if(cc<10000){
				cc++; // anti-bounce
			}  else{
				cc=0;
				flag=!flag;
				flaga=0;
				if (!flag) {
					generate_RF(tracking, 0); // stop tracking generator in case it was on
				}
			}
		} 
		if((GPIOA->IDR)& GPIO_IDR_IDR12){
			flaga=1;
		}
		if(flag){
			GPIOA->BSRR = GPIO_Pin_11;
		} else{
			GPIOA->BRR = GPIO_Pin_11; // clear
		}  
		if(flag76>0){ // 'v' version
			while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET){} //wait until finished
			USART1->DR =( FIRMWARE_VERSION& (uint16_t)0x01FF);
			flag76--;
		} else if(flag73>0){  // 's' status
			while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET){} //wait until finished
			USART1->DR =( 0x0A& (uint16_t)0x01FF);
			while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET){} //wait until finished 
			USART1->DR =( 0x00& (uint16_t)0x01FF); 
			while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET){} //wait until finished
			USART1->DR =( 0x00& (uint16_t)0x01FF); 
			while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET){} //wait until finished
			USART1->DR =( 0x00& (uint16_t)0x01FF); 
			flag73--;
		}  else if(flag6D>0) { // 'm'  measure now
			adcx=adcx1;
			while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET){} 
			USART1->DR =(((unsigned char*)&adcx)[0]& (uint16_t)0x01FF);                  
			while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET){} 
			USART1->DR =(((unsigned char*)&adcx)[1]& (uint16_t)0x01FF);
			while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET){} 
			USART1->DR =(((unsigned char*)&adcx)[0]& (uint16_t)0x01FF);
			while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET){} 
			USART1->DR =(((unsigned char*)&adcx)[1]& (uint16_t)0x01FF); 
			flag6D--;
		} else if((Buff[0]==0x8F)&&(Buff[1]==0x66)&&(len0==10)){  //  'f'  vfo mode
			Freqs=((unsigned long)(Buff[2]-0x30)*100000000+
				   (unsigned long)(Buff[3]-0x30)*10000000+
				   (unsigned long)(Buff[4]-0x30)*1000000+
				   (unsigned long)(Buff[5]-0x30)*100000+
				   (unsigned long)(Buff[6]-0x30)*10000+
				   (unsigned long)(Buff[7]-0x30)*1000+
				   (unsigned long)(Buff[8]-0x30)*100+
				   (unsigned long)(Buff[9]-0x30)*10+
				   (unsigned long)(Buff[10]-0x30)); 
			generate_RF(tracking, Freqs);
			generate_RF(analyzer, Freqs);
		} else if((Buff[0]==0x8F)&&(Buff[1]==0x78)&&(len0==22)){    // 'x' sweep
			Freqs=((unsigned long)(Buff[2]-0x30)*100000000+
				   (unsigned long)(Buff[3]-0x30)*10000000+
				   (unsigned long)(Buff[4]-0x30)*1000000+
				   (unsigned long)(Buff[5]-0x30)*100000+
				   (unsigned long)(Buff[6]-0x30)*10000+
				   (unsigned long)(Buff[7]-0x30)*1000+
				   (unsigned long)(Buff[8]-0x30)*100+
				   (unsigned long)(Buff[9]-0x30)*10+
				   (unsigned long)(Buff[10]-0x30));
			Freqj=((unsigned long)(Buff[18]-0x30) +
				   (unsigned long)(Buff[17]-0x30)*10+
				   (unsigned long)(Buff[16]-0x30)*100+
				   (unsigned long)(Buff[15]-0x30)*1000+
				   (unsigned long)(Buff[14]-0x30)*10000+
				   (unsigned long)(Buff[13]-0x30)*100000);
			Jjmps=((unsigned long)(Buff[22]-0x30)+
				   (unsigned long)(Buff[21]-0x30)*10+
				   (unsigned long)(Buff[20]-0x30)*100+
				   (unsigned long)(Buff[19]-0x30)*1000); 
			
			for(n=0 ; n<Jjmps ; n++) {
				FreqA=Freqs+Freqj*n;
				Freqs=FreqA+12000;
				if(flag){
					count=generate_RF(tracking,Freqs);
				} else{ //flag
					generate_RF(tracking,0); // 0 Freq will set ADF off
				}						
				generate_RF(analyzer,FreqA);
             
				delay_us(60000); //time delay, very important, the more the better
						 
				//if((adcx1<130)&&(flag)) adcx=130;	else adcx=adcx1;//喜欢本底线平直就把下条语句这么处理；没意思
				adcx=adcx1;		 
				while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET){} 
				USART1->DR =(((unsigned char*)&adcx)[0]& (uint16_t)0x01FF);                  
				while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET){} 
				USART1->DR =(((unsigned char*)&adcx)[1]& (uint16_t)0x01FF);
				while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET){} 
				USART1->DR =(((unsigned char*)&adcx)[0]& (uint16_t)0x01FF);
				while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET){} 
				USART1->DR =(((unsigned char*)&adcx)[1]& (uint16_t)0x01FF); 
				adf4351_LD1;
			} // end of sweep loop
			Buff[0]=0; 
			Buff[1]=0;  
			len0=0;
		} else if((Buff[0]==0x8F)&&(Buff[1]==0x61)&&(len0==25)){ // 'a'  sweep with delay between steps
			Freqs=((unsigned long)(Buff[2]-0x30)*100000000+
				   (unsigned long)(Buff[3]-0x30)*10000000+
				   (unsigned long)(Buff[4]-0x30)*1000000+
				   (unsigned long)(Buff[5]-0x30)*100000+
				   (unsigned long)(Buff[6]-0x30)*10000+
				   (unsigned long)(Buff[7]-0x30)*1000+
				   (unsigned long)(Buff[8]-0x30)*100+
				   (unsigned long)(Buff[9]-0x30)*10+
				   (unsigned long)(Buff[10]-0x30));
			Freqj=((unsigned long)(Buff[18]-0x30)+
				   (unsigned long)(Buff[17]-0x30)*10+
				   (unsigned long)(Buff[16]-0x30)*100+
				   (unsigned long)(Buff[15]-0x30)*1000+
				   (unsigned long)(Buff[14]-0x30)*10000+
				   (unsigned long)(Buff[13]-0x30)*100000);
			Jjmps=((unsigned long)(Buff[22]-0x30)+
				   (unsigned long)(Buff[21]-0x30)*10+
				   (unsigned long)(Buff[20]-0x30)*100+
				   (unsigned long)(Buff[19]-0x30)*1000);
			Delayus=(unsigned int)((Buff[23]-0x30)*100)+
				(unsigned int)((Buff[24]-0x30)*10)+
				(unsigned int)((Buff[25]-0x30)) ;
			ADC_SoftwareStartConvCmd(ADC1, ENABLE);
			for(n=0 ; n<Jjmps ; n++) {
				FreqA=Freqs+Freqj*n;
				Freqs=FreqA+12000; 
				if(flag){
					count=generate_RF(tracking,Freqs);
				} else { // flag
					generate_RF(tracking,0);
				}					
				generate_RF(analyzer,FreqA);

				delay_us(600);
//if((adcx1<130)&&(flag)) adcx=130;	else adcx=adcx1;//喜欢本底线平直就把下条语句这么处理；没意思
				adcx=adcx1;	 
				while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET){} 
				USART1->DR =(((unsigned char*)&adcx)[0]& (uint16_t)0x01FF);                  
				while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET){} 
				USART1->DR =(((unsigned char*)&adcx)[1]& (uint16_t)0x01FF);
				while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET){} 
				USART1->DR =(((unsigned char*)&adcx)[0]& (uint16_t)0x01FF);
				while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET){} 
				USART1->DR =(((unsigned char*)&adcx)[1]& (uint16_t)0x01FF); 
				delay_ms( Delayus/10);  //   multiply delay by 100
				adf4351_LD1;
			} // end of sweep loop
			ADC_SoftwareStartConvCmd(ADC1, DISABLE);
			Buff[0]=0;  
			Buff[1]=0;  
			len0=0;
		}	
		if(count){
			if(count1<120000){
				count1++;
			}else{
				count1=0;
				adf4351_LD2; 
			}
		}           
	}
}
		
/////////////////////////////////////////////////////////////////////////
void RCC_Configuration(void)
{
	ErrorStatus HSEStartUpStatus;
	
	RCC_DeInit();                //时钟管理重置 （复位成缺省值）
	RCC_HSEConfig(RCC_HSE_ON);   //打开外部晶振
	HSEStartUpStatus = RCC_WaitForHSEStartUp();  //等待外部晶振就绪
	if(HSEStartUpStatus == SUCCESS)   
    {
        RCC_HCLKConfig(RCC_SYSCLK_Div1);     //AHB使用系统时钟    72mhz
        RCC_PCLK2Config(RCC_HCLK_Div1);      //72mhz
        RCC_PCLK1Config(RCC_HCLK_Div2);      //36mhz
        FLASH_SetLatency(FLASH_Latency_2);
        FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
        RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);     //HD MD
        RCC_PLLCmd(ENABLE);
        while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)//等待PLL启动
        {
        }
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);        //将PLL设置成系统时钟源
        while(RCC_GetSYSCLKSource() != 0x08)  //检查是否将HSE_9倍频后作为系统时钟  0x08 PLL作为系统时钟，0X04 HSE作为系统时钟 0X00 HSI作为系统时钟
        {
        }
    }
  
	//Enable peripheral clocks 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA| RCC_APB2Periph_GPIOB,ENABLE);//使能AB时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 , ENABLE);
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);    //12MHZ 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);    
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);  //使能USART1时钟 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE ); 
}

//////////////////////////////////////////////////////////////////////////////
void GPIO_Configuration(void)                      ///初始化要用的端口
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_PinRemapConfig(GPIO_Remap_USART1,ENABLE);   //USART1进行重映射
  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;         // USART1 TX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;   //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
	GPIO_Init(GPIOB, &GPIO_InitStructure);            
    
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;        //USART1 RX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  //复用悬空输入 
	GPIO_Init(GPIOB, &GPIO_InitStructure);           //B端口
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_1;      /*初始化 GPIOB的 Pin_1为推挽输出*/    
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_10MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);  

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_11;      /*初始化 GPIOA的 Pin_11为推挽输出*/    
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_10MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure); 
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_12;      /*初始化 GPIOA的 Pin_12为上拉输入*/    
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_10MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure); 
	
	/* Configure PA.3 (ADC Channel) as analog input -------------------------*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	/*SPI1*/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_7;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	/*SPI2*/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_15;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
}
/////////////////////////////////////////////////////////////////////////////
void NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;        //很重要	

#ifdef  VECT_TAB_RAM  
	/* Set the Vector Table base location at 0x20000000 */ 
	NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0); 
#else  /* VECT_TAB_FLASH  */
	/* Set the Vector Table base location at 0x08000000 */ 
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);   
#endif
	/*Enable the USART1 Interrupt*/

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;    //抢占优先级0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;           //子优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;              //使能
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel =ADC1_2_IRQn ;  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;    //抢占优先级0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;           //子优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;              //使能
	NVIC_Init(&NVIC_InitStructure); 	

}
/////////////////////////////////////////////////////////////////////////////////
void USART_Configuration1(void)
{
	USART_InitTypeDef USART_InitStructure;         //串口初始化结构体声明
	USART_ClockInitTypeDef  USART_ClockInitStructure;
	
/* USART1 configuration ------------------------------------------------------*/
 
	USART_ClockInit(USART1, &USART_ClockInitStructure);             //配置USART与时钟相关的设置
	USART_InitStructure.USART_BaudRate = 57600;                      //
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;     //字长8位 
	USART_InitStructure.USART_StopBits = USART_StopBits_1;          //一位停止
	USART_InitStructure.USART_Parity = USART_Parity_No ;            //无奇偶校验
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;   //无流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //打开rx接收tx发送控制
	// Configure USART1 basic and asynchronous paramters 
	
	USART_Init(USART1, &USART_InitStructure);                       //初始化
	USART_Cmd(USART1, ENABLE);                                      //启动串口
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);                    //接收中断使能   很关键
}

//////////////////////////////////////////////////////////////////////////
void Adc_Init(void)
{ ADC_InitTypeDef ADC_InitStructure;
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;//
	ADC_InitStructure.ADC_ScanConvMode =DISABLE;         //关闭扫描模式，因为只有一个通道
	ADC_InitStructure.ADC_ContinuousConvMode =ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel =1;               //设置通道数量1
	ADC_Init(ADC1, &ADC_InitStructure);
	/* Enable ADC1 */
	ADC_Cmd(ADC1,ENABLE);
	/* Enable ADC1 reset calibration register */   
	ADC_ResetCalibration(ADC1);
	/* Check the end of ADC1 reset calibration register */
	while(ADC_GetResetCalibrationStatus(ADC1));
	/* Start ADC1 calibration */
	ADC_StartCalibration(ADC1);
	/* Check the end of ADC1 calibration */
	while(ADC_GetCalibrationStatus(ADC1));
	ADC_RegularChannelConfig(ADC1, 3, 1, ADC_SampleTime_239Cycles5);   //PA.3 
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	//ADC_TempSensorVrefintCmd(ENABLE);
	ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);
}
/////////////////////////////////////////////////////////////////////////////
void SPI1_Conf(void)
{SPI_InitTypeDef  SPI_InitStructure;
	SPI_Cmd(SPI1,DISABLE);
	SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Tx;          //单线TX
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;                      //主模式
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;                  //8位宽
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;                         //空闲低电平
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;                       //奇数边沿
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;                          //外部脚管理
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2; //波特率预分频 
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;                 //MSB
	SPI_InitStructure.SPI_CRCPolynomial = 7;                           //校验值7忽略校验
	SPI_Init(SPI1, &SPI_InitStructure); /*Enable SPI1.NSS as a GPIO*/
	//SPI_SSOutputCmd(SPI1, ENABLE);
	SPI_Cmd(SPI1, ENABLE);                                             //开启外设1
	//spi1_cs_low;
}
//////////////////////////////////////////////////////////////////////////////
void SPI2_Conf(void)
{SPI_InitTypeDef  SPI_InitStructure;
	SPI_Cmd(SPI2,DISABLE);
	SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Tx;           //单线TX
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;                       //主模式
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;                   //8位宽
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;                          //空闲低电平
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;                        //奇数边沿
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;                           //外部脚管理
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;  //波特率预分频                
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;                  //MSB
	SPI_InitStructure.SPI_CRCPolynomial = 7;                            //校验值7
	SPI_Init(SPI2, &SPI_InitStructure);                                 //
	SPI_Cmd(SPI2, ENABLE);                                              //开启外设2
}   

/******************* (C) COPYRIGHT 2007 STMicroelectronics *****END OF FILE****/
