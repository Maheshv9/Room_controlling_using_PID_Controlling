
/*# Room_controlling_using_PID_Controlling
 * 003_PID_Controlling.c
 *
 *  Created on: 27-Aug-2021
 *      Author: MAHESH
 */
#include"stm32f4xxx.h"
#include"I2C.h"
#include"OLED.h"
#include"math.h"
#include "font.h"
void PWM_Config(void);
void ADC_Setup(void);
void ADC_Enable(void);
void Start_conversion(void);
void ADC_WaitforConvertion(void);
uint32_t ADC_Data(void);

float temp(void);

float vol, temparature, low_pass=0.00;

float setpoint = 24, input_val = 0.000, PID_Output =0.000 , PID_P_gain =10, PID_I_gain = 0.02, PID_D_gain = 5.254 , Error_Signal , PID_input;

float PID_I_OutPut, PID_P_Output, PID_D_Output, PID_Output_gain=2.58, PID_Signal;

float Present_Error, Past_Error;

int main(void)
{
	char Temp[10], pid[10],E0[10], speed[10];
	float Fan_Speed;

	PWM_Config();
	USART_Init();
	I2C_Config();
    ADC_Setup();
    ADC_Enable();
    SSD1306_Init();
    temp();
    SSD1306_GotoXY(0, 0);//
	SSD1306_Puts("wait 5sec for PID ",&Font_7x10, 1);
	SSD1306_GotoXY(0, 20);
	SSD1306_Puts("Stabilization....",&Font_7x10, 1);
	SSD1306_UpdateScreen();
	for(int var=0; var<200000; var++)
	{
		PID_input = temp();
	}
	//Delay_ms(1000);
	SSD1306_ClearScreen();
	while(1)
    {
		PID_input = temp();


		Error_Signal = PID_input- setpoint;

		Present_Error = Error_Signal;

		PID_P_Output = PID_D_gain * Error_Signal;

		PID_I_OutPut = PID_I_gain * Error_Signal;

		PID_D_Output = PID_D_gain * ( Present_Error - Past_Error);

		PID_Signal  =  PID_P_Output + PID_I_OutPut + PID_D_Output ;

		PID_Output = PID_Signal * PID_Output_gain;

		if(PID_Output < 30) PID_Output = 30;
		else if(PID_Output > 255 ) PID_Output = 255;

		printdatafloat(Error_Signal);
		print("\n");

		TIM2->CCR1 = PID_Output;

		//Display the Temparature data
		sprintf(Temp,"%0.2f",PID_input);

		SSD1306_GotoXY(0, 0);//

		SSD1306_Puts("Temp:",&Font_7x10, 1);

		SSD1306_GotoXY(37, 0);//

		SSD1306_Puts(Temp,&Font_7x10, 1);


		//Display the Error signal data

		sprintf(E0,"%0.2f",Error_Signal);

		SSD1306_GotoXY(0, 14+3);//

		SSD1306_Puts("Error:",&Font_7x10, 1);

		SSD1306_GotoXY(43, 14+3);//

		SSD1306_Puts(E0,&Font_7x10, 1);


		//Display the pid_output data

		sprintf(pid,"%0.2f",PID_Output);

		SSD1306_GotoXY(0, 30+3);

		SSD1306_Puts("PID_OutPut:",&Font_7x10, 1);

		SSD1306_GotoXY(78, 30+3);

		SSD1306_Puts(pid,&Font_7x10, 1);

		//Display the Fan speed

		Fan_Speed = PID_Output /255 *100;

		sprintf(speed,"%0.2f",Fan_Speed);

		SSD1306_GotoXY(0, 60-10);

		SSD1306_Puts("FAN_Speed:",&Font_7x10, 1);

		SSD1306_GotoXY(71, 60-10);

		SSD1306_Puts(speed,&Font_7x10, 1);

		SSD1306_UpdateScreen();
		//SSD1306_ClearScreen();
		Past_Error = Present_Error;
    }
    return 0;
}
void PWM_Config(void)
{
    RCC->AHB1ENR |= 1;
    GPIOA->AFRL[0] |= 0x00100000;
    GPIOA->MODER |=  0x00000800; //
    RCC->APB1ENR |= 1; // enble the time2 clk
    TIM2->PSC = 1;//1250=50hz
    TIM2->ARR = 256-1; //arr = 255
    TIM2->CNT = 0; //timer is 0
    TIM2->CCMR1 = 0x0060;//pwm mode
    TIM2->CCER = 1;//enble the channel 1
    TIM2->CR1 = 1; //enable the timer2
}
void ADC_Setup(void)
{
	/* 1.clk for ADC1 and GPIOA
	 * 2.Prescaler 4 Divide
	 * 3.scan mode
	 * 4.set the 12Bit Resolution from CR1 write 0
	 * 5.data alingment Right CR1 Write 0
	 * 6.selcting only one channel thats why no need write the data on the SQR1
	 * 7.Selecting the cycle 3 so no need write
	 */
	RCC->AHB1ENR |=1;//GPIOA clk en
	RCC->APB2ENR |= 1 << 8;	//ADC1 Clk en
	GPIOA->MODER |= 3<<0;	//analog mode
	ADC->CCR     |= 1<<16;	//prescaler for clk
	ADC1->CR1    |= 1<<8;//enble the scan mode
	ADC1->CR2	 |= 1<<1;//continous convertion
	ADC1->CR2    |= 1<<10;

}
void ADC_Enable(void)
{
	/*
	 * ADC1 ENBLE
	 */
	ADC1->CR2 |=1;
}
void Start_conversion(void)
{
	/*
	 * Select the channel ch1
	 */
	ADC1->SQR3  = 0;
	ADC1->SQR3 |= 0;//channel 0
	ADC1->SR   = 0; //write 0 status Register
	ADC1->CR2 |=1<<30;
}
void ADC_WaitforConvertion(void)
{
	while(!(ADC1->SR & 1<<1));
}
uint32_t ADC_Data(void)
{
	return ADC1->DR;
}
float temp(void)
{

	uint32_t adc_data ;
	Start_conversion();
	ADC_WaitforConvertion();
	adc_data = ADC_Data();
	vol = (float)adc_data * 3.3;
	vol /=4096;
	temparature = (vol) * 100 - 2;

	 low_pass = 0.945 * low_pass + 0.0549 * temparature;//low pass filter
	 return low_pass;
}
