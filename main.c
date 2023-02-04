#include "NuMicro.h"
#include <math.h>

/*---------------------------------------------------------------------------------------------------------*/
/*Macro, type and constant definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define PLL_CLOCK 96000000
#define PDMAchannel1 1
#define ADC_dig 4095.0
#define A_vdd 3.3
#define M_PI 3.14159265358979323846

/*---------------------------------------------------------------------------------------------------------*/
/*Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

float grid_vadc = 0;
float grid_zero = 0;
float lsb_to_volt;

int16_t g_i32_Data[4] = { 0 };

uint32_t i;

float sine_table[361];
float cos_table[361];
float st_val, ct_val;

uint32_t degree;

float vd = 0;
float vq = 0;
float error;

float sogi_forward_integrator_output;
float sogi_backward_integrator_output;
float phase_output;
float loop_filter_kp;
float loop_filter_ki;
float angle_freq;
float update_period;
float center_angle_freq;
float loop_filter_sum;
float radian_to_degree;
float radian_of_period;

float k_gain_sogi;
float gamma;
float freq_radians;
float center_freq_radians;
float freq_Hz;
float fll_integrator;
float e, sogi_input, qv, v, temp, ef;

void SYS_Init(void)
{
	/*---------------------------------------------------------------------------------------------------------*/
	/*Init System Clock                                                                                       */
	/*---------------------------------------------------------------------------------------------------------*/
	//CLK->PWRCTL   = (CLK->PWRCTL   &~(0x0000000FUL)) | 0x0231001CUL;
	//CLK->PLLCTL   = (CLK->PLLCTL   &~(0x000FFFFFUL)) | 0x0008C03EUL;
	//CLK->CLKDIV0  = (CLK->CLKDIV0  &~(0x00FFFFFFUL)) | 0x00000000UL;
	//CLK->CLKDIV4  = (CLK->CLKDIV4  &~(0x00FFFFFFUL)) | 0x00000000UL;
	//CLK->PCLKDIV  = (CLK->PCLKDIV  &~(0x00000077UL)) | 0x00000000UL;
	//CLK->CLKSEL0  = (CLK->CLKSEL0  &~(0x0000013FUL)) | 0x0000001FUL;
	//CLK->CLKSEL1  = (CLK->CLKSEL1  &~(0x7777777FUL)) | 0x4477773BUL;
	//CLK->CLKSEL2  = (CLK->CLKSEL2  &~(0x0030033FUL)) | 0x00100328UL;
	//CLK->CLKSEL3  = (CLK->CLKSEL3  &~(0x77777700UL)) | 0x44444400UL;
	//CLK->AHBCLK   = (CLK->AHBCLK   &~(0x0000009EUL)) | 0x00000002UL;
	//CLK->APBCLK0  = (CLK->APBCLK0  &~(0x18FF33FFUL)) | 0x10000084UL;
	//CLK->APBCLK1  = (CLK->APBCLK1  &~(0x000F0300UL)) | 0x00030000UL;
	//CLK->CLKOCTL  = (CLK->CLKOCTL  &~(0x0000007FUL)) | 0x00000000UL;
	//SysTick->CTRL = (SysTick->CTRL &~(0x00000005UL)) | 0x00000004UL;
	//RTC->LXTCTL   = (RTC->LXTCTL   &~(0x00000080UL)) | 0x00000000UL;

	/*Unlock protected registers */
	SYS_UnlockReg();

	/*If the macros do not exist in your project, please refer to the related clk.h in Header folder of the tool package */
	/*Enable clock source */
	CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk | CLK_PWRCTL_HIRCEN_Msk);

	/*Waiting for clock source ready */
	CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk | CLK_STATUS_HIRCSTB_Msk);

	/*Disable PLL first to avoid unstable when setting PLL */
	CLK_DisablePLL();

	/*Set PLL frequency */
	CLK->PLLCTL = (CLK->PLLCTL &~(0x000FFFFF UL)) | 0x0008C03E UL;

	/*Waiting for PLL ready */
	CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);

	/*Set HCLK clock */
	CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

	/*Set PCLK-related clock */
	CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV1 | CLK_PCLKDIV_APB1DIV_DIV1);

	/*Enable IP clock */
	CLK_EnableModuleClock(ACMP01_MODULE);
	CLK_EnableModuleClock(ADC_MODULE);
	CLK_EnableModuleClock(PDMA_MODULE);
	CLK_EnableModuleClock(PWM0_MODULE);
	CLK_EnableModuleClock(PWM1_MODULE);
	CLK_EnableModuleClock(TMR0_MODULE);
	CLK_EnableModuleClock(TMR1_MODULE);

	/*Set IP clock */
	CLK_SetModuleClock(ADC_MODULE, CLK_CLKSEL2_ADCSEL_PLL, CLK_CLKDIV0_ADC(1));
	CLK_SetModuleClock(PWM0_MODULE, CLK_CLKSEL2_PWM0SEL_PLL, MODULE_NoMsk);
	CLK_SetModuleClock(PWM1_MODULE, CLK_CLKSEL2_PWM1SEL_PLL, MODULE_NoMsk);
	CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HIRC, MODULE_NoMsk);
	CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_HIRC, MODULE_NoMsk);

	/*Update System Core Clock */
	/*User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
	SystemCoreClockUpdate();

	/*Set multi-function pins for PWM Channels */
	SYS->GPC_MFPL = (SYS->GPC_MFPL &~(SYS_GPC_MFPL_PC5MFP_Msk |
			SYS_GPC_MFPL_PC4MFP_Msk)) |
		(SYS_GPC_MFPL_PC5MFP_PWM1_CH0 |
			SYS_GPC_MFPL_PC4MFP_PWM1_CH1);

	SYS->GPA_MFPL = (SYS->GPA_MFPL &~(SYS_GPA_MFPL_PA5MFP_Msk |
			SYS_GPA_MFPL_PA4MFP_Msk)) |
		(SYS_GPA_MFPL_PA5MFP_PWM0_CH0 |
			SYS_GPA_MFPL_PA4MFP_PWM0_CH1);

	/*Set PB.4 multi-function pin for ACMP1 positive input pin and PB6 multi-function pin for ACMP1 output pin */
	SYS->GPB_MFPL = (SYS->GPB_MFPL &~(SYS_GPB_MFPL_PB4MFP_Msk | SYS_GPB_MFPL_PB6MFP_Msk)) |
		(SYS_GPB_MFPL_PB4MFP_ACMP1_P1 | SYS_GPB_MFPL_PB6MFP_ACMP1_O);

	/*PB.4 ACMP and PB.0-1-7-8 ADC */
	GPIO_SetMode(PB, BIT0 | BIT1 | BIT4 | BIT7 | BIT8, GPIO_MODE_INPUT);

	/*Disable the PB0 ~ PB3 digital input path to avoid the leakage current. */
	GPIO_DISABLE_DIGITAL_PATH(PB, BIT0 | BIT1 | BIT4 | BIT7 | BIT8);

	/*Lock protected registers */
	SYS_LockReg();

	return;
}

void PDMA_Init()
{
	/*Configure PDMA peripheral mode form ADC to memory */
	/*Open PDMA Channel 1 based on PDMAchannel setting*/

	PDMA_Open(PDMA, 1 << PDMAchannel1);

	/*transfer width is half word(16 bit) and transfer count is ADCDatalenght+1 */
	PDMA_SetTransferCnt(PDMA, PDMAchannel1, PDMA_WIDTH_16, 4);
	PDMA_SetTransferAddr(PDMA, PDMAchannel1, (uint32_t) &ADC->ADPDMA, PDMA_SAR_FIX, (uint32_t) g_i32_Data, PDMA_SAR_FIX);
	PDMA_SetTransferMode(PDMA, PDMAchannel1, PDMA_ADC_RX, FALSE, 0);
	PDMA_SetBurstType(PDMA, PDMAchannel1, PDMA_REQ_SINGLE, 0);

	/*Set source address as ADC data register (no increment) and destination address as g_i32ConversionData array (increment) */

}

void ReloadPDMA()
{
	/*transfer width is half word(16 bit) and transfer count is ADCDatalenght+1 */
	PDMA_SetTransferCnt(PDMA, PDMAchannel1, PDMA_WIDTH_16, 4);
	PDMA_SetTransferMode(PDMA, PDMAchannel1, PDMA_ADC_RX, FALSE, (uint32_t) NULL);

}

void lookUpTable()
{
	for (i = 0; i <= 360; i++)
	{
		st_val = sin(M_PI *i / 180);
		ct_val = cos(M_PI *i / 180);
		sine_table[i] = st_val;
		cos_table[i] = ct_val;
	}
}

void init_pll_sogi(float k_gain, float gam, float kp, float ki, float dt, float fc)
{
	center_angle_freq = fc *2 * M_PI;
	angle_freq = center_angle_freq;
	update_period = dt;
	loop_filter_ki = ki * update_period;
	loop_filter_kp = kp;
	phase_output = 0;
	sogi_backward_integrator_output = 0;
	sogi_forward_integrator_output = 0;
	loop_filter_sum = 0;
	radian_to_degree = 180 / M_PI;
	radian_of_period = 2 * M_PI;

	freq_Hz = fc;
	k_gain_sogi = k_gain;
	gamma = gam;
	fll_integrator = 0;
	freq_radians = fc *2 * M_PI;
}

/*---------------------------------------------------------------------------------------------------------*/
/*Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
	/*Init System, IP clock and multi-function I/O
	 In the end of SYS_Init() will issue SYS_LockReg()
	 to lock protected register. If user want to write
	 protected register, please issue SYS_UnlockReg()
	 to unlock protected register if necessary */

	/*Unlock protected registers */
	SYS_UnlockReg();

	/*Init System, IP clock and multi-function I/O */
	SYS_Init();

	/*Lock protected registers */
	SYS_LockReg();

	PDMA_Init();

	lookUpTable();

	init_pll_sogi(1.414213562, 0.5, 1, 10, 14.3e-05, 50);

	ADC_POWER_ON(ADC);
	CLK_SysTickDelay(10000);
	ADC_Open(ADC, ADC_ADCR_DIFFEN_SINGLE_END, ADC_ADCR_ADMD_SINGLE_CYCLE, BIT0 | BIT1 | BIT7 | BIT8);	//BIT0-->0___BIT1-->3___BIT7-->1___BIT8-->2
	ADC_ENABLE_PDMA(ADC);

	TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, 7000);

	TIMER_EnableInt(TIMER1);

	NVIC_EnableIRQ(TMR1_IRQn);

	NVIC_SetPriority(TMR1_IRQn, 0);

	GPIO_SetMode(PF, BIT15, GPIO_MODE_OUTPUT);	//Push_Pull_EN_PF.15
	GPIO_SetMode(PC, BIT2, GPIO_MODE_OUTPUT);	//H_Bridge PWM CH3
	GPIO_SetMode(PC, BIT3, GPIO_MODE_OUTPUT);	//H_Bridge PWM CH2
	GPIO_SetMode(PB, BIT2, GPIO_MODE_OUTPUT);
	GPIO_SetMode(PA, BIT14, GPIO_MODE_OUTPUT);	//Push_Pull_EN_PF.15

	/*
	_Sample PWM Configuration_
	Duty ratio = CMR / (CNR + 1)
	Period = Clock MHz / (prescaler *(CNR + 1))
	Period = (PLL)96 Mhz / (1* (750 + 1)) = 128 kHz 
    */

	/*PWM0 channel 0 frequency prescaler to 1 */
	PWM_SET_PRESCALER(PWM1, 0, 1 - 1);
	PWM_SET_PRESCALER(PWM0, 0, 1 - 1);

	/*PWM0 channel 0 frequency period to 375 */
	PWM_SET_CNR(PWM1, 0, 750);	//128kHz Up_Down (Edge Aligned)
	PWM_SET_CNR(PWM0, 0, 750);	//64kHz Push_Pull (Center Aligned)

	/*PWM0 channel 0 frequency comparator to 223--> */
	PWM_SET_CMR(PWM1, 0, 450);	//CH0 Low-Side D3 -- CH1 High-Side D2 450(%60 High Side)
	PWM_SET_CMR(PWM0, 0, 360);	//CH0 D8 --> 750(%100) 375(%50)
	PWM_SET_CMR(PWM0, 1, 750 - 360);

	PWM_SET_ALIGNED_TYPE(PWM1, BIT0, PWM_EDGE_ALIGNED);
	PWM_ENABLE_COMPLEMENTARY_MODE(PWM1);
	PWM_SET_ALIGNED_TYPE(PWM0, BIT0, PWM_CENTER_ALIGNED);

	/*PWM_SET_OUTPUT_LEVEL(pwm, u32ChannelMask, u32ZeroLevel, u32CmpUpLevel(compare up), u32PeriodLevel, u32CmpDownLevel) */
	PWM_SET_OUTPUT_LEVEL(PWM1, PWM_CH_0_MASK, PWM_OUTPUT_NOTHING, PWM_OUTPUT_HIGH, PWM_OUTPUT_NOTHING, PWM_OUTPUT_LOW);	//Low-Side
	PWM_SET_OUTPUT_LEVEL(PWM1, PWM_CH_1_MASK, PWM_OUTPUT_NOTHING, PWM_OUTPUT_LOW, PWM_OUTPUT_NOTHING, PWM_OUTPUT_HIGH);	//High-Side
	PWM_SET_OUTPUT_LEVEL(PWM0, PWM_CH_0_MASK, PWM_OUTPUT_NOTHING, PWM_OUTPUT_LOW, PWM_OUTPUT_NOTHING, PWM_OUTPUT_HIGH);	//Düz olan Low-Side
	PWM_SET_OUTPUT_LEVEL(PWM0, PWM_CH_1_MASK, PWM_OUTPUT_NOTHING, PWM_OUTPUT_HIGH, PWM_OUTPUT_NOTHING, PWM_OUTPUT_LOW);	//Tersi olan Low-Side

	SYS_UnlockReg();
	PWM_EnableDeadZone(PWM1, 0, 3);	//(1/CPUfreq)*Duration--> (1/96Mhz)*6=62.5 ns
	SYS_LockReg();

	/*Enable output of PWM channels */
	PWM_EnableOutput(PWM1, PWM_CH_0_MASK);
	PWM_EnableOutput(PWM1, PWM_CH_1_MASK);
	PWM_EnableOutput(PWM0, PWM_CH_0_MASK);
	PWM_EnableOutput(PWM0, PWM_CH_1_MASK);

	/*Start PWM counter */
	PWM_Start(PWM1, PWM_CH_0_MASK);
	PWM_Start(PWM1, PWM_CH_1_MASK);
	PWM_Start(PWM0, PWM_CH_0_MASK);
	PWM_Start(PWM0, PWM_CH_1_MASK);

	TIMER_Start(TIMER0);
	TIMER_Start(TIMER1);

	PC2 = 0;
	PC3 = 0;
	//PF15=1;	//Push-Pull IX4340N Enable 

	while (1);

}

void TMR1_IRQHandler(void)
{
	ReloadPDMA();
	ADC_START_CONV(ADC);

	grid_vadc = g_i32_Data[2] *(A_vdd / ADC_dig);
	grid_zero = grid_vadc - 2.5;	// tl431

	e = grid_zero - sogi_forward_integrator_output;
	sogi_input = e * k_gain_sogi;
	qv = sogi_backward_integrator_output * freq_radians;
	temp = sogi_input - qv;
	temp *= freq_radians;

	sogi_forward_integrator_output += temp * update_period;
	sogi_backward_integrator_output += sogi_forward_integrator_output * update_period;

	ef = e * qv;
	fll_integrator += (-gamma) *ef * update_period;
	freq_radians = center_angle_freq + fll_integrator;

	error = sogi_forward_integrator_output *cos_table[degree] +
		sogi_backward_integrator_output *angle_freq *sine_table[degree];

	//    vd=error;

	//    vq=sogi_backward_integrator_output *angle_freq *cos_table[degree] -
	//    		(sogi_forward_integrator_output *sine_table[degree]) ;

	loop_filter_sum += error * loop_filter_ki;
	angle_freq = center_angle_freq + loop_filter_sum + loop_filter_kp * error;
	phase_output += angle_freq * update_period;

	if (phase_output > radian_of_period)
	{
		phase_output -= radian_of_period;
	}

	degree = phase_output * radian_to_degree;
	//180/M_PI;=57.2957797

	if (degree < 180)
	{
		PC3 = 0;
		CLK_SysTickDelay(50);
		PC2 = 1;
	}
	else
	{
		PC2 = 0;
		CLK_SysTickDelay(50);
		PC3 = 1;
	}

	TIMER_ClearIntFlag(TIMER1);

}