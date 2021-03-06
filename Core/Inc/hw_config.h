#ifndef HW_CONFIG_H
#define HW_CONFIG_H

#define COMMUTATE_OVERRIDE 0

/* Timer and PWM */
#define TIM_PWM			htim1				// PWM/ISR timer handle
#define TIM_CH_W		TIM_CHANNEL_3		// Terminal U timer channel
#define TIM_CH_V		TIM_CHANNEL_2		// Terminal V timer channel
#define TIM_CH_U		TIM_CHANNEL_1		// Terminal W timer channel

#define INVERT_DTC		1					// PWM inverting (1) or non-inverting (0)

/* ISRs */
#define PWM_ISR			TIM1_UP_TIM16_IRQn	// PWM Timer ISR
#define CAN_ISR			FDCAN2_IT0_IRQn		// CAN Receive ISR

/* ADC */

#define ADC_CH_MAIN		hadc1				// ADC channel handle which drives simultaneous mode
#define ADC_CH_IA		hadc1				// Phase A current sense ADC channel handle.  0 = unused
#define ADC_CH_IB		hadc2				// Phase B current sense ADC channel handle.  0 = unused
#define ADC_CH_IC		0					// Phase C current sense ADC channel handle.  0 = unused
#define ADC_CH_VBUS		hadc3				// Bus voltage ADC channel handle.  0 = unused

/* DRV Gate drive */
#define ENABLE_PIN 		GPIOA, GPIO_PIN_11  // Enable gate drive pin.
#define DRV_SPI			hspi1				// DRV SPI handle
#define DRV_CS			GPIOA, GPIO_PIN_4	// DRV CS pin

/* SPI encoder */
#define ENC_SPI			hspi3				// Encoder SPI handle
#define ENC_CS			GPIOA, GPIO_PIN_15	// Encoder SPI CS pin
#define ENC_CPR			65536				// Encoder counts per revolution
#define INV_CPR			1.0f/ENC_CPR
#define ENC_READ_WORD	0x00				// Encoder read command

/* Misc. GPIO */
#define LED1         	GPIOC, GPIO_PIN_2	// LED Pin
#define LED2         	GPIOC, GPIO_PIN_3	// LED Pin

/* CAN */
#define CAN_H			hfdcan2				// CAN handle

/* Other hardware-related constants */
#define I_SCALE 			0.02014160156f  // Amps per A/D Count
//measurements 07.01.2022
//input U on adcs a, b = 1.73 1.65
//raw measurements from adcs - 2058, 1955
//voltage on shunt resistors - 0
//measurements 19.01.2022
//u a, b 1.628 1.663
//raw 2000 1960
//#define V_SCALE_ORIG 		0.012890625f    // Bus volts per A/D Count
#define V_SCALE				16.0*0.0515625f // Bus volts per A/D Count
#define V_SCALE_SHUNTS      0.000879f		// Shunt volts per A/D count
#define DTC_MAX 			0.94f          	// Max duty cycle
#define DTC_MIN 			0.0f          	// Min duty cycle
#define DTC_COMP 			0.000f          // deadtime compensation (100 ns / 25 us)
// original #define DT					.000025f		// Loop period
#define DT					.000040f		// Loop period
#define EN_ENC_LINEARIZATION 1				// Enable/disable encoder linearization


/* Current controller */
#define L_D .000003f				// D axis inductance
#define L_Q .000003f				// Q axis inductance
//#define K_D .05f //orig                   // Loop gain,  Volts/Amp
//#define K_Q .05f //orig                   // Loop gain,  Volts/Amp
#define K_D .05f
#define K_Q .05f
#define K_SCALE 0.0001f             // K_loop/Loop BW (Hz) 0.0042
//#define KI_D 0.045f //orig               // PI zero, in radians per sample
//#define KI_Q 0.045f //orig               // PI zero, in radians per sample
#define KI_D 0.045f
#define KI_Q 0.045f
#define OVERMODULATION 1.15f        // 1.0 = no overmodulation
#define CURRENT_FILT_ALPHA	.01f	// 1st order d/q current filter (not used in control)
#define VBUS_FILT_ALPHA		.1f		// 1st order bus voltage filter

#define D_INT_LIM V_BUS/(K_D*KI_D)  // Amps*samples
#define Q_INT_LIM V_BUS/(K_Q*KI_Q)  // Amps*samples


#endif
