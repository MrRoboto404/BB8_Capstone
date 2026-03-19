/*
 * emulate.h
 *
 *  Created on: Nov 12, 2020
 *      Author: garbini
 */

#ifndef EMULATE_H_
#define EMULATE_H_

#include "AIO.h"
#include "DIO.h"
#include "Encoder.h"

#define T1a 1
#define T1b 2

#ifndef TARGET_SYS
#define TARGET_SYS T1a
#endif

#if TARGET_SYS == T1a
#define EMULATE_R 23.8 // armature resistance (ohms)
#define EMULATE_V_AMP_MAX 12 // amplifier output voltage limit
#define EMULATE_KVI 0.06 // Amplifier gain (A/V)
#define EMULATE_KT 0.0698 // Torque constant
#define EMULATE_KE (2048./2./M_PI) // encoder gain: (rad)*(rev/2/pi rad)*2048 BDI/rev) = BDI
#define EMULATE_JP 1.1e-05 // moment of inertia
#define EMULATE_B 5.3368e-06 // Damping
#define EMULATE_PWM_DIR DIOC_70DIR // PWM input DIR register
#define EMULATE_PWM_OUT DIOC_70OUT // PWM input OUT register
#define EMULATE_PWM_IN DIOC_70IN // PWM input IN register
#define EMULATE_PWM_BIT 3 // PWM input bit
#define EMULATE_PWM_GAIN 0.01515 // Amplifier PWM input gain (A/V)
#define EMULATE_PWM_V_LOW 0.0 // PWM input low amplifier input voltage
#define EMULATE_PWM_V_HIGH 3.3 // PWM input high amplifier input voltage
#elif TARGET_SYS == T1b
#define EMULATE_R 6.8 // armature resistance (ohms)
#define EMULATE_V_AMP_MAX 24 // amplifier output voltage limit
#define EMULATE_KVI 0.4 // Amplifier gain (A/V)
#define EMULATE_KT 0.11 // Torque constant (Kt=0.11 Nm/A)
#define EMULATE_KE (2000./2./M_PI) // encoder gain: (rad)*(rev/2/pi rad)*2000 BDI/rev) = BDI
#define EMULATE_JP 3.2e-4 // moment of inertia (N-m-s^2/rad)
#define EMULATE_B 1.e-4 // damping likely not significant. Torque offset likely
#define EMULATE_PWM_DIR DIOA_70DIR // PWM input DIR register
#define EMULATE_PWM_OUT DIOA_70OUT // PWM input OUT register
#define EMULATE_PWM_IN DIOA_70IN // PWM input IN register
#define EMULATE_PWM_BIT 0 // PWM input bit
#define EMULATE_PWM_GAIN EMULATE_KVI // Amplifier PWM input gain (A/V)
#define EMULATE_PWM_V_LOW 0.633 // PWM input low amplifier input voltage
#define EMULATE_PWM_V_HIGH 0.0 // PWM input high amplifier input voltage
#else
#error "Unknown target system!"
#endif

// substitute emulated i/o
#define Encoder_Counter(x) 			Encoder_CounterE(x)
#define EncoderC_initialize(x,y)	EncoderC_initializeE(x,y)
#define Aio_Write(x,y) 				Aio_WriteE(x,y)
#define	Aio_Read(x) 				Aio_ReadE(x)
#define AIO_initialize(x,y)			AIO_initializeE(x,y)
#define Dio_WriteBit(x,y) 			Dio_WriteBitE(x,y)


// emulator prototypes
uint32_t 		Encoder_CounterE(MyRio_Encoder* channel);
NiFpga_Status	EncoderC_initializeE(NiFpga_Session myrio_session, MyRio_Encoder *encC0p);
void 			Aio_WriteE(MyRio_Aio* channel, double value);
double 			Aio_ReadE(MyRio_Aio* channel);
void 			AIO_initializeE(MyRio_Aio *CI0p, MyRio_Aio *CO0p);
void 			Dio_WriteBitE(MyRio_Dio* channel, NiFpga_Bool value);
double			TODsec(void);
void 			fg_set(uint32_t mode, double amp, double freq_Hz);


#endif /* EMULATE_H_ */
