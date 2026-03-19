/*
 * T1.h
 *
 *  Created on: Sep 18, 2015
 *      Author: garbini
 */

#ifndef T1_H_
#define T1_H_

#include "AIO.h"
#include "Encoder.h"

#define ENT		10
#define	DEL		8
#define UP		91
#define DN		93

int	putchar_lcd(int c);
int printf_lcd(const char *format, ...);
int scanf_keypad(char *format, ...);
char getkey(void);
int	msDelay(int tms);
int getchar_keypad(void);
float	float_in(char *prompt);
double	double_in(char *prompt);
char * fgets_keypad(char *buf, int buflen);
void Aio_InitCI0(MyRio_Aio *CI0);
void Aio_InitCI1(MyRio_Aio *CI1);
void Aio_InitCO0(MyRio_Aio *CO0);
void Aio_InitCO1(MyRio_Aio *CO1);
NiFpga_Status	EncoderC_initialize(NiFpga_Session myrio_session, MyRio_Encoder *encC0);

#endif /* T1_H_ */
