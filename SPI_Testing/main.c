/*
 * Author: Dante Rieger
 * Date: March 19th, 2026 AD
 */

/*______________INCLUDES_______________*/

#include <stdio.h>
#include "SPI.h" // for SPI functions
#include "MyRio.h" // for MyRio connection
#include "T1.h" // maybe I will need Garbini's stuff
#include "IRQConfigure.h" // set up interrupt threads
#include <pthread.h> // for threads
#include "TimerIRQ.h" // using timer system in order to have consistent sampling rate
#include "DIO.h" // might need this for digital channels?
#include <math.h> // might need this for math


/*______________ Declarations__________*/
/*
 * myrio session
 */
NiFpga_Session myrio_session;


/*
 * Flags for SPI status register
 * Dante note: I have no idea what this does, I just copied the comment from Garbini's file
 */
typedef enum
{
    Spi_Busy = 0x1,   /* 0b00000001 */
} Spi_StatusMask;


/*
 * thread resource definition
 */
typedef struct {
  NiFpga_IrqContext irqContext;  // scontext
  table *a_table;                // table
  NiFpga_Bool irqThreadRdy;      // ready flag
} ThreadResource;

/*___________________Prototypes_________________*/





int main(int argc, char **argv)
{

}
