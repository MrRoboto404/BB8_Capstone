/*
 * Sramps.h
 *
 *  Created on: 2024-01-04
 *      Author: Rico Picone
 */

#ifndef Sramps_H_
#define Sramps_H_

#include "math.h"
typedef struct {double xfa; double v; double a; double d;} seg;

int  Sramps(seg *segs, int nseg, int *iseg, int *itime, double T, double *xa);

#endif /* Sramps_H_ */
