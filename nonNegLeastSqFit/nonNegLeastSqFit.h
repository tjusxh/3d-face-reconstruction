#ifndef NONNEGLEASTSQFIT_H
#define NONNEGLEASTSQFIT_H
#include <stdio.h>
#include <math.h>
#define nnls_max(a,b) ((a) >= (b) ? (a) : (b))
#define nnls_abs(x) ((x) >= 0 ? (x) : -(x))
typedef int integer;
typedef double doublereal;

#ifdef __cplusplus
extern "C"
{
#endif
	int nnls_(double*a,
		int *mda, int *m, int *n,
		double *b,double *x, double *rnorm,double *w,double *zz,
		int *index,int *mode);
#ifdef __cplusplus
};
#endif

#endif

