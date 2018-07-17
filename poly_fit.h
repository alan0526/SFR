
#ifndef POLY_FIT_H
	#define POLY_FIT_H

	#define INDEX(x,y,width) ((y)*(width)+(x))
	#define POLYFIT_SUCCESS					0
	#define POLYFIT_MEMORY_ALLOC_ERROR		1
	#define POLYFIT_UNKNOWN_MODE			2
	#define POLYFIT_UNSUPPORTED_ORDER		3
	#define POLYFIT_INSUFFICIENT_SAMPLES	4
	#define POLYFIT_INPUT_RANGE_INVALID		5
	#define POLYFIT_STRUCT_NOT_INITIALIZED	6
	#define POLYFIT_MODE_NAIVE				0
	#define POLYFIT_MODE_CHEBYSHEV			1

typedef struct {
		unsigned int polymode;
		int polyorder;
		unsigned int samples;
		unsigned int allocated_samples;
		double **precomputed;
		double *a,*L;
		double (*basisfnct)(double);
	} polyfit_t;

	extern int polyfit_delete(polyfit_t* hPolyfit);
	extern int polyfit_init(polyfit_t* hPolyfit, int mode, int order, int numsamp);
	extern int polyfit_resize(polyfit_t* hPolyfit, int numsamp);
	extern int polyfit_regress(polyfit_t* hPolyfit, double* x, double* y);
	extern double polyfit_interp(polyfit_t* hPolyfit, double x);

#endif
