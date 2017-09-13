/*
	What:	poly_fit.c
	Who:	Matt Warmuth
	Where: ZF TRW
	When:	14.June.2015
	Why:	Used to fit polynomial functions to data

	This was developed for use in SFR routines that try to align edges 
	before aggregating super-sampled Line Spread Functions (LSF)
*/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "poly_fit.h"

// Macro to index into 1D array with 2D coordinates
#define INDEX(x,y,width) ((y)*(width)+(x))

// Macro to insert file and line number of error into response
#define ERROR_MACRO(code,message) { _reporterror(code, message, __FILE__, __LINE__); return(code); }

void _reporterror(int errorcode, char* message, char* fileerror, int lineerror) {


} // End reporterror()

// Local Basis Functions
double naive(double x) { return x; }
double Chebyshev(double x) { return 2*x-1; }

// Initialize polynomial fitting struct (allocate memory, etc.)
extern int polyfit_init(polyfit_t* hPolyfit, int mode, int order, int numsamp) {
	int i;
	// Initialize parameters
	hPolyfit->polyorder=0;
	hPolyfit->samples=0;
	hPolyfit->allocated_samples=0;
	hPolyfit->a = NULL;
	hPolyfit->L = NULL;
	hPolyfit->precomputed = NULL;

	// Handle mode setting
	switch (mode) {
	case POLYFIT_MODE_NAIVE:
		hPolyfit->polymode=POLYFIT_MODE_NAIVE;
		hPolyfit->basisfnct = naive;
		break;
	case POLYFIT_MODE_CHEBYSHEV:
		hPolyfit->polymode=POLYFIT_MODE_CHEBYSHEV;
		hPolyfit->basisfnct = Chebyshev;
		break;
	default:
		ERROR_MACRO(POLYFIT_UNKNOWN_MODE,"Unknown Poly Fit mode; Struct not initialized\n");
	};

	// Handle Poly order
	if (order > 1 && order < 9) {
		hPolyfit->polyorder = order;
		hPolyfit->precomputed = (double**)malloc(order * sizeof(double*));
		if (!hPolyfit->precomputed) { ERROR_MACRO(POLYFIT_MEMORY_ALLOC_ERROR,NULL); }
		hPolyfit->a = (double*)malloc(order * sizeof(double));
		if (!hPolyfit->a) { ERROR_MACRO(POLYFIT_MEMORY_ALLOC_ERROR,NULL); }
		hPolyfit->L = (double*)malloc(order * (order+1) * sizeof(double));
		if (!hPolyfit->L) { ERROR_MACRO(POLYFIT_MEMORY_ALLOC_ERROR,NULL); }
	}
	else {
		ERROR_MACRO(POLYFIT_UNSUPPORTED_ORDER,"Only supports polynominal orders 2 -> 9.\n");
	}

	// Handle Number of Samples
	if (numsamp < hPolyfit->polyorder) {
		ERROR_MACRO(POLYFIT_INSUFFICIENT_SAMPLES,"Underdetermined sample matrix.\n");
	} else {
		hPolyfit->samples = numsamp;
		hPolyfit->allocated_samples = numsamp;
		for (i=0; i<hPolyfit->polyorder; i++) {
			hPolyfit->precomputed[i] = (double*)malloc(hPolyfit->samples * sizeof(double));
			if (!hPolyfit->L) { ERROR_MACRO(POLYFIT_MEMORY_ALLOC_ERROR,NULL); }
		}
	}

	// No failures; return success
	return POLYFIT_SUCCESS;
} // end init_polyfit()

extern int polyfit_resize(polyfit_t* hPolyfit, int numsamp) {
	// This allows user to resize the number of samples used for regression,
	// but will only re-allocate memory if the number goes above the current
	// allocation.
	int i;
	// Verify we have already completed an initialization successfully
	if (!hPolyfit->polyorder) { ERROR_MACRO(POLYFIT_STRUCT_NOT_INITIALIZED,NULL); }
	// Ensure they still supplying enough
	if (numsamp < hPolyfit->polyorder) { ERROR_MACRO(POLYFIT_INSUFFICIENT_SAMPLES,"Underdetermined sample matrix.\n"); }
	// Requesting more samples than previously allocated; need to clear and re-allocate memory
	if (numsamp > hPolyfit->allocated_samples) {
		for (i=0; i<hPolyfit->polyorder; i++) {
			// Clear old allocation
			if (hPolyfit->precomputed[i]) { free(hPolyfit->precomputed[i]); hPolyfit->precomputed[i]=NULL; }
			// Make new allocation
			hPolyfit->precomputed[i] = (double*)malloc(numsamp * sizeof(double));
			// Check allocation
			if (!hPolyfit->precomputed[i]) { ERROR_MACRO(POLYFIT_MEMORY_ALLOC_ERROR,NULL); }
		}
		// Updated allocated and set number of samples
		hPolyfit->allocated_samples = numsamp;
		hPolyfit->samples = numsamp;
	} else {
		// Update only set number of samples
		hPolyfit->samples = numsamp;
	}
	return POLYFIT_SUCCESS;
} // End polyfit_resize()

extern int polyfit_delete(polyfit_t* hPolyfit) {
	int i;
	// De-allocate all pre=computed pointers
	if (hPolyfit->precomputed) {
		for (i=0; i<hPolyfit->polyorder;i++) {
			if (hPolyfit->precomputed[i]) { free(hPolyfit->precomputed[i]); hPolyfit->precomputed[i]=NULL; }
		}
		// Free top level pointer
		free(hPolyfit->precomputed); hPolyfit->precomputed = NULL;
	}
	// De-allocation solving matrix
	if (hPolyfit->L) { free(hPolyfit->L); hPolyfit->L = NULL; }
	// De-allocation coeff matrix
	if (hPolyfit->a) { free(hPolyfit->a); hPolyfit->L = NULL; }
	// Set some other values for checks
	hPolyfit->polyorder=0;
	return POLYFIT_SUCCESS;
} // end polyfit_delete

extern int polyfit_regress(polyfit_t* hPolyfit, double* x, double* y) {
	/*
		Use given x,y pairs to find regression coefficients
	*/
	int order, row, i, j, k;
	// Ensure we have a valid struct
	if (!hPolyfit->polyorder) { return 1; }
	///////////////////////////////////////
	// First, fill in pre-computed sample arrays
	///////////////////////////////////////
	// Compute all the component arrays
	for (order=0; order<hPolyfit->polyorder; order++) {
		if (order) {
			// Fill in higher order modes using recursive relationship
			for (i=0; i<hPolyfit->samples; i++) {
				hPolyfit->precomputed[order][i] = hPolyfit->precomputed[order-1][i] * hPolyfit->basisfnct(x[i]);
			} // end for(i)
		} else {
			// Constant order
			for (i=0; i<hPolyfit->samples; i++) {
				// Ensure x samples have a normalized range
				if (x[i]<0.0 || x[i]>1.0) { ERROR_MACRO(POLYFIT_INPUT_RANGE_INVALID,NULL); }
				// Zero order functions are constant
				hPolyfit->precomputed[order][i] = 1.0;
			} // for (i)
		} // End if (!order)
	} // end for (order)

	///////////////////////////////////////
	// Fill matrix with inner products
	///////////////////////////////////////
	for (j=0; j<hPolyfit->polyorder; j++) {
		for (i=j; i<hPolyfit->polyorder; i++ ) {
			// Initialize accumulator to zero
			hPolyfit->L[INDEX(i,j,1+hPolyfit->polyorder)] = 0.0;
			// Sum inner product
			for (k=0; k<hPolyfit->samples; k++) {
				hPolyfit->L[INDEX(i,j,1+hPolyfit->polyorder)] += hPolyfit->precomputed[j][k] * hPolyfit->precomputed[i][k];
			} // End for(k)
			// Mirror matrix entry if not on diagonal
			if (i!=j) {
				hPolyfit->L[INDEX(j,i,1+hPolyfit->polyorder)] = hPolyfit->L[INDEX(i,j,1+hPolyfit->polyorder)];
			}
		} // End for(i)
		// Write last column entry
		hPolyfit->L[INDEX(hPolyfit->polyorder,j,1+hPolyfit->polyorder)] = 0.0;
		for (k=0; k<hPolyfit->samples; k++) {
			hPolyfit->L[INDEX(hPolyfit->polyorder,j,1+hPolyfit->polyorder)] += hPolyfit->precomputed[j][k] * y[k];
		} // End for(k)
	} // End for (j)

	///////////////////////////////////////
	// Partially solve matrix
	//	- Get ones on diagonal and zeros on the lower triangle
	///////////////////////////////////////
	// Loop over matrix rows
	for (j=0; j<hPolyfit->polyorder; j++) {
		// Find diagonal value to normalize by
		double scale = 1/hPolyfit->L[INDEX(j,j,1+hPolyfit->polyorder)];
		// Scale j-th row by this value
		for (i=j; i<=hPolyfit->polyorder; i++ ) {
			hPolyfit->L[INDEX(i,j,1+hPolyfit->polyorder)] *= scale;
		}

		// Zero out each row's j-th column
		for (row=j+1; row<hPolyfit->polyorder; row++) {
			// Get scaling value to subtract
			double scale = hPolyfit->L[INDEX(j,row,1+hPolyfit->polyorder)];
			// Scale and subtract i-th column value
			for (i=0; i<=hPolyfit->polyorder; i++ ) {
				hPolyfit->L[INDEX(i,row,1+hPolyfit->polyorder)] -= scale * hPolyfit->L[INDEX(i,j,1+hPolyfit->polyorder)];
			}
		}
	} // End for (j)

	///////////////////////////////////////
	// Back substitute to complete solution
	///////////////////////////////////////
	for (j=hPolyfit->polyorder-1; j>=0; j--) {
		// Initialize solution with last element in row
		hPolyfit->a[j] = hPolyfit->L[INDEX(hPolyfit->polyorder,j,1+hPolyfit->polyorder)];
		for (i=hPolyfit->polyorder-1; i>j; i--) {
			// Subtract all the other non-diagonal terms with previous coeff's
			hPolyfit->a[j] -= hPolyfit->L[INDEX(i,j,1+hPolyfit->polyorder)] * hPolyfit->a[i];
		}
	} // End for (j)

	// Return success
	return POLYFIT_SUCCESS;
} //end polyfit_regress()

// Interpolate value from regression 
extern double polyfit_interp(polyfit_t* hPolyfit, double x) {
	int i;
	double interp;


	// Initial sum with constant term
	interp = hPolyfit->a[0];
	// Add all other terms by order
	for (i=1; i<hPolyfit->polyorder; i++) {
		interp += hPolyfit->a[i] * pow(hPolyfit->basisfnct(x),i);
	}
	return interp;
} // End polyfit_interp()