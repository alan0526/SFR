

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

//extern FILE *g_mtfout;

FILE *g_mtfout;

#include "sfr.h"
#include "poly_fit.h"		// Higher order polynominal fitting
/*****************************************************************************/

static double sqrarg;
int ROI=0;

unsigned short ftwos(long, double, double *, long, double, double *);
void apply_hamming_window(  unsigned short, unsigned int, unsigned short,
			    double *, long *);
void locate_max_PSF(unsigned int, double *, long *);
void calculate_derivative(unsigned int, double *, double *, double *, int);
unsigned short bin_to_regular_xgrid(unsigned short, double *, double *, 
				    double *, long *,
				    unsigned short, unsigned short);
unsigned short fit(unsigned long, double *, double *, double *, double *, 
		   double *, double *, double *);
unsigned short locate_centroids(double *, double *,double *, double *,
				unsigned short, unsigned short, double *);
unsigned short locate_threshold(double *, double *,double *, double *,
				unsigned short, unsigned short, double *);
unsigned short check_image_data(double *, unsigned short, unsigned short);
unsigned short check_slope(double, unsigned short *, int *, double, int);

/*****************************************************************************/
/* Data passed to this function is assumed to be radiometrically corrected,  */
/* and oriented vertically, with black on left, white on right. The black to */
/* white orientation doesn't appear to be terribly important in this code.   */
/* Radiometric correct data (or at least 0-1 scaled) is important for the    */
/* contrast check.                                                           */
/*     Parameters:
          Input:  farea  = radiometric image data in ROI
                  size_x = number of columns in ROI
		  nrows  = number of rows in ROI

	  Output: freq   = new array of relative spatial frequencies 
	          sfr    = new array of computed SFR at each freq
		  len    = length of freq & sfr arrays
		  slope  = estimated slope value
		  numcycles = number of full periods included in SFR
		  pcnt2  = location of edge mid-point in oversample space
		  off    = shift to center edge in original rows
		  version = 0 = default ([-1 1] deriv, no rounding, no peak)
		            1 = add rounding
			    2 = add peak
			    4 = [-1 0 1] derivative, rounding, no peak
		  iterate = 0 do just a single run
		            1 means more runs after this, don't change farea
		               and let numcycles go as low as 1.0
		  user_angle = flag to indicate if the line fit has
		               been precomputed
		  
                  farea  = size_x*4 of ESF and len*2 of LSF values 
    		                 (if iterate = 0)                            
                  */
/*****************************************************************************/
short sfrProc (double **freq, double **sfr, 
	       int *len,
	       double *farea,
	       unsigned short size_x, int *nrows,
	       double *slope, int *numcycles, int *pcnt2, double *off, double *R2,
	       int version, int iterate, int user_angle, int fitorder)
{
	unsigned short i, j, col, err = 0;
	long pcnt;
	double sfrc, tmp, tmp2;
	double *tempx=NULL, *tempy=NULL, *shifts=NULL, *edgex=NULL;
	double *AveEdge=NULL, *AveTmp=NULL;
	long *counts=NULL;	
	unsigned short size_y;
	unsigned short ww_in_pixels;
	unsigned int bin_len;	
	double avar, bvar, offset1, offset2, offset;
	double centroid;
	int nzero, start_row, center_row;
	double *farea_old, *norm_range;
	double cyclelimit;
	polyfit_t polyparams;
	FILE *fp = NULL;
	//static int ROI=0;		// Keep track of which ROI number
	size_y = *nrows;

	/* Verify input selection dimensions are EVEN */
	/*
	if (size_x%2 != 0) { 
		fprintf(stderr, "ROI width is not even.  Does this really matter???\n");
		return 1;
	}
	*/
 
	/* At least this many cycles required. */
	/* For special iterative versions of the code, it can go lower */
	if (iterate) { cyclelimit = 1.0; }
	else { cyclelimit = 5.0; }

	/* Allocate memory */
	shifts = (double *)malloc(size_y*sizeof(double));
	tempx = (double *)malloc(size_x*sizeof(double));
	tempy = (double *)malloc(size_y*sizeof(double));
	norm_range = (double *)malloc(size_y*sizeof(double));
	edgex = (double *)malloc(size_y*size_x*sizeof(double));

	if( !user_angle ) {
		//err = locate_centroids(farea, tempx, tempy, shifts, size_x, size_y, &offset1); 
		err = locate_threshold(farea, tempx, tempy, shifts, size_x, size_y, &offset1);   //use thershold to find the transition points
		if (err != 0) 
		{ 
			// This is why you don't return from multiple points - plugs mem leak on failures
			free(shifts);
			free(tempx);
			free(tempy);
			free(norm_range);
			free(edgex);
			return 2; 
		}

		/* Calculate the best fit line to the centroids */
		err = fit(size_y, tempy, shifts, slope, &offset2, R2, &avar, &bvar);
		if (err != 0) 
		{ 
			// This is why you don't return from multiple points - plugs mem leak on failures
			free(shifts);
			free(tempx);
			free(tempy);
			free(norm_range);
			free(edgex);
			return 3;
		}
    
		if (version) {
			MTFPRINT4("\nLinear Fit:  R2 = %.3f SE_intercept = %.2f  SE_angle = %.3f\n",  
				*R2, avar, atan(*slope)*(double)(180.0/M_PI));
		}
	} // end if (user_angle)

	/* Check slope is OK, and set size_y to be full multiple of cycles */
	err = check_slope(*slope, &size_y, numcycles, cyclelimit, 1);

	/* Start image at new location, so that same row is center */
	center_row = *nrows/2;
	start_row = center_row - size_y/2;
	farea_old = farea;
	farea = farea + start_row*size_x;
	/* On center row how much shift to get edge centered in row. */
	/* offset = 0.;  Original code effectively used this (no centering)*/
	if (user_angle) {
		offset = *off - size_x/2;
	} else {
		offset = offset1 + 0.5 + offset2  - size_x/2; 
	}

	*off = offset;
	if (version & ROUND || version & DER3) { offset += 0.125; }
	if (err != 0) {
		/*	Slopes are bad.  But send back enough
			data, so a diagnostic image has a chance. */
		*pcnt2 = 2*size_x;  /* Ignore derivative peak */

		// This is why you don't return from multiple points - plugs mem leak on failures
		free(shifts);
		free(tempx);
		free(tempy);
		free(norm_range);
		free(edgex);
		return 4; 
	}

	/* Initialize polynominal fit to requested order using naive polynomials */
	polyfit_init(&polyparams, POLYFIT_MODE_CHEBYSHEV, fitorder, size_y);
	/* reference the temp and shifts values to the new y centre */
	/* Instead of using the values in shifts, synthesize new ones based on 
     the best fit line. */
	col = size_y/2;
	// The poly_fit functions define order as 1=constant, 2=linear, 3=quadratic, etc.)
	if (fitorder==2) {
		// Use old ISO code for linear fit for backward compatibility
		for (i=0; i < size_y; i++) { shifts[i] = (*slope) * (double)(i-col) + offset; }
	} else {
		/* Make normalized input range array for polynominal fitting */
		for (i=0; i < size_y; i++) { norm_range[i] = (double)i/(double)(size_y-1); }
		/* Do a regression fit against centroids using higher order fit */
		polyfit_regress(&polyparams, norm_range, shifts);
		/* Calculate value of offset that places edge at center of array in the middle of the edge */
		offset = offset1 - (double)(size_x/2);
		/* New fit using higher order polynomial */
		for (i=0; i < size_y; i++) {
			shifts[i] = polyfit_interp(&polyparams,norm_range[i])+offset;
		}
	}

	// If user has requested output of Edge Spread Functions, open file for writing
	if (version & ESFFILE) {
		fp = fopen("esf.txt","a");
		fprintf(fp, "Edge %d\n", ROI++);
	}

  /* Calculate a long paired list of x values and signal values */
	pcnt = 0;
	for (j = 0; j < size_y; j++) {
		for (i = 0; i < size_x; i++) {
			edgex[pcnt] = (double)i - shifts[j];
			if ((version & ESFFILE) && edgex[pcnt] < size_x/2 + 5 && edgex[pcnt] > size_x/2 - 5) {
				fprintf(fp, "%f %f\n", edgex[pcnt], farea[pcnt]);
			}
			pcnt++;
		} // End for i
	} // End for j

	/* Free arrays no longer needed */
	free(tempx);
	free(tempy);
	free(shifts);
	free(norm_range);
	polyfit_delete(&polyparams);

	// Close file handle
	if (version & ESFFILE) { fclose(fp); }

	/* Allocate more memory */
	bin_len = (unsigned int)(ALPHA*size_x);
	AveEdge = (double *)malloc(bin_len*sizeof(double));
	AveTmp = (double *)malloc(bin_len*sizeof(double));
	counts = (long *)malloc(bin_len*sizeof(long));

	/* Project ESF data into supersampled bins */
	nzero = bin_to_regular_xgrid((unsigned short)ALPHA, edgex, farea, 
			       AveEdge, counts, 
			       size_x, size_y); 
	free(counts);
	free(edgex);

	/* Compute LSF from ESF.  Not yet centered or windowed. */
	calculate_derivative( bin_len, AveTmp, AveEdge, &centroid, (version & DER3 ? 1:0) );

	if (iterate == 0) {
	    /* Copy ESF to output area */
		for ( i=0; i<bin_len; i++ ) { farea_old[i] = AveTmp[i]; }
	}

	/* Find the peak/center of LSF */
	locate_max_PSF( bin_len, AveEdge, &pcnt);

	ww_in_pixels = size_x;

	if (version) { MTFPRINT3("Off center distance (1/4 pixel units): Peak %ld  Centroid %.2f\n", pcnt-2*size_x, centroid-2*size_x); }

	if ((version & PEAK) == 0) { 
		/* Ignore derivative peak */
		pcnt = 2*size_x;
	} else {
		MTFPRINT("Shifting peak to center\n");
 }

	/* Here the array length is shortened to ww_in_pixels*ALPHA,
	and the LSF peak is centered and Hamming windowed. */
	apply_hamming_window((unsigned short)ALPHA, bin_len, 
		       (unsigned short)ww_in_pixels, 
		       AveEdge, &pcnt);

	/* From now on this is the length used. */
	bin_len = ww_in_pixels*ALPHA;
	*len = bin_len/2;

	if (iterate == 0) {
    /* Copy LSF_w to output area */
		for ( i=0; i<bin_len; i++ ) { farea_old[size_x*(int)ALPHA+i] = AveEdge[i]; }
	}

	tmp = 1.0;
	tmp2 = 1.0/((double)bin_len) ;

	/* Now perform the DFT on AveEdge */
	/* ftwos ( nx, dx, lsf(x), nf, df, sfr(f) ) */
	(void) ftwos(bin_len, tmp, AveEdge, (long)(*len), 
	       tmp2, AveTmp); 

	/* Allocate memory if it hasen't already been*/
	if(*freq==NULL) { (*freq) = (double *)malloc((*len)*sizeof(double)); }
	if(*sfr==NULL) { (*sfr) = (double *)malloc((*len)*sizeof(double)); }

	/* Normalize MTF values to zero spatial frequency */
	for (i=0; i<(*len); i++) {
		sfrc = AveTmp[i];
		(*freq)[i]= ((double)i/(double)ww_in_pixels);
		(*sfr)[i] = (double) (sfrc/AveTmp[0]); 
	}

	/* Free */
	free(AveEdge);
	free(AveTmp);

	*nrows = size_y;
	*pcnt2 = pcnt;

	return(0);
}

//#pragma mark ---- vector processing routines ----
/*****************************************************************************/
unsigned short locate_centroids(double *farea, double *tempx, double *tempy, double *shifts,
				unsigned short size_x, unsigned short size_y,
				double *offset) {
	unsigned long i, j;
	double dt, dt1, dt2;
	int idx;

  /* Compute the first difference on each line. Interpolate to find the 
     centroid of the first derivatives. */

	for (j = 0; j < size_y; j++) {
		dt = 0.0;
		dt1 = 0.0;
		// Calculate Finite difference derivative
		tempx[0] = tempx[size_x-1] = 0.0;
		for (i = 1; i < size_x-1; i++) {
			tempx[i] = farea[(j*(long)size_x)+(i+1)] - farea[(j*(long)size_x)+i-1];
			// Following two lines are for debugging
			dt += tempx[i] * (double)i;
			dt1 += tempx[i];
			// End debugging
		}

		// Following averages derivative before centroid
		///////////////////////////////////////////////////////////////////////
		shifts[j]=dt/dt1;

		if(ROI == 17)
		{
		printf("%d %f\n",j,shifts[j]);
		}

		dt = 0.0;
		dt1 = 0.0;
		// Initialize accumulator
		dt2 = (tempx[0] + tempx[1] + tempx[2] + tempx[3]);
		// Smooth this curve by averaging over 9 values
		for (i = 0; i < size_x-1; i++) {
			if (i+4 < size_x-1) { dt2 += tempx[i+4]; }
			if (i > 4) { dt2 -= tempx[i-5]; }
			dt += dt2 * (double)i;
			dt1 += dt2;
		}
#ifdef FILTER_ESF
		shifts[j]=dt/dt1;
#endif
		///////////////////////////////////////////////////////////////////////
	}
	

  /* check again to be sure we aren't too close to an edge on the corners. 
     If the black to white transition is closer than 2 pixels from either 
     side of the data box, return an error of 5; the calling program will 
     display an error message (the same one as if there were not a difference 
     between the left and right sides of the box ) */
	if (shifts[size_y-1] < 2  || size_x - shifts[size_y-1] < 2) {
		fprintf(stderr,"** WARNING: Edge comes too close to the ROI corners.\n");
		return 5;
	}
	if (shifts[0] < 2 || size_x - shifts[0] < 2) {
		fprintf(stderr,"** WARNING: Edge comes too close to the ROI corners.\n");
		return 5;
	}


	/* Reference rows to the vertical centre of the data box */
	j = size_y/2;
	dt = shifts[j];
	for (i = 0; i < size_y; i++) {
		tempy[i] = (double)i - (double)j;
		shifts[i] -= dt;
	}
	*offset = dt;
	printf("ROI = %d",ROI);
	return 0;
}

unsigned short locate_threshold(double *farea, double *tempx, double *tempy, double *shifts,
				unsigned short size_x, unsigned short size_y,
				double *offset) {
	unsigned long i,j;
	double dt,temp1,temp2,thres = 0;
	

	for(j = 0; j< size_y; j++)
	{
		thres += farea[(j*(long)size_x)] + farea[(j*(long)size_x)+size_x-1];
	}
	thres /= size_y*2; //the threshold

	for(j = 0; j< size_y; j++) //find transition point for each column
	{
		temp1 =  farea[(j*(long)size_x)] - thres;
		temp2 = farea[(j*(long)size_x)+1] - thres;
		for(i = 2; i< size_x && temp1*temp2 > 0; i++)
		{
			temp1 = temp2;
			temp2 = farea[(j*(long)size_x)+i] - thres;
		}

		if(temp1 < 0)
		{temp1 = -temp1;}
		if(temp2 < 0)
		{temp2 = -temp2;}

		shifts[j] = (double)i-2 + temp1/(temp1 + temp2); //find the interpolated x of transition point

		if(ROI == 17)
		{
		printf("%d %f\n",j,shifts[j]);
		}
	}

	if (shifts[size_y-1] < 2  || size_x - shifts[size_y-1] < 2) {
		fprintf(stderr,"** WARNING: Edge comes too close to the ROI corners.\n");
		return 5;
	}
	if (shifts[0] < 2 || size_x - shifts[0] < 2) {
		fprintf(stderr,"** WARNING: Edge comes too close to the ROI corners.\n");
		return 5;
	}

	/* Reference rows to the vertical centre of the data box */
	j = size_y/2;
	dt = shifts[j];
	for (i = 0; i < size_y; i++) {
		tempy[i] = (double)i - (double)j;
		shifts[i] -= dt;
	}
	*offset = dt;

	printf("ROI = %d",ROI);

	return 0;

}

/***************************************************************************/
unsigned short fit(unsigned long ndata, double *x, double *y, double *b, 
		   double *a, double *R2, double *avar, double *bvar)
{
  unsigned long i;
  double t,sxoss,syoss,sx=0.0,sy=0.0,st2=0.0;
  double ss,sst,sigdat,chi2,siga,sigb;

  *b=0.0;
  for ( i=0; i < ndata; i++ ) {
    sx += x[i];
    sy += y[i];
  }
  ss=(double)ndata;
  sxoss=sx/ss;
  syoss=sy/ss;
  for ( i=0; i < ndata; i++ ) {
    t = x[i] - sxoss;
    st2 += t*t;
    *b += t * y[i];  
  }
  *b /= st2;         /* slope  */
  *a =(sy-sx*(*b))/ss; /* intercept */
  siga=sqrt((1.0+sx*sx/(ss*st2))/ss);
  sigb=sqrt(1.0/st2);
  chi2=0.0;
  sst=0.0;
  for (i=0; i < ndata; i++) {
    chi2 += SQR( y[i] - (*a) - (*b) * x[i]); 
    sst += SQR( y[i] - syoss); 
  }
  sigdat=sqrt(chi2/(ndata-2));
  siga *= sigdat;
  sigb *= sigdat;
  *R2 = 1.0 - chi2/sst;
  *avar = siga;
  *bvar = sigb;
  return 0;
}

/****************************************************************************/
unsigned short check_slope( double slope, unsigned short *size_y, int *numcycles, double mincyc, int errflag)
{
  double absslope;
  absslope = fabs(slope);


  if( *numcycles <= 0 ) { (*numcycles) = (int)((*size_y)*absslope); }

	/* If the slope is too small not enough rows for mincyc (typically 5) 
     full periods, then alert the user. */
	if ( absslope < mincyc/(double)(*size_y) ) {
		if(errflag == 1) {
			fprintf(stderr, "WARNING: Edge angle (%f) is so shallow it needs\n",
			atan(slope)*180/M_PI );
			fprintf(stderr, "  %d lines of data (%.1f cycles) for accurate results\n",
			(int)ceil(mincyc/absslope), mincyc);
			return 0;
		} else {
			return 1;
		}
	}

	if ( absslope > (double)(1.0/4.0) ) { 
		int rows_per_col;
		double bestcycle, x;

		if( absslope > (double)(5.0/4.0) ) {
			fprintf(stderr, "ERROR: Edge angle (%f) is too large\n",
			atan(slope)*180/M_PI);
			return 1;
		}

		rows_per_col = (int)floor(1/absslope + 0.5);
		x = fabs(1/(double)rows_per_col - absslope);
		bestcycle = 4*rows_per_col*x*ceil(1.0/x/(double)rows_per_col/(double)rows_per_col - 1.0);
		if ((int)ceil(mincyc*bestcycle) > *size_y) {
			if(errflag == 1) {
				fprintf(stderr, "WARNING: Edge angle (%f) will reduce SFR accuracy\n",
					atan(slope)*180/M_PI);
				fprintf(stderr, "   if %g * %f = %d lines of data are not in ROI\n\n",
					mincyc, bestcycle,
					(int)ceil(mincyc*bestcycle));
				return 0;
			} else {
				return 1;
			}
		}
	}

  /* Figure out how many lines to use for size_y: new window will start at 
     top and go down that number of lines < size_y such that an integer 
     number of x-transitions are made by the edge; for example, if we had a 
     slope of 10 (the edge goes down 10 lines before jumping over one pixel 
     horizontally), and size_y = 35, the new size_y is going to be 30 (an 
     integer multiple of 10, less than 35). */

	if( ((*numcycles)/absslope) <= *size_y)	{
		*size_y = (unsigned short)((*numcycles)/absslope);
	}

	return 0;
}

/*****************************************************************************/
/* Notes: this part gets averages and puts them in a number of bins, equal to 
size_x times alpha.  Next a long check is done in case one bin gets no values 
put into it: if this is the case, it will keep checking previous bins until it
finds one with non-zero counts and will use that value as its current bin 
average. If the first bin has zero counts the program checks bins in the 
forward rather than reverse direction. If, in any case, the end of the array 
of bins is reached before finding a non-zero count, the program starts 
checking in the opposite direction. A bin with zero counts is not allowed, 
since each bin will be divided by counts at the end. */

unsigned short bin_to_regular_xgrid(unsigned short alpha,
				    double *edgex, double *Signal, 
				    double *AveEdge, long *counts,
				    unsigned short size_x,
				    unsigned short size_y)
{
	long i, j, k,bin_number;
	long bin_len;
	int nzeros=0;

	bin_len = size_x * alpha;

	// Initialize Bins to zero
	for (i=0; i<bin_len; i++) {
		AveEdge[i] = 0;
		counts[i] = 0;
	}

	// Accumulate values and counts for each bin
	for (i=0; i<(size_x*(long)size_y); i++) {
		bin_number = (long)floor((double)alpha*edgex[i]);
		if (bin_number >= 0) {
			if (bin_number <= (bin_len - 1) ) {
				AveEdge[bin_number] += Signal[i];
				counts[bin_number] ++;
			}
		}
	}

	// Back fill empty bins
	for (i=0; i<bin_len; i++) {
		j = 0;
		k = 1;
		// Does this bin have data?
		if (counts[i] != 0) {
			// Just take the average on this bin
			AveEdge[i] = (AveEdge[i])/ ((double) counts[i]);
		} else {
			// Empty bin
			nzeros++;
			// Is this the first bin?
			if (i == 0) {
				// Find the first non-zero bin and use its average value
				while (!j) {
					if (counts[i+k] != 0) {
						AveEdge[i] = AveEdge[i+k]/((double) counts[i+k]);
						j = 1;
					} else {
						k++;
					}
				}
			} else {
				// Look backwards until we find a non-zero bin and use it
				while (!j && ((i-k) >= 0) ) { 
					if ( counts[i-k] != 0) {
						AveEdge[i] = AveEdge[i-k];   /* Don't divide by counts since it already happened in previous iteration */
						j = 1;
					}
					else k++;
				}
				// If for some reason we didn't find a value looking backward, look forward
				if ( (i-k) < 0 ) {
					k = 1;
					while (!j) {
						if (counts[i+k] != 0) {
							AveEdge[i] = AveEdge[i+k]/((double) counts[i+k]);
							j = 1;
						} 
						else k++;
					}
				}
			}
		}
	} // End for i

	if (nzeros > 0) {
		fprintf(stderr, "\nWARNING: %d Zero counts found during projection binning.\n", nzeros);
	    fprintf(stderr, "The edge angle may be large, or you may need more lines of data.\n\n");
	}

	return nzeros;
}
/*****************************************************************************/
/* This has been modified from Annex A, to more closely match Annex D and 
   reduce finite difference errors.  Now allows either [-1 1] derivative
   (when separation = 0) or [-1/2 0 1/2] derivative (when separation=1)

   Inputs:   len          length of ESF array
             AveEdge      array of ESF values
             separation   type of derivative 
	                    0 = [-1 1]
			    1 = [-1/2 0 1/2]

   Outputs:  AveTmp       array of original ESF values
             AveEdge      array of derivative (LSF) values
             centroid     centroid of the derivative 
*/
void calculate_derivative(unsigned int len, double *AveTmp, double *AveEdge,
			  double *centroid,int separation)
{
  unsigned long i;
  double dt, dt1;

  dt = 0.0;
  dt1 = 0.0;

  for (i=0; i< len; i++) 
    AveTmp[i] = AveEdge[i];

  for (i=1; i< len-separation; i++) {
    /* Not wasting time with division by 2 since constant factors don't 
       change SFR computation */
    AveEdge[i] = (AveTmp[i+separation]-AveTmp[i-1]);  
	if (separation == 1) { AveEdge[i] *= 0.5; }
    dt += AveEdge[i] * (double)i;
    dt1 += AveEdge[i];
  }
  // Calculate centroid
  *centroid = dt/dt1;
  // Fill in data for first sample (replicate from next data point)
  AveEdge[0] = AveEdge[1];
  // If doing central difference, also replicate last 'valid' entry
  if (separation == 1)  {AveEdge[len-1] = AveEdge[len-2]; }
} // End calculate_derivative
/*****************************************************************************/
void locate_max_PSF(unsigned int len, 
		    double *AveEdge, long *pcnt2) 
{
  unsigned long i;
  double dt=0.0,dt_new=0.0;
  long left = -1L,right = -1L;

  /* find maximim value in Point Spread Function array */
  for (i=0; i<len; i++) {
    dt_new = fabs(AveEdge[i]);
    if ( dt_new > dt) {
      (*pcnt2) = (long) i;
      dt = dt_new;
    }
  }
  /* find leftmost and rightmost occurrence of maximum */
  for (i=0; i<len; i++) {
    dt_new = fabs(AveEdge[i]);
    if ( dt_new == dt ) {
		if ( left < 0 ) { left = i; }
      right = i;
    }
  }
  /* find centre of maxima */
  (*pcnt2) = (right+left)/2;
}

/*****************************************************************************/
void apply_hamming_window(  unsigned short alpha,
			    unsigned int oldlen,
			  unsigned short newxwidth,
			  double *AveEdge, long *pcnt2)
{
	long i,j,k, begin, end, edge_offset;
	double sfrc;

	/* Shift the AvgEdge[i] vector to centre the lsf in the transform window */
	edge_offset = (*pcnt2) - (oldlen/2);
	// Only need to shift if offset !=0
	if (edge_offset != 0) {
		if (edge_offset < 0 ) {
			// Shift data left
			for (i=oldlen-1; i > -edge_offset-1; i--)  { AveEdge[i] = (AveEdge[i+edge_offset]); }
			for (i=0; i < -edge_offset; i++) { AveEdge[i] = 0.00; }
		} else {
			for (i=0; i < oldlen-edge_offset; i++) { AveEdge[i] = (AveEdge[i+edge_offset]); }
			for (i=oldlen-edge_offset; i < oldlen; i++) { AveEdge[i] = 0.00; }
		}
	}
	/* Multiply the LSF data by a Hamming window of width NEWXWIDTH*alpha */
	// Saturate beginning and end points
	begin = (oldlen/2)-(newxwidth*alpha/2);
	if (begin < 0) { begin = 0; }
	end = (oldlen/2)+(newxwidth*alpha/2);
	if (end > oldlen ) { end = oldlen; }
	// Zero values outside the 'valid' region
	for (i=0; i< begin; i++) { AveEdge[i] = 0.0; }
	for (i=end; i< oldlen; i++) { AveEdge[i] = 0.0; }
	// Apply Hamming weights everywhere else
	for (i=begin,j = -newxwidth*alpha/2; i < end; i++,j++) {
		sfrc = 0.54 + 0.46*cos( (M_PI*(double)j)/(newxwidth*alpha/2) );
		AveEdge[i] = (AveEdge[i])*sfrc; 
	}

	if (begin != 0) {
		for (k=0, i=begin; k<newxwidth*alpha; i++,k++) { AveEdge[k] = AveEdge[i]; }
	}
}

/*****************************************************************************/
/* This is the DFT magnitude code                                            */
unsigned short ftwos(long number, double dx, double *lsf, 
		     long ns, double ds, double *sfr)
{
  double a, b, twopi, g;
  long i,j;

  twopi = 2.0 * M_PI;
  for (j = 0; j < ns; j++){
    g = twopi * dx * ds * (double)j;
    for (i = 0, a = 0, b = 0; i < number; i++) { 
      a += lsf[i] * cos(g * (double)(i));
      b += lsf[i] * sin(g * (double)(i)); 
    }
    sfr[j] = sqrt(a * a + b * b); 
  }
  return 0;
}

