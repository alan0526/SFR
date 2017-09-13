/*
	What:	SimpleCapture.h
	Who:	Marwan Shakar
	Where:	TRW Automotive
	When:	???
	Why:	Header for software to do SFR scoring on images
*/

#ifndef SIMPLECAPTURE_H
#define SIMPLECAPTURE_H

#ifndef M_PI
#define M_PI  3.14159265358979323846
#endif

#define PREPROCESS_FLIPLR	0x0000001
#define PREPROCESS_FLIPTB	0x0000002

typedef struct _PGMData 
{   
	int row;
	int col;
	int max_gray;
	int **matrix;
}PGMData;

enum ROIUSAGE 
{ 
	LEFTSIDE,
	RIGHTSIDE,
	BOTH,
	NEVER
};

enum SB_USEAGE 
{ 
	N=0,
	E=1,
	S=2,
	W=3 
};

struct Roi_for_SFR {
	cv::Rect roi;
	int x_Delta;
	int y_Delta;
	bool isHorz;
	double DesiredFreq;
	double SFRLimit;
	ROIUSAGE sideused;
	bool inRange;
	double SFRValue;
	double Edgeangle;
};

struct SFRPlus {
	cv::Point box_ctr;
	unsigned int SB_Size;
	bool roi_used[ROI_PER_SLANTBOX];
	cv::Mat CFid;
	cv::Mat CFid_32BitsPerPixel;
	Roi_for_SFR SFRBox[ROI_PER_SLANTBOX];
};

SFRPlus Boxes[MAX_NUMBER_BOXES];
typedef struct 
{
	cv::Point m_TargetBoxCenter;
	int m_TargetBoxWidth;
	int m_TargetBoxHeight;
} AllTargetBoxesType;

typedef struct SFRflags
{
	int WriteImageLevel;
	int Distance;
	int Demosaic;
	int lTargetDetection;
	bool saturationswitch;
	int lEmbeddedData;
	int lRunningMode;
	bool Display;
	bool Mobileye;
	float fivemwall;
	unsigned int num_frames;
	unsigned int preprocessing;
	float SpatialFreq;
	int EdgeFitOrder;
	unsigned int EdgeAngleCorr;
	unsigned int FuseID[4];
	unsigned int typeoflens;
	bool savelive;
	bool MIPI;
	bool OPTO;
	bool star; //If starfield target is used
	bool showedgeangle;
	int increment;
} flags_t;

void initflags(flags_t& flags) {
	flags.WriteImageLevel = 0;	// Default to only saving whole image (no ROI's), [0=Basic, 1=Detected Objects, 2=Basic, Live mode, 3=More, Live mode, 5=All ROI's]
	flags.Distance = 1;			// Default target distance to 5.0 meter [0=3.5 m, 1=5.0 m]
	flags.Demosaic = 0;			// Default red pixel fix to row/column removal [0=Red Removal, 1=Demosaic]
	flags.lTargetDetection = 1;	// Default Target Detection to edge method [1=Template Match, 2=Circle Detection, 3=Edge Detection]
	flags.lEmbeddedData = 2;	// Default to embedded data in images (None/Included) [1=Embedded, 2=Not embedded]
	flags.lRunningMode = 2;		// Default Target Detection to linear mode [1=HDR, 2=Linear]
	flags.Display = false;		// Default to Displaying Overlay (ROI scores) for live mode 
	flags.num_frames = 0;		// Number of frames in 'to collect' queue
	flags.preprocessing = 0;	// No image preprocessing
	flags.typeoflens = 0;       //Default to main 0 [0 = main, 1 = narrow, 2 = wide]
	flags.SpatialFreq = 67.0;	// Default Spatial Frequency reporting
	flags.EdgeFitOrder = 2;		// Default to linear fit; quadratic = 3
	flags.EdgeAngleCorr = 1;	// Default to old version of not applying edge angle correction
	flags.Mobileye = 1;       // Default to Mobileye software,[1 = Mobileye 12 bits software, 0 = 16bits ordinary software]
	flags.fivemwall = 0; //Default to not scoring 5 m wall with value = 0
	memset(flags.FuseID, 0, 4*sizeof(unsigned int));
	// Initialize to known value to ensure we know if something is amiss
	for (int i=0; i<4; i++) { flags.FuseID[i]=0xBEEF; }
	flags.savelive = true; //
	flags.MIPI = true; //Turn it on for live SFRscoring with MIPI interface
	flags.OPTO = false; //Default to paper-based target, [0= paper-based target, 1 = Optikos target]
	flags.saturationswitch = false; //Default to without saturation detection, [0 = no saturation detection, 1 = saturation detection]
	flags.star = false; //Default to not using starfield target
	flags.showedgeangle = true; //Default to show edge angle
	flags.increment=0;
} // End initflags

void printusage(void) {
	printf("Usage:\nSimpleCapture [options] (filename|live)\n");
	printf("\tfilename\t\tPNG or PGM format image filename or 'live' for live Camera input\n");
	printf("\t-fitorder n\t\tFit edges to polynomial (n=2 linear [default], n=3 quadratic)\n");
	printf("\t-images n\t\tSave intermediate images (n=5 for all, default = none)\n");
	printf("\t-edgeanglecorr\t\tCorrect Spatial Frequency using edge slope\n");
	printf("\t-noembedded\t\tData does not contain embedded metadata\n");
	printf("\t-replaceredpixel\tReplace Red pixels with interpolated clear values\n");
	printf("\t-targettemplate\tUse fiducial matching template for box finding\n");
	printf("\t-targetcircle\tUse circle finding for box finding\n");
	printf("\t-fliplr\t\tFlip incoming image left-to-right\n");
	printf("\t-fliptb\t\tFlip incoming image top-to-bottom\n");
	printf("\t never have words of Narrow and Wide in the path name except the image filenames\n");
}

int GetCmdLineArgument(const int argc, const char **argv, const char *flag ) {
	// This function searches for an instance of 'flag' amonst the command
	// line parameters and returns its index if found, or -1 if it wasn't
	int ind=1;
	// Loop over arguments until one matches the flag sent in
	while ( ind < argc && strcmp(argv[ind],flag)!=0 ) { ind++; }
	// Did we exit the loop because we didn't find one...?
	if ( ind == argc ) { return -1;	}
	// ...or because we did
	else { return ind;	} // End if
} // End GetCmdLineArgument

void DelayExit(const int code) {

	exit(code);
}

void Debug_Report(const char* text) {
#ifdef DEBUG_
	cout << text << endl;
#endif
}

void ProcessArgs(const int argc, char** argv, flags_t& flags) {

	/*
		Amount of image data to write out; default to minimum amount
		0 - Basic (12 bpp image)
		1 - Detected Objects:(12 bpp image, 12 bpp image with detected object)
		2 - Selective 12 BPP Image/Scores Save (Only For Live Scoring). Press S or s to store image
		3 - Selective 12 BPP Image/Scores/ROIs Save (Only For Live Scoring). Press S or s to store image
		4 - TBD
		5 - All Images
	*/

	initflags(flags);

	// If user has requested live imagery, go ahead and set default save mode
	if (!strcmp(argv[argc-1], "live")) {
		flags.WriteImageLevel=2;
	}

	// Is there a command line flag setting this value?
	int argcntr=GetCmdLineArgument( argc, (const char **)argv, "-images" );
	// If a flag is found and there's another parameter after it...
	if ( argcntr > 0 && argcntr+1 < argc )
	{
		// Convert text to number
		int mode=atoi(argv[argcntr+1]);
		// Validate mode number; set flag if valid, otherwise set to default value
		if (strcmp(argv[argc-1], "live")) {
			if ( mode!=0 && mode!=1 && mode!=5 ) { flags.WriteImageLevel = 0; }
			else { flags.WriteImageLevel = mode; }
		} else {
			if ( mode!=2 && mode!=3 ) { flags.WriteImageLevel = 2; }
			else { flags.WriteImageLevel = mode; }
		}
//		else { std::cout << "Unknow Image Saving level- Image Saving Level is Set to " << flags.WriteImageLevel << std::endl; }
	}

	// Has user requested custom spatial frequency value?
	argcntr=GetCmdLineArgument( argc, (const char **)argv, "-lpmm" );
	// If a flag is found and there's another parameter after it...
	if ( argcntr > 0 && argcntr+1 < argc )
	{
		// Convert text to number
		float spat_freq=atof(argv[argcntr+1]);
		// Validate spatial frequency value and set if valid, otherwise set to default value
		if (spat_freq > 0.0f && spat_freq < 100.0f) { flags.SpatialFreq = spat_freq; }
		else { std::cout << "Invalid Spatial Frequency.  Using default of " << flags.SpatialFreq << std::endl; }
	}

	// Has user requested custom spatial frequency value?
	argcntr=GetCmdLineArgument( argc, (const char **)argv, "-fitorder" );
	// If a flag is found and there's another parameter after it...
	if ( argcntr > 0 && argcntr+1 < argc )
	{
		// Convert text to number
		int order=atoi(argv[argcntr+1]);
		// Validate spatial frequency value and set if valid, otherwise set to default value
		if (order > 1 && order < 9) { flags.EdgeFitOrder = order; }
		else { std::cout << "Invalid Edge Fit Polynominal Order.  Using default of " << flags.EdgeFitOrder << std::endl; }
	}

	// If user has requested live imagery, default display to true
	if (flags.WriteImageLevel == 2 || flags.WriteImageLevel == 3) { flags.Display = true; }

	// Is there a command line flag setting this value?
	if ( GetCmdLineArgument( argc, (const char **)argv, "-edgeanglecorr" ) > 0 ) {
		flags.EdgeAngleCorr = 1;
		printf("Turning on Edge Angle Correction\n");
	}

	// Is there a command line flag setting this value?
	if ( GetCmdLineArgument( argc, (const char **)argv, "-closetarget" ) > 0 ) { flags.Distance = 0; }


	// Is there a command line flag setting this value?
	if ( GetCmdLineArgument( argc, (const char **)argv, "-replaceredpixel" ) > 0 ) { flags.Demosaic = 1; }

	// Is there a command line flag setting this value?
	if ( GetCmdLineArgument( argc, (const char **)argv, "-targettemplate" ) > 0 )  { flags.lTargetDetection = 1; }

	//
	if ( GetCmdLineArgument( argc, (const char **)argv, "-targetedge" ) > 0 )  { flags.lTargetDetection = 3; }
	
	// Is there a command line flag setting this value?
	if ( GetCmdLineArgument( argc, (const char **)argv, "-targetcircle" ) > 0 ) { flags.lTargetDetection = 2; }

	// Is there a command line flag setting this value?
	if ( GetCmdLineArgument( argc, (const char **)argv, "-hdr" ) > 0 ) { flags.lRunningMode=1; }

	// Is there a command line flag setting this value?
	if ( GetCmdLineArgument( argc, (const char **)argv, "-embedded" ) > 0 ) { flags.lEmbeddedData = 1; }

	// Is there a command line flag setting this value?
	if ( GetCmdLineArgument( argc, (const char **)argv, "-fliplr" ) > 0 ) { flags.preprocessing |= PREPROCESS_FLIPLR; }

	// Is there a command line flag setting this value?
	if ( GetCmdLineArgument( argc, (const char **)argv, "-fliptb" ) > 0 ) { flags.preprocessing |= PREPROCESS_FLIPTB; }

	// Is MIPI interface used
	if ( GetCmdLineArgument( argc, (const char **)argv, "-MIPI" ) > 0 ) { flags.MIPI = true; }

	if ( GetCmdLineArgument( argc, (const char **)argv, "live" ) > 0 ) { flags.Mobileye = 0; }

	// Is Optikos target used?
	if ( GetCmdLineArgument( argc, (const char **)argv, "-opto" ) > 0 ) { flags.OPTO = true; flags.lTargetDetection = 1;}

	//Find out which type of lens is used
	if ( strstr(argv[argc-1],"Narrow")||strstr(argv[argc-1],"narrow") ) { flags.typeoflens = 1; }

	if ( strstr(argv[argc-1],"Wide")||strstr(argv[argc-1],"wide") ) { flags.typeoflens = 2; }

    if ( GetCmdLineArgument( argc, (const char **)argv, "-ordinary" ) > 0 ) { flags.Mobileye = 0; }

	if ( GetCmdLineArgument( argc, (const char **)argv, "-star" ) > 0 ) { flags.star = true; }

	if ( GetCmdLineArgument( argc, (const char **)argv, "-noshowedgeangle" ) > 0 ) { flags.showedgeangle = false; }

} // End ProcessArgs()

#endif