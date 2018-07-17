///////////////////////////////////////////////////////////////
// D E F I N E S
///////////////////////////////////////////////////////////////

//#define Check_Error_Frames 1
//#define NO_CAMERA
// default amount of frames to capture...can also specify on command line
#define DEFAULT_IMAGE_WIDTH     1280 
// default amount of frames to capture...can also specify on command line
//#define DEFAULT_IMAGE_HEIGHT    960 
#define DEFAULT_IMAGE_HEIGHT    960
// default amount of frames to capture...can also specify on command line
#define DEFAULT_NUM_FRAMES     100
// the amount of tries before giving up trying to grab a valid frame. 
#define MAX_BADFRAME_TRIES      100

// No Key Pressed
#define NO_KEY_PRESSED -1

#define MAX_NUMBER_BOXES 11
#define ROI_PER_SLANTBOX 4
#define KERNAL_SIZE 7
#define VERSION 1
#define RELEASE 1
#define BUILD 0

#ifndef DebugError
#define DebugError 0
#endif

///////////////////////////////////////////////////////////////
// M A C R O S
///////////////////////////////////////////////////////////////
#define HI(num) (((num) & 0x0000FF00) >> 8)
#define LO(num) ((num) & 0x000000FF) 

///////////////////////////////////////////////////////////////
// I N C L U D E S
///////////////////////////////////////////////////////////////
// C++ Includes
#include <iostream>
#include <fstream>
#include <ctime>
// C includes
//#include <stdio.h>
#include <stdlib.h>
#include <io.h>
#include <math.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#ifdef _WIN32
#include <conio.h>
#else
#define _getch getchar
#define sscanf_s sscanf
#endif


#include "stdafx.h"
#include "windows.h"
#include <iomanip>
#include <sstream>

// Aptina Includes
#ifndef APBASE_LITE
#include "apbase.h"
#else
#include "apbase_lite.h"
#endif
//#include <atlbase.h>



//OmniVision Includes and defines
#define YUV            0
#define RGBRAW         1
#define RGBRAWC        11
#define RGB565         2
#define RGB422         3
#define RGB555         4
#define RGB444         5
#define OVJPEG         6
#define OMNI           1
#define APTI           0

//unsigned char   *pBuffer=NULL;
int             camerabrand = -1;  //flag to differentiate Aptina from OmniVision sensor


// OpenMP
#if defined(_OPENMP)
	#define <omp.h>
#endif

// OpenCV includes
//#include "opencv2/core/cvstd.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
// PNG includes
#undef _UNICODE
#include "include\IL\il.h"
// User includes
#include "SimpleCapture.h"
#include "sfr.h"

using namespace std;

///////////////////////////////////////////////////////////////
// E X T E R N S
///////////////////////////////////////////////////////////////

//typedef int(OVImgProcCallBack)(unsigned char *buf,int dl); //or void type

extern "C" __declspec(dllexport) int OVCam_ConnectionCheck();
extern "C" __declspec(dllexport) int InitOVDriverSCCB();
extern "C" __declspec(dllexport) int InitOVDriver(int width, int height);
extern "C" __declspec(dllexport) int ReadRegister(int SCCB_Addr, int RegAddr);
extern "C" __declspec(dllexport) int InitOVDriver(int xSize,int ySize);
extern "C" __declspec(dllexport) int FinishDriver();
extern "C" __declspec(dllexport) int SendRegFile2(char *fname);
//extern "C" __declspec(dllexport) int SetCBRaw(OVImgProcCallBack pFunc);
extern "C" __declspec(dllexport) int ChangeSeq(int seq);

extern "C" __declspec(dllexport) void AcquireImage (unsigned char *pBuf);
extern "C" __declspec(dllexport) void WriteRegister(int SCCB_Addr, int RegAddr,int RegData);
extern "C" __declspec(dllexport) void ChangeResolution(int xSize, int ySize);
extern "C" __declspec(dllexport) void ChangeSensorMode(int Mode);
extern "C" __declspec(dllexport) void SendRegFile(char* fname);

extern "C" __declspec(dllexport) char* GetBaseIFVersion();

extern "C" short sfrProc(double **,double **, int*, double *, 
			unsigned short, int *, double *, int *, int *, double*, double*, 
			int, int, int,unsigned int);

#pragma comment( lib, "DevIL.lib" )

///////////////////////////////////////////////////////////////
// G L O B A L S
///////////////////////////////////////////////////////////////

bool DeAllocate12BPP_Memory = false;

AllTargetBoxesType AllTargetBoxes[MAX_NUMBER_BOXES];

cv::Point m_AllTargetCenterBoxes[MAX_NUMBER_BOXES];
bool m_DetectedTargetCenterBoxes[MAX_NUMBER_BOXES];
cv::Mat CFid;
cv::Mat CFid_32BitsPerPixel;
cv::Mat CFid2;
cv::Mat CFid2_32BitsPerPixel;
int m_FrameCounter;
char lSaturationLog = 'N';
char szCharWithoutExePath[1024] = ".\\";	// Default to current path
char xmlpath[1024] = "C:\\Users\\xuz12\\Desktop\\AptinaSFRscoring\\OmniVisionSFRScoring - quad target\\xmlfiles";
bool Once = false;
bool Display = true;
cv::Rect rectm(0,0,50,50);
int lKey = NO_KEY_PRESSED;

////////////////////////////////////////////////////////////////////////////////
// Helper functions used in Main()
////////////////////////////////////////////////////////////////////////////////

//
//  Callback for INI PROMPT= command. Some initialization files
//  use this.
//

//void g_CBRAWCallBackProc(unsigned char *buf,int dl)
//{
	//if(dl==VGAX*VGAY*2)
//	{
//		memcpy(pBuffer,buf, dl);
		//count++;
		//cout<<"frame count"<<count<<": "<<dl<<endl;
		/*if (count%3==0)
		{
			ostringstream convert;   
			convert << count;
			s_file="raw_image.raw";
			s_file=s_file.append(convert.str());
			ofstream file(s_file, ios::out | ios::binary);
			if (file.is_open())
			{
				int a_dl = min(VGAX*VGAY*2,dl);
				cout<<"dl used: "<<a_dl<<endl;
				file.write(reinterpret_cast<const char*>(g_IBuf), a_dl);
				file.close();
				cout<<"image "<<count<<" has been saved...\n";
			}
		}*/
//	}
//}

__int8 showtemp()
{
  unsigned __int8 R;
  R = ReadRegister(0x60,0x304c);
  if(R <= 0xC0)
	  return R;
  else
	  return R- 0x100;
}

int APBASE_DECL MultipleChoice(void *pContext,
                               const char *szMessage,
                               const char *szChoices)
{
    int         nChoices = 0;

    if (szMessage)
        printf("\n%s\n", szMessage);

    if (szChoices == NULL || szChoices[0] == 0)
        return 0;   //   no choices, just an informational message

    printf("%4d. Cancel\n", -1);
    printf("%4d. Skip this\n", 0);
    for (const char *p = szChoices; nChoices < 25; ++p)
    {
        if (strlen(p) == 0)
            break;
        ++nChoices;
        printf("%4d. %s\n", nChoices, p);
        while (*p)
            ++p;
    }

    printf("Enter selection [1]: ");
    char s[256];
    fgets(s, sizeof(s), stdin);
    int n = 1;
    sscanf_s(s, "%i", &n);
    return n;
}

//  Callback for INI file LOG= command
void APBASE_DECL MyLogComment(void *pContext, const char *szComment)
{
    printf("%s\n", szComment);
}

//  Callback for user query (based on MessageBox)
int APBASE_DECL MyErrorMessage(void *pContext, const char *szMessage, unsigned int mbType)
{
    printf("%s\n", szMessage);
    switch (mbType & 0xF)
    {
    case AP_MSGTYPE_OK:                 return AP_MSG_OK;
    case AP_MSGTYPE_OKCANCEL:           return AP_MSG_OK;
    case AP_MSGTYPE_ABORTRETRYIGNORE:   return AP_MSG_ABORT;
    case AP_MSGTYPE_YESNOCANCEL:        return AP_MSG_YES;
    case AP_MSGTYPE_YESNO:              return AP_MSG_YES;
    case AP_MSGTYPE_RETRYCANCEL:        return AP_MSG_CANCEL;
    case AP_MSGTYPE_CANCELTRYCONTINUE:  return AP_MSG_CANCEL;
    }
    return AP_MSG_OK;
}

//  Callback for Python print() calls, etc.
void APBASE_DECL MyScriptOutput(void *pContext, int nUnit, const char *szString)
{
    switch (nUnit)
    {
    case 0:
    default:
        printf("%s", szString);
        break;
    case 1:
        fprintf(stderr, "%s", szString);
        break;
    }
}

int **allocate_dynamic_matrix(int row, int col) 
{
	int **ret_val; 
	int i;  
	ret_val = (int **)malloc(sizeof(int *) * row);
	if (ret_val == NULL)
	{
		perror("memory allocation failure");
		exit(EXIT_FAILURE);
	} 
	for (i = 0; i < row; ++i)
	{
		ret_val[i] = (int *)malloc(sizeof(int) * col);
		if (ret_val[i] == NULL)
		{ 
			perror("memory allocation failure");  
			exit(EXIT_FAILURE); 
		} 
	}
	return ret_val; 
}

void deallocate_dynamic_matrix(int **matrix, int row)
{
	int i;
	for (i = 0; i < row; ++i) 
		free(matrix[i]);
	free(matrix);
} 


void SkipComments(FILE *fp)
{ 
	int ch;
	char line[100];
	while ((ch = fgetc(fp)) != EOF && isspace(ch));
		if (ch == '#')
		{
			fgets(line, sizeof(line), fp); 
			SkipComments(fp);
		} 
		else
			fseek(fp, -1, SEEK_CUR); 
} 

/*for reading:*/
PGMData* readPGM(const char *file_name, PGMData *data) 
{
	FILE *pgmFile;
	char version[3];
	int i, j;  
	int lo, hi;
	pgmFile = fopen(file_name, "rb");
	if (pgmFile == NULL) 
	{
		perror("cannot open file to read");  
		exit(EXIT_FAILURE); 
	}
	fgets(version, sizeof(version), pgmFile);  
	if (strcmp(version, "P5"))
	{
		fprintf(stderr, "Wrong file type!\n");  
		exit(EXIT_FAILURE);  
	} 
	SkipComments(pgmFile);
	fscanf(pgmFile, "%d", &data->col); 
	SkipComments(pgmFile);
	fscanf(pgmFile, "%d", &data->row); 
	SkipComments(pgmFile);
	fscanf(pgmFile, "%d", &data->max_gray);
	fgetc(pgmFile); 
	data->matrix = allocate_dynamic_matrix(data->row, data->col);
	// Is this data 16-bpp or 8-bpp?
	if (data->max_gray > 255)
	{
		for (i = 0; i < data->row; ++i)
			for (j = 0; j < data->col; ++j)
			{
				// Default for PGM is Bigp-Endian storage
				hi = fgetc(pgmFile);
				lo = fgetc(pgmFile); 
				data->matrix[i][j] = (hi << 8) + lo;
			} 
	}
	else 
	{
		for (i = 0; i < data->row; ++i) {
			for (j = 0; j < data->col; ++j) {
				data->matrix[i][j] = static_cast<int>(fgetc(pgmFile))<<8;  
			}
		}
	}

	fclose(pgmFile); 
	return data;
} 

PGMData* setPGM(ILubyte *bytes, PGMData *data) 
{
	int i, j;  
	int lo, hi;
	data->matrix = allocate_dynamic_matrix(data->row, data->col);
	int k = 0;
	if (data->max_gray > 255)
	{		
		for (i = 0; i < data->row; ++i)
		{
			for (j = 0; j < data->col; ++j)
			{
				lo = bytes[k];
				k++;
				hi = bytes[k]; 
				k++;
				data->matrix[i][j] = (hi << 8) + lo;
			} 
		}
	}
	else 
	{
		for (i = 0; i < data->row; ++i)
		{
			for (j = 0; j < data->col; ++j)
			{
				lo = bytes[k]; 
				k++;
				data->matrix[i][j] = lo;  
			}
		}
	}

	return data;

}
/*and for writing*/  
void writePGM(const char *filename, const PGMData *data) 
{
	FILE *pgmFile; 
	int i, j;   
	pgmFile = fopen(filename, "wb"); 
	if (pgmFile == NULL)
	{
		perror("cannot open file to write");   
		exit(EXIT_FAILURE);
	}
	// Write Header
	fprintf(pgmFile, "P5\n");  
	fprintf(pgmFile, "%d %d\n", data->col, data->row); 
	fprintf(pgmFile, "%d\n", data->max_gray); 
	// Write pixel data
	for (i = 0; i < data->row; ++i)
	{
		for (j = 0; j < data->col; ++j)
		{
			int pixdata=data->matrix[i][j];
			// If we have multi-byte writes, PGM requires Big Endian
			if (data->max_gray > 255) {
				fputc(HI(pixdata), pgmFile); 
			}
		fputc(LO(pixdata), pgmFile);   
		}
	}
	// Close file handle
	fclose(pgmFile); 
	// De-allocate memory if
//	deallocate_dynamic_matrix(data->matrix, data->row); 
} 

#ifdef false
/*and for writing*/  
void writePGM_16(const char *filename, const PGMData *data, int x, int y, int height, int width, int gray, bool deallocate_memory = true) 
{
	FILE *pgmFile; 
	int i, j, lo;  
	pgmFile = fopen(filename, "wb"); 
	if (pgmFile == NULL)
	{
		perror("cannot open file to write");   
		exit(EXIT_FAILURE);
	}
	// Write PGM Header into file
	fprintf(pgmFile, "P5\n");  
	fprintf(pgmFile, "%d %d\n", width, height); 
	fprintf(pgmFile, "%d\n", gray); 
	// Loop over array and write elements	
	for (i = y; i < y+ height; ++i)
	{
		for (j = x; j < x + width; ++j)
		{
			int pixdata=data->matrix[i][j];
			// If we have multi-byte writes, PGM requires Big Endian
			if (data->max_gray > 255) {
				fputc(HI(pixdata), pgmFile); 
			}
			fputc(LO(pixdata), pgmFile);   
		}
	}
	// Close file handle
	fclose(pgmFile); 
	if (deallocate_memory)
		deallocate_dynamic_matrix(data->matrix, data->row); 
} 
#endif

void ImageWrite(std::string filename, cv::Mat ImageData)
{
	try {
		cv::imwrite(filename, ImageData);
	}
	catch (runtime_error& ex) {
		cout << "Error: unable to store image" << endl;
	}
}

bool InitTemplateAllBoxes(string filePath)
{
	string lFileName = filePath + "\\cfid_fullres_box";
	bool error = true;
	for (int i = 0; i < MAX_NUMBER_BOXES; i++)
	{
		string lFullFileName = lFileName + std::to_string((_ULonglong) i) +".pgm";
		Boxes[i].CFid = cv::imread(lFullFileName,CV_LOAD_IMAGE_ANYDEPTH);
		if (CFid.rows == 0 )
		{
			cout <<"error reading fidduicial template for Box"<<i << endl;
			error = false;
		}
		else			
			Boxes[i].CFid.convertTo(Boxes[i].CFid_32BitsPerPixel,CV_32F);
	}
	return error;
}

bool InitTemplates(string filePath)
{
	string Filename = filePath + "\\cfid_fullres_11boxes.pgm";
	string Filename2 = filePath + "\\cfid_fullres_11boxes_corners.pgm";
	
	bool error = true;
	
	CFid = cv::imread(Filename,CV_LOAD_IMAGE_ANYDEPTH);	
	
	cout << Filename << endl;
	if (CFid.rows == 0 )
	{
		cout <<"error reading the central fidduicial template" << endl;
		error = false;
	}
	
	CFid.convertTo(CFid_32BitsPerPixel,CV_32F);

	CFid2 = cv::imread(Filename2,CV_LOAD_IMAGE_ANYDEPTH);	// Force Greyscale load.
	
	if (CFid2.rows == 0 )
	{
		cout <<"error reading the outside fidduicial template" << endl;
		error = false;
	}
	CFid2.convertTo(CFid2_32BitsPerPixel,CV_32F);

	return error;
}

void MeasureRCCCAttributes(cv::Mat& ImageData, int* redchan, float* scaling) {
	// Find sum of each of the four RCCC channels 
	unsigned int chansums[4] = {0, 0, 0, 0};
	for (int row=0; row<ImageData.rows; row+=2) 
	{
		for (int col=0; col<ImageData.cols; col+=2)
		{
			chansums[0] += ImageData.at<ushort>(row,col);
			chansums[1] += ImageData.at<ushort>(row,col+1);
			chansums[2] += ImageData.at<ushort>(row+1,col);
			chansums[3] += ImageData.at<ushort>(row+1,col+1);
		}
	}

	// Find channel with lowest signal; this is the red channel
	int redpix = 0;
	for (int chan=1; chan<4; chan++) {
		// Subtract Black floor value (~220 for AR0132) to ensure normalization below is correct
		chansums[chan] -= (200 * ((ImageData.rows*ImageData.cols)>>2) );
		// Find channel with minimum value
		if (chansums[chan] < chansums[redpix] ) { redpix = chan; }
	} // end for chan

	// Calculate normalization factors; normalize to pixel diagonal from red
	for (int chan=0; chan<4; chan++) { scaling[chan] = (float)chansums[3-redpix] / chansums[chan]; }

	// Update red channel
	*redchan = redpix;

} // End MeasureRCCCAttributes

bool acceptLinePair(cv::Vec2f line1, cv::Vec2f line2, float minTheta)
{
    float theta1 = line1[1], theta2 = line2[1];

    if(theta1 < minTheta)
    {
        theta1 += (float)CV_PI; // dealing with 0 and 180 ambiguities...
    }

    if(theta2 < minTheta)
    {
        theta2 += (float)CV_PI; // dealing with 0 and 180 ambiguities...
    }

    return abs(theta1 - theta2) > minTheta;
}

std::vector<cv::Point2f> lineToPointPair(cv::Vec2f line)
{
    vector<cv::Point2f> points;

    float r = line[0], t = line[1];
    double cos_t = cos(t), sin_t = sin(t);
    double x0 = r*cos_t, y0 = r*sin_t;
    double alpha = 1000;

    points.push_back(cv::Point2f((float)(x0 + alpha*(-sin_t)), (float)(y0 + alpha*cos_t)));
    points.push_back(cv::Point2f((float)(x0 - alpha*(-sin_t)), (float)(y0 - alpha*cos_t)));

    return points;
}

// the long nasty wikipedia line-intersection equation...bleh...
cv::Point2f computeIntersect(cv::Vec2f line1, cv::Vec2f line2)
{
    vector<cv::Point2f> p1 = lineToPointPair(line1);
    vector<cv::Point2f> p2 = lineToPointPair(line2);

    float denom = (p1[0].x - p1[1].x)*(p2[0].y - p2[1].y) - (p1[0].y - p1[1].y)*(p2[0].x - p2[1].x);
    cv::Point2f intersect(((p1[0].x*p1[1].y - p1[0].y*p1[1].x)*(p2[0].x - p2[1].x) -
                       (p1[0].x - p1[1].x)*(p2[0].x*p2[1].y - p2[0].y*p2[1].x)) / denom,
                      ((p1[0].x*p1[1].y - p1[0].y*p1[1].x)*(p2[0].y - p2[1].y) -
                       (p1[0].y - p1[1].y)*(p2[0].x*p2[1].y - p2[0].y*p2[1].x)) / denom);

    return intersect;
}

int LocateMultipleFeaturesDetection2(cv::Mat img, cv::Mat img_display, int width, int height, const flags_t modeflags)
{
	cv::Mat lTempImg = cv::Mat(cv::Mat::zeros(height, width, CV_8U));

	if (((modeflags.WriteImageLevel > 3) && (lKey == NO_KEY_PRESSED)) || (modeflags.num_frames && (modeflags.WriteImageLevel == 3)))
		ImageWrite("ObjectDetection_Original.pgm", img_display);
	img.convertTo(lTempImg,CV_8U);
	if (((modeflags.WriteImageLevel > 3) && (lKey == NO_KEY_PRESSED)) || (modeflags.num_frames && (modeflags.WriteImageLevel == 3)))
		ImageWrite("ObjectDetection_8U.pgm", lTempImg);
	threshold(lTempImg,  lTempImg, 30, 255, CV_THRESH_BINARY);
	if (((modeflags.WriteImageLevel > 3) && (lKey == NO_KEY_PRESSED)) || (modeflags.num_frames && (modeflags.WriteImageLevel == 3)))
		ImageWrite("ObjectDetection_CV_THRESH_BINARY.pgm", lTempImg);
	dilate(lTempImg,  lTempImg, cv::Mat(), cv::Point(-1, -1), 2, 1, 1);
	if (((modeflags.WriteImageLevel > 3) && (lKey == NO_KEY_PRESSED)) || (modeflags.num_frames && (modeflags.WriteImageLevel == 3)))
		ImageWrite("ObjectDetection_CV_THRESH_BINARY_dilate.pgm", lTempImg);


    //Find the contours. Use the contourOutput Mat so the original image doesn't get overwritten
    std::vector<std::vector<cv::Point> > contours;
    cv::Mat contourOutput = cv::Mat(lTempImg.clone());
    cv::findContours( contourOutput, contours, CV_RETR_TREE, CV_CHAIN_APPROX_NONE );

	if (((modeflags.WriteImageLevel >3)&& (lKey == NO_KEY_PRESSED)) || (modeflags.num_frames && (modeflags.WriteImageLevel == 3)))
		ImageWrite("ObjectDetection_CV_THRESH_BINARY_dilate_FindContoursOutput.pgm", contourOutput);


	vector<vector<cv::Point> > contours_poly( contours.size() );
	vector<cv::Rect> boundRect( contours.size() );
	vector<cv::Point2f>center( contours.size() );
	vector<float>radius( contours.size() );

    for (size_t idx = 0; idx < contours.size(); idx++) 
	{
		cv::approxPolyDP( cv::Mat(contours[idx]), contours_poly[idx], 3, true );
		boundRect[idx] = cv::boundingRect( cv::Mat(contours_poly[idx]) );
		cv::minEnclosingCircle( (cv::Mat)contours_poly[idx], center[idx], radius[idx] );
	}
	/// Draw polygonal contour + bonding rects + circles
	cv::Mat drawing = cv::Mat::zeros( contourOutput.size(), CV_8UC1 );
	for( int i = 0; i< (int)contours.size(); i++ )
	{
		cv::Scalar color = cv::Scalar( 255, 0, 0 );
		drawContours( img_display, contours_poly, i, color, 1, 8, vector<cv::Vec4i>(), 0, cv::Point() );
		if ((((boundRect[i].width > 65) && (boundRect[i].width < 180) && (modeflags.Distance == 1)) && ((boundRect[i].height > 65) && (boundRect[i].height < 180) && (modeflags.Distance == 1))) ||  // 5 meters 
			((boundRect[i].width > 100) && ( modeflags.Distance == 0)))  // 3.5 meters
		{
			if (modeflags.WriteImageLevel >=5)
				rectangle( img_display, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
			if(boundRect[i].y < (int)(height/3))
			{
				if (boundRect[i].x <= (int)(width/3))
				{
					//if (!m_DetectedTargetCenterBoxes[10])
					//{
						AllTargetBoxes[10].m_TargetBoxHeight = boundRect[i].height;
						AllTargetBoxes[10].m_TargetBoxWidth = boundRect[i].width;
						AllTargetBoxes[10].m_TargetBoxCenter.x = boundRect[i].x + boundRect[i].width/2;
						AllTargetBoxes[10].m_TargetBoxCenter.y = boundRect[i].y + boundRect[i].height/2;
						m_DetectedTargetCenterBoxes[10] = true;
					//}
					//else
					//	continue;
				}
				else if (boundRect[i].x >= (int)(2*width/3))
				{
					//if (!m_DetectedTargetCenterBoxes[5])
					//{
						AllTargetBoxes[5].m_TargetBoxHeight = boundRect[i].height;
						AllTargetBoxes[5].m_TargetBoxWidth = boundRect[i].width;
						AllTargetBoxes[5].m_TargetBoxCenter.x = boundRect[i].x + boundRect[i].width/2;
						AllTargetBoxes[5].m_TargetBoxCenter.y = boundRect[i].y + boundRect[i].height/2;
						m_DetectedTargetCenterBoxes[5] = true;
					//}
					//else
					//	continue;
				}
				else
				{
					//if (!m_DetectedTargetCenterBoxes[1])
					//{
						AllTargetBoxes[1].m_TargetBoxHeight = boundRect[i].height;
						AllTargetBoxes[1].m_TargetBoxWidth = boundRect[i].width;
						AllTargetBoxes[1].m_TargetBoxCenter.x = boundRect[i].x + boundRect[i].width/2;
						AllTargetBoxes[1].m_TargetBoxCenter.y = boundRect[i].y + boundRect[i].height/2;
						m_DetectedTargetCenterBoxes[1] = true;
					//}
					//else
					//	continue;
				}
			}
			else if(boundRect[i].y >= (int)(2*height/3))
			{
				if (boundRect[i].x <= (int)(width/3))
				{
					//if (!m_DetectedTargetCenterBoxes[8])
					//{
						AllTargetBoxes[8].m_TargetBoxHeight = boundRect[i].height;
						AllTargetBoxes[8].m_TargetBoxWidth = boundRect[i].width;
						AllTargetBoxes[8].m_TargetBoxCenter.x = boundRect[i].x + boundRect[i].width/2;
						AllTargetBoxes[8].m_TargetBoxCenter.y = boundRect[i].y + boundRect[i].height/2;
						m_DetectedTargetCenterBoxes[8] = true;
					//}
					//else
					//	continue;
				}
				else if (boundRect[i].x >= (int)(2*width/3))
				{
					//if (!m_DetectedTargetCenterBoxes[7])
					//{
						AllTargetBoxes[7].m_TargetBoxHeight = boundRect[i].height;
						AllTargetBoxes[7].m_TargetBoxWidth = boundRect[i].width;
						AllTargetBoxes[7].m_TargetBoxCenter.x = boundRect[i].x + boundRect[i].width/2;
						AllTargetBoxes[7].m_TargetBoxCenter.y = boundRect[i].y + boundRect[i].height/2;
						m_DetectedTargetCenterBoxes[7] = true;
					//}
					//else
					//	continue;
				}
				else
				{
					//if (!m_DetectedTargetCenterBoxes[3])
					//{
						AllTargetBoxes[3].m_TargetBoxHeight = boundRect[i].height;
						AllTargetBoxes[3].m_TargetBoxWidth = boundRect[i].width;
						AllTargetBoxes[3].m_TargetBoxCenter.x = boundRect[i].x + boundRect[i].width/2;
						AllTargetBoxes[3].m_TargetBoxCenter.y = boundRect[i].y + boundRect[i].height/2;
						m_DetectedTargetCenterBoxes[3] = true;
					//}
					//else
					//	continue;
				}
			}
			else
			{
				if (boundRect[i].x <= (int)(width/5))
				{
					//if (!m_DetectedTargetCenterBoxes[9])
					//{
						AllTargetBoxes[9].m_TargetBoxHeight = boundRect[i].height;
						AllTargetBoxes[9].m_TargetBoxWidth = boundRect[i].width;
						AllTargetBoxes[9].m_TargetBoxCenter.x = boundRect[i].x + boundRect[i].width/2;
						AllTargetBoxes[9].m_TargetBoxCenter.y = boundRect[i].y + boundRect[i].height/2;
						m_DetectedTargetCenterBoxes[9] = true;
					//}
					//else
					//	continue;
				}
				else if ((boundRect[i].x >= (int)(width/5))
					&& (boundRect[i].x <= (int)(2*width/5)))
				{
					//if (!m_DetectedTargetCenterBoxes[4])
					//{
						AllTargetBoxes[4].m_TargetBoxHeight = boundRect[i].height;
						AllTargetBoxes[4].m_TargetBoxWidth = boundRect[i].width;
						AllTargetBoxes[4].m_TargetBoxCenter.x = boundRect[i].x + boundRect[i].width/2;
						AllTargetBoxes[4].m_TargetBoxCenter.y = boundRect[i].y + boundRect[i].height/2;
						m_DetectedTargetCenterBoxes[4] = true;
					//}
					//else
					//	continue;
				}
				else if ((boundRect[i].x >= (int)(2*width/5))
					&& (boundRect[i].x <= (int)(3*width/5)))
				{
					//if (!m_DetectedTargetCenterBoxes[0])
					//{
						AllTargetBoxes[0].m_TargetBoxHeight = boundRect[i].height;
						AllTargetBoxes[0].m_TargetBoxWidth = boundRect[i].width;
						AllTargetBoxes[0].m_TargetBoxCenter.x = boundRect[i].x + boundRect[i].width/2;
						AllTargetBoxes[0].m_TargetBoxCenter.y = boundRect[i].y + boundRect[i].height/2;
						m_DetectedTargetCenterBoxes[0] = true;
//					}
//					else
//						continue*/;
				}
				else if ((boundRect[i].x >= (int)(3*width/5))
					&& (boundRect[i].x <= (int)(4*width/5)))
				{
					//if (!m_DetectedTargetCenterBoxes[2])
					//{
						AllTargetBoxes[2].m_TargetBoxHeight = boundRect[i].height;
						AllTargetBoxes[2].m_TargetBoxWidth = boundRect[i].width;
						AllTargetBoxes[2].m_TargetBoxCenter.x = boundRect[i].x + boundRect[i].width/2;
						AllTargetBoxes[2].m_TargetBoxCenter.y = boundRect[i].y + boundRect[i].height/2;
						m_DetectedTargetCenterBoxes[2] = true;
					//}
					//else
					//	continue;
				}
				else
				{
					//if (!m_DetectedTargetCenterBoxes[6])
					//{
						AllTargetBoxes[6].m_TargetBoxHeight = boundRect[i].height;
						AllTargetBoxes[6].m_TargetBoxWidth = boundRect[i].width;
						AllTargetBoxes[6].m_TargetBoxCenter.x = boundRect[i].x + boundRect[i].width/2;
						AllTargetBoxes[6].m_TargetBoxCenter.y = boundRect[i].y + boundRect[i].height/2;
						m_DetectedTargetCenterBoxes[6] = true;
					//}
					//else
					//	continue;
				}
			}
		}
		if (((int)radius[i] > 25) && ((int)radius[i] < 30) &&(modeflags.WriteImageLevel >=5))
		{
			circle( img_display, center[i], (int)radius[i], color, 2, 8, 0 );
		}
	}
	if (((modeflags.WriteImageLevel >3)&& (lKey == NO_KEY_PRESSED)) || (modeflags.num_frames && (modeflags.WriteImageLevel == 3)))
		ImageWrite("ObjectDetection_CV_THRESH_BINARY_dilate_FindContours_ContourImage.pgm", img_display);
#if DebugError
#endif
	// Count how many boxes we got
	int BoxesDetected=0;
	for (int cnt=0; cnt<12; cnt++) {
		if (m_DetectedTargetCenterBoxes[cnt]) { BoxesDetected++; }
	}

	return BoxesDetected;
}

int LocateMultipleFeaturesDetection(cv::Mat img, cv::Mat img_display, int width, int height, const flags_t modeflags)
{
	cv::Mat lTempImg = cv::Mat::zeros(height, width, CV_8U);
	if (modeflags.WriteImageLevel >=5)
		ImageWrite("ObjectDetection_Original.pgm", img_display);
	img.convertTo(lTempImg,CV_8U);
	if (modeflags.WriteImageLevel >=5)
		ImageWrite("ObjectDetection_8U.pgm", lTempImg);
	GaussianBlur(lTempImg, lTempImg, cv::Size(7, 7), 2.0, 2.0);
	if (modeflags.WriteImageLevel >=5)
		ImageWrite("ObjectDetection_8U_GaussianBlur.pgm", lTempImg);
	threshold(lTempImg,  lTempImg, 10, 255, CV_THRESH_OTSU);
	if (modeflags.WriteImageLevel >=5)	
		ImageWrite("ObjectDetection_8U_GaussianBlur_CV_THRESH_OTSU.pgm", lTempImg);
 //   erode(lTempImg,  lTempImg, cv::Mat(), cv::Point(-1, -1), 2, 1, 1);
	//if (modeflags.WriteImageLevel >=5)
	//	ImageWrite("ObjectDetection_8U_GaussianBlur_CV_THRESH_OTSU_erode.pgm", lTempImg);
	//dilate(lTempImg,  lTempImg, cv::Mat(), cv::Point(-1, -1), 2, 1, 1);
	//if (modeflags.WriteImageLevel >=5)
	//	ImageWrite("ObjectDetection_8U_GaussianBlur_CV_THRESH_OTSU_erode_dilate.pgm", lTempImg);


	Canny(lTempImg,  lTempImg, 200, 100, 3 );
	if (modeflags.WriteImageLevel >=5)
		ImageWrite("ObjectDetection_8U_GaussianBlur_CV_THRESH_OTSU_dilate_erode_Canny.pgm", lTempImg);

	vector<cv::Vec3f> circles;
	if (modeflags.Distance == 0)
		HoughCircles( lTempImg, circles, CV_HOUGH_GRADIENT, 1, height/8, 200,25, 27, 110); // 3.6 m
	else if (modeflags.Distance == 1)
		HoughCircles( lTempImg, circles, CV_HOUGH_GRADIENT, 1, height/8, 200,25, 10, 90); // 5m


	//vector<cv::Vec2f> lines;
 //   HoughLines( lTempImg, lines, 1, CV_PI/180, 117, 0, 0 );

	//cout << "Detected Lines: " << lines.size() << " lines." << endl;
	// // compute the intersection from the lines detected...
 //   vector<cv::Point2f> intersections;
 //   for( size_t i = 0; i < lines.size(); i++ )
 //   {
 //       for(size_t j = 0; j < lines.size(); j++)
 //       {
 //           cv::Vec2f line1 = lines[i];
 //           cv::Vec2f line2 = lines[j];
 //           if(acceptLinePair(line1, line2, CV_PI / 32))
 //           {
 //               cv::Point2f intersection = computeIntersect(line1, line2);
 //               intersections.push_back(intersection);
 //           }
 //       }

 //   }
	//cout << "Detected Lines Intersection: " << intersections.size() << " intersections." << endl;

 //   if(intersections.size() > 0)
 //   {
 //       vector<cv::Point2f>::iterator i;
 //       for(i = intersections.begin(); i != intersections.end(); ++i)
 //       {
 //           cout << "Intersection is " << i->x << ", " << i->y << endl;
 //           circle(img_display, *i, 1, cv::Scalar(0, 255, 0), 30);
 //       }
 //   }

	//cout << "Size = " << circles.size() << end;lh
    for( size_t i = 0; i < circles.size(); i++ )
    {
		cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
		int radius = cvRound(circles[i][2]);

		cv:: Point TargetCenterDetection = center;

		if(TargetCenterDetection.y < (int)(height/3))
		{
			if (TargetCenterDetection.x <= (int)(width/3))
			{
				if (!m_DetectedTargetCenterBoxes[10])
				{
					m_AllTargetCenterBoxes[10] = TargetCenterDetection;
					m_DetectedTargetCenterBoxes[10] = true;
				}
				else
					continue;
			}
			else if (TargetCenterDetection.x >= (int)(2*width/3))
			{
				if (!m_DetectedTargetCenterBoxes[5])
				{
					m_AllTargetCenterBoxes[5] = TargetCenterDetection;
					m_DetectedTargetCenterBoxes[5] = true;
				}
				else
					continue;
			}
			else
			{
				if (!m_DetectedTargetCenterBoxes[1])
				{
					m_AllTargetCenterBoxes[1] = TargetCenterDetection;
					m_DetectedTargetCenterBoxes[1] = true;
				}
				else
					continue;
			}
		}
		else if(TargetCenterDetection.y >= (int)(2*height/3))
		{
			if (TargetCenterDetection.x <= (int)(width/3))
			{
				if (!m_DetectedTargetCenterBoxes[8])
				{
					m_AllTargetCenterBoxes[8] = TargetCenterDetection;
					m_DetectedTargetCenterBoxes[8] = true;
				}
				else
					continue;
			}
			else if (TargetCenterDetection.x >= (int)(2*width/3))
			{
				if (!m_DetectedTargetCenterBoxes[7])
				{
					m_AllTargetCenterBoxes[7] = TargetCenterDetection;
					m_DetectedTargetCenterBoxes[7] = true;
				}
				else
					continue;
			}
			else
			{
				if (!m_DetectedTargetCenterBoxes[3])
				{
					m_AllTargetCenterBoxes[3] = TargetCenterDetection;
					m_DetectedTargetCenterBoxes[3] = true;
				}
				else
					continue;
			}
		}
		else
		{
			if (TargetCenterDetection.x <= (int)(width/5))
			{
				if (!m_DetectedTargetCenterBoxes[9])
				{
					m_AllTargetCenterBoxes[9] = TargetCenterDetection;
					m_DetectedTargetCenterBoxes[9] = true;
				}
				else
					continue;
			}
			else if ((TargetCenterDetection.x >= (int)(width/5))
				&& (TargetCenterDetection.x <= (int)(2*width/5)))
			{
				if (!m_DetectedTargetCenterBoxes[4])
				{
					m_AllTargetCenterBoxes[4] = TargetCenterDetection;
					m_DetectedTargetCenterBoxes[4] = true;
				}
				else
					continue;
			}
			else if ((TargetCenterDetection.x >= (int)(2*width/5))
				&& (TargetCenterDetection.x <= (int)(3*width/5)))
			{
				if (!m_DetectedTargetCenterBoxes[0])
				{
					m_AllTargetCenterBoxes[0] = TargetCenterDetection;
					m_DetectedTargetCenterBoxes[0] = true;
				}
				else
					continue;
			}
			else if ((TargetCenterDetection.x >= (int)(3*width/5))
				&& (TargetCenterDetection.x <= (int)(4*width/5)))
			{
				if (!m_DetectedTargetCenterBoxes[2])
				{
					m_AllTargetCenterBoxes[2] = TargetCenterDetection;
					m_DetectedTargetCenterBoxes[2] = true;
				}
				else
					continue;
			}
			else
			{
				if (!m_DetectedTargetCenterBoxes[6])
				{
					m_AllTargetCenterBoxes[6] = TargetCenterDetection;
					m_DetectedTargetCenterBoxes[6] = true;
				}
				else
					continue;
			}
		}
         // draw the circle center
         circle( img_display, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
         // draw the circle outline
         circle( img_display, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );
    }

	//ImageWrite("ObjectDetection.pgm", img_display);

	return 0;
}

int LocateMultipleFeatures(cv::Mat img, cv::Mat img_display, int width, int height)
{
	int lMaxIteration = 100;
	cv::Mat left_result,right_result;

	int left_result_cols =  img.cols - CFid_32BitsPerPixel.cols + 1;
	int left_result_rows = img.rows - CFid_32BitsPerPixel.rows + 1;
	
	int Right_result_cols;
	int Right_result_rows; 
	if(!CFid2.empty())
	{
		Right_result_cols =  img.cols - CFid2_32BitsPerPixel.cols + 1;
		Right_result_rows = img.rows - CFid2_32BitsPerPixel.rows + 1;   
	}

	double minVal, maxVal, threshold = 0.65; 

	cv::Rect edgebox;

	cv::Point matchLoc;
	cv::Point minLoc;
	cv::Point maxLoc;

	int match_found = false;

	//img.copyTo( img_display );

	 /// Create the result matrix (left and right)
	left_result.create( left_result_cols, left_result_rows, CV_32FC1 );
	right_result.create( Right_result_cols, Right_result_rows, CV_32FC1 );

	/// Do the Matching and Normalize
	//matchTemplate( img, CFid, left_result, CV_TM_SQDIFF );
	//normalize( left_result, left_result, 0, 1, NORM_MINMAX, -1, Mat() );
	matchTemplate( img, CFid_32BitsPerPixel, left_result, CV_TM_CCOEFF_NORMED );
	cv::threshold(left_result, left_result, 0.0, 1, CV_THRESH_TOZERO);
	//normalize( left_result, left_result, 0, 1, NORM_MINMAX, -1, Mat() );

	//GaussianBlur(left_result, left_result, cv::Size(5, 5), 2.0, 2.0);

	matchTemplate( img, CFid2_32BitsPerPixel, right_result, CV_TM_CCOEFF_NORMED );
	cv::threshold(right_result, right_result, 0.0, 1, CV_THRESH_TOZERO);

	//GaussianBlur(right_result, right_result, cv::Size(5, 5), 2.0, 2.0);

	//To show the matched result

	//cv::circle(left_result,cv::Point(640,480),50,CV_RGB(0,0,0),-1,8,0);

	#ifdef _DEBUG
	cv::namedWindow("matchresult1", CV_WINDOW_AUTOSIZE);
	cv::imshow("matchresult1",right_result);
	cv::namedWindow("matchresult2", CV_WINDOW_AUTOSIZE);
	cv::imshow("matchresult2",left_result);
	cv::waitKey(0);
	#endif

	int lTotalDetectedBoxes = 11;
	for (int i = 0; i < MAX_NUMBER_BOXES ; i++)
		m_DetectedTargetCenterBoxes[i] = false;
	cv::Point TargetCenterDetection;

	

	while (true)
	{
		if ((lTotalDetectedBoxes <= 0) || (lMaxIteration <= 0))
			break;
		/// Localizing the best match with minMaxLoc
		minMaxLoc( left_result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat() );
		
		/// For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all the other methods, the higher the better
		if (maxVal >= threshold)
		{
			lMaxIteration--;
			matchLoc = maxLoc;
			//matchLoc = minLoc;

			
			TargetCenterDetection.x = matchLoc.x + CFid_32BitsPerPixel.cols/2;
			TargetCenterDetection.y = matchLoc.y + CFid_32BitsPerPixel.rows/2;


			if(TargetCenterDetection.y < (int)(height/3))
			{
				if (TargetCenterDetection.x <= (int)(width/3))
				{
					if (!m_DetectedTargetCenterBoxes[10])
					{
						m_AllTargetCenterBoxes[10] = TargetCenterDetection;
						m_DetectedTargetCenterBoxes[10] = true;
						lTotalDetectedBoxes--;
					}
					else
					{
						//cv::floodFill(left_result, maxLoc, cv::Scalar(0), 0, cv::Scalar(0.1), cv::Scalar(1.));
						cv::circle(left_result,matchLoc,50,CV_RGB(0,0,0),-1,8,0);
						continue;
					}
				}
				else if (TargetCenterDetection.x >= (int)(2*width/3))
				{
					if (!m_DetectedTargetCenterBoxes[5])
					{
						m_AllTargetCenterBoxes[5] = TargetCenterDetection;
						m_DetectedTargetCenterBoxes[5] = true;
						lTotalDetectedBoxes--;
					}
					else
					{
						//cv::floodFill(left_result, maxLoc, cv::Scalar(0), 0, cv::Scalar(0.1), cv::Scalar(1.));
						cv::circle(left_result,matchLoc,50,CV_RGB(0,0,0),-1,8,0);
						continue;
					}
				}
				else
				{
					if (!m_DetectedTargetCenterBoxes[1])
					{
						m_AllTargetCenterBoxes[1] = TargetCenterDetection;
						m_DetectedTargetCenterBoxes[1] = true;
						lTotalDetectedBoxes--;
					}
					else
					{
						//cv::floodFill(left_result, maxLoc, cv::Scalar(0), 0, cv::Scalar(0.1), cv::Scalar(1.));
						cv::circle(left_result,matchLoc,50,CV_RGB(0,0,0),-1,8,0);
						continue;
					}
				}
			}
			else if(TargetCenterDetection.y >= (int)(2*height/3))
			{
				if (TargetCenterDetection.x <= (int)(width/3))
				{
					if (!m_DetectedTargetCenterBoxes[8])
					{
						m_AllTargetCenterBoxes[8] = TargetCenterDetection;
						m_DetectedTargetCenterBoxes[8] = true;
						lTotalDetectedBoxes--;
					}
					else
					{
						//cv::floodFill(left_result, maxLoc, cv::Scalar(0), 0, cv::Scalar(0.1), cv::Scalar(1.));
						cv::circle(left_result,matchLoc,50,CV_RGB(0,0,0),-1,8,0);
						continue;
					}
				}
				else if (TargetCenterDetection.x >= (int)(2*width/3))
				{
					if (!m_DetectedTargetCenterBoxes[7])
					{
						m_AllTargetCenterBoxes[7] = TargetCenterDetection;
						m_DetectedTargetCenterBoxes[7] = true;
						lTotalDetectedBoxes--;
					}
					else
					{
						//cv::floodFill(left_result, maxLoc, cv::Scalar(0), 0, cv::Scalar(0.1), cv::Scalar(1.));
						cv::circle(left_result,matchLoc,50,CV_RGB(0,0,0),-1,8,0);
						continue;
					}
				}
				else
				{
					if (!m_DetectedTargetCenterBoxes[3])
					{
						m_AllTargetCenterBoxes[3] = TargetCenterDetection;
						m_DetectedTargetCenterBoxes[3] = true;
						lTotalDetectedBoxes--;
					}
					else
					{
						//cv::floodFill(left_result, maxLoc, cv::Scalar(0), 0, cv::Scalar(0.1), cv::Scalar(1.));
						cv::circle(left_result,matchLoc,50,CV_RGB(0,0,0),-1,8,0);
						continue;
					}
				}
			}
			else
			{
				if (TargetCenterDetection.x <= (int)(width/5))
				{
					if (!m_DetectedTargetCenterBoxes[9])
					{
						m_AllTargetCenterBoxes[9] = TargetCenterDetection;
						m_DetectedTargetCenterBoxes[9] = true;
						lTotalDetectedBoxes--;
					}
					else
					{
						//cv::floodFill(left_result, maxLoc, cv::Scalar(0), 0, cv::Scalar(0.1), cv::Scalar(1.));
						cv::circle(left_result,matchLoc,50,CV_RGB(0,0,0),-1,8,0);
						continue;
					}
				}
				else if ((TargetCenterDetection.x >= (int)(width/5))
					&& (TargetCenterDetection.x <= (int)(2*width/5)))
				{
					if (!m_DetectedTargetCenterBoxes[4])
					{
						m_AllTargetCenterBoxes[4] = TargetCenterDetection;
						m_DetectedTargetCenterBoxes[4] = true;
						lTotalDetectedBoxes--;
					}
					else
					{
						//cv::floodFill(left_result, maxLoc, cv::Scalar(0), 0, cv::Scalar(0.1), cv::Scalar(1.));
						cv::circle(left_result,matchLoc,50,CV_RGB(0,0,0),-1,8,0);
						continue;
					}
				}
				else if ((TargetCenterDetection.x >= (int)(2*width/5))
					&& (TargetCenterDetection.x <= (int)(3*width/5)))
				{
					if (!m_DetectedTargetCenterBoxes[0])
					{
						m_AllTargetCenterBoxes[0] = TargetCenterDetection;
						m_DetectedTargetCenterBoxes[0] = true;
						lTotalDetectedBoxes--;
					}
					else
					{
						//cv::floodFill(left_result, maxLoc, cv::Scalar(0), 0, cv::Scalar(0.1), cv::Scalar(1.));
						cv::circle(left_result,matchLoc,50,CV_RGB(0,0,0),-1,8,0);
						continue;
					}
				}
				else if ((TargetCenterDetection.x >= (int)(3*width/5))
					&& (TargetCenterDetection.x <= (int)(4*width/5)))
				{
					if (!m_DetectedTargetCenterBoxes[2])
					{
						m_AllTargetCenterBoxes[2] = TargetCenterDetection;
						m_DetectedTargetCenterBoxes[2] = true;
						lTotalDetectedBoxes--;
					}
					else
					{
						//cv::floodFill(left_result, maxLoc, cv::Scalar(0), 0, cv::Scalar(0.1), cv::Scalar(1.));
						cv::circle(left_result,matchLoc,50,CV_RGB(0,0,0),-1,8,0);
						continue;
					}
				}
				else
				{
					if (!m_DetectedTargetCenterBoxes[6])
					{
						m_AllTargetCenterBoxes[6] = TargetCenterDetection;
						m_DetectedTargetCenterBoxes[6] = true;
						lTotalDetectedBoxes--;
					}
					else
					{
						//cv::floodFill(left_result, maxLoc, cv::Scalar(0), 0, cv::Scalar(0.1), cv::Scalar(1.));
						cv::circle(left_result,matchLoc,50,CV_RGB(0,0,0),-1,8,0);
						continue;
					}
				}
			}

			if(	(matchLoc.x < width-CFid.cols) &&
				(matchLoc.y < height-CFid.rows) &&
				(matchLoc.x > 0) &&
				(matchLoc.y > 0) )
			{
				rectangle( img_display, matchLoc, cv::Point( matchLoc.x + CFid_32BitsPerPixel.cols , matchLoc.y + CFid_32BitsPerPixel.rows ), cv::Scalar::all(0), 1, 8, 0 ); 

				line( img_display, cv::Point(TargetCenterDetection.x+5,TargetCenterDetection.y),cv::Point(TargetCenterDetection.x-5,TargetCenterDetection.y),(255,255,255), 1, 8, 0);
				line( img_display, cv::Point(TargetCenterDetection.x,TargetCenterDetection.y+5),cv::Point(TargetCenterDetection.x,TargetCenterDetection.y-5),(255,255,255), 1, 8, 0);
				//cv::floodFill(left_result, maxLoc, cv::Scalar(0), 0, cv::Scalar(0.1), cv::Scalar(1.));
				cv::circle(left_result,matchLoc,50,CV_RGB(0,0,0),-1,8,0);
				match_found = true;
			}
			//else
			//	match_found = false;
		}
		else
		{
			break;
		}

#ifdef _DEBUG
			cv::imshow("matchresult2",left_result);
			cv::waitKey(0);
#endif
	}

	while (true)
	{
		if ((lTotalDetectedBoxes <= 0) || (lMaxIteration <= 0))
			break;
		/// Localizing the best match with minMaxLoc
		minMaxLoc( right_result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat() );
		
		/// For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all the other methods, the higher the better
		if (maxVal >= threshold)
		{
			lMaxIteration--;
			matchLoc = maxLoc;
			//matchLoc = minLoc;

			TargetCenterDetection.x = matchLoc.x + CFid2_32BitsPerPixel.cols/2;
			TargetCenterDetection.y = matchLoc.y + CFid2_32BitsPerPixel.rows/2;


			if(TargetCenterDetection.y < (int)(height/3))
			{
				if (TargetCenterDetection.x <= (int)(width/3))
				{
					if (!m_DetectedTargetCenterBoxes[10])
					{
						m_AllTargetCenterBoxes[10] = TargetCenterDetection;
						m_DetectedTargetCenterBoxes[10] = true;
						lTotalDetectedBoxes--;
					}
					else
					{
						//cv::floodFill(right_result, maxLoc, cv::Scalar(0), 0, cv::Scalar(0.1), cv::Scalar(1.));
						cv::circle(right_result,matchLoc,50,CV_RGB(0,0,0),-1,8,0);
						continue;
					}
				}
				else if (TargetCenterDetection.x >= (int)(2*width/3))
				{
					if (!m_DetectedTargetCenterBoxes[5])
					{
						m_AllTargetCenterBoxes[5] = TargetCenterDetection;
						m_DetectedTargetCenterBoxes[5] = true;
						lTotalDetectedBoxes--;
					}
					else
					{
						//cv::floodFill(right_result, maxLoc, cv::Scalar(0),0, cv::Scalar(0.1), cv::Scalar(1.));
						cv::circle(right_result,matchLoc,50,CV_RGB(0,0,0),-1,8,0);
						continue;
					}
				}
				else
				{
					if (!m_DetectedTargetCenterBoxes[1])
					{
						m_AllTargetCenterBoxes[1] = TargetCenterDetection;
						m_DetectedTargetCenterBoxes[1] = true;
						lTotalDetectedBoxes--;
					}
					else
					{
						//cv::floodFill(right_result, maxLoc, cv::Scalar(0), 0, cv::Scalar(0.1), cv::Scalar(1.));
						cv::circle(right_result,matchLoc,50,CV_RGB(0,0,0),-1,8,0);
						continue;
					}
				}
			}
			else if(TargetCenterDetection.y >= (int)(2*height/3))
			{
				if (TargetCenterDetection.x <= (int)(width/3))
				{
					if (!m_DetectedTargetCenterBoxes[8])
					{
						m_AllTargetCenterBoxes[8] = TargetCenterDetection;
						m_DetectedTargetCenterBoxes[8] = true;
						lTotalDetectedBoxes--;
					}
					else
					{
						//cv::floodFill(right_result, maxLoc, cv::Scalar(0), 0, cv::Scalar(0.1), cv::Scalar(1.));
						cv::circle(right_result,matchLoc,50,CV_RGB(0,0,0),-1,8,0);
						continue;
					}
				}
				else if (TargetCenterDetection.x >= (int)(2*width/3))
				{
					if (!m_DetectedTargetCenterBoxes[7])
					{
						m_AllTargetCenterBoxes[7] = TargetCenterDetection;
						m_DetectedTargetCenterBoxes[7] = true;
						lTotalDetectedBoxes--;
					}
					else
					{
						//cv::floodFill(right_result, maxLoc, cv::Scalar(0), 0, cv::Scalar(0.1), cv::Scalar(1.));
						cv::circle(right_result,matchLoc,50,CV_RGB(0,0,0),-1,8,0);
						continue;
					}
				}
				else
				{
					if (!m_DetectedTargetCenterBoxes[3])
					{
						m_AllTargetCenterBoxes[3] = TargetCenterDetection;
						m_DetectedTargetCenterBoxes[3] = true;
						lTotalDetectedBoxes--;
					}
					else
					{
						//cv::floodFill(right_result, maxLoc, cv::Scalar(0), 0, cv::Scalar(0.1), cv::Scalar(1.));
						cv::circle(right_result,matchLoc,50,CV_RGB(0,0,0),-1,8,0);
						continue;
					}
				}
			}
			else
			{
				if (TargetCenterDetection.x <= (int)(width/5))
				{
					if (!m_DetectedTargetCenterBoxes[9])
					{
						m_AllTargetCenterBoxes[9] = TargetCenterDetection;
						m_DetectedTargetCenterBoxes[9] = true;
						lTotalDetectedBoxes--;
					}
					else
					{
						//cv::floodFill(right_result, maxLoc, cv::Scalar(0), 0, cv::Scalar(0.1), cv::Scalar(1.));
						cv::circle(right_result,matchLoc,50,CV_RGB(0,0,0),-1,8,0);
						continue;
					}
				}
				else if ((TargetCenterDetection.x >= (int)(width/5))
					&& (TargetCenterDetection.x <= (int)(2*width/5)))
				{
					if (!m_DetectedTargetCenterBoxes[4])
					{
						m_AllTargetCenterBoxes[4] = TargetCenterDetection;
						m_DetectedTargetCenterBoxes[4] = true;
						lTotalDetectedBoxes--;
					}
					else
					{
						//cv::floodFill(right_result, maxLoc, cv::Scalar(0), 0, cv::Scalar(0.1), cv::Scalar(1.));
						cv::circle(right_result,matchLoc,50,CV_RGB(0,0,0),-1,8,0);
						continue;
					}
				}
				else if ((TargetCenterDetection.x >= (int)(2*width/5))
					&& (TargetCenterDetection.x <= (int)(3*width/5)))
				{
					if (!m_DetectedTargetCenterBoxes[0])
					{
						m_AllTargetCenterBoxes[0] = TargetCenterDetection;
						m_DetectedTargetCenterBoxes[0] = true;
						lTotalDetectedBoxes--;
					}
					else
					{
						//cv::floodFill(right_result, maxLoc, cv::Scalar(0), 0, cv::Scalar(0.1), cv::Scalar(1.));
						cv::circle(right_result,matchLoc,50,CV_RGB(0,0,0),-1,8,0);
						continue;
					}
				}
				else if ((TargetCenterDetection.x >= (int)(3*width/5))
					&& (TargetCenterDetection.x <= (int)(4*width/5)))
				{
					if (!m_DetectedTargetCenterBoxes[2])
					{
						m_AllTargetCenterBoxes[2] = TargetCenterDetection;
						m_DetectedTargetCenterBoxes[2] = true;
						lTotalDetectedBoxes--;
					}
					else
					{
						//cv::floodFill(right_result, maxLoc, cv::Scalar(0), 0, cv::Scalar(0.1), cv::Scalar(1.));
						cv::circle(right_result,matchLoc,50,CV_RGB(0,0,0),-1,8,0);
						continue;
					}
				}
				else
				{
					if (!m_DetectedTargetCenterBoxes[6])
					{
						m_AllTargetCenterBoxes[6] = TargetCenterDetection;
						m_DetectedTargetCenterBoxes[6] = true;
						lTotalDetectedBoxes--;
					}
					else
					{
						//cv::floodFill(right_result, maxLoc, cv::Scalar(0), 0, cv::Scalar(0.1), cv::Scalar(1.));
						cv::circle(right_result,matchLoc,50,CV_RGB(0,0,0),-1,8,0);
						continue;
					}
				}
			}


			if(	(matchLoc.x < width-CFid2.cols) &&
				(matchLoc.y < height-CFid2.rows) &&
				(matchLoc.x > 0) &&
				(matchLoc.y > 0) )
			{
				rectangle( img_display, matchLoc, cv::Point( matchLoc.x + CFid2_32BitsPerPixel.cols , matchLoc.y + CFid2_32BitsPerPixel.rows ), cv::Scalar::all(0), 1, 8, 0 ); 

				line( img_display, cv::Point(TargetCenterDetection.x+5,TargetCenterDetection.y),cv::Point(TargetCenterDetection.x-5,TargetCenterDetection.y),(255,255,255), 1, 8, 0);
				line( img_display, cv::Point(TargetCenterDetection.x,TargetCenterDetection.y+5),cv::Point(TargetCenterDetection.x,TargetCenterDetection.y-5),(255,255,255), 1, 8, 0);
				//cv::floodFill(right_result, maxLoc, cv::Scalar(0), 0, cv::Scalar(0.1), cv::Scalar(1.));
				cv::circle(right_result,matchLoc,50,CV_RGB(0,0,0),-1,8,0);
				match_found = true;
			}
			//else
			//	match_found = false;
		}
		else
		{
			break;
		}
#ifdef _DEBUG
		cv::imshow("matchresult2",left_result);
		cv::imshow("matchresult1",right_result);
		cv::waitKey(0);
#endif
	}


	return match_found;
}

bool InitializeROIs(string filePath)
{
	int symetric_offset = 0;
	int box_col = 0;		
	int xoffset,yoffset;
	cv::FileStorage ConfigurationXML;

	ConfigurationXML.open(filePath + "\\Quad2.xml", cv::FileStorage::READ);
	if (!ConfigurationXML.isOpened())
	{
		return false;
	}


	cv::FileNode Boxes_Node = ConfigurationXML["QUAD_SFR_11Boxes"];

	for (int boxNumber = 0; boxNumber < MAX_NUMBER_BOXES; boxNumber++)
	{
		std::string BoxValue("Box");
		BoxValue += std::to_string(static_cast<long long>(boxNumber));
				
		cv::FileNode BoxValue_Node = Boxes_Node[BoxValue];
		Boxes[boxNumber].SB_Size = (int)BoxValue_Node["SB_Size"];

				
		cv::FileNode SFRBox_ROIUsed_Node = BoxValue_Node["ROIUsed"];	
		// Apply "rules" based on rows columns or both. Set defautls for all boxes first and then modify based on rules
		Boxes[boxNumber].roi_used[N] = static_cast<bool>((int)SFRBox_ROIUsed_Node["North"]);
		Boxes[boxNumber].roi_used[S] = static_cast<bool>((int)SFRBox_ROIUsed_Node["South"]);
		Boxes[boxNumber].roi_used[E] = static_cast<bool>((int)SFRBox_ROIUsed_Node["East"]);
		Boxes[boxNumber].roi_used[W] = static_cast<bool>((int)SFRBox_ROIUsed_Node["West"]);

		Boxes[boxNumber].box_ctr.y =  m_AllTargetCenterBoxes[boxNumber].y;
		Boxes[boxNumber].box_ctr.x = m_AllTargetCenterBoxes[boxNumber].x;


		//cv::FileNode SFRBox_BoxCtr_Node = BoxValue_Node["BoxCtr"];
		//Boxes[boxNumber].box_ctr.y = (int)SFRBox_BoxCtr_Node["Y"];
		//Boxes[boxNumber].box_ctr.x = (int)SFRBox_BoxCtr_Node["X"];

		cv::FileNode AllSFRBox_Value_Nodes = BoxValue_Node["SRFBox"];
		std::string SFRBox_Value;
	
		for(int i = 0;i<ROI_PER_SLANTBOX;i++)
		{
			switch (i)
			{
			case 0: SFRBox_Value = "Zero"; break;
			case 1: SFRBox_Value = "One"; break;
			case 2: SFRBox_Value = "Two"; break;
			case 3: SFRBox_Value = "Three"; break;
			default: break;
			}

			cv::FileNode SFRBox_Value_Node = AllSFRBox_Value_Nodes[SFRBox_Value];
			Boxes[boxNumber].SFRBox[i].isHorz = static_cast<bool>((int)SFRBox_Value_Node["isHorz"]);
					
			cv::FileNode SFRBox_ROI_Node = SFRBox_Value_Node["ROI"];
			Boxes[boxNumber].SFRBox[i].roi.width = (int)SFRBox_ROI_Node["Width"];
			Boxes[boxNumber].SFRBox[i].roi.height = (int)SFRBox_ROI_Node["Height"];
			Boxes[boxNumber].SFRBox[i].x_Delta = (int)SFRBox_ROI_Node["X_Delta"];
			Boxes[boxNumber].SFRBox[i].y_Delta = (int)SFRBox_ROI_Node["Y_Delta"];
					

			switch (i) 
			{
			case 0 : //North
				yoffset = -((int)Boxes[boxNumber].SB_Size/2)-(Boxes[boxNumber].SFRBox[i].roi.height/2);
				xoffset = -((int)Boxes[boxNumber].SFRBox[i].roi.width/2);	
				break;
			case 2 : //south
				yoffset = ((int)Boxes[boxNumber].SB_Size/2)-(Boxes[boxNumber].SFRBox[i].roi.height/2);
				xoffset = -((int)Boxes[boxNumber].SFRBox[i].roi.width/2);
				break;
			case 1 : // EAST
				yoffset = -((int)Boxes[boxNumber].SFRBox[i].roi.height/2);
				xoffset = ((int)Boxes[boxNumber].SB_Size/2)-(Boxes[boxNumber].SFRBox[i].roi.width/2);
				break;
			case 3 : //West
				yoffset = -((int)Boxes[boxNumber].SFRBox[i].roi.height/2);
				xoffset = -((int)Boxes[boxNumber].SB_Size/2)-(Boxes[boxNumber].SFRBox[i].roi.width/2);
				break;
			default :
				break;
			}
			Boxes[boxNumber].SFRBox[i].roi.x = Boxes[boxNumber].box_ctr.x+ xoffset + Boxes[boxNumber].SFRBox[i].x_Delta;
			Boxes[boxNumber].SFRBox[i].roi.y = Boxes[boxNumber].box_ctr.y+ yoffset + Boxes[boxNumber].SFRBox[i].y_Delta;
			
			if ( Boxes[boxNumber].SFRBox[i].roi.x < 0)
				Boxes[boxNumber].SFRBox[i].roi.x = 0;
			if (Boxes[boxNumber].SFRBox[i].roi.y < 0)
				Boxes[boxNumber].SFRBox[i].roi.y = 0;
			Boxes[boxNumber].SFRBox[i].DesiredFreq = (double)SFRBox_Value_Node["DesiredFreq"];
			Boxes[boxNumber].SFRBox[i].SFRLimit = (double)SFRBox_Value_Node["SFRLimit"]; 
			Boxes[boxNumber].SFRBox[i].sideused = static_cast<ROIUSAGE>((int)SFRBox_Value_Node["SideUsed"]);
			Boxes[boxNumber].SFRBox[i].inRange = static_cast<bool>((int)SFRBox_Value_Node["inRange"]);
			Boxes[boxNumber].SFRBox[i].SFRValue = (double)SFRBox_Value_Node["SFRValue"];
					
		}
	}
	ConfigurationXML.release();
	return true;
}

double CalcSFR_Hi(wchar_t* ptr_img, Roi_for_SFR & SFRBox, int width, int height, int grey_Level, double MM_PER_PIX, const flags_t modeflags, const int redchan)
{
	// added for SFR
	int i;
	double *farea;
	double slope, scale;
	int size_x, size_y;
	int err, bin_len;
	int center;
	int numcycles=0;
	double *Freq=NULL;
	double *disp=NULL;
	double off, R2;
	
	double ScoreSFR = 0.0; 
	double Limit_Cymm = 0.0;
	
	int g_version = 0;	// flag to use different alg variation; zero = Forward Difference, no peak.
	g_version |= 2;		// Shift hamming window to center on PEAK
	g_version |= ESFFILE;		// Utilize central difference on averaged LSF
	
	wchar_t *img = (wchar_t *)malloc(height*width*sizeof(wchar_t));
	wchar_t *imgOrg = (wchar_t *)malloc(height*width*sizeof(wchar_t));
	memcpy(img, ptr_img, height*width*sizeof(wchar_t));
	memcpy(imgOrg, ptr_img, height*width*sizeof(wchar_t));

	Limit_Cymm = SFRBox.DesiredFreq;
	
	size_x = SFRBox.roi.width;
	size_y = SFRBox.roi.height;

	farea = (double *)malloc(size_y*size_x*sizeof(double));
	
	/* calculate the sfr on this area */
	//  Original call err = sfrProc(&Freq, &disp, &bin_len, farea, (unsigned short)size_x, &size_y, &slope, &numcycles,&center,&off, &R2, g_version, 0, g_userangle);
	
	int index= 0;
	// Vert SFR with Mosaic need rows removed to eliminate RED and maintain resolution across SFR Edge.
	// Due using the whole array (starting at row 0) with top 3 rows black the red pixels are EVEN rows and ODD columns in original image
	// so depending on ROI coordinates need start on correct row or column to skip red pixels
	
	int odd_x_offset = redchan & 1 ? 1 : 0;
	int odd_y_offset = redchan & 2 ? 1 : 0;
	if (!(SFRBox.roi.x % 2)) { odd_x_offset = 1-odd_x_offset; } 	
	if (!(SFRBox.roi.y % 2)) { odd_y_offset = 1-odd_y_offset; }
#ifdef _DEBUG
	printf("ROI Location - (%d,%d)\n",SFRBox.roi.x, SFRBox.roi.y);
#endif
	std::stringstream ss;
	ss << m_FrameCounter;
	std::string lFileName = "ROI_Binary_"+ ss.str(); // + "_AfterRemovingRedPixels.pgm";
	m_FrameCounter++;
	int lRow = 0;
	int lCol = 0;

	if (modeflags.Demosaic == 0)
	{
		if(!SFRBox.isHorz)
		{
			lRow = 0;
			lCol = 0;
			if (((modeflags.WriteImageLevel >3)&& (lKey == NO_KEY_PRESSED)) || (modeflags.num_frames && (modeflags.WriteImageLevel == 3)))
			{
				std::string lFileNameBefore = lFileName + "_BeforeRemovingRedPixels.pgm";
				cv::Mat roi_file_Before(size_y, size_x,CV_16U,cv::Scalar::all(0));
				// Vert SFR Calc - read rows col by col and convert to ratiometric (0 - 1) array.
				for (int row=SFRBox.roi.y+odd_y_offset;row< SFRBox.roi.y+size_y + odd_y_offset;row++)
				{
					lCol = 0;
					for (int col=SFRBox.roi.x+ odd_x_offset;col< SFRBox.roi.x+size_x + odd_x_offset;col++)
					{
						roi_file_Before.at<ushort>(lRow,lCol) = (ushort)img[width*row + col];
						lCol++;
					}
					lRow++;
				}

				ImageWrite(lFileNameBefore,roi_file_Before);
				roi_file_Before.release();

			}
			lRow = 0;
			lCol = 0;
			std::string lFileNameAfter = lFileName + "_AfterRemovingRedPixels.pgm";

			cv::Mat roi_file2(ceil(float(size_y)/2), size_x,CV_16U,cv::Scalar::all(0));
			// Vert SFR Calc - read rows col by col and convert to ratiometric (0 - 1) array.
			for (int row=SFRBox.roi.y+odd_y_offset;row< SFRBox.roi.y+size_y + odd_y_offset;row=row+2)
			{
				lCol = 0;
				for (int col=SFRBox.roi.x + odd_x_offset;col< SFRBox.roi.x+size_x +odd_x_offset;col++)
				{
					roi_file2.at<ushort>(lRow,lCol) = (ushort)img[width*row + col];
					lCol++;
					farea[index] = (double)( img[width*row + col])/(double)grey_Level;
					index++;
				}
				lRow++;
			}
			if (((modeflags.WriteImageLevel >3)&& (lKey == NO_KEY_PRESSED)) || (modeflags.num_frames && (modeflags.WriteImageLevel == 3)))
				ImageWrite(lFileNameAfter,roi_file2);
			roi_file2.release();
		}
		else
		{
			lRow = 0;
			lCol = 0;
			if (((modeflags.WriteImageLevel >3)&& (lKey == NO_KEY_PRESSED)) || (modeflags.num_frames && (modeflags.WriteImageLevel == 3)))
			{
				std::string lFileNameBefore = lFileName + "_BeforeRemovingRedPixels.pgm";
				cv::Mat roi_file_Before(size_y, size_x,CV_16U,cv::Scalar::all(0));
				// Vert SFR Calc - read rows col by col and convert to ratiometric (0 - 1) array.
				for (int col=SFRBox.roi.x+odd_x_offset;col< SFRBox.roi.x+size_x+odd_x_offset;col++)
				{
					lRow = 0;
					for (int row=SFRBox.roi.y + odd_y_offset;row< SFRBox.roi.y+size_y+odd_y_offset;row++)
					{
						roi_file_Before.at<ushort>(lRow,lCol) = (ushort)img[width*row + col];
						lRow++;
					}
					lCol++;
				}

				ImageWrite(lFileNameBefore,roi_file_Before);
				roi_file_Before.release();
			}

			lRow = 0;
			lCol = 0;
			std::string lFileNameAfter = lFileName + "_AfterRemovingRedPixels.pgm";
			// Horz SFR Calc - read cols row by row an convert
			cv::Mat roi_file2(size_y,ceil(float(size_x)/2),CV_16U,cv::Scalar::all(0));
			for (int col=SFRBox.roi.x+odd_x_offset;col< SFRBox.roi.x+size_x+odd_x_offset;col=col+2)
			{
				lRow = 0;
				for (int row=SFRBox.roi.y+odd_y_offset;row< SFRBox.roi.y+size_y+odd_y_offset;row++)
				{
					roi_file2.at<ushort>(lRow,lCol) = (ushort)img[width*row + col];
					lRow++;
		

			//for (int col=0;col< size_x;col=col+2)
			//{
			//	for (int row=0;row< size_y;row++)
			//	{
					farea[index] = (double)( img[width*row + col])/(double)grey_Level;
					index++;
				}
				lCol++;
			}
			// now that copy is complete must transpose x and y sizes to match the "rotated" image
			size_x = SFRBox.roi.height;
			size_y = SFRBox.roi.width;
			if (((modeflags.WriteImageLevel >3)&& (lKey == NO_KEY_PRESSED)) || (modeflags.num_frames && (modeflags.WriteImageLevel == 3)))
				ImageWrite(lFileNameAfter,roi_file2);
			roi_file2.release();
	  
		}
		// SFRPROC always passed a "vert" edge so rows are always reduced.
		// TODO: check that /2 rounding errors will not throw things off.
		size_y = size_y/2;
	}
	else if (modeflags.Demosaic == 1)
	{
		if (((modeflags.WriteImageLevel >3)&& (lKey == NO_KEY_PRESSED)) || (modeflags.num_frames && (modeflags.WriteImageLevel == 3)))
		{
			std::string lFileNameBefore = lFileName + "_BeforeDemosaic.pgm";
			cv::Mat roi_file_Before(size_y, size_x,CV_16U,cv::Scalar::all(0));
			// Vert SFR Calc - read rows col by col and convert to ratiometric (0 - 1) array.
			for (int row=SFRBox.roi.y+odd_y_offset;row< SFRBox.roi.y+size_y;row++)
			{
				lCol = 0;
				for (int col=SFRBox.roi.x;col< SFRBox.roi.x+size_x;col++)
				{
					roi_file_Before.at<ushort>(lRow,lCol) = (ushort)img[width*row + col];
					lCol++;
				}
				lRow++;
			}

			ImageWrite(lFileNameBefore,roi_file_Before);
			roi_file_Before.release();

		}
		lRow = 0;
		lCol = 0;
		// Demosaic
		double DemosaicMask1[5][5] = 
		{
			{0,0,0,0,0},
			{0,0,0.25,0,0},
			{0,0.25,0,0.25,0},
			{0,0,0.25,0,0},
			{0,0,0,0,0},
		};
		double DemosaicMask2[5][5] = 
		{
			{0,0,-0.25,0,0},
			{0,0,0,0,0},
			{-0.25,0,1,0,-0.25},
			{0,0,0,0,0},
			{0,0,-0.25,0,0},
		};
		for (int row=SFRBox.roi.y+odd_y_offset;row< SFRBox.roi.y+size_y;row += 2)
		{
			for (int col=SFRBox.roi.x+odd_x_offset;col< SFRBox.roi.x+size_x;col+=2)
			{
				double Mask1 = 0.0;
				for (int mRow = 0; mRow< 5; mRow++)
				{
					for (int mCol = 0; mCol< 5; mCol++)
					{
						//cout << "Imag[" << row-2+mRow << "][" << col-2+mCol << "]="<< imgOrg[width*(row-2+mRow) + (col-2+mCol)] << endl;
						Mask1 += imgOrg[width*(row-2+mRow) + (col-2+mCol)]*DemosaicMask1[mRow][mCol];
					}
				}
				double Mask2 = 0.0;
				for (int mRow = 0; mRow< 5; mRow++)
				{
					for (int mCol = 0; mCol< 5; mCol++)
					{
						Mask2 += imgOrg[width*(row-2+mRow) + (col-2+mCol)]*DemosaicMask2[mRow][mCol];
					}
				}
				img[width*row + col] = Mask1 + Mask2;
			}
		}
		if (((modeflags.WriteImageLevel >3)&& (lKey == NO_KEY_PRESSED)) || (modeflags.num_frames && (modeflags.WriteImageLevel == 3)))
		{
			std::string lFileNameBefore = lFileName + "_AfterDemosaic.pgm";
			cv::Mat roi_file_Before(size_y, size_x,CV_16U,cv::Scalar::all(0));
			// Vert SFR Calc - read rows col by col and convert to ratiometric (0 - 1) array.
			for (int row=SFRBox.roi.y+odd_y_offset;row< SFRBox.roi.y+size_y;row++)
			{
				lCol = 0;
				for (int col=SFRBox.roi.x+odd_x_offset;col< SFRBox.roi.x+size_x;col++)
				{
					roi_file_Before.at<ushort>(lRow,lCol) = (ushort)img[width*row + col];
					lCol++;
				}
				lRow++;
			}

			ImageWrite(lFileNameBefore,roi_file_Before);
			roi_file_Before.release();

		}

		for (int row=SFRBox.roi.y+odd_y_offset;row< SFRBox.roi.y+size_y;row++)
		{
			for (int col=SFRBox.roi.x+odd_x_offset;col< SFRBox.roi.x+size_x;col++)
			{
				farea[index] = (double)( img[width*row + col])/(double)grey_Level;
				index++;
			}
		}

	}
	
	err = sfrProc(&Freq, &disp, &bin_len, farea, (unsigned short)size_x, &size_y, &slope, &numcycles,&center,&off, &R2, g_version, 0, 0, modeflags.EdgeFitOrder);
	
	// Apply spatial frequency dependent correction from discrete differentiation
	if (!err)
	{
		double f, sfr, last_f=0, last_sfr=0;
		double freq, fd_scale;
		bool score_complete = false;
		
		scale = 1/MM_PER_PIX; // Pix per mm 
		
		for( i=0; i<bin_len/2; i++) 
		{
			freq = M_PI*Freq[i];
			
			// Version is always "0" but left over in case needed.
			/* [-1 0 0 0 1]  freq /= 1.0; */
			if (g_version & 4) { freq /= 2.0; }		/* [-1 0 1] */
			else { freq /= 4.0; }					/* [-1 1] */
			// Address division by zero when freq == 0.0
			if (freq == 0.0) {
				fd_scale = 1.0;
			} else { 
				fd_scale = freq / sin(freq); 
			}
			// Convert normalized frequency to lp/mm
			/*
				Version below corrects for higher angled edges that distorts sampling spacing
				Assumes that we are decimating rows/columns by two so we need to adjust the
				slope by a factor of two (lower, always).  Imatest does a correction for high
				angled edges, and I'm guessing this is something like it.  (Matt Warmuth, 26.Oct.2015) 
			*/
			if (modeflags.EdgeAngleCorr) { f = Freq[i]*scale/cos(atan(slope/2)); }
			else  { f = Freq[i]*scale; }

			// Apply correction to SFR value
			sfr = disp[i]*fd_scale;

			//  look at scaled values to interpolate the limit value
			//  This is a linear iterpolation between the two points in the Freq/SFR arrays that
			//  are above and below the limit value that we want to check.

			if( (f < Limit_Cymm) && !score_complete)
			{
				last_f = f;
				last_sfr= sfr;
			}
			else if( (f >= Limit_Cymm)  && !score_complete)
			{
				ScoreSFR = last_sfr + (sfr -last_sfr )*((last_f-Limit_Cymm)/(last_f-f));
				score_complete = true;
			}
		} // end for(i)
  } // end if(!err)

  // it is possible for the computation of the SFR frequencies to produce Indeterminate results (shows as -1.#IND) which
  // doesn't flag a bad number in the comparisons.
  // Check for this here and set value to 0.0 which will fail tests.
 
	if( !_finite(ScoreSFR) ) { ScoreSFR = 0.0; }
  
	// Freq and disp get allocated inside sfrproc but never free'd might not be an issue.
	// Not sure this is needed.
	free(Freq);
	free(disp);
	free(farea);
	//  roi_file.release();
	free(img);
	free(imgOrg);
	//img.release();

	if(!err)
	{
		SFRBox.SFRValue = ScoreSFR;
		printf("SFRscore = %f\n",ScoreSFR);
		return ScoreSFR;
	}
	else
	{
		// any value over 1 is an error. 5 is used just as an example. 
		// It can be replaced by "SFRBox.SFRValue = (double)err;" if needed for debuging
		SFRBox.SFRValue = err; 
		return (double)err;
	}
} // End for CalcSFR_Hi()

void InitializeDetection()
{
	for ( int i = 0 ; i < MAX_NUMBER_BOXES; i++)
	{
		m_AllTargetCenterBoxes[i].x = 0;
		m_AllTargetCenterBoxes[i].y = 0;
		m_DetectedTargetCenterBoxes[i] = false;
	}
}

void OverlayROIs(UINT8* DisplayImg, int width, int height, SFRPlus & SB)
{
	cv::Mat img_display(height,width,CV_8U,DisplayImg);	

	char ScoreSTR[128];
	int i;
	cv::Size textSize;
	int baseline;
	cv::Point s,e,c,t;

	for(i=0;i<ROI_PER_SLANTBOX;i++)
	{
		if(SB.SFRBox[i].inRange)
		{
			s.x=SB.SFRBox[i].roi.x;
			s.y=SB.SFRBox[i].roi.y;
			e.x =s.x+SB.SFRBox[i].roi.width;
			e.y =s.y+SB.SFRBox[i].roi.height;
			c.x = s.x-((s.x-e.x)/2);
			c.y = s.y-((s.y-e.y)/2);

			rectangle( img_display, s , e, (128,128,128), 1, 8, 0 ); 
			sprintf(ScoreSTR,"%4.2f",ceilf(SB.SFRBox[i].SFRValue * 100) / 100);

			textSize = getTextSize(ScoreSTR,cv::FONT_HERSHEY_PLAIN,0.75,1,&baseline);

			t.x = c.x-(textSize.width/2);
			t.y = c.y+(textSize.height/2);

			rectangle(img_display,t,cv::Point(t.x+(textSize.width),t.y-(textSize.height+baseline)),(255,255,255),CV_FILLED);

			putText(img_display, ScoreSTR, t ,cv::FONT_HERSHEY_PLAIN,0.75,(0,0,0),1);
		}
	}

} // End OverlayROIs()

void OverlayROIs_16BPP(UINT8* DisplayImg, int width, int height, SFRPlus & SB)
{
	cv::Mat img_display(height,width,CV_16U,DisplayImg);	

	char ScoreSTR[128];
	int i;
	cv::Size textSize;
	int baseline;
	cv::Point s,e,c,t;

	for(i=0;i<ROI_PER_SLANTBOX;i++)
	{
		if(SB.SFRBox[i].inRange)
		{
			s.x=SB.SFRBox[i].roi.x;
			s.y=SB.SFRBox[i].roi.y;
			e.x =s.x+SB.SFRBox[i].roi.width;
			e.y =s.y+SB.SFRBox[i].roi.height;
			c.x = s.x-((s.x-e.x)/2);
			c.y = s.y-((s.y-e.y)/2);

			rectangle( img_display, s , e, (128,128,128), 1, 8, 0 ); 
			sprintf(ScoreSTR,"%4.2f",ceilf(SB.SFRBox[i].SFRValue * 100) / 100);

			textSize = getTextSize(ScoreSTR,cv::FONT_HERSHEY_PLAIN,0.75,1,&baseline);

			t.x = c.x-(textSize.width/2);
			t.y = c.y+(textSize.height/2);

			rectangle(img_display,t,cv::Point(t.x+(textSize.width),t.y-(textSize.height+baseline)),(255,255,255),CV_FILLED);

			putText(img_display, ScoreSTR, t ,cv::FONT_HERSHEY_PLAIN,0.75,(0,0,0),1);
		}
	}

} // End OverlayROIs_16BPP()

void InitializeBoxes()
{
	for ( int i = 0 ; i < MAX_NUMBER_BOXES; i++)
	{
		Boxes[i].box_ctr.x = 0;
		Boxes[i].box_ctr.y = 0;
		for ( int j = 0; j < ROI_PER_SLANTBOX; j++)
		{
			Boxes[i].roi_used[j] = false;
			Boxes[i].SFRBox[j].DesiredFreq = 0;
			Boxes[i].SFRBox[j].inRange = false;
			Boxes[i].SFRBox[j].isHorz = false;
			Boxes[i].SFRBox[j].roi.x = 0;
			Boxes[i].SFRBox[j].roi.y = 0;
			Boxes[i].SFRBox[j].roi.width = 0;
			Boxes[i].SFRBox[j].roi.height = 0;
			Boxes[i].SFRBox[j].x_Delta = 0;
			Boxes[i].SFRBox[j].y_Delta = 0;
			Boxes[i].SFRBox[j].SFRLimit = 0;
			Boxes[i].SFRBox[j].SFRValue = 0;
			Boxes[i].SFRBox[j].sideused = NEVER;
		}
		Boxes[i].SB_Size = 0;
	}
} // End InitializeBoxes()

void Score_Image(int argc, char* argv[], const flags_t modeflags)
{
	bool lSaturated = false;
	InitializeBoxes();
	InitializeDetection();
	char szChar[1024] = {0};
	strcpy(szChar, argv[argc]);
	cout << "Filename: " << szChar << endl; 
	size_t lLength = strlen(szChar);
	PGMData* dataArray_Hi_16BitsPerPixel = new PGMData();
	PGMData* dataArray_Hi_12BitsPerPixel = new PGMData();
	int lCol = 0;
	int lRow = 0;

	if (((szChar[lLength-1] == 'm') || (szChar[lLength-1] == 'M')) && ((szChar[lLength-2] == 'g') || (szChar[lLength-2] == 'G')) && ((szChar[lLength-3] == 'p')|| (szChar[lLength-3] == 'P')))
	{
		// Read PGM data as a 2d Matrix
		cout << "Reading .pgm file" << endl;
		dataArray_Hi_16BitsPerPixel = readPGM(szChar, dataArray_Hi_16BitsPerPixel);
		lCol = dataArray_Hi_16BitsPerPixel->col;
		lRow = dataArray_Hi_16BitsPerPixel->row;
		//dataArray_Hi_16BitsPerPixel->max_gray = (1<<16)-1;
	}
	else if (((szChar[lLength-1] == 'g') || (szChar[lLength-1] == 'G')) && ((szChar[lLength-2] == 'n') || (szChar[lLength-2] == 'N')) && ((szChar[lLength-3] == 'p')|| (szChar[lLength-3] == 'P')))
	{
		cout << "Reading .png file" << endl;
		ILboolean result = ilLoadImage( szChar ) ;
		if(!result)
		{
			cout << "Unable to read: " << szChar << endl;
			DelayExit(1);
		}

		int lSize = ilGetInteger( IL_IMAGE_SIZE_OF_DATA ) ;
		//data->row, data->col
		int lBits_Per_Pixel = ilGetInteger(IL_IMAGE_BITS_PER_PIXEL);
		dataArray_Hi_16BitsPerPixel->col = ilGetInteger(IL_IMAGE_WIDTH);
		dataArray_Hi_16BitsPerPixel->row = ilGetInteger(IL_IMAGE_HEIGHT);
		dataArray_Hi_16BitsPerPixel->max_gray = (1<<lBits_Per_Pixel) - 1;
		setPGM(ilGetData(),dataArray_Hi_16BitsPerPixel);

		lCol = ilGetInteger(IL_IMAGE_WIDTH);
		lRow = ilGetInteger(IL_IMAGE_HEIGHT);

	}
	else
	{
		cout << "Unsupported file format: " << szChar << endl;
		delete dataArray_Hi_12BitsPerPixel;
		delete dataArray_Hi_16BitsPerPixel;
		exit(1);
	}

	dataArray_Hi_12BitsPerPixel->col = lCol;
	dataArray_Hi_12BitsPerPixel->row = lRow;
	dataArray_Hi_12BitsPerPixel->matrix = allocate_dynamic_matrix(dataArray_Hi_12BitsPerPixel->row, dataArray_Hi_12BitsPerPixel->col);
	// Reset maximum gray value for 12 bpp data
	dataArray_Hi_12BitsPerPixel->max_gray = 4095;

	// Determine how many bits to shift data
	unsigned int bits2shift = 0;
	if (dataArray_Hi_16BitsPerPixel->max_gray == 65535) { bits2shift = 4; }
	// Convert to 12 bit image (if needed)
	for (int i = 0; i < dataArray_Hi_16BitsPerPixel->row; ++i)
	{
		int ii = i;
		// If user has requested flipping the image vertically, do it
		if (modeflags.preprocessing & PREPROCESS_FLIPTB) { ii = dataArray_Hi_16BitsPerPixel->row - i - 1; }
		for (int j = 0; j < dataArray_Hi_16BitsPerPixel->col; ++j)
		{
			int jj = j;
			// If user has requested flipping the image horizontally, do it
			if (modeflags.preprocessing & PREPROCESS_FLIPLR) {	jj = dataArray_Hi_16BitsPerPixel->col - j - 1; }
			// If source file is truly 16-bit shift by 4 bits to get data into the 12 LSB
			dataArray_Hi_12BitsPerPixel->matrix[ii][jj] = (dataArray_Hi_16BitsPerPixel->matrix[i][j])>>bits2shift; 
		}		
	}
		
	/*
		The next section below seems to be a lazy way to get the 12-bit PGM struct data
		into another data format by writing to a file and then reading it back in using 
		OpenCV code.  Can this be directly coverted in memory so we don't have to go 
		through all the extra I/O to disk?  MWW
	*/
	
	strcat(szChar, "_12bits.pgm");
	if (modeflags.WriteImageLevel >= 0)
	{
		writePGM(szChar, dataArray_Hi_12BitsPerPixel);
		cout << szChar << " is Created" << endl; 
	}

	char szChar_DD[1024] = {0};
	unsigned int index2= 0;

	size_t length2= strlen(szChar);
	for(int j=0; j<length2; j++)
	{
		//szChar_DD[index2++]= szChar[j];
		if(szChar[j]!= '\\')
		{
			szChar_DD[index2++]= szChar[j];
		}
		else
		{
			strcat(szChar_DD,"\\\\");
			index2+= 2;
		}
	}
	cout << "Reading file " << szChar_DD << endl;
	cv::Mat lImageData_12BitsPerPixel = cv::imread(szChar_DD, CV_LOAD_IMAGE_ANYDEPTH);
	if (lImageData_12BitsPerPixel.rows == 0 )
	{
		cout << "Error reading the 12 bits Per Pixel Image" << endl;
		DelayExit(1);
	}
	////////////////////////////////////////////////////
	// C O M M O N   S T A R T I N G   P O I N T
	////////////////////////////////////////////////////

	// Find Red channel from data averages and correct white pixel values
	int redchan = 0;
	float chanscale[4];
	MeasureRCCCAttributes( lImageData_12BitsPerPixel, &redchan, chanscale );

	// Create floating point version of image data
	cv::Mat lImageData_32BitsPerPixel;
	lImageData_12BitsPerPixel.convertTo(lImageData_32BitsPerPixel,CV_32F);
	if (modeflags.Demosaic == 1) {
		// Re-scale channels; normalize to pixel opposite red
		#pragma omp parallel for
		for (unsigned int row=0; row<lImageData_32BitsPerPixel.rows; row+=2) 
		{
			for (unsigned int col=0; col<lImageData_32BitsPerPixel.cols; col+=2)
			{
				dataArray_Hi_12BitsPerPixel->matrix[row][col] = (int)(((float)dataArray_Hi_12BitsPerPixel->matrix[row][col]) * chanscale[0]);
				dataArray_Hi_12BitsPerPixel->matrix[row][col+1] = (int)(((float)dataArray_Hi_12BitsPerPixel->matrix[row][col+1]) * chanscale[1]);
				dataArray_Hi_12BitsPerPixel->matrix[row+1][col] = (int)(((float)dataArray_Hi_12BitsPerPixel->matrix[row+1][col]) * chanscale[2]);
				dataArray_Hi_12BitsPerPixel->matrix[row+1][col+1] = (int)(((float)dataArray_Hi_12BitsPerPixel->matrix[row+1][col+1]) * chanscale[3]);
			} // End for(col)
		} // End for(row)
	} // End if (demosaic)
	// Duplicate 12 bit image for Display purposes (??)
	cv::Mat lImageDataDisplay_12BitsPerPixel = lImageData_12BitsPerPixel.clone();

	int lNumberOfDetectedBoxes;
	int SaturationX = 0;
	int SaturationY = 0;

	// Switch fiducial finding based on algorithm
	switch (modeflags.lTargetDetection)
	{
	case 1:
		{
			cv::Mat lImageDataDisplay_32BitsPerPixel;
			lImageDataDisplay_12BitsPerPixel.convertTo(lImageDataDisplay_32BitsPerPixel,CV_32F);
			for (int i = 0; i < lImageDataDisplay_12BitsPerPixel.rows; ++i)
			{
				for (int j = 0; j < lImageDataDisplay_12BitsPerPixel.cols; ++j)
				{
					// to avoid the embedded data
					if ((i >= 4) && (i <= 945))
					{
						if ((modeflags.lRunningMode == 1) && (lImageDataDisplay_12BitsPerPixel.at<ushort>(i,j) > 2048))
						{
							lSaturated = true;
							SaturationX = j;
							SaturationY = i;
							break;
						}
						if ((modeflags.lRunningMode == 2) && (lImageDataDisplay_12BitsPerPixel.at<ushort>(i,j) > 3800))
						{
							lSaturated = true;
							SaturationX = j;
							SaturationY = i;
							break;
						}
					}
				}
				if (lSaturated)
					break;					
			}
			if(!lSaturated)
				lNumberOfDetectedBoxes = LocateMultipleFeatures(lImageData_32BitsPerPixel, lImageDataDisplay_32BitsPerPixel, lImageDataDisplay_32BitsPerPixel.cols, lImageDataDisplay_32BitsPerPixel.rows);
		}
		break;
	case 2:
		{
			cv::Mat lImageDataDisplay_8BitsPerPixel = cv::Mat::zeros(lImageDataDisplay_12BitsPerPixel.rows, lImageDataDisplay_12BitsPerPixel.cols, CV_8U);
			for (int i = 0; i < lImageDataDisplay_12BitsPerPixel.rows; ++i)
			{
				for (int j = 0; j < lImageDataDisplay_12BitsPerPixel.cols; ++j)
				{
					// to avoid the embedded data
					if ((i >= 4) && (i <= 945))
					{
						if ((modeflags.lRunningMode == 1) && (lImageDataDisplay_12BitsPerPixel.at<ushort>(i,j) > 2048))
						{
							lSaturated = true;
							SaturationX = j;
							SaturationY = i;
							break;
						}
						if ((modeflags.lRunningMode == 2) && (lImageDataDisplay_12BitsPerPixel.at<ushort>(i,j) > 3800))
						{
							lSaturated = true;
							SaturationX = j;
							SaturationY = i;
							break;
						}
					}
					// Division by 16
					lImageDataDisplay_8BitsPerPixel.at<char>(i,j) = lImageDataDisplay_12BitsPerPixel.at<ushort>(i,j) / 16.0;
				}
				if (lSaturated)
					break;					
			}
			if(!lSaturated)
				lNumberOfDetectedBoxes = LocateMultipleFeaturesDetection(lImageDataDisplay_8BitsPerPixel, lImageDataDisplay_12BitsPerPixel, lImageDataDisplay_8BitsPerPixel.cols, lImageDataDisplay_8BitsPerPixel.rows, modeflags);
		}
		break;
	case 3:
		{
			cout << "Detect Objects and check for Saturation" << endl;
			cv::Mat lImageDataDisplay_8BitsPerPixel = cv::Mat::zeros(lImageDataDisplay_12BitsPerPixel.rows, lImageDataDisplay_12BitsPerPixel.cols, CV_8U);
			for (int i = 0; i < lImageDataDisplay_12BitsPerPixel.rows; ++i)
			{
				for (int j = 0; j < lImageDataDisplay_12BitsPerPixel.cols; ++j)
				{
					// to avoid the embedded data
					if ((i >= 4) && (i <= 945))
					{
						if ((modeflags.lRunningMode == 1) && (lImageDataDisplay_12BitsPerPixel.at<ushort>(i,j) > 2048))
						{
							lSaturated = true;
							SaturationX = j;
							SaturationY = i;
							break;
						}
						if ((modeflags.lRunningMode == 2) && (lImageDataDisplay_12BitsPerPixel.at<ushort>(i,j) > 3800))
						{
							lSaturated = true;
							SaturationX = j;
							SaturationY = i;
							break;
						}
					}
					// Division by 16
					lImageDataDisplay_8BitsPerPixel.at<char>(i,j) = lImageDataDisplay_12BitsPerPixel.at<ushort>(i,j) / 16.0;
				}
				if (lSaturated)
					break;			
			}
			if(!lSaturated)
				//ImageWrite("8bitdead.pgm",lImageDataDisplay_8BitsPerPixel);
			    //ImageWrite("12bitdead.pgm",lImageDataDisplay_12BitsPerPixel);
				lNumberOfDetectedBoxes = LocateMultipleFeaturesDetection2(lImageDataDisplay_8BitsPerPixel, lImageDataDisplay_12BitsPerPixel, lImageDataDisplay_8BitsPerPixel.cols, lImageDataDisplay_8BitsPerPixel.rows, modeflags);
		}
		break;
	default:
		cout << "Not supported approach" << endl;
		exit(1);
	}

	if (lSaturated && modeflags.lRunningMode==2)
	{
		cout << "Saturated Image at  Image[" << SaturationX << "][" << SaturationY << "] = " << lImageDataDisplay_12BitsPerPixel.at<ushort>(SaturationY,SaturationX) <<  endl;
		cout << "Press Any Key to Continue" << endl;
		getchar(); 
	}

	if (modeflags.lTargetDetection != 3)
	{
		cout << endl << "Initializing ROIs" << endl;
		InitializeROIs(xmlpath);
	}
	else
	{
		cout << endl << "Detected ROIs" << endl;
		bool lIsHorz = true;
		for (int boxNumber = 0; boxNumber < MAX_NUMBER_BOXES; boxNumber++)
		{
			Boxes[boxNumber].SB_Size = (AllTargetBoxes[boxNumber].m_TargetBoxHeight + AllTargetBoxes[boxNumber].m_TargetBoxWidth)/2;	
			// Apply "rules" based on rows columns or both. Set defautls for all boxes first and then modify based on rules
			Boxes[boxNumber].roi_used[N] = true;
			Boxes[boxNumber].roi_used[S] = true;
			Boxes[boxNumber].roi_used[E] = true;
			Boxes[boxNumber].roi_used[W] = true;

			Boxes[boxNumber].box_ctr.y =  AllTargetBoxes[boxNumber].m_TargetBoxCenter.y;
			Boxes[boxNumber].box_ctr.x = AllTargetBoxes[boxNumber].m_TargetBoxCenter.x;


	
			for(int i = 0;i<ROI_PER_SLANTBOX;i++)
			{
				Boxes[boxNumber].SFRBox[i].isHorz = (i%2 == 0)? true: false; // for boxes 
				//Boxes[boxNumber].SFRBox[i].isHorz = (i%2 == 0)? false: true; //
				// Re-do next two lines to worry about even size in another step ( bitwise & with (~1) )	
				Boxes[boxNumber].SFRBox[i].roi.width = (i%2 == 0)? (((Boxes[boxNumber].SB_Size*2/3)%2 !=0)? (Boxes[boxNumber].SB_Size*2/3)-1:(Boxes[boxNumber].SB_Size*2/3)): (((Boxes[boxNumber].SB_Size*1/3)%2 != 0)? (Boxes[boxNumber].SB_Size*1/3)-1:(Boxes[boxNumber].SB_Size*1/3));
				Boxes[boxNumber].SFRBox[i].roi.height = (i%2 == 0)? (((Boxes[boxNumber].SB_Size*1/3)%2 !=0)?(Boxes[boxNumber].SB_Size*1/3)-1:(Boxes[boxNumber].SB_Size*1/3)): (((Boxes[boxNumber].SB_Size*2/3)%2 != 0)? (Boxes[boxNumber].SB_Size*2/3)-1:(Boxes[boxNumber].SB_Size*2/3));
				Boxes[boxNumber].SFRBox[i].x_Delta = 10;
				Boxes[boxNumber].SFRBox[i].y_Delta = 10;
			    
					
				switch (i) 
				{
				case 0 : //North
					Boxes[boxNumber].SFRBox[i].roi.x = AllTargetBoxes[boxNumber].m_TargetBoxCenter.x -
						Boxes[boxNumber].SFRBox[i].roi.width/2 - 
						Boxes[boxNumber].SFRBox[i].x_Delta;

					Boxes[boxNumber].SFRBox[i].roi.y = AllTargetBoxes[boxNumber].m_TargetBoxCenter.y - 
						AllTargetBoxes[boxNumber].m_TargetBoxHeight/2 -
						Boxes[boxNumber].SFRBox[i].roi.height/2 +
						Boxes[boxNumber].SFRBox[i].y_Delta;	
					break;
				case 2 : //south
					Boxes[boxNumber].SFRBox[i].roi.x = AllTargetBoxes[boxNumber].m_TargetBoxCenter.x -
						Boxes[boxNumber].SFRBox[i].roi.width/2 +
						Boxes[boxNumber].SFRBox[i].x_Delta;

					Boxes[boxNumber].SFRBox[i].roi.y = AllTargetBoxes[boxNumber].m_TargetBoxCenter.y + 
						AllTargetBoxes[boxNumber].m_TargetBoxHeight/2 -
						Boxes[boxNumber].SFRBox[i].roi.height/2 - 
						Boxes[boxNumber].SFRBox[i].y_Delta;	
					break;
				case 1 : // EAST
					Boxes[boxNumber].SFRBox[i].roi.x = AllTargetBoxes[boxNumber].m_TargetBoxCenter.x +
						AllTargetBoxes[boxNumber].m_TargetBoxWidth/2-
						Boxes[boxNumber].SFRBox[i].roi.width/2 -
						Boxes[boxNumber].SFRBox[i].x_Delta;

					Boxes[boxNumber].SFRBox[i].roi.y = AllTargetBoxes[boxNumber].m_TargetBoxCenter.y -
						Boxes[boxNumber].SFRBox[i].roi.height/2 - 
						Boxes[boxNumber].SFRBox[i].y_Delta;	
					break;
				case 3 : //West
					Boxes[boxNumber].SFRBox[i].roi.x = AllTargetBoxes[boxNumber].m_TargetBoxCenter.x -
						AllTargetBoxes[boxNumber].m_TargetBoxWidth/2-
						Boxes[boxNumber].SFRBox[i].roi.width/2 +
						Boxes[boxNumber].SFRBox[i].x_Delta;

					Boxes[boxNumber].SFRBox[i].roi.y = AllTargetBoxes[boxNumber].m_TargetBoxCenter.y -
						Boxes[boxNumber].SFRBox[i].roi.height/2 +
						Boxes[boxNumber].SFRBox[i].y_Delta;
					break;
				default :
					break;
				}
			
				if ( Boxes[boxNumber].SFRBox[i].roi.x < 0)
					Boxes[boxNumber].SFRBox[i].roi.x = 0;
				if (Boxes[boxNumber].SFRBox[i].roi.y < 0)
					Boxes[boxNumber].SFRBox[i].roi.y = 0;
				Boxes[boxNumber].SFRBox[i].isHorz =  lIsHorz;
				lIsHorz = !lIsHorz;
				Boxes[boxNumber].SFRBox[i].DesiredFreq = modeflags.SpatialFreq;
				Boxes[boxNumber].SFRBox[i].SFRLimit = 0.5; 
				Boxes[boxNumber].SFRBox[i].sideused = static_cast<ROIUSAGE>(2);
				Boxes[boxNumber].SFRBox[i].inRange = static_cast<bool>(0);
				Boxes[boxNumber].SFRBox[i].SFRValue = 0.0;
					
			}
		}

	}
		
	for ( int lRoi = 0; lRoi< ROI_PER_SLANTBOX; lRoi++)
	{
		for (int lBoxes = 0; lBoxes < MAX_NUMBER_BOXES; lBoxes++)
		{
			if ((Boxes[lBoxes].box_ctr.x != 0 ) &&  (Boxes[lBoxes].box_ctr.y != 0 ))
			{
				cv::Point p1;
				p1.x = Boxes[lBoxes].SFRBox[lRoi].roi.x;
				p1.y = Boxes[lBoxes].SFRBox[lRoi].roi.y;
				cv::Point p2;
				p2.x = Boxes[lBoxes].SFRBox[lRoi].roi.x + Boxes[lBoxes].SFRBox[lRoi].roi.width;
				p2.y = Boxes[lBoxes].SFRBox[lRoi].roi.y + Boxes[lBoxes].SFRBox[lRoi].roi.height;
				rectangle( lImageDataDisplay_12BitsPerPixel, p1, p2, cv::Scalar::all(0), 1, 8, 0 ); 
			}
		}
	}
	if (modeflags.WriteImageLevel >= 1)
	{
		strcat(szChar_DD, "_ObjectDetection.pgm");
		ImageWrite(szChar_DD,lImageDataDisplay_12BitsPerPixel);
	}

	cout << endl << "Applying RCC Filter and Scores Slant-Edges" << endl;

	// change 2d Matrix into 1d vector
	wchar_t *DataVector_Hi = new wchar_t[dataArray_Hi_12BitsPerPixel->row *dataArray_Hi_12BitsPerPixel->col];
	memset(DataVector_Hi,0, dataArray_Hi_12BitsPerPixel->row *dataArray_Hi_12BitsPerPixel->col *2);
	int k_Hi = 0;
	//writePGM("deadSFR.pgm",dataArray_Hi_12BitsPerPixel);
	for (int i = 0; i < dataArray_Hi_12BitsPerPixel->row; ++i)
	{
		for (int j = 0; j < dataArray_Hi_12BitsPerPixel->col; ++j)
		{
			DataVector_Hi[k_Hi] = static_cast<wchar_t>(static_cast<int>(dataArray_Hi_12BitsPerPixel->matrix[i][j])); 
			k_Hi++;
		}
	}

	for (int lBoxes = 0; lBoxes < MAX_NUMBER_BOXES; lBoxes++)
	{
#ifdef _DEBUG
		printf("=========>  B O X   %2d  <===========\n",lBoxes);
#endif
		for ( int lRoi = 0; lRoi< ROI_PER_SLANTBOX; lRoi++)
		{
			if ((Boxes[lBoxes].box_ctr.x != 0 ) &&  (Boxes[lBoxes].box_ctr.y != 0 ))
				CalcSFR_Hi(DataVector_Hi, Boxes[lBoxes].SFRBox[lRoi], dataArray_Hi_12BitsPerPixel->col, dataArray_Hi_12BitsPerPixel->row, dataArray_Hi_12BitsPerPixel->max_gray, 0.00375, modeflags, redchan);
		}
	}
	
	// Deallocate dynamic array
	delete[] DataVector_Hi;

	// Report to screen
	cout <<"Data"<<" ";
	cout <<"  X  "<<" ";
	cout <<"  Y  "<<" ";
	for (int k = 0; k < MAX_NUMBER_BOXES; k++)
		cout <<"Box"<<k<< " ";
	cout<<endl;

	for (int lBoxes = 0; lBoxes < MAX_NUMBER_BOXES; lBoxes++)
	{
		cout <<Boxes[lBoxes].box_ctr.x <<"   ";			
	}
	cout<<endl;
	for (int lBoxes = 0; lBoxes < MAX_NUMBER_BOXES; lBoxes++)
	{
		cout <<Boxes[lBoxes].box_ctr.y <<"   ";
	}
	cout<<endl;

	for ( int lRoi = 0; lRoi< ROI_PER_SLANTBOX; lRoi++)
	{
		for (int lBoxes = 0; lBoxes < MAX_NUMBER_BOXES; lBoxes++)
		{
			cout <<Boxes[lBoxes].SFRBox[lRoi].SFRValue <<"   ";
		}
		cout<<endl;
	}
	// Declare file objects
	std::fstream testread("Datasheet.csv",ios::in);
	std::ofstream MyFile;
	// Test to see if open is 'good'; if not, file probably doesn't exist and we should put in header
	if (!Once && !testread.good()) {
		// Close test file handle
		testread.close();
		// Open file for writing
		MyFile.open ("DataSheet.csv", ios::out | ios::ate | ios::app | ios::binary);
		// Write header
		MyFile << "SN,Version,Time,Demosaic Appraoch, Detection Approach, Running Mode,Spatial Freq,X,Y,ROI0,ROI1,ROI2,ROI3,X,Y,ROI4,ROI5,ROI6,ROI7,X,Y,ROI8,ROI9,ROI10,ROI11,X,Y,ROI12,ROI13,ROI14,ROI15,X,Y,ROI16,ROI17,ROI18,ROI19,X,Y,ROI20,ROI21,ROI22,ROI23,X,Y,ROI24,ROI25,ROI26,ROI27,X,Y,ROI28,ROI29,ROI30,ROI31,X,Y,ROI32,ROI33,ROI34,ROI35,X,Y,ROI36,ROI37,ROI38,ROI39,X,Y,ROI40,ROI41,ROI42,ROI43" << endl;
		Once = true;
	} else {
		// Close test file handle
		testread.close();
		// Open file for writing
		MyFile.open ("DataSheet.csv", ios::out | ios::ate | ios::app | ios::binary);
		Once = true;
	}
	MyFile << szChar << ",";
	MyFile << VERSION << "." << RELEASE << "." << BUILD<<",";
	char s[100];
	time_t now = time(NULL);
	struct tm * p = localtime(&now);
	strftime(s, 100, "%A %B:%d:%Y", p);

	MyFile << s << "," ;
	switch (modeflags.Demosaic)
	{
		case 0:MyFile <<"Row/Col removal,";
			break;
		case 1: MyFile <<"Modified Gradient,";
			break;
	}
	switch (modeflags.lTargetDetection)
	{
		case 1:MyFile <<"Template Match,";
			break;
		case 2: MyFile <<"Circle Detection,";
		break;
	case 3: MyFile <<"Edge Detection,";
			break;
		}
	switch (modeflags.lRunningMode)
	{
		case 1:MyFile <<"HDR,";
			break;
		case 2: MyFile <<"Linear,";
			break;
	}
	// Report Spatial Freq.
	MyFile << modeflags.SpatialFreq << ",";
	// Report box locations and scores
	for (int lBoxes = 0; lBoxes < MAX_NUMBER_BOXES; lBoxes++)
	{
		MyFile << Boxes[lBoxes].box_ctr.x <<"," << Boxes[lBoxes].box_ctr.y <<",";
		for ( int lRoi = 0; lRoi< ROI_PER_SLANTBOX; lRoi++)
		{
				MyFile <<Boxes[lBoxes].SFRBox[lRoi].SFRValue <<",";
		}
	}
	MyFile <<endl;
	MyFile.close();

	deallocate_dynamic_matrix(dataArray_Hi_12BitsPerPixel->matrix, dataArray_Hi_12BitsPerPixel->row);
	deallocate_dynamic_matrix(dataArray_Hi_16BitsPerPixel->matrix, dataArray_Hi_16BitsPerPixel->row);
	delete dataArray_Hi_12BitsPerPixel;
	delete dataArray_Hi_16BitsPerPixel;
}

void Score_Image(unsigned char *pBuffer, const flags_t modeflags)
{
	bool lSaturated = false;
	PGMData* dataArray_Hi_12BitsPerPixel = new PGMData();
	InitializeBoxes();
	InitializeDetection();
	// Get PGM data as a 2d Matrix
	dataArray_Hi_12BitsPerPixel->col = DEFAULT_IMAGE_WIDTH;
	dataArray_Hi_12BitsPerPixel->row = DEFAULT_IMAGE_HEIGHT;
	dataArray_Hi_12BitsPerPixel->matrix = allocate_dynamic_matrix(dataArray_Hi_12BitsPerPixel->row, dataArray_Hi_12BitsPerPixel->col);
	dataArray_Hi_12BitsPerPixel->max_gray = 4095;
	// Convert to 12 bit image
	int pBuffer_Counter = 0;
	//char savefn2[30] = "12bitliveimage_";
	char s[100];
	time_t now;
	struct tm * p;
	#pragma omp parallel for
	for (int i = 0; i < dataArray_Hi_12BitsPerPixel->row; ++i)
	{
		int ii = i;
		// If user has requested flipping the image vertically, do it
		if (modeflags.preprocessing & PREPROCESS_FLIPTB) { ii = dataArray_Hi_12BitsPerPixel->row - i - 1; }
		for (int j = 0; j < dataArray_Hi_12BitsPerPixel->col; ++j)
		{
			unsigned short pixel_firstbyte = pBuffer[pBuffer_Counter];
			unsigned short pixel_secondbyte = pBuffer[pBuffer_Counter+1];
			unsigned short pixel = (pixel_secondbyte << 8) | pixel_firstbyte;
			if(camerabrand == OMNI)
			{
				pixel = pixel/16.0;
			}
			int jj = j;
			// If user has requested flipping the image horizontally, do it
			if (modeflags.preprocessing & PREPROCESS_FLIPLR) {	jj = dataArray_Hi_12BitsPerPixel->col - j - 1; }
			dataArray_Hi_12BitsPerPixel->matrix[ii][jj] = static_cast<wchar_t>(static_cast<int>(pixel)); 
			pBuffer_Counter += 2;			
		}
	}
/*
#ifdef _DEBUG
	// Are we going to save off raw images with numbered/serialized filenames (usable for SFR scoring)
	if (modeflags.num_frames) {
		// Use last 32 bits of fuse ID for serialization reporting of images
		unsigned int combined[2];
		combined[0] = (modeflags.FuseID[3] << 16) & 0xffff0000 | (modeflags.FuseID[2] & 0x0000ffff);
		combined[1] = (modeflags.FuseID[1] << 16) & 0xffff0000 | (modeflags.FuseID[0] & 0x0000ffff);
		char savefn[30];
		sprintf(savefn,"%08X-%02d.pgm",combined[0],combined[1],modeflags.num_frames);
		writePGM(savefn, dataArray_Hi_12BitsPerPixel);
	}
#endif
*/
	if(modeflags.savelive)
	{
		now = time(NULL);
		p = localtime(&now);
		strftime(s, 100, "Time is %H %M %S", p);
		//strcat(savefn2,s);
		//strcat(savefn2,".pgm");
		writePGM("liveimage.pgm", dataArray_Hi_12BitsPerPixel);
		//memset(savefn2, 0, sizeof(savefn2));
	}

	//////////////////////////////////////////////////////////////////
	// Line below is a disconnect; above we do image flipping from the pBuffer into dataArray_Hi_12BitsPerPixel (PGMData), which is 
	// ...what the ROI's are extracted from below, but in the line below, we're copying the pBuffer again into a new variable
	// ...lImageData_12BitsPerPixel (cv::Mat)
	///////////////////////////////////////////////////////////////////

	cv::Mat lImageData_12BitsPerPixel = cv::Mat(dataArray_Hi_12BitsPerPixel->row, dataArray_Hi_12BitsPerPixel->col, CV_16UC1, pBuffer);
	////////////////////////////////////////////////////
	// C O M M O N   S T A R T I N G   P O I N T
	////////////////////////////////////////////////////
	if(camerabrand == OMNI)
	{
		lImageData_12BitsPerPixel.convertTo(lImageData_12BitsPerPixel, CV_16U, 1.0/16.0);
	}

	// Find Red channel from data averages and correct white pixel values
	int redchan = 0;
	float chanscale[4];
	MeasureRCCCAttributes( lImageData_12BitsPerPixel, &redchan, chanscale );

	// Create floating point version of image data
	cv::Mat lImageData_32BitsPerPixel;
	lImageData_12BitsPerPixel.convertTo(lImageData_32BitsPerPixel,CV_32F);
	// Re-scale channels; normalize to pixel opposite red
	#pragma omp parallel for
	for (unsigned int row=0; row<lImageData_32BitsPerPixel.rows; row+=2) 
	{
		for (unsigned int col=0; col<lImageData_32BitsPerPixel.cols; col+=2)
		{
			/*
				This correction needs to be done on dataArray_Hi_12BitsPerPixel, which is where the ROI's are extracted from

			lImageData_32BitsPerPixel.at<ushort>(row,col) *= chanscale[0];
			lImageData_32BitsPerPixel.at<ushort>(row,col+1) *= chanscale[1];
			lImageData_32BitsPerPixel.at<ushort>(row+1,col) *= chanscale[2];
			lImageData_32BitsPerPixel.at<ushort>(row+1,col+1) *= chanscale[3];
			*/
		}
	}

	cv::Mat lImageDataDisplay_12BitsPerPixel = lImageData_12BitsPerPixel.clone();
	cv::Mat lImageDataDisplay_8BitsPerPixel;

	int lNumberOfDetectedBoxes;
	int SaturationX = 0;
	int SaturationY = 0;

	// Switch fiducial finding based on algorithm
	switch (modeflags.lTargetDetection)
	{
	case 1:
		{
			cv::Mat lImageDataDisplay_32BitsPerPixel;
			lImageDataDisplay_12BitsPerPixel.convertTo(lImageDataDisplay_32BitsPerPixel,CV_32F);
			for (int i = 0; i < lImageDataDisplay_12BitsPerPixel.rows; ++i)
			{
				for (int j = 0; j < lImageDataDisplay_12BitsPerPixel.cols; ++j)
				{
					// to avoid the embedded data
					if ((i >= 4) && (i <= 945))
					{
						if ((modeflags.lRunningMode == 1) && (lImageDataDisplay_12BitsPerPixel.at<ushort>(i,j) > 2048))
						{
							lSaturated = true;
							SaturationX = j;
							SaturationY = i;
							break;
						}
						if ((modeflags.lRunningMode == 2) && (lImageDataDisplay_12BitsPerPixel.at<ushort>(i,j) > 3800))
						{
							lSaturated = true;
							SaturationX = j;
							SaturationY = i;
							break;
						}
					}
				}
				if (lSaturated)
					break;					
			}
			if(!lSaturated)
				lNumberOfDetectedBoxes = LocateMultipleFeatures(lImageData_32BitsPerPixel, lImageDataDisplay_32BitsPerPixel, lImageDataDisplay_32BitsPerPixel.cols, lImageDataDisplay_32BitsPerPixel.rows);
		}
		break;
	case 2:
		{
			lImageDataDisplay_8BitsPerPixel = cv::Mat::zeros(lImageDataDisplay_12BitsPerPixel.rows, lImageDataDisplay_12BitsPerPixel.cols, CV_8U);
			for (int i = 0; i < lImageDataDisplay_12BitsPerPixel.rows; ++i)
			{
				for (int j = 0; j < lImageDataDisplay_12BitsPerPixel.cols; ++j)
				{
					// to avoid the embedded data
					if ((i >= 4) && (i <= 945))
					{
						if ((modeflags.lRunningMode == 1) && (lImageDataDisplay_12BitsPerPixel.at<ushort>(i,j) > 2048))
						{
							lSaturated = true;
							SaturationX = j;
							SaturationY = i;
							break;
						}
						if ((modeflags.lRunningMode == 2) && (lImageDataDisplay_12BitsPerPixel.at<ushort>(i,j) > 3800))
						{
							lSaturated = true;
							SaturationX = j;
							SaturationY = i;
							break;
						}
					}
					// Division by 16
					lImageDataDisplay_8BitsPerPixel.at<char>(i,j) = lImageDataDisplay_12BitsPerPixel.at<ushort>(i,j) / 16;
				}
				if (lSaturated)
					break;					
			}
			if(!lSaturated)
				lNumberOfDetectedBoxes = LocateMultipleFeaturesDetection(lImageDataDisplay_8BitsPerPixel, lImageDataDisplay_12BitsPerPixel, lImageDataDisplay_8BitsPerPixel.cols, lImageDataDisplay_8BitsPerPixel.rows, modeflags);
		}
		break;
	case 3:
		{
			lImageDataDisplay_8BitsPerPixel = cv::Mat::zeros(lImageDataDisplay_12BitsPerPixel.rows, lImageDataDisplay_12BitsPerPixel.cols, CV_8U);
			for (int i = 0; i < lImageDataDisplay_12BitsPerPixel.rows; ++i)
			{
				for (int j = 0; j < lImageDataDisplay_12BitsPerPixel.cols; ++j)
				{
					// to avoid the embedded data
					if ((i >= 4) && (i <= 945))
					{
						if ((modeflags.lRunningMode == 1) && (lImageDataDisplay_12BitsPerPixel.at<ushort>(i,j) > 2048))
						{
							lSaturated = true;
							SaturationX = j;
							SaturationY = i;
							break;
						}
						if ((modeflags.lRunningMode == 2) && (lImageDataDisplay_12BitsPerPixel.at<ushort>(i,j) > 3800))
						{
							lSaturated = true;
							SaturationX = j;
							SaturationY = i;
							break;
						}
					}
					// Division by 16
					//lImageDataDisplay_8BitsPerPixel.at<uchar>(i,j) = lImageDataDisplay_12BitsPerPixel.at<ushort>(i,j) / 16;
				} // End for j
			} // End for i
			lImageDataDisplay_12BitsPerPixel.convertTo(lImageDataDisplay_8BitsPerPixel, CV_8U, 1.0/16.0);
			// Marwan skipped the next step if we had saturated pixels, do we still need that?
			//ImageWrite("8bitlive.pgm",lImageDataDisplay_8BitsPerPixel);
			//ImageWrite("12bitlive.pgm",lImageDataDisplay_12BitsPerPixel);
			lNumberOfDetectedBoxes = LocateMultipleFeaturesDetection2(lImageDataDisplay_8BitsPerPixel, lImageDataDisplay_12BitsPerPixel, lImageDataDisplay_8BitsPerPixel.cols, lImageDataDisplay_8BitsPerPixel.rows, modeflags);
		} // End Case 3
		break;
	default:
		cout << "Not supported approach" << endl;
		return;
	}

	if (modeflags.lTargetDetection != 3)
	{
		InitializeROIs(xmlpath);
	}
	else
	{
		//bool lIsHorz = true;
		bool lIsHorz = true;
		for (int boxNumber = 0; boxNumber < MAX_NUMBER_BOXES; boxNumber++)
		{
			Boxes[boxNumber].SB_Size = (AllTargetBoxes[boxNumber].m_TargetBoxHeight + AllTargetBoxes[boxNumber].m_TargetBoxWidth)/2;	
			// Apply "rules" based on rows columns or both. Set defautls for all boxes first and then modify based on rules
			Boxes[boxNumber].roi_used[N] = true;
			Boxes[boxNumber].roi_used[S] = true;
			Boxes[boxNumber].roi_used[E] = true;
			Boxes[boxNumber].roi_used[W] = true;

			Boxes[boxNumber].box_ctr.y =  AllTargetBoxes[boxNumber].m_TargetBoxCenter.y;
			Boxes[boxNumber].box_ctr.x = AllTargetBoxes[boxNumber].m_TargetBoxCenter.x;
	
			for(int i = 0;i<ROI_PER_SLANTBOX;i++)
			{
				Boxes[boxNumber].SFRBox[i].isHorz = (i%2 == 0)? true: false;
					
				Boxes[boxNumber].SFRBox[i].roi.width = (i%2 == 0)? (((Boxes[boxNumber].SB_Size*2/3)%2 !=0)? (Boxes[boxNumber].SB_Size*2/3)-1:(Boxes[boxNumber].SB_Size*2/3)): (((Boxes[boxNumber].SB_Size*1/3)%2 != 0)? (Boxes[boxNumber].SB_Size*1/3)-1:(Boxes[boxNumber].SB_Size*1/3));
				Boxes[boxNumber].SFRBox[i].roi.height = (i%2 == 0)? (((Boxes[boxNumber].SB_Size*1/3)%2 !=0)?(Boxes[boxNumber].SB_Size*1/3)-1:(Boxes[boxNumber].SB_Size*1/3)): (((Boxes[boxNumber].SB_Size*2/3)%2 != 0)? (Boxes[boxNumber].SB_Size*2/3)-1:(Boxes[boxNumber].SB_Size*2/3));
				Boxes[boxNumber].SFRBox[i].x_Delta = 10;
				Boxes[boxNumber].SFRBox[i].y_Delta = 10;
					
				switch (i) 
				{
				case 0 : //North
					Boxes[boxNumber].SFRBox[i].roi.x = AllTargetBoxes[boxNumber].m_TargetBoxCenter.x -
						Boxes[boxNumber].SFRBox[i].roi.width/2 - 
						Boxes[boxNumber].SFRBox[i].x_Delta;

					Boxes[boxNumber].SFRBox[i].roi.y = AllTargetBoxes[boxNumber].m_TargetBoxCenter.y - 
						AllTargetBoxes[boxNumber].m_TargetBoxHeight/2 -
						Boxes[boxNumber].SFRBox[i].roi.height/2 +
						Boxes[boxNumber].SFRBox[i].y_Delta;	
					break;
				case 2 : //south
					Boxes[boxNumber].SFRBox[i].roi.x = AllTargetBoxes[boxNumber].m_TargetBoxCenter.x -
						Boxes[boxNumber].SFRBox[i].roi.width/2 +
						Boxes[boxNumber].SFRBox[i].x_Delta;

					Boxes[boxNumber].SFRBox[i].roi.y = AllTargetBoxes[boxNumber].m_TargetBoxCenter.y + 
						AllTargetBoxes[boxNumber].m_TargetBoxHeight/2 -
						Boxes[boxNumber].SFRBox[i].roi.height/2 - 
						Boxes[boxNumber].SFRBox[i].y_Delta;	
					break;
				case 1 : // EAST
					Boxes[boxNumber].SFRBox[i].roi.x = AllTargetBoxes[boxNumber].m_TargetBoxCenter.x +
						AllTargetBoxes[boxNumber].m_TargetBoxWidth/2-
						Boxes[boxNumber].SFRBox[i].roi.width/2 -
						Boxes[boxNumber].SFRBox[i].x_Delta;

					Boxes[boxNumber].SFRBox[i].roi.y = AllTargetBoxes[boxNumber].m_TargetBoxCenter.y -
						Boxes[boxNumber].SFRBox[i].roi.height/2 - 
						Boxes[boxNumber].SFRBox[i].y_Delta;	
					break;
				case 3 : //West
					Boxes[boxNumber].SFRBox[i].roi.x = AllTargetBoxes[boxNumber].m_TargetBoxCenter.x -
						AllTargetBoxes[boxNumber].m_TargetBoxWidth/2-
						Boxes[boxNumber].SFRBox[i].roi.width/2 +
						Boxes[boxNumber].SFRBox[i].x_Delta;

					Boxes[boxNumber].SFRBox[i].roi.y = AllTargetBoxes[boxNumber].m_TargetBoxCenter.y -
						Boxes[boxNumber].SFRBox[i].roi.height/2 +
						Boxes[boxNumber].SFRBox[i].y_Delta;
					break;
				default :
					break;
				}
			
				if ( Boxes[boxNumber].SFRBox[i].roi.x < 0)
					Boxes[boxNumber].SFRBox[i].roi.x = 0;
				if (Boxes[boxNumber].SFRBox[i].roi.y < 0)
					Boxes[boxNumber].SFRBox[i].roi.y = 0;
				Boxes[boxNumber].SFRBox[i].isHorz =  lIsHorz;
				lIsHorz = !lIsHorz;
				Boxes[boxNumber].SFRBox[i].DesiredFreq = modeflags.SpatialFreq;
				Boxes[boxNumber].SFRBox[i].SFRLimit = 0.5; 
				Boxes[boxNumber].SFRBox[i].sideused = static_cast<ROIUSAGE>(2);
				Boxes[boxNumber].SFRBox[i].inRange = static_cast<bool>(1);
				Boxes[boxNumber].SFRBox[i].SFRValue = 0.0;
					
			}
		}

	}
		
	for ( int lRoi = 0; lRoi< ROI_PER_SLANTBOX; lRoi++)
	{
		for (int lBoxes = 0; lBoxes < MAX_NUMBER_BOXES; lBoxes++)
		{
			if ((Boxes[lBoxes].box_ctr.x != 0 ) &&  (Boxes[lBoxes].box_ctr.y != 0 ))
			{
				cv::Point p1;
				p1.x = Boxes[lBoxes].SFRBox[lRoi].roi.x;
				p1.y = Boxes[lBoxes].SFRBox[lRoi].roi.y;
				cv::Point p2;
				p2.x = Boxes[lBoxes].SFRBox[lRoi].roi.x + Boxes[lBoxes].SFRBox[lRoi].roi.width;
				p2.y = Boxes[lBoxes].SFRBox[lRoi].roi.y + Boxes[lBoxes].SFRBox[lRoi].roi.height;
				rectangle( lImageDataDisplay_12BitsPerPixel, p1, p2, cv::Scalar::all(0), 1, 8, 0 ); 
			}
		}
	}

	// change 2d Matrix into 1d vector
	wchar_t *DataVector_Hi = new wchar_t[dataArray_Hi_12BitsPerPixel->row *dataArray_Hi_12BitsPerPixel->col];
	memset(DataVector_Hi,0, dataArray_Hi_12BitsPerPixel->row *dataArray_Hi_12BitsPerPixel->col *2);
	int k_Hi = 0;
	//writePGM("liveSFR.pgm",dataArray_Hi_12BitsPerPixel);
	for (int i = 0; i < dataArray_Hi_12BitsPerPixel->row; ++i)
	{
		for (int j = 0; j < dataArray_Hi_12BitsPerPixel->col; ++j)
		{
			DataVector_Hi[k_Hi] = static_cast<wchar_t>(static_cast<int>(dataArray_Hi_12BitsPerPixel->matrix[i][j])); 
			k_Hi++;
		}
	}

	for (int lBoxes = 0; lBoxes < MAX_NUMBER_BOXES; lBoxes++)
	{
		// Did the Box detector find this one?
//		if (m_DetectedTargetCenterBoxes[lBoxes]) {
		if (true) {
			for ( int lRoi = 0; lRoi< ROI_PER_SLANTBOX; lRoi++)
			{
				if ((Boxes[lBoxes].box_ctr.x != 0 ) &&  (Boxes[lBoxes].box_ctr.y != 0 ))
					CalcSFR_Hi(DataVector_Hi, Boxes[lBoxes].SFRBox[lRoi], dataArray_Hi_12BitsPerPixel->col, dataArray_Hi_12BitsPerPixel->row, dataArray_Hi_12BitsPerPixel->max_gray, 0.00375, modeflags, redchan);
			}
		}
	}

	// Deallocate memory
	delete[] DataVector_Hi;

	if(Display)
	{
		switch(camerabrand)
		{
		case APTI:
		//0.5 FOV
		cv::circle(lImageDataDisplay_8BitsPerPixel,cv::Point(640,480),400,CV_RGB(100,100,100),3,4);
		//0.7 FOV
		cv::circle(lImageDataDisplay_8BitsPerPixel,cv::Point(640,480),560,CV_RGB(100,100,100),3,4);
		break;
		case OMNI:
		//0.5 FOV
		cv::circle(lImageDataDisplay_8BitsPerPixel,cv::Point(640,480),400,CV_RGB(100,100,100),3,4);
		//0.7 FOV
		cv::circle(lImageDataDisplay_8BitsPerPixel,cv::Point(640,480),560,CV_RGB(100,100,100),3,4);
		break;
		}
		for ( int lBoxNumber = 0; lBoxNumber < MAX_NUMBER_BOXES; lBoxNumber++)
		{
			OverlayROIs(lImageDataDisplay_8BitsPerPixel.data, lImageDataDisplay_8BitsPerPixel.cols,lImageDataDisplay_8BitsPerPixel.rows,Boxes[lBoxNumber]);
			OverlayROIs_16BPP(lImageDataDisplay_12BitsPerPixel.data, lImageDataDisplay_12BitsPerPixel.cols,lImageDataDisplay_12BitsPerPixel.rows,Boxes[lBoxNumber]);
			
		}
		// Update display window with new image
		cv::circle(lImageDataDisplay_8BitsPerPixel,cv::Point(640,480),8,CV_RGB(250,120,120),3,4);
		//switch(camerabrand)
		//{
		//case APTI:
		//imshow( "DisplayWindow", lImageDataDisplay_8BitsPerPixel );
		//break;
		//case OMNI:
		imshow( "DisplayWindow", lImageDataDisplay_8BitsPerPixel );
		//break;
		//}

	}  // End if (Display)

	if ( ( modeflags.num_frames && ((modeflags.WriteImageLevel == 2)||(modeflags.WriteImageLevel == 3)) ) ||
		( ((modeflags.WriteImageLevel < 2) && (modeflags.WriteImageLevel > 3)) && (lKey == NO_KEY_PRESSED)) )
	{
		cout <<"Data"<<" ";
		cout <<"  X  "<<" ";
		cout <<"  Y  "<<" ";
		for (int k = 0; k < MAX_NUMBER_BOXES; k++)
			cout <<"Box"<<k<< " ";
		cout<<endl;

		for (int lBoxes = 0; lBoxes < MAX_NUMBER_BOXES; lBoxes++)
		{
			cout <<Boxes[lBoxes].box_ctr.x <<"   ";			
		}
		cout<<endl;
		for (int lBoxes = 0; lBoxes < MAX_NUMBER_BOXES; lBoxes++)
		{
			cout <<Boxes[lBoxes].box_ctr.y <<"   ";
		}
		cout<<endl;

		for ( int lRoi = 0; lRoi< ROI_PER_SLANTBOX; lRoi++)
		{
			for (int lBoxes = 0; lBoxes < MAX_NUMBER_BOXES; lBoxes++)
			{
				cout <<Boxes[lBoxes].SFRBox[lRoi].SFRValue <<"   ";
			}
			cout<<endl;
		}

		std::ofstream MyFile;
		MyFile.open ("DataSheet.csv", ios::out | ios::ate | ios::app | ios::binary);
		if (!Once)
		{
			MyFile << "Version,Time,Demosaic Appraoch, Detection Approach, Running Mode, Saturation, Spat. Freq.,FuseID,X,Y,ROI0,ROI1,ROI2,ROI3,X,Y,ROI4,ROI5,ROI6,ROI7,X,Y,ROI8,ROI9,ROI10,ROI11,X,Y,ROI12,ROI13,ROI14,ROI15,X,Y,ROI16,ROI17,ROI18,ROI19,X,Y,ROI20,ROI21,ROI22,ROI23,X,Y,ROI24,ROI25,ROI26,ROI27,X,Y,ROI28,ROI29,ROI30,ROI31,X,Y,ROI32,ROI33,ROI34,ROI35,X,Y,ROI36,ROI37,ROI38,ROI39,X,Y,ROI40,ROI41,ROI42,ROI43" << endl;
			Once = true;
		}
		//MyFile <<szChar_D<<",";
		MyFile <<VERSION<<"."<< RELEASE << "." << BUILD<<",";
		
		now = time(NULL);
		p = localtime(&now);
		strftime(s, 100, "%A %B:%d:%Y", p);

		MyFile << s << ",";
		switch (modeflags.Demosaic)
		{
			case 0:
				MyFile <<"Row/Col removal,";
				break;
			case 1:
				MyFile <<"Modified Gradient,";
				break;
		}
		switch (modeflags.lTargetDetection)
		{
			case 1:MyFile <<"Template Match,";
				break;
			case 2: MyFile <<"Circle Detection,";
				break;
			case 3: MyFile <<"Edge Detection,";
				break;
		}
		switch (modeflags.lRunningMode)
		{
			case 1:MyFile <<"HDR,";
				break;
			case 2: MyFile <<"Linear,";
				break;
		}
		// Saturation Status
		MyFile << lSaturationLog<< ',';
		// Report Spatial Freq.
		MyFile << modeflags.SpatialFreq << ",";
		// Fuse ID's
		unsigned int combined = (modeflags.FuseID[3] << 16) & 0xffff0000 | (modeflags.FuseID[2] & 0x0000ffff);
		// Set format flag to hexidecimal and write out top 32 bits of Fuse ID.
		MyFile << hex << combined;
		combined = (modeflags.FuseID[1] << 16) & 0xffff0000 | (modeflags.FuseID[0] & 0x0000ffff);
		// Write bottom 32 bits of Fuse ID and switch format back to decimal
		MyFile << combined << ',' << dec;
		// Box locations and scores
		for (int lBoxes = 0; lBoxes < MAX_NUMBER_BOXES; lBoxes++)
		{
			MyFile << Boxes[lBoxes].box_ctr.x << "," << Boxes[lBoxes].box_ctr.y << ",";
			for ( int lRoi = 0; lRoi< ROI_PER_SLANTBOX; lRoi++)
			{
					MyFile <<Boxes[lBoxes].SFRBox[lRoi].SFRValue << ",";
			}
		}
		MyFile << endl;
		MyFile.close();
	}
	// Deallocate dynamic memory??
	deallocate_dynamic_matrix(dataArray_Hi_12BitsPerPixel->matrix, dataArray_Hi_12BitsPerPixel->row);
	delete dataArray_Hi_12BitsPerPixel;
} // End live scoring section



int main(int argc, char* argv[])
{
    AP_HANDLE       hCamera;
	//int             camerabrand = -1;
    unsigned long   frameLength;						//size of the frames from the camera
    unsigned char   *pBuffer=NULL;						//grabFrame buffer
    size_t          nBufferSize;						//size of the buffer
    unsigned int    num_frames = 0;						//number of frames to capture
    ap_u32          nWidth  = DEFAULT_IMAGE_WIDTH;		//width of image (taken from command line or set to default below)
    ap_u32          nHeight = DEFAULT_IMAGE_HEIGHT;		//height of image (taken from command line or set to default below)
    FILE            *imfile;							//capture file
    char            imagetypestr[64];
	flags_t			modeflags;
	m_FrameCounter = 0;

	// Check to see if we have any arguments (required)
	if (argc==1) {
		printusage();
		DelayExit(1);
	}

	// Initial Image Library
	ilInit();

 	int lCounter =0;
	// Find the last instance of a path delimiter
	for (int iEx = 0; iEx < (int)strlen(argv[0]); iEx++)
	{
		if (argv[0][iEx] == '\\') { lCounter = iEx; }
	}
	// Did we find a path delimiter?  If not, path was defaulted to '.\'
	if ( lCounter ) {
		for (int iEx = 0; iEx <= lCounter; iEx++) { szCharWithoutExePath[iEx] = argv[0][iEx];	}
		// Close the string with a null character
		szCharWithoutExePath[lCounter] = '\0';
	} 
	// Announce the found path
	Debug_Report("Execution Path: ");

	///////////////////////////////////////////////////////////////////////
	// C O M M A N D   L I N E   A R G U M E N T   P A R S I N G 
	///////////////////////////////////////////////////////////////////////
	ProcessArgs(argc, argv, modeflags);

	// If we're using template matching algorithm, attempt to load the templates
	if (modeflags.lTargetDetection==1 && !InitTemplates(szCharWithoutExePath))
	{
		cout << "Error, can't read the fidducial templete" << endl; 
		DelayExit(1);
	}

	// Is last argument a filename and not a flag?
	if ( strcmp(argv[argc-1], "live") ) {
		Score_Image(argc-1, argv, modeflags);
	} else {
		// Has user requested no overlay?
		if ( GetCmdLineArgument( argc, (const char **)argv, "-nooverlay" ) > 0 ) { Display = false; }
		else { Display = true; }

#ifndef NO_CAMERA
		// Test to if we have Aptina camera available
		if (ap_DeviceProbe(NULL)) {
			printf("Could not find Aptina Probe error %d.\n", ap_GetLastError() );
		//	DelayExit(1);
		} else if (!ap_NumCameras()) {
			printf("Could not find any Aptina camera.\n" );
		//	DelayExit(1);
		} else {
			//  Just take the first device
			hCamera = ap_Create(0);
			// Verify we have a valid camera handle
			if (!hCamera)
			{
				printf("Could not get handle to Aptina camera (Err #: %d). \n",ap_GetLastError());
				ap_Finalize();
			//	DelayExit(1);
			}
			else
			{
				printf("Found Aptina camera. \n");
				camerabrand = APTI;
			}
		} // End Aptina camera initiliazation

		//Test to see if we have Omnivision camera available
		if(OVCam_ConnectionCheck() !=1 )
		{// did not find OmniVision camera
		    printf("Could not find OmniVision camera. \n");
		}
		else
		{
			printf("Found OmniVision camera, will proceed with OmniVision camera. \n");
			camerabrand = OMNI;
		}


		if(camerabrand < 0)
		{// could not find Aptina or OmniVision camera
		    printf("Could not find Aptina or OmniVision camera. \n");
			DelayExit(1);
		}

		switch(camerabrand)
		{
		case APTI:
		//  Application functions that scripts may use
//		ap_SetCallback_MultipleChoice(hCamera, MultipleChoice, NULL);
		ap_SetCallback_LogComment(hCamera, MyLogComment, NULL);
		ap_SetCallback_ErrorMessage(hCamera, MyErrorMessage, NULL);
		ap_SetCallback_ScriptOutput(hCamera, MyScriptOutput, NULL);

		//  Load default initialization preset [Demo Initialization] in default ini file
		ap_LoadIniPreset(hCamera, NULL, NULL);
		ap_CheckSensorState(hCamera, 0);

		//  Command line dimensions override
		if (nWidth && nHeight) { ap_SetImageFormat(hCamera, nWidth, nHeight, NULL); }

		//display some stats
		ap_GetImageFormat(hCamera, &nWidth, &nHeight, imagetypestr, sizeof(imagetypestr));

		// Make Fuse ID registers readable
		ap_SetSensorRegister( hCamera, "RESET_REGISTER", "REG_RD_EN", 1, NULL);
		// Fetch the registers
		ap_GetSensorRegister( hCamera, "FUSE_ID1", NULL, modeflags.FuseID, false);	// 0x31F4 for AR0132
		ap_GetSensorRegister( hCamera, "FUSE_ID2", NULL, modeflags.FuseID+1, false);	// 0x31F6 for AR0132
		ap_GetSensorRegister( hCamera, "FUSE_ID3", NULL, modeflags.FuseID+2, false);	// 0x31F8 for AR0132
		ap_GetSensorRegister( hCamera, "FUSE_ID4", NULL, modeflags.FuseID+3, false);	// 0x31FA for AR0132
		// Report Imager Fuse ID
		printf( "Imager Fuse ID: %04x %04x %04x %04x\n",
				modeflags.FuseID[3], modeflags.FuseID[2], modeflags.FuseID[1], modeflags.FuseID[0] ) ;

		//Allocate a buffer to store the images
		nBufferSize = ap_GrabFrame(hCamera, NULL, 0);
#else
		nBufferSize = 1280 * 964 *2;
#endif
		pBuffer = new unsigned char[nBufferSize];
//		pBuffer  = (unsigned char *)malloc(nBufferSize);
		if (pBuffer == NULL)
		{
			printf("Error trying to create a buffer of size %lu to grab the frames.\n", 
				(unsigned long)nBufferSize );
#ifndef NO_CAMERA
			ap_Destroy(hCamera);
#endif
			ap_Finalize();
			DelayExit(1);
		}

		//  For informational purposes, enable log for SHIP commands during ap_LoadIniPreset
		ap_OpenIoLog(AP_LOG_SHIP, "ship_log_.txt");
		printf("Log file: %s \n", ap_GetIoLogFilename());

		Debug_Report("Saving frames to img.raw...");
		imfile = fopen("img.raw","wb");

		// Open Display once we have the necessary items obtained
		if (Display) { cv::namedWindow( "DisplayWindow", cv::WINDOW_NORMAL); }
		break;
		case OMNI:
			nWidth = 1280;
			nHeight = 960;
			//Initialize OmniVision Driver;
			if(InitOVDriver(nWidth,nHeight)<0)	
			{
				printf("Initialize OmniVision error. Please Restart the program. \n");
				DelayExit(1); 
			}

			nBufferSize = nWidth*nHeight*3;
			pBuffer = new unsigned char[nBufferSize];
            //Set all register values
			//SendRegFile("RegSetting.txt");
			if(modeflags.MIPI)
			SendRegFile("RegSettingMIPI.txt");
			else
				SendRegFile("RegSetting.txt");

			WriteRegister(0x102, 0xa1,0x1);
			WriteRegister(0xc8, 0x81,0x1);
			WriteRegister(0x102, 0xd0,0x1);
			//Sleep(3000);
			//Enable temperature sensor
			WriteRegister(0x60, 0x303a, 0x04);
			WriteRegister(0x60,0x303b,0x7f);
			WriteRegister(0x60,0x303c,0xfe);
			WriteRegister(0x60,0x303d,0x19);
			WriteRegister(0x60,0x303e,0xd7);
			WriteRegister(0x60,0x303f,0x09);
			WriteRegister(0x60,0x3040,0x78);
			WriteRegister(0x60,0x3042,0x05);


			ChangeResolution(nWidth,nHeight);
			ChangeSensorMode(0x67);
			//SendRegFile("RegSetting.txt");
			imfile = fopen("img.raw","wb");
			break;
		}

		// Loop forever
		while (true)
		{
			// Look for any key press
			lKey = cv::waitKey(2);
			// If user has pressed 'Esc', pause execution until another key is hit
			if (lKey == 0x1B) { cv::waitKey(0); }
			// Switch on key pressed
			switch (lKey)
			{
				// Save images/SFR scores
				case 's':
					// Add one frame to collection task list
					modeflags.num_frames++;
					printf("Adding 1 frame to queue\n");
					break;
				case 'c':
					modeflags.savelive = true;
					modeflags.num_frames++;
					printf("Adding 1 frame to queue and saving a PGM file onto hard drive\n");
					break;
				case 'S':
					// Add one frame to collection task list
					modeflags.num_frames += 10;
					printf("Adding 10 frames to queue\n");
					break;
				// Quit
				case 'q':
				case 'Q':
					// Clean up before exiting
					fclose(imfile);
				    switch(camerabrand){
					case APTI:
#ifndef NO_CAMERA
					ap_Destroy(hCamera);
#endif
					ap_Finalize();
					delete[] pBuffer;
					ap_CloseIoLog();
					exit(0);
					break;
					case OMNI:
						FinishDriver();
		                if(pBuffer!=NULL)
						{
							delete[] pBuffer;
						}
						exit(0);
						break;
					}
					break;
				// If nothing we're interested in is pressed...
				default:
					lKey = NO_KEY_PRESSED;
					break;
			}

			/*
				skip frames until a good frame is found.
			 */
			int count = 0;
	#if Check_Error_Frames
			do
			{
#ifndef NO_CAMERA
				switch(camerabrand)
				{case APTI:
					frameLength = ap_GrabFrame(hCamera, pBuffer, (ap_u32)nBufferSize);break;
				case OMNI:
				    AcquireImage(pBuffer);
					break;
				}
#endif
				if (ap_GetLastError() == AP_CAMERA_SUCCESS)
					break;
				printf("b");
			} while (count++ < MAX_BADFRAME_TRIES);
	#else
#ifndef NO_CAMERA
			switch(camerabrand)
				{case APTI:
					frameLength = ap_GrabFrame(hCamera, pBuffer, (ap_u32)nBufferSize);
					//printf("pBuffer[60000]= %d \n",pBuffer[60000]);
					break;
				case OMNI:
				    AcquireImage(pBuffer);

					//read sensor temperature
					//printf("Sensor temperature is %d degrees C \n",showtemp());
					
					//swap the endian of the pBuffer
					unsigned char temp;
					for(int i = 0;i<nBufferSize;i = i+2)
					{
						temp = pBuffer[i];
						pBuffer[i] = pBuffer[i+1];
						pBuffer[i+1] = temp;
					}
					//printf("pBuffer[60000]= %d \n",pBuffer[60000]);
					break;
				}
#endif
	#endif

	#if Check_Error_Frames
			if (ap_GetLastError() != AP_CAMERA_SUCCESS)
				printf("B (error code: %d)", ap_GetLastError());
			else
			{
				Score_Image(pBuffer, modeflags);
				fwrite(pBuffer, frameLength, 1, imfile);
			}
	#else
			Score_Image(pBuffer, modeflags);
	#endif
			// If we have collected frames in the queue, decrement
			if (modeflags.num_frames) { modeflags.num_frames--; }
			modeflags.savelive= false;
		
		} // end while()
		
	} // End if (live images)

	// Delay exit if in Debug; return success
	DelayExit(0);
}
