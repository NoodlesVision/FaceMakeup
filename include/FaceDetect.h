/**********************************************************************************************
 *
 * FaceDetect.h
 * 
 * DESCRIPTION:	This is a version of the original Viola-Jones face detector, reworked to incorporate
 *						new detector functionality, rotations and skin prefiltering. This is based on Casey 
 *						Miller's (Ft Collins Camera Division) adaption of the original code set for the camera platform. 
 * 
 * AUTHORS:			Darryl Greig & Casey Miller
 *
 * OWNER:			Darryl Greig HP Labs Bristol, darryl.greig@hp.com
 *
 * FEATURES:		- All the classes & functions use integer operations only, and do not allocate or delete 
 *						any memory internally. 
 *						- There are two FaceDetect classes provided: FaceDetect1 and FaceDetect3.
 *						Both classes have run functions for still images and video, with optional skin 
 *						prefiltering in all cases. In addition both classes can handle detectors with multiple
 *						cascades of different rotation class (i.e. differing inplane and out of plane rotations).
 *						- The Rotate functions enable using a source cascade (detector) to generate a sequence of 
 *						sequence of cascades (a new detector) with different in-plane rotations via
 *						90 degree rotations (only) and different out-of-plane rotations via reflections about the
 *						vertical axis. Thus given a detector that picks up left profile, upright faces, a new
 *						detector picking up both left and right profile faces, in 0, 90, 180 and 270 degree 
 *						rotations can be quickly created. 
 *						- Both the FaceDetect classes expect input in Yxx format, where xx can be empty (greyscale
 *						image) or CbCr (that is YCbCr, 444 format). Obviously skin detection is only available
 *						when colour channels are present. 
 *						- The difference between FaceDetect1 and FaceDetect3 is purely to do with the options for
 *						handling in-plane rotations:
 *
 *							FaceDetect1:	applies all the views of the specified detector to the upright input
 *												image only. Thus each desired output view must be explicity catered
 *												for in the specified detector. The advantage of this is that there is 
 *												significantly less internal memory and image manipulation required than
 *												in the FaceDetect3 case. The disadvantage is that the detection quality
 *												of the Viola-Jones detectors for in-plane rotations that are not on the
 *												90 degree axes (e.g. 30 degrees, 60 degrees) is significantly less than
 *												that of those detectors on the axes. Furthermore, separate detectors need
 *												to be trained for the in-plane rotations.
 *							FaceDetect3:	applies all the view of the specified detector to the upright, 30 and 330 
 *												rotated images. Thus any view supplied in the specified detector is 
 *												evaluated in both upright and +-30 degrees. The advantage of this is that
 *												the detector accuracy at all rotations should be comparable, also fewer
 *												views need to be trained. The disadvantage is that more runtime memory is
 *												needed to hold the extra images, also the extra image processing is likely 
 *												to slow this method down. 
 *
 * LAST MOD DATE: 27 November 2007
 *
 * MODS:				- Added Reflect function for cascades (24 Jan 06)
 *						- Added some macros to give the reflected classes (about vertical axis) for in and out of plane
 *						  rotations (9 Mar 06)
 *						- Included some #define contained code specifically for training rotation invariant prefilters
 *						  This should NOT be activated in the regular runtime, just in the training (12 June 06)
 *						- Added an #ifdef around some character types that change if you wish to use the EARLY_OPTOUT 
 *						  functionality. (14 June 06)
 *						- removed all dependency on standard libraries (stdio.h, memory.h) (11 Sept 06)
 *						- changed the SKINLUT so the dimension is taken from a #define (23 Oct 06)
 *						- a few minor changes for sensitive compilers (28 Feb 07)
 *						- Changed the SKINLUT struct to handle multiple pixel formats (YCC and LAB initially) (26 March 07)
 *						- Added support for the new face certainty functionality based on training stats (18 Apr 07)
 *						- Extended the new face certainty support to video (30 Apr 07)
 *						- Introduced functions for in-place reflection and rotation of cascades (2nd May 07)
 *						- Pulled the bit that computes stagger parameters in RunVideo commands into a separate function & debugged (30 May 07)
 *						- Introduced an inline Reflect function for in-place reflection of a single feature (8 August 07)
 *						- Added an inline Rotate function for in-place rotation of a single feature (23 August 07)
 *						- added new Evaluate(STAGE*,..) overload (16 Oct 07)
 *						- Removed all references to the old sensitivity function (20 Nov 07)
 *						- Removed quite a bit of dead code (27 Nov 07)
 *
 * (C) Copyright 2006, Hewlett-Packard Ltd, all rights reserved.
 *
 ************************************************************************************************/


#ifndef FACE_DETECT_H
#define FACE_DETECT_H

// The following defines are necessary when compiling for WIN32
   #define INT8   char
   #define UINT8  unsigned char
   #define INT16  short
   #define UINT16 unsigned short
   #define INT32  int
   #define UINT32 unsigned int   
	#ifndef VERIFY  
		#define VERIFY(x) { }
	#endif
   #include <time.h>

   inline UINT32 BinfTickGet() { return (UINT32)clock();}
   inline UINT32 BinfTicksPerSec() { return CLOCKS_PER_SEC; }

#ifndef MAX
#define MAX(A,B) ((A) > (B)) ?  (A) : (B)
#endif
#ifndef MIN
#define MIN(A,B) ((A) < (B)) ?  (A) : (B)
#endif
#define ABS(A)   ((A) <  0   ? -(A) : (A))

#define QBITS 10                                   // fixed-point fractional bits
#define HALF (1 << (QBITS-1))                      // value of 0.5 
#define MULT_FP(x, y) (((x)*(y) + HALF) >> QBITS)  // fixed-point multiply

#define MAXDETECTIONS            300
#define STEP_SIZE       (UINT32)(2.00 * (1 << QBITS))
#define SCALE_FACTOR    (UINT32)(1.25 * (1 << QBITS))
#define SCALE_FACTOR_2	(UINT32)(1.5625 * (1 << QBITS))	// for variance features, need the scale^2
#define FACE_SIDE       (UINT32)(24)
#define GRID_OFFSET					 5					  // number of pixels on the image border to miss out
#define MIN_FACE_SIDE	(UINT32)(24)
#define POSE_DETECTOR_DEPTH 30
#define MAX_VIEWS 60


#define		HORIZONTAL_EDGE			0x000
#define		VERTICAL_EDGE				0x001
#define		HORIZONTAL_LINE			0x002
#define		VERTICAL_LINE				0x003
#define		CHECKERBOARD				0x004
#define		CENTREPOINT					0x005

#define		WIDE_HORIZONTAL_LINE		0x006	
#define		WIDE_VERTICAL_LINE		0x007
#define		VERTICAL_LOWER_STEP		0x008
#define		VERTICAL_UPPER_STEP		0x009
#define		HORIZONTAL_UPPER_STEP	0x00a
#define		HORIZONTAL_LOWER_STEP	0x00b

#ifdef _FD_TRAIN_
#define		ANNULUS						0x00c	// this is a different definition: {tlx,tly,w,h} == {tlx,tlx2,w1,w2}
#endif // _FD_TRAIN_

#define		FEATURE_BASE_MASK			0x00f // all features have a base type between 0 and 15


//////////////////////////////////////////////////////////////////////////////////////////////
// rotation states
enum {
	// NOT A FACE
	NOTFACE				=0x0,

	// DEACTIVATED_FACE
	DEACTIVATED			=0x0000001,

	// IN_PLANE_ROTATIONS
	IPR_0					=0x0000002,
	IPR_30				=0x0000004,	// IPR_0  <<1
	IPR_60				=0x0000008,	// IPR_30 <<1
	IPR_90				=0x0000010,
	IPR_120				=0x0000020,	
	IPR_150				=0x0000040,
	IPR_180				=0x0000080,
	IPR_210				=0x0000100,	
	IPR_240				=0x0000200,
	IPR_270				=0x0000400,
	IPR_300				=0x0000800,	
	IPR_330				=0x0001000,	

	// HORIZ_OUT_OF_PLANE_ROTATIONS
	HOOP_MINUS_90		=0x0002000,
	HOOP_MINUS_60		=0x0004000,
	HOOP_MINUS_30		=0x0008000,
	HOOP_0				=0x0010000,
	HOOP_PLUS_30		=0x0020000,
	HOOP_PLUS_60		=0x0040000,
	HOOP_PLUS_90		=0x0080000,

	// VERT_OUT_OF_PLANE_ROTATIONS
	VOOP_MINUS_90		=0x0100000,
	VOOP_MINUS_60		=0x0200000,
	VOOP_MINUS_30		=0x0400000,
	VOOP_0				=0x0800000,
	VOOP_PLUS_30		=0x1000000,
	VOOP_PLUS_60		=0x2000000,
	VOOP_PLUS_90		=0x4000000,

	// flag for (in plane) rotation invariant classifier 
	IPR_INVARIANT		=0x8000000
};


#define NUM_IPR			12
#define NUM_HOOP		7
#define NUM_VOOP		7
// NUM_ROTATIONS is NUM_IPR*NUM_HOOP*NUM_VOOP + 2 (NONOBJECT & DEACTIVATED)
#define NUM_ROTATIONS	590 

// rotation state functions
#define IN_PLANE_ROTATIONS				(IPR_0|IPR_30|IPR_60|IPR_90|IPR_120|IPR_150|IPR_180|IPR_210|IPR_240|IPR_270|IPR_300|IPR_330)
#define HORIZ_OUT_OF_PLANE_ROTATIONS	(HOOP_MINUS_90|HOOP_MINUS_60|HOOP_MINUS_30|HOOP_0|HOOP_PLUS_30|HOOP_PLUS_60|HOOP_PLUS_90)
#define VERT_OUT_OF_PLANE_ROTATIONS		(VOOP_MINUS_90|VOOP_MINUS_60|VOOP_MINUS_30|VOOP_0|VOOP_PLUS_30|VOOP_PLUS_60|VOOP_PLUS_90)
#define OUT_OF_PLANE_ROTATIONS			(HORIZ_OUT_OF_PLANE_ROTATIONS|VERT_OUT_OF_PLANE_ROTATIONS)

#define IS_SYMMETRIC(type)	((type & HORIZ_OUT_OF_PLANE_ROTATIONS) == HOOP_0)

#define FACE	(IN_PLANE_ROTATIONS|HORIZ_OUT_OF_PLANE_ROTATIONS|VERT_OUT_OF_PLANE_ROTATIONS|DEACTIVATED)

/*
// got this from http://graphics.stanford.edu/~seander/bithacks.html
const unsigned int b[] = {0xAAAAAAAA, 0xCCCCCCCC, 0xF0F0F0F0, 0xFF00FF00, 
			  0xFFFF0000};
register unsigned int c = (v & b[0]) != 0;
for (i = 4; i > 0; i--) // unroll for speed...
{
  c |= ((v & b[i]) != 0) << i;
}*/

// return logbase2 of the given integer
#define LOGBASE2(v) (((v & 0xAAAAAAAA) != 0) | (((v & 0xCCCCCCCC) != 0) << 1) | (((v & 0xF0F0F0F0) != 0) << 2) | (((v & 0xFF00FF00) != 0) << 3) | (((v & 0xFFFF0000) != 0) << 4))

// return the rotation state corresponding to degrees supplied
#define IPR_CLASS(ipr)		(IPR_0 << ((ipr / 30) % 12))
#define HOOP_CLASS(hoop)	(HOOP_MINUS_90 << (((hoop / 30) + 3) % 7))
#define VOOP_CLASS(voop)	(VOOP_MINUS_90 << (((voop / 30) + 3) % 7))

// return the rotation degrees corresponding to the rotation state supplied
#define IPR_DEGREES(ipr_class) (30 * (LOGBASE2((ipr_class/IPR_0))))
#define HOOP_DEGREES(hoop_class) (30 * (LOGBASE2((hoop_class/HOOP_MINUS_90))) - 90)
#define VOOP_DEGREES(voop_class) (30 * (LOGBASE2((voop_class/VOOP_MINUS_90))) - 90)

// return the reflected classes (reflection about the horizontal axis)
#define REFLECTED_IPR_CLASS(ipr_class)		((ipr_class==IPR_0)?IPR_0:(IPR_CLASS((360-(IPR_DEGREES(ipr_class))))))
#define REFLECTED_HOOP_CLASS(hoop_class)	(HOOP_CLASS(-(HOOP_DEGREES(hoop_class))))
#define REFLECTED_VOOP_CLASS(voop_class)	(voop_class)

/////////////////////////////////////////////////////////////////////////////
// Box structure & related routines
//
// NOTE: The destructor of BoxList *does* delete the entry array. This facility
//			is not ever used in the FaceDetect files - this memory should be handled
//			carefully in any application using these classes.

typedef struct {
	int tlx, tly; // top left coordinates
	int brx, bry; // bottom right coordinates
	int rotation_state;	  // rotation state of face this box points to	
	UINT8 confidence;			// confidence (0-255) that this box is a face	
} Box;


// Array of boxes class
class BoxList
{
public:
   BoxList() { length = 0; entry = NULL; };
   ~BoxList() { if ( entry != NULL ) delete [] entry; };

   int length;  // number of BoxListEntry elements
	Box *entry;  // pointer to BoxListEntry array
};

bool BoxesOverlap(Box box1, Box box2);

int MedianBoxes(Box *box_cluster, int boxes_in_cluster, Box *final_box);

int AverageBoxes(Box *box_cluster, int boxes_in_cluster, Box *final_box);

// test all the boxes in the array and combine overlapping boxes using average or median 
int CombineBoxes(Box *boxes,				// the combined boxes are written back into this array
					  int num_detections,
					  bool use_median);

// End of Box structure & related routines
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Skin prefilter structure
// 
// The SKINLUT is a fixed size structure based on the assumption that a LUT entry
// is given for all 256x256 (shifted) CbCr values (1 byte per channel). 

#define SKINLUT_DIM 256
#define SKINLUT_SHIFT 0

typedef enum {
	YCC_PIX,
	LAB_PIX,
	LHC_PIX
} pix_fmt;

struct SKINLUT {
	bool		*lut[SKINLUT_DIM];
	bool		lut_ptr[SKINLUT_DIM * SKINLUT_DIM];
	int		threshold;
	UINT8		minLum;
	UINT8		maxLum;
	pix_fmt  fmt;
	SKINLUT() {
		int n;
		threshold = -1; 
		int ptr_idx = 0;
		fmt = YCC_PIX;
		minLum = 0;
		maxLum = 255;
		for ( n=0; n < SKINLUT_DIM; ++n ) {
			// index into the array block
			lut[n] = (lut_ptr+ptr_idx);
			ptr_idx += SKINLUT_DIM;
		}
		
		// set everything to "false"
		for (n=0; n < SKINLUT_DIM * SKINLUT_DIM; ++n ) lut_ptr[n] = false;
		
	};
};


// End of skin prefilter structure
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Integral image class & related routines
// 
// This is an internal structure for holding an integral image (every location contains
// the sum of all values in the original image rectangle to the top and left of the location).
// This class requires a user supplied block of external memory. 

class IImage
{
public:

	IImage();
   IImage(int w, int h, UINT8 *pBuf, UINT32 bufferSizeInBytes);

   ~IImage() {}; // no memory allocated in this class

	bool Reset(int w, int h, UINT8 *pBuf, UINT32 bufferSizeInBytes);

	static int GetBufSize(int w, int h);

   INT32 **ptr;
   int w;
   int h;
};

// compute the integral and squared integral image from a single data array in the format
// Y[0,0],+,Y[0,1],+,...,Y[0,width-1],+,x,Y[1,0],+,...,Y[height-1,width-1],+,x where the "+" is
// bytesPerCol-1 extra bytes per location to be skipped, and "x" is bytesPerRow-bytesPerCol*width 
// extra bytes per row to be skipped.
void ComputeIntegralImages(UINT8 *pData, 
									int bytesPerCol, 
									int bytesPerRow,
									int width,
									int height,
									IImage *iimg,
									IImage *iimg_sq
									);

// compute the integral, squared integral image and skin integral image from a single data array with three bytes per column in the format
// Lum[0,0],C1[0,0],C2[0,0],Lum[0,1],C1[0,1],C2[0,1],...,Lum[0,width-1],C1[0,width-1],C2[0,width-1],x,Lum[1,0],C1[1,0],C2[1,0],...,
// Lum[height-1,width-1],C1[height-1,width-1],C2[height-1,width-1],x where the "x" is bytesPerRow-bytesPerCol*width 
// extra bytes per row to be skipped. Lum,C1,C2 is either LAB or YCbCr. If the data array is two bytes per column, the format is
// YCbYCr (YCC422). 
void ComputeIntegralImages(UINT8 *pData, 
									int bytesPerCol, 
									int bytesPerRow,
									int width,
									int height,
									SKINLUT *skin,
									IImage *iimg_skin,
									IImage *iimg,
									IImage *iimg_sq
									);

// compute the rotated integral and squared integral image from pData, format is as above. 
// The rotation amount (degrees) *must* be one of 30 or 330 degrees.
void ComputeRotatedIntegralImages(UINT8 *pData, 
									int bytesPerCol, 
									int bytesPerRow,
									int width,
									int height,
									int degrees,
									IImage *riimg);

// compute both the upright and rotated integral images from pData, format is as above.
void ComputeIntegralImages(UINT8 *pData, 
									int bytesPerCol, 
									int bytesPerRow,
									int width,
									int height,
									IImage *iimg0,
									IImage *iimg30,
									IImage *iimg330,
									IImage *iimg0_sq
									);

// compute the upright, rotated and skin integral images from pData, format is as above
void ComputeIntegralImages(UINT8 *pData, 
									int bytesPerCol, 
									int bytesPerRow,
									int width,
									int height,
									SKINLUT *skin,
									IImage *iimg_skin,
									IImage *iimg0,
									IImage *iimg30,
									IImage *iimg330,
									IImage *iimg0_sq
									);

// compute the patch normalization constants (both for mean and variance features) using the upright integral image
void ComputeNormConstants(int yoff, 
								  int xoff, 
								  IImage* iimg,
								  IImage* iimg_sq,
								  int patch_side, 
								  int patch_mult, 
								  int scale,
								  int &meanX,
								  int &varX,
								  int &norm_const);


// compute the stagger parameters for video sampling
void ComputeStaggerParams(UINT8 frame_index, 
								  UINT8 stagger_level, 
								  int &stagger_step, 
								  int &vstep_off, 
								  int &hstep_off);


// End of integral image class & related routines
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Detector data structures and related routines


// this structure holds the geometric features used in the detector
typedef struct {
	INT16  tlx, tly;        //  4 bytes 
	UINT16 w, h;            //  4 bytes
	UINT8 type;             //  1 byte
#ifdef EARLY_OPTOUT
	// need 4 byte integers here for the early optout versions of the cascades
	INT32 threshold;        //  4 bytes
	INT32 alpha_p, alpha_n; //  8 bytes  
#else
	INT16 threshold;        //  2 bytes
	INT16 alpha_p, alpha_n; //  4 bytes  (signed 4.12 format)
#endif // EARLY_OPTOUT
} FEATURE;						// 17 bytes total

// the chaining feature takes as input the raw output of the previous stage
typedef struct {				
#ifdef EARLY_OPTOUT
	// need 4 byte integers here for the early optout versions of the cascades
	INT32 threshold;        //  4 bytes
	INT32 alpha_p, alpha_n; //  8 bytes 
#else
	INT16 threshold;        //  2 bytes
	INT16 alpha_p, alpha_n; //  4 bytes  (signed 4.12 format)
#endif // EARLY_OPTOUT
} CHAIN;							// 6 bytes total

// this struct is used to hold the stats for each stage, for the new face certainty map
typedef struct { 
	UINT8	start_stage;
	UINT8	*face_prior;
	/* UINT8 *nonface_prior = 255 - face_prior; */
	INT32	*face_mean;
	INT32	*face_sd;
	INT32	*nonface_mean;
	INT32	*nonface_sd;
} STAGE_STATS;

// a stage is a chaining feature together with a list of geometric features and an
// accompanying theta value to threshold the result
struct STAGE {
	int num_features;  // number of features in the stage
	int theta;         // final threshold value for this stage
	FEATURE *feature;  // pointer to array of num_features features
	CHAIN chain;		 // only use this if chaining is enabled
	STAGE() { num_features = 0; feature = NULL; };
	~STAGE() { if ( feature != NULL ) delete [] feature; };
};

// a cascade is a list of stages representing a structure for detecting a single view or rotation type.
// there is also data in this structure for adjusting the ROC point of this cascade
struct CASCADE {
	int num_stages;	// number of stages in the cascade
	bool chaining;		// cascade uses chaining features
	bool symmetric;	// cascade is a symmetric rotation type
	INT32 type;			// rotation type of this cascade
	STAGE *stage;		// pointer to array of num_stages stages
	CASCADE() { stage = NULL; num_stages = 0; };
	~CASCADE() { 
		if ( stage != NULL ) {
			// destructor of stage should handle this
			delete [] stage;
		}
	};

};

// a detector is a set of prefilters (thresholds for patch mean and variance, and a SKINLUT structure)
// together with a list of cascades representing the different views detectable by this detector.
struct DETECTOR {
	// array of cascades for different poses
	int num_cascades;
	CASCADE *cascade;

	// global prefilters - evaluated before any of the poses
	INT32 min_patch_mean;		// if >= patch mean, don't process
	INT32 max_patch_mean;		// if <= patch mean, don't process
	INT32 min_patch_variance;	// if >= patch variance, don't process
	INT32 max_patch_variance;	// if <= patch variance, don't process
	SKINLUT *skin;				// use this to create a skin integral image prefilter

	DETECTOR() { 
		num_cascades = 0; 
		cascade = NULL; 
		skin = NULL; 
		// put these parameters in so the default behaviour is sensible!
		min_patch_mean = 5;
		max_patch_mean = 250;
		min_patch_variance = 1;
		max_patch_variance = 10000; 
	};
	~DETECTOR() {
		if ( cascade != NULL ) delete [] cascade;
		if ( skin != NULL ) delete skin;
	};
};

// this function determines the amount of skin in the specified patch. If this amount exceeds the
// specified threshold (which is SCALED for the current patch_side), then the function returns 0,
// alternatively it returns the largest integer step possible before another patch must be evaluated.
// For example, if 25% skin is necessary for a success, and the current patch has no skin whatsoever,
// then any step that is less than 25% of the patch width will necessarily result in a patch that 
// cannot exceed the skin threshold. Therefore the step in this case is patch_side * 0.25. 
inline int steps_to_skin(int y, int x, int patch_side, int scaled_skin_threshold, IImage *skin_iimg)
{
	int sum      = skin_iimg->ptr[y -              1][x -              1] + 
						skin_iimg->ptr[y + patch_side - 1][x + patch_side - 1] -
						skin_iimg->ptr[y -              1][x + patch_side - 1] -
						skin_iimg->ptr[y + patch_side - 1][x -              1];

	if ( sum >= scaled_skin_threshold ) return 0; // this means a success - no steps to skin

	return (scaled_skin_threshold - sum) / patch_side; // this is the max steps possible ensuring no skin
};

// evaluate the skinlut on a single pixel
inline bool Evaluate(SKINLUT *skin, UINT8 Lum, UINT8 Color1, UINT8 Color2)
{
	return (Lum >= skin->minLum && Lum <= skin->maxLum && skin->lut[(Color1>>SKINLUT_SHIFT)][(Color2>>SKINLUT_SHIFT)]);
};

//////////////////////////////////
//	Evaluate methods
/////////////////////////////////

// evaluate all the views of a detector on the upright image only at the specified location - the predicted
// rotation state of the patch is returned
int Evaluate(DETECTOR *detector,
				 int yoff, 
				 int xoff, 
				 IImage *iimg, 
				 int norm_const);

// evaluate a detector on this patch with stage stats for each view to give a patch confidence
int Evaluate(DETECTOR* detector, 
				 int yoff, 
				 int xoff, 
				 IImage *iimg,
				 int norm_const,
				 STAGE_STATS *view_stage_stats,
				 UINT8 &confidence);

// evaluate all the views of a detector on the upright, 30 and 330 degree images at the specified locations - 
// the predicted rotation state of the patch is returned
int Evaluate(DETECTOR *detector,
				 int yoff0, 
				 int xoff0,
				 int yoff30,
				 int xoff30,
				 int yoff330,
				 int xoff330,
				 IImage *iimg0,
				 IImage *iimg30,
				 IImage *iimg330,
				 int norm_const
				 );

// evaluate a detector on this patch with stage stats for each view to give a patch confidence
int Evaluate(DETECTOR* detector, 
				 int yoff0, 
				 int xoff0,
				 int yoff30,
				 int xoff30,
				 int yoff330,
				 int xoff330,
				 IImage *iimg0,
				 IImage *iimg30,
				 IImage *iimg330,
				 int norm_const,
				 STAGE_STATS *view_stage_stats,
				 UINT8 &confidence
				 );

// evaluate the cascade at the specified location - the predicted state of the patch is returned
int Evaluate(CASCADE* cascade, 
				 int yoff, 
				 int xoff, 
				 IImage *iimg,
				 int norm_const);

// evaluate the cascade at the specified location using stage stats to generate patch confidence
int Evaluate(CASCADE* cascade, 
			 int yoff, 
			 int xoff, 
			 IImage *iimg,
			 int norm_const,
			 STAGE_STATS *stage_stats,
			 UINT8 &confidence
			 );


// evaluate a first stage / nonchaining at the specified location - the numerical evaluation of the stage is returned
int Evaluate(STAGE* stage, 
				 int yoff, 
				 int xoff, 
				 IImage *iimg,
				 int norm_const);

// evaluate the stage at the specified location - the numerical evaluation of the stage is returned
int Evaluate(STAGE* stage, 
				 int yoff, 
				 int xoff, 
				 IImage *iimg,
				 int norm_const,
				 int last_stage_val);

// evaluate the feature at the specified location - the numerical evaluation of the feature is returned
// NOTE: about 80% of the runtime is spent in this routine - it should be *heavily* optimized
inline int Evaluate(FEATURE* feature, 
				 int yoff, 
				 int xoff, 
				 IImage *iimg);

// scale all the features "detector" by shifted floating point amount scale (e.g. (1.625 * (1 << QBITS))).
// the corresponding patch_side must be supplied, and the resultant scaled detector is returned in "scaled_detector"
// which must be initialized as a (scaled) copy of detector.
void Scale(DETECTOR* detector,
			  DETECTOR* scaled_detector,
			  UINT32 scale,
			  int patch_side);


// scale all the features in "cascade" as for the detector routine above - scaled_cascade must be initialized as a
// (scaled) copy of cascade.
void Scale(CASCADE* cascade, 
			  CASCADE *scaled_cascade, 
			  UINT32 scale, 
			  int patch_side);

// for each cascade (view) in detector, generate 0,90,180,270 degree rotated versions in r_detector. r_detector must
// be initialized with the same parameters as detector, but with its cascade array modified as follows:
// detector->cascade = [view0,view1,view2,...]; r_detector->cascade = [view0,view0,view0,view0,view1,view1,view1,view1,...]
void Rotate(DETECTOR* detector,
				DETECTOR* r_detector,
				int patch_side);

// generate 0,90,180,270 degree rotated version of cascade in r_cascade. r_cascade must be initialized as a vector
// of the form [cascade,cascade,cascade,cascade]. Note that the first entry of r_cascade can be identical to cascade
// since the parameters are simply copied across.
void Rotate(CASCADE* cascade,
				CASCADE* r_cascade,
				int patch_side);

// in-place rotate
void Rotate(CASCADE* cascade, 
				int rotation,		// IPR_0, IPR_90, IPR_180, IPR_270
				int patch_side);

// in-place rotate
void Rotate(FEATURE* pFeature, 
				int rotation,		// IPR_0, IPR_90, IPR_180, IPR_270
				int patch_side);

// in-place reflect
void Reflect(CASCADE* cascade,
				 int patch_side);

inline void Reflect(FEATURE *pFeature, int patch_side);

// for each cascade (view) in detector, generate 0,90,180,270 degree rotated versions in r_detector. In addition if
// a view is not symmetric, also generate r0, r90,r180,r270 versions corresponding to reflections about the vertical
// axis followed by rotation by the specified degree. r_detector must be initialized with the same parameters as 
// detector, but with its cascade array modified as follows: detector->cascade = [view0,view1,view2,...]; 
// r_detector->cascade = [view0,view0,view0,view0,[symmetric?,view0,view0,view0,view0],view1,view1,view1,view1,
// [symmetric?,view1,view1,view1,view1]...]
void RotateReflect(DETECTOR* detector,
						 DETECTOR* r_detector,
						 int patch_side);

// generate the 0,90,180,270,r0,r90,r180,r270 versions of cascade in r_cascade (without regard to the symmetry of 
// cascade or otherwise). r_cascade should be a vector containing 8 copies of cascade.
void RotateReflect(CASCADE* cascade,
						 CASCADE* r_cascade,
						 int patch_side);

// End of detector data structures and related routines
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// FaceDetect classes for running face detectors on images

// class for running a detector on the 0 degree rotation of the input image only
class FaceDetect1 {
public:
	FaceDetect1();
	~FaceDetect1();

	// this function returns the size of the memory block required by Reset
	static int GetBufSize(int width, int height, bool use_skin=false);

	// set the minimum size (in pixels) of faces to be looked for, must be >= FACE_SIDE == 24
	void SetMinFaceSide(UINT32 min_face_side) { m_min_face_side = MAX(FACE_SIDE,min_face_side); };

	// call this function whenever a detector is to be run on a new image - the parameters of the
	// image (width, height, byterPerCol, bytesPerRow) must be supplied, together with a buffer of
	// size at least that determined by the GetBufSize function. If skin detection is required, use_skin
	// must be set to true. 
	void Reset(
		int width, 
		int height, 
		UINT8* pBuffer, 
		int bufferSize,
		int bytesPerCol,
		int bytesPerRow,
		bool use_skin=false
		);

	// Run: run the detector supplied on the image data pData. 
	// INPUTS:
	//		-detector is a detector with all the views to be evaluated
	//		-scaled_detector is a copy of detector that is used as a memory bucket for generating the different 
	//		detector scales. Its structure and size at conclusion will be unchanged, but the internal parameter 
	//		values should be considered unknown.
	//		-the image data (pData) must be in the format
	//		Y[0,0],+,Y[0,1],+,...,Y[0,width-1],+,x,Y[1,0],+,...,Y[height-1,width-1],+,x where the "+" is
	//		bytesPerCol-1 extra bytes per location to be skipped, and "x" is bytesPerRow-bytesPerCol*width 
	//		extra bytes per row to be skipped. If skin detection is required, the "+" must correspond to Cb 
	//		and Cr channels of a single byte each (YCC 444 format). The parameters of this image are assumed
	//		to be those set by the Reset function. 
	//		- face_list is a structure with a *pre-allocated* list of Boxes of length MAXDETECTIONS. As it stands, the
	//		length argument to face_list should be set to the length of the allocated list. The returned list has
	//		some number of the boxes populated and the length parameter set to the number of the allocated boxes that
	//		has been populated. 
	//		- use_skin is a boolean indicating if skin detection is to be used or not. If skin detection hasn't  been
	//		set in the call to Reset this parameter will be ignored, also if the image supplied (pData) does not have
	//		3 bytes per pixel this parameter will be ignored. 
	// OUTPUTS:
	//		- the detected faces are recorded in boxList with the length parameter indicating the number of detections. 
	//		each Box entry also has the rotation state of the detected face.
	void Run(DETECTOR *detector, 
		DETECTOR *scaled_detector, 
		UINT8 *pData, 
		BoxList *face_list,
		bool use_skin=false);

	// Run: run the detector supplied on the image data pData. 
	// INPUTS:
	//		-detector is a detector with all the views to be evaluated
	//		-scaled_detector is a copy of detector that is used as a memory bucket for generating the different 
	//		detector scales. Its structure and size at conclusion will be unchanged, but the internal parameter 
	//		values should be considered unknown.
	//		-view_stage_stats is a vector of STAGE_STATS structs containing the stage statistics of each view cascade 
	//		for computing the face confidences.
	//		- view_start_stage is a vector of integers for each view cascade giving the stage at which we begin to 
	//		compute confidences for that view
	//		- min_certainty is a number between 0 & 255 giving the smallest face certainty allowed in the returned 
	//		BoxList.
	//		-the image data (pData) must be in the format
	//		Y[0,0],+,Y[0,1],+,...,Y[0,width-1],+,x,Y[1,0],+,...,Y[height-1,width-1],+,x where the "+" is
	//		bytesPerCol-1 extra bytes per location to be skipped, and "x" is bytesPerRow-bytesPerCol*width 
	//		extra bytes per row to be skipped. If skin detection is required, the "+" must correspond to Cb 
	//		and Cr channels of a single byte each (YCC 444 format). The parameters of this image are assumed
	//		to be those set by the Reset function. 
	//		- face_list is a structure with a *pre-allocated* list of Boxes of length MAXDETECTIONS. As it stands, the
	//		length argument to face_list should be set to the length of the allocated list. The returned list has
	//		some number of the boxes populated and the length parameter set to the number of the allocated boxes that
	//		has been populated. 
	//		- use_skin is a boolean indicating if skin detection is to be used or not. If skin detection hasn't  been
	//		set in the call to Reset this parameter will be ignored, also if the image supplied (pData) does not have
	//		3 bytes per pixel this parameter will be ignored. 
	// OUTPUTS:
	//		- the detected faces are recorded in boxList with the length parameter indicating the number of detections. 
	//		each Box entry also has the rotation state of the detected face.
	void Run(DETECTOR	*detector, 
		DETECTOR		*scaled_detector,
		STAGE_STATS		*view_stage_stats,
		UINT8			min_certainty,
		UINT8			*pData, 
		BoxList			*face_list,
		bool			use_skin = false);

	// RunVideo: run the detector supplied on the current frame (pData) of a video stream.
	// INPUTS:
	//		as in the Run function with the following additional arguments:
	//		- frame_index is the index of this frame in the list of frames
	//		- stagger_level is the coarseness of the staggered sampling grid, 0 means no staggered sampling, which is
	//		effectively the same as Run with a last_face_list, 1 means staggering on a 2x2 grid, 2 means staggering on 
	//		a 4x4 grid and 3 means staggering on a 6x6 grid. The higher the number the quicker the method should run,
	//		but with potentially less accuracy.
	//		- last_face_list is the list of faces detected on the previous frame. This structure should be *pre-allocated*
	//		in the same way as face_list.
	// OUTPUTS:
	//		as in the Run function

	void RunVideo(
		DETECTOR	*detector,
		DETECTOR	*scaled_detector,
		UINT8		*pData,
		UINT8		frame_index,			// index of this frame in the list of frames
		UINT8		stagger_level,			// amount of stagger in the staggered grid (0==none,1=2x2,2=4x4,3=6x6)
		BoxList		*last_face_list,		// list of faces from the last frame
		BoxList		*face_list,				// output list of face bounding boxes
		bool		use_skin=false);

	void RunVideo(
		DETECTOR	*detector, 
		DETECTOR	*scaled_detector,
		STAGE_STATS	*view_stage_stats,
		UINT8		min_certainty,
		UINT8		*pData, 
		UINT8		frame_index,			// index of this frame in the list of frames
		UINT8		stagger_level,			// amount of stagger in the staggered grid (0==none,1=2x2,2=4x4,3=6x6)
		BoxList		*last_face_list,		// list of faces from the last frame
		BoxList		*face_list,
		bool		use_skin = false);

protected:

	// this member variable is set by SetMinFaceSide
	UINT32		  m_min_face_side;	// minimum size of the faces sought

	//these member variables are (re)set by Reset
   int           m_width;				// width of input image
   int           m_height;				// height of input image
   int           m_bytesPerCol;		// byte offset to move 1 input column
   int           m_bytesPerRow;		// byte offset to move 1 input row
	IImage		  m_iimageSkin;		// skin integral image 
   IImage        m_iimage;				// upright integral image
   IImage        m_iimageSqr;			// upright squared integral image
};


// class for running a detector in the 0, 30 and 330 rotated images of the supplied image.
// NOTE: the functions, their arguments and return values are identical with those in FaceDetect1
class FaceDetect3 {
public:
	FaceDetect3();
	~FaceDetect3();

	static int GetBufSize(int width, int height, bool use_skin=false);

	void SetMinFaceSide(UINT32 min_face_side) { m_min_face_side = MAX(FACE_SIDE,min_face_side); };

	void Reset(
		int width,				// this is the width of the upright image
		int height,				// this is the height of the upright image
		UINT8* pBuffer, 
		int bufferSize,
		int bytesPerCol,
		int bytesPerRow,
		bool use_skin = false
		);

	void Run(DETECTOR *detector, 
		DETECTOR *scaled_detector, 
		UINT8 *pYData, 
		BoxList *boxList,
		bool use_skin=false);

	void Run(DETECTOR	*detector, 
		DETECTOR	*scaled_detector,
		STAGE_STATS *view_stage_stats,
		UINT8		min_certainty,
		UINT8		*pData, 
		BoxList		*face_list,
		bool		use_skin = false);

	void RunVideo(DETECTOR	*detector,
		DETECTOR		*scaled_detector,
		UINT8						*pData,
		UINT8						frame_index,			// index of this frame in the list of frames
		UINT8						stagger_level,			// amount of stagger in the staggered grid (0==none,1=2x2,2=4x4,3=6x6)
		BoxList					*last_face_list,		// list of faces from the last frame
		BoxList					*face_list,				// output list of face bounding boxes
		bool						use_skin=false);

	
	void RunVideo(DETECTOR		*detector, 
		DETECTOR		*scaled_detector,
		STAGE_STATS	*view_stage_stats,
		UINT8		min_certainty,
		UINT8		*pData, 
		UINT8		frame_index,			// index of this frame in the list of frames
		UINT8		stagger_level,			// amount of stagger in the staggered grid (0==none,1=2x2,2=4x4,3=6x6)
		BoxList		*lastBoxList,		// list of faces from the last frame
		BoxList		*face_list,
		bool			use_skin=false);

protected:

	// this member variable is set by SetMinFaceSide
	UINT32		  m_min_face_side;	// minimum size of the faces sought

   //these member variables are (re)set by Reset
	int           m_width0;				// width of input image
   int           m_height0;			// height of input image
   int           m_width30;			// width of input image (30 degree rotated)
   int           m_height30;			// height of input image (30 degree rotated)
   int           m_width330;			// width of input image (330 degree rotated)
   int           m_height330;			// height of input image (330 degree rotated)
   int           m_bytesPerCol;		// byte offset to move 1 input column
   int           m_bytesPerRow;		// byte offset to move 1 input row
	IImage		  m_iimageSkin;		// skin integral image 
   IImage        m_iimage0;			// upright integral image
   IImage        m_iimageSqr0;		// upright squared integral image
   IImage        m_iimage30;			// 30 degree integral image
   IImage        m_iimage330;			// 330 degree integral image
};

/*
// some test methods to do with image rotation
void Warp(unsigned char *imin,  // pointer to input data
                int hin, int win,     // height and width of input
					 int bytesPerCol,
					 int bytesPerRow,
					 int degrees,
                unsigned char *imout, // pointer to output data
                int hout, int wout    // height and width of output
					 );

#include "ImageUINT8.h"
ImageUINT8* Warp(ImageUINT8* imin, int degree);
*/

#endif
