/**********************************************************************
* File:				face_detect_lib.h: Header file for the face_detect_lib.dll library
* Description:		a DLL interface for face detection using the HPL multi-view face detector
*
* Author:			Darryl Greig
* Owner:				Darryl Greig, HP Labs Bristol, darryl.greig@hp.com
*
* External types:
*		FDP	- this is the address of a memory block that contains an instantiation of the struct FDC. 
*				By passing this address back and forth we can interface with languages like Java which do
*				not handle the instantiation of C++ classes.
*						
* Interface Functions:
*		
*		All interface functions return either an ok: FD_OK (=0x00), an error ERROR_CODE|FD_ERR(=0x01) or a warning: WARNING_CODE|FD_WARNING(=0x02).
*		The presence of an error can therefore be detected by (retval & FD_ERR) != 0, and a warning by (retval & FD_WARNING) != 0.
*
*		initFaceDetector				-   This must be called before any other (non-video) interface functions. The
*											value fd_out contains the address of the resultant FDC struct.
*											Returns FD_OK (=0x0) if OK, otherwise an error.
*		freeFaceDetector				-	This must be called when a particular FDP is finished with to free
*											allocated memory.
*		setFaceDetectorOptions			-	This is an optional function which may be called at any time to change
*											the runtime options of the initialized face detector. Using this function
*											skin detection can be dis/enabled, as can 30 and 90 degree rotations, 
*											portrait, halfprofile and profile detection. The minimal face dimension
*											can also be adjusted as can the minimum certainty of returned faces.
*		setFaceDetectorOptionsEx		-	The same as setFaceDetectorOptions except that  specific rotations (0,90,180,270) +- 30 degrees, 
*											can be enabled or not.
*		runFaceDetector					-	This function runs the initialized detector with the specified options on the
*											supplied image. The format of the image must be an 8-bit array organized in top-down row-wise format, 
*											where each pixel is represented by bpp consecutive bytes with pixel order / colorspace given by the 
*											pixType. Acceptable pixTypes are _GRAY_PIX_, _YCC_PIX_, _RGB_PIX_,_BGR_PIX_,_LAB_PIX_. The pix array is 
*											either processed as LAB or converted internally to 3 byte YCC - processing associated with the image handling 
*											can be minimized by supplying an image either in this format or in 1 byte Y format in which case no conversion 
*											takes place.
*		getFaceCount					-   This function relates to the last runFaceDetector call. The total number of faces
*											detected in that call is recorded in the num_faces pointer
*		getFace							-	This function relates to the last runFaceDetector call. The parameters of a detected face with index
*											between [0,...,n) (where n is the num_faces returned by getFaceCount) are filled into the output 
*											pointers.
*		getInvertedFace					-	This function is indentical to getFace except that the face returned is inverted - it is for working
*											with bottom-up images such as Windows bitmaps.
*		initVideoFaceDetector			-	This must be called before any other video interface functions. The value fd_out contains
*											the address of the resultant FDC struct with the video_input flag set to "true". Returns
*											FD_OK (=0x0) if OK, otherwise an error.
*		setVideoFaceDetectorOptions		-	This is an optional function which may be called at any time to change
*											the runtime options of the initialized video face detector. Using this function
*											skin detection can be dis/enabled, as can 30 and 90 degree rotations, 
*											portrait, halfprofile and profile detection. The minimal face dimension
*											can also be adjusted as can the detector sensitivity. Finally the frame stagger option can be
*											changed to speed up the video detection. I higher stagger number means faster processing, but
*											(possibly) more missed faces and longer face acquisition times. 0 = no stagger; 1 = 2x2 stagger;
*											2 = 4x4 stagger; 3 = 6x6 stagger.
*		setVideoFaceDetectorOptionsEx	-	The same as setVideoFaceDetectorOptions except that  specific rotations (0,90,180,270) +- 30 degrees, 
*											can be enabled or not.
*		runVideoFaceDetector			-	This function runs the initialized detector with the specified options on the
*											supplied video frame. The format of the image must be an 8-bit array organized in top-down row-wise format, 
*											where each pixel is represented by bpp consecutive bytes with pixel order / colorspace given by the 
*											pixType. Acceptable pixTypes are _GRAY_PIX_, _YCC_PIX_, _RGB_PIX_,_BGR_PIX_,_LAB_PIX_. The pix array is 
*											either processed as LAB or converted internally to 3 byte YCC - processing associated with the image handling 
*											can be minimized by supplying an image either in this format or in 1 byte Y format in which case no conversion 
*											takes place.
*		clearPersistentFaces			-	Clears the list of persistent faces - for video face detection when the list of persistent faces will be
*											known to be false (such as a pause).
*		getPersistentFaceCount			-	This function relates to the stored list of persistent faces, that is faces that have been detected at
*											any point in the last 1 sec. The total number of faces in the persistent list is recorded in the num_faces
*											pointer.
*		getPersistentFace				-	This function relates to the stored list of persistent faces. The parameters of a persistent faces with index
*											between [0,...,n) (where n is the num_faces returned by getPersistentFaceCount) are filled into the output
*											pointers.
*		getInvertedPersistentFace		-	This function is indentical to getPersistentFace except that the face returned is inverted - it is for working
*											with bottom-up images such as Windows bitmaps.
*		isVideoInput					-	Indicates if the detector has been initialized for video input or still image input.
*		setCropPercentages				-	Set the upper, left, right, lower crop boundaries for running the face detector. These are (floating point) percentages
*											and can be negative, values between -50 and 100.
*		setInvertedCropPercentages		-	Same as SetCropPercentages, except that the upper and lower percentages are exchanged to handle a bottom up image.
*		unsetCropPercentages			-	Reset all the crop percentages to 0.
*
* Last mod date: 28 November 2007
*
* Mods:	- Changed the linkage to remove C++ decorations, also changed some arguments and #defines to reflect the internal YCC transforms (17 Oct 06)
*			- Added functions and data structures for handling video input (20 Nov 06)
*			- Changed the image array to be of type unsigned char* rather than int* (21 Nov 06)
*			- INTERFACE CHANGE - I have added a confidence return value (between 0 & 255) to the get*Face routines, and changed the sensitivity 
*			input to min_certainty (between 0 & 100 - 0 means native running) (3 May 07)
*			- Added new interface functions to set more detailed options for the face detector (set*FaceDetectorOptionsEx), to get inverted faces
*			for handling bottom up images, and to clear the persistent face list (8 August 07)
*			- Added isVideoInput interface function (22 August 07)
*			- Added the set*CropPercentages functions (19 November 07)
*			- Added unsetCropPercentages to the API (28 Nov 07)
* 
* (C) Copyright 2006, Hewlett-Packard Ltd, all rights reserved.
*
**********************************************************************/

#ifndef FACE_DETECT_LIB
#define FACE_DETECT_LIB

#define WIN32_LEAN_AND_MEAN		// Exclude rarely-used stuff from Windows headers

#ifdef WIN32
#include <windows.h>

//#ifdef FACE_DETECT_LIB_EXPORTS
//#define LINKAGE        extern "C"
//#define FACE_DETECT_LIB_API __declspec(dllexport)
//#else
//#define LINKAGE
//#define FACE_DETECT_LIB_API __declspec(dllimport)
//#endif
//#else
//#define LINKAGE
//#define FACE_DETECT_LIB_API 
#endif

typedef void *FDP;			// this is used to pass instances of the face detector back and forth

// these are the return values of all the function calls
#define FD_OK							0x0000
#define FD_ERR							0x0001
#define FD_WARNING						0x0002
#define FD_OUT_OF_MEMORY				0x0004|FD_ERR
#define FD_UNINITIALIZED				0x0008|FD_ERR
#define FD_INCONS_OPTS					0x0010|FD_ERR
#define FD_NOIMAGE						0x0020|FD_ERR
#define FD_FACE_INDEX_OUT_OF_RANGE		0x0040|FD_ERR
#define FD_IMAGE_ERR					0x0080|FD_ERR
#define FD_UNSUPPORTED_IMAGE			0x0100|FD_ERR

// these are to flag the pixel order / type
#define _UNKNOWN_PIX_	0x00
#define _GRAY_PIX_		0x01
#define _YCC_PIX_		0x02
#define _RGB_PIX_		0x03
#define _BGR_PIX_		0x04
#define _LAB_PIX_		0x05

 int initFaceDetector(FDP *fd_out);					// [OUT] initialized FDP

 int freeFaceDetector(FDP fd);							// [IN] initialized FDP

 int	setFaceDetectorOptions(FDP fd,					// [IN] initialized FDP
														   int	use_skin,			// [IN] boolean flag to activate skin detection
														   int rot_30,				// [IN] boolean flag to activate 30 degree rotations around active 90 degree axes
														   int rot_90,				// [IN] boolean flag to activate all 4 90 degree axes
														   int no_portraits,		// [IN] boolean flag to disable portrait detection
														   int no_half_profiles,	// [IN] boolean flag to disable half-profile detection (currently does nothing)
														   int no_profiles,		// [IN] boolean flab to disable full profile detection
														   int	min_certainty,		// [IN] int value 0-100, 0 == native certainty (faster), 1->100 min->max certainty, native is about 96
														   int	min_face_dim);		// [IN] search for faces >= to this face size (>=24)

// this is the new API call which allows more flexibility in setting the options
 int	setFaceDetectorOptionsEx(FDP fd,				// [IN] initialized FDP
															 int use_skin,			// [IN] boolean flag to activate skin detection
															 int use_rot_30,		// [IN] boolean flag to activate 30 degree rotations around active 90 degree axes
															 int do_rot_0,			// [IN] boolean flag to activate 0 degree in plane rotations
															 int do_rot_90,			// [IN] boolean flag to activate 90 degree in plane rotations
															 int do_rot_180,		// [IN] boolean flag to activate 180 degree in plane rotations
															 int do_rot_270,		// [IN] boolean flag to activate 270 degree in plane rotations
															 int no_portraits,		// [IN] boolean flag to disable portrait detection
															 int no_half_profiles,	// [IN] boolean flag to disable half-profile detection (currently does nothing)
															 int no_profiles,		// [IN] boolean flab to disable full profile detection
															 int min_certainty,		// [IN] int value 0-100, 0 == native certainty (faster), 1->100 min->max certainty, native is about 96
															 int min_face_dim);		// [IN] search for faces >= to this face size (>=24)

// The input image data should be an 8-bit array organized in top-down row-wise format, where each pixel is represented by bpp consecutive bytes
// with pixel order / colorspace given by the pixType. Acceptable pixTypes are _GRAY_PIX_, _YCC_PIX_, _RGB_PIX_,_BGR_PIX_. The pix array is 
// converted internally to 3 byte YCC - processing associated with the image handling can be minimized by supplying an image either in this format
// or in 1 byte Y format.
//
 int runFaceDetector(FDP fd,								// [IN] initialized FDP
												 unsigned char* pix,							// [IN] pointer to the image data (should be recast as UINT8*)
												 int pixlen,						// [IN] length of pix in bytes
												 int bpp,							// [IN] bytes per pixel
												 int pixType,						// [IN] _GRAY_PIX_ (=0x01); _YCC_PIX_ (=0x02); _RGB_PIX_ (=0x03); _BGR_PIX_ (=0x04)
												 int width,							// [IN] width of the input image
												 int height);						// [IN] height of the input image

 int getFaceCount(FDP fd,									// [IN] initialized FDP
											  int *num_faces);						// [OUT] number of faces detected in last runFaceDetector call

 int getFace(FDP fd,										// [IN] initialized FDP
										 int face_index,								// [IN] index of face in list of detected faces (0<=face_index<num_faces)
										 int upscale_perc,							// [IN] number >= 100 giving percentage to upscale face dimensions
										 int *xpos,									// [OUT] horizontal centre of face in pixels
										 int *ypos,									// [OUT] vertical centre of face in pixels
										 int *radius,									// [OUT] radius of face in pixels
										 int *in_plane_rotation,					// [OUT] (in degrees), number between 0 & 359
										 int *horiz_out_of_plane_rotation,		// [OUT] (in degrees), number between -90 and 90
										 int *vert_out_of_plane_rotation,		// [OUT] (in degrees), number between -90 and 90
										 int *confidence);							// [OUT] confidence this is a face, between 0 & 255

// this is for bottom-up images (Win32 DIBs)
 int getInvertedFace(FDP fd,										// [IN] initialized FDP
												 int face_index,								// [IN] index of face in list of detected faces (0<=face_index<num_faces)
												 int upscale_perc,							// [IN] number >= 100 giving percentage to upscale face dimensions
												 int *xpos,									// [OUT] horizontal centre of face in pixels
												 int *ypos,									// [OUT] vertical centre of face in pixels
												 int *radius,									// [OUT] radius of face in pixels
												 int *in_plane_rotation,					// [OUT] (in degrees), number between 0 & 359
												 int *horiz_out_of_plane_rotation,		// [OUT] (in degrees), number between -90 and 90
												 int *vert_out_of_plane_rotation,		// [OUT] (in degrees), number between -90 and 90
												 int *confidence);							// [OUT] confidence this is a face, between 0 & 255


 int initVideoFaceDetector(FDP *fd_out);							// [OUT] initialized video FDP

 int	setVideoFaceDetectorOptions(FDP fd,							// [IN] initialized video FDP
																int	use_skin,				// [IN] boolean flag to activate skin detection
																int	use_rot_30,				// [IN] boolean flag to activate 30 degree rotations around active 90 degree axes
																int	use_rot_90,				// [IN] boolean flag to activate all 4 90 degree axes
																int	no_portraits,			// [IN] boolean flag to disable portrait detection
																int	no_half_profiles,		// [IN] boolean flag to disable half-profile detection (currently does nothing)
																int	no_profiles,			// [IN] boolean flab to disable full profile detection
																int	min_certainty,			// [IN] int value 0-100, 0 == native certainty (faster), 1->100 min->max certainty, native is about 96
																int	min_face_dim,			// [IN] search for faces >= to this face size (>=24)
																int	video_frame_stagger);// [IN] video stagger level (0=none; 1=2x2; 2=4x4; 3=6x6)

// this is the new API call which allows more flexibility in setting the options
 int	setVideoFaceDetectorOptionsEx(FDP fd,				// [IN] initialized FDP
																  int use_skin,			// [IN] boolean flag to activate skin detection
																  int use_rot_30,		// [IN] boolean flag to activate 30 degree rotations around active 90 degree axes
																  int do_rot_0,			// [IN] boolean flag to activate 0 degree in plane rotations
																  int do_rot_90,			// [IN] boolean flag to activate 90 degree in plane rotations
																  int do_rot_180,		// [IN] boolean flag to activate 180 degree in plane rotations
																  int do_rot_270,		// [IN] boolean flag to activate 270 degree in plane rotations
																  int no_portraits,		// [IN] boolean flag to disable portrait detection
																  int no_half_profiles,	// [IN] boolean flag to disable half-profile detection (currently does nothing)
																  int no_profiles,		// [IN] boolean flab to disable full profile detection
																  int min_certainty,		// [IN] int value 0-100, 0 == native certainty (faster), 1->100 min->max certainty, native is about 96
																  int min_face_dim,			// [IN] search for faces >= to this face size (>=24)
																  int video_frame_stagger);// [IN] video stagger level (0=none; 1=2x2; 2=4x4; 3=6x6)


 int runVideoFaceDetector(FDP fd,								// [IN] initialized video FDP
													  unsigned char* framepix,		// [IN] pointer to the image data (should be recast as UINT8*)
													  int pixlen,							// [IN] length of pix in bytes
													  int bpp,								// [IN] bytes per pixel
													  int pixType,						// [IN] _GRAY_PIX_ (=0x01); _YCC_PIX_ (=0x02); _RGB_PIX_ (=0x03); _BGR_PIX_ (=0x04)
													  int width,							// [IN] width of the input image
													  int height,							// [IN] height of the input image
													  int frameTime);					// [IN] frame time in msec

 int clearPersistentFaces(FDP fd);				// [IN] initialized video FDP

 int getPersistentFaceCount(FDP fd,				// [IN] initialized video FDP
														int *num_faces);	// [OUT] number of faces detected in the persistentFaces list

 int getPersistentFace(FDP fd,										// [IN] initialized video FDP
												   int face_index,							// [IN] index of face in list of persistent faces (0<=face_index<num_faces)
												   int upscale_perc,							// [IN] number >= 100 giving percentage to upscale face dimensions
												   int *xpos,									// [OUT] horizontal centre of face in pixels
												   int *ypos,									// [OUT] vertical centre of face in pixels
												   int *radius,								// [OUT] radius of face in pixels
												   int *in_plane_rotation,					// [OUT] (in degrees), number between 0 & 359
												   int *horiz_out_of_plane_rotation,	// [OUT] (in degrees), number between -180 and 180
												   int *vert_out_of_plane_rotation,		// [OUT] (in degrees), number between -180 and 180
												   int *face_ID,								// [OUT] unique ID for this face in the list, number between 0 & 299
												   int *time_since_last_detect,			// [OUT] (in msec), time since last detected this face
												   int *confidence);							// [OUT] confidence this is a face, between 0 & 255

 int getInvertedPersistentFace(FDP fd,										// [IN] initialized video FDP
														   int face_index,							// [IN] index of face in list of persistent faces (0<=face_index<num_faces)
														   int upscale_perc,							// [IN] number >= 100 giving percentage to upscale face dimensions
														   int *xpos,									// [OUT] horizontal centre of face in pixels
														   int *ypos,									// [OUT] vertical centre of face in pixels
														   int *radius,								// [OUT] radius of face in pixels
														   int *in_plane_rotation,					// [OUT] (in degrees), number between 0 & 359
														   int *horiz_out_of_plane_rotation,	// [OUT] (in degrees), number between -180 and 180
														   int *vert_out_of_plane_rotation,		// [OUT] (in degrees), number between -180 and 180
														   int *face_ID,								// [OUT] unique ID for this face in the list, number between 0 & 299
														   int *time_since_last_detect,			// [OUT] (in msec), time since last detected this face
														   int *confidence);							// [OUT] confidence this is a face, between 0 & 255

 int isVideoInput(FDP fd,			// [IN] initialized FDP
											 int *is_video);	// [OUT] ==0 if FDP is NOT video, nonzero otherwise

// define a crop rectangle as a percentage of the width and height of the image (can be negative too) for a bottom up image
 int	setCropPercentages(FDP fd,				// [IN] initialized FDP
												   float	top,		// [IN] crop percentage for top border [-50.0...100.0)
												   float	left,		// [IN] crop percentage for left border [-50.0...100.0)
												   float	bottom,		// [IN] crop percentage for bottom border [-50.0...100.0-top)
												   float	right);		// [IN] crop percentage for right border [-50.0...100.0-left)

// define a crop rectangle as a percentage of the width and height of the image (can be negative too) for a top down image
 int	setInvertedCropPercentages(FDP fd,				// [IN] initialized FDP
														   float	top,		// [IN] crop percentage for top border [-50.0...100.0)
														   float	left,		// [IN] crop percentage for left border [-50.0...100.0)
														   float	bottom,		// [IN] crop percentage for bottom border [-50.0...100.0-top)
														   float	right);		// [IN] crop percentage for right border [-50.0...100.0-left)

// reset all the crop percentages to 0
 int	unsetCropPercentages(FDP fd);					// [IN] initialized FDP

#endif // FACE_DETECT_LIB
