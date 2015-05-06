/**********************************************************************************************
 *
 * StaticDetector.h
 * 
 * DESCRIPTION:	This header and its associated .cpp file define a multi-view detector together with a 
 *						skin detector and other prefilter parametes. It also supplies Load and Copy routines 
 *						that load a detector (and a SKINLUT) into preallocated memory. There are two functions
 *						(StaticDetectorMemsize and StaticSkinMemsize) that return the required memory buffer size
 *						in bytes.
 * 
 * AUTHOR:			Darryl Greig
 *
 * OWNER:			Darryl Greig HP Labs Bristol, darryl.greig@hp.com
 *
 * FEATURES:		Using the methods here, the static detector and associated SKINLUT can be loaded into
 *						a block of memory without using any memory allocation. The size of the static detector
 *						is given by StaticDetectorMemsize, with the argument indicating if the static detector
 *						is to be loaded with all possible 90 degree rotations and reflections of assymetrical views.
 *						The size of the static SKINLUT is given by StaticSkinMemsize and *must be loaded explicitly*.
 *						Also a copy of a detector (with or without a skin member defined) may be made into preallocated
 *						memory using the Copy function defined here. Note that if the skin member is defined in the
 *						src detector, the associated bufferSize must be the size of the detector + the size of the skinlut.
 *
 * LAST MOD DATE: 27 November 2007
 *
 * MODS:				- Added hoop_views argument for loading the static detector (24 Jan 06)
 *						- Added a new Copy for copying a cascade using a static memory buffer (21 April 06)
 *						- a few minor changes for sensitive compilers (28 Feb 07)
 *						- Added an LAB skin detector and put in a pixel format switch for the skin detector (26 March 07)
 *						- Added support for the new face certainty functionality based on training stats (18 Apr 07)
 *						- Introduced functions for loading detectors for a predefined rotation set (2 May 07)
 *						- Removed quite a bit of dead code (27 Nov 07)
 *
 * (C) Copyright 2006, Hewlett-Packard Ltd, all rights reserved.
 *
 ************************************************************************************************/

#ifndef _STATICDETECTOR_H_
#define _STATICDETECTOR_H_

#include "FaceDetect.h"

// determine the memory size required to load the static detector
int StaticDetectorMemsize(int views);

// determine the memory size required to load the static skin lut
int StaticSkinMemsize();
// determine the memory size required to load the static stage statistic struct
int StaticStageStatsMemsize(int views);

// load the static detector into a pre-allocated memory buffer
DETECTOR*	LoadStaticDetector(const UINT8* buffer, int bufferSize, int views);

// copy a detector into a pre-allocated buffer
DETECTOR*	Copy(const DETECTOR &src, const UINT8* buffer, int bufferSize);
// copy a cascade into a pre-allocated buffer
int Copy(const CASCADE &src, CASCADE &dest, const UINT8* buffer, int bufferSize);
// load the static skin detector into a pre-allocated memory buffer
SKINLUT*	   LoadStaticSkin(const UINT8* buffer, int bufferSize, pix_fmt fmt);															
// load the static stage statistics into a pre-allocated memory buffer
STAGE_STATS* LoadStaticStageStats(const UINT8* buffer, int buffersize, DETECTOR *parent);


#endif //_STATICDETECTOR_H_
