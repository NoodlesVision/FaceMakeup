/** @file hog.h
 ** @brief Histogram of Oriented Gradients (@ref hog)
 ** @author Andrea Vedaldi
 **/

/*
 Copyright (C) 2007-12 Andrea Vedaldi and Brian Fulkerson.
 All rights reserved.

 This file is part of the VLFeat library and is made available under
 the terms of the BSD license (see the COPYING file).
*/

#ifndef VL_HOG_H
#define VL_HOG_H

#include <math.h>

/* Copied over: */

typedef unsigned long long vl_size;
typedef int vl_bool;
typedef long long vl_index;
typedef unsigned long long vl_uindex;
#define VL_EXPORT extern "C"
#define VL_EXPORT
#define VL_TRUE 1
#define VL_FALSE 0
#define VL_PI 3.141592653589793
#define VL_INLINE static //__inline

/** @brief Compute the minimum between two values
** @param x value
** @param y value
** @return the minimum of @a x and @a y.
**/
#define VL_MIN(x,y) (((x)<(y))?(x):(y))

/** @brief Compute the maximum between two values
** @param x value.
** @param y value.
** @return the maximum of @a x and @a y.
**/
#define VL_MAX(x,y) (((x)>(y))?(x):(y))

/** @brief Floor and convert to integer
** @param x argument.
** @return Similar to @c (int) floor(x)
**/
VL_INLINE long int
vl_floor_f(float x)
{
	long int xi = (long int)x;
	if (x >= 0 || (float)xi == x) return xi;
	else return xi - 1;
}

/** @brief Round
** @param x argument.
** @return @c lround(x)
** This function is either the same or similar to C99 @c lround().
**/
VL_INLINE long int
vl_round_d(double x)
{
	//return cvRound(x); // Note: stripped some optimization/ifdef logic from it

	/*#if (defined _MSC_VER && defined _M_X64) || (defined __GNUC__ && defined __x86_64__ && defined __SSE2__ && !defined __APPLE__)
    __m128d t = _mm_set_sd( x );
    return _mm_cvtsd_si32(t);
#elif defined _MSC_VER && defined _M_IX86
    int t;
    __asm
    {
        fld x;
        fistp t;
    }
    return t;
#elif defined _MSC_VER && defined _M_ARM && defined HAVE_TEGRA_OPTIMIZATION
    TEGRA_ROUND(x);
#elif defined CV_ICC || defined __GNUC__
#  ifdef HAVE_TEGRA_OPTIMIZATION
    TEGRA_ROUND(x);
#  else
    return (int)lrint(x);
#  endif
#else
    double intpart, fractpart;
    fractpart = modf(x, &intpart);
    if ((fabs(fractpart) != 0.5) || ((((int)intpart) % 2) != 0))
        return (int)(x + (x >= 0 ? 0.5 : -0.5));
    else
        return (int)intpart;
#endif*/

	double intpart, fractpart;
    fractpart = modf(x, &intpart);
    if ((fabs(fractpart) != 0.5) || ((((int)intpart) % 2) != 0))
        return (int)(x + (x >= 0 ? 0.5 : -0.5));
    else
        return (int)intpart;
}

/* End copied over */

enum VlHogVariant_ { VlHogVariantDalalTriggs, VlHogVariantUoctti } ;

typedef enum VlHogVariant_ VlHogVariant ;

struct VlHog_
{
	VlHogVariant variant ;
	vl_size dimension ;
	vl_size numOrientations ;
	vl_bool transposed ;
	vl_bool useBilinearOrientationAssigment ;

	/* left-right flip permutation */
	vl_index * permutation ;

	/* glyphs */
	float * glyphs ;
	vl_size glyphSize ;

	/* helper vectors */
	float * orientationX ;
	float * orientationY ;

	/* buffers */
	float * hog ;
	float * hogNorm ;
	vl_size hogWidth ;
	vl_size hogHeight ;
} ;

typedef struct VlHog_ VlHog ;

VL_EXPORT VlHog * vl_hog_new (VlHogVariant variant, vl_size numOrientations, vl_bool transposed) ;
VL_EXPORT void vl_hog_delete (VlHog * self) ;
VL_EXPORT void vl_hog_process (VlHog * self,
								float * features,
								float const * image,
								vl_size width, vl_size height, vl_size numChannels,
								vl_size cellSize) ;

VL_EXPORT void vl_hog_put_image (VlHog * self,
									float const * image,
									vl_size width, vl_size height, vl_size numChannels,
									vl_size cellSize) ;

VL_EXPORT void vl_hog_put_polar_field (VlHog * self,
										float const * modulus,
										float const * angle,
										vl_bool directed,
										vl_size width, vl_size height, vl_size cellSize) ;

VL_EXPORT void vl_hog_extract (VlHog * self, float * features) ;
VL_EXPORT vl_size vl_hog_get_height (VlHog * self) ;
VL_EXPORT vl_size vl_hog_get_width (VlHog * self) ;


VL_EXPORT void vl_hog_render (VlHog const * self,
								float * image,
								float const * features,
								vl_size width,
								vl_size height) ;

VL_EXPORT vl_size vl_hog_get_dimension (VlHog const * self) ;
VL_EXPORT vl_index const * vl_hog_get_permutation (VlHog const * self) ;
VL_EXPORT vl_size vl_hog_get_glyph_size (VlHog const * self) ;

VL_EXPORT vl_bool vl_hog_get_use_bilinear_orientation_assignments (VlHog const * self) ;
VL_EXPORT void vl_hog_set_use_bilinear_orientation_assignments (VlHog * self, vl_bool x) ;

/* VL_HOG_H */
#endif
