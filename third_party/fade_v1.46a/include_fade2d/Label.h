// (c) 2010 Geom e.U. Bernhard Kornberger, Graz/Austria. All rights reserved.
//
// This file is part of the Fade2D library. You can use it for your personal
// non-commercial research. Licensees holding a commercial license may use this
// file in accordance with the Commercial License Agreement provided
// with the Software.
//
// This software is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING
// THE WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
//
// Please contact the author if any conditions of this licensing are not clear
// to you.
//
// Author: Bernhard Kornberger, bkorn (at) geom.at
// http://www.geom.at


#pragma once


#include "Point2.h"

#include "common.h"
#if GEOM_PSEUDO3D==GEOM_TRUE
	namespace GEOM_FADE25D {
#elif GEOM_PSEUDO3D==GEOM_FALSE
	namespace GEOM_FADE2D {
#else
	#error GEOM_PSEUDO3D is not defined
#endif

/**  \brief Text-Label
*
* \see Visualizer2 where Label objects are used for visualizations
*/

class Label
{

public:
/** \brief Constructs a Text-%Label
*
* @param p_ is the point where the label appears
* @param s_ is the text to be shown
* @param bWithMark_ switches between text-only and text-with-mark
* @param fontSize_
*/
	CLASS_DECLSPEC
	Label(Point2 p_,std::string s_,bool bWithMark_=true,int fontSize_=8);
	CLASS_DECLSPEC
	~Label(){}
	Point2 p;
	std::string s;
	bool bWithMark;
	int fontSize;
};



} // (namespace)
