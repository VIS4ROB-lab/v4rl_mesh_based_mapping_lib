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
#include "common.h"


#if GEOM_PSEUDO3D==GEOM_TRUE
	namespace GEOM_FADE25D {
#elif GEOM_PSEUDO3D==GEOM_FALSE
	namespace GEOM_FADE2D {
#else
	#error GEOM_PSEUDO3D is not defined
#endif

class CLASS_DECLSPEC Vector2;

/** \brief Vector
*
* This class represents a vector in 2D
*/
class Vector2
{
public:

#if GEOM_PSEUDO3D==GEOM_TRUE
/** \brief Constructor
*
*/

	Vector2(const double x_,const double y_,const double z_);

/** \brief Default constructor
*
* The vector is initialized to (0,0,0)
*/

	Vector2();

/** \brief Copy constructor
*
* Create a copy of vector v_
*/

	Vector2(const Vector2& v_);

#else

/** \brief Constructor
*
*/

	Vector2(const double x_,const double y_);
/** \brief Default constructor
*
* The vector is initialized to (0,0)
*/

	Vector2();
/** \brief Copy constructor
*
* Create a copy of vector v_
*/

	Vector2(const Vector2& v_);
#endif



#if GEOM_PSEUDO3D==GEOM_TRUE
	/** \brief Get an orthogonal vector (CCW direction)
	 *
	 * @note: Only (x,y) coordinates are computed, z=0
	 */
#else
	/** \brief Get an orthogonal vector (CCW direction)
	 */
#endif

	Vector2 orthogonalVector() const;

	/** \brief Get the x-value
	*/

	double x() const;

	/** \brief Get the y-value
	*/

	double y() const;

#if GEOM_PSEUDO3D==GEOM_TRUE
	/** \brief Get the z-value.
	*/

	double z() const;
#endif

#if GEOM_PSEUDO3D==GEOM_TRUE
	/** \brief Set the values
	*/
	void set(const double x_,const double y_,const double z_);

#else
	/** \brief Set the values
	*/
	void set(const double x_,const double y_);
#endif


/** \brief Get the length of the vector
*/

	double length() const;

/** \brief Scalar product
*/
#if GEOM_PSEUDO3D==GEOM_TRUE

	double operator*(const Vector2& other) const;
#else

	double operator*(const Vector2& other) const;
#endif


/** \brief Multiply by a scalar value
*/
#if GEOM_PSEUDO3D==GEOM_TRUE

	Vector2 operator*(double val) const;
#else

	Vector2 operator*(double val) const;
#endif

/** \brief Divide by a scalar value
*/
#if GEOM_PSEUDO3D==GEOM_TRUE

	Vector2 operator/(double val) const;
#else

	Vector2 operator/(double val) const;
#endif


protected:
	double valX;
	double valY;
#if GEOM_PSEUDO3D==GEOM_TRUE
	double valZ;
#endif
};




// Free functions

CLASS_DECLSPEC
inline std::ostream &operator<<(std::ostream &stream, const Vector2& vec)
{
#if GEOM_PSEUDO3D==GEOM_TRUE
	stream << "Vector2: "<<vec.x()<<", "<<vec.y()<<", "<<vec.z();
#else
	stream << "Vector2: "<<vec.x()<<", "<<vec.y();
#endif
	return stream;
}

#if GEOM_PSEUDO3D==GEOM_TRUE
/** \brief Cross product
*/
CLASS_DECLSPEC
inline Vector2 crossProduct(const Vector2& vec0,const Vector2& vec1)
{

	double x=vec0.y() * vec1.z() - vec0.z() * vec1.y();
	double y=vec0.z() * vec1.x() - vec0.x() * vec1.z();
	double z=vec0.x() * vec1.y() - vec0.y() * vec1.x();
	return Vector2(x,y,z);
}
#endif




/** \brief Normalize the vector
*/
CLASS_DECLSPEC
inline Vector2 normalize(const Vector2& other)
{
	double len(other.length());
#if GEOM_PSEUDO3D==GEOM_TRUE
	if(len>0)
	{
		return Vector2(other.x()/len,other.y()/len,other.z()/len);
	}
	else
	{
		std::cout<<"warning: normalize(const Vector2& other), Null length vector!"<<std::endl;
		return Vector2(0,0,0);
	}
#else
	if(len>0)
	{

		return Vector2(other.x()/len,other.y()/len);
	}
	else
	{
		std::cout<<"warning: normalize(const Vector2& other), Null length vector!"<<std::endl;
		return Vector2(0,0);
	}
#endif
}

CLASS_DECLSPEC
inline Vector2 operator-(const Vector2& in)
{
#if GEOM_PSEUDO3D==GEOM_TRUE
	return Vector2(-in.x(),-in.y(),-in.z());
#else
	return Vector2(-in.x(),-in.y());
#endif
}

CLASS_DECLSPEC
inline Vector2 operator*(double d,const Vector2& vec)
{
#if GEOM_PSEUDO3D==GEOM_TRUE
	return Vector2(d*vec.x(),d*vec.y(),d*vec.z());
#else
	return Vector2(d*vec.x(),d*vec.y());
#endif
}


} // (namespace)
