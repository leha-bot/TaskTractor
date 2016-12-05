#include "SMat.h"

#ifndef NULL
#define NULL 0
#endif

//-------------------------------[class SVec]----------------------------------
SVec::SVec():x0(0.),x1(0.),x2(0.) {}

//----------------------------------------------
SVec::SVec( const double * v )
{
	x0 = v[0];
	x1 = v[1];
	x2 = v[2];
}
//----------------------------------------------
SVec::SVec( const float * v )
{
	x0 = v[0];
	x1 = v[1];
	x2 = v[2];
}
//----------------------------------------------
SVec::SVec( const double v )
{
	x0 = x1 = x2 = v;
}
//----------------------------------------------
SVec::SVec( const double v0, const double v1, const double v2 )
{
	x0 = v0;
	x1 = v1;
	x2 = v2;
}
//----------------------------------------------
SVec & SVec::operator = ( const double v[3] )
{
	x0 = v[0];
	x1 = v[1];
	x2 = v[2];
	return *this;
}
//----------------------------------------------
SVec SVec::operator - () const
{
	return SVec( -x0, -x1, -x2 );
}
//----------------------------------------------
SVec SVec::operator * ( const double r ) const
{
	return SVec( x0 * r, x1 * r, x2 * r );
}
//----------------------------------------------
SVec SVec::operator / ( const double r ) const
{
	return SVec( x0 / r, x1 / r, x2 / r );
}
//----------------------------------------------
SVec & SVec::operator *= ( const double r ) 
{
	x0 *= r;
	x1 *= r;
	x2 *= r;
return *this;
}
//----------------------------------------------
SVec & SVec::operator /= ( const double r ) 
{
	x0 /= r;
	x1 /= r;
	x2 /= r;
	return *this;
}
//----------------------------------------------
SVec SVec::operator + ( const SVec & b ) const
{
	return SVec( x0 + b.x0, x1 + b.x1, x2 + b.x2 );
}
//----------------------------------------------
SVec & SVec::operator -= ( const SVec & other ) 
{
	x0 -= other.x0;
	x1 -= other.x1;
	x2 -= other.x2;
	return *this; 
}
//----------------------------------------------
SVec & SVec::operator += ( const SVec & other ) 
{
	x0 += other.x0;
	x1 += other.x1;
	x2 += other.x2;
	return *this; 
}
//----------------------------------------------
SVec SVec::operator - ( const SVec & b ) const
{
	return SVec( x0 - b.x0, x1 - b.x1, x2 - b.x2 );
}
//----------------------------------------------
void SVec::set( const double val) 
{
	x0 = x1 = x2 = val;
}
//----------------------------------------------
bool SVec::is_zero()
{
	return(fabs(x0) < EPS && fabs(x1) < EPS && fabs(x2) < EPS);
}
//-------------------------------[class SMat]-----------------------------------
SMat::SMat() {}
//----------------------------------------------
SMat::SMat( const SVec & v0, const SVec & v1, const SVec & v2): c0(v0), c1(v1), c2(v2){}
//----------------------------------------------
SMat::SMat( double a00, double a01, double a02, 
	  double a10, double a11, double a12, 
	  double a20, double a21, double a22 ):
		c0(a00, a10, a20),
		c1(a01, a11, a21),
		c2(a02, a12, a22){}
//----------------------------------------------
SMat::SMat( double C[3][3] ):
    c0(C[0][0], C[1][0], C[2][0]),
    c1(C[0][1], C[1][1], C[2][1]),
    c2(C[0][2], C[1][2], C[2][2])  {}
//----------------------------------------------
const SVec SMat::operator*( const SVec & a ) const
{
    return c0 * a.x0 + c1 * a.x1 + c2 * a.x2;
}
//----------------------------------------------
const SVec SMat::operator*( const double a[3] ) const
{
    return c0 * a[0] + c1 * a[1] + c2 * a[2];
}

SMat SMat::operator-() const
{
    return SMat( -c0, -c1, -c2 );
}
//----------------------------------------------
SMat & SMat::operator-=( const SMat & other )
{
    c0 -= other.c0;
    c1 -= other.c1;
    c2 -= other.c2;
return *this;
}
//----------------------------------------------
SMat & SMat::operator+=( const SMat & other )
{
    c0 += other.c0;
    c1 += other.c1;
    c2 += other.c2;
    return *this;
}
//----------------------------------------------
SMat & SMat::operator*=( const double val )
{
    c0 *= val;
    c1 *= val;
    c2 *= val;
    return *this;
}
//----------------------------------------------
SMat & SMat::operator/=( const double val )
{
    c0 /= val;
    c1 /= val;
    c2 /= val;
    return *this;
}


//----global functions & operaors overload-------
SVec operator * ( const double r, const SVec & v)
{
  return SVec( r * v.x0, r * v.x1, r * v.x2 );
}
//----------------------------------------------
SVec operator-( const double d[3], const SVec & v )
{
  return SVec( d[0] -v.x0, d[1] -v.x1, d[2] -v.x2 );
}
//----------------------------------------------
SVec operator-( const float f[3], const SVec & v )
{
  return SVec( f[0] -v.x0, f[1] -v.x1, f[2] -v.x2 );
}
//----------------------------------------------
SVec operator-( const SVec & v, const double d[3] )
{
  return SVec( v.x0- d[0], v.x1- d[1], v.x2- d[2]  );
}
//----------------------------------------------
SVec operator-( const SVec & v, const float f[3] )
{
  return SVec( v.x0- f[0], v.x1- f[1], v.x2- f[2]  );
}
//----------------------------------------------
SVec operator+( const double d[3], const SVec & v )
{
  return SVec( d[0] +v.x0, d[1] +v.x1, d[2] +v.x2 );
}
//----------------------------------------------
SVec operator+( const SVec & v, const double d[3] )
{
  return SVec( v.x0 +d[0], v.x1 + d[1], v.x2+ d[2]  );
}
//----------------------------------------------
SMat operator * ( const double v, const SMat & A )
{
  return SMat( A.c0 * v, A.c1 * v, A.c2 * v );
}
//----------------------------------------------
SMat operator + ( const SMat & A, const SMat & B )
{
  return SMat( A.c0 + B.c0, A.c1 + B.c1, A.c2 + B.c2 );
}
//----------------------------------------------
SMat operator - ( const SMat & A, const SMat & B )
{
  return SMat( A.c0 - B.c0, A.c1 - B.c1, A.c2 - B.c2 );
}

//----------------------------------------------
SMat operator * ( const SMat & A, const SMat & B  )
{
  return SMat(  A * B.c0, A * B.c1, A * B.c2 );
}
//----------------------------------------------
SMat transpose( const SMat & A)
{
  return SMat( A.c0.x0, A.c0.x1, A.c0.x2, 
	       A.c1.x0, A.c1.x1, A.c1.x2, 
	       A.c2.x0, A.c2.x1, A.c2.x2 );
}
//----------------------------------------------
SMat transpose( const double A[3][3])
{
  return SMat( SVec(A[0]),SVec(A[1]),SVec(A[2]) );

}

