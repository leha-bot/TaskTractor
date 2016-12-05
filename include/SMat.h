//;-*-C++-*-   - tell Emacs to use C++ major mode

#ifndef _SMAT_H_
#define _SMAT_H_


#include <math.h>
#ifndef M_PI
	#define M_PI 3.1415926535897932384626433832795
#endif
#ifndef M_PI_2
	#define M_PI_2 1.57079632679489661923
#endif

#define PIPIMOD(x) (fmod(x+M_PI,2*M_PI)-M_PI)
#define REVDIRRAD(x) PIPIMOD(x+M_PI)

#define HHMOD(x) (fmod(x+180,360)-180)
#define REVDIR(x) HHMOD(x+180)

#define EPS (1e-10)

//-------------------------------[class SVec]----------------------------------
struct  SVec
{
  double x0;
  double x1;
  double x2;
  
  SVec();
  SVec( const double v0, const double v1, const double v2 );
  explicit SVec( const double * v );
  explicit SVec( const float * v );
  explicit SVec( const double v );
  
  SVec & operator = ( const double v[3] );
  SVec operator - () const;
  SVec operator * ( const double r ) const;
  SVec operator / ( const double r ) const;
  SVec & operator *= ( const double r ) ;
  SVec & operator /= ( const double r ) ;
  SVec operator + ( const SVec & b ) const;
  SVec & operator -= ( const SVec & other ) ;
  SVec & operator += ( const SVec & other ) ;
  SVec operator - ( const SVec & b ) const;
  
  double abs_max() const;
  double norm() const;
  void multiply( const SVec & other ) ;
  void set( const double val) ;
  bool is_zero();

};


//-------------------------------[class SMat]-----------------------------------
class SMat
{
public:
  
  SVec c0;   SVec c1;   SVec c2;
  
  SMat();
  SMat( const SVec & v0, const SVec & v1, const SVec & v2);

  SMat( double a00, double a01, double a02, 
		double a10, double a11, double a12, 
		double a20, double a21, double a22 );
  SMat( double C[3][3] );
  
  SMat operator-() const;
  SMat & operator-=( const SMat & other );
  SMat & operator+=( const SMat & other );
  SMat & operator*=( const double val );
  SMat & operator/=( const double val );
  const SVec operator*( const SVec & a ) const;
  const SVec operator*( const double a[3] ) const;
  


};

//-------------------global functions & operaors overload----------------------
SVec operator * ( const double r, const SVec & v);
SVec operator-( const double d[3], const SVec & v );
SVec operator-( const float  f[3], const SVec & v );
SVec operator-( const SVec & v, const double d[3] );
SVec operator-( const SVec & v, const float  f[3] );
SVec operator+( const double d[3], const SVec & v );
SVec operator+( const SVec & v, const double d[3] );
SMat operator * ( const double v, const SMat & A );
SMat operator + ( const SMat & A, const SMat & B );
SMat operator - ( const SMat & A, const SMat & B );
SMat operator * ( const SMat & A, const SMat & B  );
SMat transpose( const SMat & A );
void assign( double a[3], const SVec & b );



#endif // _SMAT_H_

