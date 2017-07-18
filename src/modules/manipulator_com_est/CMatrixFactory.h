#ifndef __CMATRIXFACTORY__
#define __CMATRIXFACTORY__

/************************************************************************/
/* CMatrixFactory                                                       */
/************************************************************************/



#include <math.h>
#include <mathlib/mathlib.h>
#include <matrix/Matrix.hpp>

using namespace matrix;

typedef SquareMatrix<float, 4> Matrix4f;


/************************************************************************/
/* Rotation matrix                                                      */
/************************************************************************/

      Matrix4f trotx(float alpha );
      Matrix4f troty(float beta );
      Matrix4f trotz(float gamma );


/************************************************************************/
/* Translation matrix                                                   */
/************************************************************************/
    
      Matrix4f translx(float dx );
      Matrix4f transly(float dy );
      Matrix4f translz(float dz );
      Matrix4f transl_vector(Vector3f &vec);

/************************************************************************/
/* Transformation matrix from frame i to i-1                            */
/************************************************************************/

      Matrix4f H_transls(float theta, float a, float d,float alpha);
/************************************************************************/
/* Help functions                                                       */
/************************************************************************/
      Matrix3f ext_rotation(Matrix4f & hm );
      Vector3f ext_translvector (Matrix4f & hm );

#endif
