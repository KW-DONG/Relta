#include "delta.h"
#include "type.h"
#include <math.h>
#include "config.h"

void Forward_Kinematics(float* abc, float* xyz)
{
  // Create a vector in old coordinates along x axis of new coordinate
  const float p12[3] = { x2 - x1, y2 - y1, abc[1] - abc[0] },

  // Get the reciprocal of Magnitude of vector.
  d2 = SQ(p12[0]) + SQ(p12[1]) + SQ(p12[2]), inv_d = RSQRT(d2),

  // Create unit vector by multiplying by the inverse of the magnitude.
  ex[3] = { p12[0] * inv_d, p12[1] * inv_d, p12[2] * inv_d },

  // Get the vector from the origin of the new system to the third point.
  p13[3] = { x3 - x1, y3 - y1, abc[2] - abc[0] },

  // Use the dot product to find the component of this vector on the X axis.
  i = ex[0] * p13[0] + ex[1] * p13[1] + ex[2] * p13[2],

  // Create a vector along the x axis that represents the x component of p13.
  iex[3] = { ex[0] * i, ex[1] * i, ex[2] * i };

  // Subtract the X component from the original vector leaving only Y. We use the
  // variable that will be the unit vector after we scale it.
  float ey[3] = { p13[0] - iex[0], p13[1] - iex[1], p13[2] - iex[2] };

  // The magnitude and the inverse of the magnitude of Y component
  const float j2 = SQ(ey[0]) + SQ(ey[1]) + SQ(ey[2]), inv_j = RSQRT(j2);

  // Convert to a unit vector
  ey[0] *= inv_j; ey[1] *= inv_j; ey[2] *= inv_j;

  // The cross product of the unit x and y is the unit z
  // float[] ez = vectorCrossProd(ex, ey);
  const float ez[3] = {
    ex[1] * ey[2] - ex[2] * ey[1],
    ex[2] * ey[0] - ex[0] * ey[2],
    ex[0] * ey[1] - ex[1] * ey[0]
  },

  // We now have the d, i and j values defined in Wikipedia.
  // Plug them into the equations defined in Wikipedia for Xnew, Ynew and Znew
  Xnew = (d2) * inv_d * 0.5f,
  Ynew = ((SQ(i) + j2) * 0.5f - i * Xnew) * inv_j,
  Znew = sqrtf(SQ(L) - HYPOT2(Xnew, Ynew));//

  // Start from the origin of the old coordinates and add vectors in the
  // old coords that represent the Xnew, Ynew and Znew to find the point
  // in the old system.
  xyz[0] = x1+ex[0] * Xnew + ey[0] * Ynew - ez[0] * Znew;
  xyz[1] = y1+ex[1] * Xnew + ey[1] * Ynew - ez[1] * Znew;
  xyz[2] = abc[0] + ex[2] * Xnew + ey[2] * Ynew - ez[2] * Znew;
}

void Inverse_Kinematics(float* xyz, float* abc)
{
    abc[0] = sqrtf(SQ(L)-SQ(xyz[0]-x1)-SQ(xyz[1]-y1))+xyz[2];
    abc[1] = sqrtf(SQ(L)-SQ(xyz[0]-x2)-SQ(xyz[1]-y2))+xyz[2];
    abc[2] = sqrtf(SQ(L)-SQ(xyz[0]-x3)-SQ(xyz[1]-y3))+xyz[2];
}

void Jacobian_Matrix(float* xyz_v, float* xyz, float* abc, float* abc_v)
{
    abc_v[0] = (xyz[0]-x1)*INV(xyz[2]-abc[0])*xyz_v[0]+(xyz[1]-y1)*INV(xyz[2]-abc[0])*xyz_v[1]+xyz_v[2];
    
    abc_v[1] = (xyz[0]-x2)*INV(xyz[2]-abc[1])*xyz_v[0]+(xyz[1]-y2)*INV(xyz[2]-abc[1])*xyz_v[1]+xyz_v[2];

    abc_v[2] = (xyz[0]-x3)*INV(xyz[2]-abc[2])*xyz_v[0]+(xyz[1]-y3)*INV(xyz[2]-abc[2])*xyz_v[1]+xyz_v[2];
}
