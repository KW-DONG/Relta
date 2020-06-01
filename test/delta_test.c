#include <stdio.h>
#include <math.h>
#define DELTA_CHAIN_LEN 264.0f
#define DELTA_TOWER_RADIUS 145.0f
#define DELTA_EFFECTOR_RADIUS 37.15f
#define DELTA_EFFECTOR_HEIGHT 20.0f
#define DELTA_CARRIAGE_OFFSET 16.0f
#define n1 0.0f
#define n2 3.14f
#define n3 3.14f*3.0f*INV(2.0f)
#define R   (DELTA_TOWER_RADIUS - DELTA_CARRIAGE_OFFSET)
#define r   DELTA_EFFECTOR_RADIUS
#define L   DELTA_CHAIN_LEN
#define x1  (R-r)*cosf(n1)
#define y1  (R-r)*sinf(n1)
#define x2  (R-r)*cosf(n2)
#define y2  (R-r)*sinf(n2)
#define x3  (R-r)*cosf(n3)
#define y3  (R-r)*sinf(n3)
#define SQ(x)   ((x)*(x))
#define COS(x)  (cosf(x))
#define SIN(x)  (sinf(x))
#define INV(x)  (1.0f / (x))
#define E(x)    (powf(10.0f,x))
#define PI      3.14f
#define RSQRT(x)    (1.0f / sqrtf(x))
#define HYPOT2(x,y) (SQ(x)+SQ(y))

void forward_kinematics_DELTA(float* abc, float* xyz) {
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
  Xnew = (d2) * inv_d * 0.5,
  Ynew = ((SQ(i) + j2) * 0.5 - i * Xnew) * inv_j,
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

int main()
{
    float xyz[3];
    float abc[3] = {300.f,300.f,300.f};
    float abc_t[3];

    while (abc[0]>250.f)
    {
        while (abc[1]>250.f)
        {
            while (abc[2]>250.f)
            {
                forward_kinematics_DELTA(abc,xyz);
                printf("x:%f, y:%f, z:%f\n", xyz[0], xyz[1], xyz[2]);
                Inverse_Kinematics(xyz,abc_t);
                printf("da:%f, db:%f, dc:%f\n", (abc[0]-abc_t[0]),(abc[1]-abc_t[1]),(abc[2]-abc_t[2]));
                abc[2] = abc[2] - 1.f;
            }
            abc[1] = abc[1] - 1.f;
            abc[2] = 300.f;
        }
        abc[0] = abc[0] - 1.f;
        abc[1] = 300.f;
    }


    return 0;

}
