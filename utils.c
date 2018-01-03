
#include "utils.h"
#include <math.h>

int solve_second_order(double a, double b, double c, double* root1, double* root2)
{
  if (fabs(a) < EPSILON)
  {
    if (fabs(b) < EPSILON)
      return 0;
    else {
      if (root1)
        *root1 = -c/b;
      return 1;
    }
  }
  double delta = b*b - 4.*a*c;
  if (delta < 0)
    return 0;
  if (fabs(delta) < EPSILON)
  {
    if (root1)
      *root1 = -b/(2.*a);
    return 1;
  }
  double sq_delta = sqrt(delta);
  double r1 = (-b-sq_delta)/(2.*a);
  double r2 = (-b+sq_delta)/(2.*a);

  if (root1)
    *root1 = MIN(r1,r2);
  if (root2)
    *root2 = MAX(r1,r2);
  return 2;
}


