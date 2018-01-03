
#include "vector2D.h"
#include <stdio.h>
#include <math.h>

void Vect2DZero(TVector2D* V)
{
  V->x = V->y = 0.;
}

void Vect2DMul(double lambda, TVector2D* Result)
{
  Result->x *= lambda;
  Result->y *= lambda;
}

void Vect2DAdd(TVector2D V1, TVector2D V2, TVector2D* Result)
{
  Result->x = V1.x + V2.x;
  Result->y = V1.y + V2.y;
}
void Vect2DSub(TVector2D V1, TVector2D V2, TVector2D* Result)
{
  Result->x = V1.x - V2.x;
  Result->y = V1.y - V2.y;
}

double Vect2DDot(TVector2D V1, TVector2D V2)
{
  return V1.x * V2.x + V1.y * V2.y;
}
double Vect2DSqLength(TVector2D V)
{
  return Vect2DDot(V, V);
}
double Vect2DLength(TVector2D V)
{
  return sqrt(Vect2DSqLength(V));
}
double Vect2DDistance(TVector2D V1, TVector2D V2)
{
  TVector2D V;
  Vect2DSub(V1, V2, &V);
  return Vect2DLength(V);
}
void Vect2DNormalize(TVector2D* V)
{
  double L = Vect2DLength(*V);
  Vect2DMul(1./L, V);
}

void DEBUG_Vect2DOutput(TVector2D V)
{
  printf("DEBUG Vector2D output : (%lf, %lf)\n", V.x, V.y);
}
