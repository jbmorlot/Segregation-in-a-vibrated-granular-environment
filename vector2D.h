
#ifndef __VECTOR2D_H
#define __VECTOR2D_H

typedef struct
{
  double x;
  double y;
} TVector2D;

void Vect2DZero(TVector2D* V); // Mise à zeéro d'un vecteur
void Vect2DMul(double lamda, TVector2D* Result); // Multiplier un vecteur par un scalaire
void Vect2DAdd(TVector2D V1, TVector2D V2, TVector2D* Result); // Additionner deux vecteurs
void Vect2DSub(TVector2D V1, TVector2D V2, TVector2D* Result); // Soustraire un vecteur à un autre

double Vect2DDot(TVector2D V1, TVector2D V2); // Calcul du produit scalaire
double Vect2DSqLength(TVector2D V); // Norme au carré d'un vecteur
double Vect2DLength(TVector2D V); // Norme d'un vecteur
double Vect2DDistance(TVector2D V1, TVector2D V2); // En considérant que deux vecteurs sont des points, calculs la distance entre eux
void Vect2DNormalize(TVector2D* V); // Normalise un vecteur

void DEBUG_Vect2DOutput(TVector2D V); // Affichage d'un vecteur pour débuggage.

#endif // __VECTOR2D_H
