
#ifndef __PAIR_H
#define __PAIR_H

#include "particle.h"

// D�finition d'une paire de particule pouvant int�ragir
typedef struct
{
  TParticle* Part1; // Pointeur vers une particule du couple
  TParticle* Part2; // Pointeur vers la seconde

  TVector2D ToVect; // Vecteur unitaire partant de la particule 1 vers la seconde
  double Distance;
  double MinDistance; // Distance d'interraction
  double Coef; // Coef �lastique
} TPair;

void PairInit(TPair* Pair); // Remplissage automatique de certains champs de TPair (ex: MinDistance)
void PairUpdateDistance(int Pass, double SmallStep, TPair* Pair); // Mise � jour de ToVect et Distance dans la structure pass�e en param�tre
void PairComputeForce(TPair* Pair); // Calcule la force et application de celle-ci lors d'une interraction entre deux particules. Tout est consid�r� : i.e. le test de distance est effectu�

#endif // _PAIR_H
