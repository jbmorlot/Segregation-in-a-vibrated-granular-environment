
#ifndef __PAIR_H
#define __PAIR_H

#include "particle.h"

// Définition d'une paire de particule pouvant intéragir
typedef struct
{
  TParticle* Part1; // Pointeur vers une particule du couple
  TParticle* Part2; // Pointeur vers la seconde

  TVector2D ToVect; // Vecteur unitaire partant de la particule 1 vers la seconde
  double Distance;
  double MinDistance; // Distance d'interraction
  double Coef; // Coef élastique
} TPair;

void PairInit(TPair* Pair); // Remplissage automatique de certains champs de TPair (ex: MinDistance)
void PairUpdateDistance(int Pass, double SmallStep, TPair* Pair); // Mise à jour de ToVect et Distance dans la structure passée en paramètre
void PairComputeForce(TPair* Pair); // Calcule la force et application de celle-ci lors d'une interraction entre deux particules. Tout est considéré : i.e. le test de distance est effectué

#endif // _PAIR_H
