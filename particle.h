
#ifndef __PARTICLE_H
#define __PARTICLE_H

#include "vector2D.h"
#include "defines.h"

// Structure de définition d'une particule
// Ne sont stockées que les données effectivement utiles
// Exemple : l'inverse de la masse plutôt que la masse
// Note : la vitesse est gardée pour d'autres méthodes d'intégration
typedef struct
{
  // Position est assez explicite
  // CurrentPosition stocke la position au début du pas d'intégration
  // OldPosition stocke la position au pas précédent
  // Les PositionRope permettent de stocker des données propres à l'intégration
  // typiquement, ce sont les pentes calculées à chaque semi-pas dans Runge-Kutta
  // Note : le calcul des forces prend en compte ces valeurs à ajouter
  TVector2D Position, OldPosition, CurrentPosition;
  // Position au début de la simulation
  TVector2D InitPosition;
  TVector2D Velocity, OldVelocity, CurrentVelocity;
  TVector2D PositionRope[MAX_INTEGRATION_PASS];
  TVector2D VelocityRope[MAX_INTEGRATION_PASS];
  TVector2D Acceleration;
  double InvMass;
  double Radius;
  int Color;
  int OldColor;
} TParticle;

void ParticleResetAcceleration(TParticle* Part); // Remise à zéro de l'accélération, i.e. remise à zéro de la somme des forces pour le prochain pas de calcul
void ParticleAddForce(TVector2D Force, TParticle* Part); // Ajoute une force à la particule
void ParticleAddAcceleration(TVector2D Acceleration, TParticle* Part); // Ajoute directement une accélération, pratique pour le champ de pesanteur

#endif // __PARTICLE_H
