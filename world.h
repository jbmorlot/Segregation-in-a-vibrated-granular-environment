
#ifndef __WORLD_H
#define __WORLD_H

#include "particle.h"
#include "pair.h"
#include "integration.h"
#include "corrector.h"

struct TWorld
{
  TVector2D Gravity;
  TParticle* Particles;
  int ParticleCount;
  int MaxParticleCount;
  TPair* Pairs;
  int PairCount;
  int MaxPairCount;
  TIntegrationMethod IntegrationMethod;
  TCorrector Corrector;
  int OnlyCorrectBorders;
  int CorrectorPass;
  double Lambda;
};

typedef struct TWorld (*PWorld);

PWorld WorldCreate(TIntegrationMethod IM, TCorrector Corrector, int CorrectorPass, int OnlyCorrectBorders, TVector2D Gravity, int ParticleCount, double Lambda);
void WorldStep(struct TWorld* World, double TimeStep);
void WorldStepInternal(struct TWorld* World, int Pass, double SmallStep);
void WorldDestroy(struct TWorld* World);
int WorldAddParticle(struct TWorld* World, TParticle Part);

#endif // __WORLD_H

