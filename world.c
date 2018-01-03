
#include "utils.h"
#include "defines.h"
#include "world.h"
#include "pair.h"
#include <stdio.h>
#include <stdlib.h>

PWorld WorldCreate(TIntegrationMethod IM, TCorrector Corrector, int CorrectorPass, int OnlyCorrectBorders, TVector2D Gravity, int ParticleCount, double Lambda)
{
  // Grosso modo, il s'agit de remplir la structure en faisant
  // attention pour la gestion des erreurs
  if (IM.PassCount > MAX_INTEGRATION_PASS)
    return NULL;
  struct TWorld* World = malloc(sizeof(*World));
  if (!World)
    return NULL;
  World->Gravity = Gravity;
  World->IntegrationMethod = IM;
  World->Corrector = Corrector;
  World->CorrectorPass = CorrectorPass;
  World->OnlyCorrectBorders = OnlyCorrectBorders;
  World->Lambda = Lambda;
  World->MaxParticleCount = ParticleCount;
  World->Particles = malloc(ParticleCount * sizeof(*(World->Particles)));
  if (!World->Particles)
    {
      free(World);
      return NULL;
    }
  World->MaxPairCount = (ParticleCount * (ParticleCount-1))/2;
  World->Pairs = malloc(World->MaxPairCount * sizeof(*(World->Pairs)));
  if (!World->Pairs)
    {
      free(World->Particles);
      free(World);
      return NULL;
    }
  World->ParticleCount = World->PairCount = 0;
  return World;
}
void WorldStep(struct TWorld* World, double TimeStep)
{
  int i = 0;
  int j = 0;
  // CurrentPositon et CurrentVelocity garderont la trace de la position et de la vitesse au début du pas
  for (j = 0; j < World->ParticleCount; ++j)
    {
      World->Particles[j].CurrentPosition = World->Particles[j].Position;
      World->Particles[j].CurrentVelocity = World->Particles[j].Velocity;
    }
  // L'intégration en elle-même est déléguée à une fonction de traitement externe
  (*World->IntegrationMethod.Func)(TimeStep, World);
  // On met à jour OldPosition et OldVelocity pour les méthodes qui en ont besoin (Verlet)
  for (j = 0; j < World->ParticleCount; ++j)
    {
      // Le traitement n'est généralement pas correct pour les particules de masse infini
      // On le refait comme il le faut
      if (World->Particles[j].InvMass < EPSILON)
	{
	  // On fait une prédiction de la vitesse pour les noyaux durs
	  // Un calcul exact serait possible, mais n'est pas vraiment utile en terme de précision
	  Vect2DSub(World->Particles[j].CurrentPosition, World->Particles[j].OldPosition, &World->Particles[j].Velocity);
	  Vect2DAdd(World->Particles[j].CurrentPosition, World->Particles[j].Velocity, &World->Particles[j].Position);
	  Vect2DMul(1./TimeStep, &World->Particles[j].Velocity);
	}
      World->Particles[j].OldPosition = World->Particles[j].CurrentPosition;
      World->Particles[j].OldVelocity = World->Particles[j].CurrentVelocity;
    }

  // On applique le correcteur d'il y en a un
  if (World->Corrector)
    {
      // On "démarque" les particules pour affichage
      // Seront éventuellement remarquées les particules dont la trajectoire aura été
      // corrigée
      for (j = 0; j < World->ParticleCount; ++j)
	World->Particles[j].Color = World->Particles[j].OldColor;
      // On fait éventuellement plusieurs passage de correcteur
      // En effet, la correction dépend de l'ordre d'application
      // Les temps de calcul pouvant rapidement en cas d'application correcte
      // on se contente de cette approximation
      for (i = 0; i < World->CorrectorPass; ++i)
	{
	  for (j = 0; j < World->ParticleCount; ++j)
	    ParticleResetAcceleration(&World->Particles[j]);
	  for (j = 0; j < World->PairCount; ++j)
	    {
	      // Au moins une des deux particules doient avoir une masse finie
	      if ((World->Pairs[j].Part1->InvMass > EPSILON) || (World->Pairs[j].Part2->InvMass > EPSILON))
		if ((!World->OnlyCorrectBorders) || (World->Pairs[j].Part1->InvMass < EPSILON) || (World->Pairs[j].Part2->InvMass < EPSILON))
		  (*World->Corrector)(TimeStep, &World->Pairs[j]);
	    }
	}
    }
}

void WorldStepInternal(struct TWorld* World, int Pass, double SmallStep)
{
  int j = 0;
  // On met les accumulateurs de force à zéro
  for (j = 0; j < World->ParticleCount; ++j)
    ParticleResetAcceleration(&World->Particles[j]);
  // Calcul des forces d'interaction entre particules
  for (j = 0; j < World->PairCount; ++j)
    {
      // Calcul des distances avec décalage pour RK
      PairUpdateDistance(Pass, SmallStep, &World->Pairs[j]);
      // Calcul effectif des forces
      PairComputeForce(&World->Pairs[j]);
    }
  // Calcul des forces d'amortissement et de pesanteur
  for (j = 0; j < World->ParticleCount; ++j)
    {
      if (World->Particles[j].InvMass > EPSILON)
	{
	  TVector2D Amort;
	  Amort = World->Particles[j].VelocityRope[Pass];
	  Vect2DMul(SmallStep, &Amort);
	  Vect2DAdd(Amort, World->Particles[j].CurrentVelocity, &Amort);
	  
	  Vect2DMul(-World->Particles[j].InvMass*World->Lambda, &Amort);
	  Vect2DAdd(World->Particles[j].Acceleration, Amort, &World->Particles[j].Acceleration);
	  Vect2DAdd(World->Particles[j].Acceleration, World->Gravity, &World->Particles[j].Acceleration);
	}
    }
}

void WorldDestroy(struct TWorld* World)
{
  free(World->Particles);
  free(World->Pairs);
  free(World);
}
int WorldAddParticle(struct TWorld* World, TParticle Part)
{
  int counter = 0, i = 0;
  // S'il y a trop de particules, tant pis...
  if (World->ParticleCount == World->MaxParticleCount)
    return -1;
  // On remplit correctement un certains nombres de champs qui n'intéressent
  // pas l'utilisateur
  World->Particles[World->ParticleCount] = Part;
  World->Particles[World->ParticleCount].OldPosition = Part.Position;
  World->Particles[World->ParticleCount].OldVelocity = Part.Velocity;
  World->Particles[World->ParticleCount].InitPosition = Part.Position;
  World->Particles[World->ParticleCount].OldColor = Part.Color;
  // On met à zéro le premier décalage (pratique pour RK)
  Vect2DZero(&World->Particles[World->ParticleCount].PositionRope[0]);
  Vect2DZero(&World->Particles[World->ParticleCount].VelocityRope[0]);
  // on crée toutes les paires induites par la nouvelle particule
  counter = World->PairCount;
  for (i = 0; i < World->ParticleCount; ++i)
    {
      World->Pairs[counter].Part1 = &World->Particles[i];
      World->Pairs[counter].Part2 = &World->Particles[World->ParticleCount];
      if (World->Pairs[counter].Part1->InvMass > EPSILON && World->Pairs[counter].Part2->InvMass > EPSILON)
	World->Pairs[counter].Coef = RESPONSE_COEF;
      else
	World->Pairs[counter].Coef = BORDER_COEF;
      PairInit(&World->Pairs[counter]);
      ++counter;
    }
  ++World->ParticleCount;
  World->PairCount = counter;
  return World->ParticleCount-1;
}
