
#include <stdlib.h>
#include "integration_RK2.h"
#include "vector2D.h"
#include "world.h"

void IntegrationRK2_func(double Step, struct TWorld* World)
{
  // Il s'agit de la méthode de Runge-Kutta d'ordre 2
  // Si y vérifie y'=f(y,t)
  // Alors on pose y_0 = y0
  // et y_{n+1/2} = y_n + dt/2*f(y_n,t)
  // puis y_{n+1} = y_n + dt*f(y_{n+1/2}, t+dt/2)
  //
  // Ici la méthode est adapté au cas vectoriel avec une équation du second
  // ordre
  TVector2D dt_Vel, dt_Acc;
  TParticle* Part;
  int i = 0;
  // Là, il s'agit du calcul de f(y_n,t)
  WorldStepInternal(World, 0, 0.);
  // On place les décalages en position et vitesse pour l'étape suivante
  for (i = 0; i < World->ParticleCount; ++i)
    {
      Part = &World->Particles[i];
      Part->VelocityRope[1] = Part->Acceleration;
      Part->PositionRope[1] = Part->Velocity;
    }
  // Grâce à la boucle, on effectue ici f(y_{n+1/2},t+dt/2)
  WorldStepInternal(World, 1, Step/2.);
  // Enfin, on calcul les nouvelles vitesses et position
  for (i = 0; i < World->ParticleCount; ++i)
    {
      Part = &World->Particles[i];
      dt_Acc = Part->Acceleration;
      dt_Vel = Part->Velocity;
      Vect2DMul(Step, &dt_Acc);
      Vect2DMul(Step, &dt_Vel);

      Vect2DAdd(Part->CurrentPosition, dt_Vel, &Part->Position);
      Vect2DAdd(Part->CurrentVelocity, dt_Acc, &Part->Velocity);
    }
}
TIntegrationMethod IntegrationRK2(void)
{
  TIntegrationMethod IM;
  IM.PassCount = 2;
  IM.Func = &IntegrationRK2_func;
  return IM;
}
