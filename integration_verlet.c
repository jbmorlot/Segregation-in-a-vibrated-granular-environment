
#include <stdlib.h>
#include "integration_verlet.h"
#include "world.h"

void IntegrationVerlet_Part(double dt, TParticle* Part)
{
  // Calcul de : x(t+dt) = 2*x(t) - x(t-dt) + dt^2 * a(t)
  // x = NewPos
  // dt^2*a(t) = dt_2_Acc
  // x(t-dt) = OldPosition
  // x(t) = Position
  TVector2D NewPos, dt_2_Acc;
  double dt_2 = dt*dt;
  dt_2_Acc = Part->Acceleration;
  Vect2DMul(dt_2, &dt_2_Acc); // Calcul de dt^2 * a(t)

  NewPos = Part->Position;
  Vect2DMul(2., &NewPos);
  Vect2DAdd(NewPos, dt_2_Acc, &NewPos);
  Vect2DSub(NewPos, Part->OldPosition, &NewPos);

  // Mise à jour des variables
  Part->Position = NewPos;
  Vect2DSub(Part->Position, Part->OldPosition, &Part->Velocity);
  Vect2DMul(1./dt, &Part->Velocity);
}

void IntegrationVerlet_func(double Step, struct TWorld* World)
{
  WorldStepInternal(World, 0, 0.);
  int i = 0;
  for (i = 0; i < World->ParticleCount; ++i)
    IntegrationVerlet_Part(Step, &World->Particles[i]);
}

TIntegrationMethod IntegrationVerlet(void)
{
  TIntegrationMethod IM;
  IM.PassCount = 1;
  IM.Func = &IntegrationVerlet_func;
  return IM;
}


