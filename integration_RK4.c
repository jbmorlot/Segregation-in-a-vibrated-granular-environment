
#include <stdlib.h>
#include "integration_RK4.h"
#include "vector2D.h"
#include "world.h"

void IntegrationRK4_func(double Step, struct TWorld* World)
{
  // On applique RK4
  // Si y'=f(t,y)
  // On pose y_0 = y0
  // et k1_n = f(t,y_n)
  //    k2_n = f(t+dt/2, y_n + dt/2*k1_n)
  //    k3_n = f(t+dt/2, y_n + dt/2*k2_n)
  //    k4_n = f(t+dt, y_n + dt*k3_n)
  // puis y_{n+1} = y_n + dt*(k1_n + 2*k2_n + 2*k3_n + k4)/6

  TVector2D dt_Vel, dt_Acc;
  TParticle* Part;
  int i = 0;
  // Calcul de f(t,y_n)
  WorldStepInternal(World, 0, 0.);
  for (i = 0; i < World->ParticleCount; ++i)
    {
      Part = &World->Particles[i];
      Part->VelocityRope[1] = Part->Acceleration;
      Part->PositionRope[1] = Part->Velocity;
    }
  // Calcul de f(t+dt/2,y_n+dt/2*k1_n)
  // le décalage par rapport à y_n étant calculé dans la boucle précédente
  WorldStepInternal(World, 1, Step/2.);
  for (i = 0; i < World->ParticleCount; ++i)
    {
      Part = &World->Particles[i];
      Part->VelocityRope[2] = Part->Acceleration;
      Part->PositionRope[2] = Part->Velocity;
    }
  // Calcul de f(t+dt/2,y_n+dt/2*k2_n)
  WorldStepInternal(World, 2, Step/2.);
  for (i = 0; i < World->ParticleCount; ++i)
    {
      Part = &World->Particles[i];
      Part->VelocityRope[3] = Part->Acceleration;
      Part->PositionRope[3] = Part->Velocity;
    }
  // Calcul de f(t+dt,y_n+dt*k3_n)
  WorldStepInternal(World, 3, Step);
  // Enfin, on calcule y_{n+1}
  for (i = 0; i < World->ParticleCount; ++i)
    {
      Part = &World->Particles[i];

      // Calcul de k = dt*(k1_n + 2*k2_n + 2*k3_n + k4)/6
      Vect2DMul(2., &Part->VelocityRope[2]);
      Vect2DMul(2., &Part->PositionRope[2]);
      Vect2DMul(2., &Part->VelocityRope[3]);
      Vect2DMul(2., &Part->PositionRope[3]);

      Vect2DAdd(Part->Acceleration, Part->VelocityRope[1], &dt_Acc);
      Vect2DAdd(Part->Velocity, Part->PositionRope[1], &dt_Vel);
      Vect2DAdd(dt_Acc, Part->VelocityRope[2], &dt_Acc);
      Vect2DAdd(dt_Vel, Part->PositionRope[2], &dt_Vel);
      Vect2DAdd(dt_Acc, Part->VelocityRope[3], &dt_Acc);
      Vect2DAdd(dt_Vel, Part->PositionRope[3], &dt_Vel);

      Vect2DMul(Step/6., &dt_Acc);
      Vect2DMul(Step/6., &dt_Vel);

      // Ajout à y_n
      Vect2DAdd(Part->CurrentPosition, dt_Vel, &Part->Position);
      Vect2DAdd(Part->CurrentVelocity, dt_Acc, &Part->Velocity);
    }
}
TIntegrationMethod IntegrationRK4(void)
{
  TIntegrationMethod IM;
  IM.PassCount = 4;
  IM.Func = &IntegrationRK4_func;
  return IM;
}
