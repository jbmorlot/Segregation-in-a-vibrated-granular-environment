
#include "particle.h"

void ParticleResetAcceleration(TParticle* Part)
{
  Part->Acceleration.x = Part->Acceleration.y = 0.;
}
void ParticleAddForce(TVector2D Force, TParticle* Part)
{
  Vect2DMul(Part->InvMass, &Force);
  ParticleAddAcceleration(Force, Part);
}
void ParticleAddAcceleration(TVector2D Acceleration, TParticle* Part)
{
  Vect2DAdd(Part->Acceleration, Acceleration, &Part->Acceleration);
}
