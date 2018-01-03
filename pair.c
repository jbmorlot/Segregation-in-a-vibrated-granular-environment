
#include "pair.h"
#include "utils.h"
#include "vector2D.h"

void PairInit(TPair* Pair)
{
  Pair->MinDistance = Pair->Part1->Radius + Pair->Part2->Radius;
  PairUpdateDistance(0, 0., Pair);
}

void PairUpdateDistance(int Pass, double SmallStep, TPair* Pair)
{
  TVector2D Pos1, Pos2, Rope1, Rope2;

  // Il faut prendre en compte les décalages pour RK
  Rope1 = Pair->Part1->PositionRope[Pass];
  Rope2 = Pair->Part2->PositionRope[Pass];
  Vect2DMul(SmallStep, &Rope1);
  Vect2DMul(SmallStep, &Rope2);
  Vect2DAdd(Pair->Part1->CurrentPosition, Rope1, &Pos1);
  Vect2DAdd(Pair->Part2->CurrentPosition, Rope2, &Pos2);

  Vect2DSub(Pos2, Pos1, &Pair->ToVect);       //Vecteur ToVect=Vecteur1-Vecteur2
  Pair->Distance = Vect2DLength(Pair->ToVect);      //Distance entre les 2 particules = longueur du vecteur toVect
  if (Pair->Distance > EPSILON)
    Vect2DMul(1./Pair->Distance, &Pair->ToVect);      //On normalise le vecteur
  else
    Vect2DZero(&Pair->ToVect);
  //  Pair->Distance = Vect2DDistance(Pair->Part1->Position, Pair->Part2->Position);
}

void PairComputeForce(TPair* Pair)
{
  // On vérifie la distance par rapport à la distance d'interaction
  if (Pair->Distance < Pair->MinDistance)
    {
      // Deform contient la déformation i.e. l'interpénétration des deux particules
      //      double Deform = Pair->MinDistance/Pair->Distance -1.;//)/Pair->MinDistance;
      double Deform = (Pair->MinDistance - Pair->Distance);
      // F1 = force à appliquer à la particule 1
      // F2 = force à appliquer à la particule 2
      TVector2D F1, F2;
      F1 = F2 = Pair->ToVect;
      Vect2DMul(-Deform * Pair->Coef, &F1);
      Vect2DMul(Deform * Pair->Coef, &F2);

      // On accumule les forces
      ParticleAddForce(F1, Pair->Part1);
      ParticleAddForce(F2, Pair->Part2);
    }
}
