
#include "corrector_kernel.h"
#include "vector2D.h"
#include "utils.h"
#include "defines.h"
#include <stdio.h>

void SolveCollision(double TimeStep, double CollisionTime, TVector2D V1, TVector2D V2, TPair* Pair)
{
  TVector2D Pos1, Pos2;
  TVector2D Vel1, Vel2;
  TVector2D NewVel1, NewVel2;
  TVector2D ex, ey;
  double v1x, v1y, v2x, v2y;
  double w1x, w2x; // Nouvelle vitesse selon ex;

  Vel1 = V1;
  Vel2 = V2;

  // On calcule les positions à l'instant du choc
  Pos1 = Vel1;
  Vect2DMul(CollisionTime, &Pos1);
  Vect2DAdd(Pos1, Pair->Part1->OldPosition, &Pos1);

  Pos2 = Vel2;
  Vect2DMul(CollisionTime, &Pos2);
  Vect2DAdd(Pos2, Pair->Part2->OldPosition, &Pos2);

  // Ici traitement et calcul de NewVel(1|2)
  // c'est-à-dire des vitesses juste après le choc
  // on passe par une projection sur l'axe reliant les deux centres
  Vect2DSub(Pos2, Pos1, &ex);
  Vect2DNormalize(&ex);
  ey.x = -ex.y;
  ey.y = ex.x;

  v1x = Vect2DDot(ex, Vel1);
  v1y = Vect2DDot(ey, Vel1);
  v2x = Vect2DDot(ex, Vel2);
  v2y = Vect2DDot(ey, Vel2);

  // on applique simplement
  //    w1 = (2M2*v2 - (M1-M2)*v1)/(M1+M2)
  // et w2 = (2M1*v1 - (M2-M1)*v2)/(M1+M2)
  // avec les cas particuliers aux masses infinies
  // Note : les collisions entre deux masses infinies ne sont jamais traitées
  if (Pair->Part1->InvMass < EPSILON)
    {
      w1x = v1x;
      w2x = 2.*v1x - v2x;
    }
  else if (Pair->Part2->InvMass < EPSILON)
    {
      w1x = 2.*v2x - v1x;
      w2x = v2x;
    }
  else
    {
      double m1, m2;
      m1 = 1./Pair->Part1->InvMass;
      m2 = 1./Pair->Part2->InvMass;
      w1x = (2.*m2*v2x + (m1 - m2)*v1x)/(m1+m2);
      w2x = (2.*m1*v1x + (m2 - m1)*v2x)/(m1+m2);
    }

  Vel1 = ey;
  Vel2 = ey;
  Vect2DMul(v1y, &Vel1);
  Vect2DMul(v2y, &Vel2);
  NewVel1 = ex;
  NewVel2 = ex;
  Vect2DMul(w1x, &NewVel1);
  Vect2DMul(w2x, &NewVel2);
  Vect2DAdd(Vel1, NewVel1, &NewVel1);
  Vect2DAdd(Vel2, NewVel2, &NewVel2);

  // Mise à jour des données pour t+dt (la collision ayant éventuellement lieu avant)
  Pair->Part1->Velocity = NewVel1;
  Pair->Part2->Velocity = NewVel2;

  // Le calcul des vitesses est approximatif, mais suffisant pour la simulation
  Vect2DMul(TimeStep-CollisionTime, &NewVel1);
  Vect2DAdd(NewVel1, Pos1, &Pair->Part1->Position);
  Vect2DMul(TimeStep-CollisionTime, &NewVel2);
  Vect2DAdd(NewVel2, Pos2, &Pair->Part2->Position);
}

void CorrectorKernel(double TimeStep, TPair* Pair)
{
  double r1, r2;
  TVector2D V1, V2, RelativeOldPos, RelativeVel;
  double a, b, c;
  // On cherche à calculer si entre t et t+dt les noyaux sont entrés en collision
  // pour cela on resout :
  // (v2 - v1)^2 * t^2 + 2A1A2.(v2-v1)*t + (A1A2^2 - D^2) = 0
  // où D désigne la somme des rayons des noyaux
  // vi les vitesses moyennes des particules entre t et t+dt
  // et Ai les positions des particules au début du pas d'intégration

  Vect2DSub(Pair->Part2->OldPosition, Pair->Part1->OldPosition, &RelativeOldPos);
  Vect2DSub(Pair->Part1->Position, Pair->Part1->OldPosition, &V1);
  Vect2DSub(Pair->Part2->Position, Pair->Part2->OldPosition, &V2);
  Vect2DMul(1./TimeStep, &V1);
  Vect2DMul(1./TimeStep, &V2);
  Vect2DSub(V2, V1, &RelativeVel);

  a = Vect2DSqLength(RelativeVel);
  b = 2.*Vect2DDot(RelativeOldPos, RelativeVel);
  c = Vect2DSqLength(RelativeOldPos) - 4.*KERNEL_SIZE*KERNEL_SIZE;
  // On a :
  // a = (v2-v1)^2
  // b = 2A1A2.(v2-v1)
  // c = (A1A2^2 - D^2) = (A1A2^2 - (2*KERNEL_SIZE)^2)

  if (solve_second_order(a, b, c, &r1, &r2) == 2)
    // S'il existe des solutions, il y a collision
    // le cas particulier à une seule solution correspond à des billes qui se frollent, on l'ignore (il n'y a pas de frottement)
    // On ne traite la collision que si elle commence entre t et t+dt 
      if ((r1 > 0.) && (r1 < TimeStep))
	{
	  // On marque la paire pour l'affichage
	  Pair->Part1->Color = 0xFFFFFFFF;
	  Pair->Part2->Color = 0xFFFFFFFF;
	  // On résout la collision
	  SolveCollision(TimeStep, r1, V1, V2, Pair);
	}
}

