
#include "defines.h"
#include "utils.h"
#include "world.h"
#include "vector2D.h"
#include "integration_verlet.h"
#include "integration_RK2.h"
#include "integration_RK4.h"
#include "corrector_kernel.h"
#include "draw.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

// On gardera la trace de la particule centrale pour les graphes
TParticle* BigParticle;
int BigParticleNum;

// Fonction utilitaire permettant d'éviter l'interpénétration des boules dans la situation initiale
// Elle renvoit 1 si la position proposée pour une petite boule la place dans la grosse, 0 sinon
int IsInBigParticle(TVector2D Position)
{
  double Dist = Vect2DDistance(Position, BigParticle->Position);
  return (Dist <= SMALL_RADIUS + BIG_RADIUS);
}

// Fonction utilitaire pour faire avancer la position de placement des petites boules
void NextPos(TVector2D* Pos)
{
  Pos->x += 2.*SMALL_RADIUS;
  if (Pos->x >= (double)(2*BOX_SIZE_X_2 -1)*SMALL_RADIUS)
    {
      Pos->x = -(2*BOX_SIZE_X_2-2)*SMALL_RADIUS;
      Pos->y += 2.*SMALL_RADIUS;
    }
}

// Création de la scène
void SceneCreate(struct TWorld* World)
{
  TParticle Part;
  int i;
  // Initialisation d'une graine aléatoire
  srand(time(NULL));

  // On crée le bas de la boîte
  Vect2DZero(&Part.Position);
  Vect2DZero(&Part.Velocity);
  Vect2DZero(&Part.Acceleration);
  Part.Color = DrawGetColor(0, 0, 255);
  Part.Position.y = -(2*BOX_SIZE_Y_2) * SMALL_RADIUS;
  Part.InvMass = 0.;
  Part.Radius = SMALL_RADIUS;
  for (i = -BOX_SIZE_X_2*2; i <= BOX_SIZE_X_2*2; ++i)
    {
      Part.Position.x = (double)(i)*SMALL_RADIUS;

      WorldAddParticle(World, Part);
    }

  // Le haut de la boîte
  Part.Position.y = (2*BOX_SIZE_Y_2) * SMALL_RADIUS;
  for (i = -BOX_SIZE_X_2*2; i <= BOX_SIZE_X_2*2; ++i)
    {
      Part.Position.x = (double)(i)*SMALL_RADIUS;

      WorldAddParticle(World, Part);
    }

  // le côté gauche
  Part.Position.x = -(2*BOX_SIZE_X_2)*SMALL_RADIUS;
  for (i = -BOX_SIZE_Y_2*2+1; i <= BOX_SIZE_Y_2*2-1; ++i)
    {
      Part.Position.y = (double)(i)*SMALL_RADIUS;

      WorldAddParticle(World, Part);
    }

  // le côté droit
  Part.Position.x = (2*BOX_SIZE_X_2)*SMALL_RADIUS;
  for (i = -BOX_SIZE_Y_2*2+1; i <= BOX_SIZE_Y_2*2-1; ++i)
    {
      Part.Position.y = (double)(i)*SMALL_RADIUS;

      WorldAddParticle(World, Part);
    }

  // La grosse bille
  Part.InvMass = 1./BIG_MASS;
  Part.Radius = BIG_RADIUS;
  Part.Position.x = (float)(rand()%(int)(RAND_SCALE*BIG_RADIUS))/RAND_SCALE - BIG_RADIUS/2.;
  Part.Position.y = -(2*BOX_SIZE_Y_2-1) * SMALL_RADIUS + BIG_RADIUS/2. + (float)(rand()%(int)(RAND_SCALE*BIG_RADIUS))/RAND_SCALE;
  Part.Color = DrawGetColor(255, 0, 0);
  // On conserve un moyen d'y accéder par la suite
  BigParticleNum = World->ParticleCount;
  BigParticle = &World->Particles[WorldAddParticle(World, Part)];

  // Les autres
  Part.InvMass = 1./SMALL_MASS;
  Part.Radius = SMALL_RADIUS;
  Part.Position.x = -(2*BOX_SIZE_X_2-2)*SMALL_RADIUS;
  Part.Position.y = -(2*BOX_SIZE_Y_2-2) * SMALL_RADIUS;
  for (i = 0; i < SMALL_PART_COUNT; ++i)
    {
      Part.Color = DrawGetColor(0, i/2, 100);
      while (IsInBigParticle(Part.Position))
	NextPos(&Part.Position);
      TVector2D Pos = Part.Position;
      Part.Position.y += (float)(rand()%(int)(RAND_SCALE*SMALL_RADIUS))/2./RAND_SCALE - SMALL_RADIUS/4.;
      Part.Position.x += (float)(rand()%(int)(RAND_SCALE*SMALL_RADIUS))/2./RAND_SCALE - SMALL_RADIUS/4.;
      WorldAddParticle(World, Part);
      Part.Position = Pos;
      NextPos(&Part.Position);
    }
}

// Initialement permettait de calculer un pas dynamique pour l'intégration
//  (méthode abandonnée)
double ComputeStep(struct TWorld* World)
{
  double Result = STEP;
  int i;
  // On adapte le dt vis-à-vis de l'interpénétration des paires
  for (i = 0; i < World->PairCount; ++i)
    {
      TVector2D RelativeVelocity;
      Vect2DSub(World->Pairs[i].Part2->Velocity, World->Pairs[i].Part1->Velocity, &RelativeVelocity);
      double Dot = Vect2DDot(World->Pairs[i].ToVect, RelativeVelocity);
      // Si les particules se rapprochent, on adapte
      if (Dot < 0.)
	{
	  double Speed1 = Vect2DLength(World->Pairs[i].Part1->Velocity);
  	  double Speed2 = Vect2DLength(World->Pairs[i].Part1->Velocity);
	  double dt1 = (World->Pairs[i].Distance/Speed1)*0.1;
	  double dt2 = (World->Pairs[i].Distance/Speed2)*0.1;
	  if (Result > dt1)
	    Result = dt1;
	  if (Result > dt2)
	    Result = dt2;
	}
    }
  // On adapte la dt vis-à-vis de la vitesse de chaque particule
  for (i = 0; i < World->ParticleCount; ++i)
    {
      double Speed = Vect2DLength(World->Particles[i].Velocity);
      double dt = SMALL_RADIUS*0.1 / Speed;
      if (Result > dt)
	Result = dt;
    }
  return Result;
}

int main(int argc, char** argv)
{
  struct TWorld* World;
  TVector2D Gravity;
  int i;
  double Step;

  Gravity.x = 0.;
  Gravity.y = -GRAVITY;

  // Init des utilitaires
  // Draw (cf draw/draw.h) sert pour l'affichage
  // et permet un suivi visuel de la simulation
  DrawInit(800, 600, 32, 0.02);
  // World (cf world/world.h) gère les structures de la simulation
  // "VerletIntegration" est un pointeur sur la fonction à utiliser
  // pour l'intégration (ici méthode de Verlet)
  World = WorldCreate(IntegrationRK4(), CorrectorKernel, 2, TRUE, Gravity, PARTICLE_COUNT, LAMBDA);
  if (!World)
    {
      fprintf(stderr, "Could not create simulation frame. (Not enough memory ?)\n");
      DrawStop();
      return 0;
    }

  // Initialisation de la scène
  SceneCreate(World);

  char Text[200];
  int ImgCounter = 0;
  double Time = 0.;
  double Move = 0.;
  double Energy = 0.;

  // On prépare une sortie pour gnuplot
  FILE* Output;
  int failed = 1;
  char Name[255];
  i = 0;
  // On cherche un fichier non entamé
  while (failed)
    {
      sprintf(Name, "Output_%d.txt", i);
      Output = fopen(Name, "r");
      failed = Output != NULL;
      if (failed)
	fclose(Output);
      ++i;
    }
  Output = fopen(Name, "w");
  // En-tête dans le fichier :
  fprintf(Output, "# Big mass/little mass : %fl\n", BIG_MASS/SMALL_MASS);
  fprintf(Output, "# Big size/little size : %fl\n", BIG_RADIUS/SMALL_RADIUS);
  fprintf(Output, "# Amplitude : %fl\n", MOVE_AMPL);
  fprintf(Output, "# Frequency : %fl\n", MOVE_PULSE/(2.*M_PI));
  fprintf(Output, "# Big particle init pos (x,y) : (%fl,%fl)\n", BigParticle->Position.x, BigParticle->Position.y);

  int Continue = 1;
  DrawSetFont("./Justice.ttf", 50, 255, 255, 255);
  // Boucle principale de calcul
  while (DrawRun() && Continue)
    {
      DrawClear();

      Step = STEP;//ComputeStep(World);
     
      // Mise en place de l'excitation
      Move = MOVE_AMPL * sin(MOVE_PULSE * Time);
      for (i = 0; i < BigParticleNum; ++i)
	{
	  World->Particles[i].Position.x = World->Particles[i].InitPosition.x + Move;
	  World->Particles[i].Velocity.x = MOVE_PULSE*Move;
	}

      // On effectue le pas
      WorldStep(World, Step);

      // Calcul de l'énergie du système
      Energy = 0.;
      for (i = 0; i < World->ParticleCount; ++i)
	{
	  DrawParticle(World->Particles[i].Position, World->Particles[i].Radius, World->Particles[i].Color);
	  if (World->Particles[i].InvMass > EPSILON)
	      Energy += 1./2./World->Particles[i].InvMass * Vect2DSqLength(World->Particles[i].Velocity) + GRAVITY/World->Particles[i].InvMass*World->Particles[i].Position.y;
	}

      Time += Step;
      
      // Affichage d'information sur le système
      sprintf(Text, "Image : %d; Step : %lf; Time : %lf; Energy : %lf", ImgCounter, Step, Time, Energy);
      DrawText(0, 0, Text);

      // Mise à jour du fichier pour gnuplot
      fprintf(Output, "%lf %lf\n", Time, BigParticle->Position.y);

      // Fin automatique de la simulation si la particule atteint le haut
      // ou si le temps a été dépassé
      if (BigParticle->Position.y > (2*BOX_SIZE_Y_2) * SMALL_RADIUS - 4*BIG_RADIUS)
	Continue = 0;
      if (Time >= TIME_STOP)
      	Continue = 0;

      // Affichage effectif
      DrawFlip();
      ++ImgCounter;
    }

  printf("Achievement time : %lf\n", Time);

  // On quitte dans les règles
  fclose(Output);
  WorldDestroy(World);
  DrawStop();
  return 0;
}
