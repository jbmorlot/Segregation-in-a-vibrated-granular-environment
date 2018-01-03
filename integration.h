
#ifndef __INTEGRATION_H
#define __INTEGRATION_H

#include "particle.h"

// La structure des méthodes d'intégration esrt un peu compliquée
// Étant donné la taille des données à gérer, une méthode paut avoir besoin
// de plusieurs "passes". Par exemple, pour la méthode RK2, deux points sont prélevés

// Ici, c'est le simulateur qui appelle la méthode autant de fois que nécessaire
// La structure permet de modifier la fonction appelée suivant le numéro de la passe
// courante

// Note : cette méthode n'est possible que parce que les équations sont autonomes.

struct TWorld;
typedef void (*TIntegrationFunc)(double Step, struct TWorld* World);
typedef struct
{
  int PassCount;
  TIntegrationFunc Func;
} TIntegrationMethod;

#endif // __INTEGRATION_H
