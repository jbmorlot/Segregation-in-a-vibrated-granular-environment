

#ifndef DEFINES_H
#define DEFINES_H

// Liste des grandeurs utilisées

#include <math.h>

#define TRUE 1
#define FALSE 0

#define MAX_INTEGRATION_PASS 4
#define TIME_STOP 120.                       // Permet de stopper automatiquement la simulation au bout d'un temps fixé
#define GRAVITY 1000                         // Valeur de g. Valeur ici arbitraire. Doit essentiellement être accordée
                                             //   avec les coefficients de réponses
#define RESPONSE_COEF 100000.                // Coefficient d'élasticité pour l'interaction entre deux boules (N/m)
#define BORDER_COEF 10.*RESPONSE_COEF        // On fixe un coefficient spécifique pour l'interaction avec le bord
                                             //   On évite ainsi les "effets tunels"
#define RAND_SCALE 50.                       // Distance caractéristique pour le positionnement aléatoire initial
#define STEP 1./1000.                        // pas d'intégration (s)
#define SMALL_RADIUS 1.                      // Rayon des petites boules
#define BIG_RADIUS 3.                        // Rayon de la grosse
#define BOX_SIZE_Y_2 20                      // Taille de la boite (divisé par 2) en ordonnée en nombre de petites boules
#define BOX_SIZE_X_2 10                      // Taille de la boite (divisé par 2) en abscisse en nombre de petites boules
#define SMALL_PART_COUNT 481                 // Nombre de petites boules à placer dans la boite
#define SMALL_MASS 1.                        // Masse des petites boules
#define BIG_MASS 3.                          // Masse de la grosse
#define PARTICLE_COUNT (SMALL_PART_COUNT + BOX_SIZE_Y_2*8 + BOX_SIZE_X_2*8 +1) // Nombre total de particules
#define MOVE_AMPL 15.                        // Amplitude des excitations
#define MOVE_PULSE (2.*M_PI*2.)              // Pulsation des excitations
#define KERNEL_SIZE 0.4                      // Taille des noyaux durs
#define LAMBDA 50.0                          // Coefficient de frottement fluide (Kg/s)

#endif // DEFINES_H

