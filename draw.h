
#ifndef __DRAW_H
#define __DRAW_H

#include "vector2D.h"

// Ce fichier contient la gestion de l'affichage
// Comme cela ne concerne pas vraiment la simulation
// Tout a été déporté ici.

// Permet d'activer l'affichage des chaines via SDL_TTF
// Si on le commente, les chaines s'afficheront dans la console
//#define USE_SDL_TTF

// Création de la fenêtre avec une taille resx*resy
// bpp désigne le nombre de bits par couleur (usuellement 32)
// Scale est un facteur d'échelle pour l'affichage des boules
void DrawInit(int resx, int resy, int bpp, float scale);
// Efface le contenu de la fenêtre
void DrawClear(void);

// DrawRun doit être appelé régulièrement, elle renvoie 0 si l'utilisateur
// a demandé de quitter
int DrawRun(void);

// Récupère l'heure
int DrawTime(void);

// Fonction utilitaire permettant de récupérer le codage interne pour une couleur de SDL
int DrawGetColor(int r, int g, int b);

// Sélection des propriétés pour l'affichage de texte
void DrawSetFont(const char* Font, int Size, int R, int G, int B);

// Affiche une chaine (éventuellement dans la console si SDL_TTF est désactivée)
void DrawText(int x, int y, const char* Str);

// Affiche un pixel
void DrawPixel(int x, int y, int color);
// Dessine une ligne horizontale
void DrawVLine(int x, int y1, int y2, int color);
// Dessine un disque plein
void DrawDisc(int x, int y, int radius, int color);

// Fonction de haut-niveau pour le dessin des particules
void DrawParticle(TVector2D Position, float Radius, int Color);

// Rafraichi l'écran
void DrawFlip(void);

// Quitte proprement la SDL
void DrawStop(void);

#endif // __DRAW_H
