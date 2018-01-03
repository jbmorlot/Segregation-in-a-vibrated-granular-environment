
#include "draw.h"
#include <SDL/SDL.h>
#ifdef USE_SDL_TTF
#include <SDL/SDL_ttf.h>
#endif
#include <math.h>

#define MIN(x,y) (x < y ? x : y)

SDL_Surface* Screen = NULL;
float MinRes_2;
int resx_2, resy_2;
#ifdef USE_SDL_TTF
TTF_Font *Font = NULL;
SDL_Color FontColor;
#endif

void DrawText(int x, int y, const char* Str)
{
#ifdef USE_SDL_TTF
  SDL_Surface* Text = NULL;
  SDL_Rect Pos;
  if (!Font)
    return;
  Text = TTF_RenderText_Blended(Font, Str, FontColor);
  Pos.x = x;
  Pos.y = y;
  SDL_BlitSurface(Text, NULL, Screen, &Pos);
  SDL_FreeSurface(Text);
#else
  printf("%s\n", Str);
#endif
}

void DrawParticle(TVector2D Position, float Radius, int Color)
{
  int x = (int)(MinRes_2 * Position.x) + resx_2;
  int y = (int)(MinRes_2 * (-Position.y)) + resy_2;
  int r = (int)(MinRes_2 * Radius);
  DrawDisc(x, y, r, Color);
}

void DrawDisc(int x, int y, int radius, int color)
{
  int i;
  for (i = - radius; i <= radius; ++i)
    {
      int h = (int)sqrt((float)(radius*radius - i*i));
      DrawVLine(x + i, y-h, y+h, color);
    }
}

void DrawVLine(int x, int y1, int y2, int color)
{
  int i;
  for (i = y1; i <= y2; ++i)
    DrawPixel(x, i, color);
}

void DrawPixel(int x, int y, int color)
{
  if (x >= 0 && x < Screen->w &&
      y >= 0 && y < Screen->h)
    *((int*)Screen->pixels + x + y * Screen->pitch/4) = color;
}

int DrawGetColor(int r, int g, int b)
{
  return SDL_MapRGB(Screen->format, r, g, b);
}

void DrawClear(void)
{
  SDL_FillRect(Screen, NULL, SDL_MapRGB(Screen->format, 0, 0, 0));
}

void DrawInit(int resx, int resy, int bpp, float scale)
{
  SDL_Init(SDL_INIT_VIDEO);
#ifdef USE_SDL_TTF
  TTF_Init();
#endif
  Screen = SDL_SetVideoMode(resx, resy, bpp, SDL_HWSURFACE);
  resx_2 = resx/2;
  resy_2 = resy/2;
  MinRes_2 = ((float)MIN(resx, resy)*scale)/2.;
  SDL_WM_SetCaption("Projet informatique : ségrégation par la taille", NULL);
}

int DrawRun(void)
{
  SDL_Event event;
  int ret = 1;
  while (SDL_PollEvent(&event))
    {
      switch (event.type)
	{
	case SDL_QUIT:
	  ret = 0;
	  break;
	}
    }
  return ret;
}

int DrawTime(void)
{
  static int Ticks = 0;
  int NewTicks = SDL_GetTicks();
  int Ret = NewTicks - Ticks;
  Ticks = NewTicks;
  return Ret;
}

void DrawFlip(void)
{
  SDL_Flip(Screen);
}

void DrawSetFont(const char* FontName, int Size, int R, int G, int B)
{
#ifdef USE_SDL_TTF
  if (Font)
    TTF_CloseFont(Font);
  Font = TTF_OpenFont(FontName, Size);
  FontColor.r = R;
  FontColor.g = G;
  FontColor.b = B;
#endif
}

void DrawStop(void)
{
#ifdef USE_SDL_TTF
  if (Font)
    TTF_CloseFont(Font);
  TTF_Quit();
#endif
  SDL_Quit();
}
