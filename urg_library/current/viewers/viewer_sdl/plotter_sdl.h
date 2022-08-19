#ifndef PLOTTER_SDL_H
#define PLOTTER_SDL_H

/*!
  \file
  \brief Plotter (SDL)

  \author Satofumi KAMIMURA

  $Id: plotter_sdl.h,v 2a8d923c70f3 2010/08/08 12:22:29 Satofumi $
*/

#include <stdbool.h>


extern bool plotter_initialize(int data_size);
extern void plotter_terminate(void);
extern void plotter_clear(void);
extern void plotter_swap(void);
extern void plotter_set_color(unsigned char r, unsigned g, unsigned b);
extern void plotter_plot(float x, float y);
extern bool plotter_is_quit(void);

#endif /* !PLOTTER_SDL_H */
