/*!
  \file
  \brief Plotter (SDL)

  \~japanese
  \todo glDrawElements() を使うように修正する
  \todo MAX_POINTS の 1081 の数をセンサからの情報で初期化する
  \~english
  \todo Fix the code to use glDrawElements()
  \todo Use the sensor information instead of MAX_POINTS
  \~

  \author Satofumi KAMIMURA

  $Id$
*/

//#define USE_GL_2 1

#if defined(USE_GL_2)
#define GL_GLEXT_PROTOTYPES 1
#define GL3_PROTOTYPES 1
#endif
#include "plotter_sdl.h"
#include <SDL.h>
#include <SDL_opengl.h>


enum {
    SCREEN_WIDTH = 640,
    SCREEN_HEIGHT = 480,
};


typedef struct
{
    GLfloat x;
    GLfloat y;
} vector_t;


static SDL_Surface *screen_ = NULL;
static vector_t *points_ = NULL;
static size_t max_points_size_ = 0;
static size_t points_size_ = 0;
static double draw_magnify_ = 0.1;


static void opengl_initialize(void)
{
    int bpp = SDL_GetVideoInfo()->vfmt->BitsPerPixel;

    // Initialize the display
    int rgb_size[3];
    switch (bpp) {
    case 8:
        rgb_size[0] = 3;
        rgb_size[1] = 3;
        rgb_size[2] = 2;
        break;

    case 15:
    case 16:
        rgb_size[0] = 5;
        rgb_size[1] = 5;
        rgb_size[2] = 5;
        break;

    default:
        rgb_size[0] = 8;
        rgb_size[1] = 8;
        rgb_size[2] = 8;
        break;
    }
    SDL_GL_SetAttribute(SDL_GL_RED_SIZE, rgb_size[0]);
    SDL_GL_SetAttribute(SDL_GL_GREEN_SIZE, rgb_size[1]);
    SDL_GL_SetAttribute(SDL_GL_BLUE_SIZE, rgb_size[2]);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 16);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    //SDL_GL_SetAttribute(SDL_GL_SWAP_CONTROL, 1);
}


static void opengl_setup(void)
{
    glPushAttrib(GL_ENABLE_BIT);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glEnable(GL_TEXTURE_2D);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glViewport(-1, -1, +1, +1);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    glOrtho(-1.0, +1.0, -1.0, +1.0, -10.0, +10.0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glShadeModel(GL_SMOOTH);
}


static void draw_points(void)
{

#if !defined(USE_GL_2)
    size_t i;

    glBegin(GL_POINTS);
    for (i = 0; i < points_size_; ++i) {
        glVertex2i(points_[i].x, points_[i].y);
    }
    glEnd();

    points_size_ = 0;

#else
    int memory_size = points_size * sizeof(points[0]);

    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_COLOR_ARRAY);

    glBindBuffer(GL_ARRAY_BUFFER, buffer_id);
    glBufferData(GL_ARRAY_BUFFER, memory_size, points, GL_STATIC_DRAW);

    glInterleavedArrays(GL_V2F, 0, NULL);
    glDrawArrays(GL_POINTS, 0, points_size);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glDisableClientState(GL_COLOR_ARRAY);
    glDisableClientState(GL_VERTEX_ARRAY);

    points_size = 0;
#endif
}


static void enter2D(void)
{
    glViewport(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT);

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();

    glOrtho(0.0, SCREEN_WIDTH - 1.0, 0.0, SCREEN_HEIGHT - 1.0, 0.0, 1.0);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);

    glTranslatef(SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2, 0.0);
}


bool plotter_initialize(int data_size)
{
    points_ = malloc(data_size * sizeof(vector_t));
    if (!points_) {
        return false;
    }
    max_points_size_ = data_size;

    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        printf("SDL_Init: %s\n", SDL_GetError());
        return false;
    }

    // \~japanese 画面の作成
    // \~english Prepares the display screeen
    opengl_initialize();
    screen_ = SDL_SetVideoMode(SCREEN_WIDTH, SCREEN_HEIGHT, 0, SDL_OPENGL);
    if (!screen_) {
        return false;
    }
    opengl_setup();

    // \~japanese 描画設定
    // \~english Prepares for drawing
    glPointSize(2.0);
#if defined(USE_GL_2)
    glGenBuffers(1, &buffer_id);
#endif
    enter2D();

    // \~japanese データの確保
    // \~english Reserves data

    return true;
}


void plotter_terminate(void)
{
    SDL_Quit();
}


void plotter_clear(void)
{
    glClearColor(0x00, 0x00, 0x00, 0xff);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}


void plotter_swap(void)
{
    // \~japanese 表示を入れ換えるときに、まだ描画していない内容を描画する
    // \~english Before swapping buffers, finish any pending drawing
    draw_points();

    SDL_GL_SwapBuffers();
}


void plotter_set_color(unsigned char r, unsigned g, unsigned b)
{
    // \~japanese 色を変更するときに、まとめて描画を行う
    // \~english Before changing color, finish any pending drawing
    draw_points();

    glColor3f(r / 255.0, g / 255.0, b / 255.0);
}


void plotter_plot(float x, float y)
{
    if (points_size_ >= max_points_size_) {
        return;
    }

    points_[points_size_].x = draw_magnify_ * x;
    points_[points_size_].y = draw_magnify_ * y;
    ++points_size_;
}


bool plotter_is_quit(void)
{
    bool is_quit = false;
    int magnify = 0;

    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        switch (event.type) {

        case SDL_QUIT:
            is_quit = true;
            break;

        case SDL_KEYDOWN:
            if ((event.key.keysym.sym == SDLK_q) ||
                (event.key.keysym.sym == SDLK_F4)) {
                is_quit = true;
            }
            if (event.key.keysym.sym == SDLK_COMMA) {
                --magnify;
            }
            if (event.key.keysym.sym == SDLK_PERIOD) {
                ++magnify;
            }
            break;

        case SDL_MOUSEBUTTONDOWN:
            if (event.button.button == SDL_BUTTON_WHEELUP) {
                --magnify;
            } else if (event.button.button == SDL_BUTTON_WHEELDOWN) {
                ++magnify;
            }
            break;
        }
    }

    // \~japanese 描画の拡大率を変更する
    // \~english Changes the zooming rate
    while (magnify < 0) {
        draw_magnify_ *= 0.90;
        ++magnify;
    }
    while (magnify > 0) {
        draw_magnify_ *= 1.10;
        --magnify;
    }
    if (draw_magnify_ < 0.001) {
        draw_magnify_ = 0.001;
    } else if (draw_magnify_ > 10.0) {
        draw_magnify_ = 10.0;
    }

    return is_quit;
}
