#include "SDL.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

static bool running = true;
static int dx = 0;
static int dy = 0;

void handleEvents()
{
	SDL_Event event;
	while (SDL_PollEvent(&event)) {
		switch(event.type) {
			case SDL_KEYDOWN:
				switch (event.key.keysym.sym) {
					case SDLK_LEFT:
						dx--;
						break;
					case SDLK_RIGHT:
						dx++;
						break;
					case SDLK_UP:
						dy--;
						break;
					case SDLK_DOWN:
						dy++;
						break;
					case SDLK_SPACE:
						dx = 0;
						dy = 0;
						break;
					case SDLK_ESCAPE:
						running = false;
						break;
					default:
						break;
				}
				break;
			case SDL_QUIT:
				running = false;
				break;
			default:
				break;
		}
	}
}

static const unsigned int pw = 256;
static const unsigned int ph = 256;

static unsigned int sx = 0;
static unsigned int sy = 0;

void paint(SDL_Surface *surface)
{
	typedef unsigned short Pixel;

	Pixel red = surface->format->Rmask;
	Pixel green = surface->format->Gmask;
	Pixel blue = surface->format->Bmask;
	Pixel white = red | green | blue;

	int width = surface->w;
	int height = surface->h;

	Pixel *line = surface->pixels;
	for (int y = 0; y < height; y++) {
		int py = (y + sy) % ph;
		int qy = py % (ph / 2);
		for (int x = 0; x < width; x++) {
			int px = (x + sx) % ph;
			int qx = px % (pw / 2);
			Pixel c = 0;
			if (qx ==   8 || qx == 120 || qy ==   8 || qy == 120
			 || qx ==  32 || qx ==  96 || qy ==  32 || qy ==  96
			 || qx ==  34 || qx ==  94 || qy ==  34 || qy ==  94) {
				if (px == qx && py == qy) {
					c = red;
				} else if (px == qx) {
					c = blue;
				} else if (py == qy) {
					c = green;
				} else {
					c = white;
				}
			}
			line[x] = c;
		}
		line = (Pixel *)((void *)line + surface->pitch);
	}

	sx = (pw + sx + dx) % pw;
	sy = (ph + sy + dy) % ph;
}

int main(int argc, char **argv)
{
	(void)argv; (void)argc;

	SDL_Init(SDL_INIT_VIDEO);
	atexit(SDL_Quit);

	SDL_ShowCursor(SDL_DISABLE);
	SDL_Surface *screen = SDL_SetVideoMode(
			320, 240, 16, SDL_HWSURFACE | SDL_DOUBLEBUF);
	if (screen->format->BitsPerPixel != 16) {
		fprintf(stderr, "ERROR: Did not get 16 bpp, got %u bpp instead.\n",
				screen->format->BitsPerPixel);
		exit(1);
	}
	if (!(screen->flags & SDL_HWSURFACE)) {
		fprintf(stderr, "WARNING: Did not get a hardware surface.\n");
	}
	if (!(screen->flags & SDL_DOUBLEBUF)) {
		fprintf(stderr, "WARNING: Did not get double buffering.\n");
	}

	Uint32 start = SDL_GetTicks();
	Uint32 lastPrint = start;
	int frameCount = 0;
	while (running) {
		SDL_LockSurface(screen);
		paint(screen);
		SDL_UnlockSurface(screen);
		SDL_Flip(screen);
		frameCount++;

		Uint32 now = SDL_GetTicks();
		if (now - lastPrint >= 1000) {
			float fps = frameCount / (0.001f * (now - lastPrint));
			printf("%3.2f frames per second\n", fps);
			lastPrint = now;
			frameCount = 0;
		}

		handleEvents();
	}

	return 0;
}
