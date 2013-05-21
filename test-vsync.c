#include "SDL.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

static bool running = true;

void handleEvents()
{
	SDL_Event event;
	while (SDL_PollEvent(&event)) {
		switch(event.type) {
			case SDL_KEYDOWN:
				switch (event.key.keysym.sym) {
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

static unsigned int animationStep = 0;

void paint(SDL_Surface *surface)
{
	typedef unsigned short Pixel;

	Pixel red = surface->format->Rmask;
	Uint8 greenShift = surface->format->Gshift;
	Pixel greenMask = surface->format->Gmask;
	Uint8 blueShift = surface->format->Bshift;
	Pixel blueMask = surface->format->Bmask;

	int width = surface->w;
	int height = surface->h;

	Pixel *line = surface->pixels;
	for (int y = 0; y < height; y++) {
		Pixel c = red | ((y << blueShift) & blueMask);
#if 0
		// Write pixels one at a time.
		for (int x = 0; x < width; x++) {
			line[x] = c | (((x + animationStep) << greenShift) & greenMask);
		}
#else
		// Write two 16bpp pixels as one 32-bit value.
		for (int x = 0; x < width; x += 2) {
			Pixel p1 = c | (((x + 0 + animationStep) << greenShift) & greenMask);
			Pixel p2 = c | (((x + 1 + animationStep) << greenShift) & greenMask);
			((unsigned *)line)[x / 2] = p1 | ((unsigned)p2 << 16);
		}
#endif
		line = (Pixel *)((void *)line + surface->pitch);
	}

	animationStep++;
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
