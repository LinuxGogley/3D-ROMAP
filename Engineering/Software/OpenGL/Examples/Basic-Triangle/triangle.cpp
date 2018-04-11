/* Using standard C++ output libraries */
#include <cstdlib>
#include <iostream>
using namespace std;

/* Use glew.h instead of gl.h to get all the GL prototypes declared */
#include <GL/glew.h>
/* Using SDL2 for the base window and OpenGL context init */
#include <SDL.h>

GLuint program;
GLint attribute_coord2d;

bool init_resources(void)
{ 
    GLint compile_ok = GL_FALSE, link_ok = GL_FALSE;

    GLuint vs = glCreateShader(GLD_VERTEX_SHADER);
    const char *vs_source =
	//"#version 100\n" // OpenGL ES 2.0
	"#version 120\n" // OpenGL 2.1
	"attribute vec2 coord2d;		    "
	"void main(void) {			    "
	"   gl_Position = vec4(coord2d, 0.0, 1.0);  "
	"}";
    glShaderSource(vs, 1, &vs_source, NULL);
    glCompiler(vs);
    glGetShaderiv(vs, GL_COMPILE_STATUS, &compile_ok);
    if (!compile_ok)
    {
	cerr << "Error in vertex shader" << endl;
	return false;
    }
    
    GLuint fs = glCreateShader(GL_FRAGMENT_SHADER);
    const char *fs_source =
	//"#version 100\n" // OpenGL ES 2.0
	"#version 120\n" // OpenGL 2.1
	"void main(void) {	    "
	"   gl_FragColor[0] = 0.0;  "
	"   gl_FragColor[1] = 0.0;  "
	"   gl_FragColor[2] = 1.0;  "
	"}";
    glShaderSource(fs, 1, &fs_source, NULL);
    glCompileShader(fs);
    glGetShaderiv(fs, GL_COMPILE_STATUS, &compile_ok);
    if (!compile_ok)
    {
	cerr << "Error in fragment shader" << endl;
	return false;
    }

    program = glCreateProgram();
    glAttachShader(program, vs);
    glAttachShader(program, fs);
    glLinkProgram(program);
    glGetProgramiv(program, GL_LINK_STATUS, &link_ok);
    if (!link_ok)
    {
	cerr << "Error in glLinkProgram" << endl;
	return false;
    }
    
    const char* attribute_name = "coord2d";
    attribute_coord2d = glGetAttribLocation(program, attribute_name);
    if (attribute_coord2d == -1)
    {
	cerr << "Could not bind attribute" << attribute_name << endl;
	return false;
    }

    return true;
}

void render(SDL_Window* window)
{ /* FILLED IN LATER */

}

void mainLoop(SDL_Window* window)
{
    while(true)
    {
	SDL_Event ev;
	while (SDL_PollEvent(&ev))
	{
	    if (ev.type == SDL_QUIT)
		return;
	}
	render(window);
    }
}

int main(int argc, char* argv[])
{
    /* SDL-related initializing functions */
    SDL_Init(SDL_INIT_VIDEO);
    SDL_Window* window = SDL_CreateWindow("My First Triangle",
	    SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
	    640, 480,
	    SDL_WINDOW_RESIZEABLE | SDL_WINDOW_OPENGL);
    SDL_GL_CreateContext(window);

    /* Entension wrangler intializing */
    GLenum glew_status = glewInit();
    if (glew_status != GLEW_OK)
    {
	cerr << "Error: glewInit: " << glewGetErrorString(glew_status) << endl;
	return EXIT_FAILURE;
    }

    /* When all init functions run without errors,
     * the program can initialize the resources */
    if (!init_resoures())
	return EXIT_FAILURE;

    /* We can display something if everything goes OK */
    mainLoop(window);

    /* If the program exits in the usual way,
     * free resources and exit with a success */
    free_resources();
    return EXIT_SUCCESS;
}
