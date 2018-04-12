/* Using standard C++ output libraries */
#include <cstdlib>
#include <iostream>
using namespace std;

/* Use glew.h instead of gl.h to get all the GL prototypes declared */
#include <GL/glew.h>
/* Using SDL2 for the base window and OpenGL context init */
#include <SDL2/SDL.h>

GLuint program;
GLint attribute_coord2d;

/**
 * Store all the file's contents in memory, useful to pass shaders
 * source code to OpenGL.  Using SDL_RWops for Android asset support.
 */
char* file_read(const char* filename)
{
    SDL_RWops *rw = SDL_RWFromFile(filename, "rb");
    if (rw == NULL) return NULL;

    Sint64 res_size = SDL_RWsize(rw);
    char* res = (char*)malloc(res_size + 1);

    Sint64 nb_read_total = 0, nb_read = 1;
    char* buf = res;
    
    while (nb_read_total < res_size && nb_read != 0)
    {
	nb_read = SDL_RWread(rw, buf, 1, (res_size - nb_read_total));
	nb_read_total += nb_read;
	buf += nb_read;
    }

    SDL_RWclose(rw);
    if (nb_read_total != res_size)
    {
	free(res);
	return NULL;
    }

    res[nb_read_total] = '\0';
    return res;
}

/*
 * Display compilation errors from the OpenGL shader compiler
 */

void print_log(GLuint object)
{
    GLint log_length = 0;
    if (glIsShader(object))
    {
	glGetShaderiv(object, GL_INFO_LOG_LENGTH, &log_length);
    }
    else if (glIsProgram(object))
    {
	glGetProgramiv(object, GL_INFO_LOG_LENGTH, &log_length);
    }
    else
    {
	cerr << "printlog: Not a shader or program" << endl;
	return;
    }

    char* log = (char*)malloc(log_length);

    if (glIsShader(object))
	glGetShaderInfoLog(object, log_length, NULL, log);
    else if (glIsProgram(object))
	glGetProgramInfoLog(object, log_length, NULL, log);

    cerr << log;
    free(log);
}

bool init_resources(void)
{ 
    GLint compile_ok = GL_FALSE, link_ok = GL_FALSE;

    GLuint vs = glCreateShader(GL_VERTEX_SHADER);
    const char *vs_source =
	//"#version 100\n" // OpenGL ES 2.0
	"#version 120\n" // OpenGL 2.1
	"attribute vec2 coord2d;		    "
	"void main(void) {			    "
	"   gl_Position = vec4(coord2d, 0.0, 1.0);  "
	"}";
    glShaderSource(vs, 1, &vs_source, NULL);
    glCompileShader(vs);
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
{ 
    /* Clear the background as white */
    glClearColor(1.0, 1.0, 1.0, 1.0);
    glClear(GL_COLOR_BUFFER_BIT);

    glUseProgram(program);
    glEnableVertexAttribArray(attribute_coord2d);
    GLfloat triangle_vertices[] = {
	0.0, 0.8,
	-0.8, -0.8,
	 0.8, -0.8,
    };

    /* Describe our vertices array to OpenGL
     * (it can't guess its format automatically) */
    glVertexAttribPointer
    (
        attribute_coord2d,  // attribute
	2,		    // number of elements per vertex, here (x,y)
	GL_FLOAT,	    // the type of each element
	GL_FALSE,	    // take our value as-is
	0,		    // no extra data between each position
	triangle_vertices   // pointer to the C array
    );

    /* Push each element in buffer_vertices to the vertex shader */
    glDrawArrays(GL_TRIANGLES, 0, 3);

    glDisableVertexAttribArray(attribute_coord2d);

    /* Display the result */
    SDL_GL_SwapWindow(window);
}

void free_resources()
{
    glDeleteProgram(program);
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
	    SDL_WINDOW_RESIZABLE | SDL_WINDOW_OPENGL);
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
    if (!init_resources())
	return EXIT_FAILURE;

    /* We can display something if everything goes OK */
    mainLoop(window);

    /* If the program exits in the usual way,
     * free resources and exit with a success */
    free_resources();
    return EXIT_SUCCESS;
}
