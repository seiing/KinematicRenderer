#include "GLFWApp.h"
#include "Environment.h"

int main(int argc, char** argv) {
    MASS::Environment* env = new MASS::Environment(true);
    MASS::GLFWApp app(argc, argv);
    app.setEnv(env, argc, argv);
    app.startLoop();
}