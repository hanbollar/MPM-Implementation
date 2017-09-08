
#include <iostream>
#include <chrono>
#include "viewport.h"

namespace display {

    namespace {
        void error_callback(int error, const char* description) {
          fprintf(stderr, "Error: %s\n", description);
        }
    }

Viewport::Viewport() : shouldClose(false), window(new Viewport::Window(*this)) {
}

Viewport::Window* Viewport::GetWindow() {
    return window;
}

bool Viewport::ShouldClose() const {
    return shouldClose;
}

void Viewport::Close() {
    shouldClose = true;
}

Viewport::~Viewport() {
    delete window;
}

Viewport::Window::Window(const Viewport& viewport) : viewport(viewport) {

}

uint64_t Viewport::Window::GetFrameNumber() const {
    return frameNumber;
}

bool Viewport::Window::ShouldClose() const {
    return viewport.ShouldClose() || glfwWindowShouldClose(window);
}

void Viewport::Window::Init() {
    glfwSetErrorCallback(error_callback);
    if (!glfwInit()) {
        exit(EXIT_FAILURE);
    }

    window = glfwCreateWindow(640, 480, "Viewport", NULL, NULL);
    if (!window) {
        glfwTerminate();
        exit(EXIT_FAILURE);
    }

    glfwMakeContextCurrent(window);
}

void Viewport::Window::SwapBuffers() {
    frameNumber++;
    glfwSwapBuffers(window);
    glfwPollEvents();
}

Viewport::Window::~Window() {
    if (window) {
        glfwDestroyWindow(window);
    }
    glfwTerminate();
}

}
