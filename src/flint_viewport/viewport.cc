
#include <iostream>
#include <chrono>
#include <glad/glad.h>
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

void Viewport::CaptureFrame(const FrameCaptureCallback& callback) {
    window->CaptureFrame(callback);
}

Viewport::Window::Window(const Viewport& viewport) : viewport(viewport) {

}

uint64_t Viewport::Window::GetFrameNumber() const {
    return frameNumber;
}

bool Viewport::Window::ShouldClose() const {
    return viewport.ShouldClose() || glfwWindowShouldClose(window);
}

bool Viewport::Window::IsInitialized() const {
    return initialized;
}

void Viewport::Window::Init(int width, int height) {
    glfwSetErrorCallback(error_callback);
    if (!glfwInit()) {
        exit(EXIT_FAILURE);
    }

	// glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	// glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	// glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	// glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);

    window = glfwCreateWindow(width, height, "Viewport", NULL, NULL);
    if (!window) {
        glfwTerminate();
        exit(EXIT_FAILURE);
    }

    glfwMakeContextCurrent(window);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cout << "Failed to initialize OpenGL context" << std::endl;
        exit(EXIT_FAILURE);
    }

    initialized = true;
}

void Viewport::Window::SwapBuffers() {
    ProcessCaptureRequests();
    frameNumber++;
    glfwSwapBuffers(window);
    glfwPollEvents();
}

GLFWwindow* Viewport::Window::GetGLFWWindow() const {
    return window;
}

void Viewport::Window::CaptureFrame(const FrameCaptureCallback& callback) {
    frameCaptureRequests.push_back(callback);
}

void Viewport::Window::ProcessCaptureRequests() {
    if (frameCaptureRequests.size() > 0) {
        int width, height;
        glfwGetWindowSize(window, &width, &height);

        std::vector<GLubyte> pixels(4 * width * height);

        glBindTexture(GL_TEXTURE_2D, 0);
        glPixelStorei(GL_PACK_ALIGNMENT, 1);
        glReadPixels(0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, pixels.data());

        GLenum error = glGetError();
        if (error) {
            std::cerr << "GL Error: " << error << std::endl;
        }

        for (const auto& request : frameCaptureRequests) {
            request(pixels.data(), width, height);
        }

        frameCaptureRequests.clear();
    }
}

Viewport::Window::~Window() {
    ProcessCaptureRequests();
    if (window) {
        glfwDestroyWindow(window);
    }
    glfwTerminate();
}

}
