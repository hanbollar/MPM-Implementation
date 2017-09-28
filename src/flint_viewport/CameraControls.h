
#pragma once

#include <array>
#include <functional>
#include <GLFW/glfw3.h>
#include "flint/core/Camera.h"

template <typename T>
class CameraControls {

    private:
        static Camera<T>* currentCamera;
        static std::array<bool, GLFW_MOUSE_BUTTON_LAST + 1> buttons;

        static void mouseButtonCallback(GLFWwindow*, int button, int action, int) {
            buttons[button] = (action == GLFW_PRESS);
        }

        static void cursorPosCallback(GLFWwindow*, double mouseX, double mouseY) {
            static double oldX, oldY;
            float dX = static_cast<float>(mouseX - oldX);
            float dY = static_cast<float>(mouseY - oldY);
            oldX = mouseX;
            oldY = mouseY;

            if (buttons[2] || (buttons[0] && buttons[1])) {
                currentCamera->Pan(-dX * 0.002f, dY * 0.002f);
            } else if (buttons[0]) {
                currentCamera->Rotate(dX * -0.01f, dY * 0.01f);
            } else if (buttons[1]) {
                currentCamera->Zoom(dY * -0.005f);
            }
        }

        static void scrollCallback(GLFWwindow*, double, double yOffset) {
            currentCamera->Zoom(static_cast<float>(yOffset) * 0.04f);
        }

        static void SetCallbacks(GLFWwindow* window) {
            glfwSetMouseButtonCallback(window, &CameraControls::mouseButtonCallback);
            glfwSetCursorPosCallback(window, &CameraControls::cursorPosCallback);
            glfwSetScrollCallback(window, &CameraControls::scrollCallback);
        }

        Camera<T>& camera;
        GLFWwindow* window;

    public:
        CameraControls(Camera<T>& camera, GLFWwindow* window) : camera(camera), window(window) {
        }

        void SetCurrent() {
            currentCamera = &camera;
            SetCallbacks(window);
        }
};

template <typename T>
std::array<bool, GLFW_MOUSE_BUTTON_LAST + 1> CameraControls<T>::buttons = { 0 };

template <typename T>
Camera<T>* CameraControls<T>::currentCamera = nullptr;
