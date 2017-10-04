
#pragma once

#include <cstdint>
#include <GLFW/glfw3.h>
#include <functional>
#include <vector>

namespace display {

class Viewport {
    public:

        using FrameCaptureCallback = std::function<void(const GLubyte* pixels, int width, int height)>;

        class Window {
            public:
                Window() = delete;
                Window(const Viewport& viewport);
                uint64_t GetFrameNumber() const;
                bool ShouldClose() const;
                bool IsInitialized() const;
                ~Window();

                void Init(int width, int height);
                void SwapBuffers();
                GLFWwindow* GetGLFWWindow() const;

                void CaptureFrame(const FrameCaptureCallback& callback);
                void ProcessCaptureRequests();

            private:
                const Viewport& viewport;
                uint64_t frameNumber = 0;
                bool initialized = false;
                GLFWwindow* window;
                std::vector<FrameCaptureCallback> frameCaptureRequests;
        };

        Viewport();
        bool ShouldClose() const;
        void Close();
        ~Viewport();
        Window* GetWindow();

        void CaptureFrame(const FrameCaptureCallback& callback);

    private:
        bool shouldClose;
        Window* window;
};

}
