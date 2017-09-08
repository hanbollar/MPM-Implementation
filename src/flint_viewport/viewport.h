
#pragma once

#include <cstdint>
#include <GLFW/glfw3.h>

namespace display {

class Viewport {
    public:

        class Window {
            public:
                Window() = delete;
                Window(const Viewport& viewport);
                uint64_t GetFrameNumber() const;
                bool ShouldClose() const;
                ~Window();

                void Init();
                void SwapBuffers();

            private:
                const Viewport& viewport;
                uint64_t frameNumber = 0;
                GLFWwindow* window;
        };

        Viewport();
        bool ShouldClose() const;
        void Close();
        ~Viewport();
        Window* GetWindow();

    private:
        bool shouldClose;
        Window* window;
};

}
