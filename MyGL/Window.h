#pragma once

#include <string>
#include <tuple>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include "Camera.h"

namespace MyGL
{
    class Window
    {
    public:
        Window(int width = 1280, int height = 720, std::string title = "Window");
        ~Window();

        std::tuple<int, int> get_framebuffer_size() const;

        bool should_close() const;
        void swap_buffers() const;
        void poll_events() const;

        void process_input() const;
        void process_input_camera(Camera &camera, float delta_time) const;

    private:
        GLFWwindow *window;

        void setup_GLFW(int width, int height, std::string title);
        void setup_GLAD() const;
        void setup_ImGui() const;

        static void framebuffer_size_callback(GLFWwindow *window, int width, int height);
    };
}