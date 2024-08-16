#pragma once

#include <vector>
#include <string>

#include <glad/glad.h>
#include <glm/glm.hpp>

namespace MyGL
{
    class PointCloud
    {
    public:
        PointCloud(const std::vector<glm::vec3> &vertices);

        ~PointCloud();

        void update();

        void draw();

    private:
        GLuint VAO, VBO;

        const std::vector<glm::vec3> &vertices;

        void setup();
    };
}