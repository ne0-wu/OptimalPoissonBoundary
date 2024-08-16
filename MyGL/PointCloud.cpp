#include "PointCloud.h"

MyGL::PointCloud::PointCloud(const std::vector<glm::vec3> &vertices)
    : vertices(vertices)
{
    setup();
}

MyGL::PointCloud::~PointCloud()
{
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
}

void MyGL::PointCloud::update()
{
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(glm::vec3), vertices.data(), GL_STATIC_DRAW);
}

void MyGL::PointCloud::draw()
{
    glBindVertexArray(VAO);
    glDrawArrays(GL_POINTS, 0, vertices.size());
    glBindVertexArray(0);
}

void MyGL::PointCloud::setup()
{
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);

    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(glm::vec3), vertices.data(), GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void *)0);

    glBindVertexArray(0);
}