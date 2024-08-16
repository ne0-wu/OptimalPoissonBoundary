#include "Mesh.h"

MyGL::Mesh::Mesh(const std::vector<Vertex> &vertices,
                 const std::vector<GLuint> &indices)
    : vertices(vertices), indices(indices)
{
    setup();
}

MyGL::Mesh::Mesh(const std::vector<glm::vec3> &vertices,
                 const std::vector<GLuint> &indices)
    : indices(indices)
{
    this->vertices.reserve(vertices.size());
    for (const auto &vertex : vertices)
        this->vertices.push_back({vertex, glm::vec3(0.0f), glm::vec2(0.0f)});

    setup();
}

MyGL::Mesh::~Mesh()
{
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteBuffers(1, &EBO);
}

void MyGL::Mesh::update()
{
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), vertices.data(), GL_STATIC_DRAW);
}

void MyGL::Mesh::draw() const
{
    glBindVertexArray(VAO);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
}

void MyGL::Mesh::draw_wireframe() const
{
    glBindVertexArray(VAO);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
}

void MyGL::Mesh::draw_vertices() const
{
    glBindVertexArray(VAO);
    glDrawElements(GL_POINTS, indices.size(), GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
}

void MyGL::Mesh::setup()
{
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);

    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), vertices.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(GLuint), indices.data(), GL_STATIC_DRAW);

    // Position
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void *)0);
    // Normal
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void *)offsetof(Vertex, normal));
    // TexCoords
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void *)offsetof(Vertex, tex_coords));

    glBindVertexArray(0);
}