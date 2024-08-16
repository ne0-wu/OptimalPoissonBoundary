#include "PickVertex.h"

MyGL::PickVertex::PickVertex() : ShaderProgram()
{
    try
    {
        load_from_file("data/shaders/pick_vertex.vert", "data/shaders/pick_vertex.frag");
    }
    catch (const std::exception &e)
    {
        throw e;
    }
}

int MyGL::PickVertex::pick_vertex(int x, int y, const Mesh &mesh, const glm::mat4 mvp)
{
    glDisable(GL_MULTISAMPLE);
    glClearColor(0.f, 0.f, 0.f, 0.f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    use();
    set_uniform("mvp", mvp);

    // Draw all elements to update z-buffer
    glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
    mesh.draw();

    // Draw vertices to color buffer
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    glPointSize(10.f);
    mesh.draw_vertices();

    // Read pixel color and decode vertex index
    GLubyte pixel[4];
    glReadPixels(x, y, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, pixel);
    int index = pixel[0] + pixel[1] * 256 + pixel[2] * 256 * 256 - 1;

    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    return index;
}