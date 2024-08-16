#include "Shader.h"
#include "Mesh.h"

namespace MyGL
{
    class PickVertex : public ShaderProgram
    {
    public:
        PickVertex();
        int pick_vertex(int x, int y, const Mesh &mesh, const glm::mat4 mvp);
    };
}