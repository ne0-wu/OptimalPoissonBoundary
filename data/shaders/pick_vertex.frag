#version 330

flat in int vertexId;

out vec4 FragColor;

void main()
{
    int id = vertexId + 1;
    FragColor = vec4( (id & 0x000000FF) >> 0, (id & 0x0000FF00) >> 8, (id & 0x00FF0000) >> 16, 255.f) / 255.f;
}