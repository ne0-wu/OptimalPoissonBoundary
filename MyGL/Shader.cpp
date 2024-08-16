#include <fstream>
#include <sstream>

#include <stdexcept>

#include "Shader.h"

std::string MyGL::Shader::read_file_to_string(const std::string &file_path)
{
    std::ifstream file(file_path);
    if (!file.is_open())
        throw std::runtime_error("Failed to open file: " + file_path);

    file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
    try
    {
        std::stringstream stream;
        stream << file.rdbuf();
        return stream.str();
    }
    catch (const std::ifstream::failure &e)
    {
        throw std::runtime_error("Failed to read file: " + file_path + " (" + e.what() + ")");
    }
}

MyGL::Shader::Shader(GLenum shader_type)
    : type(shader_type)
{
    switch (type)
    {
    case GL_VERTEX_SHADER:
    case GL_FRAGMENT_SHADER:
    case GL_GEOMETRY_SHADER:
        break;
    default:
        throw std::runtime_error("Invalid shader type");
    }
    ID = glCreateShader(type);
}

MyGL::Shader::~Shader()
{
    glDeleteShader(ID);
}

void MyGL::Shader::compile_from_source(const std::string &source)
{
    const char *source_c_str = source.c_str();
    glShaderSource(ID, 1, &source_c_str, nullptr);
    glCompileShader(ID);

    int success;
    glGetShaderiv(ID, GL_COMPILE_STATUS, &success);

    if (!success)
    {
        char info_log[512];
        glGetShaderInfoLog(ID, 512, nullptr, info_log);
        throw std::runtime_error("Failed to compile shader: " + std::string(info_log));
    }
}

void MyGL::Shader::compile_from_file(const std::string &file_path)
{
    std::string source = read_file_to_string(file_path);
    try
    {
        compile_from_source(source);
    }
    catch (const std::exception &e)
    {
        throw std::runtime_error("Failed to compile shader from file: " + file_path + " (" + e.what() + ")");
    }
}

MyGL::ShaderProgram::ShaderProgram()
{
    ID = glCreateProgram();
}

MyGL::ShaderProgram::~ShaderProgram()
{
    glDeleteProgram(ID);
}

void MyGL::ShaderProgram::attach_shader(const Shader &shader)
{
    glAttachShader(ID, shader.get_ID());
}

void MyGL::ShaderProgram::link()
{
    glLinkProgram(ID);

    int success;
    glGetProgramiv(ID, GL_LINK_STATUS, &success);

    if (!success)
    {
        char info_log[512];
        glGetProgramInfoLog(ID, 512, nullptr, info_log);
        throw std::runtime_error("Failed to link program: " + std::string(info_log));
    }
}

void MyGL::ShaderProgram::load_from_file(const std::string &vertex_file_path,
                                         const std::string &fragment_file_path,
                                         const std::string &geometry_file_path)
{
    Shader vertex_shader(GL_VERTEX_SHADER);
    vertex_shader.compile_from_file(vertex_file_path);
    attach_shader(vertex_shader);

    Shader fragment_shader(GL_FRAGMENT_SHADER);
    fragment_shader.compile_from_file(fragment_file_path);
    attach_shader(fragment_shader);

    if (!geometry_file_path.empty())
    {
        Shader geometry_shader(GL_GEOMETRY_SHADER);
        geometry_shader.compile_from_file(geometry_file_path);
        attach_shader(geometry_shader);
    }

    link();
}

GLint MyGL::ShaderProgram::get_uniform_location(const std::string &name) const
{
    GLint location = glGetUniformLocation(ID, name.c_str());
    if (location == -1)
        throw std::runtime_error("Uniform " + name + " not found in shader program");
    return location;
}

void MyGL::ShaderProgram::set_uniform(const std::string &name, const bool &value) const
{
    glUniform1i(get_uniform_location(name), value);
}

void MyGL::ShaderProgram::set_uniform(const std::string &name, const int &value) const
{
    glUniform1i(get_uniform_location(name), value);
}

void MyGL::ShaderProgram::set_uniform(const std::string &name, const float &value) const
{
    glUniform1f(get_uniform_location(name), value);
}

void MyGL::ShaderProgram::set_uniform(const std::string &name, const glm::vec3 &value) const
{
    glUniform3fv(get_uniform_location(name), 1, &value[0]);
}

void MyGL::ShaderProgram::set_uniform(const std::string &name, const glm::vec4 &value) const
{
    glUniform4fv(get_uniform_location(name), 1, &value[0]);
}

void MyGL::ShaderProgram::set_uniform(const std::string &name, const glm::mat3 &value) const
{
    glUniformMatrix3fv(get_uniform_location(name), 1, GL_FALSE, &value[0][0]);
}

void MyGL::ShaderProgram::set_uniform(const std::string &name, const glm::mat4 &value) const
{
    glUniformMatrix4fv(get_uniform_location(name), 1, GL_FALSE, &value[0][0]);
}

void MyGL::ShaderProgram::set_MVP(const glm::mat4 &model, const glm::mat4 &view, const glm::mat4 &projection) const
{
    set_uniform("model", model);
    set_uniform("view", view);
    set_uniform("projection", projection);
}