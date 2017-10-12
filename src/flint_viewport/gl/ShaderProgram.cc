
#include <iostream>
#include <vector>
#include "ShaderProgram.h"

namespace {

    bool checkCompileStatus(GLuint shaderId) {
        GLint compileFlag;
        glGetShaderiv(shaderId, GL_COMPILE_STATUS, &compileFlag);
        if (compileFlag == GL_FALSE) {
            GLint maxLength = 0;
            glGetShaderiv(shaderId, GL_INFO_LOG_LENGTH, &maxLength);
            std::vector<GLchar> errorLog(maxLength);
            glGetShaderInfoLog(shaderId, maxLength, &maxLength, &errorLog[0]);

            std::cerr << errorLog.data() << std::endl;
        }

        return compileFlag != GL_FALSE;
    }

    GLuint compileShader(GLenum type, const char* source, int length, bool* success = nullptr) {
        GLuint shaderId = glCreateShader(type);
        glShaderSource(shaderId, 1, &source, &length);
        glCompileShader(shaderId);
        bool compileSuccess = checkCompileStatus(shaderId);

        if (success) {
            *success = compileSuccess;
        }

        return shaderId;
    }
}

Shader::Shader(GLenum type, const std::string& source) 
    : shaderId(compileShader(type, source.c_str(), static_cast<int>(source.size()))) {
}

Shader::~Shader() {
    glDeleteShader(shaderId);
}

void ShaderProgram::LinkProgram() {
    glLinkProgram(shaderProgramId);

    GLint linkFlag = 0;
    glGetProgramiv(shaderProgramId, GL_LINK_STATUS, &linkFlag);
    if (linkFlag == GL_FALSE) {
        GLint maxLength = 0;
        glGetProgramiv(shaderProgramId, GL_INFO_LOG_LENGTH, &maxLength);
        std::vector<GLchar> errorLog(maxLength);
        glGetProgramInfoLog(shaderProgramId, maxLength, &maxLength, &errorLog[0]);

        std::cerr << errorLog.data() << std::endl;
    }
}

GLint ShaderProgram::Attribute(const char* name) const {
    return attributeLocations.at(name);
}

GLint ShaderProgram::Uniform(const char* name) const {
    return uniformLocations.at(name);
}

void ShaderProgram::Use() const {
    glUseProgram(shaderProgramId);
}

ShaderProgram::~ShaderProgram() {
    glDeleteProgram(shaderProgramId);
}