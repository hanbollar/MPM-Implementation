
#pragma once

#include <map>
#include <glad/glad.h>

class Shader {
    friend class ShaderProgram;
    public:
        Shader() = delete;
        Shader(GLenum type, const std::string& source);
        ~Shader();
    
    private:
        GLuint shaderId;
};

class ShaderProgram {
    public:
        ShaderProgram() = delete;

        template <typename... Args>
        ShaderProgram(Args&& ...shaders) : shaderProgramId(glCreateProgram()) {
            AttachShaders(std::forward<Args>(shaders)...);
            LinkProgram();
        }
        
        template <typename... Args>
        void GetAttributeLocations(Args&& ...names) {
            GetShaderLocations(glGetAttribLocation, attributeLocations, std::forward<Args>(names)...);
        }

        template <typename... Args>
        void GetUniformLocations(Args&& ...names) {
            GetShaderLocations(glGetUniformLocation, uniformLocations, std::forward<Args>(names)...);
        }

        GLint Attribute(const char* name) const;
        GLint Uniform(const char* name) const;
        void Use() const;

        ~ShaderProgram();

    private:
        void AttachShaders(Shader& shader) {
            glAttachShader(shaderProgramId, shader.shaderId);
        }

        template <typename... Args>
        void AttachShaders(Shader& shader, Args&& ...shaders) {
            glAttachShader(shaderProgramId, shader.shaderId);
            AttachShaders(std::forward<Args>(shaders)...);
        }

        void GetShaderLocations(GLint(*getLocation)(GLuint, const char*), std::map<const char*, GLint>& locationMap, const char* name) {
            locationMap[name] = getLocation(shaderProgramId, name);
            if (locationMap[name] < 0) {
                std::cerr << "Could not get shader location " << name << std::endl;
            }
        }

        template <typename... Args>
        void GetShaderLocations(GLint(*getLocation)(GLuint, const char*), std::map<const char*, GLint>& locationMap, const char* name, Args&& ...names) {
            GetShaderLocations(getLocation, locationMap, name);
        }

        void LinkProgram();

        GLuint shaderProgramId;
        std::map<const char*, GLint> attributeLocations;
        std::map<const char*, GLint> uniformLocations;
};