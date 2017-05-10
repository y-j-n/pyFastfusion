# Adapted PyOpenGL-Demo/GLUT/shader_test.py

from OpenGL.GL.ARB.shader_objects import *
from OpenGL.GL.ARB.vertex_shader import *
from OpenGL.GL.ARB.fragment_shader import *


def compileShader(source, shaderType):
    """Compile shader source of given type"""
    shader = glCreateShaderObjectARB(shaderType)
    print "glShaderSourceARB:", bool(glShaderSourceARB)
    glShaderSourceARB(shader, source)
    glCompileShaderARB(shader)
    return shader


def compileProgram(vertexSource=None, fragmentSource=None):
    program = glCreateProgramObjectARB()

    if vertexSource:
        vertexShader = compileShader(vertexSource, GL_VERTEX_SHADER_ARB)
        glAttachObjectARB(program, vertexShader)
    if fragmentSource:
        fragmentShader = compileShader(fragmentSource, GL_FRAGMENT_SHADER_ARB)
        glAttachObjectARB(program, fragmentShader)

    glBindAttribLocationARB(program, 0, 'in_position')
    glBindAttribLocationARB(program, 1, 'in_color')

    glValidateProgramARB(program)
    glLinkProgramARB(program)
    print 'shader_if: attrib loc of in_position: %s' % glGetAttribLocationARB(program, 'in_position')
    print 'shader_if: attrib loc of in_color: %s' % glGetAttribLocationARB(program, 'in_color')

    if vertexShader:
        glDeleteObjectARB(vertexShader)
    if fragmentShader:
        glDeleteObjectARB(fragmentShader)

    return program

#### osx is confined to glsl=120 !!
VERTEX_SHADER = """
#version 120
varying vec3 normal;

attribute vec3 in_position;
attribute vec3 in_color;

//http://stackoverflow.com/questions/13039439/transforming-glsl-150-to-120
//in vec4 position;  //failing, not for 120
//in vec4 color;  //failing, not for 120

void main() {
    //normal = gl_NormalMatrix * gl_Normal;
    //gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;

    gl_Position = gl_ModelViewProjectionMatrix * vec4(in_position, 1.);
    gl_FrontColor = vec4(in_color, 1.);
}
"""
#### fixme:
# VERTEX_SHADER = """
# #version 120
# in vec4 position;
# void main() {
#     gl_Position = position;
#     //gl_FrontColor = vec4(0, 0, 1, 1);
# }
# """

FRAGMENT_SHADER = """
#version 120
varying vec3 normal;
void main() {
/*
    float intensity;
    vec4 color;
    vec3 n = normalize(normal);
    vec3 l = normalize(gl_LightSource[0].position).xyz;

    // quantize to 5 steps (0, .25, .5, .75 and 1)
    intensity = (floor(dot(l, n) * 4.0) + 1.0)/4.0;
    color = vec4(intensity*1.0, intensity*0.5, intensity*0.5,
                 intensity*1.0);

    gl_FragColor = color;
*/

    //gl_FragColor = vec4(1, 0, 0, 1);
    gl_FragColor = gl_Color;  //fixme: this will not change non-attribute object colors!!!
}
"""
