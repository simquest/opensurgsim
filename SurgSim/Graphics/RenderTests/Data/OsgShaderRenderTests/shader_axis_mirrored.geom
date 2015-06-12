#version 150
#extension GL_EXT_gpu_shader4 : enable
#extension GL_EXT_geometry_shader4 : enable

layout(triangles) in;
layout(triangle_strip, max_vertices=6) out;

in vec4 vertColor[3];
out vec4 geomColor;

/// Outputs the input geometry plus it's mirror in the X-axis
void main()
{
	for (int i = 0; i < gl_VerticesIn; ++i)
	{
		gl_Position = gl_PositionIn[i];
		geomColor = vertColor[i];
		EmitVertex();
	}
	EndPrimitive();
	for (int i = 0; i < gl_VerticesIn; ++i)
	{
		vec4 vertex = gl_PositionIn[i];
		vertex.x = - vertex.x;
		gl_Position = vertex;
		geomColor = vertColor[i];
		EmitVertex();
	}
	EndPrimitive();
}