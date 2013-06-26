varying vec4 vertColor;

/// Outputs the local normal direction as the vertex color
void main(void)
{
	gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
	vertColor.rgb = gl_Normal;
	vertColor.a = 1.0;
}