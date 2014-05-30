#extension GL_ARB_texture_rectangle : enable

uniform sampler2DRect texture;

varying vec2 texCoord0;

void main(void) 
{	
	gl_FragColor = texture2DRect(texture, texCoord0);
}