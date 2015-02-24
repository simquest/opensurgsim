uniform mat3 osg_NormalMatrix; 
uniform mat4 osg_ModelViewProjectionMatrix; 
uniform mat4 osg_ModelViewMatrix;
uniform mat4 osg_ViewMatrix;

uniform vec3 _lightPosition;

uniform float constantAttenuation;
uniform float linearAttenuation;
uniform float quadraticAttenuation;

attribute vec4 osg_Vertex; 
attribute vec3 osg_Normal;

varying vec3 lightDir;
varying vec3 eyeDir;
varying vec3 reflectDir;
varying vec3 normalDir;

varying vec4 clipCoord;

varying vec3 specular;

varying float attenuation;

void main() 
{
    gl_Position = osg_ModelViewProjectionMatrix * osg_Vertex;

	clipCoord = gl_Position;
    
    vec4 eyeDir4 = osg_ModelViewMatrix * osg_Vertex;
	
	lightDir = (osg_ViewMatrix * vec4(_lightPosition, 1.0)).xyz - eyeDir4.xyz;
    
    float lightDistance = length(lightDir);
    float eyeDistance = length(eyeDir4.xyz);
    
    lightDir = normalize(lightDir);
    eyeDir = normalize(eyeDir4.xyz);
    
    normalDir = normalize(osg_NormalMatrix * osg_Normal);
    vec4 pos = osg_ModelViewMatrix * osg_Vertex;
    reflectDir = reflect(pos.xyz, normalDir);
    
    attenuation = 1.0 / (constantAttenuation + linearAttenuation*lightDistance + quadraticAttenuation*lightDistance*lightDistance);
}
