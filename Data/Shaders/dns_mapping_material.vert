uniform mat3 osg_NormalMatrix; 
uniform mat4 osg_ModelViewProjectionMatrix; 
uniform mat4 osg_ModelViewMatrix;
uniform mat4 osg_ViewMatrix;

uniform vec3 _lightPosition;
uniform vec3 _lightColor;

uniform float constantAttenuation;
uniform float linearAttenuation;
uniform float quadraticAttenuation;

uniform vec3 osg_diffuseColor;
uniform vec3 osg_specularColor;

attribute vec4 osg_Vertex; 
attribute vec3 osg_Normal;
attribute vec4 osg_MultiTexCoord0; 

attribute vec3 osg_tangent;
attribute vec3 osg_bitangent;

varying vec3 lightDir;
varying vec3 eyeDir;

varying vec2 texCoord0;
varying vec4 clipCoord;

varying vec3 diffuseColor;
varying vec3 specularColor;

void main(void) 
{
	gl_Position = osg_ModelViewProjectionMatrix * osg_Vertex;

	clipCoord = gl_Position;
	
	vec4 eyeDir4 = osg_ModelViewMatrix * osg_Vertex;
	eyeDir = eyeDir4.xyz;
 
	texCoord0 = osg_MultiTexCoord0.xy;
	
	vec3 n = normalize(osg_NormalMatrix * osg_Normal);
	vec3 t = normalize(osg_NormalMatrix * osg_tangent);
	vec3 b = normalize(osg_NormalMatrix * osg_bitangent);
	
	vec3 v;
	vec3 l = (osg_ViewMatrix * vec4(_lightPosition, 1.0)).xyz - eyeDir4.xyz;
    float lightDistance = length(l);
	v.x = dot(l, t);
	v.y = dot(l, b);
	v.z = dot(l, n);
	lightDir = normalize(v);
	
    float eyeDistance = length(eyeDir);
	v.x = dot(eyeDir, t);
	v.y = dot(eyeDir, b);
	v.z = dot(eyeDir, n);
	eyeDir = normalize(v);
    
    float attenuation = 1.0 / (constantAttenuation + linearAttenuation*lightDistance + quadraticAttenuation*lightDistance*lightDistance);
    
    diffuseColor = attenuation * osg_diffuseColor * _lightColor;	
	specularColor = attenuation * osg_specularColor * _lightColor;
} 
