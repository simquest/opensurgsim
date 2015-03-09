uniform sampler2D diffuseMap;
uniform sampler2D normalMap;
uniform sampler2D shadowMap;

uniform vec3 _lightColor;

uniform vec3 osg_ambientColor;

uniform float osg_shine;

varying vec3 lightDir;
varying vec3 eyeDir;

varying vec2 texCoord0;
varying vec4 clipCoord;

varying vec3 diffuseColor;
varying vec3 specularColor;

void main(void) 
{	
	vec2 shadowCoord = clipCoord.xy / clipCoord.w * vec2(0.5) + vec2(0.5);
	float shadowAmount = 1.0 - texture2D(shadowMap, shadowCoord).r;

	vec3 normalDir = texture2D(normalMap, texCoord0).rgb * 2.0 - 1.0;
	normalDir.g = -normalDir.g;

	vec3 vAmbient = osg_ambientColor * _lightColor;
    
	//vec3 lightDir_normalized = normalize(lightDir);
	//vec3 normalDir_normalized = normalize(normalDir);
	//vec3 eyeDir_normalized = normalize(eyeDir);

	float diffuse = max(dot(lightDir, normalDir), 0.0);
	
	vec3 vDiffuse = diffuseColor * diffuse * shadowAmount;	
 
    float temp = max(dot(reflect(lightDir, normalDir), eyeDir), 0.0);
    float specular = temp / (osg_shine - temp * osg_shine + temp);
    //float specular = max(pow(dot(reflect(lightDir, normalDir), eyeDir), osg_shine), 0.0);
    
	vec3 vSpecular = specularColor * specular * shadowAmount;		

	vec3 base = texture2D(diffuseMap, texCoord0).rgb;
	vec3 color = (vAmbient + vDiffuse) * base + vSpecular;

	gl_FragColor.rgb = color;
	gl_FragColor.a = 1.0;
}