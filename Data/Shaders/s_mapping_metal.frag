uniform float SpecularPercent;
uniform float DiffusePercent;

uniform samplerCube SpecularEnvMap;
uniform samplerCube DiffuseEnvMap;
uniform sampler2D shadowMap;

uniform vec3 _lightColor;

uniform float osg_shine;

uniform vec3 osg_ambientColor;
uniform vec3 osg_specularColor;

varying vec3 lightDir;
varying vec3 eyeDir;
varying vec3 reflectDir;
varying vec3 normalDir;

varying vec3 specular;

varying vec4 clipCoord;

varying float attenuation;

void main()
{
	vec2 shadowCoord = clipCoord.xy / clipCoord.w * vec2(0.5) + vec2(0.5);
	float shadowAmount = 1.0 - texture2D(shadowMap, shadowCoord).r;

	vec3 ambientColor = osg_ambientColor;
	
	vec3 lightDir_normalized = normalize(lightDir);
	vec3 normalDir_normalized = normalize(normalDir);
	vec3 eyeDir_normalized = normalize(eyeDir);

    // Look up environment map values in cube maps
    vec3 diffuseColor = vec3(textureCube(DiffuseEnvMap,  normalDir)) * _lightColor * attenuation * shadowAmount;

    vec3 specularColor = vec3(textureCube(SpecularEnvMap, reflectDir)) * _lightColor * attenuation;
        
    // Add lighting to base color and mix
    vec3 color = mix(ambientColor, diffuseColor, DiffusePercent);
	
	float temp = max(dot(reflect(lightDir_normalized, normalDir_normalized), eyeDir_normalized), 0.0);
    float specular = temp / (osg_shine - temp * osg_shine + temp);
    //float specular = max(pow(dot(reflect(lightDir_normalized, normalDir_normalized), eyeDir_normalized), osg_shine), 0.0);
	
    color = mix(color, specularColor + color, SpecularPercent) + specular * osg_specularColor * _lightColor * attenuation * shadowAmount;

	gl_FragColor.rgb = color;
    gl_FragColor.a = 1.0;
}

