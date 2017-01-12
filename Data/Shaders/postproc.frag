uniform sampler2D renderTarget;
uniform float offset;
varying vec2 texCoord;

void main(void) {
  vec2 texcoord = texCoord;
  // texcoord.x += sin(texcoord.y * 4.0*2.0*3.14159 + offset) / 100.0;
  // gl_FragColor = texture2D(fbo_texture, texcoord);
  gl_FragColor = (texcoord.x < 0.1 || texcoord.x > 0.9 ||
              texcoord.y < 0.1 || texcoord.y > 0.9) ? vec4(0.9, 0.0, 0.0, 0.5)
 : texture2D(renderTarget, texCoord);
}
