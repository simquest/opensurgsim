varying vec2 texCoord;

void main(void) {
  gl_Position = gl_Vertex;
  texCoord = (vec2(gl_Vertex.x, gl_Vertex.y) + 1.0) / 2.0;
}
