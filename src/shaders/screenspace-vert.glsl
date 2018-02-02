#version 300 es

precision highp float;

in vec4 vs_Pos;
uniform vec2 u_Resolution;

out vec2 fs_Resolution;

void main() {
	// TODO: Pass relevant info to fragment
	gl_Position = vs_Pos;
  fs_Resolution = u_Resolution;
}
