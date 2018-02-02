#version 300 es

precision highp float;

in vec4 vs_Pos;
uniform vec2 u_Resolution;
uniform float u_Time;

out vec2 fs_Resolution;
out float fs_Time;

void main() {
	// TODO: Pass relevant info to fragment
	gl_Position = vs_Pos;
  fs_Resolution = u_Resolution;
  fs_Time = u_Time;
}
