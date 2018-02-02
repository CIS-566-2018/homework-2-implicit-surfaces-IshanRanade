#version 300 es

precision highp float;

in vec2 fs_Resolution;

out vec4 out_Col;

float EPSILON = 0.01;
const float MIN_DIST = 0.0;
const float MAX_DIST = 100.0;
int MAX_MARCHING_STEPS = 32;

float sphereSDF(vec3 p) {
    return length(p) - 1.0;
}

float sceneSDF(vec3 samplePoint) {
    return sphereSDF(samplePoint);
}

float raymarch(vec3 eye, vec3 marchingDirection, float start, float end) {
  float depth = start;
  for (int i = 0; i < MAX_MARCHING_STEPS; i++) {
    float dist = sceneSDF(eye + depth * marchingDirection);
    if (dist < EPSILON) {
      return depth;
    }
    depth += dist;
    if (depth >= end) {
        return end;
    }
  }
  return end;
}

vec3 rayDirection(float fieldOfView, vec2 size, vec2 fragCoord) {
    vec2 xy = fragCoord - size / 2.0;
    float z = size.y / tan(radians(fieldOfView) / 2.0);
    return normalize(vec3(xy, -z));
}

vec3 estimateNormal(vec3 p) {
  return normalize(vec3(
      sceneSDF(vec3(p.x + EPSILON, p.y, p.z)) - sceneSDF(vec3(p.x - EPSILON, p.y, p.z)),
      sceneSDF(vec3(p.x, p.y + EPSILON, p.z)) - sceneSDF(vec3(p.x, p.y - EPSILON, p.z)),
      sceneSDF(vec3(p.x, p.y, p.z  + EPSILON)) - sceneSDF(vec3(p.x, p.y, p.z - EPSILON))
  ));
}

vec3 blinnPhong(vec3 position, vec3 diffuseColor, vec3 lightPosition, vec3 eyePosition) {
  vec3 lightVec = normalize(lightPosition - position);
  vec3 eyeVec   = normalize(eyePosition); 
  vec3 normal   = estimateNormal(position);
  vec3 H        = normalize(lightVec + eyeVec) / 2.0f;

  float diffuse  = max(dot(normalize(normal), lightVec), 0.0f);
  float specular = max(pow(dot(H, normalize(normal)), 40.f), 0.0f);
  float ambient  = 0.2f;

  float lightIntensity = diffuse + ambient + specular;

  return diffuseColor * lightIntensity;
}

void main() {
	vec3 dir = rayDirection(45.0, vec2(fs_Resolution.x,fs_Resolution.y), vec2(gl_FragCoord.x, gl_FragCoord.y));
  vec3 eye = vec3(0.0, 0.0, 5.0);
  vec3 lightPosition = vec3(10,10,-10);

  float dist = raymarch(eye, dir, MIN_DIST, MAX_DIST);
  
  if (dist > MAX_DIST - EPSILON) {
    
    out_Col = vec4(0,0,0,1.0);
    return;
  }
  
  vec3 color = blinnPhong(dist * dir, vec3(1,0,0), lightPosition, eye);
  out_Col = vec4(color, 1.0);
}
