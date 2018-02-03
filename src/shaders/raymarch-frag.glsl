#version 300 es

precision highp float;

in vec2 fs_Resolution;
in float fs_Time;

out vec4 out_Col;

float EPSILON = 0.01;
const float MIN_DIST = 0.0;
const float MAX_DIST = 100.0;
int MAX_MARCHING_STEPS = 24;

float udRoundBox( vec3 p, vec3 b, float r )
{
  return length(max(abs(p)-b,0.0))-r;
}

float udBox( vec3 p, vec3 b )
{
  return length(max(abs(p)-b,0.0));
}

float sdTorus82SDF( vec3 p, vec2 t )
{
  vec2 q = vec2(length(p.xz)-t.x,p.y);
  return length(q)-t.y;
}

float sphereSDF(vec3 p) {
    return length(p) - 1.0;
}

float sdTorus( vec3 p, vec2 t )
{
  vec2 q = vec2(length(p.xz)-t.x,p.y);
  return length(q)-t.y;
}

float opTwist( vec3 p )
{
    float c = cos(20.0*p.y);
    float s = sin(20.0*p.y);
    mat2  m = mat2(c,-s,s,c);
    vec3  q = vec3(m*p.xz,p.y);
    return sphereSDF(q);
}

float cubeSDF(vec3 p) {
    vec3 d = abs(p) - vec3(1.0, 1.0, 1.0);
    float insideDistance = min(max(d.x, max(d.y, d.z)), 0.0);
    float outsideDistance = length(max(d, 0.0));
    return insideDistance + outsideDistance;
}

float smin( float a, float b, float k )
{
  float h = clamp( 0.5+0.5*(b-a)/k, 0.0, 1.0 );
  return mix( b, a, h ) - k*h*(1.0-h);
}


float sdCappedCylinder( vec3 p, vec2 h )
{
  vec2 d = abs(vec2(length(p.xz),p.y)) - h;
  return min(max(d.x,d.y),0.0) + length(max(d,0.0));
}

float opBlend( vec3 p )
{
  float d1 = sphereSDF(p);
  float d2 = cubeSDF(p);
  return smin( d1, d2, 1.0f );
}








mat4 rotationMatrix(vec3 axis, float angle)
{
    axis = normalize(axis);
    float s = sin(radians(angle));
    float c = cos(radians(angle));
    float oc = 1.0 - c;
    
    return mat4(oc * axis.x * axis.x + c,           oc * axis.x * axis.y - axis.z * s,  oc * axis.z * axis.x + axis.y * s,  0.0,
                oc * axis.x * axis.y + axis.z * s,  oc * axis.y * axis.y + c,           oc * axis.y * axis.z - axis.x * s,  0.0,
                oc * axis.z * axis.x - axis.y * s,  oc * axis.y * axis.z + axis.x * s,  oc * axis.z * axis.z + c,           0.0,
                0.0,                                0.0,                                0.0,                                1.0);
}

mat4 translationMatrix(vec3 translation)
{
  return mat4(1,0,0,0,
              0,1,0,0,
              0,0,1,0,
              translation.x, translation.y, translation.z, 1);
}

struct Data {
  float SDV;
  vec3 color;
};

Data machine(vec3 p) {
  mat4 m = mat4(vec4(1,0,0,0), vec4(0,1,0,0), vec4(0,0,1,0), vec4(-13.8,-3,-15,1));
  float scale = 6.0;
  vec3 q = vec3(inverse(m)*vec4(p,1));

  mat4 m2 = mat4(vec4(1,0,0,0), vec4(0,1,0,0), vec4(0,0,1,0), vec4(1,1,1,1));
  float scale2 = 1.0;
  vec3 q2 = vec3(inverse(m2)*vec4(p,1));

  float t = min(sphereSDF(q2/scale2), udBox(q/scale, vec3(1.1,1.0,3.0)));
  return Data(t * scale, vec3(3,0,5));
}

Data object2(vec3 p) {
  return Data(sphereSDF(p), vec3(1,1,1));
}

Data leverBase(vec3 p) {
  mat4 m = translationMatrix(vec3(-7,-2,-2));
  float scale = 0.1;
  vec3 q = vec3(inverse(m)*vec4(p,1));
  return Data(udRoundBox(q, vec3(0.1,1.0,0.5), 0.1f), vec3(0,1,0));
}

Data lever(vec3 p) {
  mat4 m = translationMatrix(vec3(-6.9,-2,-2)) * 
            rotationMatrix(vec3(0,0,1), 55.0f + 45.f * 2.0 * clamp(sin(fs_Time/200.0f),0.f, 1.f)) * 
            translationMatrix(vec3(0,1,0));
  float scale = 0.1;
  vec3 q = vec3(inverse(m)*vec4(p,1));
  return Data(sdCappedCylinder(q/scale, vec2(1,10)) * scale, vec3(2,0,0));
}

Data leverTip(vec3 p) {
  mat4 m = translationMatrix(vec3(-5,-1,-0.3)) * 
          rotationMatrix(vec3(0,0,1), 55.0f + 45.f * 2.0 * clamp(sin(fs_Time/200.0f),0.f, 1.f)) * 
          translationMatrix(vec3(0,2,0));
  float scale = 0.5;
  vec3 q = vec3(inverse(m)*vec4(p,1));
  return Data(sphereSDF(q/scale) * scale, vec3(0,0,5));
}

Data sceneSDF(vec3 samplePoint) {
  //return min(sphereSDF(samplePoint), udRoundBox(samplePoint, vec3(0,0,0), 2.0f));
  //return udRoundBox(samplePoint, vec3(0,-1,0), 1.5f);
  //return sdTorus82SDF(samplePoint, vec2(0.1,0.9));
  //return sdTorus82SDF(samplePoint, vec2(1,0.5));
  //return opBlend(samplePoint);
  //return sphereSDF(samplePoint);

  // First object
  Data bestData = Data(1000000.f, vec3(0,0,0));

  Data machineData = machine(samplePoint);
  if(machineData.SDV < bestData.SDV) {
    bestData = machineData;
  }

  Data leverBaseData = leverBase(samplePoint);
  if(leverBaseData.SDV < bestData.SDV) {
    bestData = leverBaseData;
  }

  Data leverData = lever(samplePoint);
  if(leverData.SDV < bestData.SDV) {
    bestData = leverData;
  }

  leverData = lever(samplePoint);
  if(leverData.SDV < bestData.SDV) {
    bestData = leverData;
  }

  Data leverTipData = leverTip(samplePoint);
  if(leverTipData.SDV < bestData.SDV) {
    bestData = leverTipData;
  }

  return bestData;
}

Data raymarch(vec3 eye, vec3 marchingDirection, float start, float end) {
  float depth = start;
  for (int i = 0; i < MAX_MARCHING_STEPS; i++) {
    Data data = sceneSDF(eye + depth * marchingDirection);
    float dist = data.SDV;
    if (dist < EPSILON) {
      return Data(depth, data.color);
    }
    depth += dist;
    if (depth >= end) {
        return Data(end, vec3(0,0,0));
    }
  }
  return Data(end, vec3(0,0,0));
}

vec3 rayDirection(float fieldOfView, vec2 size, vec2 fragCoord) {
    vec2 xy = fragCoord - size / 2.0;
    float z = size.y / tan(radians(fieldOfView) / 2.0);
    return normalize(vec3(xy, -z));
}

vec3 estimateNormal(vec3 p) {
  float epsilon = 0.01;
  return normalize(vec3(
      sceneSDF(vec3(p.x + epsilon, p.y, p.z)).SDV - sceneSDF(vec3(p.x - epsilon, p.y, p.z)).SDV,
      sceneSDF(vec3(p.x, p.y + epsilon, p.z)).SDV - sceneSDF(vec3(p.x, p.y - epsilon, p.z)).SDV,
      sceneSDF(vec3(p.x, p.y, p.z  + epsilon)).SDV - sceneSDF(vec3(p.x, p.y, p.z - epsilon)).SDV
  ));
}

vec3 phongContribForLight(vec3 k_d, vec3 k_s, float alpha, vec3 p, vec3 eye, vec3 lightPos, vec3 lightIntensity) {
  vec3 N = estimateNormal(p);
  vec3 L = normalize(lightPos - p);
  vec3 V = normalize(eye - p);
  vec3 R = normalize(reflect(-L, N));
  
  float dotLN = dot(L, N);
  float dotRV = dot(R, V);
  
  if (dotLN < 0.0) {
    return vec3(0.0, 0.0, 0.0);
  } 
  
  if (dotRV < 0.0) {
    return lightIntensity * (k_d * dotLN);
  }
  return lightIntensity * (k_d * dotLN + k_s * pow(dotRV, alpha));
}

vec3 phongIllumination(vec3 k_a, vec3 k_d, vec3 k_s, float alpha, vec3 p, vec3 eye, vec3 diffuse) {
  vec3 ambientLight = 0.5 * diffuse;
  vec3 color = ambientLight * k_a;
  
  vec3 light1Pos = vec3(4.0 * sin(fs_Time/500.0f), 2.0, 4.0 * cos(fs_Time/500.0f));
  vec3 light1Intensity = vec3(0.4, 0.4, 0.4);
  
  color += phongContribForLight(k_d, k_s, alpha, p, eye, light1Pos, light1Intensity);
  
  vec3 light2Pos = vec3(2.0 * sin(0.37 * fs_Time/500.0f), 2.0 * cos(0.37 * fs_Time/500.0f), 2.0);
  vec3 light2Intensity = vec3(0.4, 0.4, 0.4);
  
  color += phongContribForLight(k_d, k_s, alpha, p, eye, light2Pos, light2Intensity);    
  return color;
}

mat4 viewMatrix(vec3 eye, vec3 center, vec3 up) {
  vec3 f = normalize(center - eye);
  vec3 s = normalize(cross(f, up));
  vec3 u = cross(s, f);
  return mat4(vec4(s, 0.0), vec4(u, 0.0), vec4(-f, 0.0), vec4(0.0, 0.0, 0.0, 1));
}

void main() {
	vec3 dir = rayDirection(50.0, vec2(fs_Resolution.x,fs_Resolution.y), vec2(gl_FragCoord.x, gl_FragCoord.y));
  vec3 eye = vec3(7.0, 5.0, 12.0);
  vec3 lightPosition = vec3(-10,-8,-18);

  mat4 viewToWorld = viewMatrix(eye, vec3(0.0, 0.0, -2.0), vec3(0.0, 1.0, 0.0));
  vec3 worldDir = (viewToWorld * vec4(dir, 0.0)).xyz;

  Data data = raymarch(eye, worldDir, MIN_DIST, MAX_DIST);
  float dist = data.SDV;
  
  if (dist > MAX_DIST - EPSILON) {
    out_Col = vec4(0,0,0,1.0);
    return;
  }

  vec3 p = eye + dist * worldDir;
  
  vec3 K_a = vec3(0.2, 0.2, 0.2);
  vec3 K_d = vec3(0.7, 0.2, 0.2);
  vec3 K_s = vec3(1.0, 1.0, 1.0);
  float shininess = 10.0;
  
  vec3 color = phongIllumination(K_a, K_d, K_s, shininess, p, eye, data.color);

  float fog = 1.0f / (1.0f + dist * dist * 0.1f);

  out_Col = vec4(fog * 90.f * color, 1.0f);

  //out_Col = vec4(1,1,0,1);
}
