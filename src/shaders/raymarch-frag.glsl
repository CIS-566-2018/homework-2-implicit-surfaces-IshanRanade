#version 300 es

precision highp float;

in vec2 fs_Resolution;
in float fs_Time;

out vec4 out_Col;

float EPSILON = 0.01;
const float MIN_DIST = 0.0;
const float MAX_DIST = 100.0;
int MAX_MARCHING_STEPS = 64;

float rand(const vec2 n) {
        return fract(sin(dot(n, vec2(12.9898, 4.1414))) * 43758.5453);
}

float noise(const vec2 n) {
        const vec2 d = vec2(0.0, 1.0);
  vec2 b = floor(n), f = smoothstep(vec2(0.0), vec2(1.0), fract(n));
        return mix(mix(rand(b), rand(b + d.yx), f.x), mix(rand(b + d.xy), rand(b + d.yy), f.x), f.y);
}

const int NUM_OCTAVES = 5;

// Fractal brownian motion implementation
float fbm(vec2 x) {
        float v = 0.0;
        float a = 0.5;
        vec2 shift = vec2(100);
        // Rotate to reduce axial bias
    mat2 rot = mat2(cos(0.5), sin(0.5), -sin(0.5), cos(0.50));
        for (int i = 0; i < NUM_OCTAVES; ++i) {
                v += a * noise(x);
                x = rot * x * 2.0 + shift;
                a *= 0.5;
        }
        return v;
}

// Convert pixel coordinates to uv coordinates
vec2 pixelToUV(const vec2 pixel) {
    return vec2(1.0 * pixel.x / fs_Resolution[0], 1.0 * pixel.y / fs_Resolution[1]);
}

// Convert uv coordinates to pixel coordinates
vec2 UVToPixel(const vec2 uv) {
    return vec2(1.0 * uv[0] * fs_Resolution[0], 1.0 * uv[1] * fs_Resolution[1]);
}


vec4 permute(vec4 x){return mod(((x*34.0)+1.0)*x, 289.0);}
vec4 taylorInvSqrt(vec4 r){return 1.79284291400159 - 0.85373472095314 * r;}
vec4 fade(vec4 t) {return t*t*t*(t*(t*6.0-15.0)+10.0);}

float cnoise(vec4 P){
  vec4 Pi0 = floor(P); // Integer part for indexing
  vec4 Pi1 = Pi0 + 1.0; // Integer part + 1
  Pi0 = mod(Pi0, 289.0);
  Pi1 = mod(Pi1, 289.0);
  vec4 Pf0 = fract(P); // Fractional part for interpolation
  vec4 Pf1 = Pf0 - 1.0; // Fractional part - 1.0
  vec4 ix = vec4(Pi0.x, Pi1.x, Pi0.x, Pi1.x);
  vec4 iy = vec4(Pi0.yy, Pi1.yy);
  vec4 iz0 = vec4(Pi0.zzzz);
  vec4 iz1 = vec4(Pi1.zzzz);
  vec4 iw0 = vec4(Pi0.wwww);
  vec4 iw1 = vec4(Pi1.wwww);

  vec4 ixy = permute(permute(ix) + iy);
  vec4 ixy0 = permute(ixy + iz0);
  vec4 ixy1 = permute(ixy + iz1);
  vec4 ixy00 = permute(ixy0 + iw0);
  vec4 ixy01 = permute(ixy0 + iw1);
  vec4 ixy10 = permute(ixy1 + iw0);
  vec4 ixy11 = permute(ixy1 + iw1);

  vec4 gx00 = ixy00 / 7.0;
  vec4 gy00 = floor(gx00) / 7.0;
  vec4 gz00 = floor(gy00) / 6.0;
  gx00 = fract(gx00) - 0.5;
  gy00 = fract(gy00) - 0.5;
  gz00 = fract(gz00) - 0.5;
  vec4 gw00 = vec4(0.75) - abs(gx00) - abs(gy00) - abs(gz00);
  vec4 sw00 = step(gw00, vec4(0.0));
  gx00 -= sw00 * (step(0.0, gx00) - 0.5);
  gy00 -= sw00 * (step(0.0, gy00) - 0.5);

  vec4 gx01 = ixy01 / 7.0;
  vec4 gy01 = floor(gx01) / 7.0;
  vec4 gz01 = floor(gy01) / 6.0;
  gx01 = fract(gx01) - 0.5;
  gy01 = fract(gy01) - 0.5;
  gz01 = fract(gz01) - 0.5;
  vec4 gw01 = vec4(0.75) - abs(gx01) - abs(gy01) - abs(gz01);
  vec4 sw01 = step(gw01, vec4(0.0));
  gx01 -= sw01 * (step(0.0, gx01) - 0.5);
  gy01 -= sw01 * (step(0.0, gy01) - 0.5);

  vec4 gx10 = ixy10 / 7.0;
  vec4 gy10 = floor(gx10) / 7.0;
  vec4 gz10 = floor(gy10) / 6.0;
  gx10 = fract(gx10) - 0.5;
  gy10 = fract(gy10) - 0.5;
  gz10 = fract(gz10) - 0.5;
  vec4 gw10 = vec4(0.75) - abs(gx10) - abs(gy10) - abs(gz10);
  vec4 sw10 = step(gw10, vec4(0.0));
  gx10 -= sw10 * (step(0.0, gx10) - 0.5);
  gy10 -= sw10 * (step(0.0, gy10) - 0.5);

  vec4 gx11 = ixy11 / 7.0;
  vec4 gy11 = floor(gx11) / 7.0;
  vec4 gz11 = floor(gy11) / 6.0;
  gx11 = fract(gx11) - 0.5;
  gy11 = fract(gy11) - 0.5;
  gz11 = fract(gz11) - 0.5;
  vec4 gw11 = vec4(0.75) - abs(gx11) - abs(gy11) - abs(gz11);
  vec4 sw11 = step(gw11, vec4(0.0));
  gx11 -= sw11 * (step(0.0, gx11) - 0.5);
  gy11 -= sw11 * (step(0.0, gy11) - 0.5);

  vec4 g0000 = vec4(gx00.x,gy00.x,gz00.x,gw00.x);
  vec4 g1000 = vec4(gx00.y,gy00.y,gz00.y,gw00.y);
  vec4 g0100 = vec4(gx00.z,gy00.z,gz00.z,gw00.z);
  vec4 g1100 = vec4(gx00.w,gy00.w,gz00.w,gw00.w);
  vec4 g0010 = vec4(gx10.x,gy10.x,gz10.x,gw10.x);
  vec4 g1010 = vec4(gx10.y,gy10.y,gz10.y,gw10.y);
  vec4 g0110 = vec4(gx10.z,gy10.z,gz10.z,gw10.z);
  vec4 g1110 = vec4(gx10.w,gy10.w,gz10.w,gw10.w);
  vec4 g0001 = vec4(gx01.x,gy01.x,gz01.x,gw01.x);
  vec4 g1001 = vec4(gx01.y,gy01.y,gz01.y,gw01.y);
  vec4 g0101 = vec4(gx01.z,gy01.z,gz01.z,gw01.z);
  vec4 g1101 = vec4(gx01.w,gy01.w,gz01.w,gw01.w);
  vec4 g0011 = vec4(gx11.x,gy11.x,gz11.x,gw11.x);
  vec4 g1011 = vec4(gx11.y,gy11.y,gz11.y,gw11.y);
  vec4 g0111 = vec4(gx11.z,gy11.z,gz11.z,gw11.z);
  vec4 g1111 = vec4(gx11.w,gy11.w,gz11.w,gw11.w);

  vec4 norm00 = taylorInvSqrt(vec4(dot(g0000, g0000), dot(g0100, g0100), dot(g1000, g1000), dot(g1100, g1100)));
  g0000 *= norm00.x;
  g0100 *= norm00.y;
  g1000 *= norm00.z;
  g1100 *= norm00.w;

  vec4 norm01 = taylorInvSqrt(vec4(dot(g0001, g0001), dot(g0101, g0101), dot(g1001, g1001), dot(g1101, g1101)));
  g0001 *= norm01.x;
  g0101 *= norm01.y;
  g1001 *= norm01.z;
  g1101 *= norm01.w;

  vec4 norm10 = taylorInvSqrt(vec4(dot(g0010, g0010), dot(g0110, g0110), dot(g1010, g1010), dot(g1110, g1110)));
  g0010 *= norm10.x;
  g0110 *= norm10.y;
  g1010 *= norm10.z;
  g1110 *= norm10.w;

  vec4 norm11 = taylorInvSqrt(vec4(dot(g0011, g0011), dot(g0111, g0111), dot(g1011, g1011), dot(g1111, g1111)));
  g0011 *= norm11.x;
  g0111 *= norm11.y;
  g1011 *= norm11.z;
  g1111 *= norm11.w;

  float n0000 = dot(g0000, Pf0);
  float n1000 = dot(g1000, vec4(Pf1.x, Pf0.yzw));
  float n0100 = dot(g0100, vec4(Pf0.x, Pf1.y, Pf0.zw));
  float n1100 = dot(g1100, vec4(Pf1.xy, Pf0.zw));
  float n0010 = dot(g0010, vec4(Pf0.xy, Pf1.z, Pf0.w));
  float n1010 = dot(g1010, vec4(Pf1.x, Pf0.y, Pf1.z, Pf0.w));
  float n0110 = dot(g0110, vec4(Pf0.x, Pf1.yz, Pf0.w));
  float n1110 = dot(g1110, vec4(Pf1.xyz, Pf0.w));
  float n0001 = dot(g0001, vec4(Pf0.xyz, Pf1.w));
  float n1001 = dot(g1001, vec4(Pf1.x, Pf0.yz, Pf1.w));
  float n0101 = dot(g0101, vec4(Pf0.x, Pf1.y, Pf0.z, Pf1.w));
  float n1101 = dot(g1101, vec4(Pf1.xy, Pf0.z, Pf1.w));
  float n0011 = dot(g0011, vec4(Pf0.xy, Pf1.zw));
  float n1011 = dot(g1011, vec4(Pf1.x, Pf0.y, Pf1.zw));
  float n0111 = dot(g0111, vec4(Pf0.x, Pf1.yzw));
  float n1111 = dot(g1111, Pf1);

  vec4 fade_xyzw = fade(Pf0);
  vec4 n_0w = mix(vec4(n0000, n1000, n0100, n1100), vec4(n0001, n1001, n0101, n1101), fade_xyzw.w);
  vec4 n_1w = mix(vec4(n0010, n1010, n0110, n1110), vec4(n0011, n1011, n0111, n1111), fade_xyzw.w);
  vec4 n_zw = mix(n_0w, n_1w, fade_xyzw.z);
  vec2 n_yzw = mix(n_zw.xy, n_zw.zw, fade_xyzw.y);
  float n_xyzw = mix(n_yzw.x, n_yzw.y, fade_xyzw.x);
  return 2.2 * n_xyzw;
}



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

float sdHexPrism( vec3 p, vec2 h )
{
    vec3 q = abs(p);
    return max(q.z-h.y,max((q.x*0.866025+q.y*0.5),q.y)-h.x);
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

  return Data(udBox(q/scale, vec3(1.1,1.0,3.0)) * scale, vec3(3,0,5));

  /*
  mat4 m2 = mat4(vec4(1,0,0,0), vec4(0,1,0,0), vec4(0,0,1,0), vec4(1,1,1,1));
  float scale2 = 1.0;
  vec3 q2 = vec3(inverse(m2)*vec4(p,1));

  float t = min(sphereSDF(q2/scale2), udBox(q/scale, vec3(1.1,1.0,3.0)));
  return Data(t * scale, vec3(3,0,5));*/
}

Data object(vec3 p) {
  mat4 mBase = mat4(vec4(1,0,0,0), vec4(0,1,0,0), vec4(0,0,1,0), vec4(4,0,0,1));
  float scaleBase = 1.0;
  vec3 qBase = vec3(inverse(mBase)*vec4(p,1));

  mat4 mMiddle = mat4(vec4(1,0,0,0), vec4(0,1,0,0), vec4(0,0,1,0), vec4(4,1.2,0,1));
  float scaleMiddle = 0.7;
  vec3 qMiddle = vec3(inverse(mMiddle)*vec4(p,1));

  mat4 mTop = mat4(vec4(1,0,0,0), vec4(0,1,0,0), vec4(0,0,1,0), vec4(4,2.1,0,1));
  float scaleTop = 0.4;
  vec3 qTop = vec3(inverse(mTop)*vec4(p,1));

  float baseT = sphereSDF(qBase/scaleBase);
  float middleT = sphereSDF(qMiddle/scaleMiddle);
  float topT = sphereSDF(qTop/scaleTop);

  return Data(min(min(baseT, middleT), topT), vec3(1,1,1));
}

Data hat(vec3 p) {

  float scale = clamp(sin(fs_Time/200.0f),0.f, 1.f);

  mat4 mBase = mat4(vec4(1,0,0,0), vec4(0,1,0,0), vec4(0,0,1,0), vec4(4,2.7 + 0.7*scale,0,1));
  float scaleBase = 1.0 * scale;
  vec3 qBase = vec3(inverse(mBase)*vec4(p,1));

  mat4 mTop = mat4(vec4(1,0,0,0), vec4(0,1,0,0), vec4(0,0,1,0), vec4(4,2.6,0,1)) *
    rotationMatrix(vec3(1,0,0), 90.f);
  float scaleTop = 1.0 * scale;
  vec3 qTop = vec3(inverse(mTop)*vec4(p,1));

  return Data(min(sdCappedCylinder(qBase, vec2(0.3f + 0.3 * scale, 0.9 * scale)), sdHexPrism(qTop, 
    vec2(clamp(1.5 * scale, 0.5f, 10.0f), 0.07))), vec3(0,0,0));
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

  Data leverTipData = leverTip(samplePoint);
  if(leverTipData.SDV < bestData.SDV) {
    bestData = leverTipData;
  }

  Data objectData = object(samplePoint);
  if(objectData.SDV < bestData.SDV) {
    bestData = objectData;
  }

  Data hatData = hat(samplePoint);
  if(hatData.SDV < bestData.SDV) {
    bestData = hatData;
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

Data sceneSDF2(vec3 p) {
  vec3 c = vec3(10,10,10) + vec3(5,4,5) * sin(fs_Time/1000.f) + vec3(1,1,1) * cos(fs_Time/2000.f);
  vec3 q = mod(p,c)-0.5*c;
  return Data(sphereSDF(q), vec3(0,1,1));
}

Data specialraymarch(vec3 eye, vec3 marchingDirection, float start, float end) {
  float depth = start;
  for (int i = 0; i < MAX_MARCHING_STEPS; i++) {
    Data data = sceneSDF2(eye + depth * marchingDirection);
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

vec2 sw(vec2 p) { return vec2(floor(p.x), floor(p.y)); }
vec2 se(vec2 p) { return vec2(ceil(p.x), floor(p.y)); }
vec2 nw(vec2 p) { return vec2(floor(p.x), ceil(p.y)); }
vec2 ne(vec2 p) { return vec2(ceil(p.x), ceil(p.y)); }

float smoothNoise(vec2 p) {

    vec2 interp = smoothstep(0., 1., fract(p));
    float s = mix(noise(sw(p)), noise(se(p)), interp.x);
    float n = mix(noise(nw(p)), noise(ne(p)), interp.x);
    return mix(s, n, interp.y);
        
}

float fractalNoise(vec2 p) {

    float x = 0.;
    x += smoothNoise(p      );
    x += smoothNoise(p * 2. ) / 2.;
    x += smoothNoise(p * 4. ) / 4.;
    x += smoothNoise(p * 8. ) / 8.;
    x += smoothNoise(p * 16.) / 16.;
    x /= 1. + 1./2. + 1./4. + 1./8. + 1./16.;
    return x;
            
}

float movingNoise(vec2 p) {
 
    float x = fractalNoise(p + fs_Time / 10000.0f);
    float y = fractalNoise(p - fs_Time / 10000.0f);
    return fractalNoise(p + vec2(x, y));   
    
}

// call this for water noise function
float nestedNoise(vec2 p) {
    
    float x = movingNoise(p);
    float y = movingNoise(p + 100.);
    return movingNoise(p + vec2(x, y));
    
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
    data = specialraymarch(eye, worldDir, MIN_DIST, MAX_DIST);
    dist = data.SDV;

    if (dist < MAX_DIST - EPSILON) {
        vec3 p = eye + dist * worldDir;
  
      vec3 K_a = vec3(0.5, 0.5, 0.5);
      vec3 K_d = vec3(0.7, 0.2, 0.2);
      vec3 K_s = vec3(1.0, 1.0, 1.0);
      float shininess = 10.0;
      
      vec3 color = phongIllumination(K_a, K_d, K_s, shininess, p, eye, data.color);

      float fog = 1.0f / (1.0f + dist * dist * 0.1f);

      out_Col = vec4(fog * 90.f * color, 1.0f);

      return;
    }


    float f = nestedNoise(gl_FragCoord.xy / fs_Resolution);
    vec3 color = vec3(f,f,f);
    out_Col = vec4(color,1.0);
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
