#include "noise.h"
#include "../gameshared/q_math.h"

constexpr float F2 = 0.36602540378443f; // 0.5f * ( std::sqrtf(3.0f) - 1.0f );
constexpr float G2 = 0.21132486540518f; // ( 3 - std::sqrtf(3.0f)) / 6;

constexpr float F3 = 0.33333333333333f; // 1/3
constexpr float G3 = 0.16666666666666f; // 1/6

constexpr vec3_t grad3[]= {{1,1,0},{-1,1,0},{1,-1,0},{-1,-1,0},
                            {1,0,1},{-1,0,1},{1,0,-1},{-1,0,-1},
                            {0,1,1},{0,-1,1},{0,1,-1},{0,-1,-1}};
//supplementary gradient for quick computation of curl noise
constexpr vec3_t grad32[] = {{1,-1,0},{-1,-1,0},{0,1,-1},{-1,1,0},
                             {-1,0,-1},{1,0,1},{1,1,-1},{0,-1,1},
                             {0,1,1},{1,-1,1},{0,-1,-1},{-1,0,1}};

int perm[] = {11, 8, 10, 2, 7, 10, 4, 10, 3, 2, 10, 0, 7, 7, 2, 1, 4, 5, 3, 7, 2, 9, 6, 7, 1, 8, 0, 6, 4, 8, 7, 5, 6, 0, 7, 1, 1, 4, 6, 11, 6, 8, 7, 11, 9, 7, 9, 11, 8, 9, 2, 8, 9, 5, 5, 10, 9, 6, 6, 6, 8, 4, 0, 6, 4, 4, 1, 3, 5, 3, 6, 2, 9, 5, 6, 1, 11, 5, 2, 10, 1, 1, 0, 8, 8, 6, 9, 3, 5, 10, 3, 5, 6, 7, 8, 4, 11, 6, 11, 8, 8, 4, 10, 4, 7, 1, 6, 11, 6, 4, 0, 9, 6, 8, 5, 0, 5, 2, 10, 10, 3, 0, 0, 2, 8, 1, 6, 6, 11, 10, 1, 11, 1, 2, 2, 5, 8, 6, 3, 10, 3, 9, 10, 9, 3, 11, 1, 0, 9, 1, 5, 1, 7, 8, 6, 5, 11, 2, 9, 4, 0, 5, 8, 11, 2, 5, 1, 0, 10, 10, 3, 8, 10, 9, 2, 9, 3, 5, 9, 7, 7, 1, 6, 9, 11, 8, 4, 10, 7, 0, 10, 8, 4, 5, 6, 2, 6, 9, 0, 9, 8, 3, 11, 3, 4, 7, 5, 1, 1, 1, 1, 7, 4, 0, 8, 8, 11, 1, 11, 7, 4, 3, 1, 5, 1, 4, 4, 0, 0, 6, 2, 8, 0, 8, 10, 1, 10, 3, 8, 8, 2, 11, 7, 6, 11, 8, 11, 0, 2, 11, 2, 8, 4, 1, 2, 11, 4, 9, 11, 7, 8, 9, 0, 1, 2, 11, 8, 5, 0, 3, 11, 1, 10, 4, 6, 8, 5, 1, 11, 8, 10, 11, 11, 5, 5, 8, 5, 3, 0, 7, 11, 9, 6, 2, 2, 8, 10, 0, 7, 3, 5, 2, 11, 5, 8, 3, 9, 9, 3, 1, 4, 0, 6, 7, 7, 9, 3, 4, 6, 8, 7, 11, 11, 11, 0, 8, 1, 10, 7, 0, 8, 0, 8, 5, 5, 0, 2, 11, 4, 8, 11, 8, 4, 9, 4, 1, 7, 11, 0, 3, 9, 10, 2, 5, 2, 1, 3, 8, 7, 10, 2, 9, 10, 10, 4, 6, 8, 1, 1, 11, 11, 5, 8, 10, 8, 8, 7, 3, 3, 10, 1, 0, 8, 3, 9, 8, 10, 7, 1, 4, 9, 9, 9, 4, 5, 10, 0, 2, 6, 5, 2, 2, 4, 2, 1, 2, 2, 9, 0, 11, 10, 8, 6, 0, 8, 4, 11, 9, 2, 11, 5, 3, 8, 9, 1, 3, 11, 7, 9, 1, 7, 3, 8, 1, 4, 10, 0, 10, 10, 6, 9, 5, 3, 2, 6, 3, 8, 1, 7, 6, 6, 3, 1, 3, 6, 0, 6, 7, 3, 5, 2, 8, 7, 10, 0, 1, 9, 4, 0, 1, 7, 4, 11, 1, 10, 6, 2, 3, 3, 0, 0, 8, 5, 7, 2, 10, 6, 7, 5, 3, 1, 8, 6, 11, 2, 10, 7, 11, 10, 5, 3, 8, 2, 3, 11, 7, 1, 6, 11, 9, 9, 8
};

inline float dot(const float* gp, float x, float y) {
    //std::cout << gp[0] << std::endl;
    return gp[0] * x + gp[1] * y;
}

inline float dot(const float* gp, float x, float y, float z) {
    return gp[0] * x + gp[1] * y + gp[2] * z;
}

inline int fastfloor(float x){
    return x > 0 ? (int)x : (int)x-1;
}

float SimplexNoise(float x, float y) {

    //skew coords
    float f = (x + y) * F2;
    int i = std::floor(x+f);
    int j = std::floor(y+f);

    //unskew
    float g = (i + j) * G2;
    // cell origin
    float X0 = i - g;
    float Y0 = j - g;
    // cell origin offsets
    float x0 = x - X0;
    float y0 = y - Y0;

    // masks to find which cell were in
    bool i1 = x0 >= y0;
    bool j1 = !i1;

    // cell vertex offsets
    float x1 = x0 - i1 + G2;
    float y1 = y0 - j1 + G2;
    float x2 = x0 - 1 + 2 * G2;
    float y2 = y0 - 1 + 2 * G2;

    // wrap cells
    int ii = i & 255;
    int jj = j & 255;

    // add up cell influences
    float val = 0;
    float t0 = 0.5f - x0 * x0 - y0 * y0;
    if( t0 > 0 ){
        t0 *= t0; t0 *= t0;
        int gi0 = perm[ii + perm[jj]];
        val += t0 * dot(grad3[gi0], x0, y0);
    }
    float t1 = 0.5f - x1 * x1 - y1 * y1;
    if( t1 > 0 ){
        t1 *= t1; t1 *= t1;
        int gi1 = perm[ii + i1 + perm[jj + j1]];
        val += t1 * dot(grad3[gi1], x1, y1);
    }
    float t2 = 0.5f - x2 * x2 - y2 * y2;
    if( t2 > 0 ){
        t2 *= t2; t2 *= t2;
        int gi2 = perm[ii + 1 + perm[jj + 1]];
        val += t2 * dot(grad3[gi2], x2, y2);
    }

    // scale to [-1, 1]
    return val * 70;

}

float SimplexNoise(float x, float y, float z) {

    //skew coords
    float f = (x + y + z) * F3;
    int i = std::floor(x+f);
    int j = std::floor(y+f);
    int k = std::floor(z+f);

    //unskew
    float g = (i + j + k) * G3;
    //cell origins
    float X0 = i - g;
    float Y0 = j - g;
    float Z0 = k - g;
    //cell origin offsets
    float x0 = x - X0;
    float y0 = y - Y0;
    float z0 = z - Z0;

    //figure out which cell were in
    bool x_ge_y = x0 >= y0;
    bool y_ge_z = y0 >= z0;
    bool x_ge_z = x0 >= z0;

    int i1 = x_ge_y & x_ge_z;
    int j1 = y_ge_z & !x_ge_y;
    int k1 = !(y_ge_z) & !(x_ge_z);

    int i2 = x_ge_y | x_ge_z;
    int j2 = !x_ge_y | y_ge_z;
    int k2 = !(x_ge_z & y_ge_z);

    float x1 = x0 - i1 + G3; // Offsets for second corner in (x,y,z) coords
    float y1 = y0 - j1 + G3;
    float z1 = z0 - k1 + G3;
    float x2 = x0 - i2 + 2.0f * G3; // Offsets for third corner in (x,y,z) coords
    float y2 = y0 - j2 + 2.0f * G3;
    float z2 = z0 - k2 + 2.0f * G3;
    float x3 = x0 - 1.0f + 3.0f * G3; // Offsets for last corner in (x,y,z) coords
    float y3 = y0 - 1.0f + 3.0f * G3;
    float z3 = z0 - 1.0f + 3.0f * G3;

    // Work out the hashed gradient indices of the four simplex corners
    int ii = i & 255;
    int jj = j & 255;
    int kk = k & 255;

    // add up cell influences
    float val = 0;
    float t0 = 0.5f - x0 * x0 - y0 * y0 - z0 * z0;
    if( t0 > 0 ){
        t0 *= t0; t0 *= t0;
        int gi0 = perm[ii+perm[jj+perm[kk]]];
        val += t0 * dot(grad3[gi0], x0, y0, z0);
    }
    float t1 = 0.5f - x1 * x1 - y1 * y1 - z1 * z1;
    if( t1 > 0 ){
        t1 *= t1; t1 *= t1;
        int gi1 = perm[ii+i1+perm[jj+j1+perm[kk+k1]]];
        val += t1 * dot(grad3[gi1], x1, y1, z1);
    }
    float t2 = 0.5f - x2 * x2 - y2 * y2 - z2 * z2;
    if( t2 > 0 ){
        t2 *= t2; t2 *= t2;
        int gi2 = perm[ii+i2+perm[jj+j2+perm[kk+k2]]];
        val += t2 * dot(grad3[gi2], x2, y2, z2);
    }
    float t3 = 0.5f - x3 * x3 - y3 * y3 - z3 * z3;
    if( t3 > 0 ){
        t3 *= t3; t3 *= t3;
        int gi3 = perm[ii+1+perm[jj+1+perm[kk+1]]];
        val += t3 * dot(grad3[gi3], x3, y3, z3);
    }

    return val * 32.0f;
}

void SimplexNoiseCurl(float x, float y, float z, vec_t *out) {

    //skew coords
    float f = (x + y + z) * F3;
    int i = std::floor(x+f);
    int j = std::floor(y+f);
    int k = std::floor(z+f);

    //unskew
    float g = (i + j + k) * G3;
    //cell origins
    float X0 = i - g;
    float Y0 = j - g;
    float Z0 = k - g;
    //cell origin offsets
    float x0 = x - X0;
    float y0 = y - Y0;
    float z0 = z - Z0;

    //figure out which cell were in
    bool x_ge_y = x0 >= y0;
    bool y_ge_z = y0 >= z0;
    bool x_ge_z = x0 >= z0;

    int i1 = x_ge_y & x_ge_z;
    int j1 = y_ge_z & !x_ge_y;
    int k1 = !(y_ge_z) & !(x_ge_z);

    int i2 = x_ge_y | x_ge_z;
    int j2 = !x_ge_y | y_ge_z;
    int k2 = !(x_ge_z & y_ge_z);

    float x1 = x0 - i1 + G3; // Offsets for second corner in (x,y,z) coords
    float y1 = y0 - j1 + G3;
    float z1 = z0 - k1 + G3;
    float x2 = x0 - i2 + 2.0f * G3; // Offsets for third corner in (x,y,z) coords
    float y2 = y0 - j2 + 2.0f * G3;
    float z2 = z0 - k2 + 2.0f * G3;
    float x3 = x0 - 1.0f + 3.0f * G3; // Offsets for last corner in (x,y,z) coords
    float y3 = y0 - 1.0f + 3.0f * G3;
    float z3 = z0 - 1.0f + 3.0f * G3;

    // Work out the hashed gradient indices of the four simplex corners
    int ii = i & 255;
    int jj = j & 255;
    int kk = k & 255;

    // add up cell influences
    vec3_t valA = {0.f, 0.f ,0.f};
    vec3_t valB = {0.f, 0.f ,0.f};
    float t0 = 0.5f - x0 * x0 - y0 * y0 - z0 * z0;
    if( t0 > 0 ){
        float w2 = t0 * t0;
        float w4 = w2 * w2;
        float w3 = w2 * t0;
        int gi0 = perm[ii+perm[jj+perm[kk]]];
        float dwA = -8 * w3 * dot(grad3[gi0], x0, y0, z0);
        vec3_t r0 = {x0, y0, z0};
        VectorMA(valA, w4, grad3[gi0], valA);
        VectorMA(valA, dwA, r0, valA);
        float dwB = -8 * w3 * dot(grad32[gi0], x0, y0, z0);
        VectorMA(valB, w4, grad32[gi0], valB);
        VectorMA(valB, dwB, r0, valB);
    }
    float t1 = 0.5f - x1 * x1 - y1 * y1 - z1 * z1;
    if( t1 > 0 ){
        float w2 = t1 * t1;
        float w4 = w2 * w2;
        float w3 = w2 * t1;
        int gi1 = perm[ii+i1+perm[jj+j1+perm[kk+k1]]];
        float dwA = -8 * w3 * dot(grad3[gi1], x1, y1, z1);
        vec3_t r1 = {x1, y1, z1};
        VectorMA(valA, w4, grad3[gi1], valA);
        VectorMA(valA, dwA, r1, valA);
        float dwB = -8 * w3 * dot(grad32[gi1], x1, y1, z1);
        VectorMA(valB, w4, grad32[gi1], valB);
        VectorMA(valB, dwB, r1, valB);
    }
    float t2 = 0.5f - x2 * x2 - y2 * y2 - z2 * z2;
    if( t2 > 0 ){
        float w2 = t2 * t2;
        float w4 = w2 * w2;
        float w3 = w2 * t2;
        int gi2 = perm[ii+i2+perm[jj+j2+perm[kk+k2]]];
        float dwA = -8 * w3 * dot(grad3[gi2], x2, y2, z2);
        vec3_t r2 = {x2, y2, z2};
        VectorMA(valA, w4, grad3[gi2], valA);
        VectorMA(valA, dwA, r2, valA);
        float dwB = -8 * w3 * dot(grad32[gi2], x2, y2, z2);
        VectorMA(valB, w4, grad32[gi2], valB);
        VectorMA(valB, dwB, r2, valB);
    }
    float t3 = 0.5f - x3 * x3 - y3 * y3 - z3 * z3;
    if( t3 > 0 ){
        float w2 = t3 * t3;
        float w4 = w2 * w2;
        float w3 = w2 * t3;
        int gi3 = perm[ii+1+perm[jj+1+perm[kk+1]]];
        float dwA = -8 * w3 * dot(grad3[gi3], x3, y3, z3);
        vec3_t r3 = {x3, y3, z3};
        VectorMA(valA, w4, grad3[gi3], valA);
        VectorMA(valA, dwA, r3, valA);
        float dwB = -8 * w3 * dot(grad32[gi3], x3, y3, z3);
        VectorMA(valB, w4, grad32[gi3], valB);
        VectorMA(valB, dwB, r3, valB);
    }

    CrossProduct(valA, valB, out);
    float outSqrd = VectorLengthSquared( out );
    if ( outSqrd > 0.0f ) {
        float outRcp = Q_RSqrt( outSqrd );
        VectorScale( out, outRcp, out );
    } else {
        VectorSet( out, 0, 0, 1 );
    }
}

inline float fract( float x ) {
    return x - std::floor(x);
}

// algorithm from https://www.shadertoy.com/view/3d3fWN#
inline void hash33(vec_t *p3, vec_t *out) {
    vec3_t p = {fract(p3[0] * 0.1031f), fract(p3[1] * 0.11369f), fract(p3[2] * 0.13787)};
    float pd = p[0] * (p[1] + 19.19f) + p[1] * (p[0] + 19.19f) + p[2] * (p[2] + 19.19f);
    vec3_t pdv = {pd, pd, pd};
    VectorAdd(p, pdv, p);
    out[0] = fract( (p[0] + p[1]) * p[2]);
    out[1] = fract( (p[0] + p[2]) * p[1]);
    out[2] = fract( (p[1] + p[2]) * p[0]);
}

float VoronoiNoiseSquared(float x, float y, float z) {
    // integer part
    float i = std::floor(x);
    float j = std::floor(y);
    float k = std::floor(z);
    vec3_t cell = {i, j, k};

    // fractional part
    float xf = x - i;
    float yf = y - j;
    float zf = z - k;
    vec3_t rf = {xf, yf, zf};

    float minDist = 1.f;

    for(int l = -1; l <=1; l++) {
        for (int m = -1; m <= 1; m++) {
            for (int n = -1; n <= 1; n++) {
                vec3_t offset = {(float)l, (float)m, (float)n};
                vec3_t coord;
                VectorAdd(cell, offset, coord);

                vec3_t point;
                hash33(coord, point);

                vec3_t dist;
                VectorAdd(offset, point, dist);
                VectorSubtract(dist, rf, dist);
                minDist = std::min(minDist, VectorLengthSquared(dist));
            }
        }
    }

    return minDist;
}