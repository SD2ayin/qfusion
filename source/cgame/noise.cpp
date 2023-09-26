/*#include "noise.h"
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

*/

#include "noise.h"
#include "../gameshared/q_math.h"

static const vec3_t kPrimGrad[] = {
        {1,1,0}, {-1,1,0}, {1,-1,0}, {-1,-1,0},
        {1,0,1}, {-1,0,1}, {1,0,-1}, {-1,0,-1},
        {0,1,1}, {0,-1,1}, {0,1,-1}, {0,-1,-1}
};

// Supplementary gradient for quick computation of curl noise
static const vec3_t kSuppGrad[] = {
        {1,-1,0}, {-1,-1,0}, {0,1,-1}, {-1, 1,0},
        {-1,0,-1}, {1,0,1}, {1,1,-1}, {0,-1,1},
        {1,-1,1}, {0,1,1},  {0,-1,-1}, {-1,0,1}
};

static const uint8_t kIndexTable[] = {
        151, 160, 137, 91, 90, 15, 131, 13, 201, 95, 96, 53, 194, 233, 7, 225, 140, 36, 103, 30, 69, 142, 8, 99, 37, 240,
        21, 10, 23, 190, 6, 148, 247, 120, 234, 75, 0, 26, 197, 62, 94, 252, 219, 203, 117, 35, 11, 32, 57, 177, 33, 88,
        237, 149, 56, 87, 174, 20, 125, 136, 171, 168, 68, 175, 74, 165, 71, 134, 139, 48, 27, 166, 77, 146, 158, 231, 83,
        111, 229, 122, 60, 211, 133, 230, 220, 105, 92, 41, 55, 46, 245, 40, 244, 102, 143, 54, 65, 25, 63, 161, 1, 216,
        80, 73, 209, 76, 132, 187, 208, 89, 18, 169, 200, 196, 135, 130, 116, 188, 159, 86, 164, 100, 109, 198, 173, 186,
        3, 64, 52, 217, 226, 250, 124, 123, 5, 202, 38, 147, 118, 126, 255, 82, 85, 212, 207, 206, 59, 227, 47, 16, 58, 17,
        182, 189, 28, 42, 223, 183, 170, 213, 119, 248, 152, 2, 44, 154, 163, 70, 221, 153, 101, 155, 167, 43, 172, 9, 129,
        22, 39, 253, 19, 98, 108, 110, 79, 113, 224, 232, 178, 185, 112, 104, 218, 246, 97, 228, 251, 34, 242, 193, 238,
        210, 144, 12, 191, 179, 162, 241, 81, 51, 145, 235, 249, 14, 239, 107, 49, 192, 214, 31, 181, 199, 106, 157, 184,
        84, 204, 176, 115, 121, 50, 45, 127, 4, 150, 254, 138, 236, 205, 93, 222, 114, 67, 29, 24, 72, 243, 141, 128, 195,
        78, 66, 215, 61, 156, 180, 151, 160, 137, 91, 90, 15, 131, 13, 201, 95, 96, 53, 194, 233, 7, 225, 140, 36, 103, 30, 69, 142, 8, 99, 37, 240,
        21, 10, 23, 190, 6, 148, 247, 120, 234, 75, 0, 26, 197, 62, 94, 252, 219, 203, 117, 35, 11, 32, 57, 177, 33, 88,
        237, 149, 56, 87, 174, 20, 125, 136, 171, 168, 68, 175, 74, 165, 71, 134, 139, 48, 27, 166, 77, 146, 158, 231, 83,
        111, 229, 122, 60, 211, 133, 230, 220, 105, 92, 41, 55, 46, 245, 40, 244, 102, 143, 54, 65, 25, 63, 161, 1, 216,
        80, 73, 209, 76, 132, 187, 208, 89, 18, 169, 200, 196, 135, 130, 116, 188, 159, 86, 164, 100, 109, 198, 173, 186,
        3, 64, 52, 217, 226, 250, 124, 123, 5, 202, 38, 147, 118, 126, 255, 82, 85, 212, 207, 206, 59, 227, 47, 16, 58, 17,
        182, 189, 28, 42, 223, 183, 170, 213, 119, 248, 152, 2, 44, 154, 163, 70, 221, 153, 101, 155, 167, 43, 172, 9, 129,
        22, 39, 253, 19, 98, 108, 110, 79, 113, 224, 232, 178, 185, 112, 104, 218, 246, 97, 228, 251, 34, 242, 193, 238,
        210, 144, 12, 191, 179, 162, 241, 81, 51, 145, 235, 249, 14, 239, 107, 49, 192, 214, 31, 181, 199, 106, 157, 184,
        84, 204, 176, 115, 121, 50, 45, 127, 4, 150, 254, 138, 236, 205, 93, 222, 114, 67, 29, 24, 72, 243, 141, 128, 195,
        78, 66, 215, 61, 156, 180
};

[[nodiscard]]
wsw_forceinline auto dot2( const float *gp, float x, float y ) -> float {
    return gp[0] * x + gp[1] * y;
}

[[nodiscard]]
wsw_forceinline auto dot3( const float *gp, float x, float y, float z ) -> float {
    return gp[0] * x + gp[1] * y + gp[2] * z;
}

auto calcSimplexNoise2D( float x, float y ) -> float {
    // Sanity checks, make sure we're specifying a point within the world bounds
    assert( x >= -(float)std::numeric_limits<uint16_t>::max() && x <= (float)std::numeric_limits<uint16_t>::max() );
    assert( y >= -(float)std::numeric_limits<uint16_t>::max() && y <= (float)std::numeric_limits<uint16_t>::max() );

    constexpr float F2 = 0.36602540378443f; // 0.5f * ( std::sqrtf(3.0f) - 1.0f );
    constexpr float G2 = 0.21132486540518f; // ( 3 - std::sqrtf(3.0f)) / 6;

    // Skew coords
    const float f = ( x + y ) * F2;
    const float i = std::floor( x + f );
    const float j = std::floor( y + f );

    // Unskew
    const float g = ( i + j ) * G2;

    const float cellOriginX = i - g;
    const float cellOriginY = j - g;

    const float x0 = x - cellOriginX;
    const float y0 = y - cellOriginY;

    // masks to find which cell were in
    const bool iShift = x0 >= y0;
    const bool jShift = !iShift;

    // cell vertex offsets
    const float x1 = x0 - (float)iShift + G2;
    const float y1 = y0 - (float)jShift + G2;
    const float x2 = x0 - 1 + 2 * G2;
    const float y2 = y0 - 1 + 2 * G2;

    // wrap cells
    const int iValue = (int)i & 255;
    const int jValue = (int)j & 255;

    // add up cell influences
    const float t0 = 0.5f - x0 * x0 - y0 * y0;
    const float t1 = 0.5f - x1 * x1 - y1 * y1;
    const float t2 = 0.5f - x2 * x2 - y2 * y2;

    float result = 0.0f;
    if( t0 > 0.0f ) {
        const int gradIndex = kIndexTable[iValue + kIndexTable[jValue]] % 12;
        result += wsw::square( wsw::square( t0 ) ) * dot2( kPrimGrad[gradIndex], x0, y0 );
    }
    if( t1 > 0.0f ) {
        const int gradIndex = kIndexTable[iValue + iShift + kIndexTable[jValue + jShift]] % 12;
        result += wsw::square( wsw::square( t1 ) ) * dot2( kPrimGrad[gradIndex], x1, y1 );
    }
    if( t2 > 0.0f ) {
        const int gradIndex = kIndexTable[iValue + 1 + kIndexTable[jValue + 1]] % 12;
        result += wsw::square( wsw::square( t2 ) ) * dot2( kPrimGrad[gradIndex], x2, y2 );
    }

    // scale to [-1, 1]
    return 70.0f * result;
}

wsw_forceinline auto calcGradIndex3D( int iValue, int iShift, int jValue, int jShift, int kValue, int kShift ) -> int {
    return kIndexTable[iValue + iShift + kIndexTable[jValue + jShift + kIndexTable[kValue + kShift]]];
}

static void addCellContribution( float t, int gradIndex, float x, float y, float z, float *__restrict valA, float *__restrict valB ) {
    const float w2  = t * t;
    const float w4  = w2 * w2;
    const float w3  = w2 * t;
    const float dwA = -8.0f * w3 * dot3( kPrimGrad[gradIndex], x, y, z );
    const float dwB = -8.0f * w3 * dot3( kSuppGrad[gradIndex], x, y, z );
    const vec3_t r0 = { x, y, z };
    VectorMA( valA, w4, kPrimGrad[gradIndex], valA );
    VectorMA( valA, dwA, r0, valA );
    VectorMA( valB, w4, kSuppGrad[gradIndex], valB );
    VectorMA( valB, dwB, r0, valB );
}

// Try inlining it forcefully in the final subroutines so there's no redundant calls
template <typename Result>
[[nodiscard]]
wsw_forceinline auto calcSimplexNoise3DImpl( float givenX, float givenY, float givenZ ) -> Result {
    // Sanity checks, make sure we're specifying a point within the world bounds
    assert( givenX >= -(float)std::numeric_limits<uint16_t>::max() && givenX <= (float)std::numeric_limits<uint16_t>::max() );
    assert( givenY >= -(float)std::numeric_limits<uint16_t>::max() && givenY <= (float)std::numeric_limits<uint16_t>::max() );
    assert( givenZ >= -(float)std::numeric_limits<uint16_t>::max() && givenZ <= (float)std::numeric_limits<uint16_t>::max() );

    constexpr float F3 = 1.0f / 3.0f;
    constexpr float G3 = 1.0f / 6.0f;

    // Skew coords
    const float f = ( givenX + givenY + givenZ ) * F3;

    const float pointI = std::floor( givenX + f );
    const float pointJ = std::floor( givenY + f );
    const float pointK = std::floor( givenZ + f );

    // Unskew
    const float g = ( pointI + pointJ + pointK ) * G3;

    const float cellOriginX = pointI - g;
    const float cellOriginY = pointJ - g;
    const float cellOriginZ = pointK - g;

    const float x0 = givenX - cellOriginX;
    const float y0 = givenY - cellOriginY;
    const float z0 = givenZ - cellOriginZ;

    // Figure out which cell were in
    const bool x_ge_y = x0 >= y0;
    const bool y_ge_z = y0 >= z0;
    const bool x_ge_z = x0 >= z0;

    const int iShift1 = x_ge_y & x_ge_z;
    const int jShift1 = y_ge_z & !x_ge_y;
    const int kShift1 = !y_ge_z & !x_ge_z;

    const int iShift2 = x_ge_y | x_ge_z;
    const int jShift2 = !x_ge_y | y_ge_z;
    const int kShift2 = !( x_ge_z & y_ge_z );

    // Offsets for the second corner in (x, y, z) coords
    const float x1 = x0 - (float)iShift1 + G3;
    const float y1 = y0 - (float)jShift1 + G3;
    const float z1 = z0 - (float)kShift1 + G3;
    // Offsets for the third corner in (x, y, z) coords
    const float x2 = x0 - (float)iShift2 + 2.0f * G3;
    const float y2 = y0 - (float)jShift2 + 2.0f * G3;
    const float z2 = z0 - (float)kShift2 + 2.0f * G3;
    // Offsets for the last corner in (x, y, z) coords
    const float x3 = x0 - 1.0f + 3.0f * G3;
    const float y3 = y0 - 1.0f + 3.0f * G3;
    const float z3 = z0 - 1.0f + 3.0f * G3;

    // Work out the hashed gradient indices of the four simplex corners
    const int iValue = (int)pointI & 255;
    const int jValue = (int)pointJ & 255;
    const int kValue = (int)pointK & 255;

    // add up cell influences

    const float t0 = 0.5f - x0 * x0 - y0 * y0 - z0 * z0;
    const float t1 = 0.5f - x1 * x1 - y1 * y1 - z1 * z1;
    const float t2 = 0.5f - x2 * x2 - y2 * y2 - z2 * z2;
    const float t3 = 0.5f - x3 * x3 - y3 * y3 - z3 * z3;

    constexpr bool isComputingCurl = std::is_same_v<std::remove_cvref_t<Result>, Vec3>;

    float tmp[isComputingCurl ? 9 : 1];
    [[maybe_unused]] float *const valA = tmp + ( isComputingCurl ? 3 : 0 );
    [[maybe_unused]] float *const valB = tmp + ( isComputingCurl ? 6 : 0 );

    if constexpr( isComputingCurl ) {
        VectorClear( valA );
        VectorClear( valB );
    } else {
        tmp[0] = 0.0f;
    }

    if( t0 > 0.0f ) {
        const int gradIndex = calcGradIndex3D( iValue, 0, jValue, 0, kValue, 0 ) % 12;
        if constexpr( isComputingCurl ) {
            addCellContribution( t0, gradIndex, x0, y0, z0, valA, valB );
        } else {
            tmp[0] += wsw::square( wsw::square( t0 ) ) * dot3( kPrimGrad[gradIndex], x0, y0, z0 );
        }
    }
    if( t1 > 0.0f ) {
        const int gradIndex = calcGradIndex3D( iValue, iShift1, jValue, jShift1, kValue, kShift1 ) % 12;
        if constexpr( isComputingCurl ) {
            addCellContribution( t1, gradIndex, x1, y1, z1, valA, valB );
        } else {
            tmp[0] += wsw::square( wsw::square( t1 ) ) * dot3( kPrimGrad[gradIndex], x1, y1, z1 );
        }
    }
    if( t2 > 0.0f ) {
        const int gradIndex = calcGradIndex3D( iValue, iShift2, jValue, jShift2, kValue, kShift2 ) % 12;
        if constexpr( isComputingCurl ) {
            addCellContribution( t2, gradIndex, x2, y2, z2, valA, valB );
        } else {
            tmp[0] += wsw::square( wsw::square( t2 ) ) * dot3( kPrimGrad[gradIndex], x2, y2, z2 );
        }
    }
    if( t3 > 0.0f ) {
        const int gradIndex = calcGradIndex3D( iValue, 1, jValue, 1, kValue, 1 ) % 12;
        if constexpr( isComputingCurl ) {
            addCellContribution( t3, gradIndex, x3, y3, z3, valA, valB );
        } else {
            tmp[0] += wsw::square( wsw::square( t3 ) ) * dot3( kPrimGrad[gradIndex], x3, y3, z3 );
        }
    }

    if constexpr( isComputingCurl ) {
        CrossProduct( valA, valB, tmp );
        if( const float outSquareLength = VectorLengthSquared( tmp ); outSquareLength != 0 ) [[likely]] {
            const float outRcpLength = Q_RSqrt( outSquareLength );
            VectorScale( tmp, outRcpLength, tmp );
        } else {
            VectorSet( tmp, 1.0f, 0.0f, 0.0f );
        }
        return Vec3( tmp );
    } else {
        return 32.0f * tmp[0];
    }
}

auto calcSimplexNoise3D( float x, float y, float z ) -> float {
    return calcSimplexNoise3DImpl<float>( x, y, z );
}

auto calcSimplexNoiseCurl( float x, float y, float z ) -> Vec3 {
    return calcSimplexNoise3DImpl<Vec3>(x, y, z);
}


inline float fract( float x ) {
    return x - std::floor(x);
}

/*
hash33 function:

MIT License

Copyright (c) 2019 A_Riccardi

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
 */
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