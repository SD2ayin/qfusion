#include "../gameshared/q_math.h"

float SimplexNoise(float x, float y);
float SimplexNoise(float x, float y, float z);
void SimplexNoiseCurl(float x, float y, float z, vec_t *out);

float VoronoiNoiseSquared(float x, float y, float z);
