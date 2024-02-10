
#include "geometry.h"

void unitizeGeometry( Geometry *geometry ) {
    const unsigned numVerts = geometry->vertexPositions.size();
    vec3_t *vertexPositions = geometry->vertexPositions.data();
    unsigned vertNum = 0;
    float maxRadius;
    while( ++vertNum < numVerts ){
        maxRadius = wsw::max( maxRadius, VectorLengthFast(vertexPositions[vertNum]) );
    }
    const float unitizingFactor = 1/maxRadius;
    vertNum = 0;
    while( ++vertNum < numVerts ){
        VectorScale( vertexPositions[vertNum], unitizingFactor, vertexPositions[vertNum] );
    }
}