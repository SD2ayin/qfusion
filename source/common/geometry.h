#include <span>
#include "../ref/ref.h"

#ifndef QFUSION_GEOMETRY_H
#define QFUSION_GEOMETRY_H

typedef unsigned tri[3];

struct Geometry {
    std::span<vec3_t> vertexPositions;
    std::span<tri> triIndices;
};

struct TexturedGeometry {
    Geometry geometry;
    std::span<vec2_t[3]> triUVcoords;
};

void inline getTriCoords( const unsigned *triIndices, const Geometry *geometry, vec3_t *outCoords ){
    VectorCopy( geometry->vertexPositions[triIndices[0]], outCoords[0] );
    VectorCopy( geometry->vertexPositions[triIndices[1]], outCoords[1] );
    VectorCopy( geometry->vertexPositions[triIndices[2]], outCoords[2] );
}

void unitizeGeometry( Geometry *geometry );

#endif //QFUSION_GEOMETRY_H