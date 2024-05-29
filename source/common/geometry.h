#include <span>
#include "../ref/ref.h"

#ifndef QFUSION_GEOMETRY_H
#define QFUSION_GEOMETRY_H

typedef uint16_t tri[3];

struct Geometry;

// having this function that's in alias here, with Geometry pre-declared seems very weird and could be confusing..
bool GetGeometryFromFileAliasMD3( const char *fileName, Geometry *outGeometry, const char *meshName = nullptr, const unsigned chosenFrame = 0 );

struct Geometry { // should probably have a destructor and constructor methods from alias.cpp
    std::span<vec3_t> vertexPositions;
    std::span<tri> triIndices;
    vec2_t *UVCoords; //texture coords, equal to verts

    Geometry(){
        UVCoords = nullptr;
    }
    Geometry( const char *fileName, const char *meshName = nullptr, const unsigned chosenFrame = 0 ){
        GetGeometryFromFileAliasMD3( fileName, this, meshName, chosenFrame );
    }
    ~Geometry(){
        if( vertexPositions.data() ) {
            delete[] vertexPositions.data();
            delete[] triIndices.data();
            delete[] UVCoords;
        }
    }
};

struct TexturedGeometry {
    Geometry geometry;
    std::span<vec2_t[3]> triUVcoords;
};

void inline getTriCoords( const uint16_t *triIndices, const Geometry *geometry, vec3_t *outCoords );

void inline getTriCoords( const uint16_t *triIndices, vec3_t *vertexPositions, vec3_t *outCoords );

void getTriNormals( const uint16_t *triIndices, vec3_t *vertexNormals, vec3_t *outNormals );

void unitizeGeometry( Geometry *geometry );

void calculateNormals( std::span<tri> triIndices, const Geometry *geometry, vec3_t *outNormals );

void calculateNormals( std::span<tri> triIndices, unsigned numVerts, vec3_t *vertexPositions, vec3_t *outNormals );

bool collisionCheck( Geometry *collisionGeometry, vec3_t origin, vec3_t dir, float maxDist, unsigned *outTriIdx, float *outDist, vec2_t coordsOnTri );

#endif //QFUSION_GEOMETRY_H