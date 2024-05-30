
#include "geometry.h"
#include "../cgame/cg_local.h"

void inline getTriCoords( const uint16_t *triIndices, const Geometry *geometry, vec3_t *outCoords ){
//	VectorCopy( geometry->vertexPositions[triIndices[0]], outCoords[0] );
//	VectorCopy( geometry->vertexPositions[triIndices[1]], outCoords[1] );
//	VectorCopy( geometry->vertexPositions[triIndices[2]], outCoords[2] );
	getTriCoords( triIndices, geometry->vertexPositions.data(), outCoords );
}

void inline getTriCoords( const uint16_t *triIndices, vec3_t *vertexPositions, vec3_t *outCoords ){
	VectorCopy( vertexPositions[triIndices[0]], outCoords[0] );
	VectorCopy( vertexPositions[triIndices[1]], outCoords[1] );
	VectorCopy( vertexPositions[triIndices[2]], outCoords[2] );
}

void getTriNormals( const uint16_t *triIndices, vec3_t *vertexNormals, vec3_t *outNormals ){
	VectorCopy( vertexNormals[triIndices[0]], outNormals[0] );
	VectorCopy( vertexNormals[triIndices[1]], outNormals[1] );
	VectorCopy( vertexNormals[triIndices[2]], outNormals[2] );
}

void unitizeGeometry( Geometry *geometry ) { //TODO: probably remove this?
    const unsigned numVerts = geometry->vertexPositions.size();
    vec3_t *vertexPositions = geometry->vertexPositions.data();
    unsigned vertNum = 0;
    float maxRadius = 1e-2f;
    while( vertNum < numVerts ){
        maxRadius = wsw::max( maxRadius, VectorLengthFast(vertexPositions[vertNum]) );
		vertNum++;
    }
    const float unitizingFactor = 1/maxRadius;
    vertNum = 0;
    while( vertNum < numVerts ){
        VectorScale( vertexPositions[vertNum], unitizingFactor, vertexPositions[vertNum] );
		vertNum++;
    }
}

void calculateNormals( std::span<tri> triIndices, const Geometry *geometry, vec3_t *outNormals ){
	calculateNormals( triIndices, geometry->vertexPositions.size(),
					  geometry->vertexPositions.data(), outNormals );
}

void calculateNormals( std::span<tri> triIndices, unsigned numVerts, vec3_t *vertexPositions, vec3_t *outNormals ){
	tri *tris = triIndices.data();
	unsigned numTris = triIndices.size();

	for( unsigned vertNum = 0; vertNum < numVerts; vertNum++ ){
		VectorSet( outNormals[vertNum], 0.0f, 0.0f, 0.0f );
	}

	for( unsigned triNum = 0; triNum < numTris; triNum++ ) {
		const unsigned firstVertIdx  = tris[triNum][0];
		const unsigned secondVertIdx = tris[triNum][1];
		const unsigned thirdVertIdx  = tris[triNum][2];

		vec3_t triCoords[3];
		VectorCopy( vertexPositions[firstVertIdx], triCoords[0] );
		VectorCopy( vertexPositions[secondVertIdx], triCoords[1] );
		VectorCopy( vertexPositions[thirdVertIdx], triCoords[2] );

		vec3_t vecToSecondVert;
		vec3_t vecToThirdVert;
		VectorSubtract( triCoords[1], triCoords[0], vecToSecondVert );
		VectorSubtract( triCoords[2], triCoords[0], vecToThirdVert );

		vec3_t normal;
		CrossProduct( vecToThirdVert, vecToSecondVert, normal );

		VectorAdd( outNormals[firstVertIdx], normal, outNormals[firstVertIdx] );
		VectorAdd( outNormals[secondVertIdx], normal, outNormals[secondVertIdx] );
		VectorAdd( outNormals[thirdVertIdx], normal, outNormals[thirdVertIdx] );
	}

	for( unsigned vertNum = 0; vertNum < numVerts; vertNum++ ){
		VectorNormalizeFast( outNormals[vertNum] );
		// TODO: while the mesh the artist made has to be very messed up for this to occur (possibly a flattened mesh) shouldn't we provide some clear error/crash in this case?
	}
}

bool collisionCheck( Geometry *collisionGeometry, vec3_t origin, vec3_t dir, float maxDist, unsigned *outTriIdx, float *outDist, vec2_t coordsOnTri ){
	bool foundCollision = false;

    unsigned numCollisions = 0;

	const unsigned collisionFaces = collisionGeometry->triIndices.size();

	for( unsigned faceNum = 0; ( faceNum < collisionFaces ) && !foundCollision; faceNum++ ) {

		const uint16_t *faceIndices = collisionGeometry->triIndices[faceNum];
		vec3_t triCoords[3];
		getTriCoords( faceIndices, collisionGeometry, triCoords );
		// calculate offsets of other vertices from the first one in the array
		vec3_t first; // first base vector
		vec3_t second; // second base vector
		VectorSubtract( triCoords[1], triCoords[0], first );
		VectorSubtract( triCoords[2], triCoords[0], second );

		// only vertices that move in the direction of the face can be bound by that face
		vec3_t faceNormal;
        CrossProduct( second, first, faceNormal );
		if ( DotProduct( faceNormal, dir ) <= 0.f ) {
			continue;
		}

		// solve the system of linear equations to express the vertex coordinates in terms of the vertex offsets inside the triangle
		mat3_t coefficientsMatrix;
		VectorCopy( dir, &coefficientsMatrix[0] );
		VectorCopy( first, &coefficientsMatrix[3] );
		VectorCopy( second, &coefficientsMatrix[6] );

		vec3_t result; // result in the linear matrix equation
		VectorSubtract( triCoords[0], origin, result );

		vec3_t solution;
		if( !Solve3by3( coefficientsMatrix, result, solution ) ){
			continue;
		}

		vec2_t coordinates = { -solution[1], -solution[2] };
		float distanceToCollision = solution[0];

		constexpr float FPEmargin = 1e-3f; // to account for floating point error
		if( ((coordinates[0] + coordinates[1]) < 1.0f + FPEmargin) && (coordinates[0] > 0.0f - FPEmargin) &&
			(coordinates[1] > 0.0f - FPEmargin) && (distanceToCollision >= 0.0f) && (distanceToCollision < maxDist) ) {
			if( coordsOnTri ) {
				Vector2Copy( coordinates, coordsOnTri );
			}

			if( outDist ) {
				*outDist = distanceToCollision;
			}
			if( outTriIdx ) {
				*outTriIdx = faceNum;
			}

			foundCollision = true;
            numCollisions++;

		}
	}

	return foundCollision;
}