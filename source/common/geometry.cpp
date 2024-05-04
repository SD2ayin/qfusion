
#include "geometry.h"
#include "../cgame/cg_local.h"

void unitizeGeometry( Geometry *geometry ) {
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
		Solve3by3( coefficientsMatrix, result, solution );

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