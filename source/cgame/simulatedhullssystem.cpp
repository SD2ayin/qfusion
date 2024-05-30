#include "simulatedhullssystem.h"

#include "../common/links.h"
#include "../client/client.h"
#include "../common/memspecbuilder.h"
#include "../common/mmcommon.h"
#include "../cgame/cg_local.h"
#include "../common/configvars.h"
#include "../common/noise.h"

#include <memory>
#include <unordered_map>
#include <algorithm>
#include <functional>

struct IcosphereData {
	std::span<const vec4_t> vertices;
	std::span<const uint16_t> indices;
	std::span<const uint16_t[5]> vertexNeighbours;
};

class BasicHullsHolder {
	struct Entry { unsigned firstIndex, numIndices, numVertices, firstNeighboursElement; };
public:
	static constexpr unsigned kMaxSubdivLevel = 4;

	BasicHullsHolder() {
		constexpr const uint8_t icosahedronFaces[20][3] {
			{ 0, 11, 5 }, { 0, 5, 1 }, { 0, 1, 7 }, { 0, 7, 10 }, { 0, 10, 11 },
			{ 1, 5, 9 }, { 5, 11, 4 }, { 11, 10, 2 }, { 10, 7, 6 }, { 7, 1, 8 },
			{ 3, 9, 4 }, { 3, 4, 2 }, { 3, 2, 6 }, { 3, 6, 8 }, { 3, 8, 9 },
			{ 4, 9, 5 }, { 2, 4, 11 }, { 6, 2, 10 }, { 8, 6, 7 }, { 9, 8, 1 },
		};

		constexpr const float a = 0.525731f, b = 0.850651f;
		const vec3_t icosahedronVertices[12] {
			{ -a, +b, +0 }, { +a, +b, +0 }, { -a, -b, +0 }, { +a, -b, +0 },
			{ +0, -a, +b }, { +0, +a, +b }, { +0, -a, -b }, { +0, +a, -b },
			{ +b, +0, -a }, { +b, +0, +a }, { -b, +0, -a }, { -b, +0, +a },
		};

		for( const auto &v : icosahedronVertices ) {
			m_vertices.emplace_back( Vertex { { v[0], v[1], v[2], 1.0f } } );
		}
		for( const auto &f: icosahedronFaces ) {
			m_faces.emplace_back( Face { { f[0], f[1], f[2] } } );
		}

		wsw::StaticVector<Entry, kMaxSubdivLevel + 1> icosphereEntries;
		icosphereEntries.emplace_back( Entry { 0, 3 * (unsigned)m_faces.size(), (unsigned)m_vertices.size() } );

		MidpointMap midpointCache;
		unsigned oldFacesSize = 0, facesSize = m_faces.size();
		while( !icosphereEntries.full() ) {
			for( unsigned i = oldFacesSize; i < facesSize; ++i ) {
				const Face &face = m_faces[i];
				const uint16_t v1 = face.data[0], v2 = face.data[1], v3 = face.data[2];
				assert( v1 != v2 && v2 != v3 && v1 != v3 );
				const uint16_t p = getMidPoint( v1, v2, &midpointCache );
				const uint16_t q = getMidPoint( v2, v3, &midpointCache );
				const uint16_t r = getMidPoint( v3, v1, &midpointCache );
				m_faces.emplace_back( Face { { v1, p, r } } );
				m_faces.emplace_back( Face { { v2, q, p } } );
				m_faces.emplace_back( Face { { v3, r, q } } );
				m_faces.emplace_back( Face { { p, q, r } } );
			}
			oldFacesSize = facesSize;
			facesSize = m_faces.size();
			const unsigned firstIndex = 3 * oldFacesSize;
			const unsigned numIndices = 3 * ( facesSize - oldFacesSize );
			icosphereEntries.emplace_back( Entry { firstIndex, numIndices, (unsigned)m_vertices.size() } );
		}

		// TODO: Use fixed vectors
		m_vertices.shrink_to_fit();
		m_faces.shrink_to_fit();

		// TODO can we do that during faces construction.
		buildNeighbours( icosphereEntries );

		// Build data of the public interface
		for( unsigned level = 0; level < kMaxSubdivLevel + 1; ++level ) {
			using OpaqueNeighbours     = uint16_t[5];
			const auto *vertexData     = (const vec4_t *)m_vertices.data();
			const auto *indexData      = (const uint16_t *)m_faces.data();
			const auto *neighboursData = (const OpaqueNeighbours *)m_neighbours.data();
			const Entry &entry         = icosphereEntries[level];
			std::span<const vec4_t> verticesSpan { vertexData, entry.numVertices };
			std::span<const uint16_t> indicesSpan { indexData + entry.firstIndex, entry.numIndices };
			std::span<const uint16_t[5]> neighboursSpan { neighboursData + entry.firstNeighboursElement, entry.numVertices };
			m_icospheresForLevels.emplace_back( IcosphereData { verticesSpan, indicesSpan, neighboursSpan } );
		}
	}

	[[nodiscard]]
	auto getIcosphereForLevel( unsigned level ) -> IcosphereData {
		return m_icospheresForLevels[level];
	}
private:
	struct alignas( 4 ) Vertex { float data[4]; };
	static_assert( sizeof( Vertex ) == sizeof( vec4_t ) );
	struct alignas( 2 ) Face { uint16_t data[3]; };
	static_assert( sizeof( Face ) == sizeof( uint16_t[3] ) );
	struct alignas( 2 ) Neighbours { uint16_t data[5]; };
	static_assert( sizeof( Neighbours ) == sizeof( uint16_t[5] ) );

	using MidpointMap = std::unordered_map<unsigned, uint16_t>;

	[[nodiscard]]
	auto getMidPoint( uint16_t i1, uint16_t i2, MidpointMap *midpointCache ) -> uint16_t {
		const unsigned smallest = ( i1 < i2 ) ? i1 : i2;
		const unsigned largest = ( i1 < i2 ) ? i2 : i1;
		const unsigned key = ( smallest << 16 ) | largest;
		if( const auto it = midpointCache->find( key ); it != midpointCache->end() ) {
			return it->second;
		}

		Vertex midpoint {};
		const float *v1 = m_vertices[i1].data, *v2 = m_vertices[i2].data;
		VectorAvg( v1, v2, midpoint.data );
		VectorNormalize( midpoint.data );
		midpoint.data[3] = 1.0f;

		const auto index = (uint16_t)m_vertices.size();
		m_vertices.push_back( midpoint );
		midpointCache->insert( std::make_pair( key, index ) );
		return index;
	}

	static constexpr unsigned kNumVertexNeighbours = 5;
	static constexpr unsigned kNumFaceNeighbours = 5;

	struct NeighbourFaces { uint16_t data[kNumVertexNeighbours]; };

	// Returns the number of vertices that got their neighbour faces list completed during this call
	[[nodiscard]]
	auto addFaceToNeighbours( uint16_t faceNum, NeighbourFaces *neighbourFaces,
							  uint8_t *knownNeighboursCounts ) -> unsigned {
		unsigned result = 0;

		for( unsigned i = 0; i < 3; ++i ) {
			const uint16_t vertexNum      = m_faces[faceNum].data[i];
			uint8_t *const countForVertex = &knownNeighboursCounts[vertexNum];
			assert( *countForVertex <= kNumFaceNeighbours );
			if( *countForVertex < kNumFaceNeighbours ) {
				uint16_t *const indicesBegin = neighbourFaces[vertexNum].data;
				uint16_t *const indicesEnd   = indicesBegin + *countForVertex;
				// TODO: Is this search really needed?
				if( std::find( indicesBegin, indicesEnd, faceNum ) == indicesEnd ) {
					( *countForVertex )++;
					*indicesEnd = (uint16_t)faceNum;
					// The neighbours list becomes complete
					if( *countForVertex == kNumFaceNeighbours ) {
						result++;
					}
				}
			}
		}

		return result;
	}

	[[nodiscard]]
	auto findFaceWithVertex( const NeighbourFaces *neighbourFaces, unsigned vertexNum, unsigned pickedFacesMask )
		-> std::optional<std::pair<unsigned, unsigned>> {

		// Assume that face #0 is always picked
		for( unsigned faceIndex = 1; faceIndex < kNumFaceNeighbours; ++faceIndex ) {
			if( !( pickedFacesMask & ( 1u << faceIndex ) ) ) {
				const Face &face = m_faces[neighbourFaces->data[faceIndex]];
				for( unsigned indexInFace = 0; indexInFace < 3; ++indexInFace ) {
					if( face.data[indexInFace] == vertexNum ) {
						return std::pair<unsigned, unsigned> { faceIndex, indexInFace };
					}
				}
			}
		}

		return std::nullopt;
	}

	void buildNeighboursForVertexUsingNeighbourFaces( unsigned vertexNum, Neighbours *neighbourVertices,
													  const NeighbourFaces *neighbourFaces ) {
		wsw::StaticVector<uint16_t, kNumVertexNeighbours> vertexNums;

		// Pick the first face unconditionally
		const Face &firstFace    = m_faces[neighbourFaces->data[0]];
		unsigned pickedFacesMask = 0x1;

		// Add 2 vertices of the first face
		for( unsigned i = 0; i < 3; ++i ) {
			if( firstFace.data[i] == vertexNum ) {
				vertexNums.push_back( firstFace.data[( i + 1 ) % 3] );
				vertexNums.push_back( firstFace.data[( i + 2 ) % 3] );
				break;
			}
		}

		assert( vertexNums.size() == 2 );
		while( !vertexNums.full() ) {
			bool prepend = false;
			// Try finding a face that contacts with the last added face forming a triangle fan
			auto maybeIndices = findFaceWithVertex( neighbourFaces, vertexNums.back(), pickedFacesMask );
			if( !maybeIndices ) {
				// Find a face that contacts with the first added face forming a triangle fan
				maybeIndices = findFaceWithVertex( neighbourFaces, vertexNums.front(), pickedFacesMask );
				prepend = true;
			}

			const auto [faceIndex, indexInFace] = maybeIndices.value();
			const Face &face = m_faces[neighbourFaces->data[faceIndex]];
			const unsigned otherIndex1 = ( indexInFace + 1 ) % 3;
			const unsigned otherIndex2 = ( indexInFace + 2 ) % 3;

			// Make sure the face contains the original vertex.
			assert( face.data[otherIndex1] == vertexNum || face.data[otherIndex2] == vertexNum );

			// Add the remaining one to the neighbour vertices list.
			unsigned indexOfVertexToAdd = otherIndex1;
			if( face.data[otherIndex1] == vertexNum ) {
				indexOfVertexToAdd = otherIndex2;
			}

			const uint16_t vertexNumToAdd = face.data[indexOfVertexToAdd];
			if( prepend ) {
				vertexNums.insert( vertexNums.begin(), vertexNumToAdd );
			} else {
				vertexNums.push_back( vertexNumToAdd );
			}

			pickedFacesMask |= ( 1u << faceIndex );
		}

		// Now check the normal direction with regard to direction to vertex

		const float *const v1 = m_vertices[vertexNum].data;
		const float *const v2 = m_vertices[vertexNums[0]].data;
		const float *const v3 = m_vertices[vertexNums[1]].data;

		vec3_t v1To2, v1To3, cross;
		VectorSubtract( v2, v1, v1To2 );
		VectorSubtract( v3, v1, v1To3 );
		CrossProduct( v1To2, v1To3, cross );
		// The vertex is a direction from (0, 0, 0) to itself
		if( DotProduct( v1, cross ) < 0 ) {
			std::reverse( vertexNums.begin(), vertexNums.end() );
		}

		std::copy( vertexNums.begin(), vertexNums.end(), neighbourVertices->data );
	}

	void buildNeighbours( wsw::StaticVector<Entry, kMaxSubdivLevel + 1> &icosphereEntries ) {
		NeighbourFaces neighbourFacesForVertex[3072];
		uint8_t knownNeighbourFacesCounts[3072];

		// Note that neighbours of the same vertex differ for different subdivision level
		// so we have to recompute all neighbours for each entry (each subdivision level).
		for( Entry &entry: icosphereEntries ) {
			entry.firstNeighboursElement = m_neighbours.size();
			m_neighbours.resize( m_neighbours.size() + entry.numVertices );

			Neighbours *const neighbourVerticesForVertex = m_neighbours.data() + entry.firstNeighboursElement;

			// Fill by ~0 to spot non-feasible indices fast
			std::memset( neighbourVerticesForVertex, 255, entry.numVertices * sizeof( Neighbours ) );
			std::memset( neighbourFacesForVertex, 255, entry.numVertices * sizeof( NeighbourFaces ) );

			assert( entry.numVertices < std::size( knownNeighbourFacesCounts ) );
			std::memset( knownNeighbourFacesCounts, 0, sizeof( uint8_t ) * entry.numVertices );

			const unsigned faceOffset = entry.firstIndex / 3;
			const unsigned facesCount = entry.numIndices / 3;

			// Build lists of neighbour face indices for every vertex

			unsigned numVerticesWithCompleteNeighbourFaces = 0;
			for( unsigned faceIndex = faceOffset; faceIndex < faceOffset + facesCount; ++faceIndex ) {
				numVerticesWithCompleteNeighbourFaces += addFaceToNeighbours( faceIndex,
																			  neighbourFacesForVertex,
																			  knownNeighbourFacesCounts );
				if( numVerticesWithCompleteNeighbourFaces == entry.numVertices ) {
					break;
				}
			}

			// Build lists of neighbour vertex indices for every vertex
			for( unsigned vertexIndex = 0; vertexIndex < entry.numVertices; ++vertexIndex ) {
				assert( knownNeighbourFacesCounts[vertexIndex] == kNumFaceNeighbours );

				buildNeighboursForVertexUsingNeighbourFaces( vertexIndex,
															 &neighbourVerticesForVertex[vertexIndex],
															 &neighbourFacesForVertex[vertexIndex] );
			}
		}

		// TODO: Use a fixed vector
		m_neighbours.shrink_to_fit();
	}

	wsw::Vector<Vertex> m_vertices;
	wsw::Vector<Face> m_faces;
	wsw::Vector<Neighbours> m_neighbours;

	wsw::StaticVector<IcosphereData, kMaxSubdivLevel + 1> m_icospheresForLevels;
};

static BasicHullsHolder basicHullsHolder;

std::span<const vec4_t> SimulatedHullsSystem::getUnitIcosphere(unsigned level) {
	const auto [verticesSpan, indicesSpan, neighbourSpan] = ::basicHullsHolder.getIcosphereForLevel(level);
	return verticesSpan;
}

SimulatedHullsSystem::SimulatedHullsSystem() {
	const auto seedUuid  = mm_uuid_t::Random();
	const auto qwordSeed = seedUuid.loPart ^ seedUuid.hiPart;
	const auto dwordSeed = (uint32_t)( ( qwordSeed >> 32 ) ^ ( qwordSeed & 0xFFFFFFFFu ) );
	// TODO: Use the same instance for all effect subsystems
	m_rng.setSeed( dwordSeed );

	m_storageOfSubmittedMeshOrderDesignators.reserve( kMaxMeshesPerHull * kMaxHullsWithLayers );
	m_storageOfSubmittedMeshPtrs.reserve( kMaxMeshesPerHull * kMaxHullsWithLayers );

	// TODO: Take care of exception-safety
	while( !m_freeShapeLists.full() ) {
		if( auto *shapeList = CM_AllocShapeList( cl.cms ) ) [[likely]] {
			m_freeShapeLists.push_back( shapeList );
		} else {
			wsw::failWithBadAlloc();
		}
	}
	// TODO: Take care of exception-safety
	if( !( m_tmpShapeList = CM_AllocShapeList( cl.cms ) ) ) [[unlikely]] {
		wsw::failWithBadAlloc();
	}
}

SimulatedHullsSystem::~SimulatedHullsSystem() {
	clear();
	for( CMShapeList *shapeList: m_freeShapeLists ) {
		CM_FreeShapeList( cl.cms, shapeList );
	}
	CM_FreeShapeList( cl.cms, m_tmpShapeList );
}

// facilities for dealing with md3
//bool GetGeometryFromFileAliasMD3( const char *fileName, Geometry *outGeometry, const char *meshName = nullptr, const unsigned chosenFrame = 0 );
unsigned GetNumFramesInMD3( const char *fileName );

#define maxLODs 3

// static hulls:
void SimulatedHullsSystem::StaticCagedMesh::preserveVolumeStatic( wsw::StringView pathToMesh ){
	const unsigned numFrames = this->numFrames;
	const unsigned numVertices = this->numVertices;
	this->offsetFromLim = new float[numFrames * numVertices];

	Geometry mesh;

	for( unsigned frameNum = 0; frameNum < numFrames; frameNum++ ) {
		GetGeometryFromFileAliasMD3( pathToMesh.data(), &mesh, nullptr, frameNum );

		float *offsetFromLim  = this->offsetFromLim + frameNum * numVertices;
		vec3_t *vertPositions = mesh.vertexPositions.data();

		for( unsigned vertexNum = 0; vertexNum < numVertices; vertexNum++ ) {
			vec3_t vertOrigin, vertDir;
			VectorScale( vertPositions[vertexNum], 1.01f, vertOrigin ); // begin slightly away from the face by scaling with 1.01f
			VectorCopy( vertPositions[vertexNum], vertDir ); // the direction of movement from the origin along the path to the vertex is its position
            VectorNormalizeFast( vertDir );

			bool foundCollision = false;

			foundCollision = collisionCheck( &mesh, vertOrigin, vertDir, std::numeric_limits<float>::infinity(),
											 nullptr, &offsetFromLim[vertexNum], nullptr );
			if( !foundCollision ){
				offsetFromLim[vertexNum] = this->boundingRadii[frameNum] * 4e-2f ;
			}

            if( frameNum == numFrames && vertexNum % 100 ){
                cgNotice() << "offset from lim" << offsetFromLim[vertexNum];
            }
		}

	}
}

//void SimulatedHullsSystem::StaticCagedMesh::optimizeStaticCagedMeshForCache(){ // still broken for now
//    const unsigned numVerts  = this->numVertices;
//    const unsigned numFrames = this->numFrames;
//    const unsigned numTris   = this->triIndices.size();
//
//    StaticCage *cage = std::addressof( cg.simulatedHullsSystem.m_loadedStaticCages[this->loadedCageKey] );
//    const unsigned cageTris = cage->cageGeometry.triIndices.size();
//
//    StaticCageCoordinate *originalCoordinates = this->vertexCoordinates;
//    tri *originalTriIndices                   = this->triIndices.data();
//    float *originalOffsetFromLimit            = this->offsetFromLim;
//    vec2_t *originalUVCoords                  = this->UVCoords;
//
//    StaticCageCoordinate *newCoordinates = new StaticCageCoordinate[numVerts * numFrames];
//    tri *newTriIndices                   = new tri[numTris];
//    float *newOffsetFromLimit            = new float[numVerts * numFrames];
//    vec2_t *newUVCoords                  = new vec2_t[numVerts * numFrames];
//
//    unsigned *numOfVertsForCageTri     = new unsigned[cageTris];
//    unsigned *originalIndicesForCageTri = new unsigned[cageTris * numVerts];
//
//    for( unsigned vertNum = 0; vertNum < numVerts; vertNum++ ){
//        unsigned cageTriIdxOfVert = originalCoordinates->cageTriIdx;
//        numOfVertsForCageTri[cageTriIdxOfVert]++;
//        unsigned vertIdxForCageTri = numOfVertsForCageTri[cageTriIdxOfVert] - 1;
//        originalIndicesForCageTri[cageTriIdxOfVert * numVerts + vertIdxForCageTri] = vertNum;
//    }
//    unsigned newProcessedVertices = 0;
//    for( unsigned cageTriNum = 0; cageTriNum < cageTris; cageTriNum++ ){
//        for( unsigned vertNum = 0; vertNum < numOfVertsForCageTri[cageTriNum]; vertNum++ ){
//            const unsigned newIdx = newProcessedVertices + vertNum;
//            const unsigned oldIdx = originalIndicesForCageTri[cageTriNum * numVerts + vertNum];
//
//            newCoordinates[newIdx]          = originalCoordinates[oldIdx];
//            originalOffsetFromLimit[newIdx] = originalOffsetFromLimit[oldIdx];
//
//            newTriIndices[newIdx][0] = originalTriIndices[oldIdx][0];
//            newTriIndices[newIdx][1] = originalTriIndices[oldIdx][1];
//            newTriIndices[newIdx][2] = originalTriIndices[oldIdx][2];
//
//            newUVCoords[newIdx][0] = originalUVCoords[oldIdx][0];
//            newUVCoords[newIdx][1] = originalUVCoords[oldIdx][1];
//        }
//        newProcessedVertices += numOfVertsForCageTri[cageTriNum];
//    }
//    std::span<tri> spanOfNewTriIndices( newTriIndices, numTris );
//
//    delete[] this->vertexCoordinates;
//    delete[] this->offsetFromLim;
//    delete[] this->triIndices.data();
//    delete[] this->UVCoords;
//
//    this->vertexCoordinates = newCoordinates;
//    this->offsetFromLim     = newOffsetFromLimit;
//    this->triIndices        = spanOfNewTriIndices;
//    this->UVCoords          = newUVCoords;
//}

void SimulatedHullsSystem::RegisterStaticCage( const wsw::String &identifier ) {
    m_loadedStaticCages.emplace_back();
    StaticCage *cage = std::addressof( m_loadedStaticCages.back() );

    cage->identifier = identifier;

    cgNotice() << "cage identifier found:" << cage->identifier;

    const char *format = ".md3"; // we are only using md3 format for now
    auto filepathToCage = wsw::String( cage->identifier.data(), cage->identifier.length() ).append( format );
    cgNotice() << "path to cage:" << filepathToCage;

    Geometry *cageGeometry = &cage->cageGeometry;
    GetGeometryFromFileAliasMD3( filepathToCage.data(), cageGeometry );
    //unitizeGeometry( cageGeometry );

    unsigned numCageVertices = cageGeometry->vertexPositions.size();
    cgNotice() << "number of vertices in cage:" << numCageVertices;

	float maxRadius = 0;
	for( unsigned i = 0; i < numCageVertices; i++ ){
		if( VectorLengthSquared( cageGeometry->vertexPositions[i] ) > 1e-3f ){
			const float radius = VectorLengthFast( cageGeometry->vertexPositions[i] );
			maxRadius = wsw::max( maxRadius, radius );
		}
	}
	cage->boundingRadius = maxRadius;

    size_t sizeOfBaseHull = sizeof( SimulatedHullsSystem::KeyframedHull );
    size_t sizeOfMoveDirs = sizeof( *BaseKeyframedHull::vertexMoveDirections ) * numCageVertices;
    size_t sizeOfLimits = sizeof( *BaseKeyframedHull::limitsAtDirections ) * numCageVertices;
    size_t requiredSize = sizeOfBaseHull + sizeOfMoveDirs + sizeOfLimits;

    cage->allocator = new wsw::HeapBasedFreelistAllocator( requiredSize, maxCagedHullsPerType );
}

bool SimulatedHullsSystem::StaticCagedMesh::transformToCageSpace( Geometry *cage, wsw::StringView pathToMesh ){
    const auto before = Sys_Milliseconds();

	vec3_t origin = { 0.0f, 0.0f, 0.0f };

    unsigned numFrames = GetNumFramesInMD3( pathToMesh.data() );
    cgNotice() << "number of frames in caged mesh:" << numFrames;
    this->numFrames = numFrames;

    Geometry mesh;
    if( !GetGeometryFromFileAliasMD3( pathToMesh.data(), &mesh, nullptr, 0 ) ){
        return false;
    }
    const unsigned meshVerts = mesh.vertexPositions.size();

    this->numVertices = meshVerts;

    // i find this so ugly.. isn't there something like copy()
//    auto triIndicesCopy = new tri[mesh.triIndices.size()];
//    auto UVCoordsCopy  = new vec2_t[meshVerts];
//    std::memcpy( triIndicesCopy, mesh.triIndices.data(), mesh.triIndices.size() * sizeof(tri) );
//    std::memcpy( UVCoordsCopy, mesh.UVCoords, meshVerts * sizeof(vec2_t) );
//    this->triIndices = std::span( triIndicesCopy, mesh.triIndices.size() );
//    this->UVCoords   = UVCoordsCopy;
    this->triIndices = mesh.copyTriIndices();
    this->UVCoords   = mesh.copyUVCoords().data();

    this->vertexCoordinates = new SimulatedHullsSystem::StaticCageCoordinate[meshVerts * numFrames];
	this->offsetFromLim = new float[meshVerts * numFrames];
    this->boundingRadii = new float[numFrames];

    for( unsigned frameNum = 0; frameNum < numFrames; frameNum++ ){
        GetGeometryFromFileAliasMD3( pathToMesh.data(), &mesh, nullptr, frameNum );

        vec3_t *vertexPositions = mesh.vertexPositions.data();

        SimulatedHullsSystem::StaticCageCoordinate *cageCoordinates = &this->vertexCoordinates[frameNum * meshVerts];

        float maxRadius = 0.0f;
        for( unsigned vertNum = 0; vertNum < meshVerts; vertNum++ ) {
            vec3_t vertPosition;
            VectorCopy( vertexPositions[vertNum], vertPosition );
            bool foundCoords = false;

			SimulatedHullsSystem::StaticCageCoordinate *cageCoord = &cageCoordinates[vertNum];
			if( VectorLengthFast( vertPosition ) < 1e-3f ) {
				// in this case, the vertex is practically at the origin. That means it doesn't matter which cage face it belongs to,
				// as it's going to stay at the origin either way. Otherwise, the following collision check to determine the cage face
				// that the vertex belongs to may not find any solutions.

				cageCoord->cageTriIdx = 0;
				vec2_t coordinates = { 0.0f, 0.0f };
				Vector2Copy( coordinates, cageCoord->coordsOnCageTri );

				foundCoords = true;
			} else {
				foundCoords = collisionCheck( cage, origin, vertPosition, std::numeric_limits<float>::infinity(),
											  &cageCoord->cageTriIdx, nullptr, cageCoord->coordsOnCageTri );
			}

			if( foundCoords ) {
				cageCoord->offset = VectorLengthFast( vertPosition );
				maxRadius = wsw::max( maxRadius, cageCoord->offset );
			} else {
				cgNotice() << S_COLOR_RED << "no coords found for" << vertNum << "in frame: " << frameNum;
				return false;
			}
        }

        this->boundingRadii[frameNum] = maxRadius;
		float *currFrameOffsetsFromLim = this->offsetFromLim + frameNum * meshVerts;
		std::fill( currFrameOffsetsFromLim, currFrameOffsetsFromLim + meshVerts,  maxRadius * 6e-2f );

    }

    Com_Printf("It took %d millis\n", (int)(Sys_Milliseconds() - before));
	return true;
}

SimulatedHullsSystem::StaticCagedMesh *SimulatedHullsSystem::RegisterStaticCagedMesh( const char *name ) {
    auto *cagedMesh = new SimulatedHullsSystem::StaticCagedMesh;
    auto filepath = wsw::StringView( name );
    cgNotice() << "starting static caged mesh registration for" << filepath;

    unsigned suffixIdx = filepath.lastIndexOf('_').value_or( 0 );
    const bool foundSuffix = suffixIdx != 0;
    if( !foundSuffix ) {
        cgNotice() << "static caged mesh " << wsw::StringView( name ) << "is incorrectly formatted";
        return nullptr;
    }

	unsigned suffixLength = filepath.length() - suffixIdx;
	auto identifier = filepath.dropRight( suffixLength );

	bool foundCage = false;
	unsigned loadedCageNum = 0;

	for( ; loadedCageNum < m_loadedStaticCages.size(); loadedCageNum++ ) {
		StaticCage *loadedCage = std::addressof( m_loadedStaticCages[loadedCageNum] );
        cgNotice() << S_COLOR_BLUE << "loaded cage num:" << loadedCageNum << "of" << m_loadedStaticCages.size() << S_COLOR_WHITE;
		if( wsw::StringView( loadedCage->identifier.data(), loadedCage->identifier.size() ).equals( identifier ) ){
			foundCage = true;
			break;
		}
	}

	if( foundCage ) {
		//cagedMesh->cage = foundCage;
		cagedMesh->loadedCageKey = loadedCageNum;
		StaticCage *cage = std::addressof( m_loadedStaticCages[loadedCageNum] );
		cgNotice() << S_COLOR_GREEN << "cage" << cage->identifier << "was already loaded" << "for filepath:" << filepath << S_COLOR_WHITE;
	} else {
		RegisterStaticCage( wsw::String( identifier.data(), identifier.length() ) );
		cagedMesh->loadedCageKey = m_loadedStaticCages.size() - 1; // -1 to convert to idx of the elem
	}

    StaticCage *cage = std::addressof( m_loadedStaticCages[cagedMesh->loadedCageKey] );

    cagedMesh->transformToCageSpace( &cage->cageGeometry, filepath );
	applyShading( filepath, cagedMesh ); /// TODO: for applyShading, preserve volume methods, etc, these should be called ONCE for all meshes, not separately for main/LODs
    cagedMesh->offsetFromLim = new float[cagedMesh->numFrames * cagedMesh->numVertices];
    std::fill( cagedMesh->offsetFromLim, cagedMesh->offsetFromLim + cagedMesh->numFrames * cagedMesh->numVertices, 0.0f );

	//cagedMesh->preserveVolumeStatic( filepath );
    //cagedMesh->optimizeStaticCageForCache();

	SimulatedHullsSystem::StaticCagedMesh *lastLOD = cagedMesh;
    for( unsigned LODnum = 1; LODnum <= maxLODs; LODnum++ ){
        wsw::String filepathToLOD = wsw::String( filepath.data(), filepath.length() );
		unsigned lengthOfMD3Suffix = sizeof( ".md3" );
        filepathToLOD.insert( filepathToLOD.end() - lengthOfMD3Suffix + 1, '_' );
        filepathToLOD.insert( filepathToLOD.length() - lengthOfMD3Suffix + 1, std::to_string( LODnum ) );

        cgNotice() << "next LOD" << wsw::StringView( filepathToLOD.data() );

        auto *currLOD = new SimulatedHullsSystem::StaticCagedMesh;
		lastLOD->nextLOD = currLOD;
        if( currLOD->transformToCageSpace( &cage->cageGeometry, wsw::StringView( filepathToLOD.data() ) ) ){
			applyShading( wsw::StringView( filepathToLOD.data() ), currLOD );
			currLOD->preserveVolumeStatic( wsw::StringView( filepathToLOD.data() ) );
            //currLOD->optimizeStaticCagedMeshForCache();
			lastLOD = lastLOD->nextLOD;
            lastLOD->nextLOD = nullptr;
		} else{
			delete currLOD;
			lastLOD->nextLOD = nullptr;
			cgNotice() << "found" << LODnum - 1 << "LODs";
			break;
		}
    }

    return cagedMesh;
}

void JacobianForDynamicCageTransform( vec3_t coords, const vec3_t *triPositions, const vec3_t *triNormals,
									  mat3_t outJacobian ) {
	const vec2_t coordsOnTri = { coords[0], coords[1] }; // 2: vertCoords
	const float L = coords[2];

	vec3_t position;

	vec3_t firstToSecond;
	vec3_t firstToThird;

	VectorSubtract( triPositions[1], triPositions[0], firstToSecond );
	VectorSubtract( triPositions[2], triPositions[0], firstToThird );

	vec3_t normalDiffSecond;
	vec3_t normalDiffThird;

	VectorSubtract( triNormals[1], triNormals[0], normalDiffSecond );
	VectorSubtract( triNormals[2], triNormals[0], normalDiffThird );

	vec3_t firstColumn;
	vec3_t secondColumn;
	vec3_t thirdColumn;

	VectorMA( firstToSecond, L, normalDiffSecond, firstColumn );
	VectorMA( firstToThird, L, normalDiffThird, secondColumn );
	VectorMA( triNormals[0], coordsOnTri[0], normalDiffSecond, thirdColumn );
	VectorMA( thirdColumn, coordsOnTri[1], normalDiffThird, thirdColumn );

	const unsigned numRows = 3;
	VectorCopy( firstColumn, &outJacobian[0*numRows] );
	VectorCopy( secondColumn, &outJacobian[1*numRows] );
	VectorCopy( thirdColumn, &outJacobian[2*numRows] );
	// we do NOT normalize the outcome of the linear combination of normals to keep the math simple,
	// such that the moveDir is a linear combination of the normals (it is not upon normalization)

//	VectorMA( triPositions[0], coordsOnTri[0], firstToSecond, outPos );
//	VectorMA( outPos, coordsOnTri[1], firstToThird, outPos );
//	VectorMA( outPos, L, triNormals[0], outPos );
//	VectorMA( outPos, coordsOnTri[0] * L, normalDiffSecond, outPos );
//	VectorMA( outPos, coordsOnTri[1] * L, normalDiffThird, outPos );

}

void vertexPosForDynamicCageTransform( vec3_t coords, const vec3_t *triPositions, const vec3_t *triNormals,
									   vec3_t outPos ) {

	const vec2_t coordsOnTri = { coords[0], coords[1] }; // 2: vertCoords
	const float L = coords[2];

	//vec3_t position;

//	const float coeff0 = 1 - (coordsOnTri[0] + coordsOnTri[1]);
//	const float coeff1 = coordsOnTri[0];
//	const float coeff2 = coordsOnTri[1];
//
//	VectorScale( triPositions[0], coeff0, position );
//	VectorMA( position, coeff1, triPositions[1], position );
//	VectorMA( position, coeff2, triPositions[2], position );
//
//	vec3_t moveDir;
//	VectorScale( triNormals[0], coeff0, moveDir );
//	VectorMA( moveDir, coeff1, triNormals[1], moveDir );
//	VectorMA( moveDir, coeff2, triNormals[2], moveDir );
//	// we do NOT normalize here to keep the math simple, such that the moveDir is a linear combination of the normals (it is not upon normalization)
//
//	VectorMA( position, L, moveDir, outPos );

	vec3_t firstToSecond;
    vec3_t firstToThird;

    VectorSubtract( triPositions[1], triPositions[0], firstToSecond );
    VectorSubtract( triPositions[2], triPositions[0], firstToThird );

    vec3_t normalDiffSecond;
    vec3_t normalDiffThird;

    VectorSubtract( triNormals[1], triNormals[0], normalDiffSecond );
    VectorSubtract( triNormals[2], triNormals[0], normalDiffThird );

    VectorMA( triPositions[0], coordsOnTri[0], firstToSecond, outPos );
    VectorMA( outPos, coordsOnTri[1], firstToThird, outPos );
    VectorMA( outPos, L, triNormals[0], outPos );
    VectorMA( outPos, coordsOnTri[0] * L, normalDiffSecond, outPos );
    VectorMA( outPos, coordsOnTri[1] * L, normalDiffThird, outPos );
	// we do NOT normalize the outcome of the linear combination of normals to keep the math simple,
	// such that the moveDir is a linear combination of the normals (it is not upon normalization)
}

void getErrorForDynamicCageTransform( vec3_t coords, const vec3_t *triPositions, const vec3_t *triNormals,
									  const vec3_t vertexPosition, vec3_t outError ){

//    vec2_t coordsOnTri;
//    Vector2Copy( coords, coordsOnTri );
//    const float offset = coords[2];
//
//    vec3_t firstToSecond;
//    vec3_t firstToThird;
//
//    VectorSubtract( triVertices[1], triVertices[0], firstToSecond );
//    VectorSubtract( triVertices[2], triVertices[0], firstToThird );
//
//    vec3_t normalDiffSecond;
//    vec3_t normalDiffThird;
//
//    VectorSubtract( triNormals[1], triNormals[0], normalDiffSecond );
//    VectorSubtract( triNormals[2], triNormals[0], normalDiffThird );
//
//    vec3_t position;
//
//    VectorMA( triVertices[0], coordsOnTri[0], firstToSecond, position );
//    VectorMA( position, coordsOnTri[1], firstToThird, position );
//    VectorMA( position, offset, triNormals[0], position );
//    VectorMA( position, coordsOnTri[0] * offset, normalDiffSecond, position );
//    VectorMA( position, coordsOnTri[1] * offset, normalDiffThird, position );
	vec3_t resultingPosition;

	vertexPosForDynamicCageTransform( coords, triPositions, triNormals, resultingPosition );

//	cgNotice() << "coords:" << coords[0] << coords[1] << coords[2];
////	cgNotice() << "triPositions[0]:" << triPositions[0][0] << triPositions[0][1] << triPositions[0][2];
////	cgNotice() << "triNormals[0]:" << triNormals[0][0] << triNormals[0][1] << triNormals[0][2];
//	cgNotice() << "resultingPosition:" << resultingPosition[0] << resultingPosition[1] << resultingPosition[2];
//	cgNotice() << "vertexPosition:" << vertexPosition[0] << vertexPosition[1] << vertexPosition[2];

    VectorSubtract( resultingPosition, vertexPosition, outError );
}

void SimulatedHullsSystem::RegisterDynamicCage( const wsw::String &identifier ) {
    const auto before = Sys_Milliseconds();

    m_loadedDynamicCages.emplace_back();
    DynamicCage *cage = std::addressof( m_loadedDynamicCages.back() );

    cage->identifier = identifier;

    cgNotice() << "cage identifier found:" << cage->identifier;

    const char *format = ".md3"; // we are only using md3 format for now
    auto filepathToCage = wsw::String( cage->identifier.data(), cage->identifier.length() ).append( format );
    cgNotice() << "path to cage:" << filepathToCage;

    Geometry *cageInitialGeometry = &cage->cageInitialGeometry;
    GetGeometryFromFileAliasMD3( filepathToCage.data(), cageInitialGeometry );

    unsigned numCageVertices = cageInitialGeometry->vertexPositions.size();
    unsigned numFrames       = GetNumFramesInMD3( filepathToCage.data() );
    cgNotice() << "number of vertices in cage:" << numCageVertices;

    float maxRadius = 0;
    for( unsigned i = 0; i < numCageVertices; i++ ){
        if( VectorLengthSquared( cageInitialGeometry->vertexPositions[i] ) > 1e-3f ){
            const float radius = VectorLengthFast( cageInitialGeometry->vertexPositions[i] );
            maxRadius = wsw::max( maxRadius, radius );
        }
    }

    cage->initialBoundingRadius = maxRadius;
    cage->numFrames             = numFrames;
    cage->vertexVelocities      = new vec3_t[numCageVertices * ( numFrames - 1 )]; // we cant calculate/ dont use velocities at the last frame
	cage->maxVelocityThisFrame  = new float[numFrames - 1];

    Geometry prevGeom, nextGeom;
    for( unsigned frameNum = 0; frameNum < ( numFrames - 1 ); frameNum++ ){
//vec3_t *prevVertexPositions;
		if( frameNum == 0 ) {
			GetGeometryFromFileAliasMD3( filepathToCage.data(), &prevGeom, nullptr, frameNum );
			//prevVertexPositions = prevGeom.vertexPositions.data();
		} else {
			prevGeom = nextGeom;
			//prevVertexPositions = nextGeom.vertexPositions.data();
		}

		//GetGeometryFromFileAliasMD3( filepathToCage.data(), &prevGeom, nullptr, frameNum );
        GetGeometryFromFileAliasMD3( filepathToCage.data(), &nextGeom, nullptr, ( frameNum + 1 ) );

        vec3_t *prevVertexPositions = prevGeom.vertexPositions.data();
        vec3_t *nextVertexPositions = nextGeom.vertexPositions.data();

        unsigned startVertIdxForFrame = frameNum * numCageVertices;
        vec3_t *vertexVelocities = cage->vertexVelocities + startVertIdxForFrame;
		float *maxVelocityThisFrame = cage->maxVelocityThisFrame + frameNum;
        for( unsigned vertNum = 0; vertNum < numCageVertices; vertNum++ ){
            VectorSubtract( nextVertexPositions[vertNum], prevVertexPositions[vertNum], vertexVelocities[vertNum] );
			*maxVelocityThisFrame = wsw::max( VectorLengthFast( vertexVelocities[vertNum] ), *maxVelocityThisFrame );
        }
    }

    size_t sizeOfBaseHull = sizeof( SimulatedHullsSystem::DynamicCageHull );
    size_t sizeOfPositions = sizeof( *DynamicCageHull::vertexPositions ) * numCageVertices;
    size_t sizeOfNormals = sizeof( *DynamicCageHull::vertexNormals ) * numCageVertices;
    size_t sizeOfLimits = sizeof( *DynamicCageHull::limitsAtDirections ) * numCageVertices;
    size_t requiredSize = sizeOfBaseHull + sizeOfPositions + sizeOfNormals + sizeOfLimits;

    cage->allocator = new wsw::HeapBasedFreelistAllocator( requiredSize, maxCagedHullsPerType );

    cgNotice() << "it took" << Sys_Milliseconds() - before << "millis";
}

//void add(vec3_t x, int y) { cgNotice() << x[0] + y; }
//void multiply(vec3_t x, int y) { cgNotice() << x[0] * y; }
//
//// Function that accepts an object of
//// type std::function<> as a parameter
//// as well
//void invoke(vec3_t x, int y, std::function<void(vec3_t, int)> func)
//{
//	return func(x, y);
//}



bool SimulatedHullsSystem::DynamicCagedMesh::transformToCageSpace( DynamicCage *cage, wsw::StringView pathToMesh ){
	const auto before = Sys_Milliseconds();
	const Geometry *initialCageGeometry = &cage->cageInitialGeometry;

	vec3_t origin = { 0.0f, 0.0f, 0.0f };

	numFrames = GetNumFramesInMD3( pathToMesh.data() );
	if( numFrames != cage->numFrames ) {
		cgNotice() << S_COLOR_RED << "number of frames in dynamic caged mesh doesnt match number of frames in cage";
		return false;
	}

	Geometry mesh;
	if( !GetGeometryFromFileAliasMD3( pathToMesh.data(), &mesh, nullptr, 0 ) ){
		return false;
	}
	const unsigned meshVerts = mesh.vertexPositions.size();

	this->numVertices = meshVerts;

	// i find this so ugly.. isn't there something like copy()
	auto triIndicesCopy = new tri[mesh.triIndices.size()];
	auto UVCoordsCopy  = new vec2_t[meshVerts];
	std::memcpy( triIndicesCopy, mesh.triIndices.data(), mesh.triIndices.size() * sizeof(tri) );
	std::memcpy( UVCoordsCopy, mesh.UVCoords, meshVerts * sizeof(vec2_t) );
	this->triIndices = std::span( triIndicesCopy, mesh.triIndices.size() );
	this->UVCoords   = UVCoordsCopy;

	this->vertexCoordinates = new SimulatedHullsSystem::DynamicCageCoordinate[meshVerts * numFrames];
	this->offsetFromLim = new float[meshVerts * numFrames];
	this->maxOffsetFromCageForFrame = new float[numFrames];

	const unsigned cageVerts = initialCageGeometry->vertexPositions.size();
	auto cageVertPositions = (vec3_t *)alloca( sizeof( vec3_t ) * cageVerts );
	auto cageVertNormals   = (vec3_t *)alloca( sizeof( vec3_t ) * cageVerts );
	std::memcpy( cageVertPositions, initialCageGeometry->vertexPositions.data(), cageVerts * sizeof(vec3_t) );

	std::span<tri> cageTriIndices = initialCageGeometry->triIndices;
	const unsigned cageFaces = cageTriIndices.size();

	Geometry cageMesh;
	for( unsigned frameNum = 0; frameNum < numFrames; frameNum++ ){
		const int lastFrame = (int)frameNum - 1;
		if( lastFrame >= 0 ) {
			vec3_t *distancesTravelled = cage->vertexVelocities + cageVerts * lastFrame; // velocity [units/frame] * 1 [frame]
			for( unsigned vertNum = 0; vertNum < cageVerts; vertNum++ ){
				VectorAdd( cageVertPositions[vertNum], distancesTravelled[vertNum], cageVertPositions[vertNum] );
			}
		}

		GetGeometryFromFileAliasMD3( "gfx/hulls/dynamicExample.md3", &cageMesh, nullptr, frameNum );

		///
		// TODO: this could be more accurate with getVolume( cageVertPositions, etc ) and avgCageExtent = (volume)^(1/3)
		BoundsBuilder cageBoundsBuilder;
		vec3_t cageMins, cageMaxs;
		for( unsigned vertNum = 0; vertNum < cageVerts; vertNum++ ) {
			cageBoundsBuilder.addPoint( cageVertPositions[vertNum] );
			vec3_t error;
			VectorSubtract( cageVertPositions[vertNum], cageMesh.vertexPositions[vertNum], error );
			cgNotice() << "diff between loaded from md3 and simulated:" << VectorLengthFast( error );
		}
		cageBoundsBuilder.storeTo( cageMins, cageMaxs );
		const float avgCageExtent = 0.333f * ( cageMaxs[0] - cageMins[0] + cageMaxs[1] - cageMins[1] + cageMaxs[2] - cageMins[2] );
		///
		constexpr float avgVertexSeparation = 3.0f/100; // percentage of average extent based on a cube
		const float maxAllowableError = 0.2f * avgVertexSeparation;
		const float thresholdError    = 0.005f * avgVertexSeparation;

		vec3_t initialCoordsGuess = { 0.33333f, 0.33333f, 0.2f * avgCageExtent };

		calculateNormals( cageTriIndices, cageVerts, cageVertPositions, cageVertNormals );

		GetGeometryFromFileAliasMD3( pathToMesh.data(), &mesh, nullptr, frameNum );
		vec3_t *meshVertPositions = mesh.vertexPositions.data();

		SimulatedHullsSystem::DynamicCageCoordinate *cageCoordinates = &this->vertexCoordinates[frameNum * meshVerts];

		float maxOffsetFromCage = 0.0f;
		for( unsigned vertNum = 0; vertNum < meshVerts; vertNum++ ) {
			vec3_t meshVertPosition;
			VectorCopy( meshVertPositions[vertNum], meshVertPosition );
			bool foundCoords = false;

			SimulatedHullsSystem::DynamicCageCoordinate *cageCoord = &cageCoordinates[vertNum];
			cageCoord->offsetOnTri = std::numeric_limits<float>::infinity();

			for( unsigned faceNum = 0; faceNum < cageFaces; faceNum++ ){
				//cgNotice() << S_COLOR_BLUE << "face" << faceNum << "/" << cageFaces;

				const uint16_t *faceIndices = cageTriIndices[faceNum];
				vec3_t triPositions[3];
				vec3_t triNormals[3];

				getTriCoords( faceIndices, cageVertPositions, triPositions );
				getTriNormals( faceIndices, cageVertNormals, triNormals );

				std::function<void( vec3_t, vec3_t )> function = [triPositions, triNormals, meshVertPosition]( vec3_t coords, vec3_t outError ) {
					getErrorForDynamicCageTransform( coords, triPositions, triNormals, meshVertPosition, outError );
				};
				std::function<void( vec3_t, mat3_t )> getFunctionJacobian = [triPositions, triNormals]( vec3_t coords, vec3_t outJacobian ){
					JacobianForDynamicCageTransform( coords, triPositions, triNormals, outJacobian );
				};

//				vec3_t error;
//				function( initialCoordsGuess, error );
//				cgNotice() << "initial guess:" << initialCoordsGuess[0] << initialCoordsGuess[1] << initialCoordsGuess[2];
//				cgNotice() << "initial error, length:" << VectorLengthFast(error) << "components:" << error[0] << error[1] << error[2];

				constexpr unsigned maxIterations = 24; // arbitrarily chosen, not sure if optimal
				vec3_t outCoords;

				float offset;
				if( findRootVec3Newton( initialCoordsGuess, maxAllowableError, thresholdError,
										maxIterations, function, getFunctionJacobian,outCoords ) ) {
					// convert the L coordinate to physical offset that we did not do inside the function:
					vec3_t linCombiOfNormals;
					VectorScale( triNormals[0], 1 - ( outCoords[0] + outCoords[1] ), linCombiOfNormals );
					VectorMA( linCombiOfNormals, outCoords[0], triNormals[1], linCombiOfNormals );
					VectorMA( linCombiOfNormals, outCoords[1], triNormals[2], linCombiOfNormals );
					offset = outCoords[2] * Q_RSqrt( VectorLengthSquared(linCombiOfNormals) );

					//cgNotice() << "offset:" << offset;
				} else {
					continue;
				}
				vec2_t coordsOnTri = { outCoords[0], outCoords[1] };

				constexpr float margin = 5e-3f; // to account for floating point error and solver error
				const float compareValue = offset > 0.0f ? offset : 2 * abs( offset ); // prefer positive offset
				const bool isOnTri = ((coordsOnTri[0] + coordsOnTri[1]) < 1.0f + margin) &&
						              (coordsOnTri[0] > 0.0f - margin) && (coordsOnTri[1] > 0.0f - margin);
				if( isOnTri && ( compareValue < cageCoord->offsetOnTri ) ){
					//return false;
					cageCoord->offsetOnTri = offset;
					cageCoord->cageTriIdx = faceNum;
					cageCoord->coordsOnCageTri[0] = coordsOnTri[0];
					cageCoord->coordsOnCageTri[1] = coordsOnTri[1];

//					cgNotice() << "offset:" << offset;
//					vec3_t error;
//					function( outCoords, error );
//					cgNotice() << "error:" << VectorLengthFast( error );
//					cgNotice() << "relative error:" << VectorLengthFast( error ) * Q_Rcp( avgCageExtent );
				}
			}

			if( cageCoord->offsetOnTri == std::numeric_limits<float>::infinity() ){
				cgNotice() << S_COLOR_RED << "no coords found for vert:" << vertNum << "in frame:" << frameNum;
				cgNotice() << S_COLOR_RED << "position:" << S_COLOR_WHITE << meshVertPosition[0] << meshVertPosition[1] << meshVertPosition[2];
				return false;
			} else {
				cgNotice() << S_COLOR_ORANGE << "frame:" << frameNum << "vert:" << vertNum <<
				"found fit with coords:" << cageCoord->coordsOnCageTri[0] << cageCoord->coordsOnCageTri[1] <<
				"offset:" << cageCoord->offsetOnTri << "tri index:" << cageCoord->cageTriIdx;
			}
//			if( VectorLengthFast( meshVertPosition ) < 1e-3f ) {
//				// in this case, the vertex is practically at the origin. That means it doesn't matter which cage face it belongs to,
//				// as it's going to stay at the origin either way. Otherwise, the following collision check to determine the cage face
//				// that the vertex belongs to may not find any solutions.
//
//				cageCoord->cageTriIdx = 0;
//				vec2_t coordinates = { 0.0f, 0.0f };
//				Vector2Copy( coordinates, cageCoord->coordsOnCageTri );
//
//				foundCoords = true;
//			} else {
//				foundCoords = collisionCheck( cage, origin, meshVertPosition, std::numeric_limits<float>::infinity(),
//											  &cageCoord->cageTriIdx, nullptr, cageCoord->coordsOnCageTri );
//			}
//
//			if( foundCoords ) {
//				cageCoord->offset = VectorLengthFast( meshVertPosition );
//				maxRadius = wsw::max( maxRadius, cageCoord->offset );
//			} else {
//				cgNotice() << S_COLOR_RED << "no coords found for" << vertNum << "in frame: " << frameNum;
//				return false;
//			}//
		}

		this->maxOffsetFromCageForFrame[frameNum] = maxOffsetFromCage;
		float *currFrameOffsetsFromLim = this->offsetFromLim + frameNum * meshVerts;
		//std::fill( currFrameOffsetsFromLim, currFrameOffsetsFromLim + meshVerts,  maxRadius * 6e-2f );

	}

	Com_Printf("It took %d millis\n", (int)(Sys_Milliseconds() - before));
	return true;
}

SimulatedHullsSystem::DynamicCagedMesh *SimulatedHullsSystem::RegisterDynamicCagedMesh( const char *name ) {
    auto *cagedMesh = new SimulatedHullsSystem::DynamicCagedMesh;
    auto filepath = wsw::StringView( name );
    cgNotice() << "starting dynamic caged mesh registration for" << filepath;

    unsigned suffixIdx = filepath.lastIndexOf('_').value_or( 0 );
    const bool foundSuffix = ( suffixIdx != 0 );
    if( !foundSuffix ) {
        cgNotice() << "static caged mesh " << wsw::StringView( name ) << "is incorrectly formatted";
        return nullptr;
    }

    unsigned suffixLength = filepath.length() - suffixIdx;
    auto identifier = filepath.dropRight( suffixLength );

    bool foundCage = false;
    unsigned loadedCageNum = 0;

    for( ; loadedCageNum < m_loadedDynamicCages.size(); loadedCageNum++ ) {
        DynamicCage *loadedCage = std::addressof( m_loadedDynamicCages[loadedCageNum] );
        cgNotice() << S_COLOR_BLUE << "loaded cage num:" << loadedCageNum << "of" << m_loadedDynamicCages.size() << S_COLOR_WHITE;
        if( wsw::StringView( loadedCage->identifier.data(), loadedCage->identifier.size() ).equals( identifier ) ){
            foundCage = true;
            break;
        }
    }

    if( foundCage ) {
        cagedMesh->loadedCageKey = loadedCageNum;
        DynamicCage *cage = std::addressof( m_loadedDynamicCages[loadedCageNum] );
        cgNotice() << S_COLOR_GREEN << "cage" << cage->identifier << "was already loaded" << "for filepath:" << filepath << S_COLOR_WHITE;
    } else {
        RegisterDynamicCage( wsw::String( identifier.data(), identifier.length() ) );
        cagedMesh->loadedCageKey = m_loadedDynamicCages.size() - 1; // -1 to convert to idx of the elem
    }

    DynamicCage *cage = std::addressof( m_loadedDynamicCages[cagedMesh->loadedCageKey] );

    cagedMesh->transformToCageSpace( cage, filepath );
    //applyShading( filepath, cagedMesh ); /// TODO: for applyShading, preserve volume methods, etc, these should be called ONCE for all meshes, not separately for main/LODs
    cagedMesh->offsetFromLim = new float[cagedMesh->numFrames * cagedMesh->numVertices];
    std::fill( cagedMesh->offsetFromLim, cagedMesh->offsetFromLim + cagedMesh->numFrames * cagedMesh->numVertices, 0.0f );

//    SimulatedHullsSystem::DynamicCagedMesh *lastLOD = cagedMesh;
//    for( unsigned LODnum = 1; LODnum <= maxLODs; LODnum++ ){
//        wsw::String filepathToLOD = wsw::String( filepath.data(), filepath.length() );
//        unsigned lengthOfMD3Suffix = sizeof( ".md3" );
//        filepathToLOD.insert( filepathToLOD.end() - lengthOfMD3Suffix + 1, '_' );
//        filepathToLOD.insert( filepathToLOD.length() - lengthOfMD3Suffix + 1, std::to_string( LODnum ) );
//
//        cgNotice() << "next LOD" << wsw::StringView( filepathToLOD.data() );
//
//        auto *currLOD = new SimulatedHullsSystem::DynamicCagedMesh;
//        lastLOD->nextLOD = currLOD;
//        if( currLOD->transformToCageSpace( &cage->cageInitialGeometry, wsw::StringView( filepathToLOD.data() ) ) ){
//            applyShading( wsw::StringView( filepathToLOD.data() ), currLOD );
//            //currLOD->optimizeStaticCagedMeshForCache();
//            lastLOD = lastLOD->nextLOD;
//            lastLOD->nextLOD = nullptr;
//        } else{
//            delete currLOD;
//            lastLOD->nextLOD = nullptr;
//            cgNotice() << "found" << LODnum - 1 << "LODs";
//            break;
//        }
//    }

    return cagedMesh;
}

void SimulatedHullsSystem::applyShading( wsw::StringView pathToMesh, StaticCagedMesh *cagedMesh ){ // this is smoke/fire shading
	static constexpr byte_vec4_t kFireColors[3] {
			{ 25, 25, 25, 255 },   // Gray
			{ 255, 70, 30, 255 },  // Orange
			{ 255, 160, 45, 255 }, // Yellow
	};

	static constexpr byte_vec4_t kFadeColors[2] {
			{ 0, 0, 0, 255 },
			{ 100, 100, 100, 0 },
	};

	static constexpr byte_vec4_t kHighlightColors[2] {
			{ 55, 55, 55, 0 },
			{ 0, 0, 0, 0 },
	};

	const unsigned numShadingLayers = 3;
	cagedMesh->numShadingLayers = numShadingLayers;

	const unsigned numFrames = cagedMesh->numFrames;
	const unsigned numVertices = cagedMesh->numVertices;

	constexpr unsigned kMinLifetime = 2000;

	constexpr float scrollSpeed     = 1.43f;
	constexpr float scrollDistance  = scrollSpeed * ( (float)kMinLifetime * 1e-3f );

	constexpr float initialSize     = 0.1f;
	constexpr float fadeRange       = 0.12f;
	constexpr float fadeStartAtFrac = 0.1f;
	constexpr float zFadeInfluence  = 0.3f;

	const std::span<const vec4_t> verticesSpan = SimulatedHullsSystem::getUnitIcosphere( 4 );
	assert( verticesSpan.size() == kNumVertices );

	auto *fireVertexMaskStorage = new float[numFrames * numVertices];
	auto *fadeVertexMaskStorage = new float[numFrames * numVertices];

	cagedMesh->shadingLayers = new ShadingLayer[numFrames * numShadingLayers];

	Geometry mesh;

	for( unsigned frameNum = 0; frameNum < numFrames; frameNum++ ) {
		GetGeometryFromFileAliasMD3( pathToMesh.data(), &mesh, nullptr, frameNum );

		const float frameFrac     = (float)(frameNum) / (float)( numFrames - 1 ); // -1 so the final value is 1.0f
		const float fireLifetime     = 0.54f; // 0.47
		const float fireStartFrac    = 0.9f;
		const float fireLifetimeFrac = wsw::min( 1.0f, frameFrac * ( 1.0f / fireLifetime ) );
		const float fireFrac         = fireLifetimeFrac * fireStartFrac + ( 1.0f - fireStartFrac );

		const float scrolledDistance = scrollDistance * frameFrac;

		float *const frameFireVertexMask    = &fireVertexMaskStorage[frameNum * numVertices];
		float *const frameFadeVertexMask    = &fadeVertexMaskStorage[frameNum * numVertices];

		const float initialVelocity = 5.0f;
		const float expansion       = ( 1.0f - initialSize ) * ( 1.0f - std::exp( -initialVelocity * frameFrac ) ) + initialSize;

		vec3_t *vertexPositions = mesh.vertexPositions.data();

		float minOffsetForFrame = std::numeric_limits<float>::infinity();
		float maxOffsetForFrame = 0.f;
		for( unsigned vertexNum = 0; vertexNum < numVertices; vertexNum++ ) {
			const float vertexOffset  = VectorLengthFast( vertexPositions[vertexNum] );
			minOffsetForFrame = wsw::min( vertexOffset, minOffsetForFrame );
			maxOffsetForFrame = wsw::max( vertexOffset, maxOffsetForFrame );
		}
		const float offsetInterval = wsw::max( 1e-3f, maxOffsetForFrame - minOffsetForFrame );
		const float rcpOffsetInterval = Q_Rcp( offsetInterval );

		for( unsigned vertexNum = 0; vertexNum < numVertices; vertexNum++ ) {
			const float *const vertex = vertexPositions[vertexNum];
			const float vertexOffset  = VectorLengthFast( vertex );

			frameFireVertexMask[vertexNum] = ( vertexOffset - minOffsetForFrame ) * rcpOffsetInterval; // Values between 1 and 0 where 1 has the highest offset

			const float simplexNoise       = calcSimplexNoise3D( vertex[0], vertex[1], vertex[2] - scrolledDistance );
			const float fadeFrac           = ( frameFrac - fadeStartAtFrac ) / ( 1.0f - fadeStartAtFrac );
			const float zFade              = 0.5f * ( vertex[2] + 1.0f ) * zFadeInfluence;
			frameFadeVertexMask[vertexNum] = fadeFrac - simplexNoise * ( 1.0f - zFadeInfluence ) - zFade + fadeRange;
		}

		SimulatedHullsSystem::MaskedShadingLayer fireMaskedShadingLayer;
		fireMaskedShadingLayer.vertexMaskValues = frameFireVertexMask;
		fireMaskedShadingLayer.colors           = kFireColors;

		fireMaskedShadingLayer.colorRanges[0] = fireFrac * fireFrac;
		fireMaskedShadingLayer.colorRanges[1] = fireFrac;
		fireMaskedShadingLayer.colorRanges[2] = std::sqrt( fireFrac );
		fireMaskedShadingLayer.blendMode      = SimulatedHullsSystem::BlendMode::AlphaBlend;
		fireMaskedShadingLayer.alphaMode      = SimulatedHullsSystem::AlphaMode::Override;

		// 0.1f to 0.2f produced a neat outline along the hull
		SimulatedHullsSystem::DotShadingLayer highlightDotShadingLayer;
		highlightDotShadingLayer.colors         = kHighlightColors;
		highlightDotShadingLayer.colorRanges[0] = 0.4f;
		highlightDotShadingLayer.colorRanges[1] = 0.48f;
		highlightDotShadingLayer.blendMode      = SimulatedHullsSystem::BlendMode::Add;
		highlightDotShadingLayer.alphaMode      = SimulatedHullsSystem::AlphaMode::Add;

		SimulatedHullsSystem::MaskedShadingLayer fadeMaskedShadingLayer;
		fadeMaskedShadingLayer.vertexMaskValues = frameFadeVertexMask;
		fadeMaskedShadingLayer.colors           = kFadeColors;
		fadeMaskedShadingLayer.colorRanges[0]   = 0.0f;
		fadeMaskedShadingLayer.colorRanges[1]   = fadeRange;
		fadeMaskedShadingLayer.blendMode        = SimulatedHullsSystem::BlendMode::Add;
		fadeMaskedShadingLayer.alphaMode        = SimulatedHullsSystem::AlphaMode::Override;

		static_assert( numShadingLayers == 3 );
		cagedMesh->shadingLayers[frameNum * numShadingLayers + 0] = fireMaskedShadingLayer;
		cagedMesh->shadingLayers[frameNum * numShadingLayers + 1] = highlightDotShadingLayer;
		cagedMesh->shadingLayers[frameNum * numShadingLayers + 2] = fadeMaskedShadingLayer;

//		delete[] mesh.vertexPositions.data();
//		delete[] mesh.triIndices.data();
//        delete[] mesh.UVCoords;
	}
}

void SimulatedHullsSystem::clear() {
	for( FireHull *hull = m_fireHullsHead, *nextHull; hull; hull = nextHull ) { nextHull = hull->next;
		unlinkAndFreeFireHull( hull );
	}
	for( FireClusterHull *hull = m_fireClusterHullsHead, *next; hull; hull = next ) { next = hull->next;
		unlinkAndFreeFireClusterHull( hull );
	}
	for( BlastHull *hull = m_blastHullsHead, *nextHull; hull; hull = nextHull ) { nextHull = hull->next;
		unlinkAndFreeBlastHull( hull );
	}
	for( SmokeHull *hull = m_smokeHullsHead, *nextHull; hull; hull = nextHull ) { nextHull = hull->next;
		unlinkAndFreeSmokeHull( hull );
	}
	for( WaveHull *hull = m_waveHullsHead, *nextHull; hull; hull = nextHull ) { nextHull = hull->next;
		unlinkAndFreeWaveHull( hull );
	}
}

void SimulatedHullsSystem::unlinkAndFreeStaticCageHull( KeyframedHull *hull ) {
    unsigned cageKey = hull->sharedCageCagedMeshes[0]->loadedCageKey;
	cgNotice() << "cage key is:" << cageKey;
	StaticCage *cage = std::addressof( m_loadedStaticCages[cageKey] );
	cgNotice() << "unlinking:" << cage->identifier;

    wsw::unlink( hull, &cage->head );
    hull->~KeyframedHull();
    cage->allocator->free( hull );
}

void SimulatedHullsSystem::unlinkAndFreeDynamicCageHull( DynamicCageHull *hull ) {
    unsigned cageKey = hull->sharedCageCagedMeshes[0]->loadedCageKey;
    cgNotice() << "cage key is:" << cageKey;
    DynamicCage *cage = std::addressof( m_loadedDynamicCages[cageKey] );
    cgNotice() << "unlinking:" << cage->identifier;

    wsw::unlink( hull, &cage->head );
    hull->~DynamicCageHull();
    cage->allocator->free( hull );
}

void SimulatedHullsSystem::unlinkAndFreeSmokeHull( SmokeHull *hull ) {
	wsw::unlink( hull, &m_smokeHullsHead );
	m_freeShapeLists.push_back( hull->shapeList );
	hull->~SmokeHull();
	m_smokeHullsAllocator.free( hull );
}

void SimulatedHullsSystem::unlinkAndFreeWaveHull( WaveHull *hull ) {
	wsw::unlink( hull, &m_waveHullsHead );
	m_freeShapeLists.push_back( hull->shapeList );
	hull->~WaveHull();
	m_waveHullsAllocator.free( hull );
}

void SimulatedHullsSystem::unlinkAndFreeFireHull( FireHull *hull ) {
	wsw::unlink( hull, &m_fireHullsHead );
	hull->~FireHull();
	m_fireHullsAllocator.free( hull );
}

void SimulatedHullsSystem::unlinkAndFreeFireClusterHull( FireClusterHull *hull ) {
	wsw::unlink( hull, &m_fireClusterHullsHead );
	hull->~FireClusterHull();
	m_fireClusterHullsAllocator.free( hull );
}

void SimulatedHullsSystem::unlinkAndFreeBlastHull( BlastHull *hull ) {
	wsw::unlink( hull, &m_blastHullsHead );
	hull->~BlastHull();
	m_blastHullsAllocator.free( hull );
}

auto SimulatedHullsSystem::allocStaticCageHull( StaticCage *cage, int64_t currTime, unsigned lifetime ) -> KeyframedHull * {
    unsigned numVertices = cage->cageGeometry.vertexPositions.size();
    wsw::HeapBasedFreelistAllocator *allocator = cage->allocator;
    KeyframedHull **head = &cage->head;

    void *mem = allocator->allocOrNull();
    CMShapeList *hullShapeList = nullptr;
    if( mem ) [[likely]] {
        hullShapeList = m_freeShapeLists.back();
        m_freeShapeLists.pop_back();
    } else {
        KeyframedHull *oldestHull = nullptr;
        int64_t oldestSpawnTime = std::numeric_limits<int64_t>::max();
        for( KeyframedHull *hull = *head; hull; hull = hull->next ) {
            if( oldestSpawnTime > hull->spawnTime ) {
                oldestSpawnTime = hull->spawnTime;
                oldestHull = hull;
            }
        }
        assert( oldestHull );
        wsw::unlink( oldestHull, head );
        oldestHull->~KeyframedHull();
        mem = oldestHull;
    }

    auto *const hull = new( mem )KeyframedHull( numVertices, ( void* )mem );
    hull->spawnTime  = currTime;
    hull->lifetime   = lifetime;

    wsw::link( hull, head );
    return hull;
}

auto SimulatedHullsSystem::allocDynamicCageHull( DynamicCage *cage, int64_t currTime, unsigned lifetime ) -> DynamicCageHull * {
    unsigned numVertices = cage->cageInitialGeometry.vertexPositions.size();
    wsw::HeapBasedFreelistAllocator *allocator = cage->allocator;
    DynamicCageHull **head = &cage->head;

    void *mem = allocator->allocOrNull();
    CMShapeList *hullShapeList = nullptr;
    if( !mem ) [[unlikely]] {
        DynamicCageHull *oldestHull = nullptr;
        int64_t oldestSpawnTime = std::numeric_limits<int64_t>::max();
        for( DynamicCageHull *hull = *head; hull; hull = hull->next ) {
            if( oldestSpawnTime > hull->spawnTime ) {
                oldestSpawnTime = hull->spawnTime;
                oldestHull = hull;
            }
        }
        assert( oldestHull );
        wsw::unlink( oldestHull, head );
        oldestHull->~DynamicCageHull();
        mem = oldestHull;
    }

    auto *const hull = new( mem )DynamicCageHull( numVertices, ( void* )mem );
    hull->spawnTime  = currTime;
    hull->lifetime   = lifetime;

    wsw::link( hull, head );
    return hull;
}

auto SimulatedHullsSystem::allocFireHull( int64_t currTime, unsigned lifetime ) -> FireHull * {
	return allocHull<FireHull, false>( &m_fireHullsHead, &m_fireHullsAllocator, currTime, lifetime );
}

auto SimulatedHullsSystem::allocFireClusterHull( int64_t currTime, unsigned lifetime ) -> FireClusterHull * {
	return allocHull<FireClusterHull, false>( &m_fireClusterHullsHead, &m_fireClusterHullsAllocator, currTime, lifetime );
}

auto SimulatedHullsSystem::allocBlastHull( int64_t currTime, unsigned int lifetime ) -> BlastHull * {
	return allocHull<BlastHull, false>( &m_blastHullsHead, &m_blastHullsAllocator, currTime, lifetime );
}

auto SimulatedHullsSystem::allocSmokeHull( int64_t currTime, unsigned lifetime ) -> SmokeHull * {
	return allocHull<SmokeHull, true>( &m_smokeHullsHead, &m_smokeHullsAllocator, currTime, lifetime );
}

auto SimulatedHullsSystem::allocWaveHull( int64_t currTime, unsigned lifetime ) -> WaveHull * {
	return allocHull<WaveHull, true>( &m_waveHullsHead, &m_waveHullsAllocator, currTime, lifetime );
}

// TODO: Turn on "Favor small code" for this template

template <typename Hull, bool HasShapeList>
auto SimulatedHullsSystem::allocHull( Hull **head, wsw::FreelistAllocator *allocator,
									  int64_t currTime, unsigned lifetime ) -> Hull * {
	void *mem = allocator->allocOrNull();
	CMShapeList *hullShapeList = nullptr;
	if( mem ) [[likely]] {
		if constexpr( HasShapeList ) {
			hullShapeList = m_freeShapeLists.back();
			m_freeShapeLists.pop_back();
		}
	} else {
		Hull *oldestHull = nullptr;
		int64_t oldestSpawnTime = std::numeric_limits<int64_t>::max();
		for( Hull *hull = *head; hull; hull = hull->next ) {
			if( oldestSpawnTime > hull->spawnTime ) {
				oldestSpawnTime = hull->spawnTime;
				oldestHull = hull;
			}
		}
		assert( oldestHull );
		wsw::unlink( oldestHull, head );
		if constexpr( HasShapeList ) {
			hullShapeList = oldestHull->shapeList;
		}
		oldestHull->~Hull();
		mem = oldestHull;
	}

	auto *const hull = new( mem )Hull;
	hull->spawnTime  = currTime;
	hull->lifetime   = lifetime;
	if constexpr( HasShapeList ) {
		hull->shapeList = hullShapeList;
	}

	wsw::link( hull, head );
	return hull;
}

void SimulatedHullsSystem::setupHullVertices( BaseRegularSimulatedHull *hull, const float *origin,
											  const float *color, float speed, float speedSpead,
											  const AppearanceRules &appearanceRules,
											  const float *spikeSpeedMask, float maxSpikeSpeed ) {
	const byte_vec4_t initialColor {
		(uint8_t)( color[0] * 255 ),
		(uint8_t)( color[1] * 255 ),
		(uint8_t)( color[2] * 255 ),
		(uint8_t)( color[3] * 255 )
	};

	const float minSpeed = wsw::max( 0.0f, speed - 0.5f * speedSpead );
	const float maxSpeed = speed + 0.5f * speedSpead;
	wsw::RandomGenerator *__restrict rng = &m_rng;

	const float originX = origin[0], originY = origin[1], originZ = origin[2];
	const auto [verticesSpan, indicesSpan, _] = ::basicHullsHolder.getIcosphereForLevel( hull->subdivLevel );
	const auto *__restrict vertices = verticesSpan.data();

	std::memset( hull->vertexForceVelocities, 0, verticesSpan.size() * sizeof( hull->vertexForceVelocities[0] ) );

	vec4_t *const __restrict positions       = hull->vertexPositions[0];
	vec4_t *const __restrict vertexNormals   = hull->vertexNormals;
	vec3_t *const __restrict burstVelocities = hull->vertexBurstVelocities;
	vec4_t *const __restrict altPositions    = hull->vertexPositions[1];
	byte_vec4_t *const __restrict colors     = hull->vertexColors;

	size_t i = 0;
	do {
		// Vertex positions are absolute to simplify simulation
		Vector4Set( positions[i], originX, originY, originZ, 1.0f );
		float vertexSpeed = rng->nextFloat( minSpeed, maxSpeed );
		if( spikeSpeedMask ) {
			vertexSpeed += maxSpikeSpeed * spikeSpeedMask[i];
		}
		// Unit vertices define directions
		VectorScale( vertices[i], vertexSpeed, burstVelocities[i] );
		// Set the 4th component to 1.0 for alternating positions once as well
		altPositions[i][3] = 1.0f;
		Vector4Copy( initialColor, colors[i] );
		if( vertexNormals ) {
			// Supply vertex directions as normals for the first simulation frame
			VectorCopy( vertices[i], vertexNormals[i] );
			vertexNormals[i][3] = 0.0f;
		}
	} while( ++i < verticesSpan.size() );

	hull->minZLastFrame = originZ - 1.0f;
	hull->maxZLastFrame = originZ + 1.0f;

	VectorCopy( origin, hull->origin );

	hull->appearanceRules = appearanceRules;
}

void SimulatedHullsSystem::setupHullVertices( BaseConcentricSimulatedHull *hull, const float *origin,
											  float scale, std::span<const HullLayerParams> layerParams,
											  const AppearanceRules &appearanceRules ) {
	assert( layerParams.size() == hull->numLayers );

	const float originX = origin[0], originY = origin[1], originZ = origin[2];
	const auto [verticesSpan, indicesSpan, neighboursSpan] = ::basicHullsHolder.getIcosphereForLevel( hull->subdivLevel );
	const vec4_t *__restrict vertices = verticesSpan.data();

	// Calculate move limits in each direction

	float maxVertexSpeed = layerParams[0].speed + layerParams[0].maxSpeedSpike + layerParams[0].biasAlongChosenDir ;
	for( unsigned i = 1; i < layerParams.size(); ++i ) {
		const auto &params = layerParams[i];
		maxVertexSpeed = wsw::max( maxVertexSpeed, params.speed + params.maxSpeedSpike + params.biasAlongChosenDir );
	}

	// To prevent noticeable z-fighting in case if hulls of different layers start matching (e.g due to bias)
	constexpr float maxSmallRandomOffset = 1.5f;

	maxVertexSpeed += maxSmallRandomOffset;
	// Scale by the fine-tuning scale multiplier
	maxVertexSpeed *= 1.5f * scale;

	wsw::RandomGenerator *const __restrict rng = &m_rng;

	const float radius = 0.5f * maxVertexSpeed * ( 1e-3f * (float)hull->lifetime );
	const vec3_t growthMins { originX - radius, originY - radius, originZ - radius };
	const vec3_t growthMaxs { originX + radius, originY + radius, originZ + radius };

	// TODO: Add a fused call
	CM_BuildShapeList( cl.cms, m_tmpShapeList, growthMins, growthMaxs, MASK_SOLID );
	CM_ClipShapeList( cl.cms, m_tmpShapeList, m_tmpShapeList, growthMins, growthMaxs );

	if( CM_GetNumShapesInShapeList( m_tmpShapeList ) == 0 ) {
		// Limits at each direction just match the given radius in this case
		std::fill( hull->limitsAtDirections, hull->limitsAtDirections + verticesSpan.size(), radius );
	} else {
		trace_t trace;
		for( size_t i = 0; i < verticesSpan.size(); ++i ) {
			// Vertices of the unit hull define directions
			const float *dir = verticesSpan[i];

			vec3_t limitPoint;
			VectorMA( origin, radius, dir, limitPoint );

			CM_ClipToShapeList( cl.cms, m_tmpShapeList, &trace, origin, limitPoint, vec3_origin, vec3_origin, MASK_SOLID );
			hull->limitsAtDirections[i] = trace.fraction * radius;
		}
	}

	auto *const __restrict spikeSpeedBoost = (float *)alloca( sizeof( float ) * verticesSpan.size() );

	const float *const __restrict globalBiasDir = verticesSpan[rng->nextBounded( verticesSpan.size() ) ];
	assert( std::fabs( VectorLengthFast( globalBiasDir ) - 1.0f ) < 0.001f );

	// Compute a hull-global bias for every vertex once
	auto *const __restrict globalVertexDotBias = (float *)alloca( sizeof( float ) * verticesSpan.size() );
	for( unsigned i = 0; i < verticesSpan.size(); ++i ) {
		const float *__restrict vertex = verticesSpan[i];
		// We have decided that permitting a negative bias yields better results.
		globalVertexDotBias[i] = DotProduct( vertex, globalBiasDir );
	}

	// Setup layers data
	assert( hull->numLayers >= 1 && hull->numLayers <= kMaxHullLayers );
	for( unsigned layerNum = 0; layerNum < hull->numLayers; ++layerNum ) {
		BaseConcentricSimulatedHull::Layer *layer   = &hull->layers[layerNum];
		const HullLayerParams *__restrict params    = &layerParams[layerNum];
		vec4_t *const __restrict positions          = layer->vertexPositions;
		byte_vec4_t *const __restrict colors        = layer->vertexColors;
		vec2_t *const __restrict speedsAndDistances = layer->vertexSpeedsAndDistances;
		const float *const __restrict layerBiasDir  = verticesSpan[rng->nextBounded( verticesSpan.size() )];

		assert( params->speed >= 0.0f && params->minSpeedSpike >= 0.0f && params->biasAlongChosenDir >= 0.0f );
		assert( params->minSpeedSpike < params->maxSpeedSpike );
		assert( std::fabs( VectorLengthFast( layerBiasDir ) - 1.0f ) < 0.001f );

		layer->finalOffset         = params->finalOffset;
		layer->drawOrderDesignator = (float)( hull->numLayers - layerNum );
		layer->colorChangeTimeline = params->colorChangeTimeline;

		std::fill( spikeSpeedBoost, spikeSpeedBoost + verticesSpan.size(), 0.0f );

		const float *const __restrict baseColor  = params->baseInitialColor;
		const float *const __restrict bulgeColor = params->bulgeInitialColor;

		for( size_t i = 0; i < verticesSpan.size(); ++i ) {
			// Position XYZ is computed prior to submission in stateless fashion
			positions[i][3] = 1.0f;

			const float *vertexDir   = vertices[i];
			const float layerDotBias = DotProduct( vertexDir, layerBiasDir );
			const float layerSqrBias = std::copysign( layerDotBias * layerDotBias, layerDotBias );
			const float vertexBias   = wsw::max( layerSqrBias, globalVertexDotBias[i] );

			speedsAndDistances[i][0] = params->speed + vertexBias * params->biasAlongChosenDir;

			if( rng->tryWithChance( params->speedSpikeChance ) ) [[unlikely]] {
				const float boost = rng->nextFloat( params->minSpeedSpike, params->maxSpeedSpike );
				spikeSpeedBoost[i] += boost;
				const auto &indicesOfNeighbours = neighboursSpan[i];
				for( const unsigned neighbourIndex: indicesOfNeighbours ) {
					spikeSpeedBoost[neighbourIndex] += rng->nextFloat( 0.50f, 0.75f ) * boost;
				}
			} else {
				speedsAndDistances[i][0] += rng->nextFloat( 0.0f, maxSmallRandomOffset );
			}

			speedsAndDistances[i][1] = 0.0f;

			const float colorLerpFrac  = vertexBias * vertexBias;
			const float complementFrac = 1.0f - colorLerpFrac;
			colors[i][0] = (uint8_t)( 255.0f * ( baseColor[0] * colorLerpFrac + bulgeColor[0] * complementFrac ) );
			colors[i][1] = (uint8_t)( 255.0f * ( baseColor[1] * colorLerpFrac + bulgeColor[1] * complementFrac ) );
			colors[i][2] = (uint8_t)( 255.0f * ( baseColor[2] * colorLerpFrac + bulgeColor[2] * complementFrac ) );
			colors[i][3] = (uint8_t)( 255.0f * ( baseColor[3] * colorLerpFrac + bulgeColor[3] * complementFrac ) );
		}

		for( size_t i = 0; i < verticesSpan.size(); ++i ) {
			speedsAndDistances[i][0] += wsw::min( spikeSpeedBoost[i], maxVertexSpeed );
			// Scale by the fine-tuning scale multiplier
			speedsAndDistances[i][0] *= scale;
		}
	}

	VectorCopy( origin, hull->origin );
	hull->vertexMoveDirections = vertices;

	hull->appearanceRules = appearanceRules;
}

BoolConfigVar v_showVectorsToLim( wsw::StringView("showVectorsToLim"), { .byDefault = false, .flags = CVAR_ARCHIVE } );
BoolConfigVar v_showTris( wsw::StringView("showTris"), { .byDefault = false, .flags = CVAR_ARCHIVE } );
IntConfigVar v_numVecs( wsw::StringView("numVecs"), { .byDefault = 20, .flags = CVAR_ARCHIVE } );
UnsignedConfigVar v_trisLifetime( wsw::StringView( "trisLifetime"), { .byDefault = 500, .flags = CVAR_ARCHIVE } );
IntConfigVar v_triIdx( wsw::StringView( "triIdx"), { .byDefault = -1, .flags = CVAR_ARCHIVE } );

void SimulatedHullsSystem::setupHullVertices( BaseKeyframedHull *hull, const float *origin,
											  const float scale, const float *dir, const float rotation,
                                              SimulatedHullsSystem::StaticCage *cage,
                                              PolyEffectsSystem *effectsSystem,
                                              const AppearanceRules &appearanceRules ) {
	const float originX = origin[0], originY = origin[1], originZ = origin[2];
	BoundsBuilder cageBoundsBuilder;

	Geometry *cageGeometry = &cage->cageGeometry;
	vec3_t *vertexPositions = cageGeometry->vertexPositions.data();
	unsigned numVerts = cageGeometry->vertexPositions.size();

	cgNotice() << "set up hull vertices";
	// Calculate move limits in each direction

	const float radius = scale * cage->boundingRadius;
	const vec3_t growthMins { originX - radius, originY - radius, originZ - radius };
	const vec3_t growthMaxs { originX + radius, originY + radius, originZ + radius };

	// TODO: Add a fused call
	CM_BuildShapeList( cl.cms, m_tmpShapeList, growthMins, growthMaxs, MASK_SOLID );
	CM_ClipShapeList( cl.cms, m_tmpShapeList, m_tmpShapeList, growthMins, growthMaxs );

    mat3_t transformMatrixForDir;
    Matrix3_ForRotationOfDirs( &axis_identity[AXIS_UP], dir, transformMatrixForDir );

    mat3_t transformMatrix;
    Matrix3_Rotate( transformMatrixForDir, rotation, &axis_identity[AXIS_UP], transformMatrix );

    vec3_t color { 0.99f, 0.4f, 0.1f };

    vec3_t *moveDirections    = hull->vertexMoveDirections;
    float *limitsAtDirections = hull->limitsAtDirections;

	if( CM_GetNumShapesInShapeList( m_tmpShapeList ) == 0 ) {
		for( size_t i = 0; i < numVerts; i++ ) {
			Matrix3_TransformVector( transformMatrix, vertexPositions[i], moveDirections[i] );

			vec3_t limitPoint;
			VectorMA( origin, scale, moveDirections[i], limitPoint );
			cageBoundsBuilder.addPoint( limitPoint );
		}
        cageBoundsBuilder.storeTo( hull->cageMins, hull->cageMaxs );

		// Limits at each direction just match the given radius in this case
		std::fill( limitsAtDirections, limitsAtDirections + numVerts, radius );
		cgNotice() << "no shapes";

	} else {
		trace_t trace;

        cgNotice() << "identifier size" << cage->identifier.size();
        cgNotice() << "vertices size" << cage->cageGeometry.vertexPositions.size();

        for( size_t i = 0; i < numVerts; i++ ) {
            Matrix3_TransformVector( transformMatrix, vertexPositions[i], moveDirections[i] );

            vec3_t limitPoint;
            VectorMA( origin, scale, moveDirections[i], limitPoint );

            CM_ClipToShapeList( cl.cms, m_tmpShapeList, &trace, origin, limitPoint, vec3_origin, vec3_origin,
                                MASK_SOLID );

			const float moveDirLength = VectorLengthFast( moveDirections[i] );
            limitsAtDirections[i] = trace.fraction * moveDirLength;

			VectorMA( origin, scale * trace.fraction, moveDirections[i], limitPoint );
			cageBoundsBuilder.addPoint( limitPoint );

			if( trace.fraction < 0.99f ){
				VectorSet( color, 0.99f, 0.4f, 0.1f );

			} else {
				VectorSet( color, 0.4f, 0.99f, 0.1f );
			}

            if ( v_showVectorsToLim.get()) {
                effectsSystem->spawnTransientBeamEffect( origin, limitPoint, {
                        .material          = cgs.media.shaderLaser,
                        .beamColorLifespan = {
                                .initial  = {color[0], color[1], color[2]},
                                .fadedIn  = {color[0], color[1], color[2]},
                                .fadedOut = {color[0], color[1], color[2]},
                        },
                        .width             = 8.0f,
                        .timeout           = v_trisLifetime.get(),
                } );
            }

        }

        vec3_t colorB { 0.1f, 0.4f, 0.99f };

		tri *tris = cageGeometry->triIndices.data();
		unsigned numTris = cageGeometry->triIndices.size();

		for( int triNum = 0; triNum < numTris; triNum++ ){
			for( int idxNum = 0; idxNum < 3; idxNum++ ){
				unsigned firstIdx  = idxNum;
				unsigned secondIdx = ( idxNum + 1 ) % 3;

				unsigned firstVertex  = tris[triNum][firstIdx];
				unsigned secondVertex = tris[triNum][secondIdx];

				vec3_t firstPosition;
				vec3_t secondPosition;

				const float firstLengthRcp = Q_RSqrt( VectorLengthSquared( moveDirections[firstVertex] ) );
				const float secondLengthRcp = Q_RSqrt( VectorLengthSquared( moveDirections[secondVertex] ) );

				VectorMA( origin, scale * limitsAtDirections[firstVertex] * firstLengthRcp, moveDirections[firstVertex], firstPosition );
				VectorMA( origin, scale * limitsAtDirections[secondVertex] * secondLengthRcp, moveDirections[secondVertex], secondPosition );

				if( v_triIdx.get() < 0 || triNum == v_triIdx.get() ){
					effectsSystem->spawnTransientBeamEffect( firstPosition, secondPosition, {
							.material          = cgs.media.shaderLaser,
							.beamColorLifespan = {
									.initial  = {colorB[0], colorB[1], colorB[2]},
									.fadedIn  = {colorB[0], colorB[1], colorB[2]},
									.fadedOut = {colorB[0], colorB[1], colorB[2]},
							},
							.width             = 8.0f,
							.timeout           = v_trisLifetime.get(),
					} );
				}
			}
		}

		cageBoundsBuilder.storeTo( hull->cageMins, hull->cageMaxs );
	}

	effectsSystem->spawnTransientBeamEffect( origin, hull->cageMins, {
			.material          = cgs.media.shaderLaser,
			.beamColorLifespan = {
					.initial  = {color[0], color[1], color[2]},
					.fadedIn  = {color[0], color[1], color[2]},
					.fadedOut = {color[0], color[1], color[2]},
			},
			.width             = 8.0f,
			.timeout           = v_trisLifetime.get(),
	} );
	effectsSystem->spawnTransientBeamEffect( origin, hull->cageMaxs, {
			.material          = cgs.media.shaderLaser,
			.beamColorLifespan = {
					.initial  = {color[0], color[1], color[2]},
					.fadedIn  = {color[0], color[1], color[2]},
					.fadedOut = {color[0], color[1], color[2]},
			},
			.width             = 8.0f,
			.timeout           = v_trisLifetime.get(),
	} );

	VectorCopy( origin, hull->origin );
	hull->scale = scale;
}

void SimulatedHullsSystem::setupHullVertices( BaseDynamicCageHull *hull, const float *origin,
                                              const float scale, const float *dir, const float rotation,
                                              SimulatedHullsSystem::DynamicCage *cage,
                                              PolyEffectsSystem *effectsSystem,
                                              const AppearanceRules &appearanceRules ) {
    const float originX = origin[0], originY = origin[1], originZ = origin[2];
    BoundsBuilder cageBoundsBuilder;

    Geometry *cageInitialGeometry = &cage->cageInitialGeometry;
    vec3_t *vertexPositions = cageInitialGeometry->vertexPositions.data();
    unsigned numVerts = cageInitialGeometry->vertexPositions.size();

    const float maxOffsetFromCage = hull->maxOffsetFromCage; // TODO: find the max offset among dynamic cage meshes for this value

    cgNotice() << "set up hull vertices";
    // Calculate move limits in each direction

    const float radius = scale * cage->initialBoundingRadius + maxOffsetFromCage;

    cgNotice() << "radius when setting up verts: " << radius;

    const vec3_t growthMins { originX - radius, originY - radius, originZ - radius };
    const vec3_t growthMaxs { originX + radius, originY + radius, originZ + radius };

    // TODO: Add a fused call
	CMShapeList *shapeList = m_tmpShapeList;
    CM_BuildShapeList( cl.cms, shapeList, growthMins, growthMaxs, MASK_SOLID );
    CM_ClipShapeList( cl.cms, shapeList, shapeList, growthMins, growthMaxs );

    mat3_t transformMatrixForDir;
    Matrix3_ForRotationOfDirs( &axis_identity[AXIS_UP], dir, transformMatrixForDir );

    // MULTIPLY WITH ROTATION ANGLE
    mat3_t transformMatrix;
    Matrix3_Rotate( transformMatrixForDir, rotation, &axis_identity[AXIS_UP], transformMatrix );

	Matrix3_Copy( transformMatrix, hull->transformMatrix );

    vec3_t color { 0.99f, 0.4f, 0.1f };

    vec3_t *hullVertexPositions = hull->vertexPositions;
    vec3_t *hullVertexNormals   = hull->vertexNormals;
    float *limitsAtDirections   = hull->limitsAtDirections;

    //const vec3_t maxCageBoundsOffset = { maxOffsetFromCage, maxOffsetFromCage, maxOffsetFromCage };

    if( CM_GetNumShapesInShapeList( shapeList ) == 0 ) {
        for( size_t i = 0; i < numVerts; i++ ) {
            Matrix3_TransformVector(transformMatrix, vertexPositions[i], hullVertexPositions[i] );

            //vec3_t limitPoint;
            VectorMA( origin, scale, hullVertexPositions[i], hullVertexPositions[i] );
            //cageBoundsBuilder.addPoint( limitPoint );
        }

//        cageBoundsBuilder.storeTo( hull->mins, hull->maxs );
//        VectorAdd( hull->maxs, maxCageBoundsOffset, hull->maxs );
//        VectorSubtract( hull->mins, maxCageBoundsOffset, hull->mins );

        cgNotice() << "no shapes";

    } else {
        trace_t trace;

        cgNotice() << "identifier size" << cage->identifier.size();
        cgNotice() << "vertices size" << cage->cageInitialGeometry.vertexPositions.size();

        for( size_t i = 0; i < numVerts; i++ ) {
            Matrix3_TransformVector(transformMatrix, vertexPositions[i], hullVertexPositions[i] );

            vec3_t limitPoint;
            VectorMA(origin, scale, hullVertexPositions[i], limitPoint );

            CM_ClipToShapeList( cl.cms, shapeList, &trace, origin, limitPoint, vec3_origin, vec3_origin,
                                MASK_SOLID );

            const float moveDirLength = VectorLengthFast( hullVertexPositions[i] );
            limitsAtDirections[i] = trace.fraction * moveDirLength;

            VectorMA( origin, scale * trace.fraction, hullVertexPositions[i], limitPoint );
            //cageBoundsBuilder.addPoint( limitPoint );
            VectorCopy( limitPoint, hullVertexPositions[i] );

            if( trace.fraction < 0.99f ){
                VectorSet( color, 0.99f, 0.4f, 0.1f );

            } else {
                VectorSet( color, 0.4f, 0.99f, 0.1f );
            }

            if ( v_showVectorsToLim.get()) {
                effectsSystem->spawnTransientBeamEffect( origin, limitPoint, {
                        .material          = cgs.media.shaderLaser,
                        .beamColorLifespan = {
                                .initial  = {color[0], color[1], color[2]},
                                .fadedIn  = {color[0], color[1], color[2]},
                                .fadedOut = {color[0], color[1], color[2]},
                        },
                        .width             = 8.0f,
                        .timeout           = v_trisLifetime.get(),
                } );
            }

        }

//        cageBoundsBuilder.storeTo( hull->mins, hull->maxs );
//        VectorAdd( hull->maxs, maxCageBoundsOffset, hull->maxs );
//        VectorSubtract( hull->mins, maxCageBoundsOffset, hull->mins );

    }

    vec3_t colorB { 0.1f, 0.4f, 0.99f };
    vec3_t colorC { 0.99f, 0.2f, 0.1f };

    tri *tris = cageInitialGeometry->triIndices.data();
    unsigned numTris = cageInitialGeometry->triIndices.size();

	calculateNormals( cageInitialGeometry->triIndices, numVerts, hull->vertexPositions, hullVertexNormals );

    if( CM_GetNumShapesInShapeList( shapeList ) == 0 ) {
        // Limits at each direction just match the maximum offset from the cage in this cage
        std::fill( limitsAtDirections, limitsAtDirections + numVerts, maxOffsetFromCage );
		for( size_t i = 0; i < numVerts; i++ ) {
			vec3_t offsetPos;
			VectorMA( hullVertexPositions[i], maxOffsetFromCage, hullVertexNormals[i], offsetPos );
			cageBoundsBuilder.addPoint( offsetPos );
		}
    } else {
        trace_t trace;
        for( size_t i = 0; i < numVerts; i++ ) {
            vec3_t limitPoint;
            VectorMA( hullVertexPositions[i], maxOffsetFromCage, hullVertexNormals[i], limitPoint );
            CM_ClipToShapeList(cl.cms, shapeList, &trace, hullVertexPositions[i],
                               limitPoint, vec3_origin, vec3_origin, MASK_SOLID);
            limitsAtDirections[i] = maxOffsetFromCage * trace.fraction;

			vec3_t offsetPos;
			VectorMA( hullVertexPositions[i], limitsAtDirections[i], hullVertexNormals[i], offsetPos );
			cageBoundsBuilder.addPoint( offsetPos );
        }
    }
	cageBoundsBuilder.storeTo( hull->mins, hull->maxs );

    for( int triNum = 0; triNum < numTris; triNum++ ){
        for( int idxNum = 0; idxNum < 3; idxNum++ ){
            unsigned firstIdx  = idxNum;
            unsigned secondIdx = ( idxNum + 1 ) % 3;

            unsigned firstVertex  = tris[triNum][firstIdx];
            unsigned secondVertex = tris[triNum][secondIdx];

            vec3_t firstPosition;
            vec3_t secondPosition;

            VectorCopy( hullVertexPositions[firstVertex], firstPosition );
            VectorCopy( hullVertexPositions[secondVertex], secondPosition );

            vec3_t firstOffsetPos;
            vec3_t secondOffsetPos;

            VectorMA( hullVertexPositions[firstVertex], limitsAtDirections[firstVertex], hullVertexNormals[firstVertex], firstOffsetPos );
            VectorMA( hullVertexPositions[secondVertex], limitsAtDirections[secondVertex], hullVertexNormals[secondVertex], secondOffsetPos );

            if( v_triIdx.get() < 0 || triNum == v_triIdx.get() ){
                effectsSystem->spawnTransientBeamEffect( firstPosition, secondPosition, {
                        .material          = cgs.media.shaderLaser,
                        .beamColorLifespan = {
                                .initial  = {colorB[0], colorB[1], colorB[2]},
                                .fadedIn  = {colorB[0], colorB[1], colorB[2]},
                                .fadedOut = {colorB[0], colorB[1], colorB[2]},
                        },
                        .width             = 8.0f,
                        .timeout           = v_trisLifetime.get(),
                } );

                effectsSystem->spawnTransientBeamEffect( firstOffsetPos, secondOffsetPos, {
                        .material          = cgs.media.shaderLaser,
                        .beamColorLifespan = {
                                .initial  = {colorC[0], colorC[1], colorC[2]},
                                .fadedIn  = {colorC[0], colorC[1], colorC[2]},
                                .fadedOut = {colorC[0], colorC[1], colorC[2]},
                        },
                        .width             = 8.0f,
                        .timeout           = v_trisLifetime.get(),
                } );
            }
        }
    }

    VectorCopy( origin, hull->origin );
    hull->scale = scale;
}

void SimulatedHullsSystem::addHull( AppearanceRules *appearanceRules, StaticKeyframedHullParams &hullParams ) {

    SimulatedHullsSystem::StaticCagedMesh *cagedMesh = hullParams.sharedCageCagedMeshes;
    SimulatedHullsSystem::StaticCage *cage = std::addressof( SimulatedHullsSystem::m_loadedStaticCages[cagedMesh->loadedCageKey] );
    cgNotice() << "identifier" << cage->identifier;

    if( auto *const hull = SimulatedHullsSystem::allocStaticCageHull( cage, m_lastTime, hullParams.timeout ) ){
        const vec3_t hullOrigin {
                hullParams.origin[0] + hullParams.offset[0],
                hullParams.origin[1] + hullParams.offset[1],
                hullParams.origin[2] + hullParams.offset[2]
        };

        const unsigned numMeshes = hullParams.numCagedMeshes;
        hull->numSharedCageCagedMeshes = numMeshes;

        for( unsigned meshNum = 0; meshNum < numMeshes; meshNum++ ) {
            hull->sharedCageCagedMeshes[meshNum] = &hullParams.sharedCageCagedMeshes[meshNum];
			hull->appearanceRules[meshNum]       = appearanceRules[meshNum];
        }

        SimulatedHullsSystem::setupHullVertices( hull, hullOrigin, hullParams.scale, hullParams.dir,
                                                 hullParams.rotation, cage, &cg.polyEffectsSystem );

    }

}

void SimulatedHullsSystem::addHull( AppearanceRules *appearanceRules, DynamicCageHullParams &hullParams ) {

    SimulatedHullsSystem::DynamicCagedMesh *cagedMeshes = hullParams.sharedCageCagedMeshes;
    SimulatedHullsSystem::DynamicCage *cage = std::addressof( m_loadedDynamicCages[cagedMeshes->loadedCageKey] );

    cgNotice() << "identifier" << cage->identifier;
    //cgNotice() << "size" << m_loadedDynamicCages.size();

    //cgNotice() << "amk";

    if( auto *const hull = SimulatedHullsSystem::allocDynamicCageHull( cage, m_lastTime, hullParams.timeout ) ){
        const vec3_t hullOrigin {
                hullParams.origin[0] + hullParams.offset[0],
                hullParams.origin[1] + hullParams.offset[1],
                hullParams.origin[2] + hullParams.offset[2]
        };

        const unsigned numMeshes = hullParams.numCagedMeshes;
        hull->numSharedCageCagedMeshes = numMeshes;

        for( unsigned meshNum = 0; meshNum < numMeshes; meshNum++ ) {
            hull->sharedCageCagedMeshes[meshNum] = &cagedMeshes[meshNum];
            hull->appearanceRules[meshNum]       = appearanceRules[meshNum];    
            hull->maxOffsetFromCage              = hullParams.scale * cage->initialBoundingRadius * 1.2f;
            //hull->maxOffsetFromCage              = wsw::max( hull->maxOffsetFromCage, cagedMeshes[meshNum].maxOffsetFromCage); // TODO: use this instead when mesh registration works
        }

		hull->maxOffsetSpeed = hullParams.maxOffsetSpeed;
        SimulatedHullsSystem::setupHullVertices( hull, hullOrigin, hullParams.scale, hullParams.dir,
                                                 hullParams.rotation, cage, &cg.polyEffectsSystem );

    }

}

void SimulatedHullsSystem::calcSmokeBulgeSpeedMask( float *__restrict vertexSpeedMask, unsigned subdivLevel, unsigned maxSpikes ) {
	assert( subdivLevel < 4 );
	assert( maxSpikes && maxSpikes < 10 );

	const IcosphereData &icosphereData    = ::basicHullsHolder.getIcosphereForLevel( subdivLevel );
	const vec4_t *__restrict hullVertices = icosphereData.vertices.data();
	const unsigned numHullVertices        = icosphereData.vertices.size();

	vec3_t spikeDirs[10];
	unsigned numChosenSpikes = 0;
	for( unsigned attemptNum = 0; attemptNum < 4 * maxSpikes; ++attemptNum ) {
		const unsigned vertexNum    = m_rng.nextBounded( std::size( kPredefinedDirs ) );
		const float *__restrict dir = kPredefinedDirs[vertexNum];
		if( dir[2] < -0.1f || dir[2] > 0.7f ) {
			continue;
		}

		bool foundASimilarDir = false;
		for( unsigned spikeNum = 0; spikeNum < numChosenSpikes; ++spikeNum ) {
			if( DotProduct( spikeDirs[spikeNum], dir ) > 0.7f ) {
				foundASimilarDir = true;
				break;
			}
		}
		if( !foundASimilarDir ) {
			VectorCopy( dir, spikeDirs[numChosenSpikes] );
			numChosenSpikes++;
			if( numChosenSpikes == maxSpikes ) {
				break;
			}
		}
	}

	std::fill( vertexSpeedMask, vertexSpeedMask + numHullVertices, 0.0f );

	unsigned vertexNum = 0;
	do {
		const float *__restrict vertexDir = hullVertices[vertexNum];
		float spikeStrength = 0.0f;
		unsigned spikeNum   = 0;
		do {
			// Must be non-negative to contribute to the spike strength
			const float dot = wsw::max( 0.0f, DotProduct( vertexDir, spikeDirs[spikeNum] ) );
			spikeStrength += dot * dot * dot;
		} while( ++spikeNum < numChosenSpikes );
		spikeStrength = wsw::min( 1.0f, spikeStrength );
		spikeStrength *= ( 1.0f - std::fabs( vertexDir[2] ) );
		vertexSpeedMask[vertexNum] = spikeStrength;
	} while( ++vertexNum < numHullVertices );
}

void SimulatedHullsSystem::calcSmokeSpikeSpeedMask( float *__restrict vertexSpeedMask, unsigned subdivLevel, unsigned maxSpikes ) {
	assert( subdivLevel < 4 );
	assert( maxSpikes && maxSpikes < 10 );

	const IcosphereData &icosphereData = ::basicHullsHolder.getIcosphereForLevel( subdivLevel );
	const unsigned numHullVertices     = icosphereData.vertices.size();

	unsigned spikeVertexNums[10];
	unsigned numChosenSpikes = 0;
	for( unsigned numAttempts = 0; numAttempts < 4 * maxSpikes; numAttempts++ ) {
		const unsigned vertexNum    = m_rng.nextBounded( numHullVertices );
		const float *__restrict dir = icosphereData.vertices[vertexNum];
		if( dir[2] < -0.1f || dir[2] > 0.7f ) {
			continue;
		}

		bool foundASimilarDir = false;
		for( unsigned spikeNum = 0; spikeNum < numChosenSpikes; ++spikeNum ) {
			if( DotProduct( icosphereData.vertices[spikeVertexNums[spikeNum]], dir ) > 0.7f ) {
				foundASimilarDir = true;
				break;
			}
		}
		if( !foundASimilarDir ) {
			spikeVertexNums[numChosenSpikes++] = vertexNum;
			if( numChosenSpikes == maxSpikes ) {
				break;
			}
		}
	}

	std::fill( vertexSpeedMask, vertexSpeedMask + numHullVertices, 0.0f );

	const auto *__restrict hullVertexNeighbours = icosphereData.vertexNeighbours.data();

	unsigned spikeNum = 0;
	do {
		const unsigned spikeVertexNum = spikeVertexNums[spikeNum];
		vertexSpeedMask[spikeVertexNum] += 1.0f;
		unsigned neighbourIndex = 0;
		const auto *neighbours = hullVertexNeighbours[spikeVertexNum];
		do {
			const unsigned neighbourVertexNum = neighbours[neighbourIndex];
			vertexSpeedMask[neighbourVertexNum] += 0.67f;
			const auto *nextNeighbours = hullVertexNeighbours[neighbourVertexNum];
			unsigned nextNeighbourIndex = 0;
			do {
				vertexSpeedMask[nextNeighbours[nextNeighbourIndex]] += 0.37f;
			} while( ++nextNeighbourIndex < 5 );
		} while( ++neighbourIndex < 5 );
	} while( ++spikeNum < numChosenSpikes );

	unsigned vertexNum = 0;
	do {
		vertexSpeedMask[vertexNum] = wsw::min( 1.0f, vertexSpeedMask[vertexNum] );
	} while( ++vertexNum < numHullVertices );
}

auto SimulatedHullsSystem::buildMatchingHullPairs( const BaseKeyframedHull **toonHulls, unsigned numToonHulls,
												   const BaseConcentricSimulatedHull **fireHulls, unsigned numFireHulls,
												   wsw::StaticVector<std::optional<uint8_t>, kMaxKeyframedHulls>
												       *pairIndicesForKeyframedHulls,
												   wsw::StaticVector<std::optional<uint8_t>, kMaxConcentricHulls>
												       *pairIndicesForConcentricHulls ) -> unsigned {
	pairIndicesForKeyframedHulls->clear();
	pairIndicesForConcentricHulls->clear();
	pairIndicesForKeyframedHulls->insert( pairIndicesForKeyframedHulls->end(), kMaxKeyframedHulls, std::nullopt );
	pairIndicesForConcentricHulls->insert( pairIndicesForConcentricHulls->end(), kMaxConcentricHulls, std::nullopt );

	unsigned numMatchedPairs = 0;
	// Try finding coupled hulls
	for( unsigned concentricHullIndex = 0; concentricHullIndex < numFireHulls; ++concentricHullIndex ) {
		const BaseConcentricSimulatedHull *concentricHull = fireHulls[concentricHullIndex];
		if( concentricHull->compoundMeshKey != 0 ) {
			for( unsigned keyframedHullIndex = 0; keyframedHullIndex < numToonHulls; ++keyframedHullIndex ) {
				const BaseKeyframedHull *keyframedHull = toonHulls[keyframedHullIndex];
				if( concentricHull->compoundMeshKey == 0 ) {
					( *pairIndicesForConcentricHulls )[concentricHullIndex] = (uint8_t)numMatchedPairs;
					( *pairIndicesForKeyframedHulls )[keyframedHullIndex]   = (uint8_t)numMatchedPairs;
					++numMatchedPairs;
					break;
				}
			}
		}
	}

	assert( numMatchedPairs <= kMaxToonSmokeHulls && numMatchedPairs <= kMaxFireHulls );
	return numMatchedPairs;
}

void getBoundsForCagedMesh( float boundingRadius, const vec3_t origin, const vec3_t cageMins, const vec3_t cageMaxs,
							vec3_t outMins, vec3_t outMaxs ){
	VectorCopy( cageMins, outMins );
	VectorCopy( cageMaxs, outMaxs );

	vec3_t meshBoundingBoxFromOrigin = { boundingRadius, boundingRadius, boundingRadius };

	vec3_t meshMins, meshMaxs;

	VectorSubtract( origin, meshBoundingBoxFromOrigin, meshMins );
	VectorAdd( origin, meshBoundingBoxFromOrigin, meshMaxs );

	outMaxs[0] = wsw::min( meshMaxs[0], outMaxs[0] );
	outMaxs[1] = wsw::min( meshMaxs[1], outMaxs[1] );
	outMaxs[2] = wsw::min( meshMaxs[2], outMaxs[2] );

	outMins[0] = wsw::max( meshMins[0], outMins[0] );
	outMins[1] = wsw::max( meshMins[1], outMins[1] );
	outMins[2] = wsw::max( meshMins[2], outMins[2] );
}

void SimulatedHullsSystem::simulateFrameAndSubmit( int64_t currTime, DrawSceneRequest *drawSceneRequest ) {
	// Limit the time step
	const float timeDeltaSeconds = 1e-3f * (float)wsw::min<int64_t>( 33, currTime - m_lastTime );

	wsw::StaticVector<BaseRegularSimulatedHull *, kMaxRegularHulls> activeRegularHulls;
	wsw::StaticVector<BaseConcentricSimulatedHull *, kMaxConcentricHulls> activeConcentricHulls;
	wsw::StaticVector<BaseKeyframedHull *, kMaxKeyframedHulls> activeKeyframedHulls;
    wsw::StaticVector<DynamicCageHull *, kMaxKeyframedHulls> activeDynamicCageHulls;

	for( FireHull *hull = m_fireHullsHead, *nextHull = nullptr; hull; hull = nextHull ) { nextHull = hull->next;
		if( hull->spawnTime + hull->lifetime > currTime ) [[likely]] {
			hull->simulate( currTime, timeDeltaSeconds, &m_rng );
			activeConcentricHulls.push_back( hull );
		} else {
			unlinkAndFreeFireHull( hull );
		}
	}
	for( FireClusterHull *hull = m_fireClusterHullsHead, *next = nullptr; hull; hull = next ) { next = hull->next;
		if( hull->spawnTime + hull->lifetime > currTime ) [[likely]] {
			hull->simulate( currTime, timeDeltaSeconds, &m_rng );
			activeConcentricHulls.push_back( hull );
		} else {
			unlinkAndFreeFireClusterHull( hull );
		}
	}
	for( BlastHull *hull = m_blastHullsHead, *nextHull = nullptr; hull; hull = nextHull ) { nextHull = hull->next;
		if( hull->spawnTime + hull->lifetime > currTime ) [[likely]] {
			hull->simulate( currTime, timeDeltaSeconds, &m_rng );
			activeConcentricHulls.push_back( hull );
		} else {
			unlinkAndFreeBlastHull( hull );
		}
	}
	for( SmokeHull *hull = m_smokeHullsHead, *nextHull = nullptr; hull; hull = nextHull ) { nextHull = hull->next;
		if( hull->spawnTime + hull->lifetime > currTime ) [[likely]] {
			hull->simulate( currTime, timeDeltaSeconds, &m_rng );
			activeRegularHulls.push_back( hull );
		} else {
			unlinkAndFreeSmokeHull( hull );
		}
	}
	for( WaveHull *hull = m_waveHullsHead, *nextHull = nullptr; hull; hull = nextHull ) { nextHull = hull->next;
		if( hull->spawnTime + hull->lifetime > currTime ) [[likely]] {
			hull->simulate( currTime, timeDeltaSeconds, &m_rng );
			activeRegularHulls.push_back( hull );
		} else {
			unlinkAndFreeWaveHull( hull );
		}
	}

    for( auto & m_loadedStaticCage : m_loadedStaticCages ) {
        StaticCage *cage = std::addressof( m_loadedStaticCage );
        KeyframedHull *head = cage->head;
        for( KeyframedHull *hull = head, *nextHull = nullptr; hull; hull = nextHull ) { nextHull = hull->next;
            if( hull->spawnTime + hull->lifetime > currTime ) [[likely]] {
                //hull->simulate( currTime, timeDeltaSeconds, &cg.polyEffectsSystem, cageGeometry );
                activeKeyframedHulls.push_back( hull );
            } else {
                unlinkAndFreeStaticCageHull( hull );
            }
        }
    }

	for( auto & m_loadedDynamicCage : m_loadedDynamicCages ) {
		DynamicCage *cage = std::addressof( m_loadedDynamicCage );
		DynamicCageHull *head = cage->head;
		for( DynamicCageHull *hull = head, *nextHull = nullptr; hull; hull = nextHull ) { nextHull = hull->next;
			if( hull->spawnTime + hull->lifetime > currTime ) [[likely]] {
				hull->simulate( currTime, timeDeltaSeconds, &cg.polyEffectsSystem );
				activeDynamicCageHulls.push_back( hull );
			} else {
				unlinkAndFreeDynamicCageHull( hull );
			}
		}
	}

	m_frameSharedOverrideColorsBuffer.clear();

	for( BaseRegularSimulatedHull *__restrict hull: activeRegularHulls ) {
		const SolidAppearanceRules *solidAppearanceRules = nullptr;
		const CloudAppearanceRules *cloudAppearanceRules = nullptr;
		if( const auto *solidAndCloudRules = std::get_if<SolidAndCloudAppearanceRules>( &hull->appearanceRules ) ) {
			solidAppearanceRules = &solidAndCloudRules->solidRules;
			cloudAppearanceRules = &solidAndCloudRules->cloudRules;
		} else {
			solidAppearanceRules = std::get_if<SolidAppearanceRules>( &hull->appearanceRules );
			cloudAppearanceRules = std::get_if<CloudAppearanceRules>( &hull->appearanceRules );
		}

		assert( solidAppearanceRules || cloudAppearanceRules );
		SharedMeshData *const __restrict sharedMeshData = &hull->sharedMeshData;
		
		sharedMeshData->simulatedPositions   = hull->vertexPositions[hull->positionsFrame];
		sharedMeshData->simulatedNormals     = hull->vertexNormals;
		sharedMeshData->simulatedColors      = hull->vertexColors;
		sharedMeshData->minZLastFrame        = hull->minZLastFrame;
		sharedMeshData->maxZLastFrame        = hull->maxZLastFrame;
		sharedMeshData->minFadedOutAlpha     = hull->minFadedOutAlpha;
		sharedMeshData->viewDotFade          = hull->vertexViewDotFade;
		sharedMeshData->zFade                = hull->vertexZFade;
		sharedMeshData->simulatedSubdivLevel = hull->subdivLevel;
		sharedMeshData->tesselateClosestLod  = hull->tesselateClosestLod;
		sharedMeshData->lerpNextLevelColors  = hull->leprNextLevelColors;

		sharedMeshData->cachedChosenSolidSubdivLevel     = std::nullopt;
		sharedMeshData->cachedOverrideColorsSpanInBuffer = std::nullopt;
		sharedMeshData->overrideColorsBuffer             = &m_frameSharedOverrideColorsBuffer;
		sharedMeshData->hasSibling                       = solidAppearanceRules && cloudAppearanceRules;

		assert( hull->subdivLevel );
		if( solidAppearanceRules ) [[likely]] {
			HullSolidDynamicMesh *const __restrict mesh = &hull->submittedSolidMesh;

			Vector4Copy( hull->mins, mesh->cullMins );
			Vector4Copy( hull->maxs, mesh->cullMaxs );

			mesh->applyVertexDynLight = hull->applyVertexDynLight;
			mesh->material            = solidAppearanceRules->material;
			mesh->m_shared            = sharedMeshData;

			drawSceneRequest->addDynamicMesh( mesh );
		}

		if( cloudAppearanceRules ) [[unlikely]] {
			assert( !cloudAppearanceRules->spanOfMeshProps.empty() );
			assert( cloudAppearanceRules->spanOfMeshProps.size() <= std::size( hull->submittedCloudMeshes ) );

			const float hullLifetimeFrac = (float)( currTime - hull->spawnTime ) * Q_Rcp( (float)hull->lifetime );

			for( size_t meshNum = 0; meshNum < cloudAppearanceRules->spanOfMeshProps.size(); ++meshNum ) {
				const CloudMeshProps &__restrict meshProps  = cloudAppearanceRules->spanOfMeshProps[meshNum];
				HullCloudDynamicMesh *const __restrict mesh = hull->submittedCloudMeshes + meshNum;

				mesh->m_spriteRadius = meshProps.radiusLifespan.getValueForLifetimeFrac( hullLifetimeFrac );
				if( mesh->m_spriteRadius > 1.0f ) [[likely]] {
					mesh->m_alphaScale = meshProps.alphaScaleLifespan.getValueForLifetimeFrac( hullLifetimeFrac );
					if( mesh->m_alphaScale >= ( 1.0f / 255.0f ) ) {
						Vector4Copy( hull->mins, mesh->cullMins );
						Vector4Copy( hull->maxs, mesh->cullMaxs );

						Vector4Copy( meshProps.overlayColor, mesh->m_spriteColor );

						mesh->material            = meshProps.material;
						mesh->applyVertexDynLight = hull->applyVertexDynLight;
						mesh->m_shared            = sharedMeshData;
						mesh->m_lifetimeSeconds   = 1e-3f * (float)( currTime - hull->spawnTime );
						mesh->m_applyRotation     = meshProps.applyRotation;

						mesh->m_tessLevelShiftForMinVertexIndex = meshProps.tessLevelShiftForMinVertexIndex;
						mesh->m_tessLevelShiftForMaxVertexIndex = meshProps.tessLevelShiftForMaxVertexIndex;
						mesh->m_shiftFromDefaultLevelToHide     = meshProps.shiftFromDefaultLevelToHide;

						// It's more convenient to initialize it on demand
						if ( !( mesh->m_speedIndexShiftInTable | mesh->m_phaseIndexShiftInTable )) [[unlikely]] {
							const auto randomWord = (uint16_t) m_rng.next();
							mesh->m_speedIndexShiftInTable = ( randomWord >> 0 ) & 0xFF;
							mesh->m_phaseIndexShiftInTable = ( randomWord >> 8 ) & 0xFF;
						}

						drawSceneRequest->addDynamicMesh( mesh );
					}
				}
			}
		}
	}

	// TODO: Track bounds
	wsw::StaticVector<std::optional<uint8_t>, kMaxKeyframedHulls> pairIndicesForKeyframedHulls;
	wsw::StaticVector<std::optional<uint8_t>, kMaxConcentricHulls> pairIndicesForConcentricHulls;
	buildMatchingHullPairs( (const BaseKeyframedHull **)activeKeyframedHulls.data(), activeKeyframedHulls.size(),
							(const BaseConcentricSimulatedHull **)activeConcentricHulls.data(), activeConcentricHulls.size(),
							&pairIndicesForKeyframedHulls, &pairIndicesForConcentricHulls ); /// should be REMOVED

	wsw::StaticVector<unsigned, kMaxKeyframedHulls> meshDataOffsetsForPairs;
	wsw::StaticVector<uint8_t, kMaxKeyframedHulls> numAddedMeshesForPairs;
	wsw::StaticVector<std::pair<Vec3, Vec3>, kMaxKeyframedHulls> boundsForPairs;
	wsw::StaticVector<float, kMaxKeyframedHulls> topAddedLayersForPairs;

	// We multiply given layer orders by 2 to be able to make distinct orders for solid and cloud meshes
	constexpr float solidLayerOrderBoost = 0.0f;
	constexpr float cloudLayerOrderBoost = 1.0f;

	unsigned offsetOfMultilayerMeshData = 0;

	// TODO: zipWithIndex?
	unsigned keyframedHullIndex = 0;
    /// MODIFY
	for( const BaseKeyframedHull *__restrict hull: activeKeyframedHulls ) {

		assert( hull->numLayers );

		const DynamicMesh **submittedMeshesBuffer = m_storageOfSubmittedMeshPtrs.get( 0 ) + offsetOfMultilayerMeshData;
		float *const submittedOrderDesignators    = m_storageOfSubmittedMeshOrderDesignators.get( 0 ) + offsetOfMultilayerMeshData;

        const unsigned numMeshesToRender = hull->numSharedCageCagedMeshes;
		unsigned numMeshesToSubmit = 0;

		const float rcpLifetime  = Q_Rcp( (float)hull->lifetime );
		const float lifetimeFrac = (float)( currTime - hull->spawnTime ) * rcpLifetime;

		const StaticCagedMesh *firstCagedMesh = hull->sharedCageCagedMeshes[0];
		const unsigned cageKey = firstCagedMesh->loadedCageKey;
		const Geometry *cage = &m_loadedStaticCages[cageKey].cageGeometry;

		float maxBoundingRadius = 0.0f;

        for( unsigned meshNum = 0; meshNum < numMeshesToRender; ++meshNum ) {

			assert( cageKey == hull->sharedCageCagedMeshes[meshNum]->loadedCageKey );

            const AppearanceRules *appearanceRules = &hull->appearanceRules[meshNum];

            const SolidAppearanceRules *solidAppearanceRules = nullptr;
            const CloudAppearanceRules *cloudAppearanceRules = nullptr;
            if( const auto *solidAndCloudRules = std::get_if<SolidAndCloudAppearanceRules>( appearanceRules ) ) {
                solidAppearanceRules = &solidAndCloudRules->solidRules;
                cloudAppearanceRules = &solidAndCloudRules->cloudRules;
            } else {
                solidAppearanceRules = std::get_if<SolidAppearanceRules>( appearanceRules );
                cloudAppearanceRules = std::get_if<CloudAppearanceRules>( appearanceRules );
            }

            assert( solidAppearanceRules || cloudAppearanceRules );

			assert( solidAppearanceRules || cloudAppearanceRules );
            SharedMeshData *sharedMeshData = &hull->sharedMeshData[meshNum];

			sharedMeshData->lifetimeFrac = lifetimeFrac;

			StaticCagedMesh *meshToRender = hull->sharedCageCagedMeshes[meshNum];
			sharedMeshData->meshToRender  = meshToRender;

			sharedMeshData->simulatedPositions  = nullptr;
			sharedMeshData->cageVertexPositions = hull->vertexMoveDirections;

			sharedMeshData->cageTriIndices = cage->triIndices.data();
			sharedMeshData->limitsAtDirections = hull->limitsAtDirections;

			sharedMeshData->scale  = hull->scale;
            VectorCopy( hull->origin, sharedMeshData->origin );

			sharedMeshData->simulatedNormals     = nullptr;

			sharedMeshData->minZLastFrame        = 0.0f;
			sharedMeshData->maxZLastFrame        = 0.0f;
			sharedMeshData->zFade                = ZFade::NoFade;
			sharedMeshData->simulatedSubdivLevel = 0;
			sharedMeshData->tesselateClosestLod  = false;
			sharedMeshData->lerpNextLevelColors  = true;
			sharedMeshData->nextLodTangentRatio  = 0.30f;

			sharedMeshData->cachedChosenSolidSubdivLevel     = std::nullopt;
			sharedMeshData->cachedOverrideColorsSpanInBuffer = std::nullopt;
			sharedMeshData->overrideColorsBuffer             = &m_frameSharedOverrideColorsBuffer;
			sharedMeshData->hasSibling                       = solidAppearanceRules && cloudAppearanceRules;

			sharedMeshData->isAKeyframedHull = true;

            const unsigned numFrames = meshToRender->numFrames;
            const unsigned currFrame = wsw::min( numFrames - 1, (unsigned)( lifetimeFrac * numFrames ) );

            // we can assume the LODs have about the same bounding radius as the most detailed version, also:
            // as other LODs are used at longer distances, even if the LOD was slightly larger, it will not be noticeable
            // if the culling is premature.

			vec4_t mins, maxs;
			const float currBoundingRadius = meshToRender->boundingRadii[currFrame] * hull->scale;
			getBoundsForCagedMesh( currBoundingRadius, hull->origin, hull->cageMins, hull->cageMaxs,
								   mins, maxs );

            maxBoundingRadius = wsw::max( maxBoundingRadius, currBoundingRadius );
            mins[3] = 0.0f, maxs[3] = 1.0f;

			if( solidAppearanceRules ) [[likely]] {
				HullSolidDynamicMesh *mesh = &hull->submittedSolidMesh[meshNum];

                Vector4Copy( mins, mesh->cullMins );
                Vector4Copy( maxs, mesh->cullMaxs );

				mesh->material = solidAppearanceRules->material;
				mesh->m_shared = sharedMeshData;
				// TODO: Restore this functionality if it could be useful
				//mesh->applyVertexDynLight = hull->applyVertexDynLight;

				const float drawOrderDesignator = 2.0f * (float)( numMeshesToRender - meshNum ) + solidLayerOrderBoost;

				submittedMeshesBuffer[numMeshesToSubmit]     = mesh;
				submittedOrderDesignators[numMeshesToSubmit] = drawOrderDesignator;


				numMeshesToSubmit++;
			}

            if( cloudAppearanceRules ) [[unlikely]] {

                assert( !cloudAppearanceRules->spanOfMeshProps.empty() );
                assert( cloudAppearanceRules->spanOfMeshProps.size() <= std::size( layer->submittedCloudMeshes ) );

                const float hullLifetimeFrac = (float)( currTime - hull->spawnTime ) * Q_Rcp( (float)hull->lifetime );

                for( size_t meshPropNum = 0; meshPropNum < cloudAppearanceRules->spanOfMeshProps.size(); ++meshPropNum ) { // seems wrong, the hull struct only has one single slot per layer (before) for submitted cloud meshes..
                    const CloudMeshProps &__restrict meshProps  = cloudAppearanceRules->spanOfMeshProps[meshPropNum];
                    HullCloudDynamicMesh *const __restrict mesh = &hull->submittedCloudMesh[meshNum];

                    mesh->m_spriteRadius = meshProps.radiusLifespan.getValueForLifetimeFrac( hullLifetimeFrac );
                    if( mesh->m_spriteRadius > 1.0f ) [[likely]] {
                        mesh->m_alphaScale = meshProps.alphaScaleLifespan.getValueForLifetimeFrac( hullLifetimeFrac );
                        Vector4Copy( mins, mesh->cullMins );
                        Vector4Copy( maxs, mesh->cullMaxs );

                        Vector4Copy( meshProps.overlayColor, mesh->m_spriteColor );

                        mesh->material            = meshProps.material;
                        //mesh->applyVertexDynLight = hull->applyVertexDynLight; // TODO: restore this functionality if useful
                        mesh->m_shared            = sharedMeshData;
                        mesh->m_lifetimeSeconds   = 1e-3f * (float)( currTime - hull->spawnTime );
                        mesh->m_applyRotation     = meshProps.applyRotation;

                        mesh->m_tessLevelShiftForMinVertexIndex = meshProps.tessLevelShiftForMinVertexIndex;
                        mesh->m_tessLevelShiftForMaxVertexIndex = meshProps.tessLevelShiftForMaxVertexIndex;
                        mesh->m_fractionOfParticlesToRender     = meshProps.fractionOfParticlesToRender;
                        cgNotice() << "fraction" << mesh->m_fractionOfParticlesToRender << " " << meshProps.fractionOfParticlesToRender;
                        mesh->m_shiftFromDefaultLevelToHide     = meshProps.shiftFromDefaultLevelToHide;

                        if( !( mesh->m_speedIndexShiftInTable | mesh->m_phaseIndexShiftInTable ) ) [[unlikely]] {
                            const auto randomWord          = (uint16_t)m_rng.next();
                            mesh->m_speedIndexShiftInTable = ( randomWord >> 0 ) & 0xFF;
                            mesh->m_phaseIndexShiftInTable = ( randomWord >> 8 ) & 0xFF;
                        }

                        const float drawOrderDesignator = 2.0f * (float)( numMeshesToRender - meshNum ) + cloudLayerOrderBoost;

                        submittedMeshesBuffer[numMeshesToSubmit]     = mesh;
                        submittedOrderDesignators[numMeshesToSubmit] = drawOrderDesignator;

                        numMeshesToSubmit++;
                    }
                }
            }
        }


		if( numMeshesToSubmit ) [[likely]] {
			assert( numMeshesToSubmit <= kMaxMeshesPerHull );

			vec4_t mins, maxs;
			getBoundsForCagedMesh( maxBoundingRadius, hull->origin, hull->cageMins, hull->cageMaxs,
								   mins, maxs );

			mins[3] = 0.0f, maxs[3] = 1.0f;

            drawSceneRequest->addCompoundDynamicMesh( mins, maxs, submittedMeshesBuffer,
                                                      numMeshesToSubmit, submittedOrderDesignators );
		}


		keyframedHullIndex++;

        offsetOfMultilayerMeshData += numMeshesToSubmit;

	}
	/// MODIFY END

	assert( meshDataOffsetsForPairs.size() == numAddedMeshesForPairs.size() );
	assert( meshDataOffsetsForPairs.size() == boundsForPairs.size() );
	assert( meshDataOffsetsForPairs.size() == topAddedLayersForPairs.size() );

	unsigned concentricHullIndex = 0;
	for( const BaseConcentricSimulatedHull *__restrict hull: activeConcentricHulls ) {
		assert( hull->numLayers );

		float startFromOrder;
		const DynamicMesh **submittedMeshesBuffer;
		float *submittedOrderDesignators;
		if( const std::optional<uint8_t> pairIndex = pairIndicesForConcentricHulls[concentricHullIndex] ) {
			const unsigned meshDataOffset     = meshDataOffsetsForPairs[*pairIndex];
			const unsigned numAddedToonMeshes = numAddedMeshesForPairs[*pairIndex];
			submittedMeshesBuffer             = m_storageOfSubmittedMeshPtrs.get( 0 ) + meshDataOffset + numAddedToonMeshes;
			submittedOrderDesignators         = m_storageOfSubmittedMeshOrderDesignators.get( 0 ) + meshDataOffset + numAddedToonMeshes;
			startFromOrder                    = topAddedLayersForPairs[*pairIndex];
		} else {
			submittedMeshesBuffer     = m_storageOfSubmittedMeshPtrs.get( 0 ) + offsetOfMultilayerMeshData;
			submittedOrderDesignators = m_storageOfSubmittedMeshOrderDesignators.get( 0 ) + offsetOfMultilayerMeshData;
			startFromOrder            = 0.0f;
		}

		unsigned numMeshesToSubmit = 0;
		for( unsigned layerNum = 0; layerNum < hull->numLayers; ++layerNum ) {
			BaseConcentricSimulatedHull::Layer *__restrict layer = &hull->layers[layerNum];

			const AppearanceRules *appearanceRules = &hull->appearanceRules;
			if( layer->overrideAppearanceRules ) {
				appearanceRules = layer->overrideAppearanceRules;
			}

			const SolidAppearanceRules *solidAppearanceRules = nullptr;
			const CloudAppearanceRules *cloudAppearanceRules = nullptr;
			if( const auto *solidAndCloudRules = std::get_if<SolidAndCloudAppearanceRules>( appearanceRules ) ) {
				solidAppearanceRules = &solidAndCloudRules->solidRules;
				cloudAppearanceRules = &solidAndCloudRules->cloudRules;
			} else {
				solidAppearanceRules = std::get_if<SolidAppearanceRules>( appearanceRules );
				cloudAppearanceRules = std::get_if<CloudAppearanceRules>( appearanceRules );
			}

			assert( solidAppearanceRules || cloudAppearanceRules );
			SharedMeshData *const __restrict sharedMeshData = layer->sharedMeshData;

			sharedMeshData->simulatedPositions   = layer->vertexPositions;
			sharedMeshData->simulatedNormals     = nullptr;
			sharedMeshData->simulatedColors      = layer->vertexColors;

			sharedMeshData->minZLastFrame        = 0.0f;
			sharedMeshData->maxZLastFrame        = 0.0f;
			sharedMeshData->minFadedOutAlpha     = layer->overrideMinFadedOutAlpha.value_or( hull->minFadedOutAlpha );
			sharedMeshData->viewDotFade          = layer->overrideHullFade.value_or( hull->vertexViewDotFade );
			sharedMeshData->zFade                = ZFade::NoFade;
			sharedMeshData->simulatedSubdivLevel = hull->subdivLevel;
			sharedMeshData->tesselateClosestLod  = true;
			sharedMeshData->lerpNextLevelColors  = true;

			sharedMeshData->cachedChosenSolidSubdivLevel     = std::nullopt;
			sharedMeshData->cachedOverrideColorsSpanInBuffer = std::nullopt;
			sharedMeshData->overrideColorsBuffer             = &m_frameSharedOverrideColorsBuffer;
			sharedMeshData->hasSibling                       = solidAppearanceRules && cloudAppearanceRules;

			if( solidAppearanceRules ) [[likely]] {
				HullSolidDynamicMesh *const __restrict mesh = layer->submittedSolidMesh;

				Vector4Copy( layer->mins, mesh->cullMins );
				Vector4Copy( layer->maxs, mesh->cullMaxs );
				
				mesh->material            = nullptr;
				mesh->applyVertexDynLight = hull->applyVertexDynLight;
				mesh->m_shared            = sharedMeshData;

				const float drawOrderDesignator = 2.0f * layer->drawOrderDesignator + solidLayerOrderBoost + startFromOrder;

				submittedMeshesBuffer[numMeshesToSubmit]     = mesh;
				submittedOrderDesignators[numMeshesToSubmit] = drawOrderDesignator;
				numMeshesToSubmit++;
			}

			if( cloudAppearanceRules ) [[unlikely]] {
				assert( !cloudAppearanceRules->spanOfMeshProps.empty() );
				assert( cloudAppearanceRules->spanOfMeshProps.size() <= std::size( layer->submittedCloudMeshes ) );

				const float hullLifetimeFrac = (float)( currTime - hull->spawnTime ) * Q_Rcp( (float)hull->lifetime );

				for( size_t meshNum = 0; meshNum < cloudAppearanceRules->spanOfMeshProps.size(); ++meshNum ) {
					const CloudMeshProps &__restrict meshProps  = cloudAppearanceRules->spanOfMeshProps[meshNum];
					HullCloudDynamicMesh *const __restrict mesh = layer->submittedCloudMeshes[meshNum];

					mesh->m_spriteRadius = meshProps.radiusLifespan.getValueForLifetimeFrac( hullLifetimeFrac );
					if( mesh->m_spriteRadius > 1.0f ) [[likely]] {
						mesh->m_alphaScale = meshProps.alphaScaleLifespan.getValueForLifetimeFrac( hullLifetimeFrac );
						if( mesh->m_alphaScale >= ( 1.0f / 255.0f ) ) {
							Vector4Copy( layer->mins, mesh->cullMins );
							Vector4Copy( layer->maxs, mesh->cullMaxs );

							Vector4Copy( meshProps.overlayColor, mesh->m_spriteColor );

							mesh->material            = meshProps.material;
							mesh->applyVertexDynLight = hull->applyVertexDynLight;
							mesh->m_shared            = sharedMeshData;
							mesh->m_lifetimeSeconds   = 1e-3f * (float)( currTime - hull->spawnTime );
							mesh->m_applyRotation     = meshProps.applyRotation;

							mesh->m_tessLevelShiftForMinVertexIndex = meshProps.tessLevelShiftForMinVertexIndex;
							mesh->m_tessLevelShiftForMaxVertexIndex = meshProps.tessLevelShiftForMaxVertexIndex;
							mesh->m_shiftFromDefaultLevelToHide     = meshProps.shiftFromDefaultLevelToHide;

							if( !( mesh->m_speedIndexShiftInTable | mesh->m_phaseIndexShiftInTable ) ) [[unlikely]] {
								const auto randomWord          = (uint16_t)m_rng.next();
								mesh->m_speedIndexShiftInTable = ( randomWord >> 0 ) & 0xFF;
								mesh->m_phaseIndexShiftInTable = ( randomWord >> 8 ) & 0xFF;
							}

							const float drawOrderDesignator = 2.0f * layer->drawOrderDesignator + cloudLayerOrderBoost + startFromOrder;

							submittedMeshesBuffer[numMeshesToSubmit]     = mesh;
							submittedOrderDesignators[numMeshesToSubmit] = drawOrderDesignator;
							numMeshesToSubmit++;
						}
					}
				}
			}
		}

		// If this concentric hull is coupled with a toon hull
		if( const std::optional<uint8_t> pairIndex = pairIndicesForConcentricHulls[concentricHullIndex] ) {
			// The number of meshes of the coupled toon hull
			const unsigned numAddedToonMeshes = numAddedMeshesForPairs[*pairIndex];
			// If we're going to submit something
			if( numAddedToonMeshes | numMeshesToSubmit ) [[likely]] {
				// TODO: Use some "combine bounds" subroutine
				BoundsBuilder combinedBoundsBuilder;
				combinedBoundsBuilder.addPoint( hull->mins );
				combinedBoundsBuilder.addPoint( hull->maxs );

				if( numAddedToonMeshes ) [[likely]] {
					// Roll pointers back to the data of toon hull
					submittedMeshesBuffer     -= numAddedToonMeshes;
					submittedOrderDesignators -= numAddedToonMeshes;

					combinedBoundsBuilder.addPoint( boundsForPairs[*pairIndex].first.Data() );
					combinedBoundsBuilder.addPoint( boundsForPairs[*pairIndex].second.Data() );
				}

				vec4_t combinedMins, combinedMaxs;
				combinedBoundsBuilder.storeToWithAddedEpsilon( combinedMins, combinedMaxs );
				combinedMins[3] = 0.0f, combinedMaxs[3] = 1.0f;

				const unsigned actualNumMeshesToSubmit = numAddedToonMeshes + numMeshesToSubmit;
				assert( actualNumMeshesToSubmit <= kMaxMeshesPerHull );
				drawSceneRequest->addCompoundDynamicMesh( combinedMins, combinedMaxs, submittedMeshesBuffer,
														  actualNumMeshesToSubmit, submittedOrderDesignators );
				// We add meshes to the space reserved by the toon hull, offsetOfMultilayerMeshData is kept the same
				assert( actualNumMeshesToSubmit <= kMaxMeshesPerHull );
			}
		} else {
			// Just submit meshes of this concentric hull, if any
			if( numMeshesToSubmit ) [[likely]] {
				assert( numMeshesToSubmit <= kMaxMeshesPerHull );
				drawSceneRequest->addCompoundDynamicMesh( hull->mins, hull->maxs, submittedMeshesBuffer,
														  numMeshesToSubmit, submittedOrderDesignators );
				offsetOfMultilayerMeshData += numMeshesToSubmit;
			}
		}

		++concentricHullIndex;
	}

	m_lastTime = currTime;
}

void SimulatedHullsSystem::BaseRegularSimulatedHull::simulate( int64_t currTime, float timeDeltaSeconds,
															   wsw::RandomGenerator *__restrict rng ) {
	const vec4_t *const __restrict oldPositions = this->vertexPositions[positionsFrame];
	// Switch old/new positions buffer
	positionsFrame = ( positionsFrame + 1 ) % 2;
	vec4_t *const __restrict newPositions = this->vertexPositions[positionsFrame];

	vec3_t *const __restrict forceVelocities = this->vertexForceVelocities;
	vec3_t *const __restrict burstVelocities = this->vertexBurstVelocities;

	const auto &icosphereData  = ::basicHullsHolder.getIcosphereForLevel( subdivLevel );
	const unsigned numVertices = icosphereData.vertices.size();
	auto *const __restrict combinedVelocities = (vec3_t *)alloca( sizeof( vec3_t ) * numVertices );

	BoundsBuilder boundsBuilder;
	assert( timeDeltaSeconds < 0.1f );
	assert( numVertices > 0 );

	// Compute ideal positions (as if there were no obstacles)

	const float burstSpeedDecayMultiplier = 1.0f - 1.5f * timeDeltaSeconds;
	if( expansionStartAt > currTime ) {
		unsigned i = 0;
		do {
			VectorScale( burstVelocities[i], burstSpeedDecayMultiplier, burstVelocities[i] );
			VectorAdd( burstVelocities[i], forceVelocities[i], combinedVelocities[i] );
			VectorMA( oldPositions[i], timeDeltaSeconds, combinedVelocities[i], newPositions[i] );
			// TODO: We should be able to supply vec4
			boundsBuilder.addPoint( newPositions[i] );
		} while( ++i < numVertices );
	} else {
		// Having vertex normals buffer is now mandatory for hulls expansion
		const vec4_t *const __restrict normals = vertexNormals;
		assert( normals );

		const int64_t expansionMillisSoFar = currTime - expansionStartAt;
		const int64_t expansionDuration    = lifetime - ( expansionStartAt - spawnTime );

		assert( expansionMillisSoFar >= 0 );
		assert( expansionDuration > 0 );
		const float expansionFrac = (float)expansionMillisSoFar * Q_Rcp( (float)expansionDuration );
		assert( expansionFrac >= 0.0f && expansionFrac <= 1.0f );

		const float archimedesTopAccelNow     = archimedesTopAccel.getValueForLifetimeFrac( expansionFrac );
		const float archimedesBottomAccelNow  = archimedesBottomAccel.getValueForLifetimeFrac( expansionFrac );
		const float xyExpansionTopAccelNow    = xyExpansionTopAccel.getValueForLifetimeFrac( expansionFrac );
		const float xyExpansionBottomAccelNow = xyExpansionBottomAccel.getValueForLifetimeFrac( expansionFrac );

		const float rcpDeltaZ = ( maxZLastFrame - minZLastFrame ) > 0.1f ? Q_Rcp( maxZLastFrame - minZLastFrame ) : 1.0f;

		unsigned i = 0;
		do {
			const float zFrac               = ( oldPositions[i][2] - minZLastFrame ) * rcpDeltaZ;
			const float archimedesAccel     = std::lerp( archimedesBottomAccelNow, archimedesTopAccelNow, Q_Sqrt( zFrac ) );
			const float expansionAccel      = std::lerp( xyExpansionBottomAccelNow, xyExpansionTopAccelNow, zFrac );
			const float expansionMultiplier = expansionAccel * timeDeltaSeconds;

			forceVelocities[i][0] += expansionMultiplier * normals[i][0];
			forceVelocities[i][1] += expansionMultiplier * normals[i][1];
			forceVelocities[i][2] += archimedesAccel * timeDeltaSeconds;

			VectorScale( burstVelocities[i], burstSpeedDecayMultiplier, burstVelocities[i] );
			VectorAdd( burstVelocities[i], forceVelocities[i], combinedVelocities[i] );

			VectorMA( oldPositions[i], timeDeltaSeconds, combinedVelocities[i], newPositions[i] );
			// TODO: We should be able to supply vec4
			boundsBuilder.addPoint( newPositions[i] );
		} while( ++i < numVertices );
	}

	vec3_t verticesMins, verticesMaxs;
	boundsBuilder.storeToWithAddedEpsilon( verticesMins, verticesMaxs );
	// TODO: Allow bounds builder to store 4-vectors
	VectorCopy( verticesMins, mins );
	VectorCopy( verticesMaxs, maxs );
	mins[3] = 0.0f, maxs[3] = 1.0f;

	// TODO: Add a fused call
	CM_BuildShapeList( cl.cms, shapeList, verticesMins, verticesMaxs, MASK_SOLID | MASK_WATER );
	CM_ClipShapeList( cl.cms, shapeList, shapeList, verticesMins, verticesMaxs );

	minZLastFrame = std::numeric_limits<float>::max();
	maxZLastFrame = std::numeric_limits<float>::lowest();

	// int type is used to mitigate possible branching while performing bool->float conversions
	auto *const __restrict isVertexNonContacting          = (int *)alloca( sizeof( int ) * numVertices );
	auto *const __restrict indicesOfNonContactingVertices = (unsigned *)alloca( sizeof( unsigned ) * numVertices );

	trace_t clipTrace, slideTrace;
	unsigned numNonContactingVertices = 0;
	if( CM_GetNumShapesInShapeList( shapeList ) == 0 ) {
		unsigned i = 0;
		do {
			isVertexNonContacting[i]          = 1;
			indicesOfNonContactingVertices[i] = i;
		} while( ++i < numVertices );
		numNonContactingVertices = numVertices;
	} else {
		for( unsigned i = 0; i < numVertices; ++i ) {
			CM_ClipToShapeList( cl.cms, shapeList, &clipTrace, oldPositions[i], newPositions[i],
								vec3_origin, vec3_origin, MASK_SOLID );
			if( clipTrace.fraction == 1.0f ) [[likely]] {
				isVertexNonContacting[i] = 1;
				indicesOfNonContactingVertices[numNonContactingVertices++] = i;
			} else {
				isVertexNonContacting[i] = 0;
				bool putVertexAtTheContactPosition = true;

				if( const float squareSpeed = VectorLengthSquared( combinedVelocities[i] ); squareSpeed > 10.0f * 10.0f ) {
					vec3_t velocityDir;
					const float rcpSpeed = Q_RSqrt( squareSpeed );
					VectorScale( combinedVelocities[i], rcpSpeed, velocityDir );
					if( const float dot = std::fabs( DotProduct( velocityDir, clipTrace.plane.normal ) ); dot < 0.95f ) {
						const float speed               = Q_Rcp( rcpSpeed );
						const float idealMoveThisFrame  = timeDeltaSeconds * speed;
						const float distanceToObstacle  = idealMoveThisFrame * clipTrace.fraction;
						const float distanceAlongNormal = dot * distanceToObstacle;

						//   a'     c'
						//      ^ <--- ^    b' + c' = a'    | a' = lengthAlongNormal * surface normal'
						//      |     /                     | b' = -distanceToObstacle * velocity dir'
						//      |    /                      | c' = slide vec
						//      |   / b'                    | P  = trace endpos
						//      |  /
						//      | /
						// ____ |/__________
						//      P

						// c = a - b;

						vec3_t normalVec;
						VectorScale( clipTrace.plane.normal, distanceAlongNormal, normalVec );
						vec3_t vecToObstacle;
						VectorScale( velocityDir, -distanceToObstacle, vecToObstacle );
						vec3_t slideVec;
						VectorSubtract( normalVec, vecToObstacle, slideVec );

						// If the slide distance is sufficient for checks
						if( VectorLengthSquared( slideVec ) > 1.0f * 1.0f ) {
							vec3_t slideStartPoint, slideEndPoint;
							// Add an offset from the surface while testing sliding
							VectorAdd( clipTrace.endpos, clipTrace.plane.normal, slideStartPoint );
							VectorAdd( slideStartPoint, slideVec, slideEndPoint );

							CM_ClipToShapeList( cl.cms, shapeList, &slideTrace, slideStartPoint, slideEndPoint,
												vec3_origin, vec3_origin, MASK_SOLID );
							if( slideTrace.fraction == 1.0f ) {
								VectorCopy( slideEndPoint, newPositions[i] );
								putVertexAtTheContactPosition = false;
								// TODO: Modify velocity as well?
							}
						}
					}
				}

				if( putVertexAtTheContactPosition ) {
					VectorAdd( clipTrace.endpos, clipTrace.plane.normal, newPositions[i] );
				}

				// The final position for contacting vertices is considered to be known.
				const float *__restrict position = newPositions[i];
				minZLastFrame = wsw::min( minZLastFrame, position[2] );
				maxZLastFrame = wsw::max( maxZLastFrame, position[2] );

				// Make the contacting vertex transparent
				vertexColors[i][3] = 0;
			}
		}
	}

	if( numNonContactingVertices ) {
		// Update positions of non-contacting vertices
		unsigned i;
		if( numNonContactingVertices != numVertices ) {
			auto *const __restrict neighboursForVertices = icosphereData.vertexNeighbours.data();
			auto *const __restrict neighboursFreeMoveSum = (int *)alloca( sizeof( int ) * numNonContactingVertices );

			std::memset( neighboursFreeMoveSum, 0, sizeof( float ) * numNonContactingVertices );

			i = 0;
			do {
				const unsigned vertexIndex = indicesOfNonContactingVertices[i];
				assert( vertexIndex < numVertices );
				for( const unsigned neighbourIndex: neighboursForVertices[vertexIndex] ) {
					// Add to the integer accumulator
					neighboursFreeMoveSum[i] += isVertexNonContacting[neighbourIndex];
				}
			} while( ++i < numNonContactingVertices );

			assert( std::size( neighboursForVertices[0] ) == 5 );
			constexpr float rcpNumVertexNeighbours = 0.2f;

			i = 0;
			do {
				const unsigned vertexIndex = indicesOfNonContactingVertices[i];
				const float neighboursFrac = (float)neighboursFreeMoveSum[i] * rcpNumVertexNeighbours;
				assert( neighboursFrac >= 0.0f && neighboursFrac <= 1.0f );
				const float lerpFrac = ( 1.0f / 3.0f ) + ( 2.0f / 3.0f ) * neighboursFrac;

				float *const __restrict position = newPositions[vertexIndex];
				VectorLerp( oldPositions[vertexIndex], lerpFrac, position, position );

				minZLastFrame = wsw::min( minZLastFrame, position[2] );
				maxZLastFrame = wsw::max( maxZLastFrame, position[2] );
			} while( ++i < numNonContactingVertices );
		} else {
			// Just update positions.
			// This is worth the separate branch as its quite common for smoke hulls.
			i = 0;
			do {
				const unsigned vertexIndex       = indicesOfNonContactingVertices[i];
				const float *__restrict position = newPositions[vertexIndex];
				minZLastFrame = wsw::min( minZLastFrame, position[2] );
				maxZLastFrame = wsw::max( maxZLastFrame, position[2] );
			} while( ++i < numNonContactingVertices );
		}
	}

	// Once positions are defined, recalculate vertex normals
	if( vertexNormals ) {
		const auto neighboursOfVertices = icosphereData.vertexNeighbours.data();
		unsigned vertexNum = 0;
		do {
			const uint16_t *const neighboursOfVertex = neighboursOfVertices[vertexNum];
			const float *const __restrict currVertex = newPositions[vertexNum];
			float *const __restrict normal           = vertexNormals[vertexNum];

			unsigned neighbourIndex = 0;
			Vector4Clear( normal );
			do {
				const float *__restrict v2 = newPositions[neighboursOfVertex[neighbourIndex]];
				const float *__restrict v3 = newPositions[neighboursOfVertex[( neighbourIndex + 1 ) % 5]];
				vec3_t currTo2, currTo3, cross;
				VectorSubtract( v2, currVertex, currTo2 );
				VectorSubtract( v3, currVertex, currTo3 );
				CrossProduct( currTo2, currTo3, cross );
				if( const float squaredLength = VectorLengthSquared( cross ); squaredLength > 1.0f ) [[likely]] {
					const float rcpLength = Q_RSqrt( squaredLength );
					VectorMA( normal, rcpLength, cross, normal );
				}
			} while( ++neighbourIndex < 5 );

			// The sum of partial non-zero directories could be zero, check again
			if( const float squaredLength = VectorLengthSquared( normal ); squaredLength > wsw::square( 1e-3f ) ) [[likely]] {
				const float rcpLength = Q_RSqrt( squaredLength );
				VectorScale( normal, rcpLength, normal );
			} else {
				// Copy the unit vector of the original position as a normal
				VectorCopy( icosphereData.vertices[vertexNum], normal );
			}
		} while( ++vertexNum < numVertices );
	}

	const bool hasChangedColors = processColorChange( currTime, spawnTime, lifetime, colorChangeTimeline,
													  { vertexColors, numVertices }, &colorChangeState, rng );
	if( hasChangedColors && noColorChangeVertexColor && !noColorChangeIndices.empty() ) {
		for( const auto index: noColorChangeIndices ) {
			Vector4Copy( noColorChangeVertexColor, vertexColors[index] );
		}
	}
}

BoolConfigVar v_showPerf( wsw::StringView("showPerf"), { .byDefault = 1, .flags = CVAR_ARCHIVE });
BoolConfigVar v_showConstructPerf( wsw::StringView("showConstructPerf"), { .byDefault = 1, .flags = CVAR_ARCHIVE });

void SimulatedHullsSystem::BaseConcentricSimulatedHull::simulate( int64_t currTime, float timeDeltaSeconds,
																  wsw::RandomGenerator *__restrict rng ) {
	// Just move all vertices along directions clipping by limits

	const auto before = Sys_Microseconds();

	BoundsBuilder hullBoundsBuilder;

	const float *__restrict growthOrigin = this->origin;
	const unsigned numVertices = basicHullsHolder.getIcosphereForLevel( subdivLevel ).vertices.size();

	const float speedMultiplier = 1.0f - 1.5f * timeDeltaSeconds;


	// Sanity check
	assert( numLayers >= 1 && numLayers <= kMaxHullLayers );
	for( unsigned layerNum = 0; layerNum < numLayers; ++layerNum ) {
		BoundsBuilder layerBoundsBuilder;

		BaseConcentricSimulatedHull::Layer *layer   = &layers[layerNum];
		vec4_t *const __restrict positions          = layer->vertexPositions;
		vec2_t *const __restrict speedsAndDistances = layer->vertexSpeedsAndDistances;

		const float finalOffset = layer->finalOffset;
		for( unsigned i = 0; i < numVertices; ++i ) {
			float speed = speedsAndDistances[i][0];
			float distanceSoFar = speedsAndDistances[i][1];

			speed *= speedMultiplier;
			distanceSoFar += speed * timeDeltaSeconds;

			// Limit growth by the precomputed obstacle distance
			const float limit = wsw::max( 0.0f, limitsAtDirections[i] - finalOffset );
			distanceSoFar = wsw::min( distanceSoFar, limit );

			VectorMA( growthOrigin, distanceSoFar, vertexMoveDirections[i], positions[i] );

			// TODO: Allow supplying 4-component in-memory vectors directly
			layerBoundsBuilder.addPoint( positions[i] );

			// Write back to memory
			speedsAndDistances[i][0] = speed;
			speedsAndDistances[i][1] = distanceSoFar;
		}

		// TODO: Allow storing 4-component float vectors to memory directly
		layerBoundsBuilder.storeTo( layer->mins, layer->maxs );
		layer->mins[3] = 0.0f, layer->maxs[3] = 1.0f;

		// Don't relying on what hull is external is more robust

		// TODO: Allow adding other builder directly
		hullBoundsBuilder.addPoint( layer->mins );
		hullBoundsBuilder.addPoint( layer->maxs );
	}

	if( v_showPerf.get() ) {
		Com_Printf( "simulate concentric took %d micros for %d vertices\n", (int) (Sys_Microseconds() - before),
					(int) (numVertices * numLayers));
	}

	// TODO: Allow storing 4-component float vectors to memory directly
	hullBoundsBuilder.storeTo( this->mins, this->maxs );
	this->mins[3] = 0.0f, this->maxs[3] = 1.0f;

	for( unsigned i = 0; i < numLayers; ++i ) {
		Layer *const layer = &layers[i];
		processColorChange( currTime, spawnTime, lifetime, layer->colorChangeTimeline,
							{ layer->vertexColors, numVertices }, &layer->colorChangeState, rng );
	}
}

BoolConfigVar v_showMeshToRender( wsw::StringView("showMeshToRender"), { .byDefault = 1, .flags = CVAR_ARCHIVE });
IntConfigVar v_frameToShow( wsw::StringView("frameToShow"), { .byDefault = 1, .flags = CVAR_ARCHIVE });
IntConfigVar v_LODtoShow( wsw::StringView("LODtoShow"), { .byDefault = 1, .flags = CVAR_ARCHIVE });

void SimulatedHullsSystem::BaseKeyframedHull::simulate( int64_t currTime, float timeDeltaSeconds,
                                                        PolyEffectsSystem *effectsSystem, Geometry *cageGeometry  ) { /// GEOMETRY IS TMP

    // vertexPositions was vertexMoveDirs

	const float lifetimeFrac = (float)( currTime - spawnTime ) / (float)lifetime;

	StaticCagedMesh *meshToRender = sharedCageCagedMeshes[0];

	unsigned LODnum = 0;
	unsigned LODtoRender = v_LODtoShow.get();
	for( StaticCagedMesh *currLOD = meshToRender, *nextLOD = nullptr; nextLOD/*currLOD && ( LODnum < wsw::min( LODtoRender, 2u ) )*/; currLOD = nextLOD, LODnum++ ) {
        if( currLOD ) {
            meshToRender = currLOD;
            nextLOD = currLOD->nextLOD;
        } else if( LODnum > 20 || LODnum < -1 ){
            cgNotice() << S_COLOR_RED << "WTF?? LODnum";
            break;
        } else {
            cgNotice() << S_COLOR_RED << "WTF??";
            break;
        }
	}

	unsigned cageKey = meshToRender->loadedCageKey;
	unsigned numVerts = meshToRender->numVertices;
	unsigned numFrames = meshToRender->numFrames;
    unsigned numTris = meshToRender->triIndices.size();
	StaticCageCoordinate *vertCoords = meshToRender->vertexCoordinates;

    vec3_t color { 0.1f, 0.99f, 0.4f };
    unsigned currVec = cg.time % numVerts;

    vec3_t endPoint;

    VectorMA( origin, scale * limitsAtDirections[currVec], vertexMoveDirections[currVec], endPoint );

	if( v_showVectorsToLim.get() ) {
		effectsSystem->spawnTransientBeamEffect( origin, endPoint, {
				.material          = cgs.media.shaderLaser,
				.beamColorLifespan = {
						.initial  = {color[0], color[1], color[2]},
						.fadedIn  = {color[0], color[1], color[2]},
						.fadedOut = {color[0], color[1], color[2]},
				},
				.width             = 8.0f,
				.timeout           = 50u,
		} );
	}

	tri *triIndices = cageGeometry->triIndices.data();
    tri *meshTriIndices = meshToRender->triIndices.data();
	unsigned numCageTris = cageGeometry->triIndices.size();

    unsigned currFrame;
    if( v_frameToShow.get() < 0 ){
        currFrame = wsw::min( (unsigned) ((float) numFrames * lifetimeFrac), numFrames - 1 );
    } else {
        currFrame = v_frameToShow.get();
    }

	if( v_showMeshToRender.get() ){

		unsigned startVertIdx = currFrame * numVerts; // currFrame * numVerts;

		auto vertPosStorage = new vec3_t[numVerts];

		for( unsigned vertNum = 0; vertNum < numVerts; vertNum++ ) {
			unsigned vertIdx = startVertIdx + vertNum;
			unsigned triIdx = vertCoords[vertIdx].cageTriIdx;

			/* //
			if ( ( v_triIdx.get() < 0 && ( (vertNum + cg.time) % 20 == 1 ) ) || triIdx == v_triIdx.get()) {
				for (int idxNum = 0; idxNum < 3; idxNum++) {
					unsigned firstIdx = idxNum;
					unsigned secondIdx = (idxNum + 1) % 3;

					unsigned firstVertex = triIndices[triIdx][firstIdx];
					unsigned secondVertex = triIndices[triIdx][secondIdx];

					vec3_t firstPosition;
					vec3_t secondPosition;

					VectorMA(origin, scale * limitsAtDirections[firstVertex], vertexMoveDirections[firstVertex],
							 firstPosition);
					VectorMA(origin, scale * limitsAtDirections[secondVertex], vertexMoveDirections[secondVertex],
							 secondPosition);

					vec3_t colorB{1.0f, 0.2f, 0.2f};


					effectsSystem->spawnTransientBeamEffect(firstPosition, secondPosition, {
							.material          = cgs.media.shaderLaser,
							.beamColorLifespan = {
									.initial  = {colorB[0], colorB[1], colorB[2]},
									.fadedIn  = {colorB[0], colorB[1], colorB[2]},
									.fadedOut = {colorB[0], colorB[1], colorB[2]},
							},
							.width             = 8.0f,
							.timeout           = 50u,
					});
				}
			}
			*/ //
///
			vec2_t coords    = { vertCoords[vertIdx].coordsOnCageTri[0], vertCoords[vertIdx].coordsOnCageTri[1] };

			vec3_t vertPos;
			vec3_t moveDir;

			const unsigned cageVertIdx0 = triIndices[triIdx][0];
			const unsigned cageVertIdx1 = triIndices[triIdx][1];
			const unsigned cageVertIdx2 = triIndices[triIdx][2];

			const float coeff0 = 1 - ( coords[0] + coords[1] );
			const float coeff1 = coords[0];
			const float coeff2 = coords[1];

			VectorScale( vertexMoveDirections[cageVertIdx0], coeff0, moveDir );
			VectorMA( moveDir, coeff1, vertexMoveDirections[cageVertIdx1], moveDir );
			VectorMA( moveDir, coeff2, vertexMoveDirections[cageVertIdx2], moveDir );
			VectorNormalizeFast( moveDir );

			const float limit =
					limitsAtDirections[cageVertIdx0] * coeff0 +
					limitsAtDirections[cageVertIdx1] * coeff1 +
					limitsAtDirections[cageVertIdx2] * coeff2;

			const float offset = wsw::min( vertCoords[vertIdx].offset, limit ) * scale;

			VectorMA( origin, offset, moveDir, vertPos );

			VectorCopy( vertPos, &vertPosStorage[vertNum][0] );
/*
			effectsSystem->spawnTransientBeamEffect( origin, vertPos, {
					.material          = cgs.media.shaderLaser,
					.beamColorLifespan = {
							.initial  = {color[0], color[1], color[2]},
							.fadedIn  = {color[0], color[1], color[2]},
							.fadedOut = {color[0], color[1], color[2]},
					},
					.width             = 8.0f,
					.timeout           = 10u,
			} );*/
		}
		///

        for( unsigned triNum = 0; triNum < numTris; triNum++ ) {
            for ( int idxNum = 0; idxNum < 3; idxNum++ ) {

				if( ( triNum + cg.time ) % 4 == 1 ){
					unsigned firstIdx = idxNum;
					unsigned secondIdx = (idxNum + 1) % 3;

					unsigned firstVertex = meshTriIndices[triNum][firstIdx];
					unsigned secondVertex = meshTriIndices[triNum][secondIdx];

					vec3_t firstVertPos;
					vec3_t secondVertPos;

					vec3_t colorB{1.0f, 0.2f, 0.2f};

					VectorCopy( &vertPosStorage[firstVertex][0], firstVertPos );
					VectorCopy( &vertPosStorage[secondVertex][0], secondVertPos );

					effectsSystem->spawnTransientBeamEffect( firstVertPos, secondVertPos, {
							.material          = cgs.media.shaderLaser,
							.beamColorLifespan = {
									.initial  = {colorB[0], colorB[1], colorB[2]},
									.fadedIn  = {colorB[0], colorB[1], colorB[2]},
									.fadedOut = {colorB[0], colorB[1], colorB[2]},
							},
							.width             = 3.0f,
							.timeout           = 10u,
					} );
				}
            }
        }
		delete[] vertPosStorage;
	}
}

void SimulatedHullsSystem::vertexPosFromDynamicCage( const DynamicCageCoordinate vertCoord, float scale,
													 const tri *cageTriIndices, const vec3_t *vertexPositions,
													 const vec3_t *vertexNormals, const float *limitsAtDirections,
													 const float offsetFromLim, vec3_t outPos ) {
	unsigned triIdx = vertCoord.cageTriIdx;

	const vec2_t coords = { vertCoord.coordsOnCageTri[0], vertCoord.coordsOnCageTri[1] }; // 2: vertCoords

	vec3_t position;

	const unsigned cageVertIdx0 = cageTriIndices[triIdx][0];
	const unsigned cageVertIdx1 = cageTriIndices[triIdx][1];
	const unsigned cageVertIdx2 = cageTriIndices[triIdx][2];

	const float coeff0 = 1 - (coords[0] + coords[1]);
	const float coeff1 = coords[0];
	const float coeff2 = coords[1];

	VectorScale( vertexPositions[cageVertIdx0], coeff0, position );
	VectorMA( position, coeff1, vertexPositions[cageVertIdx1], position );
	VectorMA( position, coeff2, vertexPositions[cageVertIdx2], position );

	vec3_t moveDir;
	VectorScale( vertexNormals[cageVertIdx0], coeff0, moveDir );
	VectorMA( moveDir, coeff1, vertexNormals[cageVertIdx1], moveDir );
	VectorMA( moveDir, coeff2, vertexNormals[cageVertIdx2], moveDir );
	VectorNormalizeFast(moveDir);

	const float limit =
			limitsAtDirections[cageVertIdx0] * coeff0 +
			limitsAtDirections[cageVertIdx1] * coeff1 +
			limitsAtDirections[cageVertIdx2] * coeff2;

	const float maxOffset = wsw::max( 0.0f, limit - offsetFromLim );

	const float offset = wsw::min( vertCoord.offsetOnTri, maxOffset ) * scale;

	VectorMA( position, offset, moveDir, outPos );
}

void SimulatedHullsSystem::BaseDynamicCageHull::simulate( int64_t currTime, float timeDeltaSeconds,
														  PolyEffectsSystem *effectsSystem ) {
	BoundsBuilder cageBoundsBuilder;

	const DynamicCagedMesh *cagedMesh = sharedCageCagedMeshes[0];
	const unsigned cageKey = cagedMesh->loadedCageKey;

	const DynamicCage *cage = std::addressof( cg.simulatedHullsSystem.m_loadedDynamicCages[cageKey] );
	const Geometry *initialGeometry = &cage->cageInitialGeometry;
	const unsigned cageVerts = initialGeometry->vertexPositions.size();

	const float lifetimeFrac = ( currTime - spawnTime ) * Q_Rcp((float) lifetime );

	const unsigned numFrames = cage->numFrames;
	const unsigned currFrame = wsw::min((unsigned) ( numFrames * lifetimeFrac ), ( numFrames - 1 ) - 1 );

	const float frameTime = lifetime * Q_Rcp((float) numFrames );
	const float frameTimeSeconds = frameTime * 1e-3f;
	const float deltaFrame = timeDeltaSeconds * Q_Rcp( frameTimeSeconds );

	const float maxVelocity = cage->maxVelocityThisFrame[currFrame];
	vec3_t *vertexVelocities = cage->vertexVelocities + currFrame * cageVerts;

	const float rcpSqrtThree = 0.57735027;
	const vec3_t boundingOffsetDir = {rcpSqrtThree, rcpSqrtThree, rcpSqrtThree};
	const float maxOffsetFromPrevCage = maxOffsetFromCage + scale * maxVelocity * deltaFrame;
	vec3_t collisionMins, collisionMaxs;
	VectorMA( maxs, maxOffsetFromPrevCage, boundingOffsetDir,
			  collisionMaxs ); // will be consistently maxOffsetFromCage too large as mins/maxs already take this maxOffsetFromCage into account
	VectorMA( mins, -maxOffsetFromPrevCage, boundingOffsetDir, collisionMins );

	vec3_t color{0.99f, 0.4f, 0.1f};

	effectsSystem->spawnTransientBeamEffect( collisionMins, collisionMaxs, {
			.material          = cgs.media.shaderLaser,
			.beamColorLifespan = {
					.initial  = {color[0], color[1], color[2]},
					.fadedIn  = {color[0], color[1], color[2]},
					.fadedOut = {color[0], color[1], color[2]},
			},
			.width             = 4.0f,
			.timeout           = 10u,
	} );

	CMShapeList *shapeList = cg.simulatedHullsSystem.m_tmpShapeList;
	CM_BuildShapeList( cl.cms, shapeList, collisionMins, collisionMaxs, MASK_SOLID);
	CM_ClipShapeList( cl.cms, shapeList, shapeList, collisionMins, collisionMaxs );

	if ( CM_GetNumShapesInShapeList( shapeList ) == 0 ) {
		for ( unsigned vertNum = 0; vertNum < cageVerts; vertNum++ ) {
			vec3_t vertexVelocity;
			Matrix3_TransformVector( transformMatrix, vertexVelocities[vertNum], vertexVelocity );

			VectorMA( vertexPositions[vertNum], deltaFrame * scale, vertexVelocity,
					  vertexPositions[vertNum] );
		}
		cgNotice() << "no shapes";
	} else {
		trace_t trace;

		for ( unsigned vertNum = 0; vertNum < cageVerts; vertNum++ ) {
			vec3_t vertexVelocity;
			Matrix3_TransformVector( transformMatrix, vertexVelocities[vertNum], vertexVelocity );

			vec3_t vertexDisplacement;
			VectorScale( vertexVelocity, deltaFrame * scale, vertexDisplacement );

			vec3_t limitPoint;
			VectorAdd( vertexPositions[vertNum], vertexDisplacement, limitPoint );

			CM_ClipToShapeList( cl.cms, shapeList, &trace, vertexPositions[vertNum], limitPoint,
								vec3_origin, vec3_origin, MASK_SOLID);
			VectorMA( vertexPositions[vertNum], trace.fraction, vertexDisplacement,
					  vertexPositions[vertNum] );
			const float freeFraction = trace.fraction;
			if ( freeFraction < 1.0f - 1e-1f ) { // in the case that a significant part of the movement is obstructed
				const float slideFraction = 1.0f - freeFraction;
				// vectors perpendicular and parallel to the obstacle
				vec3_t perpendicularDisplacement;
				vec3_t parallelDisplacement;

				const float lengthAlongNormal = DotProduct( vertexDisplacement, trace.plane.normal );
				VectorScale( trace.plane.normal, lengthAlongNormal, parallelDisplacement );
				VectorSubtract( vertexDisplacement, parallelDisplacement, perpendicularDisplacement );

				vec3_t slideDisplacement;
				VectorScale( perpendicularDisplacement, slideFraction, slideDisplacement );

				// is this branch worth it? regular simulated hulls use:
				// >1^2 but this is wrong as this could be a velocity of 1000ups at 1000fps (1 unit per frame)
				//if( VectorLengthSquared( slideDisplacement ) > 1e-3f * 1e-3f ){
				VectorAdd( vertexPositions[vertNum], slideDisplacement, limitPoint );

				CM_ClipToShapeList( cl.cms, shapeList, &trace, vertexPositions[vertNum], limitPoint,
									vec3_origin, vec3_origin, MASK_SOLID);
				VectorMA( vertexPositions[vertNum], trace.fraction, slideDisplacement,
						  vertexPositions[vertNum] );
				//}
			}
		}
	}

	calculateNormals( initialGeometry->triIndices, cageVerts, vertexPositions, vertexNormals );

	const float maxOffsetDist = maxOffsetSpeed * timeDeltaSeconds;
	if ( CM_GetNumShapesInShapeList( shapeList ) == 0 ) {
		for ( size_t i = 0; i < cageVerts; i++ ) {
			// Limits at each direction just match the maximum offset from the cage in this cage
			const float limitDiff = maxOffsetFromCage - limitsAtDirections[i];
			// the limits can only expand in this case, never decrease as they now tend towards their maximum
			limitsAtDirections[i] += wsw::min( limitDiff, maxOffsetDist );

			vec3_t offsetPos;
			VectorMA( vertexPositions[i], maxOffsetFromCage, vertexNormals[i], offsetPos );
			cageBoundsBuilder.addPoint( offsetPos );
		}
	} else {
		trace_t trace;
		for ( size_t i = 0; i < cageVerts; i++ ) {
			vec3_t limitPoint;
			VectorMA( vertexPositions[i], maxOffsetFromCage, vertexNormals[i], limitPoint );
			CM_ClipToShapeList( cl.cms, shapeList, &trace, vertexPositions[i],
								limitPoint, vec3_origin, vec3_origin, MASK_SOLID);
			const float limitDiff = maxOffsetFromCage * trace.fraction - limitsAtDirections[i];
			limitsAtDirections[i] += wsw::clamp( limitDiff, -maxOffsetDist, maxOffsetDist );

			vec3_t offsetPos;
			VectorMA( vertexPositions[i], limitsAtDirections[i], vertexNormals[i], offsetPos );
			cageBoundsBuilder.addPoint( offsetPos );
		}
	}
	cageBoundsBuilder.storeTo( mins, maxs );

	const unsigned numCageTris = initialGeometry->triIndices.size();
	tri *cageTris = initialGeometry->triIndices.data();

	vec3_t colorB{0.1f, 0.4f, 0.99f};
	vec3_t colorC{0.99f, 0.2f, 0.1f};
	vec3_t colorD{0.2f, 0.99f, 0.1f};

	for ( unsigned triNum = 0; triNum < numCageTris; triNum++ ) {
		for ( int idxNum = 0; idxNum < 3; idxNum++ ) {
			unsigned firstIdx = idxNum;
			unsigned secondIdx = ( idxNum + 1 ) % 3;

			unsigned firstVertex = cageTris[triNum][firstIdx];
			unsigned secondVertex = cageTris[triNum][secondIdx];

			vec3_t firstPosition;
			vec3_t secondPosition;

			VectorCopy( vertexPositions[firstVertex], firstPosition );
			VectorCopy( vertexPositions[secondVertex], secondPosition );

			vec3_t firstOffsetPos;
			vec3_t secondOffsetPos;

			VectorMA( vertexPositions[firstVertex], limitsAtDirections[firstVertex], vertexNormals[firstVertex],
					  firstOffsetPos );
			VectorMA( vertexPositions[secondVertex], limitsAtDirections[secondVertex], vertexNormals[secondVertex],
					  secondOffsetPos );

			if ((( v_triIdx.get() < 0 ) && (( triNum + cg.time ) % 4 == 1 )) || triNum == v_triIdx.get()) {
				effectsSystem->spawnTransientBeamEffect( firstPosition, secondPosition, {
						.material          = cgs.media.shaderLaser,
						.beamColorLifespan = {
								.initial  = {colorB[0], colorB[1], colorB[2]},
								.fadedIn  = {colorB[0], colorB[1], colorB[2]},
								.fadedOut = {colorB[0], colorB[1], colorB[2]},
						},
						.width             = 4.0f,
						.timeout           = 10u
				} );

				effectsSystem->spawnTransientBeamEffect( firstOffsetPos, secondOffsetPos, {
						.material          = cgs.media.shaderLaser,
						.beamColorLifespan = {
								.initial  = {colorC[0], colorC[1], colorC[2]},
								.fadedIn  = {colorC[0], colorC[1], colorC[2]},
								.fadedOut = {colorC[0], colorC[1], colorC[2]},
						},
						.width             = 4.0f,
						.timeout           = 10u,
				} );

				effectsSystem->spawnTransientBeamEffect( firstOffsetPos, secondOffsetPos, {
						.material          = cgs.media.shaderLaser,
						.beamColorLifespan = {
								.initial  = {colorC[0], colorC[1], colorC[2]},
								.fadedIn  = {colorC[0], colorC[1], colorC[2]},
								.fadedOut = {colorC[0], colorC[1], colorC[2]},
						},
						.width             = 4.0f,
						.timeout           = 10u,
				} );
			}
		}
		SimulatedHullsSystem::DynamicCageCoordinate posOnTriCoord = {
				.cageTriIdx = triNum,
				.coordsOnCageTri = {0.33f, 0.33f},
				.offsetOnTri = 0.0f,
		};
		SimulatedHullsSystem::DynamicCageCoordinate offsetPosCoord = {
				.cageTriIdx = triNum,
				.coordsOnCageTri = {0.33f, 0.33f},
				.offsetOnTri = maxOffsetFromCage,
		};

		vec3_t posOnTri;
		vec3_t offsetPos;

		vertexPosFromDynamicCage( posOnTriCoord, 1.0f, cageTris, vertexPositions, vertexNormals, limitsAtDirections, 0.0f,
								  posOnTri );
		vertexPosFromDynamicCage( offsetPosCoord, 1.0f, cageTris, vertexPositions, vertexNormals, limitsAtDirections, 0.0f,
								  offsetPos );

//		if( ( ( v_triIdx.get() < 0 ) && ( ( triNum + cg.time ) % 4 == 1 ) ) || triNum == v_triIdx.get() ) {
//			effectsSystem->spawnTransientBeamEffect( posOnTri, offsetPos, {
//					.material          = cgs.media.shaderLaser,
//					.beamColorLifespan = {
//							.initial  = {colorD[0], colorD[1], colorD[2]},
//							.fadedIn  = {colorD[0], colorD[1], colorD[2]},
//							.fadedOut = {colorD[0], colorD[1], colorD[2]},
//					},
//					.width             = 3.0f,
//					.timeout           = 10u
//			} );
//		}
	}

	const unsigned meshVerts = cagedMesh->numVertices;
	const unsigned numMeshTris = cagedMesh->triIndices.size();
	const tri *meshTris = cagedMesh->triIndices.data();
	DynamicCageCoordinate *meshVertCoords = cagedMesh->vertexCoordinates;

	unsigned startIdx = currFrame * meshVerts;

	for ( unsigned triNum = 0; triNum < numMeshTris; triNum++ ) {
		for ( int idxNum = 0; idxNum < 3; idxNum++ ) {
			unsigned firstIdx = idxNum;
			unsigned secondIdx = ( idxNum + 1 ) % 3;

			unsigned firstVertex = meshTris[triNum][firstIdx] + startIdx;
			unsigned secondVertex = meshTris[triNum][secondIdx] + startIdx;

			vec3_t firstPosition;
			vec3_t secondPosition;

			vertexPosFromDynamicCage( meshVertCoords[firstVertex], scale, cageTris, vertexPositions, vertexNormals, limitsAtDirections, 0.0f,
									  firstPosition );
			vertexPosFromDynamicCage( meshVertCoords[secondVertex], scale, cageTris, vertexPositions, vertexNormals, limitsAtDirections, 0.0f,
									  secondPosition );

			if ((( v_triIdx.get() < 0 ) && (( triNum + cg.time ) % 4 == 1 )) || triNum == v_triIdx.get()) {
				effectsSystem->spawnTransientBeamEffect( firstPosition, secondPosition, {
						.material          = cgs.media.shaderLaser,
						.beamColorLifespan = {
								.initial  = {colorD[0], colorD[1], colorD[2]},
								.fadedIn  = {colorD[0], colorD[1], colorD[2]},
								.fadedOut = {colorD[0], colorD[1], colorD[2]},
						},
						.width             = 4.0f,
						.timeout           = 10u
				} );

			}
		}
	}
}

auto SimulatedHullsSystem::computePrevKeyframeIndex( unsigned startFromIndex, int64_t currTime,
													 int64_t spawnTime, unsigned effectDuration,
													 std::span<const OffsetKeyframe> offsetKeyframeSet ) -> unsigned {
	const auto currLifetimeFraction = (float)( currTime - spawnTime ) * Q_Rcp( (float)effectDuration );
	unsigned setSize = offsetKeyframeSet.size();

	// Assume that startFromIndex is "good" even if the conditions in the loop won't be held for it
	unsigned currIndex = startFromIndex, lastGoodIndex = startFromIndex;
	for(;; ) {
		if( currIndex == ( setSize - 1 ) ) { // it should never reach the last keyframe as we always need a next one for interpolation
			break;
		}
		if( offsetKeyframeSet[currIndex].lifetimeFraction > currLifetimeFraction ) {
			break;
		}
		lastGoodIndex = currIndex;
		currIndex++;
	}

	return lastGoodIndex;
}

auto SimulatedHullsSystem::computeCurrTimelineNodeIndex( unsigned startFromIndex, int64_t currTime,
														 int64_t spawnTime, unsigned effectDuration,
														 std::span<const ColorChangeTimelineNode> timeline )
	-> unsigned {
	// Sanity checks
	assert( effectDuration && effectDuration < std::numeric_limits<uint16_t>::max() );
	assert( currTime - spawnTime >= 0 && currTime - spawnTime < std::numeric_limits<uint16_t>::max() );

	assert( startFromIndex < timeline.size() );

	const auto currLifetimeFraction = (float)( currTime - spawnTime ) * Q_Rcp( (float)effectDuration );
	assert( currLifetimeFraction >= 0.0f && currLifetimeFraction <= 1.001f );

	// Assume that startFromIndex is "good" even if the conditions in the loop won't be held for it
	unsigned currIndex = startFromIndex, lastGoodIndex = startFromIndex;
	for(;; ) {
		if( currIndex == timeline.size() ) {
			break;
		}
		if( timeline[currIndex].activateAtLifetimeFraction > currLifetimeFraction ) {
			break;
		}
		lastGoodIndex = currIndex;
		currIndex++;
	}

	return lastGoodIndex;
}

enum ColorChangeFlags : unsigned { MayDrop = 0x1, MayReplace = 0x2, MayIncreaseOpacity = 0x4, AssumePow2Palette = 0x8 };

template <unsigned Flags>
static void changeColors( std::span<byte_vec4_t> colorsSpan, wsw::RandomGenerator *__restrict rng,
						  [[maybe_unused]] std::span<const byte_vec4_t> replacementPalette,
						  [[maybe_unused]] float dropChance, [[maybe_unused]] float replacementChance ) {

	byte_vec4_t *const __restrict colors = colorsSpan.data();
	const unsigned numColors             = colorsSpan.size();

	[[maybe_unused]] unsigned paletteIndexMask = 0;
	if constexpr( Flags & AssumePow2Palette ) {
		paletteIndexMask = (unsigned)( replacementPalette.size() - 1 );
	}

	unsigned i = 0;
	do {
		// Don't process elements that became void
		if( colors[i][3] != 0 ) [[likely]] {
			[[maybe_unused]] bool dropped = false;
			if constexpr( Flags & MayDrop ) {
				if( rng->tryWithChance( dropChance ) ) [[unlikely]] {
					colors[i][3] = 0;
					dropped = true;
				}
			}
			if constexpr( Flags & MayReplace ) {
				if( !dropped ) {
					if( rng->tryWithChance( replacementChance ) ) [[unlikely]] {
						const uint8_t *chosenColor;
						if constexpr( Flags & AssumePow2Palette ) {
							chosenColor = replacementPalette[rng->next() & paletteIndexMask];
						} else {
							chosenColor = replacementPalette[rng->nextBounded( replacementPalette.size() ) ];
						}
						auto *const existingColor = colors[i];
						if constexpr( Flags & MayIncreaseOpacity ) {
							Vector4Copy( chosenColor, existingColor );
						} else {
							// Branching could be perfectly avoided in this case
							// (replacement is actually not that "unlikely" for some parts of hull lifetimes
							// so slightly optimizing it makes sense).
							const uint8_t *srcColorsToSelect[2] { existingColor, chosenColor };
							const uint8_t *selectedColor = srcColorsToSelect[chosenColor[3] <= existingColor[3]];
							Vector4Copy( selectedColor, existingColor );
						}
					}
				}
			}
		}
	} while( ++i < numColors );
}

static const struct ColorChangeFnTableHolder {
	using Fn = void (*)( std::span<byte_vec4_t>, wsw::RandomGenerator *, std::span<const byte_vec4_t>, float, float );
	Fn table[16] {};

#define ADD_FN_FOR_FLAGS_TO_TABLE( Flags ) do { table[Flags] = &::changeColors<Flags>; } while( 0 )
	ColorChangeFnTableHolder() noexcept {
		// Add a reference to a template instantiation for each feasible combination of flags.
		// (Don't force generating code for unfeasible ones
		// by a brute-force instantiation of templates for all 16 values).
		ADD_FN_FOR_FLAGS_TO_TABLE( MayDrop | MayReplace | MayIncreaseOpacity | AssumePow2Palette );
		ADD_FN_FOR_FLAGS_TO_TABLE( MayDrop | MayReplace | MayIncreaseOpacity );
		ADD_FN_FOR_FLAGS_TO_TABLE( MayDrop | MayReplace | AssumePow2Palette );
		ADD_FN_FOR_FLAGS_TO_TABLE( MayDrop | MayReplace );
		ADD_FN_FOR_FLAGS_TO_TABLE( MayReplace | MayIncreaseOpacity | AssumePow2Palette );
		ADD_FN_FOR_FLAGS_TO_TABLE( MayReplace | MayIncreaseOpacity );
		ADD_FN_FOR_FLAGS_TO_TABLE( MayReplace | AssumePow2Palette );
		ADD_FN_FOR_FLAGS_TO_TABLE( MayReplace );
		ADD_FN_FOR_FLAGS_TO_TABLE( MayDrop );
	}
#undef ADD_FN_FOR_FLAGS_TO_TABLE
} colorChangeFnTableHolder;

bool SimulatedHullsSystem::processColorChange( int64_t currTime,
											   int64_t spawnTime,
											   unsigned effectDuration,
											   std::span<const ColorChangeTimelineNode> timeline,
											   std::span<byte_vec4_t> colorsSpan,
											   ColorChangeState *__restrict state,
											   wsw::RandomGenerator *__restrict rng ) {
	// This helps to handle non-color-changing hulls in a least-effort fashion
	if( state->lastNodeIndex >= timeline.size() ) [[unlikely]] {
		return false;
	}

	// Compute the current node in an immediate mode. This is inexpensive for a realistic input.
	state->lastNodeIndex = computeCurrTimelineNodeIndex( state->lastNodeIndex, currTime, spawnTime,
														 effectDuration, timeline );

	constexpr int64_t colorChangeIntervalMillis = 15;

	const ColorChangeTimelineNode &currNode = timeline[state->lastNodeIndex];
	if( state->lastColorChangeAt + colorChangeIntervalMillis > currTime ) [[likely]] {
		return false;
	}

	// Do nothing during the first frame
	if( state->lastColorChangeAt <= 0 ) [[unlikely]] {
		state->lastColorChangeAt = currTime;
		return false;
	}

	const auto timeDeltaSeconds = 1e-3f * (float)( currTime - state->lastColorChangeAt );
	assert( timeDeltaSeconds >= 1e-3f * (float)colorChangeIntervalMillis );
	state->lastColorChangeAt = currTime;

	float currSegmentLifetimePercentage;
	if( state->lastNodeIndex + 1 < timeline.size() ) {
		const ColorChangeTimelineNode &nextNode = timeline[state->lastNodeIndex + 1];
		currSegmentLifetimePercentage = nextNode.activateAtLifetimeFraction - currNode.activateAtLifetimeFraction;
	} else {
		currSegmentLifetimePercentage = 1.0f - currNode.activateAtLifetimeFraction;
	}

	assert( currSegmentLifetimePercentage >= 0.0f && currSegmentLifetimePercentage <= 1.0f );
	assert( currSegmentLifetimePercentage > 1e-3f );

	// Reconstruct duration of this timeline segment
	const float currSegmentDurationSeconds = currSegmentLifetimePercentage * ( 1e-3f * (float)effectDuration );

	float dropChance, replacementChance;
	// Protect from going out of value bounds (e.g. after a freeze due to external reasons)
	if( timeDeltaSeconds < currSegmentDurationSeconds ) [[likely]] {
		// Compute how much this color change frame takes of the segment
		// TODO: Don't convert to seconds prior to the division?
		const auto currFrameFractionOfTheSegment = timeDeltaSeconds * Q_Rcp( currSegmentDurationSeconds );
		assert( currFrameFractionOfTheSegment > 0.0f && currFrameFractionOfTheSegment <= 1.0f );
		// Convert accumulative chances for the entire segment to chances for this color change frame.
		dropChance        = currNode.sumOfDropChanceForThisSegment * currFrameFractionOfTheSegment;
		replacementChance = currNode.sumOfReplacementChanceForThisSegment * currFrameFractionOfTheSegment;
	} else {
		dropChance        = currNode.sumOfDropChanceForThisSegment;
		replacementChance = currNode.sumOfReplacementChanceForThisSegment;
	}

	// Don't let the chance drop to zero if the specified integral chance is non-zero.
	// Don't let it exceed 1.0 as well during rapid color changes.
	constexpr float minDropChance = 1e-3f, minReplacementChance = 1e-3f;
	if( currNode.sumOfDropChanceForThisSegment > 0.0f ) {
		dropChance = wsw::clamp( dropChance, minDropChance, 1.0f );
	}
	if( currNode.sumOfReplacementChanceForThisSegment > 0.0f ) {
		replacementChance = wsw::clamp( replacementChance, minReplacementChance, 1.0f );
	}

	const auto palette    = currNode.replacementPalette;
	const bool mayDrop    = dropChance > 0.0f;
	const bool mayReplace = replacementChance > 0.0f && !palette.empty();

	if( mayDrop | mayReplace ) {
		const bool isPalettePow2  = ( palette.size() & ( palette.size() - 1u ) ) == 0;
		const unsigned dropBit    = ( mayDrop ) ? MayDrop : 0;
		const unsigned replaceBit = ( mayReplace ) ? MayReplace : 0;
		const unsigned opacityBit = ( currNode.allowIncreasingOpacity ) ? MayIncreaseOpacity : 0;
		const unsigned pow2Bit    = ( mayReplace & isPalettePow2 ) ? AssumePow2Palette : 0;
		const unsigned fnIndex    = dropBit | replaceBit | opacityBit | pow2Bit;

		// Call the respective template specialization for flags
		::colorChangeFnTableHolder.table[fnIndex]( colorsSpan, rng, palette, dropChance, replacementChance );
	}

	return true;
}

class MeshTesselationHelper {
public:
	vec4_t *m_tmpTessPositions { nullptr };
	vec4_t *m_tmpTessFloatColors { nullptr };
	int8_t *m_tessNeighboursCount { nullptr };
	vec4_t *m_tessPositions { nullptr };
	byte_vec4_t *m_tessByteColors { nullptr };
	uint16_t *m_neighbourLessVertices { nullptr };
	bool *m_isVertexNeighbourLess { nullptr };

	std::unique_ptr<uint8_t[]> m_allocationBuffer;
	unsigned m_lastNumNextLevelVertices { 0 };
	unsigned m_lastAllocationSize { 0 };

	template <bool SmoothColors>
	void exec( const SimulatedHullsSystem::HullDynamicMesh *mesh, const byte_vec4_t *overrideColors, unsigned numSimulatedVertices,
			   unsigned numNextLevelVertices, const uint16_t nextLevelNeighbours[][5] );
private:
	static constexpr unsigned kAlignment = 16;

	void setupBuffers( unsigned numSimulatedVertices, unsigned numNextLevelVertices );

	template <bool SmoothColors>
	void runPassOverOriginalVertices( const SimulatedHullsSystem::HullDynamicMesh *mesh, const byte_vec4_t *overrideColors,
									  unsigned numSimulatedVertices, const uint16_t nextLevelNeighbours[][5] );
	template <bool SmoothColors>
	[[nodiscard]]
	auto collectNeighbourLessVertices( unsigned numSimulatedVertices, unsigned numNextLevelVertices ) -> unsigned;
	template <bool SmoothColors>
	void processNeighbourLessVertices( unsigned numNeighbourLessVertices, const uint16_t nextLevelNeighbours[][5] );
	template <bool SmoothColors>
	void runSmoothVerticesPass( unsigned numNextLevelVertices, const uint16_t nextLevelNeighbours[][5] );
};

template <bool SmoothColors>
void MeshTesselationHelper::exec( const SimulatedHullsSystem::HullDynamicMesh *mesh, const byte_vec4_t *overrideColors,
								  unsigned numSimulatedVertices, unsigned numNextLevelVertices,
								  const uint16_t nextLevelNeighbours[][5] ) {
	setupBuffers( numSimulatedVertices, numNextLevelVertices );

	runPassOverOriginalVertices<SmoothColors>( mesh, overrideColors, numSimulatedVertices, nextLevelNeighbours );
	unsigned numNeighbourLessVertices = collectNeighbourLessVertices<SmoothColors>( numSimulatedVertices, numNextLevelVertices );
	processNeighbourLessVertices<SmoothColors>( numNeighbourLessVertices, nextLevelNeighbours );
	runSmoothVerticesPass<SmoothColors>( numNextLevelVertices, nextLevelNeighbours );
}

void MeshTesselationHelper::setupBuffers( unsigned numSimulatedVertices, unsigned numNextLevelVertices ) {
	if( m_lastNumNextLevelVertices != numNextLevelVertices ) {
		// Compute offsets from the base ptr
		wsw::MemSpecBuilder memSpecBuilder( wsw::MemSpecBuilder::initiallyEmpty() );

		const auto tmpTessPositionsSpec       = memSpecBuilder.addAligned<vec4_t>( numNextLevelVertices, kAlignment );
		const auto tmpTessFloatColorsSpec     = memSpecBuilder.addAligned<vec4_t>( numNextLevelVertices, kAlignment );
		const auto tessPositionsSpec          = memSpecBuilder.addAligned<vec4_t>( numNextLevelVertices, kAlignment );
		const auto tmpTessNeighboursCountSpec = memSpecBuilder.add<int8_t>( numNextLevelVertices );
		const auto tessByteColorsSpec         = memSpecBuilder.add<byte_vec4_t>( numNextLevelVertices );
		const auto neighbourLessVerticesSpec  = memSpecBuilder.add<uint16_t>( numNextLevelVertices );
		const auto isNextVertexNeighbourLess  = memSpecBuilder.add<bool>( numNextLevelVertices );

		if ( m_lastAllocationSize < memSpecBuilder.sizeSoFar() ) [[unlikely]] {
			m_lastAllocationSize = memSpecBuilder.sizeSoFar();
			m_allocationBuffer   = std::make_unique<uint8_t[]>( m_lastAllocationSize );
		}

		void *const basePtr     = m_allocationBuffer.get();
		m_tmpTessPositions      = tmpTessPositionsSpec.get( basePtr );
		m_tmpTessFloatColors    = tmpTessFloatColorsSpec.get( basePtr );
		m_tessNeighboursCount   = tmpTessNeighboursCountSpec.get( basePtr );
		m_tessPositions         = tessPositionsSpec.get( basePtr );
		m_tessByteColors        = tessByteColorsSpec.get( basePtr );
		m_neighbourLessVertices = neighbourLessVerticesSpec.get( basePtr );
		m_isVertexNeighbourLess = isNextVertexNeighbourLess.get( basePtr );

		m_lastNumNextLevelVertices = numNextLevelVertices;
	}

	assert( numSimulatedVertices && numNextLevelVertices && numSimulatedVertices < numNextLevelVertices );
	const unsigned numAddedVertices = numNextLevelVertices - numSimulatedVertices;

	std::memset( m_tessPositions, 0, sizeof( m_tessPositions[0] ) * numNextLevelVertices );
	std::memset( m_tessByteColors, 0, sizeof( m_tessByteColors[0] ) * numNextLevelVertices );
	std::memset( m_isVertexNeighbourLess, 0, sizeof( m_isVertexNeighbourLess[0] ) * numNextLevelVertices );

	std::memset( m_tmpTessPositions + numSimulatedVertices, 0, sizeof( m_tmpTessPositions[0] ) * numAddedVertices );
	std::memset( m_tmpTessFloatColors + numSimulatedVertices, 0, sizeof( m_tmpTessFloatColors[0] ) * numAddedVertices );
	std::memset( m_tessNeighboursCount + numSimulatedVertices, 0, sizeof( m_tessNeighboursCount[0] ) * numAddedVertices );
}

template <bool SmoothColors>
void MeshTesselationHelper::runPassOverOriginalVertices( const SimulatedHullsSystem::HullDynamicMesh *mesh,
														 const byte_vec4_t *overrideColors,
														 unsigned numSimulatedVertices,
														 const uint16_t nextLevelNeighbours[][5] ) {
	vec4_t *const __restrict tmpTessFloatColors  = std::assume_aligned<kAlignment>( m_tmpTessFloatColors );
	vec4_t *const __restrict tmpTessPositions    = std::assume_aligned<kAlignment>( m_tmpTessPositions );
	const byte_vec4_t *const __restrict colors   = overrideColors ? overrideColors : mesh->m_shared->simulatedColors;
	byte_vec4_t *const __restrict tessByteColors = m_tessByteColors;
	int8_t *const __restrict tessNeighboursCount = m_tessNeighboursCount;

	// For each vertex in the original mesh
	for( unsigned vertexIndex = 0; vertexIndex < numSimulatedVertices; ++vertexIndex ) {
		const auto byteColor  = colors[vertexIndex];
		const float *position = mesh->m_shared->simulatedPositions[vertexIndex];

		if constexpr( SmoothColors ) {
			// Write the color to the accum buffer for smoothing it later
			Vector4Copy( byteColor, tmpTessFloatColors[vertexIndex] );
		} else {
			// Write the color directly to the resulting color buffer
			Vector4Copy( byteColor, tessByteColors[vertexIndex] );
		}

		// Copy for the further smooth pass
		Vector4Copy( position, tmpTessPositions[vertexIndex] );

		// For each neighbour of this vertex in the tesselated mesh
		for( const unsigned neighbourIndex: nextLevelNeighbours[vertexIndex] ) {
			if( neighbourIndex >= numSimulatedVertices ) {
				VectorAdd( tmpTessPositions[neighbourIndex], position, tmpTessPositions[neighbourIndex] );
				// Add integer values as is to the float accumulation buffer
				Vector4Add( tmpTessFloatColors[neighbourIndex], byteColor, tmpTessFloatColors[neighbourIndex] );
				tessNeighboursCount[neighbourIndex]++;
			}
		}
	}
}

template <bool SmoothColors>
auto MeshTesselationHelper::collectNeighbourLessVertices( unsigned numSimulatedVertices,
														  unsigned numNextLevelVertices ) -> unsigned {
	assert( numSimulatedVertices && numNextLevelVertices && numSimulatedVertices < numNextLevelVertices );

	vec4_t *const __restrict tmpTessPositions        = std::assume_aligned<kAlignment>( m_tmpTessPositions );
	vec4_t *const __restrict  tmpTessFloatColors     = std::assume_aligned<kAlignment>( m_tmpTessFloatColors );
	int8_t *const __restrict tessNeighboursCount     = m_tessNeighboursCount;
	uint16_t *const __restrict neighbourLessVertices = m_neighbourLessVertices;
	bool *const __restrict isVertexNeighbourLess     = m_isVertexNeighbourLess;
	byte_vec4_t *const __restrict tessByteColors     = m_tessByteColors;

	unsigned numNeighbourLessVertices = 0;
	for( unsigned vertexIndex = numSimulatedVertices; vertexIndex < numNextLevelVertices; ++vertexIndex ) {
		// Wtf? how do such vertices exist?
		if( !tessNeighboursCount[vertexIndex] ) [[unlikely]] {
			neighbourLessVertices[numNeighbourLessVertices++] = vertexIndex;
			isVertexNeighbourLess[vertexIndex] = true;
			continue;
		}

		const float scale = Q_Rcp( (float)tessNeighboursCount[vertexIndex] );
		VectorScale( tmpTessPositions[vertexIndex], scale, tmpTessPositions[vertexIndex] );
		tmpTessPositions[vertexIndex][3] = 1.0f;

		if constexpr( SmoothColors ) {
			// Just scale by the averaging multiplier
			Vector4Scale( tmpTessFloatColors[vertexIndex], scale, tmpTessFloatColors[vertexIndex] );
		} else {
			// Write the vertex color directly to the resulting color buffer
			Vector4Scale( tmpTessFloatColors[vertexIndex], scale, tessByteColors[vertexIndex] );
		}
	}

	return numNeighbourLessVertices;
}

template <bool SmoothColors>
void MeshTesselationHelper::processNeighbourLessVertices( unsigned numNeighbourLessVertices,
														  const uint16_t nextLevelNeighbours[][5] ) {
	vec4_t *const __restrict tmpTessFloatColors            = std::assume_aligned<kAlignment>( m_tmpTessFloatColors );
	const uint16_t *const __restrict neighbourLessVertices = m_neighbourLessVertices;
	const bool *const __restrict isVertexNeighbourLess     = m_isVertexNeighbourLess;
	byte_vec4_t *const __restrict tessByteColors           = m_tessByteColors;

	// Hack for neighbour-less vertices: apply a gathering pass
	// (the opposite to what we do for each vertex in the original mesh)
	for( unsigned i = 0; i < numNeighbourLessVertices; ++i ) {
		const unsigned vertexIndex = neighbourLessVertices[i];
		alignas( 16 ) vec4_t accumulatedColor { 0.0f, 0.0f, 0.0f, 0.0f };
		unsigned numAccumulatedColors = 0;
		for( unsigned neighbourIndex: nextLevelNeighbours[vertexIndex] ) {
			if( !isVertexNeighbourLess[neighbourIndex] ) [[likely]] {
				numAccumulatedColors++;
				if constexpr( SmoothColors ) {
					const float *const __restrict neighbourColor = tmpTessFloatColors[neighbourIndex];
					Vector4Add( neighbourColor, accumulatedColor, accumulatedColor );
				} else {
					const uint8_t *const __restrict neighbourColor = tessByteColors[neighbourIndex];
					Vector4Add( neighbourColor, accumulatedColor, accumulatedColor );
				}
			}
		}
		if( numAccumulatedColors ) [[likely]] {
			const float scale = Q_Rcp( (float)numAccumulatedColors );
			if constexpr( SmoothColors ) {
				Vector4Scale( accumulatedColor, scale, tmpTessFloatColors[vertexIndex] );
			} else {
				Vector4Scale( accumulatedColor, scale, tessByteColors[vertexIndex] );
			}
		}
	}
}

template <bool SmoothColors>
void MeshTesselationHelper::runSmoothVerticesPass( unsigned numNextLevelVertices,
												   const uint16_t nextLevelNeighbours[][5] ) {
	vec4_t *const __restrict tmpTessPositions    = std::assume_aligned<kAlignment>( m_tmpTessPositions );
	vec4_t *const __restrict tmpTessFloatColors  = std::assume_aligned<kAlignment>( m_tmpTessFloatColors );
	vec4_t *const __restrict tessPositions       = std::assume_aligned<kAlignment>( m_tessPositions );
	byte_vec4_t *const __restrict tessByteColors = m_tessByteColors;

	// Each icosphere vertex has 5 neighbours (we don't make a distinction between old and added ones for this pass)
	constexpr float icoAvgScale = 1.0f / 5.0f;
	constexpr float smoothFrac  = 0.5f;

	// Apply the smooth pass
	if constexpr( SmoothColors ) {
		for( unsigned vertexIndex = 0; vertexIndex < numNextLevelVertices; ++vertexIndex ) {
			alignas( 16 ) vec4_t sumOfNeighbourPositions { 0.0f, 0.0f, 0.0f, 0.0f };
			alignas( 16 ) vec4_t sumOfNeighbourColors { 0.0f, 0.0f, 0.0f, 0.0f };

			for( const unsigned neighbourIndex: nextLevelNeighbours[vertexIndex] ) {
				Vector4Add( tmpTessPositions[neighbourIndex], sumOfNeighbourPositions, sumOfNeighbourPositions );
				Vector4Add( tmpTessFloatColors[neighbourIndex], sumOfNeighbourColors, sumOfNeighbourColors );
			}

			Vector4Scale( sumOfNeighbourPositions, icoAvgScale, sumOfNeighbourPositions );

			// Write the average color
			Vector4Scale( sumOfNeighbourColors, icoAvgScale, tessByteColors[vertexIndex] );

			// Write combined positions
			Vector4Lerp( tmpTessPositions[vertexIndex], smoothFrac, sumOfNeighbourPositions, tessPositions[vertexIndex] );
		}
	} else {
		for( unsigned vertexIndex = 0; vertexIndex < numNextLevelVertices; ++vertexIndex ) {
			alignas( 16 ) vec4_t sumOfNeighbourPositions { 0.0f, 0.0f, 0.0f, 0.0f };

			for( const unsigned neighbourIndex: nextLevelNeighbours[vertexIndex] ) {
				Vector4Add( tmpTessPositions[neighbourIndex], sumOfNeighbourPositions, sumOfNeighbourPositions );
			}

			Vector4Scale( sumOfNeighbourPositions, icoAvgScale, sumOfNeighbourPositions );

			// Write combined positions
			Vector4Lerp( tmpTessPositions[vertexIndex], smoothFrac, sumOfNeighbourPositions, tessPositions[vertexIndex] );
		}
	}
}

static MeshTesselationHelper meshTesselationHelper;

template <SimulatedHullsSystem::ViewDotFade Fade>
[[nodiscard]]
static wsw_forceinline auto calcAlphaFracForViewDirDotNormal( const float *__restrict viewDir,
																	 const float *__restrict normal ) -> float {
	assert( std::fabs( VectorLengthFast( viewDir ) - 1.0f ) < 0.001f );
	assert( std::fabs( VectorLengthFast( normal ) - 1.0f ) < 0.001f );

	const float absDot = std::fabs( DotProduct( viewDir, normal ) );
	if constexpr( Fade == SimulatedHullsSystem::ViewDotFade::FadeOutContour ) {
		return absDot;
	} else if constexpr( Fade == SimulatedHullsSystem::ViewDotFade::FadeOutCenterLinear ) {
		return 1.0f - absDot;
	} else if constexpr( Fade == SimulatedHullsSystem::ViewDotFade::FadeOutCenterQuadratic ) {
		const float frac = ( 1.0f - absDot );
		return frac * frac;
	} else if constexpr( Fade == SimulatedHullsSystem::ViewDotFade::FadeOutCenterCubic ) {
		const float frac = ( 1.0f - absDot );
		return frac * frac * frac;
	} else {
		return 1.0f;
	}
}

template <SimulatedHullsSystem::ZFade ZFade>
[[nodiscard]]
static wsw_forceinline auto calcAlphaFracForDeltaZ( float z, float minZ, float rcpDeltaZ ) -> float {
	if constexpr( ZFade == SimulatedHullsSystem::ZFade::FadeOutBottom ) {
		assert( z >= minZ );
		return ( z - minZ ) * rcpDeltaZ;
	} else {
		// Don't put an assertion here as it's often supposed to be called for default (zero) minZ values
		return 1.0f;
	}
}

using IcosphereVertexNeighbours = const uint16_t (*)[5];

template <SimulatedHullsSystem::ViewDotFade ViewDotFade, SimulatedHullsSystem::ZFade ZFade>
static void calcNormalsAndApplyAlphaFade( byte_vec4_t *const __restrict resultColors,
										  const vec4_t *const __restrict positions,
										  const IcosphereVertexNeighbours neighboursOfVertices,
										  const byte_vec4_t *const __restrict givenColors,
										  const float *const __restrict viewOrigin,
										  unsigned numVertices, float minZ, float maxZ, float minFadedOutAlpha ) {
	[[maybe_unused]] const float rcpDeltaZ = minZ < maxZ ? Q_Rcp( maxZ - minZ ) : 1.0f;
	// Convert to the byte range
	assert( minFadedOutAlpha >= 0.0f && minFadedOutAlpha <= 1.0f );
	minFadedOutAlpha *= 255.0f;

	unsigned vertexNum = 0;
	do {
		const uint16_t *const neighboursOfVertex = neighboursOfVertices[vertexNum];
		const float *const __restrict currVertex = positions[vertexNum];
		vec3_t normal { 0.0f, 0.0f, 0.0f };
		unsigned neighbourIndex = 0;
		do {
			const float *__restrict v2 = positions[neighboursOfVertex[neighbourIndex]];
			const float *__restrict v3 = positions[neighboursOfVertex[( neighbourIndex + 1 ) % 5]];
			vec3_t currTo2, currTo3, cross;
			VectorSubtract( v2, currVertex, currTo2 );
			VectorSubtract( v3, currVertex, currTo3 );
			CrossProduct( currTo2, currTo3, cross );
			if( const float squaredLength = VectorLengthSquared( cross ); squaredLength > 1.0f ) [[likely]] {
				const float rcpLength = Q_RSqrt( squaredLength );
				VectorMA( normal, rcpLength, cross, normal );
			}
		} while( ++neighbourIndex < 5 );

		VectorCopy( givenColors[vertexNum], resultColors[vertexNum] );
		// The sum of partial non-zero directories could be zero, check again
		const float squaredNormalLength = VectorLengthSquared( normal );
		if( squaredNormalLength > wsw::square( 1e-3f ) ) [[likely]] {
			vec3_t viewDir;
			VectorSubtract( currVertex, viewOrigin, viewDir );
			const float squareDistanceToVertex = VectorLengthSquared( viewDir );
			if( squareDistanceToVertex > 1.0f ) [[likely]] {
				const float rcpNormalLength   = Q_RSqrt( squaredNormalLength );
				const float rcpDistance       = Q_RSqrt( squareDistanceToVertex );
				VectorScale( normal, rcpNormalLength, normal );
				VectorScale( viewDir, rcpDistance, viewDir );
				const float givenAlpha        = givenColors[vertexNum][3];
				const float viewDirAlphaFrac  = calcAlphaFracForViewDirDotNormal<ViewDotFade>( viewDir, normal );
				const float deltaZAlphaFrac   = calcAlphaFracForDeltaZ<ZFade>( currVertex[2], minZ, rcpDeltaZ );
				const float combinedAlphaFrac = viewDirAlphaFrac * deltaZAlphaFrac;
				// Disallow boosting the alpha over the existing value (so transparent vertices remain the same)
				const float minAlphaForVertex = wsw::min( givenAlpha, minFadedOutAlpha );
				const float newAlpha          = wsw::max( minAlphaForVertex, givenAlpha * combinedAlphaFrac );
				resultColors[vertexNum][3]    = (uint8_t)wsw::clamp( newAlpha, 0.0f, 255.0f );
			} else {
				resultColors[vertexNum][3] = 0;
			}
		} else {
			resultColors[vertexNum][3] = 0;
		}
	} while( ++vertexNum < numVertices );
}

template <SimulatedHullsSystem::ViewDotFade ViewDotFade, SimulatedHullsSystem::ZFade ZFade>
static void applyAlphaFade( byte_vec4_t *const __restrict resultColors,
							const vec4_t *const __restrict positions,
							const vec4_t *const __restrict normals,
							const byte_vec4_t *const __restrict givenColors,
							const float *const __restrict viewOrigin,
							unsigned numVertices, float minZ, float maxZ, float minFadedOutAlpha ) {
	[[maybe_unused]] const float rcpDeltaZ = minZ < maxZ ? Q_Rcp( maxZ - minZ ) : 1.0f;
	// Convert to the byte range
	assert( minFadedOutAlpha >= 0.0f && minFadedOutAlpha <= 1.0f );
	minFadedOutAlpha *= 255.0f;

	unsigned vertexNum = 0;
	do {
		VectorCopy( givenColors[vertexNum], resultColors[vertexNum] );
		vec3_t viewDir;
		VectorSubtract( positions[vertexNum], viewOrigin, viewDir );
		const float squareDistanceToVertex = VectorLengthSquared( viewDir );
		if( squareDistanceToVertex > 1.0f ) [[likely]] {
			const float rcpDistance = Q_RSqrt( squareDistanceToVertex );
			VectorScale( viewDir, rcpDistance, viewDir );
			const float givenAlpha        = givenColors[vertexNum][3];
			const float viewDirAlphaFrac  = calcAlphaFracForViewDirDotNormal<ViewDotFade>( viewDir, normals[vertexNum] );
			const float deltaZAlphaFrac   = calcAlphaFracForDeltaZ<ZFade>( positions[vertexNum][2], minZ, rcpDeltaZ );
			const float combinedAlphaFrac = viewDirAlphaFrac * deltaZAlphaFrac;
			assert( combinedAlphaFrac >= -0.01f && combinedAlphaFrac <= +1.01f );
			// Disallow boosting the alpha over the existing value (so transparent vertices remain the same)
			const float minAlphaForVertex = wsw::min( givenAlpha, minFadedOutAlpha );
			const float newAlpha          = wsw::max( minAlphaForVertex, givenAlpha * combinedAlphaFrac );
			resultColors[vertexNum][3]    = (uint8_t)wsw::clamp( newAlpha, 0.0f, 255.0f );
		} else {
			resultColors[vertexNum][3] = 0;
		}
	} while( ++vertexNum < numVertices );
}

static void applyLightsToVertices( const vec4_t *__restrict givenPositions,
								   const byte_vec4_t *__restrict givenColors,
								   byte_vec4_t *__restrict destColors,
								   const byte_vec4_t *__restrict alphaSourceColors,
								   const Scene::DynamicLight *__restrict lights,
								   std::span<const uint16_t> affectingLightIndices,
								   unsigned numVertices ) {
	const auto numAffectingLights = (unsigned)affectingLightIndices.size();

	assert( numVertices );
	unsigned vertexIndex = 0;
	do {
		const float *__restrict vertexOrigin  = givenPositions[vertexIndex];
		auto *const __restrict givenColor     = givenColors[vertexIndex];
		auto *const __restrict resultingColor = destColors[vertexIndex];

		alignas( 16 ) vec4_t accumColor;
		VectorScale( givenColor, ( 1.0f / 255.0f ), accumColor );

		unsigned lightNum = 0;
		do {
			const Scene::DynamicLight *__restrict light = lights + affectingLightIndices[lightNum];
			const float squareLightToVertexDistance = DistanceSquared( light->origin, vertexOrigin );
			// May go outside [0.0, 1.0] as we test against the bounding box of the entire hull
			float impactStrength = 1.0f - Q_Sqrt( squareLightToVertexDistance ) * Q_Rcp( light->maxRadius );
			// Just clamp so the code stays branchless
			impactStrength = wsw::clamp( impactStrength, 0.0f, 1.0f );
			VectorMA( accumColor, impactStrength, light->color, accumColor );
		} while( ++lightNum < numAffectingLights );

		resultingColor[0] = (uint8_t)( 255.0f * wsw::clamp( accumColor[0], 0.0f, 1.0f ) );
		resultingColor[1] = (uint8_t)( 255.0f * wsw::clamp( accumColor[1], 0.0f, 1.0f ) );
		resultingColor[2] = (uint8_t)( 255.0f * wsw::clamp( accumColor[2], 0.0f, 1.0f ) );
		resultingColor[3] = alphaSourceColors[vertexIndex][3];
	} while( ++vertexIndex < numVertices );
}

auto SimulatedHullsSystem::HullDynamicMesh::calcSolidSubdivLodLevel( const float *viewOrigin,
																	 float cameraViewTangent ) const
	-> std::optional<unsigned> {
	assert( cameraViewTangent > 0.0f );
	assert( m_shared->nextLodTangentRatio > 0.0f && m_shared->nextLodTangentRatio < 1.0f );

	vec3_t center, extentVector;
	VectorAvg( this->cullMins, this->cullMaxs, center );
	VectorSubtract( this->cullMaxs, this->cullMins, extentVector );
	const float squareExtentValue = VectorLengthSquared( extentVector );
	if( squareExtentValue < wsw::square( 1.0f ) ) [[unlikely]] {
		// Skip drawing
		return std::nullopt;
	}


    unsigned chosenSubdivLevel;
    if( !m_shared->isAKeyframedHull ) {
        // Get a suitable subdiv level and store it for further use during this frame
        chosenSubdivLevel = m_shared->simulatedSubdivLevel;
        if (m_shared->tesselateClosestLod) {
            chosenSubdivLevel += 1;
        }

        const float extentValue = Q_Sqrt(squareExtentValue);
        const float squareDistance = DistanceSquared(center, viewOrigin);
        // Don't even try using lesser lods if the mesh is sufficiently close to the viewer
        if (squareDistance > wsw::square(0.5f * extentValue + 64.0f)) {
            const float meshViewTangent = extentValue * Q_RSqrt(squareDistance);
            const float meshTangentRatio = meshViewTangent * Q_Rcp(cameraViewTangent);

            // Diminish lod tangent ratio in the loop, drop subdiv level every step.
            float lodTangentRatio = m_shared->nextLodTangentRatio;
            for (;;) {
                if (meshTangentRatio > lodTangentRatio) {
                    break;
                }
                if (chosenSubdivLevel == 0) {
                    break;
                }
                chosenSubdivLevel--;
                lodTangentRatio *= m_shared->nextLodTangentRatio;
                // Sanity check
                if (lodTangentRatio < 1e-6) [[unlikely]] {
                    break;
                }
            };
        }
    } else {
        // TODO: it should not be subdiv level, it is LOD level !! these are more or less OPPOSITE
        // Get a suitable subdiv level and store it for further use during this frame
        chosenSubdivLevel = 0;

        const float extentValue = Q_Sqrt(squareExtentValue);
        const float squareDistance = DistanceSquared(center, viewOrigin);
        // Don't even try using lesser lods if the mesh is sufficiently close to the viewer
        if (squareDistance > wsw::square(0.5f * extentValue + 64.0f)) {
            const float meshViewTangent = extentValue * Q_RSqrt(squareDistance);
            const float meshTangentRatio = meshViewTangent * Q_Rcp(cameraViewTangent);

            // Diminish lod tangent ratio in the loop, drop subdiv level every step.
            float lodTangentRatio = m_shared->nextLodTangentRatio;
            for (;;) {
                if (meshTangentRatio > lodTangentRatio) {
                    break;
                }
                if (chosenSubdivLevel == maxLODs) {
                    break;
                }
                chosenSubdivLevel++;
                lodTangentRatio *= m_shared->nextLodTangentRatio;
                // Sanity check
                if (lodTangentRatio < 1e-6) [[unlikely]] {
                    break;
                }
            };
        }
    }

	return chosenSubdivLevel;
}

static const struct FadeFnHolder {
	using CalcNormalsAndApplyAlphaFadeFn = void (*)( byte_vec4_t *,
													 const vec4_t *,
													 const IcosphereVertexNeighbours,
													 const byte_vec4_t *,
													 const float *,
													 unsigned, float, float, float );
	using ApplyAlphaFadeFn = void (*)( byte_vec4_t *,
									   const vec4_t *,
									   const vec4_t *,
									   const byte_vec4_t *,
									   const float *,
									   unsigned, float, float, float );

	CalcNormalsAndApplyAlphaFadeFn calcNormalsAndApplyFadeFn[10] {};
	ApplyAlphaFadeFn applyFadeFn[10] {};

#define ADD_FNS_FOR_FADE_TO_TABLES( ViewDotFade, ZFade ) \
	do { \
        calcNormalsAndApplyFadeFn[indexForFade( SimulatedHullsSystem::ViewDotFade, SimulatedHullsSystem::ZFade )] = \
			&::calcNormalsAndApplyAlphaFade<SimulatedHullsSystem::ViewDotFade, SimulatedHullsSystem::ZFade>; \
		applyFadeFn[indexForFade( SimulatedHullsSystem::ViewDotFade, SimulatedHullsSystem::ZFade )] = \
			&::applyAlphaFade<SimulatedHullsSystem::ViewDotFade, SimulatedHullsSystem::ZFade>; \
	} while( 0 )

	FadeFnHolder() noexcept {
		// Add a reference to a template instantiation for each feasible combination.
		ADD_FNS_FOR_FADE_TO_TABLES( ViewDotFade::NoFade, ZFade::FadeOutBottom );
		ADD_FNS_FOR_FADE_TO_TABLES( ViewDotFade::FadeOutContour, ZFade::NoFade );
		ADD_FNS_FOR_FADE_TO_TABLES( ViewDotFade::FadeOutContour, ZFade::FadeOutBottom );
		ADD_FNS_FOR_FADE_TO_TABLES( ViewDotFade::FadeOutCenterLinear, ZFade::NoFade );
		ADD_FNS_FOR_FADE_TO_TABLES( ViewDotFade::FadeOutCenterLinear, ZFade::FadeOutBottom );
		ADD_FNS_FOR_FADE_TO_TABLES( ViewDotFade::FadeOutCenterQuadratic, ZFade::NoFade );
		ADD_FNS_FOR_FADE_TO_TABLES( ViewDotFade::FadeOutCenterQuadratic, ZFade::FadeOutBottom );
		ADD_FNS_FOR_FADE_TO_TABLES( ViewDotFade::FadeOutCenterCubic, ZFade::NoFade );
		ADD_FNS_FOR_FADE_TO_TABLES( ViewDotFade::FadeOutCenterCubic, ZFade::FadeOutBottom );
	}

	[[nodiscard]]
	static constexpr auto indexForFade( SimulatedHullsSystem::ViewDotFade viewDotFade,
										SimulatedHullsSystem::ZFade zFade ) -> unsigned {
		assert( (unsigned)viewDotFade < 5 && (unsigned)zFade < 2 );
		return 2 * (unsigned)viewDotFade + (unsigned)zFade;
	}

#undef ADD_FN_FOR_FLAGS_TO_TABLES
} fadeFnHolder;


void SimulatedHullsSystem::HullDynamicMesh::calcOverrideColors( byte_vec4_t *__restrict buffer,
																const float *__restrict viewOrigin,
																const float *__restrict viewAxis,
																const Scene::DynamicLight *lights,
																std::span<const uint16_t> affectingLightIndices,
																bool applyViewDotFade,
																bool applyZFade,
																bool applyLights ) const {
	assert( applyViewDotFade || applyZFade || applyLights );

	byte_vec4_t *overrideColors = nullptr;

	if( applyViewDotFade || applyZFade ) {
		const ZFade effectiveZFade        = applyZFade ? m_shared->zFade : ZFade::NoFade;
		const unsigned dataLevelToUse     = wsw::min( m_chosenSubdivLevel, m_shared->simulatedSubdivLevel );
		const IcosphereData &lodDataToUse = ::basicHullsHolder.getIcosphereForLevel( dataLevelToUse );

		// If tesselation is going to be performed, apply light to the base non-tesselated lod colors
		const auto numVertices = (unsigned)lodDataToUse.vertices.size();
		overrideColors         = buffer;

		// Either we have already calculated normals or do not need normals
		if( m_shared->simulatedNormals || !applyViewDotFade ) {
			// Call specialized implementations for each fade func

			const unsigned fnIndex = ::fadeFnHolder.indexForFade( m_shared->viewDotFade, effectiveZFade );
			const auto applyFadeFn = ::fadeFnHolder.applyFadeFn[fnIndex];

			applyFadeFn( overrideColors, m_shared->simulatedPositions, m_shared->simulatedNormals,
						 m_shared->simulatedColors, viewOrigin,  numVertices,
						 m_shared->minZLastFrame, m_shared->maxZLastFrame, m_shared->minFadedOutAlpha );
		} else {
			// Call specialized implementations for each fade func

			const unsigned fnIndex = ::fadeFnHolder.indexForFade( m_shared->viewDotFade, effectiveZFade );
			const auto applyFadeFn = ::fadeFnHolder.calcNormalsAndApplyFadeFn[fnIndex];

			applyFadeFn( overrideColors, m_shared->simulatedPositions, lodDataToUse.vertexNeighbours.data(),
						 m_shared->simulatedColors, viewOrigin, numVertices,
						 m_shared->minZLastFrame, m_shared->maxZLastFrame, m_shared->minFadedOutAlpha );
		}
	}

	if( applyLights ) {
		unsigned numVertices;
		// If tesselation is going to be performed, apply light to the base non-tesselated lod colors
		if( m_chosenSubdivLevel > m_shared->simulatedSubdivLevel ) {
			numVertices = ::basicHullsHolder.getIcosphereForLevel( m_shared->simulatedSubdivLevel ).vertices.size();
		} else {
			numVertices = ::basicHullsHolder.getIcosphereForLevel( m_chosenSubdivLevel ).vertices.size();
		}

		// Copy alpha from these colors
		const byte_vec4_t *alphaSourceColors;
		if( overrideColors ) {
			// We have applied view dot fade, use this view-dot-produced data
			alphaSourceColors = overrideColors;
		} else {
			overrideColors    = buffer;
			alphaSourceColors = m_shared->simulatedColors;
		}

		applyLightsToVertices( m_shared->simulatedPositions, m_shared->simulatedColors,
							   overrideColors, alphaSourceColors,
							   lights, affectingLightIndices, numVertices );
	}
}

auto SimulatedHullsSystem::HullDynamicMesh::getOverrideColorsCheckingSiblingCache( byte_vec4_t *__restrict localBuffer,
																				   const float *__restrict viewOrigin,
																				   const float *__restrict viewAxis,
																				   const Scene::DynamicLight *lights,
																				   std::span<const uint16_t>
																				       affectingLightIndices ) const
	-> const byte_vec4_t * {
	if( m_shared->cachedOverrideColorsSpanInBuffer ) {
		auto *maybeSpanAddress = std::addressof( *m_shared->cachedOverrideColorsSpanInBuffer );
		if( auto *span = std::get_if<std::pair<unsigned, unsigned>>( maybeSpanAddress ) ) {
			static_assert( sizeof( byte_vec4_t ) == sizeof( uint32_t ) );
			return (byte_vec4_t *)( m_shared->overrideColorsBuffer->data() + span->first );
		}
	}

	const bool shouldApplyViewDotFade   = m_shared->viewDotFade != ViewDotFade::NoFade;
	const bool actuallyApplyViewDotFade = shouldApplyViewDotFade;
	const bool shouldApplyZFade         = m_shared->zFade != ZFade::NoFade;
	const bool actuallyApplyZFade       = shouldApplyZFade && m_shared->maxZLastFrame - m_shared->minZLastFrame > 1.0f;
	const bool actuallyApplyLights      = applyVertexDynLight && !affectingLightIndices.empty();

	// Nothing to do, save this fact
	if( !( actuallyApplyViewDotFade | actuallyApplyZFade | actuallyApplyLights ) ) {
		m_shared->cachedOverrideColorsSpanInBuffer = std::monostate();
		return nullptr;
	}

	byte_vec4_t *buffer = localBuffer;
	// If a dynamic allocation is worth it
	if( m_shared->hasSibling ) {
		// Allocate some room within the global buffer for frame colors allocation.
		wsw::Vector<uint32_t> *const sharedBuffer = m_shared->overrideColorsBuffer;
		const auto offset = sharedBuffer->size();
		const auto length = ::basicHullsHolder.getIcosphereForLevel( m_shared->simulatedSubdivLevel ).vertices.size();
		sharedBuffer->resize( sharedBuffer->size() + length );
		// The data could have been reallocated during resize, retrieve the data pointer after the resize() call
		static_assert( sizeof( byte_vec4_t ) == sizeof( uint32_t ) );
		buffer = (byte_vec4_t *)( sharedBuffer->data() + offset );
		// Store as a relative offset, so it's stable regardless of reallocations
		m_shared->cachedOverrideColorsSpanInBuffer = std::make_pair<unsigned, unsigned>( offset, length );
	}

	calcOverrideColors( buffer, viewOrigin, viewAxis, lights, affectingLightIndices,
						actuallyApplyViewDotFade, actuallyApplyZFade, actuallyApplyLights );

	return buffer;
}



auto SimulatedHullsSystem::HullSolidDynamicMesh::getStorageRequirements( const float *viewOrigin,
																		 const float *viewAxis,
																		 float cameraViewTangent ) const
	-> std::optional<std::pair<unsigned, unsigned>> {
	if( m_shared->cachedChosenSolidSubdivLevel ) {
		auto *drawOrSkipAddress = std::addressof( *m_shared->cachedChosenSolidSubdivLevel );
		if( const auto *drawLevel = std::get_if<unsigned>( drawOrSkipAddress ) ) {
			m_chosenSubdivLevel   = *drawLevel;
		} else {
			return std::nullopt;
		}
	} else if( const std::optional<unsigned> drawLevel = calcSolidSubdivLodLevel( viewOrigin, cameraViewTangent ) ) {
		m_chosenSubdivLevel                    = *drawLevel;
		m_shared->cachedChosenSolidSubdivLevel = *drawLevel;
	} else {
		m_shared->cachedChosenSolidSubdivLevel = std::monostate();
		return std::nullopt;
	}

	unsigned numVertices;
	unsigned numIndices;

	if( m_shared->isAKeyframedHull ){
		unsigned LODnum = 0;
        unsigned LODtoRender;
        if( v_LODtoShow.get() >= 0 ){
            LODtoRender = v_LODtoShow.get();
        } else {
            LODtoRender = m_chosenSubdivLevel;
        }
		StaticCagedMesh *meshToRender = m_shared->meshToRender;

		for( StaticCagedMesh *currLOD = meshToRender, *nextLOD = nullptr; currLOD && ( LODnum <= LODtoRender ); currLOD = nextLOD, LODnum++ ) {
			meshToRender = currLOD;
			nextLOD = currLOD->nextLOD;
		}
        // TODO: won't reassigning m_shared->meshToRender like this in this stage confuse others?
        m_shared->meshToRender = meshToRender; // assign the appropriate LOD

		numVertices = meshToRender->numVertices;
		numIndices  = meshToRender->triIndices.size() * 3;
	} else {
		const IcosphereData &chosenLevelData = ::basicHullsHolder.getIcosphereForLevel( m_chosenSubdivLevel );
		numVertices = chosenLevelData.vertices.size();
		numIndices  = chosenLevelData.indices.size();
	}

	std::pair storageRequirements( numVertices, numIndices );

	return storageRequirements;
}

auto SimulatedHullsSystem::HullCloudDynamicMesh::getStorageRequirements( const float *viewOrigin,
																		 const float *viewAxis,
																		 float cameraViewTangent ) const
	-> std::optional<std::pair<unsigned, unsigned>> {
	if( m_shared->cachedChosenSolidSubdivLevel ) {
		auto *drawOrSkipAddress = std::addressof( *m_shared->cachedChosenSolidSubdivLevel );
		if( const auto *drawLevel = std::get_if<unsigned>( drawOrSkipAddress ) ) {
			m_chosenSubdivLevel   = *drawLevel;
		} else {
			return std::nullopt;
		}
	} else if( const std::optional<unsigned> drawLevel = calcSolidSubdivLodLevel( viewOrigin, cameraViewTangent ) ) {
		m_chosenSubdivLevel                    = *drawLevel;
		m_shared->cachedChosenSolidSubdivLevel = *drawLevel;
	} else {
		m_shared->cachedChosenSolidSubdivLevel = std::monostate();
		return std::nullopt;
	}

	unsigned numGridVertices;

	assert( m_shiftFromDefaultLevelToHide <= 0 );

	assert( m_tessLevelShiftForMinVertexIndex <= 0 );
	assert( m_tessLevelShiftForMaxVertexIndex <= 0 );
	assert( m_tessLevelShiftForMinVertexIndex <= m_tessLevelShiftForMaxVertexIndex );

	if( !m_shared->isAKeyframedHull ){
        if( m_chosenSubdivLevel > m_shared->simulatedSubdivLevel ) {
            m_chosenSubdivLevel = m_shared->simulatedSubdivLevel;
        } else {
            const int shiftFromDefaultLevel = (int)m_chosenSubdivLevel - (int)m_shared->simulatedSubdivLevel;
            if( shiftFromDefaultLevel <= m_shiftFromDefaultLevelToHide ) {
                return std::nullopt;
            }
        }

		if( const int level = (int) m_chosenSubdivLevel + this->m_tessLevelShiftForMinVertexIndex; level > 0 ) {
			m_minVertexNumThisFrame = (unsigned) ::basicHullsHolder.getIcosphereForLevel( level - 1 ).vertices.size();
		} else {
			m_minVertexNumThisFrame = 0;
		}

		if( const int level = (int) m_chosenSubdivLevel + this->m_tessLevelShiftForMaxVertexIndex; level >= 0 ) {
			m_vertexNumLimitThisFrame = ::basicHullsHolder.getIcosphereForLevel( level ).vertices.size();
		} else {
			m_vertexNumLimitThisFrame = ::basicHullsHolder.getIcosphereForLevel( 0 ).vertices.size();
		}

		assert( m_minVertexNumThisFrame < m_vertexNumLimitThisFrame );
		numGridVertices = m_vertexNumLimitThisFrame - m_minVertexNumThisFrame;
	} else {
		unsigned LODnum = 0;
		unsigned LODtoRender;
		if( v_LODtoShow.get() >= 0 ){
			LODtoRender = v_LODtoShow.get();
		} else {
			LODtoRender = m_chosenSubdivLevel;
            cgNotice() << "lod to render in get storage" << LODtoRender;
		}
		StaticCagedMesh *meshToRender = m_shared->meshToRender;

		for( StaticCagedMesh *currLOD = meshToRender, *nextLOD = nullptr; currLOD && ( LODnum <= LODtoRender ); currLOD = nextLOD, LODnum++ ) {
			meshToRender = currLOD;
			nextLOD = currLOD->nextLOD;
		}
		// TODO: won't reassigning m_shared->meshToRender like this in this stage confuse others?
		m_shared->meshToRender = meshToRender; // assign the appropriate LOD

		numGridVertices = meshToRender->numVertices * m_fractionOfParticlesToRender;
		cgNotice() << "grid vertices:" << numGridVertices;
	}

	const unsigned numVertices = numGridVertices * 4;
	const unsigned numIndices  = numGridVertices * 6;

	return std::make_pair( numVertices, numIndices );
}

static void lerpLayerColorsAndRangesBetweenFrames( byte_vec4_t *__restrict destColors, float *__restrict destColorRanges,
												   unsigned numColors, float lerpFrac,
												   const byte_vec4_t *__restrict prevFrameColors,
												   const byte_vec4_t *__restrict nextFrameColors,
												   const float *__restrict prevFrameColorRanges,
												   const float *__restrict nextFrameColorRanges ) {
	assert( numColors );
	unsigned colorNum = 0;
	do {
		Vector4Lerp( prevFrameColors[colorNum], lerpFrac, nextFrameColors[colorNum], destColors[colorNum] );
		destColorRanges[colorNum] = std::lerp( prevFrameColorRanges[colorNum], nextFrameColorRanges[colorNum], lerpFrac );
	} while( ++colorNum < numColors );
}

static void addLayerContributionToResultColor( float rampValue, unsigned numColors,
											   const byte_vec4_t *__restrict lerpedColors, const float *__restrict lerpedColorRanges,
											   SimulatedHullsSystem::BlendMode blendMode, SimulatedHullsSystem::AlphaMode alphaMode,
											   uint8_t *__restrict resultColor, unsigned layerNum ) {
	unsigned nextColorIndex = 0; // the index of the next color after the point in the color ramp
	while( rampValue > lerpedColorRanges[nextColorIndex] && nextColorIndex < numColors ) {
		nextColorIndex++;
	}

	byte_vec4_t layerColor;
	if( nextColorIndex == 0 ) {
		Vector4Copy( lerpedColors[0], layerColor );
	} else if( nextColorIndex == numColors ) {
		Vector4Copy( lerpedColors[numColors - 1], layerColor );
	} else {
		const unsigned prevColorIndex = nextColorIndex - 1;
		const float offsetInRange     = rampValue - lerpedColorRanges[prevColorIndex];
		const float rangeLength       = lerpedColorRanges[nextColorIndex] - lerpedColorRanges[prevColorIndex];
		assert( offsetInRange >= 0.0f && offsetInRange <= rangeLength );
		const float lerpFrac          = wsw::clamp( offsetInRange * Q_Rcp( rangeLength ), 0.0f, 1.0f );
		Vector4Lerp( lerpedColors[prevColorIndex], lerpFrac, lerpedColors[nextColorIndex], layerColor );
	}

	if( layerNum == 0 ) {
		Vector4Copy( layerColor, resultColor );
	} else {
		if( blendMode == SimulatedHullsSystem::BlendMode::AlphaBlend ) {
			VectorLerp( resultColor, layerColor[3], layerColor, resultColor );
		} else if( blendMode == SimulatedHullsSystem::BlendMode::Add ) {
			for( int i = 0; i < 3; i++ ) {
				resultColor[i] = wsw::min( resultColor[i] + layerColor[i], 255 );
			}
		} else if( blendMode == SimulatedHullsSystem::BlendMode::Subtract ) {
			for( int i = 0; i < 3; i++ ) {
				resultColor[i] = wsw::max( resultColor[i] - layerColor[i], 0 );
			}
		}

		if( alphaMode == SimulatedHullsSystem::AlphaMode::Override ) {
			resultColor[3] = layerColor[3];
		} else if( alphaMode == SimulatedHullsSystem::AlphaMode::Add ) {
			resultColor[3] = wsw::min( resultColor[3] + layerColor[3], 255 );
		} else if( alphaMode == SimulatedHullsSystem::AlphaMode::Subtract ) {
			resultColor[3] = wsw::max( resultColor[3] - layerColor[3], 0 );
		}
	}
}

void SimulatedHullsSystem::vertexPosFromStaticCage( const StaticCageCoordinate vertCoord, float scale, const tri *cageTriIndices,
													const vec3_t *vertexMoveDirections, const float *limitsAtDirections,
													const float offsetFromLim, vec3_t outPos ) {
	unsigned triIdx = vertCoord.cageTriIdx;

	const vec2_t coords = { vertCoord.coordsOnCageTri[0], vertCoord.coordsOnCageTri[1] }; // 2: vertCoords

	vec3_t moveDir;

	const unsigned cageVertIdx0 = cageTriIndices[triIdx][0];
	const unsigned cageVertIdx1 = cageTriIndices[triIdx][1];
	const unsigned cageVertIdx2 = cageTriIndices[triIdx][2];

	const float coeff0 = 1 - (coords[0] + coords[1]);
	const float coeff1 = coords[0];
	const float coeff2 = coords[1];

	VectorScale( vertexMoveDirections[cageVertIdx0], coeff0, moveDir );
	VectorMA( moveDir, coeff1, vertexMoveDirections[cageVertIdx1], moveDir );
	VectorMA( moveDir, coeff2, vertexMoveDirections[cageVertIdx2], moveDir );
	VectorNormalizeFast( moveDir );

	const float limit =
			limitsAtDirections[cageVertIdx0] * coeff0 +
			limitsAtDirections[cageVertIdx1] * coeff1 +
			limitsAtDirections[cageVertIdx2] * coeff2;

	const float maxOffset = wsw::max( 0.0f, limit - offsetFromLim );

	const float offset = wsw::min( vertCoord.offset, maxOffset ) * scale;

	VectorScale( moveDir, offset, outPos );
}

IntConfigVar v_testMode( wsw::StringView("testMode"), { .byDefault = 0, .flags = CVAR_ARCHIVE } );

auto SimulatedHullsSystem::HullSolidDynamicMesh::fillMeshBuffers( const float *__restrict viewOrigin,
																  const float *__restrict viewAxis,
																  float cameraFovTangent,
																  const Scene::DynamicLight *lights,
																  std::span<const uint16_t> affectingLightIndices,
																  vec4_t *__restrict destPositions,
																  vec4_t *__restrict destNormals,
																  vec2_t *__restrict destTexCoords,
																  byte_vec4_t *__restrict destColors,
																  uint16_t *__restrict destIndices ) const
	-> std::pair<unsigned, unsigned> {//

	unsigned numResultVertices, numResultIndices;

	if( !m_shared->isAKeyframedHull ) {
		assert( m_shared->simulatedSubdivLevel <= BasicHullsHolder::kMaxSubdivLevel );
		assert( m_chosenSubdivLevel <= m_shared->simulatedSubdivLevel + 1 );
		assert( m_shared->minZLastFrame <= m_shared->maxZLastFrame );

		// Keep always allocating the default buffer even if it's unused, so we can rely on it
		const auto colorsBufferLevel = wsw::min( m_chosenSubdivLevel, m_shared->simulatedSubdivLevel );
		const auto colorsBufferSize  = (unsigned)basicHullsHolder.getIcosphereForLevel( colorsBufferLevel ).vertices.size();
		assert( colorsBufferSize && colorsBufferSize < ( 1 << 12 ) );
		auto *const overrideColorsBuffer = (byte_vec4_t *)alloca( sizeof( byte_vec4_t ) * colorsBufferSize );

		const byte_vec4_t *overrideColors;

		overrideColors = getOverrideColorsCheckingSiblingCache( overrideColorsBuffer, viewOrigin,
																viewAxis, lights,
																affectingLightIndices);

		// HACK Perform an additional tesselation of some hulls.
		// CPU-side tesselation is the single option in the current codebase state.
		if( m_shared->tesselateClosestLod && m_chosenSubdivLevel > m_shared->simulatedSubdivLevel ) {
			assert( m_shared->simulatedSubdivLevel + 1 == m_chosenSubdivLevel );
			const IcosphereData &nextLevelData = ::basicHullsHolder.getIcosphereForLevel( m_chosenSubdivLevel );
			const IcosphereData &simLevelData  = ::basicHullsHolder.getIcosphereForLevel( m_shared->simulatedSubdivLevel );

			const IcosphereVertexNeighbours nextLevelNeighbours = nextLevelData.vertexNeighbours.data();

			const auto numSimulatedVertices = (unsigned)simLevelData.vertices.size();
			const auto numNextLevelVertices = (unsigned)nextLevelData.vertices.size();

			MeshTesselationHelper *const tesselationHelper = &::meshTesselationHelper;
			if( m_shared->lerpNextLevelColors ) {
				tesselationHelper->exec<true>( this, overrideColors,
											   numSimulatedVertices, numNextLevelVertices, nextLevelNeighbours );
			} else {
				tesselationHelper->exec<false>( this, overrideColors,
												numSimulatedVertices, numNextLevelVertices, nextLevelNeighbours );
			}

			numResultVertices = numNextLevelVertices;
			numResultIndices  = (unsigned)nextLevelData.indices.size();

			// TODO: Eliminate this excessive copying
			std::memcpy( destPositions, tesselationHelper->m_tessPositions, sizeof( destPositions[0] ) * numResultVertices );
			std::memcpy( destColors, tesselationHelper->m_tessByteColors, sizeof( destColors[0] ) * numResultVertices );
			std::memcpy( destIndices, nextLevelData.indices.data(), sizeof( uint16_t ) * numResultIndices );
		} else {
			const IcosphereData &dataToUse = ::basicHullsHolder.getIcosphereForLevel( m_chosenSubdivLevel );
			const byte_vec4_t *colorsToUse = overrideColors ? overrideColors : m_shared->simulatedColors;

			numResultVertices = (unsigned)dataToUse.vertices.size();
			numResultIndices  = (unsigned)dataToUse.indices.size();

			const vec4_t *simulatedPositions = m_shared->simulatedPositions;
			std::memcpy( destPositions, simulatedPositions, sizeof( simulatedPositions[0] ) * numResultVertices );
			std::memcpy( destColors, colorsToUse, sizeof( colorsToUse[0] ) * numResultVertices );
			std::memcpy( destIndices, dataToUse.indices.data(), sizeof( uint16_t ) * numResultIndices );
		}

	} else {
        const auto before = Sys_Microseconds();

        // the correct LOD has already been assigned in getSotrageRequirements()
		StaticCagedMesh *meshToRender = m_shared->meshToRender;
        const float lifetimeFrac = m_shared->lifetimeFrac;

		// write positions to destPositions
		// memcpy indices from mesh to render
        const unsigned numFrames  = meshToRender->numFrames;
		const unsigned numTris    = meshToRender->triIndices.size();
		const unsigned numIndices = numTris * 3;
        const unsigned numVerts   = meshToRender->numVertices;

        numResultVertices = numVerts;
        numResultIndices  = numIndices;

        const tri *meshTriIndices = meshToRender->triIndices.data();

		std::memcpy( destIndices, meshTriIndices, sizeof( uint16_t ) * numIndices );
        std::memcpy( destTexCoords, meshToRender->UVCoords, sizeof( vec2_t ) * numVerts );

        const tri *cageTriIndices = m_shared->cageTriIndices;
        const StaticCageCoordinate *vertCoords = meshToRender->vertexCoordinates;
		const float *offsetsFromLim = meshToRender->offsetFromLim;

        const float *limitsAtDirections    = m_shared->limitsAtDirections;
        const vec3_t *vertexMoveDirections = m_shared->cageVertexPositions;
        const float scale                  = m_shared->scale;
        const float *origin                = m_shared->origin;

        // write positions to dest positions
        unsigned currFrame;
		float lerpFrac;
        if( v_frameToShow.get() < 0 ){
            currFrame = wsw::min( (unsigned) ((float) numFrames * lifetimeFrac), numFrames - 1 );
			lerpFrac  = (float)numFrames * lifetimeFrac - (float)currFrame;
        } else {
            currFrame = v_frameToShow.get();
			lerpFrac  = 0.0f;
        }
		const unsigned nextFrame   = wsw::min( currFrame + 1, numFrames - 1 );
		const float complementFrac = 1.0f - lerpFrac;

        unsigned currFrameStartVertIdx = currFrame * numVerts;
		unsigned nextFrameStartVertIdx = nextFrame * numVerts;

        vec3_t color;
        VectorSet( color, 0.99f, 0.4f, 0.1f );

		const auto beforeConstruct = Sys_Microseconds();
        for( unsigned vertNum = 0; vertNum < numVerts; vertNum++ ) {
            unsigned currFrameVertIdx = currFrameStartVertIdx + vertNum;
			unsigned nextFrameVertIdx = nextFrameStartVertIdx + vertNum;
			vec3_t nextPositionContrib;

			vertexPosFromStaticCage( vertCoords[currFrameVertIdx], scale * complementFrac, cageTriIndices,
									 vertexMoveDirections, limitsAtDirections, offsetsFromLim[currFrameVertIdx],
									 destPositions[vertNum] );
			vertexPosFromStaticCage( vertCoords[nextFrameVertIdx], scale * lerpFrac, cageTriIndices,
									 vertexMoveDirections, limitsAtDirections, offsetsFromLim[nextFrameVertIdx],
									 nextPositionContrib );
			VectorAdd( destPositions[vertNum], nextPositionContrib, destPositions[vertNum] );
			VectorAdd( destPositions[vertNum], origin, destPositions[vertNum] );

            if( vertCoords[currFrameVertIdx].cageTriIdx == v_triIdx.get() ){
                vec3_t to;
                VectorSubtract( destPositions[vertNum], origin, to );
                VectorScale( to, 1.15f, to );
                VectorAdd( to, origin, to );
                cg.polyEffectsSystem.spawnTransientBeamEffect( destPositions[vertNum], to, {
                        .material          = cgs.media.shaderLaser,
                        .beamColorLifespan = {
                                .initial  = {color[0], color[1], color[2]},
                                .fadedIn  = {color[0], color[1], color[2]},
                                .fadedOut = {color[0], color[1], color[2]},
                        },
                        .width             = 4.5f,
                        .timeout           = v_trisLifetime.get(),
                } );
            }

            destPositions[vertNum][3] = 1.0f;
        }
		if( v_showConstructPerf.get() ) {
			Com_Printf( "Construction took %d micros for %d vertices and %d indices\n",
						(int) (Sys_Microseconds() - beforeConstruct), (int) numResultVertices, (int) numResultIndices );
		}

		bool needsViewDotResults = false;
		float *viewDotResults    = nullptr;

		const unsigned numShadingLayers = meshToRender->numShadingLayers;

		for(  unsigned layerNum = 0; layerNum < numShadingLayers; ++layerNum ) {
			const ShadingLayer &layer = meshToRender->shadingLayers[layerNum];
			if( !std::holds_alternative<MaskedShadingLayer>( layer ) ) {
				assert( std::holds_alternative<DotShadingLayer>( layer ) || std::holds_alternative<CombinedShadingLayer>( layer ) );
				needsViewDotResults = true;
				break;
			}
		}

		const auto beforeViewDot = Sys_Microseconds();
		if( needsViewDotResults ) {

			assert( numVertices > 0 && numVertices < 4096 );
			viewDotResults = (float *)alloca( sizeof( float ) * numVerts );

			auto *normals = (vec3_t *)alloca( sizeof( vec3_t ) * numVerts );
			for( unsigned vertNum = 0; vertNum < numVerts; vertNum++ ){
				VectorSet( normals[vertNum], 0.0f, 0.0f, 0.0f );
			}

			unsigned triNum = 0;
			do {
				const unsigned firstVertIdx  = meshTriIndices[triNum][0];
				const unsigned secondVertIdx = meshTriIndices[triNum][1];
				const unsigned thirdVertIdx  = meshTriIndices[triNum][2];

				vec3_t triCoords[3]; // TODO: eliminate this copying here and in cloud one
				VectorCopy( destPositions[firstVertIdx], triCoords[0] );
				VectorCopy( destPositions[secondVertIdx], triCoords[1] );
				VectorCopy( destPositions[thirdVertIdx], triCoords[2] );

				vec3_t vecToSecondVert;
				vec3_t vecToThirdVert;
				VectorSubtract( triCoords[1], triCoords[0], vecToSecondVert );
				VectorSubtract( triCoords[2], triCoords[0], vecToThirdVert );

				vec3_t normal;
				CrossProduct( vecToThirdVert, vecToSecondVert, normal );

				VectorAdd( normals[firstVertIdx], normal, normals[firstVertIdx] );
				VectorAdd( normals[secondVertIdx], normal, normals[secondVertIdx] );
				VectorAdd( normals[thirdVertIdx], normal, normals[thirdVertIdx] );
			} while( ++triNum < numTris );

			for( unsigned vertNum = 0; vertNum < numVerts; vertNum++ ){
				const float *const __restrict currVertex = destPositions[vertNum];
				const float *const __restrict currNormal = normals[vertNum];

				const float squaredNormalLength = VectorLengthSquared( currNormal );
				vec3_t viewDir;
				VectorSubtract( currVertex, viewOrigin, viewDir );
				const float squareDistanceToVertex = VectorLengthSquared( viewDir );

				const float squareRcpNormalizingFactor = squaredNormalLength * squareDistanceToVertex;
				// check that both the length of the normal and distance to vertex are not 0 in one branch
				if( squareRcpNormalizingFactor > 1.0f ) [[likely]] {
					const float normalizingFactor = Q_RSqrt( squareRcpNormalizingFactor );
					viewDotResults[vertNum] = std::fabs( DotProduct( viewDir, currNormal ) ) * normalizingFactor;
				} else {
					viewDotResults[vertNum] = 0.0f;
				}
//
//				VectorNormalizeFast( normals[vertNum] );
//				viewDotResults[vertNum] = DotProduct( normals[vertNum], viewAxis );
			}
		}
		if( v_showPerf.get() ) {
			Com_Printf( "viewdot results took %d micros for %d vertices and %d indices\n",
						(int) (Sys_Microseconds() - beforeViewDot), (int) numResultVertices, (int) numResultIndices );
		}


		for( unsigned layerNum = 0; layerNum < numShadingLayers; ++layerNum ) {
			const ShadingLayer &prevShadingLayer = meshToRender->shadingLayers[currFrame * numShadingLayers + layerNum];
			const ShadingLayer &nextShadingLayer = meshToRender->shadingLayers[nextFrame * numShadingLayers + layerNum];

			const auto beforeLayer = Sys_Microseconds();
			if( const auto *const prevMaskedLayer = std::get_if<MaskedShadingLayer>( &prevShadingLayer ) ) {
				const auto *const nextMaskedLayer = std::get_if<MaskedShadingLayer>( &nextShadingLayer );

				const unsigned numColors = prevMaskedLayer->colors.size();
				assert( numColors > 0 && numColors <= kMaxLayerColors );

				byte_vec4_t lerpedColors[kMaxLayerColors];
				float lerpedColorRanges[kMaxLayerColors];
				lerpLayerColorsAndRangesBetweenFrames( lerpedColors, lerpedColorRanges, numColors, lerpFrac,
													   prevMaskedLayer->colors.data(), nextMaskedLayer->colors.data(),
													   prevMaskedLayer->colorRanges, nextMaskedLayer->colorRanges );

				unsigned vertexNum = 0;
				do {
					const float vertexMaskValue = std::lerp( prevMaskedLayer->vertexMaskValues[vertexNum],
															 nextMaskedLayer->vertexMaskValues[vertexNum],
															 lerpFrac );

					addLayerContributionToResultColor( vertexMaskValue, numColors, lerpedColors, lerpedColorRanges,
													   prevMaskedLayer->blendMode, prevMaskedLayer->alphaMode,
													   destColors[vertexNum], layerNum );

				} while ( ++vertexNum < numVerts );
			} else if( const auto *const prevDotLayer = std::get_if<DotShadingLayer>( &prevShadingLayer ) ) {
				const auto *const nextDotLayer = std::get_if<DotShadingLayer>( &nextShadingLayer );

				const unsigned numColors = prevDotLayer->colors.size();
				assert( numColors > 0 && numColors <= kMaxLayerColors );

				byte_vec4_t lerpedColors[kMaxLayerColors];
				float lerpedColorRanges[kMaxLayerColors];
				lerpLayerColorsAndRangesBetweenFrames( lerpedColors, lerpedColorRanges, numColors, lerpFrac,
													   prevDotLayer->colors.data(), nextDotLayer->colors.data(),
													   prevDotLayer->colorRanges, nextDotLayer->colorRanges );

				unsigned vertexNum = 0;
				do {
					addLayerContributionToResultColor( viewDotResults[vertexNum], numColors, lerpedColors, lerpedColorRanges,
													   prevDotLayer->blendMode, prevDotLayer->alphaMode,
													   destColors[vertexNum], layerNum );
				} while ( ++vertexNum < numVerts );
			}
			else if( const auto *const prevCombinedLayer = std::get_if<CombinedShadingLayer>( &prevShadingLayer ) ) {
				const auto *const nextCombinedLayer = std::get_if<CombinedShadingLayer>( &nextShadingLayer );

				const unsigned numColors = prevCombinedLayer->colors.size();
				assert( numColors > 0 && numColors <= kMaxLayerColors );

				byte_vec4_t lerpedColors[kMaxLayerColors];
				float lerpedColorRanges[kMaxLayerColors];
				lerpLayerColorsAndRangesBetweenFrames( lerpedColors, lerpedColorRanges, numColors, lerpFrac,
													   prevCombinedLayer->colors.data(), nextCombinedLayer->colors.data(),
													   prevCombinedLayer->colorRanges, nextCombinedLayer->colorRanges );

				unsigned vertexNum = 0;
				do {
					const float vertexDotValue  = viewDotResults[vertexNum];
					const float vertexMaskValue = std::lerp( prevCombinedLayer->vertexMaskValues[vertexNum],
															 nextCombinedLayer->vertexMaskValues[vertexNum],
															 lerpFrac );

					const float dotInfluence        = prevCombinedLayer->dotInfluence;
					const float maskInfluence       = 1.0f - dotInfluence;
					const float vertexCombinedValue = vertexDotValue * dotInfluence + vertexMaskValue * maskInfluence;

					addLayerContributionToResultColor( vertexCombinedValue, numColors, lerpedColors, lerpedColorRanges,
													   prevCombinedLayer->blendMode, prevCombinedLayer->alphaMode,
													   destColors[vertexNum], layerNum );
				} while ( ++vertexNum < numVerts );
			} else {
				wsw::failWithLogicError( "Unreachable" );
			}
		}

		if( v_showPerf.get() ) {
			Com_Printf( "Total took %d micros for %d vertices and %d indices\n", (int) (Sys_Microseconds() - before),
						(int) numResultVertices, (int) numResultIndices );
		}
	}

	return { numResultVertices, numResultIndices };
};

// TODO: Lift it to the top level, make views of different types that point to it as well

alignas( 16 ) static const uint8_t kRandomBytes[] = {
	0x01, 0x2b, 0x1e, 0xbc, 0x47, 0x26, 0xc3, 0x0a, 0x6c, 0x50, 0x0f, 0x5c, 0x64, 0x14, 0xe8, 0x56, 0x06, 0x05, 0x5b,
	0x8e, 0xe1, 0x0a, 0x3d, 0xcb, 0x9c, 0x94, 0x54, 0xe2, 0x61, 0xba, 0x03, 0xb9, 0xcd, 0x5d, 0x03, 0x6b, 0x2a, 0xfd,
	0x05, 0xec, 0x12, 0x21, 0x0a, 0x5c, 0x43, 0x41, 0xc5, 0xcb, 0x1e, 0xf8, 0x71, 0x17, 0x00, 0x06, 0x35, 0xb2, 0x2d,
	0x50, 0xf3, 0x08, 0x38, 0x69, 0x8e, 0x9e, 0xdc, 0x58, 0x6e, 0x2f, 0xff, 0xfc, 0xad, 0xa3, 0x57, 0x3d, 0xfe, 0x74,
	0x2a, 0x0b, 0x84, 0xaa, 0x61, 0x82, 0xa1, 0xb8, 0x78, 0x11, 0x34, 0x88, 0xb4, 0x89, 0x65, 0x8e, 0xcb, 0x8c, 0xdd,
	0x64, 0xf5, 0xc3, 0x6f, 0xce, 0xf6, 0xf7, 0x58, 0xc0, 0xb3, 0x8a, 0xc4, 0x90, 0xe9, 0x11, 0x38, 0x86, 0x9f, 0x2b,
	0xd4, 0x02, 0xcc, 0xbc, 0x53, 0xd5, 0x65, 0x94, 0x55, 0x1d, 0x1c, 0xc6, 0xe7, 0x4f, 0x53, 0x26, 0xa8, 0xc7, 0x7c,
	0xed, 0x9d, 0x09, 0x67, 0xea, 0xdb, 0xf2, 0x7c, 0x51, 0xe2, 0xf6, 0x0f, 0x38, 0x44, 0x5e, 0x22, 0x4a, 0xea, 0xd8,
	0xf3, 0xce, 0x4e, 0xaf, 0xb6, 0x33, 0x0d, 0x14, 0xaa, 0x77, 0xc5, 0xc3, 0x26, 0x7a, 0xca, 0x8d, 0x0c, 0x9c, 0x34,
	0x8f, 0x63, 0xae, 0x65, 0xf1, 0x0a, 0x87, 0xf1, 0x25, 0x92, 0x6c, 0x56, 0x79, 0x2a, 0x7f, 0xbc, 0x3f, 0x8e, 0x72,
	0x10, 0x30, 0x33, 0x18, 0xa5, 0x83, 0xcc, 0xb9, 0x7d, 0x88, 0xd1, 0xf7, 0xc0, 0x86, 0x4b, 0x5f, 0x94, 0xd3, 0xd3,
	0xfb, 0x50, 0x61, 0xc3, 0x56, 0xe4, 0xcf, 0xb9, 0x72, 0x01, 0x97, 0xf2, 0x1c, 0x17, 0x82, 0x54, 0xf8, 0x5f, 0x77,
	0x2d, 0xde, 0x01, 0xa9, 0xb3, 0x49, 0xa1, 0x5a, 0xf3, 0x66, 0x79, 0x24, 0xfb, 0x44, 0x03, 0x42, 0x12, 0x7a, 0xaf,
	0x17, 0x1a, 0xa3, 0x4e, 0x5d, 0xfb, 0xf8, 0x27, 0x01, 0x70, 0xae, 0xed, 0x4d, 0x7a, 0x25, 0x96, 0x69, 0x56, 0xc3,
	0xf6, 0x24, 0xb0, 0x88, 0x35, 0x0d, 0x5f, 0x9a, 0x70, 0x74, 0x84, 0x7e, 0x64, 0x62, 0x19, 0x09, 0xf5, 0x1b, 0x33,
	0x21, 0xf1, 0xac, 0x5a, 0x90, 0xcd, 0x56, 0x77, 0xfe, 0x75, 0xac, 0xb1, 0x7d, 0xb3, 0x44, 0x87, 0xe9, 0x9e, 0xb6,
	0x4f, 0x5d, 0x50, 0xe1, 0xac, 0x65, 0xaf, 0x61, 0xfd, 0xaa, 0x22, 0x74, 0x6a, 0xf3, 0x28, 0x3e, 0x95, 0x3d, 0x1e,
	0x2d, 0xff, 0x2f, 0xd3, 0xbb, 0x3e, 0x6b, 0xb9, 0x6d, 0x42, 0x31, 0x96, 0x4f, 0xf4, 0xad, 0x57, 0xce, 0x9a, 0xf3,
	0x4b, 0x5c, 0x57, 0x7b, 0x44, 0x40, 0x17, 0x10, 0x73, 0x40, 0xbf, 0x0e, 0xf0, 0xec, 0x2a, 0x3d, 0xbb, 0x4f, 0xea,
	0xb2, 0x5c, 0x53, 0x25, 0x86, 0xa4, 0xf1, 0x35, 0x44, 0x64, 0xdb, 0x6c, 0xcf, 0xce, 0xcb, 0x58, 0xa8, 0x50, 0x07,
	0x32, 0xf5, 0x67, 0x80, 0x2b, 0xbb, 0xc4, 0x57, 0x63, 0x34, 0x56, 0x9c, 0x2c, 0x17, 0x16, 0xb6, 0x9e, 0x47, 0xf0,
	0xd7, 0x39, 0x4d, 0x1f, 0xd9, 0x5e, 0x15, 0x61, 0xf9, 0x8b, 0x27, 0x93, 0x12, 0x0e, 0xac, 0x5c, 0x0c, 0xd9, 0xce,
	0xe3, 0x38, 0x1c, 0x23, 0xf6, 0xaf, 0x9a, 0x1b, 0x71, 0xdf, 0xbb, 0x5b, 0x84, 0x02, 0xf3, 0x52, 0x2b, 0x4e, 0x7a,
	0xdd, 0x75, 0x49, 0xba, 0xb8, 0x8b, 0x2c, 0x9d, 0x68, 0xfb, 0x98, 0x74, 0x07, 0xb6, 0x6d, 0xb4, 0xe7, 0x48, 0x2a,
	0x39, 0xbe, 0x1f, 0x81, 0x36, 0xbb, 0xd1, 0x5a, 0x14, 0x2b, 0x8e, 0x1c, 0x00, 0xd2, 0x6b, 0x53, 0x2f, 0x81, 0x04,
	0x2b, 0x54, 0x07, 0xd5, 0x11, 0x75, 0xdb, 0x45, 0x65, 0x60, 0xa2, 0x44, 0x13, 0xa1, 0x93, 0xd7, 0x69, 0x3a, 0xa4,
	0x59, 0x26, 0x61, 0x7b, 0x73, 0xec, 0x57, 0x36, 0xb1, 0xd0, 0x3f, 0x99, 0x5c, 0xad, 0xe8, 0x6a, 0x87, 0x21, 0xab,
	0x5a, 0xec, 0x56, 0xfb, 0xfd, 0xc9, 0x19, 0x85, 0x38, 0x9e, 0x20, 0xce, 0x18, 0xfc, 0x1e, 0x75, 0x01, 0xb5, 0x8f,
	0x7b, 0x80, 0x47, 0x2f, 0x31, 0x91, 0x99, 0x45, 0x32, 0x60, 0xde, 0xc3, 0x1d, 0xee, 0xfa, 0x35, 0x91, 0xad, 0xd9,
	0x06, 0xdb, 0xb3, 0x6c, 0xe1, 0x33, 0x4d, 0x6e, 0x01, 0xaf, 0xa0, 0x1f, 0x3f, 0x63, 0x4f, 0x2d, 0x54, 0x50, 0xbc,
	0xf3, 0xac, 0xa4, 0x3e, 0xc4, 0x23, 0x02, 0xeb, 0xe2, 0x4e, 0xf0, 0xe3, 0xdd, 0x49, 0x09, 0xb6, 0xc5, 0x41, 0x7d,
	0x79, 0x39, 0xdc, 0x2e, 0xb4, 0xa2, 0x7b, 0xa2, 0x63, 0xc0, 0x08, 0x80, 0xa5, 0xfe, 0xe7, 0x63, 0x2d, 0x92, 0x02,
	0x0c, 0x82, 0x20, 0x4c, 0x11, 0x26, 0xb6, 0x94, 0xd1, 0x08, 0x60, 0xc4, 0x4b, 0x5d, 0x82, 0xac, 0x35, 0x98, 0x0a,
	0xb5, 0x2a, 0x25, 0x21, 0x0c, 0x3c, 0x0b, 0x79, 0xa9, 0x86, 0x08, 0x14, 0x1f, 0x01, 0x90, 0x22, 0x10, 0x11, 0x29,
	0x6c, 0x14, 0x0e, 0x15, 0x97, 0x36, 0xf4, 0xe4, 0x67, 0x53, 0xd0, 0x6b, 0x7a, 0x93, 0x50, 0xbf, 0x41, 0xa0, 0x14,
	0x26, 0x07, 0x33, 0x47, 0xab, 0x01, 0x9b, 0xf7, 0x9c, 0x35, 0xae, 0xda, 0x89, 0xf6, 0x78, 0x16, 0xd8, 0x98, 0xf5,
	0xc0, 0x86, 0x76, 0x74, 0xad, 0xb0, 0xff, 0x7d, 0x46, 0xb2, 0xf3, 0x2f, 0x0f, 0x81, 0x31, 0xe6, 0x85, 0x19, 0x66,
	0x8d, 0xea, 0x8f, 0x3a, 0x94, 0x80, 0x8b, 0x5c, 0x36, 0xee, 0xd1, 0xd1, 0xb4, 0x8b, 0x28, 0x2d, 0x60, 0x4f, 0xe7,
	0xc1, 0x58, 0x0c, 0x3e, 0xec, 0xb2, 0x01, 0x88, 0xda, 0x8d, 0x67, 0x17, 0xa5, 0x34, 0x1f, 0x07, 0xae, 0xfa, 0xc5,
	0x09, 0x22, 0x90, 0x8b, 0x38, 0xdb, 0x5a, 0x0b, 0xfa, 0xd2, 0x13, 0xe4, 0x59, 0x9f, 0xa6, 0x35, 0x4b, 0xe4, 0xc4,
	0xc3, 0xeb, 0x6e, 0xd4, 0x87, 0xf6, 0xc8, 0x25, 0xfe, 0x7c, 0x0c, 0xaf, 0x0b, 0x70, 0x5f, 0xcb, 0xed, 0x48, 0xcf,
	0x50, 0x79, 0x4b, 0xde, 0x42, 0x98, 0x26, 0xe7, 0xcc, 0x97, 0xfa, 0xd4, 0x10, 0xf4, 0x14, 0x16, 0x79, 0x93, 0x55,
	0xea, 0x09, 0xeb, 0x53, 0x43, 0xc5, 0xda, 0xcc, 0xce, 0xb9, 0x17, 0xc5, 0xb2, 0x1d, 0xa2, 0x40, 0x1e, 0x96, 0x08,
	0x0a, 0x67, 0x14, 0x28, 0x57, 0x69, 0x4c, 0x13, 0x32, 0xf8, 0x09, 0xde, 0xe7, 0x9f, 0x0f, 0xa9, 0xb0, 0x55, 0x53,
	0xf4, 0xd8, 0xdd, 0x07, 0x86, 0x22, 0xb0, 0x73, 0x43, 0x98, 0xd1, 0x32, 0xfe, 0xe3, 0x89, 0x5c, 0x93, 0xf9, 0x31,
	0x37, 0x90, 0x05, 0x47, 0x59, 0x13, 0x28, 0x1d, 0xdc, 0xd8, 0xf4, 0x91, 0x58, 0xb9, 0xcd, 0xe3, 0x7b, 0xb4, 0x25,
	0x86, 0x0e, 0xfe, 0x8d, 0x18, 0x10, 0x5c, 0x00, 0xa2, 0xd7, 0x9c, 0x91, 0x04, 0x20, 0xcc, 0x9d, 0xa3, 0xfc, 0x63,
	0xef, 0xf5, 0x00, 0x97, 0xe7, 0x73, 0xb5, 0x38, 0x57, 0x71, 0x96, 0xd0, 0xee, 0xc9, 0x10, 0xcc, 0x2d, 0x56, 0xb6,
	0x91, 0x32, 0x82, 0xa7, 0x33, 0x48, 0x60, 0xd8, 0x8e, 0x5f, 0x45, 0xfd, 0x20, 0x1c, 0xc8, 0x91, 0x22, 0x17, 0x3d,
	0x5e, 0x01, 0xa8, 0xf5, 0xa7, 0x12, 0x9b, 0x76, 0xdd, 0x67, 0x7d, 0x01, 0xb9, 0xd1, 0xb3, 0x3e, 0xbc, 0x4d, 0x7b,
	0x50, 0xf0, 0xb2, 0xe1, 0x22, 0x63, 0xe2, 0x6b, 0x69, 0xbb, 0x9b, 0x0e, 0xf4, 0x9a, 0xdd, 0x78, 0x44, 0x1b, 0xb4,
	0xba, 0x6b, 0x86, 0xf1, 0x05, 0x9b, 0xcf, 0xe4, 0xb8, 0xcf, 0x26, 0xd1, 0xd5, 0xf7, 0xc8, 0x58, 0x4c, 0x7d, 0xf8,
	0xcb, 0x80, 0xfb, 0xf6, 0xf3, 0x2f, 0x2e, 0xe2, 0x84, 0xfc, 0xb0, 0xd1, 0xb8, 0xf3, 0xb2, 0x39, 0x85, 0x17, 0x7c,
	0x47, 0x8c, 0x23, 0x31, 0x4d, 0x33, 0x97, 0x00, 0xd1, 0x34, 0x09, 0xa3, 0xf1, 0x65, 0x4a, 0x08, 0xb9
};

static_assert( std::size( kRandomBytes ) == 1024 );

auto SimulatedHullsSystem::HullCloudDynamicMesh::fillMeshBuffers( const float *__restrict viewOrigin,
																  const float *__restrict viewAxis,
																  float cameraFovTangent,
																  const Scene::DynamicLight *lights,
																  std::span<const uint16_t> affectingLightIndices,
																  vec4_t *__restrict destPositions,
																  vec4_t *__restrict destNormals,
																  vec2_t *__restrict destTexCoords,
																  byte_vec4_t *__restrict destColors,
																  uint16_t *__restrict destIndices ) const
-> std::pair<unsigned, unsigned> {
    unsigned numResultVertices = 0;
	unsigned numResultIndices = 0;

    if( !m_shared->isAKeyframedHull ){
        assert(m_shared->simulatedSubdivLevel < BasicHullsHolder::kMaxSubdivLevel);
        assert(m_chosenSubdivLevel <= m_shared->simulatedSubdivLevel + 1);
        assert(m_shared->minZLastFrame <= m_shared->maxZLastFrame);

        // Keep always allocating the default buffer even if it's unused, so we can rely on its availability
        const auto colorsBufferLevel = wsw::min(m_chosenSubdivLevel, m_shared->simulatedSubdivLevel);
        const auto colorsBufferSize = (unsigned) basicHullsHolder.getIcosphereForLevel(
                colorsBufferLevel).vertices.size();
        assert(colorsBufferSize && colorsBufferSize < (1 << 12));
        auto *const overrideColorsBuffer = (byte_vec4_t *) alloca(sizeof(byte_vec4_t) * colorsBufferSize);

        const byte_vec4_t *overrideColors = getOverrideColorsCheckingSiblingCache(overrideColorsBuffer, viewOrigin,
                                                                                  viewAxis, lights,
                                                                                  affectingLightIndices);

        const byte_vec4_t *colorsToUse = overrideColors ? overrideColors : m_shared->simulatedColors;

        alignas(16) vec4_t viewLeft, viewUp;
        // TODO: Flip if needed
        VectorCopy(&viewAxis[AXIS_RIGHT], viewLeft);
        VectorCopy(&viewAxis[AXIS_UP], viewUp);

        alignas(16) vec4_t normal;
        VectorNegate(&viewAxis[AXIS_FORWARD], normal);
        normal[3] = 0.0f;

        const float radius = m_spriteRadius;
        constexpr float normalizer = 1.0f / 255.0f;

        // We sample the random data by vertex numbers using random shifts that remain stable during the hull lifetime
        assert(m_vertexNumLimitThisFrame + m_phaseIndexShiftInTable <= std::size(kRandomBytes));
        assert(m_vertexNumLimitThisFrame + m_speedIndexShiftInTable <= std::size(kRandomBytes));

        unsigned vertexNum = m_minVertexNumThisFrame;
        assert(vertexNum < m_vertexNumLimitThisFrame);

        [[maybe_unused]] vec3_t tmpLeftStorage, tmpUpStorage;

        do {
            const float *__restrict vertexPosition = m_shared->simulatedPositions[vertexNum];
            const uint8_t *const __restrict vertexColor = colorsToUse[vertexNum];

            vec4_t *const __restrict positions = destPositions + numResultVertices; /// D:\qfusion\source\cgame\simulatedhullssystem.cpp(3651): warning C4700: uninitialized local variable 'numResultVertices' used [D:\qfusion\source\cmake-build-release\client\warsow.vcxproj]
            vec4_t *const __restrict normals = destNormals + numResultVertices;
            byte_vec4_t *const __restrict colors = destColors + numResultVertices;
            vec2_t *const __restrict texCoords = destTexCoords + numResultVertices;
            uint16_t *const __restrict indices = destIndices + numResultIndices;

            // Test the color alpha first for an early cutoff

            byte_vec4_t resultingColor;
            resultingColor[3] = (uint8_t) wsw::clamp((float) vertexColor[3] * m_spriteColor[3] * m_alphaScale, 0.0f,
                                                     255.0f);
            if (resultingColor[3] < 1) {
                continue;
            }

            resultingColor[0] = (uint8_t) wsw::clamp((float) vertexColor[0] * m_spriteColor[0], 0.0f, 255.0f);
            resultingColor[1] = (uint8_t) wsw::clamp((float) vertexColor[1] * m_spriteColor[1], 0.0f, 255.0f);
            resultingColor[2] = (uint8_t) wsw::clamp((float) vertexColor[2] * m_spriteColor[2], 0.0f, 255.0f);

            Vector4Copy(resultingColor, colors[0]);
            Vector4Copy(resultingColor, colors[1]);
            Vector4Copy(resultingColor, colors[2]);
            Vector4Copy(resultingColor, colors[3]);

            // 1 unit is equal to a rotation of 360 degrees
            const float initialPhase =
                    ((float) kRandomBytes[vertexNum + m_phaseIndexShiftInTable] - 127.0f) * normalizer;
            const float angularSpeed =
                    ((float) kRandomBytes[vertexNum + m_speedIndexShiftInTable] - 127.0f) * normalizer;
            const float rotationDegrees = 360.0f * (initialPhase + angularSpeed * m_lifetimeSeconds);

            const float *left, *up;
            // TODO: Avoid dynamic branching, add templated specializations?
            if (m_applyRotation) {
                // TODO: This could be probably reduced to a single sincos() calculation + few vector transforms
                RotatePointAroundVector(tmpLeftStorage, normal, viewLeft, rotationDegrees);
                RotatePointAroundVector(tmpUpStorage, normal, viewUp, rotationDegrees);

                left = tmpLeftStorage;
                up = tmpUpStorage;
            } else {
                left = viewLeft;
                up = viewUp;
            }

            vec3_t point;
            VectorMA(vertexPosition, -radius, up, point);
            VectorMA(point, +radius, left, positions[0]);
            VectorMA(point, -radius, left, positions[3]);

            VectorMA(vertexPosition, radius, up, point);
            VectorMA(point, +radius, left, positions[1]);
            VectorMA(point, -radius, left, positions[2]);

            positions[0][3] = positions[1][3] = positions[2][3] = positions[3][3] = 1.0f;

            Vector4Copy(normal, normals[0]);
            Vector4Copy(normal, normals[1]);
            Vector4Copy(normal, normals[2]);
            Vector4Copy(normal, normals[3]);

            VectorSet(indices + 0, numResultVertices + 0, numResultVertices + 1, numResultVertices + 2);
            VectorSet(indices + 3, numResultVertices + 0, numResultVertices + 2, numResultVertices + 3);

            Vector2Set(texCoords[0], 0.0f, 1.0f);
            Vector2Set(texCoords[1], 0.0f, 0.0f);
            Vector2Set(texCoords[2], 1.0f, 0.0f);
            Vector2Set(texCoords[3], 1.0f, 1.0f);

            numResultVertices += 4;
            numResultIndices += 6;
        } while (++vertexNum < m_vertexNumLimitThisFrame);
    } else {
		// the correct LOD has already been assigned in getSotrageRequirements()
		StaticCagedMesh *meshToRender = m_shared->meshToRender;
		const float lifetimeFrac = m_shared->lifetimeFrac;

		const tri *cageTriIndices = m_shared->cageTriIndices;
		const StaticCageCoordinate *vertCoords = meshToRender->vertexCoordinates;
        const float *offsetsFromLim = meshToRender->offsetFromLim;

		const float *limitsAtDirections    = m_shared->limitsAtDirections;
		const vec3_t *vertexMoveDirections = m_shared->cageVertexPositions;
		const float scale                  = m_shared->scale;
		const float *origin                = m_shared->origin;

		// write positions to destPositions
		// memcpy indices from mesh to render
		const unsigned numFrames    = meshToRender->numFrames;
        const unsigned numTris      = meshToRender->triIndices.size();
		const unsigned numGridVerts = meshToRender->numVertices;

        const float fractionOfParticlesToRender = m_fractionOfParticlesToRender;
        const auto numParticlesToRender         = (unsigned)( fractionOfParticlesToRender * numGridVerts );
        const float rcpFractionToRender         = 1 / fractionOfParticlesToRender;

        const tri *meshTriIndices = meshToRender->triIndices.data();

        unsigned currFrame;
        float lerpFrac;
        if( v_frameToShow.get() < 0 ){
            currFrame = wsw::min( (unsigned) ((float) numFrames * lifetimeFrac), numFrames - 1 );
            lerpFrac  = (float)numFrames * lifetimeFrac - (float)currFrame;
        } else {
            currFrame = v_frameToShow.get();
            lerpFrac  = 0.0f;
        }
        const unsigned nextFrame   = wsw::min( currFrame + 1, numFrames - 1 );
        const float complementFrac = 1.0f - lerpFrac;

        unsigned currFrameStartVertIdx = currFrame * numGridVerts;
        unsigned nextFrameStartVertIdx = nextFrame * numGridVerts;

        bool needsViewDotResults   = false;
        float *viewDotResults      = nullptr;
        vec3_t *gridVertexPosition = nullptr;

        const unsigned numShadingLayers = meshToRender->numShadingLayers;

        for(  unsigned layerNum = 0; layerNum < numShadingLayers; ++layerNum ) {
            const ShadingLayer &layer = meshToRender->shadingLayers[layerNum];
            if( !std::holds_alternative<MaskedShadingLayer>( layer ) ) {
                assert( std::holds_alternative<DotShadingLayer>( layer ) || std::holds_alternative<CombinedShadingLayer>( layer ) );
                needsViewDotResults = true;
                break;
            }
        }

        alignas(16) vec4_t viewLeft, viewUp;
        // TODO: Flip if needed
        VectorCopy(&viewAxis[AXIS_RIGHT], viewLeft);
        VectorCopy(&viewAxis[AXIS_UP], viewUp);

        alignas(16) vec4_t normal;
        VectorNegate(&viewAxis[AXIS_FORWARD], normal);
        normal[3] = 0.0f;

        const float radius = m_spriteRadius;
        constexpr float normalizer = 1.0f / 255.0f;

        // We sample the random data by vertex numbers using random shifts that remain stable during the hull lifetime
        assert(m_vertexNumLimitThisFrame + m_phaseIndexShiftInTable <= std::size(kRandomBytes));
        assert(m_vertexNumLimitThisFrame + m_speedIndexShiftInTable <= std::size(kRandomBytes));

        unsigned gridVertNum = 0;
        assert(vertexNum < m_vertexNumLimitThisFrame);

        [[maybe_unused]] vec3_t tmpLeftStorage, tmpUpStorage;

        if( needsViewDotResults ){
            gridVertexPosition = (vec3_t *)alloca( sizeof( vec3_t ) * numGridVerts );

            do {
                unsigned currFrameVertIdx = currFrameStartVertIdx + gridVertNum;
                unsigned nextFrameVertIdx = nextFrameStartVertIdx + gridVertNum;
                vec3_t nextPositionContrib;

                vertexPosFromStaticCage( vertCoords[currFrameVertIdx], scale * complementFrac, cageTriIndices,
                                         vertexMoveDirections, limitsAtDirections, offsetsFromLim[currFrameVertIdx],
										 gridVertexPosition[gridVertNum] );
                vertexPosFromStaticCage( vertCoords[nextFrameVertIdx], scale * lerpFrac, cageTriIndices,
                                         vertexMoveDirections, limitsAtDirections, offsetsFromLim[nextFrameVertIdx],
										 nextPositionContrib );
                VectorAdd( gridVertexPosition[gridVertNum], nextPositionContrib, gridVertexPosition[gridVertNum] );
                VectorAdd( gridVertexPosition[gridVertNum], origin, gridVertexPosition[gridVertNum] );
            } while ( ++gridVertNum < numGridVerts );

            gridVertNum = 0;

            do {
                const auto vertToRenderIdx = (unsigned)( (float)gridVertNum * rcpFractionToRender );

                vec4_t *const __restrict positions = destPositions + numResultVertices;
                vec4_t *const __restrict normals = destNormals + numResultVertices;
                byte_vec4_t *const __restrict colors = destColors + numResultVertices;
                vec2_t *const __restrict texCoords = destTexCoords + numResultVertices;
                uint16_t *const __restrict indices = destIndices + numResultIndices;

                // 1 unit is equal to a rotation of 360 degrees
                const float initialPhase =
                        ((float) kRandomBytes[gridVertNum + m_phaseIndexShiftInTable] - 127.0f) * normalizer;
                const float angularSpeed =
                        ((float) kRandomBytes[gridVertNum + m_speedIndexShiftInTable] - 127.0f) * normalizer;
                /// IMPORTANT: won't kRandomBytes[vertexNum + shift] go out of index? the array has 1024 entries and vertexNum can be >1024, while shift<=255.
                const float rotationDegrees = 360.0f * (initialPhase + angularSpeed * m_lifetimeSeconds);

                const float *left, *up;
                // TODO: Avoid dynamic branching, add templated specializations?
                if (m_applyRotation) {
                    // TODO: This could be probably reduced to a single sincos() calculation + few vector transforms
                    RotatePointAroundVector(tmpLeftStorage, normal, viewLeft, rotationDegrees);
                    RotatePointAroundVector(tmpUpStorage, normal, viewUp, rotationDegrees);

                    left = tmpLeftStorage;
                    up = tmpUpStorage;
                } else {
                    left = viewLeft;
                    up = viewUp;
                }

                vec3_t point;
                VectorMA(gridVertexPosition[vertToRenderIdx], -radius, up, point);
                VectorMA(point, +radius, left, positions[0]);
                VectorMA(point, -radius, left, positions[3]);

                VectorMA(gridVertexPosition[vertToRenderIdx], radius, up, point);
                VectorMA(point, +radius, left, positions[1]);
                VectorMA(point, -radius, left, positions[2]);

                positions[0][3] = positions[1][3] = positions[2][3] = positions[3][3] = 1.0f;

                Vector4Copy(normal, normals[0]);
                Vector4Copy(normal, normals[1]);
                Vector4Copy(normal, normals[2]);
                Vector4Copy(normal, normals[3]);

                VectorSet(indices + 0, numResultVertices + 0, numResultVertices + 1, numResultVertices + 2);
                VectorSet(indices + 3, numResultVertices + 0, numResultVertices + 2, numResultVertices + 3);

                Vector2Set(texCoords[0], 0.0f, 1.0f);
                Vector2Set(texCoords[1], 0.0f, 0.0f);
                Vector2Set(texCoords[2], 1.0f, 0.0f);
                Vector2Set(texCoords[3], 1.0f, 1.0f);

                numResultVertices += 4;
                numResultIndices += 6;
            } while ( ++gridVertNum < numParticlesToRender );

            assert( numVertices > 0 && numVertices < 4096 );
            viewDotResults = (float *)alloca( sizeof( float ) * numParticlesToRender );

            auto *meshNormals = (vec3_t *)alloca( sizeof( vec3_t ) * numGridVerts );
            for( unsigned vertNum = 0; vertNum < numGridVerts; vertNum++ ){
                VectorSet( meshNormals[vertNum], 0.0f, 0.0f, 0.0f );
            }

            unsigned triNum = 0;
            do {
                const unsigned firstVertIdx  = meshTriIndices[triNum][0];
                const unsigned secondVertIdx = meshTriIndices[triNum][1];
                const unsigned thirdVertIdx  = meshTriIndices[triNum][2];

                vec3_t triCoords[3];
                VectorCopy( gridVertexPosition[firstVertIdx], triCoords[0] );
                VectorCopy( gridVertexPosition[secondVertIdx], triCoords[1] );
                VectorCopy( gridVertexPosition[thirdVertIdx], triCoords[2] );

                vec3_t vecToSecondVert;
                vec3_t vecToThirdVert;
                VectorSubtract( triCoords[1], triCoords[0], vecToSecondVert );
                VectorSubtract( triCoords[2], triCoords[0], vecToThirdVert );

                vec3_t normalContrib;
                CrossProduct( vecToThirdVert, vecToSecondVert, normalContrib );

                VectorAdd( meshNormals[firstVertIdx], normalContrib, meshNormals[firstVertIdx] );
                VectorAdd( meshNormals[secondVertIdx], normalContrib, meshNormals[secondVertIdx] );
                VectorAdd( meshNormals[thirdVertIdx], normalContrib, meshNormals[thirdVertIdx] );
            } while( ++triNum < numTris );

            for( unsigned vertNum = 0; vertNum < numParticlesToRender; vertNum++ ){
                const unsigned vertToRenderIdx = vertNum * rcpFractionToRender;

                const float *const __restrict currVertex = gridVertexPosition[vertToRenderIdx];
                const float *const __restrict currNormal = meshNormals[vertToRenderIdx];

                const float squaredNormalLength = VectorLengthSquared( currNormal );
                vec3_t viewDir;
                VectorSubtract( currVertex, viewOrigin, viewDir );
                const float squareDistanceToVertex = VectorLengthSquared( viewDir );

                const float squareRcpNormalizingFactor = squaredNormalLength * squareDistanceToVertex;
                // check that both the length of the normal and distance to vertex are not 0 in one branch
                if( squareRcpNormalizingFactor > 1.0f ) [[likely]] {
                    const float normalizingFactor = Q_RSqrt( squareRcpNormalizingFactor );
                    viewDotResults[vertNum] = std::fabs( DotProduct( viewDir, currNormal ) ) * normalizingFactor;
                } else {
                    viewDotResults[vertNum] = 0.0f;
                }
            }
        } else {
            do {
                const unsigned vertToRenderIdx = gridVertNum * rcpFractionToRender;

                vec3_t gridVertPosition;

                unsigned currFrameVertIdx = currFrameStartVertIdx + vertToRenderIdx;
                unsigned nextFrameVertIdx = nextFrameStartVertIdx + vertToRenderIdx;
                vec3_t nextPositionContrib;

                vertexPosFromStaticCage( vertCoords[currFrameVertIdx], scale * complementFrac, cageTriIndices,
                                        vertexMoveDirections, limitsAtDirections, offsetsFromLim[currFrameVertIdx],
										gridVertPosition );
                vertexPosFromStaticCage( vertCoords[nextFrameVertIdx], scale * lerpFrac, cageTriIndices,
                                        vertexMoveDirections, limitsAtDirections, offsetsFromLim[nextFrameVertIdx],
										nextPositionContrib );
                VectorAdd(gridVertPosition, nextPositionContrib, gridVertPosition);
                VectorAdd(gridVertPosition, origin, gridVertPosition);

                vec4_t *const __restrict positions = destPositions + numResultVertices;
                vec4_t *const __restrict normals = destNormals + numResultVertices;
                byte_vec4_t *const __restrict colors = destColors + numResultVertices;
                vec2_t *const __restrict texCoords = destTexCoords + numResultVertices;
                uint16_t *const __restrict indices = destIndices + numResultIndices;

                // 1 unit is equal to a rotation of 360 degrees
                const float initialPhase =
                        ((float) kRandomBytes[gridVertNum + m_phaseIndexShiftInTable] - 127.0f) * normalizer;
                const float angularSpeed =
                        ((float) kRandomBytes[gridVertNum + m_speedIndexShiftInTable] - 127.0f) * normalizer;
                /// IMPORTANT: won't kRandomBytes[vertexNum + shift] go out of index? the array has 1024 entries and vertexNum can be >1024, while shift<=255.
                const float rotationDegrees = 360.0f * (initialPhase + angularSpeed * m_lifetimeSeconds);

                const float *left, *up;
                // TODO: Avoid dynamic branching, add templated specializations?
                if (m_applyRotation) {
                    // TODO: This could be probably reduced to a single sincos() calculation + few vector transforms
                    RotatePointAroundVector(tmpLeftStorage, normal, viewLeft, rotationDegrees);
                    RotatePointAroundVector(tmpUpStorage, normal, viewUp, rotationDegrees);

                    left = tmpLeftStorage;
                    up = tmpUpStorage;
                } else {
                    left = viewLeft;
                    up = viewUp;
                }

                vec3_t point;
                VectorMA(gridVertPosition, -radius, up, point);
                VectorMA(point, +radius, left, positions[0]);
                VectorMA(point, -radius, left, positions[3]);

                VectorMA(gridVertPosition, radius, up, point);
                VectorMA(point, +radius, left, positions[1]);
                VectorMA(point, -radius, left, positions[2]);

                positions[0][3] = positions[1][3] = positions[2][3] = positions[3][3] = 1.0f;

                Vector4Copy(normal, normals[0]);
                Vector4Copy(normal, normals[1]);
                Vector4Copy(normal, normals[2]);
                Vector4Copy(normal, normals[3]);

                VectorSet(indices + 0, numResultVertices + 0, numResultVertices + 1, numResultVertices + 2);
                VectorSet(indices + 3, numResultVertices + 0, numResultVertices + 2, numResultVertices + 3);

                Vector2Set(texCoords[0], 0.0f, 1.0f);
                Vector2Set(texCoords[1], 0.0f, 0.0f);
                Vector2Set(texCoords[2], 1.0f, 0.0f);
                Vector2Set(texCoords[3], 1.0f, 1.0f);

                numResultVertices += 4;
                numResultIndices += 6;
            } while ( ++gridVertNum < numParticlesToRender );
        }


        const auto beforeViewDot = Sys_Microseconds();
        if( v_showPerf.get() ) {
            Com_Printf( "viewdot results took %d micros for %d vertices and %d indices\n",
                        (int) (Sys_Microseconds() - beforeViewDot), (int) numResultVertices, (int) numResultIndices );
        }

        for( unsigned layerNum = 0; layerNum < numShadingLayers; ++layerNum ) {
            const ShadingLayer &prevShadingLayer = meshToRender->shadingLayers[currFrame * numShadingLayers +
                                                                               layerNum];
            const ShadingLayer &nextShadingLayer = meshToRender->shadingLayers[nextFrame * numShadingLayers +
                                                                               layerNum];

            const auto beforeLayer = Sys_Microseconds();
            if (const auto *const prevMaskedLayer = std::get_if<MaskedShadingLayer>(&prevShadingLayer)) {
                const auto *const nextMaskedLayer = std::get_if<MaskedShadingLayer>(&nextShadingLayer);

                const unsigned numColors = prevMaskedLayer->colors.size();
                assert(numColors > 0 && numColors <= kMaxLayerColors);

                byte_vec4_t lerpedColors[kMaxLayerColors];
                float lerpedColorRanges[kMaxLayerColors];
                lerpLayerColorsAndRangesBetweenFrames(lerpedColors, lerpedColorRanges, numColors, lerpFrac,
                                                      prevMaskedLayer->colors.data(),
                                                      nextMaskedLayer->colors.data(),
                                                      prevMaskedLayer->colorRanges, nextMaskedLayer->colorRanges);

                unsigned vertexNum = 0;
                do {
                    const unsigned vertToRenderIdx = vertexNum * rcpFractionToRender;

                    byte_vec4_t *const __restrict colors = destColors + vertexNum * 4;
                    const float vertexMaskValue = std::lerp(prevMaskedLayer->vertexMaskValues[vertToRenderIdx],
                                                            nextMaskedLayer->vertexMaskValues[vertToRenderIdx],
                                                            lerpFrac);

                    addLayerContributionToResultColor(vertexMaskValue, numColors, lerpedColors, lerpedColorRanges,
                                                      prevMaskedLayer->blendMode, prevMaskedLayer->alphaMode,
                                                      colors[0], layerNum);
                    Vector4Copy(colors[0], colors[1]);
                    Vector4Copy(colors[0], colors[2]);
                    Vector4Copy(colors[0], colors[3]);

                } while (++vertexNum < numParticlesToRender);
            } else if (const auto *const prevDotLayer = std::get_if<DotShadingLayer>(&prevShadingLayer)) {
                const auto *const nextDotLayer = std::get_if<DotShadingLayer>(&nextShadingLayer);

                const unsigned numColors = prevDotLayer->colors.size();
                assert(numColors > 0 && numColors <= kMaxLayerColors);

                byte_vec4_t lerpedColors[kMaxLayerColors];
                float lerpedColorRanges[kMaxLayerColors];
                lerpLayerColorsAndRangesBetweenFrames(lerpedColors, lerpedColorRanges, numColors, lerpFrac,
                                                      prevDotLayer->colors.data(), nextDotLayer->colors.data(),
                                                      prevDotLayer->colorRanges, nextDotLayer->colorRanges);

                unsigned vertexNum = 0;
                do {
                    byte_vec4_t *const __restrict colors = destColors + vertexNum * 4;
                    addLayerContributionToResultColor(viewDotResults[vertexNum], numColors, lerpedColors,
                                                      lerpedColorRanges,
                                                      prevDotLayer->blendMode, prevDotLayer->alphaMode,
                                                      colors[0], layerNum);
                    Vector4Copy(colors[0], colors[1]);
                    Vector4Copy(colors[0], colors[2]);
                    Vector4Copy(colors[0], colors[3]);

                } while (++vertexNum < numParticlesToRender);
            } else if (const auto *const prevCombinedLayer = std::get_if<CombinedShadingLayer>(&prevShadingLayer)) {
                const auto *const nextCombinedLayer = std::get_if<CombinedShadingLayer>(&nextShadingLayer);

                const unsigned numColors = prevCombinedLayer->colors.size();
                assert(numColors > 0 && numColors <= kMaxLayerColors);

                byte_vec4_t lerpedColors[kMaxLayerColors];
                float lerpedColorRanges[kMaxLayerColors];
                lerpLayerColorsAndRangesBetweenFrames(lerpedColors, lerpedColorRanges, numColors, lerpFrac,
                                                      prevCombinedLayer->colors.data(),
                                                      nextCombinedLayer->colors.data(),
                                                      prevCombinedLayer->colorRanges,
                                                      nextCombinedLayer->colorRanges);

                unsigned vertexNum = 0;
                do {
                    const unsigned vertToRenderIdx = vertexNum * rcpFractionToRender;

                    byte_vec4_t *const __restrict colors = destColors + vertexNum * 4;
                    const float vertexDotValue = viewDotResults[vertexNum];
                    const float vertexMaskValue = std::lerp(prevCombinedLayer->vertexMaskValues[vertToRenderIdx],
                                                            nextCombinedLayer->vertexMaskValues[vertToRenderIdx],
                                                            lerpFrac);

                    const float dotInfluence = prevCombinedLayer->dotInfluence;
                    const float maskInfluence = 1.0f - dotInfluence;
                    const float vertexCombinedValue =
                            vertexDotValue * dotInfluence + vertexMaskValue * maskInfluence;

                    addLayerContributionToResultColor(vertexCombinedValue, numColors, lerpedColors,
                                                      lerpedColorRanges,
                                                      prevCombinedLayer->blendMode, prevCombinedLayer->alphaMode,
                                                      colors[0], layerNum);
                    Vector4Copy(colors[0], colors[1]);
                    Vector4Copy(colors[0], colors[2]);
                    Vector4Copy(colors[0], colors[3]);

                } while (++vertexNum < numParticlesToRender);
            } else {
                wsw::failWithLogicError("Unreachable");
            }
        }
    }

	return { numResultVertices, numResultIndices };
};