#ifndef WSW_25e6c863_70a4_4d36_9597_133f93b31f91_H
#define WSW_25e6c863_70a4_4d36_9597_133f93b31f91_H

#include <span>

#include "../ref/ref.h"
#include "../common/randomgenerator.h"
#include "../common/freelistallocator.h"
#include "../common/podbufferholder.h"
#include "../common/wswvector.h"
#include "../common/geometry.h"
#include "../common/wswstring.h"
#include "polyeffectssystem.h"
#include "../common/wswstringview.h"

struct CMShapeList;

class SimulatedHullsSystem {
	friend class TransientEffectsSystem;
	friend class MeshTesselationHelper;
public:
    static std::span<const vec4_t> getUnitIcosphere( unsigned level );
	// TODO: Split function and fading direction?
	enum class ViewDotFade : uint8_t {
		NoFade, FadeOutContour, FadeOutCenterLinear, FadeOutCenterQuadratic, FadeOutCenterCubic
	};
	enum class ZFade : uint8_t { NoFade, FadeOutBottom };

	// TODO: Fade/light props should logically belong to these rules

	struct SolidAppearanceRules {
		shader_s *material { nullptr };
	};

	struct CloudMeshProps {
		shader_s *material { nullptr };
		vec4_t overlayColor { 1.0f, 1.0f, 1.0f, 1.0f };

		ValueLifespan alphaScaleLifespan {
			.initial                      = 0.0f,
			.fadedIn                      = 1.0f,
			.fadedOut                     = 0.0f,
			.finishFadingInAtLifetimeFrac = 0.37f,
			.startFadingOutAtLifetimeFrac = 0.67f,
		};

		ValueLifespan radiusLifespan {
			.initial                      = 0.0f,
			.fadedIn                      = 16.0f,
			.fadedOut                     = 0.0f,
			.finishFadingInAtLifetimeFrac = 0.37f,
			.startFadingOutAtLifetimeFrac = 0.67f,
		};

		// Feasible values of these properties must be non-positive
		int tessLevelShiftForMinVertexIndex { 0 };
		int tessLevelShiftForMaxVertexIndex { 0 };
		// Feasible values are non-positive (if zero, gets shown only for the extra tesselated lod)
		int shiftFromDefaultLevelToHide { std::numeric_limits<int>::min() };
		bool applyRotation { false };
	};

	struct CloudAppearanceRules {
		std::span<const CloudMeshProps> spanOfMeshProps;
	};

	struct SolidAndCloudAppearanceRules {
		SolidAppearanceRules solidRules;
		CloudAppearanceRules cloudRules;
	};

	using AppearanceRules = std::variant<SolidAppearanceRules, CloudAppearanceRules, SolidAndCloudAppearanceRules>;

	struct ColorChangeTimelineNode {
		// Specifying it as a fraction is more flexible than absolute offsets
		float activateAtLifetimeFraction { 0.0f };
		// Colors get chosen randomly during replacement from this span
		std::span<const byte_vec4_t> replacementPalette;
		// 1.0f does not guarantee a full replacement
		float sumOfDropChanceForThisSegment { 0.0f };
		// 1.0f does not guarantee a full replacement
		float sumOfReplacementChanceForThisSegment { 0.0f };
		// True if vertex colors may be replaced by more opaque colors
		bool allowIncreasingOpacity { false };
	};

	struct HullLayerParams {
		float speed;
		float finalOffset;
		float speedSpikeChance;
		float minSpeedSpike, maxSpeedSpike;
		float biasAlongChosenDir;
		float baseInitialColor[4];
		float bulgeInitialColor[4];

		std::span<const ColorChangeTimelineNode> colorChangeTimeline;
	};

	static constexpr unsigned kMaxLayerColors = 8;
    static constexpr unsigned kMaxSharedCageCagedMeshes = 16;

	enum class BlendMode : unsigned {
		AlphaBlend,
		Add,
		Subtract
	};
	enum class AlphaMode : unsigned {
		Add,
		Subtract,
		Override
	};

	struct MaskedShadingLayer {
		const float *vertexMaskValues { nullptr };
		std::span<const byte_vec4_t> colors;
		float colorRanges[kMaxLayerColors];
		BlendMode blendMode { BlendMode::AlphaBlend };
		AlphaMode alphaMode { AlphaMode::Add };
	};

	struct DotShadingLayer {
		std::span<const byte_vec4_t> colors;
		float colorRanges[kMaxLayerColors];
		BlendMode blendMode { BlendMode::AlphaBlend };
		AlphaMode alphaMode { AlphaMode::Add };
	};

	struct CombinedShadingLayer {
		const float *vertexMaskValues { nullptr };
		std::span<const byte_vec4_t> colors;
		float colorRanges[kMaxLayerColors];
		// Between 0 and 1, vertexValue = ( dotInfluence - 1 ) * vertexMaskValue + dotInfluence * dotProduct(viewVector, normal)
		float dotInfluence { 0.5f };
		BlendMode blendMode { BlendMode::AlphaBlend };
		AlphaMode alphaMode { AlphaMode::Add };
	};

	using ShadingLayer = std::variant<MaskedShadingLayer, DotShadingLayer, CombinedShadingLayer>;

private: struct KeyframedHull;

public:
    struct StaticCage {
        Geometry cageGeometry;
        wsw::String identifier;
		float boundingRadius; /// maybe this should combine with scale from params?
        wsw::HeapBasedFreelistAllocator *allocator;
        KeyframedHull *head;
    };

    wsw::Vector<StaticCage> m_loadedStaticCages;

    struct StaticCageCoordinate {
        unsigned cageTriIdx;
        float coordsOnCageTri[2]; // barycentric coordinates
        float offset; // offset along direction
        /// ? float moveDir ? precompute the moveDir? -> this should be impossible i guess
        //float offsetFromLimit; // offset from boundaries like walls
    };

    struct StaticCagedMesh {
		unsigned loadedCageKey;
        StaticCageCoordinate *vertexCoordinates;
        unsigned numVertices;
        unsigned numFrames;
        std::span<tri> triIndices;
        ShadingLayer *shadingLayers;
		unsigned numShadingLayers;
        float *boundingRadii; // maximum radius of the mesh for a given frame for culling
        StaticCagedMesh *nextLOD { nullptr }; // pointer to the next lower detail version
    };

    struct StaticKeyframedHullParams {
        vec3_t origin { 0.0f, 0.0f, 0.0f };
        vec3_t offset { 0.0f, 0.0f, 0.0f };
        vec3_t dir { 0.0f, 0.0f, 1.0f };
        float rotation { 0.0f };
        float scale { 32.0f };
        unsigned timeout { 400u };
        StaticCagedMesh *sharedCageCagedMeshes { nullptr };
        unsigned numCagedMeshes { 1u };
    };

    unsigned maxCagedHullsPerType = 64;

	static void vertexPosFromStaticCage( unsigned vertIdx, const StaticCageCoordinate *vertCoords, float scale,
							      const tri *cageTriIndices, const vec3_t *vertexMoveDirections,
							      const float *limitsAtDirections, const vec3_t origin, vec3_t outPos );

    void RegisterStaticCage( const wsw::String &identifier );

    StaticCagedMesh *RegisterStaticCagedMesh( const char *name );

	void applyShading( wsw::StringView pathToMesh, SimulatedHullsSystem::StaticCagedMesh *cagedMesh ); // this stuff is terrible, remove ASAP, should be done on GPU WITH SHADERS

	struct OffsetKeyframe {
		float lifetimeFraction { 0.0f };
		const float *offsets { nullptr };
		// The offset from an obstacle
		const float *offsetsFromLimit { nullptr };
		std::span<const ShadingLayer> shadingLayers;
	};

	SimulatedHullsSystem();
	~SimulatedHullsSystem();

	void clear();

	void simulateFrameAndSubmit( int64_t currTime, DrawSceneRequest *request );
private:
	static constexpr unsigned kNumVerticesForSubdivLevel[5] { 12, 42, 162, 642, 2562 };

	// We have to supply solid and cloud parts separately for a proper handling of surface sorting in the renderer.
	// Still, they share many properties.
	struct SharedMeshData {
		wsw::Vector<uint32_t> *overrideColorsBuffer { nullptr };
		// Must be reset each frame
		std::optional<std::variant<std::pair<unsigned, unsigned>, std::monostate>> cachedOverrideColorsSpanInBuffer;
		std::optional<std::variant<unsigned, std::monostate>> cachedChosenSolidSubdivLevel;
		const vec4_t *simulatedPositions { nullptr };
		const vec4_t *simulatedNormals { nullptr };
		const byte_vec4_t *simulatedColors { nullptr };
		unsigned simulatedSubdivLevel { 0 };
		float nextLodTangentRatio { 0.18f };
		float minZLastFrame { 0.0f };
		float maxZLastFrame { 0.0f };
		float minFadedOutAlpha { 0.0f };
		ViewDotFade viewDotFade { ViewDotFade::NoFade };
		ZFade zFade { ZFade::NoFade };
		bool tesselateClosestLod { false };
		bool lerpNextLevelColors { false };
		bool hasSibling { false };

		// currently for keyframed hull shading
		bool isAKeyframedHull { false };
        /// DELETE
		//float lerpFrac { 0.0f };
		//std::span<const ShadingLayer> prevShadingLayers;
		//std::span<const ShadingLayer> nextShadingLayers;
        /// DELETE END

        /// for caged hulls
        float lifetimeFrac { 0.0f };
        StaticCagedMesh *meshToRender { nullptr };
        StaticCage *cage { nullptr };
        //unsigned numMeshesToRender { 1u };
        float scale { 35.0f };
        vec3_t origin { 0.0f, 0.0f, 0.0f };
        //Geometry *cage { nullptr };
		vec3_t *cageVertexPositions { nullptr };
		tri *cageTriIndices { nullptr };
        float *limitsAtDirections { nullptr };
        /// for caged hulls END
	};

    //draft
    struct SharedCageMeshData {
        std::optional<std::variant<unsigned, std::monostate>> cachedChosenSolidSubdivLevel; // maybe ?
        const vec4_t *cageVertexMoveDirs;
        const float *cageVertexLimits;
        float nextLodTangentRatio { 0.18f };
        bool hasSibling { false };

        float lifetimeFrac;
        std::span<const ShadingLayer> prevShadingLayers;
        std::span<const ShadingLayer> nextShadingLayers;

        //wsw::Vector<uint32_t> *overrideColorsBuffer { nullptr }; NO NEED DIRECTLY COMPUTE COLORS
        // Must be reset each frame
        // std::optional<std::variant<std::pair<unsigned, unsigned>, std::monostate>> cachedOverrideColorsSpanInBuffer; X
        // const vec4_t *simulatedPositions { nullptr }; NO NEED DIRECTLY COMPUTE POSITIONS
        //const vec4_t *simulatedNormals { nullptr }; COMPUTE DIRECTLY
        //const byte_vec4_t *simulatedColors { nullptr }; COMPUTE DIRECTLY
        //unsigned simulatedSubdivLevel { 0 }; THERE IS NO "SUBDIV LEVEL"
        //float minZLastFrame { 0.0f }; X
        //float maxZLastFrame { 0.0f }; X
        //float minFadedOutAlpha { 0.0f }; X
        //ViewDotFade viewDotFade { ViewDotFade::NoFade }; X
        //ZFade zFade { ZFade::NoFade }; X
        //bool tesselateClosestLod { false }; X
        //bool lerpNextLevelColors { false }; X
        // currently for keyframed hull shading
        //bool isAKeyframedHull { false }; X
        //float lerpFrac { 0.0f };
    };

	class HullDynamicMesh : public DynamicMesh {
		friend class SimulatedHullsSystem;
		friend class MeshTesselationHelper;

		SharedMeshData *m_shared { nullptr };
		// Gets set in getStorageRequirements()
		mutable unsigned m_chosenSubdivLevel { 0 };

		[[nodiscard]]
		auto calcSolidSubdivLodLevel( const float *viewOrigin, float cameraViewTangent ) const -> std::optional<unsigned>;

		void calcOverrideColors( byte_vec4_t *__restrict buffer,
								 const float *__restrict viewOrigin,
								 const float *__restrict viewAxis,
								 const Scene::DynamicLight *lights,
								 std::span<const uint16_t> affectingLightIndices,
								 bool applyViewDotFade, bool applyZFade, bool applyLights ) const;

		[[nodiscard]]
		auto getOverrideColorsCheckingSiblingCache( byte_vec4_t *__restrict localBuffer,
													const float *__restrict viewOrigin,
													const float *__restrict viewAxis,
													const Scene::DynamicLight *lights,
													std::span<const uint16_t> affectingLightIndices ) const
														-> const byte_vec4_t *;

        /*auto copyColors( const unsigned numVertices, const byte_vec4_t* colors, const float* colorRanges ) const
        -> const byte_vec4_t *;*/
	};

	class HullSolidDynamicMesh : public HullDynamicMesh {
		friend class SimulatedHullsSystem;

		[[nodiscard]]
		auto getStorageRequirements( const float *viewOrigin, const float *viewAxis, float cameraViewTangent ) const
			-> std::optional<std::pair<unsigned, unsigned>> override;

		[[nodiscard]]
		auto fillMeshBuffers( const float *__restrict viewOrigin,
							  const float *__restrict viewAxis,
							  float cameraViewTangent,
							  const Scene::DynamicLight *lights,
							  std::span<const uint16_t> affectingLightIndices,
							  vec4_t *__restrict destPositions,
							  vec4_t *__restrict destNormals,
							  vec2_t *__restrict destTexCoords,
							  byte_vec4_t *__restrict destColors,
							  uint16_t *__restrict destIndices ) const -> std::pair<unsigned, unsigned> override;

	};

	class HullCloudDynamicMesh : public HullDynamicMesh {
		friend class SimulatedHullsSystem;

		vec4_t m_spriteColor { 1.0f, 1.0f, 1.0f, 1.0f };
		// Can go above 1.0f
		float m_alphaScale { 1.0f };
		float m_spriteRadius { 16.0f };
		float m_lifetimeSeconds { 0.0f };
		int m_tessLevelShiftForMinVertexIndex { 0 };
		int m_tessLevelShiftForMaxVertexIndex { 0 };
		int m_shiftFromDefaultLevelToHide { std::numeric_limits<int>::min() };
		// These fields get updated by getStorageRequirements()
		mutable unsigned m_minVertexNumThisFrame { 0 };
		mutable unsigned m_vertexNumLimitThisFrame { 0 };
		uint16_t m_phaseIndexShiftInTable { 0 };
		uint16_t m_speedIndexShiftInTable { 0 };
		bool m_applyRotation { false };

		[[nodiscard]]
		auto getStorageRequirements( const float *viewOrigin, const float *viewAxis, float cameraViewTangent ) const
			-> std::optional<std::pair<unsigned, unsigned>> override;

		[[nodiscard]]
		auto fillMeshBuffers( const float *__restrict viewOrigin,
							  const float *__restrict viewAxis,
							  float cameraViewTangent,
							  const Scene::DynamicLight *lights,
							  std::span<const uint16_t> affectingLightIndices,
							  vec4_t *__restrict destPositions,
							  vec4_t *__restrict destNormals,
							  vec2_t *__restrict destTexCoords,
							  byte_vec4_t *__restrict destColors,
							  uint16_t *__restrict destIndices ) const -> std::pair<unsigned, unsigned> override;
	};

	struct ColorChangeState {
		int64_t lastColorChangeAt { 0 };
		unsigned lastNodeIndex { 0 };
	};

	struct BaseRegularSimulatedHull {
		CMShapeList *shapeList { nullptr };
		int64_t spawnTime { 0 };

		vec4_t mins, maxs;
		vec3_t origin;

		unsigned lifetime { 0 };
		// Archimedes/xy expansion activation offset
		int64_t expansionStartAt { std::numeric_limits<int64_t>::max() };

		// Old/current
		vec4_t *vertexPositions[2];

		vec4_t *vertexNormals;

		// Velocities of an initial burst, decelerate with time
		vec3_t *vertexBurstVelocities;
		// Velocities produced by external forces during simulation
		vec3_t *vertexForceVelocities;

		byte_vec4_t *vertexColors;

		std::span<const ColorChangeTimelineNode> colorChangeTimeline;
		ColorChangeState colorChangeState;

		AppearanceRules appearanceRules = SolidAppearanceRules { .material = nullptr };

		// It's actually cheaper to process these vertices as regular ones
		// and overwrite possible changes after processColorChange() calls.
		// We can't just set the alpha to zero like it used to be,
		// as zero-alpha vertices still may be overwritten with new color replacement rules.
		std::span<const uint16_t> noColorChangeIndices;
		const uint8_t *noColorChangeVertexColor { nullptr };

		// TODO: Make smoke hulls subtypes, move these fields to a subtype

		ValueLifespan archimedesTopAccel;
		ValueLifespan archimedesBottomAccel;
		ValueLifespan xyExpansionTopAccel;
		ValueLifespan xyExpansionBottomAccel;

		float minZLastFrame { std::numeric_limits<float>::max() };
		float maxZLastFrame { std::numeric_limits<float>::lowest() };

		float minFadedOutAlpha { 0.0f };

		bool tesselateClosestLod { false };
		bool leprNextLevelColors { false };
		bool applyVertexDynLight { false };
		ViewDotFade vertexViewDotFade { ViewDotFade::NoFade };
		ZFade vertexZFade { ZFade::NoFade };

		uint8_t positionsFrame { 0 };
		uint8_t subdivLevel { 0 };

		// The renderer assumes external lifetime of the submitted spans. Keep it within the hull.
		// TODO: Allocate on demand?
		SharedMeshData sharedMeshData;
		HullSolidDynamicMesh submittedSolidMesh;
		// Keep it limited to two meshes for now
		HullCloudDynamicMesh submittedCloudMeshes[2];

		void simulate( int64_t currTime, float timeDeltaSeconds, wsw::RandomGenerator *__restrict rng );
	};

	template <unsigned SubdivLevel, bool CalcNormals = false>
	struct RegularSimulatedHull : public BaseRegularSimulatedHull {
		static constexpr auto kNumVertices = kNumVerticesForSubdivLevel[SubdivLevel];

		RegularSimulatedHull<SubdivLevel, CalcNormals> *prev { nullptr }, *next { nullptr };

		vec4_t storageOfPositions[2][kNumVertices];
		vec3_t storageOfBurstVelocities[kNumVertices];
		vec3_t storageOfForceVelocities[kNumVertices];
		byte_vec4_t storageOfColors[kNumVertices];

		// TODO: Allocate storage manually instead of specifying it as a member?
#ifdef _MSC_VER
#pragma warning( push )
#pragma warning( disable: 4200 )
#endif
		vec4_t storageOfNormals[CalcNormals ? kNumVertices : 0];
#ifdef _MSC_VER
#pragma warning( pop )
#endif

		RegularSimulatedHull() {
			this->vertexPositions[0]    = storageOfPositions[0];
			this->vertexPositions[1]    = storageOfPositions[1];
			this->vertexNormals         = CalcNormals ? storageOfNormals : nullptr;
			this->vertexBurstVelocities = storageOfBurstVelocities;
			this->vertexForceVelocities = storageOfForceVelocities;
			this->vertexColors          = storageOfColors;
			this->subdivLevel           = SubdivLevel;
		}
	};

	static constexpr unsigned kMaxHullLayers = 5;

	struct BaseConcentricSimulatedHull {
		// Externally managed, should point to the unit mesh data
		const vec4_t *vertexMoveDirections;
		// Distances to the nearest obstacle (or the maximum growth radius in case of no obstacles)
		float *limitsAtDirections;
		int64_t spawnTime { 0 };

		struct Layer {
			vec4_t mins, maxs;
			vec4_t *vertexPositions;
			// Contains pairs (speed, distance from origin along the direction)
			vec2_t *vertexSpeedsAndDistances;
			byte_vec4_t *vertexColors;
			SharedMeshData *sharedMeshData;
			HullSolidDynamicMesh *submittedSolidMesh;
			HullCloudDynamicMesh *submittedCloudMeshes[1];

			// Subtracted from limitsAtDirections for this layer, must be non-negative.
			// This offset is supposed to prevent hulls from ending at the same distance in the end position.
			float finalOffset { 0 };
			float drawOrderDesignator { 0 };

			std::span<const ColorChangeTimelineNode> colorChangeTimeline;
			ColorChangeState colorChangeState;

			const AppearanceRules *overrideAppearanceRules;
			std::optional<ViewDotFade> overrideHullFade;
			std::optional<float> overrideMinFadedOutAlpha;
		};

		AppearanceRules appearanceRules = SolidAppearanceRules { .material = nullptr };

		Layer *layers { nullptr };

		vec4_t mins, maxs;
		vec3_t origin;

		unsigned numLayers { 0 };
		unsigned lifetime { 0 };
		unsigned compoundMeshKey { 0 };

		float minFadedOutAlpha { 0.0f };

		uint8_t subdivLevel { 0 };
		bool applyVertexDynLight { false };
		ViewDotFade vertexViewDotFade { ViewDotFade::NoFade };

		void simulate( int64_t currTime, float timeDeltaSeconds, wsw::RandomGenerator *__restrict rng );
	};

	template <unsigned SubdivLevel, unsigned NumLayers>
	struct ConcentricSimulatedHull : public BaseConcentricSimulatedHull {
		static constexpr auto kNumVertices = kNumVerticesForSubdivLevel[SubdivLevel];

		ConcentricSimulatedHull<SubdivLevel, NumLayers> *prev { nullptr }, *next { nullptr };

		Layer storageOfLayers[NumLayers];
		float storageOfLimits[kNumVertices];
		vec4_t storageOfPositions[kNumVertices * NumLayers];
		vec2_t storageOfSpeedsAndDistances[kNumVertices * NumLayers];
		byte_vec4_t storageOfColors[kNumVertices * NumLayers];
		SharedMeshData storageOfSharedMeshData[NumLayers];
		// TODO: Allocate dynamically on demand?
		// TODO: Optimize the memory layout
		HullSolidDynamicMesh storageOfSolidMeshes[NumLayers];
		HullCloudDynamicMesh storageOfCloudMeshes[NumLayers];

		ConcentricSimulatedHull() {
			this->numLayers                  = NumLayers;
			this->subdivLevel                = SubdivLevel;
			this->layers                     = &storageOfLayers[0];
			this->limitsAtDirections         = &storageOfLimits[0];
			for( unsigned i = 0; i < NumLayers; ++i ) {
				Layer *const layer              = &layers[i];
				layer->vertexPositions          = &storageOfPositions[i * kNumVertices];
				layer->vertexSpeedsAndDistances = &storageOfSpeedsAndDistances[i * kNumVertices];
				layer->vertexColors             = &storageOfColors[i * kNumVertices];
				layer->sharedMeshData           = &storageOfSharedMeshData[i];
				layer->submittedSolidMesh       = &storageOfSolidMeshes[i];

				// There is a single mesh per layer
				assert( std::size( layer->submittedCloudMeshes ) == 1 );
				layer->submittedCloudMeshes[0] = &storageOfCloudMeshes[i];
			}
		}
	};

	struct BaseKeyframedHull {

		float scale;
		// Externally managed, should point to the unit mesh data
		vec3_t *vertexMoveDirections;

        /// BGN TEMP!
        unsigned numVerts;
        /// END TEMP!

		// Distances to the nearest obstacle (or the maximum growth radius in case of no obstacles)
		float *limitsAtDirections;
		int64_t spawnTime { 0 };

        /// should these arrays be statically allocated at amount kMaxSharedCageCagedMeshes, or dynamically created with new?
		StaticCagedMesh *sharedCageCagedMeshes[kMaxSharedCageCagedMeshes];
        /// the following should all be arrays, as the above "sharedCageCagedMeshes"
        SharedMeshData *sharedMeshData;
        AppearanceRules *appearanceRules;// = SolidAppearanceRules { .material = nullptr };
		HullSolidDynamicMesh *submittedSolidMesh;
        HullCloudDynamicMesh *submittedCloudMesh;
        /// END

		vec3_t cageOffsetMinsDir, cageOffsetMaxsDir;
		vec3_t origin;

		unsigned numSharedCageCagedMeshes { 0 };
		unsigned lifetime { 0 };

		//bool applyVertexDynLight { false }; should be re-implemented, if useful ->
        /// physically lights do not really effect see through surfaces, even less waves or emitting objects (like explosions)
        /// the renderer is currently capable of lighting opaque meshes and this is what should be done


		void simulate( int64_t currTime, float timeDeltaSeconds,
                       PolyEffectsSystem *effectsSystem, Geometry *cageGeometry ); /// SIMULATE IS TMP, FOR DEBUGGING
	};

	struct KeyframedHull : public BaseKeyframedHull {

		KeyframedHull *prev { nullptr }, *next { nullptr };

		SharedMeshData storageOfSharedMeshData[kMaxSharedCageCagedMeshes];
		AppearanceRules storageOfAppearanceRules[kMaxSharedCageCagedMeshes];
		HullSolidDynamicMesh storageOfSolidMeshes[kMaxSharedCageCagedMeshes];
        HullCloudDynamicMesh storageOfCloudMeshes[kMaxSharedCageCagedMeshes];

		explicit KeyframedHull( unsigned numVerts = 1, void *addressOfMem = nullptr ) {
            if( addressOfMem ) {

				sharedMeshData     = &storageOfSharedMeshData[0];
				appearanceRules    = &storageOfAppearanceRules[0];
				submittedSolidMesh = &storageOfSolidMeshes[0];
                submittedCloudMesh = &storageOfCloudMeshes[0];

                unsigned offset = sizeof( KeyframedHull );
                this->vertexMoveDirections = ( vec3_t* )( ( uint8_t* )addressOfMem + offset );
                offset += sizeof( *BaseKeyframedHull::vertexMoveDirections ) * numVerts;
                this->limitsAtDirections = ( float* )( ( uint8_t* )addressOfMem + offset );

                this->sharedMeshData = new SharedMeshData;
            }
		}
	};

	static constexpr unsigned kNumFireHullLayers      = 5;
	static constexpr unsigned kNumToonSmokeHullLayers = 1;

	using FireHull        = ConcentricSimulatedHull<3, kNumFireHullLayers>;
	using FireClusterHull = ConcentricSimulatedHull<1, 2>;
	using BlastHull       = ConcentricSimulatedHull<3, 3>;
	using SmokeHull       = RegularSimulatedHull<3, true>;
	using WaveHull        = RegularSimulatedHull<2>;

    void unlinkAndFreeStaticCageHull( KeyframedHull *hull );

	void unlinkAndFreeFireHull( FireHull *hull );
	void unlinkAndFreeFireClusterHull( FireClusterHull *hull );
	void unlinkAndFreeBlastHull( BlastHull *hull );
	void unlinkAndFreeSmokeHull( SmokeHull *hull );
	void unlinkAndFreeWaveHull( WaveHull *hull );

	template <typename Hull, bool HasShapeLists>
	[[nodiscard]]
	auto allocHull( Hull **head, wsw::FreelistAllocator *allocator, int64_t currTime, unsigned lifetime ) -> Hull *;

	// TODO: Having these specialized methods while the actual setup is performed by the caller feels wrong...

    [[nodiscard]]
    auto allocStaticCageHull( KeyframedHull **head, wsw::FreelistAllocator *allocator, int64_t currTime, unsigned lifetime,
                                                    unsigned numVertices ) -> KeyframedHull *;
	[[nodiscard]]
	auto allocFireHull( int64_t currTime, unsigned lifetime ) -> FireHull *;
	[[nodiscard]]
	auto allocFireClusterHull( int64_t currTime, unsigned lifetime ) -> FireClusterHull *;
	[[nodiscard]]
	auto allocBlastHull( int64_t currTime, unsigned lifetime ) -> BlastHull *;
	[[nodiscard]]
	auto allocSmokeHull( int64_t currTime, unsigned lifetime ) -> SmokeHull *;
	[[nodiscard]]
	auto allocWaveHull( int64_t currTime, unsigned lifetime ) -> WaveHull *;

	void setupHullVertices( BaseRegularSimulatedHull *hull, const float *origin, const float *color,
						   float speed, float speedSpread,
						   const AppearanceRules &appearanceRules = SolidAppearanceRules { nullptr },
						   const float *spikeImpactMask = nullptr, float maxSpikeSpeed = 0.0f );

	void setupHullVertices( BaseConcentricSimulatedHull *hull, const float *origin,
							float scale, std::span<const HullLayerParams> paramsOfLayers,
							const AppearanceRules &appearanceRules = SolidAppearanceRules { nullptr } );

	void setupHullVertices( BaseKeyframedHull *hull, const float *origin,
							float scale, const float *dir, const float rotation,
                            SimulatedHullsSystem::StaticCagedMesh *meshToRender, PolyEffectsSystem *effectSystem,
                            const AppearanceRules &appearanceRules = SolidAppearanceRules { nullptr } );

    void addHull( AppearanceRules *appearanceRules, StaticKeyframedHullParams &hullParams );

	void calcSmokeBulgeSpeedMask( float *__restrict vertexSpeedMask, unsigned subdivLevel, unsigned maxSpikes );
	void calcSmokeSpikeSpeedMask( float *__restrict vertexSpeedMask, unsigned subdivLevel, unsigned maxSpikes );

	[[maybe_unused]]
	static bool processColorChange( int64_t currTime, int64_t spawnTime, unsigned effectDuration,
									std::span<const ColorChangeTimelineNode> timeline,
									std::span<byte_vec4_t> colors,
									ColorChangeState *__restrict state,
									wsw::RandomGenerator *__restrict rng );

	[[nodiscard]]
	static auto computeCurrTimelineNodeIndex( unsigned startFromIndex, int64_t currTime,
											  int64_t spawnTime, unsigned effectDuration,
											  std::span<const ColorChangeTimelineNode> timeline ) -> unsigned;

	[[nodiscard]]
	static auto computePrevKeyframeIndex( unsigned startFromIndex, int64_t currTime,
										  int64_t spawnTime, unsigned effectDuration,
										  std::span<const OffsetKeyframe> offsetKeyframeSet ) -> unsigned;

	static constexpr unsigned kMaxFireHulls        = 32;
	static constexpr unsigned kMaxFireClusterHulls = kMaxFireHulls * 2;
	static constexpr unsigned kMaxBlastHulls       = 32;
	static constexpr unsigned kMaxSmokeHulls       = kMaxFireHulls * 2;
	static constexpr unsigned kMaxToonSmokeHulls   = kMaxFireHulls;
	static constexpr unsigned kMaxWaveHulls        = kMaxFireHulls;

	static constexpr unsigned kMaxConcentricHulls = kMaxFireHulls + kMaxFireClusterHulls + kMaxBlastHulls;
	static constexpr unsigned kMaxKeyframedHulls  = kMaxToonSmokeHulls;
	static constexpr unsigned kMaxRegularHulls    = kMaxWaveHulls;

	static_assert( kMaxConcentricHulls <= std::numeric_limits<uint8_t>::max() );
	static_assert( kMaxKeyframedHulls <= std::numeric_limits<uint8_t>::max() );

	[[maybe_unused]]
	auto buildMatchingHullPairs( const BaseKeyframedHull **keyframedHulls, unsigned numKeyframedHulls,
								 const BaseConcentricSimulatedHull **concentricHulls, unsigned numConcentricHulls,
								 wsw::StaticVector<std::optional<uint8_t>, kMaxKeyframedHulls> *pairIndicesForKeyframedHulls,
								 wsw::StaticVector<std::optional<uint8_t>, kMaxConcentricHulls> *pairIndicesForConcentricHulls ) -> unsigned;

	FireHull *m_fireHullsHead { nullptr };
	FireClusterHull *m_fireClusterHullsHead { nullptr };
	BlastHull *m_blastHullsHead { nullptr };
	SmokeHull *m_smokeHullsHead { nullptr };
	WaveHull *m_waveHullsHead { nullptr };

	wsw::StaticVector<CMShapeList *, kMaxSmokeHulls + kMaxWaveHulls> m_freeShapeLists;
	CMShapeList *m_tmpShapeList { nullptr };

	wsw::HeapBasedFreelistAllocator m_fireHullsAllocator { sizeof( FireHull ), kMaxFireHulls };
	wsw::HeapBasedFreelistAllocator m_fireClusterHullsAllocator { sizeof( FireClusterHull ), kMaxFireHulls };
	wsw::HeapBasedFreelistAllocator m_blastHullsAllocator { sizeof( BlastHull ), kMaxBlastHulls };
	wsw::HeapBasedFreelistAllocator m_smokeHullsAllocator { sizeof( SmokeHull ), kMaxSmokeHulls };
	wsw::HeapBasedFreelistAllocator m_waveHullsAllocator { sizeof( WaveHull ), kMaxWaveHulls };

	// Can't specify byte_vec4_t as the template parameter
	wsw::Vector<uint32_t> m_frameSharedOverrideColorsBuffer;

	// We may submit solid and cloud meshes for each layer, so it's twice as much as the number of layers
	static constexpr unsigned kMaxMeshesPerHullGeneral = ( 2 * kMaxHullLayers );

	static_assert( kMaxFireHulls == kMaxToonSmokeHulls );
	// We may submit solid and cloud meshes for each layer, so it's twice as much as the sum of numbers of layers
	static constexpr unsigned kMaxMeshesPerHullForCombinedExplosion = 2 * ( kNumToonSmokeHullLayers + kNumFireHullLayers );

	// Choose the biggest value for bins, this simplifies the code even if a bit overuse memory
	static constexpr unsigned kMaxMeshesPerHull   = wsw::max( kMaxMeshesPerHullGeneral, kMaxMeshesPerHullForCombinedExplosion );
	static constexpr unsigned kMaxHullsWithLayers = kMaxFireHulls + kMaxFireClusterHulls + kMaxBlastHulls + kMaxToonSmokeHulls;

	PodBufferHolder<const DynamicMesh *> m_storageOfSubmittedMeshPtrs;
	PodBufferHolder<float> m_storageOfSubmittedMeshOrderDesignators;

	wsw::RandomGenerator m_rng;
	int64_t m_lastTime { 0 };
};

#endif