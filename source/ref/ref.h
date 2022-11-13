/*
Copyright (C) 1997-2001 Id Software, Inc.

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.

*/

#ifndef WSW_ac367414_9635_4452_b2fc_9477f5254db5_H
#define WSW_ac367414_9635_4452_b2fc_9477f5254db5_H

#include "../qcommon/wswstaticvector.h"
#include "../gameshared/q_math.h"

#include <optional>
#include <span>
#include <variant>

// FIXME: move these to r_local.h?
#define MAX_DLIGHTS             32
#define MAX_ENTITIES            2048
#define MAX_POLY_VERTS          3000
#define MAX_QUAD_POLYS          256
#define MAX_COMPLEX_POLYS       64

// entity_state_t->renderfx flags
#define RF_MINLIGHT             0x1       // always have some light (viewmodel)
#define RF_FULLBRIGHT           0x2       // always draw full intensity
#define RF_FRAMELERP            0x4
#define RF_NOSHADOW             0x8

#define RF_VIEWERMODEL          0x10     // don't draw through eyes, only mirrors
#define RF_WEAPONMODEL          0x20     // only draw through eyes and depth hack
#define RF_CULLHACK             0x40
#define RF_FORCENOLOD           0x80
#define RF_NOPORTALENTS         0x100
#define RF_ALPHAHACK            0x200   // force alpha blending on opaque passes, read alpha from entity
#define RF_GREYSCALE            0x400
#define RF_NODEPTHTEST          0x800
#define RF_NOCOLORWRITE         0x1000

// refdef flags
#define RDF_UNDERWATER          0x1     // warp the screen as apropriate
#define RDF_NOWORLDMODEL        0x2     // used for player configuration screen
#define RDF_OLDAREABITS         0x4     // forces R_MarkLeaves if not set
#define RDF_PORTALINVIEW        0x8     // cull entities using vis too because pvs\areabits are merged serverside
#define RDF_SKYPORTALINVIEW     0x10    // draw skyportal instead of regular sky
#define RDF_FLIPPED             0x20
#define RDF_WORLDOUTLINES       0x40    // draw cell outlines for world surfaces
#define RDF_CROSSINGWATER       0x80    // potentially crossing water surface
#define RDF_USEORTHO            0x100   // use orthographic projection
#define RDF_BLURRED             0x200

// skm flags
#define SKM_ATTACHMENT_BONE     1

typedef struct orientation_s {
	mat3_t axis;
	vec3_t origin;
} orientation_t;

typedef struct bonepose_s {
	dualquat_t dualquat;
} bonepose_t;

typedef struct fragment_s {
	int firstvert;
	int numverts;                       // can't exceed MAX_POLY_VERTS
	int fognum;                         // -1 - no fog
	                                    //  0 - determine fog in R_AddPolyToScene
	                                    // >0 - valid fog volume number returned by R_GetClippedFragments
	vec3_t normal;
} fragment_t;

typedef struct {
	float rgb[3];                       // 0.0 - 2.0
} lightstyle_t;

typedef struct {
	float fov;
	float scale;
	vec3_t vieworg;
	vec3_t viewanglesOffset;
	bool noEnts;
} skyportal_t;

typedef enum {
	RT_MODEL,
	RT_SPRITE,
	RT_PORTALSURFACE,

	NUM_RTYPES
} refEntityType_t;

typedef struct entity_s {
	refEntityType_t rtype;
	unsigned number;

	union {
		int flags;
		int renderfx;
	};

	struct model_s *model;              // opaque type outside refresh

	/*
	** most recent data
	*/
	mat3_t axis;
	vec3_t origin, origin2;
	vec3_t lightingOrigin;
	int frame;
	bonepose_t *boneposes;              // pretransformed boneposes for current frame

	/*
	** previous data for lerping
	*/
	int oldframe;
	bonepose_t *oldboneposes;           // pretransformed boneposes for old frame
	float backlerp;                     // 0.0 = current, 1.0 = old

	/*
	** texturing
	*/
	struct Skin *customSkin;      // registered .skin file
	struct shader_s *customShader;      // NULL for inline skin

	/*
	** misc
	*/
	int64_t shaderTime;
	union {
		byte_vec4_t color;
		uint8_t shaderRGBA[4];
	};

	float scale;
	float radius;                       // used as RT_SPRITE's radius
	float rotation;

	float outlineHeight;
	union {
		byte_vec4_t outlineColor;
		uint8_t outlineRGBA[4];
	};
} entity_t;

typedef struct refdef_s {
	int x, y, width, height;            // viewport, in virtual screen coordinates
	int scissor_x, scissor_y, scissor_width, scissor_height;
	int ortho_x, ortho_y;
	float fov_x, fov_y;
	vec3_t vieworg;
	mat3_t viewaxis;
	float blend[4];                     // rgba 0-1 full screen blend
	int64_t time;                       // time is used for timing offsets
	int rdflags;                        // RDF_UNDERWATER, etc
	skyportal_t skyportal;
	uint8_t *areabits;                  // if not NULL, only areas with set bits will be drawn
	float weaponAlpha;
	float minLight;                     // minimum value of ambient lighting applied to RF_MINLIGHT entities
	struct shader_s *colorCorrection;   // post processing color correction lookup table to apply
} refdef_t;

struct ColorLifespan {
	float initialColor[4] { 1.0f, 1.0f, 1.0f, 0.0f };
	float fadedInColor[4] { 1.0f, 1.0f, 1.0f, 1.0f };
	float fadedOutColor[4] { 1.0f, 1.0f, 1.0f, 0.0f };
	float finishFadingInAtLifetimeFrac { 0.25f };
	float startFadingOutAtLifetimeFrac { 0.75f };

	void getColorForLifetimeFrac( float lifetimeFrac, float *colorRgba ) const __restrict {
		assert( lifetimeFrac >= 0.0f && lifetimeFrac <= 1.0f );

		assert( finishFadingInAtLifetimeFrac > 0.01f );
		assert( startFadingOutAtLifetimeFrac < 0.99f );
		assert( finishFadingInAtLifetimeFrac + 0.01f < startFadingOutAtLifetimeFrac );

		if( lifetimeFrac < finishFadingInAtLifetimeFrac ) [[unlikely]] {
			// Fade in
			float fadeInFrac = lifetimeFrac * Q_Rcp( finishFadingInAtLifetimeFrac );
			assert( fadeInFrac > -0.01f && fadeInFrac < 1.01f );
			fadeInFrac = wsw::clamp( fadeInFrac, 0.0f, 1.0f );
			Vector4Lerp( initialColor, fadeInFrac, fadedInColor, colorRgba );
		} else {
			if( lifetimeFrac > startFadingOutAtLifetimeFrac ) [[unlikely]] {
				// Fade out
				float fadeOutFrac = lifetimeFrac - startFadingOutAtLifetimeFrac;
				fadeOutFrac *= Q_Rcp( 1.0f - startFadingOutAtLifetimeFrac );
				assert( fadeOutFrac > -0.01f && fadeOutFrac < 1.01f );
				fadeOutFrac = wsw::clamp( fadeOutFrac, 0.0f, 1.0f );
				Vector4Lerp( fadedInColor, fadeOutFrac, fadedOutColor, colorRgba );
			} else {
				// Use the color of the "faded-in" state
				Vector4Copy( fadedInColor, colorRgba );
			}
		}
	}
};

struct LightLifespan {
	float initialColor[3] { 1.0f, 1.0f, 1.0f };
	float fadedInColor[3] { 1.0f, 1.0f, 1.0f };
	float fadedOutColor[3] { 1.0f, 1.0f, 1.0f };

	float finishColorFadingInAtLifetimeFrac { 0.25f };
	float startColorFadingOutAtLifetimeFrac { 0.75f };

	float initialRadius { 0.0f };
	float fadedInRadius { 128.0f };
	float fadedOutRadius { 0.0f };

	float finishRadiusFadingInAtLifetimeFrac { 0.33f };
	float startRadiusFadingOutAtLifetimeFrac { 0.67f };

	void getRadiusAndColorForLifetimeFrac( float lifetimeFrac, float *radius, float *colorRgb ) const __restrict {
		assert( finishColorFadingInAtLifetimeFrac > 0.01f );
		assert( startColorFadingOutAtLifetimeFrac < 0.99f );
		assert( finishColorFadingInAtLifetimeFrac + 0.01f < startColorFadingOutAtLifetimeFrac );

		if( lifetimeFrac < finishColorFadingInAtLifetimeFrac ) [[unlikely]] {
			// Fade in
			float fadeInFrac = lifetimeFrac * Q_Rcp( finishColorFadingInAtLifetimeFrac );
			assert( fadeInFrac > -0.01f && fadeInFrac < 1.01f );
			fadeInFrac = wsw::clamp( fadeInFrac, 0.0f, 1.0f );
			VectorLerp( initialColor, fadeInFrac, fadedInColor, colorRgb );
		} else {
			if( lifetimeFrac > startColorFadingOutAtLifetimeFrac ) [[unlikely]] {
				// Fade out
				float fadeOutFrac = lifetimeFrac - startColorFadingOutAtLifetimeFrac;
				fadeOutFrac *= Q_Rcp( 1.0f - startColorFadingOutAtLifetimeFrac );
				assert( fadeOutFrac > -0.01f && fadeOutFrac < 1.01f );
				fadeOutFrac = wsw::clamp( fadeOutFrac, 0.0f, 1.0f );
				VectorLerp( fadedInColor, fadeOutFrac, fadedOutColor, colorRgb );
			} else {
				// Use the color of the "faded-in" state
				VectorCopy( fadedInColor, colorRgb );
			}
		}

		assert( finishRadiusFadingInAtLifetimeFrac > 0.01f );
		assert( startRadiusFadingOutAtLifetimeFrac < 0.99f );
		assert( finishRadiusFadingInAtLifetimeFrac + 0.01f < startRadiusFadingOutAtLifetimeFrac );

		if( lifetimeFrac < finishRadiusFadingInAtLifetimeFrac ) [[unlikely]] {
			float fadeInFrac = lifetimeFrac * Q_Rcp( finishRadiusFadingInAtLifetimeFrac );
			assert( fadeInFrac > -0.01f && fadeInFrac < 1.01f );
			fadeInFrac = wsw::clamp( fadeInFrac, 0.0f, 1.0f );
			*radius = fadeInFrac * fadedInRadius + ( 1.0f - fadeInFrac ) * initialRadius;
		} else {
			if( lifetimeFrac > startRadiusFadingOutAtLifetimeFrac ) [[unlikely]] {
				float fadeOutFrac = lifetimeFrac - startRadiusFadingOutAtLifetimeFrac;
				fadeOutFrac *= Q_Rcp( 1.0f - startRadiusFadingOutAtLifetimeFrac );
				assert( fadeOutFrac > -0.01f && fadeOutFrac < 1.01f );
				fadeOutFrac = wsw::clamp( fadeOutFrac, 0.0f, 1.0f );
				*radius = fadeOutFrac * fadedOutRadius + ( 1.0f - fadeOutFrac ) * fadedInRadius;
			} else {
				*radius = fadedInRadius;
			}
		}

		assert( finishColorFadingInAtLifetimeFrac > 0.01f );
		assert( startColorFadingOutAtLifetimeFrac < 0.99f );
		assert( finishColorFadingInAtLifetimeFrac + 0.01f < startColorFadingOutAtLifetimeFrac );

		if( lifetimeFrac < finishColorFadingInAtLifetimeFrac ) [[unlikely]] {
			// Fade in
			float fadeInFrac = lifetimeFrac * Q_Rcp( finishColorFadingInAtLifetimeFrac );
			assert( fadeInFrac > -0.01f && fadeInFrac < 1.01f );
			fadeInFrac = wsw::clamp( fadeInFrac, 0.0f, 1.0f );
			VectorLerp( initialColor, fadeInFrac, fadedInColor, colorRgb );
		} else {
			if( lifetimeFrac > startColorFadingOutAtLifetimeFrac ) [[unlikely]] {
				// Fade out
				float fadeOutFrac = lifetimeFrac - startColorFadingOutAtLifetimeFrac;
				fadeOutFrac *= Q_Rcp( 1.0f - startColorFadingOutAtLifetimeFrac );
				assert( fadeOutFrac > -0.01f && fadeOutFrac < 1.01f );
				fadeOutFrac = wsw::clamp( fadeOutFrac, 0.0f, 1.0f );
				VectorLerp( fadedInColor, fadeOutFrac, fadedOutColor, colorRgb );
			} else {
				// Use the color of the "faded-in" state
				VectorCopy( fadedInColor, colorRgb );
			}
		}
	}
};

struct alignas( 16 ) Particle {
	enum SizeBehaviour : uint8_t {
		SizeNotChanging,
		Expanding,
		Shrinking,
		ExpandingAndShrinking,
	};

	// Few notes on these value ranges:
	// They are ad-hoc structs for now, as we don't need more complicated stuff at this stage.
	// Distributions are assumed to be uniform in range (mean - spread, mean + spread) for historical reasons.
	// Negative values are clamped to zero.

	struct SpriteRules {
		// TODO: Why do we have to specify the {} initializer explicitly?
		struct { float mean { 1.0f }, spread { 0.0f }; } radius {};
		SizeBehaviour sizeBehaviour { SizeNotChanging };
	};

	struct SparkRules {
		struct { float mean { 1.0f }, spread { 0.0f }; } length {};
		struct { float mean { 1.0f }, spread { 0.0f }; } width {};
		float viewDirPartScale { 0.67f };
		SizeBehaviour sizeBehaviour { SizeNotChanging };
	};

	struct FlareProps {
		std::span<const LightLifespan> lightProps;
		float alphaScale { 1.0f };
		float radiusScale { 1.0f };
		// Flares are very cheap to process, even if the current implementation is not optimal.
		// These options are primarily for purposes of fine-tuning appearance.
		uint16_t flockFrameAffinityIndex { 0 };
		uint16_t flockFrameAffinityModulo { 0 };
		// For skipping individual particles
		uint16_t particleFrameAffinityModulo { 0 };
	};

	// Common for flocks/aggregates.
	// The name "rules" seems to be more appropriate than "params" for these stateless/shared objects.
	struct AppearanceRules {
		// Points to an external buffer with a greater lifetime.
		shader_s **materials;

		// Points to external buffers with a greater lifetime
		std::span<const ColorLifespan> colors;

		// Unfortunately, std::span can't be used for materials due to value type restrictions.
		// TODO: Use our custom span type
		uint8_t numMaterials { 1 };

		// Program light properties.
		// Note that only a single particle (which is cycled) in a flock may be a light emitter during scene submission.
		// This span points to external buffers with a greater lifetime.
		// Empty if no light.
		// 1 element if the light props are the same for the entire flock.
		// Matches length of colors if it should be addressed by Particle::instanceColorIndex.
		// Other length options are invalid.
		std::span<const LightLifespan> lightProps;

		// These flares are similar to corona lights but are submitted as particle aggregates and not scene lights
		// (even corona lights contribute something else to the scene, e.g. they could affect vertex-lit surfaces).
		// Span values have the same meaning as lightProps ones.
		// In the current codebase state, we should not try drawing flares
		// during submission of spark particles due to material mismatch which leads to switching textures
		// (this could be solved by using a texture atlas, which is not available yet (?)).

		std::optional<FlareProps> flareProps;

		std::variant<SpriteRules, SparkRules> geometryRules;

		uint16_t lightFrameAffinityIndex { 0 };
		uint16_t lightFrameAffinityModulo { 0 };

		bool applyVertexDynLight { false };

	};

	float origin[4];
	float oldOrigin[4];
	float velocity[4];
	float accel[4];
	float rotationAngle;
	float angularVelocity;

	int64_t spawnTime;
	// Gets updated every simulation frame prior to submission for rendering
	float lifetimeFrac;

	uint16_t lifetime;
	uint16_t activationDelay;

	uint8_t bounceCount;

	// TODO: All of this ties Particle to fixed AppearanceRules, allow supplying different instance data

	static constexpr float kByteParamNormalizer = 1.0f / 128.0f;

	// Should be set once upon spawn. Fractions are stored in a compact representation,
	// floating-point values should be reconstructed by multiplying by kByteParamNormalizer
	// The real parameter value is a multiple of this fraction by AppearanceRules:: parameter spread
	int8_t instanceLengthSpreadFraction;
	int8_t instanceWidthSpreadFraction;
	int8_t instanceRadiusSpreadFraction;

	uint8_t instanceLengthExtraScale;
	uint8_t instanceWidthExtraScale;
	uint8_t instanceRadiusExtraScale;

	// Keeps an index of an instance material in the AppearanceRules span
	uint8_t instanceMaterialIndex;
	// Keeps an index of instance color parameters in AppearanceRules color-related spans
	uint8_t instanceColorIndex;

	uint8_t rotationAxisIndex;
};

struct VisualTrace {
	shader_s *shader;
	const char *name;
	float endpos[3];
	cplane_t plane;
	int surfFlags;
	float fraction;
};

namespace wsw::ref {

void traceAgainstBspWorld( VisualTrace *tr, const float *start, const float *end, int skipSurfMask = 0 );
void traceAgainstBrushModel( VisualTrace *tr, const model_s *model, const float *origin,
							 const float *axis, const float *start, const float *end, int skipSurfMask = 0 );

}

namespace wsw::ref { class Frontend; }

struct QuadPoly;
struct DynamicMesh;

class Scene {
	friend class wsw::ref::Frontend;
public:
	struct DynamicLight {
		float origin[3];
		float programRadius;
		float coronaRadius;
		float maxRadius;
		float color[3];
		float mins[4];
		float maxs[4];
		bool hasProgramLight;
		bool hasCoronaLight;
	};

	// A flock of particles or just a bunch of particles with enclosing bounds
	struct ParticlesAggregate {
		const Particle *particles;
		Particle::AppearanceRules appearanceRules;
		float mins[4], maxs[4];
		unsigned numParticles { 0 };
	};

	struct CompoundDynamicMesh {
		float cullMins[4], cullMaxs[4];
		const DynamicMesh **parts;
		unsigned numParts;
		std::optional<uint8_t> drawOnTopPartIndex;
	};
protected:
	Scene();

	wsw::StaticVector<DynamicLight, 1024> m_dynamicLights;

	entity_t *m_worldent;
	entity_t *m_polyent;

	wsw::StaticVector<entity_t *, MAX_ENTITIES + 48> m_entities;
	wsw::StaticVector<entity_t, 2> m_localEntities;
	// TODO: Use different subtypes so the storage is more compact
	wsw::StaticVector<entity_t, MAX_ENTITIES> m_nullModelEntities;
	wsw::StaticVector<entity_t, MAX_ENTITIES> m_aliasModelEntities;
	wsw::StaticVector<entity_t, MAX_ENTITIES> m_skeletalModelEntities;
	wsw::StaticVector<entity_t, MAX_ENTITIES> m_brushModelEntities;
	wsw::StaticVector<entity_t, MAX_ENTITIES> m_spriteEntities;

	// These polys are externally owned with a lifetime greater than the frame
	wsw::StaticVector<QuadPoly *, MAX_QUAD_POLYS> m_quadPolys;

	static constexpr unsigned kMaxParticlesInAggregate = 256;
	static constexpr unsigned kMaxParticleAggregates = 1024;

	static constexpr unsigned kMaxPartsInCompoundMesh = 8;
	static constexpr unsigned kMaxCompoundDynamicMeshes = 64;
	static constexpr unsigned kMaxDynamicMeshes = 128;

	wsw::StaticVector<ParticlesAggregate, kMaxParticleAggregates> m_particles;
	wsw::StaticVector<const DynamicMesh *, kMaxDynamicMeshes> m_dynamicMeshes;
	wsw::StaticVector<CompoundDynamicMesh, kMaxCompoundDynamicMeshes> m_compoundDynamicMeshes;
};

// TODO: Aggregate Scene as a member?
class DrawSceneRequest : public Scene {
	friend class wsw::ref::Frontend;

	// TODO: Get rid of "refdef_t"
	refdef_t m_refdef;
public:
	void addLight( const float *origin, float programRadius, float coronaRadius, float r, float g, float b );
	void addLight( const float *origin, float programRadius, float coronaRadius, const float *color ) {
		addLight( origin, programRadius, coronaRadius, color[0], color[1], color[2] );
	}

	// TODO: Allow adding multiple particle aggregates at once
	void addParticles( const float *mins, const float *maxs,
					   const Particle::AppearanceRules &appearanceRules,
					   const Particle *particles, unsigned numParticles );

	void addCompoundDynamicMesh( const float *mins, const float *maxs,
								 const DynamicMesh **parts, unsigned numParts,
								 std::optional<uint8_t> drawOnTopPartIndex = std::nullopt );

	void addDynamicMesh( const DynamicMesh *mesh );

	void addEntity( const entity_t *ent );

	// No copying is being performed
	void addPoly( QuadPoly *poly ) {
		if( !m_quadPolys.full() ) [[likely]] {
			m_quadPolys.push_back( poly );
		}
	}

	explicit DrawSceneRequest( const refdef_t &refdef ) : m_refdef( refdef ) {}
};

struct QuadPoly {
	struct shader_s *material;
	float color[4];
	float from[3];
	float to[3];
	float dir[3];
	float width;
	float length;
	float tileLength;
};

struct DynamicMesh {
	struct shader_s *material;
	float cullMins[4], cullMaxs[4];

	bool applyVertexDynLight { false };

	virtual ~DynamicMesh() = default;

	[[nodiscard]]
	virtual auto getStorageRequirements( const float *__restrict viewOrigin,
										 const float *__restrict viewAxis,
										 float cameraViewTangent ) const
		-> std::optional<std::pair<unsigned, unsigned>> = 0;

	[[nodiscard]]
	virtual auto fillMeshBuffers( const float *__restrict viewOrigin,
								  const float *__restrict viewAxis,
								  float cameraViewTangent,
								  const Scene::DynamicLight *dynamicLights,
								  std::span<const uint16_t> affectingLightIndices,
								  vec4_t *__restrict destPositions,
								  vec4_t *__restrict destNormals,
								  vec2_t *__restrict destTexCoords,
								  byte_vec4_t *__restrict destColors,
								  uint16_t *__restrict destIndices ) const -> std::pair<unsigned, unsigned> = 0;
};

DrawSceneRequest *CreateDrawSceneRequest( const refdef_t &refdef );
void SubmitDrawSceneRequest( DrawSceneRequest *request );

class Texture;

void        R_DrawStretchPic( int x, int y, int w, int h, float s1, float t1, float s2, float t2,
							  const vec4_t color, const shader_s *shader );
void        R_DrawRotatedStretchPic( int x, int y, int w, int h, float s1, float t1, float s2, float t2,
									 float angle, const vec4_t color, const shader_s *shader );
void        R_DrawStretchQuick( int x, int y, int w, int h, float s1, float t1, float s2, float t2,
								const vec4_t color, int program_type, Texture *image, int blendMask );

void        R_DrawExternalTextureOverlay( unsigned externalTexNum );

struct model_s;

void        R_ModelBounds( const model_s *model, vec3_t mins, vec3_t maxs );
void        R_ModelFrameBounds( const model_s *model, int frame, vec3_t mins, vec3_t maxs );
void        R_RegisterWorldModel( const char *model );
model_s *R_RegisterModel( const char *name );

int         R_SkeletalGetBoneInfo( const model_s *mod, int bonenum, char *name, size_t name_size, int *flags );
void        R_SkeletalGetBonePose( const model_s *mod, int bonenum, int frame, bonepose_t *bonepose );
int         R_SkeletalGetNumBones( const model_s *mod, int *numFrames );

shader_s    *R_RegisterShader( const char *name, int type );
shader_s    *R_RegisterPic( const char *name );
shader_s    *R_RegisterRawAlphaMask( const char *name, int width, int height, const uint8_t *data );
shader_s    *R_RegisterSkin( const char *name );
shader_s    *R_RegisterLinearPic( const char *name );

struct ImageOptions {
	std::optional<std::pair<unsigned, unsigned>> desiredSize;
	unsigned borderWidth { 0 };
	bool fitSizeForCrispness { false };
	bool useOutlineEffect { false };

	template <typename T>
	void setDesiredSize( T width, T height ) {
		assert( width > 0 && height > 0 && width < (T)( 1 << 16 ) && height < (T)( 1 << 16 ) );
		desiredSize = std::make_pair( (unsigned)width, (unsigned)height );
	}
};

shader_s *R_CreateExplicitlyManaged2DMaterial();
void R_ReleaseExplicitlyManaged2DMaterial( shader_s *material );
bool R_UpdateExplicitlyManaged2DMaterialImage( shader_s *material, const char *name, const ImageOptions &options );

[[nodiscard]]
auto R_GetShaderDimensions( const shader_s *shader ) -> std::optional<std::pair<unsigned, unsigned>>;

void        R_ReplaceRawSubPic( shader_s *shader, int x, int y, int width, int height, const uint8_t *data );

struct Skin;
Skin *R_RegisterSkinFile( const char *name );
shader_s *R_FindShaderForSkinFile( const Skin *skin, const char *meshname );

void RF_TransformVectorToScreen( const refdef_t *rd, const vec3_t in, vec2_t out );
bool RF_LerpTag( orientation_t *orient, const model_s *mod, int oldframe, int frame, float lerpfrac, const char *name );

void        R_SetCustomColor( int num, int r, int g, int b );

struct VidModeOptions {
	bool fullscreen { false };
	bool borderless { false };
};

rserr_t R_TrySettingMode( int x, int y, int width, int height, int displayFrequency, const VidModeOptions &options );

void RF_BeginRegistration();
void RF_EndRegistration();

void RF_AppActivate( bool active, bool minimize, bool destroy );
void RF_Shutdown( bool verbose );

void RF_BeginFrame( bool forceClear, bool forceVsync, bool uncappedFPS );
void RF_EndFrame();

void R_Set2DMode( bool );

const char *RF_GetSpeedsMessage( char *out, size_t size );

int RF_GetAverageFrametime();

void R_Finish();

rserr_t     R_Init( const char *applicationName, const char *screenshotPrefix, int startupColor,
					int iconResource, const int *iconXPM,
					void *hinstance, void *wndproc, void *parenthWnd,
					bool verbose );

#endif // __REF_H
