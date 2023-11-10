#ifndef WSW_d66e64a4_3cf7_44a9_a3f3_cc95b2beaa6a_H
#define WSW_d66e64a4_3cf7_44a9_a3f3_cc95b2beaa6a_H

struct CMShapeList;

template <typename> class SingletonHolder;

#include "../qcommon/freelistallocator.h"
#include "../qcommon/randomgenerator.h"
#include "../ref/ref.h"

#include <span>

#ifdef min
#undef min
#endif

#ifdef max
#undef max
#endif

// Few notes on these value ranges:
// They are ad-hoc structs for now, as we don't need more complicated stuff at this stage.
// Distributions are obviously assumed to be uniform.
// It should not matter whether a bound is inclusive or exclusive due to a coarse nature of the corresponding value,
// unless it's explicitly indicated via a field name.

// Mutability of fields makes adjusting parameters in a loop more convenient
struct EllipsoidalFlockParams {
	float origin[3] { 0.0f, 0.0f, 0.0f };
	float offset[3] { 0.0f, 0.0f, 0.0f };
	float shiftDir[3] { 0.0f, 0.0f, 1.0f };
	// So far, only stretching/flattening the sphere along the single dir seems to be useful
	float stretchDir[3] { 0.0f, 0.0f, 1.0f };
	float stretchScale { 1.0f };
	float gravity { 600 };
	float drag { 0.0f };
	float vorticity { 0.0f }; // induced velocity at up to 16 units radius from particle origin
	float vorticityAxis[3] {0.0f, 0.0f, 1.0f }; // the axis around which particles rotate
    float outflow {0.0f};
    float outflowAxis[3] {0.0f, 0.0f, 1.0f}; // axis of the outflow, should be a unit vector
    float turbulence { 0.0f };
    float turbulenceScale {1.0f};
	float restitution { 0.75f };
	struct { unsigned minInclusive { 1 }, maxInclusive { 1 }; } bounceCount;
	struct { float min { 300 }, max { 300 }; } speed;
	struct { float min { 0.0f }, max { 0.0f }; } shiftSpeed;
	struct { float min { 0.0f }, max { 0.0f }; } angularVelocity;
	struct { float min { 0.0f }, max { 1.0f }; } percentage;
	struct { unsigned min { 300u }, max { 700u }; } timeout;
	struct { unsigned min { 0 }, max { 0 }; } activationDelay;
	unsigned startBounceCounterDelay { 0 };
};

// Mutability of fields makes adjusting parameters in a loop more convenient
struct ConicalFlockParams {
	float origin[3] { 0.0f, 0.0f, 0.0f };
	float offset[3] { 0.0f, 0.0f, 0.0f };
	float dir[3] { 0.0f, 0.0f, 1.0f };
	float shiftDir[3] { 0.0f, 0.0f, 1.0f };
	float gravity { 600 };
	float drag { 0.0f };
	float vorticity { 0.0f }; // induced velocity at up to 16 units radius from particle origin
	float vorticityAxis[3] {0.0f, 0.0f, 1.0f }; // the axis around which particles rotate
    float outflow {0.0f};
    float outflowAxis[3] {0.0f, 0.0f, 1.0f}; // axis of the outflow, should be a unit vector
    float turbulence { 0.0f };
    float turbulenceScale {1.0f};
	float restitution { 0.75f };
	float angle { 45.0f };
	float innerAngle { 0.0f };
	struct { unsigned minInclusive { 1 }, maxInclusive { 1 }; } bounceCount;
	struct { float min { 300 }, max { 300 }; } speed;
	struct { float min { 0.0f }, max { 0.0f }; } shiftSpeed;
	struct { float min { 0.0f }, max { 0.0f }; } angularVelocity;
	struct { float min { 0.0f }, max { 1.0f }; } percentage;
	struct { unsigned min { 300u }, max { 700u }; } timeout;
	struct { unsigned min { 0 }, max { 0 }; } activationDelay;
	unsigned startBounceCounterDelay { 0 };
};

struct FillFlockResult {
	int64_t resultTimeout;
	unsigned numParticles;
};

[[nodiscard]]
auto fillParticleFlock( const EllipsoidalFlockParams *__restrict params,
				        Particle *__restrict particles,
				        unsigned maxParticles,
				        const Particle::AppearanceRules *__restrict appearanceRules,
				        wsw::RandomGenerator *__restrict rng,
				        int64_t currTime, signed signedStride = 1 ) -> FillFlockResult;

[[nodiscard]]
auto fillParticleFlock( const ConicalFlockParams *__restrict params,
				        Particle *__restrict particles,
				        unsigned maxParticles,
						const Particle::AppearanceRules *__restrict appearanceRules,
				  		wsw::RandomGenerator *__restrict rng,
						int64_t currTime, signed signedStride = 1 ) -> FillFlockResult;

struct alignas( 16 ) ParticleFlock {
	Particle::AppearanceRules appearanceRules;
	// Caution: No drag simulation is currently performed for non-clipped flocks
	float drag { 0.0f };
    // note that the accumulated error due simulation timestep size leads to travelling in the radial direction
    // this leads to larger spirals at lower framerates (or at lag spikes)
    // also note induced velocities by vorticity and outflow are proportional to 1/r
    // for vorticity the distance to vorticityOrigin and for outflow the distance to the line formed by the origin and vector
	float vorticity { 0.0f }; // the induced velocity up to 16 units radius from the vorticity origin
	float vorticityOrigin[3]; // the origin of the vorticity effect
	float vorticityAxis[3]; // the axis around which particles rotate
    float outflow {0.0f};
    float outflowOrigin[3]; // point used to define the line of the outflow
    float outflowAxis[3] {0.0f, 0.0f, 1.0f}; // axis of the outflow, should be a unit vector
    float turbulence { 0.0f };
    float turbulenceCoordinateScale {1.0f};
	float restitution { 0.75f };
	Particle *particles;
	int64_t timeoutAt;
	unsigned numActivatedParticles { 0 };
	unsigned numDelayedParticles { 0 };
	// Delayed particles are kept in the same memory chunk after the spawned ones.
	// delayedParticlesOffset should be >= numActivatedParticles.
	unsigned delayedParticlesOffset { 0 };
	unsigned binIndex;
	unsigned minBounceCount { 0 }, maxBounceCount { 0 };
	unsigned startBounceCounterDelay { 0 };
	float keepOnImpactProbability { 1.0f };
	CMShapeList *shapeList;
	// TODO: Make links work with "m_"
	ParticleFlock *prev { nullptr }, *next { nullptr };
	float mins[4];
	float maxs[4];
	unsigned lastLightEmitterParticleIndex;
	bool hasRotatingParticles;
};

class ParticleSystem {
public:
	static constexpr unsigned kMaxClippedTrailFlockSize = 64;
	static constexpr unsigned kMaxNonClippedTrailFlockSize = 96;

	static constexpr unsigned kClippedTrailFlocksBin = 3;
	static constexpr unsigned kNonClippedTrailFlocksBin = 4;
private:
	template<typename> friend class SingletonHolder;

	// TODO: Just align manually in the combined memory chunk
	static_assert( sizeof( ParticleFlock ) % 16 == 0 );

	struct FlocksBin {
		ParticleFlock *head { nullptr };
		wsw::HeapBasedFreelistAllocator allocator;
		const unsigned maxParticlesPerFlock;
		bool needsShapeLists { true };

		FlocksBin( unsigned maxParticlesPerFlock, unsigned maxFlocks )
			: allocator( sizeof( ParticleFlock ) + sizeof( Particle ) * maxParticlesPerFlock, maxFlocks )
			, maxParticlesPerFlock( maxParticlesPerFlock ) {}
	};

	static constexpr unsigned kMaxSmallFlocks  = 128;
	static constexpr unsigned kMaxMediumFlocks = 64;
	static constexpr unsigned kMaxLargeFlocks  = 20;

	static constexpr unsigned kMaxClippedTrailFlocks    = 32;
	static constexpr unsigned kMaxNonClippedTrailFlocks = 16;

	static constexpr unsigned kMaxSmallFlockSize  = 8;
	static constexpr unsigned kMaxMediumFlockSize = 48;
	static constexpr unsigned kMaxLargeFlockSize  = 144;

	static constexpr unsigned kMaxNumberOfClippedFlocks = kMaxClippedTrailFlocks +
		kMaxSmallFlocks + kMaxMediumFlocks + kMaxLargeFlocks;

	wsw::StaticVector<CMShapeList *, kMaxNumberOfClippedFlocks> m_freeShapeLists;

	wsw::StaticVector<FlocksBin, 5> m_bins;
	int64_t m_lastTime { 0 };

	wsw::RandomGenerator m_rng;

	// TODO: Heap-allocate (we do not want to include heavyweight std headers and there's no alternative for now).
	// Caution: Once-added data must preserve its address during frame, so relocatable vectors are not an option.
	wsw::StaticVector<Particle::AppearanceRules, 36> m_frameFlareAppearanceRules;
	wsw::StaticVector<RgbaLifespan, 256> m_frameFlareColorLifespans;
	wsw::StaticVector<Particle, 256> m_frameFlareParticles;

	void unlinkAndFree( ParticleFlock *flock );

	[[nodiscard]]
	auto createFlock( unsigned binIndex, int64_t currTime, const Particle::AppearanceRules &rules ) -> ParticleFlock *;

	template <typename FlockParams>
	void addParticleFlockImpl( const Particle::AppearanceRules &appearanceRules,
							   const FlockParams &flockParams,
							   unsigned binIndex, unsigned maxParticles );

	static void runStepKinematics( ParticleFlock *__restrict flock, float deltaSeconds, vec3_t resultBounds[2] );

	[[nodiscard]]
	static auto activateDelayedParticles( ParticleFlock *flock, int64_t currTime ) -> std::optional<int64_t>;

	static void simulate( ParticleFlock *__restrict flock, wsw::RandomGenerator *__restrict rng,
						  int64_t currTime, float deltaSeconds );
	static void simulateWithoutClipping( ParticleFlock *__restrict flock, int64_t currTime, float deltaSeconds );
public:
	ParticleSystem();
	~ParticleSystem();

	// Use this non-templated interface to reduce call site code bloat

	void addSmallParticleFlock( const Particle::AppearanceRules &rules, const EllipsoidalFlockParams &flockParams );
	void addSmallParticleFlock( const Particle::AppearanceRules &rules, const ConicalFlockParams &flockParams );

	void addMediumParticleFlock( const Particle::AppearanceRules &rules, const EllipsoidalFlockParams &flockParams );
	void addMediumParticleFlock( const Particle::AppearanceRules &rules, const ConicalFlockParams &flockParams );

	void addLargeParticleFlock( const Particle::AppearanceRules &rules, const EllipsoidalFlockParams &flockParams );
	void addLargeParticleFlock( const Particle::AppearanceRules &rules, const ConicalFlockParams &flockParams );

	// Caution: Trail particles aren't assumed to be bouncing by default
	// (otherwise, respective flock fields should be set manually)
	[[nodiscard]]
	auto createTrailFlock( const Particle::AppearanceRules &appearanceRules, unsigned binIndex ) -> ParticleFlock *;

	void destroyTrailFlock( ParticleFlock *flock ) { unlinkAndFree( flock ); }

	void runFrame( int64_t currTime, DrawSceneRequest *drawSceneRequest );

	void tryAddingLight( ParticleFlock *flock, DrawSceneRequest *drawSceneRequest );
	void tryAddingFlares( ParticleFlock *flock, DrawSceneRequest *drawSceneRequest );
};

#endif
