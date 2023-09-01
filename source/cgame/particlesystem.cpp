#include "particlesystem.h"
#include "../qcommon/links.h"
#include "../client/client.h"
#include "noise.h"
#include "cg_local.h"

ParticleSystem::ParticleSystem() {
	// TODO: All of this asks for exception-safety

	do {
		if( CMShapeList *list = CM_AllocShapeList( cl.cms ) ) [[likely]] {
			m_freeShapeLists.push_back( list );
		} else {
			wsw::failWithBadAlloc();
		}
	} while( !m_freeShapeLists.full() );

	new( m_bins.unsafe_grow_back() )FlocksBin( kMaxSmallFlockSize, kMaxSmallFlocks );
	new( m_bins.unsafe_grow_back() )FlocksBin( kMaxMediumFlockSize, kMaxMediumFlocks );
	new( m_bins.unsafe_grow_back() )FlocksBin( kMaxLargeFlockSize, kMaxLargeFlocks );
	new( m_bins.unsafe_grow_back() )FlocksBin( kMaxClippedTrailFlockSize, kMaxClippedTrailFlocks );
	auto *lastBin = new( m_bins.unsafe_grow_back() )FlocksBin( kMaxNonClippedTrailFlockSize, kMaxNonClippedTrailFlocks );
	lastBin->needsShapeLists = false;
}

ParticleSystem::~ParticleSystem() {
	for( FlocksBin &bin: m_bins ) {
		ParticleFlock *nextFlock = nullptr;
		for( ParticleFlock *flock = bin.head; flock; flock = nextFlock ) {
			nextFlock = flock->next;
			unlinkAndFree( flock );
		}
	}
	for( CMShapeList *list : m_freeShapeLists ) {
		CM_FreeShapeList( cl.cms, list );
	}
}

void ParticleSystem::unlinkAndFree( ParticleFlock *flock ) {
	FlocksBin &bin = m_bins[flock->binIndex];
	wsw::unlink( flock, &bin.head );
	if( flock->shapeList ) {
		m_freeShapeLists.push_back( flock->shapeList );
	}
	flock->~ParticleFlock();
	bin.allocator.free( flock );
}

auto ParticleSystem::createFlock( unsigned binIndex, int64_t currTime,
								  const Particle::AppearanceRules &appearanceRules ) -> ParticleFlock * {
	assert( binIndex < std::size( m_bins ) );
	FlocksBin &bin = m_bins[binIndex];

	CMShapeList *shapeList = nullptr;
	uint8_t *mem = bin.allocator.allocOrNull();
	if( !mem ) [[unlikely]] {
		ParticleFlock *oldestFlock = nullptr;
		auto leastTimeout = std::numeric_limits<int64_t>::max();
		for( ParticleFlock *flock = bin.head; flock; flock = flock->next ) {
			if( flock->timeoutAt < leastTimeout ) {
				leastTimeout = flock->timeoutAt;
				oldestFlock = flock;
			}
		}
		assert( oldestFlock );
		wsw::unlink( oldestFlock, &bin.head );
		shapeList = oldestFlock->shapeList;
		oldestFlock->~ParticleFlock();
		mem = (uint8_t *)oldestFlock;
	}

	assert( mem );
	if( bin.needsShapeLists && !shapeList ) {
		assert( !m_freeShapeLists.empty() );
		shapeList = m_freeShapeLists.back();
		m_freeShapeLists.pop_back();
	}

	auto *const particles = (Particle *)( mem + sizeof( ParticleFlock ) );
	assert( ( (uintptr_t)particles % 16 ) == 0 );

	auto *const flock = new( mem )ParticleFlock {
		.appearanceRules = appearanceRules,
		.particles       = particles,
		.binIndex        = binIndex,
		.shapeList       = shapeList,
	};

	wsw::link( flock, &bin.head );
	return flock;
}

template <typename FlockParams>
void ParticleSystem::addParticleFlockImpl( const Particle::AppearanceRules &appearanceRules,
										   const FlockParams &flockParams, unsigned binIndex, unsigned maxParticles ) {
	const int64_t currTime = cg.time;
	ParticleFlock *flock   = createFlock( binIndex, currTime, appearanceRules );

	signed fillStride;
	unsigned initialOffset, activatedCountMultiplier, delayedCountMultiplier;
	if( flockParams.activationDelay.max == 0 ) {
		fillStride               = 1;
		initialOffset            = 0;
		activatedCountMultiplier = 1;
		delayedCountMultiplier   = 0;
	} else {
		fillStride               = -1;
		initialOffset            = maxParticles - 1;
		activatedCountMultiplier = 0;
		delayedCountMultiplier   = 1;
	}

	const FillFlockResult fillResult = fillParticleFlock( std::addressof( flockParams ),
														  flock->particles + initialOffset,
														  maxParticles, std::addressof( appearanceRules ),
														  &m_rng, currTime, fillStride );

	flock->timeoutAt               = fillResult.resultTimeout;
	flock->numActivatedParticles   = fillResult.numParticles * activatedCountMultiplier;
	flock->numDelayedParticles     = fillResult.numParticles * delayedCountMultiplier;
	flock->delayedParticlesOffset  = ( initialOffset + 1 ) - fillResult.numParticles * delayedCountMultiplier;
	flock->drag                    = flockParams.drag;
	flock->vorticity               = flockParams.vorticity;
	//VectorAdd( flockParams.origin, flockParams.offset, refOrigin );
	Vector4Set( flock->refOrigin, flockParams.origin[0], flockParams.origin[1], flockParams.origin[2], 0.0f );
	Vector4Set( flock->refAxis, flockParams.refAxis[0], flockParams.refAxis[1], flockParams.refAxis[2], 0.0f );
    flock->turbulence              = flockParams.turbulence;
    flock->turbulenceScale         = flockParams.turbulenceScale;
	flock->restitution             = flockParams.restitution;
	flock->hasRotatingParticles    = flockParams.angularVelocity.min != 0.0f || flockParams.angularVelocity.max != 0.0f;
	flock->minBounceCount          = flockParams.bounceCount.minInclusive;
	flock->maxBounceCount          = flockParams.bounceCount.maxInclusive;
	flock->startBounceCounterDelay = flockParams.startBounceCounterDelay;

	if( flock->minBounceCount < flock->maxBounceCount ) {
		// Assume that probability of dropping the particle for varyingCount + 1 impacts is finalDropProbability
		// Hence that probability of not dropping it for varyingCount + 1 impacts is 1.0f - finalDropProbability
		// (We assume that almost all particles must be dropped the next step after the max one, hence the +1)
		// The probability of not dropping it every step is pow( finalKeepProbability, 1.0f / ( varyingCount + 1 ) )
		constexpr float finalDropProbability = 0.95f;
		constexpr float finalKeepProbability = 1.00f - finalDropProbability;
		const unsigned varyingCount          = flock->maxBounceCount - flock->minBounceCount;
		flock->keepOnImpactProbability       = std::pow( finalKeepProbability, Q_Rcp( (float)( varyingCount + 1 ) ) );
	}
}

void ParticleSystem::addSmallParticleFlock( const Particle::AppearanceRules &rules,
											const EllipsoidalFlockParams &flockParams ) {
	addParticleFlockImpl<EllipsoidalFlockParams>( rules, flockParams, 0, kMaxSmallFlockSize );
}

void ParticleSystem::addSmallParticleFlock( const Particle::AppearanceRules &rules,
											const ConicalFlockParams &flockParams ) {
	addParticleFlockImpl<ConicalFlockParams>( rules, flockParams, 0, kMaxSmallFlockSize );
}

void ParticleSystem::addMediumParticleFlock( const Particle::AppearanceRules &rules,
											 const EllipsoidalFlockParams &flockParams ) {
	addParticleFlockImpl<EllipsoidalFlockParams>( rules, flockParams, 1, kMaxMediumFlockSize );
}

void ParticleSystem::addMediumParticleFlock( const Particle::AppearanceRules &rules,
											 const ConicalFlockParams &flockParams ) {
	addParticleFlockImpl<ConicalFlockParams>( rules, flockParams, 1, kMaxMediumFlockSize );
}

void ParticleSystem::addLargeParticleFlock( const Particle::AppearanceRules &rules,
											const EllipsoidalFlockParams &flockParams ) {
	addParticleFlockImpl<EllipsoidalFlockParams>( rules, flockParams, 2, kMaxLargeFlockSize );
}

void ParticleSystem::addLargeParticleFlock( const Particle::AppearanceRules &rules,
											const ConicalFlockParams &flockParams ) {
	addParticleFlockImpl<ConicalFlockParams>( rules, flockParams, 2, kMaxLargeFlockSize );
}

auto ParticleSystem::createTrailFlock( const Particle::AppearanceRules &rules, unsigned binIndex ) -> ParticleFlock * {
	assert( binIndex == kClippedTrailFlocksBin || binIndex == kNonClippedTrailFlocksBin );

	// Don't let it evict anything
	const int64_t currTime = std::numeric_limits<int64_t>::min();
	ParticleFlock *flock   = createFlock( binIndex, currTime, rules );

	// Externally managed
	flock->timeoutAt             = std::numeric_limits<int64_t>::max();
	flock->numActivatedParticles = 0;
	flock->numDelayedParticles   = 0;

	return flock;
}

auto fillParticleFlock( const EllipsoidalFlockParams *__restrict params,
						Particle *__restrict particles,
						unsigned maxParticles,
						const Particle::AppearanceRules *__restrict appearanceRules,
						wsw::RandomGenerator *__restrict rng,
						int64_t currTime, signed signedStride )
	-> FillFlockResult {
	const vec3_t initialOrigin {
		params->origin[0] + params->offset[0],
		params->origin[1] + params->offset[1],
		params->origin[2] + params->offset[2]
	};

	unsigned numParticles = maxParticles;
	// We do not specify the exact bounds but a percentage
	// so we can use the same filler for different bin flocks.
	if( params->percentage.min != 1.0f && params->percentage.max != 1.0f ) {
		assert( params->percentage.min >= 0.0f && params->percentage.min <= 1.0f );
		assert( params->percentage.max >= 0.0f && params->percentage.min <= 1.0f );
		const float percentage = rng->nextFloat( params->percentage.min, params->percentage.max );
		numParticles = (unsigned)( (float)maxParticles * percentage );
		numParticles = wsw::clamp( numParticles, 1u, maxParticles );
	}

	assert( params->speed.min >= 0.0f && params->speed.min <= 1000.0f );
	assert( params->speed.max >= 0.0f && params->speed.max <= 1000.0f );
	assert( params->speed.min <= params->speed.max );

	// Negative shift speed could be feasible
	assert( params->shiftSpeed.min <= params->shiftSpeed.max );
	assert( std::fabs( VectorLength( params->shiftDir ) - 1.0f ) < 1e-3f );

	assert( params->angularVelocity.min <= params->angularVelocity.max );

	assert( params->activationDelay.min <= params->activationDelay.max );
	assert( params->activationDelay.max <= 10'000 );

	const vec3_t *__restrict dirs = ::kPredefinedDirs;

	assert( params->timeout.min && params->timeout.min <= params->timeout.max && params->timeout.max < 3000 );
	const unsigned timeoutSpread = params->timeout.max - params->timeout.min;
	auto resultTimeout = std::numeric_limits<int64_t>::min();

	const bool hasMultipleMaterials       = appearanceRules->numMaterials > 1;
	const bool hasMultipleColors          = appearanceRules->colors.size() > 1;
	const bool hasSpeedShift              = params->shiftSpeed.min != 0.0f || params->shiftSpeed.max != 0.0f;
	const bool isSpherical                = params->stretchScale == 1.0f;
	const bool hasVariableAngularVelocity = params->angularVelocity.min < params->angularVelocity.max;
	const bool hasVariableDelay           = params->activationDelay.min < params->activationDelay.max;

	unsigned colorsIndexMask = 0, materialsIndexMask = 0;
	if( hasMultipleColors ) {
		if( const unsigned maybeMask = appearanceRules->colors.size() - 1; ( maybeMask & ( maybeMask + 1 ) ) == 0 ) {
			colorsIndexMask = maybeMask;
		}
	}
	if( hasMultipleMaterials ) {
		if( const unsigned maybeMask = appearanceRules->numMaterials - 1; ( maybeMask & ( maybeMask + 1 ) ) == 0 ) {
			materialsIndexMask = maybeMask;
		}
	}

	assert( std::fabs( VectorLength( params->stretchDir ) - 1.0f ) < 1e-3f );

	for( unsigned i = 0; i < numParticles; ++i ) {
		Particle *const __restrict p = particles + signedStride * (signed)i;

		Vector4Set( p->oldOrigin, initialOrigin[0], initialOrigin[1], initialOrigin[2], 0.0f );
		if( params->vorticity > 0.0f ) {
			Vector4Copy( p->oldOrigin, p->refOrigin );
		}
		Vector4Set( p->accel, 0, 0, -params->gravity, 0 );

		const float *__restrict randomDir = dirs[rng->nextBounded( NUMVERTEXNORMALS )];
		const float speed = rng->nextFloat( params->speed.min, params->speed.max );

		// We try relying on branch prediction facilities
		// TODO: Add template/if constexpr specializations
		if( isSpherical ) {
			VectorScale( randomDir, speed, p->velocity );
		} else {
			const float stretchScale = params->stretchScale;
			const float stretchDot   = DotProduct( randomDir, params->stretchDir );
			vec3_t alignedPart, perpPart;
			VectorScale( params->stretchDir, stretchDot, alignedPart );
			VectorSubtract( randomDir, alignedPart, perpPart );
			// Compute using the prior knowledge instead of a late normalization to break dependency chains
			const float newAlignedSquareLen = ( stretchDot * stretchScale ) * ( stretchDot * stretchScale );
			const float perpSquareLen       = VectorLengthSquared( perpPart );
			// TODO: Supply hints that the arg is non-zero
			const float rcpNewDirLen  = Q_RSqrt( newAlignedSquareLen + perpSquareLen );
			const float velocityScale = speed * rcpNewDirLen;
			// Combine the perpendicular part with the aligned part using the strech scale
			VectorMA( perpPart, stretchScale, alignedPart, p->velocity );
			// Normalize and scale by the speed
			VectorScale( p->velocity, velocityScale, p->velocity );
		}

		if( hasSpeedShift ) {
			const float shift = rng->nextFloat( params->shiftSpeed.min, params->shiftSpeed.max );
			VectorMA( p->velocity, shift, params->shiftDir, p->velocity );
		}

		p->velocity[3] = 0.0f;

		p->rotationAngle = 0.0f;
		if( hasVariableAngularVelocity ) {
			p->rotationAxisIndex = rng->nextBoundedFast( std::size( kPredefinedDirs ) );
			p->angularVelocity   = rng->nextFloat( params->angularVelocity.min, params->angularVelocity.max );
		} else {
			p->rotationAxisIndex = 0;
			p->angularVelocity   = params->angularVelocity.min;
		}

		p->spawnTime   = currTime;
		p->lifetime    = params->timeout.min + rng->nextBoundedFast( timeoutSpread );
		p->bounceCount = 0;

		p->activationDelay = params->activationDelay.min;
		if( hasVariableDelay ) [[unlikely]] {
			p->activationDelay += rng->nextBoundedFast( params->activationDelay.max - params->activationDelay.min );
		}

		// TODO: Branchless?
		resultTimeout = wsw::max( p->spawnTime + p->lifetime, resultTimeout );

		const uint32_t randomDword = rng->next();
		p->instanceWidthSpreadFraction   = (int8_t)( ( randomDword >> 0 ) & 0xFF );
		p->instanceLengthSpreadFraction  = (int8_t)( ( randomDword >> 8 ) & 0xFF );
		p->instanceRadiusSpreadFraction  = (int8_t)( ( randomDword >> 16 ) & 0xFF );

		p->instanceWidthExtraScale = p->instanceLengthExtraScale = p->instanceRadiusExtraScale = 1;

		if( hasMultipleMaterials ) {
			if( materialsIndexMask ) {
				p->instanceMaterialIndex = rng->next() & materialsIndexMask;
			} else {
				p->instanceMaterialIndex = (uint8_t)rng->nextBounded( appearanceRules->numMaterials );
			}
		} else {
			p->instanceMaterialIndex = 0;
		}
		if( hasMultipleColors ) {
			if( colorsIndexMask ) {
				p->instanceColorIndex = rng->next() & colorsIndexMask;
			} else {
				p->instanceColorIndex = (uint8_t)rng->nextBounded( appearanceRules->colors.size() );
			}
		} else {
			p->instanceColorIndex = 0;
		}
	}

	return FillFlockResult { .resultTimeout = resultTimeout, .numParticles = numParticles };
}

auto fillParticleFlock( const ConicalFlockParams *__restrict params,
						Particle *__restrict particles,
						unsigned maxParticles,
						const Particle::AppearanceRules *__restrict appearanceRules,
						wsw::RandomGenerator *__restrict rng,
						int64_t currTime, signed signedStride )
	-> FillFlockResult {
	const vec3_t initialOrigin {
		params->origin[0] + params->offset[0],
		params->origin[1] + params->offset[1],
		params->origin[2] + params->offset[2]
	};

	unsigned numParticles = maxParticles;
	if( params->percentage.min != 1.0f && params->percentage.max != 1.0f ) {
		assert( params->percentage.min >= 0.0f && params->percentage.min <= 1.0f );
		assert( params->percentage.max >= 0.0f && params->percentage.min <= 1.0f );
		// We do not specify the exact bounds but a percentage
		// so we can use the same filler for different bin flocks.
		const float percentage = rng->nextFloat( params->percentage.min, params->percentage.max );
		numParticles = (unsigned)( (float)maxParticles * percentage );
		numParticles = wsw::clamp( numParticles, 1u, maxParticles );
	}

	assert( params->speed.min >= 0.0f && params->speed.min <= 1000.0f );
	assert( params->speed.max >= 0.0f && params->speed.max <= 1000.0f );
	assert( params->speed.min <= params->speed.max );
	assert( std::fabs( VectorLength( params->dir ) - 1.0f ) < 1e-3f );

	// Negative shift speed could be feasible
	assert( params->shiftSpeed.min <= params->shiftSpeed.max );
	assert( std::fabs( VectorLength( params->shiftDir ) - 1.0f ) < 1e-3f );

	assert( params->angle >= 0.0f && params->angle <= 180.0f );
	assert( params->innerAngle >= 0.0f && params->innerAngle <= 180.0f );
	assert( params->innerAngle < params->angle );

	assert( params->angularVelocity.min <= params->angularVelocity.max );

	assert( params->activationDelay.min <= params->activationDelay.max );
	assert( params->activationDelay.max <= 10'000 );

	// TODO: Supply cosine values as parameters?

	float maxZ = 1.0f;
	if( params->innerAngle > 0.0f ) [[unlikely]] {
		maxZ = std::cos( (float) DEG2RAD( params->innerAngle ) );
	}

	const float minZ = std::cos( (float)DEG2RAD( params->angle ) );

	assert( params->timeout.min && params->timeout.min <= params->timeout.max && params->timeout.max < 3000 );
	const unsigned timeoutSpread = params->timeout.max - params->timeout.min;
	auto resultTimeout = std::numeric_limits<int64_t>::min();

	mat3_t transformMatrix;
	Matrix3_ForRotationOfDirs( &axis_identity[AXIS_UP], params->dir, transformMatrix );

	const bool hasMultipleMaterials       = appearanceRules->numMaterials > 1;
	const bool hasMultipleColors          = appearanceRules->colors.size() > 1;
	const bool hasSpeedShift              = params->shiftSpeed.min != 0.0f || params->shiftSpeed.max != 0.0f;
	const bool hasVariableAngularVelocity = params->angularVelocity.min < params->angularVelocity.max;
	const bool hasVariableDelay           = params->activationDelay.min != params->activationDelay.max;

	unsigned colorsIndexMask = 0, materialsIndexMask = 0;
	if( hasMultipleColors ) {
		if( const unsigned maybeMask = appearanceRules->colors.size() - 1; ( maybeMask & ( maybeMask + 1 ) ) == 0 ) {
			colorsIndexMask = maybeMask;
		}
	}
	if( hasMultipleMaterials ) {
		if( const unsigned maybeMask = appearanceRules->numMaterials - 1; ( maybeMask & ( maybeMask + 1 ) ) == 0 ) {
			materialsIndexMask = maybeMask;
		}
	}

	// TODO: Make cached conical samples for various angles?
	for( unsigned i = 0; i < numParticles; ++i ) {
		Particle *const __restrict p = particles + signedStride * (signed)i;

		Vector4Set( p->oldOrigin, initialOrigin[0], initialOrigin[1], initialOrigin[2], 0.0f );
		if( params->vorticity > 0.0f ) {
			Vector4Copy( p->oldOrigin, p->refOrigin );
		}
		Vector4Set( p->accel, 0, 0, -params->gravity, 0 );

		// https://math.stackexchange.com/a/205589
		const float z   = rng->nextFloat( minZ, maxZ );
		const float r   = Q_Sqrt( 1.0f - z * z );
		const float phi = rng->nextFloat( 0.0f, 2.0f * (float)M_PI );

		const float speed = rng->nextFloat( params->speed.min, params->speed.max );
		const vec3_t untransformed { speed * r * std::cos( phi ), speed * r * std::sin( phi ), speed * z };
		Matrix3_TransformVector( transformMatrix, untransformed, p->velocity );


		// We try relying on branch prediction facilities
		// TODO: Add template/if constexpr specializations
		if( hasSpeedShift ) {
			const float shift = rng->nextFloat( params->shiftSpeed.min, params->shiftSpeed.max );
			VectorMA( p->velocity, shift, params->shiftDir, p->velocity );
		}

		p->velocity[3] = 0.0f;

		p->rotationAngle = 0.0f;
		if( hasVariableAngularVelocity ) {
			p->rotationAxisIndex = rng->nextBoundedFast( std::size( kPredefinedDirs ) );
			p->angularVelocity   = rng->nextFloat( params->angularVelocity.min, params->angularVelocity.max );
		} else {
			p->rotationAxisIndex = 0;
			p->angularVelocity   = params->angularVelocity.min;
		}

		p->spawnTime   = currTime;
		p->lifetime    = params->timeout.min + rng->nextBoundedFast( timeoutSpread );
		p->bounceCount = 0;

		p->activationDelay = params->activationDelay.min;
		if( hasVariableDelay ) [[unlikely]] {
			p->activationDelay += rng->nextBoundedFast( params->activationDelay.max - params->activationDelay.min );
		}

		// TODO: Branchless?
		resultTimeout = wsw::max( p->spawnTime + p->lifetime, resultTimeout );

		const uint32_t randomDword = rng->next();
		p->instanceWidthSpreadFraction   = (int8_t)( ( randomDword >> 0 ) & 0xFF );
		p->instanceLengthSpreadFraction  = (int8_t)( ( randomDword >> 8 ) & 0xFF );
		p->instanceRadiusSpreadFraction  = (int8_t)( ( randomDword >> 16 ) & 0xFF );

		p->instanceWidthExtraScale = p->instanceLengthExtraScale = p->instanceRadiusExtraScale = 1;

		if( hasMultipleMaterials ) {
			if( materialsIndexMask ) {
				p->instanceMaterialIndex = rng->next() & materialsIndexMask;
			} else {
				p->instanceMaterialIndex = (uint8_t)rng->nextBounded( appearanceRules->numMaterials );
			}
		} else {
			p->instanceMaterialIndex = 0;
		}
		if( hasMultipleColors ) {
			if( colorsIndexMask ) {
				p->instanceColorIndex = rng->next() & colorsIndexMask;
			} else {
				p->instanceColorIndex = (uint8_t)rng->nextBounded( appearanceRules->colors.size() );
			}
		} else {
			p->instanceColorIndex = 0;
		}
	}

	return FillFlockResult { .resultTimeout = resultTimeout, .numParticles = numParticles };
}

[[nodiscard]]
static inline bool canShowForCurrentCgFrame( unsigned affinityIndex, unsigned affinityModulo ) {
	if( affinityModulo > 1 ) {
		assert( affinityIndex < affinityModulo );
		const auto moduloAsCountType = ( decltype( cg.frameCount ) )affinityModulo;
		const auto indexAsCountType  = ( decltype( cg.frameCount ) )affinityIndex;
		return indexAsCountType == cg.frameCount % moduloAsCountType;
	}
	return true;
}

void ParticleSystem::runFrame( int64_t currTime, DrawSceneRequest *request ) {
	// Limit delta by sane bounds
	const float deltaSeconds = 1e-3f * (float)wsw::clamp( (int)( currTime - m_lastTime ), 1, 33 );
	m_lastTime = currTime;
    unsigned int count;
    count = 0;

	// We split simulation/rendering loops for a better instructions cache utilization
	for( FlocksBin &bin: m_bins ) {
		ParticleFlock *nextFlock = nullptr;
		for( ParticleFlock *flock = bin.head; flock; flock = nextFlock ) {
			nextFlock = flock->next;
			if( currTime < flock->timeoutAt ) [[likely]] {
				// Otherwise, the flock could be awaiting filling externally, don't modify its timeout
				if( flock->numActivatedParticles + flock->numDelayedParticles > 0 ) [[likely]] {
					if( flock->shapeList ) {
						simulate( flock, &m_rng, currTime, deltaSeconds );
                        count += flock->numActivatedParticles;
					} else {
						simulateWithoutClipping( flock, currTime, deltaSeconds );
					}
				}
			} else {
				unlinkAndFree( flock );
			}
		}
	}
    if ( count > 1 ) {
        Com_Printf("amount of particles:%i", count);
    }

	m_frameFlareParticles.clear();
	m_frameFlareColorLifespans.clear();
	m_frameFlareAppearanceRules.clear();

	for( FlocksBin &bin: m_bins ) {
		for( ParticleFlock *flock = bin.head; flock; flock = flock->next ) {
			if( const unsigned numParticles = flock->numActivatedParticles ) [[likely]] {
				request->addParticles( flock->mins, flock->maxs, flock->appearanceRules, flock->particles, numParticles );
				const Particle::AppearanceRules &rules = flock->appearanceRules;
				if( !rules.lightProps.empty() ) [[unlikely]] {
					// If the light display is tied to certain frames (e.g., every 3rd one, starting from 2nd absolute)
					if( canShowForCurrentCgFrame( rules.lightFrameAffinityIndex, rules.lightFrameAffinityModulo ) ) {
						tryAddingLight( flock, request );
					}
				}
				if( rules.flareProps ) [[unlikely]] {
					const Particle::FlareProps &props = *rules.flareProps;
					if( canShowForCurrentCgFrame( props.flockFrameAffinityIndex, props.flockFrameAffinityModulo ) ) {
						tryAddingFlares( flock, request );
					}
				}
			}
		}
	}
}

void ParticleSystem::tryAddingLight( ParticleFlock *flock, DrawSceneRequest *drawSceneRequest ) {
	const Particle::AppearanceRules &rules = flock->appearanceRules;
	assert( flock->numActivatedParticles );
	assert( rules.lightProps.size() == 1 || rules.lightProps.size() == rules.colors.size() );

	flock->lastLightEmitterParticleIndex = ( flock->lastLightEmitterParticleIndex + 1 ) % flock->numActivatedParticles;
	const Particle &particle = flock->particles[flock->lastLightEmitterParticleIndex];
	assert( particle.lifetimeFrac >= 0.0f && particle.lifetimeFrac <= 1.0f );

	const LightLifespan *lightLifespan;
	if( rules.lightProps.size() == 1 ) {
		lightLifespan = rules.lightProps.data();
	} else {
		lightLifespan = rules.lightProps.data() + particle.instanceColorIndex;
	}

	float lightRadius, lightColor[3];
	lightLifespan->getRadiusAndColorForLifetimeFrac( particle.lifetimeFrac, &lightRadius, lightColor );
	if( lightRadius >= 1.0f ) {
		drawSceneRequest->addLight( particle.origin, lightRadius, 0.0f, lightColor );
	}
}

void ParticleSystem::tryAddingFlares( ParticleFlock *flock, DrawSceneRequest *drawSceneRequest ) {
	assert( m_frameFlareParticles.size() == m_frameFlareColorLifespans.size() );

	if( m_frameFlareParticles.full() ) [[unlikely]] {
		return;
	}
	if( m_frameFlareAppearanceRules.full() ) [[unlikely]] {
		return;
	}

	assert( flock->numActivatedParticles );
	assert( flock->appearanceRules.flareProps );
	const Particle::FlareProps &flareProps = *flock->appearanceRules.flareProps;
	assert( flareProps.lightProps.size() == 1 || flareProps.lightProps.size() == flock->appearanceRules.colors.size() );

	const unsigned oldNumFrameFlareParticles = m_frameFlareParticles.size();

	unsigned frameCountByModulo = 0;
	if( unsigned modulo = flareProps.particleFrameAffinityModulo; modulo > 1 ) {
		frameCountByModulo = (unsigned)( cg.frameCount % (decltype( cg.frameCount ) )modulo );
	}

	BoundsBuilder boundsBuilder;
	unsigned numAddedParticles  = 0;
	unsigned flockParticleIndex = 0;

	do {
		if( flareProps.particleFrameAffinityModulo > 1 ) {
			if( ( flockParticleIndex % flareProps.particleFrameAffinityModulo ) != frameCountByModulo ) {
				continue;
			}
		}

		const Particle &baseParticle = flock->particles[flockParticleIndex];
		assert( baseParticle.lifetimeFrac >= 0.0f && baseParticle.lifetimeFrac <= 1.0f );

		// TODO: Lift this condition out of the loop?
		const LightLifespan *lightLifespan;
		if( flareProps.lightProps.size() == 1 ) {
			lightLifespan = flareProps.lightProps.data();
		} else {
			lightLifespan = flareProps.lightProps.data() + baseParticle.instanceColorIndex;
		}

		float lightRadius, lightColor[3];
		lightLifespan->getRadiusAndColorForLifetimeFrac( baseParticle.lifetimeFrac, &lightRadius, lightColor );

		assert( flareProps.radiusScale > 0.0f );
		lightRadius *= flareProps.radiusScale;

		if( lightRadius >= 1.0f ) {
			auto *const addedParticle = new( m_frameFlareParticles.unsafe_grow_back() )Particle( baseParticle );

			addedParticle->lifetimeFrac          = 0.0f;
			addedParticle->instanceColorIndex    = numAddedParticles;
			addedParticle->instanceMaterialIndex = 0;

			// Keep radius in the appearance rules the same (the thing we have to do), modify instance radius scale
			addedParticle->instanceRadiusExtraScale = (int8_t)lightRadius;

			// TODO: This kind of sucks, can't we just supply inline colors?
			m_frameFlareColorLifespans.push_back( RgbaLifespan {
				.initial = { lightColor[0], lightColor[1], lightColor[2], flareProps.alphaScale },
			});

			// TODO: Load 4 components explicitly
			boundsBuilder.addPoint( addedParticle->origin );
			numAddedParticles++;

			if( m_frameFlareParticles.full() ) [[unlikely]] {
				break;
			}
		}
	} while( ++flockParticleIndex < flock->numActivatedParticles );

	assert( m_frameFlareParticles.size() == m_frameFlareColorLifespans.size() );

	if( numAddedParticles ) {
		const Particle *const addedParticles  = m_frameFlareParticles.data() + oldNumFrameFlareParticles;
		const RgbaLifespan *const addedColors = m_frameFlareColorLifespans.data() + oldNumFrameFlareParticles;
		m_frameFlareAppearanceRules.emplace_back( Particle::AppearanceRules {
			.materials     = cgs.media.shaderParticleFlare.getAddressOfHandle(),
			.colors        = { addedColors, numAddedParticles },
			.geometryRules = Particle::SpriteRules { .radius = { .mean = 1.0f } },
		});

		vec4_t mins, maxs;
		boundsBuilder.storeTo( mins, maxs );

		drawSceneRequest->addParticles( mins, maxs, m_frameFlareAppearanceRules.back(), addedParticles, numAddedParticles );
	}
}

void ParticleSystem::runStepKinematics( ParticleFlock *__restrict flock, float deltaSeconds, vec3_t resultBounds[2] ) {
	assert( flock->numActivatedParticles );

	BoundsBuilder boundsBuilder;
	vec4_t accel = {0.f, 0.f, 0.f, 0.f};

	for( unsigned i = 0; i < flock->numActivatedParticles; ++i ) {
		Particle *const __restrict particle = flock->particles + i;
		Vector4Copy( particle->accel, accel );
        Vector4Set( particle->effectiveVelocity, 0.f, 0.f, 0.f, 0.f );

		if( flock->drag > 0.0f ) {
			if( const float speed = VectorLengthFast( particle->velocity ); speed > 1.0f ) [[likely]] {
				const float drag = flock->drag * speed;
				VectorMA( accel, -drag, particle->velocity, accel );
			}
		}
		if( flock->vorticity > 0.0f ) {
			vec4_t refOffset;
			VectorSubtract( particle->origin, flock->refOrigin, refOffset );
			vec4_t vorticityAxis;
			Vector4Set( vorticityAxis, 0.0f, 0.0f, 1.0f, 0.0f );
			vec4_t vorticity;
			CrossProduct( refOffset, flock->refAxis, vorticity );
			VectorNormalizeFast( vorticity );
			VectorMA( accel, flock->vorticity, vorticity, accel );
		}
        if( flock->turbulence > 0.0f && particle->lifetimeFrac > 0.1f ) {
            vec4_t scaledOrigin;
            VectorScale( particle->origin, flock->turbulenceScale, scaledOrigin);
            vec4_t turbulence;
            SimplexNoiseCurl(scaledOrigin[0], scaledOrigin[1], scaledOrigin[2], turbulence);
            VectorScale(turbulence, flock->turbulence, turbulence);
            VectorAdd( particle->effectiveVelocity, turbulence, particle->effectiveVelocity );
        }

		VectorMA( particle->velocity, deltaSeconds, accel, particle->velocity );
        VectorAdd( particle->effectiveVelocity, particle->velocity, particle->effectiveVelocity );
		VectorMA( particle->oldOrigin, deltaSeconds, particle->effectiveVelocity, particle->origin );
		// TODO: Supply this 4-component vector explicitly
		boundsBuilder.addPoint( particle->origin );

        Com_Printf("origin: %f %f %f", particle->origin[0], particle->origin[1], particle->origin[2]);

		if( flock->hasRotatingParticles ) {
			particle->rotationAngle += particle->angularVelocity * deltaSeconds;
			particle->rotationAngle = AngleNormalize360( particle->rotationAngle );
		}
	}


	boundsBuilder.storeToWithAddedEpsilon( resultBounds[0], resultBounds[1] );
}

[[nodiscard]]
static inline auto computeParticleLifetimeFrac( int64_t currTime, const Particle &__restrict particle,
												const Particle::AppearanceRules &__restrict rules ) -> float {
	const auto offset                 = (int)particle.activationDelay;
	const auto correctedDuration      = wsw::max( 1, particle.lifetime - offset );
	const auto lifetimeSoFar          = (int)( currTime - particle.spawnTime );
	const auto correctedLifetimeSoFar = wsw::max( 0, lifetimeSoFar - offset );
	return (float)correctedLifetimeSoFar * Q_Rcp( (float)correctedDuration );
}

void ParticleSystem::simulate( ParticleFlock *__restrict flock, wsw::RandomGenerator *__restrict rng,
							   int64_t currTime, float deltaSeconds ) {
	assert( flock->shapeList && flock->numActivatedParticles + flock->numDelayedParticles > 0 );

	auto timeoutOfParticlesLeft = std::numeric_limits<int64_t>::min();
	if( const auto maybeTimeoutOfDelayedParticles = activateDelayedParticles( flock, currTime ) ) {
		timeoutOfParticlesLeft = *maybeTimeoutOfDelayedParticles;
	}

	// Skip the further simulation if we do not have activated particles and did not manage to activate some.
	if( !flock->numActivatedParticles ) {
		// This also schedules disposal in case if all delayed particles
		// got timed out prior to activation during the activation attempt.
		flock->timeoutAt = timeoutOfParticlesLeft;
		return;
	}

	vec3_t possibleBounds[2];
	runStepKinematics( flock, deltaSeconds, possibleBounds );

	// TODO: Add a fused call
	CM_BuildShapeList( cl.cms, flock->shapeList, possibleBounds[0], possibleBounds[1], MASK_SOLID );
	CM_ClipShapeList( cl.cms, flock->shapeList, flock->shapeList, possibleBounds[0], possibleBounds[1] );

	// TODO: Let the BoundsBuilder store 4-component vectors
	VectorCopy( possibleBounds[0], flock->mins );
	VectorCopy( possibleBounds[1], flock->maxs );
	flock->mins[3] = 0.0f, flock->maxs[3] = 1.0f;

	assert( flock->restitution >= 0.0f && flock->restitution <= 1.0f );

	// Skip collision calls if the built shape list is empty
	if( CM_GetNumShapesInShapeList( flock->shapeList ) == 0 ) {
		unsigned particleIndex = 0;
		do {
			Particle *const __restrict p = flock->particles + particleIndex;
			assert( p->spawnTime + p->activationDelay <= currTime );
			const int64_t particleTimeoutAt = p->spawnTime + p->lifetime;

			if( particleTimeoutAt > currTime ) [[likely]] {
				// Save the current origin as the old origin
				VectorCopy( p->origin, p->oldOrigin );
				p->lifetimeFrac = computeParticleLifetimeFrac( currTime, *p, flock->appearanceRules );
				timeoutOfParticlesLeft = wsw::max( particleTimeoutAt, timeoutOfParticlesLeft );
				++particleIndex;
			} else {
				// Replace by the last particle
				flock->particles[particleIndex] = flock->particles[--flock->numActivatedParticles];
			}
		} while( particleIndex < flock->numActivatedParticles );
	} else {
		trace_t trace;

		unsigned particleIndex = 0;
		do {
			Particle *const __restrict p = flock->particles + particleIndex;
			assert( p->spawnTime + p->activationDelay <= currTime );
			const int64_t particleTimeoutAt = p->spawnTime + p->lifetime;

			if( particleTimeoutAt <= currTime ) [[unlikely]] {
				// Replace by the last particle
				flock->particles[particleIndex] = flock->particles[--flock->numActivatedParticles];
				continue;
			}

			CM_ClipToShapeList( cl.cms, flock->shapeList, &trace, p->oldOrigin,
								p->origin, vec3_origin, vec3_origin, MASK_SOLID );

			if( trace.fraction == 1.0f ) [[likely]] {
				// Save the current origin as the old origin
				VectorCopy( p->origin, p->oldOrigin );
				p->lifetimeFrac = computeParticleLifetimeFrac( currTime, *p, flock->appearanceRules );
				timeoutOfParticlesLeft = wsw::max( particleTimeoutAt, timeoutOfParticlesLeft );
				++particleIndex;
				continue;
			}

			bool keepTheParticleByImpactRules = false;
			if( !( trace.allsolid | trace.startsolid ) && !( trace.contents & CONTENTS_WATER ) ) [[likely]] {
				keepTheParticleByImpactRules = true;
				// Skip checking/updating the bounce counter during startBounceCounterDelay from spawn.
				// This helps with particles that should not normally bounce but happen to touch surfaces at start.
				// This condition always holds for a zero skipBounceCounterDelay.
				if ( p->spawnTime + flock->startBounceCounterDelay <= currTime ) [[likely]] {
					p->bounceCount++;
					if ( p->bounceCount > flock->maxBounceCount ) {
						keepTheParticleByImpactRules = false;
					} else if ( flock->keepOnImpactProbability != 1.0f && p->bounceCount > flock->minBounceCount ) {
						keepTheParticleByImpactRules = rng->tryWithChance( flock->keepOnImpactProbability );
					}
				}
			}

			if( !keepTheParticleByImpactRules ) [[unlikely]] {
				// Replace by the last particle
				flock->particles[particleIndex] = flock->particles[--flock->numActivatedParticles];
				continue;
			}

			// Reflect the velocity
			const float oldSquareSpeed    = VectorLengthSquared( p->effectiveVelocity );
			const float oldSpeedThreshold = 1.0f;
			const float newSpeedThreshold = oldSpeedThreshold * Q_Rcp( flock->restitution );
			if( oldSquareSpeed < wsw::square( newSpeedThreshold ) ) [[unlikely]] {
				// Replace by the last particle
				flock->particles[particleIndex] = flock->particles[--flock->numActivatedParticles];
				continue;
			}

			const float rcpOldSpeed = Q_RSqrt( oldSquareSpeed );
			vec3_t oldVelocityDir { p->effectiveVelocity[0], p->effectiveVelocity[1], p->effectiveVelocity[2] };
			VectorScale( oldVelocityDir, rcpOldSpeed, oldVelocityDir );

			vec3_t reflectedVelocityDir;
			VectorReflect( oldVelocityDir, trace.plane.normal, 0, reflectedVelocityDir );

			if( p->lifetime > 32 ) {
				addRandomRotationToDir( reflectedVelocityDir, rng, 0.70f, 0.97f );
			}

			const float newSpeed = flock->restitution * ( oldSquareSpeed * rcpOldSpeed );
			// Save the reflected velocity
			VectorScale( reflectedVelocityDir, newSpeed, p->velocity );
            Com_Printf("velocity: %f %f %f", p->velocity[0], p->velocity[1], p->velocity[2]);

			// Save the trace endpos with a slight offset as an origin for the next step.
			// This is not really correct but is OK.
			VectorAdd( trace.endpos, reflectedVelocityDir, p->oldOrigin );



            float debugBeamScale = Cvar_Value("debug_impact");
            if (debugBeamScale > 0.0f) {
                vec3_t to;
                VectorMA(p->oldOrigin, debugBeamScale, p->velocity, to);
                vec3_t color = {0.0f, 1.0f, 0.0f};
                cg.effectsSystem.spawnGameDebugBeam(p->oldOrigin, to, color, 0);
            }

			p->lifetimeFrac = computeParticleLifetimeFrac( currTime, *p, flock->appearanceRules );
			timeoutOfParticlesLeft = wsw::max( particleTimeoutAt, timeoutOfParticlesLeft );
			++particleIndex;
		} while( particleIndex < flock->numActivatedParticles );
	}

	flock->timeoutAt = timeoutOfParticlesLeft;
}

void ParticleSystem::simulateWithoutClipping( ParticleFlock *__restrict flock, int64_t currTime, float deltaSeconds ) {
	assert( !flock->shapeList && flock->numActivatedParticles + flock->numDelayedParticles > 0 );

	auto timeoutOfParticlesLeft = std::numeric_limits<int64_t>::min();
	if( const auto maybeTimeoutOfDelayedParticles = activateDelayedParticles( flock, currTime ) ) {
		timeoutOfParticlesLeft = *maybeTimeoutOfDelayedParticles;
	}

	BoundsBuilder boundsBuilder;
	for( unsigned i = 0; i < flock->numActivatedParticles; ) {
		Particle *const __restrict p = flock->particles + i;
		assert( p->spawnTime + p->activationDelay <= currTime );
		if( const int64_t particleTimeoutAt = p->spawnTime + p->lifetime; particleTimeoutAt > currTime ) [[likely]] {
			// TODO: Two origins are redundant for non-clipped particles
			// TODO: Simulate drag
			VectorMA( p->velocity, deltaSeconds, p->accel, p->velocity );

			VectorMA( p->oldOrigin, deltaSeconds, p->velocity, p->origin );
			VectorCopy( p->origin, p->oldOrigin );

			boundsBuilder.addPoint( p->origin );

			timeoutOfParticlesLeft = wsw::max( particleTimeoutAt, timeoutOfParticlesLeft );
			p->lifetimeFrac = computeParticleLifetimeFrac( currTime, *p, flock->appearanceRules );

			if( flock->hasRotatingParticles ) {
				p->rotationAngle += p->angularVelocity * deltaSeconds;
				p->rotationAngle = wsw::clamp( p->rotationAngle, 0.0f, 360.0f );
			}

			++i;
		} else {
			flock->numActivatedParticles--;
			flock->particles[i] = flock->particles[flock->numActivatedParticles];
		}
	}

	// We still have to compute and store bounds as they're used by the renderer culling systems
	if( flock->numActivatedParticles ) {
		vec3_t possibleMins, possibleMaxs;
		boundsBuilder.storeToWithAddedEpsilon( possibleMins, possibleMaxs );
		// TODO: Let the BoundsBuilder store 4-component vectors
		VectorCopy( possibleMins, flock->mins );
		VectorCopy( possibleMaxs, flock->maxs );
		flock->mins[3] = 0.0f, flock->maxs[3] = 1.0f;
	} else {
		constexpr float minVal = std::numeric_limits<float>::max(), maxVal = std::numeric_limits<float>::lowest();
		Vector4Set( flock->mins, minVal, minVal, minVal, minVal );
		Vector4Set( flock->maxs, maxVal, maxVal, maxVal, maxVal );
	}

	flock->timeoutAt = timeoutOfParticlesLeft;
}

auto ParticleSystem::activateDelayedParticles( ParticleFlock *flock, int64_t currTime ) -> std::optional<int64_t> {
	int64_t timeoutOfDelayedParticles = 0;

	// We must keep the delayed particles chunk "right-aligned" within the particles buffer,
	// so the data of delayed particles does not overlap with the data of activated ones.

	// Check the overlap invariant
	assert( !flock->numDelayedParticles || flock->numActivatedParticles <= flock->delayedParticlesOffset );

	const unsigned endIndexInBuffer = flock->delayedParticlesOffset + flock->numDelayedParticles;
	for( unsigned indexInBuffer = flock->delayedParticlesOffset; indexInBuffer < endIndexInBuffer; ++indexInBuffer ) {
		Particle *__restrict p = flock->particles + indexInBuffer;
		bool disposeParticle = true;
		if( const int64_t timeoutAt = p->spawnTime + p->lifetime; timeoutAt > currTime ) [[likely]] {
			if( const int64_t activationAt = p->spawnTime + p->activationDelay; activationAt <= currTime ) {
				//Com_Printf( "Activating: Copying delayed particle %d to %d\n", indexInBuffer, flock->numActivatedParticles );
				// Append it to the array of activated particles
				flock->particles[flock->numActivatedParticles++] = *p;
			} else {
				// The particle is kept delayed.
				// Calculate the result timeout, so it affects timeout of the entire flock.
				// (Newly activated particles contribute to the flock timeout in their simulation subroutines).
				timeoutOfDelayedParticles = wsw::max( timeoutAt, timeoutOfDelayedParticles );
				disposeParticle = false;
			}
		}
		if( disposeParticle ) {
			// Overwrite the particle data at the current index by the left-most one
			// This condition also prevents overwriting the back of the activated particles span
			// (that is currently harmless, but it's better to keep it correct)
			if( indexInBuffer != flock->delayedParticlesOffset ) {
				flock->particles[indexInBuffer] = flock->particles[flock->delayedParticlesOffset];
			}
			// Advance the left boundary
			flock->delayedParticlesOffset++;
			flock->numDelayedParticles--;
		}
	}

	assert( !flock->numDelayedParticles || flock->numActivatedParticles <= flock->delayedParticlesOffset );

	return timeoutOfDelayedParticles ? std::optional( timeoutOfDelayedParticles ) : std::nullopt;
}
