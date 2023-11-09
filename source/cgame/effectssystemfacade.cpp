/*
Copyright (C) 1997-2001 Id Software, Inc.
Copyright (C) 2022 Chasseur de bots

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

#include "effectssystemfacade.h"

#include "cg_local.h"
#include "../common/common.h"
#include "../client/snd_public.h"
#include "../common/configvars.h"

void EffectsSystemFacade::startSound( sfx_s *sfx, const float *origin, float attenuation ) {
	SoundSystem::instance()->startFixedSound( sfx, origin, CHAN_AUTO, v_volumeEffects.get(), attenuation );
}

void EffectsSystemFacade::startRelativeSound( sfx_s *sfx, int entNum, float attenuation ) {
	SoundSystem::instance()->startRelativeSound( sfx, entNum, CHAN_AUTO, v_volumeEffects.get(), attenuation );
}

void EffectsSystemFacade::spawnRocketExplosionEffect( const float *origin, const float *dir, int mode ) {
	sfx_s *sfx = mode == FIRE_MODE_STRONG ? cgs.media.sfxRocketLauncherStrongHit : cgs.media.sfxRocketLauncherWeakHit;
	const bool addSoundLfe = v_heavyRocketExplosions.get();
	spawnExplosionEffect( origin, dir, sfx, 64.0f, addSoundLfe );
}

void EffectsSystemFacade::spawnGrenadeExplosionEffect( const float *origin, const float *dir, int mode ) {
	sfx_s *sfx = mode == FIRE_MODE_STRONG ? cgs.media.sfxGrenadeStrongExplosion : cgs.media.sfxGrenadeWeakExplosion;
	const bool addSoundLfe = v_heavyGrenadeExplosions.get();
	spawnExplosionEffect( origin, dir, sfx, 64.0f, addSoundLfe );
}

void EffectsSystemFacade::spawnGenericExplosionEffect( const float *origin, int mode, float radius ) {
	const vec3_t dir { 0.0f, 0.0f, 1.0f };
	spawnExplosionEffect( origin, dir, cgs.media.sfxRocketLauncherStrongHit, radius, true );
}

// TODO: std::optional<std::pair<Vec3, Vec3>>
[[nodiscard]]
static bool findWaterHitPointBetweenTwoPoints( const float *checkFromPoint, const float *pointInWater,
											   float *waterHitPoint, float *traceDir = nullptr ) {
	const float squareDistance = DistanceSquared( checkFromPoint, pointInWater );
	if( squareDistance < wsw::square( 1.0f ) ) {
		return false;
	}

	vec3_t tmpTraceDir;
	VectorSubtract( pointInWater, checkFromPoint, tmpTraceDir );
	VectorNormalizeFast( tmpTraceDir );

	// Check if there's a solid surface between pointInWater and checkFromPoint.
	// If there is one, continue checking from a point slightly closer to pointInWater than the check hit point.

	trace_t trace;
	vec3_t airPoint;
	VectorCopy( checkFromPoint, airPoint );
	CG_Trace( &trace, pointInWater, vec3_origin, vec3_origin, airPoint, 0, MASK_SOLID );
	if( trace.fraction != 1.0f && ( trace.contents & CONTENTS_SOLID ) ) {
		VectorMA( trace.endpos, 0.1f, tmpTraceDir, airPoint );
	}

	CG_Trace( &trace, airPoint, vec3_origin, vec3_origin, pointInWater, 0, MASK_SOLID | MASK_WATER );
	// Make sure we didn't start in solid
	if( trace.fraction != 1.0f && !trace.startsolid ) {
		if( ( trace.contents & MASK_WATER ) && !( trace.contents & MASK_SOLID ) ) {
			// Make sure we can retrace after the hit point
			if( DistanceSquared( trace.endpos, pointInWater ) > wsw::square( 1.0f ) ) {
				vec3_t tmpHitPoint;
				VectorCopy( trace.endpos, tmpHitPoint );

				vec3_t retraceStart;
				VectorMA( trace.endpos, 0.1f, tmpTraceDir, retraceStart );

				CG_Trace( &trace, retraceStart, vec3_origin, vec3_origin, pointInWater, 0, MASK_SOLID );
				if( trace.fraction == 1.0f && !trace.startsolid ) {
					if( waterHitPoint ) {
						VectorCopy( tmpHitPoint, waterHitPoint );
					}
					if( traceDir ) {
						VectorCopy( tmpTraceDir, traceDir );
					}
					return true;
				}
			}
		}
	}

	return false;
}

static void addUnderwaterSplashImpactsForKnownWaterZ( const float *fireOrigin, float radius, float waterZ,
													  wsw::RandomGenerator *rng, int impactContents,
													  wsw::StaticVector<LiquidImpact, 12> *impacts ) {
	assert( waterZ - fireOrigin[2] > 0.0f && "Make sure directions are normalizable" );
	assert( radius > 0.0f );

	// TODO: Build a shape list once and clip against it in the loop

	// This number of attempts is sufficient in this case
	const unsigned maxTraceAttempts = ( 3 * impacts->capacity() ) / 2;
	for( unsigned i = 0; i < maxTraceAttempts; ++i ) {
		// Sample a random point on the sphere projection onto the water plane (let it actually be a donut)
		const float phi    = rng->nextFloat( 0.0f, 2.0f * (float)M_PI );
		const float r      = rng->nextFloat( 0.1f * radius, 1.0f * radius );
		const float sinPhi = std::sin( phi ), cosPhi = std::cos( phi );
		const vec3_t startPoint { fireOrigin[0] + r * sinPhi, fireOrigin[1] + r * cosPhi, waterZ + 1.0f };

		trace_t trace;
		CG_Trace( &trace, startPoint, vec3_origin, vec3_origin, fireOrigin, 0, MASK_SOLID | MASK_WATER );
		if( trace.fraction != 1.0f && ( trace.contents & CONTENTS_WATER ) ) {
			vec3_t burstDir;
			VectorSubtract( startPoint, fireOrigin, burstDir );
			// Condense it towards the Z axis
			Vector2Scale( burstDir, -0.67f, burstDir );
			VectorNormalizeFast( burstDir );
			assert( burstDir[2] > 0.0f );

			impacts->emplace_back( LiquidImpact {
				.origin   = { trace.endpos[0], trace.endpos[1], trace.endpos[2] },
				.burstDir = { burstDir[0], burstDir[1], burstDir[2] },
				.contents = impactContents,
			});

			// TODO: Vary limit of spawned impacts by explosion depth
			if( impacts->full() ) [[unlikely]] {
				break;
			}
		}
	}
}

static void addUnderwaterSplashImpactsForUnknownWaterZ( const float *fireOrigin, float radius, float maxZ,
														wsw::RandomGenerator *rng, int impactContents,
														wsw::StaticVector<LiquidImpact, 12> *impacts ) {
	assert( maxZ - fireOrigin[2] > 0.0f && "Make sure directions are normalizable" );

	// TODO: Build a shape list once and clip against it in the loop
	// TODO: Use a grid propagation instead?

	const unsigned maxTraceAttempts = 6 * impacts->capacity();
	for( unsigned i = 0; i < maxTraceAttempts; ++i ) {
		// Sample a random point on the sphere projection onto the water plane.
		// Let it actually be a donut with an outer radius larger than the explosion radius
		// (otherwise this approach fails way too much).
		const float phi    = rng->nextFloat( 0.0f, 2.0f * (float)M_PI );
		const float r      = rng->nextFloat( 0.5f * radius, 1.5f * radius );
		const float sinPhi = std::sin( phi ), cosPhi = std::cos( phi );

		vec3_t waterHitPoint, traceDir;
		const vec3_t startPoint { fireOrigin[0] + r * sinPhi, fireOrigin[1] + r * cosPhi, maxZ + 1.0f };
		if( findWaterHitPointBetweenTwoPoints( startPoint, fireOrigin, waterHitPoint, traceDir ) ) {
			vec3_t burstDir;
			VectorNegate( traceDir, burstDir );
			Vector2Scale( burstDir, -0.75f, burstDir );
			VectorNormalizeFast( burstDir );
			assert( burstDir[2] > 0.0f );

			impacts->emplace_back( LiquidImpact {
				.origin   = { waterHitPoint[0], waterHitPoint[1], waterHitPoint[2] },
				.burstDir = { burstDir[0], burstDir[1], burstDir[2] },
				.contents = impactContents,
			});

			if( impacts->full() ) [[unlikely]] {
				break;
			}
		}
	}
}

static void makeRegularExplosionImpacts( const float *fireOrigin, float radius, wsw::RandomGenerator *rng,
										 wsw::StaticVector<SolidImpact, 12> *solidImpacts,
										 wsw::StaticVector<LiquidImpact, 12> *waterImpacts ) {
	const unsigned numTraceAttempts = 3 * solidImpacts->capacity();
	for( unsigned i = 0; i < numTraceAttempts; ++i ) {
		trace_t trace;
		vec3_t traceEnd;
		// Don't check for similarity with the last direction.
		// Otherwise, columns/flat walls do not produce noticeable effects.
		const float *const traceDir = kPredefinedDirs[rng->nextBounded( std::size( kPredefinedDirs ) )];
		// TODO: Make the trace depth it match the visual radius (with is not really equal to `radius`)
		VectorMA( fireOrigin, 0.5f * radius, traceDir, traceEnd );
		// TODO: Build a shape list and clip against it
		CG_Trace( &trace, fireOrigin, vec3_origin, vec3_origin, traceEnd, 0, MASK_SOLID | MASK_WATER );
		if( trace.fraction != 1.0f && !trace.startsolid && !trace.allsolid ) {
			if( !( trace.surfFlags & ( SURF_FLESH | SURF_NOIMPACT ) ) ) {
				bool addedOnThisStep = false;
				if( trace.contents & MASK_WATER ) {
					if( !waterImpacts->full() ) {
						// This condition produces better-looking results so far
						if( const float absDirZ = std::fabs( traceDir[2] ); absDirZ > 0.1f && absDirZ < 0.7f ) {
							vec3_t burstDir;
							VectorReflect( traceDir, trace.plane.normal, 0.0f, burstDir );
							waterImpacts->emplace_back( LiquidImpact {
								.origin   = { trace.endpos[0], trace.endpos[1], trace.endpos[2] },
								.burstDir = { burstDir[0], burstDir[1], burstDir[2] },
								.contents = trace.contents,
							});
							addedOnThisStep = true;
						}
					}
				} else {
					if( !solidImpacts->full() ) {
						const auto surfFlags = getSurfFlagsForImpact( trace, traceDir );
						const auto material  = decodeSurfImpactMaterial( (unsigned)surfFlags );
						// Make sure it adds something to visuals in the desired way.
						if( material != SurfImpactMaterial::Unknown && material != SurfImpactMaterial::Metal ) {
							solidImpacts->emplace_back( SolidImpact {
								.origin      = { trace.endpos[0], trace.endpos[1], trace.endpos[2] },
								.normal      = { trace.plane.normal[0], trace.plane.normal[1], trace.plane.normal[2] },
								.incidentDir = { traceDir[0], traceDir[1], traceDir[2] },
								.surfFlags   = surfFlags,
							});
							addedOnThisStep = true;
						}
					}
				}
				if( addedOnThisStep ) {
					if( solidImpacts->full() && waterImpacts->full() ) {
						break;
					}
				}
			}
		}
	}
}


static const RgbaLifespan kExplosionSparksColors[3] {
	{
		.initial  = { 1.0f, 1.0f, 1.0f, 0.0f },
		.fadedIn  = { 1.0f, 0.6f, 0.3f, 1.0f },
		.fadedOut = { 0.5f, 0.5f, 0.5f, 0.3f },
	},
	{
		.initial  = { 1.0f, 1.0f, 1.0f, 0.0f },
		.fadedIn  = { 1.0f, 0.8f, 0.4f, 1.0f },
		.fadedOut = { 0.5f, 0.5f, 0.5f, 0.3f },
	},
	{
		.initial  = { 1.0f, 1.0f, 1.0f, 0.0f },
		.fadedIn  = { 1.0f, 0.7f, 0.5f, 1.0f },
		.fadedOut = { 0.5f, 0.5f, 0.5f, 0.3f },
	},
};

static const LightLifespan kExplosionSparksFlareProps[1] {
	{
		.colorLifespan = {
			.initial  = { 1.0f, 1.0f, 1.0f },
			.fadedIn  = { 1.0f, 0.8f, 0.4f },
			.fadedOut = { 1.0f, 0.6f, 0.3f }
		},
		.radiusLifespan = { .fadedIn = 10.0f },
	}
};

static const RgbaLifespan kExplosionSmokeColors[3] {
	{
		.initial  = { 0.5f, 0.5f, 0.5f, 0.0f },
		.fadedIn  = { 0.5f, 0.5f, 0.5f, 0.2f },
		.fadedOut = { 0.9f, 0.9f, 0.9f, 0.0f },
		.finishFadingInAtLifetimeFrac = 0.67f, .startFadingOutAtLifetimeFrac = 0.76f,
	},
	{
		.initial  = { 0.5f, 0.5f, 0.5f, 0.0f },
		.fadedIn  = { 0.6f, 0.6f, 0.6f, 0.2f },
		.fadedOut = { 0.9f, 0.9f, 0.9f, 0.0f },
		.finishFadingInAtLifetimeFrac = 0.67f, .startFadingOutAtLifetimeFrac = 0.76f,
	},
	{
		.initial  = { 0.5f, 0.5f, 0.5f, 0.0f },
		.fadedIn  = { 0.7f, 0.7f, 0.7f, 0.2f },
		.fadedOut = { 0.9f, 0.9f, 0.9f, 0.0f },
		.finishFadingInAtLifetimeFrac = 0.67f, .startFadingOutAtLifetimeFrac = 0.76f,
	},
};

static const Particle::AppearanceRules kExplosionSmokeAppearanceRules {
	.colors        = kExplosionSmokeColors,
	.geometryRules = Particle::SpriteRules { .radius = { .mean = 9.0f, .spread = 5.0f } },
};

static const EllipsoidalFlockParams kExplosionSmokeFlockParams {
	.stretchScale = 1.25f,
	.gravity      = -45.0f,
	.restitution  = 0.33f,
	.speed        = { .min = 35.0f, .max = 55.0f },
	.shiftSpeed   = { .min = 60.0f, .max = 65.0f },
	.percentage   = { .min = 0.7f, .max = 0.9f },
	.timeout      = { .min = 1200, .max = 1750 },
};

void EffectsSystemFacade::spawnExplosionEffect( const float *origin, const float *dir, sfx_s *sfx,
												float radius, bool addSoundLfe ) {
	vec3_t fireOrigin, almostExactOrigin;
	VectorMA( origin, 8.0f, dir, fireOrigin );
	VectorAdd( origin, dir, almostExactOrigin );

	wsw::StaticVector<SolidImpact, 12> solidImpacts;
	wsw::StaticVector<LiquidImpact, 12> waterImpacts;

	trace_t fireOriginTrace;
	CG_Trace( &fireOriginTrace, origin, vec3_origin, vec3_origin, fireOrigin, 0, MASK_SOLID );
	if( fireOriginTrace.fraction != 1.0f ) {
		VectorMA( origin, 0.5f * fireOriginTrace.fraction, dir, fireOrigin );
		if( DistanceSquared( fireOriginTrace.endpos, origin ) < wsw::square( 1.0f ) ) {
			VectorAvg( origin, fireOriginTrace.endpos, almostExactOrigin );
		}
	}

	std::optional<int> liquidContentsAtFireOrigin;
	if( v_explosionSmoke.get() || v_particles.get() ) {
		if( const int contents = CG_PointContents( fireOrigin ); contents & MASK_WATER ) {
			liquidContentsAtFireOrigin = contents;
		}
	}

	vec3_t tmpSmokeOrigin;
	const float *smokeOrigin = nullptr;
	if( v_explosionSmoke.get() || v_particles.get() ) {
		if( liquidContentsAtFireOrigin ) {
			VectorCopy( fireOrigin, tmpSmokeOrigin );
			tmpSmokeOrigin[2] += radius;

			if( !( CG_PointContents( tmpSmokeOrigin ) & MASK_WATER ) ) {
				vec3_t waterHitPoint;
				if( findWaterHitPointBetweenTwoPoints( tmpSmokeOrigin, fireOrigin, waterHitPoint ) ) {
					if( waterHitPoint[2] - fireOrigin[2] > 1.0f ) {
						if( v_explosionSmoke.get() ) {
							VectorCopy( waterHitPoint, tmpSmokeOrigin );
							tmpSmokeOrigin[2] += 1.0f;
							smokeOrigin = tmpSmokeOrigin;
						}

						waterImpacts.emplace_back( LiquidImpact {
							.origin   = { waterHitPoint[0], waterHitPoint[1], waterHitPoint[2] },
							.burstDir = { 0.0f, 0.0f, +1.0f },
							.contents = *liquidContentsAtFireOrigin,
						});

						const float waterZ = waterHitPoint[2];
						addUnderwaterSplashImpactsForKnownWaterZ( fireOrigin, radius, waterZ, &m_rng,
																  *liquidContentsAtFireOrigin, &waterImpacts );
					} else if( radius > 2.0f ) {
						// Generate impacts from a point above the fire origin but within the fire radius
						const vec3_t shiftedFireOrigin { fireOrigin[0], fireOrigin[1], fireOrigin[2] + 0.75f * radius };
						makeRegularExplosionImpacts( shiftedFireOrigin, radius, &m_rng, &solidImpacts, &waterImpacts );

						if( v_explosionSmoke.get() ) {
							VectorCopy( shiftedFireOrigin, tmpSmokeOrigin );
							smokeOrigin = tmpSmokeOrigin;
						}
					}
				} else {
					const float maxZ = tmpSmokeOrigin[2];
					addUnderwaterSplashImpactsForUnknownWaterZ( fireOrigin, radius, maxZ, &m_rng,
																*liquidContentsAtFireOrigin, &waterImpacts );
				}
			}
		} else {
			if( v_explosionSmoke.get() ) {
				smokeOrigin = fireOrigin;
			}

			makeRegularExplosionImpacts( fireOrigin, radius, &m_rng, &solidImpacts, &waterImpacts );
		}
	}

	startSound( sfx, almostExactOrigin, ATTN_DISTANT );

	if( addSoundLfe ) {
		startSound( cgs.media.sfxExplosionLfe, almostExactOrigin, ATTN_NORM );
	}

	if( v_particles.get() && !liquidContentsAtFireOrigin ) {
		Particle::AppearanceRules appearanceRules {
			.materials     = cgs.media.shaderExplosionSpriteParticle.getAddressOfHandle(),
			.colors        = kExplosionSparksColors,
			.flareProps    = Particle::FlareProps {
				.lightProps                  = kExplosionSparksFlareProps,
				.alphaScale                  = 0.08f,
				.particleFrameAffinityModulo = 4,
			},
			.geometryRules = Particle::SpriteRules { .radius = { .mean = 1.25f, .spread = 0.25f } },
		};

		EllipsoidalFlockParams flockParams {
			.origin        = { origin[0], origin[1], origin[2] },
			.offset        = { dir[0], dir[1], dir[2] },
			.gravity       = 0.25f * GRAVITY,
			.drag          = 0.025f,
			.restitution   = 0.33f,
			.speed         = { .min = 150.0f, .max = 400.0f },
			.shiftSpeed    = { .min = 100.0f, .max = 200.0f },
			.percentage    = { .min = 0.5f, .max = 0.8f },
			.timeout       = { .min = 400, .max = 750 },
		};

		cg.particleSystem.addMediumParticleFlock( appearanceRules, flockParams );

		appearanceRules.materials = cgs.media.shaderExplosionSpikeParticle.getAddressOfHandle();

		appearanceRules.geometryRules = Particle::SparkRules {
			.length        = { .mean = 25.0f, .spread = 7.5f },
			.width         = { .mean = 4.0f, .spread = 1.0f },
			.sizeBehaviour = Particle::Shrinking,
		};

		// Suppress the flare for spikes
		appearanceRules.flareProps = std::nullopt;

		flockParams.speed      = { .min = 550, .max = 650 };
		flockParams.drag       = 0.01f;
		flockParams.timeout    = { .min = 100, .max = 150 };
		flockParams.percentage = { .min = 0.5f, .max = 1.0f };
		flockParams.shiftSpeed = { .min = 50.0f, .max = 65.0f };

		cg.particleSystem.addMediumParticleFlock( appearanceRules, flockParams );

		if( v_explosionSmoke.get() ) {
			flockParams.speed = { .min = 125.0f, .max = 175.0f };
		} else {
			flockParams.speed = { .min = 150.0f, .max = 225.0f };
		}

		flockParams.angularVelocity = { .min = 360.0f, .max = 2 * 360.0f };
		flockParams.timeout         = { .min = 350, .max = 450 };
		flockParams.percentage      = { .min = 1.0f, .max = 1.0f };

		cg.particleSystem.addMediumParticleFlock( appearanceRules, flockParams );
	}

	if( smokeOrigin ) {
		EllipsoidalFlockParams flockParams( kExplosionSmokeFlockParams );
		VectorCopy( smokeOrigin, flockParams.origin );
		Particle::AppearanceRules appearanceRules( kExplosionSmokeAppearanceRules );
		appearanceRules.materials = cgs.media.shaderSmokeHullHardParticle.getAddressOfHandle();
		m_transientEffectsSystem.addDelayedParticleEffect( 300, TransientEffectsSystem::ParticleFlockBin::Large,
														   flockParams, appearanceRules );
	}

	m_transientEffectsSystem.spawnExplosionHulls( fireOrigin, smokeOrigin );

	spawnMultipleExplosionImpactEffects( solidImpacts );
	spawnMultipleLiquidImpactEffects( waterImpacts, 1.0f, { 0.7f, 0.9f }, std::make_pair( 0u, 100u ) );
}

void EffectsSystemFacade::spawnShockwaveExplosionEffect( const float *origin, const float *dir, int mode ) {
}

static const RgbaLifespan kPlasmaParticlesColors[1] {
	{
		.initial  = { 0.0f, 1.0f, 0.0f, 0.0f },
		.fadedIn  = { 0.3f, 1.0f, 0.5f, 1.0f },
		.fadedOut = { 0.7f, 1.0f, 0.7f, 0.0f },
	}
};

static const LightLifespan kPlasmaParticlesFlareProps[1] {
	{
		.colorLifespan = {
			.initial  = { 0.0f, 1.0f, 0.0f },
			.fadedIn  = { 0.3f, 1.0f, 0.5f },
			.fadedOut = { 0.7f, 1.0f, 0.7f },
		},
		.radiusLifespan = { .fadedIn = 10.0f },
	}
};

void EffectsSystemFacade::spawnPlasmaExplosionEffect( const float *origin, const float *impactNormal, int mode ) {
	const vec3_t soundOrigin { origin[0] + impactNormal[0], origin[1] + impactNormal[1], origin[2] + impactNormal[2] };
	sfx_s *sfx = ( mode == FIRE_MODE_STRONG ) ? cgs.media.sfxPlasmaStrongHit : cgs.media.sfxPlasmaWeakHit;
	startSound( sfx, soundOrigin, ATTN_IDLE );

	if( v_particles.get() ) {
		EllipsoidalFlockParams flockParams {
			.origin     = { origin[0], origin[1], origin[2] },
			.offset     = { impactNormal[0], impactNormal[1], impactNormal[2] },
			.gravity    = 250.0f,
			.percentage = { .min = 0.5f, .max = 0.8f },
			.timeout    = { .min = 125, .max = 150 },
		};
		Particle::AppearanceRules appearanceRules {
			.materials     = cgs.media.shaderPlasmaImpactParticle.getAddressOfHandle(),
			.colors        = kPlasmaParticlesColors,
			.flareProps    = Particle::FlareProps {
				.lightProps                  = kPlasmaParticlesFlareProps,
				.alphaScale                  = 0.08f,
				.particleFrameAffinityModulo = 2,
			},
			.geometryRules = Particle::SpriteRules {
				.radius        = { .mean = 1.5f, .spread = 0.5f },
				.sizeBehaviour = Particle::ExpandingAndShrinking
			},
		};
		cg.particleSystem.addMediumParticleFlock( appearanceRules, flockParams );
	}

	m_transientEffectsSystem.spawnPlasmaImpactEffect( origin, impactNormal );
}

void EffectsSystemFacade::clear() {
	m_transientEffectsSystem.clear();
	m_trackedEffectsSystem.clear();
}

void EffectsSystemFacade::simulateFrameAndSubmit( int64_t currTime, DrawSceneRequest *request ) {
	m_transientEffectsSystem.simulateFrameAndSubmit( currTime, request );
	m_trackedEffectsSystem.simulateFrameAndSubmit( currTime, request );
}

void EffectsSystemFacade::spawnGrenadeBounceEffect( int entNum, int mode ) {
	assert( mode == FIRE_MODE_STRONG || mode == FIRE_MODE_WEAK );
	sfx_s *sound = nullptr;
	if( mode == FIRE_MODE_STRONG ) {
		sound = cgs.media.sfxGrenadeStrongBounce[m_rng.nextBounded( 2 )];
	} else {
		sound = cgs.media.sfxGrenadeWeakBounce[m_rng.nextBounded( 2 )];
	}
	startRelativeSound( sound, entNum, ATTN_IDLE );
}

static const vec4_t kBloodColors[] {
	{ 1.0f, 0.3f, 0.7f, 1.0f },
	{ 1.0f, 0.6f, 0.3f, 1.0f },
	{ 0.3f, 1.0f, 0.5f, 1.0f },
	{ 0.3f, 0.7f, 1.0f, 1.0f },
	{ 1.0f, 1.0f, 1.0f, 1.0f },
};

void EffectsSystemFacade::spawnPlayerHitEffect( const float *origin, const float *dir, int damage ) {
	if( const int bloodStyle = v_bloodStyle.get() ) {
		const int indexForPalette = wsw::clamp<int>( v_bloodPalette.get(), 0, std::size( kBloodColors ) - 1 );
		const int baseTime        = wsw::clamp<int>( v_bloodTime.get(), 200, 400 );
		const float *effectColor  = kBloodColors[indexForPalette];

		if( bloodStyle < 0 ) {
			unsigned damageLevel;
			if( damage <= kPain1UpperInclusiveBound ) {
				damageLevel = 1;
			} else if( damage <= kPain2UpperInclusiveBound ) {
				damageLevel = 2;
			} else if( damage <= kPain3UpperInclusiveBound ) {
				damageLevel = 3;
			} else {
				damageLevel = 4;
			}
			m_transientEffectsSystem.spawnBleedingVolumeEffect( origin, dir, damageLevel, effectColor, (unsigned)baseTime );
		} else {
			unsigned numParts;
			if( damage <= kPain1UpperInclusiveBound ) {
				numParts = 1;
			} else if( damage <= kPain2UpperInclusiveBound ) {
				numParts = 3;
			} else if( damage <= kPain3UpperInclusiveBound ) {
				numParts = 4;
			} else {
				numParts = 5;
			}
			if( numParts == 1 ) {
				m_transientEffectsSystem.spawnBleedingVolumeEffect( origin, dir, 1, effectColor, (unsigned)baseTime, 1.0f );
			} else {
				m_transientEffectsSystem.spawnBleedingVolumeEffect( origin, dir, 1, effectColor, (unsigned)baseTime, 1.1f );

				// TODO: Avoid hardcoding it
				const float offset = 18.0f;
				vec3_t usedOrigins[6];
				// Don't pick a dir that is close to the last one
				const float *lastOffsetDir = kPredefinedDirs[0];
				for( unsigned partNum = 1; partNum < numParts; ++partNum ) {
					bool didPickPartOrigin = false;
					vec3_t newPartOrigin;

					// Protect from infinite looping
					for( unsigned attemptNum = 0; attemptNum < 16; ++attemptNum ) {
						const float *offsetDir = kPredefinedDirs[m_rng.nextBounded( std::size( kPredefinedDirs ) )];
						if( DotProduct( offsetDir, lastOffsetDir ) > 0.7f ) {
							continue;
						}

						VectorMA( origin, offset, offsetDir, newPartOrigin );

						bool isOccupedByOtherPart = false;
						for( unsigned spawnedPartNum = 1; spawnedPartNum < partNum; ++spawnedPartNum ) {
							const float *spawnedPartOrigin = usedOrigins[spawnedPartNum - 1];
							if( DistanceSquared( spawnedPartOrigin, newPartOrigin ) < wsw::square( 0.75f * offset ) ) {
								isOccupedByOtherPart = true;
								break;
							}
						}
						if( !isOccupedByOtherPart ) {
							lastOffsetDir     = offsetDir;
							didPickPartOrigin = true;
							break;
						}
					}

					// Interrupt at this
					if( !didPickPartOrigin ) {
						break;
					}

					const float scale = m_rng.nextFloat( 0.5f, 0.9f );
					VectorCopy( newPartOrigin, usedOrigins[partNum - 1] );
					m_transientEffectsSystem.spawnBleedingVolumeEffect( newPartOrigin, dir, 1, effectColor, baseTime, scale );
				}
			}
		}
	}

	m_transientEffectsSystem.spawnCartoonHitEffect( origin, dir, damage );
}

static ParticleColorsForTeamHolder electroboltParticleColorsHolder {
	.defaultColors = {
		.initial  = { 1.0f, 1.0f, 1.0f, 1.0f },
		.fadedIn  = { 0.7f, 0.7f, 1.0f, 1.0f },
		.fadedOut = { 0.1f, 0.1f, 1.0f, 0.0f },
		.finishFadingInAtLifetimeFrac = 0.05f,
		.startFadingOutAtLifetimeFrac = 0.50f,
	}
};

[[nodiscard]]
auto getTeamForOwner( int ownerNum ) -> int {
	if( ownerNum && ownerNum < gs.maxclients + 1 ) [[likely]] {
		return cg_entities[ownerNum].current.team;
	}
	return TEAM_SPECTATOR;
}

void EffectsSystemFacade::spawnElectroboltHitEffect( const float *origin, const float *impactNormal,
													 const float *impactDir, bool spawnDecal, int ownerNum ) {
	assert( std::fabs( VectorLengthFast( impactNormal ) - 1.0f ) < 0.1f );
	assert( std::fabs( VectorLengthFast( impactDir ) - 1.0f ) < 0.1f );
	assert( DotProduct( impactDir, impactNormal ) <= 0.0f );

	const int team = getTeamForOwner( ownerNum );

	vec4_t teamColor, decalColor, energyColor;
	const bool useTeamColor = getElectroboltTeamColor( team, teamColor );
	if( useTeamColor ) {
		Vector4Copy( teamColor, decalColor );
		Vector4Copy( teamColor, energyColor );
	} else {
		Vector4Copy( colorWhite, decalColor );
		Vector4Set( energyColor, 0.3f, 0.6f, 1.0f, 1.0f );
	}

	if( v_particles.get() ) {
		vec3_t invImpactDir, coneDir;
		VectorNegate( impactDir, invImpactDir );
		VectorReflect( invImpactDir, impactNormal, 0.0f, coneDir );

		const RgbaLifespan *singleColorAddress;
		ParticleColorsForTeamHolder *colorsHolder = &::electroboltParticleColorsHolder;
		if( useTeamColor ) {
			singleColorAddress = colorsHolder->getColorsForTeam( team, teamColor );
		} else {
			singleColorAddress = &colorsHolder->defaultColors;
		}

		const ConicalFlockParams flockParams {
			.origin     = { origin[0], origin[1], origin[2] },
			.offset     = { impactNormal[0], impactNormal[1], impactNormal[2] },
			.dir        = { coneDir[0], coneDir[1], coneDir[2] },
			.gravity    = GRAVITY,
			.angle      = 45.0f,
			.speed      = { .min = 500.0f, .max = 950.0f },
			.percentage = { .min = 0.33f, .max = 0.67f },
			.timeout    = { .min = 100, .max = 300 },
		};

		const Particle::AppearanceRules appearanceRules {
			.materials     = cgs.media.shaderElectroImpactParticle.getAddressOfHandle(),
			.colors        = { singleColorAddress, 1 },
			.geometryRules = Particle::SparkRules {
				.length = { .mean = 12.5f, .spread = 2.5f },
				.width  = { .mean = 2.0f, .spread = 1.0f },
			}
		};

		cg.particleSystem.addMediumParticleFlock( appearanceRules, flockParams );
	}

	const vec3_t soundOrigin { origin[0] + impactNormal[0], origin[1] + impactNormal[1], origin[2] + impactNormal[2] };
	startSound( cgs.media.sfxElectroboltHit, soundOrigin, ATTN_STATIC );

	m_transientEffectsSystem.spawnElectroboltHitEffect( origin, impactNormal, decalColor, energyColor, spawnDecal );
}

static ParticleColorsForTeamHolder instagunParticleColorsHolder {
	.defaultColors = {
		.initial  = { 1.0f, 1.0f, 1.0f, 1.0f },
		.fadedIn  = { 1.0f, 0.0f, 1.0f, 0.5f },
		.fadedOut = { 0.0f, 0.0f, 1.0f, 0.0f },
		.finishFadingInAtLifetimeFrac = 0.05f,
		.startFadingOutAtLifetimeFrac = 0.75f,
	}
};

void EffectsSystemFacade::spawnInstagunHitEffect( const float *origin, const float *impactNormal,
												  const float *impactDir, bool spawnDecal, int ownerNum ) {
	assert( std::fabs( VectorLengthFast( impactNormal ) - 1.0f ) < 0.1f );
	assert( std::fabs( VectorLengthFast( impactDir ) - 1.0f ) < 0.1f );
	assert( DotProduct( impactDir, impactNormal ) <= 0.0f );

	vec4_t teamColor, decalColor, energyColor;
	const int team = getTeamForOwner( ownerNum );
	const bool useTeamColor = getInstagunTeamColor( team, teamColor );
	if( useTeamColor ) {
		Vector4Copy( teamColor, decalColor );
		Vector4Copy( teamColor, energyColor );
	} else {
		Vector4Set( decalColor, 1.0f, 0.0f, 0.4f, 1.0f );
		Vector4Set( energyColor, 1.0f, 0.0f, 0.4f, 1.0f );
	}

	if( v_particles.get() ) {
		vec3_t invImpactDir, coneDir;
		VectorNegate( impactDir, invImpactDir );
		VectorReflect( invImpactDir, impactNormal, 0.0f, coneDir );

		const RgbaLifespan *singleColorAddress;
		ParticleColorsForTeamHolder *colorsHolder = &::instagunParticleColorsHolder;
		if( useTeamColor ) {
			singleColorAddress = colorsHolder->getColorsForTeam( team, teamColor );
		} else {
			singleColorAddress = &colorsHolder->defaultColors;
		}

		const ConicalFlockParams flockParams {
			.origin     = { origin[0], origin[1], origin[2] },
			.offset     = { impactNormal[0], impactNormal[1], impactNormal[2] },
			.dir        = { coneDir[0], coneDir[1], coneDir[2] },
			.gravity    = GRAVITY,
			.angle      = 45.0f,
			.speed      = { .min = 750.0f, .max = 950.0f },
			.percentage = { .min = 0.5f, .max = 1.0f },
			.timeout    = { .min = 150, .max = 225 },
		};

		const Particle::AppearanceRules appearanceRules {
			.materials     = cgs.media.shaderInstaImpactParticle.getAddressOfHandle(),
			.colors        = { singleColorAddress, 1 },
			.geometryRules = Particle::SparkRules {
				.length = { .mean = 10.0f, .spread = 2.5f },
				.width  = { .mean = 1.5f, .spread = 0.5f },
			}
		};

		cg.particleSystem.addSmallParticleFlock( appearanceRules, flockParams );
	}

	// TODO: Don't we need an IG-specific sound
	const vec3_t soundOrigin { origin[0] + impactNormal[0], origin[1] + impactNormal[1], origin[2] + impactNormal[2] };
	startSound( cgs.media.sfxElectroboltHit, soundOrigin, ATTN_STATIC );

	m_transientEffectsSystem.spawnInstagunHitEffect( origin, impactNormal, decalColor, energyColor, spawnDecal );
}

static const RgbaLifespan kGunbladeHitColors[1] {
	{
		.initial  = { 1.0f, 0.5f, 0.1f, 0.0f },
		.fadedIn  = { 1.0f, 1.0f, 1.0f, 1.0f },
		.fadedOut = { 0.5f, 0.5f, 0.5f, 0.5f },
	}
};

void EffectsSystemFacade::spawnGunbladeBladeHitEffect( const float *pos, const float *dir ) {
	// Find what are we hitting
	vec3_t local_pos, local_dir;
	VectorCopy( pos, local_pos );
	VectorNormalize2( dir, local_dir );
	vec3_t end;
	VectorMA( pos, -1.0, local_dir, end );

	trace_t trace;
	CG_Trace( &trace, local_pos, vec3_origin, vec3_origin, end, cg.view.POVent, MASK_SHOT );

	if( trace.fraction != 1.0 ) {
		bool isHittingFlesh = false;
		if( trace.surfFlags & SURF_FLESH ) {
			isHittingFlesh = true;
		} else if( const int entNum = trace.ent; entNum > 0 ) {
			if( const auto type = cg_entities[entNum].current.type; type == ET_PLAYER || type == ET_CORPSE ) {
				isHittingFlesh = true;
			}
		}

		if( isHittingFlesh ) {
			// TODO: Check sound origin
			startSound( cgs.media.sfxBladeFleshHit[m_rng.nextBounded( 3 )], pos, ATTN_NORM );
		} else {
			m_transientEffectsSystem.spawnGunbladeBladeImpactEffect( trace.endpos, trace.plane.normal );

			// TODO: Check sound origin
			startSound( cgs.media.sfxBladeWallHit[m_rng.nextBounded( 2 )], pos, ATTN_NORM );

			if( v_particles.get() ) {
				ConicalFlockParams flockParams {
					.origin = { pos[0], pos[1], pos[2] },
					.offset = { dir[0], dir[1], dir[2] },
					.dir    = { dir[0], dir[1], dir[2] },
					.angle  = 60
				};
				Particle::AppearanceRules appearanceRules {
					.materials     = cgs.media.shaderBladeImpactParticle.getAddressOfHandle(),
					.colors        = kGunbladeHitColors,
					.geometryRules = Particle::SparkRules {
						.length = { .mean = 4.0f, .spread = 1.0f },
						.width  = { .mean = 1.0f, .spread = 0.25f },
					},
				};
				cg.particleSystem.addMediumParticleFlock( appearanceRules, flockParams );
			}
		}
	}
}

static const RgbaLifespan kGunbladeBlastColors[3] {
	{
		.initial  = { 1.0f, 1.0f, 1.0f, 1.0f },
		.fadedIn  = { 1.0f, 0.8f, 0.5f, 0.7f },
		.fadedOut = { 0.5f, 0.3f, 0.1f, 0.0f }
	},
	{
		.initial  = { 1.0f, 1.0f, 1.0f, 1.0f },
		.fadedIn  = { 1.0f, 0.8f, 0.4f, 0.7f },
		.fadedOut = { 0.7f, 0.3f, 0.1f, 0.0f }
	},
	{
		.initial  = { 1.0f, 1.0f, 1.0f, 1.0f },
		.fadedIn  = { 1.0f, 0.7f, 0.4f, 0.7f },
		.fadedOut = { 0.9f, 0.3f, 0.1f, 0.0f }
	},
};

static const LightLifespan kGunbladeBlastFlareProps[1] {
	{
		.colorLifespan = {
			.initial  = { 1.0f, 0.8f, 0.5f },
			.fadedIn  = { 1.0f, 0.8f, 0.4f },
			.fadedOut = { 1.0f, 0.7f, 0.4f },
		},
		.radiusLifespan = { .fadedIn = 10.0f },
	}
};

void EffectsSystemFacade::spawnGunbladeBlastHitEffect( const float *origin, const float *dir ) {
	startSound( cgs.media.sfxGunbladeStrongHit[m_rng.nextBounded( 2 )], origin, ATTN_IDLE );

	if( v_particles.get() ) {
		EllipsoidalFlockParams flockParams {
			.origin     = { origin[0], origin[1], origin[2] },
			.offset     = { dir[0], dir[1], dir[2] },
			.gravity    = -50.0f,
			.speed      = { .min = 50, .max = 100 },
			.percentage = { .min = 1.0f, .max = 1.0f },
		};
		Particle::AppearanceRules appearanceRules {
			.materials     = cgs.media.shaderBlastImpactParticle.getAddressOfHandle(),
			.colors        = kGunbladeBlastColors,
			.flareProps    = Particle::FlareProps {
				.lightProps                  = kGunbladeBlastFlareProps,
				.alphaScale                  = 0.09f,
				.particleFrameAffinityModulo = 4,
			},
			.geometryRules = Particle::SpriteRules { .radius = { .mean = 1.50f, .spread = 0.25f } },
		};
		cg.particleSystem.addMediumParticleFlock( appearanceRules, flockParams );
	}

	m_transientEffectsSystem.spawnGunbladeBlastImpactEffect( origin, dir );
}

[[nodiscard]]
static auto makeRicochetFlockOrientation( const SolidImpact &impact, wsw::RandomGenerator *rng,
										  const std::pair<float, float> &angleCosineRange = { 0.30f, 0.95f } )
										  -> FlockOrientation {
	assert( std::fabs( VectorLengthFast( impact.incidentDir ) - 1.0f ) < 0.1f );
	assert( std::fabs( VectorLengthSquared( impact.normal ) - 1.0f ) < 0.1f );
	assert( DotProduct( impact.incidentDir, impact.normal ) <= 0 );

	vec3_t oppositeDir, flockDir;
	VectorNegate( impact.incidentDir, oppositeDir );
	VectorReflect( oppositeDir, impact.normal, 0.0f, flockDir );

	const float coneAngleCosine = Q_Sqrt( rng->nextFloat( angleCosineRange.first, angleCosineRange.second ) );
	addRandomRotationToDir( flockDir, rng, coneAngleCosine );

	return FlockOrientation {
		.origin = { impact.origin[0], impact.origin[1], impact.origin[2] },
		.offset = { impact.normal[0], impact.normal[1], impact.normal[2] },
		.dir    = { flockDir[0], flockDir[1], flockDir[2] }
	};
}

void addRandomRotationToDir( float *dir, wsw::RandomGenerator *rng, float minConeAngleCosine, float maxConeAngleCosine ) {
	const float coneAngleCosine = rng->nextFloat( minConeAngleCosine, maxConeAngleCosine );
	addRandomRotationToDir( dir, rng, coneAngleCosine );
}

void addRandomRotationToDir( float *dir, wsw::RandomGenerator *rng, float coneAngleCosine ) {
	assert( coneAngleCosine > 0.0f && coneAngleCosine < 1.0f );
	const float z     = coneAngleCosine;
	const float r     = Q_Sqrt( 1.0f - z * z );
	const float phi   = rng->nextFloat( 0.0f, 2.0f * (float)M_PI );
	const vec3_t newZDir { r * std::cos( phi ), r * std::sin( phi ), z };

	mat3_t transformMatrix;
	Matrix3_ForRotationOfDirs( &axis_identity[AXIS_UP], newZDir, transformMatrix );

	vec3_t result;
	Matrix3_TransformVector( transformMatrix, dir, result );
	// wtf?
	VectorNormalizeFast( result );
	VectorCopy( result, dir );
}

template <typename FlockParams>
static inline void assignUpShiftAndModifyBaseSpeed( FlockParams *flockParams, float upShiftScale,
													float minShiftSpeed, float maxShiftSpeed ) {
	assert( upShiftScale >= 0.0f && upShiftScale <= 1.0f );
	flockParams->shiftSpeed.min = minShiftSpeed * upShiftScale;
	flockParams->shiftSpeed.max = maxShiftSpeed * upShiftScale;
	const float baseSpeedScale = 1.0f + upShiftScale;
	// Apply the upper bound to avoid triggering an assertion on speed feasibility
	flockParams->speed.min = wsw::min( 999.9f, flockParams->speed.min * baseSpeedScale );
	flockParams->speed.max = wsw::min( 999.9f, flockParams->speed.max * baseSpeedScale );
}

static const RgbaLifespan kBulletRosetteSpikeColorLifespan {
	.initial  = { 1.0f, 1.0f, 1.0f, 1.0f },
	.fadedIn  = { 0.9f, 0.9f, 1.0f, 1.0f },
	.fadedOut = { 0.9f, 0.9f, 0.8f, 1.0f },
};

static const RgbaLifespan kBulletRosetteFlareColorLifespan {
	.initial  = { 1.0f, 1.0f, 1.0f, 0.0f },
	.fadedIn  = { 0.9f, 0.9f, 1.0f, 0.1f },
	.fadedOut = { 0.9f, 0.9f, 0.8f, 0.1f },
};

static const LightLifespan kBulletRosetteLightLifespan {
	.colorLifespan = {
		.initial  = { 1.0f, 1.0f, 1.0f },
		.fadedIn  = { 0.9f, 0.9f, 1.0f },
		.fadedOut = { 0.9f, 0.9f, 0.7f },
	},
	.radiusLifespan = { .fadedIn = 32.0f },
};

void EffectsSystemFacade::spawnBulletGenericImpactRosette( unsigned delay, const FlockOrientation &orientation,
														   float minPercentage, float maxPercentage,
														   unsigned lightFrameAffinityIndex,
														   unsigned lightFrameAffinityModulo ) {
	assert( minPercentage >= 0.0f && maxPercentage <= 1.0f );
	assert( maxPercentage >= 0.0f && maxPercentage <= 1.0f );
	assert( minPercentage <= maxPercentage );

	// TODO: Fix FlockOrientation direction

	spawnOrPostponeImpactRosetteEffect( delay, PolyEffectsSystem::ImpactRosetteParams {
		.spikeMaterial      = cgs.media.shaderGenericImpactRosetteSpike,
		.flareMaterial      = cgs.media.shaderBulletImpactFlare,
		.origin             = { orientation.origin[0], orientation.origin[1], orientation.origin[2] },
		.offset             = { orientation.offset[0], orientation.offset[1], orientation.offset[2] },
		.dir                = { -orientation.dir[0], -orientation.dir[1], -orientation.dir[2] },
		.innerConeAngle     = 5.0f,
		.outerConeAngle     = 10.0f + 50.0f * maxPercentage,
		.spawnRingRadius    = 0.0f,
		.length             = { .mean = 20.0f, .spread = 5.0f, },
		.width              = { .mean = 1.75f, .spread = 0.25f },
		.timeout            = { .min = 50, .max = 75 },
		.count              = { .min = (unsigned)( 7 * minPercentage ), .max = (unsigned)( 12 * maxPercentage ) },
		.startColorLifespan = kBulletRosetteSpikeColorLifespan,
		.endColorLifespan   = kBulletRosetteSpikeColorLifespan,
		.flareColorLifespan = kBulletRosetteFlareColorLifespan,
		.elementFlareFrameAffinityModulo = 2,
	});
}

void EffectsSystemFacade::spawnBulletImpactDoubleRosette( unsigned delay, const FlockOrientation &orientation,
														  float minPercentage, float maxPercentage,
														  unsigned lightFrameAffinityIndex,
														  unsigned lightFrameAffinityModulo,
														  const RgbaLifespan &startSpikeColorLifespan,
														  const RgbaLifespan &endSpikeColorLifespan ) {
	uint16_t innerLightFrameAffinityIndex, innerLightFrameAffinityModulo;
	uint16_t outerLightFrameAffinityIndex, outerLightFrameAffinityModulo;
	// Account for twice as many lights due to two rosettes
	if( lightFrameAffinityModulo ) {
		innerLightFrameAffinityModulo = outerLightFrameAffinityModulo = 2 * lightFrameAffinityModulo;
		innerLightFrameAffinityIndex  = lightFrameAffinityIndex;
		outerLightFrameAffinityIndex  = lightFrameAffinityIndex + lightFrameAffinityModulo;
	} else {
		innerLightFrameAffinityModulo = outerLightFrameAffinityModulo = 2;
		innerLightFrameAffinityIndex = 0;
		outerLightFrameAffinityIndex = 1;
	}

	// TODO: Fix FlockOrientation direction

	spawnOrPostponeImpactRosetteEffect( delay, PolyEffectsSystem::ImpactRosetteParams {
		.spikeMaterial      = cgs.media.shaderMetalImpactRosetteInnerSpike,
		.flareMaterial      = cgs.media.shaderBulletImpactFlare,
		.origin             = { orientation.origin[0], orientation.origin[1], orientation.origin[2] },
		.offset             = { orientation.offset[0], orientation.offset[1], orientation.offset[2] },
		.dir                = { -orientation.dir[0], -orientation.dir[1], -orientation.dir[2] },
		.innerConeAngle     = 5.0f,
		.outerConeAngle     = 15.0f,
		.spawnRingRadius    = 0.0f,
		.length             = { .mean = 32.5f, .spread = 7.5f },
		.width              = { .mean = 1.75f, .spread = 0.25f },
		.timeout            = { .min = 50, .max = 75 },
		.count              = { .min = (unsigned)( 4 * minPercentage ), .max = (unsigned)( 7 * maxPercentage ) },
		.startColorLifespan = startSpikeColorLifespan,
		.endColorLifespan   = endSpikeColorLifespan,
		.flareColorLifespan = kBulletRosetteFlareColorLifespan,
		.lightLifespan      = kBulletRosetteLightLifespan,
		.elementFlareFrameAffinityModulo = 2,
		.lightFrameAffinityModulo        = outerLightFrameAffinityModulo,
		.lightFrameAffinityIndex         = outerLightFrameAffinityIndex,
	});

	spawnOrPostponeImpactRosetteEffect( delay, PolyEffectsSystem::ImpactRosetteParams {
		.spikeMaterial      = cgs.media.shaderMetalImpactRosetteOuterSpike,
		.flareMaterial      = cgs.media.shaderBulletImpactFlare,
		.origin             = { orientation.origin[0], orientation.origin[1], orientation.origin[2] },
		.offset             = { orientation.offset[0], orientation.offset[1], orientation.offset[2] },
		.dir                = { -orientation.dir[0], -orientation.dir[1], -orientation.dir[2] },
		.innerConeAngle     = 45.0f,
		.outerConeAngle     = 75.0f,
		.spawnRingRadius    = 1.0f,
		.length             = { .mean = 15.0f, .spread = 5.0f },
		.width              = { .mean = 1.25f, .spread = 0.25f },
		.timeout            = { .min = 50, .max = 75 },
		.count              = { .min = (unsigned)( 7 * minPercentage ), .max = (unsigned)( 15 * maxPercentage ) },
		.startColorLifespan = startSpikeColorLifespan,
		.endColorLifespan   = endSpikeColorLifespan,
		.flareColorLifespan = kBulletRosetteFlareColorLifespan,
		.lightLifespan      = kBulletRosetteLightLifespan,
		.elementFlareFrameAffinityModulo = 2,
		.lightFrameAffinityModulo        = innerLightFrameAffinityModulo,
		.lightFrameAffinityIndex         = innerLightFrameAffinityIndex,
	});
}

void EffectsSystemFacade::spawnBulletMetalImpactRosette( unsigned delay, const FlockOrientation &orientation,
														 float minPercentage, float maxPercentage,
														 unsigned lightFrameAffinityIndex, unsigned lightFrameAffinityModulo ) {
	spawnBulletImpactDoubleRosette( delay, orientation, minPercentage, maxPercentage,
									lightFrameAffinityIndex, lightFrameAffinityModulo,
									kBulletRosetteSpikeColorLifespan, kBulletRosetteSpikeColorLifespan );
}

static const RgbaLifespan kBulletGlassRosetteSpikeStartColorLifespan {
	.initial  = { 1.0f, 1.0f, 1.0f, 1.0f },
	.fadedIn  = { 1.0, 1.0, 1.0f, 1.0f },
	.fadedOut = { 0.7f, 1.0f, 1.0f, 1.0f },
};

static const RgbaLifespan kBulletGlassRosetteSpikeEndColorLifespan {
	.initial  = { 1.0f, 1.0f, 1.0f, 1.0f },
	.fadedIn  = { 0.9f, 1.0f, 1.0f, 1.0f },
	.fadedOut = { 0.3f, 1.0f, 1.0f, 1.0f },
	.startFadingOutAtLifetimeFrac = 0.5f,
};

void EffectsSystemFacade::spawnBulletGlassImpactRosette( unsigned delay, const FlockOrientation &orientation,
														 float minPercentage, float maxPercentage,
														 unsigned lightFrameAffinityIndex, unsigned lightFrameAffinityModulo ) {
	spawnBulletImpactDoubleRosette( delay, orientation, minPercentage, maxPercentage,
									lightFrameAffinityIndex, lightFrameAffinityModulo,
									kBulletGlassRosetteSpikeStartColorLifespan, kBulletGlassRosetteSpikeEndColorLifespan );
}

static const RgbaLifespan kBulletMetalRicochetColors[1] {
	{
		.initial  = { 1.0f, 1.0f, 1.0f, 1.0f },
		.fadedIn  = { 0.9f, 0.9f, 1.0f, 1.0f },
		.fadedOut = { 0.9f, 0.9f, 0.7f, 1.0f },
	}
};

static const LightLifespan kBulletMetalRicochetLightProps[1] {
	{
		.colorLifespan = {
			.initial  = { 1.0f, 1.0f, 1.0f },
			.fadedIn  = { 0.9f, 0.9f, 1.0f },
			.fadedOut = { 0.8f, 0.8f, 1.0f },
		},
		.radiusLifespan = { .fadedIn = 32.0f },
	}
};

void EffectsSystemFacade::spawnBulletMetalRicochetParticles( unsigned delay, const FlockOrientation &orientation,
															 float upShiftScale, unsigned,
															 float minPercentage, float maxPercentage,
															 unsigned lightFrameAffinityIndex,
															 unsigned lightFrameAffinityModulo ) {
	const Particle::AppearanceRules appearanceRules {
		.materials     = cgs.media.shaderMetalRicochetParticle.getAddressOfHandle(),
		.colors        = kBulletMetalRicochetColors,
		.lightProps    = kBulletMetalRicochetLightProps,
		.flareProps    = Particle::FlareProps {
			.lightProps                  = kBulletMetalRicochetLightProps,
			.alphaScale                  = 0.06f,
			.radiusScale                 = 0.5f,
			.particleFrameAffinityModulo = 2,
		},
		.geometryRules = Particle::SparkRules {
			.length        = { .mean = 7.0f, .spread = 1.0f },
			.width         = { .mean = 0.75f, .spread = 0.05f },
			.sizeBehaviour = Particle::Expanding
		},
		.lightFrameAffinityIndex  = (uint16_t)lightFrameAffinityIndex,
		.lightFrameAffinityModulo = (uint16_t)lightFrameAffinityModulo,
	};

	ConicalFlockParams flockParams {
		.gravity     = GRAVITY,
		.drag        = 0.008f,
		.restitution = 0.5f,
		.angle       = 18.0f,
		.speed       = { .min = 700.0f, .max = 950.0f },
		.percentage  = { .min = minPercentage, .max = maxPercentage },
		.timeout     = { .min = 300, .max = 350 },
	};

	orientation.copyToFlockParams( &flockParams );
	assignUpShiftAndModifyBaseSpeed( &flockParams, upShiftScale, 100.0f, 150.0f );
	spawnOrPostponeImpactParticleEffect( delay, flockParams, appearanceRules );
}

static const RgbaLifespan kBulletMetalDebrisColors[3] {
	{
		.initial  = { 1.0f, 1.0f, 1.0f, 1.0f },
		.fadedIn  = { 1.0f, 0.8f, 0.5f, 1.0f },
		.fadedOut = { 1.0f, 0.8f, 0.5f, 1.0f }
	},
	{
		.initial  = { 1.0f, 1.0f, 1.0f, 1.0f },
		.fadedIn  = { 1.0f, 0.8f, 0.4f, 1.0f },
		.fadedOut = { 1.0f, 0.8f, 0.4f, 1.0f }
	},
	{
		.initial  = { 1.0f, 1.0f, 1.0f, 1.0f },
		.fadedIn  = { 1.0f, 0.7f, 0.3f, 1.0f },
		.fadedOut = { 1.0f, 0.7f, 0.3f, 1.0f }
	},
};

static LightLifespan bulletMetalDebrisLightProps[3];
static bool bulletMetalDebrisLightPropsInitialized;

void EffectsSystemFacade::spawnBulletMetalDebrisParticles( unsigned delay, const FlockOrientation &orientation,
														   float upShiftScale, unsigned,
														   float minPercentage, float maxPercentage,
														   unsigned lightFrameAffinityIndex,
														   unsigned lightFrameAffinityModulo ) {
	// Tone it down for machinegun bullet impacts
	const unsigned programLightAffinityModulo = lightFrameAffinityModulo ? lightFrameAffinityModulo : 4;
	const unsigned programLightAffinityIndex  = lightFrameAffinityIndex;

	static_assert( std::size( kBulletMetalDebrisColors ) == std::size( bulletMetalDebrisLightProps ) );
	if( !bulletMetalDebrisLightPropsInitialized ) [[unlikely]] {
		bulletMetalDebrisLightPropsInitialized = true;
		for( size_t i = 0; i < std::size( kBulletMetalDebrisColors ); ++i ) {
			const RgbaLifespan *from   = &kBulletMetalDebrisColors[i];
			LightLifespan *const to    = &bulletMetalDebrisLightProps[i];
			to->radiusLifespan.fadedIn = 24.0f;
			// TODO: We don't copy lifetime fracs, should we?
			VectorCopy( from->initial,  to->colorLifespan.initial );
			VectorCopy( from->fadedIn,  to->colorLifespan.fadedIn );
			VectorCopy( from->fadedOut, to->colorLifespan.fadedOut );
		}
	}

	const Particle::AppearanceRules appearanceRules {
		.materials      = cgs.media.shaderMetalDebrisParticle.getAddressOfHandle(),
		.colors         = kBulletMetalDebrisColors,
		.lightProps     = bulletMetalDebrisLightProps,
		.flareProps     = Particle::FlareProps {
			.lightProps                  = bulletMetalDebrisLightProps,
			.alphaScale                  = 0.06f,
			.radiusScale                 = 0.5f,
			.particleFrameAffinityModulo = 2,
		},
		.geometryRules  = Particle::SparkRules {
			.length        = { .mean = 2.5f, .spread = 1.0f },
			.width         = { .mean = 0.75f, .spread  = 0.05f },
			.sizeBehaviour = Particle::Expanding
		},
		.lightFrameAffinityIndex  = (uint16_t)programLightAffinityIndex,
		.lightFrameAffinityModulo = (uint16_t)programLightAffinityModulo,
	};

	ConicalFlockParams flockParams {
		.gravity      = GRAVITY,
		.restitution  = 0.3f,
		.angle        = 30.0f,
		.bounceCount  = { .minInclusive = 1, .maxInclusive = 3 },
		.speed        = { .min = 75.0f, .max = 125.0f },
		.percentage   = { .min = minPercentage, .max = maxPercentage },
		.timeout      = { .min = 200, .max = 700 },
	};

	orientation.copyToFlockParams( &flockParams );
	assignUpShiftAndModifyBaseSpeed( &flockParams, upShiftScale, 150.0f, 200.0f );
	spawnOrPostponeImpactParticleEffect( delay, flockParams, appearanceRules );
}

static const RgbaLifespan kGreyDustColors[1] {
	{
		.initial  = { 0.5f, 0.5f, 0.5f, 0.1f },
		.fadedIn  = { 0.5f, 0.5f, 0.5f, 0.1f },
		.fadedOut = { 0.5f, 0.5f, 0.5f, 0.0f },
		.startFadingOutAtLifetimeFrac = 0.67f
	}
};

static shader_s *g_stoneDustMaterialsStorage[2];

void EffectsSystemFacade::spawnStoneDustParticles( unsigned delay, const FlockOrientation &orientation,
												   float upShiftScale, unsigned materialParam,
												   float dustPercentageScale ) {
	g_stoneDustMaterialsStorage[0] = cgs.media.shaderStoneDustHard;
	g_stoneDustMaterialsStorage[1] = cgs.media.shaderStoneDustSoft;

	const Particle::AppearanceRules appearanceRules {
		.materials           = g_stoneDustMaterialsStorage,
		.colors              = kGreyDustColors,
		.numMaterials        = std::size( g_stoneDustMaterialsStorage ),
		.geometryRules       = Particle::SpriteRules {
			.radius = { .mean = 25.0f, .spread = 7.5f }, .sizeBehaviour = Particle::Expanding
		},
		.applyVertexDynLight = true
	};

	ConicalFlockParams flockParams {
		.gravity         = 50.0f,
		.drag            = 0.03f,
		.restitution     = 1.0f,
		.angle           = 30.0f,
		.speed           = { .min = 100.0f, .max = 500.0f },
		.angularVelocity = { .min = -180.0f, .max = +180.0f },
		.percentage      = { .min = 0.7f * dustPercentageScale, .max = 1.0f * dustPercentageScale },
		.timeout         = { .min = 750, .max = 1000 },
	};

	orientation.copyToFlockParams( &flockParams );
	assignUpShiftAndModifyBaseSpeed( &flockParams, upShiftScale, 10.0f, 20.0f );
	spawnOrPostponeImpactParticleEffect( delay, flockParams, appearanceRules );
}

static shader_s *g_stuccoDustMaterialsStorage[3];

void EffectsSystemFacade::spawnStuccoDustParticles( unsigned delay, const FlockOrientation &orientation,
													float upShiftScale, unsigned materialParam ) {
	Particle::AppearanceRules appearanceRules {
		.materials           = cgs.media.shaderStuccoDustSoft.getAddressOfHandle(),
		.colors              = kGreyDustColors,
		.geometryRules       = Particle::SpriteRules {
			.radius = { .mean = 50.0f, .spread = 5.0f }, .sizeBehaviour = Particle::Expanding
		},
		.applyVertexDynLight = true
	};

	ConicalFlockParams flockParams {
		.gravity     = 50.0f,
		.drag        = 0.03f,
		.restitution = 1.0f,
		.angle       = 30.0f,
		.speed       = { .min = 100.0f, .max = 500.0f },
		.percentage  = { .min = 0.5f, .max = 1.0f },
		.timeout     = { .min = 1500, .max = 2000 },
	};

	orientation.copyToFlockParams( &flockParams );
	assignUpShiftAndModifyBaseSpeed( &flockParams, upShiftScale, 20.0f, 30.0f );
	spawnOrPostponeImpactParticleEffect( delay, flockParams, appearanceRules );

	g_stuccoDustMaterialsStorage[0] = cgs.media.shaderStuccoDustMedium;
	g_stuccoDustMaterialsStorage[1] = cgs.media.shaderStuccoDustMedium;
	g_stuccoDustMaterialsStorage[2] = cgs.media.shaderStuccoDustHard;

	appearanceRules.materials     = g_stuccoDustMaterialsStorage;
	appearanceRules.numMaterials  = std::size( g_stuccoDustMaterialsStorage );
	appearanceRules.geometryRules = Particle::SpriteRules {
		.radius = { .mean = 15.0f, .spread = 2.5f }, .sizeBehaviour = Particle::Expanding,
	};

	flockParams.angularVelocity = { .min = -60.0f, .max = +60.0f };
	flockParams.percentage      = { .min = 0.3f, .max = 0.5f };

	spawnOrPostponeImpactParticleEffect( delay, flockParams, appearanceRules );
}

static const RgbaLifespan kWoodImpactColors[1] {
	{
		.initial  = { 0.5f, 0.4f, 0.3f, 1.0f },
		.fadedIn  = { 0.5f, 0.4f, 0.3f, 1.0f },
		.fadedOut = { 0.5f, 0.4f, 0.3f, 1.0f },
	}
};

static const RgbaLifespan kWoodDustColors[1] {
	{
		.initial  = { 0.5f, 0.4f, 0.3f, 0.0f },
		.fadedIn  = { 0.5f, 0.4f, 0.3f, 0.1f },
		.fadedOut = { 0.5f, 0.4f, 0.3f, 0.0f },
		.finishFadingInAtLifetimeFrac = 0.1f,
		.startFadingOutAtLifetimeFrac = 0.7f
	}
};

void EffectsSystemFacade::spawnWoodBulletImpactParticles( unsigned delay, const FlockOrientation &orientation,
														  float upShiftScale, unsigned materialParam,
														  float debrisPercentageScale ) {
	const Particle::AppearanceRules burstAppearanceRules {
		.materials     = cgs.media.shaderWoodBurstParticle.getAddressOfHandle(),
		.colors        = kWoodImpactColors,
		.geometryRules = Particle::SparkRules {
			.length        = { .mean = 20.0f, .spread = 3.0f },
			.width         = { .mean = 3.0f, .spread = 0.5f },
			.sizeBehaviour = Particle::Shrinking,
		}
	};

	ConicalFlockParams burstFlockParams {
		.angle      = 24,
		.speed      = { .min = 700, .max = 900 },
		.percentage = { .min = 0.3f, .max = 0.6f },
		.timeout    = { .min = 75, .max = 150 },
	};

	const Particle::AppearanceRules dustAppearanceRules {
		.materials           = cgs.media.shaderWoodDustParticle.getAddressOfHandle(),
		.colors              = kWoodDustColors,
		.geometryRules       = Particle::SpriteRules {
			.radius = { .mean = 25.0f, .spread = 5.0f }, .sizeBehaviour = Particle::Expanding,
		},
		.applyVertexDynLight = true,
	};

	ConicalFlockParams dustFlockParams {
		.gravity    = 25.0f,
		.angle      = 24.0f,
		.speed      = { .min = 50.0f, .max = 150.0f },
		.percentage = { .min = 1.0f, .max = 1.0f },
		.timeout    = { .min = 300, .max = 500 },
	};

	const Particle::AppearanceRules debrisAppearanceRules {
		.materials     = cgs.media.shaderWoodDebrisParticle.getAddressOfHandle(),
		.colors        = kWoodImpactColors,
		.geometryRules = Particle::SparkRules {
			.length = { .mean = 5.0f, .spread = 1.5f },
			.width  = { .mean = 1.5f, .spread  = 0.5f },
		}
	};

	ConicalFlockParams debrisFlockParams {
		.gravity         = 0.75f * GRAVITY,
		.drag            = 0.02f,
		.restitution     = 0.5f,
		.angle           = 30.0f,
		.bounceCount     = { .minInclusive = 2, .maxInclusive = 3 },
		.speed           = { .min = 400.0f, .max = 700.0f },
		.angularVelocity = { .min = -9.0f * 360.0f, .max = 9.0f * 360.0f },
		.percentage      = { .min = 0.2f * debrisPercentageScale, .max = 0.5f * debrisPercentageScale },
		.timeout         = { .min = 150, .max = 500 },
	};

	orientation.copyToFlockParams( &burstFlockParams );
	spawnOrPostponeImpactParticleEffect( delay, burstFlockParams, burstAppearanceRules,
										 TransientEffectsSystem::ParticleFlockBin::Medium );

	orientation.copyToFlockParams( &dustFlockParams );
	spawnOrPostponeImpactParticleEffect( delay, dustFlockParams, dustAppearanceRules );

	orientation.copyToFlockParams( &debrisFlockParams );
	assignUpShiftAndModifyBaseSpeed( &debrisFlockParams, upShiftScale, 75.0f, 125.0f );
	spawnOrPostponeImpactParticleEffect( delay, debrisFlockParams, debrisAppearanceRules,
										 TransientEffectsSystem::ParticleFlockBin::Medium );
}

static const RgbaLifespan kDirtImpactColors[1] {
	{
		.initial  = { 0.3f, 0.25f, 0.1f, 1.0f },
		.fadedIn  = { 0.3f, 0.25f, 0.1f, 1.0f },
		.fadedOut = { 0.3f, 0.25f, 0.1f, 0.0f },
	}
};

static const RgbaLifespan kDirtDustColors[1] {
	{
		.initial  = { 0.3f, 0.25f, 0.1f, 0.0f },
		.fadedIn  = { 0.3f, 0.25f, 0.1f, 0.3f },
		.fadedOut = { 0.3f, 0.25f, 0.1f, 0.0f },
	}
};

static shader_s *g_dirtCloudMaterials[2];

void EffectsSystemFacade::spawnDirtImpactParticles( unsigned delay, const FlockOrientation &orientation,
													float upShiftScale, unsigned materialParam,
													float percentageScale, float dustSpeedScale ) {
	ConicalFlockParams burstStripesFlockParams {
		.gravity    = GRAVITY,
		.angle      = 12,
		.speed      = { .min = 500, .max = 700 },
		.percentage = { .min = 0.5f * percentageScale, .max = 1.0f * percentageScale },
		.timeout    = { .min = 100, .max = 200 },
	};

	Particle::AppearanceRules burstStripesAppearanceRules {
		.materials     = cgs.media.shaderDirtImpactBurst.getAddressOfHandle(),
		.colors        = kDirtImpactColors,
		.geometryRules = Particle::SparkRules {
			.length        = { .mean = 30.0f, .spread = 10.0f },
			.width         = { .mean = 4.0f, .spread  = 1.0f },
			.sizeBehaviour = Particle::Shrinking,
		},
	};

	ConicalFlockParams particlesAndCloudFlockParams {
		.gravity    = GRAVITY,
		.drag       = 0.01f,
		.angle      = 12,
		.speed      = { .min = 500, .max = 700 },
		.percentage = { .min = 0.5f * percentageScale, .max = 1.0f * percentageScale },
		.timeout    = { .min = 350, .max = 1000 }
	};

	const Particle::AppearanceRules smallParticlesAppearanceRules {
		.materials     = cgs.media.shaderDirtImpactParticle.getAddressOfHandle(),
		.colors        = kDirtImpactColors,
		.geometryRules = Particle::SpriteRules { .radius = { .mean = 3.0f }, .sizeBehaviour = Particle::Shrinking },
	};

	static_assert( std::size( g_dirtCloudMaterials ) == 2 );
	g_dirtCloudMaterials[0] = cgs.media.shaderDirtImpactCloudSoft;
	g_dirtCloudMaterials[1] = cgs.media.shaderDirtImpactCloudHard;

	Particle::AppearanceRules cloudAppearanceRules {
		.materials     = g_dirtCloudMaterials,
		.colors        = kDirtDustColors,
		.numMaterials  = std::size( g_dirtCloudMaterials ),
		.geometryRules = Particle::SpriteRules {
			.radius = { .mean = 10.0f, .spread = 7.5f }, .sizeBehaviour = Particle::ExpandingAndShrinking,
		},
	};

	orientation.copyToFlockParams( &burstStripesFlockParams );
	// Never delay stripes
	cg.particleSystem.addSmallParticleFlock( burstStripesAppearanceRules, burstStripesFlockParams );

	orientation.copyToFlockParams( &particlesAndCloudFlockParams );
	assignUpShiftAndModifyBaseSpeed( &particlesAndCloudFlockParams, upShiftScale, 150.0f, 200.0f );
	// Never delay burst
	cg.particleSystem.addMediumParticleFlock( smallParticlesAppearanceRules, particlesAndCloudFlockParams );

	particlesAndCloudFlockParams.timeout         = { .min = 350, .max = 700 };
	particlesAndCloudFlockParams.angularVelocity = { .min = -180.0f, .max = 180.0f };

	assignUpShiftAndModifyBaseSpeed( &particlesAndCloudFlockParams, upShiftScale, 50.0f, 125.0f );
	spawnOrPostponeImpactParticleEffect( delay, particlesAndCloudFlockParams, cloudAppearanceRules );
}

static const RgbaLifespan kSandImpactColors[1] {
	{
		.initial  = { 0.8f, 0.7f, 0.5f, 0.7f },
		.fadedIn  = { 0.8f, 0.7f, 0.5f, 0.7f },
		.fadedOut = { 0.8f, 0.7f, 0.5f, 0.0f },
		.startFadingOutAtLifetimeFrac = 0.67f,
	}
};

static const RgbaLifespan kSandDustColors[1] {
	{
		.initial  = { 0.8f, 0.7f, 0.5f, 0.3f },
		.fadedIn  = { 0.8f, 0.7f, 0.5f, 0.3f },
		.fadedOut = { 0.8f, 0.7f, 0.5f, 0.0f },
		.startFadingOutAtLifetimeFrac = 0.67f,
	}
};

void EffectsSystemFacade::spawnSandImpactParticles( unsigned delay, const FlockOrientation &orientation,
													float upShiftScale, unsigned materialParam,
													float percentageScale, float dustSpeedScale ) {
	// Don't let the percentage affect burst
	ConicalFlockParams burstFlockParams {
		.gravity    = GRAVITY,
		.angle      = 12,
		.speed      = { .min = 550, .max = 700 },
		.percentage = { .min = 0.7f, .max = 1.0f },
		.timeout    = { .min = 300, .max = 400 },
	};

	const Particle::AppearanceRules burstParticlesAppearanceRules {
		.materials           = cgs.media.shaderSandImpactBurst.getAddressOfHandle(),
		.colors              = kSandImpactColors,
		.geometryRules       = Particle::SpriteRules {
			.radius = { .mean = 3.0f }, .sizeBehaviour = Particle::Shrinking
		},
		.applyVertexDynLight = true,
	};

	orientation.copyToFlockParams( &burstFlockParams );
	assignUpShiftAndModifyBaseSpeed( &burstFlockParams, upShiftScale, 150.0f, 200.0f );
	// Never delay burst
	cg.particleSystem.addSmallParticleFlock( burstParticlesAppearanceRules, burstFlockParams );

	assert( dustSpeedScale >= 1.0f );
	const float timeoutScale = 0.5f * ( 1.0f + Q_Rcp( dustSpeedScale ) );

	EllipsoidalFlockParams dustFlockParams {
		.stretchScale = 0.33f * Q_Rcp( dustSpeedScale ),
		.gravity      = 100.0f,
		.speed        = { .min = 50 * dustSpeedScale, .max = 100 * dustSpeedScale },
		.percentage   = { .min = 0.7f * percentageScale, .max = 1.0f * percentageScale },
		.timeout      = { .min = (unsigned)( 500 * timeoutScale ), .max = (unsigned)( 750 * timeoutScale ) },
	};

	Particle::AppearanceRules dustAppearanceRules {
		.materials           = cgs.media.shaderSandImpactDustSoft.getAddressOfHandle(),
		.colors              = kSandDustColors,
		.geometryRules       = Particle::SpriteRules {
			.radius = { .mean = 40.0f, .spread = 7.5f }, .sizeBehaviour = Particle::Expanding,
		},
		.applyVertexDynLight = true,
	};

	orientation.copyToFlockParams( &dustFlockParams );
	assignUpShiftAndModifyBaseSpeed( &dustFlockParams, upShiftScale, 20.0f, 50.0f );
	spawnOrPostponeImpactParticleEffect( delay, dustFlockParams, dustAppearanceRules );

	dustAppearanceRules.materials = cgs.media.shaderSandImpactDustHard.getAddressOfHandle();
	dustAppearanceRules.geometryRules = Particle::SpriteRules {
		.radius = { .mean = 7.5f, .spread = 2.5f }, .sizeBehaviour = Particle::Expanding,
	};

	dustFlockParams.speed           = { .min = 30 * dustSpeedScale, .max = 50 * dustSpeedScale };
	dustFlockParams.timeout         = { .min = (unsigned)( 200 * timeoutScale ), .max = (unsigned)( 450 * timeoutScale ) };
	dustFlockParams.angularVelocity = { .min = -180.0f, .max = +180.0f };
	assignUpShiftAndModifyBaseSpeed( &dustFlockParams, upShiftScale, 50.0f, 75.0f );
	spawnOrPostponeImpactParticleEffect( delay, dustFlockParams, dustAppearanceRules );
}

static const RgbaLifespan kGlassDebrisColors[1] {
	{
		.initial  = { 1.0f, 1.0f, 1.0f, 0.0f },
		.fadedIn  = { 0.7f, 1.0f, 1.0f, 0.5f },
		.fadedOut = { 0.7f, 1.0f, 1.0f, 0.0f },
		.finishFadingInAtLifetimeFrac = 0.25f,
		.startFadingOutAtLifetimeFrac = 0.50f,
	}
};

void EffectsSystemFacade::spawnGlassImpactParticles( unsigned delay, const FlockOrientation &orientation,
													 float upShiftScale, unsigned materialParam, float percentageScale ) {
	Particle::AppearanceRules appearanceRules {
		.materials     = cgs.media.shaderGlassDebrisParticle.getAddressOfHandle(),
		.colors        = kGlassDebrisColors,
		.geometryRules = Particle::SparkRules {
			.length           = { .mean = 2.5f, .spread = 1.0f },
			.width            = { .mean = 2.5f, .spread = 1.0f },
			.viewDirPartScale = 0.33f,
		},
	};

	ConicalFlockParams flockParams {
		.gravity         = 1.5f * GRAVITY,
		.drag            = 0.003f,
		.angle           = 37.0f,
		.speed           = { .min = 300.0f, .max = 500.0f },
		.angularVelocity = { .min = 3 * 360.0f, .max = 7 * 360.0f },
		.percentage      = { .min = 1.0f * percentageScale, .max = 1.0f * percentageScale },
		.timeout         = { .min = 150, .max = 350 },
	};

	orientation.copyToFlockParams( &flockParams );
	spawnOrPostponeImpactParticleEffect( delay, flockParams, appearanceRules );

	if( m_rng.tryWithChance( 0.33f ) ) {
		appearanceRules.geometryRules = Particle::SparkRules {
			.length           = { .mean = 5.5f, .spread = 1.5f },
			.width            = { .mean = 5.5f, .spread = 1.5f },
			.viewDirPartScale = 0.33f,
		};

		flockParams.percentage = { .min = 0.1f * percentageScale, .max = 0.3f * percentageScale };
		flockParams.timeout    = { .min = 250, .max = 350 };
		spawnOrPostponeImpactParticleEffect( delay, flockParams, appearanceRules );
	}
}

void EffectsSystemFacade::spawnBulletImpactEffect( unsigned delay, const SolidImpact &impact ) {
	const FlockOrientation flockOrientation = makeRicochetFlockOrientation( impact, &m_rng );

	sfx_s *sfx         = nullptr;
	uintptr_t groupTag = 0;
	if( v_particles.get() ) {
		const SurfImpactMaterial impactMaterial = decodeSurfImpactMaterial( impact.surfFlags );
		const unsigned materialParam            = decodeSurfImpactMaterialParam( impact.surfFlags );
		spawnBulletImpactParticleEffectForMaterial( delay, flockOrientation, impactMaterial, materialParam );
		// TODO: Using enum (doesn't work with GCC 10)
		using IM = SurfImpactMaterial;
		if( impactMaterial == IM::Metal ) {
			spawnBulletMetalImpactRosette( delay, flockOrientation, 1.0f, 1.0f );
		} else if( impactMaterial == IM::Stone ) {
			spawnBulletGenericImpactRosette( delay, flockOrientation, 0.5f, 1.0f );
		} else if( impactMaterial == IM::Glass ) {
			spawnBulletGlassImpactRosette( delay, flockOrientation, 0.5f, 1.0f );
		} else if( impactMaterial == IM::Unknown ) {
			spawnBulletGenericImpactRosette( delay, flockOrientation, 0.3f, 1.0f );
		}
		if( impactMaterial == IM::Metal || impactMaterial == IM::Stone || impactMaterial == IM::Unknown ) {
			// TODO: Postpone if needed
			m_transientEffectsSystem.spawnBulletImpactModel( impact.origin, impact.normal );
		}
		if( impactMaterial == IM::Metal || impactMaterial == IM::Glass ) {
			spawnBulletLikeImpactRingUsingLimiter( delay, impact );
		}
		const unsigned group = getImpactSfxGroupForMaterial( impactMaterial );
		sfx      = getSfxForImpactGroup( group );
		groupTag = group;
	} else {
		spawnBulletGenericImpactRosette( delay, flockOrientation, 0.5f, 1.0f );
		// TODO: Postpone if needed
		m_transientEffectsSystem.spawnBulletImpactModel( impact.origin, impact.normal );
		if( const unsigned numSfx = cgs.media.sfxImpactSolid.length() ) {
			sfx      = cgs.media.sfxImpactSolid[m_rng.nextBounded( numSfx )];
			groupTag = 0;
		}
	}

	if( sfx ) {
		startSoundForImpactUsingLimiter( sfx, groupTag, impact, EventRateLimiterParams {
			.dropChanceAtZeroDistance = 0.5f,
			.startDroppingAtDistance  = 144.0f,
			.dropChanceAtZeroTimeDiff = 1.0f,
			.startDroppingAtTimeDiff  = 250,
		});
	}
}

void EffectsSystemFacade::spawnBulletImpactParticleEffectForMaterial( unsigned delay,
																	  const FlockOrientation &flockOrientation,
																	  SurfImpactMaterial impactMaterial,
																	  unsigned materialParam ) {
	// TODO: We used to test against impact normal Z
	[[maybe_unused]] const float upShiftScale = Q_Sqrt( wsw::max( 0.0f, flockOrientation.dir[2] ) );

	switch( impactMaterial ) {
		case SurfImpactMaterial::Unknown:
			break;
		case SurfImpactMaterial::Stone:
			spawnStoneDustParticles( delay, flockOrientation, upShiftScale, materialParam );
			break;
		case SurfImpactMaterial::Stucco:
			spawnStuccoDustParticles( delay, flockOrientation, upShiftScale, materialParam );
			break;
		case SurfImpactMaterial::Wood:
			spawnWoodBulletImpactParticles( delay, flockOrientation, upShiftScale, materialParam );
			break;
		case SurfImpactMaterial::Dirt:
			spawnDirtImpactParticles( delay, flockOrientation, upShiftScale, materialParam );
			break;
		case SurfImpactMaterial::Sand:
			spawnSandImpactParticles( delay, flockOrientation, upShiftScale, materialParam );
			break;
		case SurfImpactMaterial::Metal:
			spawnBulletMetalRicochetParticles( delay, flockOrientation, upShiftScale, materialParam, 0.7f, 1.0f );
			spawnBulletMetalDebrisParticles( delay, flockOrientation, upShiftScale, materialParam, 0.3f, 0.9f );
			break;
		case SurfImpactMaterial::Glass:
			spawnGlassImpactParticles( delay, flockOrientation, upShiftScale, materialParam, 1.0f );
			break;
	}
}

void EffectsSystemFacade::spawnBulletLikeImpactRingUsingLimiter( unsigned delay, const SolidImpact &impact ) {
	const EventRateLimiterParams limiterParams {
		.startDroppingAtDistance = 144.0f,
		.startDroppingAtTimeDiff = 350,
	};

	if( !m_solidImpactRingsRateLimiter.acquirePermission( cg.time, impact.origin, limiterParams ) ) {
		return;
	}

	constexpr float minDot      = 0.15f;
	constexpr float maxDot      = 0.55f;
	constexpr float rcpDotRange = 1.0f / ( maxDot - minDot );
	// Limit to prevent infinite looping (it should not really happen though).
	for( unsigned attemptNum = 0; attemptNum < 12; ++attemptNum ) {
		const float *axisDir = kPredefinedDirs[m_rng.nextBoundedFast( std::size( kPredefinedDirs ) )];
		const float dot      = DotProduct( axisDir, impact.incidentDir );
		if( dot > minDot && dot < maxDot ) {
			// Prefer values closer to the min dot
			const float dotFrac      = ( dot - minDot ) * rcpDotRange;
			const float acceptChance = 1.0f - dotFrac;
			if( m_rng.tryWithChance( acceptChance ) ) {
				cg.polyEffectsSystem.spawnSimulatedRing( PolyEffectsSystem::SimulatedRingParams {
					.origin     = { impact.origin[0], impact.origin[1], impact.origin[2] },
					.offset     = { impact.normal[0], impact.normal[1], impact.normal[2] },
					.axisDir    = { axisDir[0], axisDir[1], axisDir[2] },
					.alphaLifespan = ValueLifespan {
						.initial   = 1.0f,
						.fadedIn   = 0.33f,
						.fadedOut  = 0.0f,
						.finishFadingInAtLifetimeFrac = 0.15f,
						.startFadingOutAtLifetimeFrac = 0.35f,
					},
					.innerSpeed              = { 200.0f, 5.0f },
					.outerSpeed              = { 400.0f, 5.0f },
					.lifetime                = 375 + delay,
					.simulationDelay         = delay,
					.movementDuration        = 300,
					.innerTexCoordFrac       = 0.5f,
					.outerTexCoordFrac       = 1.0f,
					.numClipMoveSmoothSteps  = 10,
					.numClipAlphaSmoothSteps = 3,
					.softenOnContact         = true,
					.material                = cgs.media.shaderImpactRing,
				});
				return;
			}
		}
	}
}

auto EffectsSystemFacade::getImpactSfxGroupForMaterial( SurfImpactMaterial impactMaterial ) -> unsigned {
	using IM = SurfImpactMaterial;
	if( impactMaterial == IM::Metal ) {
		return 1;
	}
	if( impactMaterial == IM::Stucco || impactMaterial == IM::Dirt || impactMaterial == IM::Sand ) {
		return 2;
	}
	if( impactMaterial == IM::Wood ) {
		return 3;
	}
	if( impactMaterial == IM::Glass ) {
		return 4;
	}
	return 0;
}

auto EffectsSystemFacade::getSfxForImpactGroup( unsigned group ) -> sfx_s * {
	// Build in a lazy fashion, so we don't have to care of lifetimes
	if( !m_impactSfxForGroups.full() ) [[unlikely]] {
		auto &ma = cgs.media;
		m_impactSfxForGroups.push_back( { ma.sfxImpactSolid.getAddressOfHandles(), ma.sfxImpactSolid.length() } );
		m_impactSfxForGroups.push_back( { ma.sfxImpactMetal.getAddressOfHandles(), ma.sfxImpactMetal.length() } );
		m_impactSfxForGroups.push_back( { ma.sfxImpactSoft.getAddressOfHandles(), ma.sfxImpactSoft.length() } );
		m_impactSfxForGroups.push_back( { ma.sfxImpactWood.getAddressOfHandles(), ma.sfxImpactWood.length() } );
		m_impactSfxForGroups.push_back( { ma.sfxImpactGlass.getAddressOfHandles(), ma.sfxImpactGlass.length() } );
	}

	assert( m_impactSfxForGroups.full() && group < m_impactSfxForGroups.size() );
	auto [sfxData, dataLen] = m_impactSfxForGroups[group];
	if( dataLen ) {
		return sfxData[m_rng.nextBounded( dataLen )];
	}
	return nullptr;
}

void EffectsSystemFacade::spawnPelletImpactParticleEffectForMaterial( unsigned delay,
																	  const FlockOrientation &flockOrientation,
																	  SurfImpactMaterial impactMaterial,
																	  unsigned materialParam,
																	  unsigned lightFrameAffinityIndex,
																	  unsigned lightFrameAffinityModulo ) {
	// TODO: We used to test against impact normal Z
	[[maybe_unused]] const float upShiftScale = Q_Sqrt( wsw::max( 0.0f, flockOrientation.dir[2] ) );

	switch( impactMaterial ) {
		case SurfImpactMaterial::Unknown:
			break;
		case SurfImpactMaterial::Stone:
			spawnStoneDustParticles( delay, flockOrientation, upShiftScale, materialParam, 0.75f );
			break;
		case SurfImpactMaterial::Stucco:
			spawnStuccoDustParticles( delay, flockOrientation, upShiftScale, materialParam );
			break;
		case SurfImpactMaterial::Wood:
			spawnWoodBulletImpactParticles( delay, flockOrientation, upShiftScale, materialParam, 0.5f );
			break;
		case SurfImpactMaterial::Dirt:
			spawnDirtImpactParticles( delay, flockOrientation, upShiftScale, materialParam, 0.33f );
			break;
		case SurfImpactMaterial::Sand:
			spawnSandImpactParticles( delay, flockOrientation, upShiftScale, materialParam, 0.25f );
			break;
		case SurfImpactMaterial::Metal:
			// These conditionals make light frame affinity slightly incorrect but this is harmless
			if( m_rng.tryWithChance( 0.5f ) ) {
				spawnBulletMetalRicochetParticles( delay, flockOrientation, upShiftScale, materialParam, 0.0f, 0.5f,
												   lightFrameAffinityIndex, lightFrameAffinityModulo );
			}
			if( m_rng.tryWithChance( 0.5f ) ) {
				spawnBulletMetalDebrisParticles( delay, flockOrientation, upShiftScale, materialParam, 0.0f, 0.5f,
												 lightFrameAffinityIndex, lightFrameAffinityModulo );
			}
			break;
		case SurfImpactMaterial::Glass:
			spawnGlassImpactParticles( delay, flockOrientation, upShiftScale, materialParam, 0.25f );
			break;
	}
}

void EffectsSystemFacade::spawnExplosionImpactParticleEffectForMaterial( const FlockOrientation &flockOrientation,
																		 SurfImpactMaterial impactMaterial,
																		 unsigned materialParam,
																		 unsigned lightFrameAffinityIndex,
																		 unsigned lightFrameAffinityModulo ) {
	// TODO: We used to test against impact normal Z
	[[maybe_unused]] const float upShiftScale = Q_Sqrt( wsw::max( 0.0f, flockOrientation.dir[2] ) );
	[[maybe_unused]] unsigned delay = 0;

	switch( impactMaterial ) {
		case SurfImpactMaterial::Unknown:
			break;
		case SurfImpactMaterial::Stone:
			delay = 100 + m_rng.nextBoundedFast( 100 );
			spawnStoneDustParticles( delay, flockOrientation, upShiftScale, materialParam );
			break;
		case SurfImpactMaterial::Stucco:
			delay = 100 + m_rng.nextBoundedFast( 100 );
			spawnStuccoDustParticles( delay, flockOrientation, upShiftScale, materialParam );
			break;
		case SurfImpactMaterial::Wood:
			delay = 50 + m_rng.nextBoundedFast( 150 );
			spawnWoodBulletImpactParticles( delay, flockOrientation, upShiftScale, materialParam );
			break;
		case SurfImpactMaterial::Dirt:
			delay = m_rng.nextBoundedFast( 300 );
			spawnDirtImpactParticles( delay, flockOrientation, upShiftScale, materialParam, 0.67f, 2.0f );
			break;
		case SurfImpactMaterial::Sand:
			delay = 100 + m_rng.nextBoundedFast( 300 );
			spawnSandImpactParticles( delay, flockOrientation, upShiftScale, materialParam, 0.67f, 2.0f );
			break;
		case SurfImpactMaterial::Metal:
			delay = m_rng.nextBoundedFast( 100 );
			if( m_rng.next() % 2 ) {
				spawnBulletMetalRicochetParticles( delay, flockOrientation, upShiftScale, materialParam, 0.7f, 1.0f,
												   lightFrameAffinityIndex, lightFrameAffinityModulo );
			} else {
				spawnBulletMetalDebrisParticles( delay, flockOrientation, upShiftScale, materialParam, 0.3f, 0.9f,
												 lightFrameAffinityIndex, lightFrameAffinityModulo );
			}
			break;
		case SurfImpactMaterial::Glass:
			delay = m_rng.nextBoundedFast( 100 );
			spawnGlassImpactParticles( delay, flockOrientation, upShiftScale, materialParam, 0.25f );
			break;
	}
}

template <typename FlockParams>
void EffectsSystemFacade::spawnOrPostponeImpactParticleEffect( unsigned delay,
															   const FlockParams &flockParams,
															   const Particle::AppearanceRules &appearanceRules,
															   TransientEffectsSystem::ParticleFlockBin bin,
															   const ParamsOfParticleTrailOfParticles *paramsOfParticleTrail,
															   const ParamsOfPolyTrailOfParticles *paramsOfPolyTrail ) {
	if( delay ) {
		m_transientEffectsSystem.addDelayedParticleEffect( delay, bin, flockParams, appearanceRules,
														   paramsOfParticleTrail, paramsOfPolyTrail );
	} else {
		// TODO: Hide bins from ParticleSystem public interface
		if( bin == TransientEffectsSystem::ParticleFlockBin::Small ) {
			cg.particleSystem.addSmallParticleFlock( appearanceRules, flockParams, paramsOfParticleTrail, paramsOfPolyTrail );
		} else if( bin == TransientEffectsSystem::ParticleFlockBin::Medium ) {
			cg.particleSystem.addMediumParticleFlock( appearanceRules, flockParams, paramsOfParticleTrail, paramsOfPolyTrail );
		} else {
			cg.particleSystem.addLargeParticleFlock( appearanceRules, flockParams, paramsOfParticleTrail, paramsOfPolyTrail );
		}
	}
}

void EffectsSystemFacade::spawnOrPostponeImpactRosetteEffect( unsigned delay, PolyEffectsSystem::ImpactRosetteParams &&params ) {
	if( delay ) {
		// TODO: Use generic closures?
		m_transientEffectsSystem.addDelayedImpactRosetteEffect( delay, params );
	} else {
		cg.polyEffectsSystem.spawnImpactRosette( std::forward<PolyEffectsSystem::ImpactRosetteParams>( params ) );
	}
}

static const RgbaLifespan kWaterSplashColors[1] {
	{
		.initial  = { 1.0f, 1.0f, 1.0f, 0.7f },
		.fadedIn  = { 1.0f, 1.0f, 1.0f, 0.3f },
		.fadedOut = { 0.0f, 0.0f, 1.0f, 0.0f },
	}
};

static const RgbaLifespan kWaterDustColors[1] {
	{
		.initial  = { 1.0f, 1.0f, 1.0f, 0.0f },
		.fadedIn  = { 1.0f, 1.0f, 1.0f, 0.1f },
		.fadedOut = { 0.0f, 0.0f, 1.0f, 0.0f },
	}
};

static const RgbaLifespan kSlimeSplashColors[1] {
	{
		.initial  = { 1.0f, 1.0f, 0.0f, 0.7f },
		.fadedIn  = { 0.0f, 1.0f, 0.0f, 0.3f },
		.fadedOut = { 0.0f, 1.0f, 0.0f, 0.0f },
	}
};

static const RgbaLifespan kSlimeDustColors[1] {
	{
		.initial  = { 1.0f, 1.0f, 1.0f, 0.0f },
		.fadedIn  = { 0.8f, 1.0f, 0.9f, 0.1f },
		.fadedOut = { 0.0f, 1.0f, 0.0f, 0.0f },
	}
};

static const RgbaLifespan kLavaSplashColors[1] {
	{
		.initial  = { 1.0f, 0.67f, 0.0f, 1.0f },
		.fadedIn  = { 1.0f, 0.67f, 0.0f, 1.0f },
		.fadedOut = { 0.5f, 0.3f, 0.3f, 0.0f },
	}
};

static const RgbaLifespan kLavaDropsColors[3] {
	{
		.initial  = { 1.0f, 0.67f, 0.1f, 1.0f },
		.fadedIn  = { 1.0f, 0.67f, 0.01f, 1.0f },
		.fadedOut = { 1.0f, 0.67f, 0.075f, 0.3f }
	},
	{
		.initial  = { 1.0f, 0.67f, 0.1f, 1.0f },
		.fadedIn  = { 1.0f, 0.5f, 0.1f, 1.0f },
		.fadedOut = { 1.0f, 0.5f, 0.1f, 0.3f },
	},
	{
		.initial  = { 1.0f, 0.67f, 0.1f, 1.0f },
		.fadedIn  = { 0.7f, 0.39f, 0.075f, 1.0f },
		.fadedOut = { 0.7f, 0.39f, 0.075f, 0.3f },
	}
};

static const RgbaLifespan kLavaDustColors[1] {
	{
		.initial  = { 1.0f, 0.67f, 0.0f, 0.00f },
		.fadedIn  = { 1.0f, 0.67f, 0.0f, 0.05f },
		.fadedOut = { 0.5f, 0.3f, 0.3f, 0.00f },
	}
};

void EffectsSystemFacade::spawnLiquidImpactParticleEffect( unsigned delay, const LiquidImpact &impact, float percentageScale,
														   std::pair<float, float> randomRotationAngleCosineRange ) {
	std::variant<Particle::SpriteRules, Particle::SparkRules> dropParticlesGeometryRules = Particle::SpriteRules {
		.radius = { .mean = 1.25f, .spread = 0.25f }, .sizeBehaviour = Particle::ExpandingAndShrinking
	};

	std::span<const RgbaLifespan> splashColors, dropsColors, dustColors;

	shader_s **materials     = nullptr;
	float minDropsPercentage = 0.5f;
	float maxDropsPercentage = 1.0f;

	if( impact.contents & CONTENTS_WATER ) {
		splashColors = kWaterSplashColors;
		dustColors   = kWaterDustColors;
		materials    = cgs.media.shaderLiquidImpactCloud.getAddressOfHandle();
	} else if( impact.contents & CONTENTS_SLIME ) {
		// TODO: We don't actually have slime on default maps, do we?

		splashColors = kSlimeSplashColors,
		dustColors   = kSlimeDustColors,
		materials    = cgs.media.shaderLiquidImpactCloud.getAddressOfHandle();
	} else if( impact.contents & CONTENTS_LAVA ) {
		splashColors = kLavaSplashColors;
		dustColors   = kLavaDustColors;
		dropsColors  = kLavaDropsColors;

		dropParticlesGeometryRules = Particle::SparkRules {
			.length = { .mean = 3.0f }, .width = { .mean = 1.5f }, .sizeBehaviour = Particle::ExpandingAndShrinking
		};

		minDropsPercentage = 0.3f;
		maxDropsPercentage = 0.5f;

		materials = cgs.media.shaderLavaImpactDrop.getAddressOfHandle();
	}

	if( materials ) {
		vec3_t flockDir { impact.burstDir[0], impact.burstDir[1], impact.burstDir[2] };
		const auto [minCosine, maxCosine]  = randomRotationAngleCosineRange;
		const float coneAngleCosine        = Q_Sqrt( m_rng.nextFloat( minCosine, maxCosine ) );
		addRandomRotationToDir( flockDir, &m_rng, coneAngleCosine );

		const FlockOrientation flockOrientation {
			.origin = { impact.origin[0], impact.origin[1], impact.origin[2] },
			.offset = { impact.burstDir[0], impact.burstDir[1], impact.burstDir[2] },
			.dir    = { flockDir[0], flockDir[1], flockDir[2] },
		};

		if( dropsColors.empty() ) {
			dropsColors = splashColors;
		}

		ConicalFlockParams splashFlockParams {
			.gravity    = GRAVITY,
			.angle      = 12,
			.speed      = { .min = 500, .max = 700 },
			.percentage = { .min = 0.7f * percentageScale, .max = 1.0f * percentageScale },
			.timeout    = { .min = 100, .max = 200 },
		};

		const Particle::AppearanceRules splashAppearanceRules {
			.materials     = materials,
			.colors        = splashColors,
			.geometryRules = Particle::SparkRules {
				.length        = { .mean = 40.0f, .spread = 10.0f },
				.width         = { .mean = 4.0f, .spread = 1.0f },
				.sizeBehaviour = Particle::Shrinking,
			},
		};

		ConicalFlockParams dropsFlockParams {
			.gravity     = GRAVITY,
			.drag        = 0.015f,
			.angle       = 15,
			.bounceCount = { .minInclusive = 0, .maxInclusive = 0 },
			.speed       = { .min = 300, .max = 900 },
			.percentage  = { .min = minDropsPercentage * percentageScale, .max = maxDropsPercentage * percentageScale },
			.timeout     = { .min = 350, .max = 700 },
		};

		const Particle::AppearanceRules dropsAppearanceRules {
			.materials     = materials,
			.colors        = dropsColors,
			.geometryRules = dropParticlesGeometryRules,
		};

		ConicalFlockParams dustFlockParams {
			.gravity    = 100.0f,
			.angle      = 7.5f,
			.speed      = { .min = 50, .max = 100 },
			.shiftSpeed = { .min = 450.0f, .max = 550.0f },
			.percentage = { .min = 0.4f * percentageScale, .max = 0.7f * percentageScale },
			.timeout    = { .min = 100, .max = 150 },
		};

		const Particle::AppearanceRules dustAppearanceRules {
			.materials     = materials,
			.colors        = dustColors,
			.geometryRules = Particle::SpriteRules {
				.radius = { .mean = 25.0f, .spread = 7.5f }, .sizeBehaviour = Particle::Expanding,
			},
		};

		flockOrientation.copyToFlockParams( &splashFlockParams );
		spawnOrPostponeImpactParticleEffect( delay, splashFlockParams, splashAppearanceRules );

		flockOrientation.copyToFlockParams( &dropsFlockParams );
		spawnOrPostponeImpactParticleEffect( delay, dropsFlockParams, dropsAppearanceRules,
											 TransientEffectsSystem::ParticleFlockBin::Medium );

		flockOrientation.copyToFlockParams( &dustFlockParams );
		spawnOrPostponeImpactParticleEffect( delay, dustFlockParams, dustAppearanceRules );
	}
}

const EffectsSystemFacade::EventRateLimiterParams EffectsSystemFacade::kLiquidImpactSoundLimiterParams {
	.startDroppingAtDistance = 192.0f,
	.startDroppingAtTimeDiff = 250,
};

const EffectsSystemFacade::EventRateLimiterParams EffectsSystemFacade::kLiquidImpactRingLimiterParams {
	.dropChanceAtZeroDistance = 0.75f,
	.startDroppingAtDistance  = 48.0f,
	.dropChanceAtZeroTimeDiff = 0.75f,
	.startDroppingAtTimeDiff  = 150,
};

void EffectsSystemFacade::spawnBulletLiquidImpactEffect( unsigned delay, const LiquidImpact &impact ) {
	spawnLiquidImpactParticleEffect( delay, impact, 1.0f, { 0.70f, 0.95f } );

	spawnWaterImpactRing( delay, impact.origin );

	if( const unsigned numSfx = cgs.media.sfxImpactWater.length() ) {
		sfx_s *sfx = cgs.media.sfxImpactWater[m_rng.nextBounded( numSfx )];
		startSoundForImpactUsingLimiter( sfx, impact, kLiquidImpactSoundLimiterParams );
	}
}

void EffectsSystemFacade::spawnMultiplePelletImpactEffects( std::span<const SolidImpact> impacts,
															std::span<const unsigned> delays ) {
	assert( impacts.size() == delays.size() );

	[[maybe_unused]] const EventRateLimiterParams limiterParams {
		.startDroppingAtDistance = 144.0f,
		.startDroppingAtTimeDiff = 250,
	};

	if( v_particles.get() ) {
		using IM = SurfImpactMaterial;
		assert( impacts.size() <= 64 );

		uint64_t genericRosetteImpactsMask  = 0;
		uint64_t metalRosetteImpactsMask    = 0;
		uint64_t glassRosetteImpactsMask    = 0;
		unsigned totalNumRosetteImpacts     = 0;
		for( unsigned i = 0; i < impacts.size(); ++i ) {
			const SurfImpactMaterial material = decodeSurfImpactMaterial( impacts[i].surfFlags );
			if( material == IM::Stone || material == IM::Unknown ) {
				genericRosetteImpactsMask |= (uint64_t)1 << i;
				totalNumRosetteImpacts++;
			} else if( material == IM::Metal ) {
				metalRosetteImpactsMask |= (uint64_t)1 << i;
				totalNumRosetteImpacts++;
			} else if( material == IM::Glass ) {
				glassRosetteImpactsMask |= (uint64_t)1 << i;
				totalNumRosetteImpacts++;
			}
		}

		unsigned numRosetteImpactsSoFar = 0;
		for( unsigned i = 0; i < impacts.size(); ++i ) {
			const SolidImpact &impact           = impacts[i];
			const unsigned delay                = delays[i];
			const SurfImpactMaterial material   = decodeSurfImpactMaterial( impact.surfFlags );
			const unsigned materialParam        = decodeSurfImpactMaterialParam( impact.surfFlags );
			const FlockOrientation orientation  = makeRicochetFlockOrientation( impact, &m_rng );

			spawnPelletImpactParticleEffectForMaterial( delay, orientation, material, materialParam,
														numRosetteImpactsSoFar, totalNumRosetteImpacts );

			if( material == SurfImpactMaterial::Glass || material == SurfImpactMaterial::Metal ) {
				spawnBulletLikeImpactRingUsingLimiter( delay, impact );
			}

			const uint64_t maskBit = (uint64_t)1 << i;
			if( maskBit & ( genericRosetteImpactsMask | metalRosetteImpactsMask | glassRosetteImpactsMask ) ) {
				const unsigned lightAffinityIndex = numRosetteImpactsSoFar;
				const unsigned lightAffinityModulo = totalNumRosetteImpacts;

				if( maskBit & genericRosetteImpactsMask ) {
					spawnBulletGenericImpactRosette( delay, orientation, 0.1f, 0.5f, lightAffinityIndex, lightAffinityModulo );
				} else if( maskBit & metalRosetteImpactsMask ) {
					spawnBulletMetalImpactRosette( delay, orientation, 0.1f, 0.5f, lightAffinityIndex, lightAffinityModulo );
				} else {
					spawnBulletGlassImpactRosette( delay, orientation, 0.1f, 0.5f, lightAffinityIndex, lightAffinityModulo );
				}

				// TODO: Postpone if needed
				m_transientEffectsSystem.spawnPelletImpactModel( impact.origin, impact.normal );
				numRosetteImpactsSoFar++;
			}
		}
		for( const SolidImpact &impact: impacts ) {
			const unsigned group     = getImpactSfxGroupForMaterial( decodeSurfImpactMaterial( impact.surfFlags ) );
			const uintptr_t groupTag = group;
			sfx_s *sfx               = getSfxForImpactGroup( group );
			startSoundForImpactUsingLimiter( sfx, groupTag, impact, limiterParams );
		}
	} else {
		for( unsigned i = 0; i < impacts.size(); ++i ) {
			const SolidImpact &impact = impacts[i];
			const unsigned delay      = delays[i];
			const FlockOrientation orientation = makeRicochetFlockOrientation( impact, &m_rng );
			spawnBulletGenericImpactRosette( delay, orientation, 0.1f, 0.5f, i, impacts.size() );
			// TODO: Postpone if needed
			m_transientEffectsSystem.spawnPelletImpactModel( impact.origin, impact.normal );
		}
		if( const unsigned numSfx = cgs.media.sfxImpactSolid.length() ) {
			const auto groupTag   = (uintptr_t)cgs.media.sfxImpactSolid.getAddressOfHandles();
			for( const SolidImpact &impact: impacts ) {
				sfx_s *sfx = cgs.media.sfxImpactSolid[m_rng.nextBounded( numSfx )];
				startSoundForImpactUsingLimiter( sfx, groupTag, impact, limiterParams );
			}
		}
	}
}

void EffectsSystemFacade::spawnMultipleExplosionImpactEffects( std::span<const SolidImpact> impacts ) {
	unsigned totalNumRosetteImpacts = 0;
	for( const SolidImpact &impact: impacts ) {
		// Only metal surfaces produce impact rosettes in case of explosions
		if( decodeSurfImpactMaterial( impact.surfFlags ) == SurfImpactMaterial::Metal ) {
			totalNumRosetteImpacts++;
		}
	}
	unsigned numRosetteImpactsSoFar = 0;
	for( const SolidImpact &impact: impacts ) {
		const SurfImpactMaterial material  = decodeSurfImpactMaterial( impact.surfFlags );
		const FlockOrientation orientation = makeRicochetFlockOrientation( impact, &m_rng );
		const unsigned materialParam       = decodeSurfImpactMaterialParam( impact.surfFlags );
		spawnExplosionImpactParticleEffectForMaterial( orientation, material, materialParam,
													   numRosetteImpactsSoFar, totalNumRosetteImpacts );
		if( material == SurfImpactMaterial::Metal ) {
			numRosetteImpactsSoFar++;
		}
	}
	const EventRateLimiterParams limiterParams {
		.startDroppingAtDistance = 192.0f,
		.startDroppingAtTimeDiff = 333,
	};
	for( const SolidImpact &impact: impacts ) {
		const unsigned group     = getImpactSfxGroupForMaterial( decodeSurfImpactMaterial( impact.surfFlags ) );
		const uintptr_t groupTag = group;
		sfx_s *sfx               = getSfxForImpactGroup( group );
		startSoundForImpactUsingLimiter( sfx, groupTag, impact, limiterParams );
	}
}

void EffectsSystemFacade::spawnMultipleLiquidImpactEffects( std::span<const LiquidImpact> impacts, float percentageScale,
															std::pair<float, float> randomRotationAngleCosineRange,
															std::variant<std::span<const unsigned>,
															std::pair<unsigned, unsigned>> delaysOrDelayRange ) {
	assert( impacts.size() < 64 );
	wsw::StaticVector<unsigned, 64> tmpDelays;
	std::span<const unsigned> chosenDelays;

	if( const auto *delayRange = std::get_if<std::pair<unsigned, unsigned>>( &delaysOrDelayRange ) ) {
		assert( delayRange->first <= delayRange->second );
		if( delayRange->second > 0 && delayRange->first != delayRange->second ) {
			for( const LiquidImpact &impact: impacts ) {
				const unsigned delay = delayRange->first + m_rng.nextBoundedFast( delayRange->second - delayRange->first );
				spawnLiquidImpactParticleEffect( delay, impact, percentageScale, randomRotationAngleCosineRange );
				tmpDelays.push_back( delay );
			}
		} else {
			for( const LiquidImpact &impact: impacts ) {
				spawnLiquidImpactParticleEffect( delayRange->first, impact, percentageScale, randomRotationAngleCosineRange );
				tmpDelays.push_back( delayRange->first );
			}
		}
		chosenDelays = std::span<const unsigned> { tmpDelays.data(), tmpDelays.size() };
	} else if( const auto *individualDelays = std::get_if<std::span<const unsigned>>( &delaysOrDelayRange ) ) {
		assert( individualDelays->size() == impacts.size() );
		for( size_t i = 0; i < impacts.size(); ++i ) {
			spawnLiquidImpactParticleEffect( ( *individualDelays )[i], impacts[i], percentageScale, randomRotationAngleCosineRange );
		}
		chosenDelays = *individualDelays;
	} else [[unlikely]] {
		wsw::failWithRuntimeError( "Unreachable" );
	}

	// It's better to keep loops split for a better instruction cache utilization
	assert( chosenDelays.size() == impacts.size() );
	for( size_t i = 0; i < impacts.size(); ++i ) {
		spawnWaterImpactRing( chosenDelays[i], impacts[i].origin );
	}

	// TODO: Add delays to sounds?
	if( const unsigned numSfx = cgs.media.sfxImpactWater.length() ) {
		for( const LiquidImpact &impact: impacts ) {
			sfx_s *sfx = cgs.media.sfxImpactWater[m_rng.nextBounded( numSfx )];
			startSoundForImpactUsingLimiter( sfx, impact, kLiquidImpactSoundLimiterParams );
		}
	}
}

void EffectsSystemFacade::spawnWaterImpactRing( unsigned delay, const float *origin ) {
	if( m_liquidImpactRingsRateLimiter.acquirePermission( cg.time, origin, kLiquidImpactRingLimiterParams ) ) {
		cg.polyEffectsSystem.spawnSimulatedRing( PolyEffectsSystem::SimulatedRingParams {
			// Hack: Force aligning to Z-axis for now (burst dirs could differ from it).
			// We'd like to support non-horizontal water surfaces in further development.
			.origin     = { origin[0], origin[1], origin[2] },
			.offset     = { 0.0f, 0.0f, 0.1f },
			.axisDir    = { 0.0f, 0.0f, 1.0f },
			.alphaLifespan = ValueLifespan {
				.initial   = 0.0f,
				.fadedIn   = 0.7f,
				.fadedOut  = 0.0f,
				.finishFadingInAtLifetimeFrac = 0.10f,
				.startFadingOutAtLifetimeFrac = 0.15f,
			},
			.innerSpeed              = { 50.0f, 2.5f },
			.outerSpeed              = { 100.0f, 5.0f },
			.lifetime                = 600 + delay,
			.simulationDelay         = delay,
			.movementDuration        = 500,
			.innerTexCoordFrac       = 0.5f,
			.outerTexCoordFrac       = 1.0f,
			.numClipMoveSmoothSteps  = 10,
			.softenOnContact         = false,
			.material                = cgs.media.shaderImpactRing,
		});
	}
}

void EffectsSystemFacade::startSoundForImpactUsingLimiter( sfx_s *sfx, uintptr_t group, const SolidImpact &impact,
														   const EventRateLimiterParams &params ) {
	assert( std::fabs( VectorLengthFast( impact.normal ) - 1.0f ) < 1e-2f );
	if( sfx ) {
		vec3_t soundOrigin;
		VectorAdd( impact.origin, impact.normal, soundOrigin );
		assert( !( CG_PointContents( soundOrigin ) & MASK_SOLID ) );
		if( m_solidImpactSoundsRateLimiter.acquirePermission( cg.time, soundOrigin, group, params ) ) {
			startSound( sfx, soundOrigin );
		}
	}
}

void EffectsSystemFacade::startSoundForImpactUsingLimiter( sfx_s *sfx, const LiquidImpact &impact,
														   const EventRateLimiterParams &params ) {
	if( sfx ) {
		vec3_t soundOrigin;
		VectorAdd( impact.origin, impact.burstDir, soundOrigin );
		assert( !( CG_PointContents( soundOrigin ) & MASK_SOLID ) );
		if( m_liquidImpactSoundsRateLimiter.acquirePermission( cg.time, soundOrigin, params ) ) {
			startSound( sfx, soundOrigin );
		}
	}
}

bool EffectsSystemFacade::EventRateLimiter::acquirePermission( int64_t timestamp, const float *origin,
															   const EventRateLimiterParams &params ) {

	assert( params.startDroppingAtTimeDiff > 0 && params.startDroppingAtDistance > 0.0f );

	// TODO: Supply as an argument/dependency
	int64_t closestTimestamp    = std::numeric_limits<int64_t>::min();
	float closestSquareDistance = std::numeric_limits<float>::max();

	// TODO: Keep entries for different groups in different hash bins?
	const float squareDistanceThreshold = wsw::square( params.startDroppingAtDistance );
	const int64_t minTimestampThreshold = timestamp - params.startDroppingAtTimeDiff;
	for( const Entry &entry: m_entries ) {
		assert( entry.timestamp <= timestamp );
		if( entry.timestamp >= minTimestampThreshold ) {
			const float squareDistance = DistanceSquared( origin, entry.origin );
			// If the entry passes the distance threshold
			if( squareDistance < squareDistanceThreshold ) {
				closestSquareDistance = wsw::min( squareDistance, closestSquareDistance );
				closestTimestamp      = wsw::max( entry.timestamp, closestTimestamp );
			}
		}
	}

	bool result = true;
	if( closestSquareDistance < squareDistanceThreshold ) {
		const float distance     = Q_Sqrt( closestSquareDistance );
		const float distanceFrac = distance * Q_Rcp( params.startDroppingAtDistance );
		assert( distanceFrac > -0.01f && distanceFrac < 1.01f );

		const int64_t timeDiff = timestamp - closestTimestamp;
		const float timeFrac   = (float)timeDiff * Q_Rcp( (float) params.startDroppingAtTimeDiff );
		assert( timeFrac > -0.01f && timeFrac < 1.01f );

		const float dropByDistanceChance = params.dropChanceAtZeroDistance * ( 1.0f - distanceFrac );
		const float dropByTimeDiffChance = params.dropChanceAtZeroTimeDiff * ( 1.0f - timeFrac );

		const float keepByDistanceChance = 1.0f - dropByDistanceChance;
		const float keepByTimeDiffChance = 1.0f - dropByTimeDiffChance;
		const float combinedKeepChance   = wsw::clamp( keepByDistanceChance * keepByTimeDiffChance, 0.0f, 1.0f );

		result = m_rng->tryWithChance( combinedKeepChance );
	}

	if( result ) {
		if( m_entries.full() ) {
			m_entries.pop_front();
		}
		m_entries.emplace_back( Entry { .timestamp = timestamp, .origin = { origin[0], origin[1], origin[2] } } );
	}

	return result;
}

bool EffectsSystemFacade::MultiGroupEventRateLimiter::acquirePermission( int64_t timestamp,
																		 const float *origin, uintptr_t group,
																		 const EventRateLimiterParams &params ) {
	for( Entry &entry: m_entries ) {
		if( entry.group == group ) {
			return entry.limiter.acquirePermission( timestamp, origin, params );
		}
	}

	EventRateLimiter *chosenLimiter = nullptr;
	if( !m_entries.full() ) {
		auto *const entry = new( m_entries.unsafe_grow_back() )Entry( m_rng );
		entry->group      = group;
		chosenLimiter     = &entry->limiter;
	} else {
		Entry *chosenEntry      = nullptr;
		int64_t oldestTimestamp = std::numeric_limits<int64_t>::max();
		for( Entry &entry: m_entries ) {
			if( std::optional<int64_t> maybeTimestamp = entry.limiter.getLastTimestamp() ) {
				if( *maybeTimestamp < oldestTimestamp ) {
					oldestTimestamp = *maybeTimestamp;
					chosenEntry     = std::addressof( entry );
				}
			} else {
				chosenEntry = &entry;
				// We've found a free entry.
				break;
			}
		}
		assert( chosenEntry );
		chosenEntry->group = group;
		chosenEntry->limiter.clear();
		chosenLimiter = &chosenEntry->limiter;
	}

	return chosenLimiter->acquirePermission( timestamp, origin, params );
}

[[maybe_unused]]
static auto adjustTracerOriginForOwner( int owner, const float *givenOrigin, float *adjustedOrigin ) -> std::pair<float, float> {
	// This produces satisfactory results, assuming a reasonable prestep.

	VectorCopy( givenOrigin, adjustedOrigin );
	if( owner == (int)cg.predictedPlayerState.POVnum ) {
		const int handValue     = cgs.demoPlaying ? v_hand.get() : cgs.clientInfo[owner - 1].hand;
		const float handScale   = ( handValue >= 0 && handValue <= 1 ) ? ( handValue ? -1.0f : +1.0f ) : 0.0f;
		const float rightOffset = wsw::clamp( handScale * v_handOffset.get() + v_gunX.get(), -16.0f, +16.0f );
		const float zOffset     = -playerbox_stand_viewheight;

		vec3_t right;
		AngleVectors( cg.predictedPlayerState.viewangles, nullptr, right, nullptr );
		VectorMA( adjustedOrigin, rightOffset, right, adjustedOrigin );

		adjustedOrigin[2] += zOffset;

		return { rightOffset, zOffset };
	}

	return { 0, 0 };
}

auto EffectsSystemFacade::spawnBulletTracer( int owner, const float *from, const float *to ) -> unsigned {
	vec3_t adjustedFrom;
	const auto [rightOffset, zOffset] = adjustTracerOriginForOwner( owner, from, adjustedFrom );

	std::optional<PolyEffectsSystem::TracerParams::AlignForPovParams> alignForPovParams;
	if( owner == (int)cg.predictedPlayerState.POVnum ) {
		constexpr float magicRightOffsetScale = 2.0f, magicZOffsetScale = 0.5f;
		alignForPovParams = PolyEffectsSystem::TracerParams::AlignForPovParams {
			.originRightOffset = magicRightOffsetScale * rightOffset,
			.originZOffset     = magicZOffsetScale * zOffset,
			.povNum            = cg.predictedPlayerState.POVnum,
		};
	}

	const std::optional<unsigned> maybeTimeout = cg.polyEffectsSystem.spawnTracerEffect( adjustedFrom, to, {
		.material           = cgs.media.shaderBulletTracer,
		.alignForPovParams  = alignForPovParams,
		.duration           = 200,
		.prestepDistance    = m_rng.nextFloat( 72.0f, 96.0f ),
		.smoothEdgeDistance = 172.0f,
		.width              = m_rng.nextFloat( 2.0f, 2.5f ),
		.minLength          = m_rng.nextFloat( 80.0f, 108.0f ),
		.distancePercentage = ( owner == (int)cg.predictedPlayerState.POVnum ) ? 0.24f : 0.18f,
		.programLightRadius = 72.0f,
		.coronaLightRadius  = 108.0f,
		.lightColor         = { 0.9f, 0.8f, 1.0f }
	});

	return maybeTimeout.value_or( 0 );
}

void EffectsSystemFacade::spawnPelletTracers( int owner, const float *from, std::span<const vec3_t> to,
											  unsigned *timeoutsBuffer ) {
	vec3_t adjustedFrom;
	adjustTracerOriginForOwner( owner, from, adjustedFrom );

	for( size_t i = 0; i < to.size(); ++i ) {
		const std::optional<unsigned> maybeTimeout = cg.polyEffectsSystem.spawnTracerEffect( adjustedFrom, to[i], {
			.material                 = cgs.media.shaderPelletTracer,
			.duration                 = 125,
			.prestepDistance          = m_rng.nextFloat( 32.0f, 72.0f ),
			.width                    = m_rng.nextFloat( 0.9f, 1.0f ),
			.minLength                = m_rng.nextFloat( 72.0f, 108.0f ),
			.distancePercentage       = 0.18f,
			.color                    = { 1.0f, 0.9f, 0.8f, 1.0f },
			.programLightRadius       = 96.0f,
			.coronaLightRadius        = 192.0f,
			.lightColor               = { 1.0f, 0.9f, 0.8f },
			.lightFrameAffinityModulo = (uint8_t)to.size(),
			.lightFrameAffinityIndex  = (uint8_t)i,
		});

		timeoutsBuffer[i] = maybeTimeout.value_or( 0u );
	}
}

void EffectsSystemFacade::spawnDustImpactEffect( const float *origin, const float *dir, float radius ) {
	if( !( CG_PointContents( origin ) & CONTENTS_WATER ) ) [[likely]] {
		m_transientEffectsSystem.spawnDustImpactEffect( origin, dir, radius );
	}
}

void EffectsSystemFacade::spawnDashEffect( const float *oldOrigin, const float *newOrigin ) {
	vec3_t dir { newOrigin[0] - oldOrigin[0], newOrigin[1] - oldOrigin[1], newOrigin[2] - oldOrigin[2] };
	const float squaredLength2D = dir[0] * dir[0] + dir[1] * dir[1];
	const float length2DThreshold = 6.0f;
	if( squaredLength2D > length2DThreshold * length2DThreshold ) {
		if( !( CG_PointContents( newOrigin ) & CONTENTS_WATER ) ) [[likely]] {
			const vec3_t effectOrigin { newOrigin[0], newOrigin[1], newOrigin[2] + playerbox_stand_mins[2] };
			const float rcpLength = Q_RSqrt( squaredLength2D + dir[2] * dir[2] );
			VectorScale( dir, rcpLength, dir );
			m_transientEffectsSystem.spawnDashEffect( effectOrigin, dir );
		}
	}
}

bool getElectroboltTeamColor( int team, float *color ) {
	if( v_teamColoredBeams.get() && ( ( team == TEAM_ALPHA || team == TEAM_BETA ) ) ) {
		CG_TeamColor( team, color );
		return true;
	}
	return false;
}

void EffectsSystemFacade::spawnElectroboltBeam( const vec3_t start, const vec3_t end, int team ) {
	if( v_ebBeamTime.get() <= 0.0f || v_ebBeamWidth.get() <= 0.0f ) {
		return;
	}

	vec4_t color;
	if( !getElectroboltTeamColor( team, color ) ) {
		Vector4Copy( colorWhite, color );
	}

	struct shader_s *material = cgs.media.shaderElectroBeam;
	if( v_teamColoredBeams.get() && ( team == TEAM_ALPHA || team == TEAM_BETA ) ) {
		if( team == TEAM_ALPHA ) {
			material = cgs.media.shaderElectroBeamAlpha;
		} else {
			material = cgs.media.shaderElectroBeamBeta;
		}
	}

	const auto timeoutSeconds = wsw::clamp( v_ebBeamTime.get(), 0.1f, 1.0f );
	const auto timeoutMillis  = (unsigned)( 1.00f * 1000 * timeoutSeconds );
	const auto lightTimeout   = (unsigned)( 0.25f * 1000 * timeoutSeconds );

	cg.polyEffectsSystem.spawnTransientBeamEffect( start, end, {
		.material          = material,
		.beamColorLifespan = RgbaLifespan {
			.initial  = { 1.0f, 1.0f, 1.0f, color[3] },
			.fadedIn  = { color[0], color[1], color[2], color[3] },
			.fadedOut = { 0.0f, 0.0f, 0.0f, 0.0f },
			.finishFadingInAtLifetimeFrac = 0.2f,
			.startFadingOutAtLifetimeFrac = 0.5f,
		},
		.lightProps        = std::pair<unsigned, LightLifespan> {
			lightTimeout, LightLifespan {
				.colorLifespan = {
					.initial  = { 1.0f, 1.0f, 1.0f },
					.fadedIn  = { color[0], color[1], color[2] },
					.fadedOut = { color[0], color[1], color[2] },
				},
				.radiusLifespan = {
					.initial  = 100.0f,
					.fadedIn  = 250.0f,
					.fadedOut = 100.0f,
				},
			}
		},
		.width      = wsw::clamp( v_ebBeamWidth.get(), 0.0f, 128.0f ),
		.tileLength = 128.0f,
		.timeout    = timeoutMillis,
	});
}

bool getInstagunTeamColor( int team, float *color ) {
	if( v_teamColoredInstaBeams.get() && ( team == TEAM_ALPHA || team == TEAM_BETA ) ) {
		CG_TeamColor( team, color );
		return true;
	}
	return false;
}

void EffectsSystemFacade::spawnInstagunBeam( const vec3_t start, const vec3_t end, int team ) {
	if( v_instaBeamTime.get() <= 0.0f || v_instaBeamWidth.get() <= 0.0f ) {
		return;
	}

	vec4_t color;
	if( !getInstagunTeamColor( team, color ) ) {
		Vector4Set( color, 1.0f, 0.0f, 0.4f, 0.35f );
	}

	const auto timeoutSeconds = wsw::clamp( v_instaBeamTime.get(), 0.1f, 1.0f );
	const auto timeoutMillis  = (unsigned)( 1.00f * 1000 * timeoutSeconds );
	const auto lightTimeout   = (unsigned)( 0.25f * 1000 * timeoutSeconds );

	cg.polyEffectsSystem.spawnTransientBeamEffect( start, end, {
		.material          = cgs.media.shaderInstaBeam,
		.beamColorLifespan = RgbaLifespan {
			.initial  = { 1.0f, 1.0f, 1.0f, color[3] },
			.fadedIn  = { color[0], color[1], color[2], color[3] },
			.fadedOut = { 0.0f, 0.0f, 0.0f, 0.0f },
			.finishFadingInAtLifetimeFrac = 0.2f,
			.startFadingOutAtLifetimeFrac = 0.5f,
		},
		.lightProps        = std::pair<unsigned, LightLifespan> {
			lightTimeout, LightLifespan {
				.colorLifespan = {
					.initial  = { 1.0f, 1.0f, 1.0f },
					.fadedIn  = { color[0], color[1], color[2] },
					.fadedOut = { color[0], color[1], color[2] },
				},
				.radiusLifespan = {
					.initial  = 100.0f,
					.fadedIn  = 250.0f,
					.fadedOut = 100.0f,
				},
			}
		},
		.width      = wsw::clamp( v_instaBeamWidth.get(), 0.0f, 128.0f ),
		.tileLength = 128.0f,
		.timeout    = timeoutMillis,
	});
}

void EffectsSystemFacade::spawnWorldLaserBeam( const float *from, const float *to, float width ) {
	// TODO: Either disable fading out or make it tracked
	const auto timeout = wsw::max( 2u, cgs.snapFrameTime );
	cg.polyEffectsSystem.spawnTransientBeamEffect( from, to, {
		.material      = cgs.media.shaderLaser,
		.timeout       = timeout,
	});
}

void EffectsSystemFacade::spawnGameDebugBeam( const float *from, const float *to, const float *color, int ) {
	// TODO: Utilize the parameter
	cg.polyEffectsSystem.spawnTransientBeamEffect( from, to, {
		.material          = cgs.media.shaderLaser,
		.beamColorLifespan = {
			.initial  = { color[0], color[1], color[2] },
			.fadedIn  = { color[0], color[1], color[2] },
			.fadedOut = { color[0], color[1], color[2] },
		},
		.width             = 8.0f,
		.timeout           = 500u,
	});
}