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

#ifndef WSW_17bd35f8_e886_4cc0_a896_33e286002b87_H
#define WSW_17bd35f8_e886_4cc0_a896_33e286002b87_H

#include "trackedeffectssystem.h"
#include "transienteffectssystem.h"
#include "../qcommon/randomgenerator.h"
#include "../qcommon/wswstaticdeque.h"
#include "../qcommon/smallassocarray.h"

class DrawSceneRequest;
struct sfx_s;
struct trace_s;

struct SolidImpact {
	float origin[3] { 0.0f, 0.0f, 0.0f };
	float normal[3] { 0.0f, 0.0f, +1.0f };
	float incidentDir[3] { 0.0f, 0.0f, -1.0f };
	int surfFlags { 0 };
};

struct LiquidImpact {
	float origin[3] { 0.0f, 0.0f, 0.0f };
	float burstDir[3] { 0.0f, 0.0f, +1.0f };
	int contents { 0 };
};

enum class SurfImpactMaterial : unsigned;

struct FlockOrientation;

class EffectsSystemFacade {
public:
	void spawnRocketExplosionEffect( const float *origin, const float *impactNormal, int mode );
	void spawnGrenadeExplosionEffect( const float *origin, const float *impactNormal, int mode );
	void spawnShockwaveExplosionEffect( const float *origin, const float *impactNormal, int mode );
	void spawnPlasmaExplosionEffect( const float *origin, const float *impactNormal, int mode );
	void spawnGenericExplosionEffect( const float *origin, int mode, float radius );

	void spawnGrenadeBounceEffect( int entNum, int mode );

	void spawnPlayerHitEffect( const float *origin, const float *dir, int damage );

	void spawnElectroboltHitEffect( const float *origin, const float *impactNormal, const float *impactDir,
									bool spawnDecal, int ownerNum );
	void spawnInstagunHitEffect( const float *origin, const float *impactNormal, const float *impactDir,
								 bool spawnDecal, int ownerNum );

	void spawnGunbladeBladeHitEffect( const float *origin, const float *dir );
	void spawnGunbladeBlastHitEffect( const float *origin, const float *dir );

	void spawnBulletImpactEffect( unsigned delay, const SolidImpact &impact );

	void spawnUnderwaterBulletImpactEffect( unsigned delay, const float *origin, const float *normal ) {
		// TODO: Postpone if needed
		m_transientEffectsSystem.spawnBulletImpactModel( origin, normal );
	}

	void spawnUnderwaterPelletImpactEffect( unsigned delay, const float *origin, const float *normal ) {
		// TODO: Postpone if needed
		m_transientEffectsSystem.spawnPelletImpactModel( origin, normal );
	}

	void spawnMultiplePelletImpactEffects( std::span<const SolidImpact> impacts, std::span<const unsigned> delays );

	void spawnBulletLiquidImpactEffect( unsigned delay, const LiquidImpact &impact );

	void spawnMultipleLiquidImpactEffects( std::span<const LiquidImpact> impacts, float percentageScale,
										   std::pair<float, float> randomRotationAngleCosineRange,
										   std::variant<std::span<const unsigned>,
										       std::pair<unsigned, unsigned>> delaysOrDelayRange = std::make_pair( 0u, 0u ) );

	[[nodiscard]]
	auto spawnBulletTracer( int owner, const float *from, const float *to ) -> unsigned;

	void spawnPelletTracers( int owner, const float *from, std::span<const vec3_t> to, unsigned *timeoutsBuffer );

	void spawnLandingDustImpactEffect( const float *origin, const float *dir ) {
		spawnDustImpactEffect( origin, dir, 50.0f );
	}

	void spawnWalljumpDustImpactEffect( const float *origin, const float *dir ) {
		spawnDustImpactEffect( origin, dir, 64.0f );
	}

	void spawnDashEffect( const float *oldOrigin, const float *newOrigin );

	void touchStrongRocketTrail( int entNum, const float *origin, int64_t currTime ) {
		m_trackedEffectsSystem.touchStrongRocketTrail( entNum, origin, currTime );
	}
	void touchWeakRocketTrail( int entNum, const float *origin, int64_t currTime ) {
		m_trackedEffectsSystem.touchWeakRocketTrail( entNum, origin, currTime );
	}
	void touchStrongGrenadeTrail( int entNum, const float *origin, int64_t currTime ) {
		m_trackedEffectsSystem.touchStrongGrenadeTrail( entNum, origin, currTime );
	}
	void touchWeakGrenadeTrail( int entNum, const float *origin, int64_t currTime ) {
		m_trackedEffectsSystem.touchWeakGrenadeTrail( entNum, origin, currTime );
	}
	void touchBlastTrail( int entNum, const float *origin, int64_t currTime ) {
		m_trackedEffectsSystem.touchBlastTrail( entNum, origin, currTime );
	}
	void touchElectroTrail( int entNum, int ownerNum, const float *origin, int64_t currTime ) {
		m_trackedEffectsSystem.touchElectroTrail( entNum, ownerNum, origin, currTime );
	}
	void touchStrongPlasmaTrail( int entNum, const float *origin, int64_t currTime ) {
		m_trackedEffectsSystem.touchStrongPlasmaTrail( entNum, origin, currTime );
	}
	void touchWeakPlasmaTrail( int entNum, const float *origin, int64_t currTime ) {
		m_trackedEffectsSystem.touchWeakPlasmaTrail( entNum, origin, currTime );
	}

	void detachPlayerTrail( int entNum ) {
		m_trackedEffectsSystem.detachPlayerTrail( entNum );
	}

	void touchPlayerTrail( int entNum, const float *origin, int64_t currTime ) {
		m_trackedEffectsSystem.touchPlayerTrail( entNum, origin, currTime );
	}

	void touchCorpseTrail( int entNum, const float *origin, int64_t currTime ) {
		m_trackedEffectsSystem.touchCorpseTrail( entNum, origin, currTime );
	}

	void updateStraightLaserBeam( int entNum, const float *from, const float *to, int64_t currTime ) {
		m_trackedEffectsSystem.updateStraightLaserBeam( entNum, from, to, currTime );
	}

	void updateCurvedLaserBeam( int entNum, std::span<const vec3_t> points, int64_t currTime ) {
		m_trackedEffectsSystem.updateCurvedLaserBeam( entNum, points, currTime );
	}

	void spawnElectroboltBeam( const float *from, const float *to, int team );
	void spawnInstagunBeam( const float *from, const float *to, int team );

	void spawnWorldLaserBeam( const float *from, const float *to, float width );
	void spawnGameDebugBeam( const float *from, const float *to, const float *color, int parm );

	void spawnPlayerTeleInEffect( int entNum, const float *origin, model_s *model ) {
		m_trackedEffectsSystem.spawnPlayerTeleInEffect( entNum, origin, model );
	}
	void spawnPlayerTeleOutEffect( int entNum, const float *origin, model_s *model ) {
		m_trackedEffectsSystem.spawnPlayerTeleOutEffect( entNum, origin, model );
	}

	void resetEntityEffects( int entNum ) { m_trackedEffectsSystem.resetEntityEffects( entNum ); }

	void simulateFrameAndSubmit( int64_t currTime, DrawSceneRequest *drawSceneRequest );
private:
	void startSound( sfx_s *sfx, const float *origin, float attenuation = 1.0f );
	void startRelativeSound( sfx_s *sfx, int entNum, float attenuation = 1.0f );

	void spawnExplosionEffect( const float *origin, const float *dir, sfx_s *sfx, float radius, bool addSoundLfe );

	void spawnDustImpactEffect( const float *origin, const float *dir, float radius );

	void spawnBulletGenericImpactRosette( unsigned delay, const FlockOrientation &orientation,
										  float minPercentage, float maxPercentage,
										  unsigned lightFrameAffinityIndex = 0, unsigned lightFrameAffinityModulo = 0 );

	void spawnBulletMetalImpactRosette( unsigned delay, const FlockOrientation &orientation,
										float minPercentage, float maxPercentage,
										unsigned lightFrameAffinityIndex = 0, unsigned lightFrameAffinityModulo = 0 );

	void spawnBulletGlassImpactRosette( unsigned delay, const FlockOrientation &orientation,
										float minPercentage, float maxPercentage,
										unsigned lightFrameAffinityIndex = 0, unsigned lightFrameAffinityModulo = 0 );

	void spawnBulletImpactDoubleRosette( unsigned delay, const FlockOrientation &orientation,
										 float minPercentage, float maxPercentage,
										 unsigned lightFrameAffinityIndex, unsigned lightFrameAffinityModulo,
										 const RgbaLifespan &startSpikeColorLifespan,
										 const RgbaLifespan &endSpikeColorLifespan );

	// Normally `delay` would have been a last default argument,
	// but there are already fine tune parameters,
	// and packing parameters in a struct adds way too much clutter.

	void spawnBulletMetalRicochetParticles( unsigned delay, const FlockOrientation &orientation, float upShiftScale,
											unsigned materialParam, float minPercentage, float maxPercentage,
											unsigned lightFrameAffinityIndex = 0, unsigned lightFrameAffinityModulo = 0 );

	void spawnBulletMetalDebrisParticles( unsigned delay, const FlockOrientation &orientation, float upShiftScale,
										  unsigned materialParam, float minPercentage, float maxPercentage,
										  unsigned lightFrameAffinityIndex = 0, unsigned lightFrameAffinityModulo = 0 );

	void spawnStoneDustParticles( unsigned delay, const FlockOrientation &orientation, float upShiftScale,
								  unsigned materialParam, float dustPercentageScale = 1.0f );

	void spawnStuccoDustParticles( unsigned delay, const FlockOrientation &orientation,
								   float upShiftScale, unsigned materialParam );

	void spawnWoodBulletImpactParticles( unsigned delay, const FlockOrientation &orientation,
										 float upShiftScale, unsigned materialParam,
										 float debrisPercentageScale = 1.0f );

	void spawnDirtImpactParticles( unsigned delay, const FlockOrientation &orientation,
								   float upShiftScale, unsigned materialParam );

	void spawnSandImpactParticles( unsigned delay, const FlockOrientation &orientation,
								   float upShiftScale, unsigned materialParam, float dustPercentageScale = 1.0f );

	void spawnGlassImpactParticles( unsigned delay, const FlockOrientation &orientation,
									float upShiftScale, unsigned materialParam, float percentageScale );

	void spawnBulletImpactParticleEffectForMaterial( unsigned delay, const FlockOrientation &flockOrientation,
													 SurfImpactMaterial impactMaterial, unsigned materialParam );

	void spawnPelletImpactParticleEffectForMaterial( unsigned delay, const FlockOrientation &flockOrientation,
													 SurfImpactMaterial impactMaterial, unsigned materialParam,
													 unsigned lightFrameAffinityIndex, unsigned lightFrameAffinityModulo );

	// TODO: Hide bins from ParticleSystem public interface
	template <typename FlockParams>
	void spawnOrPostponeImpactParticleEffect( unsigned delay, const FlockParams &flockParams,
											  const Particle::AppearanceRules &appearanceRules,
											  TransientEffectsSystem::ParticleFlockBin bin =
												  TransientEffectsSystem::ParticleFlockBin::Small );

	void spawnOrPostponeImpactRosetteEffect( unsigned delay, PolyEffectsSystem::ImpactRosetteParams &&params );

	void spawnExplosionImpactParticleEffectForMaterial( const FlockOrientation &flockOrientation,
														SurfImpactMaterial impactMaterial, unsigned materialParam,
														unsigned lightFrameAffinityIndex, unsigned lightFrameAffinityModulo );

	[[nodiscard]]
	static auto getImpactSfxGroupForMaterial( SurfImpactMaterial impactMaterial ) -> unsigned;

	[[nodiscard]]
	auto getSfxForImpactGroup( unsigned group ) -> sfx_s *;

	void spawnMultipleExplosionImpactEffects( std::span<const SolidImpact> impacts );

	struct EventRateLimiterParams {
		float dropChanceAtZeroDistance { 1.0f };
		float startDroppingAtDistance { 96.0f };
		float dropChanceAtZeroTimeDiff { 1.0f };
		unsigned startDroppingAtTimeDiff { 100 };
	};

	static const EventRateLimiterParams kLiquidImpactSoundLimiterParams;
	static const EventRateLimiterParams kLiquidImpactRingLimiterParams;

	void spawnBulletLikeImpactRingUsingLimiter( unsigned delay, const SolidImpact &impact );

	void startSoundForImpactUsingLimiter( sfx_s *sfx, uintptr_t groupTag, const SolidImpact &impact,
										  const EventRateLimiterParams &params );
	void startSoundForImpactUsingLimiter( sfx_s *sfx, const LiquidImpact &impact,
										  const EventRateLimiterParams &params );

	void spawnLiquidImpactParticleEffect( unsigned delay, const LiquidImpact &impact, float percentageScale,
										  std::pair<float, float> randomRotationAngleCosineRange );

	TrackedEffectsSystem m_trackedEffectsSystem;
	TransientEffectsSystem m_transientEffectsSystem;
	wsw::RandomGenerator m_rng;

	shader_s *m_bloodMaterials[3];
	wsw::StaticVector<std::pair<sfx_s **, unsigned>, 5> m_impactSfxForGroups;

	class EventRateLimiter {
	public:
		explicit EventRateLimiter( wsw::RandomGenerator *rng ) : m_rng( rng ) {}

		[[nodiscard]]
		bool acquirePermission( int64_t timestamp, const float *origin, const EventRateLimiterParams &params );

		[[nodiscard]]
		auto getLastTimestamp() -> std::optional<int64_t> {
			return !m_entries.empty() ? std::optional( m_entries.back().timestamp ) : std::nullopt;
		}

		void clear() { m_entries.clear(); }
	private:
		wsw::RandomGenerator *const m_rng;

		struct Entry {
			// TODO: pack / use a draining time instead of absolute timestamp?
			int64_t timestamp;
			vec3_t origin;
		};

		// TODO: Make it configurable
		wsw::StaticDeque<Entry, 24> m_entries;
	};

	class MultiGroupEventRateLimiter {
	public:
		explicit MultiGroupEventRateLimiter( wsw::RandomGenerator *rng ) : m_rng( rng ) {}

		[[nodiscard]]
		bool acquirePermission( int64_t timestamp, const float *origin, uintptr_t group,
								const EventRateLimiterParams &params );
	private:
		wsw::RandomGenerator *const m_rng;

		struct Entry {
			uintptr_t group { 0 };
			EventRateLimiter limiter;

			explicit Entry( wsw::RandomGenerator *rng ) : limiter( rng ) {}
		};

		wsw::StaticVector<Entry, 8> m_entries;
	};

	MultiGroupEventRateLimiter m_solidImpactSoundsRateLimiter { &m_rng };
	EventRateLimiter m_liquidImpactSoundsRateLimiter { &m_rng };
	EventRateLimiter m_solidImpactRingsRateLimiter { &m_rng };
	EventRateLimiter m_liquidImpactRingsRateLimiter { &m_rng };
};

#endif