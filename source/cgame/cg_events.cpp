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
#include "cg_local.h"
#include "../client/snd_public.h"

/*
* CG_Event_WeaponBeam
*/
static void CG_Event_WeaponBeam( vec3_t origin, vec3_t dir, int ownerNum, int weapon, int firemode ) {
	gs_weapon_definition_t *weapondef;
	int range;
	vec3_t end;
	trace_t trace;

	switch( weapon ) {
		case WEAP_ELECTROBOLT:
			weapondef = GS_GetWeaponDef( WEAP_ELECTROBOLT );
			range = ELECTROBOLT_RANGE;
			break;

		case WEAP_INSTAGUN:
			weapondef = GS_GetWeaponDef( WEAP_INSTAGUN );
			range = weapondef->firedef.timeout;
			break;

		default:
			return;
	}

	VectorNormalizeFast( dir );

	VectorMA( origin, range, dir, end );

	// retrace to spawn wall impact
	CG_Trace( &trace, origin, vec3_origin, vec3_origin, end, cg.view.POVent, MASK_SOLID );
	if( trace.ent != -1 ) {
		[[maybe_unused]] bool spawnDecal = ( trace.surfFlags & ( SURF_FLESH | SURF_NOMARKS ) ) == 0;
		if( weapondef->weapon_id == WEAP_ELECTROBOLT ) {
			cg.effectsSystem.spawnElectroboltHitEffect( trace.endpos, trace.plane.normal, dir, ownerNum, spawnDecal );
		} else if( weapondef->weapon_id == WEAP_INSTAGUN ) {
			cg.effectsSystem.spawnInstagunHitEffect( trace.endpos, trace.plane.normal, dir, ownerNum, spawnDecal );
		}
	}

	// when it's predicted we have to delay the drawing until the view weapon is calculated
	cg_entities[ownerNum].localEffects[LOCALEFFECT_EV_WEAPONBEAM] = weapon;
	VectorCopy( origin, cg_entities[ownerNum].laserOrigin );
	VectorCopy( trace.endpos, cg_entities[ownerNum].laserPoint );
}

void CG_WeaponBeamEffect( centity_t *cent ) {
	orientation_t projection;

	if( !cent->localEffects[LOCALEFFECT_EV_WEAPONBEAM] ) {
		return;
	}

	// now find the projection source for the beam we will draw
	if( !CG_PModel_GetProjectionSource( cent->current.number, &projection ) ) {
		VectorCopy( cent->laserOrigin, projection.origin );
	}

	if( cent->localEffects[LOCALEFFECT_EV_WEAPONBEAM] == WEAP_ELECTROBOLT ) {
		cg.effectsSystem.spawnElectroboltBeam( projection.origin, cent->laserPoint, cent->current.team );
	} else {
		cg.effectsSystem.spawnInstagunBeam( projection.origin, cent->laserPoint, cent->current.team );
	}

	cent->localEffects[LOCALEFFECT_EV_WEAPONBEAM] = 0;
}

static centity_t *laserOwner = nullptr;
static DrawSceneRequest *laserDrawSceneRequest = nullptr;

static vec_t *_LaserColor( vec4_t color ) {
	Vector4Set( color, 1, 1, 1, 1 );
	if( cg_teamColoredBeams->integer && ( laserOwner != NULL ) && ( laserOwner->current.team == TEAM_ALPHA || laserOwner->current.team == TEAM_BETA ) ) {
		CG_TeamColor( laserOwner->current.team, color );
		AdjustTeamColorValue( color );
	}
	return color;
}

static ParticleColorsForTeamHolder laserImpactParticleColorsHolder {
	.defaultColors = {
		.initial  = { 1.0f, 1.0f, 1.0f, 1.0f },
		.fadedIn  = { 1.0f, 1.0f, 1.0f, 1.0f },
		.fadedOut = { 1.0f, 0.9f, 0.0f, 0.0f },
	}
};

static void _LaserImpact( trace_t *trace, vec3_t dir ) {
	if( !trace || trace->ent < 0 ) {
		return;
	}

	if( laserOwner && !trace->allsolid ) {
		// Track it regardless of cg_particles settings to prevent hacks with toggling the var on/off
		if( laserOwner->localEffects[LOCALEFFECT_LASERBEAM_SMOKE_TRAIL] + 32 <= cg.time ) {
			laserOwner->localEffects[LOCALEFFECT_LASERBEAM_SMOKE_TRAIL] = cg.time;

			if( cg_particles->integer ) {
				bool useTeamColors = false;
				if( cg_teamColoredBeams->integer ) {
					if( const int team = laserOwner->current.team; team == TEAM_ALPHA || team == TEAM_BETA ) {
						useTeamColors = true;
					}
				}

				const RgbaLifespan *singleColorAddress;
				ParticleColorsForTeamHolder *holder = &::laserImpactParticleColorsHolder;
				if( useTeamColors ) {
					vec4_t teamColor;
					const int team = laserOwner->current.team;
					CG_TeamColor( team, teamColor );
					singleColorAddress = holder->getColorsForTeam( team, teamColor );
				} else {
					singleColorAddress = &holder->defaultColors;
				}

				EllipsoidalFlockParams flockParams {
					.origin       = { trace->endpos[0], trace->endpos[1], trace->endpos[2] },
					.offset       = { trace->plane.normal[0], trace->plane.normal[1], trace->plane.normal[2] },
					.stretchDir   = { trace->plane.normal[0], trace->plane.normal[1], trace->plane.normal[2] },
					.stretchScale = 0.5f,
					.gravity      = 0.7f * GRAVITY,
					.vorticity    = 1000.0f,
					.refAxis      = { trace->plane.normal[0], trace->plane.normal[1], trace->plane.normal[2] },
					.speed        = { .min = 150, .max = 200 },
					.shiftSpeed   = { .min = 100, .max = 125 },
					.percentage   = { .min = 1.0f, .max = 1.0f },
					.timeout      = { .min = 250, .max = 500 },
				};
				Particle::AppearanceRules appearanceRules {
					.materials      = cgs.media.shaderLaserImpactParticle.getAddressOfHandle(),
					.colors         = { singleColorAddress, singleColorAddress + 1 },
					.geometryRules  = Particle::SpriteRules {
						.radius = { .mean = 1.25f, .spread = 0.25f }, .sizeBehaviour = Particle::Shrinking
					},
				};
				cg.particleSystem.addSmallParticleFlock( appearanceRules, flockParams );
			}

			SoundSystem::instance()->startFixedSound( cgs.media.sfxLasergunHit[rand() % 3], trace->endpos, CHAN_AUTO,
													  cg_volume_effects->value, ATTN_STATIC );
		}
#undef TRAILTIME
	}

	vec3_t lightOrigin;
	// Offset the light origin from the impact surface
	VectorMA( trace->endpos, 4.0f, trace->plane.normal, lightOrigin );

	// it's a brush model
	if( trace->ent == 0 || !( cg_entities[trace->ent].current.effects & EF_TAKEDAMAGE ) ) {
		vec4_t color;
		CG_LaserGunImpact( trace->endpos, trace->plane.normal, 15.0f, dir, _LaserColor( color ), laserDrawSceneRequest );
	} else {
		// it's a player
		// TODO: add player-impact model
	}

	laserDrawSceneRequest->addLight( lightOrigin, 144.0f, 0.0f, 0.75f, 0.75f, 0.375f );
}

void CG_LaserBeamEffect( centity_t *owner, DrawSceneRequest *drawSceneRequest ) {
	const signed ownerEntNum = owner->current.number;
	const bool isOwnerThePov = ISVIEWERENTITY( ownerEntNum );
	const bool isCurved      = owner->laserCurved;
	auto *const soundSystem  = SoundSystem::instance();

	// TODO: Move the entire handling of lasers to the effects system and get rid of this state
	if( owner->localEffects[LOCALEFFECT_LASERBEAM] <= cg.time ) {
		if( owner->localEffects[LOCALEFFECT_LASERBEAM] ) {
			sfx_s *sound = isCurved ? cgs.media.sfxLasergunWeakStop : cgs.media.sfxLasergunStrongStop;
			if( isOwnerThePov ) {
				soundSystem->startGlobalSound( sound, CHAN_AUTO, cg_volume_effects->value );
			} else {
				soundSystem->startRelativeSound( sound, ownerEntNum, CHAN_AUTO, cg_volume_effects->value, ATTN_NORM );
			}
		}
		owner->localEffects[LOCALEFFECT_LASERBEAM] = 0;
		return;
	}

	vec3_t laserOrigin, laserAngles, laserPoint;
	if( isOwnerThePov && !cg.view.thirdperson ) {
		VectorCopy( cg.predictedPlayerState.pmove.origin, laserOrigin );
		laserOrigin[2] += cg.predictedPlayerState.viewheight;
		VectorCopy( cg.predictedPlayerState.viewangles, laserAngles );

		VectorLerp( owner->laserPointOld, cg.lerpfrac, owner->laserPoint, laserPoint );
	} else {
		VectorLerp( owner->laserOriginOld, cg.lerpfrac, owner->laserOrigin, laserOrigin );
		VectorLerp( owner->laserPointOld, cg.lerpfrac, owner->laserPoint, laserPoint );
		if( isCurved ) {
			// Use player entity angles
			for( int i = 0; i < 3; i++ ) {
				laserAngles[i] = LerpAngle( owner->prev.angles[i], owner->current.angles[i], cg.lerpfrac );
			}
		} else {
			// Make up the angles from the start and end points (s->angles is not so precise)
			vec3_t dir;
			VectorSubtract( laserPoint, laserOrigin, dir );
			VecToAngles( dir, laserAngles );
		}
	}

	// draw the beam: for drawing we use the weapon projection source (already handles the case of viewer entity)
	orientation_t projectsource;
	if( !CG_PModel_GetProjectionSource( ownerEntNum, &projectsource ) ) {
		VectorCopy( laserOrigin, projectsource.origin );
	}

	laserOwner = owner;
	laserDrawSceneRequest = drawSceneRequest;

	if( isCurved ) {
		vec3_t from, dir, blendPoint, blendAngles;
		// we redraw the full beam again, and trace each segment for stop dead impact
		VectorCopy( laserPoint, blendPoint );
		VectorCopy( projectsource.origin, from );
		VectorSubtract( blendPoint, projectsource.origin, dir );
		VecToAngles( dir, blendAngles );

		vec3_t points[MAX_CURVELASERBEAM_SUBDIVISIONS + 1];
		VectorCopy( from, points[0] );
		size_t numAddedPoints = 1;

		int passthrough             = ownerEntNum;
		const auto range            = (float)GS_GetWeaponDef( WEAP_LASERGUN )->firedef_weak.timeout;
		const int minSubdivisions   = CURVELASERBEAM_SUBDIVISIONS;
		const int maxSubdivisions   = MAX_CURVELASERBEAM_SUBDIVISIONS;
		const int subdivisions      = wsw::clamp( cg_laserBeamSubdivisions->integer, minSubdivisions, maxSubdivisions );
		const float rcpSubdivisions = Q_Rcp( (float)subdivisions );
		for( int segmentNum = 0; segmentNum < subdivisions; segmentNum++ ) {
			const auto frac = (float)( segmentNum + 1 ) * rcpSubdivisions;

			vec3_t tmpangles;
			for( int j = 0; j < 3; j++ ) {
				tmpangles[j] = LerpAngle( laserAngles[j], blendAngles[j], frac );
			}

			vec3_t end;
			AngleVectors( tmpangles, dir, nullptr, nullptr );
			VectorMA( projectsource.origin, range * frac, dir, end );

			float *const addedPoint = points[numAddedPoints++];

			trace_t trace;
			GS_TraceLaserBeam( &trace, from, tmpangles, DistanceFast( from, end ), passthrough, 0, _LaserImpact );
			VectorCopy( trace.endpos, addedPoint );

			if( trace.fraction != 1.0f ) {
				VectorCopy( trace.endpos, addedPoint );
				break;
			}

			passthrough = trace.ent;
			VectorCopy( trace.endpos, from );
		}

		std::span<const vec3_t> pointsSpan( points, numAddedPoints );
		cg.effectsSystem.updateCurvedLaserBeam( ownerEntNum, pointsSpan, cg.time );
	} else {
		const auto range = (float)GS_GetWeaponDef( WEAP_LASERGUN )->firedef.timeout;

		trace_t trace;
		// trace the beam: for tracing we use the real beam origin
		GS_TraceLaserBeam( &trace, laserOrigin, laserAngles, range, ownerEntNum, 0, _LaserImpact );

		cg.effectsSystem.updateStraightLaserBeam( ownerEntNum, projectsource.origin, trace.endpos, cg.time );
	}

	// enable continuous flash on the weapon owner
	if( cg_weaponFlashes->integer ) {
		cg_entPModels[ownerEntNum].flash_time = cg.time + CG_GetWeaponInfo( WEAP_LASERGUN )->flashTime;
	}

	sfx_s *sound;
	if( isCurved ) {
		sound = owner->current.effects & EF_QUAD ? cgs.media.sfxLasergunWeakQuadHum : cgs.media.sfxLasergunWeakHum;
	} else {
		sound = owner->current.effects & EF_QUAD ? cgs.media.sfxLasergunStrongQuadHum : cgs.media.sfxLasergunStrongHum;
	}

	if( sound ) {
		const float attenuation = isOwnerThePov ? ATTN_NONE : ATTN_STATIC;
		// Tokens in range [1, MAX_EDICTS] are reserved for generic server-sent attachments
		const uintptr_t loopIdentifyingToken = ownerEntNum + MAX_EDICTS;
		soundSystem->addLoopSound( sound, ownerEntNum, loopIdentifyingToken, cg_volume_effects->value, attenuation );
	}

	laserOwner = nullptr;
	laserDrawSceneRequest = nullptr;
}

void CG_Event_LaserBeam( int entNum, int weapon, int fireMode ) {
	centity_t *cent = &cg_entities[entNum];
	unsigned int timeout;
	vec3_t dir;

	if( !cg_predictLaserBeam->integer ) {
		return;
	}

	// lasergun's smooth refire
	if( fireMode == FIRE_MODE_STRONG ) {
		cent->laserCurved = false;
		timeout = GS_GetWeaponDef( WEAP_LASERGUN )->firedef.reload_time + 10;

		// find destiny point
		VectorCopy( cg.predictedPlayerState.pmove.origin, cent->laserOrigin );
		cent->laserOrigin[2] += cg.predictedPlayerState.viewheight;
		AngleVectors( cg.predictedPlayerState.viewangles, dir, NULL, NULL );
		VectorMA( cent->laserOrigin, GS_GetWeaponDef( WEAP_LASERGUN )->firedef.timeout, dir, cent->laserPoint );
	} else {
		cent->laserCurved = true;
		timeout = GS_GetWeaponDef( WEAP_LASERGUN )->firedef_weak.reload_time + 10;

		// find destiny point
		VectorCopy( cg.predictedPlayerState.pmove.origin, cent->laserOrigin );
		cent->laserOrigin[2] += cg.predictedPlayerState.viewheight;
		if( !G_GetLaserbeamPoint( &cg.weaklaserTrail, &cg.predictedPlayerState, cg.predictingTimeStamp, cent->laserPoint ) ) {
			AngleVectors( cg.predictedPlayerState.viewangles, dir, NULL, NULL );
			VectorMA( cent->laserOrigin, GS_GetWeaponDef( WEAP_LASERGUN )->firedef.timeout, dir, cent->laserPoint );
		}
	}

	// it appears that 64ms is that maximum allowed time interval between prediction events on localhost
	if( timeout < 65 ) {
		timeout = 65;
	}

	VectorCopy( cent->laserOrigin, cent->laserOriginOld );
	VectorCopy( cent->laserPoint, cent->laserPointOld );
	cent->localEffects[LOCALEFFECT_LASERBEAM] = cg.time + timeout;
}

/*
* CG_FireWeaponEvent
*/
static void CG_FireWeaponEvent( int entNum, int weapon, int fireMode ) {
	float attenuation;
	struct sfx_s *sound = NULL;
	weaponinfo_t *weaponInfo;

	if( !weapon ) {
		return;
	}

	// hack idle attenuation on the plasmagun to reduce sound flood on the scene
	if( weapon == WEAP_PLASMAGUN ) {
		attenuation = ATTN_IDLE;
	} else {
		attenuation = ATTN_NORM;
	}

	weaponInfo = CG_GetWeaponInfo( weapon );

	// sound
	if( fireMode == FIRE_MODE_STRONG ) {
		if( weaponInfo->num_strongfire_sounds ) {
			sound = weaponInfo->sound_strongfire[(int)brandom( 0, weaponInfo->num_strongfire_sounds )];
		}
	} else {
		if( weaponInfo->num_fire_sounds ) {
			sound = weaponInfo->sound_fire[(int)brandom( 0, weaponInfo->num_fire_sounds )];
		}
	}

	if( sound ) {
		if( ISVIEWERENTITY( entNum ) ) {
			SoundSystem::instance()->startGlobalSound( sound, CHAN_AUTO, cg_volume_effects->value );
		} else {
			SoundSystem::instance()->startRelativeSound( sound, entNum, CHAN_AUTO, cg_volume_effects->value, attenuation );
		}

		if( ( cg_entities[entNum].current.effects & EF_QUAD ) && ( weapon != WEAP_LASERGUN ) ) {
			struct sfx_s *quadSfx = cgs.media.sfxQuadFireSound;
			if( ISVIEWERENTITY( entNum ) ) {
				SoundSystem::instance()->startGlobalSound( quadSfx, CHAN_AUTO, cg_volume_effects->value );
			} else {
				SoundSystem::instance()->startRelativeSound( quadSfx, entNum, CHAN_AUTO, cg_volume_effects->value, attenuation );
			}
		}
	}

	// flash and barrel effects

	if( weapon == WEAP_GUNBLADE ) { // gunblade is special
		if( fireMode == FIRE_MODE_STRONG ) {
			// light flash
			if( cg_weaponFlashes->integer && weaponInfo->flashTime ) {
				cg_entPModels[entNum].flash_time = cg.time + weaponInfo->flashTime;
			}
		} else {
			// start barrel rotation or offsetting
			if( weaponInfo->barrelTime ) {
				cg_entPModels[entNum].barrel_time = cg.time + weaponInfo->barrelTime;
			}
		}
	} else {
		// light flash
		if( cg_weaponFlashes->integer && weaponInfo->flashTime ) {
			cg_entPModels[entNum].flash_time = cg.time + weaponInfo->flashTime;
		}

		// start barrel, belt rotation or offsetting
		if( weaponInfo->barrelTime ) {
			cg_entPModels[entNum].barrel_time = cg.time + weaponInfo->barrelTime;
		}
		if( weaponInfo->beltTime ) {
			cg_entPModels[entNum].fire_time = cg.time;
			cg_entPModels[entNum].belt_time = cg.time + weaponInfo->beltTime; // beltTime should be converted to an int to stop arithmetic overflow
		}
		if( weaponInfo->emitAtFire ) {
			cg_entPModels[entNum].emitter_amount = weaponInfo->emitAmount;
			cg_entPModels[entNum].emitter_time = 0;
		}
	}

	// add animation to the player model
	switch( weapon ) {
		case WEAP_NONE:
			break;

		case WEAP_GUNBLADE:
			if( fireMode == FIRE_MODE_WEAK ) {
				CG_PModel_AddAnimation( entNum, 0, TORSO_SHOOT_BLADE, 0, EVENT_CHANNEL );
			} else {
				CG_PModel_AddAnimation( entNum, 0, TORSO_SHOOT_PISTOL, 0, EVENT_CHANNEL );
			}
			break;

		case WEAP_LASERGUN:
			CG_PModel_AddAnimation( entNum, 0, TORSO_SHOOT_PISTOL, 0, EVENT_CHANNEL );
			break;

		default:
		case WEAP_RIOTGUN:
		case WEAP_PLASMAGUN:
			CG_PModel_AddAnimation( entNum, 0, TORSO_SHOOT_LIGHTWEAPON, 0, EVENT_CHANNEL );
			break;

		case WEAP_ROCKETLAUNCHER:
		case WEAP_GRENADELAUNCHER:
			CG_PModel_AddAnimation( entNum, 0, TORSO_SHOOT_HEAVYWEAPON, 0, EVENT_CHANNEL );
			break;

		case WEAP_ELECTROBOLT:
			CG_PModel_AddAnimation( entNum, 0, TORSO_SHOOT_AIMWEAPON, 0, EVENT_CHANNEL );
			break;
	}

	// add animation to the view weapon model
	if( ISVIEWERENTITY( entNum ) && !cg.view.thirdperson ) {
		CG_ViewWeapon_StartAnimationEvent( fireMode == FIRE_MODE_STRONG ? WEAPMODEL_ATTACK_STRONG : WEAPMODEL_ATTACK_WEAK );
	}
}

[[nodiscard]]
static bool canShowBulletImpactForDirAndTrace( const float *incidentDir, const trace_t &trace ) {
	if( trace.allsolid ) {
		return false;
	}
	if( trace.surfFlags & ( SURF_NOIMPACT | SURF_FLESH ) ) {
		return false;
	}
	// Wtf how does it happen
	if( DotProduct( trace.plane.normal, incidentDir ) > 0 ) {
		return false;
	}
	const auto entNum = trace.ent;
	if( entNum < 0 ) {
		return false;
	}
	if( const auto entType = cg_entities[entNum].current.type; entType == ET_PLAYER || entType == ET_CORPSE ) {
		return false;
	}
	return true;
}

auto getSurfFlagsForImpact( const trace_t &trace, const float *impactDir ) -> int {
	// Hacks
	// TODO: Trace against brush submodels as well
	if( trace.shaderNum == cgs.fullclipShaderNum ) {
		VisualTrace visualTrace {};
		vec3_t testPoint;

		// Check behind
		VectorMA( trace.endpos, +4.0f, impactDir, testPoint );
		wsw::ref::traceAgainstBspWorld( &visualTrace, trace.endpos, testPoint );
		if( visualTrace.fraction != 1.0f ) {
			return visualTrace.surfFlags;
		}

		// Check in front
		VectorMA( trace.endpos, -4.0f, impactDir, testPoint );
		wsw::ref::traceAgainstBspWorld( &visualTrace, testPoint, trace.endpos );
		if( visualTrace.fraction != 1.0f ) {
			return visualTrace.surfFlags;
		}
	}

	return trace.surfFlags;
}

static void CG_Event_FireMachinegun( vec3_t origin, vec3_t dir, int weapon, int firemode, int seed, int owner ) {
	const auto *weaponDef = GS_GetWeaponDef( weapon );
	const auto *fireDef   = firemode ? &weaponDef->firedef : &weaponDef->firedef_weak;

	// circle shape
	const float alpha = M_PI * Q_crandom( &seed ); // [-PI ..+PI]
	const float s     = fabs( Q_crandom( &seed ) ); // [0..1]
	const float r     = s * (float)fireDef->spread * std::cos( alpha );
	const float u     = s * (float)fireDef->v_spread * std::sin( alpha );

	VectorNormalizeFast( dir );

	trace_t trace;

	[[maybe_unused]]
	const trace_t *waterTrace = GS_TraceBullet( &trace, origin, dir, r, u, (int)fireDef->timeout, owner, 0 );
	if( waterTrace ) {
		[[maybe_unused]] const unsigned delay = cg.effectsSystem.spawnBulletTracer( owner, origin, waterTrace->endpos );

		if( canShowBulletImpactForDirAndTrace( dir, trace ) ) {
			cg.effectsSystem.spawnUnderwaterBulletImpactEffect( delay, trace.endpos, trace.plane.normal );
		}

		if( !VectorCompare( waterTrace->endpos, origin ) ) {
			cg.effectsSystem.spawnBulletLiquidImpactEffect( delay, LiquidImpact {
				.origin   = { waterTrace->endpos[0], waterTrace->endpos[1], waterTrace->endpos[2] },
				.burstDir = { waterTrace->plane.normal[0], waterTrace->plane.normal[1], waterTrace->plane.normal[2] },
				.contents = waterTrace->contents,
			});
		}
	} else {
		[[maybe_unused]] const unsigned delay = cg.effectsSystem.spawnBulletTracer( owner, origin, trace.endpos );
		if( canShowBulletImpactForDirAndTrace( dir, trace ) ) {
			cg.effectsSystem.spawnBulletImpactEffect( delay, SolidImpact {
				.origin      = { trace.endpos[0], trace.endpos[1], trace.endpos[2] },
				.normal      = { trace.plane.normal[0], trace.plane.normal[1], trace.plane.normal[2] },
				.incidentDir = { dir[0], dir[1], dir[2] },
				.surfFlags   = getSurfFlagsForImpact( trace, dir ),
			});
		}
	}
}

static void CG_Fire_SunflowerPattern( vec3_t start, vec3_t dir, int *seed, int owner, int count, vec2_t offset,
									  int hspread, int vspread, int range ) {
	assert( seed );
	assert( count > 0 && count < 64 );
	assert( std::abs( VectorLengthFast( dir ) - 1.0f ) < 0.001f );

	auto *const solidImpacts  = (SolidImpact *)alloca( sizeof( SolidImpact ) * count );
	auto *const liquidImpacts = (LiquidImpact *)alloca( sizeof( LiquidImpact ) * count );
	auto *const tracerTargets = (vec3_t *)alloca( sizeof( vec3_t ) * count );

	auto *const underwaterImpactOrigins       = (vec3_t *)alloca( sizeof( vec3_t ) * count );
	auto *const underwaterImpactNormals       = (vec3_t *)alloca( sizeof( vec3_t ) * count );
	auto *const underwaterImpactTracerIndices = (int *)alloca( sizeof( int ) * count );

	auto *const solidImpactTracerIndices  = (unsigned *)alloca( sizeof( unsigned ) * count );
	auto *const liquidImpactTracerIndices = (unsigned *)alloca( sizeof( unsigned ) * count );

	unsigned numSolidImpacts = 0, numLiquidImpacts = 0, numUnderwaterImpacts = 0, numTracerTargets = 0;

	for( int i = 0; i < count; i++ ) {
		// TODO: Is this correct?
		const float phi = 1.3f * (float)i; // magic value creating Fibonacci numbers
		const float sqrtPhi = std::sqrt( phi );

		// TODO: Is this correct?
		const float r = std::cos( (float)*seed + phi ) * (float)hspread * sqrtPhi + offset[0];
		const float u = std::sin( (float)*seed + phi ) * (float)vspread * sqrtPhi + offset[1];

		trace_t trace;
		const trace_t *waterTrace = GS_TraceBullet( &trace, start, dir, r, u, range, owner, 0 );
		if( waterTrace ) {
			const bool shouldShowUnderwaterImpact = canShowBulletImpactForDirAndTrace( dir, trace );
			if( shouldShowUnderwaterImpact ) {
				// We don't know the delay yet
				VectorCopy( trace.endpos, underwaterImpactOrigins[numUnderwaterImpacts] );
				VectorCopy( trace.plane.normal, underwaterImpactNormals[numUnderwaterImpacts] );
			}
			if( !VectorCompare( waterTrace->endpos, start ) ) {
				liquidImpacts[numLiquidImpacts] = LiquidImpact{
					.origin = { waterTrace->endpos[0], waterTrace->endpos[1], waterTrace->endpos[2] },
					.burstDir = { waterTrace->plane.normal[0], waterTrace->plane.normal[1], waterTrace->plane.normal[2] },
					.contents = waterTrace->contents,
				};
				liquidImpactTracerIndices[numLiquidImpacts] = i;
				++numLiquidImpacts;
				if( shouldShowUnderwaterImpact ) {
					underwaterImpactTracerIndices[numUnderwaterImpacts] = (int)i;
				}
			} else {
				if( shouldShowUnderwaterImpact ) {
					underwaterImpactTracerIndices[numUnderwaterImpacts] = -1;
				}
			}
			if( shouldShowUnderwaterImpact ) {
				numUnderwaterImpacts++;
			}
			VectorCopy( waterTrace->endpos, tracerTargets[numTracerTargets] );
			numTracerTargets++;
		} else {
			if( canShowBulletImpactForDirAndTrace( dir, trace ) ) {
				solidImpacts[numSolidImpacts] = SolidImpact{
					.origin = { trace.endpos[0], trace.endpos[1], trace.endpos[2] },
					.normal = { trace.plane.normal[0], trace.plane.normal[1], trace.plane.normal[2] },
					.incidentDir = { dir[0], dir[1], dir[2] },
					.surfFlags = getSurfFlagsForImpact( trace, dir ),
				};
				solidImpactTracerIndices[numSolidImpacts] = i;
				numSolidImpacts++;
			}
			VectorCopy( trace.endpos, tracerTargets[numTracerTargets] );
			numTracerTargets++;
		}
	}

	auto *const tracerDelaysBuffer = (unsigned *)alloca( sizeof( unsigned ) * count );
	auto *const liquidImpactDelays = (unsigned *)alloca( sizeof( unsigned ) * count );
	auto *const solidImpactDelays  = (unsigned *)alloca( sizeof( unsigned ) * count );

	// TODO: Pass the origin stride plus impacts?
	cg.effectsSystem.spawnPelletTracers( owner, start, { tracerTargets, numTracerTargets }, tracerDelaysBuffer );

	for( unsigned i = 0; i < numSolidImpacts; ++i ) {
		solidImpactDelays[i] = tracerDelaysBuffer[solidImpactTracerIndices[i]];
	}
	for( unsigned i = 0; i < numLiquidImpacts; ++i ) {
		liquidImpactDelays[i] = tracerDelaysBuffer[liquidImpactTracerIndices[i]];
	}

	for( unsigned i = 0; i < numUnderwaterImpacts; ++i ) {
		unsigned delay = 0;
		if( const int tracerIndex = underwaterImpactTracerIndices[i]; tracerIndex >= 0 ) {
			// Using the tracer delay is not really correct as there's underwater added distance but this issue is minor
			delay = tracerDelaysBuffer[tracerIndex];
		}
		cg.effectsSystem.spawnUnderwaterPelletImpactEffect( delay, underwaterImpactOrigins[i], underwaterImpactNormals[i] );
	}

	cg.effectsSystem.spawnMultiplePelletImpactEffects( { solidImpacts, numSolidImpacts }, { solidImpactDelays, numSolidImpacts } );

	cg.effectsSystem.spawnMultipleLiquidImpactEffects( { liquidImpacts, numLiquidImpacts }, 0.1f, { 0.3f, 0.9f },
													   std::span<const unsigned> { liquidImpactDelays, numLiquidImpacts } );
}

static void CG_Event_FireRiotgun( vec3_t origin, vec3_t dirVec, int weapon, int firemode, int seed, int owner ) {
	vec3_t dir;
	VectorCopy( dirVec, dir );
	VectorNormalizeFast( dir );

	gs_weapon_definition_t *weapondef = GS_GetWeaponDef( weapon );
	firedef_t *firedef = ( firemode ) ? &weapondef->firedef : &weapondef->firedef_weak;

	Com_Printf( "spreads: %i %i", firedef->spread, firedef->v_spread );

	const int isOdd = ( firedef->usage_count % 2 != 0 ) ? 1 : 0;
	const int offsetDist = 400;
	for( int i = 0; i < firedef->usage_count; i++ ) {
		vec2_t offset;
		
		if( firedef->usage_count == 1 ) {
			offset[0] = 0;
			offset[1] = 0;
		} else {
			offset[0] = offsetDist * std::cos( 2 * M_PI / firedef->usage_count * i + isOdd * M_PI / 2 );
			offset[1] = offsetDist * std::sin( 2 * M_PI / firedef->usage_count * i + isOdd * M_PI / 2 );
		}
		CG_Fire_SunflowerPattern( origin, dir, &seed, owner, firedef->projectile_count / firedef->usage_count, offset, firedef->spread, firedef->v_spread, firedef->timeout );
	}
}


//==================================================================

//=========================================================
#define CG_MAX_ANNOUNCER_EVENTS 32
#define CG_MAX_ANNOUNCER_EVENTS_MASK ( CG_MAX_ANNOUNCER_EVENTS - 1 )
#define CG_ANNOUNCER_EVENTS_FRAMETIME 1500 // the announcer will speak each 1.5 seconds
typedef struct cg_announcerevent_s
{
	struct sfx_s *sound;
} cg_announcerevent_t;
cg_announcerevent_t cg_announcerEvents[CG_MAX_ANNOUNCER_EVENTS];
static int cg_announcerEventsCurrent = 0;
static int cg_announcerEventsHead = 0;
static int cg_announcerEventsDelay = 0;

/*
* CG_ClearAnnouncerEvents
*/
void CG_ClearAnnouncerEvents( void ) {
	cg_announcerEventsCurrent = cg_announcerEventsHead = 0;
}

/*
* CG_AddAnnouncerEvent
*/
void CG_AddAnnouncerEvent( struct sfx_s *sound, bool queued ) {
	if( !sound ) {
		return;
	}

	if( !queued ) {
		SoundSystem::instance()->startLocalSound( sound, cg_volume_announcer->value );
		cg_announcerEventsDelay = CG_ANNOUNCER_EVENTS_FRAMETIME; // wait
		return;
	}

	if( cg_announcerEventsCurrent + CG_MAX_ANNOUNCER_EVENTS >= cg_announcerEventsHead ) {
		// full buffer (we do nothing, just let it overwrite the oldest
	}

	// add it
	cg_announcerEvents[cg_announcerEventsHead & CG_MAX_ANNOUNCER_EVENTS_MASK].sound = sound;
	cg_announcerEventsHead++;
}

/*
* CG_ReleaseAnnouncerEvents
*/
void CG_ReleaseAnnouncerEvents( void ) {
	// see if enough time has passed
	cg_announcerEventsDelay -= cg.realFrameTime;
	if( cg_announcerEventsDelay > 0 ) {
		return;
	}

	if( cg_announcerEventsCurrent < cg_announcerEventsHead ) {
		struct sfx_s *sound;

		// play the event
		sound = cg_announcerEvents[cg_announcerEventsCurrent & CG_MAX_ANNOUNCER_EVENTS_MASK].sound;
		if( sound ) {
			SoundSystem::instance()->startLocalSound( sound, cg_volume_announcer->value );
			cg_announcerEventsDelay = CG_ANNOUNCER_EVENTS_FRAMETIME; // wait
		}
		cg_announcerEventsCurrent++;
	} else {
		cg_announcerEventsDelay = 0; // no wait
	}
}

//==================================================================

//==================================================================

/*
* CG_Event_Fall
*/
void CG_Event_Fall( entity_state_t *state, int parm ) {
	if( ISVIEWERENTITY( state->number ) ) {
		if( cg.frame.playerState.pmove.pm_type != PM_NORMAL ) {
			CG_SexedSound( state->number, CHAN_AUTO, "*fall_0", cg_volume_players->value, state->attenuation );
			return;
		}

		CG_StartFallKickEffect( ( parm + 5 ) * 10 );

		if( parm >= 15 ) {
			CG_DamageIndicatorAdd( parm, tv( 0, 0, 1 ) );
		}
	}

	if( parm > 10 ) {
		CG_SexedSound( state->number, CHAN_PAIN, "*fall_2", cg_volume_players->value, state->attenuation );
		switch( (int)brandom( 0, 3 ) ) {
			case 0:
				CG_PModel_AddAnimation( state->number, 0, TORSO_PAIN1, 0, EVENT_CHANNEL );
				break;
			case 1:
				CG_PModel_AddAnimation( state->number, 0, TORSO_PAIN2, 0, EVENT_CHANNEL );
				break;
			case 2:
			default:
				CG_PModel_AddAnimation( state->number, 0, TORSO_PAIN3, 0, EVENT_CHANNEL );
				break;
		}
	} else if( parm > 0 ) {
		CG_SexedSound( state->number, CHAN_PAIN, "*fall_1", cg_volume_players->value, state->attenuation );
	} else {
		CG_SexedSound( state->number, CHAN_PAIN, "*fall_0", cg_volume_players->value, state->attenuation );
	}

	// smoke effect
	if( parm > 0 && ( cg_cartoonEffects->integer & 2 ) ) {
		vec3_t start, end;
		trace_t trace;

		if( ISVIEWERENTITY( state->number ) ) {
			VectorCopy( cg.predictedPlayerState.pmove.origin, start );
		} else {
			VectorCopy( state->origin, start );
		}

		VectorCopy( start, end );
		end[2] += playerbox_stand_mins[2] - 48.0f;

		CG_Trace( &trace, start, vec3_origin, vec3_origin, end, state->number, MASK_PLAYERSOLID );
		if( trace.ent == -1 ) {
			start[2] += playerbox_stand_mins[2] + 8;
			cg.effectsSystem.spawnLandingDustImpactEffect( start, tv( 0, 0, 1 ) );
		} else if( !( trace.surfFlags & SURF_NODAMAGE ) ) {
			VectorMA( trace.endpos, 8, trace.plane.normal, end );
			cg.effectsSystem.spawnLandingDustImpactEffect( end, trace.plane.normal );
		}
	}
}

/*
* CG_Event_Pain
*/
void CG_Event_Pain( entity_state_t *state, int parm ) {
	if( parm == PAIN_WARSHELL ) {
		if( ISVIEWERENTITY( state->number ) ) {
			SoundSystem::instance()->startGlobalSound( cgs.media.sfxShellHit, CHAN_PAIN,
													   cg_volume_players->value );
		} else {
			SoundSystem::instance()->startRelativeSound( cgs.media.sfxShellHit, state->number, CHAN_PAIN,
														 cg_volume_players->value, state->attenuation );
		}
	} else {
		CG_SexedSound( state->number, CHAN_PAIN, va( S_PLAYER_PAINS, 25 * parm ),
					   cg_volume_players->value, state->attenuation );
	}

	switch( (int)brandom( 0, 3 ) ) {
		case 0:
			CG_PModel_AddAnimation( state->number, 0, TORSO_PAIN1, 0, EVENT_CHANNEL );
			break;
		case 1:
			CG_PModel_AddAnimation( state->number, 0, TORSO_PAIN2, 0, EVENT_CHANNEL );
			break;
		case 2:
		default:
			CG_PModel_AddAnimation( state->number, 0, TORSO_PAIN3, 0, EVENT_CHANNEL );
			break;
	}
}

/*
* CG_Event_Die
*/
void CG_Event_Die( entity_state_t *state, int parm ) {
	CG_SexedSound( state->number, CHAN_PAIN, S_PLAYER_DEATH, cg_volume_players->value, state->attenuation );

	switch( parm ) {
		case 0:
		default:
			CG_PModel_AddAnimation( state->number, BOTH_DEATH1, BOTH_DEATH1, ANIM_NONE, EVENT_CHANNEL );
			break;
		case 1:
			CG_PModel_AddAnimation( state->number, BOTH_DEATH2, BOTH_DEATH2, ANIM_NONE, EVENT_CHANNEL );
			break;
		case 2:
			CG_PModel_AddAnimation( state->number, BOTH_DEATH3, BOTH_DEATH3, ANIM_NONE, EVENT_CHANNEL );
			break;
	}
}

/*
* CG_Event_Dash
*/
void CG_Event_Dash( entity_state_t *state, int parm ) {
	switch( parm ) {
		default:
			break;
		case 0: // dash front
			CG_PModel_AddAnimation( state->number, LEGS_DASH, 0, 0, EVENT_CHANNEL );
			CG_SexedSound( state->number, CHAN_BODY, va( S_PLAYER_DASH_1_to_2, ( rand() & 1 ) + 1 ),
						   cg_volume_players->value, state->attenuation );
			break;
		case 1: // dash left
			CG_PModel_AddAnimation( state->number, LEGS_DASH_LEFT, 0, 0, EVENT_CHANNEL );
			CG_SexedSound( state->number, CHAN_BODY, va( S_PLAYER_DASH_1_to_2, ( rand() & 1 ) + 1 ),
						   cg_volume_players->value, state->attenuation );
			break;
		case 2: // dash right
			CG_PModel_AddAnimation( state->number, LEGS_DASH_RIGHT, 0, 0, EVENT_CHANNEL );
			CG_SexedSound( state->number, CHAN_BODY, va( S_PLAYER_DASH_1_to_2, ( rand() & 1 ) + 1 ),
						   cg_volume_players->value, state->attenuation );
			break;
		case 3: // dash back
			CG_PModel_AddAnimation( state->number, LEGS_DASH_BACK, 0, 0, EVENT_CHANNEL );
			CG_SexedSound( state->number, CHAN_BODY, va( S_PLAYER_DASH_1_to_2, ( rand() & 1 ) + 1 ),
						   cg_volume_players->value, state->attenuation );
			break;
	}

	cg.effectsSystem.spawnDashEffect( cg_entities[state->number].prev.origin, state->origin );

	// since most dash animations jump with right leg, reset the jump to start with left leg after a dash
	cg_entities[state->number].jumpedLeft = true;
}

/*
* CG_Event_WallJump
*/
void CG_Event_WallJump( entity_state_t *state, int parm, int ev ) {
	vec3_t normal, forward, right;

	ByteToDir( parm, normal );

	AngleVectors( tv( state->angles[0], state->angles[1], 0 ), forward, right, NULL );

	if( DotProduct( normal, right ) > 0.3 ) {
		CG_PModel_AddAnimation( state->number, LEGS_WALLJUMP_RIGHT, 0, 0, EVENT_CHANNEL );
	} else if( -DotProduct( normal, right ) > 0.3 ) {
		CG_PModel_AddAnimation( state->number, LEGS_WALLJUMP_LEFT, 0, 0, EVENT_CHANNEL );
	} else if( -DotProduct( normal, forward ) > 0.3 ) {
		CG_PModel_AddAnimation( state->number, LEGS_WALLJUMP_BACK, 0, 0, EVENT_CHANNEL );
	} else {
		CG_PModel_AddAnimation( state->number, LEGS_WALLJUMP, 0, 0, EVENT_CHANNEL );
	}

	if( ev == EV_WALLJUMP_FAILED ) {
		if( ISVIEWERENTITY( state->number ) ) {
			SoundSystem::instance()->startGlobalSound( cgs.media.sfxWalljumpFailed, CHAN_BODY, cg_volume_effects->value );
		} else {
			SoundSystem::instance()->startRelativeSound( cgs.media.sfxWalljumpFailed, state->number, CHAN_BODY, cg_volume_effects->value, ATTN_NORM );
		}
	} else {
		CG_SexedSound( state->number, CHAN_BODY, va( S_PLAYER_WALLJUMP_1_to_2, ( rand() & 1 ) + 1 ),
					   cg_volume_players->value, state->attenuation );

		// smoke effect
		if( cg_cartoonEffects->integer & 1 ) {
			vec3_t pos;
			VectorCopy( state->origin, pos );
			pos[2] += 15;
			cg.effectsSystem.spawnWalljumpDustImpactEffect( pos, normal );
		}
	}
}

/*
* CG_Event_DoubleJump
*/
void CG_Event_DoubleJump( entity_state_t *state, int parm ) {
	CG_SexedSound( state->number, CHAN_BODY, va( S_PLAYER_JUMP_1_to_2, ( rand() & 1 ) + 1 ),
				   cg_volume_players->value, state->attenuation );
}

/*
* CG_Event_Jump
*/
void CG_Event_Jump( entity_state_t *state, int parm ) {
	float attenuation = state->attenuation;
	// Hack for the bobot jump sound.
	// Amplifying it is not an option as it becomes annoying at close range.
	// Note that this can not and should not be handled at the game-server level as a client may use an arbitrary model.
	if( const char *modelName = cg_entPModels[state->number].pmodelinfo->name ) {
		if( ::strstr( modelName, "bobot" ) ) {
			attenuation = ATTN_DISTANT;
		}
	}

	centity_t *cent = &cg_entities[state->number];
	float xyspeedcheck = Q_Sqrt( cent->animVelocity[0] * cent->animVelocity[0] + cent->animVelocity[1] * cent->animVelocity[1] );
	if( xyspeedcheck < 100 ) { // the player is jumping on the same place, not running
		CG_PModel_AddAnimation( state->number, LEGS_JUMP_NEUTRAL, 0, 0, EVENT_CHANNEL );
		CG_SexedSound( state->number, CHAN_BODY, va( S_PLAYER_JUMP_1_to_2, ( rand() & 1 ) + 1 ),
					   cg_volume_players->value, attenuation );
	} else {
		vec3_t movedir;
		mat3_t viewaxis;

		movedir[0] = cent->animVelocity[0];
		movedir[1] = cent->animVelocity[1];
		movedir[2] = 0;
		VectorNormalizeFast( movedir );

		Matrix3_FromAngles( tv( 0, cent->current.angles[YAW], 0 ), viewaxis );

		// see what's his relative movement direction
		if( DotProduct( movedir, &viewaxis[AXIS_FORWARD] ) > 0.25f ) {
			cent->jumpedLeft = !cent->jumpedLeft;
			if( !cent->jumpedLeft ) {
				CG_PModel_AddAnimation( state->number, LEGS_JUMP_LEG2, 0, 0, EVENT_CHANNEL );
				CG_SexedSound( state->number, CHAN_BODY, va( S_PLAYER_JUMP_1_to_2, ( rand() & 1 ) + 1 ),
							   cg_volume_players->value, attenuation );
			} else {
				CG_PModel_AddAnimation( state->number, LEGS_JUMP_LEG1, 0, 0, EVENT_CHANNEL );
				CG_SexedSound( state->number, CHAN_BODY, va( S_PLAYER_JUMP_1_to_2, ( rand() & 1 ) + 1 ),
							   cg_volume_players->value, attenuation );
			}
		} else {
			CG_PModel_AddAnimation( state->number, LEGS_JUMP_NEUTRAL, 0, 0, EVENT_CHANNEL );
			CG_SexedSound( state->number, CHAN_BODY, va( S_PLAYER_JUMP_1_to_2, ( rand() & 1 ) + 1 ),
						   cg_volume_players->value, attenuation );
		}
	}
}

static void handleWeaponActivateEvent( entity_state_t *ent, int parm, bool predicted ) {
	const int weapon = ( parm >> 1 ) & 0x3f;
	const int fireMode = ( parm & 0x1 ) ? FIRE_MODE_STRONG : FIRE_MODE_WEAK;
	const bool viewer = ISVIEWERENTITY( ent->number );

	CG_PModel_AddAnimation( ent->number, 0, TORSO_WEAPON_SWITCHIN, 0, EVENT_CHANNEL );

	if( predicted ) {
		cg_entities[ent->number].current.weapon = weapon;
		if( fireMode == FIRE_MODE_STRONG ) {
			cg_entities[ent->number].current.effects |= EF_STRONG_WEAPON;
		}

		CG_ViewWeapon_RefreshAnimation( &cg.weapon );
	}

	if( viewer ) {
		cg.predictedWeaponSwitch = 0;
	}

	if( viewer ) {
		SoundSystem::instance()->startGlobalSound( cgs.media.sfxWeaponUp, CHAN_AUTO, cg_volume_effects->value );
	} else {
		SoundSystem::instance()->startFixedSound( cgs.media.sfxWeaponUp, ent->origin, CHAN_AUTO, cg_volume_effects->value, ATTN_NORM );
	}
}

static void handleSmoothRefireWeaponEvent( entity_state_t *ent, int parm, bool predicted ) {
	if( predicted ) {
		const int weapon = ( parm >> 1 ) & 0x3f;
		const int fireMode = ( parm & 0x1 ) ? FIRE_MODE_STRONG : FIRE_MODE_WEAK;

		cg_entities[ent->number].current.weapon = weapon;
		if( fireMode == FIRE_MODE_STRONG ) {
			cg_entities[ent->number].current.effects |= EF_STRONG_WEAPON;
		}

		CG_ViewWeapon_RefreshAnimation( &cg.weapon );

		if( weapon == WEAP_LASERGUN ) {
			CG_Event_LaserBeam( ent->number, weapon, fireMode );
		}
	}
}

static void handleFireWeaponEvent( entity_state_t *ent, int parm, bool predicted ) {
	const int weapon = ( parm >> 1 ) & 0x3f;
	const int fireMode = ( parm & 0x1 ) ? FIRE_MODE_STRONG : FIRE_MODE_WEAK;

	if( predicted ) {
		cg_entities[ent->number].current.weapon = weapon;
		if( fireMode == FIRE_MODE_STRONG ) {
			cg_entities[ent->number].current.effects |= EF_STRONG_WEAPON;
		}
	}

	CG_FireWeaponEvent( ent->number, weapon, fireMode );

	// riotgun bullets, electrobolt and instagun beams are predicted when the weapon is fired
	if( predicted ) {
		vec3_t origin, dir;

		if( ( weapon == WEAP_ELECTROBOLT && fireMode == FIRE_MODE_STRONG ) || weapon == WEAP_INSTAGUN ) {
			VectorCopy( cg.predictedPlayerState.pmove.origin, origin );
			origin[2] += cg.predictedPlayerState.viewheight;
			AngleVectors( cg.predictedPlayerState.viewangles, dir, NULL, NULL );
			CG_Event_WeaponBeam( origin, dir, cg.predictedPlayerState.POVnum, weapon, fireMode );
		} else if( weapon == WEAP_RIOTGUN || weapon == WEAP_MACHINEGUN ) {
			int seed = cg.predictedEventTimes[EV_FIREWEAPON] & 255;

			VectorCopy( cg.predictedPlayerState.pmove.origin, origin );
			origin[2] += cg.predictedPlayerState.viewheight;
			AngleVectors( cg.predictedPlayerState.viewangles, dir, NULL, NULL );

			if( weapon == WEAP_RIOTGUN ) {
				CG_Event_FireRiotgun( origin, dir, weapon, fireMode, seed, cg.predictedPlayerState.POVnum );
			} else {
				CG_Event_FireMachinegun( origin, dir, weapon, fireMode, seed, cg.predictedPlayerState.POVnum );
			}
		} else if( weapon == WEAP_LASERGUN ) {
			CG_Event_LaserBeam( ent->number, weapon, fireMode );
		}
	}
}

static void handleElectroTrailEvent( entity_state_t *ent, int parm, bool predicted ) {
	// check the owner for predicted case
	if( ISVIEWERENTITY( parm ) && ( predicted != cg.view.playerPrediction ) ) {
		return;
	}

	CG_Event_WeaponBeam( ent->origin, ent->origin2, parm, WEAP_ELECTROBOLT, ent->firemode );
}

static void handleInstaTrailEvent( entity_state_t *ent, int parm, bool predicted ) {
	// check the owner for predicted case
	if( ISVIEWERENTITY( parm ) && ( predicted != cg.view.playerPrediction ) ) {
		return;
	}

	CG_Event_WeaponBeam( ent->origin, ent->origin2, parm, WEAP_INSTAGUN, FIRE_MODE_STRONG );
}

static void handleFireRiotgunEvent( entity_state_t *ent, int parm, bool predicted ) {
	// check the owner for predicted case
	if( ISVIEWERENTITY( ent->ownerNum ) && ( predicted != cg.view.playerPrediction ) ) {
		return;
	}

	CG_Event_FireRiotgun( ent->origin, ent->origin2, ent->weapon, ent->firemode, parm, ent->ownerNum );
}

static void handleFireBulletEvent( entity_state_t *ent, int parm, bool predicted ) {
	// check the owner for predicted case
	if( ISVIEWERENTITY( ent->ownerNum ) && ( predicted != cg.view.playerPrediction ) ) {
		return;
	}

	CG_Event_FireMachinegun( ent->origin, ent->origin2, ent->weapon, ent->firemode, parm, ent->ownerNum );
}

static void handleNoAmmoClickEvent( entity_state_t *ent, int parm, bool predicted ) {
	if( ISVIEWERENTITY( ent->number ) ) {
		SoundSystem::instance()->startGlobalSound( cgs.media.sfxWeaponUpNoAmmo, CHAN_ITEM, cg_volume_effects->value );
	} else {
		SoundSystem::instance()->startFixedSound( cgs.media.sfxWeaponUpNoAmmo, ent->origin, CHAN_ITEM, cg_volume_effects->value, ATTN_IDLE );
	}
}

static void handleJumppadEvent( entity_state_t *ent, bool predicted ) {
	CG_SexedSound( ent->number, CHAN_BODY, va( S_PLAYER_JUMP_1_to_2, ( rand() & 1 ) + 1 ),
				   cg_volume_players->value, ent->attenuation );
	CG_PModel_AddAnimation( ent->number, LEGS_JUMP_NEUTRAL, 0, 0, EVENT_CHANNEL );
}

static void handleSexedSoundEvent( entity_state_t *ent, int parm, bool predicted ) {
	if( parm == 2 ) {
		CG_SexedSound( ent->number, CHAN_AUTO, S_PLAYER_GASP, cg_volume_players->value, ent->attenuation );
	} else if( parm == 1 ) {
		CG_SexedSound( ent->number, CHAN_AUTO, S_PLAYER_DROWN, cg_volume_players->value, ent->attenuation );
	}
}

static void handlePnodeEvent( entity_state_t *ent, int parm, bool predicted ) {
	vec4_t color;
	color[0] = COLOR_R( ent->colorRGBA ) * ( 1.0 / 255.0 );
	color[1] = COLOR_G( ent->colorRGBA ) * ( 1.0 / 255.0 );
	color[2] = COLOR_B( ent->colorRGBA ) * ( 1.0 / 255.0 );
	color[3] = COLOR_A( ent->colorRGBA ) * ( 1.0 / 255.0 );
	cg.effectsSystem.spawnGameDebugBeam( ent->origin, ent->origin2, color, parm );
}

static const RgbaLifespan kSparksColor {
	.initial  = { 1.0f, 0.5f, 0.1f, 0.0f },
	.fadedIn  = { 1.0f, 1.0f, 1.0f, 1.0f },
	.fadedOut = { 0.5f, 0.5f, 0.5f, 0.5f },
};

static void handleSparksEvent( entity_state_t *ent, int parm, bool predicted ) {
	if( cg_particles->integer ) {
		vec3_t dir;
		ByteToDir( parm, dir );

		int count;
		if( ent->damage > 0 ) {
			count = (int)( ent->damage * 0.25f );
			Q_clamp( count, 1, 10 );
		} else {
			count = 6;
		}

		ConicalFlockParams flockParams {
			.origin = { ent->origin[0], ent->origin[1], ent->origin[2] },
			.offset = { dir[0], dir[1], dir[2] },
			.dir    = { dir[0], dir[1], dir[2] }
		};

		Particle::AppearanceRules appearanceRules {
			.materials     = cgs.media.shaderMetalRicochetParticle.getAddressOfHandle(),
			.colors        = { &kSparksColor, 1 },
			.geometryRules = Particle::SparkRules { .length = { .mean = 4.0f }, .width = { .mean = 1.0f } },
		};

		cg.particleSystem.addSmallParticleFlock( appearanceRules, flockParams );
	}
}

static void handleBulletSparksEvent( entity_state_t *ent, int parm, bool predicted ) {
	vec3_t dir;
	ByteToDir( parm, dir );
	// TODO???
}

static void handleItemRespawnEvent( entity_state_t *ent, int parm, bool predicted ) {
	cg_entities[ent->number].respawnTime = cg.time;
	SoundSystem::instance()->startRelativeSound( cgs.media.sfxItemRespawn, ent->number, CHAN_AUTO,
												 cg_volume_effects->value, ATTN_IDLE );
}

static void handlePlayerRespawnEvent( entity_state_t *ent, int parm, bool predicted ) {
	if( (unsigned)ent->ownerNum == cgs.playerNum + 1 ) {
		CG_ResetKickAngles();
		CG_ResetColorBlend();
		CG_ResetDamageIndicator();
	}

	SoundSystem::instance()->startFixedSound( cgs.media.sfxPlayerRespawn, ent->origin, CHAN_AUTO, cg_volume_effects->value, ATTN_NORM );

	if( ent->ownerNum && ent->ownerNum < gs.maxclients + 1 ) {
		cg_entities[ent->ownerNum].localEffects[LOCALEFFECT_EV_PLAYER_TELEPORT_IN] = cg.time;
		VectorCopy( ent->origin, cg_entities[ent->ownerNum].teleportedTo );
	}
}

static void handlePlayerTeleportInEvent( entity_state_t *ent, int parm, bool predicted ) {
	SoundSystem::instance()->startFixedSound( cgs.media.sfxTeleportIn, ent->origin, CHAN_AUTO, cg_volume_effects->value, ATTN_NORM );

	if( ent->ownerNum && ent->ownerNum < gs.maxclients + 1 ) {
		cg_entities[ent->ownerNum].localEffects[LOCALEFFECT_EV_PLAYER_TELEPORT_IN] = cg.time;
		VectorCopy( ent->origin, cg_entities[ent->ownerNum].teleportedTo );
	}
}

static void handlePlayerTeleportOutEvent( entity_state_t *ent, int parm, bool predicted ) {
	SoundSystem::instance()->startFixedSound( cgs.media.sfxTeleportOut, ent->origin, CHAN_AUTO, cg_volume_effects->value, ATTN_NORM );

	if( ent->ownerNum && ent->ownerNum < gs.maxclients + 1 ) {
		cg_entities[ent->ownerNum].localEffects[LOCALEFFECT_EV_PLAYER_TELEPORT_OUT] = cg.time;
		VectorCopy( ent->origin, cg_entities[ent->ownerNum].teleportedFrom );
	}
}

static void handlePlasmaExplosionEvent( entity_state_t *ent, int parm, bool predicted ) {
	vec3_t dir;
	ByteToDir( parm, dir );

	cg.effectsSystem.spawnPlasmaExplosionEffect( ent->origin, dir, ent->firemode );

	if( ent->firemode == FIRE_MODE_STRONG ) {
		CG_StartKickAnglesEffect( ent->origin, 50, ent->weapon * 8, 100 );
	} else {
		CG_StartKickAnglesEffect( ent->origin, 30, ent->weapon * 8, 75 );
	}
}

static inline void decodeBoltImpact( int parm, vec3_t impactNormal, vec3_t impactDir, bool *spawnWallImpact ) {
	const unsigned impactDirByte    = ( (unsigned)parm >> 8 ) & 0xFF;
	const unsigned impactNormalByte = ( (unsigned)parm >> 0 ) & 0xFF;

	ByteToDir( (int)impactNormalByte, impactNormal );
	ByteToDir( (int)impactDirByte, impactDir );

	*spawnWallImpact = ( (unsigned)parm >> 16 ) != 0;
}

static void handleBoltExplosionEvent( entity_state_t *ent, int parm, bool predicted ) {
	vec3_t impactNormal, impactDir;
	bool spawnWallImpact;
	decodeBoltImpact( parm, impactNormal, impactDir, &spawnWallImpact );

	cg.effectsSystem.spawnElectroboltHitEffect( ent->origin, impactNormal, impactDir, spawnWallImpact, ent->ownerNum );
}

static void handleInstaExplosionEvent( entity_state_t *ent, int parm, bool predicted ) {
	vec3_t impactNormal, impactDir;
	bool spawnWallImpact;
	decodeBoltImpact( parm, impactNormal, impactDir, &spawnWallImpact );

	cg.effectsSystem.spawnInstagunHitEffect( ent->origin, impactNormal, impactDir, spawnWallImpact, ent->ownerNum );
}

static void handleGrenadeExplosionEvent( entity_state_t *ent, int parm, bool predicted ) {
	vec3_t dir;
	if( parm ) {
		// we have a direction
		ByteToDir( parm, dir );
		cg.effectsSystem.spawnGrenadeExplosionEffect( ent->origin, dir, ent->firemode );
	} else {
		cg.effectsSystem.spawnGrenadeExplosionEffect( ent->origin, &axis_identity[AXIS_UP], ent->firemode );
	}

	if( ent->firemode == FIRE_MODE_STRONG ) {
		CG_StartKickAnglesEffect( ent->origin, 135, ent->weapon * 8, 325 );
	} else {
		CG_StartKickAnglesEffect( ent->origin, 125, ent->weapon * 8, 300 );
	}
}

static void handleRocketExplosionEvent( entity_state_t *ent, int parm, bool predicted ) {
	vec3_t dir;
	ByteToDir( parm, dir );

	cg.effectsSystem.spawnRocketExplosionEffect( ent->origin, dir, ent->firemode );

	if( ent->firemode == FIRE_MODE_STRONG ) {
		CG_StartKickAnglesEffect( ent->origin, 135, ent->weapon * 8, 300 );
	} else {
		CG_StartKickAnglesEffect( ent->origin, 125, ent->weapon * 8, 275 );
	}
}

static void handleShockwaveExplosionEvent( entity_state_t *ent, int parm, bool predicted ) {
	vec3_t dir;
	ByteToDir( parm, dir );

	cg.effectsSystem.spawnShockwaveExplosionEffect( ent->origin, dir, ent->firemode );

	if( ent->firemode == FIRE_MODE_STRONG ) {
		CG_StartKickAnglesEffect( ent->origin, 90, ent->weapon * 8, 200 );
	} else {
		CG_StartKickAnglesEffect( ent->origin, 90, ent->weapon * 8, 200 );
	}
}

static void handleGunbladeBlastImpactEvent( entity_state_t *ent, int parm, bool predicted ) {
	vec3_t dir;
	ByteToDir( parm, dir );

	cg.effectsSystem.spawnGunbladeBlastHitEffect( ent->origin, dir );

	//ent->skinnum is knockback value
	CG_StartKickAnglesEffect( ent->origin, ent->skinnum * 8, ent->weapon * 8, 200 );
}

static void handleBloodEvent( entity_state_t *ent, int parm, bool predicted ) {
	if( cg_showPOVBlood->integer || !ISVIEWERENTITY( ent->ownerNum ) ) {
		vec3_t dir;
		ByteToDir( parm, dir );
		if( VectorCompare( dir, vec3_origin ) ) {
			dir[2] = 1.0f;
		}
		cg.effectsSystem.spawnPlayerHitEffect( ent->origin, dir, ent->damage );
	}
}

static void handleMoverEvent( entity_state_t *ent, int parm ) {
	vec3_t so;
	CG_GetEntitySpatilization( ent->number, so, NULL );
	SoundSystem::instance()->startFixedSound( cgs.soundPrecache[parm], so, CHAN_AUTO, cg_volume_effects->value, ATTN_STATIC );
}

void CG_EntityEvent( entity_state_t *ent, int ev, int parm, bool predicted ) {
	if( ISVIEWERENTITY( ent->number ) && ( ev < PREDICTABLE_EVENTS_MAX ) && ( predicted != cg.view.playerPrediction ) ) {
		return;
	}

	switch( ev ) {
		//  PREDICTABLE EVENTS

		case EV_WEAPONACTIVATE: return handleWeaponActivateEvent( ent, parm, predicted );
		case EV_SMOOTHREFIREWEAPON: return handleSmoothRefireWeaponEvent( ent, parm, predicted );
		case EV_FIREWEAPON: return handleFireWeaponEvent( ent, parm, predicted );
		case EV_ELECTROTRAIL: return handleElectroTrailEvent( ent, parm, predicted );
		case EV_INSTATRAIL: return handleInstaTrailEvent( ent, parm, predicted );
		case EV_FIRE_RIOTGUN: return handleFireRiotgunEvent( ent, parm, predicted );
		case EV_FIRE_BULLET: return handleFireBulletEvent( ent, parm, predicted );
		case EV_NOAMMOCLICK: return handleNoAmmoClickEvent( ent, parm, predicted );
		case EV_DASH: return CG_Event_Dash( ent, parm );
		case EV_WALLJUMP: [[fallthrough]];
		case EV_WALLJUMP_FAILED: return CG_Event_WallJump( ent, parm, ev );
		case EV_DOUBLEJUMP: return CG_Event_DoubleJump( ent, parm );
		case EV_JUMP: return CG_Event_Jump( ent, parm );
		case EV_JUMP_PAD: return handleJumppadEvent( ent, predicted );
		case EV_FALL: return CG_Event_Fall( ent, parm );

		//  NON PREDICTABLE EVENTS

		case EV_WEAPONDROP: return CG_PModel_AddAnimation( ent->number, 0, TORSO_WEAPON_SWITCHOUT, 0, EVENT_CHANNEL );
		case EV_SEXEDSOUND: return handleSexedSoundEvent( ent, parm, predicted );
		case EV_PAIN: return CG_Event_Pain( ent, parm );
		case EV_DIE: return CG_Event_Die( ent, parm );
		case EV_GIB: return;
		case EV_EXPLOSION1: return cg.effectsSystem.spawnGenericExplosionEffect( ent->origin, FIRE_MODE_WEAK, parm * 8 );
		case EV_EXPLOSION2: return cg.effectsSystem.spawnGenericExplosionEffect( ent->origin, FIRE_MODE_STRONG, parm * 16 );
		case EV_GREEN_LASER: return;
		case EV_PNODE: return handlePnodeEvent( ent, parm, predicted );
		case EV_SPARKS: return handleSparksEvent( ent, parm, predicted );
		case EV_BULLET_SPARKS: return handleBulletSparksEvent( ent, parm, predicted );
		case EV_LASER_SPARKS: return;
		case EV_GESTURE: return CG_SexedSound( ent->number, CHAN_BODY, "*taunt", cg_volume_players->value, ent->attenuation );
		case EV_DROP: return CG_PModel_AddAnimation( ent->number, 0, TORSO_DROP, 0, EVENT_CHANNEL );
		case EV_SPOG: return CG_SmallPileOfGibs( ent->origin, parm, ent->origin2, ent->team );
		case EV_ITEM_RESPAWN: return handleItemRespawnEvent( ent, parm, predicted );
		case EV_PLAYER_RESPAWN: return handlePlayerRespawnEvent( ent, parm, predicted );
		case EV_PLAYER_TELEPORT_IN: return handlePlayerTeleportInEvent( ent, parm, predicted );
		case EV_PLAYER_TELEPORT_OUT: return handlePlayerTeleportOutEvent( ent, parm, predicted );
		case EV_PLASMA_EXPLOSION: return handlePlasmaExplosionEvent( ent, parm, predicted );
		case EV_BOLT_EXPLOSION: return handleBoltExplosionEvent( ent, parm, predicted );
		case EV_INSTA_EXPLOSION: return handleInstaExplosionEvent( ent, parm, predicted );
		case EV_GRENADE_EXPLOSION: return handleGrenadeExplosionEvent( ent, parm, predicted );
		case EV_ROCKET_EXPLOSION: return handleRocketExplosionEvent( ent, parm, predicted );
		case EV_WAVE_EXPLOSION: return handleShockwaveExplosionEvent( ent, parm, predicted );
		case EV_GRENADE_BOUNCE: return cg.effectsSystem.spawnGrenadeBounceEffect( ent->number, parm );
		case EV_BLADE_IMPACT: return cg.effectsSystem.spawnGunbladeBladeHitEffect( ent->origin, ent->origin2 );
		case EV_GUNBLADEBLAST_IMPACT: return handleGunbladeBlastImpactEvent( ent, parm, predicted );
		case EV_BLOOD: return handleBloodEvent( ent, parm, predicted );

		// func movers

		case EV_PLAT_HIT_TOP: [[fallthrough]];
		case EV_PLAT_HIT_BOTTOM: [[fallthrough]];
		case EV_PLAT_START_MOVING: [[fallthrough]];
		case EV_DOOR_HIT_TOP: [[fallthrough]];
		case EV_DOOR_HIT_BOTTOM: [[fallthrough]];
		case EV_DOOR_START_MOVING: [[fallthrough]];
		case EV_BUTTON_FIRE: [[fallthrough]];
		case EV_TRAIN_STOP: [[fallthrough]];
		case EV_TRAIN_START: return handleMoverEvent( ent, parm );

		default: return;
	}
}

#define ISEARLYEVENT( ev ) ( ev == EV_WEAPONDROP )

static void CG_FireEntityEvents( bool early ) {

	for( int pnum = 0; pnum < cg.frame.numEntities; pnum++ ) {
		entity_state_t *state = &cg.frame.parsedEntities[pnum & ( MAX_PARSE_ENTITIES - 1 )];

		if( state->type == ET_SOUNDEVENT ) {
			if( early ) {
				CG_SoundEntityNewState( &cg_entities[state->number] );
			}
			continue;
		}

		for( int j = 0; j < 2; j++ ) {
			if( early == ISEARLYEVENT( state->events[j] ) ) {
				CG_EntityEvent( state, state->events[j], state->eventParms[j], false );
			}
		}
	}
}

static void handlePlayerStateHitSoundEvent( unsigned event, unsigned parm ) {
	if( parm < 4 ) {
		// hit of some caliber
		SoundSystem::instance()->startLocalSound( cgs.media.sfxWeaponHit[parm], cg_volume_hitsound->value );
		SoundSystem::instance()->startLocalSound( cgs.media.sfxWeaponHit2[parm], cg_volume_hitsound->value );
		CG_ScreenCrosshairDamageUpdate();
	} else if( parm == 4 ) {
		// killed an enemy
		SoundSystem::instance()->startLocalSound( cgs.media.sfxWeaponKill, cg_volume_hitsound->value );
		CG_ScreenCrosshairDamageUpdate();
	} else if( parm <= 6 ) {
		// hit a teammate
		SoundSystem::instance()->startLocalSound( cgs.media.sfxWeaponHitTeam, cg_volume_hitsound->value );
		if( cg_showhelp->integer ) {
			if( random() <= 0.5f ) {
				CG_CenterPrint( "Don't shoot at members of your team!" );
			} else {
				CG_CenterPrint( "You are shooting at your team-mates!" );
			}
		}
	}
}

static void handlePlayerStatePickupEvent( unsigned event, unsigned parm ) {
	if( cg_pickup_flash->integer && !cg.view.thirdperson ) {
		CG_StartColorBlendEffect( 1.0f, 1.0f, 1.0f, 0.25f, 150 );
	}

	bool processAutoSwitch = false;
	const int autoSwitchVarValue = cg_weaponAutoSwitch->integer;
	if( autoSwitchVarValue && !cgs.demoPlaying && ( parm > WEAP_NONE && parm < WEAP_TOTAL ) ) {
		if( cg.predictedPlayerState.pmove.pm_type == PM_NORMAL && cg.predictedPlayerState.POVnum == cgs.playerNum + 1 ) {
			if( !cg.oldFrame.playerState.inventory[parm] ) {
				processAutoSwitch = true;
			}
		}
	}

	if( processAutoSwitch ) {
		// Auto-switch only works when the user didn't have the just-picked weapon
		if( autoSwitchVarValue == 2 ) {
			// Switch when player's only weapon is gunblade
			unsigned i;
			for( i = WEAP_GUNBLADE + 1; i < WEAP_TOTAL; i++ ) {
				if( i != parm && cg.predictedPlayerState.inventory[i] ) {
					break;
				}
			}
			if( i == WEAP_TOTAL ) { // didn't have any weapon
				CG_UseItem( va( "%i", parm ) );
			}
		} else if( autoSwitchVarValue == 1 ) {
			// Switch when the new weapon improves player's selected weapon
			unsigned best = WEAP_GUNBLADE;
			for( unsigned i = WEAP_GUNBLADE + 1; i < WEAP_TOTAL; i++ ) {
				if( i != parm && cg.predictedPlayerState.inventory[i] ) {
					best = i;
				}
			}
			if( best < parm ) {
				CG_UseItem( va( "%i", parm ) );
			}
		}
	}
}

/*
* CG_FirePlayerStateEvents
* This events are only received by this client, and only affect it.
*/
static void CG_FirePlayerStateEvents( void ) {
	if( cg.view.POVent != (int)cg.frame.playerState.POVnum ) {
		return;
	}

	vec3_t dir;
	for( unsigned count = 0; count < 2; count++ ) {
		// first byte is event number, second is parm
		const unsigned event = cg.frame.playerState.event[count] & 127;
		const unsigned parm = cg.frame.playerState.eventParm[count] & 0xFF;

		switch( event ) {
			case PSEV_HIT:
				handlePlayerStateHitSoundEvent( event, parm );
				break;

			case PSEV_PICKUP:
				handlePlayerStatePickupEvent( event, parm );
				break;

			case PSEV_DAMAGE_20:
				ByteToDir( parm, dir );
				CG_DamageIndicatorAdd( 20, dir );
				break;

			case PSEV_DAMAGE_40:
				ByteToDir( parm, dir );
				CG_DamageIndicatorAdd( 40, dir );
				break;

			case PSEV_DAMAGE_60:
				ByteToDir( parm, dir );
				CG_DamageIndicatorAdd( 60, dir );
				break;

			case PSEV_DAMAGE_80:
				ByteToDir( parm, dir );
				CG_DamageIndicatorAdd( 80, dir );
				break;

			case PSEV_INDEXEDSOUND:
				if( cgs.soundPrecache[parm] ) {
					SoundSystem::instance()->startGlobalSound( cgs.soundPrecache[parm], CHAN_AUTO, cg_volume_effects->value );
				}
				break;

			case PSEV_ANNOUNCER:
				CG_AddAnnouncerEvent( cgs.soundPrecache[parm], false );
				break;

			case PSEV_ANNOUNCER_QUEUED:
				CG_AddAnnouncerEvent( cgs.soundPrecache[parm], true );
				break;

			default:
				break;
		}
	}
}

/*
* CG_FireEvents
*/
void CG_FireEvents( bool early ) {
	if( !cg.fireEvents ) {
		return;
	}

	CG_FireEntityEvents( early );

	if( early ) {
		return;
	}

	CG_FirePlayerStateEvents();
	cg.fireEvents = false;
}
