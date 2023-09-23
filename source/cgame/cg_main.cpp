/*
Copyright (C) 2002-2003 Victor Luchits

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
#include "../gameshared/gs_public.h"
#include "../qcommon/wswstaticstring.h"
#include "../client/snd_public.h"
#include "../client/client.h"
#include "../ui/uisystem.h"

cg_static_t cgs;
cg_state_t cg;

centity_t cg_entities[MAX_EDICTS];

cvar_t *cg_predict;
cvar_t *cg_predict_optimize;
cvar_t *cg_showMiss;

cvar_t *cg_model;
cvar_t *cg_skin;
cvar_t *cg_hand;
cvar_t *cg_clan;
cvar_t *cg_handicap;

cvar_t *cg_addDecals;

//cvar_t *cg_footSteps;

cvar_t *cg_gun;

cvar_t *cg_thirdPerson;
cvar_t *cg_thirdPersonAngle;
cvar_t *cg_thirdPersonRange;

cvar_t *cg_colorCorrection;

cvar_t *cg_weaponFlashes;
cvar_t *cg_gunx;
cvar_t *cg_guny;
cvar_t *cg_gunz;
cvar_t *cg_debugPlayerModels;
cvar_t *cg_debugWeaponModels;
cvar_t *cg_gunbob;

cvar_t *cg_handOffset;
cvar_t *cg_gun_fov;
cvar_t *cg_gun_alpha;
cvar_t *cg_volume_players;
cvar_t *cg_volume_effects;
cvar_t *cg_volume_announcer;
cvar_t *cg_volume_hitsound;
cvar_t *cg_projectileSmokeTrail;
cvar_t *cg_projectileFireTrail;
cvar_t *cg_projectilePolyTrail;
cvar_t *cg_plasmaTrail;
cvar_t *cg_playerTrail;
cvar_t *cg_bloodTrailPalette;
cvar_t *cg_bloodTrailTime;
cvar_t *cg_showPOVBlood;
cvar_t *cg_projectileFireTrailAlpha;
cvar_t *cg_heavyRocketExplosions;
cvar_t *cg_heavyGrenadeExplosions;
cvar_t *cg_heavyShockwaveExplosions;
cvar_t *cg_explosionsWave;
cvar_t *cg_explosionsSmoke;
cvar_t *cg_explosionsClusters;
cvar_t *cg_gibs;
cvar_t *cg_outlineModels;
cvar_t *cg_outlineWorld;
cvar_t *cg_outlinePlayers;
cvar_t *cg_drawEntityBoxes;
cvar_t *cg_fov;
cvar_t *cg_zoomfov;
cvar_t *cg_movementStyle;
cvar_t *cg_noAutohop;
cvar_t *cg_predictLaserBeam;
cvar_t *cg_voiceChats;
cvar_t *cg_showSelfShadow;
cvar_t *cg_laserBeamSubdivisions;
cvar_t *cg_projectileAntilagOffset;
cvar_t *cg_raceGhosts;
cvar_t *cg_raceGhostsAlpha;
cvar_t *cg_chatBeep;
cvar_t *cg_chatFilter;
cvar_t *cg_chatShowIgnored;

cvar_t *cg_cartoonEffects;
cvar_t *cg_cartoonHitEffect;

cvar_t *cg_autoaction_demo;
cvar_t *cg_autoaction_screenshot;
cvar_t *cg_autoaction_stats;
cvar_t *cg_autoaction_spectator;
cvar_t *cg_simpleItems;
cvar_t *cg_simpleItemsSize;
cvar_t *cg_particles;
cvar_t *cg_showhelp;
cvar_t *cg_showClamp;

cvar_t *debug_impact;

cvar_t *cg_damage_indicator;
cvar_t *cg_damage_indicator_time;
cvar_t *cg_pickup_flash;

cvar_t *cg_weaponAutoSwitch;

// force models
cvar_t *cg_teamPLAYERSmodel;
cvar_t *cg_teamPLAYERSmodelForce;
cvar_t *cg_teamALPHAmodel;
cvar_t *cg_teamALPHAmodelForce;
cvar_t *cg_teamBETAmodel;
cvar_t *cg_teamBETAmodelForce;

cvar_t *cg_teamPLAYERSskin;
cvar_t *cg_teamALPHAskin;
cvar_t *cg_teamBETAskin;

cvar_t *cg_teamPLAYERScolor;
cvar_t *cg_teamPLAYERScolorForce;
cvar_t *cg_teamALPHAcolor;
cvar_t *cg_teamBETAcolor;

cvar_t *cg_forceMyTeamAlpha;
cvar_t *cg_teamColoredBeams;
cvar_t *cg_teamColoredInstaBeams;

//cvar_t *cg_teamColorBeamMinimum;

cvar_t *cg_ebbeam_width;
cvar_t *cg_ebbeam_time;
cvar_t *cg_instabeam_width;
cvar_t *cg_instabeam_time;

cvar_t *cg_playList;
cvar_t *cg_playListShuffle;

cvar_t *cg_flashWindowCount;

cvar_t *cg_viewBob;

cvar_t *cg_viewSize;
cvar_t *cg_showFPS;
cvar_t *cg_draw2D;

cvar_t *cg_showZoomEffect;

cvar_t *cg_showViewBlends;

cvar_t *cg_showHUD;

static cvar_t *cg_showminimap;
static cvar_t *cg_showitemtimers;

cvar_t *cg_showPlayerNames;
cvar_t *cg_showPlayerNames_alpha;
cvar_t *cg_showPlayerNames_zfar;
cvar_t *cg_showPlayerNames_barWidth;
cvar_t *cg_showTeamInfo;

static cvar_t *cg_showPressedKeys;
cvar_t *cg_showChasers;

static cvar_t *cg_showMessageFeed;

static cvar_t *cg_centerTime;

cvar_t *cg_showPointedPlayer;

cvar_t *cg_showSpeed;
cvar_t *cg_showPickup;
cvar_t *cg_showAwards;
cvar_t *cg_showCaptureAreas;
cvar_t *cg_showFragsFeed;
cvar_t *cg_showTimer;

void CG_Error( const char *format, ... ) {
	char msg[1024];
	va_list argptr;

	va_start( argptr, format );
	Q_vsnprintfz( msg, sizeof( msg ), format, argptr );
	va_end( argptr );

	Com_Error( ERR_DROP, "%s\n", msg );
}

/*
* CG_LocalPrint
*/
void CG_LocalPrint( const char *format, ... ) {
	va_list argptr;
	char msg[GAMECHAT_STRING_SIZE];

	va_start( argptr, format );
	const int res = Q_vsnprintfz( msg, sizeof( msg ), format, argptr );
	va_end( argptr );

	const wsw::StringView msgView( msg, wsw::min( (size_t)res, sizeof( msg ) ) );
	wsw::ui::UISystem::instance()->addToMessageFeed( msgView.trim() );

	Con_PrintSilent( msg );
}

/*
* CG_GS_Malloc
*
* Used only for gameshared linking
*/
static void *CG_GS_Malloc( size_t size ) {
	return Q_malloc( size );
}

/*
* CG_GS_Free
*
* Used only for gameshared linking
*/
static void CG_GS_Free( void *data ) {
	Q_free(   data );
}

/*
* CG_GS_Trace
*/
static void CG_GS_Trace( trace_t *t, const vec3_t start, const vec3_t mins, const vec3_t maxs, const vec3_t end, int ignore, int contentmask, int timeDelta ) {
	assert( !timeDelta );
	CG_Trace( t, start, mins, maxs, end, ignore, contentmask );
}

/*
* CG_GS_PointContents
*/
static int CG_GS_PointContents( const vec3_t point, int timeDelta ) {
	assert( !timeDelta );
	return CG_PointContents( point );
}

/*
* CG_GS_GetEntityState
*/
static entity_state_t *CG_GS_GetEntityState( int entNum, int deltaTime ) {
	centity_t *cent;

	if( entNum == -1 ) {
		return NULL;
	}

	assert( entNum >= 0 && entNum < MAX_EDICTS );
	cent = &cg_entities[entNum];

	if( cent->serverFrame != cg.frame.serverFrame ) {
		return NULL;
	}
	return &cent->current;
}

/*
* CG_GS_GetConfigString
*/
static const char *CG_GS_GetConfigString( int index ) {
	if( index < 0 || index >= MAX_CONFIGSTRINGS ) {
		return NULL;
	}

	if( auto maybeConfigString = cgs.configStrings.get( index ) ) {
		return maybeConfigString->data();
	}

	return nullptr;
}

/*
* CG_InitGameShared
*
* Give gameshared access to some utilities
*/
static void CG_InitGameShared( void ) {
	memset( &gs, 0, sizeof( gs_state_t ) );
	gs.module = GS_MODULE_CGAME;
	gs.maxclients = atoi( cl.configStrings.getMaxClients()->data() );
	if( gs.maxclients < 1 || gs.maxclients > MAX_CLIENTS ) {
		gs.maxclients = MAX_CLIENTS;
	}

	module_PredictedEvent = CG_PredictedEvent;
	module_Error = CG_Error;
	module_Printf = Com_Printf;
	module_Malloc = CG_GS_Malloc;
	module_Free = CG_GS_Free;
	module_Trace = CG_GS_Trace;
	module_GetEntityState = CG_GS_GetEntityState;
	module_PointContents = CG_GS_PointContents;
	module_PMoveTouchTriggers = CG_Predict_TouchTriggers;
	module_GetConfigString = CG_GS_GetConfigString;

	GS_InitWeapons();
}

/*
* CG_TranslateColoredString
*/
const char *CG_TranslateColoredString( const char *string, char *dst, size_t dst_size ) {
	Q_strncpyz( dst, string, dst_size );
	return dst;
}

/*
* CG_RegisterWeaponModels
*/
static void CG_RegisterWeaponModels( void ) {
	int i;

	for( i = 0; i < cgs.numWeaponModels; i++ ) {
		cgs.weaponInfos[i] = CG_RegisterWeaponModel( cgs.weaponModels[i], i );
	}

	// special case for weapon 0. Must always load the animation script
	if( !cgs.weaponInfos[0] ) {
		cgs.weaponInfos[0] = CG_CreateWeaponZeroModel( cgs.weaponModels[0] );
	}
}

/*
* CG_RegisterModels
*/
static void CG_RegisterModels( void ) {
	if( cgs.precacheModelsStart == MAX_MODELS ) {
		return;
	}

	if( cgs.precacheModelsStart == 0 ) {
		const auto maybeName = cgs.configStrings.getWorldModel();
		if( maybeName ) {
			const auto name = *maybeName;
			if( !CG_LoadingItemName( name.data() ) ) {
				return;
			}
			CG_LoadingString( name.data() );
			R_RegisterWorldModel( name.data() );
		}

		CG_LoadingString( "models" );

		cgs.numWeaponModels = 1;
		Q_strncpyz( cgs.weaponModels[0], "generic/generic.md3", sizeof( cgs.weaponModels[0] ) );

		cgs.precacheModelsStart = 1;
	}

	for( unsigned i = cgs.precacheModelsStart; i < MAX_MODELS; i++ ) {
		const auto maybeName = cgs.configStrings.getModel( i );
		if( !maybeName ) {
			cgs.precacheModelsStart = MAX_MODELS;
			break;
		}

		const auto name = *maybeName;
		cgs.precacheModelsStart = i;

		if( name.startsWith( '#' ) ) {
			// special player weapon model
			if( cgs.numWeaponModels >= WEAP_TOTAL ) {
				continue;
			}

			if( !CG_LoadingItemName( name.data() ) ) {
				return;
			}

			Q_strncpyz( cgs.weaponModels[cgs.numWeaponModels], name.data() + 1, sizeof( cgs.weaponModels[cgs.numWeaponModels] ) );
			cgs.numWeaponModels++;
		} else if( name.startsWith( '$' ) ) {
			if( !CG_LoadingItemName( name.data() ) ) {
				return;
			}

			// indexed pmodel
			cgs.pModelsIndex[i] = CG_RegisterPlayerModel( name.data() + 1 );
		} else {
			if( !CG_LoadingItemName( name.data() ) ) {
				return;
			}
			cgs.modelDraw[i] = CG_RegisterModel( name.data() );
		}
	}

	if( cgs.precacheModelsStart != MAX_MODELS ) {
		return;
	}

	cgs.media.registerModels();

	CG_RegisterBasePModel(); // never before registering the weapon models
	CG_RegisterWeaponModels();

	// precache forcemodels if defined
	CG_RegisterForceModels();

	// create a tag to offset the weapon models when seen in the world as items
	VectorSet( cgs.weaponItemTag.origin, 0, 0, 0 );
	Matrix3_Copy( axis_identity, cgs.weaponItemTag.axis );
	VectorMA( cgs.weaponItemTag.origin, -14, &cgs.weaponItemTag.axis[AXIS_FORWARD], cgs.weaponItemTag.origin );
}

/*
* CG_RegisterSounds
*/
static void CG_RegisterSounds( void ) {
	if( cgs.precacheSoundsStart == MAX_SOUNDS ) {
		return;
	}

	if( !cgs.precacheSoundsStart ) {
		CG_LoadingString( "sounds" );

		cgs.precacheSoundsStart = 1;
	}

	for( unsigned i = cgs.precacheSoundsStart; i < MAX_SOUNDS; i++ ) {
		const auto maybeName = cgs.configStrings.getSound( i );
		if( !maybeName ) {
			cgs.precacheSoundsStart = MAX_SOUNDS;
			break;
		}

		const auto name = *maybeName;
		cgs.precacheSoundsStart = i;

		if( !name.startsWith( '*' ) ) {
			if( !CG_LoadingItemName( name.data() ) ) {
				return;
			}
			cgs.soundPrecache[i] = SoundSystem::instance()->registerSound( name.data() );
		}
	}

	if( cgs.precacheSoundsStart != MAX_SOUNDS ) {
		return;
	}

	cgs.media.registerSounds();
}

/*
* CG_RegisterShaders
*/
static void CG_RegisterShaders( void ) {
	if( cgs.precacheShadersStart == MAX_IMAGES ) {
		return;
	}

	if( !cgs.precacheShadersStart ) {
		CG_LoadingString( "shaders" );

		cgs.precacheShadersStart = 1;
	}

	for( unsigned i = cgs.precacheShadersStart; i < MAX_IMAGES; i++ ) {
		const auto maybeName = cgs.configStrings.getImage( i );
		if( !maybeName ) {
			cgs.precacheShadersStart = MAX_IMAGES;
			break;
		}

		const auto name = *maybeName;
		cgs.precacheShadersStart = i;

		if( !CG_LoadingItemName( name.data() ) ) {
			return;
		}

		if( strstr( name.data(), "correction/" ) ) { // HACK HACK HACK -- for color correction LUTs
			cgs.imagePrecache[i] = R_RegisterLinearPic( name.data() );
		} else {
			cgs.imagePrecache[i] = R_RegisterPic( name.data() );
		}
	}

	if( cgs.precacheShadersStart != MAX_IMAGES ) {
		return;
	}

	cgs.media.registerMaterials();
}

/*
* CG_RegisterSkinfiles
*/
static void CG_RegisterSkinFiles( void ) {
	if( cgs.precacheSkinsStart == MAX_SKINFILES ) {
		return;
	}

	if( !cgs.precacheSkinsStart ) {
		CG_LoadingString( "skins" );

		cgs.precacheSkinsStart = 1;
	}

	for( unsigned i = cgs.precacheSkinsStart; i < MAX_SKINFILES; i++ ) {
		const auto maybeName = cgs.configStrings.getSkinFile( i );
		if( !maybeName ) {
			cgs.precacheSkinsStart = MAX_SKINFILES;
			break;
		}

		const auto name = *maybeName;
		cgs.precacheSkinsStart = i;

		if( !CG_LoadingItemName( name.data() ) ) {
			return;
		}

		cgs.skinPrecache[i] = R_RegisterSkinFile( name.data() );
	}

	cgs.precacheSkinsStart = MAX_SKINFILES;
}

/*
* CG_RegisterClients
*/
static void CG_RegisterClients( void ) {
	if( cgs.precacheClientsStart == MAX_CLIENTS ) {
		return;
	}

	if( !cgs.precacheClientsStart ) {
		CG_LoadingString( "clients" );
	}

	for( unsigned i = cgs.precacheClientsStart; i < MAX_CLIENTS; i++ ) {
		const auto maybeString = cgs.configStrings.getPlayerInfo( i );
		cgs.precacheClientsStart = i;

		if( !maybeString ) {
			continue;
		}
		if( !CG_LoadingItemName( maybeString->data() ) ) {
			return;
		}

		CG_LoadClientInfo( i, *maybeString );
	}

	cgs.precacheClientsStart = MAX_CLIENTS;
}

/*
* CG_RegisterLightStyles
*/
static void CG_RegisterLightStyles( void ) {
	for( unsigned i = 0; i < MAX_LIGHTSTYLES; i++ ) {
		if( auto maybeString = cgs.configStrings.getLightStyle( i ) ) {
			CG_SetLightStyle( i, *maybeString );
		} else {
			break;
		}
	}
}

/*
* CG_RegisterVariables
*/
static void CG_RegisterVariables( void ) {
	cg_predict =        Cvar_Get( "cg_predict", "1", 0 );
	cg_predict_optimize = Cvar_Get( "cg_predict_optimize", "1", 0 );
	cg_showMiss =       Cvar_Get( "cg_showMiss", "0", 0 );

	cg_debugPlayerModels =  Cvar_Get( "cg_debugPlayerModels", "0", CVAR_CHEAT | CVAR_ARCHIVE );
	cg_debugWeaponModels =  Cvar_Get( "cg_debugWeaponModels", "0", CVAR_CHEAT | CVAR_ARCHIVE );

	cg_model =          Cvar_Get( "model", DEFAULT_PLAYERMODEL, CVAR_USERINFO | CVAR_ARCHIVE );
	cg_skin =           Cvar_Get( "skin", DEFAULT_PLAYERSKIN, CVAR_USERINFO | CVAR_ARCHIVE );
	cg_hand =           Cvar_Get( "hand", "0", CVAR_USERINFO | CVAR_ARCHIVE );
	cg_handicap =       Cvar_Get( "handicap", "0", CVAR_USERINFO | CVAR_ARCHIVE );
	cg_clan =           Cvar_Get( "clan", "", CVAR_USERINFO | CVAR_ARCHIVE );
	// Should be 1 by default so we don't have to explain every newcomer how to turn it on. They are a majority.
	cg_movementStyle =  Cvar_Get( "cg_movementStyle", "1", CVAR_USERINFO | CVAR_ARCHIVE );
	cg_noAutohop =  Cvar_Get( "cg_noAutohop", "0", CVAR_USERINFO | CVAR_ARCHIVE );
	cg_fov =        Cvar_Get( "fov", "100", CVAR_ARCHIVE );
	cg_zoomfov =    Cvar_Get( "zoomfov", "30", CVAR_ARCHIVE );

	cg_addDecals =      Cvar_Get( "cg_decals", "1", CVAR_ARCHIVE );

	//cg_footSteps =	    Cvar_Get( "cg_footSteps", "1", 0 );

	cg_thirdPerson =    Cvar_Get( "cg_thirdPerson", "0", CVAR_CHEAT );
	cg_thirdPersonAngle =   Cvar_Get( "cg_thirdPersonAngle", "0", 0 );
	cg_thirdPersonRange =   Cvar_Get( "cg_thirdPersonRange", "90", 0 );

	cg_colorCorrection = Cvar_Get( "cg_colorCorrection", "1", CVAR_ARCHIVE );

	cg_gun =        Cvar_Get( "cg_gun", "1", CVAR_ARCHIVE );
	cg_gunx =       Cvar_Get( "cg_gunx", "0", CVAR_ARCHIVE );
	cg_guny =       Cvar_Get( "cg_guny", "0", CVAR_ARCHIVE );
	cg_gunz =       Cvar_Get( "cg_gunz", "0", CVAR_ARCHIVE );
	cg_gunbob =     Cvar_Get( "cg_gunbob", "1", CVAR_ARCHIVE );

	cg_gun_fov =        Cvar_Get( "cg_gun_fov", "75", CVAR_ARCHIVE );
	cg_gun_alpha =      Cvar_Get( "cg_gun_alpha", "1", CVAR_ARCHIVE );
	cg_weaponFlashes =  Cvar_Get( "cg_weaponFlashes", "2", CVAR_ARCHIVE );

	// wsw
	cg_handOffset =     Cvar_Get( "cg_handOffset", "5", CVAR_ARCHIVE );
	cg_projectileSmokeTrail =    Cvar_Get( "cg_projectileSmokeTrail", "1", CVAR_ARCHIVE );
	cg_projectileFireTrail = Cvar_Get( "cg_projectileFireTrail", "1", CVAR_ARCHIVE );
	cg_projectilePolyTrail = Cvar_Get( "cg_projectilePolyTrail", "1", CVAR_ARCHIVE );
	cg_plasmaTrail = Cvar_Get( "cg_plasmaTrail", "1", CVAR_ARCHIVE );
	cg_playerTrail = Cvar_Get( "cg_playerTrail", "1", CVAR_ARCHIVE );
	cg_bloodTrailTime    =   Cvar_Get( "cg_bloodTrailTime", "300", CVAR_ARCHIVE );
	cg_bloodTrailPalette = Cvar_Get( "cg_bloodTrailPalette", "1", CVAR_ARCHIVE );
	cg_showPOVBlood      = Cvar_Get( "cg_showPOVBlood", "1", CVAR_ARCHIVE );
	cg_projectileFireTrailAlpha =   Cvar_Get( "cg_projectileFireTrailAlpha", "0.45", CVAR_ARCHIVE );
	cg_gibs =       Cvar_Get( "cg_gibs", "0", CVAR_DEVELOPER );
	cg_outlineModels =  Cvar_Get( "cg_outlineModels", "0", CVAR_ARCHIVE );
	cg_outlineWorld =   Cvar_Get( "cg_outlineWorld", "0", CVAR_ARCHIVE );
	cg_outlinePlayers = Cvar_Get( "cg_outlinePlayers", "0", CVAR_ARCHIVE );
	cg_drawEntityBoxes =    Cvar_Get( "cg_drawEntityBoxes", "0", CVAR_DEVELOPER );
	cg_autoaction_demo =    Cvar_Get( "cg_autoaction_demo", "0", CVAR_ARCHIVE );
	cg_autoaction_screenshot =  Cvar_Get( "cg_autoaction_screenshot", "0", CVAR_ARCHIVE );
	cg_autoaction_stats =   Cvar_Get( "cg_autoaction_stats", "0", CVAR_ARCHIVE );
	cg_autoaction_spectator = Cvar_Get( "cg_autoaction_spectator", "0", CVAR_ARCHIVE );
	cg_simpleItems =    Cvar_Get( "cg_simpleItems", "0", CVAR_ARCHIVE );
	cg_simpleItemsSize =    Cvar_Get( "cg_simpleItemsSize", "16", CVAR_ARCHIVE );
	cg_particles =      Cvar_Get( "cg_particles", "1", CVAR_ARCHIVE );
	cg_showhelp =       Cvar_Get( "cg_showhelp", "1", CVAR_ARCHIVE );
	cg_predictLaserBeam =   Cvar_Get( "cg_predictLaserBeam", "1", CVAR_ARCHIVE );
	cg_showSelfShadow = Cvar_Get( "cg_showSelfShadow", "0", CVAR_ARCHIVE );

    debug_impact = Cvar_Get("debug_impact", "0.0", CVAR_CHEAT | CVAR_ARCHIVE);

	cg_cartoonEffects =     Cvar_Get( "cg_cartoonEffects", "7", CVAR_ARCHIVE );
	cg_cartoonHitEffect =   Cvar_Get( "cg_cartoonHitEffect", "1", CVAR_ARCHIVE );

	cg_damage_indicator =   Cvar_Get( "cg_damage_indicator", "1", CVAR_ARCHIVE );
	cg_damage_indicator_time =  Cvar_Get( "cg_damage_indicator_time", "25", CVAR_ARCHIVE );
	cg_pickup_flash =   Cvar_Get( "cg_pickup_flash", "0", CVAR_ARCHIVE );

	cg_weaponAutoSwitch =   Cvar_Get( "cg_weaponAutoSwitch", "2", CVAR_ARCHIVE );

	cg_voiceChats =     Cvar_Get( "cg_voiceChats", "1", CVAR_ARCHIVE );

	cg_laserBeamSubdivisions = Cvar_Get( "cg_laserBeamSubdivisions", "24", CVAR_ARCHIVE );
	cg_projectileAntilagOffset = Cvar_Get( "cg_projectileAntilagOffset", "1.0", CVAR_ARCHIVE );

	cg_raceGhosts =     Cvar_Get( "cg_raceGhosts", "0", CVAR_ARCHIVE );
	cg_raceGhostsAlpha =    Cvar_Get( "cg_raceGhostsAlpha", "0.25", CVAR_ARCHIVE );

	cg_chatBeep =       Cvar_Get( "cg_chatBeep", "1", CVAR_ARCHIVE );
	cg_chatFilter =     Cvar_Get( "cg_chatFilter", "0", CVAR_ARCHIVE | CVAR_USERINFO );
	cg_chatShowIgnored =   Cvar_Get( "cg_chatShowIgnored", "1", CVAR_ARCHIVE );

	// developer cvars
	cg_showClamp =      Cvar_Get( "cg_showClamp", "0", CVAR_DEVELOPER );

	// dmh - learn0more's team colored beams
	cg_teamColoredBeams = Cvar_Get( "cg_teamColoredBeams", "0", CVAR_ARCHIVE );
	cg_teamColoredInstaBeams = Cvar_Get( "cg_teamColoredInstaBeams", "1", CVAR_ARCHIVE );

	cg_ebbeam_width = Cvar_Get( "cg_ebbeam_width", "64", CVAR_ARCHIVE );
	cg_ebbeam_time = Cvar_Get( "cg_ebbeam_time", "0.4", CVAR_ARCHIVE );

	cg_instabeam_width = Cvar_Get( "cg_instabeam_width", "7", CVAR_ARCHIVE );
	cg_instabeam_time = Cvar_Get( "cg_instabeam_time", "0.4", CVAR_ARCHIVE );

	cg_playList = Cvar_Get( "cg_playList", S_PLAYLIST_MATCH, CVAR_ARCHIVE );
	cg_playListShuffle = Cvar_Get( "cg_playListShuffle", "1", CVAR_ARCHIVE );

	cg_flashWindowCount = Cvar_Get( "cg_flashWindowCount", "4", CVAR_ARCHIVE );

	cg_viewBob = Cvar_Get( "cg_viewBob", "1", CVAR_ARCHIVE );
}

/*
* CG_ValidateItemDef
*
* Compares name and tag against the itemlist to make sure cgame and game lists match
*/
void CG_ValidateItemDef( int tag, const char *name ) {
	gsitem_t *item;

	item = GS_FindItemByName( name );
	if( !item ) {
		CG_Error( "Client/Server itemlist missmatch (Game and Cgame version/mod differs). Item '%s' not found\n", name );
	}

	if( item->tag != tag ) {
		CG_Error( "Client/Server itemlist missmatch (Game and Cgame version/mod differs).\n" );
	}
}

/*
* CG_OverrideWeapondef
*
* Compares name and tag against the itemlist to make sure cgame and game lists match
*/
void CG_OverrideWeapondef( int index, const char *cstring ) {
	int weapon, i;
	bool strong, race;
	gs_weapon_definition_t *weapondef;
	firedef_t *firedef;

	// see G_PrecacheWeapondef, uses same operations
	weapon = index % ( MAX_WEAPONDEFS / 4 );
	race = index > ( MAX_WEAPONDEFS / 2 );
	strong = ( index % ( MAX_WEAPONDEFS / 2 ) ) > ( MAX_WEAPONDEFS / 4 );

	weapondef = GS_GetWeaponDefExt( weapon, race );
	if( !weapondef ) {
		CG_Error( "CG_OverrideWeapondef: Invalid weapon index\n" );
	}

	firedef = strong ? &weapondef->firedef : &weapondef->firedef_weak;

	i = sscanf( cstring, "%7i %7i %7u %7u %7u %7u %7u %7i %7i %7i",
				&firedef->usage_count,
				&firedef->projectile_count,
				&firedef->weaponup_time,
				&firedef->weapondown_time,
				&firedef->reload_time,
				&firedef->cooldown_time,
				&firedef->timeout,
				&firedef->speed,
				&firedef->spread,
				&firedef->v_spread
				);

	if( i != 10 ) {
		CG_Error( "CG_OverrideWeapondef: Bad configstring: %s \"%s\" (%i)\n", weapondef->name, cstring, i );
	}
}

/*
* CG_ValidateItemList
*/
static void CG_ValidateItemList( void ) {
	for( unsigned i = 0; i < MAX_ITEMS; i++ ) {
		if( const auto s = cgs.configStrings.getItem( i ) ) {
			CG_ValidateItemDef( i, s->data() );
		}
	}

	for( unsigned i = 0; i < MAX_WEAPONDEFS; i++ ) {
		if( const auto s = cgs.configStrings.getWeaponDef( i ) ) {
			CG_OverrideWeapondef( i, s->data() );
		}
	}
}

/*
* CG_Precache
*/
void CG_Precache( void ) {
	if( cgs.precacheDone ) {
		return;
	}

	cgs.precacheStart = cgs.precacheCount;
	cgs.precacheStartMsec = Sys_Milliseconds();

	CG_RegisterModels();
	if( cgs.precacheModelsStart < MAX_MODELS ) {
		return;
	}

	CG_RegisterSounds();
	if( cgs.precacheSoundsStart < MAX_SOUNDS ) {
		return;
	}

	CG_RegisterShaders();
	if( cgs.precacheShadersStart < MAX_IMAGES ) {
		return;
	}

	CG_RegisterSkinFiles();
	if( cgs.precacheSkinsStart < MAX_SKINFILES ) {
		return;
	}

	CG_RegisterClients();
	if( cgs.precacheClientsStart < MAX_CLIENTS ) {
		return;
	}

	cgs.precacheDone = true;
}

/*
* CG_RegisterConfigStrings
*/
static void CG_RegisterConfigStrings( void ) {
	cgs.precacheCount = cgs.precacheTotal = 0;

	for( unsigned i = 0; i < MAX_CONFIGSTRINGS; i++ ) {
		const auto maybeString = cl.configStrings.get( i );
		if( maybeString ) {
			cgs.configStrings.set( i, *maybeString );
		} else {
			// TODO: Add clear() call or set( unsinged, std::optional... ) call?
			cgs.configStrings.set( i, wsw::StringView() );
			continue;
		}

		if( i == CS_WORLDMODEL ) {
			cgs.precacheTotal++;
		} else if( i >= CS_MODELS && i < CS_MODELS + MAX_MODELS ) {
			cgs.precacheTotal++;
		} else if( i >= CS_SOUNDS && i < CS_SOUNDS + MAX_SOUNDS ) {
			cgs.precacheTotal++;
		} else if( i >= CS_IMAGES && i < CS_IMAGES + MAX_IMAGES ) {
			cgs.precacheTotal++;
		} else if( i >= CS_SKINFILES && i < CS_SKINFILES + MAX_SKINFILES ) {
			cgs.precacheTotal++;
		} else if( i >= CS_PLAYERINFOS && i < CS_PLAYERINFOS + MAX_CLIENTS ) {
			cgs.precacheTotal++;
		}
	}

	// backup initial configstrings for CG_Reset
	cgs.baseConfigStrings.copyFrom( cgs.configStrings );

	GS_SetGametypeName( cgs.configStrings.getGametypeName().value_or( wsw::StringView() ).data() );

	Cbuf_ExecuteText( EXEC_NOW, va( "exec configs/client/%s.cfg silent", gs.gametypeName ) );

	CG_SC_AutoRecordAction( cgs.configStrings.getAutoRecordState().value_or( wsw::StringView() ).data() );
}

/*
* CG_StartBackgroundTrack
*/
void CG_StartBackgroundTrack( void ) {
	if( const auto maybeConfigString = cgs.configStrings.getAudioTrack() ) {
		if( const auto configString = *maybeConfigString; configString.length() < 256 ) {
			wsw::StaticString<256> buffer( configString );
			char intro[MAX_QPATH], loop[MAX_QPATH];
			char *string = buffer.data();
			Q_strncpyz( intro, COM_Parse( &string ), sizeof( intro ) );
			Q_strncpyz( loop, COM_Parse( &string ), sizeof( loop ) );
			if( intro[0] ) {
				SoundSystem::instance()->startBackgroundTrack( intro, loop, 0 );
				return;
			}
		}
	}

	if( cg_playList->string[0] ) {
		SoundSystem::instance()->startBackgroundTrack( cg_playList->string, NULL, cg_playListShuffle->integer ? 1 : 0 );
	}
}

/*
* CG_Reset
*/
void CG_Reset( void ) {
	cgs.configStrings.copyFrom( cgs.baseConfigStrings );

	CG_ResetClientInfos();

	CG_ResetPModels();

	CG_ResetKickAngles();
	CG_ResetColorBlend();
	CG_ResetDamageIndicator();
	CG_ResetItemTimers();

	CG_SC_ResetFragsFeed();

	CG_ClearLocalEntities();

	// start up announcer events queue from clean
	CG_ClearAnnouncerEvents();

	CG_ClearInputState();

	CG_ClearPointedNum();

	CG_ClearChaseCam();

	cg.time = 0;
	cg.realTime = 0;
	cg.helpmessage_time = 0;
	cg.motd_time = 0;

	chaseCam.cmd_mode_delay = 0; // cg.time

	// reset prediction optimization
	cg.predictFrom = 0;

	memset( cg_entities, 0, sizeof( cg_entities ) );
}

void CG_InitPersistentState() {
	CG_InitInputVars();

	//team models
	cg_teamPLAYERSmodel = Cvar_Get( "cg_teamPLAYERSmodel", DEFAULT_TEAMPLAYERS_MODEL, CVAR_ARCHIVE );
	cg_teamPLAYERSmodelForce = Cvar_Get( "cg_teamPLAYERSmodelForce", "0", CVAR_ARCHIVE );
	cg_teamPLAYERSskin = Cvar_Get( "cg_teamPLAYERSskin", DEFAULT_PLAYERSKIN, CVAR_ARCHIVE );
	cg_teamPLAYERScolor = Cvar_Get( "cg_teamPLAYERScolor", DEFAULT_TEAMBETA_COLOR, CVAR_ARCHIVE );
	cg_teamPLAYERScolorForce = Cvar_Get( "cg_teamPLAYERScolorForce", "0", CVAR_ARCHIVE );
	cg_teamPLAYERSmodel->modified = true;
	cg_teamPLAYERSmodelForce->modified = true;
	cg_teamPLAYERSskin->modified = true;
	cg_teamPLAYERScolor->modified = true;
	cg_teamPLAYERScolorForce->modified = true;

	cg_teamALPHAmodel = Cvar_Get( "cg_teamALPHAmodel", DEFAULT_TEAMALPHA_MODEL, CVAR_ARCHIVE );
	cg_teamALPHAmodelForce = Cvar_Get( "cg_teamALPHAmodelForce", "1", CVAR_ARCHIVE );
	cg_teamALPHAskin = Cvar_Get( "cg_teamALPHAskin", DEFAULT_PLAYERSKIN, CVAR_ARCHIVE );
	cg_teamALPHAcolor = Cvar_Get( "cg_teamALPHAcolor", DEFAULT_TEAMALPHA_COLOR, CVAR_ARCHIVE );
	cg_teamALPHAmodel->modified = true;
	cg_teamALPHAmodelForce->modified = true;
	cg_teamALPHAskin->modified = true;
	cg_teamALPHAcolor->modified = true;

	cg_teamBETAmodel = Cvar_Get( "cg_teamBETAmodel", DEFAULT_TEAMBETA_MODEL, CVAR_ARCHIVE );
	cg_teamBETAmodelForce = Cvar_Get( "cg_teamBETAmodelForce", "1", CVAR_ARCHIVE );
	cg_teamBETAskin = Cvar_Get( "cg_teamBETAskin", DEFAULT_PLAYERSKIN, CVAR_ARCHIVE );
	cg_teamBETAcolor = Cvar_Get( "cg_teamBETAcolor", DEFAULT_TEAMBETA_COLOR, CVAR_ARCHIVE );
	cg_teamBETAmodel->modified = true;
	cg_teamBETAmodelForce->modified = true;
	cg_teamBETAskin->modified = true;
	cg_teamBETAcolor->modified = true;

	cg_forceMyTeamAlpha = Cvar_Get( "cg_forceMyTeamAlpha", "0", CVAR_ARCHIVE );

	cg_viewSize =       Cvar_Get( "cg_viewSize", "100", CVAR_ARCHIVE );
	cg_showFPS =        Cvar_Get( "cg_showFPS", "0", CVAR_ARCHIVE );
	cg_draw2D =     Cvar_Get( "cg_draw2D", "1", 0 );

	cg_showViewBlends = Cvar_Get( "cg_showViewBlends", "1", CVAR_ARCHIVE );
	cg_showZoomEffect = Cvar_Get( "cg_showZoomEffect", "1", CVAR_ARCHIVE );

	cg_showHUD =        Cvar_Get( "cg_showHUD", "1", CVAR_ARCHIVE );

	cg_centerTime =     Cvar_Get( "cg_centerTime", "2.5", 0 );

	cg_showPointedPlayer =  Cvar_Get( "cg_showPointedPlayer", "1", CVAR_ARCHIVE );

	cg_showPlayerNames =        Cvar_Get( "cg_showPlayerNames", "1", CVAR_ARCHIVE );
	cg_showPlayerNames_alpha =  Cvar_Get( "cg_showPlayerNames_alpha", "0.4", CVAR_ARCHIVE );
	cg_showPlayerNames_zfar =   Cvar_Get( "cg_showPlayerNames_zfar", "1024", CVAR_ARCHIVE );
	cg_showPlayerNames_barWidth =   Cvar_Get( "cg_showPlayerNames_barWidth", "8", CVAR_ARCHIVE );
	cg_showTeamInfo =      Cvar_Get( "cg_showTeamInfo", "1", CVAR_ARCHIVE );

	cg_showPressedKeys = Cvar_Get( "cg_showPressedKeys", "0", CVAR_ARCHIVE );

	cg_showSpeed =      Cvar_Get( "cg_showSpeed", "1", CVAR_ARCHIVE );
	cg_showPickup =     Cvar_Get( "cg_showPickup", "1", CVAR_ARCHIVE );

	cg_showTimer =      Cvar_Get( "cg_showTimer", "1", CVAR_ARCHIVE );
	cg_showAwards =     Cvar_Get( "cg_showAwards", "1", CVAR_ARCHIVE );

	cg_showFragsFeed = Cvar_Get( "cg_showFragsFeed", "1", CVAR_ARCHIVE );

	cg_showMessageFeed = Cvar_Get( "cg_showMessageFeed", "1", CVAR_ARCHIVE );

	cg_showminimap = Cvar_Get( "cg_showMiniMap", "0", CVAR_ARCHIVE );
	cg_showitemtimers = Cvar_Get( "cg_showItemTimers", "3", CVAR_ARCHIVE );

	cg_showCaptureAreas = Cvar_Get( "cg_showCaptureAreas", "1", CVAR_ARCHIVE );

	cg_showChasers = Cvar_Get( "cg_showChasers", "1", CVAR_ARCHIVE );

	cg_volume_players = Cvar_Get( "cg_volume_players", "1.0", CVAR_ARCHIVE );
	cg_volume_effects = Cvar_Get( "cg_volume_effects", "1.0", CVAR_ARCHIVE );
	cg_volume_announcer =   Cvar_Get( "cg_volume_announcer", "1.0", CVAR_ARCHIVE );
	cg_volume_hitsound =    Cvar_Get( "cg_volume_hitsound", "1.0", CVAR_ARCHIVE );

	cg_heavyRocketExplosions = Cvar_Get( "cg_heavyRocketExplosions", "1", CVAR_ARCHIVE );
	cg_heavyGrenadeExplosions = Cvar_Get( "cg_heavyGrenadeExplosions", "1", CVAR_ARCHIVE );
	cg_heavyShockwaveExplosions = Cvar_Get( "cg_heavyShockwaveExplosions", "1", CVAR_ARCHIVE );

	cg_explosionsWave = Cvar_Get( "cg_explosionsWave", "1", CVAR_ARCHIVE );
	cg_explosionsSmoke = Cvar_Get( "cg_explosionsSmoke", "1", CVAR_ARCHIVE );
	cg_explosionsClusters = Cvar_Get( "cg_explosionsClusters", "1", CVAR_ARCHIVE );

	CrosshairState::initPersistentState();

	CG_InitTemporaryBoneposesCache();
}

/*
* CG_Init
*/
void CG_Init( const char *serverName, unsigned int playerNum,
			  int vidWidth, int vidHeight, float pixelRatio,
			  bool demoplaying, const char *demoName, bool pure,
			  unsigned snapFrameTime, int protocol, const char *demoExtension,
			  int sharedSeed, bool gameStart ) {
	CG_InitGameShared();

	// Hacks, see below
	cg.~cg_state_t();
	memset( &cg, 0, sizeof( cg_state_t ) );
	new( &cg )cg_state_t;

	// Hacks, see a related comment in the client/ code
	cgs.~cg_static_t();
	memset( (void *)&cgs, 0, sizeof( cg_static_t ) );
	new( &cgs )cg_static_t;

	memset( cg_entities, 0, sizeof( cg_entities ) );

	srand( time( NULL ) );

	// save server name
	cgs.serverName = Q_strdup( serverName );

	// save local player number
	cgs.playerNum = playerNum;

	// save current width and height
	cgs.vidWidth = vidWidth;
	cgs.vidHeight = vidHeight;
	cgs.pixelRatio = pixelRatio;

	// demo
	cgs.demoPlaying = demoplaying == true;
	cgs.demoName = demoName;
	Q_strncpyz( cgs.demoExtension, demoExtension, sizeof( cgs.demoExtension ) );

	// whether to only allow pure files
	cgs.pure = pure == true;

	// game protocol number
	cgs.gameProtocol = protocol;
	cgs.snapFrameTime = snapFrameTime;

	cgs.hasGametypeMenu = false; // this will update as soon as we receive configstrings
	cgs.gameMenuRequested = !gameStart;

	cgs.fullclipShaderNum = CM_ShaderrefForName( cl.cms, "textures/common/fullclip" );

	CG_InitInput();

	CG_RegisterVariables();
	CG_PModelsInit();
	CG_WModelsInit();

	CrosshairState::handleCGameInit();

	CG_ClearLightStyles();

	CG_ClearLocalEntities();

	// get configstrings
	CG_RegisterConfigStrings();

	// register fonts here so loading screen works
	CG_RegisterFonts();
	cgs.shaderWhite = R_RegisterPic( "$whiteimage" );

	CG_RegisterLevelMinimap();

	CG_RegisterCGameCommands();
	CG_RegisterLightStyles();

	CG_ValidateItemList();

	CG_LoadingString( "" );

	CG_ClearChaseCam();

	// start up announcer events queue from clean
	CG_ClearAnnouncerEvents();

	cgs.demoTutorial = cgs.demoPlaying && ( strstr( cgs.demoName, "tutorials/" ) != NULL );

	cg.firstFrame = true; // think of the next frame in CG_NewFrameSnap as of the first one

	// now that we're done with precaching, let the autorecord actions do something
	CG_ConfigString( CS_AUTORECORDSTATE, cgs.configStrings.get( CS_AUTORECORDSTATE ).value_or( wsw::StringView() ) );

	CG_DemocamInit();
}

/*
* CG_Shutdown
*/
void CG_Shutdown( void ) {
	CG_SC_ResetFragsFeed();
	CG_FreeLocalEntities();
	CG_DemocamShutdown();
	CG_UnregisterCGameCommands();
	CrosshairState::handleCGameShutdown();
	CG_WModelsShutdown();
	CG_PModelsShutdown();
	CG_FreeTemporaryBoneposesCache();
	CG_ShutdownInput();
}
