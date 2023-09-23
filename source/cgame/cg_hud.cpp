/*
Copyright (C) 2006 Pekka Lampila ("Medar"), Damien Deville ("Pb")
and German Garcia Fernandez ("Jal") for Chasseur de bots association.


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
#include "../client/client.h"
#include "../ui/uisystem.h"
#include "../ui/huddatamodel.h"

extern cvar_t *cg_showHUD;
extern cvar_t *cg_showTeamInfo;
extern cvar_t *cg_showTimer;
extern cvar_t *cg_showPlayerNames;
extern cvar_t *cg_showPlayerNames_alpha;
extern cvar_t *cg_showPlayerNames_barWidth;
extern cvar_t *cg_showPlayerNames_zfar;
extern cvar_t *cg_showPointedPlayer;

void CG_CenterPrint( const char *str ) {
	if( str && *str ) {
		wsw::ui::UISystem::instance()->addStatusMessage( wsw::StringView( str ) );
	}
}

int CG_HorizontalAlignForWidth( const int x, int align, int width ) {
	int nx = x;

	if( align % 3 == 0 ) { // left
		nx = x;
	}
	if( align % 3 == 1 ) { // center
		nx = x - width / 2;
	}
	if( align % 3 == 2 ) { // right
		nx = x - width;
	}

	return nx;
}

int CG_VerticalAlignForHeight( const int y, int align, int height ) {
	int ny = y;

	if( align / 3 == 0 ) { // top
		ny = y;
	} else if( align / 3 == 1 ) { // middle
		ny = y - height / 2;
	} else if( align / 3 == 2 ) { // bottom
		ny = y - height;
	}

	return ny;
}

static void CG_DrawHUDRect( int x, int y, int align, int w, int h, int val, int maxval, vec4_t color, struct shader_s *shader ) {
	float frac;
	vec2_t tc[2];

	if( val < 1 || maxval < 1 || w < 1 || h < 1 ) {
		return;
	}

	if( !shader ) {
		shader = cgs.shaderWhite;
	}

	if( val >= maxval ) {
		frac = 1.0f;
	} else {
		frac = (float)val / (float)maxval;
	}

	tc[0][0] = 0.0f;
	tc[0][1] = 1.0f;
	tc[1][0] = 0.0f;
	tc[1][1] = 1.0f;
	if( h > w ) {
		h = (int)( (float)h * frac + 0.5 );
		if( align / 3 == 0 ) { // top
			tc[1][1] = 1.0f * frac;
		} else if( align / 3 == 1 ) {   // middle
			tc[1][0] = ( 1.0f - ( 1.0f * frac ) ) * 0.5f;
			tc[1][1] = ( 1.0f * frac ) * 0.5f;
		} else if( align / 3 == 2 ) {   // bottom
			tc[1][0] = 1.0f - ( 1.0f * frac );
		}
	} else {
		w = (int)( (float)w * frac + 0.5 );
		if( align % 3 == 0 ) { // left
			tc[0][1] = 1.0f * frac;
		}
		if( align % 3 == 1 ) { // center
			tc[0][0] = ( 1.0f - ( 1.0f * frac ) ) * 0.5f;
			tc[0][1] = ( 1.0f * frac ) * 0.5f;
		}
		if( align % 3 == 2 ) { // right
			tc[0][0] = 1.0f - ( 1.0f * frac );
		}
	}

	x = CG_HorizontalAlignForWidth( x, align, w );
	y = CG_VerticalAlignForHeight( y, align, h );

	R_DrawStretchPic( x, y, w, h, tc[0][0], tc[1][0], tc[0][1], tc[1][1], color, shader );
}

void CG_ScreenCrosshairDamageUpdate() {
	cg.crosshairState.touchDamageState();
	cg.strongCrosshairState.touchDamageState();
}

static void drawCrosshair( CrosshairState *state ) {
	if( auto maybeMaterialAndDimensions = state->getDrawingMaterial() ) {
		auto [material, width, height] = *maybeMaterialAndDimensions;
		const int x = ( cgs.vidWidth - (int)width ) / 2;
		const int y = ( cgs.vidHeight - (int)height ) / 2;
		R_DrawStretchPic( x, y, (int)width, (int)height, 0, 0, 1, 1, state->getDrawingColor(), material );
	}
}

void CG_UpdateCrosshair() {
	CrosshairState::updateSharedPart();
	if( unsigned weapon = cg.predictedPlayerState.stats[STAT_WEAPON] ) {
		cg.crosshairState.update( weapon );
		cg.strongCrosshairState.update( weapon );
	} else {
		cg.crosshairState.clear();
		cg.strongCrosshairState.clear();
	}
}

void CG_DrawCrosshair() {
	const auto *const playerState = &cg.predictFromPlayerState;
	const auto weapon = playerState->stats[STAT_WEAPON];
	if( !weapon ) {
		return;
	}

	const auto *const firedef = GS_FiredefForPlayerState( playerState, weapon );
	if( !firedef ) {
		return;
	}

	if( cg.strongCrosshairState.canBeDrawn() && ( firedef->fire_mode == FIRE_MODE_STRONG ) ) {
		::drawCrosshair( &cg.strongCrosshairState );
	}
	if( cg.crosshairState.canBeDrawn() ) {
		::drawCrosshair( &cg.crosshairState );
	}
}

/*
* CG_ClearPointedNum
*/
void CG_ClearPointedNum( void ) {
	cg.pointedNum = 0;
	cg.pointRemoveTime = 0;
	cg.pointedHealth = 0;
	cg.pointedArmor = 0;
}

/*
* CG_UpdatePointedNum
*/
static void CG_UpdatePointedNum( void ) {
	// disable cases
	if( CG_IsScoreboardShown() || cg.view.thirdperson || cg.view.type != VIEWDEF_PLAYERVIEW || !cg_showPointedPlayer->integer ) {
		CG_ClearPointedNum();
		return;
	}

	if( cg.predictedPlayerState.stats[STAT_POINTED_PLAYER] ) {
		bool mega = false;

		cg.pointedNum = cg.predictedPlayerState.stats[STAT_POINTED_PLAYER];
		cg.pointRemoveTime = cg.time + 150;

		cg.pointedHealth = 3.2 * ( cg.predictedPlayerState.stats[STAT_POINTED_TEAMPLAYER] & 0x1F );
		mega = cg.predictedPlayerState.stats[STAT_POINTED_TEAMPLAYER] & 0x20 ? true : false;
		cg.pointedArmor = 5 * ( cg.predictedPlayerState.stats[STAT_POINTED_TEAMPLAYER] >> 6 & 0x3F );
		if( mega ) {
			cg.pointedHealth += 100;
			if( cg.pointedHealth > 200 ) {
				cg.pointedHealth = 200;
			}
		}
	}

	if( cg.pointRemoveTime <= cg.time ) {
		CG_ClearPointedNum();
	}

	if( cg.pointedNum && cg_showPointedPlayer->integer == 2 ) {
		if( cg_entities[cg.pointedNum].current.team != cg.predictedPlayerState.stats[STAT_TEAM] ) {
			CG_ClearPointedNum();
		}
	}
}

void CG_DrawPlayerNames() {
	qfontface_s *font = cgs.fontSystemMedium;
	const float *color = colorWhite;
	static vec4_t alphagreen = { 0, 1, 0, 0 }, alphared = { 1, 0, 0, 0 }, alphayellow = { 1, 1, 0, 0 }, alphamagenta = { 1, 0, 1, 1 }, alphagrey = { 0.85, 0.85, 0.85, 1 };
	centity_t *cent;
	vec4_t tmpcolor;
	vec3_t dir, drawOrigin;
	vec2_t coords;
	float dist, fadeFrac;
	trace_t trace;
	int i;

	if( !cg_showPlayerNames->integer && !cg_showPointedPlayer->integer ) {
		return;
	}

	CG_UpdatePointedNum();

	for( i = 0; i < gs.maxclients; i++ ) {
		int pointed_health, pointed_armor;

		if( !cgs.clientInfo[i].name[0] || ISVIEWERENTITY( i + 1 ) ) {
			continue;
		}

		cent = &cg_entities[i + 1];
		if( cent->serverFrame != cg.frame.serverFrame ) {
			continue;
		}

		if( cent->current.effects & EF_PLAYER_HIDENAME ) {
			continue;
		}

		// only show the pointed player
		if( !cg_showPlayerNames->integer && ( cent->current.number != cg.pointedNum ) ) {
			continue;
		}

		if( ( cg_showPlayerNames->integer == 2 ) && ( cent->current.team != cg.predictedPlayerState.stats[STAT_TEAM] ) ) {
			continue;
		}

		if( !cent->current.modelindex || !cent->current.solid ||
			cent->current.solid == SOLID_BMODEL || cent->current.team == TEAM_SPECTATOR ) {
			continue;
		}

		// Kill if behind the view
		VectorSubtract( cent->ent.origin, cg.view.origin, dir );
		dist = VectorNormalize( dir ) * cg.view.fracDistFOV;

		if( DotProduct( dir, &cg.view.axis[AXIS_FORWARD] ) < 0 ) {
			continue;
		}

		Vector4Copy( color, tmpcolor );

		if( cent->current.number != cg.pointedNum ) {
			if( dist > cg_showPlayerNames_zfar->value ) {
				continue;
			}

			fadeFrac = ( cg_showPlayerNames_zfar->value - dist ) / ( cg_showPlayerNames_zfar->value * 0.25f );
			Q_clamp( fadeFrac, 0.0f, 1.0f );

			tmpcolor[3] = cg_showPlayerNames_alpha->value * color[3] * fadeFrac;
		} else {
			fadeFrac = (float)( cg.pointRemoveTime - cg.time ) / 150.0f;
			Q_clamp( fadeFrac, 0.0f, 1.0f );

			tmpcolor[3] = color[3] * fadeFrac;
		}

		if( tmpcolor[3] <= 0.0f ) {
			continue;
		}

		CG_Trace( &trace, cg.view.origin, vec3_origin, vec3_origin, cent->ent.origin, cg.predictedPlayerState.POVnum, MASK_OPAQUE );
		if( trace.fraction < 1.0f && trace.ent != cent->current.number ) {
			continue;
		}

		VectorSet( drawOrigin, cent->ent.origin[0], cent->ent.origin[1], cent->ent.origin[2] + playerbox_stand_maxs[2] + 16 );

		// find the 3d point in 2d screen
		RF_TransformVectorToScreen( &cg.view.refdef, drawOrigin, coords );
		if( ( coords[0] < 0 || coords[0] > cgs.vidWidth ) || ( coords[1] < 0 || coords[1] > cgs.vidHeight ) ) {
			continue;
		}

		SCR_DrawString( coords[0], coords[1], ALIGN_CENTER_BOTTOM, cgs.clientInfo[i].name, font, tmpcolor );

		// if not the pointed player we are done
		if( cent->current.number != cg.pointedNum ) {
			continue;
		}

		pointed_health = cg.pointedHealth;
		pointed_armor = cg.pointedArmor;

		// pointed player hasn't a health value to be drawn, so skip adding the bars
		if( pointed_health && cg_showPlayerNames_barWidth->integer > 0 ) {
			int x, y;
			int barwidth = SCR_strWidth( "_", font, 0 ) * cg_showPlayerNames_barWidth->integer; // size of 8 characters
			int barheight = SCR_FontHeight( font ) * 0.25; // quarter of a character height
			int barseparator = barheight * 0.333;

			alphagreen[3] = alphared[3] = alphayellow[3] = alphamagenta[3] = alphagrey[3] = tmpcolor[3];

			// soften the alpha of the box color
			tmpcolor[3] *= 0.4f;

			// we have to align first, then draw as left top, cause we want the bar to grow from left to right
			x = CG_HorizontalAlignForWidth( coords[0], ALIGN_CENTER_TOP, barwidth );
			y = CG_VerticalAlignForHeight( coords[1], ALIGN_CENTER_TOP, barheight );

			// draw the background box
			CG_DrawHUDRect( x, y, ALIGN_LEFT_TOP, barwidth, barheight * 3, 100, 100, tmpcolor, NULL );

			y += barseparator;

			if( pointed_health > 100 ) {
				alphagreen[3] = alphamagenta[3] = 1.0f;
				CG_DrawHUDRect( x, y, ALIGN_LEFT_TOP, barwidth, barheight, 100, 100, alphagreen, NULL );
				CG_DrawHUDRect( x, y, ALIGN_LEFT_TOP, barwidth, barheight, pointed_health - 100, 100, alphamagenta, NULL );
				alphagreen[3] = alphamagenta[3] = alphared[3];
			} else {
				if( pointed_health <= 33 ) {
					CG_DrawHUDRect( x, y, ALIGN_LEFT_TOP, barwidth, barheight, pointed_health, 100, alphared, NULL );
				} else if( pointed_health <= 66 ) {
					CG_DrawHUDRect( x, y, ALIGN_LEFT_TOP, barwidth, barheight, pointed_health, 100, alphayellow, NULL );
				} else {
					CG_DrawHUDRect( x, y, ALIGN_LEFT_TOP, barwidth, barheight, pointed_health, 100, alphagreen, NULL );
				}
			}

			if( pointed_armor ) {
				y += barseparator + barheight;
				CG_DrawHUDRect( x, y, ALIGN_LEFT_TOP, barwidth, barheight, pointed_armor, 150, alphagrey, NULL );
			}
		}
	}
}

void CG_DrawTeamMates() {
	centity_t *cent;
	vec3_t dir, drawOrigin;
	vec2_t coords;
	vec4_t color;
	int i;
	int pic_size = 18 * cgs.vidHeight / 600;

	if( !cg_showTeamInfo->integer ) {
		return;
	}

	if( cg.predictedPlayerState.stats[STAT_TEAM] < TEAM_ALPHA ) {
		return;
	}

	for( i = 0; i < gs.maxclients; i++ ) {
		trace_t trace;

		if( !cgs.clientInfo[i].name[0] || ISVIEWERENTITY( i + 1 ) ) {
			continue;
		}

		cent = &cg_entities[i + 1];
		if( cent->serverFrame != cg.frame.serverFrame ) {
			continue;
		}

		if( cent->current.team != cg.predictedPlayerState.stats[STAT_TEAM] ) {
			continue;
		}

		VectorSet( drawOrigin, cent->ent.origin[0], cent->ent.origin[1], cent->ent.origin[2] + playerbox_stand_maxs[2] + 16 );
		VectorSubtract( drawOrigin, cg.view.origin, dir );

		// ignore, if not in view
		if( DotProduct( dir, &cg.view.axis[AXIS_FORWARD] ) < 0 ) {
			continue;
		}

		if( !cent->current.modelindex || !cent->current.solid ||
			cent->current.solid == SOLID_BMODEL || cent->current.team == TEAM_SPECTATOR ) {
			continue;
		}

		// find the 3d point in 2d screen
		RF_TransformVectorToScreen( &cg.view.refdef, drawOrigin, coords );
		if( ( coords[0] < 0 || coords[0] > cgs.vidWidth ) || ( coords[1] < 0 || coords[1] > cgs.vidHeight ) ) {
			continue;
		}

		CG_Trace( &trace, cg.view.origin, vec3_origin, vec3_origin, cent->ent.origin, cg.predictedPlayerState.POVnum, MASK_OPAQUE );
		if( cg_showTeamInfo->integer == 1 && trace.fraction == 1.0f ) {
			continue;
		}

		coords[0] -= pic_size / 2;
		coords[1] -= pic_size / 2;
		Q_clamp( coords[0], 0, cgs.vidWidth - pic_size );
		Q_clamp( coords[1], 0, cgs.vidHeight - pic_size );

		CG_TeamColor( cg.predictedPlayerState.stats[STAT_TEAM], color );

		shader_s *shader;
		if( cent->current.effects & EF_CARRIER ) {
			shader = cgs.media.shaderTeamCarrierIndicator;
		} else {
			shader = cgs.media.shaderTeamMateIndicator;
		}

		R_DrawStretchPic( coords[0], coords[1], pic_size, pic_size, 0, 0, 1, 1, color, shader );
	}
}

void CG_DrawHUD() {
	CG_UpdateCrosshair();

	if( !cg_showHUD->integer ) {
		return;
	}

	if( !CG_IsScoreboardShown() ) {
		CG_DrawTeamMates();
		CG_DrawPlayerNames();
	}

	// TODO: Does it work for chasers?
	if( cg.predictedPlayerState.pmove.pm_type == PM_NORMAL ) {
		if( !wsw::ui::UISystem::instance()->isShown() ) {
			CG_DrawCrosshair();
		}
	}
}

bool CG_IsSpectator() {
	return ISREALSPECTATOR();
}

std::optional<unsigned> CG_ActiveChasePov() {
	// Don't even bother in another case
	if( const unsigned statePovNum = cg.predictedPlayerState.POVnum; statePovNum > 0 ) {
		unsigned chosenPovNum = 0;
		if( !ISREALSPECTATOR() ) {
			chosenPovNum = statePovNum;
		} else {
			if( statePovNum != cg.predictedPlayerState.playerNum + 1 ) {
				chosenPovNum = statePovNum;
			}
		}
		// Protect against chasing other entities (is it a thing?)
		if( chosenPovNum && chosenPovNum < (unsigned)( gs.maxclients + 1 ) ) {
			return chosenPovNum - 1u;
		}
	}
	return std::nullopt;
}

bool CG_IsPovAlive() {
	return !( cg.predictedPlayerState.stats[STAT_FLAGS] & STAT_FLAG_DEADPOV );
}

bool CG_HasTwoTeams() {
	return GS_TeamBasedGametype();
}

bool CG_CanBeReady() {
	return !ISREALSPECTATOR() && ( GS_MatchState() == MATCH_STATE_WARMUP || GS_MatchState() == MATCH_STATE_COUNTDOWN );
}

bool CG_IsReady() {
	return ( cg.predictedPlayerState.stats[STAT_FLAGS] & STAT_FLAG_READY ) != 0;
}

bool CG_IsOperator() {
	return ( cg.predictedPlayerState.stats[STAT_FLAGS] & STAT_FLAG_OPERATOR ) != 0;
}

bool CG_IsChallenger() {
	return ( cg.predictedPlayerState.stats[STAT_FLAGS] & STAT_FLAG_CHALLENGER ) != 0;
}

int CG_MyRealTeam() {
	return cg.predictedPlayerState.stats[STAT_REALTEAM];
}

int CG_ActiveWeapon() {
	return cg.predictedPlayerState.stats[STAT_WEAPON];
}

bool CG_HasWeapon( int weapon ) {
	assert( (unsigned)weapon < (unsigned)WEAP_TOTAL );
	return cg.predictedPlayerState.inventory[weapon];
}

int CG_Health() {
	return cg.predictedPlayerState.stats[STAT_HEALTH];
}

int CG_Armor() {
	return cg.predictedPlayerState.stats[STAT_ARMOR];
}

[[nodiscard]]
static auto getTeamColor( int team ) -> int {
	vec4_t color;
	CG_TeamColor( team, color );
	return COLOR_RGB( (int)( color[0] * 255 ), (int)( color[1] * 255 ), (int)( color[2] * 255 ) );
}

int CG_TeamAlphaDisplayedColor() {
	return getTeamColor( TEAM_ALPHA );
}

int CG_TeamBetaDisplayedColor() {
	return getTeamColor( TEAM_BETA );
}

auto CG_HudIndicatorState( int num ) -> wsw::ui::ObjectiveIndicatorState {
	assert( (unsigned)num < 3 );
	const auto *const stats = cg.predictedPlayerState.stats;

	static_assert( (int)wsw::ui::HudDataModel::NoAnim == (int)HUD_INDICATOR_NO_ANIM );
	static_assert( (int)wsw::ui::HudDataModel::AlertAnim == (int)HUD_INDICATOR_ALERT_ANIM );
	static_assert( (int)wsw::ui::HudDataModel::ActionAnim == (int)HUD_INDICATOR_ACTION_ANIM );

	int anim = stats[STAT_INDICATOR_1_ANIM + num];
	if( (unsigned)anim > (unsigned)HUD_INDICATOR_ACTION_ANIM ) {
		anim = HUD_INDICATOR_NO_ANIM;
	}

	int iconNum = stats[STAT_INDICATOR_1_ICON + num];
	if( (unsigned)iconNum >= (unsigned)MAX_GENERAL ) {
		iconNum = 0;
	}

	const int progress = wsw::clamp<int>( stats[STAT_INDICATOR_1_PROGRESS + num], -100, +100 );
	const bool enabled = stats[STAT_INDICATOR_1_ENABLED + num] != 0;

	int packedColor = ~0;
	if( const int colorTeam = stats[STAT_INDICATOR_1_COLORTEAM + num] ) {
		if( colorTeam == TEAM_ALPHA ) {
			packedColor = CG_TeamAlphaDisplayedColor();
		} else if( colorTeam == TEAM_BETA ) {
			packedColor = CG_TeamBetaDisplayedColor();
		}
	}

	const QColor color( QColor::fromRgb( COLOR_R( packedColor ), COLOR_G( packedColor ), COLOR_B( packedColor ) ) );
	return { .color = color, .anim = anim, .progress = progress, .iconNum = iconNum, .enabled = enabled };
}

auto CG_HudIndicatorIconPath( int iconNum ) -> std::optional<wsw::StringView> {
	assert( (unsigned)iconNum <= (unsigned)CS_GENERAL );
	if( iconNum ) {
		return cgs.configStrings.get( (unsigned)( CS_GENERAL + iconNum - 1 ) );
	}
	return std::nullopt;
}

std::pair<int, int> CG_WeaponAmmo( int weapon ) {
	const auto *weaponDef = GS_GetWeaponDef( weapon );
	const int *inventory = cg.predictedPlayerState.inventory;
	return { inventory[weaponDef->firedef_weak.ammo_id], inventory[weaponDef->firedef.ammo_id] };
}

auto CG_GetMatchClockTime() -> std::pair<int, int> {
	int64_t clocktime, startTime, duration, curtime;
	double seconds;
	int minutes;

	if( !cg_showTimer ) {
		return { 0, 0 };
	}

	if( !cg_showTimer->integer ) {
		return { 0, 0 };
	}

	if( GS_MatchState() > MATCH_STATE_PLAYTIME ) {
		return { 0, 0 };
	}

	if( GS_RaceGametype() ) {
		if( cg.predictedPlayerState.stats[STAT_TIME_SELF] != STAT_NOTSET ) {
			clocktime = cg.predictedPlayerState.stats[STAT_TIME_SELF] * 100;
		} else {
			clocktime = 0;
		}
	} else if( GS_MatchClockOverride() ) {
		clocktime = GS_MatchClockOverride();
	} else {
		curtime = ( GS_MatchWaiting() || GS_MatchPaused() ) ? cg.frame.serverTime : cg.time;
		duration = GS_MatchDuration();
		startTime = GS_MatchStartTime();

		// count downwards when having a duration
		if( duration && ( cg_showTimer->integer != 3 ) ) {
			if( duration + startTime < curtime ) {
				duration = curtime - startTime; // avoid negative results

			}
			clocktime = startTime + duration - curtime;
		} else {
			if( curtime >= startTime ) { // avoid negative results
				clocktime = curtime - startTime;
			} else {
				clocktime = 0;
			}
		}
	}

	seconds = (double)clocktime * 0.001;
	minutes = (int)( seconds / 60 );
	seconds -= minutes * 60;

	return { minutes, seconds };
}

wsw::StringView CG_LocationName( unsigned location ) {
	return ::cl.configStrings.getLocation( location ).value();
}
