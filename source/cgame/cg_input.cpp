/*
Copyright (C) 2015 SiPlus, Chasseur de bots

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

/**
 * Warsow-specific input code.
 */

#include "cg_local.h"
#include "../client/input.h"
#include "../client/keys.h"
#include "../client/client.h"
#include "../common/cmdargs.h"
#include "../common/configvars.h"
#include "../common/wswtonum.h"

using wsw::operator""_asView;

static FloatConfigVar v_yawSpeed( "cl_yawSpeed"_asView, { .byDefault = 140.0f } );
static FloatConfigVar v_pitchSpeed( "cl_pitchSpeed"_asView, { .byDefault = 150.0f } );
static FloatConfigVar v_angleSpeedKey( "cl_angleSpeedKey"_asView, { .byDefault = 1.5f } );

static BoolConfigVar v_run( "cl_run"_asView, { .byDefault = true, .flags = CVAR_ARCHIVE } );

static FloatConfigVar v_sensitivity( "sensitivity"_asView, { .byDefault = 3.0f, .flags = CVAR_ARCHIVE } );
static FloatConfigVar v_zoomsens( "zoomsens"_asView, { .byDefault = 0.0f, .flags = CVAR_ARCHIVE } );
static FloatConfigVar v_accel( "m_accel"_asView, { .byDefault = 0.0f, .flags = CVAR_ARCHIVE } );
static IntConfigVar v_accelStyle( "m_accelStyle"_asView, { .byDefault = 0, .flags = CVAR_ARCHIVE } );
static FloatConfigVar v_accelOffset( "m_accelOffset"_asView, { .byDefault = 0.0f, .flags = CVAR_ARCHIVE } );
static FloatConfigVar v_accelPow( "m_accelPow"_asView, { .byDefault = 2.0f, .flags = CVAR_ARCHIVE } );
static BoolConfigVar v_filter( "m_filter"_asView, { .byDefault = false, .flags = CVAR_ARCHIVE } );
static FloatConfigVar v_pitch( "m_pitch"_asView, { .byDefault = 0.022f, .flags = CVAR_ARCHIVE } );
static FloatConfigVar v_yaw( "m_yaw"_asView, { .byDefault = 0.022f, .flags = CVAR_ARCHIVE } );
static FloatConfigVar v_sensCap( "m_sensCap"_asView, { .byDefault = 0.0f, .flags = CVAR_ARCHIVE } );

static int64_t cg_inputTimestamp;
static int cg_inputKeyboardDelta;
static float cg_inputMouseDelta;
static bool cg_inputCenterView;

static float mouse_x = 0, mouse_y = 0;
static float old_mouse_x = 0, old_mouse_y = 0;

/*
===============================================================================

KEY BUTTONS

Continuous button event tracking is complicated by the fact that two different
input sources (say, mouse button 1 and the control key) can both press the
same button, but the button should only be released when both of the
pressing key have been released.

When a key event issues a button command (+forward, +attack, etc), it appends
its key number as a parameter to the command so it can be matched up with
the release.

state bit 0 is the current state of the key
state bit 1 is edge triggered on the up to down transition
state bit 2 is edge triggered on the down to up transition


Key_Event (int key, bool down, int64_t time);

===============================================================================
*/

struct CommandKeyState {
	CommandKeyState *m_next { nullptr };
	const char *m_name { nullptr };
	int64_t m_downtime { 0 }; // msec timestamp
	int m_keysHeldDown[2] { 0, 0 };   // key nums holding it down
	unsigned m_msec { 0 };    // msec down this frame
	unsigned m_state { 0 };
	static inline CommandKeyState *s_head { nullptr };

	explicit CommandKeyState( const char *name ) noexcept : m_name( name ) {
		m_next = s_head;
		s_head = this;
	}

	virtual ~CommandKeyState() = default;

	void prepareCommandNames( wsw::StaticString<32> *upName, wsw::StaticString<32> *downName ) const {
		upName->clear(), downName->clear();
		*upName << '-' << wsw::StringView( m_name );
		*downName << '+' << wsw::StringView( m_name );
	}

	virtual void registerCommandCallbacks( wsw::StringView upName, wsw::StringView downName ) = 0;

	void registerCommands() {
		wsw::StaticString<32> upName, downName;
		prepareCommandNames( &upName, &downName );
		registerCommandCallbacks( upName.asView(), downName.asView() );
	}

	void unregisterCommands() {
		wsw::StaticString<32> upName, downName;
		prepareCommandNames( &upName, &downName );
		CL_Cmd_Unregister( upName.asView() );
		CL_Cmd_Unregister( downName.asView() );
	}

	void handleUpCmd( const CmdArgs & );
	void handleDownCmd( const CmdArgs & );
};

// This is a hack to generate different function pointers for registerCommandCallbacks() method.
// Command handling does not currenly accept anything besides free function pointers,
// and using std::function<> should be discouraged.
// TODO: Allow passing user pointers during command registration.

template <unsigned>
struct CommandKeyState_ : public CommandKeyState {
	explicit CommandKeyState_( const char *name_ ) noexcept : CommandKeyState( name_ ) {}
	void registerCommandCallbacks( wsw::StringView upName, wsw::StringView downName ) override {
		static CommandKeyState *myInstance = this;
		static CmdFunc upFn = []( const CmdArgs &cmdArgs ) { myInstance->handleUpCmd( cmdArgs ); };
		static CmdFunc downFn = []( const CmdArgs &cmdArgs ) { myInstance->handleDownCmd( cmdArgs ); };
		CL_Cmd_Register( upName, upFn );
		CL_Cmd_Register( downName, downFn );
	}
};

static CommandKeyState_<0> in_klook( "klook" );
static CommandKeyState_<1> in_left( "left" );
static CommandKeyState_<2> in_right( "right" );
static CommandKeyState_<3> in_forward( "forward" );
static CommandKeyState_<4> in_back( "back" );
static CommandKeyState_<5> in_lookup( "lookup" );
static CommandKeyState_<6> in_lookdown( "lookdown" );
static CommandKeyState_<7> in_moveleft( "moveleft" );
static CommandKeyState_<8> in_moveright( "moveright" );
static CommandKeyState_<9> in_strafe( "strafe" );
static CommandKeyState_<10> in_speed( "speed" );
static CommandKeyState_<11> in_use( "use" );
static CommandKeyState_<12> in_attack( "attack" );
static CommandKeyState_<13> in_up( "moveup" );
static CommandKeyState_<14> in_down( "movedown" );
static CommandKeyState_<15> in_special( "special" );
static CommandKeyState_<16> in_zoom( "zoom" );

void CommandKeyState::handleDownCmd( const CmdArgs &cmdArgs ) {
	// On parsing failure, assuming it being typed manually at the console for continuous down
	const int key = wsw::toNum<int>( cmdArgs[1] ).value_or( -1 );
	if( key != m_keysHeldDown[0] && key != m_keysHeldDown[1] ) {
		// If there are free slots for this "down" key
		if( auto end = m_keysHeldDown + 2, it = std::find( m_keysHeldDown, end, 0 ); it != end ) {
			*it = key;
			// If was not down
			if( !( m_state & 1 ) ) {
				// Save the timestamp
				if( const std::optional<int64_t> downtime = wsw::toNum<int64_t>( cmdArgs[2] ) ) {
					m_downtime = *downtime;
				} else {
					m_downtime = cg_inputTimestamp - 100;
				}
				m_state |= 1 + 2; // down + impulse down
			}
		} else {
			cgWarning() << "Three keys down for a button!";
		}
	}
}

void CommandKeyState::handleUpCmd( const CmdArgs &cmdArgs ) {
	if( const std::optional<int> key = wsw::toNum<int>( cmdArgs[1] ) ) {
		// Find slot of this key
		if( auto end = m_keysHeldDown + 2, it = std::find( m_keysHeldDown, end, *key ); it != end ) {
			*it = 0;
			// If it cannot longer be considered down
			if( !m_keysHeldDown[0] && !m_keysHeldDown[1] ) {
				// Must be down (should always pass?)
				if( m_state & 1 ) {
					// save timestamp
					if( const std::optional<int64_t> uptime = wsw::toNum<int>( cmdArgs[2] ) ) {
						m_msec += *uptime - m_downtime;
					} else {
						m_msec += 10;
					}
					m_state &= ~1; // now up
					m_state |= 4;  // impulse up
				}
			}
		} else {
			; // key up without corresponding down (menu pass through) TODO?
		}
	} else {
		// typed manually at the console, assume for unsticking, so clear all
		m_keysHeldDown[0] = m_keysHeldDown[1] = 0;
		m_state = 4;
	}
}

static float CG_KeyState( CommandKeyState *key ) {
	key->m_state &= 1; // clear impulses

	int msec = (int)key->m_msec;
	key->m_msec = 0;

	if( key->m_state ) {
		// still down
		msec += (int)( cg_inputTimestamp - key->m_downtime );
		key->m_downtime = cg_inputTimestamp;
	}

	if( cg_inputKeyboardDelta > 0 ) {
		return wsw::clamp( (float) msec / (float)cg_inputKeyboardDelta, 0.0f, 1.0f );
	}

	return 0;
}

static void CG_AddKeysViewAngles( vec3_t viewAngles ) {
	float speed;
	if( in_speed.m_state & 1 ) {
		speed = ( (float)cg_inputKeyboardDelta * 0.001f ) * v_angleSpeedKey.get();
	} else {
		speed = (float)cg_inputKeyboardDelta * 0.001f;
	}

	if( !( in_strafe.m_state & 1 ) ) {
		const float yawSpeed = speed * v_yawSpeed.get();
		viewAngles[YAW] -= yawSpeed * CG_KeyState( &in_right );
		viewAngles[YAW] += yawSpeed * CG_KeyState( &in_left );
	}

	const float pitchSpeed = speed * v_pitchSpeed.get();
	if( in_klook.m_state & 1 ) {
		viewAngles[PITCH] -= pitchSpeed * CG_KeyState( &in_forward );
		viewAngles[PITCH] += pitchSpeed * CG_KeyState( &in_back );
	}

	viewAngles[PITCH] -= pitchSpeed * CG_KeyState( &in_lookup );
	viewAngles[PITCH] += pitchSpeed * CG_KeyState( &in_lookdown );
}

static void CG_AddKeysMovement( vec3_t movement ) {
	if( in_strafe.m_state & 1 ) {
		movement[0] += CG_KeyState( &in_right );
		movement[0] -= CG_KeyState( &in_left );
	}

	movement[0] += CG_KeyState( &in_moveright );
	movement[0] -= CG_KeyState( &in_moveleft );

	if( !( in_klook.m_state & 1 ) ) {
		movement[1] += CG_KeyState( &in_forward );
		movement[1] -= CG_KeyState( &in_back );
	}

	movement[2] += CG_KeyState( &in_up );
	if( const float down = CG_KeyState( &in_down ); down > movement[2] ) {
		movement[2] -= down;
	}
}

unsigned CG_GetButtonBits() {
	int buttons = 0;

	// figure button bits

	if( in_attack.m_state & 3 ) {
		buttons |= BUTTON_ATTACK;
	}
	in_attack.m_state &= ~2;

	if( in_special.m_state & 3 ) {
		buttons |= BUTTON_SPECIAL;
	}
	in_special.m_state &= ~2;

	if( in_use.m_state & 3 ) {
		buttons |= BUTTON_USE;
	}
	in_use.m_state &= ~2;

	if( ( in_speed.m_state & 1 ) ^ !v_run.get() ) {
		buttons |= BUTTON_WALK;
	}

	if( in_zoom.m_state & 3 ) {
		buttons |= BUTTON_ZOOM;
	}
	in_zoom.m_state &= ~2;

	return buttons;
}

void CG_MouseMove( int mx, int my ) {
	if( v_filter.get() ) {
		mouse_x = 0.5f * ( (float)mx + old_mouse_x );
		mouse_y = 0.5f * ( (float)my + old_mouse_y );
	} else {
		mouse_x = (float)mx;
		mouse_y = (float)my;
	}

	old_mouse_x = (float)mx;
	old_mouse_y = (float)my;

	float resultingSensitivity = v_sensitivity.get();

	if( v_accel.get() != 0.0f && cg_inputMouseDelta != 0.0f ) {
		// QuakeLive-style mouse acceleration, ported from ioquake3
		// original patch by Gabriel Schnoering and TTimo
		if( v_accelStyle.get() == 1 ) {
			float base[2];
			float power[2];

			// sensitivity remains pretty much unchanged at low speeds
			// m_accel is a power value to how the acceleration is shaped
			// m_accelOffset is the rate for which the acceleration will have doubled the non accelerated amplification
			// NOTE: decouple the config cvars for independent acceleration setup along X and Y?

			base[0] = (float) ( abs( mx ) ) / (float) cg_inputMouseDelta;
			base[1] = (float) ( abs( my ) ) / (float) cg_inputMouseDelta;
			power[0] = powf( base[0] / v_accelOffset.get(), v_accel.get() );
			power[1] = powf( base[1] / v_accelOffset.get(), v_accel.get() );

			mouse_x = ( mouse_x + ( ( mouse_x < 0 ) ? -power[0] : power[0] ) * v_accelOffset.get() );
			mouse_y = ( mouse_y + ( ( mouse_y < 0 ) ? -power[1] : power[1] ) * v_accelOffset.get() );
		} else if( v_accelStyle.get() == 2 ) {
			// ch : similar to normal acceleration with offset and variable pow mechanisms

			// sanitize values
			const float accelPow    = v_accelPow.get() > 1.0f ? v_accelPow.get() : 2.0f;
			const float accelOffset = v_accelOffset.get() >= 0.0f ? v_accelOffset.get() : 0.0f;

			float rate = sqrt( mouse_x * mouse_x + mouse_y * mouse_y ) / (float)cg_inputMouseDelta;
			rate -= accelOffset;
			if( rate < 0 ) {
				rate = 0.0;
			}

			// ch : TODO sens += pow( rate * m_accel->value, m_accelPow->value - 1.0 )
			resultingSensitivity += std::pow( rate * v_accel.get(), accelPow - 1.0f );

			// TODO : move this outside of this branch?
			if( v_sensCap.get() > 0 && resultingSensitivity > v_sensCap.get() ) {
				resultingSensitivity = v_sensCap.get();
			}
		} else {
			const float rate = sqrt( mouse_x * mouse_x + mouse_y * mouse_y ) / (float)cg_inputMouseDelta;
			resultingSensitivity += rate * v_accel.get();
		}
	}

	resultingSensitivity *= CG_GetSensitivityScale( v_sensitivity.get(), v_zoomsens.get() );

	mouse_x *= resultingSensitivity;
	mouse_y *= resultingSensitivity;
}

static void CG_AddMouseViewAngles( vec3_t viewAngles ) {
	// add mouse X/Y movement to cmd
	if( mouse_x != 0.0f ) {
		viewAngles[YAW] -= v_yaw.get() * mouse_x;
	}
	if( mouse_y != 0.0f ) {
		viewAngles[PITCH] += v_pitch.get() * mouse_y;
	}
}

static void CG_CenterView( const CmdArgs & ) {
	cg_inputCenterView = true;
}

void CG_InitInput() {
	for( CommandKeyState *state = CommandKeyState::s_head; state; state = state->m_next ) {
		state->registerCommands();
	}

	CL_Cmd_Register( "centerview"_asView, CG_CenterView );
}

void CG_ShutdownInput() {
	for( CommandKeyState *state = CommandKeyState::s_head; state; state = state->m_next ) {
		state->unregisterCommands();
	}

	CL_Cmd_Unregister( "centerview"_asView );
}

void CG_AddViewAngles( vec3_t viewAngles ) {
	vec3_t deltaAngles { 0.0f, 0.0f, 0.0f };

	CG_AddKeysViewAngles( deltaAngles );
	CG_AddMouseViewAngles( deltaAngles );

	VectorAdd( viewAngles, deltaAngles, viewAngles );

	if( cg_inputCenterView ) {
		viewAngles[PITCH] = (float)-SHORT2ANGLE( cg.predictedPlayerState.pmove.delta_angles[PITCH] );
		cg_inputCenterView = false;
	}
}

void CG_AddMovement( vec3_t movement ) {
	vec3_t deltaMovement { 0.0f, 0.0f, 0.0f };

	CG_AddKeysMovement( deltaMovement );

	VectorAdd( movement, deltaMovement, movement );
}

void CG_InputFrame( int64_t inputTimestamp, int keyboardDeltaMillis, float mouseDeltaMillis ) {
	cg_inputTimestamp     = inputTimestamp;
	cg_inputKeyboardDelta = keyboardDeltaMillis;
	cg_inputMouseDelta    = mouseDeltaMillis;
}

void CG_ClearInputState() {
	cg_inputKeyboardDelta = 0;
	cg_inputMouseDelta    = 0.0f;
}

void CG_GetBoundKeysString( const char *cmd, char *keys, size_t keysSize ) {
	const wsw::StringView cmdView( cmd );
	wsw::StaticString<32> keyNames[2];

	int numKeys = 0;
	const auto *const bindingsSystem = wsw::cl::KeyBindingsSystem::instance();
	// TODO: If the routine turns to be really useful,
	// implement such functionality at the bindings system level
	// in an optimized fashion instead of doing a loop over all keys here.
	for( int key = 0; key < 256; key++ ) {
		if( const std::optional<wsw::StringView> maybeBinding = bindingsSystem->getBindingForKey( key ) ) {
			if( maybeBinding->equalsIgnoreCase( cmdView ) ) {
				if( const std::optional<wsw::StringView> maybeName = bindingsSystem->getNameForKey( key ) ) {
					keyNames[numKeys].assign( maybeName->take( keysSize ) );
					numKeys++;
					if( numKeys == 2 ) {
						break;
					}
				}
			}
		}
	}

	if( numKeys == 2 ) {
		Q_snprintfz( keys, keysSize, "%s or %s", keyNames[0].data(), keyNames[1].data() );
	} else if( numKeys == 1 ) {
		Q_strncpyz( keys, keyNames[0].data(), keysSize );
	} else {
		Q_strncpyz( keys, "UNBOUND", keysSize );
	}
}
