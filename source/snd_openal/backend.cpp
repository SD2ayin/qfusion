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

#include "snd_local.h"
#include "snd_env_sampler.h"
#include "alsystemfacade.h"

typedef struct qbufPipe_s sndCmdPipe_t;

static ALCdevice *alDevice;
static ALCcontext *alContext;

#define UPDATE_MSEC 10
static int64_t s_last_update_time;

static bool S_Init( void *hwnd, int maxEntities, bool verbose ) {
	int numDevices;
	int userDeviceNum = -1;
	char *devices, *defaultDevice;
	cvar_t *s_openAL_device;
	int attrList[6];
	int *attrPtr;

	alDevice = NULL;
	alContext = NULL;

	s_last_update_time = 0;

	// get system default device identifier
	defaultDevice = ( char * )alcGetString( NULL, ALC_DEFAULT_DEVICE_SPECIFIER );
	if( !defaultDevice ) {
		Com_Printf( "Failed to get openAL default device\n" );
		return false;
	}

	s_openAL_device = Cvar_Get( "s_openAL_device", defaultDevice, CVAR_ARCHIVE | CVAR_LATCH_SOUND );

	devices = ( char * )alcGetString( NULL, ALC_DEVICE_SPECIFIER );
	for( numDevices = 0; *devices; devices += strlen( devices ) + 1, numDevices++ ) {
		if( !Q_stricmp( s_openAL_device->string, devices ) ) {
			userDeviceNum = numDevices;

			// force case sensitive
			if( strcmp( s_openAL_device->string, devices ) ) {
				Cvar_ForceSet( "s_openAL_device", devices );
			}
		}
	}

	if( !numDevices ) {
		Com_Printf( "Failed to get openAL devices\n" );
		return false;
	}

	// the device assigned by the user is not available
	if( userDeviceNum == -1 ) {
		Com_Printf( "'s_openAL_device': incorrect device name, reseting to default\n" );

		Cvar_ForceSet( "s_openAL_device", defaultDevice );

		devices = ( char * )alcGetString( NULL, ALC_DEVICE_SPECIFIER );
		for( numDevices = 0; *devices; devices += strlen( devices ) + 1, numDevices++ ) {
			if( !Q_stricmp( s_openAL_device->string, devices ) ) {
				userDeviceNum = numDevices;
			}
		}

		if( userDeviceNum == -1 ) {
			Cvar_ForceSet( "s_openAL_device", defaultDevice );
		}
	}

	alDevice = alcOpenDevice( (const ALchar *)s_openAL_device->string );
	if( !alDevice ) {
		Com_Printf( "Failed to open device\n" );
		return false;
	}

	attrPtr = &attrList[0];
	if( s_environment_effects->integer ) {
		// We limit each source to a single "auxiliary send" for optimization purposes.
		// This means each source has a single auxiliary output that feeds an effect aside from a direct output.
		*attrPtr++ = ALC_MAX_AUXILIARY_SENDS;
		*attrPtr++ = 1;
	}

	*attrPtr++ = ALC_HRTF_SOFT;
	*attrPtr++ = s_hrtf->integer ? 1 : 0;

	// Terminate the attributes pairs list
	*attrPtr++ = 0;
	*attrPtr++ = 0;

	// Create context
	alContext = alcCreateContext( alDevice, attrList );
	if( !alContext ) {
		Com_Printf( "Failed to create context\n" );
		return false;
	}

	alcMakeContextCurrent( alContext );

	if( verbose ) {
		sNotice() << "OpenAL initialized";

		if( numDevices ) {
			int i;

			Com_Printf( "  Devices:    " );

			devices = ( char * )alcGetString( NULL, ALC_DEVICE_SPECIFIER );
			for( i = 0; *devices; devices += strlen( devices ) + 1, i++ )
				Com_Printf( "%s%s", devices, ( i < numDevices - 1 ) ? ", " : "" );
			Com_Printf( "\n" );

			if( defaultDevice && *defaultDevice ) {
				Com_Printf( "  Default system device: %s\n", defaultDevice );
			}

			Com_Printf( "\n" );
		}

		sNotice() << "  Device:     " << wsw::StringView( alcGetString( alDevice, ALC_DEVICE_SPECIFIER ) );
		sNotice() << "  Vendor:     " << wsw::StringView( alGetString( AL_VENDOR ) );
		sNotice() << "  Version:    " << wsw::StringView( alGetString( AL_VERSION ) );
		sNotice() << "  Renderer:   " << wsw::StringView( alGetString( AL_RENDERER ) );
		sNotice() << "  Extensions: " << wsw::StringView( alGetString( AL_EXTENSIONS ) );
	}

	alDopplerFactor( s_doppler->value );
	// Defer s_sound_velocity application to S_Update() in order to avoid code duplication
	s_sound_velocity->modified = true;
	s_doppler->modified = false;

	S_LockBackgroundTrack( false );

	if( !S_InitDecoders( verbose ) ) {
		Com_Printf( "Failed to init decoders\n" );
		return false;
	}
	if( !S_InitSources( maxEntities, verbose ) ) {
		Com_Printf( "Failed to init sources\n" );
		return false;
	}

	// If we have reached this condition, EFX support is guaranteed.
	if( s_environment_effects->integer ) {
		// Set the value once
		alListenerf( AL_METERS_PER_UNIT, QF_METERS_PER_UNIT );
		if( alGetError() != AL_NO_ERROR ) {
			return false;
		}
	}

	return true;
}

static void S_Shutdown( bool verbose ) {
	S_StopStreams();
	S_LockBackgroundTrack( false );
	S_StopBackgroundTrack();

	S_ShutdownSources();
	S_ShutdownDecoders( verbose );

	if( alContext ) {
		alcMakeContextCurrent( NULL );
		alcDestroyContext( alContext );
		alContext = NULL;
	}

	if( alDevice ) {
		alcCloseDevice( alDevice );
		alDevice = NULL;
	}
}

static void S_SetListener( const vec3_t origin, const vec3_t velocity, const mat3_t axis ) {
	float orientation[6];

	orientation[0] = axis[AXIS_FORWARD + 0];
	orientation[1] = axis[AXIS_FORWARD + 1];
	orientation[2] = axis[AXIS_FORWARD + 2];
	orientation[3] = axis[AXIS_UP + 0];
	orientation[4] = axis[AXIS_UP + 1];
	orientation[5] = axis[AXIS_UP + 2];

	alListenerfv( AL_POSITION, origin );
	alListenerfv( AL_VELOCITY, velocity );
	alListenerfv( AL_ORIENTATION, orientation );

	ENV_UpdateListener( origin, velocity, axis );
}

namespace wsw::snd {

void Backend::init( bool verbose ) {
	if( S_Init( nullptr, MAX_EDICTS, verbose ) ) {
		m_initialized = true;
	}
}

void Backend::shutdown( bool verbose ) {
	S_Shutdown( verbose );

	// Note: this is followed by a separate "terminate pipe" call
}

void Backend::clear() {
	S_Clear();
}

void Backend::stopAllSounds( unsigned flags ) {
	S_StopStreams();
	S_StopAllSources();
	if( flags & SoundSystem::StopMusic ) {
		S_StopBackgroundTrack();
	}
	if( flags & SoundSystem::StopAndClear ) {
		S_Clear();
	}
}

void Backend::processFrameUpdates() {
	S_UpdateSources();
}

void Backend::freeSound( int id ) {
	S_UnloadBuffer( S_GetBufferById( id ) );
}

void Backend::loadSound( int id ) {
	S_LoadBuffer( S_GetBufferById( id ) );
}

void Backend::setEntitySpatialParams( const EntitySpatialParamsBatch &batch ) {
	for( unsigned i = 0; i < batch.count; ++i ) {
		S_SetEntitySpatialization( batch.entNums[i], batch.origins[i], batch.velocities[i] );
	}
}

void Backend::setListener( const Vec3 &origin, const Vec3 &velocity, const std::array<Vec3, 3> &axis ) {
	S_SetListener( origin.Data(), velocity.Data(), (const float *)axis.data() );
}

void Backend::startLocalSound( int id, float volume ) {
	S_StartLocalSound( S_GetBufferById( id ), volume );
}

void Backend::startFixedSound( int id, const Vec3 &origin, int channel, float volume, float attenuation ) {
	S_StartFixedSound( S_GetBufferById( id ), origin.Data(), channel, volume, attenuation );
}

void Backend::startGlobalSound( int id, int channel, float volume ) {
	S_StartGlobalSound( S_GetBufferById( id ), channel, volume );
}

void Backend::startRelativeSound( int id, int entNum, int channel, float volume, float attenuation ) {
	S_StartRelativeSound( S_GetBufferById( id ), entNum, channel, volume, attenuation );
}

void Backend::addLoopSound( int id, int entNum, uintptr_t identifyingToken, float volume, float attenuation ) {
	S_AddLoopSound( S_GetBufferById( id ), entNum, identifyingToken, volume, attenuation );
}

void Backend::startBackgroundTrack( const wsw::String &intro, const wsw::String &loop, int mode ) {
	S_StartBackgroundTrack( intro.c_str(), loop.c_str(), mode );
}

void Backend::stopBackgroundTrack() {
	S_StopBackgroundTrack();
}

void Backend::lockBackgroundTrack( bool lock ) {
	S_LockBackgroundTrack( lock );
}

void Backend::advanceBackgroundTrack( int value ) {
	if( value < 0 ) {
		S_PrevBackgroundTrack();
	} else if( value > 0 ) {
		S_NextBackgroundTrack();
	} else {
		S_PauseBackgroundTrack();
	}
}

void Backend::activate( bool active ) {
	S_Clear();

	S_LockBackgroundTrack( !active );

	// TODO: Actually stop playing sounds while not active?
	if( active ) {
		alListenerf( AL_GAIN, 1 );
	} else {
		alListenerf( AL_GAIN, 0 );
	}
}

}

static void S_Update( void ) {
	S_UpdateMusic();

	S_UpdateStreams();

	if( s_doppler->modified ) {
		if( s_doppler->value > 0.0f ) {
			alDopplerFactor( s_doppler->value );
		} else {
			alDopplerFactor( 0.0f );
		}
		s_doppler->modified = false;
	}

	if( s_sound_velocity->modified ) {
		// If environment effects are supported, we can set units to meters ratio.
		// In this case, we have to scale the hardcoded s_sound_velocity value.
		// (The engine used to assume that units to meters ratio is 1).
		float appliedVelocity = s_sound_velocity->value;
		if( appliedVelocity <= 0.0f ) {
			appliedVelocity = 0.0f;
		} else if( s_environment_effects->integer ) {
			appliedVelocity *= QF_METERS_PER_UNIT;
		}
		alDopplerVelocity( appliedVelocity );
		alSpeedOfSound( appliedVelocity );
		s_sound_velocity->modified = false;
	}
}

static int S_EnqueuedCmdsWaiter( sndCmdPipe_t *queue, bool timeout ) {
	const int read = QBufPipe_ReadCmds( queue );
	if( read < 0 ) {
		// shutdown
		return read;
	}

	const int64_t now = Sys_Milliseconds();
	if( timeout || now >= s_last_update_time + UPDATE_MSEC ) {
		s_last_update_time = now;
		S_Update();
	}

	return read;
}

void *S_BackgroundUpdateProc( void *param ) {
	using namespace wsw::snd;

	auto *arg        = ( ALSoundSystem::ThreadProcArg *)param;
	qbufPipe_s *pipe = arg->pipe;

	// Don't hold the arg heap memory forever
	Q_free( arg );

	QBufPipe_Wait( pipe, S_EnqueuedCmdsWaiter, UPDATE_MSEC );

	return NULL;
}
