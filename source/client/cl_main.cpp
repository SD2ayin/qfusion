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
// cl_main.c  -- client main loop

#include "client.h"
#include "cl_mm.h"

#include "../qcommon/asyncstream.h"
#include "../qcommon/cmdsystem.h"
#include "../qcommon/singletonholder.h"
#include "../qcommon/hash.h"
#include "../ui/uisystem.h"

#include "serverlist.h"

#include <random>

using wsw::operator""_asView;

cvar_t *cl_stereo_separation;
cvar_t *cl_stereo;

cvar_t *rcon_client_password;
cvar_t *rcon_address;

cvar_t *cl_timeout;
cvar_t *cl_maxfps;
cvar_t *cl_sleep;
cvar_t *cl_pps;
cvar_t *cl_shownet;

cvar_t *cl_extrapolationTime;
cvar_t *cl_extrapolate;

cvar_t *cl_timedemo;

cvar_t *sensitivity;
cvar_t *zoomsens;
cvar_t *m_accel;
cvar_t *m_accelStyle;
cvar_t *m_accelOffset;
cvar_t *m_accelPow;
cvar_t *m_filter;
cvar_t *m_filterStrength;
cvar_t *m_sensCap;

cvar_t *m_pitch;
cvar_t *m_yaw;

//
// userinfo
//
cvar_t *info_password;
cvar_t *rate;

cvar_t *cl_infoservers;

// wsw : debug netcode
cvar_t *cl_debug_serverCmd;
cvar_t *cl_debug_timeDelta;

cvar_t *cl_downloads;
cvar_t *cl_downloads_from_web;
cvar_t *cl_downloads_from_web_timeout;
cvar_t *cl_download_allow_modules;

static char cl_nextString[MAX_STRING_CHARS];
static char cl_connectChain[MAX_STRING_CHARS];

client_static_t cls;
client_state_t cl;

entity_state_t cl_baselines[MAX_EDICTS];

static bool cl_initialized = false;

static async_stream_module_t *cl_async_stream;

//======================================================================


/*
=======================================================================

CLIENT RELIABLE COMMAND COMMUNICATION

=======================================================================
*/

/*
* CL_AddReliableCommand
*
* The given command will be transmitted to the server, and is gauranteed to
* not have future usercmd_t executed before it is executed
*/
void CL_AddReliableCommand( const char *cmd ) {
	if( !cmd || !strlen( cmd ) ) {
		return;
	}

	// if we would be losing an old command that hasn't been acknowledged,
	// we must drop the connection
	if( cls.reliableSequence > cls.reliableAcknowledge + MAX_RELIABLE_COMMANDS ) {
		cls.reliableAcknowledge = cls.reliableSequence; // try to avoid loops
		Com_Error( ERR_DROP, "Client command overflow %" PRIi64 "%" PRIi64, cls.reliableAcknowledge, cls.reliableSequence );
	}

	cls.reliableSequence++;
	const auto index = (int)( cls.reliableSequence & ( MAX_RELIABLE_COMMANDS - 1 ) );
	Q_strncpyz( cls.reliableCommands[index], cmd, sizeof( cls.reliableCommands[index] ) );
}

/*
* CL_UpdateClientCommandsToServer
*
* Add the pending commands to the message
*/
void CL_UpdateClientCommandsToServer( msg_t *msg ) {
	int64_t i;

	// write any unacknowledged clientCommands
	for( i = cls.reliableAcknowledge + 1; i <= cls.reliableSequence; i++ ) {
		if( !strlen( cls.reliableCommands[i & ( MAX_RELIABLE_COMMANDS - 1 )] ) ) {
			continue;
		}

		MSG_WriteUint8( msg, clc_clientcommand );
		if( !cls.reliable ) {
			MSG_WriteIntBase128( msg, i );
		}
		MSG_WriteString( msg, cls.reliableCommands[i & ( MAX_RELIABLE_COMMANDS - 1 )] );
	}

	cls.reliableSent = cls.reliableSequence;
	if( cls.reliable ) {
		cls.reliableAcknowledge = cls.reliableSent;
	}
}

/*
* CL_ForwardToServer_f
*/
void CL_ForwardToServer_f( const CmdArgs &cmdArgs ) {
	if( cls.demoPlayer.playing ) {
		return;
	}

	if( cls.state != CA_CONNECTED && cls.state != CA_ACTIVE ) {
		Com_Printf( "Can't \"%s\", not connected\n", Cmd_Argv( 0 ) );
		return;
	}

	// don't forward the first argument
	if( Cmd_Argc() > 1 ) {
		CL_AddReliableCommand( Cmd_Args() );
	}
}

/*
* CL_ServerDisconnect_f
*/
void CL_ServerDisconnect_f( const CmdArgs &cmdArgs ) {
	char menuparms[MAX_STRING_CHARS];
	int type;
	char reason[MAX_STRING_CHARS];

	type = atoi( Cmd_Argv( 1 ) );
	if( type < 0 || type >= DROP_TYPE_TOTAL ) {
		type = DROP_TYPE_GENERAL;
	}

	Q_strncpyz( reason, Cmd_Argv( 2 ), sizeof( reason ) );

	CL_Disconnect_f( {} );

	Com_Printf( "Connection was closed by server: %s\n", reason );

	Q_snprintfz( menuparms, sizeof( menuparms ), "menu_open connfailed dropreason %i servername \"%s\" droptype %i rejectmessage \"%s\"",
				 DROP_REASON_CONNTERMINATED, cls.servername, type, reason );

	CL_Cmd_ExecuteNow( menuparms );
}

/*
* CL_Quit
*/
void CL_Quit( void ) {
	CL_Disconnect( NULL );
	Com_Quit( {} );
}

/*
* CL_Quit_f
*/
static void CL_Quit_f( const CmdArgs & ) {
	CL_Quit();
}

/*
* CL_SendConnectPacket
*
* We have gotten a challenge from the server, so try and
* connect.
*/
static void CL_SendConnectPacket( void ) {
	userinfo_modified = false;

	const char *ticketString = CLStatsowFacade::Instance()->GetTicketString().data();
	Netchan_OutOfBandPrint( cls.socket, &cls.serveraddress, "connect %i %i %i \"%s\" %i %s\n",
							APP_PROTOCOL_VERSION, Netchan_GamePort(), cls.challenge, Cvar_Userinfo(), 0, ticketString );
}

/*
* CL_CheckForResend
*
* Resend a connect message if the last one has timed out
*/
static void CL_CheckForResend( void ) {
	// FIXME: should use cls.realtime, but it can be old here after starting a server
	int64_t realtime = Sys_Milliseconds();

	if( cls.demoPlayer.playing ) {
		return;
	}

	// if the local server is running and we aren't then connect
	if( cls.state == CA_DISCONNECTED && Com_ServerState() ) {
		CL_SetClientState( CA_CONNECTING );
		if( cls.servername ) {
			Q_free( cls.servername );
		}
		cls.servername = Q_strdup( "localhost" );
		cls.servertype = SOCKET_LOOPBACK;
		NET_InitAddress( &cls.serveraddress, NA_LOOPBACK );
		if( !NET_OpenSocket( &cls.socket_loopback, cls.servertype, &cls.serveraddress, false ) ) {
			Com_Error( ERR_FATAL, "Couldn't open the loopback socket\n" );
			return;
		}
		cls.socket = &cls.socket_loopback;
	}

	// resend if we haven't gotten a reply yet
	if( cls.state == CA_CONNECTING && !cls.reliable ) {
		if( realtime - cls.connect_time < 3000 ) {
			return;
		}
		if( cls.connect_count > 3 ) {
			CL_Disconnect( "Connection timed out" );
			return;
		}
		cls.connect_count++;
		cls.connect_time = realtime; // for retransmit requests

		Com_Printf( "Connecting to %s...\n", cls.servername );

		Netchan_OutOfBandPrint( cls.socket, &cls.serveraddress, "getchallenge\n" );
	}

	if( cls.state == CA_CONNECTING && cls.reliable ) {
		if( realtime - cls.connect_time < 3000 ) {
			return;
		}

#ifdef TCP_ALLOW_CONNECT
		if( cls.socket->type == SOCKET_TCP && !cls.socket->connected ) {
			connection_status_t status;

			if( !cls.connect_count ) {
				Com_Printf( "Connecting to %s...\n", cls.servername );

				status = NET_Connect( cls.socket, &cls.serveraddress );
			} else {
				Com_Printf( "Checking connection to %s...\n", cls.servername );

				status = NET_CheckConnect( cls.socket );
			}

			cls.connect_count++;
			cls.connect_time = realtime;

			if( status == CONNECTION_FAILED ) {
				CL_Disconnect( va( "TCP connection failed: %s", NET_ErrorString() ) );
				return;
			}

			if( status == CONNECTION_INPROGRESS ) {
				return;
			}

			Com_Printf( "Connection made, asking for challenge %s...\n", cls.servername );
			Netchan_OutOfBandPrint( cls.socket, &cls.serveraddress, "getchallenge\n" );
			return;
		}
#endif

		if( realtime - cls.connect_time < 10000 ) {
			return;
		}

		CL_Disconnect( "Connection timed out" );
	}
}

/*
* CL_Connect
*/
static void CL_Connect( const char *servername, socket_type_t type, netadr_t *address, const char *serverchain ) {
	netadr_t socketaddress;
	connstate_t newstate;

	cl_connectChain[0] = '\0';
	cl_nextString[0] = '\0';

	CL_Disconnect( NULL );

	switch( type ) {
		case SOCKET_LOOPBACK:
			NET_InitAddress( &socketaddress, NA_LOOPBACK );
			if( !NET_OpenSocket( &cls.socket_loopback, SOCKET_LOOPBACK, &socketaddress, false ) ) {
				Com_Error( ERR_FATAL, "Couldn't open the loopback socket: %s\n", NET_ErrorString() ); // FIXME
				return;
			}
			cls.socket = &cls.socket_loopback;
			cls.reliable = false;
			break;

		case SOCKET_UDP:
			cls.socket = ( address->type == NA_IP6 ?  &cls.socket_udp6 :  &cls.socket_udp );
			cls.reliable = false;
			break;

#ifdef TCP_ALLOW_CONNECT
		case SOCKET_TCP:
			NET_InitAddress( &socketaddress, address->type );
			if( !NET_OpenSocket( &cls.socket_tcp, SOCKET_TCP, &socketaddress, false ) ) {
				Com_Error( ERR_FATAL, "Couldn't open the TCP socket\n" ); // FIXME
				return;
			}
			NET_SetSocketNoDelay( &cls.socket_tcp, 1 );
			cls.socket = &cls.socket_tcp;
			cls.reliable = true;
			break;
#endif

		default:
			assert( false );
			return;
	}

	cls.servertype = type;
	cls.serveraddress = *address;
	if( NET_GetAddressPort( &cls.serveraddress ) == 0 ) {
		NET_SetAddressPort( &cls.serveraddress, PORT_SERVER );
	}

	if( cls.servername ) {
		Q_free( cls.servername );
	}
	cls.servername = Q_strdup( servername );

	cl.configStrings.clear();

	// If the server supports matchmaking and that we are authenticated, try getting a matchmaking ticket before joining the server
	newstate = CA_CONNECTING;
	if( CLStatsowFacade::Instance()->IsValid() ) {
		// if( MM_GetStatus() == MM_STATUS_AUTHENTICATED && CL_MM_GetTicket( serversession ) )
		if( CLStatsowFacade::Instance()->StartConnecting( &cls.serveraddress ) ) {
			newstate = CA_GETTING_TICKET;
		}
	}
	CL_SetClientState( newstate );

	if( serverchain[0] ) {
		Q_strncpyz( cl_connectChain, serverchain, sizeof( cl_connectChain ) );
	}

	cls.connect_time = -99999; // CL_CheckForResend() will fire immediately
	cls.connect_count = 0;
	cls.rejected = false;
	cls.lastPacketReceivedTime = cls.realtime; // reset the timeout limit
	cls.mv = false;
}

/*
* CL_Connect_Cmd_f
*/
static void CL_Connect_Cmd_f( socket_type_t socket, const CmdArgs &cmdArgs ) {
	netadr_t serveraddress;
	char *servername, password[64], autowatch[64] = { 0 };
	const char *extension;
	char *connectstring, *connectstring_base;
	const char *tmp, *scheme = APP_URI_SCHEME, *proto_scheme = APP_URI_PROTO_SCHEME;
	const char *serverchain;

	if( Cmd_Argc() < 2 ) {
		Com_Printf( "Usage: %s <server>\n", Cmd_Argv( 0 ) );
		return;
	}

	connectstring_base = Q_strdup( Cmd_Argv( 1 ) );
	connectstring = connectstring_base;
	serverchain = Cmd_Argc() >= 3 ? Cmd_Argv( 2 ) : "";

	if( !Q_strnicmp( connectstring, proto_scheme, strlen( proto_scheme ) ) ) {
		connectstring += strlen( proto_scheme );
	} else if( !Q_strnicmp( connectstring, scheme, strlen( scheme ) ) ) {
		connectstring += strlen( scheme );
	}

	extension = COM_FileExtension( connectstring );
	if( extension && !Q_stricmp( extension, APP_DEMO_EXTENSION_STR ) ) {
		char *temp;
		size_t temp_size;
		const char *http_scheme = "http://";

		if( !Q_strnicmp( connectstring, http_scheme, strlen( http_scheme ) ) ) {
			connectstring += strlen( http_scheme );
		}

		temp_size = strlen( "demo " ) + strlen( http_scheme ) + strlen( connectstring ) + 1;
		temp = (char *)Q_malloc( temp_size );
		Q_snprintfz( temp, temp_size, "demo %s%s", http_scheme, connectstring );

		CL_Cmd_ExecuteNow( temp );

		Q_free( temp );
		Q_free( connectstring_base );
		return;
	}

	if( ( tmp = Q_strrstr( connectstring, "@" ) ) != NULL ) {
		assert( tmp - connectstring >= 0 );
		Q_strncpyz( password, connectstring, wsw::min( sizeof( password ), (size_t)( tmp - connectstring + 1 ) ) );
		Cvar_Set( "password", password );
		connectstring = connectstring + ( tmp - connectstring ) + 1;
	}

	if( ( tmp = Q_strrstr( connectstring, "#" ) ) != NULL ) {
		Q_strncpyz( autowatch, COM_RemoveColorTokens( tmp + 1 ), sizeof( autowatch ) );
		connectstring[tmp - connectstring] = '\0';
	}

	if( ( tmp = Q_strrstr( connectstring, "/" ) ) != NULL ) {
		connectstring[tmp - connectstring] = '\0';
	}

	Cvar_ForceSet( "autowatch", autowatch );

	if( !NET_StringToAddress( connectstring, &serveraddress ) ) {
		Q_free( connectstring_base );
		Com_Printf( "Bad server address\n" );
		return;
	}

	// wait until MM allows us to connect to a server
	// (not in a middle of login process or anything)
	CLStatsowFacade::Instance()->WaitUntilConnectionAllowed();

	servername = Q_strdup( connectstring );
	CL_Connect( servername, ( serveraddress.type == NA_LOOPBACK ? SOCKET_LOOPBACK : socket ),
				&serveraddress, serverchain );

	Q_free( servername );
	Q_free( connectstring_base );
}

/*
* CL_Connect_f
*/
static void CL_Connect_f( const CmdArgs &cmdArgs ) {
	CL_Connect_Cmd_f( SOCKET_UDP, cmdArgs );
}

/*
* CL_TCPConnect_f
*/
#if defined( TCP_ALLOW_CONNECT )
static void CL_TCPConnect_f( void ) {
	CL_Connect_Cmd_f( SOCKET_TCP );
}
#endif


/*
* CL_Rcon_f
*
* Send the rest of the command line over as
* an unconnected command.
*/
static void CL_Rcon_f( const CmdArgs &cmdArgs ) {
	char message[1024];
	int i;
	const socket_t *socket;
	const netadr_t *address;

	if( cls.demoPlayer.playing ) {
		return;
	}

	if( rcon_client_password->string[0] == '\0' ) {
		Com_Printf( "You must set 'rcon_password' before issuing an rcon command.\n" );
		return;
	}

	// wsw : jal : check for msg len abuse (thx to r1Q2)
	if( strlen( Cmd_Args() ) + strlen( rcon_client_password->string ) + 16 >= sizeof( message ) ) {
		Com_Printf( "Length of password + command exceeds maximum allowed length.\n" );
		return;
	}

	message[0] = (uint8_t)255;
	message[1] = (uint8_t)255;
	message[2] = (uint8_t)255;
	message[3] = (uint8_t)255;
	message[4] = 0;

	Q_strncatz( message, "rcon ", sizeof( message ) );

	Q_strncatz( message, rcon_client_password->string, sizeof( message ) );
	Q_strncatz( message, " ", sizeof( message ) );

	for( i = 1; i < Cmd_Argc(); i++ ) {
		Q_strncatz( message, "\"", sizeof( message ) );
		Q_strncatz( message, Cmd_Argv( i ), sizeof( message ) );
		Q_strncatz( message, "\" ", sizeof( message ) );
	}

	if( cls.state >= CA_CONNECTED ) {
		socket = cls.netchan.socket;
		address = &cls.netchan.remoteAddress;
	} else {
		if( !strlen( rcon_address->string ) ) {
			Com_Printf( "You must be connected, or set the 'rcon_address' cvar to issue rcon commands\n" );
			return;
		}

		if( rcon_address->modified ) {
			if( !NET_StringToAddress( rcon_address->string, &cls.rconaddress ) ) {
				Com_Printf( "Bad rcon_address.\n" );
				return; // we don't clear modified, so it will whine the next time too
			}
			if( NET_GetAddressPort( &cls.rconaddress ) == 0 ) {
				NET_SetAddressPort( &cls.rconaddress, PORT_SERVER );
			}

			rcon_address->modified = false;
		}

		socket = ( cls.rconaddress.type == NA_IP6 ? &cls.socket_udp6 : &cls.socket_udp );
		address = &cls.rconaddress;
	}

	NET_SendPacket( socket, message, (int)strlen( message ) + 1, address );
}

/*
* CL_GetClipboardData
*/
char *CL_GetClipboardData( void ) {
	return Sys_GetClipboardData();
}

/*
* CL_SetClipboardData
*/
void CL_SetClipboardData( const char *data ) {
	Sys_SetClipboardData( data );
}

/*
* CL_FreeClipboardData
*/
void CL_FreeClipboardData( char *data ) {
	Sys_FreeClipboardData( data );
}

/*
* CL_IsBrowserAvailable
*/
bool CL_IsBrowserAvailable( void ) {
	return Sys_IsBrowserAvailable();
}

/*
* CL_OpenURLInBrowser
*/
void CL_OpenURLInBrowser( const char *url ) {
	Sys_OpenURLInBrowser( url );
}

/*
* CL_GetBaseServerURL
*/
size_t CL_GetBaseServerURL( char *buffer, size_t buffer_size ) {
	const char *web_url = cls.httpbaseurl;

	if( !buffer || !buffer_size ) {
		return 0;
	}
	if( !web_url || !*web_url ) {
		*buffer = '\0';
		return 0;
	}

	Q_strncpyz( buffer, web_url, buffer_size );
	return strlen( web_url );
}

/*
* CL_ResetServerCount
*/
void CL_ResetServerCount( void ) {
	cl.servercount = -1;
}

/*
* CL_BeginRegistration
*/
static void CL_BeginRegistration( void ) {
	if( cls.registrationOpen ) {
		return;
	}

	cls.registrationOpen = true;

	RF_BeginRegistration();
	wsw::ui::UISystem::instance()->beginRegistration();
	SoundSystem::instance()->beginRegistration();
}

/*
* CL_EndRegistration
*/
static void CL_EndRegistration( void ) {
	if( !cls.registrationOpen ) {
		return;
	}

	cls.registrationOpen = false;

	FTLIB_TouchAllFonts();
	RF_EndRegistration();
	wsw::ui::UISystem::instance()->endRegistration();
	SoundSystem::instance()->endRegistration();
}

/*
* CL_ClearState
*/
void CL_ClearState( void ) {
	if( cl.cms ) {
		CM_ReleaseReference( cl.cms );
		cl.cms = NULL;
	}

	if( cl.frames_areabits ) {
		Q_free( cl.frames_areabits );
		cl.frames_areabits = NULL;
	}

	if( cl.cmds ) {
		Q_free( cl.cmds );
		cl.cmds = NULL;
	}

	if( cl.cmd_time ) {
		Q_free( cl.cmd_time );
		cl.cmd_time = NULL;
	}

	if( cl.snapShots ) {
		Q_free( cl.snapShots );
		cl.snapShots = NULL;
	}

	// wipe the entire cl structure

	// Hacks just to avoid writing redundant clear() methods
	// for stuff that must be eventually rewritten.
	cl.~client_state_t();
	memset( (void *)&cl, 0, sizeof( client_state_t ) );
	new( &cl )client_state_t;

	memset( cl_baselines, 0, sizeof( cl_baselines ) );

	cl.cmds = (usercmd_t *)Q_malloc( sizeof( *cl.cmds ) * CMD_BACKUP );
	cl.cmd_time = (int *)Q_malloc( sizeof( *cl.cmd_time ) * CMD_BACKUP );
	cl.snapShots = (snapshot_t *)Q_malloc( sizeof( *cl.snapShots ) * CMD_BACKUP );

	//userinfo_modified = true;
	cls.lastExecutedServerCommand = 0;
	cls.reliableAcknowledge = 0;
	cls.reliableSequence = 0;
	cls.reliableSent = 0;
	memset( cls.reliableCommands, 0, sizeof( cls.reliableCommands ) );
	// reset ucmds buffer
	cls.ucmdHead = 0;
	cls.ucmdSent = 0;
	cls.ucmdAcknowledged = 0;

	//restart realtime and lastPacket times
	cls.realtime = 0;
	cls.gametime = 0;
	cls.lastPacketSentTime = 0;
	cls.lastPacketReceivedTime = 0;

	if( cls.wakelock ) {
		Sys_ReleaseWakeLock( cls.wakelock );
		cls.wakelock = NULL;
	}
}


/*
* CL_SetNext_f
*
* Next is used to set an action which is executed at disconnecting.
*/
static void CL_SetNext_f( const CmdArgs &cmdArgs ) {
	if( Cmd_Argc() < 2 ) {
		Com_Printf( "USAGE: next <commands>\n" );
		return;
	}

	// jalfixme: I'm afraid of this being too powerful, since it basically
	// is allowed to execute everything. Shall we check for something?
	Q_strncpyz( cl_nextString, Cmd_Args(), sizeof( cl_nextString ) );
	Com_Printf( "NEXT: %s\n", cl_nextString );
}


/*
* CL_ExecuteNext
*/
static void CL_ExecuteNext( void ) {
	if( !strlen( cl_nextString ) ) {
		return;
	}

	CL_Cbuf_AppendCommand( cl_nextString );
	memset( cl_nextString, 0, sizeof( cl_nextString ) );
}

/*
* CL_Disconnect_SendCommand
*
* Sends a disconnect message to the server
*/
static void CL_Disconnect_SendCommand( void ) {
	// wsw : jal : send the packet 3 times to make sure isn't lost
	CL_AddReliableCommand( "disconnect" );
	CL_SendMessagesToServer( true );
	CL_AddReliableCommand( "disconnect" );
	CL_SendMessagesToServer( true );
	CL_AddReliableCommand( "disconnect" );
	CL_SendMessagesToServer( true );
}

/*
* CL_Disconnect
*
* Goes from a connected state to full screen console state
* Sends a disconnect message to the server
* This is also called on Com_Error, so it shouldn't cause any errors
*/
void CL_Disconnect( const char *message ) {
	// We have to shut down webdownloading first
	if( cls.download.web && !cls.download.disconnect ) {
		cls.download.disconnect = true;
		return;
	}

	if( cls.state == CA_UNINITIALIZED ) {
		return;
	}
	if( cls.state == CA_DISCONNECTED ) {
		goto done;
	}

	SV_ShutdownGame( "Owner left the listen server", false );

	if( cl_timedemo && cl_timedemo->integer ) {
		int i;
		int64_t sumcounts = 0;

		Com_Printf( "\n" );
		for( i = 1; i < 100; i++ ) {
			if( cl.timedemo.counts[i] > 0 ) {
				float fps, perc;
				
				fps = 1000.0 / i;
				perc = cl.timedemo.counts[i] * 100.0 / cl.timedemo.frames;
				sumcounts += i * cl.timedemo.counts[i];

				Com_Printf( "%2ims - %7.2ffps: %6.2f%%\n", i, fps, perc );
			}
		}

		Com_Printf( "\n" );
		if( sumcounts ) {
			float mean = 1000.0 / (double)sumcounts * cl.timedemo.frames;
			int64_t duration = Sys_Milliseconds() - cl.timedemo.startTime;
			Com_Printf( "%3.1f seconds: %3.1f mean fps\n", duration / 1000.0, mean );
		}
	}

	cls.connect_time = 0;
	cls.connect_count = 0;
	cls.rejected = false;

	if( cls.demoRecorder.recording ) {
		CL_Stop_f( {} );
	}

	if( cls.demoPlayer.playing ) {
		CL_DemoCompleted();
	} else {
		CL_Disconnect_SendCommand(); // send a disconnect message to the server

	}
	FS_RemovePurePaks();

	Com_FreePureList( &cls.purelist );

	cls.sv_pure = false;

	// udp is kept open all the time, for connectionless messages
	if( cls.socket && cls.socket->type != SOCKET_UDP ) {
		NET_CloseSocket( cls.socket );
	}

	cls.socket = NULL;
	cls.reliable = false;
	cls.mv = false;

	if( cls.httpbaseurl ) {
		Q_free( cls.httpbaseurl );
		cls.httpbaseurl = NULL;
	}

	R_Finish();

	CL_EndRegistration();

	CL_RestartMedia();

	CL_ClearState();
	CL_SetClientState( CA_DISCONNECTED );

	if( cls.download.requestname ) {
		cls.download.pending_reconnect = false;
		cls.download.cancelled = true;
		CL_DownloadDone();
	}

	if( cl_connectChain[0] == '\0' ) {
		if( message ) {
			const auto kind = wsw::ui::UISystem::ConnectionFailKind::TryReconnecting;
			auto *uiSystem = wsw::ui::UISystem::instance();
			uiSystem->notifyOfFailedConnection( wsw::StringView( message ), kind );
		}
	} else {
		const char *s = strchr( cl_connectChain, ',' );
		if( s ) {
			cl_connectChain[s - cl_connectChain] = '\0';
		} else {
			s = cl_connectChain + strlen( cl_connectChain ) - 1;
		}
		Q_snprintfz( cl_nextString, sizeof( cl_nextString ), "connect \"%s\" \"%s\"", cl_connectChain, s + 1 );
	}

done:
	SCR_EndLoadingPlaque(); // get rid of loading plaque

	// in case we disconnect while in download phase
	CL_FreeDownloadList();

	CL_ExecuteNext(); // start next action if any is defined
}

void CL_Disconnect_f( const CmdArgs & ) {
	cl_connectChain[0] = '\0';
	cl_nextString[0] = '\0';

	// We have to shut down webdownloading first
	if( cls.download.web ) {
		cls.download.disconnect = true;
		return;
	}

	CL_Disconnect( NULL );
}

/*
* CL_Changing_f
*
* Just sent as a hint to the client that they should
* drop to full console
*/
void CL_Changing_f( const CmdArgs & ) {
	//ZOID
	//if we are downloading, we don't change!  This so we don't suddenly stop downloading a map
	if( cls.download.filenum || cls.download.web ) {
		return;
	}

	if( cls.demoRecorder.recording ) {
		CL_Stop_f( {} );
	}

	Com_DPrintf( "CL:Changing\n" );

	cl.configStrings.clear();

	// ignore snapshots from previous connection
	cl.pendingSnapNum = cl.currentSnapNum = cl.receivedSnapNum = 0;

	CL_SetClientState( CA_CONNECTED ); // not active anymore, but not disconnected
}

/*
* CL_ServerReconnect_f
*
* The server is changing levels
*/
void CL_ServerReconnect_f( const CmdArgs & ) {
	if( cls.demoPlayer.playing ) {
		return;
	}

	//if we are downloading, we don't change!  This so we don't suddenly stop downloading a map
	if( cls.download.filenum || cls.download.web ) {
		cls.download.pending_reconnect = true;
		return;
	}

	if( cls.state < CA_CONNECTED ) {
		Com_Printf( "Error: CL_ServerReconnect_f while not connected\n" );
		return;
	}

	if( cls.demoRecorder.recording ) {
		CL_Stop_f( {} );
	}

	cls.connect_count = 0;
	cls.rejected = false;

	CL_GameModule_Shutdown();
	SoundSystem::instance()->stopAllSounds( SoundSystem::StopAndClear | SoundSystem::StopMusic );

	Com_Printf( "Reconnecting...\n" );

#ifdef TCP_ALLOW_CONNECT
	cls.connect_time = Sys_Milliseconds();
#else
	cls.connect_time = Sys_Milliseconds() - 1500;
#endif

	cl.configStrings.clear();

	CL_SetClientState( CA_HANDSHAKE );
	CL_AddReliableCommand( "new" );
}

/*
* CL_Reconnect_f
*
* User reconnect command.
*/
void CL_Reconnect_f( const CmdArgs & ) {
	char *servername;
	socket_type_t servertype;
	netadr_t serveraddress;

	if( !cls.servername ) {
		Com_Printf( "Can't reconnect, never connected\n" );
		return;
	}

	cl_connectChain[0] = '\0';
	cl_nextString[0] = '\0';

	servername = Q_strdup( cls.servername );
	servertype = cls.servertype;
	serveraddress = cls.serveraddress;
	CL_Disconnect( NULL );
	CL_Connect( servername, servertype, &serveraddress, "" );
	Q_free( servername );
}

/*
* CL_ConnectionlessPacket
*
* Responses to broadcasts, etc
*/
static void CL_ConnectionlessPacket( const socket_t *socket, const netadr_t *address, msg_t *msg ) {
	MSG_BeginReading( msg );
	MSG_ReadInt32( msg ); // skip the -1

	const char *s = MSG_ReadStringLine( msg );

	if( !strncmp( s, "getserversResponse", 18 ) ) {
		Com_DPrintf( "%s: %s\n", NET_AddressToString( address ), "getserversResponse" );
		ServerList::instance()->parseGetServersResponse( socket, *address, msg );
		return;
	}

	static CmdArgsSplitter argsSplitter;
	const CmdArgs &cmdArgs = argsSplitter.exec( wsw::StringView( s ) );

	const char *c = cmdArgs[0].data();

	Com_DPrintf( "%s: %s\n", NET_AddressToString( address ), s );

	// jal : wsw
	// server responding to a detailed info broadcast
	if( !strcmp( c, "infoResponse" ) ) {
		ServerList::instance()->parseGetInfoResponse( socket, *address, msg );
		return;
	}
	if( !strcmp( c, "statusResponse" ) ) {
		ServerList::instance()->parseGetStatusResponse( socket, *address, msg );
		return;
	}

	if( cls.demoPlayer.playing ) {
		Com_DPrintf( "Received connectionless cmd \"%s\" from %s while playing a demo\n", s, NET_AddressToString( address ) );
		return;
	}

	// server connection
	if( !strcmp( c, "client_connect" ) ) {
		if( cls.state == CA_CONNECTED ) {
			Com_Printf( "Dup connect received.  Ignored.\n" );
			return;
		}
		// these two are from Q3
		if( cls.state != CA_CONNECTING ) {
			Com_Printf( "client_connect packet while not connecting.  Ignored.\n" );
			return;
		}
		if( !NET_CompareAddress( address, &cls.serveraddress ) ) {
			Com_Printf( "client_connect from a different address.  Ignored.\n" );
			Com_Printf( "Was %s should have been %s\n", NET_AddressToString( address ),
						NET_AddressToString( &cls.serveraddress ) );
			return;
		}

		cls.rejected = false;

		Q_strncpyz( cls.session, MSG_ReadStringLine( msg ), sizeof( cls.session ) );

		Netchan_Setup( &cls.netchan, socket, address, Netchan_GamePort() );
		cl.configStrings.clear();
		CL_SetClientState( CA_HANDSHAKE );
		CL_AddReliableCommand( "new" );
		return;
	}

	// reject packet, used to inform the client that connection attemp didn't succeed
	if( !strcmp( c, "reject" ) ) {
		int rejectflag;

		if( cls.state != CA_CONNECTING ) {
			Com_Printf( "reject packet while not connecting, ignored\n" );
			return;
		}
		if( !NET_CompareAddress( address, &cls.serveraddress ) ) {
			Com_Printf( "reject from a different address, ignored\n" );
			Com_Printf( "Was %s should have been %s\n", NET_AddressToString( address ),
						NET_AddressToString( &cls.serveraddress ) );
			return;
		}

		cls.rejected = true;

		cls.rejecttype = atoi( MSG_ReadStringLine( msg ) );
		if( cls.rejecttype < 0 || cls.rejecttype >= DROP_TYPE_TOTAL ) {
			cls.rejecttype = DROP_TYPE_GENERAL;
		}

		rejectflag = atoi( MSG_ReadStringLine( msg ) );

		Q_strncpyz( cls.rejectmessage, MSG_ReadStringLine( msg ), sizeof( cls.rejectmessage ) );
		if( strlen( cls.rejectmessage ) > sizeof( cls.rejectmessage ) - 2 ) {
			cls.rejectmessage[strlen( cls.rejectmessage ) - 2] = '.';
			cls.rejectmessage[strlen( cls.rejectmessage ) - 1] = '.';
			cls.rejectmessage[strlen( cls.rejectmessage )] = '.';
		}

		Com_Printf( "Connection refused: %s\n", cls.rejectmessage );
		if( rejectflag & DROP_FLAG_AUTORECONNECT ) {
			Com_Printf( "Automatic reconnecting allowed.\n" );
		} else {
			Com_Printf( "Automatic reconnecting not allowed.\n" );
			const auto dropType = cls.rejecttype;
			CL_Disconnect( nullptr );
			using Kind = wsw::ui::UISystem::ConnectionFailKind;
			auto *const uiSystem = wsw::ui::UISystem::instance();
			if( dropType == DROP_TYPE_GENERAL ) {
				uiSystem->notifyOfFailedConnection( wsw::StringView( cls.rejectmessage ), Kind::TryReconnecting );
			} else if( dropType == DROP_TYPE_PASSWORD ) {
				uiSystem->notifyOfFailedConnection( wsw::StringView( cls.rejectmessage ), Kind::PasswordRequired );
			} else {
				uiSystem->notifyOfFailedConnection( wsw::StringView( cls.rejectmessage ), Kind::DontReconnect );
			}
		}

		return;
	}

	// remote command from gui front end
	if( !strcmp( c, "cmd" ) ) {
		if( !NET_IsLocalAddress( address ) ) {
			Com_Printf( "Command packet from remote host, ignored\n" );
			return;
		}
		Sys_AppActivate();
		s = MSG_ReadString( msg );
		CL_Cbuf_AppendCommand( s );
		CL_Cbuf_AppendCommand( "\n" );
		return;
	}
	// print command from somewhere
	if( !strcmp( c, "print" ) ) {
		// CA_CONNECTING is allowed, because old servers send protocol mismatch connection error message with it
		if( ( ( cls.state != CA_UNINITIALIZED && cls.state != CA_DISCONNECTED ) &&
			  NET_CompareAddress( address, &cls.serveraddress ) ) ||
			( rcon_address->string[0] != '\0' && NET_CompareAddress( address, &cls.rconaddress ) ) ) {
			s = MSG_ReadString( msg );
			Com_Printf( "%s", s );
			return;
		} else {
			Com_Printf( "Print packet from unknown host, ignored\n" );
			return;
		}
	}

	// ping from somewhere
	if( !strcmp( c, "ping" ) ) {
		// send any args back with the acknowledgement
		Netchan_OutOfBandPrint( socket, address, "ack %s", Cmd_Args() );
		return;
	}

	// ack from somewhere
	if( !strcmp( c, "ack" ) ) {
		return;
	}

	// challenge from the server we are connecting to
	if( !strcmp( c, "challenge" ) ) {
		// these two are from Q3
		if( cls.state != CA_CONNECTING ) {
			Com_Printf( "challenge packet while not connecting, ignored\n" );
			return;
		}
		if( !NET_CompareAddress( address, &cls.serveraddress ) ) {
			Com_Printf( "challenge from a different address, ignored\n" );
			Com_Printf( "Was %s", NET_AddressToString( address ) );
			Com_Printf( " should have been %s\n", NET_AddressToString( &cls.serveraddress ) );
			return;
		}

		cls.challenge = atoi( Cmd_Argv( 1 ) );
		//wsw : r1q2[start]
		//r1: reset the timer so we don't send dup. getchallenges
		cls.connect_time = Sys_Milliseconds();
		//wsw : r1q2[end]
		CL_SendConnectPacket();
		return;
	}

	// echo request from server
	if( !strcmp( c, "echo" ) ) {
		Netchan_OutOfBandPrint( socket, address, "%s", Cmd_Argv( 1 ) );
		return;
	}

	Com_Printf( "Unknown connectionless packet from %s\n%s\n", NET_AddressToString( address ), c );
}

/*
* CL_ProcessPacket
*/
static bool CL_ProcessPacket( netchan_t *netchan, msg_t *msg ) {
	int zerror;

	if( !Netchan_Process( netchan, msg ) ) {
		return false; // wasn't accepted for some reason

	}
	// now if compressed, expand it
	MSG_BeginReading( msg );
	MSG_ReadInt32( msg ); // sequence
	MSG_ReadInt32( msg ); // sequence_ack
	if( msg->compressed ) {
		zerror = Netchan_DecompressMessage( msg );
		if( zerror < 0 ) {
			// compression error. Drop the packet
			Com_Printf( "CL_ProcessPacket: Compression error %i. Dropping packet\n", zerror );
			return false;
		}
	}

	return true;
}

/*
* CL_ReadPackets
*/
void CL_ReadPackets( void ) {
	static msg_t msg;
	static uint8_t msgData[MAX_MSGLEN];
	int socketind, ret;
	socket_t *socket;
	netadr_t address;

	socket_t* sockets [] =
	{
		&cls.socket_loopback,
		&cls.socket_udp,
		&cls.socket_udp6,
#ifdef TCP_ALLOW_CONNECT
		&cls.socket_tcp
#endif
	};

	MSG_Init( &msg, msgData, sizeof( msgData ) );

	for( socketind = 0; socketind < (int)( sizeof( sockets ) / sizeof( sockets[0] ) ); socketind++ ) {
		socket = sockets[socketind];

#ifdef TCP_ALLOW_CONNECT
		if( socket->type == SOCKET_TCP && !socket->connected ) {
			continue;
		}
#endif

		while( socket->open && ( ret = NET_GetPacket( socket, &address, &msg ) ) != 0 ) {
			if( ret == -1 ) {
				Com_Printf( "Error receiving packet with %s: %s\n", NET_SocketToString( socket ), NET_ErrorString() );
				if( cls.reliable && cls.socket == socket ) {
					CL_Disconnect( va( "Error receiving packet: %s\n", NET_ErrorString() ) );
				}

				continue;
			}

			// remote command packet
			if( *(int *)msg.data == -1 ) {
				CL_ConnectionlessPacket( socket, &address, &msg );
				continue;
			}

			if( cls.demoPlayer.playing ) {
				// only allow connectionless packets during demo playback
				continue;
			}

			if( cls.state == CA_DISCONNECTED || cls.state == CA_GETTING_TICKET || cls.state == CA_CONNECTING ) {
				Com_DPrintf( "%s: Not connected\n", NET_AddressToString( &address ) );
				continue; // dump it if not connected
			}

			if( msg.cursize < 8 ) {
				//wsw : r1q2[start]
				//r1: delegated to DPrintf (someone could spam your console with crap otherwise)
				Com_DPrintf( "%s: Runt packet\n", NET_AddressToString( &address ) );
				//wsw : r1q2[end]
				continue;
			}

			//
			// packet from server
			//
			if( !NET_CompareAddress( &address, &cls.netchan.remoteAddress ) ) {
				Com_DPrintf( "%s: Sequenced packet without connection\n", NET_AddressToString( &address ) );
				continue;
			}
			if( !CL_ProcessPacket( &cls.netchan, &msg ) ) {
				continue; // wasn't accepted for some reason, like only one fragment of bigger message

			}
			CL_ParseServerMessage( &msg );
			cls.lastPacketReceivedTime = cls.realtime;

#ifdef TCP_ALLOW_CONNECT
			// we might have just been disconnected
			if( socket->type == SOCKET_TCP && !socket->connected ) {
				break;
			}
#endif
		}
	}

	if( cls.demoPlayer.playing ) {
		return;
	}

	// not expected, but could happen if cls.realtime is cleared and lastPacketReceivedTime is not
	if( cls.lastPacketReceivedTime > cls.realtime ) {
		cls.lastPacketReceivedTime = cls.realtime;
	}

	// check timeout
	if( cls.state >= CA_HANDSHAKE && cls.lastPacketReceivedTime ) {
		if( cls.lastPacketReceivedTime + cl_timeout->value * 1000 < cls.realtime ) {
			if( ++cl.timeoutcount > 5 ) { // timeoutcount saves debugger
				Com_Printf( "\nServer connection timed out.\n" );
				CL_Disconnect( "Connection timed out" );
				return;
			}
		}
	} else {
		cl.timeoutcount = 0;
	}
}

//=============================================================================

/*
* CL_Userinfo_f
*/
static void CL_Userinfo_f( const CmdArgs & ) {
	Com_Printf( "User info settings:\n" );
	Info_Print( Cvar_Userinfo() );
}

static int precache_check; // for autodownload of precache items
static int precache_spawncount;
static int precache_tex;
static int precache_pure;

#define PLAYER_MULT 5

// ENV_CNT is map load
#define ENV_CNT ( CS_PLAYERINFOS + MAX_CLIENTS * PLAYER_MULT )
#define TEXTURE_CNT ( ENV_CNT + 1 )

static unsigned int CL_LoadMap( const char *name ) {
	int i;
	int areas;

	unsigned int map_checksum;

	assert( !cl.cms );

	// if local server is running, share the collision model,
	// increasing the ref counter
	if( Com_ServerState() ) {
		cl.cms = Com_ServerCM( &map_checksum );
	} else {
		cl.cms = CM_New();
		CM_LoadMap( cl.cms, name, true, &map_checksum );
	}

	CM_AddReference( cl.cms );

	assert( cl.cms );

	// allocate memory for areabits
	areas = CM_NumAreas( cl.cms );
	areas *= CM_AreaRowSize( cl.cms );

	cl.frames_areabits = (uint8_t *)Q_malloc( UPDATE_BACKUP * areas );
	for( i = 0; i < UPDATE_BACKUP; i++ ) {
		cl.snapShots[i].areabytes = areas;
		cl.snapShots[i].areabits = cl.frames_areabits + i * areas;
	}

	return map_checksum;
}

void CL_RequestNextDownload( void ) {
	char tempname[MAX_QPATH + 4];
	purelist_t *purefile;
	int i;

	if( cls.state != CA_CONNECTED ) {
		return;
	}

	// pure list
	if( cls.sv_pure ) {
		// skip
		if( !cl_downloads->integer ) {
			precache_pure = -1;
		}

		// try downloading
		if( precache_pure != -1 ) {
			i = 0;
			purefile = cls.purelist;
			while( i < precache_pure && purefile ) {
				purefile = purefile->next;
				i++;
			}

			while( purefile ) {
				precache_pure++;
				if( !CL_CheckOrDownloadFile( purefile->filename ) ) {
					return;
				}
				purefile = purefile->next;
			}
			precache_pure = -1;
		}

		if( precache_pure == -1 ) {
			bool failed = false;
			char message[MAX_STRING_CHARS];

			Q_snprintfz( message, sizeof( message ), "Pure check failed:" );

			purefile = cls.purelist;
			while( purefile ) {
				Com_DPrintf( "Adding pure file: %s\n", purefile->filename );
				if( !FS_AddPurePak( purefile->checksum ) ) {
					failed = true;
					Q_strncatz( message, " ", sizeof( message ) );
					Q_strncatz( message, purefile->filename, sizeof( message ) );
				}
				purefile = purefile->next;
			}

			if( failed ) {
				Com_Error( ERR_DROP, "%s", message );
				return;
			}
		}
	}

	// skip if download not allowed
	if( !cl_downloads->integer && precache_check < ENV_CNT ) {
		precache_check = ENV_CNT;
	}

	//ZOID
	if( precache_check == CS_WORLDMODEL ) { // confirm map
		precache_check = CS_MODELS; // 0 isn't used

		if( !CL_CheckOrDownloadFile( cl.configStrings.getWorldModel()->data() ) ) {
			return; // started a download
		}
	}

	if( precache_check >= CS_MODELS && precache_check < CS_MODELS + MAX_MODELS ) {
		for(;; ) {
			if( precache_check >= CS_MODELS + MAX_MODELS ) {
				break;
			}
			const auto maybeConfigString = cl.configStrings.get( precache_check );
			if( !maybeConfigString ) {
				break;
			}

			const auto string = *maybeConfigString;
			precache_check++;

			// disable playermodel downloading for now
			if( string.startsWith( '*' ) || string.startsWith( '$' ) || string.startsWith( '#' ) ) {
				continue;
			}

			// started a download
			if( !CL_CheckOrDownloadFile( string.data() ) ) {
				return;
			}
		}
		precache_check = CS_SOUNDS;
	}

	if( precache_check >= CS_SOUNDS && precache_check < CS_SOUNDS + MAX_SOUNDS ) {
		if( precache_check == CS_SOUNDS ) {
			precache_check++; // zero is blank
		}
		for(;; ) {
			if( precache_check >= CS_SOUNDS + MAX_SOUNDS ) {
				break;
			}
			const auto maybeConfigString = cl.configStrings.get( precache_check );
			if( !maybeConfigString ) {
				break;
			}

			const auto string = *maybeConfigString;
			precache_check++;

			// sexed sounds
			if( string.startsWith( '*' ) ) {
				continue;
			}

			Q_strncpyz( tempname, string.data(), sizeof( tempname ) );
			if( !COM_FileExtension( tempname ) ) {
				if( !FS_FirstExtension( tempname, SOUND_EXTENSIONS, NUM_SOUND_EXTENSIONS ) ) {
					COM_DefaultExtension( tempname, ".wav", sizeof( tempname ) );
					if( !CL_CheckOrDownloadFile( tempname ) ) {
						return; // started a download
					}
				}
			} else {
				if( !CL_CheckOrDownloadFile( tempname ) ) {
					return; // started a download
				}
			}
		}
		precache_check = CS_IMAGES;
	}
	if( precache_check >= CS_IMAGES && precache_check < CS_IMAGES + MAX_IMAGES ) {
		if( precache_check == CS_IMAGES ) {
			precache_check++; // zero is blank

		}
		// precache phase completed
		precache_check = ENV_CNT;
	}

	if( precache_check == ENV_CNT ) {
		bool restart = false;
		bool vid_restart = false;
		const char *restart_msg = "";
		unsigned map_checksum;

		// we're done with the download phase, so clear the list
		CL_FreeDownloadList();
		if( cls.pure_restart ) {
			restart = true;
			restart_msg = "Pure server. Restarting media...";
		}
		if( cls.download.successCount ) {
			restart = true;
			vid_restart = true;
			restart_msg = "Files downloaded. Restarting media...";
		}

		CL_BeginRegistration();

		if( restart ) {
			Com_Printf( "%s\n", restart_msg );

			if( vid_restart ) {
				// no media is going to survive a vid_restart...
				CL_Cmd_ExecuteNow( "s_restart 1\n" );
			} else {
				// make sure all media assets will be freed
				CL_EndRegistration();
				CL_BeginRegistration();
			}
		}

		if( !vid_restart ) {
			CL_RestartMedia();
		}

		cls.download.successCount = 0;

		map_checksum = CL_LoadMap( cl.configStrings.getWorldModel()->data() );
		if( map_checksum != (unsigned)atoi( cl.configStrings.getMapCheckSum()->data() ) ) {
			Com_Error( ERR_DROP, "Local map version differs from server: %u != '%s'",
					   map_checksum, cl.configStrings.getMapCheckSum()->data() );
			return;
		}

		precache_check = TEXTURE_CNT;
	}

	if( precache_check == TEXTURE_CNT ) {
		precache_check = TEXTURE_CNT + 1;
		precache_tex = 0;
	}

	// confirm existance of textures, download any that don't exist
	if( precache_check == TEXTURE_CNT + 1 ) {
		precache_check = TEXTURE_CNT + 999;
	}

	// load client game module
	CL_GameModule_Init();
	CL_AddReliableCommand( va( "begin %i\n", precache_spawncount ) );
}

/*
* CL_Precache_f
*
* The server will send this command right
* before allowing the client into the server
*/
void CL_Precache_f( const CmdArgs &cmdArgs ) {
	FS_RemovePurePaks();

	if( cls.demoPlayer.playing ) {
		if( !cls.demoPlayer.play_jump ) {
			CL_LoadMap( cl.configStrings.getWorldModel()->data() );

			CL_GameModule_Init();
		} else {
			CL_GameModule_Reset();
			SoundSystem::instance()->stopAllSounds();
		}

		cls.demoPlayer.play_ignore_next_frametime = true;

		return;
	}

	precache_pure = 0;
	precache_check = CS_WORLDMODEL;
	precache_spawncount = atoi( Cmd_Argv( 1 ) );

	CL_RequestNextDownload();
}

/*
* CL_WriteConfiguration
*
* Writes key bindings, archived cvars and aliases to a config file
*/
static void CL_WriteConfiguration( const char *name, bool warn ) {
	int file;

	if( FS_FOpenFile( name, &file, FS_WRITE ) == -1 ) {
		Com_Printf( "Couldn't write %s.\n", name );
		return;
	}

	if( warn ) {
		// Write 'Warsow' with UTF-8 encoded section sign in place of the s to aid
		// text editors in recognizing the file's encoding
		FS_Printf( file, "// This file is automatically generated by " APPLICATION_UTF8 ", do not modify.\r\n" );
	}

	FS_Printf( file, "\r\n// key bindings\r\n" );
	Key_WriteBindings( file );

	FS_Printf( file, "\r\n// variables\r\n" );
	Cvar_WriteVariables( file );

	FS_Printf( file, "\r\n// aliases\r\n" );
	Cmd_WriteAliases( file );

	FS_FCloseFile( file );
}


/*
* CL_WriteConfig_f
*/
static void CL_WriteConfig_f( const CmdArgs &cmdArgs ) {
	char *name;
	int name_size;

	if( Cmd_Argc() != 2 ) {
		Com_Printf( "Usage: writeconfig <filename>\n" );
		return;
	}

	name_size = sizeof( char ) * ( strlen( Cmd_Argv( 1 ) ) + strlen( ".cfg" ) + 1 );
	name = (char *)Q_malloc( name_size );
	Q_strncpyz( name, Cmd_Argv( 1 ), name_size );
	COM_SanitizeFilePath( name );

	if( !COM_ValidateRelativeFilename( name ) ) {
		Com_Printf( "Invalid filename" );
		Q_free( name );
		return;
	}

	COM_DefaultExtension( name, ".cfg", name_size );

	Com_Printf( "Writing: %s\n", name );
	CL_WriteConfiguration( name, false );

	Q_free( name );
}

static void CL_Help_f( const CmdArgs & ) {
	Com_Printf( "Type commands here. Use TAB key for getting suggestions. Use PgUp/PgDn keys for scrolling.\n");
	Com_Printf( "These commands can be useful:\n" );
	Com_Printf( "cvarlist - Displays the list of all console vars\n" );
	Com_Printf( "cvarlist <pattern> - Displays a list of console vars that match the pattern\n" );
	Com_Printf( "Example: cvarlist zoom* - Displays a list of console vars that are related to zoom\n" );
	Com_Printf( "cmdlist - Displays the list of all console commands\n" );
	Com_Printf( "cmdlist <pattern> - Displays a list of console commands that match the pattern\n" );
	Com_Printf( "Example: cmdlist *restart - Displays a list of commands that restart their corresponding subsystems\n" );
}

/*
* CL_SetClientState
*/
void CL_SetClientState( int state ) {
	cls.state = (connstate_t)state;
	Com_SetClientState( state );

	if( state <= CA_DISCONNECTED ) {
		Steam_AdvertiseGame( NULL, 0 );
	}

	switch( state ) {
		case CA_DISCONNECTED:
			Con_Close();
			break;
		case CA_GETTING_TICKET:
		case CA_CONNECTING:
			cls.cgameActive = false;
			Con_Close();
			SoundSystem::instance()->stopBackgroundTrack();
			SoundSystem::instance()->clear();
			break;
		case CA_CONNECTED:
			cls.cgameActive = false;
			Con_Close();
			Cvar_FixCheatVars();
			break;
		case CA_ACTIVE:
			cl_connectChain[0] = '\0';
			CL_EndRegistration();
			Con_Close();
			CL_AddReliableCommand( "svmotd 1" );
			SoundSystem::instance()->clear();
			break;
		default:
			break;
	}
}

/*
* CL_GetClientState
*/
connstate_t CL_GetClientState( void ) {
	return cls.state;
}

/*
* CL_InitMedia
*/
void CL_InitMedia( void ) {
	if( cls.mediaInitialized ) {
		return;
	}
	if( cls.state == CA_UNINITIALIZED ) {
		return;
	}
	if( !VID_RefreshIsActive() ) {
		return;
	}

	// random seed to be shared among game modules so pseudo-random stuff is in sync
	if( cls.state != CA_CONNECTED ) {
		srand( time( NULL ) );
		cls.mediaRandomSeed = rand();
	}

	cls.mediaInitialized = true;

	SoundSystem::instance()->stopAllSounds( SoundSystem::StopAndClear | SoundSystem::StopMusic );

	// register console font and background
	SCR_RegisterConsoleMedia();

	SCR_EnableQuickMenu( false );

	// load user interface
	wsw::ui::UISystem::init( VID_GetWindowWidth(), VID_GetWindowHeight() );
}

/*
* CL_ShutdownMedia
*/
void CL_ShutdownMedia( void ) {
	if( !cls.mediaInitialized ) {
		return;
	}
	if( !VID_RefreshIsActive() ) {
		return;
	}

	cls.mediaInitialized = false;

	SoundSystem::instance()->stopAllSounds( SoundSystem::StopAndClear | SoundSystem::StopMusic );

	// shutdown cgame
	CL_GameModule_Shutdown();

	// shutdown user interface
	wsw::ui::UISystem::shutdown();

	SCR_ShutDownConsoleMedia();
}

/*
* CL_RestartMedia
*/
void CL_RestartMedia( void ) {
	if( !VID_RefreshIsActive() ) {
		return;
	}

	if( cls.mediaInitialized ) {
		// shutdown cgame
		CL_GameModule_Shutdown();

		cls.mediaInitialized = false;
	}

	SoundSystem::instance()->stopAllSounds( SoundSystem::StopAndClear | SoundSystem::StopMusic );

	// random seed to be shared among game modules so pseudo-random stuff is in sync
	if( cls.state != CA_CONNECTED ) {
		srand( time( NULL ) );
		cls.mediaRandomSeed = rand();
	}

	cls.mediaInitialized = true;

	FTLIB_TouchAllFonts();

	// register console font and background
	SCR_RegisterConsoleMedia();
}

/*
* CL_S_Restart
*
* Restart the sound subsystem so it can pick up new parameters and flush all sounds
*/
void CL_S_Restart( bool noVideo, const CmdArgs &cmdArgs ) {
	bool verbose = ( Cmd_Argc() >= 2 ? true : false );

	// The cgame and game must also be forced to restart because handles will become invalid
	// VID_Restart also forces an audio restart
	if( !noVideo ) {
		VID_Restart( verbose, true );
		VID_CheckChanges();
	} else {
		CL_SoundModule_Shutdown( verbose );
		CL_SoundModule_Init( verbose );
	}
}

/*
* CL_S_Restart_f
*
* Restart the sound subsystem so it can pick up new parameters and flush all sounds
*/
static void CL_S_Restart_f( const CmdArgs &cmdArgs ) {
	CL_S_Restart( false, cmdArgs );
}

/*
* CL_ShowIP_f - wsw : jal : taken from Q3 (it only shows the ip when server was started)
*/
static void CL_ShowIP_f( const CmdArgs & ) {
	NET_ShowIP();
}

/*
* CL_ShowServerIP_f - wsw : pb : show the ip:port of the server the client is connected to
*/
static void CL_ShowServerIP_f( const CmdArgs & ) {
	if( cls.state != CA_CONNECTED && cls.state != CA_ACTIVE ) {
		Com_Printf( "Not connected to a server\n" );
		return;
	}

	Com_Printf( "Connected to server:\n" );
	Com_Printf( "Name: %s\n", cls.servername );
	Com_Printf( "Address: %s\n", NET_AddressToString( &cls.serveraddress ) );
}

/*
* CL_InitLocal
*/
static void CL_InitLocal( void ) {
	cvar_t *name, *color;

	cls.state = CA_DISCONNECTED;
	Com_SetClientState( CA_DISCONNECTED );

	//
	// register our variables
	//
	cl_stereo_separation =  Cvar_Get( "cl_stereo_separation", "0.4", CVAR_ARCHIVE );
	cl_stereo =     Cvar_Get( "cl_stereo", "0", CVAR_ARCHIVE );

	cl_maxfps =     Cvar_Get( "cl_maxfps", "250", CVAR_ARCHIVE );
	cl_sleep =      Cvar_Get( "cl_sleep", "1", CVAR_ARCHIVE );
	cl_pps =        Cvar_Get( "cl_pps", "62", CVAR_ARCHIVE );

	cl_extrapolationTime =  Cvar_Get( "cl_extrapolationTime", "0", CVAR_DEVELOPER );
	cl_extrapolate = Cvar_Get( "cl_extrapolate", "1", CVAR_ARCHIVE );

	cl_infoservers =  Cvar_Get( "infoservers", DEFAULT_INFO_SERVERS_IPS, 0 );

	cl_shownet =        Cvar_Get( "cl_shownet", "0", 0 );
	cl_timeout =        Cvar_Get( "cl_timeout", "120", 0 );
	cl_timedemo =       Cvar_Get( "timedemo", "0", CVAR_CHEAT );

	rcon_client_password =  Cvar_Get( "rcon_password", "", 0 );
	rcon_address =      Cvar_Get( "rcon_address", "", 0 );

	// wsw : debug netcode
	cl_debug_serverCmd =    Cvar_Get( "cl_debug_serverCmd", "0", CVAR_ARCHIVE | CVAR_CHEAT );
	cl_debug_timeDelta =    Cvar_Get( "cl_debug_timeDelta", "0", CVAR_ARCHIVE /*|CVAR_CHEAT*/ );

	cl_downloads =      Cvar_Get( "cl_downloads", "1", CVAR_ARCHIVE );
	cl_downloads_from_web = Cvar_Get( "cl_downloads_from_web", "1", CVAR_ARCHIVE | CVAR_READONLY );
	cl_downloads_from_web_timeout = Cvar_Get( "cl_downloads_from_web_timeout", "600", CVAR_ARCHIVE );
	cl_download_allow_modules = Cvar_Get( "cl_download_allow_modules", "1", CVAR_ARCHIVE );

	//
	// userinfo
	//
	info_password =     Cvar_Get( "password", "", CVAR_USERINFO );
	rate =          Cvar_Get( "rate", "60000", CVAR_DEVELOPER ); // FIXME

	name = Cvar_Get( "name", "", CVAR_USERINFO | CVAR_ARCHIVE );
	if( !name->string[0] ) {
		char steamname[MAX_NAME_BYTES * 4], *steamnameIn = steamname, *steamnameOut = steamname, c;
		steamname[0] = '\0';
		Steam_GetPersonaName( steamname, sizeof( steamname ) );
		while( ( c = *steamnameIn ) != '\0' ) {
			steamnameIn++;
			if( ( c < 32 ) || ( c >= 127 ) || ( c == '\\' ) || ( c == ';' ) || ( c == '"' ) ) {
				continue;
			}

			*( steamnameOut++ ) = c;
		}
		*steamnameOut = '\0';

		if( !( COM_RemoveColorTokens( steamname )[0] ) ) {
			// Avoid using the default random() macro as it has a default seed.
			std::minstd_rand0 randomEngine( (std::minstd_rand0::result_type)time( nullptr ) );
			// Avoid using black and grey colors.
			int colorNum;
			do {
				colorNum = (int)( randomEngine() % 10 );
			} while( colorNum == 0 || colorNum == 9 );
			int parts[3];
			for( int &part: parts ) {
				part = (int)( randomEngine() % 100 );
			}
			Q_snprintfz( steamname, sizeof( steamname ), "^%dplayer%02d%02d%02d", colorNum, parts[0], parts[1], parts[2] );
		}

		Cvar_Set( name->name, steamname );
	}

	Cvar_Get( "clan", "", CVAR_USERINFO | CVAR_ARCHIVE );
	Cvar_Get( "model", DEFAULT_PLAYERMODEL, CVAR_USERINFO | CVAR_ARCHIVE );
	Cvar_Get( "skin", DEFAULT_PLAYERSKIN, CVAR_USERINFO | CVAR_ARCHIVE );
	Cvar_Get( "hand", "0", CVAR_USERINFO | CVAR_ARCHIVE );
	Cvar_Get( "handicap", "0", CVAR_USERINFO | CVAR_ARCHIVE );

	Cvar_Get( "cl_download_name", "", CVAR_READONLY );
	Cvar_Get( "cl_download_percent", "0", CVAR_READONLY );

	color = Cvar_Get( "color", "", CVAR_ARCHIVE | CVAR_USERINFO );
	if( COM_ReadColorRGBString( color->string ) == -1 ) {
		time_t long_time; // random isn't working fine at this point.
		unsigned int hash; // so we get the user local time and use its hash
		int rgbcolor;
		time( &long_time );
		hash = COM_SuperFastHash64BitInt( ( uint64_t )long_time );
		rgbcolor = COM_ValidatePlayerColor( COLOR_RGB( hash & 0xff, ( hash >> 8 ) & 0xff, ( hash >> 16 ) & 0xff ) );
		Cvar_Set( color->name, va( "%i %i %i", COLOR_R( rgbcolor ), COLOR_G( rgbcolor ), COLOR_B( rgbcolor ) ) );
	}

	//
	// register our commands
	//
	CL_Cmd_Register( "s_restart", CL_S_Restart_f );
	CL_Cmd_Register( "cmd", CL_ForwardToServer_f );
	CL_Cmd_Register( "userinfo", CL_Userinfo_f );
	CL_Cmd_Register( "disconnect", CL_Disconnect_f );
	CL_Cmd_Register( "record", CL_Record_f );
	CL_Cmd_Register( "stop", CL_Stop_f );
	CL_Cmd_Register( "quit", CL_Quit_f );
	CL_Cmd_Register( "connect", CL_Connect_f );
#if defined( TCP_ALLOW_CONNECT ) && defined( TCP_ALLOW_CONNECT_CLIENT )
	CL_Cmd_Register( "tcpconnect", CL_TCPConnect_f );
#endif
	CL_Cmd_Register( "reconnect", CL_Reconnect_f );
	CL_Cmd_Register( "rcon", CL_Rcon_f );
	CL_Cmd_Register( "writeconfig", CL_WriteConfig_f );
	CL_Cmd_Register( "showip", CL_ShowIP_f ); // jal : wsw : print our ip
	CL_Cmd_Register( "demo", CL_PlayDemo_f );
	CL_Cmd_Register( "next", CL_SetNext_f );
	CL_Cmd_Register( "demopause", CL_PauseDemo_f );
	CL_Cmd_Register( "demojump", CL_DemoJump_f );
	CL_Cmd_Register( "showserverip", CL_ShowServerIP_f );
	CL_Cmd_Register( "downloadstatus", CL_DownloadStatus_f );
	CL_Cmd_Register( "downloadcancel", CL_DownloadCancel_f );
	CL_Cmd_Register( "help", CL_Help_f );

	Cmd_SetCompletionFunc( "demo", CL_DemoComplete );
}

/*
* CL_ShutdownLocal
*/
static void CL_ShutdownLocal( void ) {
	cls.state = CA_UNINITIALIZED;
	Com_SetClientState( CA_UNINITIALIZED );

	CL_Cmd_Unregister( "s_restart" );
	CL_Cmd_Unregister( "cmd" );
	CL_Cmd_Unregister( "userinfo" );
	CL_Cmd_Unregister( "disconnect" );
	CL_Cmd_Unregister( "record" );
	CL_Cmd_Unregister( "stop" );
	CL_Cmd_Unregister( "quit" );
	CL_Cmd_Unregister( "connect" );
#if defined( TCP_ALLOW_CONNECT )
	CL_Cmd_Unregister( "tcpconnect" );
#endif
	CL_Cmd_Unregister( "reconnect" );
	CL_Cmd_Unregister( "rcon" );
	CL_Cmd_Unregister( "writeconfig" );
	CL_Cmd_Unregister( "showip" );
	CL_Cmd_Unregister( "demo" );
	CL_Cmd_Unregister( "next" );
	CL_Cmd_Unregister( "demopause" );
	CL_Cmd_Unregister( "demojump" );
	CL_Cmd_Unregister( "showserverip" );
	CL_Cmd_Unregister( "downloadstatus" );
	CL_Cmd_Unregister( "downloadcancel" );
	CL_Cmd_Unregister( "help" );
}

//============================================================================

/*
* CL_TimedemoStats
*/
static void CL_TimedemoStats( void ) {
	if( cl_timedemo->integer && cls.demoPlayer.playing ) {
		int64_t lastTime = cl.timedemo.lastTime;
		if( lastTime != 0 ) {
			int msec;
			int64_t curTime;

			msec = RF_GetAverageFrametime();

			curTime = Sys_Milliseconds();
			if( msec  >= 100 ) {
				cl.timedemo.counts[99]++;
			} else {
				cl.timedemo.counts[msec]++;
			}
			cl.timedemo.lastTime = curTime;
			return;
		}
		cl.timedemo.lastTime = Sys_Milliseconds();
	}
}

/*
* CL_AdjustServerTime - adjust delta to new frame snap timestamp
*/
void CL_AdjustServerTime( unsigned int gameMsec ) {
	// hurry up if coming late (unless in demos)
	if( !cls.demoPlayer.playing ) {
		if( ( cl.newServerTimeDelta < cl.serverTimeDelta ) && gameMsec > 0 ) {
			cl.serverTimeDelta--;
		}
		if( cl.newServerTimeDelta > cl.serverTimeDelta ) {
			cl.serverTimeDelta++;
		}
	}

	cl.serverTime = cls.gametime + cl.serverTimeDelta;

	// it launches a new snapshot when the timestamp of the CURRENT snap is reached.
	if( cl.pendingSnapNum && ( cl.serverTime >= cl.snapShots[cl.currentSnapNum & UPDATE_MASK].serverTime ) ) {
		// fire next snapshot
		if( CL_GameModule_NewSnapshot( cl.pendingSnapNum ) ) {
			cl.previousSnapNum = cl.currentSnapNum;
			cl.currentSnapNum = cl.pendingSnapNum;
			cl.pendingSnapNum = 0;

			// getting a valid snapshot ends the connection process
			if( cls.state == CA_CONNECTED ) {
				CL_SetClientState( CA_ACTIVE );
			}
		}
	}
}

/*
* CL_RestartTimeDeltas
*/
void CL_RestartTimeDeltas( int newTimeDelta ) {
	int i;

	cl.serverTimeDelta = cl.newServerTimeDelta = newTimeDelta;
	for( i = 0; i < MAX_TIMEDELTAS_BACKUP; i++ )
		cl.serverTimeDeltas[i] = newTimeDelta;

	if( cl_debug_timeDelta->integer ) {
		Com_Printf( S_COLOR_CYAN "***** timeDelta restarted\n" );
	}
}

/*
* CL_SmoothTimeDeltas
*/
int CL_SmoothTimeDeltas( void ) {
	int i, count;
	double delta;
	snapshot_t  *snap;

	if( cls.demoPlayer.playing ) {
		if( cl.currentSnapNum <= 0 ) { // if first snap
			return cl.serverTimeDeltas[cl.pendingSnapNum & MASK_TIMEDELTAS_BACKUP];
		}

		return cl.serverTimeDeltas[cl.currentSnapNum & MASK_TIMEDELTAS_BACKUP];
	}

	i = cl.receivedSnapNum - wsw::min( MAX_TIMEDELTAS_BACKUP, 8 );
	if( i < 0 ) {
		i = 0;
	}

	for( delta = 0, count = 0; i <= cl.receivedSnapNum; i++ ) {
		snap = &cl.snapShots[i & UPDATE_MASK];
		if( snap->valid && snap->serverFrame == i ) {
			delta += (double)cl.serverTimeDeltas[i & MASK_TIMEDELTAS_BACKUP];
			count++;
		}
	}

	if( !count ) {
		return 0;
	}

	return (int)( delta / (double)count );
}

/*
* CL_UpdateSnapshot - Check for pending snapshots, and fire if needed
*/
void CL_UpdateSnapshot( void ) {
	snapshot_t  *snap;
	int i;

	// see if there is any pending snap to be fired
	if( !cl.pendingSnapNum && ( cl.currentSnapNum != cl.receivedSnapNum ) ) {
		snap = NULL;
		for( i = cl.currentSnapNum + 1; i <= cl.receivedSnapNum; i++ ) {
			if( cl.snapShots[i & UPDATE_MASK].valid && ( cl.snapShots[i & UPDATE_MASK].serverFrame > cl.currentSnapNum ) ) {
				snap = &cl.snapShots[i & UPDATE_MASK];
				//torbenh: this break was the source of the lag bug at cl_fps < sv_pps
				//break;
			}
		}

		if( snap ) { // valid pending snap found
			cl.pendingSnapNum = snap->serverFrame;

			cl.newServerTimeDelta = CL_SmoothTimeDeltas();

			if( cl_extrapolationTime->modified ) {
				if( cl_extrapolationTime->integer > (int)cl.snapFrameTime - 1 ) {
					Cvar_ForceSet( "cl_extrapolationTime", va( "%i", (int)cl.snapFrameTime - 1 ) );
				} else if( cl_extrapolationTime->integer < 0 ) {
					Cvar_ForceSet( "cl_extrapolationTime", "0" );
				}

				cl_extrapolationTime->modified = false;
			}

			if( !cls.demoPlayer.playing && cl_extrapolate->integer ) {
				cl.newServerTimeDelta += cl_extrapolationTime->integer;
			}

			// if we don't have current snap (or delay is too big) don't wait to fire the pending one
			if( ( !cls.demoPlayer.play_jump && cl.currentSnapNum <= 0 ) ||
				( !cls.demoPlayer.playing && abs( cl.newServerTimeDelta - cl.serverTimeDelta ) > 200 ) ) {
				cl.serverTimeDelta = cl.newServerTimeDelta;
			}

			// don't either wait if in a timedemo
			if( cls.demoPlayer.playing && cl_timedemo->integer ) {
				cl.serverTimeDelta = cl.newServerTimeDelta;
			}
		}
	}
}

/*
* CL_Netchan_Transmit
*/
void CL_Netchan_Transmit( msg_t *msg ) {
	// if we got here with unsent fragments, fire them all now
	Netchan_PushAllFragments( &cls.netchan );

	if( msg->cursize > 60 ) {
		int zerror = Netchan_CompressMessage( msg );
		if( zerror < 0 ) { // it's compression error, just send uncompressed
			Com_DPrintf( "CL_Netchan_Transmit (ignoring compression): Compression error %i\n", zerror );
		}
	}

	Netchan_Transmit( &cls.netchan, msg );
	cls.lastPacketSentTime = cls.realtime;
}

/*
* CL_MaxPacketsReached
*/
static bool CL_MaxPacketsReached( void ) {
	static int64_t lastPacketTime = 0;
	static float roundingMsec = 0.0f;
	int minpackettime;
	int elapsedTime;

	if( lastPacketTime > cls.realtime ) {
		lastPacketTime = cls.realtime;
	}

	if( cl_pps->integer > 62 || cl_pps->integer < 20 ) {
		Com_Printf( "'cl_pps' value is out of valid range, resetting to default\n" );
		Cvar_ForceSet( "cl_pps", va( "%s", cl_pps->dvalue ) );
	}

	elapsedTime = cls.realtime - lastPacketTime;
	if( cls.mv ) {
		minpackettime = ( 1000.0f / 2 );
	} else {
		float minTime = ( 1000.0f / cl_pps->value );

		// don't let cl_pps be smaller than sv_pps
		if( cls.state == CA_ACTIVE && !cls.demoPlayer.playing && cl.snapFrameTime ) {
			if( (unsigned int)minTime > cl.snapFrameTime ) {
				minTime = cl.snapFrameTime;
			}
		}

		minpackettime = (int)minTime;
		roundingMsec += minTime - (int)minTime;
		if( roundingMsec >= 1.0f ) {
			minpackettime += (int)roundingMsec;
			roundingMsec -= (int)roundingMsec;
		}
	}

	if( elapsedTime < minpackettime ) {
		return false;
	}

	lastPacketTime = cls.realtime;
	return true;
}

/*
* CL_SendMessagesToServer
*/
void CL_SendMessagesToServer( bool sendNow ) {
	msg_t message;
	uint8_t messageData[MAX_MSGLEN];

	if( cls.state == CA_DISCONNECTED || cls.state == CA_GETTING_TICKET || cls.state == CA_CONNECTING ) {
		return;
	}

	if( cls.demoPlayer.playing ) {
		return;
	}

	MSG_Init( &message, messageData, sizeof( messageData ) );
	MSG_Clear( &message );

	// send only reliable commands during connecting time
	if( cls.state < CA_ACTIVE ) {
		if( sendNow || cls.realtime > 100 + cls.lastPacketSentTime ) {
			// write the command ack
			if( !cls.reliable ) {
				MSG_WriteUint8( &message, clc_svcack );
				MSG_WriteIntBase128( &message, cls.lastExecutedServerCommand );
			}
			//write up the clc commands
			CL_UpdateClientCommandsToServer( &message );
			if( message.cursize > 0 ) {
				CL_Netchan_Transmit( &message );
			}
		}
	} else if( sendNow || CL_MaxPacketsReached() ) {
		// write the command ack
		if( !cls.reliable ) {
			MSG_WriteUint8( &message, clc_svcack );
			MSG_WriteIntBase128( &message, cls.lastExecutedServerCommand );
		}
		// send a userinfo update if needed
		if( userinfo_modified ) {
			userinfo_modified = false;
			CL_AddReliableCommand( va( "usri \"%s\"", Cvar_Userinfo() ) );
		}
		CL_UpdateClientCommandsToServer( &message );
		CL_WriteUcmdsToMessage( &message );
		if( message.cursize > 0 ) {
			CL_Netchan_Transmit( &message );
		}
	}
}

/*
* CL_NetFrame
*/
static void CL_NetFrame( int realMsec, int gameMsec ) {
	// read packets from server
	if( realMsec > 5000 ) { // if in the debugger last frame, don't timeout
		cls.lastPacketReceivedTime = cls.realtime;
	}

	if( cls.demoPlayer.playing ) {
		CL_ReadDemoPackets(); // fetch results from demo file
	}
	CL_ReadPackets(); // fetch results from server

	// send packets to server
	if( cls.netchan.unsentFragments ) {
		Netchan_TransmitNextFragment( &cls.netchan );
	} else {
		CL_SendMessagesToServer( false );
	}

	// resend a connection request if necessary
	CL_CheckForResend();
	CL_CheckDownloadTimeout();

	ServerList::instance()->frame();
}

/*
* CL_Frame
*/
void CL_Frame( int realMsec, int gameMsec ) {
	static int allRealMsec = 0, allGameMsec = 0, extraMsec = 0;
	static float roundingMsec = 0.0f;
	int minMsec;
	float maxFps;

	if( dedicated->integer ) {
		return;
	}

	cls.realtime += realMsec;

	if( cls.demoPlayer.playing && cls.demoPlayer.play_ignore_next_frametime ) {
		gameMsec = 0;
		cls.demoPlayer.play_ignore_next_frametime = false;
	}

	if( cls.demoPlayer.playing ) {
		if( cls.demoPlayer.paused ) {
			gameMsec = 0;
		} else {
			CL_LatchedDemoJump();
		}
	}

	cls.gametime += gameMsec;

	allRealMsec += realMsec;
	allGameMsec += gameMsec;

	CL_UpdateSnapshot();
	CL_AdjustServerTime( gameMsec );
	CL_UserInputFrame( realMsec );
	CL_NetFrame( realMsec, gameMsec );

	CLStatsowFacade::Instance()->Frame();

	if( cls.state == CA_DISCONNECTED ) {
		maxFps = 60;
		minMsec = 1000.0f / maxFps;
		roundingMsec += 1000.0f / maxFps - minMsec;
	} else if( cl_maxfps->integer > 0 && !( cl_timedemo->integer && cls.demoPlayer.playing ) ) {
		const int absMinFps = 24;

		// do not allow setting cl_maxfps to very low values to prevent cheating
		if( cl_maxfps->integer < absMinFps ) {
			Cvar_ForceSet( "cl_maxfps", STR_TOSTR( absMinFps ) );
		}
		maxFps = VID_AppIsMinimized() ? absMinFps : cl_maxfps->value;
		minMsec = (int)wsw::max( ( 1000.0f / maxFps ), 1.0f );
		roundingMsec += wsw::max( ( 1000.0f / maxFps ), 1.0f ) - minMsec;
	} else {
		maxFps = 10000.0f;
		minMsec = 1;
		roundingMsec = 0;
	}

	if( roundingMsec >= 1.0f ) {
		minMsec += (int)roundingMsec;
		roundingMsec -= (int)roundingMsec;
	}

	if( allRealMsec + extraMsec < minMsec ) {
		// let CPU sleep while playing fullscreen video, while minimized
		// or when cl_sleep is enabled
		bool sleep = cl_sleep->integer != 0 || cls.state == CA_DISCONNECTED ||
			!VID_AppIsActive() || VID_AppIsMinimized(); // FIXME: not sure about listen server here..

		if( sleep && minMsec - extraMsec > 1 ) {
			Sys_Sleep( minMsec - extraMsec - 1 );
		}
		return;
	}

	cls.frametime = allGameMsec;
	cls.realFrameTime = allRealMsec;
#if 1
	if( allRealMsec < minMsec ) { // is compensating for a too slow frame
		extraMsec -= ( minMsec - allRealMsec );
		Q_clamp( extraMsec, 0, 100 );
	} else {   // too slow, or exact frame
		extraMsec = allRealMsec - minMsec;
		Q_clamp( extraMsec, 0, 100 );
	}
#else
	extraMsec = allRealMsec - minMsec;
	Q_clamp( extraMsec, 0, minMsec );
#endif

	CL_TimedemoStats();

	// allow rendering DLL change
	VID_CheckChanges();

	// update the screen
	if( host_speeds->integer ) {
		time_before_ref = Sys_Milliseconds();
	}
	SCR_UpdateScreen();
	if( host_speeds->integer ) {
		time_after_ref = Sys_Milliseconds();
	}

	// update audio
	if( cls.state != CA_ACTIVE ) {
		// if the loading plaque is up, clear everything out to make sure we aren't looping a dirty
		// dma buffer while loading
		if( cls.disable_screen ) {
			SoundSystem::instance()->clear();
		} else {
			SoundSystem::instance()->updateListener( vec3_origin, vec3_origin, axis_identity );
		}
	}

	// advance local effects for next frame
	SCR_RunConsole( allRealMsec );

	SoundSystem::instance()->processFrameUpdates();

	allRealMsec = 0;
	allGameMsec = 0;

	cls.framecount++;
}

//============================================================================

/*
* CL_AsyncStream_Alloc
*/
static void *CL_AsyncStream_Alloc( size_t size, const char *filename, int fileline ) {
	return Q_malloc( size );
}

/*
* CL_AsyncStream_Free
*/
static void CL_AsyncStream_Free( void *data, const char *filename, int fileline ) {
	Q_free( data );
}

/*
* CL_InitAsyncStream
*/
static void CL_InitAsyncStream( void ) {
	cl_async_stream = AsyncStream_InitModule( "Client", CL_AsyncStream_Alloc, CL_AsyncStream_Free );
}

/*
* CL_ShutdownAsyncStream
*/
static void CL_ShutdownAsyncStream( void ) {
	if( !cl_async_stream ) {
		return;
	}

	AsyncStream_ShutdownModule( cl_async_stream );
	cl_async_stream = NULL;
}

/*
* CL_AddSessionHttpRequestHeaders
*/
int CL_AddSessionHttpRequestHeaders( const char *url, const char **headers ) {
	static char pH[32];

	if( cls.httpbaseurl && *cls.httpbaseurl ) {
		if( !strncmp( url, cls.httpbaseurl, strlen( cls.httpbaseurl ) ) ) {
			Q_snprintfz( pH, sizeof( pH ), "%i", cl.playernum );

			headers[0] = "X-Client";
			headers[1] = pH;
			headers[2] = "X-Session";
			headers[3] = cls.session;
			return 4;
		}
	}
	return 0;
}

/*
* CL_AsyncStreamRequest
*/
void CL_AsyncStreamRequest( const char *url, const char **headers, int timeout, int resumeFrom,
							size_t ( *read_cb )( const void *, size_t, float, int, const char *, void * ),
							void ( *done_cb )( int, const char *, void * ),
							void ( *header_cb )( const char *, void * ), void *privatep, bool urlencodeUnsafe ) {
	char *tmpUrl = NULL;
	const char *safeUrl;

	if( urlencodeUnsafe ) {
		// urlencode unsafe characters
		size_t allocSize = strlen( url ) * 3 + 1;
		tmpUrl = ( char * )Q_malloc( allocSize );
		AsyncStream_UrlEncodeUnsafeChars( url, tmpUrl, allocSize );

		safeUrl = tmpUrl;
	} else {
		safeUrl = url;
	}

	AsyncStream_PerformRequestExt( cl_async_stream, safeUrl, "GET", NULL, headers, timeout,
								   resumeFrom, read_cb, done_cb, (async_stream_header_cb_t)header_cb, NULL );

	if( urlencodeUnsafe ) {
		Q_free( tmpUrl );
	}
}

//============================================================================

/*
* CL_Init
*/
void CL_Init( void ) {
	netadr_t address;
	cvar_t *cl_port;
	cvar_t *cl_port6;

	assert( !cl_initialized );

	if( dedicated->integer ) {
		return; // nothing running on the client

	}
	cl_initialized = true;

	// all archived variables will now be loaded

	Con_Init();

	CL_Sys_Init();

	Steam_Init();
	// Do this before UI initialization!
	CLStatsowFacade::Init();

	VID_Init();

	CL_ClearState();

	// IPv4
	NET_InitAddress( &address, NA_IP );
	cl_port = Cvar_Get( "cl_port", "0", CVAR_NOSET );
	NET_SetAddressPort( &address, cl_port->integer );
	if( !NET_OpenSocket( &cls.socket_udp, SOCKET_UDP, &address, false ) ) {
		Com_Error( ERR_FATAL, "Couldn't open UDP socket: %s", NET_ErrorString() );
	}

	// IPv6
	NET_InitAddress( &address, NA_IP6 );
	cl_port6 = Cvar_Get( "cl_port6", "0", CVAR_NOSET );
	NET_SetAddressPort( &address, cl_port6->integer );
	if( !NET_OpenSocket( &cls.socket_udp6, SOCKET_UDP, &address, false ) ) {
		Com_Printf( "Error: Couldn't open UDP6 socket: %s", NET_ErrorString() );
	}

	SCR_InitScreen();
	cls.disable_screen = true; // don't draw yet

	CL_InitLocal();
	CL_InitInput();

	CL_InitAsyncStream();
	// Caution! The UI system relies on the server list being in a valid state.
	ServerList::init();

	// Initialize some vars that could be used by the UI prior to the UI loading
	CG_InitPersistentState();

	CL_InitMedia();

	ML_Init();
}

/*
* CL_Shutdown
*
* FIXME: this is a callback from Sys_Quit and Com_Error.  It would be better
* to run quit through here before the final handoff to the sys code.
*/
void CL_Shutdown( void ) {
	if( !cl_initialized ) {
		return;
	}

	SoundSystem::instance()->stopAllSounds( SoundSystem::StopAndClear | SoundSystem::StopMusic );

	ML_Shutdown();

	CLStatsowFacade::Shutdown();

	CL_WriteConfiguration( "config.cfg", true );

	CL_Disconnect( NULL );
	NET_CloseSocket( &cls.socket_udp );
	NET_CloseSocket( &cls.socket_udp6 );
	// TOCHECK: Shouldn't we close the TCP socket too?
	if( cls.servername ) {
		Q_free( cls.servername );
		cls.servername = NULL;
	}

	wsw::ui::UISystem::shutdown();
	CL_GameModule_Shutdown();
	CL_SoundModule_Shutdown( true );
	CL_ShutdownInput();
	VID_Shutdown();

	CL_ShutdownMedia();

	CL_ShutdownAsyncStream();
	ServerList::shutdown();

	CL_ShutdownLocal();

	SCR_ShutdownScreen();

	Steam_Shutdown();

	CL_Sys_Shutdown();

	Con_Shutdown();

	cls.state = CA_UNINITIALIZED;
	cl_initialized = false;
}

class CLCmdSystem : public CmdSystem {
	void registerSystemCommands() override {
		registerCommand( "exec"_asView, handlerOfExec );
		registerCommand( "echo"_asView, handlerOfEcho );
		registerCommand( "alias"_asView, handlerOfAlias );
		registerCommand( "aliasa"_asView, handlerOfAliasa );
		registerCommand( "unalias"_asView, handlerOfUnalias );
		registerCommand( "unaliasall"_asView, handlerOfUnaliasall );
		registerCommand( "wait"_asView, handlerOfWait );
		registerCommand( "vstr"_asView, handlerOfVstr );
	}

	static void handlerOfExec( const CmdArgs & );
	static void handlerOfEcho( const CmdArgs & );
	static void handlerOfAlias( const CmdArgs & );
	static void handlerOfAliasa( const CmdArgs & );
	static void handlerOfUnalias( const CmdArgs & );
	static void handlerOfUnaliasall( const CmdArgs & );
	static void handlerOfWait( const CmdArgs & );
	static void handlerOfVstr( const CmdArgs & );
};

static SingletonHolder<CLCmdSystem> g_clCmdSystemHolder;

void CLCmdSystem::handlerOfExec( const CmdArgs &cmdArgs ) {
	g_clCmdSystemHolder.instance()->helperForHandlerOfExec( cmdArgs );
}

void CLCmdSystem::handlerOfEcho( const CmdArgs &cmdArgs ) {
	g_clCmdSystemHolder.instance()->helperForHandlerOfEcho( cmdArgs );
}

void CLCmdSystem::handlerOfAlias( const CmdArgs &cmdArgs ) {
	g_clCmdSystemHolder.instance()->helperForHandlerOfAlias( false, cmdArgs );
}

void CLCmdSystem::handlerOfAliasa( const CmdArgs &cmdArgs ) {
	g_clCmdSystemHolder.instance()->helperForHandlerOfAlias( true, cmdArgs );
}

void CLCmdSystem::handlerOfUnalias( const CmdArgs &cmdArgs ) {
	g_clCmdSystemHolder.instance()->helperForHandlerOfUnalias( cmdArgs );
}

void CLCmdSystem::handlerOfUnaliasall( const CmdArgs &cmdArgs ) {
	g_clCmdSystemHolder.instance()->helperForHandlerOfUnaliasall( cmdArgs );
}

void CLCmdSystem::handlerOfWait( const CmdArgs &cmdArgs ) {
	g_clCmdSystemHolder.instance()->helperForHandlerOfWait( cmdArgs );
}

void CLCmdSystem::handlerOfVstr( const CmdArgs &cmdArgs ) {
	g_clCmdSystemHolder.instance()->helperForHandlerOfVstr( cmdArgs );
}

void CL_InitCmdSystem() {
	g_clCmdSystemHolder.init();
}

CmdSystem *CL_GetCmdSystem() {
	return g_clCmdSystemHolder.instance();
}

void CL_ShutdownCmdSystem() {
	g_clCmdSystemHolder.shutdown();
}

void CL_Cmd_Register( const wsw::StringView &name, void ( *handler )( const CmdArgs & ) ) {
	g_clCmdSystemHolder.instance()->registerCommand( name, handler );
}

void CL_Cmd_Register( const char *name, void ( *handler )( const CmdArgs & ) ) {
	g_clCmdSystemHolder.instance()->registerCommand( wsw::StringView( name ), handler );
}

void CL_Cmd_Unregister( const char *name ) {
	g_clCmdSystemHolder.instance()->unregisterCommand( wsw::StringView( name ) );
}

bool CL_Cmd_Exists( const wsw::StringView &name ) {
	return g_clCmdSystemHolder.instance()->isARegisteredCommand( name );
}

void CL_Cmd_ExecuteNow( const char *text ) {
	g_clCmdSystemHolder.instance()->executeNow( wsw::StringView( text ) );
}

void CL_Cmd_ExecuteNow( const wsw::StringView &text ) {
	g_clCmdSystemHolder.instance()->executeNow( text );
}

void CL_Cbuf_AppendCommand( const char *text ) {
	g_clCmdSystemHolder.instance()->appendCommand( wsw::StringView( text ) );
}

void CL_Cbuf_AppendCommand( const wsw::StringView &text ) {
	g_clCmdSystemHolder.instance()->appendCommand( text );
}

void CL_Cbuf_PrependCommand( const char *text ) {
	g_clCmdSystemHolder.instance()->prependCommand( wsw::StringView( text ) );
}

void CL_Cbuf_PrependCommand( const wsw::StringView &text ) {
	g_clCmdSystemHolder.instance()->prependCommand( text );
}

void CL_Cbuf_ExecutePendingCommands() {
	g_clCmdSystemHolder.instance()->executeBufferCommands();
}