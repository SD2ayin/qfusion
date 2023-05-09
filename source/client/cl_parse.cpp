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
// cl_parse.c  -- parse a message received from the server

#include "client.h"
#include "../qcommon/cmdargssplitter.h"
#include "../qcommon/cmdcompat.h"
#include "../qcommon/maplist.h"

static void CL_InitServerDownload( const char *filename, size_t size, unsigned checksum, bool allow_localhttpdownload,
								   const char *url, bool initial );
void CL_StopServerDownload( void );

/*
* CL_DownloadRequest
*
* Request file download
* return false if couldn't request it for some reason
* Files with .pk3 or .pak extension have to have gamedir attached
* Other files must not have gamedir
*/
bool CL_DownloadRequest( const char *filename, bool requestpak ) {
	if( cls.download.requestname ) {
		Com_Printf( "Can't download: %s. Download already in progress.\n", filename );
		return false;
	}

	if( !COM_ValidateRelativeFilename( filename ) ) {
		Com_Printf( "Can't download: %s. Invalid filename.\n", filename );
		return false;
	}

	if( FS_CheckPakExtension( filename ) ) {
		if( FS_PakFileExists( filename ) ) {
			Com_Printf( "Can't download: %s. File already exists.\n", filename );
			return false;
		}

		if( !Q_strnicmp( COM_FileBase( filename ), "modules", strlen( "modules" ) ) ) {
			return false;
		}
	} else {
		if( FS_FOpenFile( filename, NULL, FS_READ ) != -1 ) {
			Com_Printf( "Can't download: %s. File already exists.\n", filename );
			return false;
		}

		if( !requestpak ) {
			const char *extension;

			// only allow demo downloads
			extension = COM_FileExtension( filename );
			if( !extension || Q_stricmp( extension, APP_DEMO_EXTENSION_STR ) ) {
				Com_Printf( "Can't download, got arbitrary file type: %s\n", filename );
				return false;
			}
		}
	}

	if( cls.socket->type == SOCKET_LOOPBACK ) {
		Com_DPrintf( "Can't download: %s. Loopback server.\n", filename );
		return false;
	}

	Com_Printf( "Asking to download: %s\n", filename );

	cls.download.requestpak = requestpak;
	cls.download.requestname = (char *)Q_malloc( sizeof( char ) * ( strlen( filename ) + 1 ) );
	Q_strncpyz( cls.download.requestname, filename, sizeof( char ) * ( strlen( filename ) + 1 ) );
	cls.download.timeout = Sys_Milliseconds() + 5000;
	CL_AddReliableCommand( va( "download %i \"%s\"", requestpak, filename ) );

	return true;
}

/*
* CL_CheckOrDownloadFile
*
* Returns true if the file exists or couldn't send download request
* Files with .pk3 or .pak extension have to have gamedir attached
* Other files must not have gamedir
*/
bool CL_CheckOrDownloadFile( const char *filename ) {
	const char *ext;

	if( !cl_downloads->integer ) {
		return true;
	}

	if( !COM_ValidateRelativeFilename( filename ) ) {
		return true;
	}

	ext = COM_FileExtension( filename );
	if( !ext ) {
		return true;
	}

	if( FS_CheckPakExtension( filename ) ) {
		if( FS_PakFileExists( filename ) ) {
			return true;
		}
	} else {
		if( FS_FOpenFile( filename, NULL, FS_READ ) != -1 ) {
			return true;
		}
	}

	if( !CL_DownloadRequest( filename, true ) ) {
		return true;
	}

	cls.download.requestnext = true; // call CL_RequestNextDownload when done

	return false;
}

/*
* CL_DownloadComplete
*
* Checks downloaded file's checksum, renames it and adds to the filesystem.
*/
static void CL_DownloadComplete( void ) {
	unsigned checksum = 0;
	int length;

	FS_FCloseFile( cls.download.filenum );
	cls.download.filenum = 0;

	// verify checksum
	if( FS_CheckPakExtension( cls.download.name ) ) {
		if( !FS_IsPakValid( cls.download.tempname, &checksum ) ) {
			Com_Printf( "Downloaded file is not a valid pack file. Removing\n" );
			FS_RemoveBaseFile( cls.download.tempname );
			return;
		}
	} else {
		length = FS_LoadBaseFile( cls.download.tempname, NULL, NULL, 0 );
		if( length < 0 ) {
			Com_Printf( "Error: Couldn't load downloaded file\n" );
			return;
		}
		checksum = FS_ChecksumBaseFile( cls.download.tempname, false );
	}

	if( cls.download.checksum != checksum ) {
		Com_Printf( "Downloaded file has wrong checksum. Removing: %u %u %s\n", cls.download.checksum, checksum, cls.download.tempname );
		FS_RemoveBaseFile( cls.download.tempname );
		return;
	}

	if( !FS_MoveBaseFile( cls.download.tempname, cls.download.name ) ) {
		Com_Printf( "Failed to rename the downloaded file\n" );
		return;
	}

	// Maplist hook so we also know when a new map is added
	if( FS_CheckPakExtension( cls.download.name ) ) {
		ML_Update();
	}

	cls.download.successCount++;
	cls.download.timeout = 0;
}

/*
* CL_FreeDownloadList
*/
void CL_FreeDownloadList( void ) {
	download_list_t *next;

	while( cls.download.list ) {
		next = cls.download.list->next;
		Q_free( cls.download.list->filename );
		Q_free( cls.download.list );
		cls.download.list = next;
	}
}

/*
* CL_DownloadDone
*/
void CL_DownloadDone( void ) {
	bool requestnext;

	if( cls.download.name ) {
		CL_StopServerDownload();
	}

	Q_free( cls.download.requestname );
	cls.download.requestname = NULL;

	requestnext = cls.download.requestnext;
	cls.download.requestnext = false;
	cls.download.requestpak = false;
	cls.download.timeout = 0;
	cls.download.timestart = 0;
	cls.download.offset = cls.download.baseoffset = 0;
	cls.download.web = false;
	cls.download.filenum = 0;
	cls.download.cancelled = false;

	// the server has changed map during the download
	if( cls.download.pending_reconnect ) {
		cls.download.pending_reconnect = false;
		CL_FreeDownloadList();
		CL_ServerReconnect_f( {} );
		return;
	}

	if( requestnext && cls.state > CA_DISCONNECTED ) {
		CL_RequestNextDownload();
	}
}

/*
* CL_WebDownloadDoneCb
*/
static void CL_WebDownloadDoneCb( int status, const char *contentType, void *privatep ) {
	download_t download = cls.download;
	bool disconnect = download.disconnect;
	bool cancelled = download.cancelled;
	bool success = ( download.offset == download.size ) && ( status > -1 );
	bool try_non_official = download.web_official && !download.web_official_only;

	Com_Printf( "Web download %s: %s (%i)\n", success ? "successful" : "failed", download.tempname, status );

	if( success ) {
		CL_DownloadComplete();
	}
	if( cancelled ) {
		cls.download.requestnext = false;
	}

	// check if user pressed escape to stop the downloa
	if( disconnect ) {
		CL_Disconnect( NULL ); // this also calls CL_DownloadDone()
		return;
	}

	// try a non-official mirror (the builtin HTTP server or a remote mirror)
	if( !success && !cancelled && try_non_official ) {
		int size = download.size;
		char *filename = Q_strdup( download.origname );
		unsigned checksum = download.checksum;
		char *url = Q_strdup( download.web_url );
		bool allow_localhttp = download.web_local_http;

		cls.download.cancelled = true; // remove the temp file
		CL_StopServerDownload();
		CL_InitServerDownload( filename, size, checksum, allow_localhttp, url, false );

		Q_free( filename );
		Q_free( url );
		return;
	}

	CL_DownloadDone();
}

/*
* CL_WebDownloadReadCb
*/
static size_t CL_WebDownloadReadCb( const void *buf, size_t numb, float percentage, int status,
									const char *contentType, void *privatep ) {
	bool stop = cls.download.disconnect || cls.download.cancelled || status < 0 || status >= 300;
	size_t write = 0;

	if( !stop ) {
		write = FS_Write( buf, numb, cls.download.filenum );
	}

	// ignore percentage passed by the downloader as it doesn't account for total file size
	// of resumed downloads
	cls.download.offset += write;
	cls.download.percent = (double)cls.download.offset / (double)cls.download.size;
	Q_clamp( cls.download.percent, 0, 1 );

	Cvar_ForceSet( "cl_download_percent", va( "%.1f", cls.download.percent * 100 ) );

	cls.download.timeout = 0;

	// abort if disconnected, canclled or writing failed
	return stop ? !numb : write;
}

/*
* CL_InitDownload
*
* Hanldles server's initdownload message, starts web or server download if possible
*/
static void CL_InitServerDownload( const char *filename, size_t size, unsigned checksum, bool allow_localhttpdownload,
								   const char *url, bool initial ) {
	size_t alloc_size;
	bool modules_download = false;
	bool explicit_pure_download = false;
	bool force_web_official = initial && cls.download.requestpak;
	bool official_web_download = false;
	bool official_web_only = false;
	const char *baseurl;
	download_list_t *dl;

	// ignore download commands coming from demo files
	if( cls.demoPlayer.playing ) {
		return;
	}

	if( !cls.download.requestname ) {
		Com_Printf( "Got init download message without request\n" );
		return;
	}

	if( cls.download.filenum || cls.download.web ) {
		Com_Printf( "Got init download message while already downloading\n" );
		return;
	}

	if( size == (size_t)-1 ) {
		// means that download was refused
		Com_Printf( "Server refused download request: %s\n", url ); // if it's refused, url field holds the reason
		CL_DownloadDone();
		return;
	}

	if( size <= 0 ) {
		Com_Printf( "Server gave invalid size, not downloading\n" );
		CL_DownloadDone();
		return;
	}

	if( checksum == 0 ) {
		Com_Printf( "Server didn't provide checksum, not downloading\n" );
		CL_DownloadDone();
		return;
	}

	if( !COM_ValidateRelativeFilename( filename ) ) {
		Com_Printf( "Not downloading, invalid filename: %s\n", filename );
		CL_DownloadDone();
		return;
	}

	if( FS_CheckPakExtension( filename ) != cls.download.requestpak ) {
		const char *requested = cls.download.requestpak ? "pak" : "normal";
		const char *got = cls.download.requestpak ? "normal" : "pak";
		Com_Printf( "Got a %s file when requesting a %s file, not downloading\n", got, requested );
		CL_DownloadDone();
		return;
	}

	if( !strchr( filename, '/' ) ) {
		Com_Printf( "Refusing to download file with no gamedir: %s\n", filename );
		CL_DownloadDone();
		return;
	}

	// check that it is in game or basegame dir
	const wsw::StringView fileNameView( filename );
	if( !fileNameView.startsWith( kDataDirectory ) ||
	    fileNameView.maybeAt( kDataDirectory.size() ) != std::optional( '/' ) ) {
		Com_Printf( "Can't download, invalid game directory: %s\n", filename );
		CL_DownloadDone();
		return;
	}

	if( FS_CheckPakExtension( filename ) ) {
		if( strchr( strchr( filename, '/' ) + 1, '/' ) ) {
			Com_Printf( "Refusing to download pack file to subdirectory: %s\n", filename );
			CL_DownloadDone();
			return;
		}

		modules_download = !Q_strnicmp( COM_FileBase( filename ), "modules", strlen( "modules" ) );

		if( modules_download ) {
			CL_DownloadDone();
			return;
		}

		if( FS_PakFileExists( filename ) ) {
			Com_Printf( "Can't download, file already exists: %s\n", filename );
			CL_DownloadDone();
			return;
		}

		explicit_pure_download = FS_IsExplicitPurePak( filename, NULL );
	} else {
		if( strcmp( cls.download.requestname, strchr( filename, '/' ) + 1 ) ) {
			Com_Printf( "Can't download, got different file than requested: %s\n", filename );
			CL_DownloadDone();
			return;
		}
	}

	if( initial ) {
		if( cls.download.requestnext ) {
			dl = cls.download.list;
			while( dl != NULL ) {
				if( !Q_stricmp( dl->filename, filename ) ) {
					Com_Printf( "Skipping, already tried downloading: %s\n", filename );
					CL_DownloadDone();
					return;
				}
				dl = dl->next;
			}
		}
	}

	official_web_only = modules_download || explicit_pure_download;
	official_web_download = force_web_official || official_web_only;

	alloc_size = strlen( "downloads" ) + 1 /* '/' */ + strlen( filename ) + 1;
	cls.download.name = (char *)Q_malloc( alloc_size );
	if( official_web_download || !cls.download.requestpak ) {
		// it's an official pak, otherwise
		// if we're not downloading a pak, this must be a demo so drop it into the gamedir
		Q_snprintfz( cls.download.name, alloc_size, "%s", filename );
	} else {
		if( FS_DownloadsDirectory() == NULL ) {
			Com_Printf( "Can't download, downloads directory is disabled\n" );
			CL_DownloadDone();
			return;
		}
		Q_snprintfz( cls.download.name, alloc_size, "%s/%s", "downloads", filename );
	}

	alloc_size = strlen( cls.download.name ) + strlen( ".tmp" ) + 1;
	cls.download.tempname = (char *)Q_malloc( alloc_size );
	Q_snprintfz( cls.download.tempname, alloc_size, "%s.tmp", cls.download.name );

	cls.download.origname = Q_strdup( filename );
	cls.download.web = false;
	cls.download.web_official = official_web_download;
	cls.download.web_official_only = official_web_only;
	cls.download.web_url = Q_strdup( url );
	cls.download.web_local_http = allow_localhttpdownload;
	cls.download.cancelled = false;
	cls.download.disconnect = false;
	cls.download.size = size;
	cls.download.checksum = checksum;
	cls.download.percent = 0;
	cls.download.timeout = 0;
	cls.download.retries = 0;
	cls.download.timestart = Sys_Milliseconds();
	cls.download.offset = 0;
	cls.download.baseoffset = 0;
	cls.download.pending_reconnect = false;

	Cvar_ForceSet( "cl_download_name", COM_FileBase( filename ) );
	Cvar_ForceSet( "cl_download_percent", "0" );

	if( initial ) {
		if( cls.download.requestnext ) {
			dl = (download_list_t *)Q_malloc( sizeof( download_list_t ) );
			dl->filename = Q_strdup( filename );
			dl->next = cls.download.list;
			cls.download.list = dl;
		}
	}

	baseurl = cls.httpbaseurl;
	if( official_web_download ) {
		baseurl = APP_UPDATE_URL APP_SERVER_UPDATE_DIRECTORY;
		allow_localhttpdownload = false;
	}

	if( official_web_download ) {
		cls.download.web = true;
		Com_Printf( "Web download: %s from %s/%s\n", cls.download.tempname, baseurl, filename );
	} else if( cl_downloads_from_web->integer && allow_localhttpdownload && url && url[0] != 0 ) {
		cls.download.web = true;
		Com_Printf( "Web download: %s from %s/%s\n", cls.download.tempname, baseurl, url );
	} else if( cl_downloads_from_web->integer && url && url[0] != 0 ) {
		cls.download.web = true;
		Com_Printf( "Web download: %s from %s\n", cls.download.tempname, url );
	} else {
		Com_Printf( "Server download: %s\n", cls.download.tempname );
	}

	cls.download.baseoffset = cls.download.offset =
		(size_t)FS_FOpenBaseFile( cls.download.tempname, &cls.download.filenum, FS_APPEND );

	if( !cls.download.filenum ) {
		Com_Printf( "Can't download, couldn't open %s for writing\n", cls.download.tempname );
		CL_DownloadDone();
		return;
	}

	if( cls.download.web ) {
		char *referer, *fullurl;
		const char *headers[] = { NULL, NULL, NULL, NULL, NULL, NULL, NULL };

		if( cls.download.offset == cls.download.size ) {
			// special case for completed downloads to avoid passing empty HTTP range
			CL_WebDownloadDoneCb( 200, "", NULL );
			return;
		}

		alloc_size = strlen( APP_URI_SCHEME ) + strlen( NET_AddressToString( &cls.serveraddress ) ) + 1;
		referer = (char *)alloca( alloc_size );
		Q_snprintfz( referer, alloc_size, APP_URI_SCHEME "%s", NET_AddressToString( &cls.serveraddress ) );
		Q_strlwr( referer );

		if( official_web_download ) {
			alloc_size = strlen( baseurl ) + 1 + strlen( filename ) + 1;
			fullurl = (char *)alloca( alloc_size );
			Q_snprintfz( fullurl, alloc_size, "%s/%s", baseurl, filename );
		} else if( allow_localhttpdownload ) {
			alloc_size = strlen( baseurl ) + 1 + strlen( url ) + 1;
			fullurl = (char *)alloca( alloc_size );
			Q_snprintfz( fullurl, alloc_size, "%s/%s", baseurl, url );
		} else {
			size_t url_len = strlen( url );
			alloc_size = url_len + 1 + strlen( filename ) * 3 + 1;
			fullurl = (char *)alloca( alloc_size );
			Q_snprintfz( fullurl, alloc_size, "%s/", url );
			Q_urlencode_unsafechars( filename, fullurl + url_len + 1, alloc_size - url_len - 1 );
		}

		headers[0] = "Referer";
		headers[1] = referer;

		CL_AddSessionHttpRequestHeaders( fullurl, &headers[2] );

		CL_AsyncStreamRequest( fullurl, headers, cl_downloads_from_web_timeout->integer / 100, (int)cls.download.offset,
							   CL_WebDownloadReadCb, CL_WebDownloadDoneCb, NULL, NULL, false );

		return;
	}

	cls.download.timeout = Sys_Milliseconds() + 3000;
	cls.download.retries = 0;

	CL_AddReliableCommand( va( "nextdl \"%s\" %" PRIu64, cls.download.name, (uint64_t)cls.download.offset ) );
}

/*
* CL_InitDownload_f
*/
static void CL_InitDownload_f( const CmdArgs &cmdArgs ) {
	const char *filename;
	const char *url;
	int size;
	unsigned checksum;
	bool allow_localhttpdownload;

	// ignore download commands coming from demo files
	if( cls.demoPlayer.playing ) {
		return;
	}

	// read the data
	filename = Cmd_Argv( 1 );
	size = atoi( Cmd_Argv( 2 ) );
	checksum = (unsigned)strtoul( Cmd_Argv( 3 ), NULL, 10 );
	allow_localhttpdownload = ( atoi( Cmd_Argv( 4 ) ) != 0 ) && cls.httpbaseurl != NULL;
	url = Cmd_Argv( 5 );

	CL_InitServerDownload( filename, size, (unsigned)checksum, allow_localhttpdownload, url, true );
}

/*
* CL_StopServerDownload
*/
void CL_StopServerDownload( void ) {
	if( cls.download.filenum > 0 ) {
		FS_FCloseFile( cls.download.filenum );
		cls.download.filenum = 0;
	}

	if( cls.download.cancelled ) {
		FS_RemoveBaseFile( cls.download.tempname );
	}

	Q_free( cls.download.name );
	cls.download.name = NULL;

	Q_free( cls.download.tempname );
	cls.download.tempname = NULL;

	Q_free( cls.download.origname );
	cls.download.origname = NULL;

	Q_free( cls.download.web_url );
	cls.download.web_url = NULL;

	cls.download.offset = 0;
	cls.download.size = 0;
	cls.download.percent = 0;
	cls.download.timeout = 0;
	cls.download.retries = 0;
	cls.download.web = false;

	Cvar_ForceSet( "cl_download_name", "" );
	Cvar_ForceSet( "cl_download_percent", "0" );
}

/*
* CL_RetryDownload
* Resends download request
* Also aborts download if we have retried too many times
*/
static void CL_RetryDownload( void ) {
	if( ++cls.download.retries > 5 ) {
		Com_Printf( "Download timed out: %s\n", cls.download.name );

		// let the server know we're done
		CL_AddReliableCommand( va( "nextdl \"%s\" %i", cls.download.name, -2 ) );
		CL_DownloadDone();
	} else {
		cls.download.timeout = Sys_Milliseconds() + 3000;
		CL_AddReliableCommand( va( "nextdl \"%s\" %" PRIu64, cls.download.name, (uint64_t)cls.download.offset ) );
	}
}

/*
* CL_CheckDownloadTimeout
* Retry downloading if too much time has passed since last download packet was received
*/
void CL_CheckDownloadTimeout( void ) {
	if( !cls.download.timeout || cls.download.timeout > Sys_Milliseconds() ) {
		return;
	}

	if( cls.download.filenum ) {
		CL_RetryDownload();
	} else {
		Com_Printf( "Download request timed out.\n" );
		CL_DownloadDone();
	}
}

/*
* CL_DownloadStatus_f
*/
void CL_DownloadStatus_f( const CmdArgs & ) {
	if( !cls.download.requestname ) {
		Com_Printf( "No download active\n" );
		return;
	}

	if( !cls.download.name ) {
		Com_Printf( "%s: Requesting\n", COM_FileBase( cls.download.requestname ) );
		return;
	}

	Com_Printf( "%s: %s download %3.2f%c done\n", COM_FileBase( cls.download.name ),
				( cls.download.web ? "Web" : "Server" ), cls.download.percent * 100.0f, '%' );
}

/*
* CL_DownloadCancel_f
*/
void CL_DownloadCancel_f( const CmdArgs & ) {
	if( !cls.download.requestname ) {
		Com_Printf( "No download active\n" );
		return;
	}

	if( !cls.download.name ) {
		CL_DownloadDone();
		Com_Printf( "Canceled download request\n" );
		return;
	}

	Com_Printf( "Canceled download of %s\n", cls.download.name );

	cls.download.cancelled = true;

	if( !cls.download.web ) {
		CL_AddReliableCommand( va( "nextdl \"%s\" %i", cls.download.name, -2 ) ); // let the server know we're done
		CL_DownloadDone();
	}
}

/*
* CL_ParseDownload
* Handles download message from the server.
* Writes data to the file and requests next download block.
*/
static void CL_ParseDownload( msg_t *msg ) {
	size_t size, offset;
	char *svFilename;

	// read the data
	svFilename = MSG_ReadString( msg );
	offset = MSG_ReadInt32( msg );
	size = MSG_ReadInt32( msg );

	if( cls.demoPlayer.playing ) {
		// ignore download commands coming from demo files
		return;
	}

	if( msg->readcount + size > msg->cursize ) {
		Com_Printf( "Error: Download message didn't have as much data as it promised\n" );
		CL_RetryDownload();
		return;
	}

	if( !cls.download.filenum ) {
		Com_Printf( "Error: Download message while not dowloading\n" );
		msg->readcount += size;
		return;
	}

	if( Q_stricmp( cls.download.name, svFilename ) ) {
		Com_Printf( "Error: Download message for wrong file\n" );
		msg->readcount += size;
		return;
	}

	if( offset + size > cls.download.size ) {
		Com_Printf( "Error: Invalid download message\n" );
		msg->readcount += size;
		CL_RetryDownload();
		return;
	}

	if( cls.download.offset != offset ) {
		Com_Printf( "Error: Download message for wrong position\n" );
		msg->readcount += size;
		CL_RetryDownload();
		return;
	}

	FS_Write( msg->data + msg->readcount, size, cls.download.filenum );
	msg->readcount += size;
	cls.download.offset += size;
	cls.download.percent = (double)cls.download.offset / (double)cls.download.size;
	Q_clamp( cls.download.percent, 0, 1 );

	Cvar_ForceSet( "cl_download_percent", va( "%.1f", cls.download.percent * 100 ) );

	if( cls.download.offset < cls.download.size ) {
		cls.download.timeout = Sys_Milliseconds() + 3000;
		cls.download.retries = 0;

		CL_AddReliableCommand( va( "nextdl \"%s\" %" PRIu64, cls.download.name, (uint64_t)cls.download.offset ) );
	} else {
		Com_Printf( "Download complete: %s\n", cls.download.name );

		CL_DownloadComplete();

		// let the server know we're done
		CL_AddReliableCommand( va( "nextdl \"%s\" %i", cls.download.name, -1 ) );

		CL_DownloadDone();
	}
}

/*
=====================================================================

SERVER CONNECTING MESSAGES

=====================================================================
*/

/*
* CL_ParseServerData
*/
static void CL_ParseServerData( msg_t *msg ) {
	int i, sv_bitflags, numpure;
	int http_portnum;
	bool old_sv_pure;

	Com_DPrintf( "Serverdata packet received.\n" );

	// wipe the client_state_t struct

	CL_ClearState();
	CL_SetClientState( CA_CONNECTED );

	// parse protocol version number
	i = MSG_ReadInt32( msg );

	if( i != APP_PROTOCOL_VERSION && !( cls.demoPlayer.playing && i == APP_DEMO_PROTOCOL_VERSION ) ) {
		Com_Error( ERR_DROP, "Server returned version %i, not %i", i, APP_PROTOCOL_VERSION );
	}

	cl.servercount = MSG_ReadInt32( msg );
	cl.snapFrameTime = (unsigned int)MSG_ReadInt16( msg );
	cl.gamestart = true;

	// set extrapolation time to half snapshot time
	Cvar_ForceSet( "cl_extrapolationTime", va( "%i", (unsigned int)( cl.snapFrameTime * 0.5 ) ) );
	cl_extrapolationTime->modified = false;

	// parse player entity number
	cl.playernum = MSG_ReadInt16( msg );

	// get the full level name
	Q_strncpyz( cl.servermessage, MSG_ReadString( msg ), sizeof( cl.servermessage ) );

	sv_bitflags = MSG_ReadUint8( msg );

	if( cls.demoPlayer.playing ) {
		cls.reliable = ( sv_bitflags & SV_BITFLAGS_RELIABLE );
	} else {
		if( cls.reliable != ( ( sv_bitflags & SV_BITFLAGS_RELIABLE ) != 0 ) ) {
			Com_Error( ERR_DROP, "Server and client disagree about connection reliability" );
		}
	}

	// builting HTTP server port
	if( cls.httpbaseurl ) {
		Q_free( cls.httpbaseurl );
		cls.httpbaseurl = NULL;
	}

	if( ( sv_bitflags & SV_BITFLAGS_HTTP ) != 0 ) {
		if( ( sv_bitflags & SV_BITFLAGS_HTTP_BASEURL ) != 0 ) {
			// read base upstream url
			cls.httpbaseurl = Q_strdup( MSG_ReadString( msg ) );
		} else {
			http_portnum = MSG_ReadInt16( msg ) & 0xffff;
			cls.httpaddress = cls.serveraddress;
			if( cls.httpaddress.type == NA_IP6 ) {
				cls.httpaddress.address.ipv6.port = BigShort( http_portnum );
			} else {
				cls.httpaddress.address.ipv4.port = BigShort( http_portnum );
			}
			if( http_portnum ) {
				if( cls.httpaddress.type == NA_LOOPBACK ) {
					cls.httpbaseurl = Q_strdup( va( "http://localhost:%hu/", (unsigned short)http_portnum ) );
				} else {
					cls.httpbaseurl = Q_strdup( va( "http://%s/", NET_AddressToString( &cls.httpaddress ) ) );
				}
			}
		}
	}

	// pure list

	// clean old, if necessary
	Com_FreePureList( &cls.purelist );

	// add new
	numpure = MSG_ReadInt16( msg );
	while( numpure > 0 ) {
		const char *pakname = MSG_ReadString( msg );
		const unsigned checksum = MSG_ReadInt32( msg );

		Com_AddPakToPureList( &cls.purelist, pakname, checksum );

		numpure--;
	}

	//assert( numpure == 0 );

	// get the configstrings request
	CL_AddReliableCommand( va( "configstrings %i 0", cl.servercount ) );

	old_sv_pure = cls.sv_pure;
	cls.sv_pure = ( sv_bitflags & SV_BITFLAGS_PURE ) != 0;
	cls.pure_restart = cls.sv_pure && old_sv_pure == false;

#ifdef PURE_CHEAT
	cls.sv_pure = cls.pure_restart = false;
#endif

	cls.wakelock = Sys_AcquireWakeLock();

	if( !cls.demoPlayer.playing && ( cls.serveraddress.type == NA_IP ) ) {
		Steam_AdvertiseGame( cls.serveraddress.address.ipv4.ip, NET_GetAddressPort( &cls.serveraddress ) );
	}

	// separate the printfs so the server message can have a color
	Com_Printf( S_COLOR_WHITE "\n" "=====================================\n" );
	Com_Printf( S_COLOR_WHITE "%s\n\n", cl.servermessage );
}

/*
* CL_ParseBaseline
*/
static void CL_ParseBaseline( msg_t *msg ) {
	SNAP_ParseBaseline( msg, cl_baselines );
}

/*
* CL_ParseFrame
*/
static void CL_ParseFrame( msg_t *msg ) {
	snapshot_t *snap, *oldSnap;
	int delta;

	oldSnap = ( cl.receivedSnapNum > 0 ) ? &cl.snapShots[cl.receivedSnapNum & UPDATE_MASK] : NULL;

	snap = SNAP_ParseFrame( msg, oldSnap, &cl.suppressCount, cl.snapShots, cl_baselines, cl_shownet->integer );
	if( snap->valid ) {
		cl.receivedSnapNum = snap->serverFrame;

		if( cls.demoRecorder.recording ) {
			if( cls.demoRecorder.waiting && !snap->delta ) {
				cls.demoRecorder.waiting = false; // we can start recording now
				cls.demoRecorder.basetime = snap->serverTime;
				cls.demoRecorder.localtime = time( NULL );

				// write out messages to hold the startup information
				SNAP_BeginDemoRecording( cls.demoRecorder.file, 0x10000 + cl.servercount, cl.snapFrameTime,
										 cl.servermessage, cls.reliable ? SV_BITFLAGS_RELIABLE : 0, cls.purelist,
										 cl.configStrings, cl_baselines );

				// the rest of the demo file will be individual frames
			}

			if( !cls.demoRecorder.waiting ) {
				cls.demoRecorder.duration = snap->serverTime - cls.demoRecorder.basetime;
			}
			cls.demoRecorder.time = cls.demoRecorder.duration;
		}

		if( cl_debug_timeDelta->integer ) {
			if( oldSnap != NULL && ( oldSnap->serverFrame + 1 != snap->serverFrame ) ) {
				Com_Printf( S_COLOR_RED "***** SnapShot lost\n" );
			}
		}

		// the first snap, fill all the timeDeltas with the same value
		// don't let delta add big jumps to the smoothing ( a stable connection produces jumps inside +-3 range)
		delta = ( snap->serverTime - cl.snapFrameTime ) - cls.gametime;
		if( cl.currentSnapNum <= 0 || delta < cl.newServerTimeDelta - 175 || delta > cl.newServerTimeDelta + 175 ) {
			CL_RestartTimeDeltas( delta );
		} else {
			if( cl_debug_timeDelta->integer ) {
				if( delta < cl.newServerTimeDelta - (int)cl.snapFrameTime ) {
					Com_Printf( S_COLOR_CYAN "***** timeDelta low clamp\n" );
				} else if( delta > cl.newServerTimeDelta + (int)cl.snapFrameTime ) {
					Com_Printf( S_COLOR_CYAN "***** timeDelta high clamp\n" );
				}
			}

			Q_clamp( delta, cl.newServerTimeDelta - (int)cl.snapFrameTime, cl.newServerTimeDelta + (int)cl.snapFrameTime );

			cl.serverTimeDeltas[cl.receivedSnapNum & MASK_TIMEDELTAS_BACKUP] = delta;
		}
	}
}

//========= StringCommands================

/*
* CL_Multiview_f
*/
static void CL_Multiview_f( const CmdArgs &cmdArgs ) {
	cls.mv = ( atoi( Cmd_Argv( 1 ) ) != 0 );
	Com_Printf( "multiview: %i\n", cls.mv );
}

/*
* CL_CvarInfoRequest_f
*/
static void CL_CvarInfoRequest_f( const CmdArgs &cmdArgs ) {
	char string[MAX_STRING_CHARS];
	const char *cvarName;
	const char *cvarString;

	if( cls.demoPlayer.playing ) {
		return;
	}

	if( Cmd_Argc() < 1 ) {
		return;
	}

	cvarName = Cmd_Argv( 1 );

	string[0] = 0;
	Q_strncatz( string, "cvarinfo \"", sizeof( string ) );

	if( strlen( string ) + strlen( cvarName ) + 1 /*quote*/ + 1 /*space*/ >= MAX_STRING_CHARS - 1 ) {
		CL_AddReliableCommand( "cvarinfo \"invalid\"" );
		return;
	}

	Q_strncatz( string, cvarName, sizeof( string ) );
	Q_strncatz( string, "\" ", sizeof( string ) );

	cvarString = Cvar_String( cvarName );
	if( !cvarString[0] ) {
		cvarString = "not found";
	}

	if( strlen( string ) + strlen( cvarString ) + 2 /*quotes*/ >= MAX_STRING_CHARS - 1 ) {
		if( strlen( string ) + strlen( " \"too long\"" ) < MAX_STRING_CHARS - 1 ) {
			CL_AddReliableCommand( va( "%s\"too long\"", string ) );
		} else {
			CL_AddReliableCommand( "cvarinfo \"invalid\"" );
		}

		return;
	}

	Q_strncatz( string, "\"", sizeof( string ) );
	Q_strncatz( string, cvarString, sizeof( string ) );
	Q_strncatz( string, "\"", sizeof( string ) );

	CL_AddReliableCommand( string );
}

/*
* CL_UpdateConfigString
*/
static void CL_UpdateConfigString( int idx, const char *s ) {
	if( !s ) {
		return;
	}

	if( cl_debug_serverCmd->integer && ( cls.state >= CA_ACTIVE || cls.demoPlayer.playing ) ) {
		Com_Printf( "CL_ParseConfigstringCommand(%i): \"%s\"\n", idx, s );
	}

	if( idx < 0 || idx >= MAX_CONFIGSTRINGS ) {
		Com_Error( ERR_DROP, "configstring > MAX_CONFIGSTRINGS" );
	}

	if( !COM_ValidateConfigstring( s ) ) {
		Com_Printf( "%sWARNING:%s Invalid Configstring (%i): %s\n", S_COLOR_YELLOW, S_COLOR_WHITE, idx, s );
		return;
	}

	const wsw::StringView string( s );
	cl.configStrings.set( idx, string );
	CL_GameModule_ConfigString( idx, string );
}

static void CL_AddConfigStringFragment( int index, int fragmentNum, int numFragments, int specifiedLen, const char *s ) {
	if( (unsigned)index >= (unsigned)MAX_CONFIGSTRINGS ) {
		Com_Error( ERR_DROP, "A configstring fragment index > MAX_CONFIGSTRINGS" );
	}

	if( (unsigned)numFragments >= kMaxConfigStringFragments ) {
		Com_Error( ERR_DROP, "A configstring fragment numFragments >= kMaxConfigStringFragments" );
	}

	if( (unsigned)fragmentNum >= (unsigned)numFragments ) {
		Com_Error( ERR_DROP, "configstring fragment fragmentNum is out of a valid range" );
	}

	// TODO: We should accept a string view parameter

	const size_t actualLen = std::strlen( s );
	if( (size_t)specifiedLen != actualLen ) {
		Com_Error( ERR_DROP, "A configstring fragment len mismatches the specified one" );
	}

	if( cl.configStringFragmentIndex && cl.configStringFragmentIndex != index ) {
		Com_Error( ERR_DROP, "Got a configstring fragment while the current one is incompletely assembled" );
	}

	if( cl.configStringFragmentNum && cl.configStringFragmentNum + 1 != fragmentNum ) {
		Com_Error( ERR_DROP, "Got an illegal configstring fragments sequence" );
	}

	const size_t fragmentOffset = fragmentNum * kMaxConfigStringFragmentLen;
	std::memcpy( cl.configStringFragmentsBuffer + fragmentOffset, s, specifiedLen );

	if( fragmentNum + 1 != numFragments ) {
		cl.configStringFragmentIndex = index;
		cl.configStringFragmentNum = fragmentNum;
		return;
	}

	cl.configStringFragmentIndex = 0;
	cl.configStringFragmentNum = 0;

	const size_t combinedLen = fragmentOffset + specifiedLen;
	cl.configStringFragmentsBuffer[combinedLen] = '\0';

	const wsw::StringView view( cl.configStringFragmentsBuffer, combinedLen, wsw::StringView::ZeroTerminated );

	cl.configStrings.set( index, view );
	CL_GameModule_ConfigString( index, view );
}

static void CL_ParseConfigstringCommand( const CmdArgs &cmdArgs ) {
	const int argc = Cmd_Argc();
	if( argc < 3 ) {
		return;
	}

	// ch : configstrings may come batched now, so lets loop through them
	for( int i = 1; i < argc - 1; i += 2 ) {
		const int idx = atoi( Cmd_Argv( i ) );
		const char *s = Cmd_Argv( i + 1 );

		CL_UpdateConfigString( idx, s );
	}
}

static void CL_ParseConfigStringFragmentCommand( const CmdArgs &cmdArgs ) {
	const int argc = Cmd_Argc();
	if( argc != 6 ) {
		return;
	}

	const int index = atoi( Cmd_Argv( 1 ) );
	const int fragmentNum = atoi( Cmd_Argv( 2 ) );
	const int numFragments = atoi( Cmd_Argv( 3 ) );
	const int specifiedLen = atoi( Cmd_Argv( 4 ) );

	CL_AddConfigStringFragment( index, fragmentNum, numFragments, specifiedLen, Cmd_Argv( 5 ) );
}

typedef struct {
	const char *name;
	void ( *func )( const CmdArgs & );
} svcmd_t;

svcmd_t svcmds[] =
{
	{ "forcereconnect", CL_Reconnect_f },
	{ "reconnect", CL_ServerReconnect_f },
	{ "changing", CL_Changing_f },
	{ "precache", CL_Precache_f },
	{ "cmd", CL_ForwardToServer_f },
	{ "cs", CL_ParseConfigstringCommand },
	{ "csf", CL_ParseConfigStringFragmentCommand },
	{ "disconnect", CL_ServerDisconnect_f },
	{ "initdownload", CL_InitDownload_f },
	{ "multiview", CL_Multiview_f },
	{ "cvarinfo", CL_CvarInfoRequest_f },

	{ NULL, NULL }
};


/*
* CL_ParseServerCommand
*/
static void CL_ParseServerCommand( msg_t *msg ) {
	char *text;
	svcmd_t *cmd;

	text = MSG_ReadString( msg );

	static CmdArgsSplitter argsSplitter;
	const CmdArgs &cmdArgs = argsSplitter.exec( wsw::StringView( text ) );

	if( cl_debug_serverCmd->integer && ( cls.state < CA_ACTIVE || cls.demoPlayer.playing ) ) {
		Com_Printf( "CL_ParseServerCommand: \"%s\"\n", text );
	}

	// filter out these server commands to be called from the client
	for( cmd = svcmds; cmd->name; cmd++ ) {
		if( !strcmp( cmdArgs[0].data(), cmd->name ) ) {
			cmd->func( cmdArgs );
			return;
		}
	}

	Com_Printf( "Unknown server command: %s\n", cmdArgs[0].data() );
}

/*
=====================================================================

ACTION MESSAGES

=====================================================================
*/

/*
* CL_ParseServerMessage
*/
void CL_ParseServerMessage( msg_t *msg ) {
	if( cl_shownet->integer == 1 ) {
		Com_Printf( "%" PRIu64 " ", (uint64_t)msg->cursize );
	} else if( cl_shownet->integer >= 2 ) {
		Com_Printf( "------------------\n" );
	}

	// parse the message
	while( msg->readcount < msg->cursize ) {
		int cmd;
		int ext, len;
		size_t meta_data_maxsize;

		cmd = MSG_ReadUint8( msg );
		if( cl_debug_serverCmd->integer & 4 ) {
			const char *format = "%3" PRIi64 ":CMD %i %s\n";
			Com_Printf( format, (int64_t)( msg->readcount - 1 ), cmd, !svc_strings[cmd] ? "bad" : svc_strings[cmd] );
		}

		if( cl_shownet->integer >= 2 ) {
			if( !svc_strings[cmd] ) {
				const char *format = "%3" PRIi64 ":BAD CMD %i\n";
				Com_Printf( format, (int64_t)( msg->readcount - 1 ), cmd );
			} else {
				SHOWNET( msg, svc_strings[cmd] );
			}
		}

		// other commands
		switch( cmd ) {
			default:
				Com_Error( ERR_DROP, "CL_ParseServerMessage: Illegible server message" );
				break;

			case svc_nop:
				// Com_Printf( "svc_nop\n" );
				break;

			case svc_servercmd:
				if( !cls.reliable ) {
					int cmdNum = MSG_ReadInt32( msg );
					if( cmdNum < 0 ) {
						Com_Error( ERR_DROP, "CL_ParseServerMessage: Invalid cmdNum value received: %i\n",
								   cmdNum );
						return;
					}
					if( cmdNum <= cls.lastExecutedServerCommand ) {
						MSG_ReadString( msg ); // read but ignore
						break;
					}
					cls.lastExecutedServerCommand = cmdNum;
				}
			// fall through
			case svc_servercs: // configstrings from demo files. they don't have acknowledge
				CL_ParseServerCommand( msg );
				break;

			case svc_serverdata:
				if( cls.state == CA_HANDSHAKE ) {
					CL_Cbuf_ExecutePendingCommands(); // make sure any stuffed commands are done
					CL_ParseServerData( msg );
				} else {
					return; // ignore rest of the packet (serverdata is always sent alone)
				}
				break;

			case svc_spawnbaseline:
				CL_ParseBaseline( msg );
				break;

			case svc_download:
				CL_ParseDownload( msg );
				break;

			case svc_clcack:
				if( cls.reliable ) {
					Com_Error( ERR_DROP, "CL_ParseServerMessage: clack message for reliable client\n" );
					return;
				}
				cls.reliableAcknowledge = MSG_ReadUintBase128( msg );
				cls.ucmdAcknowledged = MSG_ReadUintBase128( msg );
				if( cl_debug_serverCmd->integer & 4 ) {
					const char *format = "svc_clcack:reliable cmd ack:%" PRIi64 " ucmdack:%" PRIi64 "\n";
					Com_Printf( format, cls.reliableAcknowledge, cls.ucmdAcknowledged );
				}
				break;

			case svc_frame:
				CL_ParseFrame( msg );
				break;

			case svc_demoinfo:
				assert( cls.demoPlayer.playing );

				MSG_ReadInt32( msg );
				MSG_ReadInt32( msg );
				cls.demoPlayer.meta_data_realsize = (size_t)MSG_ReadInt32( msg );
				meta_data_maxsize = (size_t)MSG_ReadInt32( msg );

				// sanity check
				if( cls.demoPlayer.meta_data_realsize > meta_data_maxsize ) {
					cls.demoPlayer.meta_data_realsize = meta_data_maxsize;
				}
				if( cls.demoPlayer.meta_data_realsize > sizeof( cls.demoPlayer.meta_data ) ) {
					cls.demoPlayer.meta_data_realsize = sizeof( cls.demoPlayer.meta_data );
				}

				MSG_ReadData( msg, cls.demoPlayer.meta_data, cls.demoPlayer.meta_data_realsize );
				MSG_SkipData( msg, meta_data_maxsize - cls.demoPlayer.meta_data_realsize );
				break;

			case svc_playerinfo:
			case svc_packetentities:
			case svc_match:
				Com_Error( ERR_DROP, "Out of place frame data" );
				break;

			case svc_extension:
				ext = MSG_ReadUint8( msg );  // extension id
				MSG_ReadUint8( msg );        // version number
				len = MSG_ReadInt16( msg ); // command length

				switch( ext ) {
					default:
						// unsupported
						MSG_SkipData( msg, len );
						break;
				}
				break;
		}
	}

	if( msg->readcount > msg->cursize ) {
		Com_Error( ERR_DROP, "CL_ParseServerMessage: Bad server message" );
		return;
	}

	if( cl_debug_serverCmd->integer & 4 ) {
		Com_Printf( "%3" PRIi64 ":CMD %i %s\n", (int64_t)( msg->readcount ), -1, "EOF" );
	}
	SHOWNET( msg, "END OF MESSAGE" );

	CL_AddNetgraph();

	//
	// if recording demos, copy the message out
	//
	//
	// we don't know if it is ok to save a demo message until
	// after we have parsed the frame
	//
	if( cls.demoRecorder.recording && !cls.demoRecorder.waiting ) {
		CL_WriteDemoMessage( msg );
	}
}
