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
// console.c

#include "client.h"
#include "../ref/frontend.h"

#define CON_MAXLINES    500
typedef struct {
	char *text[CON_MAXLINES];
	int x;              // offset in current line for next print
	int linecolor;      // color in current line for next print
	int linewidth;      // characters across screen (FIXME)
	int display;        // bottom of console displays this line
	int totallines;     // total lines in console scrollback
	int numlines;       // non-empty lines in console scrollback

	float times[NUM_CON_TIMES]; // cls.realtime time the line was generated
	// for transparent notify lines

	qmutex_t *mutex;
} console_t;

static console_t con;

volatile bool con_initialized;

static bool con_hasKeyboardFocus;

static cvar_t *con_notifytime;
static cvar_t *con_drawNotify;
static cvar_t *con_chatmode;
cvar_t *con_printText;

// console input line editing
#define     MAXCMDLINE  256
static char key_lines[32][MAXCMDLINE];
static unsigned int key_linepos;    // byte offset of cursor in edit line
static int input_prestep;           // pixels to skip at start when drawing
static int edit_line = 0;
static int history_line = 0;
static int search_line = 0;
static char search_text[MAXCMDLINE * 2 + 4];

static inline bool Key_IsDown( int key ) {
	return wsw::cl::KeyHandlingSystem::instance()->isKeyDown( key );
}

bool Con_HasKeyboardFocus() {
	return con_hasKeyboardFocus;
}

#define ctrl_is_down ( Key_IsDown( K_LCTRL ) || Key_IsDown( K_RCTRL ) )

/*
* Con_NumPadValue
*/
static int Con_NumPadValue( int key ) {
	switch( key ) {
		case KP_SLASH:
			return '/';

		case KP_STAR:
			return '*';

		case KP_MINUS:
			return '-';

		case KP_PLUS:
			return '+';

		case KP_HOME:
			return '7';

		case KP_UPARROW:
			return '8';

		case KP_PGUP:
			return '9';

		case KP_LEFTARROW:
			return '4';

		case KP_5:
			return '5';

		case KP_RIGHTARROW:
			return '6';

		case KP_END:
			return '1';

		case KP_DOWNARROW:
			return '2';

		case KP_PGDN:
			return '3';

		case KP_INS:
			return '0';

		case KP_DEL:
			return '.';
	}

	return key;
}

/*
* Con_ClearTyping
*/
static void Con_ClearTyping( void ) {
	key_lines[edit_line][1] = 0; // clear any typing
	key_linepos = 1;
	search_line = edit_line;
	search_text[0] = 0;
}

/*
* Con_Close
*/
void Con_Close( void ) {
	scr_con_current = 0;

	Con_ClearTyping();
	Con_ClearNotify();
	CL_ClearInputState();
}

/*
* Con_ToggleConsole_f
*/
void Con_ToggleConsole_f( void ) {
	SCR_EndLoadingPlaque(); // get rid of loading plaque

	if( cls.state == CA_GETTING_TICKET || cls.state == CA_CONNECTING || cls.state == CA_CONNECTED ) {
		return;
	}

	Con_ClearTyping();
	Con_ClearNotify();

	con_hasKeyboardFocus = !con_hasKeyboardFocus;
}

/*
* Con_Clear_f
*/
void Con_Clear_f( void ) {
	int i;

	QMutex_Lock( con.mutex );

	for( i = 0; i < CON_MAXLINES; i++ ) {
		Q_free( con.text[i] );
		con.text[i] = NULL;
	}
	con.numlines = 0;
	con.display = 0;
	con.x = 0;
	con.linecolor = COLOR_WHITE;

	QMutex_Unlock( con.mutex );
}

/*
* Con_BufferText
*
* Copies into console text 'buffer' as a single 'delim'-separated string
* Returns resulting number of characters, not counting the trailing zero
*/
static size_t Con_BufferText( char *buffer, const char *delim ) {
	int l, x;
	const char *line;
	size_t length, delim_len = strlen( delim );

	if( !con_initialized ) {
		return 0;
	}

	length = 0;
	for( l = con.numlines - 1; l >= 0; l-- ) {
		line = con.text[l] ? con.text[l] : "";
		x = strlen( line );

		if( buffer ) {
			memcpy( buffer + length, line, x );
			memcpy( buffer + length + x, delim, delim_len );
		}

		length += x + delim_len;
	}

	if( buffer ) {
		buffer[length] = '\0';
	}

	return length;
}

/*
* Con_RemoveBufferColorTokens
*
* Remove color tokens from the `in` buffer and write it to the `out`
* buffer. `out` buffer must be at least as long as `in`.
* It is valid to pass a buffer as both `in` and `out` to uncolor it
* inplace.
* Returns numer of chars written to the `out` buffer.
*/
static size_t Con_RemoveBufferColorTokens( const char *in, size_t in_size, char *out ) {
	const char *end = in + in_size;
	char *outp = out, c;
	int gc;

	while( in < end ) {
		gc = Q_GrabCharFromColorString( &in, &c, NULL );
		switch( gc ) {
			case GRABCHAR_END:
				++in;
			case GRABCHAR_CHAR:
				*outp++ = c;
			case GRABCHAR_COLOR:
				break;
			default:
				assert( 0 );
		}
	}

	assert( outp - out >= 0 );
	return (size_t)( outp - out );
}

/*
* Con_Dump_f
*
* Save the console contents out to a file
*/
static void Con_Dump_f( void ) {
	int file;
	size_t buffer_size;
	char *buffer;
	size_t name_size;
	char *name;
	const char *newline = "\r\n";

	if( !con_initialized ) {
		return;
	}

	if( Cmd_Argc() != 2 ) {
		Com_Printf( "usage: condump <filename>\n" );
		return;
	}

	name_size = sizeof( char ) * ( strlen( Cmd_Argv( 1 ) ) + strlen( ".txt" ) + 1 );
	name = (char *)Q_malloc( name_size );

	Q_strncpyz( name, Cmd_Argv( 1 ), name_size );
	COM_DefaultExtension( name, ".txt", name_size );
	COM_SanitizeFilePath( name );

	if( !COM_ValidateRelativeFilename( name ) ) {
		Com_Printf( "Invalid filename.\n" );
		Q_free( name );
		return;
	}

	if( FS_FOpenFile( name, &file, FS_WRITE ) == -1 ) {
		Com_Printf( "Couldn't open: %s\n", name );
		Q_free( name );
		return;
	}

	QMutex_Lock( con.mutex );

	buffer_size = Con_BufferText( NULL, newline ) + 1;
	buffer = (char *)Q_malloc( buffer_size );

	Con_BufferText( buffer, newline );
	buffer_size = Con_RemoveBufferColorTokens( buffer, buffer_size, buffer );

	QMutex_Unlock( con.mutex );

	FS_Write( buffer, buffer_size - 1, file );

	FS_FCloseFile( file );

	Q_free( buffer );

	Com_Printf( "Dumped console text: %s\n", name );
	Q_free( name );
}

/*
* Con_ClearNotify
*/
void Con_ClearNotify( void ) {
	int i;

	for( i = 0; i < NUM_CON_TIMES; i++ )
		con.times[i] = 0;
}

/*
* Con_CheckResize
*
* If the line width has changed, reformat the buffer.
*/
void Con_CheckResize( void ) {
	int charWidth, width = 0;

	if( cls.consoleFont ) {
		charWidth = SCR_strWidth( "M", cls.consoleFont, 0, 0 );
		if( !charWidth ) {
			charWidth = 1;
		}

		width = viddef.width / charWidth - 2;
	}

	if( width < 1 ) {   // video hasn't been initialized yet
		con.linewidth = 78;
	} else {
		con.linewidth = width;
	}
}

/*
* Con_ResetFontSize
*/
void Con_ResetFontSize() {
	SCR_ResetSystemFontConsoleSize();
}

/*
* Con_ChangeFontSize
*/
void Con_ChangeFontSize( int ch ) {
	SCR_ChangeSystemFontConsoleSize( ch );
}

/*
* Con_GetPixelRatio
*/
float Con_GetPixelRatio( void ) {
	float pixelRatio = VID_GetPixelRatio();
	clamp_low( pixelRatio, 0.5f );
	return pixelRatio;
}

/*
* Con_Init
*/
void Con_Init( void ) {
	int i;

	if( con_initialized ) {
		return;
	}

	for( i = 0; i < 32; i++ ) {
		key_lines[i][0] = ']';
		key_lines[i][1] = 0;
	}
	key_linepos = 1;

	con.totallines = CON_MAXLINES;
	con.numlines = 0;
	con.display = 0;
	con.linewidth = 78;
	con.linecolor = COLOR_WHITE;
	con.mutex = QMutex_Create();

	Com_Printf( "Console initialized.\n" );

	//
	// register our commands
	//
	con_notifytime = Cvar_Get( "con_notifytime", "3", CVAR_ARCHIVE );
	con_drawNotify = Cvar_Get( "con_drawNotify", "0", CVAR_ARCHIVE );
	con_printText  = Cvar_Get( "con_printText", "1", CVAR_ARCHIVE );
	con_chatmode = Cvar_Get( "con_chatmode", "3", CVAR_ARCHIVE );

	Cmd_AddCommand( "toggleconsole", Con_ToggleConsole_f );
	Cmd_AddCommand( "clear", Con_Clear_f );
	Cmd_AddCommand( "condump", Con_Dump_f );
	con_initialized = true;
}

/*
* Con_Shutdown
*/
void Con_Shutdown( void ) {
	if( !con_initialized ) {
		return;
	}

	Con_Clear_f();  // free scrollback text

	Cmd_RemoveCommand( "toggleconsole" );
	Cmd_RemoveCommand( "clear" );
	Cmd_RemoveCommand( "condump" );

	QMutex_Destroy( &con.mutex );

	con_initialized = false;
}

/*
* Con_Linefeed
*/
static void Con_Linefeed( bool notify ) {
	// shift scrollback text up in the buffer to make room for a new line
	if( con.numlines == con.totallines ) {
		Q_free( con.text[con.numlines - 1] );
	}
	memmove( con.text + 1, con.text, sizeof( con.text[0] ) * std::min( con.numlines, con.totallines - 1 ) );
	con.text[0] = NULL;

	// mark time for transparent overlay
	memmove( con.times + 1, con.times, sizeof( con.times[0] ) * ( NUM_CON_TIMES - 1 ) );
	con.times[0] = cls.realtime;
	if( !notify ) {
		con.times[0] -= con_notifytime->value * 1000 + 1;
	}

	con.x = 0;
	if( con.display ) {
		// the console is scrolled up, stay in the same place if possible
		con.display++;
		clamp_high( con.display, con.totallines - 1 );
	}
	con.numlines++;
	clamp_high( con.numlines, con.totallines );
}

/*
* Con_Print
*
* Handles cursor positioning, line wrapping, etc
* All console printing must go through this in order to be logged to disk
* If no console is visible, the text will appear at the top of the game window
*/
static void addcharstostr( char **s, const char *c, size_t num ) {
	size_t len = *s ? strlen( *s ) : 0, addlen = 0;
	char *newstr;

	while( num && c[addlen] ) {
		addlen = Q_Utf8SyncPos( c, addlen + 1, UTF8SYNC_RIGHT );
		num--;
	}

	newstr = (char *)Q_realloc( *s, len + addlen + 1 );
	memcpy( newstr + len, c, addlen );
	newstr[len + addlen] = '\0';
	*s = newstr;
}
static void Con_Print2( const char *txt, bool notify ) {
	int l;
	const char *ptxt;
	char colorchar[] = { Q_COLOR_ESCAPE, COLOR_WHITE, 0 };

	if( !con_initialized ) {
		return;
	}

	if( con_printText && con_printText->integer == 0 ) {
		return;
	}

	QMutex_Lock( con.mutex );

	while( *txt ) {
		ptxt = txt;

		if( txt[0] == Q_COLOR_ESCAPE ) {
			if( txt[1] == Q_COLOR_ESCAPE ) {
				txt++;
			} else if( ( txt[1] >= '0' ) && ( txt[1] < ( '0' + MAX_S_COLORS ) ) ) {
				con.linecolor = colorchar[1] = txt[1];
				addcharstostr( &con.text[0], colorchar, 2 );
				txt += 2;
				continue;
			}
		}

		// count word length
		for( l = 0; l < con.linewidth; ) {
			if( ptxt[0] == Q_COLOR_ESCAPE ) {
				if( ptxt[1] == Q_COLOR_ESCAPE ) {
					l++;
					ptxt += 2;
					continue;
				}
				if( ( txt[1] >= '0' ) && ( txt[1] < ( '0' + MAX_S_COLORS ) ) ) {
					ptxt += 2;
					continue;
				}
			} else if( ( ( unsigned char )( ptxt[0] ) <= ' ' ) || Q_IsBreakingSpace( ptxt ) ) {
				break;
			}
			l++;
			ptxt += Q_Utf8SyncPos( ptxt, 1, UTF8SYNC_RIGHT );
		}

		// word wrap
		if( l != con.linewidth && ( con.x + l > con.linewidth ) ) {
			con.x = 0;
		}

		if( !con.x ) {
			Con_Linefeed( notify );

			if( con.linecolor != COLOR_WHITE ) {
				colorchar[1] = con.linecolor;
				addcharstostr( &con.text[0], colorchar, 2 );
			}
		}

		switch( txt[0] ) {
			case '\n':
				con.linecolor = COLOR_WHITE;
				con.x = 0;
				break;

			case '\r':
				break;

			default: // display character and advance
				if( txt[0] == Q_COLOR_ESCAPE ) {
					addcharstostr( &con.text[0], txt, 1 );
				}
				addcharstostr( &con.text[0], txt, 1 );
				con.x++;
				if( con.x >= con.linewidth ) { // haha welcome to 1995 lol
					con.x = 0;
				}
				break;
		}

		txt += Q_Utf8SyncPos( txt, 1, UTF8SYNC_RIGHT );
	}

	QMutex_Unlock( con.mutex );
}

void Con_Print( const char *txt ) {
	Con_Print2( txt, true );
}

void Con_PrintSilent( const char *txt ) {
	Con_Print2( txt, false );
}

/*
==============================================================================

DRAWING

==============================================================================
*/

/*
* Q_ColorCharCount
*/
int Q_ColorCharCount( const char *s, int byteofs ) {
	wchar_t c;
	const char *end = s + byteofs;
	int charcount = 0;

	while( s < end ) {
		int gc = Q_GrabWCharFromColorString( &s, &c, NULL );
		if( gc == GRABCHAR_CHAR ) {
			charcount++;
		} else if( gc == GRABCHAR_COLOR ) {
			;
		} else if( gc == GRABCHAR_END ) {
			break;
		} else {
			assert( 0 );
		}
	}

	return charcount;
}

/*
* Q_ColorCharOffset
*/
int Q_ColorCharOffset( const char *s, int charcount ) {
	const char *start = s;
	wchar_t c;

	while( *s && charcount ) {
		int gc = Q_GrabWCharFromColorString( &s, &c, NULL );
		if( gc == GRABCHAR_CHAR ) {
			charcount--;
		} else if( gc == GRABCHAR_COLOR ) {
			;
		} else if( gc == GRABCHAR_END ) {
			break;
		} else {
			assert( 0 );
		}
	}

	return s - start;
}

/*
* Con_DrawInput
*
* The input line scrolls horizontally if typing goes beyond the right edge
*/
static void Con_DrawInput( int vislines ) {
	char draw_search_text[MAXCMDLINE * 2 + 4];
	const char *text = key_lines[edit_line];
	float pixelRatio = Con_GetPixelRatio();
	int smallCharHeight = SCR_FontHeight( cls.consoleFont );
	int margin = 8 * pixelRatio;
	int promptwidth = SCR_strWidth( "]", cls.consoleFont, 1, 0 );
	int input_width = viddef.width - margin * 2 - promptwidth - SCR_strWidth( "_", cls.consoleFont, 1, 0 );
	int text_x = margin + promptwidth;
	int text_y = vislines - (int)( 14 * pixelRatio ) - smallCharHeight;
	int textwidth;
	int prewidth;   // width of input line before cursor

	if( !Con_HasKeyboardFocus() ) {
		return;
	}

	if( search_text[0] ) {
		text = draw_search_text;
		Q_snprintfz( draw_search_text, sizeof( draw_search_text ), "%s : %s", key_lines[edit_line], search_text );
	}

	text++;

	textwidth = SCR_strWidth( text, cls.consoleFont, 0, 0 );
	prewidth = ( ( key_linepos > 1 ) ? SCR_strWidth( text, cls.consoleFont, key_linepos - 1, 0 ) : 0 );

	if( textwidth > input_width ) {
		// don't let the cursor go beyond the left screen edge
		clamp_high( input_prestep, prewidth );
		// don't let it go beyond the right screen edge
		clamp_low( input_prestep, prewidth - input_width );
		// don't leave an empty space after the string when deleting a character
		if( ( textwidth - input_prestep ) < input_width ) {
			input_prestep = textwidth - input_width;
		}
	} else {
		input_prestep = 0;
	}

	SCR_DrawRawChar( text_x - promptwidth, text_y, ']', cls.consoleFont, colorWhite );

	SCR_DrawClampString( text_x - input_prestep, text_y, text, text_x, text_y,
						 text_x + input_width, viddef.height, cls.consoleFont, colorWhite, 0 );

	if( (int)( cls.realtime >> 8 ) & 1 ) {
		SCR_DrawRawChar( text_x + prewidth - input_prestep, text_y, '_',
						 cls.consoleFont, colorWhite );
	}
}

/*
* Con_DrawNotify
*
* Draws the last few lines of output transparently over the game top
*/
void Con_DrawNotify( void ) {
	int v;
	const char *text;
	int i;
	int time;
	float pixelRatio = Con_GetPixelRatio();

	if( cls.state == CA_ACTIVE && CG_HasKeyboardFocus() ) {
		v = 0;
		if( con_drawNotify->integer || developer->integer ) {
			int x = 8 * pixelRatio;

			QMutex_Lock( con.mutex );

			for( i = std::min( NUM_CON_TIMES, con.numlines ) - 1; i >= 0; i-- ) {
				time = con.times[i];
				if( time == 0 ) {
					continue;
				}
				time = cls.realtime - time;
				if( time > con_notifytime->value * 1000 ) {
					continue;
				}
				text = con.text[i] ? con.text[i] : "";

				SCR_DrawString( x, v, ALIGN_LEFT_TOP, text, cls.consoleFont, colorWhite, 0 );

				v += SCR_FontHeight( cls.consoleFont );
			}

			QMutex_Unlock( con.mutex );
		}
	}
}

/*
* Con_DrawConsole
*
* Draws the console with the solid background
*/
void Con_DrawConsole( void ) {
	int i, x, y;
	int rows;
	const char *text;
	int row;
	unsigned int lines;
	char version[256];
	time_t long_time;
	struct tm *newtime;
	int smallCharHeight = SCR_FontHeight( cls.consoleFont );
	float pixelRatio = Con_GetPixelRatio();
	int scaled;

	lines = viddef.height * scr_con_current;
	if( lines <= 0 ) {
		return;
	}
	if( !smallCharHeight ) {
		return;
	}

	QMutex_Lock( con.mutex );

	if( lines > viddef.height ) {
		lines = viddef.height;
	}

	// draw the background
	RF_DrawStretchPic( 0, 0, viddef.width, lines, 0, 0, 1, 1, colorWhite, cls.consoleShader );
	scaled = 2 * pixelRatio;
	SCR_DrawFillRect( 0, lines - scaled, viddef.width, scaled, colorOrange );

	// get date from system
	time( &long_time );
	newtime = localtime( &long_time );

	Q_snprintfz( version, sizeof( version ), "%02d:%02d %s v%4.2f alpha", newtime->tm_hour, newtime->tm_min,
				 APPLICATION, APP_VERSION );

	scaled = 4 * pixelRatio;
	SCR_DrawString( viddef.width - SCR_strWidth( version, cls.consoleFont, 0, 0 ) - scaled,
					lines - SCR_FontHeight( cls.consoleFont ) - scaled,
					ALIGN_LEFT_TOP, version, cls.consoleFont, colorOrange, 0 );

	// prepare to draw the text
	scaled = 14 * pixelRatio;
	rows = ( lines - smallCharHeight - scaled ) / smallCharHeight;  // rows of text to draw
	y = lines - smallCharHeight - scaled - smallCharHeight;

	row = con.display;  // first line to be drawn
	if( con.display ) {
		int width = SCR_strWidth( "^", cls.consoleFont, 0, 0 );

		// draw arrows to show the buffer is backscrolled
		for( x = 0; x < con.linewidth; x += 4 )
			SCR_DrawRawChar( ( x + 1 ) * width, y, '^', cls.consoleFont, colorOrange );

		// the arrows obscure one line of scrollback
		y -= smallCharHeight;
		rows--;
		row++;
	}

	// draw from the bottom up
	for( i = 0; i < rows; i++, y -= smallCharHeight, row++ ) {
		if( row >= con.numlines ) {
			break; // reached top of scrollback

		}
		text = con.text[row] ? con.text[row] : "";

		SCR_DrawString( 8 * pixelRatio, y, ALIGN_LEFT_TOP, text, cls.consoleFont, colorWhite, 0 );
	}

	// draw the input prompt, user text, and cursor if desired
	Con_DrawInput( lines );

	QMutex_Unlock( con.mutex );
}


/*
* Con_DisplayList

* New function for tab-completion system
* Added by EvilTypeGuy
* MEGA Thanks to Taniwha
*/
static void Con_DisplayList( char **list ) {
	int i, j;
	int len = 0;
	int maxlen = 0;
	int width = con.linewidth - 4;
	int columns;
	int columnwidth;
	int items = 0;

	while( list[items] ) {
		len = (int)strlen( list[items] );
		if( len > maxlen ) {
			maxlen = len;
		}
		items++;
	}
	maxlen += 2;
	columns = width / maxlen;

	if( columns == 0 ) {
		for( i = 0; i < items; i++ )
			Com_Printf( "%s ", list[i] );
		Com_Printf( "\n" );
	} else {
		for( i = 0; i < items; i++ ) {
			columnwidth = 0;
			for( j = i % columns; j < items; j += columns ) {
				len = (int)strlen( list[j] );
				if( len > columnwidth ) {
					columnwidth = len;
				}
			}
			columnwidth += 2;

			len = (int)strlen( list[i] );

			Com_Printf( "%s", list[i] );
			for( j = 0; j < columnwidth - len; j++ )
				Com_Printf( " " );

			if( i % columns == columns - 1 ) {
				Com_Printf( "\n" );
			}
		}

		if( i % columns != 0 ) {
			Com_Printf( "\n" );
		}
	}

	Com_Printf( "\n" );
}

/*
* Con_CompleteCommandLine

* New function for tab-completion system
* Added by EvilTypeGuy
* Thanks to Fett erich@heintz.com
* Thanks to taniwha
*/
static void Con_CompleteCommandLine() {
	char stub[1] = { '\0' };
	char *cmd = stub;
	char *s;
	int c, v, a, ca, i;
	int cmd_len;
	char **list[6] = { 0, 0, 0, 0, 0, 0 };

	s = key_lines[edit_line] + 1;
	if( *s == '\\' || *s == '/' ) {
		s++;
	}
	if( !*s ) {
		return;
	}

	// Count number of possible matches
	c = Cmd_CompleteCountPossible( s );
	v = Cvar_CompleteCountPossible( s );
	a = Cmd_CompleteAliasCountPossible( s );
	ca = 0;

	if( !( c + v + a ) ) {
		// now see if there's any valid cmd in there, providing
		// a list of matching arguments
		list[4] = Cmd_CompleteBuildArgList( s );
		if( !list[4] ) {
			// No possible matches, let the user know they're insane
			Com_Printf( "\nNo matching aliases, commands, cvars, or dynvars were found.\n\n" );
			return;
		}

		// count the number of matching arguments
		for( ca = 0; list[4][ca]; ca++ ) ;
		if( !ca ) {
			// the list is empty, although non-NULL list pointer suggests that the command
			// exists, so clean up and exit without printing anything
			Q_free( list[4] );
			return;
		}
	}

	if( c + v + a + ca == 1 ) {
		// find the one match to rule them all
		if( c ) {
			list[0] = Cmd_CompleteBuildList( s );
		} else if( v ) {
			list[0] = Cvar_CompleteBuildList( s );
		} else if( a ) {
			list[0] = Cmd_CompleteAliasBuildList( s );
		} else {
			list[0] = list[4], list[4] = NULL;
		}
		cmd = *list[0];
		cmd_len = (int)strlen( cmd );
	} else {
		int i_start = 0;

		if( c ) {
			cmd = *( list[0] = Cmd_CompleteBuildList( s ) );
		}
		if( v ) {
			cmd = *( list[1] = Cvar_CompleteBuildList( s ) );
		}
		if( a ) {
			cmd = *( list[2] = Cmd_CompleteAliasBuildList( s ) );
		}
		if( ca ) {
			s = strstr( s, " " ) + 1, cmd = *( list[4] ), i_start = 4;
		}

		cmd_len = (int)strlen( s );
		do {
			for( i = i_start; i < 5; i++ ) {
				char ch = cmd[cmd_len];
				char **l = list[i];
				if( l ) {
					while( *l && ( *l )[cmd_len] == ch )
						l++;
					if( *l ) {
						break;
					}
				}
			}
			if( i == 5 ) {
				cmd_len++;
			}
		} while( i == 5 );

		// Print Possible Commands
		if( c ) {
			Com_Printf( S_COLOR_RED "%i possible command%s%s\n", c, ( c > 1 ) ? "s: " : ":", S_COLOR_WHITE );
			Con_DisplayList( list[0] );
		}

		if( v ) {
			Com_Printf( S_COLOR_CYAN "%i possible variable%s%s\n", v, ( v > 1 ) ? "s: " : ":", S_COLOR_WHITE );
			Con_DisplayList( list[1] );
		}

		if( a ) {
			Com_Printf( S_COLOR_MAGENTA "%i possible alias%s%s\n", a, ( a > 1 ) ? "es: " : ":", S_COLOR_WHITE );
			Con_DisplayList( list[2] );
		}

		if( ca ) {
			Com_Printf( S_COLOR_GREEN "%i possible argument%s%s\n", ca, ( ca > 1 ) ? "s: " : ":", S_COLOR_WHITE );
			Con_DisplayList( list[4] );
		}
	}

	if( cmd ) {
		int skip = 1;
		char *cmd_temp = NULL, *p;

		if( con_chatmode && con_chatmode->integer != 3 ) {
			skip++;
			key_lines[edit_line][1] = '/';
		}

		if( ca ) {
			size_t temp_size;

			temp_size = sizeof( key_lines[edit_line] );
			cmd_temp = (char *)Q_malloc( temp_size );

			Q_strncpyz( cmd_temp, key_lines[edit_line] + skip, temp_size );
			p = strstr( cmd_temp, " " );
			if( p ) {
				*( p + 1 ) = '\0';
			}

			cmd_len += strlen( cmd_temp );

			Q_strncatz( cmd_temp, cmd, temp_size );
			cmd = cmd_temp;
		}

		Q_strncpyz( key_lines[edit_line] + skip, cmd, sizeof( key_lines[edit_line] ) - ( 1 + skip ) );
		key_linepos = std::min( (unsigned)( cmd_len + skip ), (unsigned)sizeof( key_lines[edit_line] ) - 1 );

		if( c + v + a == 1 && key_linepos < sizeof( key_lines[edit_line] ) - 1 ) {
			key_lines[edit_line][key_linepos] = ' ';
			key_linepos++;
		}
		key_lines[edit_line][key_linepos] = 0;

		if( cmd == cmd_temp ) {
			Q_free( cmd );
		}
	}

	for( i = 0; i < 5; ++i ) {
		if( list[i] ) {
			Q_free( list[i] );
		}
	}
}

/*
==============================================================================

LINE TYPING INTO THE CONSOLE

==============================================================================
*/


/*
* Con_Key_Copy
*
* Copies console text to clipboard
* Should be Con_Copy prolly
*/
static void Con_Key_Copy( void ) {
	size_t buffer_size;
	char *buffer;
	const char *newline = "\r\n";

	if( search_text[0] ) {
		CL_SetClipboardData( search_text );
		return;
	}

	QMutex_Lock( con.mutex );

	buffer_size = Con_BufferText( NULL, newline ) + 1;
	buffer = (char *)Q_malloc( buffer_size );

	Con_BufferText( buffer, newline );

	QMutex_Unlock( con.mutex );

	CL_SetClipboardData( buffer );

	Q_free( buffer );
}

/*
* Con_Key_Paste
*
* Inserts stuff from clipboard to console
* Should be Con_Paste prolly
*/
static void Con_Key_Paste( void ) {
	char *cbd;
	char *tok;
	size_t linelen, i, next;

	cbd = CL_GetClipboardData();
	if( cbd ) {
		tok = strtok( cbd, "\n\r\b" );

		while( tok != NULL ) {
			linelen = strlen( key_lines[edit_line] );
			i = 0;
			while( tok[i] ) {
				next = Q_Utf8SyncPos( tok, i + 1, UTF8SYNC_RIGHT );
				if( next + linelen >= MAXCMDLINE ) {
					break;
				}
				i = next;
			}

			if( i ) {
				memmove( key_lines[edit_line] + key_linepos + i, key_lines[edit_line] + key_linepos, linelen - key_linepos + 1 );
				memcpy( key_lines[edit_line] + key_linepos, tok, i );
				key_linepos += i;
			}

			tok = strtok( NULL, "\n\r\b" );

			if( tok != NULL ) {
				if( key_lines[edit_line][1] == '\\' || key_lines[edit_line][1] == '/' ) {
					Cbuf_AddText( key_lines[edit_line] + 2 ); // skip the >
				} else {
					Cbuf_AddText( key_lines[edit_line] + 1 ); // valid command

				}
				Cbuf_AddText( "\n" );
				Com_Printf( "%s\n", key_lines[edit_line] );
				edit_line = ( edit_line + 1 ) & 31;
				history_line = edit_line;
				search_line = edit_line;
				search_text[0] = 0;
				key_lines[edit_line][0] = ']';
				key_lines[edit_line][1] = 0;
				key_linepos = 1;
				if( cls.state == CA_DISCONNECTED ) {
					SCR_UpdateScreen(); // force an update, because the command may take some time
				}
			}
		}

		CL_FreeClipboardData( cbd );
	}
}

/*
* Con_CharEvent
*
* Interactive line editing and console scrollback only for (Unicode) chars
*/
static void Con_CharEvent( wchar_t key ) {
	if( !con_initialized ) {
		return;
	}

	key = Con_NumPadValue( key );

	if( cls.state == CA_GETTING_TICKET || cls.state == CA_CONNECTING || cls.state == CA_CONNECTED ) {
		return;
	}

	switch( key ) {
		case 22: // CTRL - V : paste
			Con_Key_Paste();
			return;

		case 12: // CTRL - L : clear
			Cbuf_AddText( "clear\n" );
			return;

		/*
		case 8: // CTRL+H or Backspace
		if (key_linepos > 1)
		{
		// skip to the end of color sequence
		while (Q_IsColorString(key_lines[edit_line] + key_linepos))
		key_linepos += 2;

		strcpy (key_lines[edit_line] + key_linepos - 1, key_lines[edit_line] + key_linepos);
		key_linepos--;
		}
		return;
		*/

		case 16: // CTRL+P : history prev
			do {
				history_line = ( history_line - 1 ) & 31;
			} while( history_line != edit_line && !key_lines[history_line][1] );

			if( history_line == edit_line ) {
				history_line = ( edit_line + 1 ) & 31;
			}

			Q_strncpyz( key_lines[edit_line], key_lines[history_line], sizeof( key_lines[edit_line] ) );
			key_linepos = (unsigned int)strlen( key_lines[edit_line] );
			input_prestep = 0;
			return;

		case 14: // CTRL+N : history next
			if( history_line == edit_line ) {
				return;
			}

			do {
				history_line = ( history_line + 1 ) & 31;
			} while( history_line != edit_line && !key_lines[history_line][1] );

			if( history_line == edit_line ) {
				key_lines[edit_line][0] = ']';
				key_linepos = 1;
			} else {
				Q_strncpyz( key_lines[edit_line], key_lines[history_line], sizeof( key_lines[edit_line] ) );
				key_linepos = (unsigned int)strlen( key_lines[edit_line] );
				input_prestep = 0;
			}
			return;

		case 3: // CTRL+C: copy text to clipboard
			Con_Key_Copy();
			return;

		case 1: // CTRL+A: jump to beginning of line (same as HOME)
			key_linepos = 1;
			return;
		case 5: // CTRL+E: jump to end of line (same as END)
			key_linepos = (unsigned int)strlen( key_lines[edit_line] );
			return;
		case 18: // CTRL+R
		{
			size_t search_len = strlen( key_lines[edit_line] + 1 );
			if( !search_len ) {
				break;
			}
			do {
				search_line = ( search_line - 1 ) & 31;
			} while( search_line != edit_line && Q_strnicmp( key_lines[search_line] + 1, key_lines[edit_line] + 1, search_len ) );

			if( search_line != edit_line ) {
				Q_strncpyz( search_text, key_lines[search_line] + 1, sizeof( search_text ) );
			} else {
				search_text[0] = '\0';
			}
		}
		break;
	}

	if( key < 32 || key > 0xFFFF ) {
		return; // non-printable

	}
	if( key_linepos < MAXCMDLINE - 1 ) {
		char *utf = Q_WCharToUtf8Char( key );
		int utflen = strlen( utf );

		if( strlen( key_lines[edit_line] ) + utflen >= MAXCMDLINE ) {
			return;     // won't fit

		}
		// move remainder to the right
		memmove( key_lines[edit_line] + key_linepos + utflen,
				 key_lines[edit_line] + key_linepos,
				 strlen( key_lines[edit_line] + key_linepos ) + 1 ); // +1 for trailing 0

		// insert the char sequence
		memcpy( key_lines[edit_line] + key_linepos, utf, utflen );
		key_linepos += utflen;
	}
}

bool Con_HandleCharEvent( wchar_t key ) {
	if( Con_HasKeyboardFocus() ) {
		Con_CharEvent( key );
		return true;
	}
	return false;
}

/*
* Con_SendChatMessage
*/
uint64_t Con_SendChatMessage( const char *text, bool team ) {
	const char *cmd;
	char buf[MAX_CHAT_BYTES], *p;

	// convert double quotes to single quotes
	Q_strncpyz( buf, text, sizeof( buf ) );
	for( p = buf; *p; p++ )
		if( *p == '"' ) {
			*p = '\'';
		}

	if( team && Cmd_Exists( "say_team" ) ) {
		cmd = "say_team";
	} else if( Cmd_Exists( "say" ) ) {
		cmd = "say";
	} else {
		cmd = "cmd say";
	}

	const auto oldCmdNum = cls.reliableSequence;
	Cmd_ExecuteString( va( "%s \"%s\"\n", cmd, buf ) );
	assert( oldCmdNum + 1 == cls.reliableSequence );
	return oldCmdNum;
}

/*
* Con_Key_Enter
*
* Handle K_ENTER keypress in console
*/
static void Con_Key_Enter( void ) {
	enum {COMMAND, CHAT, TEAMCHAT} type;
	char *p;
	int chatmode = con_chatmode ? con_chatmode->integer : 3;
	/* 1 = always chat unless with a slash;  non-1 = smart: unknown commands are chat.
	0 used to be the NetQuake way (always a command),
	but no one will probably want it in now */

	if( search_text[0] ) {
		key_lines[edit_line][0] = ']';
		Q_strncpyz( key_lines[edit_line] + 1, search_text, sizeof( key_lines[edit_line] ) - 1 );
		key_linepos = (unsigned int)strlen( key_lines[edit_line] );
		input_prestep = 0;
		search_line = 0;
		search_text[0] = 0;
	}

	// decide whether to treat the text as chat or command
	p = key_lines[edit_line] + 1;
	if( cls.state <= CA_HANDSHAKE || cls.demoPlayer.playing ) {
		type = COMMAND;
	} else if( *p == '\\' || *p == '/' ) {
		type = COMMAND;
	} else if( ctrl_is_down ) {
		type = TEAMCHAT;
	} else if( chatmode == 1 || !Cmd_CheckForCommand( p ) ) {
		type = CHAT;
	} else {
		type = COMMAND;
	}

	// do appropriate action
	switch( type ) {
		case CHAT:
		case TEAMCHAT:
			for( p = key_lines[edit_line] + 1; *p; p++ ) {
				if( *p != ' ' ) {
					break;
				}
			}
			if( !*p ) {
				break;  // just whitespace
			}
			Con_SendChatMessage( key_lines[edit_line] + 1, type == TEAMCHAT );
			break;

		case COMMAND:
			p = key_lines[edit_line] + 1; // skip the "]"
			if( *p == '\\' || ( *p == '/' && p[1] != '/' ) ) {
				p++;
			}
			Cbuf_AddText( p );
			Cbuf_AddText( "\n" );
			break;
	}

	// echo to the console and cycle command history
	Com_Printf( "%s\n", key_lines[edit_line] );
	edit_line = ( edit_line + 1 ) & 31;
	history_line = edit_line;
	search_line = edit_line;
	search_text[0] = 0;
	key_lines[edit_line][0] = ']';
	key_lines[edit_line][1] = 0;
	key_linepos = 1;
	if( cls.state == CA_DISCONNECTED ) {
		SCR_UpdateScreen(); // force an update, because the command
	}
	// may take some time
}

/*
* Con_HistoryUp
*/
static void Con_HistoryUp( void ) {
	do {
		history_line = ( history_line - 1 ) & 31;
	} while( history_line != edit_line && !key_lines[history_line][1] );

	if( history_line == edit_line ) {
		history_line = ( edit_line + 1 ) & 31;
	}

	Q_strncpyz( key_lines[edit_line], key_lines[history_line], sizeof( key_lines[edit_line] ) );
	key_linepos = (unsigned int)strlen( key_lines[edit_line] );
	input_prestep = 0;
}

/*
* Con_HistoryDown
*/
static void Con_HistoryDown( void ) {
	if( history_line == edit_line ) {
		return;
	}

	do {
		history_line = ( history_line + 1 ) & 31;
	} while( history_line != edit_line && !key_lines[history_line][1] );

	if( history_line == edit_line ) {
		key_lines[edit_line][0] = ']';
		key_lines[edit_line][1] = 0;
		key_linepos = 1;
	} else {
		Q_strncpyz( key_lines[edit_line], key_lines[history_line], sizeof( key_lines[edit_line] ) );
		key_linepos = (unsigned int)strlen( key_lines[edit_line] );
		input_prestep = 0;
	}
}

/*
* Con_KeyDown
*
* Interactive line editing and console scrollback except for ascii char
*/
static void Con_KeyDown( int key ) {
	if( !con_initialized ) {
		return;
	}

	if( cls.state == CA_GETTING_TICKET || cls.state == CA_CONNECTING || cls.state == CA_CONNECTED ) {
		return;
	}

	key = Con_NumPadValue( key );

	if( ( ( key == K_INS ) || ( key == KP_INS ) ) && ( Key_IsDown( K_LSHIFT ) || Key_IsDown( K_RSHIFT ) ) ) {
		Con_Key_Paste();
		return;
	}

	if( ( key == K_ENTER ) || ( key == KP_ENTER ) ) {
		Con_Key_Enter();
		return;
	}

	if( key == K_TAB ) {
		// command completion
		Con_CompleteCommandLine();
		return;
	}

	if( ( key == K_LEFTARROW ) || ( key == KP_LEFTARROW ) ) {
		int charcount;
		// jump over invisible color sequences
		charcount = Q_ColorCharCount( key_lines[edit_line], key_linepos );
		if( charcount > 1 ) {
			key_linepos = Q_ColorCharOffset( key_lines[edit_line], charcount - 1 );
		}
		return;
	}

	if( key == K_BACKSPACE ) {
		if( key_linepos > 1 ) {
			// skip to the end of color sequence
			while( 1 ) {
				char *tmp = key_lines[edit_line] + key_linepos;
				wchar_t c;
				if( Q_GrabWCharFromColorString( ( const char ** )&tmp, &c, NULL ) == GRABCHAR_COLOR ) {
					key_linepos = tmp - key_lines[edit_line]; // advance, try again
				} else {                                                                                     // GRABCHAR_CHAR or GRABCHAR_END
					break;
				}
			}

			{
				int oldpos = key_linepos;
				key_linepos = Q_Utf8SyncPos( key_lines[edit_line], key_linepos - 1,
											 UTF8SYNC_LEFT );
				key_linepos = std::max( key_linepos, 1u ); // Q_Utf8SyncPos could jump over [
				memmove( key_lines[edit_line] + key_linepos,
						 key_lines[edit_line] + oldpos, strlen( key_lines[edit_line] + oldpos ) + 1 );
			}
		}

		return;
	}

	if( key == K_DEL ) {
		char *s = key_lines[edit_line] + key_linepos;
		int wc = Q_GrabWCharFromUtf8String( ( const char ** )&s );
		if( wc ) {
			memmove( key_lines[edit_line] + key_linepos, s, strlen( s ) + 1 );
		}
		return;
	}

	if( ( key == K_RIGHTARROW ) || ( key == KP_RIGHTARROW ) ) {
		int charcount = Q_ColorCharCount( key_lines[edit_line], key_linepos );

		if( strlen( key_lines[edit_line] ) == key_linepos ) {
			char *oldline = key_lines[( edit_line + 31 ) & 31];
			int oldcharcount = Q_ColorCharCount( oldline, strlen( oldline ) );
			if( oldcharcount > charcount ) {
				int oldcharofs = Q_ColorCharOffset( oldline, charcount );
				int oldutflen = Q_Utf8SyncPos( oldline + oldcharofs, 1, UTF8SYNC_RIGHT );
				if( key_linepos + oldutflen < MAXCMDLINE ) {
					memcpy( key_lines[edit_line] + key_linepos, oldline + oldcharofs, oldutflen );
					key_linepos += oldutflen;
					key_lines[edit_line][key_linepos] = '\0';
				}
			}
		} else {
			// jump over invisible color sequences
			key_linepos = Q_ColorCharOffset( key_lines[edit_line], charcount + 1 );
		}
		return;
	}

	if( ( key == K_UPARROW ) || ( key == KP_UPARROW ) ) {
		Con_HistoryUp();
		return;
	}

	if( ( key == K_DOWNARROW ) || ( key == KP_DOWNARROW ) ) {
		Con_HistoryDown();
		return;
	}

	if( ( key == K_PGUP ) || ( key == KP_PGUP ) || ( key == K_MWHEELUP ) ) { // wsw : pb : support mwheel in console
		if( ( key == K_MWHEELUP ) && ctrl_is_down ) {
			Con_ChangeFontSize( 1 );
			return;
		}
		con.display += 2;
		clamp_high( con.display, con.numlines - 1 );
		clamp_low( con.display, 0 );    // in case con.numlines is 0
		return;
	}

	if( ( key == K_PGDN ) || ( key == KP_PGDN ) || ( key == K_MWHEELDOWN ) ) { // wsw : pb : support mwheel in console
		if( ( key == K_MWHEELDOWN ) && ctrl_is_down ) {
			Con_ChangeFontSize( -1 );
			return;
		}
		con.display -= 2;
		clamp_low( con.display, 0 );
		return;
	}

	if( key == K_HOME || key == KP_HOME ) {
		if( ctrl_is_down ) {
			int smallCharHeight = SCR_FontHeight( cls.consoleFont );
			int vislines = (int)( viddef.height * bound( 0.0, scr_con_current, 1.0 ) );
			int rows = ( vislines - smallCharHeight - (int)( 14 * Con_GetPixelRatio() ) ) / smallCharHeight;  // rows of text to draw
			con.display = con.numlines - rows + 1;
			clamp_low( con.display, 0 );
		} else {
			key_linepos = 1;
		}
		return;
	}

	if( key == K_END || key == KP_END ) {
		if( ctrl_is_down ) {
			con.display = 0;
		} else {
			key_linepos = (unsigned int)strlen( key_lines[edit_line] );
		}
		return;
	}

	if( key == '0' ) {
		if( ctrl_is_down ) {
			Con_ResetFontSize();
			return;
		}
	}

	// key is a normal printable key normal which wil be HANDLE later in response to WM_CHAR event
}

bool Con_HandleKeyEvent( int key, bool down ) {
	if( !Con_HasKeyboardFocus() ) {
		return false;
	}

	// TODO: Check whether the key is a "consolekey" (see old keys.cpp)
	if( !down ) {
		return true;
	}

	Con_KeyDown( key );
	return true;
}