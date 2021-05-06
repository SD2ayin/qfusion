#include "gametypesmodel.h"

#include "../qcommon/stringspanstorage.h"
#include "../qcommon/wswstringview.h"
#include "../qcommon/wswstaticvector.h"
#include "../qcommon/maplist.h"

#include <QJsonObject>

using wsw::operator""_asView;

namespace wsw::ui {

auto GametypesModel::roleNames() const -> QHash<int, QByteArray> {
	return {
		{ Name, "name" },
		{ Title, "title" },
		{ Flags, "flags" },
		{ Maps, "maps" },
		{ Desc, "desc" }
	};
};

auto GametypesModel::rowCount( const QModelIndex & ) const -> int {
	return (int)m_gametypes.size();
};

static inline auto toQString( const wsw::StringView &view ) -> QString {
	return QString::fromUtf8( view.data(), view.size() );
}

auto GametypesModel::data( const QModelIndex &index, int role ) const -> QVariant {
	if( !index.isValid() ) {
		return QVariant();
	}
	const int row = index.row();
	if( (unsigned)row >= m_gametypes.size() ) {
		return QVariant();
	}
	switch( role ) {
		case Name: return toQString( m_gametypes[row].getName() );
		case Title: return toQString( m_gametypes[row].getTitle() );
		case Flags: return (int)m_gametypes[row].getFlags();
		case Maps: return getListOfMaps( m_gametypes[row] );
		case Desc: return toQString( m_gametypes[row].getDesc() );
		default: return QVariant();
	}
}

auto GametypesModel::getListOfMaps( const GametypeDef &def ) const -> QJsonArray {
	QJsonArray result;

	const auto &mapInfoList = def.m_mapInfoList;
	for( const auto &info: mapInfoList ) {
		int minPlayers = 0, maxPlayers = 0;
		if( auto maybeNumOfPlayers = info.numPlayers ) {
			std::tie( minPlayers, maxPlayers ) = *maybeNumOfPlayers;
		}
		result.append( QJsonObject({
			{ "name", toQString( def.m_stringDataStorage[info.fileNameSpanIndex] ) },
			{ "title", toQString( def.m_stringDataStorage[info.fullNameSpanIndex] ) },
			{ "minPlayers", minPlayers },
			{ "maxPlayers", maxPlayers }
		}));
	}

	return result;
}

auto GametypesModel::getBotConfig( int gametypeNum, int mapNum ) const -> QJsonObject {
	// Illegal values could be passed due to some QML lifetime issues, return defaults in this case
	if( (unsigned)gametypeNum >= (unsigned)m_gametypes.size() ) {
		return QJsonObject();
	}
	const auto &def = m_gametypes[gametypeNum];
	if( (unsigned)mapNum >= (unsigned)def.m_mapInfoList.size() ) {
		return QJsonObject();
	}
	const auto &botConfig = def.m_botConfig;
	if( botConfig == GametypeDef::NoBots || botConfig == GametypeDef::ScriptSpawnedBots ) {
		return QJsonObject();
	}

	int number;
	bool fixed;
	if( botConfig == GametypeDef::ExactNumBots ) {
		number = (int)def.m_exactNumBots.value();
		fixed = true;
	} else {
		assert( botConfig == GametypeDef::FixedNumBotsForMap || botConfig == GametypeDef::BestNumBotsForMap );
		const auto [minPlayers, maxPlayers] = def.m_mapInfoList[mapNum].numPlayers.value();
		assert( minPlayers && maxPlayers && minPlayers < maxPlayers );
		number = (int)( ( minPlayers + maxPlayers ) / 2 );
		fixed = ( botConfig == GametypeDef::FixedNumBotsForMap );
	}

	assert( number > 0 );
	return QJsonObject( { { "allowed", true }, { "defined", true }, { "number", number }, { "fixed", fixed } } );
}

class MapExistenceCache {
	using NamesList = wsw::StringSpanStorage<uint16_t, uint16_t>;

	NamesList m_existingMapFileNames;
	NamesList m_existingMapFullNames;
	NamesList m_missingMapFileNames;

	[[nodiscard]]
	static auto findInList( const wsw::StringView &v, const NamesList &namesList ) -> std::optional<unsigned> {
		for( unsigned i = 0; i < namesList.size(); ++i ) {
			if( namesList[i].equalsIgnoreCase( v ) ) {
				return i;
			}
		}
		return std::nullopt;
	}
public:
	[[nodiscard]]
	auto checkExistenceAndGetFullName( const wsw::StringView &mapFileName ) -> std::optional<wsw::StringView> {
		if( const auto maybeExistingIndex = findInList( mapFileName, m_existingMapFileNames ) ) {
			return m_existingMapFullNames[*maybeExistingIndex];
		}
		if( findInList( mapFileName, m_missingMapFileNames ) != std::nullopt ) {
			return std::nullopt;
		}
		assert( mapFileName.isZeroTerminated() );
		if( !ML_FilenameExists( mapFileName.data() ) ) {
			m_missingMapFileNames.add( mapFileName );
			return std::nullopt;
		}
		const wsw::StringView mapFullName( ML_GetFullname( mapFileName.data() ) );
		m_existingMapFileNames.add( mapFileName );
		m_existingMapFullNames.add( mapFullName );
		return mapFullName;
	}
};

GametypesModel::GametypesModel() {
	const wsw::StringView dir( "progs/gametypes"_asView );
	const wsw::StringView ext( ".gtd"_asView );

	wsw::StaticString<MAX_QPATH> path;
	path << dir << '/';

	wsw::fs::SearchResultHolder searchResultHolder;
	const auto maybeCallResult = searchResultHolder.findDirFiles( dir, ext );
	if( !maybeCallResult ) {
		return;
	}

	// Map existence checks require an FS access...
	// Use an existence cache as many maps are shared by gametypes.
	MapExistenceCache mapExistenceCache;

	for( const wsw::StringView &fileName: *maybeCallResult ) {
		assert( fileName.endsWith( ext ) );
		path.erase( dir.length() + 1 );
		path << fileName;

		if( auto maybeDef = GametypeDefParser::exec( path.asView() ) ) {
			auto def( std::move( *maybeDef ) );
			auto &stringStorage = def.m_stringDataStorage;
			auto &mapInfoList = def.m_mapInfoList;
			for( unsigned i = 0; i < mapInfoList.size(); ) {
				const wsw::StringView mapName( stringStorage[mapInfoList[i].fileNameSpanIndex] );
				if( const auto maybeFullName = mapExistenceCache.checkExistenceAndGetFullName( mapName ) ) {
					mapInfoList[i].fullNameSpanIndex = stringStorage.add( *maybeFullName );
					++i;
				} else {
					mapInfoList.erase( mapInfoList.begin() + i );
				}
			}
			if( !mapInfoList.empty() ) {
				def.m_nameSpanIndex = stringStorage.add( fileName.dropRight( ext.length() ) );
				m_gametypes.emplace_back( std::move( def ) );
			}
		}
	}
}

}