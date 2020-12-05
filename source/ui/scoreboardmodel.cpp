#include "scoreboardmodel.h"
#include "local.h"

namespace wsw::ui {

// TODO: Share this with the server browser
[[nodiscard]]
static auto formatPing( unsigned ping ) -> QString {
	// TODO: Optimize (we can compose styled text based on ping numeric value)
	wsw::StaticString<16> buffer;
	if( ping < 50 ) {
		buffer << wsw::StringView( S_COLOR_GREEN );
	} else if( ping < 100 ) {
		buffer << wsw::StringView( S_COLOR_YELLOW );
	} else if( ping < 150 ) {
		buffer << wsw::StringView( S_COLOR_ORANGE );
	} else {
		buffer << wsw::StringView( S_COLOR_RED );
	}
	buffer << ping;
	return toStyledText( buffer.asView() );
}

auto ScoreboardTeamModel::rowCount( const QModelIndex & ) const -> int {
	return (int)m_proxy->m_playerNumsForList[m_teamListIndex].size();
}

auto ScoreboardTeamModel::columnCount( const QModelIndex & ) const -> int {
	return (int)m_proxy->m_scoreboard.getColumnCount();
}

auto ScoreboardTeamModel::roleNames() const -> QHash<int, QByteArray> {
	return { { Kind, "kind" }, { Value, "value" } };
}

auto ScoreboardTeamModel::data( const QModelIndex &modelIndex, int role ) const -> QVariant {
	if( !modelIndex.isValid() ) {
		return QVariant();
	}
	const auto &scb = m_proxy->m_scoreboard;
	const auto &nums = m_proxy->m_playerNumsForList;
	const auto column = (unsigned)modelIndex.column();
	if( column >= scb.getColumnCount() ) {
		return QVariant();
	}
	const auto row = (unsigned)modelIndex.row();
	if( row >= nums[m_teamListIndex].size() ) {
		return QVariant();
	}
	if( role == Kind ) {
		return scb.getColumnKind( column );
	}
	if( role != Value ) {
		return QVariant();
	}
	const auto playerNum = nums[m_teamListIndex][row];
	// TODO: This is awkward a bit
	switch( scb.getColumnKind( column ) ) {
		case Nickname: return toStyledText( scb.getPlayerNameForColumn( playerNum, column ) );
		case Clan: return toStyledText( scb.getPlayerClanForColumn( playerNum, column ) );
		case Score: return scb.getPlayerScoreForColumn( playerNum, column );
		case Ping: return formatPing( scb.getPlayerPingForColumn( playerNum, column ) );
		case Number: return scb.getPlayerNumberForColumn( playerNum, column );
		case Icon: return scb.getPlayerIconForColumn( playerNum, column );
	}
	throw std::logic_error( "Unreachable" );
}

auto ScoreboardSpecsModel::rowCount( const QModelIndex & ) const -> int {
	return (int)m_proxy->m_playerNumsForList[TEAM_SPECTATOR].size();
}

auto ScoreboardSpecsModel::columnCount( const QModelIndex & ) const -> int {
	return (int)m_proxy->m_scoreboard.getColumnCount();
}

auto ScoreboardSpecsModel::roleNames() const -> QHash<int, QByteArray> {
	return { { Nickname, "name" }, { Ping, "Ping" } };
}

auto ScoreboardSpecsModel::data( const QModelIndex &modelIndex, int role ) const -> QVariant {
	if( !modelIndex.isValid() ) {
		return QVariant();
	}
	const auto &nums = m_proxy->m_playerNumsForList[TEAM_SPECTATOR];
	const auto row = (unsigned)modelIndex.row();
	if( row >= nums.size() ) {
		return QVariant();
	}
	const auto &scb = m_proxy->m_scoreboard;
	const auto playerNum = nums[row];
	// TODO: These assumptions about column numbers are risky. This is just to get the stuff working.
	if( role == Ping ) {
		const auto pingColumn = m_proxy->m_scoreboard.getColumnCount() - 1;
		return formatPing( scb.getPlayerPingForColumn( playerNum, pingColumn ) );
	}
	if( role == Nickname ) {
		const auto nameColumn = 0;
		return toStyledText( scb.getPlayerNameForColumn( playerNum, nameColumn ) );
	}
	return QVariant();
}

void ScoreboardModelProxy::reload() {
	m_scoreboard.reload();
}

void ScoreboardModelProxy::handleConfigString( unsigned configStringIndex, const wsw::StringView &string ) {
	m_scoreboard.handleConfigString( configStringIndex, string );
}

auto ScoreboardModelProxy::getColumnKind( int column ) const -> int {
	return (int)m_scoreboard.getColumnKind( (unsigned)column );
}

auto ScoreboardModelProxy::getImageAssetPath( int asset ) const -> QByteArray {
	if( auto maybePath = m_scoreboard.getImageAssetPath( (unsigned)asset ) ) {
		return QByteArray( "image://wsw/" ) + QByteArray( maybePath->data(), maybePath->size() );
	}
	return QByteArray();
}

bool ScoreboardModelProxy::isMixedListRowAlpha( int row ) const {
	const auto &nums = std::end( m_playerNumsForList )[-1];
	assert( (unsigned)row < nums.size() );
	return m_scoreboard.getPlayerTeam( nums[row] ) == TEAM_ALPHA;
}

ScoreboardModelProxy::ScoreboardModelProxy() {
	new( m_specsModelHolder.unsafe_grow_back() )ScoreboardSpecsModel( this );
	new( m_teamModelsHolder.unsafe_grow_back() )ScoreboardTeamModel( this, TEAM_PLAYERS );
	new( m_teamModelsHolder.unsafe_grow_back() )ScoreboardTeamModel( this, TEAM_ALPHA );
	new( m_teamModelsHolder.unsafe_grow_back() )ScoreboardTeamModel( this, TEAM_BETA );
	new( m_teamModelsHolder.unsafe_grow_back() )ScoreboardTeamModel( this, TEAM_BETA + 1 );
}

void ScoreboardModelProxy::dispatchPlayerRowUpdates( const PlayerUpdates &updates, int team,
													 int rowInTeam, int rowInMixedList ) {
	assert( team >= TEAM_PLAYERS && team <= TEAM_BETA );
	const QVector<int> &changedRoles = ScoreboardTeamModel::kValueRoleAsVector;
	QAbstractTableModel *const teamModel = &m_teamModelsHolder[team - 1];
	QAbstractTableModel *const mixedModel = ( team != TEAM_PLAYERS ) ? m_teamModelsHolder.end() - 1 : nullptr;

	for( unsigned i = 0; i < m_scoreboard.getColumnCount(); ++i ) {
		// TODO: Check updates for whether the column has been really changed
		QModelIndex teamModelIndex( teamModel->index( rowInTeam, (int)i ) );
		teamModel->dataChanged( teamModelIndex, teamModelIndex, changedRoles );
		if( mixedModel ) {
			QModelIndex mixedModelIndex( mixedModel->index( rowInMixedList, (int)i ) );
			mixedModel->dataChanged( mixedModelIndex, mixedModelIndex, changedRoles );
		}
	}
}

void ScoreboardModelProxy::dispatchSpecRowUpdates( const PlayerUpdates &updates, int rowInTeam ) {
	ScoreboardSpecsModel &model = m_specsModelHolder.front();
	// TODO: Check roles that got changed, build and supply updated roles vector
	const QModelIndex modelIndex( model.index( rowInTeam ) );
	model.dataChanged( modelIndex, modelIndex );
}

void ScoreboardModelProxy::update( const ReplicatedScoreboardData &currData ) {
	Scoreboard::PlayerUpdatesList playerUpdates;
	Scoreboard::TeamUpdatesList teamUpdates;
	if( !m_scoreboard.checkUpdates( currData, playerUpdates, teamUpdates ) ) {
		return;
	}

	for( auto &nums : m_playerNumsForList ) {
		nums.clear();
	}

	// We should update player nums first so fully reset models get a correct data from the very beginning

	using PlayerIndicesTable = std::array<uint8_t, MAX_CLIENTS>;
	PlayerIndicesTable listPlayerTables[5];

	// TODO: Limit by gs.maxclients
	for( unsigned playerNum = 0; playerNum < MAX_CLIENTS; ++playerNum ) {
		const auto teamNum = m_scoreboard.getPlayerTeam( playerNum );
		// TODO: How do we separate specs from empty client slots?
		auto &teamNums = m_playerNumsForList[teamNum];
		auto &teamIndices = listPlayerTables[teamNum];
		teamIndices[playerNum] = (uint8_t)teamNums.size();
		teamNums.push_back( playerNum );
		if( teamNum == TEAM_ALPHA || teamNum == TEAM_BETA ) {
			auto &mixedNums = m_playerNumsForList[TEAM_BETA + 1];
			auto &mixedIndices = listPlayerTables[TEAM_BETA + 1];
			mixedIndices[playerNum] = (uint8_t)mixedNums.size();
			mixedNums.push_back( playerNum );
		}
	}

	bool wasTeamReset[4] { false, false, false, false };

	// TODO: Use destructuring for fields
	for( const auto &teamUpdate: teamUpdates ) {
		// TODO: Handle other team updates (not only player changes) as well
		if( !teamUpdate.players ) {
			continue;
		}
		wasTeamReset[teamUpdate.team] = true;
		// Forcing a full reset is the easiest approach.
		if( teamUpdate.team == TEAM_SPECTATOR ) {
			auto &model = m_specsModelHolder.front();
			model.beginResetModel();
			model.endResetModel();
		} else {
			auto &model = m_teamModelsHolder[teamUpdate.team - 1];
			model.beginResetModel();
			model.endResetModel();
		}
	}

	if( wasTeamReset[TEAM_ALPHA ] | wasTeamReset[TEAM_BETA] ) {
		auto &model = std::end( m_teamModelsHolder )[-1];
		model.beginResetModel();
		model.endResetModel();
	}

	for( const auto &playerUpdate: playerUpdates ) {
		const auto playerNum = playerUpdate.playerNum;
		const auto teamNum = m_scoreboard.getPlayerTeam( playerNum );
		if( wasTeamReset[teamNum] ) {
			continue;
		}
		const auto &teamIndicesTable = listPlayerTables[teamNum];
		const auto rowInTeam = (unsigned)teamIndicesTable[playerNum];
		assert( rowInTeam < m_playerNumsForList[teamNum].size() );
		if( teamNum == TEAM_SPECTATOR ) {
			dispatchSpecRowUpdates( playerUpdate, rowInTeam );
			continue;
		}
		const auto &mixedIndicesTable = listPlayerTables[teamNum];
		const auto rowInMixedList = (unsigned)mixedIndicesTable[playerNum];
		dispatchPlayerRowUpdates( playerUpdate, teamNum, rowInTeam, rowInMixedList );
	}
}

}
