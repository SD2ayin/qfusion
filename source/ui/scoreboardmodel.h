#ifndef WSW_95e07a0d_69fa_4239_9546_1fa5a5580f55_H
#define WSW_95e07a0d_69fa_4239_9546_1fa5a5580f55_H

#include "scoreboard.h"
#include "../gameshared/gs_public.h"

#include <QAbstractListModel>
#include <QObject>

struct ReplicatedScoreboardData;

namespace wsw::ui {

class ScoreboardModelProxy;

class ScoreboardTeamModel : public QAbstractTableModel, ScoreboardShared {
	Q_OBJECT

	friend class ScoreboardModelProxy;

	ScoreboardModelProxy *const m_proxy;
	const int m_teamListIndex;

	enum Role {
		Kind = Qt::UserRole + 1,
		Value,
		IsGhosting
	};

	ScoreboardTeamModel( ScoreboardModelProxy *proxy, int teamListIndex )
		: m_proxy( proxy ), m_teamListIndex( teamListIndex ) {}

	static inline QVector<int> kValueRoleAsVector { Value };
	static inline QVector<int> kGhostingRoleAsVector { IsGhosting };
	static inline QVector<int> kValueAndGhostingRolesAsVector { Value, IsGhosting };

	Q_PROPERTY( int teamTag MEMBER m_teamListIndex CONSTANT )

	[[nodiscard]]
	auto rowCount( const QModelIndex & ) const -> int override;
	[[nodiscard]]
	auto columnCount( const QModelIndex & ) const -> int override;
	[[nodiscard]]
	auto roleNames() const -> QHash<int, QByteArray> override;
	[[nodiscard]]
	auto data( const QModelIndex &modelIndex, int role ) const -> QVariant override;
};

class ScoreboardSpecsModel : public QAbstractListModel {
	friend class ScoreboardModelProxy;

	ScoreboardModelProxy *const m_proxy;

	enum Role {
		Nickname = Qt::UserRole + 1,
		Ping,
	};

	explicit ScoreboardSpecsModel( ScoreboardModelProxy *proxy ) : m_proxy( proxy ) {}

	[[nodiscard]]
	auto rowCount( const QModelIndex & ) const -> int override;
	[[nodiscard]]
	auto columnCount( const QModelIndex & ) const -> int override;
	[[nodiscard]]
	auto roleNames() const -> QHash<int, QByteArray> override;
	[[nodiscard]]
	auto data( const QModelIndex &modelIndex, int role ) const -> QVariant override;
};

class ScoreboardModelProxy : public QObject, ScoreboardShared {
	Q_OBJECT

	friend class ScoreboardTeamModel;
	friend class ScoreboardSpecsModel;
public:
	enum Display {
		SideBySide,
		ColumnWise,
		Mixed
	};
	Q_ENUM( Display );

	Scoreboard m_scoreboard;

	// Can't declare a plain array due to the type being noncopyable and we don't want to use a dynamic allocation.
	StaticVector<ScoreboardTeamModel, 4> m_teamModelsHolder;
	StaticVector<ScoreboardSpecsModel, 1> m_specsModelHolder;

	cvar_s *m_displayVar { nullptr };
	Display m_display { SideBySide };

	[[nodiscard]]
	auto getDisplay() const { return m_display; }

	void checkDisplayVar();

	wsw::StaticVector<unsigned, MAX_CLIENTS> m_playerIndicesForList[5];

	using PlayerUpdates = Scoreboard::PlayerUpdates;

	void dispatchPlayerRowUpdates( const PlayerUpdates &updates, int team, int rowInTeam, int rowInMixedList );
	void dispatchSpecRowUpdates( const PlayerUpdates &updates, int rowInTeam );
public:
	enum class QmlColumnKind {
		Nickname,
		Clan,
		Score,
		Status,
		Ping,
		Number,
		Glyph,
		Icon
	};
	Q_ENUM( QmlColumnKind );
	static_assert( (int)QmlColumnKind::Nickname == (int)Nickname );
	static_assert( (int)QmlColumnKind::Clan == (int)Clan );
	static_assert( (int)QmlColumnKind::Score == (int)Score );
	static_assert( (int)QmlColumnKind::Status == (int)Status );
	static_assert( (int)QmlColumnKind::Ping == (int)Ping );
	static_assert( (int)QmlColumnKind::Number == (int)Number );
	static_assert( (int)QmlColumnKind::Glyph == (int)Glyph );
	static_assert( (int)QmlColumnKind::Icon == (int)Icon );

	ScoreboardModelProxy();

	Q_SIGNAL void teamReset( int resetTeamTag );

	[[nodiscard]]
	Q_INVOKABLE int getColumnKind( int column ) const;
	[[nodiscard]]
	Q_INVOKABLE QByteArray getColumnTitle( int column ) const;
	[[nodiscard]]
	Q_INVOKABLE int getTitleColumnSpan( int column ) const;
	[[nodiscard]]
	Q_INVOKABLE QByteArray getImageAssetPath( int asset ) const;
	[[nodiscard]]
	Q_INVOKABLE bool isMixedListRowAlpha( int row ) const;
	[[nodiscard]]
	Q_INVOKABLE int getColumnsCount() const { return m_scoreboard.getColumnsCount(); }

	Q_SIGNAL void displayChanged( Display display );
	Q_PROPERTY( Display display READ getDisplay NOTIFY displayChanged );

	[[nodiscard]]
	auto getSpecsModel() -> ScoreboardSpecsModel * { return &m_specsModelHolder[0]; }
	[[nodiscard]]
	auto getPlayersModel() -> ScoreboardTeamModel * { return &m_teamModelsHolder[0]; }
	[[nodiscard]]
	auto getAlphaModel() -> ScoreboardTeamModel * { return &m_teamModelsHolder[1]; }
	[[nodiscard]]
	auto getBetaModel() -> ScoreboardTeamModel * { return &m_teamModelsHolder[2]; }
	[[nodiscard]]
	auto getMixedModel() -> ScoreboardTeamModel * { return &m_teamModelsHolder[3]; }

	void reload();
	void update( const ReplicatedScoreboardData &currData );
};

}

#endif
