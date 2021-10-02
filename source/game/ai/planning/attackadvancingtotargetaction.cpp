#include "planninglocal.h"
#include "../bot.h"

void AttackAdvancingToTargetActionRecord::Activate() {
	BotActionRecord::Activate();
	assert( m_selectedNavEntity.isSame( Self()->GetSelectedNavEntity() ) );
	// Let's provide a spot origin that matches the nav entity
	// (we should use spots since to conform to the rest of combat actions).
	Self()->SetNavTarget( m_selectedNavEntity.navEntity );
	Self()->GetMiscTactics().Clear();
	Self()->GetMiscTactics().PreferAttackRatherThanRun();
	// This flag affects weapons choice. This action is very likely to behave similar to retreating.
	Self()->GetMiscTactics().willRetreat = true;
}

void AttackAdvancingToTargetActionRecord::Deactivate() {
	Self()->GetMiscTactics().Clear();
	Self()->ResetNavTarget();
	BotActionRecord::Deactivate();
}

AiActionRecord::Status AttackAdvancingToTargetActionRecord::UpdateStatus( const WorldState &currWorldState ) {
	if( !m_selectedNavEntity.isSame( Self()->GetSelectedNavEntity() ) ) {
		Debug( "The actual selected nav entity differs from the stored one\n" );
		return INVALID;
	}

	const auto &selectedEnemies = Self()->GetSelectedEnemies();
	if( !selectedEnemies.AreValid() || selectedEnemies.InstanceId() != selectedEnemiesInstanceId ) {
		Debug( "The currently selected enemies are not valid or have been updated\n" );
		return INVALID;
	}

	if( currWorldState.HasJustPickedGoalItemVar() ) {
		Debug( "The bot has just picked a goal item, should deactivate\n" );
		return COMPLETED;
	}

	return VALID;
}

PlannerNode *AttackAdvancingToTargetAction::TryApply( const WorldState &worldState ) {
	if( worldState.BotOriginVar().Ignore() ) {
		Debug( "Bot origin is ignored in the given world state\n" );
		return nullptr;
	}

	// Prevent excessive fruitless branching
	constexpr float distanceError = OriginVar::MAX_ROUNDING_SQUARE_DISTANCE_ERROR;
	constexpr float squareDistanceError = distanceError * distanceError;
	if( worldState.BotOriginVar().Value().SquareDistanceTo( Self()->Origin() ) > squareDistanceError ) {
		Debug( "This action is applicable only for the real bot origin\n" );
		return nullptr;
	}
	if( worldState.EnemyOriginVar().Ignore() ) {
		Debug( "Enemy is ignored in the given world state\n" );
		return nullptr;
	}

	if( worldState.HealthVar().Ignore() || worldState.ArmorVar().Ignore() ) {
		Debug( "Health or armor are ignored in the given world state\n" );
		return nullptr;
	}

	if( worldState.NavTargetOriginVar().Ignore() ) {
		Debug( "Nav target is ignored in the given world state\n" );
		return nullptr;
	}

	// Check whether the nav target is based on the selected nav entity
	// TODO: It could look much better in the planned flexible world state interface

	const std::optional<SelectedNavEntity> &maybeSelectedNavEntity = Self()->GetSelectedNavEntity();
	if( !maybeSelectedNavEntity ) {
		Debug( "The currently selected nav entity is invalid or is empty\n" );
		return nullptr;
	}

	const SelectedNavEntity &selectedNavEntity = *maybeSelectedNavEntity;
	const NavEntity *navEntity = selectedNavEntity.navEntity;
	if( navEntity->Origin().SquareDistance2DTo( worldState.NavTargetOriginVar().Value() ) > squareDistanceError ) {
		Debug( "The nav target var value does not match selected nav entity\n" );
		return nullptr;
	}

	if( worldState.CanHitEnemyVar().Ignore() ) {
		Debug( "Can bot hit enemy is ignored in the given world state\n" );
		return nullptr;
	}
	if( !worldState.CanHitEnemyVar() ) {
		Debug( "Bot can't hit enemy in the given world state\n" );
		return nullptr;
	}

	const float offensiveness = Self()->GetEffectiveOffensiveness();
	float actionPenalty = 0.5f + 1.0f * offensiveness;

	const Vec3 botOrigin( worldState.BotOriginVar().Value() );
	const Vec3 navTargetOrigin( worldState.NavTargetOriginVar().Value() );
	int travelTimeMillis = Self()->CheckTravelTimeMillis( botOrigin, navTargetOrigin );
	if( !travelTimeMillis ) {
		Debug( "Can't find a travel time from bot origin to the nav target origin\n" );
		return nullptr;
	}

	const unsigned selectedEnemiesInstanceId = Self()->GetSelectedEnemies().InstanceId();
	PlannerNodePtr plannerNode = NewNodeForRecord( pool.New( Self(), selectedNavEntity, selectedEnemiesInstanceId ) );
	if( !plannerNode ) {
		Debug( "Can't allocate planner node\n" );
		return nullptr;
	}

	// TODO: We need much more sophisticated cost calculations/applicability checks for this action!
	plannerNode.Cost() = travelTimeMillis * actionPenalty;

	plannerNode.WorldState() = worldState;
	// It is unlikely that bot is going to really have a positional advantage,
	// but that's what the terminal KillEnemy action expects
	plannerNode.WorldState().HasPositionalAdvantageVar().SetValue( true ).SetIgnore( false );
	unsigned similarWorldStateInstanceId = Self()->NextSimilarWorldStateInstanceId();
	plannerNode.WorldState().SimilarWorldStateInstanceIdVar().SetValue( similarWorldStateInstanceId ).SetIgnore( false );
	plannerNode.WorldState().BotOriginVar().SetValue( navTargetOrigin );

	return plannerNode.ReleaseOwnership();
}