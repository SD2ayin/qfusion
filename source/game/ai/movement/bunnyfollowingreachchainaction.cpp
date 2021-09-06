#include "bunnyfollowingreachchainaction.h"
#include "movementlocal.h"

enum class TravelTypeClass { Compatible, Trigger, Incompatible };

[[nodiscard]]
static auto classifyTravelType( const aas_reachability_t &reach ) -> TravelTypeClass {
	const auto travelType = reach.traveltype & TRAVELTYPE_MASK;
	if( travelType == TRAVEL_WALK ) {
		return TravelTypeClass::Compatible;
	} else if( travelType == TRAVEL_TELEPORT || travelType == TRAVEL_JUMPPAD || travelType == TRAVEL_ELEVATOR ) {
		return TravelTypeClass::Trigger;
	} else if( travelType == TRAVEL_WALKOFFLEDGE ) {
		// WTF?
		if( reach.start[2] > reach.end[2] ) {
			if( reach.start[2] - reach.end[2] < 40.0f ) {
				return TravelTypeClass::Compatible;
			}
		}
	} else if( travelType == TRAVEL_BARRIERJUMP ) {
		// WTF?
		if( reach.end[2] > reach.start[2] ) {
			if( reach.end[2] - reach.start[2] < 32.0f ) {
				return TravelTypeClass::Compatible;
			}
		}
	}
	return TravelTypeClass::Incompatible;
}

void BunnyFollowingReachChainAction::PlanPredictionStep( PredictionContext *context ) {
	// This action is the first applied action as it is specialized
	// and falls back to other bunnying actions if it cannot be applied.
	if( !GenericCheckIsActionEnabled( context, &m_subsystem->bunnyToBestFloorClusterPointAction ) ) {
		return;
	}

	if( !CheckCommonBunnyHopPreconditions( context ) ) {
		return;
	}

	const auto &__restrict entityPhysicsState = context->movementState->entityPhysicsState;
	const int currentAreaNum = entityPhysicsState.CurrAasAreaNum();
	const int droppedAreaNum = entityPhysicsState.DroppedToFloorAasAreaNum();
	if( currentAreaNum != m_cachedCurrentAreaNum || droppedAreaNum != m_cachedDroppedAreaNum ) {
		m_cachedNextReachNum = 0;
		m_cachedReachNum = context->NextReachNum();
		m_cachedReachPointsToTrigger = false;
		if( m_cachedReachNum ) {
			const auto *const aasReach = AiAasWorld::Instance()->Reachabilities();
			const auto &reach = aasReach[m_cachedReachNum];
			const TravelTypeClass travelTypeClass = classifyTravelType( reach );
			if( travelTypeClass == TravelTypeClass::Incompatible ) {
				context->SetPendingRollback();
				return;
			}
			if( travelTypeClass == TravelTypeClass::Trigger ) {
				m_cachedReachPointsToTrigger = true;
			}
			const int targetAreaNum = context->NavTargetAasAreaNum();
			if( const int nextAreaNum = reach.areanum; nextAreaNum != targetAreaNum ) {
				if( !bot->RouteCache()->PreferredRouteToGoalArea( nextAreaNum, targetAreaNum, &m_cachedNextReachNum ) ) {
					context->SetPendingRollback();
					return;
				}
				if( classifyTravelType( aasReach[m_cachedNextReachNum] ) == TravelTypeClass::Incompatible ) {
					context->SetPendingRollback();
					return;
				}
			}
		}
		m_cachedCurrentAreaNum = currentAreaNum;
		m_cachedDroppedAreaNum = droppedAreaNum;
	}

	int chosenReachNum = -1;
	const auto *const __restrict aasReach = AiAasWorld::Instance()->Reachabilities();
	if( m_cachedReachNum ) {
		const auto &reach = aasReach[m_cachedReachNum];
		if( Distance2DSquared( reach.start, entityPhysicsState.Origin() ) > SQUARE( 32.0f ) ) {
			chosenReachNum = m_cachedReachNum;
		} else if( m_cachedReachPointsToTrigger ) {
			chosenReachNum = m_cachedReachNum;
			// Keep looking at the trigger but make sure we can normalize
			if( Distance2DSquared( reach.start, entityPhysicsState.Origin() ) > SQUARE( 1.0f ) ) {
				chosenReachNum = m_cachedReachNum;
			} else {
				context->SetPendingRollback();
				return;
			}
		} else {
			if( m_cachedNextReachNum ) {
				const auto &nextReach = aasReach[m_cachedNextReachNum];
				if( Distance2DSquared( nextReach.start, entityPhysicsState.Origin() ) > SQUARE( 32.0f ) ) {
					chosenReachNum = m_cachedNextReachNum;
				} else {
					context->SetPendingRollback();
					return;
				}
			}
		}
	}

	assert( chosenReachNum >= -1 );

	Vec3 lookTarget( 0, 0, 0 );
	if( chosenReachNum > 0 ) {
		lookTarget.Set( aasReach[chosenReachNum].start );
		lookTarget.Z() += 32.0f;
		lookTarget.Z() *= Z_NO_BEND_SCALE;
	} else {
		const Vec3 navTargetOrigin( context->NavTargetOrigin() );
		if( navTargetOrigin.SquareDistance2DTo( entityPhysicsState.Origin() ) > SQUARE( 8.0f ) ) {
			navTargetOrigin.CopyTo( lookTarget );
		} else {
			context->SetPendingRollback();
			return;
		}
	}

	if( !SetupBunnyHopping( Vec3( lookTarget - entityPhysicsState.Origin() ), context ) ) {
		return;
	}
}