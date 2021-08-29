#include "baseaction.h"
#include "movementlocal.h"

void HandleTriggeredJumppadAction::PlanPredictionStep( PredictionContext *context ) {
	if( !GenericCheckIsActionEnabled( context, &DummyAction() ) ) {
		return;
	}

	auto *jumppadMovementState = &context->movementState->jumppadMovementState;
	Assert( jumppadMovementState->IsActive() );

	if( jumppadMovementState->hasEnteredJumppad ) {
		context->cannotApplyAction = true;
		context->actionSuggestedByAction = &FlyUntilLandingAction();
		Debug( "The bot has already processed jumppad trigger touch in the given context state, fly until landing\n" );
		return;
	}

	jumppadMovementState->hasEnteredJumppad = true;

	auto *botInput = &context->record->botInput;
	botInput->Clear();

	const edict_t *jumppadEntity = jumppadMovementState->JumppadEntity();
	float startLandingAtZ = m_subsystem->landOnSavedAreasAction.SaveJumppadLandingAreas( jumppadEntity );
	context->movementState->flyUntilLandingMovementState.Activate( startLandingAtZ );
	// Stop prediction (jumppad triggers are not simulated by Exec() code)
	context->isCompleted = true;
}
