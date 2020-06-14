#include "materiallocal.h"

auto MaterialIfEvaluator::withError( const char *fmt, ... ) -> std::nullopt_t {
	if( m_hadError ) {
		return std::nullopt;
	}

	char buffer[1024];

	va_list va;
	va_start( va, fmt );
	Q_vsnprintfz( buffer, sizeof( buffer ), fmt, va );
	va_end( va );

	Com_Printf( S_COLOR_RED "%s\n", buffer );

	m_hadError = true;
	return std::nullopt;
}

void MaterialIfEvaluator::warn( const char *fmt, ... ) {
	if( m_hadError ) {
		return;
	}

	char buffer[1024];

	va_list va;
	va_start( va, fmt );
	Q_vsnprintfz( buffer, sizeof( buffer ), fmt, va );
	va_end( va );

	Com_Printf( S_COLOR_YELLOW "%s\n", buffer );
}

void MaterialIfEvaluator::warnBooleanToIntegerConversion( const Value &value, const char *desc, const char *context ) {
	if( value.isInputValue ) {
		warn( "Converting %s value %s to an integer %s", desc, value ? "true" : "false", context );
	} else {
		warn( "Converting %s expression to an integer %s", desc, context );
	}
}

void MaterialIfEvaluator::warnIntegerToBooleanConversion( const Value &value, const char *desc, const char *context ) {
	if( value.isInputValue ) {
		warn( "Converting %s value %d to a boolean %s", desc, (int)value, context );
	} else {
		warn( "Converting %s expression to a boolean %s", desc, context );
	}
}

auto MaterialIfEvaluator::evalUnaryExpr() -> std::optional<Value> {
	std::optional<Tag> maybeTag = nextTokenTag();
	if( !maybeTag ) {
		return std::nullopt;
	}

	Tag tag = *maybeTag;
	if( tag == Tag::Value ) {
		return lastValue();
	}

	if( tag == Tag::UnaryNot ) {
		if( auto maybeValue = evalUnaryExpr() ) {
			Value value = *maybeValue;
			if( !value.isBool ) {
				warnIntegerToBooleanConversion( value, "an integer", "under a NOT operator");
			}
			return Value( !(bool)value );
		}
		return withError( "Expected an unary expression after an unary NOT operator" );
	}

	if( tag == Tag::LParen ) {
		if( auto maybeValue = evalExpr() ) {
			if( auto maybeParenTag = nextTokenTag() ) {
				if( *maybeParenTag == Tag::RParen ) {
					return maybeValue;
				}
				return withError( "Expected a right paren after an expression" );
			}
		}
		return withError( "Expected an expression in parentheses" );
	}

	return withError( "Expected a value, an unary NOT operator or a left paren" );
}

auto MaterialIfEvaluator::evalCmpExpr() -> std::optional<Value> {
	std::optional<Value> maybeLeft = evalUnaryExpr();
	if( !maybeLeft ) {
		return std::nullopt;
	}

	std::optional<Tag> maybeTag = nextTokenTag();
	if( !maybeTag ) {
		return maybeLeft;
	}

	if( *maybeTag != Tag::CmpOp ) {
		ungetToken();
		return maybeLeft;
	}

	const auto op = *lastCmpOp();

	std::optional<Value> maybeRight = evalUnaryExpr();
	if( !maybeRight ) {
		return withError( "Failed to evaluate a right-hand expression" );
	}

	Value left = *maybeLeft;
	if( left.isBool ) {
		warnBooleanToIntegerConversion( left, "a left-hand boolean", "for comparison" );
	}

	Value right = *maybeRight;
	if( right.isBool ) {
		warnBooleanToIntegerConversion( right, "a right-hand boolean", "for comparison" );
	}

	switch( op ) {
		case CmpOp::LS:
			return std::optional( Value( (int)left < (int)right ) );
		case CmpOp::LE:
			return std::optional( Value( (int)left <= (int)right ) );
		case CmpOp::NE:
			return std::optional( Value( (int)left != (int)right ) );
		case CmpOp::EQ:
			return std::optional( Value( (int)left == (int)right ) );
		case CmpOp::GE:
			return std::optional( Value( (int)left >= (int)right ) );
		case CmpOp::GT:
			return std::optional( Value( (int)left > (int)right ) );
	}
}

auto MaterialIfEvaluator::evalLogicExpr() -> std::optional<Value> {
	std::optional<Value> maybeLeft = evalCmpExpr();
	if( !maybeLeft ) {
		return std::nullopt;
	}

	std::optional<Tag> maybeTag = nextTokenTag();
	if( !maybeTag ) {
		return maybeLeft;
	}

	if( *maybeTag != Tag::LogicOp ) {
		ungetToken();
		return maybeLeft;
	}

	const auto op = *lastLogicOp();

	std::optional<Value> maybeRight = evalCmpExpr();
	if( !maybeRight ) {
		return withError( "Missing a right-hand operand of a logical operation" );
	}

	Value left = *maybeLeft;
	if( !left.isBool ) {
		warnIntegerToBooleanConversion( left, "a left-hand integer", "for a logical operation" );
	}

	Value right = *maybeRight;
	if( !right.isBool ) {
		warnIntegerToBooleanConversion( right, "a right-hand integer", "for a logical operation" );
	}

	switch( op ) {
		case LogicOp::And:
			return std::optional( Value( (bool)left && (bool)right ) );
		case LogicOp::Or:
			return std::optional( Value( (bool)left || (bool)right ) );
	}
}

auto MaterialIfEvaluator::exec() -> std::optional<bool> {
	std::optional<Value> maybeValue = evalExpr();
	if( m_hadError ) {
		return std::nullopt;
	}

	if( m_tapeCursor != m_numEntries ) {
		return withError( "Syntax error. Are there missing operators or mismatched parentheses?" );
	}

	if( !maybeValue ) {
		return std::nullopt;
	}

	Value value = *maybeValue;
	if( !value.isBool ) {
		warnIntegerToBooleanConversion( value, "an integer", "as an IF condition" );
	}
	return (bool)value;
}