#ifndef EXPANSION_DELAY_EXPANSION_DELAY_EVALUATOR_H
#define EXPANSION_DELAY_EXPANSION_DELAY_EVALUATOR_H

#include "../floating_point_evaluator/floating_point_evaluator.h"
#include "expansion_delay.h"

namespace expansion_delay {

class ExpansionDelayEvaluator : public floating_point_evaluator::FloatingPointEvaluator {
protected:
	const ExpansionDelay &expansion_delay;

	auto compute_value(EvaluationContext &) -> double override { return expansion_delay.get_avg_expansion_delay(); }

public:
	ExpansionDelayEvaluator(const options::Options &opts, const ExpansionDelay &expansion_delay)
		: floating_point_evaluator::FloatingPointEvaluator(opts), expansion_delay(expansion_delay) {}
	~ExpansionDelayEvaluator() override = default;
};

} // namespace expansion_delay

#endif
