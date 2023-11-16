#ifndef BOUNDED_SUBOPTIMAL_SEARCH_REMAINING_EXPANSIONS_EVALUATOR_H
#define BOUNDED_SUBOPTIMAL_SEARCH_REMAINING_EXPANSIONS_EVALUATOR_H

#include "../floating_point_evaluator/floating_point_evaluator.h"

namespace bounded_suboptimal_search {
class RemainingExpansionsEvaluator : public floating_point_evaluator::FloatingPointEvaluator {
protected:
	std::shared_ptr<FloatingPointEvaluator> distance;
	std::shared_ptr<FloatingPointEvaluator> expansion_delay;

protected:
	auto compute_value(EvaluationContext &eval_context) -> double override {
		const auto distance_value = distance->compute_result(eval_context);
		if (is_dead_end(distance_value))
			return DEAD_END;
		const auto expansion_delay_value = expansion_delay->compute_result(eval_context);
		assert(!is_dead_end(expansion_delay_value));
		return distance_value * expansion_delay_value;
	}

public:
	RemainingExpansionsEvaluator(std::shared_ptr<FloatingPointEvaluator> distance, std::shared_ptr<FloatingPointEvaluator> expansion_delay)
		: FloatingPointEvaluator(get_default_options()), distance(distance), expansion_delay(expansion_delay) {}
};
} // namespace bounded_suboptimal_search

#endif
