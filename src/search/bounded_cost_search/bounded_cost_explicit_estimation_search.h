#ifndef BOUNDED_COST_SEARCH_BOUNDED_COST_EXPLICIT_ESTIMATION_SEARCH_H
#define BOUNDED_COST_SEARCH_BOUNDED_COST_EXPLICIT_ESTIMATION_SEARCH_H

#include <memory>

#include "../floating_point_open_list/focal_open_list.h"
#include "eager_bounded_cost_search.h"

namespace options {
class Options;
} // namespace options

namespace floating_point_evaluator {
class FloatingPointEvaluator;
}

namespace floating_point_open_list {
template <std::size_t N, class T>
class FloatingPointOpenList;
}

namespace bounded_cost_search {
template <std::size_t N>
class BoundedCostExplicitEstimationSearch : public EagerBoundedCostSearch<N> {
	std::shared_ptr<floating_point_evaluator::FloatingPointEvaluator> f_hat_evaluator;
	std::shared_ptr<floating_point_evaluator::FloatingPointEvaluator> d_hat_evaluator;

protected:
	using typename EagerBoundedCostSearch<N>::EvaluatorValues;
	using typename EagerBoundedCostSearch<N>::OpenListCreationFunction;

	void insert(EvaluationContext &eval_context, const EvaluatorValues &evaluator_values, StateID state_id, bool preferred) override;

	static const OpenListCreationFunction create_focal_open_list;

public:
	explicit BoundedCostExplicitEstimationSearch(const options::Options &opts);
	virtual ~BoundedCostExplicitEstimationSearch() = default;
};

template <std::size_t N>
class BoundedCostExplicitEstimationPercentileSearch : public EagerBoundedCostSearch<N> {
	std::shared_ptr<floating_point_evaluator::FloatingPointEvaluator> d_hat_evaluator;
	std::shared_ptr<floating_point_evaluator::FloatingPointEvaluator> probability_evaluator;
	const double focal_threshold;

protected:
	using typename EagerBoundedCostSearch<N>::EvaluatorValues;
	using typename EagerBoundedCostSearch<N>::OpenListCreationFunction;

	void insert(EvaluationContext &eval_context, const EvaluatorValues &evaluator_values, StateID state_id, bool preferred) override;

	static const OpenListCreationFunction create_focal_open_list;

public:
	explicit BoundedCostExplicitEstimationPercentileSearch(const options::Options &opts);
	virtual ~BoundedCostExplicitEstimationPercentileSearch() = default;
};
} // namespace bounded_cost_search

#endif
