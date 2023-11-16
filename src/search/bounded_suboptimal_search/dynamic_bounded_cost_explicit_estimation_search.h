#ifndef BOUNDED_SUBOPTIMAL_SEARCH_DYNAMIC_BOUNDED_COST_EXPLICIT_ESTIMATION_SEARCH_H
#define BOUNDED_SUBOPTIMAL_SEARCH_DYNAMIC_BOUNDED_COST_EXPLICIT_ESTIMATION_SEARCH_H

#include <boost/heap/fibonacci_heap.hpp>
#include <map>

#include "../floating_point_open_list/best_first_open_list.h"
#include "../suboptimal_search/eager_suboptimal_search.h"

namespace bounded_suboptimal_search {
template <bool tie_breaking>
class DynamicBoundedCostExplicitEstimationSearch : public suboptimal_search::EagerSuboptimalSearch<tie_breaking ? 3 : 1> {
	static constexpr auto N = tie_breaking ? 3 : 1;

	using typename suboptimal_search::EagerSuboptimalSearch<N>::EvaluatorValues;

	void reward_progress() override;

	int f_min;

	// iterable bucket-based open list (ordered by f)
	// each bucket is a heap sorted by high g
	std::map<int, std::vector<std::pair<int, StateID>>> open_list;

	static constexpr auto open_list_compare = [](const std::pair<int, StateID> &lhs, const std::pair<int, StateID> &rhs) {
		return lhs.first < rhs.first;
	};

	const double suboptimality_factor;
	std::shared_ptr<Evaluator> heuristic;
	std::shared_ptr<Evaluator> f_evaluator;
	std::shared_ptr<floating_point_evaluator::FloatingPointEvaluator> d_hat_evaluator;

	// ordered by d-hat, ties are broken by f-hat and h
	floating_point_open_list::BestFirstOpenList<N, StateID> focal_list;

	void update_focal();

protected:
	using suboptimal_search::EagerSuboptimalSearch<N>::search_space;
	using suboptimal_search::EagerSuboptimalSearch<N>::state_registry;
	using suboptimal_search::EagerSuboptimalSearch<N>::statistics;

	auto fetch_next_node() -> std::optional<SearchNode> override;
	void insert(EvaluationContext &eval_context, const EvaluatorValues &evaluator_values, StateID state_id, bool preferred) override;

public:
	explicit DynamicBoundedCostExplicitEstimationSearch(const options::Options &opts);
	virtual ~DynamicBoundedCostExplicitEstimationSearch() = default;
};
} // namespace bounded_suboptimal_search

#endif
