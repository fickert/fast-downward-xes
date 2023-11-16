#ifndef BOUNDED_SUBOPTIMAL_SEARCH_EXTENDED_DYNAMIC_EXPECTED_EFFORT_SEARCH_H
#define BOUNDED_SUBOPTIMAL_SEARCH_EXTENDED_DYNAMIC_EXPECTED_EFFORT_SEARCH_H

#include <boost/heap/fibonacci_heap.hpp>
#include <forward_list>
#include <functional>
#include <map>

#include "../expansion_delay/expansion_delay.h"
#include "../floating_point_open_list/best_first_open_list.h"
#include "../suboptimal_search/eager_suboptimal_search.h"
#include "f_hat_min_evaluator.h"

namespace bounded_suboptimal_search {
template <std::size_t N>
class ExtendedDynamicExpectedEffortSearch : public suboptimal_search::EagerSuboptimalSearch<N> {
	const bool reopen_closed_nodes;
	const double suboptimality_factor;

	const double expected_work_error_margin;

	int f_min;

	static constexpr auto open_list_compare = [](const std::pair<int, StateID> &lhs, const std::pair<int, StateID> &rhs) {
		return lhs.first < rhs.first;
	};
	using OpenListBucketType = boost::heap::fibonacci_heap<std::pair<int, StateID>, boost::heap::compare<decltype(open_list_compare)>>;
	using OpenListHandleType = typename OpenListBucketType::handle_type;
	// iterable bucket-based open list (ordered by f)
	// each bucket is a heap sorted by high g
	std::map<int, OpenListBucketType> open_list;
	// each open state may have one or more nodes in the open list, this yields the corresponding handles
	std::unordered_map<StateID, std::forward_list<std::pair<int, OpenListHandleType>>> open_list_handles;

	floating_point_open_list::BestFirstOpenList<N, StateID> focal_list;

	const std::shared_ptr<heuristic_error::HeuristicError> h_error;
	std::shared_ptr<expansion_delay::ExpansionDelay> expansion_delay;
	PerStateInformation<int> open_list_insertion_time;

	std::shared_ptr<floating_point_evaluator::FloatingPointEvaluator> expected_work_evaluator;
	std::shared_ptr<Evaluator> f_evaluator;
	std::shared_ptr<floating_point_evaluator::FloatingPointEvaluator> f_hat_evaluator;
	std::shared_ptr<floating_point_evaluator::FloatingPointEvaluator> d_hat_evaluator;
	std::shared_ptr<FHatMinEvaluator> f_hat_min_evaluator;
	floating_point_open_list::BestFirstOpenList<1, StateID> f_hat_list; // open list to keep track of f-hat-min

	void update_f_hat_min();

	void reward_progress() override;

	// statistics
	int best_f_expansions;

protected:
	using typename suboptimal_search::EagerSuboptimalSearch<N>::EvaluatorValues;

	using suboptimal_search::EagerSuboptimalSearch<N>::search_space;
	using suboptimal_search::EagerSuboptimalSearch<N>::state_registry;
	using suboptimal_search::EagerSuboptimalSearch<N>::statistics;

	void initialize_extra(EvaluationContext &eval_context) override;

	auto fetch_next_node() -> std::optional<SearchNode> override;
	void insert(EvaluationContext &eval_context, const EvaluatorValues &evaluator_values, StateID state_id, bool preferred) override;

public:
	explicit ExtendedDynamicExpectedEffortSearch(const options::Options &opts, std::shared_ptr<expansion_delay::ExpansionDelay> expansion_delay);
	virtual ~ExtendedDynamicExpectedEffortSearch() = default;

	void print_statistics() const override;
};
} // namespace bounded_suboptimal_search

#endif
