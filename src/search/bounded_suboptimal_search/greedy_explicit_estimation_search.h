#ifndef BOUNDED_SUBOPTIMAL_SEARCH_GREEDY_EXPLICIT_ESTIMATION_SEARCH_H
#define BOUNDED_SUBOPTIMAL_SEARCH_GREEDY_EXPLICIT_ESTIMATION_SEARCH_H

#include <boost/heap/fibonacci_heap.hpp>
#include <map>

#include "../floating_point_open_list/best_first_open_list.h"
#include "../suboptimal_search/eager_suboptimal_search.h"

namespace bounded_suboptimal_search {
template <bool tie_breaking>
class GreedyExplicitEstimationSearch : public suboptimal_search::EagerSuboptimalSearch<3> {
	static constexpr auto N = 3;

	static constexpr auto D_HAT_INDEX = 0;
	static constexpr auto F_HAT_INDEX = 1;
	static constexpr auto F_INDEX = 2;

	using typename suboptimal_search::EagerSuboptimalSearch<N>::EvaluatorValues;

	void reward_progress() override;

	const double suboptimality_factor;
	std::shared_ptr<Evaluator> heuristic;
	std::shared_ptr<floating_point_evaluator::FloatingPointEvaluator> d_hat_evaluator;
	std::shared_ptr<floating_point_evaluator::FloatingPointEvaluator> f_hat_evaluator;

	int f_min;

	void update_focal(int current_f_min);

protected:
	using suboptimal_search::EagerSuboptimalSearch<N>::search_space;
	using suboptimal_search::EagerSuboptimalSearch<N>::state_registry;
	using suboptimal_search::EagerSuboptimalSearch<N>::statistics;

	auto fetch_next_node() -> std::optional<SearchNode> override;
	void insert(EvaluationContext &eval_context, const EvaluatorValues &evaluator_values, StateID state_id, bool preferred) override;

	// ordered by d-hat, ties are broken by f-hat and h
	using FocalListValueType = std::pair<std::conditional_t<tie_breaking, std::tuple<double, double, int>, double>, StateID>;
	static constexpr auto focal_list_compare = [](const FocalListValueType &lhs, const FocalListValueType &rhs) {
		return lhs.first > rhs.first;
	};

	boost::heap::fibonacci_heap<FocalListValueType, boost::heap::compare<decltype(focal_list_compare)>> focal_list;
	using FocalListHandleType = typename decltype(focal_list)::handle_type;

	using NonTieBreakingOpenListValueType = std::pair<StateID, std::optional<FocalListHandleType>>;
	using TieBreakingOpenListValueType = std::tuple<int, StateID, std::optional<FocalListHandleType>>; // includes the node's g-value
	using OpenListValueType = std::conditional_t<tie_breaking, TieBreakingOpenListValueType, NonTieBreakingOpenListValueType>;
	// if tie breaking is enabled, the vector is a heap sorted by high g
	std::map<double, std::vector<OpenListValueType>> open_list;

	static constexpr auto tie_breaking_open_list_compare = [](const TieBreakingOpenListValueType &lhs, const TieBreakingOpenListValueType &rhs) {
		return std::get<int>(lhs) < std::get<int>(rhs);
	};

	auto open_list_top() -> std::optional<SearchNode>;
	void open_list_pop();
	void open_list_push(EvaluationContext &eval_context, const EvaluatorValues &evaluator_values, StateID state_id, std::optional<FocalListHandleType> focal_handle);

	// ordered by f
	floating_point_open_list::BestFirstOpenList<tie_breaking ? 2 : 1, StateID> cleanup_list;

public:
	explicit GreedyExplicitEstimationSearch(const options::Options &opts);
	virtual ~GreedyExplicitEstimationSearch() = default;
};
} // namespace bounded_suboptimal_search

#endif
