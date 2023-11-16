#ifndef BOUNDED_SUBOPTIMAL_SEARCH_DYNAMIC_EXPECTED_EFFORT_SEARCH_H
#define BOUNDED_SUBOPTIMAL_SEARCH_DYNAMIC_EXPECTED_EFFORT_SEARCH_H

#include <functional>
#include <map>

#include "../floating_point_open_list/best_first_open_list.h"
#include "../suboptimal_search/eager_suboptimal_search.h"
#include "f_hat_min_evaluator.h"

namespace bounded_suboptimal_search {
template <std::size_t N>
class DynamicExpectedEffortSearch : public suboptimal_search::EagerSuboptimalSearch<N> {
	const bool reopen_closed_nodes;
	const double suboptimality_factor;

	const double expected_work_error_margin;

	enum class AlternationMode { NONE, F, F_HAT, BOTH } const alternation_mode;

	enum class Queue { DXES, F, F_HAT };

	auto get_active_queue() const -> Queue;

	int f_min;

	floating_point_open_list::BestFirstOpenList<N, StateID> focal_list;

	// iterable bucket-based open list (ordered by f)
	// each bucket is a heap sorted by high g
	std::map<int, std::vector<std::pair<int, StateID>>> open_list;

	static constexpr auto open_list_compare = [](const std::pair<int, StateID> &lhs, const std::pair<int, StateID> &rhs) {
		return lhs.first < rhs.first;
	};

	std::shared_ptr<Evaluator> f_evaluator;

	std::shared_ptr<floating_point_evaluator::FloatingPointEvaluator> f_hat_evaluator;
	std::shared_ptr<floating_point_evaluator::FloatingPointEvaluator> d_hat_evaluator;
	std::shared_ptr<FHatMinEvaluator> f_hat_min_evaluator;
	floating_point_open_list::BestFirstOpenList<1, StateID> f_hat_list; // open list to keep track of f-hat-min

	void update_f_hat_min();

	void reward_progress() override;

protected:
	using typename suboptimal_search::EagerSuboptimalSearch<N>::EvaluatorValues;

	using suboptimal_search::EagerSuboptimalSearch<N>::search_space;
	using suboptimal_search::EagerSuboptimalSearch<N>::state_registry;
	using suboptimal_search::EagerSuboptimalSearch<N>::statistics;

	void initialize_extra(EvaluationContext &eval_context) override;

	auto fetch_next_node() -> std::optional<SearchNode> override;
	void insert(EvaluationContext &eval_context, const EvaluatorValues &evaluator_values, StateID state_id, bool preferred) override;

public:
	explicit DynamicExpectedEffortSearch(const options::Options &opts);
	virtual ~DynamicExpectedEffortSearch() = default;
};
} // namespace bounded_suboptimal_search

#endif
