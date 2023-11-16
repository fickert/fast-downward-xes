#ifndef BOUNDED_SUBOPTIMAL_SEARCH_REORDERING_DYNAMIC_EXPECTED_EFFORT_SEARCH_H
#define BOUNDED_SUBOPTIMAL_SEARCH_REORDERING_DYNAMIC_EXPECTED_EFFORT_SEARCH_H

#include <functional>
#include <map>
#include <unordered_map>

#include "../floating_point_open_list/best_first_open_list.h"
#include "../suboptimal_search/eager_suboptimal_search.h"
#include "f_hat_min_evaluator.h"

namespace bounded_suboptimal_search {
template <std::size_t N>
class ReorderingDynamicExpectedEffortSearch : public suboptimal_search::EagerSuboptimalSearch<N> {
	using typename suboptimal_search::EagerSuboptimalSearch<N>::EvaluatorValues;

	const bool reopen_closed_nodes;
	const double suboptimality_factor;

	int f_min;

	struct FocalListBucket {
		FocalListBucket(int g, int h, int d, const EvaluatorValues &evaluator_values) : g(g), h(h), d(d), evaluator_values(evaluator_values) {}

		int g;
		int h;
		int d;

		EvaluatorValues evaluator_values;
		std::vector<StateID> state_ids;

		auto operator<(const FocalListBucket &other) const -> bool { return evaluator_values < other.evaluator_values; }
	};

	// g -> h -> d -> bucket
	std::unordered_map<int, std::unordered_map<int, std::unordered_map<int, std::shared_ptr<FocalListBucket>>>> focal_map;
	std::vector<std::shared_ptr<FocalListBucket>> focal_list;

	static constexpr auto compare_focal = [](const auto &lhs, const auto &rhs) {
		return !(*lhs < *rhs);
	};

	void push_focal(EvaluationContext &eval_context, std::function<EvaluatorValues()> get_evaluator_values);
	void push_focal(EvaluationContext &eval_context);
	void push_focal(EvaluationContext &eval_context, const EvaluatorValues &evaluator_values);

	auto top_focal() -> StateID;
	void pop_focal();

	// recompute the evaluator values for each (g, h, d)-bucket and reorder the focal list accordingly
	void reorder_focal();

	// iterable bucket-based open list (ordered by f)
	// each bucket is a heap sorted by high g
	std::map<int, std::vector<std::pair<int, StateID>>> open_list;

	static constexpr auto open_list_compare = [](const std::pair<int, StateID> &lhs, const std::pair<int, StateID> &rhs) {
		return lhs.first < rhs.first;
	};

	std::shared_ptr<Evaluator> heuristic;
	std::shared_ptr<Evaluator> distance;
	std::shared_ptr<Evaluator> f_evaluator;

	std::shared_ptr<floating_point_evaluator::FloatingPointEvaluator> f_hat_evaluator;
	std::shared_ptr<floating_point_evaluator::FloatingPointEvaluator> d_hat_evaluator;
	std::shared_ptr<FHatMinEvaluator> f_hat_min_evaluator;
	floating_point_open_list::BestFirstOpenList<1, StateID> f_hat_list; // open list to keep track of f-hat-min

	auto update_f_hat_min() -> bool;

	void reward_progress() override;

protected:
	using suboptimal_search::EagerSuboptimalSearch<N>::search_space;
	using suboptimal_search::EagerSuboptimalSearch<N>::state_registry;
	using suboptimal_search::EagerSuboptimalSearch<N>::statistics;

	void initialize_extra(EvaluationContext &eval_context) override;

	auto fetch_next_node() -> std::optional<SearchNode> override;
	void insert(EvaluationContext &eval_context, const EvaluatorValues &evaluator_values, StateID state_id, bool preferred) override;

public:
	explicit ReorderingDynamicExpectedEffortSearch(const options::Options &opts);
	virtual ~ReorderingDynamicExpectedEffortSearch() = default;
};
} // namespace bounded_suboptimal_search

#endif
