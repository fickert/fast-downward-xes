#ifndef BOUNDED_SUBOPTIMAL_SEARCH_WEIGHTED_ASTAR_SEARCH_H
#define BOUNDED_SUBOPTIMAL_SEARCH_WEIGHTED_ASTAR_SEARCH_H

#include <optional>

#include "../floating_point_open_list/best_first_open_list.h"
#include "../search_engine.h"
#include "../suboptimal_search/eager_suboptimal_search.h"

namespace bounded_suboptimal_search {
class WeightedAstarSearch : public suboptimal_search::EagerSuboptimalSearch<2> {
	floating_point_open_list::BestFirstOpenList<2, StateID> open_list;

protected:
	void reward_progress() override;

	auto fetch_next_node() -> std::optional<SearchNode> override;
	void insert(EvaluationContext &eval_context, const EvaluatorValues &evaluator_values, StateID state_id, bool preferred) override;

public:
	explicit WeightedAstarSearch(const options::Options &opts);
	virtual ~WeightedAstarSearch() = default;
};
} // namespace bounded_suboptimal_search

#endif
