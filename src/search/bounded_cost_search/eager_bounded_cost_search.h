#ifndef BOUNDED_COST_SEARCH_EAGER_BOUNDED_COST_SEARCH_H
#define BOUNDED_COST_SEARCH_EAGER_BOUNDED_COST_SEARCH_H

#include "../floating_point_open_list/floating_point_open_list.h"
#include "../suboptimal_search/eager_suboptimal_search.h"

namespace bounded_cost_search {
// N is the number of evaluators
template <std::size_t N>
class EagerBoundedCostSearch : public suboptimal_search::EagerSuboptimalSearch<N> {
	static_assert(N > 0, "There must be at least one evaluator.");

	const bool initialize_error_with_cost_bound;
	std::shared_ptr<Evaluator> distance_evaluator;

	void reward_progress() override;

protected:
	using typename suboptimal_search::EagerSuboptimalSearch<N>::EvaluatorValues;
	
	using suboptimal_search::EagerSuboptimalSearch<N>::bound;
	using suboptimal_search::EagerSuboptimalSearch<N>::heuristic_error;
	using suboptimal_search::EagerSuboptimalSearch<N>::search_space;
	using suboptimal_search::EagerSuboptimalSearch<N>::state_registry;
	using suboptimal_search::EagerSuboptimalSearch<N>::statistics;

	void initialize_heuristic_error(EvaluationContext &eval_context) override;

	auto fetch_next_node() -> std::optional<SearchNode> override;
	void insert(EvaluationContext &eval_context, const EvaluatorValues &evaluator_values, StateID state_id, bool preferred) override;

	using OpenListType = floating_point_open_list::FloatingPointOpenList<N, StateID>;
	using OpenListCreationFunction = std::function<std::unique_ptr<OpenListType>()>;
	static const OpenListCreationFunction create_best_first_open_list;
	static auto create_open_list(const options::Options &opts, OpenListCreationFunction create_default_open_list = create_best_first_open_list)
			-> std::unique_ptr<OpenListType>;

	std::unique_ptr<OpenListType> open_list;

public:
	explicit EagerBoundedCostSearch(const options::Options &opts);
	EagerBoundedCostSearch(const options::Options &opts, OpenListCreationFunction create_default_open_list);
	virtual ~EagerBoundedCostSearch() = default;
};

extern void add_options_to_parser(options::OptionParser &parser);
} // namespace bounded_cost_search

#endif
