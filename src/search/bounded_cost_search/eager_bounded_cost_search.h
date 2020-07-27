#ifndef BOUNDED_COST_SEARCH_EAGER_BOUNDED_COST_SEARCH_H
#define BOUNDED_COST_SEARCH_EAGER_BOUNDED_COST_SEARCH_H

#include <array>
#include <functional>
#include <memory>
#include <vector>

#include "../floating_point_open_list/best_first_open_list.h"
#include "../floating_point_open_list/floating_point_open_list.h"
#include "../search_engine.h"

class PruningMethod;

namespace options {
class OptionParser;
class Options;
} // namespace options

namespace floating_point_evaluator {
class FloatingPointEvaluator;
}

namespace heuristic_error {
class HeuristicError;
}

namespace bounded_cost_search {
// N is the number of evaluators
template <std::size_t N>
class EagerBoundedCostSearch : public SearchEngine {
	static_assert(N > 0, "There must be at least one evaluator.");

	const bool reopen_closed_nodes;
	const bool initialize_error_with_cost_bound;

	std::array<std::shared_ptr<floating_point_evaluator::FloatingPointEvaluator>, N> evaluators;
	std::vector<std::shared_ptr<Evaluator>> preferred_operator_evaluators;
	std::shared_ptr<Evaluator> distance_evaluator;
	std::shared_ptr<PruningMethod> pruning_method;
	std::vector<std::shared_ptr<heuristic_error::HeuristicError>> heuristic_error;

	double best_evaluator_value;
	auto check_progress(double evaluator_value) -> bool;
	void reward_progress();

protected:
	void initialize() override;
	SearchStatus step() override;

	using OpenListType = floating_point_open_list::FloatingPointOpenList<N, StateID>;
	using EvaluatorValues = std::array<double, N>;

	auto compute_results(EvaluationContext &eval_context) -> EvaluatorValues;
	auto is_dead_end(const EvaluatorValues &values) -> bool;

	using OpenListCreationFunction = std::function<std::unique_ptr<OpenListType>()>;
	static const OpenListCreationFunction create_best_first_open_list;
	static auto create_open_list(const options::Options &opts, OpenListCreationFunction create_default_open_list = create_best_first_open_list)
			-> std::unique_ptr<OpenListType>;

	std::unique_ptr<OpenListType> open_list;

	virtual void insert(EvaluationContext &eval_context, const EvaluatorValues &evaluator_values, StateID state_id, bool preferred);

public:
	explicit EagerBoundedCostSearch(const options::Options &opts);
	EagerBoundedCostSearch(const options::Options &opts, OpenListCreationFunction create_default_open_list);
	virtual ~EagerBoundedCostSearch() = default;

	void print_statistics() const override;

	void dump_search_space() const;
};

extern void add_options_to_parser(options::OptionParser &parser);
} // namespace bounded_cost_search

#endif
