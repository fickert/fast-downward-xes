#ifndef SUBOPTIMAL_SEARCH_EAGER_SUBOPTIMAL_SEARCH_H
#define SUBOPTIMAL_SEARCH_EAGER_SUBOPTIMAL_SEARCH_H

#include <array>
#include <functional>
#include <memory>
#include <optional>
#include <vector>

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

namespace suboptimal_search {
// N is the number of evaluators
template <std::size_t N>
class EagerSuboptimalSearch : public SearchEngine {
	static_assert(N > 0, "There must be at least one evaluator.");

	const bool reopen_closed_nodes;

	std::array<std::shared_ptr<floating_point_evaluator::FloatingPointEvaluator>, N> evaluators;
	std::vector<std::shared_ptr<Evaluator>> preferred_operator_evaluators;
	std::shared_ptr<PruningMethod> pruning_method;

	int max_g_value;
	auto check_progress(int g_value) -> bool;

protected:
	virtual void reward_progress() = 0;

	using EvaluatorValues = std::array<double, N>;

	std::vector<std::shared_ptr<heuristic_error::HeuristicError>> heuristic_error;

	void initialize() override;
	SearchStatus step() override;

	auto compute_results(EvaluationContext &eval_context) -> EvaluatorValues;
	auto is_dead_end(const EvaluatorValues &values) -> bool;

	virtual void initialize_heuristic_error(EvaluationContext &eval_context);
	virtual void initialize_extra(EvaluationContext &) {}

	virtual auto fetch_next_node() -> std::optional<SearchNode> = 0;
	virtual void insert(EvaluationContext &eval_context, const EvaluatorValues &evaluator_values, StateID state_id, bool preferred) = 0;

public:
	explicit EagerSuboptimalSearch(const options::Options &opts);
	virtual ~EagerSuboptimalSearch() = default;

	void print_statistics() const override;

	void dump_search_space() const;
};

extern void add_options_to_parser(options::OptionParser &parser);
} // namespace suboptimal_search

#endif
