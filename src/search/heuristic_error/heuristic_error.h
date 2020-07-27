#ifndef HEURISTIC_ERROR_HEURISTIC_ERROR_H
#define HEURISTIC_ERROR_HEURISTIC_ERROR_H

#include <memory>

class EvaluationContext;
class Evaluator;
class GlobalState;
class SearchNode;
class StateRegistry;
namespace options {
class Options;
class OptionParser;
} // namespace options

namespace heuristic_error {
class HeuristicError {
	// This is the base class for heuristic error estimators.
protected:
	std::shared_ptr<Evaluator> evaluator;

public:
	HeuristicError(const options::Options &opts);
	virtual ~HeuristicError() = default;

	virtual void initialize(const StateRegistry &state_registry) = 0;
	virtual void notify_initial_state();
	virtual void notify_initial_state(EvaluationContext &eval_context, int cost_bound, int distance);

	virtual void set_expanding_state(const GlobalState &state) = 0;
	virtual void add_successor(const SearchNode &successor_node, int op_cost) = 0;
	virtual void update_error() = 0;

	virtual auto get_average_heuristic_error() const -> double = 0;
	virtual auto get_heuristic_error_variance() const -> double = 0;

	static void add_options_to_parser(options::OptionParser &parser);
};
} // namespace heuristic_error

#endif
