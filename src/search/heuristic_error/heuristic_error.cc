#include "heuristic_error.h"

#include "../evaluator.h"
#include "../option_parser.h"
#include "../plugin.h"

namespace heuristic_error {
HeuristicError::HeuristicError(const options::Options &opts) : evaluator(opts.get<std::shared_ptr<Evaluator>>("eval")) {}

void HeuristicError::notify_initial_state() {}

void HeuristicError::notify_initial_state(EvaluationContext &, int, int) {}

void HeuristicError::add_options_to_parser(options::OptionParser &parser) {
	parser.add_option<std::shared_ptr<Evaluator>>("eval", "evaluator");
}

static PluginTypePlugin<HeuristicError> _type_plugin("HeuristicError", "Track the average error of a given heuristic.", "heuristic_error", "herror");
} // namespace heuristic_error
