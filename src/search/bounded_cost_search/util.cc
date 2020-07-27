#include "util.h"

#include "../evaluator.h"
#include "../heuristic_error/debiased_heuristic.h"
#include "../heuristic_error/one_step_heuristic_error.h"
#include "../heuristic_error/percentage_based_debiased_heuristic.h"
#include "../heuristic_error/percentage_based_heuristic_error.h"
#include "../option_parser.h"

using namespace floating_point_evaluator;
using namespace heuristic_error;
using namespace options;

namespace bounded_cost_search {
void add_warm_start_options(OptionParser &parser) {
	parser.add_option<int>("warm_start_samples", "Initialize heuristic and distance error with the given number of virtual error samples.", "50",
	                       Bounds("0", "infinity"));
	parser.add_option<double>("warm_start_value_distance", "Value to warm-start the distance error with.", ".2", Bounds("0", "1"));
	parser.add_option<double>("warm_start_value_heuristic", "Value to warm-start the heuristic error with.", "1");
	parser.add_option<bool>("initialize_error_with_cost_bound",
	                        "assume that the initial heuristic value should be equal to the cost bound for the initialization of the heuristic error", "false");
}

void add_percentage_based_error_option(OptionParser &parser) {
	parser.add_option<bool>("percentage_based_error", "use percentage-based error instead of one-step error", "false");
}

void add_online_variance_option(options::OptionParser &parser) {
	parser.add_option<bool>("use_online_variance", "use variance measured online for the probability distributions", "false");
}

void add_d_tie_breaking_option(options::OptionParser &parser) {
	parser.add_option<bool>("enable_tie_breaking", "break ties by estimated goal distance", "true");
}

void add_f_then_h_tie_breaking_option(options::OptionParser &parser) {
	parser.add_option<bool>("enable_tie_breaking", "break ties by f and then estimated cost to goal", "true");
}

void add_f_hat_then_d_tie_breaking_option(options::OptionParser &parser) {
	parser.add_option<bool>("enable_tie_breaking", "break ties by f-hat and then estimated goal distance", "true");
}

auto get_heuristic_error(bool percentage_based_error, const std::shared_ptr<Evaluator> &heuristic, int warm_start_samples, double warm_start_value)
		-> std::shared_ptr<HeuristicError> {
	auto heuristic_error_opts = Options();
	heuristic_error_opts.set("eval", heuristic);
	heuristic_error_opts.set("warm_start_samples", warm_start_samples);
	heuristic_error_opts.set("warm_start_value", warm_start_value);
	return percentage_based_error ? std::static_pointer_cast<HeuristicError>(std::make_shared<PercentageBasedHeuristicError>(heuristic_error_opts))
	                              : std::static_pointer_cast<HeuristicError>(std::make_shared<OneStepHeuristicError>(heuristic_error_opts));
}

auto get_debiased_heuristic(bool percentage_based_error, const std::shared_ptr<Evaluator> &heuristic,
                            const std::shared_ptr<FloatingPointEvaluator> &debiased_distance, const std::shared_ptr<HeuristicError> &heuristic_error)
		-> std::shared_ptr<FloatingPointEvaluator> {
	auto debiased_heuristic_opts = options::Options();
	debiased_heuristic_opts.set("cache_estimates", false);
	debiased_heuristic_opts.set("h", heuristic);
	debiased_heuristic_opts.set("d", debiased_distance);
	debiased_heuristic_opts.set("error", heuristic_error);
	return percentage_based_error ? std::static_pointer_cast<FloatingPointEvaluator>(std::make_shared<PercentageBasedDebiasedHeuristic>(debiased_heuristic_opts))
	                              : std::static_pointer_cast<FloatingPointEvaluator>(std::make_shared<DebiasedHeuristic>(debiased_heuristic_opts));
}

} // namespace bounded_cost_search
