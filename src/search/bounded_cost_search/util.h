#ifndef BOUNDED_COST_SEARCH_UTIL_H
#define BOUNDED_COST_SEARCH_UTIL_H

#include <memory>

class Evaluator;

namespace floating_point_evaluator {
class FloatingPointEvaluator;
}

namespace heuristic_error {
class HeuristicError;
}

namespace options {
class OptionParser;
}

namespace bounded_cost_search {
void add_warm_start_options(options::OptionParser &parser);
void add_percentage_based_error_option(options::OptionParser &parser);
void add_online_variance_option(options::OptionParser &parser);

void add_d_tie_breaking_option(options::OptionParser &parser);
void add_f_then_h_tie_breaking_option(options::OptionParser &parser);
void add_f_hat_then_d_tie_breaking_option(options::OptionParser &parser);

auto get_heuristic_error(bool percentage_based_error, const std::shared_ptr<Evaluator> &heuristic, int warm_start_samples, double warm_start_value)
		-> std::shared_ptr<heuristic_error::HeuristicError>;
auto get_debiased_heuristic(bool percentage_based_error, const std::shared_ptr<Evaluator> &heuristic,
                            const std::shared_ptr<floating_point_evaluator::FloatingPointEvaluator> &debiased_distance,
                            const std::shared_ptr<heuristic_error::HeuristicError> &heuristic_error)
		-> std::shared_ptr<floating_point_evaluator::FloatingPointEvaluator>;
} // namespace bounded_cost_search

#endif
