#include "f_hat_min_evaluator.h"

namespace bounded_suboptimal_search {
FHatMinEvaluator::FHatMinEvaluator(const options::Options &opts, bool track_variance)
	: floating_point_evaluator::FloatingPointEvaluator(opts),
	  f_hat_min(std::numeric_limits<double>::signaling_NaN()),
	  d_hat(std::numeric_limits<double>::signaling_NaN()),
	  track_variance(track_variance),
	  average_f_hat_min(0),
	  f_hat_min_M2_sum(0),
	  count(0) {}

void FHatMinEvaluator::update(double f_hat_min, double d_hat) {
	assert(!std::isnan(f_hat_min) && !std::isnan(d_hat));
	assert(!std::isinf(f_hat_min) && !std::isinf(d_hat));
	this->f_hat_min = f_hat_min;
	this->d_hat = d_hat;
	if (track_variance) {
		// see https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance#Welford's_online_algorithm
		++count;
		const auto delta = f_hat_min - average_f_hat_min;
		average_f_hat_min += delta / count;
		const auto delta2 = f_hat_min - average_f_hat_min;
		f_hat_min_M2_sum += std::abs(delta * delta2);
	}
}
auto FHatMinEvaluator::get_variance() const -> double {
	assert(count > 0);
	assert(f_hat_min_M2_sum >= 0);
	if (!track_variance) {
		std::cerr << "Attempted to retrieve variance but track_variance was not enabled!" << std::endl;
		utils::exit_with(utils::ExitCode::SEARCH_CRITICAL_ERROR);
	}
	return count == 1 ? 0. : f_hat_min_M2_sum / (count - 1);
}
} // namespace bounded_suboptimal_search
