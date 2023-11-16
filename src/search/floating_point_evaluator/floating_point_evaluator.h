#ifndef FLOATING_POINT_EVALUATOR_FLOATING_POINT_EVALUATOR_H
#define FLOATING_POINT_EVALUATOR_FLOATING_POINT_EVALUATOR_H

#include <cmath>
#include <limits>

#include "../per_state_information.h"

class EvaluationContext;
namespace options {
class Options;
class OptionParser;
} // namespace options

namespace floating_point_evaluator {
class FloatingPointEvaluator {
public:
	static constexpr auto DEAD_END = std::numeric_limits<double>::quiet_NaN();
	static auto is_dead_end(double value) -> bool { return std::isnan(value); }

private:
	/*
	  Cache for saving floating point values
	  Before accessing this cache always make sure that the cache_evaluator_values
	  flag is set to true - as soon as the cache is accessed it will create
	  entries for all existing states
	*/
	PerStateInformation<double> cache;
	bool cache_evaluator_values;

	virtual auto compute_value(EvaluationContext &eval_context) -> double = 0;

protected:
	static auto get_default_options() -> options::Options;

public:
	explicit FloatingPointEvaluator(const options::Options &opts);
	virtual ~FloatingPointEvaluator() = default;

	FloatingPointEvaluator(const FloatingPointEvaluator &other) = delete;
	FloatingPointEvaluator(FloatingPointEvaluator &&other) = delete;

	auto operator=(const FloatingPointEvaluator &other) -> FloatingPointEvaluator & = delete;
	auto operator=(FloatingPointEvaluator &&other) -> FloatingPointEvaluator & = delete;

	auto compute_result(EvaluationContext &eval_context) -> double;

	static void add_options_to_parser(options::OptionParser &parser);

	auto does_cache_estimates() const -> bool { return cache_evaluator_values; }
	auto is_estimate_cached(const GlobalState &state) const { return !std::isnan(cache[state]); }
	/*
	  Calling get_cached_estimate is only allowed if an estimate for
	  the given state is cached, i.e., is_estimate_cached returns true.
	*/
	auto get_cached_estimate(const GlobalState &state) const -> double {
		assert(is_estimate_cached(state));
		return cache[state];
	}
};
} // namespace floating_point_evaluator

#endif
