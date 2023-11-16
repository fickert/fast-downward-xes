#include "expansion_delay.h"

namespace expansion_delay {

ExpansionDelay::ExpansionDelay(const std::size_t moving_average_size)
	: avg_expansion_delay(0), statistics(nullptr), additional_expansions(0), last_delays_sum(0), moving_average_size(moving_average_size) {}

ExpansionDelay::ExpansionDelay(const SearchStatistics &statistics, const std::size_t moving_average_size)
	: avg_expansion_delay(0), statistics(&statistics), additional_expansions(0), last_delays_sum(0), moving_average_size(moving_average_size) {}

ExpansionDelay::ExpansionDelay(const SearchStatistics &statistics, const std::size_t moving_average_size, double initial_avg_expansion_delay_value,
                               int additional_expansions)
	: avg_expansion_delay(initial_avg_expansion_delay_value),
	  statistics(&statistics),
	  additional_expansions(additional_expansions),
	  last_delays(additional_expansions, std::lround(initial_avg_expansion_delay_value)),
	  last_delays_sum(std::lround(initial_avg_expansion_delay_value) * moving_average_size),
	  moving_average_size(moving_average_size) {}

void ExpansionDelay::initialize(const SearchStatistics &statistics) {
	this->statistics = &statistics;
}

void ExpansionDelay::update_expansion_delay(int delay) {
	assert(statistics);
	assert(delay >= 1);
	avg_expansion_delay += (delay - avg_expansion_delay) / (statistics->get_expanded() + additional_expansions);
	if (moving_average_size != 0u) {
		last_delays.push_back(delay);
		last_delays_sum += delay;
		if (last_delays.size() > moving_average_size) {
			last_delays_sum -= last_delays.front();
			last_delays.pop_front();
		}
	}
}

auto ExpansionDelay::get_avg_expansion_delay() const -> double {
	assert(statistics);
	if (statistics->get_expanded() + additional_expansions < 2)
		return 1.;
	return moving_average_size == 0u ? avg_expansion_delay : last_delays_sum / static_cast<double>(last_delays.size());
}

auto ExpansionDelay::get_total_avg_expansion_delay() const -> double {
	assert(statistics);
	if (statistics->get_expanded() + additional_expansions < 2)
		return 1.;
	return avg_expansion_delay;
}

} // namespace expansion_delay
