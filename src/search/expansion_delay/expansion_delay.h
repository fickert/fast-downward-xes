#ifndef EXPANSION_DELAY_EXPANSION_DELAY_H
#define EXPANSION_DELAY_EXPANSION_DELAY_H

#include <cassert>
#include <cmath>
#include <deque>

#include "../search_statistics.h"

namespace expansion_delay {

class ExpansionDelay {
	double avg_expansion_delay;
	const SearchStatistics *statistics;
	const int additional_expansions;

	std::deque<int> last_delays;
	long long last_delays_sum;
	const std::size_t moving_average_size;

public:
	explicit ExpansionDelay(const std::size_t moving_average_size);
	ExpansionDelay(const SearchStatistics &statistics, const std::size_t moving_average_size);
	ExpansionDelay(const SearchStatistics &statistics, const std::size_t moving_average_size, double initial_avg_expansion_delay_value, int additional_expansions);

	void initialize(const SearchStatistics &statistics);
	void update_expansion_delay(int delay);

	auto get_avg_expansion_delay() const -> double;
	auto get_total_avg_expansion_delay() const -> double;
};

} // namespace expansion_delay

#endif
