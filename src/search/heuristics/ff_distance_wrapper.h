#ifndef HEURISTICS_FF_DISTANCE_WRAPPER_H
#define HEURISTICS_FF_DISTANCE_WRAPPER_H

#include "ff_heuristic.h"

namespace ff_heuristic {
class FFDistanceWrapper : public Heuristic {
	std::shared_ptr<ff_heuristic::FFHeuristic> ff_heuristic;

public:
	explicit FFDistanceWrapper(const options::Options &opts);

	auto compute_heuristic(const GlobalState &global_state) -> int override;
	auto dead_ends_are_reliable() const -> bool override { return true; };
};
} // namespace ff_heuristic

#endif
