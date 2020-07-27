#ifndef HEURISTICS_FF_HEURISTIC_H
#define HEURISTICS_FF_HEURISTIC_H

#include "additive_heuristic.h"

#include <vector>

namespace ff_heuristic {
using relaxation_heuristic::PropID;
using relaxation_heuristic::OpID;

using relaxation_heuristic::NO_OP;

using relaxation_heuristic::Proposition;
using relaxation_heuristic::UnaryOperator;

/*
  TODO: In a better world, this should not derive from
        AdditiveHeuristic. Rather, the common parts should be
        implemented in a common base class. That refactoring could be
        made at the same time at which we also unify this with the
        other relaxation heuristics and the additional FF heuristic
        implementation in the landmark code.
*/
class FFHeuristic : public additive_heuristic::AdditiveHeuristic {
    // Relaxed plans are represented as a set of operators implemented
    // as a bit vector.
    using RelaxedPlan = std::vector<bool>;
    RelaxedPlan relaxed_plan;
    void mark_preferred_operators_and_relaxed_plan(
        const State &state, PropID goal_id, bool set_preferred_operators);
    StateID last_evaluated;
    int last_heuristic_value;
    std::vector<OperatorProxy> last_preferred_operators;
    friend class FFDistanceWrapper;
    int last_relaxed_plan_length;

    auto compute_heuristic(const GlobalState &global_state, bool set_preferred_operators) -> int;
    auto compute_heuristic_for_distance_wrapper(const GlobalState &global_state) -> int;
    virtual int compute_heuristic(const GlobalState &global_state) override;
public:
    explicit FFHeuristic(const options::Options &opts);
};
}

#endif
