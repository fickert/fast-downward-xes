#!/usr/bin/env python3
import json
import os
import sys

import planning_domains_api as api


# sanitized base suites without duplicate instances
FAI_SUITE_SAT_STRIPS = ['agricola-sat18-strips', 'airport', 'barman-sat11-strips', 'barman-sat14-strips', 'blocks', 'childsnack-sat14-strips', 'data-network-sat18-strips', 'depot', 'driverlog', 'elevators-sat08-strips', 'elevators-sat11-strips:p11.pddl', 'elevators-sat11-strips:p12.pddl', 'elevators-sat11-strips:p13.pddl', 'elevators-sat11-strips:p14.pddl', 'elevators-sat11-strips:p15.pddl', 'elevators-sat11-strips:p16.pddl', 'elevators-sat11-strips:p17.pddl', 'elevators-sat11-strips:p18.pddl', 'elevators-sat11-strips:p19.pddl', 'elevators-sat11-strips:p20.pddl', 'floortile-sat11-strips', 'floortile-sat14-strips', 'freecell', 'ged-sat14-strips', 'grid', 'gripper', 'hiking-sat14-strips', 'logistics00', 'logistics98', 'miconic', 'movie', 'mprime', 'mystery:prob01.pddl', 'mystery:prob02.pddl', 'mystery:prob03.pddl', 'mystery:prob06.pddl', 'mystery:prob09.pddl', 'mystery:prob10.pddl', 'mystery:prob11.pddl', 'mystery:prob13.pddl', 'mystery:prob14.pddl', 'mystery:prob15.pddl', 'mystery:prob17.pddl', 'mystery:prob19.pddl', 'mystery:prob20.pddl', 'mystery:prob25.pddl', 'mystery:prob26.pddl', 'mystery:prob27.pddl', 'mystery:prob28.pddl', 'mystery:prob29.pddl', 'mystery:prob30.pddl', 'nomystery-sat11-strips', 'openstacks-sat08-strips', 'openstacks-sat11-strips:p11.pddl', 'openstacks-sat11-strips:p12.pddl', 'openstacks-sat11-strips:p13.pddl', 'openstacks-sat11-strips:p14.pddl', 'openstacks-sat11-strips:p15.pddl', 'openstacks-sat11-strips:p16.pddl', 'openstacks-sat11-strips:p17.pddl', 'openstacks-sat11-strips:p18.pddl', 'openstacks-sat11-strips:p19.pddl', 'openstacks-sat11-strips:p20.pddl', 'openstacks-sat14-strips', 'openstacks-strips', 'organic-synthesis-sat18-strips', 'organic-synthesis-split-sat18-strips', 'parcprinter-08-strips', 'parcprinter-sat11-strips:p11.pddl', 'parcprinter-sat11-strips:p12.pddl', 'parcprinter-sat11-strips:p13.pddl', 'parcprinter-sat11-strips:p14.pddl', 'parcprinter-sat11-strips:p15.pddl', 'parcprinter-sat11-strips:p16.pddl', 'parcprinter-sat11-strips:p17.pddl', 'parcprinter-sat11-strips:p18.pddl', 'parcprinter-sat11-strips:p19.pddl', 'parcprinter-sat11-strips:p20.pddl', 'parking-sat11-strips', 'parking-sat14-strips', 'pathways-noneg', 'pegsol-08-strips', 'pegsol-sat11-strips:p04.pddl', 'pegsol-sat11-strips:p05.pddl', 'pegsol-sat11-strips:p06.pddl', 'pegsol-sat11-strips:p07.pddl', 'pegsol-sat11-strips:p08.pddl', 'pipesworld-notankage', 'pipesworld-tankage', 'psr-small', 'rovers', 'satellite', 'scanalyzer-08-strips:p01.pddl', 'scanalyzer-08-strips:p02.pddl', 'scanalyzer-08-strips:p03.pddl', 'scanalyzer-08-strips:p04.pddl', 'scanalyzer-08-strips:p05.pddl', 'scanalyzer-08-strips:p06.pddl', 'scanalyzer-08-strips:p07.pddl', 'scanalyzer-08-strips:p08.pddl', 'scanalyzer-08-strips:p09.pddl', 'scanalyzer-08-strips:p10.pddl', 'scanalyzer-08-strips:p11.pddl', 'scanalyzer-08-strips:p12.pddl', 'scanalyzer-08-strips:p13.pddl', 'scanalyzer-08-strips:p14.pddl', 'scanalyzer-08-strips:p15.pddl', 'scanalyzer-08-strips:p16.pddl', 'scanalyzer-08-strips:p17.pddl', 'scanalyzer-08-strips:p18.pddl', 'scanalyzer-08-strips:p19.pddl', 'scanalyzer-08-strips:p20.pddl', 'scanalyzer-08-strips:p21.pddl', 'scanalyzer-08-strips:p22.pddl', 'scanalyzer-08-strips:p25.pddl', 'scanalyzer-08-strips:p26.pddl', 'scanalyzer-08-strips:p27.pddl', 'scanalyzer-08-strips:p28.pddl', 'scanalyzer-08-strips:p29.pddl', 'scanalyzer-08-strips:p30.pddl', 'snake-sat18-strips', 'sokoban-sat08-strips', 'spider-sat18-strips', 'storage', 'termes-sat18-strips', 'tetris-sat14-strips', 'thoughtful-sat14-strips', 'tidybot-sat11-strips', 'tpp', 'transport-sat08-strips', 'transport-sat11-strips:p11.pddl', 'transport-sat11-strips:p12.pddl', 'transport-sat11-strips:p13.pddl', 'transport-sat11-strips:p14.pddl', 'transport-sat11-strips:p15.pddl', 'transport-sat11-strips:p16.pddl', 'transport-sat11-strips:p17.pddl', 'transport-sat11-strips:p18.pddl', 'transport-sat11-strips:p19.pddl', 'transport-sat11-strips:p20.pddl', 'transport-sat14-strips', 'trucks-strips', 'visitall-sat11-strips', 'visitall-sat14-strips:pfile31.pddl', 'visitall-sat14-strips:pfile33.pddl', 'visitall-sat14-strips:pfile51.pddl', 'visitall-sat14-strips:pfile52.pddl', 'visitall-sat14-strips:pfile53.pddl', 'visitall-sat14-strips:pfile54.pddl', 'visitall-sat14-strips:pfile55.pddl', 'visitall-sat14-strips:pfile56.pddl', 'visitall-sat14-strips:pfile57.pddl', 'visitall-sat14-strips:pfile58.pddl', 'visitall-sat14-strips:pfile59.pddl', 'visitall-sat14-strips:pfile60.pddl', 'visitall-sat14-strips:pfile61.pddl', 'visitall-sat14-strips:pfile62.pddl', 'visitall-sat14-strips:pfile63.pddl', 'visitall-sat14-strips:pfile64.pddl', 'visitall-sat14-strips:pfile65.pddl', 'woodworking-sat08-strips', 'woodworking-sat11-strips:p11.pddl', 'woodworking-sat11-strips:p12.pddl', 'woodworking-sat11-strips:p13.pddl', 'woodworking-sat11-strips:p14.pddl', 'woodworking-sat11-strips:p15.pddl', 'woodworking-sat11-strips:p16.pddl', 'woodworking-sat11-strips:p17.pddl', 'woodworking-sat11-strips:p18.pddl', 'woodworking-sat11-strips:p19.pddl', 'woodworking-sat11-strips:p20.pddl', 'zenotravel']
FAI_SUITE_SAT = ['agricola-sat18-strips', 'airport', 'assembly', 'barman-sat11-strips', 'barman-sat14-strips', 'blocks', 'caldera-sat18-adl', 'caldera-split-sat18-adl', 'cavediving-14-adl', 'childsnack-sat14-strips', 'citycar-sat14-adl', 'data-network-sat18-strips', 'depot', 'driverlog', 'elevators-sat08-strips', 'elevators-sat11-strips:p11.pddl', 'elevators-sat11-strips:p12.pddl', 'elevators-sat11-strips:p13.pddl', 'elevators-sat11-strips:p14.pddl', 'elevators-sat11-strips:p15.pddl', 'elevators-sat11-strips:p16.pddl', 'elevators-sat11-strips:p17.pddl', 'elevators-sat11-strips:p18.pddl', 'elevators-sat11-strips:p19.pddl', 'elevators-sat11-strips:p20.pddl', 'flashfill-sat18-adl', 'floortile-sat11-strips', 'floortile-sat14-strips', 'freecell', 'ged-sat14-strips', 'grid', 'gripper', 'hiking-sat14-strips', 'logistics00', 'logistics98', 'maintenance-sat14-adl', 'miconic', 'miconic-fulladl', 'miconic-simpleadl', 'movie', 'mprime', 'mystery:prob01.pddl', 'mystery:prob02.pddl', 'mystery:prob03.pddl', 'mystery:prob06.pddl', 'mystery:prob09.pddl', 'mystery:prob10.pddl', 'mystery:prob11.pddl', 'mystery:prob13.pddl', 'mystery:prob14.pddl', 'mystery:prob15.pddl', 'mystery:prob17.pddl', 'mystery:prob19.pddl', 'mystery:prob20.pddl', 'mystery:prob25.pddl', 'mystery:prob26.pddl', 'mystery:prob27.pddl', 'mystery:prob28.pddl', 'mystery:prob29.pddl', 'mystery:prob30.pddl', 'nomystery-sat11-strips', 'nurikabe-sat18-adl', 'openstacks', 'openstacks-sat08-adl', 'openstacks-sat08-strips', 'openstacks-sat11-strips:p11.pddl', 'openstacks-sat11-strips:p12.pddl', 'openstacks-sat11-strips:p13.pddl', 'openstacks-sat11-strips:p14.pddl', 'openstacks-sat11-strips:p15.pddl', 'openstacks-sat11-strips:p16.pddl', 'openstacks-sat11-strips:p17.pddl', 'openstacks-sat11-strips:p18.pddl', 'openstacks-sat11-strips:p19.pddl', 'openstacks-sat11-strips:p20.pddl', 'openstacks-sat14-strips', 'openstacks-strips', 'optical-telegraphs', 'organic-synthesis-sat18-strips', 'organic-synthesis-split-sat18-strips', 'parcprinter-08-strips', 'parcprinter-sat11-strips:p11.pddl', 'parcprinter-sat11-strips:p12.pddl', 'parcprinter-sat11-strips:p13.pddl', 'parcprinter-sat11-strips:p14.pddl', 'parcprinter-sat11-strips:p15.pddl', 'parcprinter-sat11-strips:p16.pddl', 'parcprinter-sat11-strips:p17.pddl', 'parcprinter-sat11-strips:p18.pddl', 'parcprinter-sat11-strips:p19.pddl', 'parcprinter-sat11-strips:p20.pddl', 'parking-sat11-strips', 'parking-sat14-strips', 'pathways', 'pathways-noneg', 'pegsol-08-strips', 'pegsol-sat11-strips:p04.pddl', 'pegsol-sat11-strips:p05.pddl', 'pegsol-sat11-strips:p06.pddl', 'pegsol-sat11-strips:p07.pddl', 'pegsol-sat11-strips:p08.pddl', 'philosophers', 'pipesworld-notankage', 'pipesworld-tankage', 'psr-large', 'psr-middle', 'psr-small', 'rovers', 'satellite', 'scanalyzer-08-strips:p01.pddl', 'scanalyzer-08-strips:p02.pddl', 'scanalyzer-08-strips:p03.pddl', 'scanalyzer-08-strips:p04.pddl', 'scanalyzer-08-strips:p05.pddl', 'scanalyzer-08-strips:p06.pddl', 'scanalyzer-08-strips:p07.pddl', 'scanalyzer-08-strips:p08.pddl', 'scanalyzer-08-strips:p09.pddl', 'scanalyzer-08-strips:p10.pddl', 'scanalyzer-08-strips:p11.pddl', 'scanalyzer-08-strips:p12.pddl', 'scanalyzer-08-strips:p13.pddl', 'scanalyzer-08-strips:p14.pddl', 'scanalyzer-08-strips:p15.pddl', 'scanalyzer-08-strips:p16.pddl', 'scanalyzer-08-strips:p17.pddl', 'scanalyzer-08-strips:p18.pddl', 'scanalyzer-08-strips:p19.pddl', 'scanalyzer-08-strips:p20.pddl', 'scanalyzer-08-strips:p21.pddl', 'scanalyzer-08-strips:p22.pddl', 'scanalyzer-08-strips:p25.pddl', 'scanalyzer-08-strips:p26.pddl', 'scanalyzer-08-strips:p27.pddl', 'scanalyzer-08-strips:p28.pddl', 'scanalyzer-08-strips:p29.pddl', 'scanalyzer-08-strips:p30.pddl', 'schedule', 'settlers-sat18-adl', 'snake-sat18-strips', 'sokoban-sat08-strips', 'spider-sat18-strips', 'storage', 'termes-sat18-strips', 'tetris-sat14-strips', 'thoughtful-sat14-strips', 'tidybot-sat11-strips', 'tpp', 'transport-sat08-strips', 'transport-sat11-strips:p11.pddl', 'transport-sat11-strips:p12.pddl', 'transport-sat11-strips:p13.pddl', 'transport-sat11-strips:p14.pddl', 'transport-sat11-strips:p15.pddl', 'transport-sat11-strips:p16.pddl', 'transport-sat11-strips:p17.pddl', 'transport-sat11-strips:p18.pddl', 'transport-sat11-strips:p19.pddl', 'transport-sat11-strips:p20.pddl', 'transport-sat14-strips', 'trucks', 'trucks-strips', 'visitall-sat11-strips', 'visitall-sat14-strips:pfile31.pddl', 'visitall-sat14-strips:pfile33.pddl', 'visitall-sat14-strips:pfile51.pddl', 'visitall-sat14-strips:pfile52.pddl', 'visitall-sat14-strips:pfile53.pddl', 'visitall-sat14-strips:pfile54.pddl', 'visitall-sat14-strips:pfile55.pddl', 'visitall-sat14-strips:pfile56.pddl', 'visitall-sat14-strips:pfile57.pddl', 'visitall-sat14-strips:pfile58.pddl', 'visitall-sat14-strips:pfile59.pddl', 'visitall-sat14-strips:pfile60.pddl', 'visitall-sat14-strips:pfile61.pddl', 'visitall-sat14-strips:pfile62.pddl', 'visitall-sat14-strips:pfile63.pddl', 'visitall-sat14-strips:pfile64.pddl', 'visitall-sat14-strips:pfile65.pddl', 'woodworking-sat08-strips', 'woodworking-sat11-strips:p11.pddl', 'woodworking-sat11-strips:p12.pddl', 'woodworking-sat11-strips:p13.pddl', 'woodworking-sat11-strips:p14.pddl', 'woodworking-sat11-strips:p15.pddl', 'woodworking-sat11-strips:p16.pddl', 'woodworking-sat11-strips:p17.pddl', 'woodworking-sat11-strips:p18.pddl', 'woodworking-sat11-strips:p19.pddl', 'woodworking-sat11-strips:p20.pddl', 'zenotravel']

FAI_SUITE_SAT_STRIPS_NO_IPC18 = [x for x in FAI_SUITE_SAT_STRIPS if 'sat18' not in x]
FAI_SUITE_SAT_NO_IPC18 = [x for x in FAI_SUITE_SAT if 'sat18' not in x]

# adapt these variables as necessary
DOWNWARD_BENCHMARKS = os.path.join(os.path.expanduser('~'), 'home_windows', 'Projects', 'downward-benchmarks')
SUITE = FAI_SUITE_SAT_STRIPS_NO_IPC18


# update this with domains that are not found automatically
DOMAIN_IDS = {
    'logistics98':              38,
    'logistics00':              43,
    'miconic':                  12,
    'miconic-fulladl':         114,
    'miconic-simpleadl':        99,
    'openstacks':               75,
    'openstacks-sat08-adl':     65,
    'openstacks-sat08-strips':  76,
    'openstacks-strips':        64,
    'pathways-noneg':           45,
    'satellite':                32,
    'trucks-strips':            16,
}

# update this with instances that are not found automatically
INSTANCE_IDS = {
    'maintenance-sat14-adl:maintenance-1-3-060-180-5-000.pddl': 1731,
    'maintenance-sat14-adl:maintenance-1-3-060-180-5-001.pddl': 1733,
    'maintenance-sat14-adl:maintenance-1-3-060-180-5-002.pddl': 1732,
    'maintenance-sat14-adl:maintenance-1-3-100-300-5-000.pddl': 1734,
    'maintenance-sat14-adl:maintenance-1-3-100-300-5-001.pddl': 1735,
    'maintenance-sat14-adl:maintenance-1-3-100-300-7-000.pddl': 1736,
    'maintenance-sat14-adl:maintenance-1-3-100-300-7-001.pddl': 1737,
    'maintenance-sat14-adl:maintenance-1-3-100-300-7-002.pddl': 1739,
    'maintenance-sat14-adl:maintenance-1-3-150-500-6-001.pddl': 1738,
    'maintenance-sat14-adl:maintenance-1-3-200-500-5-001.pddl': 1740,
    'maintenance-sat14-adl:maintenance-1-3-200-500-5-002.pddl': 1741,
    'maintenance-sat14-adl:maintenance-1-3-200-700-7-000.pddl': 1742,
    'maintenance-sat14-adl:maintenance-1-3-200-700-7-001.pddl': 1743,
    'maintenance-sat14-adl:maintenance-1-3-200-700-7-002.pddl': 1744,
    'maintenance-sat14-adl:maintenance-1-3-200-900-5-000.pddl': 1745,
    'maintenance-sat14-adl:maintenance-1-3-200-900-5-001.pddl': 1746,
    'maintenance-sat14-adl:maintenance-1-3-200-900-5-002.pddl': 1747,
    'maintenance-sat14-adl:maintenance-1-3-200-900-8-000.pddl': 1748,
    'maintenance-sat14-adl:maintenance-1-3-200-900-8-001.pddl': 1749,
    'maintenance-sat14-adl:maintenance-1-3-200-900-8-002.pddl': 1750,
}


def get_collection(domain_name):
    def _get_collection(collection_ids):
        if isinstance(collection_ids, int):
            collection_ids = [collection_ids]
        collection = []
        for collection_id in collection_ids:
            for domain in api.get_domains(collection_id):
                if all(d['domain_id'] != domain['domain_id'] for d in collection):
                    collection.append(domain)
        return collection
    if '-sat18' in domain_name:
        return _get_collection(13)
    if '-sat14' in domain_name or '-14' in domain_name:
        return _get_collection(8)
    if '-sat11' in domain_name:
        return _get_collection(7)
    if '-sat08' in domain_name or '-08' in domain_name:
        return _get_collection(6)
    return _get_collection([9, 11, 12])

def simplify_domain_name(domain_name):
    for track in ['sat18', 'sat14', '14', 'sat11', 'sat08', '08']:
        try:
            return domain_name[:domain_name.index(f'-{track}')]
        except ValueError:
            pass
    return domain_name

def replace_pfile(problem):
    try:
        replacement = 'p' if problem[problem.index('pfile') + 6].isdigit() else 'p0'
        return problem.replace('pfile', replacement)
    except ValueError:
        return problem

def find_problem(instance, problems, domain_name):
    key = f'{domain_name}:{instance}'
    if key in INSTANCE_IDS:
        return api.get_problem(INSTANCE_IDS[key])
    for problem in problems:
        if problem['problem'].lower() == instance.lower():
            return problem
    # if the exact name was not found, try again with similar names
    for problem in problems:
        if replace_pfile(problem['problem']).lower() == instance.lower():
            return problem
    print(f'Instance {instance} of domain {domain_name} not found.')
    sys.exit(1)

def find_domain(domain_name):
    if domain_name in DOMAIN_IDS:
        return DOMAIN_IDS[domain_name]
    collection = get_collection(domain_name)
    matches = []
    simplified_domain_name = simplify_domain_name(domain_name)
    for domain in collection:
        if domain['domain_name'].lower() == simplified_domain_name.lower():
            matches.append(domain)
    if len(matches) == 0:
        print(f'Domain {domain_name} not found, please add ID manually.')
        sys.exit(1)
    # if there are multiple matches it's usually because there are multiple tracks --> try to find the satisficing track
    if len(matches) > 1:
        matches = [domain for domain in matches if domain['description'].startswith('(sat')]
    if len(matches) != 1:
        print(matches)
        print(f'Found multiple matches for domain {domain_name} and failed to identify the correct one, please add ID manually.')
        sys.exit(1)
    domain_id = matches[0]['domain_id']
    DOMAIN_IDS[domain_name] = domain_id
    return domain_id

def get_all_instances(domain):
    domain_dir = os.path.join(DOWNWARD_BENCHMARKS, domain)
    return [f for f in os.listdir(domain_dir) if os.path.isfile(os.path.join(domain_dir, f)) and 'domain' not in f]

def pretty_print(cost_bounds):
    print('[')
    for instance, cost_bound in cost_bounds:
        print(f'    ["{instance}", {cost_bound}],')
    print(']')


if __name__ == '__main__':
    cost_bounds = []
    num_optimal = 0
    num_nonoptimal = 0
    num_unknown = 0
    for x in SUITE:
        if ':' in x:
            domain, instance = x.split(':')
            instances = [instance]
        else:
            domain = x
            instances = get_all_instances(domain)
        if domain == 'freecell':
            # freecell seems to have two variants
            problems = api.get_problems(29) + api.get_problems(30)
        else:
            domain_id = find_domain(domain)
            problems = api.get_problems(domain_id)
        for instance in instances:
            problem = find_problem(instance, problems, domain)
            bound = problem['upper_bound']
            if bound is not None:
                cost_bounds.append([f'{domain}:{instance}', bound])
                lower_bound = problem['lower_bound']
                if lower_bound is None:
                    num_unknown += 1
                elif lower_bound == bound:
                    num_optimal += 1
                else:
                    num_nonoptimal += 1
    pretty_print(cost_bounds)
    print(f'Got {num_optimal} optimal, {num_nonoptimal} non-optimal, and {num_unknown} unknown bounds (out of {len(cost_bounds)} instances).')
