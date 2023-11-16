This is a fork of [Fast Downward](http://www.fast-downward.org/) extended with implementations of the expected-effort based bounded-cost and bounded-suboptimal search algorithms XES and DXES as introduced in the following papers:
* M. Fickert, T. Gu, and W. Ruml: **[Bounded-Cost Search Using Estimates of Uncertainty](https://fai.cs.uni-saarland.de/fickert/papers/ijcai21.pdf)** (IJCAI'21)
* M. Fickert, T. Gu, and W. Ruml: **[New Results in Bounded-Suboptimal Search](https://fai.cs.uni-saarland.de/fickert/papers/aaai22.pdf)** (AAAI'22)

 The repository also contains implementations of the other bounded-cost and bounded-suboptimal search algorithms evaluated in the paper. This includes potential search, BEES, and BEEPS for bounded-cost search, and EES variants, CDXES, and round-robin algorithms for bounded-suboptimal search.

Example usage for XES:

`./fast-downward.py domain.pddl problem.pddl --evaluator "hff=ff()" --search "xes(hff, ff_distance(hff), preferred=[hff], bound=42)"`

Example usage for RR-DXES:

`./fast-downward.py domain.pddl problem.pddl --evaluator "hlm=lmcut()" --if-unit-cost --evaluator "hlm1=hlm" --if-non-unit-cost --evaluator "hlm1=lmcut(transform=adapt_costs(ONE))" --always --evaluator "hff=ff()" --search "dxes(hlm, hlm1, suboptimality_factor=1.5)"`

# Fast Downward

Fast Downward is a domain-independent classical planning system.

Copyright 2003-2020 Fast Downward contributors (see below).

For further information:
- Fast Downward website: <http://www.fast-downward.org>
- Report a bug or file an issue: <http://issues.fast-downward.org>
- Fast Downward mailing list: <https://groups.google.com/forum/#!forum/fast-downward>
- Fast Downward main repository: <https://github.com/aibasel/downward>


## Contributors

The following list includes all people that actively contributed to
Fast Downward, i.e. all people that appear in some commits in Fast
Downward's history (see below for a history on how Fast Downward
emerged) or people that influenced the development of such commits.
Currently, this list is sorted by the last year the person has been
active, and in case of ties, by the earliest year the person started
contributing, and finally by last name.

- 2003-2020 Malte Helmert
- 2008-2016, 2018-2020 Gabriele Roeger
- 2010-2020 Jendrik Seipp
- 2010-2011, 2013-2020 Silvan Sievers
- 2012-2020 Florian Pommerening
- 2013, 2015-2020 Salome Eriksson
- 2016-2020 Cedric Geissmann
- 2017-2020 Guillem Francès
- 2018-2020 Augusto B. Corrêa
- 2018-2020 Patrick Ferber
- 2015-2019 Manuel Heusner
- 2017 Daniel Killenberger
- 2016 Yusra Alkhazraji
- 2016 Martin Wehrle
- 2014-2015 Patrick von Reth
- 2015 Thomas Keller
- 2009-2014 Erez Karpas
- 2014 Robert P. Goldman
- 2010-2012 Andrew Coles
- 2010, 2012 Patrik Haslum
- 2003-2011 Silvia Richter
- 2009-2011 Emil Keyder
- 2010-2011 Moritz Gronbach
- 2010-2011 Manuela Ortlieb
- 2011 Vidal Alcázar Saiz
- 2011 Michael Katz
- 2011 Raz Nissim
- 2010 Moritz Goebelbecker
- 2007-2009 Matthias Westphal
- 2009 Christian Muise


## History

The current version of Fast Downward is the merger of three different
projects:

- the original version of Fast Downward developed by Malte Helmert
  and Silvia Richter
- LAMA, developed by Silvia Richter and Matthias Westphal based on
  the original Fast Downward
- FD-Tech, a modified version of Fast Downward developed by Erez
  Karpas and Michael Katz based on the original code

In addition to these three main sources, the codebase incorporates
code and features from numerous branches of the Fast Downward codebase
developed for various research papers. The main contributors to these
branches are Malte Helmert, Gabi Röger and Silvia Richter.


## License

The following directory is not part of Fast Downward as covered by
this license:

- ./src/search/ext

For the rest, the following license applies:

```
Fast Downward is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or (at
your option) any later version.

Fast Downward is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program. If not, see <https://www.gnu.org/licenses/>.
```
