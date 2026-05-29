#include "rules.hpp"
#include <cmath>
#include <iostream>

namespace boidz {

double get_distance(const SingleBoid &b1, const SingleBoid &b2) {
  Coords a = b1.getPosition();
  Coords b = b2.getPosition();

  return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}

bool check_critical_distance(const double &d_s, const SingleBoid &b1,
                             const SingleBoid &b2) {
  Coords a = b1.getPosition();
  Coords b = b2.getPosition();

  return std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2) <= std::pow(d_s, 2);
}

bool check_neighbours(const double &d, const SingleBoid &b1,
                      const SingleBoid &b2) {
  Coords a = b1.getPosition();
  Coords b = b2.getPosition();

  return std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2) <= std::pow(d, 2);
}

std::vector<SingleBoid> get_neighbours(const double &distance,
                                       const SingleBoid &s, const Flock &f) {
  std::vector<SingleBoid> neighbours{};
  for (auto x : f.flock) {
    if (check_neighbours(distance, s, x) == true) {
      if (s.getCardinality() != x.getCardinality()) {
        neighbours.push_back(x);
      }
    }
  }
  return neighbours;
}

Coords separation_for_single_boid(const Parameters &par, const Flock &stormo,
                                  const SingleBoid &a) {
  double sum_x{0};
  double sum_y{0};
  auto neighbours = get_neighbours(par.d, a, stormo);
  for (auto b : neighbours) {
    if (check_critical_distance(par.d_s, a, b)) {
      sum_x += b.getPosition().x - a.getPosition().x;
      sum_y += b.getPosition().y - a.getPosition().y;
    }
  }

  Coords v_separation{-(par.s * sum_x), -(par.s * sum_y)};
  return v_separation;
}

Coords alignment_for_single_boid(const Parameters &par, const Flock &stormo,
                                 const SingleBoid &s) {
  Coords v_alignment;
  Coords v_sum{0., 0.};
  auto neighbours = get_neighbours(par.d, s, stormo);
  int n = neighbours.size();
  for (auto b : neighbours) {
    v_sum.x += b.getVelocity().x;
    v_sum.y += b.getVelocity().y;
  }
  if (neighbours.size() >= 1) {
    v_alignment = Coords{par.a * ((v_sum.x / n) - s.getVelocity().x),
                         par.a * ((v_sum.y / n) - s.getVelocity().y)};
    return v_alignment;
  } else {
    v_alignment = Coords{0.0, 0.0};
    return v_alignment;
  }
}

Coords cohesion_for_single_boid(const Parameters &par, const Flock &stormo,
                                const SingleBoid &s) {
  Coords com{0., 0.};
  Coords v_cohesion;
  auto neighbours = get_neighbours(par.d, s, stormo);
  int n = neighbours.size();
  for (auto b : neighbours) {
    com.x += b.getPosition().x;
    com.y += b.getPosition().y;
  }

  if (neighbours.size() >= 1) {
    v_cohesion = Coords{par.c * ((com.x / n) - s.getPosition().x),
                        par.c * ((com.y / n) - s.getPosition().y)};
    return v_cohesion;
  } else {
    v_cohesion = Coords{0.0, 0.0};
    return v_cohesion;
  }
}

Coords hunt(const SingleBoid &hunter, const Flock &stormo,
            const Parameters &par) {
  Coords com{0., 0.};
  Coords v_hunt;
  for (auto b : stormo.flock) {
    com.x += b.getPosition().x;
    com.y += b.getPosition().y;
  }
  int s = stormo.flock.size();
  if (stormo.flock.size() > 0) {
    v_hunt = Coords{par.h * ((com.x / s) - hunter.getPosition().x),
                    par.h * ((com.y / s) - hunter.getPosition().y)};
  } else {
    v_hunt = Coords{0., 0.};
  }
  return v_hunt;
}

Coords hunter_repulsion(const SingleBoid &hunter, const SingleBoid &prey,
                        const Parameters &par) {
  Coords v_prey;
  if (check_critical_distance(par.d, prey, hunter)) {
    v_prey = Coords{-(par.p * (hunter.getPosition().x - prey.getPosition().x)),
                    -(par.p * (hunter.getPosition().y - prey.getPosition().y))};
  } else {
    v_prey = Coords{0., 0.};
  }
  return v_prey;
}

Coords create_velocity_with_rules_for_single_boid(const Parameters &par,
                                                  const Flock &stormo,
                                                  const SingleBoid &b,
                                                  const SingleBoid &hunter) {
  return b.getVelocity() + separation_for_single_boid(par, stormo, b) +
         alignment_for_single_boid(par, stormo, b) +
         cohesion_for_single_boid(par, stormo, b) +
         hunter_repulsion(hunter, b, par);
}

} // namespace boidz
