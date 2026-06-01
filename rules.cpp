#include "rules.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

namespace boidz {

float get_distance(const SingleBoid &b1, const SingleBoid &b2) {
  Coords a = b1.getPosition();
  Coords b = b2.getPosition();
  assert(std::isfinite(a.x) && std::isfinite(a.y));
  assert(std::isfinite(b.x) && std::isfinite(b.y));

  float distance = std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));

  assert(distance >= 0.);
  return distance;
}

bool check_critical_distance(const float &d_s, const SingleBoid &b1,
                             const SingleBoid &b2) {
  assert(d_s >= 0.);
  Coords a = b1.getPosition();
  Coords b = b2.getPosition();
  float critical_distance =
      std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2) <= std::pow(d_s, 2);

  assert(critical_distance >= 0.);
  return critical_distance;
}

bool check_neighbours(const float &d, const SingleBoid &b1,
                      const SingleBoid &b2) {
  Coords a = b1.getPosition();
  Coords b = b2.getPosition();

  return std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2) <= std::pow(d, 2);
}

std::vector<SingleBoid> get_neighbours(const float &distance,
                                       const SingleBoid &s, const Flock &f) {
  assert(distance >= 0.);
  std::vector<SingleBoid> neighbours{};

  for (auto x : f.flock) {
    if (check_neighbours(distance, s, x) == true) {
      if (s.getCardinality() != x.getCardinality()) {
        neighbours.push_back(x);
      }
    }
  }
  assert(neighbours.size() <= f.flock.size());
  return neighbours;
}

Coords separation_for_single_boid(const Parameters &par, const Flock &stormo,
                                  const SingleBoid &a) {
  float sum_x{0};
  float sum_y{0};
  auto neighbours = get_neighbours(par.d, a, stormo);
  assert(par.d >= 0.);

  for (auto b : neighbours) {
    if (check_critical_distance(par.d_s, a, b)) {
      sum_x += b.getPosition().x - a.getPosition().x;
      sum_y += b.getPosition().y - a.getPosition().y;
    }
  }

  Coords v_separation{-(par.s * sum_x), -(par.s * sum_y)};
  assert(par.s >= 0.);
  assert(std::isfinite(v_separation.x) && std::isfinite(v_separation.y));
  return v_separation;
}

Coords alignment_for_single_boid(const Parameters &par, const Flock &stormo,
                                 const SingleBoid &s) {
  Coords v_alignment;
  Coords v_sum{0., 0.};
  auto neighbours = get_neighbours(par.d, s, stormo);
  int n = neighbours.size();
  assert(n >= 0);

  for (auto b : neighbours) {
    v_sum.x += b.getVelocity().x;
    v_sum.y += b.getVelocity().y;
  }
  if (neighbours.size() >= 1) {
    v_alignment = Coords{par.a * ((v_sum.x / n) - s.getVelocity().x),
                         par.a * ((v_sum.y / n) - s.getVelocity().y)};
    assert(par.a >= 0.);
    assert(std::isfinite(v_alignment.x) && std::isfinite(v_alignment.y));
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
    com.x += b.getPosition().x / n;
    com.y += b.getPosition().y / n;
  }

  if (neighbours.size() >= 1) {
    v_cohesion = Coords{par.c * (com.x - s.getPosition().x),
                        par.c * (com.y - s.getPosition().y)};
    assert(par.c >= 0.);
    assert(std::isfinite(v_cohesion.x) && std::isfinite(v_cohesion.y));
    return v_cohesion;
  } else {
    v_cohesion = Coords{0.0, 0.0};
    return v_cohesion;
  }
}

Coords hunt_the_flock(const SingleBoid &hunter, const Flock &stormo,
                      const Parameters &par) {
  Coords com{0., 0.};
  Coords v_hunt;
  for (auto b : stormo.flock) {
    int s = stormo.flock.size();
    assert(s >= 0);
    com.x += b.getPosition().x / s;
    com.y += b.getPosition().y / s;
  }
  if (stormo.flock.size() > 0) {
    v_hunt = Coords{par.h * (com.x - hunter.getPosition().x),
                    par.h * (com.y - hunter.getPosition().y)};
    assert(par.h >= 0.);
    assert(std::isfinite(v_hunt.x) && std::isfinite(v_hunt.y));
  } else {
    v_hunt = Coords{0., 0.};
  }
  return v_hunt;
}

Coords hunt_neighbours(const SingleBoid &hunter, const Flock &stormo,
                       const Parameters &par) {
  Coords v_chase_neighbours{};
  for (auto b : stormo.flock) {
    if (check_neighbours(50., hunter, b)) {
      v_chase_neighbours =
          Coords{par.h * (b.getPosition().x - hunter.getPosition().x),
                 par.h * (b.getPosition().y - hunter.getPosition().y)};
    }
  }
  return v_chase_neighbours;
}

Coords hunter_repulsion(const SingleBoid &hunter, const SingleBoid &prey,
                        const Parameters &par) {
  Coords v_prey;
  if (check_critical_distance(par.d, prey, hunter)) {
    v_prey = Coords{-(par.p * (hunter.getPosition().x - prey.getPosition().x)),
                    -(par.p * (hunter.getPosition().y - prey.getPosition().y))};
    assert(par.p >= 0.);
    assert(std::isfinite(v_prey.x) && std::isfinite(v_prey.y));
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

}  // namespace boidz
