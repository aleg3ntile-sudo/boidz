#ifndef RULES_HPP
#define RULES_HPP

#include <cmath>
#include <iostream>

#include "boidz.hpp"

namespace boidz {
double get_distance(SingleBoid &b1, SingleBoid &b2) {
  Coords a = b1.getPosition();
  Coords b = b2.getPosition();

  return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}

bool check_critical_distance(double const &d_s, SingleBoid &b1,
                             SingleBoid &b2) {
  Coords a = b1.getPosition();
  Coords b = b2.getPosition();

  if (std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2) <= std::pow(d_s, 2)) {
    return true;
  }
  return false;
}

bool check_neighbours(double const &d, SingleBoid b1, SingleBoid b2) {
  Coords a = b1.getPosition();
  Coords b = b2.getPosition();

  if (std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2) <= std::pow(d, 2)) {
    return true;
  }
  return false;
}

std::vector<SingleBoid> get_neighbours(double const &distance, SingleBoid &s,
                                       Flock const &f) {
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

Coords separation_for_single_boid(Parameters const &par, Flock &stormo,
                                  SingleBoid &a) {
  double sum_x{0};
  double sum_y{0};
  auto neighbours = get_neighbours(par.d, a, stormo);
  for (auto b : neighbours) {
    if (check_critical_distance(par.d_s, a, b)) {
      sum_x += b.getPosition().x - a.getPosition().x;
      sum_y += b.getPosition().y - a.getPosition().y;
    }
  }

  Coords v_separation{-par.s * sum_x, -par.s * sum_y};
  return v_separation;
}

Coords alignment_for_single_boid(Parameters const &par, Flock &stormo,
                                 SingleBoid &s) {
  Coords v_alignment;
  Coords v_sum{0., 0.};
  for (auto b : stormo.flock) {
    if (b.getCardinality() != s.getCardinality()) {
      v_sum.x += b.getVelocity().x;
      v_sum.y += b.getVelocity().y;
    }
  }
  if (stormo.flock.size() > 1) {
    v_alignment = {
        par.a * (v_sum.x / (stormo.flock.size() - 1) - s.getVelocity().x),
        par.a * (v_sum.y / (stormo.flock.size() - 1) - s.getVelocity().y)};
    return v_alignment;
  } else {
    v_alignment = {0.0, 0.0};
    return v_alignment;
  }
}

Coords cohesion_for_single_boid(Parameters const &par, Flock &stormo,
                                SingleBoid &s) {
  Coords com{0., 0.};
  Coords v_cohesion;
  int n = stormo.flock.size() - 1;
  for (auto b : stormo.flock) {
    if (b.getCardinality() != s.getCardinality()) {
      com.x += b.getPosition().x;
      com.y += b.getPosition().y;
    }
  }
  if (stormo.flock.size() > 1) {
    v_cohesion = {par.c * ((com.x / n) - s.getPosition().x),
                  par.c * ((com.y / n) - s.getPosition().y)};
    return v_cohesion;
  } else {
    v_cohesion = {0.0, 0.0};
    return v_cohesion;
  }
}



void check_status(Flock &stormo) {
  for (auto b : stormo.flock) {
    std::cout << "Cardinality : " << b.getCardinality() << '\n'
              << "X Position : " << b.getPosition().x << '\n'
              << "Y Position : " << b.getPosition().y << '\n'
              << "X Velocity : " << b.getVelocity().x << '\n'
              << "Y Velocity : " << b.getVelocity().y << '\n';
  }
}

Coords update_Velocity_with_rules_for_single_boid(Parameters par, Flock stormo,
                                                  SingleBoid b) {
  auto new_velocity = b.getVelocity();
  new_velocity = new_velocity + separation_for_single_boid(par, stormo, b) +
                 alignment_for_single_boid(par, stormo, b) +
                 cohesion_for_single_boid(par, stormo, b);
  return new_velocity;
}
} // namespace boidz
#endif
