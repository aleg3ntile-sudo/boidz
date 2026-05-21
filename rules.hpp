#ifndef RULES_HPP
#define RULES_HPP

#include <iostream>

#include "boidz.hpp"

namespace boidz {
double get_distance(const SingleBoid &b1, const SingleBoid &b2) {
  Coords a = b1.getPosition();
  Coords b = b2.getPosition();

  return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}

bool check_critical_distance(double const &d_s, const SingleBoid &b1,
                             const SingleBoid &b2) {
  Coords a = b1.getPosition();
  Coords b = b2.getPosition();

  if (std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2) <= std::pow(d_s, 2)) {
    return true;
  }
  return false;
}

bool check_neighbours(double const &d, const SingleBoid &b1, SingleBoid &b2) {
  Coords a = b1.getPosition();
  Coords b = b2.getPosition();

  if (std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2) <= std::pow(d, 2)) {
    return true;
  }
  return false;
}

std::vector<SingleBoid> get_neighbours(double const &distance,
                                       const SingleBoid &s, Flock const &f) {
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

void remove_boid_from_neighbours(std::vector<SingleBoid> &neighbours) {
  neighbours.clear();
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

  Coords v_separation{-par.s * sum_x, -par.s * sum_y};
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
  if (neighbours.size() > 1) {
    v_alignment = {par.a * (v_sum.x / n - s.getVelocity().x),
                   par.a * (v_sum.y / n - s.getVelocity().y)};
    return v_alignment;
  } else {
    v_alignment = {0.0, 0.0};
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

  if (neighbours.size() > 1) {
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

Coords create_velocity_with_rules_for_single_boid(const Parameters &par,
                                                  const Flock &stormo,
                                                  const SingleBoid &b) {
  return b.getVelocity() + separation_for_single_boid(par, stormo, b) +
         alignment_for_single_boid(par, stormo, b) +
         cohesion_for_single_boid(par, stormo, b);
}
} // namespace boidz
#endif
