#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "rules.hpp"
using namespace boidz;

// operator +
TEST_CASE("check + operator on Coords") {
  Coords a{1., -9.};
  Coords b{8., 2.3};
  Coords result = a + b;
  CHECK(result.x == doctest::Approx(9.00));
  CHECK(result.y == doctest::Approx(-6.7));
}

// get_distance
TEST_CASE("check get_distance function") {
  SingleBoid b1{{1.0, 2.0}, {0.5, 0.5}, 3};
  SingleBoid b2{{3.0, 1.5}, {1.0, 0.2}, 12};
  CHECK(get_distance(b1, b2) == doctest::Approx(2.06).epsilon(0.01));
}

TEST_CASE("check get_distance function") {
  SingleBoid b1{{1.0, 2.3}, {3.1, 2.0}, 1};
  SingleBoid b2;
  CHECK(get_distance(b1, b2) == doctest::Approx(2.51).epsilon(0.01));
}

TEST_CASE("check get_distance function - one boid") {
  SingleBoid b1{{0.0, 1.0}, {4.3, 2.0}, 1};
  CHECK(get_distance(b1, b1) == 0);
}

// check_critical_distance
TEST_CASE("check check_critical_distance function") {
  SingleBoid b1{{0.0, 0.0}, {0.0, 0.0}, 1};
  SingleBoid b2{{2.0, 0.0}, {0.0, 0.0}, 1};
  CHECK(check_critical_distance(2.0, b1, b2) == true);
  CHECK(check_critical_distance(1.0, b1, b2) == false);
}

// check_neighbours
TEST_CASE("check check_neighbours function") {
  SingleBoid b1{{0.0, 0.0}, {0.0, 0.0}, 1};
  SingleBoid b2{{3.0, 4.0}, {0.0, 0.0}, 1};
  CHECK(check_neighbours(5.0, b1, b2) == true);
  CHECK(check_neighbours(4.9, b1, b2) == false);
}

// get_neighbours
TEST_CASE("check get_neighbours function - no neighbours") {
  Flock f;
  SingleBoid b1{{0.0, 0.0}, {0.0, 0.0}, 1};
  SingleBoid b2{{10.0, 0.0}, {0.0, 0.0}, 2};
  SingleBoid b3{{20.0, 0.0}, {0.0, 0.0}, 3};
  f.flock.push_back(b1);
  f.flock.push_back(b2);
  f.flock.push_back(b3);

  auto result = get_neighbours(2.0, b1, f);
  CHECK(result.size() == 0);
}

TEST_CASE("check get_neighbours function - some neighbours") {
  Flock f;
  SingleBoid b1{{0.0, 0.0}, {0.0, 0.0}, 1};
  SingleBoid b2{{1.0, 0.0}, {0.0, 0.0}, 2};
  SingleBoid b3{{10.0, 0.0}, {0.0, 0.0}, 3};
  f.flock.push_back(b1);
  f.flock.push_back(b2);
  f.flock.push_back(b3);

  auto result = get_neighbours(2.0, b1, f);
  CHECK(result.size() == 1);
}

TEST_CASE("check get_neighbours function - all are neighbours") {
  Flock f;
  SingleBoid b1{{0.0, 0.0}, {0.5, 0.0}, 1};
  SingleBoid b2{{0.0, 1.0}, {0.0, 0.5}, 2};
  SingleBoid b3{{1.0, 0.0}, {0.0, 1.0}, 3};

  f.flock.push_back(b1);
  f.flock.push_back(b2);
  f.flock.push_back(b3);

  auto result = get_neighbours(2.0, b1, f);
  CHECK(result.size() == 2);
}

// separation_for_single_boid
TEST_CASE("check separation_for_singleboid function") {
  Parameters par{5.0, 5.0, 1.0, 0.0, 0.0, 0.0, 0.0};
  Flock f;
  SingleBoid a{{0.0, 0.0}, {0.0, 0.0}, 1};
  SingleBoid b{{1.0, 0.0}, {0.0, 0.0}, 2};
  f.flock.push_back(a);
  f.flock.push_back(b);

  Coords result = separation_for_single_boid(par, f, a);

  CHECK(result.x == doctest::Approx(-1.0));
  CHECK(result.y == doctest::Approx(0.0));
}

TEST_CASE(
    "check separation_for_singleboid function - one boid is too distant") {
  Parameters par{5.0, 5.0, 1.0, 0.0, 0.0, 0.0, 0.0};
  Flock f;
  SingleBoid a{{0.0, 0.0}, {0.0, 0.0}, 1};
  SingleBoid b{{1.0, 0.0}, {0.0, 0.0}, 2};
  SingleBoid c{{40.0, 0.0}, {0.0, 0.0}, 3};
  f.flock.push_back(a);
  f.flock.push_back(b);
  f.flock.push_back(c);

  Coords result = separation_for_single_boid(par, f, a);

  CHECK(result.x == doctest::Approx(-1.0));
  CHECK(result.y == doctest::Approx(0.0));
}

// alignment_for_single_boid
TEST_CASE("check alignment_for_single_boid function") {
  Parameters par{5.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
  Flock f;
  SingleBoid a{{1.0, 0.0}, {1.0, 0.0}, 1};
  SingleBoid b{{0.0, 1.0}, {2.0, 1.0}, 2};
  f.flock.push_back(a);
  f.flock.push_back(b);
  Coords result = alignment_for_single_boid(par, f, a);

  CHECK(result.x == doctest::Approx(1.0));
  CHECK(result.y == doctest::Approx(1.0));
}

TEST_CASE("check alignment_for_single_boid function") {
  Parameters par{0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0};
  Flock f;
  SingleBoid a{{1.5, 0.5}, {1.0, -1.0}, 1};
  f.flock.push_back(a);
  Coords result = alignment_for_single_boid(par, f, a);

  CHECK(result.x == doctest::Approx(0.0));
  CHECK(result.y == doctest::Approx(0.0));
}

TEST_CASE("check alignment_for_single_boid function") {
  Parameters par{0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0};
  Flock f;
  SingleBoid a{{1.5, 0.5}, {1.0, -1.0}, 1};
  Coords result = alignment_for_single_boid(par, f, a);

  CHECK(result.x == doctest::Approx(0.0));
  CHECK(result.y == doctest::Approx(0.0));
}

TEST_CASE("check alignment_for_single_boid function") {
  Parameters par{0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0};
  Flock f;
  SingleBoid a{{1.5, 0.5}, {1.0, -1.0}, 1};
  SingleBoid b{{-1.0, 0.0}, {2.0, 3.0}, 2};
  f.flock.push_back(a);
  Coords result = alignment_for_single_boid(par, f, b);

  CHECK(result.x == doctest::Approx(0.0));
  CHECK(result.y == doctest::Approx(0.0));
}

// cohesion_for_single_boid
TEST_CASE("check cohesion_for_single_boid function") {
  Parameters par{5.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0};
  Flock f;
  SingleBoid a{{1.0, -2.0}, {0.0, 1.0}, 1};
  SingleBoid b{{-1.0, 2.5}, {1.0, 1.0}, 2};
  f.flock.push_back(a);
  f.flock.push_back(b);
  Coords result = cohesion_for_single_boid(par, f, a);

  CHECK(result.x == doctest::Approx(-2.0));
  CHECK(result.y == doctest::Approx(4.5));
}

TEST_CASE("check cohesion_for_single_boid function") {
  Parameters par{0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0};
  Flock f;
  SingleBoid a{{1.0, -2.0}, {0.0, 1.0}, 1};
  f.flock.push_back(a);
  Coords result = cohesion_for_single_boid(par, f, a);

  CHECK(result.x == doctest::Approx(0.0));
  CHECK(result.y == doctest::Approx(0.0));
}

TEST_CASE("check cohesion_for_single_boid function - one boid is too distant") {
  Parameters par{5.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0};
  Flock f;
  SingleBoid a{{1.0, -2.0}, {0.0, 1.0}, 1};
  SingleBoid b{{-1.0, 2.5}, {1.0, 1.0}, 2};
  SingleBoid c{{67.0, 1.0}, {1.0, 1.0}, 3};
  f.flock.push_back(a);
  f.flock.push_back(b);
  f.flock.push_back(c);
  Coords result = cohesion_for_single_boid(par, f, a);

  CHECK(result.x == doctest::Approx(-2.0));
  CHECK(result.y == doctest::Approx(4.5));
}

// hunt
TEST_CASE("check hunt function - no boids") {
  SingleBoid h{{0., 0.}, {0., 0.}, 1};
  Parameters par{0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0};
  Flock f;
  Coords result = hunt_the_flock(h, f, par);
  CHECK(result.x == (0.));
  CHECK(result.y == (0.));
}

TEST_CASE("check hunt function - three boids") {
  Parameters par{0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0};
  SingleBoid h{{7.0, 1.0}, {8.0, 5.0}, 11};
  Flock f;
  SingleBoid a{{6., 7}, {0., 1.}, 1};
  SingleBoid b{{1., 1.}, {1.5, 0.5}, 2};
  SingleBoid c{{0., 1.}, {1.3, 0.8}, 3};
  f.flock.push_back(a);
  f.flock.push_back(b);
  f.flock.push_back(c);
  Coords result = hunt_the_flock(h, f, par);
  CHECK(result.x == doctest::Approx(-4.666666));
  CHECK(result.y == doctest::Approx(2.0));
}

TEST_CASE("check hunt function - simmetrical configuration") {
  Parameters par{0.0, 0.0, 0.0, 0.0, 0.0, 13.0, 1.0};
  SingleBoid h{{0.0, 0.0}, {8.0, 5.0}, 11};
  Flock f;
  SingleBoid a{{-4., 0.}, {0., 1.}, 1};
  SingleBoid b{{4., 0.}, {2.5, 1.2}, 2};
  SingleBoid c{{0., -4.}, {1.3, 0.8}, 3};
  SingleBoid d{{0., 4.}, {-3.9, 1.1}, 4};
  f.flock.push_back(a);
  f.flock.push_back(b);
  f.flock.push_back(c);
  f.flock.push_back(d);
  Coords result = hunt_the_flock(h, f, par);
  CHECK(result.x == 0.);
  CHECK(result.y == 0.);
}

// hunter_repulsion
TEST_CASE("check hunter_repulsion function - no hunter in sight") {
  Parameters par{0.0, 0.0, 0.0, 0.0, 0.0, 13.0, 3.0};
  SingleBoid h{{30., 40.}, {0.0, 5.0}, 11};
  Flock f;
  SingleBoid a{{2., 1.}, {0., 1.}, 1};
  f.flock.push_back(a);
  Coords result = hunter_repulsion(h, a, par);
  CHECK(result.x == 0.);
  CHECK(result.y == 0.);
}

TEST_CASE("check hunter_repulsion function - hunter in sight") {
  Parameters par{15.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
  SingleBoid h{{10.0, 10.0}, {0.5, 1.0}, 1};
  SingleBoid prey{{5.5, 1.0}, {-4.0, 2.0}, 11};
  Coords result = hunter_repulsion(h, prey, par);
  CHECK(result.x == -4.5);
  CHECK(result.y == -9.0);
}

// create_velocity_with_rules
TEST_CASE("check create_velocity_with_rules_for_single_boid function") {
  Parameters par{5.0, 1.0, 10.0, 1.0, 3.0, 0.0, 0.0};
  SingleBoid a{{-3.0, -2.0}, {5.0, 2.0}, 1};
  SingleBoid b{{1.0, 1.0}, {10.0, 1.0}, 2};
  Flock f;
  f.flock.push_back(a);
  f.flock.push_back(b);
  SingleBoid hunter{{500., 500.0}, {0.0, 0.0}, 11};
  Coords result = create_velocity_with_rules_for_single_boid(par, f, b, hunter);
  CHECK(result.x == doctest::Approx(-7.));
  CHECK(result.y == doctest::Approx(-7.));
}