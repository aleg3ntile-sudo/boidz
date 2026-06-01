#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "stats.hpp"
#include <cmath>
#include <numeric>
using namespace boidz;

TEST_CASE("mean_velocity: empty flock returns (0,0)") {
  Flock f{};
  Coords result = mean_velocity(f);
  CHECK(result.x == doctest::Approx(0.f));
  CHECK(result.y == doctest::Approx(0.f));
}

TEST_CASE("mean_velocity: two boids") {
  Flock f{};
  f.flock.push_back(SingleBoid{{0.f, 0.f}, {3.f, 4.f}, 0});
  f.flock.push_back(SingleBoid{{0.f, 0.f}, {1.f, 2.f}, 1});

  Coords result = mean_velocity(f);
  CHECK(result.x == doctest::Approx(2.f));
  CHECK(result.y == doctest::Approx(3.f));
}

TEST_CASE("get_vector_of_distances: empty flock returns empty vector") {
  Flock f{};
  auto result = get_vector_of_distances(f);
  CHECK(result.empty());
}

TEST_CASE("get_vector_of_distances: two boids return two distances") {
  Flock f{};
  f.flock.push_back(SingleBoid{{0.f, 0.f}, {0.f, 0.f}, 0});
  f.flock.push_back(SingleBoid{{3.f, 4.f}, {0.f, 0.f}, 1});

  auto result = get_vector_of_distances(f);
  CHECK(result.size() == 2);
  CHECK(result[0] == doctest::Approx(5.f));
  CHECK(result[1] == doctest::Approx(5.f));
}

TEST_CASE("mean_distance: empty flock returns 0") {
  Flock f{};
  CHECK(mean_distance(f) == doctest::Approx(0.f));
}

TEST_CASE("mean_distance: two boids with known distance") {
  Flock f{};
  f.flock.push_back(SingleBoid{{0.f, 0.f}, {0.f, 0.f}, 0});
  f.flock.push_back(SingleBoid{{3.f, 4.f}, {0.f, 0.f}, 1});

  CHECK(mean_distance(f) == doctest::Approx(5.f));
}

TEST_CASE("standard_deviation<float>: empty flock returns 0") {
  std::vector<float> v{};
  CHECK(standard_deviation(v) == doctest::Approx(0.f));
}

TEST_CASE("standard_deviation<float>: known values") {
  std::vector<float> v{1.f, 2.f, 3.f, 4.f, 5.f};
  CHECK(standard_deviation(v) == doctest::Approx(std::sqrt(2.f)));
}

TEST_CASE("standard_deviation<SingleBoid>: empty flock returns 0") {
  std::vector<SingleBoid> v{};
  CHECK(standard_deviation(v) == doctest::Approx(0.f));
}

TEST_CASE(
    "standard_deviation<SingleBoid>: boids with equal speed give std dev = 0") {
  std::vector<SingleBoid> v{};
  v.push_back(SingleBoid{{0.f, 0.f}, {2.f, 2.f}, 0});
  v.push_back(SingleBoid{{0.f, 0.f}, {2.f, 2.f}, 1});
  v.push_back(SingleBoid{{0.f, 0.f}, {2.f, 2.f}, 2});

  CHECK(standard_deviation(v) == doctest::Approx(0.f));
}