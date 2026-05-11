#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "rules.hpp"
using namespace boidz;
// struct con elementi utilizzabili in più test per non doverli ricreare ogni
// volta
struct MocksFlock {
  Flock stormo_vuoto;
  Flock solitario{1};
  Flock mock_stormo{10};
};

struct MocksBoid {
  SingleBoid uccello_fermo;
};

struct MocksPar {};

TEST_CASE("check get_distance function ") {
  SingleBoid mock_bird_1{{1.0, 2.0}, {0.5, 0.5}, 3};
  SingleBoid mock_bird_2{{3.0, 1.5}, {1.0, 0.2}, 12};
  CHECK(get_distance(mock_bird_1, mock_bird_2) ==
        doctest::Approx(2.06).epsilon(0.01));
}