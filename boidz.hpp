

#ifndef BOIDZ_HPP
#define BOIDZ_HPP

#include <SFML/Graphics.hpp>
#include <algorithm>
#include <cmath>
#include <vector>

namespace boidz {
struct Coords {
  float x;
  float y;

  Coords() = default;
  Coords(float x_, float y_) : x(x_), y(y_) {}
};

Coords operator+(const Coords &p, const Coords &q);

struct Parameters {
  float d;   // parametro vicini
  float d_s; // paramtero vicini critici
  float s;   // parametro di separazione
  float a;   // parametro di allineamento
  float c;   // parametro di coesione
  float h;   // parametro di caccia
  float p;   // parametro di preda
};

sf::ConvexShape createBoidShape(float size);

class SingleBoid {
  Coords position;
  Coords velocity;
  int cardinality;
  sf::ConvexShape shape;

public:
  SingleBoid();
  SingleBoid(Coords p, Coords v, int i);
  int getCardinality() const;
  Coords getPosition() const;
  Coords getVelocity() const;

  void update_Position_in_time_for_single_boid(float time);
  void update_Velocity_with_rules_for_single_boid(Coords updated_velocity);
  void BorderRestriction(float width, float height);
  void SpeedRestriction(float speed_limit);
  void update_Shape();
  void draw(sf::RenderWindow &window) const;
  void setColor(const sf::Color &c);
};

class Flock {
public:
  std::vector<SingleBoid> flock{};
  Flock() : flock{} {}
  Flock(int N) : flock(N) {}
};

} // namespace boidz

#endif
