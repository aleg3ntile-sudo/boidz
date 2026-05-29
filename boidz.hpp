

#ifndef BOIDZ_HPP
#define BOIDZ_HPP

#include <SFML/Graphics.hpp>
#include <algorithm>
#include <cmath>
#include <vector>

namespace boidz {
struct Coords {
  double x;
  double y;

  Coords() = default;
  Coords(double x_, double y_) : x(x_), y(y_) {}
};

Coords operator+(const Coords &p, const Coords &q);

struct Parameters {
  double s;   // parametro di separazione
  double d;   // parametro vicini
  double d_s; // paramtero vicini critici
  double a;   // parametro di allineamento
  double c;   // parametro di coesione
  double h;   // parametro di caccia
  double p;   // parametro di preda
};

sf::ConvexShape createBoidShape(double size);

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

  void update_Position_in_time_for_single_boid(double time);
  void update_Velocity_with_rules_for_single_boid(Coords updated_velocity);
  void BorderRestriction(double width, double height);
  void SpeedRestriction(double speed_limit);
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
