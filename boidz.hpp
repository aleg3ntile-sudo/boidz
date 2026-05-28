

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
};

Coords operator+(const Coords &p, const Coords &q) {
  return {p.x + q.x, p.y + q.y};
}

struct Parameters {
  double s;    // parametro di separazione
  double d;    // parametro vicini
  double d_s;  // paramtero vicini critici
  double a;    // parametro di allineamento
  double c;    // parametro di coesione
  double h;    // parametro di caccia
  double p;    // parametro di preda
};

sf::ConvexShape createBoidShape(double size) {
  sf::ConvexShape shape;
  shape.setPointCount(3);

  shape.setPoint(0, sf::Vector2f(size, 0.));
  shape.setPoint(1, sf::Vector2f(-size * 0.6, -size * 0.5));
  shape.setPoint(2, sf::Vector2f(-size * 0.6, size * 0.5));
  shape.setFillColor(sf::Color(100, 200, 255));

  shape.setOrigin(0., 0.);

  return shape;
}

class SingleBoid {
  Coords position;
  Coords velocity;
  int cardinality;  // e che nessuno osi fiatare
  sf::ConvexShape shape;

 public:
  SingleBoid() : position{}, velocity{}, cardinality(0) {}
  SingleBoid(Coords p, Coords v, int i)
      : position(p), velocity(v), cardinality(i), shape{createBoidShape(5.)} {}
  auto getCardinality() const { return cardinality; }
  auto getPosition() const { return position; }
  auto getVelocity() const { return velocity; }

  void update_Position_in_time_for_single_boid(double time) {
    position.x = position.x + time * velocity.x;
    position.y = position.y + time * velocity.y;
  }

  void update_Shape() {
    double angle = std::atan2(velocity.y, velocity.x) * 180 / M_PI;
    shape.setPosition(sf::Vector2f(position.x, position.y));
    shape.setRotation(angle);
  }

  void update_Velocity_with_rules_for_single_boid(Coords updated_velocity) {
    velocity = updated_velocity;
  }

  void BorderRestriction(double width, double height) {
    if (position.x < 0.)
      position.x += width;
    else {
      if (position.x >= width) {
        position.x -= width;
      }
    }

    if (position.y < 0.)
      position.y += height;
    else {
      if (position.y >= height) {
        position.y -= height;
      }
    }
  }

  void SpeedRestriction(double speed_limit) {
    double speed = std::sqrt(velocity.x * velocity.x + velocity.y * velocity.y);
    if (speed > speed_limit) {
      velocity.x = velocity.x * speed_limit / speed;
      velocity.y = velocity.y * speed_limit / speed;
    }
  }

  void draw(sf::RenderWindow &window) const { window.draw(shape); }
};

class Flock {
 public:
  std::vector<SingleBoid> flock{};
  Flock() : flock{} {}
  Flock(int N) : flock(N) {}
};

}  // namespace boidz

#endif
