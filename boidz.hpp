

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

class SingleBoid {
  Coords position;
  Coords velocity;
  int cardinality;  // e che nessuno osi fiatare
  sf::Sprite sprite;

 public:
  SingleBoid() : position{}, velocity{}, cardinality(0) {}
  SingleBoid(Coords p, Coords v, int i)
      : position(p), velocity(v), cardinality(i) {}
  auto getCardinality() const { return cardinality; }
  auto getPosition() const { return position; }
  auto getVelocity() const { return velocity; }

  void update_Position_in_time_for_single_boid(double time) {
    position.x = position.x + time * velocity.x;
    position.y = position.y + time * velocity.y;
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
  // Coords update_Velocity_in_time_for_single_boid() {}
  void setupSprite(sf::Texture &texture) {
    sprite.setTexture(texture);
    sf::FloatRect bounds = sprite.getLocalBounds();  // confini dell'immagine
    sprite.setOrigin(
        bounds.width / 2.f,
        bounds.height /
            2.f);  // origine dell'immagine del boid che poi manipoliamo
    sprite.setScale(0.07f, 0.07f);  // quanto va scalata ( anche in base ai
                                    // parametri e alla posizione dell'origin)
  }  // per settare la texture

  void aggiornaSprite() {
    sprite.setPosition(
        position.x,
        position.y);  // sprite f aparte dei "transformable" di sfml,
                      // quindi eredita questo e gli altri metodi
                      // senza bisogno di includerli manualmente
  }

  void draw(sf::RenderWindow &window) const { window.draw(sprite); }
};

class Flock {
 public:
  std::vector<SingleBoid> flock{};
  Flock() : flock{} {}
  Flock(int N) : flock(N) {}
};

}  // namespace boidz

#endif
