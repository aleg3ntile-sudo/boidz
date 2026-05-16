
#include "boidz.hpp"
#include "rules.hpp"
#include <random>

// ipotetico main smfl
using namespace boidz;
int main() {
  sf::RenderWindow window(
      sf::VideoMode(800, 600),  // inizializzerò poi height e width (o lasciamo
                                // sempre fissi 600-800?)
      "Boids");                 // informazioni sulla finestra grafica
  window.setFramerateLimit(60); // fps

  // per importare grafica png
  sf::Texture texture; // creiamo una texture
  texture.loadFromFile("boidsred_real.png");

  // main originale
  Flock stormo{};
  std::random_device r;
  std::default_random_engine eng(r());
  std::uniform_real_distribution<double> px{0., 800.};
  std::uniform_real_distribution<double> py{0., 600.};
  std::uniform_real_distribution<double> vel{0., 50.};

  // funzione per generare lo stormo con i parametri di input
  for (int j{0}; j != 10; ++j) {
    Coords p{px(eng), py(eng)};
    Coords v{vel(eng), vel(eng)};
    SingleBoid b = SingleBoid(p, v, j);
    stormo.flock.push_back(b);
  }

  check_status(stormo);

  // per ogni boids nel vettore
  for (auto &x : stormo.flock) {
    // per settare posizioni, scalare
    x.setupSprite(texture);
  }

  // crea classe parametri
  Parameters params{0.1, 50.0, 15.0, 0.05, 0.01};
  /*Parameters params;
  std::cout << "Separazione (s): ";      std::cin >> params.s;
  std::cout << "Raggio vicini (d): ";    std::cin >> params.d;
  std::cout << "Raggio critico (d_s): "; std::cin >> params.d_s;
  std::cout << "Allineamento (a): ";     std::cin >> params.a;
  std::cout << "Coesione (c): ";         std::cin >> params.c;
  */

  // crea orologio
  sf::Clock clock;

  clock.restart();
  while (window.isOpen()) {
    double dt = clock.restart().asSeconds();

    sf::Event event;
    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed) {
        window.close();
      }
    }

    for (auto &x : stormo.flock) {
      Coords ruled_velocity =
          update_Velocity_with_rules_for_single_boid(params, stormo, x);
      x.update_Velocity_with_rules_for_single_boid(ruled_velocity);
      x.update_Position_in_time_for_single_boid(dt);
      x.applyBorderRestriction(800., 600.);
      x.aggiornaSprite(800.f, 600.f);
    }

    // render
    window.clear(sf::Color(15, 17, 26)); // blu notte
    for (auto &b : stormo.flock) {
      b.draw(window);
    }

    window.display(); // fai vedere la finestra
  }
  return 0;
}
