#include <SFML/Graphics.hpp>
#include <random>

#include "boidz.hpp"
#include "rules.hpp"

// ipotetico main smfl
using namespace boidz;
int main() {
  sf::RenderWindow window(
      sf::VideoMode(800, 600),   // inizializzerò poi height e width (o lasciamo
                                 // sempre fissi 600-800?)
      "Boids");                  // informazioni sulla finestra grafica
  window.setFramerateLimit(60);  // fps

  // sf::Texture texture; // creiamo una texture
  // texture.loadFromFile("bobsenzasfondo.png");

  sf::CircleShape dot(1.f);
  dot.setFillColor(sf::Color::Red);

  // main originale
  Flock stormo{};
  std::random_device r;
  std::default_random_engine eng(r());
  std::uniform_real_distribution<double> px{0., 200.};
  // importante avere come limiti le dimensioni dello schermo
  std::uniform_real_distribution<double> py{0., 150.};
  std::uniform_real_distribution<double> vel{-100., 100.};
  // che velocità sarebbe indicata?
  for (int j{0}; j != 100; ++j) {
    Coords p{px(eng), py(eng)};
    Coords v{vel(eng), vel(eng)};
    SingleBoid b = SingleBoid(p, v, j);
    stormo.flock.push_back(b);
  }

  SingleBoid hunter{{400., 300.}, {0., 0.}, 104};

  // for (auto &x : stormo.flock)
  //{
  //     // per settare posizioni, scalare (è un metodo)
  //     x.setupSprite(texture);
  // }

  Parameters params{1.2, 50.0, 7.5, 0.3, 0.02, 1., 0.5};

  sf::Clock clock;

  double timer = 0.;

  while (window.isOpen()) {
    double dt = clock.restart().asSeconds();
    timer += dt;
    double delay = 20.;
    sf::Event event;  // tiene la finestra viva anche se non crrreiamo eventi,
                      // poll event evita che la finestra si blocchi
    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed) {
        window.close();
      }
    }

    /*for (auto &x : stormo.flock) {
      Coords ruled_velocity =
          update_Velocity_with_rules_for_single_boid(params, stormo, x);
      x.update_Velocity_with_rules_for_single_boid(ruled_velocity);
      x.update_Position_in_time_for_single_boid(dt);
      x.applyBorderRestriction(1800., 1600.);
      x.aggiornaSprite(); */
    // problema di architettura del loop, i boids si aggiornano con le
    // velocità e poszioni future dello stormo, anzichè reagire tuti a quelle
    // presenti?
    std::vector<Coords> updated_velocities{};
    for (auto &x : stormo.flock) {
      Coords new_velocity =
          create_velocity_with_rules_for_single_boid(params, stormo, x, hunter);
      updated_velocities.push_back(new_velocity);
    }
    for (long unsigned int i = 0; i != stormo.flock.size(); ++i) {
      stormo.flock[i].update_Velocity_with_rules_for_single_boid(
          updated_velocities[i]);
      stormo.flock[i].update_Position_in_time_for_single_boid(dt);
      stormo.flock[i].BorderRestriction(800., 600.);
      stormo.flock[i].SpeedRestriction(130);
      stormo.flock[i].aggiornaSprite();
    }

    if (timer >= delay) {
      Coords new_hunt_velocity = hunt(hunter, stormo, params);
      hunter.update_Velocity_with_rules_for_single_boid(new_hunt_velocity);
      hunter.update_Position_in_time_for_single_boid(dt);
      hunter.BorderRestriction(800., 600.);
      hunter.SpeedRestriction(100);
      hunter.aggiornaSprite();
    }
    // render
    window.clear(sf::Color(15, 17, 26));  // blu notte
    for (auto &b : stormo.flock) {
      dot.setPosition(b.getPosition().x, b.getPosition().y);
      // b.draw(window);
      window.draw(dot);
    }
    dot.setPosition(hunter.getPosition().x, hunter.getPosition().y);
    window.draw(dot);
    window.display();  // fai vedere la finestra
  }
  return 0;
}
