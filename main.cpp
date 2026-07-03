#include "rules.hpp"
#include "stats.hpp"
#include "graphics.hpp"

using namespace boidz;
int main()
{
  int flock_size{};
  Parameters params{};
  try
  {
    params.d = read_parameter("Neighbours distance ", 20.f, 100.f);
    params.d_s = read_parameter("Critical distance ", 5.f, 25.f);
    params.s = read_parameter("Separation parameter ", 0.5f, 5.f);
    params.a = read_parameter("Alignment parameter ", 0.05f, 1.f);
    params.c = read_parameter("Cohesion parameter ", 0.01f, 0.05f);
    flock_size = read_parameter("Flock size ", 1, 200);
    params.h = 1.;
    params.p = 0.5;
  }
  catch (const std::runtime_error &e)
  {
    std::cerr << "Errore" << '\n';
    return EXIT_FAILURE;
  }

  Flock stormo{};
  stormo.generate_boids(flock_size);

  Boid hunter{{400., 300.}, {0., 0.}, -1};
  hunter.set_color(sf::Color(255, 100, 100));

  sf::RenderWindow window(sf::VideoMode(800, 600), "Boids");
  window.setFramerateLimit(60);
  sf::RenderWindow window2(sf::VideoMode(500, 600), "Velocity");
  window2.setFramerateLimit(60);
  window2.setPosition(sf::Vector2i(12, 300));
  sf::RenderWindow window3(sf::VideoMode(500, 600), "Distance");
  window3.setFramerateLimit(60);
  window3.setPosition(sf::Vector2i(1422, 300));

  sf::Clock clock;

  sf::CircleShape point(2.f);
  point.setFillColor(sf::Color::Red);
  sf::CircleShape dot(2.f);
  dot.setFillColor(sf::Color::Green);

  sf::VertexArray axis(sf::PrimitiveType::Lines, 4);
  create_axis(axis);

  sf::Text label;
  sf::Text label2;
  sf::Font font;
  font.loadFromFile("font.ttf");
  build_label(label, font);
  build_label(label2, font);

  const size_t max_points = 450;
  Statistics stats;

  float timer = 0.;
  const float delay = 5.;

  while (window.isOpen() || window2.isOpen() || window3.isOpen())
  {
    sf::Event event;
    while (window.pollEvent(event))
    {
      if (event.type == sf::Event::Closed)
      {
        window.close();
      }
    }
    while (window2.pollEvent(event))
    {
      if (event.type == sf::Event::Closed)
      {
        window2.close();
      }
    }
    while (window3.pollEvent(event))
    {
      if (event.type == sf::Event::Closed)
      {
        window3.close();
      }
    }

    if (window.isOpen())
    {
      float dt = clock.restart().asSeconds();
      timer += dt;
      update_flock(stormo, params, hunter, dt);

      if (timer >= delay)
      {
        update_hunter(hunter, stormo, params, dt);
      }

      Coords speed_with_error = mean_and_error(stormo, "speed");
      stats.vector_of_mean_speeds.push_back(speed_with_error.x);
      remove_overflow(stats.vector_of_mean_speeds, max_points);
      stats.vector_of_errors_speed.push_back(speed_with_error.y);
      remove_overflow(stats.vector_of_errors_speed, max_points);

      Coords distance_with_error = mean_and_error(stormo, "distance");
      stats.vector_of_mean_distances.push_back(distance_with_error.x);
      remove_overflow(stats.vector_of_mean_distances, max_points);
      stats.vector_of_errors_distance.push_back(distance_with_error.y);
      remove_overflow(stats.vector_of_errors_distance, max_points);

      window.clear(sf::Color(15, 17, 26));
      for (auto &b : stormo.flock)
      {
        b.draw(window);
      }
      hunter.draw(window);
      window.display();
    }

    if (window2.isOpen())
    {
      window2.clear();
      window2.draw(axis);

      draw_points(stats.vector_of_mean_speeds, stats.vector_of_errors_speed,
                  label, point, window2);

      window2.draw(label);
      window2.display();
    }

    if (window3.isOpen())
    {
      window3.clear();
      window3.draw(axis);

      draw_points(stats.vector_of_mean_distances, stats.vector_of_errors_distance,
                  label2, dot, window3);

      window3.draw(label2);
      window3.display();
    }
  }
}
