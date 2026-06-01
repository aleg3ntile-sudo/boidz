#include "rules.hpp"
#include "stats.hpp"
#include <algorithm>
#include <random>

using namespace boidz;
int main()
{
  int flock_size{};
  Parameters params{};
  try
  {
    params.d = read_parameter("Neighbours distance ", 20., 100.);
    params.d_s = read_parameter("Critical distance ", 5., 25.);
    params.s = read_parameter("Separation parameter ", 0.5, 5.);
    params.a = read_parameter("Alignment parameter ", 0.05, 1.);
    params.c = read_parameter("Cohesion parameter ", 0.01, 0.05);
    flock_size = read_parameter("Flock size ", 1, 100);
    params.h = 1.;
    params.p = 0.5;
  }
  catch (const std::runtime_error &e)
  {
    std::cerr << "Errore" << '\n';
    return EXIT_FAILURE;
  }

  Flock stormo{};
  std::random_device r;
  std::default_random_engine eng(r());
  std::uniform_real_distribution<float> px{0., 200.};
  std::uniform_real_distribution<float> py{0., 200.};
  std::uniform_real_distribution<float> vel{-50., 50.};

  for (int j{0}; j != flock_size; ++j)
  {
    Coords p{px(eng), py(eng)};
    Coords v{vel(eng), vel(eng)};
    SingleBoid b = SingleBoid(p, v, j);
    stormo.flock.push_back(b);
  }

  SingleBoid hunter{{400., 300.}, {0., 0.}, -1};
  hunter.setColor(sf::Color(255, 100, 100));

  sf::RenderWindow window(sf::VideoMode(800, 600), "Boids");
  window.setFramerateLimit(60);

  sf::RenderWindow window2(sf::VideoMode(500, 600), "Velocity");
  window2.setFramerateLimit(60);

  sf::RenderWindow window3(sf::VideoMode(500, 600), "Distance");
  window3.setFramerateLimit(60);

  sf::Clock clock;

  float margine_x = 50.f;
  float margine_y = 50.f;

  sf::CircleShape point(2.f);
  point.setFillColor(sf::Color::Red);
  sf::CircleShape dot(2.f);
  dot.setFillColor(sf::Color::Green);

  sf::Vector2f origin(margine_x, 600.f - margine_y);

  sf::VertexArray axis(sf::PrimitiveType::Lines, 4);
  sf::Color axis_color = sf::Color::White;

  axis[0].position = origin;
  axis[0].color = axis_color;
  axis[1].position = sf::Vector2f(500.f - 50.f, origin.y);
  axis[1].color = axis_color;

  axis[2].position = origin;
  axis[2].color = axis_color;
  axis[3].position = sf::Vector2f(origin.x, 50.f);
  axis[3].color = axis_color;

  sf::RectangleShape box(sf::Vector2f(200.f, 80.f));
  box.setPosition(300.f, 0.f);
  box.setFillColor(sf::Color::Black);
  sf::Text label;
  sf::Text label2;
  sf::Font font;
  font.loadFromFile("font.ttf");
  label.setFont(font);
  label2.setFont(font);
  label.setCharacterSize(16);
  label2.setCharacterSize(16);
  label.setFillColor(sf::Color::White);
  label2.setFillColor(sf::Color::White);
  label.setPosition(200.f, 10.f);
  label2.setPosition(200.f, 10.f);

  const size_t max_points = 450;
  std::vector<Coords> vector_of_mean_speeds{};
  std::vector<float> vector_of_mean_distances{};
  std::vector<float> vector_of_standard_deviations_v{};
  std::vector<float> vector_of_standard_deviations_d{};

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
      std::vector<Coords> updated_velocities{};

      std::for_each(stormo.flock.begin(), stormo.flock.end(), [&](SingleBoid &b)
                    {Coords new_velocity = create_velocity_with_rules_for_single_boid(params, stormo, b, hunter);
                     updated_velocities.push_back(new_velocity); });

      std::for_each(stormo.flock.begin(), stormo.flock.end(),
                    [&](SingleBoid &b)
                    {b.update_Velocity_with_rules_for_single_boid(updated_velocities[&b - &stormo.flock[0]]);
                     b.update_Position_in_time_for_single_boid(dt); 
                     b.BorderRestriction(800., 600.);
                     b.SpeedRestriction(130);
                     b.update_Shape(); });

      if (timer >= delay)
      {
        Coords new_hunt_velocity = hunt_the_flock(hunter, stormo, params) + hunt_neighbours(hunter, stormo, params);
        hunter.update_Velocity_with_rules_for_single_boid(new_hunt_velocity);
        hunter.update_Position_in_time_for_single_boid(dt);
        hunter.BorderRestriction(800., 600.);
        hunter.SpeedRestriction(100);
        hunter.update_Shape();
      }

      Coords mean_speed_for_cycle = mean_velocity(stormo);
      vector_of_mean_speeds.push_back(mean_speed_for_cycle);
      if (vector_of_mean_speeds.size() > max_points)
      {
        vector_of_mean_speeds.erase(
            vector_of_mean_speeds.begin(),
            vector_of_mean_speeds.begin() +
                (vector_of_mean_speeds.size() -
                 max_points));
      }
      float standard_deviation_velocity = standard_deviation(stormo.flock);
      vector_of_standard_deviations_v.push_back(standard_deviation_velocity);

      float mean_distance_boidz = mean_distance(stormo);
      vector_of_mean_distances.push_back(mean_distance_boidz);
      auto vector_of_distances = get_vector_of_distances(stormo);
      float standard_deviation_distance = standard_deviation(vector_of_distances);
      vector_of_standard_deviations_d.push_back(standard_deviation_distance);
      if (vector_of_mean_distances.size() > max_points)
      {
        vector_of_mean_distances.erase(
            vector_of_mean_distances.begin(),
            vector_of_mean_distances.begin() +
                (vector_of_mean_distances.size() -
                 max_points)); // toglie i punti extra dal grafico
      }
      if (vector_of_standard_deviations_v.size() > max_points)
      {
        vector_of_standard_deviations_v.erase(
            vector_of_standard_deviations_v.begin(),
            vector_of_standard_deviations_v.begin() +
                (vector_of_standard_deviations_v.size() -
                 max_points)); // toglie i punti extra dal grafico
      }
      if (vector_of_standard_deviations_d.size() > max_points){
        vector_of_standard_deviations_d.erase(
            vector_of_standard_deviations_d.begin(),
            vector_of_standard_deviations_d.begin() +
                (vector_of_standard_deviations_d.size() -
                 max_points)); // toglie i punti extra dal grafico
      }

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

      int visible_count = vector_of_mean_speeds.size();
      float graph_left = 50.0;
      float graph_right = 450.0;
      float graph_width = graph_right - graph_left;
      float x_step;
      if (visible_count > 1)
      {
        x_step = graph_width / (visible_count - 1);
      }
      else
      {
        x_step = 0.0;
      }

      for (int i{0}; i < visible_count; ++i)
      {
        float output_speed = std::sqrt(vector_of_mean_speeds[i].x * vector_of_mean_speeds[i].x + 
          vector_of_mean_speeds[i].y * vector_of_mean_speeds[i].y);
        float x = graph_left + x_step * i;
        float y = 550.0 - output_speed;
        if (y < 50.0)
        {
          y = 50.0;
        }
        if (y > 550.0)
        {
          y = 550.0;
        }
        label.setString("Mean speed = " + std::to_string(output_speed) + '\n' +
                        "Error = " + std::to_string(vector_of_standard_deviations_v[i]));
        point.setPosition(x, y);
        window2.draw(point);
      }
      window2.draw(box);
      window2.draw(label);
      window2.display();
    }

    if (window3.isOpen())
    {
      window3.clear();
      window3.draw(axis);

      int visible_count2 = vector_of_mean_distances.size();
      int i{0};
      float graph_left = 50.0;
      float graph_right = 450.0;
      float graph_width = graph_right - graph_left;
      float x_step;
      if (visible_count2 > 1)
      {
        x_step = graph_width / (visible_count2 - 1);
      }
      else
      {
        x_step = 0.0;
      }
      for (auto first = vector_of_mean_distances.begin(),
                last = vector_of_mean_distances.end();
           first != last && i < visible_count2; ++first, ++i)
      {
        float x = graph_left + x_step * i;
        float y = 550.0 - *first;
        if (y < 50.0)
        {
          y = 50.0;
        }
        if (y > 550.0)
        {
          y = 550.0;
        }
        label2.setString("Mean distance =" + std::to_string(*first) + '\n' +
                         "Error = " + std::to_string(vector_of_standard_deviations_d[i]));
        dot.setPosition(x, y);
        window3.draw(dot);
      }
      window3.draw(box);
      window3.draw(label2);
      window3.display();
    }
  }
}
