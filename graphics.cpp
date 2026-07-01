#include "graphics.hpp"

namespace boidz
{
    void create_axis(sf::VertexArray &axis)
    {
        float margine_x = 50.f;
        float margine_y = 50.f;

        sf::Vector2f origin(margine_x, 600.f - margine_y);

        sf::Color axis_color = sf::Color::White;

        axis[0].position = origin;
        axis[0].color = axis_color;
        axis[1].position = sf::Vector2f(500.f - 50.f, origin.y);
        axis[1].color = axis_color;
        axis[2].position = origin;
        axis[2].color = axis_color;
        axis[3].position = sf::Vector2f(origin.x, 50.f);
        axis[3].color = axis_color;
    }

    void build_label(sf::Text &label, const sf::Font &font)
    {
        label.setFont(font);
        label.setCharacterSize(16);
        label.setFillColor(sf::Color::White);
        label.setPosition(200.f, 10.f);
    }

    void draw_points(std::vector<float> &v, std::vector<float> &w,
                     sf::Text &label, sf::CircleShape &point, sf::RenderWindow &window)
    {
        auto visible_count = v.size();
        float graph_left = 50.0;
        float graph_right = 450.0;
        float graph_width = graph_right - graph_left;
        float x_step;
        if (visible_count > 1)
        {
            x_step = graph_width / static_cast<float>((visible_count - 1));
        }
        else
        {
            x_step = 0.0;
        }

        for (long unsigned int i{0}; i < visible_count; ++i)
        {
            float output_speed = v[i];
            float x = graph_left + (x_step * static_cast<float>(i));
            float y = 550.0f - output_speed;
            if (y < 50.0f)
            {
                y = 50.0f;
            }
            if (y > 550.0f)
            {
                y = 550.0f;
            }
            label.setString(
                "Mean speed = " + std::to_string(output_speed) + '\n' +
                "Error = " + std::to_string(w[i]));
            point.setPosition(x, y);
            window.draw(point);
        }
    }
}
