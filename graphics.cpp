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

    void build_label(sf::Text &label, const sf::Font &font){
        label.setFont(font);
        label.setCharacterSize(16);
        label.setFillColor(sf::Color::White);
        label.setPosition(200.f, 10.f);
    }
}
