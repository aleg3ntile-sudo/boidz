#ifndef PARAMETERS_HPP
#define PARAMETERS_HPP

#include<string>
#include<iostream>
#include<stdexcept>


struct Parameters {
  float d;   // parametro vicini
  float d_s; // paramtero vicini critici
  float s;   // parametro di separazione
  float a;   // parametro di allineamento
  float c;   // parametro di coesione
  float h;   // parametro di caccia
  float p;   // parametro di preda
};

template <typename T>
T read_parameter(const std::string &name, T min, T max)
{
  T p;

  std::cout << "Insert " << name << "between " << min << " and " << max << '\n';
  std::cin >> p;
  if (p < min || p > max)
  {
    throw std::runtime_error(name + "fuori intervallo");
  }
  return p;
}

#endif