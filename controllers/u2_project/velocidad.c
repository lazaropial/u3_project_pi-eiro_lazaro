#include "velocidad.h"

float velocidad(float a, float b, float c) {
  a=b*c;
  if (b<0) {
    a=a*-1;
  }
  return (a);
}
