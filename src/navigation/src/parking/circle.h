#ifndef CIRCLE_H
#define CIRCLE_H

class Circle {
 public:
  Circle(double x, double y, double radius) :
          x(x), y(y), radius(radius) { }

  double x, y, radius;
};

#endif // CIRCLE_H
