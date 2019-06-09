#ifndef COMMON_LIFT_H
#define COMMON_LIFT_H

// C++ requires you to type out the same function body three times to obtain
// SFINAE-friendliness and noexcept-correctness. That's unacceptable.
// For details: https://www.youtube.com/watch?v=I3T4lePH-yA
#define RETURNS(...) \
  noexcept(noexcept(__VA_ARGS__))->decltype(__VA_ARGS__) { return __VA_ARGS__; }

// The name of overload sets can be legally used as part of a function call
// - we can use a macro to create a lambda for us that "lifts" the overload set
// into a function object.
// For details: https://www.fluentcpp.com/2017/08/01/overloaded-functions-stl/
#define LIFT(f) [](auto&&... xs)RETURNS(f(::std::forward<decltype(xs)>(xs)...))

#endif // COMMON_LIFT_H
