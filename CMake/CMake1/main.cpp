#include <iostream>
#include <limits>
#include <boost/math/constants/constants.hpp>

int main ()
{
    double pi = boost::math::constants::pi<double>();

    std::cout.precision(std::numeric_limits<double>::max_digits10);
    std::cout << pi << std::endl;
}