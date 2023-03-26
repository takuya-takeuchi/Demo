#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

#ifdef _WIN32
#define _USE_MATH_DEFINES
#endif
#include <math.h>

double computeAngle(const double ax,
                    const double ay,
                    const double bx,
                    const double by,
                    const double cx,
                    const double cy,
                    const double dx,
                    const double dy)
{
    // Calculate the dot product
    auto dot_product = (bx - ax) * (dx - cx) + (by - ay) * (dy - cy);

    // Calculate the length of the vectors
    auto length1 = sqrt((bx - ax) * (bx - ax) + (by - ay) * (by - ay));
    auto length2 = sqrt((dx - cx) * (dx - cx) + (dy - cy) * (dy - cy));

    const auto epsilon = std::numeric_limits<double>::epsilon();
    if (length1 < epsilon || length2 < epsilon)
        throw std::invalid_argument("The length of the first vector is zero or too small");

    // Calculate the angle in radians
    auto theta = acos(dot_product / (length1 * length2));

    // Convert the angle to degrees
    auto angle = theta * 180.0 / M_PI;

    return angle;
}

int main ()
{
    // Test case 1
    double angle1 = computeAngle(0, 0, 1, 1, 0, 0, 1, 0);
    assert(abs(angle1 - 45.0) < 1e-6);
    std::cout << "Test case 1: " << angle1 << std::endl;

    // Test case 2
    double angle2 = computeAngle(0, 0, 1, 1, 0, 0, 0, 1);
    assert(abs(angle2 - 90.0) < 1e-6);
    std::cout << "Test case 2: " << angle2 << std::endl;

    // Test case 3
    double angle3 = computeAngle(0, 0, 1, 1, 2, 2, 3, 3);
    assert(abs(angle3 - 0.0) < 1e-6);
    std::cout << "Test case 3: " << angle3 << std::endl;

    std::cout << "OK" << std::endl;
}