#include "../bezier.h"
#include "../bezier.h"

#include <cassert>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <vector>

int main() {
    // Koniczyna
    auto fn = bezier::Cap();
    fn = bezier::MovePoint(fn, 0, 0.75, 1.25);
    fn = bezier::MovePoint(fn, 3, -0.75, 1.25);
    fn = bezier::Concatenate(fn,
                             bezier::Rotate(fn, 270),
                             bezier::Rotate(fn, 180),
                             bezier::Rotate(fn, 90));
    fn = bezier::Scale(fn, 1.0, 0.5);
    const bezier::P3CurvePlotter plot1(fn, 4, 1000);
    plot1.Print(std::cout, '*', '.');
}
