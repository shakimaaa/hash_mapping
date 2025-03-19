#ifndef COLOR_PRINT_H
#define COLOR_PRINT_H

#include <fmt/color.h>

namespace debug_tools{
class ColorPrint{
public:
    enum Color{Red, Blue, Green};

    ColorPrint(Color color);
};
}

#endif