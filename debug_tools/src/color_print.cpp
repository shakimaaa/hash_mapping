#include "debug_tools/color_print.h"

namespace debug_tools{
ColorPrint::ColorPrint(Color color){
    fmt::v8::color text_color;
    if(color == Color::Blue) text_color = fmt::color::blue_violet; 
    else if(color == Color::Red) text_color = fmt::color::red;
    else if(color == Color::Green) text_color = fmt::color::green;
    else text_color = fmt::color::black;
}
}