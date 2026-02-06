# SSD1306 greyscale

Ever wondered how to display greyscale images on a dirt-cheap SSD1306 ?</br>
This is a test made on a tiny (display area 11x5.5mm) 64 x 32 resolution display.

## How it's done
The SSD1309 has no greyscale capabilities.  This Arduino code however can flip 2 bitmaps at 50Hz with a duty cycle of 25/75%. A pixel on the first bitmap will show at 25% brightness, the same pixel on the second bitmap show at 75% brightness and the same pixel on both bitmaps obviously 100%. Flipping the bitmaps using the Tiny4KOLED library takes almost no CPU time or I2C communication, and since the PWM timing can be completely done by means of interrupt logic, the main loop() is almost 100% obstruction-free. This solution works flawlessly, even when running your MCU at 1MHz clock speed to save energy.

<img src="/res/20260115_184123.jpg">

However it looks like a gimmick, there are real use cases for greyscale as they
- increase readability
- help getting rid of aliasing artifacts
- allow 'percieve' more detail in graphs and diagrams

<a href="https://beyondmicrocontrol.github.io/SSD1306_greyscale/tools/TOOLS_CATALOG.html">TOOLS</a>

<a href="https://beyondmicrocontrol.github.io/AsciiCAD/index.html?d=eNqtlUFrgzAUx+9+inffxah1Z6ujlLopiy2F0g+wy87bbfU8mIWIMhiDXnYZ7DvlkyzJ7Ko2aqpK0LxA8v/9X/Li7M4DSnZjtx/lQdWmQftDSfzf0a4mK/Z9oeRbvMdqcdfgxXJalb3DV+Fu2ByaEpomfVvWufoeaPYGsFkicKLo4fF5bVhbMd49U2JFPk+8XxUynBwVSDGSHmDluiWFGa+A9PM4IZcpNeklJw/Q6gDqef+6sJ8JiqraAULHKiLsLlhkcicnVW6Nm8WuD9hzit05W6edeq+WaEnq5WqcegJwv0ZwO8cBi4wytexc8yLrRy7W2mDsIVO3t61rcS4bIOJcAZ6zCJ1x2RY8gWlA4N94w7nUPHKu679gGXqcS69ykV67k/chfx9wc3zIvU1NdhR0cYJ9FunlWhQV1nzFqZI3X5VyIoMdAkHESiacohpRPNLPJK8pD8tt+Yobie8XKn27kA==">SCHEMA</a>

<a href=https://github.com/BeyondMicroControl/SSD1306_greyscale/tree/main/arduino/Simple_SSD1306_greyscale>ARDUINO CODE</a>


## Feature wish-list

- [ ] Font anti-aliasing, sub-pixel rendering
- [ ] Font brightness

## Status

Actively evolving.  
Focused on correctness, editor compatibility, and expressive schematic text.

<img src="/res/20260116_134027.jpg">
