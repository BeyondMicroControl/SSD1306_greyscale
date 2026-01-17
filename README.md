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

<a href="https://beyondmicrocontrol.github.io/SSD1306_greyscale/index.html">TOOLS</a>

<a href="https://beyondmicrocontrol.github.io/AsciiCAD/index.html?d=eNqtlU9rgzAYxu9+ive+i//qztaMUuamLK4USj9ALztvt9bzDhYilcEYlMEuhX2nfJIlqV3VRk1VCZo3kDy/503eOHlEQMlm6ParPKjaNGh+KIn/O9rNaMa+a0oO4j1Ui9sGr5bTyuwtvnJ3/ebQlNA06dp2ratvgR6+ARZuFK1e3uamvYTjcPtEiRP5PPF+V0hwclIg+Ui6h5nnFRQmvADSr9OETKZUp5ecPUCjA6im/efK/k5QlNX2ELp2HmHvnkUWd3JW5da4Wez5gJGbb87FOs3UW7VES1IvV+PUI4CnuQEPUxywyCxSy441r7Fu5GKtBcbIsHRn2bgW53IAIs4V4CmLjAsux4ZXsEwI/DvUn0vNI+e6PQbPIeJcepmLdNqdrAv5R4+L41PubWyxo6CLE+yzSC/Woqiw+htOlbz+ppQTmewQCCJWMuHYqBDFA/1Lsopyv9wWr7iB+P4AnMC7VA==#">SCHEMA</a>

<a href=https://github.com/BeyondMicroControl/SSD1306_greyscale/tree/main/arduino/Simple_SSD1306_greyscale>ARDUINO CODE</a>


## Feature wish-list

- [ ] Font anti-aliasing, sub-pixel rendering
- [ ] Font brightness

## Status

Actively evolving.  
Focused on correctness, editor compatibility, and expressive schematic text.

<img src="/res/20260116_134027.jpg">
