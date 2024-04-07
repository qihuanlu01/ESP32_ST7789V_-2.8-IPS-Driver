ESP32_ST7789V_耀元鸿2.8寸IPS屏Driver

The connection between ESP Board and the LCD is as follows:

```
       ESP Board                       ILI9341 Panel + TOUCH G911
┌──────────────────────┐              ┌────────────────────┐
│             GND      ├─────────────►│ GND                │
│                      │              │                    │
│             3V3      ├─────────────►│ VCC                │
│                      │              │                    │
│             PCLK     ├─────────────►│ SCL                │
│                      │              │                    │
│             MOSI     ├─────────────►│ MOSI               │
│                      │              │                    │
│             MISO     |◄─────────────┤ MISO               │
│                      │              │                    │
│             RST      ├─────────────►│ RES                │
│                      │              │                    │
│             DC       ├─────────────►│ DC                 │
│                      │              │                    │
│       TOUCH SCL      ├─────────────►│ SCL                │
│                      │              │                    │
│       TOUCH SDA      ├─────────────►│ SDA                │
│                      │              │                    │
│       TOUCH INT      ├─────────────►│ INT                │
|                      |              |                    |
│             BK_LIGHT ├─────────────►│ BLK                │
└──────────────────────┘              └────────────────────┘
```

The GPIO number used by this example can be changed in [lvgl_example_main.c](main/spi_lcd_touch_example_main.c).
Especially, please pay attention to the level used to turn on the LCD backlight, some LCD module needs a low level to turn it on, while others take a high level. You can change the backlight level macro `EXAMPLE_LCD_BK_LIGHT_ON_LEVEL` in [lvgl_example_main.c](main/spi_lcd_touch_example_main.c).

