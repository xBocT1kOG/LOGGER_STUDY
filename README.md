# ESP32-S3 BME280 Logger

Default configuration:
- ESP32-S3 N16R8
- SDA GPIO8
- SCL GPIO9
- BME280 address 0x76
- SSD1306 address 0x3C
- UART 115200

UART commands:
- status
- log on
- log off
- flush

Behavior:
- One sensor/display/UART cycle every 1 second.
- One logical log record every 10 seconds.
- One 4096-byte RAM buffer.
- Flash is written by sector only when the buffer is full, or explicitly by `flush`.
- No wear leveling.
- No flash metadata.
- On boot, the logger scans `logdata` for the first erased sector.
