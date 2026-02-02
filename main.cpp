// C/C++ includes
#include <stdio.h>
#include <string>
#include <vector>

// Pico SDK includes
#include "hardware/i2c.h"
#include "pico/binary_info.h"
#include "pico/time.h"

// Pimoroni includes
#include "pico_explorer.hpp"
#include "drivers/st7789/st7789.hpp"
#include "libraries/pico_graphics/pico_graphics.hpp"

using namespace pimoroni;

// splash screen TGA image data
extern unsigned char _binary_chimp_tga_start[];

ST7789 st7789(PicoExplorer::WIDTH, PicoExplorer::HEIGHT, ROTATE_0, false,
    get_spi_pins(BG_SPI_FRONT));
PicoGraphics_PenRGB332 graphics(st7789.width, st7789.height, nullptr);

// By default these devices are on bus address 0x68
static int sensor_addr = 0x68;

// I2C reserves some addresses for special purposes. We exclude these from the scan.
// These are any addresses of the form 000 0xxx or 111 1xxx
bool reserved_addr(uint8_t addr) {
    return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}

static void i2c_scan() {
    printf("\nI2C Bus Scan\n");
    printf("   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");

    for (int addr = 0; addr < (1 << 7); ++addr) {
        if (addr % 16 == 0) {
            printf("%02x ", addr);
        }

        // Perform a 1-byte dummy read from the probe address. If a slave
        // acknowledges this address, the function returns the number of bytes
        // transferred. If the address byte is ignored, the function returns
        // -1.

        // Skip over any reserved addresses.
        int ret;
        uint8_t rxdata;
        if (reserved_addr(addr))
            ret = PICO_ERROR_GENERIC;
        else
            ret = i2c_read_blocking(i2c_default, addr, &rxdata, 1, false);

        printf(ret < 0 ? "." : "@");
        printf(addr % 16 == 15 ? "\n" : "  ");
    }
    printf("Done.\n");
}

static void mpu9250_reset() {
    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t buf[] = {0x6B, 0x80};
    i2c_write_blocking(i2c_default, sensor_addr, buf, 2, false);
    sleep_ms(100); // Allow device to reset and stabilize

    // Clear sleep mode (0x6B register, 0x00 value)
    buf[1] = 0x00;  // Clear sleep mode by writing 0x00 to the 0x6B register
    i2c_write_blocking(i2c_default, sensor_addr, buf, 2, false); 
    sleep_ms(10); // Allow stabilization after waking up
}

static void mpu9250_read_raw(int16_t accel[3], int16_t gyro[3], int16_t mag[3], int16_t *temp) {
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.

    uint8_t buffer[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    uint8_t val = 0x3B;
    i2c_write_blocking(i2c_default, sensor_addr, &val, 1, true); // true to keep master control of bus
    i2c_read_blocking(i2c_default, sensor_addr, buffer, 6, false);
    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Now gyro data from reg 0x43 for 6 bytes
    // The register is auto incrementing on each read
    val = 0x43;
    i2c_write_blocking(i2c_default, sensor_addr, &val, 1, true);
    i2c_read_blocking(i2c_default, sensor_addr, buffer, 6, false);  // False - finished with bus
    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
    }

#if 0
    // Now magnetometer data from reg 0x49 for 6 bytes
    // The register is auto incrementing on each read
    val = 0x49;
    i2c_write_blocking(i2c_default, sensor_addr, &val, 1, true);
    i2c_read_blocking(i2c_default, sensor_addr, buffer, 6, false);  // False - finished with bus
    for (int i = 0; i < 3; i++) {
        mag[i] = (buffer[i * 2 + 1] << 8 | buffer[i * 2]); // Note little endian for mag
    }
#endif

    // Now temperature from reg 0x41 for 2 bytes
    // The register is auto incrementing on each read
    val = 0x41;
    i2c_write_blocking(i2c_default, sensor_addr, &val, 1, true);
    i2c_read_blocking(i2c_default, sensor_addr, buffer, 2, false);  // False - finished with bus

    *temp = buffer[0] << 8 | buffer[1];
}

void draw(PicoGraphics_PenRGB332 &graphics, const std::vector<std::string> &lines) {
    // drawing code goes here
    Pen pen_bg = graphics.create_pen(0, 0, 0);
    Pen pen_white = graphics.create_pen(255, 255, 255);

    graphics.set_pen(pen_bg);
    graphics.clear();

    graphics.set_pen(pen_white);
    graphics.set_font(&font8);

    int counter=0;
    for(std::string_view line: lines) {
        graphics.text(line, Point(10, 10 + counter*24), 220);
        counter++;
    }
}

int main() {
    // Show splash screen
    for(int y = 0; y < PicoExplorer::HEIGHT; y++) {
        uint8_t *src = _binary_chimp_tga_start + 18 + (y * PicoExplorer::WIDTH * 3);
        for(int x = 0; x < PicoExplorer::WIDTH; x++) {
            uint8_t b = *src++;
            uint8_t g = *src++;
            uint8_t r = *src++;

            graphics.set_pen(graphics.create_pen(r, g, b));
            graphics.pixel(Point(x, y));
        }
    }
    st7789.update(&graphics);

    // initialize stdio
    stdio_init_all();
    sleep_ms(3000); // wait for usb serial to connect

    // initialize i2c
    i2c_init(i2c_default, 400 * 1000); // 400 kHz
    gpio_set_function(20, GPIO_FUNC_I2C);
    gpio_set_function(21, GPIO_FUNC_I2C);
    bi_decl(bi_2pins_with_func(20, 21, GPIO_FUNC_I2C));

    // initialize the mpu9250
    mpu9250_reset();

    // retrieve and display data
    int16_t accel[3] = { 0, 0, 0 },
            gyro[3]  = { 0, 0, 0 },
            mag[3]   = { 0, 0, 0 },
            temp     = 0;
    while(true) {
        // determine time at which next iteration will occur
        absolute_time_t wake_time = make_timeout_time_ms(100); // 10 Hz update rate

        // read raw data from the MPU9250
        mpu9250_read_raw(accel, gyro, mag, &temp);

        // update the screen with the data
        std::vector<std::string> lines(10);
        char buffer[30];
        snprintf(buffer, sizeof(buffer), "acc_x:  %d", accel[0]);
        lines[0] = buffer;
        snprintf(buffer, sizeof(buffer), "acc_y:  %d", accel[1]);
        lines[1] = buffer;
        snprintf(buffer, sizeof(buffer), "acc_z:  %d", accel[2]);
        lines[2] = buffer;
        snprintf(buffer, sizeof(buffer), "gyro_x: %d", gyro[0]);
        lines[3] = buffer;
        snprintf(buffer, sizeof(buffer), "gyro_y: %d", gyro[1]);
        lines[4] = buffer;
        snprintf(buffer, sizeof(buffer), "gyro_z: %d", gyro[2]);
        lines[5] = buffer;
        snprintf(buffer, sizeof(buffer), "mag_x:  %d", mag[0]);
        lines[6] = buffer;
        snprintf(buffer, sizeof(buffer), "mag_y:  %d", mag[1]);
        lines[7] = buffer;
        snprintf(buffer, sizeof(buffer), "mag_z:  %d", mag[2]);
        lines[8] = buffer;
        snprintf(buffer, sizeof(buffer), "temp:   %0.1f C", (float)temp/321.0 + 21.0);
        lines[9] = buffer;

        draw(graphics, lines);
        st7789.update(&graphics);

        // sleep until time for next loop iteration
        sleep_until(wake_time);
    }
}