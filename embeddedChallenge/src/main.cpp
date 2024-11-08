#include "mbed.h"
#include "LowPassFilter.h"

SPI spi(PF_9, PF_8, PF_7, PC_1, use_gpio_ssel); // mosi, miso, sclk, cs

#define OUT_X_L 0x28
// register fields(bits):
// data_rate(2),Bandwidth(2),Power_down(1),Zen(1),Yen(1),Xen(1)
#define CTRL_REG1 0x20
// configuration: 200Hz ODR,50Hz cutoff, Power on, Z on, Y on, X on
#define CTRL_REG1_CONFIG 0b01'10'1'1'1'1
// register fields(bits): reserved(1), endian-ness(1),Full scale sel(2),
// reserved(1),self-test(2), SPI mode(1)
#define CTRL_REG4 0x23
// configuration: reserved,little endian,500 dps,reserved,disabled,4-wire mode
#define CTRL_REG4_CONFIG 0b0'0'01'0'00'0

#define SPI_FLAG 1

#define R 1.0f

uint8_t write_buf[32];
uint8_t read_buf[32];

const int dataSize = 40; // 40 samples for each axis at 0.5 seconds = 20 seconds recorded data

float gyroX[dataSize];
float gyroY[dataSize];
float gyroZ[dataSize];

float velocity[dataSize];

float totalDist = 0;

volatile uint8_t sampleGyroFlag = 0;

volatile uint8_t offset = -1;

LowPassFilter filter = LowPassFilter(0.9);

EventFlags flags;
// The spi.transfer function requires that the callback
// provided to it takes an int parameter
void spi_cb(int event) { flags.set(SPI_FLAG); };

void setSampleGyroFlag() {
    sampleGyroFlag = 1;
    offset++;
}

void sampleGyro() {
    int16_t raw_gx, raw_gy, raw_gz;
    float gx, gy, gz;
    // reading the status register. bit 4 of the status register
    // is 1 when a new set of samples is ready
    write_buf[0] = 0x27 | 0x80;

    do {
        spi.transfer(write_buf, 2, read_buf, 2, spi_cb, SPI_EVENT_COMPLETE);
        flags.wait_all(SPI_FLAG);

    } while ((read_buf[1] & 0b0000'1000) == 0);

    // prepare the write buffer to trigger a sequential read
    write_buf[0] = OUT_X_L | 0x80 | 0x40;

    // start sequential sample reading
    spi.transfer(write_buf, 7, read_buf, 8, spi_cb, SPI_EVENT_COMPLETE);
    flags.wait_all(SPI_FLAG);

    // read_buf after transfer: garbage byte,
    // gx_low,gx_high,gy_low,gy_high,gz_low,gz_high Put the high and low bytes
    // in the correct order lowB,Highb -> HighB,LowB
    raw_gx = (((uint16_t)read_buf[2]) << 8) | ((uint16_t)read_buf[1]);
    raw_gy = (((uint16_t)read_buf[4]) << 8) | ((uint16_t)read_buf[3]);
    raw_gz = (((uint16_t)read_buf[6]) << 8) | ((uint16_t)read_buf[5]);

    // printf("RAW|\tgx: %d \t gy: %d \t gz: %d\n", raw_gx, raw_gy, raw_gz);

    gx = ((float)raw_gx) *
        (17.5f * 0.017453292519943295769236907684886f / 1000.0f);// 17.5f * pi/180 / 1000
    gy = ((float)raw_gy) *
        (17.5f * 0.017453292519943295769236907684886f / 1000.0f);
    gz = ((float)raw_gz) *
        (17.5f * 0.017453292519943295769236907684886f / 1000.0f);

    gyroX[offset] = gx;
    gyroY[offset] = gy;

    filter.update(gz);

    gyroZ[offset] = filter.getFiltered();

    //printf("Filtered|\t gz: %4.5f\n", gyroZ[offset]);

    velocity[offset] = abs(gyroZ[offset] * 3.14 * R);

    totalDist += velocity[offset] * 0.5;

    //printf("Actual|\tgx: %4.5f \t gy: %4.5f \t gz: %4.5f\n", gx, gy, gz);
    //printf("Linear velocity|\t v: %4.5f\n", velocity[offset]);
}


int main() {
    // Setup the spi for 8 bit data, high steady state clock,
    // second edge capture, with a 1MHz clock rate
    spi.format(8, 3);
    spi.frequency(1'000'000);

    write_buf[0] = CTRL_REG1;
    write_buf[1] = CTRL_REG1_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, spi_cb, SPI_EVENT_COMPLETE);
    flags.wait_all(SPI_FLAG);

    write_buf[0] = CTRL_REG4;
    write_buf[1] = CTRL_REG4_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, spi_cb, SPI_EVENT_COMPLETE);
    flags.wait_all(SPI_FLAG);

    Ticker sampler;

    sampler.attach(&setSampleGyroFlag, 0.5);

    while (1) {
        if (sampleGyroFlag) {
            sampleGyro();

            sampleGyroFlag = 0;
            // Reset offset every 20 seconds
            if (offset == 39) {
                float speed = totalDist / 20.0f;
                printf("Total dist|\t %4.5f\n", totalDist);
                printf("Avg speed|\t %4.5f\n", speed);
                offset = -1;
                totalDist = 0;
            }
        }
    }
}