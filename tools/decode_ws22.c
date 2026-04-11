/*
 * decode_ws22.c - WaveSculptor22 Motor Sense packet decoder
 *
 * Reads from a serial port (e.g. ST-Link VCP) at 470588 baud and decodes
 * the 4-byte ProHelion Motor Sense packets per the WaveSculptor200 comms spec.
 *
 * Build:  gcc -O2 -lm -o decode_ws22 decode_ws22.c
 * Run:    ./decode_ws22 /dev/ttyACM0
 *
 * --------------------------------------------------------------------------
 * Thermistor configuration (override at compile time with -D flags):
 *
 *   THERM_R_SERIES   Ohms  Series resistor in the voltage divider (to Vcc)
 *   THERM_R0         Ohms  Thermistor resistance at reference temperature
 *   THERM_T0_C       °C    Reference temperature for R0 (usually 25)
 *   THERM_BETA             Beta coefficient of the thermistor
 *   THERM_PULLUP           1 = thermistor is between ADC pin and GND (series R to Vcc)
 *                          0 = thermistor is between ADC pin and Vcc (series R to GND)
 *   ADC_BITS               Resolution of the ADC (12 for STM32F1)
 *
 * Example for a 10k NTC (beta=3977) with a 10k pullup:
 *   gcc -O2 -lm -DTHERM_BETA=3977 -DTHERM_R_SERIES=10000 -o decode_ws22 decode_ws22.c
 * --------------------------------------------------------------------------
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <sys/ioctl.h>
#include <asm/termbits.h>  /* struct termios2, BOTHER, TCGETS2, TCSETS2 */

/* ~470.6 kbaud: STM32 APB1=32MHz, divider=68 => 32e6/68 = 470588 */
#define BAUD_RATE 470588

/* Thermistor defaults — override with -D at compile time */
#ifndef THERM_R_SERIES
#define THERM_R_SERIES  10000.0f   /* 10 kΩ series resistor */
#endif
#ifndef THERM_R0
#define THERM_R0        10000.0f   /* 10 kΩ @ 25 °C */
#endif
#ifndef THERM_T0_C
#define THERM_T0_C      25.0f      /* reference temperature (°C) */
#endif
#ifndef THERM_BETA
#define THERM_BETA      3977.0f    /* Beta coefficient */
#endif
#ifndef THERM_PULLUP
#define THERM_PULLUP    1          /* 1 = thermistor to GND, series R to Vcc */
#endif
#ifndef ADC_BITS
#define ADC_BITS        12
#endif

#define ADC_MAX         ((1 << ADC_BITS) - 1)
#define KELVIN_OFFSET   273.15f

/*
 * Convert raw 12-bit ADC reading to °C using the Beta equation:
 *   1/T = 1/T0 + (1/B) * ln(R / R0)
 *
 * Voltage divider (THERM_PULLUP=1):
 *   Vcc --- R_series --- ADC --- Thermistor --- GND
 *   R_therm = R_series * adc / (ADC_MAX - adc)
 *
 * Voltage divider (THERM_PULLUP=0):
 *   Vcc --- Thermistor --- ADC --- R_series --- GND
 *   R_therm = R_series * (ADC_MAX - adc) / adc
 */
static float thermistor_to_celsius(uint16_t adc_raw)
{
    if (adc_raw == 0 || adc_raw == ADC_MAX) return -999.0f; /* rail: undefined */

    float adc = (float)adc_raw;
#if THERM_PULLUP
    float r_therm = THERM_R_SERIES * adc / ((float)ADC_MAX - adc);
#else
    float r_therm = THERM_R_SERIES * ((float)ADC_MAX - adc) / adc;
#endif

    float t0_k  = THERM_T0_C + KELVIN_OFFSET;
    float t_k   = 1.0f / (1.0f / t0_k + logf(r_therm / THERM_R0) / THERM_BETA);
    return t_k - KELVIN_OFFSET;
}

static const char *device_type_str(uint8_t t)
{
    switch (t) {
        case 0: return "INVALID";
        case 1: return "120deg Halls, NTC thermistor (TRI74-46)";
        case 2: return "Quadrature Encoder, NTC thermistor (TRI74-44)";
        case 3: return "Quadrature Encoder + Resolver 14-bit, Pt100 (TRI74-45)";
        case 4: return "Quadrature Encoder + Halls Timken M15 3-bit, Pt100 (TRI74-50)";
        case 5: return "Quadrature Encoder + Absolute position 12-bit, Pt100 (TRI74-52)";
        case 6: return "Quadrature Encoder, NTC thermistor (TRI74-46)";
        case 7: return "Quadrature Encoder + Resolver 14-bit, NTC thermistor (TRI74-45)";
        case 8: return "Quadrature Encoder, Pt100 (TRI74-46)";
        default: return "Unknown";
    }
}

static int open_serial(const char *port)
{
    int fd = open(port, O_RDONLY | O_NOCTTY);
    if (fd < 0) {
        perror("open");
        return -1;
    }

    struct termios2 tio;
    if (ioctl(fd, TCGETS2, &tio) < 0) {
        perror("TCGETS2");
        close(fd);
        return -1;
    }

    /* 8N1 raw mode */
    tio.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tio.c_oflag &= ~OPOST;
    tio.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tio.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CBAUD);
    tio.c_cflag |= CS8 | CLOCAL | CREAD | BOTHER;
    tio.c_ispeed = BAUD_RATE;
    tio.c_ospeed = BAUD_RATE;
    tio.c_cc[VMIN]  = 1;
    tio.c_cc[VTIME] = 0;

    if (ioctl(fd, TCSETS2, &tio) < 0) {
        perror("TCSETS2");
        close(fd);
        return -1;
    }

    return fd;
}

/*
 * Packet layout (from spec + firmware):
 *
 * Byte 0:  bit7=1 (start), bit6=0 device-type / bit6=1 temperature, bits5-0 = dev_type or temp[11:6]
 * Byte 1:  bit7=0, bits6-0 = reserved (device type) or temp[5:0] left-shifted 1 => temp[5:0] in bits[6:1]
 * Byte 2:  bit7=0, bits6-0 = angle_14b[13:7]
 * Byte 3:  bit7=0, bits6-0 = angle_14b[6:0]
 *
 * angle_raw (16-bit from sensor) = angle_14b << 2
 * thermistor_raw (12-bit ADC)    = temp_13b >> 1
 *   where temp_13b = (byte0[5:0] << 7) | byte1[6:0]
 */
static void decode_packet(const uint8_t *pkt, unsigned long *count)
{
    int is_temp = (pkt[0] >> 6) & 1;

    /* position/velocity is the same field in both packet types */
    uint16_t angle_14b = (uint16_t)(((pkt[2] & 0x7F) << 7) | (pkt[3] & 0x7F));
    uint16_t angle_raw = (uint16_t)(angle_14b << 2);
    float    angle_deg = angle_14b / 16384.0f * 360.0f;

    printf("#%-6lu  ", (*count)++);

    if (!is_temp) {
        uint8_t dev = pkt[0] & 0x3F;
        printf("[DEVICE TYPE] dev=%u (%s)\n", dev, device_type_str(dev));
        printf("              angle_14b=0x%04X  angle_raw=0x%04X  %.2f deg\n",
               angle_14b, angle_raw, angle_deg);
    } else {
        uint16_t temp_13b   = (uint16_t)(((pkt[0] & 0x3F) << 7) | (pkt[1] & 0x7F));
        uint16_t adc_raw    = temp_13b >> 1;
        float    temp_c     = thermistor_to_celsius(adc_raw);
        printf("[DATA]        temp_raw=%-5u  temp=%.1f°C  "
               "angle_14b=0x%04X  angle_raw=0x%04X  %.2f deg\n",
               adc_raw, temp_c, angle_14b, angle_raw, angle_deg);
    }
    fflush(stdout);
}

int main(int argc, char *argv[])
{
    const char *port = (argc > 1) ? argv[1] : "/dev/ttyACM0";

    fprintf(stderr, "Opening %s at %d baud (8N1)\n", port, BAUD_RATE);
    fprintf(stderr, "Thermistor: R_series=%.0f Ω  R0=%.0f Ω @ %.0f°C  Beta=%.0f  %s\n",
            (double)THERM_R_SERIES, (double)THERM_R0, (double)THERM_T0_C, (double)THERM_BETA,
            THERM_PULLUP ? "pullup (therm to GND)" : "pulldown (therm to Vcc)");
    int fd = open_serial(port);
    if (fd < 0) return 1;
    fprintf(stderr, "Reading... (Ctrl+C to stop)\n\n");

    uint8_t pkt[4];
    int     fill  = 0;
    unsigned long count = 0;

    while (1) {
        uint8_t b;
        ssize_t n = read(fd, &b, 1);
        if (n < 0) {
            if (errno == EINTR) continue;
            perror("read");
            break;
        }
        if (n == 0) continue;

        if (b & 0x80) {
            /* bit7 set => start of packet, begin collecting */
            pkt[0] = b;
            fill = 1;
        } else if (fill >= 1) {
            pkt[fill++] = b;
            if (fill == 4) {
                decode_packet(pkt, &count);
                fill = 0;
            }
        }
        /* non-start byte before we've seen a start: discard */
    }

    close(fd);
    return 0;
}
