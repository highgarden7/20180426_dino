#include <stdint.h>
#include <unistd.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#define ARRAY_SIZE(a)	(sizeof(a) / sizeof((a)[0]))

static const char *device = "/dev/spidev0.0";
static uint8_t	  mode;
static uint8_t	  bits = 8;
static uint32_t   speed = 2000000;
static uint16_t	  delay;

static void write_spi(int fd)
{
	int ret;
	uint8_t tx[] = "Hello World!";
	uint8_t rx[ARRAY_SIZE(tx)] = {0, };

	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
		.rx_buf = 0,
		.len = ARRAY_SIZE(tx) - 1,
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};

	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1)
		pabort("can't send spi message");

	for (ret = 0; ret < ARRAY_SIZE(tx) - 1; ret++) {
		if (!(ret % 6))
			puts("");
		printf("%.2X ", tx[ret]);
	}
	puts("");
}

static void read_spi(int fd)
{
	int ret;
	uint8_t tx[] = "0";
	uint8_t rx[ARRAY_SIZE(tx)] = {0, };
	struct spi_ioc_transfer tr = {
		.tx_buf = 0,
		.rx_buf = (unsigned long)rx,
		.len = ARRAY_SIZE(tx) - 1,
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word =bits,
	};

	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1)
		pabort("can't receive spi message");

	for (ret = 0; ret < ARRAY_SIZE(tx) - 1; ret++) {
		if(!(ret % 6))
			puts("");
		printf("%2X ", rx[ret]);
	}
	puts("");
}

static void 







































