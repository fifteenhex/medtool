#include <endian.h>
#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

#define DEBUG

#define CMD_PREAMBLE	'+'
#define CMD_STATUS	0x10
#define CMD_GET_VDC	0x13
#define CMD_RTC_GET	0x14
#define CMD_RTC_SET	0x15
#define CMD_MEM_RD	0x19
#define CMD_MEM_WR	0x1A
#define CMD_STATUS2	0x40


#define STATUS_MASK	0xff00
#define STATUS_CHECK	0xa500
#define status2_check(_status2)		((_status2 >> 24) & 0xff)
#define status2_protocolid(_status2)	((_status2 >> 16) & 0xff)
#define status2_devid(_status2)		((_status2 >> 8) & 0xff)

#define STATUS2_CHECK	0x5a
#define PROTOCOL_ID	0x05
#define DEVID_MEGAPRO	0x18
#define DEVID_MEGACORE	0x25

#define ADDR_FIFO	0x1810000

struct cntx {
	int port_fd;
};

struct __attribute__((packed)) everdrive_pkt_hdr {
	uint8_t preamble;
	uint8_t _preable;
	uint8_t cmd;
	uint8_t _cmd;
};

#define DEFINEPKT(_cmd) { CMD_PREAMBLE, ~CMD_PREAMBLE, _cmd, ~_cmd };

static const struct everdrive_pkt_hdr pkt_status = DEFINEPKT(CMD_STATUS);
static const struct everdrive_pkt_hdr pkt_vdc = DEFINEPKT(CMD_GET_VDC);
static const struct everdrive_pkt_hdr pkt_rtc_get = DEFINEPKT(CMD_RTC_GET);
static const struct everdrive_pkt_hdr pkt_rtc_set = DEFINEPKT(CMD_RTC_SET);
static const struct everdrive_pkt_hdr pkt_memrd = DEFINEPKT(CMD_MEM_RD);
static const struct everdrive_pkt_hdr pkt_memwr = DEFINEPKT(CMD_MEM_WR);
static const struct everdrive_pkt_hdr pkt_status2 = DEFINEPKT(CMD_STATUS2);

static void hexdump(const uint8_t *data, size_t len)
{
	int i, j;

	for (i = 0; i < len; i += 8) {
		for (j = 0; j < 8; j++) {
			int off = i + j;
			if (off >= len)
				break;

			printf("0x%02x ", (unsigned) data[i + j]);
		}
		printf("\n");
	}
}

static int writen(const struct cntx *cntx, const uint8_t *src, size_t howmuch)
{
	int ret;

#ifdef DEBUG
	printf("data out, %d bytes -->\n", (int) howmuch);
	hexdump(src, howmuch);
#endif

	ret = write(cntx->port_fd, src, howmuch);

	return ret;
}

static int write32(const struct cntx *cntx, uint32_t value)
{
	uint32_t tmp;

	tmp = htobe32(value);
	writen(cntx, (uint8_t*) &tmp, sizeof(tmp));

	return 0;
}

static int write16(const struct cntx *cntx, uint16_t value)
{
	uint16_t tmp;

	tmp = htobe16(value);
	writen(cntx, (uint8_t*) &tmp, sizeof(tmp));

	return 0;
}

static int write8(const struct cntx *cntx, uint8_t value)
{
	writen(cntx, &value, sizeof(value));

	return 0;
}

static int send_cmd(const struct cntx *cntx, const struct everdrive_pkt_hdr *hdr)
{
	int ret;

	ret = writen(cntx, (uint8_t*) hdr, sizeof(*hdr));
	if (ret != sizeof(*hdr)) {
		printf("failed to write packet: %d\n", ret);
		return -1;
	}

	return 0;
}

static int readn(const struct cntx *cntx, uint8_t *dst, size_t howmuch)
{
	int remaining = howmuch;
	uint8_t *cur = dst;
	int ret;

	while (remaining) {
		ret = read(cntx->port_fd, cur, remaining);
		if (ret < 0)
			return ret;

#ifdef DEBUG
		if (ret != remaining)
			printf("read %d bytes from serial port, wanted %d\n",
				ret, (int) howmuch);
#endif

		cur += ret;
		remaining -= ret;
	}
#ifdef DEBUG
	printf("<-- data in, %d bytes\n", ret);
	hexdump(dst, ret);
#endif

	return howmuch;
}

static int read32(const struct cntx *cntx, uint32_t *result)
{
	uint32_t tmp;
	size_t len = sizeof(tmp);
	int ret;

	ret = readn(cntx, (uint8_t *) &tmp, len);
	if (ret != len)
	{
		printf("whelp\n");
		return -1;
	}

	*result = be32toh(tmp);

	return 0;
}


static int read16(const struct cntx *cntx, uint16_t *result)
{
	uint16_t tmp;
	size_t len = sizeof(tmp);
	int ret;

	ret = readn(cntx, (uint8_t *) &tmp, len);
	if (ret != len)
		return -1;

	*result = be16toh(tmp);

	return 0;
}


static int read8(const struct cntx *cntx, uint8_t *result)
{
	uint8_t tmp;
	size_t len = sizeof(tmp);
	int ret;

	ret = readn(cntx, (uint8_t *) &tmp, len);
	if (ret != len)
		return -1;

	*result = tmp;

	return 0;
}

static int get_status2(struct cntx *cntx)
{
	uint32_t status2;
	int ret;

	ret = send_cmd(cntx, &pkt_status2);
	if (ret)
		return ret;

	ret = read32(cntx, &status2);
	if (ret)
		return ret;

	if (status2_check(status2) != STATUS2_CHECK)
		printf("bad status2 reply 0x%08x\n", (unsigned int) status2);

	if (status2_protocolid(status2) != PROTOCOL_ID)
		printf("bad protocol id\n");

	switch(status2_devid(status2)){
		case DEVID_MEGACORE:
			printf("core\n");
			break;
		case DEVID_MEGAPRO:
			printf("pro\n");
			break;
		default:
			printf("unknown device\n");
			break;
	}

	return 0;
}

static int get_status(struct cntx *cntx)
{
	uint16_t status;
	int ret;

	ret = send_cmd(cntx, &pkt_status);
	if (ret)
		return ret;

	ret = read16(cntx, &status);
	if (ret)
		return ret;

	if ((status & STATUS_MASK) != STATUS_CHECK)
		printf("bad status reply 0x%04x\n", (unsigned int) status);

	return 0;
}

static int get_vdc(struct cntx *cntx)
{
	uint16_t vdc;
	int ret;

	ret = send_cmd(cntx, &pkt_vdc);
	if (ret)
		return ret;

	ret = read16(cntx, &vdc);
	if (ret)
		return ret;

	ret = read16(cntx, &vdc);
	if (ret)
		return ret;

	ret = read16(cntx, &vdc);
	if (ret)
		return ret;

	ret = read16(cntx, &vdc);
	if (ret)
		return ret;

	return 0;
}

static int get_rtc(struct cntx *cntx)
{
	uint8_t vdc;
	int ret;

	ret = send_cmd(cntx, &pkt_rtc_get);
	if (ret)
		return ret;

	ret = read8(cntx, &vdc);
	if (ret)
		return ret;

	ret = read8(cntx, &vdc);
	if (ret)
		return ret;

	ret = read8(cntx, &vdc);
	if (ret)
		return ret;

	ret = read8(cntx, &vdc);
	if (ret)
		return ret;

	ret = read8(cntx, &vdc);
	if (ret)
		return ret;

	ret = read8(cntx, &vdc);
	if (ret)
		return ret;

	return 0;
}

static int read_fifo(struct cntx *cntx, size_t howmuch)
{
	uint32_t addr = ADDR_FIFO;
	uint32_t len = 1;
	uint8_t result;

	send_cmd(cntx, &pkt_memrd);
	write32(cntx, addr);
	write32(cntx, len);
	write8(cntx, 0);

	read8(cntx, &result);

	return 0;
}

static int write_fifo(struct cntx *cntx, uint8_t *what, size_t howmuch)
{
	uint32_t addr = ADDR_FIFO;
	uint32_t len = howmuch;
	uint8_t result;
	int i;

	send_cmd(cntx, &pkt_memwr);
	write32(cntx, addr);
	write32(cntx, len);
	write8(cntx, 0);

	for (i = 0; i < len; i++)
		write8(cntx, what[i]);

	return 0;
}

int main(int argc, char **argv)
{
	struct cntx cntx = { 0 };
	const char *port_path;
	struct termios tty;
	int port_fd;

	if (argc == 1) {
		printf("%s <serial port path>\n", argv[0]);
		return 0;
	}

	port_path = argv[1];
	port_fd = open(port_path, O_RDWR);
	if (port_fd < 0) {
		printf("failed to open serial port \'%s\': %d\n", port_path, port_fd);
		return 1;
	}

	tcgetattr(port_fd, &tty);
	cfmakeraw(&tty);
	tcsetattr(port_fd, TCSANOW, &tty);

#ifdef DEBUG
	printf("Opened serial port %s\n", port_path);
#endif

	/* Setup the context we'll pass around */
	cntx.port_fd = port_fd;

	/* Start poking the bear ... */
	get_status2(&cntx);

	get_vdc(&cntx);

	get_rtc(&cntx);

	//read_fifo(&cntx, 1);

	const char test[] = "hello, world";
	write_fifo(&cntx, (uint8_t*) test, sizeof(test));

	return 0;
}
