#include <endian.h>
#include <errno.h>
#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <poll.h>
#include <string.h>
#include <stdbool.h>
#include <getopt.h>

/* For terminal socket */
#include <sys/socket.h>
#include <sys/un.h>

#define DEBUG

/* Make my C less awful helpers */
#define __must_check __attribute__((warn_unused_result))
#define ARRAY_SIZE(_a) (sizeof(_a)/sizeof(_a[0]))

#define CMD_PREAMBLE	'+'
#define CMD_STATUS	0x10
#define CMD_GET_VDC	0x13
#define CMD_RTC_GET	0x14
#define CMD_RTC_SET	0x15
#define CMD_MEM_RD	0x19
#define CMD_MEM_WR	0x1A
#define CMD_STATUS2	0x40


#define STATUS_MASK			0xff00
#define STATUS_CHECK			0xa500
#define status2_check(_status2)		((_status2 >> 24) & 0xff)
#define status2_protocolid(_status2)	((_status2 >> 16) & 0xff)
#define status2_devid(_status2)		((_status2 >> 8) & 0xff)

#define STATUS2_CHECK	0x5a
#define PROTOCOL_ID	0x05
#define DEVID_MEGAPRO	0x18
#define DEVID_MEGACORE	0x25

#define ADDR_ROM	0x0000000
#define ADDR_FIFO	0x1810000
#define SIZE_FIFO	2048
#define ADDR_MAP	0x1830000

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

static int __must_check writen(const struct cntx *cntx, const uint8_t *src, size_t howmuch)
{
	int ret;

#ifdef DEBUG
	printf("data out, %d bytes -->\n", (int) howmuch);
	hexdump(src, howmuch);
#endif

	ret = write(cntx->port_fd, src, howmuch);

	return ret;
}

static int __must_check write32(const struct cntx *cntx, uint32_t value)
{
	uint32_t tmp;
	int ret;

	tmp = htobe32(value);
	ret = writen(cntx, (uint8_t*) &tmp, sizeof(tmp));
	if (ret != sizeof(tmp))
		return -EIO;

	return 0;
}

static int __must_check write16(const struct cntx *cntx, uint16_t value)
{
	uint16_t tmp;
	int ret;

	tmp = htobe16(value);
	ret = writen(cntx, (uint8_t*) &tmp, sizeof(tmp));
	if (ret != sizeof(tmp))
		return -EIO;

	return 0;
}

static int __must_check write8(const struct cntx *cntx, uint8_t value)
{
	int ret;

	ret = writen(cntx, &value, sizeof(value));
	if (ret != sizeof(value))
		return -EIO;

	return 0;
}

static int __must_check send_cmd(const struct cntx *cntx, const struct everdrive_pkt_hdr *hdr)
{
	int ret;

	ret = writen(cntx, (uint8_t*) hdr, sizeof(*hdr));
	if (ret != sizeof(*hdr)) {
		printf("failed to write packet: %d\n", ret);
		return -EIO;
	}

	return 0;
}

static int canread(const struct cntx *cntx)
{
	int ret;
	struct pollfd pfd = {
		.fd = cntx->port_fd,
		.events = POLLIN,
	};

	ret = poll(&pfd, 1, 0);

	if (ret > 0)
		if (pfd.revents & POLLIN)
			return 1;

	return 0;
}

static int drain(const struct cntx *cntx)
{
	uint8_t junk[1];

	while (canread(cntx))
		read(cntx->port_fd, junk, 1);

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
				ret, (int) remaining);
#endif

		cur += ret;
		remaining -= ret;
	}

#ifdef DEBUG
	printf("<-- data in, %zu bytes\n", howmuch);
	hexdump(dst, howmuch);
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
		return -EIO;

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
		return -EIO;

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
		return -EIO;

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

	if (status2_check(status2) != STATUS2_CHECK) {
		printf("bad status2 reply 0x%08x\n", (unsigned int) status2);
		return -1;
	}

	if (status2_protocolid(status2) != PROTOCOL_ID) {
		printf("bad protocol id\n");
		return -1;
	}

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

/* wip */
struct everdrive_vdc_readings {
	uint16_t v50;
	uint16_t v25;
	uint16_t v12;
	uint16_t vbat;
};

static int do_vdc(struct cntx *cntx)
{
	struct everdrive_vdc_readings vdc;
	int ret;

	ret = send_cmd(cntx, &pkt_vdc);
	if (ret)
		return ret;

	ret = read16(cntx, &vdc.v50);
	if (ret)
		return ret;

	ret = read16(cntx, &vdc.v25);
	if (ret)
		return ret;

	ret = read16(cntx, &vdc.v12);
	if (ret)
		return ret;

	ret = read16(cntx, &vdc.vbat);
	if (ret)
		return ret;

	return 0;
}

/* wip */
struct everdrive_rtc_datetime {
	uint8_t year;
	uint8_t month;
	uint8_t dayofmonth;
	uint8_t hour;
	uint8_t minute;
	uint8_t second;
};

static int do_rtc(struct cntx *cntx)
{
	struct everdrive_rtc_datetime rtc;
	int ret;

	ret = send_cmd(cntx, &pkt_rtc_get);
	if (ret)
		return ret;

	ret = read8(cntx, &rtc.year);
	if (ret)
		return ret;

	ret = read8(cntx, &rtc.month);
	if (ret)
		return ret;

	ret = read8(cntx, &rtc.dayofmonth);
	if (ret)
		return ret;

	ret = read8(cntx, &rtc.hour);
	if (ret)
		return ret;

	ret = read8(cntx, &rtc.minute);
	if (ret)
		return ret;

	ret = read8(cntx, &rtc.second);
	if (ret)
		return ret;

	return 0;
}

static int read_mem(struct cntx *cntx, uint8_t *whereto, uint32_t wherefrom, uint32_t howmuch)
{
	int i, ret;

	ret = send_cmd(cntx, &pkt_memrd);
	if (ret)
		return ret;

	ret = write32(cntx, wherefrom);
	if (ret)
		return ret;

	ret = write32(cntx, howmuch);
	if (ret)
		return ret;

	ret = write8(cntx, 0);
	if (ret)
		return ret;

	for (i = 0; i < howmuch; i++) {
		ret = read8(cntx, whereto++);
		if (ret)
			return ret;
	}

	return 0;
}

static int read_rom(struct cntx *cntx, uint8_t *whereto, size_t howmuch)
{
	return read_mem(cntx, whereto, ADDR_ROM, howmuch);
}

#if 0 /* reading the fifo isn't possible? there is only pc -> md fifo and no md -> pc fifo? */
static int read_fifo(struct cntx *cntx, uint8_t *whereto, size_t howmuch)
{
	return read_mem(cntx, whereto, ADDR_FIFO, howmuch);
}
#endif

static int read_mapper(struct cntx *cntx, uint8_t *whereto, size_t howmuch)
{
	return read_mem(cntx, whereto, ADDR_MAP, howmuch);
}

static int write_fifo(struct cntx *cntx, const uint8_t *what, size_t howmuch)
{
	uint32_t addr = ADDR_FIFO;
	uint32_t len = howmuch;
	int ret;
	int i;

	ret = send_cmd(cntx, &pkt_memwr);
	if (ret)
		return ret;

	ret = write32(cntx, addr);
	if (ret)
		return ret;

	ret = write32(cntx, len);
	if (ret)
		return ret;

	ret = write8(cntx, 0);
	if  (ret)
		return ret;

	for (i = 0; i < len; i++) {
		ret = write8(cntx, what[i]);
		if (ret)
			return ret;
	}

	return 0;
}

static int create_terminal_socket(const char *path) {
	struct sockaddr_un addr = {
		.sun_family = AF_UNIX,
	};
	int listen_fd, conn_fd;
	int ret;

	strcpy(addr.sun_path, path);

	listen_fd = socket(AF_UNIX, SOCK_STREAM, 0);
	if (listen_fd < 0)
		return -1;

	unlink(path);
	ret = bind(listen_fd, (struct sockaddr*)&addr, sizeof(addr));
	if (ret)
		return ret;

	ret = listen(listen_fd, 1);
	if (ret)
		return ret;

	conn_fd = accept(listen_fd, NULL, NULL);

	return conn_fd;
}

static int do_terminal(struct cntx *cntx)
{
	int conn_fd;
	int ret;

	printf("Creating socket and waiting for connection (minicom -D unix#/tmp/medtool)\n");
	conn_fd = create_terminal_socket("/tmp/medtool");
	if (conn_fd < 0)
		return -1;

	printf("connected\n");

	while(true)
	{
		uint8_t ch;
		struct pollfd pfd[] = {
			{
				.fd = cntx->port_fd,
				.events = POLLIN,
			},
			{
				.fd = conn_fd,
				.events = POLLIN,
			},
		};

		ret = poll(pfd, ARRAY_SIZE(pfd), -1);
		if (ret < 0) {
			printf("Failed to poll(): %d\n", errno);
			return -1;
		}

		if (ret > 0) {
			/* Check if poll returned because the serial port was pulled */
			if (pfd[0].revents & (POLLHUP | POLLERR)) {
				fprintf(stderr, "serial port disconnected\n");
				ret = -EIO;
				break;
			}

			/* Got md -> us */
			if (pfd[0].revents & POLLIN) {
				ret = read(cntx->port_fd, &ch, 1);
				if (ret != 1)
					return -EIO;

				write(conn_fd, &ch, 1);
			}
			/* Got us -> md */
			if (pfd[1].revents & POLLIN) {
				ret = read(conn_fd, &ch, 1);
				if (ret != 1)
					return -EIO;

				write_fifo(cntx, &ch, 1);
			}
		}
	}

	return 0;
}

static void usage(const char *progname)
{
	fprintf(stderr, "Usage: %s -p <port> -m <mode>\n", progname);
	fprintf(stderr, "  Modes: terminal, vdc, rtc\n");
}

struct mode_handler {
	const char *modestr;
	int (*handler)(struct cntx *cntx);
};

static const struct mode_handler modes[] = {
	{ .modestr = "terminal", .handler = do_terminal },
	{ .modestr = "vdc", .handler = do_vdc },
	{ .modestr = "rtc", .handler = do_rtc },
};

int main(int argc, char **argv)
{
	struct cntx cntx = { 0 };
	const char *port_path = NULL;
	const char *mode = NULL;
	struct termios tty;
	int port_fd;
	int ret;
	int opt;
	int i;

	while ((opt = getopt(argc, argv, "p:m:")) != -1) {
		switch (opt) {
		case 'p':
			port_path = optarg;
			break;
		case 'm':
			mode = optarg;
			break;
		default:
			usage(argv[0]);
			return 1;
		}
	}

	if (!port_path || !mode) {
		usage(argv[0]);
		return 1;
	}

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

	/* Clean up any remaining garbage */
	drain(&cntx);

	/* Start poking the bear ... */
	ret = get_status2(&cntx);
	if (ret)
		return 1;

	for (i = 0; i < ARRAY_SIZE(modes); i++) {
		const struct mode_handler *handler = &modes[i];

		if (strcmp(mode, handler->modestr) == 0) {
			handler->handler(&cntx);
			break;
		}
	}

	if (i == ARRAY_SIZE(modes))
		fprintf(stderr, "Unknown mode: %s\n", mode);

	//get_vdc(&cntx);

	//get_rtc(&cntx);

	//for (int i = 0; i < 1; i++) {
	//	uint8_t ch;
	//	read_fifo(&cntx, &ch, 1);
	//	printf("\'%c\'\n", (char) ch);
	//}

	//const char test[] = "hello, world";
	//write_fifo(&cntx, (uint8_t*) test, 1);

	//uint8_t buf[1];
	//return read_fifo(&cntx, buf, 2);

	//uint8_t buf[64];
	//read_rom(&cntx, buf, sizeof(buf));
	//hexdump(buf, sizeof(buf));

	//ret = get_status2(&cntx);

	return 0;
}
