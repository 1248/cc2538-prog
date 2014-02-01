#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <stdbool.h>
#include <getopt.h>
#include <sys/time.h>
#include <termios.h>
#include <sys/select.h>
#include "hex.h"

#define SERIAL_TIMEOUT 3
#define FLASH_SIZE_KB (512 * 1024)
#define FLASH_PAGE_SIZE 2048
#define FLASH_OFFSET 0x00200000

#define COMMAND_RET_SUCCESS 0x40
#define COMMAND_PING 0x20
#define COMMAND_ERASE 0x26
#define COMMAND_DOWNLOAD 0x21
#define COMMAND_SEND_DATA 0x24
#define COMMAND_RESET 0x25

struct context
{
    int fd;
};

static bool opt_console = false;
static bool opt_flash = false;
static int opt_timeout = 10;
static bool opt_device = false;
static char *flash_filename = NULL;
static char *device_name = NULL;
static struct termios orig_termios;
static uint32_t addr_off = 0x00000000;


static struct option long_options[] =
{
    {"help",    no_argument, 0, 'h'},
    {"console",   no_argument, 0, 'c'},
    {"flash",     required_argument, 0, 'f'},
    {"device",     required_argument, 0, 'd'},
    {"timeout",     required_argument, 0, 't'},
    {0, 0, 0, 0}
};

void usage(void)
{
    fprintf(stderr, "CC2538 serial loader (swru333a)\n");
    fprintf(stderr, "cc2538-prog -d /dev/ttyXYZ [-c] [-f file.hex]\n");
    fprintf(stderr, "  --help           -h          This help\n");
    fprintf(stderr, "  --console        -c          Connect console to serial port on device\n");
    fprintf(stderr, "  --flash=file.hex -f file.hex Reflash device with intel hex file\n");
    fprintf(stderr, "  --timeout=n      -t n        Search for bootload string for n seconds\n");
}

int parse_options(int argc, char **argv)
{
    int c;
    int option_index;

    while(1)
    {
        c = getopt_long (argc, argv, "hcf:d:t:", long_options, &option_index);
        if (c == -1)
            break;
        switch(c)
        {
            case 'h':
                return 1;
            break;
            case 't':
                opt_timeout = atoi(optarg);
            break;
            case 'c':
                opt_console = true;
            break;
            case 'f':
                opt_flash = true;
                flash_filename = strdup(optarg);
            break;
            case 'd':
                opt_device = true;
                device_name = strdup(optarg);
            break;
            default:
                return 1;
            break;
        }
    }

    if (!opt_device)
        return 1;

    if (!opt_flash && !opt_console)
        return 1;

    return 0;
}

static void do_exit(void)
{
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &orig_termios);
    puts("exiting\n");
}

static int enableRawMode(void)
{
    struct termios raw;

    if (!isatty(STDIN_FILENO)) 
        return -1;

    if (tcgetattr(STDIN_FILENO,&orig_termios) == -1)
        return -1;

    raw = orig_termios;  /* modify the original mode */
    raw.c_iflag &= ~(BRKINT | /*ICRNL |*/ INPCK | ISTRIP | IXON);
    raw.c_oflag &= ~(OPOST);
    raw.c_cflag |= (CS8);
    raw.c_lflag &= ~(ECHO | ICANON | IEXTEN | ISIG);
    raw.c_cc[VMIN] = 1;
    raw.c_cc[VTIME] = 0;

    if (tcsetattr(STDIN_FILENO,TCSAFLUSH,&raw) < 0)
        return -1;
    return 0;
}

int serialOpen(char *port)
{
	int fd;
	struct termios t_opt;

	if ((fd = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK)) < 0)
    {
		fprintf(stderr, "Could not open serial port %s\n", port);
		return -1;
	}

	fcntl(fd, F_SETFL, 0);
	tcgetattr(fd, &t_opt);
	cfsetispeed(&t_opt, B115200);
	cfsetospeed(&t_opt, B115200);
	t_opt.c_cflag |= (CLOCAL | CREAD);
    t_opt.c_cflag &= ~PARENB;
	t_opt.c_cflag &= ~CSTOPB;
	t_opt.c_cflag &= ~CSIZE;
	t_opt.c_cflag |= CS8;
	t_opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	t_opt.c_iflag &= ~(IXON | IXOFF | IXANY);
	t_opt.c_oflag &= ~OPOST;
	t_opt.c_cc[VMIN] = 0;
	t_opt.c_cc[VTIME] = 10;
	tcflush(fd, TCIFLUSH);
	tcsetattr(fd, TCSANOW, &t_opt);

	return fd;
}

int serialReadByte(int fd, uint8_t *c)
{
    struct timeval start, now;
    int rc;

    gettimeofday(&start, NULL);

    do
    {
        if ((rc = read(fd, c, 1)) < 0)
            return rc;
        if (rc == 1)
            return 0;
        gettimeofday(&now, NULL);
    }
    while((now.tv_sec - start.tv_sec) < SERIAL_TIMEOUT);
    return -1;
}

int serialRead(int fd, uint8_t *buf, int len)
{
    int orig_len = len;
    while(len--)
    {
        if (serialReadByte(fd, buf++) < 0)
        {
            printf("serialread byte failed\n");
            return -1;
        }
    }
    return orig_len;
}

int serialWrite(int fd, void* buf, int len)
{
    struct timeval start, now;
    int rc;

    gettimeofday(&start, NULL);

    do
    {
        if ((rc = write(fd, buf, len)) < 0)
            return rc;
        if (rc > 0)
            return rc;
        gettimeofday(&now, NULL);
    }
    while((now.tv_sec - start.tv_sec) < SERIAL_TIMEOUT);

    return 1;
}

#define serialClose close

int run_command(int fd, uint8_t *data, uint8_t len)
{
    uint8_t inbuf[2];
    uint8_t sum = 0;
    uint8_t *sump = data;
    uint8_t i = len;
    uint8_t packet_len = len+2;

    while(i--)
        sum += *sump++;

    if (1 != serialWrite(fd, &packet_len, 1))
        return 1;
    if (1 != serialWrite(fd, &sum, 1))
        return 1;
    if (len != serialWrite(fd, data, len))
        return 1;
    if (2 != serialRead(fd, inbuf, 2))
        return 1;
    if (0x00 != inbuf[0] || 0xCC != inbuf[1])
    {
        fprintf(stderr, "NAK\n");
        return 1;
    }
    return 0;
}

int erase_page(int fd, size_t page)
{
    uint32_t address = FLASH_OFFSET + (page * FLASH_PAGE_SIZE);
    uint8_t buf[9];

    buf[0] = COMMAND_ERASE;
    buf[1] = (address & 0xFF000000) >> 24;
    buf[2] = (address & 0x00FF0000) >> 16;
    buf[3] = (address & 0x0000FF00) >> 8;
    buf[4] = (address & 0x000000FF);
    buf[5] = (FLASH_PAGE_SIZE & 0xFF000000) >> 24;
    buf[6] = (FLASH_PAGE_SIZE & 0x00FF0000) >> 16;
    buf[7] = (FLASH_PAGE_SIZE & 0x0000FF00) >> 8;
    buf[8] = (FLASH_PAGE_SIZE & 0x000000FF);

    if (0 != run_command(fd, buf, 9))
    {
        printf("erase failed address=0x%08X len=0x%08x\n", address, FLASH_PAGE_SIZE);
        return 1;
    }
    return 0;
}

int download_addr_len(int fd, uint32_t addr, size_t len)
{
    uint8_t buf[9];

    buf[0] = COMMAND_DOWNLOAD;
    buf[1] = (addr & 0xFF000000) >> 24;
    buf[2] = (addr & 0x00FF0000) >> 16;
    buf[3] = (addr & 0x0000FF00) >> 8;
    buf[4] = (addr & 0x000000FF);
    buf[5] = (len & 0xFF000000) >> 24;
    buf[6] = (len & 0x00FF0000) >> 16;
    buf[7] = (len & 0x0000FF00) >> 8;
    buf[8] = (len & 0x000000FF);

    if (0 != run_command(fd, buf, 9))
    {
        printf("download_addr_len failed\n");
        return 1;
    }
    return 0;
}

int send_data(int fd, uint8_t *data, uint8_t len)
{
    uint8_t buf[256];

    buf[0] = COMMAND_SEND_DATA;
    memcpy(buf+1, data, len);
    if (0 != run_command(fd, buf, len+1))
    {
        printf("send_data failed\n");
        return 1;
    }
    return 0;
}

int reset(int fd)
{
    uint8_t buf[1];

    buf[0] = COMMAND_RESET;
    return run_command(fd, buf, 1);
}


void dump(uint8_t *p, size_t len)
{
    while(len--)
        printf("%02X", *p++);
    printf("\n");
}

int check_for_bootloader(int fd, int timeout)
{

    uint8_t cmdbuf[2];
    uint8_t inbuf[2];

    cmdbuf[0] = 0x55;   // autobauding
    cmdbuf[1] = 0x55;

    if (2 != serialWrite(fd, cmdbuf, 2))
        return 1;
    if (2 != serialRead(fd, inbuf, 2))
        return 1;
    if (inbuf[0] != 0x00 || inbuf[1] != 0xCC)
        return 1;
    cmdbuf[0] = COMMAND_PING;
    if (0 != run_command(fd, cmdbuf, 1))
    {
        printf("ping failed\n");
        return 1;
    }
    printf("ping ok\n");
    return 0;
}

void do_console(int fd)
{
    fd_set rfds;
    struct timeval tv;
    int rc;

    if (0 != enableRawMode())
    {
        fprintf(stderr, "Not a TTY?\n");
        return;
    }

    while(1)
    {
        uint8_t c;
        tv.tv_sec = 1;
        tv.tv_usec = 0;
        int maxfd = 1;

        FD_ZERO(&rfds);
        FD_SET(0, &rfds);
        FD_SET(fd, &rfds);
        maxfd = fd+1;

        rc = select(maxfd, &rfds, NULL, NULL, &tv);

        if (rc == -1)
        {
            fprintf(stderr, "select failed\n");
            return;
        }
        else
        if (rc)
        {
            if (FD_ISSET(0, &rfds))
            {
                if(read(0, &c, 1) > 0)
                {
                    if (c == 0x03 || c == 0x04) // ctrl-d, ctrl-c
                    {
                        return;
                    }
                    if (1 != serialWrite(fd, &c, 1))
                    {
                        fprintf(stderr, "write error\n");
                        return;
                    }
                }
            }
            else
            if (FD_ISSET(fd, &rfds))
            {
                if(serialRead(fd, &c, 1) > 0)
                {
                    if (1 != write(1, &c, 1))
                    {
                        fprintf(stderr, "write error\n");
                        return;
                    }
                }
            }
        }
        else
        {
            // timeout
        }
    }
}

int handle_record_00(uint32_t addr, uint8_t *data, uint8_t len, void *vctx)
{
    struct context *ctx = (struct context *)vctx;
    printf("Writing %d bytes to 0x%08X\n", len, addr + addr_off);
    if (0 != download_addr_len(ctx->fd, addr + addr_off, len))
    {
        printf("download_addr_len failed\n");
        return 1;
    }
    if (0 != send_data(ctx->fd, data, len))
    {
        printf("send_data failed\n");
        return 1;
    }
    return 0;
}

int handle_record_04(uint16_t addr, void *ctx)
{
    addr_off = addr << 16;
    return 0;
}

int main(int argc, char *argv[])
{
    int fd;
    int i;
    struct context ctx;

    if (0 != parse_options(argc, argv))
    {
        usage();
        return 1;
    }

    if ((fd = serialOpen(device_name)) < 0)
    {
        fprintf(stderr, "Failed to open %s\n", device_name);
        return 1;
    }

    ctx.fd = fd;

    if (opt_flash)
    {
        if (0 != check_for_bootloader(fd, opt_timeout))
        {
            fprintf(stderr, "No bootloader detected\n");
            return 1;
        }
        else
        {
            printf("Bootloader detected\n");
        }

        for (i=0;i<FLASH_SIZE_KB/FLASH_PAGE_SIZE;i++)
        {
            printf("Erasing page %d\n", i);
            if (0 != erase_page(fd, i))
            {
                fprintf(stderr, "Erasing page failed\n");
                return 1;
            }
        }

        if (0 != read_hexfile(flash_filename, handle_record_00, handle_record_04, &ctx))
        {
            fprintf(stderr, "Failed to read %s\n", flash_filename);
            return 1;
        }

        if (0 != reset(fd))
        {
            fprintf(stderr, "Reset failed\n");
            return 1;
        }
    }

    if (opt_console)
    {
        atexit(do_exit);
        printf("Connected to %s, ctrl-c to exit\n", device_name);
        do_console(fd);
    }

    serialClose(fd);

    return 0;
}


