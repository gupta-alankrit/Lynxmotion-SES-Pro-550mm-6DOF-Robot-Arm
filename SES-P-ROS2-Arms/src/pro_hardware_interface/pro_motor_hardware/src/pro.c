#include "pro_motor_hardware/pro.h"
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <stdlib.h>

#define BUFFER_SIZE 128
static int g_serial_fd = -1;

bool pro_init_bus(const char* port_name, int baud_rate)
{
    if (!port_name) {
        errno = EINVAL;
        printf("Error: port_name is NULL\n");
        return false;
    }

    printf("Attempting to open port: %s\n", port_name);
    g_serial_fd = open(port_name, O_RDWR);
    if (g_serial_fd < 0) {
        printf("Failed to open port: %s (errno: %d - %s)\n", 
               port_name, errno, strerror(errno));
        return false;
    }

    struct termios tty;
    if (tcgetattr(g_serial_fd, &tty) != 0) {
        printf("Failed to get port attributes (errno: %d - %s)\n", 
               errno, strerror(errno));
        close(g_serial_fd);
        return false;
    }

    // Raw-mode baseline: clears ICRNL/IXON/OPOST/ICANON/ECHO so LSS-P
    // framing (\r terminator) passes through unmangled.
    cfmakeraw(&tty);

    // Set baud rate
    speed_t speed;
    switch (baud_rate) {
        case 921600:  speed = B921600;  break;
        case 115200: speed = B115200; break;
        default:     
            printf("Using default baud rate 921600\n");
            speed = B921600; 
            break;
    }
    
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tty.c_cflag &= ~PARENB;        // No parity
    tty.c_cflag &= ~CSTOPB;        // 1 stop bit
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;            // 8 bits per byte

    // Handle flow control
    #ifdef CRTSCTS
        tty.c_cflag &= ~CRTSCTS;   // No flow control
    #endif

    tty.c_cflag |= CREAD | CLOCAL;

    // Set timeout to 100ms
    tty.c_cc[VTIME] = 1;           // 0.1 seconds timeout
    tty.c_cc[VMIN] = 0;

    tcflush(g_serial_fd, TCIOFLUSH);   // drop any stale RX/TX bytes
    if (tcsetattr(g_serial_fd, TCSANOW, &tty) != 0) {
        close(g_serial_fd);
        return false;
    }

    return true;
}

void pro_close_bus(void)
{
    if (g_serial_fd >= 0) {
        close(g_serial_fd);
        g_serial_fd = -1;
    }
}

static bool generic_write(int id, const char* cmd, const char* param)
{
    if (g_serial_fd < 0) return false;

    char buffer[BUFFER_SIZE];
    size_t len;
    
    if (param) {
        len = snprintf(buffer, BUFFER_SIZE, "%s%d%s%s%s", 
                PRO_CommandStart, id, cmd, param, PRO_CommandEnd);
    } else {
        len = snprintf(buffer, BUFFER_SIZE, "%s%d%s%s", 
                PRO_CommandStart, id, cmd, PRO_CommandEnd);
    }

    if (len >= BUFFER_SIZE) {
        errno = ENOBUFS;
        return false;
    }

    // Retry logic
    int retry_count = 0;
    const int max_retries = 3;
    
    while (retry_count < max_retries) {
        ssize_t bytes_written = write(g_serial_fd, buffer, len);
        if (bytes_written == (ssize_t)len) {
            return true;  // Success
        }
        
        // Only retry on certain errors
        if (errno != EAGAIN && errno != EIO) {
            return false;  // Don't retry on other errors
        }

        usleep(5000);  // 5ms delay before retry
        retry_count++;
    }

    return false;  // Failed after all retries
}

static int generic_read_blocking_int(int id, const char* cmd)
{
    if (g_serial_fd < 0) {
        errno = EBADF;
        return -1;
    }

    int retry_count = 0;
    const int max_retries = 3;
    
    while (retry_count < max_retries) {
        char buffer[BUFFER_SIZE];
        int pos = 0;
        char c;
        bool timeout = false;

        // Find start character '*'
        int start_attempts = 0;
        const int max_start_attempts = 100;  // Prevent infinite loop
        
        do {
            if (read(g_serial_fd, &c, 1) != 1) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    if (++start_attempts >= max_start_attempts) {
                        timeout = true;
                        break;
                    }
                    usleep(1000);  // 1ms delay
                    continue;
                }
                if (errno == 0) errno = EIO;
                break;
            }
        } while (c != PRO_CommandReplyStart[0] && !timeout);

        if (timeout) {
            usleep(5000);  // 5ms delay before retry
            retry_count++;
            continue;
        }

        // Read until end character '\r'
        do {
            if (read(g_serial_fd, &c, 1) != 1) {
                if (errno == 0) errno = EIO;
                timeout = true;
                break;
            }
            if (c != PRO_CommandEnd[0]) {
                buffer[pos++] = c;
            }
        } while (c != PRO_CommandEnd[0] && pos < BUFFER_SIZE - 1 && !timeout);

        if (timeout) {
            usleep(5000);  // 5ms delay before retry
            retry_count++;
            continue;
        }

        buffer[pos] = '\0';

        // Parse response
        int read_id, value;
        char read_cmd[10];
        if (sscanf(buffer, "%d%[A-Z]%d", &read_id, read_cmd, &value) != 3) {
            usleep(5000);  // 5ms delay before retry
            retry_count++;
            continue;
        }

        if (read_id != id || strcmp(read_cmd, cmd) != 0) {
            usleep(5000);  // 5ms delay before retry
            retry_count++;
            continue;
        }

        return value;  // Success
    }

    return -1;  // Failed after all retries
}

PRO* pro_create(int id)
{
    PRO* pro = (PRO*)malloc(sizeof(PRO));
    if (pro) {
        pro->servo_id = id;
        pro->serial_fd = g_serial_fd;
        pro->is_initialized = true;
    }
    return pro;
}

void pro_destroy(PRO* pro)
{
    if (pro) {
        free(pro);
    }
}

bool pro_reset(PRO* pro)
{
    return generic_write(pro->servo_id, PRO_ActionReset, NULL);
}

bool pro_limp(PRO* pro)
{
    return generic_write(pro->servo_id, PRO_ActionLimp, NULL);
}

bool pro_hold(PRO* pro)
{
    return generic_write(pro->servo_id, PRO_ActionHold, NULL);
}

bool pro_move(PRO* pro, int position)
{
    char param[20];
    snprintf(param, sizeof(param), "%d", position);
    return generic_write(pro->servo_id, PRO_ActionMove, param);
}

bool pro_move_timed(PRO* pro, int position, int move_time_ms)
{
    char param[40];
    snprintf(param, sizeof(param), "%d%s%d", position, PRO_ModifierMoveTimed, move_time_ms);
    return generic_write(pro->servo_id, PRO_ActionMove, param);
}

bool pro_move_speed(PRO* pro, int position, int speed)
{
    char param[40];
    snprintf(param, sizeof(param), "%d%s%d", position, PRO_ModifierMoveSpeed, speed);
    return generic_write(pro->servo_id, PRO_ActionMove, param);
}

bool pro_set_color_led(PRO* pro, int color)
{
    char param[20];
    snprintf(param, sizeof(param), "%d", color);
    return generic_write(pro->servo_id, PRO_ActionColorLED, param);
}

bool pro_set_max_speed(PRO* pro, int speed)
{
    char param[20];
    snprintf(param, sizeof(param), "%d", speed);
    return generic_write(pro->servo_id, PRO_ActionMaxSpeed, param);
}

bool pro_set_acceleration(PRO* pro, int acceleration)
{
    char param[20];
    snprintf(param, sizeof(param), "%d", acceleration);
    return generic_write(pro->servo_id, PRO_ActionAccel, param);
}

bool pro_set_deceleration(PRO* pro, int deceleration)
{
    char param[20];
    snprintf(param, sizeof(param), "%d", deceleration);
    return generic_write(pro->servo_id, PRO_ActionDecel, param);
}

int pro_get_position(PRO* pro)
{
    generic_write(pro->servo_id, PRO_QueryPosition, NULL);
    return generic_read_blocking_int(pro->servo_id, PRO_QueryPosition);
}

int pro_get_speed(PRO* pro)
{
    generic_write(pro->servo_id, PRO_QuerySpeed, NULL);
    return generic_read_blocking_int(pro->servo_id, PRO_QuerySpeed);
}

int pro_get_status(PRO* pro)
{
    generic_write(pro->servo_id, PRO_QueryStatus, NULL);
    return generic_read_blocking_int(pro->servo_id, PRO_QueryStatus);
}