#include "uart.h"

#include <iostream>
#include <stdexcept>

#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <termios.h>
#include <unistd.h>

using namespace Robot;

termios UartStream::get_termios() {
    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr (fd, &tty) != 0) {
        throw std::runtime_error("Error from tcgetattr!");
    }
    return tty;
}

void UartStream::set_termios(termios tty) {
    if (tcsetattr (fd, TCSANOW, &tty) != 0) {
        throw std::runtime_error("Error from tcsetattr!");
    }
}

void UartStream::configure_blocking(int min_chars, int interchar_timeout) {
    termios tty = get_termios();
    tty.c_cc[VMIN] = min_chars; // Minimum characters
    tty.c_cc[VTIME] = interchar_timeout; // Tenths of a sec between chars
    set_termios(tty);
}

UartStream::UartStream(std::string port_name, speed_t baud, bool parity) {
    fd = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        throw std::runtime_error("Error from file open!"); // strerror(errno)
    }
    
    termios tty = get_termios();

    cfsetospeed (&tty, baud);
    cfsetispeed (&tty, baud);
    
    if (not parity)
        tty.c_cflag &= ~PARENB;   /* Disables the Parity Enable bit(PARENB) */
	tty.c_cflag &= ~CSTOPB;   /* CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit */
    tty.c_cflag &= ~CSIZE;	 /* Clears the mask for setting the data size             */
	tty.c_cflag |=  CS8;      /* Set the data bits = 8                                 */
	tty.c_cflag &= ~CRTSCTS;       /* No Hardware flow Control                         */
	tty.c_cflag |= CREAD | CLOCAL; /* Enable receiver,Ignore Modem Control lines       */ 
	tty.c_iflag &= ~(IXON | IXOFF | IXANY);          /* Disable XON/XOFF */
	tty.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);  /* Non Cannonical mode */
	tty.c_oflag &= ~OPOST;/*No Output Processing*/

    set_termios(tty);
}

UartStream::~UartStream() {
    close(fd);
}

void UartStream::write_uart(std::string str) {
    write(fd, str.c_str(), (size_t) str.length());
}

std::string UartStream::read_uart() {
    // Wait for at least 1 char with 0.1s between further chars
    configure_blocking(1, 1);

    char buf [256];
    memset(buf, 0, sizeof(buf));
    // Save room for null-term
    size_t actual_num = read(fd, buf, sizeof(buf) - 1);

    // Big ???
    // sleep(2);
    // tcflush(fd, TCIOFLUSH);

    // Also return local buffer and clear it
    std::string s = buffer + std::string(buf);
    buffer = "";
    return s;
}

std::string UartStream::read_line_uart() {
    // Check buffer and then read indefinitely until we get a new-line
    std::string line = read_uart();
    std::string::size_type pos = line.find('\n', 0);
    while (pos == std::string::npos) {
        line += read_uart();
        pos = line.find('\n', 0);
    }

    // Extract first line and put remaining in the buffer
    buffer = line.substr(pos + 1);

    return line.substr(0, pos);
}

// Usually "/dev/ttyTHS1" or "/dev/ttyUSB0"

int main(int argc, char* argv[]) {
    
    UartStream us = UartStream(std::string(argv[1]));

    std::cout << "Starting terminal emulation program!\n";
    std::cout << "Options (w) Write Line, (r) Read Line, (q) Quit\n";
    std::cout << "-----------------------------------------------\n";
    
    bool quit = false;
    std::string response;
    while (not quit) {
        std::cout << "> " << std::flush;
        getline(std::cin, response);
        if (response == "q") {
            quit = true;
        } else if (response == "w") {
            std::cout << "Write: " << std::flush;
            getline(std::cin, response);
            us.write_uart(response + "\n");
        } else if (response == "r") {
            std::cout << "Read: " << us.read_line_uart() << "\n";
        } else {
            std::cout << "Invalid Option!\n";
        }
    }

	return 0;
}

