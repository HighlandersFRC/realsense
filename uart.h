#include <string>

#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <termios.h>
#include <unistd.h>

namespace Robot {

    class UartStream {
    private:
        int fd;
        std::string buffer;

        termios get_termios();
        void set_termios(termios tty);
        void configure_blocking(int min_chars, int interchar_timeout);

    public:
        UartStream(std::string port_name, speed_t baud, bool parity);
        UartStream(std::string port_name, speed_t baud): UartStream(port_name, baud, false) {};
        UartStream(std::string port_name): UartStream(port_name, B115200, false) {};
        ~UartStream();

        std::string read_line_uart();
        std::string read_uart();
        void write_uart(std::string str);
    };

    // UartStream& operator>>(UartStream &uart, std::string &str) { 
    //     str = uart.read_line();
	   //  return uart;
    // }

    // UartStream& operator<<(UartStream &uart, const std::string &str) {
    //     uart.write(str);
    //     return uart;
    // }
}

