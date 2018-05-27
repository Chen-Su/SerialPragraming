
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <stdlib.h>

/*
 * 'open_port()' - Open serial port 1.
 *
 * Returns the file descriptor on success or -1 on error.
 */

int open_port(const char *port_name)
{
  int fd; /* File descriptor for the port */


  fd = open(port_name, O_RDWR | O_NOCTTY | FNDELAY);
  if (fd == -1)
  {
   /*
    * Could not open the port.
    */

    printf("open_port: Unable to open %s\n", port_name);
    exit(-1);
  }
  else
    fcntl(fd, F_SETFL, 0);    // Set to block mode

  return fd;
}

void config_port(int fd)
{

  // set baud rate
  struct termios options;
/*
 * Get the current options for the port...
 */

  tcgetattr(fd, &options);

/*
 * Set the baud rates to 19200...
 */

  cfsetispeed(&options, B19200);
  cfsetospeed(&options, B19200);

/*
 * Enable the receiver and set local mode...
 */

  options.c_cflag |= (CLOCAL | CREAD);

  // No parity (8N1):
  options.c_cflag &= ~PARENB;
  options.c_cflag &= ~CSTOPB;
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;

  // Setting Hardware Flow Control
  // options.c_cflag |= CNEW_RTSCTS;
  options.c_cflag &= ~CRTSCTS;  // Disable hardware flow control

  // Setting Software Flow Control
  options.c_iflag |= (IXON | IXOFF | IXANY);

  // Choosing Raw Input
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  
  /*Choosing Raw Output*/
  options.c_oflag &= ~OPOST;

  // Configure timeout
  options.c_cc[VMIN] = 1;
  options.c_cc[VTIME] = 0;
/*
 * Set the new options for the port...
 */

  if(-1 == tcsetattr(fd, TCSANOW, &options))
    printf("Config port failed.\n");
  else
    printf("Config port success.\n");
}


int main(int argc, char *argv[])
{
  if(argc != 2)
  {
    printf("usage : exec write_port\n");
  }
  int fd_w = open_port(argv[1]);

  config_port(fd_w);


  char sign = 0;
  do{
    sign = getchar();
    printf("get data from stdin : %c\n", sign);
    getchar();  // skip '\n'
    printf("write data to port...");
    write(fd_w, &sign, 1);
    printf("done.\n");
  }while(sign != 'q');

  close(fd_w);
  return 0;
}
