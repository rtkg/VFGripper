#include <cstdio>
#include <cstring>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <fcntl.h>   // File control definitions
#include <sys/types.h>
#include <sys/stat.h>
#include <cstdlib>

using namespace std;

int
set_interface_attribs (int fd, int speed, int parity)
{
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0)
    {
	printf("error %d from tcgetattr", errno);
	return -1;
    }

    cfsetospeed (&tty, speed);
    cfsetispeed (&tty, speed);

    //8N1
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
  
    tty.c_lflag |= ICANON;
#if 0 
    /* no hardware flow control */
    tty.c_cflag &= ~CRTSCTS;
    /* enable receiver, ignore status lines */
    tty.c_cflag |= CREAD | CLOCAL;
    /* disable input/output flow control, disable restart chars */
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    /* disable canonical input, disable echo,
       disable visually erase chars,
       disable terminal-generated signals */
    tty.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    /* disable output processing */
    tty.c_oflag &= ~OPOST;
    tty.c_cc[VMIN]  = 20;
    tty.c_cc[VTIME] = 20;
#endif

    if (tcsetattr (fd, TCSANOW, &tty) != 0)
    {
	printf ("error %d from tcsetattr", errno);
	return -1;
    }
    return 0;
}




int main(int argc, char* argv[]) {


    if(argc !=4 ) {
	std::cerr<<"usage: "<<argv[0]<<" port_string [POS|NON] val\n";
	return -1;
    }
    
    const char* portname = argv[1];
    char *comm = argv[2];
    short pos = 0;
    
    if (strncmp(comm,"POS",3) == 0) {
	pos = atoi(argv[3]);
    }	
    std::cout<<"connecting to "<<portname<<" sending "<<comm<<" "<<pos<<std::endl;

    int fd = open (portname, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0)
    {
	printf("error %d opening %s: %s", errno, portname, strerror (errno));
	return -1;
    }
    sleep(2);

    set_interface_attribs (fd, B19200, 0);  // set speed to 9600 bps, 8n1 (no parity)
    tcflush(fd, 0);

    short msg_size = sizeof(short) + 3; 
    char *payload = new char[msg_size];
    snprintf(payload,4,"%s",comm);
    memcpy(payload+3,&pos,sizeof(short));
    int wrote = write (fd, payload , msg_size);
    if(wrote != msg_size) {
	printf("wrote %d, error!\n",wrote);
    }
    printf("msg size %d, string %s\n",msg_size,payload);
    delete [] payload;

    fd_set rfds;
    struct timeval tv;
    int retval;

    FD_ZERO(&rfds);
    FD_SET(fd, &rfds);

    /* Wait up to five seconds. */
    tv.tv_sec = 5;
    tv.tv_usec = 0;
    
    char buf [1000];
    int sofar=0;

    while(1) {
	retval = select(fd+1, &rfds, NULL, NULL, NULL); //&tv);
	/* Don't rely on the value of tv now! */
/*
	if (retval == -1)
	    perror("select()");
	else if (retval)
	    printf("Data is available now.\n");
	else
	    printf("No data within five seconds.\n");
*/
	if(sofar > 500) {
	    cout<<buf;
	    sofar=0;
	}
	int n = read (fd, buf+sofar, sizeof(buf)-sofar);  // read up to 100 characters if ready to read
	if ( n>0) {
	    sofar+=n;
	}
    }
    close(fd);
}
