#include <stdio.h>
#include <fcntl.h>  
#include <termios.h> 
#include <unistd.h>  
#include <errno.h>   
#include <stdlib.h>
#include <pthread.h>
int fd;
void* call_print(void* args);
struct termios oldt, newt;
void main(void)
{
		
		
       fd = open("/dev/ttyUSB0",O_RDWR | O_NOCTTY);	
			
      	if(fd < 0) {						
		printf("\n  Error! in Opening ttyUSB0  ");
		exit(0);
	} 
		
  	int ch;
  	tcgetattr(STDIN_FILENO, &oldt );
  	newt = oldt;
  	newt.c_lflag &= ~( ICANON | ECHO );
  	tcsetattr( STDIN_FILENO, TCSANOW, &newt );
  		
	struct termios SerialPortSettings;
	tcgetattr(fd, &SerialPortSettings);

	cfsetispeed(&SerialPortSettings,B9600); 
	cfsetospeed(&SerialPortSettings,B9600); 

		
	SerialPortSettings.c_cflag &= ~PARENB;   
	SerialPortSettings.c_cflag &= ~CSTOPB;   
	SerialPortSettings.c_cflag &= ~CSIZE;	 
	SerialPortSettings.c_cflag |=  CS8; 
		
	SerialPortSettings.c_cflag &= ~CRTSCTS;      
	SerialPortSettings.c_cflag |= CREAD | CLOCAL;  
		
	SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);        
	SerialPortSettings.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG); 

	SerialPortSettings.c_oflag &= ~OPOST;
		
	SerialPortSettings.c_cc[VMIN] = 40; 
	SerialPortSettings.c_cc[VTIME] = 10; 


	if((tcsetattr(fd,TCSANOW,&SerialPortSettings)) != 0) 
		printf("\n  ERROR ! in Setting attributes");
	else {
		printf("\n +----------------------------------+");
		printf("\n |           TemLedAttor            |");
		printf("\n +----------------------------------+");
		printf("\n | 1 | Green -  Led                 |");
		printf("\n | 2 | Ember -  Led                 |");
		printf("\n | 3 | Red   -  Led                 |");
		printf("\n | 4 | Blue  -  Led                 |");
		printf("\n +----------------------------------+");
		printf("\n | 0 | Temperature in celsius       |");
		printf("\n +----------------------------------+");
		printf("\n +----------------------------------+");
		printf("\n | q | Quit                         |");
		printf("\n +----------------------------------+");
		printf("\n\n\n");		
	}
		
	char read_buffer[32]; 
	int  bytes_read = 0;    
	int status;
	pthread_t thread; 
		
 	pthread_create(&thread, NULL, call_print, NULL);
 
	while (1) {
			
		tcflush(fd, TCIOFLUSH);
		bytes_read = read(fd, &read_buffer, 32);  
		printf("\n ");
		for(int i = 0; i < bytes_read; i++)	
			printf("%c", read_buffer[i]);
		printf("\n");
	}
	tcsetattr( STDIN_FILENO, TCSANOW, &oldt );
	close(fd);
	close(thread);		
}
    	
    	
void* call_print(void* args) {
int ch;
tcflush(fd, TCIOFLUSH);

while(ch = getchar()) {
	fflush(stdin);
	tcflush(fd, TCIOFLUSH);
	switch(ch) {
		case '1':
			write (fd, "1", 1);
			break;
		case '2':
			write (fd, "2", 1);
			break;
		case '3':
			write (fd, "3", 1);
			break;
		case '4':
			write (fd, "4", 1);
			break;
		case '0':
			write (fd, "0", 1);
			break;
		case 'q':
			tcsetattr( STDIN_FILENO, TCSANOW, &oldt );
			close(fd);
			exit(0);
			break;
		default: 
			printf("Wrong command!\n");
			break;
		}
		usleep (300); 
		tcflush(fd, TCIOFLUSH);
    	}
    	return 0;
}
		
