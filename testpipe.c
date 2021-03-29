// C program to implement one side of FIFO 
// This side reads first, then reads 
#include <stdio.h> 
#include <string.h> 
#include <fcntl.h> 
#include <sys/stat.h> 
#include <sys/types.h> 
#include <unistd.h>

void printData(int len, char *data)
{
	printf("Incoming data: \n\t");

	for (int i = 0; i < len; i++)
	{
		printf("%02X ", data[i]);
	}
	printf("\n\n");

}

int main() 
{ 
	int fd1;
	int len = 76;

	// FIFO file path 
	char *myfifo = "/tmp/emg"; 

	// Creating the named file(FIFO) 
	// mkfifo(<pathname>,<permission>) 
	mkfifo(myfifo, 0666); 

	char str1[len];//, str2[80]; 
	while (1) 
	{ 
		// First open in read only and read 
		fd1 = open(myfifo, O_RDONLY); 
		read(fd1, str1, len); 

		// Print the read string and close 
		// printf("Incoming data: \n\t %02X\n", str1[0]);
		printData(len, str1);
		close(fd1); 

		sleep(1);

		// // Now open in write mode and write 
		// // string taken from user. 
		// fd1 = open(myfifo,O_WRONLY); 
		// fgets(str2, 80, stdin); 
		// write(fd1, str2, strlen(str2)+1); 
		// close(fd1); 
	} 
	return 0; 
} 
