#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/kdev_t.h>
 
/* 사용할 장치 파일의 이름이 변경됨 */
#define _MORSE_PATH_ "/dev/DPG_i2s"
 
int main(int argc, char *argv[]){
	int fd = 0;
	        /* 장치파일의 주번호가 변경됨 */
	//mknod(_MORSE_PATH_, S_IRWXU | S_IRWXG | S_IFCHR, MKDEV(221, 0));
	if((fd = open( _MORSE_PATH_, O_RDWR | O_NONBLOCK)) < 0){
	            printf("aiejfliajeflijawef\n");
                    exit(1);  
	}
        printf("open sungkong!\n"); sleep(2);	
	close(fd);
	return 0;
}
