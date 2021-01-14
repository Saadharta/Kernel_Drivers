#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <linux/ioctl.h>
#include <errno.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#define SAMPLE_IOC_MAGIC 'k'
#define SAMPLE_IOC_CIPHER 'c'
#define SAMPLE_IOCRESET _IO(SAMPLE_IOC_MAGIC, 0)
#define SAMPLE_IOCCIPHER _IO(SAMPLE_IOC_CIPHER, 1)


const char *key = NULL;
char *c_key = NULL;
int main( int argc, char* argv[] ){
  int file;
  if(argc >= 2){
      printf("Doing %s on %s\n", argv[2], argv[1]);
      file = open(argv[1], O_RDONLY);
      if (file < 0) {
          perror ("open");
          printf("Error opening file %s\n",argv[1]);
          return(errno);
      }
      key = argv[2];
      c_key = (char*) argv[3];
      printf("%s\n",c_key);
      if (key[0] == 'k')
        printf("%d\n", ioctl(file,SAMPLE_IOCRESET, 0));
      if (key[0] == 'c')
        printf("%d\n", ioctl(file,SAMPLE_IOCCIPHER, c_key));
      close(file);
  }
  else
    printf("usage : %s <filename> <k|c>\n",argv[0]);
  return 0;
}