/******************************************************************************
 * FATFORMAT                                                                  *
 ******************************************************************************/

//#include <fs.h>
#include <stdio.h>
#include <stdlib.h>

void print_usage ()
{
  printf("USAGE: fatsys <image file> <boot_sector>\n");
}

int
main (int argc, char* argv[])
{
//  TOS_FAT_Device device;

//  unsigned int file_size, i;
//  unsigned char size[10] = "";
  FILE *image_file;
  
  //Variable to be used as the boot sector
  FILE *boot_sector;
  
  unsigned int i;
  
  unsigned char* buffer;
//  char temp;
//  BPB_FAT bpb_fat;
//  BPB_FAT16 bpb_fat16;
//  BPB_FAT32 bpb_fat32;

  if (!(argc == 3))
    {
      print_usage();
      return -1;
    }
  i=0;
 // printf("%s\n\n", argv[2]);
  //printf("%c\n", argv[1][0]);
  //Variable to be used as the image file
  


  boot_sector = fopen(argv[2], "rb");
  image_file = fopen(argv[1], "rb");
    
  buffer = (char*) malloc((unsigned int) 1474560);
  
  for (i = 0; i < 1474560; i++) buffer[i] = '\0';  
  i = 0;
  
  while (i < 3){
     buffer[i] = getc(boot_sector);
     i++;
   }
  
  //can't fseek since it would zero of the bytes needed
  fseek(image_file, 3, SEEK_SET);
  
  while(i < 62){
     buffer[i] = getc(image_file);
     i++;
  }
  
  //seek past place holding zeros in boot loader
  
  fseek(boot_sector, 62, SEEK_SET);
  fseek(image_file, 62, SEEK_SET);
  //finish off boot sector
  
  while(i< 512){
  
     buffer[i] = getc(boot_sector);
     i++;
     
  }
  
  fseek(image_file, 512, SEEK_SET);



  while( i < 1474560){
     buffer[i] = getc(image_file);
     i++;
   }
  
  fclose(image_file);
  fclose(boot_sector);
  
  image_file=fopen(argv[1],"wb");
  
  fwrite(buffer, 1474560, 1, image_file);
  
  fclose(image_file);
  
  free(buffer); 

  return 0;
}
