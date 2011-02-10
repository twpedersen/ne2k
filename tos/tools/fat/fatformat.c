/*************************************************
 * FATFORMAT 0.10                                *
 *************************************************/

#include <fs.h>
#include <string.h>
#include <stdlib.h>

void print_usage ()
{
  printf("USAGE: fatformat <imagefile> [size=<size>|DISC]\n");
}

int main (int argc, char** argv)
{
  unsigned int file_size, i;
  unsigned char size[10] = "";

  unsigned char* buffer;

  BPB_FAT bpb_fat;
  BPB_FAT16 bpb_fat16;
//  BPB_FAT32 bpb_fat32;

  if (!((argc == 2) || (argc == 3)))
    {
      print_usage();
      return (-1);
    }

  if (!(fp = fopen(argv[1], "r+")))
    {
      if ((argc == 2) || (strncmp(argv[2], "size=", 5) != 0))
	{
	  print_usage();
	  return -1;
	}

      strncpy(size, &argv[2][5], strlen(&argv[2][5]));
      size[strlen(&argv[2][5])] = '\0';

      if (strcmp(size, "") == 0)
	{
	  print_usage();
	  return -1;
	}

      if (strcmp(size, "DISC") == 0)
	file_size = 1474560;
      else
	file_size = atoi(size);

      buffer = (char*) malloc(file_size);

      for (i = 0; i < file_size; i++)
	buffer[i] = '\0';

      if (strcmp(size, "DISC") == 0)
	{
	  bpb_fat.bs_jmp_boot[0] = 0xEB;
	  bpb_fat.bs_jmp_boot[1] = 0x3D;
	  bpb_fat.bs_jmp_boot[2] = 0x90;

	  strncpy(bpb_fat.bs_oem_name, "MSWIN4.1", 8);

	  bpb_fat.bpb_byts_per_sec = swap16(512);
	  bpb_fat.bpb_sec_per_clus = 1;
	  bpb_fat.bpb_rsvd_sec_cnt = swap16(1);
	  bpb_fat.bpb_num_fats = 2;
	  bpb_fat.bpb_root_ent_cnt = swap16(224);
	  bpb_fat.bpb_tot_sec16 = swap16(2880);
	  bpb_fat.bpb_media = 0xF0;
	  bpb_fat.bpb_fat_sz16 = swap16(9);
	  bpb_fat.bpb_sec_per_trk = swap16(18);
	  bpb_fat.bpb_num_heads = swap16(2);
	  bpb_fat.bpb_hidd_sec = swap32(0);
	  bpb_fat.bpb_tot_sec32 = swap32(0);

	  bpb_fat16.bs_drv_num = 0;
	  bpb_fat16.bs_reserved1 = 0;
	  bpb_fat16.bs_boot_sig = 0x29;
	  bpb_fat16.bs_vol_id = swap32(0);
	  
	  strncpy(bpb_fat16.bs_vol_lab, "NO NAME    ", 11);
	  strncpy(bpb_fat16.bs_fil_sys_type, "FAT12   ", 8);

	  bpb_fat.u.fat16 = bpb_fat16;

	  memcpy(buffer, &bpb_fat, 62);
	}

      buffer[510] = 0x55;
      buffer[511] = 0xAA;

      if (!(fp = fopen(argv[1], "w")))
	{
	  printf("EXCEPTION: Image file could not be written!\n");
	  return -1;
	}

      fwrite(buffer, file_size, 1, fp);

      if (fclose(fp) == EOF)
	{
	  printf("EXCEPTION: Image file could not be closed!\n");
	  return -1;
	}

      free (buffer);
    }
  else
    {
      fseek(fp, 0, SEEK_END);
      file_size = ftell(fp);
      printf("FILESIZE = %d\n", file_size);
    }
  return 0;
}
