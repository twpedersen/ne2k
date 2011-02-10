/******************************************************************************
 * FATDIR                                                                     *
 ******************************************************************************/

#include <fs.h>
#include <string.h>

int print_directory(TOS_FAT_Device* dev, TOS_Octet* directory)
{
  TOS_UInt32 cluster, i, j, next_cluster, start_sector;

  TOS_Octet file_name[12];

  FAT_DIR_ENTRY dir_entry;
  FAT_DIR_ENTRY dir_entries[128];

  printf("Directory for %s\n\n", directory);

  if ((strcmp(directory, "/") == 0) && (dev->fat_type != FAT32_TYPE)) // (directory is root directory) and (FAT type is FAT12 or FAT16)
    {
      start_sector = dev->rsvd_sec_cnt + (dev->num_fat * dev->fat_size);

      for (i = 0; i < dev->root_dir_secs; i++)
	{
	  fs_bio_read(dir_entries, start_sector, 1);

	  for (j = 0; j < dev->bytes_per_sector / 32; j++)
	    {
	      if (dir_entries[j].dir_name[0] == 0x00)
		return 1;

	      if (dir_entries[j].dir_attr & 0x0F) // long file name - cannot be handled yet!
		{
#if 0
//AP
		  printf("<LONG FILE NAME - IGNORED!>\n");
#endif
		  continue;
		}

	      if (dir_entries[j].dir_name[0] != 0xE5)
		{
		  strncpy(file_name, &dir_entries[j].dir_name[0], 11);
		  file_name[11] = '\0';

		  if (dir_entries[j].dir_attr & 0x10)
		    printf("%s\t<DIRECTORY>\n", file_name);
		  else
		    printf("%s\n", file_name);
		}
	    }

	  start_sector++;
	}
    }
  else
    {
      if (!fs_get_directory_entry(dev, directory, &dir_entry))
	return 0;

      if ((dir_entry.dir_attr & 0x10) != 0x10) // directory_entry is not a directory
	return 0;

      cluster = dir_entry.dir_fst_clus_hi;
      cluster = (cluster << 8) + dir_entry.dir_fst_clus_lo;

      do
	{
	  next_cluster = fs_get_fat_cluster_entry(dev, cluster);

	  start_sector = fs_get_first_sector_of_cluster(dev, cluster);

	  for (i = 0; i < dev->sectors_per_cluster; i++)
	    {
	      fs_bio_read(dir_entries, start_sector, 1);

	      for (j = 0; j < dev->bytes_per_sector / 32; j++)
		{
		  if (dir_entries[j].dir_name[0] == 0x00)
		    return 1;

		  if (dir_entries[j].dir_name[0] != 0xE5)
		    {
		      strncpy(file_name, &dir_entries[j].dir_name[0], 11);
		      file_name[11] = '\0';

		      if (dir_entries[j].dir_attr & 0x10)
			printf("%s\t<DIRECTORY>\n", file_name);
		      else
			printf("%s\n", file_name);
		    }
		}

	      start_sector++;
	    }

	  cluster = next_cluster;
	}
      while (!fs_is_eoc(dev, next_cluster));
    }

  return (0);
}

int main(int argc, char** argv)
{
  TOS_FAT_Device device;

  if (argc != 3)
    {
      printf("USAGE: fatdir <image file> <directory>\n");
      return (-1);
    }

  if (!(fp = fopen(argv[1], "r+")))
    {
      printf("EXCEPTION: Image file could not be opened!\n");
      return (-1);
    }

  fs_init(&device);

  if (!print_directory(&device, argv[2]))
    {
      printf("EXCEPTION: Directory could not be read!\n");
      return (-1);
    }

  if (fclose(fp) == EOF)
    {
      printf("EXCEPTION: Image file could not be closed!\n");
      return (-1);
    }

  return (0);
}
