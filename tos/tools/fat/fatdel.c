/******************************************************************************
 * FATDEL                                                                     *
 ******************************************************************************/

#include <fs.h>

int main(int argc, char** argv)
{
  TOS_FAT_Device device;

  FAT_DIR_ENTRY dir_entry;

  unsigned int current_cluster, next_cluster;

  if (argc != 3)
    {
      printf("USAGE: fatdel <image file> <file>\n");
      return -1;
    }

  if (!(fp = fopen(argv[1], "r+")))
    {
      printf("EXCEPTION: Image file could not be opened!\n");
      return -1;
    }

  fs_init(&device);

  if (fs_get_directory_entry(&device, argv[2], &dir_entry) == 0)
    {
      printf("EXCEPTION: File could not be found!\n");
      return -1;
    }

  dir_entry.dir_name[0] = 0xE5; // mark directory entry as deleted

  if (fs_update_directory_entry(&device, argv[2], dir_entry) == 0)
    {
      printf("EXCEPTION: File could not be deleted!\n");
      return -1;
    }

  current_cluster = dir_entry.dir_fst_clus_hi;
  current_cluster = (current_cluster << 8) + dir_entry.dir_fst_clus_lo;

  // free clusters

  while (!fs_is_eoc(&device, next_cluster = fs_get_fat_cluster_entry(&device, current_cluster)))
    {
      fs_set_fat_cluster_entry(&device, current_cluster, 0);
      current_cluster = next_cluster;
    }

  fs_set_fat_cluster_entry(&device, current_cluster, 0);

  if (fclose(fp) == EOF)
    {
      printf("EXCEPTION: Image file could not be closed!\n");
      return -1;
    }
  return 0;
}

