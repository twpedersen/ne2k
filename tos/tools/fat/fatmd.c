/*************************************************
 * FATMD 0.10                                    *
 *************************************************/

#include <fs.h>

int main(int argc, char** argv)
{
  TOS_FAT_Device device;

  if (argc != 4)
    {
      printf("USAGE: fatmd <imagefile> <parentdirectory> <directory>\n");
      return (-1);
    }

  if (!(fp = fopen(argv[1], "r+")))
    {
      printf("EXCEPTION: Image file could not be opened!\n");
      return (-1);
    }

  fs_init(&device);

  if (!fs_create_directory(&device, argv[2], argv[3]))
  {
    printf("EXCEPTION: Directory could not be created!\n");
    return (-1);
  }

  if (fclose(fp) == EOF)
    {
      printf("EXCEPTION: Image file could not be closed!\n");
      return (-1);
    }

  return (0);
}
