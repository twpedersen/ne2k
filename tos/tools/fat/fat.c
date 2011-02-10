#include <assert.h>
#include <fs.h>

#ifndef FS_STANDALONE

#include <kernel.h>
#include "fd.h"

VIR_ADDR	file_copy_buffer;
VIR_ADDR	driver_copy_buffer;
PORT_ID		floppy_port;

#else
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#endif

TOS_UInt16 swap16(TOS_UInt16 val)
{
#if BYTE_ORDER == LITTLE_ENDIAN
    return (val);
#else
    TOS_UInt16 r = (val & 0x00ff) << 8;
    r |= (val & 0xff00) >> 8;
    return (r);
#endif
}

TOS_UInt32 swap32(TOS_UInt32 val)
{
#if BYTE_ORDER == LITTLE_ENDIAN
    return (val);
#else
    TOS_UInt32 r = (val & 0x000000ff) << 24;
    r |= (val & 0x0000ff00) << 8;
    r |= (val & 0x00ff0000) >> 8;
    r |= (val & 0xff000000) >> 24;

    return (r);
#endif

}

#ifndef FS_STANDALONE

char* strncpy (char* dest, const char* src, int count)
{
  char* tmp = dest;

  while (count-- && (*dest++ = *src++) != '\0')
    /* nothing */;

  return tmp;
}

int strcmp (const char* cs, const char* ct)
{
  register signed char __res;

  while (1) {
    if ((__res = *cs - *ct++) != 0 || !*cs++)
      break;
  }

  return __res;
}

#endif

void str2up(char* string)
{
  int i;

  for (i = 0; i < strlen(string); i++)
    if ((string[i] > 96) && (string[i] < 123))
      string[i] -= 32;
}

void check_sizeof_types ()
{
    assert (sizeof (TOS_Octet) == 1);
    assert (sizeof (TOS_UInt16) == 2);
    assert (sizeof (TOS_Int16) == 2);
    assert (sizeof (TOS_UInt32) == 4);
    assert (sizeof (TOS_Int32) == 4);

    assert (sizeof (BPB_FAT16) == 26);
    assert (sizeof (BPB_FAT32) == 54);
    assert (sizeof (BPB_FAT) == 36 + sizeof (BPB_FAT32));

    assert (sizeof (FAT_DIR_ENTRY) == 32);
}

/******************************************************************************
 * Block I/O subsystem                                                        *
 ******************************************************************************/

#ifndef FS_STANDALONE

u_int SendMessage( ToDriverMsg* toDriverMsgPtr )
{
    ToDriverMsg *msg;

    msg = HERMES_DATA_PTR( driver_copy_buffer );
    *msg = *toDriverMsgPtr;
    msg->MajorDevice = 0;
    msg->MinorDevice = 0;
#if 0
    switch( toDriverMsgPtr->MajorDevice ) {
    case FLOPPY_DISK:
#endif
	k_message( floppy_port, driver_copy_buffer, sizeof( ToDriverMsg ) );
#if 0
	break;
    default:
	shutdown( "SendMessage(): Bad major device" );
    }
#endif
    return TOS_NO_ERROR;
}

u_int AwaitReply (DriverMsg** driverMsgPPtr)
{
    PID		pid;
    int		nbytes;

    close_port( file_server_port );
    k_receive( driver_copy_buffer, &pid, &nbytes );
    assert( nbytes == sizeof( DriverMsg ) );
    *driverMsgPPtr = HERMES_DATA_PTR( driver_copy_buffer );
    open_port( file_server_port );
    return TOS_NO_ERROR;
}

TOS_Error fs_init_bio()
{
    return TOS_NO_ERROR;
}

TOS_Error fs_bio_read (void* buffer, TOS_UInt32 start_sector, TOS_UInt16 sector_count)
{
    ToDriverMsg msg;
    DriverMsg* reply;
    int i, j;
    TOS_Octet* user_buf = (TOS_Octet*) buffer;
    
    for (j = 0; j < sector_count; j++) {
	msg.Size = sizeof (ToDriverMsg);
	msg.Type = DEVICE_READ;
	msg.MajorDevice = 0;
	msg.MinorDevice = 0;
	msg.SectorAddr = start_sector++;
	msg.MemoryAddr = 0;
	SendMessage (&msg);
	AwaitReply (&reply);
	
	for (i = 0; i < BIO_SECTOR_SIZE; i++)
	    *user_buf++ = peek_mem_b (i);
    }
    
    return TOS_NO_ERROR;
}

TOS_Error fs_bio_write (void* buffer, TOS_UInt32 start_sector, TOS_UInt16 sector_count)
{
     ToDriverMsg msg;
     DriverMsg* reply;
     int i, j;
     TOS_Octet* user_buf = (TOS_Octet*) buffer;

     for (j = 0; j < sector_count; j++) {
	for (i = 0; i < BIO_SECTOR_SIZE; i++)
	    poke_mem_b (i, *user_buf++);
	msg.Size = sizeof (ToDriverMsg);
	msg.Type = DEVICE_WRITE;
	msg.MajorDevice = 0;
	msg.MinorDevice = 0;
	msg.SectorAddr = start_sector++;
	msg.MemoryAddr = 0;
	SendMessage (&msg);
	AwaitReply (&reply);
     }

     return TOS_NO_ERROR;
}

#else

TOS_Error fs_init_bio ()
{
  return TOS_NO_ERROR;
}

TOS_Error fs_bio_read (void* buffer, TOS_UInt32 start_sector, TOS_UInt16 sector_count)
{
  fflush(fp);
  fseek(fp, start_sector * BIO_SECTOR_SIZE, SEEK_SET);
  fread(buffer, sector_count * BIO_SECTOR_SIZE, 1, fp);

  return TOS_NO_ERROR;
}

TOS_Error fs_bio_write (void* buffer,  TOS_UInt32 start_sector, TOS_UInt16 sector_count)
{
  fseek(fp, start_sector * BIO_SECTOR_SIZE, SEEK_SET);
  fwrite(buffer, sector_count * BIO_SECTOR_SIZE, 1, fp);

  return TOS_NO_ERROR;
}

#endif

/******************************************************************************
 * FAT file system access                                                     *
 ******************************************************************************/

void fs_check_fat (BPB_FAT* fat)
{
    TOS_UInt16 bps;
    
    TOS_CHECK (fat->bs_jmp_boot[0] == 0xEB ||
	       fat->bs_jmp_boot[0] == 0xE9);

    bps = swap16(fat->bpb_byts_per_sec);
    TOS_CHECK (bps == 512 || bps == 1024 || bps == 2048 || bps == 4096);

    TOS_CHECK (fat->bpb_sec_per_clus == 1 ||
	       fat->bpb_sec_per_clus == 2 ||
	       fat->bpb_sec_per_clus == 4 ||
	       fat->bpb_sec_per_clus == 8 ||
	       fat->bpb_sec_per_clus == 16 ||
	       fat->bpb_sec_per_clus == 32 ||
	       fat->bpb_sec_per_clus == 64 ||
	       fat->bpb_sec_per_clus == 128);
    
    TOS_CHECK (fat->bpb_num_fats == 2);

    //???TOS_CHECK (fat->bs_jmp_boot[0] == fat->bpb_media);
}

void fs_init (TOS_FAT_Device* dev)
{
    Sector sector;
    BPB_FAT* fat = (BPB_FAT*) &sector;
    TOS_UInt16 sz16, rsvd_sec_cnt, bytes_per_sec, root_ent_cnt, tot_sec16;

    // Initialize the basic I/O system
    fs_init_bio();

    // Initialize FCB
    TOS_init_fcb();
    
    fs_bio_read (&sector, 0, 1);
    fs_check_fat (fat);
    
    sz16 = swap16(fat->bpb_fat_sz16);
    rsvd_sec_cnt = swap16(fat->bpb_rsvd_sec_cnt);
    bytes_per_sec = swap16(fat->bpb_byts_per_sec);
    root_ent_cnt = swap16(fat->bpb_root_ent_cnt);

    dev->num_fat = fat->bpb_num_fats;
    dev->bytes_per_sector = bytes_per_sec;
    
    // Size of the FAT in number of sectors
    if (sz16 != 0)
	dev->fat_size = sz16;
    else
	dev->fat_size = swap32(fat->u.fat32.bpb_fat_sz32);
    
    dev->sectors_per_cluster = fat->bpb_sec_per_clus;

    dev->rsvd_sec_cnt = rsvd_sec_cnt;
    
    // Compute number of sectors for root directory
    dev->root_dir_secs =
	((root_ent_cnt * 32) + (bytes_per_sec - 1)) / bytes_per_sec;

    // Compute first root dir sector
    dev->first_root_dir_sector =
	rsvd_sec_cnt + (fat->bpb_num_fats * dev->fat_size);
    
    // Compute first data sector
    dev->first_data_sector =
	rsvd_sec_cnt + (fat->bpb_num_fats * dev->fat_size) + dev->root_dir_secs;

    tot_sec16 = swap16(fat->bpb_tot_sec16);
    // # of sectors in the volume
    if (tot_sec16 != 0)
	dev->total_sectors = tot_sec16;
    else
	dev->total_sectors = swap32(fat->bpb_tot_sec32);
    
    // Compute the # of sectors in the data region of the volume
    dev->total_data_sectors = dev->total_sectors - 
	(rsvd_sec_cnt + (fat->bpb_num_fats * dev->fat_size) +
	 dev->root_dir_secs);
    
    // Compute the # of clusters in the volume
    dev->total_clusters = dev->total_data_sectors / fat->bpb_sec_per_clus;

    // Determine FAT type
    if (dev->total_clusters < 4085)
	dev->fat_type = FAT12_TYPE;
    else if (dev->total_clusters < 65525)
	dev->fat_type = FAT16_TYPE;
    else
	dev->fat_type = FAT32_TYPE;
    
    assert (dev->fat_type != FAT32_TYPE);

    // Load the FAT into memory
#ifdef FS_STANDALONE
    dev->fat = (TOS_Octet*) malloc (dev->bytes_per_sector * dev->fat_size);
#else
    dev->fat = (TOS_Octet*) k_sbrk (dev->bytes_per_sector * dev->fat_size);
#endif
    fs_bio_read (dev->fat, 1, dev->fat_size);
}

/******************************************************************************
 *                                                                            *
 * FUNCTION             = fs_get_first_sector_of_cluster                      *
 * DESCRIPTION          = gets the first sector of a cluster                  *
 *                                                                            *
 * IN VALUES:                                                                 *
 * dev                  = device                                              *
 * cluster              = cluster number                                      *
 *                                                                            *
 ******************************************************************************/

TOS_UInt32 fs_get_first_sector_of_cluster (TOS_FAT_Device* dev, TOS_UInt32 cluster)
{
  return ((cluster - 2) * dev->sectors_per_cluster) + dev->first_data_sector;
}

/******************************************************************************
 *                                                                            *
 * FUNCTION             = fs_get_fat_cluster_entry                            *
 * DESCRIPTION          = gets a fat cluster entry                            *
 *                                                                            *
 * IN VALUES:                                                                 *
 * dev                  = device                                              *
 * cluster              = cluster number                                      *
 *                                                                            *
 ******************************************************************************/

TOS_UInt32 fs_get_fat_cluster_entry (TOS_FAT_Device* dev, TOS_UInt32 cluster)
{
  TOS_UInt32 offset = 0;
  TOS_UInt32 value = 0;

  switch (dev->fat_type) {
  case FAT12_TYPE:
      offset = cluster + (cluster / 2);
      value = swap16(*((TOS_UInt16*) &dev->fat[offset]));

      if (cluster & 0x0001)
	  value = value >> 4;
      else
	  value = value & 0x0FFF;
      break;
      

  case FAT16_TYPE:
      offset = cluster * 2;
      value = swap16(*((TOS_UInt16*) &dev->fat[offset]));
      break;

  case FAT32_TYPE:
      offset = cluster * 4;
      value = swap32(*((TOS_UInt32*) &dev->fat[offset]));
      break;
  }
  return value;
}

/******************************************************************************
 *                                                                            *
 * FUNCTION             = fs_set_fat_cluster_entry                            *
 * DESCRIPTION          = sets a fat cluster entry                            *
 *                                                                            *
 * IN VALUES:                                                                 *
 * dev                  = device                                              *
 * cluster              = cluster number                                      *
 * value                = value of the FAT cluster entry                      *
 *                                                                            *
 ******************************************************************************/

void fs_set_fat_cluster_entry (TOS_FAT_Device* dev, TOS_UInt32 cluster, TOS_UInt32 value)
{
  TOS_UInt32 offset = 0;

  TOS_UInt32 thisFATEntOffset;
  TOS_UInt32 thisFATSecNum;

  int num_sec = 1;

  if (dev->fat_type == FAT12_TYPE)
    offset = cluster + (cluster / 2);

  if (dev->fat_type == FAT16_TYPE)
    offset = cluster * 2;

  if (dev->fat_type == FAT32_TYPE)
    offset = cluster * 4;

  thisFATSecNum = dev->rsvd_sec_cnt + (offset / dev->bytes_per_sector);

  // set FAT cluster entry in memory

  if (dev->fat_type == FAT12_TYPE)
    {
      if (cluster & 0x0001)
	{
	  value = value << 4;
	  value |= swap16(*((TOS_UInt16*) &dev->fat[offset])) & 0x000F;
	}
      else
	{
	  value = value & 0x0FFF;
	  value |= swap16(*((TOS_UInt16*) &dev->fat[offset])) & 0xF000;
	}

      *((TOS_UInt16*) &dev->fat[offset]) = swap16(value);

      thisFATEntOffset = offset % dev->bytes_per_sector;
      if (thisFATEntOffset == (dev->bytes_per_sector - 1))
	  num_sec = 2;
    }
  else if (dev->fat_type == FAT16_TYPE)
    {
	*((TOS_UInt16*) &dev->fat[offset]) = swap16(value);
    }
  else if (dev->fat_type == FAT32_TYPE)
    {
	value = value & 0x0FFFFFFF;
	value |= swap32(*((TOS_UInt32*) &dev->fat[offset])) & 0xF0000000;
	*((TOS_UInt32*) &dev->fat[offset]) = swap32(value);
    }

  // set FAT cluster entry on device
  
  fs_bio_write(&dev->fat[(offset / dev->bytes_per_sector) * dev->bytes_per_sector], thisFATSecNum, num_sec);
}

/******************************************************************************
 *                                                                            *
 * FUNCTION             = fs_is_eoc                                           *
 * DESCRIPTION          = checks if the cluster is the last in the cluster    *
 *                        chain                                               *
 *                                                                            *
 * IN VALUES:                                                                 *
 * dev                  = device                                              *
 * value                = value of the FAT cluster entry                      *
 *                                                                            *
 ******************************************************************************/

TOS_UInt32 fs_is_eoc(TOS_FAT_Device* dev, TOS_UInt32 value)
{
  if (dev->fat_type == FAT12_TYPE)
    return (value >= 0x0FF8);

  if (dev->fat_type == FAT16_TYPE)
    return (value >= 0xFFF8);

  if (dev->fat_type == FAT32_TYPE)
    return (value >= 0x0FFFFFF8);

  return 0;
}

/******************************************************************************
 *                                                                            *
 * FUNCTION             = fs_get_free_cluster                                 *
 * DESCRIPTION          = searches a free cluster                             *
 *                                                                            *
 * IN VALUES:                                                                 *
 * dev                  = device                                              *
 *                                                                            *
 * OUT VALUE:                                                                 *
 * cluster              = the searched cluster                                *
 *                                                                            *
 ******************************************************************************/

TOS_Bool fs_get_free_cluster(TOS_FAT_Device* dev, TOS_UInt32* cluster)
{
  for (*cluster = 2; *cluster <= dev->total_clusters; (*cluster)++)
    {
	if (!fs_get_fat_cluster_entry(dev, *cluster))
	    return TRUE;
	
    }

  return FALSE;
}

TOS_UInt32 fs_get_cluster(FAT_DIR_ENTRY* dir_entry)
{
    // XXX should this shift by 16?
    TOS_UInt32 cluster = swap16(dir_entry->dir_fst_clus_hi) << 8;
    cluster |= swap16(dir_entry->dir_fst_clus_lo);
    return (cluster);
}


/******************************************************************************
 *                                                                            *
 * FUNCTION             = fs_get_directory_entry                              *
 * DESCRIPTION          = searches a directory entry                          *
 *                                                                            *
 * IN VALUES:                                                                 *
 * dev                  = device                                              *
 * directory            = path of the searched directory entry                *
 *                                                                            *
 * OUT VALUE:                                                                 *
 * dir_entry            = the searched directory entry                        *
 *                                                                            *
 ******************************************************************************/

TOS_UInt32 fs_get_directory_entry(TOS_FAT_Device* dev, TOS_Octet* directory, FAT_DIR_ENTRY* dir_entry)
{
  TOS_UInt32 directory_cluster, current_cluster = 0, i, j, k, next_cluster, start_sector;
  TOS_UInt32 position = 1;

  TOS_Bool is_directory;

  TOS_Octet extension = 0;
  TOS_Octet directory_found = 0;
  TOS_Octet root = 1; // indicates, that the current directory is the root directory

  TOS_Octet file_name[12];
  TOS_Octet current_file_name[12];

  FAT_DIR_ENTRY dir_entries[128];

  /*
  if (dev->fat_type == FAT32_TYPE)
    current_cluster = *((int*) &BPB_FAT32.BPB_RootClus[0]);
  */

  strncpy(&file_name[0], "           ", 11);
  file_name[11] = '\0';

  if ((directory[0] != '/') || ((strcmp(directory, "/") == 0) && (dev->fat_type != FAT32_TYPE)))
    return 0;

  for (i = 1; i < strlen(directory); i++)
    {
      if (directory[i] == '.')
	{
	  extension = 1;

	  if (i - position > 8) // only directories in 8.3-format supported
	    return 0;

	  strncpy(&file_name[0], &directory[position], i - position);

	  position = i + 1;
	}

      if ((directory[i] == '/') || (i == strlen(directory) - 1))
	{
	  is_directory = FALSE;

	  if ((directory[i] == '/') && (i == strlen(directory) - 1))
	    {
	      is_directory = TRUE;

	      if ((!extension && (i - position - 1> 8)) || (extension && (i - position - 1> 3))) // only directories in 8.3-format supported
		return 0;

	      if (!extension)
		strncpy(&file_name[0], &directory[position], i - position);
	      else
		strncpy(&file_name[8], &directory[position], i - position);
	    }
	  else if (directory[i] == '/')
	    {
	      is_directory = TRUE;

	      if ((!extension && (i - position > 8)) || (extension && (i - position > 3))) // only directories in 8.3-format supported
		return 0;

	      if (!extension)
		strncpy(&file_name[0], &directory[position], i - position);
	      else
		strncpy(&file_name[8], &directory[position], i - position);
	    }
	  else if (i == strlen(directory) - 1)
	    {
	      if ((!extension && (i - position > 8)) || (extension && (i - position > 3))) // only directories in 8.3-format supported
		return 0;

	      if (!extension)
		strncpy(&file_name[0], &directory[position], i - position + 1);
	      else
		strncpy(&file_name[8], &directory[position], i - position + 1);
	    }

	  directory_found = 0;
	  extension = 0;
	  position = i + 1;

	  str2up(file_name); // converts lower case characters to upper case characters

	  if (root && (dev->fat_type != FAT32_TYPE)) // (current directory is root directory) and (FAT type is FAT12 or FAT16)
	    {
	      start_sector = dev->rsvd_sec_cnt + (dev->num_fat * dev->fat_size);

	      for (j = 0; j < dev->root_dir_secs; j++)
		{
		  fs_bio_read(dir_entries, start_sector, 1);
		  
		  for (k = 0; k < dev->bytes_per_sector / 32; k++)
		    {
		      if (dir_entries[k].dir_name[0] == 0x00)
			return 0;

		      if (dir_entries[k].dir_name[0] != 0xE5)
			{
			  strncpy(current_file_name, &dir_entries[k].dir_name[0], 11);
			  current_file_name[11] = '\0';

			  if (strcmp(current_file_name, file_name) == 0)
			    {
			      if (is_directory ^ ((dir_entries[k].dir_attr & 0x10) == 0x10))
				continue;

			      current_cluster = fs_get_cluster(&dir_entries[k]);

			      *dir_entry = dir_entries[k];

			      directory_found = 1;
			      break;
			    }
			}
		    }

		  if (directory_found)
		    break;

		  start_sector++;
		}
	    }
	  else
	    {
	      directory_cluster = current_cluster;

	      do
		{
		  next_cluster = fs_get_fat_cluster_entry(dev, directory_cluster);

		  start_sector = fs_get_first_sector_of_cluster(dev, directory_cluster);

		  for (j = 0; j < dev->sectors_per_cluster; j++)
		    {
		      fs_bio_read(dir_entries, start_sector, 1);

		      for (k = 0; k < dev->bytes_per_sector / 32; k++)
			{
			  if (dir_entries[k].dir_name[0] == 0x00)
			    return 0;

			  if (dir_entries[k].dir_name[0] != 0xE5)
			    {
			      strncpy(current_file_name, &dir_entries[k].dir_name[0], 11);
			      current_file_name[11] = '\0';

			      if (strcmp(current_file_name, file_name) == 0)
				{
				  if (is_directory ^ ((dir_entries[k].dir_attr & 0x10) == 0x10))
				    continue;

				  current_cluster = fs_get_cluster(&dir_entries[k]);

				  *dir_entry = dir_entries[k];

				  directory_found = 1;
				  break;
				}
			    }
			}

		      if (directory_found)
			break;

		      start_sector++;
		    }

		  if (directory_found)
		    break;

		  directory_cluster = next_cluster;
		}
	      while (!fs_is_eoc(dev, next_cluster));
	    }

	  root = 0;
	  strncpy(&file_name[0], "           ", 11);
	}
    }

  return 1;
}


/******************************************************************************
 *                                                                            *
 * FUNCTION             = fs_update_directory_entry                           *
 * DESCRIPTION          = updates a directory entry                           *
 *                                                                            *
 * IN VALUES:                                                                 *
 * dev                  = device                                              *
 * directory            = path of the updated directory entry                 *
 * dir_entry            = the updated directory entry                         *
 *                                                                            *
 ******************************************************************************/

TOS_UInt32 fs_update_directory_entry(TOS_FAT_Device* dev, TOS_Octet* directory, FAT_DIR_ENTRY dir_entry)
{
  TOS_UInt32 directory_cluster, current_cluster = 0, i, j, k, next_cluster, start_sector = 0;
  TOS_UInt32 position = 1;

  TOS_Bool is_directory;

  TOS_Octet extension = 0;
  TOS_Octet directory_found = 0;
  TOS_Octet root = 1; // indicates, that the current directory is the root directory

  TOS_Octet file_name[12];
  TOS_Octet current_file_name[12];

  FAT_DIR_ENTRY dir_entries[128];

  /*
  if (dev->fat_type == FAT32_TYPE)
    current_cluster = *((int*) &BPB_FAT32.BPB_RootClus[0]);
  */

  strncpy(&file_name[0], "           ", 11);
  file_name[11] = '\0';

  if ((directory[0] != '/') || ((strcmp(directory, "/") == 0) && (dev->fat_type != FAT32_TYPE)))
    return 0;

  for (i = 1; i < strlen(directory); i++)
    {
      if (directory[i] == '.')
	{
	  extension = 1;

	  if (i - position > 8) // only directories in 8.3-format supported
	    return 0;

	  strncpy(&file_name[0], &directory[position], i - position);

	  position = i + 1;
	}

      if ((directory[i] == '/') || (i == strlen(directory) - 1))
	{
	  is_directory = FALSE;

	  if ((directory[i] == '/') && (i == strlen(directory) - 1))
	    {
	      is_directory = TRUE;

	      if ((!extension && (i - position - 1> 8)) || (extension && (i - position - 1> 3))) // only directories in 8.3-format supported
		return 0;

	      if (!extension)
		strncpy(&file_name[0], &directory[position], i - position);
	      else
		strncpy(&file_name[8], &directory[position], i - position);
	    }
	  else if (directory[i] == '/')
	    {
	      is_directory = TRUE;

	      if ((!extension && (i - position > 8)) || (extension && (i - position > 3))) // only directories in 8.3-format supported
		return 0;

	      if (!extension)
		strncpy(&file_name[0], &directory[position], i - position);
	      else
		strncpy(&file_name[8], &directory[position], i - position);
	    }
	  else if (i == strlen(directory) - 1)
	    {
	      if ((!extension && (i - position > 8)) || (extension && (i - position > 3))) // only directories in 8.3-format supported
		return 0;

	      if (!extension)
		strncpy(&file_name[0], &directory[position], i - position + 1);
	      else
		strncpy(&file_name[8], &directory[position], i - position + 1);
	    }

	  directory_found = 0;
	  extension = 0;
	  position = i + 1;

	  str2up(file_name); // converts lower case characters to upper case characters

	  if (root && (dev->fat_type != FAT32_TYPE)) // (current directory is root directory) and (FAT type is FAT12 or FAT16)
	    {
	      start_sector = dev->rsvd_sec_cnt + (dev->num_fat * dev->fat_size);

	      for (j = 0; j < dev->root_dir_secs; j++)
		{
		  fs_bio_read(dir_entries, start_sector, 1);

		  for (k = 0; k < dev->bytes_per_sector / 32; k++)
		    {
		      if (dir_entries[k].dir_name[0] == 0x00)
			return 0;

		      if (dir_entries[k].dir_name[0] != 0xE5)
			{
			  strncpy(current_file_name, &dir_entries[k].dir_name[0], 11);
			  current_file_name[11] = '\0';

			  if (strcmp(current_file_name, file_name) == 0)
			    {
			      if (is_directory ^ ((dir_entries[k].dir_attr & 0x10) == 0x10))
				continue;

			      current_cluster = fs_get_cluster(&dir_entries[k]);

			      if (i == strlen(directory) - 1)
				dir_entries[k] = dir_entry;

			      directory_found = 1;
			      break;
			    }
			}
		    }

		  if (directory_found)
		    break;

		  start_sector++;
		}
	    }
	  else
	    {
	      directory_cluster = current_cluster;

	      do
		{
		  next_cluster = fs_get_fat_cluster_entry(dev, directory_cluster);

		  start_sector = fs_get_first_sector_of_cluster(dev, directory_cluster);

		  for (j = 0; j < dev->sectors_per_cluster; j++)
		    {
		      fs_bio_read(dir_entries, start_sector, 1);

		      for (k = 0; k < dev->bytes_per_sector / 32; k++)
			{
			  if (dir_entries[k].dir_name[0] == 0x00)
			    return 0;

			  if (dir_entries[k].dir_name[0] != 0xE5)
			    {
			      strncpy(current_file_name, &dir_entries[k].dir_name[0], 11);
			      current_file_name[11] = '\0';

			      if (strcmp(current_file_name, file_name) == 0)
				{
				  if (is_directory ^ ((dir_entries[k].dir_attr & 0x10) == 0x10))
				    continue;

				  current_cluster = fs_get_cluster(&dir_entries[k]);

				  if (i == strlen(directory) - 1)
				    dir_entries[k] = dir_entry;

				  directory_found = 1;
				  break;
				}
			    }
			}

		      if (directory_found)
			break;

		      start_sector++;
		    }

		  if (directory_found)
		    break;

		  directory_cluster = next_cluster;
		}
	      while (!fs_is_eoc(dev, next_cluster));
	    }

	  root = 0;
	  strncpy(&file_name[0], "           ", 11);
	}
    }

  fs_bio_write(dir_entries, start_sector, 1);

  return 1;
}

/******************************************************************************
 *                                                                            *
 * FUNCTION             = fs_create_directory_entry                           *
 * DESCRIPTION          = creates a directory entry                           *
 *                                                                            *
 * IN VALUES:                                                                 *
 * dev                  = device                                              *
 * parent_directory     = directory in which the new directory entry is       *
 *                        created                                             *
 * subdir_entry         = name of the new directory entry                     *
 *                                                                            *
 * OUT VALUE:                                                                 *
 * parent_cluster       = cluster of the directory in which the new directory *
 *                        entry is created                                    *
 *                                                                            *
 ******************************************************************************/

TOS_UInt32 fs_create_directory_entry(TOS_FAT_Device* dev, TOS_Octet* parent_directory, FAT_DIR_ENTRY* subdir_entry, TOS_UInt32* parent_cluster)
{
  unsigned char signature;

  TOS_UInt32 cluster, current_cluster, i, j, next_cluster, start_sector;

  // maximum FAT sector size = 4096 Byte -> maximum amount of 32 Byte dir entries per sector = 128

  FAT_DIR_ENTRY dir_entry;
  FAT_DIR_ENTRY parent_dir_entries[128];

  *parent_cluster = 0;

  // parent directory is root directory and FAT type is FAT12

  if ((strcmp(parent_directory, "/") == 0) && (dev->fat_type != FAT32_TYPE))
    {
      // search a free root directory entry

      start_sector = dev->rsvd_sec_cnt + (dev->num_fat * dev->fat_size);

      for (i = 0; i < dev->root_dir_secs; i++)
	{
	  fs_bio_read(parent_dir_entries, start_sector, 1);

	  for (j = 0; j < dev->bytes_per_sector / 32; j++)
	    {
	      if ((parent_dir_entries[j].dir_name[0] == 0x00) || (parent_dir_entries[j].dir_name[0] == 0xE5))
		{
		  // a free root directory entry was found

		  signature = parent_dir_entries[j].dir_name[0];

		  // create subdirectory entry

		  parent_dir_entries[j] = *subdir_entry;

		  if (signature == 0xE5)
		    {
		      fs_bio_write(&parent_dir_entries[0], start_sector, 1);
		    }
		  else // signature == 0x00
		    {
		      if (j != (dev->bytes_per_sector / 32) - 1) // not end of sector
			parent_dir_entries[j+1].dir_name[0] = 0x00; // write end-of-directory entry

		      fs_bio_write(&parent_dir_entries[0], start_sector, 1);

		      if ((j == (dev->bytes_per_sector / 32) - 1) && (i != dev->root_dir_secs - 1)) // end of sector and not last root dir sector
			{
			  // read next sector
			  fs_bio_read(&parent_dir_entries, start_sector + dev->bytes_per_sector, 1);

			  parent_dir_entries[0].dir_name[0] = 0x00; // write end-of-directory entry

			  fs_bio_write(&parent_dir_entries[0], start_sector + dev->bytes_per_sector, 1);
			}
		    }

		  return 1;
		}
	    }

	  start_sector++;
	}

      return 0; // no free root directory entry was found
    }
  else
    {
      if (!fs_get_directory_entry(dev, parent_directory, &dir_entry))
	return 0;

      current_cluster = fs_get_cluster(&dir_entry);

      //current_cluster = (dir_entry.dir_fst_clus_hi << 8) | dir_entry.dir_fst_clus_lo;

      *parent_cluster = current_cluster; // remember the first cluster of the directory

      do
	{
	  next_cluster = fs_get_fat_cluster_entry(dev, current_cluster);

	  start_sector = fs_get_first_sector_of_cluster(dev, current_cluster);

	  for (i = 0; i < dev->sectors_per_cluster; i++)
	    {
	      fs_bio_read(parent_dir_entries, start_sector, 1);

	      for (j = 0; j < dev->bytes_per_sector / 32; j++)
		{
		  if ((parent_dir_entries[j].dir_name[0] == 0x00) || (parent_dir_entries[j].dir_name[0] == 0xE5))
		    {
		      // a free directory entry was found

		      signature = parent_dir_entries[j].dir_name[0];

		      // create subdirectory entry

		      parent_dir_entries[j] = *subdir_entry;

		      if (signature == 0xE5)
			{
			  fs_bio_write(&parent_dir_entries[0], start_sector, 1);
			}
		      else // signature == 0x00
			{
			  if (j != (dev->bytes_per_sector / 32) - 1) // not end of sector
			    parent_dir_entries[j+1].dir_name[0] = 0x00; // write end-of-directory entry

			  fs_bio_write(&parent_dir_entries[0], start_sector, 1);

			  if ((j == (dev->bytes_per_sector / 32) - 1) && (i != dev->root_dir_secs - 1)) // end of sector and not last root dir sector
			    {
			      // read next sector
			      fs_bio_read(&parent_dir_entries, start_sector + dev->bytes_per_sector, 1);

			      parent_dir_entries[0].dir_name[0] = 0x00; // write end-of-directory entry

			      fs_bio_write(&parent_dir_entries[0], start_sector + dev->bytes_per_sector, 1);
			    }
			}

		      return 1;
		    }
		}
	    }

	  start_sector++;

	  if (!fs_is_eoc(dev, next_cluster))
	    current_cluster = next_cluster;
	}
      while (!fs_is_eoc(dev, next_cluster));

      // no free directory entry was found
      // search a free cluster

      if (!fs_get_free_cluster(dev, &cluster))
	return 0; // no free cluster found

      // create subdirectory entry

      parent_dir_entries[0] = *subdir_entry;
      parent_dir_entries[1].dir_name[0] = 0x00; // write end-of-directory entry

      fs_bio_write(parent_dir_entries, fs_get_first_sector_of_cluster(dev, cluster), 1);

      switch (dev->fat_type)
	{
	case FAT12_TYPE : fs_set_fat_cluster_entry(dev, cluster, 0x0FFF); break;
	case FAT16_TYPE : fs_set_fat_cluster_entry(dev, cluster, 0xFFFF); break;
	case FAT32_TYPE : fs_set_fat_cluster_entry(dev, cluster, 0x0FFFFFFF);
	}

      fs_set_fat_cluster_entry(dev, current_cluster, cluster);

      return 1;
    }
}

/******************************************************************************
 *                                                                            *
 * FUNCTION             = fs_create_directory                                 *
 * DESCRIPTION          = creates a directory                                 *
 *                                                                            *
 * IN VALUES:                                                                 *
 * dev                  = device                                              *
 * parent_directory     = directory in which the new directory is created     *
 * child_directory      = name of the new directory                           *
 *                                                                            *
 ******************************************************************************/

TOS_UInt32 fs_create_directory(TOS_FAT_Device* dev, TOS_Octet* parent_directory, TOS_Octet* child_directory)
{
  unsigned char extension = 0;
  unsigned char position = 0;
  unsigned char name[12];

  TOS_UInt32 cluster, i, parent_cluster;

  // maximum FAT sector size = 4096 Byte -> maximum amount of 32 Byte dir entries per sector = 128

  FAT_DIR_ENTRY subdir_entry;
  FAT_DIR_ENTRY dir_entries[128];
  //  FAT_DIR_ENTRY parent_dir_entries[128];

  strncpy(&name[0], "           ", 11);
  name[11] = '\0';

  if (parent_directory[0] != '/')
    return 0;

  for (i = 0; i < strlen(child_directory); i++)
    {
      if (child_directory[i] == '.')
	{
	  extension = 1;

	  if (i > 7) // only directories in 8.3-format supported
	    return 0;

	  strncpy(&name[0], child_directory, i);

	  position = i + 1;
	}
    }

  if (extension)
    {
      if (i - position > 3) // only directories in 8.3-format supported
	return 0;

      strncpy(&name[8], &child_directory[position], i - position);
    }
  else
    {
      strncpy(name, child_directory, i);
    }

  // search a free cluster

  if (!fs_get_free_cluster(dev, &cluster))
    return 0; // no free cluster found

  // allocate the cluster

  switch (dev->fat_type)
    {
    case FAT12_TYPE : fs_set_fat_cluster_entry(dev, cluster, 0x0FFF); break;
    case FAT16_TYPE : fs_set_fat_cluster_entry(dev, cluster, 0xFFFF); break;
    case FAT32_TYPE : fs_set_fat_cluster_entry(dev, cluster, 0x0FFFFFFF);
    }

  // create subdirectory entry

  strncpy(&subdir_entry.dir_name[0], "           ", 11);
  strncpy(&subdir_entry.dir_name[0], name, strlen(name));

  subdir_entry.dir_fst_clus_lo = swap16(cluster);
  subdir_entry.dir_fst_clus_hi = swap16(cluster >> 16); // always 0 for a FAT12 or FAT16 volume
  subdir_entry.dir_attr = 0x10;
  subdir_entry.dir_nt_res = 0x00;
  subdir_entry.dir_crt_time_tenth = 0x00;
  subdir_entry.dir_crt_time = swap16(0x00);
  subdir_entry.dir_crt_date = swap16(0x00);
  subdir_entry.dir_lst_acc_date = swap16(0x00);
  subdir_entry.dir_wrt_time = swap16(0x00);
  subdir_entry.dir_wrt_date = swap16(0x00);
  subdir_entry.dir_file_size = swap32(0x00);

  if (!fs_create_directory_entry(dev, parent_directory, &subdir_entry, &parent_cluster))
    {
      // deallocate the cluster

      fs_set_fat_cluster_entry(dev, cluster, 0x0);

      return 0;
    }

  // create three new directory entries in this cluster:
  // ".", "..", and the entry which indicates the end of the directory

  strncpy(&dir_entries[0].dir_name[0], ".          ", 11);
  strncpy(&dir_entries[1].dir_name[0], "..         ", 11);
  dir_entries[2].dir_name[0] = 0x00;

  dir_entries[0].dir_fst_clus_lo = swap16(cluster);
  dir_entries[1].dir_fst_clus_lo = swap16(parent_cluster);

  // always 0 for a FAT12 or FAT16 volume
  dir_entries[0].dir_fst_clus_hi = swap16(cluster >> 16);
  dir_entries[1].dir_fst_clus_hi = swap16(parent_cluster >> 16);

  for (i = 0; i < 2; i++)
    {
      dir_entries[i].dir_attr = 0x10;
      dir_entries[i].dir_nt_res = 0x00;
      dir_entries[i].dir_crt_time_tenth = 0x00;
      dir_entries[i].dir_crt_time = swap16(0x00);
      dir_entries[i].dir_crt_date = swap16(0x00);
      dir_entries[i].dir_lst_acc_date = swap16(0x00);
      dir_entries[i].dir_wrt_time = swap16(0x00);
      dir_entries[i].dir_wrt_date = swap16(0x00);
      dir_entries[i].dir_file_size = swap32(0x00);
    }

  fs_bio_write(&dir_entries[0], fs_get_first_sector_of_cluster(dev, cluster), 1);

  switch (dev->fat_type)
    {
    case FAT12_TYPE : fs_set_fat_cluster_entry(dev, cluster, 0x0FFF); break;
    case FAT16_TYPE : fs_set_fat_cluster_entry(dev, cluster, 0xFFFF); break;
    case FAT32_TYPE : fs_set_fat_cluster_entry(dev, cluster, 0x0FFFFFFF);
    }

  return 1;
}



/*
 * Computes the next cluster in the chain of clusters of a given
 * file.
 */
TOS_UInt32
TOS_fs_next_cluster_in_chain (TOS_FAT_Device* dev, TOS_UInt32 cluster)
{
    TOS_UInt32 fat_offset;
    TOS_UInt32 next_cluster = 0;
    
    switch (dev->fat_type) {
    case FAT12_TYPE:
	fat_offset = cluster + (cluster / 2);
	next_cluster = swap16(*((TOS_UInt16*) &(dev->fat [fat_offset])));
	if (cluster & 1)
	    next_cluster = next_cluster >> 4;
	else
	    next_cluster = next_cluster & 0xfff;
	break;
    case FAT16_TYPE:
	fat_offset = cluster * 2;
	next_cluster = swap16(*((TOS_UInt16*) &(dev->fat [fat_offset])));
	break;
    case FAT32_TYPE:
	fat_offset = cluster * 4;
	next_cluster = swap32(*((TOS_UInt32*) &(dev->fat [fat_offset])));
	break;
    }
    
    return next_cluster;
}


/*
 * Returns the first sector number that belongs to cluster 'cluster'
 */
TOS_UInt32
TOS_fs_cluster_to_sector (TOS_FAT_Device* dev, TOS_UInt32 cluster)
{
    return ((cluster - 2) * dev->sectors_per_cluster) + dev->first_data_sector;
}


TOS_Bool
TOS_fs_is_last_cluster (TOS_FAT_Device* dev, TOS_UInt32 cluster)
{
    TOS_Bool is_eof = FALSE;
    
    switch (dev->fat_type) {
    case FAT12_TYPE:
	if (cluster >= 0x0ff8)
	    is_eof = TRUE;
	break;
    case FAT16_TYPE:
	if (cluster >= 0xfff8)
	    is_eof = TRUE;
	break;
    case FAT32_TYPE:
	if (cluster >= 0x0ffffff8)
	    is_eof = TRUE;
	break;
    }
    
    return is_eof;
}


void
TOS_fs_format_dir_entry_name (DIR_Name entry,
			      const char* name)
{
    char ch;
    int i;
    
    for (i = 0; i < 11; i++)
	entry[i] = ' ';
    
    i = 0;
    
    while ((ch = *name++) != '\0') {
	if (ch == '.') {
	    i = 8;
	    continue;
	}
	
	// Convert to upper case
	if (ch >= 'a' && ch <= 'z')
	    ch -= 'a' - 'A';
	
	entry[i++] = ch;
    }
}


// File descriptor

static FCB_Entry fcb[NUM_FCB];

#define FCB_ENTRY(i) ((i) & 0xffff)

#define IS_VALID_FCB_ENTRY(i) ((FCB_ENTRY(i) < NUM_FCB) && \
                               (fcb[FCB_ENTRY(i)].used) && \
			       (fcb[FCB_ENTRY(i)].seq == ((i) >> 16)))


void
TOS_init_fcb ()
{
    int i;
    
    for (i = 0; i < NUM_FCB; i++) {
	fcb[i].used = FALSE;
	fcb[i].seq = 0;
    }
}


TOS_Error
TOS_fs_find_dir_entry (TOS_FAT_Device* dev,
		       const char* fname,
		       FAT_DIR_ENTRY* entry)
{
    DIR_Name name;
    Sector sector;
    int num_dir_entries;
    int i, j, k;
    FAT_DIR_ENTRY* cur_dir_entry = NULL;
    
    TOS_fs_format_dir_entry_name (name, fname);
    
    num_dir_entries = dev->root_dir_secs * dev->bytes_per_sector / 32;
    
    for (i = 0, j = 0; i < num_dir_entries; i++) {
	if (((i * 32) % dev->bytes_per_sector) == 0) {
	    fs_bio_read (&sector,
			  j + dev->first_root_dir_sector,
			  1);
	    j++;
	    cur_dir_entry = (FAT_DIR_ENTRY*) &sector;
	}
	if (cur_dir_entry->dir_name[0] == DIR_ENTRY_LAST_EMPTY)
	    break;
	if (cur_dir_entry->dir_name[0] != DIR_ENTRY_EMPTY) {
	    for (k = 0; k < 11; k++) {
		if (cur_dir_entry->dir_name[k] != name[k])
		    break;
	    }
	    
	    if (k == 11) {
		// We found the entry
		for (k = 0; k < sizeof (FAT_DIR_ENTRY); k++)
		    ((TOS_Octet*) entry)[k] = ((TOS_Octet*) cur_dir_entry)[k];
		return TOS_NO_ERROR;
	    }
	}
	cur_dir_entry += 1;
    }

    return TOS_ERR_FILE_NOT_FOUND;
}


TOS_Error
TOS_fs_open (TOS_FAT_Device* dev,
	     const char* file_name,
	     TOS_FS_OPEN_MODE mode)
{
    TOS_Error err;
    FAT_DIR_ENTRY entry;
    int i;

    //assert (mode == TOS_FS_OPEN_MODE_READ); // Jens
    
    err = TOS_fs_find_dir_entry (dev, file_name, &entry);
    if (err < 0)
	return err;
    
    // Found the file. Now initialize a FCB entry
    for (i = 0; i < NUM_FCB; i++)
	if (!fcb[i].used)
	    break;
    
    if (i == NUM_FCB)
	// FCB is full
	return TOS_ERR_FCB_FULL;

    // Initialize FCB
    fcb[i].used = TRUE;
    fcb[i].file_size = entry.dir_file_size;
    fcb[i].dev = dev;
    fcb[i].pos = 0;
    fcb[i].first_cluster = entry.dir_fst_clus_lo | (entry.dir_fst_clus_hi << 16);
    fcb[i].current_cluster = fcb[i].first_cluster;

    // Compute FD
    return i | (fcb[i].seq << 16);
}

TOS_Error
TOS_fs_close (FAT_FD fd)
{
    if (!IS_VALID_FCB_ENTRY (fd))
	return TOS_ERR_BAD_FILE_DESCR;

    // Mark FCB entry as unused
    fcb[FCB_ENTRY(fd)].used = FALSE;
    fcb[FCB_ENTRY(fd)].seq++;

    return TOS_NO_ERROR;
}

TOS_Error
TOS_fs_is_eof (FAT_FD fd)
{
    int i;
    
    if (!IS_VALID_FCB_ENTRY (fd))
	return TOS_ERR_BAD_FILE_DESCR;

    i = FCB_ENTRY (fd);
    return fcb[i].pos == fcb[i].file_size;
}

#ifndef FS_STANDALONE
TOS_Error
TOS_fs_chown (FAT_FD fd, TOS_UInt32 pid)
{
    if (!IS_VALID_FCB_ENTRY (fd))
	return TOS_ERR_BAD_FILE_DESCR;

    fcb[FCB_ENTRY (fd)].owner = pid;
    return TOS_NO_ERROR;
}
#endif


TOS_Error
TOS_fs_read (FAT_FD fd,
	     void* buf,
	     TOS_UInt32 len)
{
    int i, k;
#ifdef FS_STANDALONE
    TOS_Octet* mem = (TOS_Octet*) buf;
#else
    TOS_UInt32 mem = (TOS_UInt32) buf;
#endif
    TOS_UInt32 bytes_per_cluster;
    TOS_UInt32 cluster_to_load, cluster;
    TOS_UInt32 sector_to_load;
    TOS_UInt32 rel_sector;
    
    if (!IS_VALID_FCB_ENTRY (fd))
	return TOS_ERR_BAD_FILE_DESCR;

    i = FCB_ENTRY(fd);
    
    while (len-- != 0) {
	
	// Make sure we don't read beyond EOF
	if (fcb[i].pos == fcb[i].file_size)
	    return TOS_ERR_BEYOND_EOF;
	
	if (fcb[i].pos % fcb[i].dev->bytes_per_sector == 0) {
	    
	    // We need to load a new sector
	    // First compute which cluster the sector is in
	    bytes_per_cluster = fcb[i].dev->bytes_per_sector *
		fcb[i].dev->sectors_per_cluster;
	    
	    cluster = fcb[i].pos / bytes_per_cluster;
	    
	    // Find this cluster
	    cluster_to_load = fcb[i].first_cluster;
	    
	    for (k = 0; k < cluster; k++) {
		cluster_to_load = TOS_fs_next_cluster_in_chain (fcb[i].dev,
								cluster_to_load);
		assert (!TOS_fs_is_last_cluster (fcb[i].dev, cluster_to_load));
	    }
	    
	    sector_to_load = TOS_fs_cluster_to_sector (fcb[i].dev,
						       cluster_to_load);

	    // Adjust sector with the cluster
	    rel_sector = fcb[i].pos % bytes_per_cluster;
	    sector_to_load += rel_sector / fcb[i].dev->bytes_per_sector;
	    fs_bio_read (&fcb[i].sector,
			  sector_to_load,
			  1);
	}
#ifdef FS_STANDALONE
	*mem++ = fcb[i].sector[fcb[i].pos++ % fcb[i].dev->bytes_per_sector];
#else
	poke_mem_b (mem++, fcb[i].sector[fcb[i].pos++ % fcb[i].dev->bytes_per_sector]);
#endif
    }
    return TOS_NO_ERROR;
}

TOS_Error
TOS_fs_seek (FAT_FD fd,
	     TOS_UInt32 pos)
{
    int i, k;
    TOS_UInt32 bytes_per_cluster;
    TOS_UInt32 cluster_to_load, cluster;
    TOS_UInt32 sector_to_load;
    TOS_UInt32 rel_sector;
    
    if (!IS_VALID_FCB_ENTRY (fd))
	return TOS_ERR_BAD_FILE_DESCR;

    if (!IS_VALID_FCB_ENTRY (fd))
	return TOS_ERR_BAD_FILE_DESCR;

    i = FCB_ENTRY(fd);
    
    // Make sure we don't seek beyond EOF
    if (pos >= fcb[i].file_size)
	return TOS_ERR_BEYOND_EOF;
	
    fcb[i].pos = pos;
    
    // We need to load a new sector
    // First compute which cluster the sector is in
    bytes_per_cluster = fcb[i].dev->bytes_per_sector *
	fcb[i].dev->sectors_per_cluster;
	
    cluster = fcb[i].pos / bytes_per_cluster;
	
    // Find this cluster
    cluster_to_load = fcb[i].first_cluster;
	
    for (k = 0; k < cluster; k++) {
	cluster_to_load = TOS_fs_next_cluster_in_chain (fcb[i].dev,
							cluster_to_load);
	assert (!TOS_fs_is_last_cluster (fcb[i].dev, cluster_to_load));
    }
	
    sector_to_load = TOS_fs_cluster_to_sector (fcb[i].dev,
					       cluster_to_load);
	
    // Adjust sector with the cluster
    rel_sector = fcb[i].pos % bytes_per_cluster;
    sector_to_load += rel_sector / fcb[i].dev->bytes_per_sector;
    fs_bio_read (&fcb[i].sector, sector_to_load, 1);

    return TOS_NO_ERROR;
}

#ifndef FS_STANDALONE
TOS_Error
TOS_fs_process_exited (TOS_UInt32 pid)
{
    int i;
    
    // Search owner
    for (i = 0; i < NUM_FCB; i++) {
	if (fcb[i].used && fcb[i].owner == pid)
	    break;
    }
    
    if (i != NUM_FCB) {
	// Mark FCB entry as unused
	fcb[i].used = FALSE;
	fcb[i].seq++;
    }
    
    return TOS_NO_ERROR;
}
#endif



#ifndef FS_STANDALONE
/* -- File-server task -- */

typedef union 
{
    StandardMsg std;
    FileOpenMsg open;
    FileCloseMsg close;
    FileReadMsg read;
    FileSeekMsg seek;
    ChownHandleMsg chown;
    ProcessExitedMsg exited;
} FSMsg;

typedef union 
{
    StandardReply std;
    FileOpenReply open;
    FileCloseReply close;
    FileReadReply read;
    FileSeekReply seek;
    ChownHandleReply chown;
    ProcessExitedReply exited;
} FSReply;


static TOS_FAT_Device the_device;

void file_server( PORT_ID first_port, PORT_ID parent_port )
{
    PID		pid;
    PORT_ID	device_port;
    int		data_len;
    FSMsg       *msg;
    FSReply     *reply;
    FAT_FD      fd;
    TOS_Error   err;
    
    file_copy_buffer	= k_sbrk( RECEIVE_BUFFER_SIZE );
    driver_copy_buffer	= k_sbrk( RECEIVE_BUFFER_SIZE );

    if( create_port( &device_port ) )
	shutdown( "file_server(): Couldn't create device port" );
    if( create( floppy_task, PRIO_FLOPPY_SERVER, "Floppy server",
		&pid, &floppy_port, device_port ) )
	shutdown( "file_server(): Couldn't create floppy server" );


    msg = reply = HERMES_DATA_PTR( file_copy_buffer );

    check_sizeof_types();
    fs_init(&the_device);

    while( TRUE ) {
	k_receive( file_copy_buffer, &pid, &data_len );
	switch( msg->std.Type ) {
	case FILE_OPEN:
	    fd = TOS_fs_open (&the_device,
			      msg->open.Path,
			      msg->open.AccessMode);
	    reply->open.Size = sizeof (FileOpenReply);
	    if (fd < 0) {
		reply->open.Result = fd;
	    } else {
		reply->open.Result = TOS_NO_ERROR;
		reply->open.Handle = fd;
	    }
	    break;
	case FILE_CLOSE:
	    assert (msg->std.Size = sizeof (FileCloseMsg));
	    err = TOS_fs_close (msg->close.Handle);
	    reply->close.Size = sizeof (FileCloseReply);
	    reply->close.Result = err;
	    break;
	case FILE_READ:
	    err = TOS_fs_read (msg->read.Handle,
			       msg->read.Destination,
			       msg->read.NumberOfBytes);
	    reply->read.Size = sizeof (FileReadReply);
	    reply->read.Result = err;
	    reply->read.NumOfBytes = msg->read.NumberOfBytes;
	    break;
	case CHOWN_HANDLE:
	    err = TOS_fs_chown (msg->chown.Handle,
				msg->chown.newOwnerPid);
	    reply->chown.Size = sizeof (ChownHandleMsg);
	    reply->chown.Result = err;
	    break;
	case PROCESS_EXITED:
	    err = TOS_fs_process_exited (msg->exited.pid);
	    reply->exited.Size = sizeof (ProcessExitedReply);
	    reply->exited.Result = err;
	    break;
	case FILE_SEEK:
	    err = TOS_fs_seek (msg->seek.Handle,
			       msg->seek.NumOfBytes);
	    reply->seek.Size = sizeof (FileSeekReply);
	    reply->seek.Result = err;
	    break;
	default:
	  //printf ("Req = %d\n", msg->std.Type);
	  //printf ("PID = %x\n", pid);
	  //printf ("Name = %s\n", pcb[pid&0xff].name);
	    shutdown( "file_server(): Bad req" );
	}

	k_reply (pid, file_copy_buffer, reply->std.Size);

    }
    shutdown( "file_server(): Bad!!!" );
}

#endif // FS_STANDALONE










TOS_Error fs_convert_path(TOS_Octet* path, DIR_ENTRY_INFO* dir_entry_info)
{
  int i, extension = 0, position = 0;

  for (i = strlen(path) - 1; i >= 0; i--)
    if (path[i] == '/')
      break;

  if (i <= 254)
    strncpy(dir_entry_info->directory, path, i + 1);
  else
    return -1;

  dir_entry_info->directory[i + 1] = '\0';

  if (strlen(path) - i - 1 <= 254)
    strncpy(dir_entry_info->dir_entry, &path[i + 1], strlen(path) - i);
  else
    return -1;

  str2up(dir_entry_info->directory);
  str2up(dir_entry_info->dir_entry);

  strncpy(dir_entry_info->fat_entry, "           ", 11);
  dir_entry_info->fat_entry[11] = '\0';

  for (i = 0; i < strlen(dir_entry_info->dir_entry); i++)
    {
      if (dir_entry_info->dir_entry[i] == '.')
	{
	  extension = 1;

	  if (i > 8) // only directories in 8.3-format supported
	    return -1;

	  strncpy(dir_entry_info->fat_entry, dir_entry_info->dir_entry, i);

	  position = i + 1;
	}
    }

  if (extension)
    {
      if (i - position > 3) // only directories in 8.3-format supported
	return -1;

      strncpy(&dir_entry_info->fat_entry[8], &dir_entry_info->dir_entry[position], i - position);
    }
  else
    {
      if (i > 7) // only directories in 8.3-format supported
	return -1;

      strncpy(dir_entry_info->fat_entry, dir_entry_info->dir_entry, i);
    }

  return TOS_NO_ERROR;

  //printf("%s\n", dir_entry_info->directory);
  //printf("%s\n", dir_entry_info->dir_entry);
  //printf("%s\n", dir_entry_info->fat_entry);
}

TOS_Error fs_get_file_size (FAT_FD fd, TOS_UInt32* size)
{
  if (!IS_VALID_FCB_ENTRY (fd))
    return TOS_ERR_BAD_FILE_DESCR;

  *size = fcb[FCB_ENTRY(fd)].file_size;

  return TOS_NO_ERROR;
}

TOS_Error fs_open (TOS_FAT_Device* dev, TOS_Octet* path, TOS_FS_OPEN_MODE mode)
{
  DIR_ENTRY_INFO dir_entry_info;
  FAT_DIR_ENTRY dir_entry;

  TOS_UInt32 cluster, parent_cluster;

  int fcb_index;

  for (fcb_index = 0; fcb_index < NUM_FCB; fcb_index++)
    if (!fcb[fcb_index].used)
      break;

  if (fcb_index == NUM_FCB) // no more free FCB
    return TOS_ERR_FCB_FULL;

  if (mode == TOS_FS_OPEN_MODE_READ)
    {
      /*
	error = fs_get_directory_entry (dev, file_name, &dir_entry);
	
	if (error < 0)
	return error;
      */

      if (!fs_get_directory_entry (dev, path, &dir_entry))
	return -1;

      // Initialize FCB
      fcb[fcb_index].used = TRUE;
      fcb[fcb_index].file_size = swap32(dir_entry.dir_file_size);
      fcb[fcb_index].dev = dev;
      fcb[fcb_index].pos = 0;
      fcb[fcb_index].first_cluster = fs_get_cluster(&dir_entry);
      fcb[fcb_index].current_cluster = fcb[fcb_index].first_cluster;
      fcb[fcb_index].mode = mode;
    }
  else
    if (mode == TOS_FS_OPEN_MODE_WRITE)
      {
	if (fs_get_directory_entry (dev, path, &dir_entry))
	  {
	    // todo: append
	    return -5;
	  }
	else
	  {
	    if (fs_convert_path(path, &dir_entry_info) < 0)
	      return -5;

	    // search a free cluster

	    if (!fs_get_free_cluster(dev, &cluster))
	      return 0; // no free cluster found

	    // allocate the cluster

	    switch (dev->fat_type)
	      {
	      case FAT12_TYPE : fs_set_fat_cluster_entry(dev, cluster, 0x0FFF); break;
	      case FAT16_TYPE : fs_set_fat_cluster_entry(dev, cluster, 0xFFFF); break;
	      case FAT32_TYPE : fs_set_fat_cluster_entry(dev, cluster, 0x0FFFFFFF);
	      }

	    // create subdirectory entry

	    strncpy(dir_entry.dir_name, dir_entry_info.fat_entry, 11);

	    dir_entry.dir_fst_clus_lo = swap16(cluster);
	    dir_entry.dir_fst_clus_hi = swap16(cluster >> 16); // always 0 for a FAT12 or FAT16 volume
	    dir_entry.dir_attr = 0x00;
	    dir_entry.dir_nt_res = 0x00;
	    dir_entry.dir_crt_time_tenth = 0x00;
	    dir_entry.dir_crt_time = swap16(0x00);
	    dir_entry.dir_crt_date = swap16(0x00);
	    dir_entry.dir_lst_acc_date = swap16(0x00);
	    dir_entry.dir_wrt_time = swap16(0x00);
	    dir_entry.dir_wrt_date = swap16(0x00);
	    dir_entry.dir_file_size = swap32(0x00);

	    if (!fs_create_directory_entry(dev, dir_entry_info.directory, &dir_entry, &parent_cluster))
	      {
		// deallocate the cluster

		fs_set_fat_cluster_entry(dev, cluster, 0x0);

		return -5;
	      }

	    // Initialize FCB
	    fcb[fcb_index].used = TRUE;
	    fcb[fcb_index].file_size = 0;
	    fcb[fcb_index].dev = dev;
	    fcb[fcb_index].pos = 0;
	    fcb[fcb_index].first_cluster = cluster;
	    fcb[fcb_index].current_cluster = fcb[fcb_index].first_cluster;
	    fcb[fcb_index].mode = mode;

	    strncpy(fcb[fcb_index].path, path, 256); // workaround
	  }
      }

    // Compute FD
    return fcb_index | (fcb[fcb_index].seq << 16);
}

TOS_Error fs_close (FAT_FD fd)
{
  int i;

  TOS_UInt32 bytes_per_cluster, rel_sector, sector_to_write;

  FAT_DIR_ENTRY dir_entry;

  if (!IS_VALID_FCB_ENTRY (fd))
    return TOS_ERR_BAD_FILE_DESCR;

  i = FCB_ENTRY(fd);

  bytes_per_cluster = fcb[i].dev->bytes_per_sector * fcb[i].dev->sectors_per_cluster;

  if (fcb[i].mode == TOS_FS_OPEN_MODE_WRITE) // flush
    {
      //printf("fs_close : writing sector...\n");

      sector_to_write = fs_get_first_sector_of_cluster (fcb[i].dev, fcb[i].current_cluster);

      // Adjust sector with cluster
      rel_sector = fcb[i].pos % bytes_per_cluster;
      sector_to_write += rel_sector / fcb[i].dev->bytes_per_sector;
      fs_bio_write (&fcb[i].sector, sector_to_write, 1);

      fs_get_directory_entry(fcb[i].dev, fcb[i].path, &dir_entry);

      dir_entry.dir_file_size = swap32(fcb[i].file_size);
      fs_update_directory_entry(fcb[i].dev, fcb[i].path, dir_entry);
    }

  // Mark FCB entry as unused
  fcb[FCB_ENTRY(fd)].used = FALSE;
  fcb[FCB_ENTRY(fd)].seq++;

  return TOS_NO_ERROR;
}

TOS_Error fs_read (FAT_FD fd, void* buf, TOS_UInt32 len)
{
  int i, k;

#ifdef FS_STANDALONE
  TOS_Octet* mem = (TOS_Octet*) buf;
#else
  TOS_UInt32 mem = (TOS_UInt32) buf;
#endif

  TOS_UInt32 bytes_per_cluster;
  TOS_UInt32 cluster_to_load, cluster;
  TOS_UInt32 sector_to_load;
  TOS_UInt32 rel_sector;

  if (!IS_VALID_FCB_ENTRY (fd))
    return TOS_ERR_BAD_FILE_DESCR;

  i = FCB_ENTRY(fd);
    
  while (len-- != 0)
    {
      // Make sure we don't read beyond EOF
      if (fcb[i].pos == fcb[i].file_size)
	return TOS_ERR_BEYOND_EOF;
	
      if (fcb[i].pos % fcb[i].dev->bytes_per_sector == 0)
	{
	  // We need to load a new sector
	  // First compute which cluster the sector is in
	  bytes_per_cluster = fcb[i].dev->bytes_per_sector * fcb[i].dev->sectors_per_cluster;
	    
	  cluster = fcb[i].pos / bytes_per_cluster;
	    
	  // Find this cluster
	  cluster_to_load = fcb[i].first_cluster;
	    
	  for (k = 0; k < cluster; k++)
	    {
	      cluster_to_load = fs_get_fat_cluster_entry (fcb[i].dev, cluster_to_load);
	      assert (!TOS_fs_is_last_cluster (fcb[i].dev, cluster_to_load));
	    }
	    
	  sector_to_load = fs_get_first_sector_of_cluster (fcb[i].dev, cluster_to_load);

	  // Adjust sector with the cluster
	  rel_sector = fcb[i].pos % bytes_per_cluster;
	  sector_to_load += rel_sector / fcb[i].dev->bytes_per_sector;
	  fs_bio_read (&fcb[i].sector, sector_to_load, 1);
	}

#ifdef FS_STANDALONE
      *mem++ = fcb[i].sector[fcb[i].pos++ % fcb[i].dev->bytes_per_sector];
#else
      poke_mem_b (mem++, fcb[i].sector[fcb[i].pos++ % fcb[i].dev->bytes_per_sector]);
#endif
    }

    return TOS_NO_ERROR;
}

TOS_Error fs_write (FAT_FD fd, void* buf, TOS_UInt32 len)
{
  int i;

  TOS_Octet* mem = (TOS_Octet*) buf;

  TOS_UInt32 bytes_per_cluster, cluster, rel_sector, sector_to_write;

  if (!IS_VALID_FCB_ENTRY (fd))
    return TOS_ERR_BAD_FILE_DESCR;

  i = FCB_ENTRY(fd);

  bytes_per_cluster = fcb[i].dev->bytes_per_sector * fcb[i].dev->sectors_per_cluster;

  while (len-- != 0)
    {
      if ((fcb[i].pos != 0) && (fcb[i].pos % fcb[i].dev->bytes_per_sector == 0))
	{
	  //printf("fs_write : writing sector...\n");

	  sector_to_write = fs_get_first_sector_of_cluster (fcb[i].dev, fcb[i].current_cluster);

	  // Adjust sector with cluster
	  rel_sector = fcb[i].pos % bytes_per_cluster;
	  sector_to_write += rel_sector / fcb[i].dev->bytes_per_sector;
	  fs_bio_write (&fcb[i].sector, sector_to_write, 1);

	  //printf("fs_write : new sector needed!\n");

	  if ((fcb[i].pos != 0) && ((fcb[i].pos % bytes_per_cluster) == 0))
	    {
	      //printf("fs_write : new cluster needed!\n");

	      // search a free cluster

	      if (!fs_get_free_cluster(fcb[i].dev, &cluster))
		return -1; // no free cluster found

	      // allocate the cluster

	      switch (fcb[i].dev->fat_type)
		{
		case FAT12_TYPE : fs_set_fat_cluster_entry(fcb[i].dev, cluster, 0x0FFF); break;
		case FAT16_TYPE : fs_set_fat_cluster_entry(fcb[i].dev, cluster, 0xFFFF); break;
		case FAT32_TYPE : fs_set_fat_cluster_entry(fcb[i].dev, cluster, 0x0FFFFFFF);
		}

	      fs_set_fat_cluster_entry(fcb[i].dev, fcb[i].current_cluster, cluster);

	      fcb[i].current_cluster = cluster;
	    }
	}

      fcb[i].sector[fcb[i].pos++ % fcb[i].dev->bytes_per_sector] = *mem++;
      fcb[i].file_size++;
    }

  return 0;
}
