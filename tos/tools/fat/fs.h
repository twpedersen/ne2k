
#ifndef __TOS_FAT_H__
#define __TOS_FAT_H__

//----FS message interface--------------------------------------------

#ifdef FS_STANDALONE
#include <stdio.h>
#endif

#include <ci_types.h>
#include <stdint.h>

#define FILE_OPEN	1
#define FILE_CLOSE      2
#define FILE_READ	3
#define FILE_SEEK	4
#define CHOWN_HANDLE	5
#define PROCESS_EXITED  6

typedef struct {
    u_int Size;
    u_int Type;
} StandardMsg;

typedef struct {
    u_int Size;
    u_int Type;
    u_int AccessMode;
    char  Path[256];
} FileOpenMsg;

typedef struct {
    u_int Size;
    u_int Type;
    u_int Handle;
} FileCloseMsg;

typedef struct {
    u_int Size;
    u_int Type;
    u_int Handle;
    u_int NumberOfBytes;
    void* Destination;
} FileReadMsg;

typedef struct {
    u_int Size;
    u_int Type;
    u_int Handle;
    u_int NumOfBytes;
} FileSeekMsg;

typedef struct {
    u_int Size;
    u_int Type;
    u_int Handle;
    u_int newOwnerPid;
} ChownHandleMsg;

typedef struct {
    u_int Size;
   u_int Type;
    u_int pid;
} ProcessExitedMsg;

/* -- The replies from file-server to client -- */

typedef struct {
    u_int Size;		/* Same in all reply-types */
    u_int Result;
} StandardReply;

typedef struct {
    u_int Size;
    u_int Result;
    u_int Handle;
} FileOpenReply;

typedef struct {
    u_int Size;
    u_int Result;
} FileCloseReply;

typedef struct {
    u_int Size;
    u_int Result;
    u_int NumOfBytes;
} FileReadReply;

typedef struct {
    u_int Size;
    u_int Result;
    u_int FilePosition;
} FileSeekReply;

typedef struct {
    u_int Size;
    u_int Result;
} ChownHandleReply;

typedef struct {
    u_int Size;
    u_int Result;
} ProcessExitedReply;


//---------------------------------------------------------------------

#define TOS_CHECK(expr) assert(expr)

#define PACKED(lab) lab __attribute__ ((packed))

typedef char           TOS_Bool;
typedef unsigned char  TOS_Octet;
typedef unsigned short TOS_UInt16;
typedef short          TOS_Int16;
typedef uint32_t       TOS_UInt32;
typedef int32_t        TOS_Int32;

#define TRUE 1
#define FALSE 0

typedef struct
{
    TOS_Octet  PACKED(bs_drv_num);
    TOS_Octet  PACKED(bs_reserved1);
    TOS_Octet  PACKED(bs_boot_sig);
    TOS_UInt32 PACKED(bs_vol_id);
    TOS_Octet  PACKED(bs_vol_lab[11]);
    TOS_Octet  PACKED(bs_fil_sys_type[8]);
} BPB_FAT16;

typedef struct
{
    TOS_UInt32  PACKED(bpb_fat_sz32);
    TOS_UInt16  PACKED(bpb_ext_flags);
    TOS_UInt16  PACKED(bpb_fs_ver);
    TOS_UInt32  PACKED(bpb_root_clus);
    TOS_UInt16  PACKED(bpb_fs_info);
    TOS_UInt16  PACKED(bpb_bk_boot_sec);
    TOS_Octet   PACKED(bpb_resered[12]);
    TOS_Octet   PACKED(bs_drv_num);
    TOS_Octet   PACKED(bs_reserved1);
    TOS_Octet   PACKED(bs_boot_sig);
    TOS_UInt32  PACKED(bs_vol_id);
    TOS_Octet   PACKED(bs_vol_lab[11]);
    TOS_Octet   PACKED(bs_fil_sys_type[8]);
} BPB_FAT32;

typedef struct 
{
    TOS_Octet  PACKED(bs_jmp_boot[3]);
    TOS_Octet  PACKED(bs_oem_name[8]);
    TOS_UInt16 PACKED(bpb_byts_per_sec);
    TOS_Octet  PACKED(bpb_sec_per_clus);
    TOS_UInt16 PACKED(bpb_rsvd_sec_cnt);
    TOS_Octet  PACKED(bpb_num_fats);
    TOS_UInt16 PACKED(bpb_root_ent_cnt);
    TOS_UInt16 PACKED(bpb_tot_sec16);
    TOS_Octet  PACKED(bpb_media);
    TOS_UInt16 PACKED(bpb_fat_sz16);
    TOS_UInt16 PACKED(bpb_sec_per_trk);
    TOS_UInt16 PACKED(bpb_num_heads);
    TOS_UInt32 PACKED(bpb_hidd_sec);
    TOS_UInt32 PACKED(bpb_tot_sec32);
    union {
	BPB_FAT16 PACKED(fat16);
	BPB_FAT32 PACKED(fat32);
    } u;
} BPB_FAT;


/*
 * FAT Directory entry
 */
#define ATTR_READ_ONLY 0x01
#define ATTR_HIDDEN    0x02
#define ATTR_SYSTEM    0x04
#define ATTR_VOLUME_ID 0x08
#define ATTR_DIRECTORY 0x10
#define ATTR_ARCHIVE   0x20
#define ATTR_LONG_NAME (ATTR_READ_ONLY | \
                        ATTR_HIDDEN | \
                        ATTR_SYSTEM | \
                        ATTR_VOLUME_ID)

#define DIR_ENTRY_EMPTY      0xe5
#define DIR_ENTRY_LAST_EMPTY 0x00

typedef TOS_Octet DIR_Name[11];

typedef struct 
{
    DIR_Name   PACKED(dir_name);
    TOS_Octet  PACKED(dir_attr);
    TOS_Octet  PACKED(dir_nt_res);
    TOS_Octet  PACKED(dir_crt_time_tenth);
    TOS_UInt16 PACKED(dir_crt_time);
    TOS_UInt16 PACKED(dir_crt_date);
    TOS_UInt16 PACKED(dir_lst_acc_date);
    TOS_UInt16 PACKED(dir_fst_clus_hi);
    TOS_UInt16 PACKED(dir_wrt_time);
    TOS_UInt16 PACKED(dir_wrt_date);
    TOS_UInt16 PACKED(dir_fst_clus_lo);
    TOS_UInt32 PACKED(dir_file_size);
} FAT_DIR_ENTRY;

typedef struct
{
  unsigned char directory[256];
  unsigned char dir_entry[256];
  unsigned char fat_entry[12];
} DIR_ENTRY_INFO;

void check_sizeof_types ();

/*
 * Block I/O subsystem
 */

#ifdef FS_STANDALONE
FILE* fp;
#endif

#define BIO_SECTOR_SIZE 512
typedef TOS_Octet Sector[BIO_SECTOR_SIZE];

typedef TOS_Int32 TOS_Error;

#define TOS_NO_ERROR              0
#define TOS_ERR_FILE_NOT_FOUND   -1
#define TOS_ERR_FCB_FULL         -2
#define TOS_ERR_BAD_FILE_DESCR   -3
#define TOS_ERR_BEYOND_EOF       -4

TOS_Error fs_bio_read (void* buffer, TOS_UInt32 start_sector, TOS_UInt16 sector_count);
TOS_Error fs_bio_write (void* buffer, TOS_UInt32 start_sector, TOS_UInt16 sector_count);

/*
 * FAT
 */

typedef enum {
    FAT12_TYPE,
    FAT16_TYPE,
    FAT32_TYPE
} TOS_FAT_TYPE;

typedef struct 
{
  TOS_FAT_TYPE fat_type;
  TOS_UInt32   fat_size;       // FAT size as number of sectors
  TOS_UInt32   num_fat;
  TOS_Octet*   fat;        // Pointer to complete copy of FAT
  TOS_UInt32   bytes_per_sector;
  TOS_UInt32   sectors_per_cluster;  // # of sectors per cluster
  TOS_UInt32   root_dir_secs;  // # sectors occupied by root dir
  TOS_UInt32   first_root_dir_sector;
  TOS_UInt32   first_data_sector;
  TOS_UInt32   total_sectors;  // # of sectors in volume
  TOS_UInt32   total_data_sectors;  // # of data sectors in volume
  TOS_UInt32   total_clusters;    // # of clusters in volume
  TOS_UInt16   rsvd_sec_cnt;
} TOS_FAT_Device;

// NEW

void fs_check_fat (BPB_FAT* fat);
void fs_init (TOS_FAT_Device* dev);
TOS_UInt32 fs_get_first_sector_of_cluster (TOS_FAT_Device* dev, TOS_UInt32 cluster);
TOS_UInt32 fs_get_fat_cluster_entry (TOS_FAT_Device* dev, TOS_UInt32 cluster);
void fs_set_fat_cluster_entry (TOS_FAT_Device* dev, TOS_UInt32 cluster, TOS_UInt32 value);
TOS_UInt32 fs_is_eoc(TOS_FAT_Device* dev, TOS_UInt32 value);
TOS_Bool fs_get_free_cluster(TOS_FAT_Device* dev, TOS_UInt32* cluster);
TOS_UInt32 fs_get_directory_entry(TOS_FAT_Device* dev, TOS_Octet* directory, FAT_DIR_ENTRY* dir_entry);
TOS_UInt32 fs_update_directory_entry(TOS_FAT_Device* dev, TOS_Octet* directory, FAT_DIR_ENTRY dir_entry);
TOS_UInt32 fs_create_directory_entry(TOS_FAT_Device* dev, TOS_Octet* parent_directory, FAT_DIR_ENTRY* subdir_entry, TOS_UInt32* parent_cluster);
TOS_UInt32 fs_create_directory(TOS_FAT_Device* dev, TOS_Octet* parent_directory, TOS_Octet* child_directory);

TOS_UInt16 swap16(TOS_UInt16 val);
TOS_UInt32 swap32(TOS_UInt32 val);


// OLD (soon depricated)

void TOS_fs_check_fat (BPB_FAT* fat);
void TOS_init_fs (TOS_FAT_Device* dev);
TOS_UInt32 TOS_fs_next_cluster_in_chain (TOS_FAT_Device* dev, TOS_UInt32 cluster);
TOS_UInt32 TOS_fs_cluster_to_sector (TOS_FAT_Device* dev, TOS_UInt32 cluster);
TOS_Bool TOS_fs_is_last_cluster (TOS_FAT_Device* dev, TOS_UInt32 cluster);
void TOS_fs_format_dir_entry_name (DIR_Name entry,const char* name);

// File descriptor
typedef TOS_Int32 FAT_FD;

typedef TOS_UInt16 TOS_FS_OPEN_MODE;

TOS_Error fs_open(TOS_FAT_Device* dev, TOS_Octet* path, TOS_FS_OPEN_MODE mode);
TOS_Error fs_close(FAT_FD fd);
TOS_Error fs_read(FAT_FD fd, void* buf, TOS_UInt32 len);
TOS_Error fs_write (FAT_FD fd, void* buf, TOS_UInt32 len);


#define TOS_FS_OPEN_MODE_READ   1
#define TOS_FS_OPEN_MODE_WRITE  2
#define TOS_FS_OPEN_MODE_APPEND 4

typedef struct 
{

    TOS_Bool         used;
    TOS_UInt16       seq;
    TOS_FAT_Device*  dev;
    TOS_UInt32       file_size;
    TOS_UInt32       pos;
    TOS_UInt32       first_cluster;
    TOS_UInt32       current_cluster;
    TOS_UInt32       owner;
    Sector           sector;
    TOS_FS_OPEN_MODE mode;

    TOS_Octet        path[256]; // workaround
} FCB_Entry;

#define NUM_FCB  20

void TOS_init_fcb ();
TOS_Error TOS_fs_find_dir_entry (TOS_FAT_Device* dev, const char* fname, FAT_DIR_ENTRY* entry);
TOS_Error TOS_fs_open (TOS_FAT_Device* dev, const char* file_name, TOS_FS_OPEN_MODE mode);
TOS_Error TOS_fs_close (FAT_FD fd);
TOS_Error TOS_fs_is_eof (FAT_FD fd);
TOS_Error TOS_fs_chown (FAT_FD fd, TOS_UInt32 pid);
TOS_Error TOS_fs_read (FAT_FD fd, void* buf, TOS_UInt32 size);
TOS_Error TOS_fs_seek (FAT_FD fd, TOS_UInt32 pos);
TOS_Error TOS_fs_process_exited (TOS_UInt32 pid);

#endif
