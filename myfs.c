/*
 * myfs.c — User-Space Inode-Based File System using FUSE
 * Implements: superblock, bitmap allocator, inode table,
 *             path resolution, and all core FUSE callbacks.
 *
 * Build:
 *   gcc -Wall `pkg-config fuse --cflags --libs` myfs.c -o myfs
 *
 * Format:
 *   ./myfs --mkfs disk.img
 *
 * Mount:
 *   ./myfs disk.img /mnt/myfs
 *
 * Unmount:
 *   fusermount -u /mnt/myfs
 */

#define FUSE_USE_VERSION 26

#include <fuse.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <stdint.h>
#include <libgen.h>
#include <sys/time.h>

/* =========================================================
 * CONSTANTS
 * ========================================================= */

#define FS_MAGIC 0xDEADBEEF
#define BLOCK_SIZE 4096
#define MAX_INODES 256
#define MAX_BLOCKS 1024
#define MAX_DIRECT_PTRS 8
#define MAX_FILENAME 255
#define MAX_DIR_ENTRIES 32 /* entries per directory block */

/* File types stored in inode.file_type */
#define FT_REGULAR 1
#define FT_DIRECTORY 2

/* Disk layout offsets (all in bytes) */
#define SUPERBLOCK_OFFSET 0
#define INODE_BITMAP_OFFSET (SUPERBLOCK_OFFSET + BLOCK_SIZE)
#define BLOCK_BITMAP_OFFSET (INODE_BITMAP_OFFSET + BLOCK_SIZE)
#define INODE_TABLE_OFFSET (BLOCK_BITMAP_OFFSET + BLOCK_SIZE)
#define DATA_OFFSET (INODE_TABLE_OFFSET + MAX_INODES * sizeof(struct inode))

static int printed_flag = 0;

/* =========================================================
 * ON-DISK STRUCTURES
 * ========================================================= */

struct superblock
{
    uint32_t magic;
    uint32_t total_blocks;
    uint32_t total_inodes;
    uint32_t free_blocks;
    uint32_t free_inodes;
    uint32_t block_size;
};

struct inode
{
    uint32_t inode_number;
    uint8_t file_type; /* FT_REGULAR or FT_DIRECTORY */
    uint16_t permissions;
    uint32_t file_size;
    uint32_t link_count;
    time_t atime;
    time_t mtime;
    time_t ctime;
    int32_t block_ptrs[MAX_DIRECT_PTRS]; /* -1 = unused */
    uint8_t is_used;
};

struct dir_entry
{
    char filename[MAX_FILENAME + 1];
    uint32_t inode_number;
    uint8_t is_valid;
};

/* =========================================================
 * GLOBAL STATE
 * ========================================================= */

static int disk_fd = -1;
static struct superblock sb;
static uint8_t inode_bitmap[MAX_INODES];
static uint8_t block_bitmap[MAX_BLOCKS];

/* =========================================================
 * ALGORITHM 1 — LOW-LEVEL DISK I/O
 * ========================================================= */

/* Read exactly `len` bytes from `offset` in the disk image. */
static int disk_read(off_t offset, void *buf, size_t len)
{
    if (lseek(disk_fd, offset, SEEK_SET) < 0)
    {
        perror("lseek read");
        return -EIO;
    }

    ssize_t n = read(disk_fd, buf, len);
    if (n != (ssize_t)len)
    {
        perror("read");
        return -EIO;
    }
    return 0;
}

static int disk_write(off_t offset, const void *buf, size_t len)
{
    if (lseek(disk_fd, offset, SEEK_SET) < 0)
    {
        perror("lseek write");
        return -EIO;
    }

    ssize_t n = write(disk_fd, buf, len);
    if (n != (ssize_t)len)
    {
        perror("write");
        return -EIO;
    }
    return 0;
}

/* =========================================================
 * ALGORITHM 2 — SUPERBLOCK & BITMAP PERSISTENCE
 * ========================================================= */

static int flush_superblock(void)
{
    return disk_write(SUPERBLOCK_OFFSET, &sb, sizeof(sb));
}

static int flush_bitmaps(void)
{
    int r;
    r = disk_write(INODE_BITMAP_OFFSET, inode_bitmap, sizeof(inode_bitmap));
    if (r)
        return r;
    return disk_write(BLOCK_BITMAP_OFFSET, block_bitmap, sizeof(block_bitmap));
}

static int load_superblock(void)
{
    return disk_read(SUPERBLOCK_OFFSET, &sb, sizeof(sb));
}

static int load_bitmaps(void)
{
    int r;
    r = disk_read(INODE_BITMAP_OFFSET, inode_bitmap, sizeof(inode_bitmap));
    if (r)
        return r;
    return disk_read(BLOCK_BITMAP_OFFSET, block_bitmap, sizeof(block_bitmap));
}

/* =========================================================
 * ALGORITHM 3 — INODE TABLE I/O
 * ========================================================= */

static off_t inode_offset(uint32_t inum)
{
    return INODE_TABLE_OFFSET + (off_t)inum * sizeof(struct inode);
}

static int load_inode(uint32_t inum, struct inode *out)
{
    if (inum >= MAX_INODES)
        return -EINVAL;
    return disk_read(inode_offset(inum), out, sizeof(struct inode));
}

static int save_inode(uint32_t inum, const struct inode *in)
{
    if (inum >= MAX_INODES)
        return -EINVAL;
    return disk_write(inode_offset(inum), in, sizeof(struct inode));
}

/* =========================================================
 * ALGORITHM 4 — DATA BLOCK I/O
 * ========================================================= */

static off_t block_offset(int32_t bnum)
{
    return DATA_OFFSET + (off_t)bnum * BLOCK_SIZE;
}

static int read_block(int32_t bnum, void *buf)
{
    return disk_read(block_offset(bnum), buf, BLOCK_SIZE);
}

static int write_block(int32_t bnum, const void *buf)
{
    return disk_write(block_offset(bnum), buf, BLOCK_SIZE);
}

/* Zero-fill a freshly allocated block to prevent stale-data leaks. */
static int zero_block(int32_t bnum)
{
    uint8_t zeros[BLOCK_SIZE];
    memset(zeros, 0, BLOCK_SIZE);
    return write_block(bnum, zeros);
}

/* =========================================================
 * ALGORITHM 5 — BITMAP-BASED ALLOCATOR & DEALLOCATOR
 * ========================================================= */

/*
 * allocate_inode()
 *   Linear scan of inode_bitmap. First free slot is claimed,
 *   superblock free count decremented, bitmaps flushed.
 *   Returns inode number, or -ENOSPC if full.
 */
static int allocate_inode(void)
{
    for (uint32_t i = 0; i < MAX_INODES; i++)
    {
        if (inode_bitmap[i] == 0)
        {
            inode_bitmap[i] = 1;
            sb.free_inodes--;
            flush_bitmaps();
            flush_superblock();
            return (int)i;
        }
    }
    return -ENOSPC;
}

static int free_inode(uint32_t inum)
{
    if (inum >= MAX_INODES)
        return -EINVAL;
    inode_bitmap[inum] = 0;
    sb.free_inodes++;
    flush_bitmaps();
    flush_superblock();
    return 0;
}

/*
 * allocate_block()
 *   Linear scan of block_bitmap. Newly allocated block is
 *   zero-filled before use.
 *   Returns block number, or -ENOSPC if full.
 */
static int allocate_block(void)
{
    for (uint32_t b = 0; b < MAX_BLOCKS; b++)
    {
        if (block_bitmap[b] == 0)
        {
            block_bitmap[b] = 1;
            sb.free_blocks--;
            zero_block((int32_t)b);
            flush_bitmaps();
            flush_superblock();
            return (int)b;
        }
    }
    return -ENOSPC;
}

static int free_block(int32_t bnum)
{
    if (bnum < 0 || (uint32_t)bnum >= MAX_BLOCKS)
        return -EINVAL;
    block_bitmap[bnum] = 0;
    sb.free_blocks++;
    flush_bitmaps();
    flush_superblock();
    return 0;
}

/* =========================================================
 * ALGORITHM 6 — DIRECTORY ENTRY HELPERS
 * ========================================================= */

/*
 * add_dir_entry()
 *   Walks every allocated block of `dir_inum` looking for a
 *   slot where is_valid == 0.  If none found, allocates a new
 *   block and writes the entry into its first slot.
 */
static int add_dir_entry(uint32_t dir_inum, const char *name, uint32_t target_inum)
{
    struct inode dir_inode;
    if (load_inode(dir_inum, &dir_inode) != 0)
        return -EIO;

    struct dir_entry entries[MAX_DIR_ENTRIES];

    /* Search existing blocks for a free slot */
    for (int i = 0; i < MAX_DIRECT_PTRS; i++)
    {
        if (dir_inode.block_ptrs[i] < 0)
            continue;

        memset(entries, 0, sizeof(entries));
        if (read_block(dir_inode.block_ptrs[i], entries) != 0)
            return -EIO;

        for (int j = 0; j < MAX_DIR_ENTRIES; j++)
        {
            if (!entries[j].is_valid)
            {
                strncpy(entries[j].filename, name, MAX_FILENAME);
                entries[j].filename[MAX_FILENAME] = '\0';
                entries[j].inode_number = target_inum;
                entries[j].is_valid = 1;
                return write_block(dir_inode.block_ptrs[i], entries);
            }
        }
    }

    /* No free slot — allocate a new block */
    int new_block = allocate_block();
    if (new_block < 0)
        return -ENOSPC;

    /* Find a free block_ptr slot in the inode */
    for (int i = 0; i < MAX_DIRECT_PTRS; i++)
    {
        if (dir_inode.block_ptrs[i] < 0)
        {
            memset(entries, 0, sizeof(entries));
            strncpy(entries[0].filename, name, MAX_FILENAME);
            entries[0].filename[MAX_FILENAME] = '\0';
            entries[0].inode_number = target_inum;
            entries[0].is_valid = 1;
            if (write_block(new_block, entries) != 0)
                return -EIO;

            dir_inode.block_ptrs[i] = new_block;
            dir_inode.file_size += BLOCK_SIZE;
            return save_inode(dir_inum, &dir_inode);
        }
    }

    return -ENOSPC; /* inode has no more block_ptr slots */
}

/*
 * remove_dir_entry()
 *   Scans directory blocks and clears the matching entry.
 */
static int remove_dir_entry(uint32_t dir_inum, const char *name)
{
    struct inode dir_inode;
    if (load_inode(dir_inum, &dir_inode) != 0)
        return -EIO;

    struct dir_entry entries[MAX_DIR_ENTRIES];

    for (int i = 0; i < MAX_DIRECT_PTRS; i++)
    {
        if (dir_inode.block_ptrs[i] < 0)
            continue;

        memset(entries, 0, sizeof(entries));
        if (read_block(dir_inode.block_ptrs[i], entries) != 0)
            return -EIO;

        for (int j = 0; j < MAX_DIR_ENTRIES; j++)
        {
            if (entries[j].is_valid &&
                strncmp(entries[j].filename, name, MAX_FILENAME) == 0)
            {
                memset(&entries[j], 0, sizeof(struct dir_entry));
                return write_block(dir_inode.block_ptrs[i], entries);
            }
        }
    }
    return -ENOENT;
}

/* =========================================================
 * ALGORITHM 7 — PATH RESOLUTION (lookup)
 * ========================================================= */

/*
 * resolve_path()
 *   Splits the path into components and walks the directory
 *   tree starting from the root inode (inum = 0).
 *   Returns inode number on success, negative errno on failure.
 */
static int resolve_path(const char *path)
{
    if (strcmp(path, "/") == 0)
        return 0; /* root inode */

    /* Work on a mutable copy */
    char tmp[4096];
    strncpy(tmp, path, sizeof(tmp) - 1);
    tmp[sizeof(tmp) - 1] = '\0';

    int current_inum = 0;
    char *saveptr = NULL;
    char *token = strtok_r(tmp + 1, "/", &saveptr); /* skip leading '/' */

    while (token != NULL)
    {
        struct inode cur;
        if (load_inode((uint32_t)current_inum, &cur) != 0)
            return -EIO;
        if (cur.file_type != FT_DIRECTORY)
            return -ENOTDIR;

        struct dir_entry entries[MAX_DIR_ENTRIES];
        int found = 0;

        for (int i = 0; i < MAX_DIRECT_PTRS && !found; i++)
        {
            if (cur.block_ptrs[i] < 0)
                continue;
            memset(entries, 0, sizeof(entries));
            if (read_block(cur.block_ptrs[i], entries) != 0)
                return -EIO;

            for (int j = 0; j < MAX_DIR_ENTRIES; j++)
            {
                if (entries[j].is_valid &&
                    strncmp(entries[j].filename, token, MAX_FILENAME) == 0)
                {
                    current_inum = (int)entries[j].inode_number;
                    found = 1;
                    break;
                }
            }
        }

        if (!found)
            return -ENOENT;
        token = strtok_r(NULL, "/", &saveptr);
    }

    return current_inum;
}

/* =========================================================
 * ALGORITHM 8 — FILE SYSTEM INITIALIZATION (mkfs)
 * ========================================================= */

/*
 * init_filesystem()
 *   Writes superblock, zero-initialises bitmaps and inode table,
 *   then bootstraps the root directory with "." and ".." entries.
 */
static int init_filesystem(const char *disk_path)
{
    disk_fd = open(disk_path, O_RDWR | O_CREAT | O_TRUNC, 0644);
    if (disk_fd < 0)
    {
        perror("open");
        return -1;
    }

    /* Size the image to hold everything */
    size_t image_size = DATA_OFFSET + (size_t)MAX_BLOCKS * BLOCK_SIZE;
    if (ftruncate(disk_fd, (off_t)image_size) != 0)
    {
        perror("ftruncate");
        return -1;
    }

    /* --- Superblock --- */
    memset(&sb, 0, sizeof(sb));
    sb.magic = FS_MAGIC;
    sb.total_blocks = MAX_BLOCKS;
    sb.total_inodes = MAX_INODES;
    sb.free_blocks = MAX_BLOCKS;
    sb.free_inodes = MAX_INODES;
    sb.block_size = BLOCK_SIZE;
    flush_superblock();

    /* --- Zero bitmaps --- */
    memset(inode_bitmap, 0, sizeof(inode_bitmap));
    memset(block_bitmap, 0, sizeof(block_bitmap));
    flush_bitmaps();

    /* --- Zero inode table --- */
    struct inode blank;
    memset(&blank, 0, sizeof(blank));
    for (int i = 0; i < MAX_DIRECT_PTRS; i++)
        blank.block_ptrs[i] = -1;
    for (uint32_t i = 0; i < MAX_INODES; i++)
        save_inode(i, &blank);

    /* --- Root inode (inum = 0) --- */
    int root_inum = allocate_inode(); /* returns 0 */
    int root_block = allocate_block();

    struct inode root;
    memset(&root, 0, sizeof(root));
    root.inode_number = (uint32_t)root_inum;
    root.file_type = FT_DIRECTORY;
    root.permissions = 0755;
    root.link_count = 2;
    root.file_size = BLOCK_SIZE;
    root.atime = root.mtime = root.ctime = time(NULL);
    for (int i = 0; i < MAX_DIRECT_PTRS; i++)
        root.block_ptrs[i] = -1;
    root.block_ptrs[0] = root_block;
    root.is_used = 1;
    save_inode(0, &root);

    /* Write "." and ".." into root block */
    struct dir_entry entries[MAX_DIR_ENTRIES];
    memset(entries, 0, sizeof(entries));
    strncpy(entries[0].filename, ".", MAX_FILENAME);
    entries[0].inode_number = 0;
    entries[0].is_valid = 1;
    strncpy(entries[1].filename, "..", MAX_FILENAME);
    entries[1].inode_number = 0;
    entries[1].is_valid = 1;
    write_block(root_block, entries);

    printf("mkfs: disk image '%s' created (%zu bytes)\n", disk_path, image_size);
    close(disk_fd);
    disk_fd = -1;
    return 0;
}

/* =========================================================
 * ALGORITHM 9 — FUSE CALLBACK: getattr
 * ========================================================= */

static int fs_getattr(const char *path, struct stat *stbuf)
{

    memset(stbuf, 0, sizeof(struct stat));

    int inum = resolve_path(path);
    if (inum < 0)
        return inum;

    struct inode in;
    if (load_inode((uint32_t)inum, &in) != 0)
        return -EIO;

    stbuf->st_ino = in.inode_number;
    stbuf->st_nlink = in.link_count;
    stbuf->st_size = in.file_size;
    stbuf->st_atime = in.atime;
    stbuf->st_mtime = in.mtime;
    stbuf->st_ctime = in.ctime;

    if (in.file_type == FT_DIRECTORY)
        stbuf->st_mode = S_IFDIR | in.permissions;
    else
        stbuf->st_mode = S_IFREG | in.permissions;

    return 0;
}

/* =========================================================
 * ALGORITHM 10 — FUSE CALLBACK: readdir
 * ========================================================= */

static int fs_readdir(const char *path, void *buf, fuse_fill_dir_t filler,
                      off_t offset, struct fuse_file_info *fi)
{
    if (!printed_flag)
    {
        fprintf(stderr, "LS: %s\n", path);
        printed_flag = 1;
    }
    (void)offset;
    (void)fi;

    int inum = resolve_path(path);
    if (inum < 0)
        return inum;

    struct inode dir_inode;
    if (load_inode((uint32_t)inum, &dir_inode) != 0)
        return -EIO;
    if (dir_inode.file_type != FT_DIRECTORY)
        return -ENOTDIR;

    struct dir_entry entries[MAX_DIR_ENTRIES];

    for (int i = 0; i < MAX_DIRECT_PTRS; i++)
    {
        if (dir_inode.block_ptrs[i] < 0)
            continue;
        memset(entries, 0, sizeof(entries));
        if (read_block(dir_inode.block_ptrs[i], entries) != 0)
            return -EIO;

        for (int j = 0; j < MAX_DIR_ENTRIES; j++)
        {
            if (entries[j].is_valid)
            {
                struct stat st;
                memset(&st, 0, sizeof(st));
                struct inode child;
                if (load_inode(entries[j].inode_number, &child) == 0)
                {
                    st.st_mode = (child.file_type == FT_DIRECTORY)
                                     ? S_IFDIR | child.permissions
                                     : S_IFREG | child.permissions;
                }
                filler(buf, entries[j].filename, &st, 0);
            }
        }
    }
    printed_flag = 0;
    return 0;
}

/* =========================================================
 * ALGORITHM 11 — FUSE CALLBACK: create / mknod
 * ========================================================= */

static int fs_create(const char *path, mode_t mode, struct fuse_file_info *fi)
{
    if (!printed_flag)
    {
        fprintf(stderr, "CREATE: %s\n", path);
        printed_flag = 1;
    }
    (void)fi;

    /* Reject if already exists */
    if (resolve_path(path) >= 0)
        return -EEXIST;

    /* Compute parent path and filename */
    char tmp_path[4096], tmp_base[4096];
    strncpy(tmp_path, path, sizeof(tmp_path) - 1);
    strncpy(tmp_base, path, sizeof(tmp_base) - 1);
    char *parent_path = dirname(tmp_path);
    char *fname = basename(tmp_base);

    int parent_inum = resolve_path(parent_path);
    if (parent_inum < 0)
        return -ENOENT;

    int new_inum = allocate_inode();
    if (new_inum < 0)
        return -ENOSPC;

    struct inode new_inode;
    memset(&new_inode, 0, sizeof(new_inode));
    new_inode.inode_number = (uint32_t)new_inum;
    new_inode.file_type = FT_REGULAR;
    new_inode.permissions = mode & 0777;
    new_inode.file_size = 0;
    new_inode.link_count = 1;
    new_inode.atime = new_inode.mtime = new_inode.ctime = time(NULL);
    for (int i = 0; i < MAX_DIRECT_PTRS; i++)
        new_inode.block_ptrs[i] = -1;
    new_inode.is_used = 1;
    save_inode((uint32_t)new_inum, &new_inode);
    printed_flag = 0;

    return add_dir_entry((uint32_t)parent_inum, fname, (uint32_t)new_inum);
}

/* =========================================================
 * ALGORITHM 12 — FUSE CALLBACK: mkdir
 * ========================================================= */

static int fs_mkdir(const char *path, mode_t mode)
{
    if (resolve_path(path) >= 0)
        return -EEXIST;

    if (!printed_flag)
    {
        fprintf(stderr, "MKDIR: %s\n", path);
        printed_flag = 1;
    }

    char tmp_path[4096], tmp_base[4096];
    strncpy(tmp_path, path, sizeof(tmp_path) - 1);
    strncpy(tmp_base, path, sizeof(tmp_base) - 1);
    char *parent_path = dirname(tmp_path);
    char *dname = basename(tmp_base);

    int parent_inum = resolve_path(parent_path);
    if (parent_inum < 0)
        return -ENOENT;

    int new_inum = allocate_inode();
    if (new_inum < 0)
        return -ENOSPC;
    int new_block = allocate_block();
    if (new_block < 0)
    {
        free_inode((uint32_t)new_inum);
        return -ENOSPC;
    }

    /* Build inode */
    struct inode new_inode;
    memset(&new_inode, 0, sizeof(new_inode));
    new_inode.inode_number = (uint32_t)new_inum;
    new_inode.file_type = FT_DIRECTORY;
    new_inode.permissions = mode & 0777;
    new_inode.link_count = 2;
    new_inode.file_size = BLOCK_SIZE;
    new_inode.atime = new_inode.mtime = new_inode.ctime = time(NULL);
    for (int i = 0; i < MAX_DIRECT_PTRS; i++)
        new_inode.block_ptrs[i] = -1;
    new_inode.block_ptrs[0] = new_block;
    new_inode.is_used = 1;
    save_inode((uint32_t)new_inum, &new_inode);

    /* Write "." and ".." entries */
    struct dir_entry entries[MAX_DIR_ENTRIES];
    memset(entries, 0, sizeof(entries));
    strncpy(entries[0].filename, ".", MAX_FILENAME);
    entries[0].inode_number = (uint32_t)new_inum;
    entries[0].is_valid = 1;
    strncpy(entries[1].filename, "..", MAX_FILENAME);
    entries[1].inode_number = (uint32_t)parent_inum;
    entries[1].is_valid = 1;
    write_block(new_block, entries);
    printed_flag = 0;

    /* Link into parent */
    return add_dir_entry((uint32_t)parent_inum, dname, (uint32_t)new_inum);
}

/* =========================================================
 * ALGORITHM 13 — FUSE CALLBACK: read
 * ========================================================= */

static int fs_read(const char *path, char *buf, size_t size,
                   off_t offset, struct fuse_file_info *fi)
{
    if (!printed_flag)
    {
        fprintf(stderr, "READ: %s\n", path);
        printed_flag = 1;
    }

    int inum = resolve_path(path);
    if (inum < 0)
        return inum;

    struct inode in;
    if (load_inode((uint32_t)inum, &in) != 0)
        return -EIO;
    if (in.file_type != FT_REGULAR)
        return -EISDIR;

    /* Clamp to file size */
    if ((size_t)offset >= in.file_size)
        return 0;
    if (offset + (off_t)size > (off_t)in.file_size)
        size = (size_t)(in.file_size - offset);

    size_t bytes_read = 0;
    uint8_t block_buf[BLOCK_SIZE];

    while (bytes_read < size)
    {
        size_t cur_offset = (size_t)offset + bytes_read;
        int block_index = (int)(cur_offset / BLOCK_SIZE);
        size_t block_off = cur_offset % BLOCK_SIZE;
        size_t chunk = BLOCK_SIZE - block_off;
        if (chunk > size - bytes_read)
            chunk = size - bytes_read;

        if (block_index >= MAX_DIRECT_PTRS ||
            in.block_ptrs[block_index] < 0)
        {
            /* Sparse region — return zeros */
            memset(buf + bytes_read, 0, chunk);
        }
        else
        {
            if (read_block(in.block_ptrs[block_index], block_buf) != 0)
                return -EIO;
            memcpy(buf + bytes_read, block_buf + block_off, chunk);
        }
        bytes_read += chunk;
    }

    in.atime = time(NULL);
    save_inode((uint32_t)inum, &in);
    printed_flag = 0;
    return (int)bytes_read;
}

static int fs_rename(const char *from, const char *to);

/* =========================================================
 * ALGORITHM 14 — FUSE CALLBACK: write
 * ========================================================= */

static int fs_write(const char *path, const char *buf, size_t size,
                    off_t offset, struct fuse_file_info *fi)
{
    if (!printed_flag)
    {
        if (offset == 0)
            fprintf(stderr, "OVERWRITE: %s -> %.*s\n", path, (int)size, buf);
        else
            fprintf(stderr, "APPEND: %s -> %.*s\n", path, (int)size, buf);

        printed_flag = 1;
    }

    (void)fi;

    int inum = resolve_path(path);
    // -------- VERSIONING LOGIC START --------

// max versions to keep
#define MAX_VERSIONS 3

    char old_path[256], new_path[256];

    // shift backups (bak2 -> bak3, bak1 -> bak2, etc.)
    for (int i = MAX_VERSIONS; i >= 1; i--)
    {
        if (i == 1)
            sprintf(old_path, "%s", path); // original file
        else
            sprintf(old_path, "%s.bak%d", path, i - 1);

        sprintf(new_path, "%s.bak%d", path, i);

        // check if old_path exists
        if (resolve_path(old_path) >= 0)
        {
            fs_rename(old_path, new_path);
        }
    }

    // -------- VERSIONING LOGIC END --------
    if (inum < 0)
        return inum;

    struct inode in;
    if (load_inode((uint32_t)inum, &in) != 0)
        return -EIO;

    size_t bytes_written = 0;
    uint8_t block_buf[BLOCK_SIZE];

    while (bytes_written < size)
    {
        size_t cur_offset = (size_t)offset + bytes_written;
        int block_index = (int)(cur_offset / BLOCK_SIZE);
        size_t block_off = cur_offset % BLOCK_SIZE;
        size_t chunk = BLOCK_SIZE - block_off;
        if (chunk > size - bytes_written)
            chunk = size - bytes_written;

        if (block_index >= MAX_DIRECT_PTRS)
            return -EFBIG;

        /* Allocate block on demand */
        if (in.block_ptrs[block_index] < 0)
        {
            int nb = allocate_block();
            if (nb < 0)
                return (bytes_written > 0) ? (int)bytes_written : -ENOSPC;
            in.block_ptrs[block_index] = nb;
        }

        /* Read-modify-write (preserves existing data in partial writes) */
        if (read_block(in.block_ptrs[block_index], block_buf) != 0)
            return -EIO;
        memcpy(block_buf + block_off, buf + bytes_written, chunk);
        if (write_block(in.block_ptrs[block_index], block_buf) != 0)
            return -EIO;

        bytes_written += chunk;
    }

    size_t end = (size_t)offset + bytes_written;
    if (end > in.file_size)
        in.file_size = (uint32_t)end;
    in.mtime = time(NULL);
    save_inode((uint32_t)inum, &in);
    printed_flag = 0;
    return (int)bytes_written;
}

/* =========================================================
 * ALGORITHM 15 — FUSE CALLBACK: unlink
 * ========================================================= */

static int fs_unlink(const char *path)
{
    if (!printed_flag)
    {
        fprintf(stderr, "DELETE FILE: %s\n", path);
        printed_flag = 1;
    }
    int inum = resolve_path(path);
    if (inum < 0)
        return inum;

    struct inode in;
    if (load_inode((uint32_t)inum, &in) != 0)
        return -EIO;
    if (in.file_type == FT_DIRECTORY)
        return -EISDIR;

    in.link_count--;

    if (in.link_count == 0)
    {
        /* Free every data block */
        for (int i = 0; i < MAX_DIRECT_PTRS; i++)
        {
            if (in.block_ptrs[i] >= 0)
            {
                free_block(in.block_ptrs[i]);
                in.block_ptrs[i] = -1;
            }
        }
        in.is_used = 0;
        save_inode((uint32_t)inum, &in);
        free_inode((uint32_t)inum);
    }
    else
    {
        save_inode((uint32_t)inum, &in);
    }

    /* Remove directory entry from parent */
    char tmp_path[4096], tmp_base[4096];
    strncpy(tmp_path, path, sizeof(tmp_path) - 1);
    strncpy(tmp_base, path, sizeof(tmp_base) - 1);
    char *parent_path = dirname(tmp_path);
    char *fname = basename(tmp_base);
    int parent_inum = resolve_path(parent_path);
    printed_flag = 0;
    if (parent_inum < 0)
        return parent_inum;

    return remove_dir_entry((uint32_t)parent_inum, fname);
}

/* =========================================================
 * ALGORITHM 16 — FUSE CALLBACK: rmdir
 * ========================================================= */

static int fs_rmdir(const char *path)
{
    if (!printed_flag)
    {
        fprintf(stderr, "REMOVE DIR: %s\n", path);
        printed_flag = 1;
    }
    int inum = resolve_path(path);
    if (inum < 0)
        return inum;

    struct inode in;
    if (load_inode((uint32_t)inum, &in) != 0)
        return -EIO;
    if (in.file_type != FT_DIRECTORY)
        return -ENOTDIR;

    /* Check directory is empty (only "." and ".." allowed) */
    struct dir_entry entries[MAX_DIR_ENTRIES];
    int count = 0;
    for (int i = 0; i < MAX_DIRECT_PTRS; i++)
    {
        if (in.block_ptrs[i] < 0)
            continue;
        memset(entries, 0, sizeof(entries));
        read_block(in.block_ptrs[i], entries);
        for (int j = 0; j < MAX_DIR_ENTRIES; j++)
        {
            if (entries[j].is_valid)
                count++;
        }
    }
    if (count > 2)
        return -ENOTEMPTY; /* more than "." and ".." */

    /* Free directory blocks */
    for (int i = 0; i < MAX_DIRECT_PTRS; i++)
    {
        if (in.block_ptrs[i] >= 0)
        {
            free_block(in.block_ptrs[i]);
            in.block_ptrs[i] = -1;
        }
    }
    in.is_used = 0;
    save_inode((uint32_t)inum, &in);
    free_inode((uint32_t)inum);

    char tmp_path[4096], tmp_base[4096];
    strncpy(tmp_path, path, sizeof(tmp_path) - 1);
    strncpy(tmp_base, path, sizeof(tmp_base) - 1);
    char *parent_path = dirname(tmp_path);
    char *dname = basename(tmp_base);
    int parent_inum = resolve_path(parent_path);
    printed_flag = 0;
    if (parent_inum < 0)
        return parent_inum;

    return remove_dir_entry((uint32_t)parent_inum, dname);
}

/* =========================================================
 * ALGORITHM 17 — FUSE CALLBACK: rename
 * ========================================================= */

static int fs_rename(const char *from, const char *to)
{
    printf("[RENAME] from = %s to = %s\n", from, to);
    int src_inum = resolve_path(from);
    if (src_inum < 0)
        return src_inum;

    /* Remove old name; add new name pointing to same inode */
    char tmp_from[4096], tmp_base_from[4096];
    strncpy(tmp_from, from, sizeof(tmp_from) - 1);
    strncpy(tmp_base_from, from, sizeof(tmp_base_from) - 1);
    char *src_parent = dirname(tmp_from);
    char *src_name = basename(tmp_base_from);
    int src_parent_inum = resolve_path(src_parent);
    if (src_parent_inum < 0)
        return src_parent_inum;

    char tmp_to[4096], tmp_base_to[4096];
    strncpy(tmp_to, to, sizeof(tmp_to) - 1);
    strncpy(tmp_base_to, to, sizeof(tmp_base_to) - 1);
    char *dst_parent = dirname(tmp_to);
    char *dst_name = basename(tmp_base_to);
    int dst_parent_inum = resolve_path(dst_parent);
    if (dst_parent_inum < 0)
        return dst_parent_inum;

    /* If destination exists, unlink it first */
    if (resolve_path(to) >= 0)
        fs_unlink(to);

    remove_dir_entry((uint32_t)src_parent_inum, src_name);
    return add_dir_entry((uint32_t)dst_parent_inum, dst_name, (uint32_t)src_inum);
}

/* =========================================================
 * ALGORITHM 18 — FUSE CALLBACK: truncate
 * ========================================================= */

static int fs_truncate(const char *path, off_t new_size)
{
    printf("[TRUNCATE] path = %s, new_size = %ld\n", path, new_size);
    int inum = resolve_path(path);
    if (inum < 0)
        return inum;

    struct inode in;
    if (load_inode((uint32_t)inum, &in) != 0)
        return -EIO;

    /* Free blocks beyond the new size */
    int last_block_needed = (new_size > 0)
                                ? (int)((new_size - 1) / BLOCK_SIZE)
                                : -1;

    for (int i = 0; i < MAX_DIRECT_PTRS; i++)
    {
        if (in.block_ptrs[i] >= 0 && i > last_block_needed)
        {
            free_block(in.block_ptrs[i]);
            in.block_ptrs[i] = -1;
        }
    }

    in.file_size = (uint32_t)new_size;
    in.mtime = time(NULL);
    return save_inode((uint32_t)inum, &in);
}

/* =========================================================
 * ALGORITHM 19 — FUSE CALLBACK: utimens
 *   Called by touch, cp --preserve, and any syscall that
 *   updates atime/mtime on an existing file.
 * ========================================================= */

static int fs_utimens(const char *path, const struct timespec ts[2])
{

    int inum = resolve_path(path);
    if (inum < 0)
        return inum;

    struct inode in;
    if (load_inode((uint32_t)inum, &in) != 0)
        return -EIO;

    in.atime = ts[0].tv_sec; /* access time  */
    in.mtime = ts[1].tv_sec; /* modification time */

    return save_inode((uint32_t)inum, &in);
}

/* =========================================================
 * ALGORITHM 20 — FUSE CALLBACK: open (stateless)
 * ========================================================= */

static int fs_open(const char *path, struct fuse_file_info *fi)
{
    printf("[OPEN] path = %s\n", path);
    (void)fi;
    int inum = resolve_path(path);
    if (inum < 0)
        return inum;
    return 0;
}

/* =========================================================
 * ALGORITHM 21 — FUSE MAIN LOOP & MOUNT
 * ========================================================= */

static struct fuse_operations fs_ops = {
    .getattr = fs_getattr,
    .readdir = fs_readdir,
    .create = fs_create,
    .mkdir = fs_mkdir,
    .open = fs_open,
    .read = fs_read,
    .write = fs_write,
    .unlink = fs_unlink,
    .rmdir = fs_rmdir,
    .rename = fs_rename,
    .truncate = fs_truncate,
    .utimens = fs_utimens,
};

/*
 * mount_filesystem()
 *   Opens the disk image, loads superblock + bitmaps into
 *   memory, then hands control to fuse_main() which blocks
 *   and dispatches VFS requests to the callbacks above.
 */
static int mount_filesystem(const char *disk_path, int argc, char *argv[])
{
    disk_fd = open(disk_path, O_RDWR);
    if (disk_fd < 0)
    {
        perror("open disk");
        return -1;
    }

    if (load_superblock() != 0 || sb.magic != FS_MAGIC)
    {
        fprintf(stderr, "Bad magic — run with --mkfs first.\n");
        return -1;
    }
    if (load_bitmaps() != 0)
    {
        fprintf(stderr, "Failed to load bitmaps.\n");
        return -1;
    }

    printf("myfs: mounted '%s' (%u blocks, %u inodes free)\n",
           disk_path, sb.free_blocks, sb.free_inodes);

    return fuse_main(argc, argv, &fs_ops, NULL);
}

/* =========================================================
 * ENTRY POINT
 * ========================================================= */

int main(int argc, char *argv[])
{

    if (argc >= 2 && strcmp(argv[1], "--mkfs") == 0)
    {
        if (argc < 3)
        {
            fprintf(stderr, "Usage: %s --mkfs <disk.img>\n", argv[0]);
            return 1;
        }
        return init_filesystem(argv[2]);
    }

    if (argc < 3)
    {
        fprintf(stderr, "Usage:\n");
        fprintf(stderr, "  %s --mkfs <disk.img>\n", argv[0]);
        fprintf(stderr, "  %s <disk.img> <mountpoint> [fuse options]\n", argv[0]);
        return 1;
    }

    /* Remove disk_path from argv before passing to fuse_main */
    char *disk_path = argv[1];
    argv[1] = argv[0];
    return mount_filesystem(disk_path, argc - 1, argv + 1);
}