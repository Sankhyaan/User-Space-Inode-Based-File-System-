This project presents the design and implementation of a custom inode-based file system in user space using
the Filesystem in Userspace (FUSE) framework. The primary objective of this project is to understand and
replicate the internal workings of traditional UNIX-like file systems while avoiding kernel-level
development complexity. The file system is mounted as a virtual file system and supports fundamental file
operations — file creation, deletion, reading, writing, and directory management — through standard Linux
commands. A disk image is used as a virtual storage device to simulate persistent secondary storage. The
file system layout consists of a superblock for global metadata, an inode table for maintaining file
information, directory entries for filename-to-inode mapping, and data blocks for storing file contents. A
bitmap-based allocation mechanism manages free inodes and data blocks. The implementation is fully
operational and demonstrates key operating system concepts including storage abstraction, hierarchical path
resolution, metadata management, and block allocation strategies. 
 
