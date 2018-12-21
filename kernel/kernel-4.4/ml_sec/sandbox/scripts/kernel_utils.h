#ifndef __MLSEC_SBOX_SCRIPTS_KERNEL_UTILS_H
#define __MLSEC_SBOX_SCRIPTS_KERNEL_UTILS_H

/*
 * Implement functions and other symbols needed by the base
 * kernel implementation of the parser.
 */

#ifndef bool
#define bool _Bool
#define true 1
#define false 0
#endif

struct linux_binprm;

#include <linux/path.h>
#include <linux/ml_sec/sandbox.h>

#define ARRAY_SIZE(arr)	(sizeof(arr) / sizeof((arr)[0]))

#define ERR_PTR(err)	((void *)((long)(err)))
#define PTR_ERR(ptr)	((long)(ptr))
#define IS_ERR(ptr)	((unsigned long)(ptr) > (unsigned long)(-1000))
#define FIX_ERR(err)	(((err) == -1) ? (-(int)errno) : (err))

#define kmalloc(size, flags)			safe_malloc(size)
#define kfree(mem)				free(mem)

#define fput(file)				close((long)file)

#define LOOKUP_FOLLOW	0

static inline void *safe_malloc(size_t size)
{
	void *p = malloc(size);
	if (!p) {
		fprintf(stderr, "Failed to allocate memory!\n");
		exit(1);
	}
	return p;
}

static inline loff_t vfs_llseek(struct file *file, loff_t offset, int whence)
{
	loff_t retval = lseek((long)file, offset, whence);
	return FIX_ERR(retval);
}

static inline int kernel_read(struct file *file, loff_t offset, char *addr, unsigned long count)
{
	int retval;
	lseek((long)file, offset, SEEK_SET);
	retval = read((long)file, addr, count);
	return FIX_ERR(retval);
}

#define current_cred()	NULL
#define path_put(path)	((void)path)

static inline void init_path(struct path *path)
{
	memset(path, 0, sizeof(struct path));
	path->dentry = &path->_dentry;
}

static inline int vfs_path_lookup(struct dentry *dentry, void *mnt, const char *name,
				  unsigned int flags, struct path *path)
{
	init_path(path);
	path->dentry->d_name.name = (const unsigned char *)name;
	return 0;
}

static inline struct file *dentry_open(const struct path *path, int flags, const void *cred)
{
	int fd = open((const char *)path->dentry->d_name.name, flags);
	return ERR_PTR(FIX_ERR(fd));
}

#endif /* __MLSEC_SBOX_SCRIPTS_KERNEL_UTILS_H */

