#ifndef __MLSEC_SBOX_SCRIPTS_INCLUDE_HOST_LINUX_PATH_H
#define __MLSEC_SBOX_SCRIPTS_INCLUDE_HOST_LINUX_PATH_H

struct qstr {
	const unsigned char *name;
};

struct dentry {
	struct dentry *d_parent;
	struct qstr d_name;
};

struct path {
	void *mnt;
	struct dentry *dentry;

	struct dentry _dentry;
};

struct file {
	long fd;
};

#endif /* __MLSEC_SBOX_SCRIPTS_INCLUDE_HOST_LINUX_PATH_H */

