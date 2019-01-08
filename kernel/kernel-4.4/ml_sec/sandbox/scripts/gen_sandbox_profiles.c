/*
 * Copyright (c) 2017, Magic Leap, Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */


/*
 * This file is compiled only for the host, and is built into a small program,
 * that generates the source code for the hardcoded sandbox profiles and
 * syscall filters (as part of the kernel sandbox module build process).
 */

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <limits.h>
#include <ctype.h>
#include <errno.h>
#include <unistd.h>
#include <dirent.h>


/*****************************************************************************/
#include "kernel_utils.h"
/*
 * The following C files contain the code for parsing the
 * .sandbox.profile and .syscalls.profile files.
 * These files are shared between the kernel and the host app.
 *
 * API used:
 * - sbox_create_sandbox_filter_from_file()
 * - sbox_create_syscall_filter_from_file()
 */
#define HOSTPROG
#include "../dev/profile_file.c"
#include "../dev/sandbox_profile_file.c"
#include "../dev/syscalls_profile_file.c"
#include "../syscall_name.c"

#ifdef CONFIG_COMPAT
#undef __SYSCALL
#include "../syscall_name_compat.c"
#endif
/*****************************************************************************/


struct host_sandbox_profile {
	struct host_sandbox_profile *next;
	struct sandbox_profile profile;

	uint8_t syscall_filter[SBOX_SYS_FILTER_SIZE(__X_NR_syscalls)];
#ifdef CONFIG_COMPAT
	uint8_t syscall_filter_compat[SBOX_SYS_FILTER_SIZE(__X_NR_compat_syscalls)];
#endif
};

#define TAG_SIZE	(sizeof(uint32_t))

static const char *profiles_path;
static const char *output_path;
static int fd_sandbox_profiles = -1;
static int fd_syscall_filters = -1;
static int fd_syscall_filters_compat = -1;

static struct sandbox_profile *parse_sandbox_profile(const char *path,
						     const char *tagname);

static struct host_sandbox_profile *sandbox_profiles;

const struct sandbox_profile *sbox_lookup_profile(uint32_t tag)
{
	char tagname[TAG_SIZE + 1];
	struct host_sandbox_profile *profile;

	for (profile = sandbox_profiles; profile; profile = profile->next)
		if (profile->profile.profile_tag == tag)
			return &profile->profile;

	/* If the profile is not parsed already, try to parse it. */
	memcpy(tagname, (char *)&tag, TAG_SIZE);
	tagname[TAG_SIZE] = '\0';
	return parse_sandbox_profile(profiles_path, tagname);
}

/* Add a new sandbox_profile to the list, to be used with the given tag. */
static struct sandbox_profile *sbox_add_profile(uint32_t tag)
{
	struct host_sandbox_profile *profile;
	struct host_sandbox_profile **tail_ptr = &sandbox_profiles;

	while ((profile = *tail_ptr)) {
		if (profile->profile.profile_tag == tag)
			return NULL;

		tail_ptr = &profile->next;
	}

	profile = safe_malloc(sizeof(struct host_sandbox_profile));
	memset(profile, 0, sizeof(struct host_sandbox_profile));
	profile->profile.syscall_filter = profile->syscall_filter;
#ifdef CONFIG_COMPAT
	profile->profile.syscall_filter_compat = profile->syscall_filter_compat;
#endif

	*tail_ptr = profile;
	return &profile->profile;
}

/*
 * Lookup a profile with the exact same syscalls filters, as the given one.
 */
static struct sandbox_profile *lookup_similar_profile(const struct sandbox_profile *orig)
{
	struct host_sandbox_profile *profile;

	for (profile = sandbox_profiles; profile; profile = profile->next) {
		if (profile->profile.profile_tag == orig->profile_tag)
			break;

		if (!memcmp(orig->syscall_filter,
			    profile->syscall_filter,
			    SBOX_SYS_FILTER_SIZE(__X_NR_syscalls)) &&
		    !memcmp(orig->syscall_filter_compat,
			    profile->syscall_filter_compat,
			    SBOX_SYS_FILTER_SIZE(__X_NR_compat_syscalls)))
			return &profile->profile;
	}
	return NULL;
}

static char *join_path(const char *path, const char *name)
{
	size_t lenp;
	size_t lenn;
	char *str;

	if (!path || !*path)
		path = ".";

	lenp = strlen(path);
	lenn = strlen(name);

	if (path[lenp - 1] == '/')
		lenp--;

	str = safe_malloc(lenp + 1 + lenn + 1);
	memcpy(str, path, lenp);
	str[lenp] = '/';
	memcpy(str + lenp + 1, name, lenn + 1);
	return str;
}

/* Initializing the data of the sandbox_profile struct. */
static int construct_sandbox_profile_from_file(const char *filename, struct sandbox_profile *profile)
{
	int retval;
	int filter_flags;
	bool syscalls;
	struct path path;

	vfs_path_lookup(NULL, NULL, filename, 0, &path);

	/* Parse the sandbox runtime filter file */
	retval = sbox_create_sandbox_filter_from_file(&path);
	if (retval >= 0) {
		filter_flags = retval;
		retval = 0;
	} else {
		/*
		 * If parsing failed (ex. sandbox profile is missing)
		 * use default sandbox flags from _ML_ profile.
		 */
		const struct sandbox_profile *ml_profile;
		ml_profile = sbox_lookup_profile(SBOX_PROFILE_TAG__ML_);
		if (!ml_profile)
			return -ENOENT;
		filter_flags = (int)ml_profile->flags;
	}

	profile->flags = filter_flags;
	syscalls = (filter_flags & (SANDBOX_FILTER_SYSCALLS | SANDBOX_FILTER_SYSCALLS_PERMISSIVE)) != 0;

	/* Check for syscalls runtime filter file - and parse */
	if (syscalls) {
		int error = sbox_create_syscall_filter_from_file(&path, profile);
		if (error) {
			/* If we already had an error parsing the .sandbox.profile file. */
			if (retval)
				return retval;

			/* If a syscall filter for this profile is not found,
			 * then just default to ML internal. */
			if (error == -ENOENT) {
				const struct sandbox_profile *ml_profile;

				ml_profile = sbox_lookup_profile(SBOX_PROFILE_TAG__ML_);
				if (!ml_profile)
					return -ENOENT;

				memcpy((uint8_t *)profile->syscall_filter,
				       ml_profile->syscall_filter,
				       SBOX_SYS_FILTER_SIZE(__X_NR_syscalls));
#ifdef CONFIG_COMPAT
				memcpy((uint8_t *)profile->syscall_filter_compat,
				       ml_profile->syscall_filter_compat,
				       SBOX_SYS_FILTER_SIZE(__X_NR_compat_syscalls));
#endif
				error = 0;
			} else
				fprintf(stderr, "Failed to parse %s.syscalls.profile!\n", filename);
		}
		retval = error;
	}
	return retval;
}

static int create_file(const char *path, const char *name)
{
	int fd;
	char *filename = join_path(path, name);
	fd = open(filename, O_WRONLY | O_CREAT | O_TRUNC, 0644);
	if (fd == -1)
		fprintf(stderr, "Cannot create file '%s': %s\n", filename, strerror(errno));
	free(filename);
	return fd;
}

static int write_format(int fd, const char *format, ...)
{
	char buf[256];
	int retval;
	va_list args;
	va_start(args, format);
	retval = vsnprintf(buf, sizeof(buf), format, args);
	if ((size_t)retval >= sizeof(buf)) {
		fprintf(stderr, "Failed to format string for write (requires buffer size of %d).\n", retval);
		return -1;
	}
	retval = write(fd, buf, strlen(buf));
	va_end(args);
	return retval;
}

static void write_syscall_filter(int fd, const uint8_t *filter, size_t len, const char *tagname, const char *prefix)
{
	const uint8_t *end = filter + len;

	write_format(fd,
		     "static const uint8_t sbox%s_syscall_filter_mask_%s[%u] __attribute__((aligned(16))) = {%u",
		     prefix, tagname, (unsigned int)len, *filter);
	while (++filter < end)
		write_format(fd, ",%u", *filter);
	write_format(fd, "};\n");
}

static void write_sandbox_profile(int fd, int filter_flags, uint32_t tag,
				  const char *tagname, const char *sys_tagname)
{
	write_format(fd, "/* %s */ {.flags = 0, ", tagname); /* add a comment for readability */

	if ((filter_flags & SANDBOX_FILTER_SYSCALLS_PERMISSIVE) != 0)
		write_format(fd, "USAGE_INIT(true) ");
	else
		write_format(fd, "USAGE_INIT(false) ");

	if ((filter_flags & SANDBOX_FILTER_NO_NEW_PRIVS) != 0)
		write_format(fd, ".is_no_new_privs = true, ");

	if ((filter_flags & SANDBOX_FILTER_JAIL) != 0)
		write_format(fd, ".enforce_jail = true, ");

	if ((filter_flags & SANDBOX_FILTER_ENABLE_JIT) != 0)
		write_format(fd, ".enable_jit = true, ");

	if ((filter_flags & (SANDBOX_FILTER_SYSCALLS | SANDBOX_FILTER_SYSCALLS_PERMISSIVE)) != 0) {
		write_format(fd, ".syscall_filter = sbox_syscall_filter_mask_%s, ", sys_tagname);
#ifdef CONFIG_COMPAT
		write_format(fd, ".syscall_filter_compat = sbox_compat_syscall_filter_mask_%s, ", sys_tagname);
#endif

	if ((filter_flags & SANDBOX_FILTER_ALLOW_LOCAL_CONNECTIONS) != 0)
		write_format(fd, ".allow_local_connections = true, ");
	}
	write_format(fd, ".profile_tag = 0x%X},\n", tag);
}

/* The main function for parsing and writing a single sandbox profile. */
static struct sandbox_profile *parse_sandbox_profile(const char *path, const char *tagname)
{
	int error;
	uint32_t tag;
	struct sandbox_profile *profile;
	struct sandbox_profile *similar_profile;
	char *filename;

	memcpy((char *)&tag, tagname, TAG_SIZE);

	profile = sbox_add_profile(tag);
	if (!profile)
		return NULL;

	filename = join_path(path, tagname);
	error = construct_sandbox_profile_from_file(filename, profile);
	profile->profile_tag = tag;
	free(filename);

	if (error < 0)
		return NULL;

	/*
	 * To save (memory) space we can share the syscall filters between
	 * different sandbox profiles (if they are the same, of course).
	 */
	similar_profile = lookup_similar_profile(profile);
	if (similar_profile) {
		char similar_tagname[TAG_SIZE + 1];

		/* Prepare the tag-name of the similar profile. */
		memcpy(similar_tagname,
		       (char *) &similar_profile->profile_tag,
		       TAG_SIZE);
		similar_tagname[TAG_SIZE] = '\0';

		/* While writing the sandbox profile, use the new tag-name
		 * for the syscall filters. */
		write_sandbox_profile(fd_sandbox_profiles,
				      (int)profile->flags,
				      profile->profile_tag,
				      tagname, similar_tagname);
		return profile;
	}

	/*
	 * If we haven't found a similar profile, then write a completely new one.
	 */

	write_sandbox_profile(fd_sandbox_profiles,
			      (int)profile->flags,
			      profile->profile_tag,
			      tagname, tagname);

	write_syscall_filter(fd_syscall_filters,
			     profile->syscall_filter,
			     SBOX_SYS_FILTER_SIZE(__X_NR_syscalls),
			     tagname,
			     "");

#ifdef CONFIG_COMPAT
	write_syscall_filter(fd_syscall_filters_compat,
			     profile->syscall_filter_compat,
			     SBOX_SYS_FILTER_SIZE(__X_NR_compat_syscalls),
			     tagname,
			     "_compat");
#endif
	return profile;
}

/* Extract the tag name from the sandbox/syscalls profile file name. */
static size_t extract_profile_tag(const char *filename, char *tagname)
{
	const char *ext;

	ext = strstr(filename, "." SANDBOX_PROFILE_EXT);
	if (!ext)
		ext = strstr(filename, "." SYSCALLS_PROFILE_EXT);

	if ((filename + TAG_SIZE) != ext)
		return 0;

	memcpy(tagname, filename, TAG_SIZE);
	tagname[TAG_SIZE] = '\0';
	return TAG_SIZE;
}

/* List all tag names in the directory. */
static char *list_tagnames(DIR *dir, size_t *tags_count)
{
	struct dirent *de;
	size_t count;
	char tagname[TAG_SIZE + 1];
	char *tagnames; /* a NULL-separated string of all tag names. */
	char *curr_tagname;

	/*
	 * First pass is for counting the tag names...
	 */

	count = 0;
	while ((de = readdir(dir)) != NULL) {
		if (de->d_type != DT_REG)
			continue;

		if (!extract_profile_tag(de->d_name, tagname))
			continue;

		count++;
	}

	*tags_count = count;
	if (count == 0)
		return NULL;

	tagnames = safe_malloc(count * (TAG_SIZE + 1));
	curr_tagname = tagnames;

	/*
	 * Second pass is for extracting the tag names.
	 */

	rewinddir(dir);
	while ((de = readdir(dir)) != NULL) {
		if (de->d_type != DT_REG)
			continue;

		if (!extract_profile_tag(de->d_name, curr_tagname))
			continue;

		curr_tagname += TAG_SIZE + 1; /* Skip to the next tag name. */
	}

	/*
	 * The order if the listed file names is not deterministic.
	 * Sort it to make it so.
	 */
	qsort(tagnames, count, TAG_SIZE + 1, (int (*)(const void *, const void *))strcmp);
	return tagnames;
}

static void usage(char *argv[])
{
	const char *exec_name = strrchr(argv[0], '/');
	if (exec_name == NULL)
		exec_name = argv[0];

	fprintf(stderr, "Usage: %s <profiles_dir> <output_dir>\n", exec_name);
	exit(1);
}

int main(int argc, char *argv[])
{
	int retval = 1;
	DIR *profiles_dir;
	char *tagnames = NULL;
	char *curr_tagname;
	size_t tags_count = 0;
	struct sandbox_profile *profile;

	if (argc != 3)
		usage(argv);

	profiles_path = argv[1];
	output_path = argv[2];

	profiles_dir = opendir(profiles_path);
	if (!profiles_dir) {
		fprintf(stderr, "Cannot open profiles directory '%s': %s\n", profiles_path, strerror(errno));
		goto out;
	}

	fd_sandbox_profiles = create_file(output_path, "sandbox_profiles.inc");
	if (fd_sandbox_profiles == -1)
		goto out;

	fd_syscall_filters = create_file(output_path, "syscall_filters.c");
	if (fd_syscall_filters == -1)
		goto out;

#ifdef CONFIG_COMPAT
	fd_syscall_filters_compat = create_file(output_path, "syscall_filters_compat.c");
	if (fd_syscall_filters_compat == -1)
		goto out;
#endif

	/*
	 * OPTIMIZATION:
	 * The USER, _ML_, TOYS, and SAPP profiles are parsed first (in that
	 * order), as these profiles have the most extensive usage, and the,
	 * later, profile lookup function should encounter these first in the
	 * list.
	 *
	 * For example, if USER was somewhere in the middle, or even worse,
	 * the last one, then every time a user app is loaded, and the USER
	 * sandbox profile is requested (looked-up), then we would have to
	 * go through all the preceding profiles in the list, until we find it.
	 */
	profile = parse_sandbox_profile(profiles_path, "USER");
	if (!profile) { /* USER profile must exist! */
		fprintf(stderr, "Failed to parse 'USER' sandbox profile!\n");
		goto out;
	}

	profile = parse_sandbox_profile(profiles_path, "_ML_");
	if (!profile) { /* _ML_ profile must exist! */
		fprintf(stderr, "Failed to parse '_ML_' sandbox profile!\n");
		goto out;
	}

	retval = 0;

	parse_sandbox_profile(profiles_path, "TOYS");
	parse_sandbox_profile(profiles_path, "SAPP");

	tagnames = list_tagnames(profiles_dir, &tags_count);
	curr_tagname = tagnames;
	while (tags_count) {
		parse_sandbox_profile(profiles_path, curr_tagname);
		curr_tagname += TAG_SIZE + 1; /* Skip to the next tag name. */
		tags_count--;
	}

out:
	if (tagnames)
		free(tagnames);
	if (fd_syscall_filters_compat != -1)
		close(fd_syscall_filters_compat);
	if (fd_syscall_filters != -1)
		close(fd_syscall_filters);
	if (fd_sandbox_profiles != -1)
		close(fd_sandbox_profiles);
	if (profiles_dir)
		closedir(profiles_dir);
	return retval;
}
