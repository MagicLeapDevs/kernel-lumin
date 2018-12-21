#include <linux/kernel.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/dcache.h>
#include <uapi/linux/limits.h>

#define GRSEC_PATH_MAX	(512)

void
gr_log_rwxmmap(struct file *file)
{
#ifdef CONFIG_GRKERNSEC_RWXMAP_LOG
	char pathbuf[GRSEC_PATH_MAX];
	char *filepath;
	if (file) {
		filepath = dentry_path_raw(file->f_path.dentry, pathbuf, sizeof(pathbuf));
	} else {
		filepath = "<anonymous mapping>";
	}
	printk(KERN_WARNING "RWX VIOLATION: %s (%d): mmap: %s\n", current->comm, task_tgid_nr(current), filepath);
#endif
	return;
}

void
gr_log_rwxmprotect(struct vm_area_struct *vma)
{
#ifdef CONFIG_GRKERNSEC_RWXMAP_LOG
	char pathbuf[GRSEC_PATH_MAX];
	char *str1 = NULL;
	if (vma->vm_file)
		str1 = dentry_path_raw(vma->vm_file->f_path.dentry, pathbuf, sizeof(pathbuf));
	else if (vma->vm_flags & (VM_GROWSDOWN | VM_GROWSUP))
		str1 = "<stack>";
	else if (vma->vm_start <= current->mm->brk &&
			 vma->vm_end >= current->mm->start_brk)
		str1 = "<heap>";
	else
		str1 = "<anonymous mapping>";
	printk(KERN_WARNING "RWX VIOLATION: %s (%d): mprotect: %s\n", current->comm, task_pid_nr(current), str1);
#endif
	return;
}