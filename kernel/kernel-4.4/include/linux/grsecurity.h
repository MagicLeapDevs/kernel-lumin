#ifndef GR_SECURITY_H
#define GR_SECURITY_H

void gr_log_rwxmmap(struct file *file);
void gr_log_rwxmprotect(struct vm_area_struct *vma);

#endif