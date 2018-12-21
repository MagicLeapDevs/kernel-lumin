
#ifndef __SYSCALL
#define __SYSCALL(nr, sym)
#endif

#undef _SYS_
#define _SYS_(nr, sym)	__SYSCALL(nr, sym)

#undef _NR_SYS_
#undef _NR_COMPAT_SYS_

#undef __X_NR_syscalls
#undef __X_NR_compat_syscalls

#ifndef __COMPAT_SYSCALL_NR

#ifndef _DEF_E_X_NR_syscalls
#define _DEF_E_X_NR_syscalls
#define _NR_SYS_(nr)	enum { _E_X_NR_syscalls = nr };
#else /* _DEF_E_X_NR_syscalls */
#define _NR_SYS_(nr)
#endif

#define __X_NR_syscalls	_E_X_NR_syscalls


#ifndef _DEF_E_X_NR_compat_syscalls
#define _DEF_E_X_NR_compat_syscalls
#define _NR_COMPAT_SYS_(nr)	enum { _E_X_NR_compat_syscalls = nr };
#else /* _DEF_E_X_NR_compat_syscalls */
#define _NR_COMPAT_SYS_(nr)
#endif

#define __X_NR_compat_syscalls	_E_X_NR_compat_syscalls


#include "syscall_table.inc"

#else /* __COMPAT_SYSCALL_NR */

#ifndef _DEF_E_X_NR_syscalls_c
#define _DEF_E_X_NR_syscalls_c
#define _NR_SYS_(nr)	enum { _E_X_NR_syscalls_c = nr };
#else /* _DEF_E_X_NR_syscalls_c */
#define _NR_SYS_(nr)
#endif

#define __X_NR_syscalls	_E_X_NR_syscalls_c


#ifndef _DEF_E_X_NR_compat_syscalls_c
#define _DEF_E_X_NR_compat_syscalls_c
#define _NR_COMPAT_SYS_(nr)	enum { _E_X_NR_compat_syscalls_c = nr };
#else /* _DEF_E_X_NR_compat_syscalls_c */
#define _NR_COMPAT_SYS_(nr)
#endif

#define __X_NR_compat_syscalls	_E_X_NR_compat_syscalls_c


#include "syscall_table_compat.inc"

#endif
