#ifndef __ASM_STACK_POINTER_H
#define __ASM_STACK_POINTER_H

#ifndef __ASSEMBLY__
/*
 * how to get the current stack pointer from C
 */
register unsigned long current_stack_pointer asm ("sp");

#endif

#endif /* __ASM_STACK_POINTER_H */
