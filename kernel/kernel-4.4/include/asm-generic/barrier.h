/*
 * Generic barrier definitions, originally based on MN10300 definitions.
 *
 * It should be possible to use these on really simple architectures,
 * but it serves more as a starting point for new ports.
 *
 * Copyright (C) 2007 Red Hat, Inc. All Rights Reserved.
 * Written by David Howells (dhowells@redhat.com)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public Licence
 * as published by the Free Software Foundation; either version
 * 2 of the Licence, or (at your option) any later version.
 */
#ifndef __ASM_GENERIC_BARRIER_H
#define __ASM_GENERIC_BARRIER_H

#ifndef __ASSEMBLY__

#include <linux/compiler.h>

#ifndef nop
#define nop()	asm volatile ("nop")
#endif

/*
 * Force strict CPU ordering. And yes, this is required on UP too when we're
 * talking to devices.
 *
 * Fall back to compiler barriers if nothing better is provided.
 */

#ifndef mb
#define mb()	barrier()
#endif

#ifndef rmb
#define rmb()	mb()
#endif

#ifndef wmb
#define wmb()	mb()
#endif

#ifndef dma_rmb
#define dma_rmb()	rmb()
#endif

#ifndef dma_wmb
#define dma_wmb()	wmb()
#endif

#ifndef read_barrier_depends
#define read_barrier_depends()		do { } while (0)
#endif

/*
 * Inhibit subsequent speculative memory accesses.
 *
 * Architectures with a suitable memory barrier should provide an
 * implementation. This is non-portable, and generic code should use
 * nospec_ptr().
 */
#ifndef __nospec_barrier
#define __nospec_barrier()		do { } while (0)
#endif

/**
 * nospec_ptr() - Ensure a  pointer is bounded, even under speculation.
 *
 * @ptr: the pointer to test
 * @lo: the lower valid bound for @ptr, inclusive
 * @hi: the upper valid bound for @ptr, exclusive
 *
 * If @ptr falls in the interval [@lo, @i), returns @ptr, otherwise returns
 * NULL.
 *
 * Architectures which do not provide __nospec_barrier() should override this
 * to ensure that ptr falls in the [lo, hi) interval both under architectural
 * execution and under speculation, preventing propagation of an out-of-bounds
 * pointer to code which is speculatively executed.
 */
#ifndef nospec_ptr
#define nospec_ptr(ptr, hi)						\
({									\
	typeof (ptr) __ret;						\
	typeof (ptr) __ptr = (ptr);					\
	typeof (ptr) __hi = (hi);					\
									\
	__ret = (__ptr < __hi) ? __ptr : NULL;		\
									\
	__nospec_barrier();						\
									\
	__ret;								\
})
#endif

/**
 * nospec_array_ptr - Generate a pointer to an array element, ensuring the
 * pointer is bounded under speculation.
 *
 * @arr: the base of the array
 * @idx: the index of the element
 * @sz: the number of elements in the array
 *
 * If @idx falls in the interval [0, @sz), returns the pointer to @arr[@idx],
 * otherwise returns NULL.
 *
 * This is a wrapper around nospec_ptr(), provided for convenience.
 * Architectures should implement nospec_ptr() to ensure this is the case
 * under speculation.
 */
#define nospec_array_ptr(arr, idx, sz)					\
({									\
	typeof(*(arr)) *__arr = (arr);					\
	typeof(idx) __idx = (idx);					\
	typeof(sz) __sz = (sz);						\
									\
	nospec_ptr(__arr + __idx, __arr + __sz);			\
})

#undef __nospec_barrier

#ifdef CONFIG_SMP

#ifndef smp_mb
#define smp_mb()	mb()
#endif

#ifndef smp_rmb
#define smp_rmb()	rmb()
#endif

#ifndef smp_wmb
#define smp_wmb()	wmb()
#endif

#ifndef smp_read_barrier_depends
#define smp_read_barrier_depends()	read_barrier_depends()
#endif

#else	/* !CONFIG_SMP */

#ifndef smp_mb
#define smp_mb()	barrier()
#endif

#ifndef smp_rmb
#define smp_rmb()	barrier()
#endif

#ifndef smp_wmb
#define smp_wmb()	barrier()
#endif

#ifndef smp_read_barrier_depends
#define smp_read_barrier_depends()	do { } while (0)
#endif

#endif	/* CONFIG_SMP */

#ifndef smp_store_mb
#define smp_store_mb(var, value)  do { WRITE_ONCE(var, value); mb(); } while (0)
#endif

#ifndef smp_mb__before_atomic
#define smp_mb__before_atomic()	smp_mb()
#endif

#ifndef smp_mb__after_atomic
#define smp_mb__after_atomic()	smp_mb()
#endif

#ifndef smp_store_release
#define smp_store_release(p, v)						\
do {									\
	compiletime_assert_atomic_type(*p);				\
	smp_mb();							\
	WRITE_ONCE(*p, v);						\
} while (0)
#endif

#ifndef smp_load_acquire
#define smp_load_acquire(p)						\
({									\
	typeof(*p) ___p1 = READ_ONCE(*p);				\
	compiletime_assert_atomic_type(*p);				\
	smp_mb();							\
	___p1;								\
})
#endif

#endif /* !__ASSEMBLY__ */
#endif /* __ASM_GENERIC_BARRIER_H */
