/*
 * Based on arch/arm/include/asm/barrier.h
 *
 * Copyright (C) 2012 ARM Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef __ASM_BARRIER_H
#define __ASM_BARRIER_H

#ifndef __ASSEMBLY__

#define __nops(n)	".rept	" #n "\nnop\n.endr\n"
#define nops(n)		asm volatile(__nops(n))

#define sev()		asm volatile("sev" : : : "memory")
#define wfe()		asm volatile("wfe" : : : "memory")
#define wfi()		asm volatile("wfi" : : : "memory")

#define isb()		asm volatile("isb" : : : "memory")
#define dmb(opt)	asm volatile("dmb " #opt : : : "memory")
#define dsb(opt)	asm volatile("dsb " #opt : : : "memory")

#define mb()		dsb(sy)
#define rmb()		dsb(ld)
#define wmb()		dsb(st)

#define dma_rmb()	dmb(oshld)
#define dma_wmb()	dmb(oshst)

#define __load_no_speculate_n(ptr, hi, failval, cmpptr, w, sz)	\
({									\
	typeof(*ptr)	__nln_val;					\
	typeof(*ptr)	__failval = 					\
		(typeof(*ptr))(unsigned long)(failval);			\
									\
	asm volatile (							\
	"	cmp	%[c], %[h]\n"				\
	"	b.cs	1f\n"						\
	"	ldr" #sz " %" #w "[v], %[p]\n"				\
	"1:	csel	%" #w "[v], %" #w "[v], %" #w "[f], cc\n"	\
	"	hint	#0x14 // CSDB\n"				\
	: [v] "=&r" (__nln_val)						\
	: [p] "m" (*(ptr)), [h] "r" (hi),			\
	  [f] "rZ" (__failval), [c] "r" (cmpptr)			\
	: "cc");							\
									\
	__nln_val;							\
})

#define __load_no_speculate(ptr, hi, failval, cmpptr)		\
({									\
	typeof(*(ptr)) __nl_val;					\
									\
	switch (sizeof(__nl_val)) {					\
	case 1:								\
		__nl_val = __load_no_speculate_n(ptr, hi, failval,	\
						 cmpptr, w, b);		\
		break;							\
	case 2:								\
		__nl_val = __load_no_speculate_n(ptr, hi, failval,	\
						 cmpptr, w, h);		\
		break;							\
	case 4:								\
		__nl_val = __load_no_speculate_n(ptr, hi, failval,	\
						 cmpptr, w, );		\
		break;							\
	case 8:								\
		__nl_val = __load_no_speculate_n(ptr, hi, failval,	\
						 cmpptr, x, );		\
		break;							\
	default:							\
		BUILD_BUG();						\
	}								\
									\
	__nl_val;							\
})

#define nospec_ptr(ptr, hi)						\
({									\
	typeof(ptr) __np_ptr = (ptr);					\
	__load_no_speculate(&__np_ptr, hi, 0, __np_ptr);		\
})


#define smp_mb()	dmb(ish)
#define smp_rmb()	dmb(ishld)
#define smp_wmb()	dmb(ishst)

#define smp_store_release(p, v)						\
do {									\
	union { typeof(*p) __val; char __c[1]; } __u =			\
		{ .__val = (__force typeof(*p)) (v) }; 			\
	compiletime_assert_atomic_type(*p);				\
	switch (sizeof(*p)) {						\
	case 1:								\
		asm volatile ("stlrb %w1, %0"				\
				: "=Q" (*p)				\
				: "r" (*(__u8 *)__u.__c)		\
				: "memory");				\
		break;							\
	case 2:								\
		asm volatile ("stlrh %w1, %0"				\
				: "=Q" (*p)				\
				: "r" (*(__u16 *)__u.__c)		\
				: "memory");				\
		break;							\
	case 4:								\
		asm volatile ("stlr %w1, %0"				\
				: "=Q" (*p)				\
				: "r" (*(__u32 *)__u.__c)		\
				: "memory");				\
		break;							\
	case 8:								\
		asm volatile ("stlr %1, %0"				\
				: "=Q" (*p)				\
				: "r" (*(__u64 *)__u.__c)		\
				: "memory");				\
		break;							\
	}								\
} while (0)

#define smp_load_acquire(p)						\
({									\
	union { typeof(*p) __val; char __c[1]; } __u;			\
	compiletime_assert_atomic_type(*p);				\
	switch (sizeof(*p)) {						\
	case 1:								\
		asm volatile ("ldarb %w0, %1"				\
			: "=r" (*(__u8 *)__u.__c)			\
			: "Q" (*p) : "memory");				\
		break;							\
	case 2:								\
		asm volatile ("ldarh %w0, %1"				\
			: "=r" (*(__u16 *)__u.__c)			\
			: "Q" (*p) : "memory");				\
		break;							\
	case 4:								\
		asm volatile ("ldar %w0, %1"				\
			: "=r" (*(__u32 *)__u.__c)			\
			: "Q" (*p) : "memory");				\
		break;							\
	case 8:								\
		asm volatile ("ldar %0, %1"				\
			: "=r" (*(__u64 *)__u.__c)			\
			: "Q" (*p) : "memory");				\
		break;							\
	}								\
	__u.__val;							\
})

#define read_barrier_depends()		do { } while(0)
#define smp_read_barrier_depends()	do { } while(0)

#define smp_store_mb(var, value)	do { WRITE_ONCE(var, value); smp_mb(); } while (0)
#define nop()		asm volatile("nop");

#define smp_mb__before_atomic()	smp_mb()
#define smp_mb__after_atomic()	smp_mb()

#define speculation_barrier()                                           \
        asm volatile(   "dsb sy\n"                                      \
	                "isb\n" : : : "memory")

#endif	/* __ASSEMBLY__ */

#endif	/* __ASM_BARRIER_H */
