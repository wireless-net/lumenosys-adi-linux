/*
 * Copyright 2004-2009 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#ifndef _BLACKFIN_BITOPS_H
#define _BLACKFIN_BITOPS_H

#include <linux/compiler.h>

#include <asm-generic/bitops/__ffs.h>
#include <asm-generic/bitops/ffz.h>
#include <asm-generic/bitops/fls.h>
#include <asm-generic/bitops/__fls.h>
#include <asm-generic/bitops/fls64.h>
#include <asm-generic/bitops/find.h>

#ifndef _LINUX_BITOPS_H
#error only <linux/bitops.h> can be included directly
#endif

#include <asm-generic/bitops/sched.h>
#include <asm-generic/bitops/ffs.h>
#include <asm-generic/bitops/lock.h>
#include <asm-generic/bitops/ext2-non-atomic.h>
#include <asm-generic/bitops/ext2-atomic.h>
#include <asm-generic/bitops/minix.h>

#ifndef CONFIG_SMP
#include <linux/irqflags.h>

/*
 * clear_bit may not imply a memory barrier
 */
#ifndef smp_mb__before_clear_bit
#define smp_mb__before_clear_bit()	smp_mb()
#define smp_mb__after_clear_bit()	smp_mb()
#endif
#include <asm-generic/bitops/atomic.h>
#include <asm-generic/bitops/non-atomic.h>
#else

#include <asm/byteorder.h>	/* swab32 */
#include <linux/linkage.h>

asmlinkage int __raw_bit_set_asm(volatile unsigned long *addr, int nr);

asmlinkage int __raw_bit_clear_asm(volatile unsigned long *addr, int nr);

asmlinkage int __raw_bit_toggle_asm(volatile unsigned long *addr, int nr);

asmlinkage int __raw_bit_test_set_asm(volatile unsigned long *addr, int nr);

asmlinkage int __raw_bit_test_clear_asm(volatile unsigned long *addr, int nr);

asmlinkage int __raw_bit_test_toggle_asm(volatile unsigned long *addr, int nr);

asmlinkage int __raw_bit_test_asm(const volatile unsigned long *addr, int nr);

static inline void set_bit(int nr, volatile unsigned long *addr)
{
	volatile unsigned long *a = addr + (nr >> 5);
	__raw_bit_set_asm(a, nr & 0x1f);
}

static inline void clear_bit(int nr, volatile unsigned long *addr)
{
	volatile unsigned long *a = addr + (nr >> 5);
	__raw_bit_clear_asm(a, nr & 0x1f);
}

static inline void change_bit(int nr, volatile unsigned long *addr)
{
	volatile unsigned long *a = addr + (nr >> 5);
	__raw_bit_toggle_asm(a, nr & 0x1f);
}

static inline int test_bit(int nr, const volatile unsigned long *addr)
{
	volatile const unsigned long *a = addr + (nr >> 5);
	return __raw_bit_test_asm(a, nr & 0x1f) != 0;
}

static inline int test_and_set_bit(int nr, volatile unsigned long *addr)
{
	volatile unsigned long *a = addr + (nr >> 5);
	return __raw_bit_test_set_asm(a, nr & 0x1f);
}

static inline int test_and_clear_bit(int nr, volatile unsigned long *addr)
{
	volatile unsigned long *a = addr + (nr >> 5);
	return __raw_bit_test_clear_asm(a, nr & 0x1f);
}

static inline int test_and_change_bit(int nr, volatile unsigned long *addr)
{
	volatile unsigned long *a = addr + (nr >> 5);
	return __raw_bit_test_toggle_asm(a, nr & 0x1f);
}

/*
 * clear_bit() doesn't provide any barrier for the compiler.
 */
#define smp_mb__before_clear_bit()	barrier()
#define smp_mb__after_clear_bit()	barrier()

/**
 * __set_bit - Set a bit in memory
 * @nr: the bit to set
 * @addr: the address to start counting from
 *
 * Unlike set_bit(), this function is non-atomic and may be reordered.
 * If it's called on the same region of memory simultaneously, the effect
 * may be that only one operation succeeds.
 */
static inline void __set_bit(int nr, volatile unsigned long *addr)
{
	unsigned long mask = BIT_MASK(nr);
	unsigned long *p = ((unsigned long *)addr) + BIT_WORD(nr);

	*p  |= mask;
}

static inline void __clear_bit(int nr, volatile unsigned long *addr)
{
	unsigned long mask = BIT_MASK(nr);
	unsigned long *p = ((unsigned long *)addr) + BIT_WORD(nr);

	*p &= ~mask;
}

/**
 * __change_bit - Toggle a bit in memory
 * @nr: the bit to change
 * @addr: the address to start counting from
 *
 * Unlike change_bit(), this function is non-atomic and may be reordered.
 * If it's called on the same region of memory simultaneously, the effect
 * may be that only one operation succeeds.
 */
static inline void __change_bit(int nr, volatile unsigned long *addr)
{
	unsigned long mask = BIT_MASK(nr);
	unsigned long *p = ((unsigned long *)addr) + BIT_WORD(nr);

	*p ^= mask;
}

/**
 * __test_and_set_bit - Set a bit and return its old value
 * @nr: Bit to set
 * @addr: Address to count from
 *
 * This operation is non-atomic and can be reordered.
 * If two examples of this operation race, one can appear to succeed
 * but actually fail.  You must protect multiple accesses with a lock.
 */
static inline int __test_and_set_bit(int nr, volatile unsigned long *addr)
{
	unsigned long mask = BIT_MASK(nr);
	unsigned long *p = ((unsigned long *)addr) + BIT_WORD(nr);
	unsigned long old = *p;

	*p = old | mask;
	return (old & mask) != 0;
}

/**
 * __test_and_clear_bit - Clear a bit and return its old value
 * @nr: Bit to clear
 * @addr: Address to count from
 *
 * This operation is non-atomic and can be reordered.
 * If two examples of this operation race, one can appear to succeed
 * but actually fail.  You must protect multiple accesses with a lock.
 */
static inline int __test_and_clear_bit(int nr, volatile unsigned long *addr)
{
	unsigned long mask = BIT_MASK(nr);
	unsigned long *p = ((unsigned long *)addr) + BIT_WORD(nr);
	unsigned long old = *p;

	*p = old & ~mask;
	return (old & mask) != 0;
}

/* WARNING: non atomic and it can be reordered! */
static inline int __test_and_change_bit(int nr,
					    volatile unsigned long *addr)
{
	unsigned long mask = BIT_MASK(nr);
	unsigned long *p = ((unsigned long *)addr) + BIT_WORD(nr);
	unsigned long old = *p;

	*p = old ^ mask;
	return (old & mask) != 0;
}

#endif /* CONFIG_SMP */

/*
 * hweightN: returns the hamming weight (i.e. the number
 * of bits set) of a N-bit word
 */

static inline unsigned int hweight32(unsigned int w)
{
	unsigned int res;

	__asm__ ("%0.l = ONES %1;"
		"%0 = %0.l (Z);"
		: "=d" (res) : "d" (w));
	return res;
}

static inline unsigned int hweight64(__u64 w)
{
	return hweight32((unsigned int)(w >> 32)) + hweight32((unsigned int)w);
}

static inline unsigned int hweight16(unsigned int w)
{
	return hweight32(w & 0xffff);
}

static inline unsigned int hweight8(unsigned int w)
{
	return hweight32(w & 0xff);
}

#endif				/* _BLACKFIN_BITOPS_H */
