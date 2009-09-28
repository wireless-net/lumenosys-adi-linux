/*
 * File:         arch/blackfin/include/asm/cdef_misc.h
 * Based on:     arch/blackfin/mach-bf561/include/mach/cdefBF561.h
 * Author:
 *
 * Created:
 * Description:  C SYSTEM MMR REGISTER READ/WRITE THAT NEED DISABLE INTERRUPT
 *
 * Rev:
 *
 * Modified:
 *
 * Bugs:         Enter bugs at http://blackfin.uclinux.org/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; see the file COPYING.
 * If not, write to the Free Software Foundation,
 * 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#ifndef _CDEF_MISC_H
#define _CDEF_MISC_H

#ifndef __ASSEMBLY__
#include <asm/irq.h>
/* Writing to PLL_CTL initiates a PLL relock sequence. */
static __inline__ void bfin_write_PLL_CTL(unsigned int val)
{
	unsigned long flags = 0;
#ifdef SIC_IWR0
	unsigned long iwr0;
#ifdef SIC_IWR1
	unsigned long iwr1;
#ifdef SIC_IWR2
	unsigned long iwr2;
#endif
#endif
#else
	unsigned long iwr;
#endif

	if (val == bfin_read_PLL_CTL())
		return;

	local_irq_save_hw(flags);
	/* Enable the PLL Wakeup bit in SIC IWR */
#ifdef SIC_IWR0
	iwr0 = bfin_read32(SIC_IWR0);
#ifdef SIC_IWR1
	iwr1 = bfin_read32(SIC_IWR1);
#ifdef SIC_IWR2
	iwr2 = bfin_read32(SIC_IWR2);
	bfin_write32(SIC_IWR2, 0);
#endif
	bfin_write32(SIC_IWR1, 0);
#endif
	/* Only allow PPL Wakeup) */
	bfin_write32(SIC_IWR0, IWR_ENABLE(0));
#else
	iwr = bfin_read32(SIC_IWR);
#endif

	bfin_write16(PLL_CTL, val);
	SSYNC();
	asm("IDLE;");

#ifdef SIC_IWR0
	bfin_write32(SIC_IWR0, iwr0);
#ifdef SIC_IWR1
	bfin_write32(SIC_IWR1, iwr1);
#ifdef SIC_IWR2
	bfin_write32(SIC_IWR2, iwr2);
#endif
#endif
#else
	bfin_write32(SIC_IWR, iwr);
#endif
	local_irq_restore_hw(flags);
}

/* Writing to VR_CTL initiates a PLL relock sequence. */
static __inline__ void bfin_write_VR_CTL(unsigned int val)
{
	unsigned long flags = 0;
#ifdef SIC_IWR0
	unsigned long iwr0;
#ifdef SIC_IWR1
	unsigned long iwr1;
#ifdef SIC_IWR2
	unsigned long iwr2;
#endif
#endif
#else
	unsigned long iwr;
#endif

	if (val == bfin_read_VR_CTL())
		return;

	local_irq_save_hw(flags);
	/* Enable the PLL Wakeup bit in SIC IWR */
#ifdef SIC_IWR0
	iwr0 = bfin_read32(SIC_IWR0);
#ifdef SIC_IWR1
	iwr1 = bfin_read32(SIC_IWR1);
#ifdef SIC_IWR2
	iwr2 = bfin_read32(SIC_IWR2);
	bfin_write32(SIC_IWR2, 0);
#endif
	bfin_write32(SIC_IWR1, 0);
#endif
	/* Only allow PPL Wakeup) */
	bfin_write32(SIC_IWR0, IWR_ENABLE(0));
#else
	iwr = bfin_read32(SIC_IWR);
	bfin_write32(SIC_IWR, IWR_ENABLE(0));
#endif

	bfin_write16(VR_CTL, val);
	SSYNC();
	asm("IDLE;");

#ifdef SIC_IWR0
	bfin_write32(SIC_IWR0, iwr0);
#ifdef SIC_IWR1
	bfin_write32(SIC_IWR1, iwr1);
#ifdef SIC_IWR2
	bfin_write32(SIC_IWR2, iwr2);
#endif
#endif
#else
	bfin_write32(SIC_IWR, iwr);
#endif
	local_irq_restore_hw(flags);
}


#ifdef BF533_FAMILY

#if ANOMALY_05000311
#define BFIN_WRITE_FIO_FLAG(name) \
static inline void bfin_write_FIO_FLAG_##name(unsigned short val) \
{ \
	unsigned long flags; \
	local_irq_save_hw(flags); \
	bfin_write16(FIO_FLAG_##name, val); \
	bfin_read_CHIPID(); \
	local_irq_restore_hw(flags); \
}
BFIN_WRITE_FIO_FLAG(D)
BFIN_WRITE_FIO_FLAG(C)
BFIN_WRITE_FIO_FLAG(S)
BFIN_WRITE_FIO_FLAG(T)

#define BFIN_READ_FIO_FLAG(name) \
static inline u16 bfin_read_FIO_FLAG_##name(void) \
{ \
	unsigned long flags; \
	u16 ret; \
	local_irq_save_hw(flags); \
	ret = bfin_read16(FIO_FLAG_##name); \
	bfin_read_CHIPID(); \
	local_irq_restore_hw(flags); \
	return ret; \
}
BFIN_READ_FIO_FLAG(D)
BFIN_READ_FIO_FLAG(C)
BFIN_READ_FIO_FLAG(S)
BFIN_READ_FIO_FLAG(T)

#else
#define bfin_write_FIO_FLAG_D(val)           bfin_write16(FIO_FLAG_D, val)
#define bfin_write_FIO_FLAG_C(val)           bfin_write16(FIO_FLAG_C, val)
#define bfin_write_FIO_FLAG_S(val)           bfin_write16(FIO_FLAG_S, val)
#define bfin_write_FIO_FLAG_T(val)           bfin_write16(FIO_FLAG_T, val)
#define bfin_read_FIO_FLAG_T()               bfin_read16(FIO_FLAG_T)
#define bfin_read_FIO_FLAG_C()               bfin_read16(FIO_FLAG_C)
#define bfin_read_FIO_FLAG_S()               bfin_read16(FIO_FLAG_S)
#define bfin_read_FIO_FLAG_D()               bfin_read16(FIO_FLAG_D)
#endif

#endif

#endif
#endif