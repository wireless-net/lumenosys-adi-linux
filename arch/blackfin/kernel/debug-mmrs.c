/*
 * debugfs interface to core/system MMRs
 *
 * Copyright 2007-2010 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later
 */

#include <linux/debugfs.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <asm/blackfin.h>
#include <asm/bfin_can.h>
#include <asm/bfin_ppi.h>

#define _d(name, bits, addr, perms) debugfs_create_x##bits(name, perms, parent, (u##bits *)addr)
#define d(name, bits, addr)         _d(name, bits, addr, S_IRUSR|S_IWUSR)
#define d_RO(name, bits, addr)      _d(name, bits, addr, S_IRUSR)
#define d_WO(name, bits, addr)      _d(name, bits, addr, S_IWUSR)

#define D_RO(name, bits) d_RO(#name, bits, name)
#define D_WO(name, bits) d_WO(#name, bits, name)
#define D32(name)        d(#name, 32, name)
#define D16(name)        d(#name, 16, name)

#define REGS_OFF(peri, mmr) offsetof(struct bfin_##peri##_regs, mmr)
#define __REGS(peri, sname, rname) \
	do { \
		struct bfin_##peri##_regs r; \
		void *addr = (void *)(base + REGS_OFF(peri, rname)); \
		strcpy(_buf, sname); \
		if (sizeof(r.rname) == 2) \
			debugfs_create_x16(buf, S_IRUSR|S_IWUSR, parent, addr); \
		else \
			debugfs_create_x32(buf, S_IRUSR|S_IWUSR, parent, addr); \
	} while (0)
#define REGS_STR_PFX(buf, pfx, num) \
	({ \
		buf + (num >= 0 ? \
			sprintf(buf, #pfx "%i_", num) : \
			sprintf(buf, #pfx "_")); \
	})

/*
 * Core registers (not memory mapped)
 */
extern u32 last_seqstat;

static int debug_cclk_get(void *data, u64 *val)
{
	*val = get_cclk();
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(fops_debug_cclk, debug_cclk_get, NULL, "0x%08llx\n");

static int debug_sclk_get(void *data, u64 *val)
{
	*val = get_sclk();
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(fops_debug_sclk, debug_sclk_get, NULL, "0x%08llx\n");

#define DEFINE_SYSREG(sr, pre, post) \
static int sysreg_##sr##_get(void *data, u64 *val) \
{ \
	unsigned long tmp; \
	pre; \
	__asm__ __volatile__("%0 = " #sr ";" : "=d"(tmp)); \
	*val = tmp; \
	return 0; \
} \
static int sysreg_##sr##_set(void *data, u64 val) \
{ \
	unsigned long tmp = val; \
	__asm__ __volatile__(#sr " = %0;" : : "d"(tmp)); \
	post; \
	return 0; \
} \
DEFINE_SIMPLE_ATTRIBUTE(fops_sysreg_##sr, sysreg_##sr##_get, sysreg_##sr##_set, "0x%08llx\n")

DEFINE_SYSREG(cycles, , );
DEFINE_SYSREG(cycles2, __asm__ __volatile__("%0 = cycles;" : "=d"(tmp)), );
DEFINE_SYSREG(emudat, , );
DEFINE_SYSREG(seqstat, , );
DEFINE_SYSREG(syscfg, , CSYNC());
#define D_SYSREG(sr) debugfs_create_file(#sr, S_IRUSR|S_IWUSR, parent, NULL, &fops_sysreg_##sr)

/*
 * CAN
 */
#define CAN_OFF(mmr)  REGS_OFF(can, mmr)
#define __CAN(uname, lname) __REGS(can, #uname, lname)
static void __init __maybe_unused
bfin_debug_mmrs_can(struct dentry *parent, unsigned long base, int num)
{
	static struct dentry *am, *mb;
	int i, j;
	char buf[32], *_buf = REGS_STR_PFX(buf, CAN, num);

	if (!am) {
		am = debugfs_create_dir("am", parent);
		mb = debugfs_create_dir("mb", parent);
	}

	__CAN(MC1, mc1);
	__CAN(MD1, md1);
	__CAN(TRS1, trs1);
	__CAN(TRR1, trr1);
	__CAN(TA1, ta1);
	__CAN(AA1, aa1);
	__CAN(RMP1, rmp1);
	__CAN(RML1, rml1);
	__CAN(MBTIF1, mbtif1);
	__CAN(MBRIF1, mbrif1);
	__CAN(MBIM1, mbim1);
	__CAN(RFH1, rfh1);
	__CAN(OPSS1, opss1);

	__CAN(MC2, mc2);
	__CAN(MD2, md2);
	__CAN(TRS2, trs2);
	__CAN(TRR2, trr2);
	__CAN(TA2, ta2);
	__CAN(AA2, aa2);
	__CAN(RMP2, rmp2);
	__CAN(RML2, rml2);
	__CAN(MBTIF2, mbtif2);
	__CAN(MBRIF2, mbrif2);
	__CAN(MBIM2, mbim2);
	__CAN(RFH2, rfh2);
	__CAN(OPSS2, opss2);

	__CAN(CLOCK, clock);
	__CAN(TIMING, timing);
	__CAN(DEBUG, debug);
	__CAN(STATUS, status);
	__CAN(CEC, cec);
	__CAN(GIS, gis);
	__CAN(GIM, gim);
	__CAN(GIF, gif);
	__CAN(CONTROL, control);
	__CAN(INTR, intr);
	__CAN(VERSION, version);
	__CAN(MBTD, mbtd);
	__CAN(EWR, ewr);
	__CAN(ESR, esr);
	/*__CAN(UCREG, ucreg); no longer exists */
	__CAN(UCCNT, uccnt);
	__CAN(UCRC, ucrc);
	__CAN(UCCNF, uccnf);
	__CAN(VERSION2, version2);

	for (i = 0; i < 32; ++i) {
		sprintf(_buf, "AM%02iL", i);
		debugfs_create_x16(buf, S_IRUSR|S_IWUSR, am,
			(u16 *)(base + CAN_OFF(msk[i].aml)));
		sprintf(_buf, "AM%02iH", i);
		debugfs_create_x16(buf, S_IRUSR|S_IWUSR, am,
			(u16 *)(base + CAN_OFF(msk[i].amh)));

		for (j = 0; j < 3; ++j) {
			sprintf(_buf, "MB%02i_DATA%i", i, j);
			debugfs_create_x16(buf, S_IRUSR|S_IWUSR, mb,
				(u16 *)(base + CAN_OFF(chl[i].data[j*2])));
		}
		sprintf(_buf, "MB%02i_LENGTH", i);
		debugfs_create_x16(buf, S_IRUSR|S_IWUSR, mb,
			(u16 *)(base + CAN_OFF(chl[i].dlc)));
		sprintf(_buf, "MB%02i_TIMESTAMP", i);
		debugfs_create_x16(buf, S_IRUSR|S_IWUSR, mb,
			(u16 *)(base + CAN_OFF(chl[i].tsv)));
		sprintf(_buf, "MB%02i_ID0", i);
		debugfs_create_x16(buf, S_IRUSR|S_IWUSR, mb,
			(u16 *)(base + CAN_OFF(chl[i].id0)));
		sprintf(_buf, "MB%02i_ID1", i);
		debugfs_create_x16(buf, S_IRUSR|S_IWUSR, mb,
			(u16 *)(base + CAN_OFF(chl[i].id1)));
	}
}
#define CAN(num) bfin_debug_mmrs_can(parent, CAN##num##_MC1, num)

/*
 * EPPI
 */
#define __EPPI(uname, lname) __REGS(eppi, #uname, lname)
static void __init __maybe_unused
bfin_debug_mmrs_eppi(struct dentry *parent, unsigned long base, int num)
{
	char buf[32], *_buf = REGS_STR_PFX(buf, EPPI, num);
	__EPPI(STATUS, status);
	__EPPI(HCOUNT, hcount);
	__EPPI(HDELAY, hdelay);
	__EPPI(VCOUNT, vcount);
	__EPPI(VDELAY, vdelay);
	__EPPI(FRAME, frame);
	__EPPI(LINE, line);
	__EPPI(CLKDIV, clkdiv);
	__EPPI(CONTROL, control);
	__EPPI(FS1W_HBL, fs1w_hbl);
	__EPPI(FS1P_AVPL, fs1p_avpl);
	__EPPI(FS2W_LVB, fs2w_lvb);
	__EPPI(FS2P_LAVF, fs2p_lavf);
	__EPPI(CLIP, clip);
}
#define EPPI(num) bfin_debug_mmrs_eppi(parent, EPPI##num##_STATUS, num)

/*
 * General Purpose Timers
 */
#define GPTIMER_OFF(mmr) (TIMER0_##mmr - TIMER0_CONFIG)
#define __GPTIMER(name) \
	do { \
		strcpy(_buf, #name); \
		debugfs_create_x16(buf, S_IRUSR|S_IWUSR, parent, (u16 *)(base + GPTIMER_OFF(name))); \
	} while (0)
static void __init __maybe_unused
bfin_debug_mmrs_gptimer(struct dentry *parent, unsigned long base, int num)
{
	char buf[32], *_buf = REGS_STR_PFX(buf, TIMER, num);
	__GPTIMER(CONFIG);
	__GPTIMER(COUNTER);
	__GPTIMER(PERIOD);
	__GPTIMER(WIDTH);
}
#define GPTIMER(num) bfin_debug_mmrs_gptimer(parent, TIMER##num##_CONFIG, num)

/*
 * PPI
 */
#define __PPI(uname, lname) __REGS(ppi, #uname, lname)
static void __init __maybe_unused
bfin_debug_mmrs_ppi(struct dentry *parent, unsigned long base, int num)
{
	char buf[32], *_buf = REGS_STR_PFX(buf, PPI, num);
	__PPI(CONTROL, control);
	__PPI(STATUS, status);
	__PPI(COUNT, count);
	__PPI(DELAY, delay);
	__PPI(FRAME, frame);
}
#define PPI(num) bfin_debug_mmrs_ppi(parent, PPI##num##_STATUS, num)

/*
 * SPORT
 */
static inline int sport_width(void *mmr)
{
	unsigned long lmmr = (unsigned long)mmr;
	if ((lmmr & 0xff) == 0x10)
		/* SPORT#_TX has 0x10 offset -> SPORT#_TCR2 has 0x04 offset */
		lmmr -= 0xc;
	else
		/* SPORT#_RX has 0x18 offset -> SPORT#_RCR2 has 0x24 offset */
		lmmr += 0xc;
	/* extract SLEN field from control register 2 and add 1 */
	return (bfin_read16(lmmr) & 0x1f) + 1;
}
static int sport_set(void *mmr, u64 val)
{
	unsigned long flags;
	local_irq_save(flags);
	if (sport_width(mmr) <= 16)
		bfin_write16(mmr, val);
	else
		bfin_write32(mmr, val);
	local_irq_restore(flags);
	return 0;
}
static int sport_get(void *mmr, u64 *val)
{
	unsigned long flags;
	local_irq_save(flags);
	if (sport_width(mmr) <= 16)
		*val = bfin_read16(mmr);
	else
		*val = bfin_read32(mmr);
	local_irq_restore(flags);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(fops_sport, sport_get, sport_set, "0x%08llx\n");
/*DEFINE_SIMPLE_ATTRIBUTE(fops_sport_ro, sport_get, NULL, "0x%08llx\n");*/
DEFINE_SIMPLE_ATTRIBUTE(fops_sport_wo, NULL, sport_set, "0x%08llx\n");
#define SPORT_OFF(mmr) (SPORT0_##mmr - SPORT0_TCR1)
#define _D_SPORT(name, perms, fops) \
	do { \
		strcpy(_buf, #name); \
		debugfs_create_file(buf, perms, parent, (void *)(base + SPORT_OFF(name)), fops); \
	} while (0)
#define __SPORT_RW(name) _D_SPORT(name, S_IRUSR|S_IWUSR, &fops_sport)
#define __SPORT_RO(name) _D_SPORT(name, S_IRUSR, &fops_sport_ro)
#define __SPORT_WO(name) _D_SPORT(name, S_IWUSR, &fops_sport_wo)
#define __SPORT(name, bits) \
	do { \
		strcpy(_buf, #name); \
		debugfs_create_x##bits(buf, S_IRUSR|S_IWUSR, parent, (u##bits *)(base + SPORT_OFF(name))); \
	} while (0)
static void __init __maybe_unused
bfin_debug_mmrs_sport(struct dentry *parent, unsigned long base, int num)
{
	char buf[32], *_buf = REGS_STR_PFX(buf, SPORT, num);
	__SPORT(CHNL, 16);
	__SPORT(MCMC1, 16);
	__SPORT(MCMC2, 16);
	__SPORT(MRCS0, 32);
	__SPORT(MRCS1, 32);
	__SPORT(MRCS2, 32);
	__SPORT(MRCS3, 32);
	__SPORT(MTCS0, 32);
	__SPORT(MTCS1, 32);
	__SPORT(MTCS2, 32);
	__SPORT(MTCS3, 32);
	__SPORT(RCLKDIV, 16);
	__SPORT(RCR1, 16);
	__SPORT(RCR2, 16);
	__SPORT(RFSDIV, 16);
	__SPORT_RW(RX);
	__SPORT(STAT, 16);
	__SPORT(TCLKDIV, 16);
	__SPORT(TCR1, 16);
	__SPORT(TCR2, 16);
	__SPORT(TFSDIV, 16);
	__SPORT_WO(TX);
}
#define SPORT(num) bfin_debug_mmrs_sport(parent, SPORT##num##_TCR1, num)

/*
 * The actual debugfs generation
 */
static struct dentry *debug_mmrs_dentry;

static int __init bfin_debug_mmrs_init(void)
{
	struct dentry *top, *parent;

	pr_info("Setting up Blackfin MMR debugfs\n");

	top = debugfs_create_dir("blackfin", NULL);
	if (top == NULL)
		return -1;

	parent = debugfs_create_dir("core_regs", top);
	debugfs_create_file("cclk", S_IRUSR, parent, NULL, &fops_debug_cclk);
	debugfs_create_file("sclk", S_IRUSR, parent, NULL, &fops_debug_sclk);
	debugfs_create_x32("last_seqstat", S_IRUSR, parent, &last_seqstat);
	D_SYSREG(cycles);
	D_SYSREG(cycles2);
	D_SYSREG(emudat);
	D_SYSREG(seqstat);
	D_SYSREG(syscfg);

	/* Core MMRs */
	parent = debugfs_create_dir("ctimer", top);
	D32(TCNTL);
	D32(TCOUNT);
	D32(TPERIOD);
	D32(TSCALE);

	parent = debugfs_create_dir("cec", top);
	D32(EVT0);
	D32(EVT1);
	D32(EVT2);
	D32(EVT3);
	D32(EVT4);
	D32(EVT5);
	D32(EVT6);
	D32(EVT7);
	D32(EVT8);
	D32(EVT9);
	D32(EVT10);
	D32(EVT11);
	D32(EVT12);
	D32(EVT13);
	D32(EVT14);
	D32(EVT15);
	D32(EVT_OVERRIDE);
	D32(IMASK);
	D32(IPEND);
	D32(ILAT);
	D32(IPRIO);

	parent = debugfs_create_dir("debug", top);
	D32(DBGSTAT);
	D32(DSPID);

	parent = debugfs_create_dir("mmu", top);
	D32(SRAM_BASE_ADDRESS);
	D32(DCPLB_ADDR0);
	D32(DCPLB_ADDR10);
	D32(DCPLB_ADDR11);
	D32(DCPLB_ADDR12);
	D32(DCPLB_ADDR13);
	D32(DCPLB_ADDR14);
	D32(DCPLB_ADDR15);
	D32(DCPLB_ADDR1);
	D32(DCPLB_ADDR2);
	D32(DCPLB_ADDR3);
	D32(DCPLB_ADDR4);
	D32(DCPLB_ADDR5);
	D32(DCPLB_ADDR6);
	D32(DCPLB_ADDR7);
	D32(DCPLB_ADDR8);
	D32(DCPLB_ADDR9);
	D32(DCPLB_DATA0);
	D32(DCPLB_DATA10);
	D32(DCPLB_DATA11);
	D32(DCPLB_DATA12);
	D32(DCPLB_DATA13);
	D32(DCPLB_DATA14);
	D32(DCPLB_DATA15);
	D32(DCPLB_DATA1);
	D32(DCPLB_DATA2);
	D32(DCPLB_DATA3);
	D32(DCPLB_DATA4);
	D32(DCPLB_DATA5);
	D32(DCPLB_DATA6);
	D32(DCPLB_DATA7);
	D32(DCPLB_DATA8);
	D32(DCPLB_DATA9);
	D32(DCPLB_FAULT_ADDR);
	D32(DCPLB_STATUS);
	D32(DMEM_CONTROL);
	D32(DTEST_COMMAND);
	D32(DTEST_DATA0);
	D32(DTEST_DATA1);

	D32(ICPLB_ADDR0);
	D32(ICPLB_ADDR1);
	D32(ICPLB_ADDR2);
	D32(ICPLB_ADDR3);
	D32(ICPLB_ADDR4);
	D32(ICPLB_ADDR5);
	D32(ICPLB_ADDR6);
	D32(ICPLB_ADDR7);
	D32(ICPLB_ADDR8);
	D32(ICPLB_ADDR9);
	D32(ICPLB_ADDR10);
	D32(ICPLB_ADDR11);
	D32(ICPLB_ADDR12);
	D32(ICPLB_ADDR13);
	D32(ICPLB_ADDR14);
	D32(ICPLB_ADDR15);
	D32(ICPLB_DATA0);
	D32(ICPLB_DATA1);
	D32(ICPLB_DATA2);
	D32(ICPLB_DATA3);
	D32(ICPLB_DATA4);
	D32(ICPLB_DATA5);
	D32(ICPLB_DATA6);
	D32(ICPLB_DATA7);
	D32(ICPLB_DATA8);
	D32(ICPLB_DATA9);
	D32(ICPLB_DATA10);
	D32(ICPLB_DATA11);
	D32(ICPLB_DATA12);
	D32(ICPLB_DATA13);
	D32(ICPLB_DATA14);
	D32(ICPLB_DATA15);
	D32(ICPLB_FAULT_ADDR);
	D32(ICPLB_STATUS);
	D32(IMEM_CONTROL);
	if (!ANOMALY_05000481) {
		D32(ITEST_COMMAND);
		D32(ITEST_DATA0);
		D32(ITEST_DATA1);
	}

	parent = debugfs_create_dir("perf", top);
	D32(PFCNTR0);
	D32(PFCNTR1);
	D32(PFCTL);

	parent = debugfs_create_dir("trace", top);
	D32(TBUF);
	D32(TBUFCTL);
	D32(TBUFSTAT);

	parent = debugfs_create_dir("watchpoint", top);
	D32(WPIACTL);
	D32(WPIA0);
	D32(WPIA1);
	D32(WPIA2);
	D32(WPIA3);
	D32(WPIA4);
	D32(WPIA5);
	D32(WPIACNT0);
	D32(WPIACNT1);
	D32(WPIACNT2);
	D32(WPIACNT3);
	D32(WPIACNT4);
	D32(WPIACNT5);
	D32(WPDACTL);
	D32(WPDA0);
	D32(WPDA1);
	D32(WPDACNT0);
	D32(WPDACNT1);
	D32(WPSTAT);

	/* System MMRs */
#ifdef ATAPI_CONTROL
	parent = debugfs_create_dir("atapi", top);
	D16(ATAPI_CONTROL);
	D16(ATAPI_DEV_ADDR);
	D16(ATAPI_DEV_RXBUF);
	D16(ATAPI_DEV_TXBUF);
	D16(ATAPI_DMA_TFRCNT);
	D16(ATAPI_INT_MASK);
	D16(ATAPI_INT_STATUS);
	D16(ATAPI_LINE_STATUS);
	D16(ATAPI_MULTI_TIM_0);
	D16(ATAPI_MULTI_TIM_1);
	D16(ATAPI_MULTI_TIM_2);
	D16(ATAPI_PIO_TFRCNT);
	D16(ATAPI_PIO_TIM_0);
	D16(ATAPI_PIO_TIM_1);
	D16(ATAPI_REG_TIM_0);
	D16(ATAPI_SM_STATE);
	D16(ATAPI_STATUS);
	D16(ATAPI_TERMINATE);
	D16(ATAPI_UDMAOUT_TFRCNT);
	D16(ATAPI_ULTRA_TIM_0);
	D16(ATAPI_ULTRA_TIM_1);
	D16(ATAPI_ULTRA_TIM_2);
	D16(ATAPI_ULTRA_TIM_3);
	D16(ATAPI_UMAIN_TFRCNT);
	D16(ATAPI_XFER_LEN);
#endif

#if defined(CAN_MC1) || defined(CAN0_MC1) || defined(CAN1_MC1)
	parent = debugfs_create_dir("can", top);
# ifdef CAN_MC1
	bfin_debug_mmrs_can(parent, CAN_MC1, -1)
# endif
# ifdef CAN0_MC1
	CAN(0);
# endif
# ifdef CAN1_MC1
	CAN(1);
# endif
#endif

#ifdef CNT_COMMAND
	parent = debugfs_create_dir("counter", top);
	D16(CNT_COMMAND);
	D16(CNT_CONFIG);
	D32(CNT_COUNTER);
	D16(CNT_DEBOUNCE);
	D16(CNT_IMASK);
	D32(CNT_MAX);
	D32(CNT_MIN);
	D16(CNT_STATUS);
#endif

	parent = debugfs_create_dir("ebiu_amc", top);
	D32(EBIU_AMBCTL0);
	D32(EBIU_AMBCTL1);
	D16(EBIU_AMGCTL);
#ifdef EBIU_MBSCTL
	D16(EBIU_MBSCTL);
	D32(EBIU_ARBSTAT);
	D32(EBIU_MODE);
	D16(EBIU_FCTL);
#endif

#ifdef EBIU_SDGCTL
	parent = debugfs_create_dir("ebiu_sdram", top);
# ifdef __ADSPBF561__
	D32(EBIU_SDBCTL);
# else
	D16(EBIU_SDBCTL);
# endif
	D32(EBIU_SDGCTL);
	D16(EBIU_SDRRC);
	D16(EBIU_SDSTAT);
#endif

#ifdef EBIU_DDRACCT
	parent = debugfs_create_dir("ebiu_ddr", top);
	D32(EBIU_DDRACCT);
	D32(EBIU_DDRARCT);
	D32(EBIU_DDRBRC0);
	D32(EBIU_DDRBRC1);
	D32(EBIU_DDRBRC2);
	D32(EBIU_DDRBRC3);
	D32(EBIU_DDRBRC4);
	D32(EBIU_DDRBRC5);
	D32(EBIU_DDRBRC6);
	D32(EBIU_DDRBRC7);
	D32(EBIU_DDRBWC0);
	D32(EBIU_DDRBWC1);
	D32(EBIU_DDRBWC2);
	D32(EBIU_DDRBWC3);
	D32(EBIU_DDRBWC4);
	D32(EBIU_DDRBWC5);
	D32(EBIU_DDRBWC6);
	D32(EBIU_DDRBWC7);
	D32(EBIU_DDRCTL0);
	D32(EBIU_DDRCTL1);
	D32(EBIU_DDRCTL2);
	D32(EBIU_DDRCTL3);
	D32(EBIU_DDRGC0);
	D32(EBIU_DDRGC1);
	D32(EBIU_DDRGC2);
	D32(EBIU_DDRGC3);
	D32(EBIU_DDRMCCL);
	D32(EBIU_DDRMCEN);
	D32(EBIU_DDRQUE);
	D32(EBIU_DDRTACT);
	D32(EBIU_ERRADD);
	D16(EBIU_ERRMST);
	D16(EBIU_RSTCTL);
#endif

#ifdef EMAC_ADDRHI
	parent = debugfs_create_dir("emac", top);
	D32(EMAC_ADDRHI);
	D32(EMAC_ADDRLO);
	D32(EMAC_FLC);
	D32(EMAC_HASHHI);
	D32(EMAC_HASHLO);
	D32(EMAC_MMC_CTL);
	D32(EMAC_MMC_RIRQE);
	D32(EMAC_MMC_RIRQS);
	D32(EMAC_MMC_TIRQE);
	D32(EMAC_MMC_TIRQS);
	D32(EMAC_OPMODE);
	D32(EMAC_RXC_ALIGN);
	D32(EMAC_RXC_ALLFRM);
	D32(EMAC_RXC_ALLOCT);
	D32(EMAC_RXC_BROAD);
	D32(EMAC_RXC_DMAOVF);
	D32(EMAC_RXC_EQ64);
	D32(EMAC_RXC_FCS);
	D32(EMAC_RXC_GE1024);
	D32(EMAC_RXC_LNERRI);
	D32(EMAC_RXC_LNERRO);
	D32(EMAC_RXC_LONG);
	D32(EMAC_RXC_LT1024);
	D32(EMAC_RXC_LT128);
	D32(EMAC_RXC_LT256);
	D32(EMAC_RXC_LT512);
	D32(EMAC_RXC_MACCTL);
	D32(EMAC_RXC_MULTI);
	D32(EMAC_RXC_OCTET);
	D32(EMAC_RXC_OK);
	D32(EMAC_RXC_OPCODE);
	D32(EMAC_RXC_PAUSE);
	D32(EMAC_RXC_SHORT);
	D32(EMAC_RXC_TYPED);
	D32(EMAC_RXC_UNICST);
	D32(EMAC_RX_IRQE);
	D32(EMAC_RX_STAT);
	D32(EMAC_RX_STKY);
	D32(EMAC_STAADD);
	D32(EMAC_STADAT);
	D32(EMAC_SYSCTL);
	D32(EMAC_SYSTAT);
	D32(EMAC_TXC_1COL);
	D32(EMAC_TXC_ABORT);
	D32(EMAC_TXC_ALLFRM);
	D32(EMAC_TXC_ALLOCT);
	D32(EMAC_TXC_BROAD);
	D32(EMAC_TXC_CRSERR);
	D32(EMAC_TXC_DEFER);
	D32(EMAC_TXC_DMAUND);
	D32(EMAC_TXC_EQ64);
	D32(EMAC_TXC_GE1024);
	D32(EMAC_TXC_GT1COL);
	D32(EMAC_TXC_LATECL);
	D32(EMAC_TXC_LT1024);
	D32(EMAC_TXC_LT128);
	D32(EMAC_TXC_LT256);
	D32(EMAC_TXC_LT512);
	D32(EMAC_TXC_MACCTL);
	D32(EMAC_TXC_MULTI);
	D32(EMAC_TXC_OCTET);
	D32(EMAC_TXC_OK);
	D32(EMAC_TXC_UNICST);
	D32(EMAC_TXC_XS_COL);
	D32(EMAC_TXC_XS_DFR);
	D32(EMAC_TX_IRQE);
	D32(EMAC_TX_STAT);
	D32(EMAC_TX_STKY);
	D32(EMAC_VLAN1);
	D32(EMAC_VLAN2);
	D32(EMAC_WKUP_CTL);
	D32(EMAC_WKUP_FFCMD);
	D32(EMAC_WKUP_FFCRC0);
	D32(EMAC_WKUP_FFCRC1);
	D32(EMAC_WKUP_FFMSK0);
	D32(EMAC_WKUP_FFMSK1);
	D32(EMAC_WKUP_FFMSK2);
	D32(EMAC_WKUP_FFMSK3);
	D32(EMAC_WKUP_FFOFF);
# ifdef EMAC_PTP_ACCR
	D32(EMAC_PTP_ACCR);
	D32(EMAC_PTP_ADDEND);
	D32(EMAC_PTP_ALARMHI);
	D32(EMAC_PTP_ALARMLO);
	D16(EMAC_PTP_CTL);
	D32(EMAC_PTP_FOFF);
	D32(EMAC_PTP_FV1);
	D32(EMAC_PTP_FV2);
	D32(EMAC_PTP_FV3);
	D16(EMAC_PTP_ID_OFF);
	D32(EMAC_PTP_ID_SNAP);
	D16(EMAC_PTP_IE);
	D16(EMAC_PTP_ISTAT);
	D32(EMAC_PTP_OFFSET);
	D32(EMAC_PTP_PPS_PERIOD);
	D32(EMAC_PTP_PPS_STARTHI);
	D32(EMAC_PTP_PPS_STARTLO);
	D32(EMAC_PTP_RXSNAPHI);
	D32(EMAC_PTP_RXSNAPLO);
	D32(EMAC_PTP_TIMEHI);
	D32(EMAC_PTP_TIMELO);
	D32(EMAC_PTP_TXSNAPHI);
	D32(EMAC_PTP_TXSNAPLO);
# endif
#endif

#if defined(EPPI0_STATUS) || defined(EPPI1_STATUS) || defined(EPPI2_STATUS)
	parent = debugfs_create_dir("eppi", top);
# ifdef EPPI0_STATUS
	EPPI(0);
# endif
# ifdef EPPI1_STATUS
	EPPI(1);
# endif
# ifdef EPPI2_STATUS
	EPPI(2);
# endif
#endif

	parent = debugfs_create_dir("gptimer", top);
#ifdef TIMER_DISABLE
	D16(TIMER_DISABLE);
	D16(TIMER_ENABLE);
	D32(TIMER_STATUS);
#endif
#ifdef TIMER_DISABLE0
	D16(TIMER_DISABLE0);
	D16(TIMER_ENABLE0);
	D32(TIMER_STATUS0);
#endif
#ifdef TIMER_DISABLE1
	D16(TIMER_DISABLE1);
	D16(TIMER_ENABLE1);
	D32(TIMER_STATUS1);
#endif
	/* XXX: Should convert BF561 MMR names */
#ifdef TMRS4_DISABLE
	D16(TMRS4_DISABLE);
	D16(TMRS4_ENABLE);
	D32(TMRS4_STATUS);
	D16(TMRS8_DISABLE);
	D16(TMRS8_ENABLE);
	D32(TMRS8_STATUS);
#endif
	GPTIMER(0);
	GPTIMER(1);
	GPTIMER(2);
#ifdef TIMER3_CONFIG
	GPTIMER(3);
	GPTIMER(4);
	GPTIMER(5);
	GPTIMER(6);
	GPTIMER(7);
#endif
#ifdef TIMER8_CONFIG
	GPTIMER(8);
	GPTIMER(9);
	GPTIMER(10);
#endif
#ifdef TIMER11_CONFIG
	GPTIMER(11);
#endif

#ifdef HOST_CONTROL
	parent = debugfs_create_dir("hostdp", top);
	D16(HOST_CONTROL);
	D16(HOST_STATUS);
	D16(HOST_TIMEOUT);
#endif

#ifdef KPAD_CTL
	parent = debugfs_create_dir("keypad", top);
	D16(KPAD_CTL);
	D16(KPAD_PRESCALE);
	D16(KPAD_MSEL);
	D16(KPAD_ROWCOL);
	D16(KPAD_STAT);
	D16(KPAD_SOFTEVAL);
#endif

#ifdef MXVR_CONFIG
	parent = debugfs_create_dir("mxvr", top);
	D16(MXVR_CONFIG);
# ifdef MXVR_PLL_CTL_0
	D32(MXVR_PLL_CTL_0);
# endif
	D32(MXVR_STATE_0);
	D32(MXVR_STATE_1);
	D32(MXVR_INT_STAT_0);
	D32(MXVR_INT_STAT_1);
	D32(MXVR_INT_EN_0);
	D32(MXVR_INT_EN_1);
	D16(MXVR_POSITION);
	D16(MXVR_MAX_POSITION);
	D16(MXVR_DELAY);
	D16(MXVR_MAX_DELAY);
	D32(MXVR_LADDR);
	D16(MXVR_GADDR);
	D32(MXVR_AADDR);
	D32(MXVR_ALLOC_0);
	D32(MXVR_ALLOC_1);
	D32(MXVR_ALLOC_2);
	D32(MXVR_ALLOC_3);
	D32(MXVR_ALLOC_4);
	D32(MXVR_ALLOC_5);
	D32(MXVR_ALLOC_6);
	D32(MXVR_ALLOC_7);
	D32(MXVR_ALLOC_8);
	D32(MXVR_ALLOC_9);
	D32(MXVR_ALLOC_10);
	D32(MXVR_ALLOC_11);
	D32(MXVR_ALLOC_12);
	D32(MXVR_ALLOC_13);
	D32(MXVR_ALLOC_14);
	D32(MXVR_SYNC_LCHAN_0);
	D32(MXVR_SYNC_LCHAN_1);
	D32(MXVR_SYNC_LCHAN_2);
	D32(MXVR_SYNC_LCHAN_3);
	D32(MXVR_SYNC_LCHAN_4);
	D32(MXVR_SYNC_LCHAN_5);
	D32(MXVR_SYNC_LCHAN_6);
	D32(MXVR_SYNC_LCHAN_7);
	D32(MXVR_DMA0_CONFIG);
	D32(MXVR_DMA0_START_ADDR);
	D16(MXVR_DMA0_COUNT);
	D32(MXVR_DMA0_CURR_ADDR);
	D16(MXVR_DMA0_CURR_COUNT);
	D32(MXVR_DMA1_CONFIG);
	D32(MXVR_DMA1_START_ADDR);
	D16(MXVR_DMA1_COUNT);
	D32(MXVR_DMA1_CURR_ADDR);
	D16(MXVR_DMA1_CURR_COUNT);
	D32(MXVR_DMA2_CONFIG);
	D32(MXVR_DMA2_START_ADDR);
	D16(MXVR_DMA2_COUNT);
	D32(MXVR_DMA2_CURR_ADDR);
	D16(MXVR_DMA2_CURR_COUNT);
	D32(MXVR_DMA3_CONFIG);
	D32(MXVR_DMA3_START_ADDR);
	D16(MXVR_DMA3_COUNT);
	D32(MXVR_DMA3_CURR_ADDR);
	D16(MXVR_DMA3_CURR_COUNT);
	D32(MXVR_DMA4_CONFIG);
	D32(MXVR_DMA4_START_ADDR);
	D16(MXVR_DMA4_COUNT);
	D32(MXVR_DMA4_CURR_ADDR);
	D16(MXVR_DMA4_CURR_COUNT);
	D32(MXVR_DMA5_CONFIG);
	D32(MXVR_DMA5_START_ADDR);
	D16(MXVR_DMA5_COUNT);
	D32(MXVR_DMA5_CURR_ADDR);
	D16(MXVR_DMA5_CURR_COUNT);
	D32(MXVR_DMA6_CONFIG);
	D32(MXVR_DMA6_START_ADDR);
	D16(MXVR_DMA6_COUNT);
	D32(MXVR_DMA6_CURR_ADDR);
	D16(MXVR_DMA6_CURR_COUNT);
	D32(MXVR_DMA7_CONFIG);
	D32(MXVR_DMA7_START_ADDR);
	D16(MXVR_DMA7_COUNT);
	D32(MXVR_DMA7_CURR_ADDR);
	D16(MXVR_DMA7_CURR_COUNT);
	D16(MXVR_AP_CTL);
	D32(MXVR_APRB_START_ADDR);
	D32(MXVR_APRB_CURR_ADDR);
	D32(MXVR_APTB_START_ADDR);
	D32(MXVR_APTB_CURR_ADDR);
	D32(MXVR_CM_CTL);
	D32(MXVR_CMRB_START_ADDR);
	D32(MXVR_CMRB_CURR_ADDR);
	D32(MXVR_CMTB_START_ADDR);
	D32(MXVR_CMTB_CURR_ADDR);
	D32(MXVR_RRDB_START_ADDR);
	D32(MXVR_RRDB_CURR_ADDR);
	D32(MXVR_PAT_DATA_0);
	D32(MXVR_PAT_EN_0);
	D32(MXVR_PAT_DATA_1);
	D32(MXVR_PAT_EN_1);
	D16(MXVR_FRAME_CNT_0);
	D16(MXVR_FRAME_CNT_1);
	D32(MXVR_ROUTING_0);
	D32(MXVR_ROUTING_1);
	D32(MXVR_ROUTING_2);
	D32(MXVR_ROUTING_3);
	D32(MXVR_ROUTING_4);
	D32(MXVR_ROUTING_5);
	D32(MXVR_ROUTING_6);
	D32(MXVR_ROUTING_7);
	D32(MXVR_ROUTING_8);
	D32(MXVR_ROUTING_9);
	D32(MXVR_ROUTING_10);
	D32(MXVR_ROUTING_11);
	D32(MXVR_ROUTING_12);
	D32(MXVR_ROUTING_13);
	D32(MXVR_ROUTING_14);
# ifdef MXVR_PLL_CTL_1
	D32(MXVR_PLL_CTL_1);
# endif
	D16(MXVR_BLOCK_CNT);
# ifdef MXVR_CLK_CTL
	D32(MXVR_CLK_CTL);
# endif
# ifdef MXVR_CDRPLL_CTL
	D32(MXVR_CDRPLL_CTL);
# endif
# ifdef MXVR_FMPLL_CTL
	D32(MXVR_FMPLL_CTL);
# endif
# ifdef MXVR_PIN_CTL
	D16(MXVR_PIN_CTL);
# endif
# ifdef MXVR_SCLK_CNT
	D16(MXVR_SCLK_CNT);
# endif
#endif

#ifdef NFC_ADDR
	parent = debugfs_create_dir("nfc", top);
	D_WO(NFC_ADDR, 16);
	D_WO(NFC_CMD, 16);
	D_RO(NFC_COUNT, 16);
	D16(NFC_CTL);
	D_WO(NFC_DATA_RD, 16);
	D_WO(NFC_DATA_WR, 16);
	D_RO(NFC_ECC0, 16);
	D_RO(NFC_ECC1, 16);
	D_RO(NFC_ECC2, 16);
	D_RO(NFC_ECC3, 16);
	D16(NFC_IRQMASK);
	D16(NFC_IRQSTAT);
	D_WO(NFC_PGCTL, 16);
	D_RO(NFC_READ, 16);
	D16(NFC_RST);
	D_RO(NFC_STAT, 16);
#endif

#ifdef OTP_CONTROL
	parent = debugfs_create_dir("otp", top);
	D16(OTP_CONTROL);
	D16(OTP_BEN);
	D16(OTP_STATUS);
	D32(OTP_TIMING);
	D32(OTP_DATA0);
	D32(OTP_DATA1);
	D32(OTP_DATA2);
	D32(OTP_DATA3);
#endif

#ifdef PIXC_CTL
	parent = debugfs_create_dir("pixc", top);
	D16(PIXC_CTL);
	D16(PIXC_PPL);
	D16(PIXC_LPF);
	D16(PIXC_AHSTART);
	D16(PIXC_AHEND);
	D16(PIXC_AVSTART);
	D16(PIXC_AVEND);
	D16(PIXC_ATRANSP);
	D16(PIXC_BHSTART);
	D16(PIXC_BHEND);
	D16(PIXC_BVSTART);
	D16(PIXC_BVEND);
	D16(PIXC_BTRANSP);
	D16(PIXC_INTRSTAT);
	D32(PIXC_RYCON);
	D32(PIXC_GUCON);
	D32(PIXC_BVCON);
	D32(PIXC_CCBIAS);
	D32(PIXC_TC);
#endif

	parent = debugfs_create_dir("pll", top);
	D16(PLL_CTL);
	D16(PLL_DIV);
	D16(PLL_LOCKCNT);
	D16(PLL_STAT);
	D16(VR_CTL);
	D32(CHIPID);	/* it's part of this hardware block */

#if defined(PPI_STATUS) || defined(PPI0_STATUS) || defined(PPI1_STATUS)
	parent = debugfs_create_dir("ppi", top);
# ifdef PPI_STATUS
	bfin_debug_mmrs_ppi(parent, PPI_STATUS, -1)
# endif
# ifdef PPI0_STATUS
	PPI(0);
# endif
# ifdef PPI1_STATUS
	PPI(1);
# endif
#endif

#ifdef PWM_CTRL
	parent = debugfs_create_dir("pwm", top);
	D16(PWM_CTRL);
	D16(PWM_STAT);
	D16(PWM_TM);
	D16(PWM_DT);
	D16(PWM_GATE);
	D16(PWM_CHA);
	D16(PWM_CHB);
	D16(PWM_CHC);
	D16(PWM_SEG);
	D16(PWM_SYNCWT);
	D16(PWM_CHAL);
	D16(PWM_CHBL);
	D16(PWM_CHCL);
	D16(PWM_LSI);
	D16(PWM_STAT2);
#endif

#ifdef RSI_CONFIG
	parent = debugfs_create_dir("rsi", top);
	D32(RSI_ARGUMENT);
	D16(RSI_CEATA_CONTROL);
	D16(RSI_CLK_CONTROL);
	D16(RSI_COMMAND);
	D16(RSI_CONFIG);
	D16(RSI_DATA_CNT);
	D16(RSI_DATA_CONTROL);
	D16(RSI_DATA_LGTH);
	D32(RSI_DATA_TIMER);
	D16(RSI_EMASK);
	D16(RSI_ESTAT);
	D32(RSI_FIFO);
	D16(RSI_FIFO_CNT);
	D32(RSI_MASK0);
	D32(RSI_MASK1);
	D16(RSI_PID0);
	D16(RSI_PID1);
	D16(RSI_PID2);
	D16(RSI_PID3);
	D16(RSI_PWR_CONTROL);
	D16(RSI_RD_WAIT_EN);
	D32(RSI_RESPONSE0);
	D32(RSI_RESPONSE1);
	D32(RSI_RESPONSE2);
	D32(RSI_RESPONSE3);
	D16(RSI_RESP_CMD);
	D32(RSI_STATUS);
	D_WO(RSI_STATUSCL, 16);
#endif

#ifdef RTC_ALARM
	parent = debugfs_create_dir("rtc", top);
	D32(RTC_ALARM);
	D16(RTC_ICTL);
	D16(RTC_ISTAT);
	D16(RTC_PREN);
	D32(RTC_STAT);
	D16(RTC_SWCNT);
#endif

#ifdef SDH_CFG
	parent = debugfs_create_dir("sdh", top);
	D32(SDH_ARGUMENT);
	D16(SDH_CFG);
	D16(SDH_CLK_CTL);
	D16(SDH_COMMAND);
	D_RO(SDH_DATA_CNT, 16);
	D16(SDH_DATA_CTL);
	D16(SDH_DATA_LGTH);
	D32(SDH_DATA_TIMER);
	D16(SDH_E_MASK);
	D16(SDH_E_STATUS);
	D32(SDH_FIFO);
	D_RO(SDH_FIFO_CNT, 16);
	D32(SDH_MASK0);
	D32(SDH_MASK1);
	D_RO(SDH_PID0, 16);
	D_RO(SDH_PID1, 16);
	D_RO(SDH_PID2, 16);
	D_RO(SDH_PID3, 16);
	D_RO(SDH_PID4, 16);
	D_RO(SDH_PID5, 16);
	D_RO(SDH_PID6, 16);
	D_RO(SDH_PID7, 16);
	D16(SDH_PWR_CTL);
	D16(SDH_RD_WAIT_EN);
	D_RO(SDH_RESPONSE0, 32);
	D_RO(SDH_RESPONSE1, 32);
	D_RO(SDH_RESPONSE2, 32);
	D_RO(SDH_RESPONSE3, 32);
	D_RO(SDH_RESP_CMD, 16);
	D_RO(SDH_STATUS, 32);
	D_WO(SDH_STATUS_CLR, 16);
#endif

#ifdef SECURE_CONTROL
	parent = debugfs_create_dir("security", top);
	D16(SECURE_CONTROL);
	D16(SECURE_STATUS);
	D32(SECURE_SYSSWT);
#endif

	parent = debugfs_create_dir("sic", top);
	D16(SWRST);
	D16(SYSCR);
	D16(SIC_RVECT);
	D32(SIC_IAR0);
	D32(SIC_IAR1);
	D32(SIC_IAR2);
#ifdef SIC_IAR3
	D32(SIC_IAR3);
#endif
#ifdef SIC_IAR4
	D32(SIC_IAR4);
	D32(SIC_IAR5);
	D32(SIC_IAR6);
#endif
#ifdef SIC_IAR7
	D32(SIC_IAR7);
#endif
#ifdef SIC_IAR8
	D32(SIC_IAR8);
	D32(SIC_IAR9);
	D32(SIC_IAR10);
	D32(SIC_IAR11);
#endif
#ifdef SIC_IMASK
	D32(SIC_IMASK);
	D32(SIC_ISR);
	D32(SIC_IWR);
#endif
#ifdef SIC_IMASK0
	D32(SIC_IMASK0);
	D32(SIC_IMASK1);
	D32(SIC_ISR0);
	D32(SIC_ISR1);
	D32(SIC_IWR0);
	D32(SIC_IWR1);
#endif
#ifdef SIC_IMASK2
	D32(SIC_IMASK2);
	D32(SIC_ISR2);
	D32(SIC_IWR2);
#endif
#ifdef SICB_RVECT
	D16(SICB_SWRST);
	D16(SICB_SYSCR);
	D16(SICB_RVECT);
	D32(SICB_IAR0);
	D32(SICB_IAR1);
	D32(SICB_IAR2);
	D32(SICB_IAR3);
	D32(SICB_IAR4);
	D32(SICB_IAR5);
	D32(SICB_IAR6);
	D32(SICB_IAR7);
	D32(SICB_IMASK0);
	D32(SICB_IMASK1);
	D32(SICB_ISR0);
	D32(SICB_ISR1);
	D32(SICB_IWR0);
	D32(SICB_IWR1);
#endif

	parent = debugfs_create_dir("sport", top);
#ifdef SPORT0_STAT
	SPORT(0);
#endif
#ifdef SPORT1_STAT
	SPORT(1);
#endif
#ifdef SPORT2_STAT
	SPORT(2);
#endif
#ifdef SPORT3_STAT
	SPORT(3);
#endif

#ifdef WDOG_CNT
	parent = debugfs_create_dir("watchdog", top);
	D32(WDOG_CNT);
	D16(WDOG_CTL);
	D32(WDOG_STAT);
#endif
#ifdef WDOGA_CNT
	parent = debugfs_create_dir("watchdog", top);
	D32(WDOGA_CNT);
	D16(WDOGA_CTL);
	D32(WDOGA_STAT);
	D32(WDOGB_CNT);
	D16(WDOGB_CTL);
	D32(WDOGB_STAT);
#endif

#ifdef __ADSPBF51x__
# define USE_BF51x 1
#else
# define USE_BF51x 0
#endif
	if (USE_BF51x) {

		parent = debugfs_create_dir("DMA Controller", top);
		d("DMA0_CONFIG", 16, 0xFFC00C08);
		d("DMA0_CURR_ADDR", 32, 0xFFC00C24);
		d("DMA0_CURR_DESC_PTR", 32, 0xFFC00C20);
		d("DMA0_CURR_X_COUNT", 16, 0xFFC00C30);
		d("DMA0_CURR_Y_COUNT", 16, 0xFFC00C38);
		d("DMA0_IRQ_STATUS", 16, 0xFFC00C28);
		d("DMA0_NEXT_DESC_PTR", 32, 0xFFC00C00);
		d("DMA0_PERIPHERAL_MAP", 16, 0xFFC00C2C);
		d("DMA0_START_ADDR", 32, 0xFFC00C04);
		d("DMA0_X_COUNT", 16, 0xFFC00C10);
		d("DMA0_X_MODIFY", 16, 0xFFC00C14);
		d("DMA0_Y_COUNT", 16, 0xFFC00C18);
		d("DMA0_Y_MODIFY", 16, 0xFFC00C1C);
		d("DMA10_CONFIG", 16, 0xFFC00E88);
		d("DMA10_CURR_ADDR", 32, 0xFFC00EA4);
		d("DMA10_CURR_DESC_PTR", 32, 0xFFC00EA0);
		d("DMA10_CURR_X_COUNT", 16, 0xFFC00EB0);
		d("DMA10_CURR_Y_COUNT", 16, 0xFFC00EB8);
		d("DMA10_IRQ_STATUS", 16, 0xFFC00EA8);
		d("DMA10_NEXT_DESC_PTR", 32, 0xFFC00E80);
		d("DMA10_PERIPHERAL_MAP", 16, 0xFFC00EAC);
		d("DMA10_START_ADDR", 32, 0xFFC00E84);
		d("DMA10_X_COUNT", 16, 0xFFC00E90);
		d("DMA10_X_MODIFY", 16, 0xFFC00E94);
		d("DMA10_Y_COUNT", 16, 0xFFC00E98);
		d("DMA10_Y_MODIFY", 16, 0xFFC00E9C);
		d("DMA11_CONFIG", 16, 0xFFC00EC8);
		d("DMA11_CURR_ADDR", 32, 0xFFC00EE4);
		d("DMA11_CURR_DESC_PTR", 32, 0xFFC00EE0);
		d("DMA11_CURR_X_COUNT", 16, 0xFFC00EF0);
		d("DMA11_CURR_Y_COUNT", 16, 0xFFC00EF8);
		d("DMA11_IRQ_STATUS", 16, 0xFFC00EE8);
		d("DMA11_NEXT_DESC_PTR", 32, 0xFFC00EC0);
		d("DMA11_PERIPHERAL_MAP", 16, 0xFFC00EEC);
		d("DMA11_START_ADDR", 32, 0xFFC00EC4);
		d("DMA11_X_COUNT", 16, 0xFFC00ED0);
		d("DMA11_X_MODIFY", 16, 0xFFC00ED4);
		d("DMA11_Y_COUNT", 16, 0xFFC00ED8);
		d("DMA11_Y_MODIFY", 16, 0xFFC00EDC);
		d("DMA1_CONFIG", 16, 0xFFC00C48);
		d("DMA1_CURR_ADDR", 32, 0xFFC00C64);
		d("DMA1_CURR_DESC_PTR", 32, 0xFFC00C60);
		d("DMA1_CURR_X_COUNT", 16, 0xFFC00C70);
		d("DMA1_CURR_Y_COUNT", 16, 0xFFC00C78);
		d("DMA1_IRQ_STATUS", 16, 0xFFC00C68);
		d("DMA1_NEXT_DESC_PTR", 32, 0xFFC00C40);
		d("DMA1_PERIPHERAL_MAP", 16, 0xFFC00C6C);
		d("DMA1_START_ADDR", 32, 0xFFC00C44);
		d("DMA1_X_COUNT", 16, 0xFFC00C50);
		d("DMA1_X_MODIFY", 16, 0xFFC00C54);
		d("DMA1_Y_COUNT", 16, 0xFFC00C58);
		d("DMA1_Y_MODIFY", 16, 0xFFC00C5C);
		d("DMA2_CONFIG", 16, 0xFFC00C88);
		d("DMA2_CURR_ADDR", 32, 0xFFC00CA4);
		d("DMA2_CURR_DESC_PTR", 32, 0xFFC00CA0);
		d("DMA2_CURR_X_COUNT", 16, 0xFFC00CB0);
		d("DMA2_CURR_Y_COUNT", 16, 0xFFC00CB8);
		d("DMA2_IRQ_STATUS", 16, 0xFFC00CA8);
		d("DMA2_NEXT_DESC_PTR", 32, 0xFFC00C80);
		d("DMA2_PERIPHERAL_MAP", 16, 0xFFC00CAC);
		d("DMA2_START_ADDR", 32, 0xFFC00C84);
		d("DMA2_X_COUNT", 16, 0xFFC00C90);
		d("DMA2_X_MODIFY", 16, 0xFFC00C94);
		d("DMA2_Y_COUNT", 16, 0xFFC00C98);
		d("DMA2_Y_MODIFY", 16, 0xFFC00C9C);
		d("DMA3_CONFIG", 16, 0xFFC00CC8);
		d("DMA3_CURR_ADDR", 32, 0xFFC00CE4);
		d("DMA3_CURR_DESC_PTR", 32, 0xFFC00CE0);
		d("DMA3_CURR_X_COUNT", 16, 0xFFC00CF0);
		d("DMA3_CURR_Y_COUNT", 16, 0xFFC00CF8);
		d("DMA3_IRQ_STATUS", 16, 0xFFC00CE8);
		d("DMA3_NEXT_DESC_PTR", 32, 0xFFC00CC0);
		d("DMA3_PERIPHERAL_MAP", 16, 0xFFC00CEC);
		d("DMA3_START_ADDR", 32, 0xFFC00CC4);
		d("DMA3_X_COUNT", 16, 0xFFC00CD0);
		d("DMA3_X_MODIFY", 16, 0xFFC00CD4);
		d("DMA3_Y_COUNT", 16, 0xFFC00CD8);
		d("DMA3_Y_MODIFY", 16, 0xFFC00CDC);
		d("DMA4_CONFIG", 16, 0xFFC00D08);
		d("DMA4_CURR_ADDR", 32, 0xFFC00D24);
		d("DMA4_CURR_DESC_PTR", 32, 0xFFC00D20);
		d("DMA4_CURR_X_COUNT", 16, 0xFFC00D30);
		d("DMA4_CURR_Y_COUNT", 16, 0xFFC00D38);
		d("DMA4_IRQ_STATUS", 16, 0xFFC00D28);
		d("DMA4_NEXT_DESC_PTR", 32, 0xFFC00D00);
		d("DMA4_PERIPHERAL_MAP", 16, 0xFFC00D2C);
		d("DMA4_START_ADDR", 32, 0xFFC00D04);
		d("DMA4_X_COUNT", 16, 0xFFC00D10);
		d("DMA4_X_MODIFY", 16, 0xFFC00D14);
		d("DMA4_Y_COUNT", 16, 0xFFC00D18);
		d("DMA4_Y_MODIFY", 16, 0xFFC00D1C);
		d("DMA5_CONFIG", 16, 0xFFC00D48);
		d("DMA5_CURR_ADDR", 32, 0xFFC00D64);
		d("DMA5_CURR_DESC_PTR", 32, 0xFFC00D60);
		d("DMA5_CURR_X_COUNT", 16, 0xFFC00D70);
		d("DMA5_CURR_Y_COUNT", 16, 0xFFC00D78);
		d("DMA5_IRQ_STATUS", 16, 0xFFC00D68);
		d("DMA5_NEXT_DESC_PTR", 32, 0xFFC00D40);
		d("DMA5_PERIPHERAL_MAP", 16, 0xFFC00D6C);
		d("DMA5_START_ADDR", 32, 0xFFC00D44);
		d("DMA5_X_COUNT", 16, 0xFFC00D50);
		d("DMA5_X_MODIFY", 16, 0xFFC00D54);
		d("DMA5_Y_COUNT", 16, 0xFFC00D58);
		d("DMA5_Y_MODIFY", 16, 0xFFC00D5C);
		d("DMA6_CONFIG", 16, 0xFFC00D88);
		d("DMA6_CURR_ADDR", 32, 0xFFC00DA4);
		d("DMA6_CURR_DESC_PTR", 32, 0xFFC00DA0);
		d("DMA6_CURR_X_COUNT", 16, 0xFFC00DB0);
		d("DMA6_CURR_Y_COUNT", 16, 0xFFC00DB8);
		d("DMA6_IRQ_STATUS", 16, 0xFFC00DA8);
		d("DMA6_NEXT_DESC_PTR", 32, 0xFFC00D80);
		d("DMA6_PERIPHERAL_MAP", 16, 0xFFC00DAC);
		d("DMA6_START_ADDR", 32, 0xFFC00D84);
		d("DMA6_X_COUNT", 16, 0xFFC00D90);
		d("DMA6_X_MODIFY", 16, 0xFFC00D94);
		d("DMA6_Y_COUNT", 16, 0xFFC00D98);
		d("DMA6_Y_MODIFY", 16, 0xFFC00D9C);
		d("DMA7_CONFIG", 16, 0xFFC00DC8);
		d("DMA7_CURR_ADDR", 32, 0xFFC00DE4);
		d("DMA7_CURR_DESC_PTR", 32, 0xFFC00DE0);
		d("DMA7_CURR_X_COUNT", 16, 0xFFC00DF0);
		d("DMA7_CURR_Y_COUNT", 16, 0xFFC00DF8);
		d("DMA7_IRQ_STATUS", 16, 0xFFC00DE8);
		d("DMA7_NEXT_DESC_PTR", 32, 0xFFC00DC0);
		d("DMA7_PERIPHERAL_MAP", 16, 0xFFC00DEC);
		d("DMA7_START_ADDR", 32, 0xFFC00DC4);
		d("DMA7_X_COUNT", 16, 0xFFC00DD0);
		d("DMA7_X_MODIFY", 16, 0xFFC00DD4);
		d("DMA7_Y_COUNT", 16, 0xFFC00DD8);
		d("DMA7_Y_MODIFY", 16, 0xFFC00DDC);
		d("DMA8_CONFIG", 16, 0xFFC00E08);
		d("DMA8_CURR_ADDR", 32, 0xFFC00E24);
		d("DMA8_CURR_DESC_PTR", 32, 0xFFC00E20);
		d("DMA8_CURR_X_COUNT", 16, 0xFFC00E30);
		d("DMA8_CURR_Y_COUNT", 16, 0xFFC00E38);
		d("DMA8_IRQ_STATUS", 16, 0xFFC00E28);
		d("DMA8_NEXT_DESC_PTR", 32, 0xFFC00E00);
		d("DMA8_PERIPHERAL_MAP", 16, 0xFFC00E2C);
		d("DMA8_START_ADDR", 32, 0xFFC00E04);
		d("DMA8_X_COUNT", 16, 0xFFC00E10);
		d("DMA8_X_MODIFY", 16, 0xFFC00E14);
		d("DMA8_Y_COUNT", 16, 0xFFC00E18);
		d("DMA8_Y_MODIFY", 16, 0xFFC00E1C);
		d("DMA9_CONFIG", 16, 0xFFC00E48);
		d("DMA9_CURR_ADDR", 32, 0xFFC00E64);
		d("DMA9_CURR_DESC_PTR", 32, 0xFFC00E60);
		d("DMA9_CURR_X_COUNT", 16, 0xFFC00E70);
		d("DMA9_CURR_Y_COUNT", 16, 0xFFC00E78);
		d("DMA9_IRQ_STATUS", 16, 0xFFC00E68);
		d("DMA9_NEXT_DESC_PTR", 32, 0xFFC00E40);
		d("DMA9_PERIPHERAL_MAP", 16, 0xFFC00E6C);
		d("DMA9_START_ADDR", 32, 0xFFC00E44);
		d("DMA9_X_COUNT", 16, 0xFFC00E50);
		d("DMA9_X_MODIFY", 16, 0xFFC00E54);
		d("DMA9_Y_COUNT", 16, 0xFFC00E58);
		d("DMA9_Y_MODIFY", 16, 0xFFC00E5C);

		parent = debugfs_create_dir("DMA Traffic Control", top);
		d("DMA_TC_CNT", 16, 0xFFC00B0C);
		d("DMA_TC_PER", 16, 0xFFC00B10);

		parent = debugfs_create_dir("GPIO PIN", top);
		d("PORTF_DRIVE", 16, 0xFFC03220);
		d("PORTF_HYSTERESIS", 16, 0xFFC03240);
		d("PORTF_MUX", 16, 0xFFC03210);
		d("PORTG_DRIVE", 16, 0xFFC03224);
		d("PORTG_HYSTERESIS", 16, 0xFFC03244);
		d("PORTG_MUX", 16, 0xFFC03214);
		d("PORTH_DRIVE", 16, 0xFFC03228);
		d("PORTH_HYSTERESIS", 16, 0xFFC03248);
		d("PORTH_MUX", 16, 0xFFC03218);

		parent = debugfs_create_dir("Handshake MDMA", top);
		d("HMDMA0_BCINIT", 16, 0xFFC03308);
		d("HMDMA0_BCOUNT", 16, 0xFFC03318);
		d("HMDMA0_CONTROL", 16, 0xFFC03300);
		d("HMDMA0_ECINIT", 16, 0xFFC03304);
		d("HMDMA0_ECOUNT", 16, 0xFFC03314);
		d("HMDMA0_ECOVERFLOW", 16, 0xFFC03310);
		d("HMDMA0_ECURGENT", 16, 0xFFC0330C);
		d("HMDMA1_BCINIT", 16, 0xFFC03348);
		d("HMDMA1_BCOUNT", 16, 0xFFC03358);
		d("HMDMA1_CONTROL", 16, 0xFFC03340);
		d("HMDMA1_ECINIT", 16, 0xFFC03344);
		d("HMDMA1_ECOUNT", 16, 0xFFC03354);
		d("HMDMA1_ECOVERFLOW", 16, 0xFFC03350);
		d("HMDMA1_ECURGENT", 16, 0xFFC0334C);

		parent = debugfs_create_dir("MDMA Destination 0", top);
		d("MDMA_D0_CONFIG", 16, 0xFFC00F08);
		d("MDMA_D0_CURR_ADDR", 32, 0xFFC00F24);
		d("MDMA_D0_CURR_DESC_PTR", 32, 0xFFC00F20);
		d("MDMA_D0_CURR_X_COUNT", 16, 0xFFC00F30);
		d("MDMA_D0_CURR_Y_COUNT", 16, 0xFFC00F38);
		d("MDMA_D0_IRQ_STATUS", 16, 0xFFC00F28);
		d("MDMA_D0_NEXT_DESC_PTR", 32, 0xFFC00F00);
		d("MDMA_D0_PERIPHERAL_MAP", 16, 0xFFC00F2C);
		d("MDMA_D0_START_ADDR", 32, 0xFFC00F04);
		d("MDMA_D0_X_COUNT", 16, 0xFFC00F10);
		d("MDMA_D0_X_MODIFY", 16, 0xFFC00F14);
		d("MDMA_D0_Y_COUNT", 16, 0xFFC00F18);
		d("MDMA_D0_Y_MODIFY", 16, 0xFFC00F1C);

		parent = debugfs_create_dir("MDMA Destination 1", top);
		d("MDMA_D1_CONFIG", 16, 0xFFC00F88);
		d("MDMA_D1_CURR_ADDR", 32, 0xFFC00FA4);
		d("MDMA_D1_CURR_DESC_PTR", 32, 0xFFC00FA0);
		d("MDMA_D1_CURR_X_COUNT", 16, 0xFFC00FB0);
		d("MDMA_D1_CURR_Y_COUNT", 16, 0xFFC00FB8);
		d("MDMA_D1_IRQ_STATUS", 16, 0xFFC00FA8);
		d("MDMA_D1_NEXT_DESC_PTR", 32, 0xFFC00F80);
		d("MDMA_D1_PERIPHERAL_MAP", 16, 0xFFC00FAC);
		d("MDMA_D1_START_ADDR", 32, 0xFFC00F84);
		d("MDMA_D1_X_COUNT", 16, 0xFFC00F90);
		d("MDMA_D1_X_MODIFY", 16, 0xFFC00F94);
		d("MDMA_D1_Y_COUNT", 16, 0xFFC00F98);
		d("MDMA_D1_Y_MODIFY", 16, 0xFFC00F9C);

		parent = debugfs_create_dir("MDMA Source 0", top);
		d("MDMA_S0_CONFIG", 16, 0xFFC00F48);
		d("MDMA_S0_CURR_ADDR", 32, 0xFFC00F64);
		d("MDMA_S0_CURR_DESC_PTR", 32, 0xFFC00F60);
		d("MDMA_S0_CURR_X_COUNT", 16, 0xFFC00F70);
		d("MDMA_S0_CURR_Y_COUNT", 16, 0xFFC00F78);
		d("MDMA_S0_IRQ_STATUS", 16, 0xFFC00F68);
		d("MDMA_S0_NEXT_DESC_PTR", 32, 0xFFC00F40);
		d("MDMA_S0_PERIPHERAL_MAP", 16, 0xFFC00F6C);
		d("MDMA_S0_START_ADDR", 32, 0xFFC00F44);
		d("MDMA_S0_X_COUNT", 16, 0xFFC00F50);
		d("MDMA_S0_X_MODIFY", 16, 0xFFC00F54);
		d("MDMA_S0_Y_COUNT", 16, 0xFFC00F58);
		d("MDMA_S0_Y_MODIFY", 16, 0xFFC00F5C);

		parent = debugfs_create_dir("MDMA Source 1", top);
		d("MDMA_S1_CONFIG", 16, 0xFFC00FC8);
		d("MDMA_S1_CURR_ADDR", 32, 0xFFC00FE4);
		d("MDMA_S1_CURR_DESC_PTR", 32, 0xFFC00FE0);
		d("MDMA_S1_CURR_X_COUNT", 16, 0xFFC00FF0);
		d("MDMA_S1_CURR_Y_COUNT", 16, 0xFFC00FF8);
		d("MDMA_S1_IRQ_STATUS", 16, 0xFFC00FE8);
		d("MDMA_S1_NEXT_DESC_PTR", 32, 0xFFC00FC0);
		d("MDMA_S1_PERIPHERAL_MAP", 16, 0xFFC00FEC);
		d("MDMA_S1_START_ADDR", 32, 0xFFC00FC4);
		d("MDMA_S1_X_COUNT", 16, 0xFFC00FD0);
		d("MDMA_S1_X_MODIFY", 16, 0xFFC00FD4);
		d("MDMA_S1_Y_COUNT", 16, 0xFFC00FD8);
		d("MDMA_S1_Y_MODIFY", 16, 0xFFC00FDC);

		parent = debugfs_create_dir("NON-GPIO", top);
		d("NONGPIO_DRIVE", 16, 0xFFC03280);
		d("NONGPIO_HYSTERESIS", 16, 0xFFC03288);

		parent = debugfs_create_dir("Pin Control", top);
		d("PORTF_FER", 16, 0xFFC03200);
		d("PORTG_FER", 16, 0xFFC03204);
		d("PORTH_FER", 16, 0xFFC03208);

		parent = debugfs_create_dir("Port I-O", top);
		d("PORTFIO", 16, 0xFFC00700);
		d("PORTFIO_BOTH", 16, 0xFFC0073C);
		d("PORTFIO_CLEAR", 16, 0xFFC00704);
		d("PORTFIO_DIR", 16, 0xFFC00730);
		d("PORTFIO_EDGE", 16, 0xFFC00738);
		d("PORTFIO_INEN", 16, 0xFFC00740);
		d("PORTFIO_MASKA", 16, 0xFFC00710);
		d("PORTFIO_MASKA_CLEAR", 16, 0xFFC00714);
		d("PORTFIO_MASKA_SET", 16, 0xFFC00718);
		d("PORTFIO_MASKA_TOGGLE", 16, 0xFFC0071C);
		d("PORTFIO_MASKB", 16, 0xFFC00720);
		d("PORTFIO_MASKB_CLEAR", 16, 0xFFC00724);
		d("PORTFIO_MASKB_SET", 16, 0xFFC00728);
		d("PORTFIO_MASKB_TOGGLE", 16, 0xFFC0072C);
		d("PORTFIO_POLAR", 16, 0xFFC00734);
		d("PORTFIO_SET", 16, 0xFFC00708);
		d("PORTFIO_TOGGLE", 16, 0xFFC0070C);
		d("PORTGIO", 16, 0xFFC01500);
		d("PORTGIO_BOTH", 16, 0xFFC0153C);
		d("PORTGIO_CLEAR", 16, 0xFFC01504);
		d("PORTGIO_DIR", 16, 0xFFC01530);
		d("PORTGIO_EDGE", 16, 0xFFC01538);
		d("PORTGIO_INEN", 16, 0xFFC01540);
		d("PORTGIO_MASKA", 16, 0xFFC01510);
		d("PORTGIO_MASKA_CLEAR", 16, 0xFFC01514);
		d("PORTGIO_MASKA_SET", 16, 0xFFC01518);
		d("PORTGIO_MASKA_TOGGLE", 16, 0xFFC0151C);
		d("PORTGIO_MASKB", 16, 0xFFC01520);
		d("PORTGIO_MASKB_CLEAR", 16, 0xFFC01524);
		d("PORTGIO_MASKB_SET", 16, 0xFFC01528);
		d("PORTGIO_MASKB_TOGGLE", 16, 0xFFC0152C);
		d("PORTGIO_POLAR", 16, 0xFFC01534);
		d("PORTGIO_SET", 16, 0xFFC01508);
		d("PORTGIO_TOGGLE", 16, 0xFFC0150C);
		d("PORTHIO", 16, 0xFFC01700);
		d("PORTHIO_BOTH", 16, 0xFFC0173C);
		d("PORTHIO_CLEAR", 16, 0xFFC01704);
		d("PORTHIO_DIR", 16, 0xFFC01730);
		d("PORTHIO_EDGE", 16, 0xFFC01738);
		d("PORTHIO_INEN", 16, 0xFFC01740);
		d("PORTHIO_MASKA", 16, 0xFFC01710);
		d("PORTHIO_MASKA_CLEAR", 16, 0xFFC01714);
		d("PORTHIO_MASKA_SET", 16, 0xFFC01718);
		d("PORTHIO_MASKA_TOGGLE", 16, 0xFFC0171C);
		d("PORTHIO_MASKB", 16, 0xFFC01720);
		d("PORTHIO_MASKB_CLEAR", 16, 0xFFC01724);
		d("PORTHIO_MASKB_SET", 16, 0xFFC01728);
		d("PORTHIO_MASKB_TOGGLE", 16, 0xFFC0172C);
		d("PORTHIO_POLAR", 16, 0xFFC01734);
		d("PORTHIO_SET", 16, 0xFFC01708);
		d("PORTHIO_TOGGLE", 16, 0xFFC0170C);

		parent = debugfs_create_dir("SPI", top);
		d("SPI0_BAUD", 16, 0xFFC00514);
		d("SPI0_CTL", 16, 0xFFC00500);
		d("SPI0_FLG", 16, 0xFFC00504);
		d("SPI0_RDBR", 16, 0xFFC00510);
		d("SPI0_SHADOW", 16, 0xFFC00518);
		d("SPI0_STAT", 16, 0xFFC00508);
		d("SPI0_TDBR", 16, 0xFFC0050C);
		d("SPI1_BAUD", 16, 0xFFC03414);
		d("SPI1_CTL", 16, 0xFFC03400);
		d("SPI1_FLG", 16, 0xFFC03404);
		d("SPI1_RDBR", 16, 0xFFC03410);
		d("SPI1_SHADOW", 16, 0xFFC03418);
		d("SPI1_STAT", 16, 0xFFC03408);
		d("SPI1_TDBR", 16, 0xFFC0340C);

		parent = debugfs_create_dir("TWI", top);
		d("TWI_CLKDIV", 16, 0xFFC01400);
		d("TWI_CONTROL", 16, 0xFFC01404);
		d("TWI_FIFO_CTL", 16, 0xFFC01428);
		d("TWI_FIFO_STAT", 16, 0xFFC0142C);
		d("TWI_INT_MASK", 16, 0xFFC01424);
		d("TWI_INT_STAT", 16, 0xFFC01420);
		d("TWI_MASTER_ADDR", 16, 0xFFC0141C);
		d("TWI_MASTER_CTL", 16, 0xFFC01414);
		d("TWI_MASTER_STAT", 16, 0xFFC01418);
		d("TWI_RCV_DATA16", 16, 0xFFC0148C);
		d("TWI_RCV_DATA8", 16, 0xFFC01488);
		d("TWI_SLAVE_ADDR", 16, 0xFFC01410);
		d("TWI_SLAVE_CTL", 16, 0xFFC01408);
		d("TWI_SLAVE_STAT", 16, 0xFFC0140C);
		d("TWI_XMT_DATA16", 16, 0xFFC01484);
		d("TWI_XMT_DATA8", 16, 0xFFC01480);

		parent = debugfs_create_dir("UART0", top);
		d("UART0_DLH", 16, 0xFFC00404);
		d("UART0_DLL", 16, 0xFFC00400);
		d("UART0_GCTL", 16, 0xFFC00424);
		d("UART0_IER", 16, 0xFFC00404);
		d("UART0_IIR", 16, 0xFFC00408);
		d("UART0_LCR", 16, 0xFFC0040C);
		d("UART0_LSR", 16, 0xFFC00414);
		d("UART0_MCR", 16, 0xFFC00410);
		d("UART0_MSR", 16, 0xFFC00418);
		d("UART0_RBR", 16, 0xFFC00400);
		d("UART0_SCR", 16, 0xFFC0041C);
		d("UART0_THR", 16, 0xFFC00400);

		parent = debugfs_create_dir("UART", top);
		d("UART1_DLH", 16, 0xFFC02004);
		d("UART1_DLL", 16, 0xFFC02000);
		d("UART1_GCTL", 16, 0xFFC02024);
		d("UART1_IER", 16, 0xFFC02004);
		d("UART1_IIR", 16, 0xFFC02008);
		d("UART1_LCR", 16, 0xFFC0200C);
		d("UART1_LSR", 16, 0xFFC02014);
		d("UART1_MCR", 16, 0xFFC02010);
		d("UART1_MSR", 16, 0xFFC02018);
		d("UART1_RBR", 16, 0xFFC02000);
		d("UART1_SCR", 16, 0xFFC0201C);
		d("UART1_THR", 16, 0xFFC02000);

	}	/* BF51x */

#ifdef __ADSPBF522__
# define USE_BF522 1
#else
# define USE_BF522 0
#endif
#ifdef __ADSPBF523__
# define USE_BF523 1
#else
# define USE_BF523 0
#endif
	if (USE_BF522 || USE_BF523) {

		parent = debugfs_create_dir("DMA Controller", top);
		d("DMA0_CONFIG", 16, 0xFFC00C08);
		d("DMA0_CURR_ADDR", 32, 0xFFC00C24);
		d("DMA0_CURR_DESC_PTR", 32, 0xFFC00C20);
		d("DMA0_CURR_X_COUNT", 16, 0xFFC00C30);
		d("DMA0_CURR_Y_COUNT", 16, 0xFFC00C38);
		d("DMA0_IRQ_STATUS", 16, 0xFFC00C28);
		d("DMA0_NEXT_DESC_PTR", 32, 0xFFC00C00);
		d("DMA0_PERIPHERAL_MAP", 16, 0xFFC00C2C);
		d("DMA0_START_ADDR", 32, 0xFFC00C04);
		d("DMA0_X_COUNT", 16, 0xFFC00C10);
		d("DMA0_X_MODIFY", 16, 0xFFC00C14);
		d("DMA0_Y_COUNT", 16, 0xFFC00C18);
		d("DMA0_Y_MODIFY", 16, 0xFFC00C1C);
		d("DMA10_CONFIG", 16, 0xFFC00E88);
		d("DMA10_CURR_ADDR", 32, 0xFFC00EA4);
		d("DMA10_CURR_DESC_PTR", 32, 0xFFC00EA0);
		d("DMA10_CURR_X_COUNT", 16, 0xFFC00EB0);
		d("DMA10_CURR_Y_COUNT", 16, 0xFFC00EB8);
		d("DMA10_IRQ_STATUS", 16, 0xFFC00EA8);
		d("DMA10_NEXT_DESC_PTR", 32, 0xFFC00E80);
		d("DMA10_PERIPHERAL_MAP", 16, 0xFFC00EAC);
		d("DMA10_START_ADDR", 32, 0xFFC00E84);
		d("DMA10_X_COUNT", 16, 0xFFC00E90);
		d("DMA10_X_MODIFY", 16, 0xFFC00E94);
		d("DMA10_Y_COUNT", 16, 0xFFC00E98);
		d("DMA10_Y_MODIFY", 16, 0xFFC00E9C);
		d("DMA11_CONFIG", 16, 0xFFC00EC8);
		d("DMA11_CURR_ADDR", 32, 0xFFC00EE4);
		d("DMA11_CURR_DESC_PTR", 32, 0xFFC00EE0);
		d("DMA11_CURR_X_COUNT", 16, 0xFFC00EF0);
		d("DMA11_CURR_Y_COUNT", 16, 0xFFC00EF8);
		d("DMA11_IRQ_STATUS", 16, 0xFFC00EE8);
		d("DMA11_NEXT_DESC_PTR", 32, 0xFFC00EC0);
		d("DMA11_PERIPHERAL_MAP", 16, 0xFFC00EEC);
		d("DMA11_START_ADDR", 32, 0xFFC00EC4);
		d("DMA11_X_COUNT", 16, 0xFFC00ED0);
		d("DMA11_X_MODIFY", 16, 0xFFC00ED4);
		d("DMA11_Y_COUNT", 16, 0xFFC00ED8);
		d("DMA11_Y_MODIFY", 16, 0xFFC00EDC);
		d("DMA1_CONFIG", 16, 0xFFC00C48);
		d("DMA1_CURR_ADDR", 32, 0xFFC00C64);
		d("DMA1_CURR_DESC_PTR", 32, 0xFFC00C60);
		d("DMA1_CURR_X_COUNT", 16, 0xFFC00C70);
		d("DMA1_CURR_Y_COUNT", 16, 0xFFC00C78);
		d("DMA1_IRQ_STATUS", 16, 0xFFC00C68);
		d("DMA1_NEXT_DESC_PTR", 32, 0xFFC00C40);
		d("DMA1_PERIPHERAL_MAP", 16, 0xFFC00C6C);
		d("DMA1_START_ADDR", 32, 0xFFC00C44);
		d("DMA1_X_COUNT", 16, 0xFFC00C50);
		d("DMA1_X_MODIFY", 16, 0xFFC00C54);
		d("DMA1_Y_COUNT", 16, 0xFFC00C58);
		d("DMA1_Y_MODIFY", 16, 0xFFC00C5C);
		d("DMA2_CONFIG", 16, 0xFFC00C88);
		d("DMA2_CURR_ADDR", 32, 0xFFC00CA4);
		d("DMA2_CURR_DESC_PTR", 32, 0xFFC00CA0);
		d("DMA2_CURR_X_COUNT", 16, 0xFFC00CB0);
		d("DMA2_CURR_Y_COUNT", 16, 0xFFC00CB8);
		d("DMA2_IRQ_STATUS", 16, 0xFFC00CA8);
		d("DMA2_NEXT_DESC_PTR", 32, 0xFFC00C80);
		d("DMA2_PERIPHERAL_MAP", 16, 0xFFC00CAC);
		d("DMA2_START_ADDR", 32, 0xFFC00C84);
		d("DMA2_X_COUNT", 16, 0xFFC00C90);
		d("DMA2_X_MODIFY", 16, 0xFFC00C94);
		d("DMA2_Y_COUNT", 16, 0xFFC00C98);
		d("DMA2_Y_MODIFY", 16, 0xFFC00C9C);
		d("DMA3_CONFIG", 16, 0xFFC00CC8);
		d("DMA3_CURR_ADDR", 32, 0xFFC00CE4);
		d("DMA3_CURR_DESC_PTR", 32, 0xFFC00CE0);
		d("DMA3_CURR_X_COUNT", 16, 0xFFC00CF0);
		d("DMA3_CURR_Y_COUNT", 16, 0xFFC00CF8);
		d("DMA3_IRQ_STATUS", 16, 0xFFC00CE8);
		d("DMA3_NEXT_DESC_PTR", 32, 0xFFC00CC0);
		d("DMA3_PERIPHERAL_MAP", 16, 0xFFC00CEC);
		d("DMA3_START_ADDR", 32, 0xFFC00CC4);
		d("DMA3_X_COUNT", 16, 0xFFC00CD0);
		d("DMA3_X_MODIFY", 16, 0xFFC00CD4);
		d("DMA3_Y_COUNT", 16, 0xFFC00CD8);
		d("DMA3_Y_MODIFY", 16, 0xFFC00CDC);
		d("DMA4_CONFIG", 16, 0xFFC00D08);
		d("DMA4_CURR_ADDR", 32, 0xFFC00D24);
		d("DMA4_CURR_DESC_PTR", 32, 0xFFC00D20);
		d("DMA4_CURR_X_COUNT", 16, 0xFFC00D30);
		d("DMA4_CURR_Y_COUNT", 16, 0xFFC00D38);
		d("DMA4_IRQ_STATUS", 16, 0xFFC00D28);
		d("DMA4_NEXT_DESC_PTR", 32, 0xFFC00D00);
		d("DMA4_PERIPHERAL_MAP", 16, 0xFFC00D2C);
		d("DMA4_START_ADDR", 32, 0xFFC00D04);
		d("DMA4_X_COUNT", 16, 0xFFC00D10);
		d("DMA4_X_MODIFY", 16, 0xFFC00D14);
		d("DMA4_Y_COUNT", 16, 0xFFC00D18);
		d("DMA4_Y_MODIFY", 16, 0xFFC00D1C);
		d("DMA5_CONFIG", 16, 0xFFC00D48);
		d("DMA5_CURR_ADDR", 32, 0xFFC00D64);
		d("DMA5_CURR_DESC_PTR", 32, 0xFFC00D60);
		d("DMA5_CURR_X_COUNT", 16, 0xFFC00D70);
		d("DMA5_CURR_Y_COUNT", 16, 0xFFC00D78);
		d("DMA5_IRQ_STATUS", 16, 0xFFC00D68);
		d("DMA5_NEXT_DESC_PTR", 32, 0xFFC00D40);
		d("DMA5_PERIPHERAL_MAP", 16, 0xFFC00D6C);
		d("DMA5_START_ADDR", 32, 0xFFC00D44);
		d("DMA5_X_COUNT", 16, 0xFFC00D50);
		d("DMA5_X_MODIFY", 16, 0xFFC00D54);
		d("DMA5_Y_COUNT", 16, 0xFFC00D58);
		d("DMA5_Y_MODIFY", 16, 0xFFC00D5C);
		d("DMA6_CONFIG", 16, 0xFFC00D88);
		d("DMA6_CURR_ADDR", 32, 0xFFC00DA4);
		d("DMA6_CURR_DESC_PTR", 32, 0xFFC00DA0);
		d("DMA6_CURR_X_COUNT", 16, 0xFFC00DB0);
		d("DMA6_CURR_Y_COUNT", 16, 0xFFC00DB8);
		d("DMA6_IRQ_STATUS", 16, 0xFFC00DA8);
		d("DMA6_NEXT_DESC_PTR", 32, 0xFFC00D80);
		d("DMA6_PERIPHERAL_MAP", 16, 0xFFC00DAC);
		d("DMA6_START_ADDR", 32, 0xFFC00D84);
		d("DMA6_X_COUNT", 16, 0xFFC00D90);
		d("DMA6_X_MODIFY", 16, 0xFFC00D94);
		d("DMA6_Y_COUNT", 16, 0xFFC00D98);
		d("DMA6_Y_MODIFY", 16, 0xFFC00D9C);
		d("DMA7_CONFIG", 16, 0xFFC00DC8);
		d("DMA7_CURR_ADDR", 32, 0xFFC00DE4);
		d("DMA7_CURR_DESC_PTR", 32, 0xFFC00DE0);
		d("DMA7_CURR_X_COUNT", 16, 0xFFC00DF0);
		d("DMA7_CURR_Y_COUNT", 16, 0xFFC00DF8);
		d("DMA7_IRQ_STATUS", 16, 0xFFC00DE8);
		d("DMA7_NEXT_DESC_PTR", 32, 0xFFC00DC0);
		d("DMA7_PERIPHERAL_MAP", 16, 0xFFC00DEC);
		d("DMA7_START_ADDR", 32, 0xFFC00DC4);
		d("DMA7_X_COUNT", 16, 0xFFC00DD0);
		d("DMA7_X_MODIFY", 16, 0xFFC00DD4);
		d("DMA7_Y_COUNT", 16, 0xFFC00DD8);
		d("DMA7_Y_MODIFY", 16, 0xFFC00DDC);
		d("DMA8_CONFIG", 16, 0xFFC00E08);
		d("DMA8_CURR_ADDR", 32, 0xFFC00E24);
		d("DMA8_CURR_DESC_PTR", 32, 0xFFC00E20);
		d("DMA8_CURR_X_COUNT", 16, 0xFFC00E30);
		d("DMA8_CURR_Y_COUNT", 16, 0xFFC00E38);
		d("DMA8_IRQ_STATUS", 16, 0xFFC00E28);
		d("DMA8_NEXT_DESC_PTR", 32, 0xFFC00E00);
		d("DMA8_PERIPHERAL_MAP", 16, 0xFFC00E2C);
		d("DMA8_START_ADDR", 32, 0xFFC00E04);
		d("DMA8_X_COUNT", 16, 0xFFC00E10);
		d("DMA8_X_MODIFY", 16, 0xFFC00E14);
		d("DMA8_Y_COUNT", 16, 0xFFC00E18);
		d("DMA8_Y_MODIFY", 16, 0xFFC00E1C);
		d("DMA9_CONFIG", 16, 0xFFC00E48);
		d("DMA9_CURR_ADDR", 32, 0xFFC00E64);
		d("DMA9_CURR_DESC_PTR", 32, 0xFFC00E60);
		d("DMA9_CURR_X_COUNT", 16, 0xFFC00E70);
		d("DMA9_CURR_Y_COUNT", 16, 0xFFC00E78);
		d("DMA9_IRQ_STATUS", 16, 0xFFC00E68);
		d("DMA9_NEXT_DESC_PTR", 32, 0xFFC00E40);
		d("DMA9_PERIPHERAL_MAP", 16, 0xFFC00E6C);
		d("DMA9_START_ADDR", 32, 0xFFC00E44);
		d("DMA9_X_COUNT", 16, 0xFFC00E50);
		d("DMA9_X_MODIFY", 16, 0xFFC00E54);
		d("DMA9_Y_COUNT", 16, 0xFFC00E58);
		d("DMA9_Y_MODIFY", 16, 0xFFC00E5C);

		parent = debugfs_create_dir("DMA Traffic Control", top);
		d("DMA_TC_CNT", 16, 0xFFC00B0C);
		d("DMA_TC_PER", 16, 0xFFC00B10);

		parent = debugfs_create_dir("GPIO PIN", top);
		d("PORTF_DRIVE", 16, 0xFFC03220);
		d("PORTF_HYSTERESIS", 16, 0xFFC03240);
		d("PORTF_MUX", 16, 0xFFC03210);
		d("PORTF_SLEW", 16, 0xFFC03230);
		d("PORTG_DRIVE", 16, 0xFFC03224);
		d("PORTG_HYSTERESIS", 16, 0xFFC03244);
		d("PORTG_MUX", 16, 0xFFC03214);
		d("PORTG_SLEW", 16, 0xFFC03234);
		d("PORTH_DRIVE", 16, 0xFFC03228);
		d("PORTH_HYSTERESIS", 16, 0xFFC03248);
		d("PORTH_MUX", 16, 0xFFC03218);
		d("PORTH_SLEW", 16, 0xFFC03238);

		parent = debugfs_create_dir("Handshake MDMA", top);
		d("HMDMA0_BCINIT", 16, 0xFFC03308);
		d("HMDMA0_BCOUNT", 16, 0xFFC03318);
		d("HMDMA0_CONTROL", 16, 0xFFC03300);
		d("HMDMA0_ECINIT", 16, 0xFFC03304);
		d("HMDMA0_ECOUNT", 16, 0xFFC03314);
		d("HMDMA0_ECOVERFLOW", 16, 0xFFC03310);
		d("HMDMA0_ECURGENT", 16, 0xFFC0330C);
		d("HMDMA1_BCINIT", 16, 0xFFC03348);
		d("HMDMA1_BCOUNT", 16, 0xFFC03358);
		d("HMDMA1_CONTROL", 16, 0xFFC03340);
		d("HMDMA1_ECINIT", 16, 0xFFC03344);
		d("HMDMA1_ECOUNT", 16, 0xFFC03354);
		d("HMDMA1_ECOVERFLOW", 16, 0xFFC03350);
		d("HMDMA1_ECURGENT", 16, 0xFFC0334C);

		parent = debugfs_create_dir("MDMA Destination 0", top);
		d("MDMA_D0_CONFIG", 16, 0xFFC00F08);
		d("MDMA_D0_CURR_ADDR", 32, 0xFFC00F24);
		d("MDMA_D0_CURR_DESC_PTR", 32, 0xFFC00F20);
		d("MDMA_D0_CURR_X_COUNT", 16, 0xFFC00F30);
		d("MDMA_D0_CURR_Y_COUNT", 16, 0xFFC00F38);
		d("MDMA_D0_IRQ_STATUS", 16, 0xFFC00F28);
		d("MDMA_D0_NEXT_DESC_PTR", 32, 0xFFC00F00);
		d("MDMA_D0_PERIPHERAL_MAP", 16, 0xFFC00F2C);
		d("MDMA_D0_START_ADDR", 32, 0xFFC00F04);
		d("MDMA_D0_X_COUNT", 16, 0xFFC00F10);
		d("MDMA_D0_X_MODIFY", 16, 0xFFC00F14);
		d("MDMA_D0_Y_COUNT", 16, 0xFFC00F18);
		d("MDMA_D0_Y_MODIFY", 16, 0xFFC00F1C);

		parent = debugfs_create_dir("MDMA Destination 1", top);
		d("MDMA_D1_CONFIG", 16, 0xFFC00F88);
		d("MDMA_D1_CURR_ADDR", 32, 0xFFC00FA4);
		d("MDMA_D1_CURR_DESC_PTR", 32, 0xFFC00FA0);
		d("MDMA_D1_CURR_X_COUNT", 16, 0xFFC00FB0);
		d("MDMA_D1_CURR_Y_COUNT", 16, 0xFFC00FB8);
		d("MDMA_D1_IRQ_STATUS", 16, 0xFFC00FA8);
		d("MDMA_D1_NEXT_DESC_PTR", 32, 0xFFC00F80);
		d("MDMA_D1_PERIPHERAL_MAP", 16, 0xFFC00FAC);
		d("MDMA_D1_START_ADDR", 32, 0xFFC00F84);
		d("MDMA_D1_X_COUNT", 16, 0xFFC00F90);
		d("MDMA_D1_X_MODIFY", 16, 0xFFC00F94);
		d("MDMA_D1_Y_COUNT", 16, 0xFFC00F98);
		d("MDMA_D1_Y_MODIFY", 16, 0xFFC00F9C);

		parent = debugfs_create_dir("MDMA Source 0", top);
		d("MDMA_S0_CONFIG", 16, 0xFFC00F48);
		d("MDMA_S0_CURR_ADDR", 32, 0xFFC00F64);
		d("MDMA_S0_CURR_DESC_PTR", 32, 0xFFC00F60);
		d("MDMA_S0_CURR_X_COUNT", 16, 0xFFC00F70);
		d("MDMA_S0_CURR_Y_COUNT", 16, 0xFFC00F78);
		d("MDMA_S0_IRQ_STATUS", 16, 0xFFC00F68);
		d("MDMA_S0_NEXT_DESC_PTR", 32, 0xFFC00F40);
		d("MDMA_S0_PERIPHERAL_MAP", 16, 0xFFC00F6C);
		d("MDMA_S0_START_ADDR", 32, 0xFFC00F44);
		d("MDMA_S0_X_COUNT", 16, 0xFFC00F50);
		d("MDMA_S0_X_MODIFY", 16, 0xFFC00F54);
		d("MDMA_S0_Y_COUNT", 16, 0xFFC00F58);
		d("MDMA_S0_Y_MODIFY", 16, 0xFFC00F5C);

		parent = debugfs_create_dir("MDMA Source 1", top);
		d("MDMA_S1_CONFIG", 16, 0xFFC00FC8);
		d("MDMA_S1_CURR_ADDR", 32, 0xFFC00FE4);
		d("MDMA_S1_CURR_DESC_PTR", 32, 0xFFC00FE0);
		d("MDMA_S1_CURR_X_COUNT", 16, 0xFFC00FF0);
		d("MDMA_S1_CURR_Y_COUNT", 16, 0xFFC00FF8);
		d("MDMA_S1_IRQ_STATUS", 16, 0xFFC00FE8);
		d("MDMA_S1_NEXT_DESC_PTR", 32, 0xFFC00FC0);
		d("MDMA_S1_PERIPHERAL_MAP", 16, 0xFFC00FEC);
		d("MDMA_S1_START_ADDR", 32, 0xFFC00FC4);
		d("MDMA_S1_X_COUNT", 16, 0xFFC00FD0);
		d("MDMA_S1_X_MODIFY", 16, 0xFFC00FD4);
		d("MDMA_S1_Y_COUNT", 16, 0xFFC00FD8);
		d("MDMA_S1_Y_MODIFY", 16, 0xFFC00FDC);

		parent = debugfs_create_dir("NON-GPIO", top);
		d("NONGPIO_DRIVE", 16, 0xFFC03280);
		d("NONGPIO_HYSTERESIS", 16, 0xFFC03288);
		d("NONGPIO_SLEW", 16, 0xFFC03284);

		parent = debugfs_create_dir("Pin Control", top);
		d("PORTF_FER", 16, 0xFFC03200);
		d("PORTG_FER", 16, 0xFFC03204);
		d("PORTH_FER", 16, 0xFFC03208);

		parent = debugfs_create_dir("Port I-O", top);
		d("PORTFIO", 16, 0xFFC00700);
		d("PORTFIO_BOTH", 16, 0xFFC0073C);
		d("PORTFIO_CLEAR", 16, 0xFFC00704);
		d("PORTFIO_DIR", 16, 0xFFC00730);
		d("PORTFIO_EDGE", 16, 0xFFC00738);
		d("PORTFIO_INEN", 16, 0xFFC00740);
		d("PORTFIO_MASKA", 16, 0xFFC00710);
		d("PORTFIO_MASKA_CLEAR", 16, 0xFFC00714);
		d("PORTFIO_MASKA_SET", 16, 0xFFC00718);
		d("PORTFIO_MASKA_TOGGLE", 16, 0xFFC0071C);
		d("PORTFIO_MASKB", 16, 0xFFC00720);
		d("PORTFIO_MASKB_CLEAR", 16, 0xFFC00724);
		d("PORTFIO_MASKB_SET", 16, 0xFFC00728);
		d("PORTFIO_MASKB_TOGGLE", 16, 0xFFC0072C);
		d("PORTFIO_POLAR", 16, 0xFFC00734);
		d("PORTFIO_SET", 16, 0xFFC00708);
		d("PORTFIO_TOGGLE", 16, 0xFFC0070C);
		d("PORTGIO", 16, 0xFFC01500);
		d("PORTGIO_BOTH", 16, 0xFFC0153C);
		d("PORTGIO_CLEAR", 16, 0xFFC01504);
		d("PORTGIO_DIR", 16, 0xFFC01530);
		d("PORTGIO_EDGE", 16, 0xFFC01538);
		d("PORTGIO_INEN", 16, 0xFFC01540);
		d("PORTGIO_MASKA", 16, 0xFFC01510);
		d("PORTGIO_MASKA_CLEAR", 16, 0xFFC01514);
		d("PORTGIO_MASKA_SET", 16, 0xFFC01518);
		d("PORTGIO_MASKA_TOGGLE", 16, 0xFFC0151C);
		d("PORTGIO_MASKB", 16, 0xFFC01520);
		d("PORTGIO_MASKB_CLEAR", 16, 0xFFC01524);
		d("PORTGIO_MASKB_SET", 16, 0xFFC01528);
		d("PORTGIO_MASKB_TOGGLE", 16, 0xFFC0152C);
		d("PORTGIO_POLAR", 16, 0xFFC01534);
		d("PORTGIO_SET", 16, 0xFFC01508);
		d("PORTGIO_TOGGLE", 16, 0xFFC0150C);
		d("PORTHIO", 16, 0xFFC01700);
		d("PORTHIO_BOTH", 16, 0xFFC0173C);
		d("PORTHIO_CLEAR", 16, 0xFFC01704);
		d("PORTHIO_DIR", 16, 0xFFC01730);
		d("PORTHIO_EDGE", 16, 0xFFC01738);
		d("PORTHIO_INEN", 16, 0xFFC01740);
		d("PORTHIO_MASKA", 16, 0xFFC01710);
		d("PORTHIO_MASKA_CLEAR", 16, 0xFFC01714);
		d("PORTHIO_MASKA_SET", 16, 0xFFC01718);
		d("PORTHIO_MASKA_TOGGLE", 16, 0xFFC0171C);
		d("PORTHIO_MASKB", 16, 0xFFC01720);
		d("PORTHIO_MASKB_CLEAR", 16, 0xFFC01724);
		d("PORTHIO_MASKB_SET", 16, 0xFFC01728);
		d("PORTHIO_MASKB_TOGGLE", 16, 0xFFC0172C);
		d("PORTHIO_POLAR", 16, 0xFFC01734);
		d("PORTHIO_SET", 16, 0xFFC01708);
		d("PORTHIO_TOGGLE", 16, 0xFFC0170C);

		parent = debugfs_create_dir("SPI", top);
		d("SPI_BAUD", 16, 0xFFC00514);
		d("SPI_CTL", 16, 0xFFC00500);
		d("SPI_FLG", 16, 0xFFC00504);
		d("SPI_RDBR", 16, 0xFFC00510);
		d("SPI_SHADOW", 16, 0xFFC00518);
		d("SPI_STAT", 16, 0xFFC00508);
		d("SPI_TDBR", 16, 0xFFC0050C);

		parent = debugfs_create_dir("TWI", top);
		d("TWI_CLKDIV", 16, 0xFFC01400);
		d("TWI_CONTROL", 16, 0xFFC01404);
		d("TWI_FIFO_CTL", 16, 0xFFC01428);
		d("TWI_FIFO_STAT", 16, 0xFFC0142C);
		d("TWI_INT_MASK", 16, 0xFFC01424);
		d("TWI_INT_STAT", 16, 0xFFC01420);
		d("TWI_MASTER_ADDR", 16, 0xFFC0141C);
		d("TWI_MASTER_CTL", 16, 0xFFC01414);
		d("TWI_MASTER_STAT", 16, 0xFFC01418);
		d("TWI_RCV_DATA16", 16, 0xFFC0148C);
		d("TWI_RCV_DATA8", 16, 0xFFC01488);
		d("TWI_SLAVE_ADDR", 16, 0xFFC01410);
		d("TWI_SLAVE_CTL", 16, 0xFFC01408);
		d("TWI_SLAVE_STAT", 16, 0xFFC0140C);
		d("TWI_XMT_DATA16", 16, 0xFFC01484);
		d("TWI_XMT_DATA8", 16, 0xFFC01480);

		parent = debugfs_create_dir("UART", top);
		d("UART0_DLH", 16, 0xFFC00404);
		d("UART0_DLL", 16, 0xFFC00400);
		d("UART0_GCTL", 16, 0xFFC00424);
		d("UART0_IER", 16, 0xFFC00404);
		d("UART0_IIR", 16, 0xFFC00408);
		d("UART0_LCR", 16, 0xFFC0040C);
		d("UART0_LSR", 16, 0xFFC00414);
		d("UART0_MCR", 16, 0xFFC00410);
		d("UART0_MSR", 16, 0xFFC00418);
		d("UART0_RBR", 16, 0xFFC00400);
		d("UART0_SCR", 16, 0xFFC0041C);
		d("UART0_THR", 16, 0xFFC00400);
		d("UART1_DLH", 16, 0xFFC02004);
		d("UART1_DLL", 16, 0xFFC02000);
		d("UART1_GCTL", 16, 0xFFC02024);
		d("UART1_IER", 16, 0xFFC02004);
		d("UART1_IIR", 16, 0xFFC02008);
		d("UART1_LCR", 16, 0xFFC0200C);
		d("UART1_LSR", 16, 0xFFC02014);
		d("UART1_MCR", 16, 0xFFC02010);
		d("UART1_MSR", 16, 0xFFC02018);
		d("UART1_RBR", 16, 0xFFC02000);
		d("UART1_SCR", 16, 0xFFC0201C);
		d("UART1_THR", 16, 0xFFC02000);

	}	/* BF522 BF523 */

#ifdef __ADSPBF524__
# define USE_BF524 1
#else
# define USE_BF524 0
#endif
#ifdef __ADSPBF525__
# define USE_BF525 1
#else
# define USE_BF525 0
#endif
#ifdef __ADSPBF526__
# define USE_BF526 1
#else
# define USE_BF526 0
#endif
#ifdef __ADSPBF527__
# define USE_BF527 1
#else
# define USE_BF527 0
#endif
	if (USE_BF524 || USE_BF525 || USE_BF526 || USE_BF527) {

		parent = debugfs_create_dir("DMA Controller", top);
		d("DMA0_CONFIG", 16, 0xFFC00C08);
		d("DMA0_CURR_ADDR", 32, 0xFFC00C24);
		d("DMA0_CURR_DESC_PTR", 32, 0xFFC00C20);
		d("DMA0_CURR_X_COUNT", 16, 0xFFC00C30);
		d("DMA0_CURR_Y_COUNT", 16, 0xFFC00C38);
		d("DMA0_IRQ_STATUS", 16, 0xFFC00C28);
		d("DMA0_NEXT_DESC_PTR", 32, 0xFFC00C00);
		d("DMA0_PERIPHERAL_MAP", 16, 0xFFC00C2C);
		d("DMA0_START_ADDR", 32, 0xFFC00C04);
		d("DMA0_X_COUNT", 16, 0xFFC00C10);
		d("DMA0_X_MODIFY", 16, 0xFFC00C14);
		d("DMA0_Y_COUNT", 16, 0xFFC00C18);
		d("DMA0_Y_MODIFY", 16, 0xFFC00C1C);
		d("DMA10_CONFIG", 16, 0xFFC00E88);
		d("DMA10_CURR_ADDR", 32, 0xFFC00EA4);
		d("DMA10_CURR_DESC_PTR", 32, 0xFFC00EA0);
		d("DMA10_CURR_X_COUNT", 16, 0xFFC00EB0);
		d("DMA10_CURR_Y_COUNT", 16, 0xFFC00EB8);
		d("DMA10_IRQ_STATUS", 16, 0xFFC00EA8);
		d("DMA10_NEXT_DESC_PTR", 32, 0xFFC00E80);
		d("DMA10_PERIPHERAL_MAP", 16, 0xFFC00EAC);
		d("DMA10_START_ADDR", 32, 0xFFC00E84);
		d("DMA10_X_COUNT", 16, 0xFFC00E90);
		d("DMA10_X_MODIFY", 16, 0xFFC00E94);
		d("DMA10_Y_COUNT", 16, 0xFFC00E98);
		d("DMA10_Y_MODIFY", 16, 0xFFC00E9C);
		d("DMA11_CONFIG", 16, 0xFFC00EC8);
		d("DMA11_CURR_ADDR", 32, 0xFFC00EE4);
		d("DMA11_CURR_DESC_PTR", 32, 0xFFC00EE0);
		d("DMA11_CURR_X_COUNT", 16, 0xFFC00EF0);
		d("DMA11_CURR_Y_COUNT", 16, 0xFFC00EF8);
		d("DMA11_IRQ_STATUS", 16, 0xFFC00EE8);
		d("DMA11_NEXT_DESC_PTR", 32, 0xFFC00EC0);
		d("DMA11_PERIPHERAL_MAP", 16, 0xFFC00EEC);
		d("DMA11_START_ADDR", 32, 0xFFC00EC4);
		d("DMA11_X_COUNT", 16, 0xFFC00ED0);
		d("DMA11_X_MODIFY", 16, 0xFFC00ED4);
		d("DMA11_Y_COUNT", 16, 0xFFC00ED8);
		d("DMA11_Y_MODIFY", 16, 0xFFC00EDC);
		d("DMA1_CONFIG", 16, 0xFFC00C48);
		d("DMA1_CURR_ADDR", 32, 0xFFC00C64);
		d("DMA1_CURR_DESC_PTR", 32, 0xFFC00C60);
		d("DMA1_CURR_X_COUNT", 16, 0xFFC00C70);
		d("DMA1_CURR_Y_COUNT", 16, 0xFFC00C78);
		d("DMA1_IRQ_STATUS", 16, 0xFFC00C68);
		d("DMA1_NEXT_DESC_PTR", 32, 0xFFC00C40);
		d("DMA1_PERIPHERAL_MAP", 16, 0xFFC00C6C);
		d("DMA1_START_ADDR", 32, 0xFFC00C44);
		d("DMA1_X_COUNT", 16, 0xFFC00C50);
		d("DMA1_X_MODIFY", 16, 0xFFC00C54);
		d("DMA1_Y_COUNT", 16, 0xFFC00C58);
		d("DMA1_Y_MODIFY", 16, 0xFFC00C5C);
		d("DMA2_CONFIG", 16, 0xFFC00C88);
		d("DMA2_CURR_ADDR", 32, 0xFFC00CA4);
		d("DMA2_CURR_DESC_PTR", 32, 0xFFC00CA0);
		d("DMA2_CURR_X_COUNT", 16, 0xFFC00CB0);
		d("DMA2_CURR_Y_COUNT", 16, 0xFFC00CB8);
		d("DMA2_IRQ_STATUS", 16, 0xFFC00CA8);
		d("DMA2_NEXT_DESC_PTR", 32, 0xFFC00C80);
		d("DMA2_PERIPHERAL_MAP", 16, 0xFFC00CAC);
		d("DMA2_START_ADDR", 32, 0xFFC00C84);
		d("DMA2_X_COUNT", 16, 0xFFC00C90);
		d("DMA2_X_MODIFY", 16, 0xFFC00C94);
		d("DMA2_Y_COUNT", 16, 0xFFC00C98);
		d("DMA2_Y_MODIFY", 16, 0xFFC00C9C);
		d("DMA3_CONFIG", 16, 0xFFC00CC8);
		d("DMA3_CURR_ADDR", 32, 0xFFC00CE4);
		d("DMA3_CURR_DESC_PTR", 32, 0xFFC00CE0);
		d("DMA3_CURR_X_COUNT", 16, 0xFFC00CF0);
		d("DMA3_CURR_Y_COUNT", 16, 0xFFC00CF8);
		d("DMA3_IRQ_STATUS", 16, 0xFFC00CE8);
		d("DMA3_NEXT_DESC_PTR", 32, 0xFFC00CC0);
		d("DMA3_PERIPHERAL_MAP", 16, 0xFFC00CEC);
		d("DMA3_START_ADDR", 32, 0xFFC00CC4);
		d("DMA3_X_COUNT", 16, 0xFFC00CD0);
		d("DMA3_X_MODIFY", 16, 0xFFC00CD4);
		d("DMA3_Y_COUNT", 16, 0xFFC00CD8);
		d("DMA3_Y_MODIFY", 16, 0xFFC00CDC);
		d("DMA4_CONFIG", 16, 0xFFC00D08);
		d("DMA4_CURR_ADDR", 32, 0xFFC00D24);
		d("DMA4_CURR_DESC_PTR", 32, 0xFFC00D20);
		d("DMA4_CURR_X_COUNT", 16, 0xFFC00D30);
		d("DMA4_CURR_Y_COUNT", 16, 0xFFC00D38);
		d("DMA4_IRQ_STATUS", 16, 0xFFC00D28);
		d("DMA4_NEXT_DESC_PTR", 32, 0xFFC00D00);
		d("DMA4_PERIPHERAL_MAP", 16, 0xFFC00D2C);
		d("DMA4_START_ADDR", 32, 0xFFC00D04);
		d("DMA4_X_COUNT", 16, 0xFFC00D10);
		d("DMA4_X_MODIFY", 16, 0xFFC00D14);
		d("DMA4_Y_COUNT", 16, 0xFFC00D18);
		d("DMA4_Y_MODIFY", 16, 0xFFC00D1C);
		d("DMA5_CONFIG", 16, 0xFFC00D48);
		d("DMA5_CURR_ADDR", 32, 0xFFC00D64);
		d("DMA5_CURR_DESC_PTR", 32, 0xFFC00D60);
		d("DMA5_CURR_X_COUNT", 16, 0xFFC00D70);
		d("DMA5_CURR_Y_COUNT", 16, 0xFFC00D78);
		d("DMA5_IRQ_STATUS", 16, 0xFFC00D68);
		d("DMA5_NEXT_DESC_PTR", 32, 0xFFC00D40);
		d("DMA5_PERIPHERAL_MAP", 16, 0xFFC00D6C);
		d("DMA5_START_ADDR", 32, 0xFFC00D44);
		d("DMA5_X_COUNT", 16, 0xFFC00D50);
		d("DMA5_X_MODIFY", 16, 0xFFC00D54);
		d("DMA5_Y_COUNT", 16, 0xFFC00D58);
		d("DMA5_Y_MODIFY", 16, 0xFFC00D5C);
		d("DMA6_CONFIG", 16, 0xFFC00D88);
		d("DMA6_CURR_ADDR", 32, 0xFFC00DA4);
		d("DMA6_CURR_DESC_PTR", 32, 0xFFC00DA0);
		d("DMA6_CURR_X_COUNT", 16, 0xFFC00DB0);
		d("DMA6_CURR_Y_COUNT", 16, 0xFFC00DB8);
		d("DMA6_IRQ_STATUS", 16, 0xFFC00DA8);
		d("DMA6_NEXT_DESC_PTR", 32, 0xFFC00D80);
		d("DMA6_PERIPHERAL_MAP", 16, 0xFFC00DAC);
		d("DMA6_START_ADDR", 32, 0xFFC00D84);
		d("DMA6_X_COUNT", 16, 0xFFC00D90);
		d("DMA6_X_MODIFY", 16, 0xFFC00D94);
		d("DMA6_Y_COUNT", 16, 0xFFC00D98);
		d("DMA6_Y_MODIFY", 16, 0xFFC00D9C);
		d("DMA7_CONFIG", 16, 0xFFC00DC8);
		d("DMA7_CURR_ADDR", 32, 0xFFC00DE4);
		d("DMA7_CURR_DESC_PTR", 32, 0xFFC00DE0);
		d("DMA7_CURR_X_COUNT", 16, 0xFFC00DF0);
		d("DMA7_CURR_Y_COUNT", 16, 0xFFC00DF8);
		d("DMA7_IRQ_STATUS", 16, 0xFFC00DE8);
		d("DMA7_NEXT_DESC_PTR", 32, 0xFFC00DC0);
		d("DMA7_PERIPHERAL_MAP", 16, 0xFFC00DEC);
		d("DMA7_START_ADDR", 32, 0xFFC00DC4);
		d("DMA7_X_COUNT", 16, 0xFFC00DD0);
		d("DMA7_X_MODIFY", 16, 0xFFC00DD4);
		d("DMA7_Y_COUNT", 16, 0xFFC00DD8);
		d("DMA7_Y_MODIFY", 16, 0xFFC00DDC);
		d("DMA8_CONFIG", 16, 0xFFC00E08);
		d("DMA8_CURR_ADDR", 32, 0xFFC00E24);
		d("DMA8_CURR_DESC_PTR", 32, 0xFFC00E20);
		d("DMA8_CURR_X_COUNT", 16, 0xFFC00E30);
		d("DMA8_CURR_Y_COUNT", 16, 0xFFC00E38);
		d("DMA8_IRQ_STATUS", 16, 0xFFC00E28);
		d("DMA8_NEXT_DESC_PTR", 32, 0xFFC00E00);
		d("DMA8_PERIPHERAL_MAP", 16, 0xFFC00E2C);
		d("DMA8_START_ADDR", 32, 0xFFC00E04);
		d("DMA8_X_COUNT", 16, 0xFFC00E10);
		d("DMA8_X_MODIFY", 16, 0xFFC00E14);
		d("DMA8_Y_COUNT", 16, 0xFFC00E18);
		d("DMA8_Y_MODIFY", 16, 0xFFC00E1C);
		d("DMA9_CONFIG", 16, 0xFFC00E48);
		d("DMA9_CURR_ADDR", 32, 0xFFC00E64);
		d("DMA9_CURR_DESC_PTR", 32, 0xFFC00E60);
		d("DMA9_CURR_X_COUNT", 16, 0xFFC00E70);
		d("DMA9_CURR_Y_COUNT", 16, 0xFFC00E78);
		d("DMA9_IRQ_STATUS", 16, 0xFFC00E68);
		d("DMA9_NEXT_DESC_PTR", 32, 0xFFC00E40);
		d("DMA9_PERIPHERAL_MAP", 16, 0xFFC00E6C);
		d("DMA9_START_ADDR", 32, 0xFFC00E44);
		d("DMA9_X_COUNT", 16, 0xFFC00E50);
		d("DMA9_X_MODIFY", 16, 0xFFC00E54);
		d("DMA9_Y_COUNT", 16, 0xFFC00E58);
		d("DMA9_Y_MODIFY", 16, 0xFFC00E5C);

		parent = debugfs_create_dir("DMA Traffic Control", top);
		d("DMA_TC_CNT", 16, 0xFFC00B0C);
		d("DMA_TC_PER", 16, 0xFFC00B10);

		parent = debugfs_create_dir("GPIO PIN", top);
		d("PORTF_DRIVE", 16, 0xFFC03220);
		d("PORTF_HYSTERESIS", 16, 0xFFC03240);
		d("PORTF_MUX", 16, 0xFFC03210);
		d("PORTF_SLEW", 16, 0xFFC03230);
		d("PORTG_DRIVE", 16, 0xFFC03224);
		d("PORTG_HYSTERESIS", 16, 0xFFC03244);
		d("PORTG_MUX", 16, 0xFFC03214);
		d("PORTG_SLEW", 16, 0xFFC03234);
		d("PORTH_DRIVE", 16, 0xFFC03228);
		d("PORTH_HYSTERESIS", 16, 0xFFC03248);
		d("PORTH_MUX", 16, 0xFFC03218);
		d("PORTH_SLEW", 16, 0xFFC03238);

		parent = debugfs_create_dir("Handshake MDMA", top);
		d("HMDMA0_BCINIT", 16, 0xFFC03308);
		d("HMDMA0_BCOUNT", 16, 0xFFC03318);
		d("HMDMA0_CONTROL", 16, 0xFFC03300);
		d("HMDMA0_ECINIT", 16, 0xFFC03304);
		d("HMDMA0_ECOUNT", 16, 0xFFC03314);
		d("HMDMA0_ECOVERFLOW", 16, 0xFFC03310);
		d("HMDMA0_ECURGENT", 16, 0xFFC0330C);
		d("HMDMA1_BCINIT", 16, 0xFFC03348);
		d("HMDMA1_BCOUNT", 16, 0xFFC03358);
		d("HMDMA1_CONTROL", 16, 0xFFC03340);
		d("HMDMA1_ECINIT", 16, 0xFFC03344);
		d("HMDMA1_ECOUNT", 16, 0xFFC03354);
		d("HMDMA1_ECOVERFLOW", 16, 0xFFC03350);
		d("HMDMA1_ECURGENT", 16, 0xFFC0334C);

		parent = debugfs_create_dir("MDMA Destination 0", top);
		d("MDMA_D0_CONFIG", 16, 0xFFC00F08);
		d("MDMA_D0_CURR_ADDR", 32, 0xFFC00F24);
		d("MDMA_D0_CURR_DESC_PTR", 32, 0xFFC00F20);
		d("MDMA_D0_CURR_X_COUNT", 16, 0xFFC00F30);
		d("MDMA_D0_CURR_Y_COUNT", 16, 0xFFC00F38);
		d("MDMA_D0_IRQ_STATUS", 16, 0xFFC00F28);
		d("MDMA_D0_NEXT_DESC_PTR", 32, 0xFFC00F00);
		d("MDMA_D0_PERIPHERAL_MAP", 16, 0xFFC00F2C);
		d("MDMA_D0_START_ADDR", 32, 0xFFC00F04);
		d("MDMA_D0_X_COUNT", 16, 0xFFC00F10);
		d("MDMA_D0_X_MODIFY", 16, 0xFFC00F14);
		d("MDMA_D0_Y_COUNT", 16, 0xFFC00F18);
		d("MDMA_D0_Y_MODIFY", 16, 0xFFC00F1C);

		parent = debugfs_create_dir("MDMA Destination 1", top);
		d("MDMA_D1_CONFIG", 16, 0xFFC00F88);
		d("MDMA_D1_CURR_ADDR", 32, 0xFFC00FA4);
		d("MDMA_D1_CURR_DESC_PTR", 32, 0xFFC00FA0);
		d("MDMA_D1_CURR_X_COUNT", 16, 0xFFC00FB0);
		d("MDMA_D1_CURR_Y_COUNT", 16, 0xFFC00FB8);
		d("MDMA_D1_IRQ_STATUS", 16, 0xFFC00FA8);
		d("MDMA_D1_NEXT_DESC_PTR", 32, 0xFFC00F80);
		d("MDMA_D1_PERIPHERAL_MAP", 16, 0xFFC00FAC);
		d("MDMA_D1_START_ADDR", 32, 0xFFC00F84);
		d("MDMA_D1_X_COUNT", 16, 0xFFC00F90);
		d("MDMA_D1_X_MODIFY", 16, 0xFFC00F94);
		d("MDMA_D1_Y_COUNT", 16, 0xFFC00F98);
		d("MDMA_D1_Y_MODIFY", 16, 0xFFC00F9C);

		parent = debugfs_create_dir("MDMA Source 0", top);
		d("MDMA_S0_CONFIG", 16, 0xFFC00F48);
		d("MDMA_S0_CURR_ADDR", 32, 0xFFC00F64);
		d("MDMA_S0_CURR_DESC_PTR", 32, 0xFFC00F60);
		d("MDMA_S0_CURR_X_COUNT", 16, 0xFFC00F70);
		d("MDMA_S0_CURR_Y_COUNT", 16, 0xFFC00F78);
		d("MDMA_S0_IRQ_STATUS", 16, 0xFFC00F68);
		d("MDMA_S0_NEXT_DESC_PTR", 32, 0xFFC00F40);
		d("MDMA_S0_PERIPHERAL_MAP", 16, 0xFFC00F6C);
		d("MDMA_S0_START_ADDR", 32, 0xFFC00F44);
		d("MDMA_S0_X_COUNT", 16, 0xFFC00F50);
		d("MDMA_S0_X_MODIFY", 16, 0xFFC00F54);
		d("MDMA_S0_Y_COUNT", 16, 0xFFC00F58);
		d("MDMA_S0_Y_MODIFY", 16, 0xFFC00F5C);

		parent = debugfs_create_dir("MDMA Source 1", top);
		d("MDMA_S1_CONFIG", 16, 0xFFC00FC8);
		d("MDMA_S1_CURR_ADDR", 32, 0xFFC00FE4);
		d("MDMA_S1_CURR_DESC_PTR", 32, 0xFFC00FE0);
		d("MDMA_S1_CURR_X_COUNT", 16, 0xFFC00FF0);
		d("MDMA_S1_CURR_Y_COUNT", 16, 0xFFC00FF8);
		d("MDMA_S1_IRQ_STATUS", 16, 0xFFC00FE8);
		d("MDMA_S1_NEXT_DESC_PTR", 32, 0xFFC00FC0);
		d("MDMA_S1_PERIPHERAL_MAP", 16, 0xFFC00FEC);
		d("MDMA_S1_START_ADDR", 32, 0xFFC00FC4);
		d("MDMA_S1_X_COUNT", 16, 0xFFC00FD0);
		d("MDMA_S1_X_MODIFY", 16, 0xFFC00FD4);
		d("MDMA_S1_Y_COUNT", 16, 0xFFC00FD8);
		d("MDMA_S1_Y_MODIFY", 16, 0xFFC00FDC);

		parent = debugfs_create_dir("NON-GPIO", top);
		d("NONGPIO_DRIVE", 16, 0xFFC03280);
		d("NONGPIO_HYSTERESIS", 16, 0xFFC03288);
		d("NONGPIO_SLEW", 16, 0xFFC03284);

		parent = debugfs_create_dir("Pin Control", top);
		d("PORTF_FER", 16, 0xFFC03200);
		d("PORTG_FER", 16, 0xFFC03204);
		d("PORTH_FER", 16, 0xFFC03208);

		parent = debugfs_create_dir("Port I-O", top);
		d("PORTFIO", 16, 0xFFC00700);
		d("PORTFIO_BOTH", 16, 0xFFC0073C);
		d("PORTFIO_CLEAR", 16, 0xFFC00704);
		d("PORTFIO_DIR", 16, 0xFFC00730);
		d("PORTFIO_EDGE", 16, 0xFFC00738);
		d("PORTFIO_INEN", 16, 0xFFC00740);
		d("PORTFIO_MASKA", 16, 0xFFC00710);
		d("PORTFIO_MASKA_CLEAR", 16, 0xFFC00714);
		d("PORTFIO_MASKA_SET", 16, 0xFFC00718);
		d("PORTFIO_MASKA_TOGGLE", 16, 0xFFC0071C);
		d("PORTFIO_MASKB", 16, 0xFFC00720);
		d("PORTFIO_MASKB_CLEAR", 16, 0xFFC00724);
		d("PORTFIO_MASKB_SET", 16, 0xFFC00728);
		d("PORTFIO_MASKB_TOGGLE", 16, 0xFFC0072C);
		d("PORTFIO_POLAR", 16, 0xFFC00734);
		d("PORTFIO_SET", 16, 0xFFC00708);
		d("PORTFIO_TOGGLE", 16, 0xFFC0070C);
		d("PORTGIO", 16, 0xFFC01500);
		d("PORTGIO_BOTH", 16, 0xFFC0153C);
		d("PORTGIO_CLEAR", 16, 0xFFC01504);
		d("PORTGIO_DIR", 16, 0xFFC01530);
		d("PORTGIO_EDGE", 16, 0xFFC01538);
		d("PORTGIO_INEN", 16, 0xFFC01540);
		d("PORTGIO_MASKA", 16, 0xFFC01510);
		d("PORTGIO_MASKA_CLEAR", 16, 0xFFC01514);
		d("PORTGIO_MASKA_SET", 16, 0xFFC01518);
		d("PORTGIO_MASKA_TOGGLE", 16, 0xFFC0151C);
		d("PORTGIO_MASKB", 16, 0xFFC01520);
		d("PORTGIO_MASKB_CLEAR", 16, 0xFFC01524);
		d("PORTGIO_MASKB_SET", 16, 0xFFC01528);
		d("PORTGIO_MASKB_TOGGLE", 16, 0xFFC0152C);
		d("PORTGIO_POLAR", 16, 0xFFC01534);
		d("PORTGIO_SET", 16, 0xFFC01508);
		d("PORTGIO_TOGGLE", 16, 0xFFC0150C);
		d("PORTHIO", 16, 0xFFC01700);
		d("PORTHIO_BOTH", 16, 0xFFC0173C);
		d("PORTHIO_CLEAR", 16, 0xFFC01704);
		d("PORTHIO_DIR", 16, 0xFFC01730);
		d("PORTHIO_EDGE", 16, 0xFFC01738);
		d("PORTHIO_INEN", 16, 0xFFC01740);
		d("PORTHIO_MASKA", 16, 0xFFC01710);
		d("PORTHIO_MASKA_CLEAR", 16, 0xFFC01714);
		d("PORTHIO_MASKA_SET", 16, 0xFFC01718);
		d("PORTHIO_MASKA_TOGGLE", 16, 0xFFC0171C);
		d("PORTHIO_MASKB", 16, 0xFFC01720);
		d("PORTHIO_MASKB_CLEAR", 16, 0xFFC01724);
		d("PORTHIO_MASKB_SET", 16, 0xFFC01728);
		d("PORTHIO_MASKB_TOGGLE", 16, 0xFFC0172C);
		d("PORTHIO_POLAR", 16, 0xFFC01734);
		d("PORTHIO_SET", 16, 0xFFC01708);
		d("PORTHIO_TOGGLE", 16, 0xFFC0170C);

		parent = debugfs_create_dir("SPI", top);
		d("SPI_BAUD", 16, 0xFFC00514);
		d("SPI_CTL", 16, 0xFFC00500);
		d("SPI_FLG", 16, 0xFFC00504);
		d("SPI_RDBR", 16, 0xFFC00510);
		d("SPI_SHADOW", 16, 0xFFC00518);
		d("SPI_STAT", 16, 0xFFC00508);
		d("SPI_TDBR", 16, 0xFFC0050C);

		parent = debugfs_create_dir("TWI", top);
		d("TWI_CLKDIV", 16, 0xFFC01400);
		d("TWI_CONTROL", 16, 0xFFC01404);
		d("TWI_FIFO_CTL", 16, 0xFFC01428);
		d("TWI_FIFO_STAT", 16, 0xFFC0142C);
		d("TWI_INT_MASK", 16, 0xFFC01424);
		d("TWI_INT_STAT", 16, 0xFFC01420);
		d("TWI_MASTER_ADDR", 16, 0xFFC0141C);
		d("TWI_MASTER_CTL", 16, 0xFFC01414);
		d("TWI_MASTER_STAT", 16, 0xFFC01418);
		d("TWI_RCV_DATA16", 16, 0xFFC0148C);
		d("TWI_RCV_DATA8", 16, 0xFFC01488);
		d("TWI_SLAVE_ADDR", 16, 0xFFC01410);
		d("TWI_SLAVE_CTL", 16, 0xFFC01408);
		d("TWI_SLAVE_STAT", 16, 0xFFC0140C);
		d("TWI_XMT_DATA16", 16, 0xFFC01484);
		d("TWI_XMT_DATA8", 16, 0xFFC01480);

		parent = debugfs_create_dir("UART", top);
		d("UART0_DLH", 16, 0xFFC00404);
		d("UART0_DLL", 16, 0xFFC00400);
		d("UART0_GCTL", 16, 0xFFC00424);
		d("UART0_IER", 16, 0xFFC00404);
		d("UART0_IIR", 16, 0xFFC00408);
		d("UART0_LCR", 16, 0xFFC0040C);
		d("UART0_LSR", 16, 0xFFC00414);
		d("UART0_MCR", 16, 0xFFC00410);
		d("UART0_MSR", 16, 0xFFC00418);
		d("UART0_RBR", 16, 0xFFC00400);
		d("UART0_SCR", 16, 0xFFC0041C);
		d("UART0_THR", 16, 0xFFC00400);
		d("UART1_DLH", 16, 0xFFC02004);
		d("UART1_DLL", 16, 0xFFC02000);
		d("UART1_GCTL", 16, 0xFFC02024);
		d("UART1_IER", 16, 0xFFC02004);
		d("UART1_IIR", 16, 0xFFC02008);
		d("UART1_LCR", 16, 0xFFC0200C);
		d("UART1_LSR", 16, 0xFFC02014);
		d("UART1_MCR", 16, 0xFFC02010);
		d("UART1_MSR", 16, 0xFFC02018);
		d("UART1_RBR", 16, 0xFFC02000);
		d("UART1_SCR", 16, 0xFFC0201C);
		d("UART1_THR", 16, 0xFFC02000);

		parent = debugfs_create_dir("USB", top);
		d("USB_APHY_CALIB", 16, 0xFFC039E4);
		d("USB_APHY_CNTRL2", 16, 0xFFC039E8);
		d("USB_APHY_CNTRL", 16, 0xFFC039E0);
		d("USB_COUNT0", 16, 0xFFC03850);
		d("USB_CSR0", 16, 0xFFC03844);
		d("USB_DMA0_ADDRHIGH", 16, 0xFFC03C0C);
		d("USB_DMA0_ADDRLOW", 16, 0xFFC03C08);
		d("USB_DMA0_CONTROL", 16, 0xFFC03C04);
		d("USB_DMA0_COUNTHIGH", 16, 0xFFC03C14);
		d("USB_DMA0_COUNTLOW", 16, 0xFFC03C10);
		d("USB_DMA1_ADDRHIGH", 16, 0xFFC03C2C);
		d("USB_DMA1_ADDRLOW", 16, 0xFFC03C28);
		d("USB_DMA1_CONTROL", 16, 0xFFC03C24);
		d("USB_DMA1_COUNTHIGH", 16, 0xFFC03C34);
		d("USB_DMA1_COUNTLOW", 16, 0xFFC03C30);
		d("USB_DMA2_ADDRHIGH", 16, 0xFFC03C4C);
		d("USB_DMA2_ADDRLOW", 16, 0xFFC03C48);
		d("USB_DMA2_CONTROL", 16, 0xFFC03C44);
		d("USB_DMA2_COUNTHIGH", 16, 0xFFC03C54);
		d("USB_DMA2_COUNTLOW", 16, 0xFFC03C50);
		d("USB_DMA3_ADDRHIGH", 16, 0xFFC03C6C);
		d("USB_DMA3_ADDRLOW", 16, 0xFFC03C68);
		d("USB_DMA3_CONTROL", 16, 0xFFC03C64);
		d("USB_DMA3_COUNTHIGH", 16, 0xFFC03C74);
		d("USB_DMA3_COUNTLOW", 16, 0xFFC03C70);
		d("USB_DMA4_ADDRHIGH", 16, 0xFFC03C8C);
		d("USB_DMA4_ADDRLOW", 16, 0xFFC03C88);
		d("USB_DMA4_CONTROL", 16, 0xFFC03C84);
		d("USB_DMA4_COUNTHIGH", 16, 0xFFC03C94);
		d("USB_DMA4_COUNTLOW", 16, 0xFFC03C90);
		d("USB_DMA5_ADDRHIGH", 16, 0xFFC03CAC);
		d("USB_DMA5_ADDRLOW", 16, 0xFFC03CA8);
		d("USB_DMA5_CONTROL", 16, 0xFFC03CA4);
		d("USB_DMA5_COUNTHIGH", 16, 0xFFC03CB4);
		d("USB_DMA5_COUNTLOW", 16, 0xFFC03CB0);
		d("USB_DMA6_ADDRHIGH", 16, 0xFFC03CCC);
		d("USB_DMA6_ADDRLOW", 16, 0xFFC03CC8);
		d("USB_DMA6_CONTROL", 16, 0xFFC03CC4);
		d("USB_DMA6_COUNTHIGH", 16, 0xFFC03CD4);
		d("USB_DMA6_COUNTLOW", 16, 0xFFC03CD0);
		d("USB_DMA7_ADDRHIGH", 16, 0xFFC03CEC);
		d("USB_DMA7_ADDRLOW", 16, 0xFFC03CE8);
		d("USB_DMA7_CONTROL", 16, 0xFFC03CE4);
		d("USB_DMA7_COUNTHIGH", 16, 0xFFC03CF4);
		d("USB_DMA7_COUNTLOW", 16, 0xFFC03CF0);
		d("USB_DMA_INTERRUPT", 16, 0xFFC03C00);
		d("USB_EP0_FIFO", 16, 0xFFC03880);
		d("USB_EP1_FIFO", 16, 0xFFC03888);
		d("USB_EP2_FIFO", 16, 0xFFC03890);
		d("USB_EP3_FIFO", 16, 0xFFC03898);
		d("USB_EP4_FIFO", 16, 0xFFC038A0);
		d("USB_EP5_FIFO", 16, 0xFFC038A8);
		d("USB_EP6_FIFO", 16, 0xFFC038B0);
		d("USB_EP7_FIFO", 16, 0xFFC038B8);
		d("USB_EP_NI0_RXCOUNT", 16, 0xFFC03A10);
		d("USB_EP_NI0_RXCSR", 16, 0xFFC03A0C);
		d("USB_EP_NI0_RXINTERVAL", 16, 0xFFC03A20);
		d("USB_EP_NI0_RXMAXP", 16, 0xFFC03A08);
		d("USB_EP_NI0_RXTYPE", 16, 0xFFC03A1C);
		d("USB_EP_NI0_TXCOUNT", 16, 0xFFC03A28);
		d("USB_EP_NI0_TXCSR", 16, 0xFFC03A04);
		d("USB_EP_NI0_TXINTERVAL", 16, 0xFFC03A18);
		d("USB_EP_NI0_TXMAXP", 16, 0xFFC03A00);
		d("USB_EP_NI0_TXTYPE", 16, 0xFFC03A14);
		d("USB_EP_NI1_RXCOUNT", 16, 0xFFC03A50);
		d("USB_EP_NI1_RXCSR", 16, 0xFFC03A4C);
		d("USB_EP_NI1_RXINTERVAL", 16, 0xFFC03A60);
		d("USB_EP_NI1_RXMAXP", 16, 0xFFC03A48);
		d("USB_EP_NI1_RXTYPE", 16, 0xFFC03A5C);
		d("USB_EP_NI1_TXCOUNT", 16, 0xFFC03A68);
		d("USB_EP_NI1_TXCSR", 16, 0xFFC03A44);
		d("USB_EP_NI1_TXINTERVAL", 16, 0xFFC03A58);
		d("USB_EP_NI1_TXMAXP", 16, 0xFFC03A40);
		d("USB_EP_NI1_TXTYPE", 16, 0xFFC03A54);
		d("USB_EP_NI2_RXCOUNT", 16, 0xFFC03A90);
		d("USB_EP_NI2_RXCSR", 16, 0xFFC03A8C);
		d("USB_EP_NI2_RXINTERVAL", 16, 0xFFC03AA0);
		d("USB_EP_NI2_RXMAXP", 16, 0xFFC03A88);
		d("USB_EP_NI2_RXTYPE", 16, 0xFFC03A9C);
		d("USB_EP_NI2_TXCOUNT", 16, 0xFFC03AA8);
		d("USB_EP_NI2_TXCSR", 16, 0xFFC03A84);
		d("USB_EP_NI2_TXINTERVAL", 16, 0xFFC03A98);
		d("USB_EP_NI2_TXMAXP", 16, 0xFFC03A80);
		d("USB_EP_NI2_TXTYPE", 16, 0xFFC03A94);
		d("USB_EP_NI3_RXCOUNT", 16, 0xFFC03AD0);
		d("USB_EP_NI3_RXCSR", 16, 0xFFC03ACC);
		d("USB_EP_NI3_RXINTERVAL", 16, 0xFFC03AE0);
		d("USB_EP_NI3_RXMAXP", 16, 0xFFC03AC8);
		d("USB_EP_NI3_RXTYPE", 16, 0xFFC03ADC);
		d("USB_EP_NI3_TXCOUNT", 16, 0xFFC03AE8);
		d("USB_EP_NI3_TXCSR", 16, 0xFFC03AC4);
		d("USB_EP_NI3_TXINTERVAL", 16, 0xFFC03AD8);
		d("USB_EP_NI3_TXMAXP", 16, 0xFFC03AC0);
		d("USB_EP_NI3_TXTYPE", 16, 0xFFC03AD4);
		d("USB_EP_NI4_RXCOUNT", 16, 0xFFC03B10);
		d("USB_EP_NI4_RXCSR", 16, 0xFFC03B0C);
		d("USB_EP_NI4_RXINTERVAL", 16, 0xFFC03B20);
		d("USB_EP_NI4_RXMAXP", 16, 0xFFC03B08);
		d("USB_EP_NI4_RXTYPE", 16, 0xFFC03B1C);
		d("USB_EP_NI4_TXCOUNT", 16, 0xFFC03B28);
		d("USB_EP_NI4_TXCSR", 16, 0xFFC03B04);
		d("USB_EP_NI4_TXINTERVAL", 16, 0xFFC03B18);
		d("USB_EP_NI4_TXMAXP", 16, 0xFFC03B00);
		d("USB_EP_NI4_TXTYPE", 16, 0xFFC03B14);
		d("USB_EP_NI5_RXCOUNT", 16, 0xFFC03B50);
		d("USB_EP_NI5_RXCSR", 16, 0xFFC03B4C);
		d("USB_EP_NI5_RXINTERVAL", 16, 0xFFC03B60);
		d("USB_EP_NI5_RXMAXP", 16, 0xFFC03B48);
		d("USB_EP_NI5_RXTYPE", 16, 0xFFC03B5C);
		d("USB_EP_NI5_TXCOUNT", 16, 0xFFC03B68);
		d("USB_EP_NI5_TXCSR", 16, 0xFFC03B44);
		d("USB_EP_NI5_TXINTERVAL", 16, 0xFFC03B58);
		d("USB_EP_NI5_TXMAXP", 16, 0xFFC03B40);
		d("USB_EP_NI5_TXTYPE", 16, 0xFFC03B54);
		d("USB_EP_NI6_RXCOUNT", 16, 0xFFC03B90);
		d("USB_EP_NI6_RXCSR", 16, 0xFFC03B8C);
		d("USB_EP_NI6_RXINTERVAL", 16, 0xFFC03BA0);
		d("USB_EP_NI6_RXMAXP", 16, 0xFFC03B88);
		d("USB_EP_NI6_RXTYPE", 16, 0xFFC03B9C);
		d("USB_EP_NI6_TXCOUNT", 16, 0xFFC03BA8);
		d("USB_EP_NI6_TXCSR", 16, 0xFFC03B84);
		d("USB_EP_NI6_TXINTERVAL", 16, 0xFFC03B98);
		d("USB_EP_NI6_TXMAXP", 16, 0xFFC03B80);
		d("USB_EP_NI6_TXTYPE", 16, 0xFFC03B94);
		d("USB_EP_NI7_RXCOUNT", 16, 0xFFC03BD0);
		d("USB_EP_NI7_RXCSR", 16, 0xFFC03BCC);
		d("USB_EP_NI7_RXINTERVAL", 16, 0xFFC03BE0);
		d("USB_EP_NI7_RXMAXP", 16, 0xFFC03BC8);
		d("USB_EP_NI7_RXTYPE", 16, 0xFFC03BDC);
		d("USB_EP_NI7_TXCOUNT", 16, 0xFFC03BE8);
		d("USB_EP_NI7_TXCSR", 16, 0xFFC03BC4);
		d("USB_EP_NI7_TXINTERVAL", 16, 0xFFC03BD8);
		d("USB_EP_NI7_TXMAXP", 16, 0xFFC03BC0);
		d("USB_EP_NI7_TXTYPE", 16, 0xFFC03BD4);
		d("USB_FADDR", 16, 0xFFC03800);
		d("USB_FRAME", 16, 0xFFC03820);
		d("USB_FS_EOF1", 16, 0xFFC03954);
		d("USB_GLOBAL_CTL", 16, 0xFFC03830);
		d("USB_GLOBINTR", 16, 0xFFC0382C);
		d("USB_HS_EOF1", 16, 0xFFC03950);
		d("USB_INDEX", 16, 0xFFC03824);
		d("USB_INTRRX", 16, 0xFFC0380C);
		d("USB_INTRRXE", 16, 0xFFC03814);
		d("USB_INTRTX", 16, 0xFFC03808);
		d("USB_INTRTXE", 16, 0xFFC03810);
		d("USB_INTRUSB", 16, 0xFFC03818);
		d("USB_INTRUSBE", 16, 0xFFC0381C);
		d("USB_LINKINFO", 16, 0xFFC03948);
		d("USB_LS_EOF1", 16, 0xFFC03958);
		d("USB_NAKLIMIT0", 16, 0xFFC03858);
		d("USB_OTG_DEV_CTL", 16, 0xFFC03900);
		d("USB_OTG_VBUS_IRQ", 16, 0xFFC03904);
		d("USB_OTG_VBUS_MASK", 16, 0xFFC03908);
		d("USB_PHY_TEST", 16, 0xFFC039EC);
		d("USB_PLLOSC_CTRL", 16, 0xFFC039F0);
		d("USB_POWER", 16, 0xFFC03804);
		d("USB_RXCOUNT", 16, 0xFFC03850);
		d("USB_RXCSR", 16, 0xFFC0384C);
		d("USB_RXINTERVAL", 16, 0xFFC03860);
		d("USB_RXTYPE", 16, 0xFFC0385C);
		d("USB_RX_MAX_PACKET", 16, 0xFFC03848);
		d("USB_SRP_CLKDIV", 16, 0xFFC039F4);
		d("USB_TESTMODE", 16, 0xFFC03828);
		d("USB_TXCOUNT", 16, 0xFFC03868);
		d("USB_TXCSR", 16, 0xFFC03844);
		d("USB_TXINTERVAL", 16, 0xFFC03858);
		d("USB_TXTYPE", 16, 0xFFC03854);
		d("USB_TX_MAX_PACKET", 16, 0xFFC03840);
		d("USB_VPLEN", 16, 0xFFC0394C);

	}	/* BF524 BF525 BF526 BF527 */

#ifdef __ADSPBF531__
# define USE_BF531 1
#else
# define USE_BF531 0
#endif
#ifdef __ADSPBF532__
# define USE_BF532 1
#else
# define USE_BF532 0
#endif
#ifdef __ADSPBF533__
# define USE_BF533 1
#else
# define USE_BF533 0
#endif
	if (USE_BF531 || USE_BF532 || USE_BF533) {

		parent = debugfs_create_dir("DMA Traffic Control", top);
		d("DMA_TC_CNT", 16, 0xFFC00B0C);
		d("DMA_TC_PER", 16, 0xFFC00B10);

		parent = debugfs_create_dir("DMAFlex Ch-0 Registers", top);
		d("DMA0_CURR_ADDR", 32, 0xFFC00C24);
		d("DMA0_CURR_DESC_PTR", 32, 0xFFC00C20);
		d("DMA0_NEXT_DESC_PTR", 32, 0xFFC00C00);
		d("DMA0_START_ADDR", 32, 0xFFC00C04);
		d("DMAFLX0_CURXCOUNT", 16, 0xFFC00C30);
		d("DMAFLX0_CURYCOUNT", 16, 0xFFC00C38);
		d("DMAFLX0_DMACNFG", 16, 0xFFC00C08);
		d("DMAFLX0_IRQSTAT", 16, 0xFFC00C28);
		d("DMAFLX0_PMAP", 16, 0xFFC00C2C);
		d("DMAFLX0_XCOUNT", 16, 0xFFC00C10);
		d("DMAFLX0_XMODIFY", 16, 0xFFC00C14);
		d("DMAFLX0_YCOUNT", 16, 0xFFC00C18);
		d("DMAFLX0_YMODIFY", 16, 0xFFC00C1C);

		parent = debugfs_create_dir("DMAFlex Ch-1 Registers", top);
		d("DMA1_CURR_ADDR", 32, 0xFFC00C64);
		d("DMA1_CURR_DESC_PTR", 32, 0xFFC00C60);
		d("DMA1_NEXT_DESC_PTR", 32, 0xFFC00C40);
		d("DMA1_START_ADDR", 32, 0xFFC00C44);
		d("DMAFLX1_CURXCOUNT", 16, 0xFFC00C70);
		d("DMAFLX1_CURYCOUNT", 16, 0xFFC00C78);
		d("DMAFLX1_DMACNFG", 16, 0xFFC00C48);
		d("DMAFLX1_IRQSTAT", 16, 0xFFC00C68);
		d("DMAFLX1_PMAP", 16, 0xFFC00C6C);
		d("DMAFLX1_XCOUNT", 16, 0xFFC00C50);
		d("DMAFLX1_XMODIFY", 16, 0xFFC00C54);
		d("DMAFLX1_YCOUNT", 16, 0xFFC00C58);
		d("DMAFLX1_YMODIFY", 16, 0xFFC00C5C);

		parent = debugfs_create_dir("DMAFlex Ch-2 Registers", top);
		d("DMA2_CURR_ADDR", 32, 0xFFC00CA4);
		d("DMA2_CURR_DESC_PTR", 32, 0xFFC00CA0);
		d("DMA2_NEXT_DESC_PTR", 32, 0xFFC00C80);
		d("DMA2_START_ADDR", 32, 0xFFC00C84);
		d("DMAFLX2_CURXCOUNT", 16, 0xFFC00CB0);
		d("DMAFLX2_CURYCOUNT", 16, 0xFFC00CB8);
		d("DMAFLX2_DMACNFG", 16, 0xFFC00C88);
		d("DMAFLX2_IRQSTAT", 16, 0xFFC00CA8);
		d("DMAFLX2_PMAP", 16, 0xFFC00CAC);
		d("DMAFLX2_XCOUNT", 16, 0xFFC00C90);
		d("DMAFLX2_XMODIFY", 16, 0xFFC00C94);
		d("DMAFLX2_YCOUNT", 16, 0xFFC00C98);
		d("DMAFLX2_YMODIFY", 16, 0xFFC00C9C);

		parent = debugfs_create_dir("DMAFlex Ch-3 Registers", top);
		d("DMA3_CURR_ADDR", 32, 0xFFC00CE4);
		d("DMA3_CURR_DESC_PTR", 32, 0xFFC00CE0);
		d("DMA3_NEXT_DESC_PTR", 32, 0xFFC00CC0);
		d("DMA3_START_ADDR", 32, 0xFFC00CC4);
		d("DMAFLX3_CURXCOUNT", 16, 0xFFC00CF0);
		d("DMAFLX3_CURYCOUNT", 16, 0xFFC00CF8);
		d("DMAFLX3_DMACNFG", 16, 0xFFC00CC8);
		d("DMAFLX3_IRQSTAT", 16, 0xFFC00CE8);
		d("DMAFLX3_PMAP", 16, 0xFFC00CEC);
		d("DMAFLX3_XCOUNT", 16, 0xFFC00CD0);
		d("DMAFLX3_XMODIFY", 16, 0xFFC00CD4);
		d("DMAFLX3_YCOUNT", 16, 0xFFC00CD8);
		d("DMAFLX3_YMODIFY", 16, 0xFFC00CDC);

		parent = debugfs_create_dir("DMAFlex Ch-4 Registers", top);
		d("DMA4_CURR_ADDR", 32, 0xFFC00D24);
		d("DMA4_CURR_DESC_PTR", 32, 0xFFC00D20);
		d("DMA4_NEXT_DESC_PTR", 32, 0xFFC00D00);
		d("DMA4_START_ADDR", 32, 0xFFC00D04);
		d("DMAFLX4_CURXCOUNT", 16, 0xFFC00D30);
		d("DMAFLX4_CURYCOUNT", 16, 0xFFC00D38);
		d("DMAFLX4_DMACNFG", 16, 0xFFC00D08);
		d("DMAFLX4_IRQSTAT", 16, 0xFFC00D28);
		d("DMAFLX4_PMAP", 16, 0xFFC00D2C);
		d("DMAFLX4_XCOUNT", 16, 0xFFC00D10);
		d("DMAFLX4_XMODIFY", 16, 0xFFC00D14);
		d("DMAFLX4_YCOUNT", 16, 0xFFC00D18);
		d("DMAFLX4_YMODIFY", 16, 0xFFC00D1C);

		parent = debugfs_create_dir("DMAFlex Ch-5 Registers", top);
		d("DMA5_CURR_ADDR", 32, 0xFFC00D64);
		d("DMA5_CURR_DESC_PTR", 32, 0xFFC00D60);
		d("DMA5_NEXT_DESC_PTR", 32, 0xFFC00D40);
		d("DMA5_START_ADDR", 32, 0xFFC00D44);
		d("DMAFLX5_CURXCOUNT", 16, 0xFFC00D70);
		d("DMAFLX5_CURYCOUNT", 16, 0xFFC00D78);
		d("DMAFLX5_DMACNFG", 16, 0xFFC00D48);
		d("DMAFLX5_IRQSTAT", 16, 0xFFC00D68);
		d("DMAFLX5_PMAP", 16, 0xFFC00D6C);
		d("DMAFLX5_XCOUNT", 16, 0xFFC00D50);
		d("DMAFLX5_XMODIFY", 16, 0xFFC00D54);
		d("DMAFLX5_YCOUNT", 16, 0xFFC00D58);
		d("DMAFLX5_YMODIFY", 16, 0xFFC00D5C);

		parent = debugfs_create_dir("DMAFlex Ch-6 Registers", top);
		d("DMA6_CURR_ADDR", 32, 0xFFC00DA4);
		d("DMA6_CURR_DESC_PTR", 32, 0xFFC00DA0);
		d("DMA6_NEXT_DESC_PTR", 32, 0xFFC00D80);
		d("DMA6_START_ADDR", 32, 0xFFC00D84);
		d("DMAFLX6_CURXCOUNT", 16, 0xFFC00DB0);
		d("DMAFLX6_CURYCOUNT", 16, 0xFFC00DB8);
		d("DMAFLX6_DMACNFG", 16, 0xFFC00D88);
		d("DMAFLX6_IRQSTAT", 16, 0xFFC00DA8);
		d("DMAFLX6_PMAP", 16, 0xFFC00DAC);
		d("DMAFLX6_XCOUNT", 16, 0xFFC00D90);
		d("DMAFLX6_XMODIFY", 16, 0xFFC00D94);
		d("DMAFLX6_YCOUNT", 16, 0xFFC00D98);
		d("DMAFLX6_YMODIFY", 16, 0xFFC00D9C);

		parent = debugfs_create_dir("DMAFlex Ch-7 Registers", top);
		d("DMA7_CURR_ADDR", 32, 0xFFC00DE4);
		d("DMA7_CURR_DESC_PTR", 32, 0xFFC00DE0);
		d("DMA7_NEXT_DESC_PTR", 32, 0xFFC00DC0);
		d("DMA7_START_ADDR", 32, 0xFFC00DC4);
		d("DMAFLX7_CURXCOUNT", 16, 0xFFC00DF0);
		d("DMAFLX7_CURYCOUNT", 16, 0xFFC00DF8);
		d("DMAFLX7_DMACNFG", 16, 0xFFC00DC8);
		d("DMAFLX7_IRQSTAT", 16, 0xFFC00DE8);
		d("DMAFLX7_PMAP", 16, 0xFFC00DEC);
		d("DMAFLX7_XCOUNT", 16, 0xFFC00DD0);
		d("DMAFLX7_XMODIFY", 16, 0xFFC00DD4);
		d("DMAFLX7_YCOUNT", 16, 0xFFC00DD8);
		d("DMAFLX7_YMODIFY", 16, 0xFFC00DDC);

		parent = debugfs_create_dir("Extended Registers", top);
		d("DMA0_CONFIG", 16, 0xFFC00C08);
		d("DMA0_CURR_X_COUNT", 16, 0xFFC00C30);
		d("DMA0_CURR_Y_COUNT", 16, 0xFFC00C38);
		d("DMA0_IRQ_STATUS", 16, 0xFFC00C28);
		d("DMA0_PERIPHERAL_MAP", 16, 0xFFC00C2C);
		d("DMA0_X_COUNT", 16, 0xFFC00C10);
		d("DMA0_X_MODIFY", 16, 0xFFC00C14);
		d("DMA0_Y_COUNT", 16, 0xFFC00C18);
		d("DMA0_Y_MODIFY", 16, 0xFFC00C1C);
		d("DMA1_CONFIG", 16, 0xFFC00C48);
		d("DMA1_CURR_X_COUNT", 16, 0xFFC00C70);
		d("DMA1_CURR_Y_COUNT", 16, 0xFFC00C78);
		d("DMA1_IRQ_STATUS", 16, 0xFFC00C68);
		d("DMA1_PERIPHERAL_MAP", 16, 0xFFC00C6C);
		d("DMA1_X_COUNT", 16, 0xFFC00C50);
		d("DMA1_X_MODIFY", 16, 0xFFC00C54);
		d("DMA1_Y_COUNT", 16, 0xFFC00C58);
		d("DMA1_Y_MODIFY", 16, 0xFFC00C5C);
		d("DMA2_CONFIG", 16, 0xFFC00C88);
		d("DMA2_CURR_X_COUNT", 16, 0xFFC00CB0);
		d("DMA2_CURR_Y_COUNT", 16, 0xFFC00CB8);
		d("DMA2_IRQ_STATUS", 16, 0xFFC00CA8);
		d("DMA2_PERIPHERAL_MAP", 16, 0xFFC00CAC);
		d("DMA2_X_COUNT", 16, 0xFFC00C90);
		d("DMA2_X_MODIFY", 16, 0xFFC00C94);
		d("DMA2_Y_COUNT", 16, 0xFFC00C98);
		d("DMA2_Y_MODIFY", 16, 0xFFC00C9C);
		d("DMA3_CONFIG", 16, 0xFFC00CC8);
		d("DMA3_CURR_X_COUNT", 16, 0xFFC00CF0);
		d("DMA3_CURR_Y_COUNT", 16, 0xFFC00CF8);
		d("DMA3_IRQ_STATUS", 16, 0xFFC00CE8);
		d("DMA3_PERIPHERAL_MAP", 16, 0xFFC00CEC);
		d("DMA3_X_COUNT", 16, 0xFFC00CD0);
		d("DMA3_X_MODIFY", 16, 0xFFC00CD4);
		d("DMA3_Y_COUNT", 16, 0xFFC00CD8);
		d("DMA3_Y_MODIFY", 16, 0xFFC00CDC);
		d("DMA4_CONFIG", 16, 0xFFC00D08);
		d("DMA4_CURR_X_COUNT", 16, 0xFFC00D30);
		d("DMA4_CURR_Y_COUNT", 16, 0xFFC00D38);
		d("DMA4_IRQ_STATUS", 16, 0xFFC00D28);
		d("DMA4_PERIPHERAL_MAP", 16, 0xFFC00D2C);
		d("DMA4_X_COUNT", 16, 0xFFC00D10);
		d("DMA4_X_MODIFY", 16, 0xFFC00D14);
		d("DMA4_Y_COUNT", 16, 0xFFC00D18);
		d("DMA4_Y_MODIFY", 16, 0xFFC00D1C);
		d("DMA5_CONFIG", 16, 0xFFC00D48);
		d("DMA5_CURR_X_COUNT", 16, 0xFFC00D70);
		d("DMA5_CURR_Y_COUNT", 16, 0xFFC00D78);
		d("DMA5_IRQ_STATUS", 16, 0xFFC00D68);
		d("DMA5_PERIPHERAL_MAP", 16, 0xFFC00D6C);
		d("DMA5_X_COUNT", 16, 0xFFC00D50);
		d("DMA5_X_MODIFY", 16, 0xFFC00D54);
		d("DMA5_Y_COUNT", 16, 0xFFC00D58);
		d("DMA5_Y_MODIFY", 16, 0xFFC00D5C);
		d("DMA6_CONFIG", 16, 0xFFC00D88);
		d("DMA6_CURR_X_COUNT", 16, 0xFFC00DB0);
		d("DMA6_CURR_Y_COUNT", 16, 0xFFC00DB8);
		d("DMA6_IRQ_STATUS", 16, 0xFFC00DA8);
		d("DMA6_PERIPHERAL_MAP", 16, 0xFFC00DAC);
		d("DMA6_X_COUNT", 16, 0xFFC00D90);
		d("DMA6_X_MODIFY", 16, 0xFFC00D94);
		d("DMA6_Y_COUNT", 16, 0xFFC00D98);
		d("DMA6_Y_MODIFY", 16, 0xFFC00D9C);
		d("DMA7_CONFIG", 16, 0xFFC00DC8);
		d("DMA7_CURR_X_COUNT", 16, 0xFFC00DF0);
		d("DMA7_CURR_Y_COUNT", 16, 0xFFC00DF8);
		d("DMA7_IRQ_STATUS", 16, 0xFFC00DE8);
		d("DMA7_PERIPHERAL_MAP", 16, 0xFFC00DEC);
		d("DMA7_X_COUNT", 16, 0xFFC00DD0);
		d("DMA7_X_MODIFY", 16, 0xFFC00DD4);
		d("DMA7_Y_COUNT", 16, 0xFFC00DD8);
		d("DMA7_Y_MODIFY", 16, 0xFFC00DDC);
		d("FIO_BOTH", 16, 0xFFC0073C);
		d("FIO_DIR", 16, 0xFFC00730);
		d("FIO_EDGE", 16, 0xFFC00738);
		d("FIO_FLAG_C", 16, 0xFFC00704);
		d("FIO_FLAG_D", 16, 0xFFC00700);
		d("FIO_FLAG_S", 16, 0xFFC00708);
		d("FIO_FLAG_T", 16, 0xFFC0070C);
		d("FIO_INEN", 16, 0xFFC00740);
		d("FIO_MASKA_C", 16, 0xFFC00714);
		d("FIO_MASKA_D", 16, 0xFFC00710);
		d("FIO_MASKA_S", 16, 0xFFC00718);
		d("FIO_MASKA_T", 16, 0xFFC0071C);
		d("FIO_MASKB_C", 16, 0xFFC00724);
		d("FIO_MASKB_D", 16, 0xFFC00720);
		d("FIO_MASKB_S", 16, 0xFFC00728);
		d("FIO_MASKB_T", 16, 0xFFC0072C);
		d("FIO_POLAR", 16, 0xFFC00734);

		parent = debugfs_create_dir("MEMDMA0 Destination Registers", top);
		d("MDMAFLX0_CURXCOUNT_D", 16, 0xFFC00E30);
		d("MDMAFLX0_CURYCOUNT_D", 16, 0xFFC00E38);
		d("MDMAFLX0_DMACNFG_D", 16, 0xFFC00E08);
		d("MDMAFLX0_IRQSTAT_D", 16, 0xFFC00E28);
		d("MDMAFLX0_PMAP_D", 16, 0xFFC00E2C);
		d("MDMAFLX0_XCOUNT_D", 16, 0xFFC00E10);
		d("MDMAFLX0_XMODIFY_D", 16, 0xFFC00E14);
		d("MDMAFLX0_YCOUNT_D", 16, 0xFFC00E18);
		d("MDMAFLX0_YMODIFY_D", 16, 0xFFC00E1C);
		d("MDMA_D0_CONFIG", 16, 0xFFC00E08);
		d("MDMA_D0_CURR_ADDR", 32, 0xFFC00E24);
		d("MDMA_D0_CURR_DESC_PTR", 32, 0xFFC00E20);
		d("MDMA_D0_CURR_X_COUNT", 16, 0xFFC00E30);
		d("MDMA_D0_CURR_Y_COUNT", 16, 0xFFC00E38);
		d("MDMA_D0_IRQ_STATUS", 16, 0xFFC00E28);
		d("MDMA_D0_NEXT_DESC_PTR", 32, 0xFFC00E00);
		d("MDMA_D0_PERIPHERAL_MAP", 16, 0xFFC00E2C);
		d("MDMA_D0_START_ADDR", 32, 0xFFC00E04);
		d("MDMA_D0_X_COUNT", 16, 0xFFC00E10);
		d("MDMA_D0_X_MODIFY", 16, 0xFFC00E14);
		d("MDMA_D0_Y_COUNT", 16, 0xFFC00E18);
		d("MDMA_D0_Y_MODIFY", 16, 0xFFC00E1C);

		parent = debugfs_create_dir("MEMDMA0 Source Registers", top);
		d("MDMAFLX0_CURXCOUNT_S", 16, 0xFFC00E70);
		d("MDMAFLX0_CURYCOUNT_S", 16, 0xFFC00E78);
		d("MDMAFLX0_DMACNFG_S", 16, 0xFFC00E48);
		d("MDMAFLX0_IRQSTAT_S", 16, 0xFFC00E68);
		d("MDMAFLX0_PMAP_S", 16, 0xFFC00E6C);
		d("MDMAFLX0_XCOUNT_S", 16, 0xFFC00E50);
		d("MDMAFLX0_XMODIFY_S", 16, 0xFFC00E54);
		d("MDMAFLX0_YCOUNT_S", 16, 0xFFC00E58);
		d("MDMAFLX0_YMODIFY_S", 16, 0xFFC00E5C);
		d("MDMA_S0_CONFIG", 16, 0xFFC00E48);
		d("MDMA_S0_CURR_ADDR", 32, 0xFFC00E64);
		d("MDMA_S0_CURR_DESC_PTR", 32, 0xFFC00E60);
		d("MDMA_S0_CURR_X_COUNT", 16, 0xFFC00E70);
		d("MDMA_S0_CURR_Y_COUNT", 16, 0xFFC00E78);
		d("MDMA_S0_IRQ_STATUS", 16, 0xFFC00E68);
		d("MDMA_S0_NEXT_DESC_PTR", 32, 0xFFC00E40);
		d("MDMA_S0_PERIPHERAL_MAP", 16, 0xFFC00E6C);
		d("MDMA_S0_START_ADDR", 32, 0xFFC00E44);
		d("MDMA_S0_X_COUNT", 16, 0xFFC00E50);
		d("MDMA_S0_X_MODIFY", 16, 0xFFC00E54);
		d("MDMA_S0_Y_COUNT", 16, 0xFFC00E58);
		d("MDMA_S0_Y_MODIFY", 16, 0xFFC00E5C);

		parent = debugfs_create_dir("MEMDMA1 Destination Registers", top);
		d("MDMAFLX1_CURXCOUNT_D", 16, 0xFFC00EB0);
		d("MDMAFLX1_CURYCOUNT_D", 16, 0xFFC00EB8);
		d("MDMAFLX1_DMACNFG_D", 16, 0xFFC00E88);
		d("MDMAFLX1_IRQSTAT_D", 16, 0xFFC00EA8);
		d("MDMAFLX1_PMAP_D", 16, 0xFFC00EAC);
		d("MDMAFLX1_XCOUNT_D", 16, 0xFFC00E90);
		d("MDMAFLX1_XMODIFY_D", 16, 0xFFC00E94);
		d("MDMAFLX1_YCOUNT_D", 16, 0xFFC00E98);
		d("MDMAFLX1_YMODIFY_D", 16, 0xFFC00E9C);
		d("MDMA_D1_CONFIG", 16, 0xFFC00E88);
		d("MDMA_D1_CURR_ADDR", 32, 0xFFC00EA4);
		d("MDMA_D1_CURR_DESC_PTR", 32, 0xFFC00EA0);
		d("MDMA_D1_CURR_X_COUNT", 16, 0xFFC00EB0);
		d("MDMA_D1_CURR_Y_COUNT", 16, 0xFFC00EB8);
		d("MDMA_D1_IRQ_STATUS", 16, 0xFFC00EA8);
		d("MDMA_D1_NEXT_DESC_PTR", 32, 0xFFC00E80);
		d("MDMA_D1_PERIPHERAL_MAP", 16, 0xFFC00EAC);
		d("MDMA_D1_START_ADDR", 32, 0xFFC00E84);
		d("MDMA_D1_X_COUNT", 16, 0xFFC00E90);
		d("MDMA_D1_X_MODIFY", 16, 0xFFC00E94);
		d("MDMA_D1_Y_COUNT", 16, 0xFFC00E98);
		d("MDMA_D1_Y_MODIFY", 16, 0xFFC00E9C);

		parent = debugfs_create_dir("MEMDMA1 Source Registers", top);
		d("MDMAFLX1_CURXCOUNT_S", 16, 0xFFC00EF0);
		d("MDMAFLX1_CURYCOUNT_S", 16, 0xFFC00EF8);
		d("MDMAFLX1_DMACNFG_S", 16, 0xFFC00EC8);
		d("MDMAFLX1_IRQSTAT_S", 16, 0xFFC00EE8);
		d("MDMAFLX1_PMAP_S", 16, 0xFFC00EEC);
		d("MDMAFLX1_XCOUNT_S", 16, 0xFFC00ED0);
		d("MDMAFLX1_XMODIFY_S", 16, 0xFFC00ED4);
		d("MDMAFLX1_YCOUNT_S", 16, 0xFFC00ED8);
		d("MDMAFLX1_YMODIFY_S", 16, 0xFFC00EDC);
		d("MDMA_S1_CONFIG", 16, 0xFFC00EC8);
		d("MDMA_S1_CURR_ADDR", 32, 0xFFC00EE4);
		d("MDMA_S1_CURR_DESC_PTR", 32, 0xFFC00EE0);
		d("MDMA_S1_CURR_X_COUNT", 16, 0xFFC00EF0);
		d("MDMA_S1_CURR_Y_COUNT", 16, 0xFFC00EF8);
		d("MDMA_S1_IRQ_STATUS", 16, 0xFFC00EE8);
		d("MDMA_S1_NEXT_DESC_PTR", 32, 0xFFC00EC0);
		d("MDMA_S1_PERIPHERAL_MAP", 16, 0xFFC00EEC);
		d("MDMA_S1_START_ADDR", 32, 0xFFC00EC4);
		d("MDMA_S1_X_COUNT", 16, 0xFFC00ED0);
		d("MDMA_S1_X_MODIFY", 16, 0xFFC00ED4);
		d("MDMA_S1_Y_COUNT", 16, 0xFFC00ED8);
		d("MDMA_S1_Y_MODIFY", 16, 0xFFC00EDC);

		parent = debugfs_create_dir("SPI", top);
		d("SPI_BAUD", 16, 0xFFC00514);
		d("SPI_CTL", 16, 0xFFC00500);
		d("SPI_FLG", 16, 0xFFC00504);
		d("SPI_RDBR", 16, 0xFFC00510);
		d("SPI_SHADOW", 16, 0xFFC00518);
		d("SPI_STAT", 16, 0xFFC00508);
		d("SPI_TDBR", 16, 0xFFC0050C);

		parent = debugfs_create_dir("UART", top);
		d("UART_DLH", 16, 0xFFC00404);
		d("UART_DLL", 16, 0xFFC00400);
		d("UART_GCTL", 16, 0xFFC00424);
		d("UART_IER", 16, 0xFFC00404);
		d("UART_IIR", 16, 0xFFC00408);
		d("UART_LCR", 16, 0xFFC0040C);
		d("UART_LSR", 16, 0xFFC00414);
		d("UART_MCR", 16, 0xFFC00410);
		d("UART_RBR", 16, 0xFFC00400);
		d("UART_SCR", 16, 0xFFC0041C);
		d("UART_THR", 16, 0xFFC00400);

	}	/* BF531 BF532 BF533 */

#ifdef __ADSPBF534__
# define USE_BF534 1
#else
# define USE_BF534 0
#endif
#ifdef __ADSPBF536__
# define USE_BF536 1
#else
# define USE_BF536 0
#endif
#ifdef __ADSPBF537__
# define USE_BF537 1
#else
# define USE_BF537 0
#endif
	if (USE_BF534 || USE_BF536 || USE_BF537) {

		parent = debugfs_create_dir("DMA Controller", top);
		d("DMA0_CONFIG", 16, 0xFFC00C08);
		d("DMA0_CURR_ADDR", 32, 0xFFC00C24);
		d("DMA0_CURR_DESC_PTR", 32, 0xFFC00C20);
		d("DMA0_CURR_X_COUNT", 16, 0xFFC00C30);
		d("DMA0_CURR_Y_COUNT", 16, 0xFFC00C38);
		d("DMA0_IRQ_STATUS", 16, 0xFFC00C28);
		d("DMA0_NEXT_DESC_PTR", 32, 0xFFC00C00);
		d("DMA0_PERIPHERAL_MAP", 16, 0xFFC00C2C);
		d("DMA0_START_ADDR", 32, 0xFFC00C04);
		d("DMA0_X_COUNT", 16, 0xFFC00C10);
		d("DMA0_X_MODIFY", 16, 0xFFC00C14);
		d("DMA0_Y_COUNT", 16, 0xFFC00C18);
		d("DMA0_Y_MODIFY", 16, 0xFFC00C1C);
		d("DMA10_CONFIG", 16, 0xFFC00E88);
		d("DMA10_CURR_ADDR", 32, 0xFFC00EA4);
		d("DMA10_CURR_DESC_PTR", 32, 0xFFC00EA0);
		d("DMA10_CURR_X_COUNT", 16, 0xFFC00EB0);
		d("DMA10_CURR_Y_COUNT", 16, 0xFFC00EB8);
		d("DMA10_IRQ_STATUS", 16, 0xFFC00EA8);
		d("DMA10_NEXT_DESC_PTR", 32, 0xFFC00E80);
		d("DMA10_PERIPHERAL_MAP", 16, 0xFFC00EAC);
		d("DMA10_START_ADDR", 32, 0xFFC00E84);
		d("DMA10_X_COUNT", 16, 0xFFC00E90);
		d("DMA10_X_MODIFY", 16, 0xFFC00E94);
		d("DMA10_Y_COUNT", 16, 0xFFC00E98);
		d("DMA10_Y_MODIFY", 16, 0xFFC00E9C);
		d("DMA11_CONFIG", 16, 0xFFC00EC8);
		d("DMA11_CURR_ADDR", 32, 0xFFC00EE4);
		d("DMA11_CURR_DESC_PTR", 32, 0xFFC00EE0);
		d("DMA11_CURR_X_COUNT", 16, 0xFFC00EF0);
		d("DMA11_CURR_Y_COUNT", 16, 0xFFC00EF8);
		d("DMA11_IRQ_STATUS", 16, 0xFFC00EE8);
		d("DMA11_NEXT_DESC_PTR", 32, 0xFFC00EC0);
		d("DMA11_PERIPHERAL_MAP", 16, 0xFFC00EEC);
		d("DMA11_START_ADDR", 32, 0xFFC00EC4);
		d("DMA11_X_COUNT", 16, 0xFFC00ED0);
		d("DMA11_X_MODIFY", 16, 0xFFC00ED4);
		d("DMA11_Y_COUNT", 16, 0xFFC00ED8);
		d("DMA11_Y_MODIFY", 16, 0xFFC00EDC);
		d("DMA1_CONFIG", 16, 0xFFC00C48);
		d("DMA1_CURR_ADDR", 32, 0xFFC00C64);
		d("DMA1_CURR_DESC_PTR", 32, 0xFFC00C60);
		d("DMA1_CURR_X_COUNT", 16, 0xFFC00C70);
		d("DMA1_CURR_Y_COUNT", 16, 0xFFC00C78);
		d("DMA1_IRQ_STATUS", 16, 0xFFC00C68);
		d("DMA1_NEXT_DESC_PTR", 32, 0xFFC00C40);
		d("DMA1_PERIPHERAL_MAP", 16, 0xFFC00C6C);
		d("DMA1_START_ADDR", 32, 0xFFC00C44);
		d("DMA1_X_COUNT", 16, 0xFFC00C50);
		d("DMA1_X_MODIFY", 16, 0xFFC00C54);
		d("DMA1_Y_COUNT", 16, 0xFFC00C58);
		d("DMA1_Y_MODIFY", 16, 0xFFC00C5C);
		d("DMA2_CONFIG", 16, 0xFFC00C88);
		d("DMA2_CURR_ADDR", 32, 0xFFC00CA4);
		d("DMA2_CURR_DESC_PTR", 32, 0xFFC00CA0);
		d("DMA2_CURR_X_COUNT", 16, 0xFFC00CB0);
		d("DMA2_CURR_Y_COUNT", 16, 0xFFC00CB8);
		d("DMA2_IRQ_STATUS", 16, 0xFFC00CA8);
		d("DMA2_NEXT_DESC_PTR", 32, 0xFFC00C80);
		d("DMA2_PERIPHERAL_MAP", 16, 0xFFC00CAC);
		d("DMA2_START_ADDR", 32, 0xFFC00C84);
		d("DMA2_X_COUNT", 16, 0xFFC00C90);
		d("DMA2_X_MODIFY", 16, 0xFFC00C94);
		d("DMA2_Y_COUNT", 16, 0xFFC00C98);
		d("DMA2_Y_MODIFY", 16, 0xFFC00C9C);
		d("DMA3_CONFIG", 16, 0xFFC00CC8);
		d("DMA3_CURR_ADDR", 32, 0xFFC00CE4);
		d("DMA3_CURR_DESC_PTR", 32, 0xFFC00CE0);
		d("DMA3_CURR_X_COUNT", 16, 0xFFC00CF0);
		d("DMA3_CURR_Y_COUNT", 16, 0xFFC00CF8);
		d("DMA3_IRQ_STATUS", 16, 0xFFC00CE8);
		d("DMA3_NEXT_DESC_PTR", 32, 0xFFC00CC0);
		d("DMA3_PERIPHERAL_MAP", 16, 0xFFC00CEC);
		d("DMA3_START_ADDR", 32, 0xFFC00CC4);
		d("DMA3_X_COUNT", 16, 0xFFC00CD0);
		d("DMA3_X_MODIFY", 16, 0xFFC00CD4);
		d("DMA3_Y_COUNT", 16, 0xFFC00CD8);
		d("DMA3_Y_MODIFY", 16, 0xFFC00CDC);
		d("DMA4_CONFIG", 16, 0xFFC00D08);
		d("DMA4_CURR_ADDR", 32, 0xFFC00D24);
		d("DMA4_CURR_DESC_PTR", 32, 0xFFC00D20);
		d("DMA4_CURR_X_COUNT", 16, 0xFFC00D30);
		d("DMA4_CURR_Y_COUNT", 16, 0xFFC00D38);
		d("DMA4_IRQ_STATUS", 16, 0xFFC00D28);
		d("DMA4_NEXT_DESC_PTR", 32, 0xFFC00D00);
		d("DMA4_PERIPHERAL_MAP", 16, 0xFFC00D2C);
		d("DMA4_START_ADDR", 32, 0xFFC00D04);
		d("DMA4_X_COUNT", 16, 0xFFC00D10);
		d("DMA4_X_MODIFY", 16, 0xFFC00D14);
		d("DMA4_Y_COUNT", 16, 0xFFC00D18);
		d("DMA4_Y_MODIFY", 16, 0xFFC00D1C);
		d("DMA5_CONFIG", 16, 0xFFC00D48);
		d("DMA5_CURR_ADDR", 32, 0xFFC00D64);
		d("DMA5_CURR_DESC_PTR", 32, 0xFFC00D60);
		d("DMA5_CURR_X_COUNT", 16, 0xFFC00D70);
		d("DMA5_CURR_Y_COUNT", 16, 0xFFC00D78);
		d("DMA5_IRQ_STATUS", 16, 0xFFC00D68);
		d("DMA5_NEXT_DESC_PTR", 32, 0xFFC00D40);
		d("DMA5_PERIPHERAL_MAP", 16, 0xFFC00D6C);
		d("DMA5_START_ADDR", 32, 0xFFC00D44);
		d("DMA5_X_COUNT", 16, 0xFFC00D50);
		d("DMA5_X_MODIFY", 16, 0xFFC00D54);
		d("DMA5_Y_COUNT", 16, 0xFFC00D58);
		d("DMA5_Y_MODIFY", 16, 0xFFC00D5C);
		d("DMA6_CONFIG", 16, 0xFFC00D88);
		d("DMA6_CURR_ADDR", 32, 0xFFC00DA4);
		d("DMA6_CURR_DESC_PTR", 32, 0xFFC00DA0);
		d("DMA6_CURR_X_COUNT", 16, 0xFFC00DB0);
		d("DMA6_CURR_Y_COUNT", 16, 0xFFC00DB8);
		d("DMA6_IRQ_STATUS", 16, 0xFFC00DA8);
		d("DMA6_NEXT_DESC_PTR", 32, 0xFFC00D80);
		d("DMA6_PERIPHERAL_MAP", 16, 0xFFC00DAC);
		d("DMA6_START_ADDR", 32, 0xFFC00D84);
		d("DMA6_X_COUNT", 16, 0xFFC00D90);
		d("DMA6_X_MODIFY", 16, 0xFFC00D94);
		d("DMA6_Y_COUNT", 16, 0xFFC00D98);
		d("DMA6_Y_MODIFY", 16, 0xFFC00D9C);
		d("DMA7_CONFIG", 16, 0xFFC00DC8);
		d("DMA7_CURR_ADDR", 32, 0xFFC00DE4);
		d("DMA7_CURR_DESC_PTR", 32, 0xFFC00DE0);
		d("DMA7_CURR_X_COUNT", 16, 0xFFC00DF0);
		d("DMA7_CURR_Y_COUNT", 16, 0xFFC00DF8);
		d("DMA7_IRQ_STATUS", 16, 0xFFC00DE8);
		d("DMA7_NEXT_DESC_PTR", 32, 0xFFC00DC0);
		d("DMA7_PERIPHERAL_MAP", 16, 0xFFC00DEC);
		d("DMA7_START_ADDR", 32, 0xFFC00DC4);
		d("DMA7_X_COUNT", 16, 0xFFC00DD0);
		d("DMA7_X_MODIFY", 16, 0xFFC00DD4);
		d("DMA7_Y_COUNT", 16, 0xFFC00DD8);
		d("DMA7_Y_MODIFY", 16, 0xFFC00DDC);
		d("DMA8_CONFIG", 16, 0xFFC00E08);
		d("DMA8_CURR_ADDR", 32, 0xFFC00E24);
		d("DMA8_CURR_DESC_PTR", 32, 0xFFC00E20);
		d("DMA8_CURR_X_COUNT", 16, 0xFFC00E30);
		d("DMA8_CURR_Y_COUNT", 16, 0xFFC00E38);
		d("DMA8_IRQ_STATUS", 16, 0xFFC00E28);
		d("DMA8_NEXT_DESC_PTR", 32, 0xFFC00E00);
		d("DMA8_PERIPHERAL_MAP", 16, 0xFFC00E2C);
		d("DMA8_START_ADDR", 32, 0xFFC00E04);
		d("DMA8_X_COUNT", 16, 0xFFC00E10);
		d("DMA8_X_MODIFY", 16, 0xFFC00E14);
		d("DMA8_Y_COUNT", 16, 0xFFC00E18);
		d("DMA8_Y_MODIFY", 16, 0xFFC00E1C);
		d("DMA9_CONFIG", 16, 0xFFC00E48);
		d("DMA9_CURR_ADDR", 32, 0xFFC00E64);
		d("DMA9_CURR_DESC_PTR", 32, 0xFFC00E60);
		d("DMA9_CURR_X_COUNT", 16, 0xFFC00E70);
		d("DMA9_CURR_Y_COUNT", 16, 0xFFC00E78);
		d("DMA9_IRQ_STATUS", 16, 0xFFC00E68);
		d("DMA9_NEXT_DESC_PTR", 32, 0xFFC00E40);
		d("DMA9_PERIPHERAL_MAP", 16, 0xFFC00E6C);
		d("DMA9_START_ADDR", 32, 0xFFC00E44);
		d("DMA9_X_COUNT", 16, 0xFFC00E50);
		d("DMA9_X_MODIFY", 16, 0xFFC00E54);
		d("DMA9_Y_COUNT", 16, 0xFFC00E58);
		d("DMA9_Y_MODIFY", 16, 0xFFC00E5C);

		parent = debugfs_create_dir("DMA Traffic Control", top);
		d("DMA_TC_CNT", 16, 0xFFC00B0C);
		d("DMA_TC_PER", 16, 0xFFC00B10);

		parent = debugfs_create_dir("Handshake MDMA", top);
		d("HMDMA0_BCINIT", 16, 0xFFC03308);
		d("HMDMA0_BCOUNT", 16, 0xFFC03318);
		d("HMDMA0_CONTROL", 16, 0xFFC03300);
		d("HMDMA0_ECINIT", 16, 0xFFC03304);
		d("HMDMA0_ECOUNT", 16, 0xFFC03314);
		d("HMDMA0_ECOVERFLOW", 16, 0xFFC03310);
		d("HMDMA0_ECURGENT", 16, 0xFFC0330C);
		d("HMDMA1_BCINIT", 16, 0xFFC03348);
		d("HMDMA1_BCOUNT", 16, 0xFFC03358);
		d("HMDMA1_CONTROL", 16, 0xFFC03340);
		d("HMDMA1_ECINIT", 16, 0xFFC03344);
		d("HMDMA1_ECOUNT", 16, 0xFFC03354);
		d("HMDMA1_ECOVERFLOW", 16, 0xFFC03350);
		d("HMDMA1_ECURGENT", 16, 0xFFC0334C);

		parent = debugfs_create_dir("MDMA Destination 0", top);
		d("MDMA_D0_CONFIG", 16, 0xFFC00F08);
		d("MDMA_D0_CURR_ADDR", 32, 0xFFC00F24);
		d("MDMA_D0_CURR_DESC_PTR", 32, 0xFFC00F20);
		d("MDMA_D0_CURR_X_COUNT", 16, 0xFFC00F30);
		d("MDMA_D0_CURR_Y_COUNT", 16, 0xFFC00F38);
		d("MDMA_D0_IRQ_STATUS", 16, 0xFFC00F28);
		d("MDMA_D0_NEXT_DESC_PTR", 32, 0xFFC00F00);
		d("MDMA_D0_PERIPHERAL_MAP", 16, 0xFFC00F2C);
		d("MDMA_D0_START_ADDR", 32, 0xFFC00F04);
		d("MDMA_D0_X_COUNT", 16, 0xFFC00F10);
		d("MDMA_D0_X_MODIFY", 16, 0xFFC00F14);
		d("MDMA_D0_Y_COUNT", 16, 0xFFC00F18);
		d("MDMA_D0_Y_MODIFY", 16, 0xFFC00F1C);

		parent = debugfs_create_dir("MDMA Destination 1", top);
		d("MDMA_D1_CONFIG", 16, 0xFFC00F88);
		d("MDMA_D1_CURR_ADDR", 32, 0xFFC00FA4);
		d("MDMA_D1_CURR_DESC_PTR", 32, 0xFFC00FA0);
		d("MDMA_D1_CURR_X_COUNT", 16, 0xFFC00FB0);
		d("MDMA_D1_CURR_Y_COUNT", 16, 0xFFC00FB8);
		d("MDMA_D1_IRQ_STATUS", 16, 0xFFC00FA8);
		d("MDMA_D1_NEXT_DESC_PTR", 32, 0xFFC00F80);
		d("MDMA_D1_PERIPHERAL_MAP", 16, 0xFFC00FAC);
		d("MDMA_D1_START_ADDR", 32, 0xFFC00F84);
		d("MDMA_D1_X_COUNT", 16, 0xFFC00F90);
		d("MDMA_D1_X_MODIFY", 16, 0xFFC00F94);
		d("MDMA_D1_Y_COUNT", 16, 0xFFC00F98);
		d("MDMA_D1_Y_MODIFY", 16, 0xFFC00F9C);

		parent = debugfs_create_dir("MDMA Source 0", top);
		d("MDMA_S0_CONFIG", 16, 0xFFC00F48);
		d("MDMA_S0_CURR_ADDR", 32, 0xFFC00F64);
		d("MDMA_S0_CURR_DESC_PTR", 32, 0xFFC00F60);
		d("MDMA_S0_CURR_X_COUNT", 16, 0xFFC00F70);
		d("MDMA_S0_CURR_Y_COUNT", 16, 0xFFC00F78);
		d("MDMA_S0_IRQ_STATUS", 16, 0xFFC00F68);
		d("MDMA_S0_NEXT_DESC_PTR", 32, 0xFFC00F40);
		d("MDMA_S0_PERIPHERAL_MAP", 16, 0xFFC00F6C);
		d("MDMA_S0_START_ADDR", 32, 0xFFC00F44);
		d("MDMA_S0_X_COUNT", 16, 0xFFC00F50);
		d("MDMA_S0_X_MODIFY", 16, 0xFFC00F54);
		d("MDMA_S0_Y_COUNT", 16, 0xFFC00F58);
		d("MDMA_S0_Y_MODIFY", 16, 0xFFC00F5C);

		parent = debugfs_create_dir("MDMA Source 1", top);
		d("MDMA_S1_CONFIG", 16, 0xFFC00FC8);
		d("MDMA_S1_CURR_ADDR", 32, 0xFFC00FE4);
		d("MDMA_S1_CURR_DESC_PTR", 32, 0xFFC00FE0);
		d("MDMA_S1_CURR_X_COUNT", 16, 0xFFC00FF0);
		d("MDMA_S1_CURR_Y_COUNT", 16, 0xFFC00FF8);
		d("MDMA_S1_IRQ_STATUS", 16, 0xFFC00FE8);
		d("MDMA_S1_NEXT_DESC_PTR", 32, 0xFFC00FC0);
		d("MDMA_S1_PERIPHERAL_MAP", 16, 0xFFC00FEC);
		d("MDMA_S1_START_ADDR", 32, 0xFFC00FC4);
		d("MDMA_S1_X_COUNT", 16, 0xFFC00FD0);
		d("MDMA_S1_X_MODIFY", 16, 0xFFC00FD4);
		d("MDMA_S1_Y_COUNT", 16, 0xFFC00FD8);
		d("MDMA_S1_Y_MODIFY", 16, 0xFFC00FDC);

		parent = debugfs_create_dir("Pin Control", top);
		d("PORTF_FER", 16, 0xFFC03200);
		d("PORTG_FER", 16, 0xFFC03204);
		d("PORTH_FER", 16, 0xFFC03208);
		d("PORT_MUX", 16, 0xFFC0320C);

		parent = debugfs_create_dir("Port I-O", top);
		d("PORTFIO", 16, 0xFFC00700);
		d("PORTFIO_BOTH", 16, 0xFFC0073C);
		d("PORTFIO_CLEAR", 16, 0xFFC00704);
		d("PORTFIO_DIR", 16, 0xFFC00730);
		d("PORTFIO_EDGE", 16, 0xFFC00738);
		d("PORTFIO_INEN", 16, 0xFFC00740);
		d("PORTFIO_MASKA", 16, 0xFFC00710);
		d("PORTFIO_MASKA_CLEAR", 16, 0xFFC00714);
		d("PORTFIO_MASKA_SET", 16, 0xFFC00718);
		d("PORTFIO_MASKA_TOGGLE", 16, 0xFFC0071C);
		d("PORTFIO_MASKB", 16, 0xFFC00720);
		d("PORTFIO_MASKB_CLEAR", 16, 0xFFC00724);
		d("PORTFIO_MASKB_SET", 16, 0xFFC00728);
		d("PORTFIO_MASKB_TOGGLE", 16, 0xFFC0072C);
		d("PORTFIO_POLAR", 16, 0xFFC00734);
		d("PORTFIO_SET", 16, 0xFFC00708);
		d("PORTFIO_TOGGLE", 16, 0xFFC0070C);
		d("PORTGIO", 16, 0xFFC01500);
		d("PORTGIO_BOTH", 16, 0xFFC0153C);
		d("PORTGIO_CLEAR", 16, 0xFFC01504);
		d("PORTGIO_DIR", 16, 0xFFC01530);
		d("PORTGIO_EDGE", 16, 0xFFC01538);
		d("PORTGIO_INEN", 16, 0xFFC01540);
		d("PORTGIO_MASKA", 16, 0xFFC01510);
		d("PORTGIO_MASKA_CLEAR", 16, 0xFFC01514);
		d("PORTGIO_MASKA_SET", 16, 0xFFC01518);
		d("PORTGIO_MASKA_TOGGLE", 16, 0xFFC0151C);
		d("PORTGIO_MASKB", 16, 0xFFC01520);
		d("PORTGIO_MASKB_CLEAR", 16, 0xFFC01524);
		d("PORTGIO_MASKB_SET", 16, 0xFFC01528);
		d("PORTGIO_MASKB_TOGGLE", 16, 0xFFC0152C);
		d("PORTGIO_POLAR", 16, 0xFFC01534);
		d("PORTGIO_SET", 16, 0xFFC01508);
		d("PORTGIO_TOGGLE", 16, 0xFFC0150C);
		d("PORTHIO", 16, 0xFFC01700);
		d("PORTHIO_BOTH", 16, 0xFFC0173C);
		d("PORTHIO_CLEAR", 16, 0xFFC01704);
		d("PORTHIO_DIR", 16, 0xFFC01730);
		d("PORTHIO_EDGE", 16, 0xFFC01738);
		d("PORTHIO_INEN", 16, 0xFFC01740);
		d("PORTHIO_MASKA", 16, 0xFFC01710);
		d("PORTHIO_MASKA_CLEAR", 16, 0xFFC01714);
		d("PORTHIO_MASKA_SET", 16, 0xFFC01718);
		d("PORTHIO_MASKA_TOGGLE", 16, 0xFFC0171C);
		d("PORTHIO_MASKB", 16, 0xFFC01720);
		d("PORTHIO_MASKB_CLEAR", 16, 0xFFC01724);
		d("PORTHIO_MASKB_SET", 16, 0xFFC01728);
		d("PORTHIO_MASKB_TOGGLE", 16, 0xFFC0172C);
		d("PORTHIO_POLAR", 16, 0xFFC01734);
		d("PORTHIO_SET", 16, 0xFFC01708);
		d("PORTHIO_TOGGLE", 16, 0xFFC0170C);

		parent = debugfs_create_dir("SPI", top);
		d("SPI_BAUD", 16, 0xFFC00514);
		d("SPI_CTL", 16, 0xFFC00500);
		d("SPI_FLG", 16, 0xFFC00504);
		d("SPI_RDBR", 16, 0xFFC00510);
		d("SPI_SHADOW", 16, 0xFFC00518);
		d("SPI_STAT", 16, 0xFFC00508);
		d("SPI_TDBR", 16, 0xFFC0050C);

		parent = debugfs_create_dir("TWI", top);
		d("TWI_CLKDIV", 16, 0xFFC01400);
		d("TWI_CONTROL", 16, 0xFFC01404);
		d("TWI_FIFO_CTL", 16, 0xFFC01428);
		d("TWI_FIFO_STAT", 16, 0xFFC0142C);
		d("TWI_INT_MASK", 16, 0xFFC01424);
		d("TWI_INT_STAT", 16, 0xFFC01420);
		d("TWI_MASTER_ADDR", 16, 0xFFC0141C);
		d("TWI_MASTER_CTL", 16, 0xFFC01414);
		d("TWI_MASTER_STAT", 16, 0xFFC01418);
		d("TWI_RCV_DATA16", 16, 0xFFC0148C);
		d("TWI_RCV_DATA8", 16, 0xFFC01488);
		d("TWI_SLAVE_ADDR", 16, 0xFFC01410);
		d("TWI_SLAVE_CTL", 16, 0xFFC01408);
		d("TWI_SLAVE_STAT", 16, 0xFFC0140C);
		d("TWI_XMT_DATA16", 16, 0xFFC01484);
		d("TWI_XMT_DATA8", 16, 0xFFC01480);

		parent = debugfs_create_dir("UART", top);
		d("UART0_DLH", 16, 0xFFC00404);
		d("UART0_DLL", 16, 0xFFC00400);
		d("UART0_GCTL", 16, 0xFFC00424);
		d("UART0_IER", 16, 0xFFC00404);
		d("UART0_IIR", 16, 0xFFC00408);
		d("UART0_LCR", 16, 0xFFC0040C);
		d("UART0_LSR", 16, 0xFFC00414);
		d("UART0_MCR", 16, 0xFFC00410);
		d("UART0_MSR", 16, 0xFFC00418);
		d("UART0_RBR", 16, 0xFFC00400);
		d("UART0_SCR", 16, 0xFFC0041C);
		d("UART0_THR", 16, 0xFFC00400);
		d("UART1_DLH", 16, 0xFFC02004);
		d("UART1_DLL", 16, 0xFFC02000);
		d("UART1_GCTL", 16, 0xFFC02024);
		d("UART1_IER", 16, 0xFFC02004);
		d("UART1_IIR", 16, 0xFFC02008);
		d("UART1_LCR", 16, 0xFFC0200C);
		d("UART1_LSR", 16, 0xFFC02014);
		d("UART1_MCR", 16, 0xFFC02010);
		d("UART1_MSR", 16, 0xFFC02018);
		d("UART1_RBR", 16, 0xFFC02000);
		d("UART1_SCR", 16, 0xFFC0201C);
		d("UART1_THR", 16, 0xFFC02000);

	}	/* BF534 BF536 BF537 */

#ifdef __ADSPBF538__
# define USE_BF538 1
#else
# define USE_BF538 0
#endif
#ifdef __ADSPBF539__
# define USE_BF539 1
#else
# define USE_BF539 0
#endif
	if (USE_BF538 || USE_BF539) {

		parent = debugfs_create_dir("DMA Traffic Control", top);
		d("DMA0_TC_CNT", 16, 0xFFC00B10);
		d("DMA0_TC_PER", 16, 0xFFC00B0C);
		d("DMA1_TC_CNT", 16, 0xFFC01B10);
		d("DMA1_TC_PER", 16, 0xFFC01B0C);

		parent = debugfs_create_dir("DMA0 (DMA0 Ch-0) Registers", top);
		d("DMA0_CONFIG", 16, 0xFFC00C08);
		d("DMA0_CURR_ADDR", 32, 0xFFC00C24);
		d("DMA0_CURR_DESC_PTR", 32, 0xFFC00C20);
		d("DMA0_CURR_X_COUNT", 16, 0xFFC00C30);
		d("DMA0_CURR_Y_COUNT", 16, 0xFFC00C38);
		d("DMA0_IRQ_STATUS", 16, 0xFFC00C28);
		d("DMA0_NEXT_DESC_PTR", 32, 0xFFC00C00);
		d("DMA0_PERIPHERAL_MAP", 16, 0xFFC00C2C);
		d("DMA0_START_ADDR", 32, 0xFFC00C04);
		d("DMA0_X_COUNT", 16, 0xFFC00C10);
		d("DMA0_X_MODIFY", 16, 0xFFC00C14);
		d("DMA0_Y_COUNT", 16, 0xFFC00C18);
		d("DMA0_Y_MODIFY", 16, 0xFFC00C1C);

		parent = debugfs_create_dir("DMA1 (DMA0 Ch-1) Registers", top);
		d("DMA1_CONFIG", 16, 0xFFC00C48);
		d("DMA1_CURR_ADDR", 32, 0xFFC00C64);
		d("DMA1_CURR_DESC_PTR", 32, 0xFFC00C60);
		d("DMA1_CURR_X_COUNT", 16, 0xFFC00C70);
		d("DMA1_CURR_Y_COUNT", 16, 0xFFC00C78);
		d("DMA1_IRQ_STATUS", 16, 0xFFC00C68);
		d("DMA1_NEXT_DESC_PTR", 32, 0xFFC00C40);
		d("DMA1_PERIPHERAL_MAP", 16, 0xFFC00C6C);
		d("DMA1_START_ADDR", 32, 0xFFC00C44);
		d("DMA1_X_COUNT", 16, 0xFFC00C50);
		d("DMA1_X_MODIFY", 16, 0xFFC00C54);
		d("DMA1_Y_COUNT", 16, 0xFFC00C58);
		d("DMA1_Y_MODIFY", 16, 0xFFC00C5C);

		parent = debugfs_create_dir("DMA10 (DMA1 Ch-2) Registers", top);
		d("DMA10_CONFIG", 16, 0xFFC01C88);
		d("DMA10_CURR_ADDR", 32, 0xFFC01CA4);
		d("DMA10_CURR_DESC_PTR", 32, 0xFFC01CA0);
		d("DMA10_CURR_X_COUNT", 16, 0xFFC01CB0);
		d("DMA10_CURR_Y_COUNT", 16, 0xFFC01CB8);
		d("DMA10_IRQ_STATUS", 16, 0xFFC01CA8);
		d("DMA10_NEXT_DESC_PTR", 32, 0xFFC01C80);
		d("DMA10_PERIPHERAL_MAP", 16, 0xFFC01CAC);
		d("DMA10_START_ADDR", 32, 0xFFC01C84);
		d("DMA10_X_COUNT", 16, 0xFFC01C90);
		d("DMA10_X_MODIFY", 16, 0xFFC01C94);
		d("DMA10_Y_COUNT", 16, 0xFFC01C98);
		d("DMA10_Y_MODIFY", 16, 0xFFC01C9C);

		parent = debugfs_create_dir("DMA11 (DMA1 Ch-3) Registers", top);
		d("DMA11_CONFIG", 16, 0xFFC01CC8);
		d("DMA11_CURR_ADDR", 32, 0xFFC01CE4);
		d("DMA11_CURR_DESC_PTR", 32, 0xFFC01CE0);
		d("DMA11_CURR_X_COUNT", 16, 0xFFC01CF0);
		d("DMA11_CURR_Y_COUNT", 16, 0xFFC01CF8);
		d("DMA11_IRQ_STATUS", 16, 0xFFC01CE8);
		d("DMA11_NEXT_DESC_PTR", 32, 0xFFC01CC0);
		d("DMA11_PERIPHERAL_MAP", 16, 0xFFC01CEC);
		d("DMA11_START_ADDR", 32, 0xFFC01CC4);
		d("DMA11_X_COUNT", 16, 0xFFC01CD0);
		d("DMA11_X_MODIFY", 16, 0xFFC01CD4);
		d("DMA11_Y_COUNT", 16, 0xFFC01CD8);
		d("DMA11_Y_MODIFY", 16, 0xFFC01CDC);

		parent = debugfs_create_dir("DMA12 (DMA1 Ch-4) Registers", top);
		d("DMA12_CONFIG", 16, 0xFFC01D08);
		d("DMA12_CURR_ADDR", 32, 0xFFC01D24);
		d("DMA12_CURR_DESC_PTR", 32, 0xFFC01D20);
		d("DMA12_CURR_X_COUNT", 16, 0xFFC01D30);
		d("DMA12_CURR_Y_COUNT", 16, 0xFFC01D38);
		d("DMA12_IRQ_STATUS", 16, 0xFFC01D28);
		d("DMA12_NEXT_DESC_PTR", 32, 0xFFC01D00);
		d("DMA12_PERIPHERAL_MAP", 16, 0xFFC01D2C);
		d("DMA12_START_ADDR", 32, 0xFFC01D04);
		d("DMA12_X_COUNT", 16, 0xFFC01D10);
		d("DMA12_X_MODIFY", 16, 0xFFC01D14);
		d("DMA12_Y_COUNT", 16, 0xFFC01D18);
		d("DMA12_Y_MODIFY", 16, 0xFFC01D1C);

		parent = debugfs_create_dir("DMA13 (DMA1 Ch-5) Registers", top);
		d("DMA13_CONFIG", 16, 0xFFC01D48);
		d("DMA13_CURR_ADDR", 32, 0xFFC01D64);
		d("DMA13_CURR_DESC_PTR", 32, 0xFFC01D60);
		d("DMA13_CURR_X_COUNT", 16, 0xFFC01D70);
		d("DMA13_CURR_Y_COUNT", 16, 0xFFC01D78);
		d("DMA13_IRQ_STATUS", 16, 0xFFC01D68);
		d("DMA13_NEXT_DESC_PTR", 32, 0xFFC01D40);
		d("DMA13_PERIPHERAL_MAP", 16, 0xFFC01D6C);
		d("DMA13_START_ADDR", 32, 0xFFC01D44);
		d("DMA13_X_COUNT", 16, 0xFFC01D50);
		d("DMA13_X_MODIFY", 16, 0xFFC01D54);
		d("DMA13_Y_COUNT", 16, 0xFFC01D58);
		d("DMA13_Y_MODIFY", 16, 0xFFC01D5C);

		parent = debugfs_create_dir("DMA14 (DMA1 Ch-6) Registers", top);
		d("DMA14_CONFIG", 16, 0xFFC01D88);
		d("DMA14_CURR_ADDR", 32, 0xFFC01DA4);
		d("DMA14_CURR_DESC_PTR", 32, 0xFFC01DA0);
		d("DMA14_CURR_X_COUNT", 16, 0xFFC01DB0);
		d("DMA14_CURR_Y_COUNT", 16, 0xFFC01DB8);
		d("DMA14_IRQ_STATUS", 16, 0xFFC01DA8);
		d("DMA14_NEXT_DESC_PTR", 32, 0xFFC01D80);
		d("DMA14_PERIPHERAL_MAP", 16, 0xFFC01DAC);
		d("DMA14_START_ADDR", 32, 0xFFC01D84);
		d("DMA14_X_COUNT", 16, 0xFFC01D90);
		d("DMA14_X_MODIFY", 16, 0xFFC01D94);
		d("DMA14_Y_COUNT", 16, 0xFFC01D98);
		d("DMA14_Y_MODIFY", 16, 0xFFC01D9C);

		parent = debugfs_create_dir("DMA15 (DMA1 Ch-7) Registers", top);
		d("DMA15_CONFIG", 16, 0xFFC01DC8);
		d("DMA15_CURR_ADDR", 32, 0xFFC01DE4);
		d("DMA15_CURR_DESC_PTR", 32, 0xFFC01DE0);
		d("DMA15_CURR_X_COUNT", 16, 0xFFC01DF0);
		d("DMA15_CURR_Y_COUNT", 16, 0xFFC01DF8);
		d("DMA15_IRQ_STATUS", 16, 0xFFC01DE8);
		d("DMA15_NEXT_DESC_PTR", 32, 0xFFC01DC0);
		d("DMA15_PERIPHERAL_MAP", 16, 0xFFC01DEC);
		d("DMA15_START_ADDR", 32, 0xFFC01DC4);
		d("DMA15_X_COUNT", 16, 0xFFC01DD0);
		d("DMA15_X_MODIFY", 16, 0xFFC01DD4);
		d("DMA15_Y_COUNT", 16, 0xFFC01DD8);
		d("DMA15_Y_MODIFY", 16, 0xFFC01DDC);

		parent = debugfs_create_dir("DMA16 (DMA1 Ch-8) Registers", top);
		d("DMA16_CONFIG", 16, 0xFFC01E08);
		d("DMA16_CURR_ADDR", 32, 0xFFC01E24);
		d("DMA16_CURR_DESC_PTR", 32, 0xFFC01E20);
		d("DMA16_CURR_X_COUNT", 16, 0xFFC01E30);
		d("DMA16_CURR_Y_COUNT", 16, 0xFFC01E38);
		d("DMA16_IRQ_STATUS", 16, 0xFFC01E28);
		d("DMA16_NEXT_DESC_PTR", 32, 0xFFC01E00);
		d("DMA16_PERIPHERAL_MAP", 16, 0xFFC01E2C);
		d("DMA16_START_ADDR", 32, 0xFFC01E04);
		d("DMA16_X_COUNT", 16, 0xFFC01E10);
		d("DMA16_X_MODIFY", 16, 0xFFC01E14);
		d("DMA16_Y_COUNT", 16, 0xFFC01E18);
		d("DMA16_Y_MODIFY", 16, 0xFFC01E1C);

		parent = debugfs_create_dir("DMA17 (DMA1 Ch-9) Registers", top);
		d("DMA17_CONFIG", 16, 0xFFC01E48);
		d("DMA17_CURR_ADDR", 32, 0xFFC01E64);
		d("DMA17_CURR_DESC_PTR", 32, 0xFFC01E60);
		d("DMA17_CURR_X_COUNT", 16, 0xFFC01E70);
		d("DMA17_CURR_Y_COUNT", 16, 0xFFC01E78);
		d("DMA17_IRQ_STATUS", 16, 0xFFC01E68);
		d("DMA17_NEXT_DESC_PTR", 32, 0xFFC01E40);
		d("DMA17_PERIPHERAL_MAP", 16, 0xFFC01E6C);
		d("DMA17_START_ADDR", 32, 0xFFC01E44);
		d("DMA17_X_COUNT", 16, 0xFFC01E50);
		d("DMA17_X_MODIFY", 16, 0xFFC01E54);
		d("DMA17_Y_COUNT", 16, 0xFFC01E58);
		d("DMA17_Y_MODIFY", 16, 0xFFC01E5C);

		parent = debugfs_create_dir("DMA18 (DMA1 Ch-10) Registers", top);
		d("DMA18_CONFIG", 16, 0xFFC01E88);
		d("DMA18_CURR_ADDR", 32, 0xFFC01EA4);
		d("DMA18_CURR_DESC_PTR", 32, 0xFFC01EA0);
		d("DMA18_CURR_X_COUNT", 16, 0xFFC01EB0);
		d("DMA18_CURR_Y_COUNT", 16, 0xFFC01EB8);
		d("DMA18_IRQ_STATUS", 16, 0xFFC01EA8);
		d("DMA18_NEXT_DESC_PTR", 32, 0xFFC01E80);
		d("DMA18_PERIPHERAL_MAP", 16, 0xFFC01EAC);
		d("DMA18_START_ADDR", 32, 0xFFC01E84);
		d("DMA18_X_COUNT", 16, 0xFFC01E90);
		d("DMA18_X_MODIFY", 16, 0xFFC01E94);
		d("DMA18_Y_COUNT", 16, 0xFFC01E98);
		d("DMA18_Y_MODIFY", 16, 0xFFC01E9C);

		parent = debugfs_create_dir("DMA19 (DMA1 Ch-11) Registers", top);
		d("DMA19_CONFIG", 16, 0xFFC01EC8);
		d("DMA19_CURR_ADDR", 32, 0xFFC01EE4);
		d("DMA19_CURR_DESC_PTR", 32, 0xFFC01EE0);
		d("DMA19_CURR_X_COUNT", 16, 0xFFC01EF0);
		d("DMA19_CURR_Y_COUNT", 16, 0xFFC01EF8);
		d("DMA19_IRQ_STATUS", 16, 0xFFC01EE8);
		d("DMA19_NEXT_DESC_PTR", 32, 0xFFC01EC0);
		d("DMA19_PERIPHERAL_MAP", 16, 0xFFC01EEC);
		d("DMA19_START_ADDR", 32, 0xFFC01EC4);
		d("DMA19_X_COUNT", 16, 0xFFC01ED0);
		d("DMA19_X_MODIFY", 16, 0xFFC01ED4);
		d("DMA19_Y_COUNT", 16, 0xFFC01ED8);
		d("DMA19_Y_MODIFY", 16, 0xFFC01EDC);

		parent = debugfs_create_dir("DMA2 (DMA0 Ch-2) Registers", top);
		d("DMA2_CONFIG", 16, 0xFFC00C88);
		d("DMA2_CURR_ADDR", 32, 0xFFC00CA4);
		d("DMA2_CURR_DESC_PTR", 32, 0xFFC00CA0);
		d("DMA2_CURR_X_COUNT", 16, 0xFFC00CB0);
		d("DMA2_CURR_Y_COUNT", 16, 0xFFC00CB8);
		d("DMA2_IRQ_STATUS", 16, 0xFFC00CA8);
		d("DMA2_NEXT_DESC_PTR", 32, 0xFFC00C80);
		d("DMA2_PERIPHERAL_MAP", 16, 0xFFC00CAC);
		d("DMA2_START_ADDR", 32, 0xFFC00C84);
		d("DMA2_X_COUNT", 16, 0xFFC00C90);
		d("DMA2_X_MODIFY", 16, 0xFFC00C94);
		d("DMA2_Y_COUNT", 16, 0xFFC00C98);
		d("DMA2_Y_MODIFY", 16, 0xFFC00C9C);

		parent = debugfs_create_dir("DMA3 (DMA0 Ch-3) Registers", top);
		d("DMA3_CONFIG", 16, 0xFFC00CC8);
		d("DMA3_CURR_ADDR", 32, 0xFFC00CE4);
		d("DMA3_CURR_DESC_PTR", 32, 0xFFC00CE0);
		d("DMA3_CURR_X_COUNT", 16, 0xFFC00CF0);
		d("DMA3_CURR_Y_COUNT", 16, 0xFFC00CF8);
		d("DMA3_IRQ_STATUS", 16, 0xFFC00CE8);
		d("DMA3_NEXT_DESC_PTR", 32, 0xFFC00CC0);
		d("DMA3_PERIPHERAL_MAP", 16, 0xFFC00CEC);
		d("DMA3_START_ADDR", 32, 0xFFC00CC4);
		d("DMA3_X_COUNT", 16, 0xFFC00CD0);
		d("DMA3_X_MODIFY", 16, 0xFFC00CD4);
		d("DMA3_Y_COUNT", 16, 0xFFC00CD8);
		d("DMA3_Y_MODIFY", 16, 0xFFC00CDC);

		parent = debugfs_create_dir("DMA4 (DMA0 Ch-4) Registers", top);
		d("DMA4_CONFIG", 16, 0xFFC00D08);
		d("DMA4_CURR_ADDR", 32, 0xFFC00D24);
		d("DMA4_CURR_DESC_PTR", 32, 0xFFC00D20);
		d("DMA4_CURR_X_COUNT", 16, 0xFFC00D30);
		d("DMA4_CURR_Y_COUNT", 16, 0xFFC00D38);
		d("DMA4_IRQ_STATUS", 16, 0xFFC00D28);
		d("DMA4_NEXT_DESC_PTR", 32, 0xFFC00D00);
		d("DMA4_PERIPHERAL_MAP", 16, 0xFFC00D2C);
		d("DMA4_START_ADDR", 32, 0xFFC00D04);
		d("DMA4_X_COUNT", 16, 0xFFC00D10);
		d("DMA4_X_MODIFY", 16, 0xFFC00D14);
		d("DMA4_Y_COUNT", 16, 0xFFC00D18);
		d("DMA4_Y_MODIFY", 16, 0xFFC00D1C);

		parent = debugfs_create_dir("DMA5 (DMA0 Ch-5) Registers", top);
		d("DMA5_CONFIG", 16, 0xFFC00D48);
		d("DMA5_CURR_ADDR", 32, 0xFFC00D64);
		d("DMA5_CURR_DESC_PTR", 32, 0xFFC00D60);
		d("DMA5_CURR_X_COUNT", 16, 0xFFC00D70);
		d("DMA5_CURR_Y_COUNT", 16, 0xFFC00D78);
		d("DMA5_IRQ_STATUS", 16, 0xFFC00D68);
		d("DMA5_NEXT_DESC_PTR", 32, 0xFFC00D40);
		d("DMA5_PERIPHERAL_MAP", 16, 0xFFC00D6C);
		d("DMA5_START_ADDR", 32, 0xFFC00D44);
		d("DMA5_X_COUNT", 16, 0xFFC00D50);
		d("DMA5_X_MODIFY", 16, 0xFFC00D54);
		d("DMA5_Y_COUNT", 16, 0xFFC00D58);
		d("DMA5_Y_MODIFY", 16, 0xFFC00D5C);

		parent = debugfs_create_dir("DMA6 (DMA0 Ch-6) Registers", top);
		d("DMA6_CONFIG", 16, 0xFFC00D88);
		d("DMA6_CURR_ADDR", 32, 0xFFC00DA4);
		d("DMA6_CURR_DESC_PTR", 32, 0xFFC00DA0);
		d("DMA6_CURR_X_COUNT", 16, 0xFFC00DB0);
		d("DMA6_CURR_Y_COUNT", 16, 0xFFC00DB8);
		d("DMA6_IRQ_STATUS", 16, 0xFFC00DA8);
		d("DMA6_NEXT_DESC_PTR", 32, 0xFFC00D80);
		d("DMA6_PERIPHERAL_MAP", 16, 0xFFC00DAC);
		d("DMA6_START_ADDR", 32, 0xFFC00D84);
		d("DMA6_X_COUNT", 16, 0xFFC00D90);
		d("DMA6_X_MODIFY", 16, 0xFFC00D94);
		d("DMA6_Y_COUNT", 16, 0xFFC00D98);
		d("DMA6_Y_MODIFY", 16, 0xFFC00D9C);

		parent = debugfs_create_dir("DMA7 (DMA0 Ch-7) Registers", top);
		d("DMA7_CONFIG", 16, 0xFFC00DC8);
		d("DMA7_CURR_ADDR", 32, 0xFFC00DE4);
		d("DMA7_CURR_DESC_PTR", 32, 0xFFC00DE0);
		d("DMA7_CURR_X_COUNT", 16, 0xFFC00DF0);
		d("DMA7_CURR_Y_COUNT", 16, 0xFFC00DF8);
		d("DMA7_IRQ_STATUS", 16, 0xFFC00DE8);
		d("DMA7_NEXT_DESC_PTR", 32, 0xFFC00DC0);
		d("DMA7_PERIPHERAL_MAP", 16, 0xFFC00DEC);
		d("DMA7_START_ADDR", 32, 0xFFC00DC4);
		d("DMA7_X_COUNT", 16, 0xFFC00DD0);
		d("DMA7_X_MODIFY", 16, 0xFFC00DD4);
		d("DMA7_Y_COUNT", 16, 0xFFC00DD8);
		d("DMA7_Y_MODIFY", 16, 0xFFC00DDC);

		parent = debugfs_create_dir("DMA8 (DMA1 Ch-0) Registers", top);
		d("DMA8_CONFIG", 16, 0xFFC01C08);
		d("DMA8_CURR_ADDR", 32, 0xFFC01C24);
		d("DMA8_CURR_DESC_PTR", 32, 0xFFC01C20);
		d("DMA8_CURR_X_COUNT", 16, 0xFFC01C30);
		d("DMA8_CURR_Y_COUNT", 16, 0xFFC01C38);
		d("DMA8_IRQ_STATUS", 16, 0xFFC01C28);
		d("DMA8_NEXT_DESC_PTR", 32, 0xFFC01C00);
		d("DMA8_PERIPHERAL_MAP", 16, 0xFFC01C2C);
		d("DMA8_START_ADDR", 32, 0xFFC01C04);
		d("DMA8_X_COUNT", 16, 0xFFC01C10);
		d("DMA8_X_MODIFY", 16, 0xFFC01C14);
		d("DMA8_Y_COUNT", 16, 0xFFC01C18);
		d("DMA8_Y_MODIFY", 16, 0xFFC01C1C);

		parent = debugfs_create_dir("DMA9 (DMA1 Ch-1) Registers", top);
		d("DMA9_CONFIG", 16, 0xFFC01C48);
		d("DMA9_CURR_ADDR", 32, 0xFFC01C64);
		d("DMA9_CURR_DESC_PTR", 32, 0xFFC01C60);
		d("DMA9_CURR_X_COUNT", 16, 0xFFC01C70);
		d("DMA9_CURR_Y_COUNT", 16, 0xFFC01C78);
		d("DMA9_IRQ_STATUS", 16, 0xFFC01C68);
		d("DMA9_NEXT_DESC_PTR", 32, 0xFFC01C40);
		d("DMA9_PERIPHERAL_MAP", 16, 0xFFC01C6C);
		d("DMA9_START_ADDR", 32, 0xFFC01C44);
		d("DMA9_X_COUNT", 16, 0xFFC01C50);
		d("DMA9_X_MODIFY", 16, 0xFFC01C54);
		d("DMA9_Y_COUNT", 16, 0xFFC01C58);
		d("DMA9_Y_MODIFY", 16, 0xFFC01C5C);

		parent = debugfs_create_dir("GPIO Port C", top);
		d("PORTCIO", 16, 0xFFC01510);
		d("PORTCIO_CLEAR", 16, 0xFFC01520);
		d("PORTCIO_DIR", 16, 0xFFC01550);
		d("PORTCIO_FER", 16, 0xFFC01500);
		d("PORTCIO_INEN", 16, 0xFFC01560);
		d("PORTCIO_SET", 16, 0xFFC01530);
		d("PORTCIO_TOGGLE", 16, 0xFFC01540);

		parent = debugfs_create_dir("GPIO Port D", top);
		d("PORTDIO", 16, 0xFFC01514);
		d("PORTDIO_CLEAR", 16, 0xFFC01524);
		d("PORTDIO_DIR", 16, 0xFFC01554);
		d("PORTDIO_FER", 16, 0xFFC01504);
		d("PORTDIO_INEN", 16, 0xFFC01564);
		d("PORTDIO_SET", 16, 0xFFC01534);
		d("PORTDIO_TOGGLE", 16, 0xFFC01544);

		parent = debugfs_create_dir("GPIO Port E", top);
		d("PORTEIO", 16, 0xFFC01518);
		d("PORTEIO_CLEAR", 16, 0xFFC01528);
		d("PORTEIO_DIR", 16, 0xFFC01558);
		d("PORTEIO_FER", 16, 0xFFC01508);
		d("PORTEIO_INEN", 16, 0xFFC01568);
		d("PORTEIO_SET", 16, 0xFFC01538);
		d("PORTEIO_TOGGLE", 16, 0xFFC01548);

		parent = debugfs_create_dir("GPIO Port F", top);
		d("PORTFIO", 16, 0xFFC00700);
		d("PORTFIO_BOTH", 16, 0xFFC0073C);
		d("PORTFIO_CLEAR", 16, 0xFFC00704);
		d("PORTFIO_DIR", 16, 0xFFC00730);
		d("PORTFIO_EDGE", 16, 0xFFC00738);
		d("PORTFIO_INEN", 16, 0xFFC00740);
		d("PORTFIO_MASKA", 16, 0xFFC00710);
		d("PORTFIO_MASKA_CLEAR", 16, 0xFFC00714);
		d("PORTFIO_MASKA_SET", 16, 0xFFC00718);
		d("PORTFIO_MASKA_TOGGLE", 16, 0xFFC0071C);
		d("PORTFIO_MASKB", 16, 0xFFC00720);
		d("PORTFIO_MASKB_CLEAR", 16, 0xFFC00724);
		d("PORTFIO_MASKB_SET", 16, 0xFFC00728);
		d("PORTFIO_MASKB_TOGGLE", 16, 0xFFC0072C);
		d("PORTFIO_POLAR", 16, 0xFFC00734);
		d("PORTFIO_SET", 16, 0xFFC00708);
		d("PORTFIO_TOGGLE", 16, 0xFFC0070C);

		parent = debugfs_create_dir("MEMDMA0 Destination0 Registers", top);
		d("MDMA0_D0_CONFIG", 16, 0xFFC00E08);
		d("MDMA0_D0_CURR_ADDR", 32, 0xFFC00E24);
		d("MDMA0_D0_CURR_DESC_PTR", 32, 0xFFC00E20);
		d("MDMA0_D0_CURR_X_COUNT", 16, 0xFFC00E30);
		d("MDMA0_D0_CURR_Y_COUNT", 16, 0xFFC00E38);
		d("MDMA0_D0_IRQ_STATUS", 16, 0xFFC00E28);
		d("MDMA0_D0_NEXT_DESC_PTR", 32, 0xFFC00E00);
		d("MDMA0_D0_PERIPHERAL_MAP", 16, 0xFFC00E2C);
		d("MDMA0_D0_START_ADDR", 32, 0xFFC00E04);
		d("MDMA0_D0_X_COUNT", 16, 0xFFC00E10);
		d("MDMA0_D0_X_MODIFY", 16, 0xFFC00E14);
		d("MDMA0_D0_Y_COUNT", 16, 0xFFC00E18);
		d("MDMA0_D0_Y_MODIFY", 16, 0xFFC00E1C);

		parent = debugfs_create_dir("MEMDMA0 Destination1 Registers", top);
		d("MDMA0_D1_CONFIG", 16, 0xFFC00E88);
		d("MDMA0_D1_CURR_ADDR", 32, 0xFFC00EA4);
		d("MDMA0_D1_CURR_DESC_PTR", 32, 0xFFC00EA0);
		d("MDMA0_D1_CURR_X_COUNT", 16, 0xFFC00EB0);
		d("MDMA0_D1_CURR_Y_COUNT", 16, 0xFFC00EB8);
		d("MDMA0_D1_IRQ_STATUS", 16, 0xFFC00EA8);
		d("MDMA0_D1_NEXT_DESC_PTR", 32, 0xFFC00E80);
		d("MDMA0_D1_PERIPHERAL_MAP", 16, 0xFFC00EAC);
		d("MDMA0_D1_START_ADDR", 32, 0xFFC00E84);
		d("MDMA0_D1_X_COUNT", 16, 0xFFC00E90);
		d("MDMA0_D1_X_MODIFY", 16, 0xFFC00E94);
		d("MDMA0_D1_Y_COUNT", 16, 0xFFC00E98);
		d("MDMA0_D1_Y_MODIFY", 16, 0xFFC00E9C);

		parent = debugfs_create_dir("MEMDMA0 Source0 Registers", top);
		d("MDMA0_S0_CONFIG", 16, 0xFFC00E48);
		d("MDMA0_S0_CURR_ADDR", 32, 0xFFC00E64);
		d("MDMA0_S0_CURR_DESC_PTR", 32, 0xFFC00E60);
		d("MDMA0_S0_CURR_X_COUNT", 16, 0xFFC00E70);
		d("MDMA0_S0_CURR_Y_COUNT", 16, 0xFFC00E78);
		d("MDMA0_S0_IRQ_STATUS", 16, 0xFFC00E68);
		d("MDMA0_S0_NEXT_DESC_PTR", 32, 0xFFC00E40);
		d("MDMA0_S0_PERIPHERAL_MAP", 16, 0xFFC00E6C);
		d("MDMA0_S0_START_ADDR", 32, 0xFFC00E44);
		d("MDMA0_S0_X_COUNT", 16, 0xFFC00E50);
		d("MDMA0_S0_X_MODIFY", 16, 0xFFC00E54);
		d("MDMA0_S0_Y_COUNT", 16, 0xFFC00E58);
		d("MDMA0_S0_Y_MODIFY", 16, 0xFFC00E5C);

		parent = debugfs_create_dir("MEMDMA0 Source1 Registers", top);
		d("MDMA0_S1_CONFIG", 16, 0xFFC00EC8);
		d("MDMA0_S1_CURR_ADDR", 32, 0xFFC00EE4);
		d("MDMA0_S1_CURR_DESC_PTR", 32, 0xFFC00EE0);
		d("MDMA0_S1_CURR_X_COUNT", 16, 0xFFC00EF0);
		d("MDMA0_S1_CURR_Y_COUNT", 16, 0xFFC00EF8);
		d("MDMA0_S1_IRQ_STATUS", 16, 0xFFC00EE8);
		d("MDMA0_S1_NEXT_DESC_PTR", 32, 0xFFC00EC0);
		d("MDMA0_S1_PERIPHERAL_MAP", 16, 0xFFC00EEC);
		d("MDMA0_S1_START_ADDR", 32, 0xFFC00EC4);
		d("MDMA0_S1_X_COUNT", 16, 0xFFC00ED0);
		d("MDMA0_S1_X_MODIFY", 16, 0xFFC00ED4);
		d("MDMA0_S1_Y_COUNT", 16, 0xFFC00ED8);
		d("MDMA0_S1_Y_MODIFY", 16, 0xFFC00EDC);

		parent = debugfs_create_dir("MEMDMA1 Destination0 Registers", top);
		d("MDMA1_D0_CONFIG", 16, 0xFFC01F08);
		d("MDMA1_D0_CURR_ADDR", 32, 0xFFC01F24);
		d("MDMA1_D0_CURR_DESC_PTR", 32, 0xFFC01F20);
		d("MDMA1_D0_CURR_X_COUNT", 16, 0xFFC01F30);
		d("MDMA1_D0_CURR_Y_COUNT", 16, 0xFFC01F38);
		d("MDMA1_D0_IRQ_STATUS", 16, 0xFFC01F28);
		d("MDMA1_D0_NEXT_DESC_PTR", 32, 0xFFC01F00);
		d("MDMA1_D0_PERIPHERAL_MAP", 16, 0xFFC01F2C);
		d("MDMA1_D0_START_ADDR", 32, 0xFFC01F04);
		d("MDMA1_D0_X_COUNT", 16, 0xFFC01F10);
		d("MDMA1_D0_X_MODIFY", 16, 0xFFC01F14);
		d("MDMA1_D0_Y_COUNT", 16, 0xFFC01F18);
		d("MDMA1_D0_Y_MODIFY", 16, 0xFFC01F1C);

		parent = debugfs_create_dir("MEMDMA1 Destination1 Registers", top);
		d("MDMA1_D1_CONFIG", 16, 0xFFC01F88);
		d("MDMA1_D1_CURR_ADDR", 32, 0xFFC01FA4);
		d("MDMA1_D1_CURR_DESC_PTR", 32, 0xFFC01FA0);
		d("MDMA1_D1_CURR_X_COUNT", 16, 0xFFC01FB0);
		d("MDMA1_D1_CURR_Y_COUNT", 16, 0xFFC01FB8);
		d("MDMA1_D1_IRQ_STATUS", 16, 0xFFC01FA8);
		d("MDMA1_D1_NEXT_DESC_PTR", 32, 0xFFC01F80);
		d("MDMA1_D1_PERIPHERAL_MAP", 16, 0xFFC01FAC);
		d("MDMA1_D1_START_ADDR", 32, 0xFFC01F84);
		d("MDMA1_D1_X_COUNT", 16, 0xFFC01F90);
		d("MDMA1_D1_X_MODIFY", 16, 0xFFC01F94);
		d("MDMA1_D1_Y_COUNT", 16, 0xFFC01F98);
		d("MDMA1_D1_Y_MODIFY", 16, 0xFFC01F9C);

		parent = debugfs_create_dir("MEMDMA1 Source0 Registers", top);
		d("MDMA1_S0_CONFIG", 16, 0xFFC01F48);
		d("MDMA1_S0_CURR_ADDR", 32, 0xFFC01F64);
		d("MDMA1_S0_CURR_DESC_PTR", 32, 0xFFC01F60);
		d("MDMA1_S0_CURR_X_COUNT", 16, 0xFFC01F70);
		d("MDMA1_S0_CURR_Y_COUNT", 16, 0xFFC01F78);
		d("MDMA1_S0_IRQ_STATUS", 16, 0xFFC01F68);
		d("MDMA1_S0_NEXT_DESC_PTR", 32, 0xFFC01F40);
		d("MDMA1_S0_PERIPHERAL_MAP", 16, 0xFFC01F6C);
		d("MDMA1_S0_START_ADDR", 32, 0xFFC01F44);
		d("MDMA1_S0_X_COUNT", 16, 0xFFC01F50);
		d("MDMA1_S0_X_MODIFY", 16, 0xFFC01F54);
		d("MDMA1_S0_Y_COUNT", 16, 0xFFC01F58);
		d("MDMA1_S0_Y_MODIFY", 16, 0xFFC01F5C);

		parent = debugfs_create_dir("MEMDMA1 Source1 Registers", top);
		d("MDMA1_S1_CONFIG", 16, 0xFFC01FC8);
		d("MDMA1_S1_CURR_ADDR", 32, 0xFFC01FE4);
		d("MDMA1_S1_CURR_DESC_PTR", 32, 0xFFC01FE0);
		d("MDMA1_S1_CURR_X_COUNT", 16, 0xFFC01FF0);
		d("MDMA1_S1_CURR_Y_COUNT", 16, 0xFFC01FF8);
		d("MDMA1_S1_IRQ_STATUS", 16, 0xFFC01FE8);
		d("MDMA1_S1_NEXT_DESC_PTR", 32, 0xFFC01FC0);
		d("MDMA1_S1_PERIPHERAL_MAP", 16, 0xFFC01FEC);
		d("MDMA1_S1_START_ADDR", 32, 0xFFC01FC4);
		d("MDMA1_S1_X_COUNT", 16, 0xFFC01FD0);
		d("MDMA1_S1_X_MODIFY", 16, 0xFFC01FD4);
		d("MDMA1_S1_Y_COUNT", 16, 0xFFC01FD8);
		d("MDMA1_S1_Y_MODIFY", 16, 0xFFC01FDC);

		parent = debugfs_create_dir("SPI", top);
		d("SPI0_BAUD", 16, 0xFFC00514);
		d("SPI0_CTL", 16, 0xFFC00500);
		d("SPI0_FLG", 16, 0xFFC00504);
		d("SPI0_RDBR", 16, 0xFFC00510);
		d("SPI0_SHADOW", 16, 0xFFC00518);
		d("SPI0_STAT", 16, 0xFFC00508);
		d("SPI0_TDBR", 16, 0xFFC0050C);
		d("SPI1_BAUD", 16, 0xFFC02314);
		d("SPI1_CTL", 16, 0xFFC02300);
		d("SPI1_FLG", 16, 0xFFC02304);
		d("SPI1_RDBR", 16, 0xFFC02310);
		d("SPI1_SHADOW", 16, 0xFFC02318);
		d("SPI1_STAT", 16, 0xFFC02308);
		d("SPI1_TDBR", 16, 0xFFC0230C);
		d("SPI2_BAUD", 16, 0xFFC02414);
		d("SPI2_CTL", 16, 0xFFC02400);
		d("SPI2_FLG", 16, 0xFFC02404);
		d("SPI2_RDBR", 16, 0xFFC02410);
		d("SPI2_SHADOW", 16, 0xFFC02418);
		d("SPI2_STAT", 16, 0xFFC02408);
		d("SPI2_TDBR", 16, 0xFFC0240C);

		parent = debugfs_create_dir("TWI", top);
		d("TWI0_CLKDIV", 16, 0xFFC01400);
		d("TWI0_CONTROL", 16, 0xFFC01404);
		d("TWI0_FIFO_CTL", 16, 0xFFC01428);
		d("TWI0_FIFO_STAT", 16, 0xFFC0142C);
		d("TWI0_INT_MASK", 16, 0xFFC01424);
		d("TWI0_INT_STAT", 16, 0xFFC01420);
		d("TWI0_MASTER_ADDR", 16, 0xFFC0141C);
		d("TWI0_MASTER_CTL", 16, 0xFFC01414);
		d("TWI0_MASTER_STAT", 16, 0xFFC01418);
		d("TWI0_RCV_DATA16", 16, 0xFFC0148C);
		d("TWI0_RCV_DATA8", 16, 0xFFC01488);
		d("TWI0_SLAVE_ADDR", 16, 0xFFC01410);
		d("TWI0_SLAVE_CTRL", 16, 0xFFC01408);
		d("TWI0_SLAVE_STAT", 16, 0xFFC0140C);
		d("TWI0_XMT_DATA16", 16, 0xFFC01484);
		d("TWI0_XMT_DATA8", 16, 0xFFC01480);
		d("TWI1_CLKDIV", 16, 0xFFC02200);
		d("TWI1_CONTROL", 16, 0xFFC02204);
		d("TWI1_FIFO_CTL", 16, 0xFFC02228);
		d("TWI1_FIFO_STAT", 16, 0xFFC0222C);
		d("TWI1_INT_MASK", 16, 0xFFC02224);
		d("TWI1_INT_STAT", 16, 0xFFC02220);
		d("TWI1_MASTER_ADDR", 16, 0xFFC0221C);
		d("TWI1_MASTER_CTL", 16, 0xFFC02214);
		d("TWI1_MASTER_STAT", 16, 0xFFC02218);
		d("TWI1_RCV_DATA16", 16, 0xFFC0228C);
		d("TWI1_RCV_DATA8", 16, 0xFFC02288);
		d("TWI1_SLAVE_ADDR", 16, 0xFFC02210);
		d("TWI1_SLAVE_CTRL", 16, 0xFFC02208);
		d("TWI1_SLAVE_STAT", 16, 0xFFC0220C);
		d("TWI1_XMT_DATA16", 16, 0xFFC02284);
		d("TWI1_XMT_DATA8", 16, 0xFFC02280);

		parent = debugfs_create_dir("UART", top);
		d("UART0_DLH", 16, 0xFFC00404);
		d("UART0_DLL", 16, 0xFFC00400);
		d("UART0_GCTL", 16, 0xFFC00424);
		d("UART0_IER", 16, 0xFFC00404);
		d("UART0_IIR", 16, 0xFFC00408);
		d("UART0_LCR", 16, 0xFFC0040C);
		d("UART0_LSR", 16, 0xFFC00414);
		d("UART0_MCR", 16, 0xFFC00410);
		d("UART0_RBR", 16, 0xFFC00400);
		d("UART0_SCR", 16, 0xFFC0041C);
		d("UART0_THR", 16, 0xFFC00400);
		d("UART1_DLH", 16, 0xFFC02004);
		d("UART1_DLL", 16, 0xFFC02000);
		d("UART1_GCTL", 16, 0xFFC02024);
		d("UART1_IER", 16, 0xFFC02004);
		d("UART1_IIR", 16, 0xFFC02008);
		d("UART1_LCR", 16, 0xFFC0200C);
		d("UART1_LSR", 16, 0xFFC02014);
		d("UART1_MCR", 16, 0xFFC02010);
		d("UART1_RBR", 16, 0xFFC02000);
		d("UART1_SCR", 16, 0xFFC0201C);
		d("UART1_THR", 16, 0xFFC02000);
		d("UART2_DLH", 16, 0xFFC02104);
		d("UART2_DLL", 16, 0xFFC02100);
		d("UART2_GCTL", 16, 0xFFC02124);
		d("UART2_IER", 16, 0xFFC02104);
		d("UART2_IIR", 16, 0xFFC02108);
		d("UART2_LCR", 16, 0xFFC0210C);
		d("UART2_LSR", 16, 0xFFC02114);
		d("UART2_MCR", 16, 0xFFC02110);
		d("UART2_RBR", 16, 0xFFC02100);
		d("UART2_SCR", 16, 0xFFC0211C);
		d("UART2_THR", 16, 0xFFC02100);

	}	/* BF538 BF539 */

#ifdef __ADSPBF542__
# define USE_BF542 1
#else
# define USE_BF542 0
#endif
	if (USE_BF542) {

		parent = debugfs_create_dir("DMA", top);
		d("DMA0_CONFIG", 16, 0xFFC00C08);
		d("DMA0_CURR_ADDR", 32, 0xFFC00C24);
		d("DMA0_CURR_DESC_PTR", 32, 0xFFC00C20);
		d("DMA0_CURR_X_COUNT", 16, 0xFFC00C30);
		d("DMA0_CURR_Y_COUNT", 16, 0xFFC00C38);
		d("DMA0_IRQ_STATUS", 16, 0xFFC00C28);
		d("DMA0_NEXT_DESC_PTR", 32, 0xFFC00C00);
		d("DMA0_PERIPHERAL_MAP", 16, 0xFFC00C2C);
		d("DMA0_START_ADDR", 32, 0xFFC00C04);
		d("DMA0_X_COUNT", 16, 0xFFC00C10);
		d("DMA0_X_MODIFY", 16, 0xFFC00C14);
		d("DMA0_Y_COUNT", 16, 0xFFC00C18);
		d("DMA0_Y_MODIFY", 16, 0xFFC00C1C);
		d("DMA10_CONFIG", 16, 0xFFC00E88);
		d("DMA10_CURR_ADDR", 32, 0xFFC00EA4);
		d("DMA10_CURR_DESC_PTR", 32, 0xFFC00EA0);
		d("DMA10_CURR_X_COUNT", 16, 0xFFC00EB0);
		d("DMA10_CURR_Y_COUNT", 16, 0xFFC00EB8);
		d("DMA10_IRQ_STATUS", 16, 0xFFC00EA8);
		d("DMA10_NEXT_DESC_PTR", 32, 0xFFC00E80);
		d("DMA10_PERIPHERAL_MAP", 16, 0xFFC00EAC);
		d("DMA10_START_ADDR", 32, 0xFFC00E84);
		d("DMA10_X_COUNT", 16, 0xFFC00E90);
		d("DMA10_X_MODIFY", 16, 0xFFC00E94);
		d("DMA10_Y_COUNT", 16, 0xFFC00E98);
		d("DMA10_Y_MODIFY", 16, 0xFFC00E9C);
		d("DMA11_CONFIG", 16, 0xFFC00EC8);
		d("DMA11_CURR_ADDR", 32, 0xFFC00EE4);
		d("DMA11_CURR_DESC_PTR", 32, 0xFFC00EE0);
		d("DMA11_CURR_X_COUNT", 16, 0xFFC00EF0);
		d("DMA11_CURR_Y_COUNT", 16, 0xFFC00EF8);
		d("DMA11_IRQ_STATUS", 16, 0xFFC00EE8);
		d("DMA11_NEXT_DESC_PTR", 32, 0xFFC00EC0);
		d("DMA11_PERIPHERAL_MAP", 16, 0xFFC00EEC);
		d("DMA11_START_ADDR", 32, 0xFFC00EC4);
		d("DMA11_X_COUNT", 16, 0xFFC00ED0);
		d("DMA11_X_MODIFY", 16, 0xFFC00ED4);
		d("DMA11_Y_COUNT", 16, 0xFFC00ED8);
		d("DMA11_Y_MODIFY", 16, 0xFFC00EDC);
		d("DMA12_CONFIG", 16, 0xFFC01C08);
		d("DMA12_CURR_ADDR", 32, 0xFFC01C24);
		d("DMA12_CURR_DESC_PTR", 32, 0xFFC01C20);
		d("DMA12_CURR_X_COUNT", 16, 0xFFC01C30);
		d("DMA12_CURR_Y_COUNT", 16, 0xFFC01C38);
		d("DMA12_IRQ_STATUS", 16, 0xFFC01C28);
		d("DMA12_NEXT_DESC_PTR", 32, 0xFFC01C00);
		d("DMA12_PERIPHERAL_MAP", 16, 0xFFC01C2C);
		d("DMA12_START_ADDR", 32, 0xFFC01C04);
		d("DMA12_X_COUNT", 16, 0xFFC01C10);
		d("DMA12_X_MODIFY", 16, 0xFFC01C14);
		d("DMA12_Y_COUNT", 16, 0xFFC01C18);
		d("DMA12_Y_MODIFY", 16, 0xFFC01C1C);
		d("DMA13_CONFIG", 16, 0xFFC01C48);
		d("DMA13_CURR_ADDR", 32, 0xFFC01C64);
		d("DMA13_CURR_DESC_PTR", 32, 0xFFC01C60);
		d("DMA13_CURR_X_COUNT", 16, 0xFFC01C70);
		d("DMA13_CURR_Y_COUNT", 16, 0xFFC01C78);
		d("DMA13_IRQ_STATUS", 16, 0xFFC01C68);
		d("DMA13_NEXT_DESC_PTR", 32, 0xFFC01C40);
		d("DMA13_PERIPHERAL_MAP", 16, 0xFFC01C6C);
		d("DMA13_START_ADDR", 32, 0xFFC01C44);
		d("DMA13_X_COUNT", 16, 0xFFC01C50);
		d("DMA13_X_MODIFY", 16, 0xFFC01C54);
		d("DMA13_Y_COUNT", 16, 0xFFC01C58);
		d("DMA13_Y_MODIFY", 16, 0xFFC01C5C);
		d("DMA14_CONFIG", 16, 0xFFC01C88);
		d("DMA14_CURR_ADDR", 32, 0xFFC01CA4);
		d("DMA14_CURR_DESC_PTR", 32, 0xFFC01CA0);
		d("DMA14_CURR_X_COUNT", 16, 0xFFC01CB0);
		d("DMA14_CURR_Y_COUNT", 16, 0xFFC01CB8);
		d("DMA14_IRQ_STATUS", 16, 0xFFC01CA8);
		d("DMA14_NEXT_DESC_PTR", 32, 0xFFC01C80);
		d("DMA14_PERIPHERAL_MAP", 16, 0xFFC01CAC);
		d("DMA14_START_ADDR", 32, 0xFFC01C84);
		d("DMA14_X_COUNT", 16, 0xFFC01C90);
		d("DMA14_X_MODIFY", 16, 0xFFC01C94);
		d("DMA14_Y_COUNT", 16, 0xFFC01C98);
		d("DMA14_Y_MODIFY", 16, 0xFFC01C9C);
		d("DMA15_CONFIG", 16, 0xFFC01CC8);
		d("DMA15_CURR_ADDR", 32, 0xFFC01CE4);
		d("DMA15_CURR_DESC_PTR", 32, 0xFFC01CE0);
		d("DMA15_CURR_X_COUNT", 16, 0xFFC01CF0);
		d("DMA15_CURR_Y_COUNT", 16, 0xFFC01CF8);
		d("DMA15_IRQ_STATUS", 16, 0xFFC01CE8);
		d("DMA15_NEXT_DESC_PTR", 32, 0xFFC01CC0);
		d("DMA15_PERIPHERAL_MAP", 16, 0xFFC01CEC);
		d("DMA15_START_ADDR", 32, 0xFFC01CC4);
		d("DMA15_X_COUNT", 16, 0xFFC01CD0);
		d("DMA15_X_MODIFY", 16, 0xFFC01CD4);
		d("DMA15_Y_COUNT", 16, 0xFFC01CD8);
		d("DMA15_Y_MODIFY", 16, 0xFFC01CDC);
		d("DMA16_CONFIG", 16, 0xFFC01D08);
		d("DMA16_CURR_ADDR", 32, 0xFFC01D24);
		d("DMA16_CURR_DESC_PTR", 32, 0xFFC01D20);
		d("DMA16_CURR_X_COUNT", 16, 0xFFC01D30);
		d("DMA16_CURR_Y_COUNT", 16, 0xFFC01D38);
		d("DMA16_IRQ_STATUS", 16, 0xFFC01D28);
		d("DMA16_NEXT_DESC_PTR", 32, 0xFFC01D00);
		d("DMA16_PERIPHERAL_MAP", 16, 0xFFC01D2C);
		d("DMA16_START_ADDR", 32, 0xFFC01D04);
		d("DMA16_X_COUNT", 16, 0xFFC01D10);
		d("DMA16_X_MODIFY", 16, 0xFFC01D14);
		d("DMA16_Y_COUNT", 16, 0xFFC01D18);
		d("DMA16_Y_MODIFY", 16, 0xFFC01D1C);
		d("DMA17_CONFIG", 16, 0xFFC01D48);
		d("DMA17_CURR_ADDR", 32, 0xFFC01D64);
		d("DMA17_CURR_DESC_PTR", 32, 0xFFC01D60);
		d("DMA17_CURR_X_COUNT", 16, 0xFFC01D70);
		d("DMA17_CURR_Y_COUNT", 16, 0xFFC01D78);
		d("DMA17_IRQ_STATUS", 16, 0xFFC01D68);
		d("DMA17_NEXT_DESC_PTR", 32, 0xFFC01D40);
		d("DMA17_PERIPHERAL_MAP", 16, 0xFFC01D6C);
		d("DMA17_START_ADDR", 32, 0xFFC01D44);
		d("DMA17_X_COUNT", 16, 0xFFC01D50);
		d("DMA17_X_MODIFY", 16, 0xFFC01D54);
		d("DMA17_Y_COUNT", 16, 0xFFC01D58);
		d("DMA17_Y_MODIFY", 16, 0xFFC01D5C);
		d("DMA18_CONFIG", 16, 0xFFC01D88);
		d("DMA18_CURR_ADDR", 32, 0xFFC01DA4);
		d("DMA18_CURR_DESC_PTR", 32, 0xFFC01DA0);
		d("DMA18_CURR_X_COUNT", 16, 0xFFC01DB0);
		d("DMA18_CURR_Y_COUNT", 16, 0xFFC01DB8);
		d("DMA18_IRQ_STATUS", 16, 0xFFC01DA8);
		d("DMA18_NEXT_DESC_PTR", 32, 0xFFC01D80);
		d("DMA18_PERIPHERAL_MAP", 16, 0xFFC01DAC);
		d("DMA18_START_ADDR", 32, 0xFFC01D84);
		d("DMA18_X_COUNT", 16, 0xFFC01D90);
		d("DMA18_X_MODIFY", 16, 0xFFC01D94);
		d("DMA18_Y_COUNT", 16, 0xFFC01D98);
		d("DMA18_Y_MODIFY", 16, 0xFFC01D9C);
		d("DMA19_CONFIG", 16, 0xFFC01DC8);
		d("DMA19_CURR_ADDR", 32, 0xFFC01DE4);
		d("DMA19_CURR_DESC_PTR", 32, 0xFFC01DE0);
		d("DMA19_CURR_X_COUNT", 16, 0xFFC01DF0);
		d("DMA19_CURR_Y_COUNT", 16, 0xFFC01DF8);
		d("DMA19_IRQ_STATUS", 16, 0xFFC01DE8);
		d("DMA19_NEXT_DESC_PTR", 32, 0xFFC01DC0);
		d("DMA19_PERIPHERAL_MAP", 16, 0xFFC01DEC);
		d("DMA19_START_ADDR", 32, 0xFFC01DC4);
		d("DMA19_X_COUNT", 16, 0xFFC01DD0);
		d("DMA19_X_MODIFY", 16, 0xFFC01DD4);
		d("DMA19_Y_COUNT", 16, 0xFFC01DD8);
		d("DMA19_Y_MODIFY", 16, 0xFFC01DDC);
		d("DMA1_CONFIG", 16, 0xFFC00C48);
		d("DMA1_CURR_ADDR", 32, 0xFFC00C64);
		d("DMA1_CURR_DESC_PTR", 32, 0xFFC00C60);
		d("DMA1_CURR_X_COUNT", 16, 0xFFC00C70);
		d("DMA1_CURR_Y_COUNT", 16, 0xFFC00C78);
		d("DMA1_IRQ_STATUS", 16, 0xFFC00C68);
		d("DMA1_NEXT_DESC_PTR", 32, 0xFFC00C40);
		d("DMA1_PERIPHERAL_MAP", 16, 0xFFC00C6C);
		d("DMA1_START_ADDR", 32, 0xFFC00C44);
		d("DMA1_X_COUNT", 16, 0xFFC00C50);
		d("DMA1_X_MODIFY", 16, 0xFFC00C54);
		d("DMA1_Y_COUNT", 16, 0xFFC00C58);
		d("DMA1_Y_MODIFY", 16, 0xFFC00C5C);
		d("DMA20_CONFIG", 16, 0xFFC01E08);
		d("DMA20_CURR_ADDR", 32, 0xFFC01E24);
		d("DMA20_CURR_DESC_PTR", 32, 0xFFC01E20);
		d("DMA20_CURR_X_COUNT", 16, 0xFFC01E30);
		d("DMA20_CURR_Y_COUNT", 16, 0xFFC01E38);
		d("DMA20_IRQ_STATUS", 16, 0xFFC01E28);
		d("DMA20_NEXT_DESC_PTR", 32, 0xFFC01E00);
		d("DMA20_PERIPHERAL_MAP", 16, 0xFFC01E2C);
		d("DMA20_START_ADDR", 32, 0xFFC01E04);
		d("DMA20_X_COUNT", 16, 0xFFC01E10);
		d("DMA20_X_MODIFY", 16, 0xFFC01E14);
		d("DMA20_Y_COUNT", 16, 0xFFC01E18);
		d("DMA20_Y_MODIFY", 16, 0xFFC01E1C);
		d("DMA21_CONFIG", 16, 0xFFC01E48);
		d("DMA21_CURR_ADDR", 32, 0xFFC01E64);
		d("DMA21_CURR_DESC_PTR", 32, 0xFFC01E60);
		d("DMA21_CURR_X_COUNT", 16, 0xFFC01E70);
		d("DMA21_CURR_Y_COUNT", 16, 0xFFC01E78);
		d("DMA21_IRQ_STATUS", 16, 0xFFC01E68);
		d("DMA21_NEXT_DESC_PTR", 32, 0xFFC01E40);
		d("DMA21_PERIPHERAL_MAP", 16, 0xFFC01E6C);
		d("DMA21_START_ADDR", 32, 0xFFC01E44);
		d("DMA21_X_COUNT", 16, 0xFFC01E50);
		d("DMA21_X_MODIFY", 16, 0xFFC01E54);
		d("DMA21_Y_COUNT", 16, 0xFFC01E58);
		d("DMA21_Y_MODIFY", 16, 0xFFC01E5C);
		d("DMA22_CONFIG", 16, 0xFFC01E88);
		d("DMA22_CURR_ADDR", 32, 0xFFC01EA4);
		d("DMA22_CURR_DESC_PTR", 32, 0xFFC01EA0);
		d("DMA22_CURR_X_COUNT", 16, 0xFFC01EB0);
		d("DMA22_CURR_Y_COUNT", 16, 0xFFC01EB8);
		d("DMA22_IRQ_STATUS", 16, 0xFFC01EA8);
		d("DMA22_NEXT_DESC_PTR", 32, 0xFFC01E80);
		d("DMA22_PERIPHERAL_MAP", 16, 0xFFC01EAC);
		d("DMA22_START_ADDR", 32, 0xFFC01E84);
		d("DMA22_X_COUNT", 16, 0xFFC01E90);
		d("DMA22_X_MODIFY", 16, 0xFFC01E94);
		d("DMA22_Y_COUNT", 16, 0xFFC01E98);
		d("DMA22_Y_MODIFY", 16, 0xFFC01E9C);
		d("DMA23_CONFIG", 16, 0xFFC01EC8);
		d("DMA23_CURR_ADDR", 32, 0xFFC01EE4);
		d("DMA23_CURR_DESC_PTR", 32, 0xFFC01EE0);
		d("DMA23_CURR_X_COUNT", 16, 0xFFC01EF0);
		d("DMA23_CURR_Y_COUNT", 16, 0xFFC01EF8);
		d("DMA23_IRQ_STATUS", 16, 0xFFC01EE8);
		d("DMA23_NEXT_DESC_PTR", 32, 0xFFC01EC0);
		d("DMA23_PERIPHERAL_MAP", 16, 0xFFC01EEC);
		d("DMA23_START_ADDR", 32, 0xFFC01EC4);
		d("DMA23_X_COUNT", 16, 0xFFC01ED0);
		d("DMA23_X_MODIFY", 16, 0xFFC01ED4);
		d("DMA23_Y_COUNT", 16, 0xFFC01ED8);
		d("DMA23_Y_MODIFY", 16, 0xFFC01EDC);
		d("DMA2_CONFIG", 16, 0xFFC00C88);
		d("DMA2_CURR_ADDR", 32, 0xFFC00CA4);
		d("DMA2_CURR_DESC_PTR", 32, 0xFFC00CA0);
		d("DMA2_CURR_X_COUNT", 16, 0xFFC00CB0);
		d("DMA2_CURR_Y_COUNT", 16, 0xFFC00CB8);
		d("DMA2_IRQ_STATUS", 16, 0xFFC00CA8);
		d("DMA2_NEXT_DESC_PTR", 32, 0xFFC00C80);
		d("DMA2_PERIPHERAL_MAP", 16, 0xFFC00CAC);
		d("DMA2_START_ADDR", 32, 0xFFC00C84);
		d("DMA2_X_COUNT", 16, 0xFFC00C90);
		d("DMA2_X_MODIFY", 16, 0xFFC00C94);
		d("DMA2_Y_COUNT", 16, 0xFFC00C98);
		d("DMA2_Y_MODIFY", 16, 0xFFC00C9C);
		d("DMA3_CONFIG", 16, 0xFFC00CC8);
		d("DMA3_CURR_ADDR", 32, 0xFFC00CE4);
		d("DMA3_CURR_DESC_PTR", 32, 0xFFC00CE0);
		d("DMA3_CURR_X_COUNT", 16, 0xFFC00CF0);
		d("DMA3_CURR_Y_COUNT", 16, 0xFFC00CF8);
		d("DMA3_IRQ_STATUS", 16, 0xFFC00CE8);
		d("DMA3_NEXT_DESC_PTR", 32, 0xFFC00CC0);
		d("DMA3_PERIPHERAL_MAP", 16, 0xFFC00CEC);
		d("DMA3_START_ADDR", 32, 0xFFC00CC4);
		d("DMA3_X_COUNT", 16, 0xFFC00CD0);
		d("DMA3_X_MODIFY", 16, 0xFFC00CD4);
		d("DMA3_Y_COUNT", 16, 0xFFC00CD8);
		d("DMA3_Y_MODIFY", 16, 0xFFC00CDC);
		d("DMA4_CONFIG", 16, 0xFFC00D08);
		d("DMA4_CURR_ADDR", 32, 0xFFC00D24);
		d("DMA4_CURR_DESC_PTR", 32, 0xFFC00D20);
		d("DMA4_CURR_X_COUNT", 16, 0xFFC00D30);
		d("DMA4_CURR_Y_COUNT", 16, 0xFFC00D38);
		d("DMA4_IRQ_STATUS", 16, 0xFFC00D28);
		d("DMA4_NEXT_DESC_PTR", 32, 0xFFC00D00);
		d("DMA4_PERIPHERAL_MAP", 16, 0xFFC00D2C);
		d("DMA4_START_ADDR", 32, 0xFFC00D04);
		d("DMA4_X_COUNT", 16, 0xFFC00D10);
		d("DMA4_X_MODIFY", 16, 0xFFC00D14);
		d("DMA4_Y_COUNT", 16, 0xFFC00D18);
		d("DMA4_Y_MODIFY", 16, 0xFFC00D1C);
		d("DMA5_CONFIG", 16, 0xFFC00D48);
		d("DMA5_CURR_ADDR", 32, 0xFFC00D64);
		d("DMA5_CURR_DESC_PTR", 32, 0xFFC00D60);
		d("DMA5_CURR_X_COUNT", 16, 0xFFC00D70);
		d("DMA5_CURR_Y_COUNT", 16, 0xFFC00D78);
		d("DMA5_IRQ_STATUS", 16, 0xFFC00D68);
		d("DMA5_NEXT_DESC_PTR", 32, 0xFFC00D40);
		d("DMA5_PERIPHERAL_MAP", 16, 0xFFC00D6C);
		d("DMA5_START_ADDR", 32, 0xFFC00D44);
		d("DMA5_X_COUNT", 16, 0xFFC00D50);
		d("DMA5_X_MODIFY", 16, 0xFFC00D54);
		d("DMA5_Y_COUNT", 16, 0xFFC00D58);
		d("DMA5_Y_MODIFY", 16, 0xFFC00D5C);
		d("DMA6_CONFIG", 16, 0xFFC00D88);
		d("DMA6_CURR_ADDR", 32, 0xFFC00DA4);
		d("DMA6_CURR_DESC_PTR", 32, 0xFFC00DA0);
		d("DMA6_CURR_X_COUNT", 16, 0xFFC00DB0);
		d("DMA6_CURR_Y_COUNT", 16, 0xFFC00DB8);
		d("DMA6_IRQ_STATUS", 16, 0xFFC00DA8);
		d("DMA6_NEXT_DESC_PTR", 32, 0xFFC00D80);
		d("DMA6_PERIPHERAL_MAP", 16, 0xFFC00DAC);
		d("DMA6_START_ADDR", 32, 0xFFC00D84);
		d("DMA6_X_COUNT", 16, 0xFFC00D90);
		d("DMA6_X_MODIFY", 16, 0xFFC00D94);
		d("DMA6_Y_COUNT", 16, 0xFFC00D98);
		d("DMA6_Y_MODIFY", 16, 0xFFC00D9C);
		d("DMA7_CONFIG", 16, 0xFFC00DC8);
		d("DMA7_CURR_ADDR", 32, 0xFFC00DE4);
		d("DMA7_CURR_DESC_PTR", 32, 0xFFC00DE0);
		d("DMA7_CURR_X_COUNT", 16, 0xFFC00DF0);
		d("DMA7_CURR_Y_COUNT", 16, 0xFFC00DF8);
		d("DMA7_IRQ_STATUS", 16, 0xFFC00DE8);
		d("DMA7_NEXT_DESC_PTR", 32, 0xFFC00DC0);
		d("DMA7_PERIPHERAL_MAP", 16, 0xFFC00DEC);
		d("DMA7_START_ADDR", 32, 0xFFC00DC4);
		d("DMA7_X_COUNT", 16, 0xFFC00DD0);
		d("DMA7_X_MODIFY", 16, 0xFFC00DD4);
		d("DMA7_Y_COUNT", 16, 0xFFC00DD8);
		d("DMA7_Y_MODIFY", 16, 0xFFC00DDC);
		d("DMA8_CONFIG", 16, 0xFFC00E08);
		d("DMA8_CURR_ADDR", 32, 0xFFC00E24);
		d("DMA8_CURR_DESC_PTR", 32, 0xFFC00E20);
		d("DMA8_CURR_X_COUNT", 16, 0xFFC00E30);
		d("DMA8_CURR_Y_COUNT", 16, 0xFFC00E38);
		d("DMA8_IRQ_STATUS", 16, 0xFFC00E28);
		d("DMA8_NEXT_DESC_PTR", 32, 0xFFC00E00);
		d("DMA8_PERIPHERAL_MAP", 16, 0xFFC00E2C);
		d("DMA8_START_ADDR", 32, 0xFFC00E04);
		d("DMA8_X_COUNT", 16, 0xFFC00E10);
		d("DMA8_X_MODIFY", 16, 0xFFC00E14);
		d("DMA8_Y_COUNT", 16, 0xFFC00E18);
		d("DMA8_Y_MODIFY", 16, 0xFFC00E1C);
		d("DMA9_CONFIG", 16, 0xFFC00E48);
		d("DMA9_CURR_ADDR", 32, 0xFFC00E64);
		d("DMA9_CURR_DESC_PTR", 32, 0xFFC00E60);
		d("DMA9_CURR_X_COUNT", 16, 0xFFC00E70);
		d("DMA9_CURR_Y_COUNT", 16, 0xFFC00E78);
		d("DMA9_IRQ_STATUS", 16, 0xFFC00E68);
		d("DMA9_NEXT_DESC_PTR", 32, 0xFFC00E40);
		d("DMA9_PERIPHERAL_MAP", 16, 0xFFC00E6C);
		d("DMA9_START_ADDR", 32, 0xFFC00E44);
		d("DMA9_X_COUNT", 16, 0xFFC00E50);
		d("DMA9_X_MODIFY", 16, 0xFFC00E54);
		d("DMA9_Y_COUNT", 16, 0xFFC00E58);
		d("DMA9_Y_MODIFY", 16, 0xFFC00E5C);
		d("DMAC0_TCCNT", 16, 0xFFC00B10);
		d("DMAC0_TCPER", 16, 0xFFC00B0C);
		d("DMAC1_PERIMUX", 16, 0xFFC04340);
		d("DMAC1_TCCNT", 16, 0xFFC01B10);
		d("DMAC1_TCPER", 16, 0xFFC01B0C);

		parent = debugfs_create_dir("HMDMA", top);
		d("HMDMA0_BCINIT", 16, 0xFFC04508);
		d("HMDMA0_BCOUNT", 16, 0xFFC04518);
		d("HMDMA0_CONTROL", 16, 0xFFC04500);
		d("HMDMA0_ECINIT", 16, 0xFFC04504);
		d("HMDMA0_ECOUNT", 16, 0xFFC04514);
		d("HMDMA0_ECOVERFLOW", 16, 0xFFC04510);
		d("HMDMA0_ECURGENT", 16, 0xFFC0450C);
		d("HMDMA1_BCINIT", 16, 0xFFC04548);
		d("HMDMA1_BCOUNT", 16, 0xFFC04558);
		d("HMDMA1_CONTROL", 16, 0xFFC04540);
		d("HMDMA1_ECINIT", 16, 0xFFC04544);
		d("HMDMA1_ECOUNT", 16, 0xFFC04554);
		d("HMDMA1_ECOVERFLOW", 16, 0xFFC04550);
		d("HMDMA1_ECURGENT", 16, 0xFFC0454C);

		parent = debugfs_create_dir("MDMA", top);
		d("MDMA_D0_CONFIG", 16, 0xFFC00F08);
		d("MDMA_D0_CURR_ADDR", 32, 0xFFC00F24);
		d("MDMA_D0_CURR_DESC_PTR", 32, 0xFFC00F20);
		d("MDMA_D0_CURR_X_COUNT", 16, 0xFFC00F30);
		d("MDMA_D0_CURR_Y_COUNT", 16, 0xFFC00F38);
		d("MDMA_D0_IRQ_STATUS", 16, 0xFFC00F28);
		d("MDMA_D0_NEXT_DESC_PTR", 32, 0xFFC00F00);
		d("MDMA_D0_PERIPHERAL_MAP", 16, 0xFFC00F2C);
		d("MDMA_D0_START_ADDR", 32, 0xFFC00F04);
		d("MDMA_D0_X_COUNT", 16, 0xFFC00F10);
		d("MDMA_D0_X_MODIFY", 16, 0xFFC00F14);
		d("MDMA_D0_Y_COUNT", 16, 0xFFC00F18);
		d("MDMA_D0_Y_MODIFY", 16, 0xFFC00F1C);
		d("MDMA_D1_CONFIG", 16, 0xFFC00F88);
		d("MDMA_D1_CURR_ADDR", 32, 0xFFC00FA4);
		d("MDMA_D1_CURR_DESC_PTR", 32, 0xFFC00FA0);
		d("MDMA_D1_CURR_X_COUNT", 16, 0xFFC00FB0);
		d("MDMA_D1_CURR_Y_COUNT", 16, 0xFFC00FB8);
		d("MDMA_D1_IRQ_STATUS", 16, 0xFFC00FA8);
		d("MDMA_D1_NEXT_DESC_PTR", 32, 0xFFC00F80);
		d("MDMA_D1_PERIPHERAL_MAP", 16, 0xFFC00FAC);
		d("MDMA_D1_START_ADDR", 32, 0xFFC00F84);
		d("MDMA_D1_X_COUNT", 16, 0xFFC00F90);
		d("MDMA_D1_X_MODIFY", 16, 0xFFC00F94);
		d("MDMA_D1_Y_COUNT", 16, 0xFFC00F98);
		d("MDMA_D1_Y_MODIFY", 16, 0xFFC00F9C);
		d("MDMA_D2_CONFIG", 16, 0xFFC01F08);
		d("MDMA_D2_CURR_ADDR", 32, 0xFFC01F24);
		d("MDMA_D2_CURR_DESC_PTR", 32, 0xFFC01F20);
		d("MDMA_D2_CURR_X_COUNT", 16, 0xFFC01F30);
		d("MDMA_D2_CURR_Y_COUNT", 16, 0xFFC01F38);
		d("MDMA_D2_IRQ_STATUS", 16, 0xFFC01F28);
		d("MDMA_D2_NEXT_DESC_PTR", 32, 0xFFC01F00);
		d("MDMA_D2_PERIPHERAL_MAP", 16, 0xFFC01F2C);
		d("MDMA_D2_START_ADDR", 32, 0xFFC01F04);
		d("MDMA_D2_X_COUNT", 16, 0xFFC01F10);
		d("MDMA_D2_X_MODIFY", 16, 0xFFC01F14);
		d("MDMA_D2_Y_COUNT", 16, 0xFFC01F18);
		d("MDMA_D2_Y_MODIFY", 16, 0xFFC01F1C);
		d("MDMA_D3_CONFIG", 16, 0xFFC01F88);
		d("MDMA_D3_CURR_ADDR", 32, 0xFFC01FA4);
		d("MDMA_D3_CURR_DESC_PTR", 32, 0xFFC01FA0);
		d("MDMA_D3_CURR_X_COUNT", 16, 0xFFC01FB0);
		d("MDMA_D3_CURR_Y_COUNT", 16, 0xFFC01FB8);
		d("MDMA_D3_IRQ_STATUS", 16, 0xFFC01FA8);
		d("MDMA_D3_NEXT_DESC_PTR", 32, 0xFFC01F80);
		d("MDMA_D3_PERIPHERAL_MAP", 16, 0xFFC01FAC);
		d("MDMA_D3_START_ADDR", 32, 0xFFC01F84);
		d("MDMA_D3_X_COUNT", 16, 0xFFC01F90);
		d("MDMA_D3_X_MODIFY", 16, 0xFFC01F94);
		d("MDMA_D3_Y_COUNT", 16, 0xFFC01F98);
		d("MDMA_D3_Y_MODIFY", 16, 0xFFC01F9C);
		d("MDMA_S0_CONFIG", 16, 0xFFC00F48);
		d("MDMA_S0_CURR_ADDR", 32, 0xFFC00F64);
		d("MDMA_S0_CURR_DESC_PTR", 32, 0xFFC00F60);
		d("MDMA_S0_CURR_X_COUNT", 16, 0xFFC00F70);
		d("MDMA_S0_CURR_Y_COUNT", 16, 0xFFC00F78);
		d("MDMA_S0_IRQ_STATUS", 16, 0xFFC00F68);
		d("MDMA_S0_NEXT_DESC_PTR", 32, 0xFFC00F40);
		d("MDMA_S0_PERIPHERAL_MAP", 16, 0xFFC00F6C);
		d("MDMA_S0_START_ADDR", 32, 0xFFC00F44);
		d("MDMA_S0_X_COUNT", 16, 0xFFC00F50);
		d("MDMA_S0_X_MODIFY", 16, 0xFFC00F54);
		d("MDMA_S0_Y_COUNT", 16, 0xFFC00F58);
		d("MDMA_S0_Y_MODIFY", 16, 0xFFC00F5C);
		d("MDMA_S1_CONFIG", 16, 0xFFC00FC8);
		d("MDMA_S1_CURR_ADDR", 32, 0xFFC00FE4);
		d("MDMA_S1_CURR_DESC_PTR", 32, 0xFFC00FE0);
		d("MDMA_S1_CURR_X_COUNT", 16, 0xFFC00FF0);
		d("MDMA_S1_CURR_Y_COUNT", 16, 0xFFC00FF8);
		d("MDMA_S1_IRQ_STATUS", 16, 0xFFC00FE8);
		d("MDMA_S1_NEXT_DESC_PTR", 32, 0xFFC00FC0);
		d("MDMA_S1_PERIPHERAL_MAP", 16, 0xFFC00FEC);
		d("MDMA_S1_START_ADDR", 32, 0xFFC00FC4);
		d("MDMA_S1_X_COUNT", 16, 0xFFC00FD0);
		d("MDMA_S1_X_MODIFY", 16, 0xFFC00FD4);
		d("MDMA_S1_Y_COUNT", 16, 0xFFC00FD8);
		d("MDMA_S1_Y_MODIFY", 16, 0xFFC00FDC);
		d("MDMA_S2_CONFIG", 16, 0xFFC01F48);
		d("MDMA_S2_CURR_ADDR", 32, 0xFFC01F64);
		d("MDMA_S2_CURR_DESC_PTR", 32, 0xFFC01F60);
		d("MDMA_S2_CURR_X_COUNT", 16, 0xFFC01F70);
		d("MDMA_S2_CURR_Y_COUNT", 16, 0xFFC01F78);
		d("MDMA_S2_IRQ_STATUS", 16, 0xFFC01F68);
		d("MDMA_S2_NEXT_DESC_PTR", 32, 0xFFC01F40);
		d("MDMA_S2_PERIPHERAL_MAP", 16, 0xFFC01F6C);
		d("MDMA_S2_START_ADDR", 32, 0xFFC01F44);
		d("MDMA_S2_X_COUNT", 16, 0xFFC01F50);
		d("MDMA_S2_X_MODIFY", 16, 0xFFC01F54);
		d("MDMA_S2_Y_COUNT", 16, 0xFFC01F58);
		d("MDMA_S2_Y_MODIFY", 16, 0xFFC01F5C);
		d("MDMA_S3_CONFIG", 16, 0xFFC01FC8);
		d("MDMA_S3_CURR_ADDR", 32, 0xFFC01FE4);
		d("MDMA_S3_CURR_DESC_PTR", 32, 0xFFC01FE0);
		d("MDMA_S3_CURR_X_COUNT", 16, 0xFFC01FF0);
		d("MDMA_S3_CURR_Y_COUNT", 16, 0xFFC01FF8);
		d("MDMA_S3_IRQ_STATUS", 16, 0xFFC01FE8);
		d("MDMA_S3_NEXT_DESC_PTR", 32, 0xFFC01FC0);
		d("MDMA_S3_PERIPHERAL_MAP", 16, 0xFFC01FEC);
		d("MDMA_S3_START_ADDR", 32, 0xFFC01FC4);
		d("MDMA_S3_X_COUNT", 16, 0xFFC01FD0);
		d("MDMA_S3_X_MODIFY", 16, 0xFFC01FD4);
		d("MDMA_S3_Y_COUNT", 16, 0xFFC01FD8);
		d("MDMA_S3_Y_MODIFY", 16, 0xFFC01FDC);

		parent = debugfs_create_dir("PINT_0", top);
		d("PINT0_ASSIGN", 32, 0xFFC0140C);
		d("PINT0_EDGE_CLEAR", 32, 0xFFC01414);
		d("PINT0_EDGE_SET", 32, 0xFFC01410);
		d("PINT0_INVERT_CLEAR", 32, 0xFFC0141C);
		d("PINT0_INVERT_SET", 32, 0xFFC01418);
		d("PINT0_IRQ", 32, 0xFFC01408);
		d("PINT0_LATCH", 32, 0xFFC01424);
		d("PINT0_MASK_CLEAR", 32, 0xFFC01404);
		d("PINT0_MASK_SET", 32, 0xFFC01400);
		d("PINT0_PINSTATE", 32, 0xFFC01420);

		parent = debugfs_create_dir("PINT_1", top);
		d("PINT1_ASSIGN", 32, 0xFFC0143C);
		d("PINT1_EDGE_CLEAR", 32, 0xFFC01444);
		d("PINT1_EDGE_SET", 32, 0xFFC01440);
		d("PINT1_INVERT_CLEAR", 32, 0xFFC0144C);
		d("PINT1_INVERT_SET", 32, 0xFFC01448);
		d("PINT1_IRQ", 32, 0xFFC01438);
		d("PINT1_LATCH", 32, 0xFFC01454);
		d("PINT1_MASK_CLEAR", 32, 0xFFC01434);
		d("PINT1_MASK_SET", 32, 0xFFC01430);
		d("PINT1_PINSTATE", 32, 0xFFC01450);

		parent = debugfs_create_dir("PINT_2", top);
		d("PINT2_ASSIGN", 32, 0xFFC0146C);
		d("PINT2_EDGE_CLEAR", 32, 0xFFC01474);
		d("PINT2_EDGE_SET", 32, 0xFFC01470);
		d("PINT2_INVERT_CLEAR", 32, 0xFFC0147C);
		d("PINT2_INVERT_SET", 32, 0xFFC01478);
		d("PINT2_IRQ", 32, 0xFFC01468);
		d("PINT2_LATCH", 32, 0xFFC01484);
		d("PINT2_MASK_CLEAR", 32, 0xFFC01464);
		d("PINT2_MASK_SET", 32, 0xFFC01460);
		d("PINT2_PINSTATE", 32, 0xFFC01480);

		parent = debugfs_create_dir("PINT_3", top);
		d("PINT3_ASSIGN", 32, 0xFFC0149C);
		d("PINT3_EDGE_CLEAR", 32, 0xFFC014A4);
		d("PINT3_EDGE_SET", 32, 0xFFC014A0);
		d("PINT3_INVERT_CLEAR", 32, 0xFFC014AC);
		d("PINT3_INVERT_SET", 32, 0xFFC014A8);
		d("PINT3_IRQ", 32, 0xFFC01498);
		d("PINT3_LATCH", 32, 0xFFC014B4);
		d("PINT3_MASK_CLEAR", 32, 0xFFC01494);
		d("PINT3_MASK_SET", 32, 0xFFC01490);
		d("PINT3_PINSTATE", 32, 0xFFC014B0);

		parent = debugfs_create_dir("Port_A", top);
		d("PORTA", 16, 0xFFC014C4);
		d("PORTA_CLEAR", 16, 0xFFC014CC);
		d("PORTA_DIR_CLEAR", 16, 0xFFC014D4);
		d("PORTA_DIR_SET", 16, 0xFFC014D0);
		d("PORTA_FER", 16, 0xFFC014C0);
		d("PORTA_INEN", 16, 0xFFC014D8);
		d("PORTA_MUX", 32, 0xFFC014DC);
		d("PORTA_SET", 16, 0xFFC014C8);

		parent = debugfs_create_dir("Port_B", top);
		d("PORTB", 16, 0xFFC014E4);
		d("PORTB_CLEAR", 16, 0xFFC014EC);
		d("PORTB_DIR_CLEAR", 16, 0xFFC014F4);
		d("PORTB_DIR_SET", 16, 0xFFC014F0);
		d("PORTB_FER", 16, 0xFFC014E0);
		d("PORTB_INEN", 16, 0xFFC014F8);
		d("PORTB_MUX", 32, 0xFFC014FC);
		d("PORTB_SET", 16, 0xFFC014E8);

		parent = debugfs_create_dir("Port_C", top);
		d("PORTC", 16, 0xFFC01504);
		d("PORTC_CLEAR", 16, 0xFFC0150C);
		d("PORTC_DIR_CLEAR", 16, 0xFFC01514);
		d("PORTC_DIR_SET", 16, 0xFFC01510);
		d("PORTC_FER", 16, 0xFFC01500);
		d("PORTC_INEN", 16, 0xFFC01518);
		d("PORTC_MUX", 32, 0xFFC0151C);
		d("PORTC_SET", 16, 0xFFC01508);

		parent = debugfs_create_dir("Port_D", top);
		d("PORTD", 16, 0xFFC01524);
		d("PORTD_CLEAR", 16, 0xFFC0152C);
		d("PORTD_DIR_CLEAR", 16, 0xFFC01534);
		d("PORTD_DIR_SET", 16, 0xFFC01530);
		d("PORTD_FER", 16, 0xFFC01520);
		d("PORTD_INEN", 16, 0xFFC01538);
		d("PORTD_MUX", 32, 0xFFC0153C);
		d("PORTD_SET", 16, 0xFFC01528);

		parent = debugfs_create_dir("Port_E", top);
		d("PORTE", 16, 0xFFC01544);
		d("PORTE_CLEAR", 16, 0xFFC0154C);
		d("PORTE_DIR_CLEAR", 16, 0xFFC01554);
		d("PORTE_DIR_SET", 16, 0xFFC01550);
		d("PORTE_FER", 16, 0xFFC01540);
		d("PORTE_INEN", 16, 0xFFC01558);
		d("PORTE_MUX", 32, 0xFFC0155C);
		d("PORTE_SET", 16, 0xFFC01548);

		parent = debugfs_create_dir("Port_F", top);
		d("PORTF", 16, 0xFFC01564);
		d("PORTF_CLEAR", 16, 0xFFC0156C);
		d("PORTF_DIR_CLEAR", 16, 0xFFC01574);
		d("PORTF_DIR_SET", 16, 0xFFC01570);
		d("PORTF_FER", 16, 0xFFC01560);
		d("PORTF_INEN", 16, 0xFFC01578);
		d("PORTF_MUX", 32, 0xFFC0157C);
		d("PORTF_SET", 16, 0xFFC01568);

		parent = debugfs_create_dir("Port_G", top);
		d("PORTG", 16, 0xFFC01584);
		d("PORTG_CLEAR", 16, 0xFFC0158C);
		d("PORTG_DIR_CLEAR", 16, 0xFFC01594);
		d("PORTG_DIR_SET", 16, 0xFFC01590);
		d("PORTG_FER", 16, 0xFFC01580);
		d("PORTG_INEN", 16, 0xFFC01598);
		d("PORTG_MUX", 32, 0xFFC0159C);
		d("PORTG_SET", 16, 0xFFC01588);

		parent = debugfs_create_dir("Port_H", top);
		d("PORTH", 16, 0xFFC015A4);
		d("PORTH_CLEAR", 16, 0xFFC015AC);
		d("PORTH_DIR_CLEAR", 16, 0xFFC015B4);
		d("PORTH_DIR_SET", 16, 0xFFC015B0);
		d("PORTH_FER", 16, 0xFFC015A0);
		d("PORTH_INEN", 16, 0xFFC015B8);
		d("PORTH_MUX", 32, 0xFFC015BC);
		d("PORTH_SET", 16, 0xFFC015A8);

		parent = debugfs_create_dir("Port_I", top);
		d("PORTI", 16, 0xFFC015C4);
		d("PORTI_CLEAR", 16, 0xFFC015CC);
		d("PORTI_DIR_CLEAR", 16, 0xFFC015D4);
		d("PORTI_DIR_SET", 16, 0xFFC015D0);
		d("PORTI_FER", 16, 0xFFC015C0);
		d("PORTI_INEN", 16, 0xFFC015D8);
		d("PORTI_MUX", 32, 0xFFC015DC);
		d("PORTI_SET", 16, 0xFFC015C8);

		parent = debugfs_create_dir("Port_J", top);
		d("PORTJ", 16, 0xFFC015E4);
		d("PORTJ_CLEAR", 16, 0xFFC015EC);
		d("PORTJ_DIR_CLEAR", 16, 0xFFC015F4);
		d("PORTJ_DIR_SET", 16, 0xFFC015F0);
		d("PORTJ_FER", 16, 0xFFC015E0);
		d("PORTJ_INEN", 16, 0xFFC015F8);
		d("PORTJ_MUX", 32, 0xFFC015FC);
		d("PORTJ_SET", 16, 0xFFC015E8);

		parent = debugfs_create_dir("SPI", top);
		d("SPI0_BAUD", 16, 0xFFC00514);
		d("SPI0_CTL", 16, 0xFFC00500);
		d("SPI0_FLG", 16, 0xFFC00504);
		d("SPI0_RDBR", 16, 0xFFC00510);
		d("SPI0_SHADOW", 16, 0xFFC00518);
		d("SPI0_STAT", 16, 0xFFC00508);
		d("SPI0_TDBR", 16, 0xFFC0050C);
		d("SPI1_BAUD", 16, 0xFFC02314);
		d("SPI1_CTL", 16, 0xFFC02300);
		d("SPI1_FLG", 16, 0xFFC02304);
		d("SPI1_RDBR", 16, 0xFFC02310);
		d("SPI1_SHADOW", 16, 0xFFC02318);
		d("SPI1_STAT", 16, 0xFFC02308);
		d("SPI1_TDBR", 16, 0xFFC0230C);

		parent = debugfs_create_dir("TWI", top);
		d("TWI0_CLKDIV", 16, 0xFFC00700);
		d("TWI0_CONTROL", 16, 0xFFC00704);
		d("TWI0_FIFO_CTL", 16, 0xFFC00728);
		d("TWI0_FIFO_STAT", 16, 0xFFC0072C);
		d("TWI0_INT_MASK", 16, 0xFFC00724);
		d("TWI0_INT_STAT", 16, 0xFFC00720);
		d("TWI0_MASTER_ADDR", 16, 0xFFC0071C);
		d("TWI0_MASTER_CTL", 16, 0xFFC00714);
		d("TWI0_MASTER_STAT", 16, 0xFFC00718);
		d("TWI0_RCV_DATA16", 16, 0xFFC0078C);
		d("TWI0_RCV_DATA8", 16, 0xFFC00788);
		d("TWI0_SLAVE_ADDR", 16, 0xFFC00710);
		d("TWI0_SLAVE_CTL", 16, 0xFFC00708);
		d("TWI0_SLAVE_STAT", 16, 0xFFC0070C);
		d("TWI0_XMT_DATA16", 16, 0xFFC00784);
		d("TWI0_XMT_DATA8", 16, 0xFFC00780);

		parent = debugfs_create_dir("UART", top);
		d("UART0_DLH", 16, 0xFFC00404);
		d("UART0_DLL", 16, 0xFFC00400);
		d("UART0_GCTL", 16, 0xFFC00408);
		d("UART0_IER_CLEAR", 16, 0xFFC00424);
		d("UART0_IER_SET", 16, 0xFFC00420);
		d("UART0_LCR", 16, 0xFFC0040C);
		d("UART0_LSR", 16, 0xFFC00414);
		d("UART0_MCR", 16, 0xFFC00410);
		d("UART0_MSR", 16, 0xFFC00418);
		d("UART0_RBR", 16, 0xFFC0042C);
		d("UART0_SCR", 16, 0xFFC0041C);
		d("UART0_THR", 16, 0xFFC00428);
		d("UART1_DLH", 16, 0xFFC02004);
		d("UART1_DLL", 16, 0xFFC02000);
		d("UART1_GCTL", 16, 0xFFC02008);
		d("UART1_IER_CLEAR", 16, 0xFFC02024);
		d("UART1_IER_SET", 16, 0xFFC02020);
		d("UART1_LCR", 16, 0xFFC0200C);
		d("UART1_LSR", 16, 0xFFC02014);
		d("UART1_MCR", 16, 0xFFC02010);
		d("UART1_MSR", 16, 0xFFC02018);
		d("UART1_RBR", 16, 0xFFC0202C);
		d("UART1_SCR", 16, 0xFFC0201C);
		d("UART1_THR", 16, 0xFFC02028);
		d("UART3_DLH", 16, 0xFFC03104);
		d("UART3_DLL", 16, 0xFFC03100);
		d("UART3_GCTL", 16, 0xFFC03108);
		d("UART3_IER_CLEAR", 16, 0xFFC03124);
		d("UART3_IER_SET", 16, 0xFFC03120);
		d("UART3_LCR", 16, 0xFFC0310C);
		d("UART3_LSR", 16, 0xFFC03114);
		d("UART3_MCR", 16, 0xFFC03110);
		d("UART3_MSR", 16, 0xFFC03118);
		d("UART3_RBR", 16, 0xFFC0312C);
		d("UART3_SCR", 16, 0xFFC0311C);
		d("UART3_THR", 16, 0xFFC03128);

		parent = debugfs_create_dir("USB", top);
		d("USB_APHY_CALIB", 16, 0xFFC03DE4);
		d("USB_APHY_CNTRL2", 16, 0xFFC03DE8);
		d("USB_APHY_CNTRL", 16, 0xFFC03DE0);
		d("USB_COUNT0", 16, 0xFFC03C50);
		d("USB_CSR0", 16, 0xFFC03C44);
		d("USB_DMA0_ADDRHIGH", 16, 0xFFC0400C);
		d("USB_DMA0_ADDRLOW", 16, 0xFFC04008);
		d("USB_DMA0_CONTROL", 16, 0xFFC04004);
		d("USB_DMA0_COUNTHIGH", 16, 0xFFC04014);
		d("USB_DMA0_COUNTLOW", 16, 0xFFC04010);
		d("USB_DMA1_ADDRHIGH", 16, 0xFFC0402C);
		d("USB_DMA1_ADDRLOW", 16, 0xFFC04028);
		d("USB_DMA1_CONTROL", 16, 0xFFC04024);
		d("USB_DMA1_COUNTHIGH", 16, 0xFFC04034);
		d("USB_DMA1_COUNTLOW", 16, 0xFFC04030);
		d("USB_DMA2_ADDRHIGH", 16, 0xFFC0404C);
		d("USB_DMA2_ADDRLOW", 16, 0xFFC04048);
		d("USB_DMA2_CONTROL", 16, 0xFFC04044);
		d("USB_DMA2_COUNTHIGH", 16, 0xFFC04054);
		d("USB_DMA2_COUNTLOW", 16, 0xFFC04050);
		d("USB_DMA3_ADDRHIGH", 16, 0xFFC0406C);
		d("USB_DMA3_ADDRLOW", 16, 0xFFC04068);
		d("USB_DMA3_CONTROL", 16, 0xFFC04064);
		d("USB_DMA3_COUNTHIGH", 16, 0xFFC04074);
		d("USB_DMA3_COUNTLOW", 16, 0xFFC04070);
		d("USB_DMA4_ADDRHIGH", 16, 0xFFC0408C);
		d("USB_DMA4_ADDRLOW", 16, 0xFFC04088);
		d("USB_DMA4_CONTROL", 16, 0xFFC04084);
		d("USB_DMA4_COUNTHIGH", 16, 0xFFC04094);
		d("USB_DMA4_COUNTLOW", 16, 0xFFC04090);
		d("USB_DMA5_ADDRHIGH", 16, 0xFFC040AC);
		d("USB_DMA5_ADDRLOW", 16, 0xFFC040A8);
		d("USB_DMA5_CONTROL", 16, 0xFFC040A4);
		d("USB_DMA5_COUNTHIGH", 16, 0xFFC040B4);
		d("USB_DMA5_COUNTLOW", 16, 0xFFC040B0);
		d("USB_DMA6_ADDRHIGH", 16, 0xFFC040CC);
		d("USB_DMA6_ADDRLOW", 16, 0xFFC040C8);
		d("USB_DMA6_CONTROL", 16, 0xFFC040C4);
		d("USB_DMA6_COUNTHIGH", 16, 0xFFC040D4);
		d("USB_DMA6_COUNTLOW", 16, 0xFFC040D0);
		d("USB_DMA7_ADDRHIGH", 16, 0xFFC040EC);
		d("USB_DMA7_ADDRLOW", 16, 0xFFC040E8);
		d("USB_DMA7_CONTROL", 16, 0xFFC040E4);
		d("USB_DMA7_COUNTHIGH", 16, 0xFFC040F4);
		d("USB_DMA7_COUNTLOW", 16, 0xFFC040F0);
		d("USB_DMA_INTERRUPT", 16, 0xFFC04000);
		d("USB_EP0_FIFO", 16, 0xFFC03C80);
		d("USB_EP1_FIFO", 16, 0xFFC03C88);
		d("USB_EP2_FIFO", 16, 0xFFC03C90);
		d("USB_EP3_FIFO", 16, 0xFFC03C98);
		d("USB_EP4_FIFO", 16, 0xFFC03CA0);
		d("USB_EP5_FIFO", 16, 0xFFC03CA8);
		d("USB_EP6_FIFO", 16, 0xFFC03CB0);
		d("USB_EP7_FIFO", 16, 0xFFC03CB8);
		d("USB_EP_NI0_RXCOUNT", 16, 0xFFC03E10);
		d("USB_EP_NI0_RXCSR", 16, 0xFFC03E0C);
		d("USB_EP_NI0_RXINTERVAL", 16, 0xFFC03E20);
		d("USB_EP_NI0_RXMAXP", 16, 0xFFC03E08);
		d("USB_EP_NI0_RXTYPE", 16, 0xFFC03E1C);
		d("USB_EP_NI0_TXCOUNT", 16, 0xFFC03E28);
		d("USB_EP_NI0_TXCSR", 16, 0xFFC03E04);
		d("USB_EP_NI0_TXINTERVAL", 16, 0xFFC03E18);
		d("USB_EP_NI0_TXMAXP", 16, 0xFFC03E00);
		d("USB_EP_NI0_TXTYPE", 16, 0xFFC03E14);
		d("USB_EP_NI1_RXCOUNT", 16, 0xFFC03E50);
		d("USB_EP_NI1_RXCSR", 16, 0xFFC03E4C);
		d("USB_EP_NI1_RXINTERVAL", 16, 0xFFC03E60);
		d("USB_EP_NI1_RXMAXP", 16, 0xFFC03E48);
		d("USB_EP_NI1_RXTYPE", 16, 0xFFC03E5C);
		d("USB_EP_NI1_TXCOUNT", 16, 0xFFC03E68);
		d("USB_EP_NI1_TXCSR", 16, 0xFFC03E44);
		d("USB_EP_NI1_TXINTERVAL", 16, 0xFFC03E58);
		d("USB_EP_NI1_TXMAXP", 16, 0xFFC03E40);
		d("USB_EP_NI1_TXTYPE", 16, 0xFFC03E54);
		d("USB_EP_NI2_RXCOUNT", 16, 0xFFC03E90);
		d("USB_EP_NI2_RXCSR", 16, 0xFFC03E8C);
		d("USB_EP_NI2_RXINTERVAL", 16, 0xFFC03EA0);
		d("USB_EP_NI2_RXMAXP", 16, 0xFFC03E88);
		d("USB_EP_NI2_RXTYPE", 16, 0xFFC03E9C);
		d("USB_EP_NI2_TXCOUNT", 16, 0xFFC03EA8);
		d("USB_EP_NI2_TXCSR", 16, 0xFFC03E84);
		d("USB_EP_NI2_TXINTERVAL", 16, 0xFFC03E98);
		d("USB_EP_NI2_TXMAXP", 16, 0xFFC03E80);
		d("USB_EP_NI2_TXTYPE", 16, 0xFFC03E94);
		d("USB_EP_NI3_RXCOUNT", 16, 0xFFC03ED0);
		d("USB_EP_NI3_RXCSR", 16, 0xFFC03ECC);
		d("USB_EP_NI3_RXINTERVAL", 16, 0xFFC03EE0);
		d("USB_EP_NI3_RXMAXP", 16, 0xFFC03EC8);
		d("USB_EP_NI3_RXTYPE", 16, 0xFFC03EDC);
		d("USB_EP_NI3_TXCOUNT", 16, 0xFFC03EE8);
		d("USB_EP_NI3_TXCSR", 16, 0xFFC03EC4);
		d("USB_EP_NI3_TXINTERVAL", 16, 0xFFC03ED8);
		d("USB_EP_NI3_TXMAXP", 16, 0xFFC03EC0);
		d("USB_EP_NI3_TXTYPE", 16, 0xFFC03ED4);
		d("USB_EP_NI4_RXCOUNT", 16, 0xFFC03F10);
		d("USB_EP_NI4_RXCSR", 16, 0xFFC03F0C);
		d("USB_EP_NI4_RXINTERVAL", 16, 0xFFC03F20);
		d("USB_EP_NI4_RXMAXP", 16, 0xFFC03F08);
		d("USB_EP_NI4_RXTYPE", 16, 0xFFC03F1C);
		d("USB_EP_NI4_TXCOUNT", 16, 0xFFC03F28);
		d("USB_EP_NI4_TXCSR", 16, 0xFFC03F04);
		d("USB_EP_NI4_TXINTERVAL", 16, 0xFFC03F18);
		d("USB_EP_NI4_TXMAXP", 16, 0xFFC03F00);
		d("USB_EP_NI4_TXTYPE", 16, 0xFFC03F14);
		d("USB_EP_NI5_RXCOUNT", 16, 0xFFC03F50);
		d("USB_EP_NI5_RXCSR", 16, 0xFFC03F4C);
		d("USB_EP_NI5_RXINTERVAL", 16, 0xFFC03F60);
		d("USB_EP_NI5_RXMAXP", 16, 0xFFC03F48);
		d("USB_EP_NI5_RXTYPE", 16, 0xFFC03F5C);
		d("USB_EP_NI5_TXCOUNT", 16, 0xFFC03F68);
		d("USB_EP_NI5_TXCSR", 16, 0xFFC03F44);
		d("USB_EP_NI5_TXINTERVAL", 16, 0xFFC03F58);
		d("USB_EP_NI5_TXMAXP", 16, 0xFFC03F40);
		d("USB_EP_NI5_TXTYPE", 16, 0xFFC03F54);
		d("USB_EP_NI6_RXCOUNT", 16, 0xFFC03F90);
		d("USB_EP_NI6_RXCSR", 16, 0xFFC03F8C);
		d("USB_EP_NI6_RXINTERVAL", 16, 0xFFC03FA0);
		d("USB_EP_NI6_RXMAXP", 16, 0xFFC03F88);
		d("USB_EP_NI6_RXTYPE", 16, 0xFFC03F9C);
		d("USB_EP_NI6_TXCOUNT", 16, 0xFFC03FA8);
		d("USB_EP_NI6_TXCSR", 16, 0xFFC03F84);
		d("USB_EP_NI6_TXINTERVAL", 16, 0xFFC03F98);
		d("USB_EP_NI6_TXMAXP", 16, 0xFFC03F80);
		d("USB_EP_NI6_TXTYPE", 16, 0xFFC03F94);
		d("USB_EP_NI7_RXCOUNT", 16, 0xFFC03FD0);
		d("USB_EP_NI7_RXCSR", 16, 0xFFC03FCC);
		d("USB_EP_NI7_RXINTERVAL", 16, 0xFFC03FE0);
		d("USB_EP_NI7_RXMAXP", 16, 0xFFC03FC8);
		d("USB_EP_NI7_RXTYPE", 16, 0xFFC03FDC);
		d("USB_EP_NI7_TXCOUNT", 16, 0xFFC03FE8);
		d("USB_EP_NI7_TXCSR", 16, 0xFFC03FC4);
		d("USB_EP_NI7_TXINTERVAL", 16, 0xFFC03FD8);
		d("USB_EP_NI7_TXMAXP", 16, 0xFFC03FC0);
		d("USB_EP_NI7_TXTYPE", 16, 0xFFC03FD4);
		d("USB_FADDR", 16, 0xFFC03C00);
		d("USB_FRAME", 16, 0xFFC03C20);
		d("USB_FS_EOF1", 16, 0xFFC03D54);
		d("USB_GLOBAL_CTL", 16, 0xFFC03C30);
		d("USB_GLOBINTR", 16, 0xFFC03C2C);
		d("USB_HS_EOF1", 16, 0xFFC03D50);
		d("USB_INDEX", 16, 0xFFC03C24);
		d("USB_INTRRX", 16, 0xFFC03C0C);
		d("USB_INTRRXE", 16, 0xFFC03C14);
		d("USB_INTRTX", 16, 0xFFC03C08);
		d("USB_INTRTXE", 16, 0xFFC03C10);
		d("USB_INTRUSB", 16, 0xFFC03C18);
		d("USB_INTRUSBE", 16, 0xFFC03C1C);
		d("USB_LINKINFO", 16, 0xFFC03D48);
		d("USB_LS_EOF1", 16, 0xFFC03D58);
		d("USB_NAKLIMIT0", 16, 0xFFC03C58);
		d("USB_OTG_DEV_CTL", 16, 0xFFC03D00);
		d("USB_OTG_VBUS_IRQ", 16, 0xFFC03D04);
		d("USB_OTG_VBUS_MASK", 16, 0xFFC03D08);
		d("USB_PHY_TEST", 16, 0xFFC03DEC);
		d("USB_PLLOSC_CTRL", 16, 0xFFC03DF0);
		d("USB_POWER", 16, 0xFFC03C04);
		d("USB_RXCOUNT", 16, 0xFFC03C50);
		d("USB_RXCSR", 16, 0xFFC03C4C);
		d("USB_RXINTERVAL", 16, 0xFFC03C60);
		d("USB_RXTYPE", 16, 0xFFC03C5C);
		d("USB_RX_MAX_PACKET", 16, 0xFFC03C48);
		d("USB_SRP_CLKDIV", 16, 0xFFC03DF4);
		d("USB_TESTMODE", 16, 0xFFC03C28);
		d("USB_TXCOUNT", 16, 0xFFC03C68);
		d("USB_TXCSR", 16, 0xFFC03C44);
		d("USB_TXINTERVAL", 16, 0xFFC03C58);
		d("USB_TXTYPE", 16, 0xFFC03C54);
		d("USB_TX_MAX_PACKET", 16, 0xFFC03C40);
		d("USB_VPLEN", 16, 0xFFC03D4C);

	}	/* BF542 */

#ifdef __ADSPBF544__
# define USE_BF544 1
#else
# define USE_BF544 0
#endif
	if (USE_BF544) {

		parent = debugfs_create_dir("DMA", top);
		d("DMA0_CONFIG", 16, 0xFFC00C08);
		d("DMA0_CURR_ADDR", 32, 0xFFC00C24);
		d("DMA0_CURR_DESC_PTR", 32, 0xFFC00C20);
		d("DMA0_CURR_X_COUNT", 16, 0xFFC00C30);
		d("DMA0_CURR_Y_COUNT", 16, 0xFFC00C38);
		d("DMA0_IRQ_STATUS", 16, 0xFFC00C28);
		d("DMA0_NEXT_DESC_PTR", 32, 0xFFC00C00);
		d("DMA0_PERIPHERAL_MAP", 16, 0xFFC00C2C);
		d("DMA0_START_ADDR", 32, 0xFFC00C04);
		d("DMA0_X_COUNT", 16, 0xFFC00C10);
		d("DMA0_X_MODIFY", 16, 0xFFC00C14);
		d("DMA0_Y_COUNT", 16, 0xFFC00C18);
		d("DMA0_Y_MODIFY", 16, 0xFFC00C1C);
		d("DMA10_CONFIG", 16, 0xFFC00E88);
		d("DMA10_CURR_ADDR", 32, 0xFFC00EA4);
		d("DMA10_CURR_DESC_PTR", 32, 0xFFC00EA0);
		d("DMA10_CURR_X_COUNT", 16, 0xFFC00EB0);
		d("DMA10_CURR_Y_COUNT", 16, 0xFFC00EB8);
		d("DMA10_IRQ_STATUS", 16, 0xFFC00EA8);
		d("DMA10_NEXT_DESC_PTR", 32, 0xFFC00E80);
		d("DMA10_PERIPHERAL_MAP", 16, 0xFFC00EAC);
		d("DMA10_START_ADDR", 32, 0xFFC00E84);
		d("DMA10_X_COUNT", 16, 0xFFC00E90);
		d("DMA10_X_MODIFY", 16, 0xFFC00E94);
		d("DMA10_Y_COUNT", 16, 0xFFC00E98);
		d("DMA10_Y_MODIFY", 16, 0xFFC00E9C);
		d("DMA11_CONFIG", 16, 0xFFC00EC8);
		d("DMA11_CURR_ADDR", 32, 0xFFC00EE4);
		d("DMA11_CURR_DESC_PTR", 32, 0xFFC00EE0);
		d("DMA11_CURR_X_COUNT", 16, 0xFFC00EF0);
		d("DMA11_CURR_Y_COUNT", 16, 0xFFC00EF8);
		d("DMA11_IRQ_STATUS", 16, 0xFFC00EE8);
		d("DMA11_NEXT_DESC_PTR", 32, 0xFFC00EC0);
		d("DMA11_PERIPHERAL_MAP", 16, 0xFFC00EEC);
		d("DMA11_START_ADDR", 32, 0xFFC00EC4);
		d("DMA11_X_COUNT", 16, 0xFFC00ED0);
		d("DMA11_X_MODIFY", 16, 0xFFC00ED4);
		d("DMA11_Y_COUNT", 16, 0xFFC00ED8);
		d("DMA11_Y_MODIFY", 16, 0xFFC00EDC);
		d("DMA12_CONFIG", 16, 0xFFC01C08);
		d("DMA12_CURR_ADDR", 32, 0xFFC01C24);
		d("DMA12_CURR_DESC_PTR", 32, 0xFFC01C20);
		d("DMA12_CURR_X_COUNT", 16, 0xFFC01C30);
		d("DMA12_CURR_Y_COUNT", 16, 0xFFC01C38);
		d("DMA12_IRQ_STATUS", 16, 0xFFC01C28);
		d("DMA12_NEXT_DESC_PTR", 32, 0xFFC01C00);
		d("DMA12_PERIPHERAL_MAP", 16, 0xFFC01C2C);
		d("DMA12_START_ADDR", 32, 0xFFC01C04);
		d("DMA12_X_COUNT", 16, 0xFFC01C10);
		d("DMA12_X_MODIFY", 16, 0xFFC01C14);
		d("DMA12_Y_COUNT", 16, 0xFFC01C18);
		d("DMA12_Y_MODIFY", 16, 0xFFC01C1C);
		d("DMA13_CONFIG", 16, 0xFFC01C48);
		d("DMA13_CURR_ADDR", 32, 0xFFC01C64);
		d("DMA13_CURR_DESC_PTR", 32, 0xFFC01C60);
		d("DMA13_CURR_X_COUNT", 16, 0xFFC01C70);
		d("DMA13_CURR_Y_COUNT", 16, 0xFFC01C78);
		d("DMA13_IRQ_STATUS", 16, 0xFFC01C68);
		d("DMA13_NEXT_DESC_PTR", 32, 0xFFC01C40);
		d("DMA13_PERIPHERAL_MAP", 16, 0xFFC01C6C);
		d("DMA13_START_ADDR", 32, 0xFFC01C44);
		d("DMA13_X_COUNT", 16, 0xFFC01C50);
		d("DMA13_X_MODIFY", 16, 0xFFC01C54);
		d("DMA13_Y_COUNT", 16, 0xFFC01C58);
		d("DMA13_Y_MODIFY", 16, 0xFFC01C5C);
		d("DMA14_CONFIG", 16, 0xFFC01C88);
		d("DMA14_CURR_ADDR", 32, 0xFFC01CA4);
		d("DMA14_CURR_DESC_PTR", 32, 0xFFC01CA0);
		d("DMA14_CURR_X_COUNT", 16, 0xFFC01CB0);
		d("DMA14_CURR_Y_COUNT", 16, 0xFFC01CB8);
		d("DMA14_IRQ_STATUS", 16, 0xFFC01CA8);
		d("DMA14_NEXT_DESC_PTR", 32, 0xFFC01C80);
		d("DMA14_PERIPHERAL_MAP", 16, 0xFFC01CAC);
		d("DMA14_START_ADDR", 32, 0xFFC01C84);
		d("DMA14_X_COUNT", 16, 0xFFC01C90);
		d("DMA14_X_MODIFY", 16, 0xFFC01C94);
		d("DMA14_Y_COUNT", 16, 0xFFC01C98);
		d("DMA14_Y_MODIFY", 16, 0xFFC01C9C);
		d("DMA15_CONFIG", 16, 0xFFC01CC8);
		d("DMA15_CURR_ADDR", 32, 0xFFC01CE4);
		d("DMA15_CURR_DESC_PTR", 32, 0xFFC01CE0);
		d("DMA15_CURR_X_COUNT", 16, 0xFFC01CF0);
		d("DMA15_CURR_Y_COUNT", 16, 0xFFC01CF8);
		d("DMA15_IRQ_STATUS", 16, 0xFFC01CE8);
		d("DMA15_NEXT_DESC_PTR", 32, 0xFFC01CC0);
		d("DMA15_PERIPHERAL_MAP", 16, 0xFFC01CEC);
		d("DMA15_START_ADDR", 32, 0xFFC01CC4);
		d("DMA15_X_COUNT", 16, 0xFFC01CD0);
		d("DMA15_X_MODIFY", 16, 0xFFC01CD4);
		d("DMA15_Y_COUNT", 16, 0xFFC01CD8);
		d("DMA15_Y_MODIFY", 16, 0xFFC01CDC);
		d("DMA16_CONFIG", 16, 0xFFC01D08);
		d("DMA16_CURR_ADDR", 32, 0xFFC01D24);
		d("DMA16_CURR_DESC_PTR", 32, 0xFFC01D20);
		d("DMA16_CURR_X_COUNT", 16, 0xFFC01D30);
		d("DMA16_CURR_Y_COUNT", 16, 0xFFC01D38);
		d("DMA16_IRQ_STATUS", 16, 0xFFC01D28);
		d("DMA16_NEXT_DESC_PTR", 32, 0xFFC01D00);
		d("DMA16_PERIPHERAL_MAP", 16, 0xFFC01D2C);
		d("DMA16_START_ADDR", 32, 0xFFC01D04);
		d("DMA16_X_COUNT", 16, 0xFFC01D10);
		d("DMA16_X_MODIFY", 16, 0xFFC01D14);
		d("DMA16_Y_COUNT", 16, 0xFFC01D18);
		d("DMA16_Y_MODIFY", 16, 0xFFC01D1C);
		d("DMA17_CONFIG", 16, 0xFFC01D48);
		d("DMA17_CURR_ADDR", 32, 0xFFC01D64);
		d("DMA17_CURR_DESC_PTR", 32, 0xFFC01D60);
		d("DMA17_CURR_X_COUNT", 16, 0xFFC01D70);
		d("DMA17_CURR_Y_COUNT", 16, 0xFFC01D78);
		d("DMA17_IRQ_STATUS", 16, 0xFFC01D68);
		d("DMA17_NEXT_DESC_PTR", 32, 0xFFC01D40);
		d("DMA17_PERIPHERAL_MAP", 16, 0xFFC01D6C);
		d("DMA17_START_ADDR", 32, 0xFFC01D44);
		d("DMA17_X_COUNT", 16, 0xFFC01D50);
		d("DMA17_X_MODIFY", 16, 0xFFC01D54);
		d("DMA17_Y_COUNT", 16, 0xFFC01D58);
		d("DMA17_Y_MODIFY", 16, 0xFFC01D5C);
		d("DMA18_CONFIG", 16, 0xFFC01D88);
		d("DMA18_CURR_ADDR", 32, 0xFFC01DA4);
		d("DMA18_CURR_DESC_PTR", 32, 0xFFC01DA0);
		d("DMA18_CURR_X_COUNT", 16, 0xFFC01DB0);
		d("DMA18_CURR_Y_COUNT", 16, 0xFFC01DB8);
		d("DMA18_IRQ_STATUS", 16, 0xFFC01DA8);
		d("DMA18_NEXT_DESC_PTR", 32, 0xFFC01D80);
		d("DMA18_PERIPHERAL_MAP", 16, 0xFFC01DAC);
		d("DMA18_START_ADDR", 32, 0xFFC01D84);
		d("DMA18_X_COUNT", 16, 0xFFC01D90);
		d("DMA18_X_MODIFY", 16, 0xFFC01D94);
		d("DMA18_Y_COUNT", 16, 0xFFC01D98);
		d("DMA18_Y_MODIFY", 16, 0xFFC01D9C);
		d("DMA19_CONFIG", 16, 0xFFC01DC8);
		d("DMA19_CURR_ADDR", 32, 0xFFC01DE4);
		d("DMA19_CURR_DESC_PTR", 32, 0xFFC01DE0);
		d("DMA19_CURR_X_COUNT", 16, 0xFFC01DF0);
		d("DMA19_CURR_Y_COUNT", 16, 0xFFC01DF8);
		d("DMA19_IRQ_STATUS", 16, 0xFFC01DE8);
		d("DMA19_NEXT_DESC_PTR", 32, 0xFFC01DC0);
		d("DMA19_PERIPHERAL_MAP", 16, 0xFFC01DEC);
		d("DMA19_START_ADDR", 32, 0xFFC01DC4);
		d("DMA19_X_COUNT", 16, 0xFFC01DD0);
		d("DMA19_X_MODIFY", 16, 0xFFC01DD4);
		d("DMA19_Y_COUNT", 16, 0xFFC01DD8);
		d("DMA19_Y_MODIFY", 16, 0xFFC01DDC);
		d("DMA1_CONFIG", 16, 0xFFC00C48);
		d("DMA1_CURR_ADDR", 32, 0xFFC00C64);
		d("DMA1_CURR_DESC_PTR", 32, 0xFFC00C60);
		d("DMA1_CURR_X_COUNT", 16, 0xFFC00C70);
		d("DMA1_CURR_Y_COUNT", 16, 0xFFC00C78);
		d("DMA1_IRQ_STATUS", 16, 0xFFC00C68);
		d("DMA1_NEXT_DESC_PTR", 32, 0xFFC00C40);
		d("DMA1_PERIPHERAL_MAP", 16, 0xFFC00C6C);
		d("DMA1_START_ADDR", 32, 0xFFC00C44);
		d("DMA1_X_COUNT", 16, 0xFFC00C50);
		d("DMA1_X_MODIFY", 16, 0xFFC00C54);
		d("DMA1_Y_COUNT", 16, 0xFFC00C58);
		d("DMA1_Y_MODIFY", 16, 0xFFC00C5C);
		d("DMA20_CONFIG", 16, 0xFFC01E08);
		d("DMA20_CURR_ADDR", 32, 0xFFC01E24);
		d("DMA20_CURR_DESC_PTR", 32, 0xFFC01E20);
		d("DMA20_CURR_X_COUNT", 16, 0xFFC01E30);
		d("DMA20_CURR_Y_COUNT", 16, 0xFFC01E38);
		d("DMA20_IRQ_STATUS", 16, 0xFFC01E28);
		d("DMA20_NEXT_DESC_PTR", 32, 0xFFC01E00);
		d("DMA20_PERIPHERAL_MAP", 16, 0xFFC01E2C);
		d("DMA20_START_ADDR", 32, 0xFFC01E04);
		d("DMA20_X_COUNT", 16, 0xFFC01E10);
		d("DMA20_X_MODIFY", 16, 0xFFC01E14);
		d("DMA20_Y_COUNT", 16, 0xFFC01E18);
		d("DMA20_Y_MODIFY", 16, 0xFFC01E1C);
		d("DMA21_CONFIG", 16, 0xFFC01E48);
		d("DMA21_CURR_ADDR", 32, 0xFFC01E64);
		d("DMA21_CURR_DESC_PTR", 32, 0xFFC01E60);
		d("DMA21_CURR_X_COUNT", 16, 0xFFC01E70);
		d("DMA21_CURR_Y_COUNT", 16, 0xFFC01E78);
		d("DMA21_IRQ_STATUS", 16, 0xFFC01E68);
		d("DMA21_NEXT_DESC_PTR", 32, 0xFFC01E40);
		d("DMA21_PERIPHERAL_MAP", 16, 0xFFC01E6C);
		d("DMA21_START_ADDR", 32, 0xFFC01E44);
		d("DMA21_X_COUNT", 16, 0xFFC01E50);
		d("DMA21_X_MODIFY", 16, 0xFFC01E54);
		d("DMA21_Y_COUNT", 16, 0xFFC01E58);
		d("DMA21_Y_MODIFY", 16, 0xFFC01E5C);
		d("DMA22_CONFIG", 16, 0xFFC01E88);
		d("DMA22_CURR_ADDR", 32, 0xFFC01EA4);
		d("DMA22_CURR_DESC_PTR", 32, 0xFFC01EA0);
		d("DMA22_CURR_X_COUNT", 16, 0xFFC01EB0);
		d("DMA22_CURR_Y_COUNT", 16, 0xFFC01EB8);
		d("DMA22_IRQ_STATUS", 16, 0xFFC01EA8);
		d("DMA22_NEXT_DESC_PTR", 32, 0xFFC01E80);
		d("DMA22_PERIPHERAL_MAP", 16, 0xFFC01EAC);
		d("DMA22_START_ADDR", 32, 0xFFC01E84);
		d("DMA22_X_COUNT", 16, 0xFFC01E90);
		d("DMA22_X_MODIFY", 16, 0xFFC01E94);
		d("DMA22_Y_COUNT", 16, 0xFFC01E98);
		d("DMA22_Y_MODIFY", 16, 0xFFC01E9C);
		d("DMA23_CONFIG", 16, 0xFFC01EC8);
		d("DMA23_CURR_ADDR", 32, 0xFFC01EE4);
		d("DMA23_CURR_DESC_PTR", 32, 0xFFC01EE0);
		d("DMA23_CURR_X_COUNT", 16, 0xFFC01EF0);
		d("DMA23_CURR_Y_COUNT", 16, 0xFFC01EF8);
		d("DMA23_IRQ_STATUS", 16, 0xFFC01EE8);
		d("DMA23_NEXT_DESC_PTR", 32, 0xFFC01EC0);
		d("DMA23_PERIPHERAL_MAP", 16, 0xFFC01EEC);
		d("DMA23_START_ADDR", 32, 0xFFC01EC4);
		d("DMA23_X_COUNT", 16, 0xFFC01ED0);
		d("DMA23_X_MODIFY", 16, 0xFFC01ED4);
		d("DMA23_Y_COUNT", 16, 0xFFC01ED8);
		d("DMA23_Y_MODIFY", 16, 0xFFC01EDC);
		d("DMA2_CONFIG", 16, 0xFFC00C88);
		d("DMA2_CURR_ADDR", 32, 0xFFC00CA4);
		d("DMA2_CURR_DESC_PTR", 32, 0xFFC00CA0);
		d("DMA2_CURR_X_COUNT", 16, 0xFFC00CB0);
		d("DMA2_CURR_Y_COUNT", 16, 0xFFC00CB8);
		d("DMA2_IRQ_STATUS", 16, 0xFFC00CA8);
		d("DMA2_NEXT_DESC_PTR", 32, 0xFFC00C80);
		d("DMA2_PERIPHERAL_MAP", 16, 0xFFC00CAC);
		d("DMA2_START_ADDR", 32, 0xFFC00C84);
		d("DMA2_X_COUNT", 16, 0xFFC00C90);
		d("DMA2_X_MODIFY", 16, 0xFFC00C94);
		d("DMA2_Y_COUNT", 16, 0xFFC00C98);
		d("DMA2_Y_MODIFY", 16, 0xFFC00C9C);
		d("DMA3_CONFIG", 16, 0xFFC00CC8);
		d("DMA3_CURR_ADDR", 32, 0xFFC00CE4);
		d("DMA3_CURR_DESC_PTR", 32, 0xFFC00CE0);
		d("DMA3_CURR_X_COUNT", 16, 0xFFC00CF0);
		d("DMA3_CURR_Y_COUNT", 16, 0xFFC00CF8);
		d("DMA3_IRQ_STATUS", 16, 0xFFC00CE8);
		d("DMA3_NEXT_DESC_PTR", 32, 0xFFC00CC0);
		d("DMA3_PERIPHERAL_MAP", 16, 0xFFC00CEC);
		d("DMA3_START_ADDR", 32, 0xFFC00CC4);
		d("DMA3_X_COUNT", 16, 0xFFC00CD0);
		d("DMA3_X_MODIFY", 16, 0xFFC00CD4);
		d("DMA3_Y_COUNT", 16, 0xFFC00CD8);
		d("DMA3_Y_MODIFY", 16, 0xFFC00CDC);
		d("DMA4_CONFIG", 16, 0xFFC00D08);
		d("DMA4_CURR_ADDR", 32, 0xFFC00D24);
		d("DMA4_CURR_DESC_PTR", 32, 0xFFC00D20);
		d("DMA4_CURR_X_COUNT", 16, 0xFFC00D30);
		d("DMA4_CURR_Y_COUNT", 16, 0xFFC00D38);
		d("DMA4_IRQ_STATUS", 16, 0xFFC00D28);
		d("DMA4_NEXT_DESC_PTR", 32, 0xFFC00D00);
		d("DMA4_PERIPHERAL_MAP", 16, 0xFFC00D2C);
		d("DMA4_START_ADDR", 32, 0xFFC00D04);
		d("DMA4_X_COUNT", 16, 0xFFC00D10);
		d("DMA4_X_MODIFY", 16, 0xFFC00D14);
		d("DMA4_Y_COUNT", 16, 0xFFC00D18);
		d("DMA4_Y_MODIFY", 16, 0xFFC00D1C);
		d("DMA5_CONFIG", 16, 0xFFC00D48);
		d("DMA5_CURR_ADDR", 32, 0xFFC00D64);
		d("DMA5_CURR_DESC_PTR", 32, 0xFFC00D60);
		d("DMA5_CURR_X_COUNT", 16, 0xFFC00D70);
		d("DMA5_CURR_Y_COUNT", 16, 0xFFC00D78);
		d("DMA5_IRQ_STATUS", 16, 0xFFC00D68);
		d("DMA5_NEXT_DESC_PTR", 32, 0xFFC00D40);
		d("DMA5_PERIPHERAL_MAP", 16, 0xFFC00D6C);
		d("DMA5_START_ADDR", 32, 0xFFC00D44);
		d("DMA5_X_COUNT", 16, 0xFFC00D50);
		d("DMA5_X_MODIFY", 16, 0xFFC00D54);
		d("DMA5_Y_COUNT", 16, 0xFFC00D58);
		d("DMA5_Y_MODIFY", 16, 0xFFC00D5C);
		d("DMA6_CONFIG", 16, 0xFFC00D88);
		d("DMA6_CURR_ADDR", 32, 0xFFC00DA4);
		d("DMA6_CURR_DESC_PTR", 32, 0xFFC00DA0);
		d("DMA6_CURR_X_COUNT", 16, 0xFFC00DB0);
		d("DMA6_CURR_Y_COUNT", 16, 0xFFC00DB8);
		d("DMA6_IRQ_STATUS", 16, 0xFFC00DA8);
		d("DMA6_NEXT_DESC_PTR", 32, 0xFFC00D80);
		d("DMA6_PERIPHERAL_MAP", 16, 0xFFC00DAC);
		d("DMA6_START_ADDR", 32, 0xFFC00D84);
		d("DMA6_X_COUNT", 16, 0xFFC00D90);
		d("DMA6_X_MODIFY", 16, 0xFFC00D94);
		d("DMA6_Y_COUNT", 16, 0xFFC00D98);
		d("DMA6_Y_MODIFY", 16, 0xFFC00D9C);
		d("DMA7_CONFIG", 16, 0xFFC00DC8);
		d("DMA7_CURR_ADDR", 32, 0xFFC00DE4);
		d("DMA7_CURR_DESC_PTR", 32, 0xFFC00DE0);
		d("DMA7_CURR_X_COUNT", 16, 0xFFC00DF0);
		d("DMA7_CURR_Y_COUNT", 16, 0xFFC00DF8);
		d("DMA7_IRQ_STATUS", 16, 0xFFC00DE8);
		d("DMA7_NEXT_DESC_PTR", 32, 0xFFC00DC0);
		d("DMA7_PERIPHERAL_MAP", 16, 0xFFC00DEC);
		d("DMA7_START_ADDR", 32, 0xFFC00DC4);
		d("DMA7_X_COUNT", 16, 0xFFC00DD0);
		d("DMA7_X_MODIFY", 16, 0xFFC00DD4);
		d("DMA7_Y_COUNT", 16, 0xFFC00DD8);
		d("DMA7_Y_MODIFY", 16, 0xFFC00DDC);
		d("DMA8_CONFIG", 16, 0xFFC00E08);
		d("DMA8_CURR_ADDR", 32, 0xFFC00E24);
		d("DMA8_CURR_DESC_PTR", 32, 0xFFC00E20);
		d("DMA8_CURR_X_COUNT", 16, 0xFFC00E30);
		d("DMA8_CURR_Y_COUNT", 16, 0xFFC00E38);
		d("DMA8_IRQ_STATUS", 16, 0xFFC00E28);
		d("DMA8_NEXT_DESC_PTR", 32, 0xFFC00E00);
		d("DMA8_PERIPHERAL_MAP", 16, 0xFFC00E2C);
		d("DMA8_START_ADDR", 32, 0xFFC00E04);
		d("DMA8_X_COUNT", 16, 0xFFC00E10);
		d("DMA8_X_MODIFY", 16, 0xFFC00E14);
		d("DMA8_Y_COUNT", 16, 0xFFC00E18);
		d("DMA8_Y_MODIFY", 16, 0xFFC00E1C);
		d("DMA9_CONFIG", 16, 0xFFC00E48);
		d("DMA9_CURR_ADDR", 32, 0xFFC00E64);
		d("DMA9_CURR_DESC_PTR", 32, 0xFFC00E60);
		d("DMA9_CURR_X_COUNT", 16, 0xFFC00E70);
		d("DMA9_CURR_Y_COUNT", 16, 0xFFC00E78);
		d("DMA9_IRQ_STATUS", 16, 0xFFC00E68);
		d("DMA9_NEXT_DESC_PTR", 32, 0xFFC00E40);
		d("DMA9_PERIPHERAL_MAP", 16, 0xFFC00E6C);
		d("DMA9_START_ADDR", 32, 0xFFC00E44);
		d("DMA9_X_COUNT", 16, 0xFFC00E50);
		d("DMA9_X_MODIFY", 16, 0xFFC00E54);
		d("DMA9_Y_COUNT", 16, 0xFFC00E58);
		d("DMA9_Y_MODIFY", 16, 0xFFC00E5C);
		d("DMAC0_TCCNT", 16, 0xFFC00B10);
		d("DMAC0_TCPER", 16, 0xFFC00B0C);
		d("DMAC1_PERIMUX", 16, 0xFFC04340);
		d("DMAC1_TCCNT", 16, 0xFFC01B10);
		d("DMAC1_TCPER", 16, 0xFFC01B0C);

		parent = debugfs_create_dir("HMDMA", top);
		d("HMDMA0_BCINIT", 16, 0xFFC04508);
		d("HMDMA0_BCOUNT", 16, 0xFFC04518);
		d("HMDMA0_CONTROL", 16, 0xFFC04500);
		d("HMDMA0_ECINIT", 16, 0xFFC04504);
		d("HMDMA0_ECOUNT", 16, 0xFFC04514);
		d("HMDMA0_ECOVERFLOW", 16, 0xFFC04510);
		d("HMDMA0_ECURGENT", 16, 0xFFC0450C);
		d("HMDMA1_BCINIT", 16, 0xFFC04548);
		d("HMDMA1_BCOUNT", 16, 0xFFC04558);
		d("HMDMA1_CONTROL", 16, 0xFFC04540);
		d("HMDMA1_ECINIT", 16, 0xFFC04544);
		d("HMDMA1_ECOUNT", 16, 0xFFC04554);
		d("HMDMA1_ECOVERFLOW", 16, 0xFFC04550);
		d("HMDMA1_ECURGENT", 16, 0xFFC0454C);

		parent = debugfs_create_dir("MDMA", top);
		d("MDMA_D0_CONFIG", 16, 0xFFC00F08);
		d("MDMA_D0_CURR_ADDR", 32, 0xFFC00F24);
		d("MDMA_D0_CURR_DESC_PTR", 32, 0xFFC00F20);
		d("MDMA_D0_CURR_X_COUNT", 16, 0xFFC00F30);
		d("MDMA_D0_CURR_Y_COUNT", 16, 0xFFC00F38);
		d("MDMA_D0_IRQ_STATUS", 16, 0xFFC00F28);
		d("MDMA_D0_NEXT_DESC_PTR", 32, 0xFFC00F00);
		d("MDMA_D0_PERIPHERAL_MAP", 16, 0xFFC00F2C);
		d("MDMA_D0_START_ADDR", 32, 0xFFC00F04);
		d("MDMA_D0_X_COUNT", 16, 0xFFC00F10);
		d("MDMA_D0_X_MODIFY", 16, 0xFFC00F14);
		d("MDMA_D0_Y_COUNT", 16, 0xFFC00F18);
		d("MDMA_D0_Y_MODIFY", 16, 0xFFC00F1C);
		d("MDMA_D1_CONFIG", 16, 0xFFC00F88);
		d("MDMA_D1_CURR_ADDR", 32, 0xFFC00FA4);
		d("MDMA_D1_CURR_DESC_PTR", 32, 0xFFC00FA0);
		d("MDMA_D1_CURR_X_COUNT", 16, 0xFFC00FB0);
		d("MDMA_D1_CURR_Y_COUNT", 16, 0xFFC00FB8);
		d("MDMA_D1_IRQ_STATUS", 16, 0xFFC00FA8);
		d("MDMA_D1_NEXT_DESC_PTR", 32, 0xFFC00F80);
		d("MDMA_D1_PERIPHERAL_MAP", 16, 0xFFC00FAC);
		d("MDMA_D1_START_ADDR", 32, 0xFFC00F84);
		d("MDMA_D1_X_COUNT", 16, 0xFFC00F90);
		d("MDMA_D1_X_MODIFY", 16, 0xFFC00F94);
		d("MDMA_D1_Y_COUNT", 16, 0xFFC00F98);
		d("MDMA_D1_Y_MODIFY", 16, 0xFFC00F9C);
		d("MDMA_D2_CONFIG", 16, 0xFFC01F08);
		d("MDMA_D2_CURR_ADDR", 32, 0xFFC01F24);
		d("MDMA_D2_CURR_DESC_PTR", 32, 0xFFC01F20);
		d("MDMA_D2_CURR_X_COUNT", 16, 0xFFC01F30);
		d("MDMA_D2_CURR_Y_COUNT", 16, 0xFFC01F38);
		d("MDMA_D2_IRQ_STATUS", 16, 0xFFC01F28);
		d("MDMA_D2_NEXT_DESC_PTR", 32, 0xFFC01F00);
		d("MDMA_D2_PERIPHERAL_MAP", 16, 0xFFC01F2C);
		d("MDMA_D2_START_ADDR", 32, 0xFFC01F04);
		d("MDMA_D2_X_COUNT", 16, 0xFFC01F10);
		d("MDMA_D2_X_MODIFY", 16, 0xFFC01F14);
		d("MDMA_D2_Y_COUNT", 16, 0xFFC01F18);
		d("MDMA_D2_Y_MODIFY", 16, 0xFFC01F1C);
		d("MDMA_D3_CONFIG", 16, 0xFFC01F88);
		d("MDMA_D3_CURR_ADDR", 32, 0xFFC01FA4);
		d("MDMA_D3_CURR_DESC_PTR", 32, 0xFFC01FA0);
		d("MDMA_D3_CURR_X_COUNT", 16, 0xFFC01FB0);
		d("MDMA_D3_CURR_Y_COUNT", 16, 0xFFC01FB8);
		d("MDMA_D3_IRQ_STATUS", 16, 0xFFC01FA8);
		d("MDMA_D3_NEXT_DESC_PTR", 32, 0xFFC01F80);
		d("MDMA_D3_PERIPHERAL_MAP", 16, 0xFFC01FAC);
		d("MDMA_D3_START_ADDR", 32, 0xFFC01F84);
		d("MDMA_D3_X_COUNT", 16, 0xFFC01F90);
		d("MDMA_D3_X_MODIFY", 16, 0xFFC01F94);
		d("MDMA_D3_Y_COUNT", 16, 0xFFC01F98);
		d("MDMA_D3_Y_MODIFY", 16, 0xFFC01F9C);
		d("MDMA_S0_CONFIG", 16, 0xFFC00F48);
		d("MDMA_S0_CURR_ADDR", 32, 0xFFC00F64);
		d("MDMA_S0_CURR_DESC_PTR", 32, 0xFFC00F60);
		d("MDMA_S0_CURR_X_COUNT", 16, 0xFFC00F70);
		d("MDMA_S0_CURR_Y_COUNT", 16, 0xFFC00F78);
		d("MDMA_S0_IRQ_STATUS", 16, 0xFFC00F68);
		d("MDMA_S0_NEXT_DESC_PTR", 32, 0xFFC00F40);
		d("MDMA_S0_PERIPHERAL_MAP", 16, 0xFFC00F6C);
		d("MDMA_S0_START_ADDR", 32, 0xFFC00F44);
		d("MDMA_S0_X_COUNT", 16, 0xFFC00F50);
		d("MDMA_S0_X_MODIFY", 16, 0xFFC00F54);
		d("MDMA_S0_Y_COUNT", 16, 0xFFC00F58);
		d("MDMA_S0_Y_MODIFY", 16, 0xFFC00F5C);
		d("MDMA_S1_CONFIG", 16, 0xFFC00FC8);
		d("MDMA_S1_CURR_ADDR", 32, 0xFFC00FE4);
		d("MDMA_S1_CURR_DESC_PTR", 32, 0xFFC00FE0);
		d("MDMA_S1_CURR_X_COUNT", 16, 0xFFC00FF0);
		d("MDMA_S1_CURR_Y_COUNT", 16, 0xFFC00FF8);
		d("MDMA_S1_IRQ_STATUS", 16, 0xFFC00FE8);
		d("MDMA_S1_NEXT_DESC_PTR", 32, 0xFFC00FC0);
		d("MDMA_S1_PERIPHERAL_MAP", 16, 0xFFC00FEC);
		d("MDMA_S1_START_ADDR", 32, 0xFFC00FC4);
		d("MDMA_S1_X_COUNT", 16, 0xFFC00FD0);
		d("MDMA_S1_X_MODIFY", 16, 0xFFC00FD4);
		d("MDMA_S1_Y_COUNT", 16, 0xFFC00FD8);
		d("MDMA_S1_Y_MODIFY", 16, 0xFFC00FDC);
		d("MDMA_S2_CONFIG", 16, 0xFFC01F48);
		d("MDMA_S2_CURR_ADDR", 32, 0xFFC01F64);
		d("MDMA_S2_CURR_DESC_PTR", 32, 0xFFC01F60);
		d("MDMA_S2_CURR_X_COUNT", 16, 0xFFC01F70);
		d("MDMA_S2_CURR_Y_COUNT", 16, 0xFFC01F78);
		d("MDMA_S2_IRQ_STATUS", 16, 0xFFC01F68);
		d("MDMA_S2_NEXT_DESC_PTR", 32, 0xFFC01F40);
		d("MDMA_S2_PERIPHERAL_MAP", 16, 0xFFC01F6C);
		d("MDMA_S2_START_ADDR", 32, 0xFFC01F44);
		d("MDMA_S2_X_COUNT", 16, 0xFFC01F50);
		d("MDMA_S2_X_MODIFY", 16, 0xFFC01F54);
		d("MDMA_S2_Y_COUNT", 16, 0xFFC01F58);
		d("MDMA_S2_Y_MODIFY", 16, 0xFFC01F5C);
		d("MDMA_S3_CONFIG", 16, 0xFFC01FC8);
		d("MDMA_S3_CURR_ADDR", 32, 0xFFC01FE4);
		d("MDMA_S3_CURR_DESC_PTR", 32, 0xFFC01FE0);
		d("MDMA_S3_CURR_X_COUNT", 16, 0xFFC01FF0);
		d("MDMA_S3_CURR_Y_COUNT", 16, 0xFFC01FF8);
		d("MDMA_S3_IRQ_STATUS", 16, 0xFFC01FE8);
		d("MDMA_S3_NEXT_DESC_PTR", 32, 0xFFC01FC0);
		d("MDMA_S3_PERIPHERAL_MAP", 16, 0xFFC01FEC);
		d("MDMA_S3_START_ADDR", 32, 0xFFC01FC4);
		d("MDMA_S3_X_COUNT", 16, 0xFFC01FD0);
		d("MDMA_S3_X_MODIFY", 16, 0xFFC01FD4);
		d("MDMA_S3_Y_COUNT", 16, 0xFFC01FD8);
		d("MDMA_S3_Y_MODIFY", 16, 0xFFC01FDC);

		parent = debugfs_create_dir("PINT_0", top);
		d("PINT0_ASSIGN", 32, 0xFFC0140C);
		d("PINT0_EDGE_CLEAR", 32, 0xFFC01414);
		d("PINT0_EDGE_SET", 32, 0xFFC01410);
		d("PINT0_INVERT_CLEAR", 32, 0xFFC0141C);
		d("PINT0_INVERT_SET", 32, 0xFFC01418);
		d("PINT0_IRQ", 32, 0xFFC01408);
		d("PINT0_LATCH", 32, 0xFFC01424);
		d("PINT0_MASK_CLEAR", 32, 0xFFC01404);
		d("PINT0_MASK_SET", 32, 0xFFC01400);
		d("PINT0_PINSTATE", 32, 0xFFC01420);

		parent = debugfs_create_dir("PINT_1", top);
		d("PINT1_ASSIGN", 32, 0xFFC0143C);
		d("PINT1_EDGE_CLEAR", 32, 0xFFC01444);
		d("PINT1_EDGE_SET", 32, 0xFFC01440);
		d("PINT1_INVERT_CLEAR", 32, 0xFFC0144C);
		d("PINT1_INVERT_SET", 32, 0xFFC01448);
		d("PINT1_IRQ", 32, 0xFFC01438);
		d("PINT1_LATCH", 32, 0xFFC01454);
		d("PINT1_MASK_CLEAR", 32, 0xFFC01434);
		d("PINT1_MASK_SET", 32, 0xFFC01430);
		d("PINT1_PINSTATE", 32, 0xFFC01450);

		parent = debugfs_create_dir("PINT_2", top);
		d("PINT2_ASSIGN", 32, 0xFFC0146C);
		d("PINT2_EDGE_CLEAR", 32, 0xFFC01474);
		d("PINT2_EDGE_SET", 32, 0xFFC01470);
		d("PINT2_INVERT_CLEAR", 32, 0xFFC0147C);
		d("PINT2_INVERT_SET", 32, 0xFFC01478);
		d("PINT2_IRQ", 32, 0xFFC01468);
		d("PINT2_LATCH", 32, 0xFFC01484);
		d("PINT2_MASK_CLEAR", 32, 0xFFC01464);
		d("PINT2_MASK_SET", 32, 0xFFC01460);
		d("PINT2_PINSTATE", 32, 0xFFC01480);

		parent = debugfs_create_dir("PINT_3", top);
		d("PINT3_ASSIGN", 32, 0xFFC0149C);
		d("PINT3_EDGE_CLEAR", 32, 0xFFC014A4);
		d("PINT3_EDGE_SET", 32, 0xFFC014A0);
		d("PINT3_INVERT_CLEAR", 32, 0xFFC014AC);
		d("PINT3_INVERT_SET", 32, 0xFFC014A8);
		d("PINT3_IRQ", 32, 0xFFC01498);
		d("PINT3_LATCH", 32, 0xFFC014B4);
		d("PINT3_MASK_CLEAR", 32, 0xFFC01494);
		d("PINT3_MASK_SET", 32, 0xFFC01490);
		d("PINT3_PINSTATE", 32, 0xFFC014B0);

		parent = debugfs_create_dir("Port_A", top);
		d("PORTA", 16, 0xFFC014C4);
		d("PORTA_CLEAR", 16, 0xFFC014CC);
		d("PORTA_DIR_CLEAR", 16, 0xFFC014D4);
		d("PORTA_DIR_SET", 16, 0xFFC014D0);
		d("PORTA_FER", 16, 0xFFC014C0);
		d("PORTA_INEN", 16, 0xFFC014D8);
		d("PORTA_MUX", 32, 0xFFC014DC);
		d("PORTA_SET", 16, 0xFFC014C8);

		parent = debugfs_create_dir("Port_B", top);
		d("PORTB", 16, 0xFFC014E4);
		d("PORTB_CLEAR", 16, 0xFFC014EC);
		d("PORTB_DIR_CLEAR", 16, 0xFFC014F4);
		d("PORTB_DIR_SET", 16, 0xFFC014F0);
		d("PORTB_FER", 16, 0xFFC014E0);
		d("PORTB_INEN", 16, 0xFFC014F8);
		d("PORTB_MUX", 32, 0xFFC014FC);
		d("PORTB_SET", 16, 0xFFC014E8);

		parent = debugfs_create_dir("Port_C", top);
		d("PORTC", 16, 0xFFC01504);
		d("PORTC_CLEAR", 16, 0xFFC0150C);
		d("PORTC_DIR_CLEAR", 16, 0xFFC01514);
		d("PORTC_DIR_SET", 16, 0xFFC01510);
		d("PORTC_FER", 16, 0xFFC01500);
		d("PORTC_INEN", 16, 0xFFC01518);
		d("PORTC_MUX", 32, 0xFFC0151C);
		d("PORTC_SET", 16, 0xFFC01508);

		parent = debugfs_create_dir("Port_D", top);
		d("PORTD", 16, 0xFFC01524);
		d("PORTD_CLEAR", 16, 0xFFC0152C);
		d("PORTD_DIR_CLEAR", 16, 0xFFC01534);
		d("PORTD_DIR_SET", 16, 0xFFC01530);
		d("PORTD_FER", 16, 0xFFC01520);
		d("PORTD_INEN", 16, 0xFFC01538);
		d("PORTD_MUX", 32, 0xFFC0153C);
		d("PORTD_SET", 16, 0xFFC01528);

		parent = debugfs_create_dir("Port_E", top);
		d("PORTE", 16, 0xFFC01544);
		d("PORTE_CLEAR", 16, 0xFFC0154C);
		d("PORTE_DIR_CLEAR", 16, 0xFFC01554);
		d("PORTE_DIR_SET", 16, 0xFFC01550);
		d("PORTE_FER", 16, 0xFFC01540);
		d("PORTE_INEN", 16, 0xFFC01558);
		d("PORTE_MUX", 32, 0xFFC0155C);
		d("PORTE_SET", 16, 0xFFC01548);

		parent = debugfs_create_dir("Port_F", top);
		d("PORTF", 16, 0xFFC01564);
		d("PORTF_CLEAR", 16, 0xFFC0156C);
		d("PORTF_DIR_CLEAR", 16, 0xFFC01574);
		d("PORTF_DIR_SET", 16, 0xFFC01570);
		d("PORTF_FER", 16, 0xFFC01560);
		d("PORTF_INEN", 16, 0xFFC01578);
		d("PORTF_MUX", 32, 0xFFC0157C);
		d("PORTF_SET", 16, 0xFFC01568);

		parent = debugfs_create_dir("Port_G", top);
		d("PORTG", 16, 0xFFC01584);
		d("PORTG_CLEAR", 16, 0xFFC0158C);
		d("PORTG_DIR_CLEAR", 16, 0xFFC01594);
		d("PORTG_DIR_SET", 16, 0xFFC01590);
		d("PORTG_FER", 16, 0xFFC01580);
		d("PORTG_INEN", 16, 0xFFC01598);
		d("PORTG_MUX", 32, 0xFFC0159C);
		d("PORTG_SET", 16, 0xFFC01588);

		parent = debugfs_create_dir("Port_H", top);
		d("PORTH", 16, 0xFFC015A4);
		d("PORTH_CLEAR", 16, 0xFFC015AC);
		d("PORTH_DIR_CLEAR", 16, 0xFFC015B4);
		d("PORTH_DIR_SET", 16, 0xFFC015B0);
		d("PORTH_FER", 16, 0xFFC015A0);
		d("PORTH_INEN", 16, 0xFFC015B8);
		d("PORTH_MUX", 32, 0xFFC015BC);
		d("PORTH_SET", 16, 0xFFC015A8);

		parent = debugfs_create_dir("Port_I", top);
		d("PORTI", 16, 0xFFC015C4);
		d("PORTI_CLEAR", 16, 0xFFC015CC);
		d("PORTI_DIR_CLEAR", 16, 0xFFC015D4);
		d("PORTI_DIR_SET", 16, 0xFFC015D0);
		d("PORTI_FER", 16, 0xFFC015C0);
		d("PORTI_INEN", 16, 0xFFC015D8);
		d("PORTI_MUX", 32, 0xFFC015DC);
		d("PORTI_SET", 16, 0xFFC015C8);

		parent = debugfs_create_dir("Port_J", top);
		d("PORTJ", 16, 0xFFC015E4);
		d("PORTJ_CLEAR", 16, 0xFFC015EC);
		d("PORTJ_DIR_CLEAR", 16, 0xFFC015F4);
		d("PORTJ_DIR_SET", 16, 0xFFC015F0);
		d("PORTJ_FER", 16, 0xFFC015E0);
		d("PORTJ_INEN", 16, 0xFFC015F8);
		d("PORTJ_MUX", 32, 0xFFC015FC);
		d("PORTJ_SET", 16, 0xFFC015E8);

		parent = debugfs_create_dir("SPI", top);
		d("SPI0_BAUD", 16, 0xFFC00514);
		d("SPI0_CTL", 16, 0xFFC00500);
		d("SPI0_FLG", 16, 0xFFC00504);
		d("SPI0_RDBR", 16, 0xFFC00510);
		d("SPI0_SHADOW", 16, 0xFFC00518);
		d("SPI0_STAT", 16, 0xFFC00508);
		d("SPI0_TDBR", 16, 0xFFC0050C);
		d("SPI1_BAUD", 16, 0xFFC02314);
		d("SPI1_CTL", 16, 0xFFC02300);
		d("SPI1_FLG", 16, 0xFFC02304);
		d("SPI1_RDBR", 16, 0xFFC02310);
		d("SPI1_SHADOW", 16, 0xFFC02318);
		d("SPI1_STAT", 16, 0xFFC02308);
		d("SPI1_TDBR", 16, 0xFFC0230C);

		parent = debugfs_create_dir("TWI", top);
		d("TWI0_CLKDIV", 16, 0xFFC00700);
		d("TWI0_CONTROL", 16, 0xFFC00704);
		d("TWI0_FIFO_CTL", 16, 0xFFC00728);
		d("TWI0_FIFO_STAT", 16, 0xFFC0072C);
		d("TWI0_INT_MASK", 16, 0xFFC00724);
		d("TWI0_INT_STAT", 16, 0xFFC00720);
		d("TWI0_MASTER_ADDR", 16, 0xFFC0071C);
		d("TWI0_MASTER_CTL", 16, 0xFFC00714);
		d("TWI0_MASTER_STAT", 16, 0xFFC00718);
		d("TWI0_RCV_DATA16", 16, 0xFFC0078C);
		d("TWI0_RCV_DATA8", 16, 0xFFC00788);
		d("TWI0_SLAVE_ADDR", 16, 0xFFC00710);
		d("TWI0_SLAVE_CTL", 16, 0xFFC00708);
		d("TWI0_SLAVE_STAT", 16, 0xFFC0070C);
		d("TWI0_XMT_DATA16", 16, 0xFFC00784);
		d("TWI0_XMT_DATA8", 16, 0xFFC00780);
		d("TWI1_CLKDIV", 16, 0xFFC02200);
		d("TWI1_CONTROL", 16, 0xFFC02204);
		d("TWI1_FIFO_CTL", 16, 0xFFC02228);
		d("TWI1_FIFO_STAT", 16, 0xFFC0222C);
		d("TWI1_INT_MASK", 16, 0xFFC02224);
		d("TWI1_INT_STAT", 16, 0xFFC02220);
		d("TWI1_MASTER_ADDR", 16, 0xFFC0221C);
		d("TWI1_MASTER_CTL", 16, 0xFFC02214);
		d("TWI1_MASTER_STAT", 16, 0xFFC02218);
		d("TWI1_RCV_DATA16", 16, 0xFFC0228C);
		d("TWI1_RCV_DATA8", 16, 0xFFC02288);
		d("TWI1_SLAVE_ADDR", 16, 0xFFC02210);
		d("TWI1_SLAVE_CTL", 16, 0xFFC02208);
		d("TWI1_SLAVE_STAT", 16, 0xFFC0220C);
		d("TWI1_XMT_DATA16", 16, 0xFFC02284);
		d("TWI1_XMT_DATA8", 16, 0xFFC02280);

		parent = debugfs_create_dir("UART", top);
		d("UART0_DLH", 16, 0xFFC00404);
		d("UART0_DLL", 16, 0xFFC00400);
		d("UART0_GCTL", 16, 0xFFC00408);
		d("UART0_IER_CLEAR", 16, 0xFFC00424);
		d("UART0_IER_SET", 16, 0xFFC00420);
		d("UART0_LCR", 16, 0xFFC0040C);
		d("UART0_LSR", 16, 0xFFC00414);
		d("UART0_MCR", 16, 0xFFC00410);
		d("UART0_MSR", 16, 0xFFC00418);
		d("UART0_RBR", 16, 0xFFC0042C);
		d("UART0_SCR", 16, 0xFFC0041C);
		d("UART0_THR", 16, 0xFFC00428);
		d("UART1_DLH", 16, 0xFFC02004);
		d("UART1_DLL", 16, 0xFFC02000);
		d("UART1_GCTL", 16, 0xFFC02008);
		d("UART1_IER_CLEAR", 16, 0xFFC02024);
		d("UART1_IER_SET", 16, 0xFFC02020);
		d("UART1_LCR", 16, 0xFFC0200C);
		d("UART1_LSR", 16, 0xFFC02014);
		d("UART1_MCR", 16, 0xFFC02010);
		d("UART1_MSR", 16, 0xFFC02018);
		d("UART1_RBR", 16, 0xFFC0202C);
		d("UART1_SCR", 16, 0xFFC0201C);
		d("UART1_THR", 16, 0xFFC02028);
		d("UART3_DLH", 16, 0xFFC03104);
		d("UART3_DLL", 16, 0xFFC03100);
		d("UART3_GCTL", 16, 0xFFC03108);
		d("UART3_IER_CLEAR", 16, 0xFFC03124);
		d("UART3_IER_SET", 16, 0xFFC03120);
		d("UART3_LCR", 16, 0xFFC0310C);
		d("UART3_LSR", 16, 0xFFC03114);
		d("UART3_MCR", 16, 0xFFC03110);
		d("UART3_MSR", 16, 0xFFC03118);
		d("UART3_RBR", 16, 0xFFC0312C);
		d("UART3_SCR", 16, 0xFFC0311C);
		d("UART3_THR", 16, 0xFFC03128);

	}	/* BF544 */

#ifdef __ADSPBF547__
# define USE_BF547 1
#else
# define USE_BF547 0
#endif
#ifdef __ADSPBF548__
# define USE_BF548 1
#else
# define USE_BF548 0
#endif
#ifdef __ADSPBF549__
# define USE_BF549 1
#else
# define USE_BF549 0
#endif
	if (USE_BF547 || USE_BF548 || USE_BF549) {

		parent = debugfs_create_dir("DMA", top);
		d("DMA0_CONFIG", 16, 0xFFC00C08);
		d("DMA0_CURR_ADDR", 32, 0xFFC00C24);
		d("DMA0_CURR_DESC_PTR", 32, 0xFFC00C20);
		d("DMA0_CURR_X_COUNT", 16, 0xFFC00C30);
		d("DMA0_CURR_Y_COUNT", 16, 0xFFC00C38);
		d("DMA0_IRQ_STATUS", 16, 0xFFC00C28);
		d("DMA0_NEXT_DESC_PTR", 32, 0xFFC00C00);
		d("DMA0_PERIPHERAL_MAP", 16, 0xFFC00C2C);
		d("DMA0_START_ADDR", 32, 0xFFC00C04);
		d("DMA0_X_COUNT", 16, 0xFFC00C10);
		d("DMA0_X_MODIFY", 16, 0xFFC00C14);
		d("DMA0_Y_COUNT", 16, 0xFFC00C18);
		d("DMA0_Y_MODIFY", 16, 0xFFC00C1C);
		d("DMA10_CONFIG", 16, 0xFFC00E88);
		d("DMA10_CURR_ADDR", 32, 0xFFC00EA4);
		d("DMA10_CURR_DESC_PTR", 32, 0xFFC00EA0);
		d("DMA10_CURR_X_COUNT", 16, 0xFFC00EB0);
		d("DMA10_CURR_Y_COUNT", 16, 0xFFC00EB8);
		d("DMA10_IRQ_STATUS", 16, 0xFFC00EA8);
		d("DMA10_NEXT_DESC_PTR", 32, 0xFFC00E80);
		d("DMA10_PERIPHERAL_MAP", 16, 0xFFC00EAC);
		d("DMA10_START_ADDR", 32, 0xFFC00E84);
		d("DMA10_X_COUNT", 16, 0xFFC00E90);
		d("DMA10_X_MODIFY", 16, 0xFFC00E94);
		d("DMA10_Y_COUNT", 16, 0xFFC00E98);
		d("DMA10_Y_MODIFY", 16, 0xFFC00E9C);
		d("DMA11_CONFIG", 16, 0xFFC00EC8);
		d("DMA11_CURR_ADDR", 32, 0xFFC00EE4);
		d("DMA11_CURR_DESC_PTR", 32, 0xFFC00EE0);
		d("DMA11_CURR_X_COUNT", 16, 0xFFC00EF0);
		d("DMA11_CURR_Y_COUNT", 16, 0xFFC00EF8);
		d("DMA11_IRQ_STATUS", 16, 0xFFC00EE8);
		d("DMA11_NEXT_DESC_PTR", 32, 0xFFC00EC0);
		d("DMA11_PERIPHERAL_MAP", 16, 0xFFC00EEC);
		d("DMA11_START_ADDR", 32, 0xFFC00EC4);
		d("DMA11_X_COUNT", 16, 0xFFC00ED0);
		d("DMA11_X_MODIFY", 16, 0xFFC00ED4);
		d("DMA11_Y_COUNT", 16, 0xFFC00ED8);
		d("DMA11_Y_MODIFY", 16, 0xFFC00EDC);
		d("DMA12_CONFIG", 16, 0xFFC01C08);
		d("DMA12_CURR_ADDR", 32, 0xFFC01C24);
		d("DMA12_CURR_DESC_PTR", 32, 0xFFC01C20);
		d("DMA12_CURR_X_COUNT", 16, 0xFFC01C30);
		d("DMA12_CURR_Y_COUNT", 16, 0xFFC01C38);
		d("DMA12_IRQ_STATUS", 16, 0xFFC01C28);
		d("DMA12_NEXT_DESC_PTR", 32, 0xFFC01C00);
		d("DMA12_PERIPHERAL_MAP", 16, 0xFFC01C2C);
		d("DMA12_START_ADDR", 32, 0xFFC01C04);
		d("DMA12_X_COUNT", 16, 0xFFC01C10);
		d("DMA12_X_MODIFY", 16, 0xFFC01C14);
		d("DMA12_Y_COUNT", 16, 0xFFC01C18);
		d("DMA12_Y_MODIFY", 16, 0xFFC01C1C);
		d("DMA13_CONFIG", 16, 0xFFC01C48);
		d("DMA13_CURR_ADDR", 32, 0xFFC01C64);
		d("DMA13_CURR_DESC_PTR", 32, 0xFFC01C60);
		d("DMA13_CURR_X_COUNT", 16, 0xFFC01C70);
		d("DMA13_CURR_Y_COUNT", 16, 0xFFC01C78);
		d("DMA13_IRQ_STATUS", 16, 0xFFC01C68);
		d("DMA13_NEXT_DESC_PTR", 32, 0xFFC01C40);
		d("DMA13_PERIPHERAL_MAP", 16, 0xFFC01C6C);
		d("DMA13_START_ADDR", 32, 0xFFC01C44);
		d("DMA13_X_COUNT", 16, 0xFFC01C50);
		d("DMA13_X_MODIFY", 16, 0xFFC01C54);
		d("DMA13_Y_COUNT", 16, 0xFFC01C58);
		d("DMA13_Y_MODIFY", 16, 0xFFC01C5C);
		d("DMA14_CONFIG", 16, 0xFFC01C88);
		d("DMA14_CURR_ADDR", 32, 0xFFC01CA4);
		d("DMA14_CURR_DESC_PTR", 32, 0xFFC01CA0);
		d("DMA14_CURR_X_COUNT", 16, 0xFFC01CB0);
		d("DMA14_CURR_Y_COUNT", 16, 0xFFC01CB8);
		d("DMA14_IRQ_STATUS", 16, 0xFFC01CA8);
		d("DMA14_NEXT_DESC_PTR", 32, 0xFFC01C80);
		d("DMA14_PERIPHERAL_MAP", 16, 0xFFC01CAC);
		d("DMA14_START_ADDR", 32, 0xFFC01C84);
		d("DMA14_X_COUNT", 16, 0xFFC01C90);
		d("DMA14_X_MODIFY", 16, 0xFFC01C94);
		d("DMA14_Y_COUNT", 16, 0xFFC01C98);
		d("DMA14_Y_MODIFY", 16, 0xFFC01C9C);
		d("DMA15_CONFIG", 16, 0xFFC01CC8);
		d("DMA15_CURR_ADDR", 32, 0xFFC01CE4);
		d("DMA15_CURR_DESC_PTR", 32, 0xFFC01CE0);
		d("DMA15_CURR_X_COUNT", 16, 0xFFC01CF0);
		d("DMA15_CURR_Y_COUNT", 16, 0xFFC01CF8);
		d("DMA15_IRQ_STATUS", 16, 0xFFC01CE8);
		d("DMA15_NEXT_DESC_PTR", 32, 0xFFC01CC0);
		d("DMA15_PERIPHERAL_MAP", 16, 0xFFC01CEC);
		d("DMA15_START_ADDR", 32, 0xFFC01CC4);
		d("DMA15_X_COUNT", 16, 0xFFC01CD0);
		d("DMA15_X_MODIFY", 16, 0xFFC01CD4);
		d("DMA15_Y_COUNT", 16, 0xFFC01CD8);
		d("DMA15_Y_MODIFY", 16, 0xFFC01CDC);
		d("DMA16_CONFIG", 16, 0xFFC01D08);
		d("DMA16_CURR_ADDR", 32, 0xFFC01D24);
		d("DMA16_CURR_DESC_PTR", 32, 0xFFC01D20);
		d("DMA16_CURR_X_COUNT", 16, 0xFFC01D30);
		d("DMA16_CURR_Y_COUNT", 16, 0xFFC01D38);
		d("DMA16_IRQ_STATUS", 16, 0xFFC01D28);
		d("DMA16_NEXT_DESC_PTR", 32, 0xFFC01D00);
		d("DMA16_PERIPHERAL_MAP", 16, 0xFFC01D2C);
		d("DMA16_START_ADDR", 32, 0xFFC01D04);
		d("DMA16_X_COUNT", 16, 0xFFC01D10);
		d("DMA16_X_MODIFY", 16, 0xFFC01D14);
		d("DMA16_Y_COUNT", 16, 0xFFC01D18);
		d("DMA16_Y_MODIFY", 16, 0xFFC01D1C);
		d("DMA17_CONFIG", 16, 0xFFC01D48);
		d("DMA17_CURR_ADDR", 32, 0xFFC01D64);
		d("DMA17_CURR_DESC_PTR", 32, 0xFFC01D60);
		d("DMA17_CURR_X_COUNT", 16, 0xFFC01D70);
		d("DMA17_CURR_Y_COUNT", 16, 0xFFC01D78);
		d("DMA17_IRQ_STATUS", 16, 0xFFC01D68);
		d("DMA17_NEXT_DESC_PTR", 32, 0xFFC01D40);
		d("DMA17_PERIPHERAL_MAP", 16, 0xFFC01D6C);
		d("DMA17_START_ADDR", 32, 0xFFC01D44);
		d("DMA17_X_COUNT", 16, 0xFFC01D50);
		d("DMA17_X_MODIFY", 16, 0xFFC01D54);
		d("DMA17_Y_COUNT", 16, 0xFFC01D58);
		d("DMA17_Y_MODIFY", 16, 0xFFC01D5C);
		d("DMA18_CONFIG", 16, 0xFFC01D88);
		d("DMA18_CURR_ADDR", 32, 0xFFC01DA4);
		d("DMA18_CURR_DESC_PTR", 32, 0xFFC01DA0);
		d("DMA18_CURR_X_COUNT", 16, 0xFFC01DB0);
		d("DMA18_CURR_Y_COUNT", 16, 0xFFC01DB8);
		d("DMA18_IRQ_STATUS", 16, 0xFFC01DA8);
		d("DMA18_NEXT_DESC_PTR", 32, 0xFFC01D80);
		d("DMA18_PERIPHERAL_MAP", 16, 0xFFC01DAC);
		d("DMA18_START_ADDR", 32, 0xFFC01D84);
		d("DMA18_X_COUNT", 16, 0xFFC01D90);
		d("DMA18_X_MODIFY", 16, 0xFFC01D94);
		d("DMA18_Y_COUNT", 16, 0xFFC01D98);
		d("DMA18_Y_MODIFY", 16, 0xFFC01D9C);
		d("DMA19_CONFIG", 16, 0xFFC01DC8);
		d("DMA19_CURR_ADDR", 32, 0xFFC01DE4);
		d("DMA19_CURR_DESC_PTR", 32, 0xFFC01DE0);
		d("DMA19_CURR_X_COUNT", 16, 0xFFC01DF0);
		d("DMA19_CURR_Y_COUNT", 16, 0xFFC01DF8);
		d("DMA19_IRQ_STATUS", 16, 0xFFC01DE8);
		d("DMA19_NEXT_DESC_PTR", 32, 0xFFC01DC0);
		d("DMA19_PERIPHERAL_MAP", 16, 0xFFC01DEC);
		d("DMA19_START_ADDR", 32, 0xFFC01DC4);
		d("DMA19_X_COUNT", 16, 0xFFC01DD0);
		d("DMA19_X_MODIFY", 16, 0xFFC01DD4);
		d("DMA19_Y_COUNT", 16, 0xFFC01DD8);
		d("DMA19_Y_MODIFY", 16, 0xFFC01DDC);
		d("DMA1_CONFIG", 16, 0xFFC00C48);
		d("DMA1_CURR_ADDR", 32, 0xFFC00C64);
		d("DMA1_CURR_DESC_PTR", 32, 0xFFC00C60);
		d("DMA1_CURR_X_COUNT", 16, 0xFFC00C70);
		d("DMA1_CURR_Y_COUNT", 16, 0xFFC00C78);
		d("DMA1_IRQ_STATUS", 16, 0xFFC00C68);
		d("DMA1_NEXT_DESC_PTR", 32, 0xFFC00C40);
		d("DMA1_PERIPHERAL_MAP", 16, 0xFFC00C6C);
		d("DMA1_START_ADDR", 32, 0xFFC00C44);
		d("DMA1_X_COUNT", 16, 0xFFC00C50);
		d("DMA1_X_MODIFY", 16, 0xFFC00C54);
		d("DMA1_Y_COUNT", 16, 0xFFC00C58);
		d("DMA1_Y_MODIFY", 16, 0xFFC00C5C);
		d("DMA20_CONFIG", 16, 0xFFC01E08);
		d("DMA20_CURR_ADDR", 32, 0xFFC01E24);
		d("DMA20_CURR_DESC_PTR", 32, 0xFFC01E20);
		d("DMA20_CURR_X_COUNT", 16, 0xFFC01E30);
		d("DMA20_CURR_Y_COUNT", 16, 0xFFC01E38);
		d("DMA20_IRQ_STATUS", 16, 0xFFC01E28);
		d("DMA20_NEXT_DESC_PTR", 32, 0xFFC01E00);
		d("DMA20_PERIPHERAL_MAP", 16, 0xFFC01E2C);
		d("DMA20_START_ADDR", 32, 0xFFC01E04);
		d("DMA20_X_COUNT", 16, 0xFFC01E10);
		d("DMA20_X_MODIFY", 16, 0xFFC01E14);
		d("DMA20_Y_COUNT", 16, 0xFFC01E18);
		d("DMA20_Y_MODIFY", 16, 0xFFC01E1C);
		d("DMA21_CONFIG", 16, 0xFFC01E48);
		d("DMA21_CURR_ADDR", 32, 0xFFC01E64);
		d("DMA21_CURR_DESC_PTR", 32, 0xFFC01E60);
		d("DMA21_CURR_X_COUNT", 16, 0xFFC01E70);
		d("DMA21_CURR_Y_COUNT", 16, 0xFFC01E78);
		d("DMA21_IRQ_STATUS", 16, 0xFFC01E68);
		d("DMA21_NEXT_DESC_PTR", 32, 0xFFC01E40);
		d("DMA21_PERIPHERAL_MAP", 16, 0xFFC01E6C);
		d("DMA21_START_ADDR", 32, 0xFFC01E44);
		d("DMA21_X_COUNT", 16, 0xFFC01E50);
		d("DMA21_X_MODIFY", 16, 0xFFC01E54);
		d("DMA21_Y_COUNT", 16, 0xFFC01E58);
		d("DMA21_Y_MODIFY", 16, 0xFFC01E5C);
		d("DMA22_CONFIG", 16, 0xFFC01E88);
		d("DMA22_CURR_ADDR", 32, 0xFFC01EA4);
		d("DMA22_CURR_DESC_PTR", 32, 0xFFC01EA0);
		d("DMA22_CURR_X_COUNT", 16, 0xFFC01EB0);
		d("DMA22_CURR_Y_COUNT", 16, 0xFFC01EB8);
		d("DMA22_IRQ_STATUS", 16, 0xFFC01EA8);
		d("DMA22_NEXT_DESC_PTR", 32, 0xFFC01E80);
		d("DMA22_PERIPHERAL_MAP", 16, 0xFFC01EAC);
		d("DMA22_START_ADDR", 32, 0xFFC01E84);
		d("DMA22_X_COUNT", 16, 0xFFC01E90);
		d("DMA22_X_MODIFY", 16, 0xFFC01E94);
		d("DMA22_Y_COUNT", 16, 0xFFC01E98);
		d("DMA22_Y_MODIFY", 16, 0xFFC01E9C);
		d("DMA23_CONFIG", 16, 0xFFC01EC8);
		d("DMA23_CURR_ADDR", 32, 0xFFC01EE4);
		d("DMA23_CURR_DESC_PTR", 32, 0xFFC01EE0);
		d("DMA23_CURR_X_COUNT", 16, 0xFFC01EF0);
		d("DMA23_CURR_Y_COUNT", 16, 0xFFC01EF8);
		d("DMA23_IRQ_STATUS", 16, 0xFFC01EE8);
		d("DMA23_NEXT_DESC_PTR", 32, 0xFFC01EC0);
		d("DMA23_PERIPHERAL_MAP", 16, 0xFFC01EEC);
		d("DMA23_START_ADDR", 32, 0xFFC01EC4);
		d("DMA23_X_COUNT", 16, 0xFFC01ED0);
		d("DMA23_X_MODIFY", 16, 0xFFC01ED4);
		d("DMA23_Y_COUNT", 16, 0xFFC01ED8);
		d("DMA23_Y_MODIFY", 16, 0xFFC01EDC);
		d("DMA2_CONFIG", 16, 0xFFC00C88);
		d("DMA2_CURR_ADDR", 32, 0xFFC00CA4);
		d("DMA2_CURR_DESC_PTR", 32, 0xFFC00CA0);
		d("DMA2_CURR_X_COUNT", 16, 0xFFC00CB0);
		d("DMA2_CURR_Y_COUNT", 16, 0xFFC00CB8);
		d("DMA2_IRQ_STATUS", 16, 0xFFC00CA8);
		d("DMA2_NEXT_DESC_PTR", 32, 0xFFC00C80);
		d("DMA2_PERIPHERAL_MAP", 16, 0xFFC00CAC);
		d("DMA2_START_ADDR", 32, 0xFFC00C84);
		d("DMA2_X_COUNT", 16, 0xFFC00C90);
		d("DMA2_X_MODIFY", 16, 0xFFC00C94);
		d("DMA2_Y_COUNT", 16, 0xFFC00C98);
		d("DMA2_Y_MODIFY", 16, 0xFFC00C9C);
		d("DMA3_CONFIG", 16, 0xFFC00CC8);
		d("DMA3_CURR_ADDR", 32, 0xFFC00CE4);
		d("DMA3_CURR_DESC_PTR", 32, 0xFFC00CE0);
		d("DMA3_CURR_X_COUNT", 16, 0xFFC00CF0);
		d("DMA3_CURR_Y_COUNT", 16, 0xFFC00CF8);
		d("DMA3_IRQ_STATUS", 16, 0xFFC00CE8);
		d("DMA3_NEXT_DESC_PTR", 32, 0xFFC00CC0);
		d("DMA3_PERIPHERAL_MAP", 16, 0xFFC00CEC);
		d("DMA3_START_ADDR", 32, 0xFFC00CC4);
		d("DMA3_X_COUNT", 16, 0xFFC00CD0);
		d("DMA3_X_MODIFY", 16, 0xFFC00CD4);
		d("DMA3_Y_COUNT", 16, 0xFFC00CD8);
		d("DMA3_Y_MODIFY", 16, 0xFFC00CDC);
		d("DMA4_CONFIG", 16, 0xFFC00D08);
		d("DMA4_CURR_ADDR", 32, 0xFFC00D24);
		d("DMA4_CURR_DESC_PTR", 32, 0xFFC00D20);
		d("DMA4_CURR_X_COUNT", 16, 0xFFC00D30);
		d("DMA4_CURR_Y_COUNT", 16, 0xFFC00D38);
		d("DMA4_IRQ_STATUS", 16, 0xFFC00D28);
		d("DMA4_NEXT_DESC_PTR", 32, 0xFFC00D00);
		d("DMA4_PERIPHERAL_MAP", 16, 0xFFC00D2C);
		d("DMA4_START_ADDR", 32, 0xFFC00D04);
		d("DMA4_X_COUNT", 16, 0xFFC00D10);
		d("DMA4_X_MODIFY", 16, 0xFFC00D14);
		d("DMA4_Y_COUNT", 16, 0xFFC00D18);
		d("DMA4_Y_MODIFY", 16, 0xFFC00D1C);
		d("DMA5_CONFIG", 16, 0xFFC00D48);
		d("DMA5_CURR_ADDR", 32, 0xFFC00D64);
		d("DMA5_CURR_DESC_PTR", 32, 0xFFC00D60);
		d("DMA5_CURR_X_COUNT", 16, 0xFFC00D70);
		d("DMA5_CURR_Y_COUNT", 16, 0xFFC00D78);
		d("DMA5_IRQ_STATUS", 16, 0xFFC00D68);
		d("DMA5_NEXT_DESC_PTR", 32, 0xFFC00D40);
		d("DMA5_PERIPHERAL_MAP", 16, 0xFFC00D6C);
		d("DMA5_START_ADDR", 32, 0xFFC00D44);
		d("DMA5_X_COUNT", 16, 0xFFC00D50);
		d("DMA5_X_MODIFY", 16, 0xFFC00D54);
		d("DMA5_Y_COUNT", 16, 0xFFC00D58);
		d("DMA5_Y_MODIFY", 16, 0xFFC00D5C);
		d("DMA6_CONFIG", 16, 0xFFC00D88);
		d("DMA6_CURR_ADDR", 32, 0xFFC00DA4);
		d("DMA6_CURR_DESC_PTR", 32, 0xFFC00DA0);
		d("DMA6_CURR_X_COUNT", 16, 0xFFC00DB0);
		d("DMA6_CURR_Y_COUNT", 16, 0xFFC00DB8);
		d("DMA6_IRQ_STATUS", 16, 0xFFC00DA8);
		d("DMA6_NEXT_DESC_PTR", 32, 0xFFC00D80);
		d("DMA6_PERIPHERAL_MAP", 16, 0xFFC00DAC);
		d("DMA6_START_ADDR", 32, 0xFFC00D84);
		d("DMA6_X_COUNT", 16, 0xFFC00D90);
		d("DMA6_X_MODIFY", 16, 0xFFC00D94);
		d("DMA6_Y_COUNT", 16, 0xFFC00D98);
		d("DMA6_Y_MODIFY", 16, 0xFFC00D9C);
		d("DMA7_CONFIG", 16, 0xFFC00DC8);
		d("DMA7_CURR_ADDR", 32, 0xFFC00DE4);
		d("DMA7_CURR_DESC_PTR", 32, 0xFFC00DE0);
		d("DMA7_CURR_X_COUNT", 16, 0xFFC00DF0);
		d("DMA7_CURR_Y_COUNT", 16, 0xFFC00DF8);
		d("DMA7_IRQ_STATUS", 16, 0xFFC00DE8);
		d("DMA7_NEXT_DESC_PTR", 32, 0xFFC00DC0);
		d("DMA7_PERIPHERAL_MAP", 16, 0xFFC00DEC);
		d("DMA7_START_ADDR", 32, 0xFFC00DC4);
		d("DMA7_X_COUNT", 16, 0xFFC00DD0);
		d("DMA7_X_MODIFY", 16, 0xFFC00DD4);
		d("DMA7_Y_COUNT", 16, 0xFFC00DD8);
		d("DMA7_Y_MODIFY", 16, 0xFFC00DDC);
		d("DMA8_CONFIG", 16, 0xFFC00E08);
		d("DMA8_CURR_ADDR", 32, 0xFFC00E24);
		d("DMA8_CURR_DESC_PTR", 32, 0xFFC00E20);
		d("DMA8_CURR_X_COUNT", 16, 0xFFC00E30);
		d("DMA8_CURR_Y_COUNT", 16, 0xFFC00E38);
		d("DMA8_IRQ_STATUS", 16, 0xFFC00E28);
		d("DMA8_NEXT_DESC_PTR", 32, 0xFFC00E00);
		d("DMA8_PERIPHERAL_MAP", 16, 0xFFC00E2C);
		d("DMA8_START_ADDR", 32, 0xFFC00E04);
		d("DMA8_X_COUNT", 16, 0xFFC00E10);
		d("DMA8_X_MODIFY", 16, 0xFFC00E14);
		d("DMA8_Y_COUNT", 16, 0xFFC00E18);
		d("DMA8_Y_MODIFY", 16, 0xFFC00E1C);
		d("DMA9_CONFIG", 16, 0xFFC00E48);
		d("DMA9_CURR_ADDR", 32, 0xFFC00E64);
		d("DMA9_CURR_DESC_PTR", 32, 0xFFC00E60);
		d("DMA9_CURR_X_COUNT", 16, 0xFFC00E70);
		d("DMA9_CURR_Y_COUNT", 16, 0xFFC00E78);
		d("DMA9_IRQ_STATUS", 16, 0xFFC00E68);
		d("DMA9_NEXT_DESC_PTR", 32, 0xFFC00E40);
		d("DMA9_PERIPHERAL_MAP", 16, 0xFFC00E6C);
		d("DMA9_START_ADDR", 32, 0xFFC00E44);
		d("DMA9_X_COUNT", 16, 0xFFC00E50);
		d("DMA9_X_MODIFY", 16, 0xFFC00E54);
		d("DMA9_Y_COUNT", 16, 0xFFC00E58);
		d("DMA9_Y_MODIFY", 16, 0xFFC00E5C);
		d("DMAC0_TCCNT", 16, 0xFFC00B10);
		d("DMAC0_TCPER", 16, 0xFFC00B0C);
		d("DMAC1_PERIMUX", 16, 0xFFC04340);
		d("DMAC1_TCCNT", 16, 0xFFC01B10);
		d("DMAC1_TCPER", 16, 0xFFC01B0C);

		parent = debugfs_create_dir("HMDMA", top);
		d("HMDMA0_BCINIT", 16, 0xFFC04508);
		d("HMDMA0_BCOUNT", 16, 0xFFC04518);
		d("HMDMA0_CONTROL", 16, 0xFFC04500);
		d("HMDMA0_ECINIT", 16, 0xFFC04504);
		d("HMDMA0_ECOUNT", 16, 0xFFC04514);
		d("HMDMA0_ECOVERFLOW", 16, 0xFFC04510);
		d("HMDMA0_ECURGENT", 16, 0xFFC0450C);
		d("HMDMA1_BCINIT", 16, 0xFFC04548);
		d("HMDMA1_BCOUNT", 16, 0xFFC04558);
		d("HMDMA1_CONTROL", 16, 0xFFC04540);
		d("HMDMA1_ECINIT", 16, 0xFFC04544);
		d("HMDMA1_ECOUNT", 16, 0xFFC04554);
		d("HMDMA1_ECOVERFLOW", 16, 0xFFC04550);
		d("HMDMA1_ECURGENT", 16, 0xFFC0454C);

		parent = debugfs_create_dir("MDMA", top);
		d("MDMA_D0_CONFIG", 16, 0xFFC00F08);
		d("MDMA_D0_CURR_ADDR", 32, 0xFFC00F24);
		d("MDMA_D0_CURR_DESC_PTR", 32, 0xFFC00F20);
		d("MDMA_D0_CURR_X_COUNT", 16, 0xFFC00F30);
		d("MDMA_D0_CURR_Y_COUNT", 16, 0xFFC00F38);
		d("MDMA_D0_IRQ_STATUS", 16, 0xFFC00F28);
		d("MDMA_D0_NEXT_DESC_PTR", 32, 0xFFC00F00);
		d("MDMA_D0_PERIPHERAL_MAP", 16, 0xFFC00F2C);
		d("MDMA_D0_START_ADDR", 32, 0xFFC00F04);
		d("MDMA_D0_X_COUNT", 16, 0xFFC00F10);
		d("MDMA_D0_X_MODIFY", 16, 0xFFC00F14);
		d("MDMA_D0_Y_COUNT", 16, 0xFFC00F18);
		d("MDMA_D0_Y_MODIFY", 16, 0xFFC00F1C);
		d("MDMA_D1_CONFIG", 16, 0xFFC00F88);
		d("MDMA_D1_CURR_ADDR", 32, 0xFFC00FA4);
		d("MDMA_D1_CURR_DESC_PTR", 32, 0xFFC00FA0);
		d("MDMA_D1_CURR_X_COUNT", 16, 0xFFC00FB0);
		d("MDMA_D1_CURR_Y_COUNT", 16, 0xFFC00FB8);
		d("MDMA_D1_IRQ_STATUS", 16, 0xFFC00FA8);
		d("MDMA_D1_NEXT_DESC_PTR", 32, 0xFFC00F80);
		d("MDMA_D1_PERIPHERAL_MAP", 16, 0xFFC00FAC);
		d("MDMA_D1_START_ADDR", 32, 0xFFC00F84);
		d("MDMA_D1_X_COUNT", 16, 0xFFC00F90);
		d("MDMA_D1_X_MODIFY", 16, 0xFFC00F94);
		d("MDMA_D1_Y_COUNT", 16, 0xFFC00F98);
		d("MDMA_D1_Y_MODIFY", 16, 0xFFC00F9C);
		d("MDMA_D2_CONFIG", 16, 0xFFC01F08);
		d("MDMA_D2_CURR_ADDR", 32, 0xFFC01F24);
		d("MDMA_D2_CURR_DESC_PTR", 32, 0xFFC01F20);
		d("MDMA_D2_CURR_X_COUNT", 16, 0xFFC01F30);
		d("MDMA_D2_CURR_Y_COUNT", 16, 0xFFC01F38);
		d("MDMA_D2_IRQ_STATUS", 16, 0xFFC01F28);
		d("MDMA_D2_NEXT_DESC_PTR", 32, 0xFFC01F00);
		d("MDMA_D2_PERIPHERAL_MAP", 16, 0xFFC01F2C);
		d("MDMA_D2_START_ADDR", 32, 0xFFC01F04);
		d("MDMA_D2_X_COUNT", 16, 0xFFC01F10);
		d("MDMA_D2_X_MODIFY", 16, 0xFFC01F14);
		d("MDMA_D2_Y_COUNT", 16, 0xFFC01F18);
		d("MDMA_D2_Y_MODIFY", 16, 0xFFC01F1C);
		d("MDMA_D3_CONFIG", 16, 0xFFC01F88);
		d("MDMA_D3_CURR_ADDR", 32, 0xFFC01FA4);
		d("MDMA_D3_CURR_DESC_PTR", 32, 0xFFC01FA0);
		d("MDMA_D3_CURR_X_COUNT", 16, 0xFFC01FB0);
		d("MDMA_D3_CURR_Y_COUNT", 16, 0xFFC01FB8);
		d("MDMA_D3_IRQ_STATUS", 16, 0xFFC01FA8);
		d("MDMA_D3_NEXT_DESC_PTR", 32, 0xFFC01F80);
		d("MDMA_D3_PERIPHERAL_MAP", 16, 0xFFC01FAC);
		d("MDMA_D3_START_ADDR", 32, 0xFFC01F84);
		d("MDMA_D3_X_COUNT", 16, 0xFFC01F90);
		d("MDMA_D3_X_MODIFY", 16, 0xFFC01F94);
		d("MDMA_D3_Y_COUNT", 16, 0xFFC01F98);
		d("MDMA_D3_Y_MODIFY", 16, 0xFFC01F9C);
		d("MDMA_S0_CONFIG", 16, 0xFFC00F48);
		d("MDMA_S0_CURR_ADDR", 32, 0xFFC00F64);
		d("MDMA_S0_CURR_DESC_PTR", 32, 0xFFC00F60);
		d("MDMA_S0_CURR_X_COUNT", 16, 0xFFC00F70);
		d("MDMA_S0_CURR_Y_COUNT", 16, 0xFFC00F78);
		d("MDMA_S0_IRQ_STATUS", 16, 0xFFC00F68);
		d("MDMA_S0_NEXT_DESC_PTR", 32, 0xFFC00F40);
		d("MDMA_S0_PERIPHERAL_MAP", 16, 0xFFC00F6C);
		d("MDMA_S0_START_ADDR", 32, 0xFFC00F44);
		d("MDMA_S0_X_COUNT", 16, 0xFFC00F50);
		d("MDMA_S0_X_MODIFY", 16, 0xFFC00F54);
		d("MDMA_S0_Y_COUNT", 16, 0xFFC00F58);
		d("MDMA_S0_Y_MODIFY", 16, 0xFFC00F5C);
		d("MDMA_S1_CONFIG", 16, 0xFFC00FC8);
		d("MDMA_S1_CURR_ADDR", 32, 0xFFC00FE4);
		d("MDMA_S1_CURR_DESC_PTR", 32, 0xFFC00FE0);
		d("MDMA_S1_CURR_X_COUNT", 16, 0xFFC00FF0);
		d("MDMA_S1_CURR_Y_COUNT", 16, 0xFFC00FF8);
		d("MDMA_S1_IRQ_STATUS", 16, 0xFFC00FE8);
		d("MDMA_S1_NEXT_DESC_PTR", 32, 0xFFC00FC0);
		d("MDMA_S1_PERIPHERAL_MAP", 16, 0xFFC00FEC);
		d("MDMA_S1_START_ADDR", 32, 0xFFC00FC4);
		d("MDMA_S1_X_COUNT", 16, 0xFFC00FD0);
		d("MDMA_S1_X_MODIFY", 16, 0xFFC00FD4);
		d("MDMA_S1_Y_COUNT", 16, 0xFFC00FD8);
		d("MDMA_S1_Y_MODIFY", 16, 0xFFC00FDC);
		d("MDMA_S2_CONFIG", 16, 0xFFC01F48);
		d("MDMA_S2_CURR_ADDR", 32, 0xFFC01F64);
		d("MDMA_S2_CURR_DESC_PTR", 32, 0xFFC01F60);
		d("MDMA_S2_CURR_X_COUNT", 16, 0xFFC01F70);
		d("MDMA_S2_CURR_Y_COUNT", 16, 0xFFC01F78);
		d("MDMA_S2_IRQ_STATUS", 16, 0xFFC01F68);
		d("MDMA_S2_NEXT_DESC_PTR", 32, 0xFFC01F40);
		d("MDMA_S2_PERIPHERAL_MAP", 16, 0xFFC01F6C);
		d("MDMA_S2_START_ADDR", 32, 0xFFC01F44);
		d("MDMA_S2_X_COUNT", 16, 0xFFC01F50);
		d("MDMA_S2_X_MODIFY", 16, 0xFFC01F54);
		d("MDMA_S2_Y_COUNT", 16, 0xFFC01F58);
		d("MDMA_S2_Y_MODIFY", 16, 0xFFC01F5C);
		d("MDMA_S3_CONFIG", 16, 0xFFC01FC8);
		d("MDMA_S3_CURR_ADDR", 32, 0xFFC01FE4);
		d("MDMA_S3_CURR_DESC_PTR", 32, 0xFFC01FE0);
		d("MDMA_S3_CURR_X_COUNT", 16, 0xFFC01FF0);
		d("MDMA_S3_CURR_Y_COUNT", 16, 0xFFC01FF8);
		d("MDMA_S3_IRQ_STATUS", 16, 0xFFC01FE8);
		d("MDMA_S3_NEXT_DESC_PTR", 32, 0xFFC01FC0);
		d("MDMA_S3_PERIPHERAL_MAP", 16, 0xFFC01FEC);
		d("MDMA_S3_START_ADDR", 32, 0xFFC01FC4);
		d("MDMA_S3_X_COUNT", 16, 0xFFC01FD0);
		d("MDMA_S3_X_MODIFY", 16, 0xFFC01FD4);
		d("MDMA_S3_Y_COUNT", 16, 0xFFC01FD8);
		d("MDMA_S3_Y_MODIFY", 16, 0xFFC01FDC);

		parent = debugfs_create_dir("PINT_0", top);
		d("PINT0_ASSIGN", 32, 0xFFC0140C);
		d("PINT0_EDGE_CLEAR", 32, 0xFFC01414);
		d("PINT0_EDGE_SET", 32, 0xFFC01410);
		d("PINT0_INVERT_CLEAR", 32, 0xFFC0141C);
		d("PINT0_INVERT_SET", 32, 0xFFC01418);
		d("PINT0_IRQ", 32, 0xFFC01408);
		d("PINT0_LATCH", 32, 0xFFC01424);
		d("PINT0_MASK_CLEAR", 32, 0xFFC01404);
		d("PINT0_MASK_SET", 32, 0xFFC01400);
		d("PINT0_PINSTATE", 32, 0xFFC01420);

		parent = debugfs_create_dir("PINT_1", top);
		d("PINT1_ASSIGN", 32, 0xFFC0143C);
		d("PINT1_EDGE_CLEAR", 32, 0xFFC01444);
		d("PINT1_EDGE_SET", 32, 0xFFC01440);
		d("PINT1_INVERT_CLEAR", 32, 0xFFC0144C);
		d("PINT1_INVERT_SET", 32, 0xFFC01448);
		d("PINT1_IRQ", 32, 0xFFC01438);
		d("PINT1_LATCH", 32, 0xFFC01454);
		d("PINT1_MASK_CLEAR", 32, 0xFFC01434);
		d("PINT1_MASK_SET", 32, 0xFFC01430);
		d("PINT1_PINSTATE", 32, 0xFFC01450);

		parent = debugfs_create_dir("PINT_2", top);
		d("PINT2_ASSIGN", 32, 0xFFC0146C);
		d("PINT2_EDGE_CLEAR", 32, 0xFFC01474);
		d("PINT2_EDGE_SET", 32, 0xFFC01470);
		d("PINT2_INVERT_CLEAR", 32, 0xFFC0147C);
		d("PINT2_INVERT_SET", 32, 0xFFC01478);
		d("PINT2_IRQ", 32, 0xFFC01468);
		d("PINT2_LATCH", 32, 0xFFC01484);
		d("PINT2_MASK_CLEAR", 32, 0xFFC01464);
		d("PINT2_MASK_SET", 32, 0xFFC01460);
		d("PINT2_PINSTATE", 32, 0xFFC01480);

		parent = debugfs_create_dir("PINT_3", top);
		d("PINT3_ASSIGN", 32, 0xFFC0149C);
		d("PINT3_EDGE_CLEAR", 32, 0xFFC014A4);
		d("PINT3_EDGE_SET", 32, 0xFFC014A0);
		d("PINT3_INVERT_CLEAR", 32, 0xFFC014AC);
		d("PINT3_INVERT_SET", 32, 0xFFC014A8);
		d("PINT3_IRQ", 32, 0xFFC01498);
		d("PINT3_LATCH", 32, 0xFFC014B4);
		d("PINT3_MASK_CLEAR", 32, 0xFFC01494);
		d("PINT3_MASK_SET", 32, 0xFFC01490);
		d("PINT3_PINSTATE", 32, 0xFFC014B0);

		parent = debugfs_create_dir("Port_A", top);
		d("PORTA", 16, 0xFFC014C4);
		d("PORTA_CLEAR", 16, 0xFFC014CC);
		d("PORTA_DIR_CLEAR", 16, 0xFFC014D4);
		d("PORTA_DIR_SET", 16, 0xFFC014D0);
		d("PORTA_FER", 16, 0xFFC014C0);
		d("PORTA_INEN", 16, 0xFFC014D8);
		d("PORTA_MUX", 32, 0xFFC014DC);
		d("PORTA_SET", 16, 0xFFC014C8);

		parent = debugfs_create_dir("Port_B", top);
		d("PORTB", 16, 0xFFC014E4);
		d("PORTB_CLEAR", 16, 0xFFC014EC);
		d("PORTB_DIR_CLEAR", 16, 0xFFC014F4);
		d("PORTB_DIR_SET", 16, 0xFFC014F0);
		d("PORTB_FER", 16, 0xFFC014E0);
		d("PORTB_INEN", 16, 0xFFC014F8);
		d("PORTB_MUX", 32, 0xFFC014FC);
		d("PORTB_SET", 16, 0xFFC014E8);

		parent = debugfs_create_dir("Port_C", top);
		d("PORTC", 16, 0xFFC01504);
		d("PORTC_CLEAR", 16, 0xFFC0150C);
		d("PORTC_DIR_CLEAR", 16, 0xFFC01514);
		d("PORTC_DIR_SET", 16, 0xFFC01510);
		d("PORTC_FER", 16, 0xFFC01500);
		d("PORTC_INEN", 16, 0xFFC01518);
		d("PORTC_MUX", 32, 0xFFC0151C);
		d("PORTC_SET", 16, 0xFFC01508);

		parent = debugfs_create_dir("Port_D", top);
		d("PORTD", 16, 0xFFC01524);
		d("PORTD_CLEAR", 16, 0xFFC0152C);
		d("PORTD_DIR_CLEAR", 16, 0xFFC01534);
		d("PORTD_DIR_SET", 16, 0xFFC01530);
		d("PORTD_FER", 16, 0xFFC01520);
		d("PORTD_INEN", 16, 0xFFC01538);
		d("PORTD_MUX", 32, 0xFFC0153C);
		d("PORTD_SET", 16, 0xFFC01528);

		parent = debugfs_create_dir("Port_E", top);
		d("PORTE", 16, 0xFFC01544);
		d("PORTE_CLEAR", 16, 0xFFC0154C);
		d("PORTE_DIR_CLEAR", 16, 0xFFC01554);
		d("PORTE_DIR_SET", 16, 0xFFC01550);
		d("PORTE_FER", 16, 0xFFC01540);
		d("PORTE_INEN", 16, 0xFFC01558);
		d("PORTE_MUX", 32, 0xFFC0155C);
		d("PORTE_SET", 16, 0xFFC01548);

		parent = debugfs_create_dir("Port_F", top);
		d("PORTF", 16, 0xFFC01564);
		d("PORTF_CLEAR", 16, 0xFFC0156C);
		d("PORTF_DIR_CLEAR", 16, 0xFFC01574);
		d("PORTF_DIR_SET", 16, 0xFFC01570);
		d("PORTF_FER", 16, 0xFFC01560);
		d("PORTF_INEN", 16, 0xFFC01578);
		d("PORTF_MUX", 32, 0xFFC0157C);
		d("PORTF_SET", 16, 0xFFC01568);

		parent = debugfs_create_dir("Port_G", top);
		d("PORTG", 16, 0xFFC01584);
		d("PORTG_CLEAR", 16, 0xFFC0158C);
		d("PORTG_DIR_CLEAR", 16, 0xFFC01594);
		d("PORTG_DIR_SET", 16, 0xFFC01590);
		d("PORTG_FER", 16, 0xFFC01580);
		d("PORTG_INEN", 16, 0xFFC01598);
		d("PORTG_MUX", 32, 0xFFC0159C);
		d("PORTG_SET", 16, 0xFFC01588);

		parent = debugfs_create_dir("Port_H", top);
		d("PORTH", 16, 0xFFC015A4);
		d("PORTH_CLEAR", 16, 0xFFC015AC);
		d("PORTH_DIR_CLEAR", 16, 0xFFC015B4);
		d("PORTH_DIR_SET", 16, 0xFFC015B0);
		d("PORTH_FER", 16, 0xFFC015A0);
		d("PORTH_INEN", 16, 0xFFC015B8);
		d("PORTH_MUX", 32, 0xFFC015BC);
		d("PORTH_SET", 16, 0xFFC015A8);

		parent = debugfs_create_dir("Port_I", top);
		d("PORTI", 16, 0xFFC015C4);
		d("PORTI_CLEAR", 16, 0xFFC015CC);
		d("PORTI_DIR_CLEAR", 16, 0xFFC015D4);
		d("PORTI_DIR_SET", 16, 0xFFC015D0);
		d("PORTI_FER", 16, 0xFFC015C0);
		d("PORTI_INEN", 16, 0xFFC015D8);
		d("PORTI_MUX", 32, 0xFFC015DC);
		d("PORTI_SET", 16, 0xFFC015C8);

		parent = debugfs_create_dir("Port_J", top);
		d("PORTJ", 16, 0xFFC015E4);
		d("PORTJ_CLEAR", 16, 0xFFC015EC);
		d("PORTJ_DIR_CLEAR", 16, 0xFFC015F4);
		d("PORTJ_DIR_SET", 16, 0xFFC015F0);
		d("PORTJ_FER", 16, 0xFFC015E0);
		d("PORTJ_INEN", 16, 0xFFC015F8);
		d("PORTJ_MUX", 32, 0xFFC015FC);
		d("PORTJ_SET", 16, 0xFFC015E8);

		parent = debugfs_create_dir("SPI", top);
		d("SPI0_BAUD", 16, 0xFFC00514);
		d("SPI0_CTL", 16, 0xFFC00500);
		d("SPI0_FLG", 16, 0xFFC00504);
		d("SPI0_RDBR", 16, 0xFFC00510);
		d("SPI0_SHADOW", 16, 0xFFC00518);
		d("SPI0_STAT", 16, 0xFFC00508);
		d("SPI0_TDBR", 16, 0xFFC0050C);
		d("SPI1_BAUD", 16, 0xFFC02314);
		d("SPI1_CTL", 16, 0xFFC02300);
		d("SPI1_FLG", 16, 0xFFC02304);
		d("SPI1_RDBR", 16, 0xFFC02310);
		d("SPI1_SHADOW", 16, 0xFFC02318);
		d("SPI1_STAT", 16, 0xFFC02308);
		d("SPI1_TDBR", 16, 0xFFC0230C);
		d("SPI2_BAUD", 16, 0xFFC02414);
		d("SPI2_CTL", 16, 0xFFC02400);
		d("SPI2_FLG", 16, 0xFFC02404);
		d("SPI2_RDBR", 16, 0xFFC02410);
		d("SPI2_SHADOW", 16, 0xFFC02418);
		d("SPI2_STAT", 16, 0xFFC02408);
		d("SPI2_TDBR", 16, 0xFFC0240C);

		parent = debugfs_create_dir("TWI", top);
		d("TWI0_CLKDIV", 16, 0xFFC00700);
		d("TWI0_CONTROL", 16, 0xFFC00704);
		d("TWI0_FIFO_CTL", 16, 0xFFC00728);
		d("TWI0_FIFO_STAT", 16, 0xFFC0072C);
		d("TWI0_INT_MASK", 16, 0xFFC00724);
		d("TWI0_INT_STAT", 16, 0xFFC00720);
		d("TWI0_MASTER_ADDR", 16, 0xFFC0071C);
		d("TWI0_MASTER_CTL", 16, 0xFFC00714);
		d("TWI0_MASTER_STAT", 16, 0xFFC00718);
		d("TWI0_RCV_DATA16", 16, 0xFFC0078C);
		d("TWI0_RCV_DATA8", 16, 0xFFC00788);
		d("TWI0_SLAVE_ADDR", 16, 0xFFC00710);
		d("TWI0_SLAVE_CTL", 16, 0xFFC00708);
		d("TWI0_SLAVE_STAT", 16, 0xFFC0070C);
		d("TWI0_XMT_DATA16", 16, 0xFFC00784);
		d("TWI0_XMT_DATA8", 16, 0xFFC00780);
		d("TWI1_CLKDIV", 16, 0xFFC02200);
		d("TWI1_CONTROL", 16, 0xFFC02204);
		d("TWI1_FIFO_CTL", 16, 0xFFC02228);
		d("TWI1_FIFO_STAT", 16, 0xFFC0222C);
		d("TWI1_INT_MASK", 16, 0xFFC02224);
		d("TWI1_INT_STAT", 16, 0xFFC02220);
		d("TWI1_MASTER_ADDR", 16, 0xFFC0221C);
		d("TWI1_MASTER_CTL", 16, 0xFFC02214);
		d("TWI1_MASTER_STAT", 16, 0xFFC02218);
		d("TWI1_RCV_DATA16", 16, 0xFFC0228C);
		d("TWI1_RCV_DATA8", 16, 0xFFC02288);
		d("TWI1_SLAVE_ADDR", 16, 0xFFC02210);
		d("TWI1_SLAVE_CTL", 16, 0xFFC02208);
		d("TWI1_SLAVE_STAT", 16, 0xFFC0220C);
		d("TWI1_XMT_DATA16", 16, 0xFFC02284);
		d("TWI1_XMT_DATA8", 16, 0xFFC02280);

		parent = debugfs_create_dir("UART0", top);
		d("UART0_DLH", 16, 0xFFC00404);
		d("UART0_DLL", 16, 0xFFC00400);
		d("UART0_GCTL", 16, 0xFFC00408);
		d("UART0_IER_CLEAR", 16, 0xFFC00424);
		d("UART0_IER_SET", 16, 0xFFC00420);
		d("UART0_LCR", 16, 0xFFC0040C);
		d("UART0_LSR", 16, 0xFFC00414);
		d("UART0_MCR", 16, 0xFFC00410);
		d("UART0_MSR", 16, 0xFFC00418);
		d("UART0_RBR", 16, 0xFFC0042C);
		d("UART0_SCR", 16, 0xFFC0041C);
		d("UART0_THR", 16, 0xFFC00428);

		parent = debugfs_create_dir("UART1", top);
		d("UART1_DLH", 16, 0xFFC02004);
		d("UART1_DLL", 16, 0xFFC02000);
		d("UART1_GCTL", 16, 0xFFC02008);
		d("UART1_IER_CLEAR", 16, 0xFFC02024);
		d("UART1_IER_SET", 16, 0xFFC02020);
		d("UART1_LCR", 16, 0xFFC0200C);
		d("UART1_LSR", 16, 0xFFC02014);
		d("UART1_MCR", 16, 0xFFC02010);
		d("UART1_MSR", 16, 0xFFC02018);
		d("UART1_RBR", 16, 0xFFC0202C);
		d("UART1_SCR", 16, 0xFFC0201C);
		d("UART1_THR", 16, 0xFFC02028);

		parent = debugfs_create_dir("UART2", top);
		d("UART2_DLH", 16, 0xFFC02104);
		d("UART2_DLL", 16, 0xFFC02100);
		d("UART2_GCTL", 16, 0xFFC02108);
		d("UART2_IER_CLEAR", 16, 0xFFC02124);
		d("UART2_IER_SET", 16, 0xFFC02120);
		d("UART2_LCR", 16, 0xFFC0210C);
		d("UART2_LSR", 16, 0xFFC02114);
		d("UART2_MCR", 16, 0xFFC02110);
		d("UART2_MSR", 16, 0xFFC02118);
		d("UART2_RBR", 16, 0xFFC0212C);
		d("UART2_SCR", 16, 0xFFC0211C);
		d("UART2_THR", 16, 0xFFC02128);

		parent = debugfs_create_dir("UART3", top);
		d("UART3_DLH", 16, 0xFFC03104);
		d("UART3_DLL", 16, 0xFFC03100);
		d("UART3_GCTL", 16, 0xFFC03108);
		d("UART3_IER_CLEAR", 16, 0xFFC03124);
		d("UART3_IER_SET", 16, 0xFFC03120);
		d("UART3_LCR", 16, 0xFFC0310C);
		d("UART3_LSR", 16, 0xFFC03114);
		d("UART3_MCR", 16, 0xFFC03110);
		d("UART3_MSR", 16, 0xFFC03118);
		d("UART3_RBR", 16, 0xFFC0312C);
		d("UART3_SCR", 16, 0xFFC0311C);
		d("UART3_THR", 16, 0xFFC03128);

		parent = debugfs_create_dir("USB", top);
		d("USB_APHY_CALIB", 16, 0xFFC03DE4);
		d("USB_APHY_CNTRL2", 16, 0xFFC03DE8);
		d("USB_APHY_CNTRL", 16, 0xFFC03DE0);
		d("USB_COUNT0", 16, 0xFFC03C50);
		d("USB_CSR0", 16, 0xFFC03C44);
		d("USB_DMA0_ADDRHIGH", 16, 0xFFC0400C);
		d("USB_DMA0_ADDRLOW", 16, 0xFFC04008);
		d("USB_DMA0_CONTROL", 16, 0xFFC04004);
		d("USB_DMA0_COUNTHIGH", 16, 0xFFC04014);
		d("USB_DMA0_COUNTLOW", 16, 0xFFC04010);
		d("USB_DMA1_ADDRHIGH", 16, 0xFFC0402C);
		d("USB_DMA1_ADDRLOW", 16, 0xFFC04028);
		d("USB_DMA1_CONTROL", 16, 0xFFC04024);
		d("USB_DMA1_COUNTHIGH", 16, 0xFFC04034);
		d("USB_DMA1_COUNTLOW", 16, 0xFFC04030);
		d("USB_DMA2_ADDRHIGH", 16, 0xFFC0404C);
		d("USB_DMA2_ADDRLOW", 16, 0xFFC04048);
		d("USB_DMA2_CONTROL", 16, 0xFFC04044);
		d("USB_DMA2_COUNTHIGH", 16, 0xFFC04054);
		d("USB_DMA2_COUNTLOW", 16, 0xFFC04050);
		d("USB_DMA3_ADDRHIGH", 16, 0xFFC0406C);
		d("USB_DMA3_ADDRLOW", 16, 0xFFC04068);
		d("USB_DMA3_CONTROL", 16, 0xFFC04064);
		d("USB_DMA3_COUNTHIGH", 16, 0xFFC04074);
		d("USB_DMA3_COUNTLOW", 16, 0xFFC04070);
		d("USB_DMA4_ADDRHIGH", 16, 0xFFC0408C);
		d("USB_DMA4_ADDRLOW", 16, 0xFFC04088);
		d("USB_DMA4_CONTROL", 16, 0xFFC04084);
		d("USB_DMA4_COUNTHIGH", 16, 0xFFC04094);
		d("USB_DMA4_COUNTLOW", 16, 0xFFC04090);
		d("USB_DMA5_ADDRHIGH", 16, 0xFFC040AC);
		d("USB_DMA5_ADDRLOW", 16, 0xFFC040A8);
		d("USB_DMA5_CONTROL", 16, 0xFFC040A4);
		d("USB_DMA5_COUNTHIGH", 16, 0xFFC040B4);
		d("USB_DMA5_COUNTLOW", 16, 0xFFC040B0);
		d("USB_DMA6_ADDRHIGH", 16, 0xFFC040CC);
		d("USB_DMA6_ADDRLOW", 16, 0xFFC040C8);
		d("USB_DMA6_CONTROL", 16, 0xFFC040C4);
		d("USB_DMA6_COUNTHIGH", 16, 0xFFC040D4);
		d("USB_DMA6_COUNTLOW", 16, 0xFFC040D0);
		d("USB_DMA7_ADDRHIGH", 16, 0xFFC040EC);
		d("USB_DMA7_ADDRLOW", 16, 0xFFC040E8);
		d("USB_DMA7_CONTROL", 16, 0xFFC040E4);
		d("USB_DMA7_COUNTHIGH", 16, 0xFFC040F4);
		d("USB_DMA7_COUNTLOW", 16, 0xFFC040F0);
		d("USB_DMA_INTERRUPT", 16, 0xFFC04000);
		d("USB_EP0_FIFO", 16, 0xFFC03C80);
		d("USB_EP1_FIFO", 16, 0xFFC03C88);
		d("USB_EP2_FIFO", 16, 0xFFC03C90);
		d("USB_EP3_FIFO", 16, 0xFFC03C98);
		d("USB_EP4_FIFO", 16, 0xFFC03CA0);
		d("USB_EP5_FIFO", 16, 0xFFC03CA8);
		d("USB_EP6_FIFO", 16, 0xFFC03CB0);
		d("USB_EP7_FIFO", 16, 0xFFC03CB8);
		d("USB_EP_NI0_RXCOUNT", 16, 0xFFC03E10);
		d("USB_EP_NI0_RXCSR", 16, 0xFFC03E0C);
		d("USB_EP_NI0_RXINTERVAL", 16, 0xFFC03E20);
		d("USB_EP_NI0_RXMAXP", 16, 0xFFC03E08);
		d("USB_EP_NI0_RXTYPE", 16, 0xFFC03E1C);
		d("USB_EP_NI0_TXCOUNT", 16, 0xFFC03E28);
		d("USB_EP_NI0_TXCSR", 16, 0xFFC03E04);
		d("USB_EP_NI0_TXINTERVAL", 16, 0xFFC03E18);
		d("USB_EP_NI0_TXMAXP", 16, 0xFFC03E00);
		d("USB_EP_NI0_TXTYPE", 16, 0xFFC03E14);
		d("USB_EP_NI1_RXCOUNT", 16, 0xFFC03E50);
		d("USB_EP_NI1_RXCSR", 16, 0xFFC03E4C);
		d("USB_EP_NI1_RXINTERVAL", 16, 0xFFC03E60);
		d("USB_EP_NI1_RXMAXP", 16, 0xFFC03E48);
		d("USB_EP_NI1_RXTYPE", 16, 0xFFC03E5C);
		d("USB_EP_NI1_TXCOUNT", 16, 0xFFC03E68);
		d("USB_EP_NI1_TXCSR", 16, 0xFFC03E44);
		d("USB_EP_NI1_TXINTERVAL", 16, 0xFFC03E58);
		d("USB_EP_NI1_TXMAXP", 16, 0xFFC03E40);
		d("USB_EP_NI1_TXTYPE", 16, 0xFFC03E54);
		d("USB_EP_NI2_RXCOUNT", 16, 0xFFC03E90);
		d("USB_EP_NI2_RXCSR", 16, 0xFFC03E8C);
		d("USB_EP_NI2_RXINTERVAL", 16, 0xFFC03EA0);
		d("USB_EP_NI2_RXMAXP", 16, 0xFFC03E88);
		d("USB_EP_NI2_RXTYPE", 16, 0xFFC03E9C);
		d("USB_EP_NI2_TXCOUNT", 16, 0xFFC03EA8);
		d("USB_EP_NI2_TXCSR", 16, 0xFFC03E84);
		d("USB_EP_NI2_TXINTERVAL", 16, 0xFFC03E98);
		d("USB_EP_NI2_TXMAXP", 16, 0xFFC03E80);
		d("USB_EP_NI2_TXTYPE", 16, 0xFFC03E94);
		d("USB_EP_NI3_RXCOUNT", 16, 0xFFC03ED0);
		d("USB_EP_NI3_RXCSR", 16, 0xFFC03ECC);
		d("USB_EP_NI3_RXINTERVAL", 16, 0xFFC03EE0);
		d("USB_EP_NI3_RXMAXP", 16, 0xFFC03EC8);
		d("USB_EP_NI3_RXTYPE", 16, 0xFFC03EDC);
		d("USB_EP_NI3_TXCOUNT", 16, 0xFFC03EE8);
		d("USB_EP_NI3_TXCSR", 16, 0xFFC03EC4);
		d("USB_EP_NI3_TXINTERVAL", 16, 0xFFC03ED8);
		d("USB_EP_NI3_TXMAXP", 16, 0xFFC03EC0);
		d("USB_EP_NI3_TXTYPE", 16, 0xFFC03ED4);
		d("USB_EP_NI4_RXCOUNT", 16, 0xFFC03F10);
		d("USB_EP_NI4_RXCSR", 16, 0xFFC03F0C);
		d("USB_EP_NI4_RXINTERVAL", 16, 0xFFC03F20);
		d("USB_EP_NI4_RXMAXP", 16, 0xFFC03F08);
		d("USB_EP_NI4_RXTYPE", 16, 0xFFC03F1C);
		d("USB_EP_NI4_TXCOUNT", 16, 0xFFC03F28);
		d("USB_EP_NI4_TXCSR", 16, 0xFFC03F04);
		d("USB_EP_NI4_TXINTERVAL", 16, 0xFFC03F18);
		d("USB_EP_NI4_TXMAXP", 16, 0xFFC03F00);
		d("USB_EP_NI4_TXTYPE", 16, 0xFFC03F14);
		d("USB_EP_NI5_RXCOUNT", 16, 0xFFC03F50);
		d("USB_EP_NI5_RXCSR", 16, 0xFFC03F4C);
		d("USB_EP_NI5_RXINTERVAL", 16, 0xFFC03F60);
		d("USB_EP_NI5_RXMAXP", 16, 0xFFC03F48);
		d("USB_EP_NI5_RXTYPE", 16, 0xFFC03F5C);
		d("USB_EP_NI5_TXCOUNT", 16, 0xFFC03F68);
		d("USB_EP_NI5_TXCSR", 16, 0xFFC03F44);
		d("USB_EP_NI5_TXINTERVAL", 16, 0xFFC03F58);
		d("USB_EP_NI5_TXMAXP", 16, 0xFFC03F40);
		d("USB_EP_NI5_TXTYPE", 16, 0xFFC03F54);
		d("USB_EP_NI6_RXCOUNT", 16, 0xFFC03F90);
		d("USB_EP_NI6_RXCSR", 16, 0xFFC03F8C);
		d("USB_EP_NI6_RXINTERVAL", 16, 0xFFC03FA0);
		d("USB_EP_NI6_RXMAXP", 16, 0xFFC03F88);
		d("USB_EP_NI6_RXTYPE", 16, 0xFFC03F9C);
		d("USB_EP_NI6_TXCOUNT", 16, 0xFFC03FA8);
		d("USB_EP_NI6_TXCSR", 16, 0xFFC03F84);
		d("USB_EP_NI6_TXINTERVAL", 16, 0xFFC03F98);
		d("USB_EP_NI6_TXMAXP", 16, 0xFFC03F80);
		d("USB_EP_NI6_TXTYPE", 16, 0xFFC03F94);
		d("USB_EP_NI7_RXCOUNT", 16, 0xFFC03FD0);
		d("USB_EP_NI7_RXCSR", 16, 0xFFC03FCC);
		d("USB_EP_NI7_RXINTERVAL", 16, 0xFFC03FE0);
		d("USB_EP_NI7_RXMAXP", 16, 0xFFC03FC8);
		d("USB_EP_NI7_RXTYPE", 16, 0xFFC03FDC);
		d("USB_EP_NI7_TXCOUNT", 16, 0xFFC03FE8);
		d("USB_EP_NI7_TXCSR", 16, 0xFFC03FC4);
		d("USB_EP_NI7_TXINTERVAL", 16, 0xFFC03FD8);
		d("USB_EP_NI7_TXMAXP", 16, 0xFFC03FC0);
		d("USB_EP_NI7_TXTYPE", 16, 0xFFC03FD4);
		d("USB_FADDR", 16, 0xFFC03C00);
		d("USB_FRAME", 16, 0xFFC03C20);
		d("USB_FS_EOF1", 16, 0xFFC03D54);
		d("USB_GLOBAL_CTL", 16, 0xFFC03C30);
		d("USB_GLOBINTR", 16, 0xFFC03C2C);
		d("USB_HS_EOF1", 16, 0xFFC03D50);
		d("USB_INDEX", 16, 0xFFC03C24);
		d("USB_INTRRX", 16, 0xFFC03C0C);
		d("USB_INTRRXE", 16, 0xFFC03C14);
		d("USB_INTRTX", 16, 0xFFC03C08);
		d("USB_INTRTXE", 16, 0xFFC03C10);
		d("USB_INTRUSB", 16, 0xFFC03C18);
		d("USB_INTRUSBE", 16, 0xFFC03C1C);
		d("USB_LINKINFO", 16, 0xFFC03D48);
		d("USB_LS_EOF1", 16, 0xFFC03D58);
		d("USB_NAKLIMIT0", 16, 0xFFC03C58);
		d("USB_OTG_DEV_CTL", 16, 0xFFC03D00);
		d("USB_OTG_VBUS_IRQ", 16, 0xFFC03D04);
		d("USB_OTG_VBUS_MASK", 16, 0xFFC03D08);
		d("USB_PHY_TEST", 16, 0xFFC03DEC);
		d("USB_PLLOSC_CTRL", 16, 0xFFC03DF0);
		d("USB_POWER", 16, 0xFFC03C04);
		d("USB_RXCOUNT", 16, 0xFFC03C50);
		d("USB_RXCSR", 16, 0xFFC03C4C);
		d("USB_RXINTERVAL", 16, 0xFFC03C60);
		d("USB_RXTYPE", 16, 0xFFC03C5C);
		d("USB_RX_MAX_PACKET", 16, 0xFFC03C48);
		d("USB_SRP_CLKDIV", 16, 0xFFC03DF4);
		d("USB_TESTMODE", 16, 0xFFC03C28);
		d("USB_TXCOUNT", 16, 0xFFC03C68);
		d("USB_TXCSR", 16, 0xFFC03C44);
		d("USB_TXINTERVAL", 16, 0xFFC03C58);
		d("USB_TXTYPE", 16, 0xFFC03C54);
		d("USB_TX_MAX_PACKET", 16, 0xFFC03C40);
		d("USB_VPLEN", 16, 0xFFC03D4C);

	}	/* BF547 BF548 BF549 */

#ifdef __ADSPBF561__
# define USE_BF561 1
#else
# define USE_BF561 0
#endif
	if (USE_BF561) {

		parent = debugfs_create_dir("DMA Traffic Control", top);
		d("DMA1_TC_CNT", 16, 0xFFC01B10);
		d("DMA1_TC_PER", 16, 0xFFC01B0C);
		d("DMA2_TC_CNT", 16, 0xFFC01B10);
		d("DMA2_TC_PER", 16, 0xFFC00B0C);

		parent = debugfs_create_dir("DMA1 Channel-0", top);
		d("DMA1_0_CONFIG", 16, 0xFFC01C08);
		d("DMA1_0_CURR_ADDR", 32, 0xFFC01C24);
		d("DMA1_0_CURR_DESC_PTR", 32, 0xFFC01C20);
		d("DMA1_0_CURR_X_COUNT", 16, 0xFFC01C30);
		d("DMA1_0_CURR_Y_COUNT", 16, 0xFFC01C38);
		d("DMA1_0_IRQ_STATUS", 16, 0xFFC01C28);
		d("DMA1_0_NEXT_DESC_PTR", 32, 0xFFC01C00);
		d("DMA1_0_PERIPHERAL_MAP", 16, 0xFFC01C2C);
		d("DMA1_0_START_ADDR", 32, 0xFFC01C04);
		d("DMA1_0_X_COUNT", 16, 0xFFC01C10);
		d("DMA1_0_X_MODIFY", 16, 0xFFC01C14);
		d("DMA1_0_Y_COUNT", 16, 0xFFC01C18);
		d("DMA1_0_Y_MODIFY", 16, 0xFFC01C1C);

		parent = debugfs_create_dir("DMA1 Channel-10", top);
		d("DMA1_10_CONFIG", 16, 0xFFC01E88);
		d("DMA1_10_CURR_ADDR", 32, 0xFFC01EA4);
		d("DMA1_10_CURR_DESC_PTR", 32, 0xFFC01EA0);
		d("DMA1_10_CURR_X_COUNT", 16, 0xFFC01EB0);
		d("DMA1_10_CURR_Y_COUNT", 16, 0xFFC01EB8);
		d("DMA1_10_IRQ_STATUS", 16, 0xFFC01EA8);
		d("DMA1_10_NEXT_DESC_PTR", 32, 0xFFC01E80);
		d("DMA1_10_PERIPHERAL_MAP", 16, 0xFFC01EAC);
		d("DMA1_10_START_ADDR", 32, 0xFFC01E84);
		d("DMA1_10_X_COUNT", 16, 0xFFC01E90);
		d("DMA1_10_X_MODIFY", 16, 0xFFC01E94);
		d("DMA1_10_Y_COUNT", 16, 0xFFC01E98);
		d("DMA1_10_Y_MODIFY", 16, 0xFFC01E9C);

		parent = debugfs_create_dir("DMA1 Channel-11", top);
		d("DMA1_11_CONFIG", 16, 0xFFC01EC8);
		d("DMA1_11_CURR_ADDR", 32, 0xFFC01EE4);
		d("DMA1_11_CURR_DESC_PTR", 32, 0xFFC01EE0);
		d("DMA1_11_CURR_X_COUNT", 16, 0xFFC01EF0);
		d("DMA1_11_CURR_Y_COUNT", 16, 0xFFC01EF8);
		d("DMA1_11_IRQ_STATUS", 16, 0xFFC01EE8);
		d("DMA1_11_NEXT_DESC_PTR", 32, 0xFFC01EC0);
		d("DMA1_11_PERIPHERAL_MAP", 16, 0xFFC01EEC);
		d("DMA1_11_START_ADDR", 32, 0xFFC01EC4);
		d("DMA1_11_X_COUNT", 16, 0xFFC01ED0);
		d("DMA1_11_X_MODIFY", 16, 0xFFC01ED4);
		d("DMA1_11_Y_COUNT", 16, 0xFFC01ED8);
		d("DMA1_11_Y_MODIFY", 16, 0xFFC01EDC);

		parent = debugfs_create_dir("DMA1 Channel-1", top);
		d("DMA1_1_CONFIG", 16, 0xFFC01C48);
		d("DMA1_1_CURR_ADDR", 32, 0xFFC01C64);
		d("DMA1_1_CURR_DESC_PTR", 32, 0xFFC01C60);
		d("DMA1_1_CURR_X_COUNT", 16, 0xFFC01C70);
		d("DMA1_1_CURR_Y_COUNT", 16, 0xFFC01C78);
		d("DMA1_1_IRQ_STATUS", 16, 0xFFC01C68);
		d("DMA1_1_NEXT_DESC_PTR", 32, 0xFFC01C40);
		d("DMA1_1_PERIPHERAL_MAP", 16, 0xFFC01C6C);
		d("DMA1_1_START_ADDR", 32, 0xFFC01C44);
		d("DMA1_1_X_COUNT", 16, 0xFFC01C50);
		d("DMA1_1_X_MODIFY", 16, 0xFFC01C54);
		d("DMA1_1_Y_COUNT", 16, 0xFFC01C58);
		d("DMA1_1_Y_MODIFY", 16, 0xFFC01C5C);

		parent = debugfs_create_dir("DMA1 Channel-2", top);
		d("DMA1_2_CONFIG", 16, 0xFFC01C88);
		d("DMA1_2_CURR_ADDR", 32, 0xFFC01CA4);
		d("DMA1_2_CURR_DESC_PTR", 32, 0xFFC01CA0);
		d("DMA1_2_CURR_X_COUNT", 16, 0xFFC01CB0);
		d("DMA1_2_CURR_Y_COUNT", 16, 0xFFC01CB8);
		d("DMA1_2_IRQ_STATUS", 16, 0xFFC01CA8);
		d("DMA1_2_NEXT_DESC_PTR", 32, 0xFFC01C80);
		d("DMA1_2_PERIPHERAL_MAP", 16, 0xFFC01CAC);
		d("DMA1_2_START_ADDR", 32, 0xFFC01C84);
		d("DMA1_2_X_COUNT", 16, 0xFFC01C90);
		d("DMA1_2_X_MODIFY", 16, 0xFFC01C94);
		d("DMA1_2_Y_COUNT", 16, 0xFFC01C98);
		d("DMA1_2_Y_MODIFY", 16, 0xFFC01C9C);

		parent = debugfs_create_dir("DMA1 Channel-3", top);
		d("DMA1_3_CONFIG", 16, 0xFFC01CC8);
		d("DMA1_3_CURR_ADDR", 32, 0xFFC01CE4);
		d("DMA1_3_CURR_DESC_PTR", 32, 0xFFC01CE0);
		d("DMA1_3_CURR_X_COUNT", 16, 0xFFC01CF0);
		d("DMA1_3_CURR_Y_COUNT", 16, 0xFFC01CF8);
		d("DMA1_3_IRQ_STATUS", 16, 0xFFC01CE8);
		d("DMA1_3_NEXT_DESC_PTR", 32, 0xFFC01CC0);
		d("DMA1_3_PERIPHERAL_MAP", 16, 0xFFC01CEC);
		d("DMA1_3_START_ADDR", 32, 0xFFC01CC4);
		d("DMA1_3_X_COUNT", 16, 0xFFC01CD0);
		d("DMA1_3_X_MODIFY", 16, 0xFFC01CD4);
		d("DMA1_3_Y_COUNT", 16, 0xFFC01CD8);
		d("DMA1_3_Y_MODIFY", 16, 0xFFC01CDC);

		parent = debugfs_create_dir("DMA1 Channel-4", top);
		d("DMA1_4_CONFIG", 16, 0xFFC01D08);
		d("DMA1_4_CURR_ADDR", 32, 0xFFC01D24);
		d("DMA1_4_CURR_DESC_PTR", 32, 0xFFC01D20);
		d("DMA1_4_CURR_X_COUNT", 16, 0xFFC01D30);
		d("DMA1_4_CURR_Y_COUNT", 16, 0xFFC01D38);
		d("DMA1_4_IRQ_STATUS", 16, 0xFFC01D28);
		d("DMA1_4_NEXT_DESC_PTR", 32, 0xFFC01D00);
		d("DMA1_4_PERIPHERAL_MAP", 16, 0xFFC01D2C);
		d("DMA1_4_START_ADDR", 32, 0xFFC01D04);
		d("DMA1_4_X_COUNT", 16, 0xFFC01D10);
		d("DMA1_4_X_MODIFY", 16, 0xFFC01D14);
		d("DMA1_4_Y_COUNT", 16, 0xFFC01D18);
		d("DMA1_4_Y_MODIFY", 16, 0xFFC01D1C);

		parent = debugfs_create_dir("DMA1 Channel-5", top);
		d("DMA1_5_CONFIG", 16, 0xFFC01D48);
		d("DMA1_5_CURR_ADDR", 32, 0xFFC01D64);
		d("DMA1_5_CURR_DESC_PTR", 32, 0xFFC01D60);
		d("DMA1_5_CURR_X_COUNT", 16, 0xFFC01D70);
		d("DMA1_5_CURR_Y_COUNT", 16, 0xFFC01D78);
		d("DMA1_5_IRQ_STATUS", 16, 0xFFC01D68);
		d("DMA1_5_NEXT_DESC_PTR", 32, 0xFFC01D40);
		d("DMA1_5_PERIPHERAL_MAP", 16, 0xFFC01D6C);
		d("DMA1_5_START_ADDR", 32, 0xFFC01D44);
		d("DMA1_5_X_COUNT", 16, 0xFFC01D50);
		d("DMA1_5_X_MODIFY", 16, 0xFFC01D54);
		d("DMA1_5_Y_COUNT", 16, 0xFFC01D58);
		d("DMA1_5_Y_MODIFY", 16, 0xFFC01D5C);

		parent = debugfs_create_dir("DMA1 Channel-6", top);
		d("DMA1_6_CONFIG", 16, 0xFFC01D88);
		d("DMA1_6_CURR_ADDR", 32, 0xFFC01DA4);
		d("DMA1_6_CURR_DESC_PTR", 32, 0xFFC01DA0);
		d("DMA1_6_CURR_X_COUNT", 16, 0xFFC01DB0);
		d("DMA1_6_CURR_Y_COUNT", 16, 0xFFC01DB8);
		d("DMA1_6_IRQ_STATUS", 16, 0xFFC01DA8);
		d("DMA1_6_NEXT_DESC_PTR", 32, 0xFFC01D80);
		d("DMA1_6_PERIPHERAL_MAP", 16, 0xFFC01DAC);
		d("DMA1_6_START_ADDR", 32, 0xFFC01D84);
		d("DMA1_6_X_COUNT", 16, 0xFFC01D90);
		d("DMA1_6_X_MODIFY", 16, 0xFFC01D94);
		d("DMA1_6_Y_COUNT", 16, 0xFFC01D98);
		d("DMA1_6_Y_MODIFY", 16, 0xFFC01D9C);

		parent = debugfs_create_dir("DMA1 Channel-7", top);
		d("DMA1_7_CONFIG", 16, 0xFFC01DC8);
		d("DMA1_7_CURR_ADDR", 32, 0xFFC01DE4);
		d("DMA1_7_CURR_DESC_PTR", 32, 0xFFC01DE0);
		d("DMA1_7_CURR_X_COUNT", 16, 0xFFC01DF0);
		d("DMA1_7_CURR_Y_COUNT", 16, 0xFFC01DF8);
		d("DMA1_7_IRQ_STATUS", 16, 0xFFC01DE8);
		d("DMA1_7_NEXT_DESC_PTR", 32, 0xFFC01DC0);
		d("DMA1_7_PERIPHERAL_MAP", 16, 0xFFC01DEC);
		d("DMA1_7_START_ADDR", 32, 0xFFC01DC4);
		d("DMA1_7_X_COUNT", 16, 0xFFC01DD0);
		d("DMA1_7_X_MODIFY", 16, 0xFFC01DD4);
		d("DMA1_7_Y_COUNT", 16, 0xFFC01DD8);
		d("DMA1_7_Y_MODIFY", 16, 0xFFC01DDC);

		parent = debugfs_create_dir("DMA1 Channel-8", top);
		d("DMA1_8_CONFIG", 16, 0xFFC01E08);
		d("DMA1_8_CURR_ADDR", 32, 0xFFC01E24);
		d("DMA1_8_CURR_DESC_PTR", 32, 0xFFC01E20);
		d("DMA1_8_CURR_X_COUNT", 16, 0xFFC01E30);
		d("DMA1_8_CURR_Y_COUNT", 16, 0xFFC01E38);
		d("DMA1_8_IRQ_STATUS", 16, 0xFFC01E28);
		d("DMA1_8_NEXT_DESC_PTR", 32, 0xFFC01E00);
		d("DMA1_8_PERIPHERAL_MAP", 16, 0xFFC01E2C);
		d("DMA1_8_START_ADDR", 32, 0xFFC01E04);
		d("DMA1_8_X_COUNT", 16, 0xFFC01E10);
		d("DMA1_8_X_MODIFY", 16, 0xFFC01E14);
		d("DMA1_8_Y_COUNT", 16, 0xFFC01E18);
		d("DMA1_8_Y_MODIFY", 16, 0xFFC01E1C);

		parent = debugfs_create_dir("DMA1 Channel-9", top);
		d("DMA1_9_CONFIG", 16, 0xFFC01E48);
		d("DMA1_9_CURR_ADDR", 32, 0xFFC01E64);
		d("DMA1_9_CURR_DESC_PTR", 32, 0xFFC01E60);
		d("DMA1_9_CURR_X_COUNT", 16, 0xFFC01E70);
		d("DMA1_9_CURR_Y_COUNT", 16, 0xFFC01E78);
		d("DMA1_9_IRQ_STATUS", 16, 0xFFC01E68);
		d("DMA1_9_NEXT_DESC_PTR", 32, 0xFFC01E40);
		d("DMA1_9_PERIPHERAL_MAP", 16, 0xFFC01E6C);
		d("DMA1_9_START_ADDR", 32, 0xFFC01E44);
		d("DMA1_9_X_COUNT", 16, 0xFFC01E50);
		d("DMA1_9_X_MODIFY", 16, 0xFFC01E54);
		d("DMA1_9_Y_COUNT", 16, 0xFFC01E58);
		d("DMA1_9_Y_MODIFY", 16, 0xFFC01E5C);

		parent = debugfs_create_dir("DMA2 Channel 0", top);
		d("DMA2_0_CONFIG", 16, 0xFFC00C08);
		d("DMA2_0_CURR_ADDR", 32, 0xFFC00C24);
		d("DMA2_0_CURR_DESC_PTR", 32, 0xFFC00C20);
		d("DMA2_0_CURR_X_COUNT", 16, 0xFFC00C30);
		d("DMA2_0_CURR_Y_COUNT", 16, 0xFFC00C38);
		d("DMA2_0_IRQ_STATUS", 16, 0xFFC00C28);
		d("DMA2_0_NEXT_DESC_PTR", 32, 0xFFC00C00);
		d("DMA2_0_PERIPHERAL_MAP", 16, 0xFFC00C2C);
		d("DMA2_0_START_ADDR", 32, 0xFFC00C04);
		d("DMA2_0_X_COUNT", 16, 0xFFC00C10);
		d("DMA2_0_X_MODIFY", 16, 0xFFC00C14);
		d("DMA2_0_Y_COUNT", 16, 0xFFC00C18);
		d("DMA2_0_Y_MODIFY", 16, 0xFFC00C1C);

		parent = debugfs_create_dir("DMA2 Channel 10", top);
		d("DMA2_10_CONFIG", 16, 0xFFC00E88);
		d("DMA2_10_CURR_ADDR", 32, 0xFFC00EA4);
		d("DMA2_10_CURR_DESC_PTR", 32, 0xFFC00EA0);
		d("DMA2_10_CURR_X_COUNT", 16, 0xFFC00EB0);
		d("DMA2_10_CURR_Y_COUNT", 16, 0xFFC00EB8);
		d("DMA2_10_IRQ_STATUS", 16, 0xFFC00EA8);
		d("DMA2_10_NEXT_DESC_PTR", 32, 0xFFC00E80);
		d("DMA2_10_PERIPHERAL_MAP", 16, 0xFFC00EAC);
		d("DMA2_10_START_ADDR", 32, 0xFFC00E84);
		d("DMA2_10_X_COUNT", 16, 0xFFC00E90);
		d("DMA2_10_X_MODIFY", 16, 0xFFC00E94);
		d("DMA2_10_Y_COUNT", 16, 0xFFC00E98);
		d("DMA2_10_Y_MODIFY", 16, 0xFFC00E9C);

		parent = debugfs_create_dir("DMA2 Channel 11", top);
		d("DMA2_11_CONFIG", 16, 0xFFC00EC8);
		d("DMA2_11_CURR_ADDR", 32, 0xFFC00EE4);
		d("DMA2_11_CURR_DESC_PTR", 32, 0xFFC00EE0);
		d("DMA2_11_CURR_X_COUNT", 16, 0xFFC00EF0);
		d("DMA2_11_CURR_Y_COUNT", 16, 0xFFC00EF8);
		d("DMA2_11_IRQ_STATUS", 16, 0xFFC00EE8);
		d("DMA2_11_NEXT_DESC_PTR", 32, 0xFFC00EC0);
		d("DMA2_11_PERIPHERAL_MAP", 16, 0xFFC00EEC);
		d("DMA2_11_START_ADDR", 32, 0xFFC00EC4);
		d("DMA2_11_X_COUNT", 16, 0xFFC00ED0);
		d("DMA2_11_X_MODIFY", 16, 0xFFC00ED4);
		d("DMA2_11_Y_COUNT", 16, 0xFFC00ED8);
		d("DMA2_11_Y_MODIFY", 16, 0xFFC00EDC);

		parent = debugfs_create_dir("DMA2 Channel 1", top);
		d("DMA2_1_CONFIG", 16, 0xFFC00C48);
		d("DMA2_1_CURR_ADDR", 32, 0xFFC00C64);
		d("DMA2_1_CURR_DESC_PTR", 32, 0xFFC00C60);
		d("DMA2_1_CURR_X_COUNT", 16, 0xFFC00C70);
		d("DMA2_1_CURR_Y_COUNT", 16, 0xFFC00C78);
		d("DMA2_1_IRQ_STATUS", 16, 0xFFC00C68);
		d("DMA2_1_NEXT_DESC_PTR", 32, 0xFFC00C40);
		d("DMA2_1_PERIPHERAL_MAP", 16, 0xFFC00C6C);
		d("DMA2_1_START_ADDR", 32, 0xFFC00C44);
		d("DMA2_1_X_COUNT", 16, 0xFFC00C50);
		d("DMA2_1_X_MODIFY", 16, 0xFFC00C54);
		d("DMA2_1_Y_COUNT", 16, 0xFFC00C58);
		d("DMA2_1_Y_MODIFY", 16, 0xFFC00C5C);

		parent = debugfs_create_dir("DMA2 Channel 2", top);
		d("DMA2_2_CONFIG", 16, 0xFFC00C88);
		d("DMA2_2_CURR_ADDR", 32, 0xFFC00CA4);
		d("DMA2_2_CURR_DESC_PTR", 32, 0xFFC00CA0);
		d("DMA2_2_CURR_X_COUNT", 16, 0xFFC00CB0);
		d("DMA2_2_CURR_Y_COUNT", 16, 0xFFC00CB8);
		d("DMA2_2_IRQ_STATUS", 16, 0xFFC00CA8);
		d("DMA2_2_NEXT_DESC_PTR", 32, 0xFFC00C80);
		d("DMA2_2_PERIPHERAL_MAP", 16, 0xFFC00CAC);
		d("DMA2_2_START_ADDR", 32, 0xFFC00C84);
		d("DMA2_2_X_COUNT", 16, 0xFFC00C90);
		d("DMA2_2_X_MODIFY", 16, 0xFFC00C94);
		d("DMA2_2_Y_COUNT", 16, 0xFFC00C98);
		d("DMA2_2_Y_MODIFY", 16, 0xFFC00C9C);

		parent = debugfs_create_dir("DMA2 Channel 3", top);
		d("DMA2_3_CONFIG", 16, 0xFFC00CC8);
		d("DMA2_3_CURR_ADDR", 32, 0xFFC00CE4);
		d("DMA2_3_CURR_DESC_PTR", 32, 0xFFC00CE0);
		d("DMA2_3_CURR_X_COUNT", 16, 0xFFC00CF0);
		d("DMA2_3_CURR_Y_COUNT", 16, 0xFFC00CF8);
		d("DMA2_3_IRQ_STATUS", 16, 0xFFC00CE8);
		d("DMA2_3_NEXT_DESC_PTR", 32, 0xFFC00CC0);
		d("DMA2_3_PERIPHERAL_MAP", 16, 0xFFC00CEC);
		d("DMA2_3_START_ADDR", 32, 0xFFC00CC4);
		d("DMA2_3_X_COUNT", 16, 0xFFC00CD0);
		d("DMA2_3_X_MODIFY", 16, 0xFFC00CD4);
		d("DMA2_3_Y_COUNT", 16, 0xFFC00CD8);
		d("DMA2_3_Y_MODIFY", 16, 0xFFC00CDC);

		parent = debugfs_create_dir("DMA2 Channel 4", top);
		d("DMA2_4_CONFIG", 16, 0xFFC00D08);
		d("DMA2_4_CURR_ADDR", 32, 0xFFC00D24);
		d("DMA2_4_CURR_DESC_PTR", 32, 0xFFC00D20);
		d("DMA2_4_CURR_X_COUNT", 16, 0xFFC00D30);
		d("DMA2_4_CURR_Y_COUNT", 16, 0xFFC00D38);
		d("DMA2_4_IRQ_STATUS", 16, 0xFFC00D28);
		d("DMA2_4_NEXT_DESC_PTR", 32, 0xFFC00D00);
		d("DMA2_4_PERIPHERAL_MAP", 16, 0xFFC00D2C);
		d("DMA2_4_START_ADDR", 32, 0xFFC00D04);
		d("DMA2_4_X_COUNT", 16, 0xFFC00D10);
		d("DMA2_4_X_MODIFY", 16, 0xFFC00D14);
		d("DMA2_4_Y_COUNT", 16, 0xFFC00D18);
		d("DMA2_4_Y_MODIFY", 16, 0xFFC00D1C);

		parent = debugfs_create_dir("DMA2 Channel 5", top);
		d("DMA2_5_CONFIG", 16, 0xFFC00D48);
		d("DMA2_5_CURR_ADDR", 32, 0xFFC00D64);
		d("DMA2_5_CURR_DESC_PTR", 32, 0xFFC00D60);
		d("DMA2_5_CURR_X_COUNT", 16, 0xFFC00D70);
		d("DMA2_5_CURR_Y_COUNT", 16, 0xFFC00D78);
		d("DMA2_5_IRQ_STATUS", 16, 0xFFC00D68);
		d("DMA2_5_NEXT_DESC_PTR", 32, 0xFFC00D40);
		d("DMA2_5_PERIPHERAL_MAP", 16, 0xFFC00D6C);
		d("DMA2_5_START_ADDR", 32, 0xFFC00D44);
		d("DMA2_5_X_COUNT", 16, 0xFFC00D50);
		d("DMA2_5_X_MODIFY", 16, 0xFFC00D54);
		d("DMA2_5_Y_COUNT", 16, 0xFFC00D58);
		d("DMA2_5_Y_MODIFY", 16, 0xFFC00D5C);

		parent = debugfs_create_dir("DMA2 Channel 6", top);
		d("DMA2_6_CONFIG", 16, 0xFFC00D88);
		d("DMA2_6_CURR_ADDR", 32, 0xFFC00DA4);
		d("DMA2_6_CURR_DESC_PTR", 32, 0xFFC00DA0);
		d("DMA2_6_CURR_X_COUNT", 16, 0xFFC00DB0);
		d("DMA2_6_CURR_Y_COUNT", 16, 0xFFC00DB8);
		d("DMA2_6_IRQ_STATUS", 16, 0xFFC00DA8);
		d("DMA2_6_NEXT_DESC_PTR", 32, 0xFFC00D80);
		d("DMA2_6_PERIPHERAL_MAP", 16, 0xFFC00DAC);
		d("DMA2_6_START_ADDR", 32, 0xFFC00D84);
		d("DMA2_6_X_COUNT", 16, 0xFFC00D90);
		d("DMA2_6_X_MODIFY", 16, 0xFFC00D94);
		d("DMA2_6_Y_COUNT", 16, 0xFFC00D98);
		d("DMA2_6_Y_MODIFY", 16, 0xFFC00D9C);

		parent = debugfs_create_dir("DMA2 Channel 7", top);
		d("DMA2_7_CONFIG", 16, 0xFFC00DC8);
		d("DMA2_7_CURR_ADDR", 32, 0xFFC00DE4);
		d("DMA2_7_CURR_DESC_PTR", 32, 0xFFC00DE0);
		d("DMA2_7_CURR_X_COUNT", 16, 0xFFC00DF0);
		d("DMA2_7_CURR_Y_COUNT", 16, 0xFFC00DF8);
		d("DMA2_7_IRQ_STATUS", 16, 0xFFC00DE8);
		d("DMA2_7_NEXT_DESC_PTR", 32, 0xFFC00DC0);
		d("DMA2_7_PERIPHERAL_MAP", 16, 0xFFC00DEC);
		d("DMA2_7_START_ADDR", 32, 0xFFC00DC4);
		d("DMA2_7_X_COUNT", 16, 0xFFC00DD0);
		d("DMA2_7_X_MODIFY", 16, 0xFFC00DD4);
		d("DMA2_7_Y_COUNT", 16, 0xFFC00DD8);
		d("DMA2_7_Y_MODIFY", 16, 0xFFC00DDC);

		parent = debugfs_create_dir("DMA2 Channel 8", top);
		d("DMA2_8_CONFIG", 16, 0xFFC00E08);
		d("DMA2_8_CURR_ADDR", 32, 0xFFC00E24);
		d("DMA2_8_CURR_DESC_PTR", 32, 0xFFC00E20);
		d("DMA2_8_CURR_X_COUNT", 16, 0xFFC00E30);
		d("DMA2_8_CURR_Y_COUNT", 16, 0xFFC00E38);
		d("DMA2_8_IRQ_STATUS", 16, 0xFFC00E28);
		d("DMA2_8_NEXT_DESC_PTR", 32, 0xFFC00E00);
		d("DMA2_8_PERIPHERAL_MAP", 16, 0xFFC00E2C);
		d("DMA2_8_START_ADDR", 32, 0xFFC00E04);
		d("DMA2_8_X_COUNT", 16, 0xFFC00E10);
		d("DMA2_8_X_MODIFY", 16, 0xFFC00E14);
		d("DMA2_8_Y_COUNT", 16, 0xFFC00E18);
		d("DMA2_8_Y_MODIFY", 16, 0xFFC00E1C);

		parent = debugfs_create_dir("DMA2 Channel 9", top);
		d("DMA2_9_CONFIG", 16, 0xFFC00E48);
		d("DMA2_9_CURR_ADDR", 32, 0xFFC00E64);
		d("DMA2_9_CURR_DESC_PTR", 32, 0xFFC00E60);
		d("DMA2_9_CURR_X_COUNT", 16, 0xFFC00E70);
		d("DMA2_9_CURR_Y_COUNT", 16, 0xFFC00E78);
		d("DMA2_9_IRQ_STATUS", 16, 0xFFC00E68);
		d("DMA2_9_NEXT_DESC_PTR", 32, 0xFFC00E40);
		d("DMA2_9_PERIPHERAL_MAP", 16, 0xFFC00E6C);
		d("DMA2_9_START_ADDR", 32, 0xFFC00E44);
		d("DMA2_9_X_COUNT", 16, 0xFFC00E50);
		d("DMA2_9_X_MODIFY", 16, 0xFFC00E54);
		d("DMA2_9_Y_COUNT", 16, 0xFFC00E58);
		d("DMA2_9_Y_MODIFY", 16, 0xFFC00E5C);

		parent = debugfs_create_dir("Flag 0", top);
		d("FIO0_BOTH", 16, 0xFFC0073C);
		d("FIO0_DIR", 16, 0xFFC00730);
		d("FIO0_EDGE", 16, 0xFFC00738);
		d("FIO0_FLAG_C", 16, 0xFFC00704);
		d("FIO0_FLAG_D", 16, 0xFFC00700);
		d("FIO0_FLAG_S", 16, 0xFFC00708);
		d("FIO0_FLAG_T", 16, 0xFFC0070C);
		d("FIO0_INEN", 16, 0xFFC00740);
		d("FIO0_MASKA_C", 16, 0xFFC00714);
		d("FIO0_MASKA_D", 16, 0xFFC00710);
		d("FIO0_MASKA_S", 16, 0xFFC00718);
		d("FIO0_MASKA_T", 16, 0xFFC0071C);
		d("FIO0_MASKB_C", 16, 0xFFC00724);
		d("FIO0_MASKB_D", 16, 0xFFC00720);
		d("FIO0_MASKB_S", 16, 0xFFC00728);
		d("FIO0_MASKB_T", 16, 0xFFC0072C);
		d("FIO0_POLAR", 16, 0xFFC00734);

		parent = debugfs_create_dir("Flag 1", top);
		d("FIO1_BOTH", 16, 0xFFC0153C);
		d("FIO1_DIR", 16, 0xFFC01530);
		d("FIO1_EDGE", 16, 0xFFC01538);
		d("FIO1_FLAG_C", 16, 0xFFC01504);
		d("FIO1_FLAG_D", 16, 0xFFC01500);
		d("FIO1_FLAG_S", 16, 0xFFC01508);
		d("FIO1_FLAG_T", 16, 0xFFC0150C);
		d("FIO1_INEN", 16, 0xFFC01540);
		d("FIO1_MASKA_C", 16, 0xFFC01514);
		d("FIO1_MASKA_D", 16, 0xFFC01510);
		d("FIO1_MASKA_S", 16, 0xFFC01518);
		d("FIO1_MASKA_T", 16, 0xFFC0151C);
		d("FIO1_MASKB_C", 16, 0xFFC01524);
		d("FIO1_MASKB_D", 16, 0xFFC01520);
		d("FIO1_MASKB_S", 16, 0xFFC01528);
		d("FIO1_MASKB_T", 16, 0xFFC0152C);
		d("FIO1_POLAR", 16, 0xFFC01534);

		parent = debugfs_create_dir("Flag 2", top);
		d("FIO2_BOTH", 16, 0xFFC0173C);
		d("FIO2_DIR", 16, 0xFFC01730);
		d("FIO2_EDGE", 16, 0xFFC01738);
		d("FIO2_FLAG_C", 16, 0xFFC01704);
		d("FIO2_FLAG_D", 16, 0xFFC01700);
		d("FIO2_FLAG_S", 16, 0xFFC01708);
		d("FIO2_FLAG_T", 16, 0xFFC0170C);
		d("FIO2_INEN", 16, 0xFFC01740);
		d("FIO2_MASKA_C", 16, 0xFFC01714);
		d("FIO2_MASKA_D", 16, 0xFFC01710);
		d("FIO2_MASKA_S", 16, 0xFFC01718);
		d("FIO2_MASKA_T", 16, 0xFFC0171C);
		d("FIO2_MASKB_C", 16, 0xFFC01724);
		d("FIO2_MASKB_D", 16, 0xFFC01720);
		d("FIO2_MASKB_S", 16, 0xFFC01728);
		d("FIO2_MASKB_T", 16, 0xFFC0172C);
		d("FIO2_POLAR", 16, 0xFFC01734);

		parent = debugfs_create_dir("IMDMA Destination Channel-0", top);
		d("IMDMA_D0_CONFIG", 16, 0xFFC01808);
		d("IMDMA_D0_CURR_ADDR", 32, 0xFFC01824);
		d("IMDMA_D0_CURR_DESC_PTR", 32, 0xFFC01820);
		d("IMDMA_D0_CURR_X_COUNT", 16, 0xFFC01830);
		d("IMDMA_D0_CURR_Y_COUNT", 16, 0xFFC01838);
		d("IMDMA_D0_IRQ_STATUS", 16, 0xFFC01828);
		d("IMDMA_D0_NEXT_DESC_PTR", 32, 0xFFC01800);
		d("IMDMA_D0_START_ADDR", 32, 0xFFC01804);
		d("IMDMA_D0_X_COUNT", 16, 0xFFC01810);
		d("IMDMA_D0_X_MODIFY", 16, 0xFFC01814);
		d("IMDMA_D0_Y_COUNT", 16, 0xFFC01818);
		d("IMDMA_D0_Y_MODIFY", 16, 0xFFC0181C);

		parent = debugfs_create_dir("IMDMA Destination Channel-1", top);
		d("IMDMA_D1_CONFIG", 16, 0xFFC01888);
		d("IMDMA_D1_CURR_ADDR", 32, 0xFFC018A4);
		d("IMDMA_D1_CURR_DESC_PTR", 32, 0xFFC018A0);
		d("IMDMA_D1_CURR_X_COUNT", 16, 0xFFC018B0);
		d("IMDMA_D1_CURR_Y_COUNT", 16, 0xFFC018B8);
		d("IMDMA_D1_IRQ_STATUS", 16, 0xFFC018A8);
		d("IMDMA_D1_NEXT_DESC_PTR", 32, 0xFFC01880);
		d("IMDMA_D1_START_ADDR", 32, 0xFFC01884);
		d("IMDMA_D1_X_COUNT", 16, 0xFFC01890);
		d("IMDMA_D1_X_MODIFY", 16, 0xFFC01894);
		d("IMDMA_D1_Y_COUNT", 16, 0xFFC01898);
		d("IMDMA_D1_Y_MODIFY", 16, 0xFFC0189C);

		parent = debugfs_create_dir("IMDMA Source Channel-0", top);
		d("IMDMA_S0_CONFIG", 16, 0xFFC01848);
		d("IMDMA_S0_CURR_ADDR", 32, 0xFFC01864);
		d("IMDMA_S0_CURR_DESC_PTR", 32, 0xFFC01860);
		d("IMDMA_S0_CURR_X_COUNT", 16, 0xFFC01870);
		d("IMDMA_S0_CURR_Y_COUNT", 16, 0xFFC01878);
		d("IMDMA_S0_IRQ_STATUS", 16, 0xFFC01868);
		d("IMDMA_S0_NEXT_DESC_PTR", 32, 0xFFC01840);
		d("IMDMA_S0_START_ADDR", 32, 0xFFC01844);
		d("IMDMA_S0_X_COUNT", 16, 0xFFC01850);
		d("IMDMA_S0_X_MODIFY", 16, 0xFFC01854);
		d("IMDMA_S0_Y_COUNT", 16, 0xFFC01858);
		d("IMDMA_S0_Y_MODIFY", 16, 0xFFC0185C);

		parent = debugfs_create_dir("IMDMA Source Channel-1", top);
		d("IMDMA_S1_CONFIG", 16, 0xFFC018C8);
		d("IMDMA_S1_CURR_ADDR", 32, 0xFFC018E4);
		d("IMDMA_S1_CURR_DESC_PTR", 32, 0xFFC018E0);
		d("IMDMA_S1_CURR_X_COUNT", 16, 0xFFC018F0);
		d("IMDMA_S1_CURR_Y_COUNT", 16, 0xFFC018F8);
		d("IMDMA_S1_IRQ_STATUS", 16, 0xFFC018E8);
		d("IMDMA_S1_NEXT_DESC_PTR", 32, 0xFFC018C0);
		d("IMDMA_S1_START_ADDR", 32, 0xFFC018C4);
		d("IMDMA_S1_X_COUNT", 16, 0xFFC018D0);
		d("IMDMA_S1_X_MODIFY", 16, 0xFFC018D4);
		d("IMDMA_S1_Y_COUNT", 16, 0xFFC018D8);
		d("IMDMA_S1_Y_MODIFY", 16, 0xFFC018DC);

		parent = debugfs_create_dir("MDMA1_0 Destination", top);
		d("MDMA1_D0_CONFIG", 16, 0xFFC01F08);
		d("MDMA1_D0_CURR_ADDR", 32, 0xFFC01F24);
		d("MDMA1_D0_CURR_DESC_PTR", 32, 0xFFC01F20);
		d("MDMA1_D0_CURR_X_COUNT", 16, 0xFFC01F30);
		d("MDMA1_D0_CURR_Y_COUNT", 16, 0xFFC01F38);
		d("MDMA1_D0_IRQ_STATUS", 16, 0xFFC01F28);
		d("MDMA1_D0_NEXT_DESC_PTR", 32, 0xFFC01F00);
		d("MDMA1_D0_PERIPHERAL_MAP", 16, 0xFFC01F2C);
		d("MDMA1_D0_START_ADDR", 32, 0xFFC01F04);
		d("MDMA1_D0_X_COUNT", 16, 0xFFC01F10);
		d("MDMA1_D0_X_MODIFY", 16, 0xFFC01F14);
		d("MDMA1_D0_Y_COUNT", 16, 0xFFC01F18);
		d("MDMA1_D0_Y_MODIFY", 16, 0xFFC01F1C);

		parent = debugfs_create_dir("MDMA1_0 Source", top);
		d("MDMA1_S0_CONFIG", 16, 0xFFC01F48);
		d("MDMA1_S0_CURR_ADDR", 32, 0xFFC01F64);
		d("MDMA1_S0_CURR_DESC_PTR", 32, 0xFFC01F60);
		d("MDMA1_S0_CURR_X_COUNT", 16, 0xFFC01F70);
		d("MDMA1_S0_CURR_Y_COUNT", 16, 0xFFC01F78);
		d("MDMA1_S0_IRQ_STATUS", 16, 0xFFC01F68);
		d("MDMA1_S0_NEXT_DESC_PTR", 32, 0xFFC01F40);
		d("MDMA1_S0_PERIPHERAL_MAP", 16, 0xFFC01F6C);
		d("MDMA1_S0_START_ADDR", 32, 0xFFC01F44);
		d("MDMA1_S0_X_COUNT", 16, 0xFFC01F50);
		d("MDMA1_S0_X_MODIFY", 16, 0xFFC01F54);
		d("MDMA1_S0_Y_COUNT", 16, 0xFFC01F58);
		d("MDMA1_S0_Y_MODIFY", 16, 0xFFC01F5C);

		parent = debugfs_create_dir("MDMA1_1 Destination", top);
		d("MDMA1_D1_CONFIG", 16, 0xFFC01F88);
		d("MDMA1_D1_CURR_ADDR", 32, 0xFFC01FA4);
		d("MDMA1_D1_CURR_DESC_PTR", 32, 0xFFC01FA0);
		d("MDMA1_D1_CURR_X_COUNT", 16, 0xFFC01FB0);
		d("MDMA1_D1_CURR_Y_COUNT", 16, 0xFFC01FB8);
		d("MDMA1_D1_IRQ_STATUS", 16, 0xFFC01FA8);
		d("MDMA1_D1_NEXT_DESC_PTR", 32, 0xFFC01F80);
		d("MDMA1_D1_PERIPHERAL_MAP", 16, 0xFFC01FAC);
		d("MDMA1_D1_START_ADDR", 32, 0xFFC01F84);
		d("MDMA1_D1_X_COUNT", 16, 0xFFC01F90);
		d("MDMA1_D1_X_MODIFY", 16, 0xFFC01F94);
		d("MDMA1_D1_Y_COUNT", 16, 0xFFC01F98);
		d("MDMA1_D1_Y_MODIFY", 16, 0xFFC01F9C);

		parent = debugfs_create_dir("MDMA1_1 Source", top);
		d("MDMA1_S1_CONFIG", 16, 0xFFC01FC8);
		d("MDMA1_S1_CURR_ADDR", 32, 0xFFC01FE4);
		d("MDMA1_S1_CURR_DESC_PTR", 32, 0xFFC01FE0);
		d("MDMA1_S1_CURR_X_COUNT", 16, 0xFFC01FF0);
		d("MDMA1_S1_CURR_Y_COUNT", 16, 0xFFC01FF8);
		d("MDMA1_S1_IRQ_STATUS", 16, 0xFFC01FE8);
		d("MDMA1_S1_NEXT_DESC_PTR", 32, 0xFFC01FC0);
		d("MDMA1_S1_PERIPHERAL_MAP", 16, 0xFFC01FEC);
		d("MDMA1_S1_START_ADDR", 32, 0xFFC01FC4);
		d("MDMA1_S1_X_COUNT", 16, 0xFFC01FD0);
		d("MDMA1_S1_X_MODIFY", 16, 0xFFC01FD4);
		d("MDMA1_S1_Y_COUNT", 16, 0xFFC01FD8);
		d("MDMA1_S1_Y_MODIFY", 16, 0xFFC01FDC);

		parent = debugfs_create_dir("MDMA2_0 Destination", top);
		d("MDMA2_D0_CONFIG", 16, 0xFFC00F08);
		d("MDMA2_D0_CURR_ADDR", 32, 0xFFC00F24);
		d("MDMA2_D0_CURR_DESC_PTR", 32, 0xFFC00F20);
		d("MDMA2_D0_CURR_X_COUNT", 16, 0xFFC00F30);
		d("MDMA2_D0_CURR_Y_COUNT", 16, 0xFFC00F38);
		d("MDMA2_D0_IRQ_STATUS", 16, 0xFFC00F28);
		d("MDMA2_D0_NEXT_DESC_PTR", 32, 0xFFC00F00);
		d("MDMA2_D0_PERIPHERAL_MAP", 16, 0xFFC00F2C);
		d("MDMA2_D0_START_ADDR", 32, 0xFFC00F04);
		d("MDMA2_D0_X_COUNT", 16, 0xFFC00F10);
		d("MDMA2_D0_X_MODIFY", 16, 0xFFC00F14);
		d("MDMA2_D0_Y_COUNT", 16, 0xFFC00F18);
		d("MDMA2_D0_Y_MODIFY", 16, 0xFFC00F1C);

		parent = debugfs_create_dir("MDMA2_0 Source", top);
		d("MDMA2_S0_CONFIG", 16, 0xFFC00F48);
		d("MDMA2_S0_CURR_ADDR", 32, 0xFFC00F64);
		d("MDMA2_S0_CURR_DESC_PTR", 32, 0xFFC00F60);
		d("MDMA2_S0_CURR_X_COUNT", 16, 0xFFC00F70);
		d("MDMA2_S0_CURR_Y_COUNT", 16, 0xFFC00F78);
		d("MDMA2_S0_IRQ_STATUS", 16, 0xFFC00F68);
		d("MDMA2_S0_NEXT_DESC_PTR", 32, 0xFFC00F40);
		d("MDMA2_S0_PERIPHERAL_MAP", 16, 0xFFC00F6C);
		d("MDMA2_S0_START_ADDR", 32, 0xFFC00F44);
		d("MDMA2_S0_X_COUNT", 16, 0xFFC00F50);
		d("MDMA2_S0_X_MODIFY", 16, 0xFFC00F54);
		d("MDMA2_S0_Y_COUNT", 16, 0xFFC00F58);
		d("MDMA2_S0_Y_MODIFY", 16, 0xFFC00F5C);

		parent = debugfs_create_dir("MDMA2_1 Destination", top);
		d("MDMA2_D1_CONFIG", 16, 0xFFC00F88);
		d("MDMA2_D1_CURR_ADDR", 32, 0xFFC00FA4);
		d("MDMA2_D1_CURR_DESC_PTR", 32, 0xFFC00FA0);
		d("MDMA2_D1_CURR_X_COUNT", 16, 0xFFC00FB0);
		d("MDMA2_D1_CURR_Y_COUNT", 16, 0xFFC00FB8);
		d("MDMA2_D1_IRQ_STATUS", 16, 0xFFC00FA8);
		d("MDMA2_D1_NEXT_DESC_PTR", 32, 0xFFC00F80);
		d("MDMA2_D1_PERIPHERAL_MAP", 16, 0xFFC00FAC);
		d("MDMA2_D1_START_ADDR", 32, 0xFFC00F84);
		d("MDMA2_D1_X_COUNT", 16, 0xFFC00F90);
		d("MDMA2_D1_X_MODIFY", 16, 0xFFC00F94);
		d("MDMA2_D1_Y_COUNT", 16, 0xFFC00F98);
		d("MDMA2_D1_Y_MODIFY", 16, 0xFFC00F9C);

		parent = debugfs_create_dir("MDMA2_1 Source", top);
		d("MDMA2_S1_CONFIG", 16, 0xFFC00FC8);
		d("MDMA2_S1_CURR_ADDR", 32, 0xFFC00FE4);
		d("MDMA2_S1_CURR_DESC_PTR", 32, 0xFFC00FE0);
		d("MDMA2_S1_CURR_X_COUNT", 16, 0xFFC00FF0);
		d("MDMA2_S1_CURR_Y_COUNT", 16, 0xFFC00FF8);
		d("MDMA2_S1_IRQ_STATUS", 16, 0xFFC00FE8);
		d("MDMA2_S1_NEXT_DESC_PTR", 32, 0xFFC00FC0);
		d("MDMA2_S1_PERIPHERAL_MAP", 16, 0xFFC00FEC);
		d("MDMA2_S1_START_ADDR", 32, 0xFFC00FC4);
		d("MDMA2_S1_X_COUNT", 16, 0xFFC00FD0);
		d("MDMA2_S1_X_MODIFY", 16, 0xFFC00FD4);
		d("MDMA2_S1_Y_COUNT", 16, 0xFFC00FD8);
		d("MDMA2_S1_Y_MODIFY", 16, 0xFFC00FDC);

		parent = debugfs_create_dir("SPI", top);
		d("SPI_BAUD", 16, 0xFFC00514);
		d("SPI_CTL", 16, 0xFFC00500);
		d("SPI_FLG", 16, 0xFFC00504);
		d("SPI_RDBR", 16, 0xFFC00510);
		d("SPI_SHADOW", 16, 0xFFC00518);
		d("SPI_STAT", 16, 0xFFC00508);
		d("SPI_TDBR", 16, 0xFFC0050C);

		parent = debugfs_create_dir("UART", top);
		d("UART_DLH", 16, 0xFFC00404);
		d("UART_DLL", 16, 0xFFC00400);
		d("UART_GBL", 16, 0xFFC00424);
		d("UART_GCTL", 16, 0xFFC00424);
		d("UART_IER", 16, 0xFFC00404);
		d("UART_IIR", 16, 0xFFC00408);
		d("UART_LCR", 16, 0xFFC0040C);
		d("UART_LSR", 16, 0xFFC00414);
		d("UART_MCR", 16, 0xFFC00410);
		d("UART_MSR", 16, 0xFFC00418);
		d("UART_RBR", 16, 0xFFC00400);
		d("UART_SCR", 16, 0xFFC0041C);
		d("UART_THR", 16, 0xFFC00400);

	}	/* BF561 */

	debug_mmrs_dentry = top;

	return 0;
}
module_init(bfin_debug_mmrs_init);

static void __exit bfin_debug_mmrs_exit(void)
{
	debugfs_remove_recursive(debug_mmrs_dentry);
}
module_exit(bfin_debug_mmrs_exit);

MODULE_LICENSE("GPL");
