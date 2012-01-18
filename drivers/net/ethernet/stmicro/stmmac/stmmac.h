/*******************************************************************************
  Copyright (C) 2007-2009  STMicroelectronics Ltd

  This program is free software; you can redistribute it and/or modify it
  under the terms and conditions of the GNU General Public License,
  version 2, as published by the Free Software Foundation.

  This program is distributed in the hope it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
  more details.

  You should have received a copy of the GNU General Public License along with
  this program; if not, write to the Free Software Foundation, Inc.,
  51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.

  The full GNU General Public License is included in this distribution in
  the file called "COPYING".

  Author: Giuseppe Cavallaro <peppe.cavallaro@st.com>
*******************************************************************************/

#define STMMAC_RESOURCE_NAME   "stmmaceth"
#define DRV_MODULE_VERSION	"Feb_2012"
#include <linux/stmmac.h>
#include <linux/phy.h>
#include "common.h"
#ifdef CONFIG_STMMAC_TIMER
#include "stmmac_timer.h"
#endif

#ifdef CONFIG_STMMAC_IEEE1588
#include <linux/net_tstamp.h>
#include <linux/clocksource.h>
#include <linux/timecompare.h>
#include <linux/timer.h>

#define PTP_EN          (0x1)        /* Enable the PTP_TSYNC module */
#define PTP_TSINIT      (0x4)        /* update system timer */
#define PTP_TSENALL     (1 << 8)
#define PTP_TSVER2ENA   (1 << 10)
#define PTP_TSIPENA     (1 << 11)
#define PTP_TSIPV4ENA   (1 << 13)
#define PTP_TSEVENTENA  (1 << 14)
#define PTP_TSMASTERENA (1 << 15)
#define PTP_SNAPTYPESEL (1 << 16)

#define EMAC_TM_CTL                0x700         /* EMAC0 EMAC Time Stamp Control Register */
#define EMAC_TM_SUBSEC             0x704         /* EMAC0 EMAC Time Stamp Sub Second Increment */
#define EMAC_TM_SEC                0x708         /* EMAC0 EMAC Time Stamp Second Register */
#define EMAC_TM_NSEC               0x70C         /* EMAC0 EMAC Time Stamp Nano Second Register */
#define EMAC_TM_SECUPDT            0x710         /* EMAC0 EMAC Time Stamp Seconds Update */
#define EMAC_TM_NSECUPDT           0x714         /* EMAC0 EMAC Time Stamp Nano Seconds Update */
#define EMAC_TM_ADDEND             0x718         /* EMAC0 EMAC Time Stamp Addend Register */
#define EMAC_TM_TGTM               0x71C         /* EMAC0 EMAC Time Stamp Target Time Sec. */
#define EMAC_TM_NTGTM              0x720         /* EMAC0 EMAC Time Stamp Target Time Nanosec. */
#define EMAC_TM_HISEC              0x724         /* EMAC0 EMAC Time Stamp High Second Register */
#define EMAC_TM_STMPSTAT           0x728         /* EMAC0 EMAC Time Stamp Status Register */
#define EMAC_TM_PPSCTL             0x72C         /* EMAC0 EMAC PPS Control Register */

#define PADS_EMAC_PTP_CLKSEL 	   0xFFC03404
#endif

struct stmmac_priv {
	/* Frequently used values are kept adjacent for cache effect */
	struct dma_desc *dma_tx ____cacheline_aligned;
	dma_addr_t dma_tx_phy;
	struct sk_buff **tx_skbuff;
	unsigned int cur_tx;
	unsigned int dirty_tx;
	unsigned int dma_tx_size;
	int tx_coalesce;

	struct dma_desc *dma_rx ;
	unsigned int cur_rx;
	unsigned int dirty_rx;
	struct sk_buff **rx_skbuff;
	dma_addr_t *rx_skbuff_dma;
	struct sk_buff_head rx_recycle;

	struct net_device *dev;
	dma_addr_t dma_rx_phy;
	unsigned int dma_rx_size;
	unsigned int dma_buf_sz;
	struct device *device;
	struct mac_device_info *hw;
	void __iomem *ioaddr;

	struct stmmac_extra_stats xstats;
	struct napi_struct napi;

	int rx_coe;
	int no_csum_insertion;

	struct phy_device *phydev;
	int oldlink;
	int speed;
	int oldduplex;
	unsigned int flow_ctrl;
	unsigned int pause;
	struct mii_bus *mii;
	int mii_irq[PHY_MAX_ADDR];

	u32 msg_enable;
	spinlock_t lock;
	spinlock_t tx_lock;
	int wolopts;
	int wol_irq;
#ifdef CONFIG_STMMAC_TIMER
	struct stmmac_timer *tm;
#endif
	struct plat_stmmacenet_data *plat;
	struct stmmac_counters mmc;
	struct dma_features dma_cap;
	int hw_cap_support;
#ifdef CONFIG_STMMAC_IEEE1588
	struct cyclecounter cycles;
	struct timecounter clock;
	struct timecompare compare;
	struct hwtstamp_config stamp_cfg;
#endif
};

extern int phyaddr;

extern int stmmac_mdio_unregister(struct net_device *ndev);
extern int stmmac_mdio_register(struct net_device *ndev);
extern void stmmac_set_ethtool_ops(struct net_device *netdev);
extern const struct stmmac_desc_ops enh_desc_ops;
extern const struct stmmac_desc_ops ndesc_ops;

int stmmac_freeze(struct net_device *ndev);
int stmmac_restore(struct net_device *ndev);
int stmmac_resume(struct net_device *ndev);
int stmmac_suspend(struct net_device *ndev);
int stmmac_dvr_remove(struct net_device *ndev);
struct stmmac_priv *stmmac_dvr_probe(struct device *device,
				     struct plat_stmmacenet_data *plat_dat,
				     void __iomem *addr);
