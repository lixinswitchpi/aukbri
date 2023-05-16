/*
 * SwitchPi AUK BRI2 module Driver for DAHDI Telephony interface.
 * This driver is developed to support SwitchPi AUK BRI2 2 ports BRI board only,
 * you can use it freely, but there is no warranty as it is.
 * Written by Li Yuqian <yuqian.li@switchpi.com>
 *
 * Copyright (C) 2017-2023, SwitchPi, Inc.
 *
 * All rights reserved.
 *
 */
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#ifdef CONFIG_POST
#include <linux/post.h>
#endif
#include <dahdi/kernel.h>
#include <linux/seq_file.h>

#include "xhfc24succ.h"

// #define USE_TASKLET	0

#ifndef USE_TASKLET
#include <linux/workqueue.h>
#endif

/* xhfc board defines */
#define MAX_XHFC_SPANS 4 /* This is per chip */
#define CHAN_PER_SPAN 4	 /* D, B1, B2, PCM */
#define MAX_CHAN (MAX_XHFC_SPANS * CHAN_PER_SPAN)

#define MAX_SPANS 4

/* flags in _u16  span mode */
#define SPAN_UNUSED 0x0000
#define SPAN_MODE_NT 0x0001
#define SPAN_MODE_TE 0x0002
#define SPAN_MODE_S0 0x0004
#define SPAN_MODE_UP 0x0008
#define SPAN_MODE_EXCH_POL 0x0010
#define SPAN_MODE_LOOP_B1 0x0020
#define SPAN_MODE_LOOP_B2 0x0040
#define SPAN_MODE_LOOP_D 0x0080
#define SPAN_MODE_ENDPOINT 0x0100
#define SPAN_MODE_STARTED 0x1000

#define SPAN_MODE_LOOPS 0xE0 /* mask span mode Loop B1/B2/D */

/* NT / TE defines */
#define CLK_DLY_TE 0x0e		/* CLKDEL in TE mode */
#define CLK_DLY_NT 0x6c		/* CLKDEL in NT mode */
#define STA_ACTIVATE 0x60	/* start activation   in A_SU_WR_STA */
#define STA_DEACTIVATE 0x40 /* start deactivation in A_SU_WR_STA */
#define XHFC_TIMER_T3 2000	/* 2s activation timer T3 */
#define XHFC_TIMER_T4 500	/* 500ms deactivation timer T4 */

/* xhfc Layer1 Flags (stored in xhfc_port_t->l1_flags) */
#define HFC_L1_ACTIVATING 1
#define HFC_L1_ACTIVATED 2

#define BRIE_CHANS_PER_SPAN 3

#define FRAMES_PER_BUFFER 320 /* 40ms / (125us per frame) */
#define FRAMES_PER_TRANSFER 8
#define FRAMESIZE 64
#define DAHDI_BRI_CARD_ID 7

static int brie_spanconfig(struct file *file, struct dahdi_span *span, struct dahdi_lineconfig *lc);

static int brie_chanconfig(struct file *file, struct dahdi_chan *chan, int sigtype);
static int brie_startup(struct file *file, struct dahdi_span *span);
static int brie_shutdown(struct dahdi_span *span);
static int brie_open(struct dahdi_chan *chan);
static int brie_close(struct dahdi_chan *chan);
static int brie_ioctl(struct dahdi_chan *chan, unsigned int cmd, unsigned long data);
static void brie_hdlc_hard_xmit(struct dahdi_chan *chan);

static const struct dahdi_span_ops auk_bri_span_ops = {
	.owner = THIS_MODULE,
	.spanconfig = brie_spanconfig,
	.chanconfig = brie_chanconfig,
	.startup = brie_startup,
	.shutdown = brie_shutdown,
	.open = brie_open,
	.close = brie_close,
	.ioctl = brie_ioctl,
	.hdlc_hard_xmit = brie_hdlc_hard_xmit,
};

/* span struct for each S/U span */
struct xhfc_span
{
	int span_id; /* Physical span id */
	int id;		 /* 0 based, no gaps */
	int modidx;
	struct xhfc *xhfc;
	struct xhfc_chan *d_chan;
	struct xhfc_chan *b1_chan;
	struct xhfc_chan *b2_chan;

	int timeslot;

	atomic_t open_count;

	/* hdlc transmit data */
	int tx_idx;
	int tx_size;
	int tx_frame;
	u8 tx_buf[128];

	/* hdlc receive data */
	int rx_idx;
	u8 rx_buf[128];

	u16 mode; /* NT/TE + ST/U */
	u8 state;
	u_long l1_flags;
	struct timer_list t3_timer; /* for activation/deactivation */
	struct timer_list t4_timer; /* for activation/deactivation */
	struct timer_list t1_timer;

	/* Alarm state for dahdi */
	int newalarm;
	struct timer_list alarm_timer;

	/* chip registers */
	reg_a_su_ctrl0 su_ctrl0;
	reg_a_su_ctrl1 su_ctrl1;
	reg_a_su_ctrl2 su_ctrl2;

	/* dahdi */
	struct dahdi_span span;
	struct dahdi_chan *chans[BRIE_CHANS_PER_SPAN]; /* Individual channels */
	struct dahdi_chan *sigchan;					   /* the signaling channel for this span */
};

/* channel struct for each fifo */
struct xhfc_chan
{
	int id;
	struct xhfc_span *span;

	unsigned char writechunk[DAHDI_CHUNKSIZE];
	unsigned char readchunk[DAHDI_CHUNKSIZE];

	struct dahdi_chan chan;
};

struct xhfc
{
	__u8 chipnum;	  /* global chip number */
	__u8 modidx;	  /* module index 0 = mod a, 1 = mod b */
	struct brie *dev; /* back pointer to g_brie */

	int num_spans; /* number of S and U interfaces */
	int max_fifo;  /* always 4 fifos per span */
	__u8 max_z;	   /* fifo depth -1 */

	struct xhfc_chan *chan; /* one each D/B/PCM channel */

	/* chip registers */
	reg_r_irq_ctrl irq_ctrl;
	reg_r_misc_irqmsk misc_irqmask; /* mask of enabled interrupt
					   sources */
	reg_r_misc_irq misc_irq;		/* collect interrupt status
						   bits */

	reg_r_su_irqmsk su_irqmask; /* mask of line interface
					   state change interrupts */
	reg_r_su_irq su_irq;		/* collect interrupt status
					   bits */
	__u32 fifo_irq;				/* fifo bl irq */
	__u32 fifo_irqmask;			/* fifo bl irq */

	/* Debugging */
	u8 r_af0_oview;
	u8 r_bert_sta;
	u32 rx_fifo_errs;
	u32 tx_fifo_errs;

	struct dahdi_device *ddev;

	spinlock_t fifo_tx_lock; /* held when accessing anything to affection the fifo */
	spinlock_t fifo_lock;	 /* held when accessing anything to affection the fifo */
							 // spinlock_t tasklet_lock;
};

struct brie
{
	void __iomem *fpga;
	int irq;

	int loopback;

	int pcm_master;
#ifdef USE_TASKLET
	struct tasklet_struct brie_bh;
#else
	struct work_struct brie_work;
#endif
	u8 moda;
	u8 modb;
	u8 num_xhfcs;
	struct xhfc *xhfc;

	struct xhfc_span *spans[MAX_SPANS];

	/* DMA */
	void *tx_buf;
	void *rx_buf;

	u32 *txfifo;
	u32 *rxfifo;

	struct auk_fpga *dma;
	struct device *dev;
	spinlock_t tasklet_lock;
	spinlock_t workqueue_lock;
};

static struct workqueue_struct *brie_wq;

static int debug = 0;
module_param(debug, int, 0664);

extern u32 *get_txfifo(void);
extern u32 *get_rxfifo(void);
extern void xhfc_write(unsigned char add, unsigned char val, int cs);
extern unsigned char xhfc_read(unsigned char add, int cs);

static inline __u8 read_xhfc(struct xhfc *xhfc, __u8 reg)
{
	__u8 data = 0;
	data = xhfc_read(reg, 0);
	return data;
}

static inline void write_xhfc(struct xhfc *xhfc, __u8 reg, __u8 value)
{
	xhfc_write(reg, value, 0);
}

static inline void xhfc_waitbusy(struct xhfc *xhfc)
{
	unsigned long start;
	int timeout = HZ; /* 1s */

	start = jiffies;
	while (read_xhfc(xhfc, R_STATUS) & M_BUSY)
	{
		if (time_after(jiffies, start + timeout))
		{
			printk(KERN_ERR "xhfc_waitbusy: timeout waiting for R_STATUS to not be busy,  R_STATUS: 0x%02x\n", read_xhfc(xhfc, R_STATUS));
			return;
		}
		cpu_relax();
	}
}

static inline void xhfc_selfifo(struct xhfc *xhfc, __u8 fifo)
{
	write_xhfc(xhfc, R_FIFO, fifo);
	xhfc_waitbusy(xhfc);
}

static inline void xhfc_inc_f(struct xhfc *xhfc)
{
	write_xhfc(xhfc, A_INC_RES_FIFO, M_INC_F);
	xhfc_waitbusy(xhfc);
}

static inline __init void xhfc_resetfifo(struct xhfc *xhfc)
{
	write_xhfc(xhfc, A_INC_RES_FIFO, M_RES_FIFO | M_RES_FIFO_ERR);
	xhfc_waitbusy(xhfc);
}

static void xhfc_write_fifo(struct xhfc *xhfc, int ch_index);
static void xhfc_read_fifo(struct xhfc *xhfc, int ch_index);

/* layer 1 specific */
static void l1_activate(struct xhfc_span *span);
static void l1_deactivate(struct xhfc_span *span);
static void l1_timer_start_t1(struct xhfc_span *span);
static void l1_timer_start_t3(struct xhfc_span *span);
static void l1_timer_start_t4(struct xhfc_span *span);
static inline void l1_timer_stop_t1(struct xhfc_span *span);
static inline void l1_timer_stop_t3(struct xhfc_span *span);
static inline void l1_timer_stop_t4(struct xhfc_span *span);
static void l1_timer_expire_t1(struct timer_list *t);
static void l1_timer_expire_t3(struct timer_list *t);
static void l1_timer_expire_t4(struct timer_list *t);

static void alarm_timer_update(struct xhfc_span *span);
static void alarm_timer_expire(struct timer_list *t);

#ifdef USE_TASKLET
static void brie_tasklet(unsigned long arg);
#else
static void brie_worker(struct work_struct *work);
#endif

/* channel specific */
static void brispan_apply_config(struct xhfc_span *span);
static int brichannels_create(struct xhfc_span *span);
static int dchannel_setup_fifo(struct xhfc_chan *chan, unsigned rx);
static void dchannel_toggle_fifo(struct xhfc_chan *chan, u8 enable);
static int bchannel_setup_pcm(struct xhfc_chan *chan, unsigned rx);
static void brispan_new_state(struct xhfc_span *span,
							  u8 new_state, int expired);
static int bchannel_toggle(struct xhfc_chan *chan, u8 enable);
static int brispan_start(struct xhfc_span *span);
static int brispan_stop(struct xhfc_span *span);

static void brie_disable_interrupts(struct brie *dev);
static unsigned nt_mask = 0;
module_param(nt_mask, uint, 0444);

static unsigned endpoint_mask = 0xff;
module_param(endpoint_mask, uint, 0444);

static int teignored = 0;
module_param(teignored, int, 0664);

/* Only one instance of the driver */
static struct brie *g_brie;

/* -------------------------------------------------------------------------- */

/* ModA = 20, ModB = 21 */
static unsigned bri_int_mask;
/* Underrun = 18, Done = 19 */

int total_int_cnt = 0;
int queue_int_cnt = 0;
int su_irq_sta = 0;
int glob_irq = 0;

// use the dma interrupt to process this out,
static int xhfc_interrupt(void) //(int irq, void *arg)
{
	struct brie *dev = g_brie; // arg;
	struct xhfc *xhfc = NULL;
	u8 i, j, reg;
	u32 xhfc_irqs;
	// unsigned long flags;

	// spin_lock_irqsave(&dev->tasklet_lock,flags);
	xhfc_irqs = 0;
	// for (i = 0; i < dev->num_xhfcs; i++) {
	xhfc = &dev->xhfc[0];
	if (xhfc->irq_ctrl.bit.v_glob_irq_en &&
		read_xhfc(xhfc, R_IRQ_OVIEW))
	{
		/* mark this xhfc possibly had irq */
		xhfc_irqs |= (1 << i);
	}
	// }
	glob_irq = xhfc->irq_ctrl.reg;

	if (!xhfc_irqs)
		return -1;

	xhfc_irqs = 0;

	total_int_cnt++;

	// for (i = 0; i < dev->num_xhfcs; i++) {
	xhfc = &dev->xhfc[0];

	reg = read_xhfc(xhfc, R_MISC_IRQ);
	xhfc->misc_irq.reg |= reg;
	if (reg & 1)
	{
		xhfc->r_af0_oview = read_xhfc(xhfc, R_AF0_OVIEW);
		xhfc->r_bert_sta = read_xhfc(xhfc, R_BERT_STA);
	}

	xhfc->su_irq.reg |= read_xhfc(xhfc, R_SU_IRQ);

	/* get fifo IRQ states in bundle */
	for (j = 0; j < 4; j++)
	{
		xhfc->fifo_irq |=
			(read_xhfc(xhfc, R_FIFO_BL0_IRQ + j) << (j * 8));
	}

	/* call bottom half at events
	 *   - Timer Interrupt (or other misc_irq sources)
	 *   - SU State change
	 *   - Fifo FrameEnd interrupts (only at rx fifos enabled)
	 */
	if ((xhfc->misc_irq.reg & xhfc->misc_irqmask.reg) || (xhfc->su_irq.reg & xhfc->su_irqmask.reg) // ){
		|| (xhfc->fifo_irq & xhfc->fifo_irqmask))
	{
		/* mark this xhfc really had irq */
		xhfc_irqs |= (1 << i);

		queue_int_cnt++;
		/* queue bottom half */
#ifdef USE_TASKLET
		tasklet_schedule(&dev->brie_bh);
#else
		// schedule_work(&dev->brie_work);
		queue_work(brie_wq, &dev->brie_work);
#endif
		// brie_tasklet( (unsigned long)dev);
	}
	// }
	// spin_unlock_irqrestore(&dev->tasklet_lock, flags);

	return xhfc_irqs ? 0 : -1;
}

int su_int_cnt = 0;
static inline void handle_su_interrupt(struct xhfc *xhfc)
{
	struct xhfc_chan *dch;
	u8 state;
	int i, span_offset;

	xhfc->su_irq.reg = 0;
	su_int_cnt++;

	span_offset = xhfc->modidx * 4;
	for (i = span_offset; i < (span_offset + xhfc->num_spans); i++)
	{
		struct xhfc_span *span = xhfc->dev->spans[i];
		if (!span)
			continue;

		dch = span->d_chan;

		write_xhfc(xhfc, R_SU_SEL, span->id);
		state = read_xhfc(xhfc, A_SU_RD_STA);

		if (state != span->state)
			brispan_new_state(span, state, 0);
	}
}

int fifo_int_cnt = 0;
int fifo_int_read_cnt = 0;
int fifo_int_write_cnt = 0;
static inline void handle_fifo_interrupt(struct xhfc *xhfc, unsigned fifo_irq)
{
	int i;
	fifo_int_cnt++;
	/* Handle rx fifos */
	for (i = 0; i < xhfc->max_fifo; i++)
	{
		if (fifo_irq & (1 << (i * 2 + 1)))
		{

			xhfc->fifo_irq &= ~(1 << (i * 2 + 1));
			fifo_int_read_cnt++;
			xhfc_read_fifo(xhfc, i);
		}
	}

	for (i = 0; i < xhfc->max_fifo; i++)
	{
		if (fifo_irq & (1 << (i * 2)))
		{
			xhfc->fifo_irq &= ~(1 << (i * 2));
			fifo_int_write_cnt++;
			xhfc_write_fifo(xhfc, i);
		}
	}
}

#ifdef USE_TASKLET
static void brie_tasklet(unsigned long arg)
#else
static void brie_worker(struct work_struct *work)
#endif
{
#ifdef USE_TASKLET
	struct brie *dev = (struct brie *)arg;
#else
	struct brie *dev = container_of(work, struct brie, brie_work);
#endif
	struct xhfc *xhfc;
	unsigned fifo_irq;
	int i;
	unsigned long flags;

	spin_lock_irqsave(&dev->workqueue_lock, flags);

	xhfc = &dev->xhfc[0];
	/* run through once for each xhfc */
	// for (x = 0, xhfc = dev->xhfc; x < dev->num_xhfcs; ++x, ++xhfc) {
	/* set fifo_irq when RX data over threshold */
	for (i = 0; i < xhfc->num_spans; i++)
		xhfc->fifo_irq |=
			read_xhfc(xhfc, R_FILL_BL0 + i) << (i * 8);

	fifo_irq = xhfc->fifo_irq & xhfc->fifo_irqmask;
	if (fifo_irq)
		handle_fifo_interrupt(xhfc, fifo_irq);

	if (xhfc->su_irq.reg & xhfc->su_irqmask.reg)
		handle_su_interrupt(xhfc);
	// }
	spin_unlock_irqrestore(&dev->workqueue_lock, flags);
}

extern int aukdma_register_cb(int cardid, void (*cb)(int cardid, void *arg), void *arg);
extern int aukdma_unregister(int cardid);

int auk_dma_register(void (*cb)(int cardid, void *arg))
{
	return aukdma_register_cb(DAHDI_BRI_CARD_ID, cb, NULL);
}

/* unregister a given dma handler */
int auk_dma_unregister(void)
{
	return aukdma_unregister(DAHDI_BRI_CARD_ID);
}

int auk_dma_enable(void (*dma_handler)(int, void *arg))
{
	return (auk_dma_register(dma_handler));
}

void auk_dma_disable(void (*dma_handler)(int, void *arg))
{
	auk_dma_unregister();
}

int rx_cnt;
int read_chunk;
/* prepare incoming audio data to be received by asterisk */
void bri_receiveprep(struct xhfc_span *span, u32 *readsamples)
{
	volatile unsigned int *readchunk;
	int x;
	readchunk = readsamples;

	rx_cnt++;
	read_chunk = readchunk[0];

	for (x = 0; x < DAHDI_CHUNKSIZE; x++)
	{
		/* Send a sample, as a 32-bit word */
		switch (span->id)
		{
		case 0:
			span->b2_chan->readchunk[x] = (readchunk[x] >> 16) & 0xff;
			span->b1_chan->readchunk[x] = (readchunk[x] >> 24) & 0xff;
			break;
		case 1:
			span->b2_chan->readchunk[x] = (readchunk[x] >> 0) & 0xff;
			span->b1_chan->readchunk[x] = (readchunk[x] >> 8) & 0xff;
			break;
		case 2:
			break;
		case 3:
			break;
		}
	}

	/* this is where echo cancellation is done with the registered
	 * echo canceler
	 */
	dahdi_ec_span(&span->span);

	/* push data into asterisk */
	dahdi_receive(&span->span);
}
int tx_cnt;
int write_chunk;
/* prepare data from asterisk for transmission (put it out on the buffer) */
void bri_transmitprep(struct xhfc_span *span, u32 *writesamples)
{

	volatile unsigned int *writechunk;
	int x;
	/* Write is at interrupt address.  Start writing from normal offset */
	writechunk = writesamples;
	/* Calculate Transmission */
	dahdi_transmit(&span->span);
	tx_cnt++;

	write_chunk = writechunk[0];

	for (x = 0; x < DAHDI_CHUNKSIZE; x++)
	{
		/* Send a sample, as a 32-bit word */
		switch (span->id)
		{
		case 0:
			writechunk[x] = 0;
			writechunk[x] |= (span->b2_chan->writechunk[x] << 16);
			writechunk[x] |= (span->b1_chan->writechunk[x] << 24);
			break;
		case 1:
			writechunk[x] |= (span->b2_chan->writechunk[x] << 0);
			writechunk[x] |= (span->b1_chan->writechunk[x] << 8);
			break;
		case 2:
			break;
		case 3:
			break;
		}
	}
}

int cnt = 0;
static void dahdi_auk_dma_isr(int cardid, void *arg)
{
	int index;

	if (g_brie == NULL)
		return;

	cnt++;
	if ((g_brie->spans[0]->mode & SPAN_MODE_STARTED) & (g_brie->spans[1]->mode & SPAN_MODE_STARTED) & (g_brie->spans[2]->mode & SPAN_MODE_STARTED) & (g_brie->spans[3]->mode & SPAN_MODE_STARTED))
	{
		if ((cnt % 4) == 1)
		{
			xhfc_interrupt();
		}
	}

	for (index = 0; index < MAX_SPANS; ++index)
	// for (index = 0; index < 4; ++index)//todo change to 4 spans, but RPi only support one currently
	{
		// if (g_brie->spans[index]) {
		if (g_brie->spans[index]->span.flags & DAHDI_FLAG_RUNNING)
		{
			if ((cnt % 4) == 1)
			{
				alarm_timer_update(g_brie->spans[index]);
			}
			/* do your receive work */
			bri_receiveprep(g_brie->spans[index], g_brie->rxfifo);
			/* do your transmit work */
			bri_transmitprep(g_brie->spans[index], g_brie->txfifo);
		}
	}
}

/* -------------------------------------------------------------------------- */
/* DAHDI interface functions */

/* Called by dahdi_cfg program. */
static int brie_startup(struct file *file, struct dahdi_span *span)
{
	struct xhfc_span *xspan = container_of(span, struct xhfc_span, span);
	return brispan_start(xspan);
}

static int brie_shutdown(struct dahdi_span *span)
{ /* Never called */
	return 0;
}

static int brie_open(struct dahdi_chan *chan)
{
	if (!try_module_get(THIS_MODULE))
		return -EBUSY;

	return 0;
}

static int brie_close(struct dahdi_chan *chan)
{
	module_put(THIS_MODULE);
	return 0;
}

static int brie_ioctl(struct dahdi_chan *chan,
					  unsigned int cmd, unsigned long data)
{
	switch (cmd)
	{
	case DAHDI_TONEDETECT:
		/* We don't do tone detection. */
		return -EINVAL;

	default:
		printk(KERN_INFO "Unexpected ioctl %x\n", cmd); /*  DBG */
		return -ENOTTY;
	}

	return 0;
}

/* DAHDI calls this when it has data it wants to send to the HDLC controller */
static void brie_hdlc_hard_xmit(struct dahdi_chan *chan)
{
	struct xhfc_chan *xchan = chan->pvt;
	struct xhfc_span *span = xchan->span;

	if (span->sigchan == chan)
	{
		/* Kick it */
		span->xhfc->fifo_irq |= 1 << (span->d_chan->id * 2);

//		xhfc_write_fifo(span->xhfc,span->d_chan->id  );
// queue_work(brie_wq, &span->xhfc->dev->brie_work);
#ifdef USE_TASKLET
		tasklet_schedule(&span->xhfc->dev->brie_bh);
#else
		// schedule_work(&span->xhfc->dev->brie_work);
		// queue_work(brie_wq, &span->xhfc->dev->brie_work);
#endif
	}
	else
		printk(KERN_WARNING "WARNING: %s called on %s\n",
			   __func__, chan->name);
}

/* Called by dahdi_cfg program. */
static int brie_spanconfig(struct file *file, struct dahdi_span *span, struct dahdi_lineconfig *lc)
{
	struct xhfc_span *xspan = container_of(span, struct xhfc_span, span);

	if (lc->lineconfig & DAHDI_CONFIG_NTTE)
	{
		xspan->mode = SPAN_MODE_NT;
	}
	else
	{
		xspan->mode = SPAN_MODE_TE;
	}

	if (lc->lineconfig & DAHDI_CONFIG_TERM)
	{
		xspan->mode |= SPAN_MODE_ENDPOINT;
	}
	else
	{
		xspan->mode &= ~SPAN_MODE_ENDPOINT;
	}

	printk(KERN_INFO "%s %s span %d lineconfig %x - %s (TERM: %s) \n",
		   __func__, span->name, lc->span, lc->lineconfig, (xspan->mode & SPAN_MODE_NT) ? "NT" : "TE", (xspan->mode & SPAN_MODE_ENDPOINT) ? "Enabled" : "Disabled");

	// Update settings
	span->spantype = (xspan->mode & SPAN_MODE_TE) ? SPANTYPE_DIGITAL_BRI_TE : SPANTYPE_DIGITAL_BRI_NT;

	sprintf(span->desc, "SWITCHPI BRI_%s Module %c Span %d",
			(xspan->mode & SPAN_MODE_TE) ? "TE" : "NT",
			xspan->xhfc->modidx + 'A', xspan->id);

	// Reset start
	brispan_stop(xspan);
	brispan_start(xspan);

	return 0;
}

/* Called by dahdi_cfg program on the d-channels only. */
static int brie_chanconfig(struct file *file, struct dahdi_chan *chan, int sigtype)
{
	return 0;
}

/* -------------------------------------------------------------------------- */

/* Get a buffer full from dahdi */
static int dahdi_get_buf(struct xhfc_span *span)
{
	int res, size = sizeof(span->tx_buf);

	if (span == NULL)
		return 0;

	/* res == 0 if there is not a complete frame. */
	res = dahdi_hdlc_getbuf(span->sigchan, span->tx_buf, &size);

	if (size > 0)
	{
		span->tx_idx = 0;
		span->tx_size = size;
		span->tx_frame = res;
	}

	return size;
}

/* Takes data from DAHDI and shoots it out to the hardware.  The blob
 * may or may not be a complete HDLC frame.
 */
static void xhfc_write_fifo(struct xhfc *xhfc, int ch_index)
{
	u8 fcnt, tcnt, i;
	u8 free;
	u8 f1, f2;
	u8 fstat;
	u8 *data;
	int remain;
	struct xhfc_chan *ch = xhfc->chan + ch_index;
	struct xhfc_span *span = ch->span;
	// unsigned int flags=0;
	u8 run = 1;

	if (ch == NULL)
		return;

	if (span == NULL)
		return;

	spin_lock(&xhfc->fifo_lock);
	/* if we're ignoring TE red alarms and we are in alarm restart the S/T state machine */
	if (span->mode & SPAN_MODE_TE && teignored && span->newalarm != 0)
	{
		l1_activate(span);
	}

	if (span->tx_idx == 0)
		if (dahdi_get_buf(span) == 0)
		{

			spin_unlock(&xhfc->fifo_lock);
			return; /* nothing to send */
		}

	while (run)
	{
		remain = span->tx_size - span->tx_idx;

		if (remain <= 0)
		{
			spin_unlock(&xhfc->fifo_lock);
			return;
		}

		xhfc_selfifo(xhfc, ch_index * 2);

		fstat = read_xhfc(xhfc, A_FIFO_STA);
		free = xhfc->max_z - read_xhfc(xhfc, A_USAGE);
		tcnt = free >= remain ? remain : free;

		f1 = read_xhfc(xhfc, A_F1);
		f2 = read_xhfc(xhfc, A_F2);

		fcnt = 0x07 - ((f1 - f2) & 0x07); /* free frame count in tx fifo */

		/*pr_info("%s channel(%i) len(%i) idx(%i) f1(%i) f2(%i) fcnt(%i)"
				"tcnt(%i) free(%i) fstat(%i)\n",
				__FUNCTION__, ch_index, span->tx_size, span->tx_idx,
				 f1, f2, fcnt, tcnt, free, fstat);
		*/

		/* check for fifo underrun during frame transmission */
		fstat = read_xhfc(xhfc, A_FIFO_STA);
		if (fstat & M_FIFO_ERR)
		{
			pr_err("%s transmit fifo channel(%i) underrun idx(%i),"
				   "A_FIFO_STA(0x%02x)\n",
				   __FUNCTION__, ch_index, span->tx_size, fstat);

			write_xhfc(xhfc, A_INC_RES_FIFO, M_RES_FIFO_ERR);
			xhfc->tx_fifo_errs++;

			/* restart frame transmission */
			span->tx_idx = 0;
			continue;
		}

		if (free && fcnt && tcnt)
		{

			data = span->tx_buf + span->tx_idx;
			span->tx_idx += tcnt;

			/* write data to FIFO */
			// printk(" --- DATA WRITE: %d ch_index: %d\n", tcnt, ch_index);
			for (i = 0; i < tcnt; ++i)
			{
				write_xhfc(xhfc, A_FIFO_DATA, *(data + i));
				// printk(" %02x ", *data);
			}
			// printk("\n---END---\n");

			if (span->tx_idx == span->tx_size)
			{
				// pr_info("%s: span->tx_frame: %d\n", __FUNCTION__,span->tx_frame);
				// if (span->tx_frame) {
				/* terminate frame */
				xhfc_inc_f(xhfc);
				//}

				span->tx_idx = 0;

				/* check for fifo underrun during frame transmission */
				fstat = read_xhfc(xhfc, A_FIFO_STA);
				if (fstat & M_FIFO_ERR)
				{
					write_xhfc(xhfc, A_INC_RES_FIFO, M_RES_FIFO_ERR);
					xhfc->tx_fifo_errs++;

					/* restart frame transmission */
					span->tx_idx = 0;
					continue;
				}

				/* no underrun, go to next frame if there is one */
				// if (dahdi_get_buf(span) && (free - tcnt) > 8)
				{
					run = 0;
					// break;
					// continue;
				}
			}
			else
			{
				/* tx buffer not complete, but fifo filled to maximum */
				xhfc_selfifo(xhfc, (ch->id * 2));
				run = 0;
				break;
			}
		}
		else
		{
			run = 0;
		}
	}
	spin_unlock(&xhfc->fifo_lock);
}

static void xhfc_read_fifo(struct xhfc *xhfc, int ch_index)
{
	u8 f1, f2, z1, z2;
	u8 fstat = 0;
	int i;
	int rcnt; /* read rcnt bytes out of fifo */
	u8 *data; /* new data pointer */
	struct xhfc_chan *ch = xhfc->chan + ch_index;
	struct xhfc_span *span = ch->span;

	if (ch == NULL)
		return;

	if (span == NULL)
		return;

	spin_lock(&xhfc->fifo_lock);
	// receive_buffer:
	do
	{

		xhfc_selfifo(xhfc, ch_index * 2 + 1);

		fstat = read_xhfc(xhfc, A_FIFO_STA);
		if (fstat & M_FIFO_ERR)
		{
			write_xhfc(xhfc, A_INC_RES_FIFO, M_RES_FIFO_ERR);
			xhfc->rx_fifo_errs++;
		}

		/* hdlc rcnt */
		f1 = read_xhfc(xhfc, A_F1);
		f2 = read_xhfc(xhfc, A_F2);
		z1 = read_xhfc(xhfc, A_Z1);
		z2 = read_xhfc(xhfc, A_Z2);

		rcnt = (z1 - z2) & xhfc->max_z;
		if (f1 != f2)
			rcnt++;

		if ((span->rx_idx + rcnt) > sizeof(span->rx_buf))
			rcnt = sizeof(span->rx_buf) - span->rx_idx;

		if (rcnt <= 0)
		{
			spin_unlock(&xhfc->fifo_lock);
			return; /* nothing to read */
		}

		/* There seems to be a bug in the chip where if we read only
		 * the first byte of a new frame, it gets corrupted. We see
		 * this about once an hour under a fairly systained 16kbps
		 * load. */
		if (span->rx_idx == 0 && rcnt == 1)
		{
			spin_unlock(&xhfc->fifo_lock);
			return;
		}

		data = span->rx_buf + span->rx_idx;

		/* read data from FIFO */
		// printk("----DATA--%d-\n",rcnt);
		for (i = 0; i < rcnt; ++i)
		{
			data[i] = read_xhfc(xhfc, A_FIFO_DATA);
			//		printk(" %02x ",data[i]);
		}
		// printk("\n----END DATA---\n");
		span->rx_idx += rcnt;

		if (f1 == f2)
		{
			xhfc_selfifo(xhfc, ch_index * 2 + 1);
			spin_unlock(&xhfc->fifo_lock);
			return; /* not a frame */
		}
		/* end of frame */
		xhfc_inc_f(xhfc);

		/* check minimum frame size */
		if (span->rx_idx < 4)
			span->rx_idx = 0;
		else
		{
			/* check crc */
			if (span->rx_buf[span->rx_idx - 1])
			{
				span->rx_idx = 0;
			}
			else
			{

				/* remove cksum */
				span->rx_idx -= 1;
				//		printk("-Sending to hdlc\n");
				/* Tell dahdi about the frame. We do not send up bad frames. */
				dahdi_hdlc_putbuf(span->sigchan, span->rx_buf, span->rx_idx);
				dahdi_hdlc_finish(span->sigchan);
			}
		}

		// read_exit:
		span->rx_idx = 0;

	} while (read_xhfc(xhfc, A_USAGE) > 8);

	spin_unlock(&xhfc->fifo_lock);
}

/* -------------------------------------------------------------------------- */
/* DMA */

static int __init brie_dma_enable(struct brie *dev)
{
	printk("brie_dma_enable entering\n");
	auk_dma_enable(dahdi_auk_dma_isr);
	return 0;
}

/* -------------------------------------------------------------------------- */

/* Register at the same time? */
static void __init dahdi_span_init(struct xhfc_span *span)
{
	struct xhfc *xhfc = span->xhfc;
	struct dahdi_span *dahdi = &span->span;
	if (debug)
		printk("dahdi_span_init ---\n");

	dahdi->spantype = (span->mode & SPAN_MODE_TE) ? SPANTYPE_DIGITAL_BRI_TE : SPANTYPE_DIGITAL_BRI_NT;
	dahdi->offset = span->id;
	dahdi->channels = BRIE_CHANS_PER_SPAN;
	dahdi->flags = 0;

	dahdi->deflaw = DAHDI_LAW_ALAW;

	/* For simplicity, we'll accept all line modes since BRI
	 * ignores this setting anyway.*/
	dahdi->linecompat = DAHDI_CONFIG_AMI |
						DAHDI_CONFIG_B8ZS | DAHDI_CONFIG_D4 |
						DAHDI_CONFIG_ESF | DAHDI_CONFIG_HDB3 |
						DAHDI_CONFIG_CCS | DAHDI_CONFIG_CRC4 | DAHDI_CONFIG_NTTE | DAHDI_CONFIG_TERM;

	sprintf(dahdi->name, "BRI/%d/%d", xhfc->modidx, span->id + 1);
	/* Dahdi matches on this to tell if it is BRI */
	sprintf(dahdi->desc, "SWITCHPI BRI_%s Module %c Span %d",
			(span->mode & SPAN_MODE_TE) ? "TE" : "NT",
			xhfc->modidx + 'A', span->id + 1);

	dahdi->ops = &auk_bri_span_ops;

	dahdi->chans = span->chans;
}

static void brie_disable_interrupts(struct brie *dev)
{
	struct xhfc *xhfc;
	xhfc = &dev->xhfc[0];

	// for (xhfc = dev->xhfc, i = 0; i < dev->num_xhfcs; i++, ++xhfc) {
	/* disable global interrupts */
	xhfc->irq_ctrl.bit.v_glob_irq_en = 0;
	xhfc->irq_ctrl.bit.v_fifo_irq_en = 0;
	write_xhfc(xhfc, R_IRQ_CTRL, xhfc->irq_ctrl.reg);
	// }
}

static int brie_stop(struct brie *dev)
{
	int i;

	for (i = 0; i < MAX_SPANS; ++i)
		if (dev->spans[i])
		{
			dev->spans[i]->mode = SPAN_MODE_TE; /* reset to TE */
			brispan_stop(dev->spans[i]);
		}

	/* We must disable *after* brispan_stop */
	brie_disable_interrupts(dev);
	// aukdma_unregister(DMA_CARD_ID);
	auk_dma_disable(dahdi_auk_dma_isr);

	dev->loopback = 0;

	return 0;
}

/* We don't need to cleanup here.. if this fails brispan_cleanup_spans
 * will be called and will cleanup.
 */
static int __init brispan_create(struct xhfc *xhfc, unsigned span_id)
{
	int rc;
	struct xhfc_span *span = kzalloc(sizeof(struct xhfc_span), GFP_KERNEL);

	if (debug)
		printk("%s: entered\n", __FUNCTION__);

	if (!span)
	{
		printk(KERN_ERR "Unable to create span %d\n", span_id);
		return -ENOMEM;
	}

	/* Calc the span_id for events to user mode. */
	span->span_id = span_id;
	if (xhfc->modidx == 1)
		span->span_id += 4;

	/* Calc the base timeslot */
	if (xhfc->modidx == 1)
		span->timeslot = ((span->span_id - 4) * 2) + 32;
	else
		span->timeslot = span->span_id * 2;
	printk("brispan_create: span_id: %d timeslot: %d\n", span->span_id, span->timeslot);
	span->id = span_id;
	span->xhfc = xhfc;
	span->modidx = xhfc->modidx;

	span->mode = (nt_mask & (1 << span->span_id)) ? SPAN_MODE_NT : SPAN_MODE_TE;
	if (endpoint_mask & (1 << span->span_id))
		span->mode |= SPAN_MODE_ENDPOINT;
	span->rx_idx = 0;
	span->tx_idx = 0;

	/* init t1 timer - nt only */

	timer_setup(&span->t1_timer, l1_timer_expire_t1, 0);

	/* init t3 timer - te only */
	timer_setup(&span->t3_timer, l1_timer_expire_t3, 0);

	/* init t4 timer - te only */
	timer_setup(&span->t4_timer, l1_timer_expire_t4, 0);

	/* init dahdi alarm timer */
	timer_setup(&span->alarm_timer, alarm_timer_expire, 0);

	xhfc->dev->spans[span->span_id] = span;

	/* spans manage channels */
	rc = brichannels_create(span);
	if (rc)
		return rc;

	dahdi_span_init(span);

	span->sigchan = &span->d_chan->chan;
	if (debug)
		printk("%s: exited\n", __FUNCTION__);

	return 0;
}

static void brispan_destroy(struct xhfc_span *span)
{
	kfree(span);
}

/* Yes this is an init function. It is called to cleanup all the span
 * instances on an xhfc that where created when a span instance fails.
 */
static void __init brispan_cleanup_spans(struct xhfc *xhfc)
{
	int i;

	for (i = 0; i < MAX_SPANS; ++i)
		if (xhfc->dev->spans[i])
			if (xhfc->dev->spans[i]->xhfc == xhfc)
			{
				brispan_destroy(xhfc->dev->spans[i]);
				xhfc->dev->spans[i] = NULL;
			}
}

static int brispan_start(struct xhfc_span *span)
{
	if (span->mode & SPAN_MODE_STARTED)
		return 0;

	if (debug)
		printk(KERN_INFO "##########start up %s %s\n", __func__, span->span.name);

	span->tx_idx = span->rx_idx = 0;

	/* We must set the span in the correct deactivated state for
	 * it's mode */
	write_xhfc(span->xhfc, R_SU_SEL, span->id);
	write_xhfc(span->xhfc, A_SU_WR_STA,
			   (span->mode & SPAN_MODE_TE) ? 0x53 : 0x51);
	udelay(6);

	brispan_apply_config(span);

	dchannel_toggle_fifo(span->d_chan, 1);
	bchannel_toggle(span->b1_chan, 1);
	bchannel_toggle(span->b2_chan, 1);

	l1_activate(span);

	span->span.flags |= DAHDI_FLAG_RUNNING;
	span->mode |= SPAN_MODE_STARTED;

	return 0;
}

static int brispan_stop(struct xhfc_span *span)
{
	struct xhfc *xhfc = span->xhfc;

	clear_bit(HFC_L1_ACTIVATING, &span->l1_flags);

	dchannel_toggle_fifo(span->d_chan, 0);

	bchannel_toggle(span->b1_chan, 0);
	bchannel_toggle(span->b2_chan, 0);

	l1_deactivate(span);

	span->mode &= ~SPAN_MODE_STARTED;
	span->span.flags &= ~DAHDI_FLAG_RUNNING;

	/* These actions disable the automatic state machine No state
	   machine changes can occur again until l1_activate is
	   triggered */
	if (span->mode & SPAN_MODE_TE)
	{
		span->state = 3;
		write_xhfc(xhfc, R_SU_SEL, span->id);
		/* Manually load deactivated state (3 for TE), and set
		 * deactivating flag */
		write_xhfc(xhfc, A_SU_WR_STA, 0x53);
	}
	else
	{
		span->state = 3;
		write_xhfc(xhfc, R_SU_SEL, span->id);
		/* Manually load deactivated state (1 for NT), and set
		 * deactivating flag */
		write_xhfc(xhfc, A_SU_WR_STA, 0x51);
	}

	udelay(6);

	/* Make sure all the timers are dead */
	del_timer_sync(&span->t1_timer);
	del_timer_sync(&span->t3_timer);
	del_timer_sync(&span->t4_timer);
	del_timer_sync(&span->alarm_timer);

	return 0;
}

/* Must be called *after* setting TE/NT. */
void set_clock(struct brie *dev)
{
	struct xhfc *xhfc;
	unsigned syncsrc = 0;
	int pcm_master = -1; /* default to none */
	unsigned reg = 0;

	if (debug)
		printk("set_clock entered\n");

	if (!dev->loopback)
	{
		pcm_master = 1;
		syncsrc = 2 << dev->xhfc[pcm_master].modidx;
	}

	/* Only set the clock if it changed. */
	// if (pcm_master == dev->pcm_master && (reg & 6) == syncsrc)
	if (pcm_master == dev->pcm_master)
	{
		printk("Already set pcm_master\n");
		return;
	}

	dev->pcm_master = pcm_master;

	// here disable the dma engine to avoid other modules crash

	xhfc = &dev->xhfc[0];

	if (pcm_master == 1)
	{
		printk("Setting PCM_Master on card 0\n");
		/* short f0i0 pulse, indirect access to 9 of 15 */
		write_xhfc(xhfc, R_PCM_MD0, 0x95); // 0x94

		/* set pcm to 2mbit/s (32 timeslots) */
		write_xhfc(xhfc, R_PCM_MD1, 0x0C); // 0x0c

		/* indirect register access to A */
		write_xhfc(xhfc, R_PCM_MD0, 0xA5); // 0xa4
		write_xhfc(xhfc, R_PCM_MD2, 0x10); // 0x10

		// write_xhfc(xhfc, R_PCM_MD0, 0x05);
		// write_xhfc(xhfc, R_SL_SEL0, 0x1f);

		// write_xhfc(xhfc, R_PCM_MD0, 0xC5);
		// write_xhfc(xhfc, R_SH0L, 0x80);

		// write_xhfc(xhfc, R_PCM_MD0, 0xd5);
		// write_xhfc(xhfc, R_SH0H, 0x07);

		/* auto select line for clock source */
		// sync source from interface 0, page 249
		write_xhfc(xhfc, R_SU_SYNC, 0x0);
		// or sycn source from sync_i pin (p25)
		// write_xhfc(xhfc, R_SU_SYNC, 0x10);

		/* set pcm to master mode */
		write_xhfc(xhfc, R_PCM_MD0, 0x9);
	}
	else
	{
		printk("Setting PCM_Slave on 0\n");
		/* short f0i0 pulse, indirect access to 9 of 15 */
		write_xhfc(xhfc, R_PCM_MD0, 0x94);

		/* set pcm to 4mbit/s (64 timeslots) */
		write_xhfc(xhfc, R_PCM_MD1, 0x0C);

		/* indirect register access to A */
		write_xhfc(xhfc, R_PCM_MD0, 0xA4);
		write_xhfc(xhfc, R_PCM_MD2, 0x10);

		write_xhfc(xhfc, R_PCM_MD0, 0x04);
		write_xhfc(xhfc, R_SL_SEL0, 0x1f);

		write_xhfc(xhfc, R_PCM_MD0, 0xC4);
		write_xhfc(xhfc, R_SH0L, 0x80);

		write_xhfc(xhfc, R_PCM_MD0, 0xd4);
		write_xhfc(xhfc, R_SH0H, 0x07);

		if (!dev->loopback)
		{
			/* use interface 0 as the sync source */
			write_xhfc(xhfc, R_SU_SYNC, 0x0);
		}
		else
		{
			write_xhfc(xhfc, R_PCM_MD2, 0x04);
			/* use warp as the sync source */
			write_xhfc(xhfc, R_SU_SYNC, 0x1C);
		}

		/* set pcm to slave mode */
		write_xhfc(xhfc, R_PCM_MD0, 0x8);
	}
	reg = (reg & ~6) | syncsrc;

	mdelay(2); /* allow FXO/FXS PLL to settle */

	// aukdma_enable();
}

static void gpio_nt_te(struct xhfc_span *span, int unsigned mask)
{
	unsigned reg = read_xhfc(span->xhfc, R_GPIO_IN0);
	printk("NT/TE GPIO read in %x on span %d\n", reg, span->span_id);

	if (span->mode & SPAN_MODE_TE)
		reg |= mask;
	else
		reg &= ~mask;

	printk("NT/TE GPIO write out %x on span %d\n", reg, span->span_id);
	write_xhfc(span->xhfc, R_GPIO_OUT0, reg);
}

void brispan_apply_config(struct xhfc_span *span)
{
	unsigned reg_mask;
	u8 delay;
	struct xhfc *xhfc = span->xhfc;

	reg_mask = 8 << (xhfc->modidx + 2 * span->id);

	/* configure te/nt and endpoint */
	switch (span->span_id)
	{
	case 0:
		gpio_nt_te(span, 1);
		// gpio_endpoint(span, 4);
		break;
	case 1:
		gpio_nt_te(span, 2);
		// gpio_endpoint(span, 4);
		break;
	case 2:
		break;
	case 3:
		break;
	}
	set_clock(xhfc->dev);

	/* now xhfc stuff */
	write_xhfc(xhfc, R_SU_SEL, span->id);

	/* Reset state value */
	span->state = 0;

	if (span->mode & SPAN_MODE_NT)
	{
		delay = CLK_DLY_NT;
		span->su_ctrl0.bit.v_su_md = 1;
		l1_timer_stop_t1(span);
	}
	else
	{
		delay = CLK_DLY_TE;
		span->su_ctrl0.bit.v_su_md = 0;
	}

	if (span->mode & SPAN_MODE_EXCH_POL)
		span->su_ctrl2.reg = M_SU_EXCHG;
	else
		span->su_ctrl2.reg = 0;

	/* configure end of pulse control for ST mode (TE & NT) */
	span->su_ctrl0.bit.v_st_pu_ctrl = 1;
	write_xhfc(xhfc, A_ST_CTRL3, 0xf8);

	write_xhfc(xhfc, A_SU_CTRL0, span->su_ctrl0.reg);
	write_xhfc(xhfc, A_SU_CTRL1, span->su_ctrl1.reg);
	write_xhfc(xhfc, A_SU_CTRL2, span->su_ctrl2.reg);

	write_xhfc(xhfc, A_SU_CLK_DLY, delay);

	/*  Check if NT? */
	alarm_timer_update(span);
}

void set_te_leds(struct xhfc_span *span, u8 state)
{
	unsigned char reg;
	reg = read_xhfc(span->xhfc, R_GPIO_IN0);

	if (span->span_id == 0)
	{
		if ((state & 0xf) == 7) 
		{
			reg |= 0x4;
			reg &= ~0x80;
		}
		else 
		{
			reg &= ~0x4;
			reg |= 0x80;
		}
		write_xhfc(span->xhfc, R_GPIO_OUT0, reg);
		return;
	}
	else if (span->span_id == 1)
	{
		if ( (state & 0xf) == 7)
		{
			reg |= 0x8;
			reg &= ~0x40;
		}
		else 
		{
			reg &= ~0x8;
			reg |= 0x40;
		}
		write_xhfc(span->xhfc, R_GPIO_OUT0, reg);
		return;
	}
	else return;
}

void set_nt_leds(struct xhfc_span *span, u8 state)
{
	unsigned char reg;
	reg = read_xhfc(span->xhfc, R_GPIO_IN0);

	if (span->span_id == 0)
	{
		if ((state & 0xf) == 3) 
		{
			reg |= 0x4;
			reg &= ~0x80;
		}
		else 
		{
			reg &= ~0x4;
			reg |= 0x80;
		}
		write_xhfc(span->xhfc, R_GPIO_OUT0, reg);
		return;
	}
	else if (span->span_id == 1)
	{
		if ( (state & 0xf) == 3)
		{
			reg |= 0x8;
			reg &= ~0x40;
		}
		else 
		{
			reg &= ~0x8;
			reg |= 0x40;
		}
		write_xhfc(span->xhfc, R_GPIO_OUT0, reg);
		return;
	}
	else return;
}

void brispan_new_state(struct xhfc_span *span, u8 new_state, int expired)
{
	spin_lock(&span->xhfc->fifo_tx_lock);
	/* Hack for state F6 to F7 change. */
	if ((span->state & 0xf) == 6 && (new_state & 0x4f) == 0x47)
	{
		u8 state;

		/* We should start a timer, but then
		 * we have lots o' race conditions. */
		// mdelay(1);
		write_xhfc(span->xhfc, R_SU_SEL, span->id);
		state = read_xhfc(span->xhfc, A_SU_RD_STA);
		if ((state & 0xf) == 3)
		{
			/* ignore state 7 */
			new_state = state;
			printk(KERN_INFO
				   "Span %d %s L1 dropping state 7\n",
				   span->span_id,
				   span->mode & SPAN_MODE_TE ? "TE" : "NT");
		}
	}

	if (debug)
		printk(KERN_INFO
			   "Span %d %s L1 from state %2x to %2x (%lx)\n",
			   span->span_id,
			   span->mode & SPAN_MODE_TE ? "TE" : "NT",
			   span->state, new_state,
			   span->l1_flags | (expired << 12));

	span->state = new_state; /* update state now */

	if (span->mode & SPAN_MODE_TE)
	{

		if ((new_state & 0xf) <= 3 || (new_state & 0xf) >= 7)
			l1_timer_stop_t3(span);

		set_te_leds(span, new_state);

		switch (new_state & 0xf)
		{
		case 0: /* TE state F0: Reset */
		case 2: /* TE state F2: Sensing */
		case 4: /* TE state F4: Awaiting Signal */
			span->newalarm = DAHDI_ALARM_RED;
			if (debug)
				printk("State: %d - RED ALARM\n", span->newalarm & 0x7);
			break;
		case 5: /* TE state F5: Identifying Input */
		case 6: /* TE state F6: Synchronized */
			span->newalarm = DAHDI_ALARM_YELLOW;
			if (debug)
				printk("State: %d - RED YELLOW\n", span->newalarm & 0x7);
			break;

		case 3: /* TE state F3: Deactivated */
			span->newalarm = DAHDI_ALARM_RED;

			if (debug)
				printk("State: %d - RED ALARM\n", span->newalarm & 0x7);

			if (test_bit(HFC_L1_ACTIVATING, &span->l1_flags))
				l1_activate(span);

			if (test_and_clear_bit(HFC_L1_ACTIVATED, &span->l1_flags))
				l1_timer_start_t4(span);
			break;

		case 7: /* TE state F7: Activated */
			span->newalarm = 0;

			clear_bit(HFC_L1_ACTIVATING, &span->l1_flags);
			l1_timer_stop_t4(span);

			set_bit(HFC_L1_ACTIVATED, &span->l1_flags);

			if (debug)
				printk("State: %d - ACTIVATED\n", span->newalarm & 0x7);

			break;

		case 8: /* TE state F8: Lost Framing */
			span->newalarm = DAHDI_ALARM_YELLOW;

			clear_bit(HFC_L1_ACTIVATING, &span->l1_flags);
			l1_timer_stop_t4(span);

			if (debug)
				printk("State: %d - YELLOW ALARM\n", span->newalarm & 0x7);

			break;
		}

		if (debug)
			printk("Sync Source: %d\n", read_xhfc(span->xhfc, R_BERT_STA) & 0x7);
	}
	else if (span->mode & SPAN_MODE_NT)
	{
		set_nt_leds(span, new_state);

		switch (new_state & 0xf)
		{
		case 0: /* NT state G0: Reset */
			span->newalarm = DAHDI_ALARM_RED;
			break;

		case 1: /* NT state G1: Deactivated */
			span->newalarm = DAHDI_ALARM_RED;

			clear_bit(HFC_L1_ACTIVATED, &span->l1_flags);

			l1_timer_stop_t1(span);
			break;
		case 2: /* NT state G2: Pending Activation */
			span->newalarm = DAHDI_ALARM_YELLOW;

			if (expired)
				clear_bit(HFC_L1_ACTIVATED, &span->l1_flags);
			else
			{
				write_xhfc(span->xhfc, R_SU_SEL, span->id);
				write_xhfc(span->xhfc, A_SU_WR_STA,
						   M_SU_SET_G2_G3);

				l1_timer_start_t1(span);
			}
			break;
		case 3: /* NT state G3: Active */
			span->newalarm = 0;

			set_bit(HFC_L1_ACTIVATED, &span->l1_flags);

			l1_timer_stop_t1(span);
			break;
		case 0x4: /* NT state G4: Pending Deactivation */
			span->newalarm = DAHDI_ALARM_RED;

			l1_timer_stop_t1(span);
			break;
		}
	}
	spin_unlock(&span->xhfc->fifo_tx_lock);
}

static void l1_activate(struct xhfc_span *span)
{
	struct xhfc *xhfc = span->xhfc;

	if (test_bit(HFC_L1_ACTIVATED, &span->l1_flags))
		return; /* already activated */

	if (span->mode & SPAN_MODE_TE)
	{
		set_bit(HFC_L1_ACTIVATING, &span->l1_flags);

		write_xhfc(xhfc, R_SU_SEL, span->id);
		write_xhfc(xhfc, A_SU_WR_STA, STA_ACTIVATE);
		l1_timer_start_t3(span);
	}
	else
	{
		write_xhfc(xhfc, R_SU_SEL, span->id);
		write_xhfc(xhfc, A_SU_WR_STA, STA_ACTIVATE | M_SU_SET_G2_G3);
	}
}

/* This function is meant for deactivations that occur during certain
 * state machine timeouts, it is not meant to deactivate the state
 * machine. See brispan_stop to do that */
static void l1_deactivate(struct xhfc_span *span)
{
	struct xhfc *xhfc = span->xhfc;

	if (span->mode & SPAN_MODE_TE)
	{
		write_xhfc(xhfc, R_SU_SEL, span->id);
		// write_xhfc(xhfc, A_SU_WR_STA, STA_DEACTIVATE);
		write_xhfc(xhfc, A_SU_WR_STA, M_SU_LD_STA | 0x03);
		udelay(6);
		write_xhfc(xhfc, A_SU_WR_STA, 0);
	}
}
int l1_timer_cnt = 0;
static void l1_timer_start_t1(struct xhfc_span *span)
{
	if (!timer_pending(&span->t1_timer))
	{
		span->t1_timer.expires =
			jiffies + msecs_to_jiffies(100);
		if (span->mode & SPAN_MODE_STARTED)
		{
			l1_timer_cnt++;
			mod_timer(&span->t1_timer, span->t1_timer.expires);
			// add_timer(&span->t1_timer);
		}
	}
}

static inline void l1_timer_stop_t1(struct xhfc_span *span)
{
	del_timer(&span->t1_timer);
}

int l1_timer_expire_cnt = 0;
static void l1_timer_expire_t1(struct timer_list *t)
{
	struct xhfc_span *span = from_timer(span, t, t1_timer); //(struct xhfc_span *)arg;
	brispan_new_state(span, span->state, 1);
	l1_timer_expire_cnt++;
}

int t3_timer_cnt = 0;
static void l1_timer_start_t3(struct xhfc_span *span)
{
	if (!timer_pending(&span->t3_timer))
	{
		span->t3_timer.expires =
			jiffies + msecs_to_jiffies(XHFC_TIMER_T3);
		if (span->mode & SPAN_MODE_STARTED)
		{
			t3_timer_cnt++;
			// printk("start t3_timer\n");
			mod_timer(&span->t3_timer, span->t3_timer.expires);
			// add_timer(&span->t3_timer);
		}
	}
}

static inline void l1_timer_stop_t3(struct xhfc_span *span)
{
	del_timer(&span->t3_timer);
}

int t3_timer_expire_cnt = 0;
static void l1_timer_expire_t3(struct timer_list *t)
{
	struct xhfc_span *span = from_timer(span, t, t3_timer); //(struct xhfc_span *)arg;

	t3_timer_expire_cnt++;
	l1_deactivate(span);
	/* afterwards will attempt to reactivate in state F3 since
	 * HFC_L1_ACTIVATING is set */
}

int t4_timer_cnt = 0;
static void l1_timer_start_t4(struct xhfc_span *span)
{
	if (!timer_pending(&span->t4_timer))
	{
		span->t4_timer.expires =
			jiffies + msecs_to_jiffies(XHFC_TIMER_T4);
		if (span->mode & SPAN_MODE_STARTED)
		{
			t4_timer_cnt++;
			mod_timer(&span->t4_timer, span->t4_timer.expires);
		}
	}
}

static inline void l1_timer_stop_t4(struct xhfc_span *span)
{
	del_timer(&span->t4_timer);
}

int t4_timer_expire_cnt = 0;
static void l1_timer_expire_t4(struct timer_list *t)
{
	struct xhfc_span *span = from_timer(span, t, t4_timer);
	t4_timer_expire_cnt++;
	clear_bit(HFC_L1_ACTIVATED, &span->l1_flags);
}

int alarm_cnt = 0;
static void alarm_timer_update(struct xhfc_span *span)
{
	span->alarm_timer.expires = jiffies + msecs_to_jiffies(500);

	if (!timer_pending(&span->alarm_timer))
	{
		if (span->mode & SPAN_MODE_STARTED)
		{
			alarm_cnt++;
			mod_timer(&span->alarm_timer, span->alarm_timer.expires);
		}
	}
}

int alarm_expire_cnt = 0;
int alarm_dahdi_cnt = 0;
static void alarm_timer_expire(struct timer_list *t)
{
	struct xhfc_span *span = from_timer(span, t, alarm_timer);
	alarm_expire_cnt++;

	if (span->span.alarms != span->newalarm)
	{
		span->span.alarms = span->newalarm;
		if ((!span->newalarm && teignored) || (!teignored))
		{
			alarm_dahdi_cnt++;
			dahdi_alarm_notify(&span->span);
		}
	}
}

void dchannel_toggle_fifo(struct xhfc_chan *chan, u8 enable)
{
	struct xhfc *xhfc = chan->span->xhfc;
	unsigned fifo = chan->id * 2;
	unsigned mask = (1 << fifo) | (1 << (fifo + 1));

	if (enable)
		xhfc->fifo_irqmask |= mask;
	else
		xhfc->fifo_irqmask &= ~mask;
}

int bchannel_toggle(struct xhfc_chan *chan, u8 enable)
{
	struct xhfc_span *span = chan->span;
	int bc = chan->id % CHAN_PER_SPAN;

	if (bc < 0 || bc > 1)
		return EINVAL;

	/* Force to 1 or 0 */
	enable = enable ? 1 : 0;

	if (bc)
	{
		span->su_ctrl2.bit.v_b2_rx_en = enable;
		span->su_ctrl0.bit.v_b2_tx_en = enable;
	}
	else
	{
		span->su_ctrl2.bit.v_b1_rx_en = enable;
		span->su_ctrl0.bit.v_b1_tx_en = enable;
	}

	write_xhfc(span->xhfc, R_SU_SEL, span->id);
	write_xhfc(span->xhfc, A_SU_CTRL0, span->su_ctrl0.reg);
	write_xhfc(span->xhfc, A_SU_CTRL2, span->su_ctrl2.reg);

	return 0;
}

static void __init dahdi_chan_init(struct xhfc_chan *chan, int id)
{
	struct xhfc_span *span = chan->span;
	struct xhfc *xhfc = span->xhfc;

	span->chans[id] = &chan->chan;
	chan->chan.pvt = chan;

	if (debug)
		printk("%s: entered\n", __FUNCTION__);

	/* Dahdi matches on the B4 to know it is BRI. */
	sprintf(chan->chan.name, "B4/%d/%d/%d",
			xhfc->modidx, span->id + 1, id + 1);

	chan->chan.chanpos = id + 1;
	chan->chan.writechunk = chan->writechunk;
	chan->chan.readchunk = chan->readchunk;

	if (id == 2)
	{ /* d-chan */
		chan->chan.sigcap = DAHDI_SIG_HARDHDLC;
		// TODO
		chan->chan.sig = DAHDI_SIG_HDLCFCS;
	}
	else
	{
		chan->chan.sigcap = DAHDI_SIG_CLEAR | DAHDI_SIG_DACS;
		// TODO
		chan->chan.sig = DAHDI_SIG_CLEAR;
	}
}

int __init brichannels_create(struct xhfc_span *span)
{
	int ch_index;
	struct xhfc *xhfc = span->xhfc;
	if (debug)
		printk("%s: entered\n", __FUNCTION__);

	/* init B1 channel */
	ch_index = (span->id << 2);
	printk("first b1 channel on span %d index is %d\n", span->id, ch_index);
	span->b1_chan = &(xhfc->chan[ch_index]);
	xhfc->chan[ch_index].span = span;
	xhfc->chan[ch_index].id = ch_index;

	bchannel_setup_pcm(&xhfc->chan[ch_index], 0);
	bchannel_setup_pcm(&xhfc->chan[ch_index], 1);

	dahdi_chan_init(&xhfc->chan[ch_index], 0);

	/* init B2 channel */
	ch_index++;
	span->b2_chan = &(xhfc->chan[ch_index]);
	xhfc->chan[ch_index].span = span;
	xhfc->chan[ch_index].id = ch_index;

	bchannel_setup_pcm(&xhfc->chan[ch_index], 0);
	bchannel_setup_pcm(&xhfc->chan[ch_index], 1);

	dahdi_chan_init(&xhfc->chan[ch_index], 1);

	/* init D channel */
	ch_index++;
	span->d_chan = &(xhfc->chan[ch_index]);
	xhfc->chan[ch_index].span = span;
	xhfc->chan[ch_index].id = ch_index;

	dchannel_setup_fifo(&xhfc->chan[ch_index], 0);
	dchannel_setup_fifo(&xhfc->chan[ch_index], 1);

	dahdi_chan_init(&xhfc->chan[ch_index], 2);

	/* Clear PCM  */
	ch_index++;
	memset(&xhfc->chan[ch_index], 0, sizeof(struct xhfc_chan));

	if (debug)
		printk("brichannels_create exited\n");
	return 0;
}

int __init dchannel_setup_fifo(struct xhfc_chan *chan, unsigned rx)
{
	struct xhfc *xhfc = chan->span->xhfc;
	u8 fifo = (chan->id << 1) + rx;

	if (debug)
		printk("dchannel_setup_fifo entered: fifo: %d\n", fifo);

	xhfc_selfifo(xhfc, fifo);
	printk("D channel select fifo %02x\n", fifo);

	write_xhfc(xhfc, A_CON_HDLC, 0x5);
	write_xhfc(xhfc, A_SUBCH_CFG, 0x2);

	write_xhfc(xhfc, A_FIFO_CTRL,
			   M_FR_ABO | M_FIFO_IRQMSK | M_MIX_IRQ);

	xhfc_resetfifo(xhfc);

	return 0;
}

int __init bchannel_setup_pcm(struct xhfc_chan *chan, unsigned rx)
{
	__u8 fifo = (chan->id << 1) + rx;
	int timeslot = (chan->id >> 1) + (chan->id & 1);
	struct xhfc *xhfc = chan->span->xhfc;

	if (debug)
		printk("%s entered - tinmesolt: %d, rx: %d\n", __FUNCTION__, timeslot, rx);

	/* Setup B channel */
	write_xhfc(xhfc, R_SLOT, (timeslot << 1) | rx);
	write_xhfc(xhfc, A_SL_CFG, fifo | 0xC0);
	write_xhfc(xhfc, R_FIFO, fifo);

	write_xhfc(xhfc, A_CON_HDLC, 0xFF); /* enable fifo for PCM to S/T */

	if (rx == 0)
	{
		printk("TX: R_SLOT %02x R_FIFO %02x A_SL_CFG %02x\n", (timeslot << 1) | rx, fifo, fifo | 0xC0);
		/* tx */
		write_xhfc(xhfc, A_SUBCH_CFG, 0x0); /* 1 start bit, 6 bits */
		write_xhfc(xhfc, A_CH_MSK, 0xFF);
	}
	else
	{
		printk("RX: R_SLOT %02x R_FIFO %02x A_SL_CFG %02x\n", (timeslot << 1) | rx, fifo, fifo | 0xC0);
		/* rx _SUBCH_CFG MUST ALWAYS BE 000 in RX fifo in
		 * Simple Transparent mode */
		write_xhfc(xhfc, A_SUBCH_CFG, 0);
	}

	/* Reset Fifo*/
	write_xhfc(xhfc, A_INC_RES_FIFO, 0x0A);

	if (debug)
		printk("%s: entering xhfc_waitbusy\n", __FUNCTION__);

	xhfc_waitbusy(xhfc);
	if (debug)
		printk("%s: exited xhfc_waitbusy\n", __FUNCTION__);

	return 0;
}

static void __init brie_enable_interrupts(struct brie *dev)
{
	struct xhfc *xhfc;

	// currently, we only support one xhfc chip
	xhfc = &dev->xhfc[0];

	printk("brie_enable_interrupts entering\n");
	// for (xhfc = dev->xhfc, i = 0; i < dev->num_xhfcs; i++, ++xhfc) {
	printk("brie_enable_interrupts enter\n");
	write_xhfc(xhfc, R_TI_WD, 4);		   // every 4ms
	write_xhfc(xhfc, R_SU_IRQMSK, 0x0f);   // xhfc->su_irqmask.reg);
	write_xhfc(xhfc, R_MISC_IRQMSK, 0x02); // xhfc->su_irqmask.reg);

	/* clear all pending interrupts bits */
	read_xhfc(xhfc, R_MISC_IRQ);
	read_xhfc(xhfc, R_SU_IRQ);
	read_xhfc(xhfc, R_FIFO_BL0_IRQ);
	read_xhfc(xhfc, R_FIFO_BL1_IRQ);
	read_xhfc(xhfc, R_FIFO_BL2_IRQ);
	read_xhfc(xhfc, R_FIFO_BL3_IRQ);

	/* enable global interrupts */
	xhfc->irq_ctrl.bit.v_glob_irq_en = 1;
	xhfc->irq_ctrl.bit.v_fifo_irq_en = 1;
	write_xhfc(xhfc, R_IRQ_CTRL, 0x09); // xhfc->irq_ctrl.reg);
	// }

	printk("brie_enable_interrupts R_IRQ_CTRL: %x - R_SU_IRQMSK: %x\n", xhfc->irq_ctrl.reg, xhfc->su_irqmask.reg);
}

void bri_post_failure(char *err_str)
{
#ifdef CONFIG_POST
	post_fail(POST_BRI_DVR_ID, err_str);
#endif
	printk(KERN_ERR "%s\n", err_str);
}

static void __init brie_init(struct brie *dev)
{
	if (debug)
		printk("brie_init entering\n");
}

static int __init bri_module_init(struct xhfc *xhfc)
{
	reg_r_pcm_md0 pcm_md0;
	reg_r_pcm_md1 pcm_md1;
	u8 threshold;
	int timeout = 0x2000;
	int id = read_xhfc(xhfc, R_CHIP_ID);
	int rev = read_xhfc(xhfc, R_CHIP_RV);

	/* We no longer check the revision (MI#6993). We do not know
	 * if we can work with anthing other than rev 0. */
	if (id == CHIP_ID_2S4U)
	{
		printk(KERN_INFO "XHFC-2S4U Rev %x\n", rev);
		xhfc->num_spans = 2;
		xhfc->su_irqmask.bit.v_su0_irqmsk = 1;
		xhfc->su_irqmask.bit.v_su1_irqmsk = 1;
	}
	else if (id == CHIP_ID_4SU)
	{
		printk(KERN_INFO "XHFC-4SU Rev %x\n", rev);
		xhfc->num_spans = 4;
		xhfc->su_irqmask.bit.v_su0_irqmsk = 1;
		xhfc->su_irqmask.bit.v_su1_irqmsk = 1;
		xhfc->su_irqmask.bit.v_su2_irqmsk = 1;
		xhfc->su_irqmask.bit.v_su3_irqmsk = 1;
	}
	else
	{
		printk(KERN_WARNING "Unexpect id %x rev %x (0)\n", id, rev);
		return -ENODEV;
	}

	/* Setup FIFOs */
	if (xhfc->num_spans == 4)
	{
		/* 64 byte fifos */
		xhfc->max_z = 0x3F;
		xhfc->max_fifo = 16;
		write_xhfc(xhfc, R_FIFO_MD, 0);
		threshold = 0x34;
	}
	else
	{
		/* 128 byte fifos */
		xhfc->max_z = 0x7F;
		xhfc->max_fifo = 8;
		write_xhfc(xhfc, R_FIFO_MD, 1);
		threshold = 0x68;
	}

	/* software reset to enable R_FIFO_MD setting */
	write_xhfc(xhfc, R_CIRM, M_SRES);
	udelay(5);
	write_xhfc(xhfc, R_CIRM, 0);

	/* Set GPIO - this enables GPIO pins 0,1,2,3, and 7 then sets
	 * them to be outputs to control the analog switch
	 */
	write_xhfc(xhfc, R_GPIO_SEL, 0x8f);
	write_xhfc(xhfc, R_GPIO_EN0, 0x8f);
	write_xhfc(xhfc, R_GPIO_OUT0, 0);

	if (debug)
		printk("%s: Make sure R_STATUS is not busy\n", __FUNCTION__);

	while ((read_xhfc(xhfc, R_STATUS) & (M_BUSY | M_PCM_INIT)) && timeout)
		timeout--;

	if (!timeout)
	{
		printk(KERN_ERR
			   "%s: initialization sequence could not finish\n",
			   __func__);
		return -ENODEV;
	}

	if (debug)
		printk("%s: Setting threshold\n", __FUNCTION__);

	/* Set threshold *after* software reset done */
	write_xhfc(xhfc, R_FIFO_THRES, threshold);

	if (debug)
		printk("%s: Setting PCM slave mode\n", __FUNCTION__);

	/* set PCM master mode */
	pcm_md0.reg = 0;
	pcm_md1.reg = 0;
	pcm_md0.bit.v_pcm_md = 1;		   // Master
	write_xhfc(xhfc, R_PCM_MD0, 0x05); // pcm_md0.reg);

	if (debug)
		printk("%s: Adjust the PLL\n", __FUNCTION__);

	/* set pll adjust */
	pcm_md0.bit.v_pcm_idx = IDX_PCM_MD1;
	pcm_md1.bit.v_pll_adj = 3;

	printk("PCM_MD0: %x - pcm_md1: %x\n", pcm_md0.reg, pcm_md1.reg);

	write_xhfc(xhfc, R_PCM_MD0, 0x95); // pcm_md0.reg);
	write_xhfc(xhfc, R_PCM_MD1, 0x0c); // pcm_md1.reg);

	spin_lock_init(&xhfc->fifo_tx_lock);
	spin_lock_init(&xhfc->fifo_lock);
	spin_lock_init(&xhfc->dev->tasklet_lock);
	spin_lock_init(&xhfc->dev->workqueue_lock);

	if (debug)
		printk("%s: exiting with status code 0\n", __FUNCTION__);

	return 0;
}

// id 0, chan starts from 0;
static int __init bri_module_create(struct brie *dev, int id, int chan)
{
	int err, i;
	struct xhfc *xhfc = &(dev->xhfc[id]);
	char err_str[256];

	xhfc->dev = dev;

	xhfc->modidx = chan == 0 ? 0 : 1;
	xhfc->chipnum = chan;

	err = bri_module_init(xhfc);
	if (err)
		return err;

	/* alloc mem for all channels */
	xhfc->chan = kzalloc(sizeof(struct xhfc_chan) * MAX_CHAN, GFP_KERNEL);
	if (!xhfc->chan)
	{
		sprintf(err_str, "%d %s: No kmem for xhfc_chan_t*%i [#500]",
				xhfc->modidx, __func__, MAX_CHAN);
		bri_post_failure(err_str);
		return -ENOMEM;
	}

	xhfc->ddev = dahdi_create_device();
	if (!xhfc->ddev)
	{
		sprintf(err_str, "%s() : cannot create dahdi_device [#501]", __FUNCTION__);
		bri_post_failure(err_str);
		return -ENOMEM; // TODO: should -ENOMEM
	}
	xhfc->ddev->location = kasprintf(GFP_KERNEL, "SWITCHPI BRI on module %c", xhfc->modidx + 'A');
	if (!xhfc->ddev->location)
	{
		dahdi_free_device(xhfc->ddev);
		xhfc->ddev = NULL;
		sprintf(err_str, "%s() : cannot create location for module %d [#502]", __FUNCTION__, xhfc->modidx);
		bri_post_failure(err_str);
		return -ENOMEM; // TODO: should be -ENOMEM
	}

	xhfc->ddev->manufacturer = "SWITCHPI";
	if (xhfc->modidx == 0)
		xhfc->ddev->devicetype = "BRI Module A";
	else
		xhfc->ddev->devicetype = "BRI Module B";

	xhfc->ddev->hardware_id = kasprintf(GFP_KERNEL, "%d", xhfc->modidx);
	if (!xhfc->ddev->hardware_id)
	{
		dahdi_free_device(xhfc->ddev);
		kfree(xhfc->ddev->location);
		xhfc->ddev = NULL;
		// TODO: Fix Post
		sprintf(err_str, "%s() : cannot create hardware_id for module %d [#502]", __FUNCTION__, xhfc->modidx);
		bri_post_failure(err_str);
		return -ENOMEM;
	}

	for (i = 0; i < xhfc->num_spans; i++)
	{
		err = brispan_create(xhfc, i);
		if (err)
		{
			brispan_cleanup_spans(xhfc);
			kfree(xhfc->chan);
			return err;
		}
	}

	return 0;
}

int __init brie_create_xhfc(struct brie *dev)
{
	int err;
	unsigned imr;
	unsigned id = 0;

	dev->num_xhfcs = 1;

	dev->xhfc = kzalloc(sizeof(struct xhfc) * dev->num_xhfcs, GFP_KERNEL);
	if (!dev->xhfc)
	{
		bri_post_failure("brie_create_xhfc: NO MEMORY [#503]");
		return -ENOMEM;
	}

	if (dev->moda)
	{
		imr |= 1 << 20;
	}
	if (dev->modb)
	{
		imr |= 1 << 21;
	}

	if (dev->moda)
	{
		printk(KERN_INFO "  BRI in module A\n");
		err = bri_module_create(dev, id, 0);
		id++;
		if (err)
			goto error_cleanup;
	}
	// no more modb

	/* initial clock config */
	set_clock(dev);

	if (debug)
		printk("brie_create_xhfc exiting\n");

	return 0;

error_cleanup:
	kfree(dev->xhfc);
	return err;
}

int __init brie_create(struct brie *dev)
{
	int rc;

	rc = brie_create_xhfc(dev);
	if (rc)
		return rc;

	brie_init(dev);

	return 0;
}

static struct proc_dir_entry *proc_frame;
static int frame_proc_show(struct seq_file *m, void *v)
{
	int i = 0;

	for (i = 0; i < g_brie->num_xhfcs; i++)
	{
		struct xhfc *xhfc = &g_brie->xhfc[i];
		seq_printf(m,
				   "%c: XHFC-%s R_AF0_OVIEW: %02x  "
				   "R_BERT_STA: %02x  "
				   "FIFO: rx %u tx %u\n",
				   xhfc->modidx ? 'B' : 'A',
				   xhfc->num_spans == 4 ? "4SU " : "2S4U",
				   xhfc->r_af0_oview, xhfc->r_bert_sta,
				   xhfc->rx_fifo_errs, xhfc->tx_fifo_errs);
	}
	seq_printf(m, "TX CNT: %d\t RX CNT:%d\n", tx_cnt, rx_cnt);
	seq_printf(m, "SU INT CNT: %d\t TOTAL CNT: %d\n", su_int_cnt, cnt);
	seq_printf(m, "SU INT STA: %d\t\n", su_irq_sta);
	seq_printf(m, "FIFO INT  CNT: %d\t READ CNT: %d\t WRITE CTN: %d\n", fifo_int_cnt, fifo_int_read_cnt, fifo_int_write_cnt);
	seq_printf(m, "TOTAL INT CNT: %d\t\n", total_int_cnt);
	seq_printf(m, "QUEUE INT CNT: %d\t\n", queue_int_cnt);
	seq_printf(m, "GLOBA IRQ REG: %02x\t\n", glob_irq);
	seq_printf(m, "L1 TIMER START: %d\t\n", l1_timer_cnt);
	seq_printf(m, "L1 TIMER EXPIR: %d\t\n", l1_timer_expire_cnt);
	seq_printf(m, "T3 TIMER START: %d\t\n", t3_timer_cnt);
	seq_printf(m, "T3 TIMER EXPIR: %d\t\n", t3_timer_expire_cnt);
	seq_printf(m, "T4 TIMER START: %d\t\n", t4_timer_cnt);
	seq_printf(m, "T4 TIMER EXPIR: %d\t\n", t4_timer_expire_cnt);
	seq_printf(m, "ALARM TIMER   : %d\t\n", alarm_cnt);
	seq_printf(m, "ALARM TIMER EXP: %d\t DAHDI CALLED %d\n", alarm_expire_cnt, alarm_dahdi_cnt);
	seq_printf(m, "READ PLE   : %08x\t\n", read_chunk);
	seq_printf(m, "WRITE PLE  : %08x\t\n", write_chunk);

	return 0;
}

static int frame_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, frame_proc_show, NULL);
}

static const struct proc_ops frame_proc_ops = {
	.proc_open = frame_proc_open,
	.proc_read = seq_read,
	.proc_lseek = seq_lseek,
	// .release	= single_release,
};

extern struct device *get_thunder_device(void);

static int __init brie_init_module(void)
{
	int i, err = -EINVAL, pref = 1;
	unsigned rev;
	char err_str[256];
	struct device *thdev = NULL;

	// Init Workqueue
	brie_wq = alloc_workqueue("auk_brie_wq", 0, 0);
	if (!brie_wq)
	{
		printk(KERN_ERR __FILE__ " File to allocate workqueue\n");
		return -ENOMEM;
	}

	/* Create the private data */
	g_brie = kzalloc(sizeof(struct brie), GFP_KERNEL);
	if (!g_brie)
	{
		printk(KERN_ERR __FILE__ " NO MEMORY\n");
		return -ENOMEM;
	}
	memset(g_brie, 0, sizeof(struct brie));
	g_brie->pcm_master = -2;

	thdev = get_thunder_device();
	if (thdev == NULL)
	{
		printk("Can't Retrieve device\n");
		return -ENOMEM;
	}

	g_brie->dma = NULL;
	// Keep back warps settings
	g_brie->fpga = NULL;
	g_brie->irq = -1;
	g_brie->dev = thdev;

#ifdef USE_TASKLET
	DECLARE_TASKLET(&g_brie->brie_bh, brie_tasklet, (unsigned long)g_brie);
	// tasklet_init(&g_brie->brie_bh, brie_tasklet, (unsigned long)g_brie);
#else
	INIT_WORK(&g_brie->brie_work, brie_worker);
#endif
	rev = 1;
	g_brie->moda = 1;
	g_brie->modb = 0;
	g_brie->txfifo = get_txfifo();
	g_brie->rxfifo = get_rxfifo();

	// Get TX/RX Buffers
	// aukdma_get_buffers(&g_brie->rx_buf, &g_brie->tx_buf);

	printk(KERN_INFO "au kbri starting...\n");

	/* GSM uses the same bits as BRI */
	if (g_brie->moda)
		bri_int_mask |= 1 << 20;
	if (g_brie->modb)
		bri_int_mask |= 1 << 21;

	err = brie_create(g_brie);
	if (debug)
		printk("brie_create: return with: %d\n", err);

	if (err)
	{
		sprintf(err_str, "brie_create: failed with err: %d [#505]", err);
		bri_post_failure(err_str);
		goto irq_cleanup;
	}

	proc_frame = proc_create("driver/bri", 0, NULL, &frame_proc_ops);

	printk("Starting Dahdi Registeration\n");
	/* Register with dahdi - do this last */
	for (i = 0; i < MAX_SPANS; ++i)
		if (g_brie->spans[i])
		{
			int card_id = g_brie->spans[i]->modidx;

			if (card_id == 1 && g_brie->num_xhfcs == 1)
				card_id--;

			pr_info("i: %d card_id: %d - %s\n", i, card_id, g_brie->spans[i]->span.name);
			list_add_tail(&g_brie->spans[i]->span.device_node, &g_brie->xhfc[card_id].ddev->spans);

			pref = 0;
		}

	for (i = 0; i < g_brie->num_xhfcs; i++)
	{
		printk("Starting dahdi_register_device: %d \n", i);
		if (dahdi_register_device(g_brie->xhfc[i].ddev, g_brie->dev))
		{
			sprintf(err_str, "Unable to register span with DAHDI %d [#508]", i);
			bri_post_failure(err_str);
			kfree(g_brie->xhfc[i].ddev->location);
			dahdi_free_device(g_brie->xhfc[i].ddev);
			g_brie->xhfc[i].ddev = NULL;
			goto register_cleanup;
		}
	}

	/* And start it */
	brie_enable_interrupts(g_brie);

	err = brie_dma_enable(g_brie);
	if (err)
	{
		sprintf(err_str, "error: %d in brie_dma_enable [#509]", err);
		bri_post_failure(err_str);
		goto register_cleanup;
	}

#ifdef CONFIG_POST
	post_pass(POST_BRI_DVR_ID, "BRI Module - initialized successfully");
#endif
	return 0;

register_cleanup:

	for (i = 0; i < g_brie->num_xhfcs; i++)
	{
		dahdi_unregister_device(g_brie->xhfc[i].ddev);
		kfree(g_brie->xhfc[i].ddev->location);
		dahdi_free_device(g_brie->xhfc[i].ddev);
	}
	if (proc_frame)
		remove_proc_entry("driver/bri", NULL);

	// remove_proc_entry("driver/bri", NULL);

irq_cleanup:
	if (g_brie->irq)
		free_irq(g_brie->irq, g_brie);

// error_cleanup:
// 	g_brie->fpga = NULL;
// 	g_brie->dma = NULL;

// 	destroy_workqueue(brie_wq);

// 	kfree(g_brie);

	return err;
}
module_init(brie_init_module);

static void __exit brie_destroy_xhfc(struct brie *dev)
{
	struct xhfc *xhfc;
	int i;

	xhfc = &dev->xhfc[0];

	if (!dev->xhfc)
		return;

	// tasklet_kill(&dev->brie_bh);

	flush_workqueue(brie_wq);

	for (i = 0; i < MAX_SPANS; ++i)
	{
		if (dev->spans[i])
		{
			brispan_destroy(dev->spans[i]);
			dev->spans[i] = NULL;
		}
	}

	kfree(xhfc->chan);

	kfree(dev->xhfc);
}

void __exit brie_exit_module(void)
{
	int i;

	brie_stop(g_brie);

	for (i = 0; i < g_brie->num_xhfcs; i++)
	{
		dahdi_unregister_device(g_brie->xhfc[i].ddev);
		kfree(g_brie->xhfc[i].ddev->location);
		dahdi_free_device(g_brie->xhfc[i].ddev);
	}

	brie_destroy_xhfc(g_brie);
	if (g_brie->irq)
		free_irq(g_brie->irq, g_brie);

	if (proc_frame)
		remove_proc_entry("driver/bri", NULL);
	g_brie->fpga = NULL;
	g_brie->dma = NULL;

	destroy_workqueue(brie_wq);

	kfree(g_brie);
}
module_exit(brie_exit_module);

MODULE_DESCRIPTION("AUK BRI Driver");
MODULE_AUTHOR("Li Yuqian <yuqian.li@switchpi.com>");
MODULE_LICENSE("GPL");
