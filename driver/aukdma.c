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

#include <asm/page.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/spi/spi.h>
#include <linux/platform_data/dma-bcm2708.h>

#include <linux/string.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/sched.h>

#include <linux/list.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/time.h>
#include <linux/proc_fs.h>
#include <linux/kfifo.h>
#include <linux/semaphore.h>

#define BCM2708_PERI_BASE 0xfe000000
//i2s stuff
#define I2S_BASE                 (BCM2708_PERI_BASE + 0x203000) /* i2s controller */
#define GPIO_BASE                (BCM2708_PERI_BASE + 0x200000)

#define SPI0_BASE                (BCM2708_PERI_BASE + 0x204000)

#define CS_A     0
#define FIFO_A   1
#define MODE_A   2
#define RXC_A    3
#define TXC_A    4
#define DREQ_A   5
#define INTEN_A  6
#define INTSTC_A 7
#define GRAY     8
// GPIO setup macros. Always use INP_GPIO(x) before using OUT_GPIO(x) or SET_GPIO_ALT(x,y)
#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g) *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))
#define SET_GPIO_ALT(g,a) *(gpio+(((g)/10))) |= (((a)<=3?(a)+4:(a)==4?3:2)<<(((g)%10)*3))

#define GPIO_SET *(gpio+7)  // sets   bits which are 1 ignores bits which are 0
#define GPIO_CLR *(gpio+10) // clears bits which are 1 ignores bits which are 0

#define GPIO_SETB(g) GPIO_SET |= 1 <<(g)
#define GPIO_CLRB(g) GPIO_CLR |= 1 <<(g)

#define GPIO_READ(g)  (*(gpio + 13) &= (1<<(g)))
#define CORE_CLOCK 250000000


#define HARDWARE_BUFFER_PERIOD_BYTES 32//320/2 20ms buffers, 8 trunk is one ms is 32 (8*4 as dma transfer with one byte, but each time is 32 bits), 20*32=640 
#define HARDWARE_BUFFER_BYTES HARDWARE_BUFFER_PERIOD_BYTES //* 2
#define TX_FIFO_SIZE 1024*2 //* 2
#define RX_FIFO_SIZE 512*2 //* 2

#define FRAMES_PER_TRANSFER 8

#define DRV_NAME	"aukdma"

int aukdma_major =   236;
int aukdma_devs =    1;
int aukdma_radiopower = 0;
module_param(aukdma_major, int, 0);
module_param(aukdma_radiopower, int, 0);

unsigned int  *gpio;
unsigned int *i2s_registers;
static int device_in_use = 0;
unsigned int *spi_base = NULL;
unsigned int *spi_fifo = NULL;

static DEFINE_MUTEX(read_lock);

/* lock for procfs write access */
static DEFINE_MUTEX(write_lock);

struct dma_inst {
	struct list_head list;
	int cardid;
	void * arg;
	void (*cb)(int cardid,void *arg);
};

struct gsm_pi_dev {
	struct device		*dev;
	struct dma_chan		*dma_tx;	
	struct dma_chan		*dma_rx;
	void __iomem *regs;	
	struct dma_slave_config dma_tx_slave_config;
	struct dma_async_tx_descriptor *dma_tx_desc;

	struct dma_slave_config dma_rx_slave_config;
	struct dma_async_tx_descriptor *dma_rx_desc;

	dma_cookie_t dma_tx_cookie;
	dma_cookie_t dma_rx_cookie;

	u32 *tx_ping_buffer;
	//u32 *tx_pong_buffer;
	dma_addr_t tx_ping_buffer_dma;
	//dma_addr_t tx_pong_buffer_dma;
	u32 *rx_ping_buffer;
	//u32 *tx_pong_buffer;
	dma_addr_t rx_ping_buffer_dma;
	spinlock_t spinlock;
	DECLARE_KFIFO(rxfifo, u16, RX_FIFO_SIZE);
	DECLARE_KFIFO(txfifo, u16, TX_FIFO_SIZE);

	struct list_head list;

	bool dma_status;

	struct cdev cdev;
	struct mutex mutex;
};

//struct gsm_pi_dev *aukdma_devices;
struct gsm_pi_dev *g_dma;

static int tcnt, rcnt, fifo_txcnt, fifo_rxcnt, write_length, read_length;

#define CODEC_RESET 17
#define CODEC_CS0 25

void init_gpio_cs(void)
{
    INP_GPIO(CODEC_RESET);
    OUT_GPIO(CODEC_RESET);
    INP_GPIO(CODEC_CS0);
    OUT_GPIO(CODEC_CS0);
    //initial H
	GPIO_CLRB(CODEC_RESET);//keep reset Low
	GPIO_SETB(CODEC_CS0);
}

static void codec_reset(void)
{
	udelay(100);
	GPIO_SETB(CODEC_RESET);
	udelay(200);
}

void set_codec_cs(int cs)
{
	switch (cs){
	case 0:
		GPIO_SETB(CODEC_CS0);
		break;
	default:
		break;
	}
}

void clr_codec_cs(int cs)
{
	switch (cs){
	case 0:
		GPIO_CLRB(CODEC_CS0);
		break;
	default:
		break;
	}
}

static int __init_spi_bus(void){
	int div;
	spi_base = ioremap(SPI0_BASE, SZ_4K);
	if (!spi_base) {
		printk("could not remap memory\n");
		return 1;
	}
	div = CORE_CLOCK / 2500000;//10M

	*spi_base             = 0x00;
	*spi_base            |= ((1<<4) | (1<<5)); // clear tx,rx fifo
	*spi_base            |= ((1<<2) | (1<<3)); // setup SPI mode 0 : CPOL and CPHA = 1
	*(spi_base + 2 ) = div;  // setup CLK_DIV as div
	*(spi_base)          &=~((1<<0) | (1<<1));// chip select :CS0
	*(spi_base)          &=~( 1<<21) ; // CSPOL0
	spi_fifo = spi_base + 1;
	return 0;
}

static int __stop_spi_bus(void){
	*spi_base &=~(1<<7); // clear TA
	*spi_base             = 0x00;
	*(spi_base + 2 ) = 0;
	iounmap(spi_base);
	return 0;
}

static void __spi_write(unsigned char cmd, int cs) {
	char temp_rx = 0x0;
	unsigned int poll_time_tx = 0xffff;
	//*spi_base &=~(1<<7); // clear TA
	// clr_codec_cs(cs);
	*spi_base            |= (1<<5); // clear tx,rx fifo
	*spi_base            |= (1<<7); // set TA
	while( !(readl(spi_base) & (1<<18)) & (--poll_time_tx > 0));//TXD
	writel(cmd, spi_fifo);
	while(!(readl(spi_base) & (1<<17)) & (--poll_time_tx > 0));//RXD
	while(!(readl(spi_base) & (1<<16)) & (--poll_time_tx > 0));//TX DONE
	temp_rx = readl(spi_fifo);
	*spi_base &=~(1<<7); // clear TA
	// set_codec_cs(cs);
}

static unsigned char __spi_read(int cs) {
	char temp_rx = 0x0;
	unsigned int poll_time_tx = 0xfff;
	// clr_codec_cs(cs);
	*spi_base            |= (1<<7); // set TA
	while( !(readl(spi_base) & (1<<18))  & (--poll_time_tx > 0));
	writel(0xff, spi_fifo);
	while(!(readl(spi_base) & (1<<16)) & (--poll_time_tx > 0));
	while(!(readl(spi_base) & (1<<17)) & (--poll_time_tx > 0));
	temp_rx = readl(spi_fifo);
	*spi_base &=~(1<<7); // clear TA
	// set_codec_cs(cs);
	return temp_rx;
}

void xhfc_write(unsigned char add, unsigned char val, int cs)
{
	// udelay(2);
	clr_codec_cs(cs);
	__spi_write(0x40| 0x00| 0x0, cs);
	__spi_write(add, cs);
	udelay(2);
	set_codec_cs(cs);
	udelay(2);
	clr_codec_cs(cs);
	__spi_write(0x0, cs);
	__spi_write(val, cs);
	udelay(2);
	set_codec_cs(cs);
	udelay(2);
}
EXPORT_SYMBOL(xhfc_write);

unsigned char xhfc_read(unsigned char add, int cs)
{
	unsigned char ret;

	// udelay(2);
	clr_codec_cs(cs);
	__spi_write(0x40| 0x00| 0x0, cs);
	__spi_write(add, cs);
	udelay(2);
	set_codec_cs(cs);
	udelay(2);
	clr_codec_cs(cs);
	udelay(2);
	__spi_write(0x00| 0x80 | 0x0, cs);
	ret = __spi_read(cs);
	udelay(2);
	set_codec_cs(cs);
	udelay(2);

	return ret;
}
EXPORT_SYMBOL(xhfc_read);

static void setup_gpio(void)
{

	int pin;
	gpio = ioremap(GPIO_BASE, SZ_16K);

	for (pin = 18; pin <= 21; pin++) {
		INP_GPIO(pin);		/* set mode to GPIO input first */
		SET_GPIO_ALT(pin, 0);	/* set mode to ALT 0 */
	}

	/* SPI is on GPIO 7..11 */
	for (pin = 9; pin <= 11; pin++) {
		INP_GPIO(pin);		/* set mode to GPIO input first */
		SET_GPIO_ALT(pin, 0);	/* set mode to ALT 0 */
	}
}

void start_i2s_tx(void)
{
    *(i2s_registers+CS_A) |= 1<<2 | 1<<1;
    printk(KERN_INFO "ENABLED I2S\n");   
}

void stop_i2s_tx(void)
{
    *(i2s_registers+CS_A) &= ~(1<<2); 
    *(i2s_registers+CS_A) &= ~(1<<1); 
    printk(KERN_INFO "STOPPED I2S\n");   
}

void setup_i2s(void)
{

    int i=0;
    i2s_registers = ioremap(I2S_BASE, 32);

    *(i2s_registers+CS_A) &= ~(1<<24); //just for completeness
    *(i2s_registers+CS_A) = 0;
    *(i2s_registers+MODE_A) = 0;
    *(i2s_registers+TXC_A) = 0;
    *(i2s_registers+RXC_A) = 0;
    *(i2s_registers+GRAY) = 0;
    *(i2s_registers+DREQ_A) = 0;
    udelay(100);
    *(i2s_registers+TXC_A) = 1<<31 | 1<<30  | 8<<16  | 1<<20;
    *(i2s_registers+RXC_A) = 1<<31 | 1<<30  | 8<<16  | 1<<20;
   *(i2s_registers+MODE_A) = 1<< 23 | 255 << 10 | 1 | 1 << 21;
    // *(i2s_registers+MODE_A) = 1<< 23 | 255 << 10 | 1 ;
    *(i2s_registers+CS_A) |= 1<<25;
    udelay(50);
    *(i2s_registers+CS_A) |= 1<<3 |1<<4 ; 
    *(i2s_registers+CS_A) |= 0x01;

    for(i = 0; i < 32; i++)
       (*(i2s_registers+FIFO_A)) = 0;
    *(i2s_registers+CS_A) |= 1<<24;

    udelay(100);
    *(i2s_registers+CS_A) |=  1<<9 ; 
    *(i2s_registers+DREQ_A) |=  1 <<24 | 16<<16 | 16<<8 | 8;  
    *(i2s_registers+CS_A) |= 1<<3  |1<<4 ; 
    printk(KERN_INFO "I2S SETUP COMPLETE\n");
    return;
}

void __stop_i2s(void)
{
    /* Clear everything - Monty Burns style :) */
    //disable interrupts first
    *(i2s_registers+INTSTC_A) = 0x000f;// clear status bits
    *(i2s_registers+INTEN_A) = 0x00; 
    //*(i2s_registers+CS_A) &= ~(1<<24);
    udelay(100);
    *(i2s_registers+CS_A) = 0;
    *(i2s_registers+MODE_A) = 0;
    *(i2s_registers+TXC_A) = 0;
    *(i2s_registers+RXC_A) = 0;
    *(i2s_registers+GRAY) = 0;
    *(i2s_registers+DREQ_A) = 0;
}


static int tdm_dma_release(struct gsm_pi_dev *master)
{
	if (master->dma_rx)
		dmaengine_pause(master->dma_rx);
	if (master->dma_tx)
		dmaengine_pause(master->dma_tx);

	if (master->dma_tx_cookie) {
		dmaengine_terminate_all(master->dma_tx);
		master->dma_tx_cookie = 0;
	}

	if (master->dma_rx_cookie) {
		dmaengine_terminate_all(master->dma_rx);
		master->dma_rx_cookie = 0;
	}

	if (master->tx_ping_buffer) {
		dma_free_coherent(master->dma_tx->device->dev,
			HARDWARE_BUFFER_BYTES,
			master->tx_ping_buffer,
			master->tx_ping_buffer_dma);
	}

	if (master->rx_ping_buffer) {
		dma_free_coherent(master->dma_rx->device->dev,
			HARDWARE_BUFFER_BYTES,
			master->rx_ping_buffer,
			master->rx_ping_buffer_dma);
	}

	if (master->dma_tx)
		dma_release_channel(master->dma_tx);

	if (master->dma_rx)
		dma_release_channel(master->dma_rx);

	return 0;
}

static inline void process_rx_dma(struct gsm_pi_dev *master){

	struct gsm_pi_dev *dev = master;
	u16 tmpbuf[FRAMES_PER_TRANSFER];
	volatile unsigned int *readchunk;
	int x;

	rcnt++;

	readchunk = dev->rx_ping_buffer;

	for (x=0;x<FRAMES_PER_TRANSFER;x++) {
		tmpbuf[x] = readchunk[x];
		//tmpbuf[x] = 0x5454;
	}

    kfifo_in(&dev->rxfifo, tmpbuf, FRAMES_PER_TRANSFER);

}

static inline void process_tx_dma(struct gsm_pi_dev *master){

	struct gsm_pi_dev *dev = master;
	u16 tmpbuf[FRAMES_PER_TRANSFER];
	volatile unsigned int *writechunk;
	int i;

	tcnt++;
	dev->dma_status = true;

	writechunk = dev->tx_ping_buffer;

	if (kfifo_len(&dev->rxfifo) > FRAMES_PER_TRANSFER){
		i = kfifo_out(&dev->rxfifo, tmpbuf, FRAMES_PER_TRANSFER);
	}
	else {
		//tcnt++;
		memset(tmpbuf, 0xffff, sizeof(tmpbuf));//we are u16, so the size is 2*FRAMES_PER_TRANSFER
	}

	// for (x=0;x<FRAMES_PER_TRANSFER;x++) {
	// 	writechunk[x] = 0x00118888;//tmpbuf[x] << 16;			
	// }

}

static void dma_tx_complete(void *arg)
{
	struct gsm_pi_dev *master = arg;
	unsigned long flags;
	
	spin_lock_irqsave(&master->spinlock, flags);

	//dmaengine_pause(master->dma_rx);
	//dmaengine_pause(master->dma_tx);

	//dmaengine_resume(master->dma_rx);
	//dmaengine_resume(master->dma_tx);

	//process buffers
	spin_unlock_irqrestore(&master->spinlock, flags);

}

static void dma_rx_complete(void *arg)
{
	struct gsm_pi_dev *master = arg;
	unsigned long flags;
	struct dma_inst *inst;
	
	spin_lock_irqsave(&master->spinlock, flags);
	// process_tx_dma(master);
	// process_rx_dma(master);
	
    if (g_dma != NULL)
    {
        list_for_each_entry(inst, &g_dma->list, list)
            inst->cb(inst->cardid, inst->arg);
    }

	//process buffers
	spin_unlock_irqrestore(&master->spinlock, flags);

}

static struct dma_inst *find_inst(struct gsm_pi_dev *dma, int cardid)
{
	struct dma_inst *inst;

	list_for_each_entry(inst, &dma->list, list)
		if (inst->cardid == cardid)
			return inst;

	return NULL;
}

int aukdma_register_cb(int cardid, void (*cb)(int cardid, void * arg), void * arg)
{
    struct gsm_pi_dev *dma = g_dma;
	struct dma_inst *inst;

    char pass[20] = "hello";

	if (cardid == -1)
		return -EINVAL;

	if (find_inst(dma, cardid)) {
		printk(KERN_WARNING
		       "dma_register: cardid %d already registered.\n", cardid);
		return -EBUSY;
	}

	inst = kzalloc(sizeof(struct dma_inst), GFP_KERNEL);
	if (!inst) {
		printk(KERN_ERR "dma_register: out of memory\n");
		return -ENOMEM;
	}

	/* Disable the interrupts while adding to list. */
    stop_i2s_tx();

	inst->cardid = cardid;
	inst->arg = pass;
	inst->cb = cb;
	list_add_tail(&inst->list, &dma->list);

	/* Always safe to re-enable */
    start_i2s_tx();

	return 0;
}
EXPORT_SYMBOL(aukdma_register_cb);

int aukdma_unregister(int cardid)
{
    struct gsm_pi_dev *dma = g_dma;
	struct dma_inst *inst = find_inst(dma, cardid);

	if (!inst)
		return -EINVAL;

	/* Disable ints while removing from list */
    stop_i2s_tx();
	list_del(&inst->list);
	kfree(inst);

	//if empty, just stop it
    if (list_empty(&dma->list))
        stop_i2s_tx();
	else /* if we still have another instance requires int, we need start it again */
		start_i2s_tx();

	return 0;
}
EXPORT_SYMBOL(aukdma_unregister);

static struct device * gdevice;
struct device * get_thunder_device(void) {
	if (gdevice == NULL) return NULL;
	return gdevice;
}
EXPORT_SYMBOL(get_thunder_device);

u32 * get_txfifo(void) {
    struct gsm_pi_dev *dma = g_dma;
	if (dma != NULL) {
		return dma->tx_ping_buffer;
	}else 
	{
		printk("retrive txfifo wrong\n");
		return NULL;
	}

}
EXPORT_SYMBOL(get_txfifo);

u32 * get_rxfifo(void) {
    struct gsm_pi_dev *dma = g_dma;
	if (dma != NULL) {
		return dma->rx_ping_buffer;
	}else 
	{
		printk("retrive rxfifo wrong\n");
		return NULL;
	}

}
EXPORT_SYMBOL(get_rxfifo);


static int bcm2835_tx_pingdma_init(struct gsm_pi_dev *master, struct device *dev)
{
	int err, i;

	dma_addr_t pcm_hardware_reg_base;
	const __be32 *addr;
	/* base address in dma-space */
	addr = of_get_address(dev->of_node, 0, NULL, NULL);
	if (!addr) {
		dev_err(dev, "could not get pcm hardware register address\n");
	}
	pcm_hardware_reg_base = be32_to_cpup(addr);
	//printk("start pcm hardware address %x device on %s\n", pcm_hardware_reg_base, dev->bus->name);

	/* get tx/rx dma */
	master->dma_tx = dma_request_slave_channel(dev, "tx");
	if (!master->dma_tx) {
		dev_err(dev, "no tx-dma configuration found - not using dma mode\n");
		tdm_dma_release(master);
		return -1;
	}

	master->dma_rx = dma_request_slave_channel(dev, "rx");
	if (!master->dma_rx) {
		dev_err(dev, "no rx-dma configuration found - not using dma mode\n");
		tdm_dma_release(master);
		return -1;
	}

	/* configure DMAs */
	master->tx_ping_buffer =
		dma_alloc_coherent(master->dma_tx->device->dev,
		HARDWARE_BUFFER_BYTES,
		&master->tx_ping_buffer_dma,
		GFP_KERNEL);

	if (!master->tx_ping_buffer) {
		dev_err(dev, "could not allocate dma address\n");
		tdm_dma_release(master);
		return -1;
	}
	//printk("tx ping dma address is %x - %x, buffer size is %x\n", master->tx_ping_buffer, master->tx_ping_buffer_dma, HARDWARE_BUFFER_BYTES);

	memset(master->tx_ping_buffer, 0, HARDWARE_BUFFER_BYTES);

	for (i = 0; i < HARDWARE_BUFFER_BYTES; i++){
		master->tx_ping_buffer[i] = i;
	}

	master->rx_ping_buffer =
		dma_alloc_coherent(master->dma_rx->device->dev,
		HARDWARE_BUFFER_BYTES,
		&master->rx_ping_buffer_dma,
		GFP_KERNEL);

	if (!master->rx_ping_buffer) {
		dev_err(dev, "could not allocate rx dma address\n");
		tdm_dma_release(master);
		return -1;
	}
	//printk("rx ping dma address is %x - %x, buffer size is %x\n", master->rx_ping_buffer, master->rx_ping_buffer_dma, HARDWARE_BUFFER_BYTES);

	memset(master->rx_ping_buffer, 0, HARDWARE_BUFFER_BYTES);

	master->dma_tx_slave_config.direction = DMA_MEM_TO_DEV;
	master->dma_tx_slave_config.dst_addr = 0x7e203000 + 0x4;
	master->dma_tx_slave_config.dst_maxburst = 1;
	master->dma_tx_slave_config.dst_addr_width = 4;
	master->dma_tx_slave_config.src_addr = master->tx_ping_buffer_dma;
	master->dma_tx_slave_config.src_maxburst = 1;
	master->dma_tx_slave_config.src_addr_width = 4;

	err = dmaengine_slave_config(master->dma_tx,
		&master->dma_tx_slave_config);

	if (err < 0) {
		printk("could not setup slave_tx_config\n");
		tdm_dma_release(master);
		return err;
	}

	master->dma_rx_slave_config.direction = DMA_DEV_TO_MEM;
	master->dma_rx_slave_config.dst_addr = master->rx_ping_buffer_dma;
	master->dma_rx_slave_config.dst_maxburst = 1;
	master->dma_rx_slave_config.dst_addr_width = 4;
	master->dma_rx_slave_config.src_addr = 0x7e203000 + 0x4;
	master->dma_rx_slave_config.src_maxburst = 1;
	master->dma_rx_slave_config.src_addr_width = 4;

	err = dmaengine_slave_config(master->dma_rx,
		&master->dma_rx_slave_config);

	if (err < 0) {
		printk("could not setup slave_rx_config\n");
		tdm_dma_release(master);
		return err;
	}

	master->dma_tx_desc =
		dmaengine_prep_dma_cyclic(master->dma_tx,
		master->tx_ping_buffer_dma,
		HARDWARE_BUFFER_BYTES,
		HARDWARE_BUFFER_PERIOD_BYTES,
		DMA_MEM_TO_DEV,
		DMA_CTRL_ACK | DMA_PREP_INTERRUPT);

	if (!master->dma_tx_desc) {
		printk("could not setup dma_tx_desc\n");
		tdm_dma_release(master);
		return -1;
	}

	//printk("HARDWARE_BUFFER_BYTES %d - HARDWARE_BUFFER_PERIOD_BYTES %d \n", HARDWARE_BUFFER_BYTES, HARDWARE_BUFFER_PERIOD_BYTES);


	master->dma_rx_desc =
		dmaengine_prep_dma_cyclic(master->dma_rx,
		master->rx_ping_buffer_dma,
		HARDWARE_BUFFER_BYTES,
		HARDWARE_BUFFER_PERIOD_BYTES,
		DMA_DEV_TO_MEM,
		DMA_CTRL_ACK | DMA_PREP_INTERRUPT);

	if (!master->dma_rx_desc) {
		printk("could not setup dma_rx_desc\n");
		tdm_dma_release(master);
		return -1;
	}

	master->dma_tx_desc->callback = dma_tx_complete;
	master->dma_tx_desc->callback_param = master;

	master->dma_rx_desc->callback = dma_rx_complete;
	master->dma_rx_desc->callback_param = master;

	printk("I2S DMA configured by success!\n");
	return 0;
}

static void start_dma(struct gsm_pi_dev *master)
{
	master->dma_tx_cookie = dmaengine_submit(master->dma_tx_desc);
	dma_async_issue_pending(master->dma_tx);

	master->dma_rx_cookie = dmaengine_submit(master->dma_rx_desc);
	dma_async_issue_pending(master->dma_rx);

	printk("DMA submited! dma_tx_cookie:%d\n", master->dma_tx_cookie);
	printk("DMA submited! dma_rx_cookie:%d\n", master->dma_rx_cookie);

}

static struct proc_dir_entry *proc_frame;

static int frame_proc_show(struct seq_file *m, void *v)
{
    // struct gsm_pi_dev *dma = g_dma;
	seq_printf(m, "dma status:\nrx count: %d\ntx count: %d\nfifo_txcnt: %d\nfifo_rxcnt: %d\nwrite_length: %d\nread_length: %d\n\n\n", rcnt, tcnt, fifo_txcnt, fifo_rxcnt, write_length, read_length);
	// if (dma != NULL) {
	// 	seq_printf(m, "TXFIFO: %8x\n", dma->tx_ping_buffer);
	// 	seq_printf(m, "RXFIFO: %8x\n", dma->rx_ping_buffer);
	// }
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

static ssize_t device_read(struct file *file, char __user *buf, size_t length, loff_t *offset)
{	
	struct gsm_pi_dev *dev = file->private_data;
	int ret;
	unsigned int samples_read = 0;
	u16 tmpbuf[FRAMES_PER_TRANSFER];

	if (mutex_lock_interruptible(&dev->mutex))
		return -ERESTARTSYS;

	read_length = length;

	while( (kfifo_len(&dev->rxfifo) > FRAMES_PER_TRANSFER) && length) {
		ret = kfifo_out(&dev->rxfifo, tmpbuf, FRAMES_PER_TRANSFER);
		if (ret > 0) {
			fifo_rxcnt ++;
			if (copy_to_user(buf + (sizeof(tmpbuf) * samples_read), tmpbuf, sizeof(tmpbuf))) {
  	 	 		printk(KERN_ALERT "put to user space wrong\n");
  	 	 		mutex_unlock(&dev->mutex);
  	 	 		return -EFAULT;
  	 		}
  	 	}
  	 	else {
 			printk(KERN_ALERT "get fifo wrong - %d\n", ret);
 			mutex_unlock(&dev->mutex);
 	 		return -EFAULT;
  	 	}
  	 	length -= sizeof(tmpbuf);
  	 	samples_read++;
  	 }

	mutex_unlock(&dev->mutex);

	return sizeof(tmpbuf) * samples_read;   // Number of bytes transferred
}

static ssize_t device_write(struct file *file, const char __user *buf, size_t count, loff_t *offset)
{

	int ret = 0;
	struct gsm_pi_dev *dev = file->private_data;
	u16 tmpbuf[FRAMES_PER_TRANSFER];
	int i, index = 0;

    if (!count)
        return 0;

	if (mutex_lock_interruptible(&dev->mutex))
		return -ERESTARTSYS;
	//dma not running, just return
	if (! dev->dma_status) {
		mutex_unlock(&dev->mutex);
		return count;
	}

	for(i = 0; i < (count / (sizeof(tmpbuf))); i++){
		
		fifo_txcnt ++ ;		
		if (copy_from_user(tmpbuf, buf + index, sizeof(tmpbuf))){
			printk(KERN_ALERT "copy from user space wrong \n");
			mutex_unlock(&dev->mutex);
			return -EFAULT;
		}
		ret = kfifo_in(&dev->txfifo, tmpbuf, FRAMES_PER_TRANSFER);
		if (ret == FRAMES_PER_TRANSFER){
			index += sizeof(tmpbuf);			
		}
		else {
			//printk(KERN_ALERT "put fifo wrong - %d\n", ret);
			mutex_unlock(&dev->mutex);
			return -ERESTARTSYS;
		}
	}

	write_length = count;

	mutex_unlock(&dev->mutex);
	//if (index != count) return count;
	//return writecount;
	return index;

}

/* Called when a process attemps to open the device file */
static int device_open(struct inode *inode, struct file *file)
{
  struct gsm_pi_dev *master;
  u16 tmpbuf[FRAMES_PER_TRANSFER];

  if(device_in_use)
  {
    return -EBUSY;
  }

  device_in_use++;
  printk("AUKDMA is opened %d size is %d\n", device_in_use, sizeof(tmpbuf));
  fifo_txcnt = 0;
  fifo_rxcnt = 0;

  master = container_of(inode->i_cdev, struct gsm_pi_dev, cdev);
  file->private_data = master;

  kfifo_reset(&master->rxfifo);
  kfifo_reset(&master->txfifo);
  master->dma_status = false;

  /* Increment usage count to be able to prooperly close the module. */
  try_module_get(THIS_MODULE);

  return 0;

}

/* Called when the a process closes the device file */
static int device_release(struct inode *inode, struct file *file)
{

  device_in_use--;    /* Make this device available to other processes */

  printk("AUKDMA is released %d\n", device_in_use);

  /* Decrement usage count to be able to properly close the module. */
  module_put(THIS_MODULE);

  return 0;

}

static long device_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
  return 0;
}

static struct file_operations fops = {
	.owner	= THIS_MODULE,
	.read  = device_read,
	.write = device_write,
	.open = device_open,
	.release = device_release,
	.unlocked_ioctl = device_ioctl
};


static void aukdma_setup_cdev(struct gsm_pi_dev *dev, int index)
{
	int err, devno = MKDEV(aukdma_major, index);
    
	cdev_init(&dev->cdev, &fops);
	dev->cdev.owner = THIS_MODULE;
	dev->cdev.ops = &fops;
	err = cdev_add (&dev->cdev, devno, 1);
	/* Fail gracefully if need be */
	if (err)
		printk(KERN_NOTICE "Error %d adding aukdma%d", err, index);
}

static int pi_gsm_probe(struct platform_device *pdev)
{
	struct gsm_pi_dev *master;
	struct resource *res;
	int i;
	int result;
	dev_t dev;

	master = devm_kzalloc(&pdev->dev, sizeof(*master),
			   GFP_KERNEL);
	if (!master) {
		printk("dev alloc error\n");
		return -ENOMEM;
	}
	memset(master, 0, aukdma_devs*sizeof (struct gsm_pi_dev));
	
	INIT_KFIFO(master->txfifo);
	INIT_KFIFO(master->rxfifo);
	INIT_LIST_HEAD(&master->list);


	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	master->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(master->regs)) {
		printk("error 1\n");
	}

	dev = MKDEV(aukdma_major, 0);
	if (aukdma_major)
		result = register_chrdev_region(dev, aukdma_devs, "aukdma");
	else {
		result = alloc_chrdev_region(&dev, 0, aukdma_devs, "aukdma");
		aukdma_major = MAJOR(dev);
	}

	if (result < 0){
		printk(KERN_ALERT "can't register device\n");
		kfree(master);
		return result;
	}

	for (i = 0; i < aukdma_devs; i++) {
		//scullc_devices[i].quantum = scullc_quantum;
		//scullc_devices[i].qset = scullc_qset;
		mutex_init (&master[i].mutex);
		aukdma_setup_cdev(master + i, i);
	}
	
	platform_set_drvdata(pdev, master);
    g_dma = master;
	gdevice = &pdev->dev;

	// printk("probe done - %x\n", master->regs);

	printk(KERN_INFO "AUKDMA driver successfully assigned to major number %d.", aukdma_major);

	proc_frame = proc_create("frame", 0, NULL, &frame_proc_ops);

	setup_gpio();

	init_gpio_cs();
	__init_spi_bus();

	setup_i2s();
	bcm2835_tx_pingdma_init(master, &pdev->dev);
	start_i2s_tx();
	start_dma(master);

	printk("spi write test\n");
	codec_reset();
	xhfc_write(0xc0, 0x88, 0);
	for (i = 0; i < 255; i ++) {
		xhfc_write(0xc0, i, 0);
		if (xhfc_read(0x16, 0) != 0x63) printk ("xhfc spi read is failed on %d!!!\n", i);
		if (xhfc_read(0xc0, 0) != i)
			printk ("xhfc spi test is failed on %d!!!\n", i);
	}

	return 0;

}

static int pi_gsm_remove(struct platform_device *pdev)
{
	struct gsm_pi_dev *master;
	int i;

	master = platform_get_drvdata(pdev);
	
	tdm_dma_release(master);

	__stop_i2s();
	__stop_spi_bus();
	
	GPIO_CLRB(CODEC_RESET);//keep reset Low
    iounmap(i2s_registers);

	if (gpio != NULL)
		iounmap(gpio);
	
	if (proc_frame)
		remove_proc_entry("frame", NULL);	
	
	for (i = 0; i < aukdma_devs; i++) {
		cdev_del(&master[i].cdev);
		//aukdma_trim(aukdma_devices + i);
	}

	//kfree(master);	
	unregister_chrdev_region(MKDEV (aukdma_major, 0), aukdma_devs);
	return 0;
}

static const struct of_device_id pi_gsm_match[] = {
	{ .compatible = "brcm,pi-tdm", },
	{}
};

MODULE_DEVICE_TABLE(of, pi_gsm_match);

static struct platform_driver pi_gsm_driver = {
	.driver		= {
		.name		= DRV_NAME,
		.of_match_table	= pi_gsm_match,
	},
	.probe		= pi_gsm_probe,
	.remove		= pi_gsm_remove,
};
module_platform_driver(pi_gsm_driver);

MODULE_DESCRIPTION("SwitchPi RPi DMA module");
MODULE_AUTHOR("Li Yuqian <yuqian.li@switchpi.com>");
MODULE_LICENSE("GPL v2");