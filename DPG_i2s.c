#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/mm.h>
#include <linux/ioctl.h>
#include <linux/ioport.h>
#include <asm/barrier.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
/*************************** BEGIN IOCTL DECLARATIONS **************************/

#define I2S_SET_EN          _IOW('i', 0, char)
#define I2S_SET_TXON        _IOW('i', 1, char)
#define I2S_SET_RXON        _IOW('i', 2, char)
#define I2S_TX_BUFF_SPACE   _IOR('i', 3, int)
#define I2S_RX_BUFF_ITEMS   _IOR('i', 4, int)
#define I2S_CLEAR_TX_BUFF   _IOW('i', 5, char)
#define I2S_CLEAR_RX_BUFF   _IOW('i', 6, char)
#define I2S_WRITE_CS_A      _IOW('i', 7, uint32_t)
#define I2S_WRITE_MODE_A    _IOW('i', 8, uint32_t)
#define I2S_WRITE_RXC_A     _IOW('i', 9, uint32_t)
#define I2S_WRITE_TXC_A     _IOW('i', 10, uint32_t)
#define I2S_WRITE_DREQ_A    _IOW('i', 11, uint32_t)
#define I2S_WRITE_INTEN_A   _IOW('i', 12, uint32_t)
#define I2S_WRITE_INTSTC_A  _IOW('i', 13, uint32_t)
#define I2S_WRITE_GRAY      _IOW('i', 14, uint32_t)
#define I2S_CLR_TX_FIFO     _IOW('i', 15, char)
#define I2S_CLR_RX_FIFO     _IOW('i', 16, char)

/*************************** END IOCTL DECLARATIONS **************************/

/* Bits for CS_A Register */
#define I2S_CS_A_STBY             (0x1u << 25)
#define I2S_CS_A_SYNC             (0x1u << 24)
#define I2S_CS_A_RXSEX            (0x1u << 23)
#define I2S_CS_A_RXF_MASK         (0x1u << 22)
#define I2S_CS_A_TXE_MASK         (0x1u << 21)
#define I2S_CS_A_RXD_MASK         (0x1u << 20)
#define I2S_CS_A_TXD_MASK         (0x1u << 19)
#define I2S_CS_A_RXR_MASK         (0x1u << 18)
#define I2S_CS_A_TXW_MASK         (0x1u << 17)
#define I2S_CS_A_RXERR            (0x1u << 16)
#define I2S_CS_A_TXERR            (0x1u << 15)
#define I2S_CS_A_RXSYNC_MASK      (0x1u << 14)
#define I2S_CS_A_TXSYNC_MASK      (0x1u << 13)
#define I2S_CS_A_DMAEN            (0x1u << 9)
#define I2S_CS_A_RXTHR(val)       ((val << 7) & (0x3 << 7))
#define I2S_CS_A_TXTHR(val)       ((val << 5) & (0x3 << 5))
#define I2S_CS_A_RXCLR            (0x1u << 4)
#define I2S_CS_A_TXCLR            (0x1u << 3)
#define I2S_CS_A_TXON             (0x1u << 2)
#define I2S_CS_A_RXON             (0x1u << 1)
#define I2S_CS_A_EN               (0x1u << 0)

/* Bits for MODE_A Register */
#define I2S_MODE_A_CLK_DIS        (0x1u << 28)
#define I2S_MODE_A_PDMN           (0x1u << 27)
#define I2S_MODE_A_PDME           (0x1u << 26)
#define I2S_MODE_A_FRXP           (0x1u << 25)
#define I2S_MODE_A_FTXP           (0x1u << 24)
#define I2S_MODE_A_CLKM           (0x1u << 23)
#define I2S_MODE_A_CLKI           (0x1u << 22)
#define I2S_MODE_A_FSM            (0x1u << 21)
#define I2S_MODE_A_FSI            (0x1u << 20)
#define I2S_MODE_A_FLEN_POS       (10)
#define I2S_MODE_A_FLEN_MASK      (0x3FF << I2S_MODE_A_FLEN_POS)
#define I2S_MODE_A_FLEN(val)      (I2S_MODE_A_FLEN_MASK & (val << I2S_MODE_A_FLEN_POS))
#define I2S_MODE_A_FSLEN_POS      (0)
#define I2S_MODE_A_FSLEN_MASK     (0x3FF << I2S_MODE_A_FSLEN_POS)
#define I2S_MODE_A_FSLEN(val)     (I2S_MODE_A_FSLEN_MASK & (val << I2S_MODE_A_FSLEN_POS))

/* Bits for RXC_A Register */
#define I2S_RXC_A_CH1WEX          (0x1u << 31)
#define I2S_RXC_A_CH1EN           (0x1u << 30)
#define I2S_RXC_A_CH1POS_POS      (20)
#define I2S_RXC_A_CH1POS_MASK     (0x3FF << I2S_RXC_A_CH1POS_POS)
#define I2S_RXC_A_CH1POS(val)     (I2S_RXC_A_CH1POS_MASK & (val << I2S_RXC_A_CH1POS_POS))
#define I2S_RXC_A_CH1WID_POS      (16)
#define I2S_RXC_A_CH1WID_MASK     (0xFu << I2S_RXC_A_CH1WID_POS)
#define I2S_RXC_A_CH1WID(val)     (I2S_TXC_A_CH1WID_MASK & (val << I2S_RXC_A_CH1WID_POS))
#define I2S_RXC_A_CH2WEX          (0x1u << 15)
#define I2S_RXC_A_CH2EN           (0x1u << 14)
#define I2S_RXC_A_CH2POS_POS      (4)
#define I2S_RXC_A_CH2POS_MASK     (0x3FFu << I2S_RXC_A_CH2POS_POS)
#define I2S_RXC_A_CH2POS(val)     (I2S_RXC_A_CH2POS_MASK & (val << I2S_RXC_A_CH2POS_POS))
#define I2S_RXC_A_CH2WID_POS      (0)
#define I2S_RXC_A_CH2WID_MASK     (0xFu << I2S_RXC_A_CH2WID_POS)
#define I2S_RXC_A_CH2WID(val)     (I2S_RXC_A_CH2WID_MASK & (val << I2S_RXC_A_CH2WID_POS))

/* Bits for TXC_A Register */
#define I2S_TXC_A_CH1WEX          (0x1u << 31)
#define I2S_TXC_A_CH1EN           (0x1u << 30)
#define I2S_TXC_A_CH1POS_POS      (20)
#define I2S_TXC_A_CH1POS_MASK     (0x3FF << I2S_TXC_A_CH1POS_POS)
#define I2S_TXC_A_CH1POS(val)     (I2S_TXC_A_CH1POS_MASK & (val << I2S_TXC_A_CH1POS_POS))
#define I2S_TXC_A_CH1WID_POS      (16)
#define I2S_TXC_A_CH1WID_MASK     (0xFu << I2S_TXC_A_CH1WID_POS)
#define I2S_TXC_A_CH1WID(val)     (I2S_TXC_A_CH1WID_MASK & (val << I2S_TXC_A_CH1WID_POS))
#define I2S_TXC_A_CH2WEX          (0x1u << 15)
#define I2S_TXC_A_CH2EN           (0x1u << 14)
#define I2S_TXC_A_CH2POS_POS      (4)
#define I2S_TXC_A_CH2POS_MASK     (0x3FFu << I2S_TXC_A_CH2POS_POS)
#define I2S_TXC_A_CH2POS(val)     (I2S_TXC_A_CH2POS_MASK & (val << I2S_TXC_A_CH2POS_POS))
#define I2S_TXC_A_CH2WID_POS      (0)
#define I2S_TXC_A_CH2WID_MASK     (0xFu << I2S_TXC_A_CH2WID_POS)
#define I2S_TXC_A_CH2WID(val)     (I2S_TXC_A_CH2WID_MASK & (val << I2S_TXC_A_CH2WID_POS))

/* Bits for DREQ_A Register */
#define I2S_DREQ_A_TX_PANIC_POS   (24)
#define I2S_DREQ_A_TX_PANIC_MASK  (0x7Fu << I2S_DREQ_A_TX_PANIC_POS)
#define I2S_DREQ_A_TX_PANIC(val)  (I2S_DREQ_A_TX_PANIC_MASK & (val << I2S_DREQ_A_TX_PANIC_POS))
#define I2S_DREQ_A_RX_PANIC_POS   (16)
#define I2S_DREQ_A_RX_PANIC_MASK  (0x7Fu << I2S_DREQ_A_RX_PANIC_POS)
#define I2S_DREQ_A_RX_PANIC(val)  (I2S_DREQ_A_RX_PANIC_MASK & (val << I2S_DREQ_A_RX_PANIC_POS))
#define I2S_DREQ_A_TX_POS         (8)
#define I2S_DREQ_A_TX_MASK        (0x7Fu << I2S_DREQ_A_TX_POS)
#define I2S_DREQ_A_TX(val)        (I2S_DREQ_A_TX_MASK & (val << I2S_DREQ_A_TX_POS))
#define I2S_DREQ_A_RX_POS         (0)
#define I2S_DREQ_A_RX_MASK        (0x7Fu << I2S_DREQ_A_RX_POS)
#define I2S_DREQ_A_RX(val)        (I2S_DREQ_A_RX_MASK & (val << I2S_DREQ_A_RX_POS))

/* Bits for INTEN_A Register */
#define I2S_INTEN_A_RXERR         (0x1u << 3)
#define I2S_INTEN_A_TXERR         (0x1u << 2)
#define I2S_INTEN_A_RXR           (0x1u << 1)
#define I2S_INTEN_A_TXW           (0x1u << 0)

/* Bits for INTSTC_A Register */
#define I2S_INTSTC_A_RXERR        (0x1u << 3)
#define I2S_INTSTC_A_TXERR        (0x1u << 2)
#define I2S_INTSTC_A_RXR          (0x1u << 1)
#define I2S_INTSTC_A_TXW          (0x1u << 0)

/* Bits for GRAY Register */
#define I2S_GRAY_RXFIFOLEVEL_POS  (16)
#define I2S_GRAY_RXFIFOLEVEL_MASK (0x3Fu << I2S_GRAY_RXFIFOLEVEL_POS)
#define I2S_GRAY_FLUSHED_POS      (10)
#define I2S_GRAY_FLUSHED_MASK     (0x3Fu << I2S_GRAY_FLUSHED_POS)
#define I2S_GRAY_RXLEVEL_POS      (4)
#define I2S_GRAY_RXLEVEL_MASK     (0x3Fu << I2S_GRAY_RXLEVEL_POS)
#define I2S_GRAY_FLUSH            (0x1u << 2)
#define I2S_GRAY_CLR              (0x1u << 1)
#define I2S_GRAY_EN               (0x1u << 0)

#define MODULE_NAME "DPG_i2s"

int MAJOR = 230;

#define M_INPUT     0 //gpio function select macro
#define M_OUTPUT    1

#define S_LOW        0//gpio status macro
#define S_HIGH       1

#define BYTES_PER_SAMPLE        4    // Everything is stored in 32 bits
#define SAMPLE_BUFF_LEN         16000
#define SAMPLE_BUFF_LEN_BYTES   BYTES_PER_SAMPLE * SAMPLE_BUFF_LEN

//I/O base address in kernel mode
#define BCM2835_PERI_BASE    0x3F000000
//GPIO base address in kernel mode
#define GPIO_BASE (BCM2835_PERI_BASE + 0x00200000)
#define I2S_BASE  (BCM2835_PERI_BASE + 0x00203000)
#define I2S_SIZE  0x24
#define I2S_INTERRUPT 79
static const int LedGpioPin = 16;           // Pin LED is connected to
static int led_status = 0;                    // 0-off, 1-on
static struct kobject *led_kobj;

typedef struct GpioRegisters{
    volatile uint32_t GPFSEL[6];
    volatile uint32_t Reserved1;
    volatile uint32_t GPSET[2];
    volatile uint32_t Reserved2;
    volatile uint32_t GPCLR[2];
}GPIO;
GPIO *s_pGpioRegisters;

typedef struct i2s_inst {
    volatile uint32_t CS_A;        // Control and status
    volatile uint32_t FIFO_A;      // FIFO data
    volatile uint32_t MODE_A;      // Mode control
    volatile uint32_t RXC_A;       // Receive config
    volatile uint32_t TXC_A;       // Transmit config
    volatile uint32_t DREQ_A;      // DMA request level
    volatile uint32_t INTEN_A;     // Interrupt enables
    volatile uint32_t INTSTC_A;    // Interrupt status and clear
    volatile uint32_t GRAY;        // Gray mode control
}I2S_INST;
I2S_INST *i2s;

typedef struct i2s_buffer {
    int32_t *buffer;
    int head;
    int tail;
    int size;
}I2S_BUFF;

static struct i2s_buffer rx_buf;
static struct i2s_buffer tx_buf;
static uint32_t rx_buffer[SAMPLE_BUFF_LEN];
static uint32_t tx_buffer[SAMPLE_BUFF_LEN];

/* Keep track of the number of interrupts where an error occurs */
static int tx_error_count = 0;
static int rx_error_count = 0;



static ssize_t led_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", led_status);
}
static void SetGPIOFunction(int GPIO, int functionCode)
{
    int registerIndex = GPIO / 10;
    int bit = (GPIO%10)*3;
    unsigned oldValue = s_pGpioRegisters->GPFSEL[registerIndex];
    unsigned mask = 0b111 << bit;
    s_pGpioRegisters->GPFSEL[registerIndex] = (oldValue & ~mask) | ((functionCode << bit) & mask);
}
static void SetGPIOOutputValue(int GPIO, bool outputValue)
{
    if(outputValue)
    {
        s_pGpioRegisters->GPSET[GPIO/32] = (1<<(GPIO%32));
    }
    else
    {
        s_pGpioRegisters->GPCLR[GPIO/32] = (1<<(GPIO%32));
    }
}
static ssize_t led_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
    sscanf(buf, "%du", &led_status);
    
    if(led_status)
    {
        SetGPIOOutputValue(LedGpioPin, true); // Turn LED on.
    }
    else
    {
        SetGPIOOutputValue(LedGpioPin, false); // Turn LED off.
    }
    return count;
}
static struct kobj_attribute led_attr = __ATTR(led_pwr, 0660, led_show, led_store);

int DPG_i2s_open(struct inode *inode, struct file *filp)
{
    printk(KERN_INFO "DIAGPIO : DPG_open() is called..\n");
    SetGPIOOutputValue(LedGpioPin,1);
    printk("DPG open success!!!\n");
    return 0;
}

int DPG_i2s_release(struct inode *inode, struct file *flip)
{
    printk("DIAGPIO : DPG_release() is called.. \n");
    SetGPIOOutputValue(LedGpioPin,0);
    printk("DPG release success!!!!\n");
    return 0;
}

struct file_operations DPG_fops = {
    .owner   = THIS_MODULE,
    .open    = DPG_i2s_open,
    .release = DPG_i2s_release,
    //.read    = DPG_read,
    //.write   = DPG_write,
    //.unlocked_ioctl   = DPG_unlocked_ioctl,
};
static void buffer_init(struct i2s_buffer *b, int32_t *data, int size)
{
    b->head = 0;
    b->tail = 0;
    b->size = size;
    b->buffer = data;
}
static int __init DPG_i2s_init(void)
{
    int result;
    printk("DPG init called\n");
    
    int error = 0;
    s_pGpioRegisters = (struct GpioRegisters *)ioremap(GPIO_BASE, sizeof(struct GpioRegisters));         // Map physical address to virtual address space.
    i2s = (volatile struct i2s_inst *) ioremap(I2S_BASE, I2S_SIZE);
    printk("DPG : i2s register on \n");
    
    buffer_init(&rx_buf, rx_buffer, SAMPLE_BUFF_LEN);
    buffer_init(&tx_buf, tx_buffer, SAMPLE_BUFF_LEN);
    
    wmb();
    i2s->CS_A = 0;
    i2s->MODE_A = 0;
    i2s->TXC_A = 0;
    i2s->RXC_A = 0;
    i2s->GRAY = 0;
    
    printk("DPG : i2s register reset\n");
    
    
    printk("Configuring RPI as I2S slave..\n");
    i2s->MODE_A = I2S_MODE_A_CLKM | I2S_MODE_A_FSM;
    
    /* Configure channels and frame width
     * Gives a channel width of 24 bits,
     * First bit of channel 1 is received on the 2nd clock cycle,
     * First bit of channel 2 is received on the 34rd clock cycle */
    printk(KERN_INFO "Setting channel width...");
    i2s->RXC_A = I2S_RXC_A_CH1EN | I2S_RXC_A_CH1POS(1) | I2S_RXC_A_CH1WEX | I2S_RXC_A_CH1WID(0) | I2S_RXC_A_CH2EN | I2S_RXC_A_CH2POS(33) | I2S_RXC_A_CH2WEX | I2S_RXC_A_CH2WID(0);
    i2s->TXC_A = I2S_TXC_A_CH1EN | I2S_TXC_A_CH1POS(1) | I2S_TXC_A_CH1WEX | I2S_TXC_A_CH1WID(0) | I2S_TXC_A_CH2EN | I2S_TXC_A_CH2POS(33) | I2S_TXC_A_CH2WEX | I2S_TXC_A_CH2WID(0);

    // Disable Standby
    printk(KERN_INFO "Disabling standby...");
    i2s->CS_A |= I2S_CS_A_STBY;
    
    // Reset FIFOs
    printk(KERN_INFO "Clearing FIFOs...");
    i2s->CS_A |= I2S_CS_A_TXCLR | I2S_CS_A_RXCLR;
    
    /* Interrupt driven mode */
    /* Interrupt when TX fifo is less than full and RX fifo is full */
    i2s->CS_A |= I2S_CS_A_TXTHR(0x1) | I2S_CS_A_RXTHR(0x3);
    // Enable TXW and RXR interrupts
    i2s->INTEN_A = I2S_INTEN_A_TXW | I2S_INTEN_A_RXR;
    
    // Enable the PCM/I2S module
    printk(KERN_INFO "Enabling I2S...");
    i2s->CS_A |= I2S_CS_A_EN;
    
    printk(KERN_INFO "I2S configuration Complete.");
    printk(KERN_INFO "I2S CS contents %x", i2s->CS_A);
    printk(KERN_INFO "I2S MODE contents %x", i2s->MODE_A);
    printk(KERN_INFO "I2S RXC contents %x", i2s->RXC_A);
    printk(KERN_INFO "I2S TXC contents %x", i2s->TXC_A);
    printk(KERN_INFO "I2S INTEN contents %x", i2s->INTEN_A);
    printk(KERN_INFO "I2S INTSTC contents %x", i2s->INTSTC_A);
    printk(KERN_INFO "I2S_SET_TXON macro value = %x", I2S_SET_TXON);
    printk(KERN_INFO "I2S_SET_RXON macro value = %x", I2S_SET_RXON);
    printk(KERN_INFO "I2S_TX_BUFF_SPACE macro value = %x", I2S_TX_BUFF_SPACE);
    
    /* I2S driver is now configured, but TX and RX will need to be turned on before data is transferred */

    result = register_chrdev(MAJOR,"DPG",&DPG_fops);
    if(result <0 )
    {
        printk("DPG : Can't get major number!\n:");
        return result;
    }
    if(MAJOR == 0) MAJOR = result;
    SetGPIOFunction(LedGpioPin, 0b001);                    // Configure the pin as output.
    led_kobj = kobject_create_and_add("led_ctrl_pwr", kernel_kobj);
    if(!led_kobj) {
        return -ENOMEM;
    }
    error = sysfs_create_file(led_kobj, &led_attr.attr);
    if(error) {
        printk(KERN_INFO "Failed to register sysfs for LED");
    }
    
    return error;
}

static void __exit DPG_i2s_exit(void)
{
    SetGPIOFunction(LedGpioPin, 0);                     // Configure the pin as input.
    iounmap(s_pGpioRegisters);                          // Unmap physical to virtual addresses.
    kobject_put(led_kobj);
    printk("<1>Bye Bye Mr.Blue\n");
    unregister_chrdev(MAJOR,"DPG");
}

module_init(DPG_i2s_init);
module_exit(DPG_i2s_exit);

MODULE_LICENSE("Dual BSD/GPL");
