/*
 * PiFmRds - FM/RDS transmitter for the Raspberry Pi
 * Copyright (C) 2014, 2015 Christophe Jacquet, F8FTK
 * Copyright (C) 2012, 2015 Richard Hirst
 * Copyright (C) 2012 Oliver Mattos and Oskar Weigl
 *
 * See https://github.com/ChristopheJacquet/PiFmRds
 *
 * PI-FM-RDS: RaspberryPi FM transmitter, with RDS.
 *
 * This file contains the VHF FM modulator. All credit goes to the original
 * authors, Oliver Mattos and Oskar Weigl for the original idea, and to
 * Richard Hirst for using the Pi's DMA engine, which reduced CPU usage
 * dramatically.
 *
 * I (Christophe Jacquet) have adapted their idea to transmitting samples
 * at 228 kHz, allowing to build the 57 kHz subcarrier for RDS BPSK data.
 *
 * To make it work on the Raspberry Pi 2, I used a fix by Richard Hirst
 * (again) to request memory using Broadcom's mailbox interface. This fix
 * was published for ServoBlaster here:
 * https://www.raspberrypi.org/forums/viewtopic.php?p=699651#p699651
 *
 * Never use this to transmit VHF-FM data through an antenna, as it is
 * illegal in most countries. This code is for testing purposes only.
 * Always connect a shielded transmission line from the RaspberryPi directly
 * to a radio receiver, so as *not* to emit radio waves.
 *
 * ---------------------------------------------------------------------------
 * These are the comments from Richard Hirst's version:
 *
 * RaspberryPi based FM transmitter.  For the original idea, see:
 *
 * http://www.icrobotics.co.uk/wiki/index.php/Turning_the_Raspberry_Pi_Into_an_FM_Transmitter
 *
 * All credit to Oliver Mattos and Oskar Weigl for creating the original code.
 *
 * I have taken their idea and reworked it to use the Pi DMA engine, so
 * reducing the CPU overhead for playing a .wav file from 100% to about 1.6%.
 *
 * I have implemented this in user space, using an idea I picked up from Joan
 * on the Raspberry Pi forums - credit to Joan for the DMA from user space
 * idea.
 *
 * The idea of feeding the PWM FIFO in order to pace DMA control blocks comes
 * from ServoBlaster, and I take credit for that :-)
 *
 * This code uses DMA channel 0 and the PWM hardware, with no regard for
 * whether something else might be trying to use it at the same time (such as
 * the 3.5mm jack audio driver).
 *
 * I know nothing much about sound, subsampling, or FM broadcasting, so it is
 * quite likely the sound quality produced by this code can be improved by
 * someone who knows what they are doing.  There may be issues realting to
 * caching, as the user space process just writes to its virtual address space,
 * and expects the DMA controller to see the data; it seems to work for me
 * though.
 *
 * NOTE: THIS CODE MAY WELL CRASH YOUR PI, TRASH YOUR FILE SYSTEMS, AND
 * POTENTIALLY EVEN DAMAGE YOUR HARDWARE.  THIS IS BECAUSE IT STARTS UP THE DMA
 * CONTROLLER USING MEMORY OWNED BY A USER PROCESS.  IF THAT USER PROCESS EXITS
 * WITHOUT STOPPING THE DMA CONTROLLER, ALL HELL COULD BREAK LOOSE AS THE
 * MEMORY GETS REALLOCATED TO OTHER PROCESSES WHILE THE DMA CONTROLLER IS STILL
 * USING IT.  I HAVE ATTEMPTED TO MINIMISE ANY RISK BY CATCHING SIGNALS AND
 * RESETTING THE DMA CONTROLLER BEFORE EXITING, BUT YOU HAVE BEEN WARNED.  I
 * ACCEPT NO LIABILITY OR RESPONSIBILITY FOR ANYTHING THAT HAPPENS AS A RESULT
 * OF YOU RUNNING THIS CODE.  IF IT BREAKS, YOU GET TO KEEP ALL THE PIECES.
 *
 * NOTE ALSO:  THIS MAY BE ILLEGAL IN YOUR COUNTRY.  HERE ARE SOME COMMENTS
 * FROM MORE KNOWLEDGEABLE PEOPLE ON THE FORUM:
 *
 * "Just be aware that in some countries FM broadcast and especially long
 * distance FM broadcast could get yourself into trouble with the law, stray FM
 * broadcasts over Airband aviation is also strictly forbidden."
 *
 * "A low pass filter is really really required for this as it has strong
 * harmonics at the 3rd, 5th 7th and 9th which sit in licensed and rather
 * essential bands, ie GSM, HAM, emergency services and others. Polluting these
 * frequencies is immoral and dangerous, whereas "breaking in" on FM bands is
 * just plain illegal."
 *
 * "Don't get caught, this GPIO use has the potential to exceed the legal
 * limits by about 2000% with a proper aerial."
 *
 *
 * As for the original code, this code is released under the GPL.
 *
 * Richard Hirst <richardghirst@gmail.com>  December 2012
 */

#include <locale.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <stdarg.h>
#include <stdint.h>
#include <math.h>
#include <time.h>
#include <limits.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sndfile.h>

#include "rds.h"
#include "fm_mpx.h"
#include "nbfm.h"
#include "control_pipe.h"

#include "mailbox.h"
#define MBFILE            DEVICE_FILE_NAME    /* From mailbox.h */

#if (RASPI)==1
#define PERIPH_VIRT_BASE 0x20000000
#define PERIPH_PHYS_BASE 0x7e000000
#define DRAM_PHYS_BASE 0x40000000
#define MEM_FLAG 0x0c
#define PLLFREQ 500000000.
#elif (RASPI)==2
#define PERIPH_VIRT_BASE 0x3f000000
#define PERIPH_PHYS_BASE 0x7e000000
#define DRAM_PHYS_BASE 0xc0000000
#define MEM_FLAG 0x04
#define PLLFREQ 500000000.
#elif (RASPI)==4
#define PERIPH_VIRT_BASE 0xfe000000
#define PERIPH_PHYS_BASE 0x7e000000
#define DRAM_PHYS_BASE 0xc0000000
#define MEM_FLAG 0x04
#define PLLFREQ 750000000.
#else
#error Unknown Raspberry Pi version (variable RASPI)
#endif

#define NUM_SAMPLES        50000
#define NUM_CBS            (NUM_SAMPLES * 2)

#define BCM2708_DMA_NO_WIDE_BURSTS    (1<<26)
#define BCM2708_DMA_WAIT_RESP        (1<<3)
#define BCM2708_DMA_D_DREQ        (1<<6)
#define BCM2708_DMA_PER_MAP(x)        ((x)<<16)
#define BCM2708_DMA_END            (1<<1)
#define BCM2708_DMA_RESET        (1<<31)
#define BCM2708_DMA_INT            (1<<2)

#define DMA_CS            (0x00/4)
#define DMA_CONBLK_AD        (0x04/4)
#define DMA_DEBUG        (0x20/4)

#define DMA_BASE_OFFSET        0x00007000
#define DMA_LEN            0x24
#define PWM_BASE_OFFSET        0x0020C000
#define PWM_LEN            0x28
#define CLK_BASE_OFFSET            0x00101000
#define CLK_LEN            0xA8
#define GPIO_BASE_OFFSET    0x00200000
#define GPIO_LEN        0x100

#define DMA_VIRT_BASE        (PERIPH_VIRT_BASE + DMA_BASE_OFFSET)
#define PWM_VIRT_BASE        (PERIPH_VIRT_BASE + PWM_BASE_OFFSET)
#define CLK_VIRT_BASE        (PERIPH_VIRT_BASE + CLK_BASE_OFFSET)
#define GPIO_VIRT_BASE        (PERIPH_VIRT_BASE + GPIO_BASE_OFFSET)
#define PCM_VIRT_BASE        (PERIPH_VIRT_BASE + PCM_BASE_OFFSET)

#define PWM_PHYS_BASE        (PERIPH_PHYS_BASE + PWM_BASE_OFFSET)
#define PCM_PHYS_BASE        (PERIPH_PHYS_BASE + PCM_BASE_OFFSET)
#define GPIO_PHYS_BASE        (PERIPH_PHYS_BASE + GPIO_BASE_OFFSET)


#define PWM_CTL            (0x00/4)
#define PWM_DMAC        (0x08/4)
#define PWM_RNG1        (0x10/4)
#define PWM_FIFO        (0x18/4)

#define PWMCLK_CNTL        40
#define PWMCLK_DIV        41

#define CM_GP0DIV (0x7e101074)

#define GPCLK_CNTL        (0x70/4)
#define GPCLK_DIV        (0x74/4)

#define CM_PASSWORD        (0x5A << 24)
#define CM_CTL_MASH_1      (1 << 9)
#define CM_CTL_BUSY        (1 << 7)
#define CM_CTL_ENAB        (1 << 4)
#define CM_SRC_PLLD        6

#define PWMCTL_MODE1        (1<<1)
#define PWMCTL_PWEN1        (1<<0)
#define PWMCTL_CLRF        (1<<6)
#define PWMCTL_USEF1        (1<<5)

#define PWMDMAC_ENAB        (1<<31)
// I think this means it requests as soon as there is one free slot in the FIFO
// which is what we want as burst DMA would mess up our timing.
#define PWMDMAC_THRSHLD        ((15<<8)|(15<<0))

#define GPFSEL0            (0x00/4)

// The deviation specifies how wide the signal is.
#define WBFM_DEVIATION        25.0
#define NBFM_DEVIATION        3.5

#define FM_MIN_FREQ        76000000
#define FM_MAX_FREQ        108000000
#define NOAA_MIN_FREQ      162400000
#define NOAA_MAX_FREQ      162550000


typedef enum {
    MODULATION_MODE_WBFM,
    MODULATION_MODE_NBFM
} modulation_mode_t;


typedef struct {
    uint32_t info, src, dst, length,
         stride, next, pad[2];
} dma_cb_t;

#define BUS_TO_PHYS(x) ((x)&~0xC0000000)


static struct {
    int handle;            /* From mbox_open() */
    unsigned mem_ref;    /* From mem_alloc() */
    unsigned bus_addr;    /* From mem_lock() */
    uint8_t *virt_addr;    /* From mapmem() */
} mbox;



static volatile uint32_t *pwm_reg;
static volatile uint32_t *clk_reg;
static volatile uint32_t *dma_reg;
static volatile uint32_t *gpio_reg;
static int (*baseband_close)(void) = NULL;
static int debug_enabled = 0;

#define DBG(...) do { if(debug_enabled) fprintf(stderr, __VA_ARGS__); } while(0)

struct control_data_s {
    dma_cb_t cb[NUM_CBS];
    uint32_t sample[NUM_SAMPLES];
};

#define PAGE_SIZE    4096
#define PAGE_SHIFT    12
#define NUM_PAGES    ((sizeof(struct control_data_s) + PAGE_SIZE - 1) >> PAGE_SHIFT)

static struct control_data_s *ctl;



static void debug_dump_registers(const char *stage) {
    if(!debug_enabled) return;

    fprintf(stderr,
            "[debug] %s: dma_cs=0x%08x dma_conblk_ad=0x%08x dma_debug=0x%08x "
            "pwm_ctl=0x%08x pwm_dmac=0x%08x "
            "gpclk_cntl=0x%08x gpclk_div=0x%08x "
            "pwmclk_cntl=0x%08x pwmclk_div=0x%08x\n",
            stage,
            dma_reg ? dma_reg[DMA_CS] : 0,
            dma_reg ? dma_reg[DMA_CONBLK_AD] : 0,
            dma_reg ? dma_reg[DMA_DEBUG] : 0,
            pwm_reg ? pwm_reg[PWM_CTL] : 0,
            pwm_reg ? pwm_reg[PWM_DMAC] : 0,
            clk_reg ? clk_reg[GPCLK_CNTL] : 0,
            clk_reg ? clk_reg[GPCLK_DIV] : 0,
            clk_reg ? clk_reg[PWMCLK_CNTL] : 0,
            clk_reg ? clk_reg[PWMCLK_DIV] : 0);
}



static void debug_startup_probe(void) {
    if(!debug_enabled) return;

    for(int i = 0; i < 15; i++) {
        usleep(10000);
        debug_dump_registers("startup probe");
    }
}



static void
udelay(int us)
{
    struct timespec ts = { 0, us * 1000 };

    nanosleep(&ts, NULL);
}



static int wait_clock_busy(uint32_t clock_ctl_index, int busy) {
    for(int i = 0; i < 10000; i++) {
        int is_busy = (clk_reg[clock_ctl_index] & CM_CTL_BUSY) != 0;

        if(is_busy == busy) return 0;

        udelay(1);
    }

    return -1;
}



static void stop_clock(uint32_t clock_ctl_index) {
    clk_reg[clock_ctl_index] = CM_PASSWORD | CM_SRC_PLLD;
    if(wait_clock_busy(clock_ctl_index, 0) < 0) {
        printf("Warning: clock %u did not stop cleanly.\n", clock_ctl_index);
    }
}



static void start_clock(uint32_t clock_ctl_index) {
    clk_reg[clock_ctl_index] = CM_PASSWORD | CM_CTL_MASH_1 | CM_CTL_ENAB | CM_SRC_PLLD;
    if(wait_clock_busy(clock_ctl_index, 1) < 0) {
        printf("Warning: clock %u did not start cleanly.\n", clock_ctl_index);
    }
}



static void
terminate(int num)
{
    DBG("[debug] terminate entry: num=%d\n", num);
    debug_dump_registers("terminate entry");

    // Stop outputting and generating the clock.
    if (clk_reg && gpio_reg) {
        // Set GPIO4 to be an output (instead of ALT FUNC 0, which is the clock).
        gpio_reg[GPFSEL0] = (gpio_reg[GPFSEL0] & ~(7 << 12)) | (1 << 12);

        // Disable the clock generator.
        stop_clock(GPCLK_CNTL);
        debug_dump_registers("after GPCLK stop");
    }

    if (dma_reg) {
        dma_reg[DMA_CS] = BCM2708_DMA_RESET;
        udelay(10);
        debug_dump_registers("after DMA reset");
    }

    if (pwm_reg && clk_reg) {
        pwm_reg[PWM_CTL] = 0;
        udelay(10);
        pwm_reg[PWM_DMAC] = 0;
        udelay(10);
        pwm_reg[PWM_CTL] = PWMCTL_CLRF;
        udelay(10);
        stop_clock(PWMCLK_CNTL);
        udelay(10);
        debug_dump_registers("after PWM/PWMCLK reset");
    }

    // TODO: A later startup experiment may explicitly stop PWM/PWMCLK here.
    debug_dump_registers("before baseband/control cleanup");

    if(baseband_close != NULL) {
        baseband_close();
        baseband_close = NULL;
    }

    close_control_pipe();

    if (mbox.virt_addr != NULL) {
        unmapmem(mbox.virt_addr, NUM_PAGES * 4096);
        mem_unlock(mbox.handle, mbox.mem_ref);
        mem_free(mbox.handle, mbox.mem_ref);
    }

    printf("Terminating: cleanly deactivated the DMA engine and killed the carrier.\n");

    exit(num);
}




static int is_fm_freq(uint32_t carrier_freq) {
    return carrier_freq >= FM_MIN_FREQ && carrier_freq <= FM_MAX_FREQ;
}




static int is_noaa_freq(uint32_t carrier_freq) {
    return carrier_freq >= NOAA_MIN_FREQ && carrier_freq <= NOAA_MAX_FREQ;
}

static void
fatal(char *fmt, ...)
{
    va_list ap;

    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    terminate(0);
}

static size_t
mem_virt_to_phys(void *virt)
{
    size_t offset = (size_t)virt - (size_t)mbox.virt_addr;

    return mbox.bus_addr + offset;
}

static size_t
mem_phys_to_virt(size_t phys)
{
    return (size_t) (phys - mbox.bus_addr + mbox.virt_addr);
}

static void *
map_peripheral(uint32_t base, uint32_t len)
{
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    void * vaddr;

    if (fd < 0)
        fatal("Failed to open /dev/mem: %m.\n");
    vaddr = mmap(NULL, len, PROT_READ|PROT_WRITE, MAP_SHARED, fd, base);
    if (vaddr == MAP_FAILED)
        fatal("Failed to map peripheral at 0x%08x: %m.\n", base);
    close(fd);

    return vaddr;
}



#define SUBSIZE 1
#define DATA_SIZE 5000


int tx(uint32_t carrier_freq, char *audio_file, uint16_t pi, char *ps, char *rt, float ppm, char *control_pipe, modulation_mode_t modulation_mode) {
    int (*baseband_open)(char *, size_t) = fm_mpx_open;
    int (*baseband_get_samples)(float *) = fm_mpx_get_samples;
    float deviation = WBFM_DEVIATION;

    if(modulation_mode == MODULATION_MODE_NBFM) {
        baseband_open = nbfm_open;
        baseband_get_samples = nbfm_get_samples;
        baseband_close = nbfm_close;
        deviation = NBFM_DEVIATION;
        control_pipe = NULL;
    } else {
        baseband_close = fm_mpx_close;
    }

    // Catch all signals possible - it is vital we kill the DMA engine
    // on process exit!
    for (int i = 0; i < 64; i++) {
        struct sigaction sa;

        memset(&sa, 0, sizeof(sa));
        sa.sa_handler = terminate;
        sigaction(i, &sa, NULL);
    }

    dma_reg = map_peripheral(DMA_VIRT_BASE, DMA_LEN);
    pwm_reg = map_peripheral(PWM_VIRT_BASE, PWM_LEN);
    clk_reg = map_peripheral(CLK_VIRT_BASE, CLK_LEN);
    gpio_reg = map_peripheral(GPIO_VIRT_BASE, GPIO_LEN);
    debug_dump_registers("after peripheral mmap/register setup");

    // Force clean peripheral state before configuring RF output.
    debug_dump_registers("Force cleaning peripheral state before configuring RF output...");
    dma_reg[DMA_CS] = BCM2708_DMA_RESET;
    udelay(100);
    pwm_reg[PWM_CTL] = 0;
    udelay(100);
    pwm_reg[PWM_DMAC] = 0;
    udelay(100);
    pwm_reg[PWM_CTL] = PWMCTL_CLRF;
    udelay(100);
    stop_clock(PWMCLK_CNTL);
    udelay(100);
    stop_clock(GPCLK_CNTL);
    udelay(100);
    debug_dump_registers("after forced peripheral cleanup"); 

    // Calculate the frequency control word.
    // The fractional part is stored in the lower 12 bits.
    uint32_t freq_ctl = ((float)(PLLFREQ / carrier_freq)) * (1 << 12);

    // Use the mailbox interface to the VC to ask for physical memory.
    mbox.handle = mbox_open();
    if (mbox.handle < 0)
        fatal("Failed to open mailbox. Check kernel support for vcio / BCM2708 mailbox.\n");
    printf("Allocating physical memory: size = %zu     ", NUM_PAGES * 4096);
    if(! (mbox.mem_ref = mem_alloc(mbox.handle, NUM_PAGES * 4096, 4096, MEM_FLAG))) {
        fatal("Could not allocate memory.\n");
    }
    // TODO: How do we know that succeeded?
    printf("mem_ref = %u     ", mbox.mem_ref);
    if(! (mbox.bus_addr = mem_lock(mbox.handle, mbox.mem_ref))) {
        fatal("Could not lock memory.\n");
    }
    printf("bus_addr = %x     ", mbox.bus_addr);
    if(! (mbox.virt_addr = mapmem(BUS_TO_PHYS(mbox.bus_addr), NUM_PAGES * 4096))) {
        fatal("Could not map memory.\n");
    }
    printf("virt_addr = %p\n", mbox.virt_addr);
    debug_dump_registers("after mailbox memory setup");


    // GPIO4 needs to be ALT FUNC 0 to output the clock
    gpio_reg[GPFSEL0] = (gpio_reg[GPFSEL0] & ~(7 << 12)) | (4 << 12);
    debug_dump_registers("after GPIO4 ALT0 setup");

    // Program GPCLK to use MASH setting 1, so fractional dividers work
    debug_dump_registers("before GPCLK stop/start");
    stop_clock(GPCLK_CNTL);
    clk_reg[GPCLK_DIV] = CM_PASSWORD | freq_ctl;
    udelay(100);
    start_clock(GPCLK_CNTL);
    debug_dump_registers("after GPCLK carrier start");

    ctl = (struct control_data_s *) mbox.virt_addr;
    dma_cb_t *cbp = ctl->cb;
    uint32_t phys_sample_dst = CM_GP0DIV;
    uint32_t phys_pwm_fifo_addr = PWM_PHYS_BASE + 0x18;

    for (int i = 0; i < NUM_SAMPLES; i++) {
        ctl->sample[i] = 0x5a << 24 | freq_ctl;    // Silence
        // Write a frequency sample
        cbp->info = BCM2708_DMA_NO_WIDE_BURSTS | BCM2708_DMA_WAIT_RESP;
        cbp->src = mem_virt_to_phys(ctl->sample + i);
        cbp->dst = phys_sample_dst;
        cbp->length = 4;
        cbp->stride = 0;
        cbp->next = mem_virt_to_phys(cbp + 1);
        cbp++;
        // Delay
        cbp->info = BCM2708_DMA_NO_WIDE_BURSTS | BCM2708_DMA_WAIT_RESP | BCM2708_DMA_D_DREQ | BCM2708_DMA_PER_MAP(5);
        cbp->src = mem_virt_to_phys(mbox.virt_addr);
        cbp->dst = phys_pwm_fifo_addr;
        cbp->length = 4;
        cbp->stride = 0;
        cbp->next = mem_virt_to_phys(cbp + 1);
        cbp++;
    }
    cbp--;
    cbp->next = mem_virt_to_phys(mbox.virt_addr);
    debug_dump_registers("after DMA control block build");

    // Here we define the rate at which we want to update the GPCLK control
    // register.
    //
    // Set the range to 2 bits. PLLD is at 500 MHz, therefore to get 228 kHz
    // we need a divisor of 500000000 / 2000 / 228 = 1096.491228
    //
    // This is 1096 + 2012*2^-12 theoretically
    //
    // However the fractional part may have to be adjusted to take the actual
    // frequency of your Pi's oscillator into account. For example on my Pi,
    // the fractional part should be 1916 instead of 2012 to get exactly
    // 228 kHz. However RDS decoding is still okay even at 2012.
    //
    // So we use the 'ppm' parameter to compensate for the oscillator error

    float divider = (PLLFREQ/(2000*228*(1.+ppm/1.e6)));
    uint32_t idivider = (uint32_t) divider;
    uint32_t fdivider = (uint32_t) ((divider - idivider)*pow(2, 12));

    printf("ppm corr is %.4f, divider is %.4f (%d + %d*2^-12) [nominal 1096.4912].\n",
                ppm, divider, idivider, fdivider);

    pwm_reg[PWM_CTL] = 0;
    udelay(10);
    stop_clock(PWMCLK_CNTL);                        // Source=PLLD and disable
    // theorically : 1096 + 2012*2^-12
    clk_reg[PWMCLK_DIV] = CM_PASSWORD | (idivider<<12) | fdivider;
    udelay(100);
    start_clock(PWMCLK_CNTL);                       // Source=PLLD and enable + MASH filter 1
    pwm_reg[PWM_RNG1] = 2;
    udelay(10);
    pwm_reg[PWM_DMAC] = PWMDMAC_ENAB | PWMDMAC_THRSHLD;
    udelay(10);
    pwm_reg[PWM_CTL] = PWMCTL_CLRF;
    udelay(10);
    pwm_reg[PWM_CTL] = PWMCTL_USEF1 | PWMCTL_PWEN1;
    udelay(10);
    debug_dump_registers("after PWM clock/FIFO/DREQ setup");


    // Initialise the DMA
    dma_reg[DMA_CS] = BCM2708_DMA_RESET;
    udelay(10);
    dma_reg[DMA_CS] = BCM2708_DMA_INT | BCM2708_DMA_END;
    dma_reg[DMA_CONBLK_AD] = mem_virt_to_phys(ctl->cb);
    dma_reg[DMA_DEBUG] = 7; // clear debug error flags
    debug_dump_registers("before DMA start");
    dma_reg[DMA_CS] = 0x10880001;    // go, mid priority, wait for outstanding writes
    debug_dump_registers("after DMA start");
    debug_startup_probe();


    size_t last_cb = (size_t)ctl->cb;

    // Data structures for baseband data
    float data[DATA_SIZE];
    int data_len = 0;
    int data_index = 0;

    // Initialize the baseband generator
    debug_dump_registers("before baseband_open");
    if(baseband_open(audio_file, DATA_SIZE) < 0) return 1;
    debug_dump_registers("after baseband_open");

    char myps[9] = {0};
    uint16_t count = 0;
    uint16_t count2 = 0;
    int varying_ps = 0;

    if(modulation_mode == MODULATION_MODE_WBFM) {
        // Initialize the RDS modulator
        set_rds_pi(pi);
        set_rds_rt(rt);

        if(ps) {
            set_rds_ps(ps);
            printf("PI: %04X, PS: \"%s\".\n", pi, ps);
        } else {
            printf("PI: %04X, PS: <Varying>.\n", pi);
            varying_ps = 1;
        }
        printf("RT: \"%s\"\n", rt);
    } else {
        printf("NBFM mode: RDS, stereo pilot and control pipe are disabled.\n");
    }

    // Initialize the control pipe reader
    if(control_pipe) {
        printf("Waiting for control pipe `%s` to be opened by the writer, e.g. "
               "by running `cat >%s`.\n", control_pipe, control_pipe);
        if(open_control_pipe(control_pipe) == 0) {
            printf("Reading control commands on %s.\n", control_pipe);
        } else {
            printf("Failed to open control pipe: %s.\n", control_pipe);
            control_pipe = NULL;
        }
    }


    printf("Starting to transmit on %3.3f MHz in %s mode, deviation %.1f kHz.\n",
           carrier_freq/1e6,
           modulation_mode == MODULATION_MODE_NBFM ? "NBFM" : "WBFM/RDS",
           deviation);

    unsigned long long loop_count = 0;
    unsigned long long written_samples = 0;
    unsigned long long dma_conblk_stalled_count = 0;
    uint32_t previous_dma_conblk_ad = 0;
    int intval_min = INT_MAX;
    int intval_max = INT_MIN;
    unsigned long long intval_nonzero_count = 0;
    time_t last_debug_dump_time = time(NULL);

    for (;;) {
        if(debug_enabled) loop_count++;

        // Default (varying) PS
        if(varying_ps) {
            if(count == 512) {
                snprintf(myps, 9, "%08d", count2);
                set_rds_ps(myps);
                count2++;
            }
            if(count == 1024) {
                set_rds_ps("RPi-Live");
                count = 0;
            }
            count++;
        }

        if(control_pipe && poll_control_pipe() == CONTROL_PIPE_PS_SET) {
            varying_ps = 0;
        }

        usleep(5000);

        uint32_t current_dma_conblk_ad = dma_reg[DMA_CONBLK_AD];
        if(debug_enabled && previous_dma_conblk_ad != 0 && current_dma_conblk_ad == previous_dma_conblk_ad) {
            dma_conblk_stalled_count++;
        }
        if(debug_enabled) previous_dma_conblk_ad = current_dma_conblk_ad;

        size_t cur_cb = mem_phys_to_virt(current_dma_conblk_ad);
        int last_sample = (last_cb - (size_t)mbox.virt_addr) / (sizeof(dma_cb_t) * 2);
        int this_sample = (cur_cb - (size_t)mbox.virt_addr) / (sizeof(dma_cb_t) * 2);
        int free_slots = this_sample - last_sample;

        if (free_slots < 0)
            free_slots += NUM_SAMPLES;

        int free_slots_for_debug = free_slots;

        while (free_slots >= SUBSIZE) {
            // get more baseband samples if necessary
            if(data_len == 0) {
                if(baseband_get_samples(data) < 0) {
                    terminate(0);
                }
                data_len = DATA_SIZE;
                data_index = 0;
            }

            float dval = data[data_index] * (deviation / 10.);
            data_index++;
            data_len--;

            int intval = (int)((floor)(dval));
            //int frac = (int)((dval - (float)intval) * SUBSIZE);

            if(debug_enabled) {
                if(intval < intval_min) intval_min = intval;
                if(intval > intval_max) intval_max = intval;
                if(intval != 0) intval_nonzero_count++;
            }

            ctl->sample[last_sample++] = (0x5A << 24 | freq_ctl) + intval; //(frac > j ? intval + 1 : intval);
            if (last_sample == NUM_SAMPLES)
                last_sample = 0;

            free_slots -= SUBSIZE;
            if(debug_enabled) written_samples++;
        }
        last_cb = (size_t)(mbox.virt_addr + last_sample * sizeof(dma_cb_t) * 2);

        if(debug_enabled) {
            time_t now = time(NULL);

            if(now != last_debug_dump_time) {
                int dump_intval_min = intval_min == INT_MAX ? 0 : intval_min;
                int dump_intval_max = intval_max == INT_MIN ? 0 : intval_max;

                uint32_t gpclk_div_min = 0xffffffff;
                uint32_t gpclk_div_max = 0;
                uint32_t gpclk_div_last = 0;

                for (int k = 0; k < 128; k++) {
                    uint32_t div = ((volatile uint32_t *)clk_reg)[GPCLK_DIV];

                    if (div < gpclk_div_min)
                        gpclk_div_min = div;

                    if (div > gpclk_div_max)
                        gpclk_div_max = div;

                    gpclk_div_last = div;
                }

                uint32_t gpfsel0 = ((volatile uint32_t *)gpio_reg)[GPFSEL0];
                int gpio4_func = (gpfsel0 >> 12) & 7;

                fprintf(stderr,
                        "[debug] loop: loop_count=%llu dma_cs=0x%08x "
                        "dma_conblk_ad=0x%08x dma_debug=0x%08x "
                        "cur_cb=0x%zx this_sample=%d last_sample=%d "
                        "free_slots=%d written_samples=%llu "
                        "dma_conblk_stalled_count=%llu "
                        "intval_min=%d intval_max=%d intval_nonzero_count=%llu "
                        "gpclk_div_min=0x%08x gpclk_div_max=0x%08x gpclk_div_last=0x%08x "
                        "gpfsel0=0x%08x gpio4_func=%d "
                        "pwm_ctl=0x%08x pwm_dmac=0x%08x "
                        "gpclk_cntl=0x%08x pwmclk_cntl=0x%08x\n",
                        loop_count,
                        dma_reg[DMA_CS],
                        current_dma_conblk_ad,
                        dma_reg[DMA_DEBUG],
                        cur_cb,
                        this_sample,
                        last_sample,
                        free_slots_for_debug,
                        written_samples,
                        dma_conblk_stalled_count,
                        dump_intval_min,
                        dump_intval_max,
                        intval_nonzero_count,
                        gpclk_div_min,
                        gpclk_div_max,
                        gpclk_div_last,
                        gpfsel0,
                        gpio4_func,
                        pwm_reg[PWM_CTL],
                        pwm_reg[PWM_DMAC],
                        clk_reg[GPCLK_CNTL],
                        clk_reg[PWMCLK_CNTL]);

                intval_min = INT_MAX;
                intval_max = INT_MIN;
                intval_nonzero_count = 0;
                last_debug_dump_time = now;
            }
        }
    }

    return 0;
}


int main(int argc, char **argv) {
    char *audio_file = NULL;
    char *control_pipe = NULL;
    uint32_t carrier_freq = 107900000;
    modulation_mode_t modulation_mode = MODULATION_MODE_WBFM;
    char *ps = NULL;
    char *rt = "PiFmRds: live FM-RDS transmission from the RaspberryPi";
    uint16_t pi = 0x1234;
    float ppm = 0;


    // Parse command-line arguments
    for(int i=1; i<argc; i++) {
        char *arg = argv[i];
        char *param = NULL;

        if(arg[0] == '-' && i+1 < argc) param = argv[i+1];

        if(strcmp("-debug", arg)==0) {
            debug_enabled = 1;
        } else if((strcmp("-wav", arg)==0 || strcmp("-audio", arg)==0) && param != NULL) {
            i++;
            audio_file = param;
        } else if(strcmp("-freq", arg)==0 && param != NULL) {
            i++;
            carrier_freq = (uint32_t)(1e6 * atof(param) + 0.5);
            if(is_noaa_freq(carrier_freq)) {
                modulation_mode = MODULATION_MODE_NBFM;
            } else if(is_fm_freq(carrier_freq)) {
                modulation_mode = MODULATION_MODE_WBFM;
            } else {
                fatal("Incorrect frequency specification. Must be in megahertz, of the form 107.9 between 76 and 108, or 162.4 between 162.40 and 162.55.\n");
            }
        } else if(strcmp("-pi", arg)==0 && param != NULL) {
            i++;
            pi = (uint16_t) strtol(param, NULL, 16);
        } else if(strcmp("-ps", arg)==0 && param != NULL) {
            i++;
            ps = param;
        } else if(strcmp("-rt", arg)==0 && param != NULL) {
            i++;
            rt = param;
        } else if(strcmp("-ppm", arg)==0 && param != NULL) {
            i++;
            ppm = atof(param);
        } else if(strcmp("-ctl", arg)==0 && param != NULL) {
            i++;
            control_pipe = param;
        } else {
            fatal("Unrecognised argument: %s.\n"
            "Syntax: pi_fm_rds [-freq freq] [-audio file] [-ppm ppm_error] [-pi pi_code]\n"
            "                  [-ps ps_text] [-rt rt_text] [-ctl control_pipe] [-debug]\n", arg);
        }
    }

    // Set locale based on the environment variables. This is necessary to decode
    // non-ASCII characters using mbtowc() in rds_strings.c.
    char* locale = setlocale(LC_ALL, "");
    printf("Locale set to %s.\n", locale);

    int errcode = tx(carrier_freq, audio_file, pi, ps, rt, ppm, control_pipe, modulation_mode);

    terminate(errcode);
}
