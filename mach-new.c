/* -*- C -*- ****************************************************************
 *
 ****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <arpa/inet.h>
#include "fame.h"

#include "timer_1ms.h"

#define ANSI_ERROR_OUTPUT

#ifdef ANSI_ERROR_OUTPUT
#define NORMAL "\e[0m"
#define OK_BOLD "\e[32;40;1m"
#define ERR_BOLD "\e[31;40;1m"
#else
#define NORMAL ""
#define OK_BOLD ""
#define ERR_BOLD "** ERROR **"
#endif

#define _OK printf(OK_BOLD);
#define _ERR printf(ERR_BOLD);
#define N_ printf(NORMAL);

#undef TRACE
#define TRACE_ON_EXCP 1
#define SHOW_IRQ_ERRORS

#define EMU_TIMESLICE 4

typedef struct M68K_PROGRAM m68k_program;
typedef struct M68K_DATA m68k_data;
typedef struct M68K_CONTEXT m68k_context;

/** Memory map for the sys-1
 * Access Type      Range             Purpose
 * -s ??  ROM[bw]   0x0-0x7           (Reset SSP/PC)
 * us     RAM[bw]   0x000000-0x3fffff (4MiB)
 * 
 * --     Cons[bw]  0x800000-0x803fff (16KiB, text-mode console)
 * us     -CoMem    0x..0000-0x..3fff  -(16368B: 2byte attr+char, max 132*62)
 * 
 * --     Vid[bw]   0x900000-0x9fffff (900KiB, 8-bit max. 800x600x8 2 pages)
 * us     -Videm    0x.00000-0x.ea5ff  -(pixel memory)
 *                  0x.ea600-0x.fffff  -RESVD (mirrored)
 * 
 * --     GIO[bw]   0xa00000-0xa00fff
 * -s     -Tim0     0x...000-0x...00f (System Timer0)
 * -s     -CoCtl    0x...010-0x...01f (Console regs)
 * -s     -Term0    0x...020-0x...02f (Term0)
 * -s     -Term1    0x...030-0x...03f (Term1)
 *                                ...
 * -s     -VidCtl   0x...400-0x...4ff (256B, ctrl regs)
 *                                ...
 * -s     -Rsv      0x...800-0x...fff (2KiB: future, resvd)
 * 
 * us     ROM[bw]   0xf00000-0xf7ffff (512KiB: system ROM)
 */

/* Macros to access target system memory. Does NOT use target's emulated
 * bus cycle, so does not trip any register logic of the emulated system.
 */
#define WORD_AT(s,offs) ((uint16_t *)(s))[(offs)>>1]
#define BYTE_AT(s,offs) (s)[((offs)^1)]

#define RAM_BASE  0x0000000
#define RAM_SIZE  0x0400000
#define CON_BASE  0x0800000
#define CON_SIZE  0x0004000
#define VID_BASE  0x0900000
#define VID_AREA  0x0100000
#define VID_SIZE  0x0100000
#define GIO_BASE   0x0a00000
#define GIO_AREA   0x0001000
#define GIO_SIZE   0x0001000
#define ROM_BASE  0x0f00000
#define ROM_AREA  0x0080000
#define ROM_SIZE  0x0080000

/* Unless otherwise noted all ctl regs are read/write positive logic
   Write PEND bits to ACK corresponding IRQ and clear PEND
   Byte regs MUST be written as bytes, Word regs as word
 */

/* TIM0 -Timer0. A simple CPUCLK/DIV timer.
 * CTLb[COUNTEN]=1 -> enable counter, =0 -> stop counter
 * CTLb[PEND]=1 -> IRQ pending, =0; any write to CTLb clears PEND
 * CTLb[IRQEN]=1 -> IRQ enabled
 * 
 * Up-counting, IRQ on TIM0_DATw[n] < TIM0_DATw[n-1] -
 *   that is, uint16_t overflow or CPU writing TIM0_DATw < TIM0_DATw[n-1].
 * CTLb[PEND]=1 on above conditions
 * 
 * DIVb = TIM0 divider (writing a 0 yields DIVb of 1)
 */
#define GIO_BLK0_MASK   0x3F     /* mask to aid in narrowing decode logic */
/* GIO_BLK0: core timers, serial hardware, control regs for console, etc. */
#define GIO_VIDCTL_MASK 0x4FF

#define TIM0            0x00
#define TIM0_IRQLVL     5
#define TIM0_DATw       0x00     /* upctr: IRQ on overflow; write=clear */
        /* ANY write to TIM0_CTLb clears PEND, ACKs IRQ */
#define TIM0_CTLb       0x03     /* TIM0_CTL[2:0] COUNTEN,PEND,IRQEN */
#define   TIM0_CTLb_COUNTEN    0x04
#define   TIM0_CTLb_PEND       0x02
#define   TIM0_CTLb_IRQEN      0x01

#define TIM0_DIVb       0x05     /* TIM0_DIV[7:0] 8MHz/(1/2/4/.../128) */

#define TIMT            0x0c
#define TIMT_IRQLVL     6
#define TIMT_DAT        0x01     /* ms timer, data - IRQ at 0 - READ ONLY */
#define TIMT_LOAD       0x03     /* ms timer, reload value - 0 to disable */
                                 /* read TIMT_LOAD to ACK IRQ */

#define CON0            0x10
#define CON0_IRQLVL     2
#define CON0_CTLb       0x01 /* CON0_CTL[7:0] MODE1,MODE0,x,x,x,x,PEND,IRQEN */

#define T0           0x20
#define T1           0x30
#define TERMCHAN(a)     (((a) - T0) >> 4)
#define TERMn_IRQLVL    3
#define Tn_TXb       0x01
#define Tn_RXb       0x03    /* Reading clears CTLb[PEND,RXF] */
#define Tn_CTLb      0x05    /* Tn_CTL[7:0] BR1,BR0,TXE,RXF,RXV,TEI,RFI,IRQEN */
#define   Tn_CTLb_BR1    0x80
#define   Tn_CTLb_BR0    0x40
#define   Tn_CTLb_TXE    0x20
#define   Tn_CTLb_RXF    0x10
#define   Tn_CTLb_RXV    0x08
#define   Tn_CTLb_TEI    0x04  /* TXE IRQ PEND */
#define   Tn_CTLb_RFI    0x02  /* RXF IRQ PEND */
#define   Tn_CTLb_IRQEN  0x01

#define TRC            0x40
#define   TRC_DATw       0x00

/* TODO: VID */

uint8_t RAM[RAM_SIZE] = {0};
uint8_t Cons[CON_SIZE];
uint8_t Vid[VID_SIZE];
volatile uint8_t GIO[GIO_SIZE] = {0};
uint8_t ROM[ROM_SIZE];

void GIO_init(void) {
    BYTE_AT(GIO,TIM0_DIVb) = 0x01u;
}

typedef struct _cpuclktimer {
    uint16_t tim0dat;
    uint16_t tim0dat_last;
    uint16_t tim0offs;
} cpuclktimer;

static cpuclktimer Tim0 = {0, 0, 0};

/* TODO: Ensure all peripherals set PEND even if IRQEN == 0 */

void TIM0_count(void) {
    int irq_status;
    
    Tim0.tim0dat = (uint16_t)m68k_get_cycles_counter() / BYTE_AT(GIO,TIM0_DIVb);
    /* High-resolution TIM0 - running at CPUCLK, IRQ on overflow */
    if( BYTE_AT(GIO,TIM0_CTLb) & TIM0_CTLb_COUNTEN ) {
        if( (WORD_AT(GIO,TIM0_DATw) = Tim0.tim0dat-Tim0.tim0offs) < Tim0.tim0dat_last ) {
            /* Overflow */
            /* Raise IRQ if enabled */
            if( BYTE_AT(GIO,TIM0_CTLb) & 0x01 ) {
                if( (irq_status = m68k_raise_irq(TIM0_IRQLVL, M68K_AUTOVECTORED_IRQ)) != M68K_OK ) {
#ifdef SHOW_IRQ_ERRORS
                    _ERR printf("TIM0: Error raising IRQ! (irq_status=%d)\n", irq_status); N_;
#endif
                }
                /* IRQ pending */
                BYTE_AT(GIO,TIM0_CTLb) |= TIM0_CTLb_PEND;
            }
        }
    }
    Tim0.tim0dat_last = Tim0.tim0dat - Tim0.tim0offs;
}

uint32_t TimTick_thandle;

/** TimTick callback
 * 
 * @param[in] unused - callback arg, currently unused
 * 
 * Operation:
 * IF TIMT_LOAD != 0:
 *   TIMT_DAT <- TIMT_DAT-1
 *   IF TIMT_DAT == 0:
 *     ASSERT_IRQ
 *     TIMT_DAT <- TIMT_LOAD
 */
#ifdef __CYGWIN__
void __stdcall gio_TimTick(uint32_t timerID, uint32_t msg,
                           uint32_t unused, uint32_t resvd1,
                           uint32_t resvd2)
#else
void gio_TimTick(uint32_t unused)
#endif
{
    int irq_status;
    
    if( BYTE_AT(GIO,TIMT+TIMT_LOAD) != 0 ) {
#if 0
        _OK; printf("[tick]\n"); N_;
#endif
        BYTE_AT(GIO,TIMT+TIMT_DAT)--;
        if( BYTE_AT(GIO,TIMT+TIMT_DAT) == 0 ) {
            BYTE_AT(GIO,TIMT+TIMT_DAT) = BYTE_AT(GIO,TIMT+TIMT_LOAD);
            irq_status = m68k_raise_irq(TIMT_IRQLVL, M68K_AUTOVECTORED_IRQ);
#ifdef SHOW_IRQ_ERRORS
            if( irq_status != M68K_OK ) {
                _ERR; printf("TIMT: Error raising IRQ! (irq_status=%d)\n", irq_status); N_;
            }
            else {
//                _OK; printf("TIMT: raised IRQ.\n"); N_;
//                show_cpu_state();
            }
#endif
        }
    }
}


#define TERM_UDP  /* Use UDP sockets for TERM ports (t0,t1) */

#define TERMINIT 0
#define TERMDEINIT 1
#define TERMREAD 2
#define TERMWRITE 3
#define TERMSVC 4

#ifdef TERM_UDP
int32_t term(int op, int port, int data) {
    static int tN_fdesc[2];
    static struct sockaddr_in srvsock;
    static struct sockaddr_in clisock;
    static int clisize = sizeof(clisock);
    
    static uint32_t Tn_txTimer[2] = {0u,0u};
    
    int ret = 0;
    int status;
    
    switch(op) {
    case TERMINIT:
        {
            
            srvsock.sin_family = AF_INET;
            srvsock.sin_port = htons(8800+port);
            srvsock.sin_addr.s_addr = htonl(INADDR_ANY);
            memset(&clisock, 0, sizeof(struct sockaddr_in));
            clisock.sin_family = AF_INET;
            clisock.sin_port = htons(8800+port);
            clisock.sin_addr.s_addr = htonl(INADDR_ANY);
            
            tN_fdesc[port] = socket(PF_INET, SOCK_DGRAM, 0);
            if( tN_fdesc[port] < 0 ) {
                _ERR; printf("TERM %d init:", port); perror("socket()"); N_;
                ret = -1;
            }
            else {
                fcntl(tN_fdesc[port], F_SETFL, O_NONBLOCK);
                printf("binding port %d to port %d\n", port,
                       ntohs(srvsock.sin_port));
                if( bind(tN_fdesc[port], (const struct sockaddr *)(&srvsock), sizeof(srvsock)) != 0 ) {
                    _ERR printf("TERM %d error binding socket (%d)\n", port, errno); perror("bind()"); N_;
                    ret = -1;
                }
                else {
                    _OK printf("TERM %d init\n", port); N_;
                }
            }
        }
        break;
    case TERMDEINIT:
        close(tN_fdesc[port]);
        _OK printf("TERM %d deinit\n", port); N_;
        break;
    case TERMREAD:
        {
            uint32_t chan;
            uint8_t inData;
            int read_status;
            
            switch(port) {
            case 0:
                chan = T0;
                break;
            case 1:
                chan = T1;
                break;
            }
#ifdef TRACE
            printf("calling recvfrom()\n");
#endif
            if( (read_status = recvfrom(tN_fdesc[port], &inData, 1, 0,
                                        (struct sockaddr *)(&clisock),
                                        &clisize)) == 1 )
            {
                BYTE_AT(GIO,chan+Tn_RXb) = inData;
#if 1 //def TRACE
                _OK; printf("TERM %d received: %02x '%c'\n",
                            port, inData, (inData < ' ' ? '*' : inData)); N_;
#endif
                if( BYTE_AT(GIO,chan+Tn_CTLb) & Tn_CTLb_RXF ) {
                    /* RX already full, overflow error! */
                    _ERR printf("TERM %d RX ovf\n", port); N_;
                    BYTE_AT(GIO,chan+Tn_CTLb) |= Tn_CTLb_RXV; /* set RXV */
                }
                BYTE_AT(GIO,chan+Tn_CTLb) |= Tn_CTLb_RXF;
                if( BYTE_AT(GIO,chan+Tn_CTLb) & Tn_CTLb_IRQEN ) {
                    /* Set RX IRQ pend bit */
                    BYTE_AT(GIO,chan+Tn_CTLb) |= (Tn_CTLb_RFI);
                    m68k_raise_irq(TERMn_IRQLVL, M68K_AUTOVECTORED_IRQ);
                }
            }
            else {
#ifdef TRACE
                printf("[no data for TERM %d - read_status = %d]\n",
                       port, read_status);
#endif
            }
        }
        break;
    case TERMWRITE:
        {
            uint32_t chan;
            uint8_t outData;
            
            switch(port) {
            case 0:
                chan = T0;
                break;
            case 1:
                chan = T1;
                break;
            }
            /* If TXE is clear there should be a new byte to transmit. */
            if( (BYTE_AT(GIO,chan+Tn_CTLb) & Tn_CTLb_TXE) == 0 ) {
                outData = BYTE_AT(GIO,chan+Tn_TXb);
#ifdef TRACE
                printf("calling sendto(), %d '%c'\n",
                       outData, (outData < ' ' ? '*' : outData));
                getchar();
#endif
                sendto(tN_fdesc[port], &outData, 1, 0,
                       (const struct sockaddr *)&clisock, sizeof(clisock));
                Tn_txTimer[port] = 4u; /* simulate actual UART xmit time */
                /* TERMSVC will later set TXE */
            }
        }
        break;
    case TERMSVC:
        {
            uint32_t chan;
            
            switch(port) {
            case 0:
                chan = T0;
                break;
            case 1:
                chan = T1;
                break;
            }
            
            /** Emulate arrival of data at approximately 31250bps */
            (void)term(TERMREAD, port, 0);
        
            /** Emulate passage of transmit time */
            if( Tn_txTimer[port] > 0u ) Tn_txTimer[port]--;
        
            /* TODO: Consider sending IRQ on TXE */
            if( (Tn_txTimer[port] == 0) &&
                (BYTE_AT(GIO,chan+Tn_CTLb) & Tn_CTLb_TXE) == 0 )
            {
                /* Done transmitting */
                BYTE_AT(GIO,chan+Tn_CTLb) |= Tn_CTLb_TXE;
                /* If IRQs enabled, signal and flag this channel as pending */
                if( BYTE_AT(GIO,chan+Tn_CTLb) & Tn_CTLb_IRQEN )
                {
                    BYTE_AT(GIO,chan+Tn_CTLb) |= (Tn_CTLb_TEI);
                    m68k_raise_irq(TERMn_IRQLVL, M68K_AUTOVECTORED_IRQ);
                }
            }
        }
        break;
    default:
        ret = -1;
        break;
    }
    
    return ret;
}
#else
#error TERM type not defined! (TODO: implement TERM_STDIO, ...)
#endif /* TERM_UDP */


int32_t term_port_setup(void) {
    int32_t ret;
    
    ret = term(TERMINIT, 0, 0);
    ret |= term(TERMINIT, 1, 0);
    
    return ret;
}    




int gio_byte_read(int address)
{
    uint8_t ret = 0xFFu; /* default for unmapped areas */
    
    uint32_t offset = address-GIO_BASE;
    
    //    printf("gio_byte_read() entry - address $%08x\n", address);
    
    switch(offset) {
        /* TIM0 */
    case TIM0+TIM0_CTLb:
        ret = BYTE_AT(GIO,offset);
        break;
        
        /* TIMT */
    case TIMT+TIMT_DAT:
        ret = BYTE_AT(GIO,offset);
        break;
    case TIMT+TIMT_LOAD:
        ret = BYTE_AT(GIO,offset);  /* read acks any pending IRQ */
//        _OK; printf("[TIMT_LOAD ACK read]\n"); N_;
        break;
        
        /* T0,T1 */
    case T0+Tn_TXb:
    case T1+Tn_TXb:
        {
            int chanBase = offset & ~0x0Fu;  /* TERMn BASE */
            ret = BYTE_AT(GIO,offset);
        }
        break;
    case T0+Tn_RXb:
    case T1+Tn_RXb:
        {
            int chanBase = offset & ~0x0Fu;  /* TERMn BASE */
            ret = BYTE_AT(GIO,offset);
//            printf("Read[Rxb]:$%02x\n", ret);
            /* Reading RXb clears PEND, RXF status */
            BYTE_AT(GIO,(chanBase+Tn_CTLb)) &= ~(Tn_CTLb_RXF|Tn_CTLb_RFI);
        }
        break;
    case T0+Tn_CTLb:
    case T1+Tn_CTLb:
        /* Reading CTLb register simply returns all bits */
        ret = BYTE_AT(GIO,offset);
//        printf("Read[CTLb]:$%02x\n", ret);
        break;
    }
    
    return ret;
}

int gio_word_read(int address)
{
    uint32_t offset = address-GIO_BASE;
    return WORD_AT(GIO,offset);
}


void gio_byte_write(int address, int data)
{
    uint32_t offset = address-GIO_BASE;
    
    data &= 0xFF;
    
    switch(offset) {
        /* TIM0 */
    case TIM0+TIM0_CTLb:
        /* Any write to TIM0_CTLb clears PEND */
        data &= ~(TIM0_CTLb_PEND);
        BYTE_AT(GIO,offset) = data;
        break;
    case TIM0+TIM0_DIVb:
        if( data == 0 ) data = 1;  /* 0 is an invalid divisor for TIM0 */
        BYTE_AT(GIO,offset) = data;
        break;
        
        /* TIMT */
    case TIMT+TIMT_DAT:
        break;  /* ignored */
    case TIMT+TIMT_LOAD:
        /* Writing to TIMT_LOAD also loads TIMT_DAT */
        BYTE_AT(GIO,offset) = data;
        BYTE_AT(GIO,TIMT+TIMT_DAT) = data;
        if( data != 0 ) {
            _OK; printf("[TIMT_LOAD written nonzero - IRQ enabled]\n"); N_;
        }
        else {
            _OK; printf("[TIMT_LOAD written zero - IRQ disabled]\n"); N_;
        }
        break;
        
        /* T0,T1 */
    case T0+Tn_TXb:
    case T1+Tn_TXb:
        {
            int chan = offset & ~0x0Fu;  /* TERMn BASE */
            
//            printf("Write[Txb]:$%02x\n", data);
            BYTE_AT(GIO,offset) = data;
            BYTE_AT(GIO,chan+Tn_CTLb) &= ~Tn_CTLb_TXE; /* Clear TXE */
            (void)term(TERMWRITE, TERMCHAN(chan), data);
        }
        break;
    case T0+Tn_CTLb:
    case T1+Tn_CTLb:
        {
            int chan = offset & ~0x0Fu;  /* TERMn BASE */
            
//            printf("Write[Txb]:$%02x\n", data);
            data &= 0xc9; /* restrict to bits that are writable */
            BYTE_AT(GIO,offset) &= ~0xc9; /* preseve non-writable bits...*/
            BYTE_AT(GIO,offset) |= data;  /* ..and write the writable bits :) */
        }
        break;
    }
}

void gio_word_write(int address, int data)
{
    uint32_t offset = address-GIO_BASE;
    switch(offset) {
    case TIM0_DATw:
        /* Any write to TIM0_DATw clears counter to 0 */
        Tim0.tim0offs = (uint16_t)m68k_get_cycles_counter();
        /* Check IRQ logic BEFORE writing to counter data;
         * in case CPU clears reg just before an overflow */
        TIM0_count();
        break;
    case TRC+TRC_DATw:
        _OK; printf("[TRC $%04x]\n", data & 0xFFFF); N_;
        break;
    }
    ((unsigned short *)GIO)[offset>>1] = data & 0xFFFF;
}

void reset_handler(void) {
    /* TODO: reset devices */
    /* Set CPU's SSP, PC to that specified in ROM[0-7]
     * (This is similar to the Atari ST series' boot-remap behaviour
     *  during the first two longword fetches after a poweron/reset)
     */
    uint32_t initial_ssp = ROM[0]<<24 + ROM[1]<<16 + ROM[2]<<8 + ROM[3];
    uint32_t initial_pc =  ROM[4]<<24 + ROM[5]<<16 + ROM[6]<<8 + ROM[7];
    printf("[RESET: from ROM [initial_ssp=$%08x, initial_pc=$%08x]\n",
           initial_ssp, initial_pc);
    m68k_set_register(M68K_REG_A7, initial_ssp);
    m68k_set_register(M68K_REG_PC, initial_pc);
    m68k_control_cycles_counter(1);  /* Reset cycle counter to zero */
    m68k_add_cycles(8); /* FIXME: how many cycles on real CPU for above? */
}

void show_cpu_state(void);

void iack_handler(unsigned level) {
    uint32_t lvl = level & 7;  /* FAME bug? */
    /* TODO: handle behaviour of GIO peripherals when CPU IACKs */
    switch(lvl) {
    case TIMT_IRQLVL:
        _OK; printf("[TIMT iack]\n"); N_;
        break;
    default:
        _ERR; printf("[other irq - level %u]\n", lvl); N_;
        break;
    }
}


int32_t ext_peripheral_setup(void) {
    int32_t ret;
    
    ret = term_port_setup();
    return ret;
}

int32_t ext_peripheral_teardown(void) {
    int32_t ret;
    
    ret = term(TERMDEINIT, 0, 0);
    ret |= term(TERMDEINIT, 1, 0);
    
    return ret;
}

/* Shows the CPU internal state */
void show_cpu_state(void) 
{
    int sr;
    
    printf("PC:%08X\n",m68k_get_pc());
    printf("D0:%08X ",m68k_get_register(M68K_REG_D0));
    printf("D1:%08X ",m68k_get_register(M68K_REG_D1));
    printf("D2:%08X ",m68k_get_register(M68K_REG_D2));
    printf("D3:%08X\n",m68k_get_register(M68K_REG_D3));
    printf("D4:%08X ",m68k_get_register(M68K_REG_D4));
    printf("D5:%08X ",m68k_get_register(M68K_REG_D5));
    printf("D6:%08X ",m68k_get_register(M68K_REG_D6));
    printf("D7:%08X\n",m68k_get_register(M68K_REG_D7));
    printf("A0:%08X ",m68k_get_register(M68K_REG_A0));
    printf("A1:%08X ",m68k_get_register(M68K_REG_A1));
    printf("A2:%08X ",m68k_get_register(M68K_REG_A2));
    printf("A3:%08X\n",m68k_get_register(M68K_REG_A3));
    printf("A4:%08X ",m68k_get_register(M68K_REG_A4));
    printf("A5:%08X ",m68k_get_register(M68K_REG_A5));
    printf("A6:%08X ",m68k_get_register(M68K_REG_A6));
    printf("A7:%08X\n",m68k_get_register(M68K_REG_A7));
    printf("SR:%04X  ",(sr = m68k_get_register(M68K_REG_SR)));
    printf("Flags: %c%c%c%c%c\n",
           (sr >> 4) & 1 ? 'X' : '-',
           (sr >> 3) & 1 ? 'N' : '-',
           (sr >> 2) & 1 ? 'Z' : '-',
           (sr >> 1) & 1 ? 'V' : '-',
           (sr     ) & 1 ? 'C' : '-'
           );
}

void show_periph_state(void) {
    printf("T0: RXD:%02x TXD:%02x BR1:%d BR0:%d TXE:%d RXF:%d RXV:%d TEI:%d RFI:%d IRQEN:%d\n",
           BYTE_AT(GIO+T0,Tn_RXb),
           BYTE_AT(GIO+T0,Tn_TXb),
           (BYTE_AT(GIO+T0,Tn_CTLb) & Tn_CTLb_BR1) != 0,
           (BYTE_AT(GIO+T0,Tn_CTLb) & Tn_CTLb_BR0) != 0,
           (BYTE_AT(GIO+T0,Tn_CTLb) & Tn_CTLb_TXE) != 0,
           (BYTE_AT(GIO+T0,Tn_CTLb) & Tn_CTLb_RXF) != 0,
           (BYTE_AT(GIO+T0,Tn_CTLb) & Tn_CTLb_RXV) != 0,
           (BYTE_AT(GIO+T0,Tn_CTLb) & Tn_CTLb_TEI) != 0,
           (BYTE_AT(GIO+T0,Tn_CTLb) & Tn_CTLb_RFI) != 0,
           (BYTE_AT(GIO+T0,Tn_CTLb) & Tn_CTLb_IRQEN) != 0);
    printf("T1: RXD:%02x TXD:%02x BR1:%d BR0:%d TXE:%d RXF:%d RXV:%d TEI:%d RFI:%d IRQEN:%d\n",
           BYTE_AT(GIO+T1,Tn_RXb),
           BYTE_AT(GIO+T1,Tn_TXb),
           (BYTE_AT(GIO+T1,Tn_CTLb) & Tn_CTLb_BR1) != 0,
           (BYTE_AT(GIO+T1,Tn_CTLb) & Tn_CTLb_BR0) != 0,
           (BYTE_AT(GIO+T1,Tn_CTLb) & Tn_CTLb_TXE) != 0,
           (BYTE_AT(GIO+T1,Tn_CTLb) & Tn_CTLb_RXF) != 0,
           (BYTE_AT(GIO+T1,Tn_CTLb) & Tn_CTLb_RXV) != 0,
           (BYTE_AT(GIO+T1,Tn_CTLb) & Tn_CTLb_TEI) != 0,
           (BYTE_AT(GIO+T1,Tn_CTLb) & Tn_CTLb_RFI) != 0,
           (BYTE_AT(GIO+T1,Tn_CTLb) & Tn_CTLb_IRQEN) != 0);
}

           
/* Performs byte swapping */
void area_swap_bytes(uint8_t *area, int32_t size) 
{
    uint8_t  aux;
    uint32_t i;
    
    for(i=0; i<size-1; i+=2)
    {
        aux = area[i];
        area[i] = area[i+1];
        area[i+1] = aux;
    }
}

/* Program space */
m68k_program prog[] = {
    { RAM_BASE, RAM_BASE+RAM_SIZE-1, (int32_t)RAM - RAM_BASE }, /* RAM */
    { CON_BASE, CON_BASE+CON_SIZE-1, (int32_t)Cons - CON_BASE }, /* Cons */
    { VID_BASE, VID_BASE+VID_SIZE-1, (int32_t)Vid - VID_BASE }, /* Vid */
    { ROM_BASE, ROM_BASE+ROM_SIZE-1, (int32_t)ROM - ROM_BASE }, /* ROM */
    { -1, -1, (uint32_t)NULL }
};

/* Byte Read space */
m68k_data data_rb[] = {
    { RAM_BASE, RAM_BASE+RAM_SIZE-1, NULL, RAM - RAM_BASE },
    { CON_BASE, CON_BASE+CON_SIZE-1, NULL, Cons - CON_BASE },
    { VID_BASE, VID_BASE+VID_SIZE-1, NULL, Vid - VID_BASE },
    { GIO_BASE, GIO_BASE+GIO_SIZE-1, gio_byte_read, NULL },
    { ROM_BASE, ROM_BASE+ROM_SIZE-1, NULL, ROM - ROM_BASE },
    { -1, -1, NULL, NULL }
};

/* Byte Write space */
m68k_data data_wb[] = {
    { RAM_BASE, RAM_BASE+RAM_SIZE-1, NULL, RAM - RAM_BASE },
    { CON_BASE, CON_BASE+CON_SIZE-1, NULL, Cons - CON_BASE },
    { VID_BASE, VID_BASE+VID_SIZE-1, NULL, Vid - VID_BASE },
    { GIO_BASE, GIO_BASE+GIO_SIZE-1, gio_byte_write, NULL },
    { ROM_BASE, ROM_BASE+ROM_SIZE-1, NULL, ROM - ROM_BASE },
    { -1, -1, NULL, NULL }
};

/* Word Read space */
m68k_data data_rw[] = {
    { RAM_BASE, RAM_BASE+RAM_SIZE-1, NULL, RAM - RAM_BASE },
    { CON_BASE, CON_BASE+CON_SIZE-1, NULL, Cons - CON_BASE },
    { VID_BASE, VID_BASE+VID_SIZE-1, NULL, Vid - VID_BASE },
    { GIO_BASE, GIO_BASE+GIO_SIZE-1, gio_word_read, NULL },
    { ROM_BASE, ROM_BASE+ROM_SIZE-1, NULL, ROM - ROM_BASE },
    { -1, -1, NULL, NULL }
};

/* Word Write space */
m68k_data data_ww[] = {
    { RAM_BASE, RAM_BASE+RAM_SIZE-1, NULL, RAM  -RAM_BASE },
    { CON_BASE, CON_BASE+CON_SIZE-1, NULL, Cons -CON_BASE },
    { VID_BASE, VID_BASE+VID_SIZE-1, NULL, Vid  -VID_BASE },
    { GIO_BASE, GIO_BASE+GIO_SIZE-1, gio_word_write, NULL },
    { ROM_BASE, ROM_BASE+ROM_SIZE-1, NULL, ROM - ROM_BASE },
    { -1, -1, NULL, NULL }
};

m68k_context cpuCtx;

int main(int argc, char *argv[]) {
    int status;
    FILE *romFile;
    
    /* Init peripherals */
    GIO_init();
    
    /* Load in ROM */
    romFile = fopen( (argc>1 ? argv[1] : "bootrom"), "r");
    if( romFile == NULL ) {
        perror("fopen");
        return 1;
    }
    else {
        int32_t size_read;
        _OK printf("Read %d bytes from ROM file.\n",
                   (size_read = fread(ROM, 1, ROM_SIZE, romFile))); N_;
        fclose(romFile);
        area_swap_bytes(ROM, size_read);
    }
    
    /* Testing: set up initial SSP, PC */
    ((uint16_t *)RAM)[0] = ((uint16_t *)ROM)[0];
    ((uint16_t *)RAM)[1] = ((uint16_t *)ROM)[1];
    ((uint16_t *)RAM)[2] = ((uint16_t *)ROM)[2];
    ((uint16_t *)RAM)[3] = ((uint16_t *)ROM)[3];
    
    /* Set up CPU context */
    /* TODO: set up user vs. super areas above properly */
    memset(&cpuCtx, 0, sizeof(cpuCtx));
    cpuCtx.sv_fetch = prog;
    cpuCtx.user_fetch = prog;
    
    cpuCtx.sv_read_byte = data_rb;
    cpuCtx.user_read_byte = data_rb;
    cpuCtx.sv_read_word = data_rw;
    cpuCtx.user_read_word = data_rw;
    
    cpuCtx.sv_write_byte = data_wb;
    cpuCtx.user_write_byte = data_wb;
    cpuCtx.sv_write_word = data_ww;
    cpuCtx.user_write_word = data_ww;
    
    cpuCtx.reset_handler = NULL; //reset_handler;
    cpuCtx.iack_handler = NULL; //iack_handler;
    cpuCtx.icust_handler = NULL;
    printf("CPU context initialized.\n");
    
    m68k_set_context(&cpuCtx);
    
    printf("CPU context set.\n");
    m68k_init();
    printf("CPU initialized, ready for RESET.\n");
    
    printf("Initializing external peripherals...\n");
    if( ext_peripheral_setup() != 0 ) {
        _ERR printf("Error!\n"); N_;
        return 1;
    }
    else {
        _OK printf("ok.\n"); N_;
    }
    
    printf("Power-on reset...");
    if( (status = m68k_reset()) != M68K_OK ) {
        _ERR printf("Error - m68k_reset() returned %d\n", status); N_;
        return 1;
    }
    else {
        _OK printf("ok.\n"); N_;
        /* Start 1ms tick timer, target system's 'TimTick' */
        /* TODO: Move timer init into ext_peripheral_setup() */
        printf("Installing TimTick ms-timer...");
        if( installMsTimer(&TimTick_thandle, gio_TimTick, 0) != 0 ) {
            _ERR; printf("Error!\n"); N_;
            return 1;
        }
        else {
            _OK; printf("ok.\n"); N_;
        }
    }
    
    /* Start emulation */
    {
        int32_t done = 0;
        
        printf("Entering emulation.\n");
        do {
            int cpu_status;
            
            m68k_emulate(EMU_TIMESLICE);
            TIM0_count();
            term(TERMSVC, 0, 0);
            term(TERMSVC, 1, 0);
            
            if( (cpu_status = m68k_get_cpu_state()) == M68K_OK ) {
#ifdef TRACE
                show_periph_state();
                show_cpu_state();
                printf("cycles:%d\n", m68k_get_cycles_counter());
#endif
            }
            else {
                /* TRACE condition */
                if( (cpu_status & 48) ) {
                    _ERR printf("[CPU NOTICE: cpu_status=%d]\n", cpu_status); N_;
#if (TRACE_ON_EXCP == 1)
                    show_periph_state();
                    show_cpu_state();
                    printf("[Press a key to continue.]\n");
                    getchar();
#else
                    show_periph_state();
                    show_cpu_state();
                    printf("cycles:%d\n", m68k_get_cycles_counter());
                    return 1;
#endif
                }
                
                if( (cpu_status & 128) ) {
                    _ERR printf("[CPU NOTICE: cpu_status=%d]\n", cpu_status); N_;
                    show_periph_state();
                    show_cpu_state();
                    printf("cycles:%d\n", m68k_get_cycles_counter());
                    getchar();
                }
            }
        }
#ifdef TRACE
        while( getchar() != 'q' );
#else
        while(1);
#endif
        /* Shut down TimTick */
        /* TODO: Move into ext_peripheral_teardown() */
        printf("Disconnecting TimTick...");
        if( uninstallMsTimer(TimTick_thandle) != 0 ) {
            _ERR printf("Error!\n"); N_;
            return 1;
        }
        else {
            _OK printf("ok.\n"); N_;
        }
        
        printf("Deinitializing external peripherals...");
        if( ext_peripheral_teardown() != 0 ) {
            _ERR printf("Error!\n"); N_;
            return 1;
        }
        else {
            _OK printf("ok.\n"); N_;
        }
    }
    return 0;
}

