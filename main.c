/*****************************************************************************
*
*       I2C (TWI) Bare Metal example for KPDA Orange Pi One (H3) BSP
*
******************************************************************************/


#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <assert.h>

#include "h3_i2c.h"
/// definition of in()/out() func: see /home/si/hw/utils/rr/rr.c
#include <hw/inout.h>
/// for mmap_device_io()
#include <sys/mman.h>

//#define DEBUG 1

#define SPTR_CAST(x)     (uintptr_t)&(x)

const char * progname = "PWI_EXAMPLE";

static uint8_t s_slave_address = 0;
static uint32_t s_current_baudrate = 0;

// (from rr)
int print_reg(unsigned long long  port, int num);
// (rr)
uint32_t read_reg(unsigned long long  port);
// (rr)
int write_reg (unsigned long long  port, uint32_t val);

static inline void _cc_write_reg(const uint32_t clk_n, const uint32_t clk_m) {
#ifdef DEBUG
    printf("\nInit Clock Register...\n");
#endif
    uint32_t value = read_reg( SPTR_CAST(EXT_I2C->CC));
#ifdef DEBUG
    printf("%s: clk_n = %d, clk_m = %d\n", __FUNCTION__, clk_n, clk_m);
#endif
    value &= _CAST(uint32_t)(~(CC_CLK_M | CC_CLK_N));
    value |= ((clk_n << CLK_N_SHIFT) | (clk_m << CLK_M_SHIFT));
    write_reg(SPTR_CAST(EXT_I2C->CC), value);

}

static void _set_clock(const uint32_t clk_in, const uint32_t sclk_req) {
    uint32_t clk_m = 0;
    uint32_t clk_n = 0;
    uint32_t _2_pow_clk_n = 1;
    const uint32_t src_clk = clk_in / 10;
    uint32_t divider = src_clk / sclk_req;
    uint32_t sclk_real = 0;

    assert(divider != 0);

    while (clk_n < (CLK_N_MASK + 1)) {
        /* (m+1)*2^n = divider -->m = divider/2^n -1 */
        clk_m = (divider / _2_pow_clk_n) - 1;
        /* clk_m = (divider >> (_2_pow_clk_n>>1))-1 */

        while (clk_m < (CLK_M_MASK + 1)) {
            sclk_real = src_clk / (clk_m + 1) / _2_pow_clk_n; /* src_clk/((m+1)*2^n) */

            if (sclk_real <= sclk_req) {
                _cc_write_reg(clk_n, clk_m);
                return;
            } else {
                clk_m++;
            }
        }

        clk_n++;
        _2_pow_clk_n *= 2;
    }

    _cc_write_reg(clk_n, clk_m);

    return;
}

/* enable SDA or SCL */
static void twi_enable_lcr(unsigned int sda_scl)
{
    uint32_t reg_val = read_reg( SPTR_CAST(EXT_I2C->LCR));
    sda_scl &= 0x01;
    if (sda_scl)
        reg_val |= LCR_SCL_EN;/* enable scl line control */
    else
        reg_val |= LCR_SDA_EN;/* enable sda line control */

    write_reg(SPTR_CAST(EXT_I2C->LCR), reg_val);
#ifdef DEBUG
    printf("\n_LCR SDA/SCL is enabled. EXT_I2C->LCR: 0x%0X\n", reg_val);
#endif
}

/* disable SDA or SCL */
static void twi_disable_lcr(unsigned int sda_scl)
{
    uint32_t reg_val = read_reg( SPTR_CAST(EXT_I2C->LCR));
    sda_scl &= 0x01;
    if (sda_scl)
        reg_val &= ~LCR_SCL_EN;/* disable scl line control */
    else
        reg_val &= ~LCR_SDA_EN;/* disable sda line control */

    write_reg(SPTR_CAST(EXT_I2C->LCR), reg_val);
#ifdef DEBUG
    printf("\n_LCR SDA/SCL is disabled. EXT_I2C->LCR: 0x%0X\n", reg_val);
#endif
}

/* get SDA state */
static unsigned int twi_get_sda()
{
    unsigned int status = 0;
    status = TWI_LCR_SDA_STATE_MASK & read_reg( SPTR_CAST(EXT_I2C->LCR));
    status >>= 4;

#ifdef DEBUG
    printf("\nin LCR SDA Level is 0x%0X\n", status);
#endif

    return  (status&0x1);
}

/* set SCL level(high/low), only when SCL enable */
static void twi_set_scl(unsigned int hi_lo)
{
    unsigned int reg_val = read_reg( SPTR_CAST(EXT_I2C->LCR));
    reg_val &= ~TWI_LCR_SCL_CTL;
    hi_lo   &= 0x01;
    reg_val |= (hi_lo<<3);
    write_reg(SPTR_CAST(EXT_I2C->LCR), reg_val);
#ifdef DEBUG
    printf("LCR is set to 0x%0X\n", reg_val);
#endif
}

/* set SDA level(high/low), only when SDA enable */
static void twi_set_sda(unsigned int hi_lo)
{
    unsigned int reg_val = read_reg( SPTR_CAST(EXT_I2C->LCR));
    reg_val &= ~TWI_LCR_SDA_CTL;
    hi_lo   &= 0x01;
    reg_val |= (hi_lo<<1);
    write_reg(SPTR_CAST(EXT_I2C->LCR), reg_val);
#ifdef DEBUG
    printf("LCR is set to 0x%0X\n", reg_val);
#endif
}

/* send 9 clock to released sda */
static int twi_send_clk_9pulse()
{
    int twi_scl = 1;
    //int twi_sda = 0;
    int low = 0;
    int high = 1;
    int cycle = 0;

    //uint32_t reg_value;

    /* enable scl control */
    twi_enable_lcr(twi_scl);
    /* enable sda control */
    //twi_enable_lcr(twi_sda);

    while (cycle < 9)
    {
//        if (twi_get_sda()
//            && twi_get_sda()
//            && twi_get_sda()) {
//            break;
//        }

        /* twi_scl -> low */
        twi_set_scl(high);
//        if(cycle == 8)
//        {
//            twi_set_sda(low);
//            usleep(1);
//        }

        /* twi_scl -> high */
        twi_set_scl(low);
        //usleep(1);
        cycle++;
    }

//    if (twi_get_sda()) {
        twi_disable_lcr(twi_scl);
        //twi_disable_lcr(twi_sda);
        return H3_I2C_OK;

}

static int32_t _stop() {
    int32_t time = TIMEOUT;
    uint32_t tmp_val;

#ifdef DEBUG
    printf("\n_stop() now...\n");
#endif

    tmp_val = read_reg(SPTR_CAST(EXT_I2C->CTL));
    tmp_val |= (0x01 << 4);                         // Master Mode Stop (bit #4)
    write_reg(SPTR_CAST(EXT_I2C->CTL), tmp_val);

    //while ((time--) && (EXT_I2C->CTL & 0x10))
    while ((time--) && (read_reg(SPTR_CAST(EXT_I2C->CTL)) & 0x10))       // wait when Stop bit is down
        /*printf("wait stop bit is down\n")*/;

    if (time <= 0) {
        return -H3_I2C_NOK_TOUT;
    }

    time = TIMEOUT;
    //while ((time--) && (EXT_I2C->STAT != STAT_READY))
    while ((time--) && (read_reg(SPTR_CAST(EXT_I2C->STAT)) != STAT_READY))
        ;

    if (time <= 0) {
        return -H3_I2C_NOK_TOUT;
    }
#ifdef DEBUG
    printf ("\nIn %s STAT is:\n", __FUNCTION__);
    print_reg(SPTR_CAST(EXT_I2C->STAT), 1);
    printf("_stop() is OK\n");
#endif

    return H3_I2C_OK;
}

static int32_t _sendstart() {
    int32_t time = 0xff;
    uint32_t tmp_val;

#ifdef DEBUG
    printf ("\nEnter to _sendstart()\n");
#endif

    write_reg(SPTR_CAST(EXT_I2C->EFR), 0x0);          // Data Byte to be written after read command
#ifdef DEBUG
    printf ("\nEFR->0 is OK, Soft Reset TWI0 now... \n");
#endif
    write_reg(SPTR_CAST(EXT_I2C->SRST), 1);
//    printf ("SRST->1 is OK\n");
    usleep(100);

#ifdef DEBUG
    printf("Before sendstart() CTL->0x%0X\n", read_reg(SPTR_CAST(EXT_I2C->CTL)) );
    printf("Before sendstart() STAT->0x%0X\n", read_reg(SPTR_CAST(EXT_I2C->STAT)) );
//    tmp_val = (read_reg(SPTR_CAST(EXT_I2C->CTL)) & 0x08);
//    printf("res = 0x%0X\n", tmp_val);
#endif

    tmp_val = read_reg(SPTR_CAST(EXT_I2C->CTL));
    tmp_val  |= 0x20;                               // Master Mode start (bit #5)
    //tmp_val &= ~CTL_INT_FLAG;
    //printf("Write to EXT_I2C->CTL 0x%0X\n", tmp_val);

    write_reg(SPTR_CAST(EXT_I2C->CTL), tmp_val);

//    tmp_val = (read_reg(SPTR_CAST(EXT_I2C->CTL)) & 0x08);
//    printf("res = 0x%0X\n", tmp_val);

    while ((time--) && (!(read_reg(SPTR_CAST(EXT_I2C->CTL)) & 0x08)))   // wait INT_FLAG
        printf("in _sendstart() wait INT_FLAG time = 0x%0X\n", time)
        ;

    if (time <= 0) {
        return -H3_I2C_NOK_TOUT;
    }

    tmp_val = read_reg(SPTR_CAST(EXT_I2C->STAT));

    if (tmp_val != STAT_START_TRANSMIT) {
        return -STAT_START_TRANSMIT;
    }
#ifdef DEBUG
    printf("\nAfter _sendstart() CTL->0x%0X\n", read_reg(SPTR_CAST(EXT_I2C->CTL)) );
    printf("After _sendstart() STAT->0x%0X\n", tmp_val );
    printf("\n%s is OK\n", __FUNCTION__);
#endif

    // ****** try to unset INT
    tmp_val = read_reg(SPTR_CAST(EXT_I2C->CTL));
    tmp_val &= ~CTL_INT_FLAG;
    //printf("Write to EXT_I2C->CTL 0x%0X\n", tmp_val);
    write_reg(SPTR_CAST(EXT_I2C->CTL), tmp_val);
    // ******

    usleep(200);
    return H3_I2C_OK;
}

static int32_t _sendrestart() {
    int32_t time = 0xff;
    uint32_t tmp_val;

#ifdef DEBUG
    printf ("\nEnter to _sendrestart()\n");
#endif


#ifdef DEBUG
    printf("Before _sendrestart() CTL->0x%0X\n", read_reg(SPTR_CAST(EXT_I2C->CTL)) );
    printf("Before _sendrestart() STAT->0x%0X\n", read_reg(SPTR_CAST(EXT_I2C->STAT)) );
//    tmp_val = (read_reg(SPTR_CAST(EXT_I2C->CTL)) & 0x08);
//    printf("res = 0x%0X\n", tmp_val);
#endif

    tmp_val = read_reg(SPTR_CAST(EXT_I2C->CTL));
    tmp_val  |= 0x20;                               // Master Mode start (bit #5)
    //tmp_val &= ~CTL_INT_FLAG;
    //printf("Write to EXT_I2C->CTL 0x%0X\n", tmp_val);

    write_reg(SPTR_CAST(EXT_I2C->CTL), tmp_val);

//    tmp_val = (read_reg(SPTR_CAST(EXT_I2C->CTL)) & 0x08);
//    printf("res = 0x%0X\n", tmp_val);

    while ((time--) && (!(read_reg(SPTR_CAST(EXT_I2C->CTL)) & 0x08)))   // wait INT_FLAG
        printf("in _sendrestart() wait INT_FLAG time = 0x%0X\n", time)
        ;

    if (time <= 0) {
        return -H3_I2C_NOK_TOUT;
    }

    tmp_val = read_reg(SPTR_CAST(EXT_I2C->STAT));

#ifdef DEBUG
    printf("\nAfter _sendrestart() CTL->0x%0X\n", read_reg(SPTR_CAST(EXT_I2C->CTL)) );
    printf("After _sendrestart() STAT->0x%0X\n", tmp_val );
    printf("\n%s is OK\n", __FUNCTION__);
#endif

    if (tmp_val != STAT_RESTART_TRANSMIT) {
        return -STAT_RESTART_TRANSMIT;
    }


    return H3_I2C_OK;
}


static int32_t _send_stop_start() {
    int32_t time = 0xff;
    uint32_t tmp_val;

    printf ("\nEnter to _send_stop_start()\n");

    printf ("\nIn sendstart() CTL is:\n");
    print_reg(SPTR_CAST(EXT_I2C->CTL), 1);
    tmp_val = read_reg(SPTR_CAST(EXT_I2C->CTL));
    tmp_val  |= 0x30;                               // Master Mode start (bit #5) and stop bit (#4)
    printf("Write to EXT_I2C->CTL 0x%0X\n", tmp_val);

    write_reg(SPTR_CAST(EXT_I2C->CTL), tmp_val);

    while ((time--) && (!(read_reg(SPTR_CAST(EXT_I2C->CTL)) & 0x08)))   // wait INT_FLAG
        printf("in _send_stop_start() wait INT_FLAG time = 0x%0X\n", time)
        ;

    if (time <= 0) {
        return -H3_I2C_NOK_TOUT;
    }

    tmp_val = read_reg(SPTR_CAST(EXT_I2C->STAT));
//    printf("\nAfter _sendstart STAT->0x%0X\n", tmp_val );

    if (tmp_val != STAT_START_TRANSMIT) {
        return -STAT_START_TRANSMIT;
    }

    printf("\nAfter _send_stop_start STAT->0x%0X\n", tmp_val );
    printf("\n%s is OK\n", __FUNCTION__);
    return H3_I2C_OK;
}

static int32_t _sendslaveaddr(uint32_t mode) {
    int32_t i = 0, time = TIMEOUT;
    uint32_t tmp_val;

    mode &= 1;

#ifdef DEBUG
    printf("\n_sendslave() started...\n");
    printf("\nBefore _sendslave() CTL->0x%0X\n", read_reg(SPTR_CAST(EXT_I2C->CTL)) );
#endif

    tmp_val = _CAST(uint32_t)(s_slave_address << 1) | mode;
#ifdef DEBUG
    printf(" address to be write - 0x%0X\n", tmp_val);
#endif
    write_reg(SPTR_CAST(EXT_I2C->DATA), tmp_val);
//    printf("\nIn %s DATA is:\n", __FUNCTION__);
//    print_reg(SPTR_CAST(EXT_I2C->DATA), 1);


    tmp_val = read_reg(SPTR_CAST(EXT_I2C->CTL));
    tmp_val |= (0x01 << 3);                              // INT_FLAG ^
    //tmp_val |= CTL_A_ACK;
    write_reg(SPTR_CAST(EXT_I2C->CTL), tmp_val);

    //while ((time--) && (!(EXT_I2C->CTL & 0x08)))
    while ((time--) && (!(read_reg(SPTR_CAST(EXT_I2C->CTL)) & 0x08)))   // wait INT_FLAG
        ;

    if (time <= 0) {
        return -H3_I2C_NOK_TOUT;
    }

    //printf("***2***CTL->0x%0X\n", read_reg(SPTR_CAST(EXT_I2C->CTL)) );
    tmp_val = read_reg(SPTR_CAST(EXT_I2C->STAT));
#ifdef DEBUG
    printf("\nAfter _sendslave() CTL->0x%0X\n", read_reg(SPTR_CAST(EXT_I2C->CTL)) );
    printf("STAT->0x%0X\n", tmp_val);
#endif

    if (mode == I2C_MODE_WRITE) {
        if (tmp_val != STAT_ADDRWRITE_ACK) {
            return -STAT_ADDRWRITE_ACK;
        }
    } else {
        if (tmp_val != STAT_ADDRREAD_ACK) {
            return -STAT_ADDRREAD_ACK;
        }
    }
#ifdef DEBUG
    printf("%s() is OK\n", __FUNCTION__);
#endif
    return H3_I2C_OK;
}

static int32_t _getdata(uint8_t *data_addr, uint32_t data_count) {

    return H3_I2C_OK;
}

static int _reset_htu(char *buffer, int len) {
    int i, ret, ret0 = -1;
    int32_t time = 0xff7;
    uint32_t tmp_val;

    ret = _sendstart();
    if (ret) {
        goto i2c_read_err_occur;
    }


    ret = _sendslaveaddr(I2C_MODE_WRITE);
#ifdef DEBUG
    printf ("\n_sendslaveaddr return %d\n", ret);
    printf("+CTL->0x%0X\n", read_reg(SPTR_CAST(EXT_I2C->CTL)) );

#endif

    printf("\n send 0xFE (Reset)\n");
        write_reg(SPTR_CAST(EXT_I2C->DATA), 0xFE);

    tmp_val = read_reg(SPTR_CAST(EXT_I2C->CTL));
    tmp_val |= (0x01 << 3);                              // INT_FLAG
   // printf ("***tmp_val = 0x%0X\n", tmp_val);
    write_reg(SPTR_CAST(EXT_I2C->CTL), tmp_val);

    while ((time--) && ((read_reg(SPTR_CAST(EXT_I2C->STAT)) != 0x28)))
        printf(" wait 0x28 time = 0x%0X\n", time);
#ifdef DEBUG
    printf("+STAT->0x%0X\n", read_reg(SPTR_CAST(EXT_I2C->STAT)) );
    printf("+CTL->0x%0X\n", read_reg(SPTR_CAST(EXT_I2C->CTL)) );
#endif

    printf ("\nfinish DATA = 0x%0X\n", read_reg(SPTR_CAST(EXT_I2C->DATA)));

    printf("+STAT->0x%0X\n", read_reg(SPTR_CAST(EXT_I2C->STAT)) );
    printf("+CTL->0x%0X\n", read_reg(SPTR_CAST(EXT_I2C->CTL)) );

    if (ret) {
        goto i2c_read_err_occur;
    }

    ret0 = 0;

    i2c_read_err_occur: _stop();

    return ret0;
}

static int _read_user_reg() {
    int i = 0, ret, ret0 = -1;
    int32_t time = 0xff;
    uint32_t tmp_val;

    ret = _sendstart();
    if (ret) {
        goto i2c_read_err_occur;
    }

    ret = _sendslaveaddr(I2C_MODE_WRITE);
#ifdef DEBUG
    printf ("\n_sendslaveaddr return %d\n", ret);
    printf("+CTL->0x%0X\n", read_reg(SPTR_CAST(EXT_I2C->CTL)) );
#endif

    if (ret) {
        goto i2c_read_err_occur;
    }

    printf ("\n send 0xE7 (Read User Reg)\n");
    write_reg(SPTR_CAST(EXT_I2C->DATA), 0xE7);

    tmp_val = read_reg(SPTR_CAST(EXT_I2C->CTL));
    tmp_val |= (0x01 << 3);                              // INT_FLAG
   // printf ("***tmp_val = 0x%0X\n", tmp_val);
    write_reg(SPTR_CAST(EXT_I2C->CTL), tmp_val);

    while ((time--) && ((read_reg(SPTR_CAST(EXT_I2C->STAT)) != 0x28)))
        /*printf(" wait 0x28 time = 0x%0X\n", time)*/;
#ifdef DEBUG
    printf("+STAT->0x%0X\n", read_reg(SPTR_CAST(EXT_I2C->STAT)) );
    printf("+CTL->0x%0X\n", read_reg(SPTR_CAST(EXT_I2C->CTL)) );
#endif

    //try to send restart
    ret = _sendrestart();
    if (ret) {
        goto i2c_read_err_occur;
    }

    ret = _sendslaveaddr(I2C_MODE_READ);

    if (ret) {
        goto i2c_read_err_occur;
    }

    tmp_val = read_reg(SPTR_CAST(EXT_I2C->CTL));
    //tmp_val |= CTL_A_ACK;             // <====== if ACK needed!!!!
    tmp_val |= CTL_INT_FLAG;
    write_reg(SPTR_CAST(EXT_I2C->CTL), tmp_val);

    while ((time--) && ((read_reg(SPTR_CAST(EXT_I2C->STAT)) != 0x58)))
        ;

#ifdef DEBUG
    printf(" ->STAT->0x%0X\n", read_reg(SPTR_CAST(EXT_I2C->STAT)) );
    printf(" ->CTL->0x%0X\n", read_reg(SPTR_CAST(EXT_I2C->CTL)) );
#endif

    printf ("\n Read User Reg in DATA reg <= 0x%0X\n", read_reg(SPTR_CAST(EXT_I2C->DATA)));

    // stop / start
    ret = _sendrestart();

    ret = _sendslaveaddr(I2C_MODE_WRITE);
#ifdef DEBUG
    printf ("\n_sendslaveaddr return %d\n", ret);
    printf("+CTL->0x%0X\n", read_reg(SPTR_CAST(EXT_I2C->CTL)) );
#endif

    if (ret) {
        goto i2c_read_err_occur;
    }

    printf ("\n send 0xE6 (Write User Reg)\n");
    write_reg(SPTR_CAST(EXT_I2C->DATA), 0xE6);

    tmp_val = read_reg(SPTR_CAST(EXT_I2C->CTL));
    tmp_val |= (0x01 << 3);                              // INT_FLAG
   // printf ("***tmp_val = 0x%0X\n", tmp_val);
    write_reg(SPTR_CAST(EXT_I2C->CTL), tmp_val);

    while ((time--) && ((read_reg(SPTR_CAST(EXT_I2C->STAT)) != 0x28)))
        /*printf(" wait 0x28 time = 0x%0X\n", time)*/;
#ifdef DEBUG
    printf("+STAT->0x%0X\n", read_reg(SPTR_CAST(EXT_I2C->STAT)) );
    printf("+CTL->0x%0X\n", read_reg(SPTR_CAST(EXT_I2C->CTL)) );
#endif

    uint16_t user_reg_val = 0x03;
    printf ("\n Try to write User Reg val => 0x%0X\n", user_reg_val);

    // Register Content to be written
    write_reg(SPTR_CAST(EXT_I2C->DATA), user_reg_val);

    tmp_val = read_reg(SPTR_CAST(EXT_I2C->CTL));
    tmp_val |= (0x01 << 3);                              // INT_FLAG
    write_reg(SPTR_CAST(EXT_I2C->CTL), tmp_val);

    ret0 = 0;

    i2c_read_err_occur: _stop();

    return ret0;
}

static int _read(char *buffer, int len, uint8_t reg_code) {
    int i=0, ret, ret0 = -1;
    int32_t time = 0xffff;
    uint32_t tmp_val;

    uint8_t byte1, byte2;

    ret = _sendstart();
    if (ret) {
        goto i2c_read_err_occur;
    }


    ret = _sendslaveaddr(I2C_MODE_WRITE);
#ifdef DEBUG
    printf ("\n_sendslaveaddr return %d\n", ret);
    printf("+CTL->0x%0X\n\n", read_reg(SPTR_CAST(EXT_I2C->CTL)) );
#endif

    if (ret) {
        goto i2c_read_err_occur;
    }

    write_reg(SPTR_CAST(EXT_I2C->DATA), reg_code);

    tmp_val = read_reg(SPTR_CAST(EXT_I2C->CTL));
    tmp_val |= (0x01 << 3);                              // ^^^ INT_FLAG
    write_reg(SPTR_CAST(EXT_I2C->CTL), tmp_val);

    while ((time--) && ((read_reg(SPTR_CAST(EXT_I2C->STAT)) != 0x28)))
        /*printf(" wait STAT 0x28 time = 0x%0X\n", time)*/;


    if (time <= 0)
    {
        goto i2c_read_err_occur;
    }
    else
    {
#ifdef DEBUG
    printf ("\n Send 0x%0X (Reg Addr) is OK\n\n", reg_code);
    printf("+STAT->0x%0X\n", read_reg(SPTR_CAST(EXT_I2C->STAT)) );
    printf("+CTL->0x%0X\n", read_reg(SPTR_CAST(EXT_I2C->CTL)) );
#endif
    }

    ret = _sendrestart();
    if (ret) {
        goto i2c_read_err_occur;
    }

#ifdef DEBUG
    printf("_restart() is OK\n");
    printf("STAT->0x%0X\n", read_reg(SPTR_CAST(EXT_I2C->STAT)) );
#endif

    ret = _sendslaveaddr(I2C_MODE_READ);

    if (ret) {
        goto i2c_read_err_occur;
    }

#ifdef DEBUG
    printf("++STAT->0x%0X\n", read_reg(SPTR_CAST(EXT_I2C->STAT)) );
    printf("++CTL->0x%0X\n", read_reg(SPTR_CAST(EXT_I2C->CTL)) );
#endif


    while ((time--) && (!(read_reg(SPTR_CAST(EXT_I2C->CTL)) & 0x08)))   // wait INT_FLAG
        printf("*");

    // Receiving data
    printf("\nReceiving data:\n");
    printf("===============\n");

    time = 0xffff;
    tmp_val = read_reg(SPTR_CAST(EXT_I2C->CTL));
    tmp_val |= CTL_A_ACK;
    tmp_val |= CTL_INT_FLAG;
    write_reg(SPTR_CAST(EXT_I2C->CTL), tmp_val);

    while ((time--) && ((read_reg(SPTR_CAST(EXT_I2C->STAT)) != 0x50)))
        ;

    tmp_val = read_reg(SPTR_CAST(EXT_I2C->DATA));
    byte1 = (uint8_t)tmp_val;
    *buffer = (char)tmp_val;
    printf (" Data(MSB) = 0x%0X\n", /*tmp_val*/byte1);
#ifdef DEBUG
    printf("Measurement time in proc tick = %d\n", 0xffff-time);
    printf("++STAT->0x%0X\n", read_reg(SPTR_CAST(EXT_I2C->STAT)) );
    printf("++CTL->0x%0X\n", read_reg(SPTR_CAST(EXT_I2C->CTL)) );
#endif
    tmp_val = read_reg(SPTR_CAST(EXT_I2C->CTL));
    tmp_val |= CTL_A_ACK;
    tmp_val |= CTL_INT_FLAG;
    write_reg(SPTR_CAST(EXT_I2C->CTL), tmp_val);

    while ((time--) && ((read_reg(SPTR_CAST(EXT_I2C->STAT)) != 0x50)))
        ;

    tmp_val = read_reg(SPTR_CAST(EXT_I2C->DATA));
    byte2 = (uint8_t)tmp_val;
    buffer[1] = (char)tmp_val;
    printf (" Data(LSB) = 0x%0X\n", /*tmp_val*/byte2);
#ifdef DEBUG
    printf("Measurement time in proc tick = %d\n", 0xffff-time);
    printf("++STAT->0x%0X\n", read_reg(SPTR_CAST(EXT_I2C->STAT)) );
    printf("++CTL->0x%0X\n", read_reg(SPTR_CAST(EXT_I2C->CTL)) );
#endif

    tmp_val = read_reg(SPTR_CAST(EXT_I2C->CTL));
    tmp_val &= ~CTL_A_ACK;
    tmp_val |= CTL_INT_FLAG;
    write_reg(SPTR_CAST(EXT_I2C->CTL), tmp_val);

    while ((time--) && ((read_reg(SPTR_CAST(EXT_I2C->STAT)) != 0x58)))
        ;

    tmp_val = read_reg(SPTR_CAST(EXT_I2C->DATA));
    buffer[2] = (char)tmp_val;
    printf (" CHECKSUM  = 0x%0X\n", tmp_val);
#ifdef DEBUG
    printf("Measurement time in proc tick = %d\n", 0xffff-time);
    printf("++STAT->0x%0X\n", read_reg(SPTR_CAST(EXT_I2C->STAT)) );
    printf("++CTL->0x%0X\n", read_reg(SPTR_CAST(EXT_I2C->CTL)) );
#endif

#ifdef DEBUG
    printf("++STAT->0x%0X\n", read_reg(SPTR_CAST(EXT_I2C->STAT)) );
    printf("++CTL->0x%0X\n", read_reg(SPTR_CAST(EXT_I2C->CTL)) );
#endif

    ret0 = 0;

    i2c_read_err_occur: _stop();

    return ret0;
}


static int _read_nohold_master(char *buffer, int len) {

    return 0;
}

static int _write(const char *buffer, const int len) {

    return 0;
}

uint8_t h3_i2c_write(const char *buffer, uint32_t data_length) {
    const auto ret = _write(buffer, data_length);
#ifdef DEBUG
    if (ret) {
        printf("ret=%d\n", ret);
    }
#endif
    return _CAST(uint8_t)(-ret);
}

uint8_t h3_i2c_read(char *buffer, uint32_t data_length, uint8_t reg_code) {

    const auto ret = _read(buffer, _CAST(int)(data_length), reg_code);

#ifdef DEBUG
    if (ret) {
        printf("%s ret=%d\n", __FUNCTION__, ret);
    }
#endif
    return _CAST(uint8_t)(-ret);
}

void h3_i2c_set_baudrate(const uint32_t nBaudrate) {
    assert(nBaudrate <= H3_I2C_FULL_SPEED);

    if (__builtin_expect((s_current_baudrate != nBaudrate), 0)) {
        s_current_baudrate = nBaudrate;
        _set_clock(H3_F_24M, nBaudrate);
    }
}

void h3_i2c_set_slave_address(const uint8_t nAddress) {
    s_slave_address = nAddress;
}

int htu21d_i2c_connected(const uint8_t nAddress, const uint32_t nBaudrate) {
    h3_i2c_set_slave_address(nAddress);
    h3_i2c_set_baudrate(nBaudrate);

    uint8_t nResult;

#ifdef DEBUG
    printf("\n %s(). Device addr = 0x%0X\n", __FUNCTION__, nAddress);
#endif

    nResult = _read_user_reg();

    return (nResult == 0) ? 1 : 0;
}

int main()
{
    uint32_t i;

    printf("\nH3 TWI bare metal application started...\n");

//    printf( "H3 sizeof ull %u byte\n", sizeof(unsigned long long));
//    printf( "H3 sizeof register %d byte\n", sizeof(&(EXT_I2C->CC)) );
//    port =  (uintptr_t) &(EXT_I2C->STAT);         // type conversion used

#ifdef DEBUG
    printf( "\nTWI0 init...\n");
#endif

    //    printf ("SRST->1 is OK\n");
    if ( write_reg(SPTR_CAST(EXT_I2C->CTL), 0xC0) )  return (-1);
    //usleep(1000);

    while ( ++i < 10 && (read_reg(SPTR_CAST(EXT_I2C->CTL)) != 0xC0)) printf("wait CTL to set 0xC0...");

#ifdef DEBUG
    printf( "\nTWI0 Control Register (EXT_I2C->CTL):\n");
    print_reg(SPTR_CAST(EXT_I2C->CTL), 1);

    printf( "\nTWI0 Status Register (EXT_I2C->STAT):\n");
    if ( print_reg(SPTR_CAST(EXT_I2C->STAT), 1) < 0 ) return (-1);
#endif

    h3_i2c_set_slave_address(0x40);
    h3_i2c_set_baudrate(H3_I2C_NORMAL_SPEED);

//    if ( !htu21d_i2c_connected(0x40, H3_I2C_NORMAL_SPEED) ) printf("Error occured\n");

    uint8_t * buffer;
    buffer = malloc(sizeof(uint8_t)*3);

    if ( h3_i2c_read(buffer, 3, 0xE3) )
    {
        printf("Error occured\n");
        return (-1);
    }

    unsigned int sensor_bytes = (buffer[0] << 8 | buffer[1]) & 0xFFFC;
    double sensor_float = sensor_bytes / 65536.0;
    double measure = 0;


    measure = -46.85 + (175.72 * sensor_float);
    printf("=============\n");
    printf(" T=%f\n", measure);
    printf("=============\n");

    h3_i2c_read(buffer, 3, 0xE5);
    sensor_bytes = (buffer[0] << 8 | buffer[1]) & 0xFFFC;
    sensor_float = sensor_bytes / 65536.0;

    measure = -6.0 + (125.0 * sensor_float);
    printf("=============\n");
    printf(" H=%5.2f%%rh\n", measure);
    printf("=============\n");

#ifdef DEBUG
    printf( "\nEnd of prg...\n");
    printf("TWI0 Status register (EXT_I2C->STAT):\n");
    printf("STAT->0x%0X\n", read_reg(SPTR_CAST(EXT_I2C->STAT)) );

    printf( "\nTWI0 Control Register (EXT_I2C->CTL):\n");
    printf("CTL->0x%0X\n", read_reg(SPTR_CAST(EXT_I2C->CTL)) );

    usleep(1000);
    twi_get_sda();
#endif

    return 0;
}

int print_reg(unsigned long long  port, int num)
{
    uintptr_t           base;
    printf( "Read %d dwords from 0x%0llX\n", num, port );

    if ( (base = mmap_device_io( num, port )) == MAP_DEVICE_FAILED )
       {
           perror( "mmap_device_io()" );
           return (-1);
       }

    printf( "[0x%X]\t 0x%0X\n", 0, in32( base ) );
    return 0;
}

uint32_t read_reg(unsigned long long  port)
{
    uintptr_t base;

    if ( (base = mmap_device_io( 1, port )) == MAP_DEVICE_FAILED )
       {
           perror( "mmap_device_io()" );
       }

    return in32( base );
}

int write_reg (unsigned long long  port, uint32_t val)
{
    uintptr_t base;

    if ( (base = mmap_device_io( 1, port )) == MAP_DEVICE_FAILED )
    {
        perror( "mmap_device_io()" );
        return (-1);
    }

    out32( base, val );
    return (0);
}
