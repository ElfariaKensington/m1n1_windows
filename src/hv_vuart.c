/* SPDX-License-Identifier: MIT */

#include "../build/build_cfg.h"
#include "hv.h"
#include "hv_runtime_diag.h"
#include "aic.h"
#include "iodev.h"
#include "uart.h"
#include "uart_regs.h"
#include "usb.h"

bool active = false;

u32 ucon = 0;
u32 utrstat = 0;
u32 ufstat = 0;

int vuart_irq = 0;

static void update_irq(void)
{
    // Pump USB events for …43 so the emulated PL011 (kd) still sees incoming host bytes.
    iodev_handle_events(IODEV_USB_VUART);

    // Firmware DEBUG text always goes to the m1n1 console (…41 / hv.log). Host INPUT on …43 is
    // shared: the S5L UART consumes it only until kdcom first touches the emulated PL011, after
    // which the channel belongs to the debugger alone. Reporting RX after that would steal kd
    // bytes and raise phantom RX interrupts.
    utrstat |= UTRSTAT_TXBE | UTRSTAT_TXE;
    utrstat &= ~UTRSTAT_RXD;
    ufstat = 0;

    // Advertise received data while the debugger has not taken the channel over.
    ssize_t rx_queued;
    if (!pl011_kd_live() && (rx_queued = iodev_can_read(IODEV_USB_VUART))) {
        utrstat |= UTRSTAT_RXD;
        ufstat = rx_queued > 15 ? (FIELD_PREP(UFSTAT_RXCNT, 15) | UFSTAT_RXFULL)
                                : FIELD_PREP(UFSTAT_RXCNT, rx_queued);
        if (FIELD_GET(UCON_RXMODE, ucon) == UCON_MODE_IRQ && ucon & UCON_RXTO_ENA)
            utrstat |= UTRSTAT_RXTO;
    }

    if (FIELD_GET(UCON_TXMODE, ucon) == UCON_MODE_IRQ && ucon & UCON_TXTHRESH_ENA) {
        utrstat |= UTRSTAT_TXTHRESH;
    }

    if (vuart_irq) {
        uart_clear_irqs();
        if (utrstat & (UTRSTAT_TXTHRESH | UTRSTAT_RXTHRESH | UTRSTAT_RXTO)) {
            aic_set_sw(vuart_irq, true);
        } else {
            aic_set_sw(vuart_irq, false);
        }
    }
}

static void handle_vuart_passthrough(uint8_t b)
{
    if (!hv_runtime_diag_enabled())
        return;

#ifdef M1N1_J316S_SETUP
    /* Each framebuffer console write copies the whole scanout buffer. */
    static char line[256] = "FW> ";
    static size_t used = sizeof("FW> ") - 1;

    if (b == '\r')
        return;
    line[used++] = b;
    if (b == '\n' || used == sizeof(line) - 1) {
        if (b != '\n')
            line[used++] = '\n';
        iodev_console_write(line, used);
        used = sizeof("FW> ") - 1;
    }
#else
    // Firmware S5L console -> m1n1 console (…41 / hv.log), kept off the kd channel. Each
    // line is prefixed so it is distinguishable from m1n1's own log output.
    static bool sol = true;

    if (b == '\r')
        return;
    if (b == '\n') {
        printf("\n");
        sol = true;
        return;
    }
    if (sol) {
        printf("FW> ");
        sol = false;
    }
    printf("%c", b);
#endif
}

static bool handle_vuart(struct exc_info *ctx, u64 addr, u64 *val, bool write, int width)
{
    UNUSED(ctx);
    UNUSED(width);

    // Second page of the UART block is the emulated PL011 for the Windows kernel
    // debugger (see hv_pl011.c). It shares this proven hook so it is actually mapped.
    if (addr & 0x1000)
        return pl011_reg(addr & 0xfff, val, write);

    addr &= 0xfff;

    update_irq();

    if (write) {
        //         printf("HV: vuart W 0x%lx <- 0x%lx (%d)\n", addr, *val, width);
        switch (addr) {
            case UCON:
                ucon = *val;
                break;
            case UTXH: {
                uint8_t b = *val;
                // Firmware console goes to the m1n1 log channel only; …43 is kd-only now.
                handle_vuart_passthrough(b);
                break;
            }
            case UTRSTAT:
                utrstat &= ~(*val & (UTRSTAT_TXTHRESH | UTRSTAT_RXTHRESH | UTRSTAT_RXTO));
                break;
        }
    } else {
        switch (addr) {
            case UCON:
                *val = ucon;
                break;
            case URXH:
                // Host input reaches the firmware UART only until kdcom claims the channel.
                // Without this the UEFI Shell has no ConIn at all and cannot be typed into.
                if (!pl011_kd_live() && iodev_can_read(IODEV_USB_VUART)) {
                    uint8_t c;
                    iodev_read(IODEV_USB_VUART, &c, 1);
                    *val = c;
                } else {
                    *val = 0;
                }
                break;
            case UTRSTAT:
                *val = utrstat;
                break;
            case UFSTAT:
                //
                // HACK HACK: the below code needs to account for whether we require SAM5250 semantics for the Windows
                // UART driver or if we can get away with using the 8900 UART raw (they're basically compatible but not fully.)
                //
                *val = utrstat & UTRSTAT_TXBE ? ufstat : ufstat | BIT(24);
                break;
            case UERSTAT:
                *val = 0;
                break;
            default:
                *val = 0;
                break;
        }
        //         printf("HV: vuart R 0x%lx -> 0x%lx (%d)\n", addr, *val, width);
    }

    return true;
}

void hv_vuart_poll(void)
{
    if (!active)
        return;

    update_irq();
}

bool hv_map_vuart(u64 base, int irq, iodev_id_t iodev)
{
    int ret = hv_map_hook(base, handle_vuart, 0x2000);
    // A standalone boot may use the physical UART. Only USB iodevs need the
    // companion USB-vUART transport configured by the assisted proxy path.
    if (iodev >= IODEV_USB0 && iodev < IODEV_MAX)
        usb_iodev_vuart_setup(iodev);
    vuart_irq = irq;
    active = true;
    return ret == 0;
}
