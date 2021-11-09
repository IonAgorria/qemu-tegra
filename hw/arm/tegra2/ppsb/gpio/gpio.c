/*
 * ARM NVIDIA Tegra2 emulation.
 *
 * Copyright (c) 2014-2015 Dmitry Osipenko <digetx@gmail.com>
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License as published by the
 *  Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 *  for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include "tegra_common.h"

#include "exec/address-spaces.h"
#include "ui/console.h"
#include "hw/sysbus.h"

#include "gpio.h"
#include "iomap.h"
#include "tegra_trace.h"

#define TYPE_TEGRA_GPIO "tegra.gpio"
#define TEGRA_GPIO(obj) OBJECT_CHECK(tegra_gpio, (obj), TYPE_TEGRA_GPIO)
#define DEFINE_REG32(reg) reg##_t reg
#define gpio_in_old_t gpio_in_t

#define BANKS_NB    7
#define PORTS_NB    4

#define PORT_A      0
#define PORT_B      1
#define PORT_C      2
#define PORT_D      3
#define PORT_E      4
#define PORT_F      5
#define PORT_G      6
#define PORT_H      7
#define PORT_I      8
#define PORT_J      9
#define PORT_K      10
#define PORT_L      11
#define PORT_M      12
#define PORT_N      13
#define PORT_O      14
#define PORT_P      15
#define PORT_Q      16
#define PORT_R      17
#define PORT_S      18
#define PORT_T      19
#define PORT_U      20
#define PORT_V      21
#define PORT_W      22
#define PORT_X      23
#define PORT_Y      24
#define PORT_Z      25
#define PORT_AA     26
#define PORT_BB     27

#define PIN_MASK(PIN)           (1 << (PIN))

typedef struct tegra_gpio_port_state {
    DEFINE_REG32(gpio_cnf);
    DEFINE_REG32(gpio_oe);
    DEFINE_REG32(gpio_out);
    DEFINE_REG32(gpio_in);
    DEFINE_REG32(gpio_in_old);
    DEFINE_REG32(gpio_int_sta);
    DEFINE_REG32(gpio_int_enb);
    DEFINE_REG32(gpio_int_lvl);
    DEFINE_REG32(gpio_int_clr);
    DEFINE_REG32(gpio_msk_cnf);
    DEFINE_REG32(gpio_msk_oe);
    DEFINE_REG32(gpio_msk_out);
    DEFINE_REG32(gpio_msk_int_sta);
    DEFINE_REG32(gpio_msk_int_enb);
    DEFINE_REG32(gpio_msk_int_lvl);
} tegra_gpio_port;

static const VMStateDescription vmstate_tegra_gpio_port = {
    .name = "tegra-gpio-port",
    .version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(gpio_cnf.reg32, tegra_gpio_port),
        VMSTATE_UINT32(gpio_oe.reg32, tegra_gpio_port),
        VMSTATE_UINT32(gpio_out.reg32, tegra_gpio_port),
        VMSTATE_UINT32(gpio_in.reg32, tegra_gpio_port),
        VMSTATE_UINT32(gpio_in_old.reg32, tegra_gpio_port),
        VMSTATE_UINT32(gpio_int_sta.reg32, tegra_gpio_port),
        VMSTATE_UINT32(gpio_int_enb.reg32, tegra_gpio_port),
        VMSTATE_UINT32(gpio_int_lvl.reg32, tegra_gpio_port),
        VMSTATE_UINT32(gpio_int_clr.reg32, tegra_gpio_port),
        VMSTATE_UINT32(gpio_msk_cnf.reg32, tegra_gpio_port),
        VMSTATE_UINT32(gpio_msk_oe.reg32, tegra_gpio_port),
        VMSTATE_UINT32(gpio_msk_out.reg32, tegra_gpio_port),
        VMSTATE_UINT32(gpio_msk_int_sta.reg32, tegra_gpio_port),
        VMSTATE_UINT32(gpio_msk_int_enb.reg32, tegra_gpio_port),
        VMSTATE_UINT32(gpio_msk_int_lvl.reg32, tegra_gpio_port),
        VMSTATE_END_OF_LIST()
    }
};

typedef struct tegra_gpio_state {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    tegra_gpio_port regs[BANKS_NB * PORTS_NB];
    bool irq_raised[BANKS_NB];
    qemu_irq irq[BANKS_NB];
} tegra_gpio;

static const VMStateDescription vmstate_tegra_gpio = {
    .name = "tegra.gpio",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_STRUCT_ARRAY(regs, tegra_gpio, BANKS_NB * PORTS_NB, 0,
                             vmstate_tegra_gpio_port, tegra_gpio_port),
        VMSTATE_END_OF_LIST()
    }
};

static void tegra_gpio_update_irq(tegra_gpio *s)
{
    uint32_t mask, lvl, edge, both, enb;
    bool raised[BANKS_NB] = {};
    unsigned int port, bank;
    tegra_gpio_port *p;

    for (port = 0; port < ARRAY_SIZE(s->regs); port++) {
        p = &s->regs[port];
        bank = port / PORTS_NB;
        enb = p->gpio_int_enb.reg32 & p->gpio_cnf.reg32 & 0xff;
        lvl = (p->gpio_int_lvl.reg32 >> 0) & 0xff;
        edge = (p->gpio_int_lvl.reg32 >> 8) & 0xff;
        both = (p->gpio_int_lvl.reg32 >> 16) & 0xff;

        mask  = ~edge &  lvl &   p->gpio_in.reg32;
        mask |= ~edge & ~lvl &  ~p->gpio_in.reg32;
        mask |=  edge &  lvl & (~p->gpio_in_old.reg32 &  p->gpio_in.reg32);
        mask |=  edge & ~lvl &  (p->gpio_in_old.reg32 & ~p->gpio_in.reg32);
        mask |=  edge & both &  (p->gpio_in_old.reg32 ^  p->gpio_in.reg32);
        mask &= enb;

        if (mask) {
            p->gpio_int_sta.reg32 |= mask;

            if (!s->irq_raised[bank]) {
                TRACE_IRQ_RAISE(s->iomem.addr, s->irq[bank]);
                s->irq_raised[bank] = true;
                raised[bank] = true;
            }
        }

        if (!raised[bank] && s->irq_raised[bank]) {
            TRACE_IRQ_LOWER(s->iomem.addr, s->irq[bank]);
            s->irq_raised[bank] = false;
        }
    }
}

static void tegra_gpio_set_active(tegra_gpio *s, unsigned int port,
                                  unsigned int pin, bool active_low)
{
    tegra_gpio_port *p = &s->regs[port];

    if (active_low)
        p->gpio_in.reg32 &= ~PIN_MASK(pin);
    else
        p->gpio_in.reg32 |= PIN_MASK(pin);

    tegra_gpio_update_irq(s);

    p->gpio_in_old.reg32 = p->gpio_in.reg32;
}

static void tegra_gpio_set_inactive(tegra_gpio *s, unsigned int port,
                                    unsigned int pin, bool active_low)
{
    tegra_gpio_port *p = &s->regs[port];

    if (active_low)
        p->gpio_in.reg32 |= PIN_MASK(pin);
    else
        p->gpio_in.reg32 &= ~PIN_MASK(pin);

    tegra_gpio_update_irq(s);

    p->gpio_in_old.reg32 = p->gpio_in.reg32;
}

static void tegra_gpio_button_press(tegra_gpio *s,  unsigned int port,
                                    unsigned int pin, bool active_low)
{
    tegra_gpio_set_active(s, port, pin, active_low);
}

static void tegra_gpio_button_unpress(tegra_gpio *s, unsigned int port,
                                      unsigned int pin, bool active_low)
{
    tegra_gpio_set_inactive(s, port, pin, active_low);
}

static void tegra_gpio_button_set(tegra_gpio *s, unsigned int port,
                                  unsigned int pin, bool active_low,
                                  bool press)
{
    if (press)
        tegra_gpio_button_press(s, port, pin, active_low);
    else
        tegra_gpio_button_unpress(s, port, pin, active_low);
}

static uint64_t tegra_gpio_priv_read(void *opaque, hwaddr offset,
                                     unsigned size)
{
    tegra_gpio *s = opaque;
    tegra_gpio_port *p;
    int bank = (offset >> 7) & 0x7;
    int port = (offset & 0xf) >> 2;
    int port_nb = bank * PORTS_NB + port;
    uint64_t ret = 0;

    p = &s->regs[port_nb];

    switch (offset & 0x870) {
    case GPIO_CNF_OFFSET:
        ret = p->gpio_cnf.reg32;
        break;
    case GPIO_OE_OFFSET:
        ret = p->gpio_oe.reg32;
        break;
    case GPIO_OUT_OFFSET:
        ret = p->gpio_out.reg32;
        break;
    case GPIO_IN_OFFSET:
        ret = p->gpio_in.reg32;
        break;
    case GPIO_INT_STA_OFFSET:
        ret = p->gpio_int_sta.reg32;
        break;
    case GPIO_INT_ENB_OFFSET:
        ret = p->gpio_int_enb.reg32;
        break;
    case GPIO_INT_LVL_OFFSET:
        ret = p->gpio_int_lvl.reg32;
        break;
    case GPIO_INT_CLR_OFFSET:
        ret = p->gpio_int_clr.reg32;
        break;
    case GPIO_MSK_CNF_OFFSET:
        ret = p->gpio_msk_cnf.reg32;
        break;
    case GPIO_MSK_OE_OFFSET:
        ret = p->gpio_msk_oe.reg32;
        break;
    case GPIO_MSK_OUT_OFFSET:
        ret = p->gpio_msk_out.reg32;
        break;
    case GPIO_MSK_INT_STA_OFFSET:
        ret = p->gpio_msk_int_sta.reg32;
        break;
    case GPIO_MSK_INT_ENB_OFFSET:
        ret = p->gpio_msk_int_enb.reg32;
        break;
    case GPIO_MSK_INT_LVL_OFFSET:
        ret = p->gpio_msk_int_lvl.reg32;
        break;
    default:
        break;
    }

    TRACE_READ(s->iomem.addr, offset, ret);

    return ret;
}

static void tegra_gpio_priv_write(void *opaque, hwaddr offset,
                                  uint64_t value, unsigned size)
{
    tegra_gpio *s = opaque;
    tegra_gpio_port *p;
    int bank = (offset >> 7) & 0x7;
    int port = (offset & 0xf) >> 2;
    uint32_t reg;

    p = &s->regs[bank * PORTS_NB + port];

    switch (offset & 0x870) {
    case GPIO_CNF_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, p->gpio_cnf.reg32, value);
        p->gpio_cnf.reg32 = value;
        break;
    case GPIO_OE_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, p->gpio_oe.reg32, value);
        p->gpio_oe.reg32 = value;
        break;
    case GPIO_OUT_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, p->gpio_out.reg32, value);
        p->gpio_out.reg32 = value;
        break;
    case GPIO_INT_STA_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, p->gpio_int_sta.reg32, value);
        p->gpio_int_sta.reg32 = value;
        break;
    case GPIO_INT_ENB_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, p->gpio_int_enb.reg32, value);
        p->gpio_int_enb.reg32 = value;
        tegra_gpio_update_irq(s);
        break;
    case GPIO_INT_LVL_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, p->gpio_int_lvl.reg32, value);
        p->gpio_int_lvl.reg32 = value;
        break;
    case GPIO_INT_CLR_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, p->gpio_int_clr.reg32, value);
        p->gpio_int_clr.reg32 = value;

        p->gpio_int_sta.reg32 &= ~value;
        tegra_gpio_update_irq(s);
        break;
    case GPIO_MSK_CNF_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, p->gpio_msk_cnf.reg32, value);
        p->gpio_msk_cnf.reg32 = value;

        reg = p->gpio_cnf.reg32;
        p->gpio_cnf.reg32  = value & (value >> 8) & 0xff;
        p->gpio_cnf.reg32 |= reg & ~(value >> 8) & 0xff;

        TRACE_WRITE(s->iomem.addr, offset - (GPIO_MSK_CNF_OFFSET - GPIO_CNF_OFFSET),
                    reg, p->gpio_cnf.reg32);

        tegra_gpio_update_irq(s);
        break;
    case GPIO_MSK_OE_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, p->gpio_msk_oe.reg32, value);
        p->gpio_msk_oe.reg32 = value;
        break;
    case GPIO_MSK_OUT_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, p->gpio_msk_out.reg32, value);
        p->gpio_msk_out.reg32 |= value;
        break;
    case GPIO_MSK_INT_STA_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, p->gpio_msk_int_sta.reg32, value);
        p->gpio_msk_int_sta.reg32 = value;

        p->gpio_int_sta.reg32 &= ~(value & (value >> 8)) & 0xff;

        tegra_gpio_update_irq(s);
        break;
    case GPIO_MSK_INT_ENB_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, p->gpio_msk_int_enb.reg32, value);
        p->gpio_msk_int_enb.reg32 |= value;

        reg = p->gpio_int_enb.reg32;
        p->gpio_int_enb.reg32  = value & (value >> 8) & 0xff;
        p->gpio_int_enb.reg32 |= reg & ~(value >> 8) & 0xff;

        tegra_gpio_update_irq(s);
        break;
    case GPIO_MSK_INT_LVL_OFFSET:
        TRACE_WRITE(s->iomem.addr, offset, p->gpio_msk_int_lvl.reg32, value);
        p->gpio_msk_int_lvl.reg32 |= value;

        p->gpio_int_lvl.reg32  = p->gpio_int_sta.reg32 & ~((value >> 8)) & 0xff;
        p->gpio_int_lvl.reg32 |= value & (value >> 8) & 0xff;

        tegra_gpio_update_irq(s);
        break;
    default:
        TRACE_WRITE(s->iomem.addr, offset, 0, value);
        break;
    }
}

static void tegra_gpio_priv_reset(DeviceState *dev)
{
    tegra_gpio *s = TEGRA_GPIO(dev);
    int i;

    for (i = 0; i < BANKS_NB * PORTS_NB; i++) {
        tegra_gpio_port *p = &s->regs[i];

        p->gpio_cnf.reg32 = GPIO_CNF_RESET;
        p->gpio_oe.reg32 = GPIO_OE_RESET;
        p->gpio_out.reg32 = GPIO_OUT_RESET;
        p->gpio_in.reg32 = GPIO_IN_RESET;
        p->gpio_in_old.reg32 = GPIO_IN_RESET;
        p->gpio_int_sta.reg32 = GPIO_INT_STA_RESET;
        p->gpio_int_enb.reg32 = GPIO_INT_ENB_RESET;
        p->gpio_int_lvl.reg32 = GPIO_INT_LVL_RESET;
        p->gpio_int_clr.reg32 = GPIO_INT_CLR_RESET;
        p->gpio_msk_cnf.reg32 = GPIO_MSK_CNF_RESET;
        p->gpio_msk_oe.reg32 = GPIO_MSK_OE_RESET;
        p->gpio_msk_out.reg32 = GPIO_MSK_OUT_RESET;
        p->gpio_msk_int_sta.reg32 = GPIO_MSK_INT_STA_RESET;
        p->gpio_msk_int_enb.reg32 = GPIO_MSK_INT_ENB_RESET;
        p->gpio_msk_int_lvl.reg32 = GPIO_MSK_INT_LVL_RESET;
    }

    if (tegra_board == TEGRA2_BOARD_PICASSO ||
        tegra_board == TEGRA2_BOARD_QEMU) {

        tegra_gpio_button_unpress(s, PORT_I, 3, false);
        tegra_gpio_button_unpress(s, PORT_Q, 4, true);
        tegra_gpio_button_unpress(s, PORT_Q, 5, true);
    }
}

static const MemoryRegionOps tegra_gpio_mem_ops = {
    .read = tegra_gpio_priv_read,
    .write = tegra_gpio_priv_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

#define KEY_RELEASED    0x80
#define KEY_CODE        0x7f
#define KEYCODE_UP      0x48
#define KEYCODE_DOWN    0x50
#define KEYCODE_ENTER   0x1c

static void tegra_gpio_key_event(void *opaque, int keycode)
{
    tegra_gpio *s = opaque;

    switch (keycode & KEY_CODE) {
    case KEYCODE_UP:
        if (tegra_board == TEGRA2_BOARD_PICASSO ||
            tegra_board == TEGRA2_BOARD_QEMU)
            tegra_gpio_button_set(s, PORT_Q, 4, true,
                                  !(keycode & KEY_RELEASED));
        break;

    case KEYCODE_DOWN:
        if (tegra_board == TEGRA2_BOARD_PICASSO ||
            tegra_board == TEGRA2_BOARD_QEMU)
            tegra_gpio_button_set(s, PORT_Q, 5, true,
                                  !(keycode & KEY_RELEASED));
        break;

    case KEYCODE_ENTER:
        if (tegra_board == TEGRA2_BOARD_PICASSO ||
            tegra_board == TEGRA2_BOARD_QEMU)
            tegra_gpio_button_set(s, PORT_I, 3, false,
                                  !(keycode & KEY_RELEASED));
        break;
    }
}

static void tegra_gpio_priv_realize(DeviceState *dev, Error **errp)
{
    tegra_gpio *s = TEGRA_GPIO(dev);
    int i;

    memory_region_init_io(&s->iomem, OBJECT(dev), &tegra_gpio_mem_ops, s,
                          "tegra.gpio", TEGRA_GPIO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);

    for (i = 0; i < BANKS_NB; i++)
        sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq[i]);

    qemu_add_kbd_event_handler(tegra_gpio_key_event, s);
}

static void tegra_gpio_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = tegra_gpio_priv_realize;
    dc->vmsd = &vmstate_tegra_gpio;
    dc->reset = tegra_gpio_priv_reset;
}

static const TypeInfo tegra_gpio_info = {
    .name = TYPE_TEGRA_GPIO,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(tegra_gpio),
    .class_init = tegra_gpio_class_init,
};

static void tegra_gpio_register_types(void)
{
    type_register_static(&tegra_gpio_info);
}

type_init(tegra_gpio_register_types)
