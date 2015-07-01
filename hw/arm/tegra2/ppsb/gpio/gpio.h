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

/* Autogenerated from TRM v02p */

#ifndef TEGRA_GPIO_H
#define TEGRA_GPIO_H

#define GPIO_CNF_OFFSET 0x0
#define GPIO_CNF_RESET  0x00000000
typedef union gpio_cnf_u {
    struct {
        unsigned int bit_0:1;               /* Configures each pin to be in either GPIO or SFIO mode; 0 = SPIO; 1 = GPIO */
        unsigned int bit_1:1;               /* Configures each pin to be in either GPIO or SFIO mode; 0 = SPIO; 1 = GPIO */
        unsigned int bit_2:1;               /* Configures each pin to be in either GPIO or SFIO mode; 0 = SPIO; 1 = GPIO */
        unsigned int bit_3:1;               /* Configures each pin to be in either GPIO or SFIO mode; 0 = SPIO; 1 = GPIO */
        unsigned int bit_4:1;               /* Configures each pin to be in either GPIO or SFIO mode; 0 = SPIO; 1 = GPIO */
        unsigned int bit_5:1;               /* Configures each pin to be in either GPIO or SFIO mode; 0 = SPIO; 1 = GPIO */
        unsigned int bit_6:1;               /* Configures each pin to be in either GPIO or SFIO mode; 0 = SPIO; 1 = GPIO */
        unsigned int bit_7:1;               /* Configures each pin to be in either GPIO or SFIO mode; 0 = SPIO; 1 = GPIO */
        unsigned int undefined_bits_8_31:24;
    };

    uint32_t reg32;
} gpio_cnf_t;

#define GPIO_OE_OFFSET 0x10
#define GPIO_OE_RESET  0x00000000
typedef union gpio_oe_u {
    struct {
        unsigned int bit_0:1;               /* GPIO mode (GPIO_CNF.x=1) must be true for this condition to be valid; 0 = TRI_STATE; 1 = DRIVEN */
        unsigned int bit_1:1;               /* GPIO mode (GPIO_CNF.x=1) must be true for this condition to be valid; 0 = TRI_STATE; 1 = DRIVEN */
        unsigned int bit_2:1;               /* GPIO mode (GPIO_CNF.x=1) must be true for this condition to be valid; 0 = TRI_STATE; 1 = DRIVEN */
        unsigned int bit_3:1;               /* GPIO mode (GPIO_CNF.x=1) must be true for this condition to be valid; 0 = TRI_STATE; 1 = DRIVEN */
        unsigned int bit_4:1;               /* GPIO mode (GPIO_CNF.x=1) must be true for this condition to be valid; 0 = TRI_STATE; 1 = DRIVEN */
        unsigned int bit_5:1;               /* GPIO mode (GPIO_CNF.x=1) must be true for this condition to be valid; 0 = TRI_STATE; 1 = DRIVEN */
        unsigned int bit_6:1;               /* GPIO mode (GPIO_CNF.x=1) must be true for this condition to be valid; 0 = TRI_STATE; 1 = DRIVEN */
        unsigned int bit_7:1;               /* GPIO mode (GPIO_CNF.x=1) must be true for this condition to be valid; 0 = TRI_STATE; 1 = DRIVEN */
        unsigned int undefined_bits_8_31:24;
    };

    uint32_t reg32;
} gpio_oe_t;

#define GPIO_OUT_OFFSET 0x20
#define GPIO_OUT_RESET  0x00000000
typedef union gpio_out_u {
    struct {
        unsigned int bit_0:1;               /* GPIO_CNF.x=1 (in GPIO mode) AND GPIO_OE.x=1 (GPIO output enabled) mxst be true for this to be a valid state; 0 = LOW; 1 = HIGH */
        unsigned int bit_1:1;               /* GPIO_CNF.x=1 (in GPIO mode) AND GPIO_OE.x=1 (GPIO output enabled) mxst be true for this to be a valid state; 0 = LOW; 1 = HIGH */
        unsigned int bit_2:1;               /* GPIO_CNF.x=1 (in GPIO mode) AND GPIO_OE.x=1 (GPIO output enabled) mxst be true for this to be a valid state; 0 = LOW; 1 = HIGH */
        unsigned int bit_3:1;               /* GPIO_CNF.x=1 (in GPIO mode) AND GPIO_OE.x=1 (GPIO output enabled) mxst be true for this to be a valid state; 0 = LOW; 1 = HIGH */
        unsigned int bit_4:1;               /* GPIO_CNF.x=1 (in GPIO mode) AND GPIO_OE.x=1 (GPIO output enabled) mxst be true for this to be a valid state; 0 = LOW; 1 = HIGH */
        unsigned int bit_5:1;               /* GPIO_CNF.x=1 (in GPIO mode) AND GPIO_OE.x=1 (GPIO output enabled) mxst be true for this to be a valid state; 0 = LOW; 1 = HIGH */
        unsigned int bit_6:1;               /* GPIO_CNF.x=1 (in GPIO mode) AND GPIO_OE.x=1 (GPIO output enabled) mxst be true for this to be a valid state; 0 = LOW; 1 = HIGH */
        unsigned int bit_7:1;               /* GPIO_CNF.x=1 (in GPIO mode) AND GPIO_OE.x=1 (GPIO output enabled) mxst be true for this to be a valid state; 0 = LOW; 1 = HIGH */
        unsigned int undefined_bits_8_31:24;
    };

    uint32_t reg32;
} gpio_out_t;

#define GPIO_IN_OFFSET 0x30
#define GPIO_IN_RESET  0x00000000
typedef union gpio_in_u {
    struct {
        unsigned int bit_0:1;               /* GPIO mode (GPIO_CNF.x=1) must be true for this condition to be valid; 0 = LOW; 1 = HIGH */
        unsigned int bit_1:1;               /* GPIO mode (GPIO_CNF.x=1) must be true for this condition to be valid; 0 = LOW; 1 = HIGH */
        unsigned int bit_2:1;               /* GPIO mode (GPIO_CNF.x=1) must be true for this condition to be valid; 0 = LOW; 1 = HIGH */
        unsigned int bit_3:1;               /* GPIO mode (GPIO_CNF.x=1) must be true for this condition to be valid; 0 = LOW; 1 = HIGH */
        unsigned int bit_4:1;               /* GPIO mode (GPIO_CNF.x=1) must be true for this condition to be valid; 0 = LOW; 1 = HIGH */
        unsigned int bit_5:1;               /* GPIO mode (GPIO_CNF.x=1) must be true for this condition to be valid; 0 = LOW; 1 = HIGH */
        unsigned int bit_6:1;               /* GPIO mode (GPIO_CNF.x=1) must be true for this condition to be valid; 0 = LOW; 1 = HIGH */
        unsigned int bit_7:1;               /* GPIO mode (GPIO_CNF.x=1) must be true for this condition to be valid; 0 = LOW; 1 = HIGH */
        unsigned int undefined_bits_8_31:24;
    };

    uint32_t reg32;
} gpio_in_t;

#define GPIO_INT_STA_OFFSET 0x40
#define GPIO_INT_STA_RESET  0x00000000
typedef union gpio_int_sta_u {
    struct {
        unsigned int bit_0:1;               /* GPIO mode (GPIO_CNF.x=1) and GPIO_INT.ENB.x=1 must be true for this condition to be valid; 0 = IN_ACTIVE; 1 = ACTIVE */
        unsigned int bit_1:1;               /* GPIO mode (GPIO_CNF.x=1) and GPIO_INT.ENB.x=1 must be true for this condition to be valid; 0 = IN_ACTIVE; 1 = ACTIVE */
        unsigned int bit_2:1;               /* GPIO mode (GPIO_CNF.x=1) and GPIO_INT.ENB.x=1 must be true for this condition to be valid; 0 = IN_ACTIVE; 1 = ACTIVE */
        unsigned int bit_3:1;               /* GPIO mode (GPIO_CNF.x=1) and GPIO_INT.ENB.x=1 must be true for this condition to be valid; 0 = IN_ACTIVE; 1 = ACTIVE */
        unsigned int bit_4:1;               /* GPIO mode (GPIO_CNF.x=1) and GPIO_INT.ENB.x=1 must be true for this condition to be valid; 0 = IN_ACTIVE; 1 = ACTIVE */
        unsigned int bit_5:1;               /* GPIO mode (GPIO_CNF.x=1) and GPIO_INT.ENB.x=1 must be true for this condition to be valid; 0 = IN_ACTIVE; 1 = ACTIVE */
        unsigned int bit_6:1;               /* GPIO mode (GPIO_CNF.x=1) and GPIO_INT.ENB.x=1 must be true for this condition to be valid; 0 = IN_ACTIVE; 1 = ACTIVE */
        unsigned int bit_7:1;               /* GPIO mode (GPIO_CNF.x=1) and GPIO_INT.ENB.x=1 must be true for this condition to be valid; 0 = IN_ACTIVE; 1 = ACTIVE */
        unsigned int undefined_bits_8_31:24;
    };

    uint32_t reg32;
} gpio_int_sta_t;

#define GPIO_INT_ENB_OFFSET 0x50
#define GPIO_INT_ENB_RESET  0x00000000
typedef union gpio_int_enb_u {
    struct {
        unsigned int bit_0:1;               /* GPIO mode (GPIO_CNF.x=1) must be true for this condition to be valid; 0 = DISABLE; 1 = ENABLE */
        unsigned int bit_1:1;               /* GPIO mode (GPIO_CNF.x=1) must be true for this condition to be valid; 0 = DISABLE; 1 = ENABLE */
        unsigned int bit_2:1;               /* GPIO mode (GPIO_CNF.x=1) must be true for this condition to be valid; 0 = DISABLE; 1 = ENABLE */
        unsigned int bit_3:1;               /* GPIO mode (GPIO_CNF.x=1) must be true for this condition to be valid; 0 = DISABLE; 1 = ENABLE */
        unsigned int bit_4:1;               /* GPIO mode (GPIO_CNF.x=1) must be true for this condition to be valid; 0 = DISABLE; 1 = ENABLE */
        unsigned int bit_5:1;               /* GPIO mode (GPIO_CNF.x=1) must be true for this condition to be valid; 0 = DISABLE; 1 = ENABLE */
        unsigned int bit_6:1;               /* GPIO mode (GPIO_CNF.x=1) must be true for this condition to be valid; 0 = DISABLE; 1 = ENABLE */
        unsigned int bit_7:1;               /* GPIO mode (GPIO_CNF.x=1) must be true for this condition to be valid; 0 = DISABLE; 1 = ENABLE */
        unsigned int undefined_bits_8_31:24;
    };

    uint32_t reg32;
} gpio_int_enb_t;

#define GPIO_INT_LVL_OFFSET 0x60
#define GPIO_INT_LVL_RESET  0x00000000
typedef union gpio_int_lvl_u {
    struct {
        unsigned int bit_0:1;               /* Interrupt Activation Level or Edge (HIGH for High level or Rising Edge); 0 = LOW; 1 = HIGH */
        unsigned int bit_1:1;               /* Interrupt Activation Level or Edge (HIGH for High level or Rising Edge); 0 = LOW; 1 = HIGH */
        unsigned int bit_2:1;               /* Interrupt Activation Level or Edge (HIGH for High level or Rising Edge); 0 = LOW; 1 = HIGH */
        unsigned int bit_3:1;               /* Interrupt Activation Level or Edge (HIGH for High level or Rising Edge); 0 = LOW; 1 = HIGH */
        unsigned int bit_4:1;               /* Interrupt Activation Level or Edge (HIGH for High level or Rising Edge); 0 = LOW; 1 = HIGH */
        unsigned int bit_5:1;               /* Interrupt Activation Level or Edge (HIGH for High level or Rising Edge); 0 = LOW; 1 = HIGH */
        unsigned int bit_6:1;               /* Interrupt Activation Level or Edge (HIGH for High level or Rising Edge); 0 = LOW; 1 = HIGH */
        unsigned int bit_7:1;               /* Interrupt Activation Level or Edge (HIGH for High level or Rising Edge); 0 = LOW; 1 = HIGH */
        unsigned int edge_0:1;              /* 1 means Configure as Edge-Triggered Interrupt */
        unsigned int edge_1:1;              /* 1 means Configure as Edge-Triggered Interrupt */
        unsigned int edge_2:1;              /* 1 means Configure as Edge-Triggered Interrupt */
        unsigned int edge_3:1;              /* 1 means Configure as Edge-Triggered Interrupt */
        unsigned int edge_4:1;              /* 1 means Configure as Edge-Triggered Interrupt */
        unsigned int edge_5:1;              /* 1 means Configure as Edge-Triggered Interrupt */
        unsigned int edge_6:1;              /* 1 means Configure as Edge-Triggered Interrupt */
        unsigned int edge_7:1;              /* 1 means Configure as Edge-Triggered Interrupt */
        unsigned int delta_0:1;             /* 1 means Trigger Interrupt on ANY change of input if EDGE is TRUE */
        unsigned int delta_1:1;             /* 1 means Trigger Interrupt on ANY change of input if EDGE is TRUE */
        unsigned int delta_2:1;             /* 1 means Trigger Interrupt on ANY change of input if EDGE is TRUE */
        unsigned int delta_3:1;             /* 1 means Trigger Interrupt on ANY change of input if EDGE is TRUE */
        unsigned int delta_4:1;             /* 1 means Trigger Interrupt on ANY change of input if EDGE is TRUE */
        unsigned int delta_5:1;             /* 1 means Trigger Interrupt on ANY change of input if EDGE is TRUE */
        unsigned int delta_6:1;             /* 1 means Trigger Interrupt on ANY change of input if EDGE is TRUE */
        unsigned int delta_7:1;             /* 1 means Trigger Interrupt on ANY change of input if EDGE is TRUE */
        unsigned int undefined_bits_24_31:8;
    };

    uint32_t reg32;
} gpio_int_lvl_t;

#define GPIO_INT_CLR_OFFSET 0x70
#define GPIO_INT_CLR_RESET  0x00000000
typedef union gpio_int_clr_u {
    struct {
        unsigned int bit_0:1;               /* GPIO mode (GPIO_CNF.x=1) and GPIO_INT.ENB.x=1 must be true for this condition to be valid; 0 = SET; 1 = CLEAR */
        unsigned int bit_1:1;               /* GPIO mode (GPIO_CNF.x=1) and GPIO_INT.ENB.x=1 must be true for this condition to be valid; 0 = SET; 1 = CLEAR */
        unsigned int bit_2:1;               /* GPIO mode (GPIO_CNF.x=1) and GPIO_INT.ENB.x=1 must be true for this condition to be valid; 0 = SET; 1 = CLEAR */
        unsigned int bit_3:1;               /* GPIO mode (GPIO_CNF.x=1) and GPIO_INT.ENB.x=1 must be true for this condition to be valid; 0 = SET; 1 = CLEAR */
        unsigned int bit_4:1;               /* GPIO mode (GPIO_CNF.x=1) and GPIO_INT.ENB.x=1 must be true for this condition to be valid; 0 = SET; 1 = CLEAR */
        unsigned int bit_5:1;               /* GPIO mode (GPIO_CNF.x=1) and GPIO_INT.ENB.x=1 must be true for this condition to be valid; 0 = SET; 1 = CLEAR */
        unsigned int bit_6:1;               /* GPIO mode (GPIO_CNF.x=1) and GPIO_INT.ENB.x=1 must be true for this condition to be valid; 0 = SET; 1 = CLEAR */
        unsigned int bit_7:1;               /* GPIO mode (GPIO_CNF.x=1) and GPIO_INT.ENB.x=1 must be true for this condition to be valid; 0 = SET; 1 = CLEAR */
        unsigned int undefined_bits_8_31:24;
    };

    uint32_t reg32;
} gpio_int_clr_t;

#define GPIO_MSK_CNF_OFFSET 0x800
#define GPIO_MSK_CNF_RESET  0x00000000
typedef union gpio_msk_cnf_u {
    struct {
        unsigned int bit_0:1;               /* 0 = SPIO; 1 = GPIO */
        unsigned int bit_1:1;               /* 0 = SPIO; 1 = GPIO */
        unsigned int bit_2:1;               /* 0 = SPIO; 1 = GPIO */
        unsigned int bit_3:1;               /* 0 = SPIO; 1 = GPIO */
        unsigned int bit_4:1;               /* 0 = SPIO; 1 = GPIO */
        unsigned int bit_5:1;               /* 0 = SPIO; 1 = GPIO */
        unsigned int bit_6:1;               /* 0 = SPIO; 1= GPIO */
        unsigned int bit_7:1;               /* 0 = SPIO; 1 = GPIO */
        unsigned int msk0:1;                /* 0=Disable bit for write; 0 = DISABLE; 1 = ENABLE */
        unsigned int msk1:1;                /* 0=Disable bit for write; 0 = DISABLE; 1 = ENABLE */
        unsigned int msk2:1;                /* 0=Disable bit for write; 0 = DISABLE; 1 = ENABLE */
        unsigned int msk3:1;                /* 0=Disable bit for write; 0 = DISABLE; 1 = ENABLE */
        unsigned int msk4:1;                /* 0=Disable bit for write; 0 = DISABLE; 1 = ENABLE */
        unsigned int msk5:1;                /* 0=Disable bit for write; 0 = DISABLE; 1 = ENABLE */
        unsigned int msk6:1;                /* 0=Disable bit for write; 0 = DISABLE; 1 = ENABLE */
        unsigned int msk7:1;                /* 0=Disable bit for write; 0 = DISABLE; 1 = ENABLE */
        unsigned int undefined_bits_16_31:16;
    };

    uint32_t reg32;
} gpio_msk_cnf_t;

#define GPIO_MSK_OE_OFFSET 0x810
#define GPIO_MSK_OE_RESET  0x00000000
typedef union gpio_msk_oe_u {
    struct {
        unsigned int bit_0:1;               /* 0 = TRI_STATE; 1 = DRIVEN */
        unsigned int bit_1:1;               /* 0 = TRI_STATE; 1 = DRIVEN */
        unsigned int bit_2:1;               /* 0 = TRI_STATE; 1 = DRIVEN */
        unsigned int bit_3:1;               /* 0 = TRI_STATE; 1 = DRIVEN */
        unsigned int bit_4:1;               /* 0 = TRI_STATE; 1 = DRIVEN */
        unsigned int bit_5:1;               /* 0 = TRI_STATE; 1 = DRIVEN */
        unsigned int bit_6:1;               /* 0 = TRI_STATE; 1 = DRIVEN */
        unsigned int bit_7:1;               /* 0 = TRI_STATE; 1 = DRIVEN */
        unsigned int msk0:1;                /* 0=Disable bit for write; 0 = DISABLE; 1 = ENABLE */
        unsigned int msk1:1;                /* 0=Disable bit for write; 0 = DISABLE; 1 = ENABLE */
        unsigned int msk2:1;                /* 0=Disable bit for write; 0 = DISABLE; 1 = ENABLE */
        unsigned int msk3:1;                /* 0=Disable bit for write; 0 = DISABLE; 1 = ENABLE */
        unsigned int msk4:1;                /* 0=Disable bit for write; 0 = DISABLE; 1 = ENABLE */
        unsigned int msk5:1;                /* 0=Disable bit for write; 0 = DISABLE; 1 = ENABLE */
        unsigned int msk6:1;                /* 0=Disable bit for write; 0 = DISABLE; 1 = ENABLE */
        unsigned int msk7:1;                /* 0=Disable bit for write; 0 = DISABLE; 1 = ENABLE */
        unsigned int undefined_bits_16_31:16;
    };

    uint32_t reg32;
} gpio_msk_oe_t;

#define GPIO_MSK_OUT_OFFSET 0x820
#define GPIO_MSK_OUT_RESET  0x00000000
typedef union gpio_msk_out_u {
    struct {
        unsigned int bit_0:1;               /* 0 = LOW; 1 = HIGH */
        unsigned int bit_1:1;               /* 0 = LOW; 1 = HIGH */
        unsigned int bit_2:1;               /* 0 = LOW; 1 = HIGH */
        unsigned int bit_3:1;               /* 0 = LOW; 1 = HIGH */
        unsigned int bit_4:1;               /* 0 = LOW; 1 = HIGH */
        unsigned int bit_5:1;               /* 0 = LOW; 1 = HIGH */
        unsigned int bit_6:1;               /* 0 = LOW; 1 = HIGH */
        unsigned int bit_7:1;               /* 0 = LOW; 1 = HIGH */
        unsigned int msk0:1;                /* 0=Disable bit for write; 0 = DISABLE; 1 = ENABLE */
        unsigned int msk1:1;                /* 0=Disable bit for write; 0 = DISABLE; 1 = ENABLE */
        unsigned int msk2:1;                /* 0=Disable bit for write; 0 = DISABLE; 1 = ENABLE */
        unsigned int msk3:1;                /* 0=Disable bit for write; 0 = DISABLE; 1 = ENABLE */
        unsigned int msk4:1;                /* 0=Disable bit for write; 0 = DISABLE; 1 = ENABLE */
        unsigned int msk5:1;                /* 0=Disable bit for write; 0 = DISABLE; 1 = ENABLE */
        unsigned int msk6:1;                /* 0=Disable bit for write; 0 = DISABLE; 1 = ENABLE */
        unsigned int msk7:1;                /* 0=Disable bit for write; 0 = DISABLE; 1 = ENABLE */
        unsigned int undefined_bits_16_31:16;
    };

    uint32_t reg32;
} gpio_msk_out_t;

#define GPIO_MSK_INT_STA_OFFSET 0x840
#define GPIO_MSK_INT_STA_RESET  0x00000000
typedef union gpio_msk_int_sta_u {
    struct {
        unsigned int bit_0:1;               /* 0 = IN_ACTIVE; 1 = ACTIVE */
        unsigned int bit_1:1;               /* 0 = IN_ACTIVE; 1 = ACTIVE */
        unsigned int bit_2:1;               /* 0 = IN_ACTIVE; 1 = ACTIVE */
        unsigned int bit_3:1;               /* 0 = IN_ACTIVE; 1 = ACTIVE */
        unsigned int bit_4:1;               /* 0 = IN_ACTIVE; 1 = ACTIVE */
        unsigned int bit_5:1;               /* 0 = IN_ACTIVE; 1 = ACTIVE */
        unsigned int bit_6:1;               /* 0 = IN_ACTIVE; 1 = ACTIVE */
        unsigned int bit_7:1;               /* 0 = IN_ACTIVE; 1 = ACTIVE */
        unsigned int msk0:1;                /* 0=Disable bit for write; 0 = DISABLE; 1 = ENABLE */
        unsigned int msk1:1;                /* 0=Disable bit for write; 0 = DISABLE; 1 = ENABLE */
        unsigned int msk2:1;                /* 0=Disable bit for write; 0 = DISABLE; 1 = ENABLE */
        unsigned int msk3:1;                /* 0=Disable bit for write; 0 = DISABLE; 1 = ENABLE */
        unsigned int msk4:1;                /* 0=Disable bit for write; 0 = DISABLE; 1 = ENABLE */
        unsigned int msk5:1;                /* 0=Disable bit for write; 0 = DISABLE; 1 = ENABLE */
        unsigned int msk6:1;                /* 0=Disable bit for write; 0 = DISABLE; 1 = ENABLE */
        unsigned int msk7:1;                /* 0=Disable bit for write; 0 = DISABLE; 1 = ENABLE */
        unsigned int undefined_bits_16_31:16;
    };

    uint32_t reg32;
} gpio_msk_int_sta_t;

#define GPIO_MSK_INT_ENB_OFFSET 0x850
#define GPIO_MSK_INT_ENB_RESET  0x00000000
typedef union gpio_msk_int_enb_u {
    struct {
        unsigned int bit_0:1;               /* 0 = DISABLE; 1 = ENABLE */
        unsigned int bit_1:1;               /* 0 = DISABLE; 1 = ENABLE */
        unsigned int bit_2:1;               /* 0 = DISABLE; 1 = ENABLE */
        unsigned int bit_3:1;               /* 0 = DISABLE; 1 = ENABLE */
        unsigned int bit_4:1;               /* 0 = DISABLE; 1 = ENABLE */
        unsigned int bit_5:1;               /* 0 = DISABLE; 1 = ENABLE */
        unsigned int bit_6:1;               /* 0 = DISABLE; 1 = ENABLE */
        unsigned int bit_7:1;               /* 0 = DISABLE; 1 = ENABLE */
        unsigned int msk0:1;                /* 0=Disable bit for write; 1 = ENABLE */
        unsigned int msk1:1;                /* 0=Disable bit for write; 1 = ENABLE */
        unsigned int msk2:1;                /* 0=Disable bit for write; 1 = ENABLE */
        unsigned int msk3:1;                /* 0=Disable bit for write; 0 = DISABLE; 1 = ENABLE */
        unsigned int msk4:1;                /* 0=Disable bit for write; 0 = DISABLE; 1 = ENABLE */
        unsigned int msk5:1;                /* 0=Disable bit for write; 0 = DISABLE; 1 = ENABLE */
        unsigned int msk6:1;                /* 0=Disable bit for write; 0 = DISABLE; 1 = ENABLE */
        unsigned int msk7:1;                /* 0=Disable bit for write; 0 = DISABLE; 1 = ENABLE */
        unsigned int undefined_bits_16_31:16;
    };

    uint32_t reg32;
} gpio_msk_int_enb_t;

#define GPIO_MSK_INT_LVL_OFFSET 0x860
#define GPIO_MSK_INT_LVL_RESET  0x00000000
typedef union gpio_msk_int_lvl_u {
    struct {
        unsigned int bit_0:1;               /* 0 = LOW; 1 = HIGH */
        unsigned int bit_1:1;               /* 0 = LOW; 1 = HIGH */
        unsigned int bit_2:1;               /* 0 = LOW; 1 = HIGH */
        unsigned int bit_3:1;               /* 0 = LOW; 1 = HIGH */
        unsigned int bit_4:1;               /* 0 = LOW; 1 = HIGH */
        unsigned int bit_5:1;               /* 0 = LOW; 1 = HIGH */
        unsigned int bit_6:1;               /* 0 = LOW; 1 = HIGH */
        unsigned int bit_7:1;               /* 0 = LOW; 1 = HIGH */
        unsigned int msk0:1;                /* 0=Disable bit for write; 1 = ENABLE */
        unsigned int msk1:1;                /* 0=Disable bit for write; 1 = ENABLE */
        unsigned int msk2:1;                /* 0=Disable bit for write; 1 = ENABLE */
        unsigned int msk3:1;                /* 0=Disable bit for write; 1 = ENABLE */
        unsigned int msk4:1;                /* 0=Disable bit for write; 1 = ENABLE */
        unsigned int msk5:1;                /* 0=Disable bit for write; 1 = ENABLE */
        unsigned int msk6:1;                /* 0=Disable bit for write; 1 = ENABLE */
        unsigned int msk7:1;                /* 0=Disable bit for write; 1 = ENABLE */
        unsigned int undefined_bits_16_31:16;
    };

    uint32_t reg32;
} gpio_msk_int_lvl_t;

#endif // TEGRA_GPIO_H
