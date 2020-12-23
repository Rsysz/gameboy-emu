#include "gameboy.h"

/* Tick the internal RTC by one second */
void gb_tick_rtc(struct gb_s *gb)
{
    /* is timer running? */
    if ((gb->cart_rtc[4] & 0x40) == 0) {
        if (++gb->rtc_bits.sec == 60) {
            gb->rtc_bits.sec = 0;

            if (++gb->rtc_bits.min == 60) {
                gb->rtc_bits.min = 0;

                if (++gb->rtc_bits.hour == 24) {
                    gb->rtc_bits.hour = 0;

                    if (++gb->rtc_bits.yday == 0) {
                        if (gb->rtc_bits.high & 1)     /* Bit 8 of days */
                            gb->rtc_bits.high |= 0x80; /* Overflow bit */

                        gb->rtc_bits.high ^= 1;
                    }
                }
            }
        }
    }
}

/* Set initial values in RTC.
 * Should be called after gb_init().
 */
void gb_set_rtc(struct gb_s *gb, const struct tm *const time)
{
    gb->cart_rtc[0] = time->tm_sec;
    gb->cart_rtc[1] = time->tm_min;
    gb->cart_rtc[2] = time->tm_hour;
    gb->cart_rtc[3] = time->tm_yday & 0xFF; /* Low 8 bits of day counter */
    gb->cart_rtc[4] = time->tm_yday >> 8;   /* High 1 bit of day counter */
}

/* Internal function used to step the CPU */
void __gb_step_cpu(struct gb_s *gb)
{
    uint8_t opcode, inst_cycles;
    static const uint8_t op_cycles[0x100] = {
        /* clang-format off */
        /*          0   1   2   3   4   5   6   7   8   9   A   B   C   D   E   F */
        /* 0x00 */  4, 12,  8,  8,  4,  4,  8,  4, 20,  8,  8,  8,  4,  4,  8,  4,
        /* 0x10 */  4, 12,  8,  8,  4,  4,  8,  4, 12,  8,  8,  8,  4,  4,  8,  4,
        /* 0x20 */  8, 12,  8,  8,  4,  4,  8,  4,  8,  8,  8,  8,  4,  4,  8,  4,
        /* 0x30 */  8, 12,  8,  8, 12, 12, 12,  4,  8,  8,  8,  8,  4,  4,  8,  4,
        /* 0x40 */  4,  4,  4,  4,  4,  4,  8,  4,  4,  4,  4,  4,  4,  4,  8,  4,
        /* 0x50 */  4,  4,  4,  4,  4,  4,  8,  4,  4,  4,  4,  4,  4,  4,  8,  4,
        /* 0x60 */  4,  4,  4,  4,  4,  4,  8,  4,  4,  4,  4,  4,  4,  4,  8,  4,
        /* 0x70 */  8,  8,  8,  8,  8,  8,  4,  8,  4,  4,  4,  4,  4,  4,  8,  4,
        /* 0x80 */  4,  4,  4,  4,  4,  4,  8,  4,  4,  4,  4,  4,  4,  4,  8,  4,
        /* 0x90 */  4,  4,  4,  4,  4,  4,  8,  4,  4,  4,  4,  4,  4,  4,  8,  4,
        /* 0xA0 */  4,  4,  4,  4,  4,  4,  8,  4,  4,  4,  4,  4,  4,  4,  8,  4,
        /* 0xB0 */  4,  4,  4,  4,  4,  4,  8,  4,  4,  4,  4,  4,  4,  4,  8,  4,
        /* 0xC0 */  8, 12, 12, 16, 12, 16,  8, 16,  8, 16, 12,  8, 12, 24,  8, 16,
        /* 0xD0 */  8, 12, 12,  0, 12, 16,  8, 16,  8, 16, 12,  0, 12,  0,  8, 16,
        /* 0xE0 */ 12, 12,  8,  0,  0, 16,  8, 16, 16,  4, 16,  0,  0,  0,  8, 16,
        /* 0xF0 */ 12, 12,  8,  4,  0, 16,  8, 16, 12,  8, 16,  4,  0,  0,  8, 16,
        /* clang-format on */
    };

    /* Handle interrupts */
    if ((gb->gb_ime || gb->gb_halt) &&
        (gb->gb_reg.IF & gb->gb_reg.IE & ANY_INTR)) {
        gb->gb_halt = 0;

        if (gb->gb_ime) {
            /* Disable interrupts */
            gb->gb_ime = 0;

            /* Push program counter */
            __gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc >> 8);
            __gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc & 0xFF);

            /* Call interrupt handler if required. */
            if (gb->gb_reg.IF & gb->gb_reg.IE & VBLANK_INTR) {
                gb->cpu_reg.pc = VBLANK_INTR_ADDR;
                gb->gb_reg.IF ^= VBLANK_INTR;
            } else if (gb->gb_reg.IF & gb->gb_reg.IE & LCDC_INTR) {
                gb->cpu_reg.pc = LCDC_INTR_ADDR;
                gb->gb_reg.IF ^= LCDC_INTR;
            } else if (gb->gb_reg.IF & gb->gb_reg.IE & TIMER_INTR) {
                gb->cpu_reg.pc = TIMER_INTR_ADDR;
                gb->gb_reg.IF ^= TIMER_INTR;
            } else if (gb->gb_reg.IF & gb->gb_reg.IE & SERIAL_INTR) {
                gb->cpu_reg.pc = SERIAL_INTR_ADDR;
                gb->gb_reg.IF ^= SERIAL_INTR;
            } else if (gb->gb_reg.IF & gb->gb_reg.IE & CONTROL_INTR) {
                gb->cpu_reg.pc = CONTROL_INTR_ADDR;
                gb->gb_reg.IF ^= CONTROL_INTR;
            }
        }
    }

    /* Obtain opcode */
    opcode = (gb->gb_halt ? 0x00 : __gb_read(gb, gb->cpu_reg.pc++));
    inst_cycles = op_cycles[opcode];

    /* Execute opcode */
    switch (opcode) {
    case 0x00: /* NOP */
        break;

    case 0x01: /* LD BC, imm */
        gb->cpu_reg.c = __gb_read(gb, gb->cpu_reg.pc++);
        gb->cpu_reg.b = __gb_read(gb, gb->cpu_reg.pc++);
        break;

    case 0x02: /* LD (BC), A */
        __gb_write(gb, gb->cpu_reg.bc, gb->cpu_reg.a);
        break;

    case 0x03: /* INC BC */
        gb->cpu_reg.bc++;
        break;

    case 0x04: /* INC B */
        gb->cpu_reg.b++;
        gb->cpu_reg.f_bits.z = (gb->cpu_reg.b == 0x00);
        gb->cpu_reg.f_bits.n = 0;
        gb->cpu_reg.f_bits.h = ((gb->cpu_reg.b & 0x0F) == 0x00);
        break;

    case 0x05: /* DEC B */
        gb->cpu_reg.b--;
        gb->cpu_reg.f_bits.z = (gb->cpu_reg.b == 0x00);
        gb->cpu_reg.f_bits.n = 1;
        gb->cpu_reg.f_bits.h = ((gb->cpu_reg.b & 0x0F) == 0x0F);
        break;

    case 0x06: /* LD B, imm */
        gb->cpu_reg.b = __gb_read(gb, gb->cpu_reg.pc++);
        break;

    case 0x07: /* RLCA */
        gb->cpu_reg.a = (gb->cpu_reg.a << 1) | (gb->cpu_reg.a >> 7);
        gb->cpu_reg.f_bits.z = 0;
        gb->cpu_reg.f_bits.n = 0;
        gb->cpu_reg.f_bits.h = 0;
        gb->cpu_reg.f_bits.c = (gb->cpu_reg.a & 0x01);
        break;

    case 0x08: { /* LD (imm), SP */
        uint16_t temp = __gb_read(gb, gb->cpu_reg.pc++);
        temp |= __gb_read(gb, gb->cpu_reg.pc++) << 8;
        __gb_write(gb, temp++, gb->cpu_reg.sp & 0xFF);
        __gb_write(gb, temp, gb->cpu_reg.sp >> 8);
        break;
    }

    case 0x09: { /* ADD HL, BC */
        uint_fast32_t temp = gb->cpu_reg.hl + gb->cpu_reg.bc;
        gb->cpu_reg.f_bits.n = 0;
        gb->cpu_reg.f_bits.h =
            (temp ^ gb->cpu_reg.hl ^ gb->cpu_reg.bc) & 0x1000 ? 1 : 0;
        gb->cpu_reg.f_bits.c = (temp & 0xFFFF0000) ? 1 : 0;
        gb->cpu_reg.hl = (temp & 0x0000FFFF);
        break;
    }

    case 0x0A: /* LD A, (BC) */
        gb->cpu_reg.a = __gb_read(gb, gb->cpu_reg.bc);
        break;

    case 0x0B: /* DEC BC */
        gb->cpu_reg.bc--;
        break;

    case 0x0C: /* INC C */
        gb->cpu_reg.c++;
        gb->cpu_reg.f_bits.z = (gb->cpu_reg.c == 0x00);
        gb->cpu_reg.f_bits.n = 0;
        gb->cpu_reg.f_bits.h = ((gb->cpu_reg.c & 0x0F) == 0x00);
        break;

    case 0x0D: /* DEC C */
        gb->cpu_reg.c--;
        gb->cpu_reg.f_bits.z = (gb->cpu_reg.c == 0x00);
        gb->cpu_reg.f_bits.n = 1;
        gb->cpu_reg.f_bits.h = ((gb->cpu_reg.c & 0x0F) == 0x0F);
        break;

    case 0x0E: /* LD C, imm */
        gb->cpu_reg.c = __gb_read(gb, gb->cpu_reg.pc++);
        break;

    case 0x0F: /* RRCA */
        gb->cpu_reg.f_bits.c = gb->cpu_reg.a & 0x01;
        gb->cpu_reg.a = (gb->cpu_reg.a >> 1) | (gb->cpu_reg.a << 7);
        gb->cpu_reg.f_bits.z = 0;
        gb->cpu_reg.f_bits.n = 0;
        gb->cpu_reg.f_bits.h = 0;
        break;

    case 0x10: /* STOP */
        // gb->gb_halt = 1;
        break;

    case 0x11: /* LD DE, imm */
        gb->cpu_reg.e = __gb_read(gb, gb->cpu_reg.pc++);
        gb->cpu_reg.d = __gb_read(gb, gb->cpu_reg.pc++);
        break;

    case 0x12: /* LD (DE), A */
        __gb_write(gb, gb->cpu_reg.de, gb->cpu_reg.a);
        break;

    case 0x13: /* INC DE */
        gb->cpu_reg.de++;
        break;

    case 0x14: /* INC D */
        gb->cpu_reg.d++;
        gb->cpu_reg.f_bits.z = (gb->cpu_reg.d == 0x00);
        gb->cpu_reg.f_bits.n = 0;
        gb->cpu_reg.f_bits.h = ((gb->cpu_reg.d & 0x0F) == 0x00);
        break;

    case 0x15: /* DEC D */
        gb->cpu_reg.d--;
        gb->cpu_reg.f_bits.z = (gb->cpu_reg.d == 0x00);
        gb->cpu_reg.f_bits.n = 1;
        gb->cpu_reg.f_bits.h = ((gb->cpu_reg.d & 0x0F) == 0x0F);
        break;

    case 0x16: /* LD D, imm */
        gb->cpu_reg.d = __gb_read(gb, gb->cpu_reg.pc++);
        break;

    case 0x17: { /* RLA */
        uint8_t temp = gb->cpu_reg.a;
        gb->cpu_reg.a = (gb->cpu_reg.a << 1) | gb->cpu_reg.f_bits.c;
        gb->cpu_reg.f_bits.z = 0;
        gb->cpu_reg.f_bits.n = 0;
        gb->cpu_reg.f_bits.h = 0;
        gb->cpu_reg.f_bits.c = (temp >> 7) & 0x01;
        break;
    }

    case 0x18: { /* JR imm */
        int8_t temp = (int8_t) __gb_read(gb, gb->cpu_reg.pc++);
        gb->cpu_reg.pc += temp;
        break;
    }

    case 0x19: { /* ADD HL, DE */
        uint_fast32_t temp = gb->cpu_reg.hl + gb->cpu_reg.de;
        gb->cpu_reg.f_bits.n = 0;
        gb->cpu_reg.f_bits.h =
            (temp ^ gb->cpu_reg.hl ^ gb->cpu_reg.de) & 0x1000 ? 1 : 0;
        gb->cpu_reg.f_bits.c = (temp & 0xFFFF0000) ? 1 : 0;
        gb->cpu_reg.hl = (temp & 0x0000FFFF);
        break;
    }

    case 0x1A: /* LD A, (DE) */
        gb->cpu_reg.a = __gb_read(gb, gb->cpu_reg.de);
        break;

    case 0x1B: /* DEC DE */
        gb->cpu_reg.de--;
        break;

    case 0x1C: /* INC E */
        gb->cpu_reg.e++;
        gb->cpu_reg.f_bits.z = (gb->cpu_reg.e == 0x00);
        gb->cpu_reg.f_bits.n = 0;
        gb->cpu_reg.f_bits.h = ((gb->cpu_reg.e & 0x0F) == 0x00);
        break;

    case 0x1D: /* DEC E */
        gb->cpu_reg.e--;
        gb->cpu_reg.f_bits.z = (gb->cpu_reg.e == 0x00);
        gb->cpu_reg.f_bits.n = 1;
        gb->cpu_reg.f_bits.h = ((gb->cpu_reg.e & 0x0F) == 0x0F);
        break;

    case 0x1E: /* LD E, imm */
        gb->cpu_reg.e = __gb_read(gb, gb->cpu_reg.pc++);
        break;

    case 0x1F: { /* RRA */
        uint8_t temp = gb->cpu_reg.a;
        gb->cpu_reg.a = gb->cpu_reg.a >> 1 | (gb->cpu_reg.f_bits.c << 7);
        gb->cpu_reg.f_bits.z = 0;
        gb->cpu_reg.f_bits.n = 0;
        gb->cpu_reg.f_bits.h = 0;
        gb->cpu_reg.f_bits.c = temp & 0x1;
        break;
    }

    case 0x20: /* JP NZ, imm */
        if (!gb->cpu_reg.f_bits.z) {
            int8_t temp = (int8_t) __gb_read(gb, gb->cpu_reg.pc++);
            gb->cpu_reg.pc += temp;
            inst_cycles += 4;
        } else
            gb->cpu_reg.pc++;

        break;

    case 0x21: /* LD HL, imm */
        gb->cpu_reg.l = __gb_read(gb, gb->cpu_reg.pc++);
        gb->cpu_reg.h = __gb_read(gb, gb->cpu_reg.pc++);
        break;

    case 0x22: /* LDI (HL), A */
        __gb_write(gb, gb->cpu_reg.hl, gb->cpu_reg.a);
        gb->cpu_reg.hl++;
        break;

    case 0x23: /* INC HL */
        gb->cpu_reg.hl++;
        break;

    case 0x24: /* INC H */
        gb->cpu_reg.h++;
        gb->cpu_reg.f_bits.z = (gb->cpu_reg.h == 0x00);
        gb->cpu_reg.f_bits.n = 0;
        gb->cpu_reg.f_bits.h = ((gb->cpu_reg.h & 0x0F) == 0x00);
        break;

    case 0x25: /* DEC H */
        gb->cpu_reg.h--;
        gb->cpu_reg.f_bits.z = (gb->cpu_reg.h == 0x00);
        gb->cpu_reg.f_bits.n = 1;
        gb->cpu_reg.f_bits.h = ((gb->cpu_reg.h & 0x0F) == 0x0F);
        break;

    case 0x26: /* LD H, imm */
        gb->cpu_reg.h = __gb_read(gb, gb->cpu_reg.pc++);
        break;

    case 0x27: { /* DAA */
        uint16_t a = gb->cpu_reg.a;

        if (gb->cpu_reg.f_bits.n) {
            if (gb->cpu_reg.f_bits.h)
                a = (a - 0x06) & 0xFF;

            if (gb->cpu_reg.f_bits.c)
                a -= 0x60;
        } else {
            if (gb->cpu_reg.f_bits.h || (a & 0x0F) > 9)
                a += 0x06;

            if (gb->cpu_reg.f_bits.c || a > 0x9F)
                a += 0x60;
        }

        if ((a & 0x100) == 0x100)
            gb->cpu_reg.f_bits.c = 1;

        gb->cpu_reg.a = a;
        gb->cpu_reg.f_bits.z = (gb->cpu_reg.a == 0);
        gb->cpu_reg.f_bits.h = 0;

        break;
    }

    case 0x28: /* JP Z, imm */
        if (gb->cpu_reg.f_bits.z) {
            int8_t temp = (int8_t) __gb_read(gb, gb->cpu_reg.pc++);
            gb->cpu_reg.pc += temp;
            inst_cycles += 4;
        } else
            gb->cpu_reg.pc++;

        break;

    case 0x29: { /* ADD HL, HL */
        uint_fast32_t temp = gb->cpu_reg.hl + gb->cpu_reg.hl;
        gb->cpu_reg.f_bits.n = 0;
        gb->cpu_reg.f_bits.h = (temp & 0x1000) ? 1 : 0;
        gb->cpu_reg.f_bits.c = (temp & 0xFFFF0000) ? 1 : 0;
        gb->cpu_reg.hl = (temp & 0x0000FFFF);
        break;
    }

    case 0x2A: /* LD A, (HL+) */
        gb->cpu_reg.a = __gb_read(gb, gb->cpu_reg.hl++);
        break;

    case 0x2B: /* DEC HL */
        gb->cpu_reg.hl--;
        break;

    case 0x2C: /* INC L */
        gb->cpu_reg.l++;
        gb->cpu_reg.f_bits.z = (gb->cpu_reg.l == 0x00);
        gb->cpu_reg.f_bits.n = 0;
        gb->cpu_reg.f_bits.h = ((gb->cpu_reg.l & 0x0F) == 0x00);
        break;

    case 0x2D: /* DEC L */
        gb->cpu_reg.l--;
        gb->cpu_reg.f_bits.z = (gb->cpu_reg.l == 0x00);
        gb->cpu_reg.f_bits.n = 1;
        gb->cpu_reg.f_bits.h = ((gb->cpu_reg.l & 0x0F) == 0x0F);
        break;

    case 0x2E: /* LD L, imm */
        gb->cpu_reg.l = __gb_read(gb, gb->cpu_reg.pc++);
        break;

    case 0x2F: /* CPL */
        gb->cpu_reg.a = ~gb->cpu_reg.a;
        gb->cpu_reg.f_bits.n = 1;
        gb->cpu_reg.f_bits.h = 1;
        break;

    case 0x30: /* JP NC, imm */
        if (!gb->cpu_reg.f_bits.c) {
            int8_t temp = (int8_t) __gb_read(gb, gb->cpu_reg.pc++);
            gb->cpu_reg.pc += temp;
            inst_cycles += 4;
        } else
            gb->cpu_reg.pc++;

        break;

    case 0x31: /* LD SP, imm */
        gb->cpu_reg.sp = __gb_read(gb, gb->cpu_reg.pc++);
        gb->cpu_reg.sp |= __gb_read(gb, gb->cpu_reg.pc++) << 8;
        break;

    case 0x32: /* LD (HL), A */
        __gb_write(gb, gb->cpu_reg.hl, gb->cpu_reg.a);
        gb->cpu_reg.hl--;
        break;

    case 0x33: /* INC SP */
        gb->cpu_reg.sp++;
        break;

    case 0x34: { /* INC (HL) */
        uint8_t temp = __gb_read(gb, gb->cpu_reg.hl) + 1;
        gb->cpu_reg.f_bits.z = (temp == 0x00);
        gb->cpu_reg.f_bits.n = 0;
        gb->cpu_reg.f_bits.h = ((temp & 0x0F) == 0x00);
        __gb_write(gb, gb->cpu_reg.hl, temp);
        break;
    }

    case 0x35: { /* DEC (HL) */
        uint8_t temp = __gb_read(gb, gb->cpu_reg.hl) - 1;
        gb->cpu_reg.f_bits.z = (temp == 0x00);
        gb->cpu_reg.f_bits.n = 1;
        gb->cpu_reg.f_bits.h = ((temp & 0x0F) == 0x0F);
        __gb_write(gb, gb->cpu_reg.hl, temp);
        break;
    }

    case 0x36: /* LD (HL), imm */
        __gb_write(gb, gb->cpu_reg.hl, __gb_read(gb, gb->cpu_reg.pc++));
        break;

    case 0x37: /* SCF */
        gb->cpu_reg.f_bits.n = 0;
        gb->cpu_reg.f_bits.h = 0;
        gb->cpu_reg.f_bits.c = 1;
        break;

    case 0x38: /* JP C, imm */
        if (gb->cpu_reg.f_bits.c) {
            int8_t temp = (int8_t) __gb_read(gb, gb->cpu_reg.pc++);
            gb->cpu_reg.pc += temp;
            inst_cycles += 4;
        } else
            gb->cpu_reg.pc++;

        break;

    case 0x39: { /* ADD HL, SP */
        uint_fast32_t temp = gb->cpu_reg.hl + gb->cpu_reg.sp;
        gb->cpu_reg.f_bits.n = 0;
        gb->cpu_reg.f_bits.h =
            ((gb->cpu_reg.hl & 0xFFF) + (gb->cpu_reg.sp & 0xFFF)) & 0x1000 ? 1
                                                                           : 0;
        gb->cpu_reg.f_bits.c = temp & 0x10000 ? 1 : 0;
        gb->cpu_reg.hl = (uint16_t) temp;
        break;
    }

    case 0x3A: /* LD A, (HL) */
        gb->cpu_reg.a = __gb_read(gb, gb->cpu_reg.hl--);
        break;

    case 0x3B: /* DEC SP */
        gb->cpu_reg.sp--;
        break;

    case 0x3C: /* INC A */
        gb->cpu_reg.a++;
        gb->cpu_reg.f_bits.z = (gb->cpu_reg.a == 0x00);
        gb->cpu_reg.f_bits.n = 0;
        gb->cpu_reg.f_bits.h = ((gb->cpu_reg.a & 0x0F) == 0x00);
        break;

    case 0x3D: /* DEC A */
        gb->cpu_reg.a--;
        gb->cpu_reg.f_bits.z = (gb->cpu_reg.a == 0x00);
        gb->cpu_reg.f_bits.n = 1;
        gb->cpu_reg.f_bits.h = ((gb->cpu_reg.a & 0x0F) == 0x0F);
        break;

    case 0x3E: /* LD A, imm */
        gb->cpu_reg.a = __gb_read(gb, gb->cpu_reg.pc++);
        break;

    case 0x3F: /* CCF */
        gb->cpu_reg.f_bits.n = 0;
        gb->cpu_reg.f_bits.h = 0;
        gb->cpu_reg.f_bits.c = ~gb->cpu_reg.f_bits.c;
        break;

    case 0x40: /* LD B, B */
        break;

    case 0x41: /* LD B, C */
        gb->cpu_reg.b = gb->cpu_reg.c;
        break;

    case 0x42: /* LD B, D */
        gb->cpu_reg.b = gb->cpu_reg.d;
        break;

    case 0x43: /* LD B, E */
        gb->cpu_reg.b = gb->cpu_reg.e;
        break;

    case 0x44: /* LD B, H */
        gb->cpu_reg.b = gb->cpu_reg.h;
        break;

    case 0x45: /* LD B, L */
        gb->cpu_reg.b = gb->cpu_reg.l;
        break;

    case 0x46: /* LD B, (HL) */
        gb->cpu_reg.b = __gb_read(gb, gb->cpu_reg.hl);
        break;

    case 0x47: /* LD B, A */
        gb->cpu_reg.b = gb->cpu_reg.a;
        break;

    case 0x48: /* LD C, B */
        gb->cpu_reg.c = gb->cpu_reg.b;
        break;

    case 0x49: /* LD C, C */
        break;

    case 0x4A: /* LD C, D */
        gb->cpu_reg.c = gb->cpu_reg.d;
        break;

    case 0x4B: /* LD C, E */
        gb->cpu_reg.c = gb->cpu_reg.e;
        break;

    case 0x4C: /* LD C, H */
        gb->cpu_reg.c = gb->cpu_reg.h;
        break;

    case 0x4D: /* LD C, L */
        gb->cpu_reg.c = gb->cpu_reg.l;
        break;

    case 0x4E: /* LD C, (HL) */
        gb->cpu_reg.c = __gb_read(gb, gb->cpu_reg.hl);
        break;

    case 0x4F: /* LD C, A */
        gb->cpu_reg.c = gb->cpu_reg.a;
        break;

    case 0x50: /* LD D, B */
        gb->cpu_reg.d = gb->cpu_reg.b;
        break;

    case 0x51: /* LD D, C */
        gb->cpu_reg.d = gb->cpu_reg.c;
        break;

    case 0x52: /* LD D, D */
        break;

    case 0x53: /* LD D, E */
        gb->cpu_reg.d = gb->cpu_reg.e;
        break;

    case 0x54: /* LD D, H */
        gb->cpu_reg.d = gb->cpu_reg.h;
        break;

    case 0x55: /* LD D, L */
        gb->cpu_reg.d = gb->cpu_reg.l;
        break;

    case 0x56: /* LD D, (HL) */
        gb->cpu_reg.d = __gb_read(gb, gb->cpu_reg.hl);
        break;

    case 0x57: /* LD D, A */
        gb->cpu_reg.d = gb->cpu_reg.a;
        break;

    case 0x58: /* LD E, B */
        gb->cpu_reg.e = gb->cpu_reg.b;
        break;

    case 0x59: /* LD E, C */
        gb->cpu_reg.e = gb->cpu_reg.c;
        break;

    case 0x5A: /* LD E, D */
        gb->cpu_reg.e = gb->cpu_reg.d;
        break;

    case 0x5B: /* LD E, E */
        break;

    case 0x5C: /* LD E, H */
        gb->cpu_reg.e = gb->cpu_reg.h;
        break;

    case 0x5D: /* LD E, L */
        gb->cpu_reg.e = gb->cpu_reg.l;
        break;

    case 0x5E: /* LD E, (HL) */
        gb->cpu_reg.e = __gb_read(gb, gb->cpu_reg.hl);
        break;

    case 0x5F: /* LD E, A */
        gb->cpu_reg.e = gb->cpu_reg.a;
        break;

    case 0x60: /* LD H, B */
        gb->cpu_reg.h = gb->cpu_reg.b;
        break;

    case 0x61: /* LD H, C */
        gb->cpu_reg.h = gb->cpu_reg.c;
        break;

    case 0x62: /* LD H, D */
        gb->cpu_reg.h = gb->cpu_reg.d;
        break;

    case 0x63: /* LD H, E */
        gb->cpu_reg.h = gb->cpu_reg.e;
        break;

    case 0x64: /* LD H, H */
        break;

    case 0x65: /* LD H, L */
        gb->cpu_reg.h = gb->cpu_reg.l;
        break;

    case 0x66: /* LD H, (HL) */
        gb->cpu_reg.h = __gb_read(gb, gb->cpu_reg.hl);
        break;

    case 0x67: /* LD H, A */
        gb->cpu_reg.h = gb->cpu_reg.a;
        break;

    case 0x68: /* LD L, B */
        gb->cpu_reg.l = gb->cpu_reg.b;
        break;

    case 0x69: /* LD L, C */
        gb->cpu_reg.l = gb->cpu_reg.c;
        break;

    case 0x6A: /* LD L, D */
        gb->cpu_reg.l = gb->cpu_reg.d;
        break;

    case 0x6B: /* LD L, E */
        gb->cpu_reg.l = gb->cpu_reg.e;
        break;

    case 0x6C: /* LD L, H */
        gb->cpu_reg.l = gb->cpu_reg.h;
        break;

    case 0x6D: /* LD L, L */
        break;

    case 0x6E: /* LD L, (HL) */
        gb->cpu_reg.l = __gb_read(gb, gb->cpu_reg.hl);
        break;

    case 0x6F: /* LD L, A */
        gb->cpu_reg.l = gb->cpu_reg.a;
        break;

    case 0x70: /* LD (HL), B */
        __gb_write(gb, gb->cpu_reg.hl, gb->cpu_reg.b);
        break;

    case 0x71: /* LD (HL), C */
        __gb_write(gb, gb->cpu_reg.hl, gb->cpu_reg.c);
        break;

    case 0x72: /* LD (HL), D */
        __gb_write(gb, gb->cpu_reg.hl, gb->cpu_reg.d);
        break;

    case 0x73: /* LD (HL), E */
        __gb_write(gb, gb->cpu_reg.hl, gb->cpu_reg.e);
        break;

    case 0x74: /* LD (HL), H */
        __gb_write(gb, gb->cpu_reg.hl, gb->cpu_reg.h);
        break;

    case 0x75: /* LD (HL), L */
        __gb_write(gb, gb->cpu_reg.hl, gb->cpu_reg.l);
        break;

    case 0x76: /* HALT */
        /* TODO: Emulate HALT bug? */
        gb->gb_halt = 1;
        break;

    case 0x77: /* LD (HL), A */
        __gb_write(gb, gb->cpu_reg.hl, gb->cpu_reg.a);
        break;

    case 0x78: /* LD A, B */
        gb->cpu_reg.a = gb->cpu_reg.b;
        break;

    case 0x79: /* LD A, C */
        gb->cpu_reg.a = gb->cpu_reg.c;
        break;

    case 0x7A: /* LD A, D */
        gb->cpu_reg.a = gb->cpu_reg.d;
        break;

    case 0x7B: /* LD A, E */
        gb->cpu_reg.a = gb->cpu_reg.e;
        break;

    case 0x7C: /* LD A, H */
        gb->cpu_reg.a = gb->cpu_reg.h;
        break;

    case 0x7D: /* LD A, L */
        gb->cpu_reg.a = gb->cpu_reg.l;
        break;

    case 0x7E: /* LD A, (HL) */
        gb->cpu_reg.a = __gb_read(gb, gb->cpu_reg.hl);
        break;

    case 0x7F: /* LD A, A */
        break;

    case 0x80: { /* ADD A, B */
        uint16_t temp = gb->cpu_reg.a + gb->cpu_reg.b;
        gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
        gb->cpu_reg.f_bits.n = 0;
        gb->cpu_reg.f_bits.h =
            (gb->cpu_reg.a ^ gb->cpu_reg.b ^ temp) & 0x10 ? 1 : 0;
        gb->cpu_reg.f_bits.c = (temp & 0xFF00) ? 1 : 0;
        gb->cpu_reg.a = (temp & 0xFF);
        break;
    }

    case 0x81: { /* ADD A, C */
        uint16_t temp = gb->cpu_reg.a + gb->cpu_reg.c;
        gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
        gb->cpu_reg.f_bits.n = 0;
        gb->cpu_reg.f_bits.h =
            (gb->cpu_reg.a ^ gb->cpu_reg.c ^ temp) & 0x10 ? 1 : 0;
        gb->cpu_reg.f_bits.c = (temp & 0xFF00) ? 1 : 0;
        gb->cpu_reg.a = (temp & 0xFF);
        break;
    }

    case 0x82: { /* ADD A, D */
        uint16_t temp = gb->cpu_reg.a + gb->cpu_reg.d;
        gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
        gb->cpu_reg.f_bits.n = 0;
        gb->cpu_reg.f_bits.h =
            (gb->cpu_reg.a ^ gb->cpu_reg.d ^ temp) & 0x10 ? 1 : 0;
        gb->cpu_reg.f_bits.c = (temp & 0xFF00) ? 1 : 0;
        gb->cpu_reg.a = (temp & 0xFF);
        break;
    }

    case 0x83: { /* ADD A, E */
        uint16_t temp = gb->cpu_reg.a + gb->cpu_reg.e;
        gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
        gb->cpu_reg.f_bits.n = 0;
        gb->cpu_reg.f_bits.h =
            (gb->cpu_reg.a ^ gb->cpu_reg.e ^ temp) & 0x10 ? 1 : 0;
        gb->cpu_reg.f_bits.c = (temp & 0xFF00) ? 1 : 0;
        gb->cpu_reg.a = (temp & 0xFF);
        break;
    }

    case 0x84: { /* ADD A, H */
        uint16_t temp = gb->cpu_reg.a + gb->cpu_reg.h;
        gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
        gb->cpu_reg.f_bits.n = 0;
        gb->cpu_reg.f_bits.h =
            (gb->cpu_reg.a ^ gb->cpu_reg.h ^ temp) & 0x10 ? 1 : 0;
        gb->cpu_reg.f_bits.c = (temp & 0xFF00) ? 1 : 0;
        gb->cpu_reg.a = (temp & 0xFF);
        break;
    }

    case 0x85: { /* ADD A, L */
        uint16_t temp = gb->cpu_reg.a + gb->cpu_reg.l;
        gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
        gb->cpu_reg.f_bits.n = 0;
        gb->cpu_reg.f_bits.h =
            (gb->cpu_reg.a ^ gb->cpu_reg.l ^ temp) & 0x10 ? 1 : 0;
        gb->cpu_reg.f_bits.c = (temp & 0xFF00) ? 1 : 0;
        gb->cpu_reg.a = (temp & 0xFF);
        break;
    }

    case 0x86: { /* ADD A, (HL) */
        uint8_t hl = __gb_read(gb, gb->cpu_reg.hl);
        uint16_t temp = gb->cpu_reg.a + hl;
        gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
        gb->cpu_reg.f_bits.n = 0;
        gb->cpu_reg.f_bits.h = (gb->cpu_reg.a ^ hl ^ temp) & 0x10 ? 1 : 0;
        gb->cpu_reg.f_bits.c = (temp & 0xFF00) ? 1 : 0;
        gb->cpu_reg.a = (temp & 0xFF);
        break;
    }

    case 0x87: { /* ADD A, A */
        uint16_t temp = gb->cpu_reg.a + gb->cpu_reg.a;
        gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
        gb->cpu_reg.f_bits.n = 0;
        gb->cpu_reg.f_bits.h = temp & 0x10 ? 1 : 0;
        gb->cpu_reg.f_bits.c = (temp & 0xFF00) ? 1 : 0;
        gb->cpu_reg.a = (temp & 0xFF);
        break;
    }

    case 0x88: { /* ADC A, B */
        uint16_t temp = gb->cpu_reg.a + gb->cpu_reg.b + gb->cpu_reg.f_bits.c;
        gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
        gb->cpu_reg.f_bits.n = 0;
        gb->cpu_reg.f_bits.h =
            (gb->cpu_reg.a ^ gb->cpu_reg.b ^ temp) & 0x10 ? 1 : 0;
        gb->cpu_reg.f_bits.c = (temp & 0xFF00) ? 1 : 0;
        gb->cpu_reg.a = (temp & 0xFF);
        break;
    }

    case 0x89: { /* ADC A, C */
        uint16_t temp = gb->cpu_reg.a + gb->cpu_reg.c + gb->cpu_reg.f_bits.c;
        gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
        gb->cpu_reg.f_bits.n = 0;
        gb->cpu_reg.f_bits.h =
            (gb->cpu_reg.a ^ gb->cpu_reg.c ^ temp) & 0x10 ? 1 : 0;
        gb->cpu_reg.f_bits.c = (temp & 0xFF00) ? 1 : 0;
        gb->cpu_reg.a = (temp & 0xFF);
        break;
    }

    case 0x8A: { /* ADC A, D */
        uint16_t temp = gb->cpu_reg.a + gb->cpu_reg.d + gb->cpu_reg.f_bits.c;
        gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
        gb->cpu_reg.f_bits.n = 0;
        gb->cpu_reg.f_bits.h =
            (gb->cpu_reg.a ^ gb->cpu_reg.d ^ temp) & 0x10 ? 1 : 0;
        gb->cpu_reg.f_bits.c = (temp & 0xFF00) ? 1 : 0;
        gb->cpu_reg.a = (temp & 0xFF);
        break;
    }

    case 0x8B: { /* ADC A, E */
        uint16_t temp = gb->cpu_reg.a + gb->cpu_reg.e + gb->cpu_reg.f_bits.c;
        gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
        gb->cpu_reg.f_bits.n = 0;
        gb->cpu_reg.f_bits.h =
            (gb->cpu_reg.a ^ gb->cpu_reg.e ^ temp) & 0x10 ? 1 : 0;
        gb->cpu_reg.f_bits.c = (temp & 0xFF00) ? 1 : 0;
        gb->cpu_reg.a = (temp & 0xFF);
        break;
    }

    case 0x8C: { /* ADC A, H */
        uint16_t temp = gb->cpu_reg.a + gb->cpu_reg.h + gb->cpu_reg.f_bits.c;
        gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
        gb->cpu_reg.f_bits.n = 0;
        gb->cpu_reg.f_bits.h =
            (gb->cpu_reg.a ^ gb->cpu_reg.h ^ temp) & 0x10 ? 1 : 0;
        gb->cpu_reg.f_bits.c = (temp & 0xFF00) ? 1 : 0;
        gb->cpu_reg.a = (temp & 0xFF);
        break;
    }

    case 0x8D: { /* ADC A, L */
        uint16_t temp = gb->cpu_reg.a + gb->cpu_reg.l + gb->cpu_reg.f_bits.c;
        gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
        gb->cpu_reg.f_bits.n = 0;
        gb->cpu_reg.f_bits.h =
            (gb->cpu_reg.a ^ gb->cpu_reg.l ^ temp) & 0x10 ? 1 : 0;
        gb->cpu_reg.f_bits.c = (temp & 0xFF00) ? 1 : 0;
        gb->cpu_reg.a = (temp & 0xFF);
        break;
    }

    case 0x8E: { /* ADC A, (HL) */
        uint8_t val = __gb_read(gb, gb->cpu_reg.hl);
        uint16_t temp = gb->cpu_reg.a + val + gb->cpu_reg.f_bits.c;
        gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
        gb->cpu_reg.f_bits.n = 0;
        gb->cpu_reg.f_bits.h = (gb->cpu_reg.a ^ val ^ temp) & 0x10 ? 1 : 0;
        gb->cpu_reg.f_bits.c = (temp & 0xFF00) ? 1 : 0;
        gb->cpu_reg.a = (temp & 0xFF);
        break;
    }

    case 0x8F: { /* ADC A, A */
        uint16_t temp = gb->cpu_reg.a + gb->cpu_reg.a + gb->cpu_reg.f_bits.c;
        gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
        gb->cpu_reg.f_bits.n = 0;
        /* TODO: Optimization here? */
        gb->cpu_reg.f_bits.h =
            (gb->cpu_reg.a ^ gb->cpu_reg.a ^ temp) & 0x10 ? 1 : 0;
        gb->cpu_reg.f_bits.c = (temp & 0xFF00) ? 1 : 0;
        gb->cpu_reg.a = (temp & 0xFF);
        break;
    }

    case 0x90: { /* SUB B */
        uint16_t temp = gb->cpu_reg.a - gb->cpu_reg.b;
        gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
        gb->cpu_reg.f_bits.n = 1;
        gb->cpu_reg.f_bits.h =
            (gb->cpu_reg.a ^ gb->cpu_reg.b ^ temp) & 0x10 ? 1 : 0;
        gb->cpu_reg.f_bits.c = (temp & 0xFF00) ? 1 : 0;
        gb->cpu_reg.a = (temp & 0xFF);
        break;
    }

    case 0x91: { /* SUB C */
        uint16_t temp = gb->cpu_reg.a - gb->cpu_reg.c;
        gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
        gb->cpu_reg.f_bits.n = 1;
        gb->cpu_reg.f_bits.h =
            (gb->cpu_reg.a ^ gb->cpu_reg.c ^ temp) & 0x10 ? 1 : 0;
        gb->cpu_reg.f_bits.c = (temp & 0xFF00) ? 1 : 0;
        gb->cpu_reg.a = (temp & 0xFF);
        break;
    }

    case 0x92: { /* SUB D */
        uint16_t temp = gb->cpu_reg.a - gb->cpu_reg.d;
        gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
        gb->cpu_reg.f_bits.n = 1;
        gb->cpu_reg.f_bits.h =
            (gb->cpu_reg.a ^ gb->cpu_reg.d ^ temp) & 0x10 ? 1 : 0;
        gb->cpu_reg.f_bits.c = (temp & 0xFF00) ? 1 : 0;
        gb->cpu_reg.a = (temp & 0xFF);
        break;
    }

    case 0x93: { /* SUB E */
        uint16_t temp = gb->cpu_reg.a - gb->cpu_reg.e;
        gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
        gb->cpu_reg.f_bits.n = 1;
        gb->cpu_reg.f_bits.h =
            (gb->cpu_reg.a ^ gb->cpu_reg.e ^ temp) & 0x10 ? 1 : 0;
        gb->cpu_reg.f_bits.c = (temp & 0xFF00) ? 1 : 0;
        gb->cpu_reg.a = (temp & 0xFF);
        break;
    }

    case 0x94: { /* SUB H */
        uint16_t temp = gb->cpu_reg.a - gb->cpu_reg.h;
        gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
        gb->cpu_reg.f_bits.n = 1;
        gb->cpu_reg.f_bits.h =
            (gb->cpu_reg.a ^ gb->cpu_reg.h ^ temp) & 0x10 ? 1 : 0;
        gb->cpu_reg.f_bits.c = (temp & 0xFF00) ? 1 : 0;
        gb->cpu_reg.a = (temp & 0xFF);
        break;
    }

    case 0x95: { /* SUB L */
        uint16_t temp = gb->cpu_reg.a - gb->cpu_reg.l;
        gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
        gb->cpu_reg.f_bits.n = 1;
        gb->cpu_reg.f_bits.h =
            (gb->cpu_reg.a ^ gb->cpu_reg.l ^ temp) & 0x10 ? 1 : 0;
        gb->cpu_reg.f_bits.c = (temp & 0xFF00) ? 1 : 0;
        gb->cpu_reg.a = (temp & 0xFF);
        break;
    }

    case 0x96: { /* SUB (HL) */
        uint8_t val = __gb_read(gb, gb->cpu_reg.hl);
        uint16_t temp = gb->cpu_reg.a - val;
        gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
        gb->cpu_reg.f_bits.n = 1;
        gb->cpu_reg.f_bits.h = (gb->cpu_reg.a ^ val ^ temp) & 0x10 ? 1 : 0;
        gb->cpu_reg.f_bits.c = (temp & 0xFF00) ? 1 : 0;
        gb->cpu_reg.a = (temp & 0xFF);
        break;
    }

    case 0x97: /* SUB A */
        gb->cpu_reg.a = 0;
        gb->cpu_reg.f_bits.z = 1;
        gb->cpu_reg.f_bits.n = 1;
        gb->cpu_reg.f_bits.h = 0;
        gb->cpu_reg.f_bits.c = 0;
        break;

    case 0x98: { /* SBC A, B */
        uint16_t temp = gb->cpu_reg.a - gb->cpu_reg.b - gb->cpu_reg.f_bits.c;
        gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
        gb->cpu_reg.f_bits.n = 1;
        gb->cpu_reg.f_bits.h =
            (gb->cpu_reg.a ^ gb->cpu_reg.b ^ temp) & 0x10 ? 1 : 0;
        gb->cpu_reg.f_bits.c = (temp & 0xFF00) ? 1 : 0;
        gb->cpu_reg.a = (temp & 0xFF);
        break;
    }

    case 0x99: { /* SBC A, C */
        uint16_t temp = gb->cpu_reg.a - gb->cpu_reg.c - gb->cpu_reg.f_bits.c;
        gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
        gb->cpu_reg.f_bits.n = 1;
        gb->cpu_reg.f_bits.h =
            (gb->cpu_reg.a ^ gb->cpu_reg.c ^ temp) & 0x10 ? 1 : 0;
        gb->cpu_reg.f_bits.c = (temp & 0xFF00) ? 1 : 0;
        gb->cpu_reg.a = (temp & 0xFF);
        break;
    }

    case 0x9A: { /* SBC A, D */
        uint16_t temp = gb->cpu_reg.a - gb->cpu_reg.d - gb->cpu_reg.f_bits.c;
        gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
        gb->cpu_reg.f_bits.n = 1;
        gb->cpu_reg.f_bits.h =
            (gb->cpu_reg.a ^ gb->cpu_reg.d ^ temp) & 0x10 ? 1 : 0;
        gb->cpu_reg.f_bits.c = (temp & 0xFF00) ? 1 : 0;
        gb->cpu_reg.a = (temp & 0xFF);
        break;
    }

    case 0x9B: { /* SBC A, E */
        uint16_t temp = gb->cpu_reg.a - gb->cpu_reg.e - gb->cpu_reg.f_bits.c;
        gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
        gb->cpu_reg.f_bits.n = 1;
        gb->cpu_reg.f_bits.h =
            (gb->cpu_reg.a ^ gb->cpu_reg.e ^ temp) & 0x10 ? 1 : 0;
        gb->cpu_reg.f_bits.c = (temp & 0xFF00) ? 1 : 0;
        gb->cpu_reg.a = (temp & 0xFF);
        break;
    }

    case 0x9C: { /* SBC A, H */
        uint16_t temp = gb->cpu_reg.a - gb->cpu_reg.h - gb->cpu_reg.f_bits.c;
        gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
        gb->cpu_reg.f_bits.n = 1;
        gb->cpu_reg.f_bits.h =
            (gb->cpu_reg.a ^ gb->cpu_reg.h ^ temp) & 0x10 ? 1 : 0;
        gb->cpu_reg.f_bits.c = (temp & 0xFF00) ? 1 : 0;
        gb->cpu_reg.a = (temp & 0xFF);
        break;
    }

    case 0x9D: { /* SBC A, L */
        uint16_t temp = gb->cpu_reg.a - gb->cpu_reg.l - gb->cpu_reg.f_bits.c;
        gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
        gb->cpu_reg.f_bits.n = 1;
        gb->cpu_reg.f_bits.h =
            (gb->cpu_reg.a ^ gb->cpu_reg.l ^ temp) & 0x10 ? 1 : 0;
        gb->cpu_reg.f_bits.c = (temp & 0xFF00) ? 1 : 0;
        gb->cpu_reg.a = (temp & 0xFF);
        break;
    }

    case 0x9E: { /* SBC A, (HL) */
        uint8_t val = __gb_read(gb, gb->cpu_reg.hl);
        uint16_t temp = gb->cpu_reg.a - val - gb->cpu_reg.f_bits.c;
        gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
        gb->cpu_reg.f_bits.n = 1;
        gb->cpu_reg.f_bits.h = (gb->cpu_reg.a ^ val ^ temp) & 0x10 ? 1 : 0;
        gb->cpu_reg.f_bits.c = (temp & 0xFF00) ? 1 : 0;
        gb->cpu_reg.a = (temp & 0xFF);
        break;
    }

    case 0x9F: /* SBC A, A */
        gb->cpu_reg.a = gb->cpu_reg.f_bits.c ? 0xFF : 0x00;
        gb->cpu_reg.f_bits.z = gb->cpu_reg.f_bits.c ? 0x00 : 0x01;
        gb->cpu_reg.f_bits.n = 1;
        gb->cpu_reg.f_bits.h = gb->cpu_reg.f_bits.c;
        break;

    case 0xA0: /* AND B */
        gb->cpu_reg.a = gb->cpu_reg.a & gb->cpu_reg.b;
        gb->cpu_reg.f_bits.z = (gb->cpu_reg.a == 0x00);
        gb->cpu_reg.f_bits.n = 0;
        gb->cpu_reg.f_bits.h = 1;
        gb->cpu_reg.f_bits.c = 0;
        break;

    case 0xA1: /* AND C */
        gb->cpu_reg.a = gb->cpu_reg.a & gb->cpu_reg.c;
        gb->cpu_reg.f_bits.z = (gb->cpu_reg.a == 0x00);
        gb->cpu_reg.f_bits.n = 0;
        gb->cpu_reg.f_bits.h = 1;
        gb->cpu_reg.f_bits.c = 0;
        break;

    case 0xA2: /* AND D */
        gb->cpu_reg.a = gb->cpu_reg.a & gb->cpu_reg.d;
        gb->cpu_reg.f_bits.z = (gb->cpu_reg.a == 0x00);
        gb->cpu_reg.f_bits.n = 0;
        gb->cpu_reg.f_bits.h = 1;
        gb->cpu_reg.f_bits.c = 0;
        break;

    case 0xA3: /* AND E */
        gb->cpu_reg.a = gb->cpu_reg.a & gb->cpu_reg.e;
        gb->cpu_reg.f_bits.z = (gb->cpu_reg.a == 0x00);
        gb->cpu_reg.f_bits.n = 0;
        gb->cpu_reg.f_bits.h = 1;
        gb->cpu_reg.f_bits.c = 0;
        break;

    case 0xA4: /* AND H */
        gb->cpu_reg.a = gb->cpu_reg.a & gb->cpu_reg.h;
        gb->cpu_reg.f_bits.z = (gb->cpu_reg.a == 0x00);
        gb->cpu_reg.f_bits.n = 0;
        gb->cpu_reg.f_bits.h = 1;
        gb->cpu_reg.f_bits.c = 0;
        break;

    case 0xA5: /* AND L */
        gb->cpu_reg.a = gb->cpu_reg.a & gb->cpu_reg.l;
        gb->cpu_reg.f_bits.z = (gb->cpu_reg.a == 0x00);
        gb->cpu_reg.f_bits.n = 0;
        gb->cpu_reg.f_bits.h = 1;
        gb->cpu_reg.f_bits.c = 0;
        break;

    case 0xA6: /* AND B */
        gb->cpu_reg.a = gb->cpu_reg.a & __gb_read(gb, gb->cpu_reg.hl);
        gb->cpu_reg.f_bits.z = (gb->cpu_reg.a == 0x00);
        gb->cpu_reg.f_bits.n = 0;
        gb->cpu_reg.f_bits.h = 1;
        gb->cpu_reg.f_bits.c = 0;
        break;

    case 0xA7: /* AND A */
        gb->cpu_reg.f_bits.z = (gb->cpu_reg.a == 0x00);
        gb->cpu_reg.f_bits.n = 0;
        gb->cpu_reg.f_bits.h = 1;
        gb->cpu_reg.f_bits.c = 0;
        break;

    case 0xA8: /* XOR B */
        gb->cpu_reg.a = gb->cpu_reg.a ^ gb->cpu_reg.b;
        gb->cpu_reg.f_bits.z = (gb->cpu_reg.a == 0x00);
        gb->cpu_reg.f_bits.n = 0;
        gb->cpu_reg.f_bits.h = 0;
        gb->cpu_reg.f_bits.c = 0;
        break;

    case 0xA9: /* XOR C */
        gb->cpu_reg.a = gb->cpu_reg.a ^ gb->cpu_reg.c;
        gb->cpu_reg.f_bits.z = (gb->cpu_reg.a == 0x00);
        gb->cpu_reg.f_bits.n = 0;
        gb->cpu_reg.f_bits.h = 0;
        gb->cpu_reg.f_bits.c = 0;
        break;

    case 0xAA: /* XOR D */
        gb->cpu_reg.a = gb->cpu_reg.a ^ gb->cpu_reg.d;
        gb->cpu_reg.f_bits.z = (gb->cpu_reg.a == 0x00);
        gb->cpu_reg.f_bits.n = 0;
        gb->cpu_reg.f_bits.h = 0;
        gb->cpu_reg.f_bits.c = 0;
        break;

    case 0xAB: /* XOR E */
        gb->cpu_reg.a = gb->cpu_reg.a ^ gb->cpu_reg.e;
        gb->cpu_reg.f_bits.z = (gb->cpu_reg.a == 0x00);
        gb->cpu_reg.f_bits.n = 0;
        gb->cpu_reg.f_bits.h = 0;
        gb->cpu_reg.f_bits.c = 0;
        break;

    case 0xAC: /* XOR H */
        gb->cpu_reg.a = gb->cpu_reg.a ^ gb->cpu_reg.h;
        gb->cpu_reg.f_bits.z = (gb->cpu_reg.a == 0x00);
        gb->cpu_reg.f_bits.n = 0;
        gb->cpu_reg.f_bits.h = 0;
        gb->cpu_reg.f_bits.c = 0;
        break;

    case 0xAD: /* XOR L */
        gb->cpu_reg.a = gb->cpu_reg.a ^ gb->cpu_reg.l;
        gb->cpu_reg.f_bits.z = (gb->cpu_reg.a == 0x00);
        gb->cpu_reg.f_bits.n = 0;
        gb->cpu_reg.f_bits.h = 0;
        gb->cpu_reg.f_bits.c = 0;
        break;

    case 0xAE: /* XOR (HL) */
        gb->cpu_reg.a = gb->cpu_reg.a ^ __gb_read(gb, gb->cpu_reg.hl);
        gb->cpu_reg.f_bits.z = (gb->cpu_reg.a == 0x00);
        gb->cpu_reg.f_bits.n = 0;
        gb->cpu_reg.f_bits.h = 0;
        gb->cpu_reg.f_bits.c = 0;
        break;

    case 0xAF: /* XOR A */
        gb->cpu_reg.a = 0x00;
        gb->cpu_reg.f_bits.z = 1;
        gb->cpu_reg.f_bits.n = 0;
        gb->cpu_reg.f_bits.h = 0;
        gb->cpu_reg.f_bits.c = 0;
        break;

    case 0xB0: /* OR B */
        gb->cpu_reg.a = gb->cpu_reg.a | gb->cpu_reg.b;
        gb->cpu_reg.f_bits.z = (gb->cpu_reg.a == 0x00);
        gb->cpu_reg.f_bits.n = 0;
        gb->cpu_reg.f_bits.h = 0;
        gb->cpu_reg.f_bits.c = 0;
        break;

    case 0xB1: /* OR C */
        gb->cpu_reg.a = gb->cpu_reg.a | gb->cpu_reg.c;
        gb->cpu_reg.f_bits.z = (gb->cpu_reg.a == 0x00);
        gb->cpu_reg.f_bits.n = 0;
        gb->cpu_reg.f_bits.h = 0;
        gb->cpu_reg.f_bits.c = 0;
        break;

    case 0xB2: /* OR D */
        gb->cpu_reg.a = gb->cpu_reg.a | gb->cpu_reg.d;
        gb->cpu_reg.f_bits.z = (gb->cpu_reg.a == 0x00);
        gb->cpu_reg.f_bits.n = 0;
        gb->cpu_reg.f_bits.h = 0;
        gb->cpu_reg.f_bits.c = 0;
        break;

    case 0xB3: /* OR E */
        gb->cpu_reg.a = gb->cpu_reg.a | gb->cpu_reg.e;
        gb->cpu_reg.f_bits.z = (gb->cpu_reg.a == 0x00);
        gb->cpu_reg.f_bits.n = 0;
        gb->cpu_reg.f_bits.h = 0;
        gb->cpu_reg.f_bits.c = 0;
        break;

    case 0xB4: /* OR H */
        gb->cpu_reg.a = gb->cpu_reg.a | gb->cpu_reg.h;
        gb->cpu_reg.f_bits.z = (gb->cpu_reg.a == 0x00);
        gb->cpu_reg.f_bits.n = 0;
        gb->cpu_reg.f_bits.h = 0;
        gb->cpu_reg.f_bits.c = 0;
        break;

    case 0xB5: /* OR L */
        gb->cpu_reg.a = gb->cpu_reg.a | gb->cpu_reg.l;
        gb->cpu_reg.f_bits.z = (gb->cpu_reg.a == 0x00);
        gb->cpu_reg.f_bits.n = 0;
        gb->cpu_reg.f_bits.h = 0;
        gb->cpu_reg.f_bits.c = 0;
        break;

    case 0xB6: /* OR (HL) */
        gb->cpu_reg.a = gb->cpu_reg.a | __gb_read(gb, gb->cpu_reg.hl);
        gb->cpu_reg.f_bits.z = (gb->cpu_reg.a == 0x00);
        gb->cpu_reg.f_bits.n = 0;
        gb->cpu_reg.f_bits.h = 0;
        gb->cpu_reg.f_bits.c = 0;
        break;

    case 0xB7: /* OR A */
        gb->cpu_reg.f_bits.z = (gb->cpu_reg.a == 0x00);
        gb->cpu_reg.f_bits.n = 0;
        gb->cpu_reg.f_bits.h = 0;
        gb->cpu_reg.f_bits.c = 0;
        break;

    case 0xB8: { /* CP B */
        uint16_t temp = gb->cpu_reg.a - gb->cpu_reg.b;
        gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
        gb->cpu_reg.f_bits.n = 1;
        gb->cpu_reg.f_bits.h =
            (gb->cpu_reg.a ^ gb->cpu_reg.b ^ temp) & 0x10 ? 1 : 0;
        gb->cpu_reg.f_bits.c = (temp & 0xFF00) ? 1 : 0;
        break;
    }

    case 0xB9: { /* CP C */
        uint16_t temp = gb->cpu_reg.a - gb->cpu_reg.c;
        gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
        gb->cpu_reg.f_bits.n = 1;
        gb->cpu_reg.f_bits.h =
            (gb->cpu_reg.a ^ gb->cpu_reg.c ^ temp) & 0x10 ? 1 : 0;
        gb->cpu_reg.f_bits.c = (temp & 0xFF00) ? 1 : 0;
        break;
    }

    case 0xBA: { /* CP D */
        uint16_t temp = gb->cpu_reg.a - gb->cpu_reg.d;
        gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
        gb->cpu_reg.f_bits.n = 1;
        gb->cpu_reg.f_bits.h =
            (gb->cpu_reg.a ^ gb->cpu_reg.d ^ temp) & 0x10 ? 1 : 0;
        gb->cpu_reg.f_bits.c = (temp & 0xFF00) ? 1 : 0;
        break;
    }

    case 0xBB: { /* CP E */
        uint16_t temp = gb->cpu_reg.a - gb->cpu_reg.e;
        gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
        gb->cpu_reg.f_bits.n = 1;
        gb->cpu_reg.f_bits.h =
            (gb->cpu_reg.a ^ gb->cpu_reg.e ^ temp) & 0x10 ? 1 : 0;
        gb->cpu_reg.f_bits.c = (temp & 0xFF00) ? 1 : 0;
        break;
    }

    case 0xBC: { /* CP H */
        uint16_t temp = gb->cpu_reg.a - gb->cpu_reg.h;
        gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
        gb->cpu_reg.f_bits.n = 1;
        gb->cpu_reg.f_bits.h =
            (gb->cpu_reg.a ^ gb->cpu_reg.h ^ temp) & 0x10 ? 1 : 0;
        gb->cpu_reg.f_bits.c = (temp & 0xFF00) ? 1 : 0;
        break;
    }

    case 0xBD: { /* CP L */
        uint16_t temp = gb->cpu_reg.a - gb->cpu_reg.l;
        gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
        gb->cpu_reg.f_bits.n = 1;
        gb->cpu_reg.f_bits.h =
            (gb->cpu_reg.a ^ gb->cpu_reg.l ^ temp) & 0x10 ? 1 : 0;
        gb->cpu_reg.f_bits.c = (temp & 0xFF00) ? 1 : 0;
        break;
    }

    /* TODO: Optimization by combining similar opcode routines. */
    case 0xBE: { /* CP B */
        uint8_t val = __gb_read(gb, gb->cpu_reg.hl);
        uint16_t temp = gb->cpu_reg.a - val;
        gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
        gb->cpu_reg.f_bits.n = 1;
        gb->cpu_reg.f_bits.h = (gb->cpu_reg.a ^ val ^ temp) & 0x10 ? 1 : 0;
        gb->cpu_reg.f_bits.c = (temp & 0xFF00) ? 1 : 0;
        break;
    }

    case 0xBF: /* CP A */
        gb->cpu_reg.f_bits.z = 1;
        gb->cpu_reg.f_bits.n = 1;
        gb->cpu_reg.f_bits.h = 0;
        gb->cpu_reg.f_bits.c = 0;
        break;

    case 0xC0: /* RET NZ */
        if (!gb->cpu_reg.f_bits.z) {
            gb->cpu_reg.pc = __gb_read(gb, gb->cpu_reg.sp++);
            gb->cpu_reg.pc |= __gb_read(gb, gb->cpu_reg.sp++) << 8;
            inst_cycles += 12;
        }

        break;

    case 0xC1: /* POP BC */
        gb->cpu_reg.c = __gb_read(gb, gb->cpu_reg.sp++);
        gb->cpu_reg.b = __gb_read(gb, gb->cpu_reg.sp++);
        break;

    case 0xC2: /* JP NZ, imm */
        if (!gb->cpu_reg.f_bits.z) {
            uint16_t temp = __gb_read(gb, gb->cpu_reg.pc++);
            temp |= __gb_read(gb, gb->cpu_reg.pc++) << 8;
            gb->cpu_reg.pc = temp;
            inst_cycles += 4;
        } else
            gb->cpu_reg.pc += 2;

        break;

    case 0xC3: { /* JP imm */
        uint16_t temp = __gb_read(gb, gb->cpu_reg.pc++);
        temp |= __gb_read(gb, gb->cpu_reg.pc) << 8;
        gb->cpu_reg.pc = temp;
        break;
    }

    case 0xC4: /* CALL NZ imm */
        if (!gb->cpu_reg.f_bits.z) {
            uint16_t temp = __gb_read(gb, gb->cpu_reg.pc++);
            temp |= __gb_read(gb, gb->cpu_reg.pc++) << 8;
            __gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc >> 8);
            __gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc & 0xFF);
            gb->cpu_reg.pc = temp;
            inst_cycles += 12;
        } else
            gb->cpu_reg.pc += 2;

        break;

    case 0xC5: /* PUSH BC */
        __gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.b);
        __gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.c);
        break;

    case 0xC6: { /* ADD A, imm */
        uint8_t value = __gb_read(gb, gb->cpu_reg.pc++);
        uint16_t calc = gb->cpu_reg.a + value;
        gb->cpu_reg.f_bits.z = ((uint8_t) calc == 0) ? 1 : 0;
        gb->cpu_reg.f_bits.h =
            ((gb->cpu_reg.a & 0xF) + (value & 0xF) > 0x0F) ? 1 : 0;
        gb->cpu_reg.f_bits.c = calc > 0xFF ? 1 : 0;
        gb->cpu_reg.f_bits.n = 0;
        gb->cpu_reg.a = (uint8_t) calc;
        break;
    }

    case 0xC7: /* RST 0x0000 */
        __gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc >> 8);
        __gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc & 0xFF);
        gb->cpu_reg.pc = 0x0000;
        break;

    case 0xC8: /* RET Z */
        if (gb->cpu_reg.f_bits.z) {
            uint16_t temp = __gb_read(gb, gb->cpu_reg.sp++);
            temp |= __gb_read(gb, gb->cpu_reg.sp++) << 8;
            gb->cpu_reg.pc = temp;
            inst_cycles += 12;
        }

        break;

    case 0xC9: { /* RET */
        uint16_t temp = __gb_read(gb, gb->cpu_reg.sp++);
        temp |= __gb_read(gb, gb->cpu_reg.sp++) << 8;
        gb->cpu_reg.pc = temp;
        break;
    }

    case 0xCA: /* JP Z, imm */
        if (gb->cpu_reg.f_bits.z) {
            uint16_t temp = __gb_read(gb, gb->cpu_reg.pc++);
            temp |= __gb_read(gb, gb->cpu_reg.pc++) << 8;
            gb->cpu_reg.pc = temp;
            inst_cycles += 4;
        } else
            gb->cpu_reg.pc += 2;

        break;

    case 0xCB: /* CB INST */
        inst_cycles = __gb_execute_cb(gb);
        break;

    case 0xCC: /* CALL Z, imm */
        if (gb->cpu_reg.f_bits.z) {
            uint16_t temp = __gb_read(gb, gb->cpu_reg.pc++);
            temp |= __gb_read(gb, gb->cpu_reg.pc++) << 8;
            __gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc >> 8);
            __gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc & 0xFF);
            gb->cpu_reg.pc = temp;
            inst_cycles += 12;
        } else
            gb->cpu_reg.pc += 2;

        break;

    case 0xCD: { /* CALL imm */
        uint16_t addr = __gb_read(gb, gb->cpu_reg.pc++);
        addr |= __gb_read(gb, gb->cpu_reg.pc++) << 8;
        __gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc >> 8);
        __gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc & 0xFF);
        gb->cpu_reg.pc = addr;
    } break;

    case 0xCE: { /* ADC A, imm */
        uint8_t value, a, carry;
        value = __gb_read(gb, gb->cpu_reg.pc++);
        a = gb->cpu_reg.a;
        carry = gb->cpu_reg.f_bits.c;
        gb->cpu_reg.a = a + value + carry;

        gb->cpu_reg.f_bits.z = gb->cpu_reg.a == 0 ? 1 : 0;
        gb->cpu_reg.f_bits.h =
            ((a & 0xF) + (value & 0xF) + carry > 0x0F) ? 1 : 0;
        gb->cpu_reg.f_bits.c =
            (((uint16_t) a) + ((uint16_t) value) + carry > 0xFF) ? 1 : 0;
        gb->cpu_reg.f_bits.n = 0;
        break;
    }

    case 0xCF: /* RST 0x0008 */
        __gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc >> 8);
        __gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc & 0xFF);
        gb->cpu_reg.pc = 0x0008;
        break;

    case 0xD0: /* RET NC */
        if (!gb->cpu_reg.f_bits.c) {
            uint16_t temp = __gb_read(gb, gb->cpu_reg.sp++);
            temp |= __gb_read(gb, gb->cpu_reg.sp++) << 8;
            gb->cpu_reg.pc = temp;
            inst_cycles += 12;
        }

        break;

    case 0xD1: /* POP DE */
        gb->cpu_reg.e = __gb_read(gb, gb->cpu_reg.sp++);
        gb->cpu_reg.d = __gb_read(gb, gb->cpu_reg.sp++);
        break;

    case 0xD2: /* JP NC, imm */
        if (!gb->cpu_reg.f_bits.c) {
            uint16_t temp = __gb_read(gb, gb->cpu_reg.pc++);
            temp |= __gb_read(gb, gb->cpu_reg.pc++) << 8;
            gb->cpu_reg.pc = temp;
            inst_cycles += 4;
        } else
            gb->cpu_reg.pc += 2;

        break;

    case 0xD4: /* CALL NC, imm */
        if (!gb->cpu_reg.f_bits.c) {
            uint16_t temp = __gb_read(gb, gb->cpu_reg.pc++);
            temp |= __gb_read(gb, gb->cpu_reg.pc++) << 8;
            __gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc >> 8);
            __gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc & 0xFF);
            gb->cpu_reg.pc = temp;
            inst_cycles += 12;
        } else
            gb->cpu_reg.pc += 2;

        break;

    case 0xD5: /* PUSH DE */
        __gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.d);
        __gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.e);
        break;

    case 0xD6: { /* SUB imm */
        uint8_t val = __gb_read(gb, gb->cpu_reg.pc++);
        uint16_t temp = gb->cpu_reg.a - val;
        gb->cpu_reg.f_bits.z = ((temp & 0xFF) == 0x00);
        gb->cpu_reg.f_bits.n = 1;
        gb->cpu_reg.f_bits.h = (gb->cpu_reg.a ^ val ^ temp) & 0x10 ? 1 : 0;
        gb->cpu_reg.f_bits.c = (temp & 0xFF00) ? 1 : 0;
        gb->cpu_reg.a = (temp & 0xFF);
        break;
    }

    case 0xD7: /* RST 0x0010 */
        __gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc >> 8);
        __gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc & 0xFF);
        gb->cpu_reg.pc = 0x0010;
        break;

    case 0xD8: /* RET C */
        if (gb->cpu_reg.f_bits.c) {
            uint16_t temp = __gb_read(gb, gb->cpu_reg.sp++);
            temp |= __gb_read(gb, gb->cpu_reg.sp++) << 8;
            gb->cpu_reg.pc = temp;
            inst_cycles += 12;
        }

        break;

    case 0xD9: { /* RETI */
        uint16_t temp = __gb_read(gb, gb->cpu_reg.sp++);
        temp |= __gb_read(gb, gb->cpu_reg.sp++) << 8;
        gb->cpu_reg.pc = temp;
        gb->gb_ime = 1;
    } break;

    case 0xDA: /* JP C, imm */
        if (gb->cpu_reg.f_bits.c) {
            uint16_t addr = __gb_read(gb, gb->cpu_reg.pc++);
            addr |= __gb_read(gb, gb->cpu_reg.pc++) << 8;
            gb->cpu_reg.pc = addr;
            inst_cycles += 4;
        } else
            gb->cpu_reg.pc += 2;

        break;

    case 0xDC: /* CALL C, imm */
        if (gb->cpu_reg.f_bits.c) {
            uint16_t temp = __gb_read(gb, gb->cpu_reg.pc++);
            temp |= __gb_read(gb, gb->cpu_reg.pc++) << 8;
            __gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc >> 8);
            __gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc & 0xFF);
            gb->cpu_reg.pc = temp;
            inst_cycles += 12;
        } else
            gb->cpu_reg.pc += 2;

        break;

    case 0xDE: { /* SBC A, imm */
        uint8_t temp_8 = __gb_read(gb, gb->cpu_reg.pc++);
        uint16_t temp_16 = gb->cpu_reg.a - temp_8 - gb->cpu_reg.f_bits.c;
        gb->cpu_reg.f_bits.z = ((temp_16 & 0xFF) == 0x00);
        gb->cpu_reg.f_bits.n = 1;
        gb->cpu_reg.f_bits.h =
            (gb->cpu_reg.a ^ temp_8 ^ temp_16) & 0x10 ? 1 : 0;
        gb->cpu_reg.f_bits.c = (temp_16 & 0xFF00) ? 1 : 0;
        gb->cpu_reg.a = (temp_16 & 0xFF);
        break;
    }

    case 0xDF: /* RST 0x0018 */
        __gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc >> 8);
        __gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc & 0xFF);
        gb->cpu_reg.pc = 0x0018;
        break;

    case 0xE0: /* LD (0xFF00+imm), A */
        __gb_write(gb, 0xFF00 | __gb_read(gb, gb->cpu_reg.pc++), gb->cpu_reg.a);
        break;

    case 0xE1: /* POP HL */
        gb->cpu_reg.l = __gb_read(gb, gb->cpu_reg.sp++);
        gb->cpu_reg.h = __gb_read(gb, gb->cpu_reg.sp++);
        break;

    case 0xE2: /* LD (C), A */
        __gb_write(gb, 0xFF00 | gb->cpu_reg.c, gb->cpu_reg.a);
        break;

    case 0xE5: /* PUSH HL */
        __gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.h);
        __gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.l);
        break;

    case 0xE6: /* AND imm */
        /* TODO: Optimization? */
        gb->cpu_reg.a = gb->cpu_reg.a & __gb_read(gb, gb->cpu_reg.pc++);
        gb->cpu_reg.f_bits.z = (gb->cpu_reg.a == 0x00);
        gb->cpu_reg.f_bits.n = 0;
        gb->cpu_reg.f_bits.h = 1;
        gb->cpu_reg.f_bits.c = 0;
        break;

    case 0xE7: /* RST 0x0020 */
        __gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc >> 8);
        __gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc & 0xFF);
        gb->cpu_reg.pc = 0x0020;
        break;

    case 0xE8: { /* ADD SP, imm */
        int8_t offset = (int8_t) __gb_read(gb, gb->cpu_reg.pc++);
        /* TODO: Move flag assignments for optimization */
        gb->cpu_reg.f_bits.z = 0;
        gb->cpu_reg.f_bits.n = 0;
        gb->cpu_reg.f_bits.h =
            ((gb->cpu_reg.sp & 0xF) + (offset & 0xF) > 0xF) ? 1 : 0;
        gb->cpu_reg.f_bits.c =
            ((gb->cpu_reg.sp & 0xFF) + (offset & 0xFF) > 0xFF);
        gb->cpu_reg.sp += offset;
        break;
    }

    case 0xE9: /* JP (HL) */
        gb->cpu_reg.pc = gb->cpu_reg.hl;
        break;

    case 0xEA: { /* LD (imm), A */
        uint16_t addr = __gb_read(gb, gb->cpu_reg.pc++);
        addr |= __gb_read(gb, gb->cpu_reg.pc++) << 8;
        __gb_write(gb, addr, gb->cpu_reg.a);
        break;
    }

    case 0xEE: /* XOR imm */
        gb->cpu_reg.a = gb->cpu_reg.a ^ __gb_read(gb, gb->cpu_reg.pc++);
        gb->cpu_reg.f_bits.z = (gb->cpu_reg.a == 0x00);
        gb->cpu_reg.f_bits.n = 0;
        gb->cpu_reg.f_bits.h = 0;
        gb->cpu_reg.f_bits.c = 0;
        break;

    case 0xEF: /* RST 0x0028 */
        __gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc >> 8);
        __gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc & 0xFF);
        gb->cpu_reg.pc = 0x0028;
        break;

    case 0xF0: /* LD A, (0xFF00+imm) */
        gb->cpu_reg.a = __gb_read(gb, 0xFF00 | __gb_read(gb, gb->cpu_reg.pc++));
        break;

    case 0xF1: { /* POP AF */
        uint8_t temp_8 = __gb_read(gb, gb->cpu_reg.sp++);
        gb->cpu_reg.f_bits.z = (temp_8 >> 7) & 1;
        gb->cpu_reg.f_bits.n = (temp_8 >> 6) & 1;
        gb->cpu_reg.f_bits.h = (temp_8 >> 5) & 1;
        gb->cpu_reg.f_bits.c = (temp_8 >> 4) & 1;
        gb->cpu_reg.a = __gb_read(gb, gb->cpu_reg.sp++);
        break;
    }

    case 0xF2: /* LD A, (C) */
        gb->cpu_reg.a = __gb_read(gb, 0xFF00 | gb->cpu_reg.c);
        break;

    case 0xF3: /* DI */
        gb->gb_ime = 0;
        break;

    case 0xF5: /* PUSH AF */
        __gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.a);
        __gb_write(gb, --gb->cpu_reg.sp,
                   gb->cpu_reg.f_bits.z << 7 | gb->cpu_reg.f_bits.n << 6 |
                       gb->cpu_reg.f_bits.h << 5 | gb->cpu_reg.f_bits.c << 4);
        break;

    case 0xF6: /* OR imm */
        gb->cpu_reg.a = gb->cpu_reg.a | __gb_read(gb, gb->cpu_reg.pc++);
        gb->cpu_reg.f_bits.z = (gb->cpu_reg.a == 0x00);
        gb->cpu_reg.f_bits.n = 0;
        gb->cpu_reg.f_bits.h = 0;
        gb->cpu_reg.f_bits.c = 0;
        break;

    case 0xF7: /* PUSH AF */
        __gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc >> 8);
        __gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc & 0xFF);
        gb->cpu_reg.pc = 0x0030;
        break;

    case 0xF8: { /* LD HL, SP+/-imm */
        int8_t offset = (int8_t) __gb_read(gb, gb->cpu_reg.pc++);
        gb->cpu_reg.hl = gb->cpu_reg.sp + offset;
        gb->cpu_reg.f_bits.z = 0;
        gb->cpu_reg.f_bits.n = 0;
        gb->cpu_reg.f_bits.h =
            ((gb->cpu_reg.sp & 0xF) + (offset & 0xF) > 0xF) ? 1 : 0;
        gb->cpu_reg.f_bits.c =
            ((gb->cpu_reg.sp & 0xFF) + (offset & 0xFF) > 0xFF) ? 1 : 0;
        break;
    }

    case 0xF9: /* LD SP, HL */
        gb->cpu_reg.sp = gb->cpu_reg.hl;
        break;

    case 0xFA: { /* LD A, (imm) */
        uint16_t addr = __gb_read(gb, gb->cpu_reg.pc++);
        addr |= __gb_read(gb, gb->cpu_reg.pc++) << 8;
        gb->cpu_reg.a = __gb_read(gb, addr);
        break;
    }

    case 0xFB: /* EI */
        gb->gb_ime = 1;
        break;

    case 0xFE: { /* CP imm */
        uint8_t temp_8 = __gb_read(gb, gb->cpu_reg.pc++);
        uint16_t temp_16 = gb->cpu_reg.a - temp_8;
        gb->cpu_reg.f_bits.z = ((temp_16 & 0xFF) == 0x00);
        gb->cpu_reg.f_bits.n = 1;
        gb->cpu_reg.f_bits.h =
            ((gb->cpu_reg.a ^ temp_8 ^ temp_16) & 0x10) ? 1 : 0;
        gb->cpu_reg.f_bits.c = (temp_16 & 0xFF00) ? 1 : 0;
        break;
    }

    case 0xFF: /* RST 0x0038 */
        __gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc >> 8);
        __gb_write(gb, --gb->cpu_reg.sp, gb->cpu_reg.pc & 0xFF);
        gb->cpu_reg.pc = 0x0038;
        break;

    default:
        (gb->gb_error)(gb, GB_INVALID_OPCODE, opcode);
    }

    /* DIV register timing */
    gb->counter.div_count += inst_cycles;

    if (gb->counter.div_count >= DIV_CYCLES) {
        gb->gb_reg.DIV++;
        gb->counter.div_count -= DIV_CYCLES;
    }

    /* Check serial transmission */
    if (gb->gb_reg.SC & SERIAL_SC_TX_START) {
        /* If new transfer, call TX function. */
        if (gb->counter.serial_count == 0 && gb->gb_serial_tx != NULL)
            (gb->gb_serial_tx)(gb, gb->gb_reg.SB);

        gb->counter.serial_count += inst_cycles;

        /* If it's time to receive byte, call RX function. */
        if (gb->counter.serial_count >= SERIAL_CYCLES) {
            /* If RX can be done, do it. */
            /* If RX failed, do not change SB if using external
             * clock, or set to 0xFF if using internal clock. */
            uint8_t rx;

            if (gb->gb_serial_rx != NULL &&
                (gb->gb_serial_rx(gb, &rx) == GB_SERIAL_RX_SUCCESS)) {
                gb->gb_reg.SB = rx;

                /* Inform game of serial TX/RX completion. */
                gb->gb_reg.SC &= 0x01;
                gb->gb_reg.IF |= SERIAL_INTR;
            } else if (gb->gb_reg.SC & SERIAL_SC_CLOCK_SRC) {
                /* If using internal clock, and console is not
                 * attached to any external peripheral, shifted
                 * bits are replaced with logic 1. */
                gb->gb_reg.SB = 0xFF;

                /* Inform game of serial TX/RX completion. */
                gb->gb_reg.SC &= 0x01;
                gb->gb_reg.IF |= SERIAL_INTR;
            } else {
                /* If using external clock, and console is not
                 * attached to any external peripheral, bits are
                 * not shifted, so SB is not modified. */
            }

            gb->counter.serial_count = 0;
        }
    }

    /* TIMA register timing */
    /* TODO: Change tac_enable to struct of TAC timer control bits. */
    if (gb->gb_reg.tac_enable) {
        static const uint_fast16_t TAC_CYCLES[4] = {1024, 16, 64, 256};

        gb->counter.tima_count += inst_cycles;

        while (gb->counter.tima_count >= TAC_CYCLES[gb->gb_reg.tac_rate]) {
            gb->counter.tima_count -= TAC_CYCLES[gb->gb_reg.tac_rate];

            if (++gb->gb_reg.TIMA == 0) {
                gb->gb_reg.IF |= TIMER_INTR;
                /* On overflow, set TMA to TIMA. */
                gb->gb_reg.TIMA = gb->gb_reg.TMA;
            }
        }
    }

    /* TODO Check behaviour of LCD during LCD power off state. */
    /* If LCD is off, don't update LCD state.
     */
    if ((gb->gb_reg.LCDC & LCDC_ENABLE) == 0)
        return;

    /* LCD Timing */
    gb->counter.lcd_count += inst_cycles;

    /* New Scanline */
    if (gb->counter.lcd_count > LCD_LINE_CYCLES) {
        gb->counter.lcd_count -= LCD_LINE_CYCLES;

        /* LYC Update */
        if (gb->gb_reg.LY == gb->gb_reg.LYC) {
            gb->gb_reg.STAT |= STAT_LYC_COINC;

            if (gb->gb_reg.STAT & STAT_LYC_INTR)
                gb->gb_reg.IF |= LCDC_INTR;
        } else
            gb->gb_reg.STAT &= 0xFB;

        /* Next line */
        gb->gb_reg.LY = (gb->gb_reg.LY + 1) % LCD_VERT_LINES;

        /* VBLANK Start */
        if (gb->gb_reg.LY == LCD_HEIGHT) {
            gb->lcd_mode = LCD_VBLANK;
            gb->gb_frame = 1;
            gb->gb_reg.IF |= VBLANK_INTR;

            if (gb->gb_reg.STAT & STAT_MODE_1_INTR)
                gb->gb_reg.IF |= LCDC_INTR;

#if ENABLE_LCD
            /* If frame skip is activated, check if we need to draw
             * the frame or skip it. */
            if (gb->direct.frame_skip) {
                gb->display.frame_skip_count = !gb->display.frame_skip_count;
            }

            /* If interlaced is activated, change which lines get
             * updated. Also, only update lines on frames that are
             * actually drawn when frame skip is enabled. */
            if (gb->direct.interlace &&
                (!gb->direct.frame_skip || gb->display.frame_skip_count)) {
                gb->display.interlace_count = !gb->display.interlace_count;
            }
#endif
        }
        /* Normal Line */
        else if (gb->gb_reg.LY < LCD_HEIGHT) {
            if (gb->gb_reg.LY == 0) {
                /* Clear Screen */
                gb->display.WY = gb->gb_reg.WY;
                gb->display.window_clear = 0;
            }

            gb->lcd_mode = LCD_HBLANK;

            if (gb->gb_reg.STAT & STAT_MODE_0_INTR)
                gb->gb_reg.IF |= LCDC_INTR;
        }
    }
    /* OAM access */
    else if (gb->lcd_mode == LCD_HBLANK &&
             gb->counter.lcd_count >= LCD_MODE_2_CYCLES) {
        gb->lcd_mode = LCD_SEARCH_OAM;

        if (gb->gb_reg.STAT & STAT_MODE_2_INTR)
            gb->gb_reg.IF |= LCDC_INTR;
    }
    /* Update LCD */
    else if (gb->lcd_mode == LCD_SEARCH_OAM &&
             gb->counter.lcd_count >= LCD_MODE_3_CYCLES) {
        gb->lcd_mode = LCD_TRANSFER;
#if ENABLE_LCD
        __gb_draw_line(gb);
#endif
    }
}

void gb_run_frame(struct gb_s *gb)
{
    gb->gb_frame = 0;

    while (!gb->gb_frame)
        __gb_step_cpu(gb);
}

/* Gets the size of the save file required for the ROM. */
uint_fast32_t gb_get_save_size(struct gb_s *gb)
{
    const uint_fast16_t ram_size_location = 0x0149;
    const uint_fast32_t ram_sizes[] = {0x00, 0x800, 0x2000, 0x8000, 0x20000};
    uint8_t ram_size = gb->gb_rom_read(gb, ram_size_location);
    return ram_sizes[ram_size];
}

/* Set the function used to handle serial transfer in the front-end. This is
 * optional.
 * gb_serial_transfer takes a byte to transmit and returns the received byte. If
 * no cable is connected to the console, return 0xFF.
 */
void gb_init_serial(struct gb_s *gb,
                    void (*gb_serial_tx)(struct gb_s *, const uint8_t),
                    gb_serial_rx_ret_t (*gb_serial_rx)(struct gb_s *,
                                                       uint8_t *))
{
    gb->gb_serial_tx = gb_serial_tx;
    gb->gb_serial_rx = gb_serial_rx;
}

uint8_t gb_color_hash(struct gb_s *gb)
{
#define ROM_TITLE_START_ADDR 0x0134
#define ROM_TITLE_END_ADDR 0x0143

    uint8_t x = 0;

    for (uint16_t i = ROM_TITLE_START_ADDR; i <= ROM_TITLE_END_ADDR; i++)
        x += gb->gb_rom_read(gb, i);

    return x;
}

/* Resets the context, and initializes startup values. */
void gb_reset(struct gb_s *gb)
{
    gb->gb_halt = 0;
    gb->gb_ime = 1;
    gb->gb_bios_enable = 0;
    gb->lcd_mode = LCD_HBLANK;

    /* Initialize MBC values. */
    gb->selected_rom_bank = 1;
    gb->cart_ram_bank = 0;
    gb->enable_cart_ram = 0;
    gb->cart_mode_select = 0;

    /* Initialize CPU registers as though a DMG. */
    gb->cpu_reg.af = 0x01B0;
    gb->cpu_reg.bc = 0x0013;
    gb->cpu_reg.de = 0x00D8;
    gb->cpu_reg.hl = 0x014D;
    gb->cpu_reg.sp = 0xFFFE;
    /* TODO: Add BIOS support. */
    gb->cpu_reg.pc = 0x0100;

    gb->counter.lcd_count = 0;
    gb->counter.div_count = 0;
    gb->counter.tima_count = 0;
    gb->counter.serial_count = 0;

    gb->gb_reg.TIMA = 0x00;
    gb->gb_reg.TMA = 0x00;
    gb->gb_reg.TAC = 0xF8;
    gb->gb_reg.DIV = 0xAC;

    gb->gb_reg.IF = 0xE1;

    gb->gb_reg.LCDC = 0x91;
    gb->gb_reg.SCY = 0x00;
    gb->gb_reg.SCX = 0x00;
    gb->gb_reg.LYC = 0x00;

    /* Appease valgrind for invalid reads and unconditional jumps. */
    gb->gb_reg.SC = 0x7E;
    gb->gb_reg.STAT = 0;
    gb->gb_reg.LY = 0;

    __gb_write(gb, 0xFF47, 0xFC); /* BGP */
    __gb_write(gb, 0xFF48, 0xFF); /* OBJP0 */
    __gb_write(gb, 0xFF49, 0x0F); /* OBJP1 */
    gb->gb_reg.WY = 0x00;
    gb->gb_reg.WX = 0x00;
    gb->gb_reg.IE = 0x00;

    gb->direct.joypad = 0xFF;
    gb->gb_reg.P1 = 0xCF;
}

/* Initialize the emulator context. gb_reset() is also called to initialize
 * the CPU.
 */
gb_init_error_t gb_init(
    struct gb_s *gb,
    uint8_t (*gb_rom_read)(struct gb_s *, const uint_fast32_t),
    uint8_t (*gb_cart_ram_read)(struct gb_s *, const uint_fast32_t),
    void (*gb_cart_ram_write)(struct gb_s *,
                              const uint_fast32_t,
                              const uint8_t),
    void (*gb_error)(struct gb_s *, const gb_error_t, const uint16_t),
    void *priv)
{
    const uint16_t mbc_location = 0x0147;
    const uint16_t bank_count_location = 0x0148;
    const uint16_t ram_size_location = 0x0149;

    /* Table for cartridge type (MBC). -1 if invalid.
     * TODO: MMM01 is untested.
     * TODO: MBC6 is untested.
     * TODO: MBC7 is unsupported.
     * TODO: POCKET CAMERA is unsupported.
     * TODO: BANDAI TAMA5 is unsupported.
     * TODO: HuC3 is unsupported.
     * TODO: HuC1 is unsupported.
     **/
    const uint8_t cart_mbc[] = {0,  1,  1,  1,  -1, 2, 2, -1, 0, 0,  -1,
                                0,  0,  0,  -1, 3,  3, 3, 3,  3, -1, -1,
                                -1, -1, -1, 5,  5,  5, 5, 5,  5, -1};
    const uint8_t cart_ram[] = {0, 0, 1, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0,
                                1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0};
    const uint16_t num_rom_banks[] = {
        2, 4, 8,  16, 32, 64, 128, 256, 512, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0,  0,  0,  0,  0,   0,   0,   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0,  0,  0,  0,  0,   0,   0,   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0,  0,  0,  0,  0,   0,   0,   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 72, 80, 96, 0,  0,   0,   0,   0, 0, 0, 0, 0, 0, 0};
    const uint8_t num_ram_banks[] = {0, 1, 1, 4, 16, 8};

    gb->gb_rom_read = gb_rom_read;
    gb->gb_cart_ram_read = gb_cart_ram_read;
    gb->gb_cart_ram_write = gb_cart_ram_write;
    gb->gb_error = gb_error;
    gb->direct.priv = priv;
    
    /* Initialize serial transfer function to NULL. If the front-end does
     * not provide serial support, it will emulate no cable connected
     * automatically.
     */
    gb->gb_serial_tx = NULL;
    gb->gb_serial_rx = NULL;

    /* Check valid ROM using checksum value. */
    {
        uint8_t x = 0;

        for (uint16_t i = 0x0134; i <= 0x014C; i++)
            x = x - gb->gb_rom_read(gb, i) - 1;

        if (x != gb->gb_rom_read(gb, ROM_HEADER_CHECKSUM_LOC))
            return GB_INIT_INVALID_CHECKSUM;
    }

    /* Check if cartridge type is supported, and set MBC type. */
    {
        const uint8_t mbc_value = gb->gb_rom_read(gb, mbc_location);

        if (mbc_value > sizeof(cart_mbc) - 1 ||
            (gb->mbc = cart_mbc[gb->gb_rom_read(gb, mbc_location)]) == 255u)
            return GB_INIT_CARTRIDGE_UNSUPPORTED;
    }

    gb->cart_ram = cart_ram[gb->gb_rom_read(gb, mbc_location)];
    gb->num_rom_banks = num_rom_banks[gb->gb_rom_read(gb, bank_count_location)];
    gb->num_ram_banks = num_ram_banks[gb->gb_rom_read(gb, ram_size_location)];

    gb->display.lcd_draw_line = NULL;

    gb_reset(gb);

    return GB_INIT_NO_ERROR;
}

/* Return the title of ROM.
 *
 * \param gb		Initialized context.
 * \param title_str	Allocated string at least 16 characters.
 * \returns		Pointer to start of string, null terminated.
 */
const char *gb_get_rom_name(struct gb_s *gb, char *title_str)
{
    uint_fast16_t title_loc = 0x134;
    /* End of title may be 0x13E for newer games. */
    const uint_fast16_t title_end = 0x143;
    const char *title_start = title_str;

    for (; title_loc <= title_end; title_loc++) {
        const char title_char = gb->gb_rom_read(gb, title_loc);

        if (title_char >= ' ' && title_char <= '_') {
            *title_str = title_char;
            title_str++;
        } else
            break;
    }

    *title_str = '\0';
    return title_start;
}

#if ENABLE_LCD
void gb_init_lcd(struct gb_s *gb,
                 void (*lcd_draw_line)(struct gb_s *gb,
                                       const uint8_t *pixels,
                                       const uint_fast8_t line))
{
    gb->display.lcd_draw_line = lcd_draw_line;

    gb->direct.interlace = 0;
    gb->display.interlace_count = 0;
    gb->direct.frame_skip = 0;
    gb->display.frame_skip_count = 0;

    gb->display.window_clear = 0;
    gb->display.WY = 0;
}
#endif
