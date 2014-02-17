// Copyright (C) 2013  Matthijs Kooijman <matthijs@stdin.nl>
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//
// Compile using:
//   avr-g++ -mmcu=attiny13 -Os backpack.c -o backpack.elf
//   avr-objcopy -O ihex backpack.elf backpack.hex
//
// To flash:
//   avrdude -c stk500 -p attiny13 -P /dev/ttyUSB0 -U flash:w:backpack.hex
//
// Fuse settings are 0xfb and 0x21:
//   avrdude -c stk500 -p attiny13 -P /dev/ttyUSB0 -U hfuse:w:0xfb:m -U lfuse:w:0x21:m
//
// Note that avr-libc 1.8.0 does not provide "tiny-stack" versions of
// the crt*.o libraries but newer (suspectedly 4.7.1 and above) gcc
// versions do expect those to exist (causing "ld: cannot find
// crttn13a.o: No such file or directory"). As a workaround, suggested
// by https://savannah.nongnu.org/bugs/?35407#comment0 you can run:
//   # ln -s /usr/lib/avr/lib/avr25 /usr/lib/avr/lib/avr25/tiny-stack
// to make gcc look in the right place.
//
// TODO:
//  - In theory, a reset could happen when the main loop is processing
//    something. Now, the main loop will happily overwrite the state set
//    by the reset detection, but this should somehow be prevented
//    (perhaps set just a reset flag in the OVF ISR and let the main
//    loop do all of the other setup?)

// Implementation overview
// -----------------------
// The code in this file is divided into two main sections: The
// interrupt routines, who handle the low level of the protocol (sending
// bits and bytes) and the main loop, which handles the high level of
// the protocol (giving meaning to the bytes sent).
//
// Normal flow is that the ISRs detect a reset signal and prepare to
// receive the address byte. After the byte was received the ISRs set
// action to ACTION_STALL, which is the signal for the mainloop to
// process the byte. While the mainloop is processing, the ISRs make
// sure that it sends stall bits whenever the master initiates a bit.
// The mainloop decides wether the next bytes needs to be sent or
// received and sets the action to ACTION_READY to let the ISRs continue
// sending the ready and ack bits and then send or receive another byte.
//
// After every byte sent or received, the ISRs set action to
// ACTION_STALL again, and the mainloop decides the next action to take
// based on the data and its state variable.
//
//
// Within the ISRs, the action variable indicates both how far in the
// bit sequence it is, as well as the actions to take for the current
// action (e.g., pull the line low, or sample the line).
//
// The action byte effectively forms a state machine, that looks like
// this:
//                          ↓
//    +—————————————————→ READY ←-----+
//    |                   |   |       |
//    |                   ↓   ↓       |
//    |                ACK1   NACK1   |
//    |                 ↓       ↓     |
//    |      IDLE ←-+-ACK2-+  NACK2   |
//    |             ↓      ↓    |     |
//    |  reset → RECEIVE  SEND ←+     |
//    |          |  |      | |        |
//    +——————————+  +———+——+ +————————+
//     parity error     ↓     sent error code
//                    STALL
//
//
//
// (Note that the current implementation allows going from NACK2 to IDLE
// and RECEIVE as well, but this is expected to change, so the above
// drawing reflects the new but not yet implemented situation)
//
// Every bit starts with a falling edge, detected by the INT0 interval.
// Then, either:
//  - Nothing is done (e.g., send a 1)
//  - The line is pulled low (e.g., send a 0)
//  - The line is sampled (e.g., receive a bit, or send a 1 and check
//    for collisions)
//
//  When pulling the line low, the timer compare B interrupt is enabled
//  to released the line again after the proper time.
//
//  When the line needs to be sampled, the timer compare A interrupt is
//  enabled to sample the time at the proper time.
//
//  If the line does not need to be sampled, the ISRs move the action
//  variable to its next value right away, preparing for the next bit.
//  If the line does need to be sampled, this is delayed until after the
//  sampling (since the next action might depend on the data sampled).
//
//  The timer overflow interrupt is always enabled after a falling edge,
//  to detect the reset signal.

// 4.8Mhz oscillator with CKDIV8 fuse set
#define F_CPU (4800000/8)

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/cpufunc.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <stdbool.h>
#include "protocol.h"

// Workarounds for typos in iotn13a.h, see
// https://savannah.nongnu.org/bugs/?40567
#if defined(BPDS) && !defined(BODS)
#define BODS BPDS
#define BODSE BPDSE
#endif
#if defined(EEWE) && !defined(EEPE)
#define EEPE EEWE
#define EEMPE EEMWE
#endif

// Debug macro, generate a short pulse on the given pin (B0, B2 or B4)
#if defined(DEBUG)
#define pulse(pin) do { \
    PORTB &= ~(1 << pin); \
    {uint8_t i; for (i=0;i<10;++i) _NOP();} \
    PORTB |= (1 << pin); \
    } while(0)
#endif

// Offset of the unique ID within the EEPROM
uint8_t const UNIQUE_ID_OFFSET = 3;

// Protocol timings, in timer0 clock cycles (which runs at F_CPU / 8)
#define US_TO_CLOCKS(x) (unsigned long)(x * F_CPU / 8 / 1000000)
#define RESET_SAMPLE US_TO_CLOCKS(1800)
#define DATA_WRITE US_TO_CLOCKS(600)
#define DATA_SAMPLE US_TO_CLOCKS(300)

// Values for the action variable - low level protocol state
enum {
    // These are the actual action values, which define the action to
    // take for the current bit

    AV_IDLE = 0x0,
    AV_SEND = 0x1,
    AV_RECEIVE = 0x2,
    AV_ACK1 = 0x3,
    AV_ACK2 = 0x4,
    AV_NACK1 = 0x5,
    AV_NACK2 = 0x6,
    AV_READY = 0x7,
    AV_STALL = 0x8,

    // Mask for the action global to get one of the above values
    ACTION_MASK = 0xf,

    // These are action flags that can be combined with the above values

    // This action needs to sample the bit value
    AF_SAMPLE = 0x80,
    // This action needs to pull the line low
    AF_LINE_LOW = 0x40,
    // When FLAG_MUTE is set, the AF_SAMPLE and AF_LINE_LOW bits should
    // be ignored for this bit
    AF_MUTE = 0x20,

    // These are the complete values (action value plus any relevant
    // action flags). The action global variable is always set to one of
    // these values.
    ACTION_IDLE = AV_IDLE,
    ACTION_STALL = AV_STALL | AF_LINE_LOW,
    ACTION_SEND = AV_SEND,
    ACTION_SEND_HIGH = AV_SEND | AF_MUTE,
    ACTION_SEND_LOW = AV_SEND | AF_LINE_LOW | AF_MUTE,
    ACTION_SEND_HIGH_CHECK_COLLISION = AV_SEND | AF_SAMPLE | AF_MUTE,
    ACTION_RECEIVE = AV_RECEIVE | AF_SAMPLE,
    ACTION_ACK1 = AV_ACK1 | AF_LINE_LOW | AF_MUTE,
    ACTION_ACK2 = AV_ACK2 | AF_MUTE,
    ACTION_NACK1 = AV_NACK1 | AF_MUTE,
    ACTION_NACK2 = AV_NACK2 | AF_LINE_LOW | AF_MUTE,
    ACTION_READY = AV_READY | AF_SAMPLE,
};

// Values for the state variable - high level protocol state
enum {
    // Idle - Waiting for the next reset to participate (again)
    STATE_IDLE,
    // Bus was reset, receiving the address byte (or broadcast command)
    STATE_RECEIVE_ADDRESS,
    // BC_CMD_ENUMERATE received, bus enumeration in progress
    STATE_ENUMERATE,
    // We are adressed, receiving targeted command
    STATE_RECEIVE_COMMAND,
    // CMD_READ_EEPROM received, now receiving read address
    STATE_READ_EEPROM_RECEIVE_ADDR,
    // CMD_READ_EEPROM and read address received, now reading
    STATE_READ_EEPROM_SEND_DATA,
    // CMD_WRITE_EEPROM received, now receiving write address
    STATE_WRITE_EEPROM_RECEIVE_ADDR,
    // CMD_WRITE_EEPROM and write address received, now writing
    STATE_WRITE_EEPROM_RECEIVE_DATA,
    // CMD_WRITE_EEPROM or CMD_WRITE_EEPROM address overflowed the
    // EEPROM
    STATE_READ_EEPROM_OVERFLOW,
};

// Values for the flags variable - various flags
enum {
    // When this flag is set, this slave will no longer participate on
    // the bus, but it will still keep its state synchronized. This is
    // used during bus enumeration, when this slave has "lost" a
    // conflict and needs to wait out the current enumeration round
    // before trying to send its id again.
    FLAG_MUTE = 1,
    // The (odd) parity bit for all bits sent or received so far
    FLAG_PARITY = 2,
    // If this flag is set, the slave has completed bus enumeration
    // succesfully and the bus address in bus_addr is valid.
    FLAG_ENUMERATED = 4,
    // If this flag is set, during any high bits sent the slave will
    // check the bus for collision (e.g., when another slave is sending
    // a low bit). If collision is detected, FLAG_MUTE is set.
    FLAG_CHECK_COLLISION = 8,
    // After the ACK bit:
    //  - if FLAG_IDLE is set, switch to idle and drop off the bus
    //  - if FLAG_SEND is set, switch to sending a byte
    //  - otherwise, switch to receiving a byte
    FLAG_SEND = 16,
    FLAG_IDLE = 32,
    // After sending the ACK/NACK bit, clear FLAG_MUTE and
    // FLAG_CLEAR_MUTE
    FLAG_CLEAR_MUTE = 64,
};

// Putting global variables in fixed registers saves a lot of
// instructions for loading and storing their values to memory.
// Additionally, if _all_ globals are in registers (or declared with
// __attribute__ ((section (".noinit")))), gcc will omit the bss clear
// loop (saving another 16 bytes). Only call-used registers are
// available, so that's effectively r2-r17. Using all of those will
// probably kill the compiler, though.

// This is the byte being sent or received. Should be initialized by the
// mainloop when sending a byte, it is filled by the ISRs when receiving
// a byte.
register uint8_t byte_buf asm("r2");

// The bit currently being sent or received during ACTION_SEND or
// ACTION_RECEIVE (bitmask with exactly 1 bit enabled). If next_bit is
// 0, the parity bit should be sent or received.
register uint8_t next_bit asm("r3");

// The address of the next EEPROM byte to send.
register uint8_t next_byte asm("r4");

// The bus address of this slave (only valid when FLAG_ENUMERATED is
// set).
register uint8_t bus_addr asm("r5");

// Various flags, used by both the ISRs and the mainloop.
register uint8_t flags asm("r6");

// The action to take for the next or current bit
register uint8_t action asm("r7");

// The high level protocol state. Only valid when action != ACTION_IDLE.
register uint8_t state asm("r8");

// When this is set, the next ack/nack bit will be a nack, followed by
// this error code
register uint8_t err_code asm("r9");

// Flags related to the WDT. Flags are set when the relevant event
// occurs, and when all relevant flags are set, the watchdog timer is
// reset and these flags are cleared.
register uint8_t wdt_flags asm("r10");

enum {
    // Set when a byte was processed by the mainloop
    WDT_PROGRESS = 1,
    // Set when the line was observed to be high
    WDT_LINE_HIGH = 2,
};


// Register that is used by TIM0_COMPA_vect() and
// TIM0_COMPA_vect_do_work() to pass on the sampled value. This needs to
// happen in a global variable, since we can't clobber any other
// registers in an ISR
register uint8_t sample_val asm("r16");

// Register that contains the reset value for TCNT0. By putting this
// inside a register, the naked INT0 ISR can write it to TCNT0
// immediately, without having to mess with freeing up a register and
// loading the constant into it.
register uint8_t tcnt0_init asm("r17");

// Use a watchdog timeout of 32ms. The longest period the ISRs should be
// busy without letting the mainloop work, should be 28 bits (4
// handshaking bits, 8 databits with a parity error, another 4
// handshaking bits including a NAK, another 8 databits containing a
// parity error code and 4 handshaking bits including an ack). With
// maximum inter-bit timing (1500μs), this should take 42ms. So,
// timing out after 64ms seems reasonable.
static uint8_t const WDTCR_SETTINGS = (1 << WDP1);

// Enable the watchdog.
// Unlike the wdt_disable function in avr-libc, this does not disable
// interrupts (to save a few instructions), but this means they can only
// be called when interrupts are already disabled.
static inline void wdt_on() {
    // Only act when the watchdog is disabled, to prevent continuously
    // resetting the timer.
    if (!(WDTCR & (1 << WDE))) {
        // Reset the timer, to prevent it from triggering right away
        wdt_reset();

        // Even though the datasheet says you can't change the prescaler
        // bits without doing the WDCE magic dance, turns you you can
        // (even while setting WDCE!), so be sure to include them
        // everywhere we write to WDTCR. In practice, it seems we could
        // just suffice with the second line, without doing the dance,
        // but let's be safe just to be sure.
        WDTCR = (1 << WDCE) | (1 << WDE) | WDTCR_SETTINGS;
        WDTCR = (1 << WDE) | WDTCR_SETTINGS;
    }
}

// Disable the watchdog.
// Unlike the wdt_disable function in avr-libc, this does not disable
// interrupts (to save a few instructions), but this means they can only
// be called when interrupts are already disabled.
static inline void wdt_off() {
    // Even though the datasheet says you can't change the prescaler
    // bits without doing the WDCE magic dance, turns you you can (even
    // while setting WDCE!), so be sure to include them everywhere we
    // write to WDTCR (except when actually disabling WDE, since writing
    // 0 is potentially more efficient).
    WDTCR = (1 << WDCE) | (1 << WDE) | WDTCR_SETTINGS;
    WDTCR = 0;
}

// This is the naked ISR that is called on INT0 interrupts. It
// immediately clears the TCNT0 register so the time from the interrupt
// to the timer restart (and thus timer compare match) becomes
// deterministic. If we would do this in a normal ISR, there would be a
// delay depending on the length of the prologue generated by the
// compiler (which would again depend on the number of registers used,
// and thus saved, in the ISR).
ISR(INT0_vect, ISR_NAKED) {
    // Reset the TCNT0 register.
    asm("out %0, %1" : : "I"(_SFR_IO_ADDR(TCNT0)), "r"(tcnt0_init));

    // Jump to the function that will do the real work. Since that is
    // declared as an ISR, it will also properly do all the register
    // saving required.
    asm("rjmp __vector_bit_start");
}

// Handle the start of a bit. Called by the INT0 ISR after resetting
// timer.
// Declared as an ISR so it will properly save all registers, to allow
// calling it from the real ISR. Name starts with __vector to fool gcc
// into not giving the "appears to be a misspelled signal handler"
// warning.
ISR(__vector_bit_start)
{
    // Note that the falling edge interrupt is _always_ enabled, so if a
    // falling edge occurs before the previous bit period is processed (e.g.
    // before the timer interrupt fired), then the previous period is
    // effectively ignored. This can only happen when a device violates the
    // protocol.

    // Start with a clean slate, in case there is a falling edge before
    // compare A or B interrupts. During normal operation, this should
    // never happen, but if we don't do this we could deadlock (pulling
    // the bus forever low) if some high-speed signal is offered on the
    // bus. We disable the two compare timers and release the bus in
    // case it was not released (note when a high-speed signal is on the
    // bus, the INT0 interrupt could trigger with the bus low somehow).
    TIMSK0 = (1 << TOIE0);
    DDRB &= ~(1 << PINB1);

    // Don't bother doing either of these when we're muted
    if ((flags & FLAG_MUTE) && (action & AF_MUTE))
        action &= ~(AF_LINE_LOW | AF_SAMPLE);

    if ((action & AF_LINE_LOW)) {
        // Pull the line low and enable a timer to release it again
        DDRB |= (1 << PINB1);
        TIMSK0 |=  (1 << OCIE0B);
    }

    // Clear any interrupt flags that might have been set while the
    // timer interrupts were disabled
    TIFR0 = (1 << OCF0B) | (1 << OCF0A) | (1 << TOV0);

    // Clear the INT0 flag. If the line went high again already, our
    // pulling it low would trigger the INT0 interrupt for a second
    // time, so ignore it when that happens.  Ideally, we pull the line
    // low before the master releases it, but the protocol doesn't
    // strictly require this.
    GIFR = (1 << INTF0);

    // If we were powered-down, we'll have been set to a
    // level-triggered interrupt instead of an edge-triggered one,
    // since a edge-triggered one can wake us up. We'll need to set
    // it to edge-triggered to prevent being flooded with
    // interrupts.
    // Make INT0 falling edge-triggered (note that this assumes
    // ISC00 is not set)
    MCUCR |= (1<<ISC01);
    set_sleep_mode(SLEEP_MODE_IDLE);

    // Normally, the mainloop checks the line and sets this flag.
    // However, it can happen that the mainloop goes to sleep when the
    // line is low, the MCU is woken up because of a falling edge and
    // the mainloop continues running when the line is low again. The
    // line will have been high, but the mainloop never sees that. We'll
    // help them a bit by setting the flag here.
    wdt_flags |= WDT_LINE_HIGH;

    if (action & AF_SAMPLE) {
        // Schedule a timer to sample the line
        TIMSK0 |=  (1 << OCIE0A);
    } else {
        // If the only thing that needs to happen in the timer handler
        // is advancing to the next action, we might as well do it right
        // away (note that we can usually _not_ do it right away). We
        // don't want duplicate code, so we pretend the timer interrupt
        // happens directly.
        // We use an assembly call here, because when the compiler sees
        // a regular call, this causes this ISR (the caller) to save
        // _all_ call-clobbered registers, in case the called function
        // might actually clobber them. In this case, this is completely
        // bogus, since we're calling a signal handler which will save
        // everything it touches already (which also means that hiding
        // this function call from the compiler is safe).
        // Note that the called ISR returns with reti instead of the
        // regulat ret, causing interrupts to be enabled in the process.
        // Since we're the last instruction in the function, this
        // shouldn't be a problem (and shouldn't happen in practice
        // anyway).
        // Finally, note that we don't call the real ISR, but the
        // function that does the work, skipping a few instructions and
        // skipping the bus sampling.
        asm("rcall __vector_sample");
        // Interrupts are enabled here!
    }
}

ISR(TIM0_COMPB_vect, ISR_NAKED)
{
    // Release bus
    asm("cbi %0, %1"   : : "I"(_SFR_IO_ADDR(DDRB)), "I"(PINB1));
    asm("reti");
}

// This is the naked ISR that is called on TIM0_COMPA interrupts. It
// immediately takes a sample of the bus. It is naked to minimize the
// time until the sample is taken.
ISR(TIM0_COMPA_vect, ISR_NAKED)
{
    // Sample bus (and all other pins at the same time). Use a global
    // register variable to store the value, so we don't need to save
    // some register's value and restore it below...
    asm("in %0, %1" : "=r"(sample_val) : "I"(_SFR_IO_ADDR(PINB)));

    // Jump to TIM0_COMPA_vect_do_work, which will handle the real saving of
    // registers
    asm("rjmp __vector_sample");
}

// Declared as an ISR so it will properly save all registers, allowing
// to call or jump to it from real ISRs. Name starts with __vector to
// fool gcc into not giving the "appears to be a misspelled signal
// handler" warning.
ISR(__vector_sample)
{
    switch (action & ACTION_MASK) {
    case AV_RECEIVE:
        // Read and store bit value
        if (sample_val & (1 << PINB1)) {
            // When reading the parity bit, next_bit is 0 and this is a
            // no-op
            byte_buf |= next_bit;
            // Toggle the parity flag on every 1 received, including the
            // parity bit
            flags ^= FLAG_PARITY;
        }

        if (next_bit) {
            next_bit >>= 1;
        } else {
            // Full byte and parity bit received
            if (flags & FLAG_PARITY) {
                // Parity is ok, let the mainloop decide what to do next
                action = ACTION_STALL;
            } else {
                // Parity is not ok, skip the STALL state and send a
                // NACK and error code
                action = ACTION_READY;
                err_code = ERR_PARITY;
            }
        }
        break;
    case AV_SEND:
        if ((action & AF_SAMPLE) && !(sample_val & (1 << PINB1))) {
            // We're sending our address, but are not currently pulling the
            // line low. Check if the line is actually high. If not, someone
            // else is pulling the line low, so we drop out of the current
            // address sending round.
            flags |= FLAG_MUTE;
        }

        if (!next_bit) {
            // Just sent the parity bit
            if (err_code != ERR_OK) {
                // We just sent an error code, so skip the stall stage
                action = ACTION_READY;
                // Clear the error code, to prevent sending a nack for it
                err_code = ERR_OK;
                // Switch to idle after the ack bit
                flags |= FLAG_IDLE;
            } else {
                // Byte was a normal byte, let the mainloop decide what
                // to do next
                action = ACTION_STALL;
            }
            break;
        }

        // Send next bit, or parity bit
        next_bit >>= 1;

        bool val;
prepare_next_bit:
        if (next_bit) {
            // Send the next bit
            val = (byte_buf & next_bit);
        } else {
            // next_bit == 0 means to send the parity bit
            val = !(flags & FLAG_PARITY);
        }

        if (!val) {
            // Pull the line low
            action = ACTION_SEND_LOW;
        } else if (flags & FLAG_CHECK_COLLISION) {
            // Leave the line high, but check for collision
            action = ACTION_SEND_HIGH_CHECK_COLLISION;
            flags ^= FLAG_PARITY;
        } else {
            // Just leave the line high
            action = ACTION_SEND_HIGH;
            flags ^= FLAG_PARITY;
        }
        break;
    case AV_ACK1:
        action = ACTION_ACK2;
        break;
    case AV_NACK1:
        action = ACTION_NACK2;
        break;
    case AV_NACK2:
        // Send an error byte
        goto prepare_next_bit;
    case AV_ACK2:
        if (flags & FLAG_IDLE) {
            action = ACTION_IDLE;
            break;
        }

        // Clear FLAG_MUTE when requested
        if (flags & FLAG_CLEAR_MUTE)
            flags &= ~(FLAG_MUTE | FLAG_CLEAR_MUTE);

        // Decide upon the next action
        if (flags & FLAG_SEND) {
            // Set up the first bit
            goto prepare_next_bit;
        } else {
            action = ACTION_RECEIVE;
            byte_buf = 0;
        }

        break;

    case AV_READY:
        // Sample the line to see if anyone else is perhaps stalling the
        // bus. If so, keep trying to send our ready bit until everyone
        // is ready.
        if (!(sample_val & (1 << PINB1)))
            break;

        // Prepare for sending or receiving the next byte
        flags &= ~(FLAG_PARITY);
        next_bit = 0x80;

        if (err_code != ERR_OK) {
            action = ACTION_NACK1;
            byte_buf = err_code;
        } else {
            action = ACTION_ACK1;
            // byte_buf is already set, or will be cleared after ACK2
        }

        break;
    }
}

ISR(TIM0_OVF_vect)
{
    uint8_t val = PINB & (1 << PINB1);

    // Disable all timer interrupts
    TIMSK0 = 0;

    /* If a falling edge interrupt was triggered between the timer
     * overflow and us sampling the line, then return and handle that
     * instead and ignore the timer. */
    if (GIFR & (1 << INTF0))
        return;

    if (val) {
        // Bus has gone high. Since there hasn't been an INT0 in the
        // meantime, so more time has passed than is allowed between two
        // bits. This means the transaction is finished and the bus is
        // idle again.
        action = ACTION_IDLE;
    } else {
        // Bus is low and the flag for INT0 is not set means the bus is
        // _still_ low. We have to check INTF0 to prevent a race
        // condition where the bus has been high and just goes low at
        // the same time the timer overflows, making it otherwise look
        // like the bus has been low all the time.
        // Since we're sure the bus is still low, this is a reset pulse
        // (regardless of what state we were in previously!)
        state = STATE_RECEIVE_ADDRESS;
        action = ACTION_RECEIVE;
        err_code = ERR_OK;
        // These are normally initialized after sending the ack/nack
        // bit, but we're skipping that after a reset.
        byte_buf = 0;
        next_bit = 0x80;

        // Clear all flags, except for the enumeration status
        flags &= FLAG_ENUMERATED;
    }
}

/* Don't use avr-libc's eeprom_read/update_byte functions, since those
 * produce significantly bigger code (partly because they use 16-bit
 * addresses, partly for lack of lto probably. */
void EEPROM_write(uint8_t ucAddress, uint8_t ucData)
{
      /* Wait for completion of previous write */
    while(EECR & (1<<EEPE));
    /* Set Programming mode */
    EECR = (0<<EEPM1)|(0>>EEPM0);
    /* Set up address and data registers */
    EEARL = ucAddress;
    EEDR = ucData;
    /* Write logical one to EEMPE */
    EECR |= (1<<EEMPE);
    /* Start eeprom write by setting EEPE */
    EECR |= (1<<EEPE);
}

uint8_t EEPROM_read(uint8_t ucAddress)
{
    /* Wait for completion of previous write */
    while(EECR & (1<<EEPE));
    /* Set up address register */
    EEARL = ucAddress;
    /* Start eeprom read by writing EERE */
    EECR |= (1<<EERE);
    /* Return data from data register */
    return EEDR;
}

#if (__GNUC__ < 4 || (__GNUC__ == 4 && __GNUC_MINOR__ < 8))
// On GCC < 4.8, there is a bug that can cause writes to global register
// variables be dropped in a function that never returns (e.g., main).
// To prevent triggering this bug, don't inline the setup and loop
// functions, which could cause such writes to show up in main. This
// adds a few bytes of useless program code, but that pales in
// comparison with the bytes saved by using global register variables.
// http://gcc.gnu.org/bugzilla/show_bug.cgi?id=51447
void __attribute__((noinline)) setup(void);
void __attribute__((noinline)) loop(void);
#endif

void setup(void)
{
    action = ACTION_IDLE;
    flags &= FLAG_ENUMERATED;
    // Don't clear FLAG_ENUMERATED on a watchdog reset, so we don't
    // forget our assigned address in this case.
    if (!(MCUSR & (1 << WDRF)))
        flags &= ~FLAG_ENUMERATED;

    // Enable pullups on all ports except the bus pin (to save power)
    PORTB = ~(1 << PINB1);

    // Shut off power to the ADC
    #if defined(PRR)
    PRR = (1 << PRADC);
    #endif

    // Set ports to output for debug
    #if defined(DEBUG)
    DDRB = (1 << PINB0) | (1 << PINB2) | (1 << PINB4);

    // Bring PINB0 low after a watchdog reset
    if ((MCUSR & (1 << WDRF)))
        PORTB &= ~(1 << PINB0);
    #endif

    // On an INT0 interrupt, the counter is reset to tcnt_init, so it
    // overflows after RESET_SAMPLE ticks. OCR0A and OCR0B are set so
    // their interrupts fire after DATA_SAMPLE and DATA_RELEASE ticks
    // respectively
    tcnt0_init = (0xff - RESET_SAMPLE);
    OCR0B = tcnt0_init + DATA_WRITE;
    OCR0A = tcnt0_init + DATA_SAMPLE;

    // Enable INT0 interrupt
    GIMSK=(1<<INT0);

    // Set timer to /8 prescaler, which, together with the CLKDIV8 fuse,
    // makes 4.8Mhz / 8 / 8 = 75kHz
    TCCR0B=(1<<CS01);

    // Clear reset status flags (this is needed, since after a watchdog
    // reset, WDRF is set which forces the watchdog to be on).
    MCUSR = 0;

    // Enable sleeping, and set INT0 to falling-edge.
    //
    // The datasheet recommends setting the enable sleep bit at the
    // moment you actually want to sleep, "to prevent accidentally
    // putting the system in sleep mode", but why would we use the sleep
    // instruction if we didn't want to sleep? Silly bit.
    MCUCR = (1 << SE) | (1 << ISC01);

    sei();
}

void loop(void)
{
    if (action == ACTION_STALL) {
        // Done receiving or sending a byte. Interrupts handlers will
        // send STALL bits while we're processing. Note that the
        // ACK/NACK bit for the previous byte is _not_ sent yet.
        //
        // Here, we must process the received byte and/or prepare the
        // next byte, based on the state variable. Once done, we should:
        //  - Set err_code to send a NACK, the error code and switch to
        //    idle.  Otherwise, an ACK will be sent.
        //  - Set FLAG_SEND to send a byte, set FLAG_IDLE to go into
        //    idle after the ACK bit, or leave both unset to receive a
        //    byte.
        //  - Set action to ACTION_READY (normally) or ACTION_IDLE (when
        //    no ready or ACK/NACK bits must be sent).
        switch(state) {
        case STATE_RECEIVE_ADDRESS:
            // Read the first byte after a reset, which is either a
            // broadcast command or a bus address
            if (byte_buf == BC_CMD_ENUMERATE) {
                state = STATE_ENUMERATE;
                flags |= FLAG_CHECK_COLLISION;
                flags |= FLAG_SEND;
                flags &= ~FLAG_ENUMERATED;
                next_byte = UNIQUE_ID_OFFSET;
                bus_addr = 0;
                // Don't change out of STALL, let the next iteration
                // prepare the first byte
            } else if ((flags & FLAG_ENUMERATED) && byte_buf == bus_addr) {
                // We're addressed, find out what the master wants
                state = STATE_RECEIVE_COMMAND;
                action = ACTION_READY;
            } else {
                // We're not addressed, stop paying attention. Note that
                // this does _not_ send the ready bit and ACK bit.
                action = ACTION_IDLE;
            }
            break;
        case STATE_RECEIVE_COMMAND:
            // We were addressed and the master has just sent us a
            // command
            switch (byte_buf) {
                case CMD_READ_EEPROM:
                    state = STATE_READ_EEPROM_RECEIVE_ADDR;
                    action = ACTION_READY;
                    break;
                case CMD_WRITE_EEPROM:
                    state = STATE_WRITE_EEPROM_RECEIVE_ADDR;
                    action = ACTION_READY;
                    break;
                default:
                    // Unknown command
                    err_code = ERR_UNKNOWN_COMMAND;
                    action = ACTION_READY;
                    break;
            }
            break;
        case STATE_READ_EEPROM_RECEIVE_ADDR:
            // We're running CMD_READ_EEPROM and just received the
            // EEPROM addres to read from
            next_byte = byte_buf;
            flags |= FLAG_SEND;
            state = STATE_READ_EEPROM_SEND_DATA;
            if (next_byte > E2END) {
                err_code = ERR_READ_EEPROM_INVALID_ADDRESS;
                action = ACTION_READY;
                break;
            }
            // Don't change out of STALL, let the next iteration
            // prepare the first byte
            break;
        case STATE_WRITE_EEPROM_RECEIVE_ADDR:
            // We're running CMD_WRITE_EEPROM and just received the
            // EEPROM address to write
            next_byte = byte_buf;
            state = STATE_WRITE_EEPROM_RECEIVE_DATA;
            action = ACTION_READY;
            if (next_byte > E2END)
                err_code = ERR_READ_EEPROM_INVALID_ADDRESS;
            break;
        case STATE_WRITE_EEPROM_RECEIVE_DATA:
            if (next_byte > E2END) {
                err_code = ERR_WRITE_EEPROM_INVALID_ADDRESS;
            } else if (byte_buf != EEPROM_read(next_byte)) {
                // Byte was actually changed. Write it, unless it is a
                // read-only byte
                if (next_byte >= UNIQUE_ID_OFFSET && next_byte < UNIQUE_ID_OFFSET + UNIQUE_ID_LENGTH) {
                    err_code = ERR_WRITE_EEPROM_READ_ONLY;
                } else {
                    EEPROM_write(next_byte, byte_buf);
                    // Check if the write completed succesfully
                    if (byte_buf != EEPROM_read(next_byte))
                        err_code = ERR_WRITE_EEPROM_FAILED;
                }
            }
            next_byte++;
            action = ACTION_READY;
            break;
        case STATE_ENUMERATE:
            if (next_byte == UNIQUE_ID_OFFSET + UNIQUE_ID_LENGTH) {
                // Entire address sent
                if (flags & FLAG_MUTE) {
                    // Another device had a lower id, so try again
                    // on the next round
                    next_byte = UNIQUE_ID_OFFSET;
                    bus_addr++;
                    // Stop muting _after_ sending the ack/nack bit for
                    // the current (last) byte
                    flags |= FLAG_CLEAR_MUTE;
                } else {
                    // We have the lowest id sent during this round,
                    // so claim the current bus address and stop
                    // paying attention
                    state = STATE_IDLE;
                    flags |= FLAG_IDLE;
                    flags |= FLAG_ENUMERATED;
                    action = ACTION_READY;
                    break;
                }
            }
            // Read and send next id byte (but don't bother while
            // we're muted)
            if (!(flags & FLAG_MUTE) || (flags & FLAG_CLEAR_MUTE))
                byte_buf = EEPROM_read(next_byte);
            next_byte++;
            action = ACTION_READY;
            break;
        case STATE_READ_EEPROM_SEND_DATA:
            if (next_byte > E2END) {
                // Just send the last byte again, which will then be
                // NACKed below (but we still have to ACK the previous
                // byte first).
                state = STATE_READ_EEPROM_OVERFLOW;
            } else {
                // Read and send next EEPROM byte
                byte_buf = EEPROM_read(next_byte);
                next_byte++;
            }
            action = ACTION_READY;
            break;
        case STATE_READ_EEPROM_OVERFLOW:
            // We just send a dummy value for an overflowed read. NACK
            // this byte and send an error code
            err_code = ERR_READ_EEPROM_INVALID_ADDRESS;
            action = ACTION_READY;
        }
        // We made some progress
        wdt_flags |= WDT_PROGRESS;
    }
    if (PINB & (1 << PINB1))
        wdt_flags |= WDT_LINE_HIGH;

    // Reset the watchdog timer when:
    //  - The line has been high since the last wdt reset, and
    //  - We're idle, or we've processed a byte since that last wdt reset
    //
    //  This makes sure that we trigger a watchdog reset when:
    //   - The line stays low for too long (regardless of who causes that)
    //   - We're not idle, but also not making any progress for too long
    //
    if (wdt_flags & (WDT_LINE_HIGH) &&
        (wdt_flags & WDT_PROGRESS || action == ACTION_IDLE)) {
        wdt_reset();
        wdt_flags = 0;
    }

    cli();
    // Re-enable the watchdog if needed. Doing this here, instead of
    // after the sleep below, saves us a cli instruction at the cost of
    // re-enabling the watchdog timer a few cycles later
    wdt_on();

    // Only sleep if the main loop isn't supposed to do anything, to
    // prevent deadlock. There's a magic dance here to make sure
    // an interrupt does not set the action to ACTION_STALL after we
    // checked for it but before entering sleep mode
    if (action != ACTION_STALL) {
        if (action == ACTION_IDLE && !TIMSK0 && (PINB & (1 << PINB1))) {
            // No timers are running, so we can go to power down mode
            // (where timers stop running) instead of sleep mode.  Since
            // we can only wake up from powerdown on a low-level
            // triggered interrupt, we can only go into powerdown when
            // the bus is high. It's ok if the bus becomes low after we
            // check it above, in that case we'll go into powerdown and
            // then come out of it directly again.
            set_sleep_mode(SLEEP_MODE_PWR_DOWN);
            wdt_off();

            // Make INT0 low-level triggered (note that this assumes ISC00
            // is not set)
            // The INT0 handler takes care of making itself
            // edge-triggered again and also reset the sleep mode
            MCUCR &= ~(1<<ISC01);

            // Disable the brown-out detector during sleep in order to
            // save some power (at the cost of a slightly higher wakeup
            // time, but that's ok since we expect a reset pulse next).
            // This register is not available on the tiny13, only on
            // tiny13a
            #if defined(BODCR)
            BODCR = (1 << BODSE);
            BODCR = (1 << BODS);
            #endif
        }
        // The instruction after sei is guaranteed to execute before
        // any interrupts are triggered, so we can be sure the sleep
        // mode is entered, with interrupts enabled, but before any
        // interrupts fire (possibly leaving it again directly if an
        // interrupt is pending).
        sei();
        sleep_cpu();
    }
    sei();
}

int main(void)
{
    setup();
    while(true)
        loop();
}

/* vim: set sw=4 sts=4 expandtab: */
