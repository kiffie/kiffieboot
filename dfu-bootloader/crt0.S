/*********************************************************************
 *
 *                  C Runtime Startup
 *
 *********************************************************************
 * Filename:        crt0.S
 *
 * Processor:       PIC32
 *
 * Compiler:        MPLAB XC32
 *                  MPLAB X IDE
 * Company:         Microchip Technology Inc.
 *
 * Software License Agreement
 *
 * Copyright (c) 2014, Microchip Technology Inc. and its subsidiaries ("Microchip")
 * All rights reserved.
 * 
 * This software is developed by Microchip Technology Inc. and its
 * subsidiaries ("Microchip").
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1.      Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * 2.      Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * 3.      Microchip's name may not be used to endorse or promote products
 * derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY MICROCHIP "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL
 * MICROCHIP BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING BUT NOT LIMITED TO
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWSOEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ********************************************************************/

#include "xc.h"
#include <cp0defs.h>



/* This file contains 32-bit assembly code */
       .set nomips16

        ##################################################################
        # Entry point of the entire application
        ##################################################################
        .section .reset, "ax"
        .align 2
        .set noreorder
        .ent _reset
        .globl _reset
_reset:
        ##################################################################
        # If entered because of an NMI, jump to the application
        ##################################################################
        mfc0    k0,_CP0_STATUS
        ext     k0,k0,19,1              # Extract NMI bit
        beqz    k0,_no_nmi
        nop
        la      k0, start_app
        jr      k0
        nop
_no_nmi:

        ##################################################################
        # Initialize Stack Pointer
        #   _stack is initialized by the linker script to point to the
        #    starting location of the stack in DRM
        ##################################################################
        la      sp,_stack

        ##################################################################
        # Initialize Global Pointer
        #   _gp is initialized by the linker script to point to "middle"
        #   of the small variables region
        ##################################################################
        la      gp,_gp


#if defined(INIT_L1_CACHE)
        ##################################################################
        # Initialize L1 cache register
        ##################################################################
        .extern __pic32_init_cache
        la      t0,__pic32_init_cache
        jalr    t0
        nop
#endif
#ifndef CRT_DISABLE_BSS_INIT
        ##################################################################
        # Clear uninitialized data sections
        ##################################################################
        la      t0,_bss_begin
        la      t1,_bss_end
        b       _bss_check
        nop

_bss_init:
        sw      zero,0x0(t0)
        sw      zero,0x4(t0)
        sw      zero,0x8(t0)
        sw      zero,0xc(t0)
        addu    t0,16
_bss_check:
        bltu    t0,t1,_bss_init
        nop

        ##################################################################
        # Initialize data using the linker-generated .dinit table
        ##################################################################
        .equiv FMT_CLEAR,0
        .equiv FMT_COPY,1
_dinit_init:
        la      t0,_dinit_addr

#define SRC t0
#define DST t1
#define LEN t2
#define FMT t3

0:      lw      DST,0(SRC)
        beqz    DST,9f
        addu    SRC,4
        lw      LEN,0(SRC)
        addu    SRC,4
        lw      FMT,0(SRC)
        beq     FMT,$0,_dinit_clear
        addu    SRC,4

_dinit_copy:
        lbu     t4,0(SRC)
        subu    LEN,1
        addu    SRC,1
        sb      t4,0(DST)
        bne     LEN,$0,_dinit_copy
        addu    DST,1

        b       _dinit_end
        nop

_dinit_clear:
        sb      $0,(DST)
        subu    LEN,1
        bne     LEN,$0,_dinit_clear
        addu    DST,1

_dinit_end:
        addu    SRC,3
        addiu   LEN,$0,0xFFFFFFFC
        and     SRC,LEN,SRC
        lw      DST,0(SRC)
        bne     DST,$0,0b
        nop
9:
#endif
        ##################################################################
        # Call main
        ##################################################################
        and     a0,a0,0
        and     a1,a1,0
        la      t0,main
        jr      t0
        nop

        .end    _reset
