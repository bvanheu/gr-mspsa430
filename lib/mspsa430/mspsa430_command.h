/* --COPYRIGHT--,BSD
 * Copyright (c) 2011, Texas Instruments Incorporated
 * Copyright (c) 2013, Benjamin Vanheuverzwijn <bvanheu@gmail.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/

#ifndef MSPSA430_COMMAND_H
#define MSPSA430_COMMAND_h

// taken

enum mspsa430_command {
    // General Commands
    CMD_GETIDN         =  1,    //  IDN
    CMD_GETHWSERNR     =  2,    // Get Hardware Serial Number
    CMD_HWRESET        =  3,    // Hardware Reset (PUC) Required PSW Data PSW = 0xDE, 0xAD
    CMD_BLINKLED       =  4,    // Identify the Hardware by et the LED blinking
    CMD_GETCOREVER     =  5,
    CMD_GETLASTERROR   =  6,
    CMD_SYNC           =  7,
    // Flash operation
    CMD_FLASH_READ      = 10,
    CMD_FLASH_WRITE     = 11,
    CMD_FLASH_ERASE     = 12,
    CMD_FLASH_GETCRC    = 13,
    // Spectrum Measurement
    CMD_GETSPECVER     =  20,
    CMD_SETFSTART      =  21,   // Set Start Frequency fstart
    CMD_SETFSTOP       =  22,   // Set Stop  Frequency fstop
    CMD_SETFSTEP       =  23,   // Set Step  Frequency fstep
    CMD_SETFRQ         =  24,
    CMD_SETRBW         =  25,   // Set Rx Filter bandwidth
    CMD_SETDAC         =  26,   // Set the DC value for the balun
    CMD_SETGAIN        =  27,   // Set the gain of the Rx path
    CMD_SETIF          =  28,   // Set Intermediate Frequency
    CMD_INITPARAMETER  =  30,   // Setup the system for spectrum measurement
    CMD_GETSPECNOINIT  =  31,   // Measures the spectrum previously defined
    // Production Commands
    CMD_GETPRODVER     =  60,
    CMD_SETPRODFWINIT  =  61,
    CMD_GETTEMP        =  62,
    CMD_SETHWID        =  63,
    CMD_GETHWID        =  64,
    CMD_GETBOOTCNT     =  65,
    CMD_SETFOUT        =  66,   // 0 .. Off, 1 .. 26MHz, 2 .. next bytes are f RF
    CMD_SETFXTAL       =  67,   // f 4 byte, incl temp und cal vers, hwid
    CMD_GETFXTAL       =  68,   // f 4 byte, incl temp und cal vers, hwid
    CMD_SWEEPEDC       =  69,   // f, gain, repetition cnt
    CMD_GETCHIPTLV     =  73,
    // FIXME - is it still in use?
    CMD_FRAMEERROR     = 255,   // Frame Error does not exist anymore !!!! Check this
};

enum mspsa430_error {
    ERR_NO_ERROR               =    0,
    ERR_CMDBUFFEROVERFLOW      =  800,
    ERR_WRONGCMDLENGTH         =  801,
    ERR_CMDABORTED             =  802,
    ERR_LOSTCMD                =  803,
    ERR_CMDUNKNOWN             =  804,
    ERR_TOOMUCHDATAREQUESTEDBYUSERFUNCTION  = 805,
    ERR_RESTOREPROGRAMCOUNTER  =  806,
    ERR_BUFFERPOSOUTOFRANGE    =  807,
    ERR_EEQBUFFEROVERFLOW      =  808,
    ERR_WRONGCRCLOWBYTE        =  809,
    ERR_WRONGCRCHIGHBYTE       =  810,
    ERR_RESTOREFROMPACKETERROR =  812,
    ERR_NOFRAMESTART           =  813,
    ERR_WRONGPKTLENGTH         =  814,
    ERR_PACKETINCOMPLETE       =  815,
    ERR_PACKETERROR            =  816,
    ERR_STUPIDPACKETHANDLER    =  817,
    ERR_BUFFEROVERFLOW         =  850,
    ERR_BUFFERUNDERRUN         =  851,
    ERR_FLASHNOTERASED         = 1100,
    ERR_FLASHMISMATCH          = 1101,
    ERR_RSSIVALIDFLAGNOTSET    = 1200,
    ERR_PLLNOTSETTLED          = 1201
};

#endif
