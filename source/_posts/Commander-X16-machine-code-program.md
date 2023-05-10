---
title: Commander X16 machine code program
date: 2023-04-03 23:08:08
tags:
---

Having learnt the basics of how to program an X16, I wanted to make a little tutorial.

For the machine itself, download the latest emulator from its webpage https://www.commanderx16.com/

# BASIC programs

When the machine starts, you can enter BASIC commands.

    PRINT "HELLO WORLD"

This will print  "HELLO WORLD" to the screen.
You can also make a BASIC program.
Enter these commands, including the number in front:

    10 PRINT "HELLO"
    20 PRINT "WORLD"

This is a small BASIC program that will print HELLO on the first line and WORLD on the second line.

Ececute the program with ```RUN```.
You can examine the BASIC program with ```LIST```.

Programs can be saved and restored to/from disk, using ```SAVE "FILENAME.PRG"``` and ```LOAD "FILENAME.PRG"```

The first 2 bytes of the PRG file contains the destination address in RAM, stored as little endian. The rest of the PRG file contains the payload. The destination address is usually 0x0801, which is where the BASIC program is supposed to be stored in RAM.

The format of BASIC programs is documented here https://www.c64-wiki.com/wiki/BASIC#Technical_Details 

Each line of BASIC contains 16 bit address to next line, 16 bit line number, 8 bit BASIC token, zero-terminated string with rest of line.

RAM can be examined using monitor ```MON```:

    M 0801
    .:0801 0F 08 0A 00 99 20 22 48

Here is a full dump of the simple BASIC program above:

    10 PRINT "HELLO"
    .:0801 0F 08 (address to next line, 0x080F)
    .:0803 0A 00 (line number 0x000A -> 10)
    .:0805 99 (BASIC token for PRINT)
    .:0806 20 22 48 45 4C 4C 4F 22 00 (The string ' "HELLO"')

    20 PRINT "WORLD"
    .:080F 1D 08 (address to next line, 0x081D)
    .:0811 14 00 (line number 0x0014 -> 20)
    .:0813 99 (BASIC token for PRINT)
    .:0814 20 22 57 4F 52 4C 44 22 00 (The string ' "WORLD"')
    
    .:081D 00 00 (address to next line, 0x0000. This means end of program)

BASIC programs are not limited to printing. You can make advanced programs using BASIC. However, as this is an interpreted language, it is slower than machine code programs. Thus, most games, and other software which requres higher performance, is represented as machine code instead of BASIC.

# Machine code programs

If you want to speed up your application, you can make a machine code application instead. Commander X16 is equipped with a 65C02 microprocessor, which is quite fun to program. This CPU have 3 general-purpose registers: accumulator A, and index registers X and Y. The CPU is a RISC (Reduced Instruction Set Computer), so that it is possible to learn the reduced number of instructions when writing assembly code.

For an excellent 6502 tutorial, I can highly recommend ChibiAkumas website https://www.assemblytutorial.com/6502/ , in particular his printable cheatsheet with assembler mnemonics (instructions).

For this tutorial, I will use the ACME cross-assembler, as that is the assembler suggested by someone in the X16 community (I cannot remember the reference).

The first example will print the character 'A' to screen using the KERNAL function CHROUT (for reference, see KERNAL chapter in X16 documentation).

hello.asm
```
; Hello world
;
; Minimalistic assembly program for x16, using acme assembler
; Will print a single "A" to screen using KERNAL

; Machine code program will be loaded at address 0x0810 / 2064
; Execute the machine code program with the BASIC command
; SYS 2064

*=$0810

    lda #'A'  ; Load 'A' (0x41) to register A

    jsr $FFD2 ; Call the KERNAL function at address FFD2.
              ; This is the CHROUT function, which will print
              ; the character in register A to screen.

    rts       ; Return to BASIC

```
Build this program with ```acme -v3 -f cbm --cpu 65c02 --report report.txt -o hello.prg hello.asm```

To load this program, use the syntax ```LOAD "HELLO.PRG",8,1```. This will load from device 8 (disk/SD-card). The ",1" specifies that the file will be loaded to the absolute address specified by the two first bytes in the file. You can also load the file while starting the emulator using ```x16emu -prg hello.prg```.

To run this program, run ```SYS 2064```.
This will execute the machine code routine stored at address 2064 / 0x0810.

It is inconvenient for end users to start programs using this command. To simplify this process, a small BASIC header can be added in front of the machine code program.

```
; Hello world v2
;
; Minimalistic assembly program for x16, using acme assembler
; Will print a single "A" to screen using KERNAL

; Basic header
; 10 SYS 2064

*=$0801
!byte $0C,$08,$0A,$00,$9E,$20,$32,$30,$36,$34,$00,$00,$00
*=$0810

; Kernal functions
CHROUT=$FFD2   ; Kernal: CHROUT outputs a character to screen

    lda #'A'   ; Load 'A' (0x41) to register A

    jsr CHROUT ; Call the KERNAL function CHROUT.
               ; This will print the character in register A to screen.

    rts        ; Return to BASIC
```

This improved example contains a "BASIC header". When loading this program, you can view the BASIC program using ```LIST```. The program contains one line, ```10 SYS 2064```. This means that you can run the machine code program simply by using ```RUN```. This example also shows how you can define a symbol, CHROUT, to make the code easier to read.