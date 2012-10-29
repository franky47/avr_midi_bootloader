/*!
 *  \file       avr_MidiBootloader.c
 *  \brief      AVR bootloader with MIDI for firmware update through SysEx.
 *  \author     Francois Best
 *  \date       19/08/2009
 *  \revision   13/10/2012
 *  \version    1.1.0
 *  \license    CC-BY-SA Forty Seven Effects - 2012
 *
 * THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS 
 * OF THIS CREATIVE COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE").
 * THE WORK IS PROTECTED BY COPYRIGHT AND/OR OTHER APPLICABLE LAW. 
 * ANY USE OF THE WORK OTHER THAN AS AUTHORIZED UNDER THIS LICENSE 
 * OR COPYRIGHT LAW IS PROHIBITED.
 * 
 * BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT 
 * AND AGREE TO BE BOUND BY THE TERMS OF THIS LICENSE. 
 * TO THE EXTENT THIS LICENSE MAY BE CONSIDERED TO BE A CONTRACT, 
 * THE LICENSOR GRANTS YOU THE RIGHTS CONTAINED HERE IN CONSIDERATION 
 * OF YOUR ACCEPTANCE OF SUCH TERMS AND CONDITIONS.
 * http://creativecommons.org/licenses/by-sa/3.0/
 */

#include <inttypes.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/boot.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <string.h>
#include <compat/deprecated.h>

// -----------------------------------------------------------------------------
// Compile Flags

#define ENABLE_WRITE        1   // Burn the firmware in flash memory.
#define ENABLE_JUMP         1   // Jump to program when loaded.
#define ENABLE_EE_DATA      1   // Save state & information in internal EEPROM.
#define ENABLE_LCD          0   // Print info on hooked up LCD module.
#define ENABLE_DEBUG        0   // Print info on debug serial line.

#define DEBUG_LINE_ACTIVATED ((ENABLE_DEBUG) && !(ENABLE_LCD))


// -----------------------------------------------------------------------------
// Configuration & Settings

#if defined(__AVR_ATmega644P__)
#   define PAGE_SIZE            0x080U   // 128 words
#   define PAGE_SIZE_BYTES      0x100U   // 256 bytes
#else
#   error Implement page size for the target device.
#endif

#define INPUT_BUFFER_SIZE       256

#define UART_MIDI               1
#define UART_DEBUG              0
#define UART_LCD                0

#define BAUD_RATE_MIDI          31250
#define BAUD_RATE_LCD           9600
#define BAUD_RATE_DEBUG         38400

#define EEADDR_FIRMWARE_INFO    0x00    // Address of the first byte in EEPROM.

#define VENDOR_ID               0x2F2F
#define VENDOR_ID_MSB           (VENDOR_ID >> 8)
#define VENDOR_ID_LSB           (VENDOR_ID & 0xFF)

#ifndef DEVICE_CODE
#   define DEVICE_CODE          0x00    // Default device code.
#endif

#if DEVICE_CODE & 0x80
#   error Device codes must be lower than 128 (MSBit must be zero).
#endif


// -----------------------------------------------------------------------------
// Macros

// UART Initialisation macro
#define BEGIN_SERIAL(uart_, baud)                                               \
{                                                                               \
    UBRR##uart_##H = ((F_CPU / 16 + baud / 2) / baud - 1) >> 8;                 \
    UBRR##uart_##L = ((F_CPU / 16 + baud / 2) / baud - 1);                      \
/* reset config for UART */                                                     \
    UCSR##uart_##A = 0;                                                         \
    UCSR##uart_##B = 0;                                                         \
    UCSR##uart_##C = 0;                                                         \
/* enable rx and tx */                                                          \
    sbi(UCSR##uart_##B, RXEN##uart_);                                           \
    sbi(UCSR##uart_##B, TXEN##uart_);                                           \
/* enable interrupt on complete reception of a byte */                          \
/*sbi(UCSR##uart_##B, RXCIE##uart_);*/                                          \
    UCSR##uart_##C = _BV(UCSZ##uart_##1)|_BV(UCSZ##uart_##0);                   \
/* defaults to 8-bit, no parity, 1 stop bit */                                  \
}

#define max(a, b) (a > b) ? a : b


// -----------------------------------------------------------------------------
// Type Definitions

typedef uint8_t  byte;
typedef uint16_t word;

typedef struct Chunk_t
{
    byte        id;
    uint16_t    address;                // Address of the first instruction
    uint16_t    count;                  // Number of bytes in the chunk
    byte        data[PAGE_SIZE_BYTES];
} Chunk;


typedef struct FirmwareInfo_t
{
    byte        device;                 // Device type
    byte        day;                    // Firmware build day
    byte        month;                  // Firmware build month
    byte        year;                   // Firmware build year (09 = 2009)
    byte        hour;                   // Firmware build timestamp
    byte        minute;                 // Firmware build timestamp
    byte        numChunks;
    byte        numSubChunks;           // Num subchunks per chunk
    word        sizeOfSubChunk;         // Size in bytes
    word        size;                   // Firmware total size
    byte        checksum;
} FirmwareInfo;


// -----------------------------------------------------------------------------
// Functions Prototypes

int main(void);
byte decodeSysEx(byte* inSysEx, byte* outData, byte inLength);
byte getByteOnUSART0(void);
byte getByteOnUSART1(void);
void sendByteOnUSART0(byte);
void sendByteOnUSART1(byte);
void sendACK(byte);
void sendNAK(byte);
void sendDeviceInquiryReply(void);
void parseSysEx(byte*, word);
void parseGenericMessage(byte*, word);
void parseUniversalMessage(byte*, word);
void handleDeviceInquiry(byte*, word);
void handleHeader(byte*, word);
void handleDataPacket(byte*, word);
void handleEOF(byte*, word);

#if ENABLE_WRITE
void writeChunk();
#endif

void reboot(void);

#if ENABLE_LCD
void LCDPrint(byte, byte, char*);
void LCDClear();
void Cursor(byte, byte);
#endif

#if ENABLE_DEBUG
void debug(char*);
void debugData(byte);
void CoreDump();
#endif

void initEEPROM();
byte EEPROM_read(int);
void EEPROM_write(int, byte);


// -----------------------------------------------------------------------------
// Globals

FirmwareInfo gFirmware;
Chunk gChunk;


// -----------------------------------------------------------------------------
// Program Jumps

#if ENABLE_JUMP
void (*jumpToMainProgram)(void) = 0x0000;
#else
void jumpToMainProgram(void);
void jumpToMainProgram(void)
{
#if ENABLE_DEBUG
    CoreDump();
#endif
}
#endif

void reboot(void)
{
    cli();
    wdt_enable(WDTO_15MS);
    while (1);
}


// -----------------------------------------------------------------------------
// Debug

#if ENABLE_DEBUG

void debug(char* text)
{
    word length = strlen(text);
    word i;
    for (i = 0; i < length; ++i)
    {
        sendByteOnUSART0((byte)text[i]);
    }
    //    sendByteOnUSART0('\n');
}


void debugData(byte data)
{
    char text[4] = {0};
    byte msb = data >> 4;
    byte lsb = data & 0x0F;
    text[0] = msb + ((msb < 10) ? '0' : ('A' - 10));
    text[1] = lsb + ((lsb < 10) ? '0' : ('A' - 10));
    text[2] = ' ';
    debug(text);
}


void CoreDump()
{
    word page, ligne, oct;
    // Dump de la flash
    for (page = 0; page < 256; ++page)
    {
        for (ligne = 0; ligne < 16; ++ligne)
        {
            // écrire ici l'adresse hexa
            debug("0x");
            debugData((page * 256 + ligne * 16) >> 8);
            debugData((page * 256 + ligne * 16) & 0xFF);
            debug(":  ");
            for (oct = 0; oct < 16; ++oct)
            {
                // Ecrire ici les données
                debugData(pgm_read_byte_near(page * 256 + ligne * 16 + oct));
            }
            debug("\n");
        }
    }
    
    debug("EEPROM\n");
    for (ligne = 0; ligne < 4; ++ligne)
    {
        // écrire ici l'adresse hexa
        debug("0x");
        debugData((ligne * 16) >> 8);
        debugData((ligne * 16) & 0xFF);
        debug(":  ");
        for (oct = 0; oct < 16; ++oct)
        {
            debugData(EEPROM_read(ligne * 16 + oct));
        }
        debug("\n");
    }
}

#endif


// -----------------------------------------------------------------------------
// EEPROM

#if ENABLE_EE_DATA

enum // EEPROM Firmware info
{
    kFirmInfo_BootFlag = 0,
    kFirmInfo_DeviceCode,
    kFirmInfo_Day,
    kFirmInfo_Month,
    kFirmInfo_Year,
    kFirmInfo_Hour,
    kFirmInfo_Minute,
    kFirmInfo_BinSizeMSB,
    kFirmInfo_BinSizeLSB,
    kFirmInfo_Checksum,
};

enum // EEPROM Addresses
{
    kEEAddr_BootFlag        = EEADDR_FIRMWARE_INFO + kFirmInfo_BootFlag,
    kEEAddr_DeviceCode      = EEADDR_FIRMWARE_INFO + kFirmInfo_DeviceCode,
    kEEAddr_Day             = EEADDR_FIRMWARE_INFO + kFirmInfo_Day,
    kEEAddr_Month           = EEADDR_FIRMWARE_INFO + kFirmInfo_Month,
    kEEAddr_Year            = EEADDR_FIRMWARE_INFO + kFirmInfo_Year,
    kEEAddr_Hour            = EEADDR_FIRMWARE_INFO + kFirmInfo_Hour,
    kEEAddr_Minute          = EEADDR_FIRMWARE_INFO + kFirmInfo_Minute,
    kEEAddr_BinSizeMSB      = EEADDR_FIRMWARE_INFO + k FirmInfo_BinSizeMSB,
    kEEAddr_BinSizeLSB      = EEADDR_FIRMWARE_INFO + kFirmInfo_BinSizeLSB,
    kEEAddr_Checksum        = EEADDR_FIRMWARE_INFO + kFirmInfo_Checksum,
};

enum 
{
    kBootFlag_Invalid       = 0xFF, // Uninitialised
    kBootFlag_Boot          = 'B',  // Launch firwmare
    kBootFlag_Idle          = 'I',  // ??
    kBootFlag_UpdateRequest = 'U',  // Start listening for messages
    kBootFlag_Programming   = 'P',  // Resume update sequence
    kBootFlag_Error         = 'E',
};

void initEEPROM()
{
    EEPROM_write(kEEAddr_BootFlag,      kBootFlag_UpdateRequest);
    EEPROM_write(kEEAddr_DeviceCode,    DEVICE_CODE);
    EEPROM_write(kEEAddr_Day,           0x01); // 1st
    EEPROM_write(kEEAddr_Month,         0x01); // January
    EEPROM_write(kEEAddr_Year,          0x00); // 2000
    EEPROM_write(kEEAddr_Hour,          0x00); // Midnight
    EEPROM_write(kEEAddr_Minute,        0x00);
    EEPROM_write(kEEAddr_BinSizeMSB,    0x00);
    EEPROM_write(kEEAddr_BinSizeLSB,    0x00);
    EEPROM_write(kEEAddr_Checksum,      0x00);
}


byte EEPROM_read(int address)
{
    eeprom_busy_wait();
    return eeprom_read_byte((unsigned char*) address);
}


void EEPROM_write(int address, byte value)
{
    eeprom_busy_wait();
    eeprom_write_byte((unsigned char*) address, value);
}

#endif


// -----------------------------------------------------------------------------
// Communications

byte decodeSysEx(byte* inSysEx, byte* outData, byte inLength)
{
    byte cnt;
    byte cnt2 = 0;
    byte bits = 0;
    
    for (cnt = 0; cnt < inLength; ++cnt)
    {
        if ((cnt % 8) == 0)
        {
            bits = inSysEx[cnt];
        } 
        else
        {
            outData[cnt2++] = inSysEx[cnt] | ((bits & 1) << 7);
            bits >>= 1;
        }
    }
    return cnt2;
}


typedef struct
{
    byte index;
    byte data[7];
    byte expected;
} SysExDecoder;

SysExDecoder gSysExDecoder;

byte decodeSysExByte(byte inData)
{
    if (gSysExDecoder.index == 0)
    {
        // MSBs are received first.
        byte i;
        byte msb = inData;
        for (i = 0; i < 7; ++i)
        {
            // Initialise data with MSBs only.
            gSysExDecoder.data[i] = (msb & 0x01) << 7;
            msb >>= 1;
        }
        gSysExDecoder.index++;
        return 0;
    }
    else
    {
        gSysExDecoder.data[gSysExDecoder.index - 1] |= inData;
        
        if (gSysExDecoder.index == gSysExDecoder.expected)
        {
            gSysExDecoder.index = 0;
            return 1;
        }
        else
        {
            return 0;
        }
    }
}


byte getByteOnUSART0(void)
{
    // uint32_t count = 0;
    
    while(!(UCSR0A & _BV(RXC0))) 
    {
        //    count++;
        //    if (count > MAX_TIME_COUNT) jumpToMainProgram();
    }
    return UDR0;
}


byte getByteOnUSART1(void)
{
    //uint32_t count = 0;
    
    while(!(UCSR1A & _BV(RXC1)))
    {
        //count++;
        //if (count > MAX_TIME_COUNT) jumpToMainProgram();
    }
    return UDR1;
}


void sendByteOnUSART0(byte data)
{
    while (!(UCSR0A & _BV(UDRE0)));
    UDR0 = data;
}


void sendByteOnUSART1(byte data)
{
    while (!(UCSR1A & _BV(UDRE1)));
    UDR1 = data;
}


void sendACK(byte inPacketNumber)
{
    byte data[6] = { 0xF0, 0x7E, 0x7F, 0x7F, inPacketNumber, 0xF7 };
    word i;
    
#if (UART_MIDI == 1)
    for (i = 0; i < 6; ++i) sendByteOnUSART1(data[i]);
#else
    for (i = 0; i < 6; ++i) sendByteOnUSART0(data[i]);
#endif
}


void sendNAK(byte inPacketNumber)
{
    byte data[6] = { 0xF0, 0x7E, 0x7F, 0x7E, inPacketNumber, 0xF7 };
    word i;
    
#if (UART_MIDI == 1)
    for (i = 0; i < 6; ++i) sendByteOnUSART1(data[i]);
#else
    for (i = 0; i < 6; ++i) sendByteOnUSART0(data[i]);
#endif
    
}


// -----------------------------------------------------------------------------
// Parsers & handlers

void parseSysEx(byte* data, word length)
{
    switch (data[1])
    {
        case 0x7E: // Universal
            parseUniversalMessage(data, length);
            return;
            break;
        case 0x00: // Generic
            parseGenericMessage(data, length);
            return;
            break;
        default:
            break;
    }
}


void parseGenericMessage(byte* inData, word inLength)
{
    byte rebootRequest[7]  = { 0xF0, 0x00, VENDOR_ID_MSB, VENDOR_ID_LSB, 0x05, 'r', 0xF7 };
    byte upgradeRequest[7] = { 0xF0, 0x00, VENDOR_ID_MSB, VENDOR_ID_LSB, 0x05, 'u', 0xF7 };
    
    if (!memcmp(rebootRequest, inData, max(inLength, 7)))
    {
        reboot();
    }
    else if (!memcmp(upgradeRequest, inData, max(inLength, 7)))
    {
        
#if ENABLE_EE_DATA
        
        EEPROM_write(kEEAddr_BootFlag, kBootFlag_UpdateRequest);
        eeprom_busy_wait();
        
#endif
        
        // Pas la peine de reboot, on est déjà dans le bootloader
        // (reboot c'est pour un catch du message dans l'app)
        sendACK(0);
        return;
        
    }
    
}


void parseUniversalMessage(byte* data, word length)
{
    switch (data[5])
    {
        case 0x07: // File Dump
            switch (data[6])
        {
            case 0x01: // Header
                handleHeader(data, length);
                break;
            case 0x02: // Data Packet
                handleDataPacket(data, length);
                break;
            default:
                break;
        }
            break;
        case 0x7B: // EOF
            handleEOF(data, length);
            break;
        case 0x06: // Device info request
            handleDeviceInquiry(data, length);
            break;
        default:
            break;
    }
}


void handleDeviceInquiry(byte* data, word length)
{
    if (data[6] == 0x01)
    {
        // DeviceInquiry, prepare answer
        byte reply[17] = {
            0xF0,   // Start SysEX
            0x7E,   // Universal Non Real Time
            0x7F,   // All Devices (broadcast)
            0x06,   // Device Identity reply
            0x02,   // Device Identity reply
            0x00,   // 3 bytes for Vendor Id
            VENDOR_ID_MSB,
            VENDOR_ID_LSB,
            0x00,   // Device code
            0x00,   // Day
            0x00,   // Month
            0x00,   // Year
            0x00,   // Hour
            0x00,   // Minute
            0x00,   // Size of chunk (MSB)
            0x00,   // Size of chunk (LSB)
            0xF7    // End of Exclusive
        };
        
        byte i;
        
#if ENABLE_EE_DATA
        // Load EEPROM data (Device code + timestamp)
        const word baseAddress = kEEAddr_DeviceCode;
        for (i = 0; i < 6; ++i)
        {
            reply[8 + i] = EEPROM_read(baseAddress + i);
        }
#endif
        
        reply[14] = sizeof(Chunk) >> 7;
        reply[15] = sizeof(Chunk) & 0x7F;
        
#if (UART_MIDI == 1)
        for (i = 0; i < 17; ++i) sendByteOnUSART1(reply[i]);
#else
        for (i = 0; i < 17; ++i) sendByteOnUSART0(reply[i]);
#endif
    }
}


void handleHeader(byte* data, word length)
{
    if (memcmp(&data[8], "MFU ", 4) == 0)
    {
        // Decode header
        byte decoded[12] = { 0 };
        decodeSysEx(&data[12], decoded, 14);
        
        gFirmware.device            = decoded[0];
        gFirmware.day               = decoded[1];
        gFirmware.month             = decoded[2];
        gFirmware.year              = decoded[3];
        gFirmware.hour              = decoded[4];
        gFirmware.minute            = decoded[5];
        gFirmware.numChunks         = decoded[6];
        gFirmware.numSubChunks      = decoded[7]; // Num of subs per chunk
        gFirmware.sizeOfSubChunk    = decoded[8];
        gFirmware.size              = (uint16_t)decoded[9] << 8 | decoded[10];
        gFirmware.checksum          = decoded[11];
        
        gChunk.id = -1; // So that the first one is set to 0
        
#if ENABLE_LCD
        LCDClear();
        LCDPrint(0,0,"Update firmware:");
#endif
        
        sendACK(0);
        return;
        
    }
    else sendNAK(0);
}


void handleDataPacket(byte* data, word length)
{
    word i;
    const byte subchunkIndex = data[7];
    byte checksum = 0;
    
    for (i = 1; i < length - 2; ++i)
    {
        checksum ^= data[i];
    }
    
    if (checksum != data[length - 2])
    {
        sendNAK(subchunkIndex);
        return;
    }
    
    // Décodage
    byte decoded[68] = { 0xFF };
    decodeSysEx(&data[9], decoded, data[8] + 1);
    
    if (subchunkIndex == 0)
    {
        // First chunk packet -> reset
        gChunk.address = (decoded[0] << 8) | decoded[1];    // Address init
        gChunk.count   = (decoded[2] << 8) | decoded[3];    // Counter init
        
        for (i = 0; i < PAGE_SIZE_BYTES; ++i) 
        {
            gChunk.data[i] = 0;    // init des data
        }
        for (i = 0; i < gChunk.count; ++i) 
        {
            gChunk.data[i] = decoded[i + 4];
        }
        gChunk.id++;
    }
    else
    {
        word count = (decoded[2] << 8) | decoded[3]; // decoded data byte count
        
        // On assume que les données sont calées -> boulot de MobiusUpdater
        for (i = 0; i < count; ++i)
        {
            gChunk.data[i + gChunk.count] = decoded[i + 4];
        }
        gChunk.count += count;
    }
    
#if ENABLE_WRITE
    if (subchunkIndex == 3)
    {
        writeChunk();
    }
#endif
    
    sendACK(subchunkIndex);
    
}


void handleEOF(byte* data, word length)
{
    if (gChunk.id + 1 == gFirmware.numChunks)
    {
#if ENABLE_EE_DATA
        // Store firmware info
        EEPROM_write(kEEAddr_DeviceCode,    gFirmware.device);
        EEPROM_write(kEEAddr_Day,           gFirmware.day);
        EEPROM_write(kEEAddr_Month,         gFirmware.month);
        EEPROM_write(kEEAddr_Year,          gFirmware.year);
        EEPROM_write(kEEAddr_Hour,          gFirmware.hour);
        EEPROM_write(kEEAddr_Minute,        gFirmware.minute);
        EEPROM_write(kEEAddr_BinSizeMSB,    gFirmware.size >> 8);
        EEPROM_write(kEEAddr_BinSizeLSB,    gFirmware.size & 0xFF);
#endif
        // Compute checksum
        word i;
        byte checksum = 0;
        
        for (i = 0; i < gFirmware.size; ++i)
        {
            checksum ^= pgm_read_byte_near(i);
        }
        
        if (checksum == gFirmware.checksum)
        {    
#if ENABLE_EE_DATA
            // Write checksum in EEPROM to (optionally) check firmware validity
            EEPROM_write(kEEAddr_Checksum, checksum);
            
            // Firmware upgrade success: change boot flag status.
            EEPROM_write(kEEAddr_BootFlag, kBootFlag_Boot);
            eeprom_busy_wait();
#endif
            sendACK(0); // Notify host
            
#if ENABLE_LCD
            LCDPrint(1,0,"       OK       ");
#endif
            
            _delay_ms(500);
            reboot();
        }
        else {
            sendNAK(0);
            // Pas cool... 
        }
    }
    else {
        sendNAK(0); // Là on est dans la merde, 
        //faut tout recommencer et c'est pas prévu.
        // Reboot(); // pourquoi?
    }
    //sendACK(0);
}


#if ENABLE_WRITE
void writeChunk()
{
    byte sreg = SREG;
    cli();
    eeprom_busy_wait();
    
    boot_page_erase_safe(gChunk.address);
    boot_spm_busy_wait();
    
    word address = gChunk.address;
    word i;
    for (i = 0; i < gChunk.count; i += 2)
    {
        word temp = gChunk.data[i] | (gChunk.data[i + 1] << 8); // Little Endian
        boot_page_fill_safe(address, temp);
        address += 2;
    }
    
    boot_page_write_safe(gChunk.address);
    boot_rww_enable_safe();
    SREG = sreg;
    boot_spm_busy_wait();
    
#if ENABLE_LCD
    char whitesquare[2] = { 42, 0 };
    LCDPrint(1, (gChunk.id * 16) / gFirmware.numChunks, whitesquare);
#endif
}
#endif


// -----------------------------------------------------------------------------
// LCD Interface

#if ENABLE_LCD

void LCDPrint(byte line, byte col, char* text)
{
    Cursor(line,col);
    word i=0,length=strlen(text);
    for (i=0;i<length;i++) sendByteOnUSART0(text[i]);
}


void LCDClear()
{
    sendByteOnUSART0(0xFE);
    sendByteOnUSART0(0x01);
}


void Cursor(byte line, byte col)
{
    line %= 2;
    col %= 16;
    word offset = (line*64);
    sendByteOnUSART0(0xFE);
    sendByteOnUSART0((char)(offset + col + 128));
    _delay_ms(10);
}

#endif


// -----------------------------------------------------------------------------
// Main Entry Point

int main(void)
{
    asm volatile("nop\n\t");
    
    //    byte mcusr = MCUSR;
    MCUSR = 0;
    wdt_disable();
    
    cli();
    
#if ENABLE_EE_DATA
    if (pgm_read_byte_near(0x0000) == 0xFF)
    {
        // No program loaded, loop in bootloader and wait for instructions.
        // In the mean time, format EEPROM data.
        initEEPROM();
    }
    else
    {
        // There's a program on the flash. Look for the boot flag in EEPROM.
        const byte bootFlag = EEPROM_read(kEEAddr_BootFlag);
        
        switch (bootFlag)
        {
            case kBootFlag_Boot:
                // Launch firmware
                jumpToMainProgram();
                break;
            case kBootFlag_UpdateRequest:
                // The firmware has written the UpdateRequest flag and rebooted.
                // Loop into the bootloader and wait for messages.
                sendACK(0); // Notify host that we're ready.
                break;
                
            case kBootFlag_Programming:
                // \todo Implement resuming of firmware update.
                break;
                
            case kBootFlag_Invalid:
            case kBootFlag_Error:
            default:
                initEEPROM();
                break;
        }
    }
    
#endif
    
    
    // On lance les comms
    
#if (UART_LCD == 1)
    BEGIN_SERIAL(1, BAUD_RATE_LCD);
#else
    BEGIN_SERIAL(0, BAUD_RATE_LCD);
#endif
    
#if (UART_MIDI == 1)
    BEGIN_SERIAL(1, BAUD_RATE_MIDI);
#else 
    BEGIN_SERIAL(0, BAUD_RATE_MIDI);
#endif
    
    
    byte buffer[INPUT_BUFFER_SIZE];
    word index = 0;
    
    for (;;)
    {
        const byte received = getByteOnUSART1();
        
        if (received == 0xF0)
        {
            // Fill the message buffer
            index = 0;
            buffer[0] = received;
            
            for (index = 1; 
                 index < INPUT_BUFFER_SIZE && buffer[index-1] != 0xF7;
                 ++index)
            {
                buffer[index] = getByteOnUSART1();
            }
            
            // Launch the parser
            parseSysEx(buffer,index);
        }
    }
    
    // We shall never fall here,
    // as for(;;) is an infinite loop.
    
    return 0;
}

