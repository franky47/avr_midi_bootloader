/*!
 *  \file		avr_MidiBootloader.c
 *  \brief		AVR bootloader with MIDI for firmware update through SysEx.
 *  \author		Francois Best
 *	\date		19/08/2009
 *  \version	1.1.0
 *	Copyright   Forty Seven Effects 2012
 */


/*===========================================================================
 Includes
 ============================================================================*/

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
//#include <stdlib.h>


/*===========================================================================
 Compilation Flags
 ============================================================================*/

#define ENABLE_WRITE	1	// Burn the firmware in flash memory.
#define ENABLE_JUMP		1	// Jump to program when loaded.
#define ENABLE_EE_DATA	1	// Save state & information in internal EEPROM.
#define ENABLE_LCD		1	// Print info on hooked up LCD module.
#define ENABLE_DEBUG	0	// Print info on debug serial line.

#define DEBUG_LINE_ACTIVATED ((ENABLE_DEBUG) && !(ENABLE_LCD))


/*===========================================================================
 Configuration & Settings
 ============================================================================*/

#define PAGE_SIZE				0x080U   //128 words for ATmega644P
#define PAGE_SIZE_BYTES			0x100U   //256 bytes for ATmega644P

#define INPUT_BUFFER_SIZE		256	

#define UART_MIDI				1
#define UART_DEBUG				0
#define UART_LCD				0		// \todo Remove Serial LCD code for Release.

#define BAUD_RATE_MIDI			31250
#define BAUD_RATE_LCD			9600
#define BAUD_RATE_DEBUG			38400

#define EEPROM_FLAG				0
#define EEPROM_FIRMWARE_INFOS	0x01


/*===========================================================================
 Macros
 ============================================================================*/

// UART Initialisation macro
#define BEGIN_SERIAL(uart_, baud) \
{ \
UBRR##uart_##H = ((F_CPU / 16 + baud / 2) / baud - 1) >> 8; \
UBRR##uart_##L = ((F_CPU / 16 + baud / 2) / baud - 1); \
\
/* reset config for UART */ \
UCSR##uart_##A = 0; \
UCSR##uart_##B = 0; \
UCSR##uart_##C = 0; \
\
/* enable rx and tx */ \
sbi(UCSR##uart_##B, RXEN##uart_);\
sbi(UCSR##uart_##B, TXEN##uart_);\
\
/* enable interrupt on complete reception of a byte */ \
/*sbi(UCSR##uart_##B, RXCIE##uart_);*/ \
UCSR##uart_##C = _BV(UCSZ##uart_##1)|_BV(UCSZ##uart_##0); \
/* defaults to 8-bit, no parity, 1 stop bit */ \
}


/*===========================================================================
 Type Definitions
 ============================================================================*/

typedef uint8_t byte;
typedef uint16_t word;

typedef struct Chunk_t {
	byte		CurrentChunkID;
	uint16_t	address;			// Address of the first instruction
	uint16_t	count;				// Number of bytes in the chunk
	byte		data[PAGE_SIZE_BYTES];
} ChunkStruct;


typedef	struct FirmwareInfo_t {
	byte		device;				// Device type (see FortySevenEffects_Devices.h)
	byte		day;				// Firmware build day
	byte		month;				// Firmware build month
	byte		year;				// Firmware build year (09 = 2009)
	byte		hour;
	byte		minute;
	byte		numberofchunks;
	byte		numberofsubchunks;
	word		sizeofsubchunk;
	word		size;
	byte		checksum;
} FirmwareInfo;


/*===========================================================================
 Prototypes
 ============================================================================*/

int main(void);
byte SysExDecoder(byte *inSysEx, byte *outData, const byte inLength);
byte GetByteOnUSART0(void);
byte GetByteOnUSART1(void);
void SendByteOnUSART0(byte);
void SendByteOnUSART1(byte);
void SendACK(byte);
void SendNAK(byte);
void SendDeviceInquiryReply(void);
void ParseSysEx(byte *,word);
void ParseGenericMessage(byte *,word);
void ParseUniversalMessage(byte *,word);
void HandleDeviceInquiry(byte *,word);
void HandleHeader(byte *,word);
void HandleDataPacket(byte *,word);
void HandleEOF(byte *,word);

#if ENABLE_WRITE
void WriteChunk();
#endif

void Reboot(void);

#if ENABLE_LCD
void LCDPrint(byte,byte,char*);
void LCDClear();
void Cursor(byte,byte);
#endif

#if ENABLE_DEBUG
void Debug(char *);
void DebugData(byte);
void CoreDump();
#endif

void InitEEPROM();
byte EEPROM_read(int);
void EEPROM_write(int, byte);


/*===========================================================================
 Globals
 ============================================================================*/

FirmwareInfo	Firmware;
ChunkStruct		Chunk;


/*===========================================================================
 Program Jumps
 ============================================================================*/

#if ENABLE_JUMP
void (*JumpToMainProgram)(void) = 0x0000;
#else
void JumpToMainProgram(void);
void JumpToMainProgram(void)
{
#if ENABLE_DEBUG
	CoreDump();
#endif
}
#endif

void Reboot(void)
{
	cli();
	wdt_enable(WDTO_15MS);
	while(1);
}


/*===========================================================================
 Debug
 ============================================================================*/

#if ENABLE_DEBUG

void Debug(char * text)
{
	
	word length = strlen(text);
	word i;
	for (i=0;i<length;i++) SendByteOnUSART0((byte)text[i]);
	
	//	SendByteOnUSART0('\n');
}


void DebugData(byte data)
{

	char text[4] = {0};
	byte msb = data >> 4;
	byte lsb = data & 0x0F;
	text[0] = msb+((msb<10)?'0':('A'-10));
	text[1] = lsb+((lsb<10)?'0':('A'-10));
	text[2] = ' ';
	Debug(text);
	
}


void CoreDump()
{
	
	word page,ligne,oct;
	// Dump de la flash
	for (page=0;page<256;page++) {
		for (ligne=0;ligne<16;ligne++) {
			// écrire ici l'adresse hexa
			Debug("0x");
			DebugData((page*256+ligne*16)>>8);
			DebugData((page*256+ligne*16)&0xFF);
			Debug(":  ");
			for (oct=0;oct<16;oct++) {
				// Ecrire ici les données
				DebugData(pgm_read_byte_near(page*256+ligne*16+oct));
			}
			Debug("\n");
		}
	}
	
	Debug("EEPROM\n");
	for (ligne=0;ligne<4;ligne++) {
		// écrire ici l'adresse hexa
		Debug("0x");
		DebugData((ligne*16)>>8);
		DebugData((ligne*16)&0xFF);
		Debug(":  ");
		for (oct=0;oct<16;oct++) {
			DebugData(EEPROM_read(ligne*16+oct));
		}
		Debug("\n");
	}

}

#endif


/*===========================================================================
 EEPROM
 ============================================================================*/

#if ENABLE_EE_DATA

void InitEEPROM()
{
	EEPROM_write(EEPROM_FLAG,'U');
	EEPROM_write(EEPROM_FIRMWARE_INFOS,0x00); // Device Mobius
	EEPROM_write(EEPROM_FIRMWARE_INFOS+1,0x01); // 1er
	EEPROM_write(EEPROM_FIRMWARE_INFOS+2,0x01); // Janvier
	EEPROM_write(EEPROM_FIRMWARE_INFOS+3,0x00); // 2000
	EEPROM_write(EEPROM_FIRMWARE_INFOS+4,0x00); // Minuit 
	EEPROM_write(EEPROM_FIRMWARE_INFOS+5,0x00); // pile
	EEPROM_write(EEPROM_FIRMWARE_INFOS+6,0x00); // binsize 
	EEPROM_write(EEPROM_FIRMWARE_INFOS+7,0x00); // binsize
	EEPROM_write(EEPROM_FIRMWARE_INFOS+8,0x00); // checksum
}


byte EEPROM_read(int address)
{
	eeprom_busy_wait();
	return eeprom_read_byte((unsigned char *) address);
}


void EEPROM_write(int address, byte value)
{
	eeprom_busy_wait();
	eeprom_write_byte((unsigned char *) address, value);
}

#endif


/*===========================================================================
 Communications
 ============================================================================*/

byte SysExDecoder(byte *inSysEx, byte *outData, const byte inLength)
{
	
	byte cnt;
	byte cnt2 = 0;
	byte bits = 0;
	
	for (cnt = 0; cnt < inLength; ++cnt) {
		
		if ((cnt % 8) == 0) {
			bits = inSysEx[cnt];
		} 
		else {
			outData[cnt2++] = inSysEx[cnt] | ((bits & 1) << 7);
			bits >>= 1;
		}
		
	}
	return cnt2;
}


byte GetByteOnUSART0(void)
{
	// uint32_t count = 0;
	
    while(!(UCSR0A & _BV(RXC0))) {
		//	count++;
		//	if (count > MAX_TIME_COUNT) JumpToMainProgram();
	}
    return UDR0;
}


byte GetByteOnUSART1(void)
{
    //uint32_t count = 0;
	
    while(!(UCSR1A & _BV(RXC1))) {
    	//count++;
    	//if (count > MAX_TIME_COUNT) JumpToMainProgram();
	}
    return UDR1;
}


void SendByteOnUSART0(byte data)
{
    while (!(UCSR0A & _BV(UDRE0)));
    UDR0 = data;
}


void SendByteOnUSART1(byte data)
{
    while (!(UCSR1A & _BV(UDRE1)));
    UDR1 = data;
}


void SendACK(byte inPacketNumber)
{
	byte data[6] = {0xF0,0x7E,0x7F,0x7F,inPacketNumber,0xF7};
	word i;
	
#if (UART_MIDI == 1)
	for (i=0;i<6;i++) SendByteOnUSART1(data[i]);
#else 
	for (i=0;i<6;i++) SendByteOnUSART0(data[i]);
#endif
	
}


void SendNAK(byte inPacketNumber)
{
	
	byte data[6] = {0xF0,0x7E,0x7F,0x7E,inPacketNumber,0xF7};
	word i;
	
#if (UART_MIDI == 1)
	for (i=0;i<6;i++) SendByteOnUSART1(data[i]);
#else 
	for (i=0;i<6;i++) SendByteOnUSART0(data[i]);
#endif
	
}


/*===========================================================================
 Parsers & Handlers
 ============================================================================*/

void ParseSysEx(byte * data,word length)
{
	switch (data[1]) {
		case 0x7E: // Universal
			ParseUniversalMessage(data,length);
			return;
			break;
		case 0x00: // Generic
			ParseGenericMessage(data,length);
			return;
			break;
		default:
			break;
	}
}


void ParseGenericMessage(byte * data,word length)
{
	
	byte RebootRequest[7]  = {0xF0,0x00,0x2F,0x2F,0x05,'r',0xF7};
	byte UpgradeRequest[7] = {0xF0,0x00,0x2F,0x2F,0x05,'u',0xF7};
	
	if (!memcmp(RebootRequest,data,(length>7)?7:length)) {
		
		Reboot();
		
	}
	else if (!memcmp(UpgradeRequest,data,(length>7)?7:length)) {
		
#if ENABLE_EE_DATA
		
		EEPROM_write(EEPROM_FLAG,'U');
		eeprom_busy_wait();
		
#endif
		
		// Pas la peine de reboot, on est déjà dans le bootloader (reboot c'est pour un catch du message dans l'app)
		SendACK(0);
		return;
		
	}
	
}


void ParseUniversalMessage(byte * data, word length)
{
	switch (data[5]) {
		case 0x07: // File Dump
			switch (data[6]) {
				case 0x01: // Header (donne le départ du transfert)
					HandleHeader(data,length);
					return;
					break;
				case 0x02: // Data Packet
					HandleDataPacket(data,length);
					return;
					break;
				default: 
					return;
					break;
			}
			break;
		case 0x7B: // EOF
			HandleEOF(data,length);
			return;
			break;
		case 0x06:
			HandleDeviceInquiry(data,length);
			break;
		default: 
			return;
			break;	
	}
	
}


void HandleDeviceInquiry(byte * data,word length)
{
	
	if (data[6] == 0x01) {
		
		// DeviceInquiry, prepare answer
		byte reply[17] = {0xF0,0x7E,0x7F,0x06,0x02,0x00,0x2F,0x2F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xF7};
		byte i;
		
#if ENABLE_EE_DATA
		
		// Load the EEPROM data
		for (i=0;i<6;i++) reply[8+i] = EEPROM_read(EEPROM_FIRMWARE_INFOS+i);
		
#endif
		
		reply[14] = 0x01; // Chunk size: 260 (2 address + 2 count + 256 data)
		reply[15] = 0x04;
		
#if (UART_MIDI == 1)
		for (i=0;i<17;i++) SendByteOnUSART1(reply[i]);
#else
		for (i=0;i<17;i++) SendByteOnUSART0(reply[i]);	
#endif
		
	}
	
}


void HandleHeader(byte * data,word length)
{
	
	if (!memcmp(&data[8],"MFU ",4)) {
		// Décodage des infos
		byte decoded[12] = {0};
		SysExDecoder(&data[12],decoded,14);
		Firmware.device = decoded[0];
		Firmware.day = decoded[1];
		Firmware.month = decoded[2];
		Firmware.year = decoded[3];
		Firmware.hour = decoded[4];
		Firmware.minute = decoded[5];
		Firmware.numberofchunks = decoded[6];
		Firmware.numberofsubchunks = decoded[7]; // Nombre de subchunks par chunk
		Firmware.sizeofsubchunk = decoded[8];
		Firmware.size = ((uint16_t)decoded[9]<<8)|(decoded[10]);
		Firmware.checksum = decoded[11];
		Chunk.CurrentChunkID = -1; // So that the first one is set to 0
		
#if ENABLE_LCD
		LCDClear();
		LCDPrint(0,0,"Update firmware:");
#endif
		
		SendACK(0);
		return;
		
	}
	
	else SendNAK(0);
	
}


void HandleDataPacket(byte * data,word length)
{
	
	word i;
	const byte subchunkIndex = data[7];
	byte checksum = 0;
	
	for (i=1;i<length-2;i++) checksum ^= data[i];
	
	if (checksum != data[length-2]) {
		
		SendNAK(subchunkIndex);
		return;
	}
	
	// Décodage
	byte decoded[68] = {0xFF};
	SysExDecoder(&data[9],decoded,data[8]+1);
	
	if (subchunkIndex == 0) { // Premier paquet du chunk -> reset du Chunk
		Chunk.address = (decoded[0]<<8) | (decoded[1]);		// init de l'adresse
		Chunk.count = ((decoded[2]<<8) | (decoded[3]));		// init du compteur
		for (i=0;i<PAGE_SIZE_BYTES;i++) Chunk.data[i] = 0;	// init des data
		for (i=0;i<Chunk.count;i++) Chunk.data[i] = decoded[i+4];
		Chunk.CurrentChunkID++;		
	}
	else {
		word count = ((decoded[2]<<8) | (decoded[3]));		// decoded data byte count
		// On assume que les données sont calées -> boulot de MobiusUpdater
		for (i=0;i<count;i++) Chunk.data[i+Chunk.count] = decoded[i+4];
		Chunk.count += count;								// MaJ du compteur
	}
	
#if ENABLE_WRITE
	if (subchunkIndex == 3) WriteChunk();	
#endif
	
	SendACK(subchunkIndex);
	
}


// Y'a encore du taf sur celle là
void HandleEOF(byte * data,word length)
{ 
	
	if (Chunk.CurrentChunkID+1 == Firmware.numberofchunks) {
		
#if ENABLE_EE_DATA
		
		// Stockage des infos
		EEPROM_write(EEPROM_FIRMWARE_INFOS,Firmware.device);
		EEPROM_write(EEPROM_FIRMWARE_INFOS+1,Firmware.day);
		EEPROM_write(EEPROM_FIRMWARE_INFOS+2,Firmware.month);
		EEPROM_write(EEPROM_FIRMWARE_INFOS+3,Firmware.year);
		EEPROM_write(EEPROM_FIRMWARE_INFOS+4,Firmware.hour);
		EEPROM_write(EEPROM_FIRMWARE_INFOS+5,Firmware.minute);
		EEPROM_write(EEPROM_FIRMWARE_INFOS+6,(Firmware.size>>8));
		EEPROM_write(EEPROM_FIRMWARE_INFOS+7,(Firmware.size&0xFF));
		
#endif
		
		// Calcul du checksum
		word i;
		byte checksum = 0;
		
		for (i=0;i<Firmware.size;i++) checksum ^= pgm_read_byte_near(i);
		
		if (checksum == Firmware.checksum) {
			
#if ENABLE_EE_DATA
			
			// C'est bon
			EEPROM_write(EEPROM_FIRMWARE_INFOS+8,checksum); // Pour vérifier l'intégrité du programme à chaque démarrage
			// Le firmware est bien enregistré, on remet le flag d'EEPROM à 'I'
			EEPROM_write(EEPROM_FLAG,'I');
			eeprom_busy_wait();
			
#endif
			
			SendACK(0); // On dit au host qu'on a fini
			
#if ENABLE_LCD
			
			LCDPrint(1,0,"       OK       ");
			
#endif
			
			_delay_ms(500);
			Reboot();
			
		}
		else {
			SendNAK(0);
			// Pas cool... 
		}
	}
	else {
		SendNAK(0); // Là on est dans la merde, faut tout recommencer et c'est pas prévu.
		// Reboot(); // pourquoi?
	}
	//SendACK(0);
}


#if ENABLE_WRITE
void WriteChunk()
{
	
	byte sreg = SREG;
	cli();
	eeprom_busy_wait();
	
	boot_page_erase_safe(Chunk.address);
	
	boot_spm_busy_wait();
	
	word address = Chunk.address;
	word i=0;
	for (i=0;i<Chunk.count;i+=2) { // ou 256?
		word temp = Chunk.data[i] | (Chunk.data[i+1]<<8); // little endian
		boot_page_fill_safe(address,temp);
		address+=2;
	}
	boot_page_write_safe(Chunk.address);
	boot_rww_enable_safe();
	SREG = sreg;
	boot_spm_busy_wait();
	
#if ENABLE_LCD	
	char whitesquare[2] = {42,0};
	LCDPrint(1,(Chunk.CurrentChunkID*16)/Firmware.numberofchunks,whitesquare);
#endif
	
}
#endif


/*===========================================================================
 LCD Interface
 ============================================================================*/

#if ENABLE_LCD

void LCDPrint(byte line,byte col,char* text)
{

	Cursor(line,col);
	word i=0,length=strlen(text);
	for (i=0;i<length;i++) SendByteOnUSART0(text[i]);

}


void LCDClear()
{

	SendByteOnUSART0(0xFE);
	SendByteOnUSART0(0x01);

}


void Cursor(byte line,byte col)
{

	line %= 2;      
	col %= 16;
	word offset = (line*64);
	
	SendByteOnUSART0(0xFE);
	SendByteOnUSART0((char)(offset + col + 128));
  	
	_delay_ms(10);

}

#endif


/*===========================================================================
 Main Entry Point
 ============================================================================*/

int main(void)
{
	
	asm volatile("nop\n\t");
	
	//	byte mcusr = MCUSR;
	MCUSR = 0;
	wdt_disable();
	
	cli();
	
	if (pgm_read_byte_near(0x0000) == 0xFF) {
		
		// Aucun programme en mémoire, on va devoir tourner dans le parseur pour attendre un firmware
		
#if ENABLE_EE_DATA
		
		// En attendant, on initialise les données en EEPROM:
		InitEEPROM();
		
#endif
		
	}
	else {
		
#if ENABLE_EE_DATA
		// On a un programme chargé, on teste le flag de lancement de l'app (on pourrait aussi tester la méthode de reset, watchdog ou hardware)
		byte eeread = EEPROM_read(EEPROM_FLAG);
		if (eeread == 'I') JumpToMainProgram();	// Idle -> on lance l'appli
		else if (eeread == 0xFF) InitEEPROM();	// EEPROM corrompue -> reset
		else if (eeread == 'U') SendACK(0);		// Pour dire à l'host de continuer
		// else, bah on continue pour tomber dans le terrier du lapin blanc..
#endif
		
	}
	
	// On lance les comms
	
#if (UART_LCD == 1)
	BEGIN_SERIAL(1,BAUD_RATE_LCD);
#else
	BEGIN_SERIAL(0,BAUD_RATE_LCD);
#endif
	
#if (UART_MIDI == 1)
	BEGIN_SERIAL(1,BAUD_RATE_MIDI);
#else 
	BEGIN_SERIAL(0,BAUD_RATE_MIDI);
#endif

	
	
	
	// ######################################################## lancement du parseur
	
	byte buffer[INPUT_BUFFER_SIZE];
	word index = 0;
	
	for (;;) {
		
		const byte c = GetByteOnUSART1();
		
		if (c == 0xF0) {
			
			// Fill the message buffer
			index = 0;
			buffer[0] = c;
			
			for (index=1;index<INPUT_BUFFER_SIZE && buffer[index-1] != 0xF7;index++) {
				
				buffer[index] = GetByteOnUSART1();
				
			}
			
			// Launch the parser
			ParseSysEx(buffer,index);
			
		}
		
	}
	
	// We shall never fall here,
	// as for(;;) is an infinite loop as well..
	while (1);
	
	return 0;
	
}


