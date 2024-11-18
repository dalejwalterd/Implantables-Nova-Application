/**
 *  @file eedata.h
 *
 *  @date Created on: Oct 14, 2024
 *  @author: jenej
 */

#ifndef APP_MODULE_EEDATA_H_
#define APP_MODULE_EEDATA_H_

// -------- DEFINITIONS ----------
#define MAX_BYTES_PER_SUBINDEX  32    //TODO: Not sure what happens if subindex exceeds this
#define MAX_EEPROM_MEMORY       0x800 ///< (2KB)
#define EEPROM_RECORD_SIZE      32
#define EEPROM_ERASE_SIZE       0x800 ///< must be divisible into 2048 (2KB) by factor of 2, defines size of chunks erased from EEPROM
#define EEPROM_PAGE_SIZE		1	  ///< XXX: must match the page size in bootloader config.h
#define EEPROM_START_ADDRESS	((127 - EEPROM_PAGE_SIZE) * FLASH_PAGE_SIZE)   ///< Loation of the start of the EEPROM in flash

#define MAX_FLASH_MEMORY        FLASH_SIZE ///< (256KB)
#define FLASH_RECORD_SIZE       32

// --------   DATA   ------------


// -------- PROTOTYPES ----------

UNS8 CheckRestoreFlag(void);
void SaveValues( void );
void RestoreValues( void );
void ResetToODDefault(void);
void ResetModule(void);
void EraseEprom(UNS8);
UNS8 ReadLocalFlashData( UNS32 nvAddress, UNS8 * data, UNS8 numData );
UNS8 EEPROM_open();
UNS8 EEPROM_commit();
void EEPROM_read(UNS16 address, UNS8 * data, UNS16 length);
void EEPROM_write(UNS16 address, UNS8 * data, UNS16 length);

#endif /* APP_MODULE_EEDATA_H_ */
