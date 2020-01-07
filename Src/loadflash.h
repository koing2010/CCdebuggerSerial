/*******************************************************************************

File_name:       loadflash.h
Description:    the header file of  Model_c .

*******************************************************************************/
#ifndef     __LOADFLASH_H__
#define     __LOADFLASH_H__

/*==============================================================================
@ Include files
*/
#include "stm32f1xx_hal.h"


/*==============================================================================
@ Typedefs
*/
#define PAGE_SIZE                       1024
#define FALSH_BASE_ADDRESS              0x8000000
#define APP_ADDRES_OFFSET_PAGE          20//offset address = APP_ADDRES_OFFSET_PAGE*PAGE_SIZE
#define APP_ADDRES_START_PAGE           (APP_ADDRES_OFFSET_PAGE + 1)
#define FILE_INFOR_ADDRESS              FALSH_BASE_ADDRESS + APP_ADDRES_OFFSET_PAGE*PAGE_SIZE
#define APP_START_ADDRESS               FALSH_BASE_ADDRESS + APP_ADDRES_START_PAGE*PAGE_SIZE
#define INFO_DATA_SZIE                  PAGE_SIZE


#define MAX_DATA_SIZE   60 // 32bytes + 2btes CRC16
#pragma pack(push,1)//one byte by one byte
typedef  uint8_t    u8;               
typedef  uint16_t   u16;              
typedef  uint32_t   u32;            

/** define the process message status **/
typedef enum
{
  PROCCESS_SUCCESS,
  PROCCESS_FAILED = 1,
  PROCCESS_SUNSUPPORT_CMD,
  PROCCESS_ERRO_LENTH,
  PROCCESS_ERRO_ATTR_LENTH,
	PROCCESS_ERRO_ERASE,
	PROCCESS_ERRO_PROGRAME
} Process_Status;

/** define the command **/
typedef enum
{
  ResetCmd=0x01,
  ResetRspCmd = 0x81,
  ReadDataCmd = 0x02,
  ReadDataRspCmd = 0x82,  
	WriteFlashCmd = 0x03,
  WriteFlashRspCmd = 0x83,
	EraseFullChipCmd = 0x04,
	EraseFullChipRspCmd = 0x84,
	GetChipIDCmd = 0x05,
	GetChipIDCmdRsp = 0x85,
	DefaultRsp = 0x8F   //defaultRsp indicator of rsp
	
} Procees_Cmd;

#define CmdRsp         0x80


/** req data rsp format **/
typedef struct RSP_BIN_DATA_
{
	u32 FileOffset;
	u8  rsp_data_size;
	u8  data[];
}RSP_BIN_DATA,*pRSP_BIN_DATA;

/** req data rsp format **/
typedef enum
{
	BIN_INFO_STATUS,
	BIN_DATA_STATUS,
	BIN_END_STATUS,
}BIN_STATUS;

/** define the attributes **/
typedef enum
{
  SystemDescription = 0x01,
	Manufacture,
	ModelID,
	FileSize = 0x0A,
	FileData = 0x0B
	
}Procees_Attr;


/**define the format of usart data**/
typedef struct PROCESS_MSG_
{
  Procees_Cmd  Command;
  u16  PayoadLenth;//payload data size include CRC16 
  u8  Data[MAX_DATA_SIZE]; 
} PROCESS_MSG,*pPROCESS_MSG;

typedef void(* pDEBUG_FUNCTION_t)(pPROCESS_MSG Msg );


typedef struct  READ_FLASH_
{
	u32 R_address;
	u16 R_size;
	
}READ_FLASH, *pREAD_FLASH;

typedef struct  WRITE_FLASH_
{
	u32 W_address;
	u16 W_size;
	u8  W_data[];
}WRITE_FLASH, *pWRITE_FLASH;

/************Data Parse Status*******************/
typedef enum
{
	DATA_SUCCESS,
	DATA_FILE_MATCHING,
	DATA_INFO_ERROR,
	DATA_FRAME_ERROR,
	DATA_COMPLITED,
	DATA_FILE_LENTH_ERROR,
	DATA_VERIFYING,
	DATA_CRC32_ERROR,
	DATA_VERIFYING_SUCCESS,
	DATA_WAITING_START,
}DATA_PARSE_STATUS;






#define MAGIC_NUMBER  0x16341005ul
#pragma pack(pop)

/*==============================================================================
@ Constants and defines
*/


DATA_PARSE_STATUS ParseCmd( pPROCESS_MSG pMsg);

uint32_t file_crc32(const unsigned char *buf, uint32_t size, uint32_t crc);
uint32_t crc32(const unsigned char *buf, uint32_t size);


u8 ByteCalcFCS( u8 *msg_ptr, u16 len );

#endif
/*@*****************************end of file**********************************@*/


