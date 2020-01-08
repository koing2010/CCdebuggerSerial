/*******************************************************************************
Copyright:
File_name:       loadfalsh.c
Version:	       0.0
Revised:        $Date:2016-10-26; $
Description:    the main file of this project.
Notes:          This version targets the stm32
Editor:		      Mr.kon

*******************************************************************************/


/*==============================================================================
@ Include files
*/
#include "stm32f1xx_hal.h"
#include "loadflash.h"
#include "freeRTOS.h"
#include "string.h"
#include "ccddriver.h"
#include "usb_device.h"

/*==============================================================================
@ Global variable
*/
extern uint8_t USB_DataRequest(uint8_t cmd, uint8_t* pData, uint16_t lenth, uint8_t sequence);

static void ChipReset(pPROCESS_MSG pMsg);
static void ChipErase(pPROCESS_MSG pMsg);
static void ReadFlashData(pPROCESS_MSG pMsg);

static void WriteFlashData(pPROCESS_MSG pMsg);
static void GetChipid(pPROCESS_MSG pMsg);

#define SINGLE_REQ_SIZE           128
#define REQ_BINDATA_CMD           0x02
#define RSP_BINDATA_CMD           0x82



/*==============================================================================
@ All functions  as follow
*/
static 


#define MAX_FUNCTION_CMD  5
pDEBUG_FUNCTION_t DebugFunctionCB[MAX_FUNCTION_CMD]=
{
	ChipReset,      //ResetCmd=0x01,
	ReadFlashData,  //ReadFlashDataCmd = 0x02,
	WriteFlashData, //WriteFlashCmd = 0x03,
	ChipErase,		  //EraseFullChipCmd = 0x04,
	GetChipid			  //GetChipIDCmd = 0x05,

};

/*******************************************************************************
Function: ParseCmd
Description:

Input:        pPROCESS_MSG pMsg
Output:       None
Return:       DATA_PARSE_STATUS
Editor:	      Mr.kon
Others:	      2019-12-25
*******************************************************************************/
DATA_PARSE_STATUS ParseCmd( pPROCESS_MSG pMsg)
{
	if(pMsg->Command <= MAX_FUNCTION_CMD)
	{
		USBD_UsrLog("RxCmd %X ",pMsg->Command );
		DebugFunctionCB[pMsg->Command-1](pMsg);
	}

} 


uint8_t chip_info[3];
static void GetChipid(pPROCESS_MSG pMsg)
{
	GET_CHIP_ID(&chip_info[1], &chip_info[2]);
	
	USBD_UsrLog("chip_id= %X,chip_rev= %X ", chip_info[1], chip_info[2]);
	chip_info[0] = 0x00; 
	
	USB_DataRequest(  pMsg->Command | CmdRsp, &chip_info[0], sizeof(chip_info), pMsg->Sequence);
}
	
uint8_t retry = 0;
 void ChipReset(pPROCESS_MSG pMsg)
{

	DEBUG_INIT();
	
	osDelay(10);
	write_xdata_memory(DUP_CLKCONCMD, 0x80);
  while (read_xdata_memory(DUP_CLKCONSTA) != 0x80 && retry ++ < 16)
	{	
		osDelay(10);
	}
		
	
	uint8_t status = PROCCESS_SUCCESS;
	USB_DataRequest(  pMsg->Command | CmdRsp, &status, sizeof(status), pMsg->Sequence);
}

 static void ChipErase(pPROCESS_MSG pMsg)
{
  chip_erase();
	uint8_t status = PROCCESS_SUCCESS;
	USB_DataRequest(  pMsg->Command | CmdRsp, &status, sizeof(status), pMsg->Sequence);
}

pREAD_FLASH pReadFlash = NULL;
static void ReadFlashData(pPROCESS_MSG pMsg)
{
	if( pMsg->PayoadLenth ==  sizeof( READ_FLASH)+ 4 )
	{
		pReadFlash = (pREAD_FLASH)pMsg->Data;
		/****************************************
		* Read FLASH memory
		*****************************************/
		if(pReadFlash->R_size <=2048 )
		{
			pWRITE_FLASH_RSP	pReadRsp = (pWRITE_FLASH_RSP)pvPortMalloc( pReadFlash->R_size + sizeof(u32) + sizeof(u16));
			
			if(pReadRsp)
			{
				// Read 4 bytes starting at flash address 0x0100 (flash bank 0)
				read_flash_memory_block(pReadFlash->R_address/0x8000, pReadFlash->R_address%0x8000, pReadFlash->R_size, pReadRsp->r_data); // Bank, address, count, dest.
				
				uint8_t i = 0;
				pReadRsp->r_status = PROCCESS_SUCCESS;
				pReadRsp->r_address = pReadFlash->R_address;
				pReadRsp->r_size  = pReadFlash->R_size;
				
				while( ( USB_DataRequest(  pMsg->Command|CmdRsp,(uint8_t*)pReadRsp, pReadFlash->R_size + sizeof(WRITE_FLASH_RSP), pMsg->Sequence) == USBD_BUSY) && (i <= 3) )
				{
					i ++;
				}
				
				if(i > 3)
				{
					uint8_t status = PROCCESS_FAILED;
					USB_DataRequest(   pMsg->Command|CmdRsp, &status, sizeof(status), pMsg->Sequence);
				}
				
				vPortFree(pReadRsp);
			}
		}
		
	}
}

pWRITE_FLASH pWdata;
uint8_t bank;
static void WriteFlashData(pPROCESS_MSG pMsg)
{
	pWdata = (pWRITE_FLASH)pMsg->Data;
	uint8_t status ;

	if( pMsg->PayoadLenth ==  pWdata->W_size+ 10)
	{
			/****************************************
	* Write FLASH memory
	*****************************************/
	// Enable DMA (Disable DMA_PAUSE bit in debug configuration)
	uint8_t debug_config = 0x22;
	debug_command(CMD_WR_CONFIG, &debug_config, 1);


		
	// Program data (start address must be word aligned [32 bit])
	write_flash_memory_block( pWdata->W_data,  pWdata->W_address, pWdata->W_size); // src, address, count



	/****************************************
	* Verification
	*****************************************/
	//Bytwise check of read and written data
			uint8_t *	pData = pvPortMalloc( pWdata->W_size);




			status = PROCCESS_SUCCESS;
			if(pData)
			{   
				/****************************************
				* Read FLASH memory
				*****************************************/
				bank =  pWdata->W_address/0x8000;
				
				// Read 4 bytes starting at flash address 0x0100 (flash bank 0)
				read_flash_memory_block(bank,  pWdata->W_address %0x8000,  pWdata->W_size, pData); // Bank, address, count, dest.
								
				for(unsigned int i = 0; i < pWdata->W_size; i++) 
				{
						if(pData[i] != pWdata->W_data[i])
							{
								status = PROCCESS_FAILED;
								break;
							}
							
				}
				vPortFree(pData);
				
			}
			else
			{
				status = PROCCESS_FAILED;
			}
			
			USBD_UsrLog("Verify status= %d", status);
			
				USB_DataRequest(  pMsg->Command | CmdRsp, &status, sizeof(status), pMsg->Sequence );
	}
}

/*********************************************************************************
Function:  ModBus_CRC16
Description:
Calculate  ModBus_CRC16

Input:       uchar *puchMsg :   the buffer address
           uchar crc_count :   length of data buffer

Output:       None
Return:       CRC
Others:	      don't care
*********************************************************************************/
u16 ModBus_CRC16(u8 *puchMsg,u8 crc_count)
{
    u8 i ,j;
    u8 XORResult;
    u16 xorCRC = 0xA001; //??CRC????
    u16 CRCdata = 0xFFFF;
    for ( i = 0; i <crc_count; i++)
    {
        CRCdata ^= puchMsg[i];
        for (j = 0; j < 8; j++)
        {
            XORResult = CRCdata & 1;
            CRCdata >>= 1;
            if (XORResult)
                CRCdata ^= xorCRC;
        }
    }

    return  CRCdata;
}

/*********************************************************************************
Function:  ByteCheckSum
Description:
byte by byte

Input:       uchar *puchMsg :   the buffer address
           uchar crc_count :   length of data buffer

Output:       None
Return:       Sum
Others:	      don't care 2019-10-31
*********************************************************************************/
u8 ByteCheckSum( u8 *puchMsg,u16 crc_count )
{
   u8 sum = 0;
	
	while(crc_count-- )
	{
	 sum += *puchMsg++;
	} 
	return sum;
}

/***************************************************************************************************
 * @fn      ByteCalcFCS
 *
 * @brief   Calculate the FCS of a message buffer by XOR'ing each byte.
 *          Remember to NOT include SOP and FCS fields, so start at the CMD field.
 *
 * @param   byte *msg_ptr - message pointer
 * @param   byte len - length (in bytes) of message
 *
 * @return  result byte
 ***************************************************************************************************/
u8 ByteCalcFCS( u8 *msg_ptr, u16 len )
{
  u16 x;
  u8 xorResult;

  xorResult = 0;

  for ( x = 0; x < len; x++, msg_ptr++ )
    xorResult = xorResult ^ *msg_ptr;

  return ( xorResult );
}
uint32_t Crc32Table[256] = {
  0x00000000,0x77073096,0xEE0E612C,0x990951BA,0x076DC419,0x706AF48F,0xE963A535,0x9E6495A3,
  0x0EDB8832,0x79DCB8A4,0xE0D5E91E,0x97D2D988,0x09B64C2B,0x7EB17CBD,0xE7B82D07,0x90BF1D91,
  0x1DB71064,0x6AB020F2,0xF3B97148,0x84BE41DE,0x1ADAD47D,0x6DDDE4EB,0xF4D4B551,0x83D385C7,
  0x136C9856,0x646BA8C0,0xFD62F97A,0x8A65C9EC,0x14015C4F,0x63066CD9,0xFA0F3D63,0x8D080DF5,
  0x3B6E20C8,0x4C69105E,0xD56041E4,0xA2677172,0x3C03E4D1,0x4B04D447,0xD20D85FD,0xA50AB56B,
  0x35B5A8FA,0x42B2986C,0xDBBBC9D6,0xACBCF940,0x32D86CE3,0x45DF5C75,0xDCD60DCF,0xABD13D59,
  0x26D930AC,0x51DE003A,0xC8D75180,0xBFD06116,0x21B4F4B5,0x56B3C423,0xCFBA9599,0xB8BDA50F,
  0x2802B89E,0x5F058808,0xC60CD9B2,0xB10BE924,0x2F6F7C87,0x58684C11,0xC1611DAB,0xB6662D3D,
  0x76DC4190,0x01DB7106,0x98D220BC,0xEFD5102A,0x71B18589,0x06B6B51F,0x9FBFE4A5,0xE8B8D433,
  0x7807C9A2,0x0F00F934,0x9609A88E,0xE10E9818,0x7F6A0DBB,0x086D3D2D,0x91646C97,0xE6635C01,
  0x6B6B51F4,0x1C6C6162,0x856530D8,0xF262004E,0x6C0695ED,0x1B01A57B,0x8208F4C1,0xF50FC457,
  0x65B0D9C6,0x12B7E950,0x8BBEB8EA,0xFCB9887C,0x62DD1DDF,0x15DA2D49,0x8CD37CF3,0xFBD44C65,
  0x4DB26158,0x3AB551CE,0xA3BC0074,0xD4BB30E2,0x4ADFA541,0x3DD895D7,0xA4D1C46D,0xD3D6F4FB,
  0x4369E96A,0x346ED9FC,0xAD678846,0xDA60B8D0,0x44042D73,0x33031DE5,0xAA0A4C5F,0xDD0D7CC9,
  0x5005713C,0x270241AA,0xBE0B1010,0xC90C2086,0x5768B525,0x206F85B3,0xB966D409,0xCE61E49F,
  0x5EDEF90E,0x29D9C998,0xB0D09822,0xC7D7A8B4,0x59B33D17,0x2EB40D81,0xB7BD5C3B,0xC0BA6CAD,
  0xEDB88320,0x9ABFB3B6,0x03B6E20C,0x74B1D29A,0xEAD54739,0x9DD277AF,0x04DB2615,0x73DC1683,
  0xE3630B12,0x94643B84,0x0D6D6A3E,0x7A6A5AA8,0xE40ECF0B,0x9309FF9D,0x0A00AE27,0x7D079EB1,
  0xF00F9344,0x8708A3D2,0x1E01F268,0x6906C2FE,0xF762575D,0x806567CB,0x196C3671,0x6E6B06E7,
  0xFED41B76,0x89D32BE0,0x10DA7A5A,0x67DD4ACC,0xF9B9DF6F,0x8EBEEFF9,0x17B7BE43,0x60B08ED5,
  0xD6D6A3E8,0xA1D1937E,0x38D8C2C4,0x4FDFF252,0xD1BB67F1,0xA6BC5767,0x3FB506DD,0x48B2364B,
  0xD80D2BDA,0xAF0A1B4C,0x36034AF6,0x41047A60,0xDF60EFC3,0xA867DF55,0x316E8EEF,0x4669BE79,
  0xCB61B38C,0xBC66831A,0x256FD2A0,0x5268E236,0xCC0C7795,0xBB0B4703,0x220216B9,0x5505262F,
  0xC5BA3BBE,0xB2BD0B28,0x2BB45A92,0x5CB36A04,0xC2D7FFA7,0xB5D0CF31,0x2CD99E8B,0x5BDEAE1D,
  0x9B64C2B0,0xEC63F226,0x756AA39C,0x026D930A,0x9C0906A9,0xEB0E363F,0x72076785,0x05005713,
  0x95BF4A82,0xE2B87A14,0x7BB12BAE,0x0CB61B38,0x92D28E9B,0xE5D5BE0D,0x7CDCEFB7,0x0BDBDF21,
  0x86D3D2D4,0xF1D4E242,0x68DDB3F8,0x1FDA836E,0x81BE16CD,0xF6B9265B,0x6FB077E1,0x18B74777,
  0x88085AE6,0xFF0F6A70,0x66063BCA,0x11010B5C,0x8F659EFF,0xF862AE69,0x616BFFD3,0x166CCF45,
  0xA00AE278,0xD70DD2EE,0x4E048354,0x3903B3C2,0xA7672661,0xD06016F7,0x4969474D,0x3E6E77DB,
  0xAED16A4A,0xD9D65ADC,0x40DF0B66,0x37D83BF0,0xA9BCAE53,0xDEBB9EC5,0x47B2CF7F,0x30B5FFE9,
  0xBDBDF21C,0xCABAC28A,0x53B39330,0x24B4A3A6,0xBAD03605,0xCDD70693,0x54DE5729,0x23D967BF,
  0xB3667A2E,0xC4614AB8,0x5D681B02,0x2A6F2B94,0xB40BBE37,0xC30C8EA1,0x5A05DF1B,0x2D02EF8D
};


uint32_t crc32(const unsigned char *buf, uint32_t size)
{
  uint32_t i, crc;
  crc = 0xFFFFFFFF;
  for (i = 0; i < size; i++)
    crc = Crc32Table[(crc ^ buf[i]) & 0xff] ^ (crc >> 8);
  return crc^0xFFFFFFFF;
}


uint32_t file_crc32(const unsigned char *buf, uint32_t size, uint32_t crc)
{
  uint16_t i;
  for (i = 0; i < size; i++)
    crc = Crc32Table[(crc ^ buf[i]) & 0xff] ^ (crc >> 8);
  return crc;
}
/*@*****************************end of file**********************************@*/
