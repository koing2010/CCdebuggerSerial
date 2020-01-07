/*******************************************************************************
Copyright:
File_name:       ccddriver.c
Version:	     0.0
Revised:        $Date:2018-2-6  ; $
Description:    the main file of this project.
Notes:          This version targets the SH99F01
Editor:		    Mr.kon

*******************************************************************************/


/*==============================================================================
@ Include files
*/
#include "ccddriver.h"
#include "string.h"
#include "cmsis_os.h"

#ifndef NULL
#define NULL ((void *)0)
#endif


/*==============================================================================
@ Global variable
*/


#define LOBYTE(w) ((uint8_t)(w))
#define HIBYTE(w) ((uint8_t)(((uint16_t)(w) >> 8) & 0xFF))

//! Convert XREG register declaration to an XDATA integer address
#define XREG_TO_INT(a)      ((unsigned short)(&(a)))
static GPIO_InitTypeDef GPIO_InitStruct;
unsigned char wait_dup_ready(void);

const unsigned char dma_desc_0[8] =
{
    // Debug Interface -> Buffer
    HIBYTE(DUP_DBGDATA),            // src[15:8]
    LOBYTE(DUP_DBGDATA),            // src[7:0]
    HIBYTE(ADDR_BUF0),              // dest[15:8]
    LOBYTE(ADDR_BUF0),              // dest[7:0]
    0,                              // len[12:8] - filled in later
    0,                              // len[7:0]
    31,                             // trigger: DBG_BW
    0x11                            // increment destination
};
//! DUP DMA descriptor
const unsigned char dma_desc_1[8] =
{
    // Buffer -> Flash controller
    HIBYTE(ADDR_BUF0),              // src[15:8]
    LOBYTE(ADDR_BUF0),              // src[7:0]
    HIBYTE(DUP_FWDATA),             // dest[15:8]
    LOBYTE(DUP_FWDATA),             // dest[7:0]
    0,                              // len[12:8] - filled in later
    0,                              // len[7:0]
    18,                             // trigger: FLASH
    0x42,                           // increment source
};

/*==============================================================================
@ All functions  as follow
*/

#define CLK_DELAY   		__nop();__nop();

//about 125 ns
void DeBugDelay(unsigned int N)
{
  do
    {
      N --;
    }
  while(N);
}

static void DBG_Turn_DD_Dir_OUT()
{

  /*Configure GPIO pins : DBG_DD_PIN_Pin*/ 
  GPIO_InitStruct.Pin = DBG_DD_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DBG_DD_PIN_GPIO_Port, &GPIO_InitStruct);
	
}

static void DBG_Turn_DD_Dir_IN()
{

  /*Configure GPIO pins : DBG_DD_PIN_Pin*/
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  HAL_GPIO_Init(DBG_DD_PIN_GPIO_Port, &GPIO_InitStruct); 
	
	//DBG_DD_H;
}
/*******************************************************************************
Function: DEBUG_INIT
Description:

Resets the chip for debug mode.

Input:        None
Output:       None
Return:       None
Editor:	      Mr.kon
Others:	      2018-2-6
*******************************************************************************/
void DEBUG_INIT( void )
{

  DBG_Turn_DD_Dir_OUT();

  DBG_DC_L;

  osDelay(20);//may 20ms

  DBG_RST_L;

  osDelay(20);//may 20ms

// need to rising edge to enter DEBUG mode
  DBG_DC_H;
  DeBugDelay(2);

  DBG_DC_L;
  DeBugDelay(2);

  DBG_DC_H;
  DeBugDelay(2);

  DBG_DC_L;


  osDelay(20);//may 200us
  DBG_RST_H;

}
/*******************************************************************************
Function: DBG_WRITE_BYTE
Description:

falling edge the mcu read the DD line status

Input:        None
Output:       None
Return:       None
Editor:	      Mr.kon
Others:	      2018-2-6
*******************************************************************************/
void DBG_WRITE_BYTE(uint8_t Byte)
{
  uint8_t i = 0;

  for( ; i < 8; i ++)
    {
      DBG_DC_H;//CLK high

      if((Byte & 0x80))
        {
          DBG_DD_H;
        }
      else
        {
          DBG_DD_L;
        }

      DBG_DC_L;//CLK low,target dev read the data

      Byte <<= 1;

    }
}

uint8_t DBG_READ_BYTE(void)
{
  uint8_t i = 0,Byte =0;

  for( ; i < 8; i ++)
    {
      Byte <<= 1;
      DBG_DC_H;//CLK high

      if(DBG_DD_READ)
        {
          Byte |= 0x01;;
        }

      DBG_DC_L;//CLK low, target dev read the data

    }
  return Byte;
}
/*******************************************************************************
Function: DEBUG_INIT
Description:

Resets the chip for debug mode.

Input:        None
Output:       None
Return:       None
Editor:	      Mr.kon
Others:	      2018-2-6
*******************************************************************************/
void GET_CHIP_ID(uint8_t* chip_id_8, uint8_t* chip_rev_8)
{
  DBG_Turn_DD_Dir_OUT();

  DBG_WRITE_BYTE( GET_CHIP_ID_CMD );

  DBG_Turn_DD_Dir_IN();

  wait_dup_ready();

  *chip_id_8  = DBG_READ_BYTE(); //read chip id
  *chip_rev_8 = DBG_READ_BYTE(); //read chip rev

  DBG_Turn_DD_Dir_OUT();

}

/*******************************************************************************
Function: CHIP_ERASE
Description:

Erases the entire flash memory, including lock bits.
Debug command header = 0x14.
Input:        None
Output:       None
Return:       None
Editor:	      Mr.kon
Others:	      2018-2-7
*******************************************************************************/
void CHIP_ERASE( void )
{
  DBG_Turn_DD_Dir_OUT();

  DBG_WRITE_BYTE( CHIP_ERASE_CMD );

  DBG_Turn_DD_Dir_IN();
	
  wait_dup_ready();

  DBG_READ_BYTE();//discard the data returned

  DBG_Turn_DD_Dir_OUT();
}

/*******************************************************************************
Function: WR_CONFIG(IN: config_8)
Description:

Writes the debug configuration byte, which contains the following bits:
  0x08 - TIMERS_OFF
  0x04 - DMA_PAUSE
  0x02 - TIMER_SUSPEND
  0x01 - SEL_FLASH_INFO_PAGE
Debug command header = 0x1D.
Input:        None
Output:       None
Return:       None
Editor:	      Mr.kon
Others:	      2018-2-7
*******************************************************************************/
void WR_CONFIG(W_CFG config_8)
{
  DBG_Turn_DD_Dir_OUT();

  DBG_WRITE_BYTE( WR_CONFIG_CMD );//cmd

  DBG_WRITE_BYTE( (uint8_t)config_8 );// write config

  DBG_Turn_DD_Dir_IN();
	
  wait_dup_ready();

  DBG_READ_BYTE();//discard the data returned

  DBG_Turn_DD_Dir_OUT();
}

/*******************************************************************************
Function: READ_STATUS(OUT: status_8)
Description:

Reads the debug status byte, which contains the following bits:
  0x80 - CHIP_ERASE_DONE
  0x40 - PCON_IDLE
  0x20 - CPU_HALTED
  0x10 - POWER_MODE_0
  0x08 - HALT_STATUS
  0x04 - DEBUG_LOCKED
  0x02 - OSCILLATOR_STABLE
  0x01 - STACK_OVERFLOW

Debug command header = 0x34.

Input:        None
Output:       None
Return:       None
Editor:	      Mr.kon
Others:	      2018-2-7
*******************************************************************************/
void READ_STATUS(uint8_t* status_8 )
{

  DBG_Turn_DD_Dir_OUT();

  DBG_WRITE_BYTE( READ_STATUS_CMD );

  DBG_Turn_DD_Dir_IN();
	
  wait_dup_ready();

  *status_8 = DBG_READ_BYTE();//return the data returned

  DBG_Turn_DD_Dir_OUT();
}
/*******************************************************************************
Function: HALT()
Description:

Halts the CPU
Debug command header = 0x44.

Input:        None
Output:       None
Return:       None
Editor:	      Mr.kon
Others:	      2018-2-7
*******************************************************************************/
void HALT( void )
{
  DBG_Turn_DD_Dir_OUT();

  DBG_WRITE_BYTE( HALT_CMD );

  DBG_Turn_DD_Dir_IN();
	
  wait_dup_ready();

  DBG_READ_BYTE();//discard the data returned

  DBG_Turn_DD_Dir_OUT();
}

/*******************************************************************************
Function: RESUME
Description:

Starts/resumes the CPU
Debug command header = 0x4C.

Input:        None
Output:       None
Return:       None
Editor:	      Mr.kon
Others:	      2018-2-7
*******************************************************************************/
void RESUME( void )
{
  DBG_Turn_DD_Dir_OUT();

  DBG_WRITE_BYTE( RESUME_CMD );

  DBG_Turn_DD_Dir_IN();
	
  wait_dup_ready();

  DBG_READ_BYTE();//discard the data returned

  DBG_Turn_DD_Dir_OUT();
}



unsigned char wait_dup_ready(void)
{
    // DUP pulls DD low when ready
    unsigned char count = 0;
    while (DBG_DD_READ != GPIO_PIN_RESET && count < 16)
    {
        // Clock out 8 bits before checking if DD is low again
        DBG_READ_BYTE();
        count++;
    }
    return (count == 16) ? 0 : 1;
}
/**************************************************************************//**
* @brief    Issues a command on the debug interface. Only commands that return
*           one output byte are supported.
*
* @param    cmd             Command byte
* @param    cmd_bytes       Pointer to the array of data bytes following the
*                           command byte [0-3]
* @param    num_cmd_bytes   The number of data bytes (input to DUP) [0-3]
*
* @return   Data returned by command
******************************************************************************/
unsigned char debug_command(unsigned char cmd, unsigned char *cmd_bytes,
                            unsigned short num_cmd_bytes)
{
    unsigned short i;
    unsigned char output = 0;
taskENTER_CRITICAL();
    // Make sure DD is output
    DBG_Turn_DD_Dir_OUT();

    // Send command
    DBG_WRITE_BYTE(cmd);

    // Send bytes
    for (i = 0; i < num_cmd_bytes; i++)
    {
        DBG_WRITE_BYTE(cmd_bytes[i]);
    }

    // Set DD as input
     DBG_Turn_DD_Dir_IN();

    // Wait for data to be ready
    wait_dup_ready();

    // Read returned byte
    output =  DBG_READ_BYTE();

    // Set DD as output
     DBG_Turn_DD_Dir_OUT();
taskEXIT_CRITICAL();
    return output;
}
/**************************************************************************//**
* @brief    Sends a block of data over the debug interface using the
*           BURST_WRITE command.
*
* @param    src         Pointer to the array of input bytes
* @param    num_bytes   The number of input bytes
*
* @return   None.
******************************************************************************/
void burst_write_block(unsigned char *src, unsigned short num_bytes)
{
    unsigned short i;
taskENTER_CRITICAL();
    // Make sure DD is output
       DBG_Turn_DD_Dir_OUT();

    DBG_WRITE_BYTE(CMD_BURST_WRITE | (HIBYTE(num_bytes)&0x07) );
    DBG_WRITE_BYTE(LOBYTE(num_bytes));
    for (i = 0; i < num_bytes; i++)
    {
        DBG_WRITE_BYTE(src[i]);
    }

    // Set DD as input
    DBG_Turn_DD_Dir_IN();

    // Wait for DUP to be ready
    wait_dup_ready();

    DBG_READ_BYTE(); // ignore output

    // Set DD as output
    DBG_Turn_DD_Dir_OUT();
taskEXIT_CRITICAL();
}
/**************************************************************************//**
* @brief    Writes a byte to a specific address in the DUP's XDATA space.
*
* @param    address     XDATA address
* @param    value       Value to write
*
* @return   None.
******************************************************************************/
void write_xdata_memory(unsigned short address, unsigned char value)
{
    unsigned char instr[3];
taskENTER_CRITICAL();
    // MOV DPTR, address
    instr[0] = 0x90;
    instr[1] = HIBYTE(address);
    instr[2] = LOBYTE(address);
    debug_command(CMD_DEBUG_INSTR_3B, instr, 3);

    // MOV A, values[i]
    instr[0] = 0x74;
    instr[1] = value;
    debug_command(CMD_DEBUG_INSTR_2B, instr, 2);

    // MOV @DPTR, A
    instr[0] = 0xF0;
    debug_command(CMD_DEBUG_INSTR_1B, instr, 1);
taskEXIT_CRITICAL();
}
/**************************************************************************//**
* @brief    Read a byte from a specific address in the DUP's XDATA space.
*
* @param    address     XDATA address
*
* @return   Value read from XDATA
******************************************************************************/
unsigned char read_xdata_memory(unsigned short address)
{
    unsigned char instr[3];
taskENTER_CRITICAL();
    // MOV DPTR, address
    instr[0] = 0x90;
    instr[1] = HIBYTE(address);
    instr[2] = LOBYTE(address);
    debug_command(CMD_DEBUG_INSTR_3B, instr, 3);

    // MOVX A, @DPTR
    instr[0] = 0xE0;
taskEXIT_CRITICAL();
    return debug_command(CMD_DEBUG_INSTR_1B, instr, 1);
}

/**************************************************************************//**
* @brief    Issues a CHIP_ERASE command on the debug interface and waits for it
*           to complete.
*
* @return   None.
******************************************************************************/
void chip_erase(void)
{
    volatile unsigned char status;
taskENTER_CRITICAL();
    // Send command
    debug_command(CMD_CHIP_ERASE, 0, 0);

    // Wait for status bit 7 to go low
    do {
        status = debug_command(CMD_READ_STATUS, 0, 0);
    } while((status & STATUS_CHIP_ERASE_BUSY_BM));
taskEXIT_CRITICAL();
}

/**************************************************************************//**
* @brief    Writes a block of data to the DUP's XDATA space.
*
* @param    address     XDATA start address
* @param    values      Pointer to the array of bytes to write
* @param    num_bytes   Number of bytes to write
*
* @return   None.
******************************************************************************/
void write_xdata_memory_block(unsigned short address,
                              const unsigned char *values,
                              unsigned short num_bytes)
{
    unsigned char instr[3];
    unsigned short i;
taskENTER_CRITICAL();
    // MOV DPTR, address
    instr[0] = 0x90;
    instr[1] = HIBYTE(address);
    instr[2] = LOBYTE(address);
    debug_command(CMD_DEBUG_INSTR_3B, instr, 3);

    for (i = 0; i < num_bytes; i++)
    {
        // MOV A, values[i]
        instr[0] = 0x74;
        instr[1] = values[i];
        debug_command(CMD_DEBUG_INSTR_2B, instr, 2);

        // MOV @DPTR, A
        instr[0] = 0xF0;
        debug_command(CMD_DEBUG_INSTR_1B, instr, 1);

        // INC DPTR
        instr[0] = 0xA3;
        debug_command(CMD_DEBUG_INSTR_1B, instr, 1);
    }
taskEXIT_CRITICAL();
}

/**************************************************************************//**
* @brief    Reads 1-32767 bytes from DUP's flash to a given buffer on the
*           programmer.
*
* @param    bank        Flash bank to read from [0-7]
* @param    address     Flash memory start address [0x0000 - 0x7FFF]
* @param    values      Pointer to destination buffer.
*
* @return   None.
******************************************************************************/
void read_flash_memory_block(unsigned char bank,unsigned short flash_addr,
                             unsigned short num_bytes, unsigned char *values)
{
taskENTER_CRITICAL();
    unsigned char instr[3];
    unsigned short i;
    unsigned short xdata_addr = (0x8000 + flash_addr);

    // 1. Map flash memory bank to XDATA address 0x8000-0xFFFF
    write_xdata_memory(DUP_MEMCTR, bank);

    // 2. Move data pointer to XDATA address (MOV DPTR, xdata_addr)
    instr[0] = 0x90;
    instr[1] = HIBYTE(xdata_addr);
    instr[2] = LOBYTE(xdata_addr);
    debug_command(CMD_DEBUG_INSTR_3B, instr, 3);

    for (i = 0; i < num_bytes; i++)
    {
        // 3. Move value pointed to by DPTR to accumulator (MOVX A, @DPTR)
        instr[0] = 0xE0;
        values[i] = debug_command(CMD_DEBUG_INSTR_1B, instr, 1);

        // 4. Increment data pointer (INC DPTR)
        instr[0] = 0xA3;
        debug_command(CMD_DEBUG_INSTR_1B, instr, 1);
    }
taskEXIT_CRITICAL();
}


/**************************************************************************//**
* @brief    Writes 4-2048 bytes to DUP's flash memory. Parameter \c num_bytes
*           must be a multiple of 4.
*
* @param    src         Pointer to programmer's source buffer (in XDATA space)
* @param    start_addr  FLASH memory start address [0x0000 - 0x7FFF]
* @param    num_bytes   Number of bytes to transfer [4-2048]
*
* @return   None.
******************************************************************************/
void write_flash_memory_block(unsigned char *src, unsigned long start_addr,
                              unsigned short num_bytes)
{
taskENTER_CRITICAL();
    // 1. Write the 2 DMA descriptors to RAM
    write_xdata_memory_block(ADDR_DMA_DESC_0, dma_desc_0, 8);
    write_xdata_memory_block(ADDR_DMA_DESC_1, dma_desc_1, 8);

    // 2. Update LEN value in DUP's DMA descriptors
    unsigned char len[2] = {HIBYTE(num_bytes), LOBYTE(num_bytes)};
    write_xdata_memory_block((ADDR_DMA_DESC_0+4), len, 2);  // LEN, DBG => ram
    write_xdata_memory_block((ADDR_DMA_DESC_1+4), len, 2);  // LEN, ram => flash

    // 3. Set DMA controller pointer to the DMA descriptors
    write_xdata_memory(DUP_DMA0CFGH, HIBYTE(ADDR_DMA_DESC_0));
    write_xdata_memory(DUP_DMA0CFGL, LOBYTE(ADDR_DMA_DESC_0));
    write_xdata_memory(DUP_DMA1CFGH, HIBYTE(ADDR_DMA_DESC_1));
    write_xdata_memory(DUP_DMA1CFGL, LOBYTE(ADDR_DMA_DESC_1));

    // 4. Set Flash controller start address (wants 16MSb of 18 bit address)
    write_xdata_memory(DUP_FADDRH, HIBYTE( (start_addr>>2) ));
    write_xdata_memory(DUP_FADDRL, LOBYTE( (start_addr>>2) ));

    // 5. Arm DBG=>buffer DMA channel and start burst write
    write_xdata_memory(DUP_DMAARM, CH_DBG_TO_BUF0);
    burst_write_block(src, num_bytes);

    // 6. Start programming: buffer to flash
    write_xdata_memory(DUP_DMAARM, CH_BUF0_TO_FLASH);
    write_xdata_memory(DUP_FCTL, 0x06);

    // 7. Wait until flash controller is done
    while (read_xdata_memory(XREG_TO_INT(FCTL)) & 0x80);
taskEXIT_CRITICAL();
}

/*@*****************************end of file**********************************@*/
