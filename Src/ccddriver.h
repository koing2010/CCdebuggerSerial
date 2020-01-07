/*******************************************************************************

File_name:       Model.h
Description:    the header file of  Model_c .

*******************************************************************************/
#ifndef     __CCDDRIVER_H
#define     __CCDDRIVER_H
/*==============================================================================
@ Include files
*/
#include "stm32f1xx_hal.h"

/*==============================================================================
@ Typedefs
*/
void CLOCK_INIT( void );
void HALT( void );
void DEBUG_INIT( void );
void GET_CHIP_ID(uint8_t* chip_id_8, uint8_t* chip_rev_8);
void WRITE_FLASH_PAGE(uint32_t address_17, uint8_t* inputArray_8,uint8_t  erase_page_1);
void READ_FLASH_PAGE(uint32_t linearAddress_17, uint8_t* outputArray_8);

/*==============================================================================
@ Constants and defines
*/

#define BITBAND(addr, bitnum)			((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)					*((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)			MEM_ADDR(BITBAND(addr, bitnum)) 

/**********************************************************
                       GPIO地址映射
              基地址加上寄存器偏移地址组成
**********************************************************/

#define GPIOA_ODR_Addr    (GPIOA_BASE+12)	//0x4001080C 
#define GPIOB_ODR_Addr    (GPIOB_BASE+12)	//0x40010C0C 
#define GPIOC_ODR_Addr    (GPIOC_BASE+12)	//0x4001100C 
#define GPIOD_ODR_Addr    (GPIOD_BASE+12)	//0x4001140C 
#define GPIOE_ODR_Addr    (GPIOE_BASE+12)	//0x4001180C 
#define GPIOF_ODR_Addr    (GPIOF_BASE+12)	//0x40011A0C    
#define GPIOG_ODR_Addr    (GPIOG_BASE+12)	//0x40011E0C    

#define GPIOA_IDR_Addr    (GPIOA_BASE+8)	//0x40010808 
#define GPIOB_IDR_Addr    (GPIOB_BASE+8)	//0x40010C08 
#define GPIOC_IDR_Addr    (GPIOC_BASE+8)	//0x40011008 
#define GPIOD_IDR_Addr    (GPIOD_BASE+8)	//0x40011408 
#define GPIOE_IDR_Addr    (GPIOE_BASE+8)	//0x40011808 
#define GPIOF_IDR_Addr    (GPIOF_BASE+8)	//0x40011A08 
#define GPIOG_IDR_Addr    (GPIOG_BASE+8)	//0x40011E08 

/**********************************************************
             实现单一IO操作，类似于51的IO操作
                   n值要小于IO具体数目
**********************************************************/ 

#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //输出 
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //输入 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //输出 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //输入 

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //输出 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //输入 

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //输出 
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //输入 

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //输出 
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //输入

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //输出 
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //输入

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //输出 
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //输入

#define  DBG_DC_H   PBout(1) = 1//GPIOB->BSRR = DBG_DC_PIN_Pin//HAL_GPIO_WritePin(DBG_DC_PIN_GPIO_Port, DBG_DC_PIN_Pin,  GPIO_PIN_RESET)
#define  DBG_DC_L   PBout(1) = 0// GPIOB->BSRR = (uint32_t)DBG_DC_PIN_Pin << 16U;//HAL_GPIO_WritePin(DBG_DC_PIN_GPIO_Port, DBG_DC_PIN_Pin,  GPIO_PIN_SET)
#define  DBG_RST_H  PBout(0) = 1//GPIOB->BSRR = DBG_RST_PIN_Pin//HAL_GPIO_WritePin(DBG_RST_PIN_GPIO_Port, DBG_RST_PIN_Pin,  GPIO_PIN_RESET)
#define  DBG_RST_L  PBout(0) = 0//GPIOB->BSRR = (uint32_t)DBG_RST_PIN_Pin << 16U;//HAL_GPIO_WritePin(DBG_RST_PIN_GPIO_Port, DBG_RST_PIN_Pin,  GPIO_PIN_SET)
#define  DBG_DD_H   PAout(4) = 1//GPIOA->BSRR = DBG_DD_PIN_Pin//HAL_GPIO_WritePin(DBG_DD_PIN_GPIO_Port, DBG_DD_PIN_Pin,  GPIO_PIN_RESET)
#define  DBG_DD_L   PAout(4) = 0//GPIOA->BSRR = DBG_DD_PIN_Pin// HAL_GPIO_WritePin(DBG_DD_PIN_GPIO_Port, DBG_DD_PIN_Pin,  GPIO_PIN_SET)

#define  DBG_DD_READ   HAL_GPIO_ReadPin(DBG_DD_PIN_GPIO_Port, DBG_DD_PIN_Pin)   //(GPIOA->IDR & DBG_DD_PIN_Pin)//     

//write config data value
typedef enum
{
 CFG_SEL_FLASH_INFO_PAGE = 0x01,
	
 CFG_TIMER_SUSPEND = 0x02,
	
 CFG_DMA_PAUSE = 0x04,
 CFG_TIMERS_OFF = 0x08 
} W_CFG;

//read debug status of mcu
typedef enum
{
 CHIP_ERASE_DONE_STATUS = 0x80,
 PCON_IDLE_STATUS       = 0x40,
 CPU_HALTED_STATUS      = 0x20,
 POWER_MODE_0_STATUS    = 0x10,
 HALT_STATUS_STATUS     = 0x08,
 DEBUG_LOCKED_STATUS    = 0x04,
 OSCILLATOR_STABLE_STATUS = 0x02,
 STACK_OVERFLOW_STATUS   = 0x01
}DBG_STATUS;


//Debug Commands
//The debug commands are shown as follow.

#define CHIP_ERASE_CMD        0x10         /* Perform flash chip erase (mass erase) and clear lock bits. If any other
																								command, except READ_STATUS, is issued, then the use of CHIP_ERASE is disabled.*/
																								
#define WR_CONFIG_CMD         0x19         /* Write configuration data. */

#define RD_CONFIG_CMD         0x24         /* Read configuration data. Returns value set by WR_CONFIG command. */

#define GET_PC_CMD            0b0010 1000         /* Return value of 16-bit program counter. Returns 2 bytes regardless of value of bit 2 in instruction code */

#define READ_STATUS_CMD       0x34                /* Read status byte. */

#define SET_HW_BRKPNT_CMD     0b0011 1011         /* Set hardware breakpoint */

#define HALT_CMD              0x44         /* Halt CPU operation */

#define RESUME_CMD            0x4C         /* Resume CPU operation. The CPU must be in halted state for this command to be run.*/

#define DEBUG_INSTR_CMD       0x55         /* Run debug instruction. The supplied instruction will be executed by theCPU without incrementing the program counter. 
                                                The CPU must be in halted state for this command to be run.*/
																								
#define STEP_INSTR_CMD        0b0101 1100         /* Step CPU instruction. The CPU will execute the next instruction from program memory and increment the program counter 
																							 after execution. The CPU must be in halted state for this command to be run. */
																							 
#define STEP_REPLACE_CMD      0b0110 01xx         /* Step and replace CPU instruction. The supplied instruction will be executed by the CPU instead of the next instruction 
                                                in program memory. The program counter will be incremented after execution. The CPU must be in halted state for this 
                                                command to be run.*/
																								
#define GET_CHIP_ID_CMD       0x68              /* Return value of 16-bit chip ID and version number.*/



#define XREG(addr)       ((unsigned char volatile *) 0)[addr]
#define PXREG(addr)      ((unsigned char volatile *) addr)
#define FCTL            XREG( 0x6270 )



// Start addresses on DUP (Increased buffer size improves performance)
#define ADDR_BUF0                   0x0000 // Buffer (2048 bytes)
#define ADDR_DMA_DESC_0             0x0800 // DMA descriptors (8 bytes)
#define ADDR_DMA_DESC_1             (ADDR_DMA_DESC_0 + 8)

// DMA channels used on DUP
#define CH_DBG_TO_BUF0              0x01   // Channel 0
#define CH_BUF0_TO_FLASH            0x02   // Channel 1
// Debug commands
#define CMD_CHIP_ERASE              0x10
#define CMD_WR_CONFIG               0x19
#define CMD_RD_CONFIG               0x24
#define CMD_READ_STATUS             0x30
#define CMD_RESUME                  0x4C
#define CMD_DEBUG_INSTR_1B          (0x54|1)
#define CMD_DEBUG_INSTR_2B          (0x54|2)
#define CMD_DEBUG_INSTR_3B          (0x54|3)
#define CMD_BURST_WRITE             0x80
#define CMD_GET_CHIP_ID             0x68

// Debug status bitmasks
#define STATUS_CHIP_ERASE_BUSY_BM   0x80 // New debug interface
#define STATUS_PCON_IDLE_BM         0x40
#define STATUS_CPU_HALTED_BM        0x20
#define STATUS_PM_ACTIVE_BM         0x10
#define STATUS_HALT_STATUS_BM       0x08
#define STATUS_DEBUG_LOCKED_BM      0x04
#define STATUS_OSC_STABLE_BM        0x02
#define STATUS_STACK_OVERFLOW_BM    0x01

// DUP registers (XDATA space address)
#define DUP_DBGDATA                 0x6260  // Debug interface data buffer
#define DUP_FCTL                    0x6270  // Flash controller
#define DUP_FADDRL                  0x6271  // Flash controller addr
#define DUP_FADDRH                  0x6272  // Flash controller addr
#define DUP_FWDATA                  0x6273  // Clash controller data buffer
#define DUP_CLKCONSTA               0x709E  // Sys clock status
#define DUP_CLKCONCMD               0x70C6  // Sys clock configuration
#define DUP_MEMCTR                  0x70C7  // Flash bank xdata mapping
#define DUP_DMA1CFGL                0x70D2  // Low byte, DMA config ch. 1
#define DUP_DMA1CFGH                0x70D3  // Hi byte , DMA config ch. 1
#define DUP_DMA0CFGL                0x70D4  // Low byte, DMA config ch. 0
#define DUP_DMA0CFGH                0x70D5  // Low byte, DMA config ch. 0
#define DUP_DMAARM                  0x70D6  // DMA arming register

unsigned char read_xdata_memory(unsigned short address);
void write_xdata_memory(unsigned short address, unsigned char value);
void chip_erase(void);
unsigned char debug_command(unsigned char cmd, unsigned char *cmd_bytes,
                            unsigned short num_cmd_bytes);
void write_xdata_memory_block(unsigned short address,
                              const unsigned char *values,
                              unsigned short num_bytes);
void read_flash_memory_block(unsigned char bank,unsigned short flash_addr,
                             unsigned short num_bytes, unsigned char *values);

void write_flash_memory_block(unsigned char *src, unsigned long start_addr,
                              unsigned short num_bytes);
#endif
/*@*****************************end of file**********************************@*/


