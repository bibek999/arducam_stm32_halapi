/**
  * @file    ArduCAM.c
  * @author  Arducam 
  * @version V0.1
  * @date    2018.06.18
  * @brief   Arducam mainly driver
  */
#include "main.h"
#include "ArduCAM.h"
#include "ov5642_regs.h"

extern SPI_HandleTypeDef hspi2;

extern UART_HandleTypeDef huart1;

extern I2C_HandleTypeDef hi2c2;

//byte sensor_model = 0;
byte sensor_addr = 0;
byte m_fmt = JPEG;
uint32_t length = 0;
uint8_t is_header= false ;

void ArduCAM_Init(void)
{
	  wrSensorReg16_8(0x3008, 0x80);
	  WRSensorRegs16_8(OV5642_QVGA_Preview);
	  HAL_Delay(100);
	  WRSensorRegs16_8(OV5642_JPEG_Capture_QSXGA);
	  WRSensorRegs16_8(ov5642_320x240);
	  wrSensorReg16_8(0x3818, 0xa8);
	  wrSensorReg16_8(0x3621, 0x10);
	  wrSensorReg16_8(0x3801, 0xb0);
	  wrSensorReg16_8(0x4407, 0x04);
}


//Control the CS pin
void CS_HIGH(void)
{
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
}

void CS_LOW(void)
{
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
}

void set_format(byte fmt)
{
  if (fmt == BMP)
    m_fmt = BMP;
  else
    m_fmt = JPEG;
}

uint8_t read_reg(uint8_t address)
{
	 uint8_t data = 0x00;
     CS_LOW();
     HAL_SPI_Transmit(&hspi2, &address, sizeof(address), 500);
	 HAL_SPI_Transmit(&hspi2, &data, sizeof(data), 500);
	 HAL_SPI_Receive(&hspi2, &data, sizeof(data), 500);
	 CS_HIGH();
	 return data;
}

void write_reg(uint8_t address, uint8_t value)
{
	uint8_t add = 0x80 | address;
	CS_LOW();
	HAL_SPI_Transmit(&hspi2, &add, sizeof(add), 100);
	HAL_SPI_Transmit(&hspi2, &value, sizeof(value), 100);
	CS_HIGH();
}


uint8_t read_fifo(void)
{
	uint8_t data;
	data = read_reg(SINGLE_FIFO_READ);
	return data;
}
//void set_fifo_burst()
//{
//	SPI1_ReadWriteByte(BURST_FIFO_READ);
//}


void flush_fifo(void)
{
	write_reg(ARDUCHIP_FIFO, FIFO_CLEAR_MASK);
}

void start_capture(void)
{
	write_reg(ARDUCHIP_FIFO, FIFO_START_MASK);
}

void clear_fifo_flag(void )
{
	write_reg(ARDUCHIP_FIFO, FIFO_CLEAR_MASK);
}

uint32_t read_fifo_length(void)
{
	uint32_t len1,len2,len3,len=0;
	len1 = read_reg(FIFO_SIZE1);
	len2 = read_reg(FIFO_SIZE2);
	len3 = read_reg(FIFO_SIZE3) & 0x7f;
	len = ((len3 << 16) | (len2 << 8) | len1) & 0x07fffff;
	return len;	
}

//Set corresponding bit  
void set_bit(uint8_t addr, uint8_t bit)
{
	uint8_t tem;
	tem = read_reg(addr);
	write_reg(addr, tem | bit);
}
//Clear corresponding bit 
void clear_bit(uint8_t addr, uint8_t bit)
{
	uint8_t tem;
	tem = read_reg(addr);
	write_reg(addr, tem & (~bit));
}

//Get corresponding bit status
uint8_t get_bit(uint8_t addr, uint8_t bit)
{
  uint8_t tem;
  tem = read_reg(addr);
  tem = tem && bit;
  return tem;
}

//Set ArduCAM working mode
//MCU2LCD_MODE: MCU writes the LCD screen GRAM
//CAM2LCD_MODE: Camera takes control of the LCD screen
//LCD2MCU_MODE: MCU read the LCD screen GRAM
void set_mode(uint8_t mode)
{
  switch (mode)
  {
    case MCU2LCD_MODE:
      write_reg(ARDUCHIP_MODE, MCU2LCD_MODE);
      break;
    case CAM2LCD_MODE:
      write_reg(ARDUCHIP_MODE, CAM2LCD_MODE);
      break;
    case LCD2MCU_MODE:
      write_reg(ARDUCHIP_MODE, LCD2MCU_MODE);
      break;
    default:
      write_reg(ARDUCHIP_MODE, MCU2LCD_MODE);
      break;
  }
}

void OV5642_set_JPEG_size(uint8_t size)
{
  switch (size)
  {
    case O5642_320x240:
      WRSensorRegs16_8(ov5642_320x240);
      break;
    case O5642_640x480:
      WRSensorRegs16_8(ov5642_640x480);
      break;
    case O5642_1024x768:
      WRSensorRegs16_8(ov5642_1024x768);
      break;
    case O5642_1280x960:
      WRSensorRegs16_8(ov5642_1280x960);
      break;
    case O5642_1600x1200:
      WRSensorRegs16_8(ov5642_1600x1200);
      break;
    case O5642_2048x1536:
      WRSensorRegs16_8(ov5642_2048x1536);
      break;
    case O5642_2592x1944:
      WRSensorRegs16_8(ov5642_2592x1944);
      break;
    default:
      WRSensorRegs16_8(ov5642_320x240);
      break;
  }
}


//I2C Array Write 8bit address, 8bit data
int wrSensorRegs8_8(const struct sensor_reg reglist[])
{
  int err = 0;
  uint16_t reg_addr = 0;
  uint16_t reg_val = 0;
  const struct sensor_reg *next = reglist;
  while ((reg_addr != 0xff) | (reg_val != 0xff))
  {
    reg_addr = next->reg;
    reg_val = next->val;
    err = wrSensorReg8_8(reg_addr, reg_val);
 //   delay_us(400);
    next++;
  }

  return err;
}

uint8_t wrSensorReg16_8(uint16_t regID, uint8_t regDat)
{
	HAL_I2C_Mem_Write(&hi2c2, OV5642_ADD_WRITE, regID, sizeof(regID), &regDat, sizeof(regDat), 100);
	return regDat;
}

//int wrSensorRegs16_8(const struct sensor_reg reglist[])
//{
//  int err = 0;
//
//  uint16_t reg_addr;
//  uint8_t reg_val;
//  const struct sensor_reg *next = reglist;
//
//  while ((reg_addr != 0xffff) || (reg_val != 0xff))
//  {
//    reg_addr =next->reg;
//    reg_val = next->val;
//    err = wrSensorReg16_8(reg_addr, reg_val);
//    HAL_Delay(600);
//    next++;
//  }
//  return err;
//}

uint8_t WRSensorRegs16_8(const struct sensor_reg reglist[])
{
	uint8_t err=0;
	  uint16_t reg_addr;
	  uint8_t reg_val;
	  const struct sensor_reg *next = reglist;

	  while ((reg_addr != 0xffff) || (reg_val != 0xff))
	  {
	    reg_addr =next->reg;
	    reg_val = next->val;
	    err = wrSensorReg16_8(reg_addr, reg_val);
	    next++;
	  }
	  return err;
}


uint8_t rdSensorReg16_8(uint16_t regID, uint8_t *regDat)
{
	HAL_I2C_Master_Transmit(&hi2c2, OV5642_ADD_WRITE, regID, 2, 100);
	HAL_I2C_Mem_Read(&hi2c2, OV5642_ADD_READ, regID, sizeof(regID), regDat, 1, 100);
	return *regDat;
}

