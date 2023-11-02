#include "I2C_utils.h"
#include "wm_include.h"
#include "wm_i2c.h"
#include <string.h>
#include "wm_gpio_afsel.h"


#define I2C_FREQ		(200000)

/**
 * @brief	read one byte from the specified address of the eeprom
 * @param addr the eeprom address will be read from
 * @retval     the read data
 */
u8 i2c_bus_ReadOneByte(u16 addr)
{				  
	u8 temp=0;		  	    																 
	printf("\nread addr=%x\n",ReadAddr);
	tls_i2c_write_byte(0xA0,1);   
	tls_i2c_wait_ack(); 
    	tls_i2c_write_byte(addr,0);   
	tls_i2c_wait_ack();	    

	tls_i2c_write_byte(0xA1,1);
	tls_i2c_wait_ack();	 
	temp=tls_i2c_read_byte(0,1);
	printf("\nread byte=%x\n",temp);
	return temp;
}

/**
 * @brief	read multibytes from the specified address of the eeprom
 * @param[in] addr the eeprom address will be read from
 * @param[in] buf 	Pointer to data buffer
 * @param[in] len	amount of data to be read
 * @retval    null
 */
void i2c_bus_ReadLenByte(u16 addr,u8 *buf,u16 len)
{				  
	printf("\nread len addr=%x\n",ReadAddr);
	tls_i2c_write_byte(0xA0,1);   
	tls_i2c_wait_ack(); 
    	tls_i2c_write_byte(addr,0);   
	tls_i2c_wait_ack();	    
	tls_i2c_write_byte(0xA1,1);
	tls_i2c_wait_ack();	
	while(len > 1)
	{
		*buf++ = tls_i2c_read_byte(1,0);
		printf("\nread byte=%x\n",*(pBuffer - 1));
		len --;
	}
   	*buf = tls_i2c_read_byte(0,1);
}

/**
 * @brief	write one byte to the specified address of the eeprom
 * @param addr the eeprom address will be write to
 * @retval     null
 */
void i2c_bus_WriteOneByte(u16 addr, u8 data)
{				   	  	    																 
	tls_i2c_write_byte(0XA0, 1); 
	tls_i2c_wait_ack();	   
	tls_i2c_write_byte(addr, 0);
	tls_i2c_wait_ack(); 	 										  		   
	tls_i2c_write_byte(data, 0); 				   
	tls_i2c_wait_ack();  	   
 	tls_i2c_stop();
	tls_os_time_delay(1);
}

/**
 * @brief	check the eeprom is normal or not
 * @retval 
 *     0---success
 *     1---failed
 * @note 
 *	different 24Cxx chip will use the different addr
 */
u8 i2c_bus_Check(void)
{
	u8 temp;
	temp=i2c_bus_ReadOneByte(255);
	if (temp==0x55)return 0;		   
	else
	{
		i2c_bus_WriteOneByte(255, 0x55);
		tls_os_time_delay(1);
		temp=i2c_bus_ReadOneByte(255);	  
		if (temp==0x55)return 0;
	}

	return 1;											  
}


/**
 * @brief	read multibytes from the specified address of the eeprom
 * @param[in] addr the eeprom address will be read from
 * @param[in] buf 	Pointer to data buffer
 * @param[in] len	amount of data to be read
 * @retval    null
 */
void i2c_bus_Read(u16 addr, u8 *buf, u16 len)
{
	while(len)
	{
		*buf++=i2c_bus_ReadOneByte(addr++);	
		len--;
	}
}  

/**
 * @brief	write multibytes from the specified address of the eeprom
 * @param[in] addr the eeprom address will be read from
 * @param[in] buf 	Pointer to data buffer
 * @param[in] len	amount of data to be write
 * @retval    null
 */
void i2c_bus_Write(u16 addr, u8 *buf, u16 len)
{
	while(len--)
	{
		i2c_bus_WriteOneByte(addr,*buf);
		addr++;
		buf++;
	}
} 

int i2c_sensor_demo(char *buf)
{
	u8 testbuf[] = {"i2c_bus I2C TEST OK"};
	u8 datatmp[32];

    wm_i2c_scl_config(WM_IO_PA_01);
    wm_i2c_sda_config(WM_IO_PA_04);
    
	tls_i2c_init(I2C_FREQ);

	while(i2c_bus_Check())
	{
		printf("\ni2c_bus check faild\n");
	}
	tls_os_time_delay(1);
	printf("\ni2c_bus check success\n");

	i2c_bus_Write(0,(u8 *)testbuf,sizeof(testbuf));
	tls_os_time_delay(1);
	memset(datatmp,0,sizeof(datatmp));
	//i2c_bus_Read(0,datatmp,sizeof(testbuf));
	i2c_bus_ReadLenByte(0,(u8 *)datatmp,sizeof(testbuf));
	printf("\nread data is:%s\n",datatmp);
	
	return WM_SUCCESS;
}


