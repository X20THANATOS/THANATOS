
#include "soft_i2c.h"
//#include "i2c.h"
#include "global.h"



#define I2C_ACK		0		/* SDA level to ack a byte */
#define I2C_NOACK	1		/* SDA level to noack a byte */

#define I2C_DIR_WRITE	0
#define I2C_DIR_READ	1

//clk
void I2C_DELAY(void)
{
  u8 i;
  
  for(i=0; i<20; i++);      //84k
  return;
}


void sys_i2c_init_board(void)
{
    //初始化IO口,SCL配置为输出高
    GPIO_InitTypeDef PB6={GPIO_Pin_6,GPIO_Speed_2MHz,GPIO_Mode_Out_OD};
    GPIO_Init(GPIOB, &PB6);
    //初始化IO口，SDA配置为输出高
    GPIO_InitTypeDef PB7={GPIO_Pin_7,GPIO_Speed_2MHz,GPIO_Mode_Out_OD};
    GPIO_Init(GPIOB,&PB7);
}

//START: High -> Low on SDA while SCL is High
void soft_i2c_send_start(void)
{
    SET_SDA_OUT;
    I2C_DELAY();
    
    I2C_SDA(1);
    I2C_SCL(1);
    I2C_DELAY();
    
    I2C_SDA(0);
    I2C_DELAY();
    
    I2C_SCL(0);
    I2C_DELAY();
}

//STOP: Low -> High on SDA while SCL is High
void soft_i2c_send_stop(void)
{
    I2C_SCL(0);
    SET_SDA_OUT;
    I2C_SDA(0);
    I2C_DELAY();
    
  
    I2C_SCL(1);
    I2C_DELAY();

    I2C_SDA(1);
    I2C_DELAY();
}

void soft_i2c_init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    sys_i2c_init_board();

    soft_i2c_send_stop();
}




 //ack should be I2C_ACK or I2C_NOACK
void soft_i2c_send_ack(int ack)
{
    SET_SDA_OUT;
    I2C_SDA(ack);
    I2C_DELAY();
    
    I2C_SCL(1);
    I2C_DELAY();
    I2C_DELAY();
    I2C_SCL(0);
    I2C_DELAY();
   
}


/*-----------------------------------------------------------------------
 * Send 8 bits and look for an acknowledgement.
 */
static int soft_i2c_write_byte(u8 data)
{
  int j;
  int nack;
  
  SET_SDA_OUT;
  for(j = 0; j < 8; j++) 
  {
    I2C_SDA(data & 0x80);
    I2C_DELAY();
    I2C_SCL(1);
    I2C_DELAY();
    I2C_DELAY();
    I2C_SCL(0);
    I2C_DELAY();
    
    data <<= 1;
  }

  /*
   * Look for an <ACK>(negative logic) and return it.
   */
  
  SET_SDA_IN;
  I2C_DELAY();
  I2C_SCL(1);
  I2C_DELAY();
  nack = I2C_SDA_VALUE_IN;
  I2C_DELAY();
  I2C_SCL(0);
  I2C_DELAY();
  
  SET_SDA_OUT;
  //I2C_SDA(1);
  return(nack);	/* not a nack is an ack */
}

/*-----------------------------------------------------------------------
 * if ack == I2C_ACK, ACK the byte so can continue reading, else
 * send I2C_NOACK to end the read.
 */
static u8 soft_i2c_read_byte(int ack)
{
  u8  data;
  int  j;
  
  SET_SDA_IN;
  
  data = 0;
  for(j = 0; j < 8; j++) 
  {
    I2C_DELAY();
    I2C_SCL(1);
    I2C_DELAY();
    data <<= 1;
    data |= I2C_SDA_VALUE_IN;
    I2C_DELAY();
    I2C_SCL(0);
    I2C_DELAY();
  }
  soft_i2c_send_ack(ack);
  
  return(data);
}

/*-----------------------------------------------------------------------
 * Probe to see if a chip is present.  Also good for checking for the
 * completion of EEPROM writes since the chip stops responding until
 * the write completes (typically 10mSec).
 */
int soft_i2c_probe(u8 chipaddr)
{
  int rc;
  
  /*
   * perform 1 byte write transaction with just address byte
   * (fake write)
   */
  soft_i2c_send_start();
  rc = soft_i2c_write_byte((chipaddr << 1) | 0);
  soft_i2c_send_stop();
  
  return (rc ? 1 : 0);
}

/*-----------------------------------------------------------------------
 * Read bytes
 */
int soft_i2c_read(u8 chipaddr, u32 internalAddr, u32 internalAddrLen, u8 *buffer, int len)
{
  int shift;

  /*
   * Do the addressing portion of a write cycle to set the
   * chip's address pointer.  If the address length is zero,
   * don't do the normal write cycle to set the address pointer,
   * there is no address pointer in this chip.
   */
  soft_i2c_send_start();
  if(internalAddrLen > 0) 
  {
    if(soft_i2c_write_byte(chipaddr << 1)) 	/* write cycle */
    {
      soft_i2c_send_stop();
      return(1);
    }
    shift = (internalAddrLen-1) * 8;
    while(internalAddrLen-- > 0) 
    {
      if(soft_i2c_write_byte(internalAddr >> shift)) 
      {
        //printf("i2c_read, internal address not <ACK>ed\n");
        return(1);
      }
      shift -= 8;
    }

  /* Some I2C chips need a stop/start sequence here,
   * other chips don't work with a full stop and need
   * only a start.  Default behaviour is to send the
   * stop/start sequence.
   */
#ifdef CONFIG_SOFT_I2C_READ_REPEATED_START
    soft_i2c_send_start();
#else
    soft_i2c_send_stop();
    soft_i2c_send_start();
#endif
  }
  /*
   * Send the chip address again, this time for a read cycle.
   * Then read the data.  On the last byte, we do a NACK instead
   * of an ACK(len == 0) to terminate the read.
   */
  soft_i2c_write_byte((chipaddr << 1) | 1);	/* read cycle */
  while(len-- > 0) 
  {
      *buffer++ = soft_i2c_read_byte(len == 0);
  }
  soft_i2c_send_stop();
  return(0);
}


/*-----------------------------------------------------------------------
 * Write bytes
 */
int soft_i2c_write(u8 chipaddr, u32 internalAddr, u32 internalAddrLen, u8 *buffer, int len)
{
    int shift, failures = 0;

    //printf("i2c_write: chip %02X addr %02X alen %d buffer %p len %d\n",chip, addr, alen, buffer, len);

    soft_i2c_send_start();
    if(soft_i2c_write_byte(chipaddr<<1)) 	/* write cycle */
    {	
        soft_i2c_send_stop();
        //printf("i2c_write, no chip responded %02X\n", chipaddr);
        return(1);
    }
    shift = (internalAddrLen-1) * 8;
    while(internalAddrLen-- > 0) 
    {
        if(soft_i2c_write_byte(internalAddr >> shift)) 
        {
            //printf("i2c_write, address not <ACK>ed\n");
            return(1);
        }
        shift -= 8;
    }

    while(len-- > 0) 
    {
        if(soft_i2c_write_byte(*buffer++)) 
        {
            failures++;
        }
    }
    soft_i2c_send_stop();
    return(failures);
}



