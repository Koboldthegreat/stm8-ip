#include "spi.h"
#include "stm8s.h"
#include "delay.h"
#include "uart.h"
#include "se8r01.h"

#define CE_L() PC_ODR &= ~(1<<CE_PIN)
#define CE_H() PC_ODR |= (1<<CE_PIN)

#define CSN_L() PC_ODR &= ~(1<<CSN_PIN)
#define CSN_H() PC_ODR |= (1<<CSN_PIN)

int putchar(int c) {
    uart_write(c);
    return 0;
}

char getchar() {
    return uart_read();
}

uint8_t SPI_write_read_reg(uint8_t reg, uint8_t value) {
    uint8_t status;

    CSN_L(); //enable spi
    status = SPI_write_read_bit(reg); //select register
    SPI_write_read_bit(value); //write value to it
    CSN_H(); //disable spi

    return status;
}

uint8_t SPI_write_buf(uint8_t reg, uint8_t *pBuf, uint8_t bytes) {
    uint8_t status;
    
    CSN_L();
    status = SPI_write_read_bit(reg);
    for (uint8_t i=0; i<bytes; i++) {
        SPI_write_read_bit(pBuf[i]);
    }
    CSN_H();
    return status;
}

uint8_t SPI_read_buf(uint8_t reg, uint8_t *pBuf, uint8_t bytes) {
    uint8_t status;
    CSN_L();
    status = SPI_write_read_bit(reg); 
    for(uint8_t i=0; i<bytes; i++) {
        pBuf[i] = SPI_write_read_bit(0xff);
    }
    CSN_H();
    return status;
}


void se8r01_init(uint8_t mode) {

    printf("Initializing SPI\n");
    //SPI_init();
    delay_ms(100);
    init_bit(); //bit bashing SPI
    init_io();
    printf("Status: %02x \n", se8r01_get_status());

    CE_L();
    delay_ms(10);
    se8r01_powerup();
    se8r01_calibration();
    se8r01_setup();
    se8r01_settings();
    if (mode == 'r') { //rx mode
        printf("RX mode\n"); 
        SPI_write_read_reg(WRITE_REG | iRF_BANK0_CONFIG, 0x3f);
    } else { //tx mode
        printf("TX mode, status %02x\n",
        SPI_write_read_reg(WRITE_REG | iRF_BANK0_CONFIG, 0x3e)
        );
        printf("connected? %02x\n", SPI_write_read_reg(SETUP_AW, 0xff));
    }
    CE_H();
} 

uint8_t se8r01_wait_rx(uint8_t *rx_buf) {
    uint8_t status; 
    //while (PD_IDR & (1<<IRQ_PIN)){delay_ms(1);} //wait till irq is low
    while (PB_IDR & (1<<IRQ_PIN));
    delay_ms(1);
    status = se8r01_get_status();  
    if (status & STA_MARK_RX) {
        SPI_read_buf(RD_RX_PLOAD, rx_buf, TX_PLOAD_WIDTH);
        se8r01_flush_RX();
        SPI_write_read_reg(WRITE_REG|STATUS, 0xff);//  ,0xff);
        return 1;
    } else {
        SPI_write_read_reg(WRITE_REG|STATUS, 0xff);
        return 0;
    }
    /*} else {
        return 0;
    }*/
}

void se8r01_tx(uint8_t *tx_buf) {
    uint8_t status;
    
    status = se8r01_get_status();
    if (status & (1<<5)) {
        PD_ODR |= (1<<4);
    } else {
        PD_ODR &= ~(1<<4);
    } 
    se8r01_flush_TX();
    SPI_write_buf(WR_TX_PLOAD, tx_buf, TX_PLOAD_WIDTH);
    SPI_write_read_reg(WRITE_REG+STATUS, 0xff);
    printf("Status: %02x \n", status);
}

uint8_t se8r01_get_status() {
    return SPI_write_read_reg(STATUS, 0);
}

void init_io() {
  //IRQ pin 
  PB_DDR &= ~(1<<IRQ_PIN); //Input
  PB_CR1 &= ~(1<<IRQ_PIN); //No pull-up
  PB_CR2 &= ~(1<<IRQ_PIN); //disable externernal interupt
   
  //CE pin
  PC_DDR |= (1<<CE_PIN); //output
  PC_CR1 |= (1<<CE_PIN); //push-pull
  PC_CR2 |= (1<<CE_PIN); //10Mhz speed
  
  //CSN pin
  PC_DDR |= (1<<CSN_PIN); //output
  PC_CR1 |= (1<<CSN_PIN); //push-pull
  PC_CR2 |= (1<<CSN_PIN); //10Mhz speed

  //status pin
  PD_DDR |= (1<<4);
  PD_CR1 |= (1<<4);

  CSN_H(); //disable spi
  CE_L(); //power down at startup
}

void clear_IRQ_flags() {
    uint8_t status;
    
    status = se8r01_get_status();
    SPI_write_read_reg(STATUS, status|0x70);
    printf("clearing irq flag\n");
}

void rf_switch_bank(uint8_t bankindex)
{
    uint8_t temp0;

    temp0 = SPI_write_read_bit(iRF_BANK0_STATUS);
    printf("Temp 0: %02x \n", temp0);
    if((temp0 & 0x80) != bankindex)
    {
        SPI_write_read_reg(iRF_CMD_ACTIVATE,0x53);
    }
}

void se8r01_powerup()
{
        rf_switch_bank(iBANK0);
        SPI_write_read_reg(iRF_CMD_WRITE_REG|iRF_BANK0_CONFIG,0x03);
        SPI_write_read_reg(iRF_CMD_WRITE_REG|iRF_BANK0_RF_CH,0x32);
        SPI_write_read_reg(iRF_CMD_WRITE_REG|iRF_BANK0_RF_SETUP,0x48);
        SPI_write_read_reg(iRF_CMD_WRITE_REG|iRF_BANK0_PRE_GURD,0x77); //2450 calibration
}


void se8r01_calibration()

{
        uint8_t gtemp[5];

        rf_switch_bank(iBANK1);

        gtemp[0]=0x40;
        gtemp[1]=0x00;
        gtemp[2]=0x10;
        gtemp[3]=0xE6;
        SPI_write_buf(iRF_CMD_WRITE_REG|iRF_BANK1_PLL_CTL0, gtemp, 4);

        gtemp[0]=0x20;
        gtemp[1]=0x08;
        gtemp[2]=0x50;
        gtemp[3]=0x40;
        gtemp[4]=0x50;
        SPI_write_buf(iRF_CMD_WRITE_REG|iRF_BANK1_CAL_CTL, gtemp, 5);

        gtemp[0]=0x00;
        gtemp[1]=0x00;
        gtemp[2]=0x1E;
        SPI_write_buf(iRF_CMD_WRITE_REG|iRF_BANK1_IF_FREQ, gtemp, 3);

        gtemp[0]=0x29;
        SPI_write_buf(iRF_CMD_WRITE_REG|iRF_BANK1_FDEV, gtemp, 1);

        gtemp[0]=0x00;
        SPI_write_buf(iRF_CMD_WRITE_REG|iRF_BANK1_DAC_CAL_LOW, gtemp, 1);

        gtemp[0]=0x7F;
        SPI_write_buf(iRF_CMD_WRITE_REG|iRF_BANK1_DAC_CAL_HI, gtemp, 1);

        gtemp[0]=0x02;
        gtemp[1]=0xC1;
        gtemp[2]=0xEB;
        gtemp[3]=0x1C;
        SPI_write_buf(iRF_CMD_WRITE_REG|iRF_BANK1_AGC_GAIN, gtemp, 4);

        gtemp[0]=0x97;
        gtemp[1]=0x64;
        gtemp[2]=0x00;
        gtemp[3]=0x81;
        SPI_write_buf(iRF_CMD_WRITE_REG|iRF_BANK1_RF_IVGEN, gtemp, 4);

        rf_switch_bank(iBANK0);

        CE_H();
        delay_us(30);
        CE_L();

        delay_us(50);                            // delay 50ms waiting for calibaration.

        CE_H();
        delay_us(30);
        CE_L();

        delay_us(50);                            // delay 50ms waiting for calibaration.
        // calibration end

   }
  
  void se8r01_setup()
  
  {
        uint8_t gtemp[5];
        gtemp[0]=0x28;
        gtemp[1]=0x32;
        gtemp[2]=0x80;
        gtemp[3]=0x90;
        gtemp[4]=0x00;
        SPI_write_buf(iRF_CMD_WRITE_REG|iRF_BANK0_SETUP_VALUE, gtemp, 5);

        delay_us(2);

        rf_switch_bank(iBANK1);

        gtemp[0]=0x40;
        gtemp[1]=0x01;
        gtemp[2]=0x30;
        gtemp[3]=0xE2;
        SPI_write_buf(iRF_CMD_WRITE_REG|iRF_BANK1_PLL_CTL0, gtemp, 4);

        gtemp[0]=0x29;
        gtemp[1]=0x89;
        gtemp[2]=0x55;
        gtemp[3]=0x40;
        gtemp[4]=0x50;
        SPI_write_buf(iRF_CMD_WRITE_REG|iRF_BANK1_CAL_CTL, gtemp, 5);

        gtemp[0]=0x29;
        SPI_write_buf(iRF_CMD_WRITE_REG|iRF_BANK1_FDEV, gtemp, 1);

        gtemp[0]=0x55;
        gtemp[1]=0xC2;
        gtemp[2]=0x09;
        gtemp[3]=0xAC;
        SPI_write_buf(iRF_CMD_WRITE_REG|iRF_BANK1_RX_CTRL, gtemp, 4);

        gtemp[0]=0x00;
        gtemp[1]=0x14;
        gtemp[2]=0x08;
        gtemp[3]=0x29;
        SPI_write_buf(iRF_CMD_WRITE_REG|iRF_BANK1_FAGC_CTRL_1, gtemp, 4);

        gtemp[0]=0x02;
        gtemp[1]=0xC1;
        gtemp[2]=0xCB;
        gtemp[3]=0x1C;
        SPI_write_buf(iRF_CMD_WRITE_REG|iRF_BANK1_AGC_GAIN, gtemp, 4);

        gtemp[0]=0x97;
        gtemp[1]=0x64;
        gtemp[2]=0x00;
        gtemp[3]=0x01;
        SPI_write_buf(iRF_CMD_WRITE_REG|iRF_BANK1_RF_IVGEN, gtemp, 4);

        gtemp[0]=0x2A;
        gtemp[1]=0x04;
        gtemp[2]=0x00;
        gtemp[3]=0x7D;
        SPI_write_buf(iRF_CMD_WRITE_REG|iRF_BANK1_TEST_PKDET, gtemp, 4);

        rf_switch_bank(iBANK0);

}

void se8r01_settings() {
        uint8_t TX_ADDRESS[TX_ADR_WIDTH]  = 
        {
          0x34,0x43,0x10,0x10
        }; // Define a static TX address
        
        SPI_write_read_reg(WRITE_REG|iRF_BANK0_EN_AA, 0x01);          //enable auto acc on pip 1
        SPI_write_read_reg(WRITE_REG|iRF_BANK0_EN_RXADDR, 0x01);      //enable pip 1
        SPI_write_read_reg(WRITE_REG|iRF_BANK0_SETUP_AW, 0x02);        //4 byte adress
        
        SPI_write_read_reg(WRITE_REG|iRF_BANK0_SETUP_RETR, 0x0a);        //lowest 4 bits 0-15 rt transmisston higest 4 bits 256-4096us Auto Retransmit Delay
        SPI_write_read_reg(WRITE_REG|iRF_BANK0_RF_CH, 40);
        SPI_write_read_reg(WRITE_REG|iRF_BANK0_RF_SETUP, 0x5f);        //500kps 0x4f
        //SPI_write_read_reg(WRITE_REG|iRF_BANK0_DYNPD, 0x01);          //pipe0 pipe1 enable dynamic payload length data
        //SPI_write_read_reg(WRITE_REG|iRF_BANK0_FEATURE, 0x07);        // enable dynamic paload lenght; enbale payload with ack enable w_tx_payload_noack
        
        SPI_write_buf(WRITE_REG + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);  //from tx
        SPI_write_buf(WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH); // Use the same address on the RX device as the TX device
        SPI_write_read_reg(WRITE_REG + RX_PW_P0, TX_PLOAD_WIDTH); // Select same RX payload width as TX Payload width
        
}

void se8r01_flush_TX() {
    SPI_write_read_reg(FLUSH_TX, 0xff);
}

void se8r01_flush_RX() {
    SPI_write_read_reg(FLUSH_RX, 0xff);
}
