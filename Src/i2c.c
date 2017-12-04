/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "i2c.h"
#include "gpio.h"
#include "bsp.h"
#include <string.h>
#include "motor_control.h"

#define BUFFER_DEPTH 256
typedef struct
{
  uint8_t state;
  uint8_t address;
  uint8_t rx_index;
  uint8_t rx_buffer[BUFFER_DEPTH];
  uint8_t tx_index;
  uint8_t tx_buffer[BUFFER_DEPTH];
  uint8_t tx_count;
}i2c_struct_t;

i2c_struct_t i2c_struct;
void I2C_IRQ_Handler(unsigned char flags, unsigned short rx_data);

//uint8_t *bemf_debug_buffer;
enum I2C_FLAG_ENUM {
  I2C_TX_READY = 0,
  I2C_RX_AVAILABLE,
  I2C_ADDR_MATCH,
  I2C_NACK_DETECT,
  I2C_ACK_DETECT,
  I2C_STOP_DETECT,
  I2C_TIMEOUT_DETECT,
  I2C_ERROR_DETECT
};

enum I2C_STATE_ENUM {
  I2C_IDLE = 0,
  I2C_START,
  I2C_READ_ADDR,
  I2C_READ_FROM_MASTER,
  I2C_WRITE_TO_MASTER,
  I2C_WAIT_FOR_RESPONSE,
  I2C_STOP
};

enum I2C_COMM_REG_ADDR {
  I2C_REG_STATUS            = 1,
  I2C_REG_ERROR             = 2,
  I2C_REG_RPM               = 3,
  I2C_REG_CURRENT           = 4,
  I2C_REG_VBUS              = 5,
  I2C_REG_TEMP              = 6,
  I2C_REG_TARGET_OL_ERPM    = 7,
  I2C_REG_BEMF_SAMPLE_COUNT = 8,
  I2C_REG_BEMF_BUFFER       = 9,
  I2C_REG_BEMF_BUFFER_DEPTH = 10
};

int i2c_debug = 0;
extern uint32_t uwTick;
uint32_t temp = 0;

void I2C1_EV_IRQHandler(void){
  //uint32_t flag;
  //i2c_debug ++;

/*
**  I2C_FLAG_TXE
**  I2C_FLAG_TXIS
**  I2C_FLAG_RXNE
**  I2C_FLAG_ADDR
**  I2C_FLAG_NACKF
**  I2C_FLAG_STOPF
XX  I2C_FLAG_TC (MASTER MODE ONLY)
XX  I2C_FLAG_TCR (MASTER MODE ONLY ??)
**  I2C_FLAG_BERR
**  I2C_FLAG_ARLO
**  I2C_FLAG_OVR
**  I2C_FLAG_PECERR
**  I2C_FLAG_TIMEOUT
**  I2C_FLAG_ALERT
XX  I2C_FLAG_BUSY
*/

  //Detected an Address Match
  //if (I2C_GetITStatus(I2C1, I2C_IT_ADDR) == SET){
  if (LL_I2C_IsActiveFlag_ADDR(I2C1)){
    //flag = LL_I2C_GetTransferDirection(I2C1);
    //temp = LL_I2C_GetTransferDirection(I2C1);
    I2C_IRQ_Handler(I2C_ADDR_MATCH, (LL_I2C_GetTransferDirection(I2C1) > 0));
    LL_I2C_ClearFlag_ADDR(I2C1);
    //XXX: Execute a local function for I2C Specific UI Behavior (Put the data in the receive buffer)
    //i2c_struct.state = I2C_START;
  }




  //Receive Data Register Not Empty
  //if (I2C_GetITStatus(I2C1, I2C_IT_RXNE) == SET){
  if (LL_I2C_IsActiveFlag_RXNE(I2C1)){
    I2C_IRQ_Handler(I2C_RX_AVAILABLE, LL_I2C_ReceiveData8(I2C1));
    //The only way to clear it is to read from the read register
    //LL_I2C_ClearFlag_(I2C1);
    //XXX: Execute a local function for I2C Specific UI Behavior (Put the data in the receive buffer)
  }

  //Detect a stop condition
  //if (I2C_GetITStatus(I2C1, I2C_IT_STOPF) == SET){
  if (LL_I2C_IsActiveFlag_STOP(I2C1)){
    I2C_IRQ_Handler(I2C_STOP_DETECT, 0);
    LL_I2C_ClearFlag_STOP(I2C1);
    //XXX: Execute a local function for I2C Specific UI Behavior Reset the local state machine
    //i2c_struct.state = I2C_IDLE;
  }

  //Detect a nack condition
  //if (I2C_GetITStatus(I2C1, I2C_IT_NACKF) == SET){
  if (LL_I2C_IsActiveFlag_NACK(I2C1)){
    I2C_IRQ_Handler(I2C_NACK_DETECT, 0);
    LL_I2C_ClearFlag_NACK(I2C1);
    //XXX: Execute a local function for I2C Specific UI Behavior Reset the local state machine
    //i2c_struct.state = I2C_IDLE;
  }

  //Transmit Data Register Empty
  //if (I2C_GetITStatus(I2C1, I2C_FLAG_TXE) == SET){
/*
  if (LL_I2C_IsActiveFlag_TXE(I2C1)){
    i2c_debug++;
    I2C_IRQ_Handler(I2C_TX_READY, 0);
    //LL_I2C_ClearFlag_(I2C1);
    LL_I2C_ClearFlag_TXE(I2C1);
    //XXX: Execute a local function for I2C Specific UI Behavior (Send another byte?)
  }
*/

  //Transmit Data Register Empty
  //if (I2C_GetITStatus(I2C1, I2C_IT_TXIS) == SET){
  if (LL_I2C_IsActiveFlag_TXIS(I2C1)){
    //i2c_debug++;
    I2C_IRQ_Handler(I2C_TX_READY, 0);
    //The only way to clear is to write to the transmit register
    //LL_I2C_ClearFlag_(I2C1);
    //LL_I2C_ClearFlag_TXE(I2C1);
    //XXX: Execute a local function for I2C Specific UI Behavior (Send another byte?)
  }

  //Error Conditions
  //Detect a bus error condition
  //if (I2C_GetITStatus(I2C1, I2C_IT_BERR) == SET){
  if (LL_I2C_IsActiveFlag_BERR(I2C1)){
    I2C_IRQ_Handler(I2C_ERROR_DETECT, 0);
    LL_I2C_ClearFlag_BERR(I2C1);
    //XXX: Execute a local function for I2C Specific UI Behavior Reset the local state machine
    //i2c_struct.state = I2C_IDLE;
  }

  //Detect a bus error condition
  //if (I2C_GetITStatus(I2C1, I2C_IT_ARLO) == SET){
  if (LL_I2C_IsActiveFlag_ARLO(I2C1)){
    I2C_IRQ_Handler(I2C_ERROR_DETECT, 0);
    LL_I2C_ClearFlag_ARLO(I2C1);
    //XXX: Execute a local function for I2C Specific UI Behavior Reset the local state machine
    //i2c_struct.state = I2C_IDLE;
  }

  //Detect a bus error condition
  //if (I2C_GetITStatus(I2C1, I2C_IT_OVR) == SET){
  if (LL_I2C_IsActiveFlag_OVR(I2C1)){
    I2C_IRQ_Handler(I2C_ERROR_DETECT, 0);
    LL_I2C_ClearFlag_OVR(I2C1);
    //XXX: Execute a local function for I2C Specific UI Behavior Reset the local state machine
    //i2c_struct.state = I2C_IDLE;
  }

  //Detect a timeout condition
  //if (I2C_GetITStatus(I2C1, I2C_IT_TIMEOUT) == SET){
  if (LL_I2C_IsActiveSMBusFlag_TIMEOUT(I2C1)){
    I2C_IRQ_Handler(I2C_ERROR_DETECT, 0);
    LL_I2C_ClearSMBusFlag_TIMEOUT(I2C1);
    //XXX: Execute a local function for I2C Specific UI Behavior Reset the local state machine
    //i2c_struct.state = I2C_IDLE;
  }

  //Detect a PEC Error condition
  //if (I2C_GetITStatus(I2C1, I2C_IT_PECERR) == SET){
  if (LL_I2C_IsActiveSMBusFlag_PECERR(I2C1)){
    I2C_IRQ_Handler(I2C_ERROR_DETECT, 0);
    LL_I2C_ClearSMBusFlag_PECERR(I2C1);
    //XXX: Execute a local function for I2C Specific UI Behavior Reset the local state machine
    //i2c_struct.state = I2C_IDLE;
  }
}

void I2C1_ER_IRQHandler(void){
    //i2c_debug ++;
}

void MX_I2C1_Init(void)
{
  LL_I2C_InitTypeDef I2C_InitStruct;

  LL_GPIO_InitTypeDef GPIO_InitStruct;

  /**I2C1 GPIO Configuration
  PB6   ------> I2C1_SCL
  PB7   ------> I2C1_SDA
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6|LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  //GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  //GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

  /* I2C1 interrupt Init */
  NVIC_SetPriority(I2C1_EV_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),3, 0));
  NVIC_EnableIRQ(I2C1_EV_IRQn);
  NVIC_SetPriority(I2C1_ER_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(I2C1_ER_IRQn);

    /**I2C Initialization
    */
  I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
  //I2C_InitStruct.Timing = 0x2000090E;
  I2C_InitStruct.Timing = 0x0000020B;
  I2C_InitStruct.AnalogFilter = LL_I2C_ANALOGFILTER_ENABLE;
  I2C_InitStruct.DigitalFilter = 0;
  I2C_InitStruct.OwnAddress1 = 82;
  I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
  I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
  LL_I2C_Init(I2C1, &I2C_InitStruct);

  LL_I2C_EnableAutoEndMode(I2C1);

  LL_I2C_SetOwnAddress2(I2C1, 0, LL_I2C_OWNADDRESS2_NOMASK);

  LL_I2C_DisableOwnAddress2(I2C1);

  LL_I2C_DisableGeneralCall(I2C1);
  //LL_I2C_EnableGeneralCall(I2C1);

  LL_I2C_EnableClockStretching(I2C1);
  LL_I2C_EnableSlaveByteControl(I2C1);

  i2c_struct.state = I2C_IDLE;
  i2c_struct.address = 0;
  i2c_struct.rx_index = 0;
  i2c_struct.tx_index = 0;
  i2c_struct.tx_count = 0;


  LL_I2C_EnableIT_ERR(I2C1);
  LL_I2C_EnableIT_STOP(I2C1);
  LL_I2C_EnableIT_NACK(I2C1);
  LL_I2C_EnableIT_ADDR(I2C1);
  LL_I2C_EnableIT_TX(I2C1);
  LL_I2C_EnableIT_RX(I2C1);
  //LL_I2C_EnableIT_TC(I2C1);

  LL_I2C_EnableOwnAddress1(I2C1);
  LL_I2C_Enable(I2C1);


  //bemf_debug_buffer = mc_get_bemf_buffer_handle();
}

/**
  * @}
  */

void I2C_IRQ_Handler(unsigned char flags, unsigned short rx_data){
  int32_t data;
	//int16_t d16_data;
  //This just updates the state
  switch (flags) {
    case (I2C_ADDR_MATCH):
      if (rx_data == 0) {
        i2c_struct.rx_index = 0;
        i2c_struct.state = I2C_READ_ADDR;
      }
      else {
        i2c_debug = 0;
        i2c_struct.tx_index = 0;
        switch(i2c_struct.address){
          case (0):
            //i2c_struct.tx_buffer[i2c_struct.tx_index++] = (uint8_t) MCI_GetSTMState(cmci);
            //i2c_struct.tx_buffer[i2c_struct.tx_index++] = (uint8_t) 0;
            i2c_struct.tx_buffer[i2c_struct.tx_index++] = (uint8_t) (uwTick >> 24) & 0xFF;
            i2c_struct.tx_buffer[i2c_struct.tx_index++] = (uint8_t) (uwTick >> 16) & 0xFF;
            i2c_struct.tx_buffer[i2c_struct.tx_index++] = (uint8_t) (uwTick >>  8) & 0xFF;
            i2c_struct.tx_buffer[i2c_struct.tx_index++] = (uint8_t) (uwTick >>  0) & 0xFF;
            break;
          case (I2C_REG_STATUS):
            data = mc_get_state();
            i2c_struct.tx_buffer[i2c_struct.tx_index++] = (uint8_t) (data & 0xFF);
            break;
          case (I2C_REG_ERROR):
            data = mc_get_error();
            i2c_struct.tx_buffer[i2c_struct.tx_index++] = (uint8_t) (data & 0xFF);
            break;
          case (I2C_REG_RPM):
						data = mc_get_rpm();
            i2c_struct.tx_buffer[i2c_struct.tx_index++] = (uint8_t) (data >> 24) & 0xFF;
            i2c_struct.tx_buffer[i2c_struct.tx_index++] = (uint8_t) (data >> 16) & 0xFF;
            i2c_struct.tx_buffer[i2c_struct.tx_index++] = (uint8_t) (data >>  8) & 0xFF;
            i2c_struct.tx_buffer[i2c_struct.tx_index++] = (uint8_t) (data >>  0) & 0xFF;
            break;
          case (I2C_REG_CURRENT):
            data = mc_get_current();
            i2c_struct.tx_buffer[i2c_struct.tx_index++] = (uint8_t) (data >> 24) & 0xFF;
            i2c_struct.tx_buffer[i2c_struct.tx_index++] = (uint8_t) (data >> 16) & 0xFF;
            i2c_struct.tx_buffer[i2c_struct.tx_index++] = (uint8_t) (data >>  8) & 0xFF;
            i2c_struct.tx_buffer[i2c_struct.tx_index++] = (uint8_t) (data >>  0) & 0xFF;
            break;
          case (I2C_REG_VBUS):
            data = mc_get_bus_voltage();
            i2c_struct.tx_buffer[i2c_struct.tx_index++] = (uint8_t) (data >> 24) & 0xFF;
            i2c_struct.tx_buffer[i2c_struct.tx_index++] = (uint8_t) (data >> 16) & 0xFF;
            i2c_struct.tx_buffer[i2c_struct.tx_index++] = (uint8_t) (data >>  8) & 0xFF;
            i2c_struct.tx_buffer[i2c_struct.tx_index++] = (uint8_t) (data >>  0) & 0xFF;
            break;
          case (I2C_REG_TEMP):
            data = mc_get_temperature();
            i2c_struct.tx_buffer[i2c_struct.tx_index++] = (uint8_t) (data >> 24) & 0xFF;
            i2c_struct.tx_buffer[i2c_struct.tx_index++] = (uint8_t) (data >> 16) & 0xFF;
            i2c_struct.tx_buffer[i2c_struct.tx_index++] = (uint8_t) (data >>  8) & 0xFF;
            i2c_struct.tx_buffer[i2c_struct.tx_index++] = (uint8_t) (data >>  0) & 0xFF;
            break;
          case (I2C_REG_TARGET_OL_ERPM):
            data = mc_get_ol_target_rpm();
            i2c_struct.tx_buffer[i2c_struct.tx_index++] = (uint8_t) (data >> 8) & 0xFF;
            i2c_struct.tx_buffer[i2c_struct.tx_index++] = (uint8_t) (data >> 0) & 0xFF;
            break;

          default:
            //No Command To Execute
            i2c_struct.tx_buffer[i2c_struct.tx_index++] = 0xCA;
          break;
        }
        i2c_struct.tx_count = i2c_struct.tx_index;

        LL_I2C_SetTransferSize(I2C1, i2c_struct.tx_count);
        i2c_struct.tx_index = 0;
        LL_I2C_TransmitData8(I2C1, i2c_struct.tx_buffer[i2c_struct.tx_index]);
        i2c_struct.tx_index++;
        i2c_struct.state = I2C_WRITE_TO_MASTER;

        //LL_I2C_EnableIT_TX(I2C1);
      }
      break;
//    case (I2C_NACK_DETECT):
//      //XXX: What is a master nack do again??
//      i2c_struct.state = I2C_WRITE_TO_MASTER;
//      break;

//      i2c_debug = 1;
//      break;
//    case (I2C_ACK_DETECT):
//      //XXX: What is a master nack do again??
//      break;
    case (I2C_TIMEOUT_DETECT):
      i2c_struct.state = I2C_IDLE;
      //i2c_debug = -1;
      break;
   case (I2C_ERROR_DETECT):
      i2c_struct.state = I2C_IDLE;
      //i2c_debug = -1;
      break;
    default:
      break;
  }

  switch (i2c_struct.state){
    case (I2C_IDLE):
      LL_I2C_SetTransferSize(I2C1, 0);
      //LL_I2C_DisableIT_TX(I2C1);
      break;
    case (I2C_READ_ADDR):
      //i2c_debug = 4;
      if (flags == I2C_RX_AVAILABLE) {
        i2c_struct.address = rx_data;
        i2c_struct.state = I2C_READ_FROM_MASTER;
      }
      break;
    case (I2C_READ_FROM_MASTER):
      if (flags == I2C_RX_AVAILABLE) {
        i2c_struct.rx_buffer[i2c_struct.rx_index] = rx_data;
        i2c_struct.rx_index++;
      }

			if (flags == I2C_STOP_DETECT){
				switch(i2c_struct.address){
					case (0):
						if (i2c_struct.rx_buffer[0] == 1) {
							mc_set_speed(2000);
							mc_start_motor();
						}
						else {
							mc_stop_motor();
						}
						break;
					case (I2C_REG_STATUS):
						break;
					case (I2C_REG_RPM):
						break;
					default:
						//No Command To Execute
						break;
				}
			}




      break;
    case (I2C_WRITE_TO_MASTER):
      if (flags == I2C_TX_READY){
        //i2c_debug++;
        LL_I2C_TransmitData8(I2C1, i2c_struct.tx_buffer[i2c_struct.tx_index]);
        i2c_struct.tx_index++;
      }
      break;
    case (I2C_WAIT_FOR_RESPONSE):
      break;
    case (I2C_STOP):
      break;
    default:
      i2c_struct.state = I2C_IDLE;
      break;
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
