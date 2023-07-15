/*
References:
https://store.tmotor.com/images/file/20220216/1644991915235842.pdf
https://docs.espressif.com/projects/esp-idf/en/release-v3.3/api-reference/peripherals/can.html

servo mode has 6 control modes:

duty cycle
current loop
current brake
velocity
position
position velocity

*/

void comm_can_transmit_eid(uint32_t id, const uint8_t *data, uint8_t len)
{
  uint8_t i=0;
  if (len > 8)
  {
    len = 8;
  }
  CanTxMsg TxMessage;
  TxMessage.StdId = 0;
  TxMessage.ExtId = id;
  TxMessage.RTR = CAN_RTR_DATA;
  TxMessage.DLC = len;
  //memcpy(txmsg.data8, data, len);
  for (i = 0; i<len; i++){}
  TxMessage.Data[i] = data[i];
  CAN_Transmit(CHASSIS_CAN, &TxMessage);
}

void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index)
{
  buffer[(*index)++] = number >> 24;
}

//current brake mode:
void comm_can_set_cb(uint8_t controller_id, float current)
{
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
  comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE << 8), buffer, send_index);

}