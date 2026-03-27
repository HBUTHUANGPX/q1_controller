#pragma once

/* for class AoyiHand */
uint8_t HAND_SetFingerPos(void* ctx, uint8_t hand_id, uint8_t finger_id, uint16_t pos, uint8_t speed, uint8_t* remote_err);
uint8_t HAND_SetCustom(uint8_t hand_id, uint8_t *data, uint8_t send_data_size, uint8_t *recv_data_size, uint8_t *remote_err);
uint8_t HAND_SendCmd(uint8_t addr, uint8_t cmd, uint8_t *data, uint8_t nb_data);
uint8_t HAND_GetResponse(void* ctx, uint8_t addr, uint8_t cmd, uint16_t time_out, void* resp_bytes, uint8_t* byte_count, uint8_t* remote_err);
void HAND_OnData(void* ctx, uint8_t data);