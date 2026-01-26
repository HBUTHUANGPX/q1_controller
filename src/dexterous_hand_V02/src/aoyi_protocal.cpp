#include "dexterous_hand/aoyi_hand.hpp"
// #include "dexterous_hand/aoyi_protocal.hpp"

uint8_t HAND_ProtocolLRC(uint8_t* lrcBytes, uint8_t lrcByteCount)
{
  uint8_t lrc = 0;
  uint8_t i;

  for (i = 0; i < lrcByteCount; i++)
  {
    lrc ^= lrcBytes[i];
  }

  return lrc;
}

/*
 * I2C protocol omits the header and peer ID:
 *   <ownID> <commandID> <data byte count> <data1> <data2> ... <dataN> <lrc>
 */

uint8_t AoyiHand::HAND_SendCmd(uint8_t addr, uint8_t cmd, uint8_t *data, uint8_t nb_data)
{
    uint8_t i;
    uint8_t lrc;
    uint8_t send_buf[MAX_PROTOCOL_DATA_SIZE + 7];

    if (nb_data >= MAX_PROTOCOL_DATA_SIZE)
        return HAND_RESP_DATA_SIZE_TOO_BIG;

    // uint8_t lrc = HAND_ProtocolLRC(data, nb_data);
    // lrc ^= cmd;
    // lrc ^= nb_data;

    send_buf[0] = 0x55;
    send_buf[1] = 0xAA;
    send_buf[2] = addr;
    send_buf[3] = get_master_id(); // master address
    send_buf[4] = cmd;
    send_buf[5] = nb_data;

    for (i = 0; i < nb_data; i++)
    {
        send_buf[6 + i] = data[i];
    }

    lrc = HAND_ProtocolLRC(send_buf + 2, 4 + nb_data);
    send_buf[nb_data + 6] = lrc;

    can_ptr_->send_multi_frame(addr, send_buf, nb_data + 7);
    return HAND_RESP_SUCCESS;
}

// validate command response with time out (in ms)
// return value:
// 0x00: command success, resp_bytes stores response data
// other: error number
uint8_t AoyiHand::HAND_GetResponse(void *ctx, uint8_t addr, uint8_t cmd, uint16_t time_out, void *resp_bytes, uint8_t *byte_count,
                                   uint8_t *remote_err)
{
    if (ctx == NULL)
        return HAND_RESP_INVALID_CONTEXT;

    AoyiHand *_ctx = static_cast<AoyiHand *>(ctx);

    // check if there is response data in buffer with time out

    //   uint32_t wait_start = _get_milli_seconds_impl();
    //   uint32_t wait_timeout = wait_start + time_out;

    //   ohand_context_t* _ctx = ctx;

    //   while (_ctx->is_whole_packet == 0)
    //   {
    //     _delay_milli_seconds_impl(1);

    //     if (_ctx->recv_data_impl != NULL)
    //       _ctx->recv_data_impl(ctx);

    //     if (wait_timeout < _get_milli_seconds_impl())
    //     {
    //       _ctx->decode_state = _ctx->initial_state;

    //       return HAND_RESP_TIMEOUT;
    //     }
    //   }

    // if ( _ctx->is_whole_packet == 0)
    // return HAND_RESP_DATA_INVALID;

    if (_ctx->is_whole_packet == 0)
        return 0;

    // validate lrc
    uint8_t lrc = HAND_ProtocolLRC(_ctx->packet_data,
                                   _ctx->packet_data[DATA_CNT_BYTE_NUM] + 4); // lrc for node_id, own_id, command_id, byte_cnt, data[]

    if (lrc != _ctx->packet_data[DATA_START_BYTE_NUM + _ctx->packet_data[DATA_CNT_BYTE_NUM]])
    {
        _ctx->is_whole_packet = 0;
        return ERR_PROTOCOL_WRONG_LRC;
    }

    // check if response is error
    if ((_ctx->packet_data[CMD_ID_BYTE_NUM] & CMD_ERROR_MASK) != 0)
    {
        if (remote_err != NULL)
        {
            // uint8_t node_addr = _ctx->packet_data[OWN_ID_BYTE_NUM];
            // uint8_t nb_err = _ctx->packet_data[DATA_CNT_BYTE_NUM]; // count

            /*
            uint8_t i;

            for (i=0; i<MIN(nb_err, sizeof(_ctx->packet_data) - 5); i++)
            {
              remote_err(node_addr, _ctx->packet_data[i + DATA_START_BYTE_NUM], _ctx);
            }
            */
            *remote_err = _ctx->packet_data[DATA_START_BYTE_NUM];
        }

        return HAND_RESP_HAND_ERROR;
    }

    if (_ctx->packet_data[OWN_ID_BYTE_NUM] != addr && addr != 0xFF /*broadcast*/)
    {
        _ctx->is_whole_packet = 0;
        return HAND_RESP_UNMATCHED_ADDR;
    }

    if (_ctx->packet_data[CMD_ID_BYTE_NUM] != cmd)
    {
        _ctx->is_whole_packet = 0;
        return HAND_RESP_UNMATCHED_CMD;
    }

    uint8_t ret = HAND_RESP_SUCCESS;

    // copy response data
    if (resp_bytes != NULL)
    {
        uint8_t packet_byte_count = _ctx->packet_data[DATA_CNT_BYTE_NUM];

        if (packet_byte_count > *byte_count)
        {
            ret = HAND_RESP_INVALID_OUT_BUFFER_SIZE;
        }
        else
        {
            void *p = &(_ctx->packet_data[DATA_START_BYTE_NUM]);
            memcpy(resp_bytes, p, packet_byte_count);
        }

        *byte_count = packet_byte_count;
    }

    //   _ctx->is_whole_packet = 0;

    return ret;
}

void AoyiHand::HAND_OnData(void *ctx, uint8_t data)
{
    if (ctx == NULL)
        return;

    AoyiHand *_ctx = static_cast<AoyiHand *>(ctx);

    if (_ctx->is_whole_packet == 1)
        return; /* Old packet is not processed, ignore */

    // uint32_t curr_time = _get_milli_seconds_impl();
    // uint32_t time_elapsed = (uint32_t)(curr_time - _ctx->_last_time);

    // if (time_elapsed > 50)
    // _ctx->_decode_state = _ctx->_initial_state; /* Timeout, reset decode status */

    // _ctx->_last_time = curr_time;

    switch (_ctx->decode_state)
    {
        case WAIT_ON_HEADER_0:
            if (data == 0x55)
                _ctx->decode_state = WAIT_ON_HEADER_1;

            break;

        case WAIT_ON_HEADER_1:
            _ctx->decode_state = (data == 0xAA) ? WAIT_ON_ADDRESSED_NODE_ID : WAIT_ON_HEADER_0;
            break;

        case WAIT_ON_ADDRESSED_NODE_ID:
#if 0
            // if (data == _ctx->_address_master)
            //{
            _ctx->decode_state = WAIT_ON_OWN_NODE_ID;
            //_ctx->_is_whole_packet = 0;
            _ctx->packet_data[NODE_ID_BYTE_NUM] = data;
            //}
            // else
            //{
            // _ctx->_decode_state = _ctx->_initial_state;
            //}
#else
            if (data == _ctx->address_master)
            {
                _ctx->decode_state = WAIT_ON_OWN_NODE_ID;
                //_ctx->_is_whole_packet = 0;
                _ctx->packet_data[NODE_ID_BYTE_NUM] = data;
            }
            else
            {
                _ctx->decode_state = _ctx->initial_state;
            }
#endif
            break;

        case WAIT_ON_OWN_NODE_ID:
            _ctx->packet_data[OWN_ID_BYTE_NUM] = data;
            _ctx->decode_state = WAIT_ON_COMMAND_ID;

            break;

        case WAIT_ON_COMMAND_ID:
            _ctx->packet_data[CMD_ID_BYTE_NUM] = data;
            _ctx->decode_state = WAIT_ON_BYTECOUNT;

            break;

        case WAIT_ON_BYTECOUNT:
            _ctx->packet_data[DATA_CNT_BYTE_NUM] = data;
            _ctx->byte_count = data;

            if (_ctx->byte_count > MAX_PROTOCOL_DATA_SIZE)
                _ctx->decode_state = _ctx->initial_state;
            else if (_ctx->byte_count > 0)
                _ctx->decode_state = WAIT_ON_DATA;
            else
                _ctx->decode_state = WAIT_ON_LRC;

            break;

        case WAIT_ON_DATA:
            _ctx->packet_data[DATA_START_BYTE_NUM + _ctx->packet_data[DATA_CNT_BYTE_NUM] - _ctx->byte_count] = data;

            if (--_ctx->byte_count == 0)
                _ctx->decode_state = WAIT_ON_LRC;

            break;

        case WAIT_ON_LRC:
            _ctx->packet_data[DATA_START_BYTE_NUM + _ctx->packet_data[DATA_CNT_BYTE_NUM]] = data;

            if (_ctx->packet_data[NODE_ID_BYTE_NUM] == _ctx->address_master)
                _ctx->is_whole_packet = 1;

            /*
            printf("Recv from device: 0x55 0xaa ");
            int i;
            for (i=0; i<=DATA_START_BYTE_NUM + _ctx->_packet_data[DATA_CNT_BYTE_NUM]; i++)
              printf("0x%02x ", _ctx->_packet_data[i]);
            printf("\n");
            */

            _ctx->decode_state = _ctx->initial_state;

            break;

        default:
            _ctx->decode_state = _ctx->initial_state;
    }
}

uint8_t AoyiHand::HAND_SetFingerPos(void *ctx, uint8_t hand_id, uint8_t finger_id, uint16_t pos, uint8_t speed, uint8_t *remote_err)
{
    if (ctx == NULL)
        return HAND_RESP_INVALID_CONTEXT;

    // AoyiHand *_ctx = static_cast<AoyiHand *>(ctx);

    uint8_t err;

    uint8_t data[sizeof(finger_id) + sizeof(pos) + sizeof(speed)];
    const uint8_t nb_data = sizeof(data);

    data[0] = (uint8_t)finger_id;
    data[1] = (uint8_t)pos;
    data[2] = (uint8_t)(pos >> 8);
    data[3] = (uint8_t)speed;

    err = HAND_SendCmd(hand_id, HAND_CMD_SET_FINGER_POS, data, nb_data);

    //   if (err == HAND_RESP_SUCCESS)
    //     err = HAND_GetResponse(ctx, hand_id, HAND_CMD_SET_FINGER_POS, ((ohand_context_t*)ctx)->timeout, NULL, NULL, remote_err);

    return err;
}


uint8_t AoyiHand::HAND_SetCustom(uint8_t hand_id, uint8_t *data, uint8_t send_data_size, uint8_t *recv_data_size, uint8_t *remote_err)
{
    uint8_t err;

    err = HAND_SendCmd(hand_id, HAND_CMD_SET_CUSTOM, data, send_data_size);

    //   if (err == HAND_RESP_SUCCESS)
    //     err = HAND_GetResponse(this, hand_id, HAND_CMD_SET_CUSTOM, 0, data, recv_data_size, remote_err);

    return err;
}
