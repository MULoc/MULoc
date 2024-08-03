#include "dwm1000_timestamp.h"

/* Declaration of static functions. */
uint64 get_tx_timestamp_u64(void);
uint64 get_rx_timestamp_u64(void);
uint64 get_cur_timestamp_u64(void);

void final_msg_get_ts(const uint8 *ts_field, uint32 *ts);
void final_msg_set_ts(uint8 *ts_field, uint64 ts);

void final2_msg_set_fp(uint8 *fp_field, uint16 fp_index);
void final2_msg_get_fp(const uint8 *fp_field, uint16 *fp_index);
	
/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_tx_timestamp_u64()
 *
 * @brief Get the TX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
uint64 get_tx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readtxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

uint64 get_cur_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readsystime(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_rx_timestamp_u64()
 *
 * @brief Get the RX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
uint64 get_rx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readrxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn final_msg_get_ts()
 *
 * @brief Read a given timestamp value from the final message. In the timestamp fields of the final message, the least
 *        significant byte is at the lower address.
 *
 * @param  ts_field  pointer on the first byte of the timestamp field to read
 *         ts  timestamp value
 *
 * @return none
 */
void final_msg_get_ts(const uint8 *ts_field, uint32 *ts)
{
    int i;
    *ts = 0;
    for (i = 0; i < 4; i++)
    {
        *ts += ts_field[i] << (i * 8);
    }
}


void final_msg_set_ts(uint8 *ts_field, uint64 ts)
{
    int i;
    for (i = 0; i < 5; i++)
    {
        ts_field[i] = (uint8) ts;
        ts >>= 8;
    }
}

void final2_msg_set_fp(uint8 *fp_field, uint16 fp_index)
{
    uint8 i;
    for (i = 0; i < 2; i++)
    {
        fp_field[i] = (uint8_t)fp_index;
        fp_index >>= 8;
    }
}

void final2_msg_get_fp(const uint8 *fp_field, uint16 *fp_index)
{
    uint8 i;
    *fp_index = 0;
    for (i = 0; i < 2; i++)
    {
        *fp_index += ((uint16_t)fp_field[i] << (i * 8));
    }
}

