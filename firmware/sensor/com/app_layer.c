/*
 * app_layer.c
 *
 * Created: 20.08.2017 10:29:18
 *  Author: Michael
 */ 


#include "app_layer.h" 

void app_layer_send_message(uint8_t header, const uint8_t* data_buffer, uint8_t len) {
	datalink_begin_message();
	datalink_put_message_data(&header, sizeof(header));
	datalink_put_message_data(data_buffer, len);
	datalink_end_message();
}
