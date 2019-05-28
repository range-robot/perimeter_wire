
#ifndef PERIMETER_WIRE_SENSOR_MESSAGES_H_
#define PERIMETER_WIRE_SENSOR_MESSAGES_H_

// client => host
#define MESSAGE_HELLO 'H'
// hello: header(8) - version(16)
#define MESSAGE_VERSION 'V'
// version: header(8) - version(16)
#define MESSAGE_ERROR 'E'
// error: header(8) - error(8)
#define MESSAGE_REG_VALUE 'R'
// reg_value: header(8) - address(8) - value(8)
#define MESSAGE_REG_VALUE_16 'S'
// reg_value: header(8) - address(8) - value(16, HL)
#define MESSAGE_COMMAND_RESULT 'C'
// result: header(8) - cmd(8) - result(8)

// host => client
#define MESSAGE_COMMAND 'c'
// cmd: header(8) - cmd(8) 
#define MESSAGE_GET_VERSION 'v'
// stop: header(8)
#define MESSAGE_RESET 'r'
// reset: header(8)
#define MESSAGE_SET_REG 's'
// set_data: header(8) - address(8) - value(8)
#define MESSAGE_SET_REG_16 't'
// set_data: header(8) - address(8) - value(16, H-L)
#define MESSAGE_GET_REG 'g'
// get_data: header(8) - address(8)
#define MESSAGE_GET_REG_16 'h'
// get_data: header(8) - address(8)


#endif /* PERIMETER_WIRE_SENSOR_MESSAGES_H_ */