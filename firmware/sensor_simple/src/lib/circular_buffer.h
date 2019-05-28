/*
 * circular_buffer.h
 *
 * Created: 31.07.2017 15:36:07
 *  Author: Michael
 */ 


#ifndef CIRCULAR_BUFFER_H_
#define CIRCULAR_BUFFER_H_

#define CIRCULAR_BUFFER_MASK(size) (( size & (size-1) ) ? (1/0) : (size-1))

struct circular_buffer_header {
	volatile uint8_t head;
	volatile uint8_t tail;
};

#define CIRCULAR_BUFFER_NAMED_STRUCT(name, size)		\
struct name {									\
	struct circular_buffer_header header;	\
	uint8_t buffer[size];					\
}
#define CIRCULAR_BUFFER_STRUCT(size) CIRCULAR_BUFFER_NAMED_STRUCT(,size)

#define CIRCULAR_BUFFER_INIT {.header = {.head = 0, .tail = 0}}

#define CIRCULAR_BUFFER_CLEAR(buf) { buf.head=0; buf.tail=0; }
#define CIRCULAR_BUFFER_IS_EMPTY(buffer) (buffer.head == buffer.tail)

#define CIRCULAR_BUFFER_READ(buf, size, result) CIRCULAR_BUFFER_READ2(buf.buffer, buf.header, size, result)
#define CIRCULAR_BUFFER_READ2(buf, header, size, result) {	\
	uint8_t tmptail = (header.tail + 1) & CIRCULAR_BUFFER_MASK(size);	\
	header.tail = tmptail;								\
	result = buf[tmptail];	\
}

#define CIRCULAR_BUFFER_WRITE(buf, size, value, result) CIRCULAR_BUFFER_WRITE2(buf.buffer, buf.header, size, value, result)
#define CIRCULAR_BUFFER_WRITE2(buf, header, size, value, result) {		\
	uint8_t tmphead = ( header.head + 1 ) &  CIRCULAR_BUFFER_MASK(size);	\
	if (tmphead == header.tail)	{						\
		result = 0;										\
	} else {											\
		buf[tmphead] = value;							\
		header.head = tmphead;							\
		result = 1;										\
	}													\
}

#endif /* CIRCULAR_BUFFER_H_ */