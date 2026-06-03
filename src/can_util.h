#if !defined(_CAN_UTIL_H_)

#define DEFAULT_NODE_ID 0x7C

uint8_t load_node_id(void);
int store_node_id(uint8_t node_id);

#define _CAN_UTIL_H_
#endif
