#ifndef CAN_TESTS_H_
#define CAN_TESTS_H_

int8_t 	TEST_sendBit(uint8_t bit);
uint8_t TEST_recvBit();
void 	CAN_runTests(uint8_t rxEn, uint8_t txEn);

#endif /* CAN_TESTS_H_ */
