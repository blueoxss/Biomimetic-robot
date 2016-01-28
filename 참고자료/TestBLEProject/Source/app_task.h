#ifndef APP_TEST_TASK_H_

#define TASK_START				0x0001

extern void TestBLE_TaskInit( uint8 task_id );
extern uint16 TestBLE_OSALEvents( uint8 task_id, uint16 events );

#endif