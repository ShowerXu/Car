#define N	8
#define STEP_VALUE	 500
#define MID_VALUE_33 3270	//3280-10
#define MID_VALUE_45 4418	//4428-10

const unsigned int	gyro[2][3]={{MID_VALUE_33-STEP_VALUE,MID_VALUE_33,MID_VALUE_33+STEP_VALUE},{MID_VALUE_45-STEP_VALUE,MID_VALUE_45,MID_VALUE_45+STEP_VALUE}};

volatile unsigned char speed_mode;
volatile unsigned char key_power_time;
volatile unsigned char key_speed_time;
volatile unsigned char key_stop_time;
//
void key_init(void);
void key_scan(void);
//
void goto_sleep(void);
void IOCA_int(void);
//
unsigned int filter(int *arr);
void key_scan_H(void);
//
void test_init(void);