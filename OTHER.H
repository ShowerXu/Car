#define N	10
#define MID_VALUE 2857
#define MAX_VALUE (MID_VALUE+500)
#define MIN_VALUE (MID_VALUE-500)

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