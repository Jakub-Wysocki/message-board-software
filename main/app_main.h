#ifndef APP_MAIN_H
#define APP_MAIN_H

typedef struct ms_board_configuration{
    char data[128];
    int data_len;
    short font;
    int display_time;
} ms_board_configuration;

void e_paper_task(void *pvParameter);

#define URI "mqtt://10.77.21.158:1883"


#endif