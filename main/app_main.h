#ifndef APP_MAIN_H
#define APP_MAIN_H

typedef struct displaying_data{
    char data[128];
    int data_len;
    short font;
    int display_time;
} displaying_data;

void e_paper_task(void *pvParameter);

#define URI "mqtt://10.77.22.144:1883"


#endif