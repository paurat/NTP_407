#ifndef __MYAPI_H
#define __MYAPI_H


#include "lwip/opt.h"

#include "lwip/apps/httpd.h"
#include "lwip/def.h"
#include "lwip/mem.h"
#include <stdio.h>
#include <string.h>
#include "lwip.h"
#include "fs.h"

extern RTC_HandleTypeDef hrtc;
extern struct Time_rx gps;
extern int PPS_count;

int fs_read_custom(struct fs_file *file, char *buffer, int count);
int fs_open_custom(struct fs_file *file, const char *name);
void fs_close_custom(struct fs_file *file);


#endif //__MYAPI_H
