/*
 * myapi.c
 *
 *  Created on: Mar 4, 2022
 *      Author: User
 */
#include "local_files.h"
#include "myapi.h"
#include "lwip.h"
#define JSON_SIZE 1608
struct tm WebPageTime;
int fs_read_custom(struct fs_file *file, char *buffer, int count){
	return 0;
}
int fs_open_custom(struct fs_file *file, const char *name){
	char generated_html[JSON_SIZE];
	memset(generated_html, 0, JSON_SIZE);
	u16_t offset = 0;

	if (!strcmp(name, "/info.json")) {

		char* ip=user_info.ip;
		int zone=user_info.zone;
		char* contacts=user_info.contacts;
		char* netmask=user_info.netmask;
		char macstr[18]; //string to be displayed on web page
		sprintf (macstr, "%02X:%02X:%02X:%02X:%02X:%02X", getMAC(0), getMAC(1), getMAC(2), getMAC(3), getMAC(4), getMAC(5) );

			offset = sprintf(generated_html,"[{\"IPaddress\":\"%s\",\"NetMask\":\"%s\",\"Timezone\":\"%d\",\"contacts\":\"%s\",\"software_version\":\"0.00001\",\"mac\":\"%s\"}]",ip,netmask,zone,contacts,macstr);
	}
	else if (!strcmp(name, "/uptime.json")) {
		RTC_DateTypeDef dateStruct;
		RTC_TimeTypeDef timeStruct;

		//hrtc.Instance = RTC;
		if (gps.year[0]!='V'&&PPS_count>12){
			// Read actual date and time
			HAL_RTC_GetTime(&hrtc, &timeStruct, FORMAT_BIN); // Read time first!
			HAL_RTC_GetDate(&hrtc, &dateStruct, FORMAT_BIN);

			struct tm timeinfo;

			timeinfo.tm_wday = dateStruct.WeekDay;
			timeinfo.tm_mon = dateStruct.Month;//-1 do January==0 month
			timeinfo.tm_mday = dateStruct.Date;
			timeinfo.tm_year = dateStruct.Year;
			timeinfo.tm_hour = timeStruct.Hours;
			timeinfo.tm_min = timeStruct.Minutes;
			timeinfo.tm_sec = timeStruct.Seconds;

			time_t t = mktime(&timeinfo)+offset_unix[user_info.zone];
			// time_t  to   tm

			localtime_r(  &t, &WebPageTime );

			int Hours=WebPageTime.tm_hour;
			int Minutes=WebPageTime.tm_min;
			int Seconds=WebPageTime.tm_sec;
			int Date=WebPageTime.tm_mday;
			int Month=WebPageTime.tm_mon+1;
			int Year=WebPageTime.tm_year+2000;


			offset = sprintf(generated_html,"%02d:%02d:%02d %02d.%02d.%04d",Hours,Minutes,Seconds,Date,Month,Year);
		}
		else if (gps.year[0]=='V'||PPS_count<13) {
			offset = sprintf(generated_html,"no reference time");
		}
//		else if (gps.year[0]=='') {
//					offset = sprintf(generated_html,"no Reference Timestamp");
//				}
	}

	if (offset>0){
		memset(file, 0, sizeof(struct fs_file));
		file->pextension = mem_malloc(offset);
	}

	if (file->pextension != NULL) {
		/* instead of doing memcpy, you would generate e.g. a JSON here */
		memcpy(file->pextension, generated_html, offset);
		file->data = (const char *)file->pextension;
		file->len = offset; /* don't send the trailing 0 */
		file->index = file->len;
		/* allow persisteng connections */
		file->flags = FS_FILE_FLAGS_HEADER_INCLUDED;
		return 1;
	}

	printf("Extension fail %s\n\r",name);
	return 0;
}
void fs_close_custom(struct fs_file *file){
	if (file && file->pextension) {
		mem_free(file->pextension);
		file->pextension = NULL;
	}
}

