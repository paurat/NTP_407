#ifndef SRC_POST_API_H_
#define SRC_POST_API_H_



#include "lwip/opt.h"

#include "lwip/apps/httpd.h"
#include "lwip/def.h"
#include "lwip/mem.h"
#include <stdio.h>
#include <string.h>

err_t httpd_post_begin(void *connection, const char *uri, const char *http_request, u16_t http_request_len, int content_len, char *response_uri, u16_t response_uri_len, u8_t *post_auto_wnd);

err_t httpd_post_receive_data(void *connection, struct pbuf *p);

void httpd_post_finished(void *connection, char *response_uri, u16_t response_uri_len);


#endif /* SRC_POST_API_H_ */
