#include "post_api.h"
#include "local_files.h"
#include "jsmn.h"
#include "MyFlash.h"
#include "lwip.h"
#include <sys/socket.h>
#include <arpa/inet.h>
#define USER_PASS_BUFSIZE 512
#define mymin(a,b) \
		({ __typeof__ (a) _a = (a); \
		__typeof__ (b) _b = (b); \
		_a < _b ? _a : _b; })
static void *current_connection;
static void *valid_connection;
int post_offset=0;
//static char last_user[USER_PASS_BUFSIZE];

char buf_data[USER_PASS_BUFSIZE];

static int jsoneq(const char *json, jsmntok_t *tok, const char *s) {
	if (tok->type == JSMN_STRING && (int)strlen(s) == tok->end - tok->start &&
			strncmp(json + tok->start, s, tok->end - tok->start) == 0) {
		return 0;
	}
	return -1;
}
err_t
httpd_post_begin(void *connection, const char *uri, const char *http_request, u16_t http_request_len, int content_len, char *response_uri, u16_t response_uri_len, u8_t *post_auto_wnd)
{
	post_offset=0;

	memset(buf_data,0,sizeof(buf_data));
	LWIP_UNUSED_ARG(connection);
	LWIP_UNUSED_ARG(http_request);
	LWIP_UNUSED_ARG(http_request_len);
	LWIP_UNUSED_ARG(content_len);
	LWIP_UNUSED_ARG(post_auto_wnd);
	if (!memcmp(uri, "/info.html", 10)) {
		if (current_connection != connection) {
			current_connection = connection;
			valid_connection = NULL;
			/* default page is "login failed" */
			snprintf(response_uri, response_uri_len, "/info.html");
			/* e.g. for large uploads to slow flash over a fast connection, you should
         manually update the rx window. That way, a sender can only send a full
         tcp window at a time. If this is required, set 'post_aut_wnd' to 0.
         We do not need to throttle upload speed here, so: */
			*post_auto_wnd = 1;
			return ERR_OK;
		}
	}
	return ERR_VAL;
}

err_t
httpd_post_receive_data(void *connection, struct pbuf *p)
{
	err_t ret;

	if ((post_offset+p->len) <= USER_PASS_BUFSIZE){
		strncpy(buf_data+post_offset, p->payload,p->len);
	}
	else{
		ret = ERR_VAL;
		pbuf_free(p);
		return ret;
	}
	post_offset += p->len;

	int data_len = strlen(buf_data);
	if (buf_data[data_len-1]!=']'){
		ret = ERR_OK;
		pbuf_free(p);
		return ret;
	}

	LWIP_ASSERT("NULL pbuf", p != NULL);

	if (current_connection == connection) {
		jsmn_parser parser;
		jsmntok_t t[512]; /* We expect no more than 512 JSON tokens */
		jsmn_init(&parser);

		int tokens = jsmn_parse(&parser, buf_data, strlen(buf_data), t, 128);
		for (int i = 0; i<tokens; i++){
			if (jsoneq(buf_data, &t[i], "IPaddress") == 0) {
				buf_data[t[i+1].end] = '\0';
				memset(user_info.ip,0,16);
				int len = mymin(16,t[i+1].end-t[i+1].start);
				strncpy(user_info.ip,&buf_data[t[i+1].start],len);
				i++;
				continue;
			}
			if (jsoneq(buf_data, &t[i], "NetMask") == 0) {
				buf_data[t[i+1].end] = '\0';
				memset(user_info.netmask,0,16);
				int len = mymin(16,t[i+1].end-t[i+1].start);
				strncpy(user_info.netmask,&buf_data[t[i+1].start],len);
				i++;
				continue;
			}
			if (jsoneq(buf_data, &t[i], "Timezone") == 0) {
				user_info.zone = atoi(&buf_data[t[i+1].start]);
				i++;
				continue;
			}
			if (jsoneq(buf_data, &t[i], "contacts") == 0) {
				memset(user_info.contacts,0,INFOLEN);
				buf_data[t[i+1].end] = '\0';
				int len = mymin(INFOLEN,t[i+1].end-t[i+1].start);
				strncpy(user_info.contacts,&buf_data[t[i+1].start],len);
				i++;
				continue;
			}
		}
		//setIPaddr
		ip4_addr_t add;
		inet_aton(user_info.ip, &add);
		setIP(add.addr);
		//setNetMask
		ip4_addr_t mask;
		inet_aton(user_info.netmask, &mask);
		setNetmask(mask.addr);

        clearFlash();
        int offset=0;
        WriteDeviceAddressOffset((char*) &user_info, sizeof(user_info), offset);
        offset+=sizeof(user_info);
		/* not returning ERR_OK aborts the connection, so return ERR_OK unless the
       connection is unknown */
		ret = ERR_OK;
	} else {
		ret = ERR_VAL;
	}

	/* this function must ALWAYS free the pbuf it is passed or it will leak memory */
	pbuf_free(p);

	return ret;
}

void
httpd_post_finished(void *connection, char *response_uri, u16_t response_uri_len)
{
	/* default page is "login failed" */
	snprintf(response_uri, response_uri_len, "/404.html");
	if (current_connection == connection) {
		if (valid_connection == connection) {
			/* login succeeded */
			snprintf(response_uri, response_uri_len, "/info.html");
		}
		current_connection = NULL;
		valid_connection = NULL;
	}
}

