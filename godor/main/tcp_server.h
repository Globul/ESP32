/* OpenSSL server Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#ifndef _TCP_SERVER_H_
#define _TCP_SERVER_H_

#include "sdkconfig.h"

/* The examples use simple WiFi configuration that you can set via
   'make menuconfig'.

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
#define EXAMPLE_WIFI_SSID               "Livebox-C266"
#define EXAMPLE_WIFI_PASS               "Nessie3008"

#define TCP_TASK_NAME        "tcp_godor"
#define TCP_TASK_STACK_WORDS 10240
#define TCP_TASK_PRIORITY    8

#define TCP_EXAMPLE_RECV_BUF_LEN       1024

#define TCP_EXAMPLE_LOCAL_TCP_PORT     443

#endif

