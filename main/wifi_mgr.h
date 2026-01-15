#ifndef WIFI_MGR_H
#define WIFI_MGR_H

#include <stdbool.h>
#include <stddef.h>

void wifi_init_sta(void);

bool wifi_is_connected(void);
int wifi_get_rssi(void);
int wifi_get_channel(void);
void wifi_get_ip(char *out, size_t len);
void wifi_get_ssid(char *out, size_t len);

#endif // WIFI_MGR_H
