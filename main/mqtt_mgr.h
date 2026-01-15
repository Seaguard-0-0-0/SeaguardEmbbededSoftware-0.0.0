#ifndef MQTT_MGR_H
#define MQTT_MGR_H

#include <stdbool.h>

typedef void (*mqtt_control_cb_t)(const char *topic, const char *json_payload);

void mqtt_init_start(const char *broker_uri, const char *boat_id);
void mqtt_set_control_callback(mqtt_control_cb_t cb);
bool mqtt_publish(const char *topic, const char *payload);
bool mqtt_is_connected(void);

#endif // MQTT_MGR_H
