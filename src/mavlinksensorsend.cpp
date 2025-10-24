//#include "mavlink.h"  // or mavlink/common/mavlink.h depending on setup
/*
extern HardwareSerial PixhawkSerial;  // The UART connected to Pixhawk

void mavlink_send_tof_distance(uint8_t sensor_id, uint16_t distance_mm, uint8_t signal_quality) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_distance_sensor_t sensor_msg;
    sensor_msg.time_boot_ms = millis(); // or TOF_0.system_time if synced
    sensor_msg.min_distance = 30;       // minimum measurable distance [mm]
    sensor_msg.max_distance = 1200;     // maximum measurable distance [mm]
    sensor_msg.current_distance = distance_mm;
    sensor_msg.type = MAV_DISTANCE_SENSOR_LASER; // laser TOF
    sensor_msg.id = sensor_id;
    sensor_msg.orientation = MAV_SENSOR_ROTATION_PITCH_270; // adjust orientation if needed
    sensor_msg.covariance = 0;          // if unknown
    sensor_msg.signal_quality = signal_quality; // optional

    mavlink_msg_distance_sensor_encode(1, 200, &msg, &sensor_msg); // sysid=1, compid=200
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    PixhawkSerial.write(buf, len);
}
*/