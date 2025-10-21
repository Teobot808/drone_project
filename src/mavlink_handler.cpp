#include "mavlink_handler.h"

TelemetryData telemetryData = {};

void handleMavlinkMessage(mavlink_message_t* msg) {
    switch (msg->msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT: {
            mavlink_heartbeat_t hb;
            mavlink_msg_heartbeat_decode(msg, &hb);
            telemetryData.system_mode = hb.base_mode;
            telemetryData.system_state = hb.system_status;
            break;
        }

        case MAVLINK_MSG_ID_SYS_STATUS: {
            mavlink_sys_status_t sys;
            mavlink_msg_sys_status_decode(msg, &sys);
            telemetryData.battery_voltage = sys.voltage_battery / 1000.0f; // mV -> V
            telemetryData.battery_current = sys.current_battery / 100.0f;  // cA -> A
            telemetryData.battery_remaining = sys.battery_remaining;
            break;
        }

        case MAVLINK_MSG_ID_ATTITUDE: {
            mavlink_attitude_t att;
            mavlink_msg_attitude_decode(msg, &att);
            telemetryData.roll = att.roll;
            telemetryData.pitch = att.pitch;
            telemetryData.yaw = att.yaw;
            break;
        }

        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
            mavlink_global_position_int_t pos;
            mavlink_msg_global_position_int_decode(msg, &pos);
            telemetryData.lat = pos.lat;
            telemetryData.lon = pos.lon;
            telemetryData.alt = pos.alt;
            break;
        }

        case MAVLINK_MSG_ID_GPS_RAW_INT: {
            mavlink_gps_raw_int_t gps;
            mavlink_msg_gps_raw_int_decode(msg, &gps);
            telemetryData.fix_type = gps.fix_type;
            telemetryData.satellites_visible = gps.satellites_visible;
            break;
        }

        case MAVLINK_MSG_ID_RAW_IMU: {
            mavlink_raw_imu_t imu;
            mavlink_msg_raw_imu_decode(msg, &imu);
            telemetryData.accel_x = imu.xacc / 1000.0f;
            telemetryData.accel_y = imu.yacc / 1000.0f;
            telemetryData.accel_z = imu.zacc / 1000.0f;
            telemetryData.gyro_x = imu.xgyro / 1000.0f;
            telemetryData.gyro_y = imu.ygyro / 1000.0f;
            telemetryData.gyro_z = imu.zgyro / 1000.0f;
            telemetryData.mag_x = imu.xmag / 1000.0f;
            telemetryData.mag_y = imu.ymag / 1000.0f;
            telemetryData.mag_z = imu.zmag / 1000.0f;
            break;
        }

        case MAVLINK_MSG_ID_VFR_HUD: {
            mavlink_vfr_hud_t hud;
            mavlink_msg_vfr_hud_decode(msg, &hud);
            telemetryData.airspeed = hud.airspeed;
            telemetryData.groundspeed = hud.groundspeed;
            telemetryData.heading = hud.heading;
            telemetryData.throttle = hud.throttle;
            telemetryData.climb_rate = hud.climb;
            break;
        }

        default:
            // You can log other message IDs for debugging
            // Serial.printf("Received MAVLink message ID: %d\n", msg->msgid);
            break;
    }
}

// Prints selected key telemetry values to Serial
void printEssentialTelemetry() {
    Serial.println("📡 --- ESSENTIAL TELEMETRY ---");
    Serial.printf("Battery: %.2f V, %.2f A, %.0f %%\n", 
                  telemetryData.battery_voltage, 
                  telemetryData.battery_current, 
                  telemetryData.battery_remaining);
    Serial.printf("Attitude: Roll %.2f, Pitch %.2f, Yaw %.2f\n", 
                  telemetryData.roll, telemetryData.pitch, telemetryData.yaw);
    Serial.printf("GPS: fix %d, sats %d, lat %.7f, lon %.7f, alt %.2fm\n",
                  telemetryData.fix_type,
                  telemetryData.satellites_visible,
                  telemetryData.lat / 1e7,
                  telemetryData.lon / 1e7,
                  telemetryData.alt / 1000.0);
    Serial.printf("Speed: air %.2f, ground %.2f, throttle %d, climb %.2f\n",
                  telemetryData.airspeed, 
                  telemetryData.groundspeed, 
                  telemetryData.throttle, 
                  telemetryData.climb_rate);
    Serial.println("-------------------------------\n");
}
