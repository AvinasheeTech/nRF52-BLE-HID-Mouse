#ifndef _HID_H_
#define _HID_H_

//for bluetooth
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>

//service and characteristics 
#define NRF52_SVC_HID_UUID         BT_UUID_HIDS_VAL                      //HID Service
#define NRF52_SVC_BAT_UUID         BT_UUID_BAS_VAL                       //Battery Status Service
#define NRF52_SVC_DEV_UUID         BT_UUID_DIS_VAL                       //Device Information Service

#define NRF52_CHAR_HIDINFO_UUID        BT_UUID_HIDS_INFO           //HID Information
#define NRF52_CHAR_HIDMAP_UUID         BT_UUID_HIDS_REPORT_MAP     //HID Report Map
#define NRF52_CHAR_HIDCTRL_UUID        BT_UUID_HIDS_CTRL_POINT     //HID Control Point
#define NRF52_CHAR_HIDREPORT_UUID      BT_UUID_HIDS_REPORT         //HID Input Report
#define NRF52_CHAR_HIDPROT_UUID        BT_UUID_HIDS_PROTOCOL_MODE  //HID Protocol Mode
#define NRF52_CHAR_HIDBOOTREPORT_UUID  BT_UUID_HIDS_BOOT_MOUSE_IN_REPORT  //HID Boot Mouse Input Report

/* 
 * HID Input Report Structure
 * 8 bits - pressed buttons bitmask (2 LSB used + 6 MSB padding)
 * 16 bits - x movement (little endian)
 * 16 bits - y movement (little endian)
 * 8 bits - vertical wheel rotation 
 * NOTE: pack your structures to avoid byte padding and save power and 
 * transmission time over BLE
 */
typedef struct
{
    uint8_t         buttons_bitmask;
    uint8_t         move_x_lsb;
    uint8_t         move_x_msb;
    uint8_t         move_y_lsb;
    uint8_t         move_y_msb;
    uint8_t         scroll_v;
} __attribute__((__packed__)) hids_report_mouse_t;


/*
 * HID Boot Report Structure
 *  8 bits - pressed buttons bitmask
 *  8 bits - x movement
 *  8 bits - y movement
 *  8 bits - vertical wheel rotation (device specific)
 * NOTE: Boot Report is fixed to 4 bytes and CANNOT be modified.
 */
typedef struct
{
    uint8_t         buttons_bitmask;
    int8_t          move_x;
    int8_t          move_y;
    int8_t          scroll_v;

} __attribute__((__packed__)) hids_report_mouse_boot_t;


/*
 * ble function declaration
*/
void input_rep_changed(const struct bt_gatt_attr *attr, uint16_t value);
void boot_input_rep_changed(const struct bt_gatt_attr *attr, uint16_t value);

ssize_t readinfo_cb(struct bt_conn *conn,const struct bt_gatt_attr *attr,void *buf, uint16_t len,uint16_t offset);
ssize_t readreportmap_cb(struct bt_conn *conn,const struct bt_gatt_attr *attr,void *buf, uint16_t len,uint16_t offset);
ssize_t readreport_cb(struct bt_conn *conn,const struct bt_gatt_attr *attr,void *buf, uint16_t len,uint16_t offset);
ssize_t read_reportdesc_cb(struct bt_conn *conn,const struct bt_gatt_attr *attr,void *buf, uint16_t len,uint16_t offset);
ssize_t readprot_cb(struct bt_conn *conn,const struct bt_gatt_attr *attr,void *buf, uint16_t len,uint16_t offset);
ssize_t readreport_cb(struct bt_conn *conn,const struct bt_gatt_attr *attr,void *buf, uint16_t len,uint16_t offset);

ssize_t writectrl_cb(struct bt_conn *conn,const struct bt_gatt_attr *attr,const void *buf, uint16_t len,uint16_t offset, uint8_t flags);
ssize_t writeprot_cb(struct bt_conn *conn,const struct bt_gatt_attr *attr,const void *buf, uint16_t len,uint16_t offset, uint8_t flags);

void hids_connected(struct bt_conn *conn);
void hids_disconnected(void);

uint8_t hids_prot_mode(void);
bool hids_report_writable(void);

void hids_mouse_notify_input(const void* data, uint8_t dataLen);
void hids_mouse_notify_input_boot(const void* data, uint8_t dataLen);

#endif