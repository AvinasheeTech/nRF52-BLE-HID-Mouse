#include "hid.h"


extern struct bt_conn *my_connection; //bluetooth connection reference struct

/********************
 * HID Information  *
*********************/ 

// HID Info flags 
enum
{
    REMOTE_WAKE            = BIT(0),
    NORMALLY_CONNECTABLE   = BIT(1),
};

/* 
 * HID Info structure
 * NOTE: pack your structures to avoid byte padding and save power and 
 * transmission time over BLE
 */
typedef struct{
    uint16_t hid_version;
    uint8_t country_code;
    uint8_t flags;
}__attribute__((__packed__)) _hids_info_t_;

const _hids_info_t_ hids_info = {
    .hid_version = 0x0101,
    .country_code = 0x00,
    .flags = NORMALLY_CONNECTABLE,
};

// HID Info callback
ssize_t readinfo_cb(struct bt_conn *conn,const struct bt_gatt_attr *attr,void *buf, uint16_t len,uint16_t offset){
    return bt_gatt_attr_read(conn,attr,buf,len,offset,&hids_info,sizeof(_hids_info_t_));
}

/********************
 * HID Report Map  *
*********************/ 

// HID Report map
const uint8_t report_map[] =
{
    /* MOUSE INPUT REPORT MAP */
    0x05, 0x01,                 // Usage Page (Generic Desktop Ctrls)
    0x09, 0x02,                 // Usage (Mouse)
    0xA1, 0x01,                 // Collection (Application)
    0x85, 0x01,                 //   Report ID (1)
    0x09, 0x01,                 //   Usage (Pointer)
    0xA1, 0x00,                 //   Collection (Physical)
    0x95, 0x02,                 //     Report Count (2)
    0x75, 0x01,                 //     Report Size (1)
    0x15, 0x00,                 //     Logical Minimum (0)
    0x25, 0x01,                 //     Logical Maximum (1)
    0x05, 0x09,                 //     Usage Page (Button)
    0x19, 0x01,                 //     Usage Minimum (0x01)
    0x29, 0x02,                 //     Usage Maximum (0x02)
    0x81, 0x02,                 //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x95, 0x01,                 //     Report Count (1)
    0x75, 0x06,                 //     Report Size (6)
    0x81, 0x03,                 //     Input (Cnst, Var, Abs)
    0x05, 0x01,                 //     Usage Page (Generic Desktop Ctrls)
    0x16, 0x00, 0x80,           //     Logical Minimum (-32768)
    0x26, 0xFF, 0x7F,           //     Logical Maximum (32767)
    0x75, 0x10,                 //     Report Size (16)
    0x95, 0x02,                 //     Report Count (2)
    0x09, 0x30,                 //     Usage (X)
    0x09, 0x31,                 //     Usage (Y)
    0x81, 0x06,                 //     Input (Data,Var,Rel,No Wrap,Linear,Preferred State,No Null Position)
    0x15, 0x81,                 //     Logical Minimum (-127)
    0x25, 0x7F,                 //     Logical Maximum (127)
    0x75, 0x08,                 //     Report Size (8)
    0x95, 0x01,                 //     Report Count (1)
    0x09, 0x38,                 //     Usage (Wheel)
    0x81, 0x06,                 //     Input (Data,Var,Rel,No Wrap,Linear,Preferred State,No Null Position)
    0xC0,                       //   End Collection
    0xC0,                       // End Collection
};

//HID Report Map callback
ssize_t readreportmap_cb(struct bt_conn *conn,const struct bt_gatt_attr *attr,void *buf, uint16_t len,uint16_t offset){
    return bt_gatt_attr_read(conn,attr,buf,len,offset,report_map,sizeof(report_map));
}

/*********************
 * HID Control Point *
**********************/ 

//HID Control Point Device States
enum{
    SUSPEND = 0x00,
    EXIT_SUSPEND = 0x01,
};

typedef uint8_t hid_ctrl_point;

//HID Control Point callback
ssize_t writectrl_cb(struct bt_conn *conn,const struct bt_gatt_attr *attr,const void *buf, uint16_t len,uint16_t offset, uint8_t flags){
    
    uint8_t ctrl_point_value = *((uint8_t*)buf);
    hid_ctrl_point ctrl_point;
    uint8_t* ctrl_point_buf = (uint8_t*)&ctrl_point;

    //check flags
    if(flags!=BT_GATT_WRITE_FLAG_CMD){
        /* Only write without response accepted */
        return BT_GATT_ERR(BT_ATT_ERR_WRITE_REQ_REJECTED);
    }

    //check length
    if((offset+len)>sizeof(hid_ctrl_point)){
        /* Make sure client writes with correct offset */
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    //check value
    if((ctrl_point_value!=SUSPEND) | (ctrl_point_value!=EXIT_SUSPEND)){
        return BT_GATT_ERR(BT_ATT_ERR_NOT_SUPPORTED);
    }

    memcpy(ctrl_point_buf+offset, buf, len);

    return len;

}



/*********************
 * HID Input Report  *
**********************/ 

bool input_rep_notif_enabled;

#define REPORT_MOUSE_SIZE sizeof(hids_report_mouse_t)
uint8_t input_report[REPORT_MOUSE_SIZE];

//HID Report Info structure
typedef struct
{
    uint8_t* const report_ref;
    uint8_t report_len;

} hids_report_info_t;

const hids_report_info_t input_rep_info =
{
    .report_ref = input_report,
    .report_len = sizeof(input_report),
};

// HID Report Characteristic Descriptor 
typedef struct
{
    uint8_t id;                 //report id 
    uint8_t type;               //report type
} __packed hids_report_desc_t;

// Report descriptors
const uint8_t HIDS_REPORT_ID_MOUSE          =  0x01;

// Report direction
enum{
    HIDS_REPORT_INPUT   = 0x01,
    HIDS_REPORT_OUTPUT  = 0x02,
    HIDS_REPORT_FEATURE = 0x03,
};

//descriptor
const hids_report_desc_t input_desc =
{
    .id = HIDS_REPORT_ID_MOUSE,
    .type = HIDS_REPORT_INPUT,
};

/*
 * HID Regular Report read callback
*/
ssize_t readreport_cb(struct bt_conn *conn,const struct bt_gatt_attr *attr,void *buf, uint16_t len,uint16_t offset){
    const hids_report_info_t* hids_report = (const hids_report_info_t*)attr->user_data;
    return bt_gatt_attr_read(conn,attr,buf,len,offset,hids_report->report_ref,hids_report->report_len);
}

/*
 * CCCD callback function for regular report mode
*/
void input_rep_changed(const struct bt_gatt_attr *attr, uint16_t value){
    ARG_UNUSED(attr);
    switch(value)
    {
        case BT_GATT_CCC_NOTIFY: 
            // Start sending stuff! No ACK
			input_rep_notif_enabled = 1;  //notify data send start
            break;

        case 0: 
            // Stop sending stuff
			input_rep_notif_enabled = 0;  //notify data send stop
            break;
        
        default: 
            //Error, CCCD set to an invalid value    
    } 
}

/*
 * regular report description read callback
*/
ssize_t read_reportdesc_cb(struct bt_conn *conn,const struct bt_gatt_attr *attr,void *buf, uint16_t len,uint16_t offset){
    return bt_gatt_attr_read(conn,attr,buf,len,offset,attr->user_data,sizeof(hids_report_desc_t));
}

/*********************
 * HID Protocol Mode *
**********************/

uint8_t prot_mode;

//HID Protocol modes
enum{
    BOOT_MODE   = 0x00,
    REPORT_MODE = 0x01,
};

/* 
 * HID Protocol read callback
*/
ssize_t readprot_cb(struct bt_conn *conn,const struct bt_gatt_attr *attr,void *buf, uint16_t len,uint16_t offset){
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &prot_mode, sizeof(prot_mode));
}

/*
 * HID Protocol write callback
*/
ssize_t writeprot_cb(struct bt_conn *conn,const struct bt_gatt_attr *attr,const void *buf, uint16_t len,uint16_t offset, uint8_t flags){

    uint8_t prot_mode_value = *(uint8_t*)buf;
    uint8_t* prot_mode_buf = (uint8_t*)&prot_mode;

    //check flags
    if(flags!=BT_GATT_WRITE_FLAG_CMD){
        /* Only write without response accepted */
        return BT_GATT_ERR(BT_ATT_ERR_WRITE_REQ_REJECTED);
    }

    //check len
    if((offset+len)>sizeof(prot_mode)){
        /* Make sure client writes with correct offset */
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    //check value
    if((prot_mode_value!=BOOT_MODE) | (prot_mode_value!=REPORT_MODE)){
        return BT_GATT_ERR(BT_ATT_ERR_NOT_SUPPORTED);
    }

    memcpy(prot_mode_buf+offset, buf, len);

    return len;

}

/*******************************
 * HID Boot Mouse Input Report *
********************************/

#define BOOT_REPORT_MOUSE_SIZE sizeof(hids_report_mouse_boot_t)

uint8_t boot_input_report[BOOT_REPORT_MOUSE_SIZE];
bool boot_input_rep_notif_enabled;

//hid boot report structure init 
const hids_report_info_t boot_input_rep_info =
{
    .report_ref = boot_input_report,
    .report_len = sizeof(boot_input_report)
};

/*
 * CCCD callback function for boot report mode
*/
void boot_input_rep_changed(const struct bt_gatt_attr *attr, uint16_t value){
    ARG_UNUSED(attr);
    switch(value)
    {
        case BT_GATT_CCC_NOTIFY: 
            // Start sending stuff! No ACK
			boot_input_rep_notif_enabled = 1;  //notify data send start
            break;

        case 0: 
            // Stop sending stuff
			boot_input_rep_notif_enabled = 0;  //notify data send stop
            break;
        
        default: 
            //Error, CCCD set to an invalid value    
    } 
}

/*
 * register ble service  (GATT)
*/
BT_GATT_SERVICE_DEFINE(my_service,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_HIDS),      //Service Setup
    //Characteristic setup
    //HID Information
    BT_GATT_CHARACTERISTIC(NRF52_CHAR_HIDINFO_UUID, 
                            BT_GATT_CHRC_READ,
                            BT_GATT_PERM_READ,
                            readinfo_cb, NULL, NULL),
    //HID Report Map
    BT_GATT_CHARACTERISTIC(NRF52_CHAR_HIDMAP_UUID,
                            BT_GATT_CHRC_READ,
                            BT_GATT_PERM_READ,
                            readreportmap_cb, NULL, NULL),
    //HID Control Point
    BT_GATT_CHARACTERISTIC(NRF52_CHAR_HIDCTRL_UUID,
                            BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                            BT_GATT_PERM_WRITE,
                            NULL, writectrl_cb, NULL),
    //HID Input Report
    BT_GATT_CHARACTERISTIC(NRF52_CHAR_HIDREPORT_UUID,
                            BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                            BT_GATT_PERM_READ,
                            readreport_cb, NULL,
                            (hids_report_info_t*)&input_rep_info),
    
    BT_GATT_CCC(input_rep_changed,  //Client Characteristic Configuration Descriptor - Enable/Disable of Notification
        BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    
    BT_GATT_DESCRIPTOR(BT_UUID_HIDS_REPORT_REF,
                        BT_GATT_PERM_READ,
                        read_reportdesc_cb, NULL,
                        (hids_report_desc_t*) &input_desc),
    //HID Protocol Mode
    BT_GATT_CHARACTERISTIC(NRF52_CHAR_HIDPROT_UUID,
                            BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                            BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                            readprot_cb, writeprot_cb, NULL),
    //HID Boot Mouse Input Report
    BT_GATT_CHARACTERISTIC(NRF52_CHAR_HIDBOOTREPORT_UUID,
                            BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                            BT_GATT_PERM_READ,
                            readreport_cb, NULL,
                            (hids_report_info_t*)&boot_input_rep_info),
    
    BT_GATT_CCC(boot_input_rep_changed,  //Client Characteristic Configuration Descriptor - Enable/Disable of Notification
            BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    
);

/*
 * updates connection reference once device connected
*/
void hids_connected(struct bt_conn *conn){
    my_connection = conn;
    prot_mode = REPORT_MODE; //Set Protocol Mode back to default (Report)
}

/*
 * updates connection reference once device disconnected
*/
void hids_disconnected(void){
    my_connection = NULL;
}

/*
 * returns current selected protocol mode by client (regular report or boot report)
*/
uint8_t hids_prot_mode(void){
    return prot_mode;
}

/*
 * checks if report is writable or not (CCCD flag enabled or not)
*/
bool hids_report_writable(void){
    if(prot_mode == BOOT_MODE)
    {
        return boot_input_rep_notif_enabled;
    }
    else
    {
        return input_rep_notif_enabled;
    }
}

/*
 *@brief : function to notify regular report data
 *@param : pointer to data structure, length of that data
 *@retval : None 
 *@note :  This function is called to notify client with updated regular report data
*/
void hids_mouse_notify_input(const void* data, uint8_t dataLen)
{
    __ASSERT_NO_MSG(input_rep_notif_enabled);  //check if notify enabled

    int err;
    struct bt_gatt_notify_params params = {0};

    memcpy(input_report, data, dataLen);

    //params update
    params.uuid = NRF52_CHAR_HIDREPORT_UUID;
    params.data = data;
    params.len = dataLen;
    params.func = NULL;

    err = bt_gatt_notify_cb(my_connection, &params);
}

/*
 *@brief : function to notify boot report data
 *@param : pointer to data structure, length of that data
 *@retval : None 
 *@note :  This function is called to notify client with updated boot report data
*/
void hids_mouse_notify_input_boot(const void* data, uint8_t dataLen){

    __ASSERT_NO_MSG(boot_input_rep_notif_enabled); //check if notify enabled

    int err;
    struct bt_gatt_notify_params params = {0};

    memcpy(boot_input_report, data, dataLen);

    //params update
    params.uuid = NRF52_CHAR_HIDBOOTREPORT_UUID;        
    params.data = data;
    params.len = dataLen;
    params.func = NULL;

    err = bt_gatt_notify_cb(my_connection, &params);
}