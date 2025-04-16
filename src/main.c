//header files
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>  //for uart
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>  //for button
#include <zephyr/sys/util.h>
#include <inttypes.h>

#include <zephyr/settings/settings.h>

#include "hid.h"

#define DEBUG 0     //enable uart based debug messages

/*
 * Button Macros and devicetree container
*/
#define BUTTON1_NODE  DT_NODELABEL(input_signal1)
#define BUTTON2_NODE  DT_NODELABEL(input_signal2)
#define BUTTON3_NODE  DT_NODELABEL(input_signal3)
#define BUTTON4_NODE  DT_NODELABEL(input_signal4)

#define BUTTON1_PIN  13   //0.13
#define BUTTON2_PIN  9    //0.09
#define BUTTON3_PIN  15   //0.15
#define BUTTON4_PIN  10   //0.10

static const struct gpio_dt_spec button1 = GPIO_DT_SPEC_GET_OR(BUTTON1_NODE, gpios,
							      {0});
static const struct gpio_dt_spec button2 = GPIO_DT_SPEC_GET_OR(BUTTON2_NODE, gpios,
                                  {0});
static const struct gpio_dt_spec button3 = GPIO_DT_SPEC_GET_OR(BUTTON3_NODE, gpios,
                                  {0});
static const struct gpio_dt_spec button4 = GPIO_DT_SPEC_GET_OR(BUTTON4_NODE, gpios,
                                  {0});


/*
 * Uart configuration
*/

const struct device *uart_dev = DEVICE_DT_GET(DT_NODELABEL(uart0));
const struct uart_config uart_cfg = {
		.baudrate = 115200,
		.parity = UART_CFG_PARITY_NONE,
		.stop_bits = UART_CFG_STOP_BITS_1,
		.data_bits = UART_CFG_DATA_BITS_8,
		.flow_ctrl = UART_CFG_FLOW_CTRL_NONE
};


/*
 * Bluetooth Macros and variables
*/

/* Key used for left click, move cursor left, move cursor up or scroll up */
#define KEY_LEFT_UP_MASK   			BIT(BUTTON1_PIN)
/* Key used for right click, move cursor right, move cursor down or scroll down */
#define KEY_RIGHT_DOWN_MASK  		BIT(BUTTON2_PIN)
/* Key used to move to the next input mode */
#define KEY_INPUT_MODE_PREV_MASK	BIT(BUTTON3_PIN)
/* Key used to the previous input mode */
#define KEY_INPUT_MODE_NEXT_MASK	BIT(BUTTON4_PIN)

#define MOVE_STEP_SIZE		15        //step size for movement in x and y direction
#define SCROLL_STEP_SIZE	1         //scroll step size for scrolling in vertical direction
#define LEFT_CLICK          BIT(0)    //left click bit position
#define RIGHT_CLICK         BIT(1)    //right click bit positiion
#define KEYS_MONITORING_PERIOD_MS	50   //monitor key after given interval

#define DEVICE_NAME             CONFIG_BT_DEVICE_NAME     //ble device name
#define DEVICE_NAME_LEN         (sizeof(DEVICE_NAME) - 1) 

#define BT_ADV_INT_MIN  48 /* 0.625ms units --> 30ms */
#define BT_ADV_INT_MAX  80 /* 0.625ms units --> 50ms */

//modes available
typedef enum
{
    INPUT_MODE_CLICK,
    INPUT_MODE_MOVEX,
    INPUT_MODE_MOVEY,
    INPUT_MODE_SCROLL,

    INPUT_MODE_N

} inputMode_t;

inputMode_t curr_input_mode;

char ble_uart_buffer[150] = {0};    //buffer to transmit bluetooth specific messages over uart
volatile uint8_t notify_client = 0; //flag for notification enable/disable status
struct bt_conn *my_connection;      //bluetooth connection reference struct

static bt_addr_le_t bonded_addr;    //record bonded device address
static bool bonded_addr_present;    //check if bonded device present

static bool dir_adv_timeout;
static bool is_connected;
static int64_t last_advertising_start_ms;

static struct k_work adv_work;      //advertising work structure
static K_SEM_DEFINE(ble_init_ok, 0, 1); //semaphore for ble initialization

/*
 * ble device advertisement (GAP)
*/

static const struct bt_data ad[] = //advertising data setup
{
    BT_DATA_BYTES(BT_DATA_GAP_APPEARANCE,
        (CONFIG_BT_DEVICE_APPEARANCE >> 0) & 0xff,
        (CONFIG_BT_DEVICE_APPEARANCE >> 8) & 0xff),

	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_LIMITED | BT_LE_AD_NO_BREDR)), 

    BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(NRF52_SVC_HID_UUID), BT_UUID_16_ENCODE(NRF52_SVC_BAT_UUID),
                BT_UUID_16_ENCODE(NRF52_SVC_DEV_UUID),
),
	
};
static const struct bt_data sd[] = //scan response data setup
{
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

/*
 *@brief : function to retrieve bonded address
 *@param : bonded device info pointer
 *@retval : None 
 *@note : gets the stored bonded device address
*/
static void bonded_addr_cache(const struct bt_bond_info *info, void *user_data)
{
    memcpy(&bonded_addr, &info->addr, sizeof(bt_addr_le_t));
    bonded_addr_present = true;
}

/*
 *@brief : advertising start function
 *@param : None
 *@retval : None 
 *@note : only one paired device is supported, so the function passed as argument is invoked once at most
*/
static void advertising_start(void)
{
    bt_foreach_bond(BT_ID_DEFAULT, bonded_addr_cache, NULL); //go through existing bonds
    k_work_submit(&adv_work);  //submit work for execution to system workqueue thread
}

/*
 *@brief : uart tranmsit function
 *@param : buffer to be transmitted
 *@retval : None 
 *@note : function to transmit uart messages using blocking poll out method
*/
void nrf52_uart_tx(uint8_t *tx_buff){
	for(int buflen=0;buflen<=strlen(tx_buff);buflen++){
                uart_poll_out(uart_dev,*(tx_buff+buflen));
	}
	k_msleep(100);
}


/*
 *@brief : bluetooth notification function
 *@param : button mask, movement along x axis or horizontal direction, movement along y axis or 
           vertical direction, scroll along vertical direction
 *@retval : None 
 *@note : This function sends a notification to a Client with the provided data according to the protocol
          mode selected (boot or report) if notification is enabled
*/
void send_notification(uint8_t button_bitmasks, int16_t movement_x, int16_t movement_y, int8_t scroll)
{
    uint8_t current_prot_mode = hids_prot_mode();

    if(hids_report_writable()){               //check if respective mode notification is enabled
        if(current_prot_mode==0x00){          //Boot Mode
            hids_report_mouse_boot_t boot_report_t = {    //fix data format of 4 bytes
                .buttons_bitmask = button_bitmasks,
                .move_x = (int8_t)movement_x,
                .move_y = (int8_t)movement_y,
                .scroll_v = scroll,
            };

            hids_mouse_notify_input_boot(&boot_report_t,sizeof(hids_report_mouse_boot_t));  //sends notification


        }else if(current_prot_mode==0x01){    //Report Mode
            hids_report_mouse_t report_t = {              //flexible data format for added features
                .buttons_bitmask = button_bitmasks,
                .move_x_lsb = (uint8_t)(movement_x & 0xFF),
                .move_x_msb = (uint8_t)((movement_x>>8)&0xFF),
                .move_y_lsb = (uint8_t)(movement_y & 0xFF),
                .move_y_msb = (uint8_t)((movement_y>>8)&0xFF),
                .scroll_v = scroll,
            };

            hids_mouse_notify_input(&report_t, sizeof(hids_report_mouse_t));                  //sends notification
        }
    }
}


/*
 *@brief : function to select button input mode
 *@param : buttons that are set at the moment
 *@retval : None 
 *@note : user can select next or previous input mode using respective buttons (marked N and P on breadboard)
          Mode 0 : Left & Right Click
          Mode 1 : Left & Right Movement or Movement along x axis
          Mode 2 : Up & Down Movement or Movement along y axis
          Mode 3 : Up & Down Scroll Movement or Vertical Scrolling 
*/
static void update_input_modes(uint32_t buttons_set)
{
    inputMode_t new_input_mode = curr_input_mode;

    if (buttons_set & KEY_INPUT_MODE_NEXT_MASK)
    {
        /* Go to the next mode */
        new_input_mode = (curr_input_mode + 1) % INPUT_MODE_N;
    }
    else if (buttons_set & KEY_INPUT_MODE_PREV_MASK)
    {
        /* Go to previous mode */
        if(curr_input_mode == 0)
        {
            new_input_mode = INPUT_MODE_N - 1;
        }
        else
        {
            new_input_mode = curr_input_mode - 1;
        }
    }

    if (new_input_mode != curr_input_mode)
    {
        curr_input_mode = new_input_mode;  //update mode

#if DEBUG
        memset(ble_uart_buffer,0,sizeof(ble_uart_buffer));
        sprintf(ble_uart_buffer,"New Input Mode = %d\n", curr_input_mode);
        nrf52_uart_tx(ble_uart_buffer);
#endif

    }
}


/*
 *@brief : function to act according to input mode selected
 *@param : all buttons state at the moment, buttons whose state has changed
 *@retval : None 
 *@note : take action according to the mode selected using button left and right (marked L and R on breadboard)
*/
static void take_action(uint32_t button_state, uint32_t has_changed)
{
    if (curr_input_mode == INPUT_MODE_CLICK)        //Mode 0 - Left and Right Click
    {
        if((has_changed & KEY_LEFT_UP_MASK) & (button_state & KEY_LEFT_UP_MASK)){
            send_notification(LEFT_CLICK, 0, 0, 0);  //left click notify
#if DEBUG
            nrf52_uart_tx("left click\n");
#endif
        } 
        else if((has_changed & KEY_RIGHT_DOWN_MASK) & (button_state & KEY_RIGHT_DOWN_MASK))
        {
            send_notification(RIGHT_CLICK, 0, 0, 0);  //right click notify
#if DEBUG
            nrf52_uart_tx("right click\n");
#endif
        }
        else{
            send_notification(0,0,0,0);
        }
    }
    else if (curr_input_mode == INPUT_MODE_MOVEX)    //Mode 1 - Left and Right Movement
    {
        if(button_state & KEY_LEFT_UP_MASK)
        {
            send_notification(0, -(MOVE_STEP_SIZE), 0, 0);
        }
        else if(button_state & KEY_RIGHT_DOWN_MASK)
        {
            send_notification(0, MOVE_STEP_SIZE, 0, 0);
        }
    }
    else if(curr_input_mode == INPUT_MODE_MOVEY)      //Mode 2 - Up and Down Movement
    {
        if(button_state & KEY_LEFT_UP_MASK)
        {
            send_notification(0, 0, -(MOVE_STEP_SIZE), 0);
        }
        else if(button_state & KEY_RIGHT_DOWN_MASK)
        {
            send_notification(0, 0, MOVE_STEP_SIZE, 0);
        }
    }
    else if(curr_input_mode == INPUT_MODE_SCROLL)     //Mode 3 - Up and Down Scroll
    {
        if(button_state & KEY_LEFT_UP_MASK)
        {
            send_notification(0, 0, 0, SCROLL_STEP_SIZE);
        }
        else if(button_state & KEY_RIGHT_DOWN_MASK)
        {
            send_notification(0, 0, 0, -(SCROLL_STEP_SIZE));
        }
    }
}


/*
 *@brief : function to execute advertising
 *@param : None
 *@retval : None 
 *@note : Directed or regular advertising options available
*/
static void advertising_exec(void)
{
    int err = 0;
    struct bt_le_adv_param adv_param;

    /* Directed advertising if bonding is present and we did not already timed out */
    if ((bonded_addr_present) && (!dir_adv_timeout))
    {
        char addr_buf[BT_ADDR_LE_STR_LEN];

        adv_param = *BT_LE_ADV_CONN_DIR(&bonded_addr);
        adv_param.options |= BT_LE_ADV_OPT_DIR_ADDR_RPA;

        err = bt_le_adv_start(&adv_param, NULL, 0, NULL, 0);      //start adv.
        if (err)
        {

#if DEBUG
            memset(ble_uart_buffer,0,sizeof(ble_uart_buffer));
            sprintf(ble_uart_buffer,"Directed advertising failed to start (err %d)\n", err);
            nrf52_uart_tx(ble_uart_buffer);
#endif

            return;
        }

        bt_addr_le_to_str(&bonded_addr, addr_buf, BT_ADDR_LE_STR_LEN);  //bonded addr to string

#if DEBUG
        memset(ble_uart_buffer,0,sizeof(ble_uart_buffer));
        sprintf(ble_uart_buffer,"Direct advertising to %s started\n", addr_buf);
        nrf52_uart_tx(ble_uart_buffer);
#endif

    }
    /* Regular advertising if no bonding is present */
    else
    {
        adv_param = *BT_LE_ADV_CONN;
        adv_param.interval_min = BT_ADV_INT_MIN;
        adv_param.interval_max = BT_ADV_INT_MAX;
        adv_param.options |= BT_LE_ADV_OPT_ONE_TIME | BT_LE_ADV_OPT_SCANNABLE;

        err = bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));  //start adv.
        if (err)
        {

#if DEBUG
            memset(ble_uart_buffer,0,sizeof(ble_uart_buffer));
            sprintf(ble_uart_buffer,"Advertising failed to start (err %d)\n", err);
            nrf52_uart_tx(ble_uart_buffer);
#endif

            return;
        }

#if DEBUG
        nrf52_uart_tx("Regular advertising started\n");
#endif

    }

    if(err == 0)
    {
        /* Cache last start of advertising */
        last_advertising_start_ms = k_uptime_get();
    }
}

/*
 *@brief : work item handler function
 *@param : pointer to work structure 
 *@retval : None 
*/
static void advertising_process(struct k_work *work)
{
    advertising_exec();
}

/*
 *@brief : bluetooth connection callback function
 *@param : bluetooth connection structure , error status
 *@retval : None 
 *@note : This function is called whenever the device is connected with a Client
*/
static void connected(struct bt_conn *conn, uint8_t err)
{
	struct bt_conn_info info; 
	char addr[BT_ADDR_LE_STR_LEN];

	my_connection = conn;

    if (err) 
    {
        if (err == BT_HCI_ERR_ADV_TIMEOUT)  //adv timed out error
        {

#if DEBUG
            nrf52_uart_tx("Direct advertising timed out\n");
#endif

            dir_adv_timeout = true;
            k_work_submit(&adv_work);       //resubmit adv work
        }
        else
        {

#if DEBUG            
            memset(ble_uart_buffer,0,sizeof(ble_uart_buffer));
            sprintf(ble_uart_buffer,"Failed to connect (%u)\n", err);
            nrf52_uart_tx(ble_uart_buffer);
#endif

        }

        return;
    }

    //check for connection info on successful connection
	if(bt_conn_get_info(conn, &info)!=0)
	{

#if DEBUG
		nrf52_uart_tx("connection info unavailable\n");
#endif

		return;
	}
	else
	{
		
		bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));  //get peer address in string format

#if DEBUG
		memset(ble_uart_buffer,0,sizeof(ble_uart_buffer));
		sprintf(ble_uart_buffer,"Connected to: %s					\n \
			Role: %u							\n \
			Conn ivl: %u				\n \
			Slv lat: %u					\n \
			Conn timeout: %u	\n" 
			, addr, info.role, info.le.interval, info.le.latency, info.le.timeout
		);
		nrf52_uart_tx(ble_uart_buffer);
#endif

    }

    //update variables
    is_connected = true;
    dir_adv_timeout = false;
    last_advertising_start_ms = 0;

    /* Inform HID Service */
    hids_connected(conn);

}


/*
 *@brief : bluetooth disconnection callback function
 *@param : bluetooth connection structure , reason for disconnection
 *@retval : None 
 *@note :  This function is called whenever the device is disconnected from a Client
*/
static void disconnected(struct bt_conn *conn, uint8_t reason)
{

#if DEBUG
	memset(ble_uart_buffer,0,sizeof(ble_uart_buffer));
	sprintf(ble_uart_buffer,"Dis.(%u)\n", reason);
	nrf52_uart_tx(ble_uart_buffer);
#endif

    is_connected = false;  //update variable

    /* Inform HID Service */
    hids_disconnected();

    /* Re-start advertising */
    advertising_start();
}

/*
 *@brief : security changed callback function
 *@param : bluetooth connection structure, security level selected, error if any 
 *@retval : None 
 *@note :  This function is called whenever Client requests an update in security level
*/
static void security_changed(struct bt_conn *conn, bt_security_t level,
    enum bt_security_err err)
{
    char addr[BT_ADDR_LE_STR_LEN];
    memset(ble_uart_buffer,0,sizeof(ble_uart_buffer));

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));    

    if (!err)
    {
        sprintf(ble_uart_buffer,"Security changed: %s lvl %u\n", addr, level);
    }
    else
    {
        sprintf(ble_uart_buffer,"Security failed: %s lvl %u err %d\n", addr, level, err);
    }
#if DEBUG
    nrf52_uart_tx(ble_uart_buffer);
#endif

}

/*
 *@brief : connection params updated callback function
 *@param : bluetooth connection structure, connection interval, latency and timeout
 *@retval : None 
 *@note :  This function is called whenever Client requests an update in parameters 
           related to connection
*/
static void conn_params_updated(struct bt_conn *conn, uint16_t interval,
    uint16_t latency, uint16_t timeout)
{
    struct bt_conn_info info;
    bt_conn_get_info(conn, &info);

#if DEBUG
    memset(ble_uart_buffer,0,sizeof(ble_uart_buffer));
    sprintf(ble_uart_buffer,"Conn params updated, ivl: %u, lat: %u, timeout: %u\n",
    info.le.interval, info.le.latency, info.le.timeout);
    nrf52_uart_tx(ble_uart_buffer);
#endif

}

/*
  * structure for connection based callbacks
*/
static struct bt_conn_cb conn_callbacks = 
{
	.connected			= connected,
	.disconnected   	= disconnected,
    .security_changed   = security_changed,
    .le_param_updated   = conn_params_updated,
};


/*
 *@brief : pairing complete callback function
 *@param : bluetooth connection structure, bonding status
 *@retval : None 
 *@note :  This function is called when pairing process is completed between device and client
*/
static void pairing_complete(struct bt_conn *conn, bool bonded)
{
    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

#if DEBUG
    memset(ble_uart_buffer,0,sizeof(ble_uart_buffer));
    sprintf(ble_uart_buffer,"Pairing completed: %s, bonded: %d\n", addr, bonded);
    nrf52_uart_tx(ble_uart_buffer);
#endif

}

/*
 *@brief : pairing failure callback function
 *@param : bluetooth connection structure, reason for failure
 *@retval : None 
 *@note :  This function is called when pairing process fails
*/
static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

#if DEBUG
    memset(ble_uart_buffer,0,sizeof(ble_uart_buffer));
    sprintf(ble_uart_buffer,"Pairing failed conn: %s, reason %d\n", addr, reason);
    nrf52_uart_tx(ble_uart_buffer);
#endif

}

/*
  * structure for connection authentication based callbacks
*/
static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
    .pairing_complete = pairing_complete,
    .pairing_failed   = pairing_failed,
};

/*
 *@brief : Callback for notifying that Bluetooth has been enabled.
 *@param : error condition (zero on success or (negative) error code otherwise.)
 *@retval : None 
*/
static void bt_ready(int err)
{
	if (err) 
	{
#if DEBUG
		memset(ble_uart_buffer,0,sizeof(ble_uart_buffer));
		sprintf(ble_uart_buffer,"BLE init failed with error code %d\n", err);
		nrf52_uart_tx(ble_uart_buffer);
#endif
        return;
	}

	//Register connection callback
	bt_conn_cb_register(&conn_callbacks);

    //Register authentication information callback
    bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);

	k_sem_give(&ble_init_ok); //give semaphore
}


/*
 *@brief : function to read button status
 *@param : pointer to current button state variable, pointer to buttons that have changed variable 
 *@retval : None 
 *@note :  This function is called after specified delay of 50ms to read status of buttons.
           Button 1 - used for left click, moving left, moving up and scrolling up 
           Button 2 - used for right click, moving right, moving down and scrolling down
           Button 3 - used to select previous input mode 
           Button 4 - used to select next input mode
*/
void read_buttons(uint32_t* buttons_state, uint32_t* buttons_changed){

    static int previous_button_val[4] = {0};
    int current_button_val[4] = {0};
    uint32_t current_buttons_state = 0;
    uint32_t current_buttons_changed = 0;
    
    //button 1
    current_button_val[0] = gpio_pin_get_dt(&button1); //get button1 status
#if DEBUG
    if(current_button_val[0]==1){
        nrf52_uart_tx("button 1 pressed");
    }
#endif
    __ASSERT_NO_MSG(current_button_val[0]>=0);
    if(previous_button_val[0]!=current_button_val[0]){        //check for any change in button 1
        previous_button_val[0] = current_button_val[0];
        current_buttons_changed |= BIT(BUTTON1_PIN);
        current_buttons_state |= (current_button_val[0]<<BUTTON1_PIN);
    }

    //button 2
    current_button_val[1] = gpio_pin_get_dt(&button2); //get button2 status
#if DEBUG
    if(current_button_val[1]==1){
        nrf52_uart_tx("button 2 pressed");
    }
#endif
    __ASSERT_NO_MSG(current_button_val[1]>=0);                //check for any change in button 2
    if(previous_button_val[1]!=current_button_val[1]){
        previous_button_val[1] = current_button_val[1];
        current_buttons_changed |= BIT(BUTTON2_PIN);
        current_buttons_state |= (current_button_val[1]<<BUTTON2_PIN);
    }

    //button 3
    current_button_val[2] = gpio_pin_get_dt(&button3); //get button3 status
#if DEBUG
    if(current_button_val[2]==1){
        nrf52_uart_tx("button 3 pressed");
    }
#endif
    __ASSERT_NO_MSG(current_button_val[2]>=0);
    if(previous_button_val[2]!=current_button_val[2]){        //check for any change in button 3
        previous_button_val[2] = current_button_val[2];
        current_buttons_changed |= BIT(BUTTON3_PIN);
        current_buttons_state |= (current_button_val[2]<<BUTTON3_PIN);
    }

    //button 4
    current_button_val[3] = gpio_pin_get_dt(&button4); //get button4 status
#if DEBUG
    if(current_button_val[3]==1){
        nrf52_uart_tx("button 4 pressed");
    }
#endif
    __ASSERT_NO_MSG(current_button_val[3]>=0);
    if(previous_button_val[3]!=current_button_val[3]){       //check for any change in button 4
        previous_button_val[3] = current_button_val[3];
        current_buttons_changed |= BIT(BUTTON4_PIN);
        current_buttons_state |= (current_button_val[3]<<BUTTON4_PIN);
    }


    *buttons_state = current_buttons_state;                 //update button status
    *buttons_changed = current_buttons_changed;             //update buttons that have changed
}

/*
 *@brief : main function
 *@param : execution status
 *@retval : None 
*/
int main(void)
{	    
        /******************************************************************** 
        ***************************Button Section****************************
        ********************************************************************/
	
        int err;

        //Button 1 - used for left click, moving left, moving up and scrolling up 
        if (!gpio_is_ready_dt(&button1)) { //check if gpio ready 
                return 0;
        }

        err = gpio_pin_configure_dt(&button1, GPIO_INPUT); //input gpio direction
        if (err != 0) {
                return 0;
        }
	    
        //Button 2 - used for right click, moving right, moving down and scrolling down
        if (!gpio_is_ready_dt(&button2)) { //check if gpio ready 
            return 0;
        }

        err = gpio_pin_configure_dt(&button2, GPIO_INPUT); //input gpio direction
        if (err != 0) {
            return 0;
        }

        //Button 3 - used to select previous input mode 
        if (!gpio_is_ready_dt(&button3)) { //check if gpio ready 
            return 0;
        }

        err = gpio_pin_configure_dt(&button3, GPIO_INPUT); //input gpio direction
        if (err != 0) {
            return 0;
        }
    
        //Button 4 - used to select next input mode
        if (!gpio_is_ready_dt(&button4)) { //check if gpio ready 
            return 0;
        }

        err = gpio_pin_configure_dt(&button4, GPIO_INPUT); //input gpio direction
        if (err != 0) {
           return 0;
        } 


        /********************************************************************* 
        *************************Bluetooth Section****************************
        *********************************************************************/
        
        //Enable Bluetooth
        err = bt_enable(bt_ready);
        if (err) 
        {
		//Bluetooth Enabling failed
#if DEBUG
		  nrf52_uart_tx("bluetooth enable error\n"); 
#endif
	    }

         	
        //Bluetooth stack should be ready in less than 100 msec. 								
        //We use this semaphore to wait for bt_enable to call bt_ready.
        //This task/thread is blocked for the time period or until semaphore is given back.... 	

        err = k_sem_take(&ble_init_ok, K_MSEC(500));
        if (!err) 
        {
		  //Bluetooth initialized
	    } else 
	    {
        //Bluetooth initialization did not complete in Time
#if DEBUG
		  nrf52_uart_tx("bluetooth init timeout\n");
#endif
	    }

        //needed for bonding configuration settings
        if (IS_ENABLED(CONFIG_SETTINGS))   
        {
            settings_load();
        }

    
        /* Init workqueue item to help during advertising */
        k_work_init(&adv_work, advertising_process);

        /* Immediately request advertising to start by submitting work*/
        advertising_start();

        /********************************************************************* 
        ****************************Uart Section******************************
        *********************************************************************/
        
        if (!device_is_ready(uart_dev)) {  //check if device ready
                return 0;
        }

        err = uart_configure(uart_dev, &uart_cfg);  //set cfg.
        if (err == -ENOSYS) {
                return -ENOSYS;
        }

        //variables for button input
        uint32_t buttons_state = 0;
        uint32_t buttons_changed = 0;
        uint32_t buttons_just_set = 0;

        while(1){

            /* If we are not connected, check the advertising timeout as termination condition */
            if(!is_connected)
            {
                int64_t ref_time = last_advertising_start_ms;

                if ((ref_time != 0) && k_uptime_delta(&ref_time) >= (CONFIG_BT_LIM_ADV_TIMEOUT * 1000))
                {
#if DEBUG
                  nrf52_uart_tx("reset board to restart the application...\n");
#endif
                  break;
                }
            }

            /* Get current buttons state and witness buttons just set*/
            read_buttons(&buttons_state, &buttons_changed);
            buttons_just_set = buttons_state & buttons_changed;

            /* Check input mode buttons and update state, if needed */
            update_input_modes(buttons_just_set);

            /* take mouse related action, depending on current input mode */
            take_action(buttons_state, buttons_changed);
        
            /* Sleep */
            k_sleep(K_MSEC(KEYS_MONITORING_PERIOD_MS));
        }
}
