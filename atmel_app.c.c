#include <asf.h> 
#include "platform.h" 
#include "at_ble_api.h" 
#include "console_serial.h" 
#include "timer_hw.h" 
#include "ble_manager.h" 
#include "ble_utils.h" 
#include "button.h" 
#include "startup_template_app.h" 
#include "blp_sensor.h"
#include "profiles.h"
#include "hr_sensor.h"
#include <string.h>
#include "immediate_alert.h"
#include "find_me_target.h"


/* === GLOBALS ============================================================ */

/* Timer application task flag */

bool app_timer_done = false;

/** @brief Timer interval variable for application task */
static uint8_t timer_interval = INIT_TIMER_INTERVAL;

/**
 * \brief Timer callback handler
 */
//static void timer_callback_handler(void)
//{
	///* Stop the timer */
	//hw_timer_stop();
//
	///* Enable the flag the serve the task */
	//app_timer_done = true;
//
	//send_plf_int_msg_ind(USER_TIMER_CALLBACK, TIMER_EXPIRED_CALLBACK_TYPE_DETECT, NULL, 0);
//}

/**
 * @brief Alerting function on immediate alert
 *
 * @param[in] alert level of alert level characteristic of immediate alert
 * service
 *
 */
static void app_immediate_alert(uint8_t alert_val)
{
	if (alert_val == IAS_HIGH_ALERT) {
		DBG_LOG("Find Me : High Alert");
		//LED_On(LED0);
		timer_interval = LED_FAST_INTERVAL;
		hw_timer_start(timer_interval);
	} else if (alert_val == IAS_MID_ALERT) {
		DBG_LOG("Find Me : Mild Alert");
		//LED_On(LED0);
		timer_interval = LED_MILD_INTERVAL;
		hw_timer_start(timer_interval);
	} else if (alert_val == IAS_NO_ALERT) {
		DBG_LOG("Find Me : No Alert");
		//hw_timer_stop();
		//LED_Off(LED0);
	}
}

/**
 * \brief User callback handler
 */
static void user_callback_handler(void)
{
	uint16_t plf_event_type;
	uint16_t plf_event_data_len;
	uint8_t plf_event_data[16];

	platform_event_get(&plf_event_type, plf_event_data, &plf_event_data_len);
	if (plf_event_type == ((TIMER_EXPIRED_CALLBACK_TYPE_DETECT << 8) | USER_TIMER_CALLBACK)) {
		if (app_timer_done) {
			//LED_Toggle(LED0);
			hw_timer_start(timer_interval);
			app_timer_done = false;
		}
	}
}

// HEART RATE BEGINS 
/****************************************************************************************
*							        Globals		
*                                       *
****************************************************************************************/
volatile bool app_state1 = 0 ; /*!< flag to represent the application state*/
volatile bool advertisement_flag = false;/*!< to check if the device is in advertisement*/
volatile bool notification_flag1 = false; /*!< flag to start notification*/
volatile bool disconnect_flag = false;	/*!< flag for disconnection*/
volatile bool hr_initializer_flag = 1; /*!< flag for initialization of hr for each category*/
volatile bool notification_sent1 = true;
uint8_t second_counter = 0;	/*!< second_counter to count the time*/
uint16_t energy_expended_val = ENERGY_EXP_NORMAL; /*!< to count the energy expended*/
uint16_t energy_incrementor ;	/*!< energy incrementor for various heart rate values*/
uint16_t heart_rate_value = HEART_RATE_MIN_NORM; /*!< to count the heart rate value*/
uint16_t rr_interval_value = RR_VALUE_MIN; /*!< to count the rr interval value*/
uint8_t activity = 0; /*!< activiy which will determine the */
uint8_t prev_activity = DEFAULT_ACTIVITY;/*!< previous activity */
int8_t inc_changer	= 1;/*!< increment operator to change heart rate */
int8_t time_operator ;/*!< operator to change the seconds */
uint8_t hr_min_value;/*!<the minimum heart rate value*/
uint8_t hr_max_value;/*!<the maximum heart rate value*/
uint8_t energy_inclusion = 0;/*!<To check for including the energy in hr measurement*/

static const ble_event_callback_t app_gap_handle1[] = {
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	app_connected_event_handler1,
	app_disconnected_event_handler1,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL
};

static const ble_event_callback_t app_gatt_server_handle1[] = {
	app_notification_cfm_handler,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL
};

/****************************************************************************************
*							        Functions											*
****************************************************************************************/

/** @brief notification handler function called by the profile
 *	@param[in] notification_enable which will tell the state of the
 *  application
 */
static void app_notification_handler1(uint8_t notification_enable)
{
	if (notification_enable == true) {
		DBG_LOG("Notification Enabled");
		hw_timer_start(NOTIFICATION_INTERVAL);
	} else {
		hw_timer_stop();
		notification_flag1 = false;
		DBG_LOG("Notification Disabled");
	}
}

/** @brief hr_notification_confirmation_handler called by ble manager 
 *	to give the status of notification sent
 *  @param[in] at_ble_cmd_complete_event_t address of the cmd completion
 */	
at_ble_status_t app_notification_cfm_handler(void *params)
{
	at_ble_cmd_complete_event_t event_params;
	memcpy(&event_params,params,sizeof(at_ble_cmd_complete_event_t));
	if (event_params.status == AT_BLE_SUCCESS) {
		DBG_LOG_DEV("App Notification Successfully sent over the air");
		notification_sent1 = true;
	} else {
		DBG_LOG_DEV("Sending Notification over the air failed");
		notification_sent1 = false;
	}
	return AT_BLE_SUCCESS;
}

/** @brief energy expended handler called by profile to reset the energy values
 *
 */

static void app_reset_handler(void)
{
	energy_expended_val = 0;
	DBG_LOG("Energy Expended is made '0'on user Reset");
}

/** @brief heart_rate_value_init will initializes the heart rate values
 *	 for simulation.
 *	 Based on the time different heart rate values are chosen to indicate
 *	 different activity.
 */
static void heart_rate_value_init(void)
{
	activity = second_counter / 40;

	if (activity != prev_activity) {		
		switch(activity) {
		case ACTIVITY_NORMAL:
			hr_min_value = HEART_RATE_MIN_NORM;
			hr_max_value = HEART_RATE_MAX_NORM;
			heart_rate_value = hr_min_value;
			energy_incrementor = ENERGY_EXP_NORMAL;
			break;
			
		case ACTIVITY_WALKING:
			hr_min_value = HEART_RATE_MIN_WALKING;
			hr_max_value = HEART_RATE_MAX_WALKING;
			heart_rate_value = hr_min_value;
			energy_incrementor = ENERGY_EXP_WALKING;
			break;
			
		case ACTIVITY_BRISK_WALKING:
			hr_min_value = HEART_RATE_MIN_BRISK_WALK;
			hr_max_value = HEART_RATE_MAX_BRISK_WALK;
			heart_rate_value = hr_min_value;
			energy_incrementor = ENERGY_EXP_BRISK_WALKING;
			break;
			
		case ACTIVITY_RUNNING:
			hr_min_value = HEART_RATE_MIN_RUNNING;
			hr_max_value = HEART_RATE_MAX_RUNNING;
			heart_rate_value = hr_min_value;
			energy_incrementor = ENERGY_EXP_RUNNING;
			break;
			
		case ACTIVITY_FAST_RUNNING:
			hr_min_value = HEART_RATE_MIN_FAST_RUNNING;
			hr_max_value = HEART_RATE_MAX_FAST_RUNNING;
			heart_rate_value = hr_min_value;
			energy_incrementor = ENERGY_EXP_FAST_RUNNING;
			break;
		}
		prev_activity = activity;
	}
	
	if (heart_rate_value == hr_max_value) {
		inc_changer = -1;
	} else if (heart_rate_value == hr_min_value) {
		inc_changer = 1;
	}
}

/** @brief connected state handler
 *  @param[in] status of the application
 */
static at_ble_status_t app_connected_event_handler1(void *params)
{
	app_state1 = true;
	DBG_LOG("Enable the notification in app to listen "
	"heart rate or press the button to disconnect");
	advertisement_flag = false;
	notification_sent1 = true;
        ALL_UNUSED(params);
	return AT_BLE_SUCCESS;
}

static at_ble_status_t app_disconnected_event_handler1(void *params)
{
	app_state1 = false;
	hw_timer_stop();
	notification_flag1 = false;
	energy_expended_val = ENERGY_EXP_NORMAL;
	second_counter = 0;
	activity = ACTIVITY_NORMAL;
	prev_activity = DEFAULT_ACTIVITY;
	energy_inclusion = 0;
	heart_rate_value_init();
	//DBG_LOG("Press button to advertise");
	ALL_UNUSED(params);
	return AT_BLE_SUCCESS;
}

/**
 * @brief Button Press Callback
 */
//static void button_cb(void)
//{
	//if (app_state) {
		//DBG_LOG_DEV("Going to disconnect ");
		//disconnect_flag = true;
	//} else if (app_state == false && advertisement_flag == false) {
		///* To check if the device is in advertisement */
		//DBG_LOG_DEV("Going to advertisement");
		//start_advertisement = true;
		//advertisement_flag = true;	
	//}
	//send_plf_int_msg_ind(USER_TIMER_CALLBACK, TIMER_EXPIRED_CALLBACK_TYPE_DETECT, NULL, 0);
//}

/** @brief hr_measurment_send sends the notifications after adding the hr values
 *	heart rate values starts @60bpm increments by 1 goes upto 255 bpm and
 *	restarts @60
 *  Energy Expended will be sent on every 10th notification,it starts @ 0 and
 *	increments by 20
 *  rr interval values, two rr interval values will be sent in every
 *	notification
 */
static void hr_measurment_send(void)
{
	uint8_t hr_data[HR_CHAR_VALUE_LEN];
	uint8_t idx = 0;
	
	if ((energy_expended_val == ENERGY_RESET) || (second_counter % 10 == energy_inclusion)) {
		hr_data[idx] = (RR_INTERVAL_VALUE_PRESENT | ENERGY_EXPENDED_FIELD_PRESENT);
		
		/* To send energy expended after 10 notifications after reset */
		if (energy_expended_val == ENERGY_RESET) {
			energy_inclusion = second_counter % 10 ;
		}
	} else {
		hr_data[idx] = RR_INTERVAL_VALUE_PRESENT ;
	}
	idx += 1;			
	DBG_LOG("Heart Rate: %d bpm", heart_rate_value);
	heart_rate_value += (inc_changer);

	/* Heart Rate Value 8bit*/
	hr_data[idx++] = (uint8_t)heart_rate_value ;
	if (hr_data[0] & ENERGY_EXPENDED_FIELD_PRESENT) {
		memcpy(&hr_data[idx], &energy_expended_val, 2);
		idx += 2;	
	}
	
	/* Appending RR interval values*/	
	if (rr_interval_value >= RR_VALUE_MAX) {
		rr_interval_value = (uint8_t) RR_VALUE_MIN; 
	}	
	DBG_LOG_CONT("\tRR Values:(%d,%d)msec",
				rr_interval_value, rr_interval_value + 200);
	memcpy(&hr_data[idx], &rr_interval_value, 2);
	idx += 2;
	rr_interval_value += 200;
	memcpy(&hr_data[idx], &rr_interval_value, 2);
	idx += 2;
	rr_interval_value += 200;
	
	/*printing the user activity,simulation*/
	switch(activity) {
	case ACTIVITY_NORMAL:
		DBG_LOG_CONT(" User Status:Idle");
		break;
		
	case ACTIVITY_WALKING:
		DBG_LOG_CONT(" User Status:Walking");
		break;
		
	case ACTIVITY_BRISK_WALKING:
		DBG_LOG_CONT(" User status:Brisk walking");
		break;
		
	case ACTIVITY_RUNNING:
		DBG_LOG_CONT(" User status:Running");
		break;
		
	case ACTIVITY_FAST_RUNNING:
		DBG_LOG_CONT(" User Status:Fast Running");
		break;	
	}
	
	/* Printing the energy*/
	if ((hr_data[0] & ENERGY_EXPENDED_FIELD_PRESENT)) {
		DBG_LOG("Energy Expended :%d KJ\n", energy_expended_val);
		energy_expended_val += energy_incrementor;
	}
	
	/* Sending the data for notifications*/
	hr_sensor_send_notification(hr_data, idx);
}

/**
 * \brief Timer callback handler called on timer expiry
 */
//static void timer_callback_handler(void)
//{
	//if (second_counter == START_OF_FIRST_ACTIVITY) {
		//time_operator = 1;
	//} else if (second_counter == END_OF_LAST_ACTIVITY) {
		//time_operator = -1;
	//}
	//second_counter += (time_operator);
	//heart_rate_value_init();
	//notification_flag1 = true;
	//
	//send_plf_int_msg_ind(USER_TIMER_CALLBACK, TIMER_EXPIRED_CALLBACK_TYPE_DETECT, NULL, 0);
//}
//HEART RATE CODE ENDS
//! [module_inst]
struct uart_module uart_instance;
//! [module_inst]

void configure_gpio_pins(void);

//! [dma_resource]
struct dma_resource uart_dma_resource_tx;
struct dma_resource uart_dma_resource_rx;
//! [dma_resource]

//! [usart_buffer]
#define BUFFER_LEN    2
static int8_t string[BUFFER_LEN];
float int_value;
float _temp_value;
float float_value;
//! [usart_buffer]


//! [transfer_descriptor]
struct dma_descriptor example_descriptor_tx;
struct dma_descriptor example_descriptor_rx;
//! [transfer_descriptor]
volatile at_ble_status_t status; 
at_ble_handle_t htpt_conn_handle; 
volatile bool Timer_Flag = false; 
volatile bool Temp_Notification_Flag = false; 


#define APP_STACK_SIZE  (1024)
volatile unsigned char app_stack_patch[APP_STACK_SIZE];

/** flag to check if indication has been sent successfully over the air*/
/*volatile*/ bool indication_sent = true;

/** flag to check if notification has been sent successfully over the air*/
/*volatile*/ bool notification_sent = true;

/** Flag to change the events from mmgh to kpa and vice versa*/
/*volatile*/ bool units = APP_DEFAULT_VAL;

/** flag to send notifications */
/*volatile*/ bool notification_flag = APP_DEFAULT_VAL;

/** flag to send indication */
/*volatile*/ bool indication_flag = APP_DEFAULT_VAL;

/** Flag to identify user request for indication and notification*/
/*volatile*/ bool user_request_flag =  APP_DEFAULT_VAL;

/** Counter to maintain interval of indication*/
/*volatile*/ uint8_t timer_count = APP_DEFAULT_VAL;

/** flag to send one notification for one second*/
/*volatile*/ bool notify = 0;

/** flag to check the app state*/
/*volatile*/ bool app_state;

/** flags for reversing the direction of characteristic*
 *       change for indication*/
/*volatile*/ int8_t operator_blp[9] = {1, 1, 1, 1, 1, 1, 1, 1, 1};

/** Current systolic value  in mmhg*/
uint16_t systolic_val_mmhg = SYSTOLIC_MIN_MMHG;

/** Current diastolic value in mmhg*/
uint16_t diastolic_val_mmhg = DIASTOLIC_MIN_MMHG;

/** Current map value in mmhg*/
uint16_t map_val_mmhg = MAP_MIN_MMHG;

/** Current systolic in kpa*/
uint16_t systolic_val_kpa = SYSTOLIC_MIN_KPA;

/** current diastolic value in kpa*/
uint16_t diastolic_val_kpa = DIASTOLIC_MIN_KPA;

/** current map value in kpa*/
uint16_t map_val_kpa = MAP_MIN_KPA;

/** Current pulse rate value in kpa*/
uint16_t pulse_rate_val = PULSE_RATE_MIN;

/** Current time stamp */
at_ble_prf_date_time_t time_stamp;

/* Intermediate Cuff Pressure Values for notification */
uint16_t interim_diastolic_mmhg = DIASTOLIC_MIN_MMHG;

uint16_t interim_diastolic_kpa = DIASTOLIC_MIN_KPA; 

uint16_t interim_systolic_mmhg = SYSTOLIC_MIN_MMHG;

uint16_t interim_systolic_kpa = SYSTOLIC_MIN_KPA;

uint16_t interim_map_mmhg = MAP_MIN_MMHG;

uint16_t interim_map_kpa = MAP_MIN_KPA;

static /*const*/ ble_event_callback_t app_gap_handle[] = {
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	app_connected_state_handler,
	app_disconnected_state_handler,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL
};

static /*const*/ ble_event_callback_t app_gatt_server_handle[] = {
	app_notification_confirmation_handler,
	app_indication_confirmation_handler,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL
};

/* To keep the app in execution mode */
bool app_exec = true;

bool isButton = false;
bool isTimer = false;
bool isIndication = false;
uint8_t g_blp_data[BLP_DATA_LEN];
uint8_t g_idx = 0;

/****************************************************************************************
*							        Functions											*
****************************************************************************************/
/**
 * @brief app_connected_state profile notifies the application about state
 * @param[in] connected
 */
static at_ble_status_t app_connected_state_handler(void *params)
{
	app_state = true;
	return AT_BLE_SUCCESS;
}

static at_ble_status_t app_disconnected_state_handler(void *param)
{
	app_state = false;
	//Resetting all the simulated values
		
		interim_diastolic_mmhg = DIASTOLIC_MIN_MMHG;
		interim_diastolic_kpa = DIASTOLIC_MIN_KPA;
		interim_systolic_mmhg = SYSTOLIC_MIN_MMHG;
		interim_systolic_kpa = SYSTOLIC_MIN_KPA;
		interim_map_mmhg = MAP_MIN_MMHG;
		interim_map_kpa = MAP_MIN_KPA;
		systolic_val_mmhg = SYSTOLIC_MIN_MMHG;
		diastolic_val_mmhg = DIASTOLIC_MIN_MMHG;
		map_val_mmhg = MAP_MIN_MMHG;
		systolic_val_kpa = SYSTOLIC_MIN_KPA;
		diastolic_val_kpa = DIASTOLIC_MIN_KPA;
		map_val_kpa = MAP_MIN_KPA;
		pulse_rate_val = PULSE_RATE_MIN;
		units = !units;
		indication_sent = true;
		notification_sent = true;
		notify = false;
		timer_count = APP_DEFAULT_VAL;
		user_request_flag =  APP_DEFAULT_VAL;
		indication_flag = APP_DEFAULT_VAL;
		notification_flag = APP_DEFAULT_VAL;
		
		/* Starting advertisement */
		blp_sensor_adv();
		//ALL_UNUSED(param);
		return AT_BLE_SUCCESS;
}

/** @brief Updating the time stamp
 *
 */
static void update_time_stamp(void)
{
	if (time_stamp.sec < SECOND_MAX)
	{
		time_stamp.sec++;
	}
	else
	{
		time_stamp.sec = 0;	
		if (time_stamp.min < MINUTE_MAX)
		{
			time_stamp.min++;
		}
		else
		{
			time_stamp.min = 0;
			if (time_stamp.hour < HOUR_MAX)
			{
				time_stamp.hour++;
			}
			else
			{
				time_stamp.hour = 0;
				if (time_stamp.day < DAY_MAX)
				{
					time_stamp.day++;
				}
				else
				{
					time_stamp.day = 1;
					if (time_stamp.month < MONTH_MAX)
					{
						time_stamp.month++;
					}
					else
					{
						time_stamp.month = 1;
						if (time_stamp.year < YEAR_MAX)
						{
							time_stamp.year++;
						} 
						else
						{
							time_stamp.year = 2015;
						}
					}
				}
			}
		}			
	}	
}

/** @brief initializes the time stamp with default time stamp
 *
 */
static void time_stamp_init(void)
{
	memset((uint8_t *)&time_stamp, 0, sizeof(at_ble_prf_date_time_t));
	time_stamp.year = 2015;
	time_stamp.day = 1;
	time_stamp.month = 1;
}

/** @brief app_notification_confirmation_handler called by ble manager 
 *	to give the status of notification sent
 *  @param[in] at_ble_cmd_complete_event_t address of the cmd completion
 */	
static at_ble_status_t app_notification_confirmation_handler(void *params)
{

	if (((at_ble_cmd_complete_event_t *)params)->status == AT_BLE_SUCCESS) {
		DBG_LOG_DEV("App Notification Successfully sent over the air");
		notification_sent = true;
	} else {
		DBG_LOG_DEV("Sending Notification over the air failed");
		notification_sent = false;
	}
	return AT_BLE_SUCCESS;
}

/** @brief app_indication_confirmation_handler called by ble manager 
 *	to give the status of notification sent
 *  @param[in] at_ble_cmd_complete_event_t address of the cmd completion
 */	
static at_ble_status_t app_indication_confirmation_handler(void *params)
{
	if (((at_ble_cmd_complete_event_t * )params)->status == AT_BLE_SUCCESS) {
		DBG_LOG_DEV("App Indication successfully sent over the air");
		indication_sent = true;
		user_request_flag = false;
		DBG_LOG("\r\nPress the button to receive the blood pressure parameters");
	} else {
		DBG_LOG_DEV("Sending indication over the air failed reason %x ",
								((at_ble_cmd_complete_event_t * )params)->status);
		indication_sent = false;
	}
	return AT_BLE_SUCCESS;
}

/** @brief blp_value_update which will change the blood pressure measurement operations
 *  @param[in] data to which the blood pressure parameter value to be appended
 *  @param[in] idx index where the value has to be updated
 *  @param[in] value_type which will determine which blood pressure parameter 
 */
static void blp_value_update(uint8_t *data, uint8_t idx, uint16_t value, uint8_t value_type)
{
	uint8_t min_val = 0, max_val = 0;
	
	switch(value_type) {
	case SYSTOLIC_MMHG:	
		min_val = SYSTOLIC_MIN_MMHG;
		max_val = SYSTOLIC_MAX_MMHG;
	break;
	
	case DIASTOLIC_MMHG:
		min_val = DIASTOLIC_MIN_MMHG;
		max_val = DIASTOLIC_MAX_MMHG;
	break;
	
	case MAP_MMHG:
		min_val = MAP_MIN_MMHG;
		max_val = MAP_MAX_MMHG;
	break;
	
	case PULSE_RATE:
		min_val = PULSE_RATE_MIN;
		max_val = PULSE_RATE_MAX;
	break;
	
	case SYSTOLIC_KPA:
		min_val = SYSTOLIC_MIN_KPA;
		max_val = SYSTOLIC_MAX_KPA;
	break;
	
	case DIASTOLIC_KPA:
		min_val = DIASTOLIC_MIN_KPA;
		max_val = DIASTOLIC_MAX_KPA;
	break;
	
	case MAP_KPA:
		min_val = MAP_MIN_KPA;
		max_val = MAP_MAX_KPA;
	break;
	
	case INTERIM_SYS_MMHG:
		min_val = SYSTOLIC_MIN_MMHG;
		max_val = SYSTOLIC_MAX_MMHG;
	break;
	
	case INTERIM_SYS_KPA:
		min_val = SYSTOLIC_MIN_KPA;
		max_val = SYSTOLIC_MAX_KPA;
	break;
	}
	
	if (value == max_val) {
		operator_blp[value_type] = -1;
	} else if (value == min_val) {
		operator_blp[value_type] = 1;
	}
	memcpy((data + idx),&value,2);
}
/** @brief sends the characteristic data for the profile to send indication
 *
 */
static void blp_char_indication(void)
{
	uint8_t blp_data[BLP_DATA_LEN];
	uint8_t idx = 0;

	memset(blp_data, 0, sizeof(uint8_t) * BLP_DATA_LEN);
	idx = 0;

	DBG_LOG("\n\n");
	
	DBG_LOG("The Blood Pressure Values are:");
	/* initializing flags to zero*/
	blp_data[0] = 0;
	/** Blood pressure measurement flags */
	if (units) {
		/** Units in mmhg*/
			blp_data[idx] |= (0x1)  & ~(BLOOD_PRESSURE_UNITS_FLAG_MASK);
	} else {
		/** Units in kpa*/
		    blp_data[idx] |= (0x1)  | BLOOD_PRESSURE_UNITS_FLAG_MASK ;
	} 
	
	/** Appending the flags for time stamp , pulse rate, user id , mm status */
	blp_data[idx]	|= BLOOD_PRESSURE_TIME_FLAG_MASK;
	blp_data[idx]	|= BLOOD_PRESSURE_PULSE_FLAG_MASK;
	blp_data[idx]	|= BLOOD_PRESSURE_USERID_FLAG_MASK;
	blp_data[idx++] |= BLOOD_PRESSURE_MMT_STATUS_FLAG_MASK;
	
	if (units) {
		systolic_val_mmhg = systolic_val_mmhg + (operator_blp[SYSTOLIC_MMHG]);
		blp_value_update(blp_data, idx, systolic_val_mmhg, SYSTOLIC_MMHG);
		idx += 2;
		DBG_LOG("%-12s", "Systolic");
		DBG_LOG_CONT("   %d mmhg", systolic_val_mmhg);

		diastolic_val_mmhg = diastolic_val_mmhg + (operator_blp[DIASTOLIC_MMHG]);
		blp_value_update(blp_data, idx, diastolic_val_mmhg, DIASTOLIC_MMHG);
		idx += 2;
		DBG_LOG("%-12s", "Diastolic");
		DBG_LOG_CONT("   %d mmhg", diastolic_val_mmhg);

		map_val_mmhg = map_val_mmhg + (operator_blp[MAP_MMHG]);
		blp_value_update(blp_data, idx, map_val_mmhg, MAP_MMHG);
		idx += 2;
		DBG_LOG("%-12s", "Map");
		DBG_LOG_CONT("   %d mmhg", map_val_mmhg);
	} else {
		systolic_val_kpa = systolic_val_kpa + (operator_blp[SYSTOLIC_KPA]);
		blp_value_update(blp_data, idx, systolic_val_kpa, SYSTOLIC_KPA);
		idx += 2;
		DBG_LOG("%-12s", "Systolic");
		DBG_LOG_CONT("   %02d kpa", systolic_val_kpa);
		diastolic_val_kpa = diastolic_val_kpa + (operator_blp[DIASTOLIC_KPA]);
		blp_value_update(blp_data, idx, diastolic_val_kpa, DIASTOLIC_KPA);
		idx += 2;
		DBG_LOG("%-12s", "Diastolic");
		DBG_LOG_CONT("   %02d kpa", diastolic_val_kpa);
		map_val_kpa = map_val_kpa + (operator_blp[MAP_KPA]);
		blp_value_update(blp_data, idx, map_val_kpa, MAP_KPA);
		idx += 2;
		DBG_LOG("%-12s", "Map");
		DBG_LOG_CONT("   %02d kpa", map_val_kpa);
	}
		memcpy(&blp_data[idx],(uint8_t*)&(time_stamp.year),2);
		idx += 2;
		memcpy(&blp_data[idx],(uint8_t*)&(time_stamp.month),1);
		idx += 1;
		memcpy(&blp_data[idx],(uint8_t*)&(time_stamp.day),1);
		idx += 1;
		memcpy(&blp_data[idx],(uint8_t*)&(time_stamp.hour),1);
		idx += 1;
		memcpy(&blp_data[idx],(uint8_t*)&(time_stamp.min),1);
		idx += 1;
		memcpy(&blp_data[idx],(uint8_t*)&(time_stamp.sec),1);
		idx += 1;
		
	pulse_rate_val = pulse_rate_val + (operator_blp[PULSE_RATE]);
	blp_value_update(blp_data, idx, pulse_rate_val, PULSE_RATE);
	idx += 2;
	DBG_LOG("%-12s", "Pulserate");
	DBG_LOG_CONT("   %d bpm", pulse_rate_val);

	/** Appending User id */
	if (units) {
		blp_data[idx++] = USERID_1;
	} else {
		blp_data[idx++] = USERID_2;
	}
	
	/** Appending Measurement status field */
	blp_data[idx++] = 0xf;
	blp_data[idx++] = 0x0;
	
	blp_sensor_send_indication(blp_data,idx);	
	
}

/** @brief sends the characteristic data for profile to send notification
 *
 */
static void blp_char_notification(void)
{
	uint8_t blp_data[BLP_DATA_LEN];	
	uint8_t idx = 0;

	memset(blp_data, 0, sizeof(uint8_t) * BLP_DATA_LEN);
	idx = 0;

	if (units) {
		/** Units in mmhg*/
		blp_data[idx++] |= (0x1)  & ~(BLOOD_PRESSURE_UNITS_FLAG_MASK);
		blp_data[0] = blp_data[0] & 1;
		DBG_LOG("Cuff pressure  %d mmhg", interim_systolic_mmhg);
		blp_value_update(blp_data, idx, interim_systolic_mmhg, INTERIM_SYS_MMHG);
		idx += 2;
		interim_systolic_mmhg = interim_systolic_mmhg + (operator_blp[7]);
	} else {
		/** Units in kpa*/
		blp_data[idx++] |=  (0x1)  | BLOOD_PRESSURE_UNITS_FLAG_MASK;
		blp_data[0] = blp_data[0] & 1;
		DBG_LOG("Cuff pressure  %02d kpa", interim_systolic_kpa);
		blp_value_update(blp_data, idx, interim_systolic_kpa, INTERIM_SYS_KPA);
		idx += 2;
		interim_systolic_kpa = interim_systolic_kpa + (operator_blp[8]);
	}

	/** Appending diastolic in kpa*/
	blp_data[idx++] = 0;
	blp_data[idx++] = 0;
	
	/** Appending map in kpa */
	blp_data[idx++] = 0;
	blp_data[idx++] = 0;
	
	blp_data[0]	|= BLOOD_PRESSURE_USERID_FLAG_MASK;
	
	/** Appending User id */
	if (units) {
		blp_data[idx++] = USERID_1;
		} else {
		blp_data[idx++] = USERID_2;
	}
	
	blp_sensor_send_notification(blp_data,idx);
}
/** @brief notification handler function called by the profile
 *	@param[in] enable will give weather notification has to enabled
 *  or disabled.
 */
static void app_notification_handler(bool enable)
{
	notification_flag = enable;
	
	if (notification_flag) {
		DBG_LOG("Notifications enabled by the remote device for interim cuff pressure");
	} else{
		DBG_LOG("Disabled notifications by the remote device for interim cuff pressure");
		timer_count = INDICATION_TIMER_VAL;
	}
}

/** @brief indication handler function called by the profile
 *	@param[in] enable will give weather indication has to enabled
 *  or disabled.
 */
static void app_indication_handler(bool enable)
{
	uint8_t blp_data[BLP_DATA_LEN];
	uint8_t idx = 0;	 

	memset(blp_data, 0, sizeof(uint8_t) * BLP_DATA_LEN);
	idx = 0;

	indication_flag = enable;

	if (indication_flag) {
		DBG_LOG("Indications enabled by the remote device for blood pressure\n ");
		if (units) {
			blp_data[idx++] = 0;
			DBG_LOG("Systolic       %02d mmhg",systolic_val_mmhg);
			memcpy(&blp_data[idx],&systolic_val_mmhg,2);
			idx += 2;
			DBG_LOG("Diastolic      %02d mmhg",diastolic_val_mmhg);
			memcpy(&blp_data[idx],&diastolic_val_mmhg,2);
			idx += 2;
			DBG_LOG("Map            %02d mmhg",map_val_mmhg);
			memcpy(&blp_data[idx],&map_val_mmhg,2);
			idx += 2;
		} else {
			blp_data[idx++] = 0x1;
			memcpy(&blp_data[idx],&systolic_val_kpa,2);
			idx += 2;
			DBG_LOG("Systolic       %02d kpa",systolic_val_kpa);
			memcpy(&blp_data[idx],&diastolic_val_kpa,2);
			idx += 2;
			DBG_LOG("Diastolic      %02d kpa",diastolic_val_kpa);
			memcpy(&blp_data[idx],&map_val_kpa,2);
			idx += 2;
			DBG_LOG("Map            %02d kpa",map_val_kpa);
		}
		
		blp_data[0]	|= BLOOD_PRESSURE_PULSE_FLAG_MASK;
			DBG_LOG("Pulse rate     %d bpm",pulse_rate_val);
		memcpy(&blp_data[idx],&pulse_rate_val,2);
		idx += 2;
		/* DBG_LOG("Flags are %d and length is %d",blp_data[0],idx); */

		isIndication = true;

		memcpy(g_blp_data, blp_data, sizeof(uint8_t) * BLP_DATA_LEN);
		g_idx = idx;
		send_plf_int_msg_ind(USER_TIMER_CALLBACK, TIMER_EXPIRED_CALLBACK_TYPE_DETECT, NULL, 0);

		/* Sending the default notification for the first time */
		/* blp_sensor_send_indication(blp_data,idx); */
	} else {
		DBG_LOG("Disabled indication by the remote server for blood pressure");
	}
}


/**
 * @brief Button Press Callback
 */
static void button_cb(void)
{
	isButton = true;
	send_plf_int_msg_ind(USER_TIMER_CALLBACK, TIMER_EXPIRED_CALLBACK_TYPE_DETECT, NULL, 0);
}

/**
 * \brief Timer callback handler called on timer expiry

static void timer_callback_handler(void)
{
	hw_timer_stop();

	isTimer = true;
	send_plf_int_msg_ind(USER_TIMER_CALLBACK, TIMER_EXPIRED_CALLBACK_TYPE_DETECT, NULL, 0);
}*/


//#define HTPT_FAHRENHEIT 
static at_ble_status_t app_htpt_cfg_indntf_ind_handler(void *params) 
{ 
at_ble_htpt_cfg_indntf_ind_t htpt_cfg_indntf_ind_params; 
memcpy((uint8_t *)&htpt_cfg_indntf_ind_params, params, sizeof(at_ble_htpt_cfg_indntf_ind_t)); 
if (htpt_cfg_indntf_ind_params.ntf_ind_cfg == 0x3) { 
printf("Started HTP Temperature Notification"); 
Temp_Notification_Flag = true; 
} 
else { 
printf("HTP Temperature Notification Stopped"); 
Temp_Notification_Flag = false; 
} 
return AT_BLE_SUCCESS; 
} 
static const ble_event_callback_t app_htpt_handle[] = { 
NULL, 
NULL, 
NULL, 
NULL, 
NULL, 
app_htpt_cfg_indntf_ind_handler, 
NULL, 
NULL, 
NULL 
}; 
/* Timer callback */ 
static void timer_callback_handler(void) 
{ 
/* Stop timer */ 
hw_timer_stop(); 
/* Set timer Alarm flag */ 
Timer_Flag = true; 
isTimer = true;
app_timer_done = true;
if (second_counter == START_OF_FIRST_ACTIVITY) {
	time_operator = 1;
	} else if (second_counter == END_OF_LAST_ACTIVITY) {
	time_operator = -1;
}
second_counter += (time_operator);
heart_rate_value_init();
notification_flag1 = true;
send_plf_int_msg_ind(USER_TIMER_CALLBACK, TIMER_EXPIRED_CALLBACK_TYPE_DETECT, NULL, 0);
/* Restart Timer */ 
hw_timer_start(10); 
} 
/* Sending the temperature value after reading it from IO1 Xplained Pro */ 
static void htp_temperature_send(void) 
{ 
at_ble_prf_date_time_t timestamp; 
float temperature; 
/* Read Temperature Value from IO1 Xplained Pro */ 
temperature =    (float)_temp_value;//(char)string[0]+ (char)string[1]+ (char)string[2] +(char)string[3];//uart_instance.hw->RECEIVE_DATA.reg;//at30tse_read_temperature(); 
#ifdef HTPT_FAHRENHEIT 
temperature = (((temperature * 9.0)/5.0) + 32.0); 
#endif 
/* Read Temperature Value from IO1 Xplained Pro */ 
timestamp.day = 1; 
timestamp.hour = 9; 
timestamp.min = 2; 
timestamp.month = 8; 
timestamp.sec = 36; 
timestamp.year = 15; 
/* Read Temperature Value from IO1 Xplained Pro */ 
if(at_ble_htpt_temp_send(convert_ieee754_ieee11073_float((float)temperature), &timestamp, 
#ifdef HTPT_FAHRENHEIT 
(at_ble_htpt_temp_flags)(HTPT_FLAG_FAHRENHEIT | HTPT_FLAG_TYPE), 
#else 
(at_ble_htpt_temp_flags)(HTPT_FLAG_CELSIUS | HTPT_FLAG_TYPE), 
#endif 
HTP_TYPE_ARMPIT, 
1 
) == AT_BLE_SUCCESS) 
{ 
#ifdef HTPT_FAHRENHEIT 
printf("Temperature: %d Fahrenheit", (uint16_t)temperature); 
#else 
printf("Temperature: %d Deg Celsius", (uint16_t)temperature); 
#endif 
} 
} 
static void htp_init (void) 
{ 
printf("\nAssignment 4.1: Init Health temperature service "); 
/* Create htp service in GATT database*/ 
status = at_ble_htpt_create_db( 
HTPT_TEMP_TYPE_CHAR_SUP, 
HTP_TYPE_ARMPIT, 
1, 
30, 
1, 
HTPT_AUTH, 
&htpt_conn_handle); 
if (status != AT_BLE_SUCCESS){ 
printf("HTP Data Base creation failed"); 
while(1); 
} 
} 
static void ble_advertise (void) 
{ 
printf("\nAssignment 2.1 : Start Advertising"); 
status = ble_advertisement_data_set(); 

if(status != AT_BLE_SUCCESS) 
{ 
printf("\n\r## Advertisement data set failed : error %x",status); 
while(1); 
} 
/* Start of advertisement */ 
status = at_ble_adv_start(AT_BLE_ADV_TYPE_UNDIRECTED, 
AT_BLE_ADV_GEN_DISCOVERABLE, 
NULL, 
AT_BLE_ADV_FP_ANY, 
1000, 
655, 
0); 
if(status != AT_BLE_SUCCESS) 
{ 
printf("\n\r## Advertisement data set failed : error %x",status); 
while(1); 
} 
} 


//! [setup]
void configure_gpio_pins(void)
{
	//! [setup_1]
	struct gpio_config config_gpio_pin;
	//! [setup_1]
	//! [setup_2]
	gpio_get_config_defaults(&config_gpio_pin);
	//! [setup_2]

	//! [setup_3]
	config_gpio_pin.direction  = GPIO_PIN_DIR_INPUT;
	config_gpio_pin.input_pull = GPIO_PIN_PULL_UP;
	//! [setup_3]
	//! [setup_4]
	//gpio_pin_set_config(BUTTON_0_PIN, &config_gpio_pin);
	//! [setup_4]

	//! [setup_5]
	config_gpio_pin.direction = GPIO_PIN_DIR_OUTPUT;
	//! [setup_5]
	//! [setup_6]
	gpio_pin_set_config(LED_0_PIN, &config_gpio_pin);
	//! [setup_6]
}
//! [setup]
static void htp_temperature_read(void) 
{ 
float temperature; 
/* Read Temperature Value from IO1 Xplained Pro */ 
temperature =   (float)(_temp_value);//at30tse_read_temperature(); 
/* Display temperature on com port */ 
#ifdef HTPT_FAHRENHEIT 
printf("Temperature: %d Fahrenheit", (uint16_t)temperature); 
#else 
printf("Temperature: %d Deg Celsius", (uint16_t)temperature); 
#endif 
} 
/* Callback registered for AT_BLE_CONNECTED event*/ 
static at_ble_status_t ble_paired_cb (void *param) 
{ 
at_ble_pair_done_t *pair_params = param; 
printf("\nAssignment 3.2: Application paired "); 
/* Enable the HTP Profile */ 
printf("\nAssignment 4.1: enable health temperature service "); 
status = at_ble_htpt_enable(pair_params->handle, 
HTPT_CFG_INTERM_MEAS_NTF); 
if(status != AT_BLE_SUCCESS){ 
printf("*** Failure in HTP Profile Enable"); 
while(1); 
} 
ALL_UNUSED(param); 
return AT_BLE_SUCCESS; 
} 
/* Callback registered for AT_BLE_DISCONNECTED event */ 
static at_ble_status_t ble_disconnected_cb (void *param) 
{ 
printf("\nAssignment 3.2: Application disconnected "); 
ble_advertise(); 
ALL_UNUSED(param);return AT_BLE_SUCCESS; 
} 
static const ble_event_callback_t app_gap_cb[] = { 

NULL, // AT_BLE_UNDEFINED_EVENT 
NULL, // AT_BLE_SCAN_INFO 
NULL, // AT_BLE_SCAN_REPORT 
NULL, // AT_BLE_ADV_REPORT 
NULL, // AT_BLE_RAND_ADDR_CHANGED 
NULL, // AT_BLE_CONNECTED 
ble_disconnected_cb, // AT_BLE_DISCONNECTED 
NULL, // AT_BLE_CONN_PARAM_UPDATE_DONE 
NULL, // AT_BLE_CONN_PARAM_UPDATE_REQUEST 
ble_paired_cb, // AT_BLE_PAIR_DONE 
NULL, // AT_BLE_PAIR_REQUEST 
NULL, // AT_BLE_SLAVE_SEC_REQUEST 
NULL, // AT_BLE_PAIR_KEY_REQUEST 
NULL, // AT_BLE_ENCRYPTION_REQUEST 
NULL, // AT_BLE_ENCRYPTION_STATUS_CHANGED 
NULL, // AT_BLE_RESOLV_RAND_ADDR_STATUS 
NULL, // AT_BLE_SIGN_COUNTERS_IND 
NULL, // AT_BLE_PEER_ATT_INFO_IND 
NULL // AT_BLE_CON_CHANNEL_MAP_IND 
}; 

/* Register GAP callbacks at BLE manager level*/ 
static void register_ble_callbacks (void) 
{ 
/* Register GAP Callbacks */ 
printf("\nAssignment 3.2: Register bluetooth events callbacks"); 
status = ble_mgr_events_callback_handler(REGISTER_CALL_BACK,BLE_GAP_EVENT_TYPE,app_gap_cb); 
if (status != true) { 
printf("\n##Error when Registering SAMB11 gap callbacks"); 
} 
status = ble_mgr_events_callback_handler(REGISTER_CALL_BACK,BLE_GATT_HTPT_EVENT_TYPE,app_htpt_handle); 
if (status != true) { 
printf("\n##Error when Registering SAMB11 htpt callbacks"); 
} 
} 


//! [setup]
//! [transfer_done_tx]
static void transfer_done_tx(struct dma_resource* const resource )
{
		printf("\nhey you came in call back tx\n");
	dma_start_transfer_job(&uart_dma_resource_rx);
}
//! [transfer_done_tx]

//! [transfer_done_rx]
static void transfer_done_rx(struct dma_resource* const resource )
{
	dma_start_transfer_job(&uart_dma_resource_rx);

int_value = string[0];
float_value = string[1];

	if((int)int_value == 79 && (int)float_value == 78 )
	{
		gpio_pin_set_output_level(LED_0_PIN, false);
	}
	else if ((int)int_value == 79 && (int)float_value == 70)
	{
	gpio_pin_set_output_level(LED_0_PIN, true);
	}
	else
	{		
		int_value = string[0];
		float_value = (float)(string[1]*(0.1));
		_temp_value = (float)((int_value)+(float_value));
	}
}
//! [transfer_done_rx]

//! [config_dma_resource_tx]
static void configure_dma_resource_tx(struct dma_resource *resource)
{
//! [setup_tx_1]
	struct dma_resource_config config;
//! [setup_tx_1]

//! [setup_tx_2]
	dma_get_config_defaults(&config);
//! [setup_tx_2]

//! [setup_tx_3]
	config.des.periph = UART0TX_DMA_PERIPHERAL;
	config.des.enable_inc_addr = false;
	config.src.periph = UART0TX_DMA_PERIPHERAL;
//! [setup_tx_3]

//! [setup_tx_4]
	dma_allocate(resource, &config);
//! [setup_tx_4]
}
//! [config_dma_resource_tx]

//! [setup_dma_transfer_tx_descriptor]
static void setup_transfer_descriptor_tx(struct dma_descriptor *descriptor)
{

//! [setup_tx_5]
	dma_descriptor_get_config_defaults(descriptor);
//! [setup_tx_5]

//! [setup_tx_6]
	descriptor->buffer_size = BUFFER_LEN;
	descriptor->read_start_addr = (uint32_t)string;
	descriptor->write_start_addr = 
			(uint32_t)(&uart_instance.hw->TRANSMIT_DATA.reg);
//! [setup_tx_6]
}
//! [setup_dma_transfer_tx_descriptor]

//! [config_dma_resource_rx]
static void configure_dma_resource_rx(struct dma_resource *resource)
{
	//! [setup_rx_1]
	struct dma_resource_config config;
	//! [setup_rx_1]

	//! [setup_rx_2]
	dma_get_config_defaults(&config);
	//! [setup_rx_2]

	//! [setup_rx_3]
	config.src.periph = UART0RX_DMA_PERIPHERAL;
	config.src.enable_inc_addr = false;
	config.src.periph_delay = 1;
	//! [setup_rx_3]

	//! [setup_rx_4]
	dma_allocate(resource, &config);
	//! [setup_rx_4]
}
//! [config_dma_resource_rx]

//! [setup_dma_transfer_rx_descriptor]
static void setup_transfer_descriptor_rx(struct dma_descriptor *descriptor)
{
	//! [setup_rx_5]
	dma_descriptor_get_config_defaults(descriptor);
	//! [setup_rx_5]

	//! [setup_tx_6]
	descriptor->buffer_size = BUFFER_LEN;
	descriptor->read_start_addr =
			(uint32_t)(&uart_instance.hw->RECEIVE_DATA.reg);
	descriptor->write_start_addr = (uint32_t)string;
	//! [setup_tx_6]
}
//! [setup_dma_transfer_rx_descriptor]

//! [setup_usart]
static void configure_usart(void)
{
//! [setup_config]
	struct uart_config config_uart;
//! [setup_config]

//! [setup_config_defaults]
	uart_get_config_defaults(&config_uart);
//! [setup_config_defaults]

//! [setup_change_config]
	config_uart.baud_rate = 9600;
	config_uart.pin_number_pad[0] = EDBG_CDC_SERCOM_PIN_PAD0;
	config_uart.pin_number_pad[1] = EDBG_CDC_SERCOM_PIN_PAD1;
	config_uart.pin_number_pad[2] = EDBG_CDC_SERCOM_PIN_PAD2;
	config_uart.pin_number_pad[3] = EDBG_CDC_SERCOM_PIN_PAD3;
	config_uart.pinmux_sel_pad[0] = EDBG_CDC_SERCOM_MUX_PAD0;
	config_uart.pinmux_sel_pad[1] = EDBG_CDC_SERCOM_MUX_PAD1;
	config_uart.pinmux_sel_pad[2] = EDBG_CDC_SERCOM_MUX_PAD2;
	config_uart.pinmux_sel_pad[3] = EDBG_CDC_SERCOM_MUX_PAD3;
//! [setup_change_config]

//! [setup_set_config]
	while (uart_init(&uart_instance,
			EDBG_CDC_MODULE, &config_uart) != STATUS_OK) {
	}
//! [setup_set_config]

//! [enable_interrupt]
	uart_enable_transmit_dma(&uart_instance);
	uart_enable_receive_dma(&uart_instance);
//! [enable_interrupt]
}
//! [setup_usart]

//! [setup_callback]
static void configure_dma_callback(void)
{
//! [setup_callback_register]
	dma_register_callback(&uart_dma_resource_tx, transfer_done_tx, DMA_CALLBACK_TRANSFER_DONE);
	dma_register_callback(&uart_dma_resource_rx, transfer_done_rx, DMA_CALLBACK_TRANSFER_DONE);
//! [setup_callback_register]

//! [setup_enable_callback]
	dma_enable_callback(&uart_dma_resource_tx, DMA_CALLBACK_TRANSFER_DONE);
	dma_enable_callback(&uart_dma_resource_rx, DMA_CALLBACK_TRANSFER_DONE);
//! [setup_enable_callback]

//! [enable_inic]
	NVIC_EnableIRQ(PROV_DMA_CTRL0_IRQn);
//! [enable_inic]
}
//! [setup_callback]

//! [setup]


int main (void) 
{ 
	indication_sent = true;
	notification_sent = true;
	units = APP_DEFAULT_VAL;
	notification_flag = APP_DEFAULT_VAL;
	indication_flag = APP_DEFAULT_VAL;
	user_request_flag =  APP_DEFAULT_VAL;
	timer_count = APP_DEFAULT_VAL;
	notify = 0;
	app_state = 0;
	memset(operator_blp, 1, sizeof(int8_t) * 9);
	systolic_val_mmhg = SYSTOLIC_MIN_MMHG;
	diastolic_val_mmhg = DIASTOLIC_MIN_MMHG;
	map_val_mmhg = MAP_MIN_MMHG;
	systolic_val_kpa = SYSTOLIC_MIN_KPA;
	diastolic_val_kpa = DIASTOLIC_MIN_KPA;
	map_val_kpa = MAP_MIN_KPA;
	pulse_rate_val = PULSE_RATE_MIN;
	interim_diastolic_mmhg = DIASTOLIC_MIN_MMHG;
	interim_diastolic_kpa = DIASTOLIC_MIN_KPA;
	interim_systolic_mmhg = SYSTOLIC_MIN_MMHG;
	interim_systolic_kpa = SYSTOLIC_MIN_KPA;
	interim_map_mmhg = MAP_MIN_MMHG;
	interim_map_kpa = MAP_MIN_KPA;
	app_exec = true;
	isButton = false;
	isTimer = false;
	isIndication = false;
	memset(g_blp_data, 0, sizeof(uint8_t) * BLP_DATA_LEN);
	g_idx = 0;
	
	timer_interval = INIT_TIMER_INTERVAL;
	
	//HEART RATE BEGINS
	app_state1 = 0;  /*!< flag to represent the application state*/
	advertisement_flag = false; /*!< to check if the device is in advertisement*/
	notification_flag1 = false; /*!< flag to start notification*/
	disconnect_flag = false;        /*!< flag for disconnection*/
	hr_initializer_flag = 1; /*!< flag for initialization of hr for each category*/
	second_counter = 0;     /*!< second_counter to count the time*/
	energy_expended_val = ENERGY_EXP_NORMAL; /*!< to count the energy expended*/
	energy_incrementor = 0;
	heart_rate_value = HEART_RATE_MIN_NORM; /*!< to count the heart rate value*/
	rr_interval_value = RR_VALUE_MIN; /*!< to count the rr interval value*/
	activity = 0; /*!< activiy which will determine the */
	prev_activity = DEFAULT_ACTIVITY; /*!< previous activity */
	inc_changer     = 1; /*!< increment operator to change heart rate */
	time_operator = 0; /*!< operator to change the seconds */
	hr_min_value = 0; /*!<the minimum heart rate value*/
	hr_max_value = 0; /*!<the maximum heart rate value*/
	// HEART RATE ENDS
	
	platform_driver_init();
	acquire_sleep_lock();
	/* Initialize the button */
	gpio_init();
	button_init();
	button_register_callback(button_cb);
	
	/* Initialize serial console */
	serial_console_init();
	/* Initialize the hardware timer */
	hw_timer_init();
	/* Register the callback */
	hw_timer_register_callback(timer_callback_handler);
	DBG_LOG("Initializing Blood Pressure Sensor Application");

	/* initialize the ble chip  and Set the device mac address */
	ble_device_init(NULL);

	
	/* Start timer */
	hw_timer_start(1);

	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;

	//! [setup_init]
	//! [init_system]
	system_clock_config(CLOCK_RESOURCE_XO_26_MHZ, CLOCK_FREQ_26_MHZ);
	//! [init_system]
	configure_gpio_pins();
	//! [setup_usart]
	configure_usart();
	//! [setup_usart]

	//! [setup_dma_resource]
	configure_dma_resource_tx(&uart_dma_resource_tx);
	configure_dma_resource_rx(&uart_dma_resource_rx);
	//! [setup_dma_resource]

	//! [setup_transfer_descriptor]
	setup_transfer_descriptor_tx(&example_descriptor_tx);
	setup_transfer_descriptor_rx(&example_descriptor_rx);
	//! [setup_transfer_descriptor]

	//! [add_descriptor_to_resource]
	dma_add_descriptor(&uart_dma_resource_tx, &example_descriptor_tx);
	dma_add_descriptor(&uart_dma_resource_rx, &example_descriptor_rx);
	//! [add_descriptor_to_resource]

	//! [configure_callback]
	configure_dma_callback();
	//! [configure_callback]
	//! [setup_init]

	//! [main]
	//! [main_1]
	dma_start_transfer_job(&uart_dma_resource_rx);
	//! [main_1]

/* read the temperature from the sensor */ 
htp_temperature_read(); 
/* Initialize the htp service */ 
htp_init(); 

fmp_target_init(NULL);

/* callback registration for immediate alert value*/
register_find_me_handler(app_immediate_alert);

register_ble_user_event_cb(user_callback_handler);


/* Register Bluetooth events Callbacks */ 
register_ble_callbacks(); 
/* Initialize the blood pressure sensor profile */
blp_sensor_init(NULL);

/** Initializing the application time stamp*/
time_stamp_init();

/* Registering the app_notification_handler with the profile */
register_blp_notification_handler(app_notification_handler);

/* Registering the app_indication_handler with the profile */
register_blp_indication_handler(app_indication_handler);

/* Triggering advertisement */
//blp_sensor_adv();

ble_mgr_events_callback_handler(REGISTER_CALL_BACK,
BLE_GAP_EVENT_TYPE,
app_gap_handle);
ble_mgr_events_callback_handler(REGISTER_CALL_BACK,
BLE_GATT_SERVER_EVENT_TYPE,
app_gatt_server_handle);
//HEART RATE BEGINS
/* Initialize the profile */
hr_sensor_init(NULL);

DBG_LOG("Press the button to start advertisement");

/* Registering the app_notification_handler with the profile */
register_hr_notification_handler(app_notification_handler1);

/* Registering the app_reset_handler with the profile */
register_hr_reset_handler(app_reset_handler);

/* Registering the call backs for events with the ble manager */
ble_mgr_events_callback_handler(REGISTER_CALL_BACK,
BLE_GAP_EVENT_TYPE,
app_gap_handle1);

ble_mgr_events_callback_handler(REGISTER_CALL_BACK,
BLE_GATT_SERVER_EVENT_TYPE,
app_gatt_server_handle1);
//HEART RATE ENDS

/* Start Advertising process */ 
ble_advertise(); 
		gpio_pin_set_output_level(LED_0_PIN, true);

while(true) 
	{ 
ble_event_task(0x655); 
		if (Timer_Flag & Temp_Notification_Flag)
		{
			htp_temperature_send();
		}
		
		//HEART RATE BEGINS
		/* Flag to start notification */
		if (notification_flag1) {
			//LED_Toggle(LED0);
			if (notification_sent1) {
				hr_measurment_send();
				} else {
				DBG_LOG("Previous notification not sent");
			}
			
			notification_flag1 = false;
		}
		//HEART RATE ENDS
		
		if (isTimer == true) {
			if (user_request_flag) {
				timer_count++;
				notify = true;
			}

			update_time_stamp();

			hw_timer_start(TIMER_INTERVAL);

			isTimer = false;
		}

		if (isButton == true) {
			/* App connected state*/
			if (app_state) {
				if (user_request_flag == false) {
					if (indication_flag) {
						/** For changing the units for each button press*/
						units = !units;
					}

					if (indication_flag || notification_flag) {
						/** To trigger the blood pressure indication */
						user_request_flag = true;
						timer_count = 0;
					}
					if (notification_flag) {
						DBG_LOG("\r\nStarted sending Interim Cuff Pressure Values");
					}
				}
			}

			isButton = false;
		}

		if (isIndication == true) {
			/* Sending the default notification for the first time */
			blp_sensor_send_indication(g_blp_data, g_idx);

			isIndication = false;
		}

		/* Checking for button press */
		if (user_request_flag ) {
			
			/*Sending notifications of interim cuff pressure*/
			
			if (timer_count < INDICATION_TIMER_VAL ) {
				
				/* checking for notification enabled */
				if (notification_flag) {
					
					/* Sending one notification per second */
					if (notify) {
						
						/* Checking for previous notification sent over the air */
						if (notification_sent) {
							blp_char_notification();
						}
						notify = 0;
					}
				}
			}
			
			if (timer_count == INDICATION_TIMER_VAL) {
				if (indication_flag) {
					/*Checking for previous indication sent over the  air */
					if (indication_sent) {
						
						/* Send a indication */
						blp_char_indication();
						} else {
						DBG_LOG("Previous indication is failed and device is disconnecting");
						blp_disconnection();
					}
				}
				timer_count = 0;
				user_request_flag = 0;
			}
		}
	} 
} 
