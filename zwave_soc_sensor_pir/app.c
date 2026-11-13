/**
 * Z-Wave Certified Application Sensor PIR
 *
 * @copyright 2018 Silicon Laboratories Inc.
 */

#include <stdbool.h>
#include <stdint.h>
#include <assert.h>
#include "MfgTokens.h"
#include "zpal_log.h"
#include "AppTimer.h"
#include "ZW_system_startup_api.h"
#include "CC_Basic.h"
#include "CC_Battery.h"
#include "CC_Notification.h"
#include "cc_notification_config_api.h"
#include "ZAF_Common_helper.h"
#include "ZAF_Common_interface.h"
#include "ZAF_network_learn.h"
#include "ZAF_network_management.h"
#include "events.h"
#include "zpal_watchdog.h"
#include "board_indicator.h"
#include "SensorPIR_UserTask_DataAcquisition.h"
#include "ZAF_ApplicationEvents.h"
#include "zaf_event_distributor_soc.h"
#include "cc_agi_config_api.h"
#include "zpal_misc.h"
#include "zaf_protocol_config.h"
#include "ZW_TransportEndpoint.h"
#include "zpal_power_manager.h"
#include "ZAF_PrintAppInfo.h"
#ifdef SL_CATALOG_ZW_PM_TRANSITION_EVENT_PRESENT
#include "app_pm_transition_event.h"
#endif

#ifdef SL_CATALOG_ZW_CLI_SLEEPING_PRESENT
#include "zw_cli_sleeping.h"
#include "zw_cli_sleeping_config.h"
#endif

#include "app_hw.h"
#include <stdio.h>
#include <string.h>

/*****************************************************
 * Set this macro to create a multi-threaded
 * application example! - This feature add a separate
 * UserTask to run with this application.
 * Place any code with external output in the separate
 * user-task file along side this application.
 *
 * Creating a user task will lead to increased power
 * consumption so if it is not needed it should be
 * disabled.
 *
 * Disable: 0
 * Enable:  1
 ****************************************************/
#define CREATE_USER_TASK          0

/**
 * The value basic set command should use when an event occur.
 */
#define BASIC_SET_TRIGGER_VALUE   (0xFF)
/**
 * The timeout value in milli seconds that used when sending basic set command due to an event
 */
#define BASIC_SET_TIMEOUT         10000

static uint8_t supportedEvents = NOTIFICATION_EVENT_HOME_SECURITY_MOTION_DETECTION_UNKNOWN_LOCATION;

// Timer
static SSwTimer EventJobsTimer;

/********************************
 * Data Acquisition Task
 *******************************/
#if CREATE_USER_TASK
#define TASK_STACK_SIZE_DATA_ACQUISITION           1000  // [bytes]

static TaskHandle_t m_xTaskHandleDataAcquisition   = NULL;

// Task and stack buffer allocation for the default/main application task!
static StaticTask_t DataAcquisitionTaskBuffer;
static uint8_t DataAcquisitionStackBuffer[TASK_STACK_SIZE_DATA_ACQUISITION];
#endif //CREATE_USER_TASK

void ApplicationTask(SApplicationHandles* pAppHandles);

void ZCB_EventJobsTimer(SSwTimer *pTimer);

/**
 * @brief See description for function prototype in ZW_basis_api.h.
 */
ZW_APPLICATION_STATUS
ApplicationInit(__attribute__((unused)) zpal_reset_reason_t eResetReason)
{
  SRadioConfig_t* RadioConfig;

  ZPAL_LOG_DEBUG(ZPAL_LOG_APP, "Enabling watchdog\n");
  zpal_watchdog_init();
  zpal_enable_watchdog(true);

  ZPAL_LOG_INFO(ZPAL_LOG_APP, "ApplicationInit eResetReason = %d\n", eResetReason);

  RadioConfig = zaf_get_radio_config();

#ifdef SL_CATALOG_ZW_PM_TRANSITION_EVENT_PRESENT
  // register callback from power manager transitions
  ZW_PmTransitionEventInit();
#endif

  // Read Rf region from MFG_ZWAVE_COUNTRY_FREQ
  zpal_radio_region_t regionMfg;
  ZW_GetMfgTokenDataCountryFreq((void*) &regionMfg);
  if (isRfRegionValid(regionMfg)) {
    RadioConfig->eRegion = regionMfg;
  } else {
    ZW_SetMfgTokenDataCountryRegion((void*) &RadioConfig->eRegion);
  }

  /*************************************************************************************
  * CREATE USER TASKS  -  ZW_ApplicationRegisterTask() and ZW_UserTask_CreateTask()
  *************************************************************************************
  * Register the main APP task function.
  *
  * ATTENTION: This function is the only task that can call ZAF API functions!!!
  * Failure to follow guidelines will result in undefined behavior.
  *
  * Furthermore, this function is the only way to register Event Notification
  * Bit Numbers for associating to given event handlers.
  *
  * ZW_UserTask_CreateTask() can be used to create additional tasks.
  * @see zwave_soc_sensor_pir example for more info.
  *************************************************************************************/
  __attribute__((unused)) bool bWasTaskCreated = ZW_ApplicationRegisterTask(
    ApplicationTask,
    EAPPLICATIONEVENT_ZWRX,
    EAPPLICATIONEVENT_ZWCOMMANDSTATUS,
    zaf_get_protocol_config()
    );
  assert(bWasTaskCreated);

  /*****************************************************
   * This is an multi-threaded application example!
   *
   * This next section creates the additional threads
   * by using ZW_UserTask.h API.
   *
   * If a multi-threaded application is not needed,
   * this next section can be removed by setting the
   * macro CREATE_USER_TASK to zero.
   ****************************************************/
#if CREATE_USER_TASK

  // Create the buffer bundle!
  ZW_UserTask_Buffer_t mainAppTaskBuffer;
  mainAppTaskBuffer.taskBuffer = &DataAcquisitionTaskBuffer;
  mainAppTaskBuffer.stackBuffer = DataAcquisitionStackBuffer;
  mainAppTaskBuffer.stackBufferLength = TASK_STACK_SIZE_DATA_ACQUISITION;

  // Create the task setting-structure!
  ZW_UserTask_t task;
  task.pTaskFunc = (TaskFunction_t)SensorPIR_DataAcquisitionTask;
  task.pTaskName = "DataAcqu";
  task.pUserTaskParam = NULL;  // We pass nothing here, as the EventHelper is already initialized and can be used for task IPC!
  task.priority = USERTASK_PRIORITY_HIGHEST;  // The difficult example is with the HIGHEST priority.
  task.taskBuffer = &mainAppTaskBuffer;

  // Create the task!
  ZW_UserTask_CreateTask(&task, &m_xTaskHandleDataAcquisition);

#endif //CREATE_USER_TASK

  return(APPLICATION_RUNNING);
}

/**
 * A pointer to this function is passed to ZW_ApplicationRegisterTask() making it the FreeRTOS
 * application task.
 */
void
ApplicationTask(SApplicationHandles* pAppHandles)
{
  uint32_t unhandledEvents = 0;
  zpal_reset_reason_t resetReason;

  ZPAL_LOG_DEBUG(ZPAL_LOG_APP, "\r\nSensorPIR Main App/Task started! \n");

  ZAF_Init(xTaskGetCurrentTaskHandle(), pAppHandles);

  ZAF_PrintAppInfo();

  app_hw_init();

  ////////////////////////////////////////////////////////////////////////////////////////////////////////
  /// TEST MAB 2025.12.29
  /// Display DSK over serial port
  /// (see zw_cli_common.c, cli_get_dsk() for CLI routine for displaying DSK)
  // DSK starts from the 13th byte in the QR code
  #define SL_CLI_COMMON_DSK_START_INDEX   12
  #define SL_CLI_COMMON_DSK_LEN_CHAR      40
  #define SL_CLI_DSK_CHUNK_LEN_CHAR       5
  char qr_code[TOKEN_MFG_ZW_QR_CODE_SIZE];
  char qr_code_print_buffer[TOKEN_MFG_ZW_QR_CODE_SIZE + 7 /*we put - after every 5th char*/];
  char lucIntegerString[10];
  char lucDSKHexString[50];
  char lucNWIHomeIDString[50];
  char lucAuthHomeIDString[50];
  uint16_t luiTempDSKInt;
  uint16_t luiTempNWIHomeIDInt;
  uint16_t luiTempAuthHomeIDInt;

  memset(qr_code, 0, sizeof(qr_code));
  memset(qr_code_print_buffer, 0, sizeof(qr_code_print_buffer));
  memset(lucDSKHexString, 0, sizeof(lucDSKHexString));
  memset(lucNWIHomeIDString, 0, sizeof(lucNWIHomeIDString));
  memset(lucAuthHomeIDString, 0, sizeof(lucAuthHomeIDString));

  ZW_GetMfgTokenData(&qr_code, TOKEN_MFG_ZW_QR_CODE_ID, TOKEN_MFG_ZW_QR_CODE_SIZE);
  ZPAL_LOG_INFO(ZPAL_LOG_APP, "%s: \n", __FUNCTION__);
  ZPAL_LOG_INFO(ZPAL_LOG_APP, "%s:     QR code = %s\n", __FUNCTION__, qr_code);

  for (uint8_t i = SL_CLI_COMMON_DSK_START_INDEX; i < SL_CLI_COMMON_DSK_START_INDEX + SL_CLI_COMMON_DSK_LEN_CHAR; i += 5)
  {
    // Debug display hex value of 5-digit number string
    memset(lucIntegerString, 0, sizeof(lucIntegerString));
    memcpy(lucIntegerString, &qr_code[i], SL_CLI_DSK_CHUNK_LEN_CHAR);
    luiTempDSKInt = atoi(lucIntegerString);
    //ZPAL_LOG_INFO(ZPAL_LOG_APP, "%s: %05d = 0x%04X\n", __FUNCTION__, luiTempDSKInt, luiTempDSKInt);
    strcat(qr_code_print_buffer, lucIntegerString);
    memset(lucIntegerString, 0, sizeof(lucIntegerString));
    snprintf(lucIntegerString, 5, "%04X", luiTempDSKInt); // 4 hex digits plus NULL, apparently...
    strcat(lucDSKHexString, lucIntegerString);

    //if (i < (SL_CLI_COMMON_DSK_START_INDEX + SL_CLI_COMMON_DSK_LEN_CHAR) - SL_CLI_DSK_CHUNK_LEN_CHAR) {
    if (i <= (SL_CLI_COMMON_DSK_START_INDEX + SL_CLI_COMMON_DSK_LEN_CHAR) - SL_CLI_DSK_CHUNK_LEN_CHAR) {
        if (i < (SL_CLI_COMMON_DSK_START_INDEX + SL_CLI_COMMON_DSK_LEN_CHAR) - SL_CLI_DSK_CHUNK_LEN_CHAR)
          {
            strcat(qr_code_print_buffer, "-");
            strcat(lucDSKHexString, "  ");
          }

        // Process NWI HomeID
        if ( i == (SL_CLI_COMMON_DSK_START_INDEX + 4*SL_CLI_DSK_CHUNK_LEN_CHAR) )
          {
            luiTempNWIHomeIDInt = luiTempDSKInt | 0xC000;
            memset(lucIntegerString, 0, sizeof(lucIntegerString));
            snprintf(lucIntegerString, 5, "%04X", luiTempNWIHomeIDInt); // 4 hex digits plus NULL, apparently...
            strcat(lucNWIHomeIDString, lucIntegerString);
            strcat(lucNWIHomeIDString, "  ");
          }
        else if ( i == (SL_CLI_COMMON_DSK_START_INDEX + 5*SL_CLI_DSK_CHUNK_LEN_CHAR) )
          {
            luiTempNWIHomeIDInt = luiTempDSKInt & 0xFFFE;
            memset(lucIntegerString, 0, sizeof(lucIntegerString));
            snprintf(lucIntegerString, 5, "%04X", luiTempNWIHomeIDInt); // 4 hex digits plus NULL, apparently...
            strcat(lucNWIHomeIDString, lucIntegerString);
            strcat(lucNWIHomeIDString, "  ");
          }
        else if ( i == (SL_CLI_COMMON_DSK_START_INDEX + 7*SL_CLI_DSK_CHUNK_LEN_CHAR) )
          {
            //strcat(lucNWIHomeIDString, "....");
            strcat(lucNWIHomeIDString, "    ");
          }
        else
          {
            //strcat(lucNWIHomeIDString, "....  ");
            strcat(lucNWIHomeIDString, "      ");
          }

        // Process Auth HomeID
        if ( i == (SL_CLI_COMMON_DSK_START_INDEX + 6*SL_CLI_DSK_CHUNK_LEN_CHAR) )
          {
            luiTempAuthHomeIDInt = luiTempDSKInt | 0xC000;
            memset(lucIntegerString, 0, sizeof(lucIntegerString));
            snprintf(lucIntegerString, 5, "%04X", luiTempAuthHomeIDInt); // 4 hex digits plus NULL, apparently...
            strcat(lucAuthHomeIDString, lucIntegerString);
            strcat(lucAuthHomeIDString, "  ");
          }
        else if ( i == (SL_CLI_COMMON_DSK_START_INDEX + 7*SL_CLI_DSK_CHUNK_LEN_CHAR) )
          {
            luiTempNWIHomeIDInt = luiTempDSKInt & 0xFFFE;
            memset(lucIntegerString, 0, sizeof(lucIntegerString));
            snprintf(lucIntegerString, 5, "%04X", luiTempNWIHomeIDInt); // 4 hex digits plus NULL, apparently...
            strcat(lucAuthHomeIDString, lucIntegerString);
            strcat(lucAuthHomeIDString, "  ");
          }
        else
          {
            //strcat(lucAuthHomeIDString, "....  ");
            strcat(lucAuthHomeIDString, "      ");
          }
    }
  }

  ZPAL_LOG_INFO(ZPAL_LOG_APP, "%s:   \n", __FUNCTION__);
  ZPAL_LOG_INFO(ZPAL_LOG_APP, "%s:         DSK = %s\n", __FUNCTION__, qr_code_print_buffer);
  ZPAL_LOG_INFO(ZPAL_LOG_APP, "%s:     DSK hex = %s\n", __FUNCTION__, lucDSKHexString);
  ZPAL_LOG_INFO(ZPAL_LOG_APP, "%s:  NWI HomeID = %s\n", __FUNCTION__, lucNWIHomeIDString);
  ZPAL_LOG_INFO(ZPAL_LOG_APP, "%s: Auth HomeID = %s\n", __FUNCTION__, lucAuthHomeIDString);
  ////////////////////////////////////////////////////////////////////////////////////////////////////////

  /* Make sure to call AppTimerDeepSleepPersistentRegister() _after_ ZAF_Init().
   * It will access the app handles */
  AppTimerDeepSleepPersistentRegister(&EventJobsTimer, false, ZCB_EventJobsTimer);  // register for event jobs timeout event

  resetReason = GetResetReason();

  /* Check the battery level.
   * If required, go to TRANSMIT state to send the report to the lifeline.
   * The Battery Report must be sent out before the WakeUp Notification. Therefore this function
   * must called prior to anything CC Wakeup related.
   */
  if (true == cc_battery_check_level_changed()) {
    zaf_event_distributor_enqueue_app_event(EVENT_APP_BATTERY_REPORT);
  }

  /* Re-load and process Deep Sleep persistent application timers.
   * NB: Before calling AppTimerDeepSleepPersistentLoadAll here all
   *     application timers must have been been registered with
   *     AppTimerRegister() or AppTimerDeepSleepPersistentRegister().
   *     Essentially it means that all CC handlers must be
   *     initialized first.
   */
  AppTimerDeepSleepPersistentLoadAll(resetReason);

  if (ZPAL_RESET_REASON_DEEP_SLEEP_EXT_INT == resetReason) {
    app_hw_deep_sleep_wakeup_handler();
  }

  /**
   * Set the maximum inclusion request interval for SmartStart.
   * Valid range 0 and 5-99. 0 is default value and corresponds to 512 sec.
   * The range 5-99 corresponds to 640-12672sec in units of 128sec/tick in between.
   */
  ZAF_SetMaxInclusionRequestIntervals(0);

  if (ZPAL_RESET_REASON_DEEP_SLEEP_EXT_INT != resetReason) {
    /* Enter SmartStart*/
    /* Protocol will commence SmartStart only if the node is NOT already included in the network */
    ZAF_setNetworkLearnMode(E_NETWORK_LEARN_MODE_INCLUSION_SMARTSTART);
  }

  ZPAL_LOG_DEBUG(ZPAL_LOG_APP, "\r\nIsWakeupCausedByRtccTimeout=%s", (IsWakeupCausedByRtccTimeout()) ? "true" : "false");
  ZPAL_LOG_DEBUG(ZPAL_LOG_APP, "\r\nCompletedSleepDurationMs   =%u", GetCompletedSleepDurationMs());

#ifdef SL_CATALOG_ZW_CLI_SLEEPING_PRESENT
  // Stay awake to allow user to send the prevent sleeping command through the CLI
  if (GetResetReason() == ZPAL_RESET_REASON_PIN) {
    zw_cli_sleeping_util_prevent_sleeping_timeout(ZW_CLI_SLEEPING_WAKEUP_TIME_AFTER_RESET);
  }
#endif

  // Wait for and process events
  ZPAL_LOG_DEBUG(ZPAL_LOG_APP, "\r\nSensorPIR Event processor Started\n");
  for (;; ) {
    unhandledEvents = zaf_event_distributor_distribute();
    if (0 != unhandledEvents) {
      ZPAL_LOG_DEBUG(ZPAL_LOG_APP, "Unhandled Events: 0x%08lx\n", unhandledEvents);
#ifdef UNIT_TEST
      return;
#endif
    }
  }
}

/**
 * @brief The core state machine of this sample application.
 * @param event The event that triggered the call of zaf_event_distributor_app_event_manager.
 */
void
zaf_event_distributor_app_event_manager(const uint8_t event)
{
  ZPAL_LOG_DEBUG(ZPAL_LOG_APP, "zaf_event_distributor_app_event_manager Ev: 0x%02x\r\n", event);
  cc_agi_group_t const * const agiTableRootDeviceGroups = cc_agi_get_rootdevice_groups();

  switch (event) {
    case EVENT_APP_BATTERY_REPORT:
      /* BATTERY REPORT EVENT received. Send a battery level report */
      ZPAL_LOG_DEBUG(ZPAL_LOG_APP, "\r\nBattery Level report transmit\n");
      (void) CC_Battery_LevelReport_tx(NULL, ENDPOINT_ROOT, NULL);
      break;
    case EVENT_APP_TRANSITION_TO_ACTIVE:
      if (!TimerIsActive(&EventJobsTimer)) {
        ZPAL_LOG_DEBUG(ZPAL_LOG_APP, "\r\n");
        ZPAL_LOG_DEBUG(ZPAL_LOG_APP, "\r\n      *!*!**!*!**!*!**!*!**!*!**!*!**!*!**!*!*");
        ZPAL_LOG_DEBUG(ZPAL_LOG_APP, "\r\n      *!*!*       PIR EVENT ACTIVE       *!*!*");
        ZPAL_LOG_DEBUG(ZPAL_LOG_APP, "\r\n      *!*!**!*!**!*!**!*!**!*!**!*!**!*!**!*!*");
        ZPAL_LOG_DEBUG(ZPAL_LOG_APP, "\r\n");

        (void) CC_Notification_TriggerAndTransmit(0,
                                                  NOTIFICATION_EVENT_HOME_SECURITY_MOTION_DETECTION_UNKNOWN_LOCATION,
                                                  &supportedEvents,
                                                  0,
                                                  NULL,
                                                  false);
        (void) CC_Basic_Set_tx(&agiTableRootDeviceGroups[0].profile, ENDPOINT_ROOT, BASIC_SET_TRIGGER_VALUE, true, NULL);

        (void) AppTimerDeepSleepPersistentStart(&EventJobsTimer, BASIC_SET_TIMEOUT);
      }
      break;
    case EVENT_APP_TRANSITION_TO_DEACTIVE:
      AppTimerDeepSleepPersistentStop(&EventJobsTimer);
      ZCB_EventJobsTimer(&EventJobsTimer);
      break;
    case EVENT_APP_USERTASK_DATA_ACQUISITION_READY:
      ZPAL_LOG_INFO(ZPAL_LOG_APP, "\r\nMainApp: Data Acquisition UserTask started and ready!");
      break;
    case EVENT_APP_USERTASK_DATA_ACQUISITION_FINISHED:
      ZPAL_LOG_INFO(ZPAL_LOG_APP, "MainApp: Data Acquisition UserTask finished!\r\n");
      break;
    default:
      break;
  }

#ifdef SL_CATALOG_ZW_CLI_COMMON_PRESENT
  cli_log_system_events(event);
#endif
}

void
zaf_nvm_app_reset(void)
{
  AppTimerDeepSleepPersistentResetStorage();
}

/**
 * @brief event jobs timeout event
 *
 * @param pTimer Timer connected to this method
 */
void
ZCB_EventJobsTimer(__attribute__((unused)) SSwTimer *pTimer)
{
  cc_agi_group_t const * const agiTableRootDeviceGroups = cc_agi_get_rootdevice_groups();

  ZPAL_LOG_DEBUG(ZPAL_LOG_APP, "\r\nTimer callback: ZCB_EventJobsTimer() pTimer->Id=%d", pTimer->Id);
  ZPAL_LOG_DEBUG(ZPAL_LOG_APP, "\r\n");
  ZPAL_LOG_DEBUG(ZPAL_LOG_APP, "\r\n      *!*!**!*!**!*!**!*!**!*!**!*!**!*!**!*!*");
  ZPAL_LOG_DEBUG(ZPAL_LOG_APP, "\r\n      *!*!*      PIR EVENT INACTIVE      *!*!*");
  ZPAL_LOG_DEBUG(ZPAL_LOG_APP, "\r\n      *!*!**!*!**!*!**!*!**!*!**!*!**!*!**!*!*");
  ZPAL_LOG_DEBUG(ZPAL_LOG_APP, "\r\n");

  /* If the node has been woken up from Deep Sleep because the event job timer timed out
   * the app will now be in the state STATE_APP_STARTUP. Need to switch to
   * STATE_APP_TRANSMIT_DATA to properly process the job events
   */
  (void) CC_Notification_TriggerAndTransmit(0,
                                            NOTIFICATION_EVENT_HOME_SECURITY_NO_EVENT,
                                            &supportedEvents,
                                            0,
                                            NULL,
                                            false);
  (void) CC_Basic_Set_tx(&agiTableRootDeviceGroups[0].profile, ENDPOINT_ROOT, 0, true, NULL);
}
