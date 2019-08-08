//=====================================================================
//   Leafony Platform BLE leaf Bule Geko library(TBGLib)
//     Platform     : BGM11x 
//
//		(c) 2019  Trillion-Node Study Group
//		Released under the MIT license
//		https://opensource.org/licenses/MIT
//
//      Rev.00 2019/08/01 First release
//=====================================================================
// Bluegiga BGLib Arduino interface library source file
// Copyright (c) 2014 Jeff Rowberg
// BGLib Arduino interface library code is placed under the MIT license
//=====================================================================
#include "TBGLib.h"


BGLib::BGLib( HardwareSerial *module, HardwareSerial *output, uint8_t pMode ) {
    /*  */
    uModule = module;
    uOutput = output;
    packetMode = pMode;
    // initialize packet buffers
    bgapiRXBuffer = (uint8_t *)malloc( bgapiRXBufferSize = 32 );
    bgapiTXBuffer = (uint8_t *)malloc( bgapiTXBufferSize = 32 );
    bgapiRXBufferPos = bgapiTXBufferPos = 0;
    /*  */
    onBusy = 0;
    onIdle = 0;
    onTimeout = 0;
    onBeforeTXCommand = 0;
    onTXCommandComplete = 0;
    /*  */

    /*  */
    /* [coex] cooexistence interface */
    #ifdef BGLIB_ENABLE_CMD_COEX_GET_COUNTERS
        ble_rsp_coex_get_counters = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_COEX_SET_OPTIONS
        ble_rsp_coex_set_options = 0;
    #endif
    /*  */

    /* [dfu] Device Firmware Upgrade */
    #ifdef BGLIB_ENABLE_CMD_DFU_FLASH_SET_ADDRESS
        ble_rsp_dfu_flash_set_address = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_DFU_FLASH_UPLOAD
        ble_rsp_dfu_flash_upload = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_DFU_FLASH_UPLOAD_FINISH
        ble_rsp_dfu_flash_upload_finish = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_DFU_RESET
//[未]        ble_rsp_dfu_reset = 0;
        /* no response */
    #endif
    /*  */
    #ifdef BGLIB_ENABLE_EVT_DFU_BOOT
        ble_evt_dfu_boot = 0;
    #endif
    #ifdef BGLIB_ENABLE_EVT_DFU_BOOT_FAILURE
        ble_evt_dfu_boot_failure =0;
    #endif
    /*  */

    /* [endpoint] Endpoint */
    #ifdef BGLIB_ENABLE_CMD_ENDPOINT_CLOSE
        ble_rsp_endpoint_close = 0;
    #endif
    /*  */

    /* [flash] Peripheral Store */
    #ifdef BGLIB_ENABLE_CMD_FLASH_PS_ERASE
        ble_rsp_flash_ps_erase = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_FLASH_PS_ERASE_ALL
        ble_rsp_flash_ps_erase_all = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_FLASH_PS_LOAD
        ble_rsp_flash_ps_load = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_FLASH_PS_SAVE
        ble_rsp_flash_ps_save = 0;
    #endif
    /*  */

    /* [gatt] Generic Attribute Profile */
    #ifdef BGLIB_ENABLE_CMD_GATT_DISCOVER_CHARACTERISTICS
        ble_rsp_gatt_discover_characteristics = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_GATT_DISCOVER_CHARACTERISTICS_BY_UUID
        ble_rsp_gatt_discover_characteristics_by_uuid = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_GATT_DISCOVER_DESCRIPTORS
        ble_rsp_gatt_discover_descriptors = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_GATT_DISCOVER_PRIMARY_SERVICES
        ble_rsp_gatt_discover_primary_services = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_GATT_DISCOVER_PRIMARY_SERVICES_BY_UUID
        ble_rsp_gatt_discover_primary_services_by_uuid = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_GATT_EXECUTE_CHARACTERISTIC_VALUE_WRITE
        ble_rsp_gatt_execute_characteristic_value_write = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_GATT_FIND_INCLUDED_SERVICES
        ble_rsp_gatt_find_include_services = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_GATT_PREPARE_CHARACTERISTIC_VALUE_RELIABLE_WRITE
        ble_rsp_gatt_prepare_characteristic_value_reliable_write = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_GATT_PREPARE_CHARACTERISTIC_VALUE_WRITE
        ble_rsp_gatt_prepare_characteristic_value_write = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_GATT_READ_CHARACTERISTIC_VALUE
        ble_rsp_gatt_read_characteristic_value = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_GATT_READ_CHARACTERISTIC_VALUE_BY_UUID
        ble_rsp_gatt_read_characteristic_value_by_uuid = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_GATT_READ_CHARACTERICTIC_VALUE_FROM_OFFSET
        ble_rsp_gatt_read_characteristic_value_from_offset = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_GATT_READ_DESCRIPTOR_VALUE
        ble_rsp_gatt_read_descriptor_value = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_GATT_READ_MULTIPLE_CHARACTERISTIC_VALUES
        ble_rsp_gatt_read_multiple_characteristic_values = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_GATT_SEND_CHARACTERICTIC_CONFIRMATION
        ble_rsp_gatt_send_characteristic_confirmation = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_GATT_SET_CHARACTERISTIC_NOTIFICATION
        ble_rsp_gatt_set_characteristic_notification = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_GATT_SET_MAX_MTU
        ble_rsp_gatt_set_max_mtu = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_GATT_WRITE_CHARACTERISTIC_VALUE
        ble_rsp_gatt_write_characteristic_value = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_GATT_WRITE_CHARACTERISTIC_VALUE_WITHOUT_RESPONSE
        ble_rsp_gatt_write_characteristic_value_without_response = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_GATT_WRITE_DESCRIPTOR_VALUE
        ble_rsp_gatt_write_descriptor_value = 0;
    #endif
    /*  */
    #ifdef BGLIB_ENABLE_EVT_GATT_CHARACTERISTIC
        ble_evt_gatt_characteristic = 0;
    #endif
    #ifdef BGLIB_ENABLE_EVT_GATT_CHARACTERISTIC_VALUE
        ble_evt_gatt_characteristic_value = 0;
    #endif
    #ifdef BGLIB_ENABLE_EVT_GATT_DESCRIPTOR
        ble_evt_gatt_descriptor = 0;
    #endif
    #ifdef BGLIB_ENABLE_EVT_GATT_DESCRIPTOR_VALUE
        ble_evt_gatt_descriptor_value = 0;
    #endif
    #ifdef BGLIB_ENABLE_EVT_GATT_MTU_EXCHANGED
        ble_evt_gatt_mtu_exchanged = 0;
    #endif
    #ifdef BGLIB_ENABLE_EVT_GATT_PROCEDURE_COMPLETED
        ble_evt_gatt_procedure_completed = 0;
    #endif
    #ifdef BGLIB_ENABLE_EVT_GATT_SERVICE
        ble_evt_gatt_service = 0;
    #endif
    /*  */

    /* [gatt_server] Generic Attribure Profile Server */
    #ifdef BGLIB_ENABLE_CMD_GATT_SERVER_FIND_ATTRIBUTE
        ble_rsp_gatt_server_find_attribute = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_GATT_SERVER_READ_ATTRIBUTE_TYPE
        ble_rsp_gatt_server_read_attribute_type = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_GATT_SERVER_READ_ATTRIBUTE_VALUE
        ble_rsp_gatt_server_read_attribute_value = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_GATT_SERVER_SEND_CHARACTERISTIC_NOTIFICATION
        ble_rsp_gatt_server_send_caracteristic_notification = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_GATT_SERVER_SEND_USER_READ_RESPONSE
        ble_rsp_gatt_server_send_user_read_response = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_GATT_SERVER_SEND_USER_WRITE_RESPONSE
        ble_rsp_gatt_server_send_user_write_response = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_GATT_SERVER_SENT_CAPABILITIES
        ble_rsp_gatt_server_sent_capabilities = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_GATT_SERVER_SET_DATABASE
        ble_rsp_gatt_server_set_database = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_GATT_SERVER_WRITE_ATTRIBUTE_VALUE
        ble_rsp_gatt_server_write_attribute_value = 0;
    #endif
    /*  */
    #ifdef BGLIB_ENABLE_EVT_GATT_SERVER_ATTRIBUTE_VALUE
        ble_evt_gatt_server_attribute_value = 0;
    #endif
    #ifdef BGLIB_ENABLE_EVT_GATT_SERVER_CHARACTERISTIC_STATUS
        ble_evt_gatt_server_characteristic_status = 0;
    #endif
    #ifdef BGLIB_ENABLE_EVT_GATT_SERVER_EXECUTE_WRITE_COMPLETED
        ble_evt_gatt_server_execute_write_completed = 0;
    #endif
    #ifdef BGLIB_ENABLE_EVT_GATT_SERVER_USER_READ_REQUEST
        ble_evt_gatt_server_user_read_request = 0;
    #endif
    #ifdef BGLIB_ENABLE_EVT_GATT_SERVER_USER_WRITE_REQUEST
        ble_evt_gatt_server_user_write_request = 0;
    #endif
    /*  */

    /* [hardware] Hardware */
    #ifdef BGLIB_ENABLE_CMD_HARDWARE_ENABLE_DCDC
        ble_rsp_hardware_enable_dcdc = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_HARDWARE_GET_TIME
        ble_rsp_hardware_get_time = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_HARDWARE_SET_LAZY_SOFT_TIMER
        ble_rsp_hardware_set_lazy_soft_timer = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_HARDWARE_SET_SOFT_TIMER
        ble_rsp_hardware_set_soft_timer = 0;
    #endif
    /*  */
    #ifdef BGLIB_ENABLE_EVT_HARDWARE_SOFT_TIMER
        ble_evt_hardware_soft_timer = 0;
    #endif
    /*  */


    /* [le connection] Connection management */
    #ifdef BGLIB_ENABLE_CMD_LE_CONNECTION_CLOSE
        ble_rsp_le_connection_close = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_LE_CONNECTION_DISABLE_SLAVE_LATENCY
        ble_rsp_le_connection_disable_slave_latancy = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_LE_CONNECTION_GET_RSSI
        ble_rsp_le_connection_get_rssi = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_LE_CONNECTION_SET_PARAMETERS
        ble_rsp_le_connection_set_parameters = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_LE_CONNECTION_SET_PHY
        ble_rsp_le_connection_set_phy = 0;
    #endif
    /*  */
    #ifdef BGLIB_ENABLE_EVT_LE_CONNECTION_CLOSED
        ble_evt_le_connection_closed = 0;
    #endif
    #ifdef BGLIB_ENABLE_EVT_LE_CONNECTION_OPENED
        ble_evt_le_connection_opend = 0;
    #endif
    #ifdef BGLIB_ENABLE_EVT_LE_CONNECTION_PARAMETERS
        ble_evt_le_connection_parameters = 0;
    #endif
    #ifdef BGLIB_ENABLE_EVT_LE_CONNECTION_PHY_STATUS
        ble_evt_le_connection_phy_status = 0;
    #endif
    #ifdef BGLIB_ENABLE_EVT_LE_CONNECTION_RSSI
        ble_evt_le_connection_rssi = 0;
    #endif
    /*  */


    /* [le gap] Generic Access Profile commands */
    #ifdef BGLIB_ENABLE_CMD_LE_GAP_BT5_SET_ADV_DATA
        ble_rsp_le_gap_bt5_set_adv_data = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_LE_GAP_BT5_SET_ADV_PARAMETERS
    #endif
    #ifdef BGLIB_ENABLE_CMD_LE_GAP_BT5_SET_MODE
        ble_rsp_le_gap_bt5_set_mode = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_LE_GAP_CLEAR_ADVERTISE_CONFIGURATION
        ble_rsp_le_gap_clear_advertise_configuration = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_LE_GAP_CONNECT
        ble_rsp_le_gap_connect = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_LE_GAP_DISCOVER
        ble_rsp_le_gap_discover = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_LE_GAP_END_PROCEDURE
        ble_rsp_le_gap_end_procedure = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_LE_GAP_OPEN
        ble_rsp_le_gap_open = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_LE_GAP_SET_ADV_DATA
        ble_rsp_le_gap_set_adv_data = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_LE_GAP_SET_ADV_PARAMETERS
        ble_rsp_le_gap_set_adv_parameters = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_LE_GAP_SET_ADV_TIMEOUT
        ble_rsp_le_gap_set_adv_timeout = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_LE_GAP_SET_ADVERTISE_CHANNEL_MAP
        ble_rsp_le_gap_set_advertise_channel_map = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_LE_GAP_SET_ADVERTISE_CONFIGURATION
        ble_rsp_le_gap_set_advertise_configuration = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_LE_GAP_SET_ADVERTISE_PHY
        ble_rsp_le_gap_set_advertise_phy = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_LE_GAP_SET_ADVERTISE_REPORT_SCAN_REQUEST
        ble_rsp_le_gap_set_advertise_report_scan_request = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_LE_GAP_SET_ADVERTISE_TIMING
        ble_rsp_le_gap_set_advertise_timing = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_LE_GAP_SET_CONN_PARAMEETERS
        ble_rsp_le_gap_set_conn_parameters = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_LE_GAP_SET_DATA_CHANNEL_CLASSIFICATION
        ble_rsp_le_gap_set_data_channel_classification = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_LE_GAP_SET_DISCOVERY_TIMING
        ble_rsp_le_gap_set_discovery_timing = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_LE_GAP_SET_DISCOVERY_TYPE
        ble_rsp_le_gap_set_discovery_type = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_LE_GAP_SET_MODE
        ble_rsp_le_gap_set_mode = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_LE_GAP_SET_PRIVACY_MODE
        ble_rsp_le_gap_set_privacy_mode = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_LE_GAP_SET_SCAN_PARAMETERS
        ble_rsp_le_gap_set_scan_parameters = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_LE_GAP_START_ADVERTISING
        ble_rsp_le_gap_start_advertising = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_LE_GAP_START_DISCOVERY
        ble_rsp_le_gap_start_discovery = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_LE_GAP_STOP_ADVERTISING
        ble_rsp_le_gap_stop_advertising = 0;
    #endif
    /*  */
    #ifdef BGLIB_ENABLE_EVT_LE_GAP_ADV_TIMEOUT
        ble_evt_le_gap_adv_timeout = 0;
    #endif
    #ifdef BGLIB_ENABLE_EVT_LE_GAP_SCAN_REQUEST
        ble_evt_le_gap_scan_request = 0;
    #endif
    #ifdef BGLIB_ENABLE_EVT_LE_GAP_SCAN_RESPONSE
        ble_evt_le_gap_scan_response = 0;
    #endif
    /*  */


    /* [SM] Security Manager */
    #ifdef BGLIB_ENABLE_CMD_SM_BONDING_CONFIRM
        ble_rsp_sm_bonding_confirm = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_SM_CONFIGURE
        ble_rsp_sm_configure = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_SM_DELETE_BONDING
        ble_rsp_sm_delete_bonding = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_SM_DELETE_BONDINGS
        ble_rsp_sm_delete_bondings = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_SM_ENTER_PASSKEY
        ble_rsp_sm_enter_passkey = 0;
    #endif
    #ifdef BGLIB_ENABLE_SM_INCREASE_SECURITY
        ble_rsp_sm_increase_security = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_SM_LIST_ALL_BONDINGS
        ble_rsp_sm_list_all_bondings = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_SM_PASSKEY_CONFIRM
        ble_rsp_sm_passkey_confirm = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_SM_SET_BONDABLE_MODE
        ble_rsp_sm_set_bondable_mode = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_SM_SET_DEBUG_MODE
        ble_rsp_sm_set_debug_mode = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_SM_SET_OOB_DATA
        ble_rsp_sm_set_oob_data = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_SM_SET_PASSKEY
        ble_rsp_sm_set_passkey = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_SM_SET_SC_REMOTE_OOB_DATA
        ble_rsp_sm_set_sc_remote_oob_data = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_SM_STORE_BONDING_CONFIGURATION
        ble_rsp_sm_store_bonding_configuration = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_SM_USE_SC_OOB
        ble_rsp_sm_use_sc_oob = 0;
    #endif
    /*  */
    #ifdef BGLIB_ENABLE_EVT_SM_BONDED
        ble_evt_sm_bonded = 0;
    #endif
    #ifdef BGLIB_ENABLE_EVT_SM_BONDING_FAILED
        ble_evt_sm_bonding_failed = 0;
    #endif
    #ifdef BGLIB_ENABLE_EVT_SM_CONFIRM_BONDING
        ble_evt_sm_confirm_bonding = 0;
    #endif
    #ifdef BGLIB_ENABLE_EVT_SM_CONFIRM_PASSKEY
        ble_evt_sm_confirm_passkey = 0;
    #endif
    #ifdef BGLIB_ENABLE_EVT_SM_LIST_ALL_BONDINGS_COMPLETE
        ble_evt_sm_list_all_bondings_complete = 0;
    #endif
    #ifdef BGLIB_ENABLE_EVT_SM_LIST_BONDING_ENTRY
        ble_evt_sm_list_bonding_entry = 0;
    #endif
    #ifdef BGLIB_ENABLE_EVT_SM_PASSKEY_DISPLAY
        ble_evt_sm_passkey_display = 0;
    #endif
    #ifdef BGLIB_ENABLE_EVT_SM_PASSKEY_REQUEST
        ble_evt_sm_passkey_request = 0;
    #endif
    /*  */


    /* [system] System commands */
    #ifdef BGLIB_ENABLE_CMD_SYSTEM_GET_BT_ADDRESS
        ble_rsp_system_get_bt_address = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_SYSTEM_GET_COUNTERS
        ble_rsp_system_get_counters = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_SYSTEM_GET_RANDOM_DATA
        ble_rsp_system_get_random_data = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_SYSTEM_HALT
        ble_rsp_system_halt = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_SYSTEM_HELLO
        ble_rsp_system_hello = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_SYSTEM_RESET
        ble_rsp_system_reset = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_SYSTEM_SET_BT_ADDRESS
        ble_rsp_system_set_bt_address = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_SYSTEM_SET_DEVICE_NAME
        ble_rsp_system_set_device_name = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_SYSTEM_SET_TX_POWER
        ble_rsp_system_set_tx_power = 0;
    #endif
    /*  */
    #ifdef BGLIB_ENABLE_EVT_SYSTEM_AWAKE
        ble_evt_system_awake = 0;
    #endif
    #ifdef BGLIB_ENABLE_EVT_SYSTEM_BOOT
        ble_evt_system_boot = 0;
    #endif
    #ifdef BGLIB_ENABLE_EVT_SYSTEM_ERROR
        ble_evt_system_error = 0;
    #endif
    #ifdef BGLIB_ENABLE_EVT_SYSTEM_EXTERNAL_SIGNAL
        ble_evt_system_external_signal = 0;
    #endif
    #ifdef BGLIB_ENABLE_EVT_SYSTEM_HARDWARE_ERROR
        ble_evt_system_hardware_error = 0;
    #endif
    /*  */


    /* [test] testing */
    #ifdef BGLIB_ENABLE_CMD_TEST_DTM_END
        ble_rsp_test_dtm_end = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_TEST_DTM_RX
        ble_rsp_test_dtm_rx = 0;
    #endif
    #ifdef BGLIB_ENABLE_CMD_TEST_DTM_TX
        ble_rsp_test_dtm_tx = 0;
    #endif
    /*  */
    #ifdef BGLIB_ENABLE_EVT_TEST_DTM_COMPLETED
        ble_evt_test_dtm_completed = 0;
    #endif
    /*  */


    /* [user] User message */
    #ifdef BGLIB_ENABLE_CMD_USER_MESSAGE_TO_TARGET
        ble_rsp_user_message_to_target = 0;
    #endif
    /*  */
    #ifdef BGLIB_ENABLE_EVT_USER_MASSAGE_TO_HOST
        ble_evt_user_message_to_host = 0;
    #endif
    /*  */
}


uint8_t BGLib::checkActivity( uint16_t timeout ) {
    /*  */
    uint16_t ch;
    /*  */
#if 1
/* [moto] 仮 */
    timeoutStart = millis();          /* [moto] 初期値を設定する*/
#endif
    /*  */
    while( ((ch = uModule -> read()) < 256) && ((timeout == 0) || (millis() - timeoutStart < timeout)) ) {
        /* 読込みデータが無く、タイムアウトするまで */
        parse( ch );                                        /* 解析 */
        if( timeout > 0 ) timeoutStart = millis();          /* 受信データがある限り、スタート時間を更新する ?? */
    }   /* while */
    /*  */
    if( (timeout > 0) && busy && (millis() - timeoutStart >= timeout) ) {
        lastTimeout = true;
        if( onTimeout != 0 ) onTimeout();
        setBusy( false );
    }
    /*  */
    return busy;
}

uint8_t BGLib::checkError( void ) {
    return lastError;
}

uint8_t BGLib::checkTimeout( void ) {
    return lastTimeout;
}

uint8_t *BGLib::getLastCommand( void ) {
    return lastCommand;
}
uint8_t *BGLib::getLastResponse( void ) {
    return lastResponse;
}
uint8_t *BGLib::getLastEvent( void ) {
    return lastEvent;
}
void *BGLib::getLastRXPayload( void ) {
    return (bgapiRXBuffer + 4);
}

void BGLib::setBusy( bool busyEnabled ) {
    busy = busyEnabled;
    if( busy )
    {
        timeoutStart = millis();
        lastTimeout = false;
        if( onBusy ) onBusy();
    }
    else
    {
        lastError = false;
        if( onIdle ) onIdle();
    }
}

// set/update UART port objects
void BGLib::setModuleUART( HardwareSerial *module ) {
    uModule = module;
}

void BGLib::setOutputUART( HardwareSerial *output ) {
    uOutput = output;
}


uint8_t BGLib::parse( uint8_t ch, uint8_t packetMode ) {
    #ifdef BGLIB_DEBUG
        // DEBUG: output hex value of incoming character
        if( ch < 16 ) SerialUSB.write( 0x30 );      // leading '0'
        SerialUSB.print( ch, HEX );                 // actual hex value
        SerialUSB.write( 0x20 );                    // trailing ' '
    #endif

    /*  */
    if( bgapiRXBufferPos == bgapiRXBufferSize )
    {
        // expand RX buffer to prevent overflow
        bgapiRXBuffer = (uint8_t *)realloc( bgapiRXBuffer, bgapiRXBufferSize += 16 );
    }

    // check packet position
    if( bgapiRXBufferPos == 0 )
    {
        // beginning of packet, check for correct framing/expected byte(s)
        // BGAPI packet for Bluetooth Smart Single Mode must be either Command/Response (0x00) or Event (0x80)
        // BGAPI packet for Bluetooth Smart Single Mode must be either Command/Response (0x20) or Event (0xA0)
//[moto]        if( (ch & 0x78) == 0x00 )
        if( (ch & 0x58) == 0x00 )
        {
            // store new character in RX buffer
            if( ch != 0x00 )
            {
                /* リセット後に最初に0x00が付いてくるので、除外する */
                bgapiRXBuffer[ bgapiRXBufferPos++ ] = ch;
            }
        }
        else
        {
            #ifdef BGLIB_DEBUG
                SerialUSB.print( "*** Packet frame sync error! Expected .0000... binary, got 0x" );
                SerialUSB.println( ch, HEX );
            #endif
            return 1; // packet format error
        }
    }
    else
    {
        // middle of packet, assume we're okay
        bgapiRXBuffer[ bgapiRXBufferPos++ ] = ch;
#if 1
        if( bgapiRXBufferPos == 2 )
        {
            // just received "Length Low" byte, so store expected packet length
            bgapiRXDataLen = ch;
        }
#else
        if( bgapiRXBufferPos == 2 )
        {
            // just received "Length Low" byte, so store expected packet length
            bgapiRXDataLen = ch + ( (bgapiRXBuffer[0] & 0x03) << 8 );
        }
#endif
        else if( bgapiRXBufferPos == (bgapiRXDataLen + 4) )
        {
            // just received last expected byte
            #ifdef BGLIB_DEBUG
                SerialUSB.print( "\n<=[ " );
                for( uint8_t i = 0; i < bgapiRXBufferPos; i++ )
                {
                    if( bgapiRXBuffer[i] < 16 ) SerialUSB.write( 0x30 );
                    SerialUSB.print( bgapiRXBuffer[i], HEX );
                    SerialUSB.write( 0x20 );
                }
                SerialUSB.println( "]" );
            #endif

            // reset RX packet buffer position to be ready for new packet
            bgapiRXBufferPos = 0;

            // check packet type
//[moto]            if( (bgapiRXBuffer[0] & 0x80) == 0 )
//            if( (bgapiRXBuffer[0] & 0xA0) == 0x00 )
            if( (bgapiRXBuffer[0] & 0x80) == 0x00 )
            {
                // 0x00 = Response packet [old]
                // 0x20 = Response packet

                // capture last response class/command bytes
                lastResponse[0] = bgapiRXBuffer[2];
                lastResponse[1] = bgapiRXBuffer[3];


                if( bgapiRXBuffer[2] == MESSAGE_CLASS_COEX )
                {
                    /* Cooexistence interface */
                    if (false) { }
                    #ifdef BGLIB_ENABLE_CMD_COEX_GET_COUNTERS
                        else if( bgapiRXBuffer[3] == RSP_COEX_GET_COUNTERS_ID ){
                            if( ble_rsp_coex_get_counters ){
                                ble_rsp_coex_get_counters( (const struct ble_msg_coex_get_counters_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_COEX_SET_OPTIONS
                        else if( bgapiRXBuffer[3] == RSP_COEX_SET_OPTIONS_ID ){
                            if( ble_rsp_coex_set_options ){
                                ble_rsp_coex_set_options( (const struct ble_msg_coex_set_options_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                }
                else if( bgapiRXBuffer[2] == MESSAGE_CLASS_DFU )
                {
                    /* [dfu] Device Firmware Upgrade */
                    if (false) { }
                    #ifdef BGLIB_ENABLE_CMD_DFU_FLASH_SET_ADDRESS
                        else if( bgapiRXBuffer[3] == RSP_DFU_FLASH_SET_ADDRESS_ID ){
                            if( ble_rsp_dfu_flash_set_address ){
                                ble_rsp_dfu_flash_set_address( (const struct ble_msg_dfu_flash_set_address_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_DFU_FLASH_UPLOAD
                        else if( bgapiRXBuffer[3] == RSP_DFU_FLASH_UPLOAD_ID ){
                            if( ble_rsp_dfu_flash_upload ){
                                ble_rsp_dfu_flash_upload( (const struct ble_msg_dfu_flash_upload_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_DFU_FLASH_UPLOAD_FINISH
                        else if( bgapiRXBuffer[3] == RSP_DFU_FLASH_UPLOAD_FINISH_ID ){
                        }
                            if( ble_rsp_dfu_flash_upload_finish ){
                                ble_rsp_dfu_flash_upload_finish( (const struct ble_msg_dfu_flash_upload_finish_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_DFU_RESET
                        /* no response */
                    #endif
                }
                else if( bgapiRXBuffer[2] == MESSAGE_CLASS_ENDPOINT )
                {
                    /* [endpoint] Endpoint */
                    if (false) { }
                    #ifdef BGLIB_ENABLE_CMD_ENDPOINT_CLOSE
                        else if( bgapiRXBuffer[3] == RSP_ENDPOINT_CLOSE_ID ){
                            if( ble_rsp_endpoint_close ){
                                ble_rsp_endpoint_close( (const struct ble_msg_endpoint_close_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                }
                else if( bgapiRXBuffer[2] == MESSAGE_CLASS_FLASH )
                {
                    /* [flash] Peripheral Store */
                    if (false) { }
                    #ifdef BGLIB_ENABLE_CMD_FLASH_PS_ERASE
                        else if( bgapiRXBuffer[3] == RSP_FLASH_PS_ERASE_ID ){
                            if( ble_rsp_flash_ps_erase ){
                                ble_rsp_flash_ps_erase( (const struct ble_msg_flash_ps_erase_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_FLASH_PS_ERASE_ALL
                        else if( bgapiRXBuffer[3] == RSP_FLASH_PS_ERASE_ALL_ID ){
                            if( ble_rsp_flash_ps_erase_all ){
                                ble_rsp_flash_ps_erase_all( (const struct ble_msg_flash_ps_erase_all_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_FLASH_PS_LOAD
                        else if( bgapiRXBuffer[3] == RSP_FLASH_PS_LOAD_ID ){
                            if( ble_rsp_flash_ps_load ){
                                ble_rsp_flash_ps_load( (const struct ble_msg_flash_ps_load_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_FLASH_PS_SAVE
                        else if( bgapiRXBuffer[3] == RSP_FLASH_PS_SAVE_ID ){
                            if( ble_rsp_flash_ps_save ){
                                ble_rsp_flash_ps_save( (const struct ble_msg_flash_ps_save_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                }
                else if( bgapiRXBuffer[2] == MESSAGE_CLASS_GATT )
                {
                    /* [gatt] Generic Attribute Profile */
                    if (false) { }
                    #ifdef BGLIB_ENABLE_CMD_GATT_DISCOVER_CHARACTERISTICS
                        else if( bgapiRXBuffer[3] == RSP_GATT_DISCOVER_CHARACTERISTICS_ID ){
                            if( ble_rsp_gatt_discover_characteristics ){
                                ble_rsp_gatt_discover_characteristics( (const struct ble_msg_gatt_discover_characteristics_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_GATT_DISCOVER_CHARACTERISTICS_BY_UUID
                        else if( bgapiRXBuffer[3] == RSP_GATT_DISCOVER_CHARACTERISTICS_BY_UUID_ID ){
                            if( ble_rsp_gatt_discover_characteristics_by_uuid ){
                                ble_rsp_gatt_discover_characteristics_by_uuid( (const struct ble_msg_gatt_discover_characteristics_by_uuid_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_GATT_DISCOVER_DESCRIPTORS
                        else if( bgapiRXBuffer[3] == RSP_GATT_DISCOVER_DESCRIPTORS_ID ){
                            if( ble_rsp_gatt_discover_descriptors ){
                                ble_rsp_gatt_discover_descriptors( (const struct ble_msg_gatt_discover_descriptors_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_GATT_DISCOVER_PRIMARY_SERVICES
                        else if( bgapiRXBuffer[3] == RSP_GATT_DISCOVER_PRIMARY_SERVICES_ID ){
                            if( ble_rsp_gatt_discover_primary_services ){
                                ble_rsp_gatt_discover_primary_services( (const struct ble_msg_gatt_discover_primary_services_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_GATT_DISCOVER_PRIMARY_SERVICES_BY_UUID
                        else if( bgapiRXBuffer[3] == RSP_GATT_DISCOVER_PRIMARY_SERVICES_BY_UUID_ID ){
                            if( ble_rsp_gatt_discover_primary_services_by_uuid ){
                                ble_rsp_gatt_discover_primary_services_by_uuid( (const struct ble_msg_gatt_discover_primary_services_by_uuid_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_GATT_EXECUTE_CHARACTERISTIC_VALUE_WRITE
                        else if( bgapiRXBuffer[3] == RSP_GATT_EXECUTE_CHARACTERISTIC_VALUE_WRITE_ID ){
                            if( ble_rsp_gatt_execute_characteristic_value_write ){
                                ble_rsp_gatt_execute_characteristic_value_write( (const struct ble_msg_gatt_execute_characteristic_value_write_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_GATT_FIND_INCLUDED_SERVICES
                        else if( bgapiRXBuffer[3] == RSP_GATT_FIND_INCLUDED_SERVICES_ID ){
                            if( ble_rsp_gatt_find_include_services ){
                                ble_rsp_gatt_find_include_services( (const struct ble_msg_gatt_find_include_services_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_GATT_PREPARE_CHARACTERISTIC_VALUE_RELIABLE_WRITE
                        else if( bgapiRXBuffer[3] == RSP_GATT_PREPARE_CHARACTERISTIC_VALUE_RELIABLE_WRITE_ID ){
                            if( ble_rsp_gatt_prepare_characteristic_value_reliable_write ){
                                ble_rsp_gatt_prepare_characteristic_value_reliable_write( (const struct ble_msg_gatt_prepare_characteristic_value_reliable_write_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_GATT_PREPARE_CHARACTERISTIC_VALUE_WRITE
                        else if( bgapiRXBuffer[3] == RSP_GATT_PREPARE_CHARACTERISTIC_VALUE_WRITE_ID ){
                            if( ble_rsp_gatt_prepare_characteristic_value_write ){
                                ble_rsp_gatt_prepare_characteristic_value_write( (const struct ble_msg_gatt_prepare_characteristic_value_write_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_GATT_READ_CHARACTERISTIC_VALUE
                        else if( bgapiRXBuffer[3] == RSP_GATT_READ_CHARACTERISTIC_VALUE_ID ){
                            if( ble_rsp_gatt_read_characteristic_value ){
                                ble_rsp_gatt_read_characteristic_value( (const struct ble_msg_gatt_read_characteristic_value_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_GATT_READ_CHARACTERISTIC_VALUE_BY_UUID
                        else if( bgapiRXBuffer[3] == RSP_GATT_READ_CHARACTERISTIC_VALUE_BY_UUID_ID ){
                            if( ble_rsp_gatt_read_characteristic_value_by_uuid ){
                                ble_rsp_gatt_read_characteristic_value_by_uuid( (const struct ble_msg_gatt_read_characteristic_value_by_uuid_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_GATT_READ_CHARACTERICTIC_VALUE_FROM_OFFSET
                        else if( bgapiRXBuffer[3] == RSP_GATT_READ_CHARACTERISTIC_VALUE_FROM_OFFSET_ID ){
                            if( ble_rsp_gatt_read_characteristic_value_from_offset ){
                                ble_rsp_gatt_read_characteristic_value_from_offset( (const struct ble_msg_gatt_read_characteristic_value_from_offset_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_GATT_READ_DESCRIPTOR_VALUE
                        else if( bgapiRXBuffer[3] == RSP_GATT_READ_DESCRIPTOR_VALUE_ID ){
                            if( ble_rsp_gatt_read_descriptor_value ){
                                ble_rsp_gatt_read_descriptor_value( (const struct ble_msg_gatt_read_descriptor_value_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_GATT_READ_MULTIPLE_CHARACTERISTIC_VALUES
                        else if( bgapiRXBuffer[3] == RSP_GATT_READ_MULTIPLE_CHARACTERISTIC_VALUES_ID ){
                            if( ble_rsp_gatt_read_multiple_characteristic_values ){
                                ble_rsp_gatt_read_multiple_characteristic_values( (const struct ble_msg_gatt_read_multiple_characteristic_values_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_GATT_SEND_CHARACTERICTIC_CONFIRMATION
                        else if( bgapiRXBuffer[3] == RSP_GATT_SEND_CHARACTERISTIC_CONFIRMATION_ID ){
                            if( ble_rsp_gatt_send_characteristic_confirmation ){
                                ble_rsp_gatt_send_characteristic_confirmation( (const struct ble_msg_gatt_send_characteristic_confirmation_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_GATT_SET_CHARACTERISTIC_NOTIFICATION
                        else if( bgapiRXBuffer[3] == RSP_GATT_SET_CHARACTERISTIC_NOTIFICATION_ID ){
                            if( ble_rsp_gatt_set_characteristic_notification ){
                                ble_rsp_gatt_set_characteristic_notification( (const struct ble_msg_gatt_set_characteristic_notification_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_GATT_SET_MAX_MTU
                        else if( bgapiRXBuffer[3] == RSP_GATT_SET_MAX_MTU_ID ){
                            if( ble_rsp_gatt_set_max_mtu ){
                                ble_rsp_gatt_set_max_mtu( (const struct ble_msg_gatt_set_max_mtu_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_GATT_WRITE_CHARACTERISTIC_VALUE
                        else if( bgapiRXBuffer[3] == RSP_GATT_WRITE_CHARACTERISTIC_VALUE_ID ){
                            if( ble_rsp_gatt_write_characteristic_value ){
                                ble_rsp_gatt_write_characteristic_value( (const struct ble_msg_gatt_write_characteristic_value_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_GATT_WRITE_CHARACTERISTIC_VALUE_WITHOUT_RESPONSE
                        else if( bgapiRXBuffer[3] == RSP_GATT_WRITE_CHARACTERISTIC_VALUE_WITHOUT_RESPONSE_ID ){
                            if( ble_rsp_gatt_write_characteristic_value_without_response ){
                                ble_rsp_gatt_write_characteristic_value_without_response( (const struct ble_msg_gatt_write_characteristic_value_without_response_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_GATT_WRITE_DESCRIPTOR_VALUE
                        else if( bgapiRXBuffer[3] == RSP_GATT_WRITE_DESCRIPTOR_VALUE_ID ){
                            if( ble_rsp_gatt_write_descriptor_value ){
                                ble_rsp_gatt_write_descriptor_value( (const struct ble_msg_gatt_write_descriptor_value_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                }
                else if( bgapiRXBuffer[2] == MESSAGE_CLASS_GATT_SERVER )
                {
                    /* [gatt_server] Generic Attribure Profile Server */
                    if (false) { }
                    #ifdef BGLIB_ENABLE_CMD_GATT_SERVER_FIND_ATTRIBUTE
                        else if( bgapiRXBuffer[3] == RSP_GATT_SERVER_FIND_ATTRIBUTE_ID ){
                            if( ble_rsp_gatt_server_find_attribute ){
                                ble_rsp_gatt_server_find_attribute( (const struct ble_msg_gatt_server_find_attribute_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_GATT_SERVER_READ_ATTRIBUTE_TYPE
                        else if( bgapiRXBuffer[3] == RSP_GATT_SERVER_READ_ATTRIBUTE_TYPE_ID ){
                            if( ble_rsp_gatt_server_read_attribute_type ){
                                ble_rsp_gatt_server_read_attribute_type( (const struct ble_msg_gatt_server_read_attribute_type_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_GATT_SERVER_READ_ATTRIBUTE_VALUE
                        else if( bgapiRXBuffer[3] == RSP_GATT_SERVER_READ_ATTRIBUTE_VALUE_ID ){
                            if( ble_rsp_gatt_server_read_attribute_value ){
                                ble_rsp_gatt_server_read_attribute_value( (const struct ble_msg_gatt_server_read_attribute_balue_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_GATT_SERVER_SEND_CHARACTERISTIC_NOTIFICATION
                        else if( bgapiRXBuffer[3] == RSP_GATT_SERVER_SEND_CHARACTERISTIC_NOTIFICATION_ID ){
                            if( ble_rsp_gatt_server_send_caracteristic_notification ){
                                ble_rsp_gatt_server_send_caracteristic_notification( (const struct ble_msg_gatt_server_send_characteristic_notification_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_GATT_SERVER_SEND_USER_READ_RESPONSE
                        else if( bgapiRXBuffer[3] == RSP_GATT_SERVER_SEND_USER_READ_RESPONS_ID ){
                            if( ble_rsp_gatt_server_send_user_read_response ){
                                ble_rsp_gatt_server_send_user_read_response( (const struct ble_msg_gatt_server_send_user_read_response_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_GATT_SERVER_SEND_USER_WRITE_RESPONSE
                        else if( bgapiRXBuffer[3] == RSP_GATT_SERVER_SEND_USER_WRITE_RESPONS_ID ){
                            if( ble_rsp_gatt_server_send_user_write_response ){
                                ble_rsp_gatt_server_send_user_write_response( (const struct ble_msg_gatt_server_send_user_write_response_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_GATT_SERVER_SENT_CAPABILITIES
                        else if( bgapiRXBuffer[3] == RSP_GATT_SERVER_SET_CAPABILITIES_ID ){
                            if( ble_rsp_gatt_server_sent_capabilities ){
                                ble_rsp_gatt_server_sent_capabilities( (const struct ble_msg_gatt_server_sent_capabilities_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_GATT_SERVER_SET_DATABASE
                        else if( bgapiRXBuffer[3] == RSP_GATT_SERVER_SET_DATABASE_ID ){
                            if( ble_rsp_gatt_server_set_database ){
                                ble_rsp_gatt_server_set_database( (const struct ble_msg_gatt_server_set_database_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_GATT_SERVER_WRITE_ATTRIBUTE_VALUE
                        else if( bgapiRXBuffer[3] == RSP_GATT_SERVER_WRITE_ATTRIBUTE_VALUE_ID ){
                            if( ble_rsp_gatt_server_write_attribute_value ){
                                ble_rsp_gatt_server_write_attribute_value( (const struct ble_msg_gatt_server_write_arttribute_value_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                }
                else if( bgapiRXBuffer[2] == MESSAGE_CLASS_HARDWARE )
                {
                    /* [hardware] Hardware */
                    if (false) { }
                    #ifdef BGLIB_ENABLE_CMD_HARDWARE_ENABLE_DCDC
                        else if( bgapiRXBuffer[3] == RSP_GATT_HARDWARE_ENABLE_DCDC_ID ){
                            if( ble_rsp_hardware_enable_dcdc ){
                                ble_rsp_hardware_enable_dcdc( (const struct ble_msg_hardware_enable_dcdc_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_HARDWARE_GET_TIME
                        else if( bgapiRXBuffer[3] == RSP_GATT_HARDWARE_GET_TIME_ID ){
                            if( ble_rsp_hardware_get_time ){
                                ble_rsp_hardware_get_time( (const struct ble_msg_hardware_get_time_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_HARDWARE_SET_LAZY_SOFT_TIMER
                        else if( bgapiRXBuffer[3] == RSP_GATT_HARDWARE_SET_LAZY_SOFT_TIMER_ID ){
                            if( ble_rsp_hardware_set_lazy_soft_timer ){
                                ble_rsp_hardware_set_lazy_soft_timer( (const struct ble_msg_hardware_set_lazy_soft_timer_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_HARDWARE_SET_SOFT_TIMER
                        else if( bgapiRXBuffer[3] == RSP_GATT_HARDWARE_SET_SOFT_TIMER_ID ){
                            if( ble_rsp_hardware_set_soft_timer ){
                                ble_rsp_hardware_set_soft_timer( (const struct ble_msg_hardware_set_soft_timer_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                }
                else if( bgapiRXBuffer[2] == MESSAGE_CLASS_LE_CONNECTION )
                {
                    /* [le connection] Connection management */
                    if (false) { }
                    #ifdef BGLIB_ENABLE_CMD_LE_CONNECTION_CLOSE
                        else if( bgapiRXBuffer[3] == RSP_LE_CONNECTION_CLOSE_ID ){
                            if( ble_rsp_le_connection_close ){
                                ble_rsp_le_connection_close( (const struct ble_msg_le_connection_close_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_LE_CONNECTION_DISABLE_SLAVE_LATENCY
                        else if( bgapiRXBuffer[3] == RSP_LE_CONNECTION_DISABLE_SLAVE_LATENCY_ID ){
                            if( ble_rsp_le_connection_disable_slave_latancy ){
                                ble_rsp_le_connection_disable_slave_latancy( (const struct ble_msg_le_connection_disable_slave_latancy_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_LE_CONNECTION_GET_RSSI
                        else if( bgapiRXBuffer[3] == RSP_LE_CONNECTION_GET_RSSI_ID ){
                            if( ble_rsp_le_connection_get_rssi ){
                                ble_rsp_le_connection_get_rssi( (const struct ble_msg_le_connection_get_rssi_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_LE_CONNECTION_SET_PARAMETERS
                        else if( bgapiRXBuffer[3] == RSP_LE_CONNECTION_SET_PARAMETERS_ID ){
                            if( ble_rsp_le_connection_set_parameters ){
                                ble_rsp_le_connection_set_parameters( (const struct ble_msg_le_connection_set_parameters_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_LE_CONNECTION_SET_PHY
                        else if( bgapiRXBuffer[3] == RSP_LE_CONNECTION_SET_PHY_ID ){
                            if( ble_rsp_le_connection_set_phy ){
                                ble_rsp_le_connection_set_phy( (const struct ble_msg_le_connection_set_phy_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                }
                else if( bgapiRXBuffer[2] == MESSAGE_CLASS_LE_GAP )
                {
                    /* [le gap] Generic Access Profile commcnds */
                    if (false) { }
                    #ifdef BGLIB_ENABLE_CMD_LE_GAP_BT5_SET_ADV_DATA
                        else if( bgapiRXBuffer[3] == RSP_LE_GAP_BT5_SET_ADV_DATA_ID ){
                            if( ble_rsp_le_gap_bt5_set_adv_data ){
                                ble_rsp_le_gap_bt5_set_adv_data( (const struct ble_msg_le_gap_bt5_set_adv_data_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_LE_GAP_BT5_SET_ADV_PARAMETERS
                        else if( bgapiRXBuffer[3] == RSP_LE_GAP_BT5_SET_ADV_PARAMETERS_ID ){
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_LE_GAP_BT5_SET_MODE
                        else if( bgapiRXBuffer[3] == RSP_LE_GAP_BT5_SET_MODE_ID ){
                            if( ble_rsp_le_gap_bt5_set_mode ){
                                ble_rsp_le_gap_bt5_set_mode( (const struct ble_msg_le_gap_bt5_set_mode_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_LE_GAP_CLEAR_ADVERTISE_CONFIGURATION
                        else if( bgapiRXBuffer[3] == RSP_LE_GAP_CLEAR_ADVERTISE_CONFIGURATION_ID ){
                            if( ble_rsp_le_gap_clear_advertise_configuration ){
                                ble_rsp_le_gap_clear_advertise_configuration( (const struct ble_msg_le_gap_clear_advertise_configuration_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_LE_GAP_CONNECT
                        else if( bgapiRXBuffer[3] == RSP_LE_GAP_CONNECT_ID ){
                            if( ble_rsp_le_gap_connect ){
                                ble_rsp_le_gap_connect( (const struct ble_msg_le_gap_connect_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_LE_GAP_DISCOVER
                        else if( bgapiRXBuffer[3] == RSP_LE_GAP_DISCOVER_ID ){
                            if( ble_rsp_le_gap_discover ){
                                ble_rsp_le_gap_discover( (const struct ble_msg_le_gap_discover_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_LE_GAP_END_PROCEDURE
                        else if( bgapiRXBuffer[3] == RSP_LE_GAP_PROCEDURE_ID ){
                            if( ble_rsp_le_gap_end_procedure ){
                                ble_rsp_le_gap_end_procedure( (const struct ble_msg_le_gap_end_procedure_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_LE_GAP_OPEN
                        else if( bgapiRXBuffer[3] == RSP_LE_GAP_OPEN_ID ){
                            if( ble_rsp_le_gap_open ){
                                ble_rsp_le_gap_open( (const struct ble_msg_le_gap_open_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_LE_GAP_SET_ADV_DATA
                        else if( bgapiRXBuffer[3] == RSP_LE_GAP_SET_ADV_DATA_ID ){
                            if( ble_rsp_le_gap_set_adv_data ){
                                ble_rsp_le_gap_set_adv_data( (const struct ble_msg_le_gap_set_adv_data_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_LE_GAP_SET_ADV_PARAMETERS
                        else if( bgapiRXBuffer[3] == RSP_LE_GAP_SET_ADV_PARAMETERS_ID ){
                            if( ble_rsp_le_gap_set_adv_parameters ){
                                ble_rsp_le_gap_set_adv_parameters( (const struct ble_msg_le_gap_set_adv_parameters_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_LE_GAP_SET_ADV_TIMEOUT
                        else if( bgapiRXBuffer[3] == RSP_LE_GAP_SET_ADV_TIMEOUT_ID ){
                            if( ble_rsp_le_gap_set_adv_timeout ){
                                ble_rsp_le_gap_set_adv_timeout( (const struct ble_msg_le_gap_set_adv_timeout_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_LE_GAP_SET_ADVERTISE_CHANNEL_MAP
                        else if( bgapiRXBuffer[3] == RSP_LE_GAP_SET_ADVERTISE_CHANNEL_MAP_ID ){
                            if( ble_rsp_le_gap_set_advertise_channel_map ){
                                ble_rsp_le_gap_set_advertise_channel_map( (const struct ble_msg_le_gap_set_advertise_channel_map_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_LE_GAP_SET_ADVERTISE_CONFIGURATION
                        else if( bgapiRXBuffer[3] == RSP_LE_GAP_SET_ADVERTISE_CONFIGURATION_ID ){
                            if( ble_rsp_le_gap_set_advertise_configuration ){
                                ble_rsp_le_gap_set_advertise_configuration( (const struct ble_msg_le_gap_set_advertise_configuration_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_LE_GAP_SET_ADVERTISE_PHY
                        else if( bgapiRXBuffer[3] == RSP_LE_GAP_SET_ADVERTISE_PHY_ID ){
                            if( ble_rsp_le_gap_set_advertise_phy ){
                                ble_rsp_le_gap_set_advertise_phy( (const struct ble_msg_le_gap_set_advertise_phy_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_LE_GAP_SET_ADVERTISE_REPORT_SCAN_REQUEST
                        else if( bgapiRXBuffer[3] == RSP_LE_GAP_SET_ADVERTISE_REPORT_SCAN_REQUEST_ID ){
                            if( ble_rsp_le_gap_set_advertise_report_scan_request ){
                                ble_rsp_le_gap_set_advertise_report_scan_request( (const struct ble_msg_le_gap_set_advertise_report_scan_request_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_LE_GAP_SET_ADVERTISE_TIMING
                        else if( bgapiRXBuffer[3] == RSP_LE_GAP_SET_ADVERTISE_TIMING_ID ){
                            if( ble_rsp_le_gap_set_advertise_timing ){
                                ble_rsp_le_gap_set_advertise_timing( (const struct ble_msg_le_gap_set_advertise_timing_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_LE_GAP_SET_CONN_PARAMEETERS
                        else if( bgapiRXBuffer[3] == RSP_LE_GAP_SET_CONN_PARAMETERS_ID ){
                            if( ble_rsp_le_gap_set_conn_parameters ){
                                ble_rsp_le_gap_set_conn_parameters( (const struct ble_msg_le_gap_set_conn_parameters_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_LE_GAP_SET_DATA_CHANNEL_CLASSIFICATION
                        else if( bgapiRXBuffer[3] == RSP_LE_GAP_SET_DATA_CHANNEL_CLASSIFICATION_ID ){
                            if( ble_rsp_le_gap_set_data_channel_classification ){
                                ble_rsp_le_gap_set_conn_parameters( (const struct ble_msg_le_gap_set_conn_parameters_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                       }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_LE_GAP_SET_DISCOVERY_TIMING
                        else if( bgapiRXBuffer[3] == RSP_LE_GAP_SET_DISCOVERY_TIMING_ID ){
                            if( ble_rsp_le_gap_set_discovery_timing ){
                                ble_rsp_le_gap_set_discovery_timing( (const struct ble_msg_le_gap_set_discovery_timing_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_LE_GAP_SET_DISCOVERY_TYPE
                        else if( bgapiRXBuffer[3] == RSP_LE_GAP_SET_DISCOVERY_TYPE_ID ){
                            if( ble_rsp_le_gap_set_discovery_type ){
                                ble_rsp_le_gap_set_discovery_type( (const struct ble_msg_le_gap_set_discovery_type_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_LE_GAP_SET_MODE
                        else if( bgapiRXBuffer[3] == RSP_LE_GAP_SET_MODE_ID ){
                            if( ble_rsp_le_gap_set_mode ){
                                ble_rsp_le_gap_set_mode( (const struct ble_msg_le_gap_set_mode_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_LE_GAP_SET_PRIVACY_MODE
                        else if( bgapiRXBuffer[3] == RSP_LE_GAP_SET_PRIVACY_MODE_ID ){
                            if( ble_rsp_le_gap_set_privacy_mode ){
                                ble_rsp_le_gap_set_privacy_mode( (const struct ble_msg_le_gap_set_privacy_mode_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_LE_GAP_SET_SCAN_PARAMETERS
                        else if( bgapiRXBuffer[3] == RSP_LE_GAP_SET_SCAN_PARAMETERS_ID ){
                            if( ble_rsp_le_gap_set_scan_parameters ){
                                ble_rsp_le_gap_set_scan_parameters( (const struct ble_msg_le_gap_set_scan_parameters_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_LE_GAP_START_ADVERTISING
                        else if( bgapiRXBuffer[3] == RSP_LE_GAP_START_ADVERTISING_ID ){
                            if( ble_rsp_le_gap_start_advertising ){
                                ble_rsp_le_gap_start_advertising( (const struct ble_msg_le_gap_start_advertising_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_LE_GAP_START_DISCOVERY
                        else if( bgapiRXBuffer[3] == RSP_LE_GAP_START_DISCOVERY_ID ){
                            if( ble_rsp_le_gap_start_discovery ){
                                ble_rsp_le_gap_start_discovery( (const struct ble_msg_le_gap_start_discovery_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_LE_GAP_STOP_ADVERTISING
                        else if( bgapiRXBuffer[3] == RSP_LE_GAP_STOP_ADVERTISING_ID ){
                            if( ble_rsp_le_gap_stop_advertising ){
                                ble_rsp_le_gap_stop_advertising( (const struct ble_msg_le_gap_stop_advertising_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                }
                else if( bgapiRXBuffer[2] == MESSAGE_CLASS_SM )
                {
                    /* [SM] Security Manager */
                    if (false) { }
                    #ifdef BGLIB_ENABLE_CMD_SM_BONDING_CONFIRM
                        else if( bgapiRXBuffer[3] == RSP_SM_BONDING_CONFIRM_ID ){
                            if( ble_rsp_sm_bonding_confirm ){
                                ble_rsp_sm_bonding_confirm( (const struct ble_msg_sm_bonding_confirm_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_SM_CONFIGURE
                        else if( bgapiRXBuffer[3] == RSP_SM_CONFIGURE_ID ){
                            if( ble_rsp_sm_configure ){
                                ble_rsp_sm_configure( (const struct ble_msg_sm_configure_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_SM_DELETE_BONDING
                        else if( bgapiRXBuffer[3] == RSP_SM_DELETE_BONDING_ID ){
                            if( ble_rsp_sm_delete_bonding ){
                                ble_rsp_sm_delete_bonding( (const struct ble_msg_sm_delete_bonding_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_SM_DELETE_BONDINGS
                        else if( bgapiRXBuffer[3] == RSP_SM_DELETE_BONDINGS_ID ){
                            if( ble_rsp_sm_delete_bondings ){
                                ble_rsp_sm_delete_bondings( (const struct ble_msg_sm_delete_bondings_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_SM_ENTER_PASSKEY
                        else if( bgapiRXBuffer[3] == RSP_SM_ENTER_PASSKEY_ID ){
                            if( ble_rsp_sm_enter_passkey ){
                                ble_rsp_sm_enter_passkey( (const struct ble_msg_sm_enter_passkey_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_SM_INCREASE_SECURITY
                        else if( bgapiRXBuffer[3] == RSP_SM_INCREASE_SECURITY_ID ){
                            if( ble_rsp_sm_increase_security ){
                                ble_rsp_sm_increase_security( (const struct ble_msg_sm_increase_security_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_SM_LIST_ALL_BONDINGS
                        else if( bgapiRXBuffer[3] == RSP_SM_LIST_ALL_BONDINGS_ID ){
                            if( ble_rsp_sm_list_all_bondings ){
                                ble_rsp_sm_list_all_bondings( (const struct ble_msg_sm_list_all_bondings_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_SM_PASSKEY_CONFIRM
                        else if( bgapiRXBuffer[3] == RSP_SM_PASSKEY_CONFIRM_ID ){
                            if( ble_rsp_sm_passkey_confirm ){
                                ble_rsp_sm_passkey_confirm( (const struct ble_msg_sm_passkey_confirm_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_SM_SET_BONDABLE_MODE
                        else if( bgapiRXBuffer[3] == RSP_SM_SET_BONDABLE_MODE_ID ){
                            if( ble_rsp_sm_set_bondable_mode ){
                                ble_rsp_sm_set_bondable_mode( (const struct ble_msg_sm_set_bondable_mode_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_SM_SET_DEBUG_MODE
                        else if( bgapiRXBuffer[3] == RSP_SM_SET_DEBUG_MODE_ID ){
                            if( ble_rsp_sm_set_debug_mode ){
                                ble_rsp_sm_set_debug_mode( (const struct ble_msg_sm_set_debug_mode_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_SM_SET_OOB_DATA
                        else if( bgapiRXBuffer[3] == RSP_SM_SET_OOB_DATA_ID ){
                            if( ble_rsp_sm_set_oob_data ){
                                ble_rsp_sm_set_oob_data( (const struct ble_msg_sm_set_oob_data_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_SM_SET_PASSKEY
                        else if( bgapiRXBuffer[3] == RSP_SM_SET_PASSKEY_ID ){
                            if( ble_rsp_sm_set_passkey ){
                                ble_rsp_sm_set_passkey( (const struct ble_msg_sm_set_passkey_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_SM_SET_SC_REMOTE_OOB_DATA
                        else if( bgapiRXBuffer[3] == RSP_SM_SET_SC_REMOTE_OOB_DATA_ID ){
                            if( ble_rsp_sm_set_sc_remote_oob_data ){
                                ble_rsp_sm_set_sc_remote_oob_data( (const struct ble_msg_sm_set_sc_remote_oob_data_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_SM_STORE_BONDING_CONFIGURATION
                        else if( bgapiRXBuffer[3] == RSP_SM_STORE_BONDING_CONFIGURATION_ID ){
                            if( ble_rsp_sm_store_bonding_configuration ){
                                ble_rsp_sm_store_bonding_configuration( (const struct ble_msg_sm_store_bonding_configuration_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_SM_USE_SC_OOB
                        else if( bgapiRXBuffer[3] == RSP_SM_USE_SC_OOB_ID ){
                            if( ble_rsp_sm_use_sc_oob ){
                                ble_rsp_sm_use_sc_oob( (const struct ble_msg_sm_use_sc_oob_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                }
                else if( bgapiRXBuffer[2] == MESSAGE_CLASS_SYSTEM )
                {
                    /* [system] System */
                    if( false ){ }
                    #ifdef BGLIB_ENABLE_CMD_SYSTEM_GET_BT_ADDRESS
                        else if( bgapiRXBuffer[3] == RSP_SYSTEM_GET_BT_ADDRES_ID ){
                            if( ble_rsp_system_get_bt_address ){
                                ble_rsp_system_get_bt_address( (const struct ble_msg_system_get_bt_address_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_SYSTEM_GET_COUNTERS
                        else if( bgapiRXBuffer[3] == RSP_SYSTEM_GET_COUNTERS_ID ){
                            if( ble_rsp_system_get_counters ){
                                ble_rsp_system_get_counters( (const struct ble_msg_system_get_counters_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_SYSTEM_GET_RANDOM_DATA
                        else if( bgapiRXBuffer[3] == RSP_SYSTEM_GET_RANDOM_DATA_ID ){
                            if( ble_rsp_system_get_random_data ){
                                ble_rsp_system_get_random_data( (const struct ble_msg_system_get_random_data_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_SYSTEM_HALT
                        else if( bgapiRXBuffer[3] == RSP_SYSTEM_HALT_ID ){
                            if( ble_rsp_system_halt ){
                                ble_rsp_system_halt( (const struct ble_msg_system_halt_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_SYSTEM_HELLO
                        else if( bgapiRXBuffer[3] == RSP_SYSTEM_HELLO_ID ){
                            if( ble_rsp_system_hello ){
                                ble_rsp_system_hello( (const struct ble_msg_system_hello_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
#if 0
                    #ifdef BGLIB_ENABLE_CMD_SYSTEM_RESET
                        else if( bgapiRXBuffer[3] == 0x01 ){
                            if( ble_rsp_system_reset ) ble_rsp_system_reset((const struct ble_msg_system_reset_rsp_t *)(bgapiRXBuffer + 4));
                        }
                    #endif
#endif
                    #ifdef BGLIB_ENABLE_CMD_SYSTEM_SET_BT_ADDRESS
                        else if( bgapiRXBuffer[3] == RSP_SYSTEM_SET_BT_ADDRESS_ID ){
                            if( ble_rsp_system_set_bt_address ) {
                                ble_rsp_system_set_bt_address( (const struct ble_msg_system_set_bt_address_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_SYSTEM_SET_DEVICE_NAME
                        else if( bgapiRXBuffer[3] == RSP_SYSTEM_SET_DEVICE_NAME_ID ){
                            if( ble_rsp_system_set_device_name ){
                                ble_rsp_system_set_device_name( (const struct ble_msg_system_set_device_name_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_SYSTEM_SET_TX_POWER
                        else if( bgapiRXBuffer[3] == RSP_SYSTEM_SET_TX_POWER_ID ){
                            if( ble_rsp_system_set_tx_power ){
                                ble_rsp_system_set_tx_power( (const struct ble_msg_system_set_tx_power_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                }
                else if( bgapiRXBuffer[2] == MESSAGE_CLASS_TEST )
                {
                    /* [test] testing */
                    if (false) { }
                    #ifdef BGLIB_ENABLE_CMD_TEST_DTM_END
                        else if( bgapiRXBuffer[3] == RSP_TEST_DTM_END_ID ){
                            if( ble_rsp_test_dtm_end ){
                                ble_rsp_test_dtm_end( (const struct ble_msg_test_dtm_end_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_TEST_DTM_RX
                        else if( bgapiRXBuffer[3] == RSP_TEST_DTM_RX_ID ){
                            if( ble_rsp_test_dtm_rx ){
                                ble_rsp_test_dtm_rx( (const struct ble_msg_test_dtm_rx_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_CMD_TEST_DTM_TX
                        else if( bgapiRXBuffer[3] == RSP_TEST_DTM_TX_ID ){
                            if( ble_rsp_test_dtm_tx ){
                                ble_rsp_test_dtm_tx( (const struct ble_msg_test_dtm_tx_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                }
                else if( bgapiRXBuffer[2] == MESSAGE_CLASS_USER )
                {
                    /* [user] User message */
                    if (false) { }
                    #ifdef BGLIB_ENABLE_CMD_USER_MESSAGE_TO_TARGET
                        else if( bgapiRXBuffer[3] == RSP_USER_MESSAGE_TO_TARGET_ID ){
                            if( ble_rsp_user_message_to_target ){
                                ble_rsp_user_message_to_target( (const struct ble_msg_user_message_to_target_rsp_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                }
                /*  */
                setBusy(false);
            }
            else
            {
                // 0x80 = Event packet
                // 0xA0 = Event packet
                // capture last event class/command bytes
                lastEvent[0] = bgapiRXBuffer[2];
                lastEvent[1] = bgapiRXBuffer[3];
#if 0
                else if( bgapiRXBuffer[2] == MESSAGE_CLASS_COEX )
                {
                    /* Cooexistence interface */
                    if (false) { }
                }
#endif
                if( bgapiRXBuffer[2] == MESSAGE_CLASS_DFU )
                {
                    /* [dfu] Device Firmware Upgrade */
                    if (false) { }
                    /* [dfu] Device Firmware Upgrade */
                    #ifdef BGLIB_ENABLE_EVT_DFU_BOOT
                        else if( bgapiRXBuffer[3] == EVT_DFU_BOOT_ID ){
                            if( ble_evt_dfu_boot ){
                                ble_evt_dfu_boot( (const struct ble_msg_dfu_boot_evt_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_EVT_DFU_BOOT_FAILURE
                        else if( bgapiRXBuffer[3] == EVT_DFU_BOOT_FAILURE_ID ){
                            if( ble_evt_dfu_boot_failure ){
                                ble_evt_dfu_boot_failure( (const struct ble_msg_dfu_boot_failure_evt_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                }
#if 0
                else if( bgapiRXBuffer[2] == MESSAGE_CLASS_ENDPOINT )
                {
                    /* Endpoint */
                    if (false) { }
                }
#endif
#if 0
                else if( bgapiRXBuffer[2] == MESSAGE_CLASS_FLASH )
                {
                    /* Persistent Store */
                    if (false) { }
                }
#endif
                else if( bgapiRXBuffer[2] == MESSAGE_CLASS_GATT )
                {
                    /* Generic Attribute Profile */
                    if (false) { }
                    #ifdef BGLIB_ENABLE_EVT_GATT_CHARACTERISTIC
                        else if( bgapiRXBuffer[3] == EVT_GATT_CHARACTERISTIC_ID ){
                            if( ble_evt_gatt_characteristic ){
                                ble_evt_gatt_characteristic( (const struct ble_msg_gatt_characteristic_evt_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_EVT_GATT_CHARACTERISTIC_VALUE
                        else if( bgapiRXBuffer[3] == EVT_GATT_CHARACTERISTIC_VALUE_ID ){
                            if( ble_evt_gatt_characteristic_value ){
                                ble_evt_gatt_characteristic_value( (const struct ble_msg_gatt_characteristic_value_evt_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_EVT_GATT_DESCRIPTOR
                        else if( bgapiRXBuffer[3] == EVT_GATT_DESCRIPTOR_ID ){
                            if( ble_evt_gatt_descriptor ){
                                ble_evt_gatt_descriptor( (const struct ble_msg_gatt_descriptor_evt_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_EVT_GATT_DESCRIPTOR_VALUE
                        else if( bgapiRXBuffer[3] == EVT_GATT_DESCRIPTOR_VALUE_ID ){
                            if( ble_evt_gatt_descriptor_value ){
                                ble_evt_gatt_descriptor_value( (const struct ble_msg_gatt_descriptor_value_evt_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_EVT_GATT_MTU_EXCHANGED
                        else if( bgapiRXBuffer[3] == EVT_GATT_MTU_EXCHANGED_ID ){
                            if( ble_evt_gatt_mtu_exchanged ){
                                ble_evt_gatt_mtu_exchanged( (const struct ble_msg_gatt_mtu_exchanged_evt_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_EVT_GATT_PROCEDURE_COMPLETED
                        else if( bgapiRXBuffer[3] == EVT_GATT_PROCEDURE_COMPLETED_ID ){
                            if( ble_evt_gatt_procedure_completed ){
                                ble_evt_gatt_procedure_completed( (const struct ble_msg_gatt_procedure_completed_evt_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_EVT_GATT_SERVICE
                        else if( bgapiRXBuffer[3] == EVT_GATT_SERVICE_ID ){
                            if( ble_evt_gatt_service ){
                                ble_evt_gatt_service( (const struct ble_msg_gatt_service_evt_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                }
                else if( bgapiRXBuffer[2] == MESSAGE_CLASS_GATT_SERVER )
                {
                    /* [gatt_server] Generic Attribure Profile Server */
                    if (false) { }
                    #ifdef BGLIB_ENABLE_EVT_GATT_SERVER_ATTRIBUTE_VALUE
                        else if( bgapiRXBuffer[3] == EVT_GATT_SERVER_ATTRIBUTE_VALUE_ID ){
                            if( ble_evt_gatt_server_attribute_value ){
                                ble_evt_gatt_server_attribute_value( (const struct ble_msg_gatt_server_attribute_value_evt_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_EVT_GATT_SERVER_CHARACTERISTIC_STATUS
                        else if( bgapiRXBuffer[3] == EVT_GATT_SERVER_CHARACTERISTIC_STATUS_ID ){
                            if( ble_evt_gatt_server_characteristic_status ){
                                ble_evt_gatt_server_characteristic_status( (const struct ble_msg_gatt_server_characteristic_status_evt_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_EVT_GATT_SERVER_EXECUTE_WRITE_COMPLETED
                        else if( bgapiRXBuffer[3] == EVT_GATT_SERVER_EXECUTE_WRITE_COMPLETED_ID ){
                            if( ble_evt_gatt_server_execute_write_completed ){
                                ble_evt_gatt_server_execute_write_completed( (const struct ble_msg_gatt_server_execute_write_completed_evt_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_EVT_GATT_SERVER_USER_READ_REQUEST
                        else if( bgapiRXBuffer[3] == EVT_GATT_SERVER_USER_READ_REQUEST_ID ){
                            if( ble_evt_gatt_server_user_read_request ){
                                ble_evt_gatt_server_user_read_request( (const struct ble_msg_gatt_server_user_read_request_evt_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_EVT_GATT_SERVER_USER_WRITE_REQUEST
                        else if( bgapiRXBuffer[3] == EVT_GATT_SERVER_USER_WRITE_REQUEST_ID ){
                            if( ble_evt_gatt_server_user_write_request ){
                                ble_evt_gatt_server_user_write_request( (const struct ble_msg_gatt_server_user_write_request_evt_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                }
                else if( bgapiRXBuffer[2] == MESSAGE_CLASS_HARDWARE )
                {
                    /* [hardware] Hardware */
                    if (false) { }
                    #ifdef BGLIB_ENABLE_EVT_HARDWARE_SOFT_TIMER
                        else if( bgapiRXBuffer[3] == EVT_GATT_HARDWARE_SOFT_TIMER_ID ){
                            if( ble_evt_hardware_soft_timer ){
                                ble_evt_hardware_soft_timer( (const struct ble_msg_hardware_soft_timer_evt_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                }
                else if( bgapiRXBuffer[2] == MESSAGE_CLASS_LE_CONNECTION )
                {
                    /* Connection management */
                    if (false) { }
                    #ifdef BGLIB_ENABLE_EVT_LE_CONNECTION_CLOSED
                        else if( bgapiRXBuffer[3] == EVT_LE_CONNECTION_CLOSED_ID ){
                            if( ble_evt_le_connection_closed ){
                                ble_evt_le_connection_closed( (const struct ble_msg_le_connection_closed_evt_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_EVT_LE_CONNECTION_OPENED
                        else if( bgapiRXBuffer[3] == EVT_LE_CONNECTION_OPENED_ID ){
                            if( ble_evt_le_connection_opend ){
                                ble_evt_le_connection_opend( (const struct ble_msg_le_connection_opend_evt_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_EVT_LE_CONNECTION_PARAMETERS
                        else if( bgapiRXBuffer[3] == EVT_LE_CONNECTION_PARAMETERS_ID ){
                            if( ble_evt_le_connection_parameters ){
                                ble_evt_le_connection_parameters( (const struct ble_msg_le_connection_parameters_evt_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_EVT_LE_CONNECTION_PHY_STATUS
                        else if( bgapiRXBuffer[3] == EVT_LE_CONNECTION_PHY_STATUS_ID ){
                            if( ble_evt_le_connection_phy_status ){
                                ble_evt_le_connection_phy_status( (const struct ble_msg_le_connection_phy_status_evt_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_EVT_LE_CONNECTION_RSSI
                        else if( bgapiRXBuffer[3] == EVT_LE_CONNECTION_RSSI_ID ){
                            if( ble_evt_le_connection_rssi ){
                                ble_evt_le_connection_rssi( (const struct ble_msg_le_connection_rssi_evt_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                }
                else if( bgapiRXBuffer[2] == MESSAGE_CLASS_LE_GAP )
                {
                    /* [le gap] Generic Access Profile events */
                    if (false) { }
                    #ifdef BGLIB_ENABLE_EVT_LE_GAP_ADV_TIMEOUT
                        else if( bgapiRXBuffer[3] == EVT_LE_GAP_ADV_TIMEOUT_ID ){
                            if( ble_evt_le_gap_adv_timeout ){
                                ble_evt_le_gap_adv_timeout( (const struct ble_msg_le_gap_adv_timeout_evt_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_EVT_LE_GAP_SCAN_REQUEST
                        else if( bgapiRXBuffer[3] == EVT_LE_GAP_SCAN_REQUEST_ID ){
                            if( ble_evt_le_gap_scan_request ){
                                ble_evt_le_gap_scan_request( (const struct ble_msg_le_gap_scan_request_evt_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_EVT_LE_GAP_SCAN_RESPONSE
                        else if( bgapiRXBuffer[3] == EVT_LE_GAP_SCAN_RESPONSE_ID ){
                            if( ble_evt_le_gap_scan_response ){
                                ble_evt_le_gap_scan_response( (const struct ble_msg_le_gap_scan_response_evt_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                }
                else if( bgapiRXBuffer[2] == MESSAGE_CLASS_SM )
                {
                    /* Security Manager */
                    if (false) { }
                    #ifdef BGLIB_ENABLE_EVT_SM_BONDED
                        else if( bgapiRXBuffer[3] == EVT_SM_BONDED_ID ){
                            if( ble_evt_sm_bonded ){
                                ble_evt_sm_bonded( (const struct ble_msg_sm_bonded_evt_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_EVT_SM_BONDING_FAILED
                        else if( bgapiRXBuffer[3] == EVT_SM_BONDING_FAILED_ID ){
                            if( ble_evt_sm_bonding_failed ){
                                ble_evt_sm_bonding_failed( (const struct ble_msg_sm_bonding_failed_evt_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_EVT_SM_CONFIRM_BONDING
                        else if( bgapiRXBuffer[3] == EVT_SM_CONFIRM_BONDING_ID ){
                            if( ble_evt_sm_confirm_bonding ){
                                ble_evt_sm_confirm_bonding( (const struct ble_msg_sm_confirm_bonding_evt_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_EVT_SM_CONFIRM_PASSKEY
                        else if( bgapiRXBuffer[3] == EVT_SM_CONFIRM_PASSKEY_ID ){
                            if( ble_evt_sm_confirm_passkey ){
                                ble_evt_sm_confirm_passkey( (const struct ble_msg_sm_confirm_passkey_evt_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_EVT_SM_LIST_ALL_BONDINGS_COMPLETE
                        else if( bgapiRXBuffer[3] == EVT_SM_LIST_ALL_BONDINGS_COMPLETE_ID ){
                            if( ble_evt_sm_list_all_bondings_complete ){
                                ble_evt_sm_list_all_bondings_complete( (const struct ble_msg_sm_list_all_bondings_complete_evt_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_EVT_SM_LIST_BONDING_ENTRY
                        else if( bgapiRXBuffer[3] == EVT_SM_LIST_BONDING_ENTRY_ID ){
                            if( ble_evt_sm_list_bonding_entry ){
                                ble_evt_sm_list_bonding_entry( (const struct ble_msg_sm_list_bonding_entry_evt_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_EVT_SM_PASSKEY_DISPLAY
                        else if( bgapiRXBuffer[3] == EVT_SM_PASSKEY_DISPLAY_ID ){
                            if( ble_evt_sm_passkey_display ){
                                ble_evt_sm_passkey_display( (const struct ble_msg_sm_passkey_display_evt_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_EVT_SM_PASSKEY_REQUEST
                        else if( bgapiRXBuffer[3] == EVT_SM_PASSKEY_REQUEST_ID ){
                            if( ble_evt_sm_passkey_request ){
                                ble_evt_sm_passkey_request( (const struct ble_msg_sm_passkey_request_evt_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                }
                else if( bgapiRXBuffer[2] == MESSAGE_CLASS_SYSTEM )
                {
                    /* [system] System */
                    if (false) { }
                    #ifdef BGLIB_ENABLE_EVT_SYSTEM_AWAKE
                        else if( bgapiRXBuffer[3] == EVT_SYSTEM_AWAKE_ID ){
                            if( ble_evt_system_awake ) {
                                ble_evt_system_awake();
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_EVT_SYSTEM_BOOT
                        else if( bgapiRXBuffer[3] == EVT_SYSTEM_BOOT_ID ){
                            if( ble_evt_system_boot ) {
                                ble_evt_system_boot((const struct ble_msg_system_boot_evt_t *)(bgapiRXBuffer + 4));
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_EVT_SYSTEM_ERROR
                        else if( bgapiRXBuffer[3] == EVT_SYSTEM_ERROR_ID ){
                            if( ble_evt_system_error ) {
                                ble_evt_system_error((const struct ble_msg_system_error_evt_t *)(bgapiRXBuffer + 4));
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_EVT_SYSTEM_EXTERNAL_SIGNAL
                        else if( bgapiRXBuffer[3] == EVT_SYSTEM_EXTERNAL_SIGNAL_ID ){
                            if( ble_evt_system_error ) {
                                ble_evt_system_error((const struct ble_msg_system_error_evt_t *)(bgapiRXBuffer + 4));
                            }
                        }
                    #endif
                    #ifdef BGLIB_ENABLE_EVT_SYSTEM_HARDWARE_ERROR
                        else if( bgapiRXBuffer[3] == EVT_SYSTEM_HARDWARE_ERROR_ID ){
                            if( ble_evt_system_hardware_error ) {
                                ble_evt_system_hardware_error((const struct ble_msg_system_hardware_error_evt_t *)(bgapiRXBuffer + 4));
                            }
                        }
                    #endif
                }
                else if( bgapiRXBuffer[2] == MESSAGE_CLASS_TEST )
                {
                    /* testing commands */
                    if (false) { }
                    #ifdef BGLIB_ENABLE_EVT_TEST_DTM_COMPLETED
                        else if( bgapiRXBuffer[3] == EVT_TEST_DTM_COMPLETED_ID ){
                            if( ble_evt_test_dtm_completed ){
                                ble_evt_test_dtm_completed( (const struct ble_msg_test_dtm_completed_evt_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                }
                else if( bgapiRXBuffer[2] == MESSAGE_CLASS_USER )
                {
                    /* User messaging */
                    if (false) { }
                    #ifdef BGLIB_ENABLE_EVT_USER_MASSAGE_TO_HOST
                        else if( bgapiRXBuffer[3] == EVT_USER_MESSAGE_TO_HOST_ID ){
                            if( ble_evt_user_message_to_host ){
                                ble_evt_user_message_to_host( (const struct ble_msg_user_message_to_host_evt_t *)(bgapiRXBuffer + 4) );
                            }
                        }
                    #endif
                }
                /*  */
            }
        }
    }

    return 0;       // parsed successfully
}


uint8_t BGLib::sendCommand( uint16_t len, uint8_t commandClass, uint8_t commandId, void *payload )
{
    bgapiTXBuffer = (uint8_t *)malloc( len + 4 );
//    bgapiTXBuffer[0] = 0x00;                            /* [要確認] */
    bgapiTXBuffer[0] = 0x20;                            /* [要確認] */
    bgapiTXBuffer[1] = (len & 0xFF);
    bgapiTXBuffer[2] = commandClass;
    bgapiTXBuffer[3] = commandId;
    lastCommand[0] = commandClass;
    lastCommand[1] = commandId;
    /*  */
    if( len > 0) memcpy( bgapiTXBuffer + 4, payload, len );
    #ifdef BGLIB_DEBUG
        SerialUSB.print( "\n=>[ " );
        if( packetMode )
        {
            if( len + 4 < 16 ) SerialUSB.write( 0x30 );
            SerialUSB.print( len + 4, HEX );
            SerialUSB.write( 0x20 );
        }
        for( uint8_t i = 0; i < len + 4; i++ )
        {
            if( bgapiTXBuffer[i] < 16 ) SerialUSB.write( 0x30 );
            SerialUSB.print( bgapiTXBuffer[i], HEX );
            SerialUSB.write( 0x20 );                    /* space */
        }
        SerialUSB.println( "]" );
    #endif
//    SerialUSB.println( "onBeforeTXCommand" );             /* [moto] */
//    if( onBeforeTXCommand ) onBeforeTXCommand();      // [要確認] 戻ってこなくなるので保留とする(WAKEUPと関連)
//    SerialUSB.println( "setBusy" );                       /* [moto] */
    setBusy( true );
//    SerialUSB.println( "write" );                             /* [moto] */
    if( packetMode ) uModule -> write( len + 4 );       // outgoing packet length byte first
    uModule -> write( bgapiTXBuffer, len + 4 );

    if( onTXCommandComplete ) onTXCommandComplete();
//    SerialUSB.println( "free" );                              /* [moto] */
    free( bgapiTXBuffer );
//    SerialUSB.println( "end" );                               /* [moto] */
    return 0;
}




/* [coex] cooexistence interface */
#ifdef BGLIB_ENABLE_CMD_COEX_GET_COUNTERS
uint8_t BGLib::ble_cmd_coex_get_counters( uint8 reset ) {
    uint8_t d[1];
    memcpy(d + 0, &reset, sizeof(uint8));
    return sendCommand( 1, 0x20, 1, d );       /* [1:len]1, [2:commandClass]0x20, [3:commandId]1, [4]d */
}
#endif
#ifdef BGLIB_ENABLE_CMD_COEX_SET_OPTIONS
uint8_t BGLib::ble_cmd_coex_set_options( uint32 mask, uint32 options ) {
    uint8_t d[8];
    memcpy(d + 0, &mask, sizeof(uint32));
    memcpy(d + 4, &options, sizeof(uint32));
    return sendCommand( 8, 0x20, 0, d );       /* [1:len]8, [2:commandClass]0x20, [3:commandId]0, [4]d */
}
#endif
/*  */


/* [dfu] Device Firmware Upgrade */
#ifdef BGLIB_ENABLE_CMD_DFU_FLASH_SET_ADDRESS
uint8_t BGLib::ble_cmd_dfu_flash_set_address( uint32 address ) {
    uint8_t d[4];
    memcpy(d + 0, &address, sizeof(uint32));
    return sendCommand( 4, 0x00, 1, d );       /* [1:len]4, [2:commandClass]0x00, [3:commandId]1, [4]d */
}
#endif
#ifdef BGLIB_ENABLE_CMD_DFU_FLASH_UPLOAD
uint8_t BGLib::ble_cmd_dfu_flash_upload( uint8 data_len, const uint8 *data_data ) {
    uint8_t d[1 + data_len];
    memcpy(d + 0, &data_len, sizeof(uint8));
    memcpy(d + 1, data_data, data_len);
    return sendCommand( (1 + data_len), 0x00, 2, d );       /* [1:len]1, [2:commandClass]0x00, [3:commandId]2, [4]d */
}
#endif
#ifdef BGLIB_ENABLE_CMD_DFU_FLASH_UPLOAD_FINISH
uint8_t BGLib::ble_cmd_dfu_flash_upload_finish( void ) {
    return sendCommand( 0, 0x00, 0 );       /* [1:len]0, [2:commandClass]0x00, [3:commandId]0 */
}
#endif
#ifdef BGLIB_ENABLE_CMD_DFU_RESET
uint8_t BGLib::ble_cmd_dfu_reset( uint8 dfu ) {
    uint8_t d[1];
    memcpy(d + 0, &dfu, sizeof(uint8));
    return sendCommand( 1, 0x00, 0, d );       /* [1:len]0, [2:commandClass]0x00, [3:commandId]0, [4]d */
}
#endif
/*  */


/* [endpoint] Endpoint */
#ifdef BGLIB_ENABLE_CMD_ENDPOINT_CLOSE
uint8_t BGLib::ble_cmd_endpoint_close( uint8 endpoint ) {
    uint8_t d[1];
    memcpy(d + 0, &endpoint, sizeof(uint8));
    return sendCommand( 1, 0x0B, 2, d );       /* [1:len]1, [2:commandClass]B, [3:commandId]2, [4]d */
}
#endif
/*  */


/* [flash] Peripheral Store */
#ifdef BGLIB_ENABLE_CMD_FLASH_PS_ERASE
uint8_t BGLib::ble_cmd_flash_ps_erase( uint16 key ) {
    uint8_t d[2];
    memcpy(d + 0, &key, sizeof(uint16));
    return sendCommand( 2, 0x0D, 4, d );       /* [1:len]2, [2:commandClass]D, [3:commandId]4, [4]d */
}
#endif
#ifdef BGLIB_ENABLE_CMD_FLASH_PS_ERASE_ALL
uint8_t BGLib::ble_cmd_flash_ps_erase_all( void ) {
    return sendCommand( 0, 0x0D, 0x0D);       /* [1:len]0, [2:commandClass]D, [3:commandId]D */
}
#endif
#ifdef BGLIB_ENABLE_CMD_FLASH_PS_LOAD
uint8_t BGLib::ble_cmd_flash_ps_load( uint16 key ) {
    uint8_t d[2];
    memcpy(d + 0, &key, sizeof(uint16));
    return sendCommand( 2, 0x0D, 3, d );       /* [1:len]2, [2:commandClass]D, [3:commandId]3, [4]d */
}
#endif
#ifdef BGLIB_ENABLE_CMD_FLASH_PS_SAVE
uint8_t BGLib::ble_cmd_flash_ps_save( uint16 key, uint8 value_len, const uint8 *value_data ) {
    uint8_t d[3 + value_len];
    memcpy(d + 0, &key, sizeof(uint16));
    memcpy(d + 2, &value_len, sizeof(uint8));
    memcpy(d + 3, value_data, value_len);
    return sendCommand( (3 + value_len), 0x0D, 2, d );       /* [1:len]3, [2:commandClass]D, [3:commandId]2, [4]d */
}
#endif
/*  */


/* [gatt] Generic Attribute Profile */
#ifdef BGLIB_ENABLE_CMD_GATT_DISCOVER_CHARACTERISTICS
uint8_t BGLib::ble_cmd_gatt_discover_characteristics( uint8 connection, uint32 service ) {
    uint8_t d[5];
    memcpy(d + 0, &connection, sizeof(uint8));
    memcpy(d + 1, &service, sizeof(uint32));
    return sendCommand( 5, 0x09, 3, d );       /* [1:len]5, [2:commandClass]9, [3:commandId]3, [4]d */
}
#endif
#ifdef BGLIB_ENABLE_CMD_GATT_DISCOVER_CHARACTERISTICS_BY_UUID
uint8_t BGLib::ble_cmd_gatt_discover_characteristics_by_uuid( uint8 connection, uint32 service, uint8 uuid_len, const uint8 *uuid_data ) {
    uint8_t d[6 + uuid_len];
    memcpy(d + 0, &connection, sizeof(uint8));
    memcpy(d + 1, &service, sizeof(uint32));
    memcpy(d + 5, &uuid_len, sizeof(uint8));
    memcpy(d + 6, uuid_data, uuid_len);
    return sendCommand( (6 + uuid_len), 0x09, 4, d );       /* [1:len]6, [2:commandClass]9, [3:commandId]4, [4]d */
}
#endif
#ifdef BGLIB_ENABLE_CMD_GATT_DISCOVER_DESCRIPTORS
uint8_t BGLib::ble_cmd_gatt_discover_descriptors( uint8 connection, uint16 characteristic ) {
    uint8_t d[3];
    memcpy(d + 0, &connection, sizeof(uint8));
    memcpy(d + 1, &characteristic, sizeof(uint16));
    return sendCommand( 3, 0x09, 6, d );       /* [1:len]3, [2:commandClass]9, [3:commandId]6, [4]d */
}
#endif
#ifdef BGLIB_ENABLE_CMD_GATT_DISCOVER_PRIMARY_SERVICES
uint8_t BGLib::ble_cmd_gatt_discover_primary_services( uint8 connection ) {
    uint8_t d[1];
    memcpy(d + 0, &connection, sizeof(uint8));
    return sendCommand( 1, 0x09, 1, d );       /* [1:len]1, [2:commandClass]9, [3:commandId]1, [4]d */
}
#endif
#ifdef BGLIB_ENABLE_CMD_GATT_DISCOVER_PRIMARY_SERVICES_BY_UUID
uint8_t BGLib::ble_cmd_gatt_discover_primary_services_by_uuid( uint8 connection, uint8 uuid_len, const uint8 *uuid_data ) {
    uint8_t d[2 + uuid_len];
    memcpy(d + 0, &connection, sizeof(uint8));
    memcpy(d + 1, &uuid_len, sizeof(uint8));
    memcpy(d + 2, uuid_data, uuid_len);
    return sendCommand( (2 + uuid_len), 0x09, 2, d );       /* [1:len]2, [2:commandClass]9, [3:commandId]2, [4]d */
}
#endif
#ifdef BGLIB_ENABLE_CMD_GATT_EXECUTE_CHARACTERISTIC_VALUE_WRITE
uint8_t BGLib::ble_cmd_gatt_execute_characteristic_value_write( uint8 connection, uint8 flags ) {
    uint8_t d[2];
    memcpy(d + 0, &connection, sizeof(uint8));
    memcpy(d + 1, &flags, sizeof(uint8));
    return sendCommand( 2, 0x09, 0x0C, d );         /* [1:len]2, [2:commandClass]9, [3:commandId]C, [4]d */
}
#endif
#ifdef BGLIB_ENABLE_CMD_GATT_FIND_INCLUDED_SERVICES
uint8_t BGLib::ble_cmd_gatt_find_include_services( uint8 connection, uint32 service ) {
    uint8_t d[5];
    memcpy(d + 0, &connection, sizeof(uint8));
    memcpy(d + 1, &service, sizeof(uint32));
    return sendCommand( 2, 0x09, 0x10, d );         /* [1:len]2, [2:commandClass]9, [3:commandId]0x10, [4]d */
}
#endif
#ifdef BGLIB_ENABLE_CMD_GATT_PREPARE_CHARACTERISTIC_VALUE_RELIABLE_WRITE
uint8_t BGLib::ble_cmd_gatt_prepare_characteristic_value_reliable_write( uint8 connection, uint16 characteristic, uint16 offset, uint16 value_len, const uint8 *value_data ) {
    uint8_t d[6 + value_len];
    memcpy(d + 0, &connection, sizeof(uint8));
    memcpy(d + 1, &characteristic, sizeof(uint16));
    memcpy(d + 3, &offset, sizeof(uint16));
    memcpy(d + 5, &value_len, sizeof(uint8));
    memcpy(d + 6, value_data, value_len);
    return sendCommand( (6 + value_len), 0x09, 0x13, d );       /* [1:len]6, [2:commandClass]9, [3:commandId]0x13, [4]d */
}
#endif
#ifdef BGLIB_ENABLE_CMD_GATT_PREPARE_CHARACTERISTIC_VALUE_WRITE
uint8_t BGLib::ble_cmd_gatt_prepare_characteristic_value_write( uint8 connection, uint16 characteristic, uint16 offset, uint16 value_len, const uint8 *value_data ) {
    uint8_t d[6 + value_len];
    memcpy(d + 0, &connection, sizeof(uint8));
    memcpy(d + 1, &characteristic, sizeof(uint16));
    memcpy(d + 3, &offset, sizeof(uint16));
    memcpy(d + 5, &value_len, sizeof(uint8));
    memcpy(d + 6, value_data, value_len);
    return sendCommand( (6 + value_len), 0x09, 0x13, d );       /* [1:len]6, [2:commandClass]9, [3:commandId]0x13, [4]d */
}
#endif
#ifdef BGLIB_ENABLE_CMD_GATT_READ_CHARACTERISTIC_VALUE
uint8_t BGLib::ble_cmd_gatt_read_characteristic_value( uint8 connection, uint16 characteristic ) {
    uint8_t d[3];
    memcpy(d + 0, &connection, sizeof(uint8));
    memcpy(d + 1, &characteristic, sizeof(uint16));
    return sendCommand( 3, 0x09, 7, d );       /* [1:len]3, [2:commandClass]9, [3:commandId]7, [4]d */
}
#endif
#ifdef BGLIB_ENABLE_CMD_GATT_READ_CHARACTERISTIC_VALUE_BY_UUID
uint8_t BGLib::ble_cmd_gatt_read_characteristic_value_by_uuid( uint8 connection, uint32 service, uint8 uuid_len, const uint8 *uuid_data ) {
    uint8_t d[6 + uuid_len];
    memcpy(d + 0, &connection, sizeof(uint8));
    memcpy(d + 1, &service, sizeof(uint32));
    memcpy(d + 5, &uuid_len, sizeof(uint8));
    memcpy(d + 6, uuid_data, uuid_len);
    return sendCommand( (6 + uuid_len), 0x09, 0x08, d );       /* [1:len]6, [2:commandClass]9, [3:commandId]0x08, [4]d */
}
#endif
#ifdef BGLIB_ENABLE_CMD_GATT_READ_CHARACTERICTIC_VALUE_FROM_OFFSET
uint8_t BGLib::ble_cmd_gatt_read_characteristic_value_from_offset( uint8 connection, uint16 characteristic, uint16 offset, uint16 maxlen ) {
    uint8_t d[7];
    memcpy(d + 0, &connection, sizeof(uint8));
    memcpy(d + 1, &characteristic, sizeof(uint16));
    memcpy(d + 3, &offset, sizeof(uint16));
    memcpy(d + 5, &maxlen, sizeof(uint16));
    return sendCommand( 7, 0x09, 0x12, d );       /* [1:len]7, [2:commandClass]9, [3:commandId]0x12, [4]d */
}
#endif
#ifdef BGLIB_ENABLE_CMD_GATT_READ_DESCRIPTOR_VALUE
uint8_t BGLib::ble_cmd_gatt_read_descriptor_value( uint8 connection, uint16 descriptor ) {
    uint8_t d[3];
    memcpy(d + 0, &connection, sizeof(uint8));
    memcpy(d + 1, &descriptor, sizeof(uint16));
    return sendCommand( 3, 0x09, 0x0E, d );     /* [1:len]3, [2:commandClass]9, [3:commandId]E, [4]d */
}
#endif
#ifdef BGLIB_ENABLE_CMD_GATT_READ_MULTIPLE_CHARACTERISTIC_VALUES
uint8_t BGLib::ble_cmd_gatt_read_multiple_characteristic_values( uint8 connection, uint8 characteristic_list_len, const uint8 *characteristic_list_data ) {
    uint8_t d[2 + characteristic_list_len];
    memcpy(d + 0, &connection, sizeof(uint8));
    memcpy(d + 1, &characteristic_list_len, sizeof(uint8));
    memcpy(d + 2, characteristic_list_data, characteristic_list_len);
    return sendCommand( (2 + characteristic_list_len), 0x09, 0x11, d );       /* [1:len]2, [2:commandClass]9, [3:commandId]0x11, [4]d */
}
#endif
#ifdef BGLIB_ENABLE_CMD_GATT_SEND_CHARACTERICTIC_CONFIRMATION
uint8_t BGLib::ble_cmd_gatt_send_characteristic_confirmation( uint8 connection ) {
    uint8_t d[1];
    memcpy(d + 0, &connection, sizeof(uint8));
    return sendCommand( 1, 0x09, 0x0D, d );     /* [1:len]1, [2:commandClass]9, [3:commandId]D, [4]d */
}
#endif
#ifdef BGLIB_ENABLE_CMD_GATT_SET_CHARACTERISTIC_NOTIFICATION
uint8_t BGLib::ble_cmd_gatt_set_characteristic_notification( uint8 connection, uint16 characteristic, uint8 flags ) {
    uint8_t d[4];
    memcpy(d + 0, &connection, sizeof(uint8));
    memcpy(d + 1, &characteristic, sizeof(uint16));
    memcpy(d + 3, &flags, sizeof(uint8));
    return sendCommand( 4, 0x09, 5, d );       /* [1:len]4, [2:commandClass]9, [3:commandId]5, [4]d */
}
#endif
#ifdef BGLIB_ENABLE_CMD_GATT_SET_MAX_MTU
uint8_t BGLib::ble_cmd_gatt_set_max_mtu( uint16 max_mtu ) {
    uint8_t d[2];
    memcpy(d + 0, &max_mtu, sizeof(uint8));
    return sendCommand( 2, 0x09, 0, d );       /* [1:len]2, [2:commandClass]9, [3:commandId]0, [4]d */
}
#endif
#ifdef BGLIB_ENABLE_CMD_GATT_WRITE_CHARACTERISTIC_VALUE
uint8_t BGLib::ble_cmd_gatt_write_characteristic_value( uint8 connection, uint16 characteristic, uint8 value_len, const uint8 *value_data ) {
    uint8_t d[4 + value_len];
    memcpy(d + 0, &connection, sizeof(uint8));
    memcpy(d + 1, &characteristic, sizeof(uint16));
    memcpy(d + 3, &value_len, sizeof(uint8));
    memcpy(d + 4, value_data, value_len);
    return sendCommand( (4 + value_len), 0x09, 9, d );       /* [1:len]3, [2:commandClass]9, [3:commandId]9, [4]d */
}
#endif
#ifdef BGLIB_ENABLE_CMD_GATT_WRITE_CHARACTERISTIC_VALUE_WITHOUT_RESPONSE
uint8_t BGLib::ble_cmd_gatt_write_characteristic_value_without_response( uint8 connection, uint16 characteristic, uint8 value_len, const uint8 *value_data ) {
    uint8_t d[4 + value_len];
    memcpy(d + 0, &connection, sizeof(uint8));
    memcpy(d + 1, &characteristic, sizeof(uint16));
    memcpy(d + 3, &value_len, sizeof(uint8));
    memcpy(d + 4, value_data, value_len);
    return sendCommand( (4 + value_len), 0x09, 0x0A, d );   /* [1:len]3, [2:commandClass]9, [3:commandId]A, [4]d */
}
#endif
#ifdef BGLIB_ENABLE_CMD_GATT_WRITE_DESCRIPTOR_VALUE
uint8_t BGLib::ble_cmd_gatt_write_descriptor_value( uint8 connection, uint16 descriptor, uint8 value_len, const uint8 *value_data ) {
    uint8_t d[4 + value_len];
    memcpy(d + 0, &connection, sizeof(uint8));
    memcpy(d + 1, &descriptor, sizeof(uint16));
    memcpy(d + 3, &value_len, sizeof(uint8));
    memcpy(d + 4, value_data, value_len);
    return sendCommand( (4 + value_len), 0x09, 0x0F, d );   /* [1:len]3, [2:commandClass]9, [3:commandId]F, [4]d */
}
#endif
/*  */


/* [gatt_server] Generic Attribure Profile Server */
#ifdef BGLIB_ENABLE_CMD_GATT_SERVER_FIND_ATTRIBUTE
uint8_t BGLib::ble_cmd_gatt_server_find_attribute( uint16 start, uint8 type_len, const uint8 *type_data ) {
    uint8_t d[3 + type_len];
    memcpy(d + 0, &start, sizeof(uint16));
    memcpy(d + 2, &type_len, sizeof(uint8));
    memcpy(d + 3, type_data, type_len);
    return sendCommand( (3 + type_len), 0x0A, 6, d );       /* [1:len]3, [2:commandClass]A, [3:commandId]6, [4]d */
}
#endif
#ifdef BGLIB_ENABLE_CMD_GATT_SERVER_READ_ATTRIBUTE_TYPE
uint8_t BGLib::ble_cmd_gatt_server_read_attribute_type( uint16 attribute ) {
    uint8_t d[2];
    memcpy(d + 0, &attribute, sizeof(uint16));
    return sendCommand( 2, 0x0A, 1, d );                   /* [1:len]2, [2:commandClass]A, [3:commandId]1, [4]d */
}
#endif
#ifdef BGLIB_ENABLE_CMD_GATT_SERVER_READ_ATTRIBUTE_VALUE
uint8_t BGLib::ble_cmd_gatt_server_read_attribute_value( uint16 attribute, uint16 offset ) {
    uint8_t d[4];
    memcpy(d + 0, &attribute, sizeof(uint16));
    memcpy(d + 2, &offset, sizeof(uint16));
    return sendCommand( 4, 0x0A, 0, d );                   /* [1:len]4, [2:commandClass]A, [3:commandId]0, [4]d */
}
#endif
#ifdef BGLIB_ENABLE_CMD_GATT_SERVER_SEND_CHARACTERISTIC_NOTIFICATION
uint8_t BGLib::ble_cmd_gatt_server_send_characteristic_notification( uint8 connection, uint16 characteristic, uint8 value_len, const uint8 *value_data ) {
    uint8_t d[4 + value_len];
    memcpy(d + 0, &connection, sizeof(uint8));
    memcpy(d + 1, &characteristic, sizeof(uint16));
    memcpy(d + 3, &value_len, sizeof(uint8));
    memcpy(d + 4, value_data, value_len);
    return sendCommand( (4 + value_len), 0x0A, 5, d );      /* [1:len]4, [2:commandClass]A, [3:commandId]5, [4]d */
}
#endif
#ifdef BGLIB_ENABLE_CMD_GATT_SERVER_SEND_USER_READ_RESPONSE
uint8_t BGLib::ble_cmd_gatt_server_send_user_read_response( uint8 connection, uint16 characteristic, uint8 att_errorcode, uint8 value_len, const uint8 *value_data ) {
    uint8_t d[5 + value_len];
    memcpy(d + 0, &connection, sizeof(uint8));
    memcpy(d + 1, &characteristic, sizeof(uint16));
    memcpy(d + 3, &att_errorcode, sizeof(uint8));
    memcpy(d + 4, &value_len, sizeof(uint8));
    memcpy(d + 5, value_data, value_len);
    return sendCommand( (5 + value_len), 0x0A, 3, d );      /* [1:len]5, [2:commandClass]A, [3:commandId]3, [4]d */
}
#endif
#ifdef BGLIB_ENABLE_CMD_GATT_SERVER_SEND_USER_WRITE_RESPONSE
uint8_t BGLib::ble_cmd_gatt_server_send_user_write_response( uint8 connection, uint16 characteristic, uint8 att_errorcode ) {
    uint8_t d[4];
    memcpy(d + 0, &connection, sizeof(uint8));
    memcpy(d + 1, &characteristic, sizeof(uint16));
    memcpy(d + 3, &att_errorcode, sizeof(uint8));
    return sendCommand( 4, 0x0A, 4, d );                   /* [1:len]4, [2:commandClass]A, [3:commandId]4, [4]d */
}
#endif
#ifdef BGLIB_ENABLE_CMD_GATT_SERVER_SENT_CAPABILITIES
uint8_t BGLib::ble_cmd_gatt_server_send_capabilities( uint32 caps, uint32 reserved ) {
    uint8_t d[8];
    memcpy(d + 0, &caps, sizeof(uint32));
    memcpy(d + 4, &reserved, sizeof(uint32));
    return sendCommand( 8, 0x0A, 8, d );                   /* [1:len]8, [2:commandClass]A, [3:commandId]8, [4]d */
}
#endif
#ifdef BGLIB_ENABLE_CMD_GATT_SERVER_SET_DATABASE
uint8_t BGLib::ble_cmd_gatt_server_set_database( uint32 ptr ) {
    uint8_t d[4];
    memcpy(d + 0, &ptr, sizeof(uint32));
    return sendCommand( 4, 0x0A, 7, d );                   /* [1:len]4, [2:commandClass]A, [3:commandId]7, [4]d */
}
#endif
#ifdef BGLIB_ENABLE_CMD_GATT_SERVER_WRITE_ATTRIBUTE_VALUE
uint8_t BGLib::ble_cmd_gatt_server_write_attribute_value( uint16 attribute, uint16 offset, uint8 value_len, const uint8 *value_data ) {
    uint8_t d[5 + value_len];
    memcpy(d + 0, &attribute, sizeof(uint16));
    memcpy(d + 2, &offset, sizeof(uint16));
    memcpy(d + 4, &value_len, sizeof(uint8));
    memcpy(d + 5, value_data, value_len);
    return sendCommand( (5 + value_len), 0x0A, 2, d );      /* [1:len]5, [2:commandClass]A, [3:commandId]2, [4]d */
}
#endif
/*  */


/* [hardware] Hardware */
#ifdef BGLIB_ENABLE_CMD_HARDWARE_ENABLE_DCDC
uint8_t BGLib::ble_cmd_hardware_enable_dcdc( uint8 enable ) {
    uint8_t d[1];
    memcpy(d + 0, &enable, sizeof(uint8));
    return sendCommand( 1, 0x0C, 0x0D, d );                 /* [1:len]1, [2:commandClass]C, [3:commandId]D, [4]d */
}
#endif
#ifdef BGLIB_ENABLE_CMD_HARDWARE_GET_TIME
uint8_t BGLib::ble_cmd_haerware_get_time( void ) {
    return sendCommand( 0, 0x0C, 0x0B );                    /* [1:len]0, [2:commandClass]C, [3:commandId]B */
}
#endif
#ifdef BGLIB_ENABLE_CMD_HARDWARE_SET_LAZY_SOFT_TIMER
uint8_t BGLib::ble_cmd_hardware_set_lazy_soft_timer( uint32 time, uint32 slack,uint8 handle, uint8 single_shot ) {
    uint8_t d[10];
    memcpy(d + 0, &time, sizeof(uint32));
    memcpy(d + 4, &slack, sizeof(uint32));
    memcpy(d + 8, &handle, sizeof(uint8));
    memcpy(d + 9, &single_shot, sizeof(uint8));
    return sendCommand( 0x0A, 0x0C, 0x0C, d );              /* [1:len]A, [2:commandClass]C, [3:commandId]C, [4]d */
}
#endif
#ifdef BGLIB_ENABLE_CMD_HARDWARE_SET_SOFT_TIMER
uint8_t BGLib::ble_cmd_hardware_set_soft_timer( uint32 time, uint8 handle, uint8 single_shot ) {
    uint8_t d[6];
    memcpy(d + 0, &time, sizeof(uint32));
    memcpy(d + 4, &handle, sizeof(uint8));
    memcpy(d + 5, &single_shot, sizeof(uint8));
    return sendCommand( 0x06, 0x0C, 0x00, d );              /* [1:len]6, [2:commandClass]C, [3:commandId]0, [4]d */
}
#endif
/*  */


/* [le connection] Connection management */
#ifdef BGLIB_ENABLE_CMD_LE_CONNECTION_CLOSE
uint8_t BGLib::ble_cmd_le_connection_close( uint8 connection ) {
    uint8_t d[1];
    memcpy(d + 0, &connection, sizeof(uint8));
    return sendCommand( 1, 8, 4, d );                   /* [1:len]1, [2:commandClass]8, [3:commandId]4, [4]d */
}
#endif
#ifdef BGLIB_ENABLE_CMD_LE_CONNECTION_DISABLE_SLAVE_LATENCY
uint8_t BGLib::ble_cmd_le_connection_disable_slave_latency( uint8 connection, uint8 disable ) {
    uint8_t d[2];
    memcpy(d + 0, &connection, sizeof(uint8));
    memcpy(d + 1, &disable, sizeof(uint8));
    return sendCommand( 2, 8, 2, d );                   /* [1:len]2, [2:commandClass]8, [3:commandId]2, [4]d */
}
#endif
#ifdef BGLIB_ENABLE_CMD_LE_CONNECTION_GET_RSSI
uint8_t BGLib::ble_cmd_le_connection_get_rssi( uint8 connection ) {
    uint8_t d[1];
    memcpy(d + 0, &connection, sizeof(uint8));
    return sendCommand( 1, 8, 1, d );                   /* [1:len]1, [2:commandClass]8, [3:commandId]1, [4]d */
}
#endif
#ifdef BGLIB_ENABLE_CMD_LE_CONNECTION_SET_PARAMETERS
uint8_t BGLib::ble_cmd_le_connection_set_parameters( uint8 connection, uint16 min_interval, uint16 max_interval, uint16 latency, uint16 timeout ) {
    uint8_t d[9];
    memcpy(d + 0, &connection, sizeof(uint8));
    memcpy(d + 1, &min_interval, sizeof(uint16));
    memcpy(d + 3, &max_interval, sizeof(uint16));
    memcpy(d + 5, &latency, sizeof(uint16));
    memcpy(d + 7, &timeout, sizeof(uint16));
    return sendCommand( 9, 8, 0, d );                   /* [1:len]9, [2:commandClass]8, [3:commandId]0, [4]d */
}
#endif
#ifdef BGLIB_ENABLE_CMD_LE_CONNECTION_SET_PHY
uint8_t BGLib::ble_cmd_le_connection_set_phy( uint8 connection, uint8 phy ) {
    uint8_t d[2];
    memcpy(d + 0, &connection, sizeof(uint8));
    memcpy(d + 1, &phy, sizeof(uint8));
    return sendCommand( 2, 8, 3, d );                   /* [1:len]2, [2:commandClass]8, [3:commandId]3, [4]d */
}
#endif
/*  */


/* [le gap] Generic Access Profile */
#ifdef BGLIB_ENABLE_CMD_LE_GAP_BT5_SET_ADV_DATA
uint8_t BGLib::ble_cmd_le_gap_bt5_set_adv_data( uint8 handle, uint8 scan_rsp, uint8 adv_data_len, const uint8 *adv_data_data ) {
    uint8_t d[3 + adv_data_len];
    memcpy(d + 0, &handle, sizeof(uint8));
    memcpy(d + 1, &scan_rsp, sizeof(uint8));
    memcpy(d + 2, &adv_data_len, sizeof(uint8));
    memcpy(d + 3, adv_data_data, adv_data_len);
    return sendCommand( (3 + adv_data_len), 3, 0x0C, d );               /* [1:len]3, [2:commandClass]3, [3:commandId]C, [4]d */
}
#endif
#ifdef BGLIB_ENABLE_CMD_LE_GAP_BT5_SET_ADV_PARAMETERS
uint8_t BGLib::ble_cmd_le_gap_bt5_set_adv_parameters( uint8 handle, uint16 interval_min, uint16 interval_max, uint8 channel_map, uint8 report_scan ) {
    uint8_t d[7];
    memcpy(d + 0, &handle, sizeof(uint8));
    memcpy(d + 1, &interval_min, sizeof(uint16));
    memcpy(d + 3, &interval_max, sizeof(uint16));
    memcpy(d + 5, &channel_map, sizeof(uint8));
    memcpy(d + 6, &report_scan, sizeof(uint8));
    return sendCommand( 7, 3, 0x0B, d );                   /* [1:len]7, [2:commandClass]3, [3:commandId]B, [4]d */
}
#endif
#ifdef BGLIB_ENABLE_CMD_LE_GAP_BT5_SET_MODE
uint8_t BGLib::ble_cmd_le_gap_bt5_set_mode( uint8 handle, uint8 discover, uint8 connect, uint16 maxevents, uint8 address_type ) {
    uint8_t d[6];
    memcpy(d + 0, &handle, sizeof(uint8));
    memcpy(d + 1, &discover, sizeof(uint8));
    memcpy(d + 2, &connect, sizeof(uint8));
    memcpy(d + 3, &maxevents, sizeof(uint16));
    memcpy(d + 5, &address_type, sizeof(uint8));
    return sendCommand( 6, 3, 0x0A, d );                   /* [1:len]6, [2:commandClass]3, [3:commandId]A, [4]d */
}
#endif
#ifdef BGLIB_ENABLE_CMD_LE_GAP_CLEAR_ADVERTISE_CONFIGURATION
uint8_t BGLib::ble_cmd_le_gap_clear_advertise_configuration( uint8 handle, uint32 configurations ) {
    uint8_t d[5];
    memcpy(d + 0, &handle, sizeof(uint8));
    memcpy(d + 1, &configurations, sizeof(uint32));
    return sendCommand( 5, 3, 0x13, d );                   /* [1:len]6, [2:commandClass]3, [3:commandId]A, [4]d */
}
#endif
#ifdef BGLIB_ENABLE_CMD_LE_GAP_CONNECT
uint8_t BGLib::ble_cmd_le_gap_connect( bd_addr address, uint8 address_type, uint8 initiating_phy ) {
    uint8_t d[8];
    memcpy(d + 0, &address, sizeof(bd_addr));
    memcpy(d + 6, &address_type, sizeof(uint8));
    memcpy(d + 7, &initiating_phy, sizeof(uint8));
    return sendCommand( 8, 3, 0x1A, d );                   /* [1:len]8, [2:commandClass]3, [3:commandId]0x1A, [4]d */
}
#endif
#ifdef BGLIB_ENABLE_CMD_LE_GAP_DISCOVER
uint8_t BGLib::ble_cmd_le_gap_discover( uint8 mode ) {
    uint8_t d[1];
    memcpy(d + 0, &mode, sizeof(uint8));
    return sendCommand( 1, 3, 2, d );                   /* [1:len]1, [2:commandClass]3, [3:commandId]2, [4]d */
}
#endif
#ifdef BGLIB_ENABLE_CMD_LE_GAP_END_PROCEDURE
uint8_t BGLib::ble_cmd_le_gap_end_procedure( void ) {
    return sendCommand( 0, 3, 3 );                      /* [1:len]0, [2:commandClass]3, [3:commandId]3 */
}
#endif
#ifdef BGLIB_ENABLE_CMD_LE_GAP_OPEN
uint8_t BGLib::ble_cmd_le_gap_open( bd_addr address, uint8 address_type ) {
    uint8_t d[7];
    memcpy(d + 0, &address, sizeof(bd_addr));
    memcpy(d + 6, &address_type, sizeof(uint8));
    return sendCommand( 7, 3, 0, d );                   /* [1:len]7, [2:commandClass]3, [3:commandId]0, [4]d */
}
#endif
#ifdef BGLIB_ENABLE_CMD_LE_GAP_SET_ADV_DATA
uint8_t BGLib::ble_cmd_le_gap_set_adv_data( uint8 scan_rsp, uint8 adv_data_len, const uint8 *adv_data_data ) {
    uint8_t d[2 + adv_data_len];
    memcpy(d + 0, &scan_rsp, sizeof(uint8));
    memcpy(d + 1, &adv_data_len, sizeof(uint8));
    memcpy(d + 2, adv_data_data, adv_data_len);
    return sendCommand( (2+ adv_data_len), 3, 7, d );   /* [1:len]2, [2:commandClass]3, [3:commandId]7, [4]d */
}
#endif
#ifdef BGLIB_ENABLE_CMD_LE_GAP_SET_ADV_PARAMETERS
uint8_t BGLib::ble_cmd_le_gap_set_adv_parameters( uint16 interval_min, uint16 interval_max, uint8 channnel_map ) {
    uint8_t d[5];
    memcpy(d + 0, &interval_min, sizeof(uint16));
    memcpy(d + 2, &interval_max, sizeof(uint16));
    memcpy(d + 4, &channnel_map, sizeof(uint8));
    return sendCommand( 5, 3, 4, d );                   /* [1:len]5, [2:commandClass]3, [3:commandId]4, [4]d */
}
#endif
#ifdef BGLIB_ENABLE_CMD_LE_GAP_SET_ADV_TIMEOUT
uint8_t BGLib::ble_cmd_le_gap_set_adv_timeout( uint8 maxevents ) {
    uint8_t d[1];
    memcpy(d + 0, &maxevents, sizeof(uint8));
    return sendCommand( 1, 3, 8, d );                   /* [1:len]1, [2:commandClass]3, [3:commandId]8, [4]d */
}
#endif
#ifdef BGLIB_ENABLE_CMD_LE_GAP_SET_ADVERTISE_CHANNEL_MAP
uint8_t BGLib::ble_cmd_le_gap_set_advertise_channel_map( uint8 handle, uint8 channel_map ) {
    uint8_t d[2];
    memcpy(d + 0, &handle, sizeof(uint8));
    memcpy(d + 1, &channel_map, sizeof(uint8));
    return sendCommand( 1, 3, 8, d );                   /* [1:len]1, [2:commandClass]3, [3:commandId]8, [4]d */
}
#endif
#ifdef BGLIB_ENABLE_CMD_LE_GAP_SET_ADVERTISE_CONFIGURATION
uint8_t BGLib::ble_cmd_le_gap_set_advertise_configuration( uint8 handle, uint32 configuration ) {
    uint8_t d[5];
    memcpy(d + 0, &handle, sizeof(uint8));
    memcpy(d + 1, &configuration, sizeof(uint32));
    return sendCommand( 5, 3, 0x12, d );                /* [1:len]5, [2:commandClass]3, [3:commandId]0x12, [4]d */
}
#endif
#ifdef BGLIB_ENABLE_CMD_LE_GAP_SET_ADVERTISE_PHY
uint8_t BGLib::ble_cmd_le_gap_set_advertise_phy( uint8 handle, uint8 primary_phy, uint8 secondary_phy ) {
    uint8_t d[3];
    memcpy(d + 0, &handle, sizeof(uint8));
    memcpy(d + 1, &primary_phy, sizeof(uint8));
    memcpy(d + 2, &secondary_phy, sizeof(uint8));
    return sendCommand( 3, 3, 0x11, d );                /* [1:len]3, [2:commandClass]3, [3:commandId]0x11, [4]d */
}
#endif
#ifdef BGLIB_ENABLE_CMD_LE_GAP_SET_ADVERTISE_REPORT_SCAN_REQUEST
uint8_t BGLib::ble_cmd_le_gap_set_advertise_report_scan_request( uint8 handle, uint8 report_scan_req ) {
    uint8_t d[2];
    memcpy(d + 0, &handle, sizeof(uint8));
    memcpy(d + 1, &report_scan_req, sizeof(uint8));
    return sendCommand( 2, 3, 0x10, d );                /* [1:len]2, [2:commandClass]3, [3:commandId]0x10, [4]d */
}
#endif
#ifdef BGLIB_ENABLE_CMD_LE_GAP_SET_ADVERTISE_TIMING
uint8_t BGLib::ble_cmd_le_gap_set_advertise_timing( uint8 handle, uint32 interval_min, uint32 interval_max, uint16 duration, uint8 maxevents ) {
    uint8_t d[12];
    memcpy(d + 0, &handle, sizeof(uint8));
    memcpy(d + 1, &interval_min, sizeof(uint32));
    memcpy(d + 5, &interval_max, sizeof(uint32));
    memcpy(d + 9, &duration, sizeof(uint16));
    memcpy(d + 11, &maxevents, sizeof(uint8));
    return sendCommand( 12, 3, 0x0E, d );                /* [1:len]12, [2:commandClass]3, [3:commandId]0x0E, [4]d */
}
#endif
#ifdef BGLIB_ENABLE_CMD_LE_GAP_SET_CONN_PARAMEETERS
uint8_t BGLib::ble_cmd_le_gap_set_conn_parameters( uint16 interval_min, uint16 interval_max, uint16 latency, uint16 timeout ) {
    uint8_t d[8];
    memcpy(d + 0, &interval_min, sizeof(uint16));
    memcpy(d + 2, &interval_max, sizeof(uint16));
    memcpy(d + 4, &latency, sizeof(uint16));
    memcpy(d + 6, &timeout, sizeof(uint16));
    return sendCommand( 8, 3, 0x05, d );                /* [1:len]8, [2:commandClass]3, [3:commandId]0x05, [4]d */
}
#endif
#ifdef BGLIB_ENABLE_CMD_LE_GAP_SET_DATA_CHANNEL_CLASSIFICATION
uint8_t BGLib::ble_cmd_le_gap_set_data_channel_classification( uint8 channel_map_len, const uint8 *channel_map_data ) {
    uint8_t d[1 + channel_map_len];
    memcpy(d + 0, &channel_map_len, sizeof(uint8));
    memcpy(d + 1, channel_map_data, channel_map_len);
    return sendCommand( (1 + channel_map_len), 3, 0x19, d );    /* [1:len]1, [2:commandClass]3, [3:commandId]0x19, [4]d */
}
#endif
#ifdef BGLIB_ENABLE_CMD_LE_GAP_SET_DISCOVERY_TIMING
uint8_t BGLib::ble_cmd_le_gap_set_discovery_timing( uint8 phys, uint16 scan_interval, uint16 scan_window ) {
    uint8_t d[5];
    memcpy(d + 0, &phys, sizeof(uint8));
    memcpy(d + 1, &scan_interval, sizeof(uint16));
    memcpy(d + 3, &scan_window, sizeof(uint16));
    return sendCommand( 5, 3, 0x16, d );            /* [1:len]5, [2:commandClass]3, [3:commandId]0x16, [4]d */
}
#endif
#ifdef BGLIB_ENABLE_CMD_LE_GAP_SET_DISCOVERY_TYPE
uint8_t BGLib::ble_cmd_le_gap_set_discovery_type( uint8 phys, uint8 scan_type ) {
    uint8_t d[2];
    memcpy(d + 0, &phys, sizeof(uint8));
    memcpy(d + 1, &scan_type, sizeof(uint8));
    return sendCommand( 2, 3, 0x17, d );            /* [1:len]2, [2:commandClass]3, [3:commandId]0x17, [4]d */
}
#endif
#ifdef BGLIB_ENABLE_CMD_LE_GAP_SET_MODE
uint8_t BGLib::ble_cmd_le_gap_set_mode( uint8 discover, uint8 connect ) {
    uint8_t d[2];
    memcpy(d + 0, &discover, sizeof(uint8));
    memcpy(d + 1, &connect, sizeof(uint8));
    return sendCommand( 2, 3, 1, d );                   /* [1:len]2, [2:commandClass]3, [3:commandId]1, [4]d */
}
#endif
#ifdef BGLIB_ENABLE_CMD_LE_GAP_SET_PRIVACY_MODE
uint8_t BGLib::ble_cmd_le_gap_set_privacy_mode( uint8 privacy, uint8 interval ) {
    uint8_t d[2];
    memcpy(d + 0, &privacy, sizeof(uint8));
    memcpy(d + 1, &interval, sizeof(uint8));
    return sendCommand( 2, 3, 0x0D, d );                /* [1:len]2, [2:commandClass]3, [3:commandId]0x0D, [4]d */
}
#endif
#ifdef BGLIB_ENABLE_CMD_LE_GAP_SET_SCAN_PARAMETERS
uint8_t BGLib::ble_cmd_le_gap_set_scan_parameters( uint16 scan_interval, uint16 scan_window, uint8 active ) {
    uint8_t d[5];
    memcpy(d + 0, &scan_interval, sizeof(uint16));
    memcpy(d + 2, &scan_window, sizeof(uint16));
    memcpy(d + 4, &active, sizeof(uint8));
    return sendCommand( 5, 3, 6, d );                   /* [1:len]5, [2:commandClass]3, [3:commandId]6, [4]d */
}
#endif
#ifdef BGLIB_ENABLE_CMD_LE_GAP_START_ADVERTISING
uint8_t BGLib::ble_cmd_le_gap_start_advertising( uint8 handle, uint8 discover, uint8 connect ) {
    uint8_t d[3];
    memcpy(d + 0, &handle, sizeof(uint8));
    memcpy(d + 1, &discover, sizeof(uint8));
    memcpy(d + 2, &connect, sizeof(uint8));
    return sendCommand( 3, 3, 0x14, d );            /* [1:len]3, [2:commandClass]3, [3:commandId]0x14, [4]d */
}
#endif
#ifdef BGLIB_ENABLE_CMD_LE_GAP_START_DISCOVERY
uint8_t BGLib::ble_cmd_le_gap_start_discovery( uint8 scanning_phy, uint8 mode ) {
    uint8_t d[2];
    memcpy(d + 0, &scanning_phy, sizeof(uint8));
    memcpy(d + 1, &mode, sizeof(uint8));
    return sendCommand( 2, 3, 0x18, d );            /* [1:len]2, [2:commandClass]3, [3:commandId]0x18, [4]d */
}
#endif
#ifdef BGLIB_ENABLE_CMD_LE_GAP_STOP_ADVERTISING
uint8_t BGLib::ble_cmd_le_gap_stop_advertising( uint8 handle ) {
    uint8_t d[1];
    memcpy(d + 0, &handle, sizeof(uint8));
    return sendCommand( 1, 3, 0x15, d );                   /* [1:len]1, [2:commandClass]3, [3:commandId]5, [4]d */
}
#endif
/*  */


/* [SM] Security Manager */
#ifdef BGLIB_ENABLE_CMD_SM_BONDING_CONFIRM
uint8_t BGLib::ble_cmd_sm_bonding_confirm( uint8 connection, uint8 confirm ) {
    uint8_t d[2];
    memcpy(d + 0, &connection, sizeof(uint8));
    memcpy(d + 1, &confirm, sizeof(uint8));
    return sendCommand( 2, 0x0F, 0x0E, d );                   /* [1:len]2, [2:commandClass]0x0F, [3:commandId]0x0E, [4]d */
}
#endif
#ifdef BGLIB_ENABLE_CMD_SM_CONFIGURE
uint8_t BGLib::ble_cmd_sm_configure( uint8 flags, uint8 io_capabilities ) {
    uint8_t d[2];
    memcpy(d + 0, &flags, sizeof(uint8));
    memcpy(d + 1, &io_capabilities, sizeof(uint8));
    return sendCommand( 2, 0x0F, 0x01, d );                   /* [1:len]2, [2:commandClass]0x0F, [3:commandId]1, [4]d */
}
#endif
#ifdef BGLIB_ENABLE_CMD_SM_DELETE_BONDING
uint8_t BGLib::ble_cmd_sm_delete_bonding( uint8 bonding ) {
    uint8_t d[1];
    memcpy(d + 0, &bonding, sizeof(uint8));
    return sendCommand( 1, 0x0F, 0x06, d );                   /* [1:len]1, [2:commandClass]0x0F, [3:commandId]6, [4]d */
}
#endif
#ifdef BGLIB_ENABLE_CMD_SM_DELETE_BONDINGS
uint8_t BGLib::ble_cmd_sm_delete_bondings( void ) {
    return sendCommand( 0, 0x0F, 0x07 );                    /* [1:len]0, [2:commandClass]0x0F, [3:commandId]7 */
}
#endif
#ifdef BGLIB_ENABLE_CMD_SM_ENTER_PASSKEY
uint8_t BGLib::ble_cmd_sm_enter_passkey( uint8 connection, int32 passkey ) {
    uint8_t d[5];
    memcpy(d + 0, &connection, sizeof(uint8));
    memcpy(d + 1, &passkey, sizeof(int32));
    return sendCommand( 5, 0x0F, 0x08, d );                   /* [1:len]5, [2:commandClass]0x0F, [3:commandId]8, [4]d */
}
#endif
#ifdef BGLIB_ENABLE_CMD_SM_INCREASE_SECURITY
uint8_t BGLib::ble_cmd_sm_increase_security( uint8 connection ) {
    uint8_t d[1];
    memcpy(d + 0, &connection, sizeof(uint8));
    return sendCommand( 1, 0x0F, 0x04, d );                   /* [1:len]1, [2:commandClass]0x0F, [3:commandId]4, [4]d */
}
#endif
#ifdef BGLIB_ENABLE_CMD_SM_LIST_ALL_BONDINGS
uint8_t BGLib::ble_cmd_sm_list_all_bondings( void ) {
    return sendCommand( 0, 0x0F, 0x0B );                   /* [1:len]0, [2:commandClass]0x0F, [3:commandId]0x0B */
}
#endif
#ifdef BGLIB_ENABLE_CMD_SM_PASSKEY_CONFIRM
uint8_t BGLib::ble_cmd_sm_passkey_confirm( uint8 connection, uint8 confirm ) {
    uint8_t d[2];
    memcpy(d + 0, &connection, sizeof(uint8));
    memcpy(d + 1, &confirm, sizeof(uint8));
    return sendCommand( 2, 0x0F, 0x09, d );                   /* [1:len]2, [2:commandClass]0x0F, [3:commandId]9, [4]d */
}
#endif
#ifdef BGLIB_ENABLE_CMD_SM_SET_BONDABLE_MODE
uint8_t BGLib::ble_cmd_sm_set_bondable_mode( uint8 bondable ) {
    uint8_t d[1];
    memcpy(d + 0, &bondable, sizeof(uint8));
    return sendCommand( 1, 0x0F, 0x00, d );                   /* [1:len]1, [2:commandClass]0x0F, [3:commandId]0, [4]d */
}
#endif
#ifdef BGLIB_ENABLE_CMD_SM_SET_DEBUG_MODE
uint8_t BGLib::ble_cmd_sm_set_debug_mode( void ) {
    return sendCommand( 0, 0x0F, 0x0F );                   /* [1:len]0, [2:commandClass]0x0F, [3:commandId]0x0F */
}
#endif
#ifdef BGLIB_ENABLE_CMD_SM_SET_OOB_DATA
uint8_t BGLib::ble_cmd_sm_set_oob_data( uint8 oob_data_len, const uint8 *oob_data_data ) {
    uint8_t d[1 + oob_data_len];
    memcpy(d + 0, &oob_data_len, sizeof(uint8));
    memcpy(d + 1, oob_data_data, oob_data_len);
    return sendCommand( (1 + oob_data_len), 0x0F, 0x0A, d );    /* [1:len]1, [2:commandClass]0x0F, [3:commandId]A, [4]d */
}
#endif
#ifdef BGLIB_ENABLE_CMD_SM_SET_PASSKEY
uint8_t BGLib::ble_cmd_sm_set_passkey( int32 passkey ) {
    uint8_t d[4];
    memcpy(d + 0, &passkey, sizeof(int32));
    return sendCommand( 4, 0x0F, 0x10, d );    /* [1:len]4, [2:commandClass]0x0F, [3:commandId]0x10, [4]d */
}
#endif
#ifdef BGLIB_ENABLE_CMD_SM_SET_SC_REMOTE_OOB_DATA
uint8_t BGLib::ble_cmd_sm_set_sc_remote_oob_data( uint8 oob_data_len, const uint8 *oob_data_data ) {
    uint8_t d[1 + oob_data_len];
    memcpy(d + 0, &oob_data_len, sizeof(uint8));
    memcpy(d + 1, oob_data_data, oob_data_len);
    return sendCommand( (1 + oob_data_len), 0x0F, 0x12, d );    /* [1:len]1, [2:commandClass]0x0F, [3:commandId]0x12, [4]d */
}
#endif
#ifdef BGLIB_ENABLE_CMD_SM_STORE_BONDING_CONFIGURATION
uint8_t BGLib::ble_cmd_sm_store_bonding_configuration( uint8 max_bonding_count, uint8 policy_flags ) {
    uint8_t d[2];
    memcpy(d + 0, &max_bonding_count, sizeof(uint8));
    memcpy(d + 1, &policy_flags, sizeof(uint8));
    return sendCommand( 2, 0x0F, 0x02, d );    /* [1:len]2, [2:commandClass]0x0F, [3:commandId]0x02, [4]d */
}
#endif
#ifdef BGLIB_ENABLE_CMD_SM_USE_SC_OOB
uint8_t BGLib::ble_cmd_sm_use_sc_oob( uint8 enable ) {
    uint8_t d[1];
    memcpy(d + 0, &enable, sizeof(uint8));
    return sendCommand( 1, 0x0F, 0x11, d );    /* [1:len]1, [2:commandClass]0x0F, [3:commandId]0x11, [4]d */
}
#endif
/*  */


/* [system] System commands */
#ifdef BGLIB_ENABLE_CMD_SYSTEM_GET_BT_ADDRESS
uint8_t BGLib::ble_cmd_system_get_bt_address( void ) {
    return sendCommand( 0, 1, 3 );                      /* [1:len]0, [2:commandClass]1, [3:commandId]3 */
}
#endif
#ifdef BGLIB_ENABLE_CMD_SYSTEM_GET_COUNTERS
uint8_t BGLib::ble_cmd_system_get_counters( uint8 reset ) {
    uint8_t d[1];
    memcpy(d + 0, &reset, sizeof(uint8));
    return sendCommand( 1, 1, 0x0F, d );                /* [1:len]1, [2:commandClass]1, [3:commandId]F, [4]reset */
}
#endif
#ifdef BGLIB_ENABLE_CMD_SYSTEM_GET_RANDOM_DATA
uint8_t BGLib::ble_cmd_system_get_random_data( uint8 length ) {
    uint8_t d[1];
    memcpy(d + 0, &length, sizeof(uint8));
    return sendCommand( 1, 1, 0x0B, d );                /* [1:len]1, [2:commandClass]1, [3:commandId]B, [4]length */
}
#endif
#ifdef BGLIB_ENABLE_CMD_SYSTEM_HALT
uint8_t BGLib::ble_cmd_system_halt( uint8 halt ) {
    uint8_t d[1];
    memcpy(d + 0, &halt, sizeof(uint8));
    return sendCommand( 1, 1, 0x0C, d );                /* [1:len]1, [2:commandClass]1, [3:commandId]C, [4]halt */
}
#endif
#ifdef BGLIB_ENABLE_CMD_SYSTEM_HELLO
uint8_t BGLib::ble_cmd_system_hello( void ) {
    return sendCommand( 0, 1, 0 );                      /* [1:len]0, [2:commandClass]1, [3:commandId]0 */
}
#endif
#ifdef BGLIB_ENABLE_CMD_SYSTEM_RESET
uint8_t BGLib::ble_cmd_system_reset( uint8 boot_in_dfu ) {
    uint8_t d[1];
    memcpy(d + 0, &boot_in_dfu, sizeof(uint8));
    return sendCommand( 1, 1, 1, d );                   /* [1:len]1, [2:commandClass]1, [3:commandId]1, [4]dfu */
}
#endif
#ifdef BGLIB_ENABLE_CMD_SYSTEM_SET_BT_ADDRESS
uint8_t BGLib::ble_cmd_system_set_bt_address( bd_addr address ) {
    uint8_t d[6];
    memcpy(d + 0, &address, sizeof(bd_addr));
    return sendCommand( 6, 1, 4, d );                   /* [1:len]6, [2:commandClass]1, [3:commandId]4, [4]address */
}
#endif
#ifdef BGLIB_ENABLE_CMD_SYSTEM_SET_DEVICE_NAME
uint8_t BGLib::ble_cmd_system_set_device_name( uint8 type, uint8 name_len, const uint8 *name_data ) {
    uint8_t d[2 + name_len];
    memcpy(d + 0, &type, sizeof(uint8));
    memcpy(d + 1, &name_len, sizeof(uint8));
    memcpy(d + 2, name_data, name_len);
    return sendCommand( (2 + name_len), 1, 0x0D, d );       /* [1:len]2, [2:commandClass]1, [3:commandId]D, [4]power */
}
#endif
#ifdef BGLIB_ENABLE_CMD_SYSTEM_SET_TX_POWER
uint8_t BGLib::ble_cmd_system_set_tx_power( int16 power ) {
    uint8_t d[2];
    memcpy(d + 0, &power, sizeof(int16));
    return sendCommand( 2, 1, 0x0A, d );                /* [1:len]2, [2:commandClass]1, [3:commandId]A, [4]power */
}
#endif
/*  */


/* [test] testing */
#ifdef BGLIB_ENABLE_CMD_TEST_DTM_END
uint8_t BGLib::ble_cmd_test_dtm_end( void ) {
    return sendCommand( 0, 0x0E, 0x02 );                /* [1:len]0, [2:commandClass]E, [3:commandId]2 */
}
#endif
#ifdef BGLIB_ENABLE_CMD_TEST_DTM_RX
uint8_t BGLib::ble_cmd_test_dtm_rx( uint8 channel, uint8 phy ) {
    uint8_t d[2];
    memcpy(d + 0, &channel, sizeof(uint8));
    memcpy(d + 1, &phy, sizeof(uint8));
    return sendCommand( 2, 0x0E, 0x01, d );             /* [1:len]2, [2:commandClass]E, [3:commandId]1, [4]power */
}
#endif
#ifdef BGLIB_ENABLE_CMD_TEST_DTM_TX
uint8_t BGLib::ble_cmd_test_dtm_tx( uint8 packet_type, uint8 length, uint8 channel, uint8 phy ) {
    uint8_t d[4];
    memcpy(d + 0, &packet_type, sizeof(uint8));
    memcpy(d + 1, &length, sizeof(uint8));
    memcpy(d + 2, &channel, sizeof(uint8));
    memcpy(d + 3, &phy, sizeof(uint8));
    return sendCommand( 4, 0x0E, 0x00, d );             /* [1:len]4, [2:commandClass]E, [3:commandId]0, [4]power */
}
#endif
/*  */


/* [user] User message */
#ifdef BGLIB_ENABLE_CMD_USER_MESSAGE_TO_TARGET
uint8_t BGLib::ble_cmd_user_message_to_target( uint8 data_len, const uint8 *data_data ) {
    uint8_t d[1 + data_len];
    memcpy(d + 0, &data_len, sizeof(uint8));
    memcpy(d + 1, data_data, data_len);
    return sendCommand( (1 + data_len), 0xFF, 0, d );       /* [1:len]1, [2:commandClass]0xFF, [3:commandId]0, [4]power */
}
#endif
/*  */
