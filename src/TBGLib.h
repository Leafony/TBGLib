//=====================================================================
//   Leafony Platform BLE leaf Bule Geko library
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

#ifndef __TBGLIB_H__
#define __TBGLIB_H__

#include <Arduino.h>
#include <stdint.h>
#include "TBGLibConfig.h"



//#define __attribute__((packed))         PACKED
//#define __attribute__((aligned(0x4)))   ALIGNED


typedef enum {
    MESSAGE_CLASS_DFU           = 0x00,         /* Device Firmware Upgrade */
    MESSAGE_CLASS_SYSTEM        = 0x01,         /* System */
    MESSAGE_CLASS_LE_GAP        = 0x03,         /* Generic Access Profile */
    MESSAGE_CLASS_LE_CONNECTION = 0x08,         /* Connection management */
    MESSAGE_CLASS_GATT          = 0x09,         /* Generic Attribute Profile */
    MESSAGE_CLASS_GATT_SERVER   = 0x0A,         /* Generic Attribute Profile Server */
    MESSAGE_CLASS_ENDPOINT      = 0x0B,         /* Endpoint */
    MESSAGE_CLASS_HARDWARE      = 0x0C,         /* Haedware */
    MESSAGE_CLASS_FLASH         = 0x0D,         /* Persistent Store */
    MESSAGE_CLASS_TEST          = 0x0E,         /* testing commands */
    MESSAGE_CLASS_SM            = 0x0F,         /* Security Manager */
    MESSAGE_CLASS_COEX          = 0x20,         /* Cooexistence interface */
    MESSAGE_CLASS_USER          = 0xFF          /* User messaging */
} eMessgaClassID;


/* [coex] cooexistence interface */
typedef enum {
    RSP_COEX_GET_COUNTERS_ID                                = 0x01,     /*  */
    RSP_COEX_SET_OPTIONS_ID                                 = 0x00      /*  */
} eMesCoexResponseID;
/*  */

/* [dfu] Device Firmware Upgrade */
typedef enum {
    RSP_DFU_FLASH_SET_ADDRESS_ID                            = 0x01,     /*  */
    RSP_DFU_FLASH_UPLOAD_ID                                 = 0x02,     /*  */
    RSP_DFU_FLASH_UPLOAD_FINISH_ID                          = 0x03,     /*  */
    RSP_DFU_RESET_ID                                        = 0x00      /*  */
} eMesDfuResponseID;
typedef enum {
    EVT_DFU_BOOT_ID                                         = 0x00,     /*  */
    EVT_DFU_BOOT_FAILURE_ID                                 = 0x01      /*  */
} eMesDfuEventID;
/*  */

/* [endpoint] Endpoint */
typedef enum {
    RSP_ENDPOINT_CLOSE_ID                                   = 0x002     /*  */
} eMesEndpointResponseID;
/*  */

/* [flash] Peripheral Store */
typedef enum {
    RSP_FLASH_PS_ERASE_ID                                   = 0x04,     /*  */
    RSP_FLASH_PS_ERASE_ALL_ID                               = 0x01,     /*  */
    RSP_FLASH_PS_LOAD_ID                                    = 0x03,     /*  */
    RSP_FLASH_PS_SAVE_ID                                    = 0x02      /*  */
} eMesFlashResponseID;
/*  */

/* [gatt] Generic Attribute Profile */
typedef enum {
    RSP_GATT_DISCOVER_CHARACTERISTICS_ID                    = 0x03,     /*  */
    RSP_GATT_DISCOVER_CHARACTERISTICS_BY_UUID_ID            = 0x04,     /*  */
    RSP_GATT_DISCOVER_DESCRIPTORS_ID                        = 0x06,     /*  */
    RSP_GATT_DISCOVER_PRIMARY_SERVICES_ID                   = 0x01,     /*  */
    RSP_GATT_DISCOVER_PRIMARY_SERVICES_BY_UUID_ID           = 0x02,     /*  */
    RSP_GATT_EXECUTE_CHARACTERISTIC_VALUE_WRITE_ID          = 0x0C,     /*  */
    RSP_GATT_FIND_INCLUDED_SERVICES_ID                      = 0x10,     /*  */
    RSP_GATT_PREPARE_CHARACTERISTIC_VALUE_RELIABLE_WRITE_ID = 0x13,     /*  */
    RSP_GATT_PREPARE_CHARACTERISTIC_VALUE_WRITE_ID          = 0x0B,     /*  */
    RSP_GATT_READ_CHARACTERISTIC_VALUE_ID                   = 0x07,     /*  */
    RSP_GATT_READ_CHARACTERISTIC_VALUE_BY_UUID_ID           = 0x08,     /*  */
    RSP_GATT_READ_CHARACTERISTIC_VALUE_FROM_OFFSET_ID       = 0x12,     /*  */
    RSP_GATT_READ_DESCRIPTOR_VALUE_ID                       = 0x0E,     /*  */
    RSP_GATT_READ_MULTIPLE_CHARACTERISTIC_VALUES_ID         = 0x11,     /*  */
    RSP_GATT_SEND_CHARACTERISTIC_CONFIRMATION_ID            = 0x0D,     /*  */
    RSP_GATT_SET_CHARACTERISTIC_NOTIFICATION_ID             = 0x05,     /*  */
    RSP_GATT_SET_MAX_MTU_ID                                 = 0x00,     /*  */
    RSP_GATT_WRITE_CHARACTERISTIC_VALUE_ID                  = 0x09,     /*  */
    RSP_GATT_WRITE_CHARACTERISTIC_VALUE_WITHOUT_RESPONSE_ID = 0x0A,     /*  */
    RSP_GATT_WRITE_DESCRIPTOR_VALUE_ID                      = 0x0F      /*  */
} eMesGattResponseID;
typedef enum {
    EVT_GATT_CHARACTERISTIC_ID                              = 0x02,     /*  */
    EVT_GATT_CHARACTERISTIC_VALUE_ID                        = 0x04,     /*  */
    EVT_GATT_DESCRIPTOR_ID                                  = 0x03,     /*  */
    EVT_GATT_DESCRIPTOR_VALUE_ID                            = 0x05,     /*  */
    EVT_GATT_MTU_EXCHANGED_ID                               = 0x00,     /*  */
    EVT_GATT_PROCEDURE_COMPLETED_ID                         = 0x06,     /*  */
    EVT_GATT_SERVICE_ID                                     = 0x01      /*  */
} eMesGattEventID;
/*  */

/* [gatt_server] Generic Attribure Profile Server (Response ID) */
typedef enum {
    RSP_GATT_SERVER_FIND_ATTRIBUTE_ID                       = 0x06,     /*  */
    RSP_GATT_SERVER_READ_ATTRIBUTE_TYPE_ID                  = 0x01,     /*  */
    RSP_GATT_SERVER_READ_ATTRIBUTE_VALUE_ID                 = 0x00,     /*  */
    RSP_GATT_SERVER_SEND_CHARACTERISTIC_NOTIFICATION_ID     = 0x05,     /*  */
    RSP_GATT_SERVER_SEND_USER_READ_RESPONS_ID               = 0x03,     /*  */
    RSP_GATT_SERVER_SEND_USER_WRITE_RESPONS_ID              = 0x04,     /*  */
    RSP_GATT_SERVER_SET_CAPABILITIES_ID                     = 0x08,     /*  */
    RSP_GATT_SERVER_SET_DATABASE_ID                         = 0x07,     /*  */
    RSP_GATT_SERVER_WRITE_ATTRIBUTE_VALUE_ID                = 0x02      /*  */
} eMesGattServerResponseID;
typedef enum {
    EVT_GATT_SERVER_ATTRIBUTE_VALUE_ID                      = 0x00,     /*  */
    EVT_GATT_SERVER_CHARACTERISTIC_STATUS_ID                = 0x03,     /*  */
    EVT_GATT_SERVER_EXECUTE_WRITE_COMPLETED_ID              = 0x04,     /*  */
    EVT_GATT_SERVER_USER_READ_REQUEST_ID                    = 0x01,     /*  */
    EVT_GATT_SERVER_USER_WRITE_REQUEST_ID                   = 0x02,     /*  */
} eMesGattServerEventID;
/*  */

/* [hardware] Hardware */
typedef enum {
    RSP_GATT_HARDWARE_ENABLE_DCDC_ID                        = 0x0D,     /*  */
    RSP_GATT_HARDWARE_GET_TIME_ID                           = 0x0B,     /*  */
    RSP_GATT_HARDWARE_SET_LAZY_SOFT_TIMER_ID                = 0x0C,     /*  */
    RSP_GATT_HARDWARE_SET_SOFT_TIMER_ID                     = 0x00,     /*  */
} eMesHardwareResponseID;
typedef enum {
    EVT_GATT_HARDWARE_SOFT_TIMER_ID                         = 0x00,     /*  */
} eMesHardwareEventID;
/*  */

/* [le connection] Connection management (Response ID) */
typedef enum {
    RSP_LE_CONNECTION_CLOSE_ID                              = 0x04,     /*  */
    RSP_LE_CONNECTION_DISABLE_SLAVE_LATENCY_ID              = 0x02,     /*  */
    RSP_LE_CONNECTION_GET_RSSI_ID                           = 0x01,     /*  */
    RSP_LE_CONNECTION_SET_PARAMETERS_ID                     = 0x00,     /*  */
    RSP_LE_CONNECTION_SET_PHY_ID                            = 0x03      /*  */
} eMesLeConnectionResponseID;
typedef enum {
    EVT_LE_CONNECTION_CLOSED_ID                             = 0x01,     /*  */
    EVT_LE_CONNECTION_OPENED_ID                             = 0x00,     /*  */
    EVT_LE_CONNECTION_PARAMETERS_ID                         = 0x02,     /*  */
    EVT_LE_CONNECTION_PHY_STATUS_ID                         = 0x04,     /*  */
    EVT_LE_CONNECTION_RSSI_ID                               = 0x03      /*  */
} eMesLeConnectionEventID;
/*  */

/* [le gap] Generic Access Profile (Response ID) */
typedef enum {
    RSP_LE_GAP_BT5_SET_ADV_DATA_ID                          = 0x0C,     /*  */
    RSP_LE_GAP_BT5_SET_ADV_PARAMETERS_ID                    = 0x0B,     /*  */
    RSP_LE_GAP_BT5_SET_MODE_ID                              = 0x0A,     /*  */
    RSP_LE_GAP_CLEAR_ADVERTISE_CONFIGURATION_ID             = 0x13,     /*  */
    RSP_LE_GAP_CONNECT_ID                                   = 0x1A,     /*  */
    RSP_LE_GAP_DISCOVER_ID                                  = 0x02,     /*  */
    RSP_LE_GAP_PROCEDURE_ID                                 = 0x03,     /*  */
    RSP_LE_GAP_OPEN_ID                                      = 0x00,     /*  */
    RSP_LE_GAP_SET_ADV_DATA_ID                              = 0x07,     /*  */
    RSP_LE_GAP_SET_ADV_PARAMETERS_ID                        = 0x04,     /*  */
    RSP_LE_GAP_SET_ADV_TIMEOUT_ID                           = 0x08,     /*  */
    RSP_LE_GAP_SET_ADVERTISE_CHANNEL_MAP_ID                 = 0x0F,     /*  */
    RSP_LE_GAP_SET_ADVERTISE_CONFIGURATION_ID               = 0x12,     /*  */
    RSP_LE_GAP_SET_ADVERTISE_PHY_ID                         = 0x11,     /*  */
    RSP_LE_GAP_SET_ADVERTISE_REPORT_SCAN_REQUEST_ID         = 0x10,     /*  */
    RSP_LE_GAP_SET_ADVERTISE_TIMING_ID                      = 0x0E,     /*  */
    RSP_LE_GAP_SET_CONN_PARAMETERS_ID                       = 0x05,     /*  */
    RSP_LE_GAP_SET_DATA_CHANNEL_CLASSIFICATION_ID           = 0x19,     /*  */
    RSP_LE_GAP_SET_DISCOVERY_TIMING_ID                      = 0x16,     /*  */
    RSP_LE_GAP_SET_DISCOVERY_TYPE_ID                        = 0x17,     /*  */
    RSP_LE_GAP_SET_MODE_ID                                  = 0x01,     /*  */
    RSP_LE_GAP_SET_PRIVACY_MODE_ID                          = 0x0D,     /*  */
    RSP_LE_GAP_SET_SCAN_PARAMETERS_ID                       = 0x06,     /*  */
    RSP_LE_GAP_START_ADVERTISING_ID                         = 0x14,     /*  */
    RSP_LE_GAP_START_DISCOVERY_ID                           = 0x18,     /*  */
    RSP_LE_GAP_STOP_ADVERTISING_ID                          = 0x15      /*  */
} eMesLeGapResponseID;
typedef enum {
    LE_GAP_NON_DISCOVERABLE                                 = 0,        /*  */
    LE_GAP_LIMITED_DISCOVERABLE                             = 1,        /*  */
    LE_GAP_GENERAL_DISCOVERABLE                             = 2,        /*  */
    LE_GAP_BROADCAST                                        = 3,        /*  */
    LE_GAP_USER_DATA                                        = 4         /*  */
} eLeGapDiscaverableMode;
typedef enum {
    LE_GAP_NON_CONNECTABLE                                  = 0,        /*  */
    LE_GAP_DIRECTED_CONNECTABLE                             = 1,        /*  */
    LE_GAP_UNDIRECTED_CONNECTABLE                           = 2,        /*  */
    LE_GAP_CONNECTABLE_SCANNABLE                            = 2,        /*  */
    LE_GAP_SCANNABLE_NON_CONNECTABLE                        = 3,        /*  */
    LE_GAP_CONNECTABLE_NON_SCANNABLE                        = 4        /*  */
} eLeGapConnectableMode;
typedef enum {
    EVT_LE_GAP_ADV_TIMEOUT_ID                               = 0x01,     /*  */
    EVT_LE_GAP_SCAN_REQUEST_ID                              = 0x02,     /*  */
    EVT_LE_GAP_SCAN_RESPONSE_ID                             = 0x00      /*  */
} eMesLeGapEventID;
/*  */

/* [SM] Security Manager */
typedef enum {
    RSP_SM_BONDING_CONFIRM_ID                               = 0x0E,     /*  */
    RSP_SM_CONFIGURE_ID                                     = 0x01,     /*  */
    RSP_SM_DELETE_BONDING_ID                                = 0x06,     /*  */
    RSP_SM_DELETE_BONDINGS_ID                               = 0x07,     /*  */
    RSP_SM_ENTER_PASSKEY_ID                                 = 0x08,     /*  */
    RSP_SM_INCREASE_SECURITY_ID                             = 0x04,     /*  */
    RSP_SM_LIST_ALL_BONDINGS_ID                             = 0x0B,     /*  */
    RSP_SM_PASSKEY_CONFIRM_ID                               = 0x09,     /*  */
    RSP_SM_SET_BONDABLE_MODE_ID                             = 0x00,     /*  */
    RSP_SM_SET_DEBUG_MODE_ID                                = 0x0F,     /*  */
    RSP_SM_SET_OOB_DATA_ID                                  = 0x0A,     /*  */
    RSP_SM_SET_PASSKEY_ID                                   = 0x10,     /*  */
    RSP_SM_SET_SC_REMOTE_OOB_DATA_ID                        = 0x12,     /*  */
    RSP_SM_STORE_BONDING_CONFIGURATION_ID                   = 0x02,     /*  */
    RSP_SM_USE_SC_OOB_ID                                    = 0x11      /*  */
} eMesSmResponseID;
typedef enum {
    EVT_SM_BONDED_ID                                        = 0x03,     /*  */
    EVT_SM_BONDING_FAILED_ID                                = 0x04,     /*  */
    EVT_SM_CONFIRM_BONDING_ID                               = 0x09,     /*  */
    EVT_SM_CONFIRM_PASSKEY_ID                               = 0x02,     /*  */
    EVT_SM_LIST_ALL_BONDINGS_COMPLETE_ID                    = 0x06,     /*  */
    EVT_SM_LIST_BONDING_ENTRY_ID                            = 0x05,     /*  */
    EVT_SM_PASSKEY_DISPLAY_ID                               = 0x00,     /*  */
    EVT_SM_PASSKEY_REQUEST_ID                               = 0x01      /*  */
} eMesSmEventID;
/*  */


/* [system] System (Response ID) */
typedef enum {
    RSP_SYSTEM_GET_BT_ADDRES_ID                             = 0x03,     /*  */
    RSP_SYSTEM_GET_COUNTERS_ID                              = 0x0F,     /*  */
    RSP_SYSTEM_GET_RANDOM_DATA_ID                           = 0x0B,     /*  */
    RSP_SYSTEM_HALT_ID                                      = 0x0C,     /*  */
    RSP_SYSTEM_HELLO_ID                                     = 0x00,     /*  */
    RSP_SYSTEM_SET_BT_ADDRESS_ID                            = 0x04,     /*  */
    RSP_SYSTEM_SET_DEVICE_NAME_ID                           = 0x0D,     /*  */
    RSP_SYSTEM_SET_TX_POWER_ID                              = 0x0A      /*  */
} eMesSystemResponseID;
typedef enum {
    EVT_SYSTEM_AWAKE_ID                                 = 0x04,     /*  */
    EVT_SYSTEM_BOOT_ID                                  = 0x00,     /*  */
    EVT_SYSTEM_ERROR_ID                                 = 0x06,     /*  */
    EVT_SYSTEM_EXTERNAL_SIGNAL_ID                       = 0x03,     /*  */
    EVT_SYSTEM_HARDWARE_ERROR_ID                        = 0x05      /*  */
} eMesSystemEventID;
/*  */

/* [test] testing */
typedef enum {
    RSP_TEST_DTM_END_ID                                     = 0x02,     /*  */
    RSP_TEST_DTM_RX_ID                                      = 0x01,     /*  */
    RSP_TEST_DTM_TX_ID                                      = 0x00      /*  */
} eMesTestResponseEventID;
typedef enum {
    EVT_TEST_DTM_COMPLETED_ID                               = 0x00      /*  */
} eMesTestEventID;
/*  */

/* [user] User message */
typedef enum {
    RSP_USER_MESSAGE_TO_TARGET_ID                           = 0x00      /*  */
} eMesUserResponseID;
typedef enum {
    EVT_USER_MESSAGE_TO_HOST_ID                             = 0x00      /*  */
} eMesUserEventID;
/*  */








/*
 * <Command>
 * [2] class
 * [3] method(message id)
 * 
 * <response>
 * [2] class
 * [3] method(message id)
 * 
 */

#define BGLIB_GAP_USER_DATA                                         4

#define BGLIB_GAP_UNDIRECTED_CONNECTABLE                            2


#define BGLIB_GAP_DISCOVER_GENERIC                                  1

#define BGLIB_GAP_AD_TYPE_FLAGS                                     1

#define BGLIB_GAP_AD_TYPE_SERVICES_128BIT_ALL                       7
#define BGLIB_GAP_AD_TYPE_LOCALNAME_SHORT                           8
#define BGLIB_GAP_AD_TYPE_LOCALNAME_COMPLETE                        9


typedef enum {
    SCAN_RSP_ADVERTISING_PACKETS                            = 0x00,     /*  */
    SCAN_RSP_SCAN_RESPONSE_PACKETS                          = 0x01,     /*  */
    SCAN_RSP_OTA_ADVERTISING_PACKETS                        = 0x02,     /*  */
    SCAN_RSP_OTA_SCAN_RESPONSE_PACKETS                      = 0x04      /*  */
} eScanRsp;




/* stdint.hq?*/
typedef uint8_t     uint8;
typedef uint16_t    uint16;
typedef uint32_t    uint32;
typedef int8_t      int8;
typedef int16_t     int16;
typedef int32_t     int32;
/*  */

typedef struct bd_addr_t {
    uint8 addr[ 6 ];
} bd_addr;
typedef bd_addr hwaddr;
/* */
typedef struct {
    uint8 len;
    uint8 data[];
} uint8array;
/*  */
typedef struct {
    uint8 len;
    uint8 data[];
} string;
/*  */
struct ble_header {
    uint8 type_bilen;
    uint8 lolen;
    uint8 clas;
    uint8 command;
};
/*  */


/* [coex] cooexistence interface */
#ifdef BGLIB_ENABLE_CMD_COEX_GET_COUNTERS
struct ble_msg_coex_get_counters_rsp_t {
    uint16      result;                 /* [4:5] */
    uint8array  counters;               /* [6~] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_COEX_SET_OPTIONS
struct ble_msg_coex_set_options_rsp_t {
    uint16      result;                 /* [4:5] */
} __attribute__((packed));
#endif
/*  */
/* [dfu] Device Firmware Upgrade */
#ifdef BGLIB_ENABLE_CMD_DFU_FLASH_SET_ADDRESS
struct ble_msg_dfu_flash_set_address_rsp_t {
    uint16      result;                 /* [4:5] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_DFU_FLASH_UPLOAD
struct ble_msg_dfu_flash_upload_rsp_t {
    uint16      result;                 /* [4:5] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_DFU_FLASH_UPLOAD_FINISH
struct ble_msg_dfu_flash_upload_finish_rsp_t {
    uint16      result;                 /* [4:5] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_DFU_RESET
    /* no response */
#endif
/*  */
#ifdef BGLIB_ENABLE_EVT_DFU_BOOT
struct ble_msg_dfu_boot_evt_t {
    uint32      version;                /* [4] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_EVT_DFU_BOOT_FAILURE
struct ble_msg_dfu_boot_failure_evt_t {
    uint16      reason;                 /* [4] */
} __attribute__((packed));
#endif
/*  */


/* [endpoint] Endpoint */
#ifdef BGLIB_ENABLE_CMD_ENDPOINT_CLOSE
struct ble_msg_endpoint_close_rsp_t {
    uint16  result;                     /* [4:5] */
    uint8   endpoint;                   /* [6] */
} __attribute__((packed));
#endif
/*  */

/* [flash] Peripheral Store */
#ifdef BGLIB_ENABLE_CMD_FLASH_PS_ERASE
struct ble_msg_flash_ps_erase_rsp_t {
    uint16  result;                     /* [4:5] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_FLASH_PS_ERASE_ALL
struct ble_msg_flash_ps_erase_all_rsp_t {
    uint16  result;                     /* [4:5] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_FLASH_PS_LOAD
struct ble_msg_flash_ps_load_rsp_t {
    uint16      result;                 /* [4:5] */
    uint8array  value;                  /* [6~] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_FLASH_PS_SAVE
struct ble_msg_flash_ps_save_rsp_t {
    uint16  result;                     /* [4:5] */
} __attribute__((packed));
#endif
/*  */

/* [gatt] Generic Attribute Profile */
#ifdef BGLIB_ENABLE_CMD_GATT_DISCOVER_CHARACTERISTICS
struct ble_msg_gatt_discover_characteristics_rsp_t {
    uint16  result;                     /* [4:5] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_GATT_DISCOVER_CHARACTERISTICS_BY_UUID
struct ble_msg_gatt_discover_characteristics_by_uuid_rsp_t {
    uint16  result;                     /* [4:5] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_GATT_DISCOVER_DESCRIPTORS
struct ble_msg_gatt_discover_descriptors_rsp_t {
    uint16  result;                     /* [4:5] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_GATT_DISCOVER_PRIMARY_SERVICES
struct ble_msg_gatt_discover_primary_services_rsp_t {
    uint16  result;                     /* [4:5] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_GATT_DISCOVER_PRIMARY_SERVICES_BY_UUID
struct ble_msg_gatt_discover_primary_services_by_uuid_rsp_t {
    uint16  result;                     /* [4:5] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_GATT_EXECUTE_CHARACTERISTIC_VALUE_WRITE
struct ble_msg_gatt_gatt_execute_characteristic_value_write_rsp_t {
    uint16  result;                     /* [4:5] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_GATT_FIND_INCLUDED_SERVICES
struct ble_msg_gatt_gatt_find_include_services_rsp_t {
    uint16  result;                     /* [4:5] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_GATT_PREPARE_CHARACTERISTIC_VALUE_RELIABLE_WRITE
struct ble_msg_gatt_gatt_prepare_characteristic_value_reliable_write_rsp_t {
    uint16  result;                     /* [4:5] */
    uint16  sent_len;                   /* [6:7] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_GATT_PREPARE_CHARACTERISTIC_VALUE_WRITE
struct ble_msg_gatt_gatt_prepare_characteristic_value_write_rsp_t {
    uint16  result;                     /* [4:5] */
    uint16  sent_len;                   /* [6:7] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_GATT_READ_CHARACTERISTIC_VALUE
struct ble_msg_gatt_read_characteristic_value_rsp_t {
    uint16  result;                     /* [4:5] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_GATT_READ_CHARACTERISTIC_VALUE_BY_UUID
struct ble_msg_gatt_read_characteristic_value_by_uuid_rsp_t {
    uint16  result;                     /* [4:5] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_GATT_READ_CHARACTERICTIC_VALUE_FROM_OFFSET
struct ble_msg_gatt_read_characteristic_value_from_offset_rsp_t {
    uint16  result;                     /* [4:5] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_GATT_READ_DESCRIPTOR_VALUE
struct ble_msg_gatt_read_descriptor_value_rsp_t {
    uint16  result;                     /* [4:5] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_GATT_READ_MULTIPLE_CHARACTERISTIC_VALUES
struct ble_msg_gatt_read_multiple_characteristic_values_rsp_t {
    uint16  result;                     /* [4:5] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_GATT_SEND_CHARACTERICTIC_CONFIRMATION
struct ble_msg_gatt_send_characteristic_confirmation_rsp_t {
    uint16  result;                     /* [4:5] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_GATT_SET_CHARACTERISTIC_NOTIFICATION
struct ble_msg_gatt_set_characteristic_notification_rsp_t {
    uint16  result;                     /* [4:5] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_GATT_SET_MAX_MTU
struct ble_msg_gatt_set_max_mtu_rsp_t {
    uint16  result;                     /* [4:5] */
    uint16  max_mtu;                    /* [6:7] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_GATT_WRITE_CHARACTERISTIC_VALUE
struct ble_msg_gatt_write_characteristic_value_rsp_t {
    uint16  result;                     /* [4:5] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_GATT_WRITE_CHARACTERISTIC_VALUE_WITHOUT_RESPONSE
struct ble_msg_gatt_write_characteristic_value_without_response_rsp_t {
    uint16  result;                     /* [4:5] */
    uint16  sent_len;                   /* [6:7] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_GATT_WRITE_DESCRIPTOR_VALUE
struct ble_msg_gatt_write_descriptor_value_rsp_t {
    uint16  result;                     /* [4:5] */
} __attribute__((packed));
#endif
/*  */
#ifdef BGLIB_ENABLE_EVT_GATT_CHARACTERISTIC
struct ble_msg_gatt_characteristic_evt_t {
    uint8       connection;             /* [4] */
    uint16      characteristic;         /* [5~6] */
    uint8       properties;             /* [7] */
    uint8array  uuid;                   /* [8~] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_EVT_GATT_CHARACTERISTIC_VALUE
struct ble_msg_gatt_characteristic_value_evt_t {
    uint8       connection;             /* [4] */
    uint16      characteristic;         /* [5~6] */
    uint8       att_opcode;             /* [7] */
    uint16      offset;                 /* [8~9] */
    uint8array  value;                  /* [10~] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_EVT_GATT_DESCRIPTOR
struct ble_msg_gatt_descriptor_evt_t {
    uint8       connection;             /* [4] */
    uint16      descriptor;             /* [5~6] */
    uint8array  uuid;                   /* [7] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_EVT_GATT_DESCRIPTOR_VALUE
struct ble_msg_gatt_descriptor_value_evt_t {
    uint8       connection;             /* [4] */
    uint16      descriptor;             /* [5~6] */
    uint16      offset;                 /* [7~8] */
    uint8array  value;                  /* [9~] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_EVT_GATT_MTU_EXCHANGED
struct ble_msg_gatt_mtu_exchanged_evt_t {
    uint8       connection;             /* [4] */
    uint16      mtu;                    /* [5~6] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_EVT_GATT_PROCEDURE_COMPLETED
struct ble_msg_gatt_procedure_completed_evt_t {
    uint8       connection;             /* [4] */
    uint16      result;                 /* [5~6] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_EVT_GATT_SERVICE
struct ble_msg_gatt_service_evt_t {
    uint8       connection;             /* [4] */
    uint32      service;                /* [5~8] */
    uint8array  uuid;                   /* [9~] */
} __attribute__((packed));
#endif
/*  */

/* [gatt_server] Generic Attribure Profile Server */
#ifdef BGLIB_ENABLE_CMD_GATT_SERVER_FIND_ATTRIBUTE
struct ble_msg_gatt_server_find_attribute_rsp_t {
    uint16  result;                     /* [4:5] */
    uint16  attribute;                  /* [6:7] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_GATT_SERVER_READ_ATTRIBUTE_TYPE
struct ble_msg_gatt_server_read_attribute_type_rsp_t {
    uint16      result;                 /* [4:5] */
    uint8array  type;                   /* [6~] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_GATT_SERVER_READ_ATTRIBUTE_VALUE
struct ble_msg_gatt_server_read_attribute_balue_rsp_t {
    uint16      result;                 /* [4:5] */
    uint8array  type;                   /* [6~] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_GATT_SERVER_SEND_CHARACTERISTIC_NOTIFICATION
struct ble_msg_gatt_server_send_characteristic_notification_rsp_t {
    uint16  result;                     /* [4:5] */
    uint16  sent_len;                   /* [6:7] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_GATT_SERVER_SEND_USER_READ_RESPONSE
struct ble_msg_gatt_server_send_user_read_response_rsp_t {
    uint16  result;                     /* [4:5] */
    uint16  sent_len;                   /* [6:7] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_GATT_SERVER_SEND_USER_WRITE_RESPONSE
struct ble_msg_gatt_server_send_user_write_response_rsp_t {
    uint16  result;                     /* [4:5] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_GATT_SERVER_SENT_CAPABILITIES
struct ble_msg_gatt_server_sent_capabilities_rsp_t {
    uint16  result;                     /* [4:5] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_GATT_SERVER_SET_DATABASE
struct ble_msg_gatt_server_set_database_rsp_t {
    uint16  result;                     /* [4:5] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_GATT_SERVER_WRITE_ATTRIBUTE_VALUE
struct ble_msg_gatt_server_write_arttribute_value_rsp_t {
    uint16  result;                     /* [4:5] */
} __attribute__((packed));
#endif
/*  */
#ifdef BGLIB_ENABLE_EVT_GATT_SERVER_ATTRIBUTE_VALUE
struct ble_msg_gatt_server_attribute_value_evt_t {
    uint8       connection;             /* [4] */
    uint16      attribute;              /* [5~6] */
    uint8       att_opcode;             /* [7] */
    uint16      offset;                 /* [8~9] */
    uint8array  value;                  /* [10~] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_EVT_GATT_SERVER_CHARACTERISTIC_STATUS
struct ble_msg_gatt_server_characteristic_status_evt_t {
    uint8       connection;             /* [4] */
    uint16      characteristic;         /* [5~6] */
    uint8       status_flags;           /* [7] */
    uint16      client_config_flags;    /* [8~9] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_EVT_GATT_SERVER_EXECUTE_WRITE_COMPLETED
struct ble_msg_gatt_server_execute_write_completed_evt_t {
    uint8       connection;             /* [4] */
    uint16      result;                 /* [5~6] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_EVT_GATT_SERVER_USER_READ_REQUEST
struct ble_msg_gatt_server_user_read_request_evt_t {
    uint8       connection;             /* [4] */
    uint16      characteristic;         /* [5~6] */
    uint8       att_opcode;             /* [7] */
    uint16      offset;                 /* [8~9] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_EVT_GATT_SERVER_USER_WRITE_REQUEST
struct ble_msg_gatt_server_user_write_request_evt_t {
    uint8       connection;             /* [4] */
    uint16      characteristic;         /* [5~6] */
    uint8       att_opcode;             /* [7] */
    uint16      offset;                 /* [8~9] */
    uint8array  value;                  /* [10~] */
} __attribute__((packed));
#endif
/*  */


/* [hardware] Hardware */
#ifdef BGLIB_ENABLE_CMD_HARDWARE_ENABLE_DCDC
struct ble_msg_hardware_enable_dcdc_rsp_t {
    uint16  result;                     /* [4:5] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_HARDWARE_GET_TIME
struct ble_msg_hardware_get_time_rsp_t {
    uint32  seconds;                    /* [4:7] */
    uint32  ticks;                      /* [8:9] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_HARDWARE_SET_LAZY_SOFT_TIMER
struct ble_msg_hardware_set_lazy_soft_timer_rsp_t {
    uint16  result;                     /* [4:5] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_HARDWARE_SET_SOFT_TIMER
struct ble_msg_hardware_set_soft_timer_rsp_t {
    uint16  result;                     /* [4:5] */
} __attribute__((packed));
#endif
/*  */
#ifdef BGLIB_ENABLE_EVT_HARDWARE_SOFT_TIMER
struct ble_msg_hardware_soft_timer_evt_t {
    uint8       handle;                 /* [4] */
} __attribute__((packed));
#endif
/*  */


/* [le connection] Connection management */
#ifdef BGLIB_ENABLE_CMD_LE_CONNECTION_CLOSE
struct ble_msg_le_connection_close_rsp_t {
    uint16  result;                     /* [4:5] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_LE_CONNECTION_DISABLE_SLAVE_LATENCY
struct ble_msg_le_connection_disable_slave_latancy_rsp_t {
    uint16  result;                     /* [4:5] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_LE_CONNECTION_GET_RSSI
struct ble_msg_le_connection_get_rssi_rsp_t {
    uint16  result;                     /* [4:5] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_LE_CONNECTION_SET_PARAMETERS
struct ble_msg_le_connection_set_parameters_rsp_t {
    uint16  result;                     /* [4:5] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_LE_CONNECTION_SET_PHY
struct ble_msg_le_connection_set_phy_rsp_t {
    uint16  result;                     /* [4:5] */
} __attribute__((packed));
#endif
/*  */
#ifdef BGLIB_ENABLE_EVT_LE_CONNECTION_CLOSED
struct ble_msg_le_connection_closed_evt_t {
    uint16      reason;             /* [4~5] */
    uint8       connection;         /* [6] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_EVT_LE_CONNECTION_OPENED
struct ble_msg_le_connection_opend_evt_t {
    bd_addr     address;            /* [4~9] */
    uint8       address_type;       /* [10] */
    uint8       master;             /* [11] */
    uint8       connection;         /* [12] */
    uint8       bonding;            /* [13] */
    uint8       advertiser;         /* [14] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_EVT_LE_CONNECTION_PARAMETERS
struct ble_msg_le_connection_parameters_evt_t {
    uint8       connection;         /* [4] */
    uint16      interval;           /* [5~6] */
    uint16      latency;            /* [7~8] */
    uint16      timeout;            /* [9~10] */
    uint8       security_mode;      /* [11] */
    uint16      tzsize;             /* [12~13] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_EVT_LE_CONNECTION_PHY_STATUS
struct ble_msg_le_connection_phy_status_evt_t {
    uint8       connection;         /* [4] */
    uint8       phy;                /* [5] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_EVT_LE_CONNECTION_RSSI
struct ble_msg_le_connection_rssi_evt_t {
    uint8       connection;         /* [4] */
    uint8       status;             /* [5] */
    uint8       rssi;               /* [6] */
} __attribute__((packed));
#endif
/*  */

/* [le gap] Generic Access Profile commcnds */
#ifdef BGLIB_ENABLE_CMD_LE_GAP_BT5_SET_ADV_DATA
struct ble_msg_le_gap_bt5_set_adv_data_rsp_t {
    uint16  result;                     /* [4:5] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_LE_GAP_BT5_SET_ADV_PARAMETERS
struct ble_msg_le_gap_bt5_set_adv_parameters_rsp_t {
    uint16  result;                     /* [4:5] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_LE_GAP_BT5_SET_MODE
struct ble_msg_le_gap_bt5_set_mode_rsp_t {
    uint16  result;                     /* [4:5] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_LE_GAP_CLEAR_ADVERTISE_CONFIGURATION
struct ble_msg_le_gap_clear_advertise_configuration_rsp_t {
    uint16  result;                     /* [4:5] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_LE_GAP_CONNECT
struct ble_msg_le_gap_gap_connect_rsp_t {
    uint16  result;                     /* [4:5] */
    uint8   connection;                 /* [6] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_LE_GAP_DISCOVER
struct ble_msg_le_gap_discover_rsp_t {
    uint16  result;                     /* [4:5] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_LE_GAP_END_PROCEDURE
struct ble_msg_le_gap_end_procedure_rsp_t {
    uint16  result;                     /* [4:5] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_LE_GAP_OPEN
struct ble_msg_le_gap_open_rsp_t {
    uint16  result;                     /* [4:5] */
    uint8   connection;                 /* [6] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_LE_GAP_SET_ADV_DATA
struct ble_msg_le_gap_set_adv_data_rsp_t {
    uint16  result;                     /* [4:5] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_LE_GAP_SET_ADV_PARAMETERS
struct ble_msg_le_gap_set_adv_parameters_rsp_t {
    uint16  result;                     /* [4:5] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_LE_GAP_SET_ADV_TIMEOUT
struct ble_msg_le_gap_set_adv_timeout_rsp_t {
    uint16  result;                     /* [4:5] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_LE_GAP_SET_ADVERTISE_CHANNEL_MAP
struct ble_msg_le_gap_set_advertise_channel_map_rsp_t {
    uint16  result;                     /* [4:5] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_LE_GAP_SET_ADVERTISE_CONFIGURATION
struct ble_msg_le_gap_set_advertise_configuration_rsp_t {
    uint16  result;                     /* [4:5] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_LE_GAP_SET_ADVERTISE_PHY
struct ble_msg_le_gap_set_advertise_phy_rsp_t {
    uint16  result;                     /* [4:5] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_LE_GAP_SET_ADVERTISE_REPORT_SCAN_REQUEST
struct ble_msg_le_gap_set_advertise_report_scan_request_rsp_t {
    uint16  result;                     /* [4:5] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_LE_GAP_SET_ADVERTISE_TIMING
struct ble_msg_le_gap_set_advertise_timing_rsp_t {
    uint16  result;                     /* [4:5] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_LE_GAP_SET_CONN_PARAMEETERS
struct ble_msg_le_gap_set_conn_parameters_rsp_t {
    uint16  result;                     /* [4:5] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_LE_GAP_SET_DATA_CHANNEL_CLASSIFICATION
struct ble_msg_le_gap_set_data_channel_classification_rsp_t {
    uint16  result;                     /* [4:5] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_LE_GAP_SET_DISCOVERY_TIMING
struct ble_msg_le_gap_set_discovery_timing_rsp_t {
    uint16  result;                     /* [4:5] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_LE_GAP_SET_DISCOVERY_TYPE
struct ble_msg_le_gap_set_discovery_type_rsp_t {
    uint16  result;                     /* [4:5] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_LE_GAP_SET_MODE
struct ble_msg_le_gap_set_mode_rsp_t {
    uint16  result;                     /* [4:5] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_LE_GAP_SET_PRIVACY_MODE
struct ble_msg_le_gap_set_privacy_mode_rsp_t {
    uint16  result;                     /* [4:5] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_LE_GAP_SET_SCAN_PARAMETERS
struct ble_msg_le_gap_set_scan_parameters_rsp_t {
    uint16  result;                     /* [4:5] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_LE_GAP_START_ADVERTISING
struct ble_msg_le_gap_start_advertising_rsp_t {
    uint16  result;                     /* [4:5] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_LE_GAP_START_DISCOVERY
struct ble_msg_le_gap_start_discovery_rsp_t {
    uint16  result;                     /* [4:5] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_LE_GAP_STOP_ADVERTISING
struct ble_msg_le_gap_stop_advertising_rsp_t {
    uint16  result;                     /* [4:5] */
} __attribute__((packed));
#endif
/*  */
#ifdef BGLIB_ENABLE_EVT_LE_GAP_ADV_TIMEOUT
struct ble_msg_le_gap_adv_timeout_evt_t {
    uint8       handle;             /* [4] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_EVT_LE_GAP_SCAN_REQUEST
struct ble_msg_le_gap_scan_request_evt_t {
    uint8       handle;             /* [4] */
    bd_addr     address;            /* [5~10] */
    uint8       address_type;       /* [11] */
    uint8       bonding;            /* [12] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_EVT_LE_GAP_SCAN_RESPONSE
struct ble_msg_le_gap_scan_response_evt_t {
    int8        rssi;               /* [4] */
    uint8       packet_type;        /* [5] */
    bd_addr     sender;             /* [6~11] */
    uint8       address_type;       /* [12] */
    uint8       bond;               /* [13] */
    uint8array  data;               /* [14] */
} __attribute__((packed));
#endif
/*  */


/* [SM] Security Manager */
#ifdef BGLIB_ENABLE_CMD_SM_BONDING_CONFIRM
struct ble_msg_sm_bonding_confirm_rsp_t {
    uint16  result;                     /* [4:5] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_SM_CONFIGURE
struct ble_msg_sm_configure_rsp_t {
    uint16  result;                     /* [4:5] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_SM_DELETE_BONDING
struct ble_msg_sm_delete_bonding_rsp_t {
    uint16  result;                     /* [4:5] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_SM_DELETE_BONDINGS
struct ble_msg_sm_delete_bondings_rsp_t {
    uint16  result;                     /* [4:5] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_SM_ENTER_PASSKEY
struct ble_msg_sm_enter_passkey_rsp_t {
    uint16  result;                     /* [4:5] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_SM_INCREASE_SECURITY
struct ble_msg_sm_increase_security_rsp_t {
    uint16  result;                     /* [4:5] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_SM_LIST_ALL_BONDINGS
struct ble_msg_sm_list_all_bondings_rsp_t {
    uint16  result;                     /* [4:5] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_SM_PASSKEY_CONFIRM
struct ble_msg_sm_passkey_confirm_rsp_t {
    uint16  result;                     /* [4:5] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_SM_SET_BONDABLE_MODE
//[åú¡¦        ble_rsp_sm_set_bondable_mode = 0;
struct ble_msg_sm_set_bondable_mode_rsp_t {
    uint16  result;                     /* [4:5] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_SM_SET_DEBUG_MODE
struct ble_msg_sm_set_debug_mode_rsp_t {
    uint16  result;                     /* [4:5] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_SM_SET_OOB_DATA
struct ble_msg_sm_set_oob_data_rsp_t {
    uint16  result;                     /* [4:5] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_SM_SET_PASSKEY
//[åú¡¦        ble_rsp_sm_set_passkey = 0;
struct ble_msg_sm_set_passkey_rsp_t {
    uint16  result;                     /* [4:5] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_SM_SET_SC_REMOTE_OOB_DATA
struct ble_msg_sm_set_sc_remote_oob_data_rsp_t {
    uint16  result;                     /* [4:5] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_SM_STORE_BONDING_CONFIGURATION
struct ble_msg_sm_store_bonding_configuration_rsp_t {
    uint16  result;                     /* [4:5] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_SM_USE_SC_OOB
struct ble_msg_sm_use_sc_oob_rsp_t {
    uint16      result;                 /* [4:5] */
    uint8array  oob_data;               /* [6] */
} __attribute__((packed));
#endif
/*  */
#ifdef BGLIB_ENABLE_EVT_SM_BONDED
struct ble_msg_sm_bonded_evt_t {
    uint8   connection;                 /* [4] */
    uint8   bonding;                    /* [5]  */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_EVT_SM_BONDING_FAILED
struct ble_msg_sm_bonding_failed_evt_t {
    uint8   connection;                 /* [4] */
    uint16  reason;                     /* [5:6]  */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_EVT_SM_CONFIRM_BONDING
struct ble_msg_sm_confirm_bonding_evt_t {
    uint8   connection;                 /* [4] */
    int8    bonding_handle;             /* [5]  */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_EVT_SM_CONFIRM_PASSKEY
struct ble_msg_sm_confirm_passkey_evt_t {
    uint8   connection;                 /* [4] */
    uint32  passkey;                    /* [5:8]  */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_EVT_SM_LIST_ALL_BONDINGS_COMPLETE
    /* none */
#endif
#ifdef BGLIB_ENABLE_EVT_SM_LIST_BONDING_ENTRY
struct ble_msg_sm_list_bonding_entry_evt_t {
    uint8   bonding;                    /* [4] */
    bd_addr address;                    /* [5:10]  */
    uint8   address_type;               /* [11]  */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_EVT_SM_PASSKEY_DISPLAY
struct ble_msg_sm_passkey_display_evt_t {
    uint8   connection;                 /* [4] */
    uint32  passkey;                    /* [5:8]  */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_EVT_SM_PASSKEY_REQUEST
struct ble_msg_sm_passkey_request_evt_t {
    uint8   connection;                 /* [4] */
} __attribute__((packed));
#endif
/*  */


/* [system] System commands */
#ifdef BGLIB_ENABLE_CMD_SYSTEM_GET_BT_ADDRESS
struct ble_msg_system_get_bt_address_rsp_t {
    bd_addr address;                    /* [4:9] Bluetooth public address in Little-Endian format */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_SYSTEM_GET_COUNTERS
struct ble_msg_system_get_counters_rsp_t {
    uint16  result;                     /* [4:5]   Result code */
    uint16  tx_packets;                 /* [6:7]    */
    uint16  rx_packets;                 /* [8:9]    */
    uint16  crc_errors;                 /* [10:11]  */
    uint16  failures;                   /* [12:13]  */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_SYSTEM_GET_RANDOM_DATA
struct ble_msg_system_get_random_data_rsp_t {
    uint16      result;                 /* [4:5]   Result code */
    uint8array  data;                   /* [6] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_SYSTEM_HALT
struct ble_msg_system_halt_rsp_t {
    uint16  result;                     /* [4:5]   Result code */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_SYSTEM_HELLO
struct ble_msg_system_hello_rsp_t {
    uint16  result;                     /* [4:5]   Result code */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_SYSTEM_RESET
struct ble_msg_system_reset_cmd_t {
    uint8   boot_in_dfu;                /* [4] */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_SYSTEM_SET_BT_ADDRESS
struct ble_msg_system_set_bt_address_cmd_t {
    bd_addr address;                    /* [4:9] address */
} __attribute__((packed));
struct ble_msg_system_set_bt_address_rsp_t {
    uint16  result;                     /* [4:5]   Result code */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_SYSTEM_SET_DEVICE_NAME
struct ble_msg_system_set_device_name_cmd_t {
    uint8       type;                   /* [4] */
    uint8array  name;                   /* [5] Device name */
} __attribute__((packed));
struct ble_msg_system_set_device_name_rsp_t {
    uint16  result;                     /* [4:5]   Result code */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_SYSTEM_SET_TX_POWER
struct ble_msg_system_set_tx_power_rsp_t {
    int16   set_power;                  /* [4:5] */
} __attribute__((packed));
#endif
/*  */
#ifdef BGLIB_ENABLE_EVT_SYSTEM_AWAKE
#endif
#ifdef BGLIB_ENABLE_EVT_SYSTEM_BOOT
struct ble_msg_system_boot_evt_t {
    uint16 major;                       /* [4:5] Major release version */
    uint16 minor;                       /* [6:7] Minor release version */
    uint16 patch;                       /* [8:9] Patch release number */
    uint16 build;                       /* [10:11] Build number */
    uint32 bootloader;                  /* [12:15] Bootloader version */
    uint16 hw;                          /* [16:17] Hardware type */
    uint32 hash;                        /* [18:21] Version hash */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_EVT_SYSTEM_ERROR
struct ble_msg_system_error_evt_t {
    uint16      reason;                 /* [4:5] */
    uint8array  name;                   /* [6]  */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_EVT_SYSTEM_EXTERNAL_SIGNAL
struct ble_msg_system_external_signal_evt_t {
    uint32  extsignals;                 /* [4:7]  */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_EVT_SYSTEM_HARDWARE_ERROR
struct ble_msg_system_hardware_error_evt_t {
    uint16  status;                     /* [4:5] Result code */
} __attribute__((packed));
#endif
/*  */


/* [test] testing */
#ifdef BGLIB_ENABLE_CMD_TEST_DTM_END
struct ble_msg_test_dtm_end_rsp_t {
    uint16  status;                     /* [4:5] Result code */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_TEST_DTM_RX
struct ble_msg_test_dtm_rx_rsp_t {
    uint16  status;                     /* [4:5] Result code */
} __attribute__((packed));
#endif
#ifdef BGLIB_ENABLE_CMD_TEST_DTM_TX
struct ble_msg_test_dtm_tx_rsp_t {
    uint16  status;                     /* [4:5] Result code */
} __attribute__((packed));
#endif
/*  */
#ifdef BGLIB_ENABLE_EVT_TEST_DTM_COMPLETED
struct ble_msg_test_dtm_completed_evt_t {
    uint16  result;                     /* [4:5] Result code */
    uint16  number_of_packets;          /* [6:7] Number of packets */
} __attribute__((packed));
#endif
/*  */


/* [user] User message */
#ifdef BGLIB_ENABLE_CMD_USER_MESSAGE_TO_TARGET
struct ble_msg_user_message_to_target_rsp_t {
    uint16      result;                 /* [4:5]   Result code */
    uint8array  data;                   /* [5] response message */
} __attribute__((packed));
#endif
/*  */
#ifdef BGLIB_ENABLE_EVT_USER_MASSAGE_TO_HOST
struct ble_msg_user_message_to_host_evt_t {
    uint8array  data;                   /* [6]  */
} __attribute__((packed));
#endif
/*  */




class BGLib {
    public:
        BGLib( HardwareSerial *module=0, HardwareSerial *output = 0, uint8_t pMode = 0 );
        uint8_t checkActivity( uint16_t timeout = 0 );
        uint8_t checkError( void );
        uint8_t checkTimeout( void );
        void setBusy( bool busyEnabled );
        /*  */
        uint8_t *getLastCommand( void );
        uint8_t *getLastResponse( void );
        uint8_t *getLastEvent( void );
        void *getLastRXPayload( void );
        /*  */

        // set/update UART port objects
        void setModuleUART( HardwareSerial *module );
        void setOutputUART( HardwareSerial *debug );
        /*  */
        uint8_t parse( uint8_t ch, uint8_t packetMode = 0 );
        uint8_t sendCommand( uint16_t len, uint8_t commandClass, uint8_t commandId, void *payload = 0 );
        /*  */
        void (*onBusy)( void );               // special function to run when entering a "busy" state (e.g. mid-packet)
        void (*onIdle)( void );               // special function to run when returning to idle mode
        void (*onTimeout)( void );            // special function to run when the parser times out waiting for expected data
        void (*onBeforeTXCommand)( void );    // special function to run immediately before sending a command
        void (*onTXCommandComplete)( void );  // special function to run immediately after command transmission is complete
        /*  */



        /* [coex] cooexistence interface */
        #ifdef BGLIB_ENABLE_CMD_COEX_GET_COUNTERS
            uint8_t ble_cmd_coex_get_counters( uint8 reset );
            void (*ble_rsp_coex_get_counters)( const struct ble_msg_coex_get_counters_rsp_t *msg );
        #endif
        #ifdef BGLIB_ENABLE_CMD_COEX_SET_OPTIONS
            uint8_t ble_cmd_coex_set_options( uint32 mask, uint32 options );
            void (*ble_rsp_coex_set_options)( const struct ble_msg_coex_set_options_rsp_t *msg );
        #endif
        /*  */

        /* [dfu] Device Firmware Upgrade */
        #ifdef BGLIB_ENABLE_CMD_DFU_FLASH_SET_ADDRESS
            uint8_t ble_cmd_dfu_flash_set_address( uint32 address );
            void (*ble_rsp_dfu_flash_set_address)( const struct ble_msg_dfu_flash_set_address_rsp_t *msg );
        #endif
        #ifdef BGLIB_ENABLE_CMD_DFU_FLASH_UPLOAD
            uint8_t ble_cmd_dfu_flash_upload( uint8 data_len, const uint8 *data_data );
            void (*ble_rsp_dfu_flash_upload)( const struct ble_msg_dfu_flash_upload_rsp_t *msg );
        #endif
        #ifdef BGLIB_ENABLE_CMD_DFU_FLASH_UPLOAD_FINISH
            uint8_t ble_cmd_dfu_flash_upload_finish( void );
            void (*ble_rsp_dfu_flash_upload_finish)( const struct ble_msg_dfu_flash_upload_finish_rsp_t *msg );
        #endif
        #ifdef BGLIB_ENABLE_CMD_DFU_RESET
            uint8_t ble_cmd_dfu_reset( uint8 dfu );
            /* no response */
        #endif
        /*  */
        #ifdef BGLIB_ENABLE_EVT_DFU_BOOT
            void (*ble_evt_dfu_boot)(const struct ble_msg_dfu_boot_evt_t *msg);
        #endif
        #ifdef BGLIB_ENABLE_EVT_DFU_BOOT_FAILURE
            void (*ble_evt_dfu_boot_failure)(const struct ble_msg_dfu_boot_failure_evt_t *msg);
        #endif
        /*  */

        /* [endpoint] Endpoint */
        #ifdef BGLIB_ENABLE_CMD_ENDPOINT_CLOSE
            uint8_t ble_cmd_endpoint_close( uint8 endpoint );
            void (*ble_rsp_endpoint_close)( const struct ble_msg_endpoint_close_rsp_t *msg );
        #endif
        /*  */

        /* [flash] Peripheral Store */
        #ifdef BGLIB_ENABLE_CMD_FLASH_PS_ERASE
            uint8_t ble_cmd_flash_ps_erase( uint16 key );
            void (*ble_rsp_flash_ps_erase)( const struct ble_msg_flash_ps_erase_rsp_t *msg );
        #endif
        #ifdef BGLIB_ENABLE_CMD_FLASH_PS_ERASE_ALL
            uint8_t ble_cmd_flash_ps_erase_all( void );
            void (*ble_rsp_flash_ps_erase_all)( const struct ble_msg_flash_ps_erase_all_rsp_t *msg );
        #endif
        #ifdef BGLIB_ENABLE_CMD_FLASH_PS_LOAD
            uint8_t ble_cmd_flash_ps_load( uint16 key );
            void (*ble_rsp_flash_ps_load)( const struct ble_msg_flash_ps_load_rsp_t *msg );
        #endif
        #ifdef BGLIB_ENABLE_CMD_FLASH_PS_SAVE
            uint8_t ble_cmd_flash_ps_save( uint16 key, uint8 value_len, const uint8 *value_data );
            void (*ble_rsp_flash_ps_save)( const struct ble_msg_flash_ps_save_rsp_t *msg );
        #endif
        /*  */

        /* [gatt] Generic Attribute Profile */
        #ifdef BGLIB_ENABLE_CMD_GATT_DISCOVER_CHARACTERISTICS
            uint8_t ble_cmd_gatt_discover_characteristics( uint8 connection, uint32 service );
            void (*ble_rsp_gatt_discover_characteristics)( const struct ble_msg_gatt_discover_characteristics_rsp_t *msg );
        #endif
        #ifdef BGLIB_ENABLE_CMD_GATT_DISCOVER_CHARACTERISTICS_BY_UUID
            uint8_t ble_cmd_gatt_discover_characteristics_by_uuid( uint8 connection, uint32 service, uint8 uuid_len, const uint8 *uuid_data );
            void (*ble_rsp_gatt_discover_characteristics_by_uuid)( const struct ble_msg_gatt_discover_characteristics_by_uuid_rsp_t *msg );
        #endif
        #ifdef BGLIB_ENABLE_CMD_GATT_DISCOVER_DESCRIPTORS
            uint8_t ble_cmd_gatt_discover_descriptors( uint8 connection, uint16 characteristic );
            void (*ble_rsp_gatt_discover_descriptors)( const struct ble_msg_gatt_discover_descriptors_rsp_t *msg );
        #endif
        #ifdef BGLIB_ENABLE_CMD_GATT_DISCOVER_PRIMARY_SERVICES
            uint8_t ble_cmd_gatt_discover_primary_services( uint8 connection );
            void (*ble_rsp_gatt_discover_primary_services)( const struct ble_msg_gatt_discover_primary_services_rsp_t *msg );
        #endif
        #ifdef BGLIB_ENABLE_CMD_GATT_DISCOVER_PRIMARY_SERVICES_BY_UUID
            uint8_t ble_cmd_gatt_discover_primary_services_by_uuid( uint8 connection, uint8 uuid_len, const uint8 *uuid_data );
            void (*ble_rsp_gatt_discover_primary_services_by_uuid)( const struct ble_msg_gatt_discover_primary_services_by_uuid_rsp_t *msg );
        #endif
        #ifdef BGLIB_ENABLE_CMD_GATT_EXECUTE_CHARACTERISTIC_VALUE_WRITE
            uint8_t ble_cmd_gatt_execute_characteristic_value_write( uint8 connection, uint8 flags );
            void (*ble_rsp_gatt_execute_characteristic_value_write)( const struct ble_msg_gatt_execute_characteristic_value_write_rsp_t *msg );
        #endif
        #ifdef BGLIB_ENABLE_CMD_GATT_FIND_INCLUDED_SERVICES
            uint8_t ble_cmd_gatt_find_include_services( uint8 connection, uint32 service );
            void (*ble_rsp_gatt_find_include_services)( const struct ble_msg_gatt_find_include_services_rsp_t *msg );
        #endif
        #ifdef BGLIB_ENABLE_CMD_GATT_PREPARE_CHARACTERISTIC_VALUE_RELIABLE_WRITE
            uint8_t ble_cmd_gatt_prepare_characteristic_value_reliable_write( uint8 connection, uint16 characteristic, uint16 offset, uint16 value_len, const uint8 *value_data );
            void (*ble_rsp_gatt_prepare_characteristic_value_reliable_write)( const struct ble_msg_gatt_prepare_characteristic_value_reliable_write_rsp_t *msg );
        #endif
        #ifdef BGLIB_ENABLE_CMD_GATT_PREPARE_CHARACTERISTIC_VALUE_WRITE
            uint8_t ble_cmd_gatt_prepare_characteristic_value_write( uint8 connection, uint16 characteristic, uint16 offset, uint16 value_len, const uint8 *value_data );
            void (*ble_rsp_gatt_prepare_characteristic_value_write)( const struct ble_msg_gatt_prepare_characteristic_value_write_rsp_t *msg );
        #endif
        #ifdef BGLIB_ENABLE_CMD_GATT_READ_CHARACTERISTIC_VALUE
            uint8_t ble_cmd_gatt_read_characteristic_value( uint8 connection, uint16 characteristic );
            void (*ble_rsp_gatt_read_characteristic_value)( const struct ble_msg_gatt_read_characteristic_value_rsp_t *msg );
        #endif
        #ifdef BGLIB_ENABLE_CMD_GATT_READ_CHARACTERISTIC_VALUE_BY_UUID
            uint8_t ble_cmd_gatt_read_characteristic_value_by_uuid( uint8 connection, uint32 service, uint8 uuid_len, const uint8 *uuid_data );
            void (*ble_rsp_gatt_read_characteristic_value_by_uuid)( const struct ble_msg_gatt_read_characteristic_value_by_uuid_rsp_t *msg );
        #endif
        #ifdef BGLIB_ENABLE_CMD_GATT_READ_CHARACTERICTIC_VALUE_FROM_OFFSET
            uint8_t ble_cmd_gatt_read_characteristic_value_from_offset( uint8 connection, uint16 characteristic, uint16 offset, uint16 maxlen );
            void (*ble_rsp_gatt_read_characteristic_value_from_offset)( const struct ble_msg_gatt_read_characteristic_value_from_offset_rsp_t *msg );
        #endif
        #ifdef BGLIB_ENABLE_CMD_GATT_READ_DESCRIPTOR_VALUE
            uint8_t ble_cmd_gatt_read_descriptor_value( uint8 connection, uint16 descriptor );
            void (*ble_rsp_gatt_read_descriptor_value)( const struct ble_msg_gatt_read_descriptor_value_rsp_t *msg );
        #endif
        #ifdef BGLIB_ENABLE_CMD_GATT_READ_MULTIPLE_CHARACTERISTIC_VALUES
            uint8_t ble_cmd_gatt_read_multiple_characteristic_values( uint8 connection, uint8 characteristic_list_len, const uint8 *characteristic_list_data );
            void (*ble_rsp_gatt_read_multiple_characteristic_values)( const struct ble_msg_gatt_read_multiple_characteristic_values_rsp_t *msg );
        #endif
        #ifdef BGLIB_ENABLE_CMD_GATT_SEND_CHARACTERICTIC_CONFIRMATION
            uint8_t ble_cmd_gatt_send_characteristic_confirmation( uint8 connection );
            void (*ble_rsp_gatt_send_characteristic_confirmation)( const struct ble_msg_gatt_send_characteristic_confirmation_rsp_t *msg );
        #endif
        #ifdef BGLIB_ENABLE_CMD_GATT_SET_CHARACTERISTIC_NOTIFICATION
            uint8_t ble_cmd_gatt_set_characteristic_notification( uint8 connection, uint16 characteristic, uint8 flags );
            void (*ble_rsp_gatt_set_characteristic_notification)( const struct ble_msg_gatt_set_characteristic_notification_rsp_t *msg );
        #endif
        #ifdef BGLIB_ENABLE_CMD_GATT_SET_MAX_MTU
            uint8_t ble_cmd_gatt_set_max_mtu( uint16 max_mtu );
            void (*ble_rsp_gatt_set_max_mtu)( const struct ble_msg_gatt_set_max_mtu_rsp_t *msg );
        #endif
        #ifdef BGLIB_ENABLE_CMD_GATT_WRITE_CHARACTERISTIC_VALUE
            uint8_t ble_cmd_gatt_write_characteristic_value( uint8 connection, uint16 characteristic, uint8 value_len, const uint8 *value_data );
            void (*ble_rsp_gatt_write_characteristic_value)( const struct ble_msg_gatt_write_characteristic_value_rsp_t *msg );
        #endif
        #ifdef BGLIB_ENABLE_CMD_GATT_WRITE_CHARACTERISTIC_VALUE_WITHOUT_RESPONSE
            uint8_t ble_cmd_gatt_write_characteristic_value_without_response( uint8 connection, uint16 characteristic, uint8 value_len, const uint8 *value_data );
            void (*ble_rsp_gatt_write_characteristic_value_without_response)( const struct ble_msg_gatt_write_characteristic_value_without_response_rsp_t *msg );
        #endif
        #ifdef BGLIB_ENABLE_CMD_GATT_WRITE_DESCRIPTOR_VALUE
            uint8_t ble_cmd_gatt_write_descriptor_value( uint8 connection, uint16 characteristic, uint8 value_len, const uint8 *value_data );
            void (*ble_rsp_gatt_write_descriptor_value)( const struct ble_msg_gatt_write_descriptor_value_rsp_t *msg );
        #endif
        /*  */
        #ifdef BGLIB_ENABLE_EVT_GATT_CHARACTERISTIC
            void (*ble_evt_gatt_characteristic)(const struct ble_msg_gatt_characteristic_evt_t *msg);
        #endif
        #ifdef BGLIB_ENABLE_EVT_GATT_CHARACTERISTIC_VALUE
            void (*ble_evt_gatt_characteristic_value)(const struct ble_msg_gatt_characteristic_value_evt_t *msg);
        #endif
        #ifdef BGLIB_ENABLE_EVT_GATT_DESCRIPTOR
            void (*ble_evt_gatt_descriptor)(const struct ble_msg_gatt_descriptor_evt_t *msg);
        #endif
        #ifdef BGLIB_ENABLE_EVT_GATT_DESCRIPTOR_VALUE
            void (*ble_evt_gatt_descriptor_value)(const struct ble_msg_gatt_descriptor_value_evt_t *msg);
        #endif
        #ifdef BGLIB_ENABLE_EVT_GATT_MTU_EXCHANGED
            void (*ble_evt_gatt_mtu_exchanged)(const struct ble_msg_gatt_mtu_exchanged_evt_t *msg);
        #endif
        #ifdef BGLIB_ENABLE_EVT_GATT_PROCEDURE_COMPLETED
            void (*ble_evt_gatt_procedure_completed)(const struct ble_msg_gatt_procedure_completed_evt_t *msg);
        #endif
        #ifdef BGLIB_ENABLE_EVT_GATT_SERVICE
            void (*ble_evt_gatt_service)(const struct ble_msg_gatt_service_evt_t *msg);
        #endif
        /*  */


        /* [gatt_server] Generic Attribure Profile Server */
        #ifdef BGLIB_ENABLE_CMD_GATT_SERVER_FIND_ATTRIBUTE
            uint8_t ble_cmd_gatt_server_find_attribute( uint16 start, uint8 data_len, const uint8 *data_data );
            void (*ble_rsp_gatt_server_find_attribute)( const struct ble_msg_gatt_server_find_attribute_rsp_t *msg );
        #endif
        #ifdef BGLIB_ENABLE_CMD_GATT_SERVER_READ_ATTRIBUTE_TYPE
            uint8_t ble_cmd_gatt_server_read_attribute_type( uint16 attribute );
            void (*ble_rsp_gatt_server_read_attribute_type)( const struct ble_msg_gatt_server_read_attribute_type_rsp_t *msg );
        #endif
        #ifdef BGLIB_ENABLE_CMD_GATT_SERVER_READ_ATTRIBUTE_VALUE
            uint8_t ble_cmd_gatt_server_read_attribute_value( uint16 attribute, uint16 offset );
            void (*ble_rsp_gatt_server_read_attribute_value)( const struct ble_msg_gatt_server_read_attribute_balue_rsp_t *msg );
        #endif
        #ifdef BGLIB_ENABLE_CMD_GATT_SERVER_SEND_CHARACTERISTIC_NOTIFICATION
            uint8_t ble_cmd_gatt_server_send_characteristic_notification( uint8 connection, uint16 characteristic, uint8 value_len, const uint8 *value_data );
            void (*ble_rsp_gatt_server_send_caracteristic_notification)( const struct ble_msg_gatt_server_send_characteristic_notification_rsp_t *msg );
        #endif
        #ifdef BGLIB_ENABLE_CMD_GATT_SERVER_SEND_USER_READ_RESPONSE
            uint8_t ble_cmd_gatt_server_send_user_read_response( uint8 connection, uint16 characteristic, uint8 att_errorcode, uint8 value_len, const uint8 *value_data );
            void (*ble_rsp_gatt_server_send_user_read_response)( const struct ble_msg_gatt_server_send_user_read_response_rsp_t *msg );
        #endif
        #ifdef BGLIB_ENABLE_CMD_GATT_SERVER_SEND_USER_WRITE_RESPONSE
            uint8_t ble_cmd_gatt_server_send_user_write_response( uint8 connection, uint16 characteristic, uint8 att_errorcode );
            void (*ble_rsp_gatt_server_send_user_write_response)( const struct ble_msg_gatt_server_send_user_write_response_rsp_t *msg );
        #endif
        #ifdef BGLIB_ENABLE_CMD_GATT_SERVER_SENT_CAPABILITIES
            uint8_t ble_cmd_gatt_server_send_capabilities( uint32 caps, uint32 reserved );
            void (*ble_rsp_gatt_server_sent_capabilities)( const struct ble_msg_gatt_server_sent_capabilities_rsp_t *msg );
        #endif
        #ifdef BGLIB_ENABLE_CMD_GATT_SERVER_SET_DATABASE
            uint8_t ble_cmd_gatt_server_set_database( uint32 ptr );
            void (*ble_rsp_gatt_server_set_database)( const struct ble_msg_gatt_server_set_database_rsp_t *msg );
        #endif
        #ifdef BGLIB_ENABLE_CMD_GATT_SERVER_WRITE_ATTRIBUTE_VALUE
            uint8_t ble_cmd_gatt_server_write_attribute_value( uint16 attribute, uint16 offset, uint8 value_len, const uint8 *value_data );
            void (*ble_rsp_gatt_server_write_attribute_value)( const struct ble_msg_gatt_server_write_arttribute_value_rsp_t *msg );
        #endif
        /*  */
        #ifdef BGLIB_ENABLE_EVT_GATT_SERVER_ATTRIBUTE_VALUE
            void (*ble_evt_gatt_server_attribute_value)(const struct ble_msg_gatt_server_attribute_value_evt_t *msg);
        #endif
        #ifdef BGLIB_ENABLE_EVT_GATT_SERVER_CHARACTERISTIC_STATUS
            void (*ble_evt_gatt_server_characteristic_status)(const struct ble_msg_gatt_server_characteristic_status_evt_t *msg);
        #endif
        #ifdef BGLIB_ENABLE_EVT_GATT_SERVER_EXECUTE_WRITE_COMPLETED
            void (*ble_evt_gatt_server_execute_write_completed)(const struct ble_msg_gatt_server_execute_write_completed_evt_t *msg);
        #endif
        #ifdef BGLIB_ENABLE_EVT_GATT_SERVER_USER_READ_REQUEST
            void (*ble_evt_gatt_server_user_read_request)(const struct ble_msg_gatt_server_user_read_request_evt_t *msg);
        #endif
        #ifdef BGLIB_ENABLE_EVT_GATT_SERVER_USER_WRITE_REQUEST
            void (*ble_evt_gatt_server_user_write_request)(const struct ble_msg_gatt_server_user_write_request_evt_t *msg);
        #endif
        /*  */


        /* [hardware] Hardware */
        #ifdef BGLIB_ENABLE_CMD_HARDWARE_ENABLE_DCDC
            uint8_t ble_cmd_hardware_enable_dcdc( uint8 enable );
            void (*ble_rsp_hardware_enable_dcdc)( const struct ble_msg_hardware_enable_dcdc_rsp_t *msg );
        #endif
        #ifdef BGLIB_ENABLE_CMD_HARDWARE_GET_TIME
            uint8_t ble_cmd_haerware_get_time( void );
            void (*ble_rsp_hardware_get_time)( const struct ble_msg_hardware_get_time_rsp_t *msg );
        #endif
        #ifdef BGLIB_ENABLE_CMD_HARDWARE_SET_LAZY_SOFT_TIMER
            uint8_t ble_cmd_hardware_set_lazy_soft_timer( uint32 time, uint32 slack,uint8 handle, uint8 single_shot );
            void (*ble_rsp_hardware_set_lazy_soft_timer)( const struct ble_msg_hardware_set_lazy_soft_timer_rsp_t *msg );
        #endif
        #ifdef BGLIB_ENABLE_CMD_HARDWARE_SET_SOFT_TIMER
            uint8_t ble_cmd_hardware_set_soft_timer( uint32 time, uint8 handle, uint8 single_shot );
            void (*ble_rsp_hardware_set_soft_timer)( const struct ble_msg_hardware_set_soft_timer_rsp_t *msg );
        #endif
        /*  */
        #ifdef BGLIB_ENABLE_EVT_HARDWARE_SOFT_TIMER
            void (*ble_evt_hardware_soft_timer)(const struct ble_msg_hardware_soft_timer_evt_t *msg);
        #endif
        /*  */


        /* [le connection] Connection management */
        #ifdef BGLIB_ENABLE_CMD_LE_CONNECTION_CLOSE
            uint8_t ble_cmd_le_connection_close( uint8 connection );
            void (*ble_rsp_le_connection_close)( const struct ble_msg_le_connection_close_rsp_t *msg );
        #endif
        #ifdef BGLIB_ENABLE_CMD_LE_CONNECTION_DISABLE_SLAVE_LATENCY
            uint8_t ble_cmd_le_connection_disable_slave_latency( uint8 connection, uint8 disable );
            void (*ble_rsp_le_connection_disable_slave_latancy)( const struct ble_msg_le_connection_disable_slave_latancy_rsp_t *msg );
        #endif
        #ifdef BGLIB_ENABLE_CMD_LE_CONNECTION_GET_RSSI
            uint8_t ble_cmd_le_connection_get_rssi( uint8 connection );
            void (*ble_rsp_le_connection_get_rssi)( const struct ble_msg_le_connection_get_rssi_rsp_t *msg );
        #endif
        #ifdef BGLIB_ENABLE_CMD_LE_CONNECTION_SET_PARAMETERS
            uint8_t ble_cmd_le_connection_set_parameters( uint8 connection, uint16 min_interval, uint16 max_interval, uint16 latency, uint16 timeout );
            void (*ble_rsp_le_connection_set_parameters)( const struct ble_msg_le_connection_set_parameters_rsp_t *msg );
        #endif
        #ifdef BGLIB_ENABLE_CMD_LE_CONNECTION_SET_PHY
            uint8_t ble_cmd_le_connection_set_phy( uint8 connection, uint8 phy );
            void (*ble_rsp_le_connection_set_phy)( const struct ble_msg_le_connection_set_phy_rsp_t *msg );
        #endif
        /*  */
        #ifdef BGLIB_ENABLE_EVT_LE_CONNECTION_CLOSED
            void (*ble_evt_le_connection_closed)(const struct ble_msg_le_connection_closed_evt_t *msg);
        #endif
        #ifdef BGLIB_ENABLE_EVT_LE_CONNECTION_OPENED
            void (*ble_evt_le_connection_opend)(const struct ble_msg_le_connection_opend_evt_t *msg);
        #endif
        #ifdef BGLIB_ENABLE_EVT_LE_CONNECTION_PARAMETERS
            void (*ble_evt_le_connection_parameters)(const struct ble_msg_le_connection_parameters_evt_t *msg);
        #endif
        #ifdef BGLIB_ENABLE_EVT_LE_CONNECTION_PHY_STATUS
            void (*ble_evt_le_connection_phy_status)(const struct ble_msg_le_connection_phy_status_evt_t *msg);
        #endif
        #ifdef BGLIB_ENABLE_EVT_LE_CONNECTION_RSSI
            void (*ble_evt_le_connection_rssi)(const struct ble_msg_le_connection_rssi_evt_t *msg);
        #endif
        /*  */


        /* [le gap] Generic Access Profile */
        #ifdef BGLIB_ENABLE_CMD_LE_GAP_BT5_SET_ADV_DATA
            uint8_t ble_cmd_le_gap_bt5_set_adv_data( uint8 handle, uint8 scan_rsp, uint8 adv_data_len, const uint8 *adv_data_data );
            void (*ble_rsp_le_gap_bt5_set_adv_data)( const struct ble_msg_le_gap_bt5_set_adv_data_rsp_t *msg );
        #endif
        #ifdef BGLIB_ENABLE_CMD_LE_GAP_BT5_SET_ADV_PARAMETERS
            uint8_t ble_cmd_le_gap_bt5_set_adv_parameters( uint8 handle, uint16 interval_min, uint16 interval_max, uint8 channel_map, uint8 report_scan );
            void (*ble_rsp_le_gap_bt5_set_adv_parameters)( const struct ble_msg_le_gap_bt5_set_adv_parameters_rsp_t *msg );
        #endif
        #ifdef BGLIB_ENABLE_CMD_LE_GAP_BT5_SET_MODE
            uint8_t ble_cmd_le_gap_bt5_set_mode( uint8 handle, uint8 discover, uint8 connect, uint16 maxevents, uint8 address_type );
            void (*ble_rsp_le_gap_bt5_set_mode)( const struct ble_msg_le_gap_bt5_set_mode_rsp_t *msg );
        #endif
        #ifdef BGLIB_ENABLE_CMD_LE_GAP_CLEAR_ADVERTISE_CONFIGURATION
            uint8_t ble_cmd_le_gap_clear_advertise_configuration( uint8 handle, uint32 configurations );
            void (*ble_rsp_le_gap_clear_advertise_configuration)( const struct ble_msg_le_gap_clear_advertise_configuration_rsp_t *msg );
        #endif
        #ifdef BGLIB_ENABLE_CMD_LE_GAP_CONNECT
            uint8_t ble_cmd_le_gap_connect( bd_addr address, uint8 address_type, uint8 initiating_phy );
            void (*ble_rsp_le_gap_connect)( const struct ble_msg_le_gap_connect_rsp_t *msg );
        #endif
        #ifdef BGLIB_ENABLE_CMD_LE_GAP_DISCOVER
            uint8_t ble_cmd_le_gap_discover( uint8 mode );
            void (*ble_rsp_le_gap_discover)( const struct ble_msg_le_gap_discover_rsp_t *msg );
        #endif
        #ifdef BGLIB_ENABLE_CMD_LE_GAP_END_PROCEDURE
            uint8_t ble_cmd_le_gap_end_procedure( void );
            void (*ble_rsp_le_gap_end_procedure)(const struct ble_msg_le_gap_end_procedure_rsp_t *msg);
        #endif
        #ifdef BGLIB_ENABLE_CMD_LE_GAP_OPEN
            uint8_t ble_cmd_le_gap_open( bd_addr address, uint8 address_type);
            void (*ble_rsp_le_gap_open)(const struct ble_msg_le_gap_open_rsp_t *msg);
        #endif
        #ifdef BGLIB_ENABLE_CMD_LE_GAP_SET_ADV_DATA
            uint8_t ble_cmd_le_gap_set_adv_data( uint8 scan_rsp, uint8 adv_data_len, const uint8 *adv_data_data );
            void (*ble_rsp_le_gap_set_adv_data)(const struct ble_msg_le_gap_set_adv_data_rsp_t *msg);
        #endif
        #ifdef BGLIB_ENABLE_CMD_LE_GAP_SET_ADV_PARAMETERS
            uint8_t ble_cmd_le_gap_set_adv_parameters( uint16 interval_min, uint16 interval_max, uint8 channnel_map );
            void (*ble_rsp_le_gap_set_adv_parameters)(const struct ble_msg_le_gap_set_adv_parameters_rsp_t *msg);
        #endif
        #ifdef BGLIB_ENABLE_CMD_LE_GAP_SET_ADV_TIMEOUT
            uint8_t ble_cmd_le_gap_set_adv_timeout( uint8 maxevents );
            void (*ble_rsp_le_gap_set_adv_timeout)(const struct ble_msg_le_gap_set_adv_timeout_rsp_t *msg);
        #endif
        #ifdef BGLIB_ENABLE_CMD_LE_GAP_SET_ADVERTISE_CHANNEL_MAP
            uint8_t ble_cmd_le_gap_set_advertise_channel_map( uint8 handle, uint8 channel_map );
            void (*ble_rsp_le_gap_set_advertise_channel_map)(const struct ble_msg_le_gap_set_advertise_channel_map_rsp_t *msg);
        #endif
        #ifdef BGLIB_ENABLE_CMD_LE_GAP_SET_ADVERTISE_CONFIGURATION
            uint8_t ble_cmd_le_gap_set_advertise_configuration( uint8 handle, uint32 configuration );
            void (*ble_rsp_le_gap_set_advertise_configuration)(const struct ble_msg_le_gap_set_advertise_configuration_rsp_t *msg);
        #endif
        #ifdef BGLIB_ENABLE_CMD_LE_GAP_SET_ADVERTISE_PHY
            uint8_t ble_cmd_le_gap_set_advertise_phy( uint8 handle, uint8 primary_phy, uint8 secondary_phy );
            void (*ble_rsp_le_gap_set_advertise_phy)(const struct ble_msg_le_gap_set_advertise_phy_rsp_t *msg);
        #endif
        #ifdef BGLIB_ENABLE_CMD_LE_GAP_SET_ADVERTISE_REPORT_SCAN_REQUEST
            uint8_t ble_cmd_le_gap_set_advertise_report_scan_request( uint8 handle, uint8 report_scan_req );
            void (*ble_rsp_le_gap_set_advertise_report_scan_request)(const struct ble_msg_le_gap_set_advertise_report_scan_request_rsp_t *msg);
        #endif
        #ifdef BGLIB_ENABLE_CMD_LE_GAP_SET_ADVERTISE_TIMING
            uint8_t ble_cmd_le_gap_set_advertise_timing( uint8 handle, uint32 interval_min, uint32 interval_max, uint16 duration, uint8 maxevents );
            void (*ble_rsp_le_gap_set_advertise_timing)(const struct ble_msg_le_gap_set_advertise_timing_rsp_t *msg);
        #endif
        #ifdef BGLIB_ENABLE_CMD_LE_GAP_SET_CONN_PARAMEETERS
            uint8_t ble_cmd_le_gap_set_conn_parameters( uint16 interval_min, uint16 interval_max, uint16 latency, uint16 timeout );
            void (*ble_rsp_le_gap_set_conn_parameters)(const struct ble_msg_le_gap_set_conn_parameters_rsp_t *msg);
        #endif
        #ifdef BGLIB_ENABLE_CMD_LE_GAP_SET_DATA_CHANNEL_CLASSIFICATION
            uint8_t ble_cmd_le_gap_set_data_channel_classification( uint8 channel_map_len, const uint8 *channel_map_data );
            void (*ble_rsp_le_gap_set_data_channel_classification)(const struct ble_msg_le_gap_set_data_channel_classification_rsp_t *msg);
        #endif
        #ifdef BGLIB_ENABLE_CMD_LE_GAP_SET_DISCOVERY_TIMING
            uint8_t ble_cmd_le_gap_set_discovery_timing( uint8 phys, uint16 scan_interval, uint16 scan_window );
            void (*ble_rsp_le_gap_set_discovery_timing)(const struct ble_msg_le_gap_set_discovery_timing_rsp_t *msg);
        #endif
        #ifdef BGLIB_ENABLE_CMD_LE_GAP_SET_DISCOVERY_TYPE
            uint8_t ble_cmd_le_gap_set_discovery_type( uint8 phys, uint8 scan_type );
            void (*ble_rsp_le_gap_set_discovery_type)(const struct ble_msg_le_gap_set_discovery_type_rsp_t *msg);
        #endif
        #ifdef BGLIB_ENABLE_CMD_LE_GAP_SET_MODE
            uint8_t ble_cmd_le_gap_set_mode( uint8 discover, uint8 connect );
            void (*ble_rsp_le_gap_set_mode) (const struct ble_msg_le_gap_set_mode_rsp_t *msg );
        #endif
        #ifdef BGLIB_ENABLE_CMD_LE_GAP_SET_PRIVACY_MODE
            uint8_t ble_cmd_le_gap_set_privacy_mode( uint8 privacy, uint8 interval );
            void (*ble_rsp_le_gap_set_privacy_mode) (const struct ble_msg_le_gap_set_privacy_mode_rsp_t *msg );
        #endif
        #ifdef BGLIB_ENABLE_CMD_LE_GAP_SET_SCAN_PARAMETERS
            uint8_t ble_cmd_le_gap_set_scan_parameters( uint16 scan_interval, uint16 scan_window, uint8 active );
            void (*ble_rsp_le_gap_set_scan_parameters) (const struct ble_msg_le_gap_set_scan_parameters_rsp_t *msg );
        #endif
        #ifdef BGLIB_ENABLE_CMD_LE_GAP_START_ADVERTISING
            uint8_t ble_cmd_le_gap_start_advertising( uint8 handle, uint8 discover, uint8 connect );
            void (*ble_rsp_le_gap_start_advertising) (const struct ble_msg_le_gap_start_advertising_rsp_t *msg );
        #endif
        #ifdef BGLIB_ENABLE_CMD_LE_GAP_START_DISCOVERY
            uint8_t ble_cmd_le_gap_start_discovery( uint8 scanning_phy, uint8 mode );
            void (*ble_rsp_le_gap_start_discovery) (const struct ble_msg_le_gap_start_discovery_rsp_t *msg );
        #endif
        #ifdef BGLIB_ENABLE_CMD_LE_GAP_STOP_ADVERTISING
            uint8_t ble_cmd_le_gap_stop_advertising( uint8 handle );
            void (*ble_rsp_le_gap_stop_advertising) (const struct ble_msg_le_gap_stop_advertising_rsp_t *msg );
        #endif
        /*  */
        #ifdef BGLIB_ENABLE_EVT_LE_GAP_ADV_TIMEOUT
            void (*ble_evt_le_gap_adv_timeout)(const struct ble_msg_le_gap_adv_timeout_evt_t *msg);
        #endif
        #ifdef BGLIB_ENABLE_EVT_LE_GAP_SCAN_REQUEST
            void (*ble_evt_le_gap_scan_request)(const struct ble_msg_le_gap_scan_request_evt_t *msg);
        #endif
        #ifdef BGLIB_ENABLE_EVT_LE_GAP_SCAN_RESPONSE
            void (*ble_evt_le_gap_scan_response)(const struct ble_msg_le_gap_scan_response_evt_t *msg);
        #endif
        /*  */


        /* [SM] Security Manager */
        #ifdef BGLIB_ENABLE_CMD_SM_BONDING_CONFIRM
            uint8_t ble_cmd_sm_bonding_confirm( uint8 connection, uint8 confirm );
            void (*ble_rsp_sm_bonding_confirm) (const struct ble_msg_sm_bonding_confirm_rsp_t *msg );
        #endif
        #ifdef BGLIB_ENABLE_CMD_SM_CONFIGURE
            uint8_t ble_cmd_sm_configure( uint8 flags, uint8 io_capabilities );
            void (*ble_rsp_sm_configure) (const struct ble_msg_sm_configure_rsp_t *msg );
        #endif
        #ifdef BGLIB_ENABLE_CMD_SM_DELETE_BONDING
            uint8_t ble_cmd_sm_delete_bonding( uint8 bonding );
            void (*ble_rsp_sm_delete_bonding) (const struct ble_msg_sm_delete_bonding_rsp_t *msg );
        #endif
        #ifdef BGLIB_ENABLE_CMD_SM_DELETE_BONDINGS
            uint8_t ble_cmd_sm_delete_bondings( void );
            void (*ble_rsp_sm_delete_bondings) (const struct ble_msg_sm_delete_bondings_rsp_t *msg );
        #endif
        #ifdef BGLIB_ENABLE_CMD_SM_ENTER_PASSKEY
            uint8_t ble_cmd_sm_enter_passkey( uint8 connection, int32 passkey );
            void (*ble_rsp_sm_enter_passkey) (const struct ble_msg_sm_enter_passkey_rsp_t *msg );
        #endif
        #ifdef BGLIB_ENABLE_CMD_SM_INCREASE_SECURITY
            uint8_t ble_cmd_sm_increase_security( uint8 connection );
            void (*ble_rsp_sm_increase_security) (const struct ble_msg_sm_increase_security_rsp_t *msg );
        #endif
        #ifdef BGLIB_ENABLE_CMD_SM_LIST_ALL_BONDINGS
            uint8_t ble_cmd_sm_list_all_bondings( void );
            void (*ble_rsp_sm_list_all_bondings) (const struct ble_msg_sm_list_all_bondings_rsp_t *msg );
        #endif
        #ifdef BGLIB_ENABLE_CMD_SM_PASSKEY_CONFIRM
            uint8_t ble_cmd_sm_passkey_confirm( uint8 connection, uint8 confirm );
            void (*ble_rsp_sm_passkey_confirm) (const struct ble_msg_sm_passkey_confirm_rsp_t *msg );
        #endif
        #ifdef BGLIB_ENABLE_CMD_SM_SET_BONDABLE_MODE
            uint8_t ble_cmd_sm_set_bondable_mode( uint8 bondable );
            void (*ble_rsp_sm_set_bondable_mode) (const struct ble_msg_sm_set_bondable_mode_rsp_t *msg );
        #endif
        #ifdef BGLIB_ENABLE_CMD_SM_SET_DEBUG_MODE
            uint8_t ble_cmd_sm_set_debug_mode( void );
            void (*ble_rsp_sm_set_debug_mode) (const struct ble_msg_sm_set_debug_mode_rsp_t *msg );
        #endif
        #ifdef BGLIB_ENABLE_CMD_SM_SET_OOB_DATA
            uint8_t ble_cmd_sm_set_oob_data( uint8 oob_data_len, const uint8 *oob_data_data );
            void (*ble_rsp_sm_set_oob_data) (const struct ble_msg_sm_set_oob_data_rsp_t *msg );
        #endif
        #ifdef BGLIB_ENABLE_CMD_SM_SET_PASSKEY
            uint8_t ble_cmd_sm_set_passkey( int32 passkey );
            void (*ble_rsp_sm_set_passkey) (const struct ble_msg_sm_set_passkey_rsp_t *msg );
        #endif
        #ifdef BGLIB_ENABLE_CMD_SM_SET_SC_REMOTE_OOB_DATA
            uint8_t ble_cmd_sm_set_sc_remote_oob_data( uint8 oob_data_len, const uint8 *oob_data_data );
            void (*ble_rsp_sm_set_sc_remote_oob_data) (const struct ble_msg_sm_set_sc_remote_oob_data_rsp_t *msg );
        #endif
        #ifdef BGLIB_ENABLE_CMD_SM_STORE_BONDING_CONFIGURATION
            uint8_t ble_cmd_sm_store_bonding_configuration( uint8 max_bonding_count, uint8 policy_flags );
            void (*ble_rsp_sm_store_bonding_configuration) (const struct ble_msg_sm_store_bonding_configuration_rsp_t *msg );
        #endif
        #ifdef BGLIB_ENABLE_CMD_SM_USE_SC_OOB
            uint8_t ble_cmd_sm_use_sc_oob( uint8 enable );
            void (*ble_rsp_sm_use_sc_oob) (const struct ble_msg_sm_use_sc_oob_rsp_t *msg );
        #endif
        /*  */
        #ifdef BGLIB_ENABLE_EVT_SM_BONDED
            void (*ble_evt_sm_bonded)(const struct ble_msg_sm_bonded_evt_t *msg);
        #endif
        #ifdef BGLIB_ENABLE_EVT_SM_BONDING_FAILED
            void (*ble_evt_sm_bonding_failed)(const struct ble_msg_sm_bonding_failed_evt_t *msg);
        #endif
        #ifdef BGLIB_ENABLE_EVT_SM_CONFIRM_BONDING
            void (*ble_evt_sm_confirm_bonding)(const struct ble_msg_sm_confirm_bonding_evt_t *msg);
        #endif
        #ifdef BGLIB_ENABLE_EVT_SM_CONFIRM_PASSKEY
            void (*ble_evt_sm_confirm_passkey)(const struct ble_msg_sm_confirm_passkey_evt_t *msg);
        #endif
        #ifdef BGLIB_ENABLE_EVT_SM_LIST_ALL_BONDINGS_COMPLETE
            void (*ble_evt_sm_list_all_bondings_complete)(const struct ble_msg_sm_list_all_bondings_complete_evt_t *msg);
        #endif
        #ifdef BGLIB_ENABLE_EVT_SM_LIST_BONDING_ENTRY
            void (*ble_evt_sm_list_bonding_entry)(const struct ble_msg_sm_list_bonding_entry_evt_t *msg);
        #endif
        #ifdef BGLIB_ENABLE_EVT_SM_PASSKEY_DISPLAY
            void (*ble_evt_sm_passkey_display)(const struct ble_msg_sm_passkey_display_evt_t *msg);
        #endif
        #ifdef BGLIB_ENABLE_EVT_SM_PASSKEY_REQUEST
            void (*ble_evt_sm_passkey_request)(const struct ble_msg_sm_passkey_request_evt_t *msg);
        #endif
        /*  */


        /* [system] System commands */
        #ifdef BGLIB_ENABLE_CMD_SYSTEM_GET_BT_ADDRESS
            uint8_t ble_cmd_system_get_bt_address( void );
            void (*ble_rsp_system_get_bt_address)( const struct ble_msg_system_get_bt_address_rsp_t *msg );
        #endif
        #ifdef BGLIB_ENABLE_CMD_SYSTEM_GET_COUNTERS
            uint8_t ble_cmd_system_get_counters( uint8 reset );
            void (*ble_rsp_system_get_counters)(const struct ble_msg_system_get_counters_rsp_t *msg);
        #endif
        #ifdef BGLIB_ENABLE_CMD_SYSTEM_GET_RANDOM_DATA
            uint8_t ble_cmd_system_get_random_data( uint8 length );
            void (*ble_rsp_system_get_random_data) (const struct ble_msg_system_get_random_data_rsp_t *msg);
        #endif
        #ifdef BGLIB_ENABLE_CMD_SYSTEM_HALT
            uint8_t ble_cmd_system_halt( uint8 halt );
            void (*ble_rsp_system_halt)(const struct ble_msg_system_halt_rsp_t *msg );
        #endif
        #ifdef BGLIB_ENABLE_CMD_SYSTEM_HELLO
            uint8_t ble_cmd_system_hello( void );
            void (*ble_rsp_system_hello)( const struct ble_msg_system_hello_rsp_t *msg );
        #endif
        #ifdef BGLIB_ENABLE_CMD_SYSTEM_RESET
            uint8_t ble_cmd_system_reset( uint8 boot_in_dfu );
            void (*ble_rsp_system_reset)( const struct ble_msg_system_reset_rsp_t *msg );
        #endif
        #ifdef BGLIB_ENABLE_CMD_SYSTEM_SET_BT_ADDRESS
            uint8_t ble_cmd_system_set_bt_address( bd_addr address );
            void (*ble_rsp_system_set_bt_address)( const struct ble_msg_system_set_bt_address_rsp_t *msg );
        #endif
        #ifdef BGLIB_ENABLE_CMD_SYSTEM_SET_DEVICE_NAME
            uint8_t ble_cmd_system_set_device_name( uint8 type, uint8 neme_len, const uint8 *name_data );
            void (*ble_rsp_system_set_device_name)( const struct ble_msg_system_set_device_name_rsp_t *msg );
        #endif
        #ifdef BGLIB_ENABLE_CMD_SYSTEM_SET_TX_POWER
            uint8_t ble_cmd_system_set_tx_power( int16 power );
            void (*ble_rsp_system_set_tx_power)( const struct ble_msg_system_set_tx_power_rsp_t *msg );
        #endif
        /*  */
        #ifdef BGLIB_ENABLE_EVT_SYSTEM_AWAKE
//            void (*ble_evt_system_awake)(const struct ble_msg_system_awake_evt_t *msg);
            void (*ble_evt_system_awake)( void );
        #endif
        #ifdef BGLIB_ENABLE_EVT_SYSTEM_BOOT
            void (*ble_evt_system_boot)(const struct ble_msg_system_boot_evt_t *msg);
        #endif
        #ifdef BGLIB_ENABLE_EVT_SYSTEM_ERROR
            void (*ble_evt_system_error)(const struct ble_msg_system_error_evt_t *msg);
        #endif
        #ifdef BGLIB_ENABLE_EVT_SYSTEM_EXTERNAL_SIGNAL
            void (*ble_evt_system_external_signal)(const struct ble_msg_system_external_signal_evt_t *msg);
        #endif
        #ifdef BGLIB_ENABLE_EVT_SYSTEM_HARDWARE_ERROR
            void (*ble_evt_system_hardware_error)(const struct ble_msg_system_hardware_error_evt_t *msg);
        #endif
        /*  */


        /* [test] testing */
        #ifdef BGLIB_ENABLE_CMD_TEST_DTM_END
            uint8_t ble_cmd_test_dtm_end( void );
            void (*ble_rsp_test_dtm_end)( const struct ble_msg_test_dtm_end_rsp_t *msg );
        #endif
        #ifdef BGLIB_ENABLE_CMD_TEST_DTM_RX
            uint8_t ble_cmd_test_dtm_rx( uint8 channel, uint8 phy );
            void (*ble_rsp_test_dtm_rx)( const struct ble_msg_test_dtm_rx_rsp_t *msg );
        #endif
        #ifdef BGLIB_ENABLE_CMD_TEST_DTM_TX
            uint8_t ble_cmd_test_dtm_tx( uint8 packet_type, uint8 length, uint8 channel, uint8 phy );
            void (*ble_rsp_test_dtm_tx)( const struct ble_msg_test_dtm_tx_rsp_t *msg );
        #endif
        /*  */
        #ifdef BGLIB_ENABLE_EVT_TEST_DTM_COMPLETED
            void (*ble_evt_test_dtm_completed)(const struct ble_msg_test_dtm_completed_evt_t *msg);
        #endif
        /*  */


        /* [user] User message */
        #ifdef BGLIB_ENABLE_CMD_USER_MESSAGE_TO_TARGET
            uint8_t ble_cmd_user_message_to_target( uint8 data_len, const uint8 *data_data );
            void (*ble_rsp_user_message_to_target)( const struct ble_msg_user_message_to_target_rsp_t *msg );
        #endif
        /*  */
        /* [user] User message */
        #ifdef BGLIB_ENABLE_EVT_USER_MASSAGE_TO_HOST
            void (*ble_evt_user_message_to_host)(const struct ble_msg_user_message_to_host_evt_t *msg);
        #endif
        /*  */



    private:
        // incoming packet buffer vars
        uint8_t *bgapiRXBuffer;
        uint8_t bgapiRXBufferSize;
        uint8_t bgapiRXBufferPos;
        uint16_t bgapiRXDataLen;
        /*  */
        // outgoing package buffer vars
        uint8_t *bgapiTXBuffer;
        uint8_t bgapiTXBufferSize;
        uint8_t bgapiTXBufferPos;
        /*  */

        // BGAPI packet structure representation
        const struct ble_msg *packetMessage;
        struct ble_header packetHeader;
        uint8_t *packetData;
        /*  */
        HardwareSerial *uModule;        // required UART object with module connection
        HardwareSerial *uOutput;        // optional UART object for host/debug connection
        /*  */
        bool busy;
        uint8_t packetMode;
        uint8_t lastCommand[2];
        uint8_t lastResponse[2];
        uint8_t lastEvent[2];
        uint32_t timeoutStart;
        bool lastError;
        bool lastTimeout;
};


#endif  /* __TBGLIB_H__ */
