#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"

#include "OnBoard.h"
#include "hal_adc.h"
#include "hal_led.h"
#include "hal_key.h"

#include "gatt.h"

#include "hci.h"

#include "gapgattserver.h"
#include "gattservapp.h"

#include "gatt_profile_uuid.h"

#include "devinfoservice.h"

#include "peripheral.h"

#include "gapbondmgr.h"

#include "app_task.h"

// What is the advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          160

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely

#if defined ( CC2540_MINIDK )
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_LIMITED
#else
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL
#endif  // defined ( CC2540_MINIDK )

// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     80

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     800

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          1000

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         TRUE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         6

static uint8 testBLETask_TaskID;
static gaprole_States_t gapProfileState = GAPROLE_INIT;

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8 scanRspData[] =
{
      // complete name
      0x6,   // length of this data
      GAP_ADTYPE_LOCAL_NAME_COMPLETE,
      'H',
      'E',
      'L',
      'L',
      'O',
      // connection interval range
      0x05,   // length of this data
      GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
      LO_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),   // 100ms
      HI_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),
      LO_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),   // 1s
      HI_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),
      
      // Tx power level
      0x02,   // length of this data
      GAP_ADTYPE_POWER_LEVEL,
      0       // 0dBm
};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8 advertData[] =
{
      // Flags; this sets the device to use limited discoverable
      // mode (advertises for 30 seconds at a time) instead of general
      // discoverable mode (advertises indefinitely)
      0x02,   // length of this data
      GAP_ADTYPE_FLAGS,
      DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,
      
      // service UUID, to notify central devices what services are included
      // in this peripheral
      0x03,   // length of this data
      GAP_ADTYPE_16BIT_MORE,      // some of the UUID's, but not all
      LO_UINT16( DEVINFO_SERV_UUID ),
      HI_UINT16( DEVINFO_SERV_UUID ),
};

// GAP GATT Attributes
static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "Test BLE Peripheral";

static void TestBLE_GapRoleStateNotificationCB( gaprole_States_t newState);
static void TestBLE_HandleKey( uint8 shift, uint8 keys );
static void TestBLE_ProcessOSALMsg( osal_event_hdr_t* msg );

static gapRolesCBs_t TestBLE_GapRoleCallbacks = 
{
      TestBLE_GapRoleStateNotificationCB,
      NULL,
};

static gapBondCBs_t TestBLE_BondCallbacks = 
{
      NULL,
      NULL,
};

// GATT Profiles callbacks

// Public Function
void TestBLE_TaskInit( uint8 task_id )
{
      testBLETask_TaskID = task_id;
      
      // Setup the GAP
      VOID GAP_SetParamValue( TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL );
      
      // Setup the GAP Peripheral Role Profile
      {
	    // For the CC2540DK-MINI keyfob, device doesn't start advertising until button is pressed
	    uint8 initial_advertising_enable = FALSE;
	    
	    // By setting this to zero, the device will go into the waiting state after
	    // being discoverable for 30.72 second, and will not being advertising again
	    // until the enabler is set back to TRUE
	    uint16 gapRole_AdvertOffTime = 0;
	    
	    uint8 enable_update_request = DEFAULT_ENABLE_UPDATE_REQUEST;
	    uint16 desired_min_interval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
	    uint16 desired_max_interval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
	    uint16 desired_slave_latency = DEFAULT_DESIRED_SLAVE_LATENCY;
	    uint16 desired_conn_timeout = DEFAULT_DESIRED_CONN_TIMEOUT;
	    
	    // Set the GAP Role Parameters
	    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
	    GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime );
	    
	    GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanRspData ), scanRspData );
	    GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );
	    
	    GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &enable_update_request );
	    GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval );
	    GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_max_interval );
	    GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency );
	    GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout );
      }
      
      // Set the GAP Characteristics
      GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName );
      
      // Set advertising interval
      {
	    uint16 advInt = DEFAULT_ADVERTISING_INTERVAL;
	    
	    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, advInt );
	    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, advInt );
	    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt );
	    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt );
      }
      
      // Setup the GAP Bond Manager
      {
	    uint32 passkey = 0; // passkey "000000"
	    uint8 pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
	    uint8 mitm = TRUE;
	    uint8 ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
	    uint8 bonding = TRUE;
	    GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof ( uint32 ), &passkey );
	    GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof ( uint8 ), &pairMode );
	    GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof ( uint8 ), &mitm );
	    GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof ( uint8 ), &ioCap );
	    GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof ( uint8 ), &bonding );
      }
      
      // Initialize GATT attributes
      GGS_AddService( GATT_ALL_SERVICES );            // GAP
      GATTServApp_AddService( GATT_ALL_SERVICES );    // GATT attributes
      DevInfo_AddService();                           // Device Information Service
      
      // Register for all key events - This app will handle all key events
      RegisterForKeys( testBLETask_TaskID );
      
      VOID osal_set_event( testBLETask_TaskID, TASK_START );
}

uint16 TestBLE_OSALEvents( uint8 task_id, uint16 events )
{
      VOID task_id;
      
      if ( events & SYS_EVENT_MSG )
      {
	    uint8* msg;
	    
	    if ( (msg = osal_msg_receive( testBLETask_TaskID)) != NULL )
	    {
		  TestBLE_ProcessOSALMsg( (osal_event_hdr_t*) msg );
		  
		  VOID osal_msg_deallocate( msg );
	    }
	    
	    return (events ^ SYS_EVENT_MSG);
      }
      
      if ( events & TASK_START )
      {
	    // Start the Device
	    VOID GAPRole_StartDevice( &TestBLE_GapRoleCallbacks );
	    
	    // Start Bond Manager
	    VOID GAPBondMgr_Register( &TestBLE_BondCallbacks );
	    return (events ^ TASK_START);
      }
      // Discard unknown events
      return 0;
}

static void TestBLE_ProcessOSALMsg( osal_event_hdr_t* msg )
{
      switch(msg->event)
      {
	 case KEY_CHANGE:
	    TestBLE_HandleKey( ((keyChange_t *)msg)->state, ((keyChange_t *)msg)->keys );
	    break;
	 default:
	    // do nothing
	    break;
      }
}

static void TestBLE_HandleKey( uint8 shift, uint8 keys )
{
      VOID shift;
      if ( keys & HAL_KEY_SW_2 )
      {
	    // if device is not in a connection, pressing the right key should toggle
	    // advertising on and off
	    // Note:  If PLUS_BROADCASTER is define this condition is ignored and
	    //        Device may advertise during connections as well. 
#ifndef PLUS_BROADCASTER  
	    if( gapProfileState != GAPROLE_CONNECTED )
	    {
#endif // PLUS_BROADCASTER
		  uint8 current_adv_enabled_status;
		  uint8 new_adv_enabled_status;
		  
		  //Find the current GAP advertisement status
		  GAPRole_GetParameter( GAPROLE_ADVERT_ENABLED, &current_adv_enabled_status );
		  
		  if( current_adv_enabled_status == FALSE )
		  {
			new_adv_enabled_status = TRUE;
		  }
		  else
		  {
			new_adv_enabled_status = FALSE;
		  }
		  
		  //change the GAP advertisement status to opposite of current status
		  GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &new_adv_enabled_status );
#ifndef PLUS_BROADCASTER
	    }
#endif // PLUS_BROADCASTER
      }
}

static void TestBLE_GapRoleStateNotificationCB( gaprole_States_t newState)
{
      switch ( newState )
      {
	 case GAPROLE_STARTED:
	    {
		  uint8 ownAddress[B_ADDR_LEN];
		  uint8 systemId[DEVINFO_SYSTEM_ID_LEN];
		  
		  GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);
		  
		  // use 6 bytes of device address for 8 bytes of system ID value
		  systemId[0] = ownAddress[0];
		  systemId[1] = ownAddress[1];
		  systemId[2] = ownAddress[2];
		  
		  // set middle bytes to zero
		  systemId[4] = 0x00;
		  systemId[3] = 0x00;
		  
		  // shift three bytes up
		  systemId[7] = ownAddress[5];
		  systemId[6] = ownAddress[4];
		  systemId[5] = ownAddress[3];
		  
		  DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);
	    }
	    break;
	    
	 case GAPROLE_ADVERTISING:
	    {
	    }
	    break;
	    
	 case GAPROLE_CONNECTED:
	    {
	    }
	    break;
	    
	 case GAPROLE_CONNECTED_ADV:
	    {
	    }
	    break;      
	 case GAPROLE_WAITING:
	    {
	    }
	    break;
	    
	 case GAPROLE_WAITING_AFTER_TIMEOUT:
	    {
	    }
	    break;
	    
	 case GAPROLE_ERROR:
	    {
	    }
	    break;
	    
	 default:
	    {
	    }
	    break;
	    
      }
      
      gapProfileState = newState;
}


