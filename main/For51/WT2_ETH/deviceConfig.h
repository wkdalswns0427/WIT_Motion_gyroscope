#define device_mac "D86595654321"
#define SENSOR_ACC1_X 600
#define SENSOR_ACC1_Y 601
#define SENSOR_ACC1_Z 602
#define SENSOR_ANG1_X 603
#define SENSOR_ANG1_Y 604
#define SENSOR_ANG1_Z 605
#define SENSOR_ANGVEL1_X 606
#define SENSOR_ANGVEL1_Y 607
#define SENSOR_ANGVEL1_Z 608
#define ERRCNT_FAIL_THRESHOLD 10
#define ERRCNT_SUCCESS_THRESHOLD 3

#define ETH_POWER_PIN   16
#define ETH_MDC_PIN     23
#define ETH_MDIO_PIN    18
#define ETH_ADDR        1
#define ETH_CLK_MODE    ETH_CLOCK_GPIO0_IN
#define ETH_TYPE        ETH_PHY_LAN8720
static bool eth_connected = false;
