/* i2c - Example

   For other examples please check:
   https://github.com/espressif/esp-idf/tree/master/examples

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include <time.h>
//#include "driver/i2c.h"
#include "tcp_server.h"
#include "i2cdev.h"
#include "pca9685.h"

#include <sys/fcntl.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_err.h"
#include "esp_event_loop.h"
#include "freertos/event_groups.h"
#include "esp_event.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_eth.h"

#include "rom/ets_sys.h"
#include "rom/gpio.h"

#include "soc/dport_reg.h"
#include "soc/io_mux_reg.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/gpio_reg.h"
#include "soc/gpio_sig_map.h"

#include "tcpip_adapter.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

#include "eth_phy/phy_lan8720.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"

#define DEFAULT_ETHERNET_PHY_CONFIG phy_lan8720_default_ethernet_config
// #define PIN_PHY_POWER CONFIG_PHY_POWER_PIN				//not needed for our board configuration

#define PIN_SMI_MDC   GPIO_NUM_23
#define PIN_SMI_MDIO  GPIO_NUM_18

#define	TCP_SERVER_PORT			3000

static EventGroupHandle_t wifi_event_group;
static EventGroupHandle_t tcpserver_event_group;

/* The event group allows multiple bits for each event,
   but we only care about one event - are we connected
   to the AP with an IP? */
const static int CONNECTED_BIT = BIT0;

const static char *TAG = "Godor_project";

/**
 * Informations
 *
 * Godor: robot with 4 legs, 3 servo by leg
 *
 * Documentations about the differents components are in 'docs' directory
 *
 * Use a PCA9685 to control the servos
 *
 *
 * Connections
 *
 * PCA9685 - WROOM32
 *    SDA      25 
 *    SCL      26
 *    GND      GND
 *    VCC      3.3
 *
 *
 * This example will show you how to use I2C module by running two tasks on i2c bus:
 *
 * - read external i2c sensor, here we use a BH1750 light sensor(GY-30 module) for instance.
 * - Use one I2C port(master mode) to read or write the other I2C port(slave mode) on one ESP32 chip.
 *
 * Pin assignment:
 *
 * - slave :
 *    GPIO25 is assigned as the data signal of i2c slave port
 *    GPIO26 is assigned as the clock signal of i2c slave port
 * - master:
 *    GPIO18 is assigned as the data signal of i2c master port
 *    GPIO19 is assigned as the clock signal of i2c master port
 *
 * Connection:
 *
 * - connect GPIO18 with GPIO25
 * - connect GPIO19 with GPIO26
 * - connect sda/scl of sensor with GPIO18/GPIO19
 * - no need to add external pull-up resistors, driver will enable internal pull-up resistors.
 *
 * Test items:
 *
 * - read the sensor data, if connected.
 * - i2c master(ESP32) will write data to i2c slave(ESP32).
 * - i2c master(ESP32) will read data from i2c slave(ESP32).
 */

#define DATA_LENGTH                        512              /*!<Data buffer length for test buffer*/
#define RW_TEST_LENGTH                     129              /*!<Data length for r/w test, any value from 0-DATA_LENGTH*/
#define DELAY_TIME_BETWEEN_ITEMS_MS        500             /*!< delay time between different test items */

//#define I2C_EXAMPLE_MASTER_SCL_IO          19               /*!< gpio number for I2C master clock */
//#define I2C_EXAMPLE_MASTER_SDA_IO          18               /*!< gpio number for I2C master data  */
#define I2C_EXAMPLE_MASTER_SCL_IO          26               /*!< gpio number for I2C master clock */
#define I2C_EXAMPLE_MASTER_SDA_IO          25               /*!< gpio number for I2C master data  */
#define I2C_EXAMPLE_MASTER_NUM             I2C_NUM_1        /*!< I2C port number for master dev */
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE  0                /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE  0                /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_FREQ_HZ         100000           /*!< I2C master clock frequency */

#define BH1750_SENSOR_ADDR                 0x23             /*!< slave address for BH1750 sensor */
#define BH1750_CMD_START                   0x23             /*!< Command to set measure mode */
#define ESP_SLAVE_ADDR                     0x28             /*!< ESP32 slave address, you can set any 7bit value */
#define WRITE_BIT                          I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT                           I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN                       0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS                      0x0              /*!< I2C master will not check ack from slave */
#define ACK_VAL                            0x0              /*!< I2C ack value */
#define NACK_VAL                           0x1              /*!< I2C nack value */

#define GY521_ADDR                 0x68             /*!< slave address for GY521 sensor */
//#define GY521_ADDR                 0xE8             /*!< slave address for GY521 sensor */
//SemaphoreHandle_t print_mux = NULL;

#define PCA9685_SCL	26		/*!< gpio number for I2C master clock */
#define PCA9685_SDA	25		/*!< gpio number for I2C master data  */
#define PCA9685_ADDR	0x40		/*!< slave address for PCA9685 sensor */
#define PCA9685_PORT	I2C_NUM_1	/*!< I2C port number for master dev */

int		First = 1;
i2c_dev_t	Pca;		// PCA9685 that control the servos

#define CHECK_LOOP(X, msg, ...) do { esp_err_t __; while((__ = X) != ESP_OK) { printf(msg "\n", ## __VA_ARGS__); vTaskDelay(250 / portTICK_PERIOD_MS); }} while (0)
#define	SERVOMIN	400
#define	SERVOMAX	1950
#define	SERVODIFF	(SERVOMAX-SERVOMIN)


// Ids servos
#define	SERVOFRB	0	// id servo front right bottom
#define	SERVOFRM	1	// id servo front right middle
#define	SERVOFRH	2	// id servo front right high
#define	SERVORRB	3	// id servo rear right bottom
#define	SERVORRM	4	// id servo rear right middle
#define	SERVORRH	5	// id servo rear right high
#define	SERVOFLB	6	// id servo front left bottom
#define	SERVOFLM	7	// id servo front left middle
#define	SERVOFLH	8	// id servo front left high
#define	SERVORLB	9	// id servo rear left bottom
#define	SERVORLM	10	// id servo rear left middle
#define	SERVORLH	11	// id servo rear left high

// Position 0 godor must be on the ground
// In this position, if the servo must be set at the minimum value the
// following define must be set to 1
// If the servo must be set at the maximum value, rotation must be inverted
// and the following parameter must be set to -1
#define	SERVORIGHTB	-1	// bottom servo, rotation inverted=-1
#define	SERVORIGHTM	1	// middle servo, rotation inverted=-1
#define	SERVORIGHTH	-1	// high servo, rotation inverted=-1
#define	SERVOLEFTB	1	// bottom servo, rotation inverted=-1
#define	SERVOLEFTM	-1	// middle servo, rotation inverted=-1
#define	SERVOLEFTH	1	// high servo, rotation inverted=-1

uint16_t	LastPos[12];
uint16_t	Calib[12*2];	// Calibration for 12 servos, min, max

/**
 * @brief test code to write esp-i2c-slave
 *
 * 1. set mode
 * _________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write 1 byte + ack  | stop |
 * --------|---------------------------|---------------------|------|
 * 2. wait more than 24 ms
 * 3. read data
 * ______________________________________________________________________________________
 * | start | slave_addr + rd_bit + ack | read 1 byte + ack  | read 1 byte + nack | stop |
 * --------|---------------------------|--------------------|--------------------|------|
 */
static esp_err_t i2c_example_master_sensor_test(i2c_port_t i2c_num, int16_t *accx, int16_t *accy, int16_t *accz, int16_t *temp, int16_t *gyrox, int16_t*gyroy, int16_t*gyroz)
{
	int ret;
	uint8_t buf[7*2];

	if (First)
	{
		printf("First=1: init ");
		i2c_cmd_handle_t cmd = i2c_cmd_link_create();
		i2c_master_start(cmd);
		//    i2c_master_write_byte(cmd, BH1750_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
		//    i2c_master_write_byte(cmd, BH1750_CMD_START, ACK_CHECK_EN);
		i2c_master_write_byte(cmd, (GY521_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
		i2c_master_write_byte(cmd, 0x6B, ACK_CHECK_EN);
		i2c_master_write_byte(cmd, 0, ACK_CHECK_EN);
		i2c_master_stop(cmd);
		ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
		i2c_cmd_link_delete(cmd);
		if (ret != ESP_OK) {
			return ret;
		}
		First = 0;
	}
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	//    i2c_master_write_byte(cmd, BH1750_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
	//    i2c_master_write_byte(cmd, BH1750_CMD_START, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, (GY521_ADDR << 1 )| WRITE_BIT, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, 0x3B, ACK_CHECK_EN);
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	if (ret != ESP_OK) {
		return ret;
	}
	vTaskDelay(30 / portTICK_RATE_MS);
	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	//    i2c_master_write_byte(cmd, BH1750_SENSOR_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
	ret = i2c_master_write_byte(cmd, (GY521_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
	if (ret != ESP_OK)
		printf("%s:%d ret=%d\n", __FILE__, __LINE__, ret);
//	ret = i2c_master_read_byte(cmd, &high, ACK_VAL);
	ret = i2c_master_read(cmd, buf, sizeof(buf)-1, ACK_VAL);
	if (ret != ESP_OK)
		printf("%s:%d ret=%d\n", __FILE__, __LINE__, ret);
//	ret = i2c_master_read_byte(cmd, &low, NACK_VAL);
	ret = i2c_master_read_byte(cmd, buf+sizeof(buf)-1, NACK_VAL);
	if (ret != ESP_OK)
		printf("%s:%d ret=%d\n", __FILE__, __LINE__, ret);
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	*accx = buf[0]<<8 | buf[1];
	*accy = buf[2]<<8 | buf[3];
	*accz = buf[4]<<8 | buf[5];
	*temp = buf[6]<<8 | buf[7];
	*gyrox = buf[8]<<8 | buf[9];
	*gyroy = buf[10]<<8 | buf[11];
	*gyroz = buf[12]<<8 | buf[13];
	return ret;
}

/**
 * @brief i2c master initialization
 */
static void i2c_example_master_init()
{
    int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode,
                       I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                       I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
}

#if 0
/**
 * @brief test function to show buffer
 */
static void disp_buf(uint8_t* buf, int len)
{
    int i;
    for (i = 0; i < len; i++) {
        printf("%02x ", buf[i]);
        if (( i + 1 ) % 16 == 0) {
            printf("\n");
        }
    }
    printf("\n");
}
#endif

static void i2c_test_task(void* arg)
{
    static int16_t paccx=0, paccy=0, paccz=0, ptemp=0, pgyrox=0, pgyroy=0, pgyroz=0;
    int ret;
    uint32_t task_idx = (uint32_t) arg;
    int16_t accx, accy, accz, temp, gyrox, gyroy, gyroz;
    int cnt = 0;
    while (1) {
        printf("test cnt: %d\n", cnt++);
//        ret = i2c_example_master_sensor_test( I2C_EXAMPLE_MASTER_NUM, &sensor_data_h, &sensor_data_l);
        ret = i2c_example_master_sensor_test( I2C_EXAMPLE_MASTER_NUM, &accx, &accy, &accz, &temp, &gyrox, &gyroy, &gyroz);
//        xSemaphoreTake(print_mux, portMAX_DELAY);
        if(ret == ESP_ERR_TIMEOUT) {
            printf("I2C timeout\n");
        } else if(ret == ESP_OK) {
            printf("*******************\n");
            printf("TASK[%d]  MASTER READ SENSOR( GY-521 )\n", task_idx);
            printf("*******************\n");
#if 0
            printf("accx: %d (%+d%%)\n", accx, 100*(accx-paccx)/(paccx?paccx:1));
            printf("accy: %d (%+d%%)\n", accy, 100*(accy-paccy)/(paccy?paccy:1));
            printf("accz: %d (%+d%%)\n", accz, 100*(accz-paccz)/(paccz?paccz:1));
            printf("temp: %d (%+d%%)\n", temp, 100*(temp-ptemp)/(ptemp?ptemp:1));
            printf("gyrox: %d (%+d%%)\n", gyrox, 100*(gyrox-pgyrox)/(pgyrox?pgyrox:1));
            printf("gyroy: %d (%+d%%)\n", gyroy, 100*(gyroy-pgyroy)/(pgyroy?pgyroy:1));
            printf("gyroz: %d (%+d%%)\n", gyroz, 100*(gyroz-pgyroz)/(pgyroz?pgyroz:1));
#endif
            printf("accx: %d (%+d)\n", accx, (accx-paccx));
            printf("accy: %d (%+d)\n", accy, (accy-paccy));
            printf("accz: %d (%+d)\n", accz, (accz-paccz));
            printf("temp: %d (%+d)\n", temp, (temp-ptemp));
            printf("gyrox: %d (%+d)\n", gyrox, (gyrox-pgyrox));
            printf("gyroy: %d (%+d)\n", gyroy, (gyroy-pgyroy));
            printf("gyroz: %d (%+d)\n", gyroz, (gyroz-pgyroz));
	    paccx = accx;
	    paccy = accy;
	    paccz = accz;
	    ptemp = temp;
	    pgyrox = gyrox;
	    pgyroy = gyroy;
	    pgyroz = gyroz;
        } else {
		struct tm	*st;
		time_t		now;

            	printf("%s: No ack, sensor not connected...skip...\n", esp_err_to_name(ret));
		now = time(NULL);
		st = localtime(&now);
		printf("%04d/%02d/%02d %02d:%02d:%02d\n", st->tm_year+1900, st->tm_mon+1, st->tm_mday,
				st->tm_hour, st->tm_min, st->tm_sec);
        }
        vTaskDelay(( DELAY_TIME_BETWEEN_ITEMS_MS * ( task_idx + 1 ) ) / portTICK_RATE_MS);
    }
}

void servoSet(uint8_t servo, uint16_t val)
{
	uint16_t	safe;

	printf("servoSet servo %d val %d\n", servo, val);

	if (val < SERVOMIN)
		safe = SERVOMIN;
	else if (val > SERVOMAX)
		safe = SERVOMAX;
	else
		safe = val;

	LastPos[servo] = safe;
	if (pca9685_set_pwm_value(&Pca, servo, safe) != ESP_OK)
		printf("Could not set PWM value %d to servo %d\n", safe, servo);
	else
		printf("PWM value %d set to servo %d\n", safe, servo);
}

// return -1 if inverted, or 1
int isServoInverted(uint8_t servo)
{
	switch(servo)
	{
		case SERVOFRB:	return SERVORIGHTB;
		case SERVOFRM:	return SERVORIGHTM;
		case SERVOFRH:	return SERVORIGHTH;
		case SERVOFLB:	return SERVOLEFTB;
		case SERVOFLM:	return SERVOLEFTM;
		case SERVOFLH:	return SERVOLEFTH;
		case SERVORRB:	return SERVORIGHTB;
		case SERVORRM:	return SERVORIGHTM;
		case SERVORRH:	return SERVORIGHTH;
		case SERVORLB:	return SERVOLEFTB;
		case SERVORLM:	return SERVOLEFTM;
		case SERVORLH:	return SERVOLEFTH;
	}
	return 1;
}

void servoSet100(uint8_t servo, uint8_t val)
{
	uint16_t	cor;
	
	if (val > 100)
		cor = 100;
	else if (val < 0)
		cor = 0;
	else
		cor = val;

	if (isServoInverted(servo) == -1)
		cor = 100-cor;

	cor = SERVOMIN+(float)cor*SERVODIFF/100;

	printf("cor = %d\n", cor);
	return servoSet(servo, cor);
}

char *servoName(uint8_t servo)
{
	switch(servo)
	{
		case SERVOFRB:	return "SERVOFRONTRIGHTB";
		case SERVOFRM:	return "SERVOFRONTRIGHTM";
		case SERVOFRH:	return "SERVOFRONTRIGHTH";
		case SERVOFLB:	return "SERVOFRONTLEFTB";
		case SERVOFLM:	return "SERVOFRONTLEFTM";
		case SERVOFLH:	return "SERVOFRONTLEFTH";
		case SERVORRB:	return "SERVOREARRIGHTB";
		case SERVORRM:	return "SERVOREARRIGHTM";
		case SERVORRH:	return "SERVOREARRIGHTH";
		case SERVORLB:	return "SERVOREARLEFTB";
		case SERVORLM:	return "SERVOREARLEFTM";
		case SERVORLH:	return "SERVOREARLEFTH";
	}
	return "???";
}

void initPos()
{
	int	i;
	printf("initPos\n");
	for(i=0; i<12; i++)
		servoSet100(i, 10);
}

void move()
{
	int	i, val;
	static int cycle=0;
	static int sign=-1;

	printf("move: sign = %d, cycle = %d\n", sign, cycle);
	if (cycle % 30 == 0)
	{
		sign *= -1;
		printf("New sign = %d, cycle = %d\n", sign, cycle);
	}
	for(i=0; i<12; i++)
	{
		if (sign < 0)
			val = 40 - (cycle % 30);
		else
			val = 10 + (cycle % 30);
		printf("set servo %d pos %d\n", i, val);
		servoSet100(i, val);
	}
	cycle += 1;
}

void moveRun()
{
	int	i, val;
	static int cycle=0;
	static int sign=-1;

	printf("move: sign = %d, cycle = %d\n", sign, cycle);
	if (cycle % 30 == 0)
	{
		sign *= -1;
		printf("New sign = %d, cycle = %d\n", sign, cycle);
	}
	for(i=0; i<12; i++)
	{
		// servo high
		if (i%3 == 2)
			val = 50;
		else if (sign < 0)
			val = 40 - (cycle % 30);
		else
			val = 10 + (cycle % 30);
		printf("set servo %d pos %d\n", i, val);
		servoSet100(i, val);
	}
	cycle += 1;
}

static void pca_task(void* arg)
{
	uint16_t	freq=200;
	uint16_t	val;
	uint32_t task_idx = (uint32_t) arg;

	printf("pca_task start\n");
	memset(&LastPos, 0, sizeof(LastPos));
	printf("pca_task: %s:%d\n", __FILE__, __LINE__);
//	i2c_example_master_init();

	CHECK_LOOP(i2cdev_init(),
		"Could not init I2Cdev library");
	CHECK_LOOP( pca9685_init_desc(&Pca, PCA9685_ADDR, PCA9685_PORT, PCA9685_SDA, PCA9685_SCL),
		"Could not init device descriptor");
	CHECK_LOOP(pca9685_init(&Pca),
		"Could not init PCA9685");
	CHECK_LOOP(pca9685_restart(&Pca),
		"Could not restart");

	CHECK_LOOP(pca9685_set_pwm_frequency(&Pca, freq),
		"Could not set PWM frequency");
	CHECK_LOOP(pca9685_get_pwm_frequency(&Pca, &freq),
		"Could not get PWM frequency");
	printf("Freq real %d\n", freq);
	printf("pca_task: %s:%d\n", __FILE__, __LINE__);

	initPos();
	vTaskDelay(5000 / portTICK_PERIOD_MS);

	val = SERVOMIN+(SERVOMAX-SERVOMIN)/2;
	while (1)
	{
#if 0	
		if (val >= SERVOMAX)
			val = SERVOMIN;
		else if (val <= SERVOMIN)
			val = SERVOMIN+(SERVOMAX-SERVOMIN)/2;
		else
			val = SERVOMAX;
		
//		servoSet(1, val);
		if (pca9685_get_pwm_frequency(&p, &freq) != ESP_OK)
		{
			printf("Error pca_get_pwm_frequency() !\n");
		}
		else
			printf("frequency = %u\n", freq);
#endif

		move();
		vTaskDelay(500 / portTICK_PERIOD_MS);
//		vTaskDelay(( DELAY_TIME_BETWEEN_ITEMS_MS * ( task_idx + 1 ) ) / portTICK_RATE_MS);
	}
}

void app_main_old()
{
    i2c_example_master_init();
    xTaskCreate(i2c_test_task, "i2c_test_task_0", 1024 * 2, (void* ) 0, 10, NULL);
//    xTaskCreate(i2c_test_task, "i2c_test_task_1", 1024 * 2, (void* ) 1, 10, NULL);

}

//********************************
//********************************
//********** TCP SERVER **********
//********************************
//********************************
void tcp_server(void *pvParam)
{
	int socket_id;
	int bytes_received;
	char recv_buf[64];
	int client_socket;

	ESP_LOGI(TAG,"tcp_server task started \n");
	struct sockaddr_in tcpServerAddr;
	tcpServerAddr.sin_addr.s_addr = htonl(INADDR_ANY);
	tcpServerAddr.sin_family = AF_INET;
	tcpServerAddr.sin_port = htons( TCP_SERVER_PORT );
	static struct sockaddr_in remote_addr;
	static unsigned int socklen;
	socklen = sizeof(remote_addr);


	//----- WAIT FOR ETHERNET CONNECTED -----
	ESP_LOGI(TAG, "... waiting for ethernet connect \n");
	xEventGroupWaitBits(tcpserver_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);
	while(1)
	{
		//----- ALLOCATE SOCKET -----
		socket_id = socket(AF_INET, SOCK_STREAM, 0);
		if(socket_id < 0)
		{
			//Couldn't allocate socket
			ESP_LOGE(TAG, "... Failed to allocate socket.\n");
			vTaskDelay(1000 / portTICK_PERIOD_MS);
			continue;
		}
		ESP_LOGI(TAG, "... allocated socket\n");

		//----- BIND -----
		if(bind(socket_id, (struct sockaddr *)&tcpServerAddr, sizeof(tcpServerAddr)) != 0)
		{
			ESP_LOGE(TAG, "... socket bind failed errno=%d \n", errno);
			close(socket_id);
			vTaskDelay(4000 / portTICK_PERIOD_MS);
			continue;
		}
		ESP_LOGI(TAG, "... socket bind done \n");

		//----- LISTEN -----
		if(listen (socket_id, 2) != 0)
		{
			ESP_LOGE(TAG, "... socket listen failed errno=%d \n", errno);
			close(socket_id);
			vTaskDelay(4000 / portTICK_PERIOD_MS);
			continue;
		}

		while (1)
		{
			//Once the socket has been created it doesn't matter if the network connection is lost, the Ethernet cable is unplugged and then re-plugged in,
			//this while loop will still continue to work the next time a client connects

			//----- WAIT FOR A CLIENT CONNECTION -----
			ESP_LOGI(TAG,"Waiting for new client connection...");
			client_socket = accept(socket_id,(struct sockaddr *)&remote_addr, &socklen);		//<<<< WE STALL HERE WAITING FOR CONNECTION

			//----- A CLIENT HAS CONNECTED -----
			ESP_LOGI(TAG,"New client connection");

			//Optionally set O_NONBLOCK
			//If O_NONBLOCK is set then recv() will return, otherwise it will stall until data is received or the connection is lost.
			//fcntl(client_socket,F_SETFL,O_NONBLOCK);

			bzero(recv_buf, sizeof(recv_buf));
			while (1)
			{

				bytes_received = recv(client_socket, recv_buf, sizeof(recv_buf)-1, 0);		//<<<< WE STALL HERE WAITING FOR BYTES RECEIVED
				if (bytes_received == 0)
				{
					//----- CONNECTION LOST -----
					//There is no client any more - must have disconnected
					ESP_LOGI(TAG,"Client connection lost");
					break;
				}
				else if (bytes_received < 0)
				{
					//----- NO DATA WAITING -----
					//We'll only get here if O_NONBLOCK was set

					//vTaskDelay(50 / portTICK_PERIOD_MS);		//Release to RTOS scheduler
				}
				else
				{
					//----- DATA RECEIVED -----
					ESP_LOGI(TAG,"Data received:");
					for(int i = 0; i < bytes_received; i++)
						putchar(recv_buf[i]);
					ESP_LOGI(TAG,"Data receive complete");

					//Clear the buffer for next time (not acutally needed but may as well)
					bzero(recv_buf, sizeof(recv_buf));

					//----- TRANSMIT -----
					if (write(client_socket , "Hello!" , strlen("Hello!")) < 0)
					{
						ESP_LOGE(TAG, "Transmit failed");
						close(socket_id);
						vTaskDelay(4000 / portTICK_PERIOD_MS);
						continue;
					}
					ESP_LOGI(TAG, "Transmit complete");
				}
			} //while (1)

			//We won't actually get here, the while loop never exits (unless its implementation gets changed!)

			//----- CLOSE THE SOCKET -----
			ESP_LOGI(TAG, "Closing socket");
			close(client_socket);
		}
		ESP_LOGI(TAG, "TCP server will be opened again in 3 secs...");
		vTaskDelay(3000 / portTICK_PERIOD_MS);


	}
	ESP_LOGI(TAG, "TCP client task closed\n");
}

static void tcp_server_init(void)
{
	int ret;
	xTaskHandle tcp_handle;

	ret = xTaskCreate(tcp_server,
                      TCP_TASK_NAME,
                      TCP_TASK_STACK_WORDS,
                      NULL,
                      TCP_TASK_PRIORITY,
                      &tcp_handle); 

    if (ret != pdPASS)  {
        ESP_LOGI(TAG, "create task %s failed", TCP_TASK_NAME);
    }
}

static esp_err_t wifi_event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
        tcp_server_init();
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        /* This is a workaround as ESP32 WiFi libs don't currently
           auto-reassociate. */
        esp_wifi_connect(); 
        xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
        break;
    default:
        break;
    }
    return ESP_OK;
}

static void wifi_conn_init(void)
{
    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK( esp_event_loop_init(wifi_event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_WIFI_SSID,
            .password = EXAMPLE_WIFI_PASS,
        },
    };
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_LOGI(TAG, "start the WIFI SSID:[%s] password:[%s]\n", EXAMPLE_WIFI_SSID, EXAMPLE_WIFI_PASS);
    ESP_ERROR_CHECK( esp_wifi_start() );
}

void app_main()
{
	ESP_ERROR_CHECK( nvs_flash_init() );
	wifi_conn_init();
	xTaskCreate(pca_task, "pca", 1024 * 2, (void* ) 0, 10, NULL);
//    xTaskCreate(i2c_test_task, "i2c_test_task_0", 1024 * 2, (void* ) 0, 10, NULL);
//    xTaskCreate(i2c_test_task, "i2c_test_task_1", 1024 * 2, (void* ) 1, 10, NULL);

}


















#if 0
#include <stdio.h>
#include <string.h>
#include <sys/fcntl.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_err.h"
#include "esp_event_loop.h"
#include "freertos/event_groups.h"
#include "esp_event.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_eth.h"

#include "rom/ets_sys.h"
#include "rom/gpio.h"

#include "soc/dport_reg.h"
#include "soc/io_mux_reg.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/gpio_reg.h"
#include "soc/gpio_sig_map.h"

#include "tcpip_adapter.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

#include "eth_phy/phy_lan8720.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"

#define DEFAULT_ETHERNET_PHY_CONFIG phy_lan8720_default_ethernet_config
// #define PIN_PHY_POWER CONFIG_PHY_POWER_PIN				//not needed for our board configuration

#define PIN_SMI_MDC   GPIO_NUM_23
#define PIN_SMI_MDIO  GPIO_NUM_18

#define	TCP_SERVER_PORT			3000



//******************************************
//******************************************
//********** ETHERNET GPIO CONFIG **********
//******************************************
//******************************************
static void ethernet_gpio_config_rmii(void)
{
    //RMII data pins are fixed:
    //	TXD0 = GPIO19
    //	TXD1 = GPIO22
    //	TX_EN = GPIO21
    //	RXD0 = GPIO25
    //	RXD1 = GPIO26
    //	CLK == GPIO0
    phy_rmii_configure_data_interface_pins();
    phy_rmii_smi_configure_pins(PIN_SMI_MDC, PIN_SMI_MDIO);
}


//*****************************************
//*****************************************
//********** ETHERNET INITIALISE **********
//*****************************************
//*****************************************
void ethernet_initialise (void)
{
	esp_err_t ret = ESP_OK;
	tcpip_adapter_init();
	ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );

	eth_config_t config = DEFAULT_ETHERNET_PHY_CONFIG;

	config.phy_addr = CONFIG_PHY_ADDRESS;				//Set the PHY address in configuration
	config.gpio_config = ethernet_gpio_config_rmii;
	config.tcpip_input = tcpip_adapter_eth_input;

	ret = esp_eth_init(&config);

	if(ret == ESP_OK)
	{
		esp_eth_enable();
		tcpserver_event_group = xEventGroupCreate();
		xTaskCreate(&tcp_server, "tcp_server", 4096, NULL, 5, NULL);
	}
}



//***************************************
//***************************************
//********** ESP EVENT HANDLER **********
//***************************************
//***************************************
//Call this at startup:
//ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
static esp_err_t event_handler(void *ctx, system_event_t *event)
{
	switch(event->event_id)
	{
	case SYSTEM_EVENT_ETH_CONNECTED:
		//Ethernet phy link up
		xEventGroupSetBits(tcpserver_event_group, CONNECTED_BIT);
		break;

	case SYSTEM_EVENT_ETH_DISCONNECTED:
		//Ethernet phy link down
		xEventGroupClearBits(tcpserver_event_group, CONNECTED_BIT);
		break;

	case SYSTEM_EVENT_ETH_GOT_IP:
		//Ethernet got IP from connected AP

		break;

	default:
		break;
	}
	return ESP_OK;
}



//********************************
//********************************
//********** TCP SERVER **********
//********************************
//********************************
void tcp_server(void *pvParam)
{
	int socket_id;
	int bytes_received;
	char recv_buf[64];
	int client_socket;

	ESP_LOGI(EthernetLogTag,"tcp_server task started \n");
	struct sockaddr_in tcpServerAddr;
	tcpServerAddr.sin_addr.s_addr = htonl(INADDR_ANY);
	tcpServerAddr.sin_family = AF_INET;
	tcpServerAddr.sin_port = htons( TCP_SERVER_PORT );
	static struct sockaddr_in remote_addr;
	static unsigned int socklen;
	socklen = sizeof(remote_addr);


	//----- WAIT FOR ETHERNET CONNECTED -----
	ESP_LOGI(EthernetLogTag, "... waiting for ethernet connect \n");
	xEventGroupWaitBits(tcpserver_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);
	while(1)
	{
		//----- ALLOCATE SOCKET -----
		socket_id = socket(AF_INET, SOCK_STREAM, 0);
		if(socket_id < 0)
		{
			//Couldn't allocate socket
			ESP_LOGE(EthernetLogTag, "... Failed to allocate socket.\n");
			vTaskDelay(1000 / portTICK_PERIOD_MS);
			continue;
		}
		ESP_LOGI(EthernetLogTag, "... allocated socket\n");

		//----- BIND -----
		if(bind(socket_id, (struct sockaddr *)&tcpServerAddr, sizeof(tcpServerAddr)) != 0)
		{
			ESP_LOGE(EthernetLogTag, "... socket bind failed errno=%d \n", errno);
			close(socket_id);
			vTaskDelay(4000 / portTICK_PERIOD_MS);
			continue;
		}
		ESP_LOGI(EthernetLogTag, "... socket bind done \n");

		//----- LISTEN -----
		if(listen (socket_id, 2) != 0)
		{
			ESP_LOGE(EthernetLogTag, "... socket listen failed errno=%d \n", errno);
			close(socket_id);
			vTaskDelay(4000 / portTICK_PERIOD_MS);
			continue;
		}

		while (1)
		{
			//Once the socket has been created it doesn't matter if the network connection is lost, the Ethernet cable is unplugged and then re-plugged in,
			//this while loop will still continue to work the next time a client connects

			//----- WAIT FOR A CLIENT CONNECTION -----
			ESP_LOGI(EthernetLogTag,"Waiting for new client connection...");
			client_socket = accept(socket_id,(struct sockaddr *)&remote_addr, &socklen);		//<<<< WE STALL HERE WAITING FOR CONNECTION

			//----- A CLIENT HAS CONNECTED -----
			ESP_LOGI(EthernetLogTag,"New client connection");

			//Optionally set O_NONBLOCK
			//If O_NONBLOCK is set then recv() will return, otherwise it will stall until data is received or the connection is lost.
			//fcntl(client_socket,F_SETFL,O_NONBLOCK);

			bzero(recv_buf, sizeof(recv_buf));
			while (1)
			{

				bytes_received = recv(client_socket, recv_buf, sizeof(recv_buf)-1, 0);		//<<<< WE STALL HERE WAITING FOR BYTES RECEIVED
				if (bytes_received == 0)
				{
					//----- CONNECTION LOST -----
					//There is no client any more - must have disconnected
					ESP_LOGI(EthernetLogTag,"Client connection lost");
					break;
				}
				else if (bytes_received < 0)
				{
					//----- NO DATA WAITING -----
					//We'll only get here if O_NONBLOCK was set

					//vTaskDelay(50 / portTICK_PERIOD_MS);		//Release to RTOS scheduler
				}
				else
				{
					//----- DATA RECEIVED -----
					ESP_LOGI(EthernetLogTag,"Data received:");
					for(int i = 0; i < bytes_received; i++)
						putchar(recv_buf[i]);
					ESP_LOGI(EthernetLogTag,"Data receive complete");

					//Clear the buffer for next time (not acutally needed but may as well)
					bzero(recv_buf, sizeof(recv_buf));

					//----- TRANSMIT -----
					if (write(client_socket , "Hello!" , strlen("Hello!")) < 0)
					{
						ESP_LOGE(EthernetLogTag, "Transmit failed");
						close(socket_id);
						vTaskDelay(4000 / portTICK_PERIOD_MS);
						continue;
					}
					ESP_LOGI(EthernetLogTag, "Transmit complete");
				}
			} //while (1)

			//We won't actually get here, the while loop never exits (unless its implementation gets changed!)

			//----- CLOSE THE SOCKET -----
			ESP_LOGI(EthernetLogTag, "Closing socket");
			close(client_socket);
		}
		ESP_LOGI(EthernetLogTag, "TCP server will be opened again in 3 secs...");
		vTaskDelay(3000 / portTICK_PERIOD_MS);


	}
	ESP_LOGI(EthernetLogTag, "TCP client task closed\n");
}


static esp_err_t wifi_event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
        openssl_server_init();
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        /* This is a workaround as ESP32 WiFi libs don't currently
           auto-reassociate. */
        esp_wifi_connect(); 
        xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
        break;
    default:
        break;
    }
    return ESP_OK;
}

static void wifi_conn_init(void)
{
    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK( esp_event_loop_init(wifi_event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_WIFI_SSID,
            .password = EXAMPLE_WIFI_PASS,
        },
    };
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_LOGI(TAG, "start the WIFI SSID:[%s] password:[%s]\n", EXAMPLE_WIFI_SSID, EXAMPLE_WIFI_PASS);
    ESP_ERROR_CHECK( esp_wifi_start() );
}

void app_main(void)
{
    ESP_ERROR_CHECK( nvs_flash_init() );
    wifi_conn_init();
}
#endif
