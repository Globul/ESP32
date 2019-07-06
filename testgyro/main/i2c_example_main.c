/* i2c - Example

   For other examples please check:
   https://github.com/espressif/esp-idf/tree/master/examples

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <time.h>
#include "driver/i2c.h"

/**
 * TEST CODE BRIEF
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

int	First = 1;

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

void app_main()
{
    i2c_example_master_init();
    xTaskCreate(i2c_test_task, "i2c_test_task_0", 1024 * 2, (void* ) 0, 10, NULL);
//    xTaskCreate(i2c_test_task, "i2c_test_task_1", 1024 * 2, (void* ) 1, 10, NULL);

}

