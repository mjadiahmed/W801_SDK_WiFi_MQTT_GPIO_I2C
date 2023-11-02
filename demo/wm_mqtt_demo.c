/*****************************************************************************
*
* File Name : wm_mqtt_demo.c
*
* Description: mqtt demo function
*
* Copyright (c) 2015 Winner Micro Electronic Design Co., Ltd.
* All rights reserved.
*
* Author : LiLimin
*
* Date : 2019-3-24
*****************************************************************************/
#include <string.h>
#include "wm_include.h"
#include "wm_netif.h"
#include "wm_demo.h"
#include "tls_common.h"
#include "lwip/netif.h"
#include "wm_sockets.h"
#include "lwip/inet.h"
#include "wm_sockets2.0.3.h"
#include "libemqtt.h"

#include "wm_i2c.h"
#include "wm_type_def.h"
#include "wm_gpio_afsel.h"

#define PIN_SCL WM_IO_PA_01
#define PIN_SDA WM_IO_PA_04
#define I2C_FREQ (200000)



#if DEMO_MQTT

#define MQTT_DEMO_TASK_PRIO             39
#define MQTT_DEMO_TASK_SIZE             512
#define MQTT_DEMO_QUEUE_SIZE            4

#define MQTT_DEMO_RECV_BUF_LEN_MAX      1024

#define MQTT_DEMO_CMD_START             0x1
#define MQTT_DEMO_CMD_HEART             0x2
#define MQTT_DEMO_CMD_LOOP              0x3

#define MQTT_DEMO_READ_TIMEOUT        (-1000)

#define MQTT_DEMO_READ_TIME_SEC         1
#define MQTT_DEMO_READ_TIME_US          0

#define MQTT_DEMO_CLIENT_ID            "opzeirjgzjerzergzerg"
#define MQTT_DEMO_TX_PUB_TOPIC         "winnermicro/amj_tx"
#define MQTT_DEMO_RX_PUB_TOPIC         "winnermicro/amj_rx"

#define MQTT_DEMO_SERVER_ADDR          "test.mosquitto.org"
#define MQTT_DEMO_SERVER_PORT           1883


/*LTR390*/
// LTR390 Sensor Register Addresses
#define LTR390_ADDR       (0x53)
#define LTR390_MAIN_CTRL  (0x00) // Main control register
#define LTR390_MEAS_RATE  (0x04) // Resolution and data rate
#define LTR390_GAIN       (0x05) // ALS and UVS gain range
#define LTR390_ALSDATA    (0x0D) // ALS data lowest byte, 3 bytes
#define LTR390_UVSDATA    (0x10) // UVS data lowest byte, 3 bytes

// Measurement Resolution, Gain Setting, Measurement Rate
#define RESOLUTION_20BIT_TIME400MS (0x00)
#define RESOLUTION_19BIT_TIME200MS (0x10)
#define RESOLUTION_18BIT_TIME100MS (0x20) // Default
#define RESOLUTION_17BIT_TIME50MS  (0x3)
#define RESOLUTION_16BIT_TIME25MS  (0x40)
#define RESOLUTION_13BIT_TIME12_5MS  (0x50)
#define RATE_25MS (0x0)
#define RATE_50MS (0x1)
#define RATE_100MS (0x2) // Default
#define RATE_200MS (0x3)
#define RATE_500MS (0x4)
#define RATE_1000MS (0x5)
#define RATE_2000MS (0x6)

// Measurement Gain Range
#define GAIN_1  (0x0)
#define GAIN_3  (0x1) // Default
#define GAIN_6  (0x2)
#define GAIN_9  (0x3)
#define GAIN_18 (0x4)

// Constants for LTR390 sensor configuration
#define UV_SENSITIVITY 1400
#define WFAC 1.0

/*BME680*/#define BME680_ADDR (0x76)  // You might need to adjust this address if you have a different sensor or address

// BME680 registers
#define BME680_REG_CTRL_MEAS (0x74)
#define BME680_REG_TEMP_XLSB (0x22)
#define BME680_REG_TEMP_LSB (0x23)
#define BME680_REG_TEMP_MSB (0x24)

// BME680 constants for sensor configuration
#define BME680_TEMP_OS_1X (0x20)


void i2c_init() ;
void ltr390_init() ;
int readLux() ;

static bool mqtt_demo_inited = FALSE;
static OS_STK mqtt_demo_task_stk[MQTT_DEMO_TASK_SIZE];
static tls_os_queue_t *mqtt_demo_task_queue = NULL;
static tls_os_timer_t *mqtt_demo_heartbeat_timer = NULL;

static int mqtt_demo_socket_id;
static int mqtt_demo_mqtt_keepalive = 300;
static mqtt_broker_handle_t mqtt_demo_mqtt_broker;

static uint8_t mqtt_demo_packet_buffer[MQTT_DEMO_RECV_BUF_LEN_MAX];

extern struct netif *tls_get_netif(void);
extern int wm_printf(const char *fmt, ...);

void i2c_scanner(void); // works well

static void mqtt_demo_net_status(u8 status)
{
    struct netif *netif = tls_get_netif();

    switch(status)
    {
    case NETIF_WIFI_JOIN_FAILED:
        wm_printf("sta join net failed\n");
        break;
    case NETIF_WIFI_DISCONNECTED:
        wm_printf("sta net disconnected\n");
        break;
    case NETIF_IP_NET_UP:
        wm_printf("sta ip: %v\n", netif->ip_addr.addr);
        tls_os_queue_send(mqtt_demo_task_queue, (void *)MQTT_DEMO_CMD_START, 0);
        break;
    default:
        break;
    }
}

static void mqtt_demo_heart_timer(void *ptmr, void *parg)
{
    tls_os_queue_send(mqtt_demo_task_queue, (void *)MQTT_DEMO_CMD_HEART, 0);
}

static int mqtt_demo_close_socket(mqtt_broker_handle_t *broker)
{
    int fd = broker->socketid;
    return closesocket(fd);
}

static int mqtt_demo_send_packet(int socket_info, const void *buf, unsigned int count)
{
    int fd = socket_info;
    return send(fd, buf, count, 0);
}

static int mqtt_demo_read_packet(int sec, int us)
{
    int ret = 0;

    if ((sec >= 0) || (us >= 0))
    {
        fd_set readfds;
        struct timeval tmv;

        // Initialize the file descriptor set
        FD_ZERO (&readfds);
        FD_SET (mqtt_demo_socket_id, &readfds);

        // Initialize the timeout data structure
        tmv.tv_sec = sec;
        tmv.tv_usec = us;

        // select returns 0 if timeout, 1 if input available, -1 if error
        ret = select(mqtt_demo_socket_id + 1, &readfds, NULL, NULL, &tmv);
        if(ret < 0)
            return -2;
        else if(ret == 0)
            return MQTT_DEMO_READ_TIMEOUT;

    }

    int total_bytes = 0, bytes_rcvd, packet_length;
    memset(mqtt_demo_packet_buffer, 0, sizeof(mqtt_demo_packet_buffer));

    if((bytes_rcvd = recv(mqtt_demo_socket_id, (mqtt_demo_packet_buffer + total_bytes), MQTT_DEMO_RECV_BUF_LEN_MAX, 0)) <= 0)
    {
        //printf("%d, %d\r\n", bytes_rcvd, mqtt_demo_socket_id);
        return -1;
    }
    //printf("recv [len=%d] : %s\n", bytes_rcvd, mqtt_demo_packet_buffer);
    total_bytes += bytes_rcvd; // Keep tally of total bytes
    if (total_bytes < 2)
        return -1;

    // now we have the full fixed header in mqtt_demo_packet_buffer
    // parse it for remaining length and number of bytes
    uint16_t rem_len = mqtt_parse_rem_len(mqtt_demo_packet_buffer);
    uint8_t rem_len_bytes = mqtt_num_rem_len_bytes(mqtt_demo_packet_buffer);

    //packet_length = mqtt_demo_packet_buffer[1] + 2; // Remaining length + fixed header length
    // total packet length = remaining length + byte 1 of fixed header + remaning length part of fixed header
    packet_length = rem_len + rem_len_bytes + 1;

    while(total_bytes < packet_length) // Reading the packet
    {
        if((bytes_rcvd = recv(mqtt_demo_socket_id, (mqtt_demo_packet_buffer + total_bytes), MQTT_DEMO_RECV_BUF_LEN_MAX, 0)) <= 0)
            return -1;
        total_bytes += bytes_rcvd; // Keep tally of total bytes
    }

    return packet_length;
}

static int mqtt_demo_init_socket(mqtt_broker_handle_t *broker, const char *hostname, short port, int keepalive)
{
    int flag = 1;
    struct hostent *hp;

    // Create the socket
    if((mqtt_demo_socket_id = socket(PF_INET, SOCK_STREAM, 0)) < 0)
        return -1;

    // Disable Nagle Algorithm
    if (setsockopt(mqtt_demo_socket_id, IPPROTO_TCP, 0x01, (char *)&flag, sizeof(flag)) < 0)
    {
        mqtt_demo_close_socket(&mqtt_demo_mqtt_broker);
        return -2;
    }

    // query host ip start
    hp = gethostbyname(hostname);
    if (hp == NULL )
    {
        mqtt_demo_close_socket(&mqtt_demo_mqtt_broker);
        return -2;
    }

    struct sockaddr_in socket_address;
    memset(&socket_address, 0, sizeof(struct sockaddr_in));
    socket_address.sin_family = AF_INET;
    socket_address.sin_port = htons(port);
    memcpy(&(socket_address.sin_addr), hp->h_addr, hp->h_length);

    // Connect the socket
    if((connect(mqtt_demo_socket_id, (struct sockaddr *)&socket_address, sizeof(socket_address))) < 0)
    {
        mqtt_demo_close_socket(&mqtt_demo_mqtt_broker);
        return -1;
    }

    // MQTT stuffs
    mqtt_set_alive(broker, mqtt_demo_mqtt_keepalive);
    broker->socketid = mqtt_demo_socket_id;
    broker->mqttsend = mqtt_demo_send_packet;
    //wm_printf("socket id = %d\n", mqtt_demo_socket_id);
    return 0;
}

static int mqtt_demo_init(void)
{
    int packet_length, ret = 0;
    uint16_t msg_id, msg_id_rcv;

    wm_printf("[1]: init mqtt lib.\r\n");
    mqtt_init(&mqtt_demo_mqtt_broker, MQTT_DEMO_CLIENT_ID);

    wm_printf("[2]: establishing TCP connection.\r\n");
    ret = mqtt_demo_init_socket(&mqtt_demo_mqtt_broker, MQTT_DEMO_SERVER_ADDR, MQTT_DEMO_SERVER_PORT, mqtt_demo_mqtt_keepalive);
    if(ret)
    {
        wm_printf("init_socket ret=%d\n", ret);
        return -4;
    }

    wm_printf("[3]: establishing mqtt connection.\r\n");
    ret = mqtt_connect(&mqtt_demo_mqtt_broker);
    if(ret)
    {
        wm_printf("mqtt_connect ret=%d\n", ret);
        return -5;
    }

    packet_length = mqtt_demo_read_packet(MQTT_DEMO_READ_TIME_SEC, MQTT_DEMO_READ_TIME_US);
    if(packet_length < 0)
    {
        wm_printf("Error(%d) on read packet!\n", packet_length);
        mqtt_demo_close_socket(&mqtt_demo_mqtt_broker);
        return -1;
    }

    if(MQTTParseMessageType(mqtt_demo_packet_buffer) != MQTT_MSG_CONNACK)
    {
        wm_printf("CONNACK expected!\n");
        mqtt_demo_close_socket(&mqtt_demo_mqtt_broker);
        return -2;
    }

    if(mqtt_demo_packet_buffer[3] != 0x00)
    {
        wm_printf("CONNACK failed!\n");
        mqtt_demo_close_socket(&mqtt_demo_mqtt_broker);
        return -2;
    }


    wm_printf("[4]: subscribe mqtt\r\n");
    mqtt_subscribe(&mqtt_demo_mqtt_broker, MQTT_DEMO_TX_PUB_TOPIC, &msg_id);

    packet_length = mqtt_demo_read_packet(MQTT_DEMO_READ_TIME_SEC, MQTT_DEMO_READ_TIME_US);
    if(packet_length < 0)
    {
        wm_printf("Error(%d) on read packet!\n", packet_length);
        mqtt_demo_close_socket(&mqtt_demo_mqtt_broker);
        return -1;
    }

    if(MQTTParseMessageType(mqtt_demo_packet_buffer) != MQTT_MSG_SUBACK)
    {
        wm_printf("SUBACK expected!\n");
        mqtt_demo_close_socket(&mqtt_demo_mqtt_broker);
        return -2;
    }

    msg_id_rcv = mqtt_parse_msg_id(mqtt_demo_packet_buffer);
    if(msg_id != msg_id_rcv)
    {
        wm_printf("%d message id was expected, but %d message id was found!\n", msg_id, msg_id_rcv);
        mqtt_demo_close_socket(&mqtt_demo_mqtt_broker);
        return -3;
    }

    wm_printf("[5]: start the Heart-beat preservation timer\r\n");
    ret = tls_os_timer_create(&mqtt_demo_heartbeat_timer,
                              mqtt_demo_heart_timer,
                              NULL, (10 * HZ), TRUE, NULL);
    if (TLS_OS_SUCCESS == ret)
        tls_os_timer_start(mqtt_demo_heartbeat_timer);

    /* step6: push mqtt subscription (when a subscription message is received) */

    return 0;
}

static int mqtt_demo_loop(void)
{
    int packet_length = 0;
    int counter = 0;

    counter++;
    packet_length = mqtt_demo_read_packet(0, 1);
    if(packet_length > 0)
    {
        wm_printf("received Packet Header: 0x%x...\n", mqtt_demo_packet_buffer[0]);

        if (MQTTParseMessageType(mqtt_demo_packet_buffer) == MQTT_MSG_PUBLISH)
        {
            uint8_t topic[100], *msg;
            uint16_t len;
            len = mqtt_parse_pub_topic(mqtt_demo_packet_buffer, topic);
            topic[len] = '\0'; // for printf
            len = mqtt_parse_publish_msg(mqtt_demo_packet_buffer, &msg);
            msg[len] = '\0'; // for printf
            wm_printf("[MQTT] Received: %s <<< %s\n", topic, msg);
            
            tls_gpio_write(WM_IO_PB_17,0);	
            tls_os_time_delay(500);
            tls_gpio_write(WM_IO_PB_17,1);	
            mqtt_publish(&mqtt_demo_mqtt_broker, (const char *)MQTT_DEMO_RX_PUB_TOPIC, (const char *)msg, len, 0);
            wm_printf("[MQTT] Pushed: %s >>>%s\n", MQTT_DEMO_RX_PUB_TOPIC, msg);
            tls_gpio_write(WM_IO_PB_16,0);	
            tls_os_time_delay(500);
            tls_gpio_write(WM_IO_PB_16,1);	

        }

        tls_os_queue_send(mqtt_demo_task_queue, (void *)MQTT_DEMO_CMD_LOOP, 0);
    }
    else if (packet_length == MQTT_DEMO_READ_TIMEOUT)
    {
        tls_os_queue_send(mqtt_demo_task_queue, (void *)MQTT_DEMO_CMD_LOOP, 0);
    }
    else if(packet_length == -1)
    {
        wm_printf("[E] mqtt error:(%d)!\n", packet_length);
     //   tls_os_timer_stop(mqtt_demo_heartbeat_timer);
     //   mqtt_demo_close_socket(&mqtt_demo_mqtt_broker);
    }

    return 0;
}

static void mqtt_demo_task(void *p)
{
    int ret;
    void *msg;
    struct tls_ethif *ether_if = tls_netif_get_ethif();
    
   // demo_connect_net("IAM","19660430");
    
    if (ether_if->status)
    {
        wm_printf("sta ip: %v\n", ether_if->ip_addr.addr);
        tls_os_queue_send(mqtt_demo_task_queue, (void *)MQTT_DEMO_CMD_START, 0);
    }

    for ( ; ; )
    {
        ret = tls_os_queue_receive(mqtt_demo_task_queue, (void **)&msg, 0, 0);
        if (!ret)
        {
            switch((u32)msg)
            {
            case MQTT_DEMO_CMD_START:
                do
                {
                    ret = mqtt_demo_init();
                    if (ret)
                        break;
                    tls_os_queue_send(mqtt_demo_task_queue, (void *)MQTT_DEMO_CMD_LOOP, 0);
                }
                while (0);
                break;
            case MQTT_DEMO_CMD_HEART:
                //wm_printf("send heart ping\r\n");
                //mqtt_ping(&mqtt_demo_mqtt_broker);
                mqtt_publish(&mqtt_demo_mqtt_broker, (const char *)MQTT_DEMO_RX_PUB_TOPIC, (const char *)"WM801- By AHMED MJADI", 21, 0);
                wm_printf("[MQTT] Pushed: %s >>>%s\n", MQTT_DEMO_RX_PUB_TOPIC, msg);
                tls_gpio_write(WM_IO_PB_05,1);	
                tls_os_time_delay(1000);
                tls_gpio_write(WM_IO_PB_05,0);	

                break;
            case MQTT_DEMO_CMD_LOOP:
                mqtt_demo_loop();
                break;
            default:
                mqtt_publish(&mqtt_demo_mqtt_broker, (const char *)MQTT_DEMO_RX_PUB_TOPIC, (const char *)"WM801- By AHMED MJADI", 21, 0);
                wm_printf("[MQTT] Pushed: %s >>>%s\n", MQTT_DEMO_RX_PUB_TOPIC, msg);
                ls_gpio_write(WM_IO_PB_16,0);	
                tls_os_time_delay(500);
                tls_gpio_write(WM_IO_PB_16,1);	

                break;
            }
        }
    }
}


//mqtt demo
//���Է�����:mqtt.yichen.link:3883
//�����������ڷ��͵Ķ�������Ϊ:winnermicro/mqtt_tx_demo
//�����������ڽ��յĶ�������Ϊ:winnermicro/mqtt_rx_demo
//��������: ���յ�winnermicro/mqtt_tx_demo���͵���Ϣ���ӡ����Ļ�ϣ����ٴ����͵�winnermicro/mqtt_rx_demo






int mqtt_demo(void)
{
    if (!mqtt_demo_inited)
    {
        tls_os_task_create(NULL, NULL, mqtt_demo_task,
                           NULL, (void *)mqtt_demo_task_stk,  /* task's stack start address */
                           MQTT_DEMO_TASK_SIZE * sizeof(u32), /* task's stack size, unit:byte */
                           MQTT_DEMO_TASK_PRIO, 0);

        tls_os_queue_create(&mqtt_demo_task_queue, MQTT_DEMO_QUEUE_SIZE);

        tls_netif_add_status_event(mqtt_demo_net_status);

        mqtt_demo_inited = TRUE;
    }

    return WM_SUCCESS;
}

// LEDS invers LOgic
#define HIGH 0
#define LOW 1
char *SSID_ = "Nextronic" ;
char *PASS_ = "Next2021";


static void mqtt_task(void *p)
{
    int ret;
    void *msg;
    struct tls_ethif *ether_if = tls_netif_get_ethif();
    u16 LEDs[7] = {WM_IO_PB_05, WM_IO_PB_25, WM_IO_PB_26, WM_IO_PB_18, WM_IO_PB_17, WM_IO_PB_16, WM_IO_PB_11};

    for (int i = 0; i < 7; i++) // SETUP LEDS
    {
        tls_gpio_cfg(LEDs[i], WM_GPIO_DIR_OUTPUT, WM_GPIO_ATTR_FLOATING);
        tls_gpio_write(LEDs[i],HIGH);	
        //tls_os_time_delay(50);
        //tls_gpio_write(LEDs[i],LOW);	
    }
    
    //     for (int i = 0; i < 7; i++) //turn All high
    // {
    //     tls_gpio_write(LEDs[i],HIGH);	
    //     tls_os_time_delay(50);
    // }
    

    // demo_connect_net(SSID_, PASS_);

    demo_connect_net("Nextronic", "Next2021");
    
    if (ether_if->status)
    {
        wm_printf("sta ip: %v\n", ether_if->ip_addr.addr);
        tls_os_queue_send(mqtt_demo_task_queue, (void *)MQTT_DEMO_CMD_START, 0);
        for (int i = 0; i < 7; i++) // SETUP LEDS
        {
            tls_gpio_cfg(LEDs[i], WM_GPIO_DIR_OUTPUT, WM_GPIO_ATTR_FLOATING);
            tls_gpio_write(LEDs[i],HIGH);	
    
        }
  
    }
    
    /*CONFIG I2C*/
   // wm_printf("[d] Setup I2c\r\n");
    /* wm_i2c_scl_config(PIN_SCL);
	 wm_i2c_sda_config(PIN_SDA);
	 tls_i2c_init(I2C_FREQ);*/
   
    i2c_init(); 
   
    ltr390_init();

    bme680_init() ;

    for ( ; ; )
    {
	
        i2c_scanner(); // works well

        int luxValue = readLux();
        wm_printf("******************* Lux value: %d\n", luxValue);
      
        int temperature = readTemperature();
        wm_printf("------------ Temperature: %d.%02d degrees Celsius\n", temperature / 100, temperature % 100);
    


       tls_os_time_delay(2000);
    
        /*MQTT FUNCTIONS*/
        // mqtt_demo_init();
        // wm_printf("send heart ping\r\n");
        // mqtt_ping(&mqtt_demo_mqtt_broker);


        ret = tls_os_queue_receive(mqtt_demo_task_queue, (void **)&msg, 0, 0);
        if (!ret)
        {
            switch((u32)msg)
            {
            case MQTT_DEMO_CMD_START:
                do
                {
                    ret = mqtt_demo_init();
                    if (ret)
                        break;
                    tls_os_queue_send(mqtt_demo_task_queue, (void *)MQTT_DEMO_CMD_LOOP, 0);
                }
                while (0);
                break;
            case MQTT_DEMO_CMD_HEART:
                wm_printf("send heart ping\r\n");
                mqtt_ping(&mqtt_demo_mqtt_broker);
                //mqtt_publish(&mqtt_demo_mqtt_broker, (const char *)MQTT_DEMO_RX_PUB_TOPIC, (const char *)"Hello from W801 - A.M.", 22, 0);
                i2c_scanner(); // works well
                wm_printf("pushed: %s >>>%s\n", MQTT_DEMO_RX_PUB_TOPIC, msg);
                  for (int i = 0; i < 7; i++) // SETUP LEDS
                    {
                        tls_gpio_cfg(LEDs[i], WM_GPIO_DIR_OUTPUT, WM_GPIO_ATTR_FLOATING);
                        tls_gpio_write(LEDs[i],LOW);	
                        tls_os_time_delay(50);
                        tls_gpio_write(LEDs[i],HIGH);	
                    }
                //to do : tls_get_mac_addr
                break;
            case MQTT_DEMO_CMD_LOOP:
                mqtt_demo_loop();
                break;
            default:
                mqtt_publish(&mqtt_demo_mqtt_broker, (const char *)MQTT_DEMO_RX_PUB_TOPIC, (const char *)"Hello from W801 - A.M.", 22, 0);
                wm_printf("pushed: %s >>>%s\n", MQTT_DEMO_RX_PUB_TOPIC, msg);
                  for (int i = 0; i < 7; i++) // SETUP LEDS
                    {
                        tls_gpio_cfg(LEDs[i], WM_GPIO_DIR_OUTPUT, WM_GPIO_ATTR_FLOATING);
                        tls_gpio_write(LEDs[i],HIGH);	
                        tls_os_time_delay(50);
                        tls_gpio_write(LEDs[i],LOW);	
                    }
                break;
            }
        }
          mqtt_publish(&mqtt_demo_mqtt_broker, "winnermicroMJ", "LOOP publishing", 16, 0);
  }
}




void CreateMqttTAsk(){

  tls_os_task_create(NULL, NULL, mqtt_task,
                           NULL, (void *)mqtt_demo_task_stk,  /* task's stack start address */
                           MQTT_DEMO_TASK_SIZE * sizeof(u32), /* task's stack size, unit:byte */
                           MQTT_DEMO_TASK_PRIO, 0);

        tls_os_queue_create(&mqtt_demo_task_queue, MQTT_DEMO_QUEUE_SIZE);

        tls_netif_add_status_event(mqtt_demo_net_status);

        mqtt_demo_inited = TRUE;


}


void i2c_scanner(void){   //works well
    int nDevices = 0;
    wm_printf("I2C Address scanner started...\n\n");
    for (uint8_t addr = 1; addr < 127; addr += 0x1)
    {
        tls_i2c_write_byte(addr << 1, 1);
        tls_i2c_stop();

        if (!(tls_reg_read32(HR_I2C_CR_SR) & I2C_SR_NAK))
        {
            wm_printf("Device Found at address: 0x%.2x \n", addr);
            nDevices++;
            //char buff1 [ 30 ]={0};
            //snprintf(buff1, sizeof (buff1), "i2C bus: Found device at 0x%.2X", addr);
            //wm_printf(buff1);
            //mqtt_publish(&mqtt_demo_mqtt_broker, (const char *)MQTT_DEMO_RX_PUB_TOPIC, (const char *)buff1, sizeof (buff1), 0);
            //tls_os_time_delay(HZ * 0.1);

        }
        tls_os_time_delay(HZ * 0.0001);
    }
    if (nDevices == 0) {
        wm_printf("No I2C devices found.\n");
    } else {
        wm_printf("Scan complete...Found %d Devices\n",nDevices);

    }
}

void i2c_init() {
    wm_i2c_scl_config(PIN_SCL);
    wm_i2c_sda_config(PIN_SDA);
    tls_i2c_init(I2C_FREQ);
}


// Initialize LTR390 sensor
void ltr390_init() {
    // Configure the LTR390 sensor by writing to its registers
    // You can set the desired measurement rate, gain, and other parameters here
    // For example:
    // Set measurement rate to 100ms and gain to 3
    
    tls_i2c_write_byte(LTR390_ADDR << 1, 1);  // Write address
    tls_i2c_wait_ack();

    // Write LTR390_MEAS_RATE register with desired settings
    tls_i2c_write_byte(LTR390_MEAS_RATE, 0);  // Register address
    tls_i2c_wait_ack();
    tls_i2c_write_byte(RESOLUTION_18BIT_TIME100MS, 0);  // Set resolution and measurement rate
    tls_i2c_wait_ack();

    // Write LTR390_GAIN register with desired gain
    tls_i2c_write_byte(LTR390_GAIN, 0);  // Register address
    tls_i2c_wait_ack();
    tls_i2c_write_byte(GAIN_3, 0);  // Set gain
    tls_i2c_wait_ack();

    tls_i2c_stop();

    // Optionally, you can configure other settings as needed
}

// Read Lux data from LTR390
int readLux() {
    u8 data[3];  // The sensor returns a 24-bit light value (LSB, Middle, and MSB)

    // Start measurement
    
    tls_i2c_write_byte(LTR390_ADDR << 1, 1);  // Write address
    tls_i2c_wait_ack();
    tls_i2c_write_byte(LTR390_MAIN_CTRL, 0);  // Register address (optional if the sensor is already configured)
    tls_i2c_wait_ack();
    tls_i2c_write_byte(LTR390_MEAS_RATE, 0);  // Register address (optional if the sensor is already configured)
    tls_i2c_wait_ack();
    tls_i2c_write_byte(LTR390_GAIN, 0);  // Register address (optional if the sensor is already configured)
    tls_i2c_wait_ack();
    tls_i2c_write_byte(LTR390_MAIN_CTRL, 0);  // Register address (optional if the sensor is already configured)
    tls_i2c_wait_ack();
    tls_i2c_write_byte((LTR390_ADDR << 1) | 1, 1);  // Read address
    tls_i2c_wait_ack();

    for (int i = 0; i < 3; i++) {
        data[i] = tls_i2c_read_byte(i == 2 ? 0 : 1, i == 2 ? 1 : 0);
    }

    tls_i2c_stop();

    // Combine the received data to form a 24-bit integer (little-endian format)
    u32 luxValue = ((u32)data[0]) | ((u32)data[1] << 8) | ((u32)data[2] << 16);

    // Calculate lux value
    int lux = (int)((float)luxValue / (GAIN_3 * RESOLUTION_18BIT_TIME100MS));  // Adjust gain and resolution

    return lux;
}



// Initialize BME680 sensor
void bme680_init() {
        // You can set other configurations as needed
    u8 config = BME680_TEMP_OS_1X;

    // Initialize the sensor and configure it
    
    tls_i2c_write_byte(BME680_ADDR << 1, 1);  // Write address
    tls_i2c_wait_ack();

    tls_i2c_write_byte(BME680_REG_CTRL_MEAS, 0);  // Register address
    tls_i2c_wait_ack();
    tls_i2c_write_byte(config, 0);  // Set temperature oversampling
    tls_i2c_wait_ack();

    tls_i2c_stop();
}

// Read temperature from BME680 sensor
int readTemperature() {
   i2c_init();
    bme680_init();

    // Read temperature
    tls_i2c_write_byte(BME680_ADDR << 1, 1);  // Write address
    tls_i2c_wait_ack();

    tls_i2c_write_byte(BME680_REG_TEMP_XLSB, 1);  // Register address
    tls_i2c_wait_ack();
    tls_i2c_stop();

    tls_i2c_write_byte((BME680_ADDR << 1) | 1, 1);  // Read address
    tls_i2c_wait_ack();

    u8 tempXLSB = tls_i2c_read_byte(1, 1);
    u8 tempLSB = tls_i2c_read_byte(1, 1);
    u8 tempMSB = tls_i2c_read_byte(0, 1);

    tls_i2c_stop();

    // Combine the received data to form a 16-bit integer (little-endian format)
    int temperature = (int)((tempMSB << 12) | (tempLSB << 4) | (tempXLSB >> 4));

    // Convert to temperature in degrees Celsius
    float tempCelsius = (float)temperature / 100.0;

    return (int)(tempCelsius * 100);  // Return temperature in 0.01 degrees Celsius}
}
#endif

