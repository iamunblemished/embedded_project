/* src/main.c (ESP32 Telemetry Coprocessor - Master File) */
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/mqtt.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/net/net_if.h>
#include <string.h>
#include <stdio.h>

#define MSG_SIZE 128
#define MQTT_CLIENTID "esp32_mcsa_node"
#define MQTT_TOPIC "mcsa/telemetry/node_1"

#define BROKER_HOSTNAME "broker.emqx.io"
#define BROKER_PORT "1883"

/* Get the device binding for UART2, which connects to the NXP board */
const struct device *uart_dev = DEVICE_DT_GET(DT_NODELABEL(uart2));

/* Message queue: Essential for RTOS. Safely passes the JSON payload from the 
 * high-priority hardware interrupt (ISR) to the normal-priority main thread. */
K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 10, 4);

static char rx_buf[MSG_SIZE];
static int rx_ptr = 0;

/* Buffers required by the Zephyr MQTT client to construct and parse packets */
static uint8_t rx_buffer[256];
static uint8_t tx_buffer[256];
static struct mqtt_client client_ctx;
static struct sockaddr_in broker;

/* --- 1. The "Silent" UART Interrupt --- */
void uart_cb(const struct device *dev, void *user_data) {
    if (!uart_irq_update(dev)) {
        return;
    }

    if (uart_irq_rx_ready(dev)) {
        uint8_t c;
        /* Read characters from the hardware FIFO until empty */
        while (uart_fifo_read(dev, &c, 1) == 1) {

            /* Build the string until the NXP sends a newline character (\n) */
            if (c == '\n') {
                if (rx_ptr > 0) {
                    rx_buf[rx_ptr] = '\0'; // Null-terminate the C string
                    /* Push the completed JSON string into the queue for the main thread */
                    k_msgq_put(&uart_msgq, rx_buf, K_NO_WAIT);
                    rx_ptr = 0; // Reset pointer for the next incoming message
                }
            } else if (c != '\r' && rx_ptr < MSG_SIZE - 1) {
                rx_buf[rx_ptr++] = (char)c;
            }

            /* DELIBERATELY SILENT: No printk here. Printing inside an ISR takes 
             * too long and will cause the CPU to drop incoming UART bytes. */
        }
    }
}

/* --- 2. The Wi-Fi Lock --- */
static void connect_wifi_and_wait(void) {
    struct net_if *iface = net_if_get_default();
    struct wifi_connect_req_params wifi_params = {0};

    wifi_params.ssid = "SSID";
    wifi_params.ssid_length = strlen("SSID");
    wifi_params.psk = "PASSWORD";
    wifi_params.psk_length = strlen("PASSWORD");
    wifi_params.channel = WIFI_CHANNEL_ANY;
    wifi_params.security = WIFI_SECURITY_TYPE_PSK;

    printk("\nConnecting to Wi-Fi: %s...\n", wifi_params.ssid);
    net_mgmt(NET_REQUEST_WIFI_CONNECT, iface, &wifi_params, sizeof(struct wifi_connect_req_params));

    printk("Waiting for router to assign IP address");
    /* Block execution until the router provisions a valid IPv4 address via DHCP */
    while (1) {
        if (net_if_is_up(iface)) {
            if (net_if_ipv4_get_global_addr(iface, NET_ADDR_PREFERRED) != NULL) {
                break;
            }
        }
        printk(".");
        k_sleep(K_MSEC(1000));
    }
    printk("\nIP Address secured! Network ready.\n");
}

/* --- 3. DNS & MQTT Broker Initialization --- */
void broker_init(void) {
    mqtt_client_init(&client_ctx);

    broker.sin_family = AF_INET;
    broker.sin_port = htons(1883); // Standard unencrypted MQTT port

    /* Dynamically resolve the EMQX URL to an IP address */
    struct zsock_addrinfo hints = {
        .ai_family = AF_INET,
        .ai_socktype = SOCK_STREAM,
    };
    struct zsock_addrinfo *res;

    printk("Resolving %s...\n", BROKER_HOSTNAME);
    int err = zsock_getaddrinfo(BROKER_HOSTNAME, BROKER_PORT, &hints, &res);
    
    if (err != 0) {
        printk("DNS Lookup failed! Error: %d\n", err);
        /* Hardcoded fallback IP just in case the DNS server fails */
        zsock_inet_pton(AF_INET, "44.232.241.40", &broker.sin_addr);
    } else {
        struct sockaddr_in *addr4 = (struct sockaddr_in *)res->ai_addr;
        broker.sin_addr.s_addr = addr4->sin_addr.s_addr;
        zsock_freeaddrinfo(res);
        printk("DNS Lookup Successful!\n");
    }

    /* Configure the MQTT client structure */
    client_ctx.broker = &broker;
    client_ctx.client_id.utf8 = (uint8_t *)MQTT_CLIENTID;
    client_ctx.client_id.size = strlen(MQTT_CLIENTID);
    client_ctx.password = NULL;
    client_ctx.user_name = NULL;
    client_ctx.protocol_version = MQTT_VERSION_3_1_1;
    
    client_ctx.rx_buf = rx_buffer;
    client_ctx.rx_buf_size = sizeof(rx_buffer);
    client_ctx.tx_buf = tx_buffer;
    client_ctx.tx_buf_size = sizeof(tx_buffer);
    client_ctx.transport.type = MQTT_TRANSPORT_NON_SECURE;
}

/* --- 4. Main Event Loop --- */
int main(void) {
    if (!device_is_ready(uart_dev)) {
        printk("UART device not ready\n");
        return -1;
    }

    /* Configure the interrupt handler, but DO NOT enable it yet */
    uart_irq_callback_set(uart_dev, uart_cb);

    /* Phase 1: Connect to Wi-Fi safely */
    connect_wifi_and_wait();

    /* Phase 2: Give the router 3 seconds to finalize internet routing */
    printk("Letting network settle...\n");
    k_sleep(K_SECONDS(3)); 

    /* Phase 3: Resolve DNS and Connect to the Cloud */
    broker_init();
    printk("Connecting to MQTT Broker...\n");
    int rc = mqtt_connect(&client_ctx);
    if (rc != 0) {
        printk("MQTT connect failed: %d\n", rc);
        printk("NOTE: If this fails with -116, Port 1883 is blocked by your firewall.\n");
        printk("Connect ESP32 to a cellular hotspot to bypass.\n");
        return -1;
    }
    printk("MQTT Connected Successfully!\n");

    /* Phase 4: Open the Floodgates! */
    /* Only start listening to the NXP Compute Brain AFTER the internet is locked in */
    uart_irq_rx_enable(uart_dev);
    printk("Listening for MCSA Telemetry...\n");

    char mqtt_payload[MSG_SIZE];
    struct mqtt_publish_param param;

    /* Phase 5: Continuous Publishing Loop */
    while (1) {
        /* These keep the MQTT connection alive and process incoming acks */
        mqtt_input(&client_ctx);
        mqtt_live(&client_ctx);

        /* Wait up to 50ms for a new JSON payload to arrive from the UART ISR */
        if (k_msgq_get(&uart_msgq, &mqtt_payload, K_MSEC(50)) == 0) {

            /* Map the JSON string into the MQTT publish parameters */
            param.message.payload.data = (uint8_t *)mqtt_payload;
            param.message.payload.len = strlen(mqtt_payload);
            param.message.topic.topic.utf8 = (uint8_t *)MQTT_TOPIC;
            param.message.topic.topic.size = strlen(MQTT_TOPIC);
            param.message.topic.qos = MQTT_QOS_0_AT_MOST_ONCE;
            param.retain_flag = 0;
            param.message_id = 0;

            /* Push to the cloud */
            mqtt_publish(&client_ctx, &param);

            /* Print the success message locally so you know it worked */
            printk("[PUBLISHED] %s\n", mqtt_payload);
        }

        k_sleep(K_MSEC(10)); // Yield CPU to allow other Zephyr threads to run
    }
    return 0;
}
