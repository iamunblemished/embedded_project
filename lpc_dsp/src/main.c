/* src/main.c (NXP LPC55S69 Compute Brain - Master File) */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* USB/Simulink (Console Input) - Receives the simulated motor data */
#define SIMULINK_UART DT_CHOSEN(zephyr_shell_uart)

/* ESP32 Telemetry (Verified P18 Header Output) - Sends JSON to ESP32 */
#define ESP32_UART DT_NODELABEL(flexcomm2) 

/* 1024 samples is roughly 1 second of data at a 1024Hz sample rate, 
 * providing the 1Hz resolution necessary to spot tight sidebands. */
#define PAYLOAD_SIZE 1024
#define SAMPLE_RATE 1024

static const struct device *const simulink_dev = DEVICE_DT_GET(SIMULINK_UART);
static const struct device *const esp32_dev = DEVICE_DT_GET(ESP32_UART);

static uint8_t rx_buf[PAYLOAD_SIZE];
static volatile int rx_ptr = 0;
static volatile bool data_ready = false;

/* Global Lookup Table for the Windowing Function */
float hann_window[PAYLOAD_SIZE];

/* Pre-compute the Hann Window at boot to save CPU cycles during runtime.
 * Windowing prevents spectral leakage from the massive 50Hz fundamental 
 * from bleeding over and hiding the microscopic fault sidebands. */
void init_hann_window() {
    for (int i = 0; i < PAYLOAD_SIZE; i++) {
        hann_window[i] = 0.5f * (1.0f - cosf((2.0f * 3.14159265f * i) / (PAYLOAD_SIZE - 1)));
    }
}

/* The Goertzel Algorithm 
 * Computes the magnitude of a single target frequency. Unlike an FFT which computes 
 * the entire spectrum, this is a highly targeted "magnifying glass" that saves memory. */
float goertzel_mag(float* windowed_data, int num_samples, float target_freq, int sample_rate) {
    /* Calculate the frequency bin index */
    int k = (int)(0.5 + ((float)num_samples * target_freq) / sample_rate);
    float omega = (2.0f * 3.14159265f * k) / num_samples;
    float coeff = 2.0f * cosf(omega);
    float q0 = 0, q1 = 0, q2 = 0;

    /* Process the time-domain samples */
    for(int i = 0; i < num_samples; i++) {
        q0 = coeff * q1 - q2 + windowed_data[i];
        q2 = q1;
        q1 = q0;
    }
    /* Return the computed spectral magnitude for this specific frequency */
    return sqrtf((q1 * q1) + (q2 * q2) - (q1 * q2 * coeff));
}

/* Hardware UART Interrupt: Catches Simulink Data */
void uart_cb(const struct device *dev, void *user_data)
{
    if (!uart_irq_update(dev)) return;

    if (uart_irq_rx_ready(dev)) {
        if (!data_ready) { 
            /* Pull bytes from hardware FIFO into our buffer */
            int recv_len = uart_fifo_read(dev, &rx_buf[rx_ptr], PAYLOAD_SIZE - rx_ptr);
            rx_ptr += recv_len;

            /* When the bucket is full (1024 bytes), signal the main thread and go deaf.
             * Disabling the RX interrupt ensures our DSP loop isn't interrupted and 
             * our data array isn't overwritten while we are doing the math. */
            if (rx_ptr >= PAYLOAD_SIZE) {
                data_ready = true; 
                uart_irq_rx_disable(dev); 
            }
        } else {
            /* Flush the hardware FIFO if Simulink keeps firing while we process */
            uint8_t dummy;
            while(uart_fifo_read(dev, &dummy, 1));
        }
    }
}

int main(void)
{
    k_sleep(K_MSEC(1000));
    init_hann_window();

    if (!device_is_ready(simulink_dev) || !device_is_ready(esp32_dev)) {
        printk("FATAL: UART devices not found!\n");
        return -1;
    }
    
    uart_irq_callback_set(simulink_dev, uart_cb);
    uart_irq_rx_enable(simulink_dev);

    while (1) { 
        if (data_ready) {
            static float windowed_data[PAYLOAD_SIZE];

            /* 1. Remove DC Offset and Apply Window 
             * Simulink sends uint8_t (0-255). Subtracting 127 centers the AC wave at 0.
             * The Hann window is then applied to shape the edges. */
            for(int i = 0; i < PAYLOAD_SIZE; i++) {
                float centered_sample = (float)rx_buf[i] - 127.0f;
                windowed_data[i] = centered_sample * hann_window[i];
            }

            /* 2. Math Processing via Goertzel
             * Extract the carrier wave (50Hz) and the theoretical fault sideband (75Hz).
             * (In a dynamic setup, the 75Hz target would be calculated based on slip). */
            float mag_50hz = goertzel_mag(windowed_data, PAYLOAD_SIZE, 50.0f, SAMPLE_RATE);
            float mag_75hz = goertzel_mag(windowed_data, PAYLOAD_SIZE, 75.0f, SAMPLE_RATE); 

            /* 3. Normalization (Relative dB)
             * Calculates the decibel distance between the sideband and the fundamental.
             * This prevents load variations or grid supply drops from masking the result. */
            float db_ratio = -100.0f; 
            if (mag_50hz > 0.01f && mag_75hz > 0.001f) {
                db_ratio = 20.0f * log10f(mag_75hz / mag_50hz);
            }

            /* 4. Format the JSON Payload */
            char payload[128];
            snprintf(payload, sizeof(payload), "{\"system\": \"mcsa\", \"50Hz_mag\": %.2f, \"severity_db\": %.2f}\n", mag_50hz, db_ratio);

            /* 5. Transmit Telemetry Directly to ESP32 */
            for (int i = 0; i < strlen(payload); i++) {
                uart_poll_out(esp32_dev, payload[i]);
            }

            /* 6. Send acknowledgment back to Simulink
             * This tells the Simulink Digital Twin to send the next block of 1024 bytes. */
            char ack_msg[] = "ACK\n";
            for (int i = 0; i < strlen(ack_msg); i++) {
                uart_poll_out(simulink_dev, ack_msg[i]);
            }

            /* 7. Reset the buffer and resume catching data */
            rx_ptr = 0;
            data_ready = false;
            uart_irq_rx_enable(simulink_dev);
        }

        k_sleep(K_MSEC(1)); 
    }
    return 0;
}
