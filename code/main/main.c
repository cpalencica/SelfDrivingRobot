#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include <math.h>
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include <stdio.h>
#include <string.h>
#include "driver/ledc.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "driver/adc.h"
#include "esp_adc_cal.h"

// Remote WASD
char wasd = '0';
char driverState = '0';
bool remote = false;
#define HOST "192.168.0.246"   
#define REMOTEPORT 8081

static const char *TAG_CLIENT = "udp receive";

bool stop = false;

// ADC Declarations
/* Variables for Thermistor and Temperature Conversion */
#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   10          //Multisampling


/* ADC variables */
static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel_sensor = ADC_CHANNEL_6;        // GPIO34
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;

uint32_t voltage;
float final_voltage = 0.0;

#define BLINK_GPIO 13

#define PI 3.14159265358979323846

#define HOST_IP_ADDR "192.168.0.167"
#define PORT 41234

static const char *TAG = "example";
static const char *payload = "ROBOTID 10";

// Wi-Fi configuration
#define EXAMPLE_ESP_WIFI_SSID      "TP-Link_EE49"
#define EXAMPLE_ESP_WIFI_PASS      "34383940"
#define EXAMPLE_ESP_MAXIMUM_RETRY  5

#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#define ESP_WIFI_SAE_MODE 0
#define EXAMPLE_H2E_IDENTIFIER ""

/* FreeRTOS event group to signal when we are connected */
static EventGroupHandle_t s_wifi_event_group;

/* Event group bits */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static int s_retry_num = 0;

typedef struct {
    int id;
    float x;
    float z;
    float theta;
    char status[10];
} RobotData;

// Timer and LEDC configuration for motor 1 (Left Motor)
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_HIGH_SPEED_MODE
#define LEDC_OUTPUT_IO_LEFT     (16) // PWM output GPIO for left motor
#define LEDC_CHANNEL_LEFT       LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_10_BIT // 10-bit resolution
#define LEDC_FREQUENCY          (50) // Frequency in Hz

// Timer and LEDC configuration for motor 2 (Right Motor)
#define LEDC_OUTPUT_IO_RIGHT    (14) // PWM output GPIO for right motor
#define LEDC_CHANNEL_RIGHT      LEDC_CHANNEL_1

// Motor direction control pins
#define MLA 19 // Left Motor A
#define MLB 18 // Left Motor B
#define MRA 32 // Right Motor A
#define MRB 15 // Right Motor B

// Forward declarations
void calculate_heading(double x_c, double z_c, double theta_c, double x_w, double z_w);
void actuate(float value);
void set_motor_speed(int motor_id, int duty_cycle);
void car_movement();

SemaphoreHandle_t xMutex; // Mutex for shared variables

// Global variables for heading calculations
double theta_d = 0.0;
double ehead = 0.0;

// Waypoint structure
typedef struct {
    double x;
    double z;
} Waypoint;

// Define your waypoints
Waypoint waypoints[] = {
    {742, -600},
    {-600,561},
    {734,609},
    {-728,-441},
};

// Index to keep track of the current waypoint
int current_waypoint_index = 0;

static void event_handler(void* arg, esp_event_base_t event_base,
                          int32_t event_id, void* event_data)
{

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "Retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"Connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

// Wi-Fi initialization function
void wifi_init_sta(void)
{
    // ... (existing Wi-Fi initialization code)
    // (No changes needed here)
    // ... (wifi_init_sta function continues)
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
            .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
            .sae_pwe_h2e = ESP_WIFI_SAE_MODE,
            .sae_h2e_identifier = EXAMPLE_H2E_IDENTIFIER,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    // Wait for connection
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    // Check connection result
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to AP SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}

// Initialize the LEDC PWM timers and channels for motors
static void example_ledc_init(void) {
    // Initialize the LEDC PWM timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Configure LEDC channel for left motor
    ledc_channel_config_t ledc_channel_left = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_LEFT,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO_LEFT,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_left));

    // Configure LEDC channel for right motor
    ledc_channel_config_t ledc_channel_right = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_RIGHT,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO_RIGHT,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_right));
}

// Initialize GPIO pins for motor direction control
void init_gpio() {
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << MLA) | (1ULL << MLB) | (1ULL << MRA) | (1ULL << MRB);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;

    gpio_config(&io_conf);
}

static void configure_ADC_sensor() {
    //Configure ADC
    if (unit == ADC_UNIT_1) {
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(channel_sensor, atten);
    } else {
        adc2_config_channel_atten((adc2_channel_t)channel_sensor, atten);
    }

    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
}

static void report_sensor() {
    //Continuously sample ADC1
    while (1) {
        uint32_t adc_reading = 0;
        //Multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++) {
            if (unit == ADC_UNIT_1) {
                adc_reading += adc1_get_raw((adc1_channel_t)channel_sensor);
            } else {
                int raw;
                adc2_get_raw((adc2_channel_t)channel_sensor, ADC_WIDTH_BIT_12, &raw);
                adc_reading += raw;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        adc_reading /= NO_OF_SAMPLES;

        /* Resistance Calculation */
        voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);  // get voltage reading
        final_voltage = voltage / 1000.0;

        // printf("voltage: %.2f\n", final_voltage);
        
    }
}

float linear(float voltage) {
    if ( 0.4 <= voltage && voltage < 1) return -150.0 * voltage + 210.0;
    else if ( 1 <= voltage && voltage <= 2.7 ) return -26.47 * voltage + 86.47;
    else return 0.0;

}

static void configure_led(void)
{
    gpio_reset_pin(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}

void detect_collision() {
    while(1) {
        float cm_dist = linear(final_voltage);
        // printf("distance: %.2f cm\n", cm_dist);

        if(cm_dist < 25.0 && cm_dist != 0) {
            gpio_set_level(BLINK_GPIO, 1);
            stop = true;
        }
        else {
            gpio_set_level(BLINK_GPIO, 0);
            stop = false;
        }

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

// Function to set the left motor direction (forward)
void control_left() {
    gpio_set_level(MLA, 0);
    gpio_set_level(MLB, 1);
}

// Function to set the right motor direction (forward)
void control_right() {
    gpio_set_level(MRA, 0);
    gpio_set_level(MRB, 1);
}

// Function to set motor speed (duty cycle)
void set_motor_speed(int motor_id, int duty_cycle) {
    if (motor_id == LEDC_CHANNEL_LEFT) {
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_LEFT, duty_cycle);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_LEFT);
    } else if (motor_id == LEDC_CHANNEL_RIGHT) {
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_RIGHT, duty_cycle);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_RIGHT);
    }
}

// Calculate desired heading and heading error
void calculate_heading(double x_c, double z_c, double theta_c, double x_w, double z_w) {
    // Calculate the differences in coordinates
    double delta_x = x_w - x_c;
    double delta_z = z_w - z_c;

    // Calculate the standard angle (theta_std) in degrees
    double theta_std = atan2(delta_z, delta_x) * (180.0 / PI);

    // Adjust theta_std to the robot's coordinate system to get theta_d
    double thetad = fmod(270.0 - theta_std + 360.0, 360.0);

    // Calculate the heading error (ehead)
    double Ehead = fmod(thetad - theta_c + 360.0, 360.0);
    if (Ehead > 180.0) {
        Ehead -= 360.0;
    }

    // Protect shared variables with mutex
    xSemaphoreTake(xMutex, portMAX_DELAY);
    theta_d = thetad;
    ehead = Ehead;
    xSemaphoreGive(xMutex);
}

// Parse incoming robot data
int parseRobotData(const char *input, RobotData *data) {
    // Parse the input string and populate the RobotData structure
    int parsed = sscanf(input, "%d,%f,%f,%f,%9s", &data->id, &data->x, &data->z, &data->theta, data->status);

    // Check if all fields were successfully parsed
    if (parsed == 5) {
        return 1; // Success
    } else {
        return 0; // Parsing failed
    }
}

// UDP client task to receive data
static void udp_client_task(void *pvParameters)
{
    char rx_buffer[128];
    char host_ip[] = HOST_IP_ADDR;
    int addr_family = AF_INET;
    int ip_protocol = IPPROTO_IP;

    while (1) {

        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(PORT);

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }

        // Set timeout
        struct timeval timeout;
        timeout.tv_sec = 10;
        timeout.tv_usec = 0;
        setsockopt (sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof timeout);

        while (1) {

            int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            if (err < 0) {
                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                break;
            }

            struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
            socklen_t socklen = sizeof(source_addr);
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

            // Error occurred during receiving
            if (len < 0) {
                ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
                break;
            }
            // Data received
            else {
                rx_buffer[len] = 0; // Null-terminate the received data

                RobotData data;

                if (parseRobotData(rx_buffer, &data)) {
                    // Calculate distance to current waypoint
                    double distance = sqrt(pow(data.x - waypoints[current_waypoint_index].x, 2) +
                                           pow(data.z - waypoints[current_waypoint_index].z, 2));
                    //printf("Distance to waypoint: %.2f\n", distance);

                    // Check if within +/-10 units
                    if (distance <= 200.0) {
                        current_waypoint_index++;
                        //printf("Reached waypoint\n");
                        if (current_waypoint_index >= sizeof(waypoints)/sizeof(waypoints[0])) {
                            // Reached the final waypoint
                            current_waypoint_index = 0;
                        }
                    }

                    // Now calculate heading to current waypoint
                    calculate_heading((double) data.x, (double) data.z, (double) data.theta,
                                      waypoints[current_waypoint_index].x, waypoints[current_waypoint_index].z);
                } else {
                    printf("Failed to parse robot data.\n");
                }

                if (strncmp(rx_buffer, "OK: ", 4) == 0) {
                    ESP_LOGI(TAG, "Received expected message, reconnecting");
                    break;
                }
            }

            vTaskDelay(pdMS_TO_TICKS(100)); // Wait for 100 milliseconds
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}

void udp_receive_wasd(void *pvParameters) {
    char rx_buffer[128];
    char addr_str[128];
    int addr_family = AF_INET;
    int ip_protocol = IPPROTO_IP;

    // Configure the socket address for the server
    struct sockaddr_in serverAddr;
    serverAddr.sin_addr.s_addr = INADDR_ANY;  // Listen on any IP address
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(REMOTEPORT);  // Listen on the same port used for sending data

    // Create socket
    int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
    if (sock < 0) {
        ESP_LOGE(TAG_CLIENT, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG_CLIENT, "Socket created");

    // Bind socket to the specified port
    int err = bind(sock, (struct sockaddr *)&serverAddr, sizeof(serverAddr));
    if (err < 0) {
        ESP_LOGE(TAG_CLIENT, "Socket unable to bind: errno %d", errno);
        close(sock);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG_CLIENT, "Socket bound, port %d", REMOTEPORT);

    while (1) {
        ESP_LOGI(TAG_CLIENT, "Waiting for data...");
        

        struct sockaddr_in sourceAddr;
        socklen_t socklen = sizeof(sourceAddr);

        // Receive data from any client
        int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&sourceAddr, &socklen);

        // Error occurred during receiving
        if (len < 0) {
            ESP_LOGE(TAG_CLIENT, "recvfrom failed: errno %d", errno);
            break;
            wasd = '0';

        }
        // Data received
        else {
            rx_buffer[len] = 0; // Null-terminate the received data
            inet_ntoa_r(sourceAddr.sin_addr, addr_str, sizeof(addr_str) - 1);
            ESP_LOGI(TAG_CLIENT, "Received %d bytes from %s: %s", len, addr_str, rx_buffer);
            ESP_LOGI(TAG_CLIENT, "%s", rx_buffer);

            printf("Received: %s\n", rx_buffer);

            wasd = rx_buffer[0];    // Use first element as key input

            if (wasd == 'q') {
                remote = !remote;
                printf("Remote control toggle\n");
            }

            driverState = '1';
            
        }
    }

    // Cleanup and close the socket
    if (sock != -1) {
        // ESP_LOGE(TAG_CLIENT, "Shutting down server socket...");
        shutdown(sock, 0);
        close(sock);
    }
    vTaskDelete(NULL);
}

void move_forward() {
    printf("FW\n");
    gpio_set_level(MLA, 0);
    gpio_set_level(MLB, 1); 

    gpio_set_level(MRA, 0);
    gpio_set_level(MRB, 1);
}

void move_backward() {
    printf("BW\n");

    gpio_set_level(MLA, 1);
    gpio_set_level(MLB, 0); 

    gpio_set_level(MRA, 1);
    gpio_set_level(MRB, 0);
}

void stop_movement() {
    printf("STOP\n");
    gpio_set_level(MLA, 0);
    gpio_set_level(MLB, 0); 

    gpio_set_level(MRA, 0);
    gpio_set_level(MRB, 0);
}

void move_right() {
    printf("R\n");
    gpio_set_level(MLA, 0);
    gpio_set_level(MLB, 1); 

    gpio_set_level(MRA, 0);
    gpio_set_level(MRB, 0);
}

void move_left() {
    printf("L\n");
    gpio_set_level(MLA, 0);
    gpio_set_level(MLB, 0); 

    gpio_set_level(MRA, 0);
    gpio_set_level(MRB, 1);
}

void car_movement() {
    int default_duty = 1023; // or whatever speed you want
    if (driverState == '1') {
        if (wasd == 'w') {
            move_forward();
            set_motor_speed(LEDC_CHANNEL_LEFT, default_duty);
            set_motor_speed(LEDC_CHANNEL_RIGHT, default_duty);
        }
        else if (wasd == 'a') {
            move_left();
            set_motor_speed(LEDC_CHANNEL_LEFT, default_duty);
            set_motor_speed(LEDC_CHANNEL_RIGHT, default_duty);
        }
        else if (wasd == 's') {
            move_backward();
            set_motor_speed(LEDC_CHANNEL_LEFT, default_duty);
            set_motor_speed(LEDC_CHANNEL_RIGHT, default_duty);
        }
        else if (wasd == 'd') {
            move_right();
            set_motor_speed(LEDC_CHANNEL_LEFT, default_duty);
            set_motor_speed(LEDC_CHANNEL_RIGHT, default_duty);
        }
        driverState = '0';
    } else {
        stop_movement();
        set_motor_speed(LEDC_CHANNEL_LEFT, 0);
        set_motor_speed(LEDC_CHANNEL_RIGHT, 0);
    }
}


// PID parameters
float Kp = 0.75;  // Proportional gain
float Ki = 0.0;   // Integral gain
float Kd = 0.0;   // Derivative gain

float output = 0.0; // PID output

// Variables for PID computation
float previous_error = 0.0;
float integral = 0.0;
float dt = 0.1; // Time step in seconds (100ms)

// Actuate function to adjust motor speeds
void actuate(float value) {
    // Constants
    const int MAX_DUTY = 1023;      // Maximum duty cycle for 10-bit resolution
    const int BASE_DUTY = 1023;      // Base duty cycle for normal speed
    const float K = 27.0;           // Scaling factor for adjustment (adjust as needed)

    // Variables for motor duty cycles
    int left_duty = BASE_DUTY;
    int right_duty = BASE_DUTY;

    // Calculate adjustment based on PID output
    int adjustment = (int)(K * fabs(value));

    // Adjust motor speeds based on error
    if (value > 0) {
        // Positive error: decrease right motor speed
        right_duty = BASE_DUTY - adjustment;
    } else if (value < 0) {
        // Negative error: decrease left motor speed
        left_duty = BASE_DUTY - adjustment;
    }

    // Ensure duty cycles are within valid range
    if (left_duty < 0) left_duty = 0;
    if (right_duty < 0) right_duty = 0;
    if (left_duty > MAX_DUTY) left_duty = MAX_DUTY;
    if (right_duty > MAX_DUTY) right_duty = MAX_DUTY;

    // Update PWM duty cycles for the motors
    if (stop) {
        set_motor_speed(LEDC_CHANNEL_LEFT, 0);
        set_motor_speed(LEDC_CHANNEL_RIGHT, 0);
    }
    else {
        if (!remote) {
            set_motor_speed(LEDC_CHANNEL_LEFT, right_duty);
            set_motor_speed(LEDC_CHANNEL_RIGHT, left_duty);
        }
        else {
            car_movement();
        }
    }
    // Print debug information
    printf("Desired angle: %.2f degrees \n", theta_d);
    printf("Heading error: %.2f degrees \n", ehead);
    printf("Control signal (value): %.2f\n", value);
    printf("Left motor duty cycle: %d\n", left_duty);
    printf("Right motor duty cycle: %d\n\n", right_duty);
}

// PID task to compute control output
static void pid_task(void *parameters) {
    while (1) {
        float error;

        // Protect access to shared variable ehead
        xSemaphoreTake(xMutex, portMAX_DELAY);
        error = ehead;
        xSemaphoreGive(xMutex);

        // PID calculations
        integral += error * dt;
        float derivative = (error - previous_error) / dt;
        output = Kp * error + Ki * integral + Kd * derivative;
        previous_error = error;

        // Actuate system with the PID output
        actuate(output);

        // Wait for the next cycle
        vTaskDelay(pdMS_TO_TICKS((int)(dt * 1000))); // Convert seconds to ticks
    }
}

void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize mutex
    xMutex = xSemaphoreCreateMutex();
    if (xMutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return;
    }

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();
    init_gpio();
    configure_ADC_sensor();
    configure_led();
    example_ledc_init();

    // Set motor directions once
    control_left();
    control_right();

    // Start tasks
    xTaskCreate(udp_client_task, "udp_client", 4096, NULL, 5, NULL);
    xTaskCreate(pid_task, "pid_task", 4096, NULL, 5, NULL);
    xTaskCreate(report_sensor, "detect_collision", 4096, NULL, 5, NULL);
    xTaskCreate(detect_collision, "detect_collision", 4096, NULL, 5, NULL);
    xTaskCreate(udp_receive_wasd, "udp_receive_wasd", 4096, NULL, 5, NULL);

}