#include "pico/stdlib.h"            // Biblioteca da Raspberry Pi Pico para funções padrão (GPIO, temporização, etc.)
#include "pico/cyw43_arch.h"        // Biblioteca para arquitetura Wi-Fi da Pico com CYW43
#include "pico/unique_id.h"         // Biblioteca com recursos para trabalhar com os pinos GPIO do Raspberry Pi Pico

#include "hardware/gpio.h"          // Biblioteca de hardware de GPIO
#include "hardware/irq.h"           // Biblioteca de hardware de interrupções
#include "hardware/adc.h"           // Biblioteca de hardware para conversão ADC

#include "lwip/apps/mqtt.h"         // Biblioteca LWIP MQTT -  fornece funções e recursos para conexão MQTT
#include "lwip/apps/mqtt_priv.h"    // Biblioteca que fornece funções e recursos para Geração de Conexões
#include "lwip/dns.h"               // Biblioteca que fornece funções e recursos suporte DNS:
#include "lwip/altcp_tls.h"         // Biblioteca que fornece funções e recursos para conexões seguras usando TLS:

#include <stdio.h>
#include "pico/bootrom.h"
#include "lwip/tcp.h"
#include "pico/cyw43_arch.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/i2c.h"
#include "lib/ssd1306.h"
#include "hardware/pio.h"
#include "ws2812.pio.h"
#include "hardware/timer.h" // Inclui a biblioteca para gerenciamento de temporizadores de hardware.

#include <time.h>

#include "pico/bootrom.h"

#define WIFI_SSID "Willian(gdv)18am"                  // Substitua pelo nome da sua rede Wi-Fi
#define WIFI_PASSWORD "c4iqu3246"      // Substitua pela senha da sua rede Wi-Fi
#define MQTT_SERVER "10.0.1.101"                // Substitua pelo endereço do host - broket MQTT: Ex: 192.168.1.107
#define MQTT_USERNAME "admin"     // Substitua pelo nome da host MQTT - Username
#define MQTT_PASSWORD "admin"     // Substitua pelo Password da host MQTT - credencial de acesso - caso exista


// Pinos dos componentes
#define WS2812_PIN 7 // Pino do WS2812
#define BUTTON_B_PIN 6 // Pino do botão B
#define SERVO_PIN 8  // Definição do pino PWM para o servo
#define DHT_PIN 9 // Pino do DHT11
#define BUZZER_PIN 21

#define PWM_FREQ 50   // Frequência de 50Hz (Período de 20ms)
#define PWM_WRAP 20000 // Contagem total do PWM (20ms em microsegundos

#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15 
#define address 0x3C

ssd1306_t ssd;

/*
    * Variável para armazenar o estado dos dispositivos
    * do sistema de automação residencial.
*/
typedef struct {
    bool gate;
    bool living_room_light;
    bool kitchen_light;
    bool bedroom_light;
} THINGS;

enum THINGS_MATRIX_5X5_POSITION {
    LIVING_ROOM_LIGHT = 14,
    KITCHEN_LIGHT = 12,
    BEDROOM_LIGHT = 10,
};

THINGS things = {false, false, false, false};

typedef struct {
    float temperature;
    float humidity;
} DHT11;

DHT11 dht11 = {0, 0};

bool read_sensor_dht11 = false; // Variável para controlar a leitura do sensor DHT11

#define LED_COUNT 25

struct pixel_t {
  uint8_t R, G, B; // Três valores de 8-bits compõem um pixel.
};
typedef struct pixel_t pixel_t;
typedef pixel_t npLED_t; // Mudança de nome de "struct pixel_t" para "npLED_t" por clareza.

PIO np_pio;
uint sm;
npLED_t leds[LED_COUNT];

// Definição da escala de temperatura
#ifndef TEMPERATURE_UNITS
#define TEMPERATURE_UNITS 'C' // Set to 'F' for Fahrenheit
#endif

#ifndef MQTT_SERVER
#error Need to define MQTT_SERVER
#endif

// This file includes your client certificate for client server authentication
#ifdef MQTT_CERT_INC
#include MQTT_CERT_INC
#endif

#ifndef MQTT_TOPIC_LEN
#define MQTT_TOPIC_LEN 100
#endif

//Dados do cliente MQTT
typedef struct {
    mqtt_client_t* mqtt_client_inst;
    struct mqtt_connect_client_info_t mqtt_client_info;
    char data[MQTT_OUTPUT_RINGBUF_SIZE];
    char topic[MQTT_TOPIC_LEN];
    uint32_t len;
    ip_addr_t mqtt_server_address;
    bool connect_done;
    int subscribe_count;
    bool stop_client;
} MQTT_CLIENT_DATA_T;

#ifndef DEBUG_printf
#ifndef NDEBUG
#define DEBUG_printf printf
#else
#define DEBUG_printf(...)
#endif
#endif

#ifndef INFO_printf
#define INFO_printf printf
#endif

#ifndef ERROR_printf
#define ERROR_printf printf
#endif

// Temporização da coleta de temperatura - how often to measure our temperature
#define TEMP_WORKER_TIME_S 10

// Manter o programa ativo - keep alive in seconds
#define MQTT_KEEP_ALIVE_S 60

// QoS - mqtt_subscribe
// At most once (QoS 0)
// At least once (QoS 1)
// Exactly once (QoS 2)
#define MQTT_SUBSCRIBE_QOS 1
#define MQTT_PUBLISH_QOS 1
#define MQTT_PUBLISH_RETAIN 0

// Tópico usado para: last will and testament
#define MQTT_WILL_TOPIC "/online"
#define MQTT_WILL_MSG "0"
#define MQTT_WILL_QOS 1

#ifndef MQTT_DEVICE_NAME
#define MQTT_DEVICE_NAME "pico"
#endif

// Definir como 1 para adicionar o nome do cliente aos tópicos, para suportar vários dispositivos que utilizam o mesmo servidor
#ifndef MQTT_UNIQUE_TOPIC
#define MQTT_UNIQUE_TOPIC 0
#endif

/* References for this implementation:
 * raspberry-pi-pico-c-sdk.pdf, Section '4.1.1. hardware_adc'
 * pico-examples/adc/adc_console/adc_console.c */

//Leitura de temperatura do microcotrolador
static float read_onboard_temperature(const char unit);

// Requisição para publicar
static void pub_request_cb(__unused void *arg, err_t err);

// Topico MQTT
static const char *full_topic(MQTT_CLIENT_DATA_T *state, const char *name);

// Controle do LED 
static void control_led(MQTT_CLIENT_DATA_T *state, bool on);

// Publicar temperatura
static void publish_temperature(MQTT_CLIENT_DATA_T *state);

// Requisição de Assinatura - subscribe
static void sub_request_cb(void *arg, err_t err);

// Requisição para encerrar a assinatura
static void unsub_request_cb(void *arg, err_t err);

// Tópicos de assinatura
static void sub_unsub_topics(MQTT_CLIENT_DATA_T* state, bool sub);

// Dados de entrada MQTT
static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags);

// Dados de entrada publicados
static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len);

// Publicar temperatura
static void temperature_worker_fn(async_context_t *context, async_at_time_worker_t *worker);
static async_at_time_worker_t temperature_worker = { .do_work = temperature_worker_fn };

// Conexão MQTT
static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status);

// Inicializar o cliente MQTT
static void start_client(MQTT_CLIENT_DATA_T *state);

// Call back com o resultado do DNS
static void dns_found(const char *hostname, const ip_addr_t *ipaddr, void *arg);

void gpio_irq_handler(uint gpio, uint32_t events)
{
    volatile static uint32_t last_time = 0;
    volatile uint32_t current_time = to_ms_since_boot(get_absolute_time()); 

    if (current_time - last_time < 400) { // Debounce de 400ms
        return;
    }

    last_time = current_time;

    if (gpio == BUTTON_B_PIN) {
        reset_usb_boot(0, 0);
        return;
    }
}

void gpio_setup() {
    gpio_init(BUTTON_B_PIN);
    gpio_set_dir(BUTTON_B_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_B_PIN);
    gpio_set_irq_enabled_with_callback(
        BUTTON_B_PIN, 
        GPIO_IRQ_EDGE_FALL,
        true, 
        &gpio_irq_handler
    );
}

void i2c_setup();
void show_ip_address();
void pwm_init_buzzer(uint pin);
void setup_pwm(uint pin);
void npInit(uint pin);
void npClear();
void npSetLED(const uint index, const uint8_t r, const uint8_t g, const uint8_t b);
void npWrite();
void turn_on_light(bool *light, uint index);
void turn_off_light(bool *light, uint index);
void gate(bool state);
void emit_alert();
void set_servo_position(uint pin, uint pulse_width);
void play_tone(uint pin, uint frequency, uint duration_ms);
int read_dht11(DHT11 *dht);
bool repeating_timer_callback(struct repeating_timer *t);
static void simulate_dht11();

int main(void) {

    // Inicializa todos os tipos de bibliotecas stdio padrão presentes que estão ligados ao binário.
    stdio_init_all();
    INFO_printf("mqtt client starting\n");

    gpio_setup(); // Configura o GPIO para o botão B

    i2c_setup();
    pwm_init_buzzer(BUZZER_PIN);
    setup_pwm(SERVO_PIN);

    npInit(WS2812_PIN);
    npClear();

    ssd1306_init(&ssd, WIDTH, HEIGHT, false, address, I2C_PORT);
    ssd1306_config(&ssd);
    ssd1306_send_data(&ssd);
    ssd1306_fill(&ssd, false);
    ssd1306_send_data(&ssd);

    gpio_init(DHT_PIN); // Inicializa o pino do DHT11
    sleep_ms(2000); // Espera para o sensor DHT11 se estabilizar

    // Inicializa o conversor ADC
    // adc_init();
    // adc_set_temp_sensor_enabled(true);
    // adc_select_input(4);

    // Cria registro com os dados do cliente
    static MQTT_CLIENT_DATA_T state;

    // Inicializa a arquitetura do cyw43
    if (cyw43_arch_init()) {
        panic("Failed to inizialize CYW43");
    }

    // Usa identificador único da placa
    char unique_id_buf[5];
    pico_get_unique_board_id_string(unique_id_buf, sizeof(unique_id_buf));
    for(int i=0; i < sizeof(unique_id_buf) - 1; i++) {
        unique_id_buf[i] = tolower(unique_id_buf[i]);
    }

    // Gera nome único, Ex: pico1234
    char client_id_buf[sizeof(MQTT_DEVICE_NAME) + sizeof(unique_id_buf) - 1];
    memcpy(&client_id_buf[0], MQTT_DEVICE_NAME, sizeof(MQTT_DEVICE_NAME) - 1);
    memcpy(&client_id_buf[sizeof(MQTT_DEVICE_NAME) - 1], unique_id_buf, sizeof(unique_id_buf) - 1);
    client_id_buf[sizeof(client_id_buf) - 1] = 0;
    INFO_printf("Device name %s\n", client_id_buf);

    state.mqtt_client_info.client_id = client_id_buf;
    state.mqtt_client_info.keep_alive = MQTT_KEEP_ALIVE_S; // Keep alive in sec
#if defined(MQTT_USERNAME) && defined(MQTT_PASSWORD)
    state.mqtt_client_info.client_user = MQTT_USERNAME;
    state.mqtt_client_info.client_pass = MQTT_PASSWORD;
#else
    state.mqtt_client_info.client_user = NULL;
    state.mqtt_client_info.client_pass = NULL;
#endif
    static char will_topic[MQTT_TOPIC_LEN];
    strncpy(will_topic, full_topic(&state, MQTT_WILL_TOPIC), sizeof(will_topic));
    state.mqtt_client_info.will_topic = will_topic;
    state.mqtt_client_info.will_msg = MQTT_WILL_MSG;
    state.mqtt_client_info.will_qos = MQTT_WILL_QOS;
    state.mqtt_client_info.will_retain = true;
#if LWIP_ALTCP && LWIP_ALTCP_TLS
    // TLS enabled
#ifdef MQTT_CERT_INC
    static const uint8_t ca_cert[] = TLS_ROOT_CERT;
    static const uint8_t client_key[] = TLS_CLIENT_KEY;
    static const uint8_t client_cert[] = TLS_CLIENT_CERT;
    // This confirms the indentity of the server and the client
    state.mqtt_client_info.tls_config = altcp_tls_create_config_client_2wayauth(ca_cert, sizeof(ca_cert),
            client_key, sizeof(client_key), NULL, 0, client_cert, sizeof(client_cert));
#if ALTCP_MBEDTLS_AUTHMODE != MBEDTLS_SSL_VERIFY_REQUIRED
    WARN_printf("Warning: tls without verification is insecure\n");
#endif
#else
    state->client_info.tls_config = altcp_tls_create_config_client(NULL, 0);
    WARN_printf("Warning: tls without a certificate is insecure\n");
#endif
#endif

    // Conectar à rede WiFI - fazer um loop até que esteja conectado
    cyw43_arch_enable_sta_mode();
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        panic("Failed to connect");
    }
    INFO_printf("\nConnected to Wifi\n");

    //Faz um pedido de DNS para o endereço IP do servidor MQTT
    cyw43_arch_lwip_begin();
    int err = dns_gethostbyname(MQTT_SERVER, &state.mqtt_server_address, dns_found, &state);
    cyw43_arch_lwip_end();

    // Se tiver o endereço, inicia o cliente
    if (err == ERR_OK) {
        start_client(&state);
    } else if (err != ERR_INPROGRESS) { // ERR_INPROGRESS means expect a callback
        panic("dns request failed");
    }

    struct repeating_timer timer;

    add_repeating_timer_ms(10000, repeating_timer_callback, NULL, &timer); // Lê o DHT11 a cada 5 segundos

    // read_dht11(&dht11); // Lê o sensor DHT11
    simulate_dht11(); // Simula a leitura do DHT11

    show_ip_address();

    // Loop condicionado a conexão mqtt
    while (!state.connect_done || mqtt_client_is_connected(state.mqtt_client_inst)) {
        if (read_sensor_dht11) {
            read_sensor_dht11 = false; // Reseta a variável
            // read_dht11(&dht11);  // Lê os dados do sensor DHT11
            simulate_dht11(); // Simula a leitura do DHT11
            show_ip_address();
        }

        cyw43_arch_poll();
        cyw43_arch_wait_for_work_until(make_timeout_time_ms(10000));
    }

    INFO_printf("mqtt client exiting\n");
    return 0;
}

/* References for this implementation:
 * raspberry-pi-pico-c-sdk.pdf, Section '4.1.1. hardware_adc'
 * pico-examples/adc/adc_console/adc_console.c */
static float read_onboard_temperature(const char unit) {

    /* 12-bit conversion, assume max value == ADC_VREF == 3.3 V */
    const float conversionFactor = 3.3f / (1 << 12);

    float adc = (float)adc_read() * conversionFactor;
    float tempC = 27.0f - (adc - 0.706f) / 0.001721f;

    if (unit == 'C' || unit != 'F') {
        return tempC;
    } else if (unit == 'F') {
        return tempC * 9 / 5 + 32;
    }

    return -1.0f;
}

// Requisição para publicar
static void pub_request_cb(__unused void *arg, err_t err) {
    if (err != 0) {
        ERROR_printf("pub_request_cb failed %d", err);
    }
}

//Topico MQTT
static const char *full_topic(MQTT_CLIENT_DATA_T *state, const char *name) {
#if MQTT_UNIQUE_TOPIC
    static char full_topic[MQTT_TOPIC_LEN];
    snprintf(full_topic, sizeof(full_topic), "/%s%s", state->mqtt_client_info.client_id, name);
    return full_topic;
#else
    return name;
#endif
}

// Controle do LED 
static void control_led(MQTT_CLIENT_DATA_T *state, bool on) {
    // Publish state on /state topic and on/off led board
    const char* message = on ? "On" : "Off";
    if (on)
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    else
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);

    mqtt_publish(state->mqtt_client_inst, full_topic(state, "/led/state"), message, strlen(message), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
}

// Publicar temperatura
static void publish_temperature(MQTT_CLIENT_DATA_T *state) {

    const char *temperature_key = full_topic(state, "/temperatura");
    const char *humidity_key = full_topic(state, "/umidade");

    printf("Temperatura: %.2f, Umidade: %.2f\n", dht11.temperature, dht11.humidity);

    // Publica a temperatura e umidade no tópico MQTT
    char temp_str[16];
    snprintf(temp_str, sizeof(temp_str), "%.2f", dht11.temperature);
    INFO_printf("Publishing %s to %s\n", temp_str, temperature_key);
    mqtt_publish(state->mqtt_client_inst, temperature_key, temp_str, strlen(temp_str), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
    char hum_str[16];
    snprintf(hum_str, sizeof(hum_str), "%.2f", dht11.humidity);
    INFO_printf("Publishing %s to %s\n", hum_str, humidity_key);
    mqtt_publish(state->mqtt_client_inst, humidity_key, hum_str, strlen(hum_str), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
    // static float old_temperature;
    // const char *temperature_key = full_topic(state, "/temperature");
    // float temperature = read_onboard_temperature(TEMPERATURE_UNITS);

    // if (temperature != old_temperature) {
    //     old_temperature = temperature;
    //     // Publish temperature on /temperature topic
    //     char temp_str[16];
    //     snprintf(temp_str, sizeof(temp_str), "%.2f", temperature);
    //     INFO_printf("Publishing %s to %s\n", temp_str, temperature_key);
    //     mqtt_publish(state->mqtt_client_inst, temperature_key, temp_str, strlen(temp_str), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
    // }
}

// Requisição de Assinatura - subscribe
static void sub_request_cb(void *arg, err_t err) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
    if (err != 0) {
        panic("subscribe request failed %d", err);
    }
    state->subscribe_count++;
}

// Requisição para encerrar a assinatura
static void unsub_request_cb(void *arg, err_t err) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
    if (err != 0) {
        panic("unsubscribe request failed %d", err);
    }
    state->subscribe_count--;
    assert(state->subscribe_count >= 0);

    // Stop if requested
    if (state->subscribe_count <= 0 && state->stop_client) {
        mqtt_disconnect(state->mqtt_client_inst);
    }
}

// Tópicos de assinatura
static void sub_unsub_topics(MQTT_CLIENT_DATA_T* state, bool sub) {
    // mqtt_request_cb_t cb = sub ? sub_request_cb : unsub_request_cb;
    // mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/led"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
    // mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/print"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
    // mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/ping"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
    // mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/exit"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
    // mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/sala"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
    // mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/cozinha"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
    // mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/quarto"), MQTT_SUBSCRIBE_QOS, cb, state, sub);

    mqtt_request_cb_t cb = sub ? sub_request_cb : unsub_request_cb;
    const char *lista[] = { "/sala", "/cozinha", "/quarto", "/portao", "/alerta" };
 // há um limite de 5 tópicos
    for (int i = 0; i < 5; i++) {
        const char *t = lista[i];
        const char *topo = full_topic(state, t);

         printf("[DEBUG] sub_unsub_topics() chamado. state=%p, mqtt_client_inst=%p\n",
               state, state->mqtt_client_inst);
        printf("[DEBUG] → Tentando %s tópico \"%s\" (len=%zu)\n",
               sub ? "SUBSCRIBE" : "UNSUBSCRIBE",
               topo, strlen(topo));

        int err = mqtt_sub_unsub(
            state->mqtt_client_inst,
            topo,
            MQTT_SUBSCRIBE_QOS,
            cb,
            state,
            sub
        );
        printf("[DEBUG] %s em tópico \"%s\" → retorno: %d\n",
               sub ? "Subscribe" : "Unsubscribe",
               topo,
               err);
    }
}

// Dados de entrada MQTT
static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags) {
    printf("mqtt_incoming_data_cb: flags %d, len %d\n", flags, len);
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
#if MQTT_UNIQUE_TOPIC
    const char *basic_topic = state->topic + strlen(state->mqtt_client_info.client_id) + 1;
#else
    const char *basic_topic = state->topic;
#endif
    strncpy(state->data, (const char *)data, len);
    state->len = len;
    state->data[len] = '\0';

    printf("Received data on topic: %s\n", state->topic);
    DEBUG_printf("Topic: %s, Message: %s\n", state->topic, state->data);
    if (strcmp(basic_topic, "/sala") == 0) {
        if (lwip_stricmp((const char *)state->data, "On") == 0 || strcmp((const char *)state->data, "1") == 0) {
            turn_on_light(&things.living_room_light, LIVING_ROOM_LIGHT);
        } else if (lwip_stricmp((const char *)state->data, "Off") == 0 || strcmp((const char *)state->data, "0") == 0) {
            turn_off_light(&things.living_room_light, LIVING_ROOM_LIGHT);
        }
    } else if (strcmp(basic_topic, "/cozinha") == 0) {
        if (lwip_stricmp((const char *)state->data, "On") == 0 || strcmp((const char *)state->data, "1") == 0) {
            turn_on_light(&things.kitchen_light, KITCHEN_LIGHT);
        } else if (lwip_stricmp((const char *)state->data, "Off") == 0 || strcmp((const char *)state->data, "0") == 0) {
            turn_off_light(&things.kitchen_light, KITCHEN_LIGHT);
        }
    } else if (strcmp(basic_topic, "/quarto") == 0) {
        if (lwip_stricmp((const char *)state->data, "On") == 0 || strcmp((const char *)state->data, "1") == 0) {
            turn_on_light(&things.bedroom_light, BEDROOM_LIGHT);
        } else if (lwip_stricmp((const char *)state->data, "Off") == 0 || strcmp((const char *)state->data, "0") == 0) {
            turn_off_light(&things.bedroom_light, BEDROOM_LIGHT);
        }
    } else if (strcmp(basic_topic, "/portao") == 0) {
        if (lwip_stricmp((const char *)state->data, "Open") == 0 || strcmp((const char *)state->data, "1") == 0) {
            gate(true);
        } else if (lwip_stricmp((const char *)state->data, "Close") == 0 || strcmp((const char *)state->data, "0") == 0) {
            gate(false);
        }
    } else if (strcmp(basic_topic, "/alerta") == 0) {
        // if (lwip_stricmp((const char *)state->data, "On") == 0 || strcmp((const char *)state->data, "1") == 0) {
            emit_alert();
        // }
    } else if (strcmp(basic_topic, "/exit") == 0) {
        state->stop_client = true;
        sub_unsub_topics(state, false); // unsubscribe
    }
}

// Dados de entrada publicados
static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
    strncpy(state->topic, topic, sizeof(state->topic));
}

// Publicar temperatura
static void temperature_worker_fn(async_context_t *context, async_at_time_worker_t *worker) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)worker->user_data;
    publish_temperature(state);
    async_context_add_at_time_worker_in_ms(context, worker, TEMP_WORKER_TIME_S * 1000);
}

// Conexão MQTT
static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
    if (status == MQTT_CONNECT_ACCEPTED) {
        state->connect_done = true;
        sub_unsub_topics(state, true); // subscribe;

        // indicate online
        if (state->mqtt_client_info.will_topic) {
            mqtt_publish(state->mqtt_client_inst, state->mqtt_client_info.will_topic, "1", 1, MQTT_WILL_QOS, true, pub_request_cb, state);
        }

        // Publish temperature every 10 sec if it's changed
        temperature_worker.user_data = state;
        async_context_add_at_time_worker_in_ms(cyw43_arch_async_context(), &temperature_worker, 0);
    } else if (status == MQTT_CONNECT_DISCONNECTED) {
        if (!state->connect_done) {
            panic("Failed to connect to mqtt server");
        }
    }
    else {
        panic("Unexpected status");
    }
}

// Inicializar o cliente MQTT
static void start_client(MQTT_CLIENT_DATA_T *state) {
#if LWIP_ALTCP && LWIP_ALTCP_TLS
    const int port = MQTT_TLS_PORT;
    INFO_printf("Using TLS\n");
#else
    const int port = MQTT_PORT;
    INFO_printf("Warning: Not using TLS\n");
#endif

    state->mqtt_client_inst = mqtt_client_new();
    if (!state->mqtt_client_inst) {
        panic("MQTT client instance creation error");
    }
    INFO_printf("IP address of this device %s\n", ipaddr_ntoa(&(netif_list->ip_addr)));
    INFO_printf("Connecting to mqtt server at %s\n", ipaddr_ntoa(&state->mqtt_server_address));

    cyw43_arch_lwip_begin();
    if (mqtt_client_connect(state->mqtt_client_inst, &state->mqtt_server_address, port, mqtt_connection_cb, state, &state->mqtt_client_info) != ERR_OK) {
        panic("MQTT broker connection error");
    }
#if LWIP_ALTCP && LWIP_ALTCP_TLS
    // This is important for MBEDTLS_SSL_SERVER_NAME_INDICATION
    mbedtls_ssl_set_hostname(altcp_tls_context(state->mqtt_client_inst->conn), MQTT_SERVER);
#endif
    mqtt_set_inpub_callback(state->mqtt_client_inst, mqtt_incoming_publish_cb, mqtt_incoming_data_cb, state);
    cyw43_arch_lwip_end();
}

// Call back com o resultado do DNS
static void dns_found(const char *hostname, const ip_addr_t *ipaddr, void *arg) {
    MQTT_CLIENT_DATA_T *state = (MQTT_CLIENT_DATA_T*)arg;
    if (ipaddr) {
        state->mqtt_server_address = *ipaddr;
        start_client(state);
    } else {
        panic("dns request failed");
    }
}

/**
 * Atribui uma cor RGB a um LED.
 */
void npSetLED(const uint index, const uint8_t r, const uint8_t g, const uint8_t b) {
    leds[index].R = r;
    leds[index].G = g;
    leds[index].B = b;
}
 
void npClear() {
    for (uint i = 0; i < LED_COUNT; ++i)
        npSetLED(i, 0, 0, 0);
}

void npWrite() {
    for (uint i = 0; i < LED_COUNT; ++i) {
        pio_sm_put_blocking(np_pio, sm, leds[i].G);
        pio_sm_put_blocking(np_pio, sm, leds[i].R);
        pio_sm_put_blocking(np_pio, sm, leds[i].B);
    }
    sleep_us(100);
}
  
void npInit(uint pin) {
    // Cria programa PIO.
    uint offset = pio_add_program(pio0, &ws2818b_program);
    np_pio = pio0;

    // Toma posse de uma máquina PIO.
    sm = pio_claim_unused_sm(np_pio, false);
    if (sm < 0) {
        np_pio = pio1;
        sm = pio_claim_unused_sm(np_pio, true); // Se nenhuma máquina estiver livre, panic!
    }

    // Inicia programa na máquina PIO obtida.
    ws2818b_program_init(np_pio, sm, offset, pin, 800000.f);

    npClear();
    npWrite(); // Limpa os LEDs.
}

void turn_on_light(bool *light, uint index) {
    *light = true;
    if (index == LIVING_ROOM_LIGHT) {
        npSetLED(index, 255, 0, 0); // Vermelho
    } else if (index == KITCHEN_LIGHT) {
        npSetLED(index, 0, 255, 0); // Verde
    } else if (index == BEDROOM_LIGHT) {
        npSetLED(index, 0, 0, 255); // Azul
    }
    npWrite();
}

void turn_off_light(bool *light, uint index) {
    *light = false;
    npSetLED(index, 0, 0, 0);
    npWrite();
}

void gate(bool state) {
    things.gate = state;

    if (state) {
        printf("Portão aberto\n");
        set_servo_position(SERVO_PIN, 2400); // Abre o portão
    } else {
        printf("Portão fechado\n");
        set_servo_position(SERVO_PIN, 500); // Fecha o portão
    }
}

void emit_alert() {
    printf("Alerta acionado\n");
    play_tone(BUZZER_PIN, 392, 1000); // Toca um tom de alerta
}

void pwm_init_buzzer(uint pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(pin);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 4.0f); // Ajusta divisor de clock
    pwm_init(slice_num, &config, true);
    pwm_set_gpio_level(pin, 0); // Desliga o PWM inicialmente
}

// Função para configurar o PWM
void setup_pwm(uint pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM); // Configura o pino como saída PWM
    uint slice_num = pwm_gpio_to_slice_num(pin); // Obtém o número do slice PWM
    pwm_set_wrap(slice_num, PWM_WRAP); // Define o período do PWM para 20ms
    pwm_set_clkdiv(slice_num, 125.0f); // Configuração do divisor de clock para atingir 50Hz
    pwm_set_enabled(slice_num, true); // Habilita o PWM
}

// Toca uma nota com a frequência e duração especificadas
void play_tone(uint pin, uint frequency, uint duration_ms) {
    uint slice_num = pwm_gpio_to_slice_num(pin);
    uint32_t clock_freq = clock_get_hz(clk_sys);
    uint32_t top = clock_freq / frequency - 1;

    pwm_set_wrap(slice_num, top);
    pwm_set_gpio_level(pin, top / 2); // 50% de duty cycle

    sleep_ms(duration_ms);

    pwm_set_gpio_level(pin, 0); // Desliga o som após a duração
    sleep_ms(50); // Pausa entre notas
}

// Função para definir o ciclo ativo (duty cycle) do servo em microssegundos
void set_servo_position(uint pin, uint pulse_width) {
    uint slice_num = pwm_gpio_to_slice_num(pin);
    pwm_set_gpio_level(pin, pulse_width); // Define o nível PWM
}

static void simulate_dht11() {
    // Humidade entre 30% e 90%
    dht11.humidity    = 30.0f + (rand()/(float)RAND_MAX) * 60.0f;
    // Temperatura entre 15°C e 35°C
    dht11.temperature = 15 + (rand()/(float)RAND_MAX) * 20.0f;
    printf("Simulated Umidade: %.2f%%, Temperatura: %.2f°C\n",
           dht11.humidity, dht11.temperature);
}

// Função para ler os dados do sensor DHT11
int read_dht11(DHT11 *sensor) {
    uint8_t data[5] = {0};

    // Inicializa o pino como saída e envia o pulso de start
    gpio_init(DHT_PIN);
    gpio_set_dir(DHT_PIN, GPIO_OUT);
    gpio_put(DHT_PIN, 0);
    sleep_ms(20);  // Mantém por 20ms
    gpio_put(DHT_PIN, 1);
    sleep_us(40);  // Espera 40us

    gpio_set_dir(DHT_PIN, GPIO_IN);

    // Aguarda resposta do sensor (LOW por ~80us)
    while (gpio_get(DHT_PIN) == 1);
    while (gpio_get(DHT_PIN) == 0);
    while (gpio_get(DHT_PIN) == 1);

    // Lê os 40 bits
    for (int i = 0; i < 40; i++) {
        while (gpio_get(DHT_PIN) == 0); // Espera 50us de LOW

        // Marca o tempo do pulso HIGH
        uint32_t start_time = time_us_32();
        while (gpio_get(DHT_PIN) == 1);
        uint32_t pulse_duration = time_us_32() - start_time;

        // Se o HIGH for maior que 40us, é 1, senão 0
        data[i / 8] <<= 1;
        if (pulse_duration > 40) {
            data[i / 8] |= 1;
        }
    }

    // Verifica checksum
    uint8_t checksum = data[0] + data[1] + data[2] + data[3];
    if (data[4] != checksum) return -1;

    sensor->humidity = data[0];
    sensor->temperature = data[2];

    // Exibe os dados lidos
    printf("Umidade: %d%%, Temperatura: %d°C\n", sensor->humidity, sensor->temperature);

    return 0;
}

bool repeating_timer_callback(struct repeating_timer *t) {
    read_sensor_dht11 = true; // Define a variável para ler o sensor DHT11
    return true;
}

void i2c_setup() {
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
}


void show_ip_address() {
    uint8_t *ip_address = (uint8_t*)&(cyw43_state.netif[0].ip_addr.addr);
    printf("Endereço IP %d.%d.%d.%d\n", ip_address[0], ip_address[1], ip_address[2], ip_address[3]);

    char ip_str_0[4];
    char ip_str_1[4];
    char ip_str_2[4];
    char ip_str_3[4];
    char temp_str[6];
    char hum_str[6];
    snprintf(temp_str, sizeof(temp_str), "%.2f°C", dht11.temperature);
    snprintf(hum_str, sizeof(hum_str), "%.2f%%", dht11.humidity);
    sprintf(ip_str_0, "%d.", ip_address[0]);
    sprintf(ip_str_1, "%d.", ip_address[1]);
    sprintf(ip_str_2, "%d.", ip_address[2]);
    sprintf(ip_str_3, "%d", ip_address[3]);

    ssd1306_draw_string(&ssd, "Temp: ", 3, 3);
    ssd1306_draw_string(&ssd, temp_str, 56, 3);
    ssd1306_draw_string(&ssd, "Umidade: ", 3, 13);
    ssd1306_draw_string(&ssd, hum_str, 80, 13);

    ssd1306_draw_string(&ssd, ip_str_0, 1, 30);
    ssd1306_draw_string(&ssd, ip_str_1, 26, 30);
    ssd1306_draw_string(&ssd, ip_str_2, 42, 30);
    ssd1306_draw_string(&ssd, ip_str_3, 59, 30);
    ssd1306_send_data(&ssd);
}