#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "driver/gpio.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/i2c_master.h" // <- NOVO HEADER I2C
#include "sdkconfig.h"
#include "driver/pulse_cnt.h"
#include <math.h> // Incluído para a função fabsf (valor absoluto)
#include "esp_adc/adc_oneshot.h" // A nova API recomendada para o ADC
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_sleep.h"
#include "driver/rtc_io.h"
#include "bme280.h"
#include "wifi_connect.h"

#define SENSOR_POWER_PIN GPIO_NUM_2
#define TEMPO_DE_SONO_SEGUNDOS 180 // Configuração de tempo para Deep Sleep
#define TEMPO_EM_MICROSSEGUNDOS (TEMPO_DE_SONO_SEGUNDOS * 1000000ULL) // Converte para microssegundos

// ============================================================ protótipos MQTT =============================================================================

static const char *TAG = "MQTT";
static esp_mqtt_client_handle_t client = NULL;
static EventGroupHandle_t mqtt_event_group;
const int MQTT_CONNECTED_BIT = BIT0;
#define MQTT_BROKER_URI "meu_endereço_mqtt"

/*
 * @brief Event handler registrado para receber eventos MQTT
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);

/**
 * @brief Publica uma mensagem MQTT se o cliente estiver conectado.
 * * @param topic O tópico para onde a mensagem será enviada.
 * @param payload O conteúdo (string) da mensagem.
 */
static void publish_message(const char *topic, const char *payload);


static void mqtt_app_start(void);

//============================================================= protótipos bme280 ====================================================================

static const char *TAG_BME280 = "TAG_BME280";
#define BME280_SDA_PIN GPIO_NUM_21
#define BME280_SCL_PIN GPIO_NUM_22
#define I2C_MASTER_PORT     I2C_NUM_0
#define I2C_MASTER_FREQ_HZ  400000 // 400kHz
#define I2C_TIMEOUT_MS 100
// Handle para o dispositivo BME280 no barramento I2C
static i2c_master_dev_handle_t bme280_dev_handle = NULL;
// Estrutura da biblioteca BME280 para guardar estado e calibração
static struct bme280_t bme280_dev;
// Flag para indicar se a inicialização foi bem-sucedida
static bool bme280_initialized = false;

// =================================================================================
// Funções de Interface I2C e Delay (Ponte para a biblioteca bme280.c)
// =================================================================================

void user_delay_ms(BME280_MDELAY_DATA_TYPE period);

BME280_RETURN_FUNCTION_TYPE user_i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t len);

BME280_RETURN_FUNCTION_TYPE user_i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t len);

/**
 * @brief Inicializa o barramento I2C e o sensor BME280.
 *
 * @return true se a inicialização foi bem-sucedida, false caso contrário.
 */
bool init_bme280(void);

/**
 * @brief Lê o sensor BME280 uma vez, calcula os valores e imprime no console.
 * Assume que init_bme280() já foi chamada com sucesso.
 */
void printBME280(double *temp_ptr, double *press_ptr, double *hum_ptr);

// ======================================================== sensor de chuva =====================================================================

// --- Configuração da TAG e do Pino ---
static const char *TAG_CHUVA = "TAG_CHUVA";
#define RAIN_SENSOR_GPIO_PIN    34
#define ADC_UNIT    ADC_UNIT_1     // ADC1 é usado para GPIO34
#define RAIN_SENSOR_ADC_CHANNEL ADC_CHANNEL_6  // GPIO34 corresponde ao ADC1_CHANNEL_6
#define RAIN_SENSOR_ADC_ATTEN   ADC_ATTEN_DB_11 // Atenuação para ler a faixa completa 0-~3.3V

// Handle global para a unidade ADC (necessário para a função printChuva)
adc_oneshot_unit_handle_t adc1_handle = NULL; // Inicializa como NULL

/**
 * @brief Inicializa o canal para o sensor de chuva.
 * Deve ser chamada uma vez antes de usar printChuva.
 * Simplificada para usar ESP_ERROR_CHECK para lidar com erros.
 */
void init_rain_sensor_adc(void);

/**
 * @brief Lê o sensor de chuva uma vez, calcula os valores e imprime no console.
 *
 * @param adc_handle Handle da unidade ADC já inicializada.
 */
float printChuva(adc_oneshot_unit_handle_t adc_handle);

// ======================================================== encoder ==============================================================================

static const char *TAG_ENCODER = "TAG_ENCODER";

// --- AJUSTES PARA O ENCODER E ANEMÔMETRO ---
// Definições do Hardware Encoder E38S6G5-200B-G24N
#define ENCODER_PPR_FISICO      200     // Pulsos Físicos por Rotação
#define ENCODER_RESOLUCAO_4X    (ENCODER_PPR_FISICO * 4) // Resolução efetiva com 4x decoding = 800

#define ENCODER_GPIO_A          25
#define ENCODER_GPIO_B          26

// Definições Físicas do Anemômetro
#define RAIO_ANEMOMETRO_M       0.16f   // 16 cm
#define FATOR_ANEMOMETRO        2.5f    // Fator de calibração
#define PI                      3.14159f

// Intervalo de amostragem para cálculo de velocidade em milissegundos
#define SAMPLE_INTERVAL_MS      1000

// --- Variáveis Globais ---
// Handle para a unidade PCNT (precisa ser acessível pela função de leitura)
static pcnt_unit_handle_t pcnt_unit = NULL;
// Flag para indicar se a inicialização foi bem-sucedida
static bool pcnt_initialized = false;
// Variável para guardar a posição anterior entre chamadas da função de leitura
static int pos_anterior_global = 0;


// =================================================================================
// Função de Inicialização (Chamada uma vez)
// =================================================================================

/**
 * @brief Inicializa o hardware PCNT para leitura do encoder.
 *
 * @return true se a inicialização foi bem-sucedida, false caso contrário.
 */
bool init_encoder_pcnt(void);

// =================================================================================
// Função de Leitura e Impressão
// =================================================================================
/**
 * @brief Realiza uma medição de velocidade do anemómetro durante SAMPLE_INTERVAL_MS
 * e imprime o resultado. Assume que init_encoder_pcnt() já foi chamada.
 */
float printAnemometerReading(void);

// ======================================================== biruta =====================================================================================

static const char *TAG_BIRUTA = "TAG_BIRUTA";
#define BIRUTA_GPIO_PIN        32 
#define BIRUTA_ADC_CHANNEL     ADC_CHANNEL_4 // GPIO32 corresponde ao ADC1_CHANNEL_4
#define BIRUTA_ADC_ATTEN       ADC_ATTEN_DB_11 // Atenuação para ler a faixa completa 0-~3.3V

// --- AJUSTES ---
// Define os valores ADC esperados para 0 graus (Norte) e ~360 graus
// Com o divisor 1.8k/3.3k, o máximo teórico é ~4014
#define ADC_MIN_EXPECTED 0     // Assumindo que 0V = 0 ADC = Norte
#define ADC_MAX_EXPECTED 4014
// Handle para a calibração
static adc_cali_handle_t adc1_cali_handle_biruta = NULL;
// Flag para indicar se a calibração está ativa
static bool do_calibration_biruta = false;

// Função map
long map(long x, long in_min, long in_max, long out_min, long out_max);

// Função angle_to_direction
const char* angle_to_direction(float angle);

/**
 * @brief Lê o sensor da biruta uma vez, calcula o ângulo/direção e imprime.
 * Assume que init_biruta_adc() já foi chamada com sucesso.
 */
float printBirutaReading(void);

// ====================================================== LDR =========================================================================================

static const char *TAG_LDR = "TAG_LDR";
#define LDR_GPIO_PIN            33
#define LDR_ADC_CHANNEL         ADC_CHANNEL_5  // GPIO33 corresponde ao ADC1_CHANNEL_5
#define LDR_ADC_ATTEN           ADC_ATTEN_DB_11 // Atenuação para ler a faixa completa 0-~3.3V

/**
 * @brief Lê o sensor LDR uma vez, calcula a percentagem e imprime no console.
 *
 * @param adc_handle Handle da unidade ADC já inicializada.
 */
float printLDR(adc_oneshot_unit_handle_t adc_handle);

// ====================================================================================================================================================
// ========================================================== MAIN ====================================================================================
// ====================================================================================================================================================

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    // ============================================= LIGAR SENSORES (Via MOSFET)
    // Reseta o pino para garantir que não há configurações antigas
    gpio_reset_pin(SENSOR_POWER_PIN);
    gpio_set_direction(SENSOR_POWER_PIN, GPIO_MODE_OUTPUT);
    // Se o pino estava "segurado" (hold) pelo Deep Sleep anterior, solte-o agora
    rtc_gpio_hold_dis(SENSOR_POWER_PIN);
    gpio_set_level(SENSOR_POWER_PIN, 1);
    // Espera 200ms para os sensores estabilizarem a tensão
    vTaskDelay(pdMS_TO_TICKS(200));
    // =============================================== Inicialização BME 280:
    init_bme280();
    ESP_LOGI(TAG_BME280, "Sensor BME280 inicializado.");

    //================================================ Inicialização sensor de chuva
    init_rain_sensor_adc();
    ESP_LOGI(TAG_CHUVA, "Sensor de chuva inicializado.");

    // ============================================== Inicialização do encoder
    init_encoder_pcnt();
    ESP_LOGI(TAG_ENCODER, "Encoder inicializado.");

    // ================================================================= Leitura ============================================================
    // ============================== BME 280
    double temp = 0.0;
    double press = 0.0;
    double hum = 0.0;
    printBME280(&temp, &press, &hum);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    // ============================== Sensor de chuva
    float nivelChuva = printChuva(adc1_handle);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    // =============================== Encoder (anemometro)
    float vel_vento = printAnemometerReading();
    vTaskDelay(100 / portTICK_PERIOD_MS);

    // =============================== biruta
    float direcao = printBirutaReading();
    vTaskDelay(100 / portTICK_PERIOD_MS);

    //================================= LDR
    float luminosidade = printLDR(adc1_handle);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    // ============================================
    // Forçar SDA e SCL para LOW (0V), isso drena qualquer energia residual e impede que o sensor se alimente pelos dados
    gpio_reset_pin(BME280_SDA_PIN);
    gpio_set_direction(BME280_SDA_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(BME280_SDA_PIN, 0);

    gpio_reset_pin(BME280_SCL_PIN);
    gpio_set_direction(BME280_SCL_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(BME280_SCL_PIN, 0);

    // ================================= Desliga o MOSFET
    gpio_set_level(SENSOR_POWER_PIN, 0);
    // ativa o hold para garantir que o pino continue LOW enquanto o ESP dorme
    rtc_gpio_hold_en(SENSOR_POWER_PIN);

    //=================================== MQTT
    // Espera 100ms para estabilizar a tensão
    vTaskDelay(100 / portTICK_PERIOD_MS);
    wifi_init_sta();
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "MQTT Startup..");
    mqtt_app_start();
    // Carrega as variáveis no payload e envia
    char payload_string[256];
    sprintf(payload_string, "{\"temperatura\": %.1f, \"umidade\": %.1f, \"pressao\": %.1f, \"chuva\": %.1f, \"velocidade\": %.1f,\"direcao\": %.1f, \"luminosidade\": %.1f}", 
    temp, hum, press, nivelChuva, vel_vento, direcao, luminosidade);
    publish_message("dados", payload_string);
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    //Configuração e Entrada em Sono Profundo (Deep Sleep)
    ESP_LOGI("SLEEP", "Configurando despertador para %d segundos...", TEMPO_DE_SONO_SEGUNDOS);
    esp_sleep_enable_timer_wakeup(TEMPO_EM_MICROSSEGUNDOS);
    ESP_LOGI("SLEEP", "Entrando em Deep Sleep.");
    esp_deep_sleep_start();
}

// ================================================ implementações:

// ============================================================ código MQTT =============================================================================
/*
 * @brief Event handler registrado para receber eventos MQTT
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32, base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        // Sinaliza que a conexão foi estabelecida
        xEventGroupSetBits(mqtt_event_group, MQTT_CONNECTED_BIT);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        // Limpa o bit para indicar que a conexão foi perdida
        xEventGroupClearBits(mqtt_event_group, MQTT_CONNECTED_BIT);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG, "MQTT_EVENT_ERROR");
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

/**
 * @brief Publica uma mensagem MQTT se o cliente estiver conectado.
 * * @param topic O tópico para onde a mensagem será enviada.
 * @param payload O conteúdo (string) da mensagem.
 */
static void publish_message(const char *topic, const char *payload) 
{

    ESP_LOGI(TAG, "Aguardando conexão MQTT para enviar...");

    // O código trava aqui e espera o bit MQTT_CONNECTED_BIT ficar ativo.
    // Timeout de 10 segundos (se não conectar em 10s, ele desiste).
    EventBits_t bits = xEventGroupWaitBits(mqtt_event_group,
                                           MQTT_CONNECTED_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           pdMS_TO_TICKS(10000)); 

    // Verifica se saiu do wait porque conectou ou porque deu timeout
    if (bits & MQTT_CONNECTED_BIT) {
        ESP_LOGI(TAG, "MQTT conectado! Enviando...");
        int msg_id = esp_mqtt_client_publish(client, topic, payload, 0, 1, 0);
        ESP_LOGI(TAG, "Enviado com sucesso, msg_id=%d", msg_id);
    } else {
        ESP_LOGE(TAG, "TIMEOUT: Falha ao conectar no MQTT após 10s. Dado perdido.");
    }
}
// ---------------------------------------------------------------------------------

static void mqtt_app_start(void)
{
    // Cria o event group para sincronização
    mqtt_event_group = xEventGroupCreate();

    // Configuração do cliente MQTT 
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
        // .credentials.username = "seu_usuario",    // Removido para conexão anônima
        // .credentials.authentication.password = "sua_senha", // Removido para conexão anônima
    };

    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}

//============================================================= código bme280 ====================================================================

// =================================================================================
// Funções de Interface I2C e Delay (Ponte para a biblioteca bme280.c)
// =================================================================================

void user_delay_ms(BME280_MDELAY_DATA_TYPE period) {
    vTaskDelay(pdMS_TO_TICKS(period));
}

BME280_RETURN_FUNCTION_TYPE user_i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t len) {
    // Verifica se o handle do dispositivo I2C é válido
    if (!bme280_dev_handle) return -1; // Ou outro código de erro apropriado

    uint8_t write_buf[len + 1];
    write_buf[0] = reg_addr;
    memcpy(write_buf + 1, reg_data, len);

    esp_err_t ret = i2c_master_transmit(bme280_dev_handle, write_buf, sizeof(write_buf), pdMS_TO_TICKS(I2C_TIMEOUT_MS)); // Timeout menor

    // A biblioteca espera 0 para sucesso, < 0 para erro.
    //return (ret == ESP_OK) ? SUCCESS : ERROR; // Usa erro definido pela lib
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_BME280, "I2C Write Failed: %s", esp_err_to_name(ret));
        return -1; // Retorna erro para a biblioteca BME280 parar
    }
    return SUCCESS;
}

BME280_RETURN_FUNCTION_TYPE user_i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t len) {
    if (!bme280_dev_handle) return -1;

    // A API transmit_receive faz escrita do registo + leitura dos dados
    esp_err_t ret = i2c_master_transmit_receive(bme280_dev_handle, &reg_addr, 1, reg_data, len, pdMS_TO_TICKS(I2C_TIMEOUT_MS)); // Timeout menor

    if (ret != ESP_OK) {
        ESP_LOGE(TAG_BME280, "I2C Read Failed: %s", esp_err_to_name(ret));
        return -1; // Retorna erro para a biblioteca BME280 parar
    }
    return SUCCESS;
}

/**
 * @brief Inicializa o barramento I2C e o sensor BME280.
 *
 * @return true se a inicialização foi bem-sucedida, false caso contrário.
 */
bool init_bme280(void) {
    ESP_LOGI(TAG_BME280, "Inicializando I2C e BME280...");

    // 1. Configura o barramento I2C (se não foi inicializado por outro sensor)
    i2c_master_bus_config_t i2c_bus_config = {
        .scl_io_num = BME280_SCL_PIN,
        .sda_io_num = BME280_SDA_PIN,
        .i2c_port = I2C_MASTER_PORT,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7, // Filtro de ruído
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus_handle;
    esp_err_t ret = i2c_new_master_bus(&i2c_bus_config, &bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_BME280, "Falha ao inicializar barramento I2C: %s", esp_err_to_name(ret));
        return false;
    }

    // 2. Adiciona o dispositivo BME280 ao barramento
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BME280_I2C_ADDRESS1, // 0x76 (ou BME280_I2C_ADDRESS2 para 0x77)
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    // Tenta adicionar o dispositivo, o handle é salvo em bme280_dev_handle (global)
    ret = i2c_master_bus_add_device(bus_handle, &dev_config, &bme280_dev_handle);
     if (ret != ESP_OK) {
        ESP_LOGE(TAG_BME280, "Falha ao adicionar dispositivo BME280 ao barramento: %s", esp_err_to_name(ret));
        // Se falhar ao adicionar, tenta remover o barramento para limpar
        i2c_del_master_bus(bus_handle);
        return false;
    }
    ESP_LOGI(TAG_BME280, "Dispositivo BME280 adicionado ao barramento I2C.");

    // 3. Preenche a estrutura da biblioteca BME280 com as funções de interface
    bme280_dev.bus_write = user_i2c_write;
    bme280_dev.bus_read = user_i2c_read;
    bme280_dev.delay_msec = user_delay_ms;
    bme280_dev.dev_addr = BME280_I2C_ADDRESS1; // Garante que a struct tem o endereço correto

    // 4. Inicializa a biblioteca BME280 (lê chip ID, calibração)
    s32 com_rslt = bme280_init(&bme280_dev);
    if (com_rslt != SUCCESS) {
        ESP_LOGE(TAG_BME280, "Falha ao inicializar a biblioteca BME280 (bme280_init). Erro: %d", com_rslt);
        // Tenta limpar recursos I2C em caso de falha
        i2c_master_bus_rm_device(bme280_dev_handle);
        i2c_del_master_bus(bus_handle);
        bme280_dev_handle = NULL; // Marca handle como inválido
        return false;
    }
    ESP_LOGI(TAG_BME280, "Biblioteca BME280 inicializada (Chip ID: 0x%x).", bme280_dev.chip_id);

    // 5. Configura o modo de operação do sensor (Ex: Normal Mode)
    //    Estas configurações podem ser ajustadas conforme necessidade.
    com_rslt += bme280_set_oversamp_pressure(BME280_OVERSAMP_16X);
    com_rslt += bme280_set_oversamp_temperature(BME280_OVERSAMP_2X);
    com_rslt += bme280_set_oversamp_humidity(BME280_OVERSAMP_1X);
    com_rslt += bme280_set_filter(BME280_FILTER_COEFF_16);
    // IMPORTANTE: Para leitura única, usar FORCED_MODE é mais eficiente em energia.
    //             NORMAL_MODE mantém o sensor sempre ativo.
    // com_rslt += bme280_set_power_mode(BME280_NORMAL_MODE);
    // Vamos usar FORCED_MODE para a função de leitura única
    com_rslt += bme280_set_power_mode(BME280_SLEEP_MODE); // Garante que começa em sleep

    if (com_rslt != SUCCESS) {
         ESP_LOGE(TAG_BME280, "Falha ao configurar parâmetros do BME280. Erro acumulado: %d", com_rslt);
         // Tenta limpar recursos I2C
         i2c_master_bus_rm_device(bme280_dev_handle);
         i2c_del_master_bus(bus_handle);
         bme280_dev_handle = NULL;
         return false;
    }

    ESP_LOGI(TAG_BME280, "Sensor BME280 configurado.");
    bme280_initialized = true; // Marca como inicializado com sucesso
    return true;
}

/**
 * @brief Lê o sensor BME280 uma vez, calcula os valores e imprime no console.
 * Assume que init_bme280() já foi chamada com sucesso.
 */
void printBME280(double *temp_ptr, double *press_ptr, double *hum_ptr) {
    // Só executa se a inicialização foi bem-sucedida
    if (!bme280_initialized) {
        ESP_LOGE(TAG_BME280, "BME280 não inicializado. A saltar leitura.");
        // Delay para evitar spam de logs
        vTaskDelay(pdMS_TO_TICKS(2000));
        return;
    }

    s32 com_rslt;
    s32 v_uncomp_pressure_s32;
    s32 v_uncomp_temperature_s32;
    s32 v_uncomp_humidity_s32;

    // 1. Coloca o sensor em Forced Mode para iniciar uma medição única
    com_rslt = bme280_set_power_mode(BME280_FORCED_MODE);
    if (com_rslt != SUCCESS) {
        ESP_LOGE(TAG_BME280, "Falha ao ativar Forced Mode. Erro: %d", com_rslt);
        return; // Sai da função se não conseguir ativar a medição
    }

    // 2. Calcula e espera o tempo máximo de medição (baseado nos oversamplings)
    u8 v_delaytime_u8 = 0;
    bme280_compute_wait_time(&v_delaytime_u8);
    user_delay_ms(v_delaytime_u8 + 5); // Adiciona uma pequena margem

    // 3. Lê os dados brutos (não compensados)
    com_rslt = bme280_read_uncomp_pressure_temperature_humidity(
        &v_uncomp_pressure_s32, &v_uncomp_temperature_s32, &v_uncomp_humidity_s32);

    // Nota: Após a leitura em Forced Mode, o sensor volta automaticamente para Sleep Mode.

    if (com_rslt == SUCCESS) {
        // 4. Compensa os dados para obter valores reais (usando ponto flutuante)
        double temp = bme280_compensate_temperature_double(v_uncomp_temperature_s32);
        double press = bme280_compensate_pressure_double(v_uncomp_pressure_s32) / 100.0; // Pa -> hPa
        double hum = bme280_compensate_humidity_double(v_uncomp_humidity_s32);

        // Preenche as variáveis externas utilizando ponteros, para poder enviar no payload
        *temp_ptr = temp;
        *press_ptr = press;
        *hum_ptr = hum;

        // 5. Imprime os resultados
        printf("-------------------- BME280 --------------------\n");
        printf("Temperatura: %.2f C\n", temp);
        printf("Pressão:     %.2f hPa\n", press);
        printf("Humidade:    %.2f %%\n", hum);
        printf("--------------------------------------------------\n");

    } else {
        ESP_LOGE(TAG_BME280, "Erro ao ler dados do sensor BME280. code: %d", com_rslt);
    }
} 

// ======================================================== sensor de chuva =====================================================================
/**
 * @brief Inicializa o canal para o sensor de chuva.
 * Deve ser chamada uma vez antes de usar printChuva.
 * Simplificada para usar ESP_ERROR_CHECK para lidar com erros.
 */
void init_rain_sensor_adc(void) {
    ESP_LOGI(TAG_CHUVA, "Inicializando ADC...");

    // Configuração da unidade ADC
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
   };
    // ESP_ERROR_CHECK aborta em caso de erro
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));
    // Configuração do canal ADC
    adc_oneshot_chan_cfg_t config = {
        .atten = RAIN_SENSOR_ADC_ATTEN,      // Atenuação para ler 0-3.3V
        .bitwidth = ADC_BITWIDTH_DEFAULT,    // Largura de bits padrão (12-bit)
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, RAIN_SENSOR_ADC_CHANNEL, &config));

    ESP_LOGI(TAG_CHUVA, "ADC configurado com sucesso.");
}

/**
 * @brief Lê o sensor de chuva uma vez, calcula os valores e imprime no console.
 *
 * @param adc_handle Handle da unidade ADC já inicializada.
 */
float printChuva(adc_oneshot_unit_handle_t adc_handle) {
    // Verifica se o handle é válido (a inicialização pode ter falhado antes do ESP_ERROR_CHECK parar)
    if (adc_handle == NULL) {
        ESP_LOGE(TAG_CHUVA, "Handle ADC inválido em printChuva. A inicialização falhou?");
        // Adiciona um pequeno delay para evitar spam de logs em caso de falha contínua
        vTaskDelay(pdMS_TO_TICKS(1000));
        return 1;
    }

    int leituraChuva = -1; // Valor inicial inválido

    // Lê o valor bruto do ADC
    esp_err_t read_ret = adc_oneshot_read(adc_handle, RAIN_SENSOR_ADC_CHANNEL, &leituraChuva);

    if (read_ret == ESP_OK) {
        // Lógica invertida do sensor: Mais chuva = menor leitura ADC.
        int leituraInvertida = 4095 - leituraChuva;

        // Cálculos
        float tensaoChuva = leituraChuva * (3.3 / 4095.0); // Tensão estimada (sem calibração)
        float nivelChuva = (leituraInvertida / 4095.0) * 100.0; // Nível percentual (0-100%)
        const char *classific = "";

        if (nivelChuva <= 5)       { classific = "☀️ Sem chuva - céu limpo"; }
        else if (nivelChuva <= 20) { classific = "🌤 Garoa fraca"; }
        else if (nivelChuva <= 40) { classific = "🌦 Chuva leve"; }
        else if (nivelChuva <= 60) { classific = "🌧 Chuva moderada"; }
        else if (nivelChuva <= 80) { classific = "🌧️ Chuva forte"; }
        else                       { classific = "⛈️ Temporal intenso"; }

        printf("-------------------- CHUVA ---------------------\n");
        printf("Leitura ADC:     %d\n", leituraChuva);
        printf("Tensão Estimada: %.2f V\n", tensaoChuva);
        printf("Nível de Chuva:  %.2f %%\n", nivelChuva);
        printf("Classificação:   %s\n", classific);
        printf("--------------------------------------------------\n");
        return nivelChuva;
    } else {
        ESP_LOGE(TAG_CHUVA, "Erro na leitura do ADC: %s", esp_err_to_name(read_ret));
        return 1;
    }
}

// ======================================================== encoder ==============================================================================

// =================================================================================
// Função de Inicialização (Chamada uma vez)
// =================================================================================

/**
 * @brief Inicializa o hardware PCNT para leitura do encoder.
 *
 * @return true se a inicialização foi bem-sucedida, false caso contrário.
 */
bool init_encoder_pcnt(void) {
    ESP_LOGI(TAG_ENCODER, "Configurando hardware PCNT para encoder de %d PPR (Res. Efetiva: %d)", ENCODER_PPR_FISICO, ENCODER_RESOLUCAO_4X);

    // Configuração da unidade PCNT para contagem contínua
    pcnt_unit_config_t unit_config = {
        .high_limit = 32767,
        .low_limit = -32768,
    };
    // Tenta criar a unidade, o handle é salvo em pcnt_unit (global)
    esp_err_t ret = pcnt_new_unit(&unit_config, &pcnt_unit);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_ENCODER, "Falha ao criar unidade PCNT: %s", esp_err_to_name(ret));
        return false;
    }

    // Filtro de ruído
    pcnt_glitch_filter_config_t filter_config = { .max_glitch_ns = 1000 };
    ret = pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_ENCODER, "Falha ao configurar filtro glitch: %s", esp_err_to_name(ret));
        pcnt_del_unit(pcnt_unit); // Limpa a unidade criada
        pcnt_unit = NULL;
        return false;
    }

    // Configuração dos canais A e B
    pcnt_chan_config_t chan_a_config = { .edge_gpio_num = ENCODER_GPIO_A, .level_gpio_num = ENCODER_GPIO_B };
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ret = pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a);
     if (ret != ESP_OK) {
        ESP_LOGE(TAG_ENCODER, "Falha ao criar canal A: %s", esp_err_to_name(ret));
        pcnt_del_unit(pcnt_unit);
        pcnt_unit = NULL;
        return false;
    }

    pcnt_chan_config_t chan_b_config = { .edge_gpio_num = ENCODER_GPIO_B, .level_gpio_num = ENCODER_GPIO_A };
    pcnt_channel_handle_t pcnt_chan_b = NULL;
    ret = pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b);
     if (ret != ESP_OK) {
        ESP_LOGE(TAG_ENCODER, "Falha ao criar canal B: %s", esp_err_to_name(ret));
        // Limpa canal A e unidade
        pcnt_del_channel(pcnt_chan_a);
        pcnt_del_unit(pcnt_unit);
        pcnt_unit = NULL;
        return false;
    }

    // Configuração da decodificação 4x
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    // Habilita, limpa e inicia o contador
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));

    ESP_LOGI(TAG_ENCODER, "Unidade PCNT iniciada com sucesso.");
    pcnt_initialized = true; // Marca como inicializado
    pos_anterior_global = 0; // Zera a posição anterior inicial
    return true;
}

// =================================================================================
// Função de Leitura e Impressão
// =================================================================================
/**
 * @brief Realiza uma medição de velocidade do anemómetro durante SAMPLE_INTERVAL_MS
 * e imprime o resultado. Assume que init_encoder_pcnt() já foi chamada.
 */
float printAnemometerReading(void) {
    if (!pcnt_initialized || pcnt_unit == NULL) {
        ESP_LOGE(TAG_ENCODER, "PCNT não inicializado. A saltar leitura.");
        // Delay para evitar spam de logs
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_INTERVAL_MS));
        return 1;
    }

    int pos_atual = 0;

    vTaskDelay(pdMS_TO_TICKS(SAMPLE_INTERVAL_MS));

    // Lê a posição final do contador
    esp_err_t ret = pcnt_unit_get_count(pcnt_unit, &pos_atual);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_ENCODER, "Erro ao ler contador PCNT: %s", esp_err_to_name(ret));
        // Não atualiza pos_anterior_global em caso de erro
        return 1;
    }

    // Calcula a diferença de pulsos desde a última leitura
    int delta_pulsos = pos_atual - pos_anterior_global;

    // Atualiza a posição anterior para a próxima chamada
    pos_anterior_global = pos_atual;

    // --- Cálculos de RPM e Velocidade ---
    float rpm = ((float)delta_pulsos / ENCODER_RESOLUCAO_4X) / (SAMPLE_INTERVAL_MS / 1000.0) * 60.0;
    rpm = fabsf(rpm); // Valor absoluto

    float perimetro = 2 * PI * RAIO_ANEMOMETRO_M;
    float rps = rpm / 60.0;
    float velocidade_pas_ms = perimetro * rps;
    float velocidade_vento_ms = velocidade_pas_ms * FATOR_ANEMOMETRO;
    float velocidade_vento_kmh = velocidade_vento_ms * 3.6;

    printf("------------------ ANEMÔMETRO ------------------\n");
    printf("Pulsos no intervalo: %d\n", delta_pulsos);
    printf("RPM:                 %.2f\n", rpm);
    printf("Velocidade Vento:    %.2f m/s (%.2f km/h)\n", velocidade_vento_ms, velocidade_vento_kmh);
    printf("--------------------------------------------------\n");
    return velocidade_vento_kmh;
}

// ======================================================== biruta =====================================================================================

// Função map
long map(long x, long in_min, long in_max, long out_min, long out_max) {
  if (in_min == in_max) return out_min;
  if (x < in_min) x = in_min;
  if (x > in_max) x = in_max;
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Função angle_to_direction
const char* angle_to_direction(float angle) {
    if (angle >= 337.5 || angle < 22.5) return "N";
    if (angle >= 22.5  && angle < 67.5) return "NE";
    if (angle >= 67.5  && angle < 112.5) return "E";
    if (angle >= 112.5 && angle < 157.5) return "SE";
    if (angle >= 157.5 && angle < 202.5) return "S";
    if (angle >= 202.5 && angle < 247.5) return "SW";
    if (angle >= 247.5 && angle < 292.5) return "W";
    if (angle >= 292.5 && angle < 337.5) return "NW";
    return "?";
}

/**
 * @brief Lê o sensor da biruta uma vez, calcula o ângulo/direção e imprime.
 * Assume que init_biruta_adc() já foi chamada com sucesso.
 */
float printBirutaReading(void) {

    int adc_raw_value = -1;
    int voltage_mv = 0;

    esp_err_t read_ret = adc_oneshot_read(adc1_handle, BIRUTA_ADC_CHANNEL, &adc_raw_value);

    if (read_ret == ESP_OK) {
        if (do_calibration_biruta && adc1_cali_handle_biruta != NULL) {
            // Se calibrado, converte para mV
            esp_err_t cali_ret = adc_cali_raw_to_voltage(adc1_cali_handle_biruta, adc_raw_value, &voltage_mv);
            if (cali_ret != ESP_OK) {
                 ESP_LOGW(TAG_BIRUTA, "Falha na conversão Calibrada Raw->Voltagem");
                 // Continua com a estimativa se a calibração falhar na conversão
                 voltage_mv = adc_raw_value * 3300 / 4095;
                 return 1;
            }
        } else {
            // Estima a tensão se não calibrado
            voltage_mv = adc_raw_value * 3300 / 4095;
        }

        // --- Mapeamento para Ângulo (0-360 graus) ---
        long angle_mapped = map(adc_raw_value, ADC_MIN_EXPECTED, ADC_MAX_EXPECTED, 0, 359);
        float angle_degrees = (float)angle_mapped;
        const char* direction = angle_to_direction(angle_degrees);

        printf("-------------------- BIRUTA --------------------\n");
        printf("Leitura ADC:     %4d\n", adc_raw_value);
        printf("Tensão Estimada: %4d mV\n", voltage_mv);
        printf("Ângulo:          %3.0f°\n", angle_degrees);
        printf("Direção:         %s\n", direction);
        printf("--------------------------------------------------\n");
        return angle_degrees;
    } else {
        ESP_LOGE(TAG_BIRUTA, "Erro na leitura do ADC da Biruta: %s", esp_err_to_name(read_ret));
        return 1;
    }
}

// ====================================================== LDR =========================================================================================

/**
 * @brief Lê o sensor LDR uma vez, calcula a percentagem e imprime no console.
 *
 * @param adc_handle Handle da unidade ADC já inicializada.
 */
float printLDR(adc_oneshot_unit_handle_t adc_handle) {
    // Verifica se o handle é válido
    if (adc_handle == NULL) {
        ESP_LOGE(TAG_LDR, "Handle ADC inválido em printLDR. A inicialização falhou?");
        vTaskDelay(pdMS_TO_TICKS(1000));
        return 1;
    }

    int leituraLDR = -1; // Valor inicial inválido

    // Lê o valor bruto do ADC
    esp_err_t read_ret = adc_oneshot_read(adc_handle, LDR_ADC_CHANNEL, &leituraLDR);

    if (read_ret == ESP_OK) {
        // Lógica do LDR no divisor de tensão:
        // Mais luz = menor resistência = maior tensão = maior leitura ADC.
        // Convertendo para percentagem (0% escuro, 100% claro)
        float luminosidade_percent = (float)leituraLDR / 4095.0f * 100.0f;

        printf("--------------------- LDR ----------------------\n");
        printf("Leitura ADC:     %d\n", leituraLDR);
        printf("Luminosidade:    %.2f %%\n", luminosidade_percent);
        printf("--------------------------------------------------\n");
        return luminosidade_percent;
    } else {
        ESP_LOGE(TAG_LDR, "Erro na leitura do ADC: %s", esp_err_to_name(read_ret));
        return 1;
    }
}