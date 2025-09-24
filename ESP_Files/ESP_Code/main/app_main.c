#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "protocol_examples_common.h"
#include "esp_log.h"
#include "esp_timer.h"

// HTTP istemcisi
#include "esp_tls.h"
#include "esp_crt_bundle.h"
#include "esp_tls_errors.h"
#include "esp_http_client.h"


// SNTP zaman senkronizasyonu
#include "esp_sntp.h"
#include <time.h>
#include <sys/time.h>

#include "cJSON.h"
#include "esp_log.h"
#include "esp_err.h"

// FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// GPIO / I2C
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_err.h"

// BME280
#include "bme280.h"

// MQTT
#include "mqtt_client.h"

#define SDA_PIN         GPIO_NUM_40
#define SCL_PIN         GPIO_NUM_39
#define PIR_INPUT_PIN   GPIO_NUM_17
#define ROLE_OUTPUT_PIN GPIO_NUM_18

#define TAG_BME280       "BME280"
// #define I2C_MASTER_NUM   0
#define I2C_MASTER_ACK   0
#define I2C_MASTER_NACK  1

#define MQTT_BROKER_URI  "mqtt://51.21.152.38:1883"

#define MY_DEVICE_ID "SN61"

static volatile int g_current_mode = 0;   // Gelen /setmode "current_mode"
static volatile int g_timeSeconds = 0;    // Gelen /setmode "time" parametresi (örn. 30 sn) 
static volatile bool g_buzzerEnabled = false;
static volatile bool g_buttonEnabled = false;
static int cnt = 0; // MQTT publish sayacı

static esp_mqtt_client_handle_t s_mqtt_client = NULL; // MQTT istemcisi

static void i2c_master_init(void)
{
    i2c_config_t i2c_config = {
        .mode           = I2C_MODE_MASTER,
        .sda_io_num     = SDA_PIN,
        .scl_io_num     = SCL_PIN,
        .sda_pullup_en  = GPIO_PULLUP_ENABLE,
        .scl_pullup_en  = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 1000000
    };

    i2c_param_config(I2C_NUM_0, &i2c_config);
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
}

s8 BME280_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    s32 iError = BME280_INIT_VALUE;
    esp_err_t espRc;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write(cmd, reg_data, cnt, true);
    i2c_master_stop(cmd);

    espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
    if (espRc == ESP_OK) {
        iError = SUCCESS;
    } else {
        iError = FAIL;
    }

    i2c_cmd_link_delete(cmd);
    return (s8)iError;
}

s8 BME280_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    s32 iError = BME280_INIT_VALUE;
    esp_err_t espRc;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);

    if (cnt > 1) {
        i2c_master_read(cmd, reg_data, cnt - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, reg_data + cnt - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
    if (espRc == ESP_OK) {
        iError = SUCCESS;
    } else {
        iError = FAIL;
    }

    i2c_cmd_link_delete(cmd);
    return (s8)iError;
}

void BME280_delay_msek(u32 msek)
{
    vTaskDelay(msek / portTICK_PERIOD_MS);
}

// publish_current_mode -> device_id string
static void publish_current_mode(int current_mode)
{
    if (!s_mqtt_client) return;

    char msg[128];
    snprintf(msg, sizeof(msg),
             "{\"device_id\":\"%s\", \"mode_set\":true, \"current_mode\":%d}",
             MY_DEVICE_ID, current_mode);

    esp_mqtt_client_publish(s_mqtt_client, "/getmode", msg, 0, 1, 0);
    // ESP_LOGI("MQTT", "Mode bildirildi -> /getmode : %s", msg);
}

static void mode_handler_task(void *params)
{
    static bool role_on = false;
    static int64_t off_timestamp_us = 0;   // role u kapatacağımız zaman (mikrosaniye cinsinden)
    static int last_mode = 0;             // önceki mod, mod değişimini anlamak için

    while (true) {
        int pir_value = gpio_get_level(PIR_INPUT_PIN);
        int64_t now_us = esp_timer_get_time(); // Mikro saniye cinsinden zaman

        // Eğer current_mode değiştiyse, mod geçişlerinde yapacaklarımız:
        if (g_current_mode != last_mode) {
            // ESP_LOGI("MODE_HANDLER", "Mod değişti: %d -> %d", last_mode, g_current_mode);

            // Mod geçişinde role kapatıp sıfırlayabilirsin (isteğe bağlı)
            gpio_set_level(ROLE_OUTPUT_PIN, 0);
            role_on = false;
            off_timestamp_us = 0;
            if(g_current_mode == 2) {
                cnt = 1; // MQTT publish sayacını sıfırla
            }
            // Zamanlama moduna yeni geçildiyse: (4)
            if (g_current_mode == 3) {
                // ESP_LOGI("MODE_HANDLER", "Zamanlama moduna geçildi. %d sn açik kalacak", g_timeSeconds);
                // Role aç
                gpio_set_level(ROLE_OUTPUT_PIN, 1);
                role_on = true;
                // Kapanma zamanını şimdiki zamana ekle
                off_timestamp_us = now_us + (int64_t)g_timeSeconds * 1000000LL;
            }
        }
  
        // Mevcut moda göre davranış:
        switch (g_current_mode) {
        case 1: // Otomatik mod
            if (pir_value == 1) {
                // Hareket algılandı
                if (!role_on) {
                    // Role kapalıysa aç
                    gpio_set_level(ROLE_OUTPUT_PIN, 1);
                    role_on = true;
                } else {
                    // ESP_LOGI("AUTO_MODE", "PIR=1 -> Role zaten açık, süre tazelendi");
                }
                // Her harekette kapanma süresini tazele:
                off_timestamp_us = now_us + (int64_t)g_timeSeconds * 1000000LL;
            }

            // Role açıksa ve kapanma zamanı geldiyse kapat
            if (role_on && (now_us > off_timestamp_us)) {
                gpio_set_level(ROLE_OUTPUT_PIN, 0);
                role_on = false;
                // ESP_LOGI("AUTO_MODE", "Süre doldu -> Role Kapatildi");
            }
            break;

        case 2: // Güvenlik mod (örnek: motion -> /alert)
            if (pir_value == 1) {
                // Motion alert
                // ESP_LOGI("SECURITY_MODE", "PIR=1 -> ALARM! /alert topic gönderiliyor...");
                if(cnt)
                {
                    if (s_mqtt_client) {
                        esp_mqtt_client_publish(s_mqtt_client, "/alert", "{\"motion\":true}", 0, 1, 0);
                    }
                    cnt = 0; // publish sayacını sıfırla;
                }
            }
            break;

        case 3: // Zamanlama mod
            if (role_on && (now_us > off_timestamp_us)) {
                gpio_set_level(ROLE_OUTPUT_PIN, 0);
                role_on = false;
                // ESP_LOGI("TIMER_MODE", "%d sn süre doldu -> Role Kapatildi.", g_timeSeconds);
            }
            break;

        default:
            // Diğer modlar
            break;
        }

        last_mode = g_current_mode; // en sonda güncelle
        vTaskDelay(pdMS_TO_TICKS(1000)); // 1 saniyede bir kontrol
    }
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t) event_data;
    esp_mqtt_client_handle_t client = event->client;

    switch (event_id) {

    case MQTT_EVENT_CONNECTED:
        // ESP_LOGI("MQTT", "MQTT connected.");
        // /setmode abonesi ol
        esp_mqtt_client_subscribe(client, "/setmode", 1);
        esp_mqtt_client_subscribe(client, "/button", 1);
        break;

    case MQTT_EVENT_DATA: {
        // ESP_LOGI("MQTT", "MQTT_EVENT_DATA");

        // Hedef topic ve payload buffer'ları
        char topic[128];
        char payload[512];

        int topic_len = event->topic_len;
        int data_len  = event->data_len;

        // topic'i kopyala
        if (topic_len < sizeof(topic)) {
            memcpy(topic, event->topic, topic_len);
            topic[topic_len] = '\0';
        }

        // payload'u kopyala
        if (data_len < sizeof(payload)) {
            memcpy(payload, event->data, data_len);
            payload[data_len] = '\0';
        }

        // ESP_LOGI("MQTT", "Topic: %s", topic);
        // ESP_LOGI("MQTT", "Payload: %s", payload);

        // /setmode geldiyse parse et
        if (strcmp(topic, "/setmode") == 0) {
            cJSON *root = cJSON_Parse(payload);
            if (root == NULL) {
                // ESP_LOGE("MQTT", "JSON parse error");
                break;
            }

            // device_id’yi string olarak al
            cJSON *deviceIdItem = cJSON_GetObjectItem(root, "device_id");
            if (!cJSON_IsString(deviceIdItem)) {
                // ESP_LOGW("MQTT", "device_id not found or not a string");
                cJSON_Delete(root);
                break;
            }
            char device_id_str[32];
            strncpy(device_id_str, deviceIdItem->valuestring, sizeof(device_id_str) - 1);
            device_id_str[sizeof(device_id_str) - 1] = '\0';

            // 2) Şimdiki mod
            cJSON *currentModeItem = cJSON_GetObjectItem(root, "current_mode");
            if (!cJSON_IsNumber(currentModeItem)) {
                // ESP_LOGW("MQTT", "current_mode not found or not a number");
                cJSON_Delete(root);
                break;
            }
            int current_mode = currentModeItem->valueint;
 
            //int duration = 0;
            int timeout = 0;
            // 3) settings içinden const_time, buzzer vb.
            cJSON *settings = cJSON_GetObjectItem(root, "settings");
            if (settings && cJSON_IsObject(settings)) {
                cJSON *timeItem = cJSON_GetObjectItem(settings, "timeout");
                if (cJSON_IsString(timeItem) && timeItem->valuestring != NULL) {
                    timeout = atoi(timeItem->valuestring);
                    g_timeSeconds = timeout * 60;
                }
                cJSON *durationItem = cJSON_GetObjectItem(settings, "duration");
                if (cJSON_IsString(durationItem) && durationItem->valuestring != NULL) {
                    g_timeSeconds = atoi(durationItem->valuestring); 
                    g_timeSeconds = g_timeSeconds * 60;
                }
                cJSON *buzzerItem = cJSON_GetObjectItem(settings, "soundAlarm");
                if (cJSON_IsBool(buzzerItem)) {
                    g_buzzerEnabled = cJSON_IsTrue(buzzerItem);
                }
            }

            // 4) device_id kontrolü -> Bu mesaj bu cihaza mı ait?
            if (strcmp(device_id_str, MY_DEVICE_ID) == 0) {
                // ESP_LOGI("MQTT", "Gelen ayarlar BU cihaza ait!");

                // Artık sadece global değişkeni güncelleyelim
                g_current_mode = current_mode;
                // ESP_LOGI("MQTT", "g_current_mode %d olarak ayarlandı", g_current_mode);

                // Hangi moda geçiliyorsa /getmode gönderelim
                publish_current_mode(g_current_mode);

            }
            else{
                    // Başka cihaza ait mesaj, yok say
                    // ESP_LOGI("MQTT", "Gelen mesaj bu cihaza ait değil (device_id=%d)", device_id_str);
                }

                // JSON serbest bırak
                cJSON_Delete(root);
        }
        else if (strcmp(topic, "/button") == 0)
        {
            cJSON *root = cJSON_Parse(payload);
            if (root == NULL) {
                // ESP_LOGE("MQTT", "JSON parse error");
                break;
            }
            // 1) Cihaz kimliğini ve Button durumunu al
            cJSON *deviceIdItem = cJSON_GetObjectItem(root, "device_id");
            cJSON *buttonItem = cJSON_GetObjectItem(root, "state");

            if (cJSON_IsString(deviceIdItem) && cJSON_IsBool(buttonItem)) {
                char device_id_str[32];
                strncpy(device_id_str, deviceIdItem->valuestring, sizeof(device_id_str) - 1);
                device_id_str[sizeof(device_id_str) - 1] = '\0';
    
                bool button_state = cJSON_IsTrue(buttonItem);
    
                // yine kıyas:
                if (strcmp(device_id_str, MY_DEVICE_ID) == 0) {
                    g_buttonEnabled = button_state;
                    gpio_set_level(ROLE_OUTPUT_PIN, g_buttonEnabled ? 1 : 0);
                }
            }
            cJSON_Delete(root);
        }    
    }
    break;

    case MQTT_EVENT_DISCONNECTED:
        // ESP_LOGI("MQTT", "MQTT disconnected.");
        break;

    case MQTT_EVENT_PUBLISHED:
        // ESP_LOGI("MQTT", "Message published. msg_id=%d", event->msg_id);
        break;

    default:
        break;
    }
}

static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
    };

    s_mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(s_mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(s_mqtt_client);
}

void BME280_Task(void *params)
{
    struct bme280_t bme280 = {
        .bus_write  = BME280_I2C_bus_write,
        .bus_read   = BME280_I2C_bus_read,
        .dev_addr   = BME280_I2C_ADDRESS1, // Gerekirse BME280_I2C_ADDRESS2'yi de deneyebilirsin
        .delay_msec = BME280_delay_msek
    };

    s32 com_rslt;
    s32 v_uncomp_pressure_s32;
    s32 v_uncomp_temperature_s32;
    s32 v_uncomp_humidity_s32;

    // BME280 başlatma
    com_rslt  = bme280_init(&bme280);

    // Oversampling ayarları
    com_rslt += bme280_set_oversamp_pressure(BME280_OVERSAMP_16X);
    com_rslt += bme280_set_oversamp_temperature(BME280_OVERSAMP_2X);
    com_rslt += bme280_set_oversamp_humidity(BME280_OVERSAMP_1X);

    // Filtre ve standby
    com_rslt += bme280_set_standby_durn(BME280_STANDBY_TIME_1_MS);
    com_rslt += bme280_set_filter(BME280_FILTER_COEFF_16);

    // Normal mod
    com_rslt += bme280_set_power_mode(BME280_NORMAL_MODE);

    if (com_rslt == SUCCESS) {
        while (true) {
            // Ölçüm arası gecikme
            vTaskDelay(pdMS_TO_TICKS(40));
            // vTaskDelay(40 / portTICK_PERIOD_MS);

            // Ham değerleri oku
            com_rslt = bme280_read_uncomp_pressure_temperature_humidity(
                &v_uncomp_pressure_s32,
                &v_uncomp_temperature_s32,
                &v_uncomp_humidity_s32
            );

            if (com_rslt == SUCCESS) {
                // Double olarak hesapla
                double temp = bme280_compensate_temperature_double(v_uncomp_temperature_s32);
                double press = bme280_compensate_pressure_double(v_uncomp_pressure_s32) / 100.0; // Pa -> hPa
                double hum = bme280_compensate_humidity_double(v_uncomp_humidity_s32);

                // ESP_LOGI(TAG_BME280, "Temperature: %.2f C, Pressure: %.2f hPa, Humidity: %.2f%%",
                //          temp, press, hum);

                // MQTT üzerinden gönder
                if (s_mqtt_client) {
                    char sensor_json[256];

                    // UTC zaman al
                    time_t now;
                    struct tm timeinfo;
                    time(&now);
                    gmtime_r(&now, &timeinfo);  // UTC zamanı

                    char timestamp[32];
                    strftime(timestamp, sizeof(timestamp), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);

                    // BME280_Task içi -> device_id string
                    snprintf(sensor_json, sizeof(sensor_json),
                            "{"
                            "\"timestamps\":\"%s\","
                            "\"device_id\":\"%s\","
                            "\"temperature\":%.2f,"
                            "\"rh\":%.2f,"
                            "\"pressure\":%.2f"
                            "}",
                            timestamp,
                            MY_DEVICE_ID,  // string -> "1"
                            temp, hum, press);

                    // MQTT publish
                    esp_mqtt_client_publish(s_mqtt_client,
                                            "/sensor_data",
                                            sensor_json,
                                            0,
                                            1,
                                            0);

                    // ESP_LOGI(TAG_BME280, "MQTT publish -> /sensor_data : %s", sensor_json);
                }


            } else {
                // ESP_LOGE(TAG_BME280, "Measure error. Code: %d", com_rslt);
            }
            vTaskDelay(pdMS_TO_TICKS(30000));
        }
    } else {
        // ESP_LOGE(TAG_BME280, "Init or config error. Code: %d", com_rslt);
        vTaskDelete(NULL);
    }
}

static void IO_init(void)
{
    // Giriş pini konfigürasyonu
    gpio_config_t io_conf_in = {
        .pin_bit_mask  = (1ULL << PIR_INPUT_PIN),
        .mode          = GPIO_MODE_INPUT,
        .pull_up_en    = GPIO_PULLUP_DISABLE,
        .pull_down_en  = GPIO_PULLDOWN_ENABLE,
        .intr_type     = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf_in);

    // Çıkış pini konfigürasyonu
    gpio_config_t io_conf_out = {
        .pin_bit_mask  = (1ULL << ROLE_OUTPUT_PIN),
        .mode          = GPIO_MODE_OUTPUT,
        .pull_up_en    = GPIO_PULLUP_DISABLE,
        .pull_down_en  = GPIO_PULLDOWN_DISABLE,
        .intr_type     = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf_out);
}

static void initialize_sntp(void)
{
    ESP_LOGI("SNTP", "SNTP başlatılıyor...");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");  // NTP sunucusu
    sntp_init();
}

static void wait_for_time_sync(void)
{
    time_t now = 0;
    struct tm timeinfo = { 0 };
    int retry = 0;
    const int retry_count = 10;

    while (timeinfo.tm_year < (2020 - 1900) && ++retry < retry_count) {
        ESP_LOGI("SNTP", "Zaman senkronizasyonu bekleniyor (%d)...", retry);
        vTaskDelay(pdMS_TO_TICKS(2000));
        time(&now);
        localtime_r(&now, &timeinfo);
    }

    if (timeinfo.tm_year >= (2020 - 1900)) {
        ESP_LOGI("SNTP", "Zaman senkronize edildi: %s", asctime(&timeinfo));
    } else {
        ESP_LOGW("SNTP", "Zaman senkronizasyonu başarısız.");
    }
}

static char g_bearer_token[256] = {0};  // Login sonucunda gelen Bearer token

// Bu fonksiyon, login yanıtındaki "token" alanını parse edip g_bearer_token'e yazar
static bool parse_token_from_response(const char *json_str, char *out_token, size_t out_size)
{
    cJSON *root = cJSON_Parse(json_str);
    if (!root) {
        // ESP_LOGE("HTTP_CLIENT", "parse_token_from_response: JSON parse error");
        return false;
    }

    // Eğer sunucudan gelen alan "token" ise:
    cJSON *token_item = cJSON_GetObjectItem(root, "token");
    if (cJSON_IsString(token_item)) {
        strncpy(out_token, token_item->valuestring, out_size - 1);
        out_token[out_size - 1] = '\0';
        cJSON_Delete(root);
        return true;
    }

    cJSON_Delete(root);
    return false;
}

static bool do_login_request(const char *email, const char *password)
{
    ESP_LOGI("HTTP_CLIENT", "do_login_request başlıyor...");

    // POST body
    char post_data[128];
    snprintf(post_data, sizeof(post_data),
             "{\"email\":\"%s\",\"password\":\"%s\"}",
             email, password);

    esp_http_client_config_t config = {
        .url = "https://www.iotconnecttr.com/api/login",
        .method = HTTP_METHOD_POST,
        .crt_bundle_attach = esp_crt_bundle_attach, // TLS doğrulaması
        .skip_cert_common_name_check = true, // Gerekirse devre dışı
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_http_client_set_header(client, "Accept", "application/json");
    esp_http_client_set_header(client, "Content-Type", "application/json");

    // Body
    esp_http_client_set_post_field(client, post_data, strlen(post_data));

    // İstek
    esp_err_t err = esp_http_client_perform(client);
    if (err != ESP_OK) {
        ESP_LOGE("HTTP_CLIENT", "Login request hatası: %s", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        return false;
    }

    int status = esp_http_client_get_status_code(client);
    int content_length = esp_http_client_get_content_length(client);
    ESP_LOGI("HTTP_CLIENT", "login status = %d, content_length = %d", status, content_length);

    // Cevap body oku
    char response_buf[512] = {0};
    int read_len = esp_http_client_read(client, response_buf, sizeof(response_buf) - 1);
    if (read_len > 0) {
        response_buf[read_len] = '\0';
        ESP_LOGI("HTTP_CLIENT", "login response: %s", response_buf);

        // Cevap parse → token çek
        bool ok = parse_token_from_response(response_buf, g_bearer_token, sizeof(g_bearer_token));
        if (!ok) {
            ESP_LOGE("HTTP_CLIENT", "Token parse edilemedi veya 'token' alanı yok");
            esp_http_client_cleanup(client);
            return false;
        }
        ESP_LOGI("HTTP_CLIENT", "Bearer token: %s", g_bearer_token);
    }

    esp_http_client_cleanup(client);
    return (status == 200);
}

// static bool do_reset_mode_request(const char *serial_number)
// {
//     ESP_LOGI("HTTP_CLIENT", "do_reset_mode_request başlıyor...");

//     // POST body
//     char post_data[256];
//     snprintf(post_data, sizeof(post_data),
//     "{\"serial_number\":\"%s\"}",
//     serial_number);


//     esp_http_client_config_t config = {
//         .url = "https://www.iotconnecttr.com/api/device/reset-mode",
//         .method = HTTP_METHOD_POST,
//         .crt_bundle_attach = esp_crt_bundle_attach,
//         .skip_cert_common_name_check = true, // Gerekirse devre dışı
//     };

//     esp_http_client_handle_t client = esp_http_client_init(&config);
//     esp_http_client_set_header(client, "Content-Type", "application/json");

//     // Bearer token: "Authorization: Bearer eyJhbGci..."
//     char auth_header[512];
//     snprintf(auth_header, sizeof(auth_header), "Bearer %s", g_bearer_token);
//     esp_http_client_set_header(client, "Authorization", auth_header);

//     // Body
//     esp_http_client_set_post_field(client, post_data, strlen(post_data));

//     esp_err_t err = esp_http_client_perform(client);
//     if (err != ESP_OK) {
//         ESP_LOGE("HTTP_CLIENT", "reset-mode hatasi: %s", esp_err_to_name(err));
//         esp_http_client_cleanup(client);
//         return false;
//     }

//     int status = esp_http_client_get_status_code(client);
//     int content_length = esp_http_client_get_content_length(client);
//     ESP_LOGI("HTTP_CLIENT", "reset-mode status = %d, content_length = %d", status, content_length);

//     // Yanıt oku (örnek: {"message":"Device mode and settings saved after reset."})
//     char response_buf[512] = {0};
//     int read_len = esp_http_client_read(client, response_buf, sizeof(response_buf) - 1);
//     if (read_len > 0) {
//         response_buf[read_len] = '\0';
//         ESP_LOGI("HTTP_CLIENT", "reset-mode response: %s", response_buf);
//     }

//     esp_http_client_cleanup(client);
//     return (status == 200);
// }

void app_main(void)
{
    // NVS init
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Ağ yapılandırma
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Wi-Fi'ye bağlan (menuconfig -> Example Connection ayarları)
    ESP_ERROR_CHECK(example_connect());

    // SNTP başlat ve senkronizasyonu bekle
    ESP_LOGI("MAIN", "Wi-Fi bağlantısı tamamlandı");
    initialize_sntp();
    ESP_LOGI("MAIN", "initialize_sntp çağrıldı");
    wait_for_time_sync();
    ESP_LOGI("MAIN", "wait_for_time_sync tamamlandı");
    
    bool login_ok = do_login_request("serdarilhan223@gmail.com", "abdullahilhan");
    if (!login_ok) {
        ESP_LOGE("MAIN", "Login başarisiz, ilerlenemiyor");
    } else {
        ESP_LOGI("MAIN", "Login başarili, token alindi = %s", g_bearer_token);
    }

    // bool reset_ok = do_reset_mode_request(MY_DEVICE_ID);
    // if (reset_ok) {
    //     ESP_LOGI("MAIN", "reset-mode başarılı!");
    // } else {
    //     ESP_LOGE("MAIN", "reset-mode isteği başarısız!");
    // }
    
    // MQTT başlat
    mqtt_app_start();

    // I2C başlat
    i2c_master_init();

    //GPIO başlat
    IO_init();

    // BME280 ölçüm görevi
    xTaskCreate(BME280_Task, "BME280_Task", 5 * 1024, NULL, 5, NULL);

    // Mode handling görevi
    xTaskCreate(mode_handler_task, "mode_handler_task", 5 * 1024, NULL, 5, NULL);
}
