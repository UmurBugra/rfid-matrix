#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rc522.h"

// Ortak SPI hattı (ESP32 VSPI)
#define PIN_MISO  GPIO_NUM_19
#define PIN_MOSI  GPIO_NUM_23
#define PIN_SCK   GPIO_NUM_18
#define PIN_RST   GPIO_NUM_22   // tüm okuyucularda ortak

// Her okuyucunun SDA pini. Yeni okuyucu eklemek için listeye pin ekleyin.
// Kalan uygun pinler: 16, 17, 27, 32, 33
static const gpio_num_t CS_PINS[] = {
    GPIO_NUM_21,   // Okuyucu 1
    GPIO_NUM_4,    // Okuyucu 2
    GPIO_NUM_26,   // Okuyucu 3
    GPIO_NUM_25,   // Okuyucu 4
};
#define READER_COUNT ((int)(sizeof(CS_PINS) / sizeof(CS_PINS[0])))

#define SPI_CLOCK_HZ       (1 * 1000 * 1000)
#define ANTENNA_SETTLE_MS  5    // anten açıldıktan sonra kartın enerjilenme süresi
#define REMOVE_MISSES      3    // art arda bu kadar okunamazsa kart kaldırıldı sayılır

typedef struct {
    bool ok;                        // okuyucu açılışta cevap verdi mi
    bool present;                   // şu an üzerinde kart var mı
    uint8_t uid[RC522_UID_MAX];
    uint8_t uid_len;
    uint8_t misses;
} reader_state_t;

static rc522_t readers[READER_COUNT];
static reader_state_t state[READER_COUNT];

static void print_uid(const uint8_t *uid, uint8_t len)
{
    for (int i = 0; i < len; i++) {
        printf("%02X%s", uid[i], i + 1 < len ? ":" : "");
    }
}

static void poll_reader(int i)
{
    reader_state_t *s = &state[i];
    uint8_t uid[RC522_UID_MAX];
    uint8_t uid_len = 0;

    rc522_antenna(&readers[i], true);
    vTaskDelay(pdMS_TO_TICKS(ANTENNA_SETTLE_MS));
    esp_err_t e = rc522_read_uid(&readers[i], uid, &uid_len, NULL);
    rc522_antenna(&readers[i], false);

    if (e == ESP_OK) {
        s->misses = 0;
        bool changed = !s->present || uid_len != s->uid_len ||
                       memcmp(uid, s->uid, uid_len) != 0;
        if (changed) {
            memcpy(s->uid, uid, uid_len);
            s->uid_len = uid_len;
            s->present = true;
            printf("[Okuyucu %d] Kart okundu: ", i + 1);
            print_uid(uid, uid_len);
            printf("\n");
        }
    } else if (s->present && ++s->misses >= REMOVE_MISSES) {
        s->present = false;
        printf("[Okuyucu %d] Kart kaldirildi: ", i + 1);
        print_uid(s->uid, s->uid_len);
        printf("\n");
    }
}

void app_main(void)
{
    rc522_bus_config_t bus = {
        .host = SPI3_HOST,
        .miso = PIN_MISO,
        .mosi = PIN_MOSI,
        .sck = PIN_SCK,
        .rst = PIN_RST,
        .clock_hz = SPI_CLOCK_HZ,
    };
    ESP_ERROR_CHECK(rc522_bus_init(&bus, CS_PINS, READER_COUNT, readers));

    int active = 0;
    for (int i = 0; i < READER_COUNT; i++) {
        esp_err_t e = rc522_init(&readers[i]);
        uint8_t ver = rc522_version(&readers[i]);
        state[i].ok = (e == ESP_OK);
        active += state[i].ok;
        printf("Okuyucu %d (SDA GPIO%d): VersionReg 0x%02X -> %s\n",
               i + 1, CS_PINS[i], ver,
               state[i].ok ? "bulundu" : "CEVAP YOK, kablolari kontrol edin!");
    }
    printf("%d/%d okuyucu aktif. Kart bekleniyor...\n", active, READER_COUNT);

    while (true) {
        for (int i = 0; i < READER_COUNT; i++) {
            if (state[i].ok) {
                poll_reader(i);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
