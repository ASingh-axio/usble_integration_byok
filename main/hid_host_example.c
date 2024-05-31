/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"

#include "esp_hidh.h"
#include "esp_hid_gap.h"


//Added libraries of ble above this comment



#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_err.h"
#include "esp_log.h"
#include "usb/usb_host.h"
#include "errno.h"
#include "driver/gpio.h"

#include "hid_host.h"
#include "hid_usage_keyboard.h"

#define APP_QUIT_PIN                GPIO_NUM_0
#define APP_QUIT_PIN_POLL_MS        500

#define READY_TO_UNINSTALL          (HOST_NO_CLIENT | HOST_ALL_FREE)

/* Main char symbol for ENTER key */
#define KEYBOARD_ENTER_MAIN_CHAR    '\r'
/* When set to 1 pressing ENTER will be extending with LineFeed during serial debug output */
#define KEYBOARD_ENTER_LF_EXTEND    1

#define SCAN_DURATION_SECONDS 5

//mod by as open
typedef struct {
    uint8_t key_code;
    uint8_t counter;
    bool initial_delay_passed;
} key_state_t;

#define MAX_KEYS 6
static key_state_t key_states[MAX_KEYS] = {0};
static uint8_t num_keys = 0;
//mod by as close


/**
 * @brief Application Event from USB Host driver
 *
 */
typedef enum {
    HOST_NO_CLIENT = 0x1,
    HOST_ALL_FREE = 0x2,
    DEVICE_CONNECTED = 0x4,
    DEVICE_DISCONNECTED = 0x8,
    DEVICE_ADDRESS_MASK = 0xFF0,
    APP_QUIT_EVENT = 0x10000,

} app_event_t;

/**
 * @brief Key event
 *
 */
typedef struct {
    enum key_state {
        KEY_STATE_PRESSED = 0x00,
        KEY_STATE_RELEASED = 0x01
    } state;
    uint8_t modifier;
    uint8_t key_code;
} key_event_t;

#define USB_EVENTS_TO_WAIT      (DEVICE_CONNECTED | DEVICE_ADDRESS_MASK | DEVICE_DISCONNECTED)

static const char *TAG = "example";

static const char *TAG2 = "ESP_HIDH_DEMO";


static EventGroupHandle_t usb_flags;
static hid_host_device_handle_t hid_device = NULL;
static bool hid_device_connected = false;

hid_host_interface_handle_t keyboard_handle = NULL;

/**
 * @brief Scancode to ascii table
 */
const uint8_t keycode2ascii [113][2] = {
    {0, 0}, /* HID_KEY_NO_PRESS        */
    {0, 0}, /* HID_KEY_ROLLOVER        */
    {0, 0}, /* HID_KEY_POST_FAIL       */
    {0, 0}, /* HID_KEY_ERROR_UNDEFINED */
    {'a', 'A'}, /* HID_KEY_A               */
    {'b', 'B'}, /* HID_KEY_B               */
    {'c', 'C'}, /* HID_KEY_C               */
    {'d', 'D'}, /* HID_KEY_D               */
    {'e', 'E'}, /* HID_KEY_E               */
    {'f', 'F'}, /* HID_KEY_F               */
    {'g', 'G'}, /* HID_KEY_G               */
    {'h', 'H'}, /* HID_KEY_H               */
    {'i', 'I'}, /* HID_KEY_I               */
    {'j', 'J'}, /* HID_KEY_J               */
    {'k', 'K'}, /* HID_KEY_K               */
    {'l', 'L'}, /* HID_KEY_L               */
    {'m', 'M'}, /* HID_KEY_M               */
    {'n', 'N'}, /* HID_KEY_N               */
    {'o', 'O'}, /* HID_KEY_O               */
    {'p', 'P'}, /* HID_KEY_P               */
    {'q', 'Q'}, /* HID_KEY_Q               */
    {'r', 'R'}, /* HID_KEY_R               */
    {'s', 'S'}, /* HID_KEY_S               */
    {'t', 'T'}, /* HID_KEY_T               */
    {'u', 'U'}, /* HID_KEY_U               */
    {'v', 'V'}, /* HID_KEY_V               */
    {'w', 'W'}, /* HID_KEY_W               */
    {'x', 'X'}, /* HID_KEY_X               */
    {'y', 'Y'}, /* HID_KEY_Y               */
    {'z', 'Z'}, /* HID_KEY_Z               */
    {'1', '!'}, /* HID_KEY_1               */
    {'2', '@'}, /* HID_KEY_2               */
    {'3', '#'}, /* HID_KEY_3               */
    {'4', '$'}, /* HID_KEY_4               */
    {'5', '%'}, /* HID_KEY_5               */
    {'6', '^'}, /* HID_KEY_6               */
    {'7', '&'}, /* HID_KEY_7               */
    {'8', '*'}, /* HID_KEY_8               */
    {'9', '('}, /* HID_KEY_9               */
    {'0', ')'}, /* HID_KEY_0               */
    {KEYBOARD_ENTER_MAIN_CHAR, KEYBOARD_ENTER_MAIN_CHAR}, /* HID_KEY_ENTER           */
    {0, 0}, /* HID_KEY_ESC             */
    {'\b', 0}, /* HID_KEY_DEL             */
    {0, 0}, /* HID_KEY_TAB             */
    {' ', ' '}, /* HID_KEY_SPACE           */
    {'-', '_'}, /* HID_KEY_MINUS           */
    {'=', '+'}, /* HID_KEY_EQUAL           */
    {'[', '{'}, /* HID_KEY_OPEN_BRACKET    */
    {']', '}'}, /* HID_KEY_CLOSE_BRACKET   */
    {'\\', '|'}, /* HID_KEY_BACK_SLASH      */
    {'\\', '|'}, /* HID_KEY_SHARP           */  // HOTFIX: for NonUS Keyboards repeat HID_KEY_BACK_SLASH
    {';', ':'}, /* HID_KEY_COLON           */
    {'\'', '"'}, /* HID_KEY_QUOTE           */
    {'`', '~'}, /* HID_KEY_TILDE           */
    {',', '<'}, /* HID_KEY_LESS            */
    {'.', '>'}, /* HID_KEY_GREATER         */
    {'/', '?'}, /* HID_KEY_SLASH           */
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {'\b', 0}, /*HID_KEY_DELETE*/
    {0, 0},
    {0, 0},
    {'R', 'R'}, /*HID_KEY_RIGHT*/
    {'L', 'L'}, /*HID_KEY_LEFT*/
    {'D', 'D'}, /*HID_KEY_DOWN*/
    {'U', 'U'}, /*HID_KEY_UP*/
    {0, 0},
    {'/', '/'}, /*HID_KEY_KEYPAD_DIV*/
    {'*', '*'}, /*HID_KEY_KEYPAD_MUL*/
    {'-', '-'}, /*HID_KEY_KEYPAD_SUB*/
    {'+', '+'}, /*HID_KEY_KEYPAD_ADD*/
    {KEYBOARD_ENTER_MAIN_CHAR, KEYBOARD_ENTER_MAIN_CHAR}, /*HID_KEY_KEYPAD_ENTER*/
    {'1', '1'},
    {'2', '2'},
    {'3', '3'},
    {'4', '4'},
    {'5', '5'},
    {'6', '6'},
    {'7', '7'},
    {'8', '8'},
    {'9', '9'},
    {'0', '0'},
    {'\b', '.'},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0}      /* HID_KEY_F21*/
};

/**
 * @brief Makes new line depending on report output protocol type
 *
 * @param[in] proto Current protocol to output
 */
static void hid_print_new_device_report_header(hid_protocol_t proto)
{
    static hid_protocol_t prev_proto_output = HID_PROTOCOL_NONE;

    if (prev_proto_output != proto) {
        prev_proto_output = proto;
        printf("\r\n");
        /*
        if (proto == HID_PROTOCOL_MOUSE) {
            printf("Mouse\r\n");
        }
        */
        if (proto == HID_PROTOCOL_KEYBOARD) {
            printf("Keyboard\r\n");
        }
        fflush(stdout);
    }
}

/**
 * @brief HID Keyboard modifier verification for capitalization application (right or left shift)
 *
 * @param[in] modifier
 * @return true  Modifier was pressed (left or right shift)
 * @return false Modifier was not pressed (left or right shift)
 *
 */
static inline bool hid_keyboard_is_modifier_shift(uint8_t modifier)
{
    if ((modifier && HID_LEFT_SHIFT) ||
            (modifier && HID_RIGHT_SHIFT)) {
        return true;
    }
    return false;
}

/**
 * @brief HID Keyboard get char symbol from key code
 *
 * @param[in] modifier  Keyboard modifier data
 * @param[in] key_code  Keyboard key code
 * @param[in] key_char  Pointer to key char data
 *
 * @return true  Key scancode converted successfully
 * @return false Key scancode unknown
 */
static inline bool hid_keyboard_get_char(uint8_t modifier,
        uint8_t key_code,
        unsigned char *key_char)
{
    uint8_t mod = (hid_keyboard_is_modifier_shift(modifier)) ? 1 : 0;

    if ((key_code >= HID_KEY_A) && (key_code <= HID_KEY_F21)) {       //modified by as
        *key_char = keycode2ascii[key_code][mod];
    } else {
        // All other key pressed

        return false;
    }

    return true;
}

/**
 * @brief HID Keyboard print char symbol
 *
 * @param[in] key_char  Keyboard char to stdout
 */
static inline void hid_keyboard_print_char(unsigned int key_char)
{
    if (!!key_char) {
        //edited code
        switch (key_char) {
            default:
                putchar(key_char); 
        }
        //putchar(key_char);
#if (KEYBOARD_ENTER_LF_EXTEND)
        if (KEYBOARD_ENTER_MAIN_CHAR == key_char) {
            putchar('\n');
        }
#endif // KEYBOARD_ENTER_LF_EXTEND
        fflush(stdout);
    }
}

/**
 * @brief Key Event. Key event with the key code, state and modifier.
 *
 * @param[in] key_event Pointer to Key Event structure
 *
 */
static void key_event_callback(key_event_t *key_event)
{
    unsigned char key_char;

    hid_print_new_device_report_header(HID_PROTOCOL_KEYBOARD);

    if (KEY_STATE_PRESSED == key_event->state) {
        if (hid_keyboard_get_char(key_event->modifier,
                                  key_event->key_code, &key_char)) {

            hid_keyboard_print_char(key_char);

        }
    }
}

/**
 * @brief Key buffer scan code search.
 *
 * @param[in] src       Pointer to source buffer where to search
 * @param[in] key       Key scancode to search
 * @param[in] length    Size of the source buffer
 */
static inline bool key_found(const uint8_t *const src,
                             uint8_t key,
                             unsigned int length)
{
    for (unsigned int i = 0; i < length; i++) {
        if (src[i] == key) {
            return true;
        }
    }
    return false;
}

/**
 * @brief USB HID Host Keyboard Interface report callback handler
 *
 * @param[in] data    Pointer to input report data buffer
 * @param[in] length  Length of input report data buffer
 */
static void hid_host_keyboard_report_callback(const uint8_t *const data, const int length) {
    hid_keyboard_input_report_boot_t *kb_report = (hid_keyboard_input_report_boot_t *)data;

    if (length < sizeof(hid_keyboard_input_report_boot_t)) {
        return;
    }

    static uint8_t prev_keys[HID_KEYBOARD_KEY_MAX] = {0};
    key_event_t key_event;
    num_keys = 0;

    for (int i = 0; i < HID_KEYBOARD_KEY_MAX; i++) {
        if (prev_keys[i] > HID_KEY_ERROR_UNDEFINED &&
            !key_found(kb_report->key, prev_keys[i], HID_KEYBOARD_KEY_MAX)) {
            key_event.key_code = prev_keys[i];
            key_event.modifier = 0;
            key_event.state = KEY_STATE_RELEASED;
            key_event_callback(&key_event);
            key_states[i].key_code = 0;
            key_states[i].counter = 0;
            key_states[i].initial_delay_passed = false;
        }

        if (kb_report->key[i] > HID_KEY_ERROR_UNDEFINED) {
            key_states[num_keys].key_code = kb_report->key[i];
            key_states[num_keys].counter = 0;
            key_states[num_keys].initial_delay_passed = false;
            num_keys++;
            if (!key_found(prev_keys, kb_report->key[i], HID_KEYBOARD_KEY_MAX)) {
                key_event.key_code = kb_report->key[i];
                key_event.modifier = kb_report->modifier.val;
                key_event.state = KEY_STATE_PRESSED;
                key_event_callback(&key_event);
            }
        }
    }

    memcpy(prev_keys, &kb_report->key, HID_KEYBOARD_KEY_MAX);
}


void hidh_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    esp_hidh_event_t event = (esp_hidh_event_t)id;
    esp_hidh_event_data_t *param = (esp_hidh_event_data_t *)event_data;

    switch (event) {
    case ESP_HIDH_OPEN_EVENT: {
        if (param->open.status == ESP_OK) {
            const uint8_t *bda = esp_hidh_dev_bda_get(param->open.dev);
            ESP_LOGI(TAG, ESP_BD_ADDR_STR " OPEN: %s", ESP_BD_ADDR_HEX(bda), esp_hidh_dev_name_get(param->open.dev));
            esp_hidh_dev_dump(param->open.dev, stdout);
        } else {
            ESP_LOGE(TAG, " OPEN failed!");
        }
        break;
    }
    case ESP_HIDH_BATTERY_EVENT: {
        const uint8_t *bda = esp_hidh_dev_bda_get(param->battery.dev);
        ESP_LOGI(TAG, ESP_BD_ADDR_STR " BATTERY: %d%%", ESP_BD_ADDR_HEX(bda), param->battery.level);
        break;
    }
    case ESP_HIDH_INPUT_EVENT: {
        const uint8_t *bda = esp_hidh_dev_bda_get(param->input.dev);
        static uint8_t prev_keys[HID_KEYBOARD_KEY_MAX] = {0};
        key_event_t key_event;
        num_keys = 0;

        for (int i = 0; i < HID_KEYBOARD_KEY_MAX; i++) {
            if (prev_keys[i] > HID_KEY_ERROR_UNDEFINED &&
                !key_found((&param->input.data[i+1]), prev_keys[i], HID_KEYBOARD_KEY_MAX)) {
                key_event.key_code = prev_keys[i];
                key_event.modifier = 0;
                key_event.state = KEY_STATE_RELEASED;
                key_event_callback(&key_event);
                key_states[i].key_code = 0;
                key_states[i].counter = 0;
                key_states[i].initial_delay_passed = false;
            }

            if (param->input.data[i+1] > HID_KEY_ERROR_UNDEFINED) {
                key_states[num_keys].key_code = param->input.data[i+1];
                key_states[num_keys].counter = 0;
                key_states[num_keys].initial_delay_passed = false;
                num_keys++;
                if (!key_found(prev_keys, param->input.data[i+1], HID_KEYBOARD_KEY_MAX)) {
                    key_event.key_code = param->input.data[i+1];
                    key_event.modifier = param->input.data[0];
                    key_event.state = KEY_STATE_PRESSED;
                    key_event_callback(&key_event);
                }
            }
        }

        memcpy(prev_keys, &param->input.data[1], HID_KEYBOARD_KEY_MAX);

    /*
        int length_id = param->input.length;
        int mod = param->input.data[0];
        int key = param->input.data[1];
        unsigned int key_char; */
        //ESP_LOGI(TAG, ESP_BD_ADDR_STR " INPUT: %8s, MAP: %2u, ID: %3u, Len: %d, Data:", ESP_BD_ADDR_HEX(bda), esp_hid_usage_str(param->input.usage), param->input.map_index, param->input.report_id, param->input.length);
        //ESP_LOG_BUFFER_HEX(TAG, param->input.data, param->input.length);
        //printf("Input event\n");
       /* for(int i = 0; i<length_id; i++){
            printf("%d: ", i);
            printf(" %d ", param->input.data[i]);
        }*/
        //printf("\n");
        /*
        if(param->input.data[0] > 0 && param->input.data[1] > 0){
            //printf(" %d :", param->input.data[1]);
            //printf(" %c \n", param->input.data[1]);
            key_char = keycode2ascii[param->input.data[1]][1];
            //printf("mod %d\n", key_char);
            putchar((unsigned int)key_char);
        }
        else if(param->input.data[1] > 0){
            key_char = keycode2ascii[param->input.data[1]][0];
            //printf("no mod %d\n", key_char);
            putchar((unsigned int)key_char);
        } */
        //printf("")
        //printf("\n");
        //fflush(stdout); 
        break;
    }
    case ESP_HIDH_FEATURE_EVENT: {
        const uint8_t *bda = esp_hidh_dev_bda_get(param->feature.dev);
        ESP_LOGI(TAG, ESP_BD_ADDR_STR " FEATURE: %8s, MAP: %2u, ID: %3u, Len: %d", ESP_BD_ADDR_HEX(bda),
                 esp_hid_usage_str(param->feature.usage), param->feature.map_index, param->feature.report_id,
                 param->feature.length);
        ESP_LOG_BUFFER_HEX(TAG, param->feature.data, param->feature.length);
        break;
    }
    case ESP_HIDH_CLOSE_EVENT: {
        const uint8_t *bda = esp_hidh_dev_bda_get(param->close.dev);
        ESP_LOGI(TAG, ESP_BD_ADDR_STR " CLOSE: %s", ESP_BD_ADDR_HEX(bda), esp_hidh_dev_name_get(param->close.dev));
        break;
    }
    default:
        ESP_LOGI(TAG, "EVENT: %d", event);
        break;
    }
}





void hid_demo_task(void *pvParameters)
{
    size_t results_len = 0;
    esp_hid_scan_result_t *results = NULL;
    ESP_LOGI(TAG, "SCAN...");
    //start scan for HID devices
    esp_hid_scan(SCAN_DURATION_SECONDS, &results_len, &results);
    ESP_LOGI(TAG, "SCAN: %u results", results_len);
    if (results_len) {
        esp_hid_scan_result_t *r = results;
        esp_hid_scan_result_t *cr = NULL;
        while (r) {
            printf("  %s: " ESP_BD_ADDR_STR ", ", (r->transport == ESP_HID_TRANSPORT_BLE) ? "BLE" : "BT ", ESP_BD_ADDR_HEX(r->bda));
            printf("RSSI: %d, ", r->rssi);
            printf("USAGE: %s, ", esp_hid_usage_str(r->usage));
#if CONFIG_BT_BLE_ENABLED
            if (r->transport == ESP_HID_TRANSPORT_BLE) {
                cr = r;
                printf("APPEARANCE: 0x%04x, ", r->ble.appearance);
                printf("ADDR_TYPE: '%s', ", ble_addr_type_str(r->ble.addr_type));
            }
#endif /* CONFIG_BT_BLE_ENABLED */
#if CONFIG_BT_HID_HOST_ENABLED
            if (r->transport == ESP_HID_TRANSPORT_BT) {
                cr = r;
                printf("COD: %s[", esp_hid_cod_major_str(r->bt.cod.major));
                esp_hid_cod_minor_print(r->bt.cod.minor, stdout);
                printf("] srv 0x%03x, ", r->bt.cod.service);
                print_uuid(&r->bt.uuid);
                printf(", ");
            }
#endif /* CONFIG_BT_HID_HOST_ENABLED */
            printf("NAME: %s ", r->name ? r->name : "");
            printf("\n");
            r = r->next;
        }
        if (cr) {
            //open the last result
            esp_hidh_dev_open(cr->bda, cr->transport, cr->ble.addr_type);
        }
        
        //free the results
        esp_hid_scan_results_free(results);
    }
    vTaskDelete(NULL);
}




static void handle_repeated_keys(void *arg) {
    const TickType_t xInitialDelay = pdMS_TO_TICKS(300);  // Initial delay of 500 ms
    const TickType_t xRepeatDelay = pdMS_TO_TICKS(50);  // Repeat delay of 200 ms

    while (1) {
        for (int i = 0; i < num_keys; i++) {
            if (key_states[i].key_code > HID_KEY_ERROR_UNDEFINED) {
                if (!key_states[i].initial_delay_passed) {
                    // Wait for the initial delay before starting repeated key presses
                    vTaskDelay(xInitialDelay);
                    key_states[i].initial_delay_passed = true;
                } else {
                    key_states[i].counter++;
                    if (key_states[i].counter >= 5) {  // Adjust the counter threshold as needed
                        key_event_t key_event;
                        key_event.key_code = key_states[i].key_code;
                        key_event.modifier = 0;
                        key_event.state = KEY_STATE_PRESSED;
                        key_event_callback(&key_event);
                        key_states[i].counter = 0;  // Reset counter after printing
                    }
                }
            }
        }
        vTaskDelay(xRepeatDelay);
    }
}

/**
 * @brief USB HID Host event callback. Handle such event as device connection and removing
 *
 * @param[in] event  HID device event
 * @param[in] arg    Pointer to arguments, does not used
 */
static void hid_host_event_callback(const hid_host_event_t *event, void *arg)
{
    if (event->event == HID_DEVICE_CONNECTED) {
        // Obtained USB device address is placed after application events
        xEventGroupSetBits(usb_flags, DEVICE_CONNECTED | (event->device.address << 4));
    } else if (event->event == HID_DEVICE_DISCONNECTED) {
        xEventGroupSetBits(usb_flags, DEVICE_DISCONNECTED);
    }
}

/**
 * @brief USB HID Host interface callback
 *
 * @param[in] event  HID interface event
 * @param[in] arg    Pointer to arguments, does not used
 */
static void hid_host_interface_event_callback(const hid_host_interface_event_t *event, void *arg)
{
    switch (event->event) {
    case HID_DEVICE_INTERFACE_INIT:
        ESP_LOGI(TAG, "Interface number %d, protocol %s",
                 event->interface.num,
                 (event->interface.proto == HID_PROTOCOL_KEYBOARD)
                 ? "Keyboard"
                 : "Mouse");

        if (event->interface.proto == HID_PROTOCOL_KEYBOARD) {
            const hid_host_interface_config_t hid_keyboard_config = {
                .proto = HID_PROTOCOL_KEYBOARD,
                .callback = hid_host_keyboard_report_callback,
            };

            hid_host_claim_interface(&hid_keyboard_config, &keyboard_handle);
        }

        break;
    case HID_DEVICE_INTERFACE_TRANSFER_ERROR:
        ESP_LOGD(TAG, "Interface number %d, transfer error",
                 event->interface.num);
        break;

    case HID_DEVICE_INTERFACE_CLAIM:
    case HID_DEVICE_INTERFACE_RELEASE:
        // ... do nothing here for now
        break;

    default:
        ESP_LOGI(TAG, "%s Unhandled event %X, Interface number %d",
                 __FUNCTION__,
                 event->event,
                 event->interface.num);
        break;
    }
}

/**
 * @brief Handle common USB host library events
 *
 * @param[in] args  Pointer to arguments, does not used
 */
static void handle_usb_events(void *args)
{
    while (1) {
        uint32_t event_flags;
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);

        // Release devices once all clients has deregistered
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            usb_host_device_free_all();
            xEventGroupSetBits(usb_flags, HOST_NO_CLIENT);
        }
        // Give ready_to_uninstall_usb semaphore to indicate that USB Host library
        // can be deinitialized, and terminate this task.
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
            xEventGroupSetBits(usb_flags, HOST_ALL_FREE);
        }
    }

    vTaskDelete(NULL);
}

static bool wait_for_event(EventBits_t event, TickType_t timeout)
{
    return xEventGroupWaitBits(usb_flags, event, pdTRUE, pdTRUE, timeout) & event;
}



void vUSBEventHandlerTask(void *pvParameters) {
    while (gpio_get_level(APP_QUIT_PIN) != 0) {
        EventBits_t event = xEventGroupWaitBits(
            usb_flags, 
            USB_EVENTS_TO_WAIT, 
            pdTRUE, 
            pdFALSE, 
            pdMS_TO_TICKS(APP_QUIT_PIN_POLL_MS)
        );

        if (event & DEVICE_CONNECTED) {
            xEventGroupClearBits(usb_flags, DEVICE_CONNECTED);
            hid_device_connected = true;
        }

        if (event & DEVICE_ADDRESS_MASK) {
            xEventGroupClearBits(usb_flags, DEVICE_ADDRESS_MASK);

            const hid_host_device_config_t hid_host_device_config = {
                .dev_addr = (event & DEVICE_ADDRESS_MASK) >> 4,
                .iface_event_cb = hid_host_interface_event_callback,
                .iface_event_arg = NULL,
            };

            ESP_ERROR_CHECK(hid_host_install_device(&hid_host_device_config, &hid_device));
        }

        if (event & DEVICE_DISCONNECTED) {
            xEventGroupClearBits(usb_flags, DEVICE_DISCONNECTED);

            hid_host_release_interface(keyboard_handle);

            ESP_ERROR_CHECK(hid_host_uninstall_device(hid_device));

            hid_device_connected = false;
        }

        vTaskDelay(pdMS_TO_TICKS(APP_QUIT_PIN_POLL_MS)); // Optional: Add a small delay to yield CPU time
    }
    xEventGroupSetBits(usb_flags, APP_QUIT_EVENT);
    vTaskDelete(NULL); // Delete the task if the loop exits
}



void app_main(void) {
    TaskHandle_t usb_events_task_handle;
    TaskHandle_t repeated_keys_task_handle;

    BaseType_t task_created;


    esp_err_t ret;
#if HID_HOST_MODE == HIDH_IDLE_MODE
    ESP_LOGE(TAG, "Please turn on BT HID host or BLE!");
    return;
#endif
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
    ESP_LOGI(TAG, "setting hid gap, mode:%d", HID_HOST_MODE);
    ESP_ERROR_CHECK( esp_hid_gap_init(HID_HOST_MODE) );
#if CONFIG_BT_BLE_ENABLED
    ESP_ERROR_CHECK( esp_ble_gattc_register_callback(esp_hidh_gattc_event_handler) );
#endif /* CONFIG_BT_BLE_ENABLED */
    esp_hidh_config_t config = {
        .callback = hidh_callback,
        .event_stack_size = 4096,
        .callback_arg = NULL,
    };
    ESP_ERROR_CHECK( esp_hidh_init(&config) );

    xTaskCreate(&hid_demo_task, "hid_task", 6 * 1024, NULL, 2, NULL);    

    const gpio_config_t input_pin = {
        .pin_bit_mask = BIT64(APP_QUIT_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&input_pin));

    ESP_LOGI(TAG, "HID HOST example");

    usb_flags = xEventGroupCreate();
    assert(usb_flags);

    const usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1
    };

    ESP_ERROR_CHECK(usb_host_install(&host_config));
    task_created = xTaskCreate(handle_usb_events, "usb_events", 4096, NULL, 2, &usb_events_task_handle);
    assert(task_created);

    task_created = xTaskCreate(handle_repeated_keys, "repeated_keys", 2048, NULL, 2, &repeated_keys_task_handle);
    assert(task_created);

    const hid_host_driver_config_t hid_host_config = {
        .create_background_task = true,
        .task_priority = 5,
        .stack_size = 4096,
        .core_id = 0,
        .callback = hid_host_event_callback,
        .callback_arg = NULL
    };

    ESP_ERROR_CHECK(hid_host_install(&hid_host_config));
    
    // Create the USB event handler task
    task_created = xTaskCreate(
        vUSBEventHandlerTask, 
        "USBEventHandler", 
        4096, 
        NULL, 
        2, 
        NULL
    );
    assert(task_created == pdPASS);

    xEventGroupWaitBits(usb_flags, APP_QUIT_EVENT, pdTRUE, pdFALSE, portMAX_DELAY);

    if (hid_device_connected) {
        ESP_LOGI(TAG, "Uninitializing HID Device");
        hid_host_release_interface(keyboard_handle);
       // hid_host_release_interface(mouse_handle);
        ESP_ERROR_CHECK(hid_host_uninstall_device(hid_device));
        hid_device_connected = false;
    }

    ESP_LOGI(TAG, "Uninitializing USB");
    ESP_ERROR_CHECK(hid_host_uninstall());
    wait_for_event(READY_TO_UNINSTALL, portMAX_DELAY);
    ESP_ERROR_CHECK(usb_host_uninstall());
    vTaskDelete(usb_events_task_handle);
    vTaskDelete(repeated_keys_task_handle);
    vEventGroupDelete(usb_flags);
    ESP_LOGI(TAG, "Done");
    
}
