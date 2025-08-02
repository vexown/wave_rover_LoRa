#include "persistent_params.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "persistent_params";
#define PARAMS_NAMESPACE "params"

typedef struct param_node 
{
    char* name;
    char* value;
    struct param_node* next;
} param_node_t;

static param_node_t* head = NULL;

static esp_err_t load_param_from_nvs(param_node_t* node)
{
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open(PARAMS_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) 
    {
        ESP_LOGE(TAG, "Error opening NVS handle: %s", esp_err_to_name(err));
        return err;
    }

    size_t required_size = 0;
    err = nvs_get_str(my_handle, node->name, NULL, &required_size);
    if (err == ESP_OK) 
    {
        char* value = malloc(required_size);
        if (value)
        {
            err = nvs_get_str(my_handle, node->name, value, &required_size);
            if (err == ESP_OK)
            {
                free(node->value);
                node->value = value;
                ESP_LOGI(TAG, "Loaded %s from NVS: %s", node->name, node->value);
            }
            else
            {
                free(value);
                ESP_LOGE(TAG, "Error getting string value for %s: %s", node->name, esp_err_to_name(err));
            }
        }
        else
        {
            err = ESP_ERR_NO_MEM;
            ESP_LOGE(TAG, "Failed to allocate memory for %s", node->name);
        }
    } 
    else if (err == ESP_ERR_NVS_NOT_FOUND) 
    {
        ESP_LOGI(TAG, "Value for %s not found in NVS, using default: %s", node->name, node->value);
        err = nvs_set_str(my_handle, node->name, node->value);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Error setting string value for %s: %s", node->name, esp_err_to_name(err));
        }
        else
        {
            err = nvs_commit(my_handle);
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "NVS commit failed for %s: %s", node->name, esp_err_to_name(err));
            }
        }
    }
    else 
    {
        ESP_LOGE(TAG, "Error getting required size for %s: %s", node->name, esp_err_to_name(err));
    }

    nvs_close(my_handle);
    return err;
}

esp_err_t persistent_params_init_string(const char* name, const char* default_value) 
{
    param_node_t* new_node = (param_node_t*) malloc(sizeof(param_node_t));
    if (!new_node) 
    {
        return ESP_ERR_NO_MEM;
    }
    new_node->name = strdup(name);
    new_node->value = strdup(default_value);
    new_node->next = head;
    head = new_node;

    return load_param_from_nvs(new_node);
}

esp_err_t persistent_params_get_string(const char* name, char* value, size_t len) 
{
    param_node_t* current = head;
    while (current != NULL) 
    {
        if (strcmp(current->name, name) == 0) 
        {
            strncpy(value, current->value, len);
            return ESP_OK;
        }
        current = current->next;
    }
    return ESP_ERR_NOT_FOUND;
}

esp_err_t persistent_params_set_string(const char* name, const char* value) 
{
    param_node_t* current = head;
    while (current != NULL) 
    {
        if (strcmp(current->name, name) == 0) 
        {
            free(current->value);
            current->value = strdup(value);
            
            nvs_handle_t my_handle;
            esp_err_t err = nvs_open(PARAMS_NAMESPACE, NVS_READWRITE, &my_handle);
            if (err != ESP_OK) 
            {
                return err;
            }
            err = nvs_set_str(my_handle, name, value);
            if (err == ESP_OK) 
            {
                err = nvs_commit(my_handle);
            }
            nvs_close(my_handle);
            return err;
        }
        current = current->next;
    }
    return ESP_ERR_NOT_FOUND;
}

esp_err_t persistent_params_init_integer(const char* name, int32_t default_value)
{
    char str_value[12];
    snprintf(str_value, sizeof(str_value), "%ld", (long)default_value);
    return persistent_params_init_string(name, str_value);
}

esp_err_t persistent_params_get_integer(const char* name, int32_t* value)
{
    char str_value[12];
    esp_err_t err = persistent_params_get_string(name, str_value, sizeof(str_value));
    if (err == ESP_OK)
    {
        *value = atoi(str_value);
    }
    return err;
}

esp_err_t persistent_params_set_integer(const char* name, int32_t value)
{
    char str_value[12];
    snprintf(str_value, sizeof(str_value), "%ld", (long)value);
    return persistent_params_set_string(name, str_value);
}
