#include "nv_storage.h"
#include "comminication.h"


void save_config(config_t *cfg)
{
    nvs_handle_t nvm_handle;
    esp_err_t ret = nvs_open("storage", NVS_READWRITE, &nvm_handle);
    if (ret != ESP_OK) 
    {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(ret));
    } 
    else 
    {
        // Struct'i NVM'e yazma
        ret = nvs_set_blob(nvm_handle, "config", cfg, sizeof(config_t));
        if (ret != ESP_OK) 
        {
            printf("Error (%s) writing data to NVS!\n", esp_err_to_name(ret));
        } 
        else 
        {
            printf("Data written to NVS!\n");
            nvs_commit(nvm_handle);
        }

        nvs_close(nvm_handle);
    }
}


void read_config(config_t *cfg)
{
    nvs_handle_t nvm_handle;
    esp_err_t ret = nvs_open("storage", NVS_READWRITE, &nvm_handle);
    if (ret != ESP_OK) 
    {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(ret));
    }
    else
    {
        size_t required_size;
        ret = nvs_get_blob(nvm_handle, "config", NULL, &required_size);
        if (ret == ESP_OK && required_size == sizeof(config_t)) 
        {
            ret = nvs_get_blob(nvm_handle, "config", cfg, &required_size);
            if (ret != ESP_OK) 
            {
                printf("Error (%s) reading data from NVS!\n", esp_err_to_name(ret));
            } 
        } 
        else 
        {
            printf("Error (%s) reading data size from NVS!\n", esp_err_to_name(ret));
        }

        nvs_close(nvm_handle);
    }
}