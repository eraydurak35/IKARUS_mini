#include "nv_storage.h"
#include "comminication.h"


uint8_t storage_save(void *ptr, enum Storage type)
{
    nvs_handle_t nvm_handle;
    esp_err_t ret = nvs_open("storage", NVS_READWRITE, &nvm_handle);
    if (ret != ESP_OK) 
    {
        printf("Error (%s) opening NVS handle!  \n", esp_err_to_name(ret));
        
    } 
    switch (type)
    {
        case CONFIG_DATA:

            config_t *config_ptr = (config_t *)ptr;
            // Struct'i NVM'e yazma
            ret = nvs_set_blob(nvm_handle, "config", config_ptr, sizeof(config_t));
            if (ret != ESP_OK) 
            {
                printf("Error (%s) writing config data to NVS!  \n", esp_err_to_name(ret));
                return 0;
            } 
            else 
            {
                printf("Configuration written to NVS!\n");
                nvs_commit(nvm_handle);
            }

            break;
        case ACCEL_CALIB_DATA:
            
            calibration_t *accel_calib_ptr = (calibration_t *)ptr;
            // Struct'i NVM'e yazma
            ret = nvs_set_blob(nvm_handle, "accel_calib", accel_calib_ptr, sizeof(calibration_t));
            if (ret != ESP_OK) 
            {
                printf("Error (%s) writing accel calibration data to NVS!  \n", esp_err_to_name(ret));
                return 0;
            } 
            else 
            {
                printf("Accel calibration written to NVS!\n");
                nvs_commit(nvm_handle);
            }
            break;
        case MAG_CALIB_DATA:

            calibration_t *mag_calib_ptr = (calibration_t *)ptr;
            // Struct'i NVM'e yazma
            ret = nvs_set_blob(nvm_handle, "mag_calib", mag_calib_ptr, sizeof(calibration_t));
            if (ret != ESP_OK) 
            {
                printf("Error (%s) writing mag calibration data to NVS!  \n", esp_err_to_name(ret));
                return 0;
            } 
            else 
            {
                printf("Mag calibration written to NVS!\n");
                nvs_commit(nvm_handle);
            }
            break;
        
        default:
            printf("Unknown data type, NVS Storage\n");
            return 0;
            break;
    }

    nvs_close(nvm_handle);
    return 1;
}


uint8_t storage_read(void *ptr, enum Storage type)
{
    nvs_handle_t nvm_handle;
    size_t required_size;
    esp_err_t ret = nvs_open("storage", NVS_READWRITE, &nvm_handle);
    if (ret != ESP_OK) 
    {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(ret));
        return 0;
    }

    switch (type)
    {
        case CONFIG_DATA:
            
            ret = nvs_get_blob(nvm_handle, "config", NULL, &required_size);
            if (ret == ESP_OK && required_size == sizeof(config_t)) 
            {
                ret = nvs_get_blob(nvm_handle, "config", ptr, &required_size);
                if (ret != ESP_OK) 
                {
                    printf("Custom config not found.\n");
                    nvs_close(nvm_handle);
                    return 0;
                } 
            } 
            else 
            {
                printf("Error (%s) reading data size from NVS!\n", esp_err_to_name(ret));
                nvs_close(nvm_handle);
                return 0;
            }

            
            break;

        case ACCEL_CALIB_DATA:

            ret = nvs_get_blob(nvm_handle, "accel_calib", NULL, &required_size);
            if (ret == ESP_OK && required_size == sizeof(calibration_t)) 
            {
                ret = nvs_get_blob(nvm_handle, "accel_calib", ptr, &required_size);
                if (ret != ESP_OK) 
                {
                    printf("Error (%s) reading data from NVS!\n", esp_err_to_name(ret));
                    nvs_close(nvm_handle);
                    return 0;
                } 
            } 
            else 
            {
                printf("Error (%s) reading data size from NVS!\n", esp_err_to_name(ret));
                nvs_close(nvm_handle);
                return 0;
            }
            break;

        case MAG_CALIB_DATA:

            ret = nvs_get_blob(nvm_handle, "mag_calib", NULL, &required_size);
            if (ret == ESP_OK && required_size == sizeof(calibration_t)) 
            {
                ret = nvs_get_blob(nvm_handle, "mag_calib", ptr, &required_size);
                if (ret != ESP_OK) 
                {
                    printf("Error (%s) reading data from NVS!\n", esp_err_to_name(ret));
                    nvs_close(nvm_handle);
                    return 0;
                } 
            } 
            else 
            {
                printf("Error (%s) reading data size from NVS!\n", esp_err_to_name(ret));
                nvs_close(nvm_handle);
                return 0;
            }
            break;
        
        default:
            printf("Unknown data type, NVS Storage\n");
            nvs_close(nvm_handle);
            return 0;
            break;
    }
    nvs_close(nvm_handle);
    return 1;
}
