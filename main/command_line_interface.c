#include "command_line_interface.h"
#include "argtable3/argtable3.h"
#include "esp_log.h"
#include "esp_console.h"
#include "esp_vfs_dev.h"
#include "esp_system.h"
#include "typedefs.h"
#include "nv_storage.h"
#include "defaults.h"
#include "calibration.h"

// kullanıcıdan config verisi almaya yarayan yapı
// name: konfigürasyon ismi (string)
// new_value: o isimdeki değişkenin yeni değeri (double)
// end: her zaman bulunmak zorunda 
static struct {
    struct arg_str *name;
    struct arg_dbl *new_value;
    struct arg_end *end;
} cli_cfg_args;

// gerekli yapılara ait adresleri tutacak işaretçiler
static config_t *config_p;
static calibration_t *accel_calib_data_p;
static calibration_t *mag_calib_data_p;
static imu_t *imu_data_p;

static key_value_t config_key_value_pairs[42];

// Burada kullanılan fonksiyonların prototipleri
static int free_mem(int argc, char **argv);
static void register_free(void);
static int heap_size(int argc, char **argv);
static void register_heap(void);
static int restart(int argc, char **argv);
static void register_restart(void);
static int print_config(int argc, char **argv);
static void register_print_config(void);
static int config_set(int argc, char **argv);
static void register_config_set(void);
static int config_save(int argc, char **argv);
static void register_config_save(void);
static int config_load_default(int argc, char **argv);
static void register_config_load_default(void);
static int print_calibration(int argc, char **argv);
static void register_print_calibration(void);
static int reset_accel_calib(int argc, char **argv);
static void register_reset_accel_calib(void);
static int reset_mag_calib(int argc, char **argv);
static void register_reset_mag_calib(void);

// Komut satırı arayüzünü (Command Line Interface) başlatır
void cli_begin(config_t *cfg_ptr, calibration_t *acc_p, calibration_t *mag_p, imu_t *imu_p)
{
    // gerekli veri yapılarına ait adresleri alır
    config_p = cfg_ptr;
    accel_calib_data_p = acc_p;
    mag_calib_data_p = mag_p;
    imu_data_p = imu_p;

    // config_key_value_pairs dizisini doldur
    config_key_value_pairs[0] = (key_value_t){"pitch_p", &cfg_ptr->pitch_p};
    config_key_value_pairs[1] = (key_value_t){"pitch_i", &cfg_ptr->pitch_i};
    config_key_value_pairs[2] = (key_value_t){"pitch_d", &cfg_ptr->pitch_d};
    config_key_value_pairs[3] = (key_value_t){"roll_p", &cfg_ptr->roll_p};
    config_key_value_pairs[4] = (key_value_t){"roll_i", &cfg_ptr->roll_i};
    config_key_value_pairs[5] = (key_value_t){"roll_d", &cfg_ptr->roll_d};
    config_key_value_pairs[6] = (key_value_t){"yaw_p", &cfg_ptr->yaw_p};
    config_key_value_pairs[7] = (key_value_t){"yaw_i", &cfg_ptr->yaw_i};
    config_key_value_pairs[8] = (key_value_t){"pos_p", &cfg_ptr->pos_p};
    config_key_value_pairs[9] = (key_value_t){"pos_i", &cfg_ptr->pos_i};
    config_key_value_pairs[10] = (key_value_t){"alt_p", &cfg_ptr->alt_p};
    config_key_value_pairs[11] = (key_value_t){"alt_i", &cfg_ptr->alt_i};
    config_key_value_pairs[12] = (key_value_t){"alt_d", &cfg_ptr->alt_d};
    config_key_value_pairs[13] = (key_value_t){"max_pitch_angle", &cfg_ptr->max_pitch_angle};
    config_key_value_pairs[14] = (key_value_t){"max_roll_angle", &cfg_ptr->max_roll_angle};
    config_key_value_pairs[15] = (key_value_t){"max_pitch_rate", &cfg_ptr->max_pitch_rate};
    config_key_value_pairs[16] = (key_value_t){"max_roll_rate", &cfg_ptr->max_roll_rate};
    config_key_value_pairs[17] = (key_value_t){"max_yaw_rate", &cfg_ptr->max_yaw_rate};
    config_key_value_pairs[18] = (key_value_t){"pitch_rate_scal", &cfg_ptr->pitch_rate_scal};
    config_key_value_pairs[19] = (key_value_t){"roll_rate_scal", &cfg_ptr->roll_rate_scal};
    config_key_value_pairs[20] = (key_value_t){"yaw_rate_scale", &cfg_ptr->yaw_rate_scale};
    config_key_value_pairs[21] = (key_value_t){"max_vert_vel", &cfg_ptr->max_vert_vel};
    config_key_value_pairs[22] = (key_value_t){"max_horiz_vel", &cfg_ptr->max_horiz_vel};
    config_key_value_pairs[23] = (key_value_t){"voltage_gain", &cfg_ptr->voltage_gain};
    config_key_value_pairs[24] = (key_value_t){"takeoff_alt", &cfg_ptr->takeoff_alt};
    config_key_value_pairs[25] = (key_value_t){"hover_throttle", &cfg_ptr->hover_throttle};
    config_key_value_pairs[26] = (key_value_t){"notch_1_freq", &cfg_ptr->notch_1_freq};
    config_key_value_pairs[27] = (key_value_t){"notch_1_bndwdht", &cfg_ptr->notch_1_bndwdht};
    config_key_value_pairs[28] = (key_value_t){"notch_2_freq", &cfg_ptr->notch_2_freq};
    config_key_value_pairs[29] = (key_value_t){"notch_2_bndwdht", &cfg_ptr->notch_2_bndwdht};
    config_key_value_pairs[30] = (key_value_t){"lpf_cutoff_hz", &cfg_ptr->lpf_cutoff_hz};
    config_key_value_pairs[31] = (key_value_t){"ahrs_filt_beta", &cfg_ptr->ahrs_filt_beta};
    config_key_value_pairs[32] = (key_value_t){"ahrs_filt_zeta", &cfg_ptr->ahrs_filt_zeta};
    config_key_value_pairs[33] = (key_value_t){"alt_filt_beta", &cfg_ptr->alt_filt_beta};
    config_key_value_pairs[34] = (key_value_t){"mag_declin_deg", &cfg_ptr->mag_declin_deg};
    config_key_value_pairs[35] = (key_value_t){"velz_filt_beta", &cfg_ptr->velz_filt_beta};
    config_key_value_pairs[36] = (key_value_t){"velz_filt_zeta", &cfg_ptr->velz_filt_zeta};
    config_key_value_pairs[37] = (key_value_t){"velxy_filt_beta", &cfg_ptr->velxy_filt_beta};
    config_key_value_pairs[38] = (key_value_t){"alt_vel_scale", &cfg_ptr->alt_vel_scale};
    config_key_value_pairs[39] = (key_value_t){"wp_threshold_cm", &cfg_ptr->wp_threshold_cm};
    config_key_value_pairs[40] = (key_value_t){"wp_hdg_cor_gain", &cfg_ptr->wp_hdg_cor_gain};
    config_key_value_pairs[41] = (key_value_t){"wp_dis_vel_gain", &cfg_ptr->wp_dis_vel_gain};

    // Bu builtin fonksiyon "help" komutunda kullanıcıya kayıtlı komutları ve ne işe yaradıklarını gösterir
    esp_console_register_help_command();
    // CLI üzerinden kullanılacak komutlar burada kaydedilir.
    register_free();
    register_heap();
    register_restart();
    register_print_config();
    register_config_set();
    register_config_save();
    register_config_load_default();
    register_print_calibration();
    register_reset_accel_calib();
    register_reset_mag_calib();

    // CLI için gerekli parametreler
    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_config.task_priority = 0;
    repl_config.prompt = "IKARUS>";
    repl_config.max_cmdline_length = 1024;
    esp_console_dev_usb_serial_jtag_config_t hw_config = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_serial_jtag(&hw_config, &repl_config, &repl));
    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}

// Boşta bulunan heap bellek miktarını byte cinsinden CLI ekranına yazar
static int free_mem(int argc, char **argv)
{
    printf("%"PRIu32"\n", esp_get_free_heap_size());
    return 0;
}
// "free" komutunu CLI sistemine kaydeder
static void register_free(void)
{
    const esp_console_cmd_t cmd = {
        .command = "free",
        .help = "Get the current size of free heap memory",
        .hint = NULL,
        .func = &free_mem,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}
// O ana kadar boşta bulunan en az heap bellek miktarını byte cinsinden CLI ekranına yazdırır
static int heap_size(int argc, char **argv)
{
    uint32_t heap_size = heap_caps_get_minimum_free_size(MALLOC_CAP_DEFAULT);
    printf("min heap size: %"PRIu32"\n", heap_size);
    return 0;
}
// "heap" komutunu CLI sistemine kaydeder
static void register_heap(void)
{
    const esp_console_cmd_t heap_cmd = {
        .command = "heap",
        .help = "Get minimum size of free heap memory that was available during program execution",
        .hint = NULL,
        .func = &heap_size,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&heap_cmd));
}
// Sisteme soft reset atar
static int restart(int argc, char **argv)
{
    printf("Restarting\n");
    esp_restart();
}
// "restart" komutunu CLI sistemine kaydeder
static void register_restart(void)
{
    const esp_console_cmd_t cmd = {
        .command = "restart",
        .help = "Software reset of the chip",
        .hint = NULL,
        .func = &restart,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}
// konfigürasyonu CLI ekranına yadırır.
static int print_config(int argc, char **argv)
{
    int max_name_length = 0;
    size_t num_pairs = sizeof(config_key_value_pairs) / sizeof(config_key_value_pairs[0]);

    // En uzun isim uzunluğunu bul. Bu değer muhtemelen 15'tir
    for (size_t i = 0; i < num_pairs; i++) 
    {
        int name_length = strlen(config_key_value_pairs[i].name);
        if (name_length > max_name_length) 
        {
            max_name_length = name_length;
        }
    }

    // Anahtar ve değer çiftlerini düzgün görünmeleri için en uzun ismin uzunluğunu dikkate alarak yazdır.
    // Bu sayede isimler sola dayalı, değerler sağa dayalı şekilde yazacaktır.
    for (size_t i = 0; i < num_pairs; i++) 
    {
        printf("> %-*s: %10.4f\n", max_name_length, config_key_value_pairs[i].name, *(config_key_value_pairs[i].value));
    }
    return 0;
}
// "config" komutunu CLI sistemine kaydeder
static void register_print_config(void)
{
    const esp_console_cmd_t cmd = {
        .command = "config",
        .help = "Print current config values",
        .hint = NULL,
        .func = &print_config,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}
// kullanıcının girdiği konfigürasyon ismi ile girilen değeri değiştirir
static int config_set(int argc, char **argv)
{
    // kullanıcının girdiği parametreleri ayıklar
    int nerrors = arg_parse(argc, argv, (void **) &cli_cfg_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, cli_cfg_args.end, argv[0]);
        // parametreler anlamlı değilse 1 döndür ve uyarı yazdır.
        return 1;
    }

    // kullanıcının girdiği konfigürasyon ismi var mı kontrol eder
    // isim eşleşirse o isme ait değişkenin değerini kullanıcının girdiği değer yap
    for (size_t i = 0; i < sizeof(config_key_value_pairs) / sizeof(config_key_value_pairs[0]); i++) 
    {
        // isim işleşiyor mu
        if (strcmp(config_key_value_pairs[i].name, cli_cfg_args.name->sval[0]) == 0) 
        {
            // o isme karşılık gelen değişkenin değerini değiştir.
            *(config_key_value_pairs[i].value) = cli_cfg_args.new_value->dval[0];
            printf("New value for '%s' is '%f' (NOT SAVED)\n", config_key_value_pairs[i].name, *(config_key_value_pairs[i].value));
            return 0;
        }
    }
    // isim bulunamadı
    printf("Name '%s' not found!\n", cli_cfg_args.name->sval[0]);

    return 0;
}
// "config_set" komutunu CLI sistemine kaydeder
static void register_config_set(void)
{
    // kullanıcıdan ilk istenen parametre config yapısındaki değişkenlerden birinin ismi
    // ikinci ise o değişkenin yeni değeri
    // arg_str1 --> zorunlu string parametre / arg_str0 --> isteğe bağlı string parametre
    // arg_dbl1 --> zorunlu double parametre / arg_dbl0 --> isteğe bağlı double parametre
    // bu fonksiyonlar #include "argtable3/argtable3.h" içinde tanımlı
    // örnek kullanıcı girdisi şöyle olmalı: (config_set pitch_p 3.1415) negatif ise (config_set pitch_p -- -3.1415)
    cli_cfg_args.name = arg_str1(NULL, NULL, "<name>", "Config name");
    cli_cfg_args.new_value = arg_dbl1(NULL, NULL, "<float>", "New value (add '--' before nagative value)");
    // arg_end bulunmak zorunda, burdaki 2 değeri kullanıcı yanlış parametre girerse kaç tane hata mesajı gösterileceğini belirtiyor.
    cli_cfg_args.end = arg_end(2);
    const esp_console_cmd_t cmd = {
        .command = "config_set",
        .help = "Change config value",
        .hint = NULL,
        .func = &config_set,
        .argtable = &cli_cfg_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}
// kullanıcının değiştirdiği konfigürasyonu kalıcı olarak kaydeder
static int config_save(int argc, char **argv)
{
    if (storage_save(config_p, CONFIG_DATA)) printf("Configuration saved\n");
    else printf("ERROR: Configuration NOT saved!\n");
    return 0;
}
// "config_save" komutunu CLI sistemine kaydeder
static void register_config_save(void)
{
    const esp_console_cmd_t cmd = {
        .command = "config_save",
        .help = "Save current configuration",
        .hint = NULL,
        .func = &config_save
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}
// varsayılan konfigürasyonu yükler. Ancak kaydetmez
static int config_load_default(int argc, char **argv)
{
    load_default_config(config_p);
    printf("Config not saved yet!\n");
    return 0;
}
// "config_load_def" komutunu CLI sistemine kaydeder
static void register_config_load_default(void)
{
    const esp_console_cmd_t cmd = {
        .command = "config_load_def",
        .help = "Load default configuration",
        .hint = NULL,
        .func = &config_load_default
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}
// kalibrasyon parametrelerini CLI ekranına yazdırır.
static int print_calibration(int argc, char **argv)
{
    printf("\n--Accelerometer--\n");
    printf("Offset --> X: %.4f  Y: %.4f  Z: %.4f\n", accel_calib_data_p->offset[X], accel_calib_data_p->offset[Y], accel_calib_data_p->offset[Z]);
    printf("Scale --> X: %.4f  Y: %.4f  Z: %.4f\n\n", accel_calib_data_p->scale[X], accel_calib_data_p->scale[Y], accel_calib_data_p->scale[Z]);
    printf("--Magnetometer--\n");
    printf("Offset --> X: %.4f  Y: %.4f  Z: %.4f\n", mag_calib_data_p->offset[X], mag_calib_data_p->offset[Y], mag_calib_data_p->offset[Z]);
    printf("Scale --> X: %.4f  Y: %.4f  Z: %.4f\n\n", mag_calib_data_p->scale[X], mag_calib_data_p->scale[Y], mag_calib_data_p->scale[Z]);
    printf("--Gyroscope--\n");
    printf("Offset --> X: %.4f  Y: %.4f  Z: %.4f\n\n", imu_data_p->gyro_bias_dps[X], imu_data_p->gyro_bias_dps[Y], imu_data_p->gyro_bias_dps[Z]);

    printf("--Current Calibrated IMU Measurements--\n");
    printf("\n--Accelerometer--\n");
    printf("X: %.2f m/s2  Y: %.2f m/s2  Z: %.2f m/s2\n", imu_data_p->accel_ms2[X], imu_data_p->accel_ms2[Y], imu_data_p->accel_ms2[Z]);
    printf("\n--Gyroscope--\n");
    printf("X: %.2f dps  Y: %.2f dps  Z: %.2f dps\n", imu_data_p->gyro_dps[X], imu_data_p->gyro_dps[Y], imu_data_p->gyro_dps[Z]);
    return 0;
}
// "calib" komutunu CLI sistemine kaydeder
static void register_print_calibration(void)
{
    const esp_console_cmd_t cmd = {
        .command = "calib",
        .help = "Print calibration values",
        .hint = NULL,
        .func = &print_calibration
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}
// İvmeölçer kalibrasyonunu sıfırlar
static int reset_accel_calib(int argc, char **argv)
{
    reset_calibration_data(accel_calib_data_p);
    storage_save(accel_calib_data_p, ACCEL_CALIB_DATA);
    printf("Accelerometer calibration data reset!\n");
    return 0;
}
// "reset_acc_calib" komutunu CLI sistemine kaydeder
static void register_reset_accel_calib(void)
{
    const esp_console_cmd_t cmd = {
        .command = "reset_acc_calib",
        .help = "Reset accelerometer calibration values",
        .hint = NULL,
        .func = &reset_accel_calib
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}
// Manyetik sensör kalibrasyonunu sıfırlar
static int reset_mag_calib(int argc, char **argv)
{
    reset_calibration_data(mag_calib_data_p);
    storage_save(mag_calib_data_p, MAG_CALIB_DATA);
    printf("Magnetometer calibration data reset!\n");
    return 0;
}
// "reset_mag_calib" komutunu CLI sistemine kaydeder
static void register_reset_mag_calib(void)
{
    const esp_console_cmd_t cmd = {
        .command = "reset_mag_calib",
        .help = "Reset Magnetometer calibration values",
        .hint = NULL,
        .func = &reset_mag_calib
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}