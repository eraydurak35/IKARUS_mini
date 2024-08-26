#include "calibration.h"
#include "gpio.h"
#include "math.h"
#include "quaternion.h"
#include "filters.h"
#include "nv_storage.h"
#include "typedefs.h"

static calibration_t calibration_data;

// İvme ölçer sensörün kalibrasyonunu gerçekleştirir
// Kalibrasyon sürerken 0, kalibrasyon bittiğinde 1 döndürür
uint8_t accelerometer_calibration(imu_t *imu)
{
    static float x_values[6];
    static float y_values[6];
    static float z_values[6];
    // Kalibrasyon mantığı bir yönde ölçülen en büyük ve en küçük 
    // değerleri doğru tespit etmeye bağlı olduğu için 5 değerli
    // bir medyan filtre kullanılması uygun görülmüştür.
    // Bu sayede anlık olarak çok büyük veya çok büyük değerlerin 
    // elenip ortalamada en büyük ve en küçük değerlerin tespit edilmesi sağlanmıştır.
    static median_filter_t filter_x = {5, {0}, {0}};
    static median_filter_t filter_y = {5, {0}, {0}};
    static median_filter_t filter_z = {5, {0}, {0}};
    static uint8_t current_direction_number = 0;
    // Kalibrasyonun gerçekleştirilmesi 6 ana yönde yapılacak ölçümler ile mümkündür.
    // Ölçüm yapılacak yönleri temsil eden vektörler (X, Y, Z) sıralaması ile yazılmıştır
    static vector_t referance_vector[6] = 
    {
        {0.0f, 1.0f, 0.0f},     // Yukarı (Up)
        {-1.0f, 0.0f, 0.0f},    // Sağa   (Right)
        {0.0f, -1.0f, 0.0f},    // Aşağı  (Down)
        {1.0f, 0.0f, 0.0f},     // Sola   (Left)
        {0.0f, 0.0f, -1.0f},    // Ters   (Inverted)
        {0.0f, 0.0f, 1.0f},     // Düz    (Upright)
    };
    static uint16_t data_point_counter = 0;

    // Medyan filtreye ölçülen ivme değerlerini ver
    // Çıktı olarak filtrelenmiş veri al
    imu->accel_ms2[X] = median_filter(&filter_x, imu->accel_ms2[X]);
    imu->accel_ms2[Y] = median_filter(&filter_y, imu->accel_ms2[Y]);
    imu->accel_ms2[Z] = median_filter(&filter_z, imu->accel_ms2[Z]);

    // Filtrelenmiş ivme değerlerini vektör haline getir.
    vector_t accel_vector = {imu->accel_ms2[X], imu->accel_ms2[Y], imu->accel_ms2[Z]};

    // O an ölçülmesi gereken yön vektörü ile sensörden elde edilen yön vektörü arasındaki açıyı hesapla
    // Bu açı 0'a çok yakınsa kullanıcı sensörü doğru yönde tutuyor demektir.
    // Mesela sensörün zemine paralel olması gerekiyor ve kullanıcı sensörü eğik tutuyorsa bu açı 0'a uzak bir değer olacaktır.
    float angle = angle_between_vectors(&accel_vector, &referance_vector[current_direction_number]);
    // Led'in parlaklığını arttırarak kulanıcıya veri toplandığını ve kalibrasyon işleminin devam ettiğini bildir
    led_set_brightness(data_point_counter / 10);

    // Açı değeri bu eşik değerden düşükse veri topla
    if (angle < 3.0f)
    {
        // Ortalama değer bulmak için veri topla
        x_values[current_direction_number] += imu->accel_ms2[X];
        y_values[current_direction_number] += imu->accel_ms2[Y];
        z_values[current_direction_number] += imu->accel_ms2[Z];

        printf("%.2f, %.2f, %.2f\n", x_values[current_direction_number], y_values[current_direction_number], z_values[current_direction_number]);
        // Kaç adet veri toplandığını tut
        data_point_counter++;
        // Yeterince veri toplandıysa kalibrasyonu tamamla
        if (data_point_counter == 1000)
        {
            // Oralama değer bulmak için veri adedine böl
            x_values[current_direction_number] /= 1000.0f;
            y_values[current_direction_number] /= 1000.0f;
            z_values[current_direction_number] /= 1000.0f;
            printf("%.2f, %.2f, %.2f\n", x_values[current_direction_number], y_values[current_direction_number], z_values[current_direction_number]);
            // Sonraki kalibrasyon yönüne geç
            current_direction_number++;

            // Son yön de kalibre edildiyse
            if (current_direction_number == 6)
            {
                // Ortalama offset değerini bulmak için en büyük değer ile en küçük değeri topla ve 2 ye böl
                calibration_data.offset[X] = (x_values[3] + x_values[1]) / 2.0f;
                calibration_data.offset[Y] = (y_values[0] + y_values[2]) / 2.0f;
                calibration_data.offset[Z] = (z_values[5] + z_values[4]) / 2.0f;

                // Bulunan offset değerini en büyük ve en küçük ölçümlerden çıkart
                x_values[3] -= calibration_data.offset[X];
                x_values[1] -= calibration_data.offset[X];
                y_values[0] -= calibration_data.offset[Y];
                y_values[2] -= calibration_data.offset[Y];
                z_values[5] -= calibration_data.offset[Z];
                z_values[4] -= calibration_data.offset[Z];

                // ölçek değerini bulmak için en büyük değerden en küçük değeri çıkart ve 2 ye böl
                // 1g = 9.806m/s2 değeri için ölçek bul
                calibration_data.scale[X] = 9.806f / ((x_values[3] - x_values[1]) / 2.0f);
                calibration_data.scale[Y] = 9.806f / ((y_values[0] - y_values[2]) / 2.0f);
                calibration_data.scale[Z] = 9.806f / ((z_values[5] - z_values[4]) / 2.0f);

                // Bulunan kalibrasyon değerlerini kaydet
                storage_save(&calibration_data, ACCEL_CALIB_DATA);

                printf("offset X: %.3f  Y: %.3f  Z: %.3f\n", calibration_data.offset[X], calibration_data.offset[Y], calibration_data.offset[Z]);
                printf("scale X: %.3f  Y: %.3f  Z: %.3f\n", calibration_data.scale[X], calibration_data.scale[Y], calibration_data.scale[Z]);
                // Kalibrasyon tamamlandı 1 döndür
                return 1;
            }
            // Sonraki yön için veri sayısını sıfırla
            data_point_counter = 0;
        }
    }
    // Veri toplama sırasında kart hareket ettirilirse 
    // yani açı değeri eşiğin üzerine çıkarsa kalibrasyonu sıfırla
    // O yön için baştan veri topla
    else
    {
        x_values[current_direction_number] = 0;
        y_values[current_direction_number] = 0;
        z_values[current_direction_number] = 0;
        data_point_counter = 0;
    }
    // Kalibrasyon devam ediyor 0 döndür
    return 0;
}

uint8_t magnetometer_calibration(magnetometer_t *mag)
{
    static uint8_t init = 0;
    static uint16_t counter = 0;
    static float x_biggest_value = 0;
    static float x_smallest_value = 0;
    static float y_biggest_value = 0;
    static float y_smallest_value = 0;
    static float z_biggest_value = 0;
    static float z_smallest_value = 0;
    // Kalibrasyon mantığı bir yönde ölçülen en büyük ve en küçük 
    // değerleri doğru tespit etmeye bağlı olduğu için 7 değerli
    // bir medyan filtre kullanılması uygun görülmüştür.
    // Bu sayede anlık olarak çok büyük veya çok büyük değerlerin 
    // elenip ortalamada en büyük ve en küçük değerlerin tespit edilmesi sağlanmıştır.
    static median_filter_t filter_x = {7, {0}, {0}};
    static median_filter_t filter_y = {7, {0}, {0}};
    static median_filter_t filter_z = {7, {0}, {0}};

    // Kalibrasyon işlemi başlamadan önce kullanıcıya
    // LED yakıp söndürerek işlemin başlayacağını bildir
    if (!init)
    {
        // İşlem başlamadan önce medyan filtrenin bufferını doldur
        mag->axis[X] = median_filter(&filter_x, mag->axis[X]);
        mag->axis[Y] = median_filter(&filter_y, mag->axis[Y]);
        mag->axis[Z] = median_filter(&filter_z, mag->axis[Z]);

        // LED yakıp söndür (50ms x 10 = 500ms)
        if (counter < 10) led_set_brightness(100);
        else if (counter < 20) led_set_brightness(0);
        else if (counter < 30) led_set_brightness(100);
        else if (counter < 40) 
        {
            led_set_brightness(0);
            counter = 0;
            init = 1;
            x_biggest_value = mag->axis[X];
            x_smallest_value = mag->axis[X];
            y_biggest_value = mag->axis[Y];
            y_smallest_value = mag->axis[Y];
            z_biggest_value = mag->axis[Z];
            z_smallest_value = mag->axis[Z];
            return 0;
        }
        // LED yanıp sönme süresini tutan sayaç
        counter++;
    }
    // Kalibrasyon başladı
    // Kalibrasyon süresi 60 saniyedir
    else
    {
        // Yeni gelen ölçümleri medyan filtreden geçir
        mag->axis[X] = median_filter(&filter_x, mag->axis[X]);
        mag->axis[Y] = median_filter(&filter_y, mag->axis[Y]);
        mag->axis[Z] = median_filter(&filter_z, mag->axis[Z]);

        // O ana kadar ölçülen en büyük ve en küçük değeri bul
        if (mag->axis[X] > x_biggest_value) x_biggest_value = mag->axis[X];
        else if (mag->axis[X] < x_smallest_value) x_smallest_value = mag->axis[X];
        if (mag->axis[Y] > y_biggest_value) y_biggest_value = mag->axis[Y];
        else if (mag->axis[Y] < y_smallest_value) y_smallest_value = mag->axis[Y];
        if (mag->axis[Z] > z_biggest_value) z_biggest_value = mag->axis[Z];
        else if (mag->axis[Z] < z_smallest_value) z_smallest_value = mag->axis[Z];

        // Kalibrasyon süresini tutan sayaç
        counter++;
        // LED her 10 saniyede bir tam parlaklığa ulaşıp sönecektir.
        // Bu kullanıcıya başka bir yönde veri toplaması gerektiğini gösterir
        // Bu işlem 6 yön için 6 defa tekrar eder.
        led_set_brightness((counter%200) / 2);
        // Sayaç 1200 e ulaştığında kalibrasyon tamamlanır.
        if (counter == 1200)
        {
            printf("xb:%.2f  xs:%.2f  yb:%.2f  ys:%.2f  zb:%.2f  zs:%.2f\n", x_biggest_value, x_smallest_value, y_biggest_value, y_smallest_value, z_biggest_value, z_smallest_value);

            // Tespit edilen en büyük ve en küçük değerlerden offset değerini hesaplar
            calibration_data.offset[X] = (x_biggest_value + x_smallest_value) / 2.0f;
            calibration_data.offset[Y] = (y_biggest_value + y_smallest_value) / 2.0f;
            calibration_data.offset[Z] = (z_biggest_value + z_smallest_value) / 2.0f;

            // Offset değerini çıkart
            x_biggest_value -= calibration_data.offset[X];
            x_smallest_value -= calibration_data.offset[X];
            y_biggest_value -= calibration_data.offset[Y];
            y_smallest_value -= calibration_data.offset[Y];
            z_biggest_value -= calibration_data.offset[Z];
            z_smallest_value -= calibration_data.offset[Z];

            // Ölçek değerini hesapla
            calibration_data.scale[X] = (x_biggest_value - x_smallest_value) / 2.0f;
            calibration_data.scale[Y] = (y_biggest_value - y_smallest_value) / 2.0f;
            calibration_data.scale[Z] = (z_biggest_value - z_smallest_value) / 2.0f;
            float avg_delta = (calibration_data.scale[X] + calibration_data.scale[Y] + calibration_data.scale[Z]) / 3.0f;

            calibration_data.scale[X] = avg_delta / calibration_data.scale[X];
            calibration_data.scale[Y] = avg_delta / calibration_data.scale[Y];
            calibration_data.scale[Z] = avg_delta / calibration_data.scale[Z];

            // Bulunan kalibrasyon değerlerini kayet
            storage_save(&calibration_data, MAG_CALIB_DATA);
            
            printf("offset X: %.3f  Y: %.3f  Z: %.3f\n", calibration_data.offset[X], calibration_data.offset[Y], calibration_data.offset[Z]);
            printf("scale X: %.3f  Y: %.3f  Z: %.3f\n", calibration_data.scale[X], calibration_data.scale[Y], calibration_data.scale[Z]);

            // Kalibrasyon tamamlandır 1 döndür
            return 1;
        }
    }

    // Kalibrasyon devam ediyor 0 döndür
    return 0;
}


uint8_t gyro_calibration(imu_t *imu)
{
    static uint16_t i = 0;
    static float sum_x = 0, sum_y = 0, sum_z = 0;
    float movement_vector = fabsf(imu->gyro_dps[X]) + fabsf(imu->gyro_dps[Y]) + fabsf(imu->gyro_dps[Z]);

    if (movement_vector < 2.5f)
    {
        sum_x += imu->gyro_dps[X];
        sum_y += imu->gyro_dps[Y];
        sum_z += imu->gyro_dps[Z];
        i++;

        if (i == 1000)
        {
            imu->gyro_bias_dps[X] = sum_x / 1000.0f;
            imu->gyro_bias_dps[Y] = sum_y / 1000.0f;
            imu->gyro_bias_dps[Z] = sum_z / 1000.0f;
            return 1;
        }
    }
    else
    {
        i = 0;
        sum_x = 0;
        sum_y = 0;
        sum_z = 0;
    }
    return 0;
}

void reset_calibration_data(calibration_t *data)
{
    for (uint8_t i = 0; i < 3; i++) data->offset[i] = 0.0f;
    for (uint8_t i = 0; i < 3; i++) data->scale[i] = 1.0f;
}
