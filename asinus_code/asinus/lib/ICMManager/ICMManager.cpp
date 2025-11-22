#include "ICMManager.h"
#include "AsinusManager.h"

#define ICM_AD0_VAL 1

ICMManager::ICMManager(int sda, int scl)
    : sda_pin(sda), scl_pin(scl), icmAvailable(false), dmpInitialized(false) // Inicializar dmpInitialized
{
}

bool ICMManager::beginI2C()
{
    Serial.print("Starting I2C on SDA=");
    Serial.print(sda_pin);
    Serial.print(" SCL=");
    Serial.println(scl_pin);
    Wire.begin(sda_pin, scl_pin);
    Wire.setClock(400000); // 400kHz para comunicação mais rápida
    delay(10);
    return true;
}

bool ICMManager::initialize()
{
    Serial.println(F("Initializing ICM-20948..."));
    beginI2C();

    bool initialized = false;
    int attempts = 0;
    while (!initialized && attempts < 5)
    {
        myICM.begin(Wire, ICM_AD0_VAL);
        Serial.print(F("Initialization of the sensor returned: "));
        Serial.println(myICM.statusString());
        if (myICM.status == ICM_20948_Stat_Ok)
        {
            initialized = true;
            break;
        }
        Serial.println(F("Trying again..."));
        attempts++;
        delay(500);
    }

    icmAvailable = initialized;
    if (initialized)
    {
        Serial.println(F("ICM-20948 initialized successfully! Attempting DMP setup..."));
        
        // --- CONFIGURAÇÃO DO DMP PARA ORIENTAÇÃO (QUAT9) ---
        bool success = true;
        
        success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);
        success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION) == ICM_20948_Stat_Ok);
        success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat9, 0) == ICM_20948_Stat_Ok); // ODR Máxima
        success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);
        success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);
        success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);
        success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);

        dmpInitialized = success;

        if (dmpInitialized)
        {
            Serial.println(F("DMP Quat9 Orientation ENABLED!"));
        }
        else
        {
            Serial.println(F("ERROR: DMP setup failed. Check if ICM_20948_USE_DMP is uncommented in ICM_20948_C.h."));
            icmAvailable = false; // Se o DMP falhou, não consideramos o sensor totalmente disponível para orientação
        }
    }
    else
    {
        Serial.println(F("ERROR: ICM-20948 failed to initialize."));
    }
    return icmAvailable;
}

void ICMManager::update()
{
    if (!icmAvailable)
    {
        Serial.println(F("ICM-20948 - NOT AVAILABLE"));
        return;
    }

    // Tenta ler dados do DMP primeiro, se disponível
    if (dmpInitialized)
    {
        icm_20948_DMP_data_t data;
        myICM.readDMPdataFromFIFO(&data);

        if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail))
        {
            Serial.print(F("DMP Data Available. Header: 0x"));
            Serial.println(data.header, HEX);

            if ((data.header & DMP_header_bitmap_Quat9) > 0)
            {
                // Processa o Quaternion para imprimir (temporariamente, para teste)
                IMUTelemetry imu;
                processDMPData(&data, &imu);
                Serial.print(F("Quat (w,x,y,z): "));
                Serial.print(imu.qw, 3);
                Serial.print(F(", "));
                Serial.print(imu.qx, 3);
                Serial.print(F(", "));
                Serial.print(imu.qy, 3);
                Serial.print(F(", "));
                Serial.println(imu.qz, 3);
            }
        }
        // Se o FIFO tiver mais dados, o loop principal deve chamar update() novamente rapidamente
    }
    else // Fallback para dados RAW se o DMP não estiver habilitado
    {
        if (myICM.dataReady())
        {
            myICM.getAGMT();
            printScaledAGMT();
        }
        else
        {
            // Serial.println(F("ICM-20948: Waiting for data")); // Comentado para evitar flood de serial
        }
    }
}

// Helper: print a formatted float (small version of example helper)
void ICMManager::printFormattedFloat(float val, uint8_t leading, uint8_t decimals)
{
    // ... (Mantido como estava, omitido para brevidade)
    float aval = abs(val);
    if (val < 0)
    {
        Serial.print("-");
    }
    else
    {
        Serial.print(" ");
    }
    for (uint8_t indi = 0; indi < leading; indi++)
    {
        uint32_t tenpow = 0;
        if (indi < (leading - 1))
        {
            tenpow = 1;
        }
        for (uint8_t c = 0; c < (leading - 1 - indi); c++)
        {
            tenpow *= 10;
        }
        if (aval < tenpow)
        {
            Serial.print("0");
        }
        else
        {
            break;
        }
    }
    if (val < 0)
    {
        Serial.print(-val, decimals);
    }
    else
    {
        Serial.print(val, decimals);
    }
}

// Print scaled AGMT values from the ICM object (Mantido como estava, útil para depuração)
void ICMManager::printScaledAGMT()
{
    Serial.print(F("Scaled. Acc (mg) [ "));
    printFormattedFloat(myICM.accX(), 5, 2);
    Serial.print(F(", "));
    printFormattedFloat(myICM.accY(), 5, 2);
    Serial.print(F(", "));
    printFormattedFloat(myICM.accZ(), 5, 2);
    Serial.print(F(" ], Gyr (DPS) [ "));
    printFormattedFloat(myICM.gyrX(), 5, 2);
    Serial.print(F(", "));
    printFormattedFloat(myICM.gyrY(), 5, 2);
    Serial.print(F(", "));
    printFormattedFloat(myICM.gyrZ(), 5, 2);
    Serial.print(F(" ], Mag (uT) [ "));
    printFormattedFloat(myICM.magX(), 5, 2);
    Serial.print(F(", "));
    printFormattedFloat(myICM.magY(), 5, 2);
    Serial.print(F(", "));
    printFormattedFloat(myICM.magZ(), 5, 2);
    Serial.print(F(" ], Tmp (C) [ "));
    printFormattedFloat(myICM.temp(), 5, 2);
    Serial.print(F(" ]"));
    Serial.println();
}

// NOVO: Função para processar os dados do DMP
void ICMManager::processDMPData(icm_20948_DMP_data_t *data, IMUTelemetry *imu)
{
    // O quaternion é escalado por 2^30.
    const double scale = 1073741824.0; // 2^30

    // Converte e escala Q1, Q2, Q3
    double qx = ((double)data->Quat9.Data.Q1) / scale;
    double qy = ((double)data->Quat9.Data.Q2) / scale;
    double qz = ((double)data->Quat9.Data.Q3) / scale;

    // Calcula Q0 (qw) usando Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1
    double q_mag_sq = (qx * qx) + (qy * qy) + (qz * qz);
    double qw = (q_mag_sq < 1.0) ? sqrt(1.0 - q_mag_sq) : 0.0;

    // Armazena no struct
    imu->qw = (float)qw;
    imu->qx = (float)qx;
    imu->qy = (float)qy;
    imu->qz = (float)qz;
    // TO-DO: Implementar a conversão de Quat para Yaw/Pitch/Roll se necessário
}

IMUTelemetry ICMManager::returnTelemetry()
{
    IMUTelemetry imu;
    imu.ts = millis();

    if (!icmAvailable)
    {
        return imu;
    }
    
    // --- LER DADOS DMP (ORIENTAÇÃO) ---
    if (dmpInitialized)
    {
        icm_20948_DMP_data_t data;
        myICM.readDMPdataFromFIFO(&data);

        if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail))
        {
            if ((data.header & DMP_header_bitmap_Quat9) > 0)
            {
                processDMPData(&data, &imu);
            }
        }
    }
    
    // --- LER DADOS RAW (AGMT) ---
    // Mesmo que o DMP esteja em uso, podemos ler os dados RAW separadamente (se o DMP não estiver usando o FIFO completo)
    // No entanto, para simplificar, se o DMP for o foco, podemos apenas pegar os dados RAW se o DMP não tiver fornecido
    // orientação, OU se quisermos garantir que os campos raw sejam sempre preenchidos se o DMP não estiver lendo tudo.
    
    // É mais seguro ler AGMT (acel/gyro/mag/temp) após o DMP, pois o DMP usa o FIFO
    // e os dados RAW são lidos diretamente dos registradores.
    if (myICM.dataReady())
    {
        myICM.getAGMT();
        imu.accel_x = myICM.accX();
        imu.accel_y = myICM.accY();
        imu.accel_z = myICM.accZ();
        imu.gyro_x = myICM.gyrX();
        imu.gyro_y = myICM.gyrY();
        imu.gyro_z = myICM.gyrZ();
        imu.mag_x = myICM.magX();
        imu.mag_y = myICM.magY();
        imu.mag_z = myICM.magZ();
        imu.temp = myICM.temp();
    }
    
    return imu;
}