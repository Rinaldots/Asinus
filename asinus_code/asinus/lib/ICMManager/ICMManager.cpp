// ICMManager.cpp
#include "ICMManager.h"

#define ICM_AD0_VAL 1

ICMManager::ICMManager(int sda, int scl)
    : sda_pin(sda), scl_pin(scl), icmAvailable(false) {
}

bool ICMManager::beginI2C() {
    Serial.print("Starting I2C on SDA="); Serial.print(sda_pin);
    Serial.print(" SCL="); Serial.println(scl_pin);
    Wire.begin(sda_pin, scl_pin);
    delay(10);
    return true;
}

bool ICMManager::initialize() {
    Serial.println(F("Initializing ICM-20948..."));
    beginI2C();

    bool initialized = false;
    int attempts = 0;
    while (!initialized && attempts < 5) {
        myICM.begin(Wire, ICM_AD0_VAL);
        Serial.print(F("Initialization of the sensor returned: "));
        Serial.println(myICM.statusString());
        if (myICM.status == ICM_20948_Stat_Ok) {
            initialized = true;
            break;
        }
        Serial.println(F("Trying again..."));
        attempts++;
        delay(500);
    }

    icmAvailable = initialized;
    if (initialized) {
        Serial.println(F("ICM-20948 initialized successfully!"));
    } else {
        Serial.println(F("ERROR: ICM-20948 failed to initialize."));
    }
    return initialized;
}

void ICMManager::update() {
    if (!icmAvailable) {
        Serial.println(F("ICM-20948 - NOT AVAILABLE"));
        return;
    }

    if (myICM.dataReady()) {
        myICM.getAGMT();
        printScaledAGMT();
    } else {
        Serial.println(F("ICM-20948: Waiting for data"));
    }
}

// Helper: print a formatted float (small version of example helper)
void ICMManager::printFormattedFloat(float val, uint8_t leading, uint8_t decimals)
{
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

// Print scaled AGMT values from the ICM object
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
