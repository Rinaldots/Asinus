#!/usr/bin/env python3
"""No ROS2 para a IMU ICM-20948 ligada por I2C ao Orange Pi 3B.

Le acelerometro + giroscopio + magnetometro (AK09916, em modo bypass) direto
pelo /dev/i2c-<bus> e publica:
  - sensor_msgs/Imu            em  imu/data_raw   (accel + gyro, SEM orientacao)
  - sensor_msgs/MagneticField  em  imu/mag

A orientacao NAO vem do chip aqui (o DMP so existe na lib Arduino da SparkFun);
use o imu_filter_madgwick para fundir accel+gyro+mag -> imu/data.

Ligacao (i2c2-m1): SDA=pino 3, SCL=pino 5, VCC=pino 1 (3,3V), GND=pino 6 -> /dev/i2c-2
Dependencia: smbus2   (sudo apt install python3-smbus2  ou  pip install smbus2)
"""
import time
import math

from smbus2 import SMBus

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField

# --- ICM-20948 registradores (User Bank 0) ---
REG_BANK_SEL = 0x7F
WHO_AM_I = 0x00
USER_CTRL = 0x03
PWR_MGMT_1 = 0x06
PWR_MGMT_2 = 0x07
INT_PIN_CFG = 0x0F
ACCEL_XOUT_H = 0x2D          # 6 bytes accel + 6 bytes gyro sao contiguos (0x2D..0x38)

# --- User Bank 2 ---
GYRO_CONFIG_1 = 0x01
ACCEL_CONFIG = 0x14

# --- Magnetometro AK09916 (endereco I2C proprio, acessivel via bypass) ---
AK09916_ADDR = 0x0C
AK09916_ST1 = 0x10
AK09916_HXL = 0x11
AK09916_CNTL2 = 0x31
AK09916_CNTL3 = 0x32

# Faixas fixas: accel +-2g, gyro +-250 dps
ACCEL_SENS = 16384.0         # LSB/g
GYRO_SENS = 131.0            # LSB/(deg/s)
G = 9.80665                  # m/s^2
DEG2RAD = math.pi / 180.0
MAG_SENS = 0.15              # uT/LSB (AK09916)
UT_TO_T = 1e-6               # uT -> Tesla


def _int16(high, low):
    v = (high << 8) | low
    return v - 65536 if v >= 32768 else v


class Icm20948Node(Node):
    def __init__(self):
        super().__init__('icm20948')
        self.declare_parameter('i2c_bus', 2)
        self.declare_parameter('i2c_address', 0x69)   # AD0=1 (teu codigo usa ICM_AD0_VAL 1); 0x68 se AD0=0
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('frequency', 100.0)

        self.bus_num = int(self.get_parameter('i2c_bus').value)
        self.addr = int(self.get_parameter('i2c_address').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        freq = float(self.get_parameter('frequency').value)

        self.bus = SMBus(self.bus_num)
        self._setup_imu()
        self._setup_mag()

        self.pub_imu = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.pub_mag = self.create_publisher(MagneticField, 'imu/mag', 10)
        self.timer = self.create_timer(1.0 / freq, self._tick)
        self.get_logger().info(
            f'ICM-20948 ativo em /dev/i2c-{self.bus_num} @ 0x{self.addr:02x} ({freq:.0f} Hz)')

    def _bank(self, bank):
        self.bus.write_byte_data(self.addr, REG_BANK_SEL, bank << 4)

    def _setup_imu(self):
        self._bank(0)
        who = self.bus.read_byte_data(self.addr, WHO_AM_I)
        if who != 0xEA:
            self.get_logger().warn(f'WHO_AM_I = 0x{who:02x} (esperado 0xEA) - confira endereco/ligacao')
        self.bus.write_byte_data(self.addr, PWR_MGMT_1, 0x80)   # reset
        time.sleep(0.1)
        self.bus.write_byte_data(self.addr, PWR_MGMT_1, 0x01)   # acorda, clock automatico
        self.bus.write_byte_data(self.addr, PWR_MGMT_2, 0x00)   # accel + gyro ligados
        time.sleep(0.02)

        self._bank(2)
        self.bus.write_byte_data(self.addr, GYRO_CONFIG_1, 0x01)   # +-250 dps + DLPF
        self.bus.write_byte_data(self.addr, ACCEL_CONFIG, 0x01)    # +-2 g + DLPF

        self._bank(0)
        self.bus.write_byte_data(self.addr, USER_CTRL, 0x00)    # I2C master OFF
        self.bus.write_byte_data(self.addr, INT_PIN_CFG, 0x02)  # BYPASS_EN: mag aparece em 0x0C
        time.sleep(0.02)

    def _setup_mag(self):
        try:
            self.bus.write_byte_data(AK09916_ADDR, AK09916_CNTL3, 0x01)   # reset
            time.sleep(0.01)
            self.bus.write_byte_data(AK09916_ADDR, AK09916_CNTL2, 0x08)   # continuo 100 Hz
            time.sleep(0.01)
            self._mag_ok = True
        except OSError:
            self.get_logger().warn('AK09916 nao respondeu em 0x0C - publicando so accel/gyro')
            self._mag_ok = False

    def _tick(self):
        self._bank(0)
        d = self.bus.read_i2c_block_data(self.addr, ACCEL_XOUT_H, 12)
        now = self.get_clock().now().to_msg()

        imu = Imu()
        imu.header.stamp = now
        imu.header.frame_id = self.frame_id
        imu.linear_acceleration.x = _int16(d[0], d[1]) / ACCEL_SENS * G
        imu.linear_acceleration.y = _int16(d[2], d[3]) / ACCEL_SENS * G
        imu.linear_acceleration.z = _int16(d[4], d[5]) / ACCEL_SENS * G
        imu.angular_velocity.x = _int16(d[6], d[7]) / GYRO_SENS * DEG2RAD
        imu.angular_velocity.y = _int16(d[8], d[9]) / GYRO_SENS * DEG2RAD
        imu.angular_velocity.z = _int16(d[10], d[11]) / GYRO_SENS * DEG2RAD
        imu.orientation_covariance[0] = -1.0   # convencao: sem orientacao neste topico
        self.pub_imu.publish(imu)

        if not self._mag_ok:
            return
        st1 = self.bus.read_byte_data(AK09916_ADDR, AK09916_ST1)
        if not (st1 & 0x01):        # DRDY
            return
        # le HXL..HZH (little-endian) + ate ST2 (0x18); ler ST2 libera o registrador
        m = self.bus.read_i2c_block_data(AK09916_ADDR, AK09916_HXL, 8)
        mag = MagneticField()
        mag.header.stamp = now
        mag.header.frame_id = self.frame_id
        mag.magnetic_field.x = _int16(m[1], m[0]) * MAG_SENS * UT_TO_T
        mag.magnetic_field.y = _int16(m[3], m[2]) * MAG_SENS * UT_TO_T
        mag.magnetic_field.z = _int16(m[5], m[4]) * MAG_SENS * UT_TO_T
        self.pub_mag.publish(mag)


def main(args=None):
    rclpy.init(args=args)
    node = Icm20948Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
