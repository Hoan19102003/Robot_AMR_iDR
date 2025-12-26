import serial
import time
import struct

# ================= MODBUS DEFINES =================
CMD_RIR   = 0x04   # Read Input Registers
CMD_RHR   = 0x03   # Read Holding Registers
CMD_WSR   = 0x06   # Write Single Register
CMD_REST  = 0x42   # Reset Energy

# Registers
REG_VOLTAGE   = 0x0000
REG_CURRENT   = 0x0001
REG_POWER     = 0x0002
REG_ENERGY    = 0x0004
REG_HVALARM   = 0x0006
REG_LVALARM   = 0x0007

# ==================================================

class PZEM017:
    def __init__(self, port="COM3", addr=0x01):
        self.addr = addr
        self.ser = serial.Serial(
            port=port,
            baudrate=9600,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_TWO,  # 🔥 8N2 BẮT BUỘC
            timeout=1
        )

    # ---------- CRC16 MODBUS ----------
    @staticmethod
    def crc16(data: bytes):
        crc = 0xFFFF
        for b in data:
            crc ^= b
            for _ in range(8):
                if crc & 1:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc

    # ---------- SEND MODBUS FRAME (8 BYTES) ----------
    def send_cmd8(self, cmd, reg, val):
        frame = bytearray(8)
        frame[0] = self.addr
        frame[1] = cmd
        frame[2] = (reg >> 8) & 0xFF
        frame[3] = reg & 0xFF
        frame[4] = (val >> 8) & 0xFF
        frame[5] = val & 0xFF

        crc = self.crc16(frame[:-2])
        frame[6] = crc & 0xFF
        frame[7] = (crc >> 8) & 0xFF

        self.ser.write(frame)
        time.sleep(0.05)  # 🔥 quan trọng với RS485

    # ---------- READ ALL VALUES ----------
    def read_values(self):
        # giống updateValues() trong C++
        self.send_cmd8(CMD_RIR, 0x0000, 0x0008)
        resp = self.ser.read(21)

        if len(resp) != 21:
            raise Exception("No response")

        # CRC check
        crc_rx = resp[-2] | (resp[-1] << 8)
        crc_calc = self.crc16(resp[:-2])
        if crc_rx != crc_calc:
            raise Exception("CRC error")

        voltage = (resp[3] << 8 | resp[4]) / 100.0
        current = (resp[5] << 8 | resp[6]) / 100.0
        power   = ((resp[9] << 24) | (resp[10] << 16) |
                   (resp[7] << 8) | resp[8]) / 10.0
        energy  = ((resp[13] << 24) | (resp[14] << 16) |
                   (resp[11] << 8) | resp[12]) / 1000.0
        hv_alarm = (resp[15] << 8 | resp[16])
        lv_alarm = (resp[17] << 8 | resp[18])

        return {
            "voltage": voltage,
            "current": current,
            "power": power,
            "energy": energy,
            "hv_alarm": hv_alarm,
            "lv_alarm": lv_alarm
        }

    # ---------- RESET ENERGY ----------
    def reset_energy(self):
        frame = bytearray([self.addr, CMD_REST, 0x00, 0x00])
        crc = self.crc16(frame)
        frame += struct.pack("<H", crc)
        self.ser.write(frame)

    def close(self):
        self.ser.close()
