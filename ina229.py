#============================================================================
#   Python Driver for TI INA229 85-Volt Voltage, Current and Power Monitor
#   by @Riky Suhendra 2024-01-29
#============================================================================

import time
import spidev

# INA229 Device Address
INA229_BUS = 0
INA229_DEV = 0
INA229_SHUNT_OHMS = 0.025
INA229_SHUNT_TEMPCO_VALUE = 0

#============================================================================
# INA229 Reset bytes for CONFIG [15] / [14]
# Decription: 
# RST - Generates a system reset that is the same as power-on reset.
# RSTACC - Resets the contents of accumulation registers ENERGY and CHARGE to 0
#============================================================================
INA229_RST_NBIT = 15
INA229_RSTACC_NBIT = 14

#============================================================================
# INA229 Initial conversion delay CONFIG [13:6]
# Description: Delay for initial ADC conversion in steps of 2 ms
# Values:
# 0x0 - 0 seconds (default)
# 0x1 - 2 ms
# 0xFF - 510ms
#============================================================================
INA229_CONVERSION_DELAY = 0
INA229_CONVDLY_NBIT = 6

#============================================================================
# INA229 Temperature compensation CONFIG [5]
# Description: Enables temperature compensation of an external shunt
# Values:
# 0x0 - disabled (default)
# 0x1 - enabled
#============================================================================
INA229_TEMP_COMP = 0
INA229_TEMPCOMP_NBIT = 5

#============================================================================
# INA229 ADC Shunt full scale range selection CONFIG [4]
# Description: Shunt full scale range selection across IN+ and IN–.
# Values:
# 0x0 - ±163.84 mV (default)
# 0x1 - ±40.96 mV
#============================================================================
INA229_ADCRANGE = 0
INA229_ADCRANGE_NBIT = 4

#============================================================================
# INA229 ADC Mode
# Description: The user can set the MODE bits for continuous or 
# triggered mode on bus voltage, shunt voltage or temperature measurement.
#
# Values:
# 0x0h = Shutdown
# 0x1h = Triggered bus voltage, single shot
# 0x2h = Triggered shunt voltage, single shot
# 0x3h = Triggered shunt voltage and bus voltage, single shot
# 0x4h = Triggered temperature, single shot
# 0x5h = Triggered temperature and bus voltage, single shot
# 0x6h = Triggered temperature and shunt voltage, single shot
# 0x7h = Triggered bus voltage, shunt voltage and temperature, single shot
# 0x8h = Shutdown
# 0x9h = Continuous bus voltage only
# 0xAh = Continuous shunt voltage only
# 0xBh = Continuous shunt and bus voltage
# 0xCh = Continuous temperature only
# 0xDh = Continuous bus voltage and temperature
# 0xEh = Continuous temperature and shunt voltage
# 0xFh = Continuous bus voltage, shunt voltage and temperature (default)
#============================================================================
INA229_ADC_MODE = 0xB
INA229_ADC_MODE_NBIT = 12

#============================================================================
# INA229 ADC conversion time of bus voltage meas.
# Description: Sets the conversion time of the bus voltage measurement
#
# Values:
# 0x00h = 50 μs
# 0x01h = 84 μs
# 0x02h = 150 μs
# 0x03h = 280 μs
# 0x04h = 540 μs
# 0x05h = 1052 μs
# 0x06h = 2074 μs
# 0x07h = 4120 μs
#============================================================================
INA229_VBUS_CONV_TIME = 0x05
INA229_VBUS_CONV_TIME_NBIT = 0x09

#============================================================================
# INA229 ADC conversion time of shunt voltage meas.
# Description: Sets the conversion time of the shunt voltage measurement
#
# Values:
# 0x00h = 50 μs
# 0x01h = 84 μs
# 0x02h = 150 μs
# 0x03h = 280 μs
# 0x04h = 540 μs
# 0x05h = 1052 μs
# 0x06h = 2074 μs
# 0x07h = 4120 μs
#============================================================================
INA229_VSHCT_CONV_TIME = 0x05
INA229_VSHCT_CONV_TIME_NBIT = 0x06

#============================================================================
# INA229 ADC conversion time of temperatrure meas.
# Description: Sets the conversion time of the temperatrure measurement
#
# Values:
# 0x00h = 50 μs
# 0x01h = 84 μs
# 0x02h = 150 μs
# 0x03h = 280 μs
# 0x04h = 540 μs
# 0x05h = 1052 μs
# 0x06h = 2074 μs
# 0x07h = 4120 μs
#============================================================================
INA229_VTCT_CONV_TIME = 0x05
INA229_VTCT_CONV_TIME_NBIT = 0x03


#============================================================================
# INA229 ADC sample averaging count. 
#
# Values: 
# 0x0h = 1
# 0x1h = 4
# 0x2h = 16
# 0x3h = 64
# 0x4h = 128
# 0x5h = 256
# 0x6h = 512
# 0x7h = 1024
#============================================================================
INA229_ADC_AVG = 3
INA229_AVG_NBIT = 0


#============================================================================
# INA229 ADC alerts signal
#============================================================================
INA229_ALERT_MEMSTAT = 0
INA229_ALERT_CNVRF = 1
INA229_ALERT_POL = 2
INA229_ALERT_BUSUL = 3
INA229_ALERT_BUSOL = 4
INA229_ALERT_SHNTUL = 5
INA229_ALERT_SHNTOL = 6
INA229_ALERT_TMPOL = 7
INA229_ALERT_MATHOF = 9
INA229_ALERT_CHARGEOF = 10
INA229_ALERT_ENERGYOF = 11
INA229_ALERT_APOL = 12
INA229_ALERT_SLOWALERT = 13
INA229_ALERT_CNVR = 14
INA229_ALERT_ALATCH = 15


class INA229:

    __INA229_CONFIG         = 0x00
    __INA229_ADC_CONFIG     = 0x01
    __INA229_SHUNT_CAL      = 0x02
    __INA229_SHUNT_TEMPCO   = 0x03
    __INA229_VSHUNT         = 0x04
    __INA229_VBUS           = 0x05
    __INA229_DIETEMP        = 0x06
    __INA229_CURRENT        = 0x07
    __INA229_POWER          = 0x08
    __INA229_ENERGY         = 0x09
    __INA229_CHARGE         = 0x0A
    __INA229_DIAG_ALRT      = 0x0B
    __INA229_SOVL           = 0x0C
    __INA229_SUVL           = 0x0D
    __INA229_BOVL           = 0x0E
    __INA229_BUVL               = 0x0F
    __INA229_TEMP_LIMIT         = 0x10
    __INA229_PWR_LIMIT          = 0x11
    __INA229_MANUFACTURER_ID    = 0x3E
    __INA229_DEVICE_ID          = 0x3F

    def __init__(self, busnum = INA229_BUS, devnum = INA229_DEV, shunt_ohms = INA229_SHUNT_OHMS):
        self._spi = spidev.SpiDev()
        self._spi.open(busnum,devnum)
        self._spi.max_speed_hz = 1000000
        self._spi.mode = 0b01
        self._shunt_ohms = shunt_ohms
    
    def __convert2comp2float(self, twocompdata, nrofbit, factor):
        isnegative = 1
        isnegative = (isnegative << (nrofbit - 1))

        dietemp = twocompdata

        if(dietemp > isnegative):
            dietemp = (dietemp - (2*isnegative)) * factor
        else:
            dietemp = (dietemp * factor)

        return dietemp

    def __binary_as_string(self, register_value):
        return bin(register_value)[2:].zfill(16)

    def __to_bytes(self, register_value):
        return [(register_value >> 8) & 0xFF, register_value & 0xFF]

    def read_register40(self, register):
        regb = bytearray(6)
        regb[0] = (register << 2) | 1
        result = self._spi.xfer2(regb)      
        register_value = ((result[1] << 32) & 0xFF00000000) | ((result[2] << 24) & 0xFF000000) | ((result[3] << 16) & 0xFF0000) | ((result[4] << 8) & 0xFF00) | (result[5] & 0xFF)
        return register_value

    def read_register24(self, register):
        regb = bytearray(4)
        regb[0] = (register << 2) | 1
        result = self._spi.xfer2(regb)
        print(result)
        register_value = ((result[1] << 16) & 0xFF0000) | ((result[2] << 8) & 0xFF00) | (result[3] & 0xFF)
        return register_value

    def read_register16(self, register):
        regb = bytearray(3)
        regb[0] = (register << 2) | 1
        result = self._spi.xfer2(regb)
        print(result)
        register_value = ((result[1] << 8) & 0xFF00) | (result[2] & 0xFF)
        return register_value
    
    def write_register16(self, register, register_value):
        register_bytes = self.__to_bytes(register_value)
        regb = bytearray(3)
        regb[0] = register << 2
        regb[1] = register_bytes[0]
        regb[2] = register_bytes[1]
        self._spi.writebytes(regb)

    def get_current_lsb(self):
        if(INA229_ADCRANGE == 0):
            temp = 163.84e-3
        else:
            temp = 40.96e-3
        current_lsb = (temp / self._shunt_ohms) / 524288
        return current_lsb

    def get_shunt_conv_factor(self):
        if(INA229_ADCRANGE == 0):
            shunt_conv_factor = 1.25e-6
        else:
            shunt_conv_factor = 5.0e-6
        return shunt_conv_factor

    def reset_all(self):
        config = self.read_register16(self.__INA229_CONFIG)
        data = 1 << INA229_RST_NBIT
        config = config | data
        self.write_register16(self.__INA229_CONFIG, config)
  
    def reset_energy(self):
        config = self.read_register16(self.__INA229_CONFIG)
        data = 1 << INA229_RSTACC_NBIT
        config = config | data
        self.write_register16(self.__INA229_CONFIG, config)

    def set_config(self):
        # Write settings to CONFIG register
        config = self.read_register16(self. __INA229_CONFIG)
        config = config | (INA229_CONVERSION_DELAY << INA229_CONVDLY_NBIT) | (INA229_TEMP_COMP << INA229_TEMPCOMP_NBIT) | (INA229_ADCRANGE << INA229_ADCRANGE_NBIT)
        self.write_register16(self.__INA229_CONFIG, config)

    def set_adc_config(self):
        # Write settings to ADC CONFIG register
        config = self.read_register16(self. __INA229_ADC_CONFIG)
        config = config | (INA229_ADC_MODE << INA229_ADC_MODE_NBIT) | (INA229_VBUS_CONV_TIME << INA229_VBUS_CONV_TIME_NBIT ) | (INA229_VSHCT_CONV_TIME << INA229_VSHCT_CONV_TIME_NBIT) | (INA229_VTCT_CONV_TIME << INA229_VTCT_CONV_TIME_NBIT) | (INA229_ADC_AVG << INA229_AVG_NBIT)
        self.write_register16(self.__INA229_ADC_CONFIG, config)
    
    def shunt_calib(self):
        calib_value = int(13107.2e6 * self.get_current_lsb() * self._shunt_ohms)
        self.write_register16(self.__INA229_SHUNT_CAL, calib_value)
    
    def shunt_tempco(self):
        self.write_register16(self.__INA229_SHUNT_TEMPCO, INA229_SHUNT_TEMPCO_VALUE)
    
    def configure(self):
        self.reset_all()
        self.set_config()
        self.set_adc_config()
        self.shunt_calib()
        self.shunt_tempco()
        time.sleep(0.1)
    
    def get_shunt_voltage(self):
        if(INA229_ADCRANGE == 1):
            conversion_factor = 312.5e-9                # nV/LSB
        else:
            conversion_factor = 78.125e-9               # nV/LSB  

        raw = self.read_register24(self.__INA229_VSHUNT)

        vshunt = (self.__convert2comp2float(raw >> 4, 20, conversion_factor)) * 10                  # Find and fix *10
        return float(vshunt)

    def get_vbus_voltage(self):      
        conversion_factor = 195.3125e-6                 # uV/LSB
        raw = self.read_register24(self.__INA229_VBUS)
        vbus = self.__convert2comp2float(raw >> 4, 20, conversion_factor)
        return float(vbus) #* self.__BUS_MILLIVOLTS_LSB / 1000

    def get_temp_voltage(self):
        conversion_factor = 7.8125e-3
        raw = self.read_register16(self.__INA229_DIETEMP)
        temp = self.__convert2comp2float(raw, 16, conversion_factor)
        return float(temp)
    
    def get_current(self):        
        raw = self.read_register24(self.__INA229_CURRENT)
        current = self.__convert2comp2float(raw >> 4, 20, self.get_current_lsb())
        return float(current)

    def get_power(self):
        current_lsb = self.get_current_lsb() 
        raw = self.read_register24(self.__INA229_POWER)
        power = (3.2 * raw * current_lsb)       
        return float(power)

    def get_energy(self):
        raw = self.read_register40(self.__INA229_ENERGY)
        energy = raw * 3.2 * 16 * self.get_current_lsb()
        return float(energy)
    
    def get_charge(self):
        raw = self.read_register40(self.__INA229_CHARGE)
        return (raw)
    
    def get_diag_alerts(self, alert):
        raw = self.read_register16(self.__INA229_DIAG_ALRT)
        if(alert == INA229_ALERT_ALATCH):
            if (raw & 0x1) == 0x0:
                #print('MEMSTAT: Checksum error is detected in the device trim memory space')
                return 0x1

        elif(alert == INA229_ALERT_CNVRF):
            if (raw & 0x2) == 0x2:
                #print('CNVRF: Conversion is completed')
                return 0x2

        elif(alert == INA229_ALERT_BUSUL):
            if (raw & 0x4) == 0x4:
                #print('BUSUL: Bus voltage measurement falls below the threshold limit in the bus under-limit register')
                return 0x4

        elif(alert == INA229_ALERT_BUSOL):
            if (raw & 0x8) == 0x8:
                #print('BUSOL: Bus voltage measurement exceeds the threshold limit in the bus over-limit register')
                return 0x8

        elif(alert == INA229_ALERT_SHNTUL):
            if (raw & 0x10) == 0x10:
                #print('SHNTUL: Shunt voltage measurement falls below the threshold limit in the shunt under-limit register')
                return 0x10

        elif(alert == INA229_ALERT_SHNTOL):
            if (raw & 0x40) == 0x40:
                #print('SHNTOL: Shunt voltage measurement exceeds the threshold limit in the shunt over-limit register')
                return 0x40

        elif(alert == INA229_ALERT_TMPOL):
            if (raw & 0x80) == 0x80:
                #print('TMPOL: Temperature measurement exceeds the threshold limit in the temperature over-limit register')
                return 0x80

        elif(alert == INA229_ALERT_MATHOF):
            if (raw & 0x100) == 0x100:
                #print('MATHOF: Arithmetic operation resulted in an overflow error')
                return 0x100

        elif(alert == INA229_ALERT_CHARGEOF):
            if (raw & 0x200) == 0x200:
                #print('CHARGEOF: 40 bit CHARGE register has overflowed')
                return 0x200

        elif(alert == INA229_ALERT_ENERGYOF):
            if (raw & 0x400) == 0x400:
                #print('ENERGYOF: 40 bit ENERGY register has overflowed')
                return 0x400

        elif(alert == INA229_ALERT_APOL):
            if (raw & 0x800) == 0x800:
                #print('APOL: Alert pin polarity inverted (active-high, open-drain)')
                return 0x800
            else:
                print('APOL: Alert pin polarity normale (active-low, open-drain)')
                return 0x0

        elif(alert == INA229_ALERT_SLOWALERT):
            if (raw & 0x2000) == 0x2000:
                #print('SLOWALERT: ALERT function is asserted on the completed averaged value. ALERT comparison on averaged value')
                return 0x2000
            else:
                #print('SLOWALERT: ALERT comparison on non-averaged (ADC) value')
                return 0x0

        elif(alert == INA229_ALERT_CNVR):
            if (raw & 0x4000) == 0x1:
                #print('CNVR: Alert pin to be asserted when the Conversion Ready Flag (bit 1) is asserted, indicating that a conversion cycle has completed. Enables conversion ready flag on ALERT pin')
                return 0x0
            else:
                #print('CNVR: Disable conversion ready flag on ALERT pin')
                return 0x0

        elif(alert == INA229_ALERT_ALATCH):
            if (raw & 0x8000) == 0x8000:
                #print('ALATCH: Latched')
                return 0x8000
            else:
                #print('ALATCH: Transparent')
                return 0x0

    def set_shunt_overvoltage(self, value):
        if (value >= 0):
            data = (value * self._shunt_ohms) / self.get_shunt_conv_factor()
        else:
            value_temp = value * (-1);
            data = (value_temp * self._shunt_ohms) / self.get_shunt_conv_factor()
            data = ~data
            data = data + 1
        self.read_register16(self.__INA229_SOVL)
        self.write_register16(self.__INA229_SOVL, data)
        
    def set_shunt_undervoltage(self, value):
        if (value >= 0):
            data = (value * self._shunt_ohms) / self.get_shunt_conv_factor()
        else:
            value_temp = value * (-1);
            data = (value_temp * self._shunt_ohms) / self.get_shunt_conv_factor()
            data = ~data
            data = data + 1
        self.read_register16(self.__INA229_SUVL)
        self.write_register16(self.__INA229_SUVL, data)
    
    def set_bus_overvoltage(self, value):
        data = value / (16 * 195.3125e-6)
        self.read_register16(self.__INA229_BOVL)
        self.write_register16(self.__INA229_BOVL, data)

    def set_bus_undervoltage(self, value):
        data = value / (16 * 195.3125e-6)
        self.read_register16(self.__INA229_BUVL)
        self.write_register16(self.__INA229_BUVL, data)

    def set_temp_limit(self, value, consta):
        data = value / (16 * consta)
        self.read_register16(self.__INA229_TEMP_LIMIT)
        self.write_register16(self.__INA229_TEMP_LIMIT, data)
    
    def set_power_overlimit(self, value, consta):
        data = value / (16 * consta)
        self.read_register16(self.__INA229_PWR_LIMIT)
        self.write_register16(self.__INA229_PWR_LIMIT, data)

    def get_manufacturer_id(self):
        raw_id = self.read_register16(self.__INA229_MANUFACTURER_ID)
        print('Manufacturer ID (HEX): ', hex(raw_id))
        first_byte = (raw_id >> 8) & 0xFF
        second_byte = (raw_id & 0xFF)
        print('Manufacturer ID (CHAR): ', chr(first_byte),chr(second_byte))
    
    def get_deviceid(self):
        raw_id = self.read_register16(self.__INA229_DEVICE_ID)
        print('Device ID: ', hex(raw_id >> 4))
        print('Revision: ',  hex(raw_id & 0xF))
