# Пример управления ИС PCA9685. Читайте тех. документацию.
# СИД - СветоИзлучающий Диод

# Проблемы:
# >>> controller[15]=99		СИД горит ярко
# >>> controller[15]=100	СИД НЕ горит! Ошибка
# >>> controller[15]=0		СИД горит ярко! Ошибка

from machine import I2C, Pin
import pca9685mod
import time
from sensor_pack.bus_service import I2cAdapter

if __name__ == '__main__':
    i2c = I2C(id=1, scl=Pin(7), sda=Pin(6), freq=400_000)  # on Raspberry Pi Pico
    adapter = I2cAdapter(i2c)
    controller = pca9685mod.Pca9685(adapter)
    addr_names = 'SUBADR1', 'SUBADR2', 'SUBADR3', 'ALLCALLADR'
    print("Под адрес разрешен?:")
    for id_addr in range(4):
        enbl = controller.is_sub_addr_enabled(id_addr)
        print(f"под адрес: {addr_names[id_addr]}: разрешен: {enbl}")
        if enbl:
            controller.enable_sub_addr(id_addr, not enbl)

    print("Значения под адресов:")
    for id_addr in range(4):
        print(f"под адрес: {addr_names[id_addr]}: значение: 0x{controller.get_sub_addr(id_addr):x}")
    print("---предделитель---")
    print(f"Текущее значение предделителя: 0x{controller.prescaler:x}")
    print(f"Внешнее тактирование: {controller.external_clock}")
    print(f"Режим сна: {controller.sleep_mode}")
    print(f"Выходные каскады СИД инвертированы? {controller.is_out_inverted()}")
    print(f"Выходные каскады СИД по схеме с открытым стоком? {controller.is_out_open_drain()}")
    print(32 * "-")
    print(f"Значения ШИМ выходов СИД для 0..15")
    print(f"ШИМ [%]: {controller[range(16)]}")
    # print(f"ШИМ [%]: {controller[None]}")		# то же самое, что и предидущая строка кода
    # настройка выхода для кодключения СИД через токоограничивающий резистор, анодом к +питание!
    # если у вас при controller[номер_канала] = 0 СИД этого канала горит ярко, то измените параметр
    # inverted в вызове configure_led_out
    controller.configure_led_out(inverted=True, open_drain=True)
