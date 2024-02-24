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

    controller[None] = 0

    _max = len(controller)
    _time_step = 1000

    # Демка при подключенных 16-ти СИД с общим АНОДОМ(!)
    print("---Cтупенчатое увеличение яркости всех СИД---")
    for i in range(_max):
        pwm = int(100 * (i) / _max)
        controller[None] = pwm
        print(f"ШИМ [%]: {pwm}")
        time.sleep_ms(_time_step)
    #
    controller[None] = False  # гашу все СИД
    print("---Включаю СИД через один---")
    for _ in range(_max):
        for index in range(0, _max, 2):
            controller[index] = True  # горят четные СИД
        time.sleep_ms(_time_step)
        controller[None] = False  # гашу все СИД
        for index in range(1, _max, 2):
            controller[index] = True  # горят НЕчетные СИД
        time.sleep_ms(_time_step)
        controller[None] = False  # гашу все СИД

    old_index = -1
    print("---Бегущий огонь---")
    for index in range(_max):
        if old_index >= 0:
            controller[old_index] = False
        controller[index] = True
        old_index = index
        time.sleep_ms(_time_step)

    for index in range(_max - 1, 0, -1):
        if old_index >= 0:
            controller[old_index] = False
        controller[index] = True
        old_index = index
        time.sleep_ms(_time_step)

    controller[None] = False  # гашу все СИД
