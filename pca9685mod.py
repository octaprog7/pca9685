# micropython
# mail: goctaprog@gmail.com
# MIT license
from sensor_pack import bus_service
from sensor_pack.base_sensor import Device, Iterator, check_value, all_none
import time
from micropython import const
from struct import pack_into

_bit_12 = const(0b1_0000_0000_0000)
_bit_0_11 = const(0b1111_1111_1111)
_ticks = const(4096)


def _check_id_subaddr(id_sub_addr: int):
    err_info = f"Неверное значение id_addr: {id_sub_addr}"
    if id_sub_addr is None:
        raise ValueError(err_info)
    check_value(id_sub_addr, range(4), err_info)


def _get_on_off(pwm_duty_cycle: int) -> tuple:
    """Возвращает значения регистров ШИМ канала по коэффициенту заполнения ШИМ в процентах 0..100"""
    check_value(pwm_duty_cycle, range(101), f"Неверное значение pwm_duty_cycle: {pwm_duty_cycle}")
    on_delay = 0
    tmp = 0.01 * (_ticks - 1)
    off_delay = on_delay + int(pwm_duty_cycle * tmp)
    #       on delay    off delay, full on, full off
    return on_delay, off_delay, 100 == pwm_duty_cycle, 0 == pwm_duty_cycle


def get_duty_cycle(on_delay: int, off_delay: int) -> int:
    """Возвращает коэффициент заполнения ШИМ в процентах 0..100 по значениям регистров ШИМ канала"""
    res = int(100 * (off_delay - on_delay) / _ticks)
    # print(f"DBG: _get_on_off: {on_delay}\t{off_delay}\tduty_cycle: {res}")
    return res


def get_prescaler(pwm_freq: int, clock_frequency: int = 25_000_000) -> int:
    """Возвращает значение предделителя по желаемой частоте ШИМ pwm_freq [Гц]
    и тактовой частоте clock_frequency [Гц]."""
    if pwm_freq not in range(24, 1527):
        raise ValueError(f"Частота ШИМ вне допустимых пределов: {pwm_freq}")
    return int(round(2 ** -12 * clock_frequency / pwm_freq - 1, 0))


def _get_led_address(index: [int, None]) -> tuple:
    """возвращает адреса регистров выходов в виде кортежа: (LEDxx_ON, LEDxx_OFF).
    Каждый регистр двухбайтный!!!"""
    if index is None:   # для управления всеми выходами одновременно!
        return 0xFA, 0xFC
    check_value(index, range(16), f"Неверный индекс СИД: {index}")
    tmp = 4 * index
    return 6 + tmp, 8 + tmp


class Pca9685(Device, Iterator):
    """Class for work with I2C-bus controlled 16-channel LED controller. Internal 25 MHz oscillator.
    The active LOW Output Enable input pin (OE) allows asynchronous control of the LED outputs and can be used to
    set all the outputs to a defined I2C-bus programmable logic state."""

    def __init__(self, adapter: bus_service.BusAdapter, address=0x40):
        """i2c - объект класса I2C; address - адрес датчика на шине"""
        check_value(address, range(0x40, 0x80), f"Неверное значение адреса I2C устройства: {address:x}")
        super().__init__(adapter, address, False)
        # включаю внутреннее тактирование, автоинкремент адреса, нормальный рабочий режим
        self._mode_1(None, False, True, False)
        self._buf_4 = bytearray((0 for _ in range(4)))  # для _read_buf_from_mem

    def _read_reg(self, reg_addr, bytes_count=2) -> bytes:
        """считывает из регистра датчика значение.
        bytes_count - размер значения в байтах"""
        return self.adapter.read_register(self.address, reg_addr, bytes_count)

    # BaseSensor
    def _write_reg(self, reg_addr, value: int, bytes_count=2) -> int:
        """записывает данные value в датчик, по адресу reg_addr.
        bytes_count - кол-во записываемых данных"""
        byte_order = self._get_byteorder_as_str()[0]
        return self.adapter.write_register(self.address, reg_addr, value, bytes_count, byte_order)

    def _read_buf_from_mem(self, address: int, buf) -> bytes:
        """Читает из устройства, начиная с адреса address в буфер.
        Кол-во читаемых байт равно "длине" буфера в байтах!"""
        self.adapter.read_buf_from_mem(self.address, address, buf)
        return buf

    def _write_buf_to_mem(self, address: int, buf):
        """Пишу буфер в датчик"""
        return self.adapter.write_buf_to_mem(self.address, address, buf)

    def _mode_1(
            self,
            restart: [bool, None] = None,  # bit 7,
            ext_clk: [bool, None] = None,  # bit 6, разрешение внешнего тактирования
            ai: [bool, None] = None,  # bit 5, register auto-increment
            sleep: [bool, None] = None,  # bit 4, low power mode
            sub_1: [bool, None] = None,  # bit 3, разрешает I2C-bus sub address 1
            sub_2: [bool, None] = None,  # bit 2, разрешает I2C-bus sub address 2
            sub_3: [bool, None] = None,  # bit 1, разрешает I2C-bus sub address 3
            all_call: [bool, None] = None,  # bit 0, разрешает all call sub address
    ) -> int:
        """MODE1 register. Если все параметры в None, возвращает содержимое MODE1"""
        val = self._read_reg(0x00)[0]
        if all_none(restart, ext_clk, ai, sleep, sub_1, sub_2, sub_3, all_call):
            return val
        if restart is not None:
            val &= ~(1 << 7)  # mask
            val |= restart << 7
        if ext_clk is not None:
            val &= ~(1 << 6)  # mask
            val |= ext_clk << 6
        if ai is not None:
            val &= ~(1 << 5)  # mask
            val |= ai << 5
        if sleep is not None:
            val &= ~(1 << 4)  # mask
            val |= sleep << 4
        if sub_1 is not None:
            val &= ~(1 << 3)  # mask
            val |= sub_1
        if sub_2 is not None:
            val &= ~(1 << 2)  # mask
            val |= sub_2
        if sub_3 is not None:
            val &= ~1  # mask
            val |= sub_3
        self._write_reg(0x00, val, 1)

    def _mode_2(
            self,
            invrt: [bool, None] = None,
            # bit 4, инвертировать (Истина) или нет логическое состояние выхода. Применимо, когда OE = 0.
            och: [bool, None] = None,
            # bit 3, состояние выходов меняются при ACK (Истина), состояние выходов меняются при команде STOP.
            outdrv: [bool, None] = None,
            # bit 2, (Ложь) 16 выходов LEDn имеют структуру с открытым стоком, иначе totem pole structure
            outne: [int, None] = None,  # 0: Когда OE = 1 (выходные драйверы не включены), LEDn = 0.
            # 1: Когда OE = 1 (драйверы вывода не включены):
            #                           LEDn = 1, когда OUTDRV = 1
            #                           LEDn=высокий импеданс, когда OUTDRV = 0
            # 2: Когда OE = 1 (выходные драйверы не включены), LEDn = высокий импеданс.
    ) -> int:
        """MODE2 register. Если все параметры в None, возвращает содержимое MODE2"""
        val = self._read_reg(0x01)[0]
        if all_none(invrt, och, outdrv, outne):
            return val
        check_value(outne, range(3), f"Неверное значение outne: {outne}")
        if invrt is not None:
            val &= ~(1 << 4)  # mask
            val |= invrt << 4
        if och is not None:
            val &= ~(1 << 3)  # mask
            val |= och
        if outdrv is not None:
            val &= ~(1 << 2)  # mask
            val |= outdrv
        if outne is not None:
            val &= ~0b11  # mask
            val |= outne
        self._write_reg(0x01, val, 1)

    def _from_slice(self, source: [slice, range]) -> range:
        """Преобразует срез в диапазон"""
        if not isinstance(source, (range, slice)):
            raise TypeError(f"Неверный тип параметра: {type(source)}")
        if isinstance(source, range):
            return source
        return range(*source.indices(len(self)))    #

    def _all_addr(self, id_sub_addr: int, value: [int, None] = None) -> [int, None]:
        """Чтение или запись регистров SUBADR1..3, ALLCALLADR.
        id_addr: 0 - SUBADR1,
        1 - SUBADR2,
        2 - SUBADR3,
        3 - ALLCALLADR.
        Если value is None, происходит чтение из регистра и возврат преобразованного результата.
        Если value not is None, происходит запись в регистр с преобразованием значения
        """
        _check_id_subaddr(id_sub_addr)
        addr = 2 + id_sub_addr
        val = self._read_reg(addr)[0]
        if value is None:  # чтение
            return val >> 1
        check_value(value, range(0x80), f"Неверное значение {value:x} для адреса {addr:x}")  # проверка перед записью
        self._write_reg(addr, value << 1, 1)

    def enable_sub_addr(self, id_sub_addr: int, value: bool = True):
        """Разрешает (Истина) или запрещает (Ложь) реагировать чип/микросхему на запись по шине
        на этот адрес (id_sub_addr).
        id_addr: 0 - SUBADR1, 1 - SUBADR2, 2 - SUBADR3, 3 - ALLCALLADR"""
        _check_id_subaddr(id_sub_addr)
        if 0 == id_sub_addr:
            self._mode_1(None, None, None, None, value)
        if 1 == id_sub_addr:
            self._mode_1(None, None, None, None, None, value)
        if 2 == id_sub_addr:
            self._mode_1(None, None, None, None, None, None, value)
        if 3 == id_sub_addr:
            self._mode_1(None, None, None, None, None, None, None, value)

    def is_sub_addr_enabled(self, id_sub_addr: int) -> bool:
        """Возвращает Истина, если чип/микросхема реагирует на запись по шине по этому адресу (id_sub_addr).
        id_addr: 0 - SUBADR1, 1 - SUBADR2, 2 - SUBADR3, 3 - ALLCALLADR"""
        _check_id_subaddr(id_sub_addr)
        val = 0b1111 & self._mode_1()
        n_bit = 3 - id_sub_addr
        return 0 != (1 << n_bit) & val

    def _pre_scaler(self, value: [int, None] = None) -> int:
        """Чтение/запись значения предделителя в регистр, для изменения частоты ШИМ.
        Предделитель = округлить(тактовая_частота_генератора/(4096*Частота_ШИМ)).
        Значение регистра предделителя может быть изменено только когда бит MODE1.SLEEP в единице!"""
        if value is None:
            return self._read_reg(0xFE, 1)[0]  # чтение предделителя
        # установка предделителя
        check_value(value, range(3, 0x100), f"Неверное значение предделителя: {value}")
        self._write_reg(0xFE, value, 1)

    @property
    def prescaler(self) -> int:
        return self._pre_scaler(None)

    def set_pwm_freq(self, freq: int, clock_frequency: int = 25_000_000) -> int:
        """Устанавливает частоту ШИМ, Гц.
        Возвращает новое значение предделителя!"""
        pre_scaler = get_prescaler(freq, clock_frequency)
        try:
            self.sleep_mode = True  # переход в режим сна
            self._pre_scaler(pre_scaler)
            return pre_scaler
        finally:
            self.sleep_mode = False
            time.sleep_us(500)

    def get_sub_addr(self, id_addr: int) -> int:
        """Возвращает значение дополнительных адресов I2C по их id.
        id_addr: 0 - SUBADR1, 1 - SUBADR2, 2 - SUBADR3, 3 - ALLCALLADR."""
        return self._all_addr(id_addr)

    def set_sub_addr(self, id_addr: int, value: int):
        """Устанавливает значение дополнительных адресов I2C по их id.
        id_addr: 0 - SUBADR1, 1 - SUBADR2, 2 - SUBADR3, 3 - ALLCALLADR."""
        self._all_addr(id_addr, value)

    def _set_out(self, index: [int, None], on_val: int, off_val: int, full_on: bool, full_off: bool):
        """Устанавливает пару значений регистров (LEDx_ON, LEDx_OFF), для соответствующего выхода ИС!
        index - индекс выхода вывода/ножки ИС, 0..15.
        Если index is None, то управляются все выходы одновременно!
        on_val содержит момент включения выхода в 1, начиная от начала периода ШИМ. Измеряется от 0 до 4095.
        Это значение счетчика задержки включения канала.
        off_val - содержит момент включения выхода в 0, начиная от начала периода ШИМ. Измеряется от 0 до 4095.
        full_on - если истина, то требуется ВКлючить СИД на 100 % периода ШИМ!
        full_off - если истина, то требуется ВЫключить СИД  на 100 % периода ШИМ!
        """
        _on_val, _off_val = on_val, off_val
        if full_on:
            _on_val = _bit_12
        if full_off:
            _off_val = _bit_12

        on_addr, _ = _get_led_address(index)
        buf = self._buf_4
        _format = f"{self._get_byteorder_as_str()[1]}HH"
        # пакую в буфер
        pack_into(_format, buf, 0, _on_val, _off_val)
        self._write_buf_to_mem(on_addr, buf)

    def _get_out(self, index: int) -> tuple:
        """Возвращает пару значений регистров (LEDx_ON, LEDx_OFF), для соответствующего выхода ИС!
        index - индекс выхода вывода/ножки ИС, 0..15.
        LEDx_ON содержит момент включения выхода в 1, начиная от начала периода ШИМ. Измеряется от 0 до 4095.
        Это значение счетчика задержки включения канала.
        LEDx_OFF - содержит момент включения выхода в 0, начиная от начала периода ШИМ. Измеряется от 0 до 4095.
        """
        on_addr, _ = _get_led_address(index)
        buf = self._buf_4
        self._read_buf_from_mem(on_addr, buf)   # чтение в буфер
        on_val, off_val = self.unpack("HH", buf)    # распаковка
        full_on = on_val & _bit_12
        full_off = off_val & _bit_12
        if full_off:
            full_on = False     # приоритетнее!
        return on_val & _bit_0_11, off_val & _bit_0_11, full_on, full_off      # маскирование и возврат

    def _get_out_duty_cycle(self, index: int) -> int:
        """Возвращает коэффициент заполнения ШИМ в процентах 0..100 по индексу ШИМ канала, 0..15"""
        on, off, full_on, full_off = self._get_out(index)
        if full_on:
            return 100
        if full_off:
            return 0
        return get_duty_cycle(on, off)

    def _set_out_duty_cycle(self, index: [int, None], duty_cycle: int):
        on, off, full_on, full_off = _get_on_off(duty_cycle)
        self._set_out(index, on, off, full_on, full_off)

    def __getitem__(self, key: [int, range, slice, None]) -> [int, tuple]:
        """возврат значения времени включенного состояния канала(ов) в % от периода ШИМ по его индексу или диапазону.
        key может иметь тип: int(0..15), range, slice, None.
        В случае, если key range или slice, метод возвращает кортеж значений ШИМ каналов, определенных key!
        Если key is None, то происходит возврат кортежа значений ШИМ всех каналов!"""
        if key is None:
            return tuple([self._get_out_duty_cycle(item) for item in range(len(self))])
        if isinstance(key, int):
            return self._get_out_duty_cycle(key)
        rng = self._from_slice(key)    # range из slice
        result = [self._get_out_duty_cycle(item) for item in rng]
        return tuple(result)

    def __setitem__(self, key: [None, int, range, slice], val: [int, bool]):
        """назначение значения времени включенного состояния канала(ов) в % от периода ШИМ по его индексу.
        Если key is None, происходит присвоение одного значения val всем каналам!"""
        def _convert(source: [int, bool]) -> int:
            """Преобразует source в int"""
            if isinstance(source, bool):
                return 100 * source
            return source

        _val = _convert(val)
        if key is None:
            self._set_out_duty_cycle(key, _val)
            return
        if not isinstance(val, (bool, int)):
            raise TypeError(f"Неверный тип значения % от периода ШИМ!")
        if isinstance(key, int):
            self._set_out_duty_cycle(key, _val)
            return
        rng = self._from_slice(key)  # range из slice
        for index in rng:   # присвоение в цикле
            self._set_out_duty_cycle(index, _val)

    def __len__(self) -> int:
        return 0x10     # 16 СИД

    @property
    def external_clock(self) -> bool:
        """Возвращает Истина, когда включено внешнее тактирование микросхемы, иначе тактирование внутреннее 25 МГц"""
        return 0 != 0b0100_0000 & self._mode_1()

    @external_clock.setter
    def external_clock(self, value: bool):
        """Устанавливает внешнее (Истина) или внутреннее (Ложь) тактирование микросхемы"""
        self._mode_1(None, value)

    @property
    def sleep_mode(self) -> bool:
        """Возвращает Истина, когда включен режим сна, иначе нормальный режим.
        Переход из режима сна в нормальный рабочий режим занимает 500 мкс!"""
        return 0 != 0b0001_0000 & self._mode_1()

    @sleep_mode.setter
    def sleep_mode(self, value: bool):
        """Устанавливает режим сна (Истина) или нормальный (Ложь) режим"""
        self._mode_1(None, None, None, value)

    def configure_led_out(self, inverted: bool, open_drain: bool, high_impedance: bool = True):
        """Настраивает выходной каскад, управляющий всеми СИД.
        Если inverted в Истина, логическое состояние выхода инвертировано. Значение, используемое,
            когда внешний драйвер не используется. Применимо, когда OE = 0.
        Если open_drain в Истина, выходные каскады СИД имеют структуру с открытым стоком! Включаем СИД нулём!!!
        Если open_drain в Ложь, выходные каскады СИД имеют структуру totem pole. Включаем СИД единицей!!!"""
        self._mode_2(inverted, None, not open_drain, 2 if high_impedance else 0)

    def is_out_inverted(self) -> bool:
        val = self._mode_2()
        return 0 != 0b1_0000 & val

    def is_out_open_drain(self) -> bool:
        val = self._mode_2()
        return not (0 != 0b0100 & val)

    def __del__(self):
        del self._buf_4
