# PIMP
Модуль памяти для "Электроника МК-90" на основе отладочной платы Raspberry Pi Pico.

**Возможности:**
- Простое копирование образов картриджей - при подключении к ПК модуль работает как флешка (1,8МБ);
- При подключении к МК-90 модуль монтирует autorun.bin или первый попавшийся образ и работает как обычный МПО/СМП (только на чтение);
- Поддержка (только на чтение) протокола [Genjitsu SMP](https://github.com/vladkorotnev/SMPEmu/tree/master/pdp) благодаря чему можно загрузить несколько образов и выбирать нужный из списка непосредственно на МК90.

**Что понадобится:**
- Сама отладочная плата Raspberry Pi Pico;
- 3 резистора и 3-4 диода (желательно Шотки).

**Схема подключения:**

Красивая:
<img src="/hw/scheme_bb.png?raw=true" width="100%">

Понятная:
<img src="/hw/scheme.png?raw=true" width="100%">

Для начала, нужно прошить Pico - для этого надо зажать единственную кнопку на плате, не отпуская ее подключить к ПК, плата определится как накопитель, на который копируем [pimp.uf2](/build/pimp.uf2). На этом прошивка закончена, модуль начнет определяться как Mass Storage Device, его нужно отформатировать в FAT с дефолтными параметрами, в итоге получится "флешка" на ~1,80МБ, на которую можно копировать образы. Если модуль отформатировался нормально и на него скопирован какой-либо образ, можно подключать его к МК-90 по указанной выше схеме.

Cхема относительно простая но не очень правильная :) из-за инжекции дополнительного тока в источник питания Pico. Плюс к этой проблеме есть еще одна - МК-90 не любит нагрузку в выключенном состоянии (а при лучшем варианте Pico потребляет ~700мкА) поэтому, при разряженных батарейках, с подлюченным PIMP, МК-90 может не запускаться. Пользуемся на свой страх и риск, чуть позже опубликую более сложный, но правильный вариант.

**Мультикартридж**

Для возможности выбора образа из нескольких непосредственно на МК-90, достаточно скопировать [загрузчик](https://github.com/vladkorotnev/SMPEmu/blob/master/pdp/smp0.bin) на модуль, переименовав в autorun.bin и конечно скопировать нужные образы.
