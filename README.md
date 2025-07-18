## Контроллер для АКПП Toyota A650E (Aisin 35-50LS)

Контроллер на базе платы Arduino Mega2560, но написан на AVR C без Ардуино костылей.
Реализован весь необходимый функционал для работы АКПП и протестирован на реальной АКПП.

На данный момент (17.09.2024), я считаю, мой алгоритм управления готов на 90%, остались всякие тонкие настройки, которые мало влияют на процесс, но требуют много времени для настройки.

В повседневной эксплуатации АКПП работает хорошо, переключения плавные, пинков нет, все режимы селектора работают корректно.

В папке docs лежит вся найденная информация, в том числе инструкция по ремонту A650E с исправленным порядком страниц.

upd 14.04.2025

Драйв2:

Авто - https://www.drive2.ru/r/lada/2104/595999075002745340/

Статья про ЭБУ - https://www.drive2.ru/l/678402798580157637/

Статья по алгоритму управления - https://www.drive2.ru/l/685587832189755146/

Что-то я как-то оптимистично написал, что проект готов на 90%, видимо от радости, что все поехало более менее нормально. И вообще под «пинается» каждый может понимать своё, например, я катался на паре авто с заводским автоматом, так там переключения вообще не ощущаются. У меня переключения явно чувствуются, все таки, связи с двигателем нету, но это скорее не пинок, а резкое ускорение, как при отпускании сцепления на механике.

В общем, проблемы с управлением еще есть:
1) Иногда не достаточно плавно происходит переключение 1>2.
2) Аналогично с повторным включением второй передачи после ХХ.
3) Переключение 2>3 на большой мощности – происходить закусывание.

Я потихоньку допиливаю код, иногда могу залить сырое и не проверенное обновление, чисто для сохранения для себя разных версий.
Актуальные графики настроек хранятся у меня в EEPROM ЭБУ.



Для сборки прошивки на Windows необходимо скачать avr-gcc по ссылке
https://github.com/ZakKemble/avr-gcc-build/releases/download/v8.3.0-1/avr-gcc-8.3.0-x86-mingw.zip, распаковать в C:\avr-gcc и запустить Make.bat.

Чтобы залить прошивку надо в MakefileWin указать свой COM порт и выполнить команду "Make.bat flash".