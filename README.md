## Контроллер для АКПП Toyota A650E (Aisin 35-50LS)

Контроллер на базе платы Arduino Mega2560, но написан на Си без Ардуино костылей.
Пока прошивка еще не готова...


Для сборки прошивки на Windows необходимо скачать avr-gcc по ссылке
https://github.com/ZakKemble/avr-gcc-build/releases/download/v8.3.0-1/avr-gcc-8.3.0-x86-mingw.zip, распаковать в C:\avr-gcc и запустить Make.bat.

Чтобы залить прошивку надо в MakefileWin указать свой COM порт и выполнить команду "Make.bat flash".