# CleverFlightLib
Библиотека для искользования с COEX Clever для улучшения функционала и упрощения работы с автономным полётом
# Как использовать:
Слонируйте или загрузите этот репозиторий на RaspberryPi, установленную на коптер, поддерживающий автономный полёт с ПО Clever
```bash
git clone https://github.com/artem30801/CleverFlightLib
```
Найстройсте коптер и RaspberryPi в соответветствии с [официальной документацией от Copter Express](https://clever.copterexpress.com/).
Импортируйте модули FlightLib в ваш код для автономного полёта на питоне и инициализируйте ноду ROS с любым уникальным именем.
Пример:
```python
from FlightLib import FlightLib as drone
FlightLib.init('CleverFlight') 
from FlightLib import LedLib as led
```
Для более детальнной информации и справки загляните на [вики](https://github.com/artem30801/CleverFlightLib/wiki).
