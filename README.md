# Автономный полет БПЛА мультироторного типа для поиска возможных очагов возгорания
img1 pass

## Команда проекта:
```
Лев Синюков - разработчик модуля автономного полета
Алексндр Поляков - специалист по компьютерному зрению
Егор Лаптев - архитектор бд
```

### В рамках данного проекта были использованы такие методы и технологии:

```
3D симулятор для робототехники Gazebo - https://gazebosim.org/home;

Программируемый квадрокоптер COEX Clover4 c полетным стеком PX4 и Raspberry Pi 4 в качестве управляющего бортового компьютера;

Программирование на Python;

Технология(библиотека) компьютерного зрения OpenCV;

Кроссплатформенная среда для разработки Node.js;

Фреймворк для создание ботов в телеграмм Aiogram;

Работа с базами данных (sqlite)
```

### Алгоритм работы:

Импортируем все необходимые библиотеки:

```
import json
import requests
from datetime import datetime
from pprint import pprint
import math 

from pyzbar.pyzbar import decode

import rospy 
from clover import srv 
from sensor_msgs.msg import Image 
from std_srvs.srv import Trigger 

import cv2 
from cv_bridge import CvBridge
```

Объявляем ноду:

```
rospy.init_node('flight') 
```

### Инициализируем ROS ноду и добавляем необходимые для полета прокси
```rospy.init_node('flight') 
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry) 
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger) 

bridge = CvBridge() 
```

### Создаем публикаторов в топики для отладки
```
color = rospy.Publisher('test', Image, queue_size=1)
hsv_topic = rospy.Publisher('hsv', Image, queue_size=1)
orig_topic = rospy.Publisher('orig', Image, queue_size=1)
```
### Необходимые константы
```
FLIGHT_HEIGHT = 1
FLIGHT_SPEED = 0.7
NUM_FRAME_CHECK = 10
MIN_CNT_AREA = 200
HSV_RED_FILTER = ((0, 120, 196), (20, 167, 255))
```

### URL для публикации данных в БД
```URL = "https://drone-stats.onrender.com/api/logs"```

### Алгоритм работы:
1. Взлет. После взлета записываются стартовые координаты и определяется qr код, в котором зашифрованы координаты точек маршрута (подробнее прочитать про работу с qr и компьютерным зрением - ```https://clover.coex.tech/ru/camera.html```)
2. Разрабатывается оптимальный маршрут для полета из полученых точек.
3. По прилете в каждую точку вызываются функции для определения возгораний, резальтат исследования местности в данной позиции сохраняется.
4. После облета всех точек из маршрута дрон возвращается на стартовую позици.
5. Отчет полета передается на сервер.
6. Посадка дрона.
7. Передача данных с сервера в базу данных бота.