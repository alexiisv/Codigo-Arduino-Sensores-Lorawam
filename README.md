# Código de Arduino para Sensores MLX90614, Nicla Sense ME, MAX30102, DFR0034 y SCD30

Este repositorio contiene el código de Arduino para la integración y uso de los sensores MLX90614, Nicla Sense ME, MAX30102, DFR0034 y SCD30.

## Sensores y Librerías Utilizadas

### MLX90614
- **Descripción**: Sensor de temperatura infrarrojo.
- **Librería Utilizada**: [Adafruit_MLX90614](https://github.com/adafruit/Adafruit-MLX90614-Library)

### Nicla Sense ME
- **Descripción**: Sensor de movimiento y ambiente multifuncional.
- **Librería Utilizada**: [Arduino_BHY2](https://github.com/arduino-libraries/Arduino_BHY2)

### MAX30102
- **Descripción**: Sensor de frecuencia cardíaca y oximetría.
- **Librería Utilizada**: [SparkFun MAX3010x Sensor Library](https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library)

### DFR0034
- **Descripción**: Sensor de sonido.
- **Librería Utilizada**: Sensor análogo, se lo lee con analogRead de arduino

### SCD30
- **Descripción**: Sensor de dióxido de carbono, temperatura y humedad.
- **Librería Utilizada**: [SparkFun_SCD30_Arduino_Library](https://github.com/sparkfun/SparkFun_SCD30_Arduino_Library)

## Uso

1. **Instalación de Librerías**: Antes de usar el código, asegúrate de instalar todas las librerías mencionadas a través del Administrador de Librerías de Arduino o utilizando el enlace proporcionado.
2. **Carga del Código**: Abre el archivo `.ino` correspondiente a tu proyecto en el IDE de Arduino.
3. **Configuración**: Ajusta los parámetros necesarios en el código para tu configuración de hardware específica.
   Configura las direcciones: 
5. **Conexión de Sensores**: Conecta los sensores a tu placa Arduino según se indica en el código y las especificaciones de cada sensor.
6. **Compilación y Carga**: Compila y carga el código en tu placa Arduino.

## Contribuciones

Las contribuciones son bienvenidas. Por favor, haz un fork del repositorio y envía un pull request con tus mejoras y correcciones.

## Licencia

Este proyecto está bajo la Licencia MIT. Para más detalles, consulta el archivo `LICENSE`.

---
## Autores 
Alexis Vallejo 
Johan Ordoñez

**Nota**: Asegúrate de revisar la documentación de cada librería para una integración correcta y detalles adicionales sobre el uso de los sensores.



