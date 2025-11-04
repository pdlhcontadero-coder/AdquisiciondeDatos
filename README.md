# ⚙️ Proyecto: Sistema de Monitoreo IoT aplicado a Cultivos de Lechuga Hidropónica

Este proyecto utiliza la **ESP 32** como microcontolador principal en 3 diferentes versiones. Con el fin de recolectar datos de variables criticas de los cultivos hidropónicos tales como: el pH, la conductividad electrica, la temperatura y humedad relativa, la temperatura del agua, el nidel de agua en tubos y nivel de agua en tanque.


## 🧰 Componentes utilizados
- 1 LilyGo T-SIM-A7670E
- 3 ESP-32 C3 Super Mini Plus
- 1 ESP-32 DEVKIT V1
- 1 Sensor de pH con electrodo industrial
- 1 Sensor de Conductividad Electrica
- 1 Sensor de Temperatura y Humedad DHT-22
- 2 Sensores de Temperatura DS18B20
- 4 Sensores de Nivel Analogicos
- 1 Sensor de Nivel Ultrasonico
- 2 ADS-1115
- 5 pantallas OLED 1.3'
- 5 Módulos de Carga - TP456
- 5 Paneles Solares 5V
- 10 baterías 18650 (2 por cada módulo)
- 9 Pulsadores (3 por cada módulo que los ocupa)
 
 ---

## Funcionamiento general
El sistema consiste de 5 módulos, 4 de ellos son emisores los cuales recolectan los datos de los sensores asignados para enviarlos al módulo receptor(principal) y el ultimo módulo, que es el receptor(principal), se encarga de recibir esos datos y enviarlo a la base de datos

---

## Diagramas e imágenes de los módulos

### Diagramas de conexión
![Diagrama de conexión módulo principal](imagenes/DiagramaPrinci.jpg)
![Diagrama de conexión módulo pH](imagenes/diagramaph.jpg)
![Diagrama de conexión módulo conductividad](imagenes/diagramaconducti.jpg)
![Diagrama de conexión módulo nivel de agua en tubos](imagenes/DiagramaNivelTubos.jpg)
![Diagrama de conexión módulo nivel de agua en tanque](imagenes/diagraniveltanque.jpg)

### Montaje real de los módulos
![Montaje del módulo principal](imagenes/Principal.jpg)
![Montaje del módulo pH](imagenes/CajapH.jpg)
![Montaje del módulo conductividad](imagenes/CajaConductividad.jpg)
![Montaje del módulo nivel de agua en tubos](imagenes/NiveldeTubos.jpg)
![Montaje del módulo nivel de agua en tanque](imagenes/NiveldeTanque.jpg)



## Autores
**Alejandro Díaz Igua**
**David Eraso García**
**Ana Sofía Muñoz Villota**
**Ivette Camila Yepez Moran**