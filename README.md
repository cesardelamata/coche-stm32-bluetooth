#  Coche teledirigido con STM32 y control Bluetooth (USART)
Coche teledirigido con la placa base STM32L, con control por Bluetooth (USART), sensores ultrasónicos y motores controlados mediante PWM para las ruedas.

Proyecto realizado desde cero: control de motores DC con PWM, comunicación inalámbrica por Bluetooth (USART), sensores ultrasónicos delantero/trasero y señal acústica con buzzer.  
Incluye modo **manual** (comandos por Bluetooth) y **automático** (evita obstáculos y adapta velocidad).

---

##  Objetivos
- Microcontrolador **STM32** programado en C.
- **PWM** para control de motores DC.
- **Bluetooth (USART)** para control remoto desde móvil/PC.
- **Sensores ultrasónicos** para medir distancia y evitar obstáculos.
- **Buzzer** con tono intermitente/continuo según proximidad.
- Implementar dos modos:
  - **Manual** → control con comandos.
  - **Automático** → evasión de obstáculos con control adaptativo de la velocidad.

---

##  Funcionamiento
- El coche recibe órdenes por **Bluetooth** (ejemplo: `GO`, `BACK`, `IZQ`, `DER`, `STP`).
- Los motores se controlan con señales **PWM** desde el STM32.
- Los sensores ultrasónicos miden distancias:  
  - < 30 cm → reduce velocidad.  
  - < 10 cm → entra en **modo búsqueda de salida** (giros programados).  
  - < 5 cm → se detiene y buzzer continuo.  
- Buzzer como alarma acústica: intermitente o continuo según la proximidad.  
- Cambio de modo con comandos: `MODE_MANUAL`, `MODE_AUTO`.

---

## Estructura del repositorio
- `src/` → código fuente en C (STM32).
- `docs/` → documentación y report del proyecto.
- `videos/` → demo en vídeo (añadir aquí, si <100 MB).

---

## Comandos por Bluetooth
```text
MODE_MANUAL     # activa control manual
MODE_AUTO       # activa conducción automática
MIN10           # establece distancia mínima en 10 cm
MAX25           # establece distancia máxima en 25 cm
GO              # avanzar
BACK            # retroceder
IZQ             # girar izquierda
DER             # girar derecha
STP             # detener
