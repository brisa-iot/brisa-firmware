# Proyecto BRISA — Firmware para Boya Interconectada

Este repositorio contiene dos versiones del firmware para la boya BRISA, basadas en ESP32, diseñadas para monitoreo ambiental costero:

| Firmware             | Descripción breve                                           | Carpeta           |
|----------------------|------------------------------------------------------------|-------------------|
| **main-buoy-tested** | Versión simplificada para pruebas y validación básica LoRa.| [`main-buoy-tested`](./main-buoy-tested) |
| **main-buoy**        | Versión final con lógica avanzada y registro local.         | [`main-buoy`](./main-buoy)              |

> ⚠️ **Nota:** La versión **main-buoy** aún no ha sido probada completamente; solo se validaron las mediciones y la lógica de tareas para adquisición de datos. Cualquier integración de mas alto nivel y modularización no alcanzó a ser 'testeada' por completo.

---

## ¿Cuál versión debo usar?

- Usa **main-buoy-tested** para pruebas rápidas, validación inicial y debugging simple.
- Usa **main-buoy** para desarrollo avanzado, teniendo en cuenta que aún no está completamente probado.

---

## Cómo empezar

1. Entra a la carpeta correspondiente (`main-buoy-tested` o `main-buoy`).  
2. Revisa el README específico para instrucciones de compilación, flasheo, hardware y configuración.  
3. Sigue las indicaciones que allí se detallan para comenzar a trabajar con el firmware.

---

## Documentación y soporte

Para dudas o contribuciones, revisa los READMEs internos y el código fuente de cada firmware.  


---

¡Gracias por colaborar en el proyecto BRISA!
