# Guía de Uso: Puente de Video Windows -> ROS 2

Esta guía explica cómo utilizar el sistema de transmisión de video ya configurado en este repositorio.

El objetivo es permitir que un contenedor Docker (corriendo en WSL 2) acceda a la cámara web conectada a Windows, utilizando una conexión de red TCP robusta.

---

## 📋 Requisitos Previos

Antes de ejecutar nada, asegúrate de tener instalado en Windows:

1.  **FFmpeg**: Necesario para capturar la cámara.
    *   Instalar con PowerShell: `winget install Gyan.FFmpeg`
2.  **Docker Desktop + WSL 2**: Ya configurados.

---

## 🚀 Instrucciones de Inicio

Sigue estos 3 pasos cada vez que quieras trabajar con la cámara.

### Paso 1: Configurar la IP (Solo si cambia)

Dado que Windows y WSL son redes distintas, el contenedor necesita saber a qué dirección conectarse.

1.  Abre PowerShell en Windows y ejecuta:
    ```powershell
    ipconfig
    ```
2.  Busca el adaptador **"vEthernet (WSL)"** y copia su **Dirección IPv4** (ej: `172.28.192.1`).
3.  Abre el archivo `src/udp_camera_node.py` en este repo.
4.  Busca la línea que dice `tcpclientsrc host=...` y pega tu IP ahí.

### Paso 2: Iniciar el Servidor de Video (Windows)

Este script le dice a Windows: "Enciende la cámara y espera que alguien te pida el video".

1.  En PowerShell, ve a la carpeta del proyecto.
2.  Ejecuta:
    ```powershell
    .\stream_camera_tcp.ps1
    ```
3.  **Resultado esperado**: Verás un mensaje "Iniciando SERVIDOR TCP..." y se quedará esperando. **No cierres esta ventana.**

### Paso 3: Iniciar el Receptor ROS 2 (Docker)

Este comando inicia el nodo dentro del contenedor que "llama" a Windows para pedirle el video.

1.  Abre una terminal nueva.
2.  Entra al contenedor:
    ```powershell
    docker compose exec ros2_vision bash
    ```
3.  (Opcional) Si ROS no carga, ejecuta: `source /opt/ros/humble/setup.bash`
4.  Ejecuta el nodo:
    ```bash
    python3 src/udp_camera_node.py
    ```

Si todo conecta, la ventana de Windows (Paso 2) empezará a mostrar datos de transmisión.

---

## ✅ Verificación

Para confirmar que ROS 2 está recibiendo las imágenes:

1.  Abre otra terminal y entra al contenedor.
2.  Ejecuta:
    ```bash
    ros2 topic hz /image_raw
    ```
    *Deberías ver una tasa de frames constante (ej. 15 Hz).*

---

## ❓ ¿Por qué usamos este método?

*   **USB Passthrough (usbipd)**: Es el método estándar, pero en muchos casos falla con WSL 2, provocando que la cámara se congele o bloquee el kernel Linux.
*   **TCP Streaming (Este método)**: Es universal. Windows maneja el hardware (que lo hace muy bien) y simplemente envía los datos digitales por "cable de red virtual" a Docker. Es mucho más estable y no requiere drivers especiales en Linux.
