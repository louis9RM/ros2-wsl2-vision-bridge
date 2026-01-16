# Guía Definitiva: Puente de Cámara Windows -> ROS 2 (Docker)

Esta guía documenta la solución funcional para transmitir video desde una cámara integrada en Windows hacia un contenedor Docker con ROS 2 corriendo en WSL 2, superando las limitaciones de conexión USB directa.

## 🧠 Arquitectura de la Solución

Utilizamos un enfoque **Cliente-Servidor TCP** para máxima estabilidad y compatibilidad con firewalls:

1.  **Windows (Servidor)**: Usa `ffmpeg` para capturar la cámara y levantar un servidor TCP en el puerto 5000. Espera conexiones.
2.  **Docker (Cliente)**: Usa un nodo ROS 2 con Python y GStreamer para conectarse a la IP de Windows y recibir el flujo de video.

---

## 🛠️ Requisitos Previos

1.  **FFmpeg en Windows**: Instalado via `winget install Gyan.FFmpeg`.
2.  **WSL 2 y Docker Desktop**: Instalados y funcionando.
3.  **Librerías en Docker**: El contenedor debe tener `gstreamer1.0-plugins-bad` y `gstreamer1.0-libav` (ya instalados en tu sesión).

---

## 📂 Archivos del Proyecto

Asegúrate de tener estos archivos en tu carpeta `ros2-wsl2-vision-bridge`:

### 1. `stream_camera_tcp.ps1` (Ejecutar en Windows)
Este script inicia el servidor de video.
**Nota Importante:** Escucha en `0.0.0.0` (todas las interfaces) para facilitar la conexión.

```powershell
# IP de WSL (Referencia visual)
$wsl_ip = (wsl hostname -I).Trim().Split(" ")[0]
Write-Host "Tu IP WSL es: $wsl_ip"

$port = 5000
Write-Host "Iniciando SERVIDOR TCP en el puerto $port..."
Write-Host "Esperando conexión desde Docker..."

# Servidor TCP (listen=1)
ffmpeg -f dshow -rtbufsize 100M -i video="HD User Facing" `
    -pixel_format yuyv422 -framerate 30 -video_size 640x480 `
    -c:v libx264 -preset ultrafast -tune zerolatency -pix_fmt yuv420p `
    -f mpegts "tcp://0.0.0.0:5000?listen=1"
```

### 2. `src/udp_camera_node.py` (Ejecutar en Docker)
*(Aunque se llame udp, ahora usa TCP).*
Este script se conecta a Windows y publica en ROS.

**⚠️ Configuración Crítica:** Debes editar la línea `host=...` con LA IP DE TU ADAPTADOR DE WINDOWS (vEthernet WSL).
Ejemplo actual: `172.28.192.1`.

```python
# Pipeline GStreamer (Cliente TCP)
gst_pipeline = (
    "tcpclientsrc host=172.28.192.1 port=5000 ! " # <--- TU IP DE WINDOWS AQUÍ
    "tsdemux ! "
    "h264parse ! "
    "avdec_h264 ! "
    "videoconvert ! "
    "appsink sync=false"
)
```

---

## 🚀 Cómo Iniciar el Sistema (Paso a Paso)

Debes seguir este orden estricto para que la conexión se establezca.

### Paso 1: Obtener IP de Windows (Solo si cambia)
Si reinicias el PC, tu IP de "vEthernet (WSL)" podría cambiar.
1.  En PowerShell: `ipconfig`
2.  Busca `Adaptador de Ethernet vEthernet (WSL)`.
3.  Si es diferente a `172.28.192.1`, edita `udp_camera_node.py`.

### Paso 2: Iniciar el Emisor (Windows)
En un PowerShell en la carpeta del proyecto:
```powershell
.\stream_camera_tcp.ps1
```
> **Estado:** Se quedará esperando con el mensaje "Esperando conexión...".

### Paso 3: Iniciar el Receptor (Docker)
En una terminal de Docker (`docker compose exec ros2_vision bash`):
```bash
# Cargar entorno ROS (si no está en .bashrc)
source /opt/ros/humble/setup.bash

# Ejecutar nodo
python3 src/udp_camera_node.py
```
> **Estado:** Debería decir "Connecting via TCP..." y luego el script de Windows empezará a mostrar datos de frames.

### Paso 4: Validar
En otra terminal de Docker:
```bash
ros2 topic hz /image_raw
```
Deberías ver una tasa estable (ej. 15-30 Hz).

---

## 🚑 Solución de Problemas Comunes

*   **Error "Connection refused"**:
    *   Causa: El script de Windows NO está corriendo o ya se cerró.
    *   Solución: Inicia primero Windows, luego Docker.

*   **Error "Could not open video stream" (Timeout)**:
    *   Causa: Firewall bloqueando o IP incorrecta.
    *   Solución:
        1. Verifica la IP en el py con `ipconfig`.
        2. Desactiva temporalmente el Firewall de Windows para probar.

*   **Video con mucho retraso (lag)**:
    *   Normal en WiFi. En cable es mejor.
    *   Intentar reducir resolución en el script `.ps1` a 320x240.
