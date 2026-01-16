# ROS 2 Vision Bridge (WSL 2 + Docker)

Este proyecto configura un entorno de ROS 2 Humble en Docker capaz de acceder a la cámara web de tu laptop a través de WSL 2.

## Requisitos Previos

1.  **WSL 2** instalado y funcionando.
2.  **Docker Desktop** (con integración WSL 2 activada).
3.  **usbipd-win** instalado (ver `guia_usbipd_wsl2.md`).

## Instrucciones de Uso

### 1. Conectar la Cámara a WSL
Antes de encender el contenedor, debes pasar la cámara de Windows a Linux.
*(Consulta la guía detallada en `guia_usbipd_wsl2.md` si tienes dudas)*.

En PowerShell (Admin):
```powershell
usbipd bind --busid <TU-BUSID>
```

Luego, **con una terminal de WSL abierta**, conecta el dispositivo:
```powershell
usbipd attach --wsl --busid <TU-BUSID>
```

### 2. Iniciar el Contenedor Docker
Una vez que `lsusb` en Linux muestra tu cámara, levanta el contenedor desde la carpeta del proyecto:

```bash
docker compose up -d --build
```

### 3. Probar la Cámara en ROS 2
Entra al contenedor:

```bash
docker compose exec ros2_vision bash
```

Dentro del contenedor, ejecuta el nodo de cámara:

```bash
ros2 run v4l2_camera v4l2_camera_node
```

Si todo está correcto, el nodo publicará imágenes en `/image_raw`. Puedes verificarlo en otra terminal (dentro del contenedor) con:

```bash
ros2 topic list
ros2 topic hz /image_raw
```
