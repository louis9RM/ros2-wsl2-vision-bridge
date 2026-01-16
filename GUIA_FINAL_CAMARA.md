# Guía de Uso: Puente de Video Windows -> ROS 2

Esta guía explica cómo utilizar el sistema de transmisión de video ya configurado en este repositorio.

El objetivo es permitir que un contenedor Docker (corriendo en WSL 2) acceda a la cámara web conectada a Windows, utilizando una conexión de red TCP robusta.

---

## 🛠️ Requisitos Previos e Instalación

Si estás configurando esto desde cero en una máquina nueva, necesitas instalar:

### 1. WSL 2 (Subsistema de Linux)
Si no lo tienes, abre PowerShell como Administrador y ejecuta:
```powershell
wsl --install
```
*(Reinicia tu PC si te lo pide).*

### 2. Docker Desktop
Es el motor que correrá ROS 2.
*   **Descargar**: [https://www.docker.com/products/docker-desktop/](https://www.docker.com/products/docker-desktop/)
*   Durante la instalación, asegúrate de marcar la opción de **WSL 2 backend**.

### 3. FFmpeg (En Windows)
Necesario para capturar la cámara. En PowerShell ejecuta:
```powershell
winget install Gyan.FFmpeg
```

---

## 🚀 Instrucciones de Inicio

Sigue estos 3 pasos cada vez que quieras trabajar con la cámara.

### Paso 0: Construir y Levantar el Contenedor (Primera vez)

Si acabas de clonar el proyecto (o si reiniciaste tu PC), necesitas levantar el contenedor primero:

1.  Abre una terminal en la carpeta del proyecto.
2.  Ejecuta:
    ```powershell
    docker compose up -d --build
    ```
    *Esto descargará ROS 2, instalará las librerías y dejará el sistema listo.*

### Paso 1: Configurar la IP (Solo si cambia)

Dado que Windows y WSL son redes distintas, el contenedor necesita saber a qué dirección conectarse.

1.  Abre PowerShell en Windows y ejecuta: `ipconfig`
2.  Busca el adaptador **"vEthernet (WSL)"** y copia su **Dirección IPv4** (ej: `17x.2x.19x.1`).
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
2.  Entra al contenedor con este comando:
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

1.  Abre otra terminal de PowerShell.
2.  **Entra al contenedor:**
    ```powershell
    docker compose exec ros2_vision bash
    ```
3.  **Ejecuta el verificador:**
    ```bash
    source /opt/ros/humble/setup.bash
    ros2 topic hz /image_raw
    ```
    *Deberías ver una tasa de frames constante (ej. average rate: 15.322).*

---

## 🏗️ Diagrama de Arquitectura

Puedes visualizar el flujo de datos con este digrama:

```mermaid
graph TD
    subgraph Windows_Host [Windows 11 Host]
        style Windows_Host fill:#0078D4,color:white
        Webcam(("/dev/video (Hardware Camera)"))
        FFmpeg["FFmpeg Process<br/>(stream_camera_tcp.ps1)"]
        
        Webcam -->|DirectShow| FFmpeg
        FFmpeg -.->|TCP Listen :5000| Port5000((Port 5000))
    end

    subgraph WSL2_Network [WSL 2 Network Bridge]
        style WSL2_Network fill:#f9f,stroke:#333,stroke-width:2px,stroke-dasharray: 5 5
        IP_Link["vEthernet Link<br/>(172.28.x.1)"]
    end

    subgraph Docker_Container [Docker Container: ros2_vision]
        style Docker_Container fill:#d63384,color:white
        PythonNode["ROS 2 Node<br/>(udp_camera_node.py)"]
        GStreamer["GStreamer Pipeline<br/>(tcpclientsrc)"]
        ROS_Topic["/image_raw"]

        Port5000 -->|TCP Connect| IP_Link
        IP_Link -->|Data Stream| GStreamer
        GStreamer -->|Decode H.264| PythonNode
        PythonNode -->|Publish Image| ROS_Topic
    end
```

---

## 🔍 Explicación de Componentes

Aquí detallamos qué hace cada pieza del sistema y por qué es necesaria:

### 1. FFmpeg (En Windows)
*   **Función**: Actúa como el "cerebro" que controla la cámara física. Windows tiene los drivers perfectos para la webcam (algo que falla en WSL), así que usamos FFmpeg para leer la cámara y convertir el video en datos digitales puros (H.264).
*   **Por qué TCP**: Configuramos FFmpeg en modo "Listen" (Escucha), creando un servidor en el puerto 5000. Esto simula ser una cámara IP.

### 2. WSL 2 Network Bridge (vEthernet)
*   **Función**: Es el cable de red virtual que conecta tu sistema Windows con el "mundo Linux" de WSL 2.
*   **El Reto**: Por defecto, WSL 2 tiene su propia IP. Por eso necesitamos decirle al cliente (Docker) que busque la IP del host (Windows), que es la dirección `172.x.x.1` que configuramos.

### 3. GStreamer (En Docker)
*   **Función**: Es una librería multimedia muy potente dentro de Linux.
*   **Rol**: Recibe los paquetes de datos "crudos" que llegan por la red, los ordena y los decodifica para convertirlos en imágenes que ROS pueda entender. Es mucho más robusto que OpenCV para video en streaming.

### 4. Nodo ROS 2 (`udp_camera_node.py`)
*   **Función**: Es el intermediario final.
    *   Toma la imagen descifrada por GStreamer.
    *   La convierte a un mensaje de ROS 2 (`sensor_msgs/Image`).
    *   La publica en el topic `/image_raw` para que cualquier otro nodo de tu robot (visión artificial, SLAM, etc.) pueda usarla.
