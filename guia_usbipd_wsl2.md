# Guía de Conexión de Cámara Web a WSL 2 con usbipd

Esta guía detalla los pasos para conectar una cámara web integrada (u otro dispositivo USB) a una instancia de WSL 2 utilizando `usbipd-win`.


## 0. Requisitos e Instalación

## 0. Requisitos Previos

### A. Verificar WSL (Subsistema de Linux)
Si no estás seguro de tener Linux instalado, abre PowerShell y ejecuta `wsl`. Si obtienes un error, instálalo con este comando y reinicia tu PC:

```powershell
wsl --install
```

### B. Instalar usbipd-win
La forma más fácil de instalar es usando Winget. Abre PowerShell y ejecuta:

```powershell
winget install usbipd-win
```
*Nota: Es posible que necesites reiniciar la terminal o el equipo después de la instalación.*

## 1. Diagnóstico Inicial


Primero, debemos identificar el dispositivo USB que queremos compartir. Abre **PowerShell** y ejecuta el siguiente comando:

```powershell
usbipd list
```

Busca tu dispositivo en la lista. Deberías ver algo similar a esto, donde el estado indica que aún no está compartido:

```text
BUSID  VID:PID    DEVICE                                            STATE
2-9    04f2:b6dd  Integrated Camera, camerasound III                Not shared
```

> [!NOTE]
> Toma nota del **BUSID** (en este caso `2-9`) y el **VID:PID** (`04f2:b6dd`). El estado `Not shared` confirma que el dispositivo es visible para Windows pero no accesible para WSL todavía.

## 2. Configuración de Permisos

Para permitir que el dispositivo sea compartido, debes "vincularlo" (share/bind). Este paso requiere permisos de administrador.

Abre **PowerShell como Administrador** y ejecuta:

```powershell
usbipd bind --busid 2-9
```

Esto configura la persistencia del dispositivo, permitiendo que sea compartido.

## 3. Conexión del Dispositivo

Para conectar la cámara, **es obligatorio tener la terminal de Linux abierta**. Si no lo haces, `usbipd` no tendrá dónde "inyectar" el dispositivo.

1.  **Abre tu terminal de Linux** (Ubuntu/Debian).
    *   Puedes buscar "Ubuntu" en el menú Inicio.
    *   O ejecutar `wsl` en una nueva pestaña de PowerShell.
    *   **¡Déjala abierta!**

2.  En tu ventana de **PowerShell (Admin)**, ejecuta el comando de conexión:

    ```powershell
    usbipd attach --wsl --busid 2-9
    ```

Si todo funciona bien, el comando no mostrará errores y terminará silenciosamente (o mostrará "Attached").

## 4. Validación Final

Para confirmar que la cámara está correctamente conectada y reconocida por Linux:

1.  Ve a tu terminal de **Linux**.
2.  Ejecuta el comando `lsusb`.

```bash
lsusb
```

Deberías ver una salida donde tu dispositivo aparece, típicamente en un bus virtual (como el Bus 001):

```text
Bus 001 Device 002: ID 04f2:b6dd Integrated Camera
```

> [!SUCCESS]
> Si ves el dispositivo listado con su ID correcto (`04f2:b6dd`), ¡la conexión ha sido exitosa! Ahora puedes usar la cámara desde tus aplicaciones en WSL 2.
