# Guía de Conexión de Cámara Web a WSL 2 con usbipd

Esta guía detalla los pasos para conectar una cámara web integrada (u otro dispositivo USB) a una instancia de WSL 2 utilizando `usbipd-win`.

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

## 3. El Error de Instancia Cerrada

Este es un punto crítico donde muchos fallan. Si intentas conectar el dispositivo (`attach`) **sin tener una terminal de WSL 2 (Linux) abierta**, recibirás un error.

Intento fallido (sin terminal Linux abierta):

```powershell
usbipd attach --wsl --busid 2-9
```

**Resultado del error:**
```text
usbipd: error: There is no WSL 2 distribution running.
```

> [!WARNING]
> **Error Común**: Este error ocurre porque `usbipd` necesita una instancia de WSL activa para inyectar el dispositivo USB. No basta con tener WSL instalado; el sistema "invitado" debe estar corriendo.

## 4. Solución y Conexión

Para evitar el error anterior, sigue estos pasos en orden:

1.  **Abre tu terminal de Linux (Ubuntu/Debian, etc.)**. Mantén esta ventana abierta.
2.  Regresa a tu ventana de **PowerShell** (puede ser la de usuario normal o admin).
3.  Ejecuta el comando de conexión nuevamente:

```powershell
usbipd attach --wsl --busid 2-9
```

Si todo funciona correctamente, no verás ningún mensaje de error y el cursor volverá a la línea de comandos, o verás un mensaje de éxito dependiendo de la versión.

## 5. Validación Final

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
