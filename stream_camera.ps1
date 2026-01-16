# Script para transmitir video desde Windows a Docker (ROS 2) con baja latencia

# Detectar IP de WSL (necesaria para enviar UDP desde Windows al interior de WSL)
# Detectar IP de WSL (Probamos obtenerla de Ubuntu explícitamente)
try {
    # Intentamos conectar a Ubuntu
    $wsl_ip = (wsl -d Ubuntu-22.04 hostname -I).Trim().Split(" ")[0]
} catch {
    # Si falla, intentamos la distro por defecto pero con ip addr parsing
    $wsl_ip = (wsl ip addr show eth0 | Select-String "inet ").ToString().Split(" ")[5].Split("/")[0]
}

if (-not $wsl_ip) {
    Write-Host "Error: No se pudo detectar la IP de WSL. Asegúrate de que WSL está corriendo." -ForegroundColor Red
    exit
}

Write-Host "Iniciando transmisión de cámara 'HD User Facing'..."
Write-Host "IP destino (WSL): $wsl_ip"
Write-Host "Destino: udp://${wsl_ip}:5000"
Write-Host "Presiona 'q' en esta ventana para detener."

# Explicación:
# -f dshow: Usar DirectShow (Windows)
# -i video="...": Nombre de tu cámara
# -preset ultrafast: Priorizar latencia
# -tune zerolatency: Optimizar para streaming
# -f mpegts: Formato robusto para UDP
# pkt_size=1316: Tamaño de paquete estándar para MPEG-TS

ffmpeg -f dshow -rtbufsize 100M -i video="HD User Facing" `
    -pixel_format yuyv422 -framerate 30 -video_size 640x480 `
    -c:v libx264 -preset ultrafast -tune zerolatency -pix_fmt yuv420p `
    -f mpegts udp://${wsl_ip}:5000?pkt_size=1316
