# Stream usando TCP (Modo Servidor o Listen)
# Esto suele ser más robusto ante firewalls porque establece conexión confirmada.

# IP de WSL (para informar, pero en TCP listen escuchamos en todas)
$wsl_ip = (wsl hostname -I).Trim().Split(" ")[0]
Write-Host "Tu IP WSL es: $wsl_ip"

$port = 5000
Write-Host "Iniciando SERVIDOR TCP en el puerto $port..."
Write-Host "Esperando conexión desde Docker..."

# Explicación:
# -listen 1: FFmpeg actúa como servidor
# tcp://0.0.0.0:5000: Escucha en todas las interfaces
ffmpeg -f dshow -rtbufsize 100M -i video="HD User Facing" `
    -pixel_format yuyv422 -framerate 30 -video_size 640x480 `
    -c:v libx264 -preset ultrafast -tune zerolatency -pix_fmt yuv420p `
    -f mpegts "tcp://0.0.0.0:5000?listen=1"
