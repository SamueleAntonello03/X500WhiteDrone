import subprocess
import signal
import sys

def start_rtsp_to_udp(rtsp_url, udp_port):
    cmd = [
        "ffmpeg",
        "-rtsp_transport", "udp",   # usa TCP per RTSP, più stabile
        "-i", rtsp_url,
        "-c", "copy",               # copia stream senza ricodifica
        "-f", "rtp",
        f"udp://0.0.0.0:{udp_port}"
    ]

    print(f"Avvio FFmpeg: {cmd}")
    process = subprocess.Popen(cmd)

    return process

if __name__ == "__main__":
    rtsp_url = "rtsp://192.168.144.25:8554/main.264"  # Cambia con il tuo indirizzo RTSP
    udp_port = 5600

    process = start_rtsp_to_udp(rtsp_url, udp_port)

    print("Premi Ctrl+C per fermare")

    try:
        # Attendi che il processo ffmpeg termini (o Ctrl+C)
        process.wait()
    except KeyboardInterrupt:
        print("\nTerminazione in corso...")
        process.send_signal(signal.SIGINT)
        process.wait()
        print("Processo FFmpeg terminato.")
        sys.exit(0)
