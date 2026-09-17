#!/usr/bin/env python3
"""
pixy_plotter.py - Script Python pentru vizualizarea și analiza datelor de la camera Pixy2
transmise de placa NXP către ESP32.

Utilizare:
  1. Vizualizare LIVE prin Wi-Fi de la ESP32:
     python pixy_plotter.py --live --ip 192.168.4.1

  2. Analiză offline dintr-un fișier JSON descărcat din ESP32:
     python pixy_plotter.py --file nxp_telemetry_2026-08-31.json
"""

import argparse
import json
import time
import urllib.request
import matplotlib.pyplot as plt

PIXY_WIDTH = 78
PIXY_HEIGHT = 51

def plot_pixy_frame_2d(record):
    """Trasează cadrul 2D al camerei Pixy2 (78x51 px) cu liniile de ghidaj și coordonatele (x0, y0) -> (x1, y1)."""
    fig, ax = plt.subplots(figsize=(8, 5))
    ax.set_xlim(0, PIXY_WIDTH)
    ax.set_ylim(PIXY_HEIGHT, 0) # Origine Y sus conform spațiului camerei Pixy
    ax.set_title(f"Pixy2 Camera Frame 2D - Stare: {record.get('which', 'NONE')} (Linii: {record.get('lines', 0)})", fontsize=12, fontweight='bold')
    ax.set_xlabel("X (0 .. 78 px)")
    ax.set_ylabel("Y (0 .. 51 px - 51 este aproape de mașină)")
    ax.grid(True, linestyle='--', alpha=0.5)

    # Linia mediană a mașinii
    ax.axvline(39, color='gray', linestyle=':', label='Centru Vehicul (X=39)')

    # Linia Stânga (lx0, ly0) -> (lx1, ly1)
    lx0, ly0 = record.get('lx0', 0), record.get('ly0', 0)
    lx1, ly1 = record.get('lx1', 0), record.get('ly1', 0)
    if lx0 or ly0 or lx1 or ly1:
        ax.plot([lx0, lx1], [ly0, ly1], color='tab:blue', linewidth=3, marker='o', label=f'Linie Stânga: ({lx0},{ly0})->({lx1},{ly1})')
        ax.text(lx0 + 1, ly0, f'L0({lx0},{ly0})', color='tab:blue', fontsize=9, fontweight='bold')
        ax.text(lx1 + 1, ly1, f'L1({lx1},{ly1})', color='tab:blue', fontsize=9, fontweight='bold')

    # Linia Dreaptă (rx0, ry0) -> (rx1, ry1)
    rx0, ry0 = record.get('rx0', 0), record.get('ry0', 0)
    rx1, ry1 = record.get('rx1', 0), record.get('ry1', 0)
    if rx0 or ry0 or rx1 or ry1:
        ax.plot([rx0, rx1], [ly0, ry1], color='tab:green', linewidth=3, marker='o', label=f'Linie Dreaptă: ({rx0},{ry0})->({rx1},{ry1})')
        ax.text(rx0 + 1, ry0, f'R0({rx0},{ry0})', color='tab:green', fontsize=9, fontweight='bold')
        ax.text(rx1 + 1, ry1, f'R1({rx1},{ry1})', color='tab:green', fontsize=9, fontweight='bold')

    # Unghiul de Virare / Steering Arrow
    steer = record.get('steer', 0.0)
    ax.text(39, 48, f'Steer: {steer:+.1f}°', color='orange', fontsize=11, fontweight='bold', ha='center')

    ax.legend(loc='upper right')
    plt.tight_layout()
    plt.show()

def plot_json_file(file_path):
    """Încarcă un fișier JSON exportat din ESP32 și generează diagrame temporale și statistici."""
    print(f"Încărcare date din: {file_path}")
    with open(file_path, 'r', encoding='utf-8') as f:
        data = json.load(f)

    records = data.get('records', [])
    if not records:
        print("Nu s-au găsit înregistrări în fișierul JSON.")
        return

    timestamps = [r.get('ts', 0) / 1000.0 for r in records]
    steers = [r.get('steer', 0.0) for r in records]
    num_vecs = [r.get('num_vec', 0) for r in records]
    lines_cnt = [r.get('lines', 0) for r in records]

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)

    ax1.plot(timestamps, steers, color='tab:blue', linewidth=1.5, label='Unghi Direcție (grade)')
    ax1.axhline(0, color='gray', linestyle='--', alpha=0.7)
    ax1.set_ylabel('Steering (°)')
    ax1.set_title('Evoluția Unghiului de Direcție & Detectarea Liniilor Pixy2', fontsize=12, fontweight='bold')
    ax1.grid(True)
    ax1.legend()

    ax2.plot(timestamps, lines_cnt, color='tab:green', linewidth=1.5, label='Linii Ghidaj Detectate (0..2)')
    ax2.plot(timestamps, num_vecs, color='tab:orange', linewidth=1.2, linestyle=':', label='Total Vectori Pixy')
    ax2.set_xlabel('Timp (secunde)')
    ax2.set_ylabel('Număr Vectori')
    ax2.grid(True)
    ax2.legend()

    plt.tight_layout()
    plt.show()

    # Reprezentare 2D a ultimului cadru din fișier
    plot_pixy_frame_2d(records[-1])

def live_stream(ip):
    """Conectare live la API-ul ESP32 http://<ip>/api/telemetry și plotare în timp real."""
    url = f"http://{ip}/api/telemetry"
    print(f"Pornire stream live de la ESP32: {url}")
    plt.ion()
    fig, ax = plt.subplots(figsize=(8, 5))

    last_id = 0
    while True:
        try:
            req_url = f"{url}?since={last_id}"
            with urllib.request.urlopen(req_url, timeout=2) as resp:
                res = json.loads(resp.read().decode('utf-8'))
                latest = res.get('latest')
                if latest:
                    last_id = latest.get('id', last_id)
                    ax.clear()
                    ax.set_xlim(0, PIXY_WIDTH)
                    ax.set_ylim(PIXY_HEIGHT, 0)
                    ax.set_title(f"LIVE Pixy2 Frame - Stare: {latest.get('which', 'NONE')}", fontsize=12, fontweight='bold')
                    ax.set_xlabel("X (0 .. 78 px)")
                    ax.set_ylabel("Y (0 .. 51 px)")
                    ax.grid(True, linestyle='--', alpha=0.5)
                    ax.axvline(39, color='gray', linestyle=':')

                    # Draw left
                    lx0, ly0 = latest.get('lx0', 0), latest.get('ly0', 0)
                    lx1, ly1 = latest.get('lx1', 0), latest.get('ly1', 0)
                    if lx0 or ly0 or lx1 or ly1:
                        ax.plot([lx0, lx1], [ly0, ly1], color='tab:blue', linewidth=3, marker='o', label=f'Left ({lx0},{ly0})->({lx1},{ly1})')

                    # Draw right
                    rx0, ry0 = latest.get('rx0', 0), latest.get('ry0', 0)
                    rx1, ry1 = latest.get('rx1', 0), latest.get('ry1', 0)
                    if rx0 or ry0 or rx1 or ry1:
                        ax.plot([rx0, rx1], [ly0, ry1], color='tab:green', linewidth=3, marker='o', label=f'Right ({rx0},{ry0})->({rx1},{ry1})')

                    ax.legend(loc='upper right')
                    plt.draw()
                    plt.pause(0.1)
        except KeyboardInterrupt:
            print("Stream oprit de utilizator.")
            break
        except Exception as e:
            time.sleep(0.5)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Pixy2 Telemetry Python Plotter")
    parser.add_argument('--file', type=str, help="Calea către fișierul JSON descărcat")
    parser.add_argument('--live', action='store_true', help="Activare conectare live Wi-Fi la ESP32")
    parser.add_argument('--ip', type=str, default="192.168.4.1", help="Adresa IP a serverului ESP32")

    args = parser.parse_args()
    if args.file:
        plot_json_file(args.file)
    elif args.live:
        live_stream(args.ip)
    else:
        parser.print_help()
