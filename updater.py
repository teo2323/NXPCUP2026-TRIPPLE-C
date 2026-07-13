import socket
import sys
import time
import argparse

def main():
    parser = argparse.ArgumentParser(description="PC Updater for FRDM-MCXN947 via ESP32 Wi-Fi Bridge")
    parser.add_argument("--ip", type=str, default="192.168.0.124", help="ESP32 IP address (printed on ESP32 Serial console)")
    parser.add_argument("--port", type=int, default=8080, help="ESP32 TCP port (default: 8080)")
    parser.add_argument("--bin", type=str, help="Path to compiled bin/elf file to flash (optional verification)")
    args = parser.parse_args()

    print(f"[*] Connecting to ESP32 Wi-Fi bridge at {args.ip}:{args.port}...")
    
    # 1. Establish Connection
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(5.0)
        s.connect((args.ip, args.port))
        print("[+] Connection established successfully.")
    except Exception as e:
        print(f"[-] Error connecting to ESP32: {e}")
        print("[-] Please ensure that the ESP32 is powered on, connected to the same Wi-Fi network, and that the IP is correct.")
        sys.exit(1)

    # 2. Send Trigger
    trigger_cmd = "OTA_UPDATE_START"
    print(f"[*] Sending OTA bootloader entry trigger: '{trigger_cmd}'...")
    try:
        s.sendall(trigger_cmd.encode('utf-8'))
    except Exception as e:
        print(f"[-] Error sending trigger command: {e}")
        s.close()
        sys.exit(1)

    # 3. Await Readiness (Wait for ACK_BOOTLOADER)
    print("[*] Awaiting acknowledgment from FRDM board (timeout 5s)...")
    try:
        ack_data = s.recv(1024)
        ack_str = ack_data.decode('utf-8', errors='ignore')
        if "ACK_BOOTLOADER" in ack_str:
            print("[+] Acknowledgment received: 'ACK_BOOTLOADER'.")
            print("[+] FRDM board has successfully written the magic word to SRAM and reset.")
        else:
            print(f"[-] Received unexpected response: {repr(ack_data)}")
            print("[*] Proceeding anyway with a 2-second delay to allow bootloader reset...")
            time.sleep(2.0)
    except socket.timeout:
        print("[-] Timeout waiting for ACK_BOOTLOADER.")
        print("[*] Proceeding with a 2-second delay in case ACK was missed...")
        time.sleep(2.0)
    except Exception as e:
        print(f"[-] Error receiving acknowledgment: {e}")
        s.close()
        sys.exit(1)

    # 4. Protocol Switch (Establish communication with the NXP ROM Bootloader)
    print("[*] Switching protocol to NXP Bootloader (KBOOT/mboot) framing...")
    print("[*] Sending NXP Bootloader Ping packet (0x5A 0xA6)...")
    
    ping_packet = bytes([0x5A, 0xA6])
    try:
        s.sendall(ping_packet)
    except Exception as e:
        print(f"[-] Error sending Ping packet: {e}")
        s.close()
        sys.exit(1)

    # Read Ping Response
    print("[*] Awaiting Ping response from NXP ROM Bootloader...")
    try:
        response = s.recv(1024)
        if len(response) >= 2 and response[0] == 0x5A and response[1] == 0xA7:
            print("[+] Valid Ping Response received from ROM Bootloader!")
            print(f"    Raw bytes: {list(response)}")
            
            # Parse response if it contains the full payload (typically 10 bytes)
            if len(response) >= 6:
                bugfix = response[2]
                minor = response[3]
                major = response[4]
                name = chr(response[5]) if 32 <= response[5] <= 126 else f"0x{response[5]:02X}"
                print(f"    Bootloader Protocol Version: {major}.{minor}.{bugfix}")
                print(f"    Protocol Name: '{name}'")
            if len(response) >= 8:
                options = (response[7] << 8) | response[6]
                print(f"    Protocol Options: 0x{options:04X}")
        else:
            print(f"[-] Invalid or no response received from ROM Bootloader: {list(response)}")
            print("[-] This indicates the MCU did not enter ROM ISP mode or the ESP32 bridge failed to pass the packet.")
            s.close()
            sys.exit(1)
    except Exception as e:
        print(f"[-] Error receiving Ping response: {e}")
        s.close()
        sys.exit(1)

    s.close()
    print("\n[+] Verification complete! Communication with the ROM Bootloader is verified.")
    print("==========================================================================")
    print("  HOW TO COMPLETE THE FIRMWARE UPDATE:")
    print("  To write the new binary to flash, you can use NXP's blhost utility:")
    print("  1. Map a virtual COM port (e.g. COM5) to the ESP32 IP/Port using a tool like:")
    print("     - hw-vsp (HW Virtual Serial Port)")
    print("     - com0com / RFC2217 redirectors")
    print("  2. Run blhost via the virtual COM port to flash your application:")
    print("     blhost -p COM5 -- flash-erase-all")
    print("     blhost -p COM5 -- write-memory 0x00000000 <path_to_binary>.bin")
    print("==========================================================================")

if __name__ == "__main__":
    main()
