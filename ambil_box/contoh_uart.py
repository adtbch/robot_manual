"""
contoh_uart.py
Contoh program Python untuk komunikasi dengan sistem Arm Box melalui UART
"""

import serial
import time

class ArmBoxController:
    def __init__(self, port='COM3', baudrate=115200):
        """
        Inisialisasi koneksi serial
        
        Args:
            port: Port serial (contoh: 'COM3' untuk Windows, '/dev/ttyUSB0' untuk Linux)
            baudrate: Baud rate komunikasi (default: 115200)
        """
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
            time.sleep(2)  # Tunggu Arduino ready
            print(f"✓ Terhubung ke {port} dengan baud rate {baudrate}")
        except Exception as e:
            print(f"✗ Error: Tidak dapat terhubung ke {port}")
            print(f"  Detail: {e}")
            self.ser = None
    
    def send_command(self, command):
        """
        Kirim perintah ke Arduino
        
        Args:
            command: String perintah (tanpa newline)
            
        Returns:
            Response dari Arduino atau None jika gagal
        """
        if not self.ser:
            print("✗ Error: Koneksi serial tidak tersedia")
            return None
        
        try:
            # Kirim perintah
            cmd = command + '\n'
            self.ser.write(cmd.encode())
            print(f"→ Kirim: {command}")
            
            # Baca response
            time.sleep(0.1)
            if self.ser.in_waiting > 0:
                response = self.ser.readline().decode().strip()
                print(f"← Terima: {response}")
                return response
            else:
                print("← Tidak ada response")
                return None
                
        except Exception as e:
            print(f"✗ Error mengirim perintah: {e}")
            return None
    
    def homing(self):
        """Lakukan homing semua motor"""
        print("\n=== HOMING ===")
        return self.send_command("HOMING")
    
    def putar_kanan(self):
        """Putar motor ke kanan"""
        return self.send_command("PUTAR_KANAN")
    
    def putar_kiri(self):
        """Putar motor ke kiri"""
        return self.send_command("PUTAR_KIRI")
    
    def putar_stop(self):
        """Stop motor putar"""
        return self.send_command("PUTAR_STOP")
    
    def putar_ke_posisi(self, posisi):
        """Putar ke posisi tertentu"""
        return self.send_command(f"PUTAR_POS_{posisi}")
    
    def naik(self):
        """Motor naik"""
        return self.send_command("NAIK_TURUN_NAIK")
    
    def turun(self):
        """Motor turun"""
        return self.send_command("NAIK_TURUN_TURUN")
    
    def naik_turun_stop(self):
        """Stop motor naik turun"""
        return self.send_command("NAIK_TURUN_STOP")
    
    def naik_turun_ke_posisi(self, posisi):
        """Naik/turun ke posisi tertentu"""
        return self.send_command(f"NAIK_TURUN_POS_{posisi}")
    
    def maju(self):
        """Motor maju"""
        return self.send_command("MAJU_MUNDUR_MAJU")
    
    def mundur(self):
        """Motor mundur"""
        return self.send_command("MAJU_MUNDUR_MUNDUR")
    
    def maju_mundur_stop(self):
        """Stop motor maju mundur"""
        return self.send_command("MAJU_MUNDUR_STOP")
    
    def maju_mundur_ke_posisi(self, posisi):
        """Maju/mundur ke posisi tertentu"""
        return self.send_command(f"MAJU_MUNDUR_POS_{posisi}")
    
    def buka_gripper(self):
        """Buka gripper"""
        return self.send_command("SERVO_BUKA")
    
    def tutup_gripper(self):
        """Tutup gripper"""
        return self.send_command("SERVO_TUTUP")
    
    def servo_home(self):
        """Servo ke posisi home"""
        return self.send_command("SERVO_HOME")
    
    def set_servo_angle(self, angle):
        """Set sudut servo"""
        return self.send_command(f"SERVO_ANGLE_{angle}")
    
    def relay1_on(self):
        """Nyalakan relay 1"""
        return self.send_command("RELAY_1_ON")
    
    def relay1_off(self):
        """Matikan relay 1"""
        return self.send_command("RELAY_1_OFF")
    
    def relay2_on(self):
        """Nyalakan relay 2"""
        return self.send_command("RELAY_2_ON")
    
    def relay2_off(self):
        """Matikan relay 2"""
        return self.send_command("RELAY_2_OFF")
    
    def get_status(self):
        """Request status sistem"""
        return self.send_command("STATUS")
    
    def emergency_stop(self):
        """Emergency stop semua motor"""
        print("\n⚠️ EMERGENCY STOP ⚠️")
        return self.send_command("STOP")
    
    def close(self):
        """Tutup koneksi serial"""
        if self.ser:
            self.ser.close()
            print("✓ Koneksi ditutup")


def demo_sequence():
    """Demo sequence lengkap mengambil box"""
    
    # Inisialisasi controller
    arm = ArmBoxController(port='COM3')  # Sesuaikan port Anda
    
    if not arm.ser:
        print("Gagal terhubung ke Arduino!")
        return
    
    try:
        print("\n" + "="*50)
        print("  DEMO SEQUENCE - AMBIL BOX")
        print("="*50)
        
        # 1. Homing
        print("\n[1] Homing semua motor...")
        arm.homing()
        time.sleep(3)
        
        # 2. Buka gripper
        print("\n[2] Membuka gripper...")
        arm.buka_gripper()
        time.sleep(2)
        
        # 3. Gerak ke posisi ambil
        print("\n[3] Bergerak ke posisi ambil...")
        arm.putar_ke_posisi(500)
        time.sleep(2)
        arm.maju_mundur_ke_posisi(800)
        time.sleep(2)
        arm.naik_turun_ke_posisi(300)
        time.sleep(2)
        
        # 4. Tutup gripper (ambil box)
        print("\n[4] Mengambil box...")
        arm.tutup_gripper()
        time.sleep(2)
        
        # 5. Angkat box
        print("\n[5] Mengangkat box...")
        arm.naik_turun_ke_posisi(0)
        time.sleep(2)
        
        # 6. Putar ke posisi drop
        print("\n[6] Memutar ke posisi drop...")
        arm.putar_ke_posisi(1000)
        time.sleep(2)
        
        # 7. Turunkan box
        print("\n[7] Menurunkan box...")
        arm.naik_turun_ke_posisi(300)
        time.sleep(2)
        
        # 8. Buka gripper (lepas box)
        print("\n[8] Melepas box...")
        arm.buka_gripper()
        time.sleep(2)
        
        # 9. Kembali ke home
        print("\n[9] Kembali ke home...")
        arm.naik_turun_ke_posisi(0)
        time.sleep(2)
        arm.maju_mundur_ke_posisi(0)
        time.sleep(2)
        arm.putar_ke_posisi(0)
        time.sleep(2)
        arm.servo_home()
        time.sleep(1)
        
        print("\n" + "="*50)
        print("  SEQUENCE SELESAI!")
        print("="*50)
        
        # Cek status akhir
        print("\n[Status Akhir]")
        arm.get_status()
        
    except KeyboardInterrupt:
        print("\n\n⚠️ Program dihentikan oleh user")
        arm.emergency_stop()
    
    except Exception as e:
        print(f"\n✗ Error: {e}")
        arm.emergency_stop()
    
    finally:
        arm.close()


def test_manual():
    """Test manual untuk setiap komponen"""
    
    arm = ArmBoxController(port='COM3')  # Sesuaikan port Anda
    
    if not arm.ser:
        return
    
    print("\n" + "="*50)
    print("  TEST MANUAL")
    print("="*50)
    
    while True:
        print("\n=== MENU ===")
        print("1. Test Motor Putar")
        print("2. Test Motor Naik Turun")
        print("3. Test Motor Maju Mundur")
        print("4. Test Servo")
        print("5. Test Relay")
        print("6. Homing")
        print("7. Get Status")
        print("8. Emergency Stop")
        print("0. Keluar")
        
        pilihan = input("\nPilih menu: ")
        
        if pilihan == '1':
            print("\nTest Motor Putar:")
            print("a. Kanan  b. Kiri  c. Stop  d. Posisi")
            sub = input("Pilih: ")
            if sub == 'a':
                arm.putar_kanan()
            elif sub == 'b':
                arm.putar_kiri()
            elif sub == 'c':
                arm.putar_stop()
            elif sub == 'd':
                pos = input("Masukkan posisi: ")
                arm.putar_ke_posisi(pos)
        
        elif pilihan == '2':
            print("\nTest Motor Naik Turun:")
            print("a. Naik  b. Turun  c. Stop  d. Posisi")
            sub = input("Pilih: ")
            if sub == 'a':
                arm.naik()
            elif sub == 'b':
                arm.turun()
            elif sub == 'c':
                arm.naik_turun_stop()
            elif sub == 'd':
                pos = input("Masukkan posisi: ")
                arm.naik_turun_ke_posisi(pos)
        
        elif pilihan == '3':
            print("\nTest Motor Maju Mundur:")
            print("a. Maju  b. Mundur  c. Stop  d. Posisi")
            sub = input("Pilih: ")
            if sub == 'a':
                arm.maju()
            elif sub == 'b':
                arm.mundur()
            elif sub == 'c':
                arm.maju_mundur_stop()
            elif sub == 'd':
                pos = input("Masukkan posisi: ")
                arm.maju_mundur_ke_posisi(pos)
        
        elif pilihan == '4':
            print("\nTest Servo:")
            print("a. Buka  b. Tutup  c. Home  d. Angle")
            sub = input("Pilih: ")
            if sub == 'a':
                arm.buka_gripper()
            elif sub == 'b':
                arm.tutup_gripper()
            elif sub == 'c':
                arm.servo_home()
            elif sub == 'd':
                angle = input("Masukkan sudut (0-180): ")
                arm.set_servo_angle(angle)
        
        elif pilihan == '5':
            print("\nTest Relay:")
            print("a. Relay1 ON  b. Relay1 OFF")
            print("c. Relay2 ON  d. Relay2 OFF")
            sub = input("Pilih: ")
            if sub == 'a':
                arm.relay1_on()
            elif sub == 'b':
                arm.relay1_off()
            elif sub == 'c':
                arm.relay2_on()
            elif sub == 'd':
                arm.relay2_off()
        
        elif pilihan == '6':
            arm.homing()
            time.sleep(3)
        
        elif pilihan == '7':
            arm.get_status()
        
        elif pilihan == '8':
            arm.emergency_stop()
        
        elif pilihan == '0':
            break
    
    arm.close()


if __name__ == "__main__":
    print("="*50)
    print("  ARM BOX CONTROLLER - Python Interface")
    print("="*50)
    print("\n1. Demo Sequence")
    print("2. Test Manual")
    print("0. Keluar")
    
    pilihan = input("\nPilih mode: ")
    
    if pilihan == '1':
        demo_sequence()
    elif pilihan == '2':
        test_manual()
    else:
        print("Keluar...")
