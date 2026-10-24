from pymavlink import mavutil
import time
import math
import threading

class DroneController:

    def __init__(self, connection_string="127.0.0.1:14550"):

        self.connection_string = connection_string
        self.master = None
        self.connected = False
        self.altitude = None
        self.flight_mode = "UNKNOWN"
        self.armed = False
        self.position = None
        self.last_heartbeat = None
        self.landed_state = None
        self.last_landed_state_time = None
        self.stop_event = threading.Event()
        self.mission_running = False
        self.last_position_time = None
        self.last_altitude_time = None
        self.telemetry_running = False
        self.telemetry_thread = None

        self.command_lock = threading.Lock()
        self.emergency_active = threading.Event()


    def connect(self):

        print("[CONNECT] Baglanti kuruluyor...")

        try:
            self.master = mavutil.mavlink_connection(
                self.connection_string
            )

            heartbeat = self.master.wait_heartbeat(timeout=10)

            if heartbeat is None:
                raise TimeoutError("HEARTBEAT alinamadi")#raise, Python'da bir hata oluşturmak için kullanılır.
            
            ## HEARTBEAT mesajini gonderen IHA'nin kimligini kaydet
            self.master.target_system = heartbeat.get_srcSystem()
            self.master.target_component = heartbeat.get_srcComponent()
            self.last_heartbeat = time.monotonic()  # EKLEDIGIMIZ SATIR

            self.connected = True
            print("[CONNECT] Baglanti basarili ")
            self.start_telemetry()
            self.request_landed_state()

            print("[CONNECT] Baglanti basarili")
            print(f"[CONNECT] System ID: {self.master.target_system}")
            print(f"[CONNECT] Component ID: {self.master.target_component}")

            return True

        except Exception as hata:

            self.connected = False
            self.master = None

            print(f"[ERROR] Baglanti hatasi: {hata}")

            return False


    def send_command(self, send_function, emergency=False):

        with self.command_lock:

            if not self.connected or self.master is None:
                print("[ERROR] Baglanti yok")
                return False

            if self.emergency_active.is_set() and not emergency:
                print("[COMMAND] Acil durum aktif, normal komut engellendi")
                return False

            if not emergency and not self.is_connection_alive():
                print("[ERROR] MAVLink baglantisi guncel degil")
                return False

            try:
                send_function()
                return True

            except Exception as hata:
                print(f"[ERROR] Komut gonderilemedi: {hata}")
                return False



    def update_telemetry(self):

        if not self.connected:
            return

        msg = self.master.recv_match(blocking=False)#MAVLinl mesajlarını programı bekletmeden okur

        if msg is None: #Mesaj gelmemişse fonksiyondan çıkarız.
            return

        if msg.get_srcSystem() != self.master.target_system:#iha id si kontrol edilir 
            return

        msg_type = msg.get_type()
        if msg_type in (
            "LOCAL_POSITION_NED",
            "GLOBAL_POSITION_INT",
            "EXTENDED_SYS_STATE"
        ):
            print(
                f"[MAVLINK DEBUG] Gelen mesaj: {msg_type}",
                flush=True
            )
        if msg_type == "LOCAL_POSITION_NED":
            print(
                f"[DEBUG POSITION] "
                f"X={msg.x:.2f} "
                f"Y={msg.y:.2f} "
                f"Z={msg.z:.2f}"
            )

        elif msg_type == "GLOBAL_POSITION_INT":
            print(
                f"[DEBUG ALTITUDE] "
                f"relative_alt={msg.relative_alt / 1000:.2f} m"
            )
        if msg_type == "STATUSTEXT":
            print(f"[ARDUPILOT] {msg.text}", flush=True)
            return

        if msg_type == "HEARTBEAT":

            if msg.get_srcComponent() != self.master.target_component:
                return

            self.last_heartbeat = time.monotonic()#son mesajın alındığı zaman

            new_mode = mavutil.mode_string_v10(msg)

            if new_mode != self.flight_mode:
                print(f"[MODE] {self.flight_mode} -> {new_mode}", flush=True)

            self.flight_mode = new_mode

            self.armed = bool(
                msg.base_mode &
                mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED#ihanın arm durumu
        )

        elif msg_type == "GLOBAL_POSITION_INT":

            self.altitude = msg.relative_alt / 1000 #irtifa
            self.last_altitude_time = time.monotonic()


        elif msg_type == "LOCAL_POSITION_NED":

            self.position = (msg.x, msg.y, msg.z) #yerel koordinatlar
            self.last_position_time = time.monotonic()

        elif msg_type == "EXTENDED_SYS_STATE":
            self.landed_state = msg.landed_state   
            self.last_landed_state_time=time.monotonic() 


    def change_mode(self, mode, emergency=False):

        if not self.connected:
            print("[ERROR] Baglanti yok")
            return False

        mode_map = self.master.mode_mapping()

        if mode_map is None or mode not in mode_map:
            print(f"[ERROR] Gecersiz mod: {mode}")
            return False

        result = self.send_command(
            lambda: self.master.mav.set_mode_send(
                self.master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_map[mode]
            ),
            emergency=emergency
        )

        if result:
            print(f"[MODE] {mode} komutu gonderildi")

        return result

    def arm(self):

        if not self.connected:
            print("[ERROR] Baglanti yok")
            return False

        if self.armed:
            print("[ARM] IHA zaten ARM durumunda")
            return True

        result = self.send_command(
            lambda: self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                1, 0, 0, 0, 0, 0, 0
            )
        )

        if result:
            print("[ARM] ARM komutu gonderildi")

        return result

    
    def disarm(self):

        if not self.connected:
            print("[ERROR] Baglanti yok")
            return False

        if not self.is_landed_state_fresh():
            print("[ERROR] Guncel inis durumu bilgisi yok")
            return False

        if self.landed_state != mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND:
            print("[SAFETY] DISARM engellendi: IHA yerde degil")
            return False

        result = self.send_command(
            lambda: self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                0, 0, 0, 0, 0, 0, 0
            )
        )

        if result:
            print("[DISARM] DISARM komutu gonderildi")

        return result

    def takeoff(self, altitude):

        if not self.connected:
            print("[ERROR] Baglanti yok")
            return False

        try:
            altitude = float(altitude)
        except (TypeError, ValueError):
            print("[ERROR] Gecersiz irtifa")
            return False

        if not math.isfinite(altitude) or altitude <= 0:
            print("[ERROR] Irtifa pozitif ve sonlu olmali")
            return False

        if self.flight_mode != "GUIDED":
            print("[ERROR] TAKEOFF icin GUIDED modu gerekli")
            return False

        if not self.armed:
            print("[ERROR] TAKEOFF icin ARM gerekli")
            return False

        result = self.send_command(
            lambda: self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0,
                0, 0, 0, 0, 0, 0,
                altitude
            )
        )

        if result:
            print(f"[TAKEOFF] {altitude} m kalkis komutu gonderildi")

        return result


    def land(self, emergency=False):

        if not self.connected:
            print("[ERROR] Baglanti yok")
            return False

        print("[LAND] Inis komutu gonderiliyor")

        return self.change_mode("LAND", emergency=emergency)



    def rtl(self, emergency=False):
        if not self.connected:
            print("[ERROR] Baglanti yok")
            return False

        print("[RTL] Home konumuna donus isteniyor")

        return self.change_mode("RTL", emergency=emergency)



    def calculate_square(self, side_length):
        if self.position is None:
            print("[ERROR] Konum bilgisi yok")
            return None

        if side_length <= 0:
            print("[ERROR] Kenar uzunlugu pozitif olmali")
            return None

        home_x, home_y, home_z = self.position

        d = side_length / 2

        corners = {
            "K1": (home_x + d, home_y - d),
            "K2": (home_x + d, home_y + d),
            "K3": (home_x - d, home_y + d),
            "K4": (home_x - d, home_y - d)
        }

        print("[MISSION] Kare koordinatlari hesaplandi")

        return (home_x, home_y, home_z), corners



    def create_square_route(self, corners, start_corner):

        corner_order = ["K1", "K2", "K3", "K4"]

        if start_corner not in corner_order:
            print("[ERROR] Gecersiz baslangic kosesi")
            return None

        start_index = corner_order.index(start_corner)

        route = (
            corner_order[start_index:] +
            corner_order[:start_index]
        )
        # corner_order[2:]  # ["K3", "K4"]
        # corner_order[:2]  # ["K1", "K2"]    

        waypoints = []

        for corner in route:
            x, y = corners[corner]
            waypoints.append((corner, x, y))

        print(f"[MISSION] Baslangic kosesi: {start_corner}")
        print(f"[MISSION] Rota: HOME -> {' -> '.join(route)} -> HOME")

        return waypoints


    def go_to_position(self, x, y, z):

        if not self.connected:
            print("[ERROR] Baglanti yok")
            return False

        if self.stop_event.is_set():
            print("[MOVE] Gorev iptal edildi, hareket komutu gonderilmedi")
            return False

        if self.flight_mode != "GUIDED":
            print("[ERROR] Konum komutu icin GUIDED modu gerekli")
            return False

        if not all(math.isfinite(v) for v in (x, y, z)):
            print("[ERROR] Gecersiz hedef koordinatlari")
            return False

        result = self.send_command(
            lambda: self.master.mav.set_position_target_local_ned_send(
                0,
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                0b110111111000,
                x, y, z,
                0, 0, 0,
                0, 0, 0,
                0, 0
            )
        )

        if result:
            print(f"[MOVE] Hedef: X={x:.1f}, Y={y:.1f}, Z={z:.1f}")

        return result
    
    def wait_until_reached(self, x, y, z, tolerance=1.0, timeout=60):

        start_time = time.monotonic()

        while time.monotonic() - start_time < timeout:

            if self.stop_event.is_set():
                print("[MISSION] Hedef bekleme iptal edildi")
                return False

            if not self.is_connection_alive():
                print("[ERROR] MAVLink baglantisi guncel degil")
                return False

            if not self.is_position_fresh():
                print("[ERROR] Guncel konum bilgisi yok")
                return False

            current_x, current_y, current_z = self.position

            distance = math.sqrt(
                (x - current_x)**2 +
                (y - current_y)**2 +
                (z - current_z)**2
            )

            if distance <= tolerance:
                print("[MISSION] Hedefe ulasildi")
                return True

            time.sleep(0.1)

        print("[ERROR] Hedefe ulasma zaman asimi")
        return False



    def square_mission(self, altitude, side_length, start_corner, wait_time):
        if self.stop_event.is_set():
            print("[MISSION] Gorev iptal edildi")
            return False

        # 1. Parametre kontrolü
        try:
            altitude = float(altitude)
            side_length = float(side_length)
            wait_time = float(wait_time)

        except (TypeError, ValueError):
            print("[ERROR] Gorev parametreleri sayisal olmali")
            return False

        if (
            not all(math.isfinite(v) for v in (altitude, side_length, wait_time))
            or altitude <= 0
            or side_length <= 0
            or wait_time < 0
        ):
            print("[ERROR] Gecersiz gorev parametreleri")
            return False

        if not self.connected:
            print("[ERROR] Baglanti yok")
            return False

        # 2. Telemetri kontrolu
        start_time = time.monotonic()

        while time.monotonic() - start_time < 5:

            if self.stop_event.is_set():
                return False


            if (
                self.is_connection_alive()
                and self.is_position_fresh()
                and self.is_altitude_fresh()
            ):
                break

            time.sleep(0.1)

        else:
            print("[ERROR] Guncel telemetri alinamadi")
            return False

        # 3. Kare koordinatlarini hesapla
        home, corners = self.calculate_square(side_length)

        if home is None:
            return False

        home_x, home_y, home_z = home

        # 4. Rotayi olustur
        route = self.create_square_route(corners, start_corner)

        if route is None:
            return False

        # 5. Hedef irtifayi hesapla
        target_z = home_z - altitude

        print("[MISSION] Kare gorevi hazirlaniyor")

        # 6. GUIDED moduna gec
        if not self.change_mode("GUIDED"):
            return False
        if not self.wait_for_mode("GUIDED"):
            return False
        # 7. ARM
        if not self.arm():
            return False

        if not self.wait_for_arm():
            return False

        # 8. TAKEOFF
        if not self.takeoff(altitude):
            return False

        # 9. Kalkisin tamamlanmasini bekle
        print("[MISSION] Hedef irtifa bekleniyor")

        if not self.wait_until_reached(
            home_x, home_y, target_z
        ):
            return False

        # 10. Kare koseslerini dolas
        for corner, x, y in route:

            if self.stop_event.is_set():
                    print("[MISSION] Gorev durduruldu")
                    return False

            print(f"[MISSION] {corner} hedefine gidiliyor")

            if not self.go_to_position(x, y, target_z):
                return False

            if not self.wait_until_reached(x, y, target_z):
                return False

            print(f"[MISSION] {corner} noktasina ulasildi")

            if self.stop_event.wait(wait_time):
                return False

        # 11. Gorev merkezine don
        if self.stop_event.is_set():
            print("[MISSION] Gorev iptal edildi")
            return False

        if not self.return_to_center(home, target_z):
            return False

        # 12. Inis
        if self.stop_event.is_set():
            return False

        print("[MISSION] Inis baslatiliyor")

        if not self.land():
            return False

        if not self.wait_for_mode("LAND"):
            return False

        if not self.wait_for_landing():
            return False

        print("[MISSION] Kare gorevi basariyla tamamlandi")

        return True



    def return_to_center(self, home, target_z):

        home_x, home_y, _ = home

        print("[RETURN] Gorev merkezine donuluyor")

        if not self.go_to_position(home_x, home_y, target_z):
            return False

        if not self.wait_until_reached(
            home_x, home_y, target_z
        ):
            print("[ERROR] Gorev merkezine ulasilamadi")
            return False

        print("[RETURN] Gorev merkezine ulasildi")
        return True

    def cancel_mission(self):

        print("[MISSION] Gorev iptal ediliyor")

        return self.emergency_land()



    def emergency_land(self):

        print("[EMERGENCY] Acil LAND istendi")

        with self.command_lock: #komut kilidini alıyoruz

            self.emergency_active.set() #acil durum bayrağı etkinleştirildi
            self.stop_event.set() #göreve iptal istendi

        result = self.land(emergency=True) #land 
        if not result:
            print("[ERROR] Acil LAND komutu gonderilemedi")
            return False

        print("[EMERGENCY] LAND komutu gonderildi")

        if not self.wait_for_mode("LAND", emergency=True):
            print("[ERROR] LAND moduna gecis dogrulanamadi")
            return False

        print("[EMERGENCY] IHA LAND moduna gecti")

        return True
        

    def emergency_rtl(self):

        print("[EMERGENCY] Acil RTL istendi")

        with self.command_lock:

            self.emergency_active.set()
            self.stop_event.set()

        result = self.rtl(emergency=True)

        if not result:
            print("[ERROR] Acil RTL komutu gonderilemedi")
            return False

        print("[EMERGENCY] RTL komutu gonderildi")

        if not self.wait_for_mode("RTL", emergency=True):
            print("[ERROR] RTL moduna gecis dogrulanamadi")
            return False

        print("[EMERGENCY] IHA RTL moduna gecti")

        return True


    def reset_emergency(self):

        with self.command_lock:

            if not self.is_connection_alive():
                print("[ERROR] Baglanti guncel degil")
                return False

            if self.mission_running:
                print("[ERROR] Gorev thread'i hala calisiyor")
                return False
            if not self.is_landed_state_fresh():
                print("[ERROR] Guncel inis durumu bilgisi yok")
                return False

            if self.landed_state != mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND:
                print("[ERROR] IHA yerde degil veya inis durumu bilinmiyor")
                return False

            if self.armed:
                print("[ERROR] IHA hala ARM durumunda")
                return False

            self.emergency_active.clear()
            self.stop_event.clear()

            print("[SAFETY] Acil durum sifirlandi")

            return True


    def start_square_mission(self, altitude, side_length, start_corner, wait_time):

        with self.command_lock:

            if self.mission_running:
                print("[ERROR] Zaten bir gorev calisiyor")
                return False

            if self.emergency_active.is_set():
                print("[ERROR] Acil durum aktif, gorev baslatilamaz")
                return False

            if not self.is_connection_alive():
                print("[ERROR] Baglanti guncel degil")
                return False

            self.mission_running = True
            self.stop_event.clear()

        def mission_worker():

            try:
                result = self.square_mission(
                    altitude,
                    side_length,
                    start_corner,
                    wait_time
                )

                if (
                    not result
                    and not self.emergency_active.is_set()
                    and self.armed
                    and self.flight_mode not in ("LAND", "RTL")
                ):
                    print("[SAFETY] Gorev basarisiz, acil LAND baslatiliyor")
                    self.emergency_land()

            except Exception as hata:

                print(f"[ERROR] Gorev hatasi: {hata}")

                if (
                    not self.emergency_active.is_set()
                    and self.armed
                    and self.flight_mode not in ("LAND", "RTL")
                ):
                    self.emergency_land()

            finally:
                with self.command_lock:
                    self.mission_running = False

        self.mission_thread = threading.Thread(
            target=mission_worker,
            daemon=True
        )

        try:
            self.mission_thread.start()

        except Exception as hata:
            with self.command_lock:
                self.mission_running = False

            print(f"[ERROR] Gorev thread'i baslatilamadi: {hata}")
            return False

        print("[MISSION] Gorev thread'i baslatildi")
        return True




    def wait_for_mode(self, target_mode, timeout=10, emergency=False):

        start_time = time.monotonic()

        while time.monotonic() - start_time < timeout:

            if self.stop_event.is_set() and not emergency:
                print("[MISSION] Mod bekleme iptal edildi")
                return False

            if not self.is_connection_alive():
                print("[ERROR] Mod dogrulanamadi: baglanti guncel degil")
                return False

            if self.flight_mode == target_mode:
                print(f"[MODE] {target_mode} modu dogrulandi")
                return True

            time.sleep(0.1)

        print(f"[ERROR] {target_mode} moduna gecis zaman asimi")
        return False




    def wait_for_arm(self, timeout=10):

        start_time = time.monotonic()

        while time.monotonic() - start_time < timeout:

            if self.stop_event.is_set():
                print("[MISSION] ARM bekleme iptal edildi")
                return False


            if self.armed:
                print("[ARM] IHA ARM durumuna gecti")
                return True

            time.sleep(0.1)

        print("[ERROR] ARM islemi zaman asimina ugradi")
        return False



    def wait_for_landing(self, timeout=120):

        start_time = time.monotonic()

        while time.monotonic() - start_time < timeout:

            if self.emergency_active.is_set():
                print("[MISSION] Acil durum aktif, normal inis beklemesi sonlandirildi")
                return False

            if not self.is_connection_alive():
                print("[ERROR] Inis dogrulanamadi: baglanti guncel degil")
                return False

            if not self.is_landed_state_fresh():
                print("[ERROR] Guncel inis durumu bilgisi yok")
                return False

            if (
                self.landed_state == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND
                and not self.armed
            ):
                print("[LAND] IHA yere indi ve DISARM oldu")
                return True

            time.sleep(0.2)

        print("[ERROR] Inis dogrulama zaman asimi")
        return False


    def is_connection_alive(self, timeout=5):

        if not self.connected:
            return False

        if self.last_heartbeat is None:
            return False

        elapsed = time.monotonic() - self.last_heartbeat

        if elapsed > timeout:
            return False

        return True


    def is_position_fresh(self, timeout=2):

        if self.position is None:
            return False

        if self.last_position_time is None:
            return False

        elapsed = time.monotonic() - self.last_position_time

        if elapsed > timeout:
            print("[WARNING] Konum bilgisi guncel degil")
            return False

        return True

    
    def is_altitude_fresh(self, timeout=2):

        if self.altitude is None:
            return False

        if self.last_altitude_time is None:
            return False

        elapsed = time.monotonic() - self.last_altitude_time

        if elapsed > timeout:
            print("[WARNING] Irtifa bilgisi guncel degil")
            return False

        return True

    def is_landed_state_fresh(self, timeout=3):

        if self.last_landed_state_time is None:
            return False

        return time.monotonic() - self.last_landed_state_time <= timeout




    def telemetry_loop(self):

        print("[TELEMETRY] Telemetri thread'i baslatildi")

        while self.telemetry_running:

            try:
                self.update_telemetry()

            except Exception as hata:
                print(f"[ERROR] Telemetri hatasi: {hata}")
                self.telemetry_running = False
                break

            time.sleep(0.001)

        print("[TELEMETRY] Telemetri thread'i durduruldu")


    def start_telemetry(self):

        if not self.connected:
            print("[ERROR] Baglanti yok")
            return False

        if self.telemetry_running:
            print("[TELEMETRY] Zaten calisiyor")
            return True

        self.telemetry_running = True

        self.telemetry_thread = threading.Thread(
            target=self.telemetry_loop,
            daemon=True
        )

        self.telemetry_thread.start()

        return True


    def stop_telemetry(self):

        self.telemetry_running = False

        print("[TELEMETRY] Durdurma istegi gonderildi")



    def request_landed_state(self):

        if not self.connected or self.master is None:
            return False

        print(
            "[TELEMETRY] EXTENDED_SYS_STATE mesaji isteniyor...",
            flush=True
        )

        return self.send_command(
            lambda: self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE,
                500000,
                0, 0, 0, 0, 0
            )
        )