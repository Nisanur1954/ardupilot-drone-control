import math
import queue
import threading
import tkinter as tk

from tkinter import ttk, messagebox
from drone_controller import DroneController


class DroneApp:

    def __init__(self, root):

        self.root = root
        self.root.title("ArduPilot - IHA Kontrol Paneli")
        self.root.geometry("750x760")
        self.root.minsize(650, 650)

        # Mevcut calisan controller
        self.controller = DroneController(
            connection_string="127.0.0.1:14550"
        )

        # Arayuz durumlari
        self.connecting = False
        self.action_busy = False
        self.emergency_busy = False
        self.closing = False

        self.ui_events = queue.Queue()
        self._mission_seen_running = False

        # Telemetri
        self.connection_var = tk.StringVar(value="Baglanti: Yok")
        self.mode_var = tk.StringVar(value="Mod: UNKNOWN")
        self.arm_var = tk.StringVar(value="ARM: Bilinmiyor")
        self.alt_var = tk.StringVar(value="Irtifa: --")
        self.pos_var = tk.StringVar(value="Konum (NED): --")
        self.landed_var = tk.StringVar(value="Inis durumu: --")

        # Gorev
        self.mission_var = tk.StringVar(value="Gorev: Beklemede")
        self.info_var = tk.StringVar(
            value="Once MAVLink baglantisini kurun."
        )

        # Parametreler
        self.address_var = tk.StringVar(value="127.0.0.1:14550")
        self.altitude_var = tk.StringVar(value="10")
        self.side_var = tk.StringVar(value="10")
        self.corner_var = tk.StringVar(value="K1")
        self.wait_var = tk.StringVar(value="2")

        self.build_ui()

        self.root.protocol(
            "WM_DELETE_WINDOW",
            self.on_close
        )

        self.root.after(250, self.refresh)

    # ==================================================
    # 1. ARAYUZ
    # ==================================================

    def build_ui(self):

        frame = ttk.Frame(self.root, padding=16)
        frame.pack(fill="both", expand=True)

        frame.columnconfigure(0, weight=1)

        ttk.Label(
            frame,
            text="IHA KONTROL PANELI",
            font=("Arial", 18, "bold")
        ).grid(
            row=0,
            column=0,
            sticky="w",
            pady=(0, 15)
        )

        # --------------------------------------------------
        # BAGLANTI
        # --------------------------------------------------

        connection = ttk.LabelFrame(
            frame,
            text="MAVLink Baglantisi",
            padding=10
        )

        connection.grid(
            row=1,
            column=0,
            sticky="ew",
            pady=(0, 10)
        )

        connection.columnconfigure(1, weight=1)

        ttk.Label(
            connection,
            text="Adres:"
        ).grid(row=0, column=0, padx=(0, 8))

        self.address_entry = ttk.Entry(
            connection,
            textvariable=self.address_var
        )

        self.address_entry.grid(
            row=0,
            column=1,
            sticky="ew"
        )

        self.connect_btn = ttk.Button(
            connection,
            text="BAGLAN",
            command=self.connect
        )

        self.connect_btn.grid(
            row=0,
            column=2,
            padx=(10, 0)
        )

        ttk.Label(
            connection,
            textvariable=self.connection_var
        ).grid(
            row=1,
            column=0,
            columnspan=3,
            sticky="w",
            pady=(8, 0)
        )

        # --------------------------------------------------
        # TELEMETRI
        # --------------------------------------------------

        telemetry = ttk.LabelFrame(
            frame,
            text="Canli Telemetri",
            padding=10
        )

        telemetry.grid(
            row=2,
            column=0,
            sticky="ew",
            pady=(0, 10)
        )

        telemetry_variables = (
            self.mode_var,
            self.arm_var,
            self.alt_var,
            self.pos_var,
            self.landed_var
        )

        for i, variable in enumerate(telemetry_variables):

            ttk.Label(
                telemetry,
                textvariable=variable
            ).grid(
                row=i,
                column=0,
                sticky="w",
                pady=3
            )

        # --------------------------------------------------
        # MANUEL UCUS
        # --------------------------------------------------

        manual = ttk.LabelFrame(
            frame,
            text="Manuel Ucus",
            padding=10
        )

        manual.grid(
            row=3,
            column=0,
            sticky="ew",
            pady=(0, 10)
        )

        manual.columnconfigure(1, weight=1)

        ttk.Label(
            manual,
            text="Kalkis irtifasi (m)"
        ).grid(
            row=0,
            column=0,
            sticky="w"
        )

        ttk.Entry(
            manual,
            textvariable=self.altitude_var
        ).grid(
            row=0,
            column=1,
            sticky="ew",
            padx=10
        )

        # Bagimsiz ARM / DISARM kontrolleri
        arm_row = ttk.Frame(manual)
        arm_row.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(10, 0))
        arm_row.columnconfigure(0, weight=1)
        arm_row.columnconfigure(1, weight=1)

        self.arm_btn = ttk.Button(arm_row, text="ARM", command=self.manual_arm)
        self.arm_btn.grid(row=0, column=0, sticky="ew", padx=(0, 5))

        self.disarm_btn = ttk.Button(arm_row, text="DISARM", command=self.manual_disarm)
        self.disarm_btn.grid(row=0, column=1, sticky="ew", padx=(5, 0))

        self.takeoff_btn = ttk.Button(
            manual,
            text="TAKEOFF",
            command=self.manual_takeoff
        )

        self.takeoff_btn.grid(
            row=2,
            column=0,
            columnspan=2,
            sticky="ew",
            pady=(10, 0)
        )

        # --------------------------------------------------
        # KARE GOREVI
        # --------------------------------------------------

        mission = ttk.LabelFrame(
            frame,
            text="Kare Ucus Gorevi",
            padding=10
        )

        mission.grid(
            row=4,
            column=0,
            sticky="ew",
            pady=(0, 10)
        )

        mission.columnconfigure(1, weight=1)

        fields = (
            ("Kare kenari (m)", self.side_var),
            ("Kose bekleme (sn)", self.wait_var)
        )

        for i, (label, variable) in enumerate(fields):

            ttk.Label(
                mission,
                text=label
            ).grid(
                row=i,
                column=0,
                sticky="w",
                pady=4
            )

            ttk.Entry(
                mission,
                textvariable=variable
            ).grid(
                row=i,
                column=1,
                sticky="ew",
                padx=10,
                pady=4
            )

        ttk.Label(
            mission,
            text="Baslangic kosesi"
        ).grid(
            row=2,
            column=0,
            sticky="w",
            pady=4
        )

        self.corner_box = ttk.Combobox(
            mission,
            textvariable=self.corner_var,
            values=("K1", "K2", "K3", "K4"),
            state="readonly"
        )

        self.corner_box.grid(
            row=2,
            column=1,
            sticky="w",
            padx=10,
            pady=4
        )

        self.start_btn = ttk.Button(
            mission,
            text="KARE GOREVINI BASLAT",
            command=self.start_mission
        )

        self.start_btn.grid(
            row=3,
            column=0,
            columnspan=2,
            sticky="ew",
            pady=(10, 4)
        )

        ttk.Label(
            mission,
            textvariable=self.mission_var
        ).grid(
            row=4,
            column=0,
            columnspan=2,
            sticky="w",
            pady=(6, 0)
        )

        # --------------------------------------------------
        # ACIL DURUM
        # --------------------------------------------------

        emergency = ttk.LabelFrame(
            frame,
            text="Acil Durum / Operator Mudahalesi",
            padding=10
        )

        emergency.grid(
            row=5,
            column=0,
            sticky="ew",
            pady=(0, 10)
        )

        for col in range(3):
            emergency.columnconfigure(col, weight=1)

        self.land_btn = ttk.Button(
            emergency,
            text="LAND",
            command=self.emergency_land
        )

        self.land_btn.grid(
            row=0,
            column=0,
            sticky="ew",
            padx=3,
            pady=3
        )

        self.rtl_btn = ttk.Button(
            emergency,
            text="RTL",
            command=self.emergency_rtl
        )

        self.rtl_btn.grid(
            row=0,
            column=1,
            sticky="ew",
            padx=3,
            pady=3
        )

        self.cancel_btn = ttk.Button(
            emergency,
            text="GOREVI IPTAL",
            command=self.cancel_mission
        )

        self.cancel_btn.grid(
            row=0,
            column=2,
            sticky="ew",
            padx=3,
            pady=3
        )

        self.reset_btn = ttk.Button(
            emergency,
            text="YENI UCUSA HAZIRLA",
            command=self.reset_emergency
        )

        self.reset_btn.grid(
            row=1,
            column=0,
            columnspan=3,
            sticky="ew",
            padx=3,
            pady=(10, 0)
        )

        # --------------------------------------------------
        # BILGILENDIRME
        # --------------------------------------------------

        ttk.Label(
            frame,
            textvariable=self.info_var,
            wraplength=680
        ).grid(
            row=6,
            column=0,
            sticky="w",
            pady=(10, 0)
        )

        ttk.Label(
            frame,
            text="Not: Ucus islemlerini once ArduPilot SITL / Gazebo ortaminda test edin.",
            foreground="gray"
        ).grid(
            row=7,
            column=0,
            sticky="w",
            pady=(8, 0)
        )

    # ==================================================
    # 2. ARKA PLAN ISLEMLERI
    # ==================================================

    def run_background(self, operation, callback=None):

        def worker():

            try:
                result = operation()
                error = None

            except Exception as exc:

                result = False
                error = str(exc)

                print(
                    f"[GUI ERROR] {exc}",
                    flush=True
                )

            if not self.closing:

                self.ui_events.put(
                    (callback, result, error)
                )

        threading.Thread(
            target=worker,
            daemon=True
        ).start()

    # ==================================================
    # 3. BAGLANTI
    # ==================================================

    def connect(self):

        c = self.controller

        if self.connecting:
            return

        if c.connected:

            if c.is_connection_alive():

                self.info_var.set(
                    "Baglanti zaten aktif. Tekrar baglanmak gerekmiyor."
                )

            else:

                self.info_var.set(
                    "Controller bagli ancak HEARTBEAT alinamiyor. "
                    "SITL ve MAVLink akisini kontrol edin."
                )

            return

        address = self.address_var.get().strip()

        if not address:

            messagebox.showerror(
                "Hata",
                "MAVLink adresi bos olamaz."
            )

            return

        c.connection_string = address

        self.connecting = True

        self.info_var.set(
            "Baglanti kuruluyor, HEARTBEAT bekleniyor..."
        )

        self.run_background(
            c.connect,
            self.connect_done
        )

    def connect_done(self, result, error):

        self.connecting = False

        if result:

            self.info_var.set(
                "Baglanti kuruldu. Telemetri izleniyor."
            )

        else:

            self.info_var.set(
                f"Baglanti basarisiz: "
                f"{error or 'HEARTBEAT alinamadi'}"
            )

    # ==================================================
    # 4. MANUEL ARM / DISARM
    # ==================================================

    def manual_arm(self):
        c = self.controller
        if (self.action_busy or self.emergency_busy or c.mission_running
                or c.emergency_active.is_set()):
            return
        if not c.is_connection_alive():
            messagebox.showerror("Baglanti", "MAVLink baglantisi aktif degil.")
            return
        if c.armed:
            self.info_var.set("IHA zaten ARM durumunda.")
            return
        if not c.is_landed_state_fresh() or c.landed_state != 1:
            messagebox.showwarning("ARM", "Guncel telemetri IHA'nin yerde oldugunu dogrulamiyor.")
            return

        self.action_busy = True
        self.info_var.set("ARM: GUIDED modu ve ARM durumu dogrulaniyor...")

        def operation():
            if not c.change_mode("GUIDED") or not c.wait_for_mode("GUIDED"):
                return False
            return c.arm() and c.wait_for_arm()

        self.run_background(operation, self.arm_done)

    def arm_done(self, result, error):
        self.action_busy = False
        self.info_var.set(
            "ARM basarili. IHA kalkisa hazir." if result
            else f"ARM basarisiz: {error or 'Terminali ve ArduPilot on-arm kontrollerini inceleyin.'}"
        )

    def manual_disarm(self):
        c = self.controller
        if (self.action_busy or self.emergency_busy or c.mission_running):
            return
        if not c.is_connection_alive():
            messagebox.showerror("Baglanti", "MAVLink baglantisi aktif degil.")
            return
        if not c.armed:
            self.info_var.set("IHA zaten DISARM durumunda.")
            return
        if not c.is_landed_state_fresh() or c.landed_state != 1:
            messagebox.showwarning("Guvenlik", "DISARM engellendi: IHA'nin yerde oldugu dogrulanamiyor.")
            return
        if not messagebox.askyesno("DISARM", "IHA yerde. Motorlari devre disi birakmak istiyor musunuz?"):
            return

        self.action_busy = True
        self.info_var.set("DISARM komutu gonderiliyor...")

        def operation():
            if not c.disarm():
                return False
            import time
            deadline = time.monotonic() + 10
            while time.monotonic() < deadline:
                if not c.is_connection_alive():
                    return False
                if not c.armed:
                    return True
                time.sleep(0.1)
            return False

        self.run_background(operation, self.disarm_done)

    def disarm_done(self, result, error):
        self.action_busy = False
        self.info_var.set(
            "DISARM basarili. Motorlar devre disi." if result
            else f"DISARM dogrulanamadi: {error or 'Telemetriyi ve terminali kontrol edin.'}"
        )

    # ==================================================
    # 4. TAKEOFF
    # ==================================================

    def manual_takeoff(self):

        c = self.controller

        if self.action_busy or self.emergency_busy:
            return

        if c.mission_running:
            return

        try:

            altitude = float(self.altitude_var.get())

            if not math.isfinite(altitude) or altitude <= 0:
                raise ValueError

        except ValueError:

            messagebox.showerror(
                "Hata",
                "Gecerli ve pozitif bir kalkis irtifasi girin."
            )

            return

        if not c.is_connection_alive():

            messagebox.showerror(
                "Baglanti",
                "MAVLink baglantisi aktif degil."
            )

            return

        if c.emergency_active.is_set():

            messagebox.showwarning(
                "Acil Durum",
                "Once YENI UCUSA HAZIRLA butonunu kullanin."
            )

            return

        # ARM butonuyla onceden ARM edilmis IHA da kalkis yapabilir.
        # Inis durumu biliniyorsa havada kalkisi engelle.
        # Bilgi eksikse controller ARM asamasinda
        # kendi guvenlik kontrolunu uygulayacak.
        if c.is_landed_state_fresh():

            if c.landed_state != 1:

                messagebox.showwarning(
                    "Ucus Durumu",
                    "IHA yerde gorunmuyor. "
                    "TAKEOFF baslatilmadi."
                )

                return

        self.action_busy = True

        self.info_var.set(
            "TAKEOFF: GUIDED moduna geciliyor..."
        )

        def operation():

            print(
                "[TAKEOFF] Islem basladi",
                flush=True
            )

            if c.emergency_active.is_set():
                return False

            # 1. GUIDED
            print(
                "[TAKEOFF] GUIDED komutu gonderiliyor",
                flush=True
            )

            if not c.change_mode("GUIDED"):

                print(
                    "[TAKEOFF ERROR] GUIDED komutu basarisiz",
                    flush=True
                )

                return False

            if not c.wait_for_mode("GUIDED"):

                print(
                    "[TAKEOFF ERROR] GUIDED dogrulanamadi",
                    flush=True
                )

                return False

            if c.emergency_active.is_set():
                return False

            # 2. ARM
            print(
                "[TAKEOFF] ARM komutu gonderiliyor",
                flush=True
            )

            if not c.arm():

                print(
                    "[TAKEOFF ERROR] ARM basarisiz",
                    flush=True
                )

                return False

            if not c.wait_for_arm():

                print(
                    "[TAKEOFF ERROR] ARM dogrulanamadi",
                    flush=True
                )

                return False

            if c.emergency_active.is_set():
                return False

            # 3. TAKEOFF
            print(
                f"[TAKEOFF] {altitude} metre kalkis isteniyor",
                flush=True
            )

            result = c.takeoff(altitude)

            print(
                f"[TAKEOFF] Komut sonucu: {result}",
                flush=True
            )

            return result

        self.run_background(
            operation,
            self.takeoff_done
        )

    def takeoff_done(self, result, error):

        self.action_busy = False

        if result:

            self.info_var.set(
                "TAKEOFF komutu gonderildi. "
                "Irtifa telemetriden takip ediliyor."
            )

        else:

            self.info_var.set(
                "TAKEOFF basarisiz. "
                f"{error or 'Hangi asamada durdugunu terminalden kontrol edin.'}"
            )

    # ==================================================
    # 5. KARE GOREVI
    # ==================================================

    def start_mission(self):

        c = self.controller

        if (
            c.mission_running
            or self.action_busy
            or self.emergency_busy
        ):
            return

        try:

            altitude = float(self.altitude_var.get())
            side_length = float(self.side_var.get())
            wait_time = float(self.wait_var.get())

        except ValueError:

            messagebox.showerror(
                "Hatali Parametre",
                "Sayisal alanlara gecerli degerler girin."
            )

            return

        if (
            not all(
                math.isfinite(v)
                for v in (altitude, side_length, wait_time)
            )
            or altitude <= 0
            or side_length <= 0
            or wait_time < 0
        ):

            messagebox.showerror(
                "Hatali Parametre",
                "Irtifa ve kenar pozitif olmali. "
                "Bekleme suresi negatif olamaz."
            )

            return

        if not c.is_connection_alive():

            messagebox.showerror(
                "Baglanti",
                "MAVLink baglantisi aktif degil."
            )

            return

        if c.emergency_active.is_set():

            messagebox.showwarning(
                "Acil Durum",
                "Yeni gorevden once acil durum sifirlanmali."
            )

            return

        # Kare gorevi kendi ARM ve TAKEOFF islemini yapiyor.
        if c.armed:

            messagebox.showwarning(
                "ARM",
                "Kare gorevi kendi kalkisini gerceklestiriyor.\n"
                "IHA zaten ARM durumunda.\n"
                "Once inis ve DISARM tamamlanmali."
            )

            return

        # Calisan main(2).py dosyasindaki cagri
        started = c.start_square_mission(
            altitude,
            side_length,
            self.corner_var.get(),
            wait_time
        )

        if started:

            self._mission_seen_running = True

            self.info_var.set(
                "Kare gorevi baslatildi. "
                "Ucus adimlari terminalde gorulebilir."
            )

        else:

            self.info_var.set(
                "Gorev baslatilamadi. "
                "Terminaldeki hata mesajlarini kontrol edin."
            )

    # ==================================================
    # 6. ACIL DURUM
    # ==================================================

    def emergency_land(self):

        self.emergency_action(
            self.controller.emergency_land,
            "LAND"
        )

    def emergency_rtl(self):

        self.emergency_action(
            self.controller.emergency_rtl,
            "RTL"
        )

    def cancel_mission(self):

        self.emergency_action(
            self.controller.cancel_mission,
            "GOREV IPTALI"
        )

    def emergency_action(self, operation, name):

        c = self.controller

        if self.emergency_busy:
            return

        if not c.connected:

            self.info_var.set(
                "MAVLink baglantisi yok."
            )

            return

        self.emergency_busy = True

        self.info_var.set(
            f"{name} komutu gonderiliyor..."
        )

        self.run_background(
            operation,
            lambda result, error:
                self.emergency_done(name, result, error)
        )

    def emergency_done(self, name, result, error):

        self.emergency_busy = False

        if result:

            self.info_var.set(
                f"{name}: Mod degisikligi dogrulandi. "
                "Inisin tamamlanmasi bekleniyor."
            )

        else:

            self.info_var.set(
                f"{name}: Dogrulanamadi. "
                f"{error or 'Terminali kontrol edin.'}"
            )

    # ==================================================
    # 7. ACIL DURUM SONRASI YENI UCUS
    # ==================================================

    def reset_emergency(self):

        c = self.controller

        if self.action_busy or self.emergency_busy:
            return

        if not c.emergency_active.is_set():

            self.info_var.set(
                "Acil durum kilidi zaten kapali."
            )

            return

        if not c.is_connection_alive():

            self.info_var.set(
                "Baglanti guncel degil. HEARTBEAT bekleniyor."
            )

            return

        if c.mission_running:

            self.info_var.set(
                "Gorev henuz sonlanmadi. Bekleyin."
            )

            return

        if c.armed:

            self.info_var.set(
                "IHA hala ARM durumunda. "
                "Inis ve DISARM tamamlanmali."
            )

            return

        if not c.is_landed_state_fresh():

            self.info_var.set(
                "Inis durumu telemetrisi guncel degil. "
                "Controller guvenlik nedeniyle sifirlamayi reddediyor."
            )

            return

        if c.landed_state != 1:

            self.info_var.set(
                "IHA yerde gorunmuyor. "
                "Acil durum sifirlanamaz."
            )

            return

        # Controller kendi guvenlik kontrollerini yapar.
        result = c.reset_emergency()

        if result:

            self._mission_seen_running = False

            self.info_var.set(
                "Acil durum sifirlandi. "
                "Yeni ucus baslatabilirsiniz."
            )

        else:

            self.info_var.set(
                "Acil durum sifirlanamadi. "
                "Terminali kontrol edin."
            )

    # ==================================================
    # 8. TELEMETRI VE BUTON GUNCELLEME
    # ==================================================

    def refresh(self):

        if self.closing:
            return

        # Thread sonuclarini arayuze aktar
        while True:

            try:

                callback, result, error = (
                    self.ui_events.get_nowait()
                )

            except queue.Empty:
                break

            if callback is not None:
                callback(result, error)

        c = self.controller

        # Calisan main(2).py ile ayni baglanti kontrolu
        alive = c.is_connection_alive()

        # --------------------------------------------------
        # BAGLANTI
        # --------------------------------------------------

        if alive:

            self.connection_var.set(
                "Baglanti: AKTIF"
            )

        elif c.connected:

            self.connection_var.set(
                "Baglanti: HEARTBEAT YOK"
            )

        else:

            self.connection_var.set(
                "Baglanti: Yok"
            )

        # --------------------------------------------------
        # MOD
        # --------------------------------------------------

        self.mode_var.set(
            f"Mod: {c.flight_mode}"
        )

        # --------------------------------------------------
        # ARM
        # --------------------------------------------------

        self.arm_var.set(
            "ARM: EVET" if c.armed else "ARM: HAYIR"
        )

        # --------------------------------------------------
        # IRTIFA
        # --------------------------------------------------

        # Calisan kodundaki mantik aynen korundu.
        if c.is_altitude_fresh():

            self.alt_var.set(
                f"Irtifa: {c.altitude:.2f} m"
            )

        else:

            self.alt_var.set(
                "Irtifa: --"
            )

        # --------------------------------------------------
        # KONUM
        # --------------------------------------------------

        if c.is_position_fresh():

            x, y, z = c.position

            self.pos_var.set(
                f"Konum (NED): "
                f"X={x:.2f}, Y={y:.2f}, Z={z:.2f}"
            )

        else:

            self.pos_var.set(
                "Konum (NED): --"
            )

        # --------------------------------------------------
        # INIS DURUMU
        # --------------------------------------------------

        landed_labels = {
            1: "Yerde",
            2: "Havada",
            3: "Kalkis",
            4: "Inis"
        }

        if c.is_landed_state_fresh():

            self.landed_var.set(
                "Inis durumu: " +
                landed_labels.get(
                    c.landed_state,
                    "Bilinmiyor"
                )
            )

        else:

            self.landed_var.set(
                "Inis durumu: --"
            )

        # --------------------------------------------------
        # GOREV
        # --------------------------------------------------

        if c.mission_running:

            self.mission_var.set(
                "Gorev: CALISIYOR"
            )

        elif c.emergency_active.is_set():

            self.mission_var.set(
                "Gorev: ACIL DURUM / IPTAL"
            )

        elif self._mission_seen_running:

            self.mission_var.set(
                "Gorev: SONA ERDI"
            )

        else:

            self.mission_var.set(
                "Gorev: BEKLEMEDE"
            )

        # --------------------------------------------------
        # BAGLAN BUTONU
        # --------------------------------------------------

        self.connect_btn.configure(
            state="disabled"
            if self.connecting or c.connected
            else "normal"
        )

        self.address_entry.configure(
            state="disabled"
            if self.connecting or c.connected
            else "normal"
        )

        # --------------------------------------------------
        # NORMAL UCUS BUTONLARI
        # --------------------------------------------------

        # Calisan kodundaki kosullari koruyoruz.
        normal_allowed = (
            alive
            and not c.mission_running
            and not c.emergency_active.is_set()
            and not self.action_busy
            and not self.emergency_busy
        )

        self.arm_btn.configure(
            state="normal" if normal_allowed and not c.armed
            and c.is_landed_state_fresh() and c.landed_state == 1
            else "disabled"
        )

        self.disarm_btn.configure(
            state="normal" if alive and c.armed
            and not c.mission_running and not self.action_busy
            and not self.emergency_busy and c.is_landed_state_fresh()
            and c.landed_state == 1
            else "disabled"
        )

        self.takeoff_btn.configure(
            state="normal"
            if normal_allowed
            else "disabled"
        )

        self.start_btn.configure(
            state="normal"
            if normal_allowed and not c.armed
            else "disabled"
        )

        # --------------------------------------------------
        # ACIL DURUM BUTONLARI
        # --------------------------------------------------

        emergency_enabled = (
            "normal"
            if c.connected and not self.emergency_busy
            else "disabled"
        )

        self.land_btn.configure(
            state=emergency_enabled
        )

        self.rtl_btn.configure(
            state=emergency_enabled
        )

        self.cancel_btn.configure(
            state=emergency_enabled
            if c.mission_running
            else "disabled"
        )

        # --------------------------------------------------
        # YENI UCUSA HAZIRLA
        # --------------------------------------------------

        self.reset_btn.configure(
            state="normal"
            if (
                c.connected
                and c.emergency_active.is_set()
                and not c.mission_running
                and not c.armed
                and not self.action_busy
                and not self.emergency_busy
            )
            else "disabled"
        )

        # 250 ms sonra tekrar guncelle
        self.root.after(
            250,
            self.refresh
        )

    # ==================================================
    # 9. PENCERE KAPATMA
    # ==================================================

    def on_close(self):

        if (
            self.controller.mission_running
            or self.controller.armed
        ):

            confirm = messagebox.askyesno(
                "Uyari",
                "IHA ARM durumunda veya gorev devam ediyor.\n"
                "Pencereyi kapatmak LAND komutu gondermez.\n"
                "Yine de kapatilsin mi?"
            )

            if not confirm:
                return

        self.closing = True

        self.controller.stop_telemetry()

        self.root.destroy()


# ==================================================
# PROGRAM BASLANGICI
# ==================================================

if __name__ == "__main__":

    root = tk.Tk()

    app = DroneApp(root)

    root.mainloop()