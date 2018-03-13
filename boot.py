import config

def boot_wifi(ssid, password):
    import machine
    import network
    import time
    import gc

    print(">> boot.py: START")

    wlan = network.WLAN(mode=network.WLAN.STA)

    if not wlan.isconnected():
        wlan.connect(ssid, auth=(network.WLAN.WPA2, password))

    while not wlan.isconnected():
        time.sleep(1)

    print(">> boot.py: IP: {:,}".format(wlan.ifconfig()[0]))

    rtc = machine.RTC()

    if not rtc.synced():
        rtc.ntp_sync("pool.ntp.org", 3600)

        for _ in range(5, 0, -1):
            if not rtc.synced():
                time.sleep(1)

    print(">> boot.py: Time: %04d/%02d/%02d %02d:%02d:%02d UTC" % time.localtime()[:6])

    gc.collect()

    print(">> boot.py: END - free = {}".format(gc.mem_free()))

    gc.collect()

boot_wifi(config.WIFI_SSID, config.WIFI_PASSWORD)
