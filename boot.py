import config

def boot_wifi(ssid, password):
    import machine
    import network
    import time
    import gc
    import pycom

    print(">> boot.py: START")

    wlan = network.WLAN(mode=network.WLAN.STA)

    pycom.heartbeat(False)
    pycom.rgbled(0x090000)

    wlan_timeout = time.time() + 15

    if not wlan.isconnected():
        wlan.connect(ssid, auth=(network.WLAN.WPA2, password), timeout=10000)

    colors = [ 0x000009, 0x000900, 0x090000 ]

    while not wlan.isconnected() and time.time() < wlan_timeout:
        pycom.rgbled(colors[time.time() % len(colors)])
        time.sleep(1)

    try:
        print(">> boot.py: IP: {:,}".format(wlan.ifconfig()[0]))
    except:
        pass

    rtc = machine.RTC()

    if not rtc.synced():
        rtc.ntp_sync("pool.ntp.org", 3600)

        for _ in range(5, 0, -1):
            if not rtc.synced():
                time.sleep(1)

    print(">> boot.py: Time: %04d/%02d/%02d %02d:%02d:%02d UTC" % time.localtime()[:6])

    pycom.heartbeat(False)

    gc.collect()

    print(">> boot.py: gc.mem_free = {}".format(gc.mem_free()))

    machine.info()
    gc.collect()

    print(">> boot.py: END")

boot_wifi(config.WIFI_SSID, config.WIFI_PASSWORD)
