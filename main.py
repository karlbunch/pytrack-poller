import machine
import time
import json
import machine
import pycom
import pytrack
import gc
import sys
import os

GPS_I2CADDR = const(0x10)

class GPS_Poller:
    TIME_MODE_RTC = "RTC"
    TIME_MODE_GPS_SEARCH = "GPS_SEARCH"
    TIME_MODE_GPS_INSYNC = "GPS_INSYNC"

    def __init__(self, json_output=False):
        pycom.heartbeat(False)
        pycom.rgbled(0x100000)

        self.gps_log = gps_logger()
        self.state = {}
        self.json_output = json_output
        self.i2c = machine.I2C(0, mode=machine.I2C.MASTER, pins=('P22','P21'))
        self.py = pytrack.Pytrack()
        self.cmd_queue = []
        self.cmd_wait_for = None
        self.cmd_timeout = None
        self.time_mode = self.TIME_MODE_RTC
        self.state["have_fix"] = False

        if time.localtime()[0] < 1981:
            self.time_mode = self.TIME_MODE_GPS_SEARCH

        rtc = machine.RTC()

        try:
            rtc.ntp_sync("pool.ntp.org", 3600)
        except:
            pass

        for _ in range(5):
            if not rtc.synced():
                time.sleep(1)
            else:
                break

        # Setup GPS
        self.queue_cmd("$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,1,0") # Report intervals
        self.queue_cmd("$PMTK414", "$PMTK514") # Query report intervals
        self.queue_cmd("$PQPREC,W,6,6,3,1", "$PQPREC,W,") # Set number of decimal points
        self.queue_cmd("$PQTXT,W,1,1", "$PQTXT,W,") # Enable txt messages
        self.queue_cmd("$PQVEL,W,1,1", "$PQVEL,W,") # Enable velocity reporting
        self.queue_cmd("$PQRLM,W,1,1", "$PQRLM,W,") # Enable Return Link messages
        self.queue_cmd("$PMTK869,1,1") # Enable EASY function
        # self.queue_cmd("$PMTK183") # Query log status
        self.queue_cmd("$PMTK353,1,1,1,1,0") # Enable GPS+GLONASS+Galileo+GALILEO_FULL
        self.queue_cmd("$PMTK301,2") # Enable SBAS DGPS Mode
        self.queue_cmd("$PMTK313,1") # Enable searching for SBAS satellites

    def run(self, log_interval=10):
        self.read_count = 0
        self.next_log_time = 0

        circular_buffer = b""
        max_buf_len = 0
        max_pkt_len = 0
        last_large_buf = 0
        count_large_buf = 0

        while True:
            self.read_count += 1
            rgb_color = (self.read_count % 2) * 0x000008
            if self.state["have_fix"]:
                rgb_color |= 0x000800
            else:
                rgb_color |= 0x080000

            pycom.rgbled(rgb_color)

            buf = self.i2c.readfrom(GPS_I2CADDR, 255).lstrip(b"\n").rstrip(b"\n")

            if len(buf) > max_buf_len:
                max_buf_len = len(buf)
                self.state["max_buf_len"] = max_buf_len

            if len(buf) > 55:
                last_large_buf = self.read_count
                count_large_buf += 1

            # If less than a full buffer we can try sending commands
            if len(buf) < 254:
                self.send_next_cmd()

            if len(buf) == 0:
                continue

            self.state["last_read_bytes"] = len(buf)
            self.state["last_large_buf"] = last_large_buf
            self.state["count_large_buf"] = count_large_buf

            circular_buffer += buf

            while True:
                idx = circular_buffer.find(b"\r")

                if idx < 0:
                    break

                # Extract and parse each packet
                pkt = circular_buffer[:idx].decode("ascii")
                ofs = 2 if circular_buffer[idx:idx+2] == b"\r\n" else 1
                circular_buffer = circular_buffer[idx+ofs:]

                if len(pkt) > max_pkt_len:
                    max_pkt_len = len(pkt)
                    self.state["max_pkt_len"] = max_pkt_len

                self.parse_pkt(pkt)

            if not self.json_output:
                print("\033[H", end="")
                for r in self.state.items():
                    print(r, "\033[K")

                print("\033[K\n\r>> [", circular_buffer, "]\033[K\033[0J", end='', flush=True)
            elif time.time() > self.next_log_time:
                self.next_log_time = time.time() + log_interval
                log_entry = {}
                log_entry["count"] = self.read_count
                gc.collect()
                log_entry["mem_free"] = gc.mem_free()

                try:
                    log_entry["battery"] = self.py.read_battery_voltage()
                    log_entry["battery_status"] = "OK"
                except:
                    log_entry["battery_status"] = "ERROR"

                log_entry["state"] = self.state

                self.gps_log.log(log_entry, stdout=True)

            if len(self.cmd_queue) > 0:
                time.sleep(0.01)
            else:
                time.sleep(0.1)

    def parse_pkt(self, pkt):
        if not "*" in pkt:
            return

        try:
            msg, pkt_chksum = pkt.split('*')
            pkt_chksum = int(pkt_chksum, 16)
        except ValueError:
            self.errlog("split_err", "Split failed: %s" % pkt)
            return

        if len(msg) == 0:
            return

        #print("msg(%d)[%s] pkt_chksum(%s): [%s]" % (len(msg), msg, pkt_chksum, pkt))

        msg_chksum = 0
        ofs = 1 if msg[0] == "$" else 0
        for ch in msg[ofs:]:
            msg_chksum ^= ord(ch)

        if msg_chksum != pkt_chksum:
            self.errlog("chkerr", "Skipping invalid msg_chksum(%x) != pkt_chksum(%x): [%s]" % (msg_chksum, pkt_chksum, pkt))
            return

        self.check_cmd_response(pkt)

        key = pkt[:pkt.find(",")]

        # Parse some keys out to logical keys
        try:
            if len(key) == 6:
                sub_key = key[3:6]

                if sub_key == "RMC":
                    self.parse_rmc(pkt)
                elif sub_key == "ZDA":
                    self.parse_zda(pkt)
                elif sub_key == "GLL":
                    self.parse_gll(pkt)

                if sub_key == "GSV": # Multi-sequence packet
                    key += "-%s" % pkt.split(",")[2]
                elif sub_key == "RMC": # Lat/Long "V" (Void) or "A" (Active)
                    key += "-%s" % pkt.split(",")[2]
                elif sub_key == "GLL": # Lat/Long "V" (Void) or "A" (Active)
                    key += "-%s" % pkt.split(",")[6]
        except:
            pass

        self.state[key] = pkt

    def set_gps_time(self, new_time, source):
        self.errlog("gps_time_" + source, "GPS Time {}: {} RTC: {}".format(source, new_time, time.localtime()[:6]))

        if not None in new_time:
            if new_time[0] <= 2010: # No support for time travel
                return

            # Wait for cmd queue to drain
            if len(self.cmd_queue) > 0:
                return

            self.state["clock_drift"] = abs(time.mktime(new_time[:-1] + (int(new_time[-1]),0,0)) - time.time())

            if self.state["clock_drift"] > 60:
                try:
                    ss = new_time[-1]
                    rtc_tm = new_time[:-1] + (int(ss),int((ss-int(ss))*1000000))
                    machine.RTC().init(rtc_tm)
                    self.errlog("rtc_set", "RTC set to {}".format(rtc_tm))
                except:
                    pass

    def set_fix(self, fix_time, fix_ll, source):
        if not self.state["have_fix"]:
            self.state["fix_start"] = "{} @ {}".format(fix_ll, fix_time)

        self.state["have_fix"] = True
        self.errlog("last_fix", "{} @ {}".format(fix_ll, fix_time))

    def clear_fix(self, source):
        if self.state["have_fix"]:
            self.state["have_fix"] = False
            self.state["fix_end"] = "{} @ {}".format(source, self.read_count)
            self.errlog("clear_fix", source)

    def parse_gps_utc(self, hhmmss):
        try:
            hh = int(hhmmss[0:2])
            mm = int(hhmmss[2:4])
            ss = float(hhmmss[4:])
            return (hh, mm, ss)
        except:
            return (None, None, None)

    def parse_ll_fix(self, ll):
        """ Parse Lat,Dir,Long,Dir """
        try:
            latitude = float(ll[0][0:2]) + float(ll[0][2:]) / 60

            if ll[1] == "S":
                latitude *= -1

            longitude = float(ll[2][0:3]) + float(ll[2][3:]) / 60

            if ll[3] == "W":
                longitude *= -1

            return (latitude, longitude)
        except:
            self.errlog("parse_ll_fix_error", "Error parsing: {}".format(ll))
            return (None, None)

    def parse_rmc(self, pkt):
        """
               0      1          2 3         4 5          6 7    8      9      10 11 12
        Parse: $GNRMC,142323.000,A,3446.4447,N,11145.9536,W,0.00,206.73,280318,  ,  ,D*6D
            0	Message ID $GPRMC
            1	UTC of position fix
            2	Status A=active or V=void
            3	Latitude
            4	Longitude
            5	Speed over the ground in knots
            6	Track angle in degrees (True)
            7	Date
            8	Magnetic variation in degrees
            9	The checksum data, always begins with *
         """
        fields = pkt.split(',')

        if fields[2] != 'A':
            self.clear_fix("rmc")
            return

        try:
            fix_ll = self.parse_ll_fix(fields[3:6])
            ddmmyy = fields[9]
            dd = int(ddmmyy[0:2])
            mm = int(ddmmyy[2:4])
            yy = int(ddmmyy[4:])
            if yy < 100:
                yy += 2000
            fix_time = self.parse_gps_utc(fields[1])
            self.set_gps_time((yy, mm, dd) + fix_time, "rmc")
            self.set_fix(fix_time, fix_ll, "rmc")
        except:
            self.errlog("parse_rmc_fail", "Failed to parse: " + pkt)

        return

    def parse_gll(self, pkt):
        """
               0      1         2 3          4 5          6 7
        Parse: $GNGLL,3324.8933,N,11200.4470,W,161732.000,A,A*57
            0	Message ID $GPGLL
            1	Latitude in dd mm,mmmm format (0-7 decimal places)
            2	Direction of latitude N: North S: South
            3	Longitude in ddd mm,mmmm format (0-7 decimal places)
            4	Direction of longitude E: East W: West
            5	UTC of position in hhmmss.ss format
            6	Fixed text "A" shows that data is valid
            7	The checksum data, always begins with *
        """
        fields = pkt.split(',')

        if fields[6] != 'A':
            self.clear_fix("gll")
            return

        try:
            fix_ll = self.parse_ll_fix(fields[1:4])
            fix_time = self.parse_gps_utc(fields[5])
            self.set_fix(fix_time, fix_ll, "gll")
        except:
            self.errlog("parse_gll_fail", "Failed to parse: " + pkt)

    def parse_zda(self, pkt):
        """
               0      1          2  3  4    5 6 7
        Parse: $GNZDA,142323.000,28,03,2018, ,  *4F
            0	Message ID $GPZDA
            1	UTC
            2	Day, ranging between 01 and 31
            3	Month, ranging between 01 and 12
            4	Year
            5	Local time zone offset from GMT, ranging from 00 through ±13 hours
            6	Local time zone offset from GMT, ranging from 00 through 59 minutes
            7	The checksum data, always begins with *

        Note: $GNZDA,000507.800,06,01,1980,,*45 - GPS Week 0 - Not valid date
        """
        fields = pkt.split(',')

        try:
            tm = self.parse_gps_utc(fields[1])
            dd = int(fields[2])
            mm = int(fields[3])
            yy = int(fields[4])
            self.set_gps_time((yy, mm, dd) + tm, "zda")
        except:
            self.errlog("parse_zda_fail", "Failed to parse: " + pkt)

    def queue_cmd(self, cmd, wait_for=None, timeout=10):
        self.cmd_queue.append((cmd, wait_for, timeout))

    def send_next_cmd(self, force=False):
        if len(self.cmd_queue) == 0:
            return

        if self.cmd_wait_for and force == False:
            return

        cmd, wait_for, timeout = self.cmd_queue[0]

        chksum = 0
        for ch in cmd[1:]:
            chksum ^= ord(ch)

        cmd += "*%02X\r\n" % chksum

        self.i2c.writeto(GPS_I2CADDR, cmd.encode("ascii"))

        if wait_for == None: # By default expect a PMTK001,Cmd,3 (success)
            wait_for = "$PMTK001,%s,3" % cmd[5:8]

        # Retry command after timeout seconds
        self.cmd_timeout = time.time() + timeout

        if len(wait_for) > 0:
            self.cmd_wait_for = wait_for
        else:
            self.cmd_wait_for = None
            self.cmd_queue.pop(0)

        #self.state["last_cmd_sent"] = "@%d: [%s] waitfor [%s] timeout %d" % (self.read_count, cmd, self.cmd_wait_for, timeout)

    def check_cmd_response(self, pkt):
        if not self.cmd_wait_for:
            return

        if pkt.startswith(self.cmd_wait_for):
            self.cmd_wait_for = None
            self.cmd_timeout = None
            self.cmd_queue.pop(0)
            self.send_next_cmd()
            return

        if time.time() > self.cmd_timeout:
            self.errlog("cmd_timeout", "Timeout sending %s" % self.cmd_queue[0][0])
            self.send_next_cmd(force=True)

    def errlog(self, err_type, err_msg):
        self.state["last_" + err_type] = "@%d: %s" % (self.read_count, err_msg)

        err_key = "errcnt_" + err_type

        if err_key in self.state:
            self.state[err_key] += 1
        else:
            self.state[err_key] = 1

class gps_logger:
    def __init__(self):
        self.have_SD = False
        self.log_fh = None
        self.log_tag = None

        try:
            os.listdir("/sd")
            self.have_SD = True
            print(">> SD Card already mounted")
        except:
            try:
                sd = machine.SD()
                os.mount(sd, "/sd") # pylint: disable=E1101
                self.have_SD = True
                print(">> Mounted SD card")
            except:
                pass

        if not self.have_SD:
            print(">> No SD Card")

    def log(self, obj, stdout=False):
        tm = time.localtime()
        obj["time"] = "%04d-%02d-%02d %02d:%02d:%02d UTC" % tm[:6]

        if not self.have_SD:
            obj["NOSDCARD"] = True

        log_entry = json.dumps(obj) + "\n"

        if self.have_SD:
            # YYYYMMDDHH
            log_tag = tm[0] * 1000000 + tm[1] * 10000 + tm[2] * 100 + tm[3]

            # Time for new file?
            if self.log_tag != log_tag or not self.log_fh:
                self.log_tag = log_tag
                if self.log_fh:
                    self.log_fh.close()
                    self.log_fh = None

                log_fn = "/sd/gps-log-{}.json".format(self.log_tag)
                try:
                    self.log_fh = open(log_fn, "a", encoding="ascii")
                    self.log_tag = log_tag
                    print("** Logging to %s" % log_fn)
                except Exception as e:
                    sys.print_exception(e) # pylint: disable=E1101
                    print("** failed to open %s: No SD card?" % log_fn)

            if self.log_fh:
                self.log_fh.write(log_entry)
                self.log_fh.flush()

        if stdout:
            print(log_entry, end='', flush=True)

        return

try:
    poller = GPS_Poller(json_output=True)
    poller.run()
except:
    pass

print("** Stopped **")
try:
    import os
    os.unmount("/sd") # pylint: disable=E1101
except:
    pass

if poller:
    print(poller.__dict__)
