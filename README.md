# pytrack-poller

The pycom [pytrac](https://pycom.io/product/pytrack/) module comes with a very basic driver for the GPS chip.

The pytrac uses the [L76-L GNSS](http://www.quectel.com/product/l76l.htm) chip for GPS data. This poller leverages the manufacturer's detailed [documentation](http://www.quectel.com/Qdownload/L76.zip) to enable extended features for polling.

The poller will initialize the chip and track state and output a JSON log record every 10 seconds with the latest state.

If a SD card is found the poller will also log the json records to files named /sd/gps-log-YYYYMMDDHH.json (one per hour) with one JSON object per line (newline terminated).

## Extended modes

The following extended modes are enabled by this poller:

* Set report intervals to include ZDA
* Set number of decimal points to max
* TXT messages
* Velocity reporting
* Return Link messages
* EASY function
* GPS+GLONASS+Galileo+GALILEO_FULL
* SBAS DGPS Mode
* Searching for SBAS satellites

## Example log entry:

```json
{
    "battery": 4.822186,
    "battery_status": "OK",
    "count": 61426,
    "mem_free": 2542576,
    "state": {
        "$GLGSV-1": "$GLGSV,1,1,..............................................*6A",
        "$GLGSV-2": "$GLGSV,2,2,.............*56",
        "$GNGLL-A": "$GNGLL,9999.9999,N,99999.9999,W,100057.000,A,A*5C",
        "$GNGSA": "$GNGSA,A,2.............................*1F",
        "$GNRMC": "$GNRMC,100057.000,A,9999.9999,N,99999.9999,W,9.45,137.90,130318,,,A*67",
        "$GNZDA": "$GNZDA,100059.000,13,03,2018,,*4F",
        "$GPGGA": "$GPGGA,100059.000,,,,,0,0,,,M,,M,,*45",
        "$GPGLL-V": "$GPGLL,,,,,100059.000,V,N*77",
        "$GPGSA": "$GPGSA,A,1,,,,,,,,,,,,,,,*1E",
        "$GPGSV-1": "$GPGSV,3,1,....................................................*7E",
        "$GPGSV-2": "$GPGSV,3,2,..............................................*7A",
        "$GPGSV-3": "$GPGSV,3,3,.............*49",
        "$GPRMC": "$GPRMC,100059.000,V,...............,130318,,,N*41",
        "$GPVTG": "$GPVTG,139.09,T,,M,9.75,N,18.07,K,N*05",
        "$PMTK001": "$PMTK001,313,3*31",
        "$PMTK514": "$PMTK514,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,1,0,0*33",
        "$PQPREC": "$PQPREC,W,6,6,3,1*7C",
        "$PQRLM": "$PQRLM,W,1,1*29",
        "$PQTXT": "$PQTXT,W,1,1*22",
        "$PQVEL": "$PQVEL,-3.844606,3.321756,-0.130539*6D",
        "count_large_buf": 8331,
        "last_large_buf": 61401,
        "last_read_bytes": 39,
        "max_buf_len": 254,
        "max_pkt_len": 72
    },
    "time": "2018-03-13 10:01:00 UTC"
}
```

## Configuration

If you want to connect via telnet or ftp to the module you should copy config-example.py to config.py and set your SSID and Wifi password.


