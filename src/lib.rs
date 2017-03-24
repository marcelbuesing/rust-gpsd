//#[link(name = "my_build_dependency", kind = "static")]
extern crate libc;
//use libc::size_t;

use std::default::Default;


pub const WATCH_ENABLE:libc::c_int   = 0x000001;   /* enable streaming */
pub const WATCH_DISABLE:libc::c_int  = 0x000002;   /* disable watching */
pub const WATCH_JSON:libc::c_int     = 0x000010;   /* JSON output */
pub const WATCH_NMEA:libc::c_int     = 0x000020;   /* output in NMEA */
pub const WATCH_RARE:libc::c_int     = 0x000040;   /* output of packets in hex */
pub const WATCH_RAW:libc::c_int      = 0x000080;   /* output of raw packets */
pub const WATCH_SCALED:libc::c_int   = 0x000100;   /* scale output to floats */
pub const WATCH_TIMING:libc::c_int   = 0x000200;   /* timing information */
pub const WATCH_DEVICE:libc::c_int   = 0x000800;   /* watch specific device */
pub const WATCH_SPLIT24:libc::c_int  = 0x001000;   /* split AIS Type 24s */
pub const WATCH_PPS:libc::c_int      = 0x002000;   /* enable PPS JSON */
pub const WATCH_NEWSTYLE:libc::c_int = 0x010000;   /* force JSON streaming */


//GPS_PATH_MAX:uint8 = 128;

// MAXCHANNELS = 72 /* must be > 12 GPS + 12 GLONASS + 2 WAAS */
// MAXUSERDEVS 4   /* max devices per user */

pub type GPSMaskT = libc::uint64_t;
pub type TimestampT = libc::c_double;

pub type GPSPath = [libc::c_char; 128];

#[repr(C)]
#[derive(Default)]
pub struct DopT {
    /* Dilution of precision factors */
    xdop: libc::c_double,
    ydop: libc::c_double,
    pdop: libc::c_double,
    hdop: libc::c_double,
    vdop: libc::c_double,
    tdop: libc::c_double,
    gdop: libc::c_double,
}

#[repr(C)]
pub struct DevconfigT {
    path: [libc::c_char; 128],
    flags: libc::c_int,
//    #define SEEN_GPS    0x01
//    #define SEEN_RTCM2  0x02
//    #define SEEN_RTCM3  0x04
//    #define SEEN_AIS    0x08
    driver: [libc::c_char; 64],
    subtype: [libc::c_char; 64],
    activated: libc::c_double,

    /* RS232 link parameters */
    baudrate: libc::c_uint,
    stopbits: libc::c_uint,

    /* 'N', 'O', or 'E' */
    parity: libc::c_char,

    /* refresh cycle time in seconds */
    cycle: libc::c_double,
    mincycle: libc::c_double,

    /* is driver in native mode or not? */
    driver_mode: libc::c_int,
}

impl Default for DevconfigT {
    fn default() -> DevconfigT {
        DevconfigT {
            path: [0; 128],
            flags: Default::default(),
            driver: [0; 64],
            subtype: [0; 64],
            activated: Default::default(),
            baudrate: Default::default(),
            stopbits: Default::default(),
            parity: Default::default(),
            cycle: Default::default(),
            mincycle: Default::default(),
            driver_mode: Default::default(),
        }
    }
}

#[repr(C)]
#[derive(Default)]
pub struct GPSFixT {
    time: TimestampT,   /* Time of update */

    /* Mode of fix */
    mode: libc::c_int,
//    #define MODE_NOT_SEEN   0   /* mode update not seen yet */
//    #define MODE_NO_FIX 1   /* none */
//    #define MODE_2D     2   /* good for latitude/longitude */
//    #define MODE_3D     3   /* good for altitude/climb too */

    /* Expected time uncertainty */
    ept: libc::c_double,

    /* Latitude in degrees (valid if mode >= 2) */
    latitude: libc::c_double,

    /* Latitude position uncertainty, meters */
    epy: libc::c_double,

    /* Longitude in degrees (valid if mode >= 2) */
    longitude: libc::c_double,

    /* Longitude position uncertainty, meters */
    epx: libc::c_double,

    /* Altitude in meters (valid if mode == 3) */
    altitude: libc::c_double,

    /* Vertical position uncertainty, meters */
    epv: libc::c_double,

    /* Course made good (relative to true north) */
    track: libc::c_double,

    /* Track uncertainty, degrees */
    epd: libc::c_double,

    /* Speed over ground, meters/sec */
    speed: libc::c_double,

    /* Speed uncertainty, meters/sec */
    eps: libc::c_double,

    /* Vertical speed, meters/sec */
    climb: libc::c_double,

    /* Vertical speed uncertainty */
    epc: libc::c_double,
}
#[repr(C)]
pub struct PolicyT {
    watcher: bool, /* is watcher mode on? */
    json:    bool, /* requesting JSON? */
    nmea:    bool, /* requesting dumping as NMEA? */
    raw:     i16, /* requesting raw data? */
    scaled:  bool, /* requesting report scaling? */
    timing:  bool, /* requesting timing info */
    split24: bool, /* requesting split AIS Type 24s */
    pps:     bool, /* requesting PPS in NMEA/raw modes */
    loglevel: u8,  /* requested log level of messages */
    devpath: [libc::c_char; 128], /* specific device to watch */
    remote:  [libc::c_char; 128], /* ...if this was passthrough */
}

impl Default for PolicyT {
    fn default() -> PolicyT {
        PolicyT {
            watcher: Default::default(),
            json:    Default::default(),
            nmea:    Default::default(),
            raw:     Default::default(),
            scaled:  Default::default(),
            timing:  Default::default(),
            split24: Default::default(),
            pps:     Default::default(),
            loglevel: Default::default(),
            devpath: [0; 128],
            remote:  [0; 128],
        }
    }
}

#[repr(C)]
pub struct GPSDataT {

    /* has field been set since this was last cleared? */
    set: GPSMaskT,

   /* NZ if GPS is on line, 0 if not.
    *
    * Note: gpsd clears this time when sentences
    * fail to show up within the GPS's normal
    * send cycle time. If the host-to-GPS
    * link is lossy enough to drop entire
    * sentences, this field will be
    * prone to false zero values.
    */
    online: TimestampT,

//    #ifndef USE_QT
//      gps_fd: socket_t/* socket or file descriptor to GPS */
//    #else
//      gps_fd: *void,
//    #endif

    /* accumulated PVT data */
    fix: GPSFixT,

    /* this should move to the per-driver structure */
    separation: libc::c_double,/* Geoidal separation, MSL - WGS84 (Meters) */

    /* GPS status -- always valid */
    status: libc::c_int ,/* Do we have a fix? */

    /* precision of fix -- valid if satellites_used > 0 */
    satellites_used: libc::c_int ,/* Number of satellites used in solution */
    lused: [libc::c_int; 72],/* PRNs of satellites used in solution */
    dop: DopT,

    /* redundant with the estimate elements in the fix structure */
    epe: libc::c_double,  /* spherical position error, 95% confidence (meters)  */

    /* satellite status -- valid when satellites_visible > 0 */
    skyview_time: TimestampT,/* skyview timestamp */

    /* # of satellites in view */
    satellites_visible: libc::c_int,

    /* PRNs of satellite */
    prn: [libc::c_int; 72],

    /* elevation of satellite */
    elevation: [libc::c_int; 72],

    /* azimuth */
    azimuth: [libc::c_int; 72],

    /* signal-to-noise ratio (dB) */
    ss: [libc::c_double; 72],

    dev: DevconfigT,/* device that shipped last update */

    policy: PolicyT,/* our listening policy */

    /* should be moved to privdata someday */
    tag: [libc::c_char; 73],/* tag of last sentence processed */

    /* pack things never reported together to reduce structure size */
   // #define UNION_SET(RTCM2_SET|RTCM3_SET|SUBFRAME_SET|AIS_SET|ATTITUDE_SET|GST_SET|VERSION_SET|DEVICELIST_SET|LOGMESSAGE_SET|ERROR_SET)
//    union NeverReported {
//      /* unusual forms of sensor data that might come up the pipe */
//      rtcm2: rtcm2_trtcm2,
//      rtcm3: rtcm3_trtcm3,
//      subframe: subframe_t,
//      ais:ais_t,
//      attitude: attitude_t,
//      raw: rawdata_t,
//      gst: gst_t,
//      /* "artificial" structures for various protocol responses */
//      version: version_t,
//      error: [char; 256],
//    }

    /* Private data - client code must not set this */
    //void *privdata,
}

impl Default for GPSDataT {
    fn default() -> GPSDataT {
        GPSDataT {
            set: Default::default(),
            online: Default::default(),
            fix: Default::default(),
            separation: Default::default(),
            status: Default::default(),
            satellites_used: Default::default(),
            lused: [0;72],
            dop: Default::default(),
            epe: Default::default(),
            skyview_time: Default::default(),
            satellites_visible: Default::default(),
            prn: [0;72],
            elevation: [0;72],
            azimuth: [0;72],
            ss:[0.0;72],
            dev: Default::default(),
            policy: Default::default(),
            tag: [Default::default(); 73]
        }
    }
}

#[repr(C)]
#[derive(Default)]
pub struct Devices {
    time: TimestampT,
    ndevices: libc::c_int,
    list: [DevconfigT; 4],
}

#[link(name = "gps")]
extern "C" {

    pub fn gps_sock_open(host: *const libc::c_char, port: *const libc::c_char, gpsDataT: *mut GPSDataT) -> libc::c_int;
    pub fn gps_sock_close(gpsdata: *mut GPSDataT) -> libc::c_int;
    pub fn gps_sock_send(gpsdata: *mut GPSDataT, buf: *const libc::c_char) -> libc::c_int;
    pub fn gps_sock_read(gpsdata: *mut GPSDataT) -> libc::c_int;
    pub fn gps_sock_waiting(gpsdata: *const GPSDataT, timeout: i64) -> bool;
    pub fn gps_sock_stream(gpsdata: *mut GPSDataT, flags: libc::c_int, d: *mut libc::c_void)  -> libc::c_int;
    pub fn gps_sock_data(gpsdata: *const GPSDataT) -> libc::c_char;
    //
//    extern int gps_sock_mainloop(struct gps_data_t *, int,
//                        void (*)(struct gps_data_t *));
    fn gps_sock_mainloop(GPSDataT: *mut GPSDataT,
                         timeout: libc::c_int,
                         hook: extern "C" fn (*mut GPSDataT)) -> libc::c_int;
    pub fn gps_shm_open(gpsdata: *mut GPSDataT) -> libc::c_int;
    pub fn gps_shm_close(gpsdata: *mut GPSDataT);
    pub fn gps_shm_waiting(gpsdata: *const GPSDataT, timeout: libc::c_long)  -> bool;
    pub fn gps_shm_read(gpsdata: *mut GPSDataT) -> libc::c_int;
    //fn gps_shm_mainloop(*mut GPSDataT, timeout: libc::c_long, void (*)(struct GPSDataT *)) -> libc::c_int;

    pub fn gps_dbus_open(gpsdata: *mut GPSDataT)  -> libc::c_int;
    //fn gps_dbus_mainloop(gpsdata: *mut GPSDataT, lib::c_long, void (*)(struct GPSDataT *)) -> libc::c_int;
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        unsafe {
            use GPSDataT;
            use gps_sock_open;
            use gps_sock_mainloop;
            use gps_sock_stream;
            use gps_sock_close;
            use WATCH_JSON;
            use WATCH_ENABLE;
            use std::ffi::CString;
            use std::ptr;

            let mut gps_data: GPSDataT = Default::default();
            let ip = CString::new("127.0.0.1").expect("Invalid IP");
            let port = CString::new("2947").expect("Invalid Port");


            extern "C" fn print_gps(gps_data: *mut GPSDataT) {
                unsafe {
                    println!("Satellites visible {}, online {}", (*gps_data).satellites_visible, (*gps_data).online)
                }
            }

            println!("Running now");
            let openres = gps_sock_open(ip.as_ptr(), port.as_ptr(), &mut gps_data);
            println!("Open Result {}", openres);

            let sockres = gps_sock_stream(&mut gps_data, WATCH_ENABLE | WATCH_JSON, ptr::null_mut());
            println!("Socket Result {}", sockres);

            let mainres = gps_sock_mainloop(&mut gps_data, 20000, print_gps);
            println!("Main Result {}", mainres);

            let closeres = gps_sock_close(&mut gps_data);
            println!("Close Result {}", closeres);
        }
    }
}
