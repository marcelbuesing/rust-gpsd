extern crate libc;

use std::default::Default;
use std::ptr;
use std::fmt;

/// enable streaming
pub const WATCH_ENABLE:libc::c_int   = 0x000001;
/// disable watching
pub const WATCH_DISABLE:libc::c_int  = 0x000002;
/// JSON output
pub const WATCH_JSON:libc::c_int     = 0x000010;
/// output in NMEA
pub const WATCH_NMEA:libc::c_int     = 0x000020;
/// output of packets in hex
pub const WATCH_RARE:libc::c_int     = 0x000040;
/// output of raw packets
pub const WATCH_RAW:libc::c_int      = 0x000080;
/// scale output to floats
pub const WATCH_SCALED:libc::c_int   = 0x000100;
/// timing information
pub const WATCH_TIMING:libc::c_int   = 0x000200;
/// watch specific device
pub const WATCH_DEVICE:libc::c_int   = 0x000800;
/// split AIS Type 24s
pub const WATCH_SPLIT24:libc::c_int  = 0x001000;
/// enable PPS JSON
pub const WATCH_PPS:libc::c_int      = 0x002000;
/// force JSON streaming
pub const WATCH_NEWSTYLE:libc::c_int = 0x010000;

pub const GPS_PATH_MAX: usize = 128;
/// must be > 12 GPS + 12 GLONASS + 2 WAAS
pub const MAXCHANNELS: usize = 72;
// max devices per user
pub const MAXUSERDEVS: usize = 4;

pub type GPSMaskT = libc::uint64_t;
pub type TimestampT = libc::c_double;

/// Longitude in decimal degrees
pub type Longitude = libc::c_double;
/// Latitude in decimal degrees
pub type Latitude = libc::c_double;

pub type SocketT = libc::c_int;

#[repr(C)]
#[derive(Default)]
pub struct DopT {
    /// Dilution of precision factors
    pub xdop: libc::c_double,
    pub ydop: libc::c_double,
    pub pdop: libc::c_double,
    pub hdop: libc::c_double,
    pub vdop: libc::c_double,
    pub tdop: libc::c_double,
    pub gdop: libc::c_double,
}

#[repr(C)]
pub struct DevconfigT {
    pub path: [libc::c_char; GPS_PATH_MAX],
    pub flags: libc::c_int,
//    #define SEEN_GPS    0x01
//    #define SEEN_RTCM2  0x02
//    #define SEEN_RTCM3  0x04
//    #define SEEN_AIS    0x08
    pub driver: [libc::c_char; 64],
    pub subtype: [libc::c_char; 64],
    pub activated: libc::c_double,

    /// RS232 link parameters
    pub baudrate: libc::c_uint,
    pub stopbits: libc::c_uint,

    /// 'N', 'O', or 'E'
    pub parity: libc::c_char,

    /// refresh cycle time in seconds
    pub cycle: libc::c_double,
    pub mincycle: libc::c_double,

    /// is driver in native mode or not?
    pub driver_mode: libc::c_int,
}

impl Default for DevconfigT {
    fn default() -> DevconfigT {
        DevconfigT {
            path: [0; GPS_PATH_MAX],
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
    pub time: TimestampT,   /// Time of update

    /// Mode of fix
    pub mode: libc::c_int,
//    #define MODE_NOT_SEEN   0   /// mode update not seen yet
//    #define MODE_NO_FIX 1   /// none
//    #define MODE_2D     2   /// good for latitude/longitude
//    #define MODE_3D     3   /// good for altitude/climb too

    /// Expected time uncertainty
    pub ept: libc::c_double,

    /// Latitude in degrees (valid if mode >= 2)
    pub latitude: Latitude,

    /// Latitude position uncertainty, meters
    pub epy: libc::c_double,

    /// Longitude in degrees (valid if mode >= 2)
    pub longitude: Longitude,

    /// Longitude position uncertainty, meters
    pub epx: libc::c_double,

    /// Altitude in meters (valid if mode == 3)
    pub altitude: libc::c_double,

    /// Vertical position uncertainty, meters
    pub epv: libc::c_double,

    /// Course made good (relative to true north)
    pub track: libc::c_double,

    /// Track uncertainty, degrees
    pub epd: libc::c_double,

    /// Speed over ground, meters/sec
    pub speed: libc::c_double,

    /// Speed uncertainty, meters/sec
    pub eps: libc::c_double,

    /// Vertical speed, meters/sec
    pub climb: libc::c_double,

    /// Vertical speed uncertainty
    pub epc: libc::c_double,
}

impl fmt::Display for GPSFixT {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f,
               "\nGPSFIX \n \
                Time {}\n \
                Mode {}\n \
                Ept {}\n \
                Lat {}\n \
                Epy {}\n \
                Lon {}\n \
                Epx {}\n \
                Altitude {}\n \
                Epv {}\n \
                Track {}\n \
                Epd {}\n \
                Speed {}"
               ,
               self.time,
               self.mode,
               self.ept,
               self.latitude,
               self.epy,
               self.longitude,
               self.epx,
               self.altitude,
               self.epv,
               self.track,
               self.epd,
               self.speed
         )
    }
}

#[repr(C)]
pub struct PolicyT {
    /// is watcher mode on?
    pub watcher: bool,
    /// requesting JSON?
    pub json:    bool,
    /// requesting dumping as NMEA?
    pub nmea:    bool,
    /// requesting raw data?
    pub raw:     libc::c_int,
    /// requesting report scaling?
    pub scaled:  bool,
    /// requesting timing info
    pub timing:  bool,
    /// requesting split AIS Type 24s
    pub split24: bool,
    /// requesting PPS in NMEA/raw modes
    pub pps:     bool,
    /// requested log level of messages
    pub loglevel: u8,
    /// specific device to watch
    pub devpath: [libc::c_char; GPS_PATH_MAX],
    /// ...if this was passthrough
    pub remote:  [libc::c_char; GPS_PATH_MAX],
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
            devpath: [0; GPS_PATH_MAX],
            remote:  [0; GPS_PATH_MAX],
        }
    }
}

#[derive(Clone, Copy)]
#[repr(C)]
pub struct SatelliteT {
    /// signal-to-noise ratio (dB)
    pub ss: libc::c_double,
    /// PRNs of satellites used in solution
    pub used: bool,
    /// PRNs of satellite
    pub prn: libc::c_short,
    /// elevation of satellite
    pub elevation: libc::c_short,
    /// azimuth
    pub azimuth: libc::c_short,
}

impl Default for SatelliteT {
    fn default() -> SatelliteT {
        SatelliteT {
            ss: Default::default(),
            used: Default::default(),
            prn: Default::default(),
            elevation: Default::default(),
            azimuth: Default::default(),
       }
    }
}

#[repr(C)]
pub struct GPSDataT {

    /// has field been set since this was last cleared?
    pub set: GPSMaskT,

   /// NZ if GPS is on line, 0 if not.
   ///
   /// Note: gpsd clears this time when sentences
   /// fail to show up within the GPS's normal
   /// send cycle time. If the host-to-GPS
   /// link is lossy enough to drop entire
   /// sentences, this field will be
   /// prone to false zero values.

    pub online: TimestampT,

//    #ifndef USE_QT
    pub gps_fd: SocketT, /// socket or file descriptor to GPS
//    #else
//    gps_fd: *mut libc::c_void,
//    #endif

    /// accumulated PVT data
    pub fix: GPSFixT,

    /// this should move to the per-driver structure
    pub separation: libc::c_double,/// Geoidal separation, MSL - WGS84 (Meters)

    /// GPS status -- always valid
    pub status: libc::c_int ,/// Do we have a fix?

    /// precision of fix -- valid if satellites_used > 0
    pub satellites_used: libc::c_int ,/// Number of satellites used in solution

    pub dop: DopT,

    /// redundant with the estimate elements in the fix structure
    pub epe: libc::c_double,  /// spherical position error, 95% confidence (meters)

    /// satellite status -- valid when satellites_visible > 0
    pub skyview_time: TimestampT,/// skyview timestamp

    /// # of satellites in view
    pub satellites_visible: libc::c_int,

    pub skyview: [SatelliteT; MAXCHANNELS],

    pub dev: DevconfigT,/// device that shipped last update

    pub policy: PolicyT,/// our listening policy

    pub devices: Devices,

    /// Private data rtcm3_t largest in union -> sizeof 5672 = 4*1418
    pub union_never_reported: [libc::c_int; 1418],

    /// Private data - client code must not set this
    pub privdata: *mut libc::c_void,
    //void *privdata,
}

impl Default for GPSDataT {
    fn default() -> GPSDataT {
        GPSDataT {
            set: Default::default(),
            online: Default::default(),
            gps_fd:  Default::default(),
            fix: Default::default(),
            separation: Default::default(),
            status: Default::default(),
            satellites_used: Default::default(),
            dop: Default::default(),
            epe: Default::default(),
            skyview_time: Default::default(),
            satellites_visible: Default::default(),
            skyview: [Default::default(); MAXCHANNELS],
            dev: Default::default(),
            policy: Default::default(),
            devices: Default::default(),
            union_never_reported: [Default::default(); 1418],
            privdata: ptr::null_mut(),
        }
    }
}

impl fmt::Display for GPSDataT {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f,
                "\nGPSDATAT \n \
                 Set {}\n \
                 Online {}\n \n\
                 Fix {}\n \n \
                 Separation {}\n \
                 Status {}\n \
                 SatellitesUsed {}\n \
                 Epe {}\n \
                 SkyviewTime {}\n \
                 SatellitesVisible {}\n",
               self.set,
               self.online,
               self.fix,
               self.separation,
               self.status,
               self.satellites_used,
               self.epe,
               self.skyview_time,
               self.satellites_visible,
        )
    }
}

#[repr(C)]
#[derive(Default)]
pub struct Devices {
    pub time: TimestampT,
    pub ndevices: libc::c_int,
    pub list: [DevconfigT; MAXUSERDEVS],
}

#[link(name = "gps")]
extern "C" {
    /// Open connection to gpsd socket
    /// # Arguments
    ///
    /// * `host` - GPSD socket hostname
    /// * `port` - GPSD socket port
    /// * `gpsDataT` - Ref to GPSDataT
    ///
    /// # Example
    ///
    /// ```
    /// let mut gps_data: GPSDataT = Default::default();
    /// let ip = CString::new("127.0.0.1").unwrap();
    /// let port = CString::new("2947").unwrap();
    /// let res = gps_sock_open(ip.as_ptr(), port.as_ptr(), &mut gps_data);
    /// ```
    pub fn gps_sock_open(host: *const libc::c_char, port: *const libc::c_char, gpsDataT: *mut GPSDataT) -> libc::c_int;
    /// Close connection to gpsd socket
    /// # Arguments
    ///
    /// * `gpsDataT` - Ref to GPSDataT
    ///
    /// # Example
    ///
    /// ```
    /// let mut gps_data: GPSDataT = Default::default();
    /// // see gps_sock_open ...
    /// let result = gps_sock_close(&mut gps_data);
    /// ```
    pub fn gps_sock_close(gpsdata: *mut GPSDataT) -> libc::c_int;
    pub fn gps_sock_send(gpsdata: *mut GPSDataT, buf: *const libc::c_char) -> libc::c_int;
    pub fn gps_sock_read(gpsdata: *mut GPSDataT) -> libc::c_int;
    pub fn gps_sock_waiting(gpsdata: *const GPSDataT, timeout: i64) -> bool;
    pub fn gps_sock_stream(gpsdata: *mut GPSDataT, flags: libc::c_int, d: *mut libc::c_void)  -> libc::c_int;
    pub fn gps_sock_data(gpsdata: *const GPSDataT) -> libc::c_char;
    /// Continously read data from socket, invoking the given hook when successful.
    /// # Arguments
    ///
    /// * `gpsDataT` - Ref to GPSDataT
    ///
    /// # Example
    ///
    /// ```
    /// let mut gps_data: GPSDataT = Default::default();
    /// extern "C" fn print_gps(gps_data: *mut GPSDataT) {
    ///    unsafe {
    ///        println!("Timestamp {}", (*gps_data).fix.time);
    ///        println!("{}", (*gps_data));
    ///        println!("{}", (*gps_data).fix);
    ///     }
    /// }
    /// // see gps_sock_open ...
    /// let result = gps_sock_mainloop(&mut gps_data, 20000, print_gps);
    /// gps_sock_stream(&mut gps_data, WATCH_ENABLE | WATCH_JSON, ptr::null_mut());
    /// // see gps_sock_close ...
    /// ```
    pub fn gps_sock_mainloop(GPSDataT: *mut GPSDataT,
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
            use GPSFixT;
            use DopT;
            use DevconfigT;
            use PolicyT;
            use GPSMaskT;
            use TimestampT;
            use SocketT;
            use Devices;
            use gps_sock_open;
            use gps_sock_mainloop;
            use gps_sock_stream;
            use gps_sock_close;
            use WATCH_JSON;
            use WATCH_ENABLE;
            use std::ffi::CString;
            use std::ptr;
            use std::mem;

            let mut gps_data: GPSDataT = Default::default();
            let ip = CString::new("127.0.0.1").expect("Invalid IP");
            let port = CString::new("2947").expect("Invalid Port");

            println!("GPSDataT Size {}", mem::size_of::<GPSDataT>());
            println!("GpsFixT Size {}", mem::size_of::<GPSFixT>());
            println!("DopT Size {}", mem::size_of::<DopT>());
            println!("DevconfigT Size {}", mem::size_of::<DevconfigT>());
            println!("PolicyT Size {}", mem::size_of::<PolicyT>());
            println!("GPSMaskT Size {}", mem::size_of::<GPSMaskT>());
            println!("TimestampT Size {}", mem::size_of::<TimestampT>());
            println!("SocketT Size {}", mem::size_of::<SocketT>());
            println!("Devices Size {}", mem::size_of::<Devices>());

            extern "C" fn print_gps(gps_data: *mut GPSDataT) {
                unsafe {
                    println!("Timestamp {}", (*gps_data).fix.time);
                    println!("{}", (*gps_data));
                    println!("{}", (*gps_data).fix);
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
