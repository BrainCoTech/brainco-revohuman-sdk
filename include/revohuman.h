/*
 * RevoHuman SDK — C API
 *
 * v0.1.3. Feature coverage matches the Python package `revohuman` 0.1.1. The local broadcast is compatible with
 * schema 1 of the Python `revohuman.live`, so the existing viewer in the Python SDK's `simulation/` connects unchanged.
 * Places where C deliberately departs from Python are marked next to the field ("C deviation", "C extension").
 *
 * ── Conventions ─────────────────────────────────────────────────────────────
 * 1. Return values: most functions return RhStatus (0 = success, negative = failure category). Failure details are
 *    **thread-local**: `rh_last_error_code()` gives a numeric code identical in value to Python's `revohuman.ErrorCode`
 *    (E1102 …), and `rh_last_error()` gives a string in the same format as Python's `format_error()`
 *    ("E1102 PortBusyError: …"). A successful call does not clear the previous error.
 * 2. String outputs: `(char *buf, size_t buf_len)`, returning the required length (excluding the terminating NUL) or a
 *    negative RhStatus. When `buf_len > 0` a NUL-terminated (possibly truncated) string is always written;
 *    `buf_len == 0` only queries the length.
 * 3. Struct versioning: every struct that crosses the API on its own starts with `uint32_t struct_size`.
 *    - Option structs: `rh_xxx_options_init(&opts)` fills in struct_size and the defaults of the same-named Python
 *      parameters; then change the fields you need. Passing NULL as the options pointer means all defaults.
 *    - Output structs: the caller sets `struct_size = sizeof(struct)` before the call; the recommended form is
 *      `RhGloveInfo info = RH_STRUCT_INIT(RhGloveInfo);`. The library writes only the first struct_size bytes;
 *      a struct_size smaller than the size of the struct's first released version → RH_ERR_INVALID_ARG.
 *    - Frames received in callbacks have struct_size filled in by the library (= the library's own sizeof); the
 *      caller only reads the fields it knows.
 *    - Structs embedded in other structs (RhSyncStamp, RhTactile, left/right of RhPairedFrame, left/right of
 *      RhProductIdentity) have a frozen layout within a major version; RhGloveFrame keeps reserved words at its tail
 *      for later versions.
 * 4. Handles: produced by `rh_*_connect / open / create / start` and released by `rh_*_close / stop`; all of these
 *    are NULL-safe. `rh_pair_glove()` / `rh_kit_glove()` and similar return **borrowed** pointers that are released
 *    with the parent handle; do not close them separately. When a parent handle is closed, each subscription still
 *    open under it receives one RH_FRAME_END; the subscription handle itself must still be released with
 *    `rh_sub_close()` (calling it after the parent handle is closed is safe). `rh_shutdown()` force-closes all
 *    unreleased handles; do not touch any handle afterwards.
 * 5. Subscriptions: each subscription has one bounded queue (when full, the oldest item is dropped).
 *    - Callback mode: the library starts one thread per subscription that calls the callback sequentially; the frame
 *      pointer received in the callback is valid only during the callback. Do not block in the callback for long
 *      (the queue fills up, the oldest items are dropped and you receive RH_FRAME_LAG).
 *    - Pull mode: pass NULL as the callback and take items with `rh_sub_recv_*()` with a timeout; frames are copied
 *      into the caller's struct.
 *    - Calling `rh_sub_close()` from this subscription's **own** callback is allowed: it returns immediately, no
 *      further callbacks are made, and the library reclaims the handle after the callback returns.
 *    - Closing a **parent handle** (rh_glove_close / rh_pair_close / rh_synchronizer_close / rh_kit_close /
 *      rh_live_stop / rh_shutdown) from any callback thread of this library is not carried out: the library logs an
 *      ERROR and returns. To trigger a close from a callback, signal another thread to do it.
 * 6. Time: `*_wall_ns` = CLOCK_REALTIME nanoseconds, `*_mono_ns` = CLOCK_MONOTONIC nanoseconds.
 *    `sample_*` is the device sampling instant mapped onto the host time axis; `recv_wall_ns` is when the host
 *    received the frame. For one frame, `sample_wall_ns − sample_mono_ns` equals `rh_wall_offset_ns()` at the moment
 *    of mapping; later refreshes of that offset (host clock changes / time-sync steps) are not applied to frames
 *    already delivered. Both hands share the same offset.
 * 7. This API offers no entry point for writing the device serial number (it only reads it, via `rh_glove_get_sn()`),
 *    and no private entry points. Tactile tare (see the "Tactile tare" section) sends only one fixed tare request; it
 *    does not write the serial number, handedness, calibration or any configuration.
 * 8. Runtime notices go to the log; their text starts with "N2xxx " (same codes as Python notices), so a host
 *    application can filter on the first 5 characters.
 * 9. Device node directory: serial and camera nodes are looked up under /dev by default; when the environment variable
 *    REVOHUMAN_DEV_DIR is non-empty they are looked up under it instead (for containers that mount the host's /dev
 *    elsewhere). It is read afresh on every scan / lookup; do not change it from another thread meanwhile.
 *    If an explicitly given port is /dev/... and does not exist while a node of the same name exists under that
 *    directory, the latter is used for the connection.
 *
 * Handedness source: handedness registered in the MCU firmware (firmware 0.20.3 and later; the USB serial numbers of
 * newer gloves no longer carry L / R) > MCU serial number.
 * The registered handedness can only be read over the serial port: the pre-connect lookup (including reconnects),
 * rh_scan's probe and autokit's presence scan each open every present MCU that has not been read yet once, briefly
 * and read-only. Results are cached per boot and per enumeration in /tmp/revohuman/glove_hand_register.json
 * (override with the environment variable REVOHUMAN_HAND_REGISTER_CACHE); ports that are busy or fail to read are not
 * cached and fall back to the serial number. Older firmware has no registered handedness and falls back to the serial
 * number.
 * An MCU whose registered handedness contradicts its serial number → rejected (the lookup reports RH_ERR_CONFLICT);
 * two present MCUs claiming the same hand → neither counts.
 * The C SDK neither reads nor writes the handedness registry glove_sets.yaml (the Python SDK's register / set_name have
 * no counterpart in C). Gloves whose handedness cannot be determined (not registered in firmware, and serial numbers
 * duplicated / without handedness) can only be addressed by port / unit_id; handedness can only be registered with the
 * firmware's companion production test tool.
 *
 * Not in this version: writing encoder zero calibration, wrist cameras, MCAP recording, firmware updates, doctor, and
 * the base station calibration workflow. The tracker (read-only stored calibration + pose stream) has its own section
 * at the end.
 */
#ifndef REVOHUMAN_H
#define REVOHUMAN_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(_WIN32)
#  if defined(RH_STATIC)
#    define RH_API
#  elif defined(RH_BUILDING)
#    define RH_API __declspec(dllexport)
#  else
#    define RH_API __declspec(dllimport)
#  endif
#elif defined(__GNUC__) || defined(__clang__)
#  define RH_API __attribute__((visibility("default")))
#else
#  define RH_API
#endif

/* Initializer for output structs: `RhGloveStats st = RH_STRUCT_INIT(RhGloveStats);` (all other fields zeroed).
 * C uses a designated initializer and C++ uses value-initialization; neither triggers -Wmissing-field-initializers. */
#ifdef __cplusplus
#  define RH_STRUCT_INIT(T) ([]() { T rh_v_{}; rh_v_.struct_size = (uint32_t)sizeof(T); return rh_v_; }())
#else
#  define RH_STRUCT_INIT(T) { .struct_size = (uint32_t)sizeof(T) }
#endif

/* ───────────────────────────── Version ───────────────────────────── */

#define RH_API_VERSION_MAJOR 0
#define RH_API_VERSION_MINOR 1
#define RH_API_VERSION_PATCH 3

/* ───────────────────────────── Constants ───────────────────────────── */

#define RH_JOINT_COUNT            21   /* encoder joints per hand, same order as Python joint_map */
#define RH_TACTILE_MAX_POINTS     350  /* upper bound on tactile points allowed by the protocol (0 and 351 are rejected) */
#define RH_TACTILE_POINTS_TYPE_A  211
#define RH_TACTILE_POINTS_TYPE_B  241
#define RH_STREAM_MIN_RATE_HZ     0.01
#define RH_STREAM_MAX_RATE_HZ     200.0

#define RH_PATH_MAX    256
#define RH_ID_MAX      64   /* topology fingerprints, model codes, group names, etc. */
#define RH_SERIAL_MAX  32
#define RH_NOTE_MAX    256

/* ───────────────────────────── Status codes ───────────────────────────── */

typedef int32_t RhStatus;
enum {
    RH_OK                   = 0,
    RH_ERR_INVALID_ARG      = -1,   /* invalid argument; not a single request byte has been sent; the code is 1000 */
    RH_ERR_NOT_INITIALIZED  = -2,
    RH_ERR_NOT_FOUND        = -3,   /* no device matches the criteria */
    RH_ERR_TIMEOUT          = -4,
    RH_ERR_PROTOCOL         = -5,   /* wire protocol error, device exception response, incompatible firmware */
    RH_ERR_DISCONNECTED     = -6,
    RH_ERR_BUFFER_TOO_SMALL = -7,
    RH_ERR_UNSUPPORTED      = -8,   /* not supported by the firmware / model / this version of the C SDK */
    RH_ERR_CANCELLED        = -9,
    RH_ERR_INVALID_DATA     = -10,  /* invalid content in a configuration / rules file */
    RH_ERR_BUSY             = -11,  /* serial port or tracker held by another process; broadcast socket already has a live listener */
    RH_ERR_STATE            = -12,  /* not possible in the current state (closed, capture gate closed, sync lost, insufficient
                                       evidence…) */
    RH_ERR_CONFLICT         = -13,  /* rate conflict, model differs from the declaration, registered handedness contradicts the serial
                                       number */
    RH_ERR_IO               = -14,  /* serial port / file / socket read or write failed */
    RH_ERR_INTERNAL         = -99,
};

/* Identical in value to Python's `revohuman.ErrorCode`; `rh_last_error_code()` returns one of these.
 * Causes outside the RevoHuman exception tree (including invalid arguments and broadcast socket errors) are always
 * 1000, never 0. */
typedef int32_t RhErrorCode;
enum {
    RH_E_OK                      = 0,
    RH_E_REVOHUMAN_ERROR         = 1000,
    RH_E_TRANSPORT_ERROR         = 1100,
    RH_E_DISCONNECTED            = 1101,
    RH_E_PORT_BUSY               = 1102,
    RH_E_PROTOCOL_ERROR          = 1200,
    RH_E_DEVICE_EXCEPTION        = 1201,  /* the glove returned an exception response */
    RH_E_UNSUPPORTED_FEATURE     = 1202,
    RH_E_TIME_SYNC_UNSUPPORTED   = 1203,  /* the firmware does not support time synchronization */
    RH_E_TIME_SYNC_INCOMPATIBLE  = 1204,  /* the firmware's time synchronization version is incompatible */
    RH_E_FIRMWARE_INCOMPATIBLE   = 1205,
    RH_E_CALIBRATION_ERROR       = 1300,
    RH_E_UNREGISTERED_GLOVE      = 1400,
    RH_E_STREAM_RATE_CONFLICT    = 1500,
    RH_E_CAPTURE_GATE_CLOSED     = 1600,
    RH_E_STRICT_SYNC_LOST        = 1700,
    RH_E_TACTILE_UNSUPPORTED     = 1800,
    RH_E_TACTILE_UNDETERMINED    = 1801,
    RH_E_VARIANT_MISMATCH        = 1802,
    RH_E_PRODUCT_UNIDENTIFIED    = 1803,
    RH_E_PRODUCT_MISMATCH        = 1804,
    RH_E_TRACKER_BUSY            = 1900,
    RH_E_TRACKER_STREAM_DEAD     = 1901,
    RH_E_WORLD_FRAME_CHANGED     = 1902,
    RH_E_LIGHTHOUSE_ERROR        = 1903,
    RH_E_CALIBRATION_REJECTED    = 1904,
    RH_E_TRACKER_NOT_READY       = 1905,
    RH_E_GEOMETRY_LOST           = 1906,
};

/* ───────────────────────────── Enums and bitmasks ───────────────────────────── */

typedef int32_t RhHand;             /* Python handedness; None ↔ UNKNOWN */
enum { RH_HAND_UNKNOWN = 0, RH_HAND_LEFT = 1, RH_HAND_RIGHT = 2 };

typedef int32_t RhHands;            /* open_kit(hands=…) */
enum { RH_HANDS_BOTH = 0, RH_HANDS_LEFT = 1, RH_HANDS_RIGHT = 2 };

/* Sensor bitmask: ProductVariant.sensors / HandReport.sensors / open_kit(sensors=…) */
typedef uint32_t RhSensorMask;
enum {
    RH_SENSOR_ENCODER      = 1u << 0,
    RH_SENSOR_TRACKER      = 1u << 1,
    RH_SENSOR_TACTILE      = 1u << 2,
    RH_SENSOR_WRIST_CAMERA = 1u << 3,
};

typedef int32_t RhTactileState;     /* variant.TactileState */
enum { RH_TACTILE_STATE_UNKNOWN = 0, RH_TACTILE_STATE_NONE = 1, RH_TACTILE_STATE_TACTILE = 2 };

typedef int32_t RhTactileUnit;      /* whether tactile values can be used as force: NEWTON for TYPE_B with 241 points (calibrated force
                                       values; use rh_tactile_newtons to get newtons), otherwise DEVICE_NATIVE (raw counts, not
                                       force) */
enum { RH_TACTILE_UNIT_DEVICE_NATIVE = 0, RH_TACTILE_UNIT_NEWTON = 1 };

typedef int32_t RhTactileTransport; /* Python Glove.tactile_transport: which path the current stream's tactile data takes */
enum {
    RH_TACTILE_TRANSPORT_NONE     = 0,  /* C extension: the model has no tactile (Python reports combined here) */
    RH_TACTILE_TRANSPORT_COMBINED = 1,  /* combined frames (each encoder frame carries the module's latest sample; split not requested,
                                           not supported by the firmware, or the stream has not started yet) */
    RH_TACTILE_TRANSPORT_SPLIT    = 2,  /* split packets (each module sample in its own packet, delivered one by one via
                                           rh_glove_tactile_subscribe) */
};

typedef int32_t RhSyncCause;        /* notices.SyncCause; NONE while the gate is open */
enum {
    RH_CAUSE_NONE             = 0,
    RH_CAUSE_CLOCK_NOT_READY  = 1,
    RH_CAUSE_CLOCK_DEGRADED   = 2,
    RH_CAUSE_HOST_CLOCK_EVENT = 3,
    RH_CAUSE_DEVICE_RESET     = 4,
    RH_CAUSE_LINK_UNSTABLE    = 5,
};

/* codes.TrustState; NONE = the time mapping has no verdict yet. DEGRADED = accuracy slightly reduced but still within
 * the allowed range; data is delivered as usual. The value strings in JSON (rh_glove_stats_json / rh_kit_status_json)
 * are the same as in Python. */
typedef int32_t RhStampTrust;
enum { RH_TRUST_NONE = 0, RH_TRUST_TRUSTED = 1, RH_TRUST_DEGRADED = 2, RH_TRUST_UNTRUSTED = 3 };

/* button.ButtonMode. 0 is reserved for "unknown", so a zero-initialized struct is never misread as END.
 * The device reports END=0 / START=1; the library maps them explicitly instead of casting the value. */
typedef int32_t RhButtonMode;
enum { RH_BUTTON_UNKNOWN = 0, RH_BUTTON_START = 1, RH_BUTTON_END = 2 };

typedef int32_t RhPairMode;         /* BimanualSynchronizer(mode=…) */
enum { RH_PAIR_INTERPOLATE = 0, RH_PAIR_NEAREST = 1 };

typedef int32_t RhSyncPolicy;       /* connect_pair(sync=…) / open_kit(sync=…): False / True / "best-effort" */
enum { RH_SYNC_OFF = 0, RH_SYNC_ON = 1, RH_SYNC_BEST_EFFORT = 2 };
/* Hardware sync done once when both hands connect (the two gloves are aligned to the same USB frame and join the same
 * sync session):
 *   RH_SYNC_ON (default): sync failure → rh_pair_connect / rh_kit_open returns an error;
 *   RH_SYNC_BEST_EFFORT: when sync succeeds, the same as ON (both hands aligned to the same USB frame);
 *     sync failure → warning N2201, then fall back to pairing in the host time domain;
 *   RH_SYNC_OFF: no hardware sync; pairing in the host time domain.
 * After the stream starts, sync quality is self-checked (results in rh_pair_connect_report); if sample alignment exceeds
 * resync_ms, sync is rebuilt while connecting. When sync is lost at runtime (N2202) it is rebuilt automatically according
 * to auto_resync, or manually with rh_pair_resync(). */

typedef int32_t RhProductSource;    /* ProductIdentity.source; NONE is written as "" in the broadcast */
enum {
    RH_PRODUCT_SOURCE_NONE     = 0,
    RH_PRODUCT_SOURCE_SN       = 1,  /* "sn": serial number rules */
    RH_PRODUCT_SOURCE_HARDWARE = 2,  /* "hardware": hardware evidence */
    RH_PRODUCT_SOURCE_DECLARED = 3,  /* "declared": declared by the caller */
};

typedef int32_t RhFrameKind;        /* kind passed to subscription callbacks */
enum {
    RH_FRAME_OK    = 0,  /* frame is valid */
    RH_FRAME_LAG   = 1,  /* the queue overflowed and the oldest frames were dropped; frame is NULL, drop count in rh_sub_dropped() */
    RH_FRAME_END   = 2,  /* upstream closed, the subscription has ended; frame is NULL; no further callbacks */
    RH_FRAME_ERROR = 3,  /* upstream fatal error; frame is NULL; rh_last_error*() on the callback thread gives the details; no further
                            callbacks */
};

/* RhGloveFrame.flags */
typedef uint32_t RhFrameFlags;
enum {
    RH_FRAME_FLAG_GATE_OPEN       = 1u << 0,  /* the capture gate was open at dispatch (gated subscriptions receive only these) */
    RH_FRAME_FLAG_HAS_SAMPLE_TIME = 1u << 1,  /* 0 = sample_*_ns invalid (occurs only on ungated subscriptions).
                                                 The broadcast is an ungated subscription; glove sample_ns is then sent as null */
    RH_FRAME_FLAG_HAS_TACTILE     = 1u << 2,  /* 0 = this hand's model has no tactile, or this frame has no tactile data */
};

/* RhSyncStamp.flags: raw value of the flag bits in the in-frame sync section, 16 bits; unknown bits are kept as-is.
 * Note that this is not the same thing as the 8-bit flags of the button extension section; they do not share a type. */
typedef uint16_t RhSyncFlags;
enum {
    RH_SYNC_FLAG_BUS_CLOCK   = 0x01,  /* the device sees the host USB frame clock */
    RH_SYNC_FLAG_BUS_LOCKED  = 0x02,  /* the sampling clock is following the host USB frame clock */
    RH_SYNC_FLAG_IN_SESSION  = 0x04,  /* joined a two-hand sync session */
    RH_SYNC_FLAG_HOLD        = 0x08,  /* temporarily lost the USB frame clock, holding the most recent rate */
    RH_SYNC_FLAG_STAMP_VALID = 0x10,  /* this frame's sync stamp is valid */
};

typedef int32_t RhLogLevel;
enum { RH_LOG_DEBUG = 10, RH_LOG_INFO = 20, RH_LOG_WARNING = 30, RH_LOG_ERROR = 40, RH_LOG_OFF = 100 };

/* ───────────────────────────── Opaque handles ───────────────────────────── */

typedef struct RhScanList     RhScanList;
typedef struct RhGlove        RhGlove;
typedef struct RhGlovePair    RhGlovePair;
typedef struct RhSynchronizer RhSynchronizer;
typedef struct RhSubscription RhSubscription;
typedef struct RhKit          RhKit;
typedef struct RhAutoKit      RhAutoKit;
typedef struct RhLiveServer   RhLiveServer;
typedef struct RhTracker      RhTracker;

/* ───────────────────────────── Lifecycle and errors ───────────────────────────── */

typedef struct RhInitOptions {
    uint32_t   struct_size;
    RhLogLevel log_level;            /* default RH_LOG_WARNING */
} RhInitOptions;

RH_API void        rh_init_options_init(RhInitOptions *opts);
RH_API RhStatus    rh_init(const RhInitOptions *opts);   /* idempotent */
RH_API void        rh_shutdown(void);                    /* unreleased handles are force-closed with a warning */
RH_API const char *rh_version(void);                     /* static string, e.g. "0.1.3" */

/* E code of the last error (same rules as Python error_code()). */
RH_API RhErrorCode rh_last_error_code(void);
/* "E1200 StreamOptionsRejected: …": the class-name part is the **actual** exception class name, which may be a subclass
 * without its own code (StreamOptionsRejected, ButtonExtensionMismatch, LiveSocketBusy …), same as Python
 * type(exc).__name__. */
RH_API int32_t     rh_last_error(char *buf, size_t buf_len);
/* If the last error was a glove exception response (E1201), returns the exception code the device sent back
 * (0x01 illegal function, 0x02 illegal address…); otherwise returns 0. */
RH_API int32_t     rh_last_error_exception_code(void);
/* Looks up **the root class name of the code** (1200 → "ProtocolError"; unknown codes return NULL).
 * It cannot give subclass names: to see the actual exception class, read rh_last_error(). */
RH_API const char *rh_error_class_name(RhErrorCode code);

typedef void (*RhLogCallback)(RhLogLevel level, const char *message, void *user_data);
RH_API void rh_set_log_level(RhLogLevel level);
/* Writes to stderr by default; once a callback is set, logs go only to the callback; pass NULL to go back to stderr.
 * The callback may come from any internal thread. It is called under a non-recursive lock inside the library, so log
 * lines from all threads enter the callback one at a time: keep the callback fast, and do not call this function or any
 * other rh_* function that writes logs from inside it (deadlock). When this function returns, the old callback is no
 * longer running and will never be called again, so its user_data can be freed. */
RH_API void rh_set_log_callback(RhLogCallback cb, void *user_data);

/* ───────────────────────────── Time ───────────────────────────── */

/* Process-wide offset: wall = mono + offset (measured by bracketing clock reads; refreshed after host clock changes /
 * time-sync steps). */
RH_API int64_t rh_wall_offset_ns(void);

/* ───────────────────────────── Frame structs ───────────────────────────── */

/* In-frame sync section (Python SyncStamp).
 * The pairing key (generation, cycle_sequence) is governed by in_session: it is valid while in_session, and still
 * valid during hold; "key valid, not synchronized" is the normal state under hold. Do not use synchronized to decide
 * whether the key is valid. */
typedef struct RhSyncStamp {
    uint8_t     present;          /* the frame carries a sync section; 0 = the device omitted it (Python sync is None), a protocol
                                     anomaly */
    uint8_t     in_session;       /* generation≠0 and IN_SESSION */
    uint8_t     synchronized;     /* generation≠0, BUS_CLOCK|BUS_LOCKED|IN_SESSION|STAMP_VALID all set, and not HOLD */
    uint8_t     hold;             /* HOLD bit: tells "temporarily lost the clock within a session" from "never joined a session" */
    uint8_t     bus_locked;       /* BUS_LOCKED and not HOLD (can be 1 even on gloves that have not joined a session) */
    uint8_t     _reserved;
    RhSyncFlags flags;            /* raw value; unknown bits kept */
    uint32_t    generation;       /* sync session generation; 0 = never joined a session */
    uint32_t    cycle_sequence;
    uint16_t    frame_number;     /* USB frame number at the sampling instant, mod 2048 */
    uint16_t    _reserved2;
} RhSyncStamp;

/* Python TactileData. Only the first point_count entries of values are valid, and they are raw firmware values: when
 * unit == NEWTON, convert them to newtons with rh_tactile_newtons rather than using them directly as force; when
 * DEVICE_NATIVE, they are raw counts, not force. */
typedef struct RhTactile {
    uint16_t      point_count;
    uint16_t      age_ms;
    uint8_t       valid;
    uint8_t       link_ok;
    uint8_t       calibrated;          /* 1 when unit == NEWTON; always 0 for TYPE_A */
    uint8_t       has_sample_time;     /* 0 = sample_*_ns invalid (mapping not ready); independent of the frame-level flag */
    RhTactileUnit unit;
    uint32_t      device_tick;         /* the tactile data's own sampling tick (ms); always present in live streams */
    int64_t       sample_wall_ns;      /* the tactile data's own sampling instant; use this, not the frame's, for millisecond-level
                                          cross-modal analysis */
    int64_t       sample_mono_ns;
    uint16_t      values[RH_TACTILE_MAX_POINTS];
} RhTactile;

/* Python GloveFrame. */
typedef struct RhGloveFrame {
    uint32_t     struct_size;
    RhHand       hand;                    /* UNKNOWN when connected without a hand */
    uint64_t     seq;                     /* C extension: unwrapped within a session by signed difference (goes backwards if the device
                                             does, so not guaranteed monotonic); low 32 bits = Python seq (the raw u32 on the wire) */
    uint32_t     generation;              /* +1 on every (re)start of the stream; a change = reconnected or a maintenance window ran */
    RhFrameFlags flags;
    uint32_t     device_tick;             /* device **send** tick (ms, uint32, wraps around) */
    uint32_t     joints_valid_mask;       /* bit i = joints_deg[i] is valid */
    int64_t      recv_wall_ns;            /* host receive time */
    int64_t      sample_wall_ns;          /* encoder sampling instant (always valid on gated subscriptions) */
    int64_t      sample_mono_ns;          /* the same instant in MONOTONIC, computed together with sample_wall_ns from the same
                                             offset */
    uint32_t     skipped_snapshots;       /* snapshots skipped on the device side (raw wire value) */
    uint32_t     _reserved0;
    uint64_t     dropped_local;           /* host-side drops accumulated **per stream**; per subscription see rh_sub_dropped() */
    double       joints_deg[RH_JOINT_COUNT]; /* firmware-corrected angles (degrees), unmapped; for mapping see rh_joints_to_radians */
    RhSyncStamp  sync;
    RhTactile    tactile;                 /* valid when flags & HAS_TACTILE */
    uint64_t     _reserved_tail[4];
} RhGloveFrame;

/* Python PairedFrame. The left hand is the reference axis:
 *   skew_ns = signed distance from the left frame's instant to the nearest **real** right sample (positive when the right
 *             is later); an interpolated right frame is not a sample, so in interpolate mode it ≠
 *             right.sample_wall_ns − left.sample_wall_ns. Pairs matched exactly by (generation, cycle_sequence) are
 *             always 0.
 *   In interpolate mode the sample_mono_ns of a synthesized right frame is defined by C (Python frames have no such
 *   field) and is not covered by the golden vectors. */
typedef struct RhPairedFrame {
    uint32_t     struct_size;
    uint32_t     _reserved;
    int64_t      skew_ns;
    RhGloveFrame left;
    RhGloveFrame right;
} RhPairedFrame;

/* Python TactileSample: one tactile sample in a split session (RhGloveOptions.split_tactile = 1 and supported by the
 * firmware). The firmware sends one packet for each module sample it commits; they are delivered one by one via
 * rh_glove_tactile_subscribe.
 * The timestamp semantics differ from the tactile data on frames: tactile.device_tick is an estimate of when "the last
 * UART byte of the complete sample was received", usually a few milliseconds earlier than the tactile tick in combined
 * frames; do not compare data recorded in the two modes with each other.
 * tactile.valid = 0 marks a health-only packet (about one every 100 ms while the firmware data is invalid); point_count
 * may be 0. */
typedef struct RhTactileSample {
    uint32_t  struct_size;
    RhHand    hand;                      /* UNKNOWN when connected without a hand */
    uint32_t  generation;                /* same meaning as RhGloveFrame.generation */
    uint32_t  packet_seq;                /* packet sequence number (counts health-only packets too, +1 per packet) */
    uint32_t  tactile_seq;               /* tactile sample sequence number (health-only packets do not advance it) */
    uint32_t  cycle_sequence;            /* encoder cycle number when the module sample was received (same counter as
                                            RhSyncStamp.cycle_sequence) */
    uint16_t  frame_number;              /* USB frame number when the module sample was received, mod 2048 (same counter as
                                            RhSyncStamp.frame_number); 0 and unusable when frame_stamp_valid = 0 */
    uint16_t  frame_elapsed_us;          /* microseconds since the start of that USB frame; same as above */
    uint8_t   frame_stamp_valid;
    uint8_t   overflow;                  /* the device queue overflowed since the previous packet */
    uint8_t   gate_open;                 /* 0 = a sample delivered to an ungated subscription while the capture gate was closed (for
                                            viewing only, not for alignment) */
    uint8_t   _reserved0;
    uint32_t  dropped_samples;           /* samples lost to device queue overflows since this stream started (cumulative, counts from 0
                                            again after a reconnect) */
    uint32_t  _reserved1;
    int64_t   recv_wall_ns;              /* host receive time of the packet (not the sampling instant) */
    RhTactile tactile;                   /* sampling instant in tactile.sample_*_ns (has_sample_time) */
    uint64_t  _reserved_tail[4];
} RhTactileSample;

/* Tactile force in newtons (N): when unit == RH_TACTILE_UNIT_NEWTON, converts the first point_count points and writes them
 * to out[0..point_count). Does not look at valid / link_ok; whether to use the frame is up to the caller.
 * unit == DEVICE_NATIVE (raw counts, not force) → RH_ERR_UNSUPPORTED;
 * tactile / out is NULL, out_len < point_count, or point_count > RH_TACTILE_MAX_POINTS → RH_ERR_INVALID_ARG.
 * out is not written on failure. Pure function, callable from any thread. */
RH_API RhStatus rh_tactile_newtons(const RhTactile *tactile, float *out, size_t out_len);

typedef struct RhButtonState {
    uint32_t     struct_size;
    RhButtonMode mode;
    uint8_t      pressed;
    uint8_t      has_sample_time;
    uint16_t     _reserved;
    uint32_t     press_sequence;
    uint32_t     generation;
    uint32_t     _reserved2;
    uint64_t     seq;                    /* same as RhGloveFrame.seq: C extension, low 32 bits = Python seq */
    int64_t      recv_wall_ns;
    int64_t      sample_wall_ns;
} RhButtonState;

/* Python ButtonEvent; number of missed presses = press_count − 1. */
typedef struct RhButtonEvent {
    uint32_t     struct_size;
    RhButtonMode mode;                   /* state after the press: START = start recording, END = stop */
    uint32_t     press_sequence;
    uint32_t     press_count;
    uint32_t     generation;
    uint8_t      has_sample_time;
    uint8_t      _reserved[3];
    uint64_t     seq;                    /* same as RhGloveFrame.seq */
    int64_t      recv_wall_ns;
    int64_t      sample_wall_ns;
} RhButtonEvent;

typedef void (*RhGloveFrameCallback)(RhFrameKind kind, const RhGloveFrame *frame, void *user_data);
typedef void (*RhPairedFrameCallback)(RhFrameKind kind, const RhPairedFrame *frame, void *user_data);
typedef void (*RhButtonEventCallback)(RhFrameKind kind, const RhButtonEvent *event, void *user_data);
typedef void (*RhTactileSampleCallback)(RhFrameKind kind, const RhTactileSample *sample, void *user_data);

/* ───────────────────────────── Discovery ───────────────────────────── */

/* Python DiscoveredGlove. With probe=0, fw_version and the fields after it are empty / 0. */
typedef struct RhDiscoveredGlove {
    uint32_t struct_size;
    RhHand   hand;                       /* registered handedness in firmware > MCU serial number; conflicting / duplicated /
                                            undetermined = UNKNOWN (reason in hand_note) */
    char     port[RH_PATH_MAX];          /* empty = present in sysfs but the node is unusable (missing / stale node from before a
                                            replug, convention 9) */
    char     unit_id[RH_ID_MAX];         /* USB topology fingerprint, the identity key (Python usb_topology_id) */
    uint8_t  registered;                 /* handedness determined (= hand is not UNKNOWN) */
    uint8_t  probed;                     /* 1 = this entry really did read device info over a short connection (0 for busy ports) */
    uint8_t  _reserved[2];
    char     usb_serial[RH_SERIAL_MAX];
    char     hand_source[16];            /* register / serial / conflict / none */
    char     hand_note[RH_NOTE_MAX];
    char     tracker_serial[RH_SERIAL_MAX];
    char     fw_version[RH_SERIAL_MAX];
    char     sn[RH_SERIAL_MAX];
    char     product_type[RH_ID_MAX];    /* UNKNOWN / ENCODER_ONLY / ENCODER_TACTILE */
    uint16_t encoder_count;
    uint16_t tactile_point_count;
    char     tactile_kind[16];           /* "TYPE_A" / "TYPE_B"; empty if the type is unrecognized or cannot be read */
} RhDiscoveredGlove;

typedef struct RhScanOptions {
    uint32_t    struct_size;
    uint8_t     probe;                   /* default 0; 1 = short connection to fill in firmware info, and read the handedness
                                            registration once for units not read yet (writes the cache);
                                            0 = registered handedness comes only from what is already in the cache.
                                            C deviation: ports held by another process are skipped, not read and not reported as errors
                                            (Python connects directly without flock) */
    uint8_t     _reserved[3];
    const char *sysfs_root;              /* NULL / "" → /sys/bus/usb/devices (for testing) */
} RhScanOptions;

RH_API void     rh_scan_options_init(RhScanOptions *opts);
RH_API RhStatus rh_scan(const RhScanOptions *opts, RhScanList **out);
RH_API size_t   rh_scan_count(const RhScanList *list);
RH_API RhStatus rh_scan_get(const RhScanList *list, size_t index, RhDiscoveredGlove *out);
RH_API void     rh_scan_free(RhScanList *list);

/* ───────────────────────────── Single glove ───────────────────────────── */

/* Pick one of three ways to locate the glove (hand / port / unit_id); all empty = connect without a hand when there is
 * exactly one glove on the machine.
 * hand is looked up by registered handedness in firmware > MCU serial number: undetermined / two gloves claiming the same
 * hand → RH_ERR_NOT_FOUND (E1400), use port / unit_id instead; an MCU whose registered handedness and serial number
 * contradict each other about this hand → RH_ERR_CONFLICT.
 * port is first redirected under REVOHUMAN_DEV_DIR per convention 9 (when it is set and the condition holds). */
typedef struct RhGloveTarget {
    RhHand      hand;
    const char *port;
    const char *unit_id;
} RhGloveTarget;

typedef struct RhGloveOptions {
    uint32_t    struct_size;
    double      rate_hz;                 /* default 200; the rate belongs to the connection, subscriptions cannot change it */
    uint8_t     require_tactile;         /* default 0; 1 = refuse to connect if the model has no tactile or tactile cannot be
                                            determined */
    uint8_t     button;                  /* default 0; 1 = enable button reports */
    uint8_t     split_tactile;           /* default 0; 1 = request split tactile packets when starting the stream (see
                                            rh_glove_tactile_subscribe); if the firmware does not support it, the connection proceeds
                                            with combined frames */
    uint8_t     _reserved[1];
    uint32_t    sync_loss_debounce;      /* C extension (fixed at 5 in Python): default 5; number of consecutive frames outside the
                                            current sync session generation before sync counts as lost; < 1 is treated as 1 */
} RhGloveOptions;

RH_API void     rh_glove_options_init(RhGloveOptions *opts);
/* First checks that the serial port can be held exclusively (busy → RH_ERR_BUSY / E1102; the error string includes the
 * holder's pid), then briefly connects to read device info (best effort, up to 3 attempts, failure does not block
 * starting the stream; any stream left running from a previous session is stopped along the way), then starts the
 * stream. On return the stream is starting, but the first frame may not have arrived yet. */
RH_API RhStatus rh_glove_connect(const RhGloveTarget *target, const RhGloveOptions *opts, RhGlove **out);
RH_API void     rh_glove_close(RhGlove *glove);          /* stops the stream, closes the serial port, ends all subscriptions (each
                                                            receives one END) */

/* Static info of Python Glove (cached at connect; refreshed by rh_glove_selftest, so it may change). */
typedef struct RhGloveInfo {
    uint32_t       struct_size;
    RhHand         hand;
    char           port[RH_PATH_MAX];
    char           unit_id[RH_ID_MAX];
    char           usb_serial[RH_SERIAL_MAX];
    char           sn[RH_SERIAL_MAX];              /* firmware SN, read-only; empty string if never written */
    char           fw_version[RH_SERIAL_MAX];
    char           product_type[RH_ID_MAX];
    RhTactileState tactile_state;
    RhTactileUnit  tactile_unit;                   /* available before the stream starts; same rule as on frames */
    uint16_t       encoder_count;
    uint16_t       tactile_point_count;            /* 0 without tactile */
    uint16_t       tactile_wire_version;
    uint8_t        button_enabled;
    int8_t         button_supported;               /* 1 / 0 / -1 = not known yet (no stream opened yet) */
    char           tactile_kind[16];               /* same as RhDiscoveredGlove.tactile_kind */
} RhGloveInfo;

RH_API RhStatus rh_glove_info(const RhGlove *glove, RhGloveInfo *out);
RH_API int32_t  rh_glove_get_sn(const RhGlove *glove, char *buf, size_t buf_len);
/* Actual rate (filled in from the device's echo after the stream starts); 0 if not started yet. */
RH_API double   rh_glove_effective_rate_hz(const RhGlove *glove);

typedef struct RhSubscribeOptions {
    uint32_t struct_size;
    uint32_t queue_size;                 /* default 64 (16 for buttons, 64 for pairs) */
    uint8_t  ungated;                    /* default 0; 1 = also receive while the gate is closed (for viewing only, not for alignment),
                                            corresponds to gated=False */
    uint8_t  _reserved[3];
} RhSubscribeOptions;

RH_API void     rh_subscribe_options_init(RhSubscribeOptions *opts);
/* Already fatal → the original fatal error; already closed → RH_ERR_DISCONNECTED (E1101). */
RH_API RhStatus rh_glove_subscribe(RhGlove *glove, const RhSubscribeOptions *opts,
                                   RhGloveFrameCallback cb, void *user_data, RhSubscription **out);

/* Capture gate (unconditional; there is no switch to turn it off). */
RH_API bool         rh_glove_capture_ok(const RhGlove *glove);
RH_API RhSyncCause  rh_glove_capture_cause(const RhGlove *glove);
RH_API RhStampTrust rh_glove_stamp_trust(const RhGlove *glove);

/* Open/close edges of the gate (Python on_capture_gate); called on an internal thread, do not close the glove inside it.
 * Pass NULL to cancel. */
typedef void (*RhCaptureGateCallback)(RhGlove *glove, bool ok, RhSyncCause cause, void *user_data);
RH_API RhStatus rh_glove_on_capture_gate(RhGlove *glove, RhCaptureGateCallback cb, void *user_data);

typedef void (*RhStampTrustCallback)(RhGlove *glove, RhStampTrust state, RhStampTrust prev,
                                     RhSyncCause cause, void *user_data);
RH_API RhStatus rh_glove_on_stamp_trust(RhGlove *glove, RhStampTrustCallback cb, void *user_data);

/* Once the stream can no longer recover, returns its E code (RH_E_OK = still alive; RH_E_FIRMWARE_INCOMPATIBLE when the
 * glove firmware turned out to be incompatible); details go to last_error. */
RH_API RhErrorCode rh_glove_fatal_error(const RhGlove *glove);

/* Common counters of Python stats(); the full dictionary (same keys as to_dict()) via rh_glove_stats_json(). */
typedef struct RhGloveStats {
    uint32_t     struct_size;
    uint32_t     reconnects;             /* counts only reconnects after a stall; backoff retries of failed connects / stream starts
                                            are not counted */
    uint64_t     frames_received;
    uint64_t     frames_dropped_local;
    uint64_t     device_skipped_total;
    uint64_t     seq_gap_frames;
    uint64_t     frames_blocked;         /* frames blocked by the capture gate */
    uint64_t     frames_ungated;         /* of those, the ones delivered to ungated subscriptions */
    uint32_t     generation;             /* number of successful STARTs of the current stream object; counts from 1 again when a
                                            maintenance window replaces the stream object, so it differs from the ever-increasing
                                            generation on frames */
    uint32_t     queue_len;
    uint8_t      capture_open;
    uint8_t      _reserved[3];
    RhSyncCause  capture_cause;
    RhStampTrust stamp_trust;
    uint32_t     _reserved2;
    double       last_frame_age_s;
    double       uptime_s;
    double       effective_rate_hz;
} RhGloveStats;

RH_API RhStatus rh_glove_stats(const RhGlove *glove, RhGloveStats *out);
RH_API int32_t  rh_glove_stats_json(const RhGlove *glove, char *buf, size_t buf_len);

/* Health check: does not change the device configuration, but sends stop / start stream commands, so it is not strictly
 * free of side effects. **Do not call it while recording.**
 * Runs inside a maintenance window: stop the stream → plain connection to read device info and the encoder links →
 * restart the stream; there are no new frames for about 50 ms meanwhile (a two-hand paired stream also has a gap of about
 * 50 ms). Subscriptions stay open during the window and receive no END; generation increments when the dispatch thread
 * gets the first frame of the new stream, not when this function returns. Refreshes the device info cache of
 * rh_glove_info(). Cannot be called from a library callback thread (RH_ERR_STATE). */
typedef struct RhSelftestReport {
    uint32_t struct_size;
    uint8_t  ok;
    uint8_t  _reserved[3];
    uint32_t encoder_offline_mask;       /* bit i = encoder i offline */
    uint32_t _reserved2;
    char     problems[RH_NOTE_MAX * 2];  /* multiple entries separated by '\n'; empty when ok */
} RhSelftestReport;

RH_API RhStatus rh_glove_selftest(RhGlove *glove, RhSelftestReport *out);

/* Button: requires button=1 at connect, otherwise RH_ERR_UNSUPPORTED (E1202).
 * If the firmware does not support it, the connection proceeds as usual, info.button_supported = 0, and the log shows
 * N2406. */
RH_API RhStatus rh_glove_button_read(RhGlove *glove, RhButtonState *out);   /* nothing received yet → RH_ERR_NOT_FOUND */
RH_API RhStatus rh_glove_button_subscribe(RhGlove *glove, const RhSubscribeOptions *opts,
                                          RhButtonEventCallback cb, void *user_data, RhSubscription **out);

/* Split tactile (Python split_tactile / subscribe_tactile). With split_tactile=1 at connect and firmware support, the
 * firmware sends one packet for each module sample it commits, so every sample of the module is available; the tactile
 * data on frames remains and holds the module's latest sample (tactile.valid = 0 on a frame means the latest sample is
 * too old). Without a split request, or without firmware support, frames carry tactile as before and this subscription
 * receives no samples.
 * opts NULL: queue of 512, subject to the capture gate; with opts, opts apply (note that rh_subscribe_options_init gives
 * a default of 64; the sample rate is higher than the frame rate, so 64 overflows easily). Errors are the same as
 * rh_glove_subscribe; if the model has no tactile the subscription is still created but receives no samples.
 * The callback runs on the dispatch thread; do not block. */
RH_API RhStatus rh_glove_tactile_subscribe(RhGlove *glove, const RhSubscribeOptions *opts,
                                           RhTactileSampleCallback cb, void *user_data, RhSubscription **out);
/* Tactile transport actually used by the current stream; NONE if glove is NULL. */
RH_API RhTactileTransport rh_glove_tactile_transport(const RhGlove *glove);

/* ───────────────────────────── Subscriptions (common) ───────────────────────────── */

/* Pull mode: takes one item; the type must match the subscription's source, otherwise RH_ERR_INVALID_ARG.
 * Timeout → RH_ERR_TIMEOUT; upstream closed → RH_ERR_DISCONNECTED (E1101); upstream closed because the glove firmware
 * is incompatible → RH_ERR_PROTOCOL (E1205, log notice N2405; that stream is closed and does not reconnect);
 * timeout_ms < 0 = wait forever. */
RH_API RhStatus rh_sub_recv_glove(RhSubscription *sub, int32_t timeout_ms, RhGloveFrame *out);
RH_API RhStatus rh_sub_recv_paired(RhSubscription *sub, int32_t timeout_ms, RhPairedFrame *out);
RH_API RhStatus rh_sub_recv_button(RhSubscription *sub, int32_t timeout_ms, RhButtonEvent *out);
RH_API RhStatus rh_sub_recv_tactile(RhSubscription *sub, int32_t timeout_ms, RhTactileSample *out);
RH_API uint64_t rh_sub_dropped(const RhSubscription *sub);     /* cumulative items this subscription dropped because its queue was
                                                                  full */
/* From another thread: returns after the callback thread has exited. From this subscription's own callback: returns
 * immediately; the handle is reclaimed after the callback returns. */
RH_API void     rh_sub_close(RhSubscription *sub);

/* ───────────────────────────── Two hands ───────────────────────────── */

typedef struct RhPairOptions {
    uint32_t       struct_size;
    RhSyncPolicy   sync;                 /* default ON: hardware sync, failure is an error; BEST_EFFORT falls back to host pairing on
                                            failure */
    RhGloveTarget  left;                 /* fill in only port / unit_id; all empty = look up by handedness */
    RhGloveTarget  right;
    RhGloveOptions glove;                /* shared by both hands */
    uint8_t        auto_resync;          /* default 1: rebuild sync automatically after it is lost */
    uint8_t        _reserved[3];
    uint32_t       lead_frames;          /* default 100 (valid range 20..1024) */
    uint32_t       sync_attempts;        /* default 3 */
    uint32_t       resync_attempts;      /* default 3 */
    uint32_t       auto_resync_attempts; /* default 5 */
    uint32_t       _reserved2;
    double         sync_timeout_s;       /* default 1.0 */
    double         arrival_check_s;      /* default 1.0 */
    double         arrival_warn_ms;      /* default 5.0 */
    double         align_warn_ms;        /* default 1.0 */
    double         align_fail_ms;        /* default 2.5 */
    double         time_base_timeout_s;  /* default 1.0: how long to wait after the stream starts for both hands to establish a time
                                            base */
    double         resync_ms;            /* default 0.5 */
} RhPairOptions;

/* The two one-time self-checks done at connect_pair (the public face of Python ArrivalCheck / AlignmentCheck). */
typedef struct RhPairConnectReport {
    uint32_t struct_size;
    uint8_t  synchronized;
    uint8_t  arrival_ok;
    uint8_t  align_measurable;
    uint8_t  align_ok;
    double   arrival_diff_ms;            /* median arrival difference of same-key frames */
    double   align_diff_ms;              /* median sampling-instant difference of same-key frames */
    char     reason[RH_NOTE_MAX];        /* reason when not ok */
} RhPairConnectReport;

RH_API void     rh_pair_options_init(RhPairOptions *opts);
RH_API RhStatus rh_pair_connect(const RhPairOptions *opts, RhGlovePair **out);
RH_API void     rh_pair_close(RhGlovePair *pair);
RH_API RhGlove *rh_pair_glove(RhGlovePair *pair, RhHand hand);          /* borrowed */
RH_API bool     rh_pair_synchronized(const RhGlovePair *pair);
RH_API RhStatus rh_pair_connect_report(const RhGlovePair *pair, RhPairConnectReport *out);
/* Manually rebuilds two-hand sync (the stream stops for a few hundred ms; subscriptions stay open). Failure returns
 * RH_ERR_STATE; with strict=1, failing to establish sync is E1700.
 * Cannot be called from a library callback thread (RH_ERR_STATE); a pair not created by rh_pair_connect →
 * RH_ERR_UNSUPPORTED. */
RH_API RhStatus rh_pair_resync(RhGlovePair *pair, bool strict);

typedef void (*RhSyncLostCallback)(RhGlovePair *pair, const char *reason, void *user_data);
typedef void (*RhResyncCallback)(RhGlovePair *pair, void *user_data);
/* If sync has already been lost when the callback is attached, it fires once immediately (same as Python). */
RH_API RhStatus rh_pair_on_sync_lost(RhGlovePair *pair, RhSyncLostCallback cb, void *user_data);
RH_API RhStatus rh_pair_on_resync(RhGlovePair *pair, RhResyncCallback cb, void *user_data);

typedef struct RhSynchronizerOptions {
    uint32_t   struct_size;
    RhPairMode mode;                     /* default INTERPOLATE */
    int64_t    max_skew_ns;              /* default 50 000 000 */
    double     pair_accuracy_ms;         /* default 1.0; NaN also means the default; 0.0 = emit only hardware-synced pairs; negative =
                                            no output accuracy gate */
    int32_t    tactile_gate;             /* -1 = auto (default) / 0 off / 1 on */
    uint32_t   _reserved;
} RhSynchronizerOptions;

typedef struct RhSynchronizerStats {
    uint32_t struct_size;
    uint32_t resyncs;
    uint64_t pairs_emitted;
    uint64_t pairs_blocked;
    uint64_t left_dropped;
    uint64_t device_pairs;
    uint64_t host_pairs;
    uint32_t resync_recoveries;
    uint8_t  strict_sync_lost;
    uint8_t  key_trusted;
    uint8_t  device_skew_degraded;
    uint8_t  tactile_gate_open;
    double   device_skew_p50_ms;         /* NaN until it has been settled */
} RhSynchronizerStats;

RH_API void     rh_synchronizer_options_init(RhSynchronizerOptions *opts);
/* Created from a pair (the time domain automatically uses device mode, and resync is hooked up automatically). */
RH_API RhStatus rh_pair_synchronizer(RhGlovePair *pair, const RhSynchronizerOptions *opts, RhSynchronizer **out);
/* Created from any two gloves (falls back to host mode when they were not connected via connect_pair). */
RH_API RhStatus rh_synchronizer_create(RhGlove *left, RhGlove *right, const RhSynchronizerOptions *opts,
                                       RhSynchronizer **out);
RH_API RhStatus rh_synchronizer_subscribe(RhSynchronizer *sync, const RhSubscribeOptions *opts,
                                          RhPairedFrameCallback cb, void *user_data, RhSubscription **out);
RH_API bool     rh_synchronizer_strict_sync_lost(const RhSynchronizer *sync);
RH_API RhStatus rh_synchronizer_stats(const RhSynchronizer *sync, RhSynchronizerStats *out);
RH_API int32_t  rh_synchronizer_stats_json(const RhSynchronizer *sync, char *buf, size_t buf_len);
RH_API void     rh_synchronizer_close(RhSynchronizer *sync);

/* ───────────────────────────── Product identification ───────────────────────────── */

/* Fixed-length fields of Python HandReport; lists such as claims / notes are only in the JSON. Embedded in
 * RhProductIdentity; layout frozen.
 * complete / missing are not separate fields: they follow from variant_sensors and the sensors of the requested hands
 * (same as to_dict). */
typedef struct RhHandReport {
    RhHand         hand;
    uint8_t        present;
    uint8_t        conflict;
    uint8_t        requested;            /* this hand is in opts.hands; when 0, all other fields are zero and it is absent from hands
                                            in the JSON */
    uint8_t        tactile_valid;        /* 0 ↔ Python tactile is None (not probed and no hint given); the JSON writes null */
    RhSensorMask   sensors;              /* sensors present on this hand */
    RhTactileState tactile;
    char           port[RH_PATH_MAX];
    char           unit_id[RH_ID_MAX];
    char           usb_serial[RH_SERIAL_MAX];
    char           hand_source[16];      /* register / serial / conflict / none */
    char           firmware_sn[RH_SERIAL_MAX];
    char           tracker_serial[RH_SERIAL_MAX];
    char           tracker_source[16];   /* register / serial (the tracker actually attached to the HUB of the unit located by that
                                            source) / empty */
    char           camera_usb_id[16];
    char           camera_node[RH_PATH_MAX];
    char           tactile_note[RH_NOTE_MAX];
    char           sku_rule[RH_ID_MAX];  /* match string of the serial number rule that matched, empty if none (overlong strings are
                                            rejected when the rules are loaded, see rules_path) */
    char           variant[16];          /* model code determined for this hand alone, empty if undetermined */
} RhHandReport;

typedef struct RhProductIdentity {
    uint32_t        struct_size;
    RhProductSource source;
    char            variant_code[16];    /* "P1" / "PTV1"; empty if undetermined */
    char            variant_name[RH_ID_MAX];
    RhSensorMask    variant_sensors;
    uint32_t        _reserved;
    RhHandReport    left;
    RhHandReport    right;
} RhProductIdentity;

/* Bits of RhIdentifyOptions.tactile_given */
enum { RH_IDENTIFY_TACTILE_LEFT = 1u << 0, RH_IDENTIFY_TACTILE_RIGHT = 1u << 1 };

typedef struct RhIdentifyOptions {
    uint32_t       struct_size;
    uint8_t        probe;                /* default 1: short connection to read the firmware SN and tactile info */
    uint8_t        tactile_given;        /* default 0. Hands whose bit is set are not probed; tactile_left / tactile_right are used
                                            directly (UNKNOWN is accepted as well, same as Python `hand in tactile`); other bits →
                                            RH_ERR_INVALID_ARG */
    uint8_t        _reserved[2];
    RhHands        hands;                /* default RH_HANDS_BOTH; Python's list form (ordered / empty list) is not offered in C */
    RhTactileState tactile_left;         /* read only when the tactile_given bit is set; out-of-range value → RH_ERR_INVALID_ARG */
    RhTactileState tactile_right;
    uint32_t       _reserved2;
    const char    *product;              /* NULL is equivalent to "auto": decide by evidence; "P1" / "PTV1" = declaration. Leading and
                                            trailing whitespace trimmed, case-insensitive; unknown model code → RH_ERR_INVALID_ARG */
    const char    *sysfs_root;
    const char    *rules_path;           /* NULL → $REVOHUMAN_PRODUCTS → ~/.config/revohuman/products.yaml (if the file does not exist
                                            only the built-in rules are used; if it exists, user rules come before the built-in ones);
                                            a regex in the rules beyond the common subset, or a match string longer than RH_ID_MAX-1
                                            bytes → RH_ERR_INVALID_DATA */
} RhIdentifyOptions;

RH_API void     rh_identify_options_init(RhIdentifyOptions *opts);
RH_API RhStatus rh_identify(const RhIdentifyOptions *opts, RhProductIdentity *out);
RH_API int32_t  rh_identify_json(const RhIdentifyOptions *opts, char *buf, size_t buf_len);  /* == to_dict() */
/* Model code → sensor bitmask (unknown codes return 0). */
RH_API RhSensorMask rh_variant_sensors(const char *code);

/* ───────────────────────────── Joint mapping ───────────────────────────── */

typedef struct RhJointSpec {
    uint32_t struct_size;
    uint32_t index;
    double   sign;                       /* +1.0 / −1.0; 0.0 = this channel is zeroed and unused */
    double   zero_deg;
    double   offset_rad;
    char     name[RH_ID_MAX];            /* without the handedness prefix, e.g. "index_DIP_joint" */
    char     finger[16];                 /* index / middle / ring / little / thumb */
    char     joint[16];                  /* DIP / PIP / MCP / MPR / CMR / CMP */
} RhJointSpec;

/* Mapping table version, e.g. "0.2" (< 1.0 means the official mapping is still to be confirmed). */
RH_API const char *rh_joint_map_version(void);
/* product_code NULL or unknown → falls back to the "default" table (same as Python joint_specs, no error). */
RH_API RhStatus rh_joint_spec(const char *product_code, uint32_t index, RhJointSpec *out);
/* "<left|right>_<name>", i.e. the name in the broadcast's hello.joints; no prefix when hand = UNKNOWN. */
RH_API int32_t  rh_joint_name(RhHand hand, uint32_t index, char *buf, size_t buf_len);
/* q = sign·radians(wrap180(deg − zero)) + offset, bit-identical to Python: NaN in, NaN out; ±inf gives NaN;
 * +180 may come out at the wrap180 boundary (not clamped). */
RH_API RhStatus rh_joints_to_radians(const char *product_code, const double deg[RH_JOINT_COUNT],
                                     double rad[RH_JOINT_COUNT]);

/* ───────────────────────────── Kit ───────────────────────────── */

typedef struct RhKitOptions {
    uint32_t       struct_size;
    RhHands        hands;                /* default BOTH */
    const char    *product;              /* default "auto"; "P1" / "PTV1" = declaration, refused if it contradicts the present
                                            hardware */
    RhSensorMask   sensors;              /* 0 = everything the model should have and this version of the C SDK supports; requesting an
                                            unsupported sensor → UNSUPPORTED */
    uint8_t        strict;               /* default 1: requested but missing from the present hardware → E1804 */
    uint8_t        _reserved[3];
    RhSyncPolicy   sync;                 /* default ON */
    uint32_t       _reserved2;
    double         rate_hz;              /* default 200 */
    const char    *sysfs_root;
    const char    *rules_path;
    RhGloveOptions glove;
    const char    *tracker_config_json;  /* reserved, not read in this version (the C SDK has no tracker: TRACKER in sensors →
                                            UNSUPPORTED) */
} RhKitOptions;

typedef void (*RhKitFaultCallback)(RhKit *kit, RhErrorCode code, bool recoverable, void *user_data);

RH_API void          rh_kit_options_init(RhKitOptions *opts);
RH_API RhStatus      rh_kit_open(const RhKitOptions *opts, RhKit **out);
RH_API void          rh_kit_close(RhKit *kit);
RH_API RhStatus      rh_kit_identity(const RhKit *kit, RhProductIdentity *out);
RH_API RhSensorMask  rh_kit_sensors(const RhKit *kit);                 /* the ones actually enabled */
RH_API RhGlove      *rh_kit_glove(RhKit *kit, RhHand hand);             /* borrowed; NULL if this hand is not open */
RH_API RhGlovePair  *rh_kit_pair(RhKit *kit);                           /* borrowed; NULL for a single hand */
RH_API RhTracker    *rh_kit_tracker(RhKit *kit);                        /* borrowed; NULL if not open */
RH_API RhStatus      rh_kit_synchronizer(RhKit *kit, const RhSynchronizerOptions *opts, RhSynchronizer **out);
RH_API RhErrorCode   rh_kit_fatal(const RhKit *kit);                    /* RH_E_OK = no latched fault */
RH_API RhStatus      rh_kit_on_fault(RhKit *kit, RhKitFaultCallback cb, void *user_data);
RH_API int32_t       rh_kit_status_json(const RhKit *kit, char *buf, size_t buf_len);  /* == kit.status() */

/* ───────────────────────────── Automatic kit (autokit) ───────────────────────────── */

/* Automatically manages opening, retrying and reopening a pair of gloves (C extension, not in Python). rh_autokit_open
 * returns immediately; opening and closing both happen on a background thread inside the library. The caller polls each
 * hand's state (rh_autokit_status) and pulls frames (rh_autokit_recv). Rules:
 *   - Present = a unit that is in sysfs and whose handedness can be determined (for the handedness source see the file
 *     header; units whose handedness registration has not been read yet are opened briefly once during the scan).
 *   - Start: both present → one two-hand kit (sync per kit.sync, default BEST_EFFORT); otherwise each hand is opened on
 *     its own and a missing one is retried every retry_ms. Later, if neither hand is open yet and both become present at
 *     the same time → likewise a two-hand kit; if the two-hand kit cannot be opened → switch to opening each hand on its
 *     own right away.
 *   - One hand is streaming alone and the other becomes present later: the other one is opened on its own without
 *     interrupting the streaming one. With upgrade_to_pair (off by default) the lone one is closed instead and the whole
 *     pair is reopened: the streaming hand stops for about the duration of one open and its open_count increments; if the
 *     pair cannot be opened it goes back to opening each hand on its own, and the next attempt waits until the other hand
 *     has been unplugged once; non-retryable errors (such as time sync not supported) are not retried; two hands whose
 *     pair was broken (pair_lost) are not upgraded.
 *   - Open failure: retryable failures (not plugged in, port busy, timeout, re-enumeration during identification, etc.)
 *     are retried after retry_ms; model / variant mismatch, incompatible firmware, etc. → FAILED, no more retries.
 *   - Disconnects are handled by the kit itself, which reconnects and rebuilds sync without ending the stream; a stream
 *     ends (rh_sub_recv_glove returns an error) only on an upstream fatal error, which is handled here: a single-hand kit
 *     is closed and reopened after retry_ms; a two-hand kit is closed as a whole and both hands go to AWAIT_PAIR: if both
 *     are present the pair is reopened; if only one has been present for pair_wait_ms it is opened on its own, the other
 *     one is later also opened on its own, both get pair_lost, and they are no longer paired automatically (until
 *     rh_autokit_close).
 *   - open_count increments every time the hand is (re)opened; frames, identity and device info with the same open_count
 *     come from the same stream.
 * Degradation is always explicit: paired = in a two-hand kit; synchronized = paired and hardware-synchronized
 * (rh_pair_synchronized); pair_lost see above.
 * Threads: status / recv / identity / glove_info can be called concurrently from any thread; recv reads outside the lock,
 * so a non-zero timeout_ms does not hold up other calls.
 * rh_autokit_close may run concurrently with these calls on other threads: it first waits for in-flight calls to return
 * (recv waits at most its timeout_ms), then stops the background thread and closes all subscriptions and kits; calls that
 * come in after close has started report RH_ERR_INVALID_ARG (handle closed). */
typedef int32_t RhAutoState;
enum { RH_AUTO_ABSENT = 0, RH_AUTO_OPENING = 1, RH_AUTO_AWAIT_PAIR = 2, RH_AUTO_STREAMING = 3, RH_AUTO_FAILED = 4 };

typedef struct RhAutoKitOptions {
    uint32_t     struct_size;
    uint32_t     retry_ms;               /* default 2000 */
    uint32_t     pair_wait_ms;           /* default 10000: after a pair breaks and only one hand comes back, how long to wait before
                                            opening it on its own */
    uint32_t     queue_size;             /* default 256: subscription queue per glove */
    RhKitOptions kit;                    /* hands accepts only BOTH; sync defaults to BEST_EFFORT; the rest as in rh_kit_open */
    uint8_t      upgrade_to_pair;        /* default 0; 1 = a hand streaming alone is upgraded to a full pair when the other one arrives
                                            (see above) */
    uint8_t      _reserved[7];
} RhAutoKitOptions;

typedef struct RhAutoHandStatus {
    uint32_t          struct_size;
    RhAutoState       state;
    uint32_t          open_count;
    uint8_t           paired;
    uint8_t           synchronized;
    uint8_t           pair_lost;
    uint8_t           absent;            /* the last open failed because the glove was not plugged in */
    uint8_t           capture_ok;        /* valid when STREAMING (rh_glove_capture_ok) */
    uint8_t           _reserved[3];
    RhSyncCause       capture_cause;
    RhErrorCode       last_error;        /* code of the last open failure or stream end; cleared after a successful open */
    uint32_t          tactile_dropped;   /* items dropped by the tactile sample subscription since this open (capped at UINT32_MAX); 0
                                            without split */
    uint64_t          dropped;           /* frames dropped by the SDK since this open (rh_sub_dropped) */
    char              last_error_text[256];  /* when ABSENT, also carries the hand_note of present gloves whose handedness cannot be
                                                determined (e.g. both have the same serial number) */
} RhAutoHandStatus;

RH_API void     rh_autokit_options_init(RhAutoKitOptions *opts);
/* opts NULL = all defaults; kit.hands not BOTH, retry_ms or queue_size 0, or an invalid rate → RH_ERR_INVALID_ARG. */
RH_API RhStatus rh_autokit_open(const RhAutoKitOptions *opts, RhAutoKit **out);
/* Waits for in-flight calls to return and for the background thread to finish its current step, then closes all
 * subscriptions and kits. NULL-safe; must not be called from a library callback thread. */
RH_API void     rh_autokit_close(RhAutoKit *autokit);
/* Snapshot of the hand's state. */
RH_API RhStatus rh_autokit_status(RhAutoKit *autokit, RhHand hand, RhAutoHandStatus *out);
/* Next frame of the hand's current stream (same as rh_sub_recv_glove); when open_count is non-NULL it receives the open
 * number the frame belongs to. Not streaming → RH_ERR_STATE; no frame → RH_ERR_TIMEOUT; stream ended → the error of
 * rh_sub_recv_glove (RH_ERR_DISCONNECTED, or RH_ERR_PROTOCOL for incompatible firmware), and the background thread then
 * handles it per the rules above. */
RH_API RhStatus rh_autokit_recv(RhAutoKit *autokit, RhHand hand, int32_t timeout_ms, RhGloveFrame *out,
                                uint32_t *open_count);
/* Next tactile sample of the hand's current stream (same as rh_sub_recv_tactile); rules as for rh_autokit_recv. With
 * kit.glove.split_tactile = 1, every open also opens a tactile subscription (queue of 512, gated); otherwise →
 * RH_ERR_UNSUPPORTED. If the firmware does not support split packets it keeps returning RH_ERR_TIMEOUT and the tactile
 * data stays on the frames (see rh_glove_tactile_transport). */
RH_API RhStatus rh_autokit_recv_tactile(RhAutoKit *autokit, RhHand hand, int32_t timeout_ms, RhTactileSample *out,
                                        uint32_t *open_count);
/* Product identity (same as rh_kit_identity) / device info (same as rh_glove_info, the snapshot taken at open) of the
 * hand's current stream. When open_count is non-NULL it receives the corresponding open number; it is the same stream
 * only if it equals the one given by rh_autokit_recv. Not streaming → RH_ERR_STATE; out NULL or struct_size too small →
 * RH_ERR_INVALID_ARG; out is not written on failure. */
RH_API RhStatus rh_autokit_identity(RhAutoKit *autokit, RhHand hand, RhProductIdentity *out, uint32_t *open_count);
RH_API RhStatus rh_autokit_glove_info(RhAutoKit *autokit, RhHand hand, RhGloveInfo *out, uint32_t *open_count);

/* ───────────────────────────── Tactile tare ───────────────────────────── */

/* Tactile hardware tare: the library sends one fixed tare request to the glove; the glove passes the command on to the
 * tactile modules, which take their current readings as zero.
 * This is a write to the device: request it only while the glove is still and the tactile surfaces are not touched, and
 * keep them untouched for about 3 s after the request.
 *   - rh_glove_tactile_tare_request returns immediately and the procedure runs on a library background thread; poll the
 *     result with rh_glove_tactile_tare_status; a changed version means there is a new state.
 *   - The successful final state is RH_TARE_RESULT_SENT: the tare command has been sent; whether the modules completed
 *     it is not confirmed (the glove does not report module completion).
 *   - The library never resends on its own. When the result is unconfirmed (UNCONFIRMED / TIMEOUT / ABANDONED), the
 *     caller decides whether to request again.
 *   - Holding the glove button for about 2 s also tares, sharing the device's request sequence number with host requests;
 *     while the stream runs the library reads the request record once per second, so tares started by the button or by
 *     another host show up in origin / outcome (and an INFO log line is written).
 *   - Whether the host can trigger a tare is checked once before every (re)connect and stream start; until then support
 *     is UNKNOWN.
 *   - The request record lives in the glove MCU's memory and is cleared when the MCU resets (mcu_reset = 1); if the
 *     glove has reconnected since the tare, stale = 1. In both cases it is not confirmed whether the modules kept their
 *     zero, and tared is set to 0.
 * Taring does not change the format or unit of the tactile data in frames. Threads: both functions can be called from
 * any thread (including library callback threads); request does not block. */
typedef int32_t RhTareSupport;
enum { RH_TARE_SUPPORT_UNKNOWN = 0, RH_TARE_SUPPORT_SUPPORTED = 1, RH_TARE_SUPPORT_UNSUPPORTED = 2 };

/* Who started the most recent tare request (whoever it was). */
typedef int32_t RhTareOrigin;
enum {
    RH_TARE_ORIGIN_NONE       = 0,
    RH_TARE_ORIGIN_THIS_HOST  = 1,   /* started by this handle */
    RH_TARE_ORIGIN_BUTTON     = 2,   /* glove button */
    RH_TARE_ORIGIN_OTHER_HOST = 3,   /* a host request, but not from this handle (another process / an earlier connection) */
    RH_TARE_ORIGIN_UNKNOWN    = 4,
};

/* Outcome of the most recent tare request (whoever started it). */
typedef int32_t RhTareOutcome;
enum {
    RH_TARE_OUTCOME_NONE       = 0,  /* no request record on the device */
    RH_TARE_OUTCOME_PENDING    = 1,  /* accepted by the glove, in progress */
    RH_TARE_OUTCOME_SENT       = 2,  /* tare command sent; whether the modules completed it is not confirmed */
    RH_TARE_OUTCOME_FAILED     = 3,
    RH_TARE_OUTCOME_CANCELLED  = 4,
    RH_TARE_OUTCOME_IDLE       = 5,  /* abnormal record state */
    RH_TARE_OUTCOME_SUPERSEDED = 6,  /* this handle's request was overridden by another request */
    RH_TARE_OUTCOME_TIMEOUT    = 7,  /* this handle timed out waiting for an outcome */
};

/* How this handle's most recent rh_glove_tactile_tare_request ended. */
typedef int32_t RhTareResult;
enum {
    RH_TARE_RESULT_NONE        = 0,  /* never requested */
    RH_TARE_RESULT_RUNNING     = 1,
    RH_TARE_RESULT_SENT        = 2,  /* tare command sent; whether the modules completed it is not confirmed */
    RH_TARE_RESULT_REJECTED    = 3,  /* rejected by the glove on the spot (unsupported / busy / module offline / invalid data /
                                        internal error);
                                        no tare done */
    RH_TARE_RESULT_NOT_SENT    = 4,  /* reading the request record before sending failed; the request was not sent */
    RH_TARE_RESULT_UNCONFIRMED = 5,  /* the response to the request was lost and the record did not advance either: whether it was
                                        accepted is unconfirmed */
    RH_TARE_RESULT_FAILED      = 6,  /* the glove reported failure (the command may already have been sent) */
    RH_TARE_RESULT_CANCELLED   = 7,
    RH_TARE_RESULT_SUPERSEDED  = 8,  /* overridden by another request */
    RH_TARE_RESULT_TIMEOUT     = 9,  /* timed out waiting for an outcome (including a connection loss meanwhile): result unconfirmed */
    RH_TARE_RESULT_ABANDONED   = 10, /* the handle was closed while waiting for an outcome: result unconfirmed */
};

typedef struct RhTactileTareStatus {
    uint32_t      struct_size;
    RhTareSupport support;
    uint8_t       in_progress;       /* this handle's request is in progress */
    uint8_t       tared;             /* outcome == SENT, with no reconnect and no MCU reset since */
    uint8_t       stale;             /* the glove has reconnected since the most recent tare (or the tare predates this connection) */
    uint8_t       mcu_reset;         /* the device's request record was cleared: the glove MCU has reset */
    uint8_t       has_record;        /* a request record has been read from the device */
    uint8_t       _reserved[3];
    RhTareOrigin  origin;
    RhTareOutcome outcome;
    RhTareResult  request_result;
    uint32_t      sequence;          /* device request sequence number (valid with has_record): shared by button and host, +1 per
                                        accepted request, wraps around */
    uint64_t      requests;          /* number of requests this handle has accepted */
    uint64_t      version;           /* +1 on every state change */
    uint64_t      poll_errors;       /* number of failed request record reads */
    int64_t       outcome_wall_ns;   /* host time at which outcome was established, 0 = none */
    int64_t       request_wall_ns;   /* host time at which request_result was settled, 0 = none */
    int64_t       record_wall_ns;    /* host time of the most recent request record read, 0 = never read */
    char          message[RH_NOTE_MAX];          /* description of outcome */
    char          request_message[RH_NOTE_MAX];  /* description of request_result (e.g. the rejection reason) */
    char          support_message[RH_NOTE_MAX];  /* why it is unsupported / not checked yet */
} RhTactileTareStatus;

/* Requests one tare and returns immediately. Accepted → RH_OK, result via rh_glove_tactile_tare_status. Not accepted (no
 * request is sent):
 *   the model has no tactile → RH_ERR_UNSUPPORTED (E1800); the firmware does not support host-triggered tare →
 *   RH_ERR_UNSUPPORTED (E1202), use the button tare instead (hold the glove button for about 2 s);
 *   capability not checked yet / stream not running (connecting, reconnecting, in maintenance) / the previous one still
 *   in progress → RH_ERR_STATE; handle closed → RH_ERR_DISCONNECTED. */
RH_API RhStatus rh_glove_tactile_tare_request(RhGlove *glove);
/* Snapshot of the tare state; out NULL or struct_size too small → RH_ERR_INVALID_ARG. */
RH_API RhStatus rh_glove_tactile_tare_status(const RhGlove *glove, RhTactileTareStatus *out);
/* Tare for the hand's current stream in autokit (same as the two functions above). When open_count is non-NULL it
 * receives the corresponding open number; the state starts over on every reopen.
 * Not streaming → RH_ERR_STATE. */
RH_API RhStatus rh_autokit_tactile_tare_request(RhAutoKit *autokit, RhHand hand, uint32_t *open_count);
RH_API RhStatus rh_autokit_tactile_tare_status(RhAutoKit *autokit, RhHand hand, RhTactileTareStatus *out,
                                               uint32_t *open_count);

/* ───────────────────────────── Local broadcast ───────────────────────────── */

/* Same wire protocol as Python `revohuman live` (schema 1): AF_UNIX stream socket, 4-byte big-endian length + compact
 * UTF-8 JSON. A new connection first receives hello, then the latest status; each client has its own send thread and a
 * bounded queue (the oldest is dropped when full), and a client is disconnected when writing one whole message takes
 * longer than send_timeout_s; stop broadcasts bye. The existing viewer in the Python SDK's simulation/ connects directly.
 *
 * Socket path resolution (server side): explicit socket_path → $REVOHUMAN_LIVE_SOCKET (non-empty) →
 * $XDG_RUNTIME_DIR/revohuman/live.sock (when that variable is itself a directory that is writable and searchable) →
 * /tmp/revohuman-<uid>/live.sock.
 * The first two are "foreign paths": their directory must already exist, and the library does not create it, check its
 * owner or change its permissions; the last two are "own paths": the directory is created with mode 0700 and must belong
 * to the current user and not be a symlink. The socket file is 0600. Absolute paths of ≥ 104 bytes are an error.
 *
 * Errors: outside the RevoHuman exception tree; rh_last_error_code() is always 1000.
 *   RH_ERR_BUSY (class name LiveSocketBusy): there is a live listener on the path; the test connection timed out
 *     (1.0 s) or got an error other than ECONNREFUSED / ENOENT; the path was taken between the check and bind
 *     (EADDRINUSE). In none of these cases is the file **deleted**.
 *   RH_ERR_IO (class name LiveSocketError): a non-socket file at the path; a foreign directory does not exist; an own
 *     directory has the wrong owner or is a symlink; other bind errors.
 *   RH_ERR_INVALID_ARG (class name LiveSocketError): path too long.
 *   A test connection that gets ECONNREFUSED / ENOENT means a dead socket left over from last time; it is deleted and
 *   the start continues. */

typedef int32_t RhLiveTrackerSource;
enum {
    RH_LIVE_TRACKER_KIT      = 0,  /* default: use rh_kit_tracker(kit); if there is none, no poses are broadcast */
    RH_LIVE_TRACKER_EXTERNAL = 1,  /* use the session given in the tracker field (borrowed, must outlive the broadcast);
                                      not implemented in this version of the C SDK: rh_live_start always returns RH_ERR_UNSUPPORTED and
                                      does not read the tracker field (the tracker module will come separately) */
    RH_LIVE_TRACKER_NONE     = 2,  /* no poses are broadcast; hello.tracker.reason is set to tracker_reason */
};

typedef struct RhLiveOptions {
    uint32_t            struct_size;
    uint32_t            queue_size;      /* default 256 (per client) */
    const char         *socket_path;     /* NULL = resolve in the order above */
    RhLiveTrackerSource tracker_source;  /* default KIT */
    uint32_t            backlog;         /* default 8 */
    RhTracker          *tracker;         /* used only when tracker_source == EXTERNAL */
    const char         *tracker_reason;  /* NULL / "" = Python's fallback: the model has a tracker → "tracker not opened", otherwise
                                            "none (product has no tracker)". The reason text is sent exactly as in Python */
    const char         *tracker_gate;    /* NULL → hello.tracker.gate is null; otherwise "flag" / "error" */
    double              send_timeout_s;  /* default 2.0 */
    double              pose_poll_hz;    /* default 250; accepted in this version, but no pose_loop thread is started, so it has no
                                            effect (the tracker module will come separately) */
    double              status_period_s; /* default 1.0 */
} RhLiveOptions;

/* Python Broadcaster.stats(). */
typedef struct RhLiveStats {
    uint32_t struct_size;
    uint32_t clients;                    /* current number of subscribers */
    uint64_t published;                  /* messages encoded and sent (hello not counted) */
    uint64_t poses;                      /* pose messages broadcast; always 0 in this version (no pose_loop thread is started; the
                                            tracker module will come separately) */
    uint64_t dropped;                    /* total messages dropped on full queues by the **currently connected** subscribers
                                            (disconnected ones not counted, same as Python) */
    uint64_t frames[2];                  /* [0] left / [1] right: glove frames broadcast; 0 if this hand is not open */
    uint8_t  hand_open[2];               /* this hand is in the broadcast */
    uint8_t  _reserved[6];
} RhLiveStats;

RH_API void     rh_live_options_init(RhLiveOptions *opts);
/* Only the occupancy check (the step before bind), so that a CLI / example can check it before opening the gloves. When
 * path is NULL it is resolved in the default order.
 * Returns RH_OK / RH_ERR_BUSY / RH_ERR_IO, with the same error table as rh_live_start; deletes a leftover dead socket. */
RH_API RhStatus rh_live_check_socket_free(const char *path);
RH_API RhStatus rh_live_start(RhKit *kit, const RhLiveOptions *opts, RhLiveServer **out);
/* Close order: delete the socket file it created (only the one with the same inode) → close the listener → queue one
 * bye{reason} for each client → wait at most 1.0 s per client and force-disconnect any that have not finished (they do
 * not get bye). reason NULL → "closed" (same as Python close()); pass "sdk exit" when a command-line program exits.
 * Does not close the kit. Idempotent. */
RH_API void     rh_live_stop(RhLiveServer *server, const char *reason);
RH_API uint32_t rh_live_client_count(const RhLiveServer *server);
RH_API RhStatus rh_live_stats(const RhLiveServer *server, RhLiveStats *out);
RH_API int32_t  rh_live_stats_json(const RhLiveServer *server, char *buf, size_t buf_len);  /* == stats() */
/* The absolute path actually used (normalized as a plain string against cwd; symlinks are not resolved). */
RH_API int32_t  rh_live_socket_path(const RhLiveServer *server, char *buf, size_t buf_len);
/* Same as Python default_socket_path(): the environment variable is returned **verbatim**, not made absolute. */
RH_API int32_t  rh_live_default_socket_path(char *buf, size_t buf_len);

/* ───────────────────────────── tracker (placeholder) ───────────────────────────── */

/* Scope of the first version: reading stored calibration + pose stream + world frame change events; no calibration is
 * run and no calibration file is written.
 * Held by another process → RH_ERR_BUSY (E1900). What follows is only a placeholder; the fields will follow the final
 * tracker section. */
typedef struct RhTrackerPose {
    uint32_t struct_size;
    uint32_t _reserved;
    char     role[16];                   /* "left" / "right" / … */
    char     serial[RH_SERIAL_MAX];
    double   p[3];                       /* m, tracker world frame */
    double   q[4];                       /* w, x, y, z */
    int64_t  sample_wall_ns;
    int64_t  sample_mono_ns;
    int64_t  recv_wall_ns;
} RhTrackerPose;

#ifdef __cplusplus
}  /* extern "C" */
#endif

#endif /* REVOHUMAN_H */
