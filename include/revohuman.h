/*
 * RevoHuman SDK — C API
 *
 * v0.1.1。功能面对齐 Python 包 `revohuman` 0.1.1，
 * 本机广播与 Python 侧 `revohuman.live` schema 1 兼容，现有 `simulation/` 查看器不改一行即可接。
 * C 有意偏离 Python 的地方在对应字段旁注明（「C 偏离」「C 扩展」）。
 *
 * ── 约定 ────────────────────────────────────────────────────────────────────
 * 1. 返回值：多数函数返回 RhStatus（0 = 成功，负数 = 失败大类）。失败细节是**线程局部**的：
 *    `rh_last_error_code()` 给与 Python `revohuman.ErrorCode` 逐值相同的数字码（E1102 …），
 *    `rh_last_error()` 给与 Python `format_error()` 同格式的串（"E1102 PortBusyError: …"）。
 *    成功的调用不清除上一次的错误。
 * 2. 字符串出参：`(char *buf, size_t buf_len)`，返回所需长度（不含结尾 NUL）或负的 RhStatus；
 *    `buf_len > 0` 时总会写入以 NUL 结尾的（可能被截断的）内容；`buf_len == 0` 只问长度。
 * 3. 结构体版本：凡是独立跨越 API 的结构，首字段都是 `uint32_t struct_size`。
 *    - 选项结构：`rh_xxx_options_init(&opts)` 填 struct_size 与 Python 同名参数的默认值，再改要改的字段；
 *      选项指针传 NULL 等价于全默认。
 *    - 出参结构：调用前由调用方置 `struct_size = sizeof(结构)`，推荐写 `RhGloveInfo info = RH_STRUCT_INIT(RhGloveInfo);`。
 *      库只写前 struct_size 字节；struct_size 小于该结构首个发布版本的大小 → RH_ERR_INVALID_ARG。
 *    - 回调里收到的帧由库填 struct_size（= 库自己的 sizeof），调用方只读自己认得的字段即可。
 *    - 嵌在别的结构里的结构（RhSyncStamp、RhTactile、RhPairedFrame 的 left/right、RhProductIdentity 的
 *      left/right）在同一主版本内布局冻结；RhGloveFrame 尾部留了保留字给第二期。
 * 4. 句柄：`rh_*_connect / open / create / start` 产出，`rh_*_close / stop` 释放，均对 NULL 安全。
 *    `rh_pair_glove()` / `rh_kit_glove()` 等返回的是**借用**指针，随父句柄释放，不要单独关。
 *    关父句柄时，它下面还开着的订阅各收一次 RH_FRAME_END；订阅句柄本身仍要 `rh_sub_close()` 释放
 *    （父句柄关掉之后再调也安全）。`rh_shutdown()` 会强制关掉全部未释放的句柄，之后不要再碰任何句柄。
 * 5. 订阅：每个订阅一条有界队列（满了丢最旧）。
 *    - 回调模式：库为该订阅起一条线程顺序调用回调；回调里拿到的 frame 指针只在回调期间有效；
 *      不要在回调里阻塞太久（队列会满、丢最旧并收到 RH_FRAME_LAG）。
 *    - 拉取模式：回调传 NULL，用 `rh_sub_recv_*()` 按超时取，帧拷进调用方的结构。
 *    - 在本订阅**自己的**回调里调 `rh_sub_close()` 是允许的：立即返回、之后不再回调，句柄在回调返回后由库回收。
 *    - 在任何本库回调线程里关**父句柄**（rh_glove_close / rh_pair_close / rh_synchronizer_close /
 *      rh_kit_close / rh_live_stop / rh_shutdown）不执行：库记一条 ERROR 日志后直接返回。要从回调里触发关闭，
 *      请通知别的线程去关。
 * 6. 时间：`*_wall_ns` = CLOCK_REALTIME 纳秒，`*_mono_ns` = CLOCK_MONOTONIC 纳秒。
 *    `sample_*` 是设备采样刻映射到主机时轴的结果；`recv_wall_ns` 是主机收到该帧的时刻。
 *    同一帧的 `sample_wall_ns − sample_mono_ns` 等于映射那一刻的 `rh_wall_offset_ns()`；
 *    该偏移之后刷新（主机改时 / 时间同步步进）不回算已经发出的帧。两手共用同一个偏移。
 * 7. 本 API 不提供任何写设备序列号的入口（只读 `rh_glove_get_sn()`），也不提供私有入口。
 * 8. 运行期提示走日志，文本以 "N2xxx " 开头（与 Python notices 同码），上位机按前 5 个字符过滤。
 * 9. 设备节点目录：串口 / 相机节点默认在 /dev 下找；环境变量 REVOHUMAN_DEV_DIR 非空时改在它下面找
 *    （容器把宿主 /dev 挂到别处时用），每次扫描 / 定位时现读，不要在别的线程同时改它。
 *    显式给的 port 是 /dev/... 而它本身不存在、该目录下同名节点存在时，换成后者再连。
 *
 * 手别只认 MCU 序列号：C SDK 不读也不写手别注册表 glove_sets.yaml（Python SDK 的 register / set_name
 * 在 C 里没有对应物），序列号撞号的旧一代手套只能用 port / unit_id 定位。
 *
 * 第一期不含：编码器零位标定写入、腕部相机、MCAP 录制、固件升级、doctor、
 * 基站标定流程。tracker（只读已存标定 + 位姿流）单独成节，见文末。
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

/* 出参结构的初始化：`RhGloveStats st = RH_STRUCT_INIT(RhGloveStats);`（其余字段清零）。
 * C 用指定初始化器、C++ 用值初始化，都不会触发 -Wmissing-field-initializers。 */
#ifdef __cplusplus
#  define RH_STRUCT_INIT(T) ([]() { T rh_v_{}; rh_v_.struct_size = (uint32_t)sizeof(T); return rh_v_; }())
#else
#  define RH_STRUCT_INIT(T) { .struct_size = (uint32_t)sizeof(T) }
#endif

/* ───────────────────────────── 版本 ───────────────────────────── */

#define RH_API_VERSION_MAJOR 0
#define RH_API_VERSION_MINOR 1
#define RH_API_VERSION_PATCH 1

/* ───────────────────────────── 常量 ───────────────────────────── */

#define RH_JOINT_COUNT            21   /* 每手编码器关节数，顺序同 Python joint_map */
#define RH_TACTILE_MAX_POINTS     350  /* 协议允许的触觉点数上限（0 与 351 拒收） */
#define RH_TACTILE_POINTS_TYPE_A  211
#define RH_TACTILE_POINTS_TYPE_B  241
#define RH_STREAM_MIN_RATE_HZ     0.01
#define RH_STREAM_MAX_RATE_HZ     200.0

#define RH_PATH_MAX    256
#define RH_ID_MAX      64   /* 拓扑指纹、型号码、组名等 */
#define RH_SERIAL_MAX  32
#define RH_NOTE_MAX    256

/* ───────────────────────────── 状态码 ───────────────────────────── */

typedef int32_t RhStatus;
enum {
    RH_OK                   = 0,
    RH_ERR_INVALID_ARG      = -1,   /* 参数非法；此时一个请求字节都没发，码为 1000 */
    RH_ERR_NOT_INITIALIZED  = -2,
    RH_ERR_NOT_FOUND        = -3,   /* 没找到符合条件的设备 */
    RH_ERR_TIMEOUT          = -4,
    RH_ERR_PROTOCOL         = -5,   /* 线协议错误、设备异常应答、固件不兼容 */
    RH_ERR_DISCONNECTED     = -6,
    RH_ERR_BUFFER_TOO_SMALL = -7,
    RH_ERR_UNSUPPORTED      = -8,   /* 固件 / 型号 / 本期 C SDK 不支持 */
    RH_ERR_CANCELLED        = -9,
    RH_ERR_INVALID_DATA     = -10,  /* 配置 / 规则文件内容非法 */
    RH_ERR_BUSY             = -11,  /* 串口或 tracker 被别的进程占着；广播 socket 已有活的监听者 */
    RH_ERR_STATE            = -12,  /* 当前状态下不能做（已关闭、录制门关、同步已失去、证据不足…） */
    RH_ERR_CONFLICT         = -13,  /* 速率冲突、型号与声明不符 */
    RH_ERR_IO               = -14,  /* 串口 / 文件 / socket 读写失败 */
    RH_ERR_INTERNAL         = -99,
};

/* 与 Python `revohuman.ErrorCode` 逐值相同；`rh_last_error_code()` 返回其一。
 * 不在 RevoHuman 异常树上的原因（含参数非法、广播 socket 错误）一律 1000，不回 0。 */
typedef int32_t RhErrorCode;
enum {
    RH_E_OK                      = 0,
    RH_E_REVOHUMAN_ERROR         = 1000,
    RH_E_TRANSPORT_ERROR         = 1100,
    RH_E_DISCONNECTED            = 1101,
    RH_E_PORT_BUSY               = 1102,
    RH_E_PROTOCOL_ERROR          = 1200,
    RH_E_DEVICE_EXCEPTION        = 1201,  /* 手套应答异常 */
    RH_E_UNSUPPORTED_FEATURE     = 1202,
    RH_E_TIME_SYNC_UNSUPPORTED   = 1203,  /* 固件不支持时间同步功能 */
    RH_E_TIME_SYNC_INCOMPATIBLE  = 1204,  /* 固件的时间同步功能版本不兼容 */
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

/* ───────────────────────────── 枚举与位图 ───────────────────────────── */

typedef int32_t RhHand;             /* Python handedness；None ↔ UNKNOWN */
enum { RH_HAND_UNKNOWN = 0, RH_HAND_LEFT = 1, RH_HAND_RIGHT = 2 };

typedef int32_t RhHands;            /* open_kit(hands=…) */
enum { RH_HANDS_BOTH = 0, RH_HANDS_LEFT = 1, RH_HANDS_RIGHT = 2 };

/* 传感器位图：ProductVariant.sensors / HandReport.sensors / open_kit(sensors=…) */
typedef uint32_t RhSensorMask;
enum {
    RH_SENSOR_ENCODER      = 1u << 0,
    RH_SENSOR_TRACKER      = 1u << 1,
    RH_SENSOR_TACTILE      = 1u << 2,
    RH_SENSOR_WRIST_CAMERA = 1u << 3,
};

typedef int32_t RhTactileState;     /* variant.TactileState */
enum { RH_TACTILE_STATE_UNKNOWN = 0, RH_TACTILE_STATE_NONE = 1, RH_TACTILE_STATE_TACTILE = 2 };

typedef int32_t RhTactileUnit;      /* TactileData.unit：TYPE_B 且 241 点为 mN，其余 device_native */
enum { RH_TACTILE_UNIT_DEVICE_NATIVE = 0, RH_TACTILE_UNIT_MN = 1 };

typedef int32_t RhSyncCause;        /* notices.SyncCause；门开着时为 NONE */
enum {
    RH_CAUSE_NONE             = 0,
    RH_CAUSE_CLOCK_NOT_READY  = 1,
    RH_CAUSE_CLOCK_DEGRADED   = 2,
    RH_CAUSE_HOST_CLOCK_EVENT = 3,
    RH_CAUSE_DEVICE_RESET     = 4,
    RH_CAUSE_LINK_UNSTABLE    = 5,
};

/* codes.TrustState；NONE = 映射器尚无结论。DEGRADED = 精度略降但仍在允许范围内，照常出数据；
 * JSON（rh_glove_stats_json / rh_kit_status_json）里的取值串与 Python 相同。 */
typedef int32_t RhStampTrust;
enum { RH_TRUST_NONE = 0, RH_TRUST_TRUSTED = 1, RH_TRUST_DEGRADED = 2, RH_TRUST_UNTRUSTED = 3 };

/* button.ButtonMode。0 保留为「未知」，零初始化的结构不会被误读成 END。
 * 设备上报的原值是 END=0 / START=1，库内显式映射，不做数值强转。 */
typedef int32_t RhButtonMode;
enum { RH_BUTTON_UNKNOWN = 0, RH_BUTTON_START = 1, RH_BUTTON_END = 2 };

typedef int32_t RhPairMode;         /* BimanualSynchronizer(mode=…) */
enum { RH_PAIR_INTERPOLATE = 0, RH_PAIR_NEAREST = 1 };

typedef int32_t RhSyncPolicy;       /* connect_pair(sync=…) / open_kit(sync=…)：False / True / "best-effort" */
enum { RH_SYNC_OFF = 0, RH_SYNC_ON = 1, RH_SYNC_BEST_EFFORT = 2 };
/* 双手连接时做一次硬件同步（两只手套对齐到同一个 USB 帧，加入同一个同步会话）：
 *   RH_SYNC_ON（默认）：同步失败 → rh_pair_connect / rh_kit_open 返回错误；
 *   RH_SYNC_BEST_EFFORT：同步失败 → 告警 N2201 后退到主机时间域配对；
 *   RH_SYNC_OFF：不做硬件同步，主机时间域配对。
 * 起流后自检同步质量（结果见 rh_pair_connect_report），采样对齐超 resync_ms 时连接期内重建；
 * 运行中失步（N2202）后按 auto_resync 自动重建，也可手动 rh_pair_resync()。 */

typedef int32_t RhProductSource;    /* ProductIdentity.source；NONE 在广播里写 "" */
enum {
    RH_PRODUCT_SOURCE_NONE     = 0,
    RH_PRODUCT_SOURCE_SN       = 1,  /* "sn"：序列号规则 */
    RH_PRODUCT_SOURCE_HARDWARE = 2,  /* "hardware"：硬件证据 */
    RH_PRODUCT_SOURCE_DECLARED = 3,  /* "declared"：调用方声明 */
};

typedef int32_t RhFrameKind;        /* 订阅回调的 kind */
enum {
    RH_FRAME_OK    = 0,  /* frame 有效 */
    RH_FRAME_LAG   = 1,  /* 队列溢出丢了最旧的帧；frame 为 NULL，丢弃数见 rh_sub_dropped() */
    RH_FRAME_END   = 2,  /* 上游关闭，订阅结束；frame 为 NULL，之后不再回调 */
    RH_FRAME_ERROR = 3,  /* 上游致命错误；frame 为 NULL，回调线程内 rh_last_error*() 可读细节；之后不再回调 */
};

/* RhGloveFrame.flags */
typedef uint32_t RhFrameFlags;
enum {
    RH_FRAME_FLAG_GATE_OPEN       = 1u << 0,  /* 派发时录制门开着（门控订阅只收到这种） */
    RH_FRAME_FLAG_HAS_SAMPLE_TIME = 1u << 1,  /* 0 = sample_*_ns 无效（只会出现在免门订阅）。
                                                 广播是免门订阅，此时 glove 的 sample_ns 发 null */
    RH_FRAME_FLAG_HAS_TACTILE     = 1u << 2,  /* 0 = 本手型号不带触觉或这一帧没有触觉 */
};

/* RhSyncStamp.flags：帧内同步段的标志位原值，16 位，未知位原样保留。
 * 注意与按键扩展段的 8 位 flags 不是一回事，不共用类型。 */
typedef uint16_t RhSyncFlags;
enum {
    RH_SYNC_FLAG_BUS_CLOCK   = 0x01,  /* 设备看得到主机 USB 帧时钟 */
    RH_SYNC_FLAG_BUS_LOCKED  = 0x02,  /* 采样时钟已跟上主机 USB 帧时钟 */
    RH_SYNC_FLAG_IN_SESSION  = 0x04,  /* 已加入双手同步会话 */
    RH_SYNC_FLAG_HOLD        = 0x08,  /* 暂时跟丢 USB 帧时钟，按最近的速率保持 */
    RH_SYNC_FLAG_STAMP_VALID = 0x10,  /* 本帧的同步戳有效 */
};

typedef int32_t RhLogLevel;
enum { RH_LOG_DEBUG = 10, RH_LOG_INFO = 20, RH_LOG_WARNING = 30, RH_LOG_ERROR = 40, RH_LOG_OFF = 100 };

/* ───────────────────────────── 不透明句柄 ───────────────────────────── */

typedef struct RhScanList     RhScanList;
typedef struct RhGlove        RhGlove;
typedef struct RhGlovePair    RhGlovePair;
typedef struct RhSynchronizer RhSynchronizer;
typedef struct RhSubscription RhSubscription;
typedef struct RhKit          RhKit;
typedef struct RhLiveServer   RhLiveServer;
typedef struct RhTracker      RhTracker;

/* ───────────────────────────── 生命周期与错误 ───────────────────────────── */

typedef struct RhInitOptions {
    uint32_t   struct_size;
    RhLogLevel log_level;            /* 默认 RH_LOG_WARNING */
} RhInitOptions;

RH_API void        rh_init_options_init(RhInitOptions *opts);
RH_API RhStatus    rh_init(const RhInitOptions *opts);   /* 幂等 */
RH_API void        rh_shutdown(void);                    /* 未释放的句柄会被强制关闭并告警 */
RH_API const char *rh_version(void);                     /* 静态串，如 "0.1.1" */

/* 上一个错误的 E 码（与 Python error_code() 同规则）。 */
RH_API RhErrorCode rh_last_error_code(void);
/* "E1200 StreamOptionsRejected: …"：类名段是**实际**的异常类名，可能是没有专属码的子类
 * （StreamOptionsRejected、ButtonExtensionMismatch、LiveSocketBusy …），与 Python type(exc).__name__ 相同。 */
RH_API int32_t     rh_last_error(char *buf, size_t buf_len);
/* 上一个错误是手套应答异常（E1201）时返回设备回的异常码（0x01 非法功能、0x02 非法地址…），
 * 否则返回 0。 */
RH_API int32_t     rh_last_error_exception_code(void);
/* 按码查表得到**该码的根类名**（1200 → "ProtocolError"，未知码返回 NULL）。
 * 它给不出子类名：要看实际异常类，读 rh_last_error()。 */
RH_API const char *rh_error_class_name(RhErrorCode code);

typedef void (*RhLogCallback)(RhLogLevel level, const char *message, void *user_data);
RH_API void rh_set_log_level(RhLogLevel level);
/* 默认写 stderr；设回调后只走回调。回调可能来自任意内部线程。 */
RH_API void rh_set_log_callback(RhLogCallback cb, void *user_data);

/* ───────────────────────────── 时间 ───────────────────────────── */

/* 进程级偏移：wall = mono + offset（夹心采样，主机改时 / 时间同步步进后刷新）。 */
RH_API int64_t rh_wall_offset_ns(void);

/* ───────────────────────────── 帧结构 ───────────────────────────── */

/* 帧内同步段（Python SyncStamp）。
 * 配对键 (generation, cycle_sequence) 以 in_session 为准：in_session 时有效，hold 期间仍有效；
 * 「键有效、未同步」是 hold 下的正常态。不要拿 synchronized 当键是否有效的判据。 */
typedef struct RhSyncStamp {
    uint8_t     present;          /* 帧带同步段；0 = 设备漏发（Python sync is None），属协议异常 */
    uint8_t     in_session;       /* generation≠0 且 IN_SESSION */
    uint8_t     synchronized;     /* generation≠0 且 BUS_CLOCK|BUS_LOCKED|IN_SESSION|STAMP_VALID 全置且非 HOLD */
    uint8_t     hold;             /* HOLD 位：区分「会话中暂时跟丢时钟」与「从未加入会话」 */
    uint8_t     bus_locked;       /* BUS_LOCKED 且非 HOLD（没加入会话的手套也可能为 1） */
    uint8_t     _reserved;
    RhSyncFlags flags;            /* 原值，未知位保留 */
    uint32_t    generation;       /* 同步会话代号；0 = 从未加入会话 */
    uint32_t    cycle_sequence;
    uint16_t    frame_number;     /* 采样时刻的 USB 帧号，mod 2048 */
    uint16_t    _reserved2;
} RhSyncStamp;

/* Python TactileData。values 只有前 point_count 个有效，是设备原始计数，不是力值。 */
typedef struct RhTactile {
    uint16_t      point_count;
    uint16_t      age_ms;
    uint8_t       valid;
    uint8_t       link_ok;
    uint8_t       calibrated;          /* unit == MN 时为 1；TYPE_A 恒 0 */
    uint8_t       has_sample_time;     /* 0 = sample_*_ns 无效（映射未就绪）；与帧级标志无关 */
    RhTactileUnit unit;
    uint32_t      device_tick;         /* 触觉自己的采样 tick（ms）；实时流恒有 */
    int64_t       sample_wall_ns;      /* 触觉自己的采样刻；跨模态毫秒级分析用它，不用帧的 */
    int64_t       sample_mono_ns;
    uint16_t      values[RH_TACTILE_MAX_POINTS];
} RhTactile;

/* Python GloveFrame。 */
typedef struct RhGloveFrame {
    uint32_t     struct_size;
    RhHand       hand;                    /* 裸连时 UNKNOWN */
    uint64_t     seq;                     /* C 扩展：会话内按有符号差展开（设备倒退则随之倒退，不保证单调）；低 32 位 = Python seq（线上 u32 原值） */
    uint32_t     generation;              /* 每次（重）起流 +1；变了 = 重连过或做过维护窗 */
    RhFrameFlags flags;
    uint32_t     device_tick;             /* 设备**发送** tick（ms，uint32 回绕） */
    uint32_t     joints_valid_mask;       /* bit i = joints_deg[i] 有效 */
    int64_t      recv_wall_ns;            /* 主机收到时刻 */
    int64_t      sample_wall_ns;          /* 编码器采样刻（门控订阅下恒有效） */
    int64_t      sample_mono_ns;          /* 同一时刻的 MONOTONIC 表示，与 sample_wall_ns 按同一个偏移一起算出 */
    uint32_t     skipped_snapshots;       /* 设备侧跳过的快照数（线上原值） */
    uint32_t     _reserved0;
    uint64_t     dropped_local;           /* **流级**累计的主机侧丢弃数；订阅级见 rh_sub_dropped() */
    double       joints_deg[RH_JOINT_COUNT]; /* 固件修正角（度），不做映射；映射见 rh_joints_to_radians */
    RhSyncStamp  sync;
    RhTactile    tactile;                 /* flags & HAS_TACTILE 时有效 */
    uint64_t     _reserved_tail[4];
} RhGloveFrame;

/* Python PairedFrame。以左手为参考轴：
 *   skew_ns = 左帧时刻到最近一个**真实**右采样的带符号距离（右晚为正）；
 *             插值合成的右帧不算采样点，所以 interpolate 档下它≠ right.sample_wall_ns − left.sample_wall_ns。
 *             按 (generation, cycle_sequence) 精确配上的对子恒 0。
 *   interpolate 档合成右帧的 sample_mono_ns 由 C 定义（Python 帧无此字段），不在黄金向量覆盖内。 */
typedef struct RhPairedFrame {
    uint32_t     struct_size;
    uint32_t     _reserved;
    int64_t      skew_ns;
    RhGloveFrame left;
    RhGloveFrame right;
} RhPairedFrame;

typedef struct RhButtonState {
    uint32_t     struct_size;
    RhButtonMode mode;
    uint8_t      pressed;
    uint8_t      has_sample_time;
    uint16_t     _reserved;
    uint32_t     press_sequence;
    uint32_t     generation;
    uint32_t     _reserved2;
    uint64_t     seq;                    /* 同 RhGloveFrame.seq：C 扩展，低 32 位 = Python seq */
    int64_t      recv_wall_ns;
    int64_t      sample_wall_ns;
} RhButtonState;

/* Python ButtonEvent；漏看的按压数 = press_count − 1。 */
typedef struct RhButtonEvent {
    uint32_t     struct_size;
    RhButtonMode mode;                   /* 按压之后的状态：START = 开始录制，END = 停止 */
    uint32_t     press_sequence;
    uint32_t     press_count;
    uint32_t     generation;
    uint8_t      has_sample_time;
    uint8_t      _reserved[3];
    uint64_t     seq;                    /* 同 RhGloveFrame.seq */
    int64_t      recv_wall_ns;
    int64_t      sample_wall_ns;
} RhButtonEvent;

typedef void (*RhGloveFrameCallback)(RhFrameKind kind, const RhGloveFrame *frame, void *user_data);
typedef void (*RhPairedFrameCallback)(RhFrameKind kind, const RhPairedFrame *frame, void *user_data);
typedef void (*RhButtonEventCallback)(RhFrameKind kind, const RhButtonEvent *event, void *user_data);

/* ───────────────────────────── 发现 ───────────────────────────── */

/* Python DiscoveredGlove。probe=0 时 fw_version 及其后字段为空 / 0。 */
typedef struct RhDiscoveredGlove {
    uint32_t struct_size;
    RhHand   hand;                       /* 由 MCU 序列号定；撞号 / 认不出 = UNKNOWN（原因见 hand_note） */
    char     port[RH_PATH_MAX];          /* 空串 = sysfs 有但节点不可用（不存在 / 重新插拔前的旧节点，约定 9） */
    char     unit_id[RH_ID_MAX];         /* USB 拓扑指纹，身份主键（Python usb_topology_id） */
    uint8_t  registered;                 /* 手别已定（= hand 不为 UNKNOWN） */
    uint8_t  probed;                     /* 1 = 本条确实短连读过设备信息（被占的口为 0） */
    uint8_t  _reserved[2];
    char     usb_serial[RH_SERIAL_MAX];
    char     hand_source[16];            /* serial / none */
    char     hand_note[RH_NOTE_MAX];
    char     tracker_serial[RH_SERIAL_MAX];
    char     fw_version[RH_SERIAL_MAX];
    char     sn[RH_SERIAL_MAX];
    char     product_type[RH_ID_MAX];    /* UNKNOWN / ENCODER_ONLY / ENCODER_TACTILE */
    uint16_t encoder_count;
    uint16_t tactile_point_count;
    char     tactile_kind[16];           /* "TYPE_A" / "TYPE_B"；厂商未知或读不到为空 */
} RhDiscoveredGlove;

typedef struct RhScanOptions {
    uint32_t    struct_size;
    uint8_t     probe;                   /* 默认 0；1 = 短连补齐固件信息。
                                            C 偏离：被别的进程占着的口跳过不读、不报错（Python 不做 flock 直接连） */
    uint8_t     _reserved[3];
    const char *sysfs_root;              /* NULL / "" → /sys/bus/usb/devices（测试用） */
} RhScanOptions;

RH_API void     rh_scan_options_init(RhScanOptions *opts);
RH_API RhStatus rh_scan(const RhScanOptions *opts, RhScanList **out);
RH_API size_t   rh_scan_count(const RhScanList *list);
RH_API RhStatus rh_scan_get(const RhScanList *list, size_t index, RhDiscoveredGlove *out);
RH_API void     rh_scan_free(RhScanList *list);

/* ───────────────────────────── 单手 ───────────────────────────── */

/* 定位方式三选一（hand / port / unit_id），全空 = 全机恰好一只时裸连。
 * hand 按 MCU 序列号找；序列号撞号 / 认不出 → RH_ERR_NOT_FOUND（E1400），改用 port / unit_id。
 * port 按约定 9 先换到 REVOHUMAN_DEV_DIR 下（设了且条件成立时）。 */
typedef struct RhGloveTarget {
    RhHand      hand;
    const char *port;
    const char *unit_id;
} RhGloveTarget;

typedef struct RhGloveOptions {
    uint32_t    struct_size;
    double      rate_hz;                 /* 默认 200；速率属于连接，订阅不能改 */
    uint8_t     require_tactile;         /* 默认 0；1 = 型号不带或判不出触觉即拒连 */
    uint8_t     button;                  /* 默认 0；1 = 启用按键上报 */
    uint8_t     _reserved[2];
    uint32_t    sync_loss_debounce;      /* C 扩展（Python 固定 5）：默认 5；连续几帧不在当前同步会话代上才判丢；< 1 按 1 */
} RhGloveOptions;

RH_API void     rh_glove_options_init(RhGloveOptions *opts);
/* 先独占探测串口（被占 → RH_ERR_BUSY / E1102，错误串带持有者 pid），再短连读设备信息
 * （尽力而为，最多 3 次，失败不阻塞起流；顺手停掉上次残留的流），然后起流。
 * 返回时流已在起，但第一帧可能还没到。 */
RH_API RhStatus rh_glove_connect(const RhGloveTarget *target, const RhGloveOptions *opts, RhGlove **out);
RH_API void     rh_glove_close(RhGlove *glove);          /* 停流、关串口、结束全部订阅（各收一次 END） */

/* Python Glove 的静态信息（connect 时缓存；rh_glove_selftest 之后会刷新，可能变）。 */
typedef struct RhGloveInfo {
    uint32_t       struct_size;
    RhHand         hand;
    char           port[RH_PATH_MAX];
    char           unit_id[RH_ID_MAX];
    char           usb_serial[RH_SERIAL_MAX];
    char           sn[RH_SERIAL_MAX];              /* 固件 SN，只读；未写入时为空串 */
    char           fw_version[RH_SERIAL_MAX];
    char           product_type[RH_ID_MAX];
    RhTactileState tactile_state;
    RhTactileUnit  tactile_unit;                   /* 起流前就能问，与帧上同一条规则 */
    uint16_t       encoder_count;
    uint16_t       tactile_point_count;            /* 不带触觉为 0 */
    uint16_t       tactile_wire_version;
    uint8_t        button_enabled;
    int8_t         button_supported;               /* 1 / 0 / -1 = 还不知道（还没开过流） */
    char           tactile_kind[16];               /* 同 RhDiscoveredGlove.tactile_kind */
} RhGloveInfo;

RH_API RhStatus rh_glove_info(const RhGlove *glove, RhGloveInfo *out);
RH_API int32_t  rh_glove_get_sn(const RhGlove *glove, char *buf, size_t buf_len);
/* 实际速率（起流后由设备回显填入）；还没起好返回 0。 */
RH_API double   rh_glove_effective_rate_hz(const RhGlove *glove);

typedef struct RhSubscribeOptions {
    uint32_t struct_size;
    uint32_t queue_size;                 /* 默认 64（按键 16、配对 64） */
    uint8_t  ungated;                    /* 默认 0；1 = 门关时也收（只能看，不能拿来对齐），对应 gated=False */
    uint8_t  _reserved[3];
} RhSubscribeOptions;

RH_API void     rh_subscribe_options_init(RhSubscribeOptions *opts);
/* 已致命 → 原致命错误；已关闭 → RH_ERR_DISCONNECTED（E1101）。 */
RH_API RhStatus rh_glove_subscribe(RhGlove *glove, const RhSubscribeOptions *opts,
                                   RhGloveFrameCallback cb, void *user_data, RhSubscription **out);

/* 录制门（无条件，没有关掉它的开关）。 */
RH_API bool         rh_glove_capture_ok(const RhGlove *glove);
RH_API RhSyncCause  rh_glove_capture_cause(const RhGlove *glove);
RH_API RhStampTrust rh_glove_stamp_trust(const RhGlove *glove);

/* 门的开关沿（Python on_capture_gate）；在内部线程调用，不要在里面关手套。传 NULL 取消。 */
typedef void (*RhCaptureGateCallback)(RhGlove *glove, bool ok, RhSyncCause cause, void *user_data);
RH_API RhStatus rh_glove_on_capture_gate(RhGlove *glove, RhCaptureGateCallback cb, void *user_data);

typedef void (*RhStampTrustCallback)(RhGlove *glove, RhStampTrust state, RhStampTrust prev,
                                     RhSyncCause cause, void *user_data);
RH_API RhStatus rh_glove_on_stamp_trust(RhGlove *glove, RhStampTrustCallback cb, void *user_data);

/* 流已不可恢复时返回其 E 码（RH_E_OK = 还活着），细节进 last_error。 */
RH_API RhErrorCode rh_glove_fatal_error(const RhGlove *glove);

/* Python stats() 的常用计数；完整字典（与 to_dict() 同键）走 rh_glove_stats_json()。 */
typedef struct RhGloveStats {
    uint32_t     struct_size;
    uint32_t     reconnects;             /* 只数停顿后的重连；连接 / 起流失败的退避重试不计 */
    uint64_t     frames_received;
    uint64_t     frames_dropped_local;
    uint64_t     device_skipped_total;
    uint64_t     seq_gap_frames;
    uint64_t     frames_blocked;         /* 被录制门拦下的帧 */
    uint64_t     frames_ungated;         /* 其中派给了免门订阅的 */
    uint32_t     generation;             /* 当前流对象成功 START 的次数；维护窗换新流对象后从 1 重数，
                                            与帧上一直递增的 generation 口径不同 */
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

/* 体检：不改设备配置，但会发停流 / 开流命令，不是严格无副作用。**录制中不要调用。**
 * 在维护窗里执行：停流 → 普通连接读设备信息、编码器链路 → 重开流，期间约 50 ms 没有新帧
 * （双手配对流同样空约 50 ms）。窗内订阅保持打开、不发 END；generation 在派发线程拿到新流第一帧时 +1，
 * 不是本函数返回那一刻。会刷新 rh_glove_info() 的设备信息缓存。不能在库回调线程里调用（RH_ERR_STATE）。 */
typedef struct RhSelftestReport {
    uint32_t struct_size;
    uint8_t  ok;
    uint8_t  _reserved[3];
    uint32_t encoder_offline_mask;       /* bit i = 编码器 i 离线 */
    uint32_t _reserved2;
    char     problems[RH_NOTE_MAX * 2];  /* 多条以 '\n' 分隔；ok 时为空 */
} RhSelftestReport;

RH_API RhStatus rh_glove_selftest(RhGlove *glove, RhSelftestReport *out);

/* 按键：需 connect 时 button=1，否则 RH_ERR_UNSUPPORTED（E1202）。
 * 固件不支持时照常连接，info.button_supported = 0，日志出 N2406。 */
RH_API RhStatus rh_glove_button_read(RhGlove *glove, RhButtonState *out);   /* 还没收到过 → RH_ERR_NOT_FOUND */
RH_API RhStatus rh_glove_button_subscribe(RhGlove *glove, const RhSubscribeOptions *opts,
                                          RhButtonEventCallback cb, void *user_data, RhSubscription **out);

/* ───────────────────────────── 订阅（通用） ───────────────────────────── */

/* 拉取模式取一条；类型须与订阅来源一致，否则 RH_ERR_INVALID_ARG。
 * 超时 → RH_ERR_TIMEOUT；上游已关 → RH_ERR_DISCONNECTED（E1101）；timeout_ms < 0 = 一直等。 */
RH_API RhStatus rh_sub_recv_glove(RhSubscription *sub, int32_t timeout_ms, RhGloveFrame *out);
RH_API RhStatus rh_sub_recv_paired(RhSubscription *sub, int32_t timeout_ms, RhPairedFrame *out);
RH_API RhStatus rh_sub_recv_button(RhSubscription *sub, int32_t timeout_ms, RhButtonEvent *out);
RH_API uint64_t rh_sub_dropped(const RhSubscription *sub);     /* 本订阅因队列满丢掉的累计条数 */
/* 在别的线程调：等回调线程退出后返回。在本订阅自己的回调里调：立即返回，回调返回后回收。 */
RH_API void     rh_sub_close(RhSubscription *sub);

/* ───────────────────────────── 双手 ───────────────────────────── */

typedef struct RhPairOptions {
    uint32_t       struct_size;
    RhSyncPolicy   sync;                 /* 默认 ON：硬件同步，失败即报错；BEST_EFFORT 失败退主机配对 */
    RhGloveTarget  left;                 /* 只填 port / unit_id；全空 = 按手别找 */
    RhGloveTarget  right;
    RhGloveOptions glove;                /* 两手共用 */
    uint8_t        auto_resync;          /* 默认 1：失去同步后自动重建 */
    uint8_t        _reserved[3];
    uint32_t       lead_frames;          /* 默认 100（合法 20..1024） */
    uint32_t       sync_attempts;        /* 默认 3 */
    uint32_t       resync_attempts;      /* 默认 3 */
    uint32_t       auto_resync_attempts; /* 默认 5 */
    uint32_t       _reserved2;
    double         sync_timeout_s;       /* 默认 1.0 */
    double         arrival_check_s;      /* 默认 1.0 */
    double         arrival_warn_ms;      /* 默认 5.0 */
    double         align_warn_ms;        /* 默认 1.0 */
    double         align_fail_ms;        /* 默认 2.5 */
    double         time_base_timeout_s;  /* 默认 1.0：起流后等两手建立时间基准的上限 */
    double         resync_ms;            /* 默认 0.5 */
} RhPairOptions;

/* connect_pair 时的两项一次性自检（Python ArrivalCheck / AlignmentCheck 的公开面）。 */
typedef struct RhPairConnectReport {
    uint32_t struct_size;
    uint8_t  synchronized;
    uint8_t  arrival_ok;
    uint8_t  align_measurable;
    uint8_t  align_ok;
    double   arrival_diff_ms;            /* 同键到达差中位数 */
    double   align_diff_ms;              /* 同键采样刻差中位数 */
    char     reason[RH_NOTE_MAX];        /* 不 ok 时的原因 */
} RhPairConnectReport;

RH_API void     rh_pair_options_init(RhPairOptions *opts);
RH_API RhStatus rh_pair_connect(const RhPairOptions *opts, RhGlovePair **out);
RH_API void     rh_pair_close(RhGlovePair *pair);
RH_API RhGlove *rh_pair_glove(RhGlovePair *pair, RhHand hand);          /* 借用 */
RH_API bool     rh_pair_synchronized(const RhGlovePair *pair);
RH_API RhStatus rh_pair_connect_report(const RhGlovePair *pair, RhPairConnectReport *out);
/* 手动重建双手同步（停流约数百 ms，订阅保持打开）。失败返回 RH_ERR_STATE；strict=1 时同步建立失败为 E1700。
 * 不能在库回调线程里调用（RH_ERR_STATE）；不是 rh_pair_connect 建的 → RH_ERR_UNSUPPORTED。 */
RH_API RhStatus rh_pair_resync(RhGlovePair *pair, bool strict);

typedef void (*RhSyncLostCallback)(RhGlovePair *pair, const char *reason, void *user_data);
typedef void (*RhResyncCallback)(RhGlovePair *pair, void *user_data);
/* 挂的时候已经丢过同步，会立刻补触发一次（同 Python）。 */
RH_API RhStatus rh_pair_on_sync_lost(RhGlovePair *pair, RhSyncLostCallback cb, void *user_data);
RH_API RhStatus rh_pair_on_resync(RhGlovePair *pair, RhResyncCallback cb, void *user_data);

typedef struct RhSynchronizerOptions {
    uint32_t   struct_size;
    RhPairMode mode;                     /* 默认 INTERPOLATE */
    int64_t    max_skew_ns;              /* 默认 50 000 000 */
    double     pair_accuracy_ms;         /* 默认 1.0；NaN 也取默认；0.0 = 只出硬件同步对；负数 = 不设出口精度门 */
    int32_t    tactile_gate;             /* -1 = auto（默认）/ 0 关 / 1 开 */
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
    double   device_skew_p50_ms;         /* 还没结算为 NaN */
} RhSynchronizerStats;

RH_API void     rh_synchronizer_options_init(RhSynchronizerOptions *opts);
/* 由配对创建（时间域自动用 device 档，并自动接 resync）。 */
RH_API RhStatus rh_pair_synchronizer(RhGlovePair *pair, const RhSynchronizerOptions *opts, RhSynchronizer **out);
/* 由任意两只手套创建（没走 connect_pair 时退 host 档）。 */
RH_API RhStatus rh_synchronizer_create(RhGlove *left, RhGlove *right, const RhSynchronizerOptions *opts,
                                       RhSynchronizer **out);
RH_API RhStatus rh_synchronizer_subscribe(RhSynchronizer *sync, const RhSubscribeOptions *opts,
                                          RhPairedFrameCallback cb, void *user_data, RhSubscription **out);
RH_API bool     rh_synchronizer_strict_sync_lost(const RhSynchronizer *sync);
RH_API RhStatus rh_synchronizer_stats(const RhSynchronizer *sync, RhSynchronizerStats *out);
RH_API int32_t  rh_synchronizer_stats_json(const RhSynchronizer *sync, char *buf, size_t buf_len);
RH_API void     rh_synchronizer_close(RhSynchronizer *sync);

/* ───────────────────────────── 型号识别 ───────────────────────────── */

/* Python HandReport 的定长字段；claims / notes 等列表只在 JSON 里。嵌在 RhProductIdentity 里，布局冻结。
 * complete / missing 不单列：由 variant_sensors 与 requested 那几只手的 sensors 算出（与 to_dict 同口径）。 */
typedef struct RhHandReport {
    RhHand         hand;
    uint8_t        present;
    uint8_t        conflict;
    uint8_t        requested;            /* 这只手在 opts.hands 里；为 0 时其余字段全零，JSON 的 hands 里也没有它 */
    uint8_t        tactile_valid;        /* 0 ↔ Python tactile 为 None（没探测、也没给提示），JSON 写 null */
    RhSensorMask   sensors;              /* 这只手在位的传感器 */
    RhTactileState tactile;
    char           port[RH_PATH_MAX];
    char           unit_id[RH_ID_MAX];
    char           usb_serial[RH_SERIAL_MAX];
    char           hand_source[16];      /* serial / none */
    char           firmware_sn[RH_SERIAL_MAX];
    char           tracker_serial[RH_SERIAL_MAX];
    char           tracker_source[16];   /* serial（按序列号定到的单元 HUB 上实挂的那只）/ 空 */
    char           camera_usb_id[16];
    char           camera_node[RH_PATH_MAX];
    char           tactile_note[RH_NOTE_MAX];
    char           sku_rule[RH_ID_MAX];  /* 命中的序列号规则 match 串，没命中为空（规则加载时已拒超长，见 rules_path） */
    char           variant[16];          /* 这只手单独判出的型号码，判不出为空 */
} RhHandReport;

typedef struct RhProductIdentity {
    uint32_t        struct_size;
    RhProductSource source;
    char            variant_code[16];    /* "P1" / "PTV1"；判不出为空 */
    char            variant_name[RH_ID_MAX];
    RhSensorMask    variant_sensors;
    uint32_t        _reserved;
    RhHandReport    left;
    RhHandReport    right;
} RhProductIdentity;

/* RhIdentifyOptions.tactile_given 的位 */
enum { RH_IDENTIFY_TACTILE_LEFT = 1u << 0, RH_IDENTIFY_TACTILE_RIGHT = 1u << 1 };

typedef struct RhIdentifyOptions {
    uint32_t       struct_size;
    uint8_t        probe;                /* 默认 1：短连读固件 SN 与触觉信息 */
    uint8_t        tactile_given;        /* 默认 0。置位的手不探测，直接采用 tactile_left / tactile_right
                                            （UNKNOWN 也照收，同 Python `hand in tactile`）；其它位 → RH_ERR_INVALID_ARG */
    uint8_t        _reserved[2];
    RhHands        hands;                /* 默认 RH_HANDS_BOTH；Python 的列表写法（定序 / 空列表）C 不提供 */
    RhTactileState tactile_left;         /* 仅 tactile_given 置位时读；越界值 → RH_ERR_INVALID_ARG */
    RhTactileState tactile_right;
    uint32_t       _reserved2;
    const char    *product;              /* NULL 等价 "auto"：按证据判；"P1" / "PTV1" = 声明。去首尾空白、不分大小写；
                                            未知型号码 → RH_ERR_INVALID_ARG */
    const char    *sysfs_root;
    const char    *rules_path;           /* NULL → $REVOHUMAN_PRODUCTS → ~/.config/revohuman/products.yaml
                                            （文件不存在则只用内置规则；存在时用户规则排在内置之前）；
                                            规则里的正则超出共同子集，或 match 串长于 RH_ID_MAX-1 字节 → RH_ERR_INVALID_DATA */
} RhIdentifyOptions;

RH_API void     rh_identify_options_init(RhIdentifyOptions *opts);
RH_API RhStatus rh_identify(const RhIdentifyOptions *opts, RhProductIdentity *out);
RH_API int32_t  rh_identify_json(const RhIdentifyOptions *opts, char *buf, size_t buf_len);  /* == to_dict() */
/* 型号码 → 传感器位图（未知码返回 0）。 */
RH_API RhSensorMask rh_variant_sensors(const char *code);

/* ───────────────────────────── 关节映射 ───────────────────────────── */

typedef struct RhJointSpec {
    uint32_t struct_size;
    uint32_t index;
    double   sign;                       /* +1.0 / −1.0；0.0 = 该路置零弃用 */
    double   zero_deg;
    double   offset_rad;
    char     name[RH_ID_MAX];            /* 不带手别前缀，如 "index_DIP_joint" */
    char     finger[16];                 /* index / middle / ring / little / thumb */
    char     joint[16];                  /* DIP / PIP / MCP / MPR / CMR / CMP */
} RhJointSpec;

/* 映射表版本，如 "0.2"（< 1.0 表示官方映射仍待确认）。 */
RH_API const char *rh_joint_map_version(void);
/* product_code 为 NULL 或未知 → 退回 "default" 表（同 Python joint_specs，不报错）。 */
RH_API RhStatus rh_joint_spec(const char *product_code, uint32_t index, RhJointSpec *out);
/* "<left|right>_<name>"，即广播 hello.joints 里的名字；hand = UNKNOWN 时不加前缀。 */
RH_API int32_t  rh_joint_name(RhHand hand, uint32_t index, char *buf, size_t buf_len);
/* q = sign·radians(wrap180(deg − zero)) + offset，逐位同 Python：NaN 进 NaN 出，±inf 得 NaN，
 * wrap180 边界上可能取到 +180（不钳位）。 */
RH_API RhStatus rh_joints_to_radians(const char *product_code, const double deg[RH_JOINT_COUNT],
                                     double rad[RH_JOINT_COUNT]);

/* ───────────────────────────── 整套（kit） ───────────────────────────── */

typedef struct RhKitOptions {
    uint32_t       struct_size;
    RhHands        hands;                /* 默认 BOTH */
    const char    *product;              /* 默认 "auto"；"P1" / "PTV1" = 声明，与在位硬件相反即拒 */
    RhSensorMask   sensors;              /* 0 = 型号应有、且本期 C SDK 支持的全部；要了不支持的 → UNSUPPORTED */
    uint8_t        strict;               /* 默认 1：要了但在位缺 → E1804 */
    uint8_t        _reserved[3];
    RhSyncPolicy   sync;                 /* 默认 ON */
    uint32_t       _reserved2;
    double         rate_hz;              /* 默认 200 */
    const char    *sysfs_root;
    const char    *rules_path;
    RhGloveOptions glove;
    const char    *tracker_config_json;  /* 预留，本版本不读（C SDK 不含 tracker：sensors 要 TRACKER → UNSUPPORTED） */
} RhKitOptions;

typedef void (*RhKitFaultCallback)(RhKit *kit, RhErrorCode code, bool recoverable, void *user_data);

RH_API void          rh_kit_options_init(RhKitOptions *opts);
RH_API RhStatus      rh_kit_open(const RhKitOptions *opts, RhKit **out);
RH_API void          rh_kit_close(RhKit *kit);
RH_API RhStatus      rh_kit_identity(const RhKit *kit, RhProductIdentity *out);
RH_API RhSensorMask  rh_kit_sensors(const RhKit *kit);                 /* 实际启用的 */
RH_API RhGlove      *rh_kit_glove(RhKit *kit, RhHand hand);             /* 借用；没开这只手为 NULL */
RH_API RhGlovePair  *rh_kit_pair(RhKit *kit);                           /* 借用；单手为 NULL */
RH_API RhTracker    *rh_kit_tracker(RhKit *kit);                        /* 借用；没开为 NULL */
RH_API RhStatus      rh_kit_synchronizer(RhKit *kit, const RhSynchronizerOptions *opts, RhSynchronizer **out);
RH_API RhErrorCode   rh_kit_fatal(const RhKit *kit);                    /* RH_E_OK = 无锁存故障 */
RH_API RhStatus      rh_kit_on_fault(RhKit *kit, RhKitFaultCallback cb, void *user_data);
RH_API int32_t       rh_kit_status_json(const RhKit *kit, char *buf, size_t buf_len);  /* == kit.status() */

/* ───────────────────────────── 本机广播 ───────────────────────────── */

/* 与 Python `revohuman live` 同一线协议（schema 1）：AF_UNIX 流 socket，4 字节大端长度 + 紧凑 UTF-8 JSON；
 * 新连接先收 hello 再收最新 status；每客户端独立发送线程 + 有界队列（满丢最旧），
 * 单条消息整次写完超过 send_timeout_s 即断开该客户端；stop 时广播 bye。现有 simulation/ 查看器直接可连。
 *
 * socket 路径解析（服务端）：显式 socket_path → $REVOHUMAN_LIVE_SOCKET（非空）→
 * $XDG_RUNTIME_DIR/revohuman/live.sock（该变量本身是目录且可写可进入）→ /tmp/revohuman-<uid>/live.sock。
 * 前两级是「非自有路径」：所在目录必须已存在，库不建、不查属主、不改权限；后两级是「自有路径」：
 * 目录按 0700 建，必须属于当前用户且不是符号链接。socket 文件 0600。绝对路径 ≥ 104 字节报错。
 *
 * 错误：不在 RevoHuman 异常树上，rh_last_error_code() 一律 1000。
 *   RH_ERR_BUSY（类名 LiveSocketBusy）：路径上有活的监听者；探测连接超时（1.0 s）或得到
 *     ECONNREFUSED / ENOENT 以外的错误；检测与 bind 之间被抢（EADDRINUSE）。这些情况都**不删**文件。
 *   RH_ERR_IO（类名 LiveSocketError）：路径上是非 socket 文件；非自有目录不存在；自有目录属主不符
 *     或是符号链接；bind 其它错误。
 *   RH_ERR_INVALID_ARG（类名 LiveSocketError）：路径过长。
 *   探测得 ECONNREFUSED / ENOENT 的是上次残留的死 socket，会被删掉后继续。 */

typedef int32_t RhLiveTrackerSource;
enum {
    RH_LIVE_TRACKER_KIT      = 0,  /* 默认：用 rh_kit_tracker(kit)，没有就不广播位姿 */
    RH_LIVE_TRACKER_EXTERNAL = 1,  /* 用 tracker 字段给的会话（借用，须比广播活得久）；
                                      本期 C SDK 未实现，rh_live_start 一律 RH_ERR_UNSUPPORTED，
                                      不读 tracker 字段（tracker 模块另行落地） */
    RH_LIVE_TRACKER_NONE     = 2,  /* 不广播位姿，hello.tracker.reason 写 tracker_reason */
};

typedef struct RhLiveOptions {
    uint32_t            struct_size;
    uint32_t            queue_size;      /* 默认 256（每客户端） */
    const char         *socket_path;     /* NULL = 按上面的顺序解析 */
    RhLiveTrackerSource tracker_source;  /* 默认 KIT */
    uint32_t            backlog;         /* 默认 8 */
    RhTracker          *tracker;         /* 仅 tracker_source == EXTERNAL 时用 */
    const char         *tracker_reason;  /* NULL / "" = 按 Python 兜底：型号带 tracker → "tracker not opened"，
                                            否则 "none (product has no tracker)"。原因文本照 Python 原文发 */
    const char         *tracker_gate;    /* NULL → hello.tracker.gate 为 null；否则 "flag" / "error" */
    double              send_timeout_s;  /* 默认 2.0 */
    double              pose_poll_hz;    /* 默认 250；本期收下但不起 pose_loop 线程，不生效（tracker 模块另行落地） */
    double              status_period_s; /* 默认 1.0 */
} RhLiveOptions;

/* Python Broadcaster.stats()。 */
typedef struct RhLiveStats {
    uint32_t struct_size;
    uint32_t clients;                    /* 当前订阅方数 */
    uint64_t published;                  /* 已编码发出的消息条数（hello 不计） */
    uint64_t poses;                      /* 已广播的 pose 条数；本期恒为 0（不起 pose_loop 线程，tracker 模块另行落地） */
    uint64_t dropped;                    /* **当前在连**订阅方因队列满丢掉的条数之和（已断开的不计，同 Python） */
    uint64_t frames[2];                  /* [0] 左 / [1] 右：已广播的 glove 帧数；没开这只手为 0 */
    uint8_t  hand_open[2];               /* 这只手在广播里 */
    uint8_t  _reserved[6];
} RhLiveStats;

RH_API void     rh_live_options_init(RhLiveOptions *opts);
/* 只做占用检测（bind 前那一步），供 CLI / 示例在打开手套之前先查。path 为 NULL 时按默认顺序解析。
 * 返回 RH_OK / RH_ERR_BUSY / RH_ERR_IO，与 rh_live_start 同一张错误表；会删掉残留的死 socket。 */
RH_API RhStatus rh_live_check_socket_free(const char *path);
RH_API RhStatus rh_live_start(RhKit *kit, const RhLiveOptions *opts, RhLiveServer **out);
/* 关闭顺序：先删自己建的 socket 文件（只删 inode 相同的那个）→ 关监听 → 给每个客户端排一条 bye{reason}
 * → 每个客户端最多等 1.0 s，发不完的强断（收不到 bye）。reason 为 NULL → "closed"（同 Python close()）；
 * 命令行程序退出时传 "sdk exit"。不关 kit。幂等。 */
RH_API void     rh_live_stop(RhLiveServer *server, const char *reason);
RH_API uint32_t rh_live_client_count(const RhLiveServer *server);
RH_API RhStatus rh_live_stats(const RhLiveServer *server, RhLiveStats *out);
RH_API int32_t  rh_live_stats_json(const RhLiveServer *server, char *buf, size_t buf_len);  /* == stats() */
/* 实际使用的绝对路径（按 cwd 做纯字符串规整，不解析符号链接）。 */
RH_API int32_t  rh_live_socket_path(const RhLiveServer *server, char *buf, size_t buf_len);
/* 同 Python default_socket_path()：环境变量**原样**返回、不绝对化。 */
RH_API int32_t  rh_live_default_socket_path(char *buf, size_t buf_len);

/* ───────────────────────────── tracker（占位） ───────────────────────────── */

/* 第一期范围：读已存标定 + 位姿流 + 世界系变化事件；不执行标定、不写任何标定文件。
 * 被别的进程占着 → RH_ERR_BUSY（E1900）。以下仅占位，字段以 tracker 节定稿为准。 */
typedef struct RhTrackerPose {
    uint32_t struct_size;
    uint32_t _reserved;
    char     role[16];                   /* "left" / "right" / … */
    char     serial[RH_SERIAL_MAX];
    double   p[3];                       /* m，tracker 世界系 */
    double   q[4];                       /* w, x, y, z */
    int64_t  sample_wall_ns;
    int64_t  sample_mono_ns;
    int64_t  recv_wall_ns;
} RhTrackerPose;

#ifdef __cplusplus
}  /* extern "C" */
#endif

#endif /* REVOHUMAN_H */
