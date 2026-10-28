/*
 * RevoHuman SDK — 手部正运动学与 tracker→掌根名义外参（C API）
 *
 * 与 Python 包 `revohuman` 的 hand_fk 对拍：模型常量逐位相同，位姿有限值绝对差 ≤ 1e-12。
 *
 * 暂放独立头文件，revohuman.h 定稿时并入（届时本文件只剩一行 #include）。
 *
 * ── 约定 ────────────────────────────────────────────────────────────────────
 * 1. 位姿 = (p 米, r 3×3 行主序)，r[3*i + j] 是第 i 行第 j 列；第 j 列 = 该 link 第 j 根轴在参考系中的方向。
 * 2. 全部是纯函数：不加锁、不分配，可在任意线程并发调用；失败只写线程局部 last_error，输出一个字节都不写。
 * 3. 手别只认 RH_HAND_LEFT / RH_HAND_RIGHT；RH_HAND_UNKNOWN 与越界值 → RH_ERR_INVALID_ARG（码 1000）。
 * 4. 非有限关节角按 NaN 算：第 i 个 link 的姿态与全部后代为 NaN，第 i 个 link 的原点照常有值。
 *    FK 不读 joints_valid_mask；要让无效关节传播 NaN，调用方先把对应 q 置 NaN。
 * 5. rh_hand_fk / rh_hand_model_fk 只覆盖关节表里 21 个子 link；带指尖与掌根的 27 点见 rh_hand_points
 *    （只有指定型号入口），指尖长度见 rh_hand_model_info。OpenXR 关节布局不在 SDK 里，归调用方。
 * 6. 型号：本期带 DV1 与 P 两套模型常量。不带型号参数的入口（rh_hand_model_sha256 / rh_hand_link_name /
 *    rh_hand_tracker_mount / rh_hand_fk / rh_palm_from_tracker）本期固定按 DV1 算；要用 P 只能走带
 *    RhHandModel 参数的 rh_hand_model_* 入口。不带型号参数的入口下一期改指 P，新代码请直接用显式入口。
 */
#ifndef REVOHUMAN_FK_H
#define REVOHUMAN_FK_H

#include "revohuman.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 数组元素的值类型，冻结为 POD、永不扩展，所以不带 struct_size（不适用头文件约定 struct_size 那条）。 */
typedef struct RhLinkPose {
    double p[3];   /* 米 */
    double r[9];   /* 3×3 行主序，r[3*i+j]；第 j 列 = 该 link 第 j 根轴 */
} RhLinkPose;

/* 模型指纹，64 位小写十六进制，同 Python hand_fk.MODEL_SHA256。静态存储，不必释放。 */
RH_API const char *rh_hand_model_sha256(void);

/* 第 index 个关节的子 link 名（如 "left_index_MPR_Link"；两个型号相同）；返回值约定同 rh_joint_name：
 * 成功返回所需长度（不含 NUL），buf_len > 0 时总写入以 NUL 结尾的（可能截断的）内容；失败返回负的 RhStatus。 */
RH_API int32_t  rh_hand_link_name(RhHand hand, uint32_t index, char *buf, size_t buf_len);

/* tracker link 在掌根系中的位姿（常量，姿态为单位阵）。 */
RH_API RhStatus rh_hand_tracker_mount(RhHand hand, RhLinkPose *out);

/* 21 个关节角（rad，URDF 关节正方向，顺序 = 关节表下标）→ 21 个子 link 的位姿。
 * base 为 NULL → 掌根系；否则逐条 base ∘ 结果（base 不做任何检查，非有限值按 IEEE 传播）。
 * base 可以指向 out 数组内部：实现先拷走 *base 再写 out。 */
RH_API RhStatus rh_hand_fk(RhHand hand, const double q_rad[RH_JOINT_COUNT],
                           const RhLinkPose *base, RhLinkPose out[RH_JOINT_COUNT]);

/* tracker 原始位姿 → 掌根在同一参考系中的名义位姿（按模型装配关系，没有逐只手套标定）。
 * p = RhTrackerPose.p，q_wxyz = RhTrackerPose.q（不必归一）。
 * √(w²+x²+y²+z²) 非有限或 < 1e-12 → 结果 12 个数全为 NaN；p 不检查，非有限分量按 IEEE 传播。 */
RH_API RhStatus rh_palm_from_tracker(RhHand hand, const double p[3], const double q_wxyz[4],
                                     RhLinkPose *out);

/* out = base ∘ local（p = p_b + R_b·p_l，R = R_b·R_l）；out 可与 base 或 local 同址。 */
RH_API RhStatus rh_pose_compose(const RhLinkPose *base, const RhLinkPose *local, RhLinkPose *out);

/* ── 指定型号的入口 ──────────────────────────────────────────────────────────
 * 型号只认下面两个值；0 与其它值 → RH_ERR_INVALID_ARG（码 1000），输出不写。 */
typedef int32_t RhHandModel;
enum { RH_HAND_MODEL_DV1 = 1, RH_HAND_MODEL_P = 2 };

/* 型号元数据（出参，调用前置 struct_size，推荐 RH_STRUCT_INIT(RhHandModelInfo)）。 */
typedef struct RhHandModelInfo {
    uint32_t    struct_size;
    RhHandModel model;
    char        name[16];            /* "dv1" / "p" */
    char        urdf_sha256[65];     /* 64 位小写十六进制 + NUL */
    uint32_t    points_version;      /* 该型号点集（rh_hand_points）的契约版本 = RH_HAND_POINTS_VERSION；0 = 库未提供点集 */
    double      tip_length_m[5];     /* 指尖 TIP 到 DIP link 原点的距离，米；顺序食、中、无名、小、拇；两只手相同 */
    char        tip_source[48];      /* 指尖常量的来源说明，如 "cap-mesh@4e7da46"，NUL 结尾 */
} RhHandModelInfo;

RH_API RhStatus rh_hand_model_info(RhHandModel model, RhHandModelInfo *out);

/* 语义与 rh_hand_fk / rh_palm_from_tracker 完全相同，只是模型常量按 model 取。
 * 旧入口等价于显式给本期默认型号（约定 6），结果逐位相同。 */
RH_API RhStatus rh_hand_model_fk(RhHandModel model, RhHand hand, const double q_rad[RH_JOINT_COUNT],
                                 const RhLinkPose *base, RhLinkPose out[RH_JOINT_COUNT]);
RH_API RhStatus rh_hand_model_palm_from_tracker(RhHandModel model, RhHand hand, const double p[3],
                                                const double q_wxyz[4], RhLinkPose *out);

/* ── 手内点集 ───────────────────────────────────────────────────────────────
 * 27 个点在 {hand}_tracker_link 系中的位姿。tracker_link 是模型里挂在掌根下的固定坐标系（掌根系里是 (t, I)，
 * t = rh_hand_model_palm_from_tracker 用的同一个挂载平移），算这些点用不到任何 tracker 数据。点序：
 *   0–20   关节 i 的子 link，下标与 rh_hand_model_fk 的输出相同
 *   21–25  指尖，食、中、无名、小、拇（{hand}_{finger}_TIP，挂在该指 DIP link 上）
 *   26     掌根原点（{hand}_palm_link），恒有效
 * 有效位按拓扑推、不看结果的浮点值：关节 i 有效 ⇔ joints_valid_mask 第 i 位为 1 且 q_i 有限（只读低 21 位）。
 * 点 i 的位置有效 ⇔ i 的全部祖先关节有效（不含 i 自己）；朝向有效 ⇔ i 与全部祖先都有效；
 * 指尖的位置与朝向都 ⇔ 它那根 DIP 的朝向有效。无效的量整个写 NaN（位置 3 个、朝向 9 个），
 * 所以「掩码位为 1 ⇔ 该量全有限」恒成立。点集只有指定型号入口，没有旧入口。 */
#define RH_HAND_POINT_COUNT    27
#define RH_HAND_POINTS_VERSION 1   /* = Python hand_fk.POINTS_VERSION；点序、根、指尖规则、有效性规则任一项变就加一 */

/* 出参，调用前置 struct_size，推荐 RH_STRUCT_INIT(RhHandPoints)。 */
typedef struct RhHandPoints {
    uint32_t     struct_size;
    uint32_t     points_version;          /* = RH_HAND_POINTS_VERSION */
    RhHandModel  model;                   /* 调用方给的型号 */
    RhHand       hand;
    uint64_t     seq;                     /* 抄自 RhGloveFrame.seq；rh_hand_points 写 0 */
    uint32_t     generation;              /* 抄自 RhGloveFrame.generation；rh_hand_points 写 0 */
    uint32_t     position_valid_mask;     /* bit i：points[i].p 有效；bit 27–31 恒 0 */
    uint32_t     orientation_valid_mask;  /* bit i：points[i].r 有效；bit 27–31 恒 0 */
    RhFrameFlags frame_flags;             /* 抄自 RhGloveFrame.flags；rh_hand_points 写 0 */
    int64_t      sample_wall_ns;          /* 抄自 RhGloveFrame；有效与否看 frame_flags & RH_FRAME_FLAG_HAS_SAMPLE_TIME */
    int64_t      sample_mono_ns;          /* 同上 */
    RhLinkPose   points[RH_HAND_POINT_COUNT];
} RhHandPoints;

/* 21 个关节角（rad，顺序 = 关节表下标）+ 关节有效位 → 27 点。
 * 型号、手别不认识或指针为 NULL、out->struct_size 太小 → RH_ERR_INVALID_ARG，不写输出。
 * 纯函数、无跨调用状态；q_rad 可以指向 out 内部（实现先拷走 q 再写 out）。 */
RH_API RhStatus rh_hand_points(RhHandModel model, RhHand hand, const double q_rad[RH_JOINT_COUNT],
                               uint32_t joints_valid_mask, RhHandPoints *out);

/* = rh_joints_to_radians(product_code, frame->joints_deg, q) 再
 *   rh_hand_points(model, frame->hand, q, frame->joints_valid_mask, out)，结果逐位相同；
 * 然后原样抄帧的 seq / generation / flags / sample_wall_ns / sample_mono_ns（不判不改）。
 * 不缓存上一帧、不补帧、不判帧新旧，过期与否由调用方按 seq 与采样时间判。
 * frame->hand 不是 LEFT / RIGHT（裸连时为 UNKNOWN）→ RH_ERR_INVALID_ARG，不写输出。 */
RH_API RhStatus rh_hand_points_from_frame(RhHandModel model, const char *product_code,
                                          const RhGloveFrame *frame, RhHandPoints *out);

/* 第 index 个点的名字（如 "left_index_TIP"；两个型号相同，故不带型号）。返回值约定同 rh_hand_link_name；
 * 手别不是 LEFT / RIGHT、index ≥ RH_HAND_POINT_COUNT → 负的 RhStatus。 */
RH_API int32_t  rh_hand_point_name(RhHand hand, uint32_t index, char *buf, size_t buf_len);

#ifdef __cplusplus
}
#endif

#endif /* REVOHUMAN_FK_H */
