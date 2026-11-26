/*
 * RevoHuman SDK — hand forward kinematics and nominal tracker→palm_link extrinsics (C API)
 *
 * Cross-checked against hand_fk of the Python package `revohuman`: model constants are bit-identical, and finite pose
 * values differ by at most 1e-12 in absolute terms.
 *
 * Kept in a separate header for now; it will be merged into revohuman.h when that is finalized (this file will then
 * contain only an #include).
 *
 * ── Conventions ─────────────────────────────────────────────────────────────
 * 1. Pose = (p in meters, r 3×3 row-major); r[3*i + j] is row i, column j; column j = direction of the link's j-th
 *    axis in the reference frame.
 * 2. All functions are pure: no locking, no allocation, safe to call concurrently from any thread; on failure only the
 *    thread-local last_error is written, and not a single byte of output.
 * 3. Handedness accepts only RH_HAND_LEFT / RH_HAND_RIGHT; RH_HAND_UNKNOWN and out-of-range values →
 *    RH_ERR_INVALID_ARG (code 1000).
 * 4. Non-finite joint angles count as NaN: the orientation of link i and all of its descendants is NaN, while the
 *    origin of link i still has a value. FK does not read joints_valid_mask; to propagate NaN for invalid joints, the
 *    caller sets the corresponding q to NaN first.
 * 5. rh_hand_fk / rh_hand_model_fk cover only the 21 child links of the joint table; for the 27 points including the
 *    fingertips and the palm_link origin see rh_hand_points (model-specific entry points only); for fingertip lengths
 *    see rh_hand_model_info. The OpenXR joint layout is not part of the SDK and is up to the caller.
 * 6. Models: this version carries two sets of model constants, RH_HAND_MODEL_DV1 (PTV1 glove) and RH_HAND_MODEL_P
 *    (P1 glove). The entry points without a model parameter (rh_hand_model_sha256 / rh_hand_link_name /
 *    rh_hand_tracker_mount / rh_hand_fk / rh_palm_from_tracker) always use RH_HAND_MODEL_DV1 in this version; for
 *    RH_HAND_MODEL_P use the rh_hand_model_* entry points that take an RhHandModel. The entry points without a model
 *    parameter will switch to RH_HAND_MODEL_P in the next version, so new code should use the explicit ones.
 * 7. The palm_link origin is the CAD assembly reference point of the URDF: on the back-of-hand layer, beyond the
 *    proximal end of the back plate; it is not on any joint or mesh, and it is not the anatomical palm base or wrist
 *    joint center. The origin of {hand}_tracker_link = the origin of the raw tracker pose (TrackerPose), on the axis of
 *    the 1/4" screw hole on the tracker's bottom face. The palm_link origin of both models is at the same place relative
 *    to the tracker (the P1 model file is already aligned to PTV1), so rh_hand_model_palm_from_tracker and point 26 of
 *    rh_hand_points are bit-identical for both models.
 */
#ifndef REVOHUMAN_FK_H
#define REVOHUMAN_FK_H

#include "revohuman.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Value type of array elements, frozen as POD and never extended, so it has no struct_size (the struct_size convention
 * of the header does not apply). */
typedef struct RhLinkPose {
    double p[3];   /* meters */
    double r[9];   /* 3×3 row-major, r[3*i+j]; column j = the link's j-th axis */
} RhLinkPose;

/* Model fingerprint, 64 lowercase hex digits, same as Python hand_fk.MODEL_SHA256. Static storage, do not free. */
RH_API const char *rh_hand_model_sha256(void);

/* Child link name of joint index (e.g. "left_index_MPR_Link"; the same for both models); return value convention as
 * for rh_joint_name: on success returns the required length (excluding NUL), and when buf_len > 0 always writes
 * NUL-terminated (possibly truncated) content; on failure returns a negative RhStatus. */
RH_API int32_t  rh_hand_link_name(RhHand hand, uint32_t index, char *buf, size_t buf_len);

/* Pose of the tracker link in the palm_link frame (constant, identity orientation). */
RH_API RhStatus rh_hand_tracker_mount(RhHand hand, RhLinkPose *out);

/* 21 joint angles (rad, positive URDF joint direction, order = joint table index) → poses of the 21 child links.
 * base NULL → palm_link frame; otherwise base ∘ result for each entry (base is not checked at all; non-finite values
 * propagate per IEEE). base may point into the out array: the implementation copies *base before writing out. */
RH_API RhStatus rh_hand_fk(RhHand hand, const double q_rad[RH_JOINT_COUNT],
                           const RhLinkPose *base, RhLinkPose out[RH_JOINT_COUNT]);

/* Raw tracker pose → nominal palm_link pose in the same reference frame (from the model's assembly relation, with no
 * per-glove calibration). p = RhTrackerPose.p, q_wxyz = RhTrackerPose.q (need not be normalized).
 * √(w²+x²+y²+z²) non-finite or < 1e-12 → all 12 numbers of the result are NaN; p is not checked, non-finite
 * components propagate per IEEE. */
RH_API RhStatus rh_palm_from_tracker(RhHand hand, const double p[3], const double q_wxyz[4],
                                     RhLinkPose *out);

/* out = base ∘ local (p = p_b + R_b·p_l, R = R_b·R_l); out may alias base or local. */
RH_API RhStatus rh_pose_compose(const RhLinkPose *base, const RhLinkPose *local, RhLinkPose *out);

/* ── Model-specific entry points ─────────────────────────────────────────────
 * Only the two values below are accepted as a model; 0 and any other value → RH_ERR_INVALID_ARG (code 1000), output
 * not written. RH_HAND_MODEL_DV1 = PTV1 glove, RH_HAND_MODEL_P = P1 glove. */
typedef int32_t RhHandModel;
enum { RH_HAND_MODEL_DV1 = 1, RH_HAND_MODEL_P = 2 };

/* Product model (RhProductIdentity.variant_code / RhHandReport.variant, "P1" / "PTV1") → hand geometry model
 * (RhHandModel). Fixed mapping that reads no hardware and does not depend on identify: P1 → RH_HAND_MODEL_P,
 * PTV1 → RH_HAND_MODEL_DV1. variant_code NULL, out NULL, or not recognized (including "auto", the empty string, and
 * any other string beyond case / whitespace differences) → RH_ERR_INVALID_ARG (code 1000), *out not written. There is
 * no "default model" fallback: an unrecognized code stays unrecognized and never falls back to RH_HAND_MODEL_DV1. */
RH_API RhStatus rh_variant_hand_model(const char *variant_code, RhHandModel *out);

/* Model metadata (output; set struct_size before the call, RH_STRUCT_INIT(RhHandModelInfo) recommended). */
typedef struct RhHandModelInfo {
    uint32_t    struct_size;
    RhHandModel model;
    char        name[16];            /* "dv1" / "p" */
    char        urdf_sha256[65];     /* 64 lowercase hex digits + NUL */
    uint32_t    points_version;      /* contract version of this model's point set (rh_hand_points) = RH_HAND_POINTS_VERSION; 0 = the
                                        library provides no point set */
    double      tip_length_m[5];     /* distance from fingertip TIP to the DIP link origin, meters; order index, middle, ring, little,
                                        thumb; the same for both hands */
    char        tip_source[48];      /* description of where the fingertip constants come from, e.g. "dip-mesh@21deef1b", NUL-terminated */
} RhHandModelInfo;

RH_API RhStatus rh_hand_model_info(RhHandModel model, RhHandModelInfo *out);

/* Same semantics as rh_hand_fk / rh_palm_from_tracker, except that the model constants are taken from model.
 * The entry points without a model parameter are equivalent to explicitly passing this version's default model
 * (convention 6), with bit-identical results. */
RH_API RhStatus rh_hand_model_fk(RhHandModel model, RhHand hand, const double q_rad[RH_JOINT_COUNT],
                                 const RhLinkPose *base, RhLinkPose out[RH_JOINT_COUNT]);
RH_API RhStatus rh_hand_model_palm_from_tracker(RhHandModel model, RhHand hand, const double p[3],
                                                const double q_wxyz[4], RhLinkPose *out);

/* ── Hand point set ──────────────────────────────────────────────────────────
 * Poses of 27 points in the {hand}_tracker_link frame. tracker_link is a fixed frame under palm_link in the model
 * ((t, I) in the palm_link frame, t = the same mount translation used by rh_hand_model_palm_from_tracker); computing
 * these points needs no tracker data at all. Point order:
 *   0–20   child link of joint i, same index as the output of rh_hand_model_fk
 *   21–25  fingertips: index, middle, ring, little, thumb ({hand}_{finger}_TIP, attached to that finger's DIP link)
 *   26     palm_link origin ({hand}_palm_link), always valid
 * Validity bits follow the topology, not the floating-point values of the result: joint i is valid ⇔ bit i of
 * joints_valid_mask is 1 and q_i is finite (only the low 21 bits are read).
 * Position of point i valid ⇔ all ancestor joints of i are valid (excluding i itself); orientation valid ⇔ i and all
 * ancestors are valid; a fingertip's position and orientation are both valid ⇔ the orientation of its DIP is valid.
 * An invalid quantity is written entirely as NaN (3 for position, 9 for orientation), so "mask bit is 1 ⇔ the quantity
 * is all finite" always holds. The point set has model-specific entry points only, no entry point without a model. */
#define RH_HAND_POINT_COUNT    27
#define RH_HAND_POINTS_VERSION 1   /* = Python hand_fk.POINTS_VERSION; +1 whenever point order, root, fingertip rule or validity
                                      rule changes */

/* Output; set struct_size before the call, RH_STRUCT_INIT(RhHandPoints) recommended. */
typedef struct RhHandPoints {
    uint32_t     struct_size;
    uint32_t     points_version;          /* = RH_HAND_POINTS_VERSION */
    RhHandModel  model;                   /* the model given by the caller */
    RhHand       hand;
    uint64_t     seq;                     /* copied from RhGloveFrame.seq; rh_hand_points writes 0 */
    uint32_t     generation;              /* copied from RhGloveFrame.generation; rh_hand_points writes 0 */
    uint32_t     position_valid_mask;     /* bit i: points[i].p valid; bits 27–31 always 0 */
    uint32_t     orientation_valid_mask;  /* bit i: points[i].r valid; bits 27–31 always 0 */
    RhFrameFlags frame_flags;             /* copied from RhGloveFrame.flags; rh_hand_points writes 0 */
    int64_t      sample_wall_ns;          /* copied from RhGloveFrame; valid if frame_flags & RH_FRAME_FLAG_HAS_SAMPLE_TIME */
    int64_t      sample_mono_ns;          /* same as above */
    RhLinkPose   points[RH_HAND_POINT_COUNT];
} RhHandPoints;

/* 21 joint angles (rad, order = joint table index) + joint validity bits → 27 points.
 * Unknown model or handedness, NULL pointer, or out->struct_size too small → RH_ERR_INVALID_ARG, output not written.
 * Pure function with no state across calls; q_rad may point into out (the implementation copies q before writing out). */
RH_API RhStatus rh_hand_points(RhHandModel model, RhHand hand, const double q_rad[RH_JOINT_COUNT],
                               uint32_t joints_valid_mask, RhHandPoints *out);

/* = rh_joints_to_radians(product_code, frame->joints_deg, q) followed by
 *   rh_hand_points(model, frame->hand, q, frame->joints_valid_mask, out), with bit-identical results;
 * then the frame's seq / generation / flags / sample_wall_ns / sample_mono_ns are copied as-is (not checked or changed).
 * It does not cache the previous frame, fill in frames or judge whether a frame is fresh; staleness is up to the caller,
 * based on seq and sampling time. frame->hand not LEFT / RIGHT (UNKNOWN when connected without a hand) →
 * RH_ERR_INVALID_ARG, output not written. */
RH_API RhStatus rh_hand_points_from_frame(RhHandModel model, const char *product_code,
                                          const RhGloveFrame *frame, RhHandPoints *out);

/* Name of point index (e.g. "left_index_TIP"; the same for both models, hence no model parameter). Return value
 * convention as for rh_hand_link_name; handedness not LEFT / RIGHT, or index ≥ RH_HAND_POINT_COUNT → negative
 * RhStatus. */
RH_API int32_t  rh_hand_point_name(RhHand hand, uint32_t index, char *buf, size_t buf_len);

/* ── Knuckle points ──────────────────────────────────────────────────────────
 * Poses of the nominal centers of the index, middle, ring and little finger MCP joints in the {hand}_tracker_link
 * frame; the thumb has none. Point order: 0–3 = index, middle, ring, little.
 * In rh_hand_points, the origins of that finger's MPR / MCP links (points 4k+3 / 4k+2) are on the back-of-hand
 * exoskeleton layer, not inside the finger; the knuckle point lies s_k meters toward the palm side along a fixed
 * direction d_k in the MPR link frame, and is rigidly attached to the MCP link, rotating with finger flexion:
 *   with P = point 4k+2 of rh_hand_points for the same inputs and c_k = s_k·d_k − (MCP joint origin in the MPR link
 *   frame), position = P.p + P.r·c_k and orientation = P.r.
 * d_k and s_k are nominal constants shared by both models (the Y of d_k is negated for the right hand), not per-glove
 * calibration values.
 * Validity: position and orientation of point k are both valid ⇔ the orientation bit of point 4k+2 of rh_hand_points
 * (both the MPR and MCP joints of that finger are valid); an invalid quantity is written entirely as NaN, so "mask bit
 * is 1 ⇔ the quantity is all finite" always holds. */
#define RH_HAND_KNUCKLE_COUNT    4
#define RH_HAND_KNUCKLES_VERSION 1   /* +1 whenever point order, position rule, constants or validity rule changes */

/* Output; set struct_size before the call, RH_STRUCT_INIT(RhHandKnuckles) recommended. */
typedef struct RhHandKnuckles {
    uint32_t     struct_size;
    uint32_t     knuckles_version;        /* = RH_HAND_KNUCKLES_VERSION */
    RhHandModel  model;                   /* the model given by the caller */
    RhHand       hand;
    uint32_t     position_valid_mask;     /* bit k: points[k].p valid; bits 4–31 always 0 */
    uint32_t     orientation_valid_mask;  /* bit k: points[k].r valid; always equal to the previous field; bits 4–31 always 0 */
    RhLinkPose   points[RH_HAND_KNUCKLE_COUNT];
} RhHandKnuckles;

/* Same inputs as rh_hand_points, with the same error order: unknown model or handedness, NULL pointer, or
 * out->struct_size too small → RH_ERR_INVALID_ARG, output not written. Pure function with no state across calls; q_rad
 * may point into out (the implementation copies q before writing out). */
RH_API RhStatus rh_hand_knuckles(RhHandModel model, RhHand hand, const double q_rad[RH_JOINT_COUNT],
                                 uint32_t joints_valid_mask, RhHandKnuckles *out);

/* ── Hand keypoints ──────────────────────────────────────────────────────────
 * Nominal positions of 22 human hand keypoints in the {hand}_tracker_link frame: joint centers and tips inside the
 * fingers, plus a wrist point and the palm center.
 * The points of rh_hand_points are exoskeleton link origins on the back-of-hand layer; keypoints are moved into the
 * fingers by nominal constants, as in rh_hand_knuckles, and are not per-glove calibration values. Only positions are
 * given: finger orientations depend on the consumer's skeleton convention, so the caller derives them from the link
 * orientations of rh_hand_points; in addition a palm frame palm_r is given (the palm-facing direction is a nominal fact
 * of the glove): Z is the palm normal pointing out of the palm surface, X points toward the middle finger, Y = Z × X.
 * The constants are tabulated per model; a model without a table → RH_ERR_UNSUPPORTED (code 1202).
 * Axis convention (met by every model; the SDK normalizes new models to it): −X of palm_link and of every finger link
 * points toward the back of the hand.
 * Point order (the common 21 hand keypoints, with the palm center last): 0 wrist; 1–4 thumb CMC / MCP / IP / TIP;
 *   5–8 index MCP / PIP / DIP / TIP; 9–12 middle; 13–16 ring; 17–20 little; 21 palm center.
 * With P = the point set of rh_hand_points for the same inputs, p_i = P.points[i].p, R_i = P.points[i].r,
 * x̂ = (1, 0, 0), and the Y of nominal directions negated for the right hand:
 *   wrist = p_26 + (R_26·x̂)·w; palm center = (R_26·x̂)·a (tracker_link origin along palm_link +X);
 *   MCP of the four fingers = the knuckle points of rh_hand_knuckles for the same inputs (bit-identical);
 *   PIP / DIP / TIP of finger k = p_{4k+1} / p_{4k} / p_{21+k} + (R_{4k+3}·e_k)·s_k (e_k in the MPR link frame);
 *   thumb CMC = p_20 + (R_19·x̂)·c; thumb MCP / IP / TIP = p_17 / p_16 / p_25 + (R_19·e_4)·s_4 (e_4 in the CMR link
 *   frame).
 * The wrist point is at the depth of the middle and ring finger MCPs; it is a nominal reference point, not the
 * anatomical wrist joint center.
 * Validity: a point is valid only if position and orientation of every P it uses are valid (wrist, palm center and
 * palm_r are always valid); all three components of an invalid point are written as NaN, so "mask bit is 1 ⇔ the
 * point is all finite" always holds. */
#define RH_HAND_KEYPOINT_COUNT    22
#define RH_HAND_KEYPOINTS_VERSION 1   /* +1 whenever point order, position rule, constants or validity rule changes */

/* Output; set struct_size before the call, RH_STRUCT_INIT(RhHandKeypoints) recommended. */
typedef struct RhHandKeypoints {
    uint32_t     struct_size;
    uint32_t     keypoints_version;       /* = RH_HAND_KEYPOINTS_VERSION */
    RhHandModel  model;                   /* the model given by the caller */
    RhHand       hand;
    uint32_t     valid_mask;              /* bit k: points[k] valid; bits 22–31 always 0 */
    uint32_t     reserved;                /* always 0 */
    double       points[RH_HAND_KEYPOINT_COUNT][3];
    double       palm_r[9];               /* orientation of the palm frame in tracker_link, row-major, columns = X / Y / Z axes */
} RhHandKeypoints;

/* Same inputs as rh_hand_points, with the same error order: unknown model or handedness, NULL pointer, or
 * out->struct_size too small → RH_ERR_INVALID_ARG; model without a keypoint table → RH_ERR_UNSUPPORTED; output not
 * written in either case. Pure function with no state across calls; q_rad may point into out (the implementation copies
 * q before writing out). */
RH_API RhStatus rh_hand_keypoints(RhHandModel model, RhHand hand, const double q_rad[RH_JOINT_COUNT],
                                  uint32_t joints_valid_mask, RhHandKeypoints *out);

/* Name of keypoint index (e.g. "left_index_pip", the same for both models). Return value convention as for
 * rh_hand_link_name; handedness not LEFT / RIGHT, or index ≥ RH_HAND_KEYPOINT_COUNT → negative RhStatus. */
RH_API int32_t  rh_hand_keypoint_name(RhHand hand, uint32_t index, char *buf, size_t buf_len);

/* ── Tactile point layout ────────────────────────────────────────────────────
 * Which point the i-th value of a tactile frame's RhTactile.values comes from: which link it is attached to, and its
 * position and outward normal in that link's frame.
 * With L = pose number taxels[i].link of the rh_hand_model_fk output, point = L.p + L.r·p and outward normal = L.r·n
 * (in the palm_link frame when base is NULL, in the base frame when base is given). The data is compiled into the
 * library; no files are read.
 * Only RH_HAND_MODEL_DV1 (PTV1 glove, TYPE_B, 241 points) has a layout; RH_HAND_MODEL_P (P1 glove) has no tactile.
 * Not per-unit calibration values: the positions are the taxel centers given by the hand CAD model (on the outer surface of
 * the finger covers) and the normals are the outward normals given by the same CAD model. Layout version 1 (earlier
 * releases) instead fitted the array onto the ring / cap meshes and took the normal from the nearest mesh point. */
#define RH_TAXEL_LAYOUT_VERSION 2   /* +1 whenever point order, position source or normal rule changes */

/* Value type of array elements, frozen as POD and never extended, no struct_size. */
typedef struct RhTaxel {
    double   p[3];       /* position in the link frame, meters */
    double   n[3];       /* outward normal in the link frame, unit vector */
    uint32_t link;       /* = output index of rh_hand_model_fk (0–20) */
    uint32_t reserved;   /* always 0 */
} RhTaxel;

/* Output; set struct_size before the call, RH_STRUCT_INIT(RhTaxelLayout) recommended. About 19 KB: do not put it as a
 * local variable on a thread with a small stack. */
typedef struct RhTaxelLayout {
    uint32_t    struct_size;
    uint32_t    layout_version;                  /* = RH_TAXEL_LAYOUT_VERSION */
    RhHandModel model;                           /* the model given by the caller */
    RhHand      hand;
    uint32_t    count;                           /* = the point_count given by the caller */
    char        source[48];                      /* description of the source, e.g. "cad-provided@1c093b3", NUL-terminated */
    RhTaxel     taxels[RH_TACTILE_MAX_POINTS];   /* index = RhTactile.values index; all zero after count */
} RhTaxelLayout;

/* Pass the tactile frame's RhTactile.point_count as point_count.
 * Unknown model or handedness, out NULL or struct_size too small, point_count 0 or > RH_TACTILE_MAX_POINTS
 *   → RH_ERR_INVALID_ARG;
 * the model has no tactile (RH_HAND_MODEL_P, P1 glove), or the model has no layout for this point count
 * (RH_HAND_MODEL_DV1, PTV1 glove: 241 points only) → RH_ERR_UNSUPPORTED (error code 1800).
 * Output not written on failure. Pure function, safe to call concurrently. */
RH_API RhStatus rh_hand_model_taxel_layout(RhHandModel model, RhHand hand, uint32_t point_count,
                                           RhTaxelLayout *out);

#ifdef __cplusplus
}
#endif

#endif /* REVOHUMAN_FK_H */
