// SPDX-License-Identifier: MIT
// Xtensa windowed ABI. Q registers are caller-saved by the S3 PIE ABI.
// Kernels do not touch QACC/ACCX. No calls while vector state is live.
#include <sdkconfig.h>
#ifdef CONFIG_IDF_TARGET_ESP32S3
    .section .iram1,"ax"
    .align 4
    .global hub75_s3_update8
    .type hub75_s3_update8,@function
// a2: aligned destination; a3: plane stride; a4: plane count (>0)
// a5: aligned R[8], G[8], B[8], pre-shifted; a6: r/g/b/keep masks
hub75_s3_update8:
    entry a1, 32
    ee.vld.128.ip q0, a5, 16
    ee.vld.128.ip q1, a5, 16
    ee.vld.128.ip q2, a5, 16
    mov a8, a6
    addi a7, a6, 2
    ee.vldbc.16.ip q4, a7, 2
    ee.vldbc.16.ip q5, a7, 2
    movi a10, 1
    wsr.sar a10
1:
    mov a9, a8
    ee.vldbc.16 q3, a9
    ee.andq q7, q0, q3
    ee.andq q6, q1, q4
    ee.orq q7, q7, q6
    ee.andq q6, q2, q5
    ee.orq q7, q7, q6
    mov a9, a7
    ee.vldbc.16 q3, a9
    ee.vld.128.ip q6, a2, 0
    ee.andq q6, q6, q3
    ee.orq q7, q7, q6
    ee.vst.128.xp q7, a2, a3
    // Over at most ten planes, spill between 16-bit lanes stays above bit 6.
    // Channel masks select only bits 0..5, so 32-bit shifts are sufficient.
    ee.vsr.32 q0, q0
    ee.vsr.32 q1, q1
    ee.vsr.32 q2, q2
    addi a4, a4, -1
    bnez a4, 1b
    retw.n
    .size hub75_s3_update8, .-hub75_s3_update8

    .align 4
    .global hub75_s3_mask_fill
    .type hub75_s3_mask_fill,@function
// a2: aligned destination; a3: number of eight-word blocks (>0)
// a4: keep mask and replacement pattern (two uint16_t values)
hub75_s3_mask_fill:
    entry a1, 32
    ee.vldbc.16.ip q1, a4, 2
    ee.vldbc.16 q2, a4
2:
    ee.vld.128.ip q0, a2, 0
    ee.andq q0, q0, q1
    ee.orq q0, q0, q2
    ee.vst.128.ip q0, a2, 16
    addi a3, a3, -1
    bnez a3, 2b
    retw.n
    .size hub75_s3_mask_fill, .-hub75_s3_mask_fill
#endif
