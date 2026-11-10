/**
 * @file vtest.h
 * @brief Minimal verification framework: checks are attributed to the
 *        verification case (SVC-...) that is currently running.
 */
#ifndef VTEST_H
#define VTEST_H

#include <stdbool.h>

#define CHECK(cond) \
    vt_check((cond) ? true : false, __FILE__, __LINE__, #cond)
#define CHECK_NEAR(a, b, tol) \
    vt_check_near((double)(a), (double)(b), (double)(tol), __FILE__, __LINE__, #a)

void vt_check(bool ok, const char *file, int line, const char *expr);
void vt_check_near(double a, double b, double tol, const char *file, int line, const char *expr);

/** Record a measured value as evidence for the current case (printf-like). */
void vt_note(const char *fmt, ...)
#if defined(__GNUC__)
    __attribute__((format(printf, 1, 2)))
#endif
    ;

/* ---- verification case implementations (one function per SVC) ---- */
#define SVC_FN(name) void name(void)

/* control */
SVC_FN(svc_ctl_001); SVC_FN(svc_ctl_002); SVC_FN(svc_ctl_003); SVC_FN(svc_ctl_004);
SVC_FN(svc_ctl_005); SVC_FN(svc_ctl_006); SVC_FN(svc_ctl_007); SVC_FN(svc_ctl_008);
SVC_FN(svc_ctl_009); SVC_FN(svc_ctl_010); SVC_FN(svc_ctl_011);
/* pedal */
SVC_FN(svc_ped_001); SVC_FN(svc_ped_002); SVC_FN(svc_ped_003); SVC_FN(svc_ped_004);
SVC_FN(svc_ped_005);
/* torque */
SVC_FN(svc_tq_001); SVC_FN(svc_tq_002); SVC_FN(svc_tq_003); SVC_FN(svc_tq_004);
SVC_FN(svc_tq_005); SVC_FN(svc_tq_006); SVC_FN(svc_tq_007); SVC_FN(svc_tq_008);
SVC_FN(svc_tq_009); SVC_FN(svc_tq_010); SVC_FN(svc_tq_011);
/* vehicle state machine */
SVC_FN(svc_vsm_001); SVC_FN(svc_vsm_002); SVC_FN(svc_vsm_003); SVC_FN(svc_vsm_004);
SVC_FN(svc_vsm_005); SVC_FN(svc_vsm_006); SVC_FN(svc_vsm_007); SVC_FN(svc_vsm_008);
SVC_FN(svc_vsm_009); SVC_FN(svc_vsm_010);
/* diagnostics */
SVC_FN(svc_diag_001); SVC_FN(svc_diag_002); SVC_FN(svc_diag_003); SVC_FN(svc_diag_004);
SVC_FN(svc_diag_005); SVC_FN(svc_diag_006); SVC_FN(svc_diag_007); SVC_FN(svc_diag_008);
SVC_FN(svc_diag_009); SVC_FN(svc_diag_010); SVC_FN(svc_diag_011); SVC_FN(svc_diag_012);
SVC_FN(svc_diag_013);
/* safe state / supervision */
SVC_FN(svc_safe_001); SVC_FN(svc_safe_002); SVC_FN(svc_safe_003); SVC_FN(svc_safe_004);
SVC_FN(svc_safe_005);
/* communication */
SVC_FN(svc_com_001); SVC_FN(svc_com_002); SVC_FN(svc_com_003); SVC_FN(svc_com_004);

#endif /* VTEST_H */
