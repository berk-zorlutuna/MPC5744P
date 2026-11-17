/**
 * @file vtest.c
 * @brief Verification case registry, runner and results report generator.
 *
 * Usage:
 *   tests                          run all executable cases
 *   tests --report FILE [--rev R]  also write the Markdown results report
 */
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "vtest.h"

typedef enum {
    VM_TEST = 0,   /* unit test on host */
    VM_SIM,        /* closed-loop simulation on host */
    VM_INTEG,      /* software integration test on host (complete application on BSP stub) */
    VM_ANALYSIS,   /* automated analysis/inspection on host */
    VM_REVIEW,     /* manual review / static analysis - not automated */
    VM_BENCH       /* test on target hardware - not automated here */
} vmethod_t;

static const char *const s_method_name[] = {
    "Unit test", "Simulation", "Integration test", "Analysis", "Review", "Bench test"
};

typedef struct {
    const char *id;
    const char *title;
    const char *verifies;
    vmethod_t method;
    void (*fn)(void);
} vcase_t;

static void svc_gen_001(void);
static void svc_gen_002(void);

/* clang-format off */
static const vcase_t s_cases[] = {
    { "SVC-GEN-001", "Host build with all warnings as errors", "SWR-GEN-001", VM_ANALYSIS, svc_gen_001 },
    { "SVC-GEN-002", "Hardware access confined to the BSP", "SWR-GEN-002", VM_ANALYSIS, svc_gen_002 },
    { "SVC-REV-001", "Code review: sampling instant and 1.5-sample delay compensation", "SWR-CTL-009", VM_REVIEW, NULL },
    { "SVC-REV-002", "MISRA C:2012 static analysis", "SWR-GEN-003", VM_REVIEW, NULL },

    { "SVC-CTL-001", "Clarke/Park transforms", "SWR-CTL-001", VM_TEST, svc_ctl_001 },
    { "SVC-CTL-002", "Space vector modulation", "SWR-CTL-002", VM_TEST, svc_ctl_002 },
    { "SVC-CTL-003", "PI controller anti-windup", "SWR-CTL-004", VM_TEST, svc_ctl_003 },
    { "SVC-CTL-004", "MTPA current split", "SWR-CTL-003", VM_TEST, svc_ctl_004 },
    { "SVC-CTL-005", "Torque production and current loop dynamics", "SWR-CTL-003, SWR-CTL-004, SWR-CTL-005", VM_SIM, svc_ctl_005 },
    { "SVC-CTL-006", "Regenerative torque at low speed", "SWR-CTL-003, SWR-CTL-004", VM_SIM, svc_ctl_006 },
    { "SVC-CTL-007", "Field weakening up to maximum speed", "SWR-CTL-006, SWR-CTL-009", VM_SIM, svc_ctl_007 },
    { "SVC-CTL-008", "Voltage-limited regeneration at maximum speed", "SWR-CTL-006, SWR-CTL-007", VM_SIM, svc_ctl_008 },
    { "SVC-CTL-009", "Over-demand beyond voltage-limited capability", "SWR-CTL-005, SWR-CTL-007", VM_SIM, svc_ctl_009 },
    { "SVC-CTL-010", "Inverter enable on a spinning machine", "SWR-CTL-008, SWR-CTL-009", VM_SIM, svc_ctl_010 },
    { "SVC-CTL-011", "Speed estimation", "SWR-CTL-010", VM_TEST, svc_ctl_011 },

    { "SVC-PED-001", "Pedal normal operation", "SWR-PED-001", VM_TEST, svc_ped_001 },
    { "SVC-PED-002", "Pedal range check", "SWR-PED-002", VM_TEST, svc_ped_002 },
    { "SVC-PED-003", "Pedal track plausibility", "SWR-PED-003", VM_TEST, svc_ped_003 },
    { "SVC-PED-004", "Pedal single-track degraded mode", "SWR-PED-004", VM_TEST, svc_ped_004 },
    { "SVC-PED-005", "Pedal faults in the running application", "SWR-PED-004, SWR-PED-005, SWR-TQ-011", VM_INTEG, svc_ped_005 },

    { "SVC-TQ-001", "Drive torque envelope", "SWR-TQ-001", VM_TEST, svc_tq_001 },
    { "SVC-TQ-002", "Battery discharge current limit", "SWR-TQ-002", VM_TEST, svc_tq_002 },
    { "SVC-TQ-003", "Regenerative braking levels and limits", "SWR-TQ-003", VM_TEST, svc_tq_003 },
    { "SVC-TQ-004", "Regeneration fade near standstill", "SWR-TQ-004", VM_TEST, svc_tq_004 },
    { "SVC-TQ-005", "Brake-throttle override", "SWR-TQ-005", VM_TEST, svc_tq_005 },
    { "SVC-TQ-006", "Reverse gear torque and speed limit", "SWR-TQ-006", VM_TEST, svc_tq_006 },
    { "SVC-TQ-007", "Forward speed limit", "SWR-TQ-007", VM_TEST, svc_tq_007 },
    { "SVC-TQ-008", "Thermal and undervoltage derating", "SWR-TQ-008", VM_TEST, svc_tq_008 },
    { "SVC-TQ-009", "Torque slew rate", "SWR-TQ-009", VM_TEST, svc_tq_009 },
    { "SVC-TQ-010", "Torque removal and neutral/park", "SWR-TQ-010", VM_TEST, svc_tq_010 },
    { "SVC-TQ-011", "Limp mode scaling", "SWR-TQ-011", VM_TEST, svc_tq_011 },

    { "SVC-VSM-001", "Standby state outputs", "SWR-VSM-001", VM_TEST, svc_vsm_001 },
    { "SVC-VSM-002", "Precharge sequence", "SWR-VSM-002, SWR-VSM-003", VM_TEST, svc_vsm_002 },
    { "SVC-VSM-003", "Precharge timeout", "SWR-VSM-004", VM_TEST, svc_vsm_003 },
    { "SVC-VSM-004", "Conditions for entering DRIVE", "SWR-VSM-005", VM_TEST, svc_vsm_004 },
    { "SVC-VSM-005", "Gear changes in DRIVE", "SWR-VSM-006", VM_TEST, svc_vsm_005 },
    { "SVC-VSM-006", "Key off while moving", "SWR-VSM-007", VM_TEST, svc_vsm_006 },
    { "SVC-VSM-007", "Shutdown and active discharge", "SWR-VSM-008", VM_TEST, svc_vsm_007 },
    { "SVC-VSM-008", "Discharge timeout", "SWR-VSM-009", VM_TEST, svc_vsm_008 },
    { "SVC-VSM-009", "Fault reactions in the state machine", "SWR-VSM-010", VM_TEST, svc_vsm_009 },
    { "SVC-VSM-010", "Implausible gear selector", "SWR-VSM-011", VM_INTEG, svc_vsm_010 },

    { "SVC-DIAG-001", "Reaction priority and debouncing", "SWR-DIAG-001", VM_TEST, svc_diag_001 },
    { "SVC-DIAG-002", "Fault latching and clearing", "SWR-DIAG-002", VM_INTEG, svc_diag_002 },
    { "SVC-DIAG-003", "Hardware overcurrent input", "SWR-DIAG-003", VM_INTEG, svc_diag_003 },
    { "SVC-DIAG-004", "Software phase overcurrent", "SWR-DIAG-004", VM_INTEG, svc_diag_004 },
    { "SVC-DIAG-005", "Phase current sum plausibility", "SWR-DIAG-005", VM_INTEG, svc_diag_005 },
    { "SVC-DIAG-006", "DC link overvoltage", "SWR-DIAG-006", VM_INTEG, svc_diag_006 },
    { "SVC-DIAG-007", "DC link undervoltage", "SWR-DIAG-007", VM_INTEG, svc_diag_007 },
    { "SVC-DIAG-008", "Over-temperature and temperature measurement", "SWR-DIAG-008", VM_INTEG, svc_diag_008 },
    { "SVC-DIAG-009", "Temperature sensor open/short", "SWR-DIAG-009", VM_INTEG, svc_diag_009 },
    { "SVC-DIAG-010", "Rotor position sensor fault", "SWR-DIAG-010", VM_INTEG, svc_diag_010 },
    { "SVC-DIAG-011", "Overspeed and speed measurement", "SWR-DIAG-011, SWR-CTL-010", VM_INTEG, svc_diag_011 },
    { "SVC-DIAG-012", "BMS communication loss and BMS fault", "SWR-DIAG-012, SWR-COM-003", VM_INTEG, svc_diag_012 },
    { "SVC-DIAG-013", "Precharge failure handling", "SWR-DIAG-013, SWR-VSM-004", VM_INTEG, svc_diag_013 },

    { "SVC-SAFE-001", "Freewheel / active short circuit selection", "SWR-SAFE-001", VM_INTEG, svc_safe_001 },
    { "SVC-SAFE-002", "Hardware overcurrent at high speed", "SWR-SAFE-002", VM_INTEG, svc_safe_002 },
    { "SVC-SAFE-003", "Task supervision and watchdog service", "SWR-SAFE-003", VM_INTEG, svc_safe_003 },
    { "SVC-SAFE-004", "Safe outputs at start-up and in STANDBY", "SWR-SAFE-004", VM_INTEG, svc_safe_004 },
    { "SVC-SAFE-005", "Current sensor offset calibration", "SWR-SAFE-005, SWR-CTL-011", VM_INTEG, svc_safe_005 },

    { "SVC-COM-001", "BMS message decoding", "SWR-COM-001", VM_TEST, svc_com_001 },
    { "SVC-COM-002", "Rejection of invalid BMS frames", "SWR-COM-002", VM_TEST, svc_com_002 },
    { "SVC-COM-003", "BMS message timeout", "SWR-COM-003", VM_TEST, svc_com_003 },
    { "SVC-COM-004", "Periodic VCU messages", "SWR-COM-004", VM_INTEG, svc_com_004 },

    { "SVC-HW-001", "Clock frequencies", "SWR-HW-001", VM_BENCH, NULL },
    { "SVC-HW-002", "PWM frequency, alignment and dead time", "SWR-HW-002", VM_BENCH, NULL },
    { "SVC-HW-003", "Hardware fault input shutdown time", "SWR-HW-003", VM_BENCH, NULL },
    { "SVC-HW-004", "ADC trigger instant and simultaneous sampling", "SWR-HW-004", VM_BENCH, NULL },
    { "SVC-HW-005", "CAN bit timing and mailbox operation", "SWR-HW-005", VM_BENCH, NULL },
    { "SVC-HW-006", "Task rates on target", "SWR-HW-006", VM_BENCH, NULL },
    { "SVC-HW-007", "Current loop execution time", "SWR-HW-007", VM_BENCH, NULL },
    { "SVC-HW-008", "Watchdog reset on task stall", "SWR-HW-008, SWR-SAFE-003", VM_BENCH, NULL },
    { "SVC-HW-009", "Resolver interface and offset calibration", "SWR-HW-009", VM_BENCH, NULL },
    { "SVC-HW-010", "Target build", "SWR-HW-010", VM_BENCH, NULL },
};
/* clang-format on */

#define N_CASES (sizeof(s_cases) / sizeof(s_cases[0]))

typedef struct {
    int checks;
    int failed;
    bool run;
    char note[1024];
    char first_fail[256];
} vresult_t;

static vresult_t s_res[N_CASES];
static vresult_t *s_cur;

void vt_check(bool ok, const char *file, int line, const char *expr)
{
    if (s_cur == NULL) { return; }
    s_cur->checks++;
    if (!ok) {
        s_cur->failed++;
        printf("    FAIL %s:%d: %s\n", file, line, expr);
        if (s_cur->first_fail[0] == '\0') {
            snprintf(s_cur->first_fail, sizeof(s_cur->first_fail), "%s:%d: %s", file, line, expr);
        }
    }
}

void vt_check_near(double a, double b, double tol, const char *file, int line, const char *expr)
{
    if (s_cur == NULL) { return; }
    s_cur->checks++;
    if (!(a >= b - tol && a <= b + tol)) {
        s_cur->failed++;
        printf("    FAIL %s:%d: %s = %g, expected %g +/- %g\n", file, line, expr, a, b, tol);
        if (s_cur->first_fail[0] == '\0') {
            snprintf(s_cur->first_fail, sizeof(s_cur->first_fail),
                     "%s:%d: %s = %g, expected %g +/- %g", file, line, expr, a, b, tol);
        }
    }
}

void vt_note(const char *fmt, ...)
{
    va_list ap;
    size_t len;
    if (s_cur == NULL) { return; }
    len = strlen(s_cur->note);
    if (len > 0u && len < sizeof(s_cur->note) - 3u) {
        strcat(s_cur->note, "; ");
        len += 2u;
    }
    va_start(ap, fmt);
    vsnprintf(s_cur->note + len, sizeof(s_cur->note) - len, fmt, ap);
    va_end(ap);
    printf("    %s\n", s_cur->note + len);
}

/* ----------------------------------------------------------------------- */

static void svc_gen_001(void)
{
    /* This binary exists only if the build with -Werror succeeded. */
    CHECK(true);
    vt_note("compiled with -std=c99 -Wall -Wextra -Wshadow -Wconversion -Wno-sign-conversion -Wdouble-promotion -Werror, compiler: %s", __VERSION__);
}

static void svc_gen_002(void)
{
    static const char *const files[] = {
        "src/control/foc.c", "src/control/pi.c", "src/control/pll.c", "src/control/svm.c",
        "src/control/foc.h", "src/control/pi.h", "src/control/pll.h", "src/control/svm.h",
        "src/control/mc_math.h",
        "src/app/app.c", "src/app/pedal.c", "src/app/torque_mgr.c", "src/app/vehicle_sm.c",
        "src/app/app.h", "src/app/pedal.h", "src/app/torque_mgr.h", "src/app/vehicle_sm.h",
        "src/app/ev_types.h",
        "src/safety/diag.c", "src/safety/diag.h",
        "src/comm/can_msgs.c", "src/comm/can_msgs.h",
        "src/bsp/bsp.h",
    };
    unsigned i, scanned = 0u;
    for (i = 0; i < sizeof(files) / sizeof(files[0]); i++) {
        char line[512];
        bool hit = false;
        FILE *fp = fopen(files[i], "r");
        CHECK(fp != NULL);
        if (fp == NULL) { continue; }
        scanned++;
        while (fgets(line, sizeof(line), fp) != NULL) {
            if (strstr(line, "MPC5744P.h") != NULL && strstr(line, "#include") != NULL) { hit = true; }
        }
        fclose(fp);
        if (hit) { printf("    %s includes MPC5744P.h\n", files[i]); }
        CHECK(!hit);
    }
    vt_note("%u portable source files scanned, none includes the device header", scanned);
}

/* ----------------------------------------------------------------------- */

static const char *verdict(unsigned i)
{
    if (s_cases[i].fn == NULL) { return "NOT RUN"; }
    return (s_res[i].failed == 0) ? "PASS" : "FAIL";
}

static void md_escape(FILE *fp, const char *s)
{
    for (; *s != '\0'; s++) {
        if (*s == '|') { fputs("\\|", fp); }
        else { fputc(*s, fp); }
    }
}

static void write_report(const char *path, const char *rev)
{
    FILE *fp = fopen(path, "w");
    char date[64];
    time_t now = time(NULL);
    unsigned i, n_run = 0u, n_pass = 0u, n_fail = 0u, n_review = 0u, n_bench = 0u;
    int checks = 0, failed = 0;

    if (fp == NULL) {
        printf("cannot write %s\n", path);
        return;
    }
    strftime(date, sizeof(date), "%Y-%m-%d %H:%M UTC", gmtime(&now));
    for (i = 0; i < N_CASES; i++) {
        if (s_cases[i].fn == NULL) {
            if (s_cases[i].method == VM_BENCH) { n_bench++; } else { n_review++; }
            continue;
        }
        n_run++;
        checks += s_res[i].checks;
        failed += s_res[i].failed;
        if (s_res[i].failed == 0) { n_pass++; } else { n_fail++; }
    }

    fprintf(fp, "# Software Verification Results\n\n");
    fprintf(fp, "> Generated by `make report` from `tests/vtest.c`. Do not edit by hand:\n"
                "> run `make report` again after any change.\n\n");
    fprintf(fp, "| Item | Value |\n|---|---|\n");
    fprintf(fp, "| Document | SVR, results for the cases in [03_software_verification_cases.md](03_software_verification_cases.md) |\n");
    fprintf(fp, "| Software revision | `%s` |\n", (rev != NULL) ? rev : "unknown");
    fprintf(fp, "| Date | %s |\n", date);
    fprintf(fp, "| Test environment | Host PC, `tests/` harness, BSP replaced by `tests/bsp_stub.c` |\n");
    fprintf(fp, "| Compiler | `%s`, `-std=c99 -O2 -Wall -Wextra -Wshadow -Wconversion -Wno-sign-conversion -Wdouble-promotion -Werror` |\n", __VERSION__);
    fprintf(fp, "| Target hardware | None (MPC5744P not available) |\n\n");

    fprintf(fp, "## Summary\n\n");
    fprintf(fp, "| | Cases |\n|---|---|\n");
    fprintf(fp, "| Executed | %u |\n", n_run);
    fprintf(fp, "| Passed | %u |\n", n_pass);
    fprintf(fp, "| Failed | %u |\n", n_fail);
    fprintf(fp, "| Not run: review / static analysis | %u |\n", n_review);
    fprintf(fp, "| Not run: bench test on target hardware | %u |\n", n_bench);
    fprintf(fp, "| Total | %u |\n\n", (unsigned)N_CASES);
    fprintf(fp, "Individual checks executed: %d, failed: %d.\n\n", checks, failed);
    fprintf(fp, "**Verdict:** %s\n\n",
            (n_fail == 0u)
            ? "all executed cases passed. Verification is **incomplete**: the review and "
              "bench cases below are open, so the hardware-dependent requirements (SWR-HW-xxx), "
              "SWR-CTL-009 (review part) and SWR-GEN-003 are not yet verified."
            : "**FAILED**: see the failing cases below.");

    fprintf(fp, "## Results by case\n\n");
    fprintf(fp, "| Case | Title | Verifies | Method | Checks | Result | Evidence / measured values |\n");
    fprintf(fp, "|---|---|---|---|---|---|---|\n");
    for (i = 0; i < N_CASES; i++) {
        fprintf(fp, "| %s | ", s_cases[i].id);
        md_escape(fp, s_cases[i].title);
        fprintf(fp, " | %s | %s | ", s_cases[i].verifies, s_method_name[s_cases[i].method]);
        if (s_cases[i].fn == NULL) {
            fprintf(fp, "– | NOT RUN | %s |\n",
                    (s_cases[i].method == VM_BENCH) ? "Requires MPC5744P hardware" : "Requires manual review / tool");
            continue;
        }
        fprintf(fp, "%d | **%s** | ", s_res[i].checks, verdict(i));
        md_escape(fp, s_res[i].note);
        if (s_res[i].failed != 0) {
            fputs(" FIRST FAILURE: ", fp);
            md_escape(fp, s_res[i].first_fail);
        }
        fputs(" |\n", fp);
    }
    fprintf(fp, "\n## Open items\n\n");
    fprintf(fp, "| Case | Reason not executed | Needed to close |\n|---|---|---|\n");
    for (i = 0; i < N_CASES; i++) {
        if (s_cases[i].fn != NULL) { continue; }
        fprintf(fp, "| %s | %s | %s |\n", s_cases[i].id,
                (s_cases[i].method == VM_BENCH) ? "No MPC5744P hardware available" : "Manual activity, not automated",
                (s_cases[i].method == VM_BENCH)
                ? "Execute procedure in 03_software_verification_cases.md on the target, record measurements"
                : "Perform review / run tool, record findings and disposition");
    }
    fclose(fp);
    printf("report written to %s\n", path);
}

int main(int argc, char **argv)
{
    const char *report = NULL;
    const char *rev = NULL;
    unsigned i;
    int failed_cases = 0;
    int checks = 0;
    int a;

    for (a = 1; a < argc; a++) {
        if (strcmp(argv[a], "--report") == 0 && a + 1 < argc) { report = argv[++a]; }
        else if (strcmp(argv[a], "--rev") == 0 && a + 1 < argc) { rev = argv[++a]; }
    }

    for (i = 0; i < N_CASES; i++) {
        if (s_cases[i].fn == NULL) {
            printf("[ SKIP ] %-13s %s (%s)\n", s_cases[i].id, s_cases[i].title, s_method_name[s_cases[i].method]);
            continue;
        }
        printf("[ RUN  ] %-13s %s\n", s_cases[i].id, s_cases[i].title);
        s_cur = &s_res[i];
        s_cur->run = true;
        s_cases[i].fn();
        s_cur = NULL;
        checks += s_res[i].checks;
        if (s_res[i].failed != 0) { failed_cases++; }
        printf("[ %s ] %s (%d checks)\n", (s_res[i].failed == 0) ? " OK " : "FAIL",
               s_cases[i].id, s_res[i].checks);
    }
    printf("\n%u cases (%d failed), %d checks\n", (unsigned)N_CASES, failed_cases, checks);

    if (report != NULL) { write_report(report, rev); }
    return (failed_cases != 0) ? 1 : 0;
}
