#define _GNU_SOURCE
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <complex.h>
#include <math.h>
#include <spawn.h>
#include <sys/types.h>
#include <sys/wait.h>

extern char **environ;

// settings
#define TICKS_PER_SECOND  5
#define IIR_COEF          0.2
#define SKEW_LIMIT        1
// skew is m/s/s

#define ATTR_PATH "/sys/bus/iio/devices/%s/%s"

struct accel {
    double scale;
    FILE *a_y;
    FILE *a_z;
};

const char accel_base_device_name[] = "iio:device1";
const char accel_lid_device_name[]  = "iio:device3";

/*  x/y/z base/lid
    z is perpendicular to the plane
    x is parallel to the hinge (useless here)
    y is the other one
    units is m/s/s
*/

#define ATTR_BUF_SIZE 20

// globals
struct accel accel_base;
struct accel accel_lid;

static FILE* accel_open_attr_file(const char* device, const char* attr) {
    FILE* fp;
    char* path;
    if (asprintf(&path, ATTR_PATH, device, attr) == -1) {
        fprintf(stderr, "asprintf failed\n");
        abort();
    }
    fp = fopen(path, "r");
    if (!fp) {
        perror("opening accelerometer file");
        abort();
    }
    free(path);
    return fp;
}

static FILE *accel_open_axis_file(const char* device_name, const char *axis) {
    char* attr_name;
    if (asprintf(&attr_name, "in_accel_%s_raw", axis) == -1) {
        fprintf(stderr, "asprintf failed\n");
        abort();
    }
    FILE *f = accel_open_attr_file(device_name, attr_name);
    free(attr_name);
    return f;
}

static void accel_open(struct accel* accel, const char* device_name) {
    char buf[ATTR_BUF_SIZE];
    FILE *scale_file = accel_open_attr_file(device_name, "scale");
    if (fread(buf, 1, ATTR_BUF_SIZE, scale_file) == 0) {
        fprintf(stderr, "error reading scale attribute\n");
        abort();
    };
    fclose(scale_file);

    accel->scale = atof(buf);
    accel->a_y = accel_open_axis_file(device_name, "y");
    accel->a_z = accel_open_axis_file(device_name, "z");
}

static void accel_close(struct accel* accel) {
    fclose(accel->a_y);
    fclose(accel->a_z);
}

static void accel_setup() {
    accel_open(&accel_base, accel_base_device_name);
    accel_open(&accel_lid,  accel_lid_device_name);
}

static void accel_cleanup() {
    accel_close(&accel_base);
    accel_close(&accel_lid);
}

static double read_float_from_attr_file(FILE* f) {
    char buf[ATTR_BUF_SIZE];
    if (fread(buf, 1, ATTR_BUF_SIZE, f) == 0) {
        fprintf(stderr, "error reading attribute\n");
        abort();
    };

    // attr files produce a new value every time they're read, reset
    // this file for the next read.
    rewind(f);

    return atof(buf);
}

static double complex accel_read(struct accel* accel) {
    return (read_float_from_attr_file(accel->a_y) * accel->scale) +
           (read_float_from_attr_file(accel->a_z) * accel->scale * I);
}

static double cdet(double complex a, double complex b) {
    return creal(a) * cimag(b) - cimag(a) * creal(b);
}

static double cdot(double complex a, double complex b) {
    return creal(a) * creal(b) + cimag(a) * cimag(b);
}

// Returns the hinge angle via the outparam `angle`. Returns 0 if
// successful, or non-zero otherwise. When the device is flat, the
// angle is 0; when the device is being used as a laptop, the angle is
// around 90 degrees; when the base is being used as a stand, the
// angle is around -90 degrees; when the laptop is in full tablet
// mode, the angle is around +/- 180 degrees.
static int sample_angle(double *angle) {
    double complex v_base = accel_read(&accel_base);
    double complex v_lid  = accel_read(&accel_lid);
    double m_base = cabs(v_base);
    double m_lid  = cabs(v_lid);

    // ignore if axis might be ambiguous
    if ((m_base < SKEW_LIMIT) ||
        (m_lid  < SKEW_LIMIT))
    {
        return -1;
    }

    // ignore if accel > gravity
    if ((m_base > (10 + SKEW_LIMIT)) ||
        (m_lid  > (10 + SKEW_LIMIT)))
    {
        return -1;
    }

    // normalize
    v_base = v_base / m_base;
    v_lid  = v_lid  / m_lid;

    *angle = atan2(cdet(v_lid, v_base),
                   cdot(v_lid, v_base))
        * 180 / M_PI;

    // printf("angle=%f\n", *angle);

    return 0;
}

enum mode_t {
    LAPTOP_MODE,
    TABLET_MODE,
};

static int is_laptop(double angle) {
    return angle > -20 && angle < 160;
}

static int is_tablet(double angle) {
    return angle > 170 || angle < 20;
}

// Returns when the sampled angle is further from the origin than the
// specified angle.
static mode_t wait_for_mode_change(mode_t current_mode) {
    const double sub_iir = 1.0 - IIR_COEF;
    const int us_tick    = 1000000 / TICKS_PER_SECOND;

    double angle_avg = -1000;  // un-init flag
    double angle;

    while (1) {
        usleep(us_tick);

        if (sample_angle(&angle) != 0) {
            continue;
        }

        if (angle_avg == -1000) {
            angle_avg = angle;
        }

        angle_avg = IIR_COEF * angle + sub_iir * angle_avg;

        // printf("angle_avg=%f\n", angle_avg);

        // thresholds
        if (current_mode == LAPTOP_MODE) {
            if (is_tablet(angle) && !is_laptop(angle)) {
                return TABLET_MODE;
            }
        }
        else {
            if (is_laptop(angle) && !is_tablet(angle)) {
                return LAPTOP_MODE;
            }
        }
    }
}

static void on_mode(mode_t mode, const char *hook) {
    char *mode_name = mode == TABLET_MODE ? "tablet" : "laptop";
    printf("mode: %s\n", mode_name);

    char *hook_arg = strdup(hook);
    char* args[] = { hook_arg, mode_name, NULL };
    pid_t child_pid;
    int child_status;
    if (posix_spawnp(&child_pid, hook, NULL, NULL, args, environ) != 0) {
        perror("posix_spawnp()");
        goto cleanup;
    }
    waitpid(child_pid, &child_status, WEXITED);

cleanup:
    free(hook_arg);
}

int main(int argc, char **argv) {
    const char *hook = NULL;

    if (argc >= 2) {
        hook = argv[1];
    } else {
        fprintf(stderr, "Missing hook argument\n");
        exit(1);
    }

    enum mode_t mode;
    accel_setup();

    // determine initial state
    while (1) {
        double angle;
        if (sample_angle(&angle) == 0) {
            if (is_laptop(angle) && !is_tablet(angle)) {
                mode = LAPTOP_MODE;
                break;
            }
            else if (is_tablet(angle) && !is_laptop(angle)) {
                mode = TABLET_MODE;
                break;
            }
        }
        usleep(100000); // 100ms
    }

    on_mode(mode, hook);

    while (1) {
        mode = wait_for_mode_change(mode);
        on_mode(mode, hook);
    }
}
