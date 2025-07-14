#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/sys/printk.h>
#include <zephyr/pm/device.h> // Include the device power management header

/* Get the device pointer directly from the alias in the overlay */
static const struct device *pwm0 = DEVICE_DT_GET(DT_NODELABEL(flexpwm0_pwm0));
static const struct device *pwm1 = DEVICE_DT_GET(DT_NODELABEL(flexpwm0_pwm1));
static const struct device *pwm2 = DEVICE_DT_GET(DT_NODELABEL(flexpwm0_pwm2));

/* Define your PWM parameters in nanoseconds */
#define PWM_PERIOD_NS 20000  // 20,000 ns = 50 kHz
#define PWM_PULSE_NS  5000   // Example 25% duty cycle pulse

int main(void)
{
    uint32_t period_cycles;
    uint32_t pulse_cycles;
    uint64_t cycles_per_sec0, cycles_per_sec1, cycles_per_sec2;
    int ret;


    if (!device_is_ready(pwm0)) {
        printk("Error: PWM device %s is not ready\n", pwm0->name);
        return 0;
    }
    printk("PWM device is ready.\n");
    if (!device_is_ready(pwm1)) {
        printk("Error: PWM device %s is not ready\n", pwm0->name);
        return 0;
    }
    printk("PWM device is ready.\n");
    if (!device_is_ready(pwm2)) {
        printk("Error: PWM device %s is not ready\n", pwm0->name);
        return 0;
    }
    printk("PWM device is ready.\n");

    /* Get the clock frequency of the PWM peripheral */
    ret = pwm_get_cycles_per_sec(pwm0, 0, &cycles_per_sec0);
    if (ret) {
        printk("Error %d: failed to get PWM clock rate\n", ret);
        return 0;
    }
    printk("Got PWM clock rate: %llu Hz\n", cycles_per_sec0);




    ret = pwm_get_cycles_per_sec(pwm1, 0, &cycles_per_sec1);
    if (ret) {
        printk("Error %d: failed to get PWM clock rate\n", ret);
        return 0;
    }
    printk("Got PWM clock rate: %llu Hz\n", cycles_per_sec1);



    ret = pwm_get_cycles_per_sec(pwm2, 0, &cycles_per_sec2);
    if (ret) {
        printk("Error %d: failed to get PWM clock rate\n", ret);
        return 0;
    }
    printk("Got PWM clock rate: %llu Hz\n", cycles_per_sec2);

    if (cycles_per_sec0 == 0) {
        printk("Error: PWM clock rate is zero, aborting.\n");
        return 0;
    }

    /* Calculate period and pulse width in cycles */
    period_cycles = (uint32_t)((cycles_per_sec0 * PWM_PERIOD_NS) / 1000000000ULL);
    pulse_cycles = (uint32_t)((cycles_per_sec0 * PWM_PULSE_NS) / 1000000000ULL);
    printk("Calculated cycles: Period=%u, Pulse=%u\n", period_cycles, pulse_cycles);

    /* Set the PWM period and pulse width using the calculated cycle values */
    ret = pwm_set_cycles(pwm0, 0, period_cycles, pulse_cycles, 0);
    if (ret) {
        printk("Error %d: failed to set PWM cycles\n", ret);
        return 0;
    }
    ret = pwm_set_cycles(pwm0, 1, period_cycles, pulse_cycles, 0);
    if (ret) {
        printk("Error %d: failed to set PWM cycles\n", ret);
        return 0;
    }
    ret = pwm_set_cycles(pwm1, 0, period_cycles, pulse_cycles, 0);
    if (ret) {
        printk("Error %d: failed to set PWM cycles\n", ret);
        return 0;
    }
    ret = pwm_set_cycles(pwm1, 1, period_cycles, pulse_cycles, 0);
    if (ret) {
        printk("Error %d: failed to set PWM cycles\n", ret);
        return 0;
    }
    ret = pwm_set_cycles(pwm2, 0, period_cycles, pulse_cycles, 0);
    if (ret) {
        printk("Error %d: failed to set PWM cycles\n", ret);
        return 0;
    }
    ret = pwm_set_cycles(pwm2, 1, period_cycles, pulse_cycles, 0);
    if (ret) {
        printk("Error %d: failed to set PWM cycles\n", ret);
        return 0;
    }
    printk("PWM setup complete. Signal running indefinitely.\n");

    while (1) {
    }

    return 0;
}
