
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/__assert.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor_data_types.h>
#include <zephyr/rtio/rtio.h>
#include <zephyr/dsp/print_format.h>
#include <zephyr/drivers/dac.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>
#include <canopennode.h>
#include <CO_OD.h>

LOG_MODULE_REGISTER(oresat_solar2, LOG_LEVEL_INF);

// CAN data debug logging
//#define DUMP_SOLAR_DATA 1

/* ===  Thread Parameters  === */

#define STACKSIZE               1024
#define PRIORITY                7

/* ===  Algorithm Parameters  === */
#define CC_ENABLE               true //enable corner cutting
#define CC_STEP_SCALE           400.0f //how does a trend effect our step size
#define CC_PMAX                 0.0045f
#define CC_PRATE                0.1f
#define CC_NMIN                 0.0041f
#define CC_NRATE                0.1f

#define DL_ENABLE               false

#define IE_ENABLE               DL_ENABLE || CC_ENABLE
#define IE_ARRAY_LEN            4
#define IE_SAMPLE_SPACING       8

/* MPPT configuration */
#define I_ADJ_FAILSAFE          1450000
#define I_ADJ_INITIAL           1450000
#define I_ADJ_MAX               1450000
#define I_ADJ_MIN               0

#define CRITICAL_SLOPE          0.00420f // mW/uA
#define IADJ_SAMPLE_OFFSET_uV   25000
#define SLOPE_CORRECTION_FACTOR 500.0f
#define FLOAT_DIST_TO_ZERO      0.1
#define VREF_STEP_NEGATIVE_uV  -16000
#define VREF_STEP_POSITIVE_uV   (VREF_STEP_NEGATIVE_uV * -4) //ratio of 2
#define MAX_STEP                100000 //cap steps so they aren't too big when dynamic
#define CURRENT_SETTLE_TIME     2 //ms

#define ITERATION_PERIOD        40

/* === Peripheral Parameters === */

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

//FIX: only complains about issues with dac1, should check dac0 aswell
#if (DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, dac1) && \
    DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, dac_channel_id) && \
    DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, dac_resolution))
#define DAC1_NODE DT_PHANDLE(ZEPHYR_USER_NODE, dac1)
#define DAC_CHANNEL_ID DT_PROP(ZEPHYR_USER_NODE, dac_channel_id)
#define DAC_RESOLUTION DT_PROP(ZEPHYR_USER_NODE, dac_resolution)
#else
#error "Unsupported board: see README and check /zephyr,user node"
#define DAC_NODE DT_INVALID_NODE
#define DAC_CHANNEL_ID 0
#define DAC_RESOLUTION 0
#endif

#define DAC_VDDA_uV 3333000
#define DAC_VALUES (1U << DAC_RESOLUTION)
#define DAC_UV_PER_BIT (DAC_VDDA_uV / DAC_VALUES)

/* === Peripheral Structs === */

static const struct device *const ina = DEVICE_DT_GET_ONE(ti_ina226);
const struct device *const dac1_dev = DEVICE_DT_GET(DAC1_NODE);
const struct dac_channel_cfg dac_ch_cfg = {
        .channel_id  = DAC_CHANNEL_ID,
        .resolution  = DAC_RESOLUTION,
    #if defined(CONFIG_DAC_BUFFER_NOT_SUPPORT)
        .buffered = false,
    #else
        .buffered = true,
    #endif /* CONFIG_DAC_BUFFER_NOT_SUPPORT */
}; //TODO: specify averaging

/* === GPIO data === */
#define BP_NODE DT_NODELABEL(solargpios)

static const struct gpio_dt_spec ina226_nalert = GPIO_DT_SPEC_GET(BP_NODE, ina226_nalert_gpios);
static const struct gpio_dt_spec cell1_tmp101_alert = GPIO_DT_SPEC_GET(BP_NODE, cell1_tmp101_alert_gpios);
static const struct gpio_dt_spec cell2_tmp101_alert = GPIO_DT_SPEC_GET(BP_NODE, cell2_tmp101_alert_gpios);
static const struct gpio_dt_spec lt1618_enable = GPIO_DT_SPEC_GET(BP_NODE, lt1618_enable_gpios);

/**************************************************/

static int gpios_init(void)
{
    int ret;

    ret = gpio_pin_configure_dt(&ina226_nalert, GPIO_INPUT);
    if (ret) {
        return ret;
    }
    ret = gpio_pin_configure_dt(&cell1_tmp101_alert, GPIO_INPUT);
    if (ret) {
        return ret;
    }
    ret = gpio_pin_configure_dt(&cell2_tmp101_alert, GPIO_INPUT);
    if (ret) {
        return ret;
    }
    ret = gpio_pin_configure_dt(&lt1618_enable, GPIO_OUTPUT_ACTIVE); // enable the LT1618
    if (ret) {
        return ret;
    }

    return ret;
}

/**************************************************/

/* === Algorithm Structs === */

typedef enum {
    MPPT_ALGORITHM_PAO = 0
} mppt_algorithm_t;

struct Sample //TODO: should a sample be floats or sensor_values?
{
    float power_mW;
    float voltage_mV;
    float current_uA;
    uint32_t time;
};

typedef struct {
    uint32_t iadj_uV;
    struct Sample sample;
    int32_t last_time_mS;
    struct Sample IE_samples[IE_ARRAY_LEN];
    uint32_t index_loop_counter;
} MpptState;

static int init_ina226(void)
{
    LOG_INF("Starting INA226 reading");

    if (!device_is_ready(ina)) {
        LOG_ERR("Device %s is not ready.", ina->name);
    }
    return 0;
}

uint32_t saturate_uint32_t(const int64_t v, const uint32_t min, const uint32_t max) {
    if (v > max) {
        return max;
    } else if (v < min) {
        return min;
    }
    return v;
}

int dac_write_uV(int32_t iadj)
{
    int32_t toset = (iadj / DAC_UV_PER_BIT);
    return dac_write_value(dac1_dev, DAC_CHANNEL_ID, toset);
}

//reads current and voltage of ina226
void observe(struct Sample* sample)
{
    struct sensor_value v_bus;
    struct sensor_value current;
    struct sensor_value power;
    int rc; //return code

    k_msleep(CURRENT_SETTLE_TIME);
    rc = sensor_sample_fetch(ina);
    if (rc) {
        LOG_ERR("Could not fetch sensor data: %d", rc);
        //    return 1;
    } else {
        sensor_channel_get(ina, SENSOR_CHAN_VOLTAGE, &v_bus);
        sensor_channel_get(ina, SENSOR_CHAN_CURRENT, &current);
        sensor_channel_get(ina, SENSOR_CHAN_POWER, &power);
        //TODO: should values be scaled to prefixes here? or stay as base units under double prescision?
        //TODO: I was getting very low resolution on the power values from the ina226, so we're calculating it here
        sample->power_mW = sensor_value_to_double(&v_bus) * sensor_value_to_double(&current) * 1E3;
        sample->voltage_mV = sensor_value_to_double(&v_bus) * 1E3;
        sample->current_uA = sensor_value_to_double(&current) * 1E6;
        //sample->power_mW = sensor_value_to_double(&power) * 1E3;
        sample->time = k_uptime_get();
        // LOG_INF("data after observe: voltage: %f [mV], current: %f [uA], power %f [mW]", (double)sample->voltage_mV, (double)sample->current_uA, (double)sample->power_mW);
    }

    //return 0;
}

float find_ip_slope(MpptState* state, int32_t initial_iadj)
{
    struct Sample first;
    struct Sample second;
    struct Sample third;

    int32_t working_iadj = initial_iadj;
    observe(&first);
    state->sample = first;
    LOG_DBG("first sample: voltage: %f [mV], current: %f [uA], power: %f [mW]", (double)first.voltage_mV, (double)first.current_uA, (double)first.power_mW);

    working_iadj += IADJ_SAMPLE_OFFSET_uV;
    dac_write_uV(working_iadj);
    observe(&second);
    //LOG_DBG("second sample: voltage: %f [mV], current: %f [uA]", (double)second.voltage_mV, (double)second.current_uA);

    working_iadj += IADJ_SAMPLE_OFFSET_uV;
    dac_write_uV(working_iadj);
    observe(&third);
    //LOG_DBG("third sample: voltage: %f [mV], current: %f [uA]", (double)third.voltage_mV, (double)third.current_uA);

    float delta_power1 = first.power_mW - second.power_mW;
    float delta_current1 = first.current_uA - second.current_uA;

    float delta_power2 = second.power_mW - third.power_mW;
    float delta_current2 = second.current_uA - third.current_uA;

    float slope = (delta_power1 * delta_current2 + delta_power2 * delta_current1) / (2.0f * delta_current1 * delta_current2);
    //LOG_DBG(" delta_power1:%f, delta_current1:%f, delta_power2:%f, delta_current2:%f", (double)delta_power1, (double)delta_current1, (double)delta_power2, (double)delta_current2);

    LOG_DBG("calculated slope as %f out of %f \n\r", (double)slope, (double)CRITICAL_SLOPE);
    dac_write_uV(initial_iadj);
    return slope;
}

float find_pt_slope(struct Sample* newer, struct Sample* older) {
    float dp = (newer->power_mW) - (older->power_mW);
    int32_t dt = newer->time - older->time; //FIX: NOT SURE THAT THIS HANDLES ZEPHYR TIME CORRECTLY
    float dpdt = 0.0;
    if (dt > 0) {
        dpdt = dp / (float) dt;
    }
    return dpdt;
}



int32_t calculate_step(MpptState* state)
{

#if IE_ENABLE
    float pt_slope = 0.0;

    struct Sample newer = state->IE_samples[(state->index_loop_counter - 1) % IE_ARRAY_LEN];
    struct Sample older = state->IE_samples[state->index_loop_counter % IE_ARRAY_LEN];
    pt_slope = find_pt_slope(&newer, &older);

    pt_slope = pt_slope / ((float) IE_ARRAY_LEN);
    LOG_INF("pt_slope %f", (double)pt_slope);
#endif



    int32_t CC_step = 0;
    float CC_critical_adjust = 0.0;

#if CC_ENABLE
    CC_step = pt_slope * CC_STEP_SCALE;
    LOG_INF("CC_step is %f ", (double)CC_step);

    if (pt_slope < 0) {
        CC_critical_adjust = pt_slope * CC_NRATE;
    } else if (pt_slope > 0) {
        CC_critical_adjust = pt_slope * CC_PRATE;
    }

#endif


    float ip_slope = find_ip_slope(state, state->iadj_uV);
    float reference_slope = CRITICAL_SLOPE - CC_critical_adjust;
    LOG_INF("reference slope is %f, PMAX is %f, NMIN is %f ", 
            (double)reference_slope, (double)CC_PMAX, (double)CC_NMIN);

#if CC_ENABLE
    if (reference_slope > CC_PMAX) {
        reference_slope = CC_PMAX;
    } else if (reference_slope < CC_NMIN) {
        reference_slope = CC_NMIN;
    }
    LOG_INF("reference slope bounded to %f", (double)reference_slope);
#endif

    float slope_error = (ip_slope - reference_slope) * SLOPE_CORRECTION_FACTOR;

#if DL_ENABLE
    //TODO: dynamic laziness goes here
#endif

    int32_t step = 0;
    if (slope_error < 0) {
        step = (int32_t) (VREF_STEP_POSITIVE_uV * (-slope_error) + CC_step);
    } else if (slope_error > 0) {
        step = VREF_STEP_NEGATIVE_uV + CC_step;
    } else {
        step = VREF_STEP_POSITIVE_uV + CC_step;
    }

    LOG_INF("end of calculate_step");
    return step > MAX_STEP ? MAX_STEP : step;
}

void iterate(MpptState* state)
{
    const int64_t iadj = state->iadj_uV + calculate_step(state);
    const uint32_t iadj_uV_perturbed = saturate_uint32_t(iadj, I_ADJ_MIN, I_ADJ_MAX);

    dac_write_uV(iadj_uV_perturbed);
    state->iadj_uV = iadj_uV_perturbed;
    LOG_INF("iterated");
}

int track(void)
{
    LOG_INF("Starting Solar Tracking...");
    int ret;
    i2c_recover_bus(DEVICE_DT_GET(DT_NODELABEL(flexcomm0_lpi2c0)));

    ret = gpios_init();
    if (ret != 0) {
        LOG_ERR("Error initializing GPIO lines: %d", ret);
        return ret;
    }
    init_ina226();

    /* Can we use the DAC? */
    if (!device_is_ready(dac1_dev)) {
        LOG_ERR("DAC1 device %s is not ready", dac1_dev->name);
        return -1;
    }

    /* Set it up */
    ret = dac_channel_setup(dac1_dev, &dac_ch_cfg);
    if (ret != 0) {
        LOG_ERR("Setting up of DAC1 channel failed with code %d", ret);
        return ret;
    }

    MpptState state = {
        .iadj_uV = I_ADJ_INITIAL,
    };

    //FIX: could there be issues when time wraps around?
    int32_t t_start = k_uptime_get(); //in milliseconds
    int32_t t_last = t_start;
    int32_t t_now = t_start;
    uint32_t energy_mJ = 0;

    state.index_loop_counter = 0;
    int32_t spacing_loop_counter = 0;

#if IS_ENABLED(CONFIG_CHARACTERIZATION_SWEEP) // check Kconfig; if set to Y in prj.conf or west build line, enable code below:
    //CHARACTERIZATION SWEEP
#warning "enabled characterization sweep"
    int iadj_stepsize = 2500; //uA
    int looping_iadj = 1600000;
    while(1) {
        dac_write_uV(looping_iadj);
        looping_iadj -= iadj_stepsize;
        struct Sample sample;
        observe(&sample);
        //LOG_INF("%d %f %f %f %d", k_uptime_get(), sample.voltage_mV, sample.current_uA, sample.power_mW, looping_iadj);
       // LOG_INF("solar thread ran");
        LOG_INF("%f %f %f", sample.current_uA, sample.voltage_mV, sample.power_mW);
        k_msleep(2);
    }
    //END CHARACTERIZATION SWEEP
#endif
    //Only PAO implemented for the time being
    CO_LOCK_OD();
    CO_OD_RAM.mppt_alg = MPPT_ALGORITHM_PAO;
    CO_UNLOCK_OD();

#ifdef DUMP_SOLAR_DATA
    char dump[sizeof(CO_OD_RAM.output) * 3 + 1] = {0};
#endif

    while(1) {
#if IE_ENABLE
        if (!(spacing_loop_counter % IE_SAMPLE_SPACING)) {
            state.IE_samples[state.index_loop_counter % IE_ARRAY_LEN] = state.sample;
            state.index_loop_counter++;
        }
#endif
        iterate(&state);

        spacing_loop_counter += 1;

        t_last = t_now;
        t_now = state.sample.time;
        energy_mJ += state.sample.power_mW * (t_now - t_last) * 1000; //convert ms to s

        // Dividing by 1k to convert to joules and truncate to 16 bits for the OD.
        // FIXME: truncation looks suspicious

        //send stuff to OD_RAM (eventually sent over CAN)
        CO_LOCK_OD();
        CO_OD_RAM.output.energy = (uint16_t) energy_mJ / 1000;

        CO_OD_RAM.output.voltage = (uint16_t) state.sample.voltage_mV;
        CO_OD_RAM.output.voltage_avg = (uint16_t) state.sample.voltage_mV;
        CO_OD_RAM.output.current = (uint16_t) (state.sample.current_uA / 1000);
        CO_OD_RAM.output.current_avg = (uint16_t) (state.sample.current_uA / 1000);
        CO_OD_RAM.output.power = (uint16_t) state.sample.power_mW;
        CO_OD_RAM.output.power_avg = (uint16_t) state.sample.power_mW;

        CO_OD_RAM.output.voltage_max = MAX(CO_OD_RAM.output.voltage_max, (uint16_t) state.sample.voltage_mV);
        CO_OD_RAM.output.current_max = MAX(CO_OD_RAM.output.current_max, (uint16_t) (state.sample.current_uA / 1000));
        CO_OD_RAM.output.power_max = MAX(CO_OD_RAM.output.power_max, (uint16_t) state.sample.power_mW);

        CO_OD_RAM.lt1618_iadj = state.iadj_uV / 1000;
        CO_UNLOCK_OD();

#ifdef DUMP_SOLAR_DATA
        LOG_INF("energy:%u, mV:%3.3f, uA:%3.3f, mW:%3.3f, mA:%u", energy_mJ, (double)state.sample.voltage_mV, (double)state.sample.current_uA, (double)state.sample.power_mW, state.iadj_uV);

        char *o = dump;
        char *p = (char *)&CO_OD_RAM.output;
        int len = sizeof(dump);
        for (int i = 0; i < sizeof(CO_OD_RAM.output); i++) {
            snprintk(o, len, "%02x ", *(p++));
            len -= 3;
            o += 3;
        }
        LOG_INF("output: %s", dump);
#endif

        ///send stuff to OD ram or something
        t_now = k_uptime_get(); // *TODO* reevaluate these timing calculations -- no longer constant interval between calls to interate()
        LOG_INF("main looped");
        k_msleep(ITERATION_PERIOD - (t_start - t_now) % ITERATION_PERIOD);
    }

    return 0;
}

K_THREAD_DEFINE(solar_id, STACKSIZE, track, NULL, NULL, NULL, PRIORITY, 0, 0);
