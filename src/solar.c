
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/__assert.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor_data_types.h>
#include <zephyr/rtio/rtio.h>
#include <zephyr/dsp/print_format.h>
#include <zephyr/drivers/dac.h>
#include <zephyr/drivers/adc.h>


LOG_MODULE_REGISTER(oresat_blink, LOG_LEVEL_DBG);


/* ===  Thread Parameters  === */

#define STACKSIZE 1024
#define PRIORITY 7



/* ===  Algorithm Parameters  === */

#define IE_ARRAY_LEN 4
#define IE_SAMPLE_SPACING 8

#define CC_ENABLE true //enable corner cutting
#define CC_STEP_SCALE 400.0 //how does a trend effect our step size
#define CC_PMAX 0.0045
#define CC_PRATE 0.1
#define CC_NMIN  0.0041
#define CC_NRATE 0.1

/* MPPT configuration */
#define I_ADJ_FAILSAFE          1450000
#define I_ADJ_INITIAL           1450000
#define I_ADJ_MAX               1450000
#define I_ADJ_MIN               0

#define CRITICAL_SLOPE            0.00425// mW/uA
#define IADJ_SAMPLE_OFFSET_uV 25000
#define SLOPE_CORRECTION_FACTOR 500.0
#define FLOAT_DIST_TO_ZERO 0.1
#define VREF_STEP_NEGATIVE_uV             -16000
#define VREF_STEP_POSITIVE_uV             (VREF_STEP_NEGATIVE_uV * -4) //ratio of 2
#define MAX_STEP 100000 //cap steps so they aren't too big when dynamic

#define ITERATION_PERIOD 50



/* === Peripheral Parameters === */

#if (DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, dac) && \
	DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, dac_channel_id) && \
	DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, dac_resolution))
#define DAC_NODE DT_PHANDLE(ZEPHYR_USER_NODE, dac)
#define DAC_CHANNEL_ID DT_PROP(ZEPHYR_USER_NODE, dac_channel_id)
#define DAC_RESOLUTION DT_PROP(ZEPHYR_USER_NODE, dac_resolution)
#else
#error "Unsupported board: see README and check /zephyr,user node"
#define DAC_NODE DT_INVALID_NODE
#define DAC_CHANNEL_ID 0
#define DAC_RESOLUTION 0
#endif

#define V_IN 3.3
#define DAC_VOLTS_PER_BIT V_IN / DAC_RESOLUTION



/* === Peripheral Structs === */

static const struct device *const ina = DEVICE_DT_GET_ONE(ti_ina226);
const struct device *const dac_dev = DEVICE_DT_GET(DAC_NODE);
const struct dac_channel_cfg dac_ch_cfg = {
		.channel_id  = DAC_CHANNEL_ID,
		.resolution  = DAC_RESOLUTION,
	#if defined(CONFIG_DAC_BUFFER_NOT_SUPPORT)
		.buffered = false,
	#else
		.buffered = true,
	#endif /* CONFIG_DAC_BUFFER_NOT_SUPPORT */
	}; //TODO: specify averaging



/* === Algorithm Structs === */

struct Sample //TODO: should a sample be floats or sonsor_values?
{
    float power_mW;
    float voltage_mV;
    float current_uA;
//    systime_t time;
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
    //FIX: this garbage
    int32_t toset = iadj / ((float) DAC_VOLTS_PER_BIT * 1E-6); // volts to microvolts
    return dac_write_value(dac_dev, DAC_CHANNEL_ID, toset);
}

float find_ip_slope(MpptState* state, int32_t initial_iadj)
{
    struct Sample first;
    struct Sample second;
    struct Sample third;

    int32_t working_iadj = initial_iadj;
    observe(&first);
    state->sample = first;

    working_iadj += IADJ_SAMPLE_OFFSET_uV;
    dac_write_mV(working_iadj);
    observe(&second);

    working_iadj += IADJ_SAMPLE_OFFSET_uV;
    dac_write_mV(working_iadj);
    observe(&third);

    float delta_power1 = first.power_mW - second.power_mW;
    float delta_current1 = first.current_uA - second.current_uA;

    float delta_power2 = second.power_mW - third.power_mW;
    float delta_current2 = second.current_uA - third.current_uA;

    float slope = (delta_power1 * delta_current2 + delta_power2 * delta_current1) / (2.0 * delta_current1 * delta_current2);

    LOG_INF("calculated slope as %d/10,000 out of %d \n\r", (int32_t) (slope * 10000), (int32_t) (CRITICAL_SLOPE * 10000));
    dac_write_mV(initial_iadj);
    return slope;
}

//reads current and voltage of ina226
bool observe(struct Sample* sample)
{
    struct sensor_value v_bus;
    struct sensor_value current;
    int rc; //return code

    rc = sensor_sample_fetch(ina);
    if (rc) {
        LOG_ERR("Could not fect sensor data: %d", rc);
        return 1;
    } else {
        sensor_channel_get(ina, SENSOR_CHAN_VOLTAGE, &v_bus);
        sensor_channel_get(ina, SENSOR_CHAN_CURRENT, &current);

        sample->voltage_mV = sensor_value_to_double(&v_bus);
        sample->current_uA = sensor_value_to_double(&current);
    }

    return 0;
}

int32_t calculate_step(MpptState* state)
{
    int32_t CC_step = 0;
    float CC_critical_adjust = 0.0;
    float ip_slope = find_ip_slope(state, state->iadj_uV);
    float reference_slope = CRITICAL_SLOPE - CC_critical_adjust;

    float slope_error = (ip_slope - reference_slope) * SLOPE_CORRECTION_FACTOR;

    int32_t step = 0;
    if (slope_error < 0) {
        step = VREF_STEP_POSITIVE_uV * (slope_error * -1) + CC_step;
    } else if (slope_error > 0) {
        step = VREF_STEP_NEGATIVE_uV + CC_step;
    } else {
        step = VREF_STEP_POSITIVE_uV + CC_step;
    }

    return step > MAX_STEP ? MAX_STEP : step;
}

void iterate(MpptState* state)
{
    const int64_t iadj = state->iadj_uV + calculate_step(state);
    const uint32_t iadj_uV_perturbed = saturate_uint32_t(iadj, I_ADJ_MIN, I_ADJ_MAX);

    dac_write_mV(iadj_uV_perturbed);
    state->iadj_uV = iadj_uV_perturbed;
}

int track()
{
    LOG_INF("Starting Solar Tracking...");

    int ret = 0;

    init_ina226();

	/* Can we use the DAC? */
    if (!device_is_ready(dac_dev)) {
		LOG_ERR("DAC device %s is not ready", dac_dev->name);
		return -1;
	}

	/* Set it up */
	ret = dac_channel_setup(dac_dev, &dac_ch_cfg);
	if (ret != 0) {
		LOG_ERR("Setting up of DAC channel failed with code %d", ret);
		return ret;
	}











    return 0;
}


K_THREAD_DEFINE(solar_id, STACKSIZE, track, NULL, NULL, NULL, PRIORITY, 0, 0);
