#ifndef _GPIOS_H_
#define _GPIOS_H_

typedef enum {
	HW_REV_UNKNOWN = -1,	// board revision 1.0.0
	HW_REV_1_0_0 = 0,	// board revision 1.0.0
	HW_REV_1_1_0 = 1,	// board revision 1.1.0 -- we cannot distinguish this from 0.0.0 automatically
	HW_REV_1_2_0 = 2,	// board revision 1.2.0
	HW_REV_1_3_0 = 3	// board revision 1.3.0
} board_hw_rev_t;

/**
 * @brief Initialize general gpios
 *
 * This initializes GPIOs that most modules care about.
 *
 * @return int - nonzero value if any GPIO pin could not be
 *  	   successfully configured, 0 otherwise.
 */
int init_gpios(void);

/**
 * @brief Initialize CAN gpios
 *
 * This initializes GPIOs specific to CAN.
 *
 * @return int - nonzero value if any GPIO pin could not be
 *  	   successfully configured, 0 otherwise.
 */
int init_can_gpios(void);

board_hw_rev_t get_board_hw_rev(void);

int set_can_silent(void);

int set_can_normal(void);

bool get_can_fault(void);

int set_can_shutdown(void);

int set_can_active(void);

bool get_ina226_alert(void);

bool get_cell1_tmp101_alert(void);

bool get_cell2_tmp101_alert(void);

int set_lt1618_enable(bool enable);

#endif

