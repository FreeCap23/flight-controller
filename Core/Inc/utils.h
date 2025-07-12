/**
  ******************************************************************************
  * @file           : utils.h
  * @author         : Dionisie Stratulat
  * @brief          : TODO
  ******************************************************************************
  */

#ifndef INC_UTILS_H_
#define INC_UTILS_H_

/**
 * @brief Maps a value from one range to another using linear interpolation.
 *
 *        Example: map 5 from [0, 10] to [0, 100] → returns 50
 *
 * @param in_min  The lower bound of the input range.
 * @param in_max  The upper bound of the input range.
 * @param out_min The lower bound of the output range.
 * @param out_max The upper bound of the output range.
 * @param value   The input value to map.
 * @return float  The mapped output value.
 */
static inline float map_range_float(float in_min, float in_max,
                                    float out_min, float out_max,
                                    float value) {
    if (in_max == in_min) {
        return out_min;  // Prevent division by zero
    }

    float t = (value - in_min) / (in_max - in_min);
    return out_min + t * (out_max - out_min);
}

/**
 * @brief Returns the absolute value of a float.
 *
 * @param x The input float.
 * @return The absolute value of x.
 */
static inline float absf(float x) {
    return (x < 0.0f) ? -x : x;
}

/**
 * @brief Returns the absolute value of an integer.
 *
 * @param x The input integer.
 * @return The absolute value of x.
 */
static inline int abs(int x) {
    return (x < 0) ? -x : x;
}

#endif /* INC_UTILS_H_ */
