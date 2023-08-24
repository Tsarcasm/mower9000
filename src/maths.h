#include <Arduino.h>

#define RAD_TO_DEG 57.295779513082320876798154814105


struct euler_t {
    float yaw;
    float pitch;
    float roll;
};
euler_t quaternionToEuler(float qr, float qi, float qj, float qk, bool degrees = false) {
    euler_t ypr;
    float sqr = sq(qr);
    float sqi = sq(qi);
    float sqj = sq(qj);
    float sqk = sq(qk);

    ypr.yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    ypr.pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    ypr.roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

    if (degrees) {
        ypr.yaw *= RAD_TO_DEG;
        ypr.pitch *= RAD_TO_DEG;
        ypr.roll *= RAD_TO_DEG;
    }
    return ypr;
}



struct point {
    float x;
    float y;
};

struct line {
    point A;
    point B;
};

float calc_bearing(line AB) {
    float dx = AB.B.x - AB.A.x;
    float dy = AB.B.y - AB.A.y;
    float bearing = atan2(dy, dx) * RAD_TO_DEG;
    return bearing;
}

// Returns the 
float approach_angle_calc(line AB, point C, float angle) {
    // Determine which side of the line we are on
    float bearing_AB = calc_bearing(AB);
    float bearing_AC = calc_bearing({AB.A, C});
    float bearing_diff = bearing_AB - bearing_AC;
    float sign = 0;
    if (bearing_diff > 0 || bearing_diff < -180) {
        sign = 1;
    } else {
        sign = -1;
    }
    float approach_angle = bearing_AB + (sign * angle);
    return approach_angle;
}

float gradient(line AB) {
    // Handle the case where AB.A.x - AB.B.x is 0
    if (AB.A.x == AB.B.x) {
        return std::numeric_limits<float>::infinity();  // Return infinity to signify vertical line
    }
    return (AB.B.y - AB.A.y) / (AB.B.x - AB.A.x);
}

point perpendicular_intersection(line AB, point C) {
    point D = {0, 0};
    const float m = gradient(AB);

    // Check for vertical line
    if (std::isinf(m)) {
        D.x = AB.A.x;  // or AB.B.x, since they are the same for a vertical line
        D.y = C.y;
        return D;
    }

    // Check for horizontal line
    if (m == 0) {
        D.x = C.x;
        D.y = AB.A.y;  // or AB.B.y, since they are the same for a horizontal line
        return D;
    }

    // For all other cases
    const float m_perp = -1 / m;

    D.x = (m_perp * C.x - m_perp * AB.A.x + AB.A.y - C.y) / (m_perp - m);
    D.y = m * D.x - m * AB.A.x + AB.A.y;

    return D;
}

bool has_passed_B(line AB, point C) {
    const float m = gradient(AB);
    if (std::isinf(m)) {
        // Vertical line
        // If X_a > X_b, > X_c
        // Else X_a < X_b < X_c
        if (AB.A.x > AB.B.x) {
            return C.x < AB.B.x;
        } else {
            return C.x > AB.B.x;
        }
    } else if (m == 0) {
        // Horizontal line
        if (AB.A.y > AB.B.y) {
            return C.y < AB.B.y;
        } else {
            return C.y > AB.B.y;
        }
    }

    const float m_perp = -1 / m;
    const float side_a = m_perp * (AB.A.x - AB.B.x) - AB.A.y + AB.B.y;
    const float side_b = m_perp * (C.x - AB.B.x) - C.y + AB.B.y;
    // Return true if the sign of side_a and side_b are different
    return side_a * side_b < 0;
}