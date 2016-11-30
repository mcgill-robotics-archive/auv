/*
 *==============================================================================
 * Name: Double Integration Pose Header
 * Creator: Jeremy Mallette
 * Last Updated: 30/10/2016
 * Updated By:
 *
 * "For use with doubleIntPose.cpp"
 *
 *==============================================================================
 */

#include <cmath>
using std::abs;

#ifndef DOUBLEINTPOSE_H
#define DOUBLEINTPOSE_H

class vector {
    private:
        double x;
        double y;
        double z;

    public:
        vector() {
             x = 0.0;
             y = 0.0;
             z = 0.0;
        }
        vector(double xIn, double yIn, double zIn) {
            x = xIn;
            y = yIn;
            z = zIn;
        }
        void operator = (const vector &V ) {
            x = V.x;
            y = V.y;
            z = V.z;
        }

        void setVector(float xIn, float yIn, float zIn);
        void integrate(vector curr, vector last, float deltaT);
        float getX();
        float getY();
        float getZ();
};

void vector::setVector(float xIn, float yIn, float zIn) {
        x = xIn;
        y = yIn;
        z = zIn;
    }

float calcAbsArea(float point_1, float point_2, float time) {
    float absArea;
    float triArea = abs(point_1 - point_2) / 2;

    if(abs(point_1) > abs(point_2) || (abs(point_1) == abs(point_2))) {
        absArea = (triArea * time) + abs(point_2 * time);
    } else {
        absArea = (triArea * time) + abs(point_1 * time);
    }

    return absArea;
}

float calcCrossArea(float point_1, float point_2, float time) {
    float crossArea;

    crossArea = ((point_1 - point_2) / 2) * time;

    return crossArea;
}

void vector::integrate(vector curr, vector last, float deltaT) {
    if((curr.x >= 0.0) && (last.x >= 0.0)) {
        x += calcAbsArea(curr.x, last.x, deltaT);
    } else if((curr.x <= 0.0) && (last.x <= 0.0)) {
        x -= (calcAbsArea(curr.x, last.x, deltaT));
    } else {
        x += calcCrossArea(curr.x, last.x, deltaT);
    }

    if((curr.y >= 0.0) && (last.y >= 0.0)) {
        y += calcAbsArea(curr.y, last.y, deltaT);
    } else if((curr.y <= 0.0) && (last.y <= 0.0)) {
        y -= calcAbsArea(curr.y, last.y, deltaT);
    } else {
        y += calcCrossArea(curr.y, last.y, deltaT);
    }

    if((curr.z >= 0.0) && (last.z >= 0.0)) {
        z += calcAbsArea(curr.z, last.z, deltaT);
    } else if((curr.z <= 0.0) && (last.z <= 0.0)) {
        z -= calcAbsArea(curr.z, last.z, deltaT);
    } else {
        z += calcCrossArea(curr.z, last.z, deltaT);
    }
}

float vector::getX() {
    return x;
}
float vector::getY() {
    return y;
}
float vector::getZ() {
    return z;
}

#endif
