#pragma once

struct Vec3f
{
    float X;
    float Y;
    float Z;
};

struct Vec3d
{
    double X;
    double Y;
    double Z;
};

/**
 * There are various coordinate systems such as global coordinate system, local coordinate syste and so no
 */
struct Coordinate
{
    // Base vector represent e1, e2, e3 in global coordinate system, length always 1
    Vec3d E;

    // Coordinate origin in global coordinate system
    Vec3d Origin;
};