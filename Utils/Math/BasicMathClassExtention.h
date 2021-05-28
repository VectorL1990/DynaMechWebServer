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

    Vec3d(double x, double y, double z)
    {
        X = x;
        Y = y;
        Z = z;
    }

    Vec3d operator-(Vec3d B)
    {
        return Vec3d(X-B.X, Y-B.Y, Z-B.Z); 
    }
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