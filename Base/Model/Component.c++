#include "Component.h"

void Component::UpdateComponentRelativeVectors()
{
    Quaterniond ComponentQuat = Quaterniond(Coordinates[3], Coordinates[4], Coordinates[5], Coordinates[6]);
    Matrix3d RotationMat = ComponentQuat.toRotationMatrix();

    Vector3d LocalUpVec(0.0, 1.0, 0.0);
    Vector3d LocalForwardVec(0.0, 0.0, 1.0);
    Vector3d LocalRightVec(1.0, 0.0, 0.0);

    UpVector = RotationMat*LocalUpVec;
    ForwardVector = RotationMat*LocalForwardVec;
    RightVector = RotationMat*LocalRightVec;
}

void Component::GetIndependentEulerJacobian(int TotalComponentNb)
{
    for (int i = 0; i < IndependentCoords.size(); i++)
    {
        if (IndependentCoords[i] == 3)
        {
            double Derivative1 = (-sin(EulerAngles[0])*cos(EulerAngles[2]) - cos(EulerAngles[0])*cos(EulerAngles[1])*sin(EulerAngles[2]))*EulerAngles[0] + 
                            (sin(EulerAngles[0])*sin(EulerAngles[2]) - cos(EulerAngles[0])*cos(EulerAngles[1])*cos(EulerAngles[2]))*EulerAngles[1] +
                            (cos(EulerAngles[0])*sin(EulerAngles[1]))*EulerAngles[2];

            double Derivative2 = (sin(EulerAngles[0])*sin(EulerAngles[1])*sin(EulerAngles[2]))*EulerAngles[0] +
                                    (sin(EulerAngles[0])*sin(EulerAngles[1])*cos(EulerAngles[2]))*EulerAngles[1] +
                                    (sin(EulerAngles[0])*cos(EulerAngles[1]))*EulerAngles[2];

            double Derivative3 = (-cos(EulerAngles[0])*sin(EulerAngles[2]) - sin(EulerAngles[0])*cos(EulerAngles[1])*cos(EulerAngles[2]))*EulerAngles[0] +
                                    (-cos(EulerAngles[0])*cos(EulerAngles[2]) + sin(EulerAngles[0])*cos(EulerAngles[1])*sin(EulerAngles[2]))*EulerAngles[1];

            
        }
        else if (IndependentCoords[i] == 4)
        {
            double Derivative1 = (cos(EulerAngles[0])*cos(EulerAngles[2]) - sin(EulerAngles[0])*cos(EulerAngles[1])*sin(EulerAngles[2]))*EulerAngles[0] +
                            (-cos(EulerAngles[0])*sin(EulerAngles[2]) - sin(EulerAngles[0])*cos(EulerAngles[1])*cos(EulerAngles[2]))*EulerAngles[1] +
                            (sin(EulerAngles[0])*sin(EulerAngles[1]))*EulerAngles[2];

            double Derivative2 = (-cos(EulerAngles[0])*sin(EulerAngles[1])*sin(EulerAngles[2]))*EulerAngles[0] +
                                    (-cos(EulerAngles[0])*sin(EulerAngles[1])*cos(EulerAngles[2]))*EulerAngles[1] +
                                    (-cos(EulerAngles[0])*cos(EulerAngles[1]))*EulerAngles[2];

            double Derivative3 = (-sin(EulerAngles[0])*sin(EulerAngles[1]) + cos)
        }
    }
}
