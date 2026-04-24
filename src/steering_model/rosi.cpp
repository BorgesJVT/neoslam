#include <rosi.hpp>

using namespace rosi_controller;

Rosi::Rosi() :
    BODY_WIDTH_WHEELS(-1),
    BODY_WIDTH_TRACKS(-1),
    WHEELS_RADIUS(-1),
    TRACKS_RADIUS(-1),
    MASS(-1),
    IZZ(-1),
    WHEEL_CURRENT_2_TORQUE(-1),
    ARM_CURRENT_2_TORQUE(-1)
{

}

Rosi::~Rosi()
{

}

double Rosi::setBodyWidthWheels(double value)
{
    BODY_WIDTH_WHEELS = value;
    BODY_WIDTH_WHEELS_2 = BODY_WIDTH_WHEELS / 2.0;
    INV_BODY_WIDTH_WHEELS = 1.0 / BODY_WIDTH_WHEELS;
    return  value;
}

double Rosi::setBodyWidthTracks(double value)
{
    BODY_WIDTH_TRACKS = value;
    BODY_WIDTH_TRACKS_2 = BODY_WIDTH_TRACKS / 2.0;
    INV_BODY_WIDTH_TRACKS = 1.0 / BODY_WIDTH_TRACKS;
    return value;
}

double Rosi::setWheelsRadius(double value)
{
    WHEELS_RADIUS = value;
    WHEELS_RADIUS_2 = WHEELS_RADIUS / 2.0;
    INV_WHEELS_RADIUS = 1.0 / WHEELS_RADIUS;
    return value;
}

double Rosi::setTracksRadius(double value)
{
    TRACKS_RADIUS = value;
    INV_TRACKS_RADIUS = 1.0 / TRACKS_RADIUS;
    return value;
}

double Rosi::setMass(double value)
{
    MASS = value;
    INV_MASS = 1.0 / MASS;
    return value;
}

double Rosi::setIzz(double value)
{
    IZZ = value;
    INV_IZZ = 1.0 / IZZ;
    return value;
}

double Rosi::setWheelCurrent2Torque(double value)
{
    WHEEL_CURRENT_2_TORQUE = value;
    return value;
}

double Rosi::setArmCurrent2Torque(double value)
{
    ARM_CURRENT_2_TORQUE = value;
    return value;
}

void Rosi::robot2Wheels(const std::vector<double> robot_vel, std::vector<double>::iterator wheels_vel)
{
    double aux = BODY_WIDTH_WHEELS_2 * robot_vel[1];

    *wheels_vel = *(wheels_vel + 2) = INV_WHEELS_RADIUS * (robot_vel[0] - aux);
    *(wheels_vel + 1) = *(wheels_vel + 3) = INV_WHEELS_RADIUS * (robot_vel[0] + aux);
}

void Rosi::robot2Tracks(const std::vector<double> robot_vel, std::vector<double>::iterator tracks_vel)
{
    double aux = BODY_WIDTH_TRACKS_2 * robot_vel[1];

    *tracks_vel = *(tracks_vel + 2) = - INV_TRACKS_RADIUS * (robot_vel[0] - aux);
    *(tracks_vel + 1) = *(tracks_vel + 3) = INV_TRACKS_RADIUS * (robot_vel[0] + aux);
}

void Rosi::wheels2Robot(const std::vector<double> wheels_vel, std::vector<double>::iterator robot_vel)
{
    double lwheel_vel = (wheels_vel[0] + wheels_vel[2]) / 2.0;
    double rwheel_vel = (wheels_vel[1] + wheels_vel[3]) / 2.0;

    *robot_vel = WHEELS_RADIUS_2 * (lwheel_vel + rwheel_vel);
    *(robot_vel + 1) = WHEELS_RADIUS * INV_BODY_WIDTH_WHEELS * (rwheel_vel - lwheel_vel);    
}

void Rosi::getOdomVelocity(const std::vector<double> jointVel, double &xLinVel, double &zAngVel)
{
    std::vector<double> aux;
    aux.resize(2);

    wheels2Robot(jointVel, aux.begin());

    xLinVel = aux[0];
    zAngVel = aux[1];
}
