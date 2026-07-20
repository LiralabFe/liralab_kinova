#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>

/*

Disco bianco + sonda = 320
Disco nero = 92
Sensore + maniglia = 220

*/

struct Wrench
{
    Eigen::Vector3d force;   // [N]
    Eigen::Vector3d torque;  // [Nm]
};

Wrench compensatePayloadGravity(
    const Wrench& measured,
    double mass,
    const Eigen::Vector3d& com_sensor,          // CoM rispetto al sensore, nel frame sensore [m]
    const Eigen::Isometry3d& T_world_ee,        // posa EE rispetto al mondo/base
    const Eigen::Isometry3d& T_ee_sensor,       // posa sensore rispetto all'EE
    bool subtract_gravity = true)
{
    Eigen::Isometry3d T_world_sensor = T_world_ee * T_ee_sensor;
    Eigen::Matrix3d R_world_sensor = T_world_sensor.rotation();

    Eigen::Vector3d g_world(0.0, 0.0, -9.81);

    // Gravità espressa nel frame del sensore
    Eigen::Vector3d g_sensor = R_world_sensor.transpose() * g_world;

    // Wrench dovuto al peso del carico
    Eigen::Vector3d Fg = mass * g_sensor;
    Eigen::Vector3d Tg = com_sensor.cross(Fg);

    Wrench compensated;

    if (subtract_gravity)
    {
        compensated.force  = measured.force  - Fg;
        compensated.torque = measured.torque - Tg;
    }
    else
    {
        compensated.force  = measured.force  + Fg;
        compensated.torque = measured.torque + Tg;
    }

    return compensated;
}

int main()
{
    // -----------------------------
    // 1. Wrench misurato dal sensore
    // -----------------------------
    Wrench measured;
    measured.force  << 0.2, -0.1, -7.4;      // [N]
    measured.torque << 0.01, -0.05, 0.02;    // [Nm]

    // -----------------------------
    // 2. Massa e baricentro carico
    // -----------------------------
    double mass = 0.75; // [kg]

    // CoM del carico rispetto all'origine del sensore, espresso nel frame sensore
    Eigen::Vector3d com_sensor;
    com_sensor << 0.0, 0.0, -0.08; // [m]

    // -----------------------------
    // 3. Posa EE rispetto al mondo/base
    // -----------------------------
    Eigen::Isometry3d T_world_ee = Eigen::Isometry3d::Identity();

    // Esempio: posizione EE
    T_world_ee.translation() << 0.4, 0.1, 0.3;

    // Esempio: orientamento EE
    T_world_ee.linear() =
        Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()).toRotationMatrix();

    // -----------------------------
    // 4. Posa sensore rispetto all'EE
    // -----------------------------
    Eigen::Isometry3d T_ee_sensor = Eigen::Isometry3d::Identity();

    // Esempio: sensore 5 cm sotto l'EE
    T_ee_sensor.translation() << 0.0, 0.0, -0.05;

    // Se il frame sensore è allineato al frame EE:
    T_ee_sensor.linear() = Eigen::Matrix3d::Identity();

    // Se invece è ruotato, esempio rotazione di 180° attorno a X:
    /*
    T_ee_sensor.linear() =
        Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();
    */

    // -----------------------------
    // 5. Compensazione
    // -----------------------------
    Wrench compensated = compensatePayloadGravity(
        measured,
        mass,
        com_sensor,
        T_world_ee,
        T_ee_sensor,
        true
    );

    std::cout << "Force compensated  [N]:  "
              << compensated.force.transpose() << std::endl;

    std::cout << "Torque compensated [Nm]: "
              << compensated.torque.transpose() << std::endl;

    return 0;
}