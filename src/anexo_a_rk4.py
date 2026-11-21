"""Annex A of the PMED final report: RK4 integrator for the differential-drive
kinematic model.

The method body below is transcribed verbatim from Annex A of the report
(docs/informe-pmed-ekf-odometria.pdf, pp. 15-16). The report only shows the
method; the class that contained it (the `kinematic_sim_node` ROS 2 node) is
NOT part of the annex. The `KinematicSimNode` shell here exists only so the
method can be imported without ROS 2; it is not code from the report.
"""
import numpy as np


class KinematicSimNode:  # shell added for importability; the real node is not in the annex
    def runge_kutta_step(self, x, y, theta, v, w, dt):
        """
        Calcula el siguiente estado usando RK4 para el modelo
        cinemático diferencial.
        Estado: [x, y, theta]
        Entradas: [v (lineal), w (angular)]
        """
        # k1
        k1_x = v * np.cos(theta)
        k1_y = v * np.sin(theta)
        k1_th = w

        # k2 (mitad de paso)
        theta_k2 = theta + k1_th * (dt / 2.0)
        k2_x = v * np.cos(theta_k2)
        k2_y = v * np.sin(theta_k2)
        k2_th = w

        # k3 (mitad de paso mejorado)
        theta_k3 = theta + k2_th * (dt / 2.0)
        k3_x = v * np.cos(theta_k3)
        k3_y = v * np.sin(theta_k3)
        k3_th = w

        # k4 (paso completo)
        theta_k4 = theta + k3_th * dt
        k4_x = v * np.cos(theta_k4)
        k4_y = v * np.sin(theta_k4)
        k4_th = w

        # Actualización final
        new_x = x + (dt / 6.0) * (k1_x + 2*k2_x + 2*k3_x + k4_x)
        new_y = y + (dt / 6.0) * (k1_y + 2*k2_y + 2*k3_y + k4_y)
        new_theta = theta + (dt / 6.0) * (k1_th + 2*k2_th + 2*k3_th
                                           + k4_th)

        # Normalización del ángulo entre -pi y pi
        new_theta = np.arctan2(np.sin(new_theta), np.cos(new_theta))

        return new_x, new_y, new_theta
