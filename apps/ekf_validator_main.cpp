#include "rally/core/ekf_ball_predictor.hpp"
#include <mujoco/mujoco.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <random>

using namespace rally::core;

int main(int, char**) {
    char error[1000];
    mjModel* m = mj_loadXML("assets/ball_projectile.xml", nullptr, error, 1000);
    if (!m) return 1;
    mjData* d = mj_makeData(m);

    std::ofstream file("landing_results.csv");
    file << "throw_id,true_x,true_y,pred_x,pred_y,error\n";

    std::mt19937 rng(42);
    std::uniform_real_distribution<double> vel_x_dist(2.0, 6.0);
    std::uniform_real_distribution<double> vel_y_dist(-2.0, 2.0);
    std::uniform_real_distribution<double> vel_z_dist(3.0, 7.0);

    std::cout << "Simulating 50 trajectories...\n";

    for (int i = 0; i < 50; ++i) {
        mj_resetData(m, d);
        
        // Randomize initial throw
        double vx = vel_x_dist(rng);
        double vy = vel_y_dist(rng);
        double vz = vel_z_dist(rng);
        
        d->qpos[0] = 0.0; d->qpos[1] = 0.0; d->qpos[2] = 1.0;
        d->qvel[0] = vx;  d->qvel[1] = vy;  d->qvel[2] = vz;

        EkfBallPredictor ekf(0.002, 0.01);
        ekf.initialize_state(Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(vx, vy, vz));

        Eigen::Vector2d final_prediction(0, 0);

        // Run until impact
        while (d->qpos[2] > 0.05 && d->time < 5.0) {
            mj_step(m, d);
            Eigen::Vector3d measured_pos(d->qpos[0], d->qpos[1], d->qpos[2]);
            
            // Add synthetic sensor noise here if desired
            
            ekf.predict();
            ekf.update(measured_pos);
            final_prediction = ekf.predict_landing_point();
        }

        double error_margin = std::sqrt(std::pow(final_prediction(0) - d->qpos[0], 2) + 
                                        std::pow(final_prediction(1) - d->qpos[1], 2));

        file << i << "," << d->qpos[0] << "," << d->qpos[1] << "," 
             << final_prediction(0) << "," << final_prediction(1) << "," << error_margin << "\n";
    }

    std::cout << "Done! Results saved to landing_results.csv\n";
    file.close();
    mj_deleteData(d);
    mj_deleteModel(m);
    return 0;
}