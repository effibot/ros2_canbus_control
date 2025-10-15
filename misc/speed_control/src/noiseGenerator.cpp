#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int32.hpp"
#include <cmath>
#include <fstream>
#include <iostream>

using namespace std::chrono_literals;

class NoiseGeneratorNode : public rclcpp::Node
{
public:
    NoiseGeneratorNode()
        : Node("noise_generator_node")
    {

        // Dichiarazione dei parametri
        this->declare_parameter<std::vector<double>>("freq_noise", {0.0, 0.0});
        this->declare_parameter<std::vector<double>>("ampl_noise", {0.0, 0.0});
        this->declare_parameter<std::vector<double>>("switch_times");
        this->declare_parameter<double>("dt", 1); // valore di default impostato a 1s

        // Lettura dei parametri dal file YAML
        freq_noise_arr_ = this->get_parameter("freq_noise").as_double_array();
        ampl_noise_arr_ = this->get_parameter("ampl_noise").as_double_array();
        switch_times_arr_ = this->get_parameter("switch_times").as_double_array();

        omega = 2.0 * M_PI * freq_noise_arr_[nu_];

        // x0v_ = {ampl_noise_arr_[nu_ - 1], 0.0}; // Coseno
        x0v_ = {0.0, ampl_noise_arr_[nu_] * omega};

        dt_ = this->get_parameter("dt").as_double();

        xv_ = x0v_;

        xv_old = {ampl_noise_arr_[nu_] * sin(-omega * dt_), -ampl_noise_arr_[nu_] * omega * cos(-omega * dt_)};

        publisher_ = this->create_publisher<std_msgs::msg::Float64>("/motor1/reference_speed_sub", 10);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(dt_ * 1000)), std::bind(&NoiseGeneratorNode::change_noise, this));

        // Registra il tempo iniziale
        start_time_ = this->get_clock()->now();
    }

private:
    void input_callback()
    {
        // Verlet integration:
        double x1 = xv_[0]; // Posizione corrente

        double x1_old = xv_old[0]; // Posizione precedente

        // Metodo di Verlet per la posizione
        double x1_new = 2.0 * x1 - x1_old - omega * omega * x1 * dt_ * dt_;

        // Calcolo della nuova velocità (usando la differenza centrale)
        double x2_new = (x1_new - x1_old) / (2.0 * dt_);

        // Aggiorna stato e stato precedente
        xv_old[0] = x1;
        xv_[0] = x1_new;
        xv_[1] = x2_new;

        double yv_ = Cv_[0] * xv_[0] + Cv_[1] * xv_[1];

        // Pubblica l'uscita
        auto message = std_msgs::msg::Float64();
        message.data = yv_;
        publisher_->publish(message);

        printf("Reference:  %.3f\n", message.data);
    }

    void change_noise()
    {

        // Scrivi il tempo e l'output nel file CSV
        auto now = this->get_clock()->now();

        auto elapsed_time = now - start_time_;

        double elapsed_seconds = elapsed_time.seconds();

        int switch_phase = -1;

        for (int i = 0; i < switch_times_arr_.size(); ++i)
        {
            if (elapsed_seconds < switch_times_arr_[i])
            {
                switch_phase = i;
                break;
            }
        }

        if (switch_phase == -1) {
            switch_phase = nu_;
            xv_[0] = 0;
            xv_[1] = 0;
        }

        if (switch_phase != nu_)
        {
            printf("CAMBIO\n");
            nu_ = switch_phase;
            omega = 2.0 * M_PI * freq_noise_arr_[nu_];
            x0v_ = {0.0, ampl_noise_arr_[nu_] * omega};
            xv_old = {ampl_noise_arr_[nu_] * sin(-omega * dt_), -ampl_noise_arr_[nu_] * omega * cos(-omega * dt_)};
            xv_[0] = x0v_[0];
            xv_[1] = x0v_[1];
        }

        printf("Phase: %d, Time: %f\n", switch_phase, elapsed_seconds);

        input_callback();
    }

    std::vector<double> freq_noise_arr_;
    std::vector<double> ampl_noise_arr_;
    std::vector<double> switch_times_arr_;
    int nu_ = 0;

    double pi_ = 3.14;
    double omega;

    std::vector<double> Cv_ = {1.0, 0.0}; // Vettore C dello spazio di stato

    std::vector<double> x0v_; // Vettore C dello spazio di stato
    std::vector<double> xv_;  // Vettore di stato [y, dy/dt]
    std::vector<double> xv_old;
    double dt_; // Passo di simulazione

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Time start_time_; // Tempo iniziale
};

int main(int argc, char *argv[])
{

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NoiseGeneratorNode>());
    rclcpp::shutdown();

    return 0;
}
