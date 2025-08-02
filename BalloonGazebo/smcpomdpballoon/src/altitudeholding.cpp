#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <vector>
#include <algorithm>
#include <cmath>

class SPBallon : public rclcpp::Node {
public:
    SPBallon(): Node("SPBallon"), 
    fx(0), fy(0), fh(4750), x(0), y(0), h(0), vx(0), vy(0), vh(0), 
    lambda(0.125), K(0.1), Ks(0.05), m(0.55), gamma(0.25), epsilonsmc(0.05), Dh(0),
    m_f(0.1), c_f(1550), alpha(0.075), pi(3.1415), sigma(5.67037442e-8), epsilon(0.1), T_ear(288), tau_IR_ear(0.95),
    c_V_He(12.75), c_P_He(20.25), I_D(0), I_S(0), I_R(0), h_ex(0), h_in(0), m_He_d(0), V_d(0), m_He_pre(0.0775), V_pre(1.5),
    R_He(2077), d(0.1 / 1000), E(0.2e13), R_0(12), C_d(0.8), S(0), R_m_atm(287),
    m_He(0.0775), P_He(101325), V(1.5), R(12), P_atm(101325), T_atm(288), vr(0), vwx(0.1), vwy(0.1), vwh(0.1), B(0), D(0), rho_atm(1.29), T_f(298), T_He(297),
    Nu(8), k_atm(0.026), L_0(12), Re(288000), k_He(0.0015), rho_He(0.1786), Pr_He(0.67), mu_He(1.1e-4), g(9.80),
    P_0(101325), I_0(1367), e_e(0.016708), r_e(0.3), start_hour(-2), latitude(0), day(0), clock(0),
    pretime_(this->get_clock()->now()),  initial_time(this->get_clock()->now()) {
        forces_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/forces", 10);
        state_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/droneposition/odom", 10, std::bind(&SPBallon::stateCallback, this, std::placeholders::_1));
    }
private:
    void stateCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        rclcpp::Time timenow = this->get_clock()->now();
        double dt = (timenow - pretime_).seconds();
        pretime_ = timenow;
        clock = (timenow - initial_time).seconds();

        const auto &position = msg->pose.pose.position;
        const auto &linear_velocity = msg->twist.twist.linear;

        x = position.x;
        y = position.y;
        h = position.z + 0.0001;
        vx = linear_velocity.x;
        vy = linear_velocity.y;
        vh = linear_velocity.z;

        double start_second = start_hour * 3600;
        double hour = ((start_second + clock) / (24 * 3600) - std::round((start_second + clock) / (24 * 3600))) * 24 + 12;
        day = std::fmod(day + std::round(clock / (24 * 3600)), 365);
        double w = 0.2618 * (hour - 12);
        double delta = 0.4093 * (2 * pi * (284 + day) / 365);
        double theta_ele = std::asin(std::sin(latitude) * std::sin(delta) + std::cos(w) * std::cos(latitude) * std::cos(delta));
        double lambda_e = 2 * pi * (day - 1) / 365;
        double lambda_am = P_atm / P_0 * (std::sqrt(1228.6 + std::pow(613.8 * std::sin(w), 2)) - 613.8 * std::sin(w));

        double tau_atm = (std::exp(-0.65 * lambda_am) + std::exp(-0.095 * lambda_am)) * (1 + 0.4 * std::pow(P_atm / P_0, 2)) / 2;
        double tau_sc = 0.2 * std::exp(h / 25000) * std::sin(w);

        I_D = tau_atm * I_0 * std::pow((1 + e_e * std::cos(lambda_e)) / (1 - std::pow(e_e, 2)), 2);
        I_S = tau_sc * I_D;
        I_R = r_e * (I_D * std::sin(theta_ele) + I_S);

        T_atm = calculate_atmospheric_temperature(h);
        P_atm = calculate_atmospheric_pressure(h);

        if (V <= 0) {
            RCLCPP_ERROR(this->get_logger(), "Volume must be greater than zero.");
            throw std::invalid_argument("Volume must be greater than zero.");
        }
        
        if (V >= 2.25) {
            P_He = (m_He * R_He * T_He) / V;
        } else {
            P_He = P_atm;
        }
        
        double A_projected = pi * std::pow(R, 2); 
        double A_surf = 4 * pi * std::pow(R, 2);  

        double q_D = alpha * I_D * A_projected;
        double q_S = alpha * I_S * A_surf;
        double q_R = alpha * I_R * A_projected;

        double I_IR_atm = epsilon * sigma * (std::pow(T_atm, 4) - std::pow(T_f, 4)); 
        double I_IR_ear = tau_IR_ear * epsilon * sigma * (std::pow(T_ear, 4) - std::pow(T_f, 4)); 
        double I_IR_He = epsilon * sigma * (std::pow(T_He, 4) - std::pow(T_f, 4)); 

        double q_IR_atm = I_IR_atm * A_surf;
        double q_IR_ear = I_IR_ear * A_projected;
        double q_IR_He = I_IR_He * A_surf;

        double h_natural_ex = (Nu * k_atm) / L_0;  
        double h_forced_ex = (2 + 0.41 * std::pow(Re, 0.55)) * k_atm / L_0;
        h_ex = std::pow(std::pow(h_natural_ex, 3) + std::pow(h_forced_ex, 3), 1.0 / 3.0);
        h_in = 0.13 * k_He * std::pow((std::pow(rho_He, 2) * g * std::abs(T_f - T_He) * Pr_He) / (T_He * std::pow(mu_He, 2)), 1.0 / 3.0);

        double q_conv_ex = h_ex * A_surf * (T_atm - T_f);
        double q_conv_in = h_in * A_surf * (T_He - T_f);

        V_d = (V - V_pre) / dt;
        m_He_d = (m_He - m_He_pre ) / dt;
        double dT_f = (q_D + q_S + q_R + q_IR_atm + q_IR_ear + q_IR_He + q_conv_ex + q_conv_in) / (m_f * c_f);
        double dT_He = -q_conv_in / (c_V_He * m_He) + (c_P_He / c_V_He - 1) * T_He * (m_He_d / m_He - V_d / V);

        T_f = T_f + dT_f * dt;
        T_He = T_He + dT_He * dt;
        
        V_pre = V;
        V = (m_He * R_He * T_He) / P_He; 

        double deltaP = P_He - P_atm;

        double sigma_f = (deltaP == 0) ? 0 : (deltaP * R_0) / (2 * d); 
        double epsilon_f = sigma_f / E; 

        R = (1 + epsilon_f) * R_0;

        if (V >= 2.25) {
            V = (4.0 / 3.0) * pi * std::pow(R, 3);
        }

        S = 2 * pi * std::pow(R, 2); 
        rho_atm = P_atm / (R_m_atm * T_atm);
        double madd = 0.5 * rho_atm * V;
        double hr = 10.0;
        double s = vh + lambda * (h - hr);
        double sats;
        if (std::abs(s) > epsilonsmc) {
            sats = std::copysign(1.0, s);  
        } else {
            sats = s / epsilonsmc;
        }
        double dotDh = gamma * s;
        Dh += dotDh * dt;
        double fz = (-Ks * sats - K * s - lambda * vh) * (m + madd) - Dh - (m_He * R_He * T_He * g) / (R_m_atm * T_atm) + m * g;
        B = rho_atm * g * V;
        D = 0.5 * rho_atm * std::pow(vr,2) * C_d * S;
        std_msgs::msg::Float64MultiArray forces_msg;
        forces_msg.data = {0, 0, B + fz};
        forces_pub_->publish(forces_msg);

    }

    double calculate_atmospheric_temperature(double h) {
        double T_atm;
        if (h >= 0 && h <= 11000) {
            T_atm = 288 - 0.0065 * h;
        } else if (h > 11000 && h <= 20000) {
            T_atm = 216.65;
        } else if (h > 20000 && h <= 32000) {
            T_atm = 216.65 + 0.0010 * (h - 20000);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Altitude h must be between 0 and 32000 meters.");
            throw std::invalid_argument("Altitude h must be between 0 and 32000 meters.");
        }
        return T_atm;
    }

    double calculate_atmospheric_pressure(double h) {
        double P_atm;
        if (h >= 0 && h <= 11000) {
            P_atm = 101325 * std::pow((288 - 0.0065 * h) / 288, 5.25);
        } else if (h > 11000 && h <= 20000) {
            P_atm = 22632 * std::exp(-(h - 11000) / 6341.62);
        } else if (h > 20000 && h <= 32000) {
            P_atm = 5474.87 * std::pow((216.65 + 0.0010 * (h - 20000)) / 216.65, -34.163);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Altitude h must be between 0 and 32000 meters.");
            throw std::invalid_argument("Altitude h must be between 0 and 32000 meters.");
        }
        return P_atm;
    }

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr forces_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_sub_;    
    rclcpp::Time pretime_, initial_time;
    double fx, fy, fh;
    double x, y, h, vx, vy, vh;
    double lambda, K, Ks, m, gamma, epsilonsmc, Dh;
    double m_He, P_He, V, R, P_atm, T_atm, vr, vwx, vwy, vwh, B, D, rho_atm, T_f, T_He, m_f, c_f, alpha, pi, sigma, epsilon, T_ear, tau_IR_ear;
    double c_V_He, c_P_He, I_D, I_S, I_R, h_ex, h_in, m_He_d, V_d, m_He_pre, V_pre;
    double Nu, k_atm, L_0, Re, k_He, rho_He, Pr_He, mu_He, g; 
    double P_0, I_0, e_e, r_e, start_hour, latitude, day, clock; 
    double R_He, d, E, R_0;
    double C_d, S, R_m_atm;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SPBallon>());
    rclcpp::shutdown();
    return 0;
}