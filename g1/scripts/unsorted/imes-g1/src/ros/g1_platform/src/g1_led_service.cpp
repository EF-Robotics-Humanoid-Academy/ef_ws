/*
@author    Salman Omar Sohail <support@mybotshop.de>
@copyright (c) 2025, MYBOTSHOP GmbH

[License text same as before...]
*/

#include <memory>
#include <string>
#include <algorithm>
#include <cctype>
#include <sstream>
#include <vector>
#include <unordered_map>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "g1_interface/srv/g1_modes.hpp"

#include "unitree/robot/channel/channel_factory.hpp"
#include "unitree/robot/g1/audio/g1_audio_client.hpp"   // AudioClient::LedControl(r,g,b)

namespace {

// ---------- small string helpers ----------
inline std::string trim(const std::string &s) {
    auto b = std::find_if_not(s.begin(), s.end(), ::isspace);
    auto e = std::find_if_not(s.rbegin(), s.rend(), ::isspace).base();
    return (b < e) ? std::string(b, e) : std::string();
}

inline void split_kv_pairs(const std::string &in,
                           std::vector<std::pair<std::string, std::string>> &out) {
    std::stringstream ss(in);
    std::string token;
    while (std::getline(ss, token, ';')) {
        auto eq = token.find('=');
        std::string k = trim(eq == std::string::npos ? token : token.substr(0, eq));
        std::string v = trim(eq == std::string::npos ? "" : token.substr(eq + 1));
        std::transform(k.begin(), k.end(), k.begin(),
                       [](unsigned char c){ return std::tolower(c); });
        if (!k.empty()) out.emplace_back(k, v);
    }
}

inline bool iequals(const std::string &a, const std::string &b) {
    if (a.size() != b.size()) return false;
    for (size_t i = 0; i < a.size(); ++i)
        if (std::tolower(static_cast<unsigned char>(a[i])) !=
            std::tolower(static_cast<unsigned char>(b[i]))) return false;
    return true;
}

// ---------- color parsing ----------
struct RGB { uint8_t r{0}, g{0}, b{0}; };

inline bool parse_hex2(const std::string &s, size_t pos, uint8_t &out){
    if (pos + 2 > s.size()) return false;
    unsigned int v = 0;
    std::istringstream iss(s.substr(pos, 2));
    iss >> std::hex >> v;
    if (iss.fail() || v > 255) return false;
    out = static_cast<uint8_t>(v);
    return true;
}

// Supports: "#RRGGBB", "rgb(r,g,b)", or a small set of named colors.
inline bool parse_color_string(const std::string &in, RGB &rgb) {
    static const std::unordered_map<std::string, RGB> kNamed {
        {"red",{255,0,0}}, {"green",{0,255,0}}, {"blue",{0,0,255}},
        {"white",{255,255,255}}, {"black",{0,0,0}}, {"yellow",{255,255,0}},
        {"cyan",{0,255,255}}, {"magenta",{255,0,255}}, {"orange",{255,165,0}},
        {"purple",{128,0,128}}, {"pink",{255,192,203}}
    };

    std::string s = trim(in);
    if (s.empty()) return false;

    // named
    auto it = kNamed.find(s);
    if (it != kNamed.end()) { rgb = it->second; return true; }

    // #RRGGBB
    if (s.size() == 7 && s[0] == '#') {
        uint8_t r,g,b;
        if (parse_hex2(s,1,r) && parse_hex2(s,3,g) && parse_hex2(s,5,b)) {
            rgb = {r,g,b}; return true;
        }
        return false;
    }

    // rgb(r,g,b)
    if (s.size() >= 10 && std::equal(s.begin(), s.begin()+4, "rgb(") && s.back()==')') {
        std::string inside = s.substr(4, s.size()-5);
        std::stringstream ss(inside);
        std::string tok; int vals[3] = {0,0,0}; int i=0;
        while (std::getline(ss, tok, ',') && i<3) {
            tok = trim(tok);
            if (tok.empty()) return false;
            int v = std::stoi(tok);
            if (v < 0) v = 0; if (v > 255) v = 255;
            vals[i++] = v;
        }
        if (i == 3) { rgb = {static_cast<uint8_t>(vals[0]),
                             static_cast<uint8_t>(vals[1]),
                             static_cast<uint8_t>(vals[2])}; return true; }
        return false;
    }
    return false;
}

inline RGB scale_rgb(const RGB &c, int brightness_percent) {
    double k = std::max(0, std::min(100, brightness_percent)) / 100.0;
    auto sc = [&](uint8_t u) -> uint8_t {
        int v = static_cast<int>(std::round(u * k));
        return static_cast<uint8_t>(std::max(0, std::min(255, v)));
    };
    return { sc(c.r), sc(c.g), sc(c.b) };
}

} // namespace

class G1LedServiceNode : public rclcpp::Node {
public:
    G1LedServiceNode() : Node("g1_led_service") {
        declare_parameter<std::string>("eth", "none");
        eth_ = get_parameter("eth").as_string();
        RCLCPP_INFO(get_logger(), "Using Ethernet interface: %s", eth_.c_str());

        unitree::robot::ChannelFactory::Instance()->Init(0, eth_);

        audio_ = std::make_shared<unitree::robot::g1::AudioClient>();
        audio_->Init();
        audio_->SetTimeout(10.0f);

        // Defaults
        current_color_      = {0x00, 0xFF, 0x00}; // #00ff00
        current_brightness_ = 100;                // percent

        // ---- set startup color to #00ff00 ----
        apply_led();

        srv_ = create_service<g1_interface::srv::G1Modes>(
            "hardware/led",
            std::bind(&G1LedServiceNode::handle, this,
                      std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(get_logger(),
            "Service ready: /hardware/led (G1Modes)\n"
            "Commands:\n"
            "  color=<#RRGGBB|rgb(r,g,b)|name>\n"
            "  brightness=<0..100>\n"
            "  get_color; get_brightness");
    }

private:
    // Apply current_color_ and current_brightness_ via AudioClient::LedControl
    void apply_led() {
        RGB out = scale_rgb(current_color_, current_brightness_);
        int32_t rc = audio_->LedControl(out.r, out.g, out.b);
        if (rc != 0) {
            RCLCPP_WARN(get_logger(),
                "LedControl failed (rgb=%u,%u,%u) ret=%d", out.r, out.g, out.b, rc);
        } else {
            RCLCPP_DEBUG(get_logger(),
                "LED set (rgb=%u,%u,%u) @ %d%%", out.r, out.g, out.b, current_brightness_);
        }
    }

    void handle(const std::shared_ptr<g1_interface::srv::G1Modes::Request> req,
                std::shared_ptr<g1_interface::srv::G1Modes::Response> res) {
        try {
            const std::string raw = trim(req->request_data);
            if (raw.empty()) {
                res->success = false;
                res->reason  = "Empty request_data. Examples: 'color=#00ffaa', 'brightness=60', 'get_color'.";
                return;
            }

            if (iequals(raw, "get_brightness")) {
                res->success = true;
                res->reason  = "brightness=" + std::to_string(current_brightness_);
                return;
            }
            if (iequals(raw, "get_color")) {
                std::ostringstream oss;
                oss << "color=rgb("<<(int)current_color_.r<<","<<(int)current_color_.g<<","<<(int)current_color_.b<<")";
                res->success = true;
                res->reason  = oss.str();
                return;
            }

            std::vector<std::pair<std::string, std::string>> kvs;
            split_kv_pairs(raw, kvs);

            bool did_anything = false;
            std::ostringstream status;

            // brightness first (so color can override if both provided)
            for (auto &kv : kvs) {
                if (kv.first == "brightness") {
                    if (kv.second.empty()) { status << "[brightness missing value] "; continue; }
                    int v = std::stoi(kv.second);
                    v = std::max(0, std::min(100, v));
                    current_brightness_ = v;
                    apply_led();
                    status << "[brightness set to " << v << "] ";
                    did_anything = true;
                }
            }

            // color
            for (auto &kv : kvs) {
                if (kv.first == "color") {
                    if (kv.second.empty()) { status << "[color missing value] "; continue; }
                    RGB c;
                    if (!parse_color_string(kv.second, c)) {
                        status << "[color parse error] ";
                        continue;
                    }
                    current_color_ = c;
                    apply_led();
                    status << "[color set to rgb("<<(int)c.r<<","<<(int)c.g<<","<<(int)c.b<<")] ";
                    did_anything = true;
                }
            }

            if (!did_anything) {
                res->success = false;
                res->reason  = status.str().empty() ? "No recognized commands." : status.str();
            } else {
                res->success = true;
                res->reason  = status.str();
            }
        } catch (const std::exception &e) {
            res->success = false;
            res->reason  = e.what();
        }
    }

    std::string eth_;
    std::shared_ptr<unitree::robot::g1::AudioClient> audio_;
    rclcpp::Service<g1_interface::srv::G1Modes>::SharedPtr srv_;

    // local state (since SDK doesn’t expose LED getters)
    RGB current_color_;
    int current_brightness_;  // 0..100
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<G1LedServiceNode>());
    rclcpp::shutdown();
    return 0;
}
