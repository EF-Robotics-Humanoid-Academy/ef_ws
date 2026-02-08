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

#include "rclcpp/rclcpp.hpp"
#include "g1_interface/srv/g1_modes.hpp"

#include "unitree/robot/channel/channel_factory.hpp"
#include "unitree/robot/g1/audio/g1_audio_client.hpp"

namespace
{
    inline std::string trim(const std::string &s)
    {
        auto b = std::find_if_not(s.begin(), s.end(), ::isspace);
        auto e = std::find_if_not(s.rbegin(), s.rend(), ::isspace).base();
        return (b < e) ? std::string(b, e) : std::string();
    }

    inline void split_kv_pairs(const std::string &in, std::vector<std::pair<std::string, std::string>> &out)
    {
        std::stringstream ss(in);
        std::string token;
        while (std::getline(ss, token, ';'))
        {
            auto eq = token.find('=');
            std::string k = trim(eq == std::string::npos ? token : token.substr(0, eq));
            std::string v = trim(eq == std::string::npos ? "" : token.substr(eq + 1));
            // lowercase key
            std::transform(k.begin(), k.end(), k.begin(), [](unsigned char c)
                           { return std::tolower(c); });
            if (!k.empty())
                out.emplace_back(k, v);
        }
    }
} // namespace

class G1AudioServiceNode : public rclcpp::Node
{
public:
    G1AudioServiceNode() : Node("g1_audio_service")
    {
        declare_parameter<std::string>("eth", "none");
        eth_ = get_parameter("eth").as_string();
        RCLCPP_INFO(get_logger(), "Using Ethernet interface: %s", eth_.c_str());

        unitree::robot::ChannelFactory::Instance()->Init(0, eth_);
        audio_ = std::make_shared<unitree::robot::g1::AudioClient>();
        audio_->Init();
        audio_->SetTimeout(10.0f);

        srv_ = create_service<g1_interface::srv::G1Modes>(
            "hardware/audio",
            std::bind(&G1AudioServiceNode::handle, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(get_logger(), "Service ready: /hardware/audio (G1Modes)  "
                                  "Commands: volume=<0..100>; speak=<text>; get_volume");

        // Speak startup message once
        int32_t ret = audio_->TtsMaker("MY BOT SHOP ROS 2 SOFTWARE STARTED", 1); // 1 = English
        if (ret != 0)
        {
            RCLCPP_WARN(get_logger(), "Startup TTS failed, ret=%d", ret);
        }
    }

private:
    void handle(const std::shared_ptr<g1_interface::srv::G1Modes::Request> req,
                std::shared_ptr<g1_interface::srv::G1Modes::Response> res)
    {
        try
        {
            const std::string raw = trim(req->request_data);
            if (raw.empty())
            {
                res->success = false;
                res->reason = "Empty request_data. Use: 'volume=80;speak=Hello' or 'get_volume'.";
                return;
            }

            // Single-command shortcuts
            if (raw == "get_volume")
            {
                uint8_t vol = 0;
                int32_t r = audio_->GetVolume(vol);
                if (r == 0)
                {
                    res->success = true;
                    res->reason = "volume=" + std::to_string(vol);
                }
                else
                {
                    res->success = false;
                    res->reason = "GetVolume error, ret=" + std::to_string(r);
                }
                return;
            }

            // Parse key=value;key=value...
            std::vector<std::pair<std::string, std::string>> kvs;
            split_kv_pairs(raw, kvs);

            bool did_anything = false;
            std::ostringstream status;

            // 1) Volume (if present)
            for (auto &kv : kvs)
            {
                if (kv.first == "volume")
                {
                    if (kv.second.empty())
                    {
                        status << "[volume missing value] ";
                        continue;
                    }
                    int v = std::stoi(kv.second);
                    v = std::max(0, std::min(100, v));
                    int32_t r = audio_->SetVolume(static_cast<uint8_t>(v));
                    if (r == 0)
                    {
                        status << "[volume set to " << v << "] ";
                        did_anything = true;
                    }
                    else
                    {
                        status << "[SetVolume error ret=" << r << "] ";
                    }
                }
            }

            // 2) Speak (if present)
            for (auto &kv : kvs)
            {
                if (kv.first == "speak")
                {
                    if (kv.second.empty())
                    {
                        status << "[speak missing text] ";
                        continue;
                    }
                    // English voice index = 1 (per Unitree example)
                    int32_t r = audio_->TtsMaker(kv.second, 1);
                    if (r == 0)
                    {
                        status << "[speak ok] ";
                        did_anything = true;
                    }
                    else
                    {
                        status << "[TtsMaker error ret=" << r << "] ";
                    }
                }
            }

            if (!did_anything)
            {
                res->success = false;
                res->reason = status.str().empty() ? "No recognized commands." : status.str();
            }
            else
            {
                res->success = true;
                res->reason = status.str();
            }
        }
        catch (const std::exception &e)
        {
            res->success = false;
            res->reason = e.what();
        }
    }

    std::string eth_;
    std::shared_ptr<unitree::robot::g1::AudioClient> audio_;
    rclcpp::Service<g1_interface::srv::G1Modes>::SharedPtr srv_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<G1AudioServiceNode>());
    rclcpp::shutdown();
    return 0;
}
