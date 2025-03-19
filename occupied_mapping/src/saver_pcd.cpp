#include <memory>
#include <filesystem>

#include <rclcpp/rclcpp.hpp>
#include "rm_interface/srv/save_pcd.hpp"
#include <rcutils/error_handling.h> 
#include "debug_tools/debug_tools.h"

class PcdSaver : public rclcpp::Node{
private:
    bool flag_{false};
    rclcpp::Client<rm_interface::srv::SavePCD>::SharedPtr client_;

public:
    PcdSaver(std::string node_name, rclcpp::NodeOptions option): Node(node_name, option){
        std::string topic_save_cloud = this->declare_parameter<std::string>("topic_cloud_in", "/occupied_mapping_save_pcd");
        std::string path_file_out = this->declare_parameter<std::string>("pcd_out", "/home/ld/Documents/point_cloud/cloud_out_temp.pcd");

        std::filesystem::path file_path(path_file_out);
        if(std::filesystem::exists(file_path)) debug_tools::Debug().print("文件: ", path_file_out, " 已存在， 将会覆盖");
        if(!std::filesystem::exists(file_path.parent_path())){
            debug_tools::Debug().print("目标文件夹不存在, 中断");
            return;
        }

        client_ = this->create_client<rm_interface::srv::SavePCD>(topic_save_cloud);
        while(!client_->wait_for_service(std::chrono::seconds(2))){
            debug_tools::Debug().print("waiting for service to appear...");
            if(!rclcpp::ok()){
                debug_tools::Debug().print("client interrupted while waiting for service to appear.");
                rcutils_reset_error(); // 重置错误状态
                return;
            } 
        }

        auto request = std::make_shared<rm_interface::srv::SavePCD::Request>();
        request->file_path = path_file_out;

        auto result_future = client_->async_send_request(request);
        if(rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) != rclcpp::FutureReturnCode::SUCCESS){
            debug_tools::Debug().print("service call failed");
            client_->remove_pending_request(result_future);
            return;
        }

        auto result = result_future.get();
        debug_tools::Debug().print("已保存到:", path_file_out);
        debug_tools::Debug().print("memory GB", result->memory_gb);
        debug_tools::Debug().print("result", result->result);
    }
};


int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    {
        auto pcd_saver_node = std::make_shared<PcdSaver>("pcd_saver_node", rclcpp::NodeOptions());
        rclcpp::spin(pcd_saver_node);
    }
    rclcpp::shutdown();
    return 0;
}